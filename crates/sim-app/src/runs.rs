//! Runs kept side by side for comparing them: the live session's scopes, the ghost run beside it, an analysed cycle,
//! a path that was run, a replayed trace, and the cycle of another config or machine. Each holds every signal it has,
//! sampled once per millisecond, with the time counted from its start. Runs of different machines are compared by
//! quantity (`Channel`): a motor's torque, speed and following error by the motor's place in the machine's list, and
//! the tool's position error. The views draw what this module gives them; it draws nothing itself.

use sim_components::analysis::Verdict;
use sim_core::{AxisInfo, MotorInfo, SignalInfo};
use std::borrow::Cow;

pub use sim_components::report::peak_keeping;

/// At most this many stretches a drawn line is cut into (`peak_keeping`): two points each, some six thousand a line.
pub const PLOT_BUCKETS: usize = 3000;

/// The points of a line for a plot: every point where there are few, else the lowest and highest of each of
/// `PLOT_BUCKETS` stretches, so that a peak one sample wide still shows.
pub fn plot_line(n: usize, point: impl Fn(usize) -> [f64; 2]) -> Vec<[f64; 2]> {
    peak_keeping(n, point, PLOT_BUCKETS)
}

/// The samples of a session as one flat ring buffer: a row of every signal per sample, the newest `capacity` kept.
/// One allocation of `capacity` rows in all, where a `Vec` per row cost a heap block per sample.
#[derive(Debug, Clone, Default)]
pub struct History {
    data: Vec<f64>,
    width: usize,
    capacity: usize,
    /// The row of the oldest sample, once the buffer is full.
    head: usize,
    len: usize,
    /// Samples pushed since the start, dropped ones too: changes with every sample, for caches.
    pushed: u64,
}

impl History {
    pub fn new(capacity: usize) -> Self {
        Self { capacity: capacity.max(1), ..Self::default() }
    }

    /// Adds the newest sample; past `capacity` the oldest goes. A row of another width than the ones before (another
    /// machine) starts the buffer anew.
    pub fn push(&mut self, row: &[f64]) {
        if row.len() != self.width {
            *self = Self { pushed: self.pushed, ..Self::new(self.capacity) };
            self.width = row.len();
        }
        if self.len < self.capacity {
            self.data.extend_from_slice(row);
            self.len += 1;
        } else {
            let w = self.width;
            self.data[self.head * w..][..w].copy_from_slice(row);
            self.head = (self.head + 1) % self.capacity;
        }
        self.pushed += 1;
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn pushed(&self) -> u64 {
        self.pushed
    }

    /// The `i`-th sample, oldest first.
    pub fn get(&self, i: usize) -> Option<&[f64]> {
        (i < self.len).then(|| &self.data[(self.head + i) % self.capacity * self.width..][..self.width])
    }

    pub fn back(&self) -> Option<&[f64]> {
        self.len.checked_sub(1).and_then(|i| self.get(i))
    }

    pub fn iter(&self) -> impl DoubleEndedIterator<Item = &[f64]> + ExactSizeIterator + Clone {
        (0..self.len).map(|i| &self.data[(self.head + i) % self.capacity * self.width..][..self.width])
    }

    /// The first sample whose time (column 0) is at or after `time`.
    pub fn first_at(&self, time: f64) -> usize {
        let (mut low, mut high) = (0, self.len);
        while low < high {
            let mid = (low + high) / 2;
            if self.data[(self.head + mid) % self.capacity * self.width] < time {
                low = mid + 1;
            } else {
                high = mid;
            }
        }
        low
    }

    /// A copy of the samples from `from` on, as a buffer of their own.
    pub fn tail(&self, from: usize) -> Self {
        let mut out = Self::new((self.len - from.min(self.len)).max(1));
        (from..self.len).filter_map(|i| self.get(i)).for_each(|row| out.push(row));
        out
    }
}

/// Where a run came from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RunKind {
    /// The scopes' time window of the live session.
    Live,
    /// The window of the run beside it with one effect switched (the 3D view's ghost).
    Ghost,
    /// The configured cycle, as the analysis measures it (its second pass).
    Cycle,
    /// A path that was planned and run.
    Path,
    /// A trace from a file, or its replay.
    Trace,
}

impl RunKind {
    /// Its label's translation key.
    pub fn key(self) -> &'static str {
        match self {
            Self::Live => "ui-run-live",
            Self::Ghost => "ui-run-ghost",
            Self::Cycle => "ui-run-cycle",
            Self::Path => "ui-run-path",
            Self::Trace => "ui-run-trace",
        }
    }
}

/// One run: every signal it has, once per millisecond, the time from its start.
#[derive(Debug, Clone)]
pub struct Run {
    /// What the user reads: what was changed against the config shown, or the file.
    pub label: String,
    pub kind: RunKind,
    /// The machine's id.
    pub machine: &'static str,
    pub signals: Vec<SignalInfo>,
    /// The motors' ids, in the machine's order: how `Channel::motor` finds a motor's signals.
    pub motors: Vec<String>,
    /// Per motor its drive's rated (S1) and peak torque in N m, for the utilisation; empty where not known (a trace).
    pub ratings: Vec<(f64, f64)>,
    /// Per signal its values, the first the time in s from the run's start.
    pub columns: Vec<Vec<f64>>,
    /// The checks, for a run that was judged (a cycle or a path).
    pub verdicts: Option<Vec<Verdict>>,
}

impl Run {
    /// A run from rows of samples as the simulation gives them (`Sim::sample`), the time counted from the first.
    pub fn from_rows<'a>(
        label: String,
        kind: RunKind,
        machine: &'static str,
        signals: Vec<SignalInfo>,
        motors: &[MotorInfo],
        rows: impl IntoIterator<Item = &'a [f64]>,
    ) -> Self {
        let mut columns = vec![Vec::new(); signals.len()];
        for row in rows {
            columns.iter_mut().zip(row).for_each(|(column, v)| column.push(*v));
        }
        if let Some(&start) = columns.first().and_then(|time| time.first()) {
            columns[0].iter_mut().for_each(|t| *t -= start);
        }
        let motors = motors.iter().map(|m| m.id.to_string()).collect();
        Self { label, kind, machine, signals, motors, ratings: Vec::new(), columns, verdicts: None }
    }

    /// A trace (measured, or its replay) as a run: per motor its angle, speed, torque and following error.
    pub fn from_trace(
        label: String,
        machine: &'static str,
        motors: &[MotorInfo],
        trace: &sim_components::trace::Trace,
    ) -> Self {
        let mut signals = vec![SignalInfo::new("time", "group-sim", "quantity-time", "s")];
        let mut columns = vec![trace.time.iter().map(|t| t - trace.time.first().unwrap_or(&0.0)).collect()];
        for (motor, m) in motors.iter().zip(&trace.motors) {
            let named = |name: &str, unit: &str| {
                let quantity = format!("quantity-{}", name.replace('_', "-"));
                SignalInfo::new(format!("motor.{}.{name}", motor.id), motor.key, &quantity, unit)
            };
            signals.extend([
                named("angle", "rad"),
                named("speed", "rad/s"),
                named("torque", "N m"),
                named("following_error", "rad"),
            ]);
            columns.extend([m.angle.clone(), m.speed.clone(), m.torque.clone(), m.following_error()]);
        }
        let motors = motors.iter().map(|m| m.id.to_string()).collect();
        Self { label, kind: RunKind::Trace, machine, signals, motors, ratings: Vec::new(), columns, verdicts: None }
    }

    pub fn time(&self) -> &[f64] {
        &self.columns[0]
    }

    /// The signal of this id, if the run has it.
    pub fn column(&self, id: &str) -> Option<&[f64]> {
        Some(&self.columns[self.signals.iter().position(|s| s.id == id)?])
    }

    /// The values of one quantity over the run, if the run has it: a motor's own signal, or the tool's position
    /// error, the length of the axes' following errors over the axes in metres (`Channel`).
    pub fn channel(&self, channel: Channel) -> Option<Cow<'_, [f64]>> {
        match channel.quantity.signal() {
            Some(name) => self.column(&format!("motor.{}.{name}", self.motors.get(channel.motor)?)).map(Cow::Borrowed),
            None => {
                let errors: Vec<&[f64]> = self
                    .signals
                    .iter()
                    .zip(&self.columns)
                    .filter(|(s, _)| s.id.starts_with("axis.") && s.id.ends_with(".following_error") && s.unit == "m")
                    .map(|(_, c)| c.as_slice())
                    .collect();
                (!errors.is_empty()).then(|| {
                    Cow::Owned(
                        (0..self.time().len())
                            .map(|i| errors.iter().map(|e| e[i] * e[i]).sum::<f64>().sqrt())
                            .collect(),
                    )
                })
            }
        }
    }

    /// The channels this run has, motor after motor in the order of `Quantity::ALL`, then the tool's error.
    pub fn channels(&self) -> Vec<Channel> {
        let mut out: Vec<Channel> = (0..self.motors.len())
            .flat_map(|motor| Quantity::ALL[..3].iter().map(move |&quantity| Channel { quantity, motor }))
            .filter(|c| self.channel(*c).is_some())
            .collect();
        let tool = Channel { quantity: Quantity::ToolError, motor: 0 };
        out.extend(self.channel(tool).map(|_| tool));
        out
    }

    /// The key figures (see `Figures`).
    pub fn figures(&self) -> Figures {
        let time = self.time();
        let duration = time.last().copied().unwrap_or(0.0);
        let peak = |v: &[f64]| v.iter().fold(0f64, |p, x| p.max(x.abs()));
        let rms =
            |v: &[f64]| if v.is_empty() { 0.0 } else { (v.iter().map(|x| x * x).sum::<f64>() / v.len() as f64).sqrt() };
        let of = |quantity: Quantity, motor: usize| self.channel(Channel { quantity, motor });
        let motors = (0..self.motors.len())
            .map(|k| {
                let torque = of(Quantity::Torque, k).unwrap_or_default();
                let error = of(Quantity::FollowingError, k).unwrap_or_default();
                let speed = of(Quantity::Speed, k).unwrap_or_default();
                let (peak_torque, rms_torque) = (peak(&torque), rms(&torque));
                let utilisation = self.ratings.get(k).map(|&(rated, top)| (rms_torque / rated, peak_torque / top));
                // The electrical energy the motor drew, net of what it gave back, by the trapezoid rule.
                let power = self.column(&format!("motor.{}.electrical_power", self.motors[k]));
                let energy = power
                    .map(|p| time.windows(2).zip(p.windows(2)).map(|(t, p)| (t[1] - t[0]) * (p[0] + p[1]) / 2.0).sum());
                MotorFigures {
                    peak_torque,
                    rms_torque,
                    peak_speed: peak(&speed),
                    peak_error: peak(&error),
                    rms_error: rms(&error),
                    utilisation,
                    energy,
                }
            })
            .collect();
        let tool = self.channel(Channel { quantity: Quantity::ToolError, motor: 0 });
        let verdicts =
            self.verdicts.as_ref().map(|v| (v.iter().filter(|v| v.ok).count(), v.iter().filter(|v| !v.ok).count()));
        Figures {
            duration,
            motors,
            tool_error_peak: tool.as_deref().map(peak),
            tool_error_rms: tool.as_deref().map(rms),
            verdicts,
        }
    }
}

/// What a motor did over a run.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotorFigures {
    /// In N m.
    pub peak_torque: f64,
    pub rms_torque: f64,
    /// In rad/s.
    pub peak_speed: f64,
    /// The motor's following error, in rad.
    pub peak_error: f64,
    pub rms_error: f64,
    /// RMS torque over the rated torque and peak torque over the drive's peak, where the drive is known.
    pub utilisation: Option<(f64, f64)>,
    /// Electrical energy drawn net in J, where the run has the power.
    pub energy: Option<f64>,
}

/// A run's key figures for the compare panel.
#[derive(Debug, Clone, PartialEq)]
pub struct Figures {
    pub duration: f64,
    pub motors: Vec<MotorFigures>,
    /// The tool's position error (`Quantity::ToolError`) in m, where the run has it.
    pub tool_error_peak: Option<f64>,
    pub tool_error_rms: Option<f64>,
    /// Checks passed and failed, for a run that was judged.
    pub verdicts: Option<(usize, usize)>,
}

/// A quantity compared across runs.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Quantity {
    Torque,
    Speed,
    FollowingError,
    /// The tool's position error: the length of the axes' following errors (those in metres).
    ToolError,
}

impl Quantity {
    pub const ALL: [Self; 4] = [Self::Torque, Self::Speed, Self::FollowingError, Self::ToolError];

    /// The motor signal's name after `motor.<id>.`; `None` for the tool's error, which is worked out.
    pub fn signal(self) -> Option<&'static str> {
        match self {
            Self::Torque => Some("torque"),
            Self::Speed => Some("speed"),
            Self::FollowingError => Some("following_error"),
            Self::ToolError => None,
        }
    }

    pub fn unit(self) -> &'static str {
        match self {
            Self::Torque => "N m",
            Self::Speed => "rad/s",
            Self::FollowingError => "rad",
            Self::ToolError => "m",
        }
    }

    /// Its label's translation key.
    pub fn key(self) -> &'static str {
        match self {
            Self::Torque => "quantity-torque",
            Self::Speed => "quantity-speed",
            Self::FollowingError => "quantity-following-error",
            Self::ToolError => "ui-run-tool-error",
        }
    }
}

/// One quantity of one motor (the motor's place in its machine's list; ignored for the tool's error). Two runs are
/// compared on the channels both have: of two machines with two and four motors, motors 0 and 1.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Channel {
    pub quantity: Quantity,
    pub motor: usize,
}

/// One channel of two runs on the first's time: the second's values linearly between its samples, and the first less
/// the second. Over the time both have; where one is longer, its rest is not compared.
#[derive(Debug, Clone, PartialEq)]
pub struct Aligned {
    pub time: Vec<f64>,
    pub a: Vec<f64>,
    pub b: Vec<f64>,
    pub difference: Vec<f64>,
}

impl Aligned {
    /// Peak and RMS of the difference.
    pub fn difference_peak_rms(&self) -> (f64, f64) {
        let d = &self.difference;
        let peak = d.iter().fold(0f64, |p, x| p.max(x.abs()));
        (peak, if d.is_empty() { 0.0 } else { (d.iter().map(|x| x * x).sum::<f64>() / d.len() as f64).sqrt() })
    }
}

/// `a` and `b` on `a`'s time, both counted from their start (see `Aligned`); `None` where either lacks the channel.
pub fn align(a: &Run, b: &Run, channel: Channel) -> Option<Aligned> {
    let (va, vb) = (a.channel(channel)?, b.channel(channel)?);
    let (ta, tb) = (a.time(), b.time());
    let end = *tb.last()?;
    let mut out = Aligned { time: Vec::new(), a: Vec::new(), b: Vec::new(), difference: Vec::new() };
    let mut j = 0;
    for (&t, &x) in ta.iter().zip(va.iter()).take_while(|(t, _)| **t <= end) {
        while j + 1 < tb.len() && tb[j + 1] < t {
            j += 1;
        }
        let y = match tb.get(j + 1) {
            Some(&t1) if t1 > tb[j] => vb[j] + (vb[j + 1] - vb[j]) * ((t - tb[j]) / (t1 - tb[j])).clamp(0.0, 1.0),
            _ => vb[j],
        };
        out.time.push(t);
        out.a.push(x);
        out.b.push(y);
        out.difference.push(x - y);
    }
    Some(out)
}

/// The channels two runs share: the alignment of `Channel`.
pub fn shared_channels(a: &Run, b: &Run) -> Vec<Channel> {
    let theirs = b.channels();
    a.channels().into_iter().filter(|c| theirs.contains(c)).collect()
}

/// How many runs the store keeps; the oldest goes for a newer one.
pub const MAX_RUNS: usize = 8;

/// The runs kept for comparing, oldest first, and which of them is the reference the others are compared with.
#[derive(Debug, Clone, Default)]
pub struct RunStore {
    runs: Vec<(Run, bool)>,
    reference: usize,
}

impl RunStore {
    /// Keeps a run, shown; past `MAX_RUNS` the oldest goes (the reference stays the same run where it can). A run of
    /// the same kind, machine and label (the same config run again) takes the place of the one before.
    pub fn add(&mut self, run: Run) {
        let same = |kept: &Run| kept.label == run.label && kept.kind == run.kind && kept.machine == run.machine;
        if let Some(kept) = self.runs.iter_mut().find(|(kept, _)| same(kept)) {
            kept.0 = run;
            return;
        }
        if self.runs.len() == MAX_RUNS {
            self.remove(0);
        }
        self.runs.push((run, true));
    }

    pub fn remove(&mut self, i: usize) {
        if i < self.runs.len() {
            self.runs.remove(i);
            if self.reference > i || self.reference >= self.runs.len() {
                self.reference = self.reference.saturating_sub(1);
            }
        }
    }

    pub fn clear(&mut self) {
        *self = Self::default();
    }

    pub fn len(&self) -> usize {
        self.runs.len()
    }

    pub fn is_empty(&self) -> bool {
        self.runs.is_empty()
    }

    pub fn get(&self, i: usize) -> Option<&Run> {
        self.runs.get(i).map(|(run, _)| run)
    }

    pub fn iter(&self) -> impl Iterator<Item = &Run> {
        self.runs.iter().map(|(run, _)| run)
    }

    /// Whether run `i` is drawn.
    pub fn shown(&self, i: usize) -> bool {
        self.runs.get(i).is_some_and(|(_, shown)| *shown)
    }

    pub fn set_shown(&mut self, i: usize, shown: bool) {
        if let Some(run) = self.runs.get_mut(i) {
            run.1 = shown;
        }
    }

    pub fn reference(&self) -> usize {
        self.reference
    }

    pub fn set_reference(&mut self, i: usize) {
        if i < self.runs.len() {
            self.reference = i;
        }
    }

    /// The run `i` against the reference on one channel (`align`): `None` for the reference itself and where either
    /// lacks the channel.
    pub fn against_reference(&self, i: usize, channel: Channel) -> Option<Aligned> {
        (i != self.reference).then(|| align(self.get(i)?, self.get(self.reference)?, channel)).flatten()
    }
}

/// The colour slot of a signal's owner, the same in every plot: an axis by its place in the machine's axes, a motor by
/// its place in its motors (as plots of one value per motor colour them), anything else after those by its place
/// among the other owners of `signals`. So motor 2 has one colour in the scopes, the torque-speed plot, the sweep and
/// the run comparison.
pub fn owner_slot(owner_key: &str, signals: &[SignalInfo], axes: &[AxisInfo], motors: &[MotorInfo]) -> usize {
    if let Some(k) = motors.iter().position(|m| m.key == owner_key) {
        return k;
    }
    if let Some(i) = axes.iter().position(|a| a.key == owner_key) {
        return i;
    }
    let mut others: Vec<&str> = Vec::new();
    for s in signals {
        let own = s.owner_key.as_str();
        if !others.contains(&own) && !motors.iter().any(|m| m.key == own) && !axes.iter().any(|a| a.key == own) {
            others.push(own);
        }
    }
    axes.len().max(motors.len()) + others.iter().position(|o| *o == owner_key).unwrap_or(0)
}

/// The units the plots and tables show speeds and lengths in: rpm in place of rad/s, mm in place of m. Off, a
/// value keeps its SI unit, which the scopes scale by an SI prefix to a readable size (`display_scale`).
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Units {
    pub rpm: bool,
    pub mm: bool,
}

impl Units {
    /// The factor a value in `unit` is shown multiplied by, and the unit shown; `None` where the choice leaves it.
    pub fn of(self, unit: &str) -> Option<(f64, String)> {
        let rpm = 60.0 / std::f64::consts::TAU;
        match unit {
            "rad/s" if self.rpm => Some((rpm, "rpm".into())),
            "m" | "m/s" | "m/s²" if self.mm => Some((1e3, format!("m{unit}"))),
            _ => None,
        }
    }

    /// `of`, or the value as it is.
    pub fn scale(self, unit: &str) -> (f64, String) {
        self.of(unit).unwrap_or((1.0, unit.to_string()))
    }
}

/// The travelled path's points up to `time` (all of them for `None`, live): what the 3D view draws while the timeline
/// shows an older scene. The points are in time order.
pub fn trail_until(points: &[crate::session::TrailPoint], time: Option<f64>) -> &[crate::session::TrailPoint] {
    match time {
        Some(time) => &points[..points.partition_point(|p| p.time <= time)],
        None => points,
    }
}
