//! Tuning of the velocity loops from their measured margins: the procedure the shipped gains were found with by hand
//! (`sim-cli margins` at every pose of the cycle, the gain raised until 35 degrees and 6 dB are the least left, the notch
//! put where a resonance holds the gain back), run by the program.

use crate::analysis::{LoopMargins, check_cancelled, plan_steps, step_done, velocity_loop_margins};
use crate::catalog::{motor_config, resolve};
use sim_core::{Config, ConfigError, MachineEntry};
use toml::Value;

/// What the tuning holds the loops to, and whether it may use the notch.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Targets {
    /// The least phase margin over all poses, degrees. 35 is what the shipped configs are held to.
    pub phase_margin_deg: f64,
    /// The least gain margin over all poses, dB. 6 as shipped.
    pub gain_margin_db: f64,
    /// Try the velocity loop's notch where a resonance holds the gain back.
    pub notch: bool,
}

impl Default for Targets {
    fn default() -> Self {
        Targets { phase_margin_deg: 35.0, gain_margin_db: 6.0, notch: true }
    }
}

/// Per motor the gain is searched over a bracket of `BRACKET` doublings or halvings from the config's own (together: a
/// start that fails is doubled once to see which way the slack grows), then by `BISECTIONS` bisections (in the
/// logarithm): at most 1 + 8 + 6 = 15 evaluations (`velocity_loop_margins`, which
/// measures every motor at every pose), the last resolving the gain to 1.1 %.
pub const BRACKET: usize = 8;
pub const BISECTIONS: usize = 6;
/// The joint check at the end lowers a gain that fails with all motors tuned by this factor, at most `JOINT_ROUNDS` times.
pub const JOINT_FACTOR: f64 = 0.8;
pub const JOINT_ROUNDS: usize = 4;
/// A notch stays only where the verified gain rises at least by this factor: one that buys less costs phase for nothing.
pub const NOTCH_MIN_GAIN: f64 = 1.15;
/// The notch's width as a share of its centre frequency, as the flying saw's (250 Hz at 360 Hz, hand-tuned).
pub const NOTCH_WIDTH: f64 = 0.7;
/// A gain margin limit counts as a resonance's where the sensitivity peaks at a mode of the machine or up to this factor
/// above it (the flying saw without its notch: the mode at 367 Hz, the peak at 475 to 520 Hz).
pub const PEAK_OVER_MODE: f64 = 1.5;

/// A velocity loop's gains: Kp (N m s/rad), Tn (s) and the position loop's Kv (1/s).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Gains {
    pub kp: f64,
    pub tn: f64,
    pub kv: f64,
}

/// The least margins of one motor's loop over the poses, with the pose (the run's name, "" the start pose).
#[derive(Debug, Clone, PartialEq)]
pub struct Least {
    /// NaN where no pose has a gain crossover in the measured band.
    pub phase_deg: f64,
    pub phase_pose: String,
    pub gain_db: f64,
    pub gain_pose: String,
    /// Every pose valid and `LoopMargins::stable`.
    pub stable: bool,
}

impl Least {
    fn of(runs: &[(String, Vec<LoopMargins>)], motor: usize) -> Least {
        let mut least = Least {
            phase_deg: f64::NAN,
            phase_pose: String::new(),
            gain_db: f64::INFINITY,
            gain_pose: String::new(),
            stable: true,
        };
        for (pose, margins) in runs {
            let m = &margins[motor];
            least.stable &= m.stable();
            if m.phase_margin_deg.is_nan() {
                continue;
            }
            if least.phase_deg.is_nan() || m.phase_margin_deg < least.phase_deg {
                (least.phase_deg, least.phase_pose) = (m.phase_margin_deg, pose.clone());
            }
        }
        for (pose, margins) in runs {
            let g = margins[motor].gain_margin_db;
            if g.is_nan() {
                least.gain_db = f64::NAN;
            } else if !least.gain_db.is_nan() && (g < least.gain_db || least.gain_pose.is_empty()) {
                (least.gain_db, least.gain_pose) = (g, pose.clone());
            }
        }
        least
    }

    /// Stable and both least margins at their targets (a loop without a crossover has no phase margin to miss).
    fn meets(&self, t: &Targets) -> bool {
        self.stable
            && (self.phase_deg.is_nan() || self.phase_deg >= t.phase_margin_deg)
            && self.gain_db >= t.gain_margin_db
    }

    /// How far from meeting the targets (negative) or beyond them: the smaller slack of the two, in degrees and dB.
    fn score(&self, t: &Targets) -> f64 {
        if !self.stable || self.gain_db.is_nan() {
            return -1.0e3;
        }
        let phase = if self.phase_deg.is_nan() { f64::INFINITY } else { self.phase_deg - t.phase_margin_deg };
        phase.min(self.gain_db - t.gain_margin_db)
    }
}

/// What came out for one motor.
#[derive(Debug, Clone, PartialEq)]
pub struct MotorTuning {
    pub id: String,
    /// The config keys that hold the gains (`drive.<id>.velocity_kp_nm_s_rad` where the config gives the motor its own,
    /// else the shared ones) and the position gain's, which is written only where it changed.
    pub kp_key: String,
    pub kv_key: String,
    pub before: Gains,
    pub after: Gains,
    pub before_least: Least,
    pub after_least: Least,
    /// The tuned gain met the targets at every pose, in the final measurement of the whole machine.
    pub met: bool,
    /// The joint check (all motors tuned) lowered this gain: the motors act on each other.
    pub lowered_in_joint_check: bool,
}

/// A notch the tuning kept.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Notch {
    /// The motor's index in `Tuned::motors`.
    pub motor: usize,
    pub frequency_hz: f64,
    pub bandwidth_hz: f64,
    /// The tuned gain with it over the tuned gain without it.
    pub gain_factor: f64,
}

/// The result of `velocity_loops`.
#[derive(Debug, Clone)]
pub struct Tuned {
    /// The config with the new gains (and the notch), as verified by the last `velocity_loop_margins` call.
    pub config: Config,
    pub motors: Vec<MotorTuning>,
    /// The notches kept, one per motor at most; the other motors then have none (frequency 0).
    pub notches: Vec<Notch>,
    /// Every motor meets the targets at every pose in that last measurement.
    pub met: bool,
    /// The calls of `velocity_loop_margins` made.
    pub evaluations: usize,
}

impl Tuned {
    /// The `path=value` words that make the new config out of the old one (for `--set`).
    pub fn settings(&self) -> Vec<String> {
        let mut out = Vec::new();
        for m in &self.motors {
            out.push(format!("{}={:?}", m.kp_key, m.after.kp));
            if m.after.kv != m.before.kv {
                out.push(format!("{}={:?}", m.kv_key, m.after.kv));
            }
        }
        if !self.notches.is_empty() {
            out.push("effects.notch_filter=true".into());
            for (k, m) in self.motors.iter().enumerate() {
                let key = |leaf: &str| m.kp_key.replace("velocity_kp_nm_s_rad", leaf);
                match self.notches.iter().find(|n| n.motor == k) {
                    Some(n) => {
                        out.push(format!("{}={:?}", key("notch_frequency_hz"), n.frequency_hz));
                        out.push(format!("{}={:?}", key("notch_bandwidth_hz"), n.bandwidth_hz));
                    }
                    None => out.push(format!("{}=0.0", key("notch_frequency_hz"))),
                }
            }
        }
        out
    }
}

/// Where a motor's drive keys live: the shared `[drive]`, or its own `[drive.<id>]`.
#[derive(Clone)]
enum Place {
    Shared,
    Own(String),
}

impl Place {
    fn path(&self, leaf: &str) -> String {
        match self {
            Place::Shared => format!("drive.{leaf}"),
            Place::Own(id) => format!("drive.{id}.{leaf}"),
        }
    }

    /// `value` at `leaf` of this place; a motor's own key is added where its table does not have it yet.
    fn put(&self, entry: &MachineEntry, cfg: &mut Config, leaf: &str, value: f64) -> Result<(), ConfigError> {
        crate::sim::set_config(entry, cfg, &self.path(leaf), Value::Float(value))
    }
}

/// The motor's gains as its drive sees them (`motor_config` of the resolved config).
fn gains_of(entry: &MachineEntry, cfg: &Config, id: &str) -> Result<Gains, ConfigError> {
    let own = motor_config(&resolve(entry, cfg)?, id)?;
    Ok(Gains {
        kp: own.positive("drive.velocity_kp_nm_s_rad")?,
        tn: own.positive("drive.velocity_tn_s")?,
        kv: own.positive("drive.kv_1_s")?,
    })
}

/// `cfg` with motor `place`'s velocity gain `kp`. Tn stays. Kv stays where the gain rises (the position loop's margin
/// Kp / J (Tn + 1 / Kv) only grows then) and is lowered where it falls, so that the product stays the config's own:
/// 1 / Kv' = (Tn + 1 / Kv) Kp / Kp' - Tn. (`catalog::retuned` lowers Kv to a floor of 1.25 or the config's own margin
/// by the same relation, with the inertia known; without it the config's own margin is the floor.)
fn with_gain(entry: &MachineEntry, cfg: &Config, place: &Place, base: Gains, kp: f64) -> Result<Config, ConfigError> {
    let mut out = cfg.clone();
    place.put(entry, &mut out, "velocity_kp_nm_s_rad", kp)?;
    // Written at either side: `cfg` may hold a Kv lowered for a smaller gain (the search without the notch).
    place.put(entry, &mut out, "kv_1_s", kv_for(base, kp))?;
    Ok(out)
}

fn kv_for(base: Gains, kp: f64) -> f64 {
    if kp < base.kp { 1.0 / ((base.tn + 1.0 / base.kv) * base.kp / kp - base.tn) } else { base.kv }
}

struct Probe {
    gain: f64,
    least: Least,
    /// The motor's margin at the pose with the least gain margin (its crossover and the frequency of the sensitivity's
    /// peak), for the notch, and that pose's index in `analysis::margin_poses`.
    worst: LoopMargins,
    worst_pose: usize,
}

struct Tuner<'a> {
    entry: &'a MachineEntry,
    targets: Targets,
    evaluations: usize,
}

impl Tuner<'_> {
    fn measure(&mut self, cfg: &Config) -> Result<Vec<(String, Vec<LoopMargins>)>, ConfigError> {
        check_cancelled();
        plan_steps(1);
        let runs = velocity_loop_margins(self.entry, cfg, &[])?;
        step_done();
        self.evaluations += 1;
        Ok(runs)
    }

    fn probe(&mut self, cfg: &Config, motor: usize, gain: f64) -> Result<Probe, ConfigError> {
        let runs = self.measure(cfg)?;
        let least = Least::of(&runs, motor);
        let (worst_pose, worst) = runs
            .iter()
            .map(|(_, m)| m[motor])
            .enumerate()
            .min_by(|a, b| a.1.gain_margin_db.total_cmp(&b.1.gain_margin_db))
            .ok_or_else(|| ConfigError::Invalid("margins".into()))?;
        Ok(Probe { gain, least, worst, worst_pose })
    }

    /// The largest gain of motor `motor` that meets the targets: bracket, then bisection. Returns the best probe (the
    /// largest that meets, or where none does the one with the most slack) and the lowest probe above it that fails.
    fn search(
        &mut self,
        cfg: &Config,
        motor: usize,
        place: &Place,
        base: Gains,
        start: f64,
    ) -> Result<(Probe, Option<Probe>), ConfigError> {
        let t = self.targets;
        let probe = |s: &mut Self, gain: f64| -> Result<Probe, ConfigError> {
            let candidate = with_gain(s.entry, cfg, place, base, gain)?;
            s.probe(&candidate, motor, gain)
        };
        // The better of two failing probes (the one with the most slack) is kept, for where none meets.
        let keep = |best: &mut Option<Probe>, p: Probe| {
            *best = Some(match best.take() {
                Some(b) if b.least.score(&t) >= p.least.score(&t) => b,
                _ => p,
            })
        };
        let (mut lo, mut hi, mut best_failing): (Option<Probe>, Option<Probe>, Option<Probe>) = (None, None, None);
        // The bracket's probes, doublings and halvings together.
        let mut left = BRACKET;
        let first = probe(self, start)?;
        if first.least.meets(&t) {
            lo = Some(first);
        } else {
            // A gain fails on either side of those that meet: above them on the gain margin, below them on the phase
            // margin (the integral's lag at a low crossover: the single axis at a quarter of its gain has 27 degrees).
            // The way to them is the side the slack grows to.
            let up = probe(self, 2.0 * start)?;
            left -= 1;
            if up.least.meets(&t) || up.least.score(&t) > first.least.score(&t) {
                keep(&mut best_failing, first);
                let mut current = up;
                while !current.least.meets(&t) && left > 0 {
                    let next = probe(self, 2.0 * current.gain)?;
                    left -= 1;
                    keep(&mut best_failing, current);
                    current = next;
                }
                if current.least.meets(&t) {
                    lo = Some(current);
                } else {
                    keep(&mut best_failing, current);
                }
            } else {
                keep(&mut best_failing, up);
                hi = Some(first);
                while left > 0 {
                    let p = probe(self, hi.as_ref().unwrap().gain / 2.0)?;
                    left -= 1;
                    if p.least.meets(&t) {
                        lo = Some(p);
                        break;
                    }
                    keep(&mut best_failing, hi.replace(p).unwrap());
                }
                if lo.is_none() {
                    keep(&mut best_failing, hi.take().unwrap());
                }
            }
        }
        // From a gain that meets, up to the first that fails, where that is not known yet.
        if hi.is_none() && lo.is_some() {
            while left > 0 {
                let p = probe(self, 2.0 * lo.as_ref().unwrap().gain)?;
                left -= 1;
                if p.least.meets(&t) {
                    lo = Some(p);
                } else {
                    hi = Some(p);
                    break;
                }
            }
        }
        let Some(mut lo) = lo else {
            // Nothing in the bracket meets the targets: the probe with the most slack.
            return Ok((best_failing.expect("every failing probe is kept"), None));
        };
        if let Some(mut high) = hi {
            for _ in 0..BISECTIONS {
                let p = probe(self, (lo.gain * high.gain).sqrt())?;
                if p.least.meets(&t) {
                    lo = p;
                } else {
                    high = p;
                }
            }
            hi = Some(high);
        }
        Ok((lo, hi))
    }
}

/// Tunes every motor's velocity loop: for each motor, in the order the machine lists them, the largest velocity gain
/// at which the least phase margin and the least gain margin over all poses of `margin_poses` meet `targets` and the
/// loop is `LoopMargins::stable` at every pose (the other motors as they are at that point). Tn stays and Kv follows as
/// `with_gain` says. Then, where `targets.notch` is set and the notch is off, for each motor in turn: if the gain margin
/// is what stops its gain, at a resonance (the sensitivity of the gain that fails peaks at a mode of the machine,
/// `Machine::modes` at that pose, or up to `PEAK_OVER_MODE` times above it), a notch is put on that mode (the highest
/// such) with `NOTCH_WIDTH` times its frequency as its width, the gain is searched again,
/// and the notch stays only where the gain is `NOTCH_MIN_GAIN` times higher. The switch is the whole machine's: where a
/// notch stays it is on, and every motor without one gets `notch_frequency_hz = 0` (no notch). A final call of
/// `velocity_loop_margins` on the whole tuned config verifies all motors; a motor that fails there is lowered by
/// `JOINT_FACTOR`, at most `JOINT_ROUNDS` times, and says so (`lowered_in_joint_check`). The gains returned are those
/// of the last verification; where no gain meets the targets `met` is false and the gains are the best found.
/// One search takes at most `BRACKET + BISECTIONS + 1` calls per motor (and as many for the notch).
pub fn velocity_loops(entry: &MachineEntry, cfg: &Config, targets: &Targets) -> Result<Tuned, ConfigError> {
    let mut tuner = Tuner { entry, targets: *targets, evaluations: 0 };
    let motors: Vec<String> = (entry.build)(cfg)?.motors().iter().map(|m| m.id.to_string()).collect();
    let before_runs = tuner.measure(cfg)?;
    let (mut config, mut gains, mut places) = (cfg.clone(), Vec::new(), Vec::new());
    for id in &motors {
        let base = gains_of(entry, cfg, id)?;
        let own = !id.is_empty() && cfg.get(&format!("drive.{id}.velocity_kp_nm_s_rad")).is_ok();
        // The shared key only where it is the gain the drive uses (a named part retunes it) and no other motor shares it.
        let shared = motors.len() == 1 && (cfg.f64("drive.velocity_kp_nm_s_rad")? - base.kp).abs() <= 1e-9 * base.kp;
        places.push(if own || !(id.is_empty() || shared) { Place::Own(id.clone()) } else { Place::Shared });
        gains.push(base);
    }
    let mut last_fail: Vec<Option<Probe>> = Vec::new();
    let mut found: Vec<f64> = Vec::new();
    let mut reached: Vec<bool> = Vec::new();
    for k in 0..motors.len() {
        let (best, fail) = tuner.search(&config, k, &places[k], gains[k], gains[k].kp)?;
        config = with_gain(entry, &config, &places[k], gains[k], best.gain)?;
        found.push(best.gain);
        reached.push(best.least.meets(&tuner.targets));
        last_fail.push(fail);
    }
    // The notch, motor by motor: tried against a config with the switch on and every motor's notch at 0 (none), which is
    // the config without it.
    let mut notches = Vec::new();
    if targets.notch && cfg.bool("effects.notch_filter").is_ok_and(|on| !on) {
        let mut none = config.clone();
        none.set_text("effects.notch_filter = true")?;
        for place in &places {
            place.put(entry, &mut none, "notch_frequency_hz", 0.0)?;
        }
        for k in 0..motors.len() {
            let Some(fail) = &last_fail[k] else { continue };
            let (t, w) = (&tuner.targets, &fail.worst);
            let at_gain = fail.least.stable && fail.least.gain_db < t.gain_margin_db;
            if !(at_gain && w.peak_frequency_hz.is_finite()) {
                continue;
            }
            // The notch goes on the machine's mode that makes the peak: the highest at or below it at that pose, within
            // `PEAK_OVER_MODE`. On the peak itself it lags the loop where it already turns (the flying saw: a notch at
            // its peak of 520 Hz left 2.8 dB at a gain that kept 6 without it; on its mode at 367 Hz, 7.3 dB at 1.5 times
            // that gain).
            let resolved = resolve(entry, &config)?;
            let pose = &crate::analysis::margin_poses(&resolved)?[fail.worst_pose];
            let mode = (entry.build)(&resolved)?
                .modes(pose)
                .iter()
                .map(|m| m.hz)
                .filter(|hz| *hz <= w.peak_frequency_hz && w.peak_frequency_hz <= PEAK_OVER_MODE * hz)
                .fold(f64::NAN, f64::max);
            if !mode.is_finite() {
                continue;
            }
            let (f, bw) = (mode, NOTCH_WIDTH * mode);
            let mut trial = none.clone();
            places[k].put(entry, &mut trial, "notch_frequency_hz", f)?;
            places[k].put(entry, &mut trial, "notch_bandwidth_hz", bw)?;
            let (best, _) = tuner.search(&trial, k, &places[k], gains[k], found[k])?;
            if best.least.meets(&tuner.targets) && best.gain >= NOTCH_MIN_GAIN * found[k] {
                notches.push(Notch { motor: k, frequency_hz: f, bandwidth_hz: bw, gain_factor: best.gain / found[k] });
                none = with_gain(entry, &trial, &places[k], gains[k], best.gain)?;
                found[k] = best.gain;
            }
        }
        if !notches.is_empty() {
            config = none;
        }
    }
    // The joint check, on the whole tuned config.
    let mut lowered = vec![false; motors.len()];
    let mut runs = tuner.measure(&config)?;
    for _ in 0..JOINT_ROUNDS {
        let failing: Vec<usize> =
            (0..motors.len()).filter(|&k| reached[k] && !Least::of(&runs, k).meets(&tuner.targets)).collect();
        if failing.is_empty() {
            break;
        }
        for k in failing {
            found[k] *= JOINT_FACTOR;
            lowered[k] = true;
            config = with_gain(entry, &config, &places[k], gains[k], found[k])?;
        }
        runs = tuner.measure(&config)?;
    }
    let mut out = Vec::new();
    for (k, id) in motors.iter().enumerate() {
        let after = gains_of(entry, &config, id)?;
        let least = Least::of(&runs, k);
        out.push(MotorTuning {
            id: id.clone(),
            kp_key: places[k].path("velocity_kp_nm_s_rad"),
            kv_key: places[k].path("kv_1_s"),
            before: gains[k],
            after,
            before_least: Least::of(&before_runs, k),
            met: least.meets(&tuner.targets),
            after_least: least,
            lowered_in_joint_check: lowered[k],
        });
    }
    let met = out.iter().all(|m| m.met);
    Ok(Tuned { config, motors: out, notches, met, evaluations: tuner.evaluations })
}
