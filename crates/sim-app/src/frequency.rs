//! The loop-response tab's measurement: per motor the open-loop response of its velocity loop as `sim-cli margins`
//! measures it (`analysis::velocity_loop_responses`, refined where it changes fast), its margins, the machine's
//! resonances and the input shaper's and the notch filter's frequencies, and what the plots draw of it (closed loop,
//! phases, the phase crossover). It runs in a thread of its own, with progress and a cancel switch, as the app's other
//! long runs do. No text for the user is written here.

use crate::model::Message;
use sim_components::analysis::{CANCELLED, LoopMargins, LoopPoint, Progress, loop_margins, velocity_loop_responses};
use sim_core::{Config, ConfigError, MachineEntry};
use std::sync::atomic::Ordering::Relaxed;
use std::sync::mpsc::{Receiver, TryRecvError, channel};
use std::sync::{Arc, Mutex};

/// What the measurement gives the tab.
#[derive(Debug, Clone)]
pub struct Response {
    /// Per motor its points, in frequency order, and their margins.
    pub curves: Vec<Vec<LoopPoint>>,
    pub margins: Vec<LoopMargins>,
    /// The machine's resonances at the start pose, Hz (`Machine::modes`).
    pub modes: Vec<f64>,
    /// The input shaper's and the notch filter's frequencies, where they are on: (translation key, Hz).
    pub markers: Vec<(&'static str, f64)>,
}

type Answer = Result<(Config, Response), Message>;

/// The tab's state: the last result with the config it was measured for, the motor shown, and the job under way.
#[derive(Default)]
pub struct LoopResponse {
    pub result: Option<(Config, Response)>,
    pub motor: usize,
    job: Option<(Arc<Progress>, Mutex<Receiver<Answer>>)>,
}

impl LoopResponse {
    /// Starts the measurement with `config` as it is now; one under way is cancelled.
    pub fn start(&mut self, entry: &'static MachineEntry, config: &Config) {
        self.cancel();
        let (progress, config, (send, receive)) = (Arc::new(Progress::default()), config.clone(), channel());
        let watch = progress.clone();
        std::thread::spawn(move || {
            let answer = measure(entry, &config, &watch).map_err(|e| (e.key(), e.detail().to_string()));
            let _ = send.send(answer.map(|response| (config, response)));
        });
        self.job = Some((progress, Mutex::new(receive)));
    }

    /// Stops the measurement under way: its thread ends after the points it is measuring, and nothing of it arrives.
    pub fn cancel(&mut self) {
        if let Some((progress, _)) = self.job.take() {
            progress.cancel.store(true, Relaxed);
        }
    }

    pub fn running(&self) -> bool {
        self.job.is_some()
    }

    /// Measurements done and planned so far (the refinement adds to them).
    pub fn progress(&self) -> Option<(usize, usize)> {
        self.job.as_ref().map(|(p, _)| (p.done.load(Relaxed), p.total.load(Relaxed)))
    }

    /// Takes in the answer, if there is one; the message of a measurement that failed. A thread gone without an
    /// answer (it panicked) answers with a message.
    pub fn poll(&mut self) -> Option<Message> {
        let answer = match self.job.as_ref()?.1.lock().map(|job| job.try_recv()) {
            Ok(Err(TryRecvError::Empty)) => return None,
            Ok(Ok(answer)) => answer,
            _ => Err(("ui-job-ended", String::new())),
        };
        self.job = None;
        match answer {
            Ok(result) => {
                self.motor = self.motor.min(result.1.curves.len().saturating_sub(1));
                self.result = Some(result);
                None
            }
            Err(message) => Some(message),
        }
    }
}

impl Drop for LoopResponse {
    /// A machine switched or a window closed stops the measurement rather than leave it running for nothing.
    fn drop(&mut self) {
        self.cancel();
    }
}

/// The measurement: the responses of the config's motors (`velocity_loop_responses`), their margins, and what the
/// plots mark. A cancelled one answers `ConfigError::Invalid(CANCELLED)`.
pub fn measure(entry: &MachineEntry, config: &Config, progress: &Progress) -> Result<Response, ConfigError> {
    let cfg = &sim_components::catalog::resolve(entry, config)?;
    let curves = velocity_loop_responses(entry, std::slice::from_ref(cfg), progress)?.remove(0);
    let modes = (entry.build)(cfg)?.modes(&cfg.f64s("start.pose")?).iter().map(|m| m.hz).collect();
    Ok(Response { margins: curves.iter().map(|c| loop_margins(c)).collect(), curves, modes, markers: markers(cfg) })
}

/// Whether an error is the answer of a cancelled measurement.
pub fn cancelled(e: &ConfigError) -> bool {
    matches!(e, ConfigError::Invalid(key) if key == CANCELLED)
}

/// The frequencies the config filters or shapes at: the input shaper's (the first, the second and each of
/// `profile.shaper_modes`) with `effects.input_shaping` on, the velocity loop's notch filter with
/// `effects.notch_filter` on.
pub fn markers(cfg: &Config) -> Vec<(&'static str, f64)> {
    let mut out = Vec::new();
    if cfg.bool("effects.input_shaping").unwrap_or(false) {
        for path in ["profile.shaper_frequency_hz", "profile.shaper2_frequency_hz"] {
            out.extend(cfg.f64(path).ok().filter(|hz| *hz > 0.0).map(|hz| ("ui-loop-shaper", hz)));
        }
        let hz =
            |m: &toml::Value| m.get("frequency_hz").and_then(|v| v.as_float().or(v.as_integer().map(|i| i as f64)));
        out.extend(
            cfg.array("profile.shaper_modes").into_iter().flatten().filter_map(hz).map(|hz| ("ui-loop-shaper", hz)),
        );
    }
    if cfg.bool("effects.notch_filter").unwrap_or(false) {
        out.extend(cfg.f64("drive.notch_frequency_hz").ok().map(|hz| ("ui-loop-notch", hz)));
    }
    out
}

/// The closed loop T = L / (1 + L) of a point, (real, imaginary part).
pub fn closed_loop(p: &LoopPoint) -> [f64; 2] {
    let (dr, di) = (1.0 + p.re, p.im);
    let d2 = dr * dr + di * di;
    [(p.re * dr + p.im * di) / d2, (p.im * dr - p.re * di) / d2]
}

/// The phase `raw` (degrees) turned by whole turns to lie within half a turn of the previous one, `last`: unwrapped in
/// both directions.
pub fn unwrap_near(raw: f64, last: f64) -> f64 {
    raw + 360.0 * ((last - raw) / 360.0).round()
}

/// Phases in degrees of complex values in frequency order, unwrapped along the frequency axis and turned by whole
/// turns so that the first lies in (-270, 90], as `loop_margins` takes the loop's phase. A value that is not a number
/// gives NaN and the next one is unwrapped from the last that was.
pub fn phases(values: impl IntoIterator<Item = [f64; 2]>) -> Vec<f64> {
    let (mut out, mut last, mut shift) = (Vec::new(), None::<f64>, None);
    for [re, im] in values {
        let raw = im.atan2(re).to_degrees();
        if raw.is_nan() {
            out.push(f64::NAN);
            continue;
        }
        let unwrapped = last.map_or(raw, |last| unwrap_near(raw, last));
        let shift = *shift.get_or_insert(-360.0 * ((unwrapped - 90.0) / 360.0).ceil());
        last = Some(unwrapped);
        out.push(unwrapped + shift);
    }
    out
}

/// Where the gain margin is read: the crossing `loop_margins` takes. The phase (`phases`) starts from 0 at the first
/// point's gain; of its crossings of odd multiples of -180 degrees only falling ones count (interpolated on the
/// logarithmic frequency axis, linearly in the phase), and a rising one at |L| > 1 cancels the latest falling one at
/// |L| > 1 (Nyquist). Of those left the one with the largest |L|; `None` without one.
pub fn phase_crossover(points: &[LoopPoint]) -> Option<f64> {
    let phase = phases(points.iter().map(|p| [p.re, p.im]));
    let gain = |p: &LoopPoint| p.re.hypot(p.im).ln();
    let first = points.first()?;
    let samples: Vec<(f64, f64, f64)> = std::iter::once((0.0, gain(first), first.frequency))
        .chain(points.iter().zip(&phase).map(|(p, ph)| (*ph, gain(p), p.frequency)))
        .collect();
    let (mut open, mut worst): (Vec<(f64, f64)>, Option<(f64, f64)>) = (Vec::new(), None);
    for w in samples.windows(2) {
        let ((pa, ga, fa), (pb, gb, fb)) = (w[0], w[1]);
        let at = |level: f64| {
            let share = (pa - level) / (pa - pb);
            (ga + (gb - ga) * share, fa * (fb / fa).powf(share))
        };
        let mut level = -180.0 + 360.0 * ((pa + 180.0) / 360.0).floor();
        while pb < level && level <= pa {
            let (g, f) = at(level);
            if g > 0.0 {
                open.push((g, f));
            } else if worst.is_none_or(|(w, _)| g > w) {
                worst = Some((g, f));
            }
            level -= 360.0;
        }
        level += 360.0;
        while pa < level && level <= pb {
            if at(level).0 > 0.0 {
                open.pop();
            }
            level += 360.0;
        }
    }
    open.into_iter().chain(worst).max_by(|a, b| a.0.total_cmp(&b.0)).map(|(_, f)| f)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn point(frequency: f64, [re, im]: [f64; 2]) -> LoopPoint {
        LoopPoint { frequency, re, im, sensitivity: 0.0, plant: [0.0; 2] }
    }

    /// L(s) = K / (s (1 + s / w0)^2), K = 1000 1/s, w0 = 2 pi 100 Hz: the phase is -90 - 2 atan(w / w0) and reaches
    /// -180 degrees at w = w0, 100 Hz, where |L| = K / (w0 * 2) = 1000 / 1256.6 = 0.7958: a gain margin of 1.98 dB.
    /// T at that frequency is L / (1 + L) = -0.7958 / 0.2042 = -3.897.
    #[test]
    fn phase_crossover_and_closed_loop_of_a_hand_calculated_loop() {
        let (k, w0) = (1000.0, std::f64::consts::TAU * 100.0);
        let loop_at = |f: f64| {
            let w = std::f64::consts::TAU * f;
            // (1 + j w / w0)^2 = 1 - x^2 + 2 j x; s = j w.
            let x = w / w0;
            let (ar, ai) = (-w * 2.0 * x, w * (1.0 - x * x));
            let d2 = ar * ar + ai * ai;
            point(f, [k * ar / d2, -k * ai / d2])
        };
        let points: Vec<_> = sim_components::analysis::log_spaced(10.0, 1000.0, 200).into_iter().map(loop_at).collect();
        let crossing = phase_crossover(&points).unwrap();
        assert!((crossing / 100.0 - 1.0).abs() < 2e-3, "{crossing}");
        let phase = phases(points.iter().map(|p| [p.re, p.im]));
        assert!((phase[0] - (-90.0 - 2.0 * (0.1f64).atan().to_degrees())).abs() < 1e-9, "{}", phase[0]);
        let t = closed_loop(&loop_at(100.0));
        assert!((t[0] + 3.897).abs() < 1e-3 && t[1].abs() < 1e-9, "{t:?}");
        let m = loop_margins(&points);
        assert!((m.gain_margin_db - 1.984).abs() < 0.01, "{m:?}");
    }

    /// A phase that rises through a branch cut is carried up as it is carried down: 170 after -170 is -190, 3 turns
    /// of lead give 1080 + 10 from 1075.
    #[test]
    fn phases_unwrap_in_both_directions() {
        assert!((unwrap_near(170.0, -170.0) + 190.0).abs() < 1e-9);
        assert!((unwrap_near(-170.0, 170.0) - 190.0).abs() < 1e-9);
        assert!((unwrap_near(10.0, 1075.0) - 1090.0).abs() < 1e-9);
    }

    /// Phase -200 degrees at 10 Hz, rising to -120 at 50 Hz, then falling to -250 at 1 kHz (linear in the phase over
    /// log f); gain 30 dB at 10 Hz, -30 dB/decade. The rising crossing of -180 (share 20/80 of 10..50 Hz: 14.95 Hz,
    /// 25.7 dB) undoes the falling one before the first point; the falling one at share 60/130 of 50..1000 Hz is at
    /// 50 * 20^0.4615 = 199.2 Hz (-9.1 dB), the one the margin is read at, not the 14.95 Hz of the largest |L|.
    #[test]
    fn phase_crossover_follows_loop_margins_rule() {
        let at = |f: f64, deg: f64| {
            let g = 10f64.powf((30.0 - 30.0 * (f / 10.0).log10()) / 20.0);
            point(f, [g * deg.to_radians().cos(), g * deg.to_radians().sin()])
        };
        let points = [at(10.0, -200.0), at(50.0, -120.0), at(1000.0, -250.0)];
        let crossing = phase_crossover(&points).unwrap();
        assert!((crossing / 199.2 - 1.0).abs() < 2e-3, "{crossing}");
    }

    /// The measurement on the first registered machine with one motor (a belt axis): a response per motor, refined
    /// beyond the 60 points of `loop_frequencies`, the margins `sim-cli margins` prints for it (61.2 degrees, 11.56 dB). With friction on, the plant at
    /// 10 Hz leads (+47 degrees): at a tenth of the rated torque the LuGre bristles hold the axis like a spring. Without
    /// friction it is the inertia far below the belt's resonance: speed lags torque by 90 degrees (within 5).
    #[test]
    fn a_one_motor_belt_axis_is_measured_with_its_margins_and_an_inertia_for_a_plant() {
        let motors = |e: &MachineEntry| (e.build)(&e.config()).unwrap().motors().len();
        let entry = sim_machines::MACHINES.iter().copied().find(|e| motors(e) == 1).unwrap();
        let progress = Progress::default();
        let response = measure(entry, &entry.config(), &progress).unwrap();
        assert_eq!(response.curves.len(), 1);
        let m = response.margins[0];
        assert!((m.phase_margin_deg - 61.2).abs() < 0.1 && (m.gain_margin_db - 11.56).abs() < 0.01, "{m:?}");
        assert_eq!(progress.done.load(Relaxed), progress.total.load(Relaxed));
        assert!(response.curves[0][0].plant[0] > 0.0, "{:?}", response.curves[0][0]);
        let mut cfg = entry.config();
        cfg.set_text("effects.motor_friction = false").unwrap();
        cfg.set_text("effects.guide_friction = false").unwrap();
        let plant = measure(entry, &cfg, &Progress::default()).unwrap().curves[0][0].plant;
        assert!((plant[1].atan2(plant[0]).to_degrees() + 90.0).abs() < 5.0, "{plant:?}");
        let cancelled_one = Progress::default();
        cancelled_one.cancel.store(true, Relaxed);
        assert!(super::cancelled(&measure(entry, &entry.config(), &cancelled_one).unwrap_err()));
    }
}
