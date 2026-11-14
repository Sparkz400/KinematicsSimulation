//! The numbers behind the views that show a drive's real torque and the tool's real error, for the app and the sizing
//! report alike: the torque-speed chart per motor with its limits, the RMS point, the tool's error split normal and
//! tangential to the path, a histogram, the torque split stacked, the utilisation of the drive over time, and the
//! spectrum. Everything works on a run's signals as columns (one value per millisecond, the first column the time).

use crate::drive::{DriveParams, WINDING_PER_DC_LINK};
use sim_core::{SignalInfo, TORQUE_PARTS};

/// How many speeds the torque-speed chart's curves are sampled at, from the highest speed backwards to the highest
/// forwards: odd, so that standstill is one of them.
pub const CURVE_POINTS: usize = 241;

/// A motor's torque-speed chart: lines of (speed rad/s, torque N m), each as [highest, lowest].
#[derive(Debug, Clone, PartialEq)]
pub struct TorqueSpeed {
    /// What motor and drive give at most (`DriveParams::torque_curve`): the peak torque, faded or field-weakened.
    pub peak: [Vec<[f64; 2]>; 2],
    /// The continuous (S1) rating, the rated torque up to the rated speed.
    pub s1: [Vec<[f64; 2]>; 2],
    /// What the winding's voltage leaves at the DC link `dc_voltage` (`DriveParams::voltage_limited_torque`).
    pub voltage: [Vec<[f64; 2]>; 2],
    /// The torque of the drive's maximum current (`ratings.drive_max_current_a`); `None` without that limit.
    pub current_limit: Option<f64>,
    pub dc_voltage: f64,
}

/// The chart of `drive` with the DC link at `dc_voltage`, its curves sampled at `CURVE_POINTS` speeds up to a tenth
/// beyond the highest of the no-load speed and the speed limit.
// shortcut: no S3 line, as no catalogue the simulator reads gives S3 data; add it with that data.
pub fn torque_speed(drive: &DriveParams, dc_voltage: f64) -> TorqueSpeed {
    let top = 1.1 * drive.no_load_speed.max(drive.max_speed);
    let speeds: Vec<f64> =
        (0..CURVE_POINTS).map(|i| top * (2.0 * i as f64 / (CURVE_POINTS - 1) as f64 - 1.0)).collect();
    let line = |f: &dyn Fn(f64) -> (f64, f64)| -> [Vec<[f64; 2]>; 2] {
        let pairs: Vec<_> = speeds.iter().map(|&w| (w, f(w))).collect();
        [pairs.iter().map(|&(w, (_, high))| [w, high]).collect(), pairs.iter().map(|&(w, (low, _))| [w, low]).collect()]
    };
    let s1 = |sign: f64| {
        vec![[-drive.rated_speed, sign * drive.rated_torque], [drive.rated_speed, sign * drive.rated_torque]]
    };
    TorqueSpeed {
        peak: line(&|w| drive.torque_curve(w)),
        s1: [s1(1.0), s1(-1.0)],
        voltage: line(&|w| drive.voltage_limited_torque(w, dc_voltage)),
        current_limit: drive.drive_max_current.is_finite().then(|| drive.torque_of(drive.drive_max_current)),
        dc_voltage,
    }
}

/// The DC link a run's chart is drawn at: the lowest the run's `dc_link_voltage` signal saw (the voltage limit binds
/// hardest there), or the bus the drive is set to where the run has none.
pub fn dc_link_of(drive: &DriveParams, dc_link: Option<&[f64]>) -> f64 {
    dc_link.and_then(|v| v.iter().copied().filter(|u| *u > 0.0).reduce(f64::min)).unwrap_or(drive.bus_voltage)
}

/// The RMS point of a cycle, (mean |speed|, sqrt(mean torque²)): where the motor sits on its S1 curve thermally.
/// Samples equally spaced in time, as a run's are.
pub fn rms_point(speed: &[f64], torque: &[f64]) -> [f64; 2] {
    let n = speed.len().min(torque.len()).max(1) as f64;
    let mean = speed.iter().map(|w| w.abs()).sum::<f64>() / n;
    [mean, (torque.iter().map(|t| t * t).sum::<f64>() / n).sqrt()]
}

/// An error vector split against the direction of motion: (normal, tangential). The normal part is the length of what
/// is across the direction (the contour error), the tangential the signed part along it: with the error as setpoint
/// less actual, positive is lagging behind. With no motion (`direction` zero) all of it is normal.
pub fn split(error: &[f64], direction: &[f64]) -> (f64, f64) {
    let length = direction.iter().map(|d| d * d).sum::<f64>().sqrt();
    let total = error.iter().map(|e| e * e).sum::<f64>();
    if length <= 0.0 {
        return (total.sqrt(), 0.0);
    }
    let along = error.iter().zip(direction).map(|(e, d)| e * d).sum::<f64>() / length;
    ((total - along * along).max(0.0).sqrt(), along)
}

/// The tool's real error over a run (`split`): the axes' position setpoints less where the tool really is (`axis.<id>.
/// position`, belt stretch and sag included, not the drives' encoder-side `following_error`), against the direction of
/// their speed setpoints: of the axes in m (its place) in m, and of the axes in rad (its turn, an arm's
/// orientation) the length in rad, apart, so that metres and radians are never added.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct ToolError {
    pub time: Vec<f64>,
    pub normal: Vec<f64>,
    pub tangential: Vec<f64>,
    /// Empty on a machine without axes in rad.
    pub angle: Vec<f64>,
}

/// The `ToolError` of a run's columns; `None` where the run has no axis positions with setpoints (a trace).
// shortcut: the setpoint is the one sampled with the position, held over the profile cycle, so the tangential part may
// carry up to a cycle's way of setpoint lag at speed; take the plan at the sample's time (`Control::plan_now`) into the
// run if that matters.
pub fn tool_error(signals: &[SignalInfo], columns: &[Vec<f64>]) -> Option<ToolError> {
    let find = |id: &str| signals.iter().position(|s| s.id == id).map(|i| columns[i].as_slice());
    let axes = |unit: &str| -> Vec<(Vec<f64>, Option<&[f64]>)> {
        signals
            .iter()
            .zip(columns)
            .filter(|(s, _)| s.unit == unit && s.id.starts_with("axis.") && s.id.ends_with(".position"))
            .filter_map(|(s, actual)| {
                let set = find(&format!("{}_setpoint", s.id))?;
                let error = set.iter().zip(actual).map(|(s, a)| s - a).collect();
                Some((error, find(&s.id.replace(".position", ".velocity_setpoint"))))
            })
            .collect()
    };
    let (place, turn) = (axes("m"), axes("rad"));
    if place.is_empty() && turn.is_empty() {
        return None;
    }
    let time = columns.first()?.clone();
    let mut out = ToolError { time, ..ToolError::default() };
    for i in 0..out.time.len() {
        let error: Vec<f64> = place.iter().map(|(e, _)| e[i]).collect();
        let direction: Vec<f64> = place.iter().map(|(_, v)| v.map_or(0.0, |v| v[i])).collect();
        let (normal, tangential) = split(&error, &direction);
        out.normal.push(normal);
        out.tangential.push(tangential);
        if !turn.is_empty() {
            out.angle.push(turn.iter().map(|(e, _)| e[i] * e[i]).sum::<f64>().sqrt());
        }
    }
    Some(out)
}

/// A histogram of `values` in `bins` bins of equal width from the lowest to the highest: (lower edge, upper edge, how
/// many). Empty for no values.
pub fn histogram(values: &[f64], bins: usize) -> Vec<(f64, f64, usize)> {
    let Some(low) = values.iter().copied().reduce(f64::min) else { return Vec::new() };
    let high = values.iter().copied().fold(low, f64::max);
    let (bins, width) = (bins.max(1), ((high - low) / bins.max(1) as f64).max(f64::MIN_POSITIVE));
    let mut counts = vec![0; bins];
    for v in values {
        counts[(((v - low) / width) as usize).min(bins - 1)] += 1;
    }
    counts.into_iter().enumerate().map(|(i, n)| (low + i as f64 * width, low + (i + 1) as f64 * width, n)).collect()
}

/// A motor's torque split (`TORQUE_PARTS`, the signals `motor.<id>.torque_<part>`) as the run has it: (part, values),
/// in the order of `TORQUE_PARTS`; empty where the machine reports none.
pub fn torque_parts<'a>(
    signals: &[SignalInfo],
    columns: &'a [Vec<f64>],
    motor: &str,
) -> Vec<(&'static str, &'a [f64])> {
    TORQUE_PARTS
        .iter()
        .filter_map(|part| {
            let id = format!("motor.{motor}.torque_{part}");
            signals.iter().position(|s| s.id == id).map(|i| (*part, columns[i].as_slice()))
        })
        .collect()
}

/// Parts stacked: the first, the first two added, and so on; the last line is their sum, the motor torque.
pub fn stack(parts: &[&[f64]]) -> Vec<Vec<f64>> {
    let mut sum = vec![0.0; parts.iter().map(|p| p.len()).min().unwrap_or(0)];
    parts
        .iter()
        .map(|part| {
            sum.iter_mut().zip(*part).for_each(|(s, p)| *s += p);
            sum.clone()
        })
        .collect()
}

/// The torque the drive has available at a speed in the direction of `torque`: the torque-speed curve
/// (`DriveParams::torque_curve`), and the torque of the drive's maximum current where that is lower.
pub fn available_torque(drive: &DriveParams, speed: f64, torque: f64) -> f64 {
    let (low, high) = drive.torque_curve(speed);
    let curve = if torque >= 0.0 { high } else { -low };
    if drive.drive_max_current.is_finite() { curve.min(drive.torque_of(drive.drive_max_current)) } else { curve }
}

/// How much of what is available at its speed a torque uses (`available_torque`): 1 is all of it. Infinite where
/// nothing is available and some torque is asked.
pub fn torque_utilisation(drive: &DriveParams, speed: f64, torque: f64) -> f64 {
    let available = available_torque(drive, speed, torque);
    if torque == 0.0 {
        0.0
    } else if available > 0.0 {
        torque.abs() / available
    } else {
        f64::INFINITY
    }
}

/// The current the winding may carry: the motor's maximum current, or the drive's where that is lower.
pub fn current_limit(drive: &DriveParams) -> f64 {
    drive.max_current.min(drive.drive_max_current)
}

/// One line of the utilisation timeline (`utilisation`): its name (a translation key) and its share of its limit per
/// sample.
pub type Share = (&'static str, Vec<f64>);

/// A motor's utilisation over a run, each as a share of its limit, 1 at the limit: torque against what is available at
/// its speed; the phase current (q and d) against `current_limit`; the drive's two overload image nodes against the
/// image's trip level (with effects.drive_overload); winding and housing rise against the rated rise (with
/// effects.winding_heating; the housing's with effects.motor_thermal_nodes); and the DC link voltage against the
/// overvoltage trip, with the braking threshold below it. Only the lines whose signals the run has and whose effect
/// was on.
// shortcut: the housing is held against the winding's rated rise, as the config has no housing limit; give it its own
// once a maker states one.
pub fn utilisation(drive: &DriveParams, signals: &[SignalInfo], columns: &[Vec<f64>], motor: &str) -> Vec<Share> {
    let get = |name: &str| {
        let id = format!("motor.{motor}.{name}");
        signals.iter().position(|s| s.id == id).map(|i| columns[i].as_slice())
    };
    let mut out = Vec::new();
    if let (Some(speed), Some(torque)) = (get("speed"), get("torque")) {
        let share = speed.iter().zip(torque).map(|(w, t)| torque_utilisation(drive, *w, *t)).collect();
        out.push(("charts-torque", share));
    }
    if let Some(current) = get("current") {
        let d = get("d_current");
        let limit = current_limit(drive);
        let share = (0..current.len()).map(|i| current[i].hypot(d.map_or(0.0, |d| d[i])) / limit).collect();
        out.push(("charts-current", share));
    }
    if let (Some(o), Some(image), Some(chip)) =
        (&drive.overload, get("drive_overload_image"), get("drive_overload_chip"))
    {
        out.push(("charts-overload-slow", image.iter().zip(chip).map(|(i, c)| (i - c) / o.trip).collect()));
        out.push(("charts-overload-chip", chip.iter().map(|c| c / o.trip).collect()));
    }
    if drive.heating {
        if let Some(rise) = get("winding_temperature") {
            out.push(("charts-winding", rise.iter().map(|r| r / drive.rated_rise).collect()));
        }
        if let (Some(_), Some(rise)) = (drive.thermal_nodes, get("housing_temperature")) {
            out.push(("charts-housing", rise.iter().map(|r| r / drive.rated_rise).collect()));
        }
    }
    if let (true, Some(u)) =
        (drive.dc_link && drive.overvoltage.is_finite() && drive.overvoltage > 0.0, get("dc_link_voltage"))
    {
        out.push(("charts-dc-link", u.iter().map(|u| u / drive.overvoltage).collect()));
    }
    out
}

/// The DC link's braking threshold as a share of its overvoltage trip, for the utilisation timeline; `None` without
/// the DC link simulated.
pub fn braking_share(drive: &DriveParams) -> Option<f64> {
    (drive.dc_link && drive.overvoltage.is_finite() && drive.overvoltage > 0.0)
        .then(|| drive.braking_threshold / drive.overvoltage)
}

/// The amplitude spectrum of samples `dt` apart: (frequency Hz, amplitude) from 0 to the Nyquist frequency. Of the
/// newest power of two of them, mean removed, through a Hann window; a sine of amplitude A on a bin gives A there.
/// Empty for fewer than 8 samples.
pub fn spectrum(values: &[f64], dt: f64) -> Vec<[f64; 2]> {
    if values.len() < 8 {
        return Vec::new();
    }
    let n = 1usize << values.len().ilog2();
    let amplitudes = hann_amplitudes(&values[values.len() - n..]);
    amplitudes.iter().enumerate().map(|(k, a)| [k as f64 / (n as f64 * dt), *a]).collect()
}

/// A spectrogram of samples `dt` apart: frames of `frame` samples (a power of two) every half frame, each frame's
/// amplitudes as `spectrum` gives them. With the time of each frame's middle and the bin width in Hz.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Spectrogram {
    pub times: Vec<f64>,
    pub bin_hz: f64,
    pub frames: Vec<Vec<f64>>,
}

/// The `Spectrogram` of `values`, whose first sample is at `start` s.
pub fn spectrogram(values: &[f64], dt: f64, start: f64, frame: usize) -> Spectrogram {
    let frame = frame.next_power_of_two().max(8);
    let hop = frame / 2;
    let mut out = Spectrogram { bin_hz: 1.0 / (frame as f64 * dt), ..Spectrogram::default() };
    let mut at = 0;
    while at + frame <= values.len() {
        out.times.push(start + (at + frame / 2) as f64 * dt);
        out.frames.push(hann_amplitudes(&values[at..at + frame]));
        at += hop;
    }
    out
}

/// The amplitudes of bins 0 to n / 2 of n samples (a power of two), mean removed, Hann-windowed and scaled by 2 / the
/// window's sum, so that a sine on a bin shows its amplitude.
fn hann_amplitudes(values: &[f64]) -> Vec<f64> {
    let n = values.len();
    let mean = values.iter().sum::<f64>() / n as f64;
    let window = |i: usize| 0.5 - 0.5 * (std::f64::consts::TAU * i as f64 / n as f64).cos();
    let mut re: Vec<f64> = values.iter().enumerate().map(|(i, v)| (v - mean) * window(i)).collect();
    let mut im = vec![0.0; n];
    fft(&mut re, &mut im);
    let scale = 2.0 / (0..n).map(window).sum::<f64>();
    (0..=n / 2).map(|k| re[k].hypot(im[k]) * scale).collect()
}

/// The discrete Fourier transform in place, radix 2 (the length a power of two): X_k = sum x_j e^(-2 pi i j k / n).
fn fft(re: &mut [f64], im: &mut [f64]) {
    let n = re.len();
    let mut j = 0;
    for i in 1..n {
        let mut bit = n >> 1;
        while j & bit != 0 {
            j ^= bit;
            bit >>= 1;
        }
        j |= bit;
        if i < j {
            re.swap(i, j);
            im.swap(i, j);
        }
    }
    let mut len = 2;
    while len <= n {
        let angle = -std::f64::consts::TAU / len as f64;
        for start in (0..n).step_by(len) {
            for k in 0..len / 2 {
                let (wr, wi) = ((angle * k as f64).cos(), (angle * k as f64).sin());
                let (a, b) = (start + k, start + k + len / 2);
                let (tr, ti) = (re[b] * wr - im[b] * wi, re[b] * wi + im[b] * wr);
                (re[b], im[b]) = (re[a] - tr, im[a] - ti);
                (re[a], im[a]) = (re[a] + tr, im[a] + ti);
            }
        }
        len <<= 1;
    }
}

/// The voltage limit at a DC link, as the winding gets it (`WINDING_PER_DC_LINK`), for labels.
pub fn winding_voltage(dc_voltage: f64) -> f64 {
    WINDING_PER_DC_LINK * dc_voltage
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::drive::FieldWeakening;

    /// A drive without field weakening: kt = kE = 1 N m/A, R = 2 Ohm, 10 A at most, peak 10 N m, rated 4 N m at
    /// 100 rad/s, no-load 200 rad/s, on a DC link of 300 V.
    fn drive() -> DriveParams {
        let mut cfg = sim_core::Config::parse(include_str!("../../machine-single-axis/default.toml")).unwrap();
        cfg.set_text("effects.field_weakening = false").unwrap();
        let p = DriveParams::from_config(&cfg, 62.5e-6, 125e-6).unwrap();
        DriveParams {
            kt: 1.0,
            ke: 1.0,
            resistance: 2.0,
            max_current: 10.0,
            saturation: 0.0,
            peak_torque: 10.0,
            rated_torque: 4.0,
            rated_speed: 100.0,
            no_load_speed: 200.0,
            max_speed: 200.0,
            bus_voltage: 300.0,
            drive_max_current: f64::INFINITY,
            field_weakening: None,
            ..p
        }
    }

    /// Without field weakening the steady winding meets R i + kE w = U: at 400 rad/s on 300 V the winding has
    /// sqrt(3/2) * 300 = 367.42 V, so motoring i = (367.42 - 400) / 2 < 0, none; at 300 rad/s (367.42 - 300) / 2 =
    /// 33.7 A, held at 10 A, 10 N m; at 360 rad/s 3.712 A, 3.712 N m. Braking at 360 rad/s: (367.42 + 360) / 2, held at
    /// 10 A. The chart's curve at each of its speeds is the motor model's.
    #[test]
    fn the_voltage_limit_without_field_weakening_is_the_hand_calculated_one_and_the_chart_follows_it() {
        let p = drive();
        let u = WINDING_PER_DC_LINK * 300.0;
        assert!((u - 367.423).abs() < 1e-3);
        assert_eq!(p.voltage_limited_torque(400.0, 300.0).1, 0.0);
        assert_eq!(p.voltage_limited_torque(300.0, 300.0).1, 10.0);
        let at_360 = p.voltage_limited_torque(360.0, 300.0);
        assert!((at_360.1 - (u - 360.0) / 2.0).abs() < 1e-12 && (at_360.1 - 3.7117).abs() < 1e-4, "{at_360:?}");
        assert_eq!(at_360.0, -10.0);
        assert_eq!(p.voltage_limited_torque(-360.0, 300.0), (-at_360.1, 10.0));
        let chart = torque_speed(&p, 300.0);
        assert_eq!(chart.voltage[0].len(), CURVE_POINTS);
        for (high, low) in chart.voltage[0].iter().zip(&chart.voltage[1]) {
            let (l, h) = p.voltage_limited_torque(high[0], 300.0);
            assert!(high[1] == h && low[1] == l && high[0] == low[0]);
        }
        // Standstill is sampled, and the speeds reach 1.1 times the no-load speed.
        assert!(chart.voltage[0].iter().any(|p| p[0] == 0.0));
        assert!((chart.peak[0].last().unwrap()[0] - 220.0).abs() < 1e-9);
    }

    /// With field weakening the voltage limit at 540 V is the d-q model's of the drive's own test
    /// (`the_voltage_limit_gives_the_hand_calculated_torque_from_r_l_and_the_dc_link`): 15.614 N m at 600 rad/s by hand,
    /// and it does not stop at the peak torque, which `torque_curve` caps at.
    #[test]
    fn the_voltage_limit_with_field_weakening_is_the_d_q_models() {
        let kt: f64 = 6.0 / 4.7;
        let inductance = kt * kt / (2.0 * 4.0 * 13.0);
        let p = DriveParams {
            kt,
            ke: kt,
            inductance,
            max_current: 15.1,
            peak_torque: 12.0,
            max_voltage: WINDING_PER_DC_LINK * 540.0,
            field_weakening: Some(FieldWeakening { pole_pairs: 4.0, d_inductance: inductance }),
            ..drive()
        };
        let at_600 = p.voltage_limited_torque(600.0, 540.0).1;
        assert!((at_600 - 15.614).abs() < 2e-3, "{at_600} N m");
        assert_eq!(p.torque_curve(600.0).1, 12.0);
    }

    /// The RMS point of speeds 1, -3, 2, 0 rad/s and torques 1, -2, 3, 0 N m: mean |speed| 6 / 4 = 1.5, RMS torque
    /// sqrt((1 + 4 + 9 + 0) / 4) = sqrt(3.5) = 1.8708.
    #[test]
    fn the_rms_point_is_the_root_mean_square_torque_at_the_mean_speed() {
        let [speed, torque] = rms_point(&[1.0, -3.0, 2.0, 0.0], &[1.0, -2.0, 3.0, 0.0]);
        assert_eq!(speed, 1.5);
        assert!((torque - 3.5f64.sqrt()).abs() < 1e-12 && (torque - 1.8708).abs() < 1e-4);
    }

    /// On a straight line along (3, 4) (unit (0.6, 0.8)) an error of (1, 0) mm: along it 0.6 mm (lagging), across it
    /// sqrt(1 - 0.36) = 0.8 mm. An error of (0, 2) mm with the motion along x: 2 mm across, none along. Standing still,
    /// all of it is across.
    #[test]
    fn a_known_offset_on_a_straight_line_splits_into_its_normal_and_tangential_parts() {
        let (normal, along) = split(&[1e-3, 0.0], &[3.0, 4.0]);
        assert!((normal - 0.8e-3).abs() < 1e-15 && (along - 0.6e-3).abs() < 1e-15, "{normal} {along}");
        assert_eq!(split(&[0.0, 2e-3], &[0.5, 0.0]), (2e-3, 0.0));
        let (normal, along) = split(&[-1e-3, 0.0], &[0.5, 0.0]);
        assert!(normal == 0.0 && along == -1e-3);
        assert_eq!(split(&[3e-3, 4e-3], &[0.0, 0.0]), (5e-3, 0.0));
        // The same from a run's columns: two axes in m, the motion along x, the error 1 mm along and 2 mm across.
        let signals = [
            "time",
            "axis.x.position",
            "axis.x.position_setpoint",
            "axis.x.velocity_setpoint",
            "axis.y.position",
            "axis.y.position_setpoint",
        ]
        .map(|id| SignalInfo::new(id, "k", "q", if id == "time" { "s" } else { "m" }));
        let columns = vec![
            vec![0.0, 1e-3],
            vec![0.1, 0.2],
            vec![0.101, 0.201],
            vec![0.2, 0.2],
            vec![0.0, 0.0],
            vec![2e-3, -2e-3],
        ];
        let e = tool_error(&signals, &columns).unwrap();
        let near = |v: &[f64], x: f64| v.iter().all(|v| (v - x).abs() < 1e-15);
        assert!(near(&e.normal, 2e-3) && near(&e.tangential, 1e-3) && e.angle.is_empty(), "{e:?}");
    }

    /// A sine of 0.3 on bin 50 of 1024 samples 1 ms apart, 48.828 Hz, plus an offset: the spectrum's highest bin is
    /// bin 50 at 50 / 1.024 s, with the amplitude 0.3 (Hann, scaled by 2 / the window's sum).
    #[test]
    fn the_spectrum_of_a_pure_sine_lands_at_its_bin() {
        let (n, dt, k) = (1024, 1e-3, 50.0);
        let hz = k / (n as f64 * dt);
        let values: Vec<f64> =
            (0..n).map(|i| 2.0 + 0.3 * (std::f64::consts::TAU * hz * i as f64 * dt + 0.4).sin()).collect();
        let s = spectrum(&values, dt);
        assert_eq!(s.len(), n / 2 + 1);
        let top = s.iter().enumerate().max_by(|a, b| a.1[1].total_cmp(&b.1[1])).unwrap();
        assert_eq!(top.0, 50);
        assert!((top.1[0] - 48.828125).abs() < 1e-9 && (top.1[1] - 0.3).abs() < 1e-9, "{:?}", top.1);
        // The spectrogram's frames see the same bin: frames of 256 give bins 4 times wider, so bin 12.5, between 12 and 13.
        let g = spectrogram(&values, dt, 0.0, 256);
        assert_eq!((g.frames.len(), g.bin_hz), (7, 1.0 / 0.256));
        let peak = g.frames[3].iter().enumerate().max_by(|a, b| a.1.total_cmp(b.1)).unwrap().0;
        assert!(peak == 12 || peak == 13);
    }

    /// Parts 1, -0.5 and 2 N m stack to 1, 0.5 and 2.5: the last is the sum.
    #[test]
    fn stacked_parts_end_at_their_sum() {
        let parts: [&[f64]; 3] = [&[1.0, 0.0], &[-0.5, 1.0], &[2.0, -1.0]];
        assert_eq!(stack(&parts), vec![vec![1.0, 0.0], vec![0.5, 1.0], vec![2.5, 0.0]]);
    }

    /// At 150 rad/s the peak fades linearly from 10 N m at 100 rad/s to none at 200 rad/s: 5 N m available, so 2.5 N m
    /// uses half of it. Braking at that speed the full 10 N m is there: -2.5 N m uses a quarter. A drive limited to
    /// 4 A (4 N m) leaves 4 N m: 2.5 / 4 = 0.625.
    #[test]
    fn the_utilisation_is_the_torque_over_what_is_available_at_that_speed() {
        let p = drive();
        assert_eq!(available_torque(&p, 150.0, 2.5), 5.0);
        assert_eq!(torque_utilisation(&p, 150.0, 2.5), 0.5);
        assert_eq!(torque_utilisation(&p, 150.0, -2.5), 0.25);
        let limited = DriveParams { drive_max_current: 4.0, ..p };
        assert_eq!(torque_utilisation(&limited, 150.0, 2.5), 0.625);
        assert_eq!(current_limit(&limited), 4.0);
        assert_eq!(torque_speed(&limited, 300.0).current_limit, Some(4.0));
    }

    /// Values 0, 1, 1, 2, 4 in 4 bins from 0 to 4: one each in [0, 1) and [2, 3), two in [1, 2), the top one in the last.
    #[test]
    fn a_histogram_counts_each_value_once() {
        let h = histogram(&[0.0, 1.0, 1.0, 2.0, 4.0], 4);
        assert_eq!(h.iter().map(|b| b.2).collect::<Vec<_>>(), vec![1, 2, 1, 1]);
        assert_eq!((h[0].0, h[3].1), (0.0, 4.0));
    }
}
