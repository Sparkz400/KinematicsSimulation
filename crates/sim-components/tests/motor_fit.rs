//! The machine-free identification against synthetic traces with known values: a motor with LuGre friction
//! under a speed loop, recorded the way a drive records it (encoder counts, speed from the counts, noisy
//! torque behind a current-loop lag and dead time), and a two-mass drive train.

use sim_components::analysis::{SweepPoint, log_spaced};
use sim_components::motor_fit::{
    Estimate, frequency_response, friction, lag, ringing, two_mass, two_mass_from_ringing,
};
use std::f64::consts::TAU;

/// A small deterministic noise source: uniform in -1..1.
struct Noise(u64);
impl Noise {
    fn next(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0 >> 11) as f64 / (1u64 << 52) as f64 - 1.0
    }
    /// Roughly normal, standard deviation 1.
    fn normal(&mut self) -> f64 {
        (0..12).map(|_| 0.5 * (self.next() + 1.0)).sum::<f64>() - 6.0
    }
}

fn within(name: &str, e: Estimate, truth: f64, sigmas: f64, relative: f64) {
    let (lo, hi) = e.interval();
    assert!(
        (e.value - truth).abs() <= sigmas * e.error.max(1e-15) || (e.value - truth).abs() <= relative * truth.abs(),
        "{name}: {e:?} (95 %: {lo:.4e} .. {hi:.4e}), true {truth:.4e}"
    );
}

const J: f64 = 2.0e-4;
const VISCOUS: f64 = 1.0e-4;
const COULOMB: f64 = 0.05;
const STICTION: f64 = 0.08;
const STRIBECK: f64 = 4.0;
const CONSTANT: f64 = 0.02;
const SIGMA0: f64 = 1.0e3;
const SIGMA1: f64 = 0.3;
const COUNTS: f64 = 1048576.0; // 20-bit encoder
const LAG: f64 = 1.5e-3;
const DEAD: f64 = 2.0e-3;

struct Recorded {
    time: Vec<f64>,
    setpoint_torque: Vec<f64>,
    torque: Vec<f64>,
    angle: Vec<f64>,
    speed: Vec<f64>,
}

/// One motor with LuGre friction and a constant torque, under a PI speed loop following `reference`,
/// simulated at 20 kHz and recorded at 1 kHz: angle in whole encoder counts, speed as counts per cycle,
/// the torque setpoint, and the torque acting (the setpoint behind a lag and a dead time) with noise.
fn motor(reference: impl Fn(f64) -> f64, seconds: f64, noise: f64) -> Recorded {
    let (h, every) = (5.0e-5, 20);
    let mut noise_source = Noise(0x9e3779b97f4a7c15);
    let (mut v, mut x, mut z, mut integral, mut acting) = (0.0f64, 0.0f64, 0.0f64, 0.0f64, 0.0f64);
    let mut queue = std::collections::VecDeque::from(vec![0.0; (DEAD / h).round() as usize]);
    let mut out = Recorded { time: vec![], setpoint_torque: vec![], torque: vec![], angle: vec![], speed: vec![] };
    let mut last_counts = 0.0;
    let mut command = 0.0;
    for k in 0..(seconds / h) as usize {
        let t = k as f64 * h;
        if k % every == 0 {
            let counts = (x / TAU * COUNTS).floor();
            let measured = (counts - last_counts) / COUNTS * TAU / (every as f64 * h);
            last_counts = counts;
            // Speed loop at 1 kHz on the measured speed.
            let e = reference(t) - measured;
            integral += e * every as f64 * h;
            command = 0.02 * e + 2.0 * integral;
            out.time.push(t);
            out.setpoint_torque.push(command);
            out.torque.push(acting + noise * noise_source.normal());
            out.angle.push(counts / COUNTS * TAU);
            out.speed.push(measured);
        }
        queue.push_back(command);
        let delayed = queue.pop_front().unwrap();
        acting += (h / LAG) * (delayed - acting);
        let g = COULOMB + (STICTION - COULOMB) * (-(v / STRIBECK).powi(2)).exp();
        // The bristles exactly over the step: z' = v - a z.
        let a = SIGMA0 * v.abs() / g;
        let z_new = if a > 0.0 { v / a + (z - v / a) * (-a * h).exp() } else { z + v * h };
        let z_dot = (z_new - z) / h;
        z = z_new;
        let friction = SIGMA0 * z + SIGMA1 * z_dot + VISCOUS * v;
        v += h * (acting - friction - CONSTANT) / J;
        x += h * v;
    }
    out
}

/// The test moves of the measurement procedure: fast sines for the inertia, a slow sine, plateaus of
/// constant speed both ways for the friction curve (where the Stribeck drop is), and a very slow
/// reversal that takes several rows to cross the presliding.
fn reference(t: f64) -> f64 {
    const PLATEAUS: [f64; 10] = [1.5, -1.5, 2.5, -2.5, 4.0, -4.0, 6.0, -6.0, 10.0, -10.0];
    match t {
        t if t < 4.0 => 120.0 * (TAU * 1.0 * t).sin() + 40.0 * (TAU * 3.7 * t).sin(),
        t if t < 8.0 => 25.0 * (TAU * 0.5 * t).sin(),
        t if t < 14.0 => PLATEAUS[(((t - 8.0) / 0.6) as usize).min(9)],
        t => 0.3 * (TAU * 0.5 * t).sin(),
    }
}

#[test]
fn friction_and_inertia_from_a_noisy_quantised_trace() {
    let r = motor(reference, 20.0, 0.004);
    let f = friction(&r.time, &r.angle, &r.speed, &r.torque).expect("fit");
    // The trace's torque is the one acting, so these are the true values. The model is the static
    // (Stribeck) curve of the LuGre model the trace was made with; the bristle damping and the lag of the
    // bristles make its residuals larger than the noise, so the standard errors are honest only to a
    // factor; a band of 5 standard errors or a few percent is asked.
    within("inertia", f.inertia, J, 5.0, 0.03);
    within("viscous", f.viscous, VISCOUS, 5.0, 0.05);
    within("coulomb", f.coulomb, COULOMB, 5.0, 0.05);
    within("constant", f.constant, CONSTANT, 5.0, 0.05);
    within("stiction", f.stiction.expect("stribeck drop"), STICTION, 5.0, 0.1);
    within("stribeck speed", f.stribeck_speed.unwrap(), STRIBECK, 5.0, 0.25);
    let sigma0 = f.presliding_stiffness.expect("presliding");
    within("presliding stiffness", sigma0, SIGMA0, 3.0, 0.3);
    assert!(f.quality.fit_percent > 90.0, "{:?}", f.quality);
}

#[test]
fn one_direction_is_not_fitted() {
    let r = motor(|t| 50.0 + 20.0 * (TAU * t).sin(), 3.0, 0.004);
    assert!(friction(&r.time, &r.angle, &r.speed, &r.torque).is_none());
}

#[test]
fn current_loop_lag_and_dead_time_from_the_torque_setpoint() {
    let r = motor(reference, 4.0, 0.004);
    let l = lag(&r.time, &r.setpoint_torque, &r.torque).expect("lag");
    within("gain", l.gain, 1.0, 5.0, 0.01);
    within("time constant", l.time_constant, LAG, 5.0, 0.1);
    within("dead time", l.dead_time, DEAD, 5.0, 0.05);
    assert!(l.quality.fit_percent > 90.0, "{:?}", l.quality);
}

/// Torque to motor speed of a two-mass system: (J_L s² + c s + k) / (s (J_M J_L s² + c J s + k J)).
fn two_mass_gain(f: f64, jm: f64, jl: f64, k: f64, c: f64) -> f64 {
    let w = TAU * f;
    let num = (k - jl * w * w).hypot(c * w);
    let den = w * (k * (jm + jl) - jm * jl * w * w).hypot(c * (jm + jl) * w);
    num / den
}

const JM: f64 = 1.2e-4;
const JL: f64 = 4.8e-4;
const K: f64 = 300.0;
const C: f64 = 0.01;

#[test]
fn two_mass_from_a_noisy_frequency_response() {
    let mut noise = Noise(7);
    let sweep: Vec<SweepPoint> = log_spaced(5.0, 400.0, 160)
        .into_iter()
        .map(|f| SweepPoint {
            frequency: f,
            gain: two_mass_gain(f, JM, JL, K, C) * (1.0 + 0.01 * noise.normal()),
            phase_deg: 0.0,
        })
        .collect();
    let m = two_mass(&sweep).expect("two-mass");
    within("total inertia", m.total_inertia, JM + JL, 5.0, 0.02);
    within("motor inertia", m.motor_inertia, JM, 5.0, 0.05);
    within("stiffness", m.stiffness, K, 5.0, 0.05);
    within("damping", m.damping, C, 5.0, 0.2);
    let fa = (K / JL).sqrt() / TAU;
    within("anti-resonance", m.anti_resonance, fa, 5.0, 0.01);
}

/// A two-mass train recorded at 8 kHz, the current-loop rate a drive's measuring function samples at
/// (at 1 kHz the hold of the torque moves the dip by 10 %), with white-noise torque: the response computed from the trace, then the model.
#[test]
fn frequency_response_from_an_excited_trace() {
    let mut noise = Noise(11);
    let (h, steps) = (1.25e-4, 64_000);
    let (mut wm, mut wl, mut twist) = (0.0f64, 0.0f64, 0.0f64);
    let (mut time, mut torque, mut speed) = (vec![], vec![], vec![]);
    for k in 0..steps {
        let held = 0.5 * noise.normal();
        time.push(k as f64 * h);
        torque.push(held);
        speed.push(wm + 0.002 * noise.normal());
        // Small steps of the stiff spring: semi-implicit Euler at a fifth of the sample time.
        for _ in 0..5 {
            let spring = K * twist + C * (wm - wl);
            wm += h / 5.0 * (held - spring - 1e-4 * wm) / JM;
            wl += h / 5.0 * spring / JL;
            twist += h / 5.0 * (wm - wl);
        }
    }
    let response = frequency_response(&time, &torque, &speed, &log_spaced(3.0, 300.0, 120)).expect("eight windows");
    let fa = (K / JL).sqrt() / TAU;
    let points: Vec<SweepPoint> = response.iter().map(|(p, _)| *p).collect();
    let m = two_mass(&points).expect("two-mass from the trace");
    within("anti-resonance", m.anti_resonance, fa, 5.0, 0.03);
    within("total inertia", m.total_inertia, JM + JL, 5.0, 0.1);
    within("stiffness", m.stiffness, K, 5.0, 0.15);
}

#[test]
fn stiffness_from_the_ringing_after_a_torque_step() {
    let mut noise = Noise(3);
    let h = 2.0e-5;
    let (mut wm, mut wl, mut twist) = (0.0f64, 0.0f64, 0.0f64);
    let (mut time, mut speed) = (vec![], vec![]);
    for k in 0..(0.3 / h) as usize {
        let t = k as f64 * h;
        let step = if t >= 0.05 { 0.5 } else { 0.0 };
        if k % 25 == 0 {
            time.push(t);
            speed.push(wm + 0.01 * noise.normal());
        }
        let spring = K * twist + C * (wm - wl);
        wm += h * (step - spring) / JM;
        wl += h * spring / JL;
        twist += h * (wm - wl);
    }
    let r = ringing(&time, &speed, 0.05, 0.3).expect("ringing");
    let mu = JM * JL / (JM + JL);
    let fr = (K / mu).sqrt() / TAU;
    within("resonance", r.frequency, fr, 5.0, 0.01);
    within("damping ratio", r.damping_ratio, 0.5 * C / (K * mu).sqrt(), 5.0, 0.1);
    let m = two_mass_from_ringing(&r, Estimate { value: JM + JL, error: 0.0 }, Estimate { value: JM, error: 0.0 });
    within("stiffness", m.stiffness, K, 5.0, 0.02);
    within("damping", m.damping, C, 5.0, 0.1);
}

/// The recorded trace as a TwinCAT Scope export (tab, decimal comma, a time and a value column per channel):
/// the angle as a raw 32-bit encoder count that wraps, the speed in rpm, the torques in whole per mille of
/// 0.25 N m (an INT16 as CiA 402 drives give it), and the torque channels' clock 1 ms behind.
fn as_twincat(r: &Recorded) -> String {
    let names = ["Counts", "Velo", "TorqueSet", "TorqueAct"];
    let units = ["inc (None)", "rpm (None)", "‰ (None)", "‰ (None)"];
    let pair = |key: &str, values: &[&str]| values.iter().map(|v| format!("{key}\t{v}")).collect::<Vec<_>>().join("\t");
    let mut out = String::from("Name\tsynthetic\nFile\ttrace.csv\n\n");
    out += &(pair("Name", &names) + "\n");
    out += &(pair("SampleTime[ms]", &["1"; 4]) + "\n");
    out += &(pair("Unit", &units) + "\n\n");
    let comma = |v: f64| format!("{v}").replace('.', ",");
    for i in 0..r.time.len() {
        let ms = r.time[i] * 1e3;
        let counts = ((r.angle[i] / TAU * COUNTS).round() + 4294967000.0) % 4294967296.0;
        let rpm = r.speed[i] * 60.0 / TAU;
        let permille = |t: f64| (t / 0.25 * 1000.0).round();
        let cells = [ms, counts, ms, rpm, ms + 1.0, permille(r.setpoint_torque[i]), ms + 1.0, permille(r.torque[i])];
        out += &(cells.map(comma).join("\t") + "\n");
    }
    out
}

#[test]
fn a_vendor_export_imported_through_a_mapping_fits_like_the_raw_trace() {
    let r = motor(reference, 20.0, 0.004);
    let map = sim_components::import::Mapping::parse(&format!(
        r#"
        format = "twincat"
        [[motor]]
        setpoint = {{ signal = "Counts", per_rev = {COUNTS}, wrap = 4294967296 }}
        angle = {{ signal = "Counts", per_rev = {COUNTS}, wrap = 4294967296 }}
        speed = "Velo"
        torque = {{ signal = "TorqueAct", reference = 0.25, time_offset_s = -0.001 }}
        torque_setpoint = {{ signal = "TorqueSet", reference = 0.25, time_offset_s = -0.001 }}
        "#
    ))
    .unwrap();
    let imported = sim_components::import::import(&as_twincat(&r), &map).unwrap();
    let m = &imported.trace.motors[0];
    let t = &imported.trace.time;
    let f = friction(t, &m.angle, &m.speed, &m.torque).expect("fit");
    within("inertia", f.inertia, J, 5.0, 0.03);
    within("coulomb", f.coulomb, COULOMB, 5.0, 0.05);
    within("stiction", f.stiction.expect("stribeck drop"), STICTION, 5.0, 0.1);
    let l = lag(t, imported.torque_setpoints[0].as_ref().unwrap(), &m.torque).expect("lag");
    within("time constant", l.time_constant, LAG, 5.0, 0.1);
    within("dead time", l.dead_time, DEAD, 5.0, 0.05);
}

/// torque = J a + b v + Tc sign v + T0 with an analytic speed, 1 kHz, normal noise of `noise`.
fn static_model(seed: u64, noise: f64) -> (Vec<f64>, Vec<f64>, Vec<f64>, Vec<f64>) {
    let mut n = Noise(seed);
    let time: Vec<f64> = (0..10_000).map(|i| i as f64 * 1e-3).collect();
    let speed: Vec<f64> = time.iter().map(|t| 100.0 * (TAU * 0.7 * t).sin() + 30.0 * (TAU * 2.3 * t).sin()).collect();
    let accel: Vec<f64> = time
        .iter()
        .map(|t| 100.0 * TAU * 0.7 * (TAU * 0.7 * t).cos() + 30.0 * TAU * 2.3 * (TAU * 2.3 * t).cos())
        .collect();
    let torque = speed
        .iter()
        .zip(&accel)
        .map(|(v, a)| J * a + VISCOUS * v + COULOMB * v.signum() + CONSTANT + noise * n.normal())
        .collect();
    let angle = vec![0.0; time.len()];
    (time, angle, speed, torque)
}

/// Found by review: windows that spanned a reversal mixed the two Coulomb levels, biasing viscous friction
/// by 15 % with standard errors ten times too small. Over seeds the errors have to explain the scatter.
#[test]
fn friction_estimates_are_unbiased_and_their_errors_hold_over_seeds() {
    let (time, angle, speed, torque) = static_model(1, 0.0);
    let f = friction(&time, &angle, &speed, &torque).unwrap();
    assert!(
        (f.viscous.value - VISCOUS).abs() < 0.02 * VISCOUS && (f.coulomb.value - COULOMB).abs() < 0.005 * COULOMB,
        "{f:?}"
    );
    let mut z = [0.0; 3];
    for seed in 1..=12 {
        let (time, angle, speed, torque) = static_model(seed * 7919, 0.004);
        let f = friction(&time, &angle, &speed, &torque).unwrap();
        for (k, (e, truth)) in
            [(f.viscous, VISCOUS), (f.coulomb, COULOMB), (f.constant, CONSTANT)].into_iter().enumerate()
        {
            z[k] += ((e.value - truth) / e.error).powi(2) / 12.0;
        }
    }
    assert!(z.iter().all(|z| z.sqrt() < 2.0), "RMS z {:?}", z.map(f64::sqrt));
}

/// Found by review: the lag model started from the first output sample, so its noise biased the gain.
#[test]
fn lag_does_not_hang_on_the_first_sample() {
    let mut r = motor(reference, 4.0, 0.004);
    let before = lag(&r.time, &r.setpoint_torque, &r.torque).expect("lag");
    r.torque[0] += 0.05;
    let after = lag(&r.time, &r.setpoint_torque, &r.torque).expect("lag");
    // One sample of 4000 moved by 12 noise widths may not move the fit by more than a standard error.
    assert!((after.gain.value - before.gain.value).abs() < before.gain.error, "{:?} {:?}", before.gain, after.gain);
    assert!(
        (after.time_constant.value - before.time_constant.value).abs() < before.time_constant.error,
        "{:?} {:?}",
        before.time_constant,
        after.time_constant
    );
}

/// Found by review: with one window the coherence of pure noise is 1, and below eight rows it panicked.
#[test]
fn a_response_needs_eight_windows() {
    let mut n = Noise(5);
    let time: Vec<f64> = (0..1000).map(|i| i as f64 * 1e-3).collect();
    let (a, b): (Vec<f64>, Vec<f64>) = time.iter().map(|_| (n.normal(), n.normal())).unzip();
    assert!(frequency_response(&time, &a, &b, &[2.0, 10.0]).is_none());
    assert!(frequency_response(&time[..6], &a[..6], &b[..6], &[10.0]).is_none());
    let coherence = frequency_response(&time, &a, &b, &[20.0, 100.0]).unwrap();
    assert!(coherence.iter().all(|(_, c)| *c < 0.5), "{coherence:?}");
}
