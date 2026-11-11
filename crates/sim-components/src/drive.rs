//! Servo drive: cascaded position (P, the Kv factor), velocity (PI) and current control on the
//! motor's own encoder, with feedforward, measurement effects and torque limits. One switch each.

use sim_core::{Config, ConfigError};
use std::f64::consts::TAU;

type Result<T> = std::result::Result<T, ConfigError>;

#[derive(Debug, Clone)]
pub struct DriveParams {
    pub kv: f64,
    pub kp: f64,
    pub ki: f64,
    /// Feedforward weights; 0 when switched off.
    pub velocity_ff: f64,
    pub torque_ff: f64,
    /// Fraction of the current error removed per base step (first-order lag).
    pub current_gain: f64,
    pub peak_torque: f64,
    pub rated_speed: f64,
    pub no_load_speed: f64,
    pub limit: bool,
    /// Encoder step in rad; 0 for an ideal encoder.
    pub encoder_step: f64,
    /// Low-pass gains per velocity cycle; 1 when the filter is off.
    pub velocity_filter: f64,
    pub torque_filter: f64,
    pub notch: Option<[f64; 5]>,
    pub velocity_cycle: f64,
}

impl DriveParams {
    /// `dt` is the base step, which is also the current-loop cycle.
    pub fn from_config(cfg: &Config, dt: f64, velocity_cycle: f64) -> Result<Self> {
        let on = |effect: &str| cfg.bool(&format!("effects.{effect}"));
        let low_pass = |effect: &str, leaf: &str| -> Result<f64> {
            let tau = cfg.positive(&format!("drive.{leaf}"))?;
            Ok(if on(effect)? { 1.0 - (-velocity_cycle / tau).exp() } else { 1.0 })
        };
        let kp = cfg.positive("drive.velocity_kp_nm_s_rad")?;
        let notch = if on("notch_filter")? {
            let (f, bw) = (cfg.positive("drive.notch_frequency_hz")?, cfg.positive("drive.notch_bandwidth_hz")?);
            Some(notch_coefficients(f, bw, velocity_cycle))
        } else {
            None
        };
        Ok(Self {
            kv: cfg.positive("drive.kv_1_s")?,
            kp,
            ki: kp / cfg.positive("drive.velocity_tn_s")?,
            velocity_ff: if on("velocity_feedforward")? { cfg.f64("drive.velocity_feedforward_factor")? } else { 0.0 },
            torque_ff: if on("torque_feedforward")? { cfg.f64("drive.torque_feedforward_factor")? } else { 0.0 },
            current_gain: 1.0 - (-dt / cfg.positive("drive.current_time_constant_s")?).exp(),
            peak_torque: cfg.positive("drive.peak_torque_nm")?,
            rated_speed: cfg.positive("drive.rated_speed_rad_s")?,
            no_load_speed: cfg.positive("drive.no_load_speed_rad_s")?,
            limit: on("torque_limit")?,
            encoder_step: if on("encoder_resolution")? { TAU / cfg.positive("drive.encoder_counts_per_rev")? } else { 0.0 },
            velocity_filter: low_pass("velocity_filter", "velocity_filter_s")?,
            torque_filter: low_pass("torque_filter", "torque_filter_s")?,
            notch,
            velocity_cycle,
        })
    }

    /// Torque limits (lowest, highest) at a speed: the current limit everywhere, and in the
    /// direction of rotation the torque-speed curve that the voltage limit leaves.
    pub fn torque_range(&self, speed: f64) -> (f64, f64) {
        if self.limit { self.torque_curve(speed) } else { (f64::NEG_INFINITY, f64::INFINITY) }
    }

    /// The same limits, whether or not the limit is switched on: what the motor and drive can give.
    pub fn torque_curve(&self, speed: f64) -> (f64, f64) {
        let fade = ((self.no_load_speed - speed.abs()) / (self.no_load_speed - self.rated_speed)).clamp(0.0, 1.0);
        let (peak, motoring) = (self.peak_torque, self.peak_torque * fade);
        if speed >= 0.0 { (-peak, motoring) } else { (-motoring, peak) }
    }
}

/// Biquad notch (b0, b1, b2, a1, a2), normalised by a0.
fn notch_coefficients(frequency: f64, bandwidth: f64, cycle: f64) -> [f64; 5] {
    let w0 = TAU * frequency * cycle;
    let alpha = w0.sin() / (2.0 * frequency / bandwidth);
    let a0 = 1.0 + alpha;
    [1.0 / a0, -2.0 * w0.cos() / a0, 1.0 / a0, -2.0 * w0.cos() / a0, (1.0 - alpha) / a0]
}

/// The loops and filters of one drive.
#[derive(Debug, Clone, Default)]
pub struct Drive {
    pub pos_set: f64,
    /// Speed setpoint at the last profile cycle, and its rate until the next one.
    pub vel_set: f64,
    pub acc_set: f64,
    /// Time since the profile generator last wrote the setpoints.
    pub since_setpoint: f64,
    pub torque_ff: f64,
    pub following_error: f64,
    pub speed_command: f64,
    pub torque_command: f64,
    pub torque: f64,
    /// Kv times following error, held between position-loop cycles.
    position_part: f64,
    integral: f64,
    encoder: f64,
    last_encoder: f64,
    speed: f64,
    filtered: f64,
    notch: [f64; 4],
}

impl Drive {
    /// A drive at rest at `pos`, already delivering `torque` (so a hanging axis does not sag at start).
    pub fn at_rest(p: &DriveParams, pos: f64, torque: f64, torque_ff: f64) -> Self {
        let encoder = quantise(pos, p.encoder_step);
        Self {
            pos_set: pos,
            torque_ff,
            torque_command: torque,
            torque,
            integral: torque - p.torque_ff * torque_ff,
            encoder,
            last_encoder: encoder,
            filtered: torque,
            notch: [torque; 4],
            ..Self::default()
        }
    }

    pub fn encoder(&self) -> f64 {
        self.encoder
    }

    /// Position loop: following error times Kv.
    pub fn position_loop(&mut self, p: &DriveParams, pos: f64) {
        self.following_error = self.pos_set - quantise(pos, p.encoder_step);
        self.position_part = p.kv * self.following_error;
        self.speed_command = self.position_part + p.velocity_ff * self.vel_set;
    }

    /// Velocity loop: PI on the measured speed, plus torque feedforward, then filters and limits.
    pub fn velocity_loop(&mut self, p: &DriveParams, pos: f64, true_speed: f64) {
        self.encoder = quantise(pos, p.encoder_step);
        // A real encoder gives positions only; the speed is their difference per cycle.
        let raw = if p.encoder_step > 0.0 { (self.encoder - self.last_encoder) / p.velocity_cycle } else { true_speed };
        self.last_encoder = self.encoder;
        self.speed += p.velocity_filter * (raw - self.speed);

        // Velocity feedforward, interpolated between profile cycles. Holding it for a whole profile
        // cycle would make the speed command a staircase and the torque a sawtooth.
        self.speed_command = self.position_part + p.velocity_ff * (self.vel_set + self.acc_set * self.since_setpoint);
        self.since_setpoint += p.velocity_cycle;

        let error = self.speed_command - self.speed;
        let wanted = p.kp * error + self.integral + p.torque_ff * self.torque_ff;
        self.filtered += p.torque_filter * (wanted - self.filtered);
        let mut out = self.filtered;
        if let Some([b0, b1, b2, a1, a2]) = p.notch {
            let [x1, x2, y1, y2] = self.notch;
            out = b0 * self.filtered + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
            self.notch = [self.filtered, x1, out, y1];
        }
        let (low, high) = p.torque_range(self.speed);
        self.torque_command = out.clamp(low, high);
        // Anti-windup: the integrator stops while it would push further into the limit.
        if (out <= high || error < 0.0) && (out >= low || error > 0.0) {
            self.integral += p.ki * error * p.velocity_cycle;
        }
    }

    /// Current loop as a first-order lag. Runs every base step.
    pub fn current_loop(&mut self, p: &DriveParams) {
        self.torque += p.current_gain * (self.torque_command - self.torque);
    }
}

fn quantise(pos: f64, step: f64) -> f64 {
    if step > 0.0 { (pos / step).round() * step } else { pos }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn params() -> DriveParams {
        DriveParams {
            kv: 50.0,
            kp: 0.08,
            ki: 8.0,
            velocity_ff: 0.0,
            torque_ff: 0.0,
            current_gain: 1.0 - (-62.5e-6f64 / 0.25e-3).exp(),
            peak_torque: 3.8,
            rated_speed: 314.0,
            no_load_speed: 420.0,
            limit: true,
            encoder_step: 0.0,
            velocity_filter: 1.0,
            torque_filter: 1.0,
            notch: None,
            velocity_cycle: 125e-6,
        }
    }

    #[test]
    fn position_loop_is_kv_times_following_error_plus_feedforward() {
        let mut p = params();
        let mut d = Drive { pos_set: 1.0, vel_set: 10.0, ..Drive::default() };
        d.position_loop(&p, 0.9);
        assert!((d.following_error - 0.1).abs() < 1e-15 && (d.speed_command - 5.0).abs() < 1e-12);
        p.velocity_ff = 1.0;
        d.position_loop(&p, 0.9);
        assert!((d.speed_command - 15.0).abs() < 1e-12);
    }

    #[test]
    fn velocity_loop_is_pi_with_torque_feedforward() {
        let mut p = params();
        p.torque_ff = 1.0;
        let mut d = Drive { position_part: 10.0, torque_ff: 0.3, ..Drive::default() };
        d.velocity_loop(&p, 0.0, 4.0);
        assert!((d.torque_command - (0.08 * 6.0 + 0.3)).abs() < 1e-12);
        d.velocity_loop(&p, 0.0, 4.0);
        assert!((d.torque_command - (0.08 * 6.0 + 8.0 * 6.0 * 125e-6 + 0.3)).abs() < 1e-12);
    }

    #[test]
    fn torque_limit_follows_the_torque_speed_curve() {
        let p = params();
        assert_eq!(p.torque_range(100.0), (-3.8, 3.8));
        // Half way between rated and no-load speed, half the torque is left for accelerating.
        let (low, high) = p.torque_range(367.0);
        assert!((high - 1.9).abs() < 1e-12 && low == -3.8);
        assert_eq!(p.torque_range(-500.0), (-0.0, 3.8));
        let mut d = Drive { position_part: 1000.0, ..Drive::default() };
        d.velocity_loop(&p, 0.0, 0.0);
        assert_eq!(d.torque_command, 3.8);
        let before = d.integral;
        d.velocity_loop(&p, 0.0, 0.0);
        assert_eq!(d.integral, before, "the integrator winds up at the limit");
        let unlimited = DriveParams { limit: false, ..params() };
        let mut d = Drive { position_part: 1000.0, ..Drive::default() };
        d.velocity_loop(&unlimited, 0.0, 0.0);
        assert!(d.torque_command > 3.8);
        assert_eq!(unlimited.torque_curve(100.0), (-3.8, 3.8));
    }

    #[test]
    fn velocity_feedforward_is_interpolated_between_profile_cycles() {
        let mut p = params();
        p.velocity_ff = 1.0;
        let mut d = Drive { vel_set: 10.0, acc_set: 1000.0, ..Drive::default() };
        d.velocity_loop(&p, 0.0, 0.0);
        assert!((d.speed_command - 10.0).abs() < 1e-12);
        d.velocity_loop(&p, 0.0, 0.0);
        assert!((d.speed_command - (10.0 + 1000.0 * 125e-6)).abs() < 1e-12);
    }

    #[test]
    fn current_loop_is_a_first_order_lag() {
        let p = params();
        let mut d = Drive { torque_command: 1.0, ..Drive::default() };
        // After one time constant (4 steps of 62.5 us at 0.25 ms) the torque is at 63.2 %.
        for _ in 0..4 {
            d.current_loop(&p);
        }
        assert!((d.torque - (1.0 - (-1f64).exp())).abs() < 1e-12);
    }

    #[test]
    fn encoder_resolution_quantises_position_and_speed() {
        let mut p = params();
        p.encoder_step = TAU / 4096.0;
        let mut d = Drive::at_rest(&p, 0.0, 0.0, 0.0);
        d.pos_set = 0.4 * p.encoder_step;
        d.position_loop(&p, 0.0);
        assert_eq!(d.following_error, 0.4 * p.encoder_step);
        d.position_loop(&p, 0.3 * p.encoder_step);
        assert_eq!(d.following_error, 0.4 * p.encoder_step, "a move below one count is invisible");
        // One count per velocity cycle reads as step / cycle, whatever the true speed is.
        d.velocity_loop(&p, 1.2 * p.encoder_step, 123.0);
        assert!((d.speed - p.encoder_step / 125e-6).abs() < 1e-9);
    }

    #[test]
    fn notch_removes_its_centre_frequency_and_passes_the_rest() {
        let gain_at = |frequency: f64| {
            let [b0, b1, b2, a1, a2] = notch_coefficients(60.0, 20.0, 125e-6);
            let (mut x, mut y, mut peak) = ([0.0; 2], [0.0; 2], 0f64);
            for n in 0..80_000 {
                let input = (TAU * frequency * n as f64 * 125e-6).sin();
                let out = b0 * input + b1 * x[0] + b2 * x[1] - a1 * y[0] - a2 * y[1];
                (x, y) = ([input, x[0]], [out, y[0]]);
                if n > 60_000 {
                    peak = peak.max(out.abs());
                }
            }
            peak
        };
        assert!(gain_at(60.0) < 0.01, "{}", gain_at(60.0));
        assert!((gain_at(5.0) - 1.0).abs() < 0.02 && (gain_at(600.0) - 1.0).abs() < 0.02);
    }

    #[test]
    fn a_drive_starts_holding_its_load() {
        let mut p = params();
        p.torque_ff = 1.0;
        let mut d = Drive::at_rest(&p, 2.0, 0.14, 0.137);
        d.position_loop(&p, 2.0);
        d.velocity_loop(&p, 2.0, 0.0);
        d.current_loop(&p);
        assert!((d.torque - 0.14).abs() < 1e-12 && (d.torque_command - 0.14).abs() < 1e-12);
    }
}
