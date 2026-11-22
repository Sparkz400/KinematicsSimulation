//! Stepper motor (motor.kind = "stepper"): a two-phase hybrid stepper behind an open-loop step and direction drive.
//!
//! The motor in its usual sinusoidal form, with N rotor teeth and the rotor's electrical angle e = N (angle - origin):
//!   torque = kt (i_b cos e - i_a sin e) - T_d sin 4e,
//!   L di_a/dt = u_a - R i_a + kE w sin e,   L di_b/dt = u_b - R i_b - kE w cos e,
//! kE = kt (the back-EMF powers the torque: e_a i_a + e_b i_b = torque w without the detent), the detent torque T_d
//! (effects.detent_torque) with 4N periods a turn. R, L and kt are a phase's (`motor.resistance_ohm`, `inductance_h`,
//! `torque_constant_nm_a`).
//!
//! The drive counts the position setpoint in microsteps, 2 pi / (4 N microsteps) each, and sets the current vector at
//! that electrical angle with the amplitude `drive.stepper_current_a`: i_a = I cos e_c, i_b = I sin e_c. Then
//! torque = kt I sin(e_c - e): the rotor sits on the "magnetic spring" of the holding torque kt I, its load angle
//! e_c - e. Beyond the pull-out angle pi / 2 the spring weakens; past pi the rotor falls into the next equilibrium, four
//! full steps on, and the drive, open loop, never learns of it.
//!
//! The chopper sets each base step the voltage that brings the phase currents to their setpoints by the step's end,
//! against the back-EMF at its start, as a current-regulated chopper does over its PWM periods; the voltage vector is
//! kept within the supply, the DC link voltage (`drive.bus_voltage_v`, the simulated link with effects.dc_link): a
//! two-phase bridge gives a phase +-U. So the current falls short of its setpoint at speed, and the torque with it: the
//! pull-out curve (`Stepper::pull_out`).

use crate::drive::{Drive, DriveMode, DriveParams, WINDING_PER_DC_LINK, quantise};
use sim_core::{Config, ConfigError};
use std::f64::consts::TAU;

type Result<T> = std::result::Result<T, ConfigError>;

#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Stepper {
    /// Rotor teeth N (50 for a 1.8 degree motor): 4 N full steps a turn.
    pub teeth: f64,
    /// effects.detent_torque: the detent torque's amplitude, N m; 0 off.
    pub detent: f64,
    /// One microstep at the motor, rad.
    pub microstep: f64,
    /// The current vector's amplitude the drive sets, A: sqrt 2 times a phase's current in a full step.
    pub current: f64,
}

impl Stepper {
    pub fn from_config(cfg: &Config) -> Result<Self> {
        let teeth = cfg.positive("motor.rotor_teeth")?;
        let microsteps = cfg.positive("drive.microsteps")?;
        if microsteps.fract() != 0.0 || microsteps > 4096.0 {
            return Err(ConfigError::Invalid("drive.microsteps".into()));
        }
        let detent = if cfg.get("effects.detent_torque").is_ok() && cfg.bool("effects.detent_torque")? {
            cfg.non_negative("motor.detent_torque_nm")?
        } else {
            0.0
        };
        let current = cfg.positive("drive.stepper_current_a")?;
        Ok(Self { teeth, detent, microstep: TAU / (4.0 * teeth * microsteps), current })
    }

    /// The torque of phase currents at the rotor's electrical angle `e`.
    pub fn torque(&self, p: &DriveParams, [ia, ib]: [f64; 2], e: f64) -> f64 {
        let (s, c) = e.sin_cos();
        p.kt * (ib * c - ia * s) - self.detent * (4.0 * e).sin()
    }

    /// The holding torque: kt times the current the drive sets.
    pub fn holding_torque(&self, p: &DriveParams) -> f64 {
        p.kt * self.current
    }

    /// The drive enabled at `pos`: the windings count from there, a full step, and the current vector stands on it.
    pub(crate) fn at_rest(&self, d: &mut Drive, p: &DriveParams, pos: f64) {
        // The first enable puts the electrical zero (and the detent's) where the rotor stands; later ones keep it
        // (`re_enabled`).
        let x = [self.current, 0.0, 0.0, 0.0];
        d.phases = crate::drive::Phases { x, angle: pos, command: pos, origin: pos, ..Default::default() };
        (d.current, d.torque, d.torque_demand, d.torque_command, d.controller_torque) =
            (self.current, 0.0, 0.0, 0.0, 0.0);
        d.phases.u = [p.resistance * self.current, 0.0];
    }

    /// A drive enabled again with the grid `origin` it had (the electrical zero, the detent's too): the rotor stands at `pos`,
    /// off the grid where it was turned meanwhile; the drive's count starts on the full step nearest to it, origin + n 2 pi /
    /// (4 N), and its current vector stands there (`offset` is the count's distance from the controller's setpoint, which
    /// starts at `pos`). The rotor is not moved: the magnetic spring pulls it there by the model's dynamics, from its load
    /// angle (at most a half full step) on.
    pub(crate) fn re_enabled(&self, d: &mut Drive, p: &DriveParams, pos: f64, origin: f64) {
        let full = TAU / (4.0 * self.teeth);
        let snapped = origin + ((pos - origin) / full).round() * full;
        let e = (self.teeth * (snapped - origin)).rem_euclid(TAU);
        let (s, c) = e.sin_cos();
        let ph = &mut d.phases;
        (ph.origin, ph.command, ph.offset, ph.electrical) = (origin, snapped, snapped - pos, e);
        ph.x = [self.current * c, self.current * s, 0.0, 0.0];
        ph.load_angle = e - self.teeth * (pos - origin);
        ph.u = [p.resistance * self.current * c, p.resistance * self.current * s];
    }

    /// The detent torque's potential at the rotor angle `angle`, J: the torque is -T_d sin 4e (e = N (angle - origin)), so
    /// U = -T_d cos(4e) / (4 N), whose angle derivative is T_d sin 4e. It is the magnets' energy (the torque is
    /// conservative), enabled or not: the detent's work is part of the shaft's, and what the magnets give for it, the
    /// change of U, is booked as it happens (`Drive::detent_moved`, `Drive::emf_loss`), never stored, so that nothing of
    /// it appears or vanishes at an enable or a disable.
    pub fn detent_potential(&self, angle: f64, origin: f64) -> f64 {
        -self.detent * (4.0 * self.teeth * (angle - origin)).cos() / (4.0 * self.teeth)
    }

    /// The detent torque at the rotor angle `angle`, N m: -T_d sin 4e, the magnets' alone, with the drive enabled or not.
    pub fn detent_torque(&self, angle: f64, origin: f64) -> f64 {
        -self.detent * (4.0 * self.teeth * (angle - origin)).sin()
    }

    /// The rates of the windings (`DriveParams::motor_rates`): i_a in slot 0, i_b in slot 4.
    pub(crate) fn rates(&self, p: &DriveParams, d: &Drive, w: &[f64], angle: f64, speed: f64, out: &mut [f64]) -> f64 {
        let ([ia, ib], [ua, ub]) = ([w[0], w[4]], d.phases.u);
        let e = self.teeth * (angle - d.phases.origin);
        let (s, c) = e.sin_cos();
        let (r, emf) = (d.winding_resistance(p), p.ke * speed);
        let electromagnetic = p.kt * (ib * c - ia * s);
        out[0] = (ua - r * ia + emf * s) / p.inductance;
        out[4] = (ub - r * ib - emf * c) / p.inductance;
        out[1] = ua * ia + ub * ib;
        out[2] = r * (ia * ia + ib * ib);
        // The shaft's work is the whole torque's; the detent's share of it the magnets give (`Drive::detent_moved`).
        let torque = electromagnetic + self.detent_torque(angle, d.phases.origin);
        out[3] = torque * speed;
        out[5] = 0.0;
        out[6] = 0.0;
        torque
    }

    /// One base step of the drive: the step generator's angle, the current vector on its microstep, and the chopper's
    /// voltages for the step.
    pub(crate) fn control(&self, d: &mut Drive, p: &DriveParams) {
        let dt = p.current_cycle;
        // The step generator runs the setpoint on between profile cycles; a speed setpoint (velocity mode, a fault's
        // stop ramp) it integrates.
        d.phases.command = match (d.fault_speed, d.mode()) {
            (None, DriveMode::Position) => {
                let t = d.since_setpoint;
                d.pos_set + d.phases.offset + (d.vel_set + 0.5 * d.acc_set * t) * t
            }
            _ => d.phases.command + d.speed_command * dt,
        };
        let (r, supply) = (d.winding_resistance(p), d.max_voltage(p) / WINDING_PER_DC_LINK);
        let ph = &mut d.phases;
        ph.electrical = self.teeth * quantise(ph.command - ph.origin, self.microstep);
        let set = [self.current * ph.electrical.cos(), self.current * ph.electrical.sin()];
        // The voltage that takes i to its setpoint over the step, i(dt) = i_inf + (i - i_inf) a, a = exp(-R dt / L),
        // with i_inf = (u - emf) / R and the back-EMF of the step's start.
        let e = self.teeth * (ph.angle - ph.origin);
        let emf = [-p.ke * ph.speed * e.sin(), p.ke * ph.speed * e.cos()];
        let a = (-r * dt / p.inductance).exp();
        let mut u = [0.0; 2];
        for k in 0..2 {
            u[k] = emf[k] + r * (set[k] - a * ph.x[k]) / (1.0 - a);
        }
        // The vector is held within the supply: `drive.overmodulation` (`DriveParams::fundamental_gain`) lets a bridge driven
        // into square waves give 4 / pi more, the fundamental only.
        let norm = u[0].hypot(u[1]);
        if norm > supply {
            u = u.map(|v| v * supply / norm);
        }
        ph.u = u;
        d.voltage = norm.min(supply);
        d.iron_loss = 0.0;
    }

    /// The step done: the current vector's amplitude, the torque at the rotor's angle and the load angle.
    pub(crate) fn stepped(&self, d: &mut Drive, p: &DriveParams) {
        let ph = &mut d.phases;
        let e = self.teeth * (ph.angle - ph.origin);
        let i = [ph.x[0], ph.x[1]];
        ph.load_angle = ph.electrical - e;
        d.current = i[0].hypot(i[1]);
        d.torque = self.torque(p, i, e);
        // Open loop nothing asks for a torque: what it gives is judged against the pull-out curve.
        (d.torque_demand, d.torque_command) = (d.torque, d.torque);
    }

    /// The pull-out torque at a speed, motoring, with the supply `supply` (infinite: no limit): the most torque a
    /// steady run at that speed can have from currents within the drive's amplitude I and voltages within the supply.
    /// In phasors at the electrical frequency N w, with the back-EMF E = kE w as the real axis and Z = R + j N w L, the
    /// torque is kt Re(I) and V = Z I + E: Re(I) at its highest over the intersection of the disc |I| <= I and the disc
    /// |I + E / Z| <= U / |Z|. Without detent torque, cold winding.
    pub fn pull_out(&self, p: &DriveParams, speed: f64, supply: f64) -> f64 {
        p.kt * most_in_phase(p, self, speed.abs(), supply, 1.0)
    }

    /// The braking counterpart of `pull_out`: the back-EMF helps the current.
    pub fn braking(&self, p: &DriveParams, speed: f64, supply: f64) -> f64 {
        p.kt * most_in_phase(p, self, speed.abs(), supply, -1.0)
    }

    /// (lowest, highest) torque at a speed, as `DriveParams::torque_curve`: the pull-out curve and its braking side.
    pub fn torque_curve(&self, p: &DriveParams, speed: f64, supply: f64) -> (f64, f64) {
        let (motoring, braking) = (self.pull_out(p, speed, supply), self.braking(p, speed, supply));
        if speed >= 0.0 { (-braking, motoring) } else { (-motoring, braking) }
    }
}

/// The highest Re(I) over |I| <= I_max and |Z I + sign E| <= U (see `Stepper::pull_out`).
fn most_in_phase(p: &DriveParams, s: &Stepper, speed: f64, supply: f64, sign: f64) -> f64 {
    let (limit, teeth) = (s.current, s.teeth);
    if !supply.is_finite() {
        return limit;
    }
    let (r, x, e) = (p.resistance, teeth * speed * p.inductance, sign * p.ke * speed);
    let z2 = r * r + x * x;
    // The voltage disc: centre -E / Z = -E (R - jX) / |Z|^2, radius U / |Z|.
    let (cx, cy, rho) = (-e * r / z2, e * x / z2, supply / z2.sqrt());
    let inside_voltage = |px: f64, py: f64| (px - cx).hypot(py - cy) <= rho * (1.0 + 1e-12);
    let mut best = f64::NEG_INFINITY;
    if inside_voltage(limit, 0.0) {
        best = limit;
    }
    if (cx + rho).hypot(cy) <= limit {
        best = best.max(cx + rho);
    }
    // Where the two circles cross.
    let d = cx.hypot(cy);
    if d > 0.0 && d <= limit + rho && d >= (limit - rho).abs() {
        let a = (limit * limit - rho * rho + d * d) / (2.0 * d);
        let h = (limit * limit - a * a).max(0.0).sqrt();
        // Their real parts: the chord's middle a along the centre line, h either way across it.
        let mx = a * cx / d;
        best = best.max(mx + h * cy / d).max(mx - h * cy / d);
    }
    best.max(0.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::drive::{Motor, WINDING_STATES};
    use sim_core::Rk4;

    /// A NEMA 23 sized stepper of round numbers: N = 50, kt = kE = 0.5 N m/A, R = 1 Ohm, L = 2.5 mH, driven at 2 A on
    /// 24 V: holding torque 1 N m. No detent unless a test sets it.
    fn params() -> DriveParams {
        DriveParams {
            kt: 0.5,
            ke: 0.5,
            resistance: 1.0,
            inductance: 2.5e-3,
            rated_current: 2.0,
            heating: false,
            dc_link: false,
            bus_voltage: 24.0,
            max_voltage: WINDING_PER_DC_LINK * 24.0,
            current_cycle: 62.5e-6,
            motor: Motor::Stepper(Stepper {
                teeth: 50.0,
                detent: 0.0,
                microstep: TAU / (4.0 * 50.0 * 16.0),
                current: 2.0,
            }),
            ..crate::drive::tests::params()
        }
    }

    fn stepper(p: &DriveParams) -> &Stepper {
        let Motor::Stepper(s) = &p.motor else { unreachable!() };
        s
    }

    /// The motor on a rigid inertia `j` with viscous friction `b` and a load torque `load(t)`, its drive commanding
    /// `set(t)` = (position, speed, acceleration) every millisecond, integrated as `Sim::step` does: windings and
    /// mechanics through each base step together. Returns per base step (time, angle, speed, load angle, current).
    fn run(
        p: &DriveParams,
        j: f64,
        b: f64,
        load: impl Fn(f64) -> f64,
        set: impl Fn(f64) -> [f64; 3],
        seconds: f64,
    ) -> Vec<[f64; 5]> {
        let s = stepper(p);
        let mut d = Drive::at_rest(p, 0.0, 0.0, 0.0);
        let dt = p.current_cycle;
        let mut rk = Rk4::new(2 + WINDING_STATES);
        let (mut angle, mut speed) = (0.0, 0.0);
        let mut out = Vec::new();
        let steps = (seconds / dt).round() as usize;
        for n in 0..steps {
            let t = n as f64 * dt;
            if n % 16 == 0 {
                [d.pos_set, d.vel_set, d.acc_set] = set(t);
                d.since_setpoint = 0.0;
            }
            s.control(&mut d, p);
            d.since_setpoint += dt;
            let mut y = vec![angle, speed];
            y.extend(d.winding_states(p));
            let tl = load(t);
            rk.step(
                |y, dy| {
                    let torque = p.motor_rates(&d, &y[2..], y[0], y[1], &mut dy[2..]);
                    dy[0] = y[1];
                    dy[1] = (torque - b * y[1] - tl) / j;
                },
                &mut y,
                dt,
            );
            (angle, speed) = (y[0], y[1]);
            let w = &y[2..];
            (d.phases.x, d.phases.angle, d.phases.speed) = ([w[0], w[4], w[5], w[6]], angle, speed);
            d.winding_stepped(p, w[0], [w[1], w[2], w[3]]);
            out.push([t + dt, angle, speed, d.phases.load_angle, d.current]);
        }
        out
    }

    /// Holding torque by hand: kt I = 0.5 * 2 = 1 N m at the rated current, at a load angle of 90 degrees (one full step,
    /// a quarter of an electrical turn, pi / 100 rad at the motor). The drive's currents settle at their setpoints.
    #[test]
    fn the_holding_torque_is_kt_times_the_rated_current() {
        let p = params();
        let s = stepper(&p);
        assert!((s.holding_torque(&p) - 1.0).abs() < 1e-12);
        let full_step = TAU / 200.0;
        assert!((s.torque(&p, [2.0, 0.0], -50.0 * full_step) - 1.0).abs() < 1e-12);
        // Against a load of 0.99 N m held at rest the rotor stands at asin(0.99) electrical behind its setpoint, by hand
        // 81.9 degrees, 0.02858 rad at the motor; the load comes on over 0.2 s and viscous damping lets it settle.
        let trace = run(&p, 1e-4, 2e-3, |t| 0.99 * (t / 0.2).min(1.0), |_| [0.0; 3], 0.6);
        let [_, angle, _, load_angle, current] = *trace.last().unwrap();
        assert!((load_angle - 0.99f64.asin()).abs() < 2e-3, "{load_angle}");
        assert!((angle + 0.99f64.asin() / 50.0).abs() < 1e-4, "{angle}");
        assert!((current - 2.0).abs() < 1e-6, "{current}");
    }

    /// The rotor on the magnetic spring: stiffness N T_hold = 50 N m/rad at small angles, so with J = 1e-4 kg m^2 it rings
    /// at sqrt(50 / 1e-4) / 2 pi = 112.54 Hz after a microstep, undamped (the chopper takes the back-EMF out of the loop).
    #[test]
    fn the_rotor_rings_at_the_hand_calculated_resonance() {
        let p = params();
        let step = stepper(&p).microstep;
        let trace = run(&p, 1e-4, 0.0, |_| 0.0, |_| [1.2 * step, 0.0, 0.0], 0.1);
        // Crossings of the setpoint, rising.
        let target = step;
        let ups: Vec<f64> =
            trace.windows(2).filter(|w| w[0][1] < target && w[1][1] >= target).map(|w| w[1][0]).collect();
        assert!(ups.len() > 8, "{}", ups.len());
        let f = (ups.len() - 1) as f64 / (ups.last().unwrap() - ups[0]);
        let by_hand = (50.0 * 1.0 / 1e-4f64).sqrt() / TAU;
        assert!((f - by_hand).abs() < 0.01 * by_hand, "{f} Hz against {by_hand} Hz");
    }

    /// The corner speeds by hand. Unloaded the rotor runs with its load angle near 0, so the back-EMF stands in phase with
    /// L di/dt and a phase needs sqrt((R I)^2 + (N w L I + kE w)^2) volts. It reaches the 24 V supply at
    /// w_c = sqrt(24^2 - 2^2) / (50 * 2.5e-3 * 2 + 0.5) = 31.89 rad/s (305 rpm): below it the current reaches its
    /// setpoint, above it falls short. At the pull-out angle (90 degrees) the back-EMF stands in phase with the current,
    /// and a phase needs sqrt((R I + kE w)^2 + (N w L I)^2): 24 V at 0.3125 w^2 + 2 w - 572 = 0, w = 39.70 rad/s, where the
    /// pull-out torque starts to fall from the holding torque (`pull_out`).
    #[test]
    fn the_current_falls_short_above_the_hand_calculated_corner_speed() {
        let p = params();
        let corner = (24.0f64 * 24.0 - 4.0).sqrt() / (50.0 * 2.5e-3 * 2.0 + 0.5);
        assert!((corner - 31.89).abs() < 0.01);
        // At a constant speed, after a slow ramp up to it, the current amplitude over the last 0.1 s.
        let amplitude = |speed: f64| {
            let ramp = 0.5;
            let set = move |t: f64| {
                if t < ramp {
                    [0.5 * speed / ramp * t * t, speed / ramp * t, speed / ramp]
                } else {
                    [0.5 * speed * ramp + speed * (t - ramp), speed, 0.0]
                }
            };
            let trace = run(&p, 1e-4, 1e-4, |_| 0.0, set, ramp + 0.3);
            trace[trace.len() - 1600..].iter().map(|r| r[4]).fold(f64::INFINITY, f64::min)
        };
        let (below, above) = (amplitude(0.9 * corner), amplitude(1.3 * corner));
        assert!(below > 0.99 * 2.0, "{below} A at 0.9 w_c");
        assert!(above < 0.95 * 2.0, "{above} A at 1.3 w_c");
        let s = stepper(&p);
        let torque_corner = (-2.0 + (4.0f64 + 4.0 * 0.3125 * 572.0).sqrt()) / (2.0 * 0.3125);
        assert!((torque_corner - 39.70).abs() < 0.01);
        assert!((s.pull_out(&p, 0.999 * torque_corner, 24.0) - 1.0).abs() < 1e-12);
        let past = s.pull_out(&p, 1.1 * torque_corner, 24.0);
        assert!(past < 0.98, "{past}");
    }

    /// A load above the holding torque, 1.3 N m for 30 ms, pulls the rotor out of its equilibrium; once it is gone the
    /// rotor settles on another, a whole electrical turn on, 2 pi / N: four full steps per turn (detent torque on, which
    /// has its equilibria at every full step, and it still lands on the four-step grid).
    #[test]
    fn a_load_above_the_holding_torque_slips_by_whole_multiples_of_four_full_steps() {
        let mut p = params();
        if let Motor::Stepper(s) = &mut p.motor {
            s.detent = 0.05;
        }
        let trace = run(&p, 1e-4, 5e-3, |t| if t < 0.03 { 1.3 } else { 0.0 }, |_| [0.0; 3], 0.4);
        let [_, angle, speed, load_angle, _] = *trace.last().unwrap();
        let full_steps = -angle / (TAU / 200.0);
        assert!(speed.abs() < 1e-2, "{speed}");
        assert!(full_steps >= 3.9, "{full_steps} full steps");
        assert!((full_steps / 4.0 - (full_steps / 4.0).round()).abs() < 0.01, "{full_steps} full steps");
        assert!((load_angle / TAU - (load_angle / TAU).round()).abs() < 0.01 && load_angle > 1.0, "{load_angle}");
    }

    /// Acceleration ramps, jerk-limited (the acceleration rises over 0.2 s, two periods of the rotor on its spring), on
    /// a load inertia of 0.01 kg m^2: one that needs 80 % of the pull-out torque at its end speed stays in step, one that
    /// needs 110 % falls out. Below the corner speed (to 20 rad/s) the pull-out torque is the holding torque, 1 N m:
    /// 80 rad/s^2 and 110 rad/s^2. Above it (to 55 rad/s) the pull-out curve's value there (`pull_out`), by hand 0.793 N m:
    /// E = 27.5 V, X = 6.875 Ohm, the voltage disc's centre (-0.570, 3.917) A and radius 3.454 A cross the 2 A circle at
    /// Re I = 1.586 A.
    #[test]
    fn a_ramp_beyond_the_pull_out_torque_stalls_and_one_at_80_percent_does_not() {
        let p = params();
        let s = stepper(&p);
        let j = 0.01;
        let stays_in_step = |torque: f64, end: f64| {
            let (alpha, rise) = (torque / j, 0.2);
            let set = move |t: f64| {
                // Acceleration alpha * min(t / rise, 1).
                if t < rise {
                    [alpha * t.powi(3) / (6.0 * rise), alpha * t * t / (2.0 * rise), alpha * t / rise]
                } else {
                    let (x0, v0) = (alpha * rise * rise / 6.0, alpha * rise / 2.0);
                    let tt = t - rise;
                    [x0 + v0 * tt + 0.5 * alpha * tt * tt, v0 + alpha * tt, alpha]
                }
            };
            // Until the setpoint's speed reaches `end`.
            let seconds = 0.5 * rise + end / alpha;
            let trace = run(&p, j, 0.0, |_| 0.0, set, seconds);
            let reached = trace.iter().take_while(|r| r[3].abs() < std::f64::consts::PI).last().unwrap()[2];
            (trace.iter().all(|r| r[3].abs() < std::f64::consts::PI), reached)
        };
        let below = s.pull_out(&p, 20.0, 24.0);
        assert!((below - 1.0).abs() < 1e-12);
        assert!(stays_in_step(0.8 * below, 20.0).0);
        assert!(!stays_in_step(1.1 * below, 20.0).0);
        let above = s.pull_out(&p, 55.0, 24.0);
        assert!((above - 0.793).abs() < 1e-3, "{above}");
        assert!(stays_in_step(0.8 * above, 55.0).0);
        let (kept, reached) = stays_in_step(1.1 * above, 55.0);
        assert!(!kept && reached < 55.0, "fell out at {reached} rad/s");
    }

    /// Disabled at a half step, enabled again: the grid (origin 0.3 rad here) is kept and the rotor, standing 1.4 full steps
    /// off the origin, is pulled by the magnetic spring to the nearest full step of that grid, origin + 1 full step
    /// (2 pi / 200 = 0.0314159 rad): after the ringing has died down (damping b / 2J = 10 / s, 1 s) the angle is
    /// 0.3 + 0.0314159 within 1e-4 rad, a whole number of full steps from the origin. The drive's count is 0.4 full steps
    /// behind the controller's setpoint, which holds where the rotor stood.
    #[test]
    fn an_enable_keeps_the_grid_and_pulls_the_rotor_to_the_nearest_full_step() {
        let p = params();
        let s = stepper(&p);
        let (origin, full) = (0.3, TAU / 200.0);
        let pos = origin + 1.4 * full;
        let mut d = Drive::at_rest(&p, pos, 0.0, 0.0);
        s.re_enabled(&mut d, &p, pos, origin);
        assert!((d.phases.command - (origin + full)).abs() < 1e-12 && (d.phases.offset + 0.4 * full).abs() < 1e-12);
        let (j, b, dt) = (1e-4, 2e-3, p.current_cycle);
        let mut rk = Rk4::new(2 + WINDING_STATES);
        let (mut angle, mut speed) = (pos, 0.0);
        for n in 0..16_000 {
            if n % 16 == 0 {
                (d.pos_set, d.vel_set, d.acc_set, d.since_setpoint) = (pos, 0.0, 0.0, 0.0);
            }
            s.control(&mut d, &p);
            d.since_setpoint += dt;
            let mut y = vec![angle, speed];
            y.extend(d.winding_states(&p));
            rk.step(
                |y, dy| {
                    let torque = p.motor_rates(&d, &y[2..], y[0], y[1], &mut dy[2..]);
                    (dy[0], dy[1]) = (y[1], (torque - b * y[1]) / j);
                },
                &mut y,
                dt,
            );
            (angle, speed) = (y[0], y[1]);
            let w = &y[2..];
            (d.phases.x, d.phases.angle, d.phases.speed) = ([w[0], w[4], w[5], w[6]], angle, speed);
            d.winding_stepped(&p, w[0], [w[1], w[2], w[3]]);
        }
        assert!((angle - (origin + full)).abs() < 1e-4, "{angle} rad against {}", origin + full);
        assert!(((angle - origin) / full - 1.0).abs() < 4e-3, "{} full steps", (angle - origin) / full);
    }

    /// The detent's work inside one base step. The torque -T_d sin 4e is conservative with the potential
    /// U = -T_d cos(4 N (angle - origin)) / (4 N) (dU/d angle = T_d sin 4e, so -dU/d angle is the torque). The shaft's
    /// work is the whole torque's, the detent's included, and the magnets give the detent's: `emf_loss` over the step is
    /// the change of U (`Drive::detent_moved`). Over a step electrical power - copper loss - the change of the winding's
    /// energy - the shaft's work is 0 (the electrical side), the shaft's work is the rotor's kinetic energy (no friction,
    /// no load), and so electrical - copper - winding - the magnets' - kinetic is 0, up to the integrator. T_d = 0.05 N m,
    /// N = 50: the potential swings by 0.05 / 200 = 2.5 mJ over a quarter of the detent period; the step's change of it
    /// is by hand U(angle end) - U(0.004), which the integrated shaft work beyond the electrical's matches.
    #[test]
    fn the_budget_closes_inside_one_step_with_the_detents_potential() {
        let mut p = params();
        if let Motor::Stepper(s) = &mut p.motor {
            s.detent = 0.05;
        }
        let s = stepper(&p);
        let (j, dt) = (1e-4, p.current_cycle);
        let mut d = Drive::at_rest(&p, 0.0, 0.0, 0.0);
        let (angle0, speed0) = (0.004, 8.0);
        (d.phases.angle, d.phases.speed) = (angle0, speed0);
        s.control(&mut d, &p);
        let before = d.magnetic_energy(&p);
        let mut rk = Rk4::new(2 + WINDING_STATES);
        let mut y = vec![angle0, speed0];
        y.extend(d.winding_states(&p));
        rk.step(
            |y, dy| {
                let torque = p.motor_rates(&d, &y[2..], y[0], y[1], &mut dy[2..]);
                (dy[0], dy[1]) = (y[1], torque / j);
            },
            &mut y,
            dt,
        );
        let w = &y[2..];
        (d.phases.x, d.phases.angle, d.phases.speed) = ([w[0], w[4], w[5], w[6]], y[0], y[1]);
        d.winding_stepped(&p, w[0], [w[1], w[2], w[3]]);
        d.detent_moved(&p, angle0);
        let after = d.magnetic_energy(&p);
        let kinetic = 0.5 * j * (y[1] * y[1] - speed0 * speed0);
        let magnets = d.emf_loss * dt;
        let by_hand = 0.05 / 200.0 * ((200.0 * angle0).cos() - (200.0 * y[0]).cos());
        assert!((magnets - by_hand).abs() < 1e-15 && magnets.abs() > 1e-6, "{magnets} J against {by_hand} J");
        let electrical = w[1] - w[2] - (after - before) - w[3] - magnets;
        assert!(electrical.abs() < 1e-9, "{electrical} J of the electrical side open of {} J", w[1]);
        let open = w[1] - w[2] - (after - before) - magnets - kinetic;
        assert!(open.abs() < 1e-9, "{open} J unaccounted of {} J electrical", w[1]);
        // The potential itself: -T_d / 200 at a detent minimum (4e = 0), +T_d / 200 half a period on.
        assert!((s.detent_potential(0.0, 0.0) + 0.05 / 200.0).abs() < 1e-15);
        assert!((s.detent_potential(TAU / 400.0, 0.0) - 0.05 / 200.0).abs() < 1e-15);
    }
}
