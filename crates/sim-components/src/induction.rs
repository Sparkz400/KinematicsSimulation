//! Induction motor on a frequency converter (motor.kind = "induction"): the standard d-q model, V/f control (open loop)
//! or field-oriented control (rotor flux, under the drive's speed and position loops).
//!
//! The motor in a frame turning at the electrical speed w_k the converter sets (amplitude-invariant space vectors: a
//! vector's length is a phase's amplitude), with stator and rotor flux as states, p pole pairs and the rotor's
//! electrical speed w_r = p w:
//!   dpsi_s/dt = u_s - R_s i_s - j w_k psi_s,   dpsi_r/dt = -R_r i_r - j (w_k - w_r) psi_r,
//!   psi_s = L_s i_s + L_m i_r,   psi_r = L_m i_s + L_r i_r,   L_s = L_m + L_ss,  L_r = L_m + L_sr,
//!   torque = 3/2 p (psi_s x i_s),   power 3/2 u_s . i_s,   copper loss 3/2 (R_s |i_s|^2 + R_r |i_r|^2).
//! R_s is `motor.resistance_ohm` (a phase's, warming with effects.winding_heating), the rest the motor's own keys. The
//! drive's `current` is the RMS phase current |i_s| / sqrt 2. The phase voltage's amplitude is at most U_dc / sqrt 3.
//!
//! V/f (`drive.induction_control = "vf"`): the frequency follows the speed setpoint, w_k = p w_set (plus the rated slip
//! times the estimated torque over the rated torque with `drive.vf_slip_compensation`), the voltage rises with it from
//! the boost `drive.vf_boost_v` at 0 Hz to the rated voltage at the rated frequency and stays there above (field
//! weakening by itself). The rotor slips behind: the torque-slip curve of the equivalent circuit (Kloss).
//!
//! Field-oriented (`"foc"`): the drive models the rotor flux, dpsi/dt = (R_r / L_r) (L_m i_sd - psi), and turns its
//! frame with it, w_k = w_r + R_r L_m i_sq / (L_r psi) (indirect: slip from the model). The flux setpoint is the rated
//! one up to the rated frequency and falls with 1 / speed above (field weakening), and lower only where the converter's
//! voltage U_dc / sqrt 3 and the current limit leave no room for the torque asked for at it (`control_flux`: the
//! largest flux that does). The curve, the limit, takes the flux of the most torque, torque ~ psi i_q within the two
//! circles (`flux_setpoint`); asked for the curve's torque the control sets that flux too, so that `torque_curve` is
//! the torque the control delivers; the q current is held within what the flux leaves of the voltage circle
//! (`voltage_iq`); torque = 3/2 p (L_m / L_r) psi i_sq.
//! PI current controllers on i_sd and i_sq with the decoupling the model gives: sigma L_s di/dt = u - R_sigma i + ...,
//! sigma = 1 - L_m^2 / (L_s L_r), R_sigma = R_s + (L_m / L_r)^2 R_r; tuned with Kp = sigma L_s / T and
//! Tn = sigma L_s / R_sigma the current follows its setpoint with T. The speed and position loops above are the servo's.

use crate::drive::{Drive, DriveParams, Phases};
use sim_core::{Config, ConfigError};
use std::f64::consts::{SQRT_2, TAU};

type Result<T> = std::result::Result<T, ConfigError>;

#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Induction {
    pub pole_pairs: f64,
    pub rotor_resistance: f64,
    pub magnetizing: f64,
    /// L_s and L_r, magnetizing plus leakage.
    pub stator: f64,
    pub rotor: f64,
    /// Rated phase voltage's amplitude, rated electrical speed (2 pi f_N), rated slip's electrical speed.
    pub rated_voltage: f64,
    pub rated_frequency: f64,
    pub rated_slip: f64,
    /// Field-oriented control (`"foc"`), else V/f.
    pub field_oriented: bool,
    /// V/f: the boost's phase amplitude at 0 Hz, and whether the slip is compensated.
    pub boost: f64,
    pub slip_compensation: bool,
    /// The rotor flux at the rated voltage and frequency, unloaded: the field-oriented drive's setpoint.
    pub rated_flux: f64,
}

/// Phase amplitude per line-to-line RMS voltage: sqrt 2 / sqrt 3.
const PHASE_PER_LINE: f64 = 0.816_496_580_927_726;

impl Induction {
    pub fn from_config(cfg: &Config) -> Result<Self> {
        let pole_pairs = cfg.positive("motor.pole_pairs")?;
        let magnetizing = cfg.positive("motor.magnetizing_inductance_h")?;
        let rated_frequency = TAU * cfg.positive("motor.rated_frequency_hz")?;
        let field_oriented = match cfg.get("drive.induction_control").ok().and_then(|v| v.as_str()) {
            Some("foc") => true,
            Some("vf") => false,
            _ => return Err(ConfigError::Invalid("drive.induction_control".into())),
        };
        // The name-plate speed against the synchronous one: the rated slip.
        let rated_slip = rated_frequency - pole_pairs * cfg.positive("drive.rated_speed_rad_s")?;
        if !(rated_slip > 0.0) {
            return Err(ConfigError::Invalid("drive.rated_speed_rad_s".into()));
        }
        let mut m = Self {
            pole_pairs,
            rotor_resistance: cfg.positive("motor.rotor_resistance_ohm")?,
            magnetizing,
            stator: magnetizing + cfg.positive("motor.stator_leakage_inductance_h")?,
            rotor: magnetizing + cfg.positive("motor.rotor_leakage_inductance_h")?,
            rated_voltage: PHASE_PER_LINE * cfg.positive("motor.rated_voltage_v")?,
            rated_frequency,
            rated_slip,
            field_oriented,
            boost: PHASE_PER_LINE * cfg.non_negative("drive.vf_boost_v")?,
            slip_compensation: cfg.bool("drive.vf_slip_compensation")?,
            rated_flux: 0.0,
        };
        let rs = cfg.positive("motor.resistance_ohm")?;
        m.rated_flux = magnetizing * m.rated_voltage / rs.hypot(rated_frequency * m.stator);
        Ok(m)
    }

    /// The model's currents from its fluxes [psi_sd, psi_sq, psi_rd, psi_rq]: (i_s, i_r) as [d, q].
    pub fn currents(&self, x: &[f64; 4]) -> ([f64; 2], [f64; 2]) {
        let (ls, lr, lm) = (self.stator, self.rotor, self.magnetizing);
        let det = ls * lr - lm * lm;
        let is = [(lr * x[0] - lm * x[2]) / det, (lr * x[1] - lm * x[3]) / det];
        let ir = [(ls * x[2] - lm * x[0]) / det, (ls * x[3] - lm * x[1]) / det];
        (is, ir)
    }

    /// The torque of the fluxes: 3/2 p (psi_s x i_s).
    pub fn torque(&self, x: &[f64; 4]) -> f64 {
        let (is, _) = self.currents(x);
        1.5 * self.pole_pairs * (x[0] * is[1] - x[1] * is[0])
    }

    /// The magnetic energy, 3/4 (psi_s . i_s + psi_r . i_r).
    pub fn stored_energy(&self, x: &[f64; 4]) -> f64 {
        let (is, ir) = self.currents(x);
        0.75 * (x[0] * is[0] + x[1] * is[1] + x[2] * ir[0] + x[3] * ir[1])
    }

    fn sigma_ls(&self) -> f64 {
        self.stator - self.magnetizing * self.magnetizing / self.rotor
    }

    /// The highest phase amplitude the converter gives: U_dc / sqrt 3 (`Drive::max_voltage` is sqrt(3/2) U_dc).
    fn supply(d: &Drive, p: &DriveParams) -> f64 {
        d.max_voltage(p) / crate::drive::WINDING_PER_DC_LINK / 3f64.sqrt()
    }

    /// The flux setpoint at a shaft speed, for the stator resistance `rs`, the phase amplitude `supply` the converter has
    /// and the phase current's amplitude limit `amplitude`: rated up to the rated frequency, then falling with 1 / speed
    /// (field weakening by frequency), and, where the two circles leave more torque at a weaker flux, the flux of the most
    /// torque. The torque is 3/2 p (L_m / L_r) psi i_q, so the flux psi maximises psi i_q(psi), i_q the most the current
    /// circle (i_d = psi / L_m) and the voltage circle (`voltage_iq`, motoring) leave at it. That product rises and falls
    /// once over the flux (a golden section finds its top); where the rated flux is the top (the voltage not binding, and
    /// i_d at most the current's amplitude over sqrt 2) it is returned without a search.
    fn flux_setpoint(&self, speed: f64, supply: f64, rs: f64, amplitude: f64) -> f64 {
        let w_e = self.pole_pairs * speed.abs();
        let cap = self.flux_cap(w_e);
        let product = |flux: f64| self.torque_room(rs, supply, flux, w_e, true, amplitude);
        let (i_d, limited) = (cap / self.magnetizing, self.iq_by_current(cap, amplitude));
        if i_d * SQRT_2 <= amplitude && self.voltage_iq(rs, supply, cap, w_e, true, limited) >= limited {
            return cap;
        }
        let g = 0.5 * (5f64.sqrt() - 1.0);
        let (mut lo, mut hi) = (0.05 * cap, cap);
        let (mut x1, mut x2) = (hi - g * (hi - lo), lo + g * (hi - lo));
        let (mut f1, mut f2) = (product(x1), product(x2));
        for _ in 0..28 {
            if f1 < f2 {
                (lo, x1, f1) = (x1, x2, f2);
                x2 = lo + g * (hi - lo);
                f2 = product(x2);
            } else {
                (hi, x2, f2) = (x2, x1, f1);
                x1 = hi - g * (hi - lo);
                f1 = product(x1);
            }
        }
        if f1 >= f2 { x1 } else { x2 }
    }

    /// The rated flux up to the rated frequency, falling with 1 / the electrical speed `w_e` above it.
    fn flux_cap(&self, w_e: f64) -> f64 {
        self.rated_flux * (self.rated_frequency / w_e).min(1.0)
    }

    /// The q current's amplitude the current circle `amplitude` leaves at the flux `flux` (i_d = psi / L_m).
    fn iq_by_current(&self, flux: f64, amplitude: f64) -> f64 {
        (amplitude * amplitude - (flux / self.magnetizing).powi(2)).max(0.0).sqrt()
    }

    /// psi i_q at the flux `flux`, i_q the most both circles leave (motoring or generating): the torque over 3/2 p L_m / L_r.
    fn torque_room(&self, rs: f64, supply: f64, flux: f64, w_r: f64, motoring: bool, amplitude: f64) -> f64 {
        flux * self.voltage_iq(rs, supply, flux, w_r, motoring, self.iq_by_current(flux, amplitude))
    }

    /// The flux the control sets for the torque `torque` at a shaft speed: as a field-oriented drive does, the rated one
    /// (`flux_cap` above the rated frequency) while the voltage and current circles leave room for that torque there, else
    /// the largest weaker flux that does (between the cap and the flux of the most torque, psi i_q falls with the flux,
    /// so a bracketed secant finds it). Beyond what any flux gives, the flux of the most torque, `flux_setpoint`: the
    /// curve's flux, so that at the curve's torque the control and the curve meet.
    fn control_flux(&self, torque: f64, speed: f64, supply: f64, rs: f64, amplitude: f64) -> f64 {
        let w_e = self.pole_pairs * speed.abs();
        let motoring = torque * speed >= 0.0;
        let need = torque.abs() / (1.5 * self.pole_pairs * self.magnetizing / self.rotor);
        let room = |flux: f64| self.torque_room(rs, supply, flux, w_e, motoring, amplitude);
        let cap = self.flux_cap(w_e);
        let at_cap = room(cap) - need;
        if at_cap >= 0.0 {
            return cap;
        }
        let top = self.flux_setpoint(speed, supply, rs, amplitude);
        let at_top = room(top) - need;
        if at_top <= 0.0 {
            return top;
        }
        // The root of room - need between them, kept bracketed: the secant through the bracket's ends, the end kept twice
        // in a row halved (Illinois): a handful of steps where a bisection took 30.
        let (mut lo, mut hi, mut f_lo, mut f_hi, mut kept) = (top, cap, at_top, at_cap, 0);
        for _ in 0..60 {
            let x = (lo * f_hi - hi * f_lo) / (f_hi - f_lo);
            let f = room(x) - need;
            if f >= 0.0 {
                (lo, f_lo) = (x, f);
                if kept == 1 {
                    f_hi *= 0.5;
                }
                kept = 1;
            } else {
                (hi, f_hi) = (x, f);
                if kept == -1 {
                    f_lo *= 0.5;
                }
                kept = -1;
            }
            // To 1e-10: the room's own rounding (`voltage_iq` to 1e-13 of the current) stops a tighter one short.
            if f.abs() <= 1e-10 * need {
                return x;
            }
            if hi - lo <= 1e-10 * cap {
                break;
            }
        }
        lo
    }

    /// The largest |i_q| up to `most` that the voltage circle `supply` leaves at the flux `flux` set (i_d = psi / L_m) in
    /// the steady state at the rotor's electrical speed `w_r` >= 0, motoring (i_q with the speed) or generating. From the
    /// model's steady voltage, with the frame at w_k = w_r + R_r (L_m / L_r) i_q / psi: u_d = R_s i_d - w_k sigma L_s i_q,
    /// u_q = R_s i_q + w_k L_s i_d (psi_sd = L_s i_d). |u| rises with |i_q| here: Newton's method on |u|^2 = U^2, kept
    /// within a bracket that a bisection step takes over from where Newton leaves it (a few steps instead of the 40 of a
    /// bisection: the flux search calls this some thousand times per velocity cycle).
    fn voltage_iq(&self, rs: f64, supply: f64, flux: f64, w_r: f64, motoring: bool, most: f64) -> f64 {
        let (id, k, sl) = (flux / self.magnetizing, self.magnetizing / self.rotor, self.sigma_ls());
        let s = if motoring { 1.0 } else { -1.0 };
        let c = self.rotor_resistance * k / flux;
        let limit = supply * supply;
        // |u|^2 - U^2 and its derivative over m = |i_q|.
        let f = |m: f64| {
            let (iq, w_k) = (s * m, w_r + c * s * m);
            let (ud, uq) = (rs * id - w_k * sl * iq, rs * iq + w_k * self.stator * id);
            let (dud, duq) = (-sl * s * (c * s * m + w_k), rs * s + c * s * self.stator * id);
            (ud * ud + uq * uq - limit, 2.0 * (ud * dud + uq * duq))
        };
        if f(most).0 <= 0.0 {
            return most;
        }
        if f(0.0).0 > 0.0 {
            return 0.0;
        }
        let (mut lo, mut hi, mut m) = (0.0, most, most);
        for _ in 0..60 {
            let (value, slope) = f(m);
            if value <= 0.0 {
                lo = m
            } else {
                hi = m
            }
            let newton = m - value / slope;
            let next = if newton > lo && newton < hi { newton } else { 0.5 * (lo + hi) };
            if (next - m).abs() <= 1e-13 * most || hi - lo <= 1e-13 * most {
                break;
            }
            m = next;
        }
        // Within 1e-13 of the root, on either side of it.
        m
    }

    /// The fluxes of a steady state in a frame turning at `w_k` with the rotor at the electrical speed `w_r`, under the
    /// voltage `u` [d, q]: the equivalent circuit.
    pub fn steady(&self, rs: f64, u: [f64; 2], w_k: f64, w_r: f64) -> [f64; 4] {
        // In complex numbers: u = R_s i_s + j w_k psi_s, 0 = R_r i_r + j (w_k - w_r) psi_r.
        // With psi = L i: u = (R_s + j w_k L_s) i_s + j w_k L_m i_r, 0 = j s L_m i_s + (R_r + j s L_r) i_r, s = w_k - w_r.
        type C = (f64, f64);
        let mul = |a: C, b: C| (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0);
        let div = |a: C, b: C| {
            let n = b.0 * b.0 + b.1 * b.1;
            ((a.0 * b.0 + a.1 * b.1) / n, (a.1 * b.0 - a.0 * b.1) / n)
        };
        let s = w_k - w_r;
        let (lm, ls, lr) = (self.magnetizing, self.stator, self.rotor);
        // i_r = -j s L_m i_s / (R_r + j s L_r) = k i_s.
        let k = div((0.0, -s * lm), (self.rotor_resistance, s * lr));
        let z = mul((0.0, w_k * lm), k);
        let z = (rs + z.0, w_k * ls + z.1);
        let is = div((u[0], u[1]), z);
        let ir = mul(k, is);
        [ls * is.0 + lm * ir.0, ls * is.1 + lm * ir.1, lm * is.0 + lr * ir.0, lm * is.1 + lr * ir.1]
    }

    /// The breakdown torque at the electrical speed `w_e` with the phase amplitude `u`, from the equivalent circuit's
    /// Thevenin form (R_s kept): 3/2 p / w_e U_th^2 / (2 (R_th + sqrt(R_th^2 + (X_th + X_sr)^2))).
    pub fn breakdown(&self, rs: f64, u: f64, w_e: f64) -> f64 {
        let (xm, xss, xsr) =
            (w_e * self.magnetizing, w_e * (self.stator - self.magnetizing), w_e * (self.rotor - self.magnetizing));
        let den = rs * rs + (xm + xss).powi(2);
        let u_th2 = u * u * xm * xm / den;
        // Z_th = j X_m (R_s + j X_ss) / (R_s + j (X_m + X_ss)).
        let (r_th, x_th) = (rs * xm * xm / den, xm * (rs * rs + xss * (xm + xss)) / den);
        1.5 * self.pole_pairs / w_e * u_th2 / (2.0 * (r_th + r_th.hypot(x_th + xsr)))
    }

    /// The V/f law: the phase amplitude at the electrical speed `w_k`.
    fn vf_voltage(&self, w_k: f64) -> f64 {
        self.boost + (self.rated_voltage - self.boost) * (w_k.abs() / self.rated_frequency).min(1.0)
    }

    /// The torque of the equivalent circuit's steady state (`steady`) under V/f at the stator frequency `w_s` with the rotor
    /// at `w_r` (both electrical, signed), the converter's voltage `supply` at most.
    fn vf_torque(&self, rs: f64, supply: f64, w_s: f64, w_r: f64) -> f64 {
        let u = self.vf_voltage(w_s).min(supply);
        self.torque(&self.steady(rs, [0.0, u], w_s, w_r))
    }

    /// (lowest, highest) torque at a speed with the DC link `dc_voltage`, as `DriveParams::torque_curve`: the breakdown
    /// curve. V/f: the most the stator frequency can make of it, motoring (above the rotor's electrical speed) and
    /// generating (below it) apart, each the maximum over the slip of the circuit's torque at the voltage V/f gives at
    /// that frequency (`vf_extreme`): the breakdown torque at a fixed frequency is the one at the speed that frequency
    /// belongs to only. Field-oriented the current limit at the flux the control sets (`flux_setpoint`), 3/2 p (L_m / L_r)
    /// psi sqrt(I^2 - (psi / L_m)^2) for the amplitude I of the smaller of the motor's and the drive's maximum current,
    /// within what the voltage circle leaves at that flux (`voltage_iq`), and the breakdown; and the peak torque.
    pub fn torque_curve(&self, p: &DriveParams, speed: f64, dc_voltage: f64) -> (f64, f64) {
        let w_e = (self.pole_pairs * speed.abs()).max(0.02 * self.rated_frequency);
        let supply = dc_voltage / 3f64.sqrt();
        let (motoring, generating) = if self.field_oriented {
            let amplitude = SQRT_2 * p.max_current.min(p.drive_max_current);
            let flux = self.flux_setpoint(speed, supply, p.resistance, amplitude);
            let iq = (amplitude * amplitude - (flux / self.magnetizing).powi(2)).max(0.0).sqrt();
            let w_r = self.pole_pairs * speed.abs();
            let breakdown = self.breakdown(p.resistance, supply, w_e);
            let torque = |motoring| {
                let iq = self.voltage_iq(p.resistance, supply, flux, w_r, motoring, iq);
                (1.5 * self.pole_pairs * self.magnetizing / self.rotor * flux * iq).min(breakdown)
            };
            (torque(true), torque(false))
        } else {
            let w_r = self.pole_pairs * speed.abs();
            (self.vf_extreme(p.resistance, supply, w_r, true), self.vf_extreme(p.resistance, supply, w_r, false))
        };
        let (motoring, generating) = (motoring.min(p.peak_torque), generating.min(p.peak_torque));
        if speed >= 0.0 { (-generating, motoring) } else { (-motoring, generating) }
    }

    /// V/f: the largest torque magnitude, motoring or generating, at the rotor's electrical speed `w_r` >= 0: the maximum
    /// over the slip s = w_s - w_r (> 0 motoring, < 0 generating, the stator frequency down to and through 0 for
    /// generating) of `vf_torque`. A scan over the slip, dense near 0, refined by golden section around its best.
    fn vf_extreme(&self, rs: f64, supply: f64, w_r: f64, motoring: bool) -> f64 {
        const N: usize = 96;
        let span = 2.0 * (w_r + self.rated_frequency);
        let sign = if motoring { 1.0 } else { -1.0 };
        let at = |x: f64| {
            // x in (0, 1]: the slip's magnitude span x^2; never exactly a stator frequency of 0.
            let w_s = w_r + sign * span * x * x;
            (sign * self.vf_torque(rs, supply, if w_s == 0.0 { 1e-6 } else { w_s }, w_r)).max(0.0)
        };
        let values: Vec<f64> = (1..=N).map(|n| at(n as f64 / N as f64)).collect();
        let best = (0..N).max_by(|&a, &b| values[a].total_cmp(&values[b])).unwrap_or(0);
        let (mut lo, mut hi) = ((best as f64) / N as f64, ((best + 2).min(N)) as f64 / N as f64);
        let g = 0.5 * (5f64.sqrt() - 1.0);
        let (mut x1, mut x2) = (hi - g * (hi - lo), lo + g * (hi - lo));
        let (mut f1, mut f2) = (at(x1), at(x2));
        for _ in 0..30 {
            if f1 < f2 {
                (lo, x1, f1) = (x1, x2, f2);
                x2 = lo + g * (hi - lo);
                f2 = at(x2);
            } else {
                (hi, x2, f2) = (x2, x1, f1);
                x1 = hi - g * (hi - lo);
                f1 = at(x1);
            }
        }
        values[best].max(f1).max(f2)
    }

    /// The drive enabled: field-oriented, magnetized at the rated flux and giving `torque`, in the steady state at rest;
    /// V/f unmagnetized (nothing flows until the converter puts out a voltage).
    pub(crate) fn at_rest(&self, d: &mut Drive, p: &DriveParams, pos: f64, torque: f64) {
        d.phases = Phases { angle: pos, ..Default::default() };
        if !self.field_oriented {
            (d.current, d.torque, d.torque_demand, d.torque_command) = (0.0, 0.0, 0.0, 0.0);
            return;
        }
        let flux = self.rated_flux;
        let (id, iq) =
            (flux / self.magnetizing, torque / (1.5 * self.pole_pairs * self.magnetizing / self.rotor * flux));
        let slip = self.rotor_resistance * self.magnetizing / self.rotor * iq / flux;
        // The fluxes of that state: psi_r = flux on d, psi_s = sigma L_s i_s + (L_m / L_r) psi_r.
        let k = self.magnetizing / self.rotor;
        let x = [self.sigma_ls() * id + k * flux, self.sigma_ls() * iq, flux, 0.0];
        let u = [p.resistance * id - slip * x[1], p.resistance * iq + slip * x[0]];
        let ff = self.feedforward(slip, 0.0, [id, iq], flux);
        d.phases.x = x;
        d.phases.u = u;
        d.phases.integral = [u[0] - ff[0], u[1] - ff[1]];
        (d.phases.frame_speed, d.phases.estimate) = (slip, flux);
        d.current = id.hypot(iq) / SQRT_2;
    }

    /// The decoupling the field-oriented current controllers add (see the module comment).
    fn feedforward(&self, w_k: f64, w_r: f64, [id, iq]: [f64; 2], flux: f64) -> [f64; 2] {
        let (sl, k) = (self.sigma_ls(), self.magnetizing / self.rotor);
        [-w_k * sl * iq - k * self.rotor_resistance / self.rotor * flux, w_k * sl * id + k * w_r * flux]
    }

    /// The rates of the fluxes (`DriveParams::motor_rates`), [psi_sd, psi_sq, psi_rd, psi_rq] in slots 0, 4, 5, 6.
    pub(crate) fn rates(&self, p: &DriveParams, d: &Drive, w: &[f64], speed: f64, out: &mut [f64]) -> f64 {
        let x = [w[0], w[4], w[5], w[6]];
        let (is, ir) = self.currents(&x);
        let (u, w_k, rs, rr) = (d.phases.u, d.phases.frame_speed, d.winding_resistance(p), self.rotor_resistance);
        let slip = w_k - self.pole_pairs * speed;
        out[0] = u[0] - rs * is[0] + w_k * x[1];
        out[4] = u[1] - rs * is[1] - w_k * x[0];
        out[5] = -rr * ir[0] + slip * x[3];
        out[6] = -rr * ir[1] - slip * x[2];
        let torque = 1.5 * self.pole_pairs * (x[0] * is[1] - x[1] * is[0]);
        out[1] = 1.5 * (u[0] * is[0] + u[1] * is[1]);
        out[2] = 1.5 * (rs * (is[0] * is[0] + is[1] * is[1]) + rr * (ir[0] * ir[0] + ir[1] * ir[1]));
        out[3] = torque * speed;
        torque
    }

    /// One base step of the converter, with the speed the drive measured.
    pub(crate) fn control(&self, d: &mut Drive, p: &DriveParams, speed: f64) {
        let dt = p.current_cycle;
        let supply = Self::supply(d, p);
        let rs = d.winding_resistance(p);
        let imax = p.overload.as_ref().filter(|_| d.overloaded).map_or(f64::INFINITY, |o| o.base_current);
        let (is, _) = self.currents(&d.phases.x);
        d.iron_loss = 0.0;
        if !self.field_oriented {
            // Slip compensation from the torque the converter estimates from its own power: the air gap's, (3/2 u . i
            // less the stator's copper loss) over the frequency's synchronous speed.
            let ph = &mut d.phases;
            let synchronous = self.pole_pairs * d.speed_command;
            let compensation =
                if self.slip_compensation { self.rated_slip * ph.estimate / p.rated_torque } else { 0.0 };
            let w_k = synchronous + compensation.copysign(synchronous);
            let u = self.vf_voltage(w_k).min(supply);
            ph.u = [0.0, u];
            ph.frame_speed = w_k;
            d.voltage = u;
            return;
        }
        let ph = &mut d.phases;
        let k = self.magnetizing / self.rotor;
        // The drive's flux model, exact over the step for the d current held.
        let tau = self.rotor / self.rotor_resistance;
        let target = self.magnetizing * is[0];
        ph.estimate = target + (ph.estimate - target) * (-dt / tau).exp();
        let flux = ph.estimate.max(1e-3 * self.rated_flux);
        let w_r = self.pole_pairs * speed;
        ph.frame_speed = w_r + self.rotor_resistance * k * is[1] / flux;
        // Setpoints: the flux's d current (rated, weakened only where the voltage runs out at the torque asked for,
        // `control_flux`), the torque's q current, within the current limit (the drive's base load current while it is
        // overloaded) and the voltage circle at that flux, motoring and generating apart; at the curve's torque the flux
        // is the curve's (`torque_curve`).
        let amplitude = SQRT_2 * p.max_current.min(p.drive_max_current).min(imax);
        let flux_set = self.control_flux(d.torque_command, speed, supply, rs, amplitude);
        let id_set = (flux_set / self.magnetizing).min(amplitude);
        let iq_most = (amplitude * amplitude - id_set * id_set).max(0.0).sqrt();
        let by_voltage = |motoring| self.voltage_iq(rs, supply, flux_set, w_r.abs(), motoring, iq_most);
        let (with_speed, against_speed) = (by_voltage(true), by_voltage(false));
        let (iq_high, iq_low) = if w_r >= 0.0 { (with_speed, against_speed) } else { (against_speed, with_speed) };
        let iq_set = (d.torque_command / (1.5 * self.pole_pairs * k * flux)).clamp(-iq_low, iq_high);
        let ff = self.feedforward(ph.frame_speed, w_r, is, flux);
        let error = [id_set - is[0], iq_set - is[1]];
        let wanted = [0, 1].map(|n| p.current_kp * error[n] + ph.integral[n] + ff[n]);
        // Within the voltage circle, the d axis first.
        let ud = wanted[0].clamp(-supply, supply);
        let room = (supply * supply - ud * ud).max(0.0).sqrt();
        let uq = wanted[1].clamp(-room, room);
        for (n, u) in [ud, uq].into_iter().enumerate() {
            if u == wanted[n] {
                ph.integral[n] += p.current_ki * error[n] * dt;
            }
        }
        ph.u = [ud, uq];
        d.voltage = ud.hypot(uq);
    }

    /// The step done: the RMS phase current and the torque. V/f: the torque is what the drive gave, and its estimate for
    /// the slip compensation.
    pub(crate) fn stepped(&self, d: &mut Drive, p: &DriveParams) {
        let x = d.phases.x;
        let (is, _) = self.currents(&x);
        d.current = is[0].hypot(is[1]) / SQRT_2;
        d.torque = self.torque(&x);
        if !self.field_oriented {
            (d.torque_demand, d.torque_command) = (d.torque, d.torque);
            let (u, w_k) = (d.phases.u, d.phases.frame_speed);
            let air_gap =
                1.5 * (u[0] * is[0] + u[1] * is[1]) - 1.5 * d.winding_resistance(p) * (is[0] * is[0] + is[1] * is[1]);
            let raw = if w_k.abs() > 1e-6 { self.pole_pairs * air_gap / w_k } else { 0.0 };
            // shortcut: filtered with the rotor's time constant, so that the compensation does not swing with the
            // rotor (unfiltered it rang at a few hertz); a converter's own filter time is a parameter of its own.
            let share = 1.0 - (-p.current_cycle * self.rotor_resistance / self.rotor).exp();
            d.phases.estimate += share * (raw - d.phases.estimate);
        }
    }

    /// The copper loss at the rated point of the name plate, from the equivalent circuit at the rated voltage, frequency
    /// and slip, with the stator at `resistance` (warm): what brings the motor to its rated rise.
    pub fn rated_copper_loss(&self, resistance: f64) -> f64 {
        let w = self.rated_frequency;
        let x = self.steady(resistance, [0.0, self.rated_voltage], w, w - self.rated_slip);
        let (is, ir) = self.currents(&x);
        1.5 * (resistance * (is[0] * is[0] + is[1] * is[1]) + self.rotor_resistance * (ir[0] * ir[0] + ir[1] * ir[1]))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::drive::{Motor, WINDING_PER_DC_LINK, WINDING_STATES};
    use sim_core::Rk4;

    /// A four-pole 1.5 kW motor of round numbers on 400 V, 50 Hz, 1440 rpm: R_s = 4.5 Ohm, R_r = 3.5 Ohm, L_m = 0.45 H,
    /// both leakages 0.02 H; on a 600 V DC link (the 400 V line's peak, 565.7 V, and some). Field-oriented with T = 0.5 ms: sigma L_s = 0.47 - 0.45^2 / 0.47 =
    /// 0.039149 H, Kp = 78.30 V/A, R_sigma = 4.5 + (0.45 / 0.47)^2 3.5 = 7.7085 Ohm, Tn = 5.0787 ms.
    fn params(field_oriented: bool, rs: f64) -> DriveParams {
        let mut m = Induction {
            pole_pairs: 2.0,
            rotor_resistance: 3.5,
            magnetizing: 0.45,
            stator: 0.47,
            rotor: 0.47,
            rated_voltage: PHASE_PER_LINE * 400.0,
            rated_frequency: TAU * 50.0,
            rated_slip: TAU * 50.0 - 2.0 * 1440.0 * TAU / 60.0,
            field_oriented,
            boost: 0.0,
            slip_compensation: false,
            rated_flux: 0.0,
        };
        m.rated_flux = m.magnetizing * m.rated_voltage / rs.hypot(m.rated_frequency * m.stator);
        let kp = 0.039149 / 0.5e-3;
        DriveParams {
            resistance: rs,
            heating: false,
            dc_link: false,
            bus_voltage: 600.0,
            max_voltage: WINDING_PER_DC_LINK * 600.0,
            current_kp: kp,
            current_ki: kp / 5.0787e-3,
            current_cycle: 62.5e-6,
            max_current: 10.0,
            drive_max_current: f64::INFINITY,
            rated_torque: 10.0,
            peak_torque: 30.0,
            motor: Motor::Induction(m),
            ..crate::drive::tests::params()
        }
    }

    fn motor(p: &DriveParams) -> &Induction {
        let Motor::Induction(m) = &p.motor else { unreachable!() };
        m
    }

    /// The motor on an inertia `j` (infinite: the rotor held at `speed`) against `load(t)`, its converter stepped every
    /// base step with `command(t, drive)` setting what it follows; integrated as `Sim::step` does. Returns per base
    /// step (time, speed, torque, |rotor flux|).
    fn run(
        p: &DriveParams,
        d: &mut Drive,
        j: f64,
        mut speed: f64,
        load: impl Fn(f64) -> f64,
        command: impl Fn(f64, &mut Drive),
        seconds: f64,
    ) -> Vec<[f64; 4]> {
        let m = motor(p);
        let dt = p.current_cycle;
        let mut rk = Rk4::new(2 + WINDING_STATES);
        let mut angle = 0.0;
        let mut out = Vec::new();
        for n in 0..(seconds / dt).round() as usize {
            let t = n as f64 * dt;
            command(t, d);
            m.control(d, p, speed);
            let mut y = vec![angle, speed];
            y.extend(d.winding_states(p));
            let tl = load(t);
            rk.step(
                |y, dy| {
                    let torque = p.motor_rates(d, &y[2..], y[0], y[1], &mut dy[2..]);
                    dy[0] = y[1];
                    dy[1] = if j.is_finite() { (torque - tl) / j } else { 0.0 };
                },
                &mut y,
                dt,
            );
            (angle, speed) = (y[0], y[1]);
            let w = &y[2..];
            (d.phases.x, d.phases.angle, d.phases.speed) = ([w[0], w[4], w[5], w[6]], angle, speed);
            d.winding_stepped(p, w[0], [w[1], w[2], w[3]]);
            out.push([t + dt, speed, d.torque, d.phases.x[2].hypot(d.phases.x[3])]);
        }
        out
    }

    /// Steady torque under V/f at rated voltage and frequency with the rotor held at `speed`: from the equivalent circuit's
    /// fluxes (`steady`), which the integration then keeps (without R_s nothing damps the stator flux's transient).
    fn steady_torque(p: &DriveParams, speed: f64) -> f64 {
        let m = motor(p);
        let mut d = Drive::at_rest(p, 0.0, 0.0, 0.0);
        d.phases.x = m.steady(p.resistance, [0.0, m.rated_voltage], m.rated_frequency, m.pole_pairs * speed);
        d.phases.frame_speed = m.rated_frequency;
        let trace = run(p, &mut d, f64::INFINITY, speed, |_| 0.0, |_, d| d.speed_command = 50.0 * TAU / 2.0, 0.5);
        trace.last().unwrap()[2]
    }

    /// The name plate by hand: 50 Hz on two pole pairs turn the field at 50 * 60 / 2 = 1500 rpm (157.08 rad/s); 1440 rpm
    /// is a slip of (1500 - 1440) / 1500 = 4 %.
    #[test]
    fn synchronous_speed_and_rated_slip_come_from_the_name_plate() {
        let p = params(false, 4.5);
        let m = motor(&p);
        assert!((m.rated_frequency / m.pole_pairs - 157.0796).abs() < 1e-4);
        assert!((m.rated_slip / m.rated_frequency - 0.04).abs() < 1e-12);
        // The model at the rated slip gives about the rated torque, 1.5 kW / 150.80 rad/s = 9.95 N m: within 10 % (the
        // equivalent circuit is round numbers, not fitted to it).
        let torque = steady_torque(&p, 1440.0 * TAU / 60.0);
        assert!((torque - 9.95).abs() < 1.0, "{torque} N m");
    }

    /// Kloss by hand, with R_s = 0, where it is exact: the Thevenin source U_th = U X_m / (X_m + X_ss) = 326.60 * 141.37 /
    /// 147.65 = 312.71 V, X_th = X_ss X_m / (X_m + X_ss) = 6.2832 * 141.37 / 147.65 = 6.0159 Ohm; breakdown slip
    /// s_b = R_r / (X_th + X_sr) = 3.5 / 12.299 = 0.28457 and torque T_b = 3/2 p U_th^2 / (2 w (X_th + X_sr)) = 3 *
    /// 312.71^2 / (2 * 314.16 * 12.299) = 37.97 N m; T = 2 T_b / (s / s_b + s_b / s). The model's steady torque at three
    /// slips meets it. With R_s = 4.5 Ohm the Thevenin source has R_th = R_s X_m^2 / (R_s^2 + X_s^2) = 4.121 Ohm,
    /// X_th = 6.141 Ohm and U_th = 312.55 V, and the breakdown torque 3/2 p / w U_th^2 / (2 (R_th + |R_th + j (X_th + X_sr)|))
    /// = 0.0095493 * 97690.6 / (2 * (4.121 + 13.090)) = 27.10 N m (`breakdown`): Kloss's formula without R_s overstates it
    /// by 40 %. The simplification holds for large motors, whose R_s is small against their reactances.
    #[test]
    fn the_torque_slip_curve_is_kloss_and_the_breakdown_is_the_equivalent_circuits() {
        let p = params(false, 0.0);
        let (tb, sb) = (37.97, 0.28457);
        let w_sync = 157.0796;
        for s in [0.02, sb, 0.6] {
            let kloss = 2.0 * tb / (s / sb + sb / s);
            let torque = steady_torque(&p, (1.0 - s) * w_sync);
            assert!((torque / kloss - 1.0).abs() < 2e-3, "at s = {s}: {torque} N m against {kloss}");
        }
        let m = motor(&p);
        assert!((m.breakdown(0.0, m.rated_voltage, TAU * 50.0) / tb - 1.0).abs() < 1e-3);
        // With the stator's resistance: the curve's top, found by slips, is the Thevenin form's.
        let p = params(false, 4.5);
        let m = motor(&p);
        let by_formula = m.breakdown(4.5, m.rated_voltage, TAU * 50.0);
        assert!((by_formula - 27.10).abs() < 0.01, "{by_formula}");
        let top = [0.2, 0.22, 0.24, 0.26, 0.28]
            .map(|s| steady_torque(&p, (1.0 - s) * w_sync))
            .into_iter()
            .fold(0.0, f64::max);
        assert!(top <= by_formula * 1.002 && top > 0.995 * by_formula, "{top} against {by_formula}");
    }

    /// Starting current by hand: rotor held, rated voltage and frequency. Z = R_s + j X_ss + j X_m || (R_r + j X_sr) with
    /// X_m || (3.5 + j 6.2832) = (3.2071 + j 6.1047) Ohm, so Z = (7.7071 + j 12.388) Ohm, |Z| = 14.590 Ohm, and the
    /// amplitude 326.60 / 14.590 = 22.385 A, 15.83 A RMS: 4.8 times the 3.3 A of the name plate.
    #[test]
    fn the_starting_current_is_the_equivalent_circuits() {
        let p = params(false, 4.5);
        let mut d = Drive::at_rest(&p, 0.0, 0.0, 0.0);
        run(&p, &mut d, f64::INFINITY, 0.0, |_| 0.0, |_, d| d.speed_command = 157.0796, 1.0);
        assert!((d.current / (22.385 / SQRT_2) - 1.0).abs() < 2e-3, "{} A", d.current);
    }

    /// Under V/f a load step of 5 N m at 50 Hz slows the rotor by the slip it needs: T = 3/2 p psi_r^2 w_slip / R_r
    /// (steady, exact), so w_slip = 2 * 3.5 * 5 / (3 * 2 * 0.9950^2) = 5.894 rad/s electrical, 2.947 rad/s at the shaft,
    /// with the rated flux psi_r = L_m U / |R_s + j w L_s| = 0.45 * 326.60 / 147.72 = 0.9950 Wb (unloaded; the load lowers
    /// it by a few per cent, hence 6 %). With slip compensation the converter raises the frequency by the rated slip times
    /// the torque over the rated one, 12.566 * 5 / 10 = 6.283 rad/s, and the shaft keeps its speed within 0.4 rad/s.
    #[test]
    fn under_v_f_a_load_step_costs_the_hand_calculated_slip() {
        let drop = |compensated: bool| {
            let mut p = params(false, 4.5);
            if let Motor::Induction(m) = &mut p.motor {
                m.slip_compensation = compensated;
            }
            let mut d = Drive::at_rest(&p, 0.0, 0.0, 0.0);
            let load = |t: f64| if t > 1.5 { 5.0 } else { 0.0 };
            let trace = run(&p, &mut d, 0.05, 157.0796, load, |_, d| d.speed_command = 157.0796, 3.0);
            let at = |t: f64| trace[(t / p.current_cycle) as usize - 1][1];
            at(1.5) - at(3.0)
        };
        let plain = drop(false);
        assert!((plain / 2.947 - 1.0).abs() < 0.06, "{plain} rad/s");
        let compensated = drop(true);
        assert!(compensated.abs() < 0.4, "{compensated} rad/s");
    }

    /// The field-oriented curve on a 540 V link (phase amplitude 540 / sqrt 3 = 311.77 V, the 400 V plate needs 326.6 V)
    /// is the torque the voltage leaves at the flux the control sets, the flux of the most torque. At the rated flux
    /// 0.99488 Wb (i_d = 2.2108 A) the circle leaves 23.05 N m at 115 rad/s (w = 230 rad/s): i_q = 23.05 / (3 * 0.957447 *
    /// 0.99488) = 8.066 A, slip 3.5 * 0.957447 * 8.066 / 0.99488 = 27.17 rad/s, w_k = 257.17: u_d = 4.5 * 2.2108 - 257.17 *
    /// 0.039149 * 8.066 = -71.26 V, u_q = 4.5 * 8.066 + 257.17 * 0.47 * 2.2108 = 303.5 V, |u| = 311.8 V. A weaker flux gives
    /// more: at 0.7743 Wb (i_d = 1.7207 A) i_q = 12.27 A (inside the current circle's sqrt(200 - 2.96) = 14.04 A), w_k =
    /// 230 + 3.5 * 0.957447 * 12.27 / 0.7743 = 283.1, u_d = 4.5 * 1.7207 - 283.1 * 0.039149 * 12.27 = -128.3 V, u_q =
    /// 4.5 * 12.27 + 283.1 * 0.47 * 1.7207 = 284.2 V, |u| = 311.8 V again, torque 3 * 0.957447 * 0.7743 * 12.27 = 27.29 N m
    /// (a scan of the flux finds no more). At 140 rad/s the rated flux leaves 7.24 N m and 0.6499 Wb 20.68 N m. At 60 rad/s
    /// the current circle alone binds: the rated flux stays.
    #[test]
    fn the_field_oriented_curve_follows_the_voltage_the_link_leaves() {
        let p = params(true, 4.5);
        let m = motor(&p);
        let supply = 540.0 / 3f64.sqrt();
        let high = m.torque_curve(&p, 115.0, 540.0).1;
        assert!((high - 27.29).abs() < 0.3, "{high} N m");
        let high = m.torque_curve(&p, 140.0, 540.0).1;
        assert!((high - 20.68).abs() < 0.3, "{high} N m");
        let amplitude = SQRT_2 * 10.0;
        let flux = m.flux_setpoint(115.0, supply, 4.5, amplitude);
        assert!((flux - 0.7743).abs() < 0.005, "{flux} Wb");
        assert!((m.flux_setpoint(140.0, supply, 4.5, amplitude) - 0.6499).abs() < 0.005);
        assert_eq!(m.flux_setpoint(60.0, supply, 4.5, amplitude), m.rated_flux);
    }

    /// The flux set is the top of psi i_q(psi): no flux of a fine grid up to the rated one (or the field-weakened cap) leaves
    /// more of the product, at speeds with the voltage binding, on both links, and with a smaller drive current (a 6 A
    /// drive, amplitude 8.485 A).
    #[test]
    fn the_flux_setpoint_is_the_flux_of_the_most_torque() {
        let p = params(true, 4.5);
        let m = motor(&p);
        for (speed, link, amplitude) in
            [(115.0, 540.0, 14.14), (140.0, 540.0, 14.14), (200.0, 600.0, 14.14), (130.0, 540.0, 8.485)]
        {
            let supply = link / 3f64.sqrt();
            let w_e = m.pole_pairs * speed;
            let cap = m.rated_flux * (m.rated_frequency / w_e).min(1.0);
            let product = |flux: f64| {
                let most = (amplitude * amplitude - (flux / m.magnetizing).powi(2)).max(0.0).sqrt();
                flux * m.voltage_iq(4.5, supply, flux, w_e, true, most)
            };
            let set = m.flux_setpoint(speed, supply, 4.5, amplitude);
            let best = (1..=2000).map(|n| product(cap * n as f64 / 2000.0)).fold(0.0, f64::max);
            let got = product(set);
            assert!(set <= cap && got >= best * (1.0 - 1e-4), "{speed} rad/s: {set} Wb, {got} against {best}");
        }
    }

    /// V/f at fixed voltage and frequency, by the Thevenin form: the circuit's torque at the breakdown slip s_b = R_r /
    /// sqrt(R_th^2 + (X_th + X_sr)^2) is `breakdown` (R_s = 4.5 Ohm at 50 Hz, 326.60 V: R_th = 4.121 Ohm, X_th + X_sr =
    /// 6.141 + 6.283 = 12.424 Ohm, s_b = 3.5 / 13.09 = 0.2674 rad/s per rad/s, 84.0 rad/s of the 314.16), 27.10 N m. With the
    /// rotor at w_r = 314.16 - 84.0 = 230.2 rad/s electrical the stator frequency of 50 Hz is one candidate of the V/f scan
    /// (`vf_extreme`), so the scan's top is at least that breakdown; and it is the top of a dense sweep of the stator
    /// frequency (20000 points) within the scan's refinement, 1e-3.
    #[test]
    fn the_v_f_scan_agrees_with_the_thevenin_breakdown_and_a_dense_sweep() {
        let p = params(false, 4.5);
        let m = motor(&p);
        let (w_s, rs, u) = (m.rated_frequency, 4.5, m.rated_voltage);
        let (xm, xss, xsr) = (w_s * m.magnetizing, w_s * (m.stator - m.magnetizing), w_s * (m.rotor - m.magnetizing));
        let den = rs * rs + (xm + xss).powi(2);
        let (r_th, x_th) = (rs * xm * xm / den, xm * (rs * rs + xss * (xm + xss)) / den);
        let slip = w_s * m.rotor_resistance / r_th.hypot(x_th + xsr);
        let breakdown = m.breakdown(rs, u, w_s);
        assert!((m.vf_torque(rs, 1e9, w_s, w_s - slip) / breakdown - 1.0).abs() < 1e-6, "{slip}");
        let w_r = w_s - slip;
        let scan = m.vf_extreme(rs, 1e9, w_r, true);
        assert!(scan >= breakdown * (1.0 - 1e-3), "{scan} against {breakdown}");
        let dense = (1..=20_000)
            .map(|n| m.vf_torque(rs, 1e9, w_r + 4.0 * m.rated_frequency * n as f64 / 20_000.0, w_r))
            .fold(0.0, f64::max);
        assert!((scan / dense - 1.0).abs() < 1e-3 && scan >= dense * (1.0 - 1e-3), "{scan} against {dense}");
    }

    /// The drive's current limit is the converter's, not a servo's torque constant: a 6 A drive (RMS, amplitude 8.485 A)
    /// at 50 rad/s holds i_d = 0.99488 / 0.45 = 2.2108 A and i_q = sqrt(72 - 4.888) = 8.192 A, 3 * 0.957447 * 0.99488 *
    /// 8.192 = 23.41 N m, where the servo constant (3.0 N m/A here) clamped at 18.0 N m.
    #[test]
    fn the_drive_current_limit_of_an_induction_motor_is_its_own_currents_torque() {
        let p = DriveParams { drive_max_current: 6.0, kt: 3.0, ..params(true, 4.5) };
        let high = p.torque_range_at(50.0, p.max_voltage).1;
        assert!((high - 23.41).abs() < 0.1, "{high} N m");
    }

    /// V/f at low speed: the stator frequency runs ahead of the rotor, not at it. At 10 rad/s (w = 20 rad/s) the most the
    /// circuit makes of the V/f voltage over the slip is 17.949 N m at a stator frequency of 198.3 rad/s (the old curve,
    /// the breakdown at the frequency of the rotor, 7.79 N m): at 20 rad/s the voltage is 326.6 * 20 / 314.16 = 20.8 V,
    /// little for the 4.5 Ohm stator, and more slip means more voltage with it. At 25 rad/s 19.552 N m (at 199.4 rad/s).
    /// The figures are a sweep of 200 000 stator frequencies (w_r up to w_r + 4 w_N, no boost, as here) through the
    /// per-phase circuit R_s + j w L_ss + j w L_m || (R_r w / (w - w_r) + j w L_sr), torque 3/2 p |i_r|^2 R_r / (s w), written
    /// apart from `steady` (it gives the same to 1e-9); the audit's 18.88 and 20.53 N m were not the circuit's maximum.
    #[test]
    fn the_v_f_curve_is_the_maximum_over_the_slip() {
        let p = params(false, 4.5);
        let (low, high) = motor(&p).torque_curve(&p, 10.0, 540.0);
        assert!((high - 17.949).abs() < 0.01, "{high} N m");
        assert!(low < 0.0);
        let (_, high) = motor(&p).torque_curve(&p, 25.0, 540.0);
        assert!((high - 19.552).abs() < 0.01, "{high} N m");
    }

    /// The control keeps the rated flux at part load and weakens it only where the voltage binds at the torque asked for;
    /// asked for the curve's torque it delivers it. On the 540 V link at 120 rad/s, 5 N m fit at the rated flux (the circle
    /// leaves 20.03 N m there): the rated flux stays. At 110 to 150 rad/s the rotor held at speed and the curve's torque
    /// commanded from the rated flux: after 1.5 s (11 rotor time constants) the torque is the curve's within 1 %.
    #[test]
    fn asked_for_the_curves_torque_the_field_oriented_control_delivers_it() {
        let p = DriveParams { bus_voltage: 540.0, max_voltage: WINDING_PER_DC_LINK * 540.0, ..params(true, 4.5) };
        let m = motor(&p);
        let supply = 540.0 / 3f64.sqrt();
        let amplitude = SQRT_2 * 10.0;
        let at_rated = 3.0 * m.magnetizing / m.rotor * m.torque_room(4.5, supply, m.rated_flux, 240.0, true, amplitude);
        assert!((at_rated - 20.03).abs() < 0.05, "{at_rated} N m");
        assert_eq!(m.control_flux(5.0, 120.0, supply, 4.5, amplitude), m.rated_flux);
        assert!(m.control_flux(25.0, 120.0, supply, 4.5, amplitude) < m.rated_flux);
        for speed in [110.0, 120.0, 130.0, 140.0, 150.0] {
            let curve = m.torque_curve(&p, speed, 540.0).1;
            let mut d = Drive::at_rest(&p, 0.0, 0.0, 0.0);
            let trace = run(&p, &mut d, f64::INFINITY, speed, |_| 0.0, |_, d| d.torque_command = curve, 1.5);
            let delivered = trace.last().unwrap()[2];
            assert!((delivered / curve - 1.0).abs() < 0.01, "{speed} rad/s: {delivered} N m of {curve}");
        }
    }

    /// Field-oriented from no flux: the d current steps to psi* / L_m within the current loop's 0.5 ms, and the rotor flux
    /// follows with L_r / R_r = 0.47 / 3.5 = 134.3 ms: 63.2 % of it at one time constant, 95.0 % at three.
    #[test]
    fn the_rotor_flux_builds_with_l_r_over_r_r() {
        let p = params(true, 4.5);
        let m = motor(&p);
        let mut d = Drive::at_rest(&p, 0.0, 0.0, 0.0);
        d.phases = Phases::default();
        let trace = run(&p, &mut d, f64::INFINITY, 0.0, |_| 0.0, |_, d| d.torque_command = 0.0, 0.45);
        let tau = 0.47 / 3.5;
        let at = |t: f64| trace[(t / p.current_cycle) as usize - 1][3] / m.rated_flux;
        assert!((at(tau) - 0.632).abs() < 0.01, "{}", at(tau));
        assert!((at(3.0 * tau) - 0.950).abs() < 0.01, "{}", at(3.0 * tau));
    }

    /// Field-oriented, magnetized, rotor held: a torque step of 5 N m answers as the current loop does, 63.2 % within its
    /// T = sigma L_s / Kp = 0.5 ms, and holds it (the flux stays, the slip frequency keeps the frame on it).
    #[test]
    fn under_field_orientation_a_torque_step_answers_with_the_current_loops_time_constant() {
        let p = params(true, 4.5);
        let mut d = Drive::at_rest(&p, 0.0, 0.0, 0.0);
        let trace = run(&p, &mut d, f64::INFINITY, 0.0, |_| 0.0, |_, d| d.torque_command = 5.0, 0.05);
        let at = |t: f64| trace[(t / p.current_cycle).round() as usize - 1][2];
        assert!((at(0.5e-3) / 5.0 - 0.632).abs() < 0.05, "{}", at(0.5e-3));
        assert!((at(0.05) / 5.0 - 1.0).abs() < 2e-3, "{}", at(0.05));
    }
}
