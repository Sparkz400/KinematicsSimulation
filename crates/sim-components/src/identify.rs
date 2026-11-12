//! Fitting a config to a measured machine, in stages: first the rigid values of each axis from a drive
//! trace, then the load's resonance on the belt from a frequency sweep. Nothing here edits a config; the
//! results say what the config would have to be.
//!
//! Every stage runs the same estimate on the measurement and on the config's own simulation of the same
//! run, and reports the two side by side. What the simple estimate gets wrong (gearbox losses that grow
//! with the torque, friction that depends on the load, a resonance that moves with the sweep's amplitude)
//! it gets wrong on both sides, so the difference between them is what the config is off by.

use crate::analysis::SweepPoint;
use crate::trace::{Trace, TraceError, replay};
use sim_core::{Config, Machine, MachineEntry};

/// Rows on either side of a row that are averaged with it: 11 rows in all, 11 ms in a trace from a run.
/// Long enough to average the belt's ringing and the noise, short against a ramp of the profile.
const WINDOW: usize = 5;

/// The rigid values of one axis, in this order: moved mass (with rotors and pulleys counted as mass at the
/// axis), viscous friction, Coulomb friction and constant force. In N, kg, s and the axis's unit.
#[derive(Debug, Clone, PartialEq)]
pub struct AxisFit {
    /// What the estimate gives on the config's simulation of the trace's setpoints.
    pub config: [f64; 4],
    /// What it gives on the trace, and the standard error of that.
    pub fitted: [f64; 4],
    pub error: [f64; 4],
}

impl AxisFit {
    /// What the fit says to change in the config, as translation keys with their argument: the axis named
    /// `axis` with the change of mass and constant force, and the factors on the two kinds of friction.
    pub fn advice(&self, axis: &str) -> [(&'static str, String); 4] {
        let change = |q: usize, unit: &str| format!("{axis}: {:+.3} {unit} ± {:.3}", self.fitted[q] - self.config[q], self.error[q]);
        let factor = |q: usize| format!("{axis}: × {:.3} ± {:.3}", self.fitted[q] / self.config[q], (self.error[q] / self.config[q]).abs());
        [("fit-mass", change(0, "kg")), ("fit-viscous", factor(1)), ("fit-coulomb", factor(2)), ("fit-constant", change(3, "N"))]
    }
}

/// Per axis: the force the motors put on it and its speed, both from the trace's motor values through
/// the machine's transformation. Motor angle per axis travel turns motor torques into axis forces
/// (the same power on both sides), so the fit works per axis even where every motor moves every axis.
fn axis_view(machine: &dyn Machine, trace: &Trace) -> Vec<(Vec<f64>, Vec<f64>)> {
    let (axes, motors, h) = (machine.axes().len(), trace.motors.len(), 1e-3);
    let mut out = vec![(Vec::new(), Vec::new()); axes];
    let (mut pose, mut ahead) = (vec![0.0; axes], vec![0.0; axes]);
    let (mut base, mut shifted) = (vec![0.0; motors], vec![0.0; motors]);
    for row in 0..trace.time.len() {
        let angle: Vec<f64> = trace.motors.iter().map(|m| m.angle[row]).collect();
        let moved: Vec<f64> = trace.motors.iter().map(|m| m.angle[row] + h * m.speed[row]).collect();
        machine.forward(&angle, &mut pose);
        machine.forward(&moved, &mut ahead);
        machine.inverse(&pose, &mut base);
        for (i, (force, speed)) in out.iter_mut().enumerate() {
            speed.push((ahead[i] - pose[i]) / h);
            pose[i] += h;
            machine.inverse(&pose, &mut shifted);
            pose[i] -= h;
            force.push((0..motors).map(|k| trace.motors[k].torque[row] * (shifted[k] - base[k]) / h).sum());
        }
    }
    out
}

/// Inverse of a 4 x 4 matrix by elimination; `None` if it has none.
fn invert(mut m: [[f64; 4]; 4]) -> Option<[[f64; 4]; 4]> {
    let mut inv = [[0.0; 4]; 4];
    (0..4).for_each(|i| inv[i][i] = 1.0);
    // Columns of very different size (accelerations, ones): judge a pivot against its own column.
    let scale: [f64; 4] = std::array::from_fn(|i| m[i][i]);
    for col in 0..4 {
        let pivot = (col..4).max_by(|&a, &b| m[a][col].abs().total_cmp(&m[b][col].abs()))?;
        if m[pivot][col].abs() < 1e-9 * scale[col].abs() {
            return None;
        }
        m.swap(col, pivot);
        inv.swap(col, pivot);
        let p = m[col][col];
        for j in 0..4 {
            m[col][j] /= p;
            inv[col][j] /= p;
        }
        for row in (0..4).filter(|&r| r != col) {
            let f = m[row][col];
            for j in 0..4 {
                m[row][j] -= f * m[col][j];
                inv[row][j] -= f * inv[col][j];
            }
        }
    }
    Some(inv)
}

/// Least squares of force = mass * acceleration + viscous * speed + coulomb * sign(speed) + constant on
/// the rows where the axis moves: values and standard errors. Force and speed are averaged over the
/// window, the acceleration is the slope of the speed across it. Rows slower than 5 % of the top speed
/// are left out, because there friction is neither of the two terms. `None` if the axis does not move
/// both ways, since one direction alone cannot tell Coulomb friction from a constant force.
fn rigid_values(time: &[f64], force: &[f64], speed: &[f64]) -> Option<([f64; 4], [f64; 4])> {
    let (w, rows) = (WINDOW, time.len());
    let top = speed.iter().fold(0.0, |a: f64, b| a.max(b.abs()));
    let mean = |x: &[f64], i: usize| x[i - w..=i + w].iter().sum::<f64>() / (2 * w + 1) as f64;
    let mut samples = Vec::new();
    for i in w..rows.saturating_sub(w) {
        let v = mean(speed, i);
        if v.abs() > 0.05 * top {
            samples.push(([(speed[i + w] - speed[i - w]) / (time[i + w] - time[i - w]), v, v.signum(), 1.0], mean(force, i)));
        }
    }
    let one_way = |sign: f64| samples.iter().filter(|(x, _)| x[2] == sign).count();
    if one_way(1.0).min(one_way(-1.0)) < 10 * (2 * w + 1) {
        return None;
    }
    let (mut normal, mut right) = ([[0.0; 4]; 4], [0.0; 4]);
    for (x, y) in &samples {
        for a in 0..4 {
            right[a] += x[a] * y;
            for b in 0..4 {
                normal[a][b] += x[a] * x[b];
            }
        }
    }
    let inverse = invert(normal)?;
    let values: [f64; 4] = std::array::from_fn(|a| (0..4).map(|b| inverse[a][b] * right[b]).sum());
    let residual: f64 = samples.iter().map(|(x, y)| (y - (0..4).map(|a| values[a] * x[a]).sum::<f64>()).powi(2)).sum();
    // Neighbouring rows share most of their window, so only every (2w + 1)th row counts as a measurement.
    let variance = residual / (samples.len() - 4) as f64 * (2 * w + 1) as f64;
    Some((values, std::array::from_fn(|a| (variance * inverse[a][a]).sqrt())))
}

/// Stage 1: the rigid values of every axis from a trace, next to the config's. `None` for an axis the
/// trace does not move in both directions.
pub fn fit_rigid(entry: &MachineEntry, cfg: &Config, trace: &Trace) -> Result<Vec<Option<AxisFit>>, TraceError> {
    let simulated = replay(entry, cfg, trace)?;
    let machine = (entry.build)(&crate::catalog::resolve(entry, cfg)?)?;
    let views = axis_view(machine.as_ref(), trace).into_iter().zip(axis_view(machine.as_ref(), &simulated));
    Ok(views
        .map(|(measured, config)| {
            let (fitted, error) = rigid_values(&trace.time, &measured.0, &measured.1)?;
            Some(AxisFit { config: rigid_values(&trace.time, &config.0, &config.1)?.0, fitted, error })
        })
        .collect())
}

/// The load swinging on the belt against the motor, read from a frequency sweep.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Resonance {
    pub frequency: f64,
    /// Damping ratio from the width of the dip. Friction and the gearbox widen it too, so this is not
    /// the belt's damping alone.
    pub damping: f64,
    /// Half the distance between the sweep's points at the dip: the frequency is not known better.
    pub frequency_error: f64,
}

/// Stage 2: the resonance of the load on the belt, from a frequency sweep of motor torque to motor speed
/// (`frequency_sweep`, or a drive's measurement of the mechanical system). Where the load resonates the
/// motor hardly moves, which shows as a dip. Its frequency squared is stiffness over load mass, so with
/// the masses right, measured over config frequency, squared, is the factor the stiffness is off by.
///
/// A trace of a move does not do for this: the closed loops and friction damp the ringing, and the belt's
/// stiffness changes along the way. A sweep that does not have the dip and both its flanks gives `None`.
pub fn resonance(sweep: &[SweepPoint]) -> Option<Resonance> {
    // Gain times frequency is level for a pure inertia, so the dip is not hidden in the 1 / f slope.
    let level: Vec<f64> = sweep.iter().map(|p| p.gain * p.frequency).collect();
    let lowest = (0..level.len()).min_by(|&a, &b| level[a].total_cmp(&level[b]))?;
    // The flanks, where the response has twice the power of the lowest point, are steep: noise moves
    // them little, where it can move the lowest point by a whole step.
    let limit = level[lowest] * std::f64::consts::SQRT_2;
    let below = (0..lowest).rev().find(|&i| level[i] > limit)?;
    let above = (lowest + 1..level.len()).find(|&i| level[i] > limit)?;
    let crossing = |outside: usize, inside: usize| {
        let share = (limit - level[inside]) / (level[outside] - level[inside]);
        sweep[inside].frequency + share * (sweep[outside].frequency - sweep[inside].frequency)
    };
    let (low, high) = (crossing(below, below + 1), crossing(above, above - 1));
    let frequency = (low * high).sqrt();
    let step = (sweep[below + 1].frequency - sweep[below].frequency).max(sweep[above].frequency - sweep[above - 1].frequency);
    Some(Resonance { frequency, damping: (high - low) / (2.0 * frequency), frequency_error: 0.5 * step })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The dip of a response with a known resonance: gain * f = |fa^2 - f^2 + 2 j d fa f| has its lowest
    /// point at fa and twice that power where f^2 = fa^2 (1 -+ 2 d).
    #[test]
    fn resonance_reads_frequency_and_damping_off_a_known_dip() {
        let (fa, d) = (80.0f64, 0.05);
        let sweep: Vec<SweepPoint> = crate::analysis::log_spaced(50.0, 130.0, 60)
            .into_iter()
            .map(|f| SweepPoint { frequency: f, gain: (fa * fa - f * f).hypot(2.0 * d * fa * f) / f, phase_deg: 0.0 })
            .collect();
        let found = resonance(&sweep).unwrap();
        assert!((found.frequency - fa).abs() < found.frequency_error && found.frequency_error < 0.01 * fa, "{found:?}");
        assert!((found.damping - d).abs() < 0.05 * d, "{found:?}");
        // Half a dip is none.
        assert_eq!(resonance(&sweep[..25]), None);
    }
}
