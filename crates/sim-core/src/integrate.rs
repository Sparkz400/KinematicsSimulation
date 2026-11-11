//! Fixed-step integration. The core never looks at the wall clock.
//!
//! Classical Runge-Kutta 4 with sub-steps. The plan's default was a semi-implicit scheme; the
//! convergence and energy checks of the standard suite decide, and RK4 with 4 sub-steps passes them
//! without the core having to know which states are positions and which are velocities.

use crate::Machine;

pub struct Rk4 {
    k: [Vec<f64>; 4],
    tmp: Vec<f64>,
}

impl Rk4 {
    pub fn new(n: usize) -> Self {
        Self { k: std::array::from_fn(|_| vec![0.0; n]), tmp: vec![0.0; n] }
    }

    pub fn step(&mut self, mut f: impl FnMut(&[f64], &mut [f64]), y: &mut [f64], h: f64) {
        let [k1, k2, k3, k4] = &mut self.k;
        let tmp = &mut self.tmp;
        f(y, k1);
        for i in 0..y.len() {
            tmp[i] = y[i] + 0.5 * h * k1[i];
        }
        f(tmp, k2);
        for i in 0..y.len() {
            tmp[i] = y[i] + 0.5 * h * k2[i];
        }
        f(tmp, k3);
        for i in 0..y.len() {
            tmp[i] = y[i] + h * k3[i];
        }
        f(tmp, k4);
        for i in 0..y.len() {
            y[i] += h / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
        }
    }
}

/// Advances a machine by one base step `dt` with the torques held constant, as a drive would.
pub fn advance(machine: &dyn Machine, rk: &mut Rk4, state: &mut [f64], torque: &[f64], dt: f64, substeps: u32) {
    let h = dt / substeps as f64;
    for _ in 0..substeps {
        rk.step(|y, dy| machine.derivatives(y, torque, dy), state, h);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn harmonic_oscillator_matches_closed_form() {
        let mut rk = Rk4::new(2);
        let mut y = [1.0, 0.0];
        for _ in 0..1000 {
            rk.step(
                |y, dy| {
                    dy[0] = y[1];
                    dy[1] = -y[0];
                },
                &mut y,
                1e-3,
            );
        }
        assert!((y[0] - 1f64.cos()).abs() < 1e-12);
        assert!((y[1] + 1f64.sin()).abs() < 1e-12);
    }

    #[test]
    fn error_falls_with_fourth_order() {
        let err = |n: usize| {
            let mut rk = Rk4::new(1);
            let mut y = [1.0];
            for _ in 0..n {
                rk.step(|y, dy| dy[0] = -y[0], &mut y, 1.0 / n as f64);
            }
            (y[0] - (-1f64).exp()).abs()
        };
        let ratio = err(10) / err(20);
        assert!(ratio > 14.0 && ratio < 18.0, "ratio {ratio}");
    }
}
