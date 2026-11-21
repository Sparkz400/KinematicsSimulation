//! A machine of joints, each driven by a motor through a transmission (`joint::RotaryJoint`): the machine for every
//! mechanism that can say its dynamics in joint coordinates, M(q) q'' + h(q, q') = tau. A joint's coordinate is an angle
//! (rad) or a travel (m); its gearbox's ratio is motor radians per unit of it, so a ball screw is a gearbox of ratio
//! 2 pi / lead (times any reduction before it), a rack 1 / pinion radius, a linear motor 1 (its "rotor" the forcer, its
//! torques forces). Stiffness, backlash, friction and stops are those at the joint, in its unit.
//!
//! Rigid: the rotors turn with their joints times the ratio, their inertia counts at the joint times the ratio squared,
//! and each gearbox loses its efficiency on what passes through it (two passes, as on the delta). Elastic
//! (`effects.joint_elasticity`): the rotors are states of their own behind the gearboxes' springs.
//!
//! A mechanism (`Mechanism`) gives the load's dynamics, its kinematics from joints to the tool, its own signals and
//! rating checks and its scene. Everything else (states, losses, energy, torque breakdown, friction view, heating
//! gearboxes) is here, the same for all.

use crate::joint::RotaryJoint;
use crate::mech::ThermalFeedback;
use sim_core::{
    AxisInfo, ENERGY_PARTS, FRICTION_SIGNALS, FrameMomentum, FrictionContact, LOSS_PREFIX, Machine, MotorInfo,
    RatingCheck, Scene, SignalInfo, TORQUE_PARTS, ThermalPart,
};

/// Solves the n x n system `m` (row by row) x = b by Gaussian elimination with partial pivoting; `None` where `m` is
/// singular.
pub fn solve(m: &[f64], b: &[f64]) -> Option<Vec<f64>> {
    let (mut a, mut x) = (m.to_vec(), b.to_vec());
    solve_in_place(&mut a, &mut x).then_some(x)
}

/// `solve` without its copies: eliminates `a` in place and leaves the solution in `x`; `false` where `a` is singular or
/// the solution not finite (`a` and `x` are then of no use).
pub fn solve_in_place(a: &mut [f64], x: &mut [f64]) -> bool {
    let n = x.len();
    for c in 0..n {
        let Some(p) = (c..n).max_by(|&i, &j| a[i * n + c].abs().total_cmp(&a[j * n + c].abs())) else { return false };
        if !(a[p * n + c].abs() > 1e-300) {
            return false;
        }
        if p != c {
            for k in 0..n {
                a.swap(p * n + k, c * n + k);
            }
            x.swap(p, c);
        }
        for r in c + 1..n {
            let f = a[r * n + c] / a[c * n + c];
            for k in c..n {
                a[r * n + k] -= f * a[c * n + k];
            }
            x[r] -= f * x[c];
        }
    }
    for c in (0..n).rev() {
        let s: f64 = (c + 1..n).map(|k| a[c * n + k] * x[k]).sum();
        x[c] = (x[c] - s) / a[c * n + c];
    }
    x.iter().all(|v| v.is_finite())
}

/// The inverse of an n x n matrix (row by row), column by column with `solve`; `None` where it is singular.
pub fn invert(m: &[f64], n: usize) -> Option<Vec<f64>> {
    let mut inv = vec![0.0; n * n];
    for c in 0..n {
        let e: Vec<f64> = (0..n).map(|r| if r == c { 1.0 } else { 0.0 }).collect();
        let x = solve(m, &e)?;
        for r in 0..n {
            inv[r * n + c] = x[r];
        }
    }
    Some(inv)
}

/// The eigenvalues of the symmetric n x n matrix `a` (row by row), by Jacobi rotations.
pub fn eigenvalues(mut a: Vec<f64>, n: usize) -> Vec<f64> {
    for _ in 0..100 {
        let off: f64 = (0..n * n).filter(|k| k / n != k % n).map(|k| a[k].powi(2)).sum();
        let diag: f64 = (0..n).map(|k| a[k * n + k].powi(2)).sum();
        if !(off > 1e-30 * diag) {
            break;
        }
        for p in 0..n {
            for q in p + 1..n {
                if a[p * n + q] == 0.0 {
                    continue;
                }
                let theta = 0.5 * (a[q * n + q] - a[p * n + p]) / a[p * n + q];
                let t = if theta == 0.0 { 1.0 } else { theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt()) };
                let (c, s) = (1.0 / (t * t + 1.0).sqrt(), t / (t * t + 1.0).sqrt());
                for k in 0..n {
                    let (akp, akq) = (a[k * n + p], a[k * n + q]);
                    a[k * n + p] = c * akp - s * akq;
                    a[k * n + q] = s * akp + c * akq;
                }
                for k in 0..n {
                    let (apk, aqk) = (a[p * n + k], a[q * n + k]);
                    a[p * n + k] = c * apk - s * aqk;
                    a[q * n + k] = s * apk + c * aqk;
                }
            }
        }
    }
    (0..n).map(|k| a[k * n + k]).collect()
}

/// The 2-norm condition number of a rows x cols matrix (row by row): sqrt of the largest over the smallest eigenvalue
/// of J^T J; infinite where it is singular.
pub fn condition_number(j: &[f64], rows: usize, cols: usize) -> f64 {
    let jtj: Vec<f64> =
        (0..cols * cols).map(|rc| (0..rows).map(|k| j[k * cols + rc / cols] * j[k * cols + rc % cols]).sum()).collect();
    let eig = eigenvalues(jtj, cols);
    let (most, least) = (eig.iter().copied().fold(0.0, f64::max), eig.iter().copied().fold(f64::INFINITY, f64::min));
    if !(least > 1e-24 * most) { f64::INFINITY } else { (most / least).sqrt() }
}

/// `matrix_modes_at` with K and C diagonal: K's diagonal given, C's entry of coordinate r at the mode's w (a gear's
/// damper bridged by a Maxwell arm damps by frequency, `damping_at`).
fn generalized_modes(m: &[f64], k: &[f64], c: impl Fn(usize, f64) -> f64, n: usize) -> Vec<sim_core::Mode> {
    let diagonal = (0..n * n).map(|rc| if rc / n == rc % n { k[rc / n] } else { 0.0 }).collect::<Vec<_>>();
    matrix_modes_at(m, &diagonal, |r, col, w| if r == col { c(r, w) } else { 0.0 }, n)
}

/// `matrix_modes_at` with a constant C (symmetric, row by row).
pub fn matrix_modes(m: &[f64], k: &[f64], c: &[f64], n: usize) -> Vec<sim_core::Mode> {
    matrix_modes_at(m, k, |r, col, _| c[r * n + col], n)
}

/// The modes of M x'' + C x' + K x = 0 (`n` coordinates, M, K and C symmetric, row by row): w^2 of K x = w^2 M x, the
/// eigenvalues of L^-1 K L^-T (M = L L^T) by Jacobi rotations, C's entry (r, col) at a frequency given by `c`. `hz` is
/// that undamped w (what a shaper following the mode takes). The damping ratio is the damped problem's (`damped_ratio`):
/// -Re lambda / |lambda| of the eigenvalue of M lambda^2 + C lambda + K that belongs to the mode, C at |lambda|; where
/// that is not found (an overdamped mode, or two modes' eigenvalues not told apart) the estimate on the undamped shape,
/// x^T C(w) x / (2 w) with x^T M x = 1, which differs where C couples modes that lie close (C not proportional to M
/// and K). Empty where M is not positive definite.
fn matrix_modes_at(m: &[f64], k: &[f64], c: impl Fn(usize, usize, f64) -> f64, n: usize) -> Vec<sim_core::Mode> {
    let mut l = vec![0.0; n * n];
    for i in 0..n {
        for j in 0..=i {
            let s = m[i * n + j] - (0..j).map(|p| l[i * n + p] * l[j * n + p]).sum::<f64>();
            if i == j && !(s > 0.0) {
                return Vec::new();
            }
            l[i * n + j] = if i == j { s.sqrt() } else { s / l[j * n + j] };
        }
    }
    // w = L^-1, by forward substitution column by column.
    let mut w = vec![0.0; n * n];
    for j in 0..n {
        for i in 0..n {
            let s = f64::from(u8::from(i == j)) - (0..i).map(|p| l[i * n + p] * w[p * n + j]).sum::<f64>();
            w[i * n + j] = s / l[i * n + i];
        }
    }
    // W K, then (W K) W^T.
    let wk: Vec<f64> = (0..n * n).map(|rc| (0..n).map(|p| w[rc / n * n + p] * k[p * n + rc % n]).sum()).collect();
    let mut a: Vec<f64> = (0..n * n).map(|rc| (0..n).map(|p| wk[rc / n * n + p] * w[rc % n * n + p]).sum()).collect();
    let mut v: Vec<f64> = (0..n * n).map(|rc| f64::from(u8::from(rc / n == rc % n))).collect();
    for _ in 0..100 {
        let off: f64 = (0..n * n).filter(|rc| rc / n != rc % n).map(|rc| a[rc] * a[rc]).sum();
        let diag: f64 = (0..n).map(|i| a[i * n + i] * a[i * n + i]).sum();
        if !(off > 1e-30 * diag) {
            break;
        }
        for p in 0..n {
            for q in p + 1..n {
                if a[p * n + q] == 0.0 {
                    continue;
                }
                let theta = (a[q * n + q] - a[p * n + p]) / (2.0 * a[p * n + q]);
                let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
                let (cs, sn) = (1.0 / (t * t + 1.0).sqrt(), t / (t * t + 1.0).sqrt());
                for r in 0..n {
                    let (rp, rq) = (a[r * n + p], a[r * n + q]);
                    (a[r * n + p], a[r * n + q]) = (cs * rp - sn * rq, sn * rp + cs * rq);
                }
                for r in 0..n {
                    let (pr, qr) = (a[p * n + r], a[q * n + r]);
                    (a[p * n + r], a[q * n + r]) = (cs * pr - sn * qr, sn * pr + cs * qr);
                }
                for r in 0..n {
                    let (rp, rq) = (v[r * n + p], v[r * n + q]);
                    (v[r * n + p], v[r * n + q]) = (cs * rp - sn * rq, sn * rp + cs * rq);
                }
            }
        }
    }
    let undamped: Vec<(f64, f64)> = (0..n)
        .filter_map(|i| {
            let omega = a[i * n + i].sqrt();
            // x = L^-T y, y the eigenvector.
            let x: Vec<f64> = (0..n).map(|r| (0..n).map(|p| w[p * n + r] * v[p * n + i]).sum::<f64>()).collect();
            let damping =
                (0..n * n).map(|rc| x[rc / n] * c(rc / n, rc % n, omega) * x[rc % n]).sum::<f64>() / (2.0 * omega);
            (omega > 0.0 && omega.is_finite()).then_some((omega, damping))
        })
        .collect();
    let seeds: Vec<[f64; 2]> = undamped.iter().map(|&(w, z)| seed(w, z)).collect();
    // M^-1 = W^T W.
    let m_inv: Vec<f64> = (0..n * n).map(|rc| (0..n).map(|p| w[p * n + rc / n] * w[p * n + rc % n]).sum()).collect();
    undamped
        .iter()
        .map(|&(omega, estimate)| {
            let damping = damped_ratio(&m_inv, k, &c, n, omega, estimate, &seeds).unwrap_or(estimate);
            sim_core::Mode { hz: omega / std::f64::consts::TAU, damping }
        })
        .collect()
}

/// Where a mode of undamped frequency `w` and damping ratio `z` has its eigenvalue: -z w + i w sqrt(1 - z^2).
fn seed(w: f64, z: f64) -> [f64; 2] {
    [-z * w, w * (1.0 - z * z).max(0.0).sqrt()]
}

/// The damping ratio -Re lambda / |lambda| of the eigenvalue lambda of M lambda^2 + C(|lambda|) lambda + K (M^-1 given as
/// `m_inv`) that belongs to the mode of undamped frequency `w0` and estimated ratio `z0`: the eigenvalue of the companion
/// matrix [[0, I], [-M^-1 K, -M^-1 C]] nearest the mode's `seed` (`shapes::eigen_near`), C taken anew at |lambda| until
/// it holds. `None` for an estimate of 0 (nothing to find: lambda = i w0) or of 1 and more (overdamped), where the one
/// found is nearer another mode's seed of `seeds` than its own (two modes' eigenvalues not told apart), or where the
/// iteration does not settle.
fn damped_ratio(
    m_inv: &[f64],
    k: &[f64],
    c: &impl Fn(usize, usize, f64) -> f64,
    n: usize,
    w0: f64,
    z0: f64,
    seeds: &[[f64; 2]],
) -> Option<f64> {
    if !(z0 > 0.0 && z0 < 1.0) {
        return None;
    }
    let product = |b: &[f64]| -> Vec<f64> {
        (0..n * n).map(|rc| (0..n).map(|p| m_inv[rc / n * n + p] * b[p * n + rc % n]).sum()).collect()
    };
    let stiffness = product(k);
    let size = 2 * n;
    let own = seed(w0, z0);
    let (mut lambda, mut w) = (own, w0);
    for _ in 0..50 {
        let damper = product(&(0..n * n).map(|rc| c(rc / n, rc % n, w)).collect::<Vec<_>>());
        let mut a = vec![0.0; size * size];
        for r in 0..n {
            a[r * size + n + r] = 1.0;
            for col in 0..n {
                a[(n + r) * size + col] = -stiffness[r * n + col];
                a[(n + r) * size + n + col] = -damper[r * n + col];
            }
        }
        (lambda, _) = crate::shapes::eigen_near(&a, size, lambda)?;
        let next = lambda[0].hypot(lambda[1]);
        if (next - w).abs() <= 1e-12 * w {
            let distance = |s: &[f64; 2]| (lambda[0] - s[0]).hypot(lambda[1] - s[1]);
            let nearest = seeds.iter().map(distance).fold(f64::INFINITY, f64::min);
            return (lambda[1] > 0.0 && distance(&own) <= nearest).then(|| -lambda[0] / next);
        }
        w = next;
    }
    None
}

/// `solve`'s elimination of a fixed-size matrix, kept for several right-hand sides: each solve does the very operations
/// `solve` does, so the results are the same to the bit, without a heap allocation and with the matrix eliminated once.
pub struct Lu<const N: usize> {
    a: [[f64; N]; N],
    factor: [[f64; N]; N],
    pivot: [usize; N],
}

impl<const N: usize> Lu<N> {
    /// `None` where `m` is singular (as `solve`).
    pub fn new(m: &[[f64; N]; N]) -> Option<Self> {
        let (mut a, mut factor, mut pivot) = (*m, [[0.0; N]; N], [0; N]);
        for c in 0..N {
            let p = (c..N).max_by(|&i, &j| a[i][c].abs().total_cmp(&a[j][c].abs()))?;
            if !(a[p][c].abs() > 1e-300) {
                return None;
            }
            a.swap(p, c);
            pivot[c] = p;
            for r in c + 1..N {
                let f = a[r][c] / a[c][c];
                for k in c..N {
                    a[r][k] -= f * a[c][k];
                }
                factor[r][c] = f;
            }
        }
        Some(Self { a, factor, pivot })
    }

    /// x with m x = b; `None` where it is not finite (as `solve`).
    pub fn solve(&self, mut x: [f64; N]) -> Option<[f64; N]> {
        for c in 0..N {
            x.swap(self.pivot[c], c);
            for r in c + 1..N {
                x[r] -= self.factor[r][c] * x[c];
            }
        }
        for c in (0..N).rev() {
            let s: f64 = (c + 1..N).map(|k| self.a[c][k] * x[k]).sum();
            x[c] = (x[c] - s) / self.a[c][c];
        }
        x.iter().all(|v| v.is_finite()).then_some(x)
    }

    /// m^-1 (rows), column by column as `solve` would find each; `None` where a column is not finite.
    pub fn inverse(&self) -> Option<[[f64; N]; N]> {
        let mut inv = [[0.0; N]; N];
        for c in 0..N {
            let x = self.solve(std::array::from_fn(|r| if r == c { 1.0 } else { 0.0 }))?;
            for r in 0..N {
                inv[r][c] = x[r];
            }
        }
        Some(inv)
    }
}

/// Joints (driven and passive) whose load the elastic model's derivatives keep on the stack; more take the heap.
const ON_STACK: usize = 9;

/// What a mechanism tells the joint machine.
pub trait Mechanism: Send {
    /// The load's mass matrix M (n x n, row by row, without the rotors) and h with M q'' + h = the torques (forces) at
    /// the joints: velocity terms plus gravity. With the payload as it is attached. Where the mechanism has passive
    /// joints (`passive_joints`), q and q' hold them after the driven ones, and M and h are of them all.
    fn dynamics(&self, q: &[f64], qd: &[f64], mass: &mut [f64], bias: &mut [f64]);
    /// The load's potential energy (passive joints included, their springs not); its gravity torques are those of h at
    /// rest.
    fn potential(&self, q: &[f64]) -> f64;
    /// Joints that no motor drives, after the driven ones (a payload swinging on the flange): each with its spring
    /// (N m/rad) and damper (N m s/rad) towards zero. None by default.
    fn passive_joints(&self) -> Vec<(f64, f64)> {
        Vec::new()
    }
    /// The passive joints' position signals, one per joint in their order (their values are the joints' positions):
    /// by default those of a payload swinging on the flange, `swing.<k>.angle`.
    fn passive_signals(&self) -> Vec<SignalInfo> {
        (1..=self.passive_joints().len())
            .map(|k| SignalInfo::new(format!("swing.{k}.angle"), "group-payload-swing", "quantity-swing-angle", "rad"))
            .collect()
    }
    /// The id (`power.<id>`) and the owner's key of the loss row of the passive joints' dampers: by default the swinging
    /// payload's. `None` (no row) only where every passive joint's damper is zero.
    fn passive_loss_row(&self) -> Option<(&'static str, &'static str)> {
        Some(("payload_swing", "group-payload-swing"))
    }
    /// The passive joints' positions at rest with the driven ones at `q` (a contact pressed in by its load): all zero by
    /// default.
    fn passive_rest(&self, _q: &[f64]) -> Vec<f64> {
        vec![0.0; self.passive_joints().len()]
    }
    /// A loss row of the mechanism's own (a contact's damper, a guide's friction), booked beside the joints' and the
    /// two-way modes': its id (`power.<id>`) and the owner's key; `None` (no row) by default.
    fn loss_row(&self) -> Option<(&'static str, &'static str)> {
        None
    }
    /// The power the row of `loss_row` takes out, W, at all joints' q and qd (passive ones included): 0 by default.
    fn loss_power(&self, _q: &[f64], _qd: &[f64]) -> f64 {
        0.0
    }
    fn set_payload_attached(&mut self, attached: bool);
    /// Whether the motors are linear motors (their signals in m, m/s, N): `Machine::motor_linear`. False by default.
    fn linear_motor(&self) -> bool {
        false
    }
    /// As `Machine::links` and `Machine::set_link_change`: the bodies whose inertial parameters a trace can be fitted for.
    fn links(&self) -> Vec<String> {
        Vec::new()
    }
    fn set_link_change(&mut self, _link: usize, _factor: f64, _moment: [f64; 3]) {}

    /// Joints to tool pose. The joints themselves by default.
    fn forward(&self, q: &[f64], tool: &mut [f64]) {
        tool.copy_from_slice(q);
    }
    /// Tool pose to joints; `false` where it cannot be reached.
    fn inverse(&self, tool: &[f64], q: &mut [f64]) -> bool {
        q.copy_from_slice(tool);
        true
    }
    /// What `inverse` carries from one call to the next (a numerical inverse's last solution, which the next starts
    /// from), as a fixed count of numbers, for `Machine::memory`. None by default.
    fn memory(&self) -> Vec<f64> {
        Vec::new()
    }
    /// Sets back what `memory` gave (empty: none, as before the first `inverse`), as `inverse` keeps it: behind a lock.
    fn set_memory(&self, _memory: &[f64]) {}
    fn unreachable(&self, _tool: &[f64], _slack: f64) -> Option<&'static str> {
        None
    }
    /// `Machine::condition` at a tool pose; `None` where the tool is the joints.
    fn condition(&self, _tool: &[f64]) -> Option<f64> {
        None
    }
    /// Whether the tool pose is the joints themselves (`forward` and `inverse` as by default): then the joint speeds and
    /// accelerations are the tool's, without going through the Jacobian.
    fn tool_is_joints(&self) -> bool {
        true
    }

    /// Whether the mechanism works on a process (a press force, a cut), which `process_torque` gives: then the machine
    /// reports the power given to it as `power.process`. None by default.
    fn has_process(&self) -> bool {
        false
    }
    /// Adds the torques (forces) the process puts on the joints at positions `q` and speeds `qd` to `out`, positive
    /// turning a joint up. Not conservative and not in `dynamics`: the torque breakdown counts it among the rest, and
    /// the feedforward leaves it out, as a drive does not know it.
    fn process_torque(&self, _q: &[f64], _qd: &[f64], _out: &mut [f64]) {}

    /// As `Machine::cam` and `Machine::cam_at`: an electronic cam the mechanism runs as its cycle, its slaves the tool
    /// axes. None by default.
    fn cam(&self) -> Option<sim_core::CamMaster> {
        None
    }
    fn cam_at(&self, _master: f64, _out: &mut [[f64; 4]]) {}
    fn cam_curve(&self) -> Option<sim_core::CamCurve> {
        None
    }

    /// The mechanism's own signals, after the machine's: their infos, and their values from joint positions, speeds,
    /// accelerations and the force (torque) each transmission passes to its joint.
    fn signal_infos(&self) -> Vec<SignalInfo> {
        Vec::new()
    }
    fn signals(&self, _q: &[f64], _qd: &[f64], _qdd: &[f64], _force: &[f64], _out: &mut Vec<f64>) {}
    /// Whether a signal needs the joints' jerk q''' (`signals_with_jerk`): the chain works it out only then, as it costs
    /// three more evaluations of the machine's derivatives.
    fn reads_jerk(&self) -> bool {
        false
    }
    /// `signals` with the joints' jerk as well, for a mechanism that `reads_jerk`; zeros where it does not.
    fn signals_with_jerk(&self, q: &[f64], qd: &[f64], qdd: &[f64], _qddd: &[f64], force: &[f64], out: &mut Vec<f64>) {
        self.signals(q, qd, qdd, force, out);
    }
    fn rating_checks(&self) -> Vec<RatingCheck> {
        Vec::new()
    }
    /// As `Machine::rigid_guide_loads`: the loads of the guides among `signals`, at a tool pose and acceleration.
    fn rigid_guide_loads(&self, _tool: &[f64], _acc: &[f64], _out: &mut Vec<[f64; 2]>) {}

    /// The ids of the boxes `scene` draws that move with the load and can hit an obstacle (`Machine::collision_bodies`).
    fn collision_parts(&self) -> &'static [&'static str] {
        &[]
    }

    /// For effects.base_compliance (`Machine::frame_momentum`): the load's momentum at joints q and speeds qd (passive
    /// ones included), in the frame of the tool pose's x, y, z, each driven joint's rotor adding `rotor_spin` (its
    /// inertia times its speed: N m s, or N s for a linear motor's forcer) along its own axis. `None` by default.
    fn frame_momentum(&self, _q: &[f64], _qd: &[f64], _rotor_spin: &[f64]) -> Option<FrameMomentum> {
        None
    }
    /// Bending modes of the load's links (effects.link_bending): its motion drives them and they move the tool; the
    /// joints feel them only where `flex_coupling` gives them two-way, else not (one-way, as `base::Based`). Their count
    /// must not change. Empty by default.
    fn flex_modes(&self) -> Vec<FlexMode> {
        Vec::new()
    }
    /// The force on each of `flex_modes` at the driven joints' positions q, speeds qd and accelerations qdd, in its
    /// coordinate (N).
    fn flex_loads(&self, _q: &[f64], _qd: &[f64], _qdd: &[f64], _out: &mut [f64]) {}
    /// How far the tool pose moves at joints q when the modes deflect by `deflection`, into `out` (zeroed first).
    fn flex_shift(&self, _q: &[f64], _deflection: &[f64], _out: &mut [f64]) {}
    /// effects.link_bending_reaction: the modes two-way, coordinates of their own beside the joints, at all joints' q and
    /// qd (passive ones included) and the modes' deflections d and rates dd (`FlexCoupling`). `None` where they are
    /// one-way (`flex_loads`), the default.
    fn flex_coupling(&self, _q: &[f64], _qd: &[f64], _d: &[f64], _dd: &[f64]) -> Option<FlexCoupling> {
        None
    }

    /// Draws the machine at joint positions `q` (passive ones included) with the motor shafts at `rotors` (rad) turning
    /// at `rotor_speeds`.
    fn scene(&self, q: &[f64], rotors: &[f64], rotor_speeds: &[f64], out: &mut Scene);
    fn scene_point(&self, _tool: &[f64]) -> Option<[f64; 3]> {
        None
    }
}

/// A bending mode of a mechanism's load (`Mechanism::flex_modes`): its modal mass (kg), stiffness (N/m) and damping
/// (N s/m), in the coordinate of its deflection (a link's tip, m; a twist's angle, rad, and kg m^2, N m/rad), the link
/// (0 the first) it bends and how.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FlexMode {
    pub link: usize,
    pub mass: f64,
    pub stiffness: f64,
    pub damping: f64,
    pub kind: FlexKind,
}

/// How a link deflects in a `FlexMode`: in the plane its joint turns it in, across that plane, or twisted about its
/// length.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlexKind {
    Bend,
    BendAcross,
    Twist,
}

impl FlexKind {
    /// Its signal's id after `link.<n>.`, the quantity's translation key and its unit.
    fn signal(self) -> (&'static str, &'static str, &'static str) {
        match self {
            FlexKind::Bend => ("bend", "quantity-link-bend", "m"),
            FlexKind::BendAcross => ("bend_across", "quantity-link-bend-across", "m"),
            FlexKind::Twist => ("twist", "quantity-link-twist", "rad"),
        }
    }
}

/// The two-way bending modes (`Mechanism::flex_coupling`) at one moment, with T = 1/2 q'^T M q' + d'^T p +
/// 1/2 d'^T M_d d' their share of the kinetic energy and V = -d^T Q_g of the potential:
///
/// ```text
/// M q'' + B^T d'' = tau - h + G         G = d (d^T Q_g) / dq, the joints' load as the bending moves the weights
/// B q'' + M_d d'' = F - K d - C d'      F = Q_g - B' q' - 1/2 M_d' d'
/// ```
///
/// B' q' and M_d' along the motion, so that T + V + 1/2 d^T K d changes by the joints' work less C d'^2 alone.
#[derive(Debug, Clone, PartialEq)]
pub struct FlexCoupling {
    /// M_d, modes x modes, row by row.
    pub mass: Vec<f64>,
    /// B, modes x joints (all, passive ones included), row by row.
    pub coupling: Vec<f64>,
    /// p = B q', per mode.
    pub momentum: Vec<f64>,
    /// F, per mode.
    pub force: Vec<f64>,
    /// G, per joint.
    pub joint: Vec<f64>,
    /// V = -d^T Q_g.
    pub potential: f64,
}

/// One joint as the machine sees it: its drive, the joint's id and translation key (for friction contacts and signals),
/// and where it has a gearbox ahead of a converter (screw, rack): the joint's travel per radian of the gearbox output,
/// so that the torque at the gearbox output is the joint force times it (signal `gearbox.<motor>.output_torque`).
#[derive(Debug, Clone)]
pub struct JointSpec {
    pub drive: RotaryJoint,
    pub id: &'static str,
    pub key: &'static str,
    pub gear_lever: Option<f64>,
    /// The joint's unit: "m" for a travel, "rad" for an angle (the axes may be the tool's, in other units).
    pub unit: &'static str,
}

pub struct JointMachine<M: Mechanism> {
    pub mechanism: M,
    pub joints: Vec<JointSpec>,
    pub elastic: bool,
    pub thermal: Option<ThermalFeedback>,
    /// Per joint, the efficiency its own gearbox has at the reference temperature, which `thermal`'s law warms from:
    /// empty where every joint has the shared `[gearbox]`'s (`ThermalFeedback::efficiency`). A machine whose joints have
    /// gearboxes of their own (`[gearbox.<k>]`) sets it.
    pub gear_efficiencies: Vec<f64>,
    pub axes: Vec<AxisInfo>,
    pub motors: Vec<MotorInfo>,
    brake_engaged: bool,
    /// `Machine::setpoint_slack`, worked out when first asked for (`rest_sag`, 1.4 s on the seven-joint arm).
    rest_slack: std::sync::OnceLock<f64>,
    /// Whether the payload is attached, as `set_payload_attached` last had it (attached as built).
    payload_attached: bool,
    /// `Mechanism::passive_joints`: their angles and speeds follow the joint machine's own states.
    passive: Vec<(f64, f64)>,
    /// `Mechanism::passive_loss_row`, where there are passive joints.
    passive_row: Option<(&'static str, &'static str)>,
    /// How many bending modes the mechanism has (`Mechanism::flex_modes`); their deflections and rates end the state,
    /// after the passive joints'.
    flex: usize,
    /// Whether the modes are two-way (`Mechanism::flex_coupling`).
    two_way: bool,
}

/// The losses of one moment: per joint its friction, its motor's friction with the brake, its gearbox; then the stops
/// and what the ripple gives off. And per joint the gearbox loss as a torque at the rotor.
struct Losses {
    powers: Vec<f64>,
    gear_at_rotor: Vec<f64>,
}

impl<M: Mechanism> JointMachine<M> {
    /// `axes` as many as joints, or fewer where the joints are redundant to the tool (a seven-joint arm whose tool has
    /// six axes: the mechanism's `inverse` resolves the redundancy); one motor per joint, in their order.
    pub fn new(
        mechanism: M,
        joints: Vec<JointSpec>,
        elastic: bool,
        thermal: Option<ThermalFeedback>,
        axes: Vec<AxisInfo>,
        motors: Vec<MotorInfo>,
    ) -> Self {
        assert!(axes.len() <= joints.len() && joints.len() == motors.len());
        let (passive, flex) = (mechanism.passive_joints(), mechanism.flex_modes().len());
        // Without a row the dampers' heat would be lost from the budget.
        let passive_row = if passive.is_empty() { None } else { mechanism.passive_loss_row() };
        assert!(passive_row.is_some() || passive.iter().all(|(_, c)| *c == 0.0));
        let rest = vec![0.0; joints.len() + passive.len()];
        let two_way = flex > 0 && mechanism.flex_coupling(&rest, &rest, &vec![0.0; flex], &vec![0.0; flex]).is_some();
        Self {
            mechanism,
            joints,
            elastic,
            thermal,
            gear_efficiencies: Vec::new(),
            axes,
            motors,
            brake_engaged: false,
            rest_slack: std::sync::OnceLock::new(),
            payload_attached: true,
            passive,
            passive_row,
            flex,
            two_way,
        }
    }

    /// `rest_sag` as it is at the build: worked out on first use, before anything it reads changes (the payload, a
    /// link's mass), from the mechanism's inverse as built (no last solution), whose own memory it leaves as it was.
    fn rest_slack(&self) -> f64 {
        *self.rest_slack.get_or_init(|| {
            let kept = self.mechanism.memory();
            self.mechanism.set_memory(&[]);
            let slack = self.rest_sag();
            self.mechanism.set_memory(&kept);
            slack
        })
    }

    /// How far the motors' pose can be from the tool's at rest, as the delta works it out: each gearbox twisted by the
    /// torque it holds against gravity over its stiffness, plus half its backlash, through the forward kinematics; at
    /// the corners and the centre of the axes' box, with the payload as it is (attached at the start); plus a tenth,
    /// at least the default 0.1 mm. That alone in the rigid model, whose gearboxes do not twist. An arm's gears under
    /// its weight: some mrad.
    fn rest_sag(&self) -> f64 {
        let n = self.n();
        if !self.elastic {
            return 1e-4;
        }
        // Five places along each axis, every combination: the sag is largest where a joint's weight arm is (where the
        // links lie level), which a corner of the box need not be (the arm's second joint at its lower end with the
        // first one between its ends sags 3.71 mrad, the corners and centre gave 2.77).
        const LEVELS: usize = 5;
        let m = self.axes.len();
        let poses: Vec<Vec<f64>> = (0..LEVELS.pow(m as u32))
            .map(|c| {
                (self.axes.iter().enumerate())
                    .map(|(i, a)| {
                        a.min + (a.max - a.min) * (c / LEVELS.pow(i as u32) % LEVELS) as f64 / (LEVELS - 1) as f64
                    })
                    .collect()
            })
            .collect();
        let (mut q, mut tool, mut sagged) = (vec![0.0; n], vec![0.0; m], vec![0.0; m]);
        let mut most = 0f64;
        for pose in poses {
            if !self.mechanism.inverse(&pose, &mut q) {
                continue;
            }
            let (_, gravity) = self.load(&q, &vec![0.0; n]);
            // A process force that holds at rest (a press's force through a dwell) twists the gearbox too; the
            // mechanisms whose process acts only in motion put none here.
            let process = self.process(&q, &vec![0.0; n]);
            let twisted: Vec<f64> = (0..n)
                .map(|k| q[k] + self.joints[k].drive.gearbox.static_twist_at(q[k], gravity[k] - process[k]))
                .collect();
            self.mechanism.forward(&q, &mut tool);
            self.mechanism.forward(&twisted, &mut sagged);
            most = most.max(tool.iter().zip(&sagged).map(|(a, b)| (a - b).abs()).fold(0.0, f64::max));
        }
        f64::max(1e-4, 1.1 * most)
    }

    fn n(&self) -> usize {
        self.joints.len()
    }

    /// Where the bending modes' deflections start in the state, after the passive joints'; their rates follow them.
    fn flex_at(&self) -> usize {
        self.passive_at() + 2 * self.passive.len()
    }

    /// The bending modes' deflections and rates.
    fn flex_state<'a>(&self, s: &'a [f64]) -> (&'a [f64], &'a [f64]) {
        let (at, m) = (self.flex_at(), self.flex);
        (&s[at..at + m], &s[at + m..at + 2 * m])
    }

    /// The bending modes' rates into `out`, from the joints' accelerations `acc` (the driven ones first): m d'' + c d' +
    /// k d = load.
    /// Two-way, `acc` carries the modes' accelerations after the joints' (`flex_solve`).
    fn flex_rates(&self, s: &[f64], acc: &[f64], out: &mut [f64]) {
        if self.flex == 0 {
            return;
        }
        let joints = self.n() + self.passive.len();
        if acc.len() > joints {
            let ((_, rate), at, m) = (self.flex_state(s), self.flex_at(), self.flex);
            out[at..at + m].copy_from_slice(rate);
            out[at + m..at + 2 * m].copy_from_slice(&acc[joints..]);
            return;
        }
        let (q, qd) = self.arms(s);
        let mut load = vec![0.0; self.flex];
        self.mechanism.flex_loads(q, qd, &acc[..q.len()], &mut load);
        let ((d, rate), at, m) = (self.flex_state(s), self.flex_at(), self.flex);
        for (k, mode) in self.mechanism.flex_modes().iter().enumerate() {
            out[at + k] = rate[k];
            out[at + m + k] = (load[k] - mode.stiffness * d[k] - mode.damping * rate[k]) / mode.mass;
        }
    }

    /// The two-way modes' coupling at state `s` (`Mechanism::flex_coupling`); `None` where they are one-way.
    fn flex_coupling(&self, s: &[f64]) -> Option<FlexCoupling> {
        if !self.two_way {
            return None;
        }
        let (q, qd) = self.all_joints(s);
        let (d, rate) = self.flex_state(s);
        self.mechanism.flex_coupling(&q, &qd, d, rate)
    }

    /// The joints' accelerations from their `mass` and `rhs` (all joints, M q'' = rhs); two-way (`coupling`, see
    /// `FlexCoupling`) solved together with the modes', which follow the joints'. NaN where singular.
    fn flex_solve(&self, coupling: Option<&FlexCoupling>, s: &[f64], mass: &[f64], rhs: &[f64]) -> Vec<f64> {
        let Some(c) = coupling else {
            return solve(mass, rhs).unwrap_or_else(|| vec![f64::NAN; rhs.len()]);
        };
        let (all, f) = (rhs.len(), self.flex);
        let size = all + f;
        let ((d, rate), modes) = (self.flex_state(s), self.mechanism.flex_modes());
        let (mut m, mut b) = (vec![0.0; size * size], vec![0.0; size]);
        for r in 0..all {
            m[r * size..r * size + all].copy_from_slice(&mass[r * all..(r + 1) * all]);
            b[r] = rhs[r] + c.joint[r];
        }
        for i in 0..f {
            for j in 0..all {
                m[(all + i) * size + j] = c.coupling[i * all + j];
                m[j * size + all + i] = c.coupling[i * all + j];
            }
            m[(all + i) * size + all..(all + i + 1) * size].copy_from_slice(&c.mass[i * f..(i + 1) * f]);
            b[all + i] = c.force[i] - modes[i].stiffness * d[i] - modes[i].damping * rate[i];
        }
        solve(&m, &b).unwrap_or_else(|| vec![f64::NAN; size])
    }

    /// The two-way modes with the motors held (`Machine::modes`): the joints on their gearboxes' springs where they are
    /// elastic (held where rigid; passive joints held), the modes on theirs, coupled through the mass, K x = w^2 M x.
    /// Without `damped` the frequencies alone (damping 0), as `resonance_hz` wants them.
    fn coupled_modes(&self, q: &[f64], damped: bool) -> Vec<sim_core::Mode> {
        let (n, f) = (self.n(), self.flex);
        let all = n + self.passive.len();
        let rest = |len: usize| vec![0.0; len];
        let mut qa = q.to_vec();
        qa.resize(all, 0.0);
        let Some(c) = self.mechanism.flex_coupling(&qa, &rest(all), &rest(f), &rest(f)) else { return Vec::new() };
        let modes = self.mechanism.flex_modes();
        let free = if self.elastic { n } else { 0 };
        let size = free + f;
        let (load, _) = self.load(q, &rest(n));
        let (mut m, mut k, mut damping) = (rest(size * size), rest(size), rest(size));
        for r in 0..free {
            m[r * size..r * size + free].copy_from_slice(&load[r * n..r * n + free]);
            k[r] = self.joints[r].drive.gearbox.at(q[r]).stiffness;
        }
        for i in 0..f {
            for j in 0..free {
                m[(free + i) * size + j] = c.coupling[i * all + j];
                m[j * size + free + i] = c.coupling[i * all + j];
            }
            m[(free + i) * size + free..(free + i + 1) * size].copy_from_slice(&c.mass[i * f..(i + 1) * f]);
            (k[free + i], damping[free + i]) = (modes[i].stiffness, modes[i].damping);
        }
        let c = |r: usize, w: f64| match r {
            _ if !damped => 0.0,
            r if r < free => self.joints[r].drive.gearbox.at(q[r]).damping_at(w),
            r => damping[r],
        };
        generalized_modes(&m, &k, c, size)
    }

    /// An arm's joints on their gearboxes' springs with the motors held, at joints q (`Machine::modes`): K x = w^2 M x
    /// with M the load's mass matrix there (passive joints held, the rotors left out: a velocity loop far stiffer than
    /// the mode holds them) and K, C the gearboxes' stiffnesses and dampings. Empty for one joint (`two_mass_frequency`
    /// tells its mode) and in the rigid model. With the links' bending two-way these are among `coupled_modes`. Without
    /// `damped` the frequencies alone (damping 0).
    fn held_modes(&self, q: &[f64], damped: bool) -> Vec<sim_core::Mode> {
        let n = self.n();
        if n < 2 || !self.elastic {
            return Vec::new();
        }
        let (load, _) = self.load(q, &vec![0.0; n]);
        let gear = |k: usize| &self.joints[k].drive.gearbox;
        let k: Vec<f64> = (0..n).map(|j| gear(j).at(q[j]).stiffness).collect();
        let c = |j: usize, w: f64| if damped { gear(j).at(q[j]).damping_at(w) } else { 0.0 };
        generalized_modes(&load[..n * n], &k, c, n)
    }

    /// The tool's move by the bending modes at joints q, deflections d: `Mechanism::flex_shift`.
    fn flex_shift(&self, q: &[f64], d: &[f64]) -> Vec<f64> {
        let mut out = vec![0.0; self.axes.len()];
        if self.flex > 0 {
            self.mechanism.flex_shift(q, d, &mut out);
        }
        out
    }

    /// Where the gears' Maxwell arms' states start (`Gearbox::arm`), after the bending modes' rates: one each for the
    /// elastic joints whose gear has an arm (a screw or rack with a damped gear before it).
    fn arm_at(&self) -> usize {
        self.flex_at() + 2 * self.flex
    }

    /// Joint k's arm's place in the state, if its gear has one.
    fn arm_slot(&self, k: usize) -> Option<usize> {
        let has = |i: usize| self.elastic && self.joints[i].drive.gearbox.relax_stiffness > 0.0;
        has(k).then(|| self.arm_at() + (0..k).filter(|&i| has(i)).count())
    }

    /// Joint k's arm's state, 0 where its gear has none (`Gearbox::torque_with` then leaves it out).
    fn arm(&self, s: &[f64], k: usize) -> f64 {
        self.arm_slot(k).map_or(0.0, |i| s[i])
    }

    /// Joint positions and speeds of a state.
    fn arms<'a>(&self, s: &'a [f64]) -> (&'a [f64], &'a [f64]) {
        let n = self.n();
        (&s[..n], &s[n..2 * n])
    }

    /// Rotor angles and speeds.
    fn rotors(&self, s: &[f64]) -> (Vec<f64>, Vec<f64>) {
        let n = self.n();
        if self.elastic {
            (s[2 * n..3 * n].to_vec(), s[3 * n..4 * n].to_vec())
        } else {
            (0..n).map(|k| (s[k] * self.joints[k].drive.ratio(), s[n + k] * self.joints[k].drive.ratio())).unzip()
        }
    }

    fn brakes<'a>(&self, s: &'a [f64]) -> &'a [f64] {
        let n = self.n();
        let at = if self.elastic { 4 * n } else { 2 * n };
        &s[at..at + n]
    }

    /// The process's torques at the joints (`Mechanism::process_torque`).
    fn process(&self, q: &[f64], qd: &[f64]) -> Vec<f64> {
        let mut out = vec![0.0; self.n()];
        self.mechanism.process_torque(q, qd, &mut out);
        out
    }

    /// The driven joints' part of the load with the passive joints held at rest at zero: what the rigid feedforward
    /// and the poses at rest see (the payload as if fixed to the flange).
    fn load(&self, q: &[f64], qd: &[f64]) -> (Vec<f64>, Vec<f64>) {
        let (n, p) = (self.n(), self.passive.len());
        if p == 0 {
            let (mut mass, mut bias) = (vec![0.0; n * n], vec![0.0; n]);
            self.mechanism.dynamics(q, qd, &mut mass, &mut bias);
            return (mass, bias);
        }
        let rest = |x: &[f64]| -> Vec<f64> { x.iter().copied().chain(std::iter::repeat_n(0.0, p)).collect() };
        let (mass, bias) = self.load_all(&rest(q), &rest(qd));
        let all = n + p;
        ((0..n * n).map(|rc| mass[rc / n * all + rc % n]).collect(), bias[..n].to_vec())
    }

    /// The load of all joints, passive ones included.
    fn load_all(&self, q: &[f64], qd: &[f64]) -> (Vec<f64>, Vec<f64>) {
        let all = q.len();
        let (mut mass, mut bias) = (vec![0.0; all * all], vec![0.0; all]);
        self.mechanism.dynamics(q, qd, &mut mass, &mut bias);
        (mass, bias)
    }

    /// Where the passive joints' angles start in the state; their speeds follow them.
    fn passive_at(&self) -> usize {
        self.n() * if self.elastic { 5 } else { 3 }
    }

    /// All joints' positions and speeds, passive ones after the driven ones.
    fn all_joints(&self, s: &[f64]) -> (Vec<f64>, Vec<f64>) {
        let (p, at) = (self.passive.len(), self.passive_at());
        let (q, qd) = self.arms(s);
        (q.iter().chain(&s[at..at + p]).copied().collect(), qd.iter().chain(&s[at + p..at + 2 * p]).copied().collect())
    }

    /// The passive joints' spring and damper torques.
    fn passive_torque(&self, qp: &[f64], qdp: &[f64]) -> Vec<f64> {
        self.passive.iter().zip(qp.iter().zip(qdp)).map(|((k, c), (q, qd))| -k * q - c * qd).collect()
    }

    /// Rigid model: the joints' accelerations (passive ones after the driven ones), and per driven joint the torque
    /// into its gearbox (after the rotor's own acceleration) and out of it, at the joint.
    fn rigid_solve(&self, s: &[f64], torque: &[f64]) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
        let (n, p) = (self.n(), self.passive.len());
        let all = n + p;
        let (q, qd) = self.arms(s);
        let (rotor, rotor_speed) = self.rotors(s);
        let brakes = self.brakes(s);
        let joints;
        let (qa, qda) = if p == 0 {
            (q, qd)
        } else {
            joints = self.all_joints(s);
            (&joints.0[..], &joints.1[..])
        };
        let (mass, bias) = self.load_all(qa, qda);
        let springs = self.passive_torque(&qa[n..], &qda[n..]);
        let drive: Vec<f64> = (0..n)
            .map(|k| {
                self.joints[k].drive.rigid_drive(torque[k], rotor[k], rotor_speed[k], brakes[k], self.brake_engaged).0
            })
            .collect();
        let process = self.process(q, qd);
        let rest: Vec<f64> = (0..n)
            .map(|k| self.joints[k].drive.output_torque(q[k], qd[k]) - bias[k] + process[k])
            .chain((0..p).map(|i| springs[i] - bias[n + i]))
            .collect();
        let coupling = self.flex_coupling(s);
        let solve_with = |loss: &[f64]| -> Vec<f64> {
            let mut m = mass.clone();
            for k in 0..n {
                m[k * all + k] += loss[k] * self.joints[k].drive.reflected_inertia();
            }
            let mut x: Vec<f64> = (0..all).map(|k| if k < n { loss[k] * drive[k] } else { 0.0 } + rest[k]).collect();
            // Without two-way bending the plain solve, in place; with it the links' modes beside the joints.
            match coupling.as_ref() {
                None => {
                    if !solve_in_place(&mut m, &mut x) {
                        x.fill(f64::NAN);
                    }
                    x
                }
                Some(c) => self.flex_solve(Some(c), s, &m, &x),
            }
        };
        // The loss factors of the torques going in, at their fixed point (`mech::settled_loss`).
        let loss = crate::mech::settled_loss(vec![1.0; n], |loss| {
            let acc = solve_with(loss);
            (0..n)
                .map(|k| {
                    self.joints[k]
                        .drive
                        .gearbox
                        .loss_factor(drive[k] - self.joints[k].drive.reflected_inertia() * acc[k], qd[k])
                })
                .collect()
        });
        let acc = solve_with(&loss);
        let into: Vec<f64> = (0..n).map(|k| drive[k] - self.joints[k].drive.reflected_inertia() * acc[k]).collect();
        let out = (0..n).map(|k| loss[k] * into[k]).collect();
        (acc, into, out)
    }

    /// Joint accelerations (passive ones after the driven ones) and the force (torque) each transmission passes to its
    /// joint, in either model.
    fn motion(&self, s: &[f64], torque: &[f64]) -> (Vec<f64>, Vec<f64>) {
        let n = self.n();
        if self.elastic {
            let mut d = vec![0.0; self.state_len()];
            self.derivatives(s, torque, &mut d);
            let gear = (0..n)
                .map(|k| {
                    self.joints[k]
                        .drive
                        .gearbox
                        .torque_with(s[2 * n + k], s[3 * n + k], s[k], s[n + k], self.arm(s, k))
                        .output
                })
                .collect();
            let at = self.passive_at() + self.passive.len();
            (d[n..2 * n].iter().chain(&d[at..at + self.passive.len()]).copied().collect(), gear)
        } else {
            let (acc, _, out) = self.rigid_solve(s, torque);
            (acc, out)
        }
    }

    /// The joints' jerk (passive ones after the driven ones) at the motor torque held: the rate of `motion`'s
    /// accelerations along the state's rate, a central difference over 1 us. Exact with elastic joints, whose
    /// accelerations follow from the state alone; with rigid ones it leaves out the drive's torque rate, which the
    /// machine does not know.
    fn joint_jerk(&self, s: &[f64], torque: &[f64]) -> Vec<f64> {
        let (mut rate, h) = (vec![0.0; s.len()], 1e-6);
        self.derivatives(s, torque, &mut rate);
        let at = |sign: f64| {
            let moved: Vec<f64> = s.iter().zip(&rate).map(|(x, r)| x + sign * h * r).collect();
            self.motion(&moved, torque).0
        };
        at(1.0).iter().zip(at(-1.0)).map(|(up, down)| (up - down) / (2.0 * h)).collect()
    }

    fn energy_parts(&self, s: &[f64]) -> [f64; 3] {
        let (q, _) = self.all_joints(s);
        self.energy_parts_with(s, &self.load_all(&q, &vec![0.0; q.len()]).0)
    }

    /// `energy_parts` with the load's mass matrix at rest (all joints, `load_all` at no speed) known.
    fn energy_parts_with(&self, s: &[f64], mass: &[f64]) -> [f64; 3] {
        let n = self.n();
        let (q, qd) = self.all_joints(s);
        let all = q.len();
        let mut kinetic =
            0.5 * (0..all).map(|i| (0..all).map(|j| qd[i] * mass[i * all + j] * qd[j]).sum::<f64>()).sum::<f64>();
        let mut potential = self.mechanism.potential(&q);
        let mut elastic: f64 = (0..n).map(|k| self.joints[k].drive.stop_energy(q[k])).sum::<f64>()
            + self.passive.iter().zip(&q[n..]).map(|((k, _), q)| 0.5 * k * q * q).sum::<f64>();
        // Two-way modes (`FlexCoupling`): d'^T p + 1/2 d'^T M_d d', -d^T Q_g and their springs.
        if let Some(c) = self.flex_coupling(s) {
            let ((d, rate), f) = (self.flex_state(s), self.flex);
            for (i, mode) in self.mechanism.flex_modes().iter().enumerate() {
                let own: f64 = (0..f).map(|k| c.mass[i * f + k] * rate[k]).sum();
                kinetic += rate[i] * (c.momentum[i] + 0.5 * own);
                elastic += 0.5 * mode.stiffness * d[i] * d[i];
            }
            potential += c.potential;
        }
        for k in 0..n {
            let j = &self.joints[k].drive;
            if self.elastic {
                kinetic += 0.5 * j.rotor_inertia * s[3 * n + k] * s[3 * n + k];
                elastic += j.gearbox.energy_with(s[2 * n + k], s[k], self.arm(s, k));
            } else {
                kinetic += 0.5 * j.reflected_inertia() * qd[k] * qd[k];
            }
        }
        [kinetic, potential, elastic]
    }

    fn losses(&self, s: &[f64], torque: &[f64]) -> Losses {
        let n = self.n();
        let (q, qd) = self.arms(s);
        let (rotor, rotor_speed) = self.rotors(s);
        let brakes = self.brakes(s);
        let row = usize::from(self.passive_row.is_some());
        let (mut powers, mut gear_at_rotor) = (vec![0.0; 3 * n + 2 + row], vec![0.0; n]);
        if row > 0 {
            // The passive joints' dampers.
            let (_, qd) = self.all_joints(s);
            powers[3 * n + 2] = self.passive.iter().zip(&qd[n..]).map(|((_, c), w)| c * w * w).sum();
        }
        let rigid = (!self.elastic).then(|| self.rigid_solve(s, torque));
        for k in 0..n {
            let j = &self.joints[k].drive;
            let [friction, stop, shaft, drag] = j.losses(q[k], qd[k], rotor_speed[k], brakes[k], self.brake_engaged);
            let gear = match &rigid {
                Some((_, into, out)) => {
                    gear_at_rotor[k] = (into[k] - out[k]) / j.ratio();
                    (into[k] - out[k]) * qd[k]
                }
                None => {
                    let u = self.arm(s, k);
                    let g = j.gearbox.torque_with(s[2 * n + k], s[3 * n + k], s[k], s[n + k], u);
                    gear_at_rotor[k] = g.rotor - g.output / j.ratio();
                    j.gearbox.loss_power_with(s[2 * n + k], s[3 * n + k], s[k], s[n + k], u)
                }
            };
            powers[3 * k..3 * k + 3].copy_from_slice(&[friction, shaft, gear + drag]);
            powers[3 * n] += stop;
            powers[3 * n + 1] += (torque[k] - j.ripple.apply(torque[k], rotor[k])) * rotor_speed[k];
        }
        if self.mechanism.has_process() {
            powers.push(-self.process(q, qd).iter().zip(qd).map(|(t, v)| t * v).sum::<f64>());
        }
        if self.two_way {
            let rate = self.flex_state(s).1;
            powers.push(self.mechanism.flex_modes().iter().zip(rate).map(|(m, v)| m.damping * v * v).sum());
        }
        if self.mechanism.loss_row().is_some() {
            let (q, qd) = self.all_joints(s);
            powers.push(self.mechanism.loss_power(&q, &qd));
        }
        Losses { powers, gear_at_rotor }
    }

    /// The torque breakdown, loss powers, energies and friction signals of the effect views.
    fn view_signals(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        let n = self.n();
        let (q, qd) = self.arms(s);
        let (_, rotor_speed) = self.rotors(s);
        let brakes = self.brakes(s);
        let (qa, qda) = self.all_joints(s);
        let all = qa.len();
        let (mass, bias) = self.load_all(&qa, &qda);
        let (at_rest, gravity) = self.load_all(&qa, &vec![0.0; all]);
        let (acc, _) = self.motion(s, torque);
        let losses = self.losses(s, torque);
        for k in 0..n {
            let j = &self.joints[k].drive;
            let i = j.ratio();
            let moving: f64 = (0..all).map(|c| mass[k * all + c] * acc[c]).sum();
            let held = j.brake.torque(self.brake_engaged, rotor_speed[k], brakes[k]).0;
            let parts = [
                (moving + bias[k] - gravity[k] + j.reflected_inertia() * acc[k]) / i,
                gravity[k] / i,
                j.shaft.torque(rotor_speed[k]) + held + losses.gear_at_rotor[k] + j.friction.torque_at(q[k], qd[k]) / i,
            ];
            out.extend(parts);
            out.push(torque[k] - parts.iter().sum::<f64>());
        }
        out.extend(&losses.powers);
        out.extend(self.energy_parts_with(s, &at_rest));
        for k in 0..n {
            out.extend(self.joints[k].drive.friction_signals(qd[k]));
        }
    }

    /// The joint's unit and the unit of what drives it: rad and N m, or m and N.
    fn units(&self, k: usize) -> (&'static str, &'static str) {
        if self.joints[k].unit == "m" { ("m/s", "N") } else { ("rad/s", "N m") }
    }

    fn effect_infos(&self, out: &mut Vec<SignalInfo>) {
        for (k, motor) in self.motors.iter().enumerate() {
            let unit = if self.motor_linear(k) { "N" } else { "N m" };
            out.extend(TORQUE_PARTS.map(|part| {
                SignalInfo::new(
                    format!("motor.{}.torque_{part}", motor.id),
                    motor.key,
                    &format!("quantity-torque-{part}"),
                    unit,
                )
            }));
        }
        for (k, motor) in self.motors.iter().enumerate() {
            let gear = if self.joints[k].drive.coupled {
                "quantity-coupling-loss-power"
            } else {
                "quantity-gearbox-loss-power"
            };
            out.push(SignalInfo::new(
                format!("{LOSS_PREFIX}joint_{}", motor.id),
                self.joints[k].key,
                "quantity-friction-power",
                "W",
            ));
            out.push(SignalInfo::new(
                format!("{LOSS_PREFIX}shaft_{}", motor.id),
                motor.key,
                "quantity-shaft-friction-power",
                "W",
            ));
            out.push(SignalInfo::new(format!("{LOSS_PREFIX}gearbox_{}", motor.id), motor.key, gear, "W"));
        }
        out.push(SignalInfo::new(format!("{LOSS_PREFIX}hard_stops"), "group-joint", "quantity-damping-power", "W"));
        out.push(SignalInfo::new(format!("{LOSS_PREFIX}other"), "loss-other", "quantity-power-given-off", "W"));
        // In the order of `losses`: the passive joints' dampers, then the process.
        if let Some((id, owner)) = self.passive_row {
            out.push(SignalInfo::new(format!("{LOSS_PREFIX}{id}"), owner, "quantity-damping-power", "W"));
        }
        if self.mechanism.has_process() {
            out.push(SignalInfo::new(
                format!("{LOSS_PREFIX}process"),
                "group-process",
                "quantity-power-given-off",
                "W",
            ));
        }
        if self.two_way {
            out.push(SignalInfo::new(
                format!("{LOSS_PREFIX}link_bending"),
                "group-link_bending",
                "quantity-damping-power",
                "W",
            ));
        }
        if let Some((id, owner)) = self.mechanism.loss_row() {
            out.push(SignalInfo::new(format!("{LOSS_PREFIX}{id}"), owner, "quantity-damping-power", "W"));
        }
        out.extend(ENERGY_PARTS.map(|part| {
            SignalInfo::new(format!("energy.{part}"), "group-energy", &format!("quantity-energy-{part}"), "J")
        }));
        for (k, contact) in self.friction_contacts().iter().enumerate() {
            let (speed, force) = self.units(k);
            for part in FRICTION_SIGNALS {
                let unit = match part {
                    "speed" => speed,
                    "load_scale" => "",
                    _ => force,
                };
                out.push(SignalInfo::new(
                    format!("{}.friction_{part}", contact.id),
                    contact.owner_key,
                    &format!("quantity-friction-{}", part.replace('_', "-")),
                    unit,
                ));
            }
        }
    }

    /// The lowest mode of a one-joint machine with an elastic transmission: the load (M) and the rotor seen at the joint
    /// (J i^2) on the transmission's spring k, f = sqrt(k (1/M + 1/(J i^2))) / (2 pi). `None` for more joints.
    pub fn two_mass_frequency(&self, q: &[f64]) -> Option<f64> {
        if self.n() != 1 {
            return None;
        }
        let (mass, _) = self.load(q, &[0.0]);
        let j = &self.joints[0].drive;
        Some(
            (j.gearbox.at(q[0]).stiffness * (1.0 / mass[0] + 1.0 / j.reflected_inertia())).sqrt()
                / (2.0 * std::f64::consts::PI),
        )
    }

    /// One joint's modes at q: the gearbox's twist x = q - r between the rotor (r, J i^2 at the joint) and the joint,
    /// and each passive joint on its own spring, coupled through the mass, with the joint's rigid motion (no spring)
    /// taken out: M over (x, q, p) has q's row, and q'' = -(M_qx x'' + M_qp p'') / M_qq leaves the Schur complement
    /// M_ss - M_sq M_qs / M_qq over s = (x, p); K, C the gearbox's and the passive joints' (`generalized_modes`).
    /// Without passive joints this is `two_mass_frequency`, damping c / (2 sqrt(k mu)), mu = M J i^2 / (M + J i^2).
    /// A passive joint with no spring of its own (a contact the mechanism's forces hold) is held, as `load` holds it.
    /// Without `damped` the frequencies alone (damping 0).
    fn single_joint_modes(&self, q: &[f64], damped: bool) -> Vec<sim_core::Mode> {
        let qa: Vec<f64> = q.iter().copied().chain(self.mechanism.passive_rest(q)).collect();
        let all = qa.len();
        let (mass, _) = self.load_all(&qa, &vec![0.0; all]);
        let drive = &self.joints[0].drive;
        let jr = drive.reflected_inertia();
        // The gear's spring at the joint's travel (a screw's stiffness follows its nut, `Gearbox::at`).
        let gear = drive.gearbox.at(q[0]);
        let sprung: Vec<usize> = (0..self.passive.len()).filter(|&p| self.passive[p].0 > 0.0).map(|p| p + 1).collect();
        if sprung.is_empty() {
            // The closed form, so that the frequency is the one `resonance_hz` always gave, to the bit; zeta = c w / 2k.
            let damping = |hz: f64| {
                let w = std::f64::consts::TAU * hz;
                gear.damping_at(w) * w / (2.0 * gear.stiffness)
            };
            return self
                .two_mass_frequency(q)
                .map(|hz| sim_core::Mode { hz, damping: damping(hz) })
                .into_iter()
                .collect();
        }
        let size = 1 + sprung.len();
        // The coordinates' rows of M over (x, q, p): x couples to q by -J i^2, a passive joint by M's own entries.
        let to_q: Vec<f64> = std::iter::once(-jr).chain(sprung.iter().map(|&p| mass[p * all])).collect();
        let m_qq = jr + mass[0];
        let mut m = vec![0.0; size * size];
        m[0] = jr;
        for (a, &pa) in sprung.iter().enumerate() {
            for (b, &pb) in sprung.iter().enumerate() {
                m[(a + 1) * size + b + 1] = mass[pa * all + pb];
            }
        }
        (0..size * size).for_each(|rc| m[rc] -= to_q[rc / size] * to_q[rc % size] / m_qq);
        let (k, c): (Vec<f64>, Vec<f64>) =
            std::iter::once((gear.stiffness, gear.damping)).chain(sprung.iter().map(|&p| self.passive[p - 1])).unzip();
        let c = |r: usize, w: f64| match r {
            _ if !damped => 0.0,
            0 => gear.damping_at(w),
            r => c[r],
        };
        generalized_modes(&m, &k, c, size)
    }
}

impl<M: Mechanism> Machine for JointMachine<M> {
    /// The mechanism's (`Mechanism::frame_momentum`, passive joints included), each rotor's spin from the state.
    fn frame_momentum(&self, s: &[f64]) -> Option<FrameMomentum> {
        let (q, qd) = self.all_joints(s);
        let spin: Vec<f64> =
            self.rotors(s).1.iter().zip(&self.joints).map(|(w, j)| j.drive.rotor_inertia * w).collect();
        self.mechanism.frame_momentum(&q, &qd, &spin)
    }
    fn axes(&self) -> &[AxisInfo] {
        &self.axes
    }
    fn motors(&self) -> &[MotorInfo] {
        &self.motors
    }
    fn motor_linear(&self, _motor: usize) -> bool {
        self.mechanism.linear_motor()
    }

    fn rating_checks(&self) -> Vec<RatingCheck> {
        self.mechanism.rating_checks()
    }
    fn cam(&self) -> Option<sim_core::CamMaster> {
        self.mechanism.cam()
    }
    fn cam_at(&self, master: f64, out: &mut [[f64; 4]]) {
        self.mechanism.cam_at(master, out);
    }
    fn cam_curve(&self) -> Option<sim_core::CamCurve> {
        self.mechanism.cam_curve()
    }

    /// NaN where the pose cannot be reached: the controller never asks for one (see `unreachable`).
    fn inverse(&self, tool: &[f64], motors: &mut [f64]) {
        let mut q = vec![0.0; self.n()];
        let ok = self.mechanism.inverse(tool, &mut q);
        for k in 0..self.n() {
            motors[k] = if ok { self.joints[k].drive.ratio() * q[k] } else { f64::NAN };
        }
    }
    fn forward(&self, motors: &[f64], tool: &mut [f64]) {
        let q: Vec<f64> = (0..self.n()).map(|k| motors[k] / self.joints[k].drive.ratio()).collect();
        self.mechanism.forward(&q, tool);
    }

    fn state_len(&self) -> usize {
        self.arm_at() + (0..self.n()).filter(|&k| self.arm_slot(k).is_some()).count()
    }

    /// With bending links, the tool (not the joints' pose) at `tool`: the links bent by their load at rest.
    fn equilibrium(&self, tool: &[f64], state: &mut Vec<f64>, torque: &mut [f64]) {
        let n = self.n();
        let (mut q, mut bent, mut pose) = (vec![0.0; n], vec![0.0; self.flex], tool.to_vec());
        for _ in 0..if self.flex > 0 { 3 } else { 1 } {
            if !self.mechanism.inverse(&pose, &mut q) {
                q.fill(f64::NAN);
            }
            if self.flex > 0 {
                let mut load = vec![0.0; self.flex];
                self.mechanism.flex_loads(&q, &vec![0.0; n], &vec![0.0; n], &mut load);
                let modes = self.mechanism.flex_modes();
                bent = load.iter().zip(&modes).map(|(l, m)| l / m.stiffness).collect();
                let shift = self.flex_shift(&q, &bent);
                pose = tool.iter().zip(&shift).map(|(t, s)| t - s).collect();
            }
        }
        // The passive joints where they rest (a contact pressed in by its load), the driven ones' weights with them.
        let rest = self.mechanism.passive_rest(&q);
        let gravity = if rest.iter().all(|x| *x == 0.0) {
            self.load(&q, &vec![0.0; n]).1
        } else {
            let all: Vec<f64> = q.iter().chain(&rest).copied().collect();
            self.load_all(&all, &vec![0.0; all.len()]).1[..n].to_vec()
        };
        // Two-way, the bent links shift the weights: G of `FlexCoupling` at rest.
        let mut shift = vec![0.0; n];
        if self.two_way {
            let all = n + self.passive.len();
            let mut qa = q.clone();
            qa.resize(all, 0.0);
            if let Some(c) = self.mechanism.flex_coupling(&qa, &vec![0.0; all], &bent, &vec![0.0; self.flex]) {
                shift.copy_from_slice(&c.joint[..n]);
            }
        }
        state.clear();
        state.extend(&q);
        state.extend(vec![0.0; n]);
        for k in 0..n {
            let j = &self.joints[k].drive;
            // At rest the transmission holds the load against gravity and the stops.
            let out = gravity[k] - shift[k] - j.output_torque(q[k], 0.0);
            let rotor = j.ratio() * (q[k] + if self.elastic { j.gearbox.static_twist_at(q[k], out) } else { 0.0 });
            if self.elastic {
                state.push(rotor);
            }
            let held = if self.elastic { j.gearbox.held_at(q[k], out) } else { out };
            torque[k] = j.ripple.invert(held / j.ratio(), rotor);
        }
        state.resize(self.passive_at(), 0.0);
        state.extend(rest);
        state.resize(self.flex_at(), 0.0);
        state.extend(bent);
        state.resize(self.state_len(), 0.0);
        // The arms at rest carry nothing.
        for k in 0..n {
            if let Some(i) = self.arm_slot(k) {
                state[i] = self.joints[k].drive.gearbox.arm_rest(state[2 * n + k], q[k]);
            }
        }
    }

    fn derivatives(&self, s: &[f64], torque: &[f64], out: &mut [f64]) {
        let n = self.n();
        let (q, qd) = self.arms(s);
        out[..n].copy_from_slice(qd);
        let (p, at) = (self.passive.len(), self.passive_at());
        out[at..at + p].copy_from_slice(&s[at + p..at + 2 * p]);
        if !self.elastic {
            let (acc, ..) = self.rigid_solve(s, torque);
            let (_, rotor_speed) = self.rotors(s);
            let brakes = self.brakes(s);
            out[n..2 * n].copy_from_slice(&acc[..n]);
            out[at + p..at + 2 * p].copy_from_slice(&acc[n..n + p]);
            for k in 0..n {
                out[2 * n + k] = self.joints[k].drive.brake.torque(self.brake_engaged, rotor_speed[k], brakes[k]).1;
            }
            return self.flex_rates(s, &acc, out);
        }
        // Without passive joints the driven ones are all, as they stand in the state.
        let joints;
        let (qa, qda) = if p == 0 {
            (q, qd)
        } else {
            joints = self.all_joints(s);
            (&joints.0[..], &joints.1[..])
        };
        // The load, the process and the accelerations on the stack for up to `ON_STACK` joints: this runs four times a
        // sub-step, and the heap's allocations were a tenth of a six-joint arm's step.
        let all = n + p;
        let (mut stack, mut heap) =
            (([0.0; ON_STACK * ON_STACK], [0.0; ON_STACK], [0.0; ON_STACK], [0.0; ON_STACK]), None);
        let (mass, bias, process, rhs): (&mut [f64], &mut [f64], &mut [f64], &mut [f64]) = if all <= ON_STACK {
            (&mut stack.0[..all * all], &mut stack.1[..all], &mut stack.2[..n], &mut stack.3[..all])
        } else {
            let (m, b, pr, r) = heap.insert((vec![0.0; all * all], vec![0.0; all], vec![0.0; n], vec![0.0; all]));
            (m, b, pr, r)
        };
        self.mechanism.dynamics(qa, qda, mass, bias);
        let springs = self.passive_torque(&qa[n..], &qda[n..]);
        self.mechanism.process_torque(q, qd, process);
        for i in 0..p {
            rhs[n + i] = springs[i] - bias[n + i];
        }
        for k in 0..n {
            let j = &self.joints[k].drive;
            let u = self.arm(s, k);
            let gear = j.gearbox.torque_with(s[2 * n + k], s[3 * n + k], q[k], qd[k], u);
            let (rotor_acc, brake_rate) =
                j.rotor_under(torque[k], s[2 * n + k], s[3 * n + k], gear, s[4 * n + k], self.brake_engaged);
            if let Some(i) = self.arm_slot(k) {
                out[i] = j.gearbox.arm(s[2 * n + k], q[k], u).1;
            }
            rhs[k] = gear.output - bias[k] + j.output_torque(q[k], qd[k]) + process[k];
            out[2 * n + k] = s[3 * n + k];
            out[3 * n + k] = rotor_acc;
            out[4 * n + k] = brake_rate;
        }
        // Without two-way bending the plain solve, in place; with it the links' modes beside the joints.
        let coupling = self.flex_coupling(s);
        let coupled;
        let acc: &[f64] = match coupling.as_ref() {
            None => {
                if !solve_in_place(mass, rhs) {
                    rhs.fill(f64::NAN);
                }
                rhs
            }
            Some(c) => {
                coupled = self.flex_solve(Some(c), s, mass, rhs);
                &coupled
            }
        };
        out[n..2 * n].copy_from_slice(&acc[..n]);
        out[at + p..at + 2 * p].copy_from_slice(&acc[n..n + p]);
        self.flex_rates(s, acc, out);
    }

    fn tool_state(&self, s: &[f64], pos: &mut [f64], vel: &mut [f64]) {
        let (q, qd) = self.arms(s);
        self.mechanism.forward(q, pos);
        let jac = self.jacobian(q);
        let n = self.n();
        for (i, v) in vel.iter_mut().enumerate().take(self.axes.len()) {
            *v = (0..n).map(|k| jac[i * n + k] * qd[k]).sum();
        }
        if self.flex > 0 {
            let (d, rate) = self.flex_state(s);
            pos.iter_mut().zip(self.flex_shift(q, d)).for_each(|(p, d)| *p += d);
            vel.iter_mut().zip(self.flex_shift(q, rate)).for_each(|(v, d)| *v += d);
        }
    }
    fn set_tool_velocity(&self, s: &mut [f64], vel: &[f64]) {
        let n = self.n();
        let mut pos = vec![0.0; self.axes.len()];
        self.mechanism.forward(&s[..n], &mut pos);
        let zero = vec![0.0; pos.len()];
        let (_, qd, _) = self.joint_motion(&pos, vel, &zero);
        s[n..2 * n].copy_from_slice(&qd);
        if self.elastic {
            for k in 0..n {
                s[3 * n + k] = self.joints[k].drive.ratio() * qd[k];
            }
        }
    }
    /// `rotors`, written in place (called at every stage of the integration).
    fn motor_state(&self, s: &[f64], pos: &mut [f64], vel: &mut [f64]) {
        let n = self.n();
        if self.elastic {
            pos[..n].copy_from_slice(&s[2 * n..3 * n]);
            vel[..n].copy_from_slice(&s[3 * n..4 * n]);
        } else {
            for k in 0..n {
                (pos[k], vel[k]) = (s[k] * self.joints[k].drive.ratio(), s[n + k] * self.joints[k].drive.ratio());
            }
        }
    }

    /// Rigid, loss-free, at rest: (M q'' + g) at the joint over the ratio, plus the rotor's own J i q'', with q'' what
    /// the tool's acceleration asks at rest.
    fn feedforward_torque(&self, pos: &[f64], acc: &[f64], out: &mut [f64]) {
        let zero = vec![0.0; pos.len()];
        let (q, qd, qdd) = self.joint_motion(pos, &zero, acc);
        let motor = |x: &[f64]| -> Vec<f64> { (0..self.n()).map(|k| x[k] * self.joints[k].drive.ratio()).collect() };
        self.rigid_motor_torque(&motor(&q), &motor(&qd), &motor(&qdd), out);
    }

    /// The rigid model at q'' = 0: per joint, at its speed q' (`joint_motion`), the motor shaft's friction (with the
    /// gearbox's drag) at i q' and the joint's own friction (guides, bearings, a screw nut's preload) through the gearbox
    /// as `rigid_solve` takes it: f(q') / (i loss), the loss factor that of what the gearbox passes at rest, f plus the
    /// weight it holds (driven back where the weight pulls the joint along). The brake is released and the stops are
    /// not touched in a move.
    fn steady_friction(&self, pos: &[f64], vel: &[f64], out: &mut [f64]) {
        let (_, qd, _) = self.joint_motion(pos, vel, &vec![0.0; pos.len()]);
        let mut hold = vec![0.0; out.len()];
        self.feedforward_torque(pos, &vec![0.0; pos.len()], &mut hold);
        for (k, joint) in self.joints.iter().enumerate() {
            let (j, v) = (&joint.drive, qd[k]);
            let friction = j.friction.torque(v);
            let loss = j.gearbox.loss_factor(friction + j.ratio() * hold[k], v);
            out[k] = j.shaft.torque(j.ratio() * v) + friction / (j.ratio() * loss);
        }
    }

    fn rigid_guide_loads(&self, pos: &[f64], acc: &[f64], out: &mut Vec<[f64; 2]>) {
        self.mechanism.rigid_guide_loads(pos, acc, out);
    }

    /// The friction in each joint (`RotaryJoint::friction`, its Coulomb level and viscous part, at the joint) as torque
    /// at its motor while the tool moves along `dir`: through the ratio and what a screw or rack passes on
    /// (`extra_efficiency`), the joint's speed per speed along the path from the kinematics.
    fn rigid_friction(&self, pos: &[f64], dir: &[f64], out: &mut Vec<[f64; 2]>) {
        let (_, qd, _) = self.joint_motion(pos, dir, &vec![0.0; pos.len()]);
        for (joint, rate) in self.joints.iter().zip(qd) {
            let (f, lever) = (&joint.drive.friction, 1.0 / (joint.drive.ratio() * joint.drive.extra_efficiency));
            let sign = if rate == 0.0 { 0.0 } else { rate.signum() };
            out.push([f.coulomb * sign * lever, f.viscous * rate * lever]);
        }
    }

    /// The mechanism's signals (`Mechanism::signals`) and each joint's force (`joint.<id>.force`) at the joints' motion,
    /// with the force M q'' + h that the rigid model passes to each joint (the process force left out, as the
    /// feedforward leaves it out), read for each check of `rating_checks`; NaN for a life or a signal not among them.
    /// A check on `plan.<signal>` reads <signal>: a limit of the plan alone, which the verdicts judge on the setpoint's
    /// rigid motion through this (`analysis::Tally`), not on the simulated signal.
    fn rigid_checks(&self, pos: &[f64], vel: &[f64], acc: &[f64], out: &mut Vec<f64>) {
        let n = self.n();
        let (q, qd, qdd) = self.joint_motion(pos, vel, acc);
        let (mass, bias) = self.load(&q, &qd);
        let force: Vec<f64> = (0..n).map(|k| (0..n).map(|c| mass[k * n + c] * qdd[c]).sum::<f64>() + bias[k]).collect();
        let mut values = Vec::new();
        self.mechanism.signals(&q, &qd, &qdd, &force, &mut values);
        let infos = self.mechanism.signal_infos();
        for check in self.mechanism.rating_checks() {
            let signal = check.signal.strip_prefix("plan.").unwrap_or(&check.signal);
            let joint = (0..n).find(|&k| signal == format!("joint.{}.force", self.joints[k].id));
            let value = match (check.life, joint) {
                (true, _) => f64::NAN,
                (false, Some(k)) => force[k],
                (false, None) => infos.iter().position(|s| s.id == signal).map_or(f64::NAN, |i| values[i]),
            };
            out.push(value);
        }
    }

    /// The joints' motion (`joint_motion`), then `rigid_motor_torque` with the velocity terms: the Coriolis and
    /// centripetal torques of M(q) changing with the pose, in joint space too.
    fn feedforward_torque_moving(&self, pos: &[f64], vel: &[f64], acc: &[f64], out: &mut [f64]) {
        let (q, qd, qdd) = self.joint_motion(pos, vel, acc);
        let motor = |x: &[f64]| -> Vec<f64> { (0..self.n()).map(|k| x[k] * self.joints[k].drive.ratio()).collect() };
        self.rigid_motor_torque(&motor(&q), &motor(&qd), &motor(&qdd), out);
    }

    /// Exact in joint coordinates, velocity terms included (`feedforward_torque` leaves them out): M q'' + h at the
    /// joint over the ratio, plus the rotor's own J_rotor i q''.
    fn rigid_motor_torque(&self, angle: &[f64], speed: &[f64], acc: &[f64], out: &mut [f64]) {
        self.rigid_split_torque(angle, speed, acc, acc, out);
    }

    /// Elastic: the gearbox's static twist under what reaches it, (torque − J_rotor acc) i at the joint (ripple, motor
    /// friction and the gearbox's losses and damping left out), times the ratio. Zeros where the joints are rigid.
    /// Behind a screw whose stiffness follows the nut, the far end's: `twist_at` has the nut's place.
    fn twist(&self, torque: &[f64], acc: &[f64], out: &mut [f64]) {
        for (k, joint) in self.joints.iter().enumerate() {
            let j = &joint.drive;
            out[k] = if self.elastic {
                j.ratio() * j.gearbox.static_twist((torque[k] - j.rotor_inertia * acc[k]) * j.ratio())
            } else {
                0.0
            };
        }
    }

    /// `twist` with the gear's spring where the nut is, the rotor's angle over the ratio (`Gearbox::at`).
    fn twist_at(&self, angle: &[f64], torque: &[f64], acc: &[f64], out: &mut [f64]) {
        for (k, joint) in self.joints.iter().enumerate() {
            let j = &joint.drive;
            let gear = j.gearbox.at(angle[k] / j.ratio());
            out[k] = if self.elastic {
                j.ratio() * gear.static_twist((torque[k] - j.rotor_inertia * acc[k]) * j.ratio())
            } else {
                0.0
            };
        }
    }

    fn rigid_split_torque(&self, angle: &[f64], speed: &[f64], acc: &[f64], rotor_acc: &[f64], out: &mut [f64]) {
        let n = self.n();
        let joint = |x: &[f64]| -> Vec<f64> { (0..n).map(|k| x[k] / self.joints[k].drive.ratio()).collect() };
        let (q, qd, qdd) = (joint(angle), joint(speed), joint(acc));
        let (mass, bias) = self.load(&q, &qd);
        for k in 0..n {
            let j = &self.joints[k].drive;
            let at_joint: f64 = (0..n).map(|c| mass[k * n + c] * qdd[c]).sum::<f64>() + bias[k];
            out[k] = at_joint / j.ratio() + j.rotor_inertia * rotor_acc[k];
        }
    }

    fn energy(&self, s: &[f64]) -> f64 {
        self.energy_parts(s).iter().sum()
    }

    fn set_payload_attached(&mut self, attached: bool) {
        if attached != self.payload_attached {
            self.rest_slack();
        }
        self.payload_attached = attached;
        self.mechanism.set_payload_attached(attached);
    }
    fn links(&self) -> Vec<String> {
        self.mechanism.links()
    }
    fn set_link_change(&mut self, link: usize, factor: f64, moment: [f64; 3]) {
        self.rest_slack();
        self.mechanism.set_link_change(link, factor, moment);
    }
    fn set_brake_engaged(&mut self, engaged: bool) {
        self.brake_engaged = engaged;
    }

    /// The gearboxes, in the order of `thermal_parts`.
    /// The gearboxes' hysteresis states move on to where the step ended (`Gearbox::commit`), and so do the joints'
    /// presliding friction states (`ShaftFriction::commit`).
    fn after_step(&mut self, s: &mut [f64], _dt: f64) {
        let n = self.n();
        if self.elastic {
            for k in 0..n {
                self.joints[k].drive.gearbox.commit(s[2 * n + k], s[k]);
            }
        }
        // The joints' positions and speeds lead the state in both models (`arms`).
        for k in 0..n {
            self.joints[k].drive.friction.commit(s[k], s[n + k]);
        }
    }
    /// The mechanism's, then the gearboxes' hysteresis anchors, then those of the joints whose friction has a presliding
    /// state.
    fn memory(&self) -> Vec<f64> {
        let gears = crate::mech::Gearbox::anchors(self.joints.iter().map(|j| &j.drive.gearbox));
        let frictions =
            self.joints.iter().filter(|j| j.drive.friction.presliding > 0.0).flat_map(|j| j.drive.friction.anchor);
        [self.mechanism.memory(), gears, frictions.collect()].concat()
    }
    fn set_memory(&mut self, memory: &[f64]) {
        let (own, rest) = memory.split_at(self.mechanism.memory().len().min(memory.len()));
        self.mechanism.set_memory(own);
        // Two numbers per gear, three with its mesh friction's anchor (`Gearbox::anchors`).
        let geared = crate::mech::Gearbox::anchors(self.joints.iter().map(|j| &j.drive.gearbox)).len();
        let (gears, frictions) = rest.split_at(geared.min(rest.len()));
        crate::mech::Gearbox::set_anchors(self.joints.iter_mut().map(|j| &mut j.drive.gearbox), gears);
        let sliding = self.joints.iter_mut().filter(|j| j.drive.friction.presliding > 0.0);
        for (joint, anchor) in sliding.zip(frictions.as_chunks::<2>().0) {
            joint.drive.friction.anchor = *anchor;
        }
    }

    fn set_part_temperatures(&mut self, rises: &[f64]) {
        let Some(law) = &self.thermal else { return };
        let geared = self.joints.iter_mut().enumerate().filter(|(_, j)| !j.drive.coupled);
        for ((k, joint), rise) in geared.zip(rises) {
            // From the joint's own gearbox's efficiency where it has one (a track's planetary gearbox beside an arm's
            // strain wave gears), else the shared one's.
            let warmed = match self.gear_efficiencies.get(k) {
                Some(&efficiency) => ThermalFeedback { efficiency, ..law.clone() }.gearbox_efficiency(*rise),
                None => law.gearbox_efficiency(*rise),
            };
            joint.drive.gearbox.efficiency = warmed * joint.drive.extra_efficiency;
            joint.drive.shaft.set_oil(law.viscosity_factor(*rise));
        }
    }

    /// Elastic: per joint its gearbox's spring (K1, in series with a converter's own compliance behind it) between the
    /// rotor and the joint's rigid inertia M_kk with the other joints held, referred to the motor over the ratio.
    fn trains(&self, tool_pose: &[f64]) -> Vec<Option<sim_core::Train>> {
        let mut q = vec![0.0; self.n()];
        if !self.elastic || !self.mechanism.inverse(tool_pose, &mut q) {
            return Vec::new();
        }
        let (mass, _) = self.load(&q, &vec![0.0; self.n()]);
        (0..self.n())
            .map(|k| {
                let j = &self.joints[k].drive;
                j.train_at(q[k], mass[k * self.n() + k] / (j.ratio() * j.ratio()) + j.rotor_inertia)
            })
            .collect()
    }

    /// One joint: the lowest of its two-mass mode and its passive joints' (`single_joint_modes`). An arm: the lowest of
    /// its joints' modes with the motors held (`held_modes`, or with the links' bending two-way `coupled_modes`): each
    /// turns the joints and so moves the tool.
    fn resonance_hz(&self, tool_pose: &[f64]) -> Option<f64> {
        let mut q = vec![0.0; self.n()];
        if !self.mechanism.inverse(tool_pose, &mut q) {
            return None;
        }
        let modes = if self.n() == 1 {
            self.single_joint_modes(&q, false)
        } else if self.two_way {
            self.coupled_modes(&q, false)
        } else {
            self.held_modes(&q, false)
        };
        modes.iter().map(|m| m.hz).reduce(f64::min)
    }

    /// One joint's two-mass mode and its passive joints' (`single_joint_modes`), or an arm's joints with the motors held
    /// (`held_modes`), and the links' bending modes (`Mechanism::flex_modes`), lowest first; two-way, the modes of joints
    /// and links together with the motors held (`coupled_modes`).
    fn modes(&self, tool_pose: &[f64]) -> Vec<sim_core::Mode> {
        let mut q = vec![0.0; self.n()];
        let mut out: Vec<sim_core::Mode> = if self.n() == 1 {
            if self.mechanism.inverse(tool_pose, &mut q) { self.single_joint_modes(&q, true) } else { Vec::new() }
        } else if !self.two_way && self.mechanism.inverse(tool_pose, &mut q) {
            self.held_modes(&q, true)
        } else {
            Vec::new()
        };
        if self.two_way {
            let mut q = vec![0.0; self.n()];
            if self.mechanism.inverse(tool_pose, &mut q) {
                out.extend(self.coupled_modes(&q, true));
            }
        } else {
            out.extend(self.mechanism.flex_modes().iter().map(|m| {
                let w = (m.stiffness / m.mass).sqrt();
                sim_core::Mode { hz: w / std::f64::consts::TAU, damping: m.damping / (2.0 * m.mass * w) }
            }));
        }
        out.sort_by(|a, b| a.hz.total_cmp(&b.hz));
        out
    }

    fn setpoint_slack(&self) -> f64 {
        self.rest_slack()
    }

    fn condition(&self, tool: &[f64]) -> Option<f64> {
        self.mechanism.condition(tool)
    }

    fn unreachable(&self, tool: &[f64], slack: f64) -> Option<&'static str> {
        if let Some(why) = self.mechanism.unreachable(tool, slack) {
            return Some(why);
        }
        let mut q = vec![0.0; self.n()];
        if !self.mechanism.inverse(tool, &mut q) {
            return Some("error-target-out-of-range");
        }
        // A joint beyond its stops, by more than `slack` (taken in the joint's unit: where the tool is the joints, as on
        // the arms and slides, that is the controller's slack; an arm's gears twist under its weight beyond a stop).
        let beyond = self
            .joints
            .iter()
            .zip(&q)
            .any(|(j, q)| *q < j.drive.range.0 - slack - 1e-9 || *q > j.drive.range.1 + slack + 1e-9);
        beyond.then_some("error-joint-beyond-stop")
    }

    fn signal_infos(&self) -> Vec<SignalInfo> {
        let mut out = Vec::new();
        for (k, motor) in self.motors.iter().enumerate() {
            let unit = self.joints[k].unit;
            out.push(SignalInfo::new(
                format!("joint.{}.position", self.joints[k].id),
                self.joints[k].key,
                "quantity-joint-position",
                unit,
            ));
            out.push(SignalInfo::new(
                format!("joint.{}.force", self.joints[k].id),
                self.joints[k].key,
                "quantity-joint-force",
                self.units(k).1,
            ));
            if self.joints[k].gear_lever.is_some() || self.joints[k].unit != "m" {
                out.push(SignalInfo::new(
                    format!("gearbox.{}.output_torque", motor.id),
                    motor.key,
                    "quantity-gearbox-torque",
                    "N m",
                ));
            }
        }
        out.extend(self.mechanism.passive_signals());
        out.extend(self.mechanism.signal_infos());
        out.extend(self.mechanism.flex_modes().iter().map(|m| {
            let (id, quantity, unit) = m.kind.signal();
            SignalInfo::new(format!("link.{}.{id}", m.link + 1), JOINT_KEYS[m.link], quantity, unit)
        }));
        self.effect_infos(&mut out);
        out
    }

    fn base_signals(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        let (q, qd) = self.arms(s);
        let (acc, force) = self.motion(s, torque);
        for k in 0..self.n() {
            out.extend([q[k], force[k]]);
            match self.joints[k].gear_lever {
                // Ahead of the converter: more than the joint's force times the lever while it drives, less while the
                // load drives it back.
                Some(lever) => {
                    let e = self.joints[k].drive.extra_efficiency;
                    out.push(force[k] * lever * if force[k] * qd[k] >= 0.0 { 1.0 / e } else { e });
                }
                None if self.joints[k].unit != "m" => out.push(force[k]),
                None => {}
            }
        }
        let at = self.passive_at();
        out.extend(&s[at..at + self.passive.len()]);
        let jerk = if self.mechanism.reads_jerk() { self.joint_jerk(s, torque) } else { vec![0.0; acc.len()] };
        self.mechanism.signals_with_jerk(q, qd, &acc, &jerk, &force, out);
        out.extend_from_slice(self.flex_state(s).0);
    }
    fn effect_signals(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        self.view_signals(s, torque, out);
    }
    fn loss_powers(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        out.extend(self.losses(s, torque).powers);
    }

    /// Each joint's friction (`RotaryJoint::friction_split`: with a slide's presliding also the rise to its breakaway
    /// level), owned by the joint's axis: the friction view spans that
    /// axis's speed limit (with the joint's own key it found no axis and spanned 1 m/s or rad/s).
    fn friction_contacts(&self) -> Vec<FrictionContact> {
        self.joints
            .iter()
            .zip(&self.axes)
            .map(|(j, axis)| FrictionContact { id: j.id_signal(), owner_key: axis.key })
            .collect()
    }
    fn friction_rotary(&self, i: usize) -> bool {
        i < self.joints.len() && self.units(i).0 == "rad/s"
    }
    fn friction_curve(&self, i: usize, speed: f64, _load_scale: f64) -> [f64; 3] {
        self.joints.get(i).map_or([0.0; 3], |joint| joint.drive.friction_split(speed))
    }

    /// The gearboxes, where there are gearboxes (not on a direct drive).
    fn thermal_parts(&self) -> Vec<ThermalPart> {
        self.joints
            .iter()
            .zip(&self.motors)
            .filter(|(j, _)| !j.drive.coupled)
            .enumerate()
            .map(|(k, (_, m))| ThermalPart {
                id: GEARBOX_IDS[MOTOR_IDS.iter().position(|id| *id == m.id).unwrap_or(0)],
                kind: "gearbox",
                owner_key: m.key,
                scene_part: "gearbox",
                scene_index: k,
            })
            .collect()
    }

    fn scene(&self, s: &[f64], out: &mut Scene) {
        let (q, _) = self.all_joints(s);
        let (rotor, speed) = self.rotors(s);
        out.parts.clear();
        out.labels.clear();
        self.mechanism.scene(&q, &rotor, &speed, out);
    }
    fn scene_point(&self, tool: &[f64]) -> Option<[f64; 3]> {
        self.mechanism.scene_point(tool)
    }

    /// The boxes of the scene that the mechanism names (`Mechanism::collision_parts`).
    fn collision_bodies(&self, s: &[f64]) -> Vec<sim_core::Aabb> {
        let mut scene = Scene::default();
        self.scene(s, &mut scene);
        sim_core::scene::scene_boxes(&scene.parts, self.mechanism.collision_parts())
    }
}

impl<M: Mechanism> JointMachine<M> {
    /// The joints' positions, speeds and accelerations for the tool's pose, speed and acceleration. Where the tool is
    /// the joints, those themselves. With as many axes as joints: q' = J^-1 v and q'' = J^-1 (a - J' q'), J' q' the
    /// second difference of the forward kinematics along q'. With fewer (a redundant arm): by central differences of
    /// the mechanism's `inverse`, which resolves the redundancy, along v and along a, so that speeds and accelerations
    /// are those of the very joints it gives: q' = (q(x + h v) - q(x - h v)) / 2h, q'' = (q(x + h v) - 2 q(x) +
    /// q(x - h v)) / h^2 + (q(x + k a) - q(x - k a)) / 2k, steps of 1e-3 of the tool's units (the inverse's solution is good
    /// to about 1e-11, which a second difference over 1e-4 would multiply by 1e8).
    pub fn joint_motion(&self, pos: &[f64], vel: &[f64], acc: &[f64]) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
        let (n, m) = (self.n(), self.axes.len());
        if self.mechanism.tool_is_joints() {
            return (pos[..n].to_vec(), vel[..n].to_vec(), acc[..n].to_vec());
        }
        let solved = |x: &[f64]| -> Vec<f64> {
            let mut q = vec![0.0; n];
            if !self.mechanism.inverse(x, &mut q) {
                q.fill(f64::NAN);
            }
            q
        };
        let q = solved(pos);
        if m < n {
            let along = |d: &[f64]| -> Option<(f64, Vec<f64>, Vec<f64>)> {
                let size = d.iter().map(|v| v * v).sum::<f64>().sqrt();
                (size > 0.0).then(|| {
                    let h = 1e-3 / size;
                    let at = |s: f64| solved(&(0..m).map(|i| pos[i] + s * d[i]).collect::<Vec<_>>());
                    (h, at(h), at(-h))
                })
            };
            let (mut qd, mut qdd) = (vec![0.0; n], vec![0.0; n]);
            if let Some((h, ahead, behind)) = along(&vel[..m]) {
                for k in 0..n {
                    qd[k] = (ahead[k] - behind[k]) / (2.0 * h);
                    qdd[k] = (ahead[k] - 2.0 * q[k] + behind[k]) / (h * h);
                }
            }
            if let Some((h, ahead, behind)) = along(&acc[..m]) {
                for k in 0..n {
                    qdd[k] += (ahead[k] - behind[k]) / (2.0 * h);
                }
            }
            return (q, qd, qdd);
        }
        let jac = self.jacobian(&q);
        let qd = solve(&jac, &vel[..n]).unwrap_or_else(|| vec![f64::NAN; n]);
        let speed = qd.iter().map(|v| v * v).sum::<f64>().sqrt();
        let mut curve = vec![0.0; n];
        if speed > 0.0 {
            // A step of 1e-4 rad (m) along q': truncation ~1e-8, rounding ~1e-16 / 1e-8 of the tool's size.
            let h = 1e-4 / speed;
            let (mut a, mut b, mut c) = (vec![0.0; n], vec![0.0; n], vec![0.0; n]);
            let at = |s: f64| -> Vec<f64> { (0..n).map(|k| q[k] + s * qd[k]).collect() };
            self.mechanism.forward(&at(h), &mut a);
            self.mechanism.forward(&q, &mut b);
            self.mechanism.forward(&at(-h), &mut c);
            curve = (0..n).map(|k| (a[k] - 2.0 * b[k] + c[k]) / (h * h)).collect();
        }
        let rhs: Vec<f64> = (0..n).map(|k| acc[k] - curve[k]).collect();
        let qdd = solve(&jac, &rhs).unwrap_or_else(|| vec![f64::NAN; n]);
        (q, qd, qdd)
    }

    /// d tool / d q (axes x joints, row by row), by central differences of `Mechanism::forward`; exact (to rounding)
    /// where the tool is the joints.
    pub fn jacobian(&self, q: &[f64]) -> Vec<f64> {
        let (n, rows) = (self.n(), self.axes.len());
        // Where the tool is the joints, the identity to the bit: the difference quotient of the copy is off by up to
        // 1e-10, and a model switch that rebuilt the joints changed the tool's speed by that.
        if self.mechanism.tool_is_joints() {
            return (0..rows * n).map(|i| if i / n == i % n { 1.0 } else { 0.0 }).collect();
        }
        let (mut jac, mut a, mut b) = (vec![0.0; rows * n], vec![0.0; rows], vec![0.0; rows]);
        let mut p = q.to_vec();
        for c in 0..n {
            let h = 1e-6 * q[c].abs().max(1.0);
            p[c] = q[c] + h;
            self.mechanism.forward(&p, &mut a);
            p[c] = q[c] - h;
            self.mechanism.forward(&p, &mut b);
            p[c] = q[c];
            for r in 0..rows {
                jac[r * n + c] = (a[r] - b[r]) / (2.0 * h);
            }
        }
        jac
    }
}

/// Motor ids "1" to "7" and the ids of their gearboxes' loss powers, so that `ThermalPart` gets `&'static str`s.
pub const MOTOR_IDS: [&str; 7] = ["1", "2", "3", "4", "5", "6", "7"];
const GEARBOX_IDS: [&str; 7] =
    ["gearbox_1", "gearbox_2", "gearbox_3", "gearbox_4", "gearbox_5", "gearbox_6", "gearbox_7"];
/// Translation keys of motors 1 to 7 and of joints 1 to 7.
pub const MOTOR_KEYS: [&str; 7] = [
    "motor-joint-1",
    "motor-joint-2",
    "motor-joint-3",
    "motor-joint-4",
    "motor-joint-5",
    "motor-joint-6",
    "motor-joint-7",
];
pub const JOINT_KEYS: [&str; 7] = ["joint-1", "joint-2", "joint-3", "joint-4", "joint-5", "joint-6", "joint-7"];
const JOINT_SIGNALS: [&str; 7] = ["joint.1", "joint.2", "joint.3", "joint.4", "joint.5", "joint.6", "joint.7"];

impl JointSpec {
    /// "joint.<id>" for the friction contact, where the id is one of `MOTOR_IDS`.
    fn id_signal(&self) -> &'static str {
        JOINT_SIGNALS[MOTOR_IDS.iter().position(|id| *id == self.id).unwrap_or(0)]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Two joints on springs of 100 and 50 N m/rad with the mass matrix [[2, 0.5], [0.5, 1]] kg m^2: det(K - l M) =
    /// (100 - 2 l)(50 - l) - (0.5 l)^2 = 1.75 l^2 - 200 l + 5000 = 0, l = (200 -+ sqrt(5000)) / 3.5 = 36.939805 and
    /// 77.345910 (rad/s)^2, 0.967314 and 1.399712 Hz. Uncoupled (no 0.5), each joint alone: sqrt(k / J) / 2 pi, 1.125395
    /// Hz for both. A damper of 2 N m s/rad on the first joint alone damps the uncoupled first mode by c / (2 sqrt(k J))
    /// = 2 / (2 sqrt(200)) = 0.070711.
    #[test]
    fn held_joint_modes_are_the_hand_solved_eigenvalues() {
        let tau = std::f64::consts::TAU;
        let coupled = generalized_modes(&[2.0, 0.5, 0.5, 1.0], &[100.0, 50.0], |_, _| 0.0, 2);
        let mut hz: Vec<f64> = coupled.iter().map(|m| m.hz).collect();
        hz.sort_by(f64::total_cmp);
        let by_hand = [(200.0 - 5000f64.sqrt()) / 3.5, (200.0 + 5000f64.sqrt()) / 3.5].map(|l: f64| l.sqrt() / tau);
        assert!((hz[0] - 0.967314).abs() < 1e-6 && (hz[1] - 1.399712).abs() < 1e-6, "{hz:?}");
        assert!((hz[0] - by_hand[0]).abs() < 1e-12 && (hz[1] - by_hand[1]).abs() < 1e-12);
        let alone = generalized_modes(&[2.0, 0.0, 0.0, 1.0], &[100.0, 50.0], |r, _| [2.0, 0.0][r], 2);
        for m in &alone {
            assert!((m.hz - 1.125395).abs() < 1e-6, "{alone:?}");
        }
        assert!(alone.iter().any(|m| (m.damping - 0.070711).abs() < 1e-6), "{alone:?}");
    }

    /// The two joints above with a damper of 6 N m s/rad on the first and 0.5 on the second, C not proportional to M and
    /// K: each mode's damping ratio is that of its root of det(M l^2 + C l + K) = (2 l^2 + 6 l + 100)(l^2 + 0.5 l + 50) -
    /// (0.5 l^2)^2, found here by Newton's method from the undamped frequency (not through the companion matrix the code
    /// takes), -Re l / |l|; the frequencies stay the undamped ones to the bit. The two roots' -Re l add up to half the
    /// trace of M^-1 C, (1 x 6 + 2 x 0.5) / 1.75 / 2 = 2.0 (M^-1 = [[1, -0.5], [-0.5, 2]] / 1.75).
    #[test]
    fn coupled_modes_are_damped_as_the_damped_problems_roots() {
        type Cx = (f64, f64);
        let mul = |a: Cx, b: Cx| (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0);
        let add = |a: Cx, b: Cx| (a.0 + b.0, a.1 + b.1);
        let poly = |l: Cx, d: [f64; 3]| add(add(mul((d[0], 0.0), mul(l, l)), mul((d[1], 0.0), l)), (d[2], 0.0));
        let det = |l: Cx| {
            let off = mul((0.5, 0.0), mul(l, l));
            let a = mul(poly(l, [2.0, 6.0, 100.0]), poly(l, [1.0, 0.5, 50.0]));
            add(a, mul((-1.0, 0.0), mul(off, off)))
        };
        let root = |mut l: Cx| {
            for _ in 0..100 {
                let h = 1e-7 * l.0.hypot(l.1);
                let (f, g) = (det(l), add(det((l.0 + h, l.1)), mul((-1.0, 0.0), det(l))));
                let slope = (g.0 / h, g.1 / h);
                let n = slope.0 * slope.0 + slope.1 * slope.1;
                let step = ((f.0 * slope.0 + f.1 * slope.1) / n, (f.1 * slope.0 - f.0 * slope.1) / n);
                l = (l.0 - step.0, l.1 - step.1);
            }
            l
        };
        let (m, k, c) = ([2.0, 0.5, 0.5, 1.0], [100.0, 50.0], [6.0, 0.5]);
        let modes = generalized_modes(&m, &k, |r, _| c[r], 2);
        let undamped = generalized_modes(&m, &k, |_, _| 0.0, 2);
        let mut real_sum = 0.0;
        for (mode, plain) in modes.iter().zip(&undamped) {
            assert_eq!(mode.hz, plain.hz);
            let w = std::f64::consts::TAU * mode.hz;
            let l = root((-0.1 * w, w));
            assert!(det(l).0.hypot(det(l).1) < 1e-9, "{l:?}");
            let by_hand = -l.0 / l.0.hypot(l.1);
            assert!((mode.damping - by_hand).abs() < 1e-9, "{mode:?} against {by_hand}");
            real_sum -= l.0;
        }
        assert!((real_sum - 2.0).abs() < 1e-9, "{real_sum}");
    }

    #[test]
    fn gaussian_elimination_solves_and_refuses_singular() {
        let x = solve(&[0.0, 2.0, 1.0, 1.0], &[4.0, 3.0]).unwrap();
        assert!((x[0] - 1.0).abs() < 1e-12 && (x[1] - 2.0).abs() < 1e-12);
        assert!(solve(&[1.0, 2.0, 2.0, 4.0], &[1.0, 2.0]).is_none());
    }
}
