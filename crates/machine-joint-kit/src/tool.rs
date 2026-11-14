//! Tool-space kinematics of the arms (`kinematics.tool_space = true`): the machine's axes are the tool's pose, not the
//! joints, so every planner, the cycle, the CLI and the app move the tool along straight lines and arcs, as on the
//! deltas.
//!
//! - SCARA (four joints: two arms about vertical axes, a vertical quill, a turning flange): x, y, z (m) of the flange
//!   and c (rad), its turn about the vertical, in closed form with the elbow as `kinematics.elbow` says.
//! - Any other chain of 4 to 7 joints (`Pose`): those of x, y, z (m) and rx, ry, rz (rad) that `kinematics.tool_axes`
//!   names, as many as joints or one fewer for a redundant arm. The orientation is the rotation vector (axis times
//!   angle, base frame) of the turn from the flange's orientation at zero joints, so that a flange pointing down at zero
//!   is at 0 there, not at half a turn. Numerically by damped least squares on the URDF chain, with the redundant
//!   joint's freedom spent on staying near `kinematics.home_joints` (the null space of the Jacobian). The solution is
//!   followed along the line from the last one (or from the home) to the target, so it stays on the branch (shoulder,
//!   elbow, wrist) it is on.
//!
//! The planners move along a line with its orientation by slerp (`path`): turned about one fixed axis at a steady rate.

use sim_components::chain::solve;
use sim_components::rigid::{Chain, M3, V3, dot, log, matmul, transpose};
use std::f64::consts::PI;
use std::sync::Mutex;

/// Which kinematics maps the tool pose to the joints.
pub enum Tool {
    /// The tool pose is the joints (rad, m).
    Joints,
    Scara(Scara),
    Pose(Pose),
}

/// The ids of a pose's six components, their translation keys and units, in `kinematics.tool_axes`' order.
pub const POSE_AXES: [(&str, &str, &str); 6] = [
    ("x", "axis-x", "m"),
    ("y", "axis-y", "m"),
    ("z", "axis-z", "m"),
    ("rx", "axis-rx", "rad"),
    ("ry", "axis-ry", "rad"),
    ("rz", "axis-rz", "rad"),
];

/// The flange's orientation and position at the joints `q`.
pub type Forward<'a> = &'a dyn Fn(&[f64]) -> (M3, V3);

/// The SCARA's geometry, read from its chain at zero.
pub struct Scara {
    /// Joint 1's axis (x, y).
    base: [f64; 2],
    /// Lengths and directions at zero of arm 1 (joint 1 to joint 2) and arm 2 (joint 2 to the flange), seen from above.
    l1: f64,
    l2: f64,
    phi1: f64,
    phi2: f64,
    /// The flange's height at zero and per metre of joint 3; its turn at zero.
    z0: f64,
    dz: f64,
    c0: f64,
    /// +1 for the right-handed elbow (joint 2 counter-clockwise from stretched), -1 for the left.
    elbow: f64,
}

/// Rows x cols matrix (row by row) J: the 2-norm condition number, sqrt of the largest over the smallest eigenvalue of
/// J^T J by Jacobi rotations; infinite where it is singular.
pub fn condition_number(j: &[f64], rows: usize, cols: usize) -> f64 {
    let mut a: Vec<f64> =
        (0..cols * cols).map(|rc| (0..rows).map(|k| j[k * cols + rc / cols] * j[k * cols + rc % cols]).sum()).collect();
    let n = cols;
    for _ in 0..100 {
        let off: f64 = (0..n)
            .flat_map(|r| (0..n).filter(move |c| *c != r).map(move |c| (r, c)))
            .map(|(r, c)| a[r * n + c].powi(2))
            .sum();
        let diag: f64 = (0..n).map(|k| a[k * n + k].powi(2)).sum();
        if off <= 1e-30 * diag {
            break;
        }
        for p in 0..n {
            for q in p + 1..n {
                if a[p * n + q] == 0.0 {
                    continue;
                }
                let theta = 0.5 * (a[q * n + q] - a[p * n + p]) / a[p * n + q];
                let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
                let t = if theta == 0.0 { 1.0 } else { t };
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
    let eig: Vec<f64> = (0..n).map(|k| a[k * n + k]).collect();
    let (most, least) = (eig.iter().copied().fold(0.0, f64::max), eig.iter().copied().fold(f64::INFINITY, f64::min));
    if least <= 1e-24 * most || most.is_nan() { f64::INFINITY } else { (most / least).sqrt() }
}

/// Angle in (-pi, pi].
fn wrap(a: f64) -> f64 {
    let w = a.rem_euclid(2.0 * PI);
    if w > PI { w - 2.0 * PI } else { w }
}

impl Scara {
    /// The closed form for `chain`, checked against the chain at a few joint poses; `None` where the chain is no SCARA
    /// (axes not vertical, flange off joint 4's axis).
    pub fn new(chain: &Chain, elbow: f64) -> Option<Self> {
        if chain.len() != 4 || chain.bodies.iter().map(|b| b.prismatic).ne([false, false, true, false]) {
            return None;
        }
        let frames = chain.frames(&[0.0; 4]);
        let (rot, tip) = chain.tip(&[0.0; 4]);
        let (a, b) = (frames[0].1, frames[1].1);
        let (a1, a2) = ([b[0] - a[0], b[1] - a[1]], [tip[0] - b[0], tip[1] - b[1]]);
        let scara = Self {
            base: [a[0], a[1]],
            l1: a1[0].hypot(a1[1]),
            l2: a2[0].hypot(a2[1]),
            phi1: a1[1].atan2(a1[0]),
            phi2: a2[1].atan2(a2[0]),
            z0: tip[2],
            dz: chain.tip(&[0.0, 0.0, 1.0, 0.0]).1[2] - tip[2],
            c0: rot[1][0].atan2(rot[0][0]),
            elbow,
        };
        let samples = [[0.3, 0.7, 0.05, -1.0], [-1.1, -0.4, 0.12, 2.0], [2.0, 1.5, 0.0, 0.5]];
        let fits = samples.iter().all(|q| {
            let (rot, tip) = chain.tip(q);
            let mut tool = [0.0; 4];
            scara.forward(q, &mut tool);
            (0..3).all(|k| (tool[k] - tip[k]).abs() < 1e-12)
                && (rot[2][2] - 1.0).abs() < 1e-12
                && wrap(tool[3] - rot[1][0].atan2(rot[0][0])).abs() < 1e-12
        });
        (scara.l1 > 0.0 && scara.l2 > 0.0 && scara.dz.abs() > 0.0 && fits).then_some(scara)
    }

    pub fn forward(&self, q: &[f64], tool: &mut [f64]) {
        let (t1, t12) = (q[0] + self.phi1, q[0] + q[1] + self.phi2);
        tool[0] = self.base[0] + self.l1 * t1.cos() + self.l2 * t12.cos();
        tool[1] = self.base[1] + self.l1 * t1.sin() + self.l2 * t12.sin();
        tool[2] = self.z0 + self.dz * q[2];
        tool[3] = self.c0 + q[0] + q[1] + q[3];
    }

    /// Joint 1 by the angle to the tool less the angle the elbow takes, joint 2 by the cosine law; `false` out of reach.
    pub fn inverse(&self, tool: &[f64], q: &mut [f64]) -> bool {
        let (x, y) = (tool[0] - self.base[0], tool[1] - self.base[1]);
        let cos = (x * x + y * y - self.l1 * self.l1 - self.l2 * self.l2) / (2.0 * self.l1 * self.l2);
        if !(cos.abs() <= 1.0) {
            return false;
        }
        let beta = self.elbow * cos.acos();
        let t1 = y.atan2(x) - (self.l2 * beta.sin()).atan2(self.l1 + self.l2 * beta.cos());
        q[0] = wrap(t1 - self.phi1);
        q[1] = wrap(beta - self.phi2 + self.phi1);
        q[2] = (tool[2] - self.z0) / self.dz;
        q[3] = tool[3] - self.c0 - q[0] - q[1];
        true
    }

    /// Of the arm's travel in the plane, d(x, y)/d(q1, q2): the quill and the flange's turn do not change it.
    pub fn condition(&self, q: &[f64]) -> f64 {
        let (t1, t12) = (q[0] + self.phi1, q[0] + q[1] + self.phi2);
        let (s1, c1, s12, c12) = (t1.sin(), t1.cos(), t12.sin(), t12.cos());
        let j = [-self.l1 * s1 - self.l2 * s12, -self.l2 * s12, self.l1 * c1 + self.l2 * c12, self.l2 * c12];
        condition_number(&j, 2, 2)
    }
}

/// A chain's numerical inverse in tool space: see the top of the file.
pub struct Pose {
    /// Which of x, y, z, rx, ry, rz the tool's axes are, in that order.
    pub axes: Vec<usize>,
    home: Vec<f64>,
    /// The flange's orientation at zero joints, which the tool's orientation is the turn from.
    zero_rot: M3,
    /// The length that weighs turns against travels (m per rad): the flange's distance from joint 2 at the home pose.
    length: f64,
    /// `workspace.max_condition_number`: a walk from the last solution that passes a pose beyond it may change the
    /// branch, and is walked again from the home.
    max_condition: f64,
    /// The last solution, which the next walk starts from.
    last: Mutex<Option<Vec<f64>>>,
}

impl Pose {
    /// `None` where the home is not one position per joint, the axes are not a rising selection of the six, or there are
    /// more axes than joints or more than one joint to spare.
    pub fn new(fk: Forward, home: Vec<f64>, axes: Vec<usize>, length: f64, max_condition: f64) -> Option<Self> {
        let n = home.len();
        let fits = (4..=7).contains(&n)
            && (n == axes.len() || n == axes.len() + 1)
            && axes.windows(2).all(|w| w[0] < w[1])
            && axes.iter().all(|a| *a < 6);
        fits.then(|| Self { axes, zero_rot: fk(&vec![0.0; n]).0, home, length, max_condition, last: Mutex::new(None) })
    }

    fn redundant(&self) -> bool {
        self.home.len() > self.axes.len()
    }

    /// The tool's axes at joints `q`, the angles times `scale` (1 for the axes, `length` for the solver).
    fn coordinates(&self, fk: Forward, q: &[f64], scale: f64) -> Vec<f64> {
        let (rot, pos) = fk(q);
        let turn = log(&matmul(&rot, &transpose(&self.zero_rot)));
        self.axes.iter().map(|&a| if a < 3 { pos[a] } else { scale * turn[a - 3] }).collect()
    }

    pub fn forward(&self, fk: Forward, q: &[f64], tool: &mut [f64]) {
        tool.copy_from_slice(&self.coordinates(fk, q, 1.0));
    }

    /// d (weighted coordinates) / d q (axes x joints, row by row), by central differences of 1e-6 rad (m).
    fn jacobian(&self, fk: Forward, q: &[f64]) -> Vec<f64> {
        let (m, n) = (self.axes.len(), q.len());
        let mut j = vec![0.0; m * n];
        let mut p = q.to_vec();
        for c in 0..n {
            p[c] = q[c] + 1e-6;
            let a = self.coordinates(fk, &p, self.length);
            p[c] = q[c] - 1e-6;
            let b = self.coordinates(fk, &p, self.length);
            p[c] = q[c];
            for r in 0..m {
                j[r * n + c] = (a[r] - b[r]) / 2e-6;
            }
        }
        j
    }

    /// J^T (J J^T + damping I)^-1 rhs, J of m rows and n columns.
    fn through(j: &[f64], m: usize, n: usize, rhs: &[f64], damping: f64) -> Option<Vec<f64>> {
        let jjt: Vec<f64> = (0..m * m)
            .map(|rc| {
                let (r, c) = (rc / m, rc % m);
                (0..n).map(|k| j[r * n + k] * j[c * n + k]).sum::<f64>() + if r == c { damping } else { 0.0 }
            })
            .collect();
        let y = solve(&jjt, rhs)?;
        Some((0..n).map(|k| (0..m).map(|r| j[r * n + k] * y[r]).sum()).collect())
    }

    /// Damped least squares from `q` towards the weighted coordinates `target` until what is left is below `tol` (m),
    /// at most `steps` iterations: dq = J^T (J J^T + lambda^2 I)^-1 e, lambda 1 mm. A redundant arm also moves within
    /// the null space of J towards the home, dq += (I - J^T (J J^T)^-1 J) (home - q), until that move is below 1e-11
    /// rad: there the joints are the nearest to the home of all nearby that reach the pose (the null space part of
    /// home - q is zero). `false` where it does not get there.
    fn settle(&self, fk: Forward, q: &mut [f64], target: &[f64], tol: f64, steps: usize) -> bool {
        let (m, n) = (self.axes.len(), q.len());
        // The Jacobian is kept while the error falls to a tenth each iteration (a step of a few micrometres changes it
        // by a few millionths); a redundant arm's null space wants it where the joints are.
        let (mut j, mut before) = (Vec::new(), f64::INFINITY);
        for _ in 0..steps {
            let now = self.coordinates(fk, q, self.length);
            let e: Vec<f64> = target.iter().zip(&now).map(|(t, c)| t - c).collect();
            let size = e.iter().map(|v| v * v).sum::<f64>().sqrt();
            if !size.is_finite() {
                return false;
            }
            if size < tol && !self.redundant() {
                return true;
            }
            if j.is_empty() || self.redundant() || size > 0.1 * before {
                j = self.jacobian(fk, q);
            }
            before = size;
            let mut dq = if size >= tol {
                let Some(step) = Self::through(&j, m, n, &e, 1e-6) else { return false };
                step
            } else {
                vec![0.0; n]
            };
            let mut drift = 0.0;
            if self.redundant() {
                let z: Vec<f64> = (0..n).map(|k| self.home[k] - q[k]).collect();
                let jz: Vec<f64> = (0..m).map(|r| (0..n).map(|k| j[r * n + k] * z[k]).sum()).collect();
                let Some(back) = Self::through(&j, m, n, &jz, 0.0) else { return false };
                for k in 0..n {
                    let free = z[k] - back[k];
                    dq[k] += free;
                    drift += free * free;
                }
                if size < tol && drift.sqrt() < 1e-11 {
                    return true;
                }
            }
            for (qk, d) in q.iter_mut().zip(&dq) {
                *qk += d;
            }
        }
        let now = self.coordinates(fk, q, self.length);
        now.iter().zip(target).map(|(c, t)| (t - c) * (t - c)).sum::<f64>().sqrt() < tol
    }

    /// From the joints `from` along the line in coordinates to the tool pose `tool`, in steps of at most 20 mm and 0.05
    /// rad, each settled to 1 um, the last to 1e-13 m (weighted). With `guard`, `false` where a step ends beyond
    /// `max_condition`.
    fn walk(&self, fk: Forward, from: &[f64], tool: &[f64], q: &mut [f64], guard: bool) -> bool {
        let start = self.coordinates(fk, from, 1.0);
        let steps = self
            .axes
            .iter()
            .zip(tool.iter().zip(&start))
            .map(|(&a, (t, s))| (t - s).abs() / if a < 3 { 0.02 } else { 0.05 })
            .fold(1.0, f64::max)
            .ceil() as usize;
        q.copy_from_slice(from);
        for k in 1..=steps {
            let s = k as f64 / steps as f64;
            let at: Vec<f64> = self
                .axes
                .iter()
                .enumerate()
                .map(|(i, &a)| (start[i] + s * (tool[i] - start[i])) * if a < 3 { 1.0 } else { self.length })
                .collect();
            let (tol, iterations) = if k == steps { (1e-13, 200) } else { (1e-6, 30) };
            // shortcut: a walk of one step (at most 20 mm and 0.05 rad, as along a cycle) is not checked for passing a
            // singularity; check it too where an arm's singular poses lie that close to its working poses.
            if !self.settle(fk, q, &at, tol, iterations)
                || guard && steps > 1 && !(self.condition(fk, q) <= self.max_condition)
            {
                return false;
            }
        }
        true
    }

    /// From the last solution, staying clear of singularities on the way (a walk that passes near one may change the
    /// branch); where that does not get there, from the home (as every call did before: 74 us on the six-axis arm in
    /// release, against 7.7 us from the last solution along a line of its tool). `false` where neither gets there (out
    /// of reach, or a singularity on the way that the branch cannot pass).
    pub fn inverse(&self, fk: Forward, tool: &[f64], q: &mut [f64]) -> bool {
        let last = self.last.lock().ok().and_then(|l| l.clone());
        let solved =
            last.is_some_and(|from| self.walk(fk, &from, tool, q, true)) || self.walk(fk, &self.home, tool, q, false);
        if solved && let Ok(mut l) = self.last.lock() {
            *l = Some(q.to_vec());
        }
        solved
    }

    pub fn condition(&self, fk: Forward, q: &[f64]) -> f64 {
        self.condition_of(&self.jacobian(fk, q))
    }

    /// Of the weighted Jacobian; of its transpose where it has fewer rows than columns (a redundant arm), as J^T J is
    /// singular then and J J^T tells.
    fn condition_of(&self, j: &[f64]) -> f64 {
        let (m, n) = (self.axes.len(), self.home.len());
        if m < n {
            let t: Vec<f64> = (0..n * m).map(|rc| j[(rc % m) * n + rc / m]).collect();
            condition_number(&t, n, m)
        } else {
            condition_number(j, m, n)
        }
    }

    /// Whether the orientation of a tool pose is near half a turn from the flange's at zero joints, where the rotation
    /// vector jumps.
    pub fn near_half_turn(&self, tool: &[f64]) -> bool {
        let angle2: f64 = self.axes.iter().zip(tool).filter(|(a, _)| **a >= 3).map(|(_, v)| v * v).sum();
        angle2.sqrt() > PI - 0.01
    }
}

/// The flange's distance from joint 2 (`shoulder`) at the home (at least 0.1 m), as the solver's weight of a turn.
pub fn turn_length(home_pos: V3, shoulder: V3) -> f64 {
    let d = [home_pos[0] - shoulder[0], home_pos[1] - shoulder[1], home_pos[2] - shoulder[2]];
    dot(d, d).sqrt().max(0.1)
}
