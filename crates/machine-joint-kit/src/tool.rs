//! Tool-space kinematics of the arms (`kinematics.tool_space = true`): the machine's axes are the tool's pose, not the
//! joints, so every planner, the cycle, the CLI and the app move the tool along straight lines and arcs, as on the
//! deltas.
//!
//! - SCARA (four joints: two arms about vertical axes, a vertical quill, a turning flange): x, y, z (m) of the flange
//!   and c (rad), its turn about the vertical, in closed form with the elbow as `kinematics.elbow` says.
//! - Six joints: x, y, z (m) of the flange and its orientation as a rotation vector rx, ry, rz (rad, axis times angle,
//!   in the base frame), numerically by damped least squares on the URDF chain. The solution is followed from
//!   `kinematics.home_joints` along the straight line (and the slerp of the orientation) from the home's tool pose to
//!   the target, so it stays on the branch (shoulder, elbow, wrist) the home pose is on.
//!
//! The planners interpolate each axis linearly: a line between two poses is straight, and its orientation is held
//! where both ends have the same, or turned about a fixed axis (the slerp) where the two rotation vectors are parallel,
//! as a turn about the vertical is.
//! shortcut: between rotation vectors that are not parallel the linear interpolation is not the slerp (it is up to a
//! few degrees off it for turns of a right angle); make the planner interpolate orientations by slerp when a cycle
//! needs such turns.

use sim_components::chain::solve;
use sim_components::rigid::{Chain, M3, V3, about, add, cross, dot, matmul, mul};
use std::f64::consts::PI;

/// Which kinematics maps the tool pose to the joints.
pub enum Tool {
    /// The tool pose is the joints (rad, m).
    Joints,
    Scara(Scara),
    Six(Six),
}

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

/// The six-axis arm's numerical inverse.
pub struct Six {
    home: Vec<f64>,
    /// The flange's pose at the home joints.
    home_pos: V3,
    home_rot: M3,
    /// The length that weighs turns against travels (m per rad): the flange's distance from joint 2 at the home pose.
    length: f64,
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

/// The rotation vector (axis times angle, angle up to pi) of a rotation matrix.
pub fn log(r: &M3) -> V3 {
    let vee = [r[2][1] - r[1][2], r[0][2] - r[2][0], r[1][0] - r[0][1]];
    let cos = ((r[0][0] + r[1][1] + r[2][2] - 1.0) / 2.0).clamp(-1.0, 1.0);
    let sin = 0.5 * dot(vee, vee).sqrt();
    let angle = sin.atan2(cos);
    if angle < 1e-6 {
        // sin a / a = 1 - a^2/6: below 1e-6 rad the correction is under 1e-12 of the angle.
        return vee.map(|v| 0.5 * v * (1.0 + angle * angle / 6.0));
    }
    if angle < PI - 1e-6 {
        return vee.map(|v| v * angle / (2.0 * sin));
    }
    // Near a half turn R + I = 2 k k^T (cos a + 1 small): the axis from its largest diagonal entry, its sign from vee.
    let i = (0..3).max_by(|&a, &b| r[a][a].total_cmp(&r[b][b])).unwrap_or(0);
    let col: V3 = std::array::from_fn(|k| r[k][i] + if k == i { 1.0 } else { 0.0 });
    let norm = dot(col, col).sqrt();
    let sign = if dot(col, vee) < 0.0 { -1.0 } else { 1.0 };
    col.map(|v| sign * v / norm * angle)
}

/// The rotation of a rotation vector.
pub fn exp(r: V3) -> M3 {
    let angle = dot(r, r).sqrt();
    if angle == 0.0 { about([0.0, 0.0, 1.0], 0.0) } else { about(r.map(|v| v / angle), angle) }
}

fn transpose(m: &M3) -> M3 {
    std::array::from_fn(|i| std::array::from_fn(|j| m[j][i]))
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

impl Six {
    /// `None` where the home pose is not one joint position per joint.
    pub fn new(chain: &Chain, home: Vec<f64>) -> Option<Self> {
        if home.len() != chain.len() || chain.len() != 6 {
            return None;
        }
        let (home_rot, home_pos) = chain.tip(&home);
        let shoulder = chain.frames(&home)[1].1;
        let d = [home_pos[0] - shoulder[0], home_pos[1] - shoulder[1], home_pos[2] - shoulder[2]];
        Some(Self { home, home_pos, home_rot, length: dot(d, d).sqrt().max(0.1) })
    }

    pub fn forward(chain: &Chain, q: &[f64], tool: &mut [f64]) {
        let (rot, pos) = chain.tip(q);
        tool[..3].copy_from_slice(&pos);
        tool[3..6].copy_from_slice(&log(&rot));
    }

    /// The geometric Jacobian (6 x n, row by row: the flange's speed, then its turn rate times `length`).
    fn jacobian(&self, chain: &Chain, q: &[f64]) -> Vec<f64> {
        let n = chain.len();
        let frames = chain.frames(q);
        let tip = chain.tip(q).1;
        let mut j = vec![0.0; 6 * n];
        for (c, ((rot, at), body)) in frames.iter().zip(&chain.bodies).enumerate() {
            let axis = mul(rot, body.axis);
            let (lin, ang) = if body.prismatic {
                (axis, [0.0; 3])
            } else {
                (cross(axis, [tip[0] - at[0], tip[1] - at[1], tip[2] - at[2]]), axis)
            };
            for r in 0..3 {
                j[r * n + c] = lin[r];
                j[(r + 3) * n + c] = ang[r] * self.length;
            }
        }
        j
    }

    /// What is left from joints `q` to the flange pose (`pos`, `rot`): travel, and turn times `length`.
    fn error(&self, chain: &Chain, q: &[f64], pos: V3, rot: &M3) -> [f64; 6] {
        let (r, p) = chain.tip(q);
        let turn = log(&matmul(rot, &transpose(&r))).map(|v| v * self.length);
        [pos[0] - p[0], pos[1] - p[1], pos[2] - p[2], turn[0], turn[1], turn[2]]
    }

    /// Damped least squares from `q` towards the pose until the error is below `tol` (m), at most `steps` iterations:
    /// dq = J^T (J J^T + lambda^2 I)^-1 e, lambda 1 mm. `false` where it does not get there.
    fn settle(&self, chain: &Chain, q: &mut [f64], pos: V3, rot: &M3, tol: f64, steps: usize) -> bool {
        let n = chain.len();
        for _ in 0..steps {
            let e = self.error(chain, q, pos, rot);
            let size = e.iter().map(|v| v * v).sum::<f64>().sqrt();
            if !size.is_finite() {
                return false;
            }
            if size < tol {
                return true;
            }
            let j = self.jacobian(chain, q);
            let lambda2 = 1e-6;
            let jjt: Vec<f64> = (0..36)
                .map(|rc| {
                    let (r, c) = (rc / 6, rc % 6);
                    (0..n).map(|k| j[r * n + k] * j[c * n + k]).sum::<f64>() + if r == c { lambda2 } else { 0.0 }
                })
                .collect();
            let Some(y) = solve(&jjt, &e) else { return false };
            for (k, qk) in q.iter_mut().enumerate() {
                *qk += (0..6).map(|r| j[r * n + k] * y[r]).sum::<f64>();
            }
        }
        self.error(chain, q, pos, rot).iter().map(|v| v * v).sum::<f64>().sqrt() < tol
    }

    /// From the home joints along the line from the home's flange pose to the target (its orientation by slerp), in
    /// steps of at most 20 mm and 0.05 rad, each settled to 1 um, the last to 1e-13 m; `false` where a step does not
    /// settle (out of reach, or a singularity on the way that the branch cannot pass).
    /// shortcut: every call follows the line from the home (about 90 us in release, a tool-space cycle of the arm 8 s
    /// of computing); start from the last solution nearby when the simulation needs to be faster.
    pub fn inverse(&self, chain: &Chain, tool: &[f64], q: &mut [f64]) -> bool {
        let (pos, rot) = ([tool[0], tool[1], tool[2]], exp([tool[3], tool[4], tool[5]]));
        let turn = log(&matmul(&transpose(&self.home_rot), &rot));
        let d = [pos[0] - self.home_pos[0], pos[1] - self.home_pos[1], pos[2] - self.home_pos[2]];
        let angle = dot(turn, turn).sqrt();
        let steps = (dot(d, d).sqrt() / 0.02).max(angle / 0.05).ceil().max(1.0) as usize;
        q.copy_from_slice(&self.home);
        for k in 1..=steps {
            let s = k as f64 / steps as f64;
            let at = add(self.home_pos, d.map(|v| v * s));
            let towards = matmul(&self.home_rot, &exp(turn.map(|v| v * s)));
            let (tol, iterations) = if k == steps { (1e-13, 60) } else { (1e-6, 30) };
            if !self.settle(chain, q, at, &towards, tol, iterations) {
                return false;
            }
        }
        true
    }

    pub fn condition(&self, chain: &Chain, q: &[f64]) -> f64 {
        condition_number(&self.jacobian(chain, q), 6, chain.len())
    }
}
