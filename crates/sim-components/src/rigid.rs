//! Rigid bodies in a serial chain: read from a URDF file, with their dynamics by the recursive Newton-Euler algorithm
//! (Luh, Walker and Paul 1980; Craig, "Introduction to Robotics", chapter 6). A chain of revolute and prismatic joints
//! on a fixed base; fixed joints are merged into the body before them.
//!
//! The URDF subset read: `<robot>` with `<link>` (its `<inertial>`: `<origin xyz rpy>`, `<mass value>`, `<inertia ixx
//! ixy ixz iyy iyz izz>`) and `<joint type="revolute|continuous|prismatic|fixed">` (`<parent link>`, `<child link>`,
//! `<origin xyz rpy>`, `<axis xyz>`, `<limit lower upper>`). Visuals, collisions, meshes, transmissions, Gazebo tags
//! and comments are skipped. No xacro: run `xacro` first. The links must form one chain from the root (no branches).

use sim_core::ConfigError;

pub type V3 = [f64; 3];
pub type M3 = [[f64; 3]; 3];

pub fn add(a: V3, b: V3) -> V3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
fn scale(a: V3, k: f64) -> V3 {
    [a[0] * k, a[1] * k, a[2] * k]
}
pub fn dot(a: V3, b: V3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
pub fn cross(a: V3, b: V3) -> V3 {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
pub fn mul(m: &M3, v: V3) -> V3 {
    [dot(m[0], v), dot(m[1], v), dot(m[2], v)]
}
/// m^T v.
fn tmul(m: &M3, v: V3) -> V3 {
    std::array::from_fn(|i| m[0][i] * v[0] + m[1][i] * v[1] + m[2][i] * v[2])
}
pub fn matmul(a: &M3, b: &M3) -> M3 {
    std::array::from_fn(|r| std::array::from_fn(|c| a[r][0] * b[0][c] + a[r][1] * b[1][c] + a[r][2] * b[2][c]))
}
const IDENTITY: M3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

/// URDF's roll-pitch-yaw: R = Rz(yaw) Ry(pitch) Rx(roll).
pub fn rpy(r: V3) -> M3 {
    let (sr, cr) = r[0].sin_cos();
    let (sp, cp) = r[1].sin_cos();
    let (sy, cy) = r[2].sin_cos();
    [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
}

/// Rotation by `angle` about the unit vector `k` (Rodrigues).
pub fn about(k: V3, angle: f64) -> M3 {
    let (s, c) = angle.sin_cos();
    let v = 1.0 - c;
    [
        [c + k[0] * k[0] * v, k[0] * k[1] * v - k[2] * s, k[0] * k[2] * v + k[1] * s],
        [k[1] * k[0] * v + k[2] * s, c + k[1] * k[1] * v, k[1] * k[2] * v - k[0] * s],
        [k[2] * k[0] * v - k[1] * s, k[2] * k[1] * v + k[0] * s, c + k[2] * k[2] * v],
    ]
}

pub fn transpose(m: &M3) -> M3 {
    std::array::from_fn(|i| std::array::from_fn(|j| m[j][i]))
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
    if angle < std::f64::consts::PI - 1e-6 {
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
    if angle == 0.0 { IDENTITY } else { about(r.map(|v| v / angle), angle) }
}

/// The unit quaternion (w, x, y, z) of a rotation vector.
pub fn quaternion(r: V3) -> [f64; 4] {
    let angle = dot(r, r).sqrt();
    let (s, c) = (0.5 * angle).sin_cos();
    // sin(a/2) / a -> 1/2 for a small angle.
    let k = if angle > 1e-12 { s / angle } else { 0.5 };
    [c, k * r[0], k * r[1], k * r[2]]
}

/// The rotation vector of a unit quaternion, the angle up to pi (q and -q are the same rotation).
pub fn from_quaternion(q: [f64; 4]) -> V3 {
    let q = if q[0] < 0.0 { q.map(|v| -v) } else { q };
    let s = (q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
    let angle = 2.0 * s.atan2(q[0]);
    let k = if s > 1e-12 { angle / s } else { 2.0 };
    [k * q[1], k * q[2], k * q[3]]
}

/// Spherical linear interpolation of two orientations given as rotation vectors: the turn from `a` to `b` about one
/// fixed axis, the shorter way, `s` of it done (Shoemake 1985). As a rotation vector.
pub fn slerp(a: V3, b: V3, s: f64) -> V3 {
    let (qa, mut qb) = (quaternion(a), quaternion(b));
    let mut cos: f64 = (0..4).map(|i| qa[i] * qb[i]).sum();
    if cos < 0.0 {
        (qb, cos) = (qb.map(|v| -v), -cos);
    }
    let angle = cos.clamp(-1.0, 1.0).acos();
    let (wa, wb) = if angle < 1e-9 {
        (1.0 - s, s)
    } else {
        (((1.0 - s) * angle).sin() / angle.sin(), (s * angle).sin() / angle.sin())
    };
    let q: [f64; 4] = std::array::from_fn(|i| wa * qa[i] + wb * qb[i]);
    let norm = q.iter().map(|v| v * v).sum::<f64>().sqrt();
    from_quaternion(q.map(|v| v / norm))
}

/// Mass, centre of mass and inertia about it, in a body's frame.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Inertial {
    pub mass: f64,
    pub com: V3,
    pub inertia: M3,
}

impl Inertial {
    /// A point mass at `com`.
    pub fn point(mass: f64, com: V3) -> Inertial {
        Inertial { mass, com, inertia: [[0.0; 3]; 3] }
    }

    /// The two bodies as one, `other` given in a frame that `rot`, `pos` put into this one (parallel axis theorem).
    pub fn merged(&self, other: &Inertial, rot: &M3, pos: V3) -> Inertial {
        let m = self.mass + other.mass;
        if m <= 0.0 {
            return *self;
        }
        let oc = add(mul(rot, other.com), pos);
        let com = scale(add(scale(self.com, self.mass), scale(oc, other.mass)), 1.0 / m);
        let shift = |mass: f64, d: V3| -> M3 {
            std::array::from_fn(|r| {
                std::array::from_fn(|c| mass * (if r == c { dot(d, d) } else { 0.0 } - d[r] * d[c]))
            })
        };
        let turned = matmul(&matmul(rot, &other.inertia), &transpose(rot));
        let (a, b) = (shift(self.mass, sub(self.com, com)), shift(other.mass, sub(oc, com)));
        let inertia =
            std::array::from_fn(|r| std::array::from_fn(|c| self.inertia[r][c] + a[r][c] + turned[r][c] + b[r][c]));
        Inertial { mass: m, com, inertia }
    }

    /// Mass, first moment m c and inertia about the frame's origin all times `factor`, then `moment` added to the first
    /// moment, the mass and the inertia about the origin kept (the parameters the dynamics is linear in); back as mass,
    /// centre and inertia about it. `factor` has to be positive.
    pub fn changed(&self, factor: f64, moment: V3) -> Inertial {
        let shift = |m: f64, c: V3| -> M3 {
            std::array::from_fn(|r| std::array::from_fn(|k| m * (if r == k { dot(c, c) } else { 0.0 } - c[r] * c[k])))
        };
        let about_origin = shift(self.mass, self.com);
        let mass = factor * self.mass;
        let com = scale(add(scale(self.com, factor * self.mass), moment), 1.0 / mass);
        let back = shift(mass, com);
        let inertia = std::array::from_fn(|r| {
            std::array::from_fn(|k| factor * (self.inertia[r][k] + about_origin[r][k]) - back[r][k])
        });
        Inertial { mass, com, inertia }
    }
}

fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

/// One moving joint and the body it carries.
#[derive(Debug, Clone, PartialEq)]
pub struct Body {
    pub name: String,
    /// Where the joint sits in the frame of the body before it, at zero joint position.
    pub origin_rot: M3,
    pub origin_pos: V3,
    /// Unit axis in the joint's frame.
    pub axis: V3,
    pub prismatic: bool,
    pub inertial: Inertial,
    /// The joint's limits from the file (rad or m); `None` for a continuous joint.
    pub limits: Option<(f64, f64)>,
}

/// A serial chain on a fixed base, z up in the base frame.
#[derive(Debug, Clone, PartialEq)]
pub struct Chain {
    pub bodies: Vec<Body>,
    /// A frame fixed to the last body (a flange or tool after fixed joints), in its frame.
    pub tip_rot: M3,
    pub tip_pos: V3,
}

/// Each joint's frame in the base frame: rotation and position.
pub type Frames = Vec<(M3, V3)>;

impl Chain {
    pub fn len(&self) -> usize {
        self.bodies.len()
    }
    pub fn is_empty(&self) -> bool {
        self.bodies.is_empty()
    }

    /// The joint's own motion: rotation and offset of body i's frame against the joint's frame at zero.
    fn joint_motion(b: &Body, q: f64) -> (M3, V3) {
        if b.prismatic { (IDENTITY, scale(b.axis, q)) } else { (about(b.axis, q), [0.0; 3]) }
    }

    /// Body i's frame in the frame before it: R_i and p_i.
    fn link(&self, i: usize, q: f64) -> (M3, V3) {
        let b = &self.bodies[i];
        let (r, d) = Self::joint_motion(b, q);
        (matmul(&b.origin_rot, &r), add(b.origin_pos, mul(&b.origin_rot, d)))
    }

    /// Every body's frame in the base frame at joint positions `q`.
    pub fn frames(&self, q: &[f64]) -> Frames {
        let (mut rot, mut pos) = (IDENTITY, [0.0; 3]);
        (0..self.len())
            .map(|i| {
                let (r, p) = self.link(i, q[i]);
                pos = add(pos, mul(&rot, p));
                rot = matmul(&rot, &r);
                (rot, pos)
            })
            .collect()
    }

    /// The tip's position in the base frame.
    pub fn tip(&self, q: &[f64]) -> (M3, V3) {
        let (rot, pos) = *self.frames(q).last().unwrap_or(&(IDENTITY, [0.0; 3]));
        (matmul(&rot, &self.tip_rot), add(pos, mul(&rot, self.tip_pos)))
    }

    /// Recursive Newton-Euler: the joint torques (forces for prismatic joints) that give the accelerations `qdd` at
    /// positions `q` and speeds `qd` under gravity `g` (m/s^2, along -z of the base), `extra` a payload (mass, centre and
    /// inertia about it) in the last body's frame. Out: `tau`.
    pub fn inverse_dynamics(
        &self,
        q: &[f64],
        qd: &[f64],
        qdd: &[f64],
        g: f64,
        extra: Option<Inertial>,
        tau: &mut [f64],
    ) {
        let links: Vec<(M3, V3)> = (0..self.len()).map(|i| self.link(i, q[i])).collect();
        self.newton_euler(&links, qd, qdd, g, extra, tau);
    }

    /// `inverse_dynamics` with each body's frame in the one before it (`link`) worked out already: the mass matrix
    /// takes n + 1 passes at the same positions, and the links' rotations are most of a pass's work.
    fn newton_euler(
        &self,
        links: &[(M3, V3)],
        qd: &[f64],
        qdd: &[f64],
        g: f64,
        extra: Option<Inertial>,
        tau: &mut [f64],
    ) {
        let n = self.len();
        let mut forces: Vec<(V3, V3)> = Vec::with_capacity(n);
        // The base accelerates up by g instead of gravity pulling down.
        let (mut w, mut dw, mut a) = ([0.0; 3], [0.0; 3], [0.0, 0.0, g]);
        for i in 0..n {
            let b = &self.bodies[i];
            let (r, p) = links[i];
            // The previous origin's acceleration carried to this origin, then into this frame.
            let a_here = tmul(&r, add(a, add(cross(dw, p), cross(w, cross(w, p)))));
            let (w_in, dw_in) = (tmul(&r, w), tmul(&r, dw));
            if b.prismatic {
                (w, dw) = (w_in, dw_in);
                a = add(a_here, add(scale(cross(w, b.axis), 2.0 * qd[i]), scale(b.axis, qdd[i])));
            } else {
                w = add(w_in, scale(b.axis, qd[i]));
                dw = add(add(dw_in, scale(b.axis, qdd[i])), scale(cross(w_in, b.axis), qd[i]));
                a = a_here;
            }
            let mut body = b.inertial;
            if let (true, Some(payload)) = (i + 1 == n, extra) {
                body = body.merged(&payload, &IDENTITY, [0.0; 3]);
            }
            let ac = add(a, add(cross(dw, body.com), cross(w, cross(w, body.com))));
            let force = scale(ac, body.mass);
            let moment = add(mul(&body.inertia, dw), cross(w, mul(&body.inertia, w)));
            forces.push((force, add(moment, cross(body.com, force))));
        }
        // Back from the tip: what each body passes to the one before it, about its own origin.
        let (mut f, mut m) = ([0.0; 3], [0.0; 3]);
        for i in (0..n).rev() {
            let (fi, mi) = forces[i];
            let (f_next, m_next) = match links.get(i + 1) {
                Some((r, p)) => {
                    let fr = mul(r, f);
                    (fr, add(mul(r, m), cross(*p, fr)))
                }
                None => ([0.0; 3], [0.0; 3]),
            };
            f = add(fi, f_next);
            m = add(mi, m_next);
            tau[i] = dot(if self.bodies[i].prismatic { f } else { m }, self.bodies[i].axis);
        }
    }

    /// The mass matrix M (n x n, row by row): column j is the inverse dynamics at rest without gravity for a unit
    /// acceleration of joint j.
    pub fn mass_matrix(&self, q: &[f64], extra: Option<Inertial>, out: &mut [f64]) {
        let links: Vec<(M3, V3)> = (0..self.len()).map(|i| self.link(i, q[i])).collect();
        self.mass_matrix_at(&links, extra, out);
    }

    /// The mass matrix from each body's Jacobian, in one pass along the chain: M = sum over the bodies k of
    /// m_k Jv_k^T Jv_k + Jw_k^T I_k Jw_k, its centre of mass's and its rotation's Jacobian over the joints up to it (a
    /// revolute joint j moves it by a_j x (c_k - o_j) and turns it by a_j, a prismatic one moves it by a_j), I_k its
    /// inertia about its centre in the base frame. The same matrix as n passes of Newton-Euler with a unit acceleration
    /// each, at a fraction of the work.
    fn mass_matrix_at(&self, links: &[(M3, V3)], extra: Option<Inertial>, out: &mut [f64]) {
        let n = self.len();
        out[..n * n].fill(0.0);
        let (mut rot, mut pos) = (IDENTITY, [0.0; 3]);
        let (mut axes, mut origins) = (Vec::with_capacity(n), Vec::with_capacity(n));
        let mut columns: Vec<(V3, V3)> = Vec::with_capacity(n);
        for k in 0..n {
            let (r, p) = links[k];
            pos = add(pos, mul(&rot, p));
            rot = matmul(&rot, &r);
            let b = &self.bodies[k];
            axes.push(mul(&rot, b.axis));
            origins.push(pos);
            let mut body = b.inertial;
            if let (true, Some(payload)) = (k + 1 == n, extra) {
                body = body.merged(&payload, &IDENTITY, [0.0; 3]);
            }
            let c = add(pos, mul(&rot, body.com));
            let turned = matmul(&matmul(&rot, &body.inertia), &transpose(&rot));
            columns.clear();
            columns.extend((0..=k).map(|j| {
                if self.bodies[j].prismatic {
                    (axes[j], [0.0; 3])
                } else {
                    (cross(axes[j], sub(c, origins[j])), axes[j])
                }
            }));
            for (i, (vi, wi)) in columns.iter().enumerate() {
                let iw = mul(&turned, *wi);
                for (j, (vj, wj)) in columns.iter().enumerate() {
                    out[i * n + j] += body.mass * dot(*vi, *vj) + dot(iw, *wj);
                }
            }
        }
    }

    /// The mass matrix (`mass_matrix`) and the torques at zero acceleration (velocity terms and gravity,
    /// `inverse_dynamics` with q'' = 0) at once, the links worked out once for both.
    pub fn dynamics(&self, q: &[f64], qd: &[f64], g: f64, extra: Option<Inertial>, mass: &mut [f64], bias: &mut [f64]) {
        let links: Vec<(M3, V3)> = (0..self.len()).map(|i| self.link(i, q[i])).collect();
        self.mass_matrix_at(&links, extra, mass);
        self.newton_euler(&links, qd, &vec![0.0; q.len()], g, extra, bias);
    }

    /// Potential energy in gravity g: the sum of m g z of every centre of mass.
    pub fn potential(&self, q: &[f64], g: f64, extra: Option<Inertial>) -> f64 {
        let frames = self.frames(q);
        let n = self.len();
        frames
            .iter()
            .enumerate()
            .map(|(i, (rot, pos))| {
                let b = &self.bodies[i].inertial;
                let mut energy = b.mass * g * add(*pos, mul(rot, b.com))[2];
                if let (true, Some(payload)) = (i + 1 == n, extra) {
                    energy += payload.mass * g * add(*pos, mul(rot, payload.com))[2];
                }
                energy
            })
            .sum()
    }

    /// Reads a chain from URDF text.
    pub fn from_urdf(text: &str) -> Result<Self, ConfigError> {
        let bad = |what: &str| ConfigError::Invalid(format!("urdf: {what}"));
        let elements = xml_elements(text).map_err(|e| bad(&e))?;
        // Links by name with their inertials; joints as (type, parent, child, origin, axis, limits).
        let mut links: Vec<(String, Inertial)> = Vec::new();
        struct Joint {
            kind: String,
            parent: String,
            child: String,
            rot: M3,
            pos: V3,
            axis: V3,
            limits: Option<(f64, f64)>,
        }
        let mut joints: Vec<Joint> = Vec::new();
        let mut i = 0;
        while i < elements.len() {
            let e = &elements[i];
            match (e.name.as_str(), e.depth) {
                ("link", 1) => {
                    let name = e.attr("name").ok_or_else(|| bad("a link without a name"))?.to_string();
                    let mut inertial = Inertial::default();
                    let (mut rot, mut origin) = (IDENTITY, [0.0; 3]);
                    let mut k = i + 1;
                    let mut in_inertial = false;
                    while k < elements.len() && elements[k].depth > 1 {
                        let c = &elements[k];
                        if c.depth == 2 {
                            in_inertial = c.name == "inertial";
                        } else if in_inertial && c.depth == 3 {
                            match c.name.as_str() {
                                "origin" => {
                                    origin = c.vec("xyz")?;
                                    rot = rpy(c.vec("rpy")?);
                                }
                                "mass" => inertial.mass = c.number("value")?,
                                "inertia" => {
                                    let v = |k: &str| c.number(k);
                                    let (xx, xy, xz, yy, yz, zz) =
                                        (v("ixx")?, v("ixy")?, v("ixz")?, v("iyy")?, v("iyz")?, v("izz")?);
                                    inertial.inertia = [[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]];
                                }
                                _ => {}
                            }
                        }
                        k += 1;
                    }
                    if !(inertial.mass >= 0.0) || inertial.inertia.iter().flatten().any(|v| !v.is_finite()) {
                        return Err(bad(&format!("the inertial of link {name}")));
                    }
                    inertial.com = origin;
                    inertial.inertia = matmul(&matmul(&rot, &inertial.inertia), &transpose(&rot));
                    links.push((name, inertial));
                    i = k;
                    continue;
                }
                ("joint", 1) => {
                    let kind = e.attr("type").ok_or_else(|| bad("a joint without a type"))?.to_string();
                    let mut j = Joint {
                        kind,
                        parent: String::new(),
                        child: String::new(),
                        rot: IDENTITY,
                        pos: [0.0; 3],
                        axis: [1.0, 0.0, 0.0],
                        limits: None,
                    };
                    let mut k = i + 1;
                    while k < elements.len() && elements[k].depth > 1 {
                        let c = &elements[k];
                        if c.depth == 2 {
                            match c.name.as_str() {
                                "parent" => j.parent = c.attr("link").unwrap_or_default().to_string(),
                                "child" => j.child = c.attr("link").unwrap_or_default().to_string(),
                                "origin" => {
                                    j.pos = c.vec("xyz")?;
                                    j.rot = rpy(c.vec("rpy")?);
                                }
                                "axis" => j.axis = c.vec("xyz")?,
                                // URDF's default for a bound left out is 0; a range that is empty or upside down is refused.
                                "limit" if c.attr("lower").is_some() || c.attr("upper").is_some() => {
                                    let bound =
                                        |name: &str| if c.attr(name).is_some() { c.number(name) } else { Ok(0.0) };
                                    let (lower, upper) = (bound("lower")?, bound("upper")?);
                                    if lower >= upper {
                                        return Err(bad(&format!("the limits of the joint to {}", j.child)));
                                    }
                                    j.limits = Some((lower, upper));
                                }
                                _ => {}
                            }
                        }
                        k += 1;
                    }
                    joints.push(j);
                    i = k;
                    continue;
                }
                _ => {}
            }
            i += 1;
        }
        // The root: the link that is no joint's child.
        let root = links
            .iter()
            .find(|(name, _)| joints.iter().all(|j| &j.child != name))
            .ok_or_else(|| bad("no root link"))?
            .0
            .clone();
        let inertial_of = |name: &str| {
            links.iter().find(|(n, _)| n == name).map(|(_, i)| *i).ok_or_else(|| bad(&format!("no link {name}")))
        };
        let mut chain = Chain { bodies: Vec::new(), tip_rot: IDENTITY, tip_pos: [0.0; 3] };
        // Fixed joints before the first moving one fold into where it sits on the base.
        let (mut pre_rot, mut pre_pos) = (IDENTITY, [0.0; 3]);
        let mut current = root;
        loop {
            let next: Vec<&Joint> = joints.iter().filter(|j| j.parent == current).collect();
            let j = match next[..] {
                [] => break,
                [j] => j,
                _ => return Err(bad(&format!("link {current} branches; only a serial chain is read"))),
            };
            let child = inertial_of(&j.child)?;
            match j.kind.as_str() {
                "fixed" => {
                    // Into the body before it, or on the way to the first joint.
                    let (rot, pos) = (matmul(&chain.tip_rot, &j.rot), add(chain.tip_pos, mul(&chain.tip_rot, j.pos)));
                    if let Some(last) = chain.bodies.last_mut() {
                        last.inertial = last.inertial.merged(&child, &rot, pos);
                        (chain.tip_rot, chain.tip_pos) = (rot, pos);
                    } else {
                        pre_pos = add(pre_pos, mul(&pre_rot, j.pos));
                        pre_rot = matmul(&pre_rot, &j.rot);
                    }
                }
                "revolute" | "continuous" | "prismatic" => {
                    let norm = dot(j.axis, j.axis).sqrt();
                    if !(norm > 0.0) {
                        return Err(bad(&format!("the axis of the joint to {}", j.child)));
                    }
                    let (rot, pos) = if chain.bodies.is_empty() {
                        (matmul(&pre_rot, &j.rot), add(pre_pos, mul(&pre_rot, j.pos)))
                    } else {
                        (matmul(&chain.tip_rot, &j.rot), add(chain.tip_pos, mul(&chain.tip_rot, j.pos)))
                    };
                    let limits = if j.kind == "continuous" { None } else { j.limits };
                    chain.bodies.push(Body {
                        name: j.child.clone(),
                        origin_rot: rot,
                        origin_pos: pos,
                        axis: scale(j.axis, 1.0 / norm),
                        prismatic: j.kind == "prismatic",
                        inertial: child,
                        limits,
                    });
                    (chain.tip_rot, chain.tip_pos) = (IDENTITY, [0.0; 3]);
                }
                other => return Err(bad(&format!("joint type {other} is not read"))),
            }
            current = j.child.clone();
        }
        if chain.bodies.is_empty() {
            return Err(bad("no moving joint"));
        }
        Ok(chain)
    }
}

/// One XML start tag: its name, depth (the root at 0) and attributes.
#[derive(Debug, Clone)]
struct Element {
    name: String,
    depth: usize,
    attrs: Vec<(String, String)>,
}

impl Element {
    fn attr(&self, name: &str) -> Option<&str> {
        self.attrs.iter().find(|(k, _)| k == name).map(|(_, v)| v.as_str())
    }
    fn number(&self, name: &str) -> Result<f64, ConfigError> {
        self.attr(name)
            .and_then(|v| v.trim().parse::<f64>().ok())
            .filter(|v| v.is_finite())
            .ok_or_else(|| ConfigError::Invalid(format!("urdf: {} {name}", self.name)))
    }
    /// Three numbers apart by spaces; zero where the attribute is left out (URDF's default).
    fn vec(&self, name: &str) -> Result<V3, ConfigError> {
        let Some(text) = self.attr(name) else { return Ok([0.0; 3]) };
        let v: Vec<f64> = text
            .split_whitespace()
            .map(|t| t.parse::<f64>())
            .collect::<Result<_, _>>()
            .map_err(|_| ConfigError::Invalid(format!("urdf: {} {name}", self.name)))?;
        match v[..] {
            [a, b, c] if v.iter().all(|x| x.is_finite()) => Ok([a, b, c]),
            _ => Err(ConfigError::Invalid(format!("urdf: {} {name}", self.name))),
        }
    }
}

/// The start tags of an XML document in order, with their depth. Comments, processing instructions, declarations and
/// text are skipped; end tags only count the depth. Enough for URDF, not a general XML reader (no entities but the
/// five standard ones, no CDATA).
fn xml_elements(text: &str) -> Result<Vec<Element>, String> {
    let mut out = Vec::new();
    let (mut rest, mut depth) = (text, 0usize);
    while let Some(start) = rest.find('<') {
        rest = &rest[start..];
        if let Some(after) = rest.strip_prefix("<!--") {
            let end = after.find("-->").ok_or("a comment that does not end")?;
            rest = &after[end + 3..];
            continue;
        }
        if rest.starts_with("<?") || rest.starts_with("<!") {
            let end = rest.find('>').ok_or("a declaration that does not end")?;
            rest = &rest[end + 1..];
            continue;
        }
        // A '>' inside a quoted attribute does not end the tag.
        let (mut quote, mut end) = (None, None);
        for (k, ch) in rest.char_indices().skip(1) {
            match (quote, ch) {
                (None, '"' | '\'') => quote = Some(ch),
                (Some(q), c) if c == q => quote = None,
                (None, '>') => {
                    end = Some(k);
                    break;
                }
                _ => {}
            }
        }
        let end = end.ok_or("a tag that does not end")?;
        let tag = &rest[1..end];
        rest = &rest[end + 1..];
        if let Some(closing) = tag.strip_prefix('/') {
            depth = depth.checked_sub(1).ok_or_else(|| format!("</{}> without its start", closing.trim()))?;
            continue;
        }
        let empty = tag.ends_with('/');
        let tag = tag.trim_end_matches('/');
        let name_end = tag.find(char::is_whitespace).unwrap_or(tag.len());
        let mut attrs = Vec::new();
        let mut a = tag[name_end..].trim_start();
        while !a.is_empty() {
            let eq = a.find('=').ok_or("an attribute without a value")?;
            let key = a[..eq].trim().to_string();
            let v = a[eq + 1..].trim_start();
            let q = v.chars().next().filter(|c| *c == '"' || *c == '\'').ok_or("an attribute value without quotes")?;
            let close = v[1..].find(q).ok_or("an attribute value that does not end")?;
            let value = v[1..1 + close]
                .replace("&lt;", "<")
                .replace("&gt;", ">")
                .replace("&quot;", "\"")
                .replace("&apos;", "'")
                .replace("&amp;", "&");
            attrs.push((key, value));
            a = v[close + 2..].trim_start();
        }
        out.push(Element { name: tag[..name_end].to_string(), depth, attrs });
        if !empty {
            depth += 1;
        }
    }
    if depth != 0 {
        return Err("tags that are not closed".into());
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A double pendulum in the x-z plane: two revolute joints about y, links 1 m and 0.5 m, point masses 2 kg and 1 kg at
    /// their ends, and a fixed tool 0.6 m from the elbow with 0.5 kg that merges into the second body.
    const PENDULUM: &str = r#"<?xml version="1.0"?>
<!-- a test chain -->
<robot name="pendulum">
  <link name="base"/>
  <link name="upper"><inertial><origin xyz="1 0 0" rpy="0 0 0"/><mass value="2"/><inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/></inertial>
    <visual><geometry><box size="1 0.1 0.1"/></geometry></visual></link>
  <link name="lower"><inertial><origin xyz="0.5 0 0"/><mass value="1"/><inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/></inertial></link>
  <link name="tool"><inertial><origin xyz="0 0 0"/><mass value="0.5"/><inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/></inertial></link>
  <joint name="j1" type="revolute"><parent link="base"/><child link="upper"/><origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 -1 0"/><limit lower="-3" upper="3" effort="10" velocity="1"/></joint>
  <joint name="j2" type="continuous"><parent link="upper"/><child link="lower"/><origin xyz="1 0 0"/><axis xyz="0 -1 0"/></joint>
  <joint name="flange" type="fixed"><parent link="lower"/><child link="tool"/><origin xyz="0.6 0 0"/></joint>
</robot>"#;

    /// The textbook double pendulum (Craig, example 6.5), with the tool's 0.5 kg at 0.6 m on the second link: the
    /// second body is 1.5 kg with its centre at (0.5 + 0.3) / 1.5 = 0.53333 m and 0.25 + 0.18 = 0.43 kg m^2 about the
    /// joint. Axes about -y turn x towards +z, so the angles lift the links. At q = (0, 90 degrees), at rest, gravity
    /// 9.81: tau1 = 2 9.81 1 + 1.5 9.81 1 = 34.335 N m (the second link points up), tau2 = 0. M at q2 = 90 degrees:
    /// M11 = 2 + 1.5 1 + 0.43 = 3.93 (cos q2 = 0), M12 = 0.43, M22 = 0.43. With qd1 = 1 rad/s the centrifugal force
    /// on the second body's centre pulls it outwards along the first link: tau2 = m2 l1 c2 sin(q2) qd1^2 = 1.5 1 0.53333
    /// = 0.8 N m (gravity off).
    #[test]
    fn newton_euler_gives_the_textbook_pendulum() {
        let chain = Chain::from_urdf(PENDULUM).unwrap();
        assert_eq!(chain.len(), 2);
        assert_eq!(chain.bodies[0].limits, Some((-3.0, 3.0)));
        assert_eq!(chain.bodies[1].limits, None);
        let close = |a: f64, b: f64, what: &str| assert!((a - b).abs() < 1e-9, "{what}: {a} against {b} by hand");
        close(chain.bodies[1].inertial.mass, 1.5, "merged mass");
        close(chain.bodies[1].inertial.com[0], 0.8 / 1.5, "merged centre");
        let q = [0.0, std::f64::consts::FRAC_PI_2];
        let mut tau = [0.0; 2];
        chain.inverse_dynamics(&q, &[0.0; 2], &[0.0; 2], 9.81, None, &mut tau);
        close(tau[0], 34.335, "tau1");
        close(tau[1], 0.0, "tau2");
        let mut m = [0.0; 4];
        chain.mass_matrix(&q, None, &mut m);
        for (got, want) in m.iter().zip([3.93, 0.43, 0.43, 0.43]) {
            close(*got, want, "M");
        }
        chain.inverse_dynamics(&q, &[1.0, 0.0], &[0.0; 2], 0.0, None, &mut tau);
        close(tau[1], 0.8, "centrifugal");
        // The potential: the first mass at height 0, the second body's centre 0.53333 m above the elbow.
        close(chain.potential(&q, 9.81, None), 1.5 * 9.81 * 0.8 / 1.5, "potential");
        // The tool tip 0.6 m above the elbow at (1, 0, 0.6); a payload anywhere above the elbow adds its weight times the
        // reach of 1 m to tau1.
        let (_, tip) = chain.tip(&q);
        close(tip[0], 1.0, "tip x");
        close(tip[2], 0.6, "tip z");
        chain.inverse_dynamics(&q, &[0.0; 2], &[0.0; 2], 9.81, Some(Inertial::point(2.0, [1.1, 0.0, 0.0])), &mut tau);
        close(tau[0], 34.335 + 2.0 * 9.81, "payload");
    }

    /// The energy check of the dynamics: d/dt (T + V) = qd . tau. Over a small step with the accelerations M^-1 (tau - h),
    /// the change of T + V matches the power put in, for a random-ish pose of a 3D chain.
    #[test]
    fn the_dynamics_conserve_energy() {
        let urdf = r#"<robot name="r"><link name="b"/>
            <link name="l1"><inertial><origin xyz="0.1 0.02 0.3" rpy="0.1 0.2 0.3"/><mass value="5"/><inertia ixx="0.1" ixy="0.01" ixz="0" iyy="0.12" iyz="0.02" izz="0.05"/></inertial></link>
            <link name="l2"><inertial><origin xyz="0.2 0 0.05"/><mass value="3"/><inertia ixx="0.02" ixy="0" ixz="0.003" iyy="0.06" iyz="0" izz="0.05"/></inertial></link>
            <link name="l3"><inertial><origin xyz="0 0.1 0"/><mass value="1"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.004" iyz="0" izz="0.01"/></inertial></link>
            <joint name="a" type="revolute"><parent link="b"/><child link="l1"/><origin xyz="0 0 0.4"/><axis xyz="0 0 1"/><limit lower="-3" upper="3"/></joint>
            <joint name="b" type="revolute"><parent link="l1"/><child link="l2"/><origin xyz="0.1 0 0.3" rpy="1.5707963 0 0"/><axis xyz="0 0 1"/><limit lower="-3" upper="3"/></joint>
            <joint name="c" type="prismatic"><parent link="l2"/><child link="l3"/><origin xyz="0.4 0 0"/><axis xyz="1 0 0"/><limit lower="0" upper="0.2"/></joint></robot>"#;
        let chain = Chain::from_urdf(urdf).unwrap();
        let (q0, qd0, tau) = ([0.3, -0.7, 0.1], [0.5, -0.4, 0.2], [1.0, -2.0, 3.0]);
        let accel = |q: &[f64], qd: &[f64]| -> Vec<f64> {
            let mut m = [0.0; 9];
            chain.mass_matrix(q, None, &mut m);
            let mut h = [0.0; 3];
            chain.inverse_dynamics(q, qd, &[0.0; 3], 9.81, None, &mut h);
            let rhs: Vec<f64> = (0..3).map(|i| tau[i] - h[i]).collect();
            crate::chain::solve(&m, &rhs).unwrap()
        };
        let energy = |q: &[f64], qd: &[f64]| {
            let mut m = [0.0; 9];
            chain.mass_matrix(q, None, &mut m);
            let kinetic: f64 =
                (0..3).map(|i| (0..3).map(|j| qd[i] * m[i * 3 + j] * qd[j]).sum::<f64>()).sum::<f64>() * 0.5;
            kinetic + chain.potential(q, 9.81, None)
        };
        // RK4 over 1 ms; the energy gained is the work of tau (constant) over the joint travel.
        let dt = 1e-3;
        let f = |s: &[f64]| -> Vec<f64> {
            let a = accel(&s[..3], &s[3..]);
            s[3..].iter().copied().chain(a).collect()
        };
        let s0: Vec<f64> = q0.iter().chain(&qd0).copied().collect();
        let step = |s: &[f64], k: &[f64], h: f64| -> Vec<f64> { s.iter().zip(k).map(|(a, b)| a + h * b).collect() };
        let k1 = f(&s0);
        let k2 = f(&step(&s0, &k1, dt / 2.0));
        let k3 = f(&step(&s0, &k2, dt / 2.0));
        let k4 = f(&step(&s0, &k3, dt));
        let s1: Vec<f64> = (0..6).map(|i| s0[i] + dt / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i])).collect();
        let work: f64 = (0..3).map(|i| tau[i] * (s1[i] - s0[i])).sum();
        let gained = energy(&s1[..3], &s1[3..]) - energy(&q0, &qd0);
        assert!((gained - work).abs() < 1e-9, "gained {gained} J, work {work} J");
    }

    #[test]
    fn bad_urdf_is_refused() {
        for bad in [
            "<robot><link name=\"a\"/>",
            "<robot><link name=\"a\"/></robot>",
            "<robot><link name=\"a\"/><link name=\"b\"/><joint name=\"j\" type=\"planar\"><parent link=\"a\"/><child link=\"b\"/></joint></robot>",
        ] {
            assert!(Chain::from_urdf(bad).is_err(), "{bad}");
        }
    }

    /// URDF's limits: a bound left out is 0, an empty or upside-down range is refused (it would turn the stops inside
    /// out).
    #[test]
    fn urdf_limits_default_to_zero_and_must_be_a_range() {
        let urdf = |limit: &str| {
            format!(
                "<robot><link name=\"a\"/><link name=\"b\"><inertial><mass value=\"1\"/><inertia ixx=\"1\" ixy=\"0\" ixz=\"0\" iyy=\"1\" iyz=\"0\" izz=\"1\"/></inertial></link><joint name=\"j\" type=\"prismatic\"><parent link=\"a\"/><child link=\"b\"/><axis xyz=\"0 0 1\"/>{limit}</joint></robot>"
            )
        };
        assert_eq!(
            Chain::from_urdf(&urdf("<limit upper=\"0.3\" effort=\"1\" velocity=\"1\"/>")).unwrap().bodies[0].limits,
            Some((0.0, 0.3))
        );
        assert!(Chain::from_urdf(&urdf("<limit lower=\"0.2\" upper=\"0.0\"/>")).is_err());
    }

    /// 2 kg with its centre at (0.1, 0, 0) and inertia diag(0.01, 0.02, 0.03) about it, times 3 with 0.2 kg m more first
    /// moment along y. By hand: about the frame, diag(0.01, 0.02 + 2 · 0.01, 0.03 + 2 · 0.01) = diag(0.01, 0.04, 0.05),
    /// times 3; mass 6 kg; first moment (0.6, 0.2, 0), centre (0.1, 1/30, 0), c · c = 0.011111; back about the centre:
    /// xx 0.03 − 6 (0.011111 − 0.01) = 0.023333, yy 0.12 − 6 (0.011111 − 0.001111) = 0.06, zz 0.15 − 6 · 0.011111 =
    /// 0.083333, xy 0 + 6 · 0.1 / 30 = 0.02. The factor 1 without a moment gives the body back.
    #[test]
    fn a_links_change_scales_its_parameters_about_its_frame_and_moves_its_centre() {
        let body = Inertial {
            mass: 2.0,
            com: [0.1, 0.0, 0.0],
            inertia: [[0.01, 0.0, 0.0], [0.0, 0.02, 0.0], [0.0, 0.0, 0.03]],
        };
        let changed = body.changed(3.0, [0.0, 0.2, 0.0]);
        let close = |a: f64, b: f64| (a - b).abs() < 1e-12;
        assert!(close(changed.mass, 6.0) && close(changed.com[0], 0.1) && close(changed.com[1], 1.0 / 30.0));
        let expected = [[0.07 / 3.0, 0.02, 0.0], [0.02, 0.06, 0.0], [0.0, 0.0, 0.25 / 3.0]];
        for r in 0..3 {
            for c in 0..3 {
                assert!(close(changed.inertia[r][c], expected[r][c]), "{:?}", changed.inertia);
            }
        }
        let same = body.changed(1.0, [0.0; 3]);
        assert!((0..3).all(|r| (0..3).all(|c| close(same.inertia[r][c], body.inertia[r][c]))) && same.com == body.com);
    }

    /// A payload's own inertia counts about the last joint: on the textbook pendulum (second body 0.43 kg m^2 about the
    /// elbow), 2 kg at 0.6 m along the second link with 0.05 kg m^2 about its centre around the joint's axis (y). At
    /// rest, no gravity, q2'' = 1 rad/s^2: tau2 = 0.43 + 2 * 0.6^2 + 0.05 = 1.2 N m, of which the point mass alone gives
    /// 1.15 N m.
    #[test]
    fn a_payloads_inertia_adds_its_torque_about_the_last_joint() {
        let chain = Chain::from_urdf(PENDULUM).unwrap();
        let mut tau = [0.0; 2];
        let solid = Inertial {
            mass: 2.0,
            com: [0.6, 0.0, 0.0],
            inertia: [[0.01, 0.0, 0.0], [0.0, 0.05, 0.0], [0.0, 0.0, 0.02]],
        };
        chain.inverse_dynamics(&[0.0; 2], &[0.0; 2], &[0.0, 1.0], 0.0, Some(solid), &mut tau);
        assert!((tau[1] - 1.2).abs() < 1e-12, "{} N m against 1.2 by hand", tau[1]);
        chain.inverse_dynamics(
            &[0.0; 2],
            &[0.0; 2],
            &[0.0, 1.0],
            0.0,
            Some(Inertial::point(2.0, [0.6, 0.0, 0.0])),
            &mut tau,
        );
        assert!((tau[1] - 1.15).abs() < 1e-12, "{} N m against 1.15 by hand", tau[1]);
    }

    /// The slerp midpoint of a quarter turn about x and a quarter turn about y. By hand with quaternions:
    /// qa = (c, s, 0, 0), qb = (c, 0, s, 0), c = s = cos 45 deg; cos of the angle between them c^2 = 1/2, so 60 degrees;
    /// the midpoint (qa + qb) / |qa + qb| = (2c, s, s, 0) / sqrt(4c^2 + 2s^2) = (2, 1, 1, 0) / sqrt(6): its angle
    /// 2 acos(2 / sqrt 6) = 70.53 deg about (1, 1, 0) / sqrt 2, the rotation vector 1.2310 (0.70711, 0.70711, 0).
    /// The linear blend of the rotation vectors, (pi/4, pi/4, 0), is 1.1107 rad about the same axis: 6.9 degrees short.
    #[test]
    fn the_slerp_midpoint_of_two_quarter_turns_is_worked_out_by_hand() {
        use std::f64::consts::FRAC_PI_2;
        let mid = slerp([FRAC_PI_2, 0.0, 0.0], [0.0, FRAC_PI_2, 0.0], 0.5);
        let angle = 2.0 * (2.0 / 6f64.sqrt()).acos();
        let want = [angle / 2f64.sqrt(), angle / 2f64.sqrt(), 0.0];
        assert!((angle - 1.230_959_417).abs() < 1e-9);
        assert!((0..3).all(|i| (mid[i] - want[i]).abs() < 1e-12), "{mid:?} against {want:?}");
        // The ends, and the round trip through the quaternion.
        let end = slerp([FRAC_PI_2, 0.0, 0.0], [0.0, FRAC_PI_2, 0.0], 1.0);
        assert!((end[1] - FRAC_PI_2).abs() < 1e-12 && end[0].abs() < 1e-12);
        let r = [0.3, -1.2, 2.0];
        let back = from_quaternion(quaternion(r));
        assert!((0..3).all(|i| (back[i] - r[i]).abs() < 1e-12));
        // log inverts exp, near a half turn too.
        let half = [0.0, 0.0, std::f64::consts::PI - 1e-9];
        let again = log(&exp(half));
        assert!((0..3).all(|i| (again[i] - half[i]).abs() < 1e-7), "{again:?}");
    }
}
