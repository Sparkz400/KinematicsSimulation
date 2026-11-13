//! `ConfigMachine`: a belt-driven machine of any routing, built from a netlist (`Netlist`).
//!
//! The machine has n axes (tool coordinates q) and as many motors. Each motor turns one pulley through its gearbox.
//! Belt section s has the geometric length L_s = L0_s + sum_i c_si q_i (plus the teeth in mesh at each pulley it ends
//! at, see `Belt::tooth_length`), and pulley k feeds f_sk r_k of belt into it per radian (f = +1 or -1). So:
//!
//! * the belt pulls axis i with -sum_s c_si T_s, and turns pulley k with r_mat,k sum_s f_sk M_s (M the force on the belt
//!   material, `SectionForce::material`);
//! * in the rigid model the belt keeps its length: C q = F diag(r) a, so the pulley angles are a = K q with
//!   K = (F diag(r))^+ C, which must reproduce C exactly (else the routing is refused) and be invertible;
//! * bodies move with their axes, so the mass on axis i is the sum of the bodies that move with it; the axes are
//!   taken square to each other, so the bodies' mass matrix is diagonal. Gravity pulls axis i with m_i g sin(incline_i).
//!
//! Two of the registered belt machines are this with C = [[1], [-1]] and F = [[1], [-1]], and with
//! C = [[1, 1], [0, -2], [-1, 1]] and F = [[1, 0], [-1, 1], [0, -1]]; `machines/*.toml` hold them, and
//! crates/sim-machines/tests/machine_files.rs checks them against their crates. The equations are those crates',
//! written for any C and F: their module comments derive them.

use crate::netlist::Netlist;
use sim_components::mech::{Belt, Gearbox, HardStop, HoldingBrake, Lugre, Ripple, SectionForce, ShaftFriction, ToothMesh};
use sim_core::scene::{drive_unit, idler};
use sim_core::{AxisInfo, BeltSection, Config, ConfigError, ENERGY_PARTS, FRICTION_SIGNALS, FrictionContact, LOSS_PREFIX, Label, Machine, MotorInfo, Part, Scene, Shape, SignalInfo, TORQUE_PARTS, ThermalPart};

const G: f64 = 9.80665;

/// Effects the shipped machine crates have and machine files do not yet: switched on, a machine file is refused.
pub const UNSUPPORTED_EFFECTS: [&str; 8] = ["frame_compliance", "tool_flex", "idlers", "cable_chain", "process_force", "thermal_feedback", "belt_tooth_jump", "belt_span_vibration"];

/// A `&'static str` for an id or key read from a file, which `AxisInfo` and the other machine descriptions take.
/// Each distinct text is leaked once, so rebuilding a machine on every config change does not grow memory.
pub fn intern(text: &str) -> &'static str {
    use std::collections::HashSet;
    use std::sync::{Mutex, OnceLock};
    static TEXTS: OnceLock<Mutex<HashSet<&'static str>>> = OnceLock::new();
    let mut texts = TEXTS.get_or_init(Default::default).lock().unwrap_or_else(|e| e.into_inner());
    if let Some(known) = texts.get(text) {
        return known;
    }
    let leaked: &'static str = Box::leak(text.to_string().into_boxed_str());
    texts.insert(leaked);
    leaked
}

struct Section {
    id: &'static str,
    key: &'static str,
    /// Length at q = 0 with the teeth in mesh, and that without them; per axis and per motor.
    length: f64,
    bare: f64,
    c: Vec<f64>,
    f: Vec<f64>,
    /// Unstretched belt at q = 0 with the pulleys at angle 0.
    mat0: f64,
}

struct Body {
    mass: f64,
    payload: f64,
    axes: Vec<bool>,
}

struct Guide {
    /// Its id, for its loss (`power.<id>`) and its warmth, `axis.<axis id>` for its friction signals.
    loss_id: &'static str,
    contact: &'static str,
    axis: usize,
    friction: Lugre,
    carries: Vec<usize>,
    overhang: Option<crate::netlist::Overhang>,
}

pub struct ConfigMachine {
    elastic: bool,
    axes: Vec<AxisInfo>,
    motors: Vec<MotorInfo>,
    /// Per axis: gravity along it (pulling towards -) and across it.
    g_along: Vec<f64>,
    g_across: Vec<f64>,
    /// Per axis: its angle above the horizontal, rad, for the scene.
    incline: Vec<f64>,
    /// The closed belts: each the sections of one belt whose every pulley feeds it on one side and takes it out on
    /// another, so that the belt's material in them stays what it is. A belt that ends on a pulley (a drum) has
    /// its material wound on and off, and no such sum.
    loops: Vec<Vec<usize>>,
    sections: Vec<Section>,
    bodies: Vec<Body>,
    guides: Vec<Guide>,
    /// Pulley angle per metre of each axis (motors x axes), and its inverse.
    k: Vec<Vec<f64>>,
    k_inv: Vec<Vec<f64>>,
    /// Per motor: pulley radius, unstretched belt fed per radian, pulley inertia with its share of the belt.
    r: Vec<f64>,
    r_mat: Vec<f64>,
    j_pulley: Vec<f64>,
    /// Rotor and gearbox input inertia; the gearbox ratio, the same for all.
    j_rotor: f64,
    ratio: f64,
    gears: Vec<Gearbox>,
    shaft: ShaftFriction,
    ripple: Ripple,
    mesh: ToothMesh,
    brake: HoldingBrake,
    belt: Belt,
    stops: HardStop,
    payload_attached: bool,
    brake_engaged: bool,
}

// ----------------------------------------------------------------------------------------------------------------
// Small dense linear algebra: the systems have as many unknowns as the machine has axes or belt sections.

/// Solves a x = b by Gaussian elimination with partial pivoting; `None` for a singular a.
fn solve(mut a: Vec<Vec<f64>>, mut b: Vec<f64>) -> Option<Vec<f64>> {
    let n = b.len();
    for col in 0..n {
        let pivot = (col..n).max_by(|&i, &j| a[i][col].abs().total_cmp(&a[j][col].abs()))?;
        if a[pivot][col].abs() < 1e-300 {
            return None;
        }
        a.swap(col, pivot);
        b.swap(col, pivot);
        for row in col + 1..n {
            let factor = a[row][col] / a[col][col];
            if factor != 0.0 {
                for k in col..n {
                    a[row][k] -= factor * a[col][k];
                }
                b[row] -= factor * b[col];
            }
        }
    }
    let mut x = vec![0.0; n];
    for row in (0..n).rev() {
        let sum: f64 = (row + 1..n).map(|k| a[row][k] * x[k]).sum();
        x[row] = (b[row] - sum) / a[row][row];
    }
    Some(x)
}

/// The least-squares solution of a x = b (a: rows x cols), the one of least norm where it is not unique.
fn lstsq(a: &[Vec<f64>], b: &[f64]) -> Option<Vec<f64>> {
    let (rows, cols) = (a.len(), a.first().map_or(0, Vec::len));
    if rows >= cols {
        let ata = (0..cols).map(|i| (0..cols).map(|j| (0..rows).map(|r| a[r][i] * a[r][j]).sum()).collect()).collect();
        let atb = (0..cols).map(|i| (0..rows).map(|r| a[r][i] * b[r]).sum()).collect();
        solve(ata, atb)
    } else {
        let aat = (0..rows).map(|i| (0..rows).map(|j| (0..cols).map(|c| a[i][c] * a[j][c]).sum()).collect()).collect();
        let y = solve(aat, b.to_vec())?;
        Some((0..cols).map(|c| (0..rows).map(|r| a[r][c] * y[r]).sum()).collect())
    }
}

fn dot(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b).map(|(x, y)| x * y).sum()
}

/// Generalised velocity of a rigid solve: accelerations, guide forces and states' rates, per motor the torque at the
/// gearbox output and the torque that goes into it before its loss.
struct Rigid {
    acc: Vec<f64>,
    rates: Vec<f64>,
    out: Vec<f64>,
    into: Vec<f64>,
}

/// The closed belts among the sections (see `ConfigMachine::loops`): sections that share a pulley are one belt.
fn closed_belts(sections: &[Section], motors: usize) -> Vec<Vec<usize>> {
    let mut belt: Vec<usize> = (0..sections.len()).collect();
    let root = |belt: &[usize], mut s: usize| {
        while belt[s] != s {
            s = belt[s];
        }
        s
    };
    for k in 0..motors {
        let at: Vec<usize> = (0..sections.len()).filter(|&s| sections[s].f[k] != 0.0).collect();
        for pair in at.windows(2) {
            let (a, b) = (root(&belt, pair[0]), root(&belt, pair[1]));
            belt[a] = b;
        }
    }
    let mut out: Vec<Vec<usize>> = Vec::new();
    for s in 0..sections.len() {
        let r = root(&belt, s);
        match out.iter_mut().find(|l| root(&belt, l[0]) == r) {
            Some(l) => l.push(s),
            None => out.push(vec![s]),
        }
    }
    out.retain(|l| (0..motors).all(|k| l.iter().map(|&s| sections[s].f[k]).sum::<f64>() == 0.0));
    out
}

impl ConfigMachine {
    pub fn new(cfg: &Config, net: &Netlist) -> Result<Self, ConfigError> {
        for effect in UNSUPPORTED_EFFECTS {
            if cfg.bool(&format!("effects.{effect}")).unwrap_or(false) {
                return Err(ConfigError::Invalid(format!("effects.{effect} (not in machine files yet)")));
            }
        }
        let (n, m) = (net.axes.len(), net.motors.len());
        let gravity = if cfg.bool("effects.gravity")? { G } else { 0.0 };
        let belt = Belt::from_config(cfg)?;
        let j_rotor = cfg.positive("motor.rotor_inertia_kgm2")? + cfg.non_negative("gearbox.inertia_at_input_kgm2")?;
        let gear = Gearbox::from_config(cfg)?;
        let axes = net
            .axes
            .iter()
            .map(|a| -> Result<AxisInfo, ConfigError> {
                let limit = |name: &str| cfg.positive(&format!("axis.{}.{name}", a.id));
                Ok(AxisInfo {
                    id: intern(&a.id),
                    key: intern(&a.key),
                    unit: "m",
                    min: 0.0,
                    max: a.stroke,
                    max_vel: limit("max_velocity_m_s")?,
                    max_acc: limit("max_acceleration_m_s2")?,
                    max_jerk: limit("max_jerk_m_s3")?,
                })
            })
            .collect::<Result<Vec<_>, _>>()?;
        let incline: Vec<f64> = net.axes.iter().map(|a| a.incline_deg.to_radians()).collect();
        let teeth = belt.tooth_length;
        let sections: Vec<Section> = net
            .sections
            .iter()
            .map(|s| {
                let length = s.length + teeth * s.pulleys.iter().filter(|f| **f != 0.0).count() as f64;
                Section { id: intern(&format!("belt.{}", s.id)), key: intern(&s.key), length, bare: s.length, c: s.axes.clone(), f: s.pulleys.clone(), mat0: belt.rest_length(length) }
            })
            .collect();
        let r: Vec<f64> = net.motors.iter().map(|d| d.radius).collect();
        // The rigid kinematics: F diag(r) K = C.
        let fr: Vec<Vec<f64>> = sections.iter().map(|s| (0..m).map(|k| s.f[k] * r[k]).collect()).collect();
        let mut k = vec![vec![0.0; n]; m];
        for i in 0..n {
            let column: Vec<f64> = sections.iter().map(|s| s.c[i]).collect();
            let solved = lstsq(&fr, &column).ok_or_else(|| ConfigError::Invalid("machine.section (the pulleys do not set every section)".into()))?;
            for kk in 0..m {
                k[kk][i] = solved[kk];
            }
        }
        for (s, row) in sections.iter().zip(&fr) {
            for i in 0..n {
                let follows: f64 = (0..m).map(|kk| row[kk] * k[kk][i]).sum();
                if (follows - s.c[i]).abs() > 1e-9 * (1.0 + s.c[i].abs()) {
                    return Err(ConfigError::Invalid(format!("machine.section[{}].axes (the belt cannot keep its length)", &s.id["belt.".len()..])));
                }
            }
        }
        let mut k_inv = vec![vec![0.0; m]; n];
        for kk in 0..m {
            let unit: Vec<f64> = (0..m).map(|j| if j == kk { 1.0 } else { 0.0 }).collect();
            let column = solve(k.clone(), unit).ok_or_else(|| ConfigError::Invalid("machine.motor (the motors do not set every axis)".into()))?;
            for i in 0..n {
                k_inv[i][kk] = column[i];
            }
        }
        // The belt runs at pulley surface speed: its mass is shared out on the driven pulleys.
        let belt_mass = belt.mass_per_m * sections.iter().map(|s| s.bare).sum::<f64>() / m as f64;
        let payload_on = cfg.bool("effects.payload")?;
        let guides = net
            .guides
            .iter()
            .map(|g| -> Result<Guide, ConfigError> {
                Ok(Guide {
                    loss_id: intern(&g.id),
                    contact: intern(&format!("axis.{}", net.axes[g.axis].id)),
                    axis: g.axis,
                    friction: Lugre::from_config(cfg, &g.friction)?,
                    carries: g.carries.clone(),
                    overhang: g.overhang.as_ref().map(|o| crate::netlist::Overhang { ..*o }),
                })
            })
            .collect::<Result<Vec<_>, _>>()?;
        Ok(Self {
            elastic: cfg.bool("effects.belt_stretch")?,
            g_along: incline.iter().map(|a| gravity * a.sin()).collect(),
            // cos(90 deg) is 6e-17, not 0: a vertical axis has no weight across it.
            g_across: incline.iter().map(|a| if (a.cos()).abs() < 1e-12 { 0.0 } else { gravity * a.cos() }).collect(),
            loops: closed_belts(&sections, m),
            incline,
            bodies: net.bodies.iter().map(|b| Body { mass: b.mass, payload: if payload_on { b.payload } else { 0.0 }, axes: b.axes.clone() }).collect(),
            motors: net.motors.iter().map(|d| MotorInfo { id: intern(&d.id), key: intern(&d.key) }).collect(),
            r_mat: r.iter().map(|r| belt.rest_length(*r)).collect(),
            j_pulley: net.motors.iter().map(|d| d.inertia + belt_mass * d.radius * d.radius).collect(),
            r,
            k,
            k_inv,
            j_rotor,
            ratio: gear.ratio,
            gears: (0..m).map(|_| Gearbox::from_config(cfg)).collect::<Result<Vec<_>, _>>()?,
            shaft: ShaftFriction::from_config(cfg)?,
            ripple: Ripple::from_config(cfg)?,
            mesh: ToothMesh::from_config(cfg)?,
            brake: HoldingBrake::from_config(cfg, j_rotor)?,
            stops: HardStop::from_config(cfg)?,
            belt,
            sections,
            guides,
            axes,
            payload_attached: true,
            brake_engaged: false,
        })
    }

    fn n(&self) -> usize {
        self.axes.len()
    }
    fn m(&self) -> usize {
        self.motors.len()
    }
    // Elastic state: [q, a (pulleys), p (rotors)], their rates, a friction state per axis, the slack time, a brake
    // state per motor. Rigid state: [q, v, friction states, brakes].
    fn vel(&self) -> usize {
        if self.elastic { self.n() + 2 * self.m() } else { self.n() }
    }
    fn friction_at(&self) -> usize {
        2 * self.vel()
    }
    fn slack_at(&self) -> usize {
        self.friction_at() + self.n()
    }
    fn brake_at(&self) -> usize {
        self.slack_at() + usize::from(self.elastic)
    }

    fn body_mass(&self, b: &Body) -> f64 {
        b.mass + if self.payload_attached { b.payload } else { 0.0 }
    }
    /// The bodies' mass on each axis.
    fn mass(&self) -> Vec<f64> {
        (0..self.n()).map(|i| self.bodies.iter().filter(|b| b.axes[i]).map(|b| self.body_mass(b)).sum()).collect()
    }
    /// Gravity's force on each axis.
    fn gravity(&self) -> Vec<f64> {
        self.mass().iter().zip(&self.g_along).map(|(m, g)| -m * g).collect()
    }
    /// Pulleys and rotors seen on the axes, J_pulley + i^2 J_rotor per motor, through K.
    fn drive_mass(&self) -> Vec<Vec<f64>> {
        let n = self.n();
        let mut out = vec![vec![0.0; n]; n];
        for (kk, row) in self.k.iter().enumerate() {
            let j = self.j_pulley[kk] + self.ratio * self.ratio * self.j_rotor;
            for i in 0..n {
                for l in 0..n {
                    out[i][l] += j * row[i] * row[l];
                }
            }
        }
        out
    }
    /// The rigid model's mass matrix: bodies, pulleys and rotors.
    fn rigid_mass(&self) -> Vec<Vec<f64>> {
        let mut out = self.drive_mass();
        for (i, m) in self.mass().into_iter().enumerate() {
            out[i][i] += m;
        }
        out
    }
    /// Torques at the motors for forces on the axes: (J^T)^-1 with J = i K.
    fn to_motors(&self, forces: &[f64]) -> Vec<f64> {
        (0..self.m()).map(|kk| (0..self.n()).map(|i| self.k_inv[i][kk] * forces[i]).sum::<f64>() / self.ratio).collect()
    }

    fn positions<'s>(&self, s: &'s [f64]) -> (&'s [f64], &'s [f64]) {
        let (n, v) = (self.n(), self.vel());
        (&s[..n], &s[v..v + n])
    }
    fn pulley_angle(&self, kk: usize, q: &[f64]) -> f64 {
        dot(&self.k[kk], q)
    }

    fn sections(&self, s: &[f64]) -> Vec<SectionForce> {
        let (n, m, v) = (self.n(), self.m(), self.vel());
        self.sections
            .iter()
            .map(|sec| {
                let geo = sec.length + dot(&sec.c, &s[..n]);
                let mat = sec.mat0 + (0..m).map(|kk| sec.f[kk] * self.r_mat[kk] * s[n + kk]).sum::<f64>();
                let mat_rate: f64 = (0..m).map(|kk| sec.f[kk] * self.r_mat[kk] * s[v + n + kk]).sum();
                self.belt.section(geo, mat, dot(&sec.c, &s[v..v + n]), mat_rate)
            })
            .collect()
    }
    fn geo(&self, q: &[f64]) -> Vec<f64> {
        self.sections.iter().map(|s| s.length + dot(&s.c, q)).collect()
    }
    fn mat(&self, s: &[f64]) -> Vec<f64> {
        let n = self.n();
        self.sections.iter().map(|sec| sec.mat0 + (0..self.m()).map(|kk| sec.f[kk] * self.r_mat[kk] * s[n + kk]).sum::<f64>()).collect()
    }

    /// The acceleration of each axis that the motor torques would give a loss-free rigid machine: what an overhang's
    /// tilt on a guide is worked out with. The true acceleration would make friction depend on itself, and its absolute
    /// value would turn every belt vibration into extra friction.
    fn commanded_acc(&self, torque: &[f64]) -> Vec<f64> {
        let forces: Vec<f64> = (0..self.n()).map(|i| (0..self.m()).map(|kk| self.ratio * self.k[kk][i] * torque[kk]).sum()).collect();
        solve(self.rigid_mass(), forces).unwrap_or_else(|| vec![0.0; self.n()])
    }

    /// The load on a guide: the weight across its axis of what it carries, and with an overhang the tilt
    /// 2 m a lever / spacing (and with side_load the overhang's inertia force m a).
    fn guide_load(&self, g: &Guide, q: &[f64], acc: &[f64]) -> f64 {
        let weight: f64 = g.carries.iter().map(|&b| self.body_mass(&self.bodies[b]) * self.g_across[g.axis]).sum();
        let Some(o) = &g.overhang else { return weight };
        let side = self.body_mass(&self.bodies[o.body]) * acc[o.axis];
        let lever = o.lever - o.lever_axis.map_or(0.0, |a| q[a]);
        let tilt = 2.0 * side * lever / o.spacing;
        weight + tilt.abs() + if o.side_load { side.abs() } else { 0.0 }
    }
    /// Friction level factor per guide: its load against that of what it carries without the payload.
    fn load_scales(&self, q: &[f64], torque: &[f64]) -> Vec<f64> {
        let acc = if self.guides.iter().any(|g| g.overhang.is_some()) { self.commanded_acc(torque) } else { vec![0.0; self.n()] };
        self.guides
            .iter()
            .map(|g| {
                let nominal: f64 = g.carries.iter().map(|&b| self.bodies[b].mass * self.g_across[g.axis]).sum();
                g.friction.load_scale(self.guide_load(g, q, &acc), nominal)
            })
            .collect()
    }
    /// Guide friction force and friction state rate per axis (zero where an axis has no guide).
    fn friction(&self, q: &[f64], v: &[f64], states: &[f64], torque: &[f64]) -> (Vec<f64>, Vec<f64>) {
        let (mut force, mut rate) = (vec![0.0; self.n()], vec![0.0; self.n()]);
        for (g, scale) in self.guides.iter().zip(self.load_scales(q, torque)) {
            (force[g.axis], rate[g.axis]) = g.friction.eval(v[g.axis], states[g.axis], scale);
        }
        (force, rate)
    }
    fn stop_force(&self, i: usize, pos: f64, vel: f64) -> f64 {
        self.stops.force(pos, vel, self.axes[i].max)
    }

    fn elastic_derivatives(&self, s: &[f64], torque: &[f64], out: &mut [f64]) {
        let (n, m, v) = (self.n(), self.m(), self.vel());
        let sec = self.sections(s);
        let (fa, sa) = (self.friction_at(), self.brake_at());
        let (friction, rates) = self.friction(&s[..n], &s[v..v + n], &s[fa..fa + n], torque);
        out[..v].copy_from_slice(&s[v..2 * v]);
        let mass = self.mass();
        for i in 0..n {
            let belt: f64 = self.sections.iter().zip(&sec).map(|(x, f)| -x.c[i] * f.tension).sum();
            out[v + i] = (belt - friction[i] + self.stop_force(i, s[i], s[v + i])) / mass[i] - self.g_along[i];
            out[fa + i] = rates[i];
        }
        for kk in 0..m {
            let (theta, w_theta, phi, w_phi) = (s[n + kk], s[v + n + kk], s[n + m + kk], s[v + n + m + kk]);
            let gear = self.gears[kk].torque(phi, w_phi, theta, w_theta);
            let belt: f64 = self.r_mat[kk] * self.sections.iter().zip(&sec).map(|(x, f)| x.f[kk] * f.material).sum::<f64>();
            out[v + n + kk] = (gear.output + belt + self.r[kk] * self.mesh.force(self.r[kk] * theta)) / self.j_pulley[kk];
            let (brake, brake_rate) = self.brake.torque(self.brake_engaged, w_phi, s[sa + kk]);
            out[v + n + m + kk] = (self.ripple.apply(torque[kk], phi) - gear.rotor - self.shaft.torque(w_phi) - brake) / self.j_rotor;
            out[sa + kk] = brake_rate;
        }
        out[self.slack_at()] = if sec.iter().any(|f| f.tension <= 0.0) { 1.0 } else { 0.0 };
    }

    /// Rigid model. The gearbox loses torque only on what it transmits, which is the motor torque less what
    /// accelerates the rotor itself; that depends on the acceleration solved for, and three passes settle it.
    fn rigid_solve(&self, s: &[f64], torque: &[f64]) -> Rigid {
        let (n, m) = (self.n(), self.m());
        let (q, v) = (&s[..n], &s[n..2 * n]);
        let (fa, ba) = (self.friction_at(), self.brake_at());
        let (friction, rates) = self.friction(q, v, &s[fa..fa + n], torque);
        let (angle, speed): (Vec<f64>, Vec<f64>) = (0..m).map(|kk| (self.ratio * self.pulley_angle(kk, q), self.ratio * self.pulley_angle(kk, v))).unzip();
        // The torque each rotor passes on, before its own inertia and the gearbox; a brake holds the rotor.
        let drive: Vec<f64> = (0..m)
            .map(|kk| self.ripple.apply(torque[kk], angle[kk]) - self.shaft.torque(speed[kk]) - self.brake.torque(self.brake_engaged, speed[kk], s[ba + kk]).0)
            .collect();
        let gravity = self.gravity();
        let base: Vec<f64> = (0..n)
            .map(|i| {
                let mesh: f64 = (0..m).map(|kk| self.r[kk] * self.mesh.force(self.r[kk] * angle[kk] / self.ratio) * self.k[kk][i]).sum();
                mesh - friction[i] + self.stop_force(i, q[i], v[i]) + gravity[i]
            })
            .collect();
        let mut fixed = self.drive_mass();
        let mass = self.mass();
        for i in 0..n {
            for kk in 0..m {
                for l in 0..n {
                    fixed[i][l] -= self.ratio * self.ratio * self.j_rotor * self.k[kk][i] * self.k[kk][l];
                }
            }
            fixed[i][i] += mass[i];
        }
        let rotor = self.ratio * self.ratio * self.j_rotor;
        let (mut loss, mut solved) = (vec![1.0; m], Rigid { acc: vec![0.0; n], rates, out: vec![0.0; m], into: vec![0.0; m] });
        for _ in 0..3 {
            let mut a = fixed.clone();
            let mut b = base.clone();
            for kk in 0..m {
                for i in 0..n {
                    b[i] += loss[kk] * self.ratio * drive[kk] * self.k[kk][i];
                    for l in 0..n {
                        a[i][l] += loss[kk] * rotor * self.k[kk][i] * self.k[kk][l];
                    }
                }
            }
            let acc = solve(a, b).unwrap_or_else(|| vec![0.0; n]);
            for kk in 0..m {
                // What the gearbox output carries: what the rotor's own acceleration and the losses of this pass leave.
                let into = self.ratio * (drive[kk] - self.j_rotor * self.ratio * self.pulley_angle(kk, &acc));
                (solved.out[kk], solved.into[kk]) = (loss[kk] * into, into);
            }
            solved.acc = acc;
            loss = (0..m).map(|kk| self.gears[kk].loss_factor(solved.into[kk], self.pulley_angle(kk, v))).collect();
        }
        solved
    }

    /// Kinetic, potential and elastic energy (see `ENERGY_PARTS`).
    fn energy_parts(&self, s: &[f64]) -> [f64; 3] {
        let (n, m, v) = (self.n(), self.m(), self.vel());
        let q = &s[..n];
        let potential = -dot(&self.gravity(), q);
        let stops: f64 = (0..n).map(|i| self.stops.energy(q[i], self.axes[i].max)).sum();
        let vq = &s[v..v + n];
        if !self.elastic {
            let mass = self.rigid_mass();
            let kinetic = 0.5 * (0..n).map(|i| vq[i] * dot(&mass[i], vq)).sum::<f64>();
            return [kinetic, potential, stops];
        }
        let mass = self.mass();
        let turning: f64 = (0..m).map(|kk| 0.5 * self.j_pulley[kk] * s[v + n + kk].powi(2) + 0.5 * self.j_rotor * s[v + n + m + kk].powi(2)).sum();
        let kinetic = 0.5 * (0..n).map(|i| mass[i] * vq[i] * vq[i]).sum::<f64>() + turning;
        let (geo, mat) = (self.geo(q), self.mat(s));
        let belt: f64 = geo.iter().zip(&mat).map(|(g, l)| self.belt.energy(*g, *l)).sum();
        let gears: f64 = (0..m).map(|kk| self.gears[kk].energy(s[n + m + kk], s[n + kk])).sum();
        [kinetic, potential, stops + belt + gears]
    }

    /// The loss powers in the order of `effect_infos`, and per motor the gearbox loss as a torque at the rotor and the
    /// brake torque, which the torque breakdown takes too.
    fn losses(&self, s: &[f64], torque: &[f64]) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
        let (n, m, v) = (self.n(), self.m(), self.vel());
        let (q, vq) = self.positions(s);
        let fa = self.friction_at();
        let (friction, _) = self.friction(q, vq, &s[fa..fa + n], torque);
        let (mut angle, mut speed) = (vec![0.0; m], vec![0.0; m]);
        self.motor_state(s, &mut angle, &mut speed);
        let (mut gear_loss, mut gear_power, mut brake, mut mesh_in) = (vec![0.0; m], vec![0.0; m], vec![0.0; m], 0.0);
        let mut belt = 0.0;
        if self.elastic {
            for kk in 0..m {
                let (theta, w_theta, phi, w_phi) = (s[n + kk], s[v + n + kk], s[n + m + kk], s[v + n + m + kk]);
                let gear = self.gears[kk].torque(phi, w_phi, theta, w_theta);
                gear_loss[kk] = gear.rotor - gear.output / self.ratio;
                gear_power[kk] = self.gears[kk].loss_power(phi, w_phi, theta, w_theta);
                brake[kk] = self.brake.torque(self.brake_engaged, w_phi, s[self.brake_at() + kk]).0;
                mesh_in += self.r[kk] * self.mesh.force(self.r[kk] * theta) * w_theta;
            }
            let (geo, mat) = (self.geo(q), self.mat(s));
            for (i, sec) in self.sections.iter().enumerate() {
                let mat_rate: f64 = (0..m).map(|kk| sec.f[kk] * self.r_mat[kk] * s[v + n + kk]).sum();
                belt += self.belt.loss_power(geo[i], mat[i], dot(&sec.c, vq), mat_rate);
            }
        } else {
            let solved = self.rigid_solve(s, torque);
            for kk in 0..m {
                brake[kk] = self.brake.torque(self.brake_engaged, speed[kk], s[self.brake_at() + kk]).0;
                gear_loss[kk] = (solved.into[kk] - solved.out[kk]) / self.ratio;
                gear_power[kk] = gear_loss[kk] * speed[kk];
                mesh_in += self.mesh.force(self.r[kk] * angle[kk] / self.ratio) * self.r[kk] * self.pulley_angle(kk, vq);
            }
        }
        let mut powers: Vec<f64> = self.guides.iter().map(|g| friction[g.axis] * vq[g.axis]).collect();
        powers.extend((0..m).map(|kk| (self.shaft.torque(speed[kk]) + brake[kk]) * speed[kk]));
        powers.extend(&gear_power);
        let ripple: f64 = (0..m).map(|kk| (torque[kk] - self.ripple.apply(torque[kk], angle[kk])) * speed[kk]).sum();
        let stops: f64 = (0..n).map(|i| self.stops.loss_power(q[i], vq[i], self.axes[i].max)).sum();
        powers.extend([belt, stops, ripple - mesh_in]);
        (powers, gear_loss, brake)
    }

    /// The signals of the effect views, in the order of `effect_infos`. Torque parts per motor, through (J^T)^-1:
    /// inertia from the rigid mass matrix times the axes' accelerations (as a rigid machine would turn pulleys and
    /// rotors), gravity, and friction (guides, and at the motor its shaft friction, brake and gearbox loss); the rest
    /// is the motor torque less these, so belt and gearbox ringing are in it.
    fn view_signals(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        let (n, m) = (self.n(), self.m());
        let (q, vq) = self.positions(s);
        let fa = self.friction_at();
        let states = &s[fa..fa + n];
        let (friction, _) = self.friction(q, vq, states, torque);
        let (mut angle, mut speed) = (vec![0.0; m], vec![0.0; m]);
        self.motor_state(s, &mut angle, &mut speed);
        let (powers, gear_loss, brake) = self.losses(s, torque);
        let acc = if self.elastic {
            let mut d = vec![0.0; self.state_len()];
            self.elastic_derivatives(s, torque, &mut d);
            d[self.vel()..self.vel() + n].to_vec()
        } else {
            self.rigid_solve(s, torque).acc
        };
        let mass = self.rigid_mass();
        let inertia: Vec<f64> = mass.iter().map(|row| dot(row, &acc)).collect();
        let gravity: Vec<f64> = self.gravity().iter().map(|g| -g).collect();
        let [inertia, gravity, friction_at] = [inertia, gravity, friction.clone()].map(|f| self.to_motors(&f));
        for kk in 0..m {
            let parts = [inertia[kk], gravity[kk], self.shaft.torque(speed[kk]) + brake[kk] + gear_loss[kk] + friction_at[kk]];
            out.extend(parts);
            out.push(torque[kk] - parts.iter().sum::<f64>());
        }
        out.extend(powers);
        out.extend(self.energy_parts(s));
        for (g, scale) in self.guides.iter().zip(self.load_scales(q, torque)) {
            out.push(vq[g.axis]);
            out.extend(g.friction.split(vq[g.axis], states[g.axis], scale));
            out.push(scale);
        }
        let geo = self.geo(q);
        let mat = if self.elastic { self.mat(s) } else { geo.iter().map(|g| self.belt.rest_length(*g)).collect() };
        out.extend(geo.iter().zip(&mat).map(|(g, l)| g - l));
    }

    fn effect_infos(&self, out: &mut Vec<SignalInfo>) {
        for motor in &self.motors {
            out.extend(TORQUE_PARTS.map(|part| SignalInfo::new(format!("motor.{}.torque_{part}", motor.id), motor.key, &format!("quantity-torque-{part}"), "N m")));
        }
        for g in &self.guides {
            out.push(SignalInfo::new(format!("{LOSS_PREFIX}{}", g.loss_id), self.axes[g.axis].key, "quantity-friction-power", "W"));
        }
        for motor in &self.motors {
            out.push(SignalInfo::new(format!("{LOSS_PREFIX}shaft_{}", motor.id), motor.key, "quantity-shaft-friction-power", "W"));
        }
        let gear = if self.gears[0].is_coupling() { "quantity-coupling-loss-power" } else { "quantity-gearbox-loss-power" };
        for motor in &self.motors {
            out.push(SignalInfo::new(format!("{LOSS_PREFIX}gearbox_{}", motor.id), motor.key, gear, "W"));
        }
        out.push(SignalInfo::new(format!("{LOSS_PREFIX}belt"), "group-belt", "quantity-damping-power", "W"));
        out.push(SignalInfo::new(format!("{LOSS_PREFIX}hard_stops"), "group-hard_stop", "quantity-damping-power", "W"));
        out.push(SignalInfo::new(format!("{LOSS_PREFIX}other"), "loss-other", "quantity-power-given-off", "W"));
        out.extend(ENERGY_PARTS.map(|part| SignalInfo::new(format!("energy.{part}"), "group-energy", &format!("quantity-energy-{part}"), "J")));
        for g in &self.guides {
            for part in FRICTION_SIGNALS {
                let unit = match part {
                    "speed" => "m/s",
                    "load_scale" => "",
                    _ => "N",
                };
                out.push(SignalInfo::new(format!("{}.friction_{part}", g.contact), self.axes[g.axis].key, &format!("quantity-friction-{}", part.replace('_', "-")), unit));
            }
        }
        out.extend(self.sections.iter().map(|s| SignalInfo::new(format!("{}.stretch", s.id), s.key, "quantity-section-stretch", "m")));
    }

    /// Where the scene draws a pose: each axis runs from the origin along its incline in the x-z plane.
    fn scene_at(&self, q: &[f64], moves: &[bool]) -> [f64; 3] {
        let mut p = [0.0, -0.03, 0.06];
        for i in (0..self.n()).filter(|i| moves[*i]) {
            // Up by the incline, and across: the first axis along x, the others along y.
            let (up, across) = self.incline[i].sin_cos();
            p[if i == 0 { 0 } else { 1 }] += q[i] * across;
            p[2] += q[i] * up;
        }
        p
    }
}

impl Machine for ConfigMachine {
    fn axes(&self) -> &[AxisInfo] {
        &self.axes
    }
    fn motors(&self) -> &[MotorInfo] {
        &self.motors
    }

    fn inverse(&self, tool: &[f64], motors: &mut [f64]) {
        for (kk, out) in motors.iter_mut().enumerate() {
            *out = self.ratio * self.pulley_angle(kk, tool);
        }
    }
    fn forward(&self, motors: &[f64], tool: &mut [f64]) {
        for (i, out) in tool.iter_mut().enumerate() {
            *out = dot(&self.k_inv[i], motors) / self.ratio;
        }
    }

    fn state_len(&self) -> usize {
        self.brake_at() + self.m()
    }

    fn equilibrium(&self, tool: &[f64], state: &mut Vec<f64>, torque: &mut [f64]) {
        let (n, m) = (self.n(), self.m());
        *state = vec![0.0; self.state_len()];
        state[..n].copy_from_slice(tool);
        let gravity = self.gravity();
        if !self.elastic {
            // K^T (pulley torque + r mesh force) = -gravity.
            let kt: Vec<Vec<f64>> = (0..n).map(|i| (0..m).map(|kk| self.k[kk][i]).collect()).collect();
            let at_pulley = lstsq(&kt, &gravity.iter().map(|g| -g).collect::<Vec<_>>()).unwrap_or_else(|| vec![0.0; m]);
            for kk in 0..m {
                let angle = self.ratio * self.pulley_angle(kk, tool);
                let shaft = (at_pulley[kk] - self.r[kk] * self.mesh.force(self.r[kk] * angle / self.ratio)) / self.ratio;
                torque[kk] = self.ripple.invert(shaft, angle);
            }
            return;
        }
        // The tensions at rest: they hold every axis against gravity, sum_s c_si T_s = gravity_i, and each closed belt
        // keeps its material, sum over its sections of L_s / (1 + T_s / EA) = what it had at q = 0. Newton from the
        // pretension; the first equations are linear, the others nearly so.
        let geo = self.geo(tool);
        let (ea, slack) = (self.belt.ea, self.belt.slack);
        let taut = |t: f64| if slack { t.max(0.0) } else { t };
        let mut tension = vec![self.belt.pretension; self.sections.len()];
        for _ in 0..50 {
            let mut jac: Vec<Vec<f64>> = (0..n).map(|i| self.sections.iter().map(|s| s.c[i]).collect()).collect();
            let mut res: Vec<f64> = (0..n).map(|i| gravity[i] - dot(&jac[i], &tension)).collect();
            for belt in &self.loops {
                let mut row = vec![0.0; self.sections.len()];
                let mut held = 0.0;
                for &s in belt {
                    let stretch = 1.0 + taut(tension[s]) / ea;
                    held += geo[s] / stretch - self.sections[s].mat0;
                    row[s] = if slack && tension[s] < 0.0 { 0.0 } else { -geo[s] / (ea * stretch * stretch) };
                }
                jac.push(row);
                res.push(-held);
            }
            let step = lstsq(&jac, &res).unwrap_or_else(|| vec![0.0; self.sections.len()]);
            tension.iter_mut().zip(&step).for_each(|(t, d)| *t += d);
            if step.iter().all(|d| d.abs() < 1e-12 * (1.0 + self.belt.pretension)) {
                break;
            }
        }
        let mat: Vec<f64> = geo.iter().zip(&tension).map(|(g, t)| g / (1.0 + taut(*t) / ea)).collect();
        let fed: Vec<Vec<f64>> = self.sections.iter().map(|s| (0..m).map(|kk| s.f[kk] * self.r_mat[kk]).collect()).collect();
        let change: Vec<f64> = self.sections.iter().zip(&mat).map(|(s, l)| l - s.mat0).collect();
        let angles = lstsq(&fed, &change).unwrap_or_else(|| vec![0.0; m]);
        let material: Vec<f64> = geo.iter().zip(&mat).map(|(g, l)| self.belt.section(*g, *l, 0.0, 0.0).material).collect();
        for kk in 0..m {
            let theta = angles[kk];
            let belt: f64 = self.r_mat[kk] * self.sections.iter().zip(&material).map(|(s, f)| s.f[kk] * f).sum::<f64>();
            let gear = -belt - self.r[kk] * self.mesh.force(self.r[kk] * theta);
            let phi = self.ratio * (theta + self.gears[kk].static_twist(gear));
            state[n + kk] = theta;
            state[n + m + kk] = phi;
            torque[kk] = self.ripple.invert(gear / self.ratio, phi);
        }
    }

    fn derivatives(&self, s: &[f64], torque: &[f64], out: &mut [f64]) {
        if self.elastic {
            return self.elastic_derivatives(s, torque, out);
        }
        let (n, m) = (self.n(), self.m());
        let solved = self.rigid_solve(s, torque);
        out[..n].copy_from_slice(&s[n..2 * n]);
        out[n..2 * n].copy_from_slice(&solved.acc);
        out[2 * n..3 * n].copy_from_slice(&solved.rates);
        for kk in 0..m {
            let speed = self.ratio * self.pulley_angle(kk, &s[n..2 * n]);
            out[3 * n + kk] = self.brake.torque(self.brake_engaged, speed, s[3 * n + kk]).1;
        }
    }

    fn tool_state(&self, s: &[f64], pos: &mut [f64], vel: &mut [f64]) {
        let (q, v) = self.positions(s);
        pos.copy_from_slice(q);
        vel.copy_from_slice(v);
    }
    fn set_tool_velocity(&self, s: &mut [f64], vel: &[f64]) {
        let (n, m, v) = (self.n(), self.m(), self.vel());
        s[v..v + n].copy_from_slice(vel);
        if self.elastic {
            for kk in 0..m {
                let w = self.pulley_angle(kk, vel);
                (s[v + n + kk], s[v + n + m + kk]) = (w, self.ratio * w);
            }
        }
    }
    fn motor_state(&self, s: &[f64], pos: &mut [f64], vel: &mut [f64]) {
        let (n, m, v) = (self.n(), self.m(), self.vel());
        if self.elastic {
            pos.copy_from_slice(&s[n + m..n + 2 * m]);
            vel.copy_from_slice(&s[v + n + m..v + n + 2 * m]);
        } else {
            self.inverse(&s[..n], pos);
            self.inverse(&s[n..2 * n], vel);
        }
    }

    fn feedforward_torque(&self, _pos: &[f64], acc: &[f64], out: &mut [f64]) {
        let mass = self.rigid_mass();
        let gravity = self.gravity();
        let forces: Vec<f64> = (0..self.n()).map(|i| dot(&mass[i], acc) - gravity[i]).collect();
        out.copy_from_slice(&self.to_motors(&forces));
    }

    /// The belt pulls the bodies with the force F their masses take (without pulleys and rotors), -C^T T = F, and each
    /// closed belt keeps its length: sum_s (T_s - P) L_s = 0 over its geometric section lengths (the elastic law to first
    /// order in T / EA).
    fn rigid_tensions(&self, pos: &[f64], acc: &[f64], out: &mut Vec<f64>) {
        let (n, mass, gravity) = (self.n(), self.mass(), self.gravity());
        let geo = self.geo(pos);
        let mut a: Vec<Vec<f64>> = (0..n).map(|i| self.sections.iter().map(|s| s.c[i]).collect()).collect();
        let mut b: Vec<f64> = (0..n).map(|i| -(mass[i] * acc[i] - gravity[i])).collect();
        // Each closed belt keeps its length.
        for belt in &self.loops {
            a.push((0..self.sections.len()).map(|s| if belt.contains(&s) { geo[s] } else { 0.0 }).collect());
            b.push(0.0);
        }
        let d = lstsq(&a, &b).unwrap_or_else(|| vec![0.0; self.sections.len()]);
        out.extend(d.iter().map(|d| self.belt.pretension + d));
    }

    fn energy(&self, s: &[f64]) -> f64 {
        self.energy_parts(s).iter().sum()
    }

    fn set_payload_attached(&mut self, attached: bool) {
        self.payload_attached = attached;
    }
    fn set_brake_engaged(&mut self, engaged: bool) {
        self.brake_engaged = engaged;
    }

    fn signal_infos(&self) -> Vec<SignalInfo> {
        let mut out: Vec<SignalInfo> = self.sections.iter().map(|s| SignalInfo::new(format!("{}.tension", s.id), s.key, "quantity-tension", "N")).collect();
        for axis in &self.axes {
            out.push(SignalInfo::new(format!("axis.{}.belt_stretch", axis.id), axis.key, "quantity-belt-stretch", "m"));
            out.push(SignalInfo::new(format!("axis.{}.friction", axis.id), axis.key, "quantity-friction-force", "N"));
        }
        out.push(SignalInfo::new("belt.slack_time", "group-belt", "quantity-slack-time", "s"));
        for motor in &self.motors {
            out.push(SignalInfo::new(format!("pulley.{}.radial_load", motor.id), motor.key, "quantity-radial-load", "N"));
        }
        for g in &self.guides {
            out.push(SignalInfo::new(format!("{}.guide_load", g.contact), self.axes[g.axis].key, "quantity-guide-load", "N"));
        }
        for motor in &self.motors {
            out.push(SignalInfo::new(format!("gearbox.{}.output_torque", motor.id), motor.key, "quantity-gearbox-torque", "N m"));
        }
        self.effect_infos(&mut out);
        out
    }

    fn friction_contacts(&self) -> Vec<FrictionContact> {
        self.guides.iter().map(|g| FrictionContact { id: g.contact, owner_key: self.axes[g.axis].key }).collect()
    }
    fn friction_curve(&self, i: usize, speed: f64, load_scale: f64) -> [f64; 3] {
        self.guides.get(i).map_or([0.0; 3], |g| g.friction.steady(speed, load_scale))
    }

    fn belt_sections(&self) -> Vec<BeltSection> {
        self.sections.iter().map(|s| BeltSection { id: s.id, owner_key: s.key, pretension: self.belt.pretension }).collect()
    }

    /// The gearboxes (not on a direct drive, see `Gearbox::is_coupling`), then the guides, each shown by the first
    /// body it carries (or its overhang).
    fn thermal_parts(&self) -> Vec<ThermalPart> {
        let mut parts = Vec::new();
        if !self.gears[0].is_coupling() {
            for (kk, motor) in self.motors.iter().enumerate() {
                parts.push(ThermalPart { id: intern(&format!("gearbox_{}", motor.id)), kind: "gearbox", owner_key: motor.key, scene_part: "gearbox", scene_index: kk });
            }
        }
        for g in &self.guides {
            let body = g.carries.first().copied().or(g.overhang.as_ref().map(|o| o.body)).unwrap_or(0);
            parts.push(ThermalPart { id: g.loss_id, kind: "guide", owner_key: self.axes[g.axis].key, scene_part: "carriage", scene_index: body });
        }
        parts
    }

    /// The rigid model has no belt stretch: it reports pretension, zero stretch and no slack. A section is taken to
    /// wrap the pulleys it ends at by half a turn, so the radial load on a pulley's shaft is the sum of the tensions
    /// of the sections that end there.
    fn base_signals(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        let (n, m) = (self.n(), self.m());
        let (q, vq) = self.positions(s);
        let fa = self.friction_at();
        let (friction, _) = self.friction(q, vq, &s[fa..fa + n], torque);
        let (tensions, stretch, slack, outputs) = if self.elastic {
            let tensions: Vec<f64> = self.sections(s).iter().map(|f| f.tension).collect();
            let stretch: Vec<f64> = (0..n).map(|i| q[i] - dot(&self.k_inv[i], &s[n..n + m])).collect();
            let outputs = (0..m).map(|kk| self.gears[kk].torque(s[n + m + kk], s[self.vel() + n + m + kk], s[n + kk], s[self.vel() + n + kk]).output).collect();
            (tensions, stretch, s[self.slack_at()], outputs)
        } else {
            (vec![self.belt.pretension; self.sections.len()], vec![0.0; n], 0.0, self.rigid_solve(s, torque).out)
        };
        out.extend(&tensions);
        for i in 0..n {
            out.extend([stretch[i], friction[i]]);
        }
        out.push(slack);
        for kk in 0..m {
            out.push(self.sections.iter().zip(&tensions).filter(|(sec, _)| sec.f[kk] != 0.0).map(|(_, t)| t).sum());
        }
        let acc = if self.guides.iter().any(|g| g.overhang.is_some()) { self.commanded_acc(torque) } else { vec![0.0; n] };
        out.extend(self.guides.iter().map(|g| self.guide_load(g, q, &acc)));
        out.extend(outputs);
    }
    fn effect_signals(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        self.view_signals(s, torque, out);
    }
    fn loss_powers(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        out.extend(self.losses(s, torque).0);
    }

    /// shortcut: a plain scene (a beam, a box per body, a drive unit per motor, the belt not drawn); a machine file has
    /// no geometry beyond its routing. Upgrade when node positions (`pos`) or a CAD link give the parts a place.
    fn scene(&self, s: &[f64], out: &mut Scene) {
        use std::f64::consts::TAU;
        let (n, m) = (self.n(), self.m());
        let (q, _) = self.positions(s);
        let (mut rotor, mut rotor_speed) = (vec![0.0; m], vec![0.0; m]);
        self.motor_state(s, &mut rotor, &mut rotor_speed);
        out.parts.clear();
        out.labels.clear();
        let end = self.axes[0].max;
        out.parts.push(Part::new("beam", Shape::Box { size: [end + 0.2, 0.07, 0.09] }, [end / 2.0, 0.045, 0.0], [0.62, 0.64, 0.68]));
        for (b, body) in self.bodies.iter().enumerate() {
            let at = self.scene_at(q, &body.axes);
            let shade = 0.2 + 0.1 * b as f32;
            out.parts.push(Part::new("carriage", Shape::Box { size: [0.16, 0.05, 0.03] }, at, [shade, shade + 0.02, shade + 0.06]));
        }
        for kk in 0..m {
            let x = if kk % 2 == 0 { -0.1 } else { end + 0.1 };
            let pulley = if self.elastic { s[n + kk] } else { rotor[kk] / self.ratio };
            drive_unit(&mut out.parts, [x, 0.0], -0.05, self.r[kk], -pulley, -rotor[kk], [0.9, 0.45, 0.1]);
            out.labels.push(Label { pos: [x, -0.05, 0.11], owner_key: self.motors[kk].key, value: rotor_speed[kk] * 60.0 / TAU, unit_key: "unit-rpm", decimals: 0 });
        }
        if m == 1 {
            idler(&mut out.parts, [end + 0.1, 0.0], -0.05, self.r[0], 0.0);
        }
        for (i, axis) in self.axes.iter().enumerate() {
            let moves: Vec<bool> = (0..n).map(|l| l <= i).collect();
            let at = self.scene_at(q, &moves);
            out.labels.push(Label { pos: [at[0], at[1], at[2] + 0.16 + 0.06 * i as f64], owner_key: axis.key, value: q[i], unit_key: "unit-m", decimals: 3 });
        }
    }

    fn scene_point(&self, tool_pose: &[f64]) -> Option<[f64; 3]> {
        Some(self.scene_at(tool_pose, &vec![true; self.n()]))
    }
}
