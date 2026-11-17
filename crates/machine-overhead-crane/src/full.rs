//! The crane's motion with the rope's own mass, the hook block on its sling, the elastic drives and the bridge on two
//! carriages (effects.rope_mass, hook_block, joint_elasticity, bridge_skew; one of them on). The generalised coordinates are
//!
//! ```text
//! q = (x_c, y, l, P1x, P1y, P1z [, a, b] [, psi])      bridge centre, trolley, free rope, the rope's lower end,
//!                                                       the sling's two angles, the bridge's yaw
//! ```
//!
//! and the equations of motion `M(q) q'' = f(q, q')` are assembled from the Jacobians of the bodies' points, solved densely
//! (at most 9 unknowns). The trolley's S = (x_c + psi (y - y_c), y, H), the sling's end P2 = P1 + l_s e(a, b), e = (sin a
//! cos b, sin b, -cos a cos b); a point of mass m with the acceleration a_p = J q'' + c (c = J' q', the velocity-product
//! terms) gives M += m J^T J and f += J^T (F - m c). The rope, whose mass mu l varies with the paid-out l, is derived
//! from its Lagrangian: with K = mu l / 6 (|v_S|^2 + v_S . v_P + |v_P|^2), the equation of coordinate i is
//!
//! ```text
//! mu l / 3 [J_S,i . (a_S + a_P / 2) + J_P,i . (a_P + a_S / 2)] + mu l' / 3 [J_S,i . (v_S + v_P / 2) + J_P,i . (v_P + v_S / 2)]
//!     - delta_il K / l = Q_i          (the J' terms cancel against dK/dq_i, J = dp/dq)
//! ```
//!
//! plus the wound rope, K_w = mu (capacity - l) l'^2 / 2: mu (capacity - l) l'' - mu l'^2 / 2 = Q_l.
//!
//! The drives are rigid (the rotor's inertia reflected, the gearbox's loss factor at the power it passes, iterated like
//! the plain model's) or elastic (`RotaryJoint::rotor`: the gearbox passes the torque between the rotor, a state, and
//! the wheel or drum). With the skew the bridge's two carriages A and B are at x_A = x_c - psi g / 2, x_B = x_c + psi g
//! / 2 (g the gauge), each with a drive and rolling resistance (the load on its wheels shared by the trolley's place), and
//! the drive force F_A, F_B acts as Q_x = F_A + F_B, Q_psi = (F_B - F_A) g / 2.
//!
//! The floor under the standing body (the load's point: the hook block's, or the payload's with the sling) is a
//! one-sided constraint a_z = 0 with the push N >= 0 and its Coulomb friction, solved with the coordinates' equations
//! (N = -a_z,free / (J_z M^-1 J_z^T), the friction iterated with N).

use super::*;
use sim_components::chain::solve;
use sim_components::mech::GearTorque;

/// What `full_motion` solved besides `Motion`'s fields, per drive and per generalised coordinate.
pub(crate) struct Extra {
    /// q'' of the coordinates of `Layout::pos`.
    pub acc: Vec<f64>,
    /// Per drive: the rotor's acceleration (elastic), the brake deflection's rate, the axis's travel and speed, the
    /// rolling resistance, the gearbox's torques (elastic) and the force into and out of a rigid gearbox.
    pub rotor_acc: Vec<f64>,
    pub brake_rates: Vec<f64>,
    pub travel: Vec<(f64, f64)>,
    pub rolling: Vec<f64>,
    pub gear: Vec<GearTorque>,
    pub into: Vec<f64>,
    pub out: Vec<f64>,
    /// The yaw damper's power (W).
    pub yaw_power: f64,
}

struct Sol {
    acc: Vec<f64>,
    drive: Vec<f64>,
    rows: Vec<Vec<f64>>,
    travel: Vec<(f64, f64)>,
    rolling: Vec<f64>,
    rotor_acc: Vec<f64>,
    brake_rates: Vec<f64>,
    gear: Vec<GearTorque>,
    floor_force: f64,
    floor_power: f64,
    rope: Rope,
    yaw_power: f64,
}

fn dn(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b).map(|(x, y)| x * y).sum()
}

fn sc(a: V3, k: f64) -> V3 {
    [a[0] * k, a[1] * k, a[2] * k]
}

/// e(a, b) and its first and second derivatives: e, e_a, e_b, e_aa, e_ab, e_bb.
pub(crate) fn sling_dir(a: f64, b: f64) -> [V3; 6] {
    let (sa, ca, sb, cb) = (a.sin(), a.cos(), b.sin(), b.cos());
    [
        [sa * cb, sb, -ca * cb],
        [ca * cb, 0.0, sa * cb],
        [-sa * sb, cb, ca * sb],
        [-sa * cb, 0.0, ca * cb],
        [-ca * sb, 0.0, -sa * sb],
        [-sa * cb, -sb, ca * cb],
    ]
}

/// A point of mass `m` with the Jacobian columns `jac`, the velocity-product terms `c` and the force `f`.
fn add_body(mm: &mut [f64], rhs: &mut [f64], m: f64, jac: &[V3], c: V3, f: V3) {
    let n = rhs.len();
    for i in 0..n {
        rhs[i] += dot(jac[i], f) - m * dot(jac[i], c);
        for j in 0..n {
            mm[i * n + j] += m * dot(jac[i], jac[j]);
        }
    }
}

impl Crane {
    pub(crate) fn drive(&self, d: usize) -> &RotaryJoint {
        if d < 3 { &self.drives[d] } else { self.side_b.as_ref().expect("the second carriage") }
    }

    pub(crate) fn drive_mut(&mut self, d: usize) -> &mut RotaryJoint {
        if d < 3 { &mut self.drives[d] } else { self.side_b.as_mut().expect("the second carriage") }
    }

    /// Where drive `d`'s brake deflection sits in the state.
    pub(crate) fn brake_slot(&self, d: usize) -> usize {
        if d < 3 { BRAKES + d } else { self.layout.skew.expect("the second carriage") + 2 }
    }

    /// The travel and speed of drive `d`'s axis: the carriages (0: A, 3: B) at x_c -+ psi g / 2.
    pub(crate) fn travel(&self, s: &[f64], d: usize) -> (f64, f64) {
        let (psi, rate) = self.yaw(s);
        let half = self.skew.map_or(0.0, |k| 0.5 * k.gauge);
        match d {
            0 => (s[0] - half * psi, s[RATES] - half * rate),
            1 => (s[1], s[RATES + 1]),
            2 => (s[2], s[RATES + 2]),
            _ => (s[0] + half * psi, s[RATES] + half * rate),
        }
    }

    /// Drive `d`'s axis travel as a row over the coordinates.
    fn drive_row(&self, d: usize, n: usize) -> Vec<f64> {
        let mut r = vec![0.0; n];
        match d {
            0 | 3 => {
                r[0] = 1.0;
                if let Some(k) = self.skew {
                    r[n - 1] = if d == 0 { -0.5 * k.gauge } else { 0.5 * k.gauge };
                }
            }
            1 => r[1] = 1.0,
            _ => r[2] = 1.0,
        }
        r
    }

    /// The torque command of drive `d`'s motor: the bridge's command shared equally by its two carriages' motors.
    pub(crate) fn command(&self, torque: &[f64], d: usize) -> f64 {
        match d {
            0 | 3 if self.skew.is_some() => 0.5 * torque[0],
            0 | 3 => torque[0],
            _ => torque[d],
        }
    }

    /// The body the load is: P2 with the sling and a payload on it, else the hook block's P1 (place and speed).
    pub(crate) fn lower(&self, s: &[f64]) -> (V3, V3) {
        let p = [s[LOAD], s[LOAD + 1], s[LOAD + 2]];
        let v = [s[LOAD_SPEED], s[LOAD_SPEED + 1], s[LOAD_SPEED + 2]];
        match (self.sling, self.layout.sling) {
            (Some(ls), Some(a0)) if self.payload_mass() > 0.0 => {
                let [e, ea, eb, ..] = sling_dir(s[a0], s[a0 + 1]);
                (add(p, sc(e, ls)), add(v, sc(add(sc(ea, s[a0 + 2]), sc(eb, s[a0 + 3])), ls)))
            }
            _ => (p, v),
        }
    }

    /// The equations of motion at a state under the motor torques `torque`, with the rigid drives' loss factors `loss`.
    fn assemble(&self, s: &[f64], torque: &[f64], loss: &[f64]) -> Sol {
        let lay = &self.layout;
        let (n, nd, g) = (lay.pos.len(), lay.drives, self.g);
        let mut mm = vec![0.0; n * n];
        let mut rhs = vec![0.0; n];
        let (psi, psid) = self.yaw(s);
        let (y, yd) = (s[1], s[RATES + 1]);
        let rope = self.rope(s);
        let (u, t) = (rope.u, rope.tension);
        let (l, ld) = (s[2], s[RATES + 2]);
        let vs = self.sheave_speed(s);
        let vp = [s[LOAD_SPEED], s[LOAD_SPEED + 1], s[LOAD_SPEED + 2]];
        // The trolley's sheave S, the rope's lower end P1 and the sling's end P2 as Jacobian columns.
        let mut js = vec![[0.0; 3]; n];
        js[0] = [1.0, 0.0, 0.0];
        js[1] = [psi, 1.0, 0.0];
        if self.skew.is_some() {
            js[n - 1] = [y - self.y_centre, 0.0, 0.0];
        }
        let cs = [2.0 * psid * yd, 0.0, 0.0];
        let mut jp = vec![[0.0; 3]; n];
        for k in 0..3 {
            jp[3 + k][k] = 1.0;
        }
        let (m1, m2) = match self.sling {
            Some(_) => (self.hook, self.payload_mass()),
            None => (self.load_mass(), 0.0),
        };
        let mut j2 = jp.clone();
        let mut c2 = [0.0; 3];
        if let (Some(ls), Some(a0)) = (self.sling, lay.sling) {
            let (a, b, ad, bd) = (s[a0], s[a0 + 1], s[a0 + 2], s[a0 + 3]);
            let [_, ea, eb, eaa, eab, ebb] = sling_dir(a, b);
            j2[6] = sc(ea, ls);
            j2[7] = sc(eb, ls);
            c2 = sc(add(add(sc(eaa, ad * ad), sc(eab, 2.0 * ad * bd)), sc(ebb, bd * bd)), ls);
        }
        // Bridge (the girder at its middle, its yaw inertia), trolley, drum, hook block and load.
        mm[0] += self.bridge;
        add_body(&mut mm, &mut rhs, self.trolley, &js, cs, [0.0; 3]);
        mm[2 * n + 2] += self.masses[2];
        add_body(&mut mm, &mut rhs, m1, &jp, [0.0; 3], [0.0, 0.0, -m1 * g]);
        if self.sling.is_some() {
            if m2 > 0.0 {
                add_body(&mut mm, &mut rhs, m2, &j2, c2, [0.0, 0.0, -m2 * g]);
            } else {
                // No load on the sling: its angles carry no mass and stay where they are.
                mm[6 * n + 6] += 1.0;
                mm[7 * n + 7] += 1.0;
            }
        }
        if let Some(k) = self.skew {
            mm[(n - 1) * n + n - 1] += k.inertia;
            rhs[n - 1] += -k.stiffness * psi - k.damping * psid;
        }
        // The rope's mass: the free rope's kinetic energy and weight, the wound rope's.
        if self.rope_mass > 0.0 {
            let (mu, mr) = (self.rope_mass, self.rope_mass * l);
            for i in 0..n {
                let bias = mr / 3.0 * (dot(js[i], cs) + 0.5 * dot(jp[i], cs))
                    + mu * ld / 3.0 * (dot(js[i], add(vs, sc(vp, 0.5))) + dot(jp[i], add(vp, sc(vs, 0.5))));
                rhs[i] -= bias;
                for j in 0..n {
                    mm[i * n + j] += mr / 3.0
                        * (dot(js[i], js[j]) + 0.5 * dot(js[i], jp[j]) + 0.5 * dot(jp[i], js[j]) + dot(jp[i], jp[j]));
                }
            }
            rhs[2] += mu / 6.0 * (dot(vs, vs) + dot(vs, vp) + dot(vp, vp));
            mm[2 * n + 2] += mu * (self.capacity - l);
            rhs[2] += 0.5 * mu * ld * ld;
            // The weight V = mu g [l (H + z_P) / 2 + (capacity - l) H]: -dV/dl on the drum, -dV/dz_P on the load.
            rhs[2] += mu * g * (self.height - s[LOAD + 2]) / 2.0;
            rhs[5] -= mu * g * l / 2.0;
        }
        // The elastic rope: t along u on both ends, the drum's share dV/dl and the damper's.
        for i in 0..n {
            rhs[i] += t * (dot(js[i], u) - dot(jp[i], u));
        }
        rhs[2] += rope.drum;
        // The drives. The load on the bridge's and the trolley's wheels (the carriages share it by the trolley's place).
        let down = t * -u[2] + 0.5 * self.rope_mass * g * l;
        let phi = self.skew.map_or(0.5, |k| ((y - self.y_centre) / k.gauge + 0.5).clamp(0.0, 1.0));
        let wheels: Vec<f64> = (0..nd)
            .map(|d| match (d, self.skew) {
                (0, None) => (self.bridge + self.trolley) * g + down,
                (1, _) => self.trolley * g + down,
                (2, _) => 0.0,
                (d, _) => 0.5 * self.bridge * g + (self.trolley * g + down) * if d == 0 { 1.0 - phi } else { phi },
            })
            .collect();
        let (mut travel, mut rolling, mut drive) = (vec![], vec![0.0; nd], vec![0.0; nd]);
        let (mut rotor_acc, mut brake_rates, mut gear) = (vec![0.0; nd], vec![0.0; nd], vec![]);
        let mut rows = vec![];
        for d in 0..nd {
            let j = self.drive(d);
            let (pos, speed) = self.travel(s, d);
            travel.push((pos, speed));
            if d != 2 {
                rolling[d] = self.rolling * wheels[d].max(0.0) * (speed / self.rolling_smoothing).tanh();
            }
            let (cmd, brake) = (self.command(torque, d), s[self.brake_slot(d)]);
            let row = self.drive_row(d, n);
            let mut force = -rolling[d] + j.output_torque(pos, speed);
            if let Some(r0) = lay.rotor {
                let (gt, acc, rate) = j.rotor(cmd, s[r0 + d], s[r0 + nd + d], pos, speed, brake, self.brake_engaged);
                force += gt.output;
                (rotor_acc[d], brake_rates[d]) = (acc, rate);
                gear.push(gt);
            } else {
                let (dr, rate) = j.rigid_drive(cmd, j.ratio() * pos, j.ratio() * speed, brake, self.brake_engaged);
                (drive[d], brake_rates[d]) = (dr, rate);
                force += loss[d] * dr;
                let k = loss[d] * j.reflected_inertia();
                for a in 0..n {
                    for b in 0..n {
                        mm[a * n + b] += k * row[a] * row[b];
                    }
                }
                gear.push(GearTorque { output: 0.0, rotor: 0.0 });
            }
            for a in 0..n {
                rhs[a] += row[a] * force;
            }
            rows.push(row);
        }
        let nan = || vec![f64::NAN; n];
        let mut acc = solve(&mm, &rhs).unwrap_or_else(nan);
        // The floor under the standing body.
        let (mut floor_force, mut floor_power) = (0.0, 0.0);
        let (low, low_v) = self.lower(s);
        if self.floor.is_some_and(|f| low[2] <= f && low_v[2] <= 0.0) {
            let (jac, c) = if self.sling.is_some() && m2 > 0.0 { (&j2, c2) } else { (&jp, [0.0; 3]) };
            let jz: Vec<f64> = jac.iter().map(|c| c[2]).collect();
            if dn(&jz, &acc) + c[2] < 0.0 {
                let w = solve(&mm, &jz).unwrap_or_else(nan);
                let jw = dn(&jz, &w);
                let vh = [low_v[0], low_v[1]];
                let (mut push, mut fric) = (0.0, [0.0; 2]);
                for _ in 0..6 {
                    let rhs_f: Vec<f64> = (0..n).map(|i| rhs[i] + jac[i][0] * fric[0] + jac[i][1] * fric[1]).collect();
                    let q0 = solve(&mm, &rhs_f).unwrap_or_else(nan);
                    push = (-(dn(&jz, &q0) + c[2]) / jw).max(0.0);
                    acc = (0..n).map(|i| q0[i] + w[i] * push).collect();
                    fric = self.floor_friction_force(push, vh);
                }
                floor_force = push;
                floor_power = -(fric[0] * vh[0] + fric[1] * vh[1]);
            }
        }
        Sol {
            acc,
            drive,
            rows,
            travel,
            rolling,
            rotor_acc,
            brake_rates,
            gear,
            floor_force,
            floor_power,
            rope,
            yaw_power: self.skew.map_or(0.0, |k| k.damping * psid * psid),
        }
    }

    /// `Crane::motion` of the composite model.
    pub(crate) fn full_motion(&self, s: &[f64], torque: &[f64]) -> Motion {
        let nd = self.layout.drives;
        let loss: Vec<f64> = if self.elastic_drives {
            vec![1.0; nd]
        } else {
            sim_components::mech::settled_loss(vec![1.0; nd], |loss: &Vec<f64>| {
                let sol = self.assemble(s, torque, loss);
                (0..nd)
                    .map(|d| {
                        let j = self.drive(d);
                        let a = dn(&sol.rows[d], &sol.acc);
                        j.gearbox.loss_factor(sol.drive[d] - j.reflected_inertia() * a, sol.travel[d].1)
                    })
                    .collect()
            })
        };
        let sol = self.assemble(s, torque, &loss);
        let (mut into, mut out) = (vec![0.0; nd], vec![0.0; nd]);
        if !self.elastic_drives {
            for d in 0..nd {
                into[d] = sol.drive[d] - self.drive(d).reflected_inertia() * dn(&sol.rows[d], &sol.acc);
                out[d] = loss[d] * into[d];
            }
        }
        let mut rope = sol.rope;
        rope.drum += 0.5 * self.rope_mass * self.g * (self.height - s[LOAD + 2]);
        let (t, u) = (rope.tension, rope.u);
        Motion {
            acc: [sol.acc[0], sol.acc[1], sol.acc[2]],
            load_acc: [sol.acc[3], sol.acc[4], sol.acc[5]],
            into: [into[0], into[1], into[2]],
            out: [out[0], out[1], out[2]],
            brake_rates: [sol.brake_rates[0], sol.brake_rates[1], sol.brake_rates[2]],
            rope: [t * u[0], t * u[1], rope.drum],
            rolling: [sol.rolling[0] + sol.rolling.get(3).copied().unwrap_or(0.0), sol.rolling[1], 0.0],
            floor_force: sol.floor_force,
            floor_power: sol.floor_power,
            rope_state: rope,
            full: Some(Box::new(Extra {
                acc: sol.acc,
                rotor_acc: sol.rotor_acc,
                brake_rates: sol.brake_rates,
                travel: sol.travel,
                rolling: sol.rolling,
                gear: sol.gear,
                into,
                out,
                yaw_power: sol.yaw_power,
            })),
        }
    }

    /// Kinetic, potential and elastic energy of the composite model.
    pub(crate) fn full_energy(&self, s: &[f64]) -> [f64; 3] {
        let (nd, g) = (self.layout.drives, self.g);
        let (l, ld) = (s[2], s[RATES + 2]);
        let vs = self.sheave_speed(s);
        let vp = [s[LOAD_SPEED], s[LOAD_SPEED + 1], s[LOAD_SPEED + 2]];
        let (_, psid) = self.yaw(s);
        let (m1, m2) = match self.sling {
            Some(_) => (self.hook, self.payload_mass()),
            None => (self.load_mass(), 0.0),
        };
        let (low, low_v) = self.lower(s);
        let mut kinetic = 0.5 * self.bridge * s[RATES] * s[RATES]
            + 0.5 * self.trolley * dot(vs, vs)
            + 0.5 * self.masses[2] * ld * ld
            + 0.5 * m1 * dot(vp, vp);
        if m2 > 0.0 {
            kinetic += 0.5 * m2 * dot(low_v, low_v);
        }
        if let Some(k) = self.skew {
            kinetic += 0.5 * k.inertia * psid * psid;
        }
        let mu = self.rope_mass;
        kinetic += mu * l / 6.0 * (dot(vs, vs) + dot(vs, vp) + dot(vp, vp)) + 0.5 * mu * (self.capacity - l) * ld * ld;
        let mut potential = m1 * g * s[LOAD + 2] + m2 * g * low[2];
        potential += mu * g * (l * (self.height + s[LOAD + 2]) / 2.0 + (self.capacity - l) * self.height);
        let mut elastic = 0.0;
        for d in 0..nd {
            let j = self.drive(d);
            let (pos, speed) = self.travel(s, d);
            elastic += j.stop_energy(pos);
            match self.layout.rotor {
                Some(r0) => {
                    kinetic += 0.5 * j.rotor_inertia * s[r0 + nd + d].powi(2);
                    elastic += j.gearbox.energy(s[r0 + d], pos);
                }
                None => kinetic += 0.5 * j.reflected_inertia() * speed * speed,
            }
        }
        let stretch = self.rope(s).stretch;
        if stretch > 0.0 {
            elastic += 0.5 * self.ea * stretch * stretch / l;
        }
        if let (Some(k), Some(i)) = (self.skew, self.layout.skew) {
            elastic += 0.5 * k.stiffness * s[i] * s[i];
        }
        [kinetic, potential, elastic]
    }

    /// `Crane::losses` of the composite model: the drives' (summed over the bridge's two carriages), the rolling, the
    /// rope's damper, the stops, and with `power.other` the ripple, the pick's and the floor's friction; the yaw damper
    /// with the bridge's rolling.
    pub(crate) fn full_losses(&self, s: &[f64], torque: &[f64], motion: &Motion, ex: &Extra) -> ([f64; 11], [f64; 3]) {
        let (nd, mut powers, mut at_rotor) = (self.layout.drives, [0.0; 11], [0.0; 3]);
        for d in 0..nd {
            let j = self.drive(d);
            let (pos, speed) = ex.travel[d];
            let slot = if d == 3 { 0 } else { d };
            let (rotor, rotor_speed) = match self.layout.rotor {
                Some(r0) => (s[r0 + d], s[r0 + nd + d]),
                None => (j.ratio() * pos, j.ratio() * speed),
            };
            let [_, stop, shaft, drag] = j.losses(pos, speed, rotor_speed, s[self.brake_slot(d)], self.brake_engaged);
            let gear = if self.layout.rotor.is_some() {
                at_rotor[slot] += ex.gear[d].rotor - ex.gear[d].output / j.ratio();
                j.gearbox.loss_power(rotor, rotor_speed, pos, speed)
            } else {
                at_rotor[slot] += (ex.into[d] - ex.out[d]) / j.ratio();
                (ex.into[d] - ex.out[d]) * speed
            };
            powers[2 * slot] += shaft;
            powers[2 * slot + 1] += gear + drag;
            powers[9] += stop;
            let cmd = self.command(torque, d);
            powers[10] += (cmd - j.ripple.apply(cmd, rotor)) * rotor_speed;
            if d != 2 {
                powers[if d == 1 { 7 } else { 6 }] += ex.rolling[d] * speed;
            }
        }
        powers[6] += ex.yaw_power;
        powers[10] += self.pick_loss + motion.floor_power;
        let rope = &motion.rope_state;
        powers[8] = rope.damper * (rope.rate - s[RATES + 2]);
        (powers, at_rotor)
    }

    /// `Machine::equilibrium` of the composite model: the load at rest on its rope at the pose, the rope's mean tension
    /// m g + mu g l / 2, every drive holding what the rope, the stops and the gearbox's twist ask.
    pub(crate) fn full_equilibrium(&self, tool: &[f64], state: &mut Vec<f64>, torque: &mut [f64]) {
        let nd = self.layout.drives;
        let l = (self.height - tool[2]) / self.kinematic_stretch();
        let load = self.height - l * self.rest_stretch(l);
        state.clear();
        state.resize(self.layout.len, 0.0);
        state[..3].copy_from_slice(&[tool[0], tool[1], l]);
        state[LOAD..LOAD + 3].copy_from_slice(&[tool[0], tool[1], load]);
        let rope = self.rope(state);
        let pull_l = rope.drum + 0.5 * self.rope_mass * self.g * rope.length;
        let mut held = vec![0.0; nd];
        for d in 0..nd {
            let j = self.drive(d);
            let (pos, _) = self.travel(state, d);
            // At rest the gearbox holds the axis against the rope and the stops.
            let out = -(if d == 2 { pull_l } else { 0.0 }) - j.output_torque(pos, 0.0);
            let twist = if self.elastic_drives { j.gearbox.static_twist(out) } else { 0.0 };
            let rotor = j.ratio() * (pos + twist);
            if let Some(r0) = self.layout.rotor {
                state[r0 + d] = rotor;
            }
            held[d] = j.ripple.invert(out / j.ratio(), rotor);
        }
        torque[0] = held[0] * if self.skew.is_some() { 2.0 } else { 1.0 };
        torque[1] = held[1];
        torque[2] = held[2];
    }

    /// `Machine::derivatives` of the composite model.
    pub(crate) fn full_derivatives(&self, s: &[f64], torque: &[f64], out: &mut [f64]) {
        let motion = self.full_motion(s, torque);
        let ex = motion.full.as_ref().expect("composite motion");
        let (lay, nd) = (&self.layout, self.layout.drives);
        out.fill(0.0);
        for (i, (&p, &v)) in lay.pos.iter().zip(&lay.vel).enumerate() {
            out[p] = s[v];
            out[v] = ex.acc[i];
        }
        for d in 0..nd {
            out[self.brake_slot(d)] = ex.brake_rates[d];
            if let Some(r0) = lay.rotor {
                out[r0 + d] = s[r0 + nd + d];
                out[r0 + nd + d] = ex.rotor_acc[d];
            }
        }
    }
}
