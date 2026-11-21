//! The crane's motion with the rope's own mass, the hook block on its sling, the elastic drives and the bridge on two
//! carriages (effects.rope_mass, hook_block, joint_elasticity, bridge_skew; one of them on). The generalised coordinates are
//!
//! ```text
//! q = (x_c, y, l, P1x, P1y, P1z [, dx, dy, dz] [, psi])  bridge centre, trolley, free rope, the rope's lower end,
//!                                                         the sling's span P2 - P1, the bridge's yaw
//! ```
//!
//! and the equations of motion `M(q) q'' = f(q, q')` are assembled from the Jacobians of the bodies' points, solved densely
//! (at most 10 unknowns). The trolley's S = (x_c + psi (y - y_c), y, H), the sling's end P2 = P1 + d, the sling a
//! tension-only spring (`Sling`); a point of mass m with the acceleration a_p = J q'' + c (c = J' q', the velocity-product
//! terms) gives M += m J^T J and f += J^T (F - m c). The rope, whose mass mu l varies with the paid-out l, is derived
//! from its Lagrangian. Its material runs off the sheave at the drum's speed l' along u: the free rope's velocity is
//! linear from v_A = v_S + l' u at the top to v_P at the hook (a rope paid out straight down moves all of it at l'), so
//! K = mu l / 6 (|v_A|^2 + v_A . v_P + |v_P|^2), and the wound rope moves with the trolley and the drum's surface,
//! K_w = mu (capacity - l) (|v_S|^2 + l'^2) / 2. With J_A,i = J_S,i + delta_il u, a_A = J_A q'' + c_S + l' u',
//! u' = (I - u u^T)(v_P - v_S) / L, the equation of coordinate i is
//!
//! ```text
//! mu l / 3 [J_A,i . (a_A + a_P / 2) + J_P,i . (a_P + a_A / 2)] + mu l' / 3 [J_A,i . (v_A + v_P / 2) + J_P,i . (v_P + v_A / 2)]
//!     + mu l / 3 (delta_il u' - l' du/dq_i) . (v_A + v_P / 2) - delta_il K / l = Q_i
//! ```
//!
//! (d/dt J_A,i - dv_A/dq_i = delta_il u' - l' du/dq_i, du/dq_i = (I - u u^T)(J_P,i - J_S,i) / L; S and P are functions of
//! q, so their J' terms cancel against dK/dq_i), plus the wound rope's mu (capacity - l) (J_S,i . a_S + delta_il l'') -
//! mu l' J_S,i . v_S + delta_il mu (|v_S|^2 - l'^2) / 2 = Q_i. K is quadratic in q', so K + V is conserved.
//!
//! The drives are rigid (the rotor's inertia reflected, the gearbox's loss factor at the power it passes, iterated like
//! the plain model's) or elastic (`RotaryJoint::rotor`: the gearbox passes the torque between the rotor, a state, and
//! the wheel or drum). With the skew the bridge's two carriages A and B are at x_A = x_c - psi g / 2, x_B = x_c + psi g
//! / 2 (g the gauge), each with a drive and rolling resistance (the load on its wheels shared by the trolley's place), and
//! the drive force F_A, F_B acts as Q_x = F_A + F_B, Q_psi = (F_B - F_A) g / 2.
//!
//! The floor under the standing body (the load's point: the hook block's, or the payload's with the sling) is a
//! one-sided constraint a_z = 0 with the push N >= 0 and its Coulomb friction, solved with the coordinates' equations
//! (N = -a_z,free / (J_z M^-1 J_z^T), the friction iterated with N), while it stands still on it.

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

/// The sling at one moment (effects.hook_block): a rope of the main rope's EA and damping over its length l_s, from the
/// hook block P1 to the payload P2.
pub(crate) struct Sling {
    /// Its span d = P2 - P1 and d's rate; the unit vector along d.
    pub span: V3,
    pub rate: V3,
    pub u: V3,
    /// Its stretch e = |d| - l_s and e'; its tension t = EA e / l_s + c e' / l_s, never below 0, 0 while slack (e <= 0)
    /// or with nothing on it; the damper's share of t.
    pub stretch: f64,
    pub speed: f64,
    pub tension: f64,
    pub damper: f64,
}

/// q'' = q0 + sum_j lambda_j M^-1 d_j where each constraint (a, d, b) holds, a . q'' + b = 0: q'' and the lambdas.
fn constrained(mm: &[f64], q0: &[f64], cons: &[(Vec<f64>, Vec<f64>, f64)]) -> (Vec<f64>, Vec<f64>) {
    let (n, k) = (q0.len(), cons.len());
    if k == 0 {
        return (q0.to_vec(), vec![]);
    }
    let w: Vec<Vec<f64>> = cons.iter().map(|c| solve(mm, &c.1).unwrap_or_else(|| vec![f64::NAN; n])).collect();
    let a: Vec<f64> = (0..k * k).map(|x| dn(&cons[x / k].0, &w[x % k])).collect();
    let r: Vec<f64> = cons.iter().map(|c| -(dn(&c.0, q0) + c.2)).collect();
    let lam = solve(&a, &r).unwrap_or_else(|| vec![f64::NAN; k]);
    ((0..n).map(|i| q0[i] + (0..k).map(|j| lam[j] * w[j][i]).sum::<f64>()).collect(), lam)
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
        match self.sling_state(s) {
            Some(sl) if self.payload_mass() > 0.0 => (add(p, sl.span), add(v, sl.rate)),
            _ => (p, v),
        }
    }

    /// The sling's length with the payload hanging on it at rest, l_s (1 + m_p g / EA); l_s with none.
    pub(crate) fn sling_rest(&self) -> Option<f64> {
        self.sling.map(|ls| ls * (1.0 + self.payload_mass() * self.g / self.ea))
    }

    /// The sling at a state (`Sling`); `None` without effects.hook_block.
    pub(crate) fn sling_state(&self, s: &[f64]) -> Option<Sling> {
        let (ls, a0) = (self.sling?, self.layout.sling?);
        let (span, rate) = ([s[a0], s[a0 + 1], s[a0 + 2]], [s[a0 + 3], s[a0 + 4], s[a0 + 5]]);
        let length = norm(span);
        let u = if length > 0.0 { sc(span, 1.0 / length) } else { [0.0, 0.0, -1.0] };
        let (stretch, speed) = (length - ls, dot(u, rate));
        let (mut tension, mut damper) = (0.0, 0.0);
        if stretch > 0.0 && self.payload_mass() > 0.0 {
            let spring = self.ea * stretch / ls;
            tension = (spring + self.damping / ls * speed).max(0.0);
            damper = tension - spring;
        }
        Some(Sling { span, rate, u, stretch, speed, tension, damper })
    }

    /// The equations of motion at a state under the motor torques `torque`, with the rigid drives' loss factors `loss`;
    /// the rigid rope's tension `t_roll` on the wheels (the rolling's fixed point, `full_motion`).
    fn assemble(&self, s: &[f64], torque: &[f64], loss: &[f64], t_roll: f64) -> Sol {
        let lay = &self.layout;
        let (n, nd, g) = (lay.pos.len(), lay.drives, self.g);
        let mut mm = vec![0.0; n * n];
        let mut rhs = vec![0.0; n];
        let (psi, psid) = self.yaw(s);
        let (y, yd) = (s[1], s[RATES + 1]);
        let mut rope = self.rope(s);
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
        // P2 = P1 + d: the sling's span d is coordinates 6 to 8.
        let mut j2 = jp.clone();
        if self.sling.is_some() {
            for k in 0..3 {
                j2[6 + k][k] = 1.0;
            }
        }
        // Bridge (the girder at its middle, its yaw inertia), trolley, drum, hook block and load.
        mm[0] += self.bridge;
        add_body(&mut mm, &mut rhs, self.trolley, &js, cs, [0.0; 3]);
        mm[2 * n + 2] += self.masses[2];
        add_body(&mut mm, &mut rhs, m1, &jp, [0.0; 3], [0.0, 0.0, -m1 * g]);
        if let Some(sl) = self.sling_state(s) {
            if m2 > 0.0 {
                add_body(&mut mm, &mut rhs, m2, &j2, [0.0; 3], [0.0, 0.0, -m2 * g]);
                // The sling pulls P2 with -t u and P1 with t u: on d alone.
                for k in 0..3 {
                    rhs[6 + k] -= sl.tension * sl.u[k];
                }
            } else {
                // No load on the sling: its span carries no mass and stays as it is.
                for k in 6..9 {
                    mm[k * n + k] += 1.0;
                }
            }
        }
        if let Some(k) = self.skew {
            mm[(n - 1) * n + n - 1] += k.inertia;
            rhs[n - 1] += -k.stiffness * psi - k.damping * psid;
        }
        // The rope's mass: the free rope's kinetic energy and weight, the wound rope's (module head).
        if self.rope_mass > 0.0 {
            let (mu, mr, wound) = (self.rope_mass, self.rope_mass * l, self.rope_mass * (self.capacity - l));
            let across = |v: V3| sub(v, sc(u, dot(u, v)));
            // The rope's top end A: v_A = v_S + l' u, its column J_S + u at l, c_A = c_S + l' u'.
            let ud = sc(across(sub(vp, vs)), 1.0 / rope.length);
            let mut ja = js.clone();
            ja[2] = add(ja[2], u);
            let va = add(vs, sc(u, ld));
            let ca = add(cs, sc(ud, ld));
            let (top, low) = (add(va, sc(vp, 0.5)), add(vp, sc(va, 0.5)));
            // l' du/dq_i . top = l' (J_P,i - J_S,i) . (I - u u) top / L.
            let turn = sc(across(top), ld / rope.length);
            for i in 0..n {
                let mut bias = mr / 3.0 * (dot(ja[i], ca) + 0.5 * dot(jp[i], ca))
                    + mu * ld / 3.0 * (dot(ja[i], top) + dot(jp[i], low))
                    - mr / 3.0 * dot(sub(jp[i], js[i]), turn)
                    + wound * dot(js[i], cs)
                    - mu * ld * dot(js[i], vs);
                if i == 2 {
                    bias += mr / 3.0 * dot(ud, top);
                }
                rhs[i] -= bias;
                for j in 0..n {
                    mm[i * n + j] += mr / 3.0
                        * (dot(ja[i], ja[j]) + 0.5 * dot(ja[i], jp[j]) + 0.5 * dot(jp[i], ja[j]) + dot(jp[i], jp[j]))
                        + wound * dot(js[i], js[j]);
                }
            }
            rhs[2] += mu / 6.0 * (dot(va, va) + dot(va, vp) + dot(vp, vp)) - 0.5 * mu * dot(vs, vs);
            mm[2 * n + 2] += wound;
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
        let down = if self.elastic { t } else { t_roll } * -u[2] + 0.5 * self.rope_mass * g * l;
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
        // The constraints, each a . q'' + b = 0 with the force lambda d. The rigid rope (effects.rope_stretch off) while
        // taut: L = l, its tension t pulls with -t G, G = dL/dq - dl/dq = u . (J_P - J_S) - delta_l, and L'' - l'' = G q''
        // + |(I - u u) (v_P - v_S)|^2 / L - u . c_S. A bilateral constraint as the plain rigid rope's, but on the floor.
        let mut rope_row = None;
        if !self.elastic && rope.stretch > -1e-6 {
            let gr: Vec<f64> = (0..n).map(|i| dot(u, sub(jp[i], js[i])) - if i == 2 { 1.0 } else { 0.0 }).collect();
            let dv = sub(vp, vs);
            let b = (dot(dv, dv) - dot(u, dv).powi(2)) / rope.length - dot(u, cs);
            rope_row = Some((gr.clone(), gr.iter().map(|v| -v).collect::<Vec<f64>>(), b));
        }
        // The floor under the standing body (one-sided, N >= 0), its friction iterated with N; standing, the rigid rope
        // may go slack (t >= 0), as the plain model's.
        let (low, low_v) = self.lower(s);
        let vh = [low_v[0], low_v[1]];
        // Standing: on the floor and still. Falling onto it, the body runs on through the step, whose end puts it back
        // on the floor at rest (`after_step`, plastic, booked); a push on a body still moving down would do work.
        let on_floor = self.floor.is_some_and(|f| low[2] <= f + FLOOR_ROUNDING && low_v[2].abs() <= FLOOR_ROUNDING);
        let jac = if self.sling.is_some() && m2 > 0.0 { &j2 } else { &jp };
        let jz: Vec<f64> = jac.iter().map(|c| c[2]).collect();
        let (mut acc, mut tension, mut push, mut fric) = (nan(), 0.0, 0.0, [0.0; 2]);
        for _ in 0..6 {
            let rhs_f: Vec<f64> = (0..n).map(|i| rhs[i] + jac[i][0] * fric[0] + jac[i][1] * fric[1]).collect();
            let q0 = solve(&mm, &rhs_f).unwrap_or_else(nan);
            let mut set: Vec<(Vec<f64>, Vec<f64>, f64)> = rope_row.iter().cloned().collect();
            let mut taut = rope_row.is_some();
            let (mut a, mut lam) = constrained(&mm, &q0, &set);
            push = 0.0;
            if on_floor && dn(&jz, &a) < 0.0 {
                set.push((jz.clone(), jz.clone(), 0.0));
                (a, lam) = constrained(&mm, &q0, &set);
                if taut && lam[0] < 0.0 {
                    set.remove(0);
                    taut = false;
                    (a, lam) = constrained(&mm, &q0, &set);
                }
                push = lam.last().map_or(0.0, |n| n.max(0.0));
            }
            (acc, tension) = (a, if taut { lam[0] } else { 0.0 });
            fric = self.floor_friction_force(push, vh);
            if push == 0.0 {
                break;
            }
        }
        let (floor_force, floor_power) = (push, -(fric[0] * vh[0] + fric[1] * vh[1]));
        if !self.elastic {
            (rope.tension, rope.drum) = (tension, tension);
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

    /// `Crane::motion` of the composite model: the rigid drives' loss factors and the rigid rope's tension on the wheels
    /// each to its fixed point (as the plain model's).
    pub(crate) fn full_motion(&self, s: &[f64], torque: &[f64]) -> Motion {
        let nd = self.layout.drives;
        let mut t_roll = if self.elastic { 0.0 } else { self.load_mass() * self.g * -self.rope(s).u[2] };
        let loss: Vec<f64> = if self.elastic_drives {
            vec![1.0; nd]
        } else {
            sim_components::mech::settled_loss(vec![1.0; nd], |loss: &Vec<f64>| {
                let sol = self.assemble(s, torque, loss, t_roll);
                t_roll = sol.rope.tension;
                (0..nd)
                    .map(|d| {
                        let j = self.drive(d);
                        let a = dn(&sol.rows[d], &sol.acc);
                        j.gearbox.loss_factor(sol.drive[d] - j.reflected_inertia() * a, sol.travel[d].1)
                    })
                    .collect()
            })
        };
        let mut sol = self.assemble(s, torque, &loss, t_roll);
        for _ in 0..8 {
            if self.elastic || (sol.rope.tension - t_roll).abs() <= 1e-12 * (1.0 + t_roll.abs()) {
                break;
            }
            t_roll = sol.rope.tension;
            sol = self.assemble(s, torque, &loss, t_roll);
        }
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
        let va = add(vs, sc(self.rope(s).u, ld));
        kinetic += mu * l / 6.0 * (dot(va, va) + dot(va, vp) + dot(vp, vp))
            + 0.5 * mu * (self.capacity - l) * (dot(vs, vs) + ld * ld);
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
        if self.elastic && stretch > 0.0 {
            elastic += 0.5 * self.ea * stretch * stretch / l;
        }
        if let (Some(k), Some(i)) = (self.skew, self.layout.skew) {
            elastic += 0.5 * k.stiffness * s[i] * s[i];
        }
        if let (Some(sl), Some(ls)) = (self.sling_state(s), self.sling) {
            if sl.stretch > 0.0 && m2 > 0.0 {
                elastic += 0.5 * self.ea * sl.stretch * sl.stretch / ls;
            }
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
        if let Some(sl) = self.sling_state(s) {
            powers[8] += sl.damper * sl.speed;
        }
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
        if let (Some(a0), Some(rest)) = (self.layout.sling, self.sling_rest()) {
            state[a0 + 2] = -rest;
        }
        let pull_l = self.rest_pull(l, self.load_mass());
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
