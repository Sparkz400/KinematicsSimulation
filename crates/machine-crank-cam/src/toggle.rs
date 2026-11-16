//! A toggle press (knuckle-joint press): two links in line between the frame's top pivot O and the ram, the upper one of
//! length a, the lower one of length b, their joint the knee K; a crank of radius r, turned by the motor through its
//! gearbox, pushes the knee across through a connecting rod of length c. Where the links stand stretched (the dead
//! centre) the ram is at its bottom, and near there it moves very little for the knee's travel: the force at the ram is
//! the crank's torque over a ratio dx/dq that falls to zero at the dead centre, the force amplification the machine is
//! built for.
//!
//! Why a crank and not a ball screw: the crank-driven knuckle joint is the classic form and keeps the crate's turning
//! drive, cam and axis in rad; its ratio near the dead centre falls with both the links' and the four-bar's geometry,
//! which `crank.angle_at_dead_centre_deg` sets (90: the crank across the rod there, the ratio falls linearly; towards
//! 180 the crank comes into line with the rod and the dwell grows flat). The screw-driven toggle (the knee on the nut)
//! is the same links with the knee's travel a linear joint; not built.
//!
//! Frame (scene's x across, z up), O at the origin, the ram on a vertical guide through O. The knee K = (a sin(phi),
//! -a cos(phi)), phi the upper link's angle from straight down; the ram x = a cos(phi) + sqrt(b^2 - a^2 sin^2(phi)) below
//! O (the slider-crank's formula, `crank::slider_motion`, the upper link its crank), x = a + b at the dead centre phi = 0.
//! The joint q is the crank angle from the dead centre: the crank pin P = C + r (cos(t), sin(t)), t = t0 + q,
//! t0 = `crank.angle_at_dead_centre_deg`; the crank's centre C lies where, at q = 0, the rod lies level from P to the
//! stretched knee (0, -a): C = (c, -a) - r (cos(t0), sin(t0)). The knee closes the four-bar O-K-P-C: K on the circle of
//! radius a about O and the one of radius c about P, the branch below the line O-P:
//!
//!   phi = atan2(P_z, P_x) - acos((a^2 + |P|^2 - c^2) / (2 a |P|)) + pi / 2,
//!   phi' = (D . P_q) / (D . K_phi),   D = K - P,
//!   phi'' = -(E . E + D . (K_phiphi phi'^2 - P_qq)) / (D . K_phi),   E = K_phi phi' - P_q,
//!   x' = x_phi phi',   x'' = x_phiphi phi'^2 + x_phi phi''.
//!
//! x' (m/rad) is the transmission ratio, zero at the dead centre; the ram force a crank torque tau gives is tau / x'.
//! The joint works through the dead centre (the ram then rises again) where its stops let it: `joint.max_angle_deg`
//! above 0; on one side of it where they end there. The inverse from the ram's distance to the dead centre has two
//! branches, before (phi > 0) and after it (`TogglePress::crank_angle`).
//!
//! Masses: the crank and the rod's crank end (`rod.crank_end_share`) at the pin; the upper link a uniform bar about O
//! (J = m a^2 / 3, its centre at a / 2); at the knee the rod's other end and the lower link's share
//! (`toggle.lower_link_knee_share`); at the ram the ram, the die (payload) and the rest of the lower link. So
//! J(q) = J_crank + m_pin r^2 + (m_up a^2 / 3 + m_knee a^2) phi'^2 + m_x x'^2, its speed term
//! ((m_up a^2 / 3 + m_knee a^2) phi' phi'' + m_x x' x'') q'^2, and V = g (m_pin z_P - (m_knee a + m_up a / 2) cos(phi)
//! - m_x x).
//!
//! The process force (effects.process_force) is a table over the ram's distance from the bottom dead centre
//! (`process.distance_m`, `process.force_n`), pushing the ram up while it moves down or stands, and fading out over
//! `process.smoothing_m_s` as it rises (the part springs back at once): a forming force that holds through a dwell at
//! the dead centre. The frame stretches under it as a spring in series (effects.frame_stretch,
//! `frame.stretch_stiffness_n_m` = k): the ram stands F / k short of where the links put it, so the force is the root
//! of F = table(d + F / k), d the links' distance from the dead centre; on a segment of the table from (d_i, F_i) of
//! slope s_i, F = (F_i + s_i (d - d_i)) / (1 - s_i / k). Its torque at the crank is -F x'.
//!
//! Signals: the ram's height above the bottom dead centre (with the frame's stretch), its speed and acceleration, the
//! ratio x', the press force, the force capacity at the pose and its use, the frame's stretch, the forces of the upper
//! and lower link (compression positive; the links and their pins are two-force members, so the pins carry them) and
//! of the rod, and the inertia J(q) at the crank. Verdicts: "press-capacity" (the press force within the capacity,
//! min(`ratings.rated_press_force_n`, tau_max / |x'|), tau_max the drive's peak torque through the gearbox's ratio and
//! efficiency, at most `ratings.gearbox_max_torque_nm`), "upper-link-force", "lower-link-force"
//! (`ratings.link_max_force_n`), "rod-force" (`ratings.rod_max_force_n`), "frame-deflection"
//! (`ratings.frame_max_deflection_m`).

use crate::crank::slider_motion;
use crate::{Table, axis, gravity, machine, master, payload};
use sim_components::cam::Cam;
use sim_components::chain::Mechanism;
use sim_components::joint::RotaryJoint;
use sim_core::{CamMaster, Config, ConfigError, Label, Machine, Part, RatingCheck, Scene, Shape, SignalInfo};
use std::f64::consts::{FRAC_PI_2, PI, TAU};

pub struct TogglePress {
    /// Upper and lower link, crank radius, rod length, m.
    pub a: f64,
    pub b: f64,
    pub r: f64,
    pub c: f64,
    /// The crank's angle (from the scene's x towards z) at the dead centre, rad.
    pub t0: f64,
    /// The crank's centre (x, z), m, the top pivot at the origin.
    pub centre: [f64; 2],
    pub j_crank: f64,
    /// The rod's crank end, at the pin.
    pub m_pin: f64,
    /// The upper link, a uniform bar.
    pub m_upper: f64,
    /// At the knee: the rod's other end and the lower link's share.
    pub m_knee: f64,
    /// The ram and the lower link's rest, the die (payload) apart.
    pub m_ram: f64,
    pub payload: f64,
    pub attached: bool,
    pub g: f64,
    pub(crate) process: Option<Table>,
    /// The process force fades out over this ram speed upwards, m/s.
    pub smoothing: f64,
    /// The frame's stiffness under the press force, N/m; `None` with effects.frame_stretch off (a rigid frame).
    pub frame: Option<f64>,
    /// The torque the drive can put on the crank, N m: its peak torque through the gearbox.
    pub crank_torque: f64,
    /// What the frame and the links are rated for, N.
    pub rated_force: f64,
    pub cam: Option<(Cam, CamMaster)>,
    /// The crank's assembly for the inverse: the sign of the angle from C-K to C-P.
    branch: f64,
    /// Limits of the link, rod and frame checks (infinite: no check).
    limits: [f64; 3],
}

/// The linkage at a crank angle: the pin and the knee (x, z), the upper link's angle and its two rates over q, and the
/// ram x, x', x''.
#[derive(Debug, Clone, Copy)]
pub struct Pose {
    pub pin: [f64; 2],
    pub knee: [f64; 2],
    pub phi: [f64; 3],
    pub x: [f64; 3],
}

pub fn build(cfg: &Config) -> Result<Box<dyn Machine>, ConfigError> {
    let joint = RotaryJoint::from_config(cfg)?;
    let press = TogglePress::from_config(cfg)?;
    // The four-bar must close over the whole of the joint's range, and keep away from its own dead points (rod and upper
    // link in line), where phi' would have no bound.
    for k in 0..=400 {
        let q = joint.range.0 + (joint.range.1 - joint.range.0) * k as f64 / 400.0;
        let p = press.pose(q);
        if !(p.phi[1].is_finite() && p.x[0].is_finite() && p.phi[1].abs() < 50.0) {
            return Err(ConfigError::Invalid("crank.radius_m".into()));
        }
    }
    let q = axis(cfg, "crank", "axis-crank", "rad", joint.range)?;
    machine(cfg, press, (joint, None, "rad"), q, false)
}

fn dot(u: [f64; 2], v: [f64; 2]) -> f64 {
    u[0] * v[0] + u[1] * v[1]
}

impl TogglePress {
    pub fn from_config(cfg: &Config) -> Result<Self, ConfigError> {
        let (a, b) = (cfg.positive("toggle.upper_link_m")?, cfg.positive("toggle.lower_link_m")?);
        let (r, c) = (cfg.positive("crank.radius_m")?, cfg.positive("rod.length_m")?);
        if b < a {
            return Err(ConfigError::Invalid("toggle.lower_link_m".into()));
        }
        let share = |key: &str| -> Result<f64, ConfigError> {
            let s = cfg.non_negative(key)?;
            if s > 1.0 { Err(ConfigError::Invalid(key.into())) } else { Ok(s) }
        };
        let (rod_share, knee_share) = (share("rod.crank_end_share")?, share("toggle.lower_link_knee_share")?);
        let (rod, lower) = (cfg.non_negative("rod.mass_kg")?, cfg.non_negative("toggle.lower_link_mass_kg")?);
        let t0 = cfg.f64("crank.angle_at_dead_centre_deg")?.to_radians();
        if !(0.0..PI).contains(&t0) || t0 == 0.0 {
            return Err(ConfigError::Invalid("crank.angle_at_dead_centre_deg".into()));
        }
        let cam = Cam::from_config(cfg, "cam")?;
        let master = master(cfg, cam.period * cfg.positive("crank.strokes_per_min")? / 60.0, cam.period)?;
        let crank_torque = (cfg.positive("drive.peak_torque_nm")?
            * cfg.positive("gearbox.ratio")?
            * cfg.positive("gearbox.efficiency")?)
        .min(match cfg.get("ratings.gearbox_max_torque_nm") {
            Ok(_) => cfg.positive("ratings.gearbox_max_torque_nm")?,
            Err(_) => f64::INFINITY,
        });
        let centre = [c - r * t0.cos(), -a - r * t0.sin()];
        let mut press = TogglePress {
            a,
            b,
            r,
            c,
            t0,
            centre,
            j_crank: cfg.positive("crank.inertia_kgm2")?,
            m_pin: rod_share * rod,
            m_upper: cfg.non_negative("toggle.upper_link_mass_kg")?,
            m_knee: (1.0 - rod_share) * rod + knee_share * lower,
            m_ram: cfg.positive("ram.mass_kg")? + (1.0 - knee_share) * lower,
            payload: payload(cfg)?,
            attached: true,
            g: gravity(cfg)?,
            process: if cfg.bool("effects.process_force")? {
                Some(Table::from_config(cfg, "process.distance_m", "process.force_n")?)
            } else {
                None
            },
            smoothing: cfg.positive("process.smoothing_m_s")?,
            frame: if cfg.bool("effects.frame_stretch")? {
                Some(cfg.positive("frame.stretch_stiffness_n_m")?)
            } else {
                None
            },
            crank_torque,
            rated_force: cfg.positive("ratings.rated_press_force_n")?,
            cam: master.map(|m| (cam, m)),
            branch: 1.0,
            limits: ["link_max_force_n", "rod_max_force_n", "frame_max_deflection_m"]
                .map(|k| cfg.f64(&format!("ratings.{k}")).unwrap_or(f64::INFINITY)),
        };
        // The crank's assembly: the side of C-K that puts the crank at t0 with the links stretched.
        let to_knee = [-centre[0], -a - centre[1]];
        let (gamma, delta) = (to_knee[1].atan2(to_knee[0]), press.crank_delta(to_knee));
        let wrap = |x: f64| (x + PI).rem_euclid(TAU) - PI;
        press.branch = if wrap(gamma + delta - t0).abs() < wrap(gamma - delta - t0).abs() { 1.0 } else { -1.0 };
        Ok(press)
    }

    /// The angle between C-K and C-P where the rod of length c reaches K from the crank's circle.
    fn crank_delta(&self, to_knee: [f64; 2]) -> f64 {
        let e = to_knee[0].hypot(to_knee[1]);
        ((self.r * self.r + e * e - self.c * self.c) / (2.0 * self.r * e)).acos()
    }

    /// The linkage at the crank angle q (the module's formulas); NaN where the four-bar does not close.
    pub fn pose(&self, q: f64) -> Pose {
        let (a, r, c) = (self.a, self.r, self.c);
        let (s, co) = (self.t0 + q).sin_cos();
        let pin = [self.centre[0] + r * co, self.centre[1] + r * s];
        let (pq, pqq) = ([-r * s, r * co], [-r * co, -r * s]);
        let d = pin[0].hypot(pin[1]);
        let phi = pin[1].atan2(pin[0]) - ((a * a + d * d - c * c) / (2.0 * a * d)).acos() + FRAC_PI_2;
        let (sp, cp) = phi.sin_cos();
        let knee = [a * sp, -a * cp];
        let (kp, kpp) = ([a * cp, a * sp], [-a * sp, a * cp]);
        let dk = [knee[0] - pin[0], knee[1] - pin[1]];
        let den = dot(dk, kp);
        let phi1 = dot(dk, pq) / den;
        let e = [kp[0] * phi1 - pq[0], kp[1] * phi1 - pq[1]];
        let phi2 = -(dot(e, e) + dot(dk, [kpp[0] * phi1 * phi1 - pqq[0], kpp[1] * phi1 * phi1 - pqq[1]])) / den;
        let [x, xp, xpp] = slider_motion(a, self.b, phi);
        Pose { pin, knee, phi: [phi, phi1, phi2], x: [x, xp * phi1, xpp * phi1 * phi1 + xp * phi2] }
    }

    /// The inverse: the crank angle that puts the ram (the links, without the frame's stretch) `distance` above the
    /// bottom dead centre, before the dead centre (`before`, phi > 0) or after it; `None` beyond the links' reach or the
    /// crank's.
    pub fn crank_angle(&self, distance: f64, before: bool) -> Option<f64> {
        let (a, b) = (self.a, self.b);
        let x = a + b - distance;
        let cos = (a * a + x * x - b * b) / (2.0 * a * x);
        if !(distance >= 0.0 && x > b - a && cos.abs() <= 1.0 + 1e-12) {
            return None;
        }
        let cos = cos.clamp(-1.0, 1.0);
        let phi = if before { cos.acos() } else { -cos.acos() };
        let knee = [a * phi.sin(), -a * phi.cos()];
        let to_knee = [knee[0] - self.centre[0], knee[1] - self.centre[1]];
        let delta = self.crank_delta(to_knee);
        let t = to_knee[1].atan2(to_knee[0]) + self.branch * delta;
        delta.is_finite().then(|| (t - self.t0 + PI).rem_euclid(TAU) - PI)
    }

    fn m_x(&self) -> f64 {
        self.m_ram + if self.attached { self.payload } else { 0.0 }
    }

    /// What the knee and the upper link put in J: m_up a^2 / 3 + m_knee a^2.
    fn j_knee(&self) -> f64 {
        (self.m_upper / 3.0 + self.m_knee) * self.a * self.a
    }

    /// J(q): what the crank feels of crank, pin, links, knee and ram.
    pub fn inertia(&self, q: f64) -> f64 {
        let p = self.pose(q);
        self.j_crank + self.m_pin * self.r * self.r + self.j_knee() * p.phi[1] * p.phi[1] + self.m_x() * p.x[1] * p.x[1]
    }

    /// The torque the crank needs to hold the ram force `force` at q against nothing else: F x'.
    pub fn holding_torque(&self, q: f64, force: f64) -> f64 {
        force * self.pose(q).x[1]
    }

    /// The force the press can give at q: the rated force, or less where the drive's torque through the ratio gives less.
    pub fn capacity(&self, q: f64) -> f64 {
        let ratio = self.pose(q).x[1].abs();
        if ratio * self.rated_force <= self.crank_torque { self.rated_force } else { self.crank_torque / ratio }
    }

    /// The press force at the links' distance `d` from the bottom dead centre, before the motion's share: the root of
    /// F = table(d + F / k), segment by segment of the table (linear there: F = (v_i + s_i (d - d_i)) / (1 - s_i / k)).
    pub fn press_force_at(&self, d: f64) -> f64 {
        let Some(table) = &self.process else { return 0.0 };
        let Some(k) = self.frame else { return table.at(d) };
        if table.at.len() < 2 {
            return table.at(d + table.value[0] / k).min(table.value[0]);
        }
        for i in 0..table.at.len() - 1 {
            let (a0, a1, v0, v1) = (table.at[i], table.at[i + 1], table.value[i], table.value[i + 1]);
            let slope = (v1 - v0) / (a1 - a0);
            let f = (v0 + slope * (d - a0)) / (1.0 - slope / k);
            let u = d + f / k;
            if f >= 0.0 && u >= a0 && u <= a1 {
                return f;
            }
        }
        0.0
    }

    /// The press force on the ram (pushing it up) and the frame's stretch at q and the crank's speed: the table's force
    /// while the ram moves down or stands, fading out as it rises.
    pub fn press_force(&self, q: f64, speed: f64) -> (f64, f64) {
        if self.process.is_none() {
            return (0.0, 0.0);
        }
        let p = self.pose(q);
        let share = (1.0 + p.x[1] * speed / self.smoothing).clamp(0.0, 1.0);
        let force = self.press_force_at(self.a + self.b - p.x[0]) * share;
        (force, self.frame.map_or(0.0, |k| force / k))
    }

    /// The forces of the upper link, the lower link and the rod (compression positive), N, at the crank's angle, speed
    /// and acceleration. The ram along z: C_b w / b = m_x x'' + F - m_x g (w = x - a cos(phi)). The upper link about O:
    /// across the link at the knee it pushes T = (-m_up g a sin(phi) / 2 - J_up phi'') / a; the knee:
    /// m_knee K'' = C_a u_a + T n_a - C_b u_b + C_rod u_rod + m_knee g, solved for C_a and C_rod.
    pub fn link_forces(&self, q: f64, speed: f64, acc: f64) -> [f64; 3] {
        let (a, b) = (self.a, self.b);
        let p = self.pose(q);
        let (force, _) = self.press_force(q, speed);
        let ram_acc = p.x[2] * speed * speed + p.x[1] * acc;
        let (sp, cp) = p.phi[0].sin_cos();
        let w = p.x[0] - a * cp;
        let lower = (self.m_x() * (ram_acc - self.g) + force) * b / w;
        let (phi_v, phi_a) = (p.phi[1] * speed, p.phi[2] * speed * speed + p.phi[1] * acc);
        let across = (-self.m_upper * self.g * a * sp / 2.0 - self.m_upper * a * a / 3.0 * phi_a) / a;
        let knee_acc = [a * cp * phi_a - a * sp * phi_v * phi_v, a * sp * phi_a + a * cp * phi_v * phi_v];
        let (ua, na) = ([sp, -cp], [cp, sp]);
        let ub = [-a * sp / b, -w / b];
        let ur = [(p.knee[0] - p.pin[0]) / self.c, (p.knee[1] - p.pin[1]) / self.c];
        let rhs: Vec<f64> = (0..2)
            .map(|k| self.m_knee * (knee_acc[k] + if k == 1 { self.g } else { 0.0 }) - across * na[k] + lower * ub[k])
            .collect();
        let det = ua[0] * ur[1] - ua[1] * ur[0];
        let upper = (rhs[0] * ur[1] - rhs[1] * ur[0]) / det;
        let rod = (ua[0] * rhs[1] - ua[1] * rhs[0]) / det;
        [upper, lower, rod]
    }
}

impl Mechanism for TogglePress {
    /// In the scene's frame: the crank turning about -y with the rod's crank end at the pin, the upper link a bar turning
    /// about the top pivot (about -y as phi grows), the knee's and the ram's masses, the rotor about the crank's axis; the
    /// press force pushing the ram up.
    fn frame_momentum(&self, q: &[f64], qd: &[f64], rotor_spin: &[f64]) -> Option<sim_core::FrameMomentum> {
        use sim_components::base::{add_point, add_spin};
        let (p, w) = (self.pose(q[0]), qd[0]);
        let at = |v: [f64; 2]| [v[0], 0.0, v[1]];
        let (force, _) = self.press_force(q[0], w);
        let ram = -p.x[0];
        let mut f = sim_core::FrameMomentum {
            gravity: [0.0, 0.0, -self.g],
            external: [0.0, 0.0, force],
            external_at: [0.0, 0.0, ram],
            ..Default::default()
        };
        let (s, c) = (self.t0 + q[0]).sin_cos();
        add_point(&mut f, self.m_pin, at(p.pin), [-self.r * s * w, 0.0, self.r * c * w]);
        let phi_v = p.phi[1] * w;
        let knee_v = [self.a * p.phi[0].cos() * phi_v, 0.0, self.a * p.phi[0].sin() * phi_v];
        add_point(&mut f, self.m_knee, at(p.knee), knee_v);
        add_point(&mut f, self.m_upper, at([p.knee[0] / 2.0, p.knee[1] / 2.0]), knee_v.map(|v| v / 2.0));
        add_spin(&mut f, [0.0, -1.0, 0.0], self.m_upper * self.a * self.a / 12.0, phi_v);
        add_point(&mut f, self.m_x(), [0.0, 0.0, ram], [0.0, 0.0, -p.x[1] * w]);
        add_spin(&mut f, [0.0, -1.0, 0.0], self.j_crank, w);
        f.angular[1] -= rotor_spin[0];
        Some(f)
    }
    fn dynamics(&self, q: &[f64], qd: &[f64], mass: &mut [f64], bias: &mut [f64]) {
        let p = self.pose(q[0]);
        mass[0] = self.inertia(q[0]);
        let gravity = self.g
            * (-self.m_x() * p.x[1]
                + (self.m_knee + self.m_upper / 2.0) * self.a * p.phi[0].sin() * p.phi[1]
                + self.m_pin * self.r * (self.t0 + q[0]).cos());
        bias[0] = (self.j_knee() * p.phi[1] * p.phi[2] + self.m_x() * p.x[1] * p.x[2]) * qd[0] * qd[0] + gravity;
    }
    fn potential(&self, q: &[f64]) -> f64 {
        let p = self.pose(q[0]);
        self.g
            * (self.m_pin * p.pin[1]
                - (self.m_knee + self.m_upper / 2.0) * self.a * p.phi[0].cos()
                - self.m_x() * p.x[0])
    }
    fn set_payload_attached(&mut self, attached: bool) {
        self.attached = attached;
    }
    /// The speed term of J(q) needs the feedforward's speed terms.
    fn tool_is_joints(&self) -> bool {
        false
    }
    fn has_process(&self) -> bool {
        self.process.is_some()
    }
    fn process_torque(&self, q: &[f64], qd: &[f64], out: &mut [f64]) {
        out[0] -= self.press_force(q[0], qd[0]).0 * self.pose(q[0]).x[1];
    }
    fn cam(&self) -> Option<CamMaster> {
        self.cam.as_ref().map(|(_, m)| *m)
    }
    fn cam_at(&self, master: f64, out: &mut [[f64; 4]]) {
        if let Some((cam, _)) = &self.cam {
            out[0] = cam.at(master);
        }
    }

    fn signal_infos(&self) -> Vec<SignalInfo> {
        [
            ("ram.height", "group-ram", "quantity-height-above-bdc", "m"),
            ("ram.speed", "group-ram", "quantity-speed", "m/s"),
            ("ram.acceleration", "group-ram", "quantity-acceleration", "m/s²"),
            ("toggle.ratio", "group-toggle", "quantity-transmission-ratio", "m/rad"),
            ("press.force", "group-process", "quantity-process-force", "N"),
            ("press.capacity", "group-process", "quantity-force-capacity", "N"),
            ("press.capacity_use", "group-process", "quantity-capacity-use", ""),
            ("frame.deflection", "group-frame", "quantity-frame-deflection", "m"),
            ("toggle.upper_link_force", "group-upper-link", "quantity-link-force", "N"),
            ("toggle.lower_link_force", "group-lower-link", "quantity-link-force", "N"),
            ("rod.force", "group-rod", "quantity-rod-force", "N"),
            ("crank.inertia", "group-crank", "quantity-reflected-inertia", "kg m²"),
        ]
        .into_iter()
        .map(|(id, owner, quantity, unit)| SignalInfo::new(id, owner, quantity, unit))
        .collect()
    }
    fn signals(&self, q: &[f64], qd: &[f64], qdd: &[f64], _force: &[f64], out: &mut Vec<f64>) {
        let p = self.pose(q[0]);
        let (force, stretch) = self.press_force(q[0], qd[0]);
        let capacity = self.capacity(q[0]);
        let [upper, lower, rod] = self.link_forces(q[0], qd[0], qdd[0]);
        // The frame's stretch changes with the angle and (where the force fades as the ram rises) with the speed.
        let (h, at) = (1e-7, |q: f64, w: f64| self.press_force(q, w).1);
        let stretch_rate = (at(q[0] + h, qd[0]) - at(q[0] - h, qd[0])) / (2.0 * h) * qd[0]
            + (at(q[0], qd[0] + h) - at(q[0], qd[0] - h)) / (2.0 * h) * qdd[0];
        out.extend([
            self.a + self.b - p.x[0] + stretch,
            // The height is a + b - x + stretch, so its derivatives carry the minus sign of x (x is positive down).
            // shortcut: the acceleration leaves out the stretch's second derivative (a tenth of a millimetre at most).
            -p.x[1] * qd[0] + stretch_rate,
            -(p.x[2] * qd[0] * qd[0] + p.x[1] * qdd[0]),
            p.x[1],
            force,
            capacity,
            force / capacity,
            stretch,
            upper,
            lower,
            rod,
            self.inertia(q[0]),
        ]);
    }
    fn rating_checks(&self) -> Vec<RatingCheck> {
        let [link, rod, frame] = self.limits;
        [
            ("press-capacity", "press.capacity_use", 1.0),
            ("upper-link-force", "toggle.upper_link_force", link),
            ("lower-link-force", "toggle.lower_link_force", link),
            ("rod-force", "rod.force", rod),
            ("frame-deflection", "frame.deflection", frame),
        ]
        .into_iter()
        .filter(|(_, _, limit)| limit.is_finite())
        .map(|(check, signal, limit)| RatingCheck { check, signal: signal.into(), motor: None, limit, life: false })
        .collect()
    }

    fn collision_parts(&self) -> &'static [&'static str] {
        &["ram", "payload"]
    }

    /// Seen from the front: the crown with the top pivot, the two links to the ram on its guide, the rod from the crank's
    /// pin to the knee, the crank disc with the motor behind it; the bed below the ram's bottom dead centre.
    fn scene(&self, q: &[f64], rotors: &[f64], rotor_speeds: &[f64], out: &mut Scene) {
        let (a, b, r) = (self.a, self.b, self.r);
        let p = self.pose(q[0]);
        let (_, stretch) = self.press_force(q[0], 0.0);
        let at = |v: [f64; 2]| [v[0], 0.0, v[1]];
        let ram = -(p.x[0] - stretch);
        let grey = [0.62, 0.64, 0.68];
        let bottom = -(a + b) - 0.12;
        out.parts.push(Part::new("crown", Shape::Box { size: [0.3, 0.12, 0.05] }, [0.0, 0.0, 0.04], grey));
        out.parts.push(Part::new("bed", Shape::Box { size: [0.3, 0.12, 0.05] }, [0.0, 0.0, bottom], grey));
        out.parts.push(Part::new(
            "column",
            Shape::Segment { to: [-0.17, 0.0, bottom], width: 0.04 },
            [-0.17, 0.0, 0.04],
            grey,
        ));
        out.parts.push(Part::new(
            "upper-link",
            Shape::Segment { to: at(p.knee), width: 0.03 },
            [0.0; 3],
            [0.75, 0.75, 0.78],
        ));
        out.parts.push(Part::new(
            "lower-link",
            Shape::Segment { to: [0.0, 0.0, ram], width: 0.03 },
            at(p.knee),
            [0.75, 0.75, 0.78],
        ));
        out.parts.push(Part::new("rod", Shape::Segment { to: at(p.knee), width: 0.025 }, at(p.pin), [0.6, 0.6, 0.64]));
        let centre = at(self.centre);
        out.parts.push(Part::new(
            "crank",
            Shape::Cylinder { radius: 1.25 * r, length: 0.03 },
            centre,
            [0.45, 0.47, 0.5],
        ));
        out.parts.push(
            Part::new(
                "crank-arm",
                Shape::Box { size: [r, 0.004, 0.2 * r] },
                [(centre[0] + p.pin[0]) / 2.0, -0.02, (centre[2] + p.pin[1]) / 2.0],
                [0.1, 0.1, 0.1],
            )
            .turned(self.t0 + q[0]),
        );
        out.parts.push(Part::new(
            "ram",
            Shape::Box { size: [0.12, 0.1, 0.06] },
            [0.0, 0.0, ram - 0.03],
            [0.2, 0.22, 0.26],
        ));
        if self.attached && self.payload > 0.0 {
            out.parts.push(Part::new(
                "payload",
                Shape::Box { size: [0.08, 0.08, 0.04] },
                [0.0, 0.0, ram - 0.08],
                [0.2, 0.7, 0.3],
            ));
        }
        out.parts.push(Part::new(
            sim_core::scene::MOTOR_PART,
            Shape::Cylinder { radius: 0.05, length: 0.12 },
            [centre[0], 0.12, centre[2]],
            [0.9, 0.45, 0.1],
        ));
        out.parts.push(Part::new(
            "gearbox",
            Shape::Box { size: [0.1, 0.06, 0.1] },
            [centre[0], 0.06, centre[2]],
            [0.25, 0.25, 0.28],
        ));
        out.parts.push(
            Part::new(
                "rotor-bar",
                Shape::Box { size: [0.07, 0.004, 0.012] },
                [centre[0], 0.19, centre[2]],
                [0.1, 0.1, 0.1],
            )
            .turned(rotors[0]),
        );
        out.labels.push(Label {
            pos: [centre[0], 0.0, centre[2] + 1.25 * r + 0.08],
            owner_key: "motor-joint-1",
            value: rotor_speeds[0] * 60.0 / TAU,
            unit_key: "unit-rpm",
            decimals: 0,
        });
        out.labels.push(Label {
            pos: [0.15, 0.0, ram],
            owner_key: "group-ram",
            value: a + b + ram,
            unit_key: "unit-m",
            decimals: 5,
        });
    }
    fn scene_point(&self, tool: &[f64]) -> Option<[f64; 3]> {
        Some([0.0, 0.0, -self.pose(tool[0]).x[0]])
    }
}
