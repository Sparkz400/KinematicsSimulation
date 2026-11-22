//! A disc cam: a cam disc turned by the motor through its gearbox, and a roller follower on a straight slide above it,
//! held on the disc by a return spring and its weight, as in packaging, assembly and textile machines where a servo
//! turns the disc instead of a line shaft. The joint is the disc's angle theta (rad). The follower's lift s(theta) is
//! the disc's lift law, segments of VDI 2143 laws over the disc's angle (`[[disc.segment]]`, `sim_components::cam`)
//! over one turn, back where it began.
//!
//! The follower's axis is offset by e (`follower.offset_m`) from the disc's centre; its roller of radius r_r touches
//! the disc, whose base circle has radius r_b: the roller's centre runs on the pitch curve, s0 + s above the centre with
//! s0 = sqrt((r_b + r_r)^2 - e^2). Its pressure angle, between the follower's axis and the common normal at the
//! contact:
//!
//!   tan(phi) = (s' - e) / (s0 + s),    s' = ds / dtheta,
//!
//! for a disc turning so that the follower rises with theta (a positive offset lowers the angle on the rise).
//!
//! The follower (mass m with the tool on it, `effects.payload`) is pressed down by its spring, F0 + k s
//! (`follower.spring_preload_n`, `spring_n_m`), and its weight m g: the contact force along the normal is
//! N = (m s'' + F0 + k s + m g) / cos(phi), with the follower's acceleration s'' = s''(theta) theta'^2 + s'(theta)
//! theta''. The disc feels J(theta) = J_disc + m s'^2, the speed term m s' s'' theta'^2 and the spring's and weight's
//! torque (F0 + k s + m g) s' (the potential F0 s + k s^2 / 2 + m g s). The guide takes N sin(phi) across.
//!
//! `effects.follower_lift_off` (off as shipped) lets the follower leave the disc. Its lift is then s_f = s(theta) + g,
//! g the gap (the roller's centre above the pitch curve; negative: pressed into the disc), a passive joint after the
//! disc's angle (`Mechanism::passive_joints`, no spring of its own). The follower's speed is v = s' theta' + g'. With
//! T = J_disc theta'^2 / 2 + m v^2 / 2, the potential V = (F0 + m g) s_f + k s_f^2 / 2 and the guide's friction F_f =
//! mu tanh(v / 1 mm/s) against v, Lagrange's equations are
//!
//!   (J_disc + m s'^2) theta'' + m s' g'' + m s' s'' theta'^2 + (F0 + k s_f + m g + F_f) s' = torque,
//!   m s' theta'' + m g'' + m s'' theta'^2 + F0 + k s_f + m g + F_f = F_c.
//!
//! F_c, the contact force's component along the follower's axis, is the one force that acts on g alone: the disc's
//! surface moves by s' d theta, the roller by s' d theta + d g, so its virtual work is F_c d g (the disc feels F_c s'
//! through the mass coupling). The contact is one-sided, with the depth d = -g pressed in: the steel line contact of the
//! roller (width l, `follower.roller_width_m`) gives d = 3.84e-5 Q^0.9 / l^0.8 (mm, N, mm; Palmgren's roller contact,
//! Harris and Kotzalas, Rolling Bearing Analysis), Q = (d l^0.8 / 3.84e-5)^(1 / 0.9) and its stiffness dQ/dd = Q / (0.9
//! d) (4e8 N/m at 160 N and 14 mm), its energy the integral, 9 / 19 d Q. A damper at the share zeta of critical with
//! that stiffness, c_d = 2 zeta sqrt(Q / (0.9 d) m) (`follower.contact_damping_ratio`), adds c_d d': F_c = max(0, Q +
//! c_d d'), 0 for d <= 0, so the force on the disc is never negative and, as c_d falls to 0 with d, does not jump at
//! the touch. The damper's power (F_c - Q) d' (the clamp's included) is the loss `loss_power` books, and F_f v the
//! guide's, so the energy budget closes through an impact (the damper takes the normal speed out). In free flight (g >
//! 0) the follower moves under its spring, its weight and the guide's friction alone and meets the disc again where g
//! reaches 0. At rest it sits pressed in by its load (`Mechanism::passive_rest`, 0.45 um under 160 N). The contact
//! force along the normal is F_c / cos phi (signal `follower.contact_force`); the gap is the passive joint's signal
//! `follower.gap` (negative: pressed in), with the flag `follower.off_disc`. The feedforward knows the follower on the
//! disc (g = 0), as a drive does.
//!
//! Signals: the follower's lift, speed and acceleration, the contact force, the pressure angle, the guide's load
//! across and J(theta) (with the lift-off also the gap and its flag); the contact force and the pressure angle are
//! judged against `ratings.disc_max_contact_force_n` and `ratings.disc_max_pressure_angle_deg`.

use crate::{axis, gravity, machine, master, payload};
use sim_components::cam::{Cam, Law, Segment};
use sim_components::chain::Mechanism;
use sim_components::joint::RotaryJoint;
use sim_core::{CamMaster, Config, ConfigError, Label, Machine, Part, RatingCheck, Scene, Shape, SignalInfo};
use std::f64::consts::TAU;

pub struct DiscCam {
    /// The lift law over one turn of the disc.
    pub lift: Cam,
    pub base_radius: f64,
    pub roller_radius: f64,
    pub offset: f64,
    pub j_disc: f64,
    /// The follower's own mass, the tool on it (payload) apart.
    pub m_follower: f64,
    pub payload: f64,
    pub attached: bool,
    pub spring: f64,
    pub preload: f64,
    pub g: f64,
    /// The electronic cam that turns the disc at its speed with cam.run: one turn over one turn of the master.
    pub cam: Cam,
    pub master: Option<CamMaster>,
    /// With effects.follower_lift_off: the roller's width, the contact's damping ratio, the guide's friction.
    pub lift_off: Option<LiftOff>,
    limits: [f64; 2],
}

/// The follower's one-sided contact with the disc and its guide's friction (`effects.follower_lift_off`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LiftOff {
    /// The roller's width, m (`follower.roller_width_m`), for its line contact.
    pub width: f64,
    /// The contact damper as a share of critical damping with the follower's mass (`follower.contact_damping_ratio`).
    pub zeta: f64,
    /// The guide's Coulomb friction, N (`follower.guide_friction_n`).
    pub friction: f64,
}

/// The speed at which the guide's friction reaches its full level (tanh), m/s.
const GUIDE_SMOOTHING: f64 = 1e-3;

/// Palmgren's line contact of steel, d = PALMGREN Q^0.9 / l^0.8 in m, N, m: 3.84e-5 mm N^-0.9 mm^0.8, times 1e-3 m/mm
/// over 1000^0.8 for l in m.
const PALMGREN: f64 = 3.84e-8 / 251.188_643_150_958;

impl LiftOff {
    /// The depth pressed in under the load `q`, m.
    pub(crate) fn depth(&self, q: f64) -> f64 {
        PALMGREN * q.max(0.0).powf(0.9) / self.width.powf(0.8)
    }
    /// The load at the depth `d` (0 for d <= 0), N, and the contact's stiffness there, N/m.
    pub(crate) fn load(&self, d: f64) -> [f64; 2] {
        if d <= 0.0 {
            return [0.0; 2];
        }
        let q = (d * self.width.powf(0.8) / PALMGREN).powf(1.0 / 0.9);
        [q, q / (0.9 * d)]
    }
}

pub fn build(cfg: &Config) -> Result<Box<dyn Machine>, ConfigError> {
    let joint = RotaryJoint::from_config(cfg)?;
    let disc = DiscCam::from_config(cfg)?;
    let theta = axis(cfg, "disc", "axis-disc", "rad", joint.range)?;
    machine(cfg, disc, (joint, None, "rad"), theta, true)
}

impl DiscCam {
    pub fn from_config(cfg: &Config) -> Result<Self, ConfigError> {
        let lift = Cam::from_config(cfg, "disc")?;
        // One turn of the disc, back to the base circle: the law repeats with the disc.
        if (lift.period - TAU).abs() > 1e-6 || lift.advance.abs() > 1e-9 {
            return Err(ConfigError::Invalid("disc.segment (one turn, 2 pi rad, rising by 0 in all)".into()));
        }
        let (base_radius, roller_radius) =
            (cfg.positive("disc.base_radius_m")?, cfg.positive("follower.roller_radius_m")?);
        let offset = cfg.f64("follower.offset_m")?;
        if offset.abs() >= base_radius + roller_radius {
            return Err(ConfigError::Invalid("follower.offset_m".into()));
        }
        let cam = Cam::from_segments(&[Segment { master: TAU, slave: TAU, law: Law::Linear }])
            .expect("one turn at constant speed");
        let master = master(cfg, TAU * cfg.positive("disc.turns_per_min")? / 60.0, cam.period)?;
        Ok(DiscCam {
            lift,
            base_radius,
            roller_radius,
            offset,
            j_disc: cfg.positive("disc.inertia_kgm2")?,
            m_follower: cfg.positive("follower.mass_kg")?,
            payload: payload(cfg)?,
            attached: true,
            spring: cfg.non_negative("follower.spring_n_m")?,
            preload: cfg.non_negative("follower.spring_preload_n")?,
            g: gravity(cfg)?,
            cam,
            master,
            lift_off: if cfg.bool("effects.follower_lift_off")? {
                Some(LiftOff {
                    width: cfg.positive("follower.roller_width_m")?,
                    zeta: cfg.non_negative("follower.contact_damping_ratio")?,
                    friction: cfg.non_negative("follower.guide_friction_n")?,
                })
            } else {
                None
            },
            limits: [
                cfg.f64("ratings.disc_max_contact_force_n").unwrap_or(f64::INFINITY),
                cfg.f64("ratings.disc_max_pressure_angle_deg").map_or(f64::INFINITY, f64::to_radians),
            ],
        })
    }

    fn m(&self) -> f64 {
        self.m_follower + if self.attached { self.payload } else { 0.0 }
    }

    /// The roller centre's height above the disc's centre on the base circle, s0 = sqrt((r_b + r_r)^2 - e^2).
    pub fn s0(&self) -> f64 {
        let pitch = self.base_radius + self.roller_radius;
        (pitch * pitch - self.offset * self.offset).sqrt()
    }

    /// The pressure angle at the disc's angle, rad (the module's formula).
    pub fn pressure_angle(&self, theta: f64) -> f64 {
        let [s, s1, ..] = self.lift.at(theta);
        ((s1 - self.offset) / (self.s0() + s)).atan()
    }

    /// The spring's and the weight's force on the follower, down, N.
    fn held(&self, s: f64) -> f64 {
        self.preload + self.spring * s + self.m() * self.g
    }

    /// The contact force along the normal and the guide's load across, N, at the disc's angle, speed and acceleration.
    pub fn loads(&self, theta: f64, speed: f64, acc: f64) -> [f64; 2] {
        let [s, s1, s2, _] = self.lift.at(theta);
        let phi = self.pressure_angle(theta);
        let along = self.m() * (s2 * speed * speed + s1 * acc) + self.held(s);
        let contact = along / phi.cos();
        [contact, contact * phi.sin()]
    }

    /// The contact force along the follower's axis F_c and its elastic part Q, N, at the gap g and its rate g' (both 0
    /// off the disc and without `effects.follower_lift_off`).
    pub fn contact_force(&self, g: f64, gd: f64) -> [f64; 2] {
        let Some(lo) = self.lift_off else { return [0.0; 2] };
        let [q, stiffness] = lo.load(-g);
        let damper = 2.0 * lo.zeta * (stiffness * self.m()).sqrt() * -gd;
        [(q + damper).max(0.0), q]
    }

    /// The guide's friction force against the follower's speed v, N.
    fn guide_friction(&self, v: f64) -> f64 {
        self.lift_off.map_or(0.0, |lo| lo.friction * (v / GUIDE_SMOOTHING).tanh())
    }

    /// The gap, its rate and its acceleration from the joints' q, q' and q''; 0 where they hold the disc's angle only
    /// (the rigid feedforward's checks).
    fn gap(q: &[f64], qd: &[f64], qdd: &[f64]) -> [f64; 3] {
        let at = |x: &[f64]| x.get(1).copied().unwrap_or(0.0);
        [at(q), at(qd), at(qdd)]
    }
}

impl Mechanism for DiscCam {
    /// In the scene's frame: the follower (with the tool on it) lifting on its axis, the disc and the rotor turning
    /// about -y. Its spring pushes on the frame on the base: nothing from outside.
    fn frame_momentum(&self, q: &[f64], qd: &[f64], rotor_spin: &[f64]) -> Option<sim_core::FrameMomentum> {
        let [s, s1, ..] = self.lift.at(q[0]);
        let mut f = sim_core::FrameMomentum { gravity: [0.0, 0.0, -self.g], ..Default::default() };
        let [g, gd, _] = Self::gap(q, qd, &[]);
        let at = [self.offset, 0.0, self.s0() + s + g];
        sim_components::base::add_point(&mut f, self.m(), at, [0.0, 0.0, s1 * qd[0] + gd]);
        sim_components::base::add_spin(&mut f, [0.0, -1.0, 0.0], self.j_disc, qd[0]);
        f.angular[1] -= rotor_spin[0];
        Some(f)
    }
    fn dynamics(&self, q: &[f64], qd: &[f64], mass: &mut [f64], bias: &mut [f64]) {
        let [s, s1, s2, _] = self.lift.at(q[0]);
        let m = self.m();
        mass[0] = self.j_disc + m * s1 * s1;
        bias[0] = m * s1 * s2 * qd[0] * qd[0] + self.held(s) * s1;
        if self.lift_off.is_some() {
            // With the gap as a joint: the matrix and forces of the doc comment (the disc first).
            let [g, gd, _] = Self::gap(q, qd, &[]);
            let push = self.held(s + g) + self.guide_friction(s1 * qd[0] + gd);
            (mass[1], mass[2], mass[3]) = (m * s1, m * s1, m);
            bias[0] = m * s1 * s2 * qd[0] * qd[0] + push * s1;
            bias[1] = m * s2 * qd[0] * qd[0] + push - self.contact_force(g, gd)[0];
        }
    }
    fn potential(&self, q: &[f64]) -> f64 {
        let g = q.get(1).copied().unwrap_or(0.0);
        let s = self.lift.at(q[0])[0] + g;
        let contact = self.lift_off.map_or(0.0, |lo| 9.0 / 19.0 * -g * lo.load(-g)[0]);
        (self.preload + self.m() * self.g) * s + 0.5 * self.spring * s * s + contact
    }
    fn passive_joints(&self) -> Vec<(f64, f64)> {
        self.lift_off.map(|_| (0.0, 0.0)).into_iter().collect()
    }
    fn passive_signals(&self) -> Vec<SignalInfo> {
        let gap = SignalInfo::new("follower.gap", "group-follower", "quantity-follower-gap", "m");
        self.lift_off.map(|_| gap).into_iter().collect()
    }
    /// The gap joint has no damper of its own: the contact's is `loss_row`'s.
    fn passive_loss_row(&self) -> Option<(&'static str, &'static str)> {
        None
    }
    /// Pressed in by the spring's and the weight's force: Q(d) = F0 + k (s - d) + m g, by fixed point (k d is some 1e-5
    /// of Q, so the second step is exact to 1e-10 of d).
    fn passive_rest(&self, q: &[f64]) -> Vec<f64> {
        let Some(lo) = self.lift_off else { return Vec::new() };
        let s = self.lift.at(q[0])[0];
        let mut d = 0.0;
        for _ in 0..3 {
            d = lo.depth(self.held(s - d));
        }
        vec![-d]
    }
    /// The contact damper's and the guide's friction power, as a row of the loss budget.
    fn loss_row(&self) -> Option<(&'static str, &'static str)> {
        self.lift_off.map(|_| ("follower_contact", "group-follower"))
    }
    /// (F_c - c_s d) d' with d' = -g', and F_f v: the damper's (the clamp's included) and the guide's, W.
    fn loss_power(&self, q: &[f64], qd: &[f64]) -> f64 {
        let [g, gd, _] = Self::gap(q, qd, &[]);
        let [force, spring] = self.contact_force(g, gd);
        let v = self.lift.at(q[0])[1] * qd[0] + gd;
        -(force - spring) * gd + self.guide_friction(v) * v
    }
    fn forward(&self, q: &[f64], tool: &mut [f64]) {
        tool[0] = q[0];
    }
    fn set_payload_attached(&mut self, attached: bool) {
        self.attached = attached;
    }
    /// The speed term of J(theta) needs the feedforward's speed terms.
    fn tool_is_joints(&self) -> bool {
        false
    }
    fn cam(&self) -> Option<CamMaster> {
        self.master
    }
    fn cam_at(&self, master: f64, out: &mut [[f64; 4]]) {
        out[0] = self.cam.at(master);
    }
    fn cam_curve(&self) -> Option<sim_core::CamCurve> {
        let cam = self.cam.clone();
        Some(std::sync::Arc::new(move |master, out: &mut [[f64; 4]]| out[0] = cam.at(master)))
    }

    fn signal_infos(&self) -> Vec<SignalInfo> {
        [
            ("follower.lift", "group-follower", "quantity-position", "m"),
            ("follower.speed", "group-follower", "quantity-speed", "m/s"),
            ("follower.acceleration", "group-follower", "quantity-acceleration", "m/s²"),
            ("follower.contact_force", "group-follower", "quantity-contact-force", "N"),
            ("follower.pressure_angle", "group-follower", "quantity-pressure-angle", "rad"),
            ("follower.guide_load", "group-follower", "quantity-guide-load", "N"),
            ("disc.inertia", "group-disc", "quantity-disc-inertia", "kg m²"),
        ]
        .into_iter()
        .chain(self.lift_off.map(|_| ("follower.off_disc", "group-follower", "quantity-follower-off-disc", "")))
        .map(|(id, owner, quantity, unit)| SignalInfo::new(id, owner, quantity, unit))
        .collect()
    }
    fn signals(&self, q: &[f64], qd: &[f64], qdd: &[f64], _force: &[f64], out: &mut Vec<f64>) {
        let [s, s1, s2, _] = self.lift.at(q[0]);
        let [g, gd, gdd] = Self::gap(q, qd, qdd);
        let [mut contact, mut guide] = self.loads(q[0], qd[0], qdd[0]);
        if self.lift_off.is_some() {
            // The contact's force, never below zero; the plan's (no gap in the state) is the rigid one, cut at zero.
            let (cos, sin) = (self.pressure_angle(q[0]).cos(), self.pressure_angle(q[0]).sin());
            let along = if q.len() > 1 { self.contact_force(g, gd)[0] } else { (contact * cos).max(0.0) };
            (contact, guide) = (along / cos, along / cos * sin);
        }
        out.extend([
            s + g,
            s1 * qd[0] + gd,
            s2 * qd[0] * qd[0] + s1 * qdd[0] + gdd,
            contact,
            self.pressure_angle(q[0]),
            guide,
            self.j_disc + self.m() * s1 * s1,
        ]);
        if self.lift_off.is_some() {
            // Off: the contact carries nothing (in flight, or leaving with its damper's pull cut off).
            out.push(f64::from(contact == 0.0));
        }
    }
    fn rating_checks(&self) -> Vec<RatingCheck> {
        [("disc-contact-force", "follower.contact_force"), ("disc-pressure-angle", "follower.pressure_angle")]
            .into_iter()
            .zip(self.limits)
            .filter(|(_, limit)| limit.is_finite())
            .map(|((check, signal), limit)| RatingCheck {
                check,
                signal: signal.into(),
                motor: None,
                limit,
                life: false,
            })
            .collect()
    }

    fn collision_parts(&self) -> &'static [&'static str] {
        &["follower", "payload"]
    }

    /// Seen from the front: the disc at the origin with a mark that turns with it, the roller on it at the follower's
    /// offset and lift, the follower above; the motor behind the disc.
    fn scene(&self, q: &[f64], rotors: &[f64], rotor_speeds: &[f64], out: &mut Scene) {
        let (r, e) = (self.base_radius, self.offset);
        let s = self.lift.at(q[0])[0] + q.get(1).copied().unwrap_or(0.0);
        let roller = [e, 0.0, self.s0() + s];
        out.parts.push(Part::new("disc", Shape::Cylinder { radius: r, length: 0.02 }, [0.0; 3], [0.45, 0.47, 0.5]));
        out.parts.push(
            Part::new("disc-mark", Shape::Box { size: [r, 0.004, 0.1 * r] }, [0.0, -0.012, 0.0], [0.1, 0.1, 0.1])
                .turned(q[0]),
        );
        out.parts.push(Part::new(
            "roller",
            Shape::Cylinder { radius: self.roller_radius, length: 0.02 },
            roller,
            [0.2, 0.22, 0.26],
        ));
        let top = [e, 0.0, roller[2] + 0.15];
        // A rod as a box, so that it is the follower's collision box (`collision_parts`).
        out.parts.push(Part::new(
            "follower",
            Shape::Box { size: [0.02, 0.02, 0.15] },
            [e, 0.0, roller[2] + 0.075],
            [0.75, 0.75, 0.78],
        ));
        if self.attached && self.payload > 0.0 {
            out.parts.push(Part::new("payload", Shape::Box { size: [0.06, 0.06, 0.04] }, top, [0.2, 0.7, 0.3]));
        }
        out.parts.push(Part::new(
            sim_core::scene::MOTOR_PART,
            Shape::Cylinder { radius: 0.05, length: 0.12 },
            [0.0, 0.12, 0.0],
            [0.9, 0.45, 0.1],
        ));
        out.parts.push(Part::new(
            "gearbox",
            Shape::Box { size: [0.1, 0.06, 0.1] },
            [0.0, 0.06, 0.0],
            [0.25, 0.25, 0.28],
        ));
        out.parts.push(
            Part::new("rotor-bar", Shape::Box { size: [0.07, 0.004, 0.012] }, [0.0, 0.19, 0.0], [0.1, 0.1, 0.1])
                .turned(rotors[0]),
        );
        out.labels.push(Label {
            pos: [0.0, 0.0, r + 0.08],
            owner_key: "motor-joint-1",
            value: rotor_speeds[0] * 60.0 / TAU,
            unit_key: "unit-rpm",
            decimals: 0,
        });
        out.labels.push(Label { pos: top, owner_key: "group-follower", value: s, unit_key: "unit-m", decimals: 4 });
    }
    fn scene_point(&self, tool: &[f64]) -> Option<[f64; 3]> {
        Some([self.offset, 0.0, self.s0() + self.lift.at(tool[0])[0] + 0.15])
    }
}
