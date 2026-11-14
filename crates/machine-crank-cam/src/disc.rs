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
//! shortcut: the roller stays on the disc; where N falls below zero the follower would lift off (the signal
//! follower.contact_force shows it), and the model does not part them. Model the jump if a spring is ever sized that
//! short.
//!
//! Signals: the follower's lift, speed and acceleration, the contact force, the pressure angle, the guide's load
//! across and J(theta); the contact force and the pressure angle are judged against `ratings.disc_max_contact_force_n`
//! and `ratings.disc_max_pressure_angle_deg`.

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
    limits: [f64; 2],
}

pub fn build(cfg: &Config) -> Result<Box<dyn Machine>, ConfigError> {
    let joint = RotaryJoint::from_config(cfg)?;
    let disc = DiscCam::from_config(cfg)?;
    let endless = disc.master.is_some();
    let theta = axis(cfg, "disc", "axis-disc", "rad", joint.range)?;
    machine(cfg, disc, (joint, None, "rad"), theta, endless)
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
}

impl Mechanism for DiscCam {
    fn dynamics(&self, q: &[f64], qd: &[f64], mass: &mut [f64], bias: &mut [f64]) {
        let [s, s1, s2, _] = self.lift.at(q[0]);
        mass[0] = self.j_disc + self.m() * s1 * s1;
        bias[0] = self.m() * s1 * s2 * qd[0] * qd[0] + self.held(s) * s1;
    }
    fn potential(&self, q: &[f64]) -> f64 {
        let s = self.lift.at(q[0])[0];
        (self.preload + self.m() * self.g) * s + 0.5 * self.spring * s * s
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
        .map(|(id, owner, quantity, unit)| SignalInfo::new(id, owner, quantity, unit))
        .collect()
    }
    fn signals(&self, q: &[f64], qd: &[f64], qdd: &[f64], _force: &[f64], out: &mut Vec<f64>) {
        let [s, s1, s2, _] = self.lift.at(q[0]);
        let [contact, guide] = self.loads(q[0], qd[0], qdd[0]);
        out.extend([
            s,
            s1 * qd[0],
            s2 * qd[0] * qd[0] + s1 * qdd[0],
            contact,
            self.pressure_angle(q[0]),
            guide,
            self.j_disc + self.m() * s1 * s1,
        ]);
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
        let s = self.lift.at(q[0])[0];
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
        out.parts.push(Part::new("follower", Shape::Segment { to: top, width: 0.02 }, roller, [0.75, 0.75, 0.78]));
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
