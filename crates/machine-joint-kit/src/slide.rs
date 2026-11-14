//! A carriage that slides along one axis, x (m), driven by a ball screw, a rack and pinion, or a linear motor. The
//! axis may be inclined (`slide.incline_deg`, 90 for a vertical axis moving up): gravity pulls along it by the sine.
//!
//! M = m (carriage plus payload), h = m g sin(incline). Screw and rack are `transmission::Converter`s behind the
//! joint's gearbox; the linear motor drives the carriage directly, its `[motor]` values read in N and m (force
//! constant in N/A, the forcer's mass as the rotor's inertia, not in `mass.carriage_kg`), and the attraction between
//! forcer and magnets presses on the guides (signal `axis.x.guide_load`, with the weights of carriage, payload and
//! forcer; judged by the guide checks) and adds its friction. The rack's feed force is the force on its teeth.

use crate::{axis, gravity, machine, payload, required_life};
use sim_components::chain::Mechanism;
use sim_components::joint::RotaryJoint;
use sim_components::transmission::{BallScrew, LinearMotor, RackPinion};
use sim_core::{Config, ConfigError, Label, Machine, Part, RatingCheck, Scene, Shape, SignalInfo};
use std::f64::consts::TAU;

#[derive(Debug, Clone, Copy)]
pub enum Kind {
    Screw,
    Rack,
    LinearMotor,
}

enum Drive {
    Screw(BallScrew),
    /// The rack and the efficiency in effect on its teeth.
    Rack(RackPinion, f64),
    /// The motor and the forcer's mass, which the guides carry beside the carriage.
    Linear(LinearMotor, f64),
}

pub struct Slide {
    drive: Drive,
    mass: f64,
    payload: f64,
    attached: bool,
    g: f64,
    incline: f64,
    stroke: f64,
    required_life: Option<f64>,
    /// A gearbox before screw or rack (drawn, and it warms).
    geared: bool,
}

pub fn build(cfg: &Config, kind: Kind) -> Result<Box<dyn Machine>, ConfigError> {
    let mut joint = RotaryJoint::linear_from_config(cfg)?;
    let incline = cfg.f64("slide.incline_deg")?;
    if !(-90.0..=90.0).contains(&incline) {
        return Err(ConfigError::Invalid("slide.incline_deg".into()));
    }
    let (drive, lever) = match kind {
        Kind::Screw => {
            let screw = BallScrew::from_config(cfg)?;
            if screw.length < joint.range.1 {
                return Err(ConfigError::Invalid("screw.length_m".into()));
            }
            screw.converter(screw.softest_position()).apply(cfg, &mut joint)?;
            // The nut's preload drags like friction in the guides, as a force at the nut.
            if cfg.bool("effects.joint_friction")? {
                joint.friction.coulomb += screw.preload_torque() * screw.ratio();
            }
            let lever = 1.0 / screw.ratio();
            (Drive::Screw(screw), Some(lever))
        }
        Kind::Rack => {
            let rack = RackPinion::from_config(cfg)?;
            rack.converter().apply(cfg, &mut joint)?;
            let lever = rack.radius();
            let efficiency = joint.extra_efficiency;
            (Drive::Rack(rack, efficiency), Some(lever))
        }
        Kind::LinearMotor => {
            let motor = LinearMotor::from_config(cfg)?;
            joint.ripple.cogging_order = motor.cogging_order();
            if cfg.bool("effects.joint_friction")? {
                joint.friction.coulomb +=
                    cfg.non_negative("linear_motor.guide_friction_coefficient")? * motor.attraction;
            }
            // The forcer moves with the carriage as a rotor turns with its shaft: its mass is `rotor_inertia_kgm2`.
            let forcer = joint.rotor_inertia;
            (Drive::Linear(motor, forcer), None)
        }
    };
    let slide = Slide {
        drive,
        mass: cfg.positive("mass.carriage_kg")?,
        payload: payload(cfg, "mass.payload_kg")?,
        attached: true,
        g: gravity(cfg)?,
        incline: incline.to_radians(),
        stroke: joint.range.1,
        required_life: required_life(cfg),
        geared: !joint.coupled,
    };
    let x = axis(cfg, "x", "axis-x", "m", joint.range)?;
    machine(cfg, slide, vec![(joint, lever, "m")], vec![x])
}

impl Slide {
    fn moving(&self) -> f64 {
        self.mass + if self.attached { self.payload } else { 0.0 }
    }
}

impl Mechanism for Slide {
    fn dynamics(&self, _q: &[f64], _qd: &[f64], mass: &mut [f64], bias: &mut [f64]) {
        mass[0] = self.moving();
        bias[0] = self.moving() * self.g * self.incline.sin();
    }
    fn potential(&self, q: &[f64]) -> f64 {
        self.moving() * self.g * self.incline.sin() * q[0]
    }
    fn set_payload_attached(&mut self, attached: bool) {
        self.attached = attached;
    }

    fn signal_infos(&self) -> Vec<SignalInfo> {
        match &self.drive {
            Drive::Screw(s) => s.signal_infos("1", "joint-1"),
            Drive::Rack(r, _) => vec![r.signal_info("1", "joint-1")],
            Drive::Linear(..) => vec![SignalInfo::new("axis.x.guide_load", "axis-x", "quantity-guide-load", "N")],
        }
    }
    fn signals(&self, _q: &[f64], qd: &[f64], _qdd: &[f64], force: &[f64], out: &mut Vec<f64>) {
        match &self.drive {
            Drive::Screw(s) => s.signals(qd[0], force[0], out),
            Drive::Rack(_, efficiency) => out.push(RackPinion::feed_force(force[0], qd[0], *efficiency)),
            Drive::Linear(m, forcer) => out.push(m.attraction + (self.moving() + forcer) * self.g * self.incline.cos()),
        }
    }
    /// The linear motor's guides carry the attraction and the weight across the rail, whatever the acceleration along it.
    fn rigid_guide_loads(&self, _tool: &[f64], _acc: &[f64], out: &mut Vec<[f64; 2]>) {
        if let Drive::Linear(m, forcer) = &self.drive {
            out.push([m.attraction + (self.moving() + forcer) * self.g * self.incline.cos(), 0.0]);
        }
    }
    fn rating_checks(&self) -> Vec<RatingCheck> {
        match &self.drive {
            Drive::Screw(s) => s.rating_checks("1", 0, self.required_life),
            Drive::Rack(r, _) => r.rating_checks("1", 0),
            Drive::Linear(..) => Vec::new(),
        }
    }

    fn collision_parts(&self) -> &'static [&'static str] {
        &["carriage", "payload"]
    }

    /// Drawn along the incline: frame, the screw or rack, the carriage, the motor at the start of the stroke.
    fn scene(&self, q: &[f64], rotors: &[f64], rotor_speeds: &[f64], out: &mut Scene) {
        let (s, c) = self.incline.sin_cos();
        let at = |x: f64, up: f64| [x * c - up * s, 0.0, x * s + up * c];
        let grey = [0.62, 0.64, 0.68];
        let half = 0.5 * self.stroke;
        out.parts.push(Part::new(
            "frame",
            Shape::Segment { to: at(self.stroke + 0.1, -0.05), width: 0.08 },
            at(-0.1, -0.05),
            grey,
        ));
        let (drive_id, drive_colour) = match self.drive {
            Drive::Screw(_) => ("screw", [0.75, 0.75, 0.78]),
            Drive::Rack(..) => ("rack", [0.4, 0.4, 0.42]),
            Drive::Linear(..) => ("magnet-track", [0.25, 0.3, 0.5]),
        };
        out.parts.push(Part::new(
            drive_id,
            Shape::Segment { to: at(self.stroke + 0.05, 0.0), width: 0.02 },
            at(-0.05, 0.0),
            drive_colour,
        ));
        out.parts.push(Part::new(
            "carriage",
            Shape::Box { size: [0.15, 0.12, 0.06] },
            at(q[0], 0.03),
            [0.2, 0.22, 0.26],
        ));
        if self.attached && self.payload > 0.0 {
            out.parts.push(Part::new("payload", Shape::Box { size: [0.1, 0.1, 0.08] }, at(q[0], 0.1), [0.2, 0.7, 0.3]));
        }
        let speed = rotor_speeds[0];
        match self.drive {
            Drive::Linear(..) => {
                out.parts.push(Part::new(
                    sim_core::scene::MOTOR_PART,
                    Shape::Box { size: [0.12, 0.1, 0.02] },
                    at(q[0], -0.01),
                    [0.9, 0.45, 0.1],
                ));
                out.labels.push(Label {
                    pos: at(q[0], 0.2),
                    owner_key: "motor-joint-1",
                    value: speed,
                    unit_key: "unit-m-per-s",
                    decimals: 2,
                });
            }
            _ => {
                let motor = at(-0.15, 0.0);
                out.parts.push(Part::new(
                    sim_core::scene::MOTOR_PART,
                    Shape::Cylinder { radius: 0.05, length: 0.12 },
                    motor,
                    [0.9, 0.45, 0.1],
                ));
                if self.geared {
                    out.parts.push(Part::new(
                        "gearbox",
                        Shape::Box { size: [0.08, 0.08, 0.08] },
                        at(-0.05, 0.0),
                        [0.25, 0.25, 0.28],
                    ));
                }
                out.parts.push(
                    Part::new(
                        "rotor-bar",
                        Shape::Box { size: [0.07, 0.004, 0.012] },
                        [motor[0], -0.07, motor[2]],
                        [0.1, 0.1, 0.1],
                    )
                    .turned(rotors[0]),
                );
                out.labels.push(Label {
                    pos: at(-0.15, 0.12),
                    owner_key: "motor-joint-1",
                    value: speed * 60.0 / TAU,
                    unit_key: "unit-rpm",
                    decimals: 0,
                });
            }
        }
        out.labels.push(Label {
            pos: at(half, -0.15),
            owner_key: "axis-x",
            value: q[0],
            unit_key: "unit-m",
            decimals: 3,
        });
    }
    fn scene_point(&self, tool: &[f64]) -> Option<[f64; 3]> {
        let (s, c) = self.incline.sin_cos();
        Some([tool[0] * c, 0.0, tool[0] * s])
    }
}
