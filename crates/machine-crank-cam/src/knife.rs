//! A rotary knife (cross cutter): a knife roll of radius R with `knife.knives` blades cuts a web that runs past it at
//! `web.speed_m_s`. The joint is the roll's angle phi (rad), 0 with a blade at the cut. The cam's master is the web, in
//! metres; one format length L of web per cut, one blade pitch 2 pi / n of the roll.
//!
//! The cam: in the synchronous zone, +- alpha (`knife.sync_angle_deg`) about each cut, the blade's tip runs with the web
//! (slope 1 / R); between cuts a motion law (`knife.transition_law`, VDI 2143) takes the roll the rest of the pitch,
//! 2 pi / n - 2 alpha, over the rest of the format, L - 2 R alpha. A format shorter than the roll's circumference over
//! n makes it speed up between cuts, a longer one slow down: its mean speed ratio there is
//! (2 pi / n - 2 alpha) / (L - 2 R alpha) radians per metre of web against 1 / R in the zone.
//!
//! The cut (effects.process_force) brakes the roll with a force at the blade's tip, a table over the angle from the
//! cut (`cut.angle_deg`, `cut.force_n`), while the roll turns forwards.

use crate::{Table, axis, forwards, machine, master, payload};
use sim_components::cam::{Cam, Law, Segment};
use sim_components::chain::Mechanism;
use sim_components::joint::RotaryJoint;
use sim_core::{CamMaster, Config, ConfigError, Label, Machine, Part, Scene, Shape, SignalInfo};
use std::f64::consts::TAU;

pub struct RotaryKnife {
    pub radius: f64,
    pub knives: f64,
    pub inertia: f64,
    /// Knife bars at the radius (the payload), kg.
    pub bars: f64,
    pub attached: bool,
    pub(crate) cut: Option<Table>,
    pub cam: Cam,
    pub master: Option<CamMaster>,
}

/// The rotary knife's cam (the module's): over a format length `format` of web, `knives` blades on a roll of `radius`,
/// synchronous over +- `sync` rad about each cut, `law` between. `None` where the format is no longer than the zone.
pub fn knife_cam(radius: f64, knives: f64, sync: f64, format: f64, law: Law) -> Option<Cam> {
    let zone = Segment { master: radius * sync, slave: sync, law: Law::Linear };
    let rest = Segment { master: format - 2.0 * radius * sync, slave: TAU / knives - 2.0 * sync, law };
    Cam::from_segments(&[zone, rest, zone]).ok()
}

pub fn build(cfg: &Config) -> Result<Box<dyn Machine>, ConfigError> {
    let joint = RotaryJoint::from_config(cfg)?;
    let (radius, knives) = (cfg.positive("knife.radius_m")?, cfg.positive("knife.knives")?);
    let sync = cfg.positive("knife.sync_angle_deg")?.to_radians();
    if knives.fract() != 0.0 || 2.0 * sync >= TAU / knives {
        return Err(ConfigError::Invalid("knife.sync_angle_deg".into()));
    }
    let law = cfg
        .get("knife.transition_law")?
        .as_str()
        .and_then(Law::from_name)
        .filter(|l| !matches!(l, Law::Dwell | Law::Linear))
        .ok_or_else(|| ConfigError::Invalid("knife.transition_law".into()))?;
    let cam = knife_cam(radius, knives, sync, cfg.positive("knife.format_length_m")?, law)
        .ok_or_else(|| ConfigError::Invalid("knife.format_length_m".into()))?;
    let master = master(cfg, cfg.positive("web.speed_m_s")?, cam.period)?;
    let knife = RotaryKnife {
        radius,
        knives,
        inertia: cfg.positive("knife.inertia_kgm2")?,
        bars: payload(cfg)?,
        attached: true,
        cut: if cfg.bool("effects.process_force")? {
            Some(Table::from_config(cfg, "cut.angle_deg", "cut.force_n")?)
        } else {
            None
        },
        cam,
        master,
    };
    let endless = knife.master.is_some();
    let phi = axis(cfg, "knife", "axis-knife", "rad", joint.range)?;
    machine(cfg, knife, (joint, None, "rad"), phi, endless)
}

impl RotaryKnife {
    /// The cut's force at the blade's tip at the roll's angle and speed, N.
    pub fn cut_force(&self, phi: f64, speed: f64) -> f64 {
        let Some(table) = &self.cut else { return 0.0 };
        let pitch = TAU / self.knives;
        let from_cut = phi - (phi / pitch).round() * pitch;
        table.at(from_cut.to_degrees()) * forwards(speed, 0.05)
    }
    fn j(&self) -> f64 {
        self.inertia + if self.attached { self.bars * self.radius * self.radius } else { 0.0 }
    }
}

impl Mechanism for RotaryKnife {
    fn dynamics(&self, _q: &[f64], _qd: &[f64], mass: &mut [f64], bias: &mut [f64]) {
        mass[0] = self.j();
        bias[0] = 0.0;
    }
    fn potential(&self, _q: &[f64]) -> f64 {
        0.0
    }
    fn set_payload_attached(&mut self, attached: bool) {
        self.attached = attached;
    }
    fn has_process(&self) -> bool {
        self.cut.is_some()
    }
    fn process_torque(&self, q: &[f64], qd: &[f64], out: &mut [f64]) {
        out[0] -= self.cut_force(q[0], qd[0]) * self.radius;
    }
    fn cam(&self) -> Option<CamMaster> {
        self.master
    }
    fn cam_at(&self, master: f64, out: &mut [[f64; 4]]) {
        out[0] = self.cam.at(master);
    }

    fn signal_infos(&self) -> Vec<SignalInfo> {
        vec![
            SignalInfo::new("knife.surface_speed", "group-knife", "quantity-surface-speed", "m/s"),
            SignalInfo::new("knife.cut_force", "group-process", "quantity-process-force", "N"),
        ]
    }
    fn signals(&self, q: &[f64], qd: &[f64], _qdd: &[f64], _force: &[f64], out: &mut Vec<f64>) {
        out.extend([self.radius * qd[0], self.cut_force(q[0], qd[0])]);
    }

    fn collision_parts(&self) -> &'static [&'static str] {
        &["knife-roll"]
    }

    /// Seen along the roll: the roll with its blades turning above the web, the anvil roll beneath, the motor behind.
    fn scene(&self, q: &[f64], rotors: &[f64], rotor_speeds: &[f64], out: &mut Scene) {
        let r = self.radius;
        let centre = [0.0, 0.0, r];
        out.parts.push(Part::new(
            "web",
            Shape::Box { size: [6.0 * r, 0.3, 0.004] },
            [0.0, 0.0, 0.0],
            [0.85, 0.82, 0.7],
        ));
        out.parts.push(Part::new(
            "anvil",
            Shape::Cylinder { radius: 0.8 * r, length: 0.3 },
            [0.0, 0.0, -0.8 * r],
            [0.45, 0.47, 0.5],
        ));
        out.parts.push(Part::new(
            "knife-roll",
            Shape::Cylinder { radius: 0.9 * r, length: 0.3 },
            centre,
            [0.55, 0.55, 0.6],
        ));
        for k in 0..self.knives as usize {
            // Blade k points down (at the cut) where phi = k pitch: the scene's angle -pi/2 turned by phi.
            let a = -std::f64::consts::FRAC_PI_2 + q[0] - k as f64 * TAU / self.knives;
            out.parts.push(
                Part::new(
                    "blade",
                    Shape::Box { size: [0.2 * r, 0.32, 0.01] },
                    [0.9 * r * a.cos(), 0.0, r + 0.9 * r * a.sin()],
                    [0.85, 0.85, 0.9],
                )
                .turned(a),
            );
        }
        out.parts.push(Part::new(
            sim_core::scene::MOTOR_PART,
            Shape::Cylinder { radius: 0.05, length: 0.12 },
            [0.0, 0.25, r],
            [0.9, 0.45, 0.1],
        ));
        out.parts.push(Part::new("gearbox", Shape::Box { size: [0.1, 0.06, 0.1] }, [0.0, 0.18, r], [0.25, 0.25, 0.28]));
        out.parts.push(
            Part::new("rotor-bar", Shape::Box { size: [0.07, 0.004, 0.012] }, [0.0, 0.32, r], [0.1, 0.1, 0.1])
                .turned(rotors[0]),
        );
        out.labels.push(Label {
            pos: [0.0, 0.25, 2.2 * r],
            owner_key: "motor-joint-1",
            value: rotor_speeds[0] * 60.0 / TAU,
            unit_key: "unit-rpm",
            decimals: 0,
        });
        out.labels.push(Label {
            pos: [1.5 * r, 0.0, 2.0 * r],
            owner_key: "axis-knife",
            value: q[0].to_degrees(),
            unit_key: "unit-deg",
            decimals: 1,
        });
    }
}
