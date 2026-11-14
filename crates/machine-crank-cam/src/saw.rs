//! A flying saw (or flying shear): a carriage that carries the saw along the material, driven by a pinion on a rack
//! behind a gearbox. The joint is the carriage's travel x (m) along the material, from its end stop. The cam's master is
//! the material, in metres, at `material.speed_m_s`; one cut length L of material per cut.
//!
//! The cam over one cut length, from the carriage waiting at `saw.home_m`:
//! 1. synchronise: the carriage's speed rises from 0 to the material's while the material runs v^2 / a
//!    (`saw.sync_acceleration_m_s2` = a, the mean acceleration). The speed blends as 3 u^2 - 2 u^3 (no jump in
//!    acceleration, peak 1.5 a), so the carriage covers half of that, v^2 / (2 a): the synchronisation distance.
//! 2. cut: the carriage runs with the material over the cut window W (`saw.cut_window_m`), for W / v seconds.
//! 3. leave the material: the mirror of 1, again v^2 / (2 a) of carriage travel.
//! 4. return: back by the whole stroke W + v^2 / a to home, by `saw.return_law` (VDI 2143), over the rest of the cut
//!    length, L - 2 v^2 / a - W of material.
//!
//! The cut (effects.process_force) pushes on the carriage along the material with a table over the way into the window
//! (`cut.position_m`, `cut.force_n`, positive along the material) while the carriage moves forwards.

use crate::{Table, axis, forwards, machine, master, payload};
use sim_components::cam::{Cam, Law, Segment};
use sim_components::chain::Mechanism;
use sim_components::joint::RotaryJoint;
use sim_components::transmission::RackPinion;
use sim_core::{CamMaster, Config, ConfigError, Label, Machine, Part, RatingCheck, Scene, Shape, SignalInfo};
use std::f64::consts::TAU;

pub struct FlyingSaw {
    pub mass: f64,
    pub payload: f64,
    pub attached: bool,
    pub home: f64,
    /// The carriage's travel from home to the start of the window, v^2 / (2 a).
    pub sync_distance: f64,
    pub window: f64,
    pub stroke: f64,
    pub(crate) cut: Option<Table>,
    pub cam: Cam,
    pub master: Option<CamMaster>,
    rack: RackPinion,
    efficiency: f64,
}

/// The flying saw's cam (the module's): material at `speed`, mean synchronising acceleration `acc`, cut window `window`,
/// cut length `length`, `law` for the return. `None` where the cut length leaves no room for the return.
pub fn saw_cam(speed: f64, acc: f64, window: f64, length: f64, law: Law) -> Option<Cam> {
    let sync = speed * speed / acc;
    let rest = length - 2.0 * sync - window;
    if !(rest > 0.0) {
        return None;
    }
    Cam::from_segments(&[
        Segment { master: sync, slave: sync / 2.0, law: Law::Poly345 },
        Segment { master: window, slave: window, law: Law::Linear },
        Segment { master: sync, slave: sync / 2.0, law: Law::Poly345 },
        Segment { master: rest, slave: -(window + sync), law },
    ])
    .ok()
}

pub fn build(cfg: &Config) -> Result<Box<dyn Machine>, ConfigError> {
    let mut joint = RotaryJoint::linear_from_config(cfg)?;
    let rack = RackPinion::from_config(cfg)?;
    rack.converter().apply(cfg, &mut joint)?;
    let lever = rack.radius();
    let (speed, acc) = (cfg.positive("material.speed_m_s")?, cfg.positive("saw.sync_acceleration_m_s2")?);
    let window = cfg.positive("saw.cut_window_m")?;
    let law = cfg
        .get("saw.return_law")?
        .as_str()
        .and_then(Law::from_name)
        .filter(|l| !matches!(l, Law::Dwell | Law::Linear))
        .ok_or_else(|| ConfigError::Invalid("saw.return_law".into()))?;
    let cam = saw_cam(speed, acc, window, cfg.positive("saw.cut_length_m")?, law)
        .ok_or_else(|| ConfigError::Invalid("saw.cut_length_m".into()))?;
    let home = cfg.non_negative("saw.home_m")?;
    let stroke = window + speed * speed / acc;
    if home + stroke > joint.range.1 {
        return Err(ConfigError::Invalid("saw.home_m".into()));
    }
    let saw = FlyingSaw {
        mass: cfg.positive("mass.carriage_kg")?,
        payload: payload(cfg)?,
        attached: true,
        home,
        sync_distance: speed * speed / (2.0 * acc),
        window,
        stroke,
        cut: if cfg.bool("effects.process_force")? {
            Some(Table::from_config(cfg, "cut.position_m", "cut.force_n")?)
        } else {
            None
        },
        master: master(cfg, speed, cam.period)?,
        cam,
        rack,
        efficiency: joint.extra_efficiency,
    };
    let x = axis(cfg, "x", "axis-x", "m", joint.range)?;
    machine(cfg, saw, (joint, Some(lever), "m"), x, false)
}

impl FlyingSaw {
    fn moving(&self) -> f64 {
        self.mass + if self.attached { self.payload } else { 0.0 }
    }
    /// The cut's force on the carriage at its travel and speed, N, along the material.
    pub fn cut_force(&self, x: f64, speed: f64) -> f64 {
        let Some(table) = &self.cut else { return 0.0 };
        table.at(x - self.home - self.sync_distance) * forwards(speed, 1e-3)
    }
}

impl Mechanism for FlyingSaw {
    fn dynamics(&self, _q: &[f64], _qd: &[f64], mass: &mut [f64], bias: &mut [f64]) {
        mass[0] = self.moving();
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
        out[0] += self.cut_force(q[0], qd[0]);
    }
    fn cam(&self) -> Option<CamMaster> {
        self.master
    }
    /// The cam from home.
    fn cam_at(&self, master: f64, out: &mut [[f64; 4]]) {
        out[0] = self.cam.at(master);
        out[0][0] += self.home;
    }

    fn signal_infos(&self) -> Vec<SignalInfo> {
        vec![
            self.rack.signal_info("1", "joint-1"),
            SignalInfo::new("saw.cut_force", "group-process", "quantity-process-force", "N"),
        ]
    }
    fn signals(&self, q: &[f64], qd: &[f64], _qdd: &[f64], force: &[f64], out: &mut Vec<f64>) {
        out.push(RackPinion::feed_force(force[0], qd[0], self.efficiency));
        out.push(self.cut_force(q[0], qd[0]));
    }
    fn rating_checks(&self) -> Vec<RatingCheck> {
        self.rack.rating_checks("1", 0)
    }

    fn collision_parts(&self) -> &'static [&'static str] {
        &["carriage", "payload"]
    }

    /// From the front: the material running past on its rollers, the carriage with the saw above it on the rack, the
    /// window it cuts in marked on the frame.
    fn scene(&self, q: &[f64], rotors: &[f64], rotor_speeds: &[f64], out: &mut Scene) {
        let grey = [0.62, 0.64, 0.68];
        let end = self.home + self.stroke + 0.3;
        out.parts.push(Part::new(
            "frame",
            Shape::Segment { to: [end, 0.0, 0.25], width: 0.08 },
            [-0.3, 0.0, 0.25],
            grey,
        ));
        out.parts.push(Part::new(
            "rack",
            Shape::Segment { to: [end, -0.05, 0.3], width: 0.02 },
            [-0.3, -0.05, 0.3],
            [0.4, 0.4, 0.42],
        ));
        out.parts.push(Part::new(
            "material",
            Shape::Box { size: [end + 1.0, 0.2, 0.04] },
            [end / 2.0, 0.0, 0.0],
            [0.7, 0.7, 0.75],
        ));
        let start = self.home + self.sync_distance;
        out.parts.push(Part::new(
            "window",
            Shape::Box { size: [self.window, 0.01, 0.01] },
            [start + self.window / 2.0, -0.12, 0.25],
            [0.85, 0.3, 0.2],
        ));
        out.parts.push(Part::new(
            "carriage",
            Shape::Box { size: [0.25, 0.25, 0.08] },
            [q[0], 0.0, 0.35],
            [0.2, 0.22, 0.26],
        ));
        if self.attached && self.payload > 0.0 {
            out.parts.push(Part::new(
                "payload",
                Shape::Cylinder { radius: 0.12, length: 0.01 },
                [q[0], 0.0, 0.15],
                [0.85, 0.85, 0.9],
            ));
        }
        out.parts.push(Part::new(
            sim_core::scene::MOTOR_PART,
            Shape::Cylinder { radius: 0.05, length: 0.12 },
            [q[0], -0.2, 0.45],
            [0.9, 0.45, 0.1],
        ));
        out.parts.push(
            Part::new("rotor-bar", Shape::Box { size: [0.07, 0.004, 0.012] }, [q[0], -0.27, 0.45], [0.1, 0.1, 0.1])
                .turned(rotors[0]),
        );
        out.labels.push(Label {
            pos: [q[0], -0.2, 0.6],
            owner_key: "motor-joint-1",
            value: rotor_speeds[0] * 60.0 / TAU,
            unit_key: "unit-rpm",
            decimals: 0,
        });
        out.labels.push(Label {
            pos: [q[0], 0.0, 0.55],
            owner_key: "axis-x",
            value: q[0],
            unit_key: "unit-m",
            decimals: 3,
        });
    }
    fn scene_point(&self, tool: &[f64]) -> Option<[f64; 3]> {
        Some([tool[0], 0.0, 0.35])
    }
}
