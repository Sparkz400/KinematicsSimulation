//! A rotary indexing table about a vertical axis, angle c (rad): the table's and the fixture's inertia, the workpiece's
//! as payload, no gravity torque. Its gear (`table.gear`) is "planetary" (the `[gearbox]` as it is), "strain-wave"
//! (the `[gearbox]` plus the life of `[strain_wave]`) or "worm" (the `[gearbox]` with the efficiency `[worm]` gives).
//!
//! shortcut: a worm gear driven back from the table loses as `Gearbox::loss_factor` says (1 / efficiency), not by
//! its lower back efficiency, and never locks itself; model the self-locking if a cycle brakes the table through it.

use crate::{axis, machine, payload, required_life};
use sim_components::chain::Mechanism;
use sim_components::joint::RotaryJoint;
use sim_components::transmission::{StrainWave, WormGear};
use sim_core::{Config, ConfigError, Label, Machine, Part, RatingCheck, Scene, Shape, SignalInfo};
use std::f64::consts::TAU;

pub struct Table {
    inertia: f64,
    payload: f64,
    attached: bool,
    radius: f64,
    strain_wave: Option<(StrainWave, f64)>,
    ratio: f64,
}

pub fn build(cfg: &Config) -> Result<Box<dyn Machine>, ConfigError> {
    let mut joint = RotaryJoint::from_config(cfg)?;
    let mut strain_wave = None;
    match cfg.get("table.gear")?.as_str() {
        Some("planetary") => {}
        Some("strain-wave") => {
            strain_wave = Some((StrainWave::from_config(cfg)?, required_life(cfg).unwrap_or(f64::INFINITY)))
        }
        Some("worm") if cfg.bool("effects.gearbox_efficiency")? => {
            let worm = WormGear::from_config(cfg)?.efficiency();
            joint.extra_efficiency = worm / joint.gearbox.efficiency;
            joint.gearbox.efficiency = worm;
        }
        Some("worm") => {}
        _ => return Err(ConfigError::Invalid("table.gear".into())),
    }
    let table = Table {
        inertia: cfg.positive("table.inertia_kgm2")?,
        payload: payload(cfg, "table.payload_inertia_kgm2")?,
        attached: true,
        radius: cfg.positive("table.radius_m")?,
        strain_wave,
        ratio: joint.ratio(),
    };
    let c = axis(cfg, "c", "axis-c", "rad", joint.range)?;
    machine(cfg, table, vec![(joint, None, "rad")], vec![c])
}

impl Mechanism for Table {
    /// The table, fixture and workpiece turning about z, and the motor beneath it.
    // shortcut: a worm's motor lies across the table's axis; its rotor's spin is taken about z as the others', which
    // only the base's tilt (not its twist) would tell apart.
    fn frame_momentum(&self, _q: &[f64], qd: &[f64], rotor_spin: &[f64]) -> Option<sim_core::FrameMomentum> {
        let turning = self.inertia + if self.attached { self.payload } else { 0.0 };
        Some(sim_core::FrameMomentum { angular: [0.0, 0.0, turning * qd[0] + rotor_spin[0]], ..Default::default() })
    }
    fn dynamics(&self, _q: &[f64], _qd: &[f64], mass: &mut [f64], bias: &mut [f64]) {
        mass[0] = self.inertia + if self.attached { self.payload } else { 0.0 };
        bias[0] = 0.0;
    }
    fn potential(&self, _q: &[f64]) -> f64 {
        0.0
    }
    fn set_payload_attached(&mut self, attached: bool) {
        self.attached = attached;
    }

    fn signal_infos(&self) -> Vec<SignalInfo> {
        self.strain_wave.iter().map(|(g, _)| g.signal_info("1", "joint-1")).collect()
    }
    fn signals(&self, _q: &[f64], qd: &[f64], _qdd: &[f64], force: &[f64], out: &mut Vec<f64>) {
        if let Some((g, _)) = &self.strain_wave {
            out.push(g.signal(force[0], qd[0] * self.ratio));
        }
    }
    fn rating_checks(&self) -> Vec<RatingCheck> {
        self.strain_wave
            .iter()
            .filter(|(_, life)| life.is_finite())
            .map(|(g, life)| g.rating_check("1", 0, *life))
            .collect()
    }

    fn collision_parts(&self) -> &'static [&'static str] {
        &["fixture", "payload"]
    }

    /// The table seen from the front, its index mark turning, the fixture on it; the motor beneath it.
    fn scene(&self, q: &[f64], rotors: &[f64], rotor_speeds: &[f64], out: &mut Scene) {
        let r = self.radius;
        out.parts.push(Part::new(
            "base",
            Shape::Box { size: [2.2 * r, 2.2 * r, 0.1] },
            [0.0, 0.0, -0.1],
            [0.62, 0.64, 0.68],
        ));
        out.parts.push(Part::new(
            "table",
            Shape::Cylinder { radius: r, length: 0.04 },
            [0.0, 0.0, 0.25],
            [0.55, 0.55, 0.6],
        ));
        out.parts.push(
            Part::new("index-mark", Shape::Box { size: [1.8 * r, 0.004, 0.02] }, [0.0, -0.025, 0.25], [0.1, 0.1, 0.1])
                .turned(q[0]),
        );
        // The fixture as the box it turns within, for the collision with obstacles.
        out.parts.push(Part::new(
            "fixture",
            Shape::Box { size: [1.2 * r, 0.08, 0.1] },
            [0.0, 0.0, 0.32],
            [0.45, 0.45, 0.5],
        ));
        if self.attached && self.payload > 0.0 {
            out.parts.push(Part::new(
                "payload",
                Shape::Box { size: [0.4 * r, 0.06, 0.4 * r] },
                [0.6 * r * q[0].cos(), -0.05, 0.25 + 0.6 * r * q[0].sin()],
                [0.2, 0.7, 0.3],
            ));
        }
        out.parts.push(Part::new(
            sim_core::scene::MOTOR_PART,
            Shape::Cylinder { radius: 0.05, length: 0.12 },
            [r + 0.15, 0.0, 0.0],
            [0.9, 0.45, 0.1],
        ));
        out.parts.push(Part::new(
            "gearbox",
            Shape::Box { size: [0.1, 0.1, 0.1] },
            [r + 0.15, 0.0, 0.12],
            [0.25, 0.25, 0.28],
        ));
        out.parts.push(
            Part::new("rotor-bar", Shape::Box { size: [0.07, 0.004, 0.012] }, [r + 0.15, -0.07, 0.0], [0.1, 0.1, 0.1])
                .turned(rotors[0]),
        );
        out.labels.push(Label {
            pos: [r + 0.15, 0.0, -0.12],
            owner_key: "motor-joint-1",
            value: rotor_speeds[0] * 60.0 / TAU,
            unit_key: "unit-rpm",
            decimals: 0,
        });
        out.labels.push(Label {
            pos: [0.0, 0.0, 0.25 + 1.2 * r],
            owner_key: "axis-c",
            value: q[0].to_degrees(),
            unit_key: "unit-deg",
            decimals: 1,
        });
    }
}
