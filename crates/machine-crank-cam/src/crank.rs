//! A slider-crank: a crank of radius r turned by the motor through its gearbox, a connecting rod of length l, a slider
//! on a straight guide through the crank's centre, as in mechanical and servo presses, pushers and pick-and-place by
//! crank. The joint is the crank angle theta (rad), 0 where the slider is furthest out (the outer dead centre, a press's
//! bottom dead centre). The slider's distance from the crank's centre:
//!
//!   x(theta) = r cos(theta) + sqrt(l^2 - r^2 sin^2(theta)),
//!   x' = -r sin(theta) - r^2 sin(theta) cos(theta) / w,    w = sqrt(l^2 - r^2 sin^2(theta)),
//!   x'' = -r cos(theta) - r^2 cos(2 theta) / w - r^4 sin^2(theta) cos^2(theta) / w^3.
//!
//! The rod's mass is split into a share at the crank pin (`rod.crank_end_share`) and the rest at the slider. The load's
//! inertia at the crank depends on the angle: J(theta) = J_crank + m_pin r^2 + m_x x'^2, m_x slider, die (payload) and the
//! rod's slider end; its speed term is m_x x' x'' theta'^2 (dJ/dtheta theta'^2 / 2). Gravity, the guide inclined by
//! `slider.incline_deg` (from the crank towards the slider, -90 for a ram below the crank): V = g (m_x x sin(beta) +
//! m_pin r sin(theta + beta)).
//!
//! The process force (effects.process_force) is a table over the slider's distance from the outer dead centre
//! (`process.distance_m`, `process.force_n`): it pushes back on the slider while it moves outwards, as forming does, and
//! not on the way back (springback and stripping left out). At the crank it is the torque -F x'(theta).
//!
//! Signals: the slider's position, speed and acceleration, the process force, the rod force, the guide's load across
//! (the rod's side push and the weight across), the crank bearing's load and the inertia J(theta) at the crank; the
//! rod force, the guide's and the bearing's load are judged against `ratings.rod_max_force_n`,
//! `ratings.slider_guide_max_load_n` and `ratings.crank_bearing_max_load_n`.

use crate::{Table, axis, forwards, gravity, machine, master, payload};
use sim_components::cam::Cam;
use sim_components::chain::Mechanism;
use sim_components::joint::RotaryJoint;
use sim_core::{CamMaster, Config, ConfigError, Label, Machine, Part, RatingCheck, Scene, Shape, SignalInfo};
use std::f64::consts::TAU;

pub struct SliderCrank {
    pub r: f64,
    pub l: f64,
    /// The crank's own inertia about its axis, kg m^2.
    pub j_crank: f64,
    /// The slider and the rod's slider end, the die on it (payload) apart.
    pub m_slider: f64,
    pub payload: f64,
    pub attached: bool,
    /// The rod's crank end, at the crank pin.
    pub m_pin: f64,
    pub g: f64,
    /// The guide's incline from the crank towards the slider, rad.
    pub beta: f64,
    pub(crate) process: Option<Table>,
    /// The process force fades in from rest to this slider speed, m/s.
    pub smoothing: f64,
    pub cam: Option<(Cam, CamMaster)>,
    limits: [f64; 3],
}

pub fn build(cfg: &Config) -> Result<Box<dyn Machine>, ConfigError> {
    let joint = RotaryJoint::from_config(cfg)?;
    let crank = SliderCrank::from_config(cfg)?;
    let endless = crank.cam.as_ref().is_some_and(|(c, _)| c.advance != 0.0);
    let theta = axis(cfg, "crank", "axis-crank", "rad", joint.range)?;
    machine(cfg, crank, (joint, None, "rad"), theta, endless)
}

impl SliderCrank {
    pub fn from_config(cfg: &Config) -> Result<Self, ConfigError> {
        let (r, l) = (cfg.positive("crank.radius_m")?, cfg.positive("rod.length_m")?);
        if l <= r {
            return Err(ConfigError::Invalid("rod.length_m".into()));
        }
        let share = cfg.non_negative("rod.crank_end_share")?;
        if share > 1.0 {
            return Err(ConfigError::Invalid("rod.crank_end_share".into()));
        }
        let rod = cfg.non_negative("rod.mass_kg")?;
        let incline = cfg.f64("slider.incline_deg")?;
        if !(-90.0..=90.0).contains(&incline) {
            return Err(ConfigError::Invalid("slider.incline_deg".into()));
        }
        let cam = crank_cam(cfg)?;
        let master = master(cfg, cam.period * cfg.positive("crank.strokes_per_min")? / 60.0, cam.period)?;
        Ok(SliderCrank {
            r,
            l,
            j_crank: cfg.positive("crank.inertia_kgm2")?,
            m_slider: cfg.positive("slider.mass_kg")? + (1.0 - share) * rod,
            payload: payload(cfg)?,
            attached: true,
            m_pin: share * rod,
            g: gravity(cfg)?,
            beta: incline.to_radians(),
            process: if cfg.bool("effects.process_force")? {
                Some(Table::from_config(cfg, "process.distance_m", "process.force_n")?)
            } else {
                None
            },
            smoothing: cfg.positive("process.smoothing_m_s")?,
            limits: ["rod_max_force_n", "slider_guide_max_load_n", "crank_bearing_max_load_n"]
                .map(|k| cfg.f64(&format!("ratings.{k}")).unwrap_or(f64::INFINITY)),
            cam: master.map(|m| (cam, m)),
        })
    }
}

/// The crank's cam: one linear segment over a turn (constant speed, `crank.motion = "constant"`), or `[cam]`'s.
fn crank_cam(cfg: &Config) -> Result<Cam, ConfigError> {
    match cfg.get("crank.motion")?.as_str() {
        Some("constant") => Ok(Cam::from_segments(&[sim_components::cam::Segment {
            master: TAU,
            slave: TAU,
            law: sim_components::cam::Law::Linear,
        }])
        .expect("one turn at constant speed")),
        Some("cam") => Cam::from_config(cfg, "cam"),
        _ => Err(ConfigError::Invalid("crank.motion".into())),
    }
}

impl SliderCrank {
    /// x, x' and x'' at the crank angle theta (the module's formulas).
    pub fn slider(&self, theta: f64) -> [f64; 3] {
        let (r, l) = (self.r, self.l);
        let (s, c) = theta.sin_cos();
        let w = (l * l - r * r * s * s).sqrt();
        [
            r * c + w,
            -r * s - r * r * s * c / w,
            -r * c - r * r * (c * c - s * s) / w - r.powi(4) * s * s * c * c / w.powi(3),
        ]
    }

    fn m_x(&self) -> f64 {
        self.m_slider + if self.attached { self.payload } else { 0.0 }
    }

    /// J(theta): what the crank feels of crank, pin and slider.
    pub fn inertia(&self, theta: f64) -> f64 {
        let x1 = self.slider(theta)[1];
        self.j_crank + self.m_pin * self.r * self.r + self.m_x() * x1 * x1
    }

    /// The process force on the slider at the angle and speed, N, pushing it back towards the crank.
    pub fn process_force(&self, theta: f64, speed: f64) -> f64 {
        let Some(table) = &self.process else { return 0.0 };
        let [x, x1, _] = self.slider(theta);
        table.at(self.r + self.l - x) * forwards(x1 * speed, self.smoothing)
    }

    /// The rod's force (tension positive), the guide's load across and the crank bearing's load, N, at the crank's
    /// angle, speed and acceleration. Slider along x: -T cos(phi) + F_x = m_x x'', F_x the process force and the weight
    /// along; across: N + T sin(phi) - m_x g cos(beta) = 0, sin(phi) = r sin(theta) / l. The bearing holds the pin's
    /// mass on its circle against gravity and the rod's pull T (x - r cos, -r sin) / l.
    pub fn loads(&self, theta: f64, speed: f64, acc: f64) -> [f64; 3] {
        let (r, l, m) = (self.r, self.l, self.m_x());
        let [x, x1, x2] = self.slider(theta);
        let (s, c) = theta.sin_cos();
        let (sb, cb) = self.beta.sin_cos();
        let slider_acc = x2 * speed * speed + x1 * acc;
        let along = -self.process_force(theta, speed) - m * self.g * sb;
        let w = x - r * c;
        let tension = (along - m * slider_acc) * l / w;
        let guide = (m * self.g * cb - tension * r * s / l).abs();
        let pin_acc = [r * (-acc * s - speed * speed * c), r * (acc * c - speed * speed * s)];
        let rod = [tension * w / l, -tension * r * s / l];
        let gravity = [-self.g * sb, -self.g * cb];
        let bearing: Vec<f64> = (0..2).map(|k| self.m_pin * (pin_acc[k] - gravity[k]) - rod[k]).collect();
        [tension, guide, bearing[0].hypot(bearing[1])]
    }
}

impl Mechanism for SliderCrank {
    fn dynamics(&self, q: &[f64], qd: &[f64], mass: &mut [f64], bias: &mut [f64]) {
        let [_, x1, x2] = self.slider(q[0]);
        mass[0] = self.inertia(q[0]);
        bias[0] = self.m_x() * x1 * x2 * qd[0] * qd[0]
            + self.g * (self.m_x() * x1 * self.beta.sin() + self.m_pin * self.r * (q[0] + self.beta).cos());
    }
    fn potential(&self, q: &[f64]) -> f64 {
        self.g * (self.m_x() * self.slider(q[0])[0] * self.beta.sin() + self.m_pin * self.r * (q[0] + self.beta).sin())
    }
    fn set_payload_attached(&mut self, attached: bool) {
        self.attached = attached;
    }
    /// The speed term of J(theta) needs the feedforward's speed terms.
    fn tool_is_joints(&self) -> bool {
        false
    }
    fn has_process(&self) -> bool {
        self.process.is_some()
    }
    fn process_torque(&self, q: &[f64], qd: &[f64], out: &mut [f64]) {
        out[0] -= self.process_force(q[0], qd[0]) * self.slider(q[0])[1];
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
            ("slider.position", "group-slider", "quantity-position", "m"),
            ("slider.speed", "group-slider", "quantity-speed", "m/s"),
            ("slider.acceleration", "group-slider", "quantity-acceleration", "m/s²"),
            ("slider.process_force", "group-process", "quantity-process-force", "N"),
            ("rod.force", "group-rod", "quantity-rod-force", "N"),
            ("slider.guide_load", "group-slider", "quantity-guide-load", "N"),
            ("crank.bearing_load", "group-crank", "quantity-bearing-load", "N"),
            ("crank.inertia", "group-crank", "quantity-reflected-inertia", "kg m²"),
        ]
        .into_iter()
        .map(|(id, owner, quantity, unit)| SignalInfo::new(id, owner, quantity, unit))
        .collect()
    }
    fn signals(&self, q: &[f64], qd: &[f64], qdd: &[f64], _force: &[f64], out: &mut Vec<f64>) {
        let [x, x1, x2] = self.slider(q[0]);
        let [rod, guide, bearing] = self.loads(q[0], qd[0], qdd[0]);
        out.extend([
            x,
            x1 * qd[0],
            x2 * qd[0] * qd[0] + x1 * qdd[0],
            self.process_force(q[0], qd[0]),
            rod,
            guide,
            bearing,
            self.inertia(q[0]),
        ]);
    }
    fn rating_checks(&self) -> Vec<RatingCheck> {
        [
            ("rod-force", "rod.force"),
            ("slider-guide-load", "slider.guide_load"),
            ("crank-bearing-load", "crank.bearing_load"),
        ]
        .into_iter()
        .zip(self.limits)
        .filter(|(_, limit)| limit.is_finite())
        .map(|((check, signal), limit)| RatingCheck { check, signal: signal.into(), motor: None, limit, life: false })
        .collect()
    }

    fn collision_parts(&self) -> &'static [&'static str] {
        &["slider", "payload"]
    }

    /// Seen from the front: the crank disc at the origin with its pin, the rod to the slider on its guide, inclined as
    /// configured; the motor behind the crank.
    fn scene(&self, q: &[f64], rotors: &[f64], rotor_speeds: &[f64], out: &mut Scene) {
        let (sb, cb) = self.beta.sin_cos();
        // Guide coordinates (along, across) to the scene's x and z.
        let at = |a: f64, b: f64| [a * cb - b * sb, 0.0, a * sb + b * cb];
        let (r, l, theta) = (self.r, self.l, q[0]);
        let x = self.slider(theta)[0];
        let pin = at(r * theta.cos(), r * theta.sin());
        let grey = [0.62, 0.64, 0.68];
        out.parts.push(Part::new(
            "frame",
            Shape::Segment { to: at(r + l + 0.1, -0.06), width: 0.06 },
            at(l - r - 0.1, -0.06),
            grey,
        ));
        out.parts.push(Part::new(
            "crank",
            Shape::Cylinder { radius: 1.25 * r, length: 0.03 },
            [0.0; 3],
            [0.45, 0.47, 0.5],
        ));
        out.parts.push(
            Part::new(
                "crank-arm",
                Shape::Box { size: [r, 0.004, 0.2 * r] },
                [0.5 * pin[0], -0.02, 0.5 * pin[2]],
                [0.1, 0.1, 0.1],
            )
            .turned(theta + self.beta),
        );
        out.parts.push(Part::new("rod", Shape::Segment { to: at(x, 0.0), width: 0.03 }, pin, [0.75, 0.75, 0.78]));
        out.parts.push(Part::new("slider", Shape::Box { size: [0.12, 0.1, 0.12] }, at(x, 0.0), [0.2, 0.22, 0.26]));
        if self.attached && self.payload > 0.0 {
            out.parts.push(Part::new(
                "payload",
                Shape::Box { size: [0.06, 0.08, 0.1] },
                at(x + 0.09, 0.0),
                [0.2, 0.7, 0.3],
            ));
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
            pos: [0.0, 0.0, 1.25 * r + 0.08],
            owner_key: "motor-joint-1",
            value: rotor_speeds[0] * 60.0 / TAU,
            unit_key: "unit-rpm",
            decimals: 0,
        });
        out.labels.push(Label {
            pos: at(x, 0.15),
            owner_key: "group-slider",
            value: self.r + self.l - x,
            unit_key: "unit-m",
            decimals: 4,
        });
    }
    fn scene_point(&self, tool: &[f64]) -> Option<[f64; 3]> {
        let (sb, cb) = self.beta.sin_cos();
        let x = self.slider(tool[0])[0];
        Some([x * cb, 0.0, x * sb])
    }
}
