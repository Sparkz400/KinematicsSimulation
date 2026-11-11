//! T-gantry: transformation, belt routing, assembly, scene description.
//!
//! Two fixed motors A (left) and B (right) at the ends of the X beam drive one belt. The belt ends are
//! clamped at the top of the Z axis and it runs around an idler at the bottom of the Z axis:
//!
//! clamp -> carriage idler -> motor A -> carriage idler -> Z-bottom idler -> carriage idler -> motor B
//! -> carriage idler -> clamp
//!
//! With pulley angles a and b after the gearbox: x = r/2 (a + b), z = r/2 (a - b).
//! Both motors the same way moves X, against each other moves Z.
//!
//! The two motor pulleys cut the belt into three sections of uniform tension:
//!
//! | section | from -> to            | geometric length      | belt fed in by |
//! | ------- | --------------------- | --------------------- | -------------- |
//! | 1       | clamp -> motor A      | const + x + z         | + r a          |
//! | 2       | motor A -> motor B    | const - 2 z           | - r a + r b    |
//! | 3       | motor B -> clamp      | const - x + z         | - r b          |
//!
//! Belt forces: Fx = T3 - T1, Fz = 2 T2 - T1 - T3, pulley A: r (T1 - T2), pulley B: r (T2 - T3).

use sim_components::mech::{Belt, Gearbox, Lugre, Ripple, SectionForce, ShaftFriction, ToothMesh};
use sim_core::{AxisInfo, Config, ConfigError, Machine, MachineEntry, Label, MotorInfo, Part, Scene, Shape, SignalInfo};

pub static ENTRY: MachineEntry = MachineEntry {
    id: "t-gantry",
    key: "machine-t-gantry",
    default_config: include_str!("../default.toml"),
    i18n: &[("en", include_str!("../i18n/en.ftl")), ("de", include_str!("../i18n/de.ftl"))],
    build,
    suite_configs,
};

const G: f64 = 9.80665;

/// Effects that take energy out or put it in; the standard suite runs with them off.
pub const LOSSY_EFFECTS: [&str; 8] = [
    "belt_damping",
    "guide_friction",
    "load_dependent_friction",
    "motor_friction",
    "gearbox_efficiency",
    "torque_ripple",
    "tooth_meshing",
    "temperature",
];

fn build(cfg: &Config) -> Result<Box<dyn Machine>, ConfigError> {
    Ok(Box::new(TGantry::new(cfg)?))
}

/// The default config with every loss switched off.
pub fn lossless_config() -> Config {
    let mut cfg = ENTRY.config();
    for effect in LOSSY_EFFECTS {
        cfg.set_text(&format!("effects.{effect} = false")).unwrap();
    }
    cfg.set_text("gearbox.damping_nm_s_rad = 0.0").unwrap();
    cfg.set_text("frame.damping_n_s_m = 0.0").unwrap();
    cfg
}

/// Loss-free rigid reference and loss-free full model.
fn suite_configs() -> Vec<Config> {
    let mut rigid = lossless_config();
    rigid.set_text("effects.belt_stretch = false").unwrap();
    vec![rigid, lossless_config()]
}

pub struct TGantry {
    elastic: bool,
    r: f64,
    /// Unstretched belt fed per radian of pulley. The pulley radius holds at pretension, where the
    /// belt is already stretched, so that the kinematics are exact for an evenly tensioned belt.
    r_mat: f64,
    span_min: f64,
    z_com_drop: f64,
    guide_spacing: f64,
    m_carriage: f64,
    m_z_axis: f64,
    m_payload: f64,
    j_rotor: f64,
    /// Pulley inertia including its share of the belt mass.
    j_pulley: f64,
    g: f64,
    belt: Belt,
    /// Unstretched belt per section at x = z = 0 with the pulleys at angle 0.
    mat0: [f64; 3],
    friction_x: Lugre,
    friction_z: Lugre,
    shaft: ShaftFriction,
    ripple: Ripple,
    mesh: ToothMesh,
    gearbox: Gearbox,
    frame: Option<Frame>,
    payload_attached: bool,
    axes: [AxisInfo; 2],
    motors: [MotorInfo; 2],
}

struct Frame {
    mass: f64,
    stiffness: f64,
    damping: f64,
}

// Elastic state: positions [x, z, a, b, pa, pb, q], their rates, then the friction states [sx, sz].
// a, b: pulley angles. pa, pb: motor shaft angles. q: frame deflection in X; x is relative to the frame.
const X: usize = 0;
const Z: usize = 1;
const A: usize = 2;
const PA: usize = 4;
const Q: usize = 6;
const V: usize = 7;
const S: usize = 14;
const N_ELASTIC: usize = 16;
// Rigid state: [x, z, vx, vz, sx, sz].
const N_RIGID: usize = 6;

impl TGantry {
    pub fn new(cfg: &Config) -> Result<Self, ConfigError> {
        let axis = |id: &'static str, key: &'static str, stroke: f64| -> Result<AxisInfo, ConfigError> {
            Ok(AxisInfo {
                id,
                key,
                unit: "m",
                min: 0.0,
                max: stroke,
                max_vel: cfg.positive(&format!("axis.{id}.max_velocity_m_s"))?,
                max_acc: cfg.positive(&format!("axis.{id}.max_acceleration_m_s2"))?,
                max_jerk: cfg.positive(&format!("axis.{id}.max_jerk_m_s3"))?,
            })
        };
        let belt = Belt::from_config(cfg)?;
        let r = cfg.positive("geometry.pulley_radius_m")?;
        let frame = if cfg.bool("effects.frame_compliance")? {
            Some(Frame {
                mass: cfg.positive("frame.mass_kg")?,
                stiffness: cfg.positive("frame.stiffness_n_m")?,
                damping: cfg.f64("frame.damping_n_s_m")?,
            })
        } else {
            None
        };
        let mut m = Self {
            elastic: cfg.bool("effects.belt_stretch")?,
            r,
            r_mat: belt.rest_length(r),
            span_min: cfg.positive("geometry.span_min_m")?,
            z_com_drop: cfg.f64("geometry.z_com_drop_m")?,
            guide_spacing: cfg.positive("geometry.guide_spacing_m")?,
            m_carriage: cfg.positive("mass.carriage_kg")?,
            m_z_axis: cfg.positive("mass.z_axis_kg")?,
            m_payload: if cfg.bool("effects.payload")? { cfg.f64("mass.payload_kg")? } else { 0.0 },
            j_rotor: cfg.positive("motor.rotor_inertia_kgm2")?,
            j_pulley: cfg.positive("pulley.inertia_kgm2")?,
            g: if cfg.bool("effects.gravity")? { G } else { 0.0 },
            mat0: [0.0; 3],
            friction_x: Lugre::from_config(cfg, "friction_x")?,
            friction_z: Lugre::from_config(cfg, "friction_z")?,
            shaft: ShaftFriction::from_config(cfg)?,
            ripple: Ripple::from_config(cfg)?,
            mesh: ToothMesh::from_config(cfg)?,
            gearbox: Gearbox::from_config(cfg)?,
            frame,
            payload_attached: true,
            axes: [
                axis("x", "axis-x", cfg.positive("geometry.stroke_x_m")?)?,
                axis("z", "axis-z", cfg.positive("geometry.stroke_z_m")?)?,
            ],
            motors: [MotorInfo { id: "a", key: "motor-a" }, MotorInfo { id: "b", key: "motor-b" }],
            belt,
        };
        let geo = m.geo(0.0, 0.0);
        m.mat0 = geo.map(|g| m.belt.rest_length(g));
        // Belt mass: the belt runs at pulley surface speed, so half of it is lumped on each pulley.
        m.j_pulley += 0.5 * m.belt.mass_per_m * geo.iter().sum::<f64>() * r * r;
        Ok(m)
    }

    fn payload(&self) -> f64 {
        if self.payload_attached { self.m_payload } else { 0.0 }
    }
    /// Everything that moves in Z: the vertical axis and what it carries.
    fn m_z(&self) -> f64 {
        self.m_z_axis + self.payload()
    }
    /// Everything that moves in X: the carriage and the whole Z axis on it.
    fn m_x(&self) -> f64 {
        self.m_carriage + self.m_z()
    }
    /// Inertia per motor at the pulley: pulley plus rotor seen through the gearbox.
    fn j(&self) -> f64 {
        self.j_pulley + self.gearbox.ratio * self.gearbox.ratio * self.j_rotor
    }
    /// Both motors reflected onto one tool axis: 2 J / r^2.
    fn m_motors(&self) -> f64 {
        2.0 * self.j() / (self.r * self.r)
    }

    /// Geometric length of the three belt sections.
    fn geo(&self, x: f64, z: f64) -> [f64; 3] {
        let (sx, sz, s) = (self.axes[0].max, self.axes[1].max, self.span_min);
        [2.0 * s + x + z, 4.0 * s + sx + 2.0 * sz - 2.0 * z, 2.0 * s + sx - x + z]
    }

    /// Unstretched belt in the three sections for pulley angles a and b.
    fn mat(&self, a: f64, b: f64) -> [f64; 3] {
        let r = self.r_mat;
        [self.mat0[0] + r * a, self.mat0[1] - r * a + r * b, self.mat0[2] - r * b]
    }

    fn sections(&self, s: &[f64]) -> [SectionForce; 3] {
        let (geo, mat, r) = (self.geo(s[X], s[Z]), self.mat(s[A], s[A + 1]), self.r_mat);
        let (vx, vz, va, vb) = (s[V + X], s[V + Z], s[V + A], s[V + A + 1]);
        let geo_rate = [vx + vz, -2.0 * vz, vz - vx];
        let mat_rate = [r * va, r * (vb - va), -r * vb];
        std::array::from_fn(|i| self.belt.section(geo[i], mat[i], geo_rate[i], mat_rate[i]))
    }

    /// Friction level factors for the X and Z guides at an X acceleration of the Z axis.
    ///
    /// The X guide carries the weight and the tilting moment of the overhanging Z axis; the Z guide
    /// carries the side force that drags the Z axis along and the same moment.
    fn load_scales(&self, z: f64, ax: f64) -> (f64, f64) {
        let side = self.m_z() * ax;
        let tilt = 2.0 * side * (self.z_com_drop - z) / self.guide_spacing;
        let weight = |m: f64| m * self.g;
        (
            self.friction_x.load_scale(weight(self.m_x()) + tilt.abs(), weight(self.m_carriage + self.m_z_axis)),
            self.friction_z.load_scale(side.abs() + tilt.abs(), 0.0),
        )
    }

    /// Guide friction forces and friction state rates: (fx, fz, dsx, dsz).
    ///
    /// The guide load uses the X acceleration that the motor torques would give a loss-free rigid
    /// machine. The true acceleration would make friction depend on itself, and its absolute value
    /// would turn every belt vibration into extra friction.
    fn guide_friction(&self, z: f64, vx: f64, vz: f64, sx: f64, sz: f64, torque: &[f64]) -> (f64, f64, f64, f64) {
        let ax = self.gearbox.ratio * (torque[0] + torque[1]) / self.r / (self.m_x() + self.m_motors());
        let (scale_x, scale_z) = self.load_scales(z, ax);
        let (fx, dsx) = self.friction_x.eval(vx, sx, scale_x);
        let (fz, dsz) = self.friction_z.eval(vz, sz, scale_z);
        (fx, fz, dsx, dsz)
    }

    fn elastic_derivatives(&self, s: &[f64], torque: &[f64], out: &mut [f64]) {
        let r = self.r;
        let sec = self.sections(s);
        let fx_belt = sec[2].tension - sec[0].tension;
        let fz_belt = 2.0 * sec[1].tension - sec[0].tension - sec[2].tension;
        let (ffx, ffz, dsx, dsz) = self.guide_friction(s[Z], s[V + X], s[V + Z], s[S], s[S + 1], torque);
        out[..V].copy_from_slice(&s[V..S]);
        // Frame mode: belt and guide push back on the beam with the force they put on the carriage.
        let fx = fx_belt - ffx;
        let aq = match &self.frame {
            Some(f) => (-f.stiffness * s[Q] - f.damping * s[V + Q] - fx) / f.mass,
            None => {
                out[Q] = 0.0;
                0.0
            }
        };
        out[V + X] = fx / self.m_x() - aq;
        out[V + Z] = (fz_belt - ffz) / self.m_z() - self.g;
        out[V + Q] = aq;
        let belt_torque =
            [self.r_mat * (sec[0].material - sec[1].material), self.r_mat * (sec[1].material - sec[2].material)];
        for k in 0..2 {
            let (theta, w_theta, phi, w_phi) = (s[A + k], s[V + A + k], s[PA + k], s[V + PA + k]);
            let gear = self.gearbox.torque(phi, w_phi, theta, w_theta);
            out[V + A + k] = (gear.output + belt_torque[k] + r * self.mesh.force(r * theta)) / self.j_pulley;
            out[V + PA + k] =
                (self.ripple.apply(torque[k], phi) - gear.rotor - self.shaft.torque(w_phi)) / self.j_rotor;
        }
        out[S] = dsx;
        out[S + 1] = dsz;
    }

    /// Rigid model: accelerations, guide friction forces and friction state rates
    /// [ax, az, ffx, ffz, dsx, dsz].
    ///
    /// The gearbox loses torque only on what it transmits, which is the motor torque minus what
    /// accelerates the rotor itself. That depends on the acceleration being solved for; three passes
    /// settle it.
    fn rigid_solve(&self, s: &[f64], torque: &[f64]) -> [f64; 6] {
        let (r, ratio) = (self.r, self.gearbox.ratio);
        let (mut pos, mut vel) = ([0.0; 2], [0.0; 2]);
        self.inverse(&s[0..2], &mut pos);
        self.inverse(&s[2..4], &mut vel);
        // Belt force each motor would give with a loss-free gearbox and a massless rotor.
        let drive: [f64; 2] =
            std::array::from_fn(|k| ratio * (self.ripple.apply(torque[k], pos[k]) - self.shaft.torque(vel[k])) / r);
        let mesh: [f64; 2] = std::array::from_fn(|k| self.mesh.force(r * pos[k] / ratio));
        let rotor = ratio * ratio * self.j_rotor / (r * r);
        let pulleys = 2.0 * self.j_pulley / (r * r);
        let (mx, mz) = (self.m_x() + pulleys, self.m_z() + pulleys);
        let (ffx, ffz, dsx, dsz) = self.guide_friction(s[1], s[2], s[3], s[4], s[5], torque);
        let (mut loss, mut out) = ([1.0; 2], [0.0; 6]);
        for _ in 0..3 {
            let bx = loss[0] * drive[0] + loss[1] * drive[1] + mesh[0] + mesh[1] - ffx;
            let bz = loss[0] * drive[0] - loss[1] * drive[1] + mesh[0] - mesh[1] - ffz - self.m_z() * self.g;
            // (mx + p) ax + q az = bx,  q ax + (mz + p) az = bz
            let (p, q) = (rotor * (loss[0] + loss[1]), rotor * (loss[0] - loss[1]));
            let det = (mx + p) * (mz + p) - q * q;
            let (ax, az) = ((bx * (mz + p) - q * bz) / det, ((mx + p) * bz - q * bx) / det);
            let through_gear = [drive[0] - rotor * (ax + az), drive[1] - rotor * (ax - az)];
            loss = std::array::from_fn(|k| self.gearbox.loss_factor(through_gear[k] * r, vel[k] / ratio));
            out = [ax, az, ffx, ffz, dsx, dsz];
        }
        out
    }

    fn rigid_derivatives(&self, s: &[f64], torque: &[f64], out: &mut [f64]) {
        let [ax, az, _, _, dsx, dsz] = self.rigid_solve(s, torque);
        out.copy_from_slice(&[s[2], s[3], ax, az, dsx, dsz]);
    }
}

impl Machine for TGantry {
    fn axes(&self) -> &[AxisInfo] {
        &self.axes
    }
    fn motors(&self) -> &[MotorInfo] {
        &self.motors
    }

    fn inverse(&self, tool: &[f64], motors: &mut [f64]) {
        let k = self.gearbox.ratio / self.r;
        motors[0] = k * (tool[0] + tool[1]);
        motors[1] = k * (tool[0] - tool[1]);
    }
    fn forward(&self, motors: &[f64], tool: &mut [f64]) {
        let k = self.r / (2.0 * self.gearbox.ratio);
        tool[0] = k * (motors[0] + motors[1]);
        tool[1] = k * (motors[0] - motors[1]);
    }

    fn state_len(&self) -> usize {
        if self.elastic { N_ELASTIC } else { N_RIGID }
    }

    fn equilibrium(&self, tool: &[f64], state: &mut Vec<f64>, torque: &mut [f64]) {
        let (x, z, r, ratio) = (tool[0], tool[1], self.r, self.gearbox.ratio);
        if !self.elastic {
            *state = vec![x, z, 0.0, 0.0, 0.0, 0.0];
            let mut angle = [0.0; 2];
            self.inverse(tool, &mut angle);
            let lift = self.m_z() * self.g * r / 2.0;
            for (k, at_pulley) in [lift, -lift].into_iter().enumerate() {
                let shaft = (at_pulley - r * self.mesh.force(r * angle[k] / ratio)) / ratio;
                torque[k] = self.ripple.invert(shaft, angle[k]);
            }
            return;
        }
        // X balance: T1 = T3. Z balance: 2 T2 - T1 - T3 = m_z g. The belt total fixes the level.
        let geo = self.geo(x, z);
        let mat = self.belt.static_material(&geo, &[0.0, self.m_z() * self.g / 2.0, 0.0], self.mat0.iter().sum());
        let (a, b) = ((mat[0] - self.mat0[0]) / self.r_mat, (self.mat0[2] - mat[2]) / self.r_mat);
        let m: [f64; 3] = std::array::from_fn(|i| self.belt.section(geo[i], mat[i], 0.0, 0.0).material);
        *state = vec![0.0; N_ELASTIC];
        state[X] = x;
        state[Z] = z;
        for (k, (theta, belt_torque)) in [(a, self.r_mat * (m[0] - m[1])), (b, self.r_mat * (m[1] - m[2]))].into_iter().enumerate() {
            let gear = -belt_torque - r * self.mesh.force(r * theta);
            let phi = ratio * (theta + self.gearbox.static_twist(gear));
            state[A + k] = theta;
            state[PA + k] = phi;
            torque[k] = self.ripple.invert(gear / ratio, phi);
        }
    }

    fn derivatives(&self, state: &[f64], torque: &[f64], out: &mut [f64]) {
        if self.elastic { self.elastic_derivatives(state, torque, out) } else { self.rigid_derivatives(state, torque, out) }
    }

    fn tool_state(&self, s: &[f64], pos: &mut [f64], vel: &mut [f64]) {
        let v = if self.elastic { V } else { 2 };
        pos.copy_from_slice(&s[0..2]);
        vel.copy_from_slice(&s[v..v + 2]);
    }
    fn motor_state(&self, s: &[f64], pos: &mut [f64], vel: &mut [f64]) {
        if self.elastic {
            pos.copy_from_slice(&s[PA..PA + 2]);
            vel.copy_from_slice(&s[V + PA..V + PA + 2]);
        } else {
            self.inverse(&s[0..2], pos);
            self.inverse(&s[2..4], vel);
        }
    }

    fn feedforward_torque(&self, _pos: &[f64], acc: &[f64], out: &mut [f64]) {
        let fx = (self.m_x() + self.m_motors()) * acc[0];
        let fz = (self.m_z() + self.m_motors()) * acc[1] + self.m_z() * self.g;
        let k = self.r / (2.0 * self.gearbox.ratio);
        out[0] = k * (fx + fz);
        out[1] = k * (fx - fz);
    }

    fn energy(&self, s: &[f64]) -> f64 {
        if !self.elastic {
            return 0.5 * (self.m_x() + self.m_motors()) * s[2] * s[2]
                + 0.5 * (self.m_z() + self.m_motors()) * s[3] * s[3]
                + self.m_z() * self.g * s[1];
        }
        let (geo, mat) = (self.geo(s[X], s[Z]), self.mat(s[A], s[A + 1]));
        let belt: f64 = (0..3).map(|i| self.belt.energy(geo[i], mat[i])).sum();
        let motors: f64 = (0..2)
            .map(|k| {
                0.5 * self.j_pulley * s[V + A + k].powi(2)
                    + 0.5 * self.j_rotor * s[V + PA + k].powi(2)
                    + self.gearbox.energy(s[PA + k], s[A + k])
            })
            .sum();
        let frame = self.frame.as_ref().map_or(0.0, |f| 0.5 * f.mass * s[V + Q].powi(2) + 0.5 * f.stiffness * s[Q].powi(2));
        0.5 * self.m_x() * (s[V + X] + s[V + Q]).powi(2)
            + 0.5 * self.m_z() * s[V + Z].powi(2)
            + self.m_z() * self.g * s[Z]
            + belt
            + motors
            + frame
    }

    fn set_payload_attached(&mut self, attached: bool) {
        self.payload_attached = attached;
    }

    fn signal_infos(&self) -> Vec<SignalInfo> {
        let mut out: Vec<SignalInfo> = (1..=3)
            .map(|i| SignalInfo::new(format!("belt.{i}.tension"), &format!("belt-section-{i}"), "quantity-tension", "N"))
            .collect();
        for axis in &self.axes {
            out.push(SignalInfo::new(format!("axis.{}.belt_stretch", axis.id), axis.key, "quantity-belt-stretch", "m"));
            out.push(SignalInfo::new(format!("axis.{}.friction", axis.id), axis.key, "quantity-friction-force", "N"));
        }
        out.push(SignalInfo::new("frame.deflection", "frame", "quantity-deflection", "m"));
        out
    }

    /// The rigid model has no belt stretch: it reports pretension, zero stretch and zero deflection.
    fn signals(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        if !self.elastic {
            let [_, _, ffx, ffz, ..] = self.rigid_solve(s, torque);
            out.extend([self.belt.pretension; 3]);
            out.extend([0.0, ffx, 0.0, ffz, 0.0]);
            return;
        }
        let sec = self.sections(s);
        let (ffx, ffz, ..) = self.guide_friction(s[Z], s[V + X], s[V + Z], s[S], s[S + 1], torque);
        let (a, b) = (s[A], s[A + 1]);
        out.extend(sec.iter().map(|f| f.tension));
        out.extend([s[X] - self.r / 2.0 * (a + b), ffx, s[Z] - self.r / 2.0 * (a - b), ffz, s[Q]]);
    }

    fn scene(&self, s: &[f64], out: &mut Scene) {
        use sim_core::scene::{belt, drive_unit, idler, on_circle};
        use std::f64::consts::{FRAC_PI_4, TAU};
        let (x, z, r, ratio) = (s[0], s[1], self.r, self.gearbox.ratio);
        let (sx, sz, span) = (self.axes[0].max, self.axes[1].max, self.span_min);
        let (mut rotor, mut rotor_speed) = ([0.0; 2], [0.0; 2]);
        self.motor_state(s, &mut rotor, &mut rotor_speed);
        let pulley = if self.elastic { [s[A], s[A + 1]] } else { rotor.map(|p| p / ratio) };
        // Belt colour: tension relative to pretension, per section.
        let tension = if self.elastic { self.sections(s).map(|f| f.tension) } else { [self.belt.pretension; 3] };
        let shade = tension.map(|t| ((t - self.belt.pretension) / self.belt.pretension.max(1.0)).clamp(-1.0, 1.0) as f32);

        // The belt runs in one plane in front of the beam; the motors sit behind it at both ends.
        // The carriage rides on the beam at height 0; the Z axis hangs through it and rises with z.
        let y = -0.05;
        let (end, cx, half) = (2.0 * span + sx, span + x, 0.03);
        let (left, right, top, bottom) = (cx - half, cx + half, span + z, z - sz - span);
        let floor = -(sz + 2.0 * span + 0.2);
        let part = Part::new;
        out.labels.clear();
        out.parts.clear();
        out.parts.extend([
            part("beam", Shape::Box { size: [end - 0.14, 0.07, 0.09] }, [end / 2.0, 0.045, 0.0], [0.62, 0.64, 0.68]),
            part("post", Shape::Box { size: [0.06, 0.06, -0.045 - floor] }, [0.13, 0.045, (floor - 0.045) / 2.0], [0.45, 0.47, 0.5]),
            part("post", Shape::Box { size: [0.06, 0.06, -0.045 - floor] }, [end - 0.13, 0.045, (floor - 0.045) / 2.0], [0.45, 0.47, 0.5]),
            part("carriage", Shape::Box { size: [0.16, 0.02, 0.16] }, [cx, -0.02, 0.0], [0.2, 0.22, 0.26]),
            part("z-axis", Shape::Box { size: [0.04, 0.03, top - bottom] }, [cx, y, (top + bottom) / 2.0], [0.78, 0.8, 0.84]),
            part("clamp", Shape::Box { size: [0.09, 0.035, 0.03] }, [cx, y, top + 0.015], [0.85, 0.3, 0.2]),
            part("flange", Shape::Box { size: [0.07, 0.05, 0.02] }, [cx, y, bottom - half - 0.012], [0.3, 0.3, 0.33]),
        ]);
        if self.payload() > 0.0 {
            out.parts.push(part("payload", Shape::Box { size: [0.1, 0.1, 0.1] }, [cx, y, bottom - half - 0.072], [0.2, 0.7, 0.3]));
        }
        // Positive pulley angles turn clockwise seen from the front (the top of the pulley moves right).
        drive_unit(&mut out.parts, [0.0, 0.0], y, r, -pulley[0], -rotor[0], [0.9, 0.45, 0.1]);
        drive_unit(&mut out.parts, [end, 0.0], y, r, -pulley[1], -rotor[1], [0.1, 0.45, 0.9]);
        // Carriage idlers turn the belt from the beam into the Z axis; the idler at the bottom of
        // the Z axis turns it around. Each rolls with the belt that passes it.
        let ri = 0.012;
        for (side, belt_travel) in [(-1.0, x + z), (1.0, z - x)] {
            let ix = cx + side * (half + ri);
            idler(&mut out.parts, [ix, r + ri], y, ri, side * belt_travel / ri);
            idler(&mut out.parts, [ix, -r - ri], y, ri, side * belt_travel / ri);
        }
        idler(&mut out.parts, [cx, bottom], y, half - 0.002, -(x - 2.0 * z) / half);

        // The belt path from the clamp on the left to the clamp on the right. Its length does not
        // depend on the pose, so teeth at fixed belt length travel around the pulleys by themselves.
        let (a, b, bend) = ([0.0, 0.0], [end, 0.0], [cx, bottom]);
        let path = [
            [left, top],
            [left, r],
            [0.0, r],
            on_circle(a, r, 3.0 * FRAC_PI_4),
            on_circle(a, r, 4.0 * FRAC_PI_4),
            on_circle(a, r, 5.0 * FRAC_PI_4),
            [0.0, -r],
            [left, -r],
            [left, bottom],
            on_circle(bend, half, 5.0 * FRAC_PI_4),
            on_circle(bend, half, 6.0 * FRAC_PI_4),
            on_circle(bend, half, 7.0 * FRAC_PI_4),
            [right, bottom],
            [right, -r],
            [end, -r],
            on_circle(b, r, -FRAC_PI_4),
            on_circle(b, r, 0.0),
            on_circle(b, r, FRAC_PI_4),
            [end, r],
            [right, r],
            [right, top],
        ];
        // Strips up to the middle of each motor pulley belong to the section before it.
        belt(&mut out.parts, &path, y, 0.025, 0.04, |i| shade[if i < 4 { 0 } else if i < 16 { 1 } else { 2 }]);

        let rpm = |speed: f64| speed * 60.0 / TAU;
        out.labels.extend([
            Label { pos: [0.0, y, 0.11], owner_key: self.motors[0].key, value: rpm(rotor_speed[0]), unit_key: "unit-rpm", decimals: 0 },
            Label { pos: [end, y, 0.11], owner_key: self.motors[1].key, value: rpm(rotor_speed[1]), unit_key: "unit-rpm", decimals: 0 },
            Label { pos: [cx, y, top + 0.08], owner_key: self.axes[0].key, value: x, unit_key: "unit-m", decimals: 3 },
            Label { pos: [cx + 0.12, y, bottom], owner_key: self.axes[1].key, value: z, unit_key: "unit-m", decimals: 3 },
        ]);
    }
}

#[cfg(test)]
mod tests;
