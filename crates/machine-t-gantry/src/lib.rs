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

use sim_core::{AxisInfo, Config, ConfigError, Machine, MachineEntry, MotorInfo, Part, Scene, Shape, SignalInfo};

pub static ENTRY: MachineEntry = MachineEntry {
    id: "t-gantry",
    key: "machine-t-gantry",
    default_config: include_str!("../default.toml"),
    i18n: &[("en", include_str!("../i18n/en.ftl")), ("de", include_str!("../i18n/de.ftl"))],
    build,
    suite_configs,
};

const G: f64 = 9.80665;

fn build(cfg: &Config) -> Result<Box<dyn Machine>, ConfigError> {
    Ok(Box::new(TGantry::new(cfg)?))
}

fn suite_configs() -> Vec<Config> {
    vec![ENTRY.config()]
}

pub struct TGantry {
    r: f64,
    m_carriage: f64,
    m_z_axis: f64,
    m_payload: f64,
    /// Inertia per motor at the pulley: pulley plus rotor seen through the gearbox.
    j: f64,
    ratio: f64,
    g: f64,
    payload_attached: bool,
    axes: [AxisInfo; 2],
    motors: [MotorInfo; 2],
}

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
        let ratio = cfg.positive("gearbox.ratio")?;
        Ok(Self {
            r: cfg.positive("geometry.pulley_radius_m")?,
            m_carriage: cfg.positive("mass.carriage_kg")?,
            m_z_axis: cfg.positive("mass.z_axis_kg")?,
            m_payload: if cfg.bool("effects.payload")? { cfg.f64("mass.payload_kg")? } else { 0.0 },
            j: cfg.positive("pulley.inertia_kgm2")? + ratio * ratio * cfg.positive("motor.rotor_inertia_kgm2")?,
            ratio,
            g: if cfg.bool("effects.gravity")? { G } else { 0.0 },
            payload_attached: true,
            axes: [
                axis("x", "axis-x", cfg.positive("geometry.stroke_x_m")?)?,
                axis("z", "axis-z", cfg.positive("geometry.stroke_z_m")?)?,
            ],
            motors: [MotorInfo { id: "a", key: "motor-a" }, MotorInfo { id: "b", key: "motor-b" }],
        })
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
    /// Both motors reflected onto one tool axis: 2 J / r^2.
    fn m_motors(&self) -> f64 {
        2.0 * self.j / (self.r * self.r)
    }
}

// Rigid state: [x, z, vx, vz].
impl Machine for TGantry {
    fn axes(&self) -> &[AxisInfo] {
        &self.axes
    }
    fn motors(&self) -> &[MotorInfo] {
        &self.motors
    }

    fn inverse(&self, tool: &[f64], motors: &mut [f64]) {
        motors[0] = self.ratio * (tool[0] + tool[1]) / self.r;
        motors[1] = self.ratio * (tool[0] - tool[1]) / self.r;
    }
    fn forward(&self, motors: &[f64], tool: &mut [f64]) {
        let k = self.r / (2.0 * self.ratio);
        tool[0] = k * (motors[0] + motors[1]);
        tool[1] = k * (motors[0] - motors[1]);
    }

    fn state_len(&self) -> usize {
        4
    }

    fn equilibrium(&self, tool: &[f64], state: &mut Vec<f64>, torque: &mut [f64]) {
        *state = vec![tool[0], tool[1], 0.0, 0.0];
        self.feedforward_torque(tool, &[0.0, 0.0], torque);
    }

    fn derivatives(&self, s: &[f64], torque: &[f64], out: &mut [f64]) {
        let (ta, tb) = (self.ratio * torque[0], self.ratio * torque[1]);
        let fx = (ta + tb) / self.r;
        let fz = (ta - tb) / self.r;
        out[0] = s[2];
        out[1] = s[3];
        out[2] = fx / (self.m_x() + self.m_motors());
        out[3] = (fz - self.m_z() * self.g) / (self.m_z() + self.m_motors());
    }

    fn tool_state(&self, s: &[f64], pos: &mut [f64], vel: &mut [f64]) {
        pos.copy_from_slice(&s[0..2]);
        vel.copy_from_slice(&s[2..4]);
    }
    fn motor_state(&self, s: &[f64], pos: &mut [f64], vel: &mut [f64]) {
        self.inverse(&s[0..2], pos);
        self.inverse(&s[2..4], vel);
    }

    fn feedforward_torque(&self, _pos: &[f64], acc: &[f64], out: &mut [f64]) {
        let fx = (self.m_x() + self.m_motors()) * acc[0];
        let fz = (self.m_z() + self.m_motors()) * acc[1] + self.m_z() * self.g;
        let k = self.r / (2.0 * self.ratio);
        out[0] = k * (fx + fz);
        out[1] = k * (fx - fz);
    }

    fn energy(&self, s: &[f64]) -> f64 {
        0.5 * (self.m_x() + self.m_motors()) * s[2] * s[2]
            + 0.5 * (self.m_z() + self.m_motors()) * s[3] * s[3]
            + self.m_z() * self.g * s[1]
    }

    fn set_payload_attached(&mut self, attached: bool) {
        self.payload_attached = attached;
    }

    fn signal_infos(&self) -> Vec<SignalInfo> {
        Vec::new()
    }
    fn signals(&self, _state: &[f64], _torque: &[f64], _out: &mut Vec<f64>) {}

    fn scene(&self, s: &[f64], out: &mut Scene) {
        let (x, z) = (s[0], s[1]);
        let (sx, sz) = (self.axes[0].max, self.axes[1].max);
        // The carriage runs on the beam at height 0; the Z axis hangs through it and rises with z.
        let (x0, cw) = (0.15, 0.16);
        let cx = x0 + x;
        let z_len = sz + 0.3;
        let z_mid = z - sz + z_len / 2.0 - 0.1;
        let belt = [0.1, 0.1, 0.1];
        let part = |id, shape, pos, color| Part { id, shape, pos, color, highlight: 0.0 };
        let span = |id, from: [f64; 2], to: [f64; 2]| {
            part(id, Shape::Segment { to: [to[0], -0.05, to[1]], width: 0.025 }, [from[0], -0.05, from[1]], belt)
        };
        let (top, bottom) = (z_mid + z_len / 2.0, z_mid - z_len / 2.0);
        let (left, right, end) = (cx - cw / 2.0, cx + cw / 2.0, 2.0 * x0 + sx);
        out.parts.clear();
        out.parts.extend([
            part("beam", Shape::Box { size: [end, 0.08, 0.08] }, [end / 2.0, 0.04, 0.0], [0.6, 0.6, 0.65]),
            part("motor-a", Shape::Cylinder { radius: 0.04, length: 0.12 }, [0.0, 0.0, 0.0], [0.9, 0.45, 0.1]),
            part("motor-b", Shape::Cylinder { radius: 0.04, length: 0.12 }, [end, 0.0, 0.0], [0.1, 0.45, 0.9]),
            part("carriage", Shape::Box { size: [cw, 0.06, 0.16] }, [cx, -0.03, 0.0], [0.3, 0.3, 0.35]),
            part("z-axis", Shape::Box { size: [0.06, 0.06, z_len] }, [cx, -0.09, z_mid], [0.75, 0.75, 0.8]),
            span("belt-1", [left, top], [left, 0.02]),
            span("belt-2", [left, 0.02], [0.0, 0.02]),
            span("belt-3", [0.0, -0.02], [left, -0.02]),
            span("belt-4", [left, -0.02], [left, bottom]),
            span("belt-5", [right, bottom], [right, -0.02]),
            span("belt-6", [right, -0.02], [end, -0.02]),
            span("belt-7", [end, 0.02], [right, 0.02]),
            span("belt-8", [right, 0.02], [right, top]),
        ]);
        if self.payload() > 0.0 {
            out.parts.push(part("payload", Shape::Box { size: [0.1, 0.1, 0.1] }, [cx, -0.09, bottom - 0.05], [0.2, 0.7, 0.3]));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_core::{Rk4, advance};

    fn machine(edit: &[&str]) -> TGantry {
        let mut cfg = ENTRY.config();
        for e in edit {
            cfg.set_text(e).unwrap();
        }
        TGantry::new(&cfg).unwrap()
    }

    #[test]
    fn kinematics_follow_the_plan() {
        let m = machine(&["gearbox.ratio = 1.0"]);
        let mut tool = [0.0; 2];
        // Both motors the same way: X only. Against each other: Z only.
        m.forward(&[1.0, 1.0], &mut tool);
        assert!((tool[0] - m.r).abs() < 1e-15 && tool[1].abs() < 1e-15);
        m.forward(&[1.0, -1.0], &mut tool);
        assert!(tool[0].abs() < 1e-15 && (tool[1] - m.r).abs() < 1e-15);
    }

    #[test]
    fn static_hold_on_z_needs_opposite_torques() {
        // Each motor holds m_z g r / 2 at the pulley, with opposite signs.
        let m = machine(&[]);
        let (mut state, mut hold) = (Vec::new(), [0.0; 2]);
        m.equilibrium(&[0.5, 0.25], &mut state, &mut hold);
        let expected = (5.0 + 2.0) * G * 0.02 / 2.0 / 5.0;
        assert!((hold[0] - expected).abs() < 1e-12, "{hold:?}");
        assert!((hold[1] + expected).abs() < 1e-12, "{hold:?}");
        let mut d = [0.0; 4];
        m.derivatives(&state, &hold, &mut d);
        assert!(d[2].abs() < 1e-12 && d[3].abs() < 1e-12, "{d:?}");
    }

    #[test]
    fn constant_torque_gives_force_over_effective_mass() {
        let m = machine(&[]);
        let (mut state, mut hold) = (Vec::new(), [0.0; 2]);
        m.equilibrium(&[0.1, 0.1], &mut state, &mut hold);
        let mut rk = Rk4::new(4);
        // Extra torque: 0.1 N m on both for X, and +-0.05 N m for Z on top.
        let torque = [hold[0] + 0.1 + 0.05, hold[1] + 0.1 - 0.05];
        let (dt, t_end) = (62.5e-6, 0.25);
        for _ in 0..(t_end / dt) as usize {
            advance(&m, &mut rk, &mut state, &torque, dt, 4);
        }
        let m_motors = 2.0 * (1.0e-4 + 25.0 * 0.3e-4) / (0.02 * 0.02);
        let ax = (2.0 * 0.1 * 5.0 / 0.02) / (11.0 + m_motors);
        let az = (2.0 * 0.05 * 5.0 / 0.02) / (7.0 + m_motors);
        assert!((state[0] - 0.1 - 0.5 * ax * t_end * t_end).abs() < 1e-9, "x {}", state[0]);
        assert!((state[1] - 0.1 - 0.5 * az * t_end * t_end).abs() < 1e-9, "z {}", state[1]);
        assert!((state[2] - ax * t_end).abs() < 1e-9 && (state[3] - az * t_end).abs() < 1e-9);
    }

    #[test]
    fn payload_switch_and_release_change_the_load() {
        let mut m = machine(&[]);
        assert_eq!(m.m_z(), 7.0);
        m.set_payload_attached(false);
        assert_eq!(m.m_z(), 5.0);
        assert_eq!(machine(&["effects.payload = false"]).m_z(), 5.0);
    }
}
