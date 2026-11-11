//! Single-axis test machine: a motor at the left end drives a carriage through a belt loop.
//! x = r * pulley angle. Component checks run here once and then hold for every machine.

use sim_core::{AxisInfo, Config, ConfigError, Machine, MachineEntry, MotorInfo, Part, Scene, Shape, SignalInfo};

pub static ENTRY: MachineEntry = MachineEntry {
    id: "single-axis",
    key: "machine-single-axis",
    default_config: include_str!("../default.toml"),
    i18n: &[("en", include_str!("../i18n/en.ftl")), ("de", include_str!("../i18n/de.ftl"))],
    build,
    suite_configs,
};

const G: f64 = 9.80665;

fn build(cfg: &Config) -> Result<Box<dyn Machine>, ConfigError> {
    Ok(Box::new(SingleAxis::new(cfg)?))
}

fn suite_configs() -> Vec<Config> {
    // Inclined, so that gravity takes part in the energy and static-hold checks.
    let mut inclined = ENTRY.config();
    inclined.set_text("geometry.incline_deg = 30.0").unwrap();
    vec![ENTRY.config(), inclined]
}

pub struct SingleAxis {
    r: f64,
    m_carriage: f64,
    m_payload: f64,
    /// Inertia at the pulley: pulley plus rotor seen through the gearbox.
    j: f64,
    ratio: f64,
    /// Gravity along the axis, positive pulling towards -x.
    g_along: f64,
    payload_attached: bool,
    axes: [AxisInfo; 1],
    motors: [MotorInfo; 1],
}

impl SingleAxis {
    pub fn new(cfg: &Config) -> Result<Self, ConfigError> {
        let ratio = cfg.positive("gearbox.ratio")?;
        let g = if cfg.bool("effects.gravity")? { G } else { 0.0 };
        Ok(Self {
            r: cfg.positive("geometry.pulley_radius_m")?,
            m_carriage: cfg.positive("mass.carriage_kg")?,
            m_payload: if cfg.bool("effects.payload")? { cfg.f64("mass.payload_kg")? } else { 0.0 },
            j: cfg.positive("pulley.inertia_kgm2")? + ratio * ratio * cfg.positive("motor.rotor_inertia_kgm2")?,
            ratio,
            g_along: g * cfg.f64("geometry.incline_deg")?.to_radians().sin(),
            payload_attached: true,
            axes: [AxisInfo {
                id: "x",
                key: "axis-x",
                unit: "m",
                min: 0.0,
                max: cfg.positive("geometry.stroke_m")?,
                max_vel: cfg.positive("axis.x.max_velocity_m_s")?,
                max_acc: cfg.positive("axis.x.max_acceleration_m_s2")?,
                max_jerk: cfg.positive("axis.x.max_jerk_m_s3")?,
            }],
            motors: [MotorInfo { id: "m", key: "motor-m" }],
        })
    }

    fn m(&self) -> f64 {
        self.m_carriage + if self.payload_attached { self.m_payload } else { 0.0 }
    }
    fn m_total(&self) -> f64 {
        self.m() + self.j / (self.r * self.r)
    }
}

// Rigid state: [x, v].
impl Machine for SingleAxis {
    fn axes(&self) -> &[AxisInfo] {
        &self.axes
    }
    fn motors(&self) -> &[MotorInfo] {
        &self.motors
    }

    fn inverse(&self, tool: &[f64], motors: &mut [f64]) {
        motors[0] = self.ratio * tool[0] / self.r;
    }
    fn forward(&self, motors: &[f64], tool: &mut [f64]) {
        tool[0] = motors[0] * self.r / self.ratio;
    }

    fn state_len(&self) -> usize {
        2
    }

    fn equilibrium(&self, tool: &[f64], state: &mut Vec<f64>, torque: &mut [f64]) {
        *state = vec![tool[0], 0.0];
        self.feedforward_torque(tool, &[0.0], torque);
    }

    fn derivatives(&self, s: &[f64], torque: &[f64], out: &mut [f64]) {
        out[0] = s[1];
        out[1] = (self.ratio * torque[0] / self.r - self.m() * self.g_along) / self.m_total();
    }

    fn tool_state(&self, s: &[f64], pos: &mut [f64], vel: &mut [f64]) {
        pos[0] = s[0];
        vel[0] = s[1];
    }
    fn motor_state(&self, s: &[f64], pos: &mut [f64], vel: &mut [f64]) {
        self.inverse(&s[0..1], pos);
        self.inverse(&s[1..2], vel);
    }

    fn feedforward_torque(&self, _pos: &[f64], acc: &[f64], out: &mut [f64]) {
        out[0] = (self.m_total() * acc[0] + self.m() * self.g_along) * self.r / self.ratio;
    }

    fn energy(&self, s: &[f64]) -> f64 {
        0.5 * self.m_total() * s[1] * s[1] + self.m() * self.g_along * s[0]
    }

    fn set_payload_attached(&mut self, attached: bool) {
        self.payload_attached = attached;
    }

    fn signal_infos(&self) -> Vec<SignalInfo> {
        Vec::new()
    }
    fn signals(&self, _state: &[f64], _torque: &[f64], _out: &mut Vec<f64>) {}

    fn scene(&self, s: &[f64], out: &mut Scene) {
        let (x0, end) = (0.15, 0.3 + self.axes[0].max);
        let cx = x0 + s[0];
        let belt = [0.1, 0.1, 0.1];
        let part = |id, shape, pos, color| Part { id, shape, pos, color, highlight: 0.0 };
        let span = |id, from: f64, to: f64, z: f64| {
            part(id, Shape::Segment { to: [to, -0.05, z], width: 0.025 }, [from, -0.05, z], belt)
        };
        out.parts.clear();
        out.parts.extend([
            part("beam", Shape::Box { size: [end, 0.08, 0.08] }, [end / 2.0, 0.04, 0.0], [0.6, 0.6, 0.65]),
            part("motor", Shape::Cylinder { radius: 0.04, length: 0.12 }, [0.0, 0.0, 0.0], [0.9, 0.45, 0.1]),
            part("idler", Shape::Cylinder { radius: 0.02, length: 0.04 }, [end, -0.05, 0.0], [0.5, 0.5, 0.5]),
            part("carriage", Shape::Box { size: [0.16, 0.06, 0.1] }, [cx, -0.03, 0.05], [0.3, 0.3, 0.35]),
            span("belt-1", 0.0, cx, 0.02),
            span("belt-2", cx, end, 0.02),
            span("belt-3", end, 0.0, -0.02),
        ]);
        if self.payload_attached && self.m_payload > 0.0 {
            out.parts.push(part("payload", Shape::Box { size: [0.1, 0.1, 0.1] }, [cx, -0.03, 0.15], [0.2, 0.7, 0.3]));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_core::{Rk4, advance};

    #[test]
    fn constant_torque_gives_force_over_effective_mass() {
        let m = SingleAxis::new(&ENTRY.config()).unwrap();
        let mut state = vec![0.1, 0.0];
        let mut rk = Rk4::new(2);
        let (dt, t_end) = (62.5e-6, 0.25);
        for _ in 0..(t_end / dt) as usize {
            advance(&m, &mut rk, &mut state, &[0.2], dt, 4);
        }
        let a = (0.2 * 5.0 / 0.02) / (6.0 + (1.0e-4 + 25.0 * 0.3e-4) / (0.02 * 0.02));
        assert!((state[0] - 0.1 - 0.5 * a * t_end * t_end).abs() < 1e-9, "x {}", state[0]);
        assert!((state[1] - a * t_end).abs() < 1e-9);
    }

    #[test]
    fn inclined_axis_needs_holding_torque() {
        let mut cfg = ENTRY.config();
        cfg.set_text("geometry.incline_deg = 90.0").unwrap();
        let m = SingleAxis::new(&cfg).unwrap();
        let (mut state, mut hold) = (Vec::new(), [0.0]);
        m.equilibrium(&[0.5], &mut state, &mut hold);
        assert!((hold[0] - 6.0 * G * 0.02 / 5.0).abs() < 1e-12);
    }
}
