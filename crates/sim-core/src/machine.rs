//! The machine interface: the one trait a kinematics has to implement.

use crate::{Config, ConfigError, Scene};

/// A tool-side axis. `key` is a translation key, `id` is used in signal ids and config paths.
#[derive(Debug, Clone)]
pub struct AxisInfo {
    pub id: &'static str,
    pub key: &'static str,
    pub unit: &'static str,
    pub min: f64,
    pub max: f64,
    pub max_vel: f64,
    pub max_acc: f64,
    pub max_jerk: f64,
}

#[derive(Debug, Clone)]
pub struct MotorInfo {
    pub id: &'static str,
    pub key: &'static str,
}

/// One scope trace and one PLC variable. The label is built from two translation keys.
#[derive(Debug, Clone, PartialEq)]
pub struct SignalInfo {
    pub id: String,
    pub owner_key: String,
    pub quantity_key: String,
    pub unit: &'static str,
}

impl SignalInfo {
    pub fn new(id: impl Into<String>, owner_key: &str, quantity_key: &str, unit: &'static str) -> Self {
        Self { id: id.into(), owner_key: owner_key.into(), quantity_key: quantity_key.into(), unit }
    }
}

pub trait Machine: Send {
    fn axes(&self) -> &[AxisInfo];
    fn motors(&self) -> &[MotorInfo];
    /// Tool pose to motor shaft angles, and back. Both are pure kinematics.
    fn inverse(&self, tool: &[f64], motors: &mut [f64]);
    fn forward(&self, motors: &[f64], tool: &mut [f64]);

    fn state_len(&self) -> usize;
    /// The state at rest at a tool pose, and the motor torques that hold it there.
    fn equilibrium(&self, tool: &[f64], state: &mut Vec<f64>, torque: &mut [f64]);
    fn derivatives(&self, state: &[f64], torque: &[f64], out: &mut [f64]);
    fn tool_state(&self, state: &[f64], pos: &mut [f64], vel: &mut [f64]);
    fn motor_state(&self, state: &[f64], pos: &mut [f64], vel: &mut [f64]);
    /// Motor torques from the rigid model for a tool pose and acceleration, including gravity.
    fn feedforward_torque(&self, tool_pos: &[f64], tool_acc: &[f64], out: &mut [f64]);
    /// Kinetic plus potential energy, for the conservation check.
    fn energy(&self, state: &[f64]) -> f64;
    fn set_payload_attached(&mut self, attached: bool);

    /// Signals beyond axis and motor positions, speeds and torques, which every machine gets for free.
    fn signal_infos(&self) -> Vec<SignalInfo>;
    fn signals(&self, state: &[f64], torque: &[f64], out: &mut Vec<f64>);
    fn scene(&self, state: &[f64], out: &mut Scene);

    /// Carries a running state over to a machine rebuilt from a changed config.
    fn adopt_state(&self, old: &dyn Machine, old_state: &[f64]) -> Vec<f64> {
        if old.state_len() == self.state_len() {
            return old_state.to_vec();
        }
        // The model changed its states (rigid <-> elastic): restart at rest at the same pose.
        let n = self.axes().len();
        let (mut pos, mut vel) = (vec![0.0; n], vec![0.0; n]);
        old.tool_state(old_state, &mut pos, &mut vel);
        let mut state = Vec::new();
        self.equilibrium(&pos, &mut state, &mut vec![0.0; self.motors().len()]);
        state
    }
}

/// One line of the registration list.
pub struct MachineEntry {
    pub id: &'static str,
    pub key: &'static str,
    pub default_config: &'static str,
    /// (language code, translation file contents)
    pub i18n: &'static [(&'static str, &'static str)],
    pub build: fn(&Config) -> Result<Box<dyn Machine>, ConfigError>,
    /// Configs without friction and other losses, on which the standard suite checks energy.
    pub suite_configs: fn() -> Vec<Config>,
}

impl MachineEntry {
    pub fn config(&self) -> Config {
        Config::parse(self.default_config).expect("default config of a registered machine must parse")
    }
}
