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
    pub unit: String,
}

impl SignalInfo {
    pub fn new(id: impl Into<String>, owner_key: &str, quantity_key: &str, unit: &str) -> Self {
        Self { id: id.into(), owner_key: owner_key.into(), quantity_key: quantity_key.into(), unit: unit.into() }
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
    /// Gives a state at rest (see `equilibrium`) the tool speeds `vel`: everything that moves with the
    /// tool, motors included, moves along as the kinematics say.
    fn set_tool_velocity(&self, state: &mut [f64], vel: &[f64]);
    fn motor_state(&self, state: &[f64], pos: &mut [f64], vel: &mut [f64]);
    /// Motor torques from the rigid model for a tool pose and acceleration, including gravity.
    fn feedforward_torque(&self, tool_pos: &[f64], tool_acc: &[f64], out: &mut [f64]);
    /// Kinetic plus potential energy, for the conservation check.
    fn energy(&self, state: &[f64]) -> f64;
    fn set_payload_attached(&mut self, attached: bool);
    /// Told every step whether the cycle dwells at a target, for machines with a process force there.
    fn set_dwelling(&mut self, _dwelling: bool) {}
    /// Told every step whether the drives are disabled, for machines with holding brakes.
    fn set_brake_engaged(&mut self, _engaged: bool) {}

    /// The lowest structural resonance at a tool pose, for a controller that sets its input shaper by it.
    /// `None` where the machine has none to tell.
    fn resonance_hz(&self, _tool_pose: &[f64]) -> Option<f64> {
        None
    }

    /// Signals beyond axis and motor positions, speeds and torques, which every machine gets for free.
    fn signal_infos(&self) -> Vec<SignalInfo>;
    fn signals(&self, state: &[f64], torque: &[f64], out: &mut Vec<f64>);
    fn scene(&self, state: &[f64], out: &mut Scene);
    /// Where the scene draws the tool when it is at `tool_pose`, in scene coordinates, so that a front end can
    /// draw a path of the tool into the scene. `None` for a machine that has no such mapping.
    fn scene_point(&self, _tool_pose: &[f64]) -> Option<[f64; 3]> {
        None
    }

    /// Carries a running state over to a machine rebuilt from a changed config.
    fn adopt_state(&self, old: &dyn Machine, old_state: &[f64]) -> Vec<f64> {
        let (n, motors) = (self.axes().len(), self.motors().len());
        let (mut pos, mut vel) = (vec![0.0; n], vec![0.0; n]);
        old.tool_state(old_state, &mut pos, &mut vel);
        let (mut before, mut after) = (vec![0.0; motors], vec![0.0; motors]);
        old.inverse(&pos, &mut before);
        self.inverse(&pos, &mut after);
        if old.state_len() == self.state_len() && before == after {
            return old_state.to_vec();
        }
        // The model changed its states (rigid <-> elastic) or its kinematics (ratio, pulley radius), and the
        // old motor and pulley angles belong to another tool pose now: kept, they would tear the belt. So the
        // same tool pose and speed, with belts and gears as they are at rest there. The controller goes on
        // with its motion, so a machine restarted at rest would be torn after a setpoint that is already moving.
        let mut state = Vec::new();
        self.equilibrium(&pos, &mut state, &mut vec![0.0; motors]);
        self.set_tool_velocity(&mut state, &vel);
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
