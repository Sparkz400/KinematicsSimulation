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

/// The parts of a motor's torque that a machine can report as the signals `motor.<id>.torque_<part>` (N m), with
/// the quantity keys `quantity-torque-<part>`: what accelerates the moved masses and the rotors, what holds them
/// against gravity, what friction takes (guides, shafts, gearbox losses, idlers, cable chain), and the rest (belt
/// elasticity and vibration, ripple, tooth meshing, process force, end stops). The rest is the motor torque less
/// the other three, so the four add up to the motor torque.
pub const TORQUE_PARTS: [&str; 4] = ["inertia", "gravity", "friction", "rest"];
/// Signals whose id starts with this are powers lost (W) in one place of the machine: a friction contact, a damper,
/// the gearbox. Together with the energy signals they close the machine's energy balance: the work of the motor
/// torques equals the change of `energy.*` plus the time integral of every `power.*`.
pub const LOSS_PREFIX: &str = "power.";
/// The machine's energy split, as the signals `energy.<part>` (J), quantity keys `quantity-energy-<part>`. They add up
/// to `Machine::energy`.
pub const ENERGY_PARTS: [&str; 3] = ["kinetic", "potential", "elastic"];
/// What the signals of a friction contact end with, after its id (`<id>.friction_<part>`): its sliding speed, its
/// force split into Coulomb, viscous and stiction parts (they add up to the signal `<id>.friction`), and the factor
/// its levels are scaled with by the load.
pub const FRICTION_SIGNALS: [&str; 5] = ["speed", "coulomb", "viscous", "stiction", "load_scale"];

/// A friction contact with a Coulomb, viscous and Stribeck model, for the friction view: the id its signals start
/// with (see `FRICTION_SIGNALS`) and who it belongs to, as a translation key.
#[derive(Debug, Clone, PartialEq)]
pub struct FrictionContact {
    pub id: &'static str,
    pub owner_key: &'static str,
}

/// A belt section for the belt view: the id its signals start with, `<id>.tension` (N) and `<id>.stretch` (m, its
/// length less that of its belt unstretched), who it is as a translation key, and the tension it has at rest.
#[derive(Debug, Clone, PartialEq)]
pub struct BeltSection {
    pub id: &'static str,
    pub owner_key: &'static str,
    pub pretension: f64,
}

/// The kinds of part that can warm with their loss power (`ThermalPart`); each has the config keys
/// `thermal.<kind>_resistance_k_w`, `thermal.<kind>_time_constant_s` and `thermal.<kind>_max_rise_k`, and the quantity
/// key `quantity-<kind>-temperature`.
pub const THERMAL_KINDS: [&str; 2] = ["gearbox", "guide"];
/// What the simulation calls the temperature rise of a `ThermalPart` (`temperature.<id>`, K).
pub const TEMPERATURE_PREFIX: &str = "temperature.";

/// A part that warms with one of the machine's loss powers, `power.<id>`, with effects.part_heating: of which kind
/// (`THERMAL_KINDS`), who it belongs to as a translation key, and which part of the scene shows it: the
/// `scene_index`-th part with the id `scene_part`.
#[derive(Debug, Clone, PartialEq)]
pub struct ThermalPart {
    pub id: &'static str,
    pub kind: &'static str,
    pub owner_key: &'static str,
    pub scene_part: &'static str,
    pub scene_index: usize,
}

/// A rating of one of the machine's own parts that the cycle is judged against (`Machine::rating_checks`), beside the
/// checks every machine gets from its motors, drives, gearbox, guides and belts. The analysis follows the signal `signal`
/// (one of `signal_infos`, among the base signals) over the measured pass.
#[derive(Debug, Clone, PartialEq)]
pub struct RatingCheck {
    /// The verdict's id; its label is the translation key `check-<check>`.
    pub check: &'static str,
    pub signal: String,
    pub motor: Option<usize>,
    pub limit: f64,
    /// `false`: the highest |signal| over the pass has to stay at or below `limit`. `true`: the signal is a damage
    /// rate, the share of the part's life used per hour, and the life in hours, 1 / its mean over the pass (Palmgren
    /// and Miner), has to reach `limit`; such a check's id ends in "-life" and is one of `analysis::AT_LEAST`.
    pub life: bool,
}

/// A structural mode of a machine at a pose, from `Machine::modes`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Mode {
    /// Its undamped natural frequency, Hz.
    pub hz: f64,
    /// Its damping ratio; 0 where the machine tells none.
    pub damping: f64,
}

pub trait Machine: Send {
    /// The ratings of the machine's own parts (ball screw, rack, strain wave gear, ...) that its cycle is judged
    /// against. Empty where it has none beyond the common ones.
    fn rating_checks(&self) -> Vec<RatingCheck> {
        Vec::new()
    }

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
    /// `feedforward_torque` with the tool's speed too, for what the speed asks of the motors in a kinematics that is
    /// not linear (the centripetal and Coriolis part of the torque). The controller uses this one; the default leaves
    /// the speed out, which is exact where the motor angles are a linear function of the tool pose.
    fn feedforward_torque_moving(&self, tool_pos: &[f64], _tool_vel: &[f64], tool_acc: &[f64], out: &mut [f64]) {
        self.feedforward_torque(tool_pos, tool_acc, out);
    }
    /// The motor torques of the same rigid, loss-free model at motor angles, speeds and accelerations: what fitting a
    /// trace in the motors' own coordinates needs (`identify::fit_joints`). By default through the tool: its pose by
    /// `forward`, its speed and acceleration by central differences along the motors' motion over ±0.1 ms, then
    /// `feedforward_torque_moving`. A machine whose `feedforward_torque_moving` leaves out the speed terms of a
    /// kinematics that is not linear overrides this.
    fn rigid_motor_torque(&self, angle: &[f64], speed: &[f64], acc: &[f64], out: &mut [f64]) {
        let (h, n) = (1e-4, self.axes().len());
        let at = |s: f64| {
            let motors: Vec<f64> = (0..angle.len()).map(|k| angle[k] + s * speed[k] + 0.5 * s * s * acc[k]).collect();
            let mut tool = vec![0.0; n];
            self.forward(&motors, &mut tool);
            tool
        };
        let (before, pos, after) = (at(-h), at(0.0), at(h));
        let vel: Vec<f64> = (0..n).map(|i| (after[i] - before[i]) / (2.0 * h)).collect();
        let tool_acc: Vec<f64> = (0..n).map(|i| (after[i] - 2.0 * pos[i] + before[i]) / (h * h)).collect();
        self.feedforward_torque_moving(&pos, &vel, &tool_acc, out);
    }
    /// The tension of each belt section (N, in the order of `belt_sections`) that the loss-free rigid model of
    /// `feedforward_torque` gives at a tool pose and acceleration: what the belt has to pull on the moved masses (not
    /// on the pulleys and rotors, which the motors turn), shared out so that the belt keeps its length. Linear in the
    /// acceleration. Appends nothing where the machine has no belt sections to tell.
    fn rigid_tensions(&self, _tool_pos: &[f64], _tool_acc: &[f64], _out: &mut Vec<f64>) {}
    /// The load of each guide that reports `axis.<id>.guide_load` (N, in that order) in the loss-free rigid model at a
    /// tool pose and acceleration, as [rest, moving] for the load rest + |moving|: `moving` linear in the acceleration
    /// and zero without. The time-optimal plan keeps it within `ratings.guide_max_load_n`. Appends nothing where the
    /// machine does not tell.
    fn rigid_guide_loads(&self, _tool_pos: &[f64], _tool_acc: &[f64], _out: &mut Vec<[f64; 2]>) {}
    /// Kinetic plus potential energy, for the conservation check.
    fn energy(&self, state: &[f64]) -> f64;
    fn set_payload_attached(&mut self, attached: bool);
    /// The rigid bodies whose inertial parameters a trace can be fitted for (`identify::fit_joints`), by name: the links
    /// of a URDF chain. None by default.
    fn links(&self) -> Vec<String> {
        Vec::new()
    }
    /// Changes link `link`'s inertial parameters about its joint's frame, from those it was built with: mass, first
    /// moment and inertia all times `factor`, then `moment` (kg m, in its frame) added to the first moment with mass
    /// and inertia about the frame kept. The rigid torques are linear in each, which the fit takes its columns from.
    /// (1, [0; 3]) restores it. Nothing by default.
    fn set_link_change(&mut self, _link: usize, _factor: f64, _moment: [f64; 3]) {}
    /// Told every step whether the cycle dwells at a target, for machines with a process force there.
    fn set_dwelling(&mut self, _dwelling: bool) {}
    /// Told every step whether the drives are disabled, for machines with holding brakes.
    fn set_brake_engaged(&mut self, _engaged: bool) {}
    /// Called after every base step, for what happens at once and not by integration (a belt tooth that jumps):
    /// the machine may change `state` and must report what that took out of its energy among its loss powers
    /// over this step of `dt` seconds, so that the energy budget closes.
    fn after_step(&mut self, _state: &mut [f64], _dt: f64) {}
    /// With effects.thermal_feedback: the temperature rise (K) of each part of `thermal_parts`, in that order, for
    /// the friction and losses that change with it.
    fn set_part_temperatures(&mut self, _rises: &[f64]) {}

    /// The lowest structural resonance at a tool pose, for a controller that sets its input shaper by it.
    /// `None` where the machine has none to tell.
    fn resonance_hz(&self, _tool_pose: &[f64]) -> Option<f64> {
        None
    }
    /// The mode `resonance_hz` reports at a tool pose: its frequency in Hz and its shape, a direction in the state
    /// (as long as it, positions only, rates zero). The equilibrium at that pose plus a small amplitude times the
    /// shape, drawn with `scene`, shows how the machine moves in that mode; one unit of amplitude moves the tool by
    /// 1 m along its first axis. `None` where the machine tells no resonance.
    fn mode_shape(&self, _tool_pose: &[f64]) -> Option<(f64, Vec<f64>)> {
        None
    }
    /// Every structural mode the machine knows at a tool pose, lowest first: for a second input shaper, the resonance
    /// over an axis's stroke (`sim-cli modes`) and the app's plot of it. By default the one of `resonance_hz`, its
    /// damping unknown (0).
    fn modes(&self, tool_pose: &[f64]) -> Vec<Mode> {
        self.resonance_hz(tool_pose).map(|hz| Mode { hz, damping: 0.0 }).into_iter().collect()
    }
    /// With effects.belt_wrap: the angle (rad) by which the belt wraps each driven pulley at a tool pose, from the places
    /// of the pulleys and idlers, in the order of `motors`. Empty where the machine gives no belt geometry or the
    /// effect is off: then `ratings.belt_teeth_in_mesh` stands for the teeth in mesh.
    fn belt_wraps(&self, _tool_pose: &[f64]) -> Vec<f64> {
        Vec::new()
    }

    /// Why a tool pose within the axis ranges cannot be gone to, as a translation key: out of reach of the
    /// kinematics, outside the working envelope, beyond a joint's stop. `slack` (m) widens the envelope at its
    /// edges, as it widens the axis ranges for a setpoint at rest. `None` where it can; machines whose workspace is
    /// the box of their axis ranges keep this default.
    fn unreachable(&self, _tool_pose: &[f64], _slack: f64) -> Option<&'static str> {
        None
    }

    /// The 2-norm condition number of the kinematics' Jacobian at a tool pose, for the workspace map: 1 where the
    /// motors move the tool equally well in every direction, larger towards a singularity, `f64::INFINITY` out of reach
    /// of the kinematics (whatever the envelope and the joints' stops say, which `unreachable` tells). `None` for a
    /// machine that has no map to show: one whose motor angles are a linear function of the tool pose.
    fn condition(&self, _tool_pose: &[f64]) -> Option<f64> {
        None
    }

    /// How far the setpoint of the machine at rest may be from the pose of its tool, m: the setpoint is where the
    /// motors stand, and what belts and gears give way under the load they hold puts the tool off it. The controller
    /// takes paths and trajectories that start or run this far beyond the travel range or the envelope. 0.1 mm by
    /// default (12 and 46 um on the gantry holding its Z axis).
    fn setpoint_slack(&self) -> f64 {
        1e-4
    }

    /// Signals beyond axis and motor positions, speeds and torques, which every machine gets for free: first those
    /// of `base_signals`, then those of `effect_signals`.
    fn signal_infos(&self) -> Vec<SignalInfo>;
    /// The machine's own signals but those of the effect views; cheap enough to be asked for every step.
    fn base_signals(&self, state: &[f64], torque: &[f64], out: &mut Vec<f64>);
    /// The signals of the effect views (`TORQUE_PARTS`, `LOSS_PREFIX`, `ENERGY_PARTS`, `FRICTION_SIGNALS`, belt
    /// stretch), appended after those of `base_signals`. They cost about one more evaluation of the machine's
    /// derivatives, so they are asked for only where they are recorded or added up.
    fn effect_signals(&self, _state: &[f64], _torque: &[f64], _out: &mut Vec<f64>) {}
    /// The loss powers alone, the signals `power.*` in their order among the signals, without the rest of
    /// `effect_signals`: what warms the parts every step (`thermal_parts`). Empty where the machine reports none.
    fn loss_powers(&self, _state: &[f64], _torque: &[f64], _out: &mut Vec<f64>) {}
    /// Every signal of `signal_infos`.
    fn signals(&self, state: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        self.base_signals(state, torque, out);
        self.effect_signals(state, torque, out);
    }
    fn scene(&self, state: &[f64], out: &mut Scene);
    /// Where the scene draws the tool when it is at `tool_pose`, in scene coordinates, so that a front end can
    /// draw a path of the tool into the scene. `None` for a machine that has no such mapping.
    fn scene_point(&self, _tool_pose: &[f64]) -> Option<[f64; 3]> {
        None
    }
    /// The parts of the machine that can hit an obstacle of the config, as boxes in scene coordinates at `state`. By
    /// default the tool alone, as a point where `scene_point` puts it (none without that mapping); a machine names
    /// the boxes of its scene that move with the tool through `scene::scene_boxes`.
    fn collision_bodies(&self, state: &[f64]) -> Vec<crate::Aabb> {
        let n = self.axes().len();
        let (mut pos, mut vel) = (vec![0.0; n], vec![0.0; n]);
        self.tool_state(state, &mut pos, &mut vel);
        self.scene_point(&pos).map(|centre| crate::Aabb { centre, size: [0.0; 3] }).into_iter().collect()
    }
    /// The slanted parts of the machine that can hit an obstacle (rods, arms, shafts), as capsules in scene coordinates
    /// at `state`, beside the boxes of `collision_bodies`. None by default.
    fn collision_capsules(&self, _state: &[f64]) -> Vec<crate::Capsule> {
        Vec::new()
    }

    /// The friction contacts for the friction view. Empty where the machine tells none.
    fn friction_contacts(&self) -> Vec<FrictionContact> {
        Vec::new()
    }
    /// The steady sliding force of contact `i` at `speed`, with its levels scaled by `load_scale`, split into
    /// [Coulomb, viscous, stiction]. It opposes a positive speed.
    fn friction_curve(&self, _i: usize, _speed: f64, _load_scale: f64) -> [f64; 3] {
        [0.0; 3]
    }

    /// The belt sections for the belt view. Empty where the machine tells none.
    fn belt_sections(&self) -> Vec<BeltSection> {
        Vec::new()
    }

    /// The parts that warm with a loss power, for the thermal model and view. Empty where the machine tells none.
    fn thermal_parts(&self) -> Vec<ThermalPart> {
        Vec::new()
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
