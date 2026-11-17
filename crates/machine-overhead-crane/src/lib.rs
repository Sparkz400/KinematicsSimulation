//! Overhead crane: a bridge travelling on a runway (x), a trolley travelling along the bridge's girder (y), a hoist on
//! the trolley winding the rope on its drum, and the load hanging from the rope's end as a spherical pendulum on an
//! elastic rope. Each drive is a servo motor behind a gearbox, its wheel or drum the lever (`joint::RotaryJoint` with a
//! `transmission::Converter`): the motor turns i / r rad per metre of travel or of rope.
//!
//! The tool is the hook as the drives place it: the bridge's x, the trolley's y and the height z the drum's rope gives
//! the empty hook (m); the sway is the load's own (signals `load.*`, verdict residual-sway). The rope leaves the trolley's sheave at
//! T = (x_b, y_t, H) (`crane.rope_exit_height_m`) and runs straight to the load's point P (hook and load one point mass):
//!
//! ```text
//! L = |P - T|, u = (P - T) / L                   the rope's length and direction, sheave to load
//! e = L - l                                      its stretch over its free length l, paid out by the drum
//! t = EA e / l + c / l (L' - l')  if e > 0, else 0, and never below 0 (the rope goes slack: it cannot push)
//! V = EA e^2 / (2 l)                             its elastic energy while taut
//! ```
//!
//! The rope pulls the load with -t u, the trolley with t u (its x on the bridge, its y on the trolley; its z goes into
//! the rails), and the drum, through dV/dl, with t (1 + e / (2 l)) out (the damper's share as it is): the sway acts
//! back on both travel drives and the swing's stretch on the hoist. Equations of motion, rigid drives with the
//! rotors' inertia at the axis (i^2 J_rotor), the gearbox's efficiency on the power it passes (`Gearbox::loss_factor`):
//!
//! ```text
//! (m_b + m_t) x'' = drive_x + t u_x - F_roll,x + stops      m_t y'' = drive_y + t u_y - F_roll,y + stops
//! J_drum / r^2 l'' = drive_l + t (1 + e / (2 l)) + stops    m P'' = -t u - m g e_z
//! F_roll = w N tanh(v / v_s), N = (wheels' mass) g + t (-u_z)   (effects.guide_friction)
//! ```
//!
//! The pendulum's small swing has the period 2 pi sqrt(L / g) (`Machine::resonance_hz`, which the input shaper follows
//! with effects.shaper_follows_pose: the period of the rope's length where the plan stands, so a move that hoists while
//! it travels is shaped for the length as it changes). At rest the load hangs under the sheave on a rope stretched by
//! its weight, l = L / (1 + m g / EA); the motor angles of a pose (`inverse`) count on the empty hook's weight alone
//! (`kinematic_stretch`), so the drum stands still when the payload goes on or off. Pick and set-down are quasi-static
//! (`after_step`), or, with effects.floor_contact, the load stands on the floor until the rope carries it (`motion`).
//! With effects.rope_stretch off the rope is rigid, L = l, its tension the constraint force (`motion`); with the floor,
//! the load stands on it with the rope slack, and a rope that comes taut lifts it at once (`after_step`).
//!
//! What the model leaves out: the rope's bending at the sheave and the load's own turning (a bulky load twisting on its
//! sling: the load is a point mass).
//!
//! Four further effects, each its own switch (off: the model above, exactly), solved together by one mass matrix
//! (`full.rs`) over the coordinates x_c (bridge), y, l, the hook block's point P1 (3), with the effects the sling's two
//! angles (a, b) and the bridge's yaw psi; the rigid rope there is the constraint L = l on these coordinates, its tension
//! the multiplier:
//!
//! - effects.rope_mass: a rope of mass mu per metre (`rope.mass_per_m_kg_m`, 0.0035 d^2 kg/m for a steel wire rope of d mm).
//!   A straight rope of free length l from the sheave S to P1, its material's velocity linear along it from v_A = v_S + l'
//!   u (the rope runs off the sheave at the drum's speed) to v_P, has the kinetic energy
//!   K = mu l / 6 (|v_A|^2 + v_A . v_P + |v_P|^2); the rope wound on the drum, mu (capacity - l) of it, moves with the
//!   trolley and the drum's surface, K = mu (capacity - l) (|v_S|^2 + l'^2) / 2. Its weight is
//!   V = mu g [l (H + z_P) / 2 + (capacity - l) H] (the hanging part at its middle). Lagrange's equations (the mass mu l
//!   grows with the paid-out l, so d/dt of the momentum has the mu l' terms and dK/dl is a force on the drum) give the
//!   mass matrix and the speed terms of `full.rs`; nothing is booked as loss. The tension differs along the rope by the
//!   weight below: at the drum the load's weight plus mu g times the height hanging, (m + mu l) g, at the hook m g; the
//!   signal `rope.tension` is the drum's end (the highest), `rope.shortfall` is judged at the hook's end.
//! - effects.hook_block: the hook block (`crane.hook_mass_kg`) hangs on the rope at P1, the load under it on a sling of
//!   fixed length `crane.sling_length_m` at P2 = P1 + ls e(a, b), e = (sin a cos b, sin b, -cos a cos b): a spherical
//!   double pendulum (the sling's two angles). Signals of the load (sway, position) are P2's.
//! - effects.joint_elasticity: each drive is the elastic joint other machines use (`RotaryJoint::rotor`): the gearbox's
//!   stiffness, damping and backlash between the rotor (an angle and a speed of its own) and the wheel or drum.
//! - effects.bridge_skew: the bridge a rigid girder on two end carriages, each with a drive of the bridge's motor config
//!   (the bridge's command shared equally between them), the wheels' lateral stiffness holding the yaw psi =
//!   (x_B - x_A) / gauge with the moment k_psi psi, k_psi = k_lat b^2 / 2. The trolley at y on the girder is at x = x_c +
//!   psi (y - y_c); its place shares the load between the two sides.
//!
//! With effects.floor_contact the standing load rubs on the floor with Coulomb friction mu_f N (smoothed over
//! `crane.rolling_smoothing_m_s`), N the floor's push; the rigid rope respects the floor too (`motion`).

use sim_components::catalog::motor_config;
use sim_components::joint::RotaryJoint;
use sim_components::mech::{HardStop, ShaftFriction, ThermalFeedback};
use sim_components::rigid::{V3, add, dot};
use sim_components::transmission::Converter;
use sim_core::{
    AxisInfo, Config, ConfigError, ENERGY_PARTS, LOSS_PREFIX, Label, Machine, MachineEntry, MotorInfo, Part,
    RatingCheck, Scene, Shape, SignalInfo, TORQUE_PARTS, ThermalPart,
};
use std::f64::consts::TAU;

mod full;

pub static ENTRY: MachineEntry = MachineEntry {
    id: "overhead-crane",
    key: "machine-overhead-crane",
    default_config: include_str!("../config/crane.toml"),
    i18n: &[("en", include_str!("../i18n/en.ftl")), ("de", include_str!("../i18n/de.ftl"))],
    build,
    suite_configs,
};

const G: f64 = 9.80665;

/// Effects that take energy out or put it in; the standard suite runs with them off.
pub const LOSSY_EFFECTS: [&str; 8] = [
    "guide_friction",
    "rope_damping",
    "motor_friction",
    "gearbox_drag",
    "gearbox_lost_motion",
    "gearbox_efficiency",
    "torque_ripple",
    "holding_brake",
];

/// The default config with every loss switched off.
pub fn lossless_config() -> Config {
    let mut cfg = ENTRY.config();
    for effect in LOSSY_EFFECTS {
        cfg.set_text(&format!("effects.{effect} = false")).unwrap();
    }
    cfg.set_text("crane.stop_damping_n_s_m = 0.0").unwrap();
    cfg.set_text("hoist.stop_damping_n_s_m = 0.0").unwrap();
    cfg
}

/// Loss-free elastic and rigid ropes, with and without the payload. Gravity stays on: without it the load has nothing
/// to hang by.
fn suite_configs() -> Vec<Config> {
    [(true, true), (true, false), (false, true), (false, false)]
        .into_iter()
        .map(|(elastic, payload)| {
            let mut cfg = lossless_config();
            cfg.set_text(&format!("effects.rope_stretch = {elastic}")).unwrap();
            cfg.set_text(&format!("effects.payload = {payload}")).unwrap();
            cfg
        })
        .collect()
}

/// The shipped config with the input shaper on (the travel moves shaped against the sway, its period following the
/// rope's length): what `sim-cli verdicts overhead-crane --set effects.input_shaping=true` runs.
pub fn shaped_config() -> Config {
    let mut cfg = ENTRY.config();
    cfg.set_text("effects.input_shaping = true").unwrap();
    cfg
}

fn build(cfg: &Config) -> Result<Box<dyn Machine>, ConfigError> {
    sim_components::base::mount(cfg, Box::new(Crane::new(cfg)?))
}

const MOTOR_IDS: [&str; 3] = ["bridge", "trolley", "hoist"];
const MOTOR_KEYS: [&str; 3] = ["motor-bridge", "motor-trolley", "motor-hoist"];
const GEARBOX_IDS: [&str; 3] = ["gearbox_bridge", "gearbox_trolley", "gearbox_hoist"];
const AXIS_IDS: [&str; 3] = ["x", "y", "z"];
const AXIS_KEYS: [&str; 3] = ["axis-x", "axis-y", "axis-z"];

/// The state: bridge x, trolley y, the rope's free length l, their rates; the load's place P and its speed; the three
/// brakes' deflections. With the new effects more follow (`Layout`): the bridge's yaw, its rate and the second carriage's
/// brake; the sling's two angles and rates; the rotors' angles and speeds.
const RATES: usize = 3;
const LOAD: usize = 6;
const LOAD_SPEED: usize = 9;
const BRAKES: usize = 12;
const STATE_LEN: usize = 15;

/// Where the states of the new effects sit, and the generalised coordinates of `full.rs`.
#[derive(Debug, Clone)]
struct Layout {
    /// effects.bridge_skew: the state index of the yaw psi (then psi', the second carriage's brake).
    skew: Option<usize>,
    /// effects.hook_block: the sling's a, b, a', b'.
    sling: Option<usize>,
    /// effects.joint_elasticity: the rotors' angles, then their speeds (one each per drive).
    rotor: Option<usize>,
    /// Number of drives (3, and the second bridge carriage's with the skew) and of states.
    drives: usize,
    len: usize,
    /// Per generalised coordinate: its place in the state and its rate's.
    pos: Vec<usize>,
    vel: Vec<usize>,
}

impl Layout {
    fn new(skew: bool, sling: bool, elastic: bool) -> Self {
        let mut len = STATE_LEN;
        let drives = 3 + skew as usize;
        let mut pos: Vec<usize> = vec![0, 1, 2, LOAD, LOAD + 1, LOAD + 2];
        let mut vel: Vec<usize> = vec![RATES, RATES + 1, RATES + 2, LOAD_SPEED, LOAD_SPEED + 1, LOAD_SPEED + 2];
        let mut take = |n: usize| {
            len += n;
            len - n
        };
        let sk = skew.then(|| take(3));
        let sl = sling.then(|| take(4));
        let ro = elastic.then(|| take(2 * drives));
        if let Some(a) = sl {
            pos.extend([a, a + 1]);
            vel.extend([a + 2, a + 3]);
        }
        if let Some(k) = sk {
            pos.push(k);
            vel.push(k + 1);
        }
        Self { skew: sk, sling: sl, rotor: ro, drives, len, pos, vel }
    }
}

/// The bridge on two end carriages (effects.bridge_skew).
#[derive(Debug, Clone, Copy)]
struct Skew {
    /// The rails' distance, the girder's yaw inertia m_b gauge^2 / 12, the wheels' holding moment per rad and its damper.
    gauge: f64,
    inertia: f64,
    stiffness: f64,
    damping: f64,
}

fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn norm(a: V3) -> f64 {
    dot(a, a).sqrt()
}

/// The rope at one moment.
#[derive(Debug, Clone, Copy)]
pub struct Rope {
    /// Its length L from the sheave to the load, and the unit vector u along it.
    pub length: f64,
    pub u: V3,
    /// L' and the stretch e = L - l.
    pub rate: f64,
    pub stretch: f64,
    /// The tension on its ends (on the load and the trolley), its pull on the drum (dV/dl plus the damper), and the
    /// damper's share of both.
    pub tension: f64,
    pub drum: f64,
    pub damper: f64,
}

/// The accelerations and gearbox forces of one moment.
struct Motion {
    /// x'', y'', l''.
    acc: [f64; 3],
    load_acc: V3,
    /// Per drive: the force into its gearbox (after the rotor's acceleration) and out of it, at the axis (N).
    into: [f64; 3],
    out: [f64; 3],
    brake_rates: [f64; 3],
    /// What the rope gives each axis (its pull, the drum's for the hoist) and the rolling resistance (N).
    rope: [f64; 3],
    rolling: [f64; 3],
    /// The floor's push on the load (N), 0 unless it stands on it, and the power its friction takes (W).
    floor_force: f64,
    floor_power: f64,
    rope_state: Rope,
    /// What `full.rs` solved besides: per drive and per generalised coordinate.
    full: Option<Box<full::Extra>>,
}

#[derive(Clone)]
pub struct Crane {
    /// Bridge, trolley, hoist: each in N and m at its axis.
    drives: [RotaryJoint; 3],
    /// The masses each axis moves (bridge with trolley, trolley, the drum's inertia at the rope).
    masses: [f64; 3],
    bridge: f64,
    trolley: f64,
    hook: f64,
    payload: f64,
    attached: bool,
    /// The payload went on or off since the last step: `after_step` sets the rope to its weight.
    picked: bool,
    /// What that took out of the energy over the last step (W), booked with `power.other`.
    pick_loss: f64,
    /// effects.floor_contact: the height of the load's point when it stands on the floor (`crane.floor_height_m`).
    floor: Option<f64>,
    /// Coulomb friction coefficient between the standing load and the floor.
    floor_friction: f64,
    height: f64,
    /// Where the girder's middle is along the trolley's y (the yaw turns about it).
    y_centre: f64,
    /// The rope's mass per metre (effects.rope_mass; 0 off) and the rope the drum holds.
    rope_mass: f64,
    capacity: f64,
    /// effects.hook_block: the sling's length.
    sling: Option<f64>,
    /// effects.joint_elasticity.
    elastic_drives: bool,
    /// effects.bridge_skew: the yaw and the second end carriage's drive (the first is `drives[0]`).
    skew: Option<Skew>,
    side_b: Option<RotaryJoint>,
    /// One of the four effects above is on: the motion is `full.rs`'s.
    full: bool,
    layout: Layout,
    /// effects.rope_stretch: the elastic rope; off, the rigid reference.
    elastic: bool,
    ea: f64,
    damping: f64,
    t_min: f64,
    t_max: f64,
    rolling: f64,
    rolling_smoothing: f64,
    g: f64,
    sway_limit: f64,
    brake_limit: f64,
    drum_radius: f64,
    strokes: [f64; 2],
    dwelling: bool,
    brake_engaged: bool,
    thermal: Option<ThermalFeedback>,
    axes: Vec<AxisInfo>,
    motors: Vec<MotorInfo>,
}

impl Crane {
    pub fn new(cfg: &Config) -> Result<Self, ConfigError> {
        let stops = |table: &str| -> Result<HardStop, ConfigError> {
            Ok(if cfg.bool("effects.hard_stops")? {
                HardStop {
                    stiffness: cfg.positive(&format!("{table}.stop_stiffness_n_m"))?,
                    damping: cfg.non_negative(&format!("{table}.stop_damping_n_s_m"))?,
                }
            } else {
                HardStop { stiffness: 0.0, damping: 0.0 }
            })
        };
        let wheel_radii = [cfg.positive("crane.bridge_wheel_radius_m")?, cfg.positive("crane.trolley_wheel_radius_m")?];
        let strokes = [cfg.positive("crane.bridge_stroke_m")?, cfg.positive("crane.trolley_stroke_m")?];
        let drum_radius = cfg.positive("hoist.drum_radius_m")?;
        let levers = [wheel_radii[0], wheel_radii[1], drum_radius];
        let ranges = [(0.0, strokes[0]), (0.0, strokes[1]), (0.0, cfg.positive("hoist.capacity_m")?)];
        let tables = ["crane", "crane", "hoist"];
        // What each drive's wheel or drum moves rigidly, at its travel: the mesh friction's bristle is capped by the
        // lighter of it and the rotor (`Converter::apply_to`); the load hangs on the elastic rope.
        let (bridge, trolley) = (cfg.positive("crane.bridge_mass_kg")?, cfg.positive("crane.trolley_mass_kg")?);
        let masses =
            [bridge + trolley, trolley, cfg.positive("hoist.drum_inertia_kgm2")? / (drum_radius * drum_radius)];
        let mut drives = Vec::new();
        for k in 0..3 {
            // Each drive with its own motor, gearbox and drive tables (`[motor.hoist]` over `[motor]`).
            let own = motor_config(cfg, MOTOR_IDS[k])?;
            let mut joint = RotaryJoint::with(&own, ShaftFriction::NONE, stops(tables[k])?, ranges[k])?;
            Converter { lever: levers[k], stiffness: f64::INFINITY, backlash: 0.0, efficiency: 1.0, inertia: 0.0 }
                .apply_to(&own, &mut joint, masses[k])?;
            drives.push(joint);
        }
        let drives: [RotaryJoint; 3] = drives.try_into().map_err(|_| ConfigError::Invalid("motor".into()))?;
        let height = cfg.positive("crane.rope_exit_height_m")?;
        let (min, max) = (cfg.f64s("workspace.tool_min")?, cfg.f64s("workspace.tool_max")?);
        let inside = |k: usize, lo: f64, hi: f64| min[k] >= lo && max[k] <= hi && min[k] < max[k];
        if min.len() != 3 || max.len() != 3 || !inside(0, 0.0, strokes[0]) || !inside(1, 0.0, strokes[1]) {
            return Err(ConfigError::Invalid("workspace.tool_max".into()));
        }
        if !(inside(2, 0.0, height) && height - min[2] < ranges[2].1) {
            return Err(ConfigError::Invalid("workspace.tool_min".into()));
        }
        let axes = (0..3)
            .map(|k| {
                let get = |what: &str, rate: &str| cfg.positive(&format!("axis.{}.max_{what}_m_{rate}", AXIS_IDS[k]));
                Ok(AxisInfo {
                    id: AXIS_IDS[k],
                    key: AXIS_KEYS[k],
                    unit: "m",
                    min: min[k],
                    max: max[k],
                    max_vel: get("velocity", "s")?,
                    max_acc: get("acceleration", "s2")?,
                    max_jerk: get("jerk", "s3")?,
                })
            })
            .collect::<Result<Vec<_>, ConfigError>>()?;
        let t_min = cfg.non_negative("rope.min_tension_n")?;
        let t_max = cfg.positive("rope.breaking_load_n")? / cfg.positive("rope.safety_factor")?;
        let hoist = motor_config(cfg, "hoist")?;
        let flag = |name: &str| -> Result<bool, ConfigError> {
            let key = format!("effects.{name}");
            Ok(cfg.get(&key).is_ok() && cfg.bool(&key)?)
        };
        let (rope_mass_on, hook_block, elastic_drives, skew_on) =
            (flag("rope_mass")?, flag("hook_block")?, flag("joint_elasticity")?, flag("bridge_skew")?);
        let full = rope_mass_on || hook_block || elastic_drives || skew_on;
        let mass_per_m = cfg.non_negative("rope.mass_per_m_kg_m")?;
        let sling = cfg.positive("crane.sling_length_m")?;
        let (gauge, wheelbase) = (cfg.positive("crane.bridge_gauge_m")?, cfg.positive("crane.bridge_wheelbase_m")?);
        let lateral = cfg.positive("crane.bridge_wheel_lateral_stiffness_n_m")?;
        let ratio = cfg.non_negative("crane.bridge_skew_damping_ratio")?;
        let yaw_inertia = bridge * gauge * gauge / 12.0;
        let yaw_stiffness = lateral * wheelbase * wheelbase / 2.0;
        let skew = skew_on.then(|| Skew {
            gauge,
            inertia: yaw_inertia,
            stiffness: yaw_stiffness,
            damping: if cfg.bool("effects.guide_friction").unwrap_or(false) {
                2.0 * ratio * (yaw_stiffness * yaw_inertia).sqrt()
            } else {
                0.0
            },
        });
        let side_b = skew.is_some().then(|| drives[0].clone());
        Ok(Self {
            floor_friction: cfg.non_negative("crane.floor_friction_coefficient")?,
            y_centre: 0.5 * strokes[1],
            rope_mass: if rope_mass_on { mass_per_m } else { 0.0 },
            capacity: cfg.positive("hoist.capacity_m")?,
            sling: hook_block.then_some(sling),
            elastic_drives,
            layout: Layout::new(skew.is_some(), hook_block, elastic_drives),
            skew,
            side_b,
            full,
            masses,
            drives,
            bridge,
            trolley,
            hook: cfg.positive("crane.hook_mass_kg")?,
            payload: if cfg.bool("effects.payload")? { cfg.non_negative("crane.payload_kg")? } else { 0.0 },
            attached: true,
            picked: false,
            pick_loss: 0.0,
            floor: if cfg.bool("effects.floor_contact")? {
                Some(cfg.non_negative("crane.floor_height_m")?)
            } else {
                None
            },
            height,
            elastic: cfg.bool("effects.rope_stretch")?,
            ea: cfg.positive("rope.ea_n")?,
            damping: if cfg.bool("effects.rope_damping")? {
                cfg.non_negative("rope.specific_damping_n_s")?
            } else {
                0.0
            },
            t_min,
            t_max,
            rolling: if cfg.bool("effects.guide_friction")? {
                cfg.non_negative("crane.rolling_coefficient")?
            } else {
                0.0
            },
            rolling_smoothing: cfg.positive("crane.rolling_smoothing_m_s")?,
            g: if cfg.bool("effects.gravity")? { G } else { 0.0 },
            sway_limit: cfg.positive("crane.residual_sway_max_m")?,
            brake_limit: hoist.non_negative("motor.brake_torque_nm")? / cfg.positive("hoist.brake_safety_factor")?,
            drum_radius,
            strokes,
            dwelling: false,
            brake_engaged: false,
            thermal: ThermalFeedback::from_config(cfg)?,
            axes,
            motors: (0..3).map(|k| MotorInfo { id: MOTOR_IDS[k], key: MOTOR_KEYS[k] }).collect(),
        })
    }

    /// Hook and, while attached, payload.
    pub fn load_mass(&self) -> f64 {
        self.hook + if self.attached { self.payload } else { 0.0 }
    }

    /// The rope's stretch factor at rest under the load's weight (and half the free rope's, which hangs on the load: the
    /// spring's tension is the mean of its two ends'): L = l (1 + (m g + mu g l / 2) / EA).
    fn rest_stretch(&self, l: f64) -> f64 {
        if self.elastic { 1.0 + (self.load_mass() * self.g + 0.5 * self.rope_mass * self.g * l) / self.ea } else { 1.0 }
    }

    /// The pull on the drum of `mass` hanging at rest on the free length `l` under the sheave (N): the rope's tension at
    /// the hook's end t = (mass + mu l / 2) g on the spring (its mean), with the spring's dV/dl share t (1 + e / 2l), e =
    /// t l / EA, and the other half of the hanging rope's weight, mu g L / 2.
    fn rest_pull(&self, l: f64, mass: f64) -> f64 {
        let t = (mass + 0.5 * self.rope_mass * l) * self.g;
        let e = if self.elastic { t * l / self.ea } else { 0.0 };
        t * (1.0 + e / (2.0 * l)) + 0.5 * self.rope_mass * self.g * (l + e)
    }

    /// The payload's mass while it hangs on the hook.
    fn payload_mass(&self) -> f64 {
        if self.attached { self.payload } else { 0.0 }
    }

    /// The stretch the motor angles count on (`inverse`, `forward`): the empty hook's, whatever hangs on it. So the
    /// motors stand still when the payload goes on or off, as a hoist's drum does, and the payload hangs lower than the
    /// pose by its own stretch, m g L / EA (6.8 mm for the 250 kg on 4 m of rope; `setpoint_slack`). Counting the payload
    /// in would step the hoist's setpoint by that much when the hook lets go, and the drum, paying it out faster than the
    /// hook can fall, would slacken the rope.
    fn kinematic_stretch(&self) -> f64 {
        if self.elastic { 1.0 + self.hook * self.g / self.ea } else { 1.0 }
    }

    /// The small swing's angular frequency of a rope of length L; `None` without gravity. The point load on a rope of mass
    /// mu per metre: w^2 = g (m + mu L / 2) / (L (m + mu L / 3)) (sqrt(g / L) for a light rope); with the hook block and a
    /// payload on its sling the lower root of m1 l1 l2 w^4 - (m1 + m2) g (l1 + l2) w^2 + (m1 + m2) g^2 = 0.
    pub fn sway_omega(&self, length: f64) -> Option<f64> {
        if !(self.g > 0.0 && length > 0.0) {
            return None;
        }
        if let Some(l2) = self.sling.filter(|_| self.payload_mass() > 0.0) {
            let (m1, m2, g) = (self.hook, self.payload_mass(), self.g);
            let b = (m1 + m2) * g * (length + l2);
            let w2 = (b - (b * b - 4.0 * m1 * length * l2 * (m1 + m2) * g * g).sqrt()) / (2.0 * m1 * length * l2);
            return Some(w2.sqrt());
        }
        let (m, rope) = (self.load_mass(), self.rope_mass * length);
        Some((self.g / length * (1.0 + rope / (2.0 * m)) / (1.0 + rope / (3.0 * m))).sqrt())
    }

    /// The bridge's yaw and its rate (0 without effects.bridge_skew).
    fn yaw(&self, s: &[f64]) -> (f64, f64) {
        self.layout.skew.map_or((0.0, 0.0), |k| (s[k], s[k + 1]))
    }

    /// Where the rope leaves the trolley: on a girder yawed by psi the trolley at y is at x = x_c + psi (y - y_c).
    fn sheave(&self, s: &[f64]) -> V3 {
        let (psi, _) = self.yaw(s);
        [s[0] + psi * (s[1] - self.y_centre), s[1], self.height]
    }

    /// The sheave's velocity.
    fn sheave_speed(&self, s: &[f64]) -> V3 {
        let (psi, rate) = self.yaw(s);
        [s[RATES] + rate * (s[1] - self.y_centre) + psi * s[RATES + 1], s[RATES + 1], 0.0]
    }

    /// The rope's geometry at a state, and with effects.rope_stretch its tension; the rigid rope's tension comes from
    /// the motion (`motion`), and is 0 here.
    pub fn rope(&self, s: &[f64]) -> Rope {
        let d = sub([s[LOAD], s[LOAD + 1], s[LOAD + 2]], self.sheave(s));
        let length = norm(d);
        let u = if length > 0.0 { d.map(|v| v / length) } else { [0.0, 0.0, -1.0] };
        let vs = self.sheave_speed(s);
        let relative = [s[LOAD_SPEED] - vs[0], s[LOAD_SPEED + 1] - vs[1], s[LOAD_SPEED + 2]];
        let rate = dot(u, relative);
        let (l, l_rate) = (s[2], s[RATES + 2]);
        let stretch = length - l;
        let (mut tension, mut drum, mut damper) = (0.0, 0.0, 0.0);
        if self.elastic && stretch > 0.0 {
            let spring = self.ea * stretch / l;
            tension = (spring + self.damping / l * (rate - l_rate)).max(0.0);
            damper = tension - spring;
            drum = spring * (1.0 + stretch / (2.0 * l)) + damper;
        }
        Rope { length, u, rate, stretch, tension, drum, damper }
    }

    /// The accelerations at a state under the motor torques. Each axis k: a_k = (eta_k D_k + F_k + p_k) / (M_k +
    /// eta_k J_k), D the drive's force through the gearbox, F the stops and the rolling, p the rope's pull, eta the
    /// gearbox's loss factor at the power it passes (from a first pass with eta = 1, as on every rigid drive).
    ///
    /// The rigid rope (effects.rope_stretch off) keeps L = l: its tension t is what makes L'' = l''. With each a_k =
    /// alpha_k + beta_k t (beta = (u_x, u_y, 1) / (M + eta J)) and the load's P'' = -t u / m - g e_z,
    /// L'' = u . (P'' - T'') + (|d'|^2 - (u . d')^2) / L, d' = P' - T', is linear in t:
    /// t = (kappa - g u_z - u_x alpha_x - u_y alpha_y - alpha_l) / (1 / m + u_x beta_x + u_y beta_y + beta_l).
    /// It is a bilateral constraint: where the load would leave the rope slack, t comes out below zero (the rope
    /// pushes). The reference for a taut rope; the elastic model is the one that goes slack.
    fn motion(&self, s: &[f64], torque: &[f64]) -> Motion {
        if self.full {
            return self.full_motion(s, torque);
        }
        let mut rope = self.rope(s);
        let m = self.load_mass();
        let u = rope.u;
        let (mut drive, mut brake_rates) = ([0.0; 3], [0.0; 3]);
        for k in 0..3 {
            let j = &self.drives[k];
            let (q, v) = (s[k], s[RATES + k]);
            (drive[k], brake_rates[k]) =
                j.rigid_drive(torque[k], j.ratio() * q, j.ratio() * v, s[BRAKES + k], self.brake_engaged);
        }
        let d_rate = [s[LOAD_SPEED] - s[RATES], s[LOAD_SPEED + 1] - s[RATES + 1], s[LOAD_SPEED + 2]];
        let kappa = (dot(d_rate, d_rate) - rope.rate * rope.rate) / rope.length;
        // The rigid rope with the floor: the load that would sink stands on it (`stand`), its rope slack (a gap beyond
        // rounding) or taut.
        let stand = self.floor.filter(|&floor| !self.elastic && s[LOAD + 2] <= floor && s[LOAD_SPEED + 2] <= 0.0);
        let slack = rope.stretch < -1e-6;
        let speed_h = [s[LOAD_SPEED], s[LOAD_SPEED + 1]];
        // One pass: the rolling at tension `t_roll`, the gearboxes' loss factors `loss`: (tension, rolling, a, the
        // standing rigid load's floor push and friction).
        let pass = |t_roll: f64, loss: [f64; 3]| -> (f64, [f64; 3], [f64; 3], Option<(f64, [f64; 2])>) {
            let down = t_roll * -u[2];
            let wheels = [(self.bridge + self.trolley) * self.g + down, self.trolley * self.g + down];
            let mut rolling = [0.0; 3];
            for k in 0..2 {
                rolling[k] = self.rolling * wheels[k].max(0.0) * (s[RATES + k] / self.rolling_smoothing).tanh();
            }
            let (mut alpha, mut beta) = ([0.0; 3], [0.0; 3]);
            for k in 0..3 {
                let j = &self.drives[k];
                let denominator = self.masses[k] + loss[k] * j.reflected_inertia();
                let force = -rolling[k] + j.output_torque(s[k], s[RATES + k]);
                alpha[k] = (loss[k] * drive[k] + force) / denominator;
                beta[k] = [u[0], u[1], 1.0][k] / denominator;
            }
            let mut floor = None;
            let tension = if self.elastic {
                rope.tension
            } else {
                let free = (kappa - self.g * u[2] - u[0] * alpha[0] - u[1] * alpha[1] - alpha[2])
                    / (1.0 / m + u[0] * beta[0] + u[1] * beta[1] + beta[2]);
                if stand.is_some() && (slack || -free * u[2] / m - self.g < 0.0) {
                    // Standing: the floor holds the load's height (a_z = 0), so L'' = l'' gives t from the horizontal
                    // part alone, t (u_x^2 + u_y^2 + ...) = kappa + u . F / m - u_x alpha_x - u_y alpha_y - alpha_l, F the
                    // floor's friction, which takes the floor's push N = m g + t u_z. A slack rope pulls nothing.
                    let denominator = (u[0] * u[0] + u[1] * u[1]) / m + u[0] * beta[0] + u[1] * beta[1] + beta[2];
                    let base = kappa - u[0] * alpha[0] - u[1] * alpha[1] - alpha[2];
                    let (mut t, mut n, mut f) = (0.0, m * self.g, [0.0; 2]);
                    for _ in 0..6 {
                        t = if slack { 0.0 } else { ((base + (u[0] * f[0] + u[1] * f[1]) / m) / denominator).max(0.0) };
                        n = (m * self.g + t * u[2]).max(0.0);
                        f = self.floor_friction_force(n, speed_h);
                    }
                    floor = Some((n, f));
                    t
                } else {
                    free
                }
            };
            let pull = [tension * u[0], tension * u[1], if self.elastic { rope.drum } else { tension }];
            let acc = std::array::from_fn(|k| {
                alpha[k] + pull[k] / (self.masses[k] + loss[k] * self.drives[k].reflected_inertia())
            });
            (tension, rolling, acc, floor)
        };
        let guess = if self.elastic { rope.tension } else { m * self.g * -u[2] };
        // The loss factors of the torques going in, which depend on the accelerations they give: to the fixed point
        // (`mech::settled_loss`), from the loss-free pass, the rigid rope's tension along.
        let mut first_tension = guess;
        let loss = sim_components::mech::settled_loss([1.0; 3], |loss| {
            let (tension, _, acc, _) = pass(first_tension, *loss);
            first_tension = tension;
            std::array::from_fn(|k| {
                let j = &self.drives[k];
                j.gearbox.loss_factor(drive[k] - j.reflected_inertia() * acc[k], s[RATES + k])
            })
        });
        // The rigid rope's tension depends on the loss factors (the hoist's eta moves it by some percent) and on the
        // rolling, which takes the tension: iterated to the fixed point (the rolling changes the tension by some 1e-5 of
        // itself, so three passes reach 1e-12). The losses are booked with the rolling of the last pass.
        let (tension, rolling, acc, floor_info) = if self.elastic {
            pass(first_tension, loss)
        } else {
            let mut t = pass(first_tension, loss).0;
            let mut last = pass(t, loss);
            for _ in 0..8 {
                if (last.0 - t).abs() <= 1e-12 * (1.0 + t.abs()) {
                    break;
                }
                t = last.0;
                last = pass(t, loss);
            }
            last
        };
        if !self.elastic {
            (rope.tension, rope.drum) = (tension, tension);
        }
        let into: [f64; 3] = std::array::from_fn(|k| drive[k] - self.drives[k].reflected_inertia() * acc[k]);
        let out = std::array::from_fn(|k| loss[k] * into[k]);
        let mut load_acc = [-tension * u[0] / m, -tension * u[1] / m, -tension * u[2] / m - self.g];
        // The floor under the load (one-sided: it pushes, never pulls): standing on it and sinking, the floor takes
        // what keeps the load at its height, and its friction (Coulomb, smoothed) acts along it.
        let standing = if self.elastic {
            self.floor.is_some_and(|floor| s[LOAD + 2] <= floor && s[LOAD_SPEED + 2] <= 0.0 && load_acc[2] < 0.0).then(
                || {
                    let n = -m * load_acc[2];
                    (n, self.floor_friction_force(n, speed_h))
                },
            )
        } else {
            floor_info
        };
        let (mut floor_force, mut floor_power) = (0.0, 0.0);
        if let Some((n, f)) = standing {
            floor_force = n;
            load_acc[2] = 0.0;
            if self.elastic {
                load_acc[0] += f[0] / m;
                load_acc[1] += f[1] / m;
            } else {
                load_acc[0] = (-tension * u[0] + f[0]) / m;
                load_acc[1] = (-tension * u[1] + f[1]) / m;
            }
            floor_power = -(f[0] * speed_h[0] + f[1] * speed_h[1]);
        }
        let pull = [tension * u[0], tension * u[1], rope.drum];
        Motion {
            acc,
            load_acc,
            into,
            out,
            brake_rates,
            rope: pull,
            rolling,
            floor_force,
            floor_power,
            rope_state: rope,
            full: None,
        }
    }

    /// Puts the rope's lower end on the rope's line at `length` from the sheave and its speed along it to the drum's
    /// (the rope's own factor `factor` of the paid-out speed), the speed across the rope kept.
    fn place_on_rope(&self, s: &mut [f64], length: f64, factor: f64) {
        let (rope, sheave, trolley) = (self.rope(s), self.sheave(s), self.sheave_speed(s));
        let speed = [s[LOAD_SPEED], s[LOAD_SPEED + 1], s[LOAD_SPEED + 2]];
        let along = dot(rope.u, trolley) + s[RATES + 2] * factor - dot(rope.u, speed);
        for k in 0..3 {
            s[LOAD + k] = sheave[k] + rope.u[k] * length;
            s[LOAD_SPEED + k] += rope.u[k] * along;
        }
    }

    /// The floor's Coulomb friction on the standing load that pushes it with `n` and slides at the horizontal speed `v`
    /// (N): mu n tanh(|v| / v_s) against the motion, smoothed over `crane.rolling_smoothing_m_s` like the wheels'.
    fn floor_friction_force(&self, n: f64, v: [f64; 2]) -> [f64; 2] {
        let speed = v[0].hypot(v[1]);
        if speed == 0.0 || self.floor_friction == 0.0 {
            return [0.0; 2];
        }
        let f = -self.floor_friction * n * (speed / self.rolling_smoothing).tanh() / speed;
        [f * v[0], f * v[1]]
    }

    /// The load's horizontal offset from under the sheave and its rate.
    fn sway(&self, s: &[f64]) -> ([f64; 2], [f64; 2]) {
        let (sheave, speed) = (self.sheave(s), self.sheave_speed(s));
        let (load, v) = self.lower(s);
        ([load[0] - sheave[0], load[1] - sheave[1]], [v[0] - speed[0], v[1] - speed[1]])
    }

    /// The sway's amplitude, sqrt(d^2 + (d' / w)^2) of the horizontal offset d at w = sqrt(g / L): where a free
    /// pendulum swinging like this would turn back.
    pub fn sway_amplitude(&self, s: &[f64]) -> f64 {
        let (d, rate) = self.sway(s);
        let offset = d[0].hypot(d[1]);
        match self.sway_omega(self.rope(s).length) {
            Some(w) => (offset * offset + (rate[0].hypot(rate[1]) / w).powi(2)).sqrt(),
            None => offset,
        }
    }

    /// The rope's pull `drum` on the drum at the hoist's motor (N m): what its brake holds.
    fn brake_load(&self, drum: f64) -> f64 {
        drum.abs() / self.drives[2].ratio()
    }

    /// Per drive the powers lost in its motor's friction with the brake and in its gearbox; then the wheels' rolling of
    /// bridge and trolley, the rope's damper, the stops, and what the ripple gives off. And per drive the gearbox's loss
    /// as a torque at the rotor.
    fn losses(&self, s: &[f64], torque: &[f64], motion: &Motion) -> ([f64; 11], [f64; 3]) {
        if let Some(extra) = &motion.full {
            return self.full_losses(s, torque, motion, extra);
        }
        let mut powers = [0.0; 11];
        let mut at_rotor = [0.0; 3];
        for k in 0..3 {
            let j = &self.drives[k];
            let (q, v) = (s[k], s[RATES + k]);
            let [_, stop, shaft, drag] = j.losses(q, v, j.ratio() * v, s[BRAKES + k], self.brake_engaged);
            at_rotor[k] = (motion.into[k] - motion.out[k]) / j.ratio();
            powers[2 * k] = shaft;
            powers[2 * k + 1] = (motion.into[k] - motion.out[k]) * v + drag;
            powers[9] += stop;
            powers[10] += (torque[k] - j.ripple.apply(torque[k], j.ratio() * q)) * j.ratio() * v;
        }
        powers[10] += self.pick_loss + motion.floor_power;
        powers[6] = motion.rolling[0] * s[RATES];
        powers[7] = motion.rolling[1] * s[RATES + 1];
        let rope = &motion.rope_state;
        powers[8] = rope.damper * (rope.rate - s[RATES + 2]);
        (powers, at_rotor)
    }

    fn energy_parts(&self, s: &[f64]) -> [f64; 3] {
        if self.full {
            return self.full_energy(s);
        }
        let m = self.load_mass();
        let v = [s[LOAD_SPEED], s[LOAD_SPEED + 1], s[LOAD_SPEED + 2]];
        let mut kinetic = 0.5 * m * dot(v, v);
        let mut elastic = 0.0;
        for k in 0..3 {
            let j = &self.drives[k];
            kinetic += 0.5 * (self.masses[k] + j.reflected_inertia()) * s[RATES + k].powi(2);
            elastic += j.stop_energy(s[k]);
        }
        let stretch = self.rope(s).stretch;
        if self.elastic && stretch > 0.0 {
            elastic += 0.5 * self.ea * stretch * stretch / s[2];
        }
        [kinetic, m * self.g * s[LOAD + 2], elastic]
    }
}

impl Machine for Crane {
    /// Bridge (at the middle of its girder), trolley, hook block, load on its sling and the rope (the free rope a point of
    /// its mass at its middle, moving with the mean of its ends' material: the exact momentum of a linear velocity; the
    /// wound rope at the sheave) as points; the drives' spin, and the wound rope's turning with the drum, are left out.
    fn frame_momentum(&self, s: &[f64]) -> Option<sim_core::FrameMomentum> {
        let mut f = sim_core::FrameMomentum { gravity: [0.0, 0.0, -self.g], ..Default::default() };
        let add_point = sim_components::base::add_point;
        add_point(&mut f, self.bridge, [s[0], 0.5 * self.strokes[1], self.height], [s[RATES], 0.0, 0.0]);
        let (sheave, sheave_speed) = (self.sheave(s), self.sheave_speed(s));
        add_point(&mut f, self.trolley, sheave, sheave_speed);
        let load = [s[LOAD], s[LOAD + 1], s[LOAD + 2]];
        let speed = [s[LOAD_SPEED], s[LOAD_SPEED + 1], s[LOAD_SPEED + 2]];
        let hook = if self.sling.is_some() { self.hook } else { self.load_mass() };
        add_point(&mut f, hook, load, speed);
        if self.sling.is_some() && self.payload_mass() > 0.0 {
            let (place, v) = self.lower(s);
            add_point(&mut f, self.payload_mass(), place, v);
        }
        if self.rope_mass > 0.0 {
            // The free rope's top end runs off the sheave at the drum's speed along the rope; the wound rope at the sheave.
            let (u, paid) = (self.rope(s).u, s[RATES + 2]);
            let mid = std::array::from_fn(|k| 0.5 * (sheave[k] + load[k]));
            let v = std::array::from_fn(|k| 0.5 * (sheave_speed[k] + paid * u[k] + speed[k]));
            add_point(&mut f, self.rope_mass * s[2], mid, v);
            add_point(&mut f, self.rope_mass * (self.capacity - s[2]), sheave, sheave_speed);
        }
        Some(f)
    }

    fn axes(&self) -> &[AxisInfo] {
        &self.axes
    }
    fn motors(&self) -> &[MotorInfo] {
        &self.motors
    }

    /// The rope's highest tension against its rating, how far it fell below the least (none: no slack), the sway left
    /// at a dwell, and the rope's pull on the hoist's brake.
    fn rating_checks(&self) -> Vec<RatingCheck> {
        let check =
            |check, signal: &str, motor, limit| RatingCheck { check, signal: signal.into(), motor, limit, life: false };
        vec![
            check("rope-max-tension", "rope.tension", Some(2), self.t_max),
            check("rope-min-tension", "rope.shortfall", Some(2), 0.0),
            check("residual-sway", "load.residual_sway", None, self.sway_limit),
            check("hoist-brake", "hoist.brake_load", Some(2), self.brake_limit),
        ]
    }

    /// Bridge and trolley under the load, the rope's free length that the empty hook stretches to H - z.
    fn inverse(&self, tool: &[f64], motors: &mut [f64]) {
        let l = (self.height - tool[2]) / self.kinematic_stretch();
        for (k, q) in [tool[0], tool[1], l].into_iter().enumerate() {
            motors[k] = self.drives[k].ratio() * q;
        }
    }

    fn forward(&self, motors: &[f64], tool: &mut [f64]) {
        let q: Vec<f64> = (0..3).map(|k| motors[k] / self.drives[k].ratio()).collect();
        tool[..3].copy_from_slice(&[q[0], q[1], self.height - q[2] * self.kinematic_stretch()]);
    }

    fn state_len(&self) -> usize {
        self.layout.len
    }

    fn equilibrium(&self, tool: &[f64], state: &mut Vec<f64>, torque: &mut [f64]) {
        if self.full {
            return self.full_equilibrium(tool, state, torque);
        }
        // The drum where the motors stand for the pose (`inverse`), the load on it at its own stretch: it hangs below the
        // pose by what the payload stretches the rope more than the empty hook.
        let l = (self.height - tool[2]) / self.kinematic_stretch();
        let load = self.height - l * self.rest_stretch(l);
        state.clear();
        state.extend([tool[0], tool[1], l, 0.0, 0.0, 0.0, tool[0], tool[1], load]);
        state.resize(STATE_LEN, 0.0);
        // The rigid rope holds the load's weight as it is.
        let drum = if self.elastic { self.rope(state).drum } else { self.load_mass() * self.g };
        let pull = [0.0, 0.0, drum];
        for k in 0..3 {
            let j = &self.drives[k];
            // At rest the gearbox holds the axis against the rope and the stops.
            let held = -pull[k] - j.output_torque(state[k], 0.0);
            torque[k] = j.ripple.invert(held / j.ratio(), j.ratio() * state[k]);
        }
    }

    fn derivatives(&self, s: &[f64], torque: &[f64], out: &mut [f64]) {
        if self.full {
            return self.full_derivatives(s, torque, out);
        }
        let motion = self.motion(s, torque);
        out[..3].copy_from_slice(&s[RATES..RATES + 3]);
        out[RATES..RATES + 3].copy_from_slice(&motion.acc);
        out[LOAD..LOAD + 3].copy_from_slice(&s[LOAD_SPEED..LOAD_SPEED + 3]);
        out[LOAD_SPEED..LOAD_SPEED + 3].copy_from_slice(&motion.load_acc);
        out[BRAKES..BRAKES + 3].copy_from_slice(&motion.brake_rates);
    }

    /// Where the drives put the hook (`forward` of their angles): the bridge's x, the trolley's y, the height the drum's
    /// rope gives the empty hook. The load's sway and its bounce on the rope are not in the tool pose (a crane is
    /// positioned by its drives, as its following error and settling say); they have their own signals and the verdict
    /// residual-sway.
    fn tool_state(&self, s: &[f64], pos: &mut [f64], vel: &mut [f64]) {
        let stretch = self.kinematic_stretch();
        pos[..3].copy_from_slice(&[s[0], s[1], self.height - s[2] * stretch]);
        vel[..3].copy_from_slice(&[s[RATES], s[RATES + 1], -s[RATES + 2] * stretch]);
    }

    /// Everything moves with the load: bridge and trolley under it, the drum paying out as the rope at its rest stretch.
    fn set_tool_velocity(&self, s: &mut [f64], vel: &[f64]) {
        s[RATES..RATES + 3].copy_from_slice(&[vel[0], vel[1], -vel[2] / self.kinematic_stretch()]);
        s[LOAD_SPEED..LOAD_SPEED + 3].copy_from_slice(&vel[..3]);
        if let Some(r0) = self.layout.rotor {
            let nd = self.layout.drives;
            let axis = [s[RATES], s[RATES + 1], s[RATES + 2], s[RATES]];
            for d in 0..nd {
                s[r0 + nd + d] = self.drive(d).ratio() * axis[d];
            }
        }
    }

    /// The motors' angles: the rotors' with the elastic drives (the bridge's the mean of its two carriages'), else the
    /// travels times the ratios.
    fn motor_state(&self, s: &[f64], pos: &mut [f64], vel: &mut [f64]) {
        if let Some(r0) = self.layout.rotor {
            let nd = self.layout.drives;
            for k in 0..3 {
                (pos[k], vel[k]) = (s[r0 + k], s[r0 + nd + k]);
            }
            if nd == 4 {
                pos[0] = 0.5 * (s[r0] + s[r0 + 3]);
                vel[0] = 0.5 * (s[r0 + nd] + s[r0 + nd + 3]);
            }
            return;
        }
        for k in 0..3 {
            pos[k] = self.drives[k].ratio() * s[k];
            vel[k] = self.drives[k].ratio() * s[RATES + k];
        }
    }

    fn feedforward_torque(&self, pos: &[f64], acc: &[f64], out: &mut [f64]) {
        self.feedforward_torque_moving(pos, &[0.0; 3], acc, out);
    }

    /// The rigid model: the load carried along with the trolley (no sway), the rope at its rest stretch; the rope's
    /// tension m (g + z'') on the drum with its dV/dl share. Its speed asks nothing (the motor angles are linear in the
    /// pose but for the stretch).
    fn feedforward_torque_moving(&self, pos: &[f64], _vel: &[f64], acc: &[f64], out: &mut [f64]) {
        let m = self.load_mass();
        let tension = m * (self.g + acc[2]);
        let l = (self.height - pos[2]) / self.kinematic_stretch();
        let stretch = if self.elastic { l * tension / self.ea } else { 0.0 };
        // The rope's own mass (effects.rope_mass): all of it travels with the trolley, the hanging part's weight and
        // inertia pull on the drum, the wound part turns with it.
        let (rope, wound) = (self.rope_mass * self.capacity, self.rope_mass * (self.capacity - l));
        let drum = tension * (1.0 + stretch / (2.0 * l)) + self.rope_mass * l * (self.g + acc[2]);
        let axis_acc = [acc[0], acc[1], -acc[2] / self.kinematic_stretch()];
        let force = [
            (self.masses[0] + m + rope) * acc[0],
            (self.masses[1] + m + rope) * acc[1],
            (self.masses[2] + wound) * axis_acc[2] - drum,
        ];
        for k in 0..3 {
            let j = &self.drives[k];
            // The bridge's two carriages' rotors turn for the one command (shared equally).
            let rotors = if k == 0 && self.skew.is_some() { 2.0 } else { 1.0 };
            out[k] = force[k] / j.ratio() + rotors * j.rotor_inertia * j.ratio() * axis_acc[k];
        }
    }

    /// At a steady speed: the motor shaft's friction and, through the gearbox as `motion` takes it, the wheels' rolling
    /// under the load hanging still.
    fn steady_friction(&self, pos: &[f64], vel: &[f64], out: &mut [f64]) {
        let mut held = [0.0; 3];
        self.feedforward_torque(pos, &[0.0; 3], &mut held);
        let down = self.load_mass() * self.g;
        let wheels = [(self.bridge + self.trolley) * self.g + down, self.trolley * self.g + down, 0.0];
        let speeds = [vel[0], vel[1], -vel[2] / self.kinematic_stretch()];
        for (k, j) in self.drives.iter().enumerate() {
            let (i, v) = (j.ratio(), speeds[k]);
            let rolling = self.rolling * wheels[k] * (v / self.rolling_smoothing).tanh();
            let shaft = if k == 0 && self.skew.is_some() { 2.0 } else { 1.0 } * j.shaft.torque(i * v);
            out[k] = shaft + rolling / (i * j.gearbox.loss_factor(i * held[k] + rolling, v));
        }
    }

    fn energy(&self, s: &[f64]) -> f64 {
        self.energy_parts(s).iter().sum()
    }

    fn set_payload_attached(&mut self, attached: bool) {
        self.picked |= attached != self.attached;
        self.attached = attached;
    }

    /// Pick and set-down are quasi-static: a load slung on the floor goes on as the hoist takes its weight up slowly,
    /// and comes off as it is set down. So once the payload went on or off, the load moves along the rope to where the
    /// rope's free length holds the new mass at rest, L = l (1 + m g / EA), and its speed along the rope to the drum's
    /// (the rest across the rope, its sway, is kept). Not done, the rope would snatch the new load from rest (to twice its
    /// weight) and fling the empty hook up off a slack rope. What that changes in the energy (the floor's share of the
    /// lift, some 0.1 J) is booked with `power.other` over this step.
    ///
    /// With effects.floor_contact the floor is modelled where the load stands on it (`motion`: the floor force N >= 0,
    /// N = -m a_z of the free load, which then stays at `crane.floor_height_m`): a payload's mass goes on without a jump
    /// of the rope (the floor carries it), the rope's tension takes the weight over as the drum winds (the snatch of a
    /// fast lift is the rope's stretch at the drum's speed, t = EA e / l, until t |u_z| = m g), and a slack rope on
    /// lowering leaves the load standing. The load's fall onto the floor is plastic: its downward speed is lost
    /// (m v^2 / 2, booked with `power.other`). Above the floor the pick is quasi-static as before.
    ///
    /// The standing load rubs on the floor with Coulomb friction mu_f N (`motion`), N the floor's push; the plain rigid rope
    /// respects the floor too: its load stands with the rope slack (the drum paying out, nothing pulls), and where the rope
    /// comes taut (L > l) the load is lifted to l at once with its speed along the rope made the drum's, a plastic
    /// impact whose energy is booked with `power.other`. With the sling the payload's point is the one that stands.
    fn after_step(&mut self, s: &mut [f64], dt: f64) {
        self.pick_loss = 0.0;
        if let Some(r0) = self.layout.rotor {
            for d in 0..self.layout.drives {
                let (pos, _) = self.travel(s, d);
                self.drive_mut(d).gearbox.commit(s[r0 + d], pos);
            }
        }
        let mut standing = false;
        if let Some(floor) = self.floor {
            let (low, low_v) = self.lower(s);
            standing = low[2] <= floor;
            if standing && (low[2] < floor || low_v[2] < 0.0) {
                let before = self.energy(s);
                if self.sling.is_some() && self.payload_mass() > 0.0 {
                    s[LOAD + 2] += floor - low[2];
                    s[LOAD_SPEED + 2] -= low_v[2].min(0.0);
                } else {
                    s[LOAD + 2] = floor;
                    s[LOAD_SPEED + 2] = s[LOAD_SPEED + 2].max(0.0);
                }
                self.pick_loss += (before - self.energy(s)) / dt;
            }
            if !self.elastic && self.rope(s).stretch > 1e-9 {
                let before = self.energy(s);
                self.place_on_rope(s, s[2], 1.0);
                self.pick_loss += (before - self.energy(s)) / dt;
            }
        }
        if !std::mem::take(&mut self.picked) || standing {
            return;
        }
        let before = self.energy(s);
        if self.layout.rotor.is_some() {
            // The hoist's gearbox takes the new weight as quasi-statically as the rope: the servo holds the rotor, the
            // drum gives by the change of the static twist (0.78 mm for the 250 kg at the shipped stiffness).
            let j = &self.drives[2];
            let old = self.load_mass() + if self.attached { -self.payload } else { self.payload };
            let twist = |mass: f64, l: f64| j.gearbox.static_twist(-self.rest_pull(l, mass) - j.output_torque(l, 0.0));
            let shift = twist(self.load_mass(), s[2]) - twist(old, s[2]);
            s[2] -= shift;
        }
        let factor = self.rest_stretch(s[2]);
        self.place_on_rope(s, s[2] * factor, factor);
        if let Some(a0) = self.layout.sling {
            // The payload went on or off: the sling hangs down again.
            s[a0..a0 + 4].fill(0.0);
        }
        self.pick_loss += (before - self.energy(s)) / dt;
    }

    /// What the gearboxes' hysteresis carries between steps (elastic drives only).
    fn memory(&self) -> Vec<f64> {
        if !self.elastic_drives {
            return Vec::new();
        }
        sim_components::mech::Gearbox::anchors(self.drives.iter().chain(self.side_b.iter()).map(|j| &j.gearbox))
    }
    fn set_memory(&mut self, memory: &[f64]) {
        if self.elastic_drives {
            sim_components::mech::Gearbox::set_anchors(
                self.drives.iter_mut().chain(self.side_b.iter_mut()).map(|j| &mut j.gearbox),
                memory,
            );
        }
    }
    fn set_dwelling(&mut self, dwelling: bool) {
        self.dwelling = dwelling;
    }
    fn set_brake_engaged(&mut self, engaged: bool) {
        self.brake_engaged = engaged;
    }

    /// The gearboxes, in the order of `thermal_parts`.
    fn set_part_temperatures(&mut self, rises: &[f64]) {
        let Some(law) = &self.thermal else { return };
        for (drive, rise) in self.drives.iter_mut().filter(|d| !d.coupled).zip(rises) {
            drive.gearbox.efficiency = law.gearbox_efficiency(*rise) * drive.extra_efficiency;
            drive.shaft.set_oil(law.viscosity_factor(*rise));
        }
        // The second carriage's gearbox is the bridge's twin and warms with it.
        if let (Some(b), Some(rise)) = (self.side_b.as_mut(), rises.first()) {
            b.gearbox.efficiency = law.gearbox_efficiency(*rise) * b.extra_efficiency;
            b.shaft.set_oil(law.viscosity_factor(*rise));
        }
    }

    /// The payload's own stretch on the longest rope (`kinematic_stretch`), and the default's 0.1 mm.
    fn setpoint_slack(&self) -> f64 {
        (self.height - self.axes[2].min) * self.payload * self.g / self.ea + 1e-4
    }

    /// The load's sway on its rope at the pose: sqrt(g / L) / (2 pi), L = H - z.
    fn resonance_hz(&self, tool_pose: &[f64]) -> Option<f64> {
        self.sway_omega(self.height - tool_pose[2]).map(|w| w / TAU)
    }

    fn signal_infos(&self) -> Vec<SignalInfo> {
        let mut out = vec![
            SignalInfo::new("rope.tension", "group-rope", "quantity-rope-tension", "N"),
            SignalInfo::new("rope.shortfall", "group-rope", "quantity-rope-shortfall", "N"),
            SignalInfo::new("rope.length", "group-rope", "quantity-rope-length", "m"),
            SignalInfo::new("load.sway_angle", "group-load", "quantity-sway-angle", "rad"),
            SignalInfo::new("load.sway", "group-load", "quantity-sway", "m"),
            SignalInfo::new("load.residual_sway", "group-load", "quantity-residual-sway", "m"),
            SignalInfo::new("load.floor_force", "group-load", "quantity-floor-force", "N"),
            SignalInfo::new("hoist.brake_load", "group-hoist", "quantity-brake-load", "N m"),
        ];
        if self.sling.is_some() {
            out.push(SignalInfo::new("load.sling_angle", "group-load", "quantity-sling-angle", "rad"));
        }
        if self.skew.is_some() {
            out.extend([
                SignalInfo::new("bridge.skew_angle", "group-bridge", "quantity-skew-angle", "rad"),
                SignalInfo::new("bridge.travel_a", "group-bridge", "quantity-bridge-travel", "m"),
                SignalInfo::new("bridge.travel_b", "group-bridge", "quantity-bridge-travel", "m"),
                SignalInfo::new("bridge.torque_a", "group-bridge", "quantity-bridge-torque", "N m"),
                SignalInfo::new("bridge.torque_b", "group-bridge", "quantity-bridge-torque", "N m"),
            ]);
        }
        for motor in &self.motors {
            out.extend(TORQUE_PARTS.map(|part| {
                SignalInfo::new(
                    format!("motor.{}.torque_{part}", motor.id),
                    motor.key,
                    &format!("quantity-torque-{part}"),
                    "N m",
                )
            }));
        }
        for k in 0..3 {
            out.push(SignalInfo::new(
                format!("{LOSS_PREFIX}shaft_{}", MOTOR_IDS[k]),
                MOTOR_KEYS[k],
                "quantity-shaft-friction-power",
                "W",
            ));
            let quantity =
                if self.drives[k].coupled { "quantity-coupling-loss-power" } else { "quantity-gearbox-loss-power" };
            out.push(SignalInfo::new(format!("{LOSS_PREFIX}gearbox_{}", MOTOR_IDS[k]), MOTOR_KEYS[k], quantity, "W"));
        }
        out.push(SignalInfo::new(
            format!("{LOSS_PREFIX}rolling_bridge"),
            "group-bridge",
            "quantity-friction-power",
            "W",
        ));
        out.push(SignalInfo::new(
            format!("{LOSS_PREFIX}rolling_trolley"),
            "group-trolley",
            "quantity-friction-power",
            "W",
        ));
        out.push(SignalInfo::new(format!("{LOSS_PREFIX}rope"), "group-rope", "quantity-damping-power", "W"));
        out.push(SignalInfo::new(format!("{LOSS_PREFIX}hard_stops"), "group-crane", "quantity-damping-power", "W"));
        out.push(SignalInfo::new(format!("{LOSS_PREFIX}other"), "loss-other", "quantity-power-given-off", "W"));
        out.extend(ENERGY_PARTS.map(|part| {
            SignalInfo::new(format!("energy.{part}"), "group-energy", &format!("quantity-energy-{part}"), "J")
        }));
        out
    }

    fn base_signals(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        // The rigid rope's tension is the motion's; `half` below adds the rope's weight to it.
        let mut rope = self.rope(s);
        if !self.elastic {
            rope.tension = self.motion(s, torque).rope_state.tension;
            rope.drum = rope.tension;
        }
        let (d, _) = self.sway(s);
        let offset = d[0].hypot(d[1]);
        let residual = if self.dwelling { self.sway_amplitude(s) } else { 0.0 };
        let floor = if self.floor.is_some() { self.motion(s, torque).floor_force } else { 0.0 };
        // With the rope's own mass the tension grows down to up by the weight below: the signal is the drum's end
        // (the highest), the shortfall is judged at the hook's end (the least); `rope.tension` is their mean.
        let half = 0.5 * self.rope_mass * self.g * (self.height - s[LOAD + 2]);
        out.extend([
            rope.tension + half,
            (self.t_min - (rope.tension - half)).max(0.0),
            rope.length,
            offset.atan2(self.height - self.lower(s).0[2]),
            offset,
            residual,
            floor,
            self.brake_load(rope.drum + half),
        ]);
        if let Some(a0) = self.layout.sling {
            out.push((full::sling_dir(s[a0], s[a0 + 1])[0][2].abs()).min(1.0).acos());
        }
        if let (Some(_), Some(i)) = (self.skew, self.layout.skew) {
            let torque_of = |d: usize| {
                let j = self.drive(d);
                let rotor = self.layout.rotor.map_or(j.ratio() * self.travel(s, d).0, |r0| s[r0 + d]);
                j.ripple.apply(self.command(torque, d), rotor)
            };
            out.extend([s[i], self.travel(s, 0).0, self.travel(s, 3).0, torque_of(0), torque_of(3)]);
        }
    }

    /// Torque breakdown per motor: inertia (the axis's own mass and rotor), gravity (the rope's pull on the axis: the
    /// load's weight on the hoist, the swing's pull on the travel drives), friction (motor shaft, brake, gearbox loss,
    /// the wheels' rolling), the rest (ripple, stops). Then the loss powers and the energies.
    fn effect_signals(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        let motion = self.motion(s, torque);
        let (powers, at_rotor) = self.losses(s, torque, &motion);
        for k in 0..3 {
            let j = &self.drives[k];
            let a = motion.acc[k];
            let (speed, rotor_acc) = match (&motion.full, self.layout.rotor) {
                (Some(ex), Some(r0)) => (s[r0 + self.layout.drives + k], j.rotor_inertia * ex.rotor_acc[k]),
                _ => (j.ratio() * s[RATES + k], j.rotor_inertia * j.ratio() * a),
            };
            let held = j.brake.torque(self.brake_engaged, speed, s[BRAKES + k]).0;
            let parts = [
                self.masses[k] * a / j.ratio() + rotor_acc,
                -motion.rope[k] / j.ratio(),
                j.shaft.torque(speed) + held + at_rotor[k] + motion.rolling[k] / j.ratio(),
            ];
            out.extend(parts);
            out.push(torque[k] - parts.iter().sum::<f64>());
        }
        out.extend(powers);
        out.extend(self.energy_parts(s));
    }
    fn loss_powers(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        out.extend(self.losses(s, torque, &self.motion(s, torque)).0);
    }

    fn thermal_parts(&self) -> Vec<ThermalPart> {
        (0..3)
            .filter(|k| !self.drives[*k].coupled)
            .enumerate()
            .map(|(i, k)| ThermalPart {
                id: GEARBOX_IDS[k],
                kind: "gearbox",
                owner_key: MOTOR_KEYS[k],
                scene_part: "gearbox",
                scene_index: i,
            })
            .collect()
    }

    /// The runway's rails on their columns, the bridge's girder, the trolley with its drum, each drive's motor and
    /// gearbox, the rope coloured by its tension against the load's weight, the hook and the load under it.
    fn scene(&self, s: &[f64], out: &mut Scene) {
        out.parts.clear();
        out.labels.clear();
        // The rigid rope's tension needs the torques, which a scene does not have: it shows the load's weight.
        let mut rope = self.rope(s);
        if !self.elastic {
            rope.tension = self.load_mass() * self.g;
        }
        let (xc, y, h) = (s[0], s[1], self.height);
        // The trolley is on the girder, which a skew turns: at the sheave's x.
        let x = self.sheave(s)[0];
        let span = self.strokes[1];
        let (steel, dark) = ([0.55, 0.57, 0.6], [0.25, 0.25, 0.28]);
        let rail = h + 0.25;
        // The steel as boxes (a segment is a flat strip, for belts and the rope): rails on their columns, the girder.
        let length = self.strokes[0] + 1.0;
        for side in [-0.3, span + 0.3] {
            let rails = Shape::Box { size: [length, 0.15, 0.15] };
            out.parts.push(Part::new("rail", rails, [0.5 * self.strokes[0], side, rail], steel));
            for end in [-0.5, self.strokes[0] + 0.5] {
                let column = Shape::Box { size: [0.2, 0.2, rail - 0.075] };
                out.parts.push(Part::new("column", column, [end, side, 0.5 * (rail - 0.075)], steel));
            }
        }
        let yellow = [0.85, 0.65, 0.1];
        let (psi, _) = self.yaw(s);
        if self.skew.is_some() {
            // The girder yawed by psi (x = x_c + psi (y - y_c)): drawn upright along z, turned by -psi, then laid flat.
            let girder = Shape::Box { size: [0.3, 0.35, span + 0.9] };
            let mut part = Part::new("bridge", girder, [xc, -(rail + 0.25), 0.5 * span], yellow).turned(-psi);
            part.lay_flat();
            out.parts.push(part);
        } else {
            let girder = Shape::Box { size: [0.3, span + 0.9, 0.35] };
            out.parts.push(Part::new("bridge", girder, [xc, 0.5 * span, rail + 0.25], yellow));
        }
        out.parts.push(Part::new("trolley", Shape::Box { size: [0.6, 0.5, 0.3] }, [x, y, h + 0.15], dark));
        out.parts.push(
            Part::new(
                "drum",
                Shape::Cylinder { radius: self.drum_radius, length: 0.4 },
                [x + 0.15, y, h + 0.15],
                [0.75, 0.75, 0.78],
            )
            .turned(s[2] / self.drum_radius),
        );
        // Each drive: motor, its rotor's bar, gearbox; the bridge's at the girder's end, the others on the trolley.
        let side_a = self.travel(s, 0).0;
        let places = [[side_a - 0.45, -0.3, rail + 0.15], [x - 0.45, y + 0.15, h + 0.15], [x + 0.45, y, h + 0.15]];
        for (k, place) in places.iter().enumerate() {
            let j = &self.drives[k];
            out.parts.push(Part::new(
                sim_core::scene::MOTOR_PART,
                Shape::Box { size: [0.14, 0.14, 0.14] },
                add(*place, [0.0, 0.0, 0.2]),
                [0.9, 0.45, 0.1],
            ));
            out.parts.push(
                Part::new(
                    "rotor-bar",
                    Shape::Box { size: [0.12, 0.006, 0.02] },
                    add(*place, [0.0, 0.0, 0.3]),
                    [0.1; 3],
                )
                .turned(j.ratio() * s[k]),
            );
            out.parts.push(Part::new("gearbox", Shape::Box { size: [0.1, 0.1, 0.1] }, *place, dark));
            out.labels.push(Label {
                pos: add(*place, [0.0, 0.0, 0.45]),
                owner_key: MOTOR_KEYS[k],
                value: j.ratio() * s[RATES + k] * 60.0 / TAU,
                unit_key: "unit-rpm",
                decimals: 0,
            });
        }
        let load = [s[LOAD], s[LOAD + 1], s[LOAD + 2]];
        let sway = self.sway(s);
        let weight = (self.load_mass() * self.g).max(1.0);
        out.parts.push(
            Part::new("rope", Shape::Segment { to: load, width: 0.04 }, self.sheave(s), sim_core::scene::BELT_COLOR)
                .highlighted(((rope.tension - weight) / weight).clamp(-1.0, 1.0) as f32),
        );
        out.parts.push(Part::new("hook", Shape::Box { size: [0.12, 0.08, 0.16] }, load, [0.9, 0.75, 0.1]));
        if self.attached && self.payload > 0.0 {
            // With the sling the payload hangs at its end (its centre there, the sling a line to the hook block).
            let place = if self.sling.is_some() {
                let low = self.lower(s).0;
                out.parts.push(Part::new(
                    "sling",
                    Shape::Segment { to: low, width: 0.02 },
                    load,
                    sim_core::scene::BELT_COLOR,
                ));
                low
            } else {
                add(load, [0.0, 0.0, -0.3])
            };
            out.parts.push(Part::new("payload", Shape::Box { size: [0.6, 0.6, 0.4] }, place, [0.2, 0.5, 0.75]));
        }
        out.labels.push(Label {
            pos: add(self.sheave(s), [0.0, 0.0, -0.5 * rope.length]),
            owner_key: "group-rope",
            value: rope.tension,
            unit_key: "unit-n",
            decimals: 0,
        });
        out.labels.push(Label {
            pos: add(load, [0.4, 0.0, 0.0]),
            owner_key: "group-load",
            value: sway.0[0].hypot(sway.0[1]) * 1e3,
            unit_key: "unit-mm",
            decimals: 0,
        });
    }

    fn scene_point(&self, tool: &[f64]) -> Option<[f64; 3]> {
        Some([tool[0], tool[1], tool[2]])
    }
    fn collision_bodies(&self, s: &[f64]) -> Vec<sim_core::Aabb> {
        let mut scene = Scene::default();
        self.scene(s, &mut scene);
        sim_core::scene::scene_boxes(&scene.parts, &["hook", "payload"])
    }
}

#[cfg(test)]
mod tests;
#[cfg(test)]
mod tests_full;
