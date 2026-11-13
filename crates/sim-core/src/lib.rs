//! Core: fixed-step integrator, state vector, signals, config loading, test harness. Knows no physics.

pub mod config;
pub mod i18n;
pub mod integrate;
pub mod machine;
pub mod scene;
pub mod testkit;

pub use config::{Config, ConfigError};
pub use i18n::Translations;
pub use integrate::{Rk4, advance};
pub use machine::{AxisInfo, BeltSection, RatingCheck, ENERGY_PARTS, FRICTION_SIGNALS, FrictionContact, LOSS_PREFIX, Machine, MachineEntry, Mode, MotorInfo, SignalInfo, TEMPERATURE_PREFIX, THERMAL_KINDS, ThermalPart, TORQUE_PARTS};
pub use scene::{Aabb, Arrow, Label, Obstacle, Part, Scene, Shape};
