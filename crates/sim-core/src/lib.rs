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
pub use machine::{AxisInfo, Machine, MachineEntry, MotorInfo, SignalInfo};
pub use scene::{Label, Part, Scene, Shape};
