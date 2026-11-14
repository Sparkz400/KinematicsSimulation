//! What trips a drive and how it stops then, as servo drives do it (Siemens SINAMICS names the parameters given here):
//! each drive watches its following error against a window (p2546), its following error at standstill against a
//! tighter one (p2542, after p2543), whether it is held at its torque limit at low speed for longer than a time (F07900
//! "motor blocked", p2175 and p2177), and optionally its torque against the model's expected torque (as a robot
//! controller detects a collision). The first monitor that trips a drive stops all of them, each by its own fault
//! reaction (`Reaction`). On with effects.drive_monitoring; the controller runs it (`Controller::update`).

use crate::drive::{Drive, DriveParams};
use sim_core::{Config, ConfigError};

type Result<T> = std::result::Result<T, ConfigError>;

/// How a drive stops when the drives trip.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum Reaction {
    /// OFF1: the speed setpoint ramps down on the ramp-function generator's ramp (p1121), the speed controller
    /// following it; at standstill the pulses go off.
    Off1,
    /// OFF3: the same on the fast-stop ramp (p1135).
    Off3,
    /// STO (as OFF2): the pulses go off at once and the motor coasts.
    Sto,
    /// The speed setpoint drops to zero at once (OFF3 with a ramp time of 0): the speed controller brakes at the torque
    /// limit, which the current limit sets.
    CurrentLimit,
}

impl Reaction {
    /// As `drive.fault_reaction` names it.
    pub fn parse(name: &str) -> Option<Self> {
        Some(match name {
            "off1" => Self::Off1,
            "off3" => Self::Off3,
            "sto" => Self::Sto,
            "current_limit" => Self::CurrentLimit,
            _ => return None,
        })
    }

    /// Translation key of a line telling how a drive (the argument) stops by it.
    pub fn key(self) -> &'static str {
        match self {
            Self::Off1 => "reaction-off1",
            Self::Off3 => "reaction-off3",
            Self::Sto => "reaction-sto",
            Self::CurrentLimit => "reaction-current-limit",
        }
    }
}

/// The monitor that tripped the drives.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum Monitor {
    FollowingError,
    Standstill,
    Blocked,
    TorqueBand,
}

impl Monitor {
    /// Translation key of its name.
    pub fn key(self) -> &'static str {
        match self {
            Self::FollowingError => "monitor-following-error",
            Self::Standstill => "monitor-standstill",
            Self::Blocked => "monitor-blocked",
            Self::TorqueBand => "monitor-torque-band",
        }
    }
}

/// One drive's fault reaction and monitoring, from `[drive]` (and its motor's `[drive.<id>]`). Angles and speeds at the
/// motor, rad and rad/s (m and m/s for a linear motor).
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Settings {
    pub reaction: Reaction,
    /// The ramps' decelerations, rad/s^2: the rated speed over `drive.off1_ramp_s` and `off3_ramp_s`, as p1121 and
    /// p1135 are times from the reference speed (here the rated speed) to standstill.
    pub off1_deceleration: f64,
    pub off3_deceleration: f64,
    /// Once its ramp is down, a drive takes its pulses off below this speed (p1226).
    pub pulse_off_speed: f64,
    pub following_window: f64,
    pub standstill_window: f64,
    pub standstill_time: f64,
    pub blocked_speed: f64,
    pub blocked_time: f64,
    /// The band around the expected torque, as a share of the peak torque; 0: that monitor is off.
    pub torque_band: f64,
}

impl Settings {
    /// From a motor's config (`catalog::motor_config`). A config without the keys (one written before 14 November
    /// 2026) gets the values of the shipped configs.
    pub fn from_config(cfg: &Config, p: &DriveParams) -> Result<Self> {
        let value = |key: &str, default: f64| -> Result<f64> {
            let path = format!("drive.{key}");
            if cfg.get(&path).is_err() {
                return Ok(default);
            }
            cfg.non_negative(&path)
        };
        let reaction = match cfg.get("drive.fault_reaction") {
            Err(_) => Reaction::Off3,
            Ok(v) => v
                .as_str()
                .and_then(Reaction::parse)
                .ok_or_else(|| ConfigError::Invalid("drive.fault_reaction".into()))?,
        };
        let deceleration = |key: &str, default: f64| -> Result<f64> {
            let ramp = value(key, default)?;
            Ok(if ramp > 0.0 { p.rated_speed / ramp } else { f64::INFINITY })
        };
        Ok(Self {
            reaction,
            off1_deceleration: deceleration("off1_ramp_s", 1.0)?,
            off3_deceleration: deceleration("off3_ramp_s", 0.1)?,
            pulse_off_speed: value("pulse_off_speed_rad_s", 2.09)?,
            following_window: value("following_error_window_rad", 1.0)?,
            standstill_window: value("standstill_window_rad", 0.1)?,
            standstill_time: value("standstill_time_s", 1.0)?,
            blocked_speed: value("blocked_speed_rad_s", 12.57)?,
            blocked_time: value("blocked_time_s", 3.0)?,
            torque_band: value("torque_band", 0.0)?,
        })
    }

    /// The deceleration of its stop ramp, rad/s^2: infinite at the current limit, `None` for STO (no ramp).
    pub fn deceleration(&self) -> Option<f64> {
        match self.reaction {
            Reaction::Off1 => Some(self.off1_deceleration),
            Reaction::Off3 => Some(self.off3_deceleration),
            Reaction::CurrentLimit => Some(f64::INFINITY),
            Reaction::Sto => None,
        }
    }
}

/// One drive's monitoring as it runs: how long its setpoint has stood still and how long it has been held at its
/// torque limit at low speed.
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Watch {
    still: f64,
    blocked: f64,
}

impl Watch {
    /// One base step of `dt` of the drive's monitoring, after its loops, its motor's peak torque `peak`: the monitor
    /// that trips it, the first in the order of `Monitor`.
    pub fn check(&mut self, s: &Settings, d: &Drive, peak: f64, dt: f64) -> Option<Monitor> {
        let error = d.following_error.abs();
        self.still = if d.vel_set == 0.0 && d.acc_set == 0.0 { self.still + dt } else { 0.0 };
        self.blocked = if d.at_limit && d.measured_speed().abs() < s.blocked_speed { self.blocked + dt } else { 0.0 };
        if error > s.following_window {
            Some(Monitor::FollowingError)
        } else if self.still >= s.standstill_time && error > s.standstill_window {
            Some(Monitor::Standstill)
        } else if self.blocked >= s.blocked_time {
            Some(Monitor::Blocked)
        } else if s.torque_band > 0.0 && (d.torque - d.torque_ff).abs() > s.torque_band * peak {
            Some(Monitor::TorqueBand)
        } else {
            None
        }
    }
}

/// The first trip since the drives were last enabled: which monitor of which drive, when (s), and the tool pose then;
/// how far each tool axis has gone since (the stopping distance, in the axis' unit, final once `stopped`); and when
/// the machine stood (s): every drive off and every motor below its pulse-off speed.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Fault {
    pub monitor: Monitor,
    pub drive: usize,
    pub time: f64,
    pub pose: Vec<f64>,
    pub stopping: Vec<f64>,
    pub stopped: Option<f64>,
}

/// A stop ramp's next speed setpoint after `dt`: from `speed` towards zero at `deceleration`, not beyond.
pub fn ramp(speed: f64, deceleration: f64, dt: f64) -> f64 {
    speed.signum() * (speed.abs() - deceleration * dt).max(0.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// By hand: from 300 rad/s on a 0.1 s ramp from the rated speed of 314 rad/s (3140 rad/s^2), 0.01 s takes 31.4 rad/s
    /// off; a ramp near its end stops at zero and does not swing through.
    #[test]
    fn a_stop_ramp_takes_the_deceleration_off_and_ends_at_zero() {
        assert!((ramp(300.0, 314.0 / 0.1, 0.01) - 268.6).abs() < 1e-12);
        assert!((ramp(-300.0, 3140.0, 0.01) + 268.6).abs() < 1e-12);
        assert_eq!(ramp(10.0, 3140.0, 0.01), 0.0);
        assert_eq!(ramp(10.0, f64::INFINITY, 0.01), 0.0);
    }

    fn settings() -> Settings {
        Settings {
            reaction: Reaction::Off3,
            off1_deceleration: 314.0,
            off3_deceleration: 3140.0,
            pulse_off_speed: 2.09,
            following_window: 1.0,
            standstill_window: 0.1,
            standstill_time: 1.0,
            blocked_speed: 12.57,
            blocked_time: 3.0,
            torque_band: 0.0,
        }
    }

    /// The number of steps of 1/1024 s (exact in binary, so that the times add up exactly) until `drive` trips, and on
    /// what; `None` within 10 s.
    fn steps_to_trip(s: &Settings, drive: &Drive) -> Option<(usize, Monitor)> {
        let mut watch = Watch::default();
        (1..=10_240).find_map(|k| watch.check(s, drive, 3.8, 1.0 / 1024.0).map(|m| (k, m)))
    }

    /// By hand, in steps of 1/1024 s: a following error of 1.01 rad is beyond the 1 rad window at once. 0.2 rad, within it,
    /// is beyond the standstill window of 0.1 rad once the setpoint has stood still 1 s, at the 1024th step (moving, it
    /// never trips). A drive held at its torque limit at standstill (below 12.57 rad/s as measured) trips "blocked" after
    /// 3 s, the 3072nd step. Torque 2.0 N m against an expected 0.5 N m is 1.5 N m
    /// off: beyond a band of 0.3 times the peak torque 3.8 N m (1.14), within one of 0.4 (1.52).
    #[test]
    fn each_monitor_trips_at_its_hand_counted_step() {
        let s = settings();
        let drive = |f: &dyn Fn(&mut Drive)| {
            let mut d = Drive::default();
            f(&mut d);
            d
        };
        assert_eq!(steps_to_trip(&s, &drive(&|d| d.following_error = -1.01)), Some((1, Monitor::FollowingError)));
        assert_eq!(steps_to_trip(&s, &drive(&|d| d.following_error = 0.2)), Some((1024, Monitor::Standstill)));
        let moving = drive(&|d| (d.following_error, d.vel_set) = (0.2, 1.0));
        assert_eq!(steps_to_trip(&s, &moving), None);
        assert_eq!(steps_to_trip(&s, &drive(&|d| d.at_limit = true)), Some((3072, Monitor::Blocked)));
        let off = drive(&|d| (d.torque, d.torque_ff) = (2.0, 0.5));
        assert_eq!(steps_to_trip(&Settings { torque_band: 0.3, ..settings() }, &off), Some((1, Monitor::TorqueBand)));
        assert_eq!(steps_to_trip(&Settings { torque_band: 0.4, ..settings() }, &off), None);
    }
}
