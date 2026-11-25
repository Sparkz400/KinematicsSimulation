//! The drive parameters of a tuned machine: for each motor, the numbers of the config's drive settings as the drive takes
//! them, with the parameter's number, name and unit (SINAMICS S120 servo and S210). The mapping is data, the tables
//! `catalog/siemens/parameters_s120.csv` and `parameters_s210.csv`, each row with the page of the Siemens document it comes
//! from; this module reads the config as resolved (the parts applied, each motor's own `[drive.<id>]` over the shared one)
//! and converts. A setting the table has no parameter for is listed as such, never with a guessed number.
//!
//! Rotary servo motors only: a linear motor's gains have other units (N s/m), a stepper or an induction motor has no
//! such cascade.

use crate::catalog::{motor_config, resolve, rows, table};
use sim_core::{Config, ConfigError, MachineEntry};

/// The drive families a parameter list exists for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Family {
    /// SINAMICS S120 servo on a Motor Module (List Manual LH1, 07/2018).
    S120,
    /// SINAMICS S210 (6SL3210 converters; Operating Instructions 09/2022).
    S210,
}

impl Family {
    /// `s120` or `s210`, as `--family` says it.
    pub fn parse(text: &str) -> Option<Self> {
        match text {
            "s120" => Some(Self::S120),
            "s210" => Some(Self::S210),
            _ => None,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Self::S120 => "s120",
            Self::S210 => "s210",
        }
    }

    fn mapping(self) -> &'static str {
        table(match self {
            Self::S120 => "siemens/parameters_s120.csv",
            Self::S210 => "siemens/parameters_s210.csv",
        })
    }

    /// The family a drive's article number belongs to: an S210 converter of `converters_s210.csv` (not the "S210 (New)"
    /// 6SL5310, which the Operating Instructions do not cover) or an S120 Motor Module of `motor_modules_s120.csv`.
    pub fn of_article(article: &str) -> Option<Self> {
        let article_in = |file: &'static str| rows(table(file)).into_iter().find(|r| r["part_number"] == article);
        if article_in("siemens/converters_s210.csv").is_some_and(|r| r["series"] == "S210") {
            Some(Self::S210)
        } else if article_in("siemens/motor_modules_s120.csv").is_some() {
            Some(Self::S120)
        } else {
            None
        }
    }
}

/// How a row's parameter relates to what the manual prints.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Basis {
    /// The parameter's quantity and unit as the manual prints them.
    Printed,
    /// A relation worked out from what the manual prints (the Kv unit, the notch damping, a reference speed).
    Derived,
    /// No parameter: the table has none for this setting.
    None,
}

impl Basis {
    pub fn name(self) -> &'static str {
        match self {
            Self::Printed => "printed",
            Self::Derived => "derived",
            Self::None => "none",
        }
    }
}

/// One line of the list.
#[derive(Debug, Clone, PartialEq)]
pub struct Row {
    pub motor: String,
    /// The config key without its table (`velocity_kp_nm_s_rad`), and its value and unit.
    pub key: &'static str,
    pub value: f64,
    pub unit: &'static str,
    /// The parameter ("p1460"), its name as the manual prints it and its unit; empty with `Basis::None`.
    pub parameter: String,
    pub name: String,
    pub parameter_value: Option<f64>,
    pub parameter_unit: String,
    pub basis: Basis,
}

/// Why a list could not be made.
#[derive(Debug, Clone, PartialEq)]
pub enum Error {
    Config(ConfigError),
    /// The motor has no drive part and no family was named.
    NoDrive(String),
    /// The motor's drive part is none of the two families (its article).
    UnknownDrive(String),
    /// The motor is not a rotary servo motor (its id).
    NotRotaryServo(String),
}

impl From<ConfigError> for Error {
    fn from(e: ConfigError) -> Self {
        Self::Config(e)
    }
}

/// The drive settings, with the config's unit and the effect switch the simulator needs on for the value to act (""
/// always). A setting of a motor whose effect is off is left out: the drive would not have used the number the
/// simulation tuned. The order is the list's.
pub const SETTINGS: &[(&str, &str, &str)] = &[
    ("kv_1_s", "1/s", ""),
    ("velocity_kp_nm_s_rad", "N m s/rad", ""),
    ("velocity_tn_s", "s", ""),
    ("velocity_feedforward_factor", "factor", "velocity_feedforward"),
    ("torque_feedforward_factor", "factor", "torque_feedforward"),
    ("precontrol_balance_s", "s", ""),
    ("current_kp_v_a", "V/A", ""),
    ("current_tn_s", "s", ""),
    ("peak_torque_nm", "N m", ""),
    ("max_speed_rad_s", "rad/s", ""),
    ("velocity_filter_s", "s", "velocity_filter"),
    ("torque_filter_s", "s", "torque_filter"),
    ("notch_frequency_hz", "Hz", "notch_filter"),
    ("notch_bandwidth_hz", "Hz", "notch_filter"),
    ("off1_ramp_s", "s", "drive_monitoring"),
    ("off3_ramp_s", "s", "drive_monitoring"),
    ("pulse_off_speed_rad_s", "rad/s", "drive_monitoring"),
    ("following_error_window_rad", "rad", "drive_monitoring"),
    ("standstill_window_rad", "rad", "drive_monitoring"),
    ("standstill_time_s", "s", "drive_monitoring"),
    ("blocked_speed_rad_s", "rad/s", "drive_monitoring"),
    ("blocked_time_s", "s", "drive_monitoring"),
    ("ss1_time_s", "s", "drive_monitoring"),
];

/// The drive part a motor has: its own `[parts.<id>] drive` over the shared `[parts] drive`, "" for none.
fn drive_article<'a>(cfg: &'a Config, id: &str) -> &'a str {
    let text = |path: String| cfg.get(&path).ok().and_then(|v| v.as_str());
    text(format!("parts.{id}.drive")).or_else(|| text("parts.drive".into())).unwrap_or("")
}

/// The parameter list of every motor of the machine, from `cfg` as the config file states it (the drive part is read
/// before the parts are applied). `family` names the drive family where the config has no drive part (or to override it).
pub fn list(entry: &MachineEntry, cfg: &Config, family: Option<Family>) -> Result<Vec<Row>, Error> {
    list_resolved(entry, cfg, &resolve(entry, cfg)?, family)
}

/// `list` with the resolved config given: a front end that sets values on top of the resolved parts (`--set`, which
/// overrides the retuned gain) passes its own. `cfg` is still the config as the file states it, for the drive part.
pub fn list_resolved(
    entry: &MachineEntry,
    cfg: &Config,
    resolved: &Config,
    family: Option<Family>,
) -> Result<Vec<Row>, Error> {
    let machine = (entry.build)(resolved)?;
    let mut out = Vec::new();
    for (index, motor) in machine.motors().iter().enumerate() {
        let own = motor_config(resolved, motor.id)?;
        let servo = own.get("motor.kind").ok().and_then(|v| v.as_str()).is_none_or(|k| k == "servo");
        if machine.motor_linear(index) || !servo {
            return Err(Error::NotRotaryServo(motor.id.into()));
        }
        let article = drive_article(cfg, motor.id);
        let family = match (family, article) {
            (Some(family), _) => family,
            (None, "") => return Err(Error::NoDrive(motor.id.into())),
            (None, article) => Family::of_article(article).ok_or_else(|| Error::UnknownDrive(article.into()))?,
        };
        let mapping = rows(family.mapping());
        for &(key, unit, effect) in SETTINGS {
            let Ok(value) = own.f64(&format!("drive.{key}")) else { continue };
            if !effect.is_empty() && !own.bool(&format!("effects.{effect}")).unwrap_or(false) {
                continue;
            }
            let found: Vec<_> = mapping.iter().filter(|r| r["config_key"] == key).collect();
            let row = |parameter: &str, name: &str, converted, parameter_unit: &str, basis| Row {
                motor: motor.id.into(),
                key,
                value,
                unit,
                parameter: parameter.into(),
                name: name.into(),
                parameter_value: converted,
                parameter_unit: parameter_unit.into(),
                basis,
            };
            if found.is_empty() {
                out.push(row("", "", None, "", Basis::None));
            }
            for r in found {
                let basis = if r["basis"] == "printed" { Basis::Printed } else { Basis::Derived };
                out.push(row(r["parameter"], r["name"], Some(convert(r, &own, value)?), r["unit"], basis));
            }
        }
    }
    Ok(out)
}

/// The parameter's value for the config's `value` by the row's factor and rule (see the tables' notes).
fn convert(r: &crate::catalog::Row, own: &Config, value: f64) -> Result<f64, ConfigError> {
    let factor: f64 =
        r["factor"].parse().map_err(|_| ConfigError::Invalid(format!("parameters: factor {}", r["factor"])))?;
    Ok(match r["rule"] {
        "" => factor * value,
        "constant" => factor,
        "zero" => 0.0,
        "notch_damping" => factor * value / (2.0 * own.positive("drive.notch_frequency_hz")?),
        "ramp_time" => {
            // The simulator's ramps run from the rated speed, the drive's from its maximum speed (p1082); the motor's
            // maximum speed is the no-load speed where the config has none (`DriveParams::from_config`).
            let max = own.positive("drive.max_speed_rad_s").or_else(|_| own.positive("drive.no_load_speed_rad_s"))?;
            factor * value * max / own.positive("drive.rated_speed_rad_s")?
        }
        other => return Err(ConfigError::Invalid(format!("parameters: rule {other}"))),
    })
}
