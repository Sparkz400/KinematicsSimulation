//! The component catalogue as the simulator uses it: a part named by its article number puts its
//! catalogue values into a config, and a search tries every motor (and every axis) on a machine's cycle.
//!
//! The tables of `catalog/` (schema 1: SI units, see `catalog/README.md`) are compiled in, so a front end needs no
//! path to them; `use_directory` reads them from another copy of the catalogue instead (`--catalog DIR`).

use crate::analysis::{CycleLoad, Verdict, analyse_cycle_with, verdicts};
use crate::sim::Sim;
use sim_core::{Config, ConfigError, MachineEntry};
use std::collections::HashMap;
use std::f64::consts::PI;
use std::sync::OnceLock;

/// The catalogue's schema version this simulator reads. A catalogue of another version is refused.
pub const SCHEMA_VERSION: i64 = 1;

macro_rules! tables {
    ($($name:literal),* $(,)?) => { [$(($name, include_str!(concat!("../../../catalog/", $name)))),*] };
}

/// The tables the simulator reads, by their path in the catalogue, with the compiled-in copy.
const TABLES: [(&str, &str); 24] = tables![
    "siemens/motors_s210.csv",
    "siemens/motors_s210_oi.csv",
    "siemens/dynamic_braking_1fk2.csv",
    "siemens/encoders_1fk2_1ft2.csv",
    "siemens/dc_link_trip_s120.csv",
    "siemens/motors_1fk2_s120.csv",
    "siemens/motors_1ft2_s120.csv",
    "siemens/motors_s120.csv",
    "siemens/motors_s120_electrical.csv",
    "siemens/converters_s210.csv",
    "siemens/motor_modules_s120.csv",
    "siemens/braking_s210.csv",
    "siemens/braking_s120.csv",
    "siemens/dc_link_measured.csv",
    "siemens/gearbox_nrb.csv",
    "siemens/gearbox_nrk.csv",
    "siemens/gearbox_nlc.csv",
    "siemens/gearbox_mass.csv",
    "megadyne/belts.csv",
    "megadyne/belt_tooth_load.csv",
    "festo/belt_axes.csv",
    "festo/belt_axis_torque_formula.csv",
    "bahr/belt_axes.csv",
    "catalog.toml",
];

/// The tables read from a directory by `use_directory`; unset, the compiled-in ones are used.
static LOADED: OnceLock<HashMap<&'static str, &'static str>> = OnceLock::new();

fn table(name: &str) -> &'static str {
    let compiled = || TABLES.iter().find(|(n, _)| *n == name).map(|(_, text)| *text).expect("a table of TABLES");
    LOADED.get().and_then(|t| t.get(name).copied()).unwrap_or_else(compiled)
}

/// Thermal time constants come from the configuration manuals, which cover most of the same motors.
/// The same files are the 1FK2 and 1FT2 on a SINAMICS S120.
fn motor_manuals() -> [&'static str; 2] {
    [table("siemens/motors_1fk2_s120.csv"), table("siemens/motors_1ft2_s120.csv")]
}

fn gearboxes() -> [&'static str; 3] {
    [table("siemens/gearbox_nrb.csv"), table("siemens/gearbox_nrk.csv"), table("siemens/gearbox_nlc.csv")]
}

/// Reads the catalogue from a directory in place of the compiled-in one, for the rest of the process: a copy of
/// `catalog/` (its own repository, or a newer edition of it). Every table the simulator reads must be there, and
/// `catalog.toml` must have the `schema_version` this simulator reads. Call it before any part is looked up;
/// a second call is refused.
pub fn use_directory(dir: &std::path::Path) -> Result<(), ConfigError> {
    let read = |name: &str| {
        std::fs::read_to_string(dir.join(name))
            .map_err(|e| ConfigError::Part("error-catalog-read", format!("{}: {e}", dir.join(name).display())))
    };
    let manifest: toml::Table = read("catalog.toml")?
        .parse()
        .map_err(|e| ConfigError::Part("error-catalog-read", format!("catalog.toml: {e}")))?;
    match manifest.get("schema_version").and_then(toml::Value::as_integer) {
        Some(SCHEMA_VERSION) => {}
        other => return Err(ConfigError::Part("error-catalog-version", format!("{other:?}, {}", dir.display()))),
    }
    let mut loaded = HashMap::new();
    for (name, _) in TABLES {
        // Read once and kept for the life of the process, as the compiled-in tables are.
        loaded.insert(name, &*Box::leak(read(name)?.into_boxed_str()));
    }
    LOADED.set(loaded).map_err(|_| ConfigError::Part("error-catalog-read", "the catalogue was already chosen".into()))
}

/// The supplies a motor can be named for: the S210 data at 230 V or 400 V, or the motor on a SINAMICS S120
/// (400 V line). "s120" takes the 1FK7 and 1FT7 of `motors_s120.csv` and the 1FK2 and 1FT2 of their S120 manuals.
pub const SUPPLIES: [&str; 3] = ["230v", "400v", "s120"];

type Row = HashMap<&'static str, &'static str>;

/// The rows of a catalogue file; lines starting with '#' are notes.
fn rows(text: &'static str) -> Vec<Row> {
    let mut lines = text.lines().filter(|l| !l.starts_with('#'));
    let header: Vec<&str> = lines.next().unwrap_or("").split(',').collect();
    lines.map(|line| header.iter().copied().zip(line.split(',')).collect()).collect()
}

fn number(row: &Row, column: &str) -> Option<f64> {
    row.get(column)?.parse().ok()
}

/// `what` is "motor", "gearbox", "drive", "belt" or "axis": the kind of part, which the message names in the user's language.
fn unknown(what: &str, name: &str) -> ConfigError {
    let key = match what {
        "motor" => "error-config-part-motor",
        "gearbox" => "error-config-part-gearbox",
        "drive" => "error-config-part-drive",
        "belt" => "error-config-part-belt",
        _ => "error-config-part-axis",
    };
    ConfigError::Part(key, name.into())
}

/// A catalogue value (SI) in a config's unit, `factor` of them per SI unit, rounded to 12 significant digits: the
/// catalogue's 0.00022 F is the 220 µF it was printed as, not 219.99999999999997.
fn in_unit(si: f64, factor: f64) -> f64 {
    let value = si * factor;
    format!("{value:.11e}").parse().unwrap_or(value)
}

fn set(cfg: &mut Config, path: &str, value: f64) -> Result<(), ConfigError> {
    cfg.set_text(&format!("{path} = {value:?}"))
}

/// The base load current of an S210 converter per its rated current [estimate]: the lowest share of the S120 Motor
/// Modules of up to 30 A, the range of the S210, in `motor_modules_s120.csv` (15.3 A of 18 A; 0.85 to 0.87 there).
pub const S210_BASE_LOAD_SHARE: f64 = 0.85;

/// Sets a value only where the config has the key: ratings a config leaves out are not judged.
fn set_if_present(cfg: &mut Config, path: &str, value: f64) -> Result<(), ConfigError> {
    if cfg.get(path).is_ok() { set(cfg, path, value) } else { Ok(()) }
}

/// Article numbers of the SINAMICS S210 motors, e.g. "1FK2205-4AF".
pub fn motor_articles() -> Vec<&'static str> {
    rows(table("siemens/motors_s210.csv")).iter().map(|r| r["part_number"]).collect()
}

/// Article numbers of the motors that can be named for a supply (`SUPPLIES`): the S210 motors for "230v" and
/// "400v" (not every one is offered at both), and for "s120" the 1FK7 and 1FT7 that have a data sheet and the 1FK2
/// and 1FT2 of the S120 manuals that have a rotor inertia.
pub fn motor_articles_for(supply: &str) -> Vec<&'static str> {
    match supply {
        "s120" => {
            let sheets: Vec<&str> =
                rows(table("siemens/motors_s120_electrical.csv")).iter().map(|r| r["part_number"]).collect();
            let mut out: Vec<&str> = rows(table("siemens/motors_s120.csv"))
                .iter()
                .map(|r| r["part_number"])
                .filter(|a| sheets.contains(a))
                .collect();
            out.extend(
                motor_manuals()
                    .into_iter()
                    .flat_map(|t| rows(t))
                    .filter(|r| number(r, "rotor_inertia_kgm2").is_some())
                    .map(|r| r["part_number"]),
            );
            out
        }
        _ => motor_articles(),
    }
}

/// What a motor brings into a config, from whichever catalogue file has it.
struct MotorData {
    rotor_kgm2: f64,
    /// Rotor inertia and weight with the holding brake, where the catalogue gives them.
    brake: Option<(f64, f64)>,
    /// The highest speed: the mechanical limit and, where the catalogue gives it, what the converter allows.
    max_speed: Option<f64>,
    static_torque: f64,
    stall_current: f64,
    max_torque: f64,
    max_current: f64,
    rated_current: f64,
    rated_torque: f64,
    rated_speed: f64,
    thermal_seconds: Option<f64>,
    /// Phase resistance at 20 °C in ohms and rotating field inductance in henries, where a data sheet prints them.
    winding: Option<(f64, f64)>,
    /// Pole pairs, where the catalogue prints them (`motors_s120.csv`, the 1FK7 and 1FT7).
    pole_pairs: Option<f64>,
    mass: f64,
}

fn motor_data(article: &str, supply: &str) -> Result<MotorData, ConfigError> {
    let missing = || unknown("motor", &format!("{article}, {supply}"));
    if supply == "s120" {
        if let Some(m) = rows(table("siemens/motors_s120.csv")).into_iter().find(|r| r["part_number"] == article) {
            let sheet = rows(table("siemens/motors_s120_electrical.csv"))
                .into_iter()
                .find(|r| r["part_number"] == article)
                .ok_or_else(missing)?;
            let get = |row: &Row, column: &str| number(row, column).ok_or_else(missing);
            let max_speed = ["max_speed_mechanical_rad_s", "max_speed_inverter_rad_s"]
                .iter()
                .filter_map(|c| number(&sheet, c))
                .reduce(f64::min);
            return Ok(MotorData {
                rotor_kgm2: get(&m, "rotor_inertia_kgm2")?,
                // motors_s120.csv has no brake data.
                brake: None,
                max_speed,
                static_torque: get(&m, "static_torque_nm")?,
                stall_current: get(&m, "stall_current_a")?,
                max_torque: get(&sheet, "max_torque_nm")?,
                max_current: get(&sheet, "max_current_a")?,
                rated_current: get(&m, "rated_current_a")?,
                rated_torque: get(&m, "rated_torque_nm")?,
                rated_speed: get(&m, "rated_speed_rad_s")?,
                thermal_seconds: number(&sheet, "thermal_time_constant_s"),
                winding: number(&sheet, "phase_resistance_20c_ohm").zip(number(&sheet, "phase_inductance_h")),
                pole_pairs: number(&m, "pole_pairs"),
                mass: get(&m, "mass_kg")?,
            });
        }
        let m = motor_manuals()
            .into_iter()
            .flat_map(|t| rows(t))
            .find(|r| r["part_number"] == article)
            .ok_or_else(|| unknown("motor", article))?;
        let get = |column: &str| number(&m, column).ok_or_else(missing);
        return Ok(MotorData {
            rotor_kgm2: get("rotor_inertia_kgm2")?,
            brake: number(&m, "rotor_inertia_with_brake_kgm2").zip(number(&m, "mass_with_brake_kg")),
            max_speed: number(&m, "max_speed_inverter_rad_s"),
            static_torque: get("static_torque_nm")?,
            stall_current: get("stall_current_a")?,
            max_torque: get("max_torque_nm")?,
            max_current: get("max_current_a")?,
            rated_current: get("rated_current_a")?,
            rated_torque: get("rated_torque_nm")?,
            rated_speed: get("rated_speed_rad_s")?,
            thermal_seconds: number(&m, "thermal_time_constant_s"),
            winding: None,
            pole_pairs: None,
            mass: get("mass_kg")?,
        });
    }
    let motor = rows(table("siemens/motors_s210.csv"))
        .into_iter()
        .find(|r| r["part_number"] == article)
        .ok_or_else(|| unknown("motor", article))?;
    let get = |column: &str| number(&motor, column).ok_or_else(missing);
    let rated = |quantity: &str, unit: &str| get(&format!("rated_{quantity}_{supply}_{unit}"));
    let manual = motor_manuals().into_iter().flat_map(|text| rows(text)).find(|r| r["part_number"] == article);
    // The S210 operating instructions have the 230 V windings the manual lacks; where both have a motor, the newer
    // manual's value (two differ, see motors_s210_oi.meta.toml).
    let instructions = || rows(table("siemens/motors_s210_oi.csv")).into_iter().find(|r| r["part_number"] == article);
    Ok(MotorData {
        rotor_kgm2: get("rotor_inertia_kgm2")?,
        brake: number(&motor, "rotor_inertia_with_brake_kgm2").zip(number(&motor, "mass_with_brake_kg")),
        max_speed: number(&motor, "max_speed_rad_s"),
        static_torque: get("static_torque_nm")?,
        stall_current: get("stall_current_a")?,
        max_torque: get("max_torque_nm")?,
        max_current: get("max_current_a")?,
        rated_current: rated("current", "a")?,
        rated_torque: rated("torque", "nm")?,
        rated_speed: rated("speed", "rad_s")?,
        thermal_seconds: manual
            .as_ref()
            .and_then(|r| number(r, "thermal_time_constant_s"))
            .or_else(|| instructions().and_then(|r| number(&r, "thermal_time_constant_s"))),
        winding: None,
        pole_pairs: None,
        mass: get("mass_kg")?,
    })
}

/// Weight of a motor without brake, for the supply its data is taken for (`SUPPLIES`).
pub fn motor_mass(article: &str, supply: &str) -> Option<f64> {
    motor_data(article, supply).ok().map(|m| m.mass)
}

/// Weight of a motor with the brake option a config chooses (`parts.motor_brake`, see `apply_motor`); `None` for a
/// motor the catalogue does not have, or has no brake data of where the brake is chosen.
pub fn motor_mass_for(cfg: &Config, article: &str, supply: &str) -> Option<f64> {
    let m = motor_data(article, supply).ok()?;
    if brake_chosen(cfg).ok()? { m.brake.map(|(_, mass)| mass) } else { Some(m.mass) }
}

/// `parts.motor_brake`: whether the motor has its holding brake; false where the config does not say.
fn brake_chosen(cfg: &Config) -> Result<bool, ConfigError> {
    if cfg.get("parts.motor_brake").is_ok() { cfg.bool("parts.motor_brake") } else { Ok(false) }
}

/// The motor's highest speed in rad/s from the catalogue (the lower of the mechanical limit and what the converter
/// allows, where both are given); `None` where the catalogue has none.
pub fn motor_max_speed(article: &str, supply: &str) -> Option<f64> {
    motor_data(article, supply).ok()?.max_speed
}

/// Puts a motor of the catalogue into a config, for the supply "230v", "400v" or "s120" (`SUPPLIES`): rotor
/// inertia, torque constant and saturation, rated current, thermal time constant, and the drive's torque and speed
/// limits. The peak torque stays within what the machine's pulley allows, where the config rates that.
///
/// The bus voltage becomes the DC link voltage of that supply, because the catalogue's rated speed and
/// torque hold on it: 540 V on a 400 V three-phase line (1.35 times the line voltage, the figure the
/// Siemens line module data in `catalog/siemens/line_modules_s120.csv` is given for; also for "s120", whose motor
/// data hold for a DC link of 510 to 720 V) and 325 V on a 230 V line (its peak, 230 V times the square root of 2).
/// A config written for another bus would otherwise run the motor against a back-EMF it was never meant to meet.
/// The braking threshold and the trip level of the config's DC link are scaled with the bus voltage: left as they
/// were, those of a config written for a lower bus would lie below the new one.
///
/// Winding resistance and inductance: Siemens publishes them only for the 1FK7 and 1FT7 (their data sheets, per
/// phase). For those they are put in as the model's one winding has them, three times the phase values (its
/// current is the RMS phase current and its voltage three times the phase voltage, see `drive`), and the current
/// controller is retuned with them: Kp = L / T for the config's own current loop time constant T (its L over its
/// Kp), Tn = L / R. For every other motor they stay as the config has them, and so do the current loop's gains.
/// The velocity gain stays here too; `resolve` and `with_parts` retune it for the new rotor.
///
/// Options of the motor, read from `[parts]` where the config has them: `motor_brake = true` takes the inertia and
/// weight with the holding brake (refused for a motor without brake data), and `motor_encoder` the encoder's counts per
/// revolution from `encoders_1fk2_1ft2.csv` ("AS22DQC", "AM22DQC"; the 26-bit "AS26DQC", "AM26DQC" for a 1FT2 only).
/// It sets `drive.max_speed_rad_s` to the catalogue's highest speed (the lower of the mechanical limit and what the
/// converter allows, where both are given) and `motor.stall_current_a`, where the config has them.
pub fn apply_motor(cfg: &mut Config, article: &str, supply: &str) -> Result<(), ConfigError> {
    if !SUPPLIES.contains(&supply) {
        return Err(unknown("motor", &format!("{article}, {supply}")));
    }
    let m = motor_data(article, supply)?;
    let kt = m.static_torque / m.stall_current;
    let rated_speed = m.rated_speed;
    // The holding brake's inertia turns with the rotor (`parts.motor_brake`).
    let rotor = if brake_chosen(cfg)? {
        m.brake.map(|(inertia, _)| inertia).ok_or_else(|| unknown("motor", &format!("{article} with brake")))?
    } else {
        m.rotor_kgm2
    };
    set(cfg, "motor.rotor_inertia_kgm2", rotor)?;
    // The encoder ordered with the motor (`parts.motor_encoder`, "" for the config's): its counts per revolution.
    if let Ok(encoder) = cfg.get("parts.motor_encoder").map(|v| v.as_str().unwrap_or("?").to_string())
        && !encoder.is_empty()
    {
        let series = &article[..article.len().min(4)];
        let row = rows(table("siemens/encoders_1fk2_1ft2.csv"))
            .into_iter()
            .find(|r| r["part_number"] == encoder && r["series"].split(' ').any(|s| s == series))
            .ok_or_else(|| unknown("motor", &format!("{article} with encoder {encoder}")))?;
        set(cfg, "drive.encoder_counts_per_rev", number(&row, "resolution_counts").unwrap_or(f64::NAN))?;
    }
    if let Some(speed) = m.max_speed {
        set_if_present(cfg, "drive.max_speed_rad_s", speed)?;
    }
    // effects.iron_loss and the S1 check ("motor-s1") take the stall current: static torque over torque constant.
    set_if_present(cfg, "motor.stall_current_a", m.stall_current)?;
    set(cfg, "motor.torque_constant_nm_a", kt)?;
    set(cfg, "motor.max_current_a", m.max_current)?;
    // The catalogue's own rated current, up to 18 % above rated torque over the static torque constant.
    // One row prints a rated current far above the stall current (see the file's header); no other motor's
    // is more than 5 % above it, so that is where it stops.
    set(cfg, "motor.rated_current_a", m.rated_current.min(1.05 * m.stall_current))?;
    set(cfg, "motor.torque_constant_at_max_current_nm_a", (m.max_torque / m.max_current).min(kt))?;
    if let Some(seconds) = m.thermal_seconds {
        set(cfg, "motor.thermal_time_constant_s", seconds)?;
    }
    if let Some((phase_resistance, phase_inductance)) = m.winding {
        let time_constant = cfg.positive("motor.inductance_h")? / cfg.positive("drive.current_kp_v_a")?;
        let (resistance, inductance) = (3.0 * phase_resistance, 3.0 * phase_inductance);
        set(cfg, "motor.resistance_ohm", resistance)?;
        set(cfg, "motor.inductance_h", inductance)?;
        set(cfg, "drive.current_kp_v_a", inductance / time_constant)?;
        set(cfg, "drive.current_tn_s", inductance / resistance)?;
        // effects.field_weakening: the data sheet's one rotating field inductance for both axes, as for surface magnets.
        set_if_present(cfg, "motor.d_inductance_h", inductance)?;
    }
    if let Some(pairs) = m.pole_pairs {
        set_if_present(cfg, "motor.pole_pairs", pairs)?;
    }
    // A 1FK2 without a data sheet [estimate]: the inductance from Siemens' armature short-circuit braking table. Shorted,
    // the winding brakes with R ke^2 w / (R^2 + (p w L)^2), at most ke^2 / (2 p L) where p w L = R: the table's maximum
    // braking torque gives p L = kt^2 / (2 M_br,max) (the model's ke is its kt). The current controller is retuned as
    // for a data sheet; the resistance stays the config's.
    // shortcut: R stays an estimate, and the table's conditions (rated values, no saturation) hold for the peak alone.
    if m.winding.is_none()
        && cfg.get("motor.pole_pairs").is_ok()
        && let Some(peak) = rows(table("siemens/dynamic_braking_1fk2.csv"))
            .into_iter()
            .find(|r| r["part_number"] == article)
            .and_then(|r| number(&r, "max_braking_torque_nm"))
    {
        let time_constant = cfg.positive("motor.inductance_h")? / cfg.positive("drive.current_kp_v_a")?;
        let inductance = kt * kt / (2.0 * cfg.positive("motor.pole_pairs")? * peak);
        set(cfg, "motor.inductance_h", inductance)?;
        set(cfg, "drive.current_kp_v_a", inductance / time_constant)?;
        set(cfg, "drive.current_tn_s", inductance / cfg.positive("motor.resistance_ohm")?)?;
        set_if_present(cfg, "motor.d_inductance_h", inductance)?;
    }
    let ratio = cfg.f64("gearbox.ratio").unwrap_or(1.0);
    let allowed = cfg.f64("ratings.pulley_max_torque_nm").map_or(f64::INFINITY, |at_pulley| at_pulley / ratio);
    set(cfg, "drive.peak_torque_nm", m.max_torque.min(allowed))?;
    set(cfg, "drive.rated_torque_nm", m.rated_torque)?;
    set(cfg, "drive.rated_speed_rad_s", rated_speed)?;
    let bus = if supply == "230v" { 325.0 } else { 540.0 };
    let scale = bus / cfg.positive("drive.bus_voltage_v")?;
    for rating in ["ratings.braking_threshold_v", "ratings.dc_link_overvoltage_v"] {
        if let Ok(voltage) = cfg.f64(rating) {
            set(cfg, rating, voltage * scale)?;
        }
    }
    set(cfg, "drive.bus_voltage_v", bus)?;
    // Full torque up to the rated speed; it is gone where the back-EMF uses up what the DC link gives the winding.
    set(cfg, "drive.no_load_speed_rad_s", (crate::drive::WINDING_PER_DC_LINK * bus / kt).max(1.1 * rated_speed))
}

/// The drive the catalogue pairs with a motor at a supply: for the S210 the converter the D 32 lists beside the
/// motor (at 230 V the single-phase one where there is one); for a 1FK7 or 1FT7 on S120 the Single Motor Module
/// D 21.4 names. The 1FK2 and 1FT2 on S120 have no pairing in their manuals: theirs is the smallest Single Motor
/// Module whose rated current carries the motor's stall current, the rule D 21.4 pairs the 1FK7 and 1FT7 by
/// (`siemens_s120_motors_are_plausible_and_their_motor_modules_exist` checks it); of two such, the one with the
/// higher maximum current. "" where there is none.
pub fn paired_drive(article: &str, supply: &str) -> &'static str {
    match supply {
        "230v" | "400v" => {
            let Some(motor) = rows(table("siemens/motors_s210.csv")).into_iter().find(|r| r["part_number"] == article)
            else {
                return "";
            };
            let columns: &[&str] = if supply == "400v" {
                &["paired_drive_400v_3ac"]
            } else {
                &["paired_drive_230v_1ac", "paired_drive_230v_3ac"]
            };
            columns.iter().map(|c| motor[c]).find(|a| !a.is_empty()).unwrap_or("")
        }
        "s120" => {
            let modules = rows(table("siemens/motor_modules_s120.csv"));
            if let Some(motor) =
                rows(table("siemens/motors_s120.csv")).into_iter().find(|r| r["part_number"] == article)
            {
                // "6SL3120-7TE13-0AD.": 7 for Single or Double, the last character open.
                let pattern = motor["paired_drive"].replacen("6SL3120-7", "6SL3120-1", 1);
                let stem = &pattern[..pattern.len().saturating_sub(1)];
                if let Some(m) = modules.iter().find(|m| m["part_number"].starts_with(stem)) {
                    return m["part_number"];
                }
            }
            let stall = motor_data(article, "s120").map_or(f64::INFINITY, |m| m.stall_current);
            let mut fits: Vec<&Row> = modules
                .iter()
                .filter(|m| m["axes"] == "1" && number(m, "rated_output_current_a").is_some_and(|i| i >= stall))
                .collect();
            let key = |m: &&Row| {
                (
                    number(m, "rated_output_current_a").unwrap_or(f64::INFINITY),
                    -number(m, "max_output_current_a").unwrap_or(0.0),
                )
            };
            fits.sort_by(|a, b| key(a).partial_cmp(&key(b)).unwrap_or(std::cmp::Ordering::Equal));
            fits.first().map_or("", |m| m["part_number"])
        }
        _ => "",
    }
}

/// Puts a drive into a config: an S210 converter of `converters_s210.csv` or an S120 Motor Module of
/// `motor_modules_s120.csv`, by article number. It sets the drive's rated and maximum output current as
/// `ratings.drive_rated_current_a` and `drive_max_current_a`, where the config has them; `verdicts` judges the
/// cycle's RMS and peak motor current against them (checks `drive-current`, `drive-peak-current`). The currents of
/// a Double Motor Module are per axis. With effects.drive_current_limit the maximum current also limits the simulated
/// current (`DriveParams::torque_range`): a cycle that asks more of the drive lags behind and fails the check.
///
/// The DC link, where the config has these ratings. An S210 converter (`braking_s210.csv`, from its operating
/// instructions) sets `dc_link_overvoltage_v` to its trip level (410 V on 1 AC, 820 V on 3 AC) and the powers of its
/// internal braking resistor, `braking_resistor_continuous_w` and `_peak_w`; not on the `supply` "230v" for a 3 AC
/// converter, where Siemens requires an external resistor and the config's own values stand for it. Siemens publishes
/// no switch-on threshold of the braking chopper, so `braking_threshold_v` stays the config's estimate; only where that
/// is at or above the trip level (a config for a 400 V bus scaled to a 230 V converter's 325 V) it is taken halfway
/// between the highest line peak the converter is rated for (240 or 480 V + 10 %, times the square root of 2) and the
/// trip level. Nor does Siemens publish the S210's DC link capacitance, which stays as the config has it unless
/// `dc_link_measured.csv` has one measured on a real drive of that article. An S120 Motor Module sets
/// `dc_link_capacitance_uf` from `motor_modules_s120.csv`, half of it per axis for a Double Motor Module. A Motor
/// Module has no braking resistor of its own: on the S120 the line-up brakes through a Braking Module, so it sets the
/// braking from the Braking Module row of `braking_s120.csv` (6SL3100-1AE31-0AB1 with its 6SL3100-1BE31-0AA0 resistor:
/// 1.5 kW continuous, 100 kW peak, switching on at 770 V as `braking_threshold_v`) and `dc_link_overvoltage_v` to the
/// Active Line Modules' trip at 820 V (`dc_link_trip_s120.csv`, from the booksize equipment manual), and switches
/// `effects.shared_dc_link` on, where the config has it: the axes of a line-up share its DC link (an S210 converter
/// switches it off, each has its own). Before 13 November 2026 an S120 drive was
/// judged against the S210 converter's internal resistor the config had (100 W, 5.7 kW on the EXCT-30). And it sets
/// `drive.braking_module`, so that `verdicts` judges the axes' braking power together against the one module (an S210
/// clears it); until the eleventh review each axis was judged against the whole module.
///
/// And `ratings.drive_base_load_current_a`, the current the drive falls back to when its thermal image trips
/// (effects.drive_overload): an S120 Motor Module's from `motor_modules_s120.csv`; for an S210 converter, whose base load
/// current Siemens does not publish, `S210_BASE_LOAD_SHARE` of its rated current [estimate].
pub fn apply_drive(cfg: &mut Config, article: &str, supply: &str) -> Result<(), ConfigError> {
    // An S210 brakes into a resistor of its own, the axes of an S120 line-up into its one Braking Module.
    let braking_module = |cfg: &mut Config, on: bool| {
        if cfg.get("drive.braking_module").is_ok() {
            cfg.set_text(&format!("drive.braking_module = {on}"))
        } else {
            Ok(())
        }
    };
    let shared_dc_link = |cfg: &mut Config, on: bool| {
        if cfg.get("effects.shared_dc_link").is_ok() {
            cfg.set_text(&format!("effects.shared_dc_link = {on}"))
        } else {
            Ok(())
        }
    };
    let (rated, max, base) =
        if let Some(c) = rows(table("siemens/converters_s210.csv")).into_iter().find(|r| r["part_number"] == article) {
            braking_module(cfg, false)?;
            // Every S210 converter has a DC link of its own.
            shared_dc_link(cfg, false)?;
            if let Some(b) = rows(table("siemens/braking_s210.csv")).into_iter().find(|r| r["part_number"] == article) {
                let trip = number(&b, "dc_link_overvoltage_v").ok_or_else(|| unknown("drive", article))?;
                let line = if b["line"] == "1ac" { 240.0 } else { 480.0 };
                set_if_present(cfg, "ratings.dc_link_overvoltage_v", trip)?;
                if cfg.f64("ratings.braking_threshold_v").is_ok_and(|threshold| threshold >= trip) {
                    set(cfg, "ratings.braking_threshold_v", (1.1 * line * std::f64::consts::SQRT_2 + trip) / 2.0)?;
                }
                if !(b["line"] == "3ac" && supply == "230v") {
                    let (continuous, peak) = number(&b, "continuous_braking_power_w")
                        .zip(number(&b, "peak_braking_power_w"))
                        .ok_or_else(|| unknown("drive", article))?;
                    set_if_present(cfg, "ratings.braking_resistor_continuous_w", continuous)?;
                    set_if_present(cfg, "ratings.braking_resistor_peak_w", peak)?;
                }
            }
            // Siemens publishes no base load current for the S210 (`S210_BASE_LOAD_SHARE`).
            let rated = number(&c, "rated_output_current_a");
            (rated, number(&c, "max_output_current_a"), rated.map(|r| S210_BASE_LOAD_SHARE * r))
        } else {
            let m = rows(table("siemens/motor_modules_s120.csv"))
                .into_iter()
                .find(|r| r["part_number"] == article)
                .ok_or_else(|| unknown("drive", article))?;
            if let Some(capacitance) = number(&m, "dc_link_capacitance_f").map(|f| in_unit(f, 1e6)) {
                set_if_present(
                    cfg,
                    "ratings.dc_link_capacitance_uf",
                    if m["axes"] == "2" { capacitance / 2.0 } else { capacitance },
                )?;
            }
            let module = rows(table("siemens/braking_s120.csv"))
                .into_iter()
                .find(|r| r["kind"] == "braking_module")
                .ok_or_else(|| unknown("drive", article))?;
            let (continuous, peak, threshold) = (
                number(&module, "continuous_power_w"),
                number(&module, "peak_power_w"),
                number(&module, "activation_threshold_v"),
            );
            let ((continuous, peak), threshold) =
                continuous.zip(peak).zip(threshold).ok_or_else(|| unknown("drive", article))?;
            set_if_present(cfg, "ratings.braking_resistor_continuous_w", continuous)?;
            set_if_present(cfg, "ratings.braking_resistor_peak_w", peak)?;
            set_if_present(cfg, "ratings.braking_threshold_v", threshold)?;
            // The DC link's trip is the Active Line Module's (`dc_link_trip_s120.csv`); all of them trip at the same level.
            let trip = rows(table("siemens/dc_link_trip_s120.csv"))
                .iter()
                .filter_map(|r| number(r, "dc_link_overvoltage_v"))
                .reduce(f64::min)
                .ok_or_else(|| unknown("drive", article))?;
            set_if_present(cfg, "ratings.dc_link_overvoltage_v", trip)?;
            braking_module(cfg, true)?;
            // The axes of an S120 line-up share one DC link: what one brakes, another can draw.
            shared_dc_link(cfg, true)?;
            (
                number(&m, "rated_output_current_a"),
                number(&m, "max_output_current_a"),
                number(&m, "base_load_output_current_a"),
            )
        };
    if let Some(capacitance) = rows(table("siemens/dc_link_measured.csv"))
        .into_iter()
        .find(|r| r["part_number"] == article)
        .and_then(|r| number(&r, "dc_link_capacitance_f"))
        .map(|f| in_unit(f, 1e6))
    {
        set_if_present(cfg, "ratings.dc_link_capacitance_uf", capacitance)?;
    }
    let ((rated, max), base) = rated.zip(max).zip(base).ok_or_else(|| unknown("drive", article))?;
    set_if_present(cfg, "ratings.drive_rated_current_a", rated)?;
    set_if_present(cfg, "ratings.drive_base_load_current_a", base)?;
    set_if_present(cfg, "ratings.drive_max_current_a", max)
}

/// A gearbox by name, its catalogue row: size, motor frame size and ratio, e.g. "NRK070-04-5".
fn gearbox_row(name: &str) -> Option<Row> {
    let parts: Vec<&str> = name.split('-').collect();
    let all: Vec<Row> = gearboxes().into_iter().flat_map(|text| rows(text)).collect();
    all.into_iter()
        .find(|r| parts.len() == 3 && r["size"] == parts[0] && r["motor_frame"] == parts[1] && r["ratio"] == parts[2])
}

/// Mass of a gearbox by name, from the 01/2023 configuration manual (`gearbox_mass.csv`); `None` for a name the
/// catalogue does not have.
pub fn gearbox_mass(name: &str) -> Option<f64> {
    let parts: Vec<&str> = name.split('-').collect();
    let row = rows(table("siemens/gearbox_mass.csv")).into_iter().find(|r| {
        parts.len() == 3
            && r["size"] == parts[0]
            && r["motor_frames"].split(' ').any(|f| f == parts[1])
            && r["ratio"] == parts[2]
    })?;
    number(&row, "mass_kg")
}

/// Puts a gearbox of the catalogue into a config. The name is size, motor frame size and ratio, e.g.
/// "NRK070-04-5". Sets ratio, efficiency, torsional stiffness, backlash and the gearbox's own inertia,
/// and sets the torque ratings where the config has them.
pub fn apply_gearbox(cfg: &mut Config, name: &str) -> Result<(), ConfigError> {
    let gearbox = gearbox_row(name).ok_or_else(|| unknown("gearbox", name))?;
    let get = |column: &str| number(&gearbox, column);
    let need = |column: &str| get(column).ok_or_else(|| unknown("gearbox", name));
    set(cfg, "gearbox.ratio", need("ratio")?)?;
    set(cfg, "gearbox.efficiency", need("efficiency_full_load")?)?;
    set(cfg, "gearbox.stiffness_nm_rad", need("torsional_stiffness_nm_rad")?)?;
    // The manual leaves backlash and inertia open for a few gearboxes; the config's values stay then.
    if let Some(backlash) = get("backlash_rad") {
        set(cfg, "gearbox.backlash_rad", backlash)?;
    }
    // Kept apart from the rotor's: added into it, the next motor put into the config would wipe it out.
    if let Some(inertia) = get("inertia_at_input_kgm2") {
        set(cfg, "gearbox.inertia_at_input_kgm2", inertia)?;
    }
    for (rating, column) in
        [("gearbox_rated_torque_nm", "rated_output_torque_nm"), ("gearbox_max_torque_nm", "max_output_torque_nm")]
    {
        if cfg.f64(&format!("ratings.{rating}")).is_ok() {
            set(cfg, &format!("ratings.{rating}"), need(column)?)?;
        }
    }
    Ok(())
}

/// A belt's values at a width, from the rows of its profile in `belts.csv`: linear between the standard widths,
/// in proportion to the width below the narrowest and above the widest (the cords and the mass go with the width).
/// Width in m; (pitch m, mass kg/m, allowable pull N, EA N)
fn belt_values(profile: &str, width: f64) -> Option<(f64, f64, f64, f64)> {
    let all = rows(table("megadyne/belts.csv"));
    let of: Vec<&Row> = all.iter().filter(|r| r["profile"] == profile).collect();
    let point = |r: &Row| -> Option<(f64, [f64; 3])> {
        let pull = number(r, "max_pull_n")?;
        Some((number(r, "width_m")?, [number(r, "mass_kg_m")?, pull, pull / number(r, "elongation_at_max_pull")?]))
    };
    let points: Vec<(f64, [f64; 3])> = of.iter().map(|r| point(r)).collect::<Option<_>>()?;
    let (first, last) = (points.first()?, points.last()?);
    let at = |k: usize| -> f64 {
        if width <= first.0 {
            return first.1[k] * width / first.0;
        }
        if width >= last.0 {
            return last.1[k] * width / last.0;
        }
        let i = points.iter().position(|p| p.0 >= width).unwrap_or(points.len() - 1);
        let (a, b) = (points[i - 1], points[i]);
        a.1[k] + (b.1[k] - a.1[k]) * (width - a.0) / (b.0 - a.0)
    };
    Some((number(of[0], "pitch_m")?, at(0), at(1), at(2)))
}

/// Puts a belt into a config by profile and width in mm, e.g. "AT5-25": an open-ended Megadyne MEGALINEAR belt
/// with standard steel cords (`belts.csv`), profiles T5, AT3, AT5, AT10, MTD3, MTD5, MTD8 (MTD: HTD-compatible).
/// Sets the belt's EA (allowable pull over the stretch at it), mass per metre and tooth pitch, and, where the
/// config has them, the allowable belt tension (the allowable pull, a quarter of the breaking strength), the
/// width and the tooth load table of `ratings`. A width between the standard ones is interpolated (`belt_values`).
pub fn apply_belt(cfg: &mut Config, name: &str) -> Result<(), ConfigError> {
    let (profile, width) = name.rsplit_once('-').ok_or_else(|| unknown("belt", name))?;
    let width: f64 =
        width.parse().ok().filter(|w: &f64| *w > 0.0 && w.is_finite()).ok_or_else(|| unknown("belt", name))?;
    apply_belt_at(cfg, profile, width / 1000.0).ok_or_else(|| unknown("belt", name))?
}

/// `apply_belt` for a width in m; `None` for a profile the catalogue does not have.
fn apply_belt_at(cfg: &mut Config, profile: &str, width: f64) -> Option<Result<(), ConfigError>> {
    let (pitch, mass, pull, ea) = belt_values(profile, width)?;
    let teeth: Vec<(f64, f64)> = rows(table("megadyne/belt_tooth_load.csv"))
        .iter()
        .filter(|r| r["profile"] == profile)
        .filter_map(|r| {
            number(r, "speed_rad_s")
                .map(|w| in_unit(w, 30.0 / PI))
                .zip(number(r, "specific_tooth_load_n_m").map(|n| in_unit(n, 1e-2)))
        })
        .collect();
    Some(set_belt(cfg, (pitch, mass, pull, ea), in_unit(width, 1e3), &teeth))
}

/// Belt values into a config: (pitch m, mass kg/m, allowable pull N, EA N), width mm, tooth load (r/min, N/cm).
fn set_belt(
    cfg: &mut Config,
    (pitch, mass, pull, ea): (f64, f64, f64, f64),
    width_mm: f64,
    teeth: &[(f64, f64)],
) -> Result<(), ConfigError> {
    set(cfg, "belt.stiffness_ea_n", ea)?;
    set(cfg, "belt.mass_kg_m", mass)?;
    set(cfg, "belt.tooth_pitch_m", pitch)?;
    set_if_present(cfg, "ratings.belt_max_tension_n", pull)?;
    set_if_present(cfg, "ratings.belt_width_mm", width_mm)?;
    let list =
        |values: Vec<f64>| format!("[{}]", values.iter().map(|v| format!("{v:?}")).collect::<Vec<_>>().join(", "));
    if cfg.get("ratings.belt_tooth_load_rpm").is_ok() && cfg.get("ratings.belt_tooth_load_n_cm").is_ok() {
        cfg.set_text(&format!("ratings.belt_tooth_load_rpm = {}", list(teeth.iter().map(|t| t.0).collect())))?;
        cfg.set_text(&format!("ratings.belt_tooth_load_n_cm = {}", list(teeth.iter().map(|t| t.1).collect())))?;
    }
    Ok(())
}

/// The kind of machine a config is for, told by its keys: an X/Z gantry has a Z stroke, a linear axis one stroke.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Kind {
    XzGantry,
    Single,
}

fn kind(cfg: &Config) -> Option<Kind> {
    if cfg.get("geometry.stroke_z_m").is_ok() && cfg.get("geometry.stroke_x_m").is_ok() {
        Some(Kind::XzGantry)
    } else if cfg.get("geometry.stroke_m").is_ok() {
        Some(Kind::Single)
    } else {
        None
    }
}

/// The catalogue's figures of an axis by name, as `apply_axis` and the axis search use them.
#[derive(Debug, Clone)]
struct AxisData {
    kind: Kind,
    /// Stroke limits in m: horizontal (X, or the linear axis's) from and to, and for a gantry the Z strokes
    /// offered (Festo) or the longest (Bahr, as one entry).
    stroke_x: (f64, f64),
    strokes_z: Vec<f64>,
    /// Bahr: any Z stroke up to the longest; Festo: only the strokes listed.
    z_up_to: bool,
    pulley_radius: f64,
    /// Megadyne belt that gives the belt's mass, allowable pull and tooth load: profile and width in m.
    belt: (String, f64),
    /// EA where the maker states the belt's stretch (Festo), else the belt table's.
    ea: Option<f64>,
    teeth: f64,
    /// Highest torque at the pulley, N m.
    pulley_torque: f64,
    /// Rated payload in kg (Festo) or the slide's dynamic vertical load in N (Bahr).
    payload_kg: Option<f64>,
    slide_load_n: Option<f64>,
    max_speed: f64,
    max_acceleration: Option<f64>,
    /// Bahr gantries: moved masses in X and Z. Festo: per Z stroke the torque formula's constant and motor factor
    /// (for the carriage) and its gravity mass (the Z axis).
    moved: Option<(f64, f64)>,
    festo_formula: Vec<(f64, f64, f64, f64)>,
    /// Weight at zero stroke and per metre of X and of Z stroke, kg.
    weight: (f64, f64, f64),
}

/// The axes of the catalogue by name: the Festo EXCT sizes ("EXCT-30") and the Bahr axes and X/Z gantries
/// ("ELZI-40", "ELZ-60S": series and size). The Bahr XY gantries (no machine of that kind) and the DLZPVI (a gear
/// unit built in, whose ratio is not extracted) are left out.
pub fn axis_names() -> Vec<String> {
    let mut out = festo_sizes();
    out.extend(
        rows(table("bahr/belt_axes.csv"))
            .iter()
            .filter(|r| r["kind"] != "xy_gantry" && r["series"] != "DLZPVI")
            .map(|r| format!("{}-{}", r["series"], r["size"])),
    );
    out
}

/// The Festo sizes by name, in the order of their names.
fn festo_sizes() -> Vec<String> {
    let mut out: Vec<String> =
        rows(table("festo/belt_axes.csv")).iter().map(|r| format!("{}-{}", r["series"], r["size"])).collect();
    out.sort();
    out
}

fn axis_data(name: &str) -> Option<AxisData> {
    let festo = rows(table("festo/belt_axes.csv"));
    if let Some(f) = festo.iter().find(|r| format!("{}-{}", r["series"], r["size"]) == name) {
        let get = |column: &str| number(f, column);
        let festo_formula = rows(table("festo/belt_axis_torque_formula.csv"))
            .iter()
            .filter(|t| t["series"] == f["series"] && t["size"] == f["size"])
            .map(|t| {
                Some((
                    number(t, "stroke_z_m")?,
                    number(t, "constant_kg_m")?,
                    number(t, "motor_inertia_factor_1_m")?,
                    number(t, "gravity_mass_kg")?,
                ))
            })
            .collect::<Option<Vec<_>>>()?;
        let diameter = get("pulley_effective_diameter_m")?;
        let pitch = get("belt_pitch_m")?;
        return Some(AxisData {
            kind: Kind::XzGantry,
            stroke_x: (get("stroke_x_min_m")?, get("stroke_x_max_m")?),
            strokes_z: festo_formula.iter().map(|t| t.0).collect(),
            z_up_to: false,
            pulley_radius: diameter / 2.0,
            // The EXCT belt is steel-cord polyurethane; by its stretch of the AT class (see exct-30.toml).
            belt: (format!("AT{}", in_unit(pitch, 1e3)), get("belt_width_m")?),
            ea: Some(get("belt_elongation_force_n")? / get("belt_elongation")?),
            teeth: (PI * diameter / pitch).round(),
            pulley_torque: get("max_pulley_torque_nm")?,
            payload_kg: get("rated_payload_kg"),
            slide_load_n: None,
            max_speed: get("max_speed_m_s")?,
            max_acceleration: get("max_acceleration_m_s2"),
            moved: None,
            festo_formula,
            weight: (get("mass_at_zero_stroke_kg")?, get("mass_per_stroke_kg_m")?, get("mass_per_stroke_z_kg_m")?),
        });
    }
    let all = rows(table("bahr/belt_axes.csv"));
    let r = all.iter().find(|r| {
        format!("{}-{}", r["series"], r["size"]) == name && r["kind"] != "xy_gantry" && r["series"] != "DLZPVI"
    })?;
    let kind = if r["kind"] == "xz_gantry" { Kind::XzGantry } else { Kind::Single };
    let radius = number(r, "feed_per_rev_m")? / (2.0 * PI);
    let m = |column: &str| number(r, column);
    let (stroke_x, strokes_z) = match kind {
        Kind::XzGantry => (m("max_length_x_m")?, vec![m("max_length_z_m")?]),
        Kind::Single => (m("max_length_one_piece_m").unwrap_or(f64::INFINITY), Vec::new()),
    };
    let moved = match kind {
        Kind::XzGantry => Some((number(r, "moved_mass_x_kg")?, number(r, "moved_mass_z_kg")?)),
        Kind::Single => None,
    };
    Some(AxisData {
        kind,
        stroke_x: (0.0, stroke_x),
        strokes_z,
        z_up_to: true,
        pulley_radius: radius,
        belt: (format!("MTD{}", in_unit(number(r, "belt_pitch_m")?, 1e3)), number(r, "belt_width_m")?),
        ea: None,
        teeth: number(r, "pulley_teeth")?,
        pulley_torque: number(r, "feed_force_peak_n")? * radius,
        payload_kg: None,
        slide_load_n: number(r, "fz_dynamic_n"),
        max_speed: number(r, "max_speed_m_s")?,
        max_acceleration: None,
        moved,
        festo_formula: Vec::new(),
        weight: (
            number(r, "mass_at_zero_stroke_kg")?,
            number(r, "mass_per_stroke_kg_m")?,
            number(r, "mass_per_stroke_z_kg_m").unwrap_or(0.0),
        ),
    })
}

/// The Z stroke an axis gives a config: its own where the axis allows it, for Festo the shortest stroke offered that
/// is at least the config's. `None` where the axis is too short.
fn z_stroke(axis: &AxisData, wanted: f64) -> Option<f64> {
    if axis.z_up_to {
        axis.strokes_z.first().is_some_and(|max| wanted <= *max + 1e-9).then_some(wanted)
    } else {
        axis.strokes_z.iter().copied().filter(|s| *s >= wanted - 1e-9).reduce(f64::min)
    }
}

/// Why an axis cannot carry a config's machine by its catalogue numbers alone, "" where it can: the stroke is
/// longer than the axis has (`axis-stroke`), the payload above its rating (`axis-payload`; Festo's rated load, or
/// for Bahr the weight of the Z axis and payload against the slide's dynamic vertical load Fz), or an axis limit
/// of the config above its speed or acceleration (`axis-speed`). An axis for another kind of machine is `axis-kind`.
fn axis_refusal(cfg: &Config, axis: &AxisData) -> &'static str {
    let Some(kind) = kind(cfg) else { return "axis-kind" };
    if kind != axis.kind {
        return "axis-kind";
    }
    let x = cfg
        .f64(if kind == Kind::XzGantry { "geometry.stroke_x_m" } else { "geometry.stroke_m" })
        .unwrap_or(f64::INFINITY);
    let z_short =
        kind == Kind::XzGantry && cfg.f64("geometry.stroke_z_m").map_or(true, |z| z_stroke(axis, z).is_none());
    if x < axis.stroke_x.0 - 1e-9 || x > axis.stroke_x.1 + 1e-9 || z_short {
        return "axis-stroke";
    }
    let payload = cfg.f64("mass.payload_kg").unwrap_or(0.0);
    let on_slide = axis.moved.map_or(0.0, |(_, z)| z) + payload;
    if axis.payload_kg.is_some_and(|rated| payload > rated + 1e-9)
        || axis.slide_load_n.is_some_and(|fz| on_slide * 9.81 > fz)
    {
        return "axis-payload";
    }
    let limits =
        cfg.get("axis").ok().and_then(|a| a.as_table()).into_iter().flatten().filter_map(|(_, t)| t.as_table());
    for limit in limits {
        let get = |key: &str| limit.get(key).and_then(|v| v.as_float());
        if get("max_velocity_m_s").is_some_and(|v| v > axis.max_speed + 1e-9)
            || get("max_acceleration_m_s2").zip(axis.max_acceleration).is_some_and(|(a, max)| a > max + 1e-9)
        {
            return "axis-speed";
        }
    }
    ""
}

/// Installed weight of an axis at a config's strokes, kg.
fn axis_weight(cfg: &Config, axis: &AxisData) -> f64 {
    let (zero, per_x, per_z) = axis.weight;
    match axis.kind {
        Kind::XzGantry => {
            let z = cfg.f64("geometry.stroke_z_m").ok().and_then(|z| z_stroke(axis, z)).unwrap_or(0.0);
            zero + per_x * cfg.f64("geometry.stroke_x_m").unwrap_or(0.0) + per_z * z
        }
        Kind::Single => zero + per_x * cfg.f64("geometry.stroke_m").unwrap_or(0.0),
    }
}

/// Puts an axis of the catalogue into a config (`axis_names`): pulley radius, the belt (as `apply_belt` for the
/// Megadyne belt of its profile and width; EA from the maker's own stretch where it states one, Festo), the teeth in
/// mesh (half the pulley's), the torque the pulley allows (Festo's maximum torque; Bahr's peak feed force times the
/// radius) where the config rates it, and the moved masses. The config's strokes stay, except that an EXCT gets
/// the shortest Z stroke Festo offers that holds the config's. A stroke the axis does not have is refused, and so
/// is an axis for another kind of machine (an X/Z gantry axis on a config without a Z stroke).
///
/// Moved masses: Bahr's gantry sheets state them per axis (Z axis: the Z mass; carriage: the X mass less it). Bahr's
/// single axes state none, and the config's carriage stays. Festo states the mass its torque formula holds against
/// gravity (the Z axis) and the formula's constant, in which carriage, Z axis, pulleys and belt are lumped. With a
/// machine at hand (`entry`) the carriage is set so that the machine's own torque at 45 degrees (ax = az, a = 1 m/s²,
/// no payload, the gearbox taken out) is Festo's, as the hand-written `exct-30.toml` was; without one (a config only
/// read for its ratings) the carriage stays.
pub fn apply_axis(cfg: &mut Config, name: &str, entry: Option<&MachineEntry>) -> Result<(), ConfigError> {
    let axis = axis_data(name).ok_or_else(|| unknown("axis", name))?;
    match axis_refusal(cfg, &axis) {
        "axis-kind" => return Err(ConfigError::Invalid("parts.axis".into())),
        "axis-stroke" => {
            return Err(ConfigError::Invalid(
                if axis.kind == Kind::XzGantry { "geometry.stroke_x_m" } else { "geometry.stroke_m" }.into(),
            ));
        }
        _ => {}
    }
    apply_belt_at(cfg, &axis.belt.0, axis.belt.1).ok_or_else(|| unknown("belt", &axis.belt.0))??;
    if let Some(ea) = axis.ea {
        set(cfg, "belt.stiffness_ea_n", ea)?;
    }
    set(cfg, "geometry.pulley_radius_m", axis.pulley_radius)?;
    set_if_present(cfg, "ratings.belt_teeth_in_mesh", axis.teeth / 2.0)?;
    set_if_present(cfg, "ratings.pulley_max_torque_nm", axis.pulley_torque)?;
    if let Some((x, z)) = axis.moved {
        set(cfg, "mass.carriage_kg", x - z)?;
        set_if_present(cfg, "mass.z_axis_kg", z)?;
    }
    if axis.kind == Kind::XzGantry {
        let z = z_stroke(&axis, cfg.f64("geometry.stroke_z_m")?)
            .ok_or_else(|| ConfigError::Invalid("geometry.stroke_z_m".into()))?;
        set(cfg, "geometry.stroke_z_m", z)?;
        if let Some(&(_, constant, motor_factor, gravity_mass)) =
            axis.festo_formula.iter().find(|f| (f.0 - z).abs() < 1e-9)
        {
            set(cfg, "mass.z_axis_kg", gravity_mass)?;
            if let Some(entry) = entry {
                let carriage =
                    festo_carriage(entry, cfg, constant + motor_factor * cfg.positive("motor.rotor_inertia_kgm2")?)?;
                set(cfg, "mass.carriage_kg", carriage)?;
            }
        }
    }
    Ok(())
}

/// The carriage mass at which the machine of `cfg` needs `target` N m per m/s² of acceleration at 45 degrees (both
/// axes a / sqrt(2)) on its busiest motor, payload off and the gearbox taken out: the torque is linear in the mass.
fn festo_carriage(entry: &MachineEntry, cfg: &Config, target: f64) -> Result<f64, ConfigError> {
    let mut probe = cfg.clone();
    for assignment in ["mass.payload_kg = 0.0", "gearbox.ratio = 1.0", "gearbox.inertia_at_input_kgm2 = 0.0"] {
        probe.set_text(assignment)?;
    }
    let torque = |carriage: f64| -> Result<f64, ConfigError> {
        let mut c = probe.clone();
        set(&mut c, "mass.carriage_kg", carriage)?;
        let machine = (entry.build)(&c)?;
        let pose = sim_core::testkit::mid_pose(machine.as_ref());
        let (axes, motors) = (machine.axes().len(), machine.motors().len());
        let (mut holding, mut moving) = (vec![0.0; motors], vec![0.0; motors]);
        machine.feedforward_torque(&pose, &vec![0.0; axes], &mut holding);
        machine.feedforward_torque(&pose, &vec![std::f64::consts::FRAC_1_SQRT_2; axes], &mut moving);
        Ok(moving.iter().zip(&holding).map(|(m, h)| (m - h).abs()).fold(0.0, f64::max))
    };
    let start = cfg.positive("mass.carriage_kg")?;
    let (at, slope) = (torque(start)?, torque(start + 1.0)? - torque(start)?);
    let carriage = start + (target - at) / slope;
    if carriage > 0.0 && carriage.is_finite() {
        Ok(carriage)
    } else {
        Err(ConfigError::Invalid("mass.carriage_kg".into()))
    }
}

/// The parts a config names in `[parts]`, "" for none: motor and its supply, gearbox, drive, belt and axis.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Parts<'a> {
    pub motor: &'a str,
    pub supply: &'a str,
    pub gearbox: &'a str,
    pub drive: &'a str,
    pub belt: &'a str,
    pub axis: &'a str,
}

/// The keys of `[parts]`, in the order of `Parts`.
pub const PART_KEYS: [&str; 6] = ["motor", "motor_supply", "gearbox", "drive", "belt", "axis"];

impl<'a> Parts<'a> {
    /// What `[parts]` names; "" where it names nothing or has no such key. A name that is not text (`motor = 5`)
    /// is refused: taken as no name, the config would run with other parts than it says.
    pub fn of(cfg: &'a Config) -> Result<Self, ConfigError> {
        let part = |key: &str| -> Result<&'a str, ConfigError> {
            let path = format!("parts.{key}");
            cfg.get(&path).map_or(Ok(""), |v| v.as_str().ok_or(ConfigError::Type(path.clone())))
        };
        let supply = match part("motor_supply")? {
            "" => "400v",
            supply => supply,
        };
        Ok(Parts {
            motor: part("motor")?,
            supply,
            gearbox: part("gearbox")?,
            drive: part("drive")?,
            belt: part("belt")?,
            axis: part("axis")?,
        })
    }

    /// Whether a named part changes what the machine moves, so that the velocity loop is retuned for it.
    fn moves(&self) -> bool {
        !(self.motor.is_empty() && self.gearbox.is_empty() && self.belt.is_empty() && self.axis.is_empty())
    }
}

/// Puts the named parts into a config: the axis first (it sets the pulley rating the motor's peak torque is held
/// within), then the belt (it overrides the axis's), the gearbox (its ratio is in effect for the motor), the motor and
/// the drive. The names in `[parts]` are emptied, so that doing it again changes nothing.
fn apply_parts(cfg: &mut Config, parts: &Parts, entry: Option<&MachineEntry>) -> Result<(), ConfigError> {
    if !parts.axis.is_empty() {
        apply_axis(cfg, parts.axis, entry)?;
    }
    if !parts.belt.is_empty() {
        apply_belt(cfg, parts.belt)?;
    }
    if !parts.gearbox.is_empty() {
        apply_gearbox(cfg, parts.gearbox)?;
    }
    if !parts.motor.is_empty() {
        apply_motor(cfg, parts.motor, parts.supply)?;
    }
    if !parts.drive.is_empty() {
        apply_drive(cfg, parts.drive, parts.supply)?;
    }
    for key in ["motor", "gearbox", "drive", "belt", "axis"] {
        let _ = cfg.set_text(&format!("parts.{key} = \"\""));
    }
    Ok(())
}

/// The config with the catalogue values of the parts it names, and nothing retuned: for the functions
/// that read ratings and limits from a config and have no machine to retune for. An EXCT named as axis keeps the
/// config's carriage here (see `apply_axis`).
pub(crate) fn parts_applied(cfg: &Config) -> Result<Config, ConfigError> {
    let mut out = cfg.clone();
    apply_parts(&mut out, &Parts::of(cfg)?, None)?;
    Ok(out)
}

/// The tables a config may give one motor its own values in, as `[<table>.<motor id>]` (a machine file's motors
/// that differ: a smaller Z motor, another gearbox). See `motor_config`.
pub const PER_MOTOR_TABLES: [&str; 5] = ["parts", "motor", "gearbox", "drive", "ratings"];

/// The config one motor sees: each table of `PER_MOTOR_TABLES` with that motor's `[<table>.<id>]` over it, and the
/// motor, gearbox and drive its `[parts.<id>]` names applied from the catalogue over that. A key there has to be one
/// the shared table has (`motor.<id>.rotor_inertia_kgm2` overrides `motor.rotor_inertia_kgm2`). Without such tables
/// the config as it is. The drives, the machine files' drive trains and the per-motor verdicts read their motor's.
/// shortcut: the drives' cycle times, the noise seed and a shared DC link stay those of the shared `[drive]`; the
/// velocity gain is not retuned for a part named per motor (set `drive.<id>.velocity_kp_nm_s_rad` with it).
pub fn motor_config(cfg: &Config, id: &str) -> Result<Config, ConfigError> {
    let mut out = cfg.clone();
    let mut named = false;
    for table in PER_MOTOR_TABLES {
        let Some(own) = cfg.get(&format!("{table}.{id}")).ok().filter(|_| !id.is_empty()).and_then(|v| v.as_table())
        else {
            continue;
        };
        named |= table == "parts";
        for (key, value) in own {
            out.set(&format!("{table}.{key}"), value.clone())
                .map_err(|_| ConfigError::Invalid(format!("{table}.{id}.{key}")))?;
        }
    }
    if named {
        let parts = Parts::of(&out)?;
        if !parts.belt.is_empty() || !parts.axis.is_empty() {
            return Err(ConfigError::Invalid(format!("parts.{id} (belt and axis are the machine's, not a motor's)")));
        }
        let own = Parts {
            motor: parts.motor,
            supply: parts.supply,
            gearbox: parts.gearbox,
            drive: parts.drive,
            belt: "",
            axis: "",
        };
        let mut applied = out.clone();
        apply_parts(&mut applied, &own, None)?;
        out = applied;
    }
    Ok(out)
}

/// The config with the catalogue values of the parts it names: `[parts]` with `motor = "<article>"`,
/// `motor_supply` (`SUPPLIES`), `gearbox = "<size-frame-ratio>"`, `drive = "<article>"`, `belt = "<profile-width>"`
/// and `axis = "<name>"`, "" for none. A named part overwrites the numbers the config has for it, and the velocity
/// gain is retuned for it (see `with_parts`). `Sim` does this with every config it is given, so every front end gets
/// it; the functions that read a config beside a `Sim` do it too.
pub fn resolve(entry: &MachineEntry, cfg: &Config) -> Result<Config, ConfigError> {
    with_named(entry, cfg, &Parts::of(cfg)?)
}

/// One combination of parts tried on a machine's cycle: a motor with its drive, with or without a gearbox, on the
/// config's axis or one of the catalogue's.
#[derive(Debug, Clone)]
pub struct Candidate {
    /// "" for an axis ruled out before any motor was tried on it.
    pub article: &'static str,
    /// The gearbox as `apply_gearbox` names it; "" where the config's own gearbox stayed.
    pub gearbox: String,
    /// The drive the catalogue pairs with the motor (`paired_drive`); "" where there is none, and the config's stayed.
    pub drive: &'static str,
    /// The axis of the catalogue the motor was tried on; "" for the config's own.
    pub axis: String,
    /// Motor and gearbox of one drive axis, kg.
    pub mass: f64,
    /// The stand-in for the price, which no catalogue file has: the installed weight of everything the combination
    /// names, kg. Motor and gearbox times the number of motors, and the axis at the config's strokes.
    pub cost: f64,
    /// The cycle and its verdicts; `None` where the cycle was not run, with the reason in `refused`.
    pub result: Option<(CycleLoad, Vec<Verdict>)>,
    pub refused: &'static str,
}

impl Candidate {
    pub fn passes(&self) -> bool {
        self.result.as_ref().is_some_and(|(_, verdicts)| verdicts.iter().all(|v| v.ok))
    }

    /// The check closest to (or furthest beyond) its limit, with value over limit (`analysis::utilisation`: for a
    /// life limit over value). The belt's slack does not count. Two checks of the DC link do not count while they
    /// pass, because their ratio is no measure of how close the cycle is to failing: with a braking resistor
    /// behind them the capacitors fill to their limit by design, in every cycle that brakes hard enough, and
    /// the resistor's checks say what that costs; and the voltage is never below the bus, so its ratio to
    /// the trip level starts at 0.6 or so. Nor does the warm start's convergence while it passes: it says how
    /// well the run was measured, not how close a part is to its limit.
    ///
    /// A run whose drives tripped is infinitely far beyond: its figures end at the trip, where the DC link
    /// was just past its limit, and by that ratio it would rank as the nearest to passing.
    pub fn tightest(&self) -> Option<(&'static str, f64)> {
        let (load, verdicts) = self.result.as_ref()?;
        if load.tripped {
            return Some(("dc-link-overvoltage", f64::INFINITY));
        }
        // A resistor of the axis, or one for all (an S120 line-up's Braking Module, judged once without a motor).
        let resistor = |v: &Verdict| {
            verdicts
                .iter()
                .any(|w| (w.motor == v.motor || w.motor.is_none()) && w.check.starts_with("braking-resistor"))
        };
        let by_design =
            |v: &Verdict| v.ok && (v.check == "dc-link-overvoltage" || v.check == "dc-link-energy" && resistor(v));
        // A limit of 0 gives no share while it passes ("collision" at `profile.min_clearance_m` 0); one that fails counts.
        verdicts
            .iter()
            .filter(|v| (v.limit > 0.0 || !v.ok) && v.check != "belt-slack" && !by_design(v))
            .filter(|v| !(v.ok && v.check == "thermal-warm-start"))
            .map(|v| (v.check, crate::analysis::utilisation(v)))
            .max_by(|a, b| a.1.total_cmp(&b.1))
    }
}

/// In proportion to the inertia the motors turn, rotors and load together: the torque the rigid machine
/// needs per unit of acceleration at the middle of its travel, summed over axes and motors, over the gear
/// ratio (a motor behind a higher ratio turns faster for the same axis acceleration).
fn inertia_seen(entry: &MachineEntry, cfg: &Config) -> Result<f64, ConfigError> {
    let machine = (entry.build)(cfg)?;
    let pose = sim_core::testkit::mid_pose(machine.as_ref());
    let (axes, motors) = (machine.axes().len(), machine.motors().len());
    let (mut holding, mut moving) = (vec![0.0; motors], vec![0.0; motors]);
    machine.feedforward_torque(&pose, &vec![0.0; axes], &mut holding);
    let mut sum = 0.0;
    for axis in 0..axes {
        let mut acc = vec![0.0; axes];
        acc[axis] = 1.0;
        machine.feedforward_torque(&pose, &acc, &mut moving);
        sum += moving.iter().zip(&holding).map(|(m, h)| (m - h).abs()).sum::<f64>();
    }
    Ok(sum / cfg.f64("gearbox.ratio").unwrap_or(1.0))
}

/// The config with a candidate's parts in it and the velocity loop retuned for them. Without this a
/// motor with a much lighter rotor fails by oscillating, not by being too small. `gearbox` "" keeps the
/// config's own; the drive, belt and axis the config names are put in as well.
///
/// The reference of the retuning is the config's own numbers, as they are before a part is put in:
/// `drive.velocity_kp_nm_s_rad` is taken to be tuned for the rotor, gearbox and masses the file states.
/// So a config that names a part states the gain for its own numbers, not for the part. Putting in
/// first one part and then another gives the gain of putting in both at once: the ratios multiply.
///
/// The gain is scaled with the geometric mean of two ratios: that of the inertia the motors turn, load
/// included (`inertia_seen`), and that of the rotor inertia alone (with the gearbox's, which turns with
/// it). Below the belt resonance the velocity loop moves rotor and load together, so the first ratio
/// keeps its speed; above the resonance the belt no longer carries the load along and the loop sees the
/// rotor alone, so the second ratio keeps its stability. The loop's crossover lies near the resonance,
/// where neither holds alone. Scaling with the total inertia only left a rotor a quarter as heavy or
/// lighter with several times too much gain above the resonance, and it oscillated; scaling with the
/// rotor only makes the loop needlessly slow. On the EXCT-30 every 1FK2 runs calmly with the mean.
pub fn with_parts(
    entry: &MachineEntry,
    cfg: &Config,
    article: &str,
    supply: &str,
    gearbox: &str,
) -> Result<Config, ConfigError> {
    let mut parts = Parts::of(cfg)?;
    (parts.motor, parts.supply) = (article, supply);
    if !gearbox.is_empty() {
        parts.gearbox = gearbox;
    }
    with_named(entry, cfg, &parts)
}

/// The config with the parts of `parts` in it, retuned as `with_parts` says.
pub fn with_named(entry: &MachineEntry, cfg: &Config, parts: &Parts) -> Result<Config, ConfigError> {
    let mut out = cfg.clone();
    apply_parts(&mut out, parts, Some(entry))?;
    if parts.moves() {
        let rotor = |cfg: &Config| {
            Ok::<f64, ConfigError>(
                cfg.positive("motor.rotor_inertia_kgm2")? + cfg.non_negative("gearbox.inertia_at_input_kgm2")?,
            )
        };
        let ratio = inertia_seen(entry, &out)? / inertia_seen(entry, cfg)? * rotor(&out)? / rotor(cfg)?;
        let gain = cfg.positive("drive.velocity_kp_nm_s_rad")? * ratio.sqrt();
        set(&mut out, "drive.velocity_kp_nm_s_rad", gain)?;
    }
    Ok(out)
}

/// The catalogue gearboxes of a series (a prefix of the size, e.g. "NRK" or "NRK070") that fit a motor's
/// frame and whose ratio is between half and twice `ratio`, named as `apply_gearbox` takes them. The
/// frame is in the article number: 1FK2104 and 1FK2204 have frame "04". The gearboxes are for the 1FK2 and 1FT2
/// only.
pub fn fitting_gearboxes(article: &str, series: &str, ratio: f64) -> Vec<String> {
    if article.get(3..4) != Some("2") {
        return Vec::new();
    }
    let all = gearboxes().into_iter().flat_map(|text| rows(text));
    let fits = |r: &Row| {
        r["size"].starts_with(series)
            && article.get(5..7) == Some(r["motor_frame"])
            && number(r, "ratio").is_some_and(|n| n >= 0.5 * ratio && n <= 2.0 * ratio)
    };
    all.filter(fits).map(|r| format!("{}-{}-{}", r["size"], r["motor_frame"], r["ratio"])).collect()
}

/// One combination to try: the config (with the axis named, where one is tried), motor, gearbox ("" the config's),
/// axis name and the axis's weight.
struct Job {
    cfg: Config,
    article: &'static str,
    gearbox: String,
    axis: String,
    axis_weight: f64,
    /// The id of the one motor the candidate is for (`search_motor`); "" for all of them.
    motor: String,
}

/// The jobs of one config: every motor `wanted` takes at the supply, each with every fitting gearbox of the series
/// where the machine has a gearbox (ratio above 1).
fn motor_jobs(
    cfg: &Config,
    supply: &str,
    wanted: &dyn Fn(&str) -> bool,
    gearboxes: Option<&str>,
    axis: &str,
    axis_weight: f64,
) -> Vec<Job> {
    let ratio = parts_applied(cfg).ok().and_then(|own| own.f64("gearbox.ratio").ok()).unwrap_or(1.0);
    let mut out = Vec::new();
    for article in motor_articles_for(supply).into_iter().filter(|a| wanted(a)) {
        let names = match gearboxes {
            Some(series) if ratio > 1.0 => fitting_gearboxes(article, series, ratio),
            _ => vec![String::new()],
        };
        out.extend(names.into_iter().map(|gearbox| Job {
            cfg: cfg.clone(),
            article,
            gearbox,
            axis: axis.to_string(),
            axis_weight,
            motor: String::new(),
        }));
    }
    out
}

/// Runs the jobs on all processors and ranks them: the passing ones first, cheapest first (`Candidate::cost`, then
/// `mass`), the others after them, nearest to passing first. A motor not offered at the supply is left out.
///
/// Before a cycle is simulated the catalogue numbers are checked against the axis limits: the motor speed at the
/// highest axis speed against the rated speed, and the torque of the rigid machine at the highest acceleration
/// against the motor's maximum torque (`check_limits`). A candidate that fails there is not simulated; it is
/// returned with `result` empty and the limit in `refused`.
fn run_jobs(entry: &MachineEntry, supply: &str, jobs: Vec<Job>, mut out: Vec<Candidate>) -> Vec<Candidate> {
    let next = std::sync::atomic::AtomicUsize::new(0);
    let work = || {
        let mut done = Vec::new();
        while let Some(job) = jobs.get(next.fetch_add(1, std::sync::atomic::Ordering::Relaxed)) {
            done.push(try_job(entry, supply, job));
        }
        done
    };
    let threads = std::thread::available_parallelism().map_or(1, |n| n.get()).min(jobs.len().max(1));
    // One processor, or none known (WebAssembly, which cannot start threads): on this thread.
    let found: Vec<Candidate> = if threads <= 1 {
        work().into_iter().flatten().collect()
    } else {
        std::thread::scope(|scope| {
            let workers: Vec<_> = (0..threads).map(|_| scope.spawn(work)).collect();
            workers.into_iter().flat_map(|w| w.join().expect("a run of the search panicked")).flatten().collect()
        })
    };
    out.extend(found);
    rank(&mut out);
    out
}

/// The order of a search: passing first, by cost, then mass, then motor and gearbox (so that equal weights come
/// out the same every run); the rest by how far beyond their tightest check they are.
pub fn rank(found: &mut [Candidate]) {
    let key = |c: &Candidate| {
        (!c.passes(), if c.passes() { c.cost } else { c.tightest().map_or(f64::INFINITY, |t| t.1) }, c.mass)
    };
    found.sort_by(|a, b| {
        let (ka, kb) = (key(a), key(b));
        ka.0.cmp(&kb.0)
            .then(ka.1.total_cmp(&kb.1))
            .then(ka.2.total_cmp(&kb.2))
            .then(a.axis.cmp(&b.axis))
            .then(a.article.cmp(b.article))
            .then(a.gearbox.cmp(&b.gearbox))
    });
}

fn try_job(entry: &MachineEntry, supply: &str, job: &Job) -> Option<Candidate> {
    let drive = paired_drive(job.article, supply);
    let mut parts = Parts::of(&job.cfg).ok()?;
    (parts.motor, parts.supply) = (job.article, supply);
    if !job.gearbox.is_empty() {
        parts.gearbox = &job.gearbox;
    }
    if !drive.is_empty() {
        parts.drive = drive;
    }
    // Not offered at this supply, or a part the config cannot take.
    let (with_parts, motors) = if job.motor.is_empty() {
        let with_parts = with_named(entry, &job.cfg, &parts).ok()?;
        let motors = (entry.build)(&with_parts).map_or(1, |m| m.motors().len()) as f64;
        (with_parts, motors)
    } else {
        (with_motor_part(entry, &job.cfg, &job.motor, job.article, supply, &job.gearbox, drive).ok()?, 1.0)
    };
    let gearbox_mass = if parts.gearbox.is_empty() { 0.0 } else { gearbox_mass(parts.gearbox).unwrap_or(f64::NAN) };
    let mass = motor_mass_for(&job.cfg, job.article, supply).unwrap_or(f64::INFINITY) + gearbox_mass;
    let mut candidate = Candidate {
        article: job.article,
        gearbox: job.gearbox.clone(),
        drive,
        axis: job.axis.clone(),
        mass,
        cost: motors * mass + job.axis_weight,
        result: None,
        refused: "",
    };
    match Sim::new(entry, &with_parts) {
        Err(_) => candidate.refused = "config",
        // The axis limits ask for more speed or torque than this motor has.
        Ok(sim) if !sim.limit_violations.is_empty() => candidate.refused = sim.limit_violations[0].key,
        Ok(mut sim) => match analyse_cycle_with(&mut sim, false) {
            Err(e) => candidate.refused = e.key,
            Ok(load) => match verdicts(&with_parts, &load) {
                Ok(v) => candidate.result = Some((load, v)),
                Err(_) => candidate.refused = "config",
            },
        },
    }
    Some(candidate)
}

/// Tries every catalogue motor that `wanted` accepts and that is offered at the supply (`SUPPLIES`; "s120" the
/// S120 motors) on the cycle of the config, each with the drive the catalogue pairs with it (`paired_drive`) and
/// retuned as `with_parts` says, on all processors, and returns them ranked (`rank`): the passing ones first,
/// cheapest first, then the others, nearest to passing first. No catalogue file has a price; the weight of motor
/// and gearbox stands in for it (`Candidate::cost`).
///
/// With `gearboxes` (a series, see `fitting_gearboxes`) and a config whose machine has a gearbox (ratio
/// above 1), each motor is tried with every fitting gearbox of that series in place of the config's.
///
/// What fails the catalogue screen is listed and not simulated (`run_jobs`).
pub fn search_motors(
    entry: &MachineEntry,
    cfg: &Config,
    supply: &str,
    wanted: &dyn Fn(&str) -> bool,
    gearboxes: Option<&str>,
) -> Vec<Candidate> {
    let own_axis = Parts::of(cfg).ok().map(|p| p.axis.to_string()).unwrap_or_default();
    let weight = axis_data(&own_axis).map_or(0.0, |axis| axis_weight(cfg, &axis));
    run_jobs(entry, supply, motor_jobs(cfg, supply, wanted, gearboxes, &own_axis, weight), Vec::new())
}

/// The motor search on every axis of the catalogue that is of the config's kind (`axis_names`: the EXCT sizes and
/// Bahr's X/Z gantries for a config with a Z stroke, Bahr's single axes for one with one stroke), keeping the
/// config's cycle and payload. An axis that cannot carry the machine by its catalogue numbers (`axis_refusal`:
/// stroke, payload, speed) is listed once, with `article` "" and the reason in `refused`, and no motor is tried on
/// it. The rest are ranked together, cheapest first: the axis's weight counts into `cost`.
pub fn search_axes(
    entry: &MachineEntry,
    cfg: &Config,
    supply: &str,
    wanted: &dyn Fn(&str) -> bool,
    gearboxes: Option<&str>,
) -> Vec<Candidate> {
    let (mut jobs, mut refused) = (Vec::new(), Vec::new());
    for name in axis_names() {
        let Some(axis) = axis_data(&name) else { continue };
        let reason = axis_refusal(cfg, &axis);
        if reason == "axis-kind" {
            continue;
        }
        if !reason.is_empty() {
            refused.push(Candidate {
                article: "",
                gearbox: String::new(),
                drive: "",
                axis: name,
                mass: f64::NAN,
                cost: f64::NAN,
                result: None,
                refused: reason,
            });
            continue;
        }
        let mut with_axis = cfg.clone();
        if with_axis.set_text(&format!("parts.axis = {name:?}")).is_err() {
            continue; // a config without `[parts]` names no part
        }
        jobs.extend(motor_jobs(&with_axis, supply, wanted, gearboxes, &name, axis_weight(cfg, &axis)));
    }
    run_jobs(entry, supply, jobs, refused)
}

/// `search_motors` for one motor of a machine with several (a machine file's `machine.motor` of id `motor`): each
/// candidate becomes that motor's own parts, `[parts.<motor>]` (`with_motor_part`), and the other motors stay as the
/// config has them. With `gearboxes` the motor's own gearbox ratio picks the fitting ones. The cost is that one motor's
/// and gearbox's weight; the axis is the config's.
pub fn search_motor(
    entry: &MachineEntry,
    cfg: &Config,
    supply: &str,
    wanted: &dyn Fn(&str) -> bool,
    gearboxes: Option<&str>,
    motor: &str,
) -> Result<Vec<Candidate>, ConfigError> {
    if !(entry.build)(cfg)?.motors().iter().any(|m| m.id == motor) {
        return Err(ConfigError::Invalid(format!("machine.motor[{motor}] (no motor of the machine has this id)")));
    }
    let ratio = parts_applied(&motor_config(cfg, motor)?)?.f64("gearbox.ratio").unwrap_or(1.0);
    let mut jobs = Vec::new();
    for article in motor_articles_for(supply).into_iter().filter(|a| wanted(a)) {
        let names = match gearboxes {
            Some(series) if ratio > 1.0 => fitting_gearboxes(article, series, ratio),
            _ => vec![String::new()],
        };
        jobs.extend(names.into_iter().map(|gearbox| Job {
            cfg: cfg.clone(),
            article,
            gearbox,
            axis: String::new(),
            axis_weight: 0.0,
            motor: motor.to_string(),
        }));
    }
    Ok(run_jobs(entry, supply, jobs, Vec::new()))
}

/// The config with the catalogue motor `article` at `supply`, its drive `drive` and the gearbox `gearbox` ("" each: the
/// motor's own) as motor `motor`'s own parts, `[parts.<motor>]` (`motor_config` puts them in), and that motor's velocity
/// gain `drive.<motor>.velocity_kp_nm_s_rad` retuned for them as `with_named` retunes the shared one: by the square root
/// of the ratio of the inertia that motor turns (`inertia_seen`, its column alone) times that of its rotor's.
pub fn with_motor_part(
    entry: &MachineEntry,
    cfg: &Config,
    motor: &str,
    article: &str,
    supply: &str,
    gearbox: &str,
    drive: &str,
) -> Result<Config, ConfigError> {
    let parse = |text: &str| text.parse::<toml::Table>().map_err(|e| ConfigError::Parse(e.to_string()));
    // Sets `<table>.<motor>.<key>`, making the motor's table where the config has none.
    let own = |cfg: &Config, table: &str, pairs: &[(&str, toml::Value)]| -> Result<Config, ConfigError> {
        let mut all = parse(&cfg.to_text()?)?;
        let shared = all.entry(table).or_insert_with(|| toml::Value::Table(toml::Table::new()));
        let shared = shared.as_table_mut().ok_or_else(|| ConfigError::Type(table.into()))?;
        let mine = shared.entry(motor).or_insert_with(|| toml::Value::Table(toml::Table::new()));
        let mine = mine.as_table_mut().ok_or_else(|| ConfigError::Type(format!("{table}.{motor}")))?;
        for (key, value) in pairs {
            mine.insert(key.to_string(), value.clone());
        }
        Config::parse(&toml::to_string(&all).map_err(|e| ConfigError::Parse(e.to_string()))?)
    };
    let text = |t: &str| toml::Value::String(t.into());
    let mut parts = vec![("motor", text(article)), ("motor_supply", text(supply))];
    parts.extend(
        [("drive", drive), ("gearbox", gearbox)].into_iter().filter(|(_, t)| !t.is_empty()).map(|(k, t)| (k, text(t))),
    );
    let out = own(cfg, "parts", &parts)?;
    let index =
        (entry.build)(cfg)?.motors().iter().position(|m| m.id == motor).ok_or_else(|| {
            ConfigError::Invalid(format!("machine.motor[{motor}] (no motor of the machine has this id)"))
        })?;
    let rotor = |cfg: &Config| -> Result<(f64, f64), ConfigError> {
        let mine = motor_config(cfg, motor)?;
        Ok((
            mine.positive("motor.rotor_inertia_kgm2")? + mine.non_negative("gearbox.inertia_at_input_kgm2")?,
            mine.f64("gearbox.ratio").unwrap_or(1.0),
        ))
    };
    // The torque motor `index` needs per unit of each axis's acceleration at the middle of the travel, over its ratio.
    let seen = |cfg: &Config| -> Result<f64, ConfigError> {
        let machine = (entry.build)(cfg)?;
        let pose = sim_core::testkit::mid_pose(machine.as_ref());
        let (axes, motors) = (machine.axes().len(), machine.motors().len());
        let (mut holding, mut moving) = (vec![0.0; motors], vec![0.0; motors]);
        machine.feedforward_torque(&pose, &vec![0.0; axes], &mut holding);
        let mut sum = 0.0;
        for axis in 0..axes {
            let mut acc = vec![0.0; axes];
            acc[axis] = 1.0;
            machine.feedforward_torque(&pose, &acc, &mut moving);
            sum += (moving[index] - holding[index]).abs();
        }
        Ok(sum / rotor(cfg)?.1)
    };
    let ratio = seen(&out)? / seen(cfg)? * rotor(&out)?.0 / rotor(cfg)?.0;
    let gain = motor_config(cfg, motor)?.positive("drive.velocity_kp_nm_s_rad")? * ratio.sqrt();
    own(&out, "drive", &[("velocity_kp_nm_s_rad", toml::Value::Float(gain))])
}
