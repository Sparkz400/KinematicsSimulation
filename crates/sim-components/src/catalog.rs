//! The component catalogue as the simulator uses it: a part named by its article number puts its
//! catalogue values into a config, and a search tries every motor (and every axis) on a machine's cycle.
//!
//! The tables of `catalog/` (schema 1: SI units, see `catalog/README.md`) are compiled in, so a front end needs no
//! path to them; `use_directory` reads them from another copy of the catalogue instead (`--catalog DIR`).

use crate::analysis::{CycleLoad, Verdict, analyse_cycle_with, verdicts};
use crate::sim::Sim;
use sim_core::{Config, ConfigError, Machine, MachineEntry};
use std::collections::HashMap;
use std::f64::consts::PI;
use std::sync::{Mutex, OnceLock, PoisonError};

mod mechanics;
pub use mechanics::{
    GUIDE_CARRIAGES, GUIDE_STATIC_SAFETY, guide_carriages, is_linear_motor, linear_motor_articles_for,
    magnet_track_weight, mech_axes, motor_shaft_diameter,
};

/// The catalogue's schema version this simulator reads. A catalogue of another version is refused.
pub const SCHEMA_VERSION: i64 = 1;

macro_rules! tables {
    ($($name:literal),* $(,)?) => { [$(($name, include_str!(concat!("../../../catalog/", $name)))),*] };
}

/// The tables the simulator reads, by their path in the catalogue, with the compiled-in copy.
const TABLES: &[(&str, &str)] = &tables![
    "siemens/motors_s210.csv",
    "siemens/motors_s210_oi.csv",
    "siemens/dynamic_braking_1fk2.csv",
    "siemens/s1_curves_s210.csv",
    "siemens/encoders_1fk2_1ft2.csv",
    "siemens/dc_link_trip_s120.csv",
    "siemens/derating_s210.csv",
    "siemens/derating_s120.csv",
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
    "siemens/gearbox_sp.csv",
    "siemens/gearbox_mass.csv",
    "megadyne/belts.csv",
    "megadyne/belt_tooth_load.csv",
    "festo/belt_axes.csv",
    "festo/belt_axis_torque_formula.csv",
    "bahr/belt_axes.csv",
    "beckhoff/motors_am8000.csv",
    "beckhoff/drives_ax8000.csv",
    "beckhoff/drives_ax5000.csv",
    "br/motors_8ls.csv",
    "br/drives_acopos_p3.csv",
    "kollmorgen/motors_akm.csv",
    "kollmorgen/motors_akm2g.csv",
    "kollmorgen/drives_akd.csv",
    "lenze/motors_mcs.csv",
    "lenze/motors_m850.csv",
    "lenze/drives_i950.csv",
    "mitsubishi/motors_j5.csv",
    "mitsubishi/drives_j5.csv",
    "rexroth/motors_ms2n.csv",
    "rexroth/drives_ctrlx.csv",
    "sew/motors_cmp.csv",
    "sew/drives_movic.csv",
    "yaskawa/motors_sigma7.csv",
    "yaskawa/drives_sigma7.csv",
    "wittenstein/gearbox_sp.csv",
    "wittenstein/gearbox_tp.csv",
    "neugart/gearbox_ple.csv",
    "neugart/gearbox_pln.csv",
    "neugart/gearbox_plfe.csv",
    "apex/gearbox_peii.csv",
    "apex/gearbox_af.csv",
    "harmonicdrive/gearbox_csf.csv",
    "harmonicdrive/gearbox_csg.csv",
    "harmonicdrive/gearbox_shg.csv",
    "siemens/motor_shafts.csv",
    "beckhoff/motor_shafts.csv",
    "sew/motor_shafts.csv",
    "yaskawa/motor_shafts.csv",
    "hiwin/ball_screws.csv",
    "hiwin/linear_guides.csv",
    "rexroth/linear_guides.csv",
    "thk/linear_guides.csv",
    "siemens/linear_motors_1fn3.csv",
    "beckhoff/linear_motors_al8000.csv",
    "festo/belt_axes_egc.csv",
    "festo/belt_axes_elga.csv",
    "rexroth/belt_axes_ckr.csv",
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
    // Checked all before anything is kept, so a refused directory leaks nothing.
    let mut texts = Vec::new();
    for &(name, compiled) in TABLES {
        let text = read(name)?;
        check_table(name, &text, compiled)?;
        texts.push((name, text));
    }
    let mut loaded = HashMap::new();
    for (name, text) in texts {
        // Kept for the life of the process, as the compiled-in tables are.
        loaded.insert(name, &*Box::leak(text.into_boxed_str()));
    }
    LOADED.set(loaded).map_err(|_| ConfigError::Part("error-catalog-read", "the catalogue was already chosen".into()))
}

/// Refuses a CSV table (not `catalog.toml`) whose header lacks a column of the compiled-in table of the same name, or
/// that has a row with another number of fields than its header. The readers index rows by column name and panic on
/// a missing one, so this is the one place that guards them. The required columns are the compiled-in header's, so
/// they cannot drift from the tables the code was written against; a column the code never reads is required too.
fn check_table(name: &str, text: &str, compiled: &str) -> Result<(), ConfigError> {
    if !name.ends_with(".csv") {
        return Ok(());
    }
    let fail = |what: String| ConfigError::Part("error-catalog-read", format!("{name}: {what}"));
    // As `rows` reads them: notes and blank lines passed over.
    let mut lines = text.lines().enumerate().filter(|(_, l)| !l.starts_with('#') && !l.trim().is_empty());
    let header: Vec<&str> = lines.next().map_or(Vec::new(), |(_, l)| l.split(',').collect());
    let wanted = compiled.lines().find(|l| !l.starts_with('#')).unwrap_or("");
    if let Some(missing) = wanted.split(',').find(|c| !header.contains(c)) {
        return Err(fail(format!("missing column \"{missing}\"")));
    }
    match lines.find(|(_, l)| l.split(',').count() != header.len()) {
        Some((i, l)) => {
            Err(fail(format!("line {}: {} fields, the header has {}", i + 1, l.split(',').count(), header.len())))
        }
        None => Ok(()),
    }
}

/// The supplies a motor can be named for: the S210 data at 230 V or 400 V, or the motor on a SINAMICS S120
/// (400 V line). "s120" takes the 1FK7 and 1FT7 of `motors_s120.csv` and the 1FK2 and 1FT2 of their S120 manuals.
/// "s210new-230v" and "s210new-400v" are the same 1FK2 and 1FT2 data with the "SINAMICS S210 (New)" converters
/// (6SL5310) the D 32 pairs them with. The others are another maker's motors and drives at a line voltage,
/// "<vendor>-<volts>v" (`vendor_of`): the rows of `<vendor>/motors_*.csv` and `<vendor>/drives_*.csv` with that
/// `vendor` and `line_voltage_v`. `every_supply_has_motors_and_every_vendor_table_a_supply` keeps this list and the
/// tables together.
pub const SUPPLIES: &[&str] = &[
    "230v",
    "400v",
    "s120",
    "s210new-230v",
    "s210new-400v",
    "beckhoff-400v",
    "beckhoff-480v",
    "br-400v",
    "kollmorgen-240v",
    "kollmorgen-400v",
    "lenze-400v",
    "mitsubishi-200v",
    "mitsubishi-400v",
    "rexroth-400v",
    "sew-400v",
    "yaskawa-200v",
    "yaskawa-400v",
];

/// The maker of a supply's motors and drives (`SUPPLIES`): "siemens" for the Siemens ones, else the part before "-".
pub fn vendor_of(supply: &str) -> &str {
    vendor_supply(supply).map_or("siemens", |(vendor, _)| vendor)
}

/// Another maker's supply, "<vendor>-<volts>v": the vendor and the line voltage in V.
fn vendor_supply(supply: &str) -> Option<(&str, f64)> {
    let (vendor, line) = supply.split_once('-')?;
    if vendor == "s210new" {
        return None;
    }
    Some((vendor, line.strip_suffix('v')?.parse().ok()?))
}

/// Whether every drive of the maker at that line voltage says in its `supply` column that it is fed single-phase ("1 AC",
/// "1 x ..."; "1 or 3 AC" is both and counts as three-phase, as a drive on three phases has the lower ripple and the
/// higher bus of the two).
fn single_phase_line(vendor: &str, line: f64) -> bool {
    let drives = vendor_rows("drives_", vendor, line);
    !drives.is_empty() && drives.iter().all(|d| single_phase(d.get("supply").copied().unwrap_or("")))
}

/// Whether a drive's `supply` column says single-phase: "1 AC ...", "1 x ...", "1x ...", "single ...".
fn single_phase(supply: &str) -> bool {
    let text = supply.to_lowercase();
    ["1 ac", "1 x", "1x", "single"].iter().any(|p| text.starts_with(p))
}

/// A supply of the S210 data: the voltage of its rated columns ("230v", "400v") and whether the converters are the
/// new 6SL5310 ones.
fn s210_supply(supply: &str) -> Option<(&'static str, bool)> {
    match supply {
        "230v" => Some(("230v", false)),
        "400v" => Some(("400v", false)),
        "s210new-230v" => Some(("230v", true)),
        "s210new-400v" => Some(("400v", true)),
        _ => None,
    }
}

/// The text of every other maker's tables of a kind ("motors_" or "drives_"): `<vendor>/<kind>*.csv`.
fn vendor_tables(kind: &str) -> Vec<&'static str> {
    TABLES
        .iter()
        .map(|(name, _)| *name)
        .filter(|name| name.split_once('/').is_some_and(|(maker, file)| maker != "siemens" && file.starts_with(kind)))
        .map(table)
        .collect()
}

/// The rows of another maker's tables of a kind (`vendor_tables`) for a vendor and line voltage.
fn vendor_rows(kind: &str, vendor: &str, line: f64) -> Vec<Row> {
    vendor_tables(kind)
        .into_iter()
        .flat_map(rows)
        .filter(|r| r.get("vendor") == Some(&vendor) && number(r, "line_voltage_v") == Some(line))
        .collect()
}

type Row = &'static HashMap<&'static str, &'static str>;

/// The rows of a catalogue file; lines starting with '#' are notes. Each table is parsed once and kept for the life of
/// the process, as its text is: a motor search reads the same tables for every motor it tries.
fn rows(text: &'static str) -> Vec<Row> {
    type Parsed = Vec<((usize, usize), &'static [HashMap<&'static str, &'static str>])>;
    static PARSED: Mutex<Parsed> = Mutex::new(Vec::new());
    let key = (text.as_ptr() as usize, text.len());
    let mut parsed = PARSED.lock().unwrap_or_else(PoisonError::into_inner);
    let table = match parsed.iter().find(|(k, _)| *k == key) {
        Some(&(_, table)) => table,
        None => {
            // A blank line (an editor's at the end) is no row: one would lack every column but the first.
            let mut lines = text.lines().filter(|l| !l.starts_with('#') && !l.trim().is_empty());
            let header: Vec<&str> = lines.next().unwrap_or("").split(',').collect();
            let table: &'static [_] =
                Vec::leak(lines.map(|line| header.iter().copied().zip(line.split(',')).collect()).collect());
            parsed.push((key, table));
            table
        }
    };
    table.iter().collect()
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

/// Puts a part's value into a config. A `[bounds]` entry of that value was written for the config's own value, so it
/// follows the part's: scaled by the same factor, new over old (the band keeps its width relative to the value). A
/// bound of a value that was 0 stays as written.
fn set(cfg: &mut Config, path: &str, value: f64) -> Result<(), ConfigError> {
    let bound = format!("bounds.{path}");
    if let (Ok(old), Ok([low, high])) = (cfg.f64(path), cfg.f64s(&bound).as_deref())
        && old != 0.0
        && old != value
    {
        let (low, high) = (low * value / old, high * value / old);
        cfg.set_text(&format!("{bound} = [{:?}, {:?}]", low.min(high), low.max(high)))?;
    }
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
        _ => match vendor_supply(supply) {
            Some((vendor, line)) => vendor_rows("motors_", vendor, line)
                .iter()
                .filter(|r| number(r, "rotor_inertia_kgm2").is_some())
                .map(|r| r["part_number"])
                .collect(),
            None => motor_articles(),
        },
    }
}

/// What a motor brings into a config, from whichever catalogue file has it.
struct MotorData {
    rotor_kgm2: f64,
    /// Rotor inertia and weight with the holding brake, where the catalogue gives them.
    brake: Option<(f64, f64)>,
    static_torque: f64,
    stall_current: f64,
    max_torque: f64,
    max_current: f64,
    rated_current: f64,
    rated_torque: f64,
    rated_speed: f64,
    /// The highest speed: mechanical (S210 table), what the converter allows (1FK2, 1FT2 on S120), or the lower of the
    /// two (1FK7, 1FT7).
    max_speed: f64,
    thermal_seconds: Option<f64>,
    /// Phase resistance at 20 °C in ohms and rotating field inductance in henries, where a data sheet prints them.
    winding: Option<(f64, f64)>,
    /// Pole pairs, where the catalogue prints them (the 1FK7 and 1FT7: half the data sheet's poles, else `motors_s120.csv`).
    pole_pairs: Option<f64>,
    mass: f64,
    /// A linear motor's force constant as printed (N/A), which stands in for the torque constant static over stall.
    force_constant: Option<f64>,
    /// A linear motor's magnetic attraction (N) and pole pitch (m).
    linear: Option<(f64, f64)>,
    /// The back-EMF constant in the model's one-winding convention (sqrt(3) times the printed line-to-line RMS voltage
    /// constant, V s/rad), where the table has it and it is not a documented misprint; `None` for kt.
    emf_constant: Option<f64>,
}

/// The one-winding back-EMF constant from a printed line-to-line RMS voltage constant. The two documented misprints
/// (AKM64Q at 240 V, 1FT7044-1AF7; `VENDOR_MISPRINTS` in the checks) are not used; any other printed value is, however far
/// from the torque constant (Beckhoff AM8000 up to 1.30 times it, Rexroth MS2N 1.18). `line` is the line voltage, 0 for
/// Siemens.
fn emf_constant(printed: Option<f64>, article: &str, line: f64) -> Option<f64> {
    let misprint = (article == "AKM64Q" && line == 240.0) || article == "1FT7044-1AF7";
    printed.filter(|_| !misprint).map(|ke| 3f64.sqrt() * ke)
}

fn motor_data(article: &str, supply: &str) -> Result<MotorData, ConfigError> {
    let missing = || unknown("motor", &format!("{article}, {supply}"));
    // A linear motor (`mechanics::linear_rows`): forces for torques, the coil's mass for the rotor's inertia, m/s for rad/s.
    if let Some(m) = mechanics::linear_motor_row(article, supply) {
        let get = |column: &str| number(&m, column).ok_or_else(missing);
        let max_speed = get("max_speed_m_s")?;
        return Ok(MotorData {
            rotor_kgm2: get("coil_mass_kg")?,
            brake: None,
            static_torque: get("continuous_force_n")?,
            stall_current: get("continuous_current_a")?,
            max_torque: get("peak_force_n")?,
            max_current: get("peak_current_a")?,
            rated_current: get("continuous_current_a")?,
            rated_torque: get("continuous_force_n")?,
            // Full force up to the speed the maker gives the peak force at, else up to its highest speed.
            rated_speed: number(&m, "max_speed_at_peak_force_m_s").unwrap_or(max_speed),
            max_speed,
            thermal_seconds: number(&m, "thermal_time_constant_s"),
            winding: number(&m, "phase_resistance_20c_ohm").zip(number(&m, "phase_inductance_h")),
            pole_pairs: None,
            mass: get("coil_mass_kg")?,
            force_constant: number(&m, "force_constant_n_a"),
            linear: number(&m, "attraction_force_n").zip(number(&m, "pole_pitch_m")),
            emf_constant: None,
        });
    }
    if let Some((vendor, line)) = vendor_supply(supply) {
        let m = vendor_rows("motors_", vendor, line)
            .into_iter()
            .find(|r| r["part_number"] == article)
            .ok_or_else(|| unknown("motor", article))?;
        let get = |column: &str| number(&m, column).ok_or_else(missing);
        // shortcut: a maker that prints no standstill values (Yaskawa, Mitsubishi) has its rated ones stand for them,
        // and one that prints no rated torque (SEW CMP) its standstill one, which overstates what the motor carries at
        // speed by a few per cent; upgrade with rated data. A missing rated current (Kollmorgen, Beckhoff at 480 V)
        // is the rated torque over the torque constant.
        let either = |first: &str, second: &str| number(&m, first).map_or_else(|| get(second), Ok);
        let (static_torque, stall_current) =
            (either("static_torque_nm", "rated_torque_nm")?, either("stall_current_a", "rated_current_a")?);
        let rated_current = match (number(&m, "rated_current_a"), number(&m, "rated_torque_nm")) {
            (Some(current), _) => current,
            (None, Some(torque)) => torque * stall_current / static_torque,
            (None, None) => stall_current,
        };
        return Ok(MotorData {
            rotor_kgm2: get("rotor_inertia_kgm2")?,
            brake: number(&m, "rotor_inertia_with_brake_kgm2").zip(number(&m, "mass_with_brake_kg")),
            static_torque,
            stall_current,
            max_torque: get("max_torque_nm")?,
            max_current: get("max_current_a")?,
            rated_current,
            rated_torque: either("rated_torque_nm", "static_torque_nm")?,
            rated_speed: get("rated_speed_rad_s")?,
            max_speed: get("max_speed_rad_s")?,
            thermal_seconds: number(&m, "thermal_time_constant_s"),
            // Terminal to terminal in the tables; one phase of the star is half of it.
            winding: number(&m, "line_resistance_20c_ohm")
                .zip(number(&m, "line_inductance_h"))
                .map(|(r, l)| (r / 2.0, l / 2.0)),
            pole_pairs: number(&m, "pole_pairs"),
            mass: get("mass_kg")?,
            force_constant: None,
            linear: None,
            emf_constant: emf_constant(number(&m, "voltage_constant_v_s_rad"), article, line),
        });
    }
    if supply == "s120" {
        if let Some(m) = rows(table("siemens/motors_s120.csv")).into_iter().find(|r| r["part_number"] == article) {
            let sheet = rows(table("siemens/motors_s120_electrical.csv"))
                .into_iter()
                .find(|r| r["part_number"] == article)
                .ok_or_else(missing)?;
            let get = |row: &Row, column: &str| number(row, column).ok_or_else(missing);
            return Ok(MotorData {
                rotor_kgm2: get(&m, "rotor_inertia_kgm2")?,
                // motors_s120.csv has no brake data.
                brake: None,
                static_torque: get(&m, "static_torque_nm")?,
                stall_current: get(&m, "stall_current_a")?,
                max_torque: get(&sheet, "max_torque_nm")?,
                max_current: get(&sheet, "max_current_a")?,
                rated_current: get(&m, "rated_current_a")?,
                rated_torque: get(&m, "rated_torque_nm")?,
                // Rated torque and current hold at `rated_values_at_speed_rad_s` where the table fills it (9 rows).
                rated_speed: number(&m, "rated_values_at_speed_rad_s")
                    .map_or_else(|| get(&m, "rated_speed_rad_s"), Ok)?,
                max_speed: get(&sheet, "max_speed_mechanical_rad_s")?.min(get(&sheet, "max_speed_inverter_rad_s")?),
                thermal_seconds: number(&sheet, "thermal_time_constant_s"),
                winding: number(&sheet, "phase_resistance_20c_ohm").zip(number(&sheet, "phase_inductance_h")),
                // The motor's own data sheet before the catalogue's table, which prints 5 for every 1FT7 High Dynamic
                // where their sheets give 6 and 8 poles (the 150 other motors agree in both).
                pole_pairs: number(&sheet, "poles").map(|p| p / 2.0).or_else(|| number(&m, "pole_pairs")),
                mass: get(&m, "mass_kg")?,
                force_constant: None,
                linear: None,
                emf_constant: emf_constant(number(&sheet, "voltage_constant_v_s_rad"), article, 0.0),
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
            static_torque: get("static_torque_nm")?,
            stall_current: get("stall_current_a")?,
            max_torque: get("max_torque_nm")?,
            max_current: get("max_current_a")?,
            rated_current: get("rated_current_a")?,
            rated_torque: get("rated_torque_nm")?,
            // Three rows print a rated speed that does not fit their rated torque and power (`docs/catalogue.md`): power
            // over torque gives 4500 and 3000 r/min, as the S210 catalogue has it. The data keep the printed value.
            rated_speed: if ["1FT2105-4AH", "1FT2206-4AH", "1FT2208-4AF"].contains(&article) {
                get("rated_power_w")? / get("rated_torque_nm")?
            } else {
                get("rated_speed_rad_s")?
            },
            max_speed: get("max_speed_inverter_rad_s")?,
            thermal_seconds: number(&m, "thermal_time_constant_s"),
            winding: None,
            pole_pairs: None,
            mass: get("mass_kg")?,
            force_constant: None,
            linear: None,
            emf_constant: None,
        });
    }
    let motor = rows(table("siemens/motors_s210.csv"))
        .into_iter()
        .find(|r| r["part_number"] == article)
        .ok_or_else(|| unknown("motor", article))?;
    let get = |column: &str| number(&motor, column).ok_or_else(missing);
    let voltage = s210_supply(supply).ok_or_else(missing)?.0;
    let rated = |quantity: &str, unit: &str| get(&format!("rated_{quantity}_{voltage}_{unit}"));
    let manual = motor_manuals().into_iter().flat_map(|text| rows(text)).find(|r| r["part_number"] == article);
    // The S210 operating instructions (09/2022) have the 230 V windings and the 1FT2 the manuals lack; where both have a
    // motor they agree.
    let instructions = || rows(table("siemens/motors_s210_oi.csv")).into_iter().find(|r| r["part_number"] == article);
    Ok(MotorData {
        rotor_kgm2: get("rotor_inertia_kgm2")?,
        brake: number(&motor, "rotor_inertia_with_brake_kgm2").zip(number(&motor, "mass_with_brake_kg")),
        static_torque: get("static_torque_nm")?,
        stall_current: get("stall_current_a")?,
        max_torque: get("max_torque_nm")?,
        max_current: get("max_current_a")?,
        rated_current: rated("current", "a")?,
        rated_torque: rated("torque", "nm")?,
        rated_speed: rated("speed", "rad_s")?,
        max_speed: get("max_speed_rad_s")?,
        thermal_seconds: manual
            .as_ref()
            .and_then(|r| number(r, "thermal_time_constant_s"))
            .or_else(|| instructions().and_then(|r| number(&r, "thermal_time_constant_s"))),
        winding: None,
        pole_pairs: None,
        mass: get("mass_kg")?,
        force_constant: None,
        linear: None,
        emf_constant: None,
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
    motor_data(article, supply).ok().map(|m| m.max_speed)
}

/// Puts a motor of the catalogue into a config, for a supply of `SUPPLIES` ("230v", "400v", "s120", ...): rotor
/// inertia, torque constant and saturation, rated current, thermal time constant, and the drive's torque and speed
/// limits. The peak torque stays within what the machine's pulley allows, where the config rates that.
///
/// The bus voltage becomes the DC link voltage of that supply, because the catalogue's rated speed and
/// torque hold on it: 540 V on a 400 V three-phase line (1.35 times the line voltage, the figure the
/// Siemens line module data in `catalog/siemens/line_modules_s120.csv` is given for; also for "s120", whose motor
/// data hold for a DC link of 510 to 720 V) and 325 V on a 230 V line (its peak, 230 V times the square root of 2);
/// another maker's, 1.35 times its line voltage.
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
///
/// A linear motor (`<maker>/linear_motors_*.csv`: the Siemens 1FN3 at "s120", another maker's at its supply) reads in N
/// for N m and m for rad, as a linear motor config does: its printed force constant for the torque constant, the
/// coil's mass for the rotor's inertia, the continuous and peak force and current, the speeds in m/s, the winding's
/// phase values as the 1FK7's are put in, and its magnetic attraction and pole pitch into `[linear_motor]`.
pub fn apply_motor(cfg: &mut Config, article: &str, supply: &str) -> Result<(), ConfigError> {
    if !SUPPLIES.contains(&supply) {
        return Err(unknown("motor", &format!("{article}, {supply}")));
    }
    let m = motor_data(article, supply)?;
    let kt = m.force_constant.unwrap_or(m.static_torque / m.stall_current);
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
    if let Some((attraction, pitch)) = m.linear {
        set_if_present(cfg, "linear_motor.attraction_force_n", attraction)?;
        set_if_present(cfg, "linear_motor.pole_pitch_m", pitch)?;
    }
    set_s1_curve(cfg, &s1_curve(article, supply))?;
    cap_peak_torque(cfg, m.max_torque)?;
    set(cfg, "drive.rated_torque_nm", m.rated_torque)?;
    set(cfg, "drive.rated_speed_rad_s", rated_speed)?;
    set(cfg, "drive.max_speed_rad_s", m.max_speed)?;
    // A rectified three-phase line gives 1.35 times its voltage, a single-phase one its peak, sqrt(2) times it.
    let bus = match vendor_supply(supply) {
        Some((vendor, line)) => (if single_phase_line(vendor, line) { std::f64::consts::SQRT_2 } else { 1.35 }) * line,
        None if s210_supply(supply).is_some_and(|(voltage, _)| voltage == "230v") => 325.0,
        None => 540.0,
    };
    let scale = bus / cfg.positive("drive.bus_voltage_v")?;
    for rating in ["ratings.braking_threshold_v", "ratings.dc_link_overvoltage_v"] {
        if let Ok(voltage) = cfg.f64(rating) {
            set(cfg, rating, voltage * scale)?;
        }
    }
    set(cfg, "drive.bus_voltage_v", bus)?;
    // Full torque up to the rated speed; it is gone where the back-EMF uses up what the DC link gives the winding.
    // A printed voltage constant is the model's back-EMF constant (`drive.back_emf_constant_v_s_rad`, 0 for kt) and sets
    // the no-load speed with it.
    let ke = m.emf_constant.unwrap_or(kt);
    set_if_present(cfg, "drive.back_emf_constant_v_s_rad", m.emf_constant.unwrap_or(0.0))?;
    set(cfg, "drive.no_load_speed_rad_s", (crate::drive::WINDING_PER_DC_LINK * bus / ke).max(1.1 * rated_speed))
}

/// The S1 curve Siemens prints as a graph for an S210 motor at a supply (`s1_curves_s210.csv`, digitised: article, speed,
/// torque, rising speed): the file holds one curve per motor, the 400 V 3 AC one for a motor with a rated point at 400 V
/// and else the 240 V 3 AC one (its meta file), so it is the motor's only at that voltage, "400v" or "230v" (the 6SL5310
/// converters, "s210new-...", give the same DC link and so the same curve, as they give the same motor values; not the
/// S120). Empty where the file has no curve for it.
fn s1_curve(article: &str, supply: &str) -> Vec<(f64, f64)> {
    let Some(row) = rows(table("siemens/motors_s210.csv")).into_iter().find(|r| r["part_number"] == article) else {
        return Vec::new();
    };
    let read_for = if number(&row, "rated_speed_400v_rad_s").is_some() { "400v" } else { "230v" };
    if s210_supply(supply).map(|(voltage, _)| voltage) != Some(read_for) {
        return Vec::new();
    }
    rows(table("siemens/s1_curves_s210.csv"))
        .into_iter()
        .filter(|r| r.get("article") == Some(&article))
        .filter_map(|r| Some((number(&r, "speed_rad_s")?, number(&r, "torque_nm")?)))
        .collect()
}

/// Puts a named motor's S1 curve into `motor.s1_curve_speed_rad_s` and `motor.s1_curve_torque_nm`, where it replaces
/// the straight line between the printed standstill and rated points (`analysis::s1_torque`); without one (fewer than
/// two points) it takes the keys out, as a curve the config had belongs to the motor it had.
fn set_s1_curve(cfg: &mut Config, curve: &[(f64, f64)]) -> Result<(), ConfigError> {
    const KEYS: [&str; 2] = ["s1_curve_speed_rad_s", "s1_curve_torque_nm"];
    let has = KEYS.iter().any(|k| cfg.get(&format!("motor.{k}")).is_ok());
    if curve.len() < 2 && !has {
        return Ok(());
    }
    let parse = |e: toml::de::Error| ConfigError::Parse(e.to_string());
    let mut all = cfg.to_text()?.parse::<toml::Table>().map_err(parse)?;
    let motor = all.get_mut("motor").and_then(toml::Value::as_table_mut).ok_or(ConfigError::Missing("motor".into()))?;
    for (key, pick) in KEYS.into_iter().zip([|p: &(f64, f64)| p.0, |p: &(f64, f64)| p.1]) {
        if curve.len() < 2 {
            motor.remove(key);
        } else {
            motor.insert(key.into(), toml::Value::Array(curve.iter().map(|p| toml::Value::Float(pick(p))).collect()));
        }
    }
    *cfg = Config::parse(&toml::to_string(&all).map_err(|e| ConfigError::Parse(e.to_string()))?)?;
    Ok(())
}

/// The motor's peak torque `uncapped` held within the pulley's rating. Braking, the pulley's torque is the motor's times
/// ratio over efficiency: the tighter of the two directions caps it. `apply_gearbox` calls it too, so the result does not
/// depend on which of the two came first.
fn cap_peak_torque(cfg: &mut Config, uncapped: f64) -> Result<(), ConfigError> {
    set(cfg, "drive.peak_torque_nm", uncapped.min(pulley_allowed(cfg)))
}

/// The highest motor torque the pulley's rating allows through the config's gearbox; infinite without a rating.
fn pulley_allowed(cfg: &Config) -> f64 {
    let ratio = cfg.f64("gearbox.ratio").unwrap_or(1.0);
    let efficiency = cfg.f64("gearbox.efficiency").unwrap_or(1.0).clamp(f64::MIN_POSITIVE, 1.0);
    cfg.f64("ratings.pulley_max_torque_nm").map_or(f64::INFINITY, |at_pulley| at_pulley * efficiency / ratio)
}

/// The drive the catalogue pairs with a motor at a supply: for the S210 the converter the D 32 lists beside the
/// motor (at 230 V the single-phase one where there is one); for a 1FK7 or 1FT7 on S120 the Single Motor Module
/// D 21.4 names. The 1FK2 and 1FT2 on S120 have no pairing in their manuals: theirs is the smallest Single Motor
/// Module whose rated current carries the motor's stall current, the rule D 21.4 pairs the 1FK7 and 1FT7 by
/// (`siemens_s120_motors_are_plausible_and_their_motor_modules_exist` checks it); of two such, the one with the
/// higher maximum current. For "s210new-..." the 6SL5310 converter the D 32 lists beside the motor.
///
/// Another maker's motor (`vendor_of`): the drive its row names in `paired_drive`, the maker's own pairing, where
/// the maker's drive table has it at that line voltage; else by the same rule as the 1FK2 on S120, the smallest drive of
/// the maker at that line voltage whose rated current carries the motor's stall current, a Single Motor Module before a
/// Double Motor Module. "" where there is none.
pub fn paired_drive(article: &str, supply: &str) -> &'static str {
    if let Some((vendor, line)) = vendor_supply(supply) {
        let drives = vendor_rows("drives_", vendor, line);
        let named = vendor_rows("motors_", vendor, line).into_iter().find(|r| r["part_number"] == article);
        if let Some(own) = named.and_then(|m| drives.iter().find(|d| d["part_number"] == m["paired_drive"])) {
            return own["part_number"];
        }
        let stall = motor_data(article, supply).map_or(f64::INFINITY, |m| m.stall_current);
        let key = |d: &Row| {
            (
                d["axes"] != "1",
                number(d, "rated_output_current_a").unwrap_or(f64::INFINITY),
                -number(d, "max_output_current_a").unwrap_or(0.0),
            )
        };
        return drives
            .iter()
            .filter(|d| number(d, "rated_output_current_a").is_some_and(|i| i >= stall))
            .min_by(|a, b| key(a).partial_cmp(&key(b)).unwrap_or(std::cmp::Ordering::Equal))
            .map_or("", |d| d["part_number"]);
    }
    match s210_supply(supply) {
        Some((voltage, new)) => {
            let Some(motor) = rows(table("siemens/motors_s210.csv")).into_iter().find(|r| r["part_number"] == article)
            else {
                return "";
            };
            let columns: &[&str] = if voltage == "400v" { &["400v_3ac"] } else { &["230v_1ac", "230v_3ac"] };
            let series = if new { "paired_drive_new_" } else { "paired_drive_" };
            columns.iter().map(|c| motor[format!("{series}{c}").as_str()]).find(|a| !a.is_empty()).unwrap_or("")
        }
        None => paired_s120(article, supply),
    }
}

fn paired_s120(article: &str, supply: &str) -> &'static str {
    match supply {
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

/// Which drives the search tries with each motor: the one the catalogue pairs with it (`paired_drive`), that one and
/// the next size down and up (`drive_neighbours`), or every drive of the supply (`drives_for`).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum Drives {
    #[default]
    Paired,
    Neighbours,
    All,
}

impl std::str::FromStr for Drives {
    type Err = ();
    fn from_str(text: &str) -> Result<Self, ()> {
        match text {
            "paired" => Ok(Drives::Paired),
            "neighbours" | "neighbors" => Ok(Drives::Neighbours),
            "all" => Ok(Drives::All),
            _ => Err(()),
        }
    }
}

/// The drives of a supply (`SUPPLIES`) with their rated and maximum output current, smallest rated current first (of
/// two alike, the higher maximum current first): the S210 converters for "230v" (1 AC and 3 AC) and "400v" (3 AC), the
/// new 6SL5310 ones for "s210new-...", the Single and Double Motor Modules for "s120" (a Double, "6SL3120-2...", with its per axis currents), another maker's drives at the line
/// voltage. A maker that prints no maximum current (Mitsubishi) has NaN there.
pub fn drives_for(supply: &str) -> Vec<(&'static str, f64, f64)> {
    let current = |r: &Row| {
        (r["part_number"], number(r, "rated_output_current_a"), number(r, "max_output_current_a").unwrap_or(f64::NAN))
    };
    let rows: Vec<Row> = match (vendor_supply(supply), s210_supply(supply), supply) {
        (Some((vendor, line)), ..) => vendor_rows("drives_", vendor, line),
        (_, Some((voltage, new)), _) => rows(table("siemens/converters_s210.csv"))
            .into_iter()
            .filter(|r| (r["series"] == "S210 (New)") == new && (voltage == "230v" || r["supply"].ends_with("3 AC")))
            .collect(),
        (.., "s120") => rows(table("siemens/motor_modules_s120.csv")),
        _ => Vec::new(),
    };
    let mut out: Vec<(&'static str, f64, f64)> =
        rows.iter().map(current).filter_map(|(part, rated, max)| Some((part, rated?, max))).collect();
    out.sort_by(|a, b| a.1.total_cmp(&b.1).then(b.2.total_cmp(&a.2)).then(a.0.cmp(b.0)));
    out.dedup_by(|a, b| a.0 == b.0);
    out
}

/// The paired drive (`paired_drive`) and the next size down and up by rated output current among the supply's drives
/// (`drives_for`): the largest with a lower rated current and the smallest with a higher one (of two alike, the higher
/// maximum current). Smallest first; the paired one alone where it is not in the list, none where there is none.
pub fn drive_neighbours(article: &str, supply: &str) -> Vec<&'static str> {
    let paired = paired_drive(article, supply);
    let all = drives_for(supply);
    let Some(&(_, rated, _)) = all.iter().find(|d| d.0 == paired) else {
        return [paired].into_iter().filter(|p| !p.is_empty()).collect();
    };
    let lower = all.iter().filter(|d| d.1 < rated).map(|d| d.1).reduce(f64::max);
    let higher = all.iter().find(|d| d.1 > rated).map(|d| d.1);
    let first_at = |current: Option<f64>| current.and_then(|c| all.iter().find(|d| d.1 == c)).map(|d| d.0);
    [first_at(lower), Some(paired), first_at(higher)].into_iter().flatten().collect()
}

/// The drives the search tries with a motor at a supply (`Drives`); "" alone where the catalogue pairs none, and the
/// config's drive stays.
fn drives_tried(article: &str, supply: &str, drives: Drives) -> Vec<&'static str> {
    let out = match drives {
        Drives::Paired => vec![paired_drive(article, supply)],
        Drives::Neighbours => drive_neighbours(article, supply),
        Drives::All => drives_for(supply).into_iter().map(|d| d.0).collect(),
    };
    if out.iter().all(|d| d.is_empty()) { vec![""] } else { out }
}

/// What a motor and drive of the catalogue leave to the config's estimates, by name: "motor-winding" (no phase
/// resistance and inductance: Siemens prints them for the 1FK7 and 1FT7 only), "motor-thermal-time" (no thermal time
/// constant), "motor-pole-pairs" (none printed), "drive-max-current" (the maker prints no peak current, the motor's
/// stands for it), "drive-base-load" (no base load current: `S210_BASE_LOAD_SHARE` of the rated one), and for another
/// maker's motor "motor-standstill" or "motor-rated" where it prints no standstill or no rated values (the other ones
/// stand for them, see `motor_data`). Empty: everything from the catalogue.
pub fn estimated_data(article: &str, supply: &str, drive: &str) -> Vec<&'static str> {
    let mut out = Vec::new();
    if let Ok(m) = motor_data(article, supply) {
        if m.winding.is_none() {
            out.push("motor-winding");
        }
        if m.thermal_seconds.is_none() {
            out.push("motor-thermal-time");
        }
        if m.pole_pairs.is_none() {
            out.push("motor-pole-pairs");
        }
    }
    if let Some((vendor, line)) = vendor_supply(supply)
        && let Some(m) = vendor_rows("motors_", vendor, line).into_iter().find(|r| r["part_number"] == article)
    {
        if number(&m, "static_torque_nm").is_none() || number(&m, "stall_current_a").is_none() {
            out.push("motor-standstill");
        }
        if number(&m, "rated_torque_nm").is_none() || number(&m, "rated_current_a").is_none() {
            out.push("motor-rated");
        }
    }
    if !drive.is_empty() {
        let s120 = rows(table("siemens/motor_modules_s120.csv")).into_iter().any(|r| r["part_number"] == drive);
        if !s120 {
            out.push("drive-base-load");
        }
        if drives_for(supply).iter().any(|d| d.0 == drive && d.2.is_nan()) {
            out.push("drive-max-current");
        }
    }
    out
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
///
/// The rated and the base load current are those of the drive's rated conditions, derated by `drive_derating` for the
/// installation conditions the config gives (`ratings.drive_ambient_c`, `drive_altitude_m`, `drive_pulse_frequency_hz`;
/// absent: the rated ones, and every figure as the catalogue has it). The maximum current is not derated. A drive
/// of another maker, whose rows state no derating, is refused when the config gives a condition.
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
    let vendor_drive = || {
        let all: Vec<Row> = vendor_tables("drives_").into_iter().flat_map(rows).collect();
        let line = vendor_supply(supply).map(|(_, line)| line);
        // The row at the supply's line voltage, else the drive's first.
        let of = all.iter().filter(|r| r["part_number"] == article);
        of.clone().find(|r| number(r, "line_voltage_v") == line).or(of.clone().next()).cloned()
    };
    let (rated, max, base) = if let Some(d) = vendor_drive() {
        braking_module(cfg, false)?;
        apply_vendor_dc_link(cfg, &d)?;
        // shortcut: the makers publish no base load current; the S210's share of the rated current stands in [estimate].
        let rated = number(&d, "rated_output_current_a");
        // Mitsubishi prints no peak current of its amplifiers; it pairs them for the motor's peak, which the motor put
        // into the config before the drive brings [estimate].
        let max = number(&d, "max_output_current_a").or_else(|| cfg.f64("motor.max_current_a").ok());
        (rated, max, rated.map(|r| S210_BASE_LOAD_SHARE * r))
    } else if let Some(c) = rows(table("siemens/converters_s210.csv")).into_iter().find(|r| r["part_number"] == article)
    {
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
    // The maximum current stays as printed: Siemens does not say whether its derating reaches it (`drive_derating`).
    let factor = drive_derating(cfg, article)?;
    let (rated, base) = (rated * factor, base * factor);
    set_if_present(cfg, "ratings.drive_rated_current_a", rated)?;
    set_if_present(cfg, "ratings.drive_base_load_current_a", base)?;
    set_if_present(cfg, "ratings.drive_max_current_a", max)
}

/// The keys of a drive's installation conditions in `[ratings]`: the ambient temperature of the drive itself (the
/// cabinet's air inlet, not the motor's), the installation altitude and the pulse frequency. A config that lacks one has
/// the drive at its rated condition (40 degC, 1000 m, the rated pulse frequency).
pub const DRIVE_CONDITION_KEYS: [&str; 3] =
    ["ratings.drive_ambient_c", "ratings.drive_altitude_m", "ratings.drive_pulse_frequency_hz"];

/// Linear interpolation through `points` (x, y), the value of the first or last point beyond them; 1 for no points.
fn interpolate(mut points: Vec<(f64, f64)>, x: f64) -> f64 {
    points.sort_by(|a, b| a.0.total_cmp(&b.0));
    let (Some(first), Some(last)) = (points.first().copied(), points.last().copied()) else { return 1.0 };
    if x <= first.0 {
        return first.1;
    }
    for pair in points.windows(2) {
        if x <= pair[1].0 {
            return pair[0].1 + (pair[1].1 - pair[0].1) * (x - pair[0].0) / (pair[1].0 - pair[0].0);
        }
    }
    last.1
}

/// The (x, y) points of one `quantity` of a derating table, x from the column `x`, y the output current factor.
fn derating_points(table_rows: &[Row], quantity: &str, x: &str) -> Vec<(f64, f64)> {
    table_rows
        .iter()
        .filter(|r| r["quantity"] == quantity)
        .filter_map(|r| number(r, x).zip(number(r, "output_current_factor")))
        .collect()
}

fn first_x(points: &[(f64, f64)]) -> f64 {
    points.iter().map(|p| p.0).fold(f64::INFINITY, f64::min)
}

fn last_x(points: &[(f64, f64)]) -> f64 {
    points.iter().map(|p| p.0).fold(f64::NEG_INFINITY, f64::max)
}

/// The share of its rated output current a catalogue drive gives at the installation conditions of the config
/// (`DRIVE_CONDITION_KEYS`); 1 for a config that gives none, and for an article that is neither an S210 converter nor an
/// S120 Motor Module. A drive of another maker with a condition given is refused (`error-drive-derating-none`): its rows
/// state no derating, and silently ignoring the condition would judge it at rated conditions.
///
/// S210 (`derating_s210.csv`, operating instructions Table 12-9): the table of altitude against ambient temperature,
/// interpolated linearly between its points, the pulse frequency fixed at 8 kHz. S120 booksize Motor Module
/// (`derating_s120.csv`): the factors of altitude (10 % per 1000 m above 1000 m), ambient temperature (2.67 % per K above
/// 40 degC) and pulse frequency (a curve by module group), multiplied. Siemens does not say how they combine; the product is
/// this simulator's assumption on the safe side. Of Siemens' two ways to trade altitude (reduce the current by 10 % per
/// 1000 m, or the ambient by 5 K per 1000 m) the current reduction is implemented, not both. Refused, naming the key
/// (`error-drive-derating-range`): above 4000 m, above the highest ambient (S210 50 degC, S120 55 degC), below 0 degC, an
/// S210 pulse frequency other than 8 kHz, an S120 one above 16 kHz or not above 0, and an ambient above 45 degC at 16 kHz.
pub fn drive_derating(cfg: &Config, article: &str) -> Result<f64, ConfigError> {
    let given = |key: &str| -> Result<Option<f64>, ConfigError> {
        if cfg.get(key).is_ok() { cfg.f64(key).map(Some) } else { Ok(None) }
    };
    let [ambient_key, altitude_key, pulse_key] = DRIVE_CONDITION_KEYS;
    let (ambient, altitude, pulse) = (given(ambient_key)?, given(altitude_key)?, given(pulse_key)?);
    let refused = |key: &str| ConfigError::Part("error-drive-derating-range", key.into());
    if ambient.is_some_and(|a| a < 0.0) {
        return Err(refused(ambient_key));
    }
    if rows(table("siemens/converters_s210.csv")).iter().any(|r| r["part_number"] == article) {
        let all = rows(table("siemens/derating_s210.csv"));
        let grid: Vec<(f64, f64, f64)> = all
            .iter()
            .filter(|r| r["quantity"] == "altitude_ambient")
            .filter_map(|r| {
                Some((number(r, "ambient_c")?, number(r, "altitude_m")?, number(r, "output_current_factor")?))
            })
            .collect();
        let mut ambients: Vec<f64> = grid.iter().map(|g| g.0).collect();
        ambients.sort_by(f64::total_cmp);
        ambients.dedup();
        // The factor by altitude at each ambient temperature of the table.
        let columns: Vec<(f64, Vec<(f64, f64)>)> =
            ambients.iter().map(|&t| (t, grid.iter().filter(|g| g.0 == t).map(|g| (g.1, g.2)).collect())).collect();
        let altitudes: Vec<(f64, f64)> = grid.iter().map(|g| (g.1, g.2)).collect();
        let fixed = derating_points(&all, "pulse_frequency", "pulse_frequency_hz").first().map_or(f64::NAN, |p| p.0);
        if altitude.is_some_and(|h| h > last_x(&altitudes)) {
            return Err(refused(altitude_key));
        }
        if ambient.is_some_and(|a| ambients.last().is_some_and(|top| a > *top)) {
            return Err(refused(ambient_key));
        }
        if pulse.is_some_and(|p| (p - fixed).abs() > 0.5) {
            return Err(refused(pulse_key));
        }
        let h = altitude.unwrap_or_else(|| first_x(&altitudes));
        let across: Vec<(f64, f64)> = columns.into_iter().map(|(t, points)| (t, interpolate(points, h))).collect();
        return Ok(interpolate(across.clone(), ambient.unwrap_or_else(|| first_x(&across))));
    }
    if let Some(module) = rows(table("siemens/motor_modules_s120.csv")).iter().find(|r| r["part_number"] == article) {
        let all = rows(table("siemens/derating_s120.csv"));
        // A module with curve rows of its own (the 200 A one) uses those alone, every other the rows without a part.
        let own: Vec<Row> = all.iter().filter(|r| r["part_number"] == module["part_number"]).copied().collect();
        let group: Vec<Row> =
            if own.is_empty() { all.iter().filter(|r| r["part_number"].is_empty()).copied().collect() } else { own };
        let altitudes = derating_points(&all, "altitude", "altitude_m");
        let ambients = derating_points(&all, "ambient", "ambient_c");
        let pulses = derating_points(&group, "pulse_frequency", "pulse_frequency_hz");
        if altitude.is_some_and(|h| h > last_x(&altitudes)) {
            return Err(refused(altitude_key));
        }
        if ambient.is_some_and(|a| a > last_x(&ambients)) {
            return Err(refused(ambient_key));
        }
        if pulse.is_some_and(|p| p <= 0.0 || p > last_x(&pulses)) {
            return Err(refused(pulse_key));
        }
        // At the highest pulse frequency the ambient is limited (the limit row's pulse frequency and ambient).
        for limit in all.iter().filter(|r| r["quantity"] == "ambient_limit") {
            if let (Some(p), Some(a), Some(at), Some(most)) =
                (pulse, ambient, number(limit, "pulse_frequency_hz"), number(limit, "ambient_c"))
                && p >= at
                && a > most
            {
                return Err(refused(ambient_key));
            }
        }
        let (h, a, p) = (
            altitude.unwrap_or_else(|| first_x(&altitudes)),
            ambient.unwrap_or_else(|| first_x(&ambients)),
            pulse.unwrap_or_else(|| first_x(&pulses)),
        );
        return Ok(interpolate(altitudes, h) * interpolate(ambients, a) * interpolate(pulses, p));
    }
    match DRIVE_CONDITION_KEYS.iter().find(|key| cfg.get(key).is_ok()) {
        Some(key) => Err(ConfigError::Part("error-drive-derating-none", (*key).into())),
        None => Ok(1.0),
    }
}

/// `estimated_data`'s "drive-derating": the drive's rated and base load current are derated by installation conditions
/// that are not the rated ones. Siemens states the factors per condition, not how they combine (they are multiplied),
/// and not whether the maximum current is derated too (it is not).
pub fn estimated_derating(cfg: &Config, drive: &str) -> Vec<&'static str> {
    if drive_derating(cfg, drive).is_ok_and(|factor| factor != 1.0) { vec!["drive-derating"] } else { Vec::new() }
}

/// The DC link of another maker's drive (`<vendor>/drives_*.csv`), each value only where the row gives it and the config
/// has the rating: capacitance, the internal braking resistor's continuous and peak power, the chopper's switch-on
/// voltage and the trip level. A row with the resistor's ohms and the switch-on voltage but no peak power gets the
/// power the resistor takes at that voltage, U² / R. Where the trip level is given and the config's switch-on voltage
/// lies at or above it, the switch-on voltage is taken halfway between the line's peak + 10 % and the trip, as for the
/// S210.
fn apply_vendor_dc_link(cfg: &mut Config, d: &Row) -> Result<(), ConfigError> {
    if let Some(capacitance) = number(d, "dc_link_capacitance_f").map(|f| in_unit(f, 1e6)) {
        set_if_present(cfg, "ratings.dc_link_capacitance_uf", capacitance)?;
    }
    let threshold = number(d, "braking_threshold_v");
    let peak = number(d, "braking_resistor_peak_w")
        .or_else(|| threshold.zip(number(d, "braking_resistor_internal_ohm")).map(|(u, r)| u * u / r));
    for (rating, value) in [
        ("braking_resistor_continuous_w", number(d, "braking_resistor_continuous_w")),
        ("braking_resistor_peak_w", peak),
        ("dc_link_overvoltage_v", number(d, "dc_link_overvoltage_v")),
        ("braking_threshold_v", threshold),
    ] {
        if let Some(value) = value {
            set_if_present(cfg, &format!("ratings.{rating}"), value)?;
        }
    }
    if let (None, Some(trip), Some(line)) = (threshold, number(d, "dc_link_overvoltage_v"), number(d, "line_voltage_v"))
        && cfg.f64("ratings.braking_threshold_v").is_ok_and(|own| own >= trip)
    {
        set(cfg, "ratings.braking_threshold_v", (1.1 * line * std::f64::consts::SQRT_2 + trip) / 2.0)?;
    }
    Ok(())
}

/// Mass of a gearbox by name (`apply_gearbox`): a Siemens one's from the 01/2023 configuration manual
/// (`gearbox_mass.csv`), another maker's from its own row; `None` for a name the catalogue does not have or a gearbox
/// whose mass it has not.
pub fn gearbox_mass(name: &str) -> Option<f64> {
    let base = name.strip_suffix("-r").unwrap_or(name);
    let parts: Vec<&str> = base.split('-').collect();
    let row = rows(table("siemens/gearbox_mass.csv")).into_iter().find(|r| {
        parts.len() == 3
            && r["size"] == parts[0]
            && r["motor_frames"].split(' ').any(|f| f == parts[1])
            && r["ratio"] == parts[2]
    });
    row.and_then(|r| number(&r, "mass_kg")).or_else(|| mechanics::own_mass(base))
}

/// Efficiency of a Harmonic Drive strain wave gear row without an `efficiency` value [estimate]: the maker prints it as
/// charts only (rated torque, 2000 r/min input, grease); a row whose table has the value read from them uses that, 0.70
/// stands for the rest. It is below what such charts show for the CSF/CSG/SHG ratios (about 0.65 at ratio 30 to 0.85 at
/// 160, from memory, not verified) so a motor sized with it errs on the safe side.
const STRAIN_WAVE_EFFICIENCY: f64 = 0.70;

/// The efficiency a gearbox row prints: at full load (`efficiency_full_load`), or a strain wave gear's read from the
/// maker's charts (`efficiency`).
fn printed_efficiency(row: &Row) -> Option<f64> {
    number(row, "efficiency_full_load").or_else(|| number(row, "efficiency"))
}

fn strain_wave(row: &Row) -> bool {
    matches!(row["series"], "CSF" | "CSG" | "SHG")
}

/// What a gearbox of the catalogue leaves to estimates, by name (see `estimated_data`): "gearbox-efficiency" (a strain wave
/// gear: `STRAIN_WAVE_EFFICIENCY`) and "gearbox-mass" (no mass printed; the search ranks by the motor's alone).
pub fn estimated_gearbox_data(name: &str) -> Vec<&'static str> {
    let mut out = Vec::new();
    if let Some((g, _)) = mechanics::find_gearbox(name)
        && printed_efficiency(&g.row).is_none()
        && strain_wave(&g.row)
    {
        out.push("gearbox-efficiency");
    }
    if gearbox_mass(name).is_none() {
        out.push("gearbox-mass");
    }
    out
}

/// Puts a gearbox of the catalogue into a config. The name is size, motor frame (for the gearboxes fitted through an
/// adapter plate the clamping hub's bore in mm) and ratio, e.g. "NRK070-04-5" or "SP+100MF-19-20", with "-r" after it
/// for the maker's reduced backlash ("SP100-05-20-r"). Sets ratio, efficiency, torsional stiffness, backlash and the
/// gearbox's own inertia, and sets the torque ratings where the config has them. Where the row has them and the config
/// the keys: the torque the efficiency holds at (the rated torque: makers give the efficiency at full load), the
/// no-load running torque and its speed, and the stiffness curve (a strain wave gear's K1 to K3 with its knees T1 and
/// T2 and its hysteresis; a gearbox without one is linear, both knees at its rated torque and K2 = K3 = its stiffness,
/// as `docs/config-reference.md` says for a planetary gearbox).
pub fn apply_gearbox(cfg: &mut Config, name: &str) -> Result<(), ConfigError> {
    let (gearbox, reduced) = mechanics::find_gearbox(name).ok_or_else(|| unknown("gearbox", name))?;
    // A peak the old gearbox's cap held may rise to the motor's own; one the config set lower stays its own.
    let held_by_cap = cfg.f64("drive.peak_torque_nm").is_ok_and(|peak| peak >= pulley_allowed(cfg) * (1.0 - 1e-12));
    let get = |column: &str| number(&gearbox.row, column);
    let need = |column: &str| get(column).ok_or_else(|| unknown("gearbox", name));
    let curve = get("stiffness_k1_nm_rad").zip(get("stiffness_k2_nm_rad")).zip(get("stiffness_k3_nm_rad"));
    let curve = curve.zip(get("stiffness_t1_nm")).zip(get("stiffness_t2_nm"));
    set(cfg, "gearbox.ratio", need("ratio")?)?;
    if let Some(efficiency) = printed_efficiency(&gearbox.row) {
        set(cfg, "gearbox.efficiency", efficiency)?;
    } else if strain_wave(&gearbox.row) {
        set(cfg, "gearbox.efficiency", STRAIN_WAVE_EFFICIENCY)?;
    }
    let stiffness = match curve {
        Some(((((k1, _), _), _), _)) => k1,
        None => need("torsional_stiffness_nm_rad")?,
    };
    set(cfg, "gearbox.stiffness_nm_rad", stiffness)?;
    // The manual leaves backlash and inertia open for a few gearboxes; the config's values stay then.
    if reduced {
        set(cfg, "gearbox.backlash_rad", need("backlash_reduced_rad")?)?;
    } else if let Some(backlash) = get("backlash_rad") {
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
    let rated = need("rated_output_torque_nm")?;
    set_if_present(cfg, "gearbox.efficiency_torque_nm", rated)?;
    if let (Some(torque), Some(speed)) = (get("no_load_torque_nm"), get("no_load_torque_speed_rad_s")) {
        set_if_present(cfg, "gearbox.no_load_torque_nm", torque)?;
        set_if_present(cfg, "gearbox.no_load_speed_rpm", in_unit(speed, 30.0 / PI))?;
    }
    let (t1, t2, k2, k3) = match curve {
        Some(((((_, k2), k3), t1), t2)) => (t1, t2, k2, k3),
        None => (rated, rated, stiffness, stiffness),
    };
    for (key, value) in
        [("stiffness_t1_nm", t1), ("stiffness_t2_nm", t2), ("stiffness_k2_nm_rad", k2), ("stiffness_k3_nm_rad", k3)]
    {
        set_if_present(cfg, &format!("gearbox.{key}"), value)?;
    }
    if let Some(hysteresis) = get("hysteresis_rad") {
        set_if_present(cfg, "gearbox.hysteresis_rad", hysteresis)?;
    }
    // The motor's own peak is what its max current gives at the torque constant at max current (`apply_motor` sets both);
    // a peak the old cap held may rise to it; any other peak is the config's own and is only lowered, never raised.
    if cfg.f64("ratings.pulley_max_torque_nm").is_ok()
        && let (Ok(peak), Ok(current), Ok(constant)) = (
            cfg.f64("drive.peak_torque_nm"),
            cfg.f64("motor.max_current_a"),
            cfg.f64("motor.torque_constant_at_max_current_nm_a"),
        )
    {
        cap_peak_torque(cfg, if held_by_cap { peak.max(current * constant) } else { peak })?;
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
    /// Megadyne belt that gives the belt's mass, allowable pull and tooth load: profile and width in m. `None` for an
    /// axis whose maker names no profile Megadyne has (Festo EGC and ELGA: neoprene with glass cords); the config's belt
    /// stays then.
    belt: Option<(String, f64)>,
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

/// The axes of the catalogue by name: the Festo EXCT sizes ("EXCT-30"), the Bahr axes and X/Z gantries
/// ("ELZI-40", "ELZ-60S": series and size) and the other makers' single belt axes (`<maker>/belt_axes_*.csv`: Festo
/// "EGC-TB-KF-70", "ELGA-TB-G-80", Rexroth "CKR-110"). The Bahr XY gantries (no machine of that kind) and the DLZPVI (a
/// gear unit built in, whose ratio is not extracted) are left out.
pub fn axis_names() -> Vec<String> {
    let mut out = festo_sizes();
    out.extend(belt_axis_rows().iter().map(|(_, r)| format!("{}-{}", r["series"], r["size"])));
    out
}

/// The standard belts of the catalogue as `apply_belt` takes them: profile and width in mm, "AT5-25", one per row of
/// `megadyne/belts.csv` (the Gates table is not read by name). Other widths are taken too, interpolated.
pub fn belt_names() -> Vec<String> {
    rows(table("megadyne/belts.csv"))
        .iter()
        .filter_map(|r| Some(format!("{}-{}", r["profile"], (number(r, "width_m")? * 1e9).round() / 1e6)))
        .collect()
}

/// The rows of the belt axes other than the EXCT (`festo/belt_axes.csv`, with its torque formula): Bahr's, then the
/// other makers' `belt_axes_*.csv`, with the maker; without Bahr's XY gantries and DLZPVI (`axis_names`).
fn belt_axis_rows() -> Vec<(&'static str, Row)> {
    let mut names: Vec<&str> = TABLES
        .iter()
        .map(|(name, _)| *name)
        .filter(|name| name.split_once('/').is_some_and(|(_, file)| file.starts_with("belt_axes_")))
        .collect();
    names.insert(0, "bahr/belt_axes.csv");
    names
        .into_iter()
        .flat_map(|name| rows(table(name)).into_iter().map(move |r| (name.split('/').next().unwrap_or(""), r)))
        .filter(|(_, r)| r["kind"] != "xy_gantry" && r["series"] != "DLZPVI")
        .collect()
}

/// The Festo sizes by name, in the order of their names.
fn festo_sizes() -> Vec<String> {
    let mut out: Vec<String> =
        rows(table("festo/belt_axes.csv")).iter().map(|r| format!("{}-{}", r["series"], r["size"])).collect();
    out.sort();
    out
}

/// Whether a belt axis's stroke limit is its printed module length for want of the length that is not stroke
/// (`non_stroke_length_m`: Bahr's other series than the ELZ): the stroke is then overstated by the carriage; `estimated_data`'s
/// column names it "axis-stroke".
fn axis_stroke_is_module_length(name: &str) -> bool {
    belt_axis_rows().iter().any(|(_, r)| {
        format!("{}-{}", r["series"], r["size"]) == name
            && r["kind"] == "axis"
            && number(r, "max_length_one_piece_m").is_some()
            && number(r, "non_stroke_length_m").is_none()
    })
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
            belt: Some((format!("AT{}", in_unit(pitch, 1e3)), get("belt_width_m")?)),
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
    let all = belt_axis_rows();
    let (maker, r) = all.iter().find(|(_, r)| format!("{}-{}", r["series"], r["size"]) == name)?;
    let bahr = *maker == "bahr";
    let kind = if r["kind"] == "xz_gantry" { Kind::XzGantry } else { Kind::Single };
    let radius = number(r, "feed_per_rev_m")? / (2.0 * PI);
    let m = |column: &str| number(r, column);
    let (stroke_x, strokes_z) = match kind {
        Kind::XzGantry => (m("max_length_x_m")?, vec![m("max_length_z_m")?]),
        // The module length (Rexroth's Lmax, Bahr's "aus einem Stück") is stroke plus the length that is not stroke
        // (`non_stroke_length_m`: Rexroth carriage and adapter, Bahr's basic length); a row without it keeps the module
        // length and `estimated_axis_data` flags it.
        Kind::Single => (
            m("max_length_one_piece_m")
                .map(|length| length - m("non_stroke_length_m").unwrap_or(0.0))
                .or(m("stroke_x_max_m"))
                .unwrap_or(f64::INFINITY),
            Vec::new(),
        ),
    };
    let moved = match kind {
        Kind::XzGantry => Some((number(r, "moved_mass_x_kg")?, number(r, "moved_mass_z_kg")?)),
        Kind::Single => None,
    };
    Some(AxisData {
        kind,
        stroke_x: (if bahr { 0.0 } else { m("stroke_x_min_m").unwrap_or(0.0) }, stroke_x),
        strokes_z,
        z_up_to: true,
        pulley_radius: radius,
        // Bahr's belts are HTD (Megadyne's MTD); Rexroth names its AT profile; Festo's EGC and ELGA name none.
        belt: if bahr {
            Some((format!("MTD{}", in_unit(number(r, "belt_pitch_m")?, 1e3)), number(r, "belt_width_m")?))
        } else {
            r.get("belt").filter(|b| b.starts_with("AT")).map(|b| b.to_string()).zip(number(r, "belt_width_m"))
        },
        // The maker's stretch where it states one (ELGA: at the highest feed force).
        ea: number(r, "belt_elongation_force_n").zip(number(r, "belt_elongation")).map(|(f, e)| f / e),
        teeth: number(r, "pulley_teeth")?,
        // Bahr gives the peak feed force; Festo and Rexroth the torque at the pulley as well, which holds where given.
        pulley_torque: number(r, "max_pulley_torque_nm").unwrap_or(number(r, "feed_force_peak_n")? * radius),
        payload_kg: None,
        slide_load_n: number(r, "fz_dynamic_n"),
        max_speed: number(r, "max_speed_m_s")?,
        max_acceleration: if bahr { None } else { number(r, "max_acceleration_m_s2") },
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
///
/// A mechanical axis (`mech_axes`): ball screws by part number and guide sets as "<part number>x<carriages>", joined by
/// "+" (e.g. "R25-20K4-FSC+HGH25CAx4"), as `mechanics::apply_mech_axis` puts them in.
pub fn apply_axis(cfg: &mut Config, name: &str, entry: Option<&MachineEntry>) -> Result<(), ConfigError> {
    let Some(axis) = axis_data(name) else {
        return mechanics::apply_mech_axis(cfg, name).unwrap_or_else(|| Err(unknown("axis", name)));
    };
    match axis_refusal(cfg, &axis) {
        "axis-kind" => return Err(ConfigError::Invalid("parts.axis".into())),
        "axis-stroke" => {
            return Err(ConfigError::Invalid(
                if axis.kind == Kind::XzGantry { "geometry.stroke_x_m" } else { "geometry.stroke_m" }.into(),
            ));
        }
        _ => {}
    }
    if let Some((profile, width)) = &axis.belt {
        apply_belt_at(cfg, profile, *width).ok_or_else(|| unknown("belt", profile))??;
    }
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
/// that differ: a smaller Z motor, another gearbox; an arm's smaller strain wave gear and lighter joint friction in its
/// wrist). See `motor_config`.
pub const PER_MOTOR_TABLES: [&str; 7] = ["parts", "motor", "gearbox", "drive", "ratings", "strain_wave", "joint"];

/// The config one motor sees: each table of `PER_MOTOR_TABLES` with that motor's `[<table>.<id>]` over it, and the
/// motor, gearbox and drive its `[parts.<id>]` names applied from the catalogue over that. A key there has to be one
/// the shared table has (`motor.<id>.rotor_inertia_kgm2` overrides `motor.rotor_inertia_kgm2`). Without such tables
/// the config as it is. The drives, the machine files' drive trains and the per-motor verdicts read their motor's.
/// shortcut: the drives' cycle times, the noise seed and a shared DC link stay those of the shared `[drive]`. (The
/// velocity gain of a motor with parts of its own is retuned by `retuned_per_motor`.)
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
#[derive(Debug, Clone, Default)]
pub struct Candidate {
    /// "" for an axis ruled out before any motor was tried on it.
    pub article: &'static str,
    /// The gearbox as `apply_gearbox` names it; "" where the config's own gearbox stayed.
    pub gearbox: String,
    /// The drive the catalogue pairs with the motor (`paired_drive`); "" where there is none, and the config's stayed.
    pub drive: &'static str,
    /// The supply the motor was tried at (`SUPPLIES`, which says the maker: `vendor_of`); "" for a refused axis.
    pub supply: &'static str,
    /// The axis of the catalogue the motor was tried on; "" for the config's own.
    pub axis: String,
    /// The one motor the candidate is for (`search_motor`, its id); "" for every motor that takes the shared one.
    pub motor: String,
    /// Motor and gearbox of one drive axis, kg.
    pub mass: f64,
    /// The stand-in for the price, which no catalogue file has: the installed weight of everything the combination
    /// names, kg. Motor and gearbox times the number of motors that take it (not those with a motor of their own,
    /// `[motor.<id>]` or `[parts.<id>]`; one for `search_motor`), and the axis at the config's strokes.
    pub cost: f64,
    /// The parts the combination buys, as the search's table spells them: motor, gearbox, drive (each once per motor
    /// that takes it, as `cost` counts them) and axis, with how many of each; the part a config keeps is named too.
    /// Empty for an axis ruled out before any motor was tried.
    pub parts: Vec<(String, f64)>,
    /// The user's price for `parts` (`apply_prices`); `None` without a price list or where a part has no price in it.
    pub price: Option<f64>,
    /// The cycle and its verdicts; `None` where the cycle was not run, with the reason in `refused`. With a duty mix
    /// (`Options::duties`) the first duty's cycle and the verdicts of the mix (`search::mix_verdicts`).
    pub result: Option<(CycleLoad, Vec<Verdict>)>,
    pub refused: &'static str,
    /// With a duty mix: each duty's weight and cycle, in the order of `Options::duties`; empty for one cycle.
    pub duties: Vec<(f64, CycleLoad)>,
    /// The figures of the search's table (`search::Figures`), where the cycle ran.
    pub figures: Option<crate::search::Figures>,
}

impl Candidate {
    pub fn passes(&self) -> bool {
        self.result.as_ref().is_some_and(|(_, verdicts)| verdicts.iter().all(|v| v.ok))
    }

    /// The check closest to (or furthest beyond) its limit, with value over limit (`analysis::utilisation`: for a
    /// life limit over value). The belt's slack does not count while it passes. Two checks of the DC link do not count while they
    /// pass, because their ratio is no measure of how close the cycle is to failing: with a braking resistor
    /// behind them the capacitors fill to their limit by design, in every cycle that brakes hard enough, and
    /// the resistor's checks say what that costs; and the voltage is never below the bus, so its ratio to
    /// the trip level starts at 0.6 or so. Nor does the warm start's convergence while it passes: it says how
    /// well the run was measured, not how close a part is to its limit.
    ///
    /// A run whose drives tripped is infinitely far beyond: its figures end at the trip, where the DC link
    /// was just past its limit, and by that ratio it would rank as the nearest to passing. For one motor (`search_motor`,
    /// `motor`) only its own checks count, and those of the whole machine where they fail: the other motors' are the
    /// config's, and a check of the whole machine mostly theirs.
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
            .filter(|v| (v.limit > 0.0 || !v.ok) && !by_design(v))
            // Slack while it passes and the warm start's convergence while it passes say nothing of a limit; failing, they count.
            .filter(|v| {
                !v.ok || !["belt-least-tension", "thermal-warm-start", "thermal-cycle-repeat"].contains(&v.check)
            })
            // For one motor (`search_motor`): its own checks, and those of the whole machine where they fail.
            .filter(|v| {
                self.motor.is_empty()
                    || v.motor.map_or(!v.ok, |k| load.motors.get(k).is_some_and(|m| m.id == self.motor))
            })
            .map(|v| (v.check, crate::analysis::utilisation(v)))
            .max_by(|a, b| a.1.total_cmp(&b.1))
    }
}

/// In proportion to the inertia the motors turn, rotors and load together: the torque the rigid machine
/// needs per unit of acceleration at the middle of its travel, summed over axes and motors, over the gear
/// ratio (a motor behind a higher ratio turns faster for the same axis acceleration).
fn inertia_seen(entry: &MachineEntry, cfg: &Config) -> Result<f64, ConfigError> {
    Ok(inertia_turned(entry, cfg)?.iter().sum())
}

/// The motor's angle per unit of each axis's travel at the middle of the travel, `[axis][motor]`, from `Machine::inverse`:
/// a central difference of a step small against the travel, exact for a linear map, 1e-8 off for the rest.
fn motor_rates(machine: &dyn Machine, pose: &[f64]) -> Vec<Vec<f64>> {
    let angles = |pose: &[f64]| {
        let mut out = vec![0.0; machine.motors().len()];
        machine.inverse(pose, &mut out);
        out
    };
    (0..machine.axes().len())
        .map(|axis| {
            let step = 1e-6 * (machine.axes()[axis].max - machine.axes()[axis].min).abs().max(1e-3);
            let (mut ahead, mut behind) = (pose.to_vec(), pose.to_vec());
            ahead[axis] += step;
            behind[axis] -= step;
            angles(&ahead).iter().zip(angles(&behind)).map(|(a, b)| (a - b) / (2.0 * step)).collect()
        })
        .collect()
}

/// Per motor, the inertia it turns (rotor, gearbox and load, summed over the axes that move it): the torque the rigid
/// machine needs per unit of an axis's acceleration over the motor's angle per unit of that axis's travel
/// (`motor_rates`), so a pulley radius, a screw lead or a rack's pinion count as well as the gear ratio.
fn inertia_turned(entry: &MachineEntry, cfg: &Config) -> Result<Vec<f64>, ConfigError> {
    let torques = torque_per_acceleration(entry, cfg)?;
    let machine = (entry.build)(cfg)?;
    let rates = motor_rates(machine.as_ref(), &sim_core::testkit::mid_pose(machine.as_ref()));
    Ok((0..machine.motors().len())
        .map(|k| torques.iter().zip(&rates).filter(|(_, r)| r[k].abs() > 1e-9).map(|(t, r)| (t[k] / r[k]).abs()).sum())
        .collect())
}

/// The torque each motor of the rigid machine needs per unit of each axis's acceleration at the middle of its travel,
/// standing: `[axis][motor]`, gravity taken out.
pub(crate) fn torque_per_acceleration(entry: &MachineEntry, cfg: &Config) -> Result<Vec<Vec<f64>>, ConfigError> {
    let machine = (entry.build)(cfg)?;
    let pose = sim_core::testkit::mid_pose(machine.as_ref());
    let (axes, motors) = (machine.axes().len(), machine.motors().len());
    let (mut holding, mut moving) = (vec![0.0; motors], vec![0.0; motors]);
    machine.feedforward_torque(&pose, &vec![0.0; axes], &mut holding);
    let mut out = Vec::new();
    for axis in 0..axes {
        let mut acc = vec![0.0; axes];
        acc[axis] = 1.0;
        machine.feedforward_torque(&pose, &acc, &mut moving);
        out.push(moving.iter().zip(&holding).map(|(m, h)| m - h).collect());
    }
    Ok(out)
}

/// The inertia ratio of each motor, load over motor, at the middle of the travel: the inertia the motor turns
/// besides its rotor (the load, and the gearbox's own inertia with it) over the rotor's inertia, where the rotor's is
/// that of the motor's own config (`motor_config`). Per axis the inertia the motor turns is the torque the rigid machine
/// needs per unit of that axis's acceleration (`torque_per_acceleration`) over the motor's acceleration it takes (from
/// `Machine::inverse`, standing: no speed, so no term of the speed squared); of the axes that move the motor the
/// largest. NaN for a motor no axis moves. Assumes the machine's feedforward has the rotors in, as every machine's has.
pub fn inertia_ratios(entry: &MachineEntry, cfg: &Config) -> Result<Vec<f64>, ConfigError> {
    let torques = torque_per_acceleration(entry, cfg)?;
    let machine = (entry.build)(cfg)?;
    let rates = motor_rates(machine.as_ref(), &sim_core::testkit::mid_pose(machine.as_ref()));
    let motors = machine.motors().to_vec();
    let mut out = Vec::new();
    for (k, motor) in motors.iter().enumerate() {
        let rotor = motor_config(cfg, motor.id)?.positive("motor.rotor_inertia_kgm2")?;
        let mut ratio = f64::NAN;
        for (torque, rate) in torques.iter().zip(&rates) {
            let rate = rate[k];
            if rate.abs() > 1e-9 {
                let load = torque[k] / rate - rotor;
                ratio = if ratio.is_nan() { load / rotor } else { ratio.max(load / rotor) };
            }
        }
        out.push(ratio);
    }
    Ok(out)
}

/// The velocity gain of `before` scaled for the parts of `after` as `with_parts` says: by the square root of the ratio
/// of the inertia the motors turn (`inertia_seen`) times that of the rotor's (with the gearbox's), and on an elastic
/// machine bounded by `bounded_gain`; the position gain lowered where the cascade needs it (`cascade`, the largest
/// spread any motor needs, the keys being shared).
fn retuned(entry: &MachineEntry, before: &Config, after: &mut Config) -> Result<(), ConfigError> {
    let rotor = |cfg: &Config| {
        Ok::<f64, ConfigError>(
            cfg.positive("motor.rotor_inertia_kgm2")? + cfg.non_negative("gearbox.inertia_at_input_kgm2")?,
        )
    };
    let rotor_ratio = rotor(after)? / rotor(before)?;
    let total_ratio = inertia_seen(entry, after)? / inertia_seen(entry, before)?;
    let ratio = total_ratio * rotor(after)? / rotor(before)?;
    let old = before.positive("drive.velocity_kp_nm_s_rad")?;
    let machine = (entry.build)(after)?;
    let elastic = machine.resonance_hz(&sim_core::testkit::mid_pose(machine.as_ref())).is_some();
    // A linear motor's coil is the mover itself, joined to the load by no belt or gearbox: the loop sees the whole mass at
    // every frequency, so its crossover over that mass is the config's (the geometric mean with the coil's ratio made
    // the AL8021-0E's mover loop on the track oscillate, 1592 against 1056 that keeps the config's crossover).
    let gain = if is_linear_motor(after) {
        old * total_ratio
    } else {
        bounded_gain(old * ratio.sqrt(), old, rotor_ratio, elastic)
    };
    set(after, "drive.velocity_kp_nm_s_rad", gain)?;
    let (own, new) = (cascade_margins(entry, before)?, cascade_margins(entry, after)?);
    let spread = (0..own.len()).map(|k| cascade_spread(own[k], new[k])).fold(1.0, f64::max);
    for (key, value) in cascade(after, spread)? {
        set(after, key, value)?;
    }
    Ok(())
}

/// The least position-cascade margin (`cascade_margins`) a retune leaves a motor [rule chosen from a sweep]: a quarter
/// above the edge of stability, or the config's own margin where that is smaller. Swept on 17 November 2026 on the
/// cases of `bounded_gain`'s comment (Kv alone lowered; Tn alone or both together settled worse at every floor): 1.0
/// left the EXCT-30 with the 1FK2104-5AF warm-start drifts of 0.3 to 2.1 K (not periodic) under Kp x0.9 and x1.1, frame
/// compliance or tool flex off; 1.5 took the ELZ-125 with its own motor, which passes at 1.07, to 0.175 mm settling
/// (limit 0.1 mm); 1.25 keeps it passing (0.077 mm) and the 1FK2104-5AF calm in every variation.
pub const CASCADE_MARGIN: f64 = 1.25;

/// Per motor, the position cascade's margin from instability on a rigid inertia: ω (Tn + 1 / Kv), with ω = Kp / J the
/// velocity loop's crossover over the inertia the motor turns (`inertia_ratios`: rotor, gearbox and load of the axis
/// that gives the largest) and Tn, Kv, Kp the motor's own (`motor_config`). A P position loop (Kv) on a PI velocity
/// loop (Kp, Tn) on an inertia J, torque Kp (1 + 1/(Tn s)) (Kv e - s x) with e the position error, has the
/// characteristic polynomial s³ + ω s² + ω (1/Tn + Kv) s + ω Kv / Tn; by Routh it is stable while ω (1/Tn + Kv) >
/// Kv / Tn, that is while this margin exceeds 1. It assumes a rigid inertia (the belt's resonance and the drive's delays
/// left out: those `bounded_gain` holds) and takes the axis with the most inertia, so it errs low. NaN for a motor no
/// axis moves.
fn cascade_margins(entry: &MachineEntry, cfg: &Config) -> Result<Vec<f64>, ConfigError> {
    let ratios = inertia_ratios(entry, cfg)?;
    let machine = (entry.build)(cfg)?;
    let mut out = Vec::new();
    for (motor, ratio) in machine.motors().iter().zip(ratios) {
        let own = motor_config(cfg, motor.id)?;
        let inertia = own.positive("motor.rotor_inertia_kgm2")? * (1.0 + ratio);
        let (tn, kv) = (own.positive("drive.velocity_tn_s")?, own.positive("drive.kv_1_s")?);
        out.push(own.positive("drive.velocity_kp_nm_s_rad")? / inertia * (tn + 1.0 / kv));
    }
    Ok(out)
}

/// The factor the retuned margin `new` has to grow by to reach its floor: `CASCADE_MARGIN`, or the config's own margin
/// `own` where that is smaller; 1 where it is there already (and for a motor no axis moves, NaN).
fn cascade_spread(own: f64, new: f64) -> f64 {
    let floor = own.min(CASCADE_MARGIN);
    if new < floor { floor / new } else { 1.0 }
}

/// The position gain `drive.kv_1_s` that grows the cascade margin (`cascade_margins`) of `cfg` (the motor's config with
/// its retuned gain) by `spread`: 1 / ((Tn + 1/Kv) spread - Tn), Tn as it is. Lowering Kv alone settled best: the
/// integral, which holds the axis against friction in the dwell, keeps its speed. Empty for `spread` 1, so a part that
/// leaves the margin at its floor or above (the config's own motor among them) leaves the key as the config has it.
fn cascade(cfg: &Config, spread: f64) -> Result<Vec<(&'static str, f64)>, ConfigError> {
    if spread <= 1.0 {
        return Ok(Vec::new());
    }
    let (tn, kv) = (cfg.positive("drive.velocity_tn_s")?, cfg.positive("drive.kv_1_s")?);
    Ok(vec![("drive.kv_1_s", 1.0 / ((tn + 1.0 / kv) * spread - tn))])
}

/// `retuned`'s gain `scaled` on a machine that reports a resonance (`elastic`): at most `old` times `rotor_ratio`, the
/// change of the rotor's inertia with the gearbox's, so that the speed loop's crossover over that inertia, Kp / J_M,
/// stays the config's own. On a rigid model the scaled gain stands (the loop sees the whole inertia at every frequency).
///
/// Above the drive train's resonance the belt or the arm no longer carries the load along and the loop sees the
/// rotor-side inertia alone; in every case swept the crossover lies above the first anti-resonance and its limit is
/// the drive's delays, not the mechanics. Swept on 16 November 2026 (sim-cli verdicts, margins, analyze with
/// `--set drive.velocity_kp_nm_s_rad`): the largest gain that passes every verdict with a phase margin of 45 degrees
/// (`velocity_loop_margins`) is Kp / J_M = 870 to 1550 rad/s in all of them, while the same gain is 1.0 to 12 times
/// the first resonance (`Machine::resonance_hz`, 20.5 to 160 Hz): no fixed fraction of the resonance fits, a fixed
/// crossover over the rotor-side inertia does (the shipped configs sit at 830 to 1330 rad/s).
///
/// ```text
/// case (J_M, J_total kg m2; resonance Hz)          max at 45 deg  best tracking*  before   this rule
/// one-axis belt drive (3.0e-5, 1.34e-4; 65.8)       0.036          0.07            0.025    0.025
///   pulley radius x0.5 / x2                        0.037 / 0.033  0.07 / 0.07     0.0166 / 0.025   same
///   gear ratio x0.5 / x2                           0.040 / 0.037  0.09 / 0.07     0.025 / 0.0162   same
///   Bahr ELZ-40 / ELZ-80 / ELZ-125 (r 16/31/48 mm) 0.037 / 0.026 / 0.03  0.07 / 0.055 / 0.037   0.0212 / 0.025 / 0.025   same
///   1FK2103-4AH (1.4e-5, 1.18e-4)                  0.019          0.045           0.016    0.0117
///   1FK2204-5AF (1.2e-4, 2.24e-4)                  0.127          0.2             0.025    0.0646
/// T-shaped gantry (3.0e-5, 1.28e-4; 20.5), own 0.04 0.047          0.08            -        -
///   1FK2204-5AF (1.2e-4, 2.18e-4)                  0.13           0.2             0.04     0.108
/// three-arm delta (2.4e-4, 3.9e-3; 43.0), own 0.25 0.31           0.6             -        -
/// * every verdict passes, any phase margin (the tracking error falls with the gain up to the edge of stability)
/// ```
///
/// The bound replaces a third of the resonance times J_M, which held every grown gain at the config's own (the
/// 1FK2204-5AF on the one-axis belt drive at 0.025, crossover 21 Hz below the anti-resonance), and lowers a lighter
/// rotor's gain where the geometric mean would raise its crossover (the 1FK2103-4AH from 0.016 to 0.0117, phase margin
/// 71 degrees). The ELZ-125 scaled to 0.0573 ran at 2.05 times the motor's peak; bounded it keeps 0.025 and passes.
///
/// Neither a lower bound nor more gain for a lighter rotor fits, swept on 17 November 2026 (sim-cli verdicts and
/// margins, kv 50 1/s, Tn 10 ms, the gains Kp / J_total in rad/s, failing / first passing):
///
/// ```text
/// case (inertia ratio; J_total kg m2)    fails at  passes at  phase margin 45 deg at Kp / J_M
/// ELZ-125, 1FK2103-4AH (48; 6.88e-4)    34.9      37.8 only  - (0.027 and above fail again)
/// ELZ-125, own motor (22; 7.04e-4)       28.4      35.5       1000
/// ELZ-125, 1FK2104-5AF (12; 7.30e-4)     37.0      41.1       - (40 degrees at most)
/// ELZ-80, 1FK2103-4AH (18; 2.60e-4)      30.8      38.4       1071
/// ELZ-80, 1FK2103-2AH (26; 2.55e-4)      39.2      47.0       1236
/// pulley x2, 1FK2103-4AH (29; 4.19e-4)   35.8      43.0       1393
/// one-axis belt drive, 1FK2103-4AH (7.4) -         -          1357
/// one-axis belt drive, 1FK2103-2AH (11)  -         -          1720
/// ```
///
/// A lower bound Kp >= 0.75 kv J_total (37.5 rad/s) took the ELZ-125 with its own motor from 0.025 (passes) to 0.0264,
/// which misses the settling check (0.111 mm against 0.1 mm), and still left the ELZ-125 with the 1FK2103-4AH (0.0258:
/// 0.145 mm), the pulley x2 with it (0.0157: 1.5 mm) and the gear ratio x0.5 with it (12 mm) failing. Below their
/// windows these motors run at their peak (motor-peak 1.02 to 1.10, drive current over its rating) and the axis drags
/// centimetres behind; their windows, where there are any, are a few hundredths of the gain wide and not monotonic
/// (the ELZ-125 with the 1FK2103-4AH passes at 0.026 and fails at 0.024 and 0.027): motors too small for this cycle,
/// which the verdicts report, not a gain a rule of inertias can find. More gain for a lighter
/// rotor (the bound old times the square root of `rotor_ratio`, 0.0171 for the 1FK2103-4AH) would take its phase
/// margin on the ELZ-80 to about 37 degrees (44.3 at 0.015, 31.0 at 0.018): the allowed Kp / J_M depends on the load as
/// well (1071 to 1720 for the same motor), which the inertias alone do not tell.
///
/// What those failing cases share is the position cascade: held to the rotor's crossover, a light rotor's Kp over the
/// whole inertia falls toward 1 / (Tn + 1/Kv), the edge where the P position loop on the PI velocity loop turns
/// unstable (`cascade_margins`). So `retuned` lowers Kv to keep that margin at `CASCADE_MARGIN` at least. Swept again on
/// 17 November 2026 (with gravity in the guides' commanded acceleration; margin before, Kv 50 -> after, verdicts):
///
/// ```text
/// case                                   margin  Kv     before                          after
/// one-axis belt drive, ELZ-125           1.07    39.7   pass (settling 0.097 mm)        pass (0.077 mm)
///   ELZ-125, 1FK2103-4AH                 0.51    15.7   oscillates, 3.1 N m RMS          calm 0.42 N m; settling 0.61 mm
///   ELZ-80, 1FK2103-2AH                  0.91    32.1   settling 2.85 mm, warm start    settling 0.16 mm
///   pulley x2, 1FK2103-4AH               0.84    28.7   settling 1.50 mm, warm start    settling 0.35 mm
/// EXCT-30, 1FK2104-5AF                   0.73    24.1   oscillates, 5.1 N m RMS          calm 1.21 N m; settling 0.15 mm
///   Kp x0.9 / x1.1                       0.66/0.80      5.6 N m / calm (warm start)      1.21 / 1.23 N m (x0.9: warm start 1.4 K)
///   frame compliance / tool flex / guide friction off   calm / 5.5 / 5.6 N m       1.22 / 1.21 / 1.11 N m
/// ```
///
/// Every other case of the tables above has a margin of 1.35 or more and keeps its Kv and its figures (the 1FK2204-6AF
/// on the EXCT-30 1.98, the pulley x2 1.73, the gear ratio x0.5 1.68). The velocity loop's margins (`velocity_loop_margins`)
/// move by at most 0.7 dB of gain margin, and the phase margins rise (ELZ-125 from 53.4 to 54.7, with the 1FK2103-4AH
/// from 68.7 to 80.5 degrees). The settling misses that stay are motors whose Kp, held
/// by the rotor, cannot both hold the cascade and settle within the dwell: what the verdicts are there to report.
fn bounded_gain(scaled: f64, old: f64, rotor_ratio: f64, elastic: bool) -> f64 {
    if elastic { scaled.min(old * rotor_ratio) } else { scaled }
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
/// rotor only makes the loop needlessly slow. Where the retuned gain leaves the position cascade too close to its edge
/// (`cascade_margins` below `CASCADE_MARGIN` and below the config's own), `drive.kv_1_s` is lowered to keep it there;
/// `drive.velocity_tn_s` stays. On the EXCT-30 every 1FK2 runs calmly so.
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
    guide_carriages(cfg)?; // refused here, where every front end resolves its config, not only when a guide is searched
    let mut out = cfg.clone();
    apply_parts(&mut out, parts, Some(entry))?;
    if parts.moves() {
        retuned(entry, cfg, &mut out)?;
    }
    retuned_per_motor(entry, cfg, &mut out)?;
    Ok(out)
}

/// The velocity gain of each motor that `[parts.<id>]` names parts for, written to `drive.<id>.velocity_kp_nm_s_rad`
/// as `retuned` does for the shared one (the square root of the ratio of the inertia that motor turns times that of its
/// rotor, `bounded_gain` on an elastic machine, a linear motor's by the whole mass), from the motor's gain in `cfg`
/// without its own parts, and its `drive.<id>.kv_1_s` where its cascade needs it (`cascade`). A gain the config gives
/// per motor stays.
fn retuned_per_motor(entry: &MachineEntry, cfg: &Config, out: &mut Config) -> Result<(), ConfigError> {
    let machine = (entry.build)(out)?;
    let ids: Vec<String> = machine.motors().iter().map(|m| m.id.to_string()).collect();
    let elastic = machine.resonance_hz(&sim_core::testkit::mid_pose(machine.as_ref())).is_some();
    for (index, id) in ids.iter().enumerate() {
        let named = !id.is_empty() && cfg.get(&format!("parts.{id}")).ok().is_some_and(|v| v.is_table());
        if !named || cfg.get(&format!("drive.{id}.velocity_kp_nm_s_rad")).is_ok() {
            continue;
        }
        // The config as the file states it: without this motor's own parts.
        let mut all = cfg.to_text()?.parse::<toml::Table>().map_err(|e| ConfigError::Parse(e.to_string()))?;
        if let Some(parts) = all.get_mut("parts").and_then(|p| p.as_table_mut()) {
            parts.remove(id.as_str());
        }
        let bare = Config::parse(&toml::to_string(&all).map_err(|e| ConfigError::Parse(e.to_string()))?)?;
        let rotor = |c: &Config| -> Result<f64, ConfigError> {
            let m = motor_config(c, id)?;
            Ok(m.positive("motor.rotor_inertia_kgm2")? + m.non_negative("gearbox.inertia_at_input_kgm2")?)
        };
        let (rotor_ratio, total_ratio) =
            (rotor(out)? / rotor(&bare)?, inertia_turned(entry, out)?[index] / inertia_turned(entry, &bare)?[index]);
        let old = motor_config(&bare, id)?.positive("drive.velocity_kp_nm_s_rad")?;
        let gain = if is_linear_motor(&motor_config(out, id)?) {
            old * total_ratio
        } else {
            bounded_gain(old * (total_ratio * rotor_ratio).sqrt(), old, rotor_ratio, elastic)
        };
        *out = with_own_keys(out, "drive", id, &[("velocity_kp_nm_s_rad", toml::Value::Float(gain))])?;
        *out = with_cascade(entry, &bare, out, index, id)?;
    }
    Ok(())
}

/// `cfg` with `<table>.<id>.<key>` set for each pair, the motor's table made where the config has none.
fn with_own_keys(cfg: &Config, table: &str, id: &str, pairs: &[(&str, toml::Value)]) -> Result<Config, ConfigError> {
    let mut all = cfg.to_text()?.parse::<toml::Table>().map_err(|e| ConfigError::Parse(e.to_string()))?;
    let shared = all.entry(table).or_insert_with(|| toml::Value::Table(toml::Table::new()));
    let shared = shared.as_table_mut().ok_or_else(|| ConfigError::Type(table.into()))?;
    let mine = shared.entry(id).or_insert_with(|| toml::Value::Table(toml::Table::new()));
    let mine = mine.as_table_mut().ok_or_else(|| ConfigError::Type(format!("{table}.{id}")))?;
    for (key, value) in pairs {
        mine.insert(key.to_string(), value.clone());
    }
    Config::parse(&toml::to_string(&all).map_err(|e| ConfigError::Parse(e.to_string()))?)
}

/// `out` with motor `index`'s (`id`) Kv lowered in its own `drive.<id>` table (`cascade`) so that its cascade margin is
/// at least its floor, `own` being the config before the motor's parts.
fn with_cascade(
    entry: &MachineEntry,
    own: &Config,
    out: &Config,
    index: usize,
    id: &str,
) -> Result<Config, ConfigError> {
    let spread = cascade_spread(cascade_margins(entry, own)?[index], cascade_margins(entry, out)?[index]);
    let keys: Vec<(&str, toml::Value)> = cascade(&motor_config(out, id)?, spread)?
        .into_iter()
        .map(|(key, value)| (key.trim_start_matches("drive."), toml::Value::Float(value)))
        .collect();
    if keys.is_empty() { Ok(out.clone()) } else { with_own_keys(out, "drive", id, &keys) }
}

/// A config with one value changed that moves what the motors turn (`gearbox.ratio`, `geometry.pulley_radius_m`), the
/// velocity gain retuned for it as `with_parts` retunes it for a part. `cfg` has its parts in (`resolve`).
pub fn with_value(entry: &MachineEntry, cfg: &Config, path: &str, value: f64) -> Result<Config, ConfigError> {
    let mut out = cfg.clone();
    set(&mut out, path, value)?;
    retuned(entry, cfg, &mut out)?;
    Ok(out)
}

/// The catalogue gearboxes that fit a motor, at every ratio, named as `apply_gearbox` takes them: the question the
/// search asks before it sweeps ratios. `series` picks makers and series: "" all of them, a maker ("wittenstein",
/// "neugart", "siemens"), a maker's table ("wittenstein-sp", "neugart-ple", "siemens-nrk", "harmonicdrive-csf"), or a
/// prefix of the size ("NRK", "NRK070"). Siemens' NRB, NRK and NLC fit the 1FK2 and 1FT2 of their frame (1FK2104 and
/// 1FK2204 have frame "04"), its SP+ the 1FT2 only, as Siemens sells them; another maker's gearbox fits a motor whose
/// shaft its clamping hub takes (`motor_shaft_diameter`; one hub per size and ratio, the smallest that does).
pub fn gearboxes_for_motor(article: &str, series: &str) -> Vec<String> {
    mechanics::fitting(article, series).into_iter().map(|(name, _)| name).collect()
}

/// `gearboxes_for_motor` with a ratio between half and twice `ratio`.
pub fn fitting_gearboxes(article: &str, series: &str, ratio: f64) -> Vec<String> {
    let near = |n: f64| n >= 0.5 * ratio && n <= 2.0 * ratio;
    mechanics::fitting(article, series).into_iter().filter(|(_, n)| near(*n)).map(|(name, _)| name).collect()
}

/// The axes of the catalogue a config's machine can be built from, each with why it is refused ("" where it is not)
/// by its catalogue numbers alone: the belt axes of its kind (`axis_names`, `axis_refusal`) and the mechanical axes
/// (`mech_axes`: ball screws, and guide sets where the config rates guides). The question the search asks before it
/// tries motors on them.
pub fn axes_for(cfg: &Config) -> Vec<(String, &'static str)> {
    let belts = axis_names().into_iter().filter_map(|name| {
        let reason = axis_refusal(cfg, &axis_data(&name)?);
        (reason != "axis-kind").then_some((name, reason))
    });
    belts.chain(mech_axes(cfg).into_iter().map(|(name, reason, _)| (name, reason))).collect()
}

/// One combination to try: motor, gearbox ("" the config's), drive ("" the config's), axis name ("" the config's) and the
/// axis's weight.
#[derive(Debug, Clone, Default)]
struct Job {
    article: &'static str,
    gearbox: String,
    drive: &'static str,
    axis: String,
    axis_weight: f64,
    /// The id of the one motor the candidate is for (`search_motor`); "" for all of them.
    motor: String,
}

/// What a search tries beside the motors (`search`): `gearboxes` a series to try with each motor (`fitting_gearboxes`),
/// `axes` every axis of the catalogue (`search_axes`), `motor` one motor of a machine with several (`search_motor`),
/// `drives` which drives with each motor (`Drives`), and `duties` the duty mix: each duty's config and weight (how many
/// of its cycles to one of weight 1), the first the config searched on; empty for that config alone. With a mix each
/// candidate runs every duty and is judged by `search::mix_verdicts`.
#[derive(Debug, Clone, Copy, Default)]
pub struct Options<'a> {
    pub gearboxes: Option<&'a str>,
    pub axes: bool,
    pub motor: Option<&'a str>,
    pub drives: Drives,
    pub duties: &'a [(Config, f64)],
}

/// The gearbox ratio a series' gearboxes are picked around for a machine without one (ratio 1, a direct drive): the
/// classic optimum for a pure inertia, the square root of the load's inertia over the rotor's (`search::optimum_ratio`),
/// for the motor `article` and the busiest motor of the config. `None` where it cannot be told.
fn direct_drive_ratio(entry: &MachineEntry, cfg: &Config, article: &str, supply: &str) -> Option<f64> {
    let own = parts_applied(cfg).ok()?;
    let rotor = own.positive("motor.rotor_inertia_kgm2").ok()?;
    let load = inertia_ratios(entry, &own).ok()?.into_iter().filter(|r| r.is_finite()).reduce(f64::max)? * rotor;
    Some(crate::search::optimum_ratio(load, motor_data(article, supply).ok()?.rotor_kgm2))
}

/// The motors a config's machine can take at a supply: a linear motor machine (`is_linear_motor`) the linear motors
/// of the supply (`linear_motor_articles_for`), every other the rotary ones (`motor_articles_for`).
fn articles_for(cfg: &Config, supply: &str) -> Vec<&'static str> {
    if is_linear_motor(cfg) { linear_motor_articles_for(supply) } else { motor_articles_for(supply) }
}

/// The jobs of one config: every motor `wanted` takes at the supply, each with the drives `drives` picks and, with a
/// series, every fitting gearbox of it: around the config's ratio where the machine has a gearbox (ratio above 1), around
/// the classic optimum (`direct_drive_ratio`) where it is a direct drive, which is tried without a gearbox as well.
/// A linear motor's magnet track at the config's stroke counts into the axis's weight (`magnet_track_weight`).
#[allow(clippy::too_many_arguments)]
fn motor_jobs(
    entry: &MachineEntry,
    cfg: &Config,
    supply: &str,
    wanted: &dyn Fn(&str) -> bool,
    options: &Options,
    axis: &str,
    axis_weight: f64,
    motor: &str,
) -> Vec<Job> {
    let own = if motor.is_empty() { Ok(cfg.clone()) } else { motor_config(cfg, motor) };
    let ratio =
        own.and_then(|own| parts_applied(&own)).ok().and_then(|own| own.f64("gearbox.ratio").ok()).unwrap_or(1.0);
    let mut out = Vec::new();
    for article in articles_for(cfg, supply).into_iter().filter(|a| wanted(a)) {
        let axis_weight = axis_weight + magnet_track_weight(cfg, article, supply);
        let names = match options.gearboxes {
            // A motor no gearbox of the series fits is tried with the config's own (gearbox "").
            Some(series) if ratio > 1.0 => Some(fitting_gearboxes(article, series, ratio))
                .filter(|g| !g.is_empty())
                .unwrap_or_else(|| vec![String::new()]),
            // A direct drive: as it is, and with the gearboxes around the optimum.
            Some(series) if motor.is_empty() => std::iter::once(String::new())
                .chain(
                    direct_drive_ratio(entry, cfg, article, supply)
                        .map_or_else(Vec::new, |optimum| fitting_gearboxes(article, series, optimum)),
                )
                .collect(),
            _ => vec![String::new()],
        };
        for gearbox in names {
            for drive in drives_tried(article, supply, options.drives) {
                out.push(Job {
                    article,
                    gearbox: gearbox.clone(),
                    drive,
                    axis: axis.to_string(),
                    axis_weight,
                    motor: motor.to_string(),
                });
            }
        }
    }
    out
}

/// Runs the jobs on all processors and ranks them: the passing ones first, cheapest first (`Candidate::cost`, then
/// `mass`), the others after them, nearest to passing first. A motor not offered at the supply is left out.
///
/// Every candidate is simulated: the axis limits are not a screen (`check_limits` takes every axis at its highest speed
/// and acceleration at once, which the cycle need not ask, and so ruled out motors that passed every verdict). What the
/// cycle asks of a motor is judged by the verdicts: "motor-peak" against its torque-speed curve, "motor-speed" against
/// its maximum speed. A candidate whose config or cycle cannot run is returned with `result` empty and why in `refused`.
fn run_jobs(
    entry: &MachineEntry,
    (cfg, duties): (&Config, &[(Config, f64)]),
    supply: &str,
    jobs: Vec<Job>,
    mut out: Vec<Candidate>,
) -> Vec<Candidate> {
    // As the analyses' batches: counted in the current job and stopped by its cancel.
    let found = crate::analysis::on_all_cores(jobs.len(), |i| try_job(entry, cfg, duties, supply, &jobs[i]));
    out.extend(found.into_iter().flatten());
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
            .then(a.drive.cmp(b.drive))
    });
}

/// The user's prices by part number (`--prices`): a CSV `part_number,price` with the header, `#` notes and blank lines
/// passed over as the catalogue's tables are. The currency is the user's; the file carries none. No shipped table has
/// a price (`docs/catalogue.md`).
pub struct PriceList(HashMap<String, f64>);

impl PriceList {
    /// Reads the text; on a fault the line (1-based), the message key and the argument it names (the part number or
    /// the field), for the front end to put in words.
    pub fn parse(text: &str) -> Result<Self, (usize, &'static str, String)> {
        let mut lines = text.lines().enumerate().filter(|(_, l)| !l.starts_with('#') && !l.trim().is_empty());
        let (at, header) = lines.next().unwrap_or((0, ""));
        if header.split(',').map(str::trim).ne(["part_number", "price"]) {
            return Err((at + 1, "cli-error-prices-header", header.to_string()));
        }
        let mut prices = HashMap::new();
        for (i, line) in lines {
            let fields: Vec<&str> = line.split(',').map(str::trim).collect();
            let [part, price] = fields[..] else { return Err((i + 1, "cli-error-prices-fields", line.to_string())) };
            if part.is_empty() {
                return Err((i + 1, "cli-error-prices-fields", line.to_string()));
            }
            let price = price.parse::<f64>().ok().filter(|p| *p > 0.0 && p.is_finite());
            let price = price.ok_or_else(|| (i + 1, "cli-error-prices-value", format!("{part}: {}", fields[1])))?;
            if prices.insert(part.to_string(), price).is_some() {
                return Err((i + 1, "cli-error-prices-duplicate", part.to_string()));
            }
        }
        Ok(Self(prices))
    }
}

/// Sets each candidate's price (`Candidate::price`): the sum over its parts of the count times the list's price, only
/// where every part has one. Returns the part numbers the list lacks, each once, in order of first need.
pub fn apply_prices(found: &mut [Candidate], list: &PriceList) -> Vec<String> {
    let mut missing: Vec<String> = Vec::new();
    for c in found {
        c.price = c.parts.iter().map(|(name, count)| list.0.get(name).map(|p| p * count)).sum::<Option<f64>>();
        c.price = c.price.filter(|_| !c.parts.is_empty());
        for (name, _) in c.parts.iter().filter(|(name, _)| !list.0.contains_key(name)) {
            if !missing.contains(name) {
                missing.push(name.clone());
            }
        }
    }
    missing
}

/// A job's config made from a duty's config `base`: the axis named, then the parts put in as `with_named` (or, for one
/// motor, `with_motor_part`) puts them; and how many motors take the shared motor (1 for one motor's job).
fn job_config(entry: &MachineEntry, base: &Config, supply: &str, job: &Job) -> Result<(Config, f64), ConfigError> {
    let mut cfg = base.clone();
    if !job.axis.is_empty() {
        cfg.set_text(&format!("parts.axis = {:?}", job.axis))?;
    }
    if !job.motor.is_empty() {
        return Ok((with_motor_part(entry, &cfg, &job.motor, job.article, supply, &job.gearbox, job.drive)?, 1.0));
    }
    let mut parts = Parts::of(&cfg)?;
    (parts.motor, parts.supply) = (job.article, supply);
    if !job.gearbox.is_empty() {
        parts.gearbox = &job.gearbox;
    }
    if !job.drive.is_empty() {
        parts.drive = job.drive;
    }
    let with_parts = with_named(entry, &cfg, &parts)?;
    // The motors that take the shared motor: not those with a motor of their own (`[motor.<id>]`, `[parts.<id>]`).
    let shared = |id: &str| {
        cfg.get(&format!("motor.{id}")).is_err()
            && cfg.get(&format!("parts.{id}.motor")).ok().and_then(|v| v.as_str()).is_none_or(str::is_empty)
    };
    let motors = (entry.build)(&with_parts).map_or(1, |m| m.motors().iter().filter(|m| shared(m.id)).count());
    Ok((with_parts, motors as f64))
}

/// The config a candidate of a search on `cfg` ran with (its axis, motor, gearbox and drive put in, the velocity gain
/// retuned): for judging it further, in the error bands (`search::robust_ranking`) or on its own.
pub fn candidate_config(entry: &MachineEntry, cfg: &Config, c: &Candidate) -> Result<Config, ConfigError> {
    let job = Job {
        article: c.article,
        gearbox: c.gearbox.clone(),
        drive: c.drive,
        axis: c.axis.clone(),
        axis_weight: 0.0,
        motor: c.motor.clone(),
    };
    Ok(job_config(entry, cfg, c.supply, &job)?.0)
}

/// The cycle of a config judged; the reason where it cannot run.
fn judge(entry: &MachineEntry, cfg: &Config) -> Result<(CycleLoad, Vec<Verdict>), &'static str> {
    let mut sim = Sim::new(entry, cfg).map_err(|_| "config")?;
    let load = analyse_cycle_with(&mut sim, false).map_err(|e| e.key)?;
    let v = verdicts(cfg, &load).map_err(|_| "config")?;
    Ok((load, v))
}

fn try_job(entry: &MachineEntry, cfg: &Config, duties: &[(Config, f64)], supply: &str, job: &Job) -> Option<Candidate> {
    // Not offered at this supply, or a part the config cannot take.
    let (with_parts, motors) = job_config(entry, cfg, supply, job).ok()?;
    // The gearbox the motor drives through: the candidate's, else its own (`[parts.<id>]` for one motor).
    let own = if job.motor.is_empty() { Ok(cfg.clone()) } else { motor_config(cfg, &job.motor) };
    let own = own.ok();
    let own_parts = own.as_ref().and_then(|own| Parts::of(own).ok());
    let own_gearbox = own_parts.as_ref().map(|p| p.gearbox.to_string()).unwrap_or_default();
    let gearbox = if job.gearbox.is_empty() { own_gearbox.as_str() } else { job.gearbox.as_str() };
    let drive = if job.drive.is_empty() { own_parts.as_ref().map_or("", |p| p.drive) } else { job.drive };
    // What the combination buys, for a price list: every motor that takes the shared parts has its own of each.
    let parts = [(job.article, motors), (gearbox, motors), (drive, motors), (job.axis.as_str(), 1.0)]
        .into_iter()
        .filter(|(name, _)| !name.is_empty())
        .map(|(name, count)| (name.to_string(), count))
        .collect();
    // An unknown mass counts 0 (not NaN, which sorted the row last); `estimated` says so ("gearbox-mass").
    let gearbox_mass = if gearbox.is_empty() { 0.0 } else { gearbox_mass(gearbox).unwrap_or(0.0) };
    let mass = motor_mass_for(cfg, job.article, supply).unwrap_or(f64::INFINITY) + gearbox_mass;
    let mut candidate = Candidate {
        article: job.article,
        gearbox: job.gearbox.clone(),
        drive: job.drive,
        supply: SUPPLIES.iter().copied().find(|s| *s == supply).unwrap_or(""),
        axis: job.axis.clone(),
        motor: job.motor.clone(),
        mass,
        cost: motors * mass + job.axis_weight,
        parts,
        ..Candidate::default()
    };
    // Every duty of the mix, with the same parts; the first is the config searched on.
    let mut runs: Vec<(f64, CycleLoad, Vec<Verdict>)> = Vec::new();
    let others =
        duties.iter().skip(1).map(|(base, weight)| (job_config(entry, base, supply, job).map(|c| c.0), *weight));
    for (duty, weight) in std::iter::once((Ok(with_parts.clone()), duties.first().map_or(1.0, |d| d.1))).chain(others) {
        match duty.map_err(|_| "config").and_then(|duty| judge(entry, &duty)) {
            Ok((load, v)) => runs.push((weight, load, v)),
            Err(reason) => {
                candidate.refused = reason;
                return Some(candidate);
            }
        }
    }
    let mut estimated = estimated_data(job.article, supply, job.drive);
    estimated.extend(estimated_derating(&with_parts, job.drive));
    if !gearbox.is_empty() {
        estimated.extend(estimated_gearbox_data(gearbox));
    }
    if !job.axis.is_empty() && axis_stroke_is_module_length(&job.axis) {
        estimated.push("axis-stroke");
    }
    let Ok(figures) = crate::search::figures(entry, &with_parts, &job.motor, &runs, estimated) else {
        candidate.refused = "config";
        return Some(candidate);
    };
    candidate.figures = Some(figures);
    if runs.len() == 1 {
        let (_, load, v) = runs.pop()?;
        candidate.result = Some((load, v));
    } else {
        let mixed = crate::search::mix_verdicts(&with_parts, &runs).ok()?;
        candidate.duties = runs.iter().map(|(w, load, _)| (*w, load.clone())).collect();
        candidate.result = Some((runs.swap_remove(0).1, mixed));
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
/// above 1), each motor is tried with every fitting gearbox of that series in place of the config's; a motor that none
/// fits is tried with the config's own gearbox (`gearbox` "": before, it was left out without a word).
///
/// Every candidate is simulated (`run_jobs`).
pub fn search_motors(
    entry: &MachineEntry,
    cfg: &Config,
    supply: &str,
    wanted: &dyn Fn(&str) -> bool,
    gearboxes: Option<&str>,
) -> Vec<Candidate> {
    search_motors_with(entry, cfg, supply, wanted, &Options { gearboxes, ..Options::default() })
}

fn search_motors_with(
    entry: &MachineEntry,
    cfg: &Config,
    supply: &str,
    wanted: &dyn Fn(&str) -> bool,
    options: &Options,
) -> Vec<Candidate> {
    let own_axis = Parts::of(cfg).ok().map(|p| p.axis.to_string()).unwrap_or_default();
    let weight = axis_data(&own_axis).map_or(0.0, |axis| axis_weight(cfg, &axis));
    let jobs = motor_jobs(entry, cfg, supply, wanted, options, &own_axis, weight, "");
    run_jobs(entry, (cfg, options.duties), supply, jobs, Vec::new())
}

/// The motor search on every axis of the catalogue that is of the config's kind (`axis_names`: the EXCT sizes and
/// Bahr's X/Z gantries for a config with a Z stroke, Bahr's single axes for one with one stroke), keeping the
/// config's cycle and payload. An axis that cannot carry the machine by its catalogue numbers (`axis_refusal`:
/// stroke, payload, speed) is listed once, with `article` "" and the reason in `refused`, and no motor is tried on
/// it. The rest are ranked together, cheapest first: the axis's weight counts into `cost`. A screw, rack or linear
/// motor machine is tried on the mechanical axes as well (`mech_axes`: every ball screw, with the guide sets where the
/// config rates guides), a linear motor machine with the linear motors of the supply in place of rotary motors.
pub fn search_axes(
    entry: &MachineEntry,
    cfg: &Config,
    supply: &str,
    wanted: &dyn Fn(&str) -> bool,
    gearboxes: Option<&str>,
) -> Vec<Candidate> {
    search_axes_with(entry, cfg, supply, wanted, &Options { gearboxes, ..Options::default() })
}

fn search_axes_with(
    entry: &MachineEntry,
    cfg: &Config,
    supply: &str,
    wanted: &dyn Fn(&str) -> bool,
    options: &Options,
) -> Vec<Candidate> {
    let (mut jobs, mut refused) = (Vec::new(), Vec::new());
    let belts = axis_names().into_iter().filter_map(|name| {
        let axis = axis_data(&name)?;
        let reason = axis_refusal(cfg, &axis);
        (reason != "axis-kind").then(|| (name, reason, axis_weight(cfg, &axis)))
    });
    let all: Vec<(String, &'static str, f64)> = belts.chain(mech_axes(cfg)).collect();
    for (name, reason, weight) in all {
        if !reason.is_empty() {
            refused.push(Candidate {
                axis: name,
                mass: f64::NAN,
                cost: f64::NAN,
                refused: reason,
                ..Candidate::default()
            });
            continue;
        }
        let mut with_axis = cfg.clone();
        if with_axis.set_text(&format!("parts.axis = {name:?}")).is_err() {
            continue; // a config without `[parts]` names no part
        }
        jobs.extend(motor_jobs(entry, &with_axis, supply, wanted, options, &name, weight, ""));
    }
    run_jobs(entry, (cfg, options.duties), supply, jobs, refused)
}

/// The search at several supplies (`SUPPLIES`; `vendor_of` tells a supply's maker), ranked together (`rank`): with `motor`
/// a machine's one motor of that id (`search_motor`), else with `axes` every axis (`search_axes`), else the config's
/// (`search_motors`). Each candidate says its supply, and with it its maker (`vendor_of`). An axis refused before any
/// motor was tried is listed once.
pub fn search_supplies(
    entry: &MachineEntry,
    cfg: &Config,
    supplies: &[&str],
    wanted: &dyn Fn(&str) -> bool,
    gearboxes: Option<&str>,
    axes: bool,
    motor: Option<&str>,
) -> Result<Vec<Candidate>, ConfigError> {
    search(entry, cfg, supplies, wanted, &Options { gearboxes, axes, motor, ..Options::default() })
}

/// `search_supplies` with every option of `Options`: the drives beside the paired one, and a duty mix. With a mix,
/// `cfg` is the first duty's config (`Options::duties`).
pub fn search(
    entry: &MachineEntry,
    cfg: &Config,
    supplies: &[&str],
    wanted: &dyn Fn(&str) -> bool,
    options: &Options,
) -> Result<Vec<Candidate>, ConfigError> {
    let mut all = Vec::new();
    for supply in supplies {
        let found = match options.motor {
            Some(id) => search_motor_with(entry, cfg, supply, wanted, options, id)?,
            None if options.axes => search_axes_with(entry, cfg, supply, wanted, options),
            None => search_motors_with(entry, cfg, supply, wanted, options),
        };
        let refused_before = |c: &Candidate| c.article.is_empty() && all.iter().any(|a: &Candidate| a.axis == c.axis);
        let new: Vec<Candidate> = found.into_iter().filter(|c| !refused_before(c)).collect();
        all.extend(new);
    }
    rank(&mut all);
    Ok(all)
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
    search_motor_with(entry, cfg, supply, wanted, &Options { gearboxes, ..Options::default() }, motor)
}

fn search_motor_with(
    entry: &MachineEntry,
    cfg: &Config,
    supply: &str,
    wanted: &dyn Fn(&str) -> bool,
    options: &Options,
    motor: &str,
) -> Result<Vec<Candidate>, ConfigError> {
    if !(entry.build)(cfg)?.motors().iter().any(|m| m.id == motor) {
        return Err(ConfigError::Invalid(format!("machine.motor[{motor}] (no motor of the machine has this id)")));
    }
    parts_applied(&motor_config(cfg, motor)?)?;
    let jobs = motor_jobs(entry, cfg, supply, wanted, options, "", 0.0, motor);
    Ok(run_jobs(entry, (cfg, options.duties), supply, jobs, Vec::new()))
}

/// The config with the catalogue motor `article` at `supply`, its drive `drive` and the gearbox `gearbox` ("" each: the
/// motor's own) as motor `motor`'s own parts, `[parts.<motor>]` (`motor_config` puts them in), and that motor's velocity
/// gain `drive.<motor>.velocity_kp_nm_s_rad` retuned for them as `with_named` retunes the shared one: by the square root
/// of the ratio of the inertia that motor turns (`inertia_seen`, its column alone) times that of its rotor's, and its
/// `kv_1_s` lowered where its position cascade needs it (`cascade`).
pub fn with_motor_part(
    entry: &MachineEntry,
    cfg: &Config,
    motor: &str,
    article: &str,
    supply: &str,
    gearbox: &str,
    drive: &str,
) -> Result<Config, ConfigError> {
    let text = |t: &str| toml::Value::String(t.into());
    let mut parts = vec![("motor", text(article)), ("motor_supply", text(supply))];
    parts.extend(
        [("drive", drive), ("gearbox", gearbox)].into_iter().filter(|(_, t)| !t.is_empty()).map(|(k, t)| (k, text(t))),
    );
    let out = with_own_keys(cfg, "parts", motor, &parts)?;
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
    // The inertia motor `index` turns at the middle of the travel (`inertia_turned`).
    let seen = |cfg: &Config| -> Result<f64, ConfigError> { Ok(inertia_turned(entry, cfg)?[index]) };
    let ratio = seen(&out)? / seen(cfg)? * rotor(&out)?.0 / rotor(cfg)?.0;
    let gain = motor_config(cfg, motor)?.positive("drive.velocity_kp_nm_s_rad")? * ratio.sqrt();
    let out = with_own_keys(&out, "drive", motor, &[("velocity_kp_nm_s_rad", toml::Value::Float(gain))])?;
    with_cascade(entry, cfg, &out, index, motor)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_compiled_in_tables_pass_the_check_of_a_catalogue_directory() {
        // Shows the check accepts what the code was written against, and finds a short row and a missing column.
        for &(name, text) in TABLES {
            check_table(name, text, text).unwrap_or_else(|e| panic!("{name}: {e:?}"));
        }
        let (name, text) = TABLES[0];
        let short = format!("{text}\nx,y\n");
        assert!(check_table(name, &short, text).is_err());
        // Blank lines, at the end as an editor leaves them or between rows, and Windows line ends are no error.
        let blank = format!("{}\n\n", text.replace("\r\n", "\n").replace('\n', "\r\n\r\n"));
        assert!(check_table(name, &blank, text).is_ok());
        let renamed = text.replacen("part_number", "part_no", 1);
        assert!(check_table(name, &renamed, text).is_err());
    }

    #[test]
    fn the_s120_drives_include_the_double_motor_modules_with_their_per_axis_currents() {
        // Catalogue D 21.4 N prints the 2 x 9 A Double Motor Module 6SL3120-2TE21-0AD0 with 9 A rated and 27 A maximum
        // per axis (the Single's 9 A / 27 A), and the 2 x 18 A, type C, 6SL3120-2TE21-8AC0 with 18 A / 36 A.
        let drives = drives_for("s120");
        let find = |p: &str| drives.iter().find(|d| d.0 == p).copied().unwrap();
        assert_eq!(find("6SL3120-2TE21-0AD0"), ("6SL3120-2TE21-0AD0", 9.0, 27.0));
        assert_eq!(find("6SL3120-2TE21-8AC0"), ("6SL3120-2TE21-8AC0", 18.0, 36.0));
        assert_eq!(drives.iter().filter(|d| d.0.starts_with("6SL3120-2")).count(), 5);
    }

    #[test]
    fn a_guide_set_has_four_carriages_unless_the_config_says_and_a_rail_length_sets_the_weight() {
        // Hand, HGH15CA (0.18 kg a carriage, 1.45 kg/m of rail, 0.0614 m long): rails of the stroke 0.3 m and two
        // carriages, 2 x 1.45 x (0.3 + 0.1228) + 4 x 0.18 = 1.94612 kg; rails of 0.5 m, 2 x 1.45 x 0.5 + 0.72 = 2.17 kg.
        let row = rows(table("hiwin/linear_guides.csv")).into_iter().find(|r| r["part_number"] == "HGH15CA").unwrap();
        let cfg =
            Config::parse("format_version = 1\n[ratings]\nguide_max_load_n = 1.0\n[slide]\nstroke_m = 0.3\n").unwrap();
        assert_eq!(guide_carriages(&cfg), Ok(4.0));
        assert!((mechanics::guide_weight(&cfg, &row, 4.0) - 1.94612).abs() < 1e-12);
        let cfg = Config::parse(
            "format_version = 1\n[ratings]\nguide_max_load_n = 1.0\nguide_carriages = 2.0\nguide_rail_length_m = 0.5\n\
             [slide]\nstroke_m = 0.3\n",
        )
        .unwrap();
        assert_eq!(guide_carriages(&cfg), Ok(2.0));
        assert!((mechanics::guide_weight(&cfg, &row, 4.0) - 2.17).abs() < 1e-12);
        for bad in ["2.5", "0.0", "-4.0", "\"four\""] {
            let cfg = Config::parse(&format!(
                "format_version = 1
[ratings]
guide_carriages = {bad}
"
            ))
            .unwrap();
            assert!(guide_carriages(&cfg).is_err(), "{bad}");
        }
    }

    #[test]
    fn a_named_guide_set_puts_its_carriages_mass_and_seal_drag_in_place_of_the_configs() {
        // HGH15CA: 0.18 kg and 4.7 N of seal drag a carriage, four of them 0.72 kg and 18.8 N. Without the share keys
        // the config's carriage (4 kg) and friction (15 N sliding, 20 N breakaway) hold no guide carriages: 4.72 kg,
        // 33.8 N, 38.8 N. With 0.5 kg and 2 N of them its own: 4.22 kg, 31.8 N, 36.8 N, and the shares become the new.
        let base = "format_version = 1\n[ratings]\nguide_max_load_n = 1.0\nguide_dynamic_load_rating_n = 1.0\n\
                    [mass]\ncarriage_kg = 4.0\n";
        let friction = "[friction_x]\ncoulomb_n = 15.0\nstiction_n = 20.0\n";
        let named = |text: &str| {
            let mut cfg = Config::parse(text).unwrap();
            mechanics::apply_mech_axis(&mut cfg, "HGH15CAx4").unwrap().unwrap();
            cfg
        };
        let levels = |cfg: &Config| {
            ["mass.carriage_kg", "friction_x.coulomb_n", "friction_x.stiction_n"].map(|k| cfg.f64(k).unwrap())
        };
        let close = |a: [f64; 3], b: [f64; 3]| a.iter().zip(&b).all(|(x, y)| (x - y).abs() < 1e-12);
        assert!(close(levels(&named(&format!("{base}{friction}"))), [4.72, 33.8, 38.8]));
        let cfg = named(&format!("{base}guide_carriages_kg = 0.5\n{friction}seal_drag_n = 2.0\n"));
        assert!(close(levels(&cfg), [4.22, 31.8, 36.8]));
        assert!((cfg.f64("mass.guide_carriages_kg").unwrap() - 0.72).abs() < 1e-12);
        assert!((cfg.f64("friction_x.seal_drag_n").unwrap() - 18.8).abs() < 1e-12);
        // C for 50 km 14700 N and C0 23470 N a carriage: 58800 N and 4 x 23470 / 2 = 46940 N.
        assert_eq!(cfg.f64("ratings.guide_dynamic_load_rating_n").unwrap(), 58800.0);
        assert_eq!(cfg.f64("ratings.guide_max_load_n").unwrap(), 46940.0);
    }

    #[test]
    fn a_single_phase_supply_is_told_by_its_text_and_no_makers_line_is_single_phase() {
        assert!(single_phase("1 AC 200 ... 240 V") && single_phase("1 x 230 V") && single_phase("Single-phase 230 V"));
        assert!(!single_phase("1 or 3 AC 100 ... 240 V") && !single_phase("3 AC 240 V") && !single_phase("3x 200 VAC"));
        // Every maker's line has a three-phase drive (the AKD's "1 or 3 AC" counts as three): 1.35 times the line.
        for (vendor, line) in SUPPLIES.iter().filter_map(|s| vendor_supply(s)) {
            assert!(!single_phase_line(vendor, line), "{vendor} {line}");
        }
    }

    #[test]
    fn a_missing_rated_current_is_the_rated_torque_over_the_torque_constant() {
        // Kollmorgen prints no rated current at 400 V: In = M_N * I0 / M0, below I0 for a motor whose rated torque is below M0.
        let row = vendor_rows("motors_", "kollmorgen", 400.0)
            .into_iter()
            .find(|r| number(r, "rated_current_a").is_none() && number(r, "rated_torque_nm").is_some())
            .expect("a row without rated current");
        let m = motor_data(row["part_number"], "kollmorgen-400v").unwrap();
        assert!((m.rated_current - m.rated_torque * m.stall_current / m.static_torque).abs() < 1e-12);
        assert!(m.rated_current < m.stall_current);
    }

    #[test]
    fn the_voltage_constant_is_sqrt3_times_the_printed_one_unless_it_is_a_misprint() {
        // 0.5 V s/rad line to line: sqrt(3) * 0.5 = 0.866, however far that is from kt (a genuine row may be 1.3 times it).
        assert!((emf_constant(Some(0.5), "AM8051-0J", 400.0).unwrap() - 0.866_025_4).abs() < 1e-6);
        // The two documented misprints are left out, the AKM64Q only at 240 V.
        assert_eq!(emf_constant(Some(0.0611), "AKM64Q", 240.0), None);
        assert!(emf_constant(Some(0.0611), "AKM64Q", 400.0).is_some());
        assert_eq!(emf_constant(Some(0.468), "1FT7044-1AF7", 0.0), None);
    }

    #[test]
    fn rated_values_are_paired_with_the_speed_they_hold_at() {
        // 1FT7044-5AK7: 3.0 N m at 471.24 rad/s (4500 r/min); the table's rated speed of 628 rad/s is another point.
        let m = motor_data("1FT7044-5AK7", "s120").unwrap();
        assert!((m.rated_speed - 4500.0 * PI / 30.0).abs() < 1e-6);
    }

    #[test]
    fn a_misprinted_rated_speed_is_rated_power_over_rated_torque() {
        // 1FT2105-4AH: 1740 W / 3.7 N m = 470.3 rad/s (4491 r/min), not the printed 3000 r/min.
        let m = motor_data("1FT2105-4AH", "s120").unwrap();
        assert!((m.rated_speed - 1740.0 / 3.7).abs() < 1e-9);
    }

    #[test]
    fn a_gearbox_without_mass_or_efficiency_is_named_as_estimated_not_ranked_by_nan() {
        let name = |series: &str| {
            mechanics::all_gearboxes()
                .into_iter()
                .find(|g| g.row["series"] == series)
                .map(|g| mechanics::gearbox_name(&g.row))
        };
        // The Apex PE II prints no mass; a Harmonic Drive CSF is an estimate unless its row has an `efficiency`.
        assert_eq!(estimated_gearbox_data(&name("PEII").unwrap()), ["gearbox-mass"]);
        let csf = mechanics::all_gearboxes().into_iter().find(|g| g.row["series"] == "CSF").unwrap();
        let flagged = estimated_gearbox_data(&mechanics::gearbox_name(&csf.row)).contains(&"gearbox-efficiency");
        assert_eq!(flagged, printed_efficiency(&csf.row).is_none());
    }

    #[test]
    fn a_strain_wave_gears_efficiency_column_is_used_and_the_estimate_stays_for_rows_without() {
        // `efficiency` (read from the maker's charts) counts as printed; a row with neither column has the 0.70 estimate.
        let with: Row = Box::leak(Box::new(HashMap::from([("series", "CSF"), ("efficiency", "0.83")])));
        let without: Row = Box::leak(Box::new(HashMap::from([("series", "CSF")])));
        assert_eq!(printed_efficiency(&with), Some(0.83));
        assert!(printed_efficiency(&without).is_none() && strain_wave(&without));
    }

    /// A config with what `apply_motor` and `apply_gearbox` read and a pulley rating, and the first Kollmorgen motor with
    /// a voltage constant and the first gearbox with an efficiency and a stiffness.
    fn pulley_config(pulley_max: f64) -> Config {
        Config::parse(&format!(
            "format_version = 1\n[motor]\nrotor_inertia_kgm2 = 1e-4\ntorque_constant_nm_a = 0.5\nmax_current_a = 10.0\nrated_current_a = 5.0\n\
             torque_constant_at_max_current_nm_a = 0.5\ninductance_h = 1e-3\nresistance_ohm = 1.0\n\
             thermal_time_constant_s = 600.0\n\
             [drive]\ncurrent_kp_v_a = 5.0\ncurrent_tn_s = 1e-3\npeak_torque_nm = 1.0\nrated_torque_nm = 1.0\n\
             rated_speed_rad_s = 100.0\nmax_speed_rad_s = 200.0\nbus_voltage_v = 540.0\nno_load_speed_rad_s = 300.0\n\
             back_emf_constant_v_s_rad = 0.0\n[gearbox]\nratio = 1.0\nefficiency = 1.0\n\
             stiffness_nm_rad = 1e4\nbacklash_rad = 0.0\ninertia_at_input_kgm2 = 0.0\n\
             [ratings]\npulley_max_torque_nm = {pulley_max}\n"
        ))
        .unwrap()
    }

    fn kollmorgen_motor() -> &'static str {
        vendor_rows("motors_", "kollmorgen", 400.0)
            .into_iter()
            .find(|r| number(r, "voltage_constant_v_s_rad").is_some() && r["part_number"] != "AKM64Q")
            .map(|r| r["part_number"])
            .expect("a motor with a voltage constant")
    }

    #[test]
    fn the_peak_torque_cap_does_not_depend_on_the_order_of_motor_and_gearbox() {
        let gearbox = mechanics::all_gearboxes()
            .into_iter()
            .find(|g| {
                number(&g.row, "efficiency_full_load").is_some()
                    && number(&g.row, "torsional_stiffness_nm_rad").is_some()
                    && number(&g.row, "rated_output_torque_nm").is_some()
                    && number(&g.row, "ratio").is_some_and(|r| r > 2.0)
            })
            .map(|g| mechanics::gearbox_name(&g.row))
            .unwrap();
        let g = mechanics::find_gearbox(&gearbox).unwrap().0;
        let (ratio, efficiency) = (number(&g.row, "ratio").unwrap(), number(&g.row, "efficiency_full_load").unwrap());
        let motor = kollmorgen_motor();
        let data = motor_data(motor, "kollmorgen-400v").unwrap();
        // The pulley takes half the motor's peak at the pulley: allowed = pulley * eta / i = 0.5 * peak, either order.
        let mut gearbox_first = pulley_config(0.5 * data.max_torque * ratio / efficiency);
        let mut motor_first = gearbox_first.clone();
        apply_gearbox(&mut gearbox_first, &gearbox).unwrap();
        apply_motor(&mut gearbox_first, motor, "kollmorgen-400v").unwrap();
        apply_motor(&mut motor_first, motor, "kollmorgen-400v").unwrap();
        apply_gearbox(&mut motor_first, &gearbox).unwrap();
        for cfg in [&gearbox_first, &motor_first] {
            assert!((cfg.f64("drive.peak_torque_nm").unwrap() - 0.5 * data.max_torque).abs() < 1e-9 * data.max_torque);
        }
    }

    #[test]
    fn a_config_peak_below_the_motors_is_not_raised_by_a_gearbox_applied_after_the_motor() {
        let gearbox = mechanics::all_gearboxes()
            .into_iter()
            .find(|g| {
                number(&g.row, "ratio").is_some_and(|r| r > 2.0)
                    && number(&g.row, "rated_output_torque_nm").is_some()
                    && number(&g.row, "torsional_stiffness_nm_rad").is_some()
            })
            .map(|g| mechanics::gearbox_name(&g.row))
            .unwrap();
        let g = mechanics::find_gearbox(&gearbox).unwrap().0;
        let (ratio, efficiency) = (number(&g.row, "ratio").unwrap(), printed_efficiency(&g.row).unwrap_or(1.0));
        let motor = kollmorgen_motor();
        let own = motor_data(motor, "kollmorgen-400v").unwrap().max_torque;
        let lowered = |pulley: f64| {
            let mut cfg = pulley_config(pulley);
            apply_motor(&mut cfg, motor, "kollmorgen-400v").unwrap();
            cfg.set_text(&format!("drive.peak_torque_nm = {:?}", 0.1 * own)).unwrap();
            apply_gearbox(&mut cfg, &gearbox).unwrap();
            cfg.f64("drive.peak_torque_nm").unwrap()
        };
        // A loose pulley: the config's peak, a tenth of the motor's, stays; a pulley that allows a twentieth through
        // the gearbox lowers it to that: the lower of the config's value and the cap.
        assert_eq!(lowered(1e9), 0.1 * own);
        assert!((lowered(0.05 * own * ratio / efficiency) - 0.05 * own).abs() < 1e-9 * own);
    }

    #[test]
    fn a_printed_voltage_constant_becomes_the_models_back_emf_constant() {
        // The motor's printed line-to-line RMS constant times sqrt(3), and the no-load speed sqrt(3/2) bus over it, for a
        // Kollmorgen AKM and a Beckhoff AM8000 (whose constant is up to 1.30 times its torque constant).
        let beckhoff = vendor_rows("motors_", "beckhoff", 400.0)
            .into_iter()
            .find(|r| r["part_number"].starts_with("AM80") && number(r, "voltage_constant_v_s_rad").is_some())
            .map(|r| r["part_number"])
            .unwrap();
        for (vendor, motor) in [("kollmorgen", kollmorgen_motor()), ("beckhoff", beckhoff)] {
            let printed = vendor_rows("motors_", vendor, 400.0)
                .into_iter()
                .find(|r| r["part_number"] == motor)
                .and_then(|r| number(&r, "voltage_constant_v_s_rad"))
                .unwrap();
            let mut cfg = pulley_config(1e9);
            apply_motor(&mut cfg, motor, &format!("{vendor}-400v")).unwrap();
            let ke = cfg.f64("drive.back_emf_constant_v_s_rad").unwrap();
            assert!((ke - 3f64.sqrt() * printed).abs() < 1e-12, "{motor}: {ke} for {printed}");
            let (bus, rated) = (cfg.f64("drive.bus_voltage_v").unwrap(), cfg.f64("drive.rated_speed_rad_s").unwrap());
            let no_load = (crate::drive::WINDING_PER_DC_LINK * bus / ke).max(1.1 * rated);
            assert_eq!(cfg.f64("drive.no_load_speed_rad_s").unwrap(), no_load, "{motor}");
            eprintln!(
                "{motor}: printed {printed} V s/rad, kE {ke}, bus {bus} V, no-load {no_load} rad/s, rated {rated}"
            );
        }
    }

    #[test]
    fn a_belt_axis_stroke_is_the_module_length_less_the_length_that_is_not_stroke() {
        // Rexroth CKR-110: L = 5.5 m = s_max + Lca 170 mm + Lad 25 mm, so s_max = 5.305 m.
        assert!((axis_data("CKR-110").unwrap().stroke_x.1 - 5.305).abs() < 1e-9);
        // Bahr ELZ-60: basic length 290 mm + travel = 6 m total, travel 5.71 m.
        assert!((axis_data("ELZ-60").unwrap().stroke_x.1 - 5.71).abs() < 1e-9);
        // An ELSZ has no basic length read directly: the module length stays and the row is flagged.
        assert!((axis_data("ELSZ-60").unwrap().stroke_x.1 - 6.0).abs() < 1e-9);
        assert!(axis_stroke_is_module_length("ELSZ-60") && !axis_stroke_is_module_length("ELZ-60"));
        assert!(!axis_stroke_is_module_length("CKR-110"));
    }

    #[test]
    fn a_retuned_gain_keeps_the_crossover_over_the_rotor_side_inertia() {
        // A heavier load on the same rotor keeps the config's gain; a rotor 4 times as heavy allows 4 times the gain,
        // here more than the scaled one; a rotor half as heavy half the gain, here less than the scaled one.
        assert_eq!(bounded_gain(0.0573, 0.025, 1.0, true), 0.025);
        assert_eq!(bounded_gain(0.0646, 0.025, 4.0, true), 0.0646);
        assert_eq!(bounded_gain(0.016, 0.025, 0.5, true), 0.0125);
        // A gain that shrinks below the bound, and any gain of a rigid model, stands.
        assert_eq!(bounded_gain(0.016, 0.025, 1.0, true), 0.016);
        assert_eq!(bounded_gain(0.0573, 0.025, 1.0, false), 0.0573);
    }
}
