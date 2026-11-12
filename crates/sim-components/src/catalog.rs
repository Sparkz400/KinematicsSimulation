//! The component catalogue as the simulator uses it: a part named by its article number puts its
//! catalogue values into a config, and a search tries every motor on a machine's cycle.
//!
//! The data files of `catalog/` are compiled in, so a front end needs no path to them.

use crate::analysis::{CycleLoad, Verdict, analyse_cycle, verdicts};
use crate::sim::Sim;
use sim_core::{Config, ConfigError, MachineEntry};
use std::collections::HashMap;
use std::f64::consts::PI;

const MOTORS_S210: &str = include_str!("../../../catalog/siemens/motors_s210.csv");
/// Thermal time constants come from the configuration manuals, which cover most of the same motors.
const MOTOR_MANUALS: [&str; 2] =
    [include_str!("../../../catalog/siemens/motors_1fk2_s120.csv"), include_str!("../../../catalog/siemens/motors_1ft2_s120.csv")];
const GEARBOXES: [&str; 3] = [
    include_str!("../../../catalog/siemens/gearbox_nrb.csv"),
    include_str!("../../../catalog/siemens/gearbox_nrk.csv"),
    include_str!("../../../catalog/siemens/gearbox_nlc.csv"),
];

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

/// `what` is "motor" or "gearbox": the kind of part, which the message names in the user's language.
fn unknown(what: &str, name: &str) -> ConfigError {
    ConfigError::Part(if what == "motor" { "error-config-part-motor" } else { "error-config-part-gearbox" }, name.into())
}

fn set(cfg: &mut Config, path: &str, value: f64) -> Result<(), ConfigError> {
    cfg.set_text(&format!("{path} = {value:?}"))
}

/// Article numbers of the SINAMICS S210 motors, e.g. "1FK2205-4AF".
pub fn motor_articles() -> Vec<&'static str> {
    rows(MOTORS_S210).iter().map(|r| r["article"]).collect()
}

/// Weight of a motor without brake, as a stand-in for its price when candidates are ranked.
pub fn motor_mass(article: &str) -> Option<f64> {
    number(rows(MOTORS_S210).iter().find(|r| r["article"] == article)?, "mass_kg")
}

/// Puts a motor of the S210 catalogue into a config, for the supply "230v" or "400v": rotor inertia,
/// torque constant and saturation, rated current, thermal time constant, and the drive's torque and speed
/// limits. The peak torque stays within what the machine's pulley allows, where the config rates that.
///
/// The bus voltage becomes the DC link voltage of that supply, because the catalogue's rated speed and
/// torque hold on it: 540 V on a 400 V three-phase line (1.35 times the line voltage, the figure the
/// Siemens line module data in `catalog/siemens/line_modules_s120.csv` is given for) and 325 V on a
/// 230 V line (its peak, 230 V times the square root of 2). A config written for another bus would
/// otherwise run the motor against a back-EMF it was never meant to meet. The braking threshold and the
/// trip level of the config's DC link are scaled with the bus voltage: left as they were, those of a
/// config written for a lower bus would lie below the new one.
///
/// Winding resistance and inductance stay as the config has them: Siemens does not publish them for
/// these motors. The current controller's gains follow from those two alone (Kp = L / T, Tn = L / R),
/// not from the bus voltage, so they stay as well. The velocity gain stays here too; `resolve` and
/// `with_parts` retune it for the new rotor.
pub fn apply_motor(cfg: &mut Config, article: &str, supply: &str) -> Result<(), ConfigError> {
    let all = rows(MOTORS_S210);
    let motor = all.iter().find(|r| r["article"] == article).ok_or_else(|| unknown("motor", article))?;
    let get = |column: &str| number(motor, column).ok_or_else(|| unknown("motor", &format!("{article}, {supply}")));
    let rated = |quantity: &str, unit: &str| get(&format!("rated_{quantity}_{supply}_{unit}"));
    let (static_torque, stall_current, max_torque, max_current) = (get("static_torque_nm")?, get("stall_current_a")?, get("max_torque_nm")?, get("max_current_a")?);
    let kt = static_torque / stall_current;
    let rated_speed = rated("speed", "rpm")? * PI / 30.0;
    set(cfg, "motor.rotor_inertia_kgm2", get("rotor_inertia_kgcm2")? * 1e-4)?;
    set(cfg, "motor.torque_constant_nm_a", kt)?;
    set(cfg, "motor.max_current_a", max_current)?;
    // The catalogue's own rated current, up to 18 % above rated torque over the static torque constant.
    // One row prints a rated current far above the stall current (see the file's header); no other motor's
    // is more than 5 % above it, so that is where it stops.
    set(cfg, "motor.rated_current_a", rated("current", "a")?.min(1.05 * stall_current))?;
    set(cfg, "motor.torque_constant_at_max_current_nm_a", (max_torque / max_current).min(kt))?;
    let manual = MOTOR_MANUALS.iter().flat_map(|text| rows(text)).find(|r| r["article"] == article);
    if let Some(minutes) = manual.as_ref().and_then(|r| number(r, "thermal_time_constant_min")) {
        set(cfg, "motor.thermal_time_constant_s", minutes * 60.0)?;
    }
    let ratio = cfg.f64("gearbox.ratio").unwrap_or(1.0);
    let allowed = cfg.f64("ratings.pulley_max_torque_nm").map_or(f64::INFINITY, |at_pulley| at_pulley / ratio);
    set(cfg, "drive.peak_torque_nm", max_torque.min(allowed))?;
    set(cfg, "drive.rated_torque_nm", rated("torque", "nm")?)?;
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

/// Puts a gearbox of the catalogue into a config. The name is size, motor frame size and ratio, e.g.
/// "NRK070-04-5". Sets ratio, efficiency, torsional stiffness, backlash and the gearbox's own inertia,
/// and sets the torque ratings where the config has them.
pub fn apply_gearbox(cfg: &mut Config, name: &str) -> Result<(), ConfigError> {
    let parts: Vec<&str> = name.split('-').collect();
    let all: Vec<Row> = GEARBOXES.iter().flat_map(|text| rows(text)).collect();
    let gearbox = all
        .iter()
        .find(|r| parts.len() == 3 && r["size"] == parts[0] && r["motor_frame"] == parts[1] && r["ratio"] == parts[2])
        .ok_or_else(|| unknown("gearbox", name))?;
    let get = |column: &str| number(gearbox, column);
    let need = |column: &str| get(column).ok_or_else(|| unknown("gearbox", name));
    let arcmin = PI / 180.0 / 60.0;
    set(cfg, "gearbox.ratio", need("ratio")?)?;
    set(cfg, "gearbox.efficiency", need("efficiency_full_load")?)?;
    set(cfg, "gearbox.stiffness_nm_rad", need("torsional_stiffness_nm_arcmin")? / arcmin)?;
    // The manual leaves backlash and inertia open for a few gearboxes; the config's values stay then.
    if let Some(backlash) = get("backlash_arcmin") {
        set(cfg, "gearbox.backlash_rad", backlash * arcmin)?;
    }
    // Kept apart from the rotor's: added into it, the next motor put into the config would wipe it out.
    if let Some(inertia) = get("inertia_at_input_kgcm2") {
        set(cfg, "gearbox.inertia_at_input_kgm2", inertia * 1e-4)?;
    }
    for (rating, column) in [("gearbox_rated_torque_nm", "rated_torque_nm"), ("gearbox_max_torque_nm", "max_torque_nm")] {
        if cfg.f64(&format!("ratings.{rating}")).is_ok() {
            set(cfg, &format!("ratings.{rating}"), need(column)?)?;
        }
    }
    Ok(())
}

/// A part the config names in `[parts]`; "" where it names none or has no such table. A name that is not
/// text (`motor = 5`) is refused: taken as no name, the config would run with other parts than it says.
fn part<'a>(cfg: &'a Config, key: &str) -> Result<&'a str, ConfigError> {
    let path = format!("parts.{key}");
    cfg.get(&path).map_or(Ok(""), |v| v.as_str().ok_or(ConfigError::Type(path.clone())))
}

/// Puts a gearbox and a motor into a config, "" for none. The gearbox first: the motor's peak torque is
/// held within what the pulley allows through the ratio in effect. The names in `[parts]` are emptied, so
/// that doing it again changes nothing.
fn apply_parts(cfg: &mut Config, motor: &str, supply: &str, gearbox: &str) -> Result<(), ConfigError> {
    if !gearbox.is_empty() {
        apply_gearbox(cfg, gearbox)?;
    }
    if !motor.is_empty() {
        apply_motor(cfg, motor, supply)?;
    }
    for key in ["motor", "gearbox"] {
        let _ = cfg.set_text(&format!("parts.{key} = \"\""));
    }
    Ok(())
}

fn supply(cfg: &Config) -> Result<&str, ConfigError> {
    Ok(match part(cfg, "motor_supply")? {
        "" => "400v",
        supply => supply,
    })
}

/// The config with the catalogue values of the parts it names, and nothing retuned: for the functions
/// that read ratings and limits from a config and have no machine to retune for.
pub(crate) fn parts_applied(cfg: &Config) -> Result<Config, ConfigError> {
    let mut out = cfg.clone();
    apply_parts(&mut out, part(cfg, "motor")?, supply(cfg)?, part(cfg, "gearbox")?)?;
    Ok(out)
}

/// The config with the catalogue values of the parts it names: `[parts]` with `motor = "<article>"`,
/// `motor_supply = "230v"` or `"400v"` and `gearbox = "<size-frame-ratio>"`, "" for none. A named part
/// overwrites the numbers the config has for it, and the velocity gain is retuned for it (see
/// `with_parts`). `Sim` does this with every config it is given, so every front end gets it; the functions
/// that read a config beside a `Sim` do it too.
pub fn resolve(entry: &MachineEntry, cfg: &Config) -> Result<Config, ConfigError> {
    with_named(entry, cfg, part(cfg, "motor")?, supply(cfg)?, part(cfg, "gearbox")?)
}

/// One motor, with or without a gearbox, tried on a machine's cycle.
#[derive(Debug, Clone)]
pub struct Candidate {
    pub article: &'static str,
    /// The gearbox as `apply_gearbox` names it; "" where the config's own gearbox stayed.
    pub gearbox: String,
    /// Of the motor. The gearbox files have no mass.
    pub mass: f64,
    /// The cycle and its verdicts; `None` where the cycle was not run, with the reason in `refused`.
    pub result: Option<(CycleLoad, Vec<Verdict>)>,
    pub refused: &'static str,
}

impl Candidate {
    pub fn passes(&self) -> bool {
        self.result.as_ref().is_some_and(|(_, verdicts)| verdicts.iter().all(|v| v.ok))
    }

    /// The check closest to (or furthest beyond) its limit, with value over limit. Only checks whose
    /// value must stay below a positive limit count. Two checks of the DC link do not count while they
    /// pass, because their ratio is no measure of how close the cycle is to failing: with a braking resistor
    /// behind them the capacitors fill to their limit by design, in every cycle that brakes hard enough, and
    /// the resistor's checks say what that costs; and the voltage is never below the bus, so its ratio to
    /// the trip level starts at 0.6 or so.
    ///
    /// A run whose drives tripped is infinitely far beyond: its figures end at the trip, where the DC link
    /// was just past its limit, and by that ratio it would rank as the nearest to passing.
    pub fn tightest(&self) -> Option<(&'static str, f64)> {
        let (load, verdicts) = self.result.as_ref()?;
        if load.tripped {
            return Some(("dc-link-overvoltage", f64::INFINITY));
        }
        let resistor = |v: &Verdict| verdicts.iter().any(|w| w.motor == v.motor && w.check.starts_with("braking-resistor"));
        let by_design = |v: &Verdict| v.ok && (v.check == "dc-link-overvoltage" || v.check == "dc-link-energy" && resistor(v));
        verdicts.iter().filter(|v| v.limit > 0.0 && v.check != "belt-slack" && !by_design(v)).map(|v| (v.check, v.value / v.limit)).max_by(|a, b| a.1.total_cmp(&b.1))
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
/// config's own.
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
pub fn with_parts(entry: &MachineEntry, cfg: &Config, article: &str, supply: &str, gearbox: &str) -> Result<Config, ConfigError> {
    with_named(entry, cfg, article, supply, if gearbox.is_empty() { part(cfg, "gearbox")? } else { gearbox })
}

fn with_named(entry: &MachineEntry, cfg: &Config, motor: &str, supply: &str, gearbox: &str) -> Result<Config, ConfigError> {
    let mut out = cfg.clone();
    apply_parts(&mut out, motor, supply, gearbox)?;
    if !motor.is_empty() || !gearbox.is_empty() {
        let rotor = |cfg: &Config| Ok::<f64, ConfigError>(cfg.positive("motor.rotor_inertia_kgm2")? + cfg.non_negative("gearbox.inertia_at_input_kgm2")?);
        let ratio = inertia_seen(entry, &out)? / inertia_seen(entry, cfg)? * rotor(&out)? / rotor(cfg)?;
        let gain = cfg.positive("drive.velocity_kp_nm_s_rad")? * ratio.sqrt();
        set(&mut out, "drive.velocity_kp_nm_s_rad", gain)?;
    }
    Ok(out)
}

/// The catalogue gearboxes of a series (a prefix of the size, e.g. "NRK" or "NRK070") that fit a motor's
/// frame and whose ratio is between half and twice `ratio`, named as `apply_gearbox` takes them. The
/// frame is in the article number: 1FK2104 and 1FK2204 have frame "04".
pub fn fitting_gearboxes(article: &str, series: &str, ratio: f64) -> Vec<String> {
    let all = GEARBOXES.iter().flat_map(|text| rows(text));
    let fits = |r: &Row| r["size"].starts_with(series) && article.get(5..7) == Some(r["motor_frame"]) && number(r, "ratio").is_some_and(|n| n >= 0.5 * ratio && n <= 2.0 * ratio);
    all.filter(fits).map(|r| format!("{}-{}-{}", r["size"], r["motor_frame"], r["ratio"])).collect()
}

/// Tries every catalogue motor that `wanted` accepts and that is offered at the supply on the cycle of
/// the config, each retuned as `with_parts` says, and returns them with the passing ones first, lightest
/// first, and the others after them, nearest to passing first. The weight stands in for the price, which
/// no catalogue file has; it is the motor's, because the gearbox files have no mass.
///
/// With `gearboxes` (a series, see `fitting_gearboxes`) and a config whose machine has a gearbox (ratio
/// above 1), each motor is tried with every fitting gearbox of that series in place of the config's.
///
/// Before a cycle is simulated the catalogue numbers are checked against the axis limits: the motor speed
/// at the highest axis speed against the rated speed, and the torque of the rigid machine at the highest
/// acceleration against the motor's maximum torque (`check_limits`). A candidate that fails there is not
/// simulated; it is returned with `result` empty and the limit in `refused`.
pub fn search_motors(entry: &MachineEntry, cfg: &Config, supply: &str, wanted: &dyn Fn(&str) -> bool, gearboxes: Option<&str>) -> Vec<Candidate> {
    let ratio = parts_applied(cfg).ok().and_then(|own| own.f64("gearbox.ratio").ok()).unwrap_or(1.0);
    let mut out = Vec::new();
    for article in motor_articles().into_iter().filter(|a| wanted(a)) {
        let names = match gearboxes {
            Some(series) if ratio > 1.0 => fitting_gearboxes(article, series, ratio),
            _ => vec![String::new()],
        };
        for gearbox in names {
            let Ok(with_parts) = with_parts(entry, cfg, article, supply, &gearbox) else {
                continue; // not offered at this supply
            };
            let mut candidate = Candidate { article, gearbox, mass: motor_mass(article).unwrap_or(f64::INFINITY), result: None, refused: "" };
            match Sim::new(entry, &with_parts) {
                Err(_) => candidate.refused = "config",
                // The axis limits ask for more speed or torque than this motor has.
                Ok(sim) if !sim.limit_violations.is_empty() => candidate.refused = sim.limit_violations[0].key,
                Ok(mut sim) => match analyse_cycle(&mut sim) {
                    Err(e) => candidate.refused = e.key,
                    Ok(load) => match verdicts(&with_parts, &load) {
                        Ok(v) => candidate.result = Some((load, v)),
                        Err(_) => candidate.refused = "config",
                    },
                },
            }
            out.push(candidate);
        }
    }
    let rank = |c: &Candidate| (!c.passes(), if c.passes() { c.mass } else { c.tightest().map_or(f64::INFINITY, |t| t.1) });
    out.sort_by(|a, b| rank(a).0.cmp(&rank(b).0).then(rank(a).1.total_cmp(&rank(b).1)));
    out
}
