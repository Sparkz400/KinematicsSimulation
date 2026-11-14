//! The catalogue's mechanics beyond the belts and belt axes: every maker's gearboxes and which motor each fits (by
//! the motor's frame for the Siemens geared motors, by its shaft for the gearboxes fitted through an adapter plate),
//! the motors' shafts, ball screws, profile rail guides and linear motors, and the axes a screw, rack or linear motor
//! machine can be built from (`mech_axes`).

use super::{Row, TABLES, number, rows, set, set_if_present, table, unknown, vendor_supply};
use crate::transmission::BallScrew;
use sim_core::{Config, ConfigError};
use std::collections::HashMap;

/// A gearbox row with its maker and series: the folder and the file name after `gearbox_` ("siemens", "nrk").
pub(super) struct Gearbox {
    pub maker: &'static str,
    pub series: &'static str,
    pub row: Row,
}

/// The gearbox tables, `<maker>/gearbox_<series>.csv`, as (maker, series, path); Siemens' `gearbox_mass.csv` holds
/// masses, not gearboxes.
fn gearbox_tables() -> impl Iterator<Item = (&'static str, &'static str, &'static str)> {
    TABLES.iter().filter_map(|(name, _)| {
        let (maker, file) = name.split_once('/')?;
        let series = file.strip_prefix("gearbox_")?.strip_suffix(".csv")?;
        (series != "mass").then_some((maker, series, *name))
    })
}

/// Every gearbox of the catalogue.
pub(super) fn all_gearboxes() -> Vec<Gearbox> {
    gearbox_tables()
        .flat_map(|(maker, series, name)| rows(table(name)).into_iter().map(move |row| Gearbox { maker, series, row }))
        .collect()
}

/// The name of a gearbox as `apply_gearbox` takes it: size, motor frame (or clamping hub, see the schema) and ratio.
pub(super) fn gearbox_name(r: &Row) -> String {
    format!("{}-{}-{}", r["size"], r["motor_frame"], r["ratio"])
}

/// A gearbox by name, "<size>-<frame>-<ratio>", with "-r" after it for the maker's reduced backlash option; and
/// whether that option is asked for.
pub(super) fn find_gearbox(name: &str) -> Option<(Gearbox, bool)> {
    let (base, reduced) = name.strip_suffix("-r").map_or((name, false), |base| (base, true));
    all_gearboxes().into_iter().find(|g| gearbox_name(&g.row) == base).map(|g| (g, reduced))
}

/// The shaft diameter of a motor in m: the row of the `<maker>/motor_shafts.csv` tables with the longest
/// `part_number_prefix` the article starts with; `None` where no table has it.
pub fn motor_shaft_diameter(article: &str) -> Option<f64> {
    TABLES
        .iter()
        .filter(|(name, _)| name.ends_with("/motor_shafts.csv"))
        .flat_map(|(name, _)| rows(table(name)))
        .filter(|r| !r["part_number_prefix"].is_empty() && article.starts_with(r["part_number_prefix"]))
        .max_by_key(|r| r["part_number_prefix"].len())
        .and_then(|r| number(&r, "shaft_diameter_m"))
}

/// Whether Siemens builds one of its gearboxes (`siemens/gearbox_*.csv`) onto a motor: the NRB, NRK and NLC onto the
/// 1FK2 and 1FT2 of the frame the row names (1FK2104 and 1FT2204 have frame "04"), the SP+ and TP+ (made by
/// WITTENSTEIN alpha) onto the 1FT2 only (the equipment manual, chapter 3.1: "Coaxial precision gearbox only for
/// 1FT2"). The geared motor is what Siemens sells; another frame or a 1FK2 with an SP+ it does not.
fn siemens_fits(article: &str, g: &Gearbox) -> bool {
    let precision = matches!(g.series, "sp" | "tp");
    article.get(3..4) == Some("2")
        && article.get(5..7) == Some(g.row["motor_frame"])
        && (!precision || article.starts_with("1FT2"))
}

/// The gearboxes that fit a motor, at every ratio, as (name, ratio). `series` picks them: "" every maker's; a maker's
/// folder ("wittenstein", "neugart") all its tables; "<maker>-<series>" one table, by its file name
/// (`wittenstein-sp` is `wittenstein/gearbox_sp.csv`, `neugart-ple`, `siemens-nrk`; any case); anything else a prefix of
/// the size, as the search took it before (`NRK`, `NRK070`, `SP+100MF`).
///
/// Siemens' gearboxes fit as Siemens sells them (`siemens_fits`). Another maker's fit any motor whose shaft
/// (`motor_shaft_diameter`) the input's clamping hub takes, through the adapter plate the maker makes for the motor's
/// flange: a hub bore at least the shaft's (a smaller shaft takes a bushing). Of the bores a size and ratio comes with,
/// the smallest that takes the shaft (its inertia is the lowest). A motor whose shaft no table has gets none of them.
/// shortcut: the flange is taken to have an adapter plate (the makers list them for every common servo motor) and the
/// shaft's length is not checked; upgrade with the makers' motor-specific adapter lists, which need their configurators.
pub(super) fn fitting(article: &str, series: &str) -> Vec<(String, f64)> {
    let wanted = series.to_ascii_lowercase();
    let picked = |g: &Gearbox| {
        series.is_empty()
            || wanted == g.maker
            || wanted == format!("{}-{}", g.maker, g.series)
            || g.row["size"].starts_with(series)
    };
    let shaft = motor_shaft_diameter(article);
    let mut out = Vec::new();
    // Per maker, size and ratio of the adapter gearboxes: the smallest hub that takes the shaft, its place in `out`.
    let mut smallest: HashMap<(&str, &str, &str), (f64, usize)> = HashMap::new();
    for g in all_gearboxes().into_iter().filter(picked) {
        let Some(ratio) = number(&g.row, "ratio") else { continue };
        if g.maker == "siemens" {
            if siemens_fits(article, &g) {
                out.push((gearbox_name(&g.row), ratio));
            }
            continue;
        }
        let (Some(shaft), Some(hub)) = (shaft, number(&g.row, "clamping_hub_diameter_m")) else { continue };
        if hub + 1e-9 < shaft {
            continue;
        }
        let key = (g.maker, g.row["size"], g.row["ratio"]);
        match smallest.get(&key) {
            Some(&(best, at)) if hub < best => {
                out[at] = (gearbox_name(&g.row), ratio);
                smallest.insert(key, (hub, at));
            }
            Some(_) => {}
            None => {
                smallest.insert(key, (hub, out.len()));
                out.push((gearbox_name(&g.row), ratio));
            }
        }
    }
    out
}

/// The mass of a gearbox in kg from its own row, where the table has one (the other makers'; Siemens' are in
/// `gearbox_mass.csv`).
pub(super) fn own_mass(name: &str) -> Option<f64> {
    find_gearbox(name).and_then(|(g, _)| number(&g.row, "mass_kg"))
}

/// The tables of a part class over every maker: `<maker>/<file>` for the given file name.
fn rows_of(file: &str) -> Vec<Row> {
    TABLES
        .iter()
        .filter(|(name, _)| name.ends_with(&format!("/{file}")))
        .flat_map(|(name, _)| rows(table(name)))
        .collect()
}

/// The linear motors a supply runs (`SUPPLIES`): the Siemens 1FN3 on "s120" (their Motor Modules), another maker's on
/// its supply, at the supply's line voltage where the row names one.
fn linear_rows(supply: &str) -> Vec<Row> {
    let (maker, line) = match (supply, vendor_supply(supply)) {
        ("s120", _) => ("siemens", None),
        (_, Some((maker, line))) => (maker, Some(line)),
        _ => return Vec::new(),
    };
    TABLES
        .iter()
        .filter(|(name, _)| {
            name.split_once('/').is_some_and(|(m, file)| m == maker && file.starts_with("linear_motors_"))
        })
        .flat_map(|(name, _)| rows(table(name)))
        .filter(|r| line.is_none() || number(r, "line_voltage_v").is_none_or(|v| Some(v) == line))
        .collect()
}

/// A linear motor of a supply by part number (`linear_rows`).
pub(super) fn linear_motor_row(article: &str, supply: &str) -> Option<Row> {
    linear_rows(supply).into_iter().find(|r| r["part_number"] == article)
}

/// The part numbers of the linear motors a supply runs (`linear_rows`) that have a coil mass, force constant, forces
/// and currents: what a linear motor machine's search tries in place of rotary motors.
pub fn linear_motor_articles_for(supply: &str) -> Vec<&'static str> {
    let complete = |r: &Row| {
        [
            "coil_mass_kg",
            "continuous_force_n",
            "continuous_current_a",
            "peak_force_n",
            "peak_current_a",
            "max_speed_m_s",
        ]
        .iter()
        .all(|c| number(r, c).is_some())
    };
    linear_rows(supply).into_iter().filter(complete).map(|r| r["part_number"]).collect()
}

/// Whether a config is a linear motor machine: it has `[linear_motor]` (a linear motor axis).
pub fn is_linear_motor(cfg: &Config) -> bool {
    cfg.get("linear_motor.attraction_force_n").is_ok()
}

/// The tables of a config with the same keys as one of its tables, e.g. those with `lead_m` and `root_diameter_m`:
/// `[screw]` of a screw axis, `[screw_z]` of a machine file.
fn tables_with(cfg: &Config, keys: &[&str]) -> Vec<String> {
    let mut found: HashMap<String, usize> = HashMap::new();
    for (path, _) in cfg.leaves() {
        if let Some((table, key)) = path.split_once('.')
            && !key.contains('.')
            && keys.contains(&key)
        {
            *found.entry(table.to_string()).or_default() += 1;
        }
    }
    let mut out: Vec<String> = found.into_iter().filter(|(_, n)| *n == keys.len()).map(|(t, _)| t).collect();
    out.sort();
    out
}

fn screw_tables(cfg: &Config) -> Vec<String> {
    tables_with(cfg, &["lead_m", "root_diameter_m", "dynamic_load_rating_n"])
}

/// The carriages a guide set has: two rails with two carriages each, the layout every shipped screw, rack and linear
/// motor config states. shortcut: the count is fixed; upgrade with a config key for the carriages when a machine has
/// another layout.
pub const GUIDE_CARRIAGES: f64 = 4.0;

/// The static safety factor a guide set's highest load is held to, C0 / P: HIWIN's catalogue asks 1.0 to 3.5 for
/// normal loading and 2.0 to 5.0 with vibration or impact [estimate: the middle of normal loading].
pub const GUIDE_STATIC_SAFETY: f64 = 2.0;

/// The distance a guide's dynamic load rating C is given for when `guide_life_km` takes it: 50 km (the formula
/// L10 = (C / P)^3 50 km). A rating for 100 km is 2^(1/3) times smaller.
const GUIDE_RATING_DISTANCE_M: f64 = 50e3;

fn screw_row(name: &str) -> Option<Row> {
    rows_of("ball_screws.csv").into_iter().find(|r| r["part_number"] == name)
}

/// A guide set by name, "<part_number>x<carriages>" (e.g. "HGH25CAx4"): the row and the number of carriages.
fn guide_set(name: &str) -> Option<(Row, f64)> {
    let (part, count) = name.rsplit_once('x')?;
    let count: f64 = count.parse().ok().filter(|n: &f64| *n >= 1.0 && n.fract() == 0.0)?;
    Some((rows_of("linear_guides.csv").into_iter().find(|r| r["part_number"] == part)?, count))
}

/// A guide's dynamic load rating for 50 km: the printed C converted from the distance the maker gives it for, by the
/// cube root of the distances (the life goes with C cubed). `None` without both.
fn guide_rating_50km(r: &Row) -> Option<f64> {
    Some(
        number(r, "dynamic_load_rating_n")?
            * (number(r, "dynamic_load_rating_distance_m")? / GUIDE_RATING_DISTANCE_M).cbrt(),
    )
}

/// Puts a ball screw into each screw table of a config (`screw_tables`): lead, nominal and root diameter, the load
/// ratings, and where the row has them the nut's stiffness, preload, efficiency and the d n its ball return takes.
/// Length and mounting stay the config's; a screw longer than the maker makes is refused.
fn apply_screw(cfg: &mut Config, r: &Row) -> Result<(), ConfigError> {
    let name = r["part_number"];
    let need = |column: &str| number(r, column).ok_or_else(|| unknown("axis", name));
    let tables = screw_tables(cfg);
    if tables.is_empty() {
        return Err(ConfigError::Invalid("parts.axis (the machine has no ball screw)".into()));
    }
    for t in tables {
        if number(r, "max_length_m").is_some_and(|max| cfg.f64(&format!("{t}.length_m")).is_ok_and(|l| l > max)) {
            return Err(ConfigError::Invalid(format!("{t}.length_m")));
        }
        for (key, column) in [
            ("lead_m", "lead_m"),
            ("nominal_diameter_m", "nominal_diameter_m"),
            ("root_diameter_m", "root_diameter_m"),
            ("dynamic_load_rating_n", "dynamic_load_rating_n"),
            ("static_load_rating_n", "static_load_rating_n"),
        ] {
            set(cfg, &format!("{t}.{key}"), need(column)?)?;
        }
        for (key, column) in [
            ("nut_stiffness_n_m", "axial_stiffness_n_m"),
            ("preload_n", "preload_n"),
            ("efficiency", "efficiency"),
            ("max_dn", "max_dn"),
        ] {
            if let Some(value) = number(r, column) {
                set_if_present(cfg, &format!("{t}.{key}"), value)?;
            }
        }
    }
    Ok(())
}

/// Puts a guide set into a config's ratings: `guide_dynamic_load_rating_n` the carriages' C (for 50 km) together, and
/// `guide_max_load_n` their C0 together over `GUIDE_STATIC_SAFETY`, each where the config rates it. A config that rates
/// neither is refused: the guide would change nothing it judges.
/// shortcut: the load is taken to share equally between the carriages, and the config's carriage mass and guide
/// friction stay (they hold the guides it was written for); upgrade with the carriages' mass and seal drag per guide.
fn apply_guides(cfg: &mut Config, r: &Row, count: f64) -> Result<(), ConfigError> {
    let name = r["part_number"];
    let dynamic = guide_rating_50km(r).ok_or_else(|| unknown("axis", name))?;
    let stat = number(r, "static_load_rating_n").ok_or_else(|| unknown("axis", name))?;
    if !rates_guides(cfg) {
        return Err(ConfigError::Invalid("parts.axis (the config rates no guides)".into()));
    }
    set_if_present(cfg, "ratings.guide_dynamic_load_rating_n", count * dynamic)?;
    set_if_present(cfg, "ratings.guide_max_load_n", count * stat / GUIDE_STATIC_SAFETY)
}

fn rates_guides(cfg: &Config) -> bool {
    cfg.get("ratings.guide_dynamic_load_rating_n").is_ok() || cfg.get("ratings.guide_max_load_n").is_ok()
}

/// Puts a mechanical axis into a config: its parts by name, joined by "+": ball screws (`ball_screws.csv`, part
/// number) and guide sets ("<part number>x<carriages>" of `linear_guides.csv`). `None` where a part is in no table.
pub(super) fn apply_mech_axis(cfg: &mut Config, name: &str) -> Option<Result<(), ConfigError>> {
    let mut screws = Vec::new();
    let mut guides = Vec::new();
    for part in name.split('+') {
        if let Some(s) = screw_row(part) {
            screws.push(s);
        } else {
            guides.push(guide_set(part)?);
        }
    }
    let mut apply = || -> Result<(), ConfigError> {
        for s in &screws {
            apply_screw(cfg, s)?;
        }
        for (g, count) in &guides {
            apply_guides(cfg, g, *count)?;
        }
        Ok(())
    };
    Some(apply())
}

/// The highest speed and acceleration a config's axes allow ([axis.<id>]): of the axis `id` where it has one, else of
/// all of them.
fn axis_limits(cfg: &Config, id: Option<&str>) -> (f64, f64) {
    let get = |key: &str| {
        let own = id.and_then(|id| cfg.f64(&format!("axis.{id}.{key}")).ok());
        own.unwrap_or_else(|| {
            cfg.leaves()
                .into_iter()
                .filter(|(p, _)| p.starts_with("axis.") && p.ends_with(&format!(".{key}")))
                .filter_map(|(_, v)| v.as_float())
                .fold(0.0, f64::max)
        })
    };
    (get("max_velocity_m_s"), get("max_acceleration_m_s2"))
}

/// Why a ball screw cannot carry a config's axis by its catalogue numbers alone, "" where it can: at the axis's highest
/// speed it would turn above `critical_speed_factor` times its critical speed or above the d n its nut takes
/// (`axis-speed`); the force to accelerate carriage and payload at the axis's highest acceleration (and to lift them
/// on an inclined axis) is above its static rating over `static_safety_factor` (`axis-payload`); or it is longer than
/// the maker makes it (`axis-stroke`). "config" where the screw does not make a valid `[screw]`.
fn screw_refusal(cfg: &Config, r: &Row) -> &'static str {
    let mut with = cfg.clone();
    if let Err(e) = apply_screw(&mut with, r) {
        return if e.key() == "error-config-invalid" { "axis-stroke" } else { "config" };
    }
    let mass = with.f64("mass.carriage_kg").unwrap_or(0.0) + with.f64("mass.payload_kg").unwrap_or(0.0);
    let lift = with.f64("slide.incline_deg").unwrap_or(0.0).to_radians().sin().abs() * 9.81;
    for t in screw_tables(&with) {
        // `BallScrew` reads `[screw]`: the config with this table copied there.
        let shell = with.to_text().ok().and_then(|text| text.parse::<toml::Table>().ok()).and_then(|mut all| {
            let own = all.get(&t)?.clone();
            all.insert("screw".into(), own);
            Config::parse(&toml::to_string(&all).ok()?).ok()
        });
        let Some(Ok(screw)) = shell.map(|c| BallScrew::from_config(&c)) else { return "config" };
        let (speed, acceleration) = axis_limits(&with, t.strip_prefix("screw_"));
        let rpm = speed / screw.lead * 60.0;
        if rpm > screw.allowed_speed_rpm() || (screw.max_dn > 0.0 && screw.nominal_diameter * 1e3 * rpm > screw.max_dn)
        {
            return "axis-speed";
        }
        if mass * (acceleration + lift) > screw.static_rating / screw.static_safety {
            return "axis-payload";
        }
    }
    ""
}

/// Why a guide set cannot carry a config's machine by its catalogue numbers alone: the weight of carriage and payload
/// and the linear motor's magnetic attraction (where there is one) above its C0 over `GUIDE_STATIC_SAFETY`
/// (`axis-payload`); "" where it can.
fn guide_refusal(cfg: &Config, r: &Row, count: f64) -> &'static str {
    let mass = cfg.f64("mass.carriage_kg").unwrap_or(0.0) + cfg.f64("mass.payload_kg").unwrap_or(0.0);
    let load = mass * 9.81 + cfg.f64("linear_motor.attraction_force_n").unwrap_or(0.0);
    match number(r, "static_load_rating_n") {
        Some(c0) if load <= count * c0 / GUIDE_STATIC_SAFETY => "",
        _ => "axis-payload",
    }
}

/// The stroke of a config's machine for the weights of rails and magnet tracks: `slide.stroke_m`, else the longest
/// screw.
fn stroke(cfg: &Config) -> f64 {
    cfg.f64("slide.stroke_m").unwrap_or_else(|_| {
        screw_tables(cfg).iter().filter_map(|t| cfg.f64(&format!("{t}.length_m")).ok()).fold(0.0, f64::max)
    })
}

/// The installed weight of a ball screw at a config's lengths, kg: shaft and nut. A shaft whose mass per metre the
/// maker does not print is a steel cylinder of its nominal diameter, as `BallScrew::inertia` takes it.
fn screw_weight(cfg: &Config, r: &Row) -> f64 {
    let per_m = number(r, "mass_per_length_kg_m").unwrap_or_else(|| {
        let d = number(r, "nominal_diameter_m").unwrap_or(0.0);
        crate::transmission::STEEL_DENSITY * std::f64::consts::PI * d * d / 4.0
    });
    let length: f64 = screw_tables(cfg).iter().filter_map(|t| cfg.f64(&format!("{t}.length_m")).ok()).sum();
    per_m * length + number(r, "nut_mass_kg").unwrap_or(0.0)
}

/// The installed weight of a guide set, kg: the carriages and two rails as long as the stroke and two carriages.
/// shortcut: the rails' length is the stroke plus two carriage lengths; upgrade with the machine's rail length.
fn guide_weight(cfg: &Config, r: &Row, count: f64) -> f64 {
    let rail = stroke(cfg) + 2.0 * number(r, "carriage_length_m").unwrap_or(0.0);
    count * number(r, "carriage_mass_kg").unwrap_or(0.0) + 2.0 * rail * number(r, "rail_mass_kg_m").unwrap_or(0.0)
}

/// The weight of a linear motor's magnet track at a config's stroke, kg: the stroke and the coil's length of track.
pub fn magnet_track_weight(cfg: &Config, article: &str, supply: &str) -> f64 {
    linear_motor_row(article, supply).map_or(0.0, |r| {
        let length = stroke(cfg) + number(&r, "coil_length_m").unwrap_or(0.0);
        number(&r, "magnet_track_mass_kg_m").unwrap_or(0.0) * length
    })
}

/// How many guide sizes of a series and carriage type the axis search tries: the smallest that carry the machine by
/// their static rating and the next one up (a bigger one only adds weight unless the cycle's life needs it).
const GUIDE_SIZES_TRIED: usize = 2;

/// The mechanical axes of the catalogue a config's machine can be built from, as (name for `parts.axis`, why it is
/// refused, "" where it is not; installed weight in kg): for a machine with ball screws every catalogue screw
/// (`screw_refusal`), and where the config rates guides (a screw, rack or linear motor machine with
/// `ratings.guide_*`) with each guide set (`GUIDE_CARRIAGES` carriages; per series and carriage type the
/// `GUIDE_SIZES_TRIED` smallest that pass `guide_refusal`). A screw refused is listed once on its own. Empty for a
/// machine of another kind. A linear motor machine's motors are the linear motors (`linear_motor_articles_for`).
pub fn mech_axes(cfg: &Config) -> Vec<(String, &'static str, f64)> {
    let screws: Vec<Row> = if screw_tables(cfg).is_empty() { Vec::new() } else { rows_of("ball_screws.csv") };
    let mech = !screws.is_empty() || is_linear_motor(cfg) || !tables_with(cfg, &["module_m", "teeth"]).is_empty();
    let mut guides: Vec<(String, f64)> = Vec::new();
    if mech && rates_guides(cfg) {
        let mut all: Vec<Row> = rows_of("linear_guides.csv")
            .into_iter()
            .filter(|r| guide_rating_50km(r).is_some() && guide_refusal(cfg, r, GUIDE_CARRIAGES).is_empty())
            .collect();
        all.sort_by(|a, b| number(a, "static_load_rating_n").partial_cmp(&number(b, "static_load_rating_n")).unwrap());
        let mut tried: HashMap<(&str, &str), usize> = HashMap::new();
        for r in &all {
            let n = tried
                .entry((r.get("series").copied().unwrap_or(""), r.get("carriage_type").copied().unwrap_or("")))
                .or_default();
            if *n < GUIDE_SIZES_TRIED {
                *n += 1;
                guides
                    .push((format!("{}x{}", r["part_number"], GUIDE_CARRIAGES), guide_weight(cfg, r, GUIDE_CARRIAGES)));
            }
        }
    }
    let mut out = Vec::new();
    for s in &screws {
        let (name, reason) = (s["part_number"].to_string(), screw_refusal(cfg, s));
        let weight = screw_weight(cfg, s);
        if !reason.is_empty() || guides.is_empty() {
            out.push((name, reason, weight));
            continue;
        }
        out.extend(guides.iter().map(|(g, w)| (format!("{name}+{g}"), "", weight + w)));
    }
    if screws.is_empty() {
        out.extend(guides.into_iter().map(|(g, w)| (g, "", w)));
    }
    out
}
