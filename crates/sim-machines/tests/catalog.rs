//! The component catalogue in `catalog/`: every file reads, and the numbers agree with each other
//! where physics ties them together. A value that landed in the wrong field or row breaks one of these.

use std::collections::HashSet;
use std::f64::consts::{PI, SQRT_2};
use std::fs;
use std::path::{Path, PathBuf};
use toml::Table;

fn catalog() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../catalog")
}

fn close(a: f64, b: f64, tolerance: f64) -> bool {
    (a - b).abs() <= tolerance * b.abs()
}

#[test]
fn festo_exct_values_agree_with_its_own_geometry() {
    let file: Table = fs::read_to_string(catalog().join("festo/exct.toml")).unwrap().parse().unwrap();
    let number = |t: &Table, key: &str| -> f64 {
        let v = &t[key];
        v.as_float().or_else(|| v.as_integer().map(|i| i as f64)).unwrap_or_else(|| panic!("{key} is not a number"))
    };
    let sizes: Vec<(&String, &Table)> = file.iter().filter_map(|(k, v)| Some((k, v.as_table()?))).collect();
    assert_eq!(sizes.len(), 3);
    for (name, size) in sizes {
        let belt = size["belt"].as_table().unwrap();
        // One turn of the pulley feeds one circumference of belt.
        let diameter = number(belt, "effective_diameter_mm");
        assert!(close(number(belt, "feed_constant_mm_rev"), PI * diameter, 1e-3), "{name}: feed constant");
        assert!(number(size, "max_no_load_torque_nm") < number(size, "max_torque_nm"), "{name}");
        // The torque formula is the T-gantry's own mechanics at 45 degrees, where one motor carries
        // both axes: M = r/sqrt(2) m a for the load, J sqrt(2)/r a for the motor, m g r/2 to hold Z.
        let r = diameter / 2000.0;
        let formulas = size["torque_formula"].as_array().unwrap();
        assert_eq!(formulas.len(), size["strokes_z_mm"].as_array().unwrap().len(), "{name}: one formula per Z stroke");
        let (mut last_constant, mut last_mass) = (0.0, 0.0);
        for formula in formulas {
            let f = formula.as_table().unwrap();
            assert!(close(number(f, "load_factor"), r / SQRT_2 * 1e3, 0.01), "{name}: load factor");
            // The motor inertia is in kg cm2, hence 1e-4, and the formula scales by 1e-3.
            assert!(close(number(f, "motor_inertia_factor"), SQRT_2 / r * 1e-4 * 1e3, 0.01), "{name}: motor factor");
            assert!(close(number(f, "gravity_factor"), 9.81 * r / 2.0, 0.03), "{name}: gravity factor");
            // A longer Z axis is heavier.
            let (constant, mass) = (number(f, "constant"), number(f, "gravity_mass_kg"));
            assert!(constant > last_constant && mass > last_mass, "{name}: Z strokes in rising order");
            (last_constant, last_mass) = (constant, mass);
        }
    }
}

/// The rows of a catalogue CSV as (column name -> text) lookups; lines starting with '#' are notes.
fn csv(path: &str) -> Vec<std::collections::HashMap<String, String>> {
    let text = fs::read_to_string(catalog().join(path)).unwrap();
    let mut lines = text.lines().filter(|l| !l.starts_with('#'));
    let header: Vec<&str> = lines.next().unwrap().split(',').collect();
    lines
        .map(|line| {
            let fields: Vec<&str> = line.split(',').collect();
            assert_eq!(fields.len(), header.len(), "{path}: {line}");
            header.iter().zip(fields).map(|(h, f)| (h.to_string(), f.to_string())).collect()
        })
        .collect()
}

#[test]
fn siemens_s210_motors_are_plausible_and_their_converters_exist() {
    let converters = csv("siemens/converters_s210.csv");
    let (motors, mut articles) = (csv("siemens/motors_s210.csv"), HashSet::new());
    assert!(motors.len() >= 90 && converters.len() == 24, "{} motors, {} converters", motors.len(), converters.len());
    for c in &converters {
        let number = |name: &str| -> f64 { c[name].parse().unwrap_or_else(|_| panic!("{name} of {}", c["article"])) };
        assert!(number("max_output_current_a") > 2.0 * number("rated_output_current_a"), "{}", c["article"]);
        assert!(number("line_current_a") > 0.0 && number("power_loss_w") > 0.0, "{}", c["article"]);
    }
    for m in &motors {
        let name = &m["article"];
        let number = |column: &str| -> f64 { m[column].parse().unwrap_or_else(|_| panic!("{column} of {name}")) };
        assert!(articles.insert(name), "{name} twice");
        let (m0, i0) = (number("static_torque_nm"), number("stall_current_a"));
        assert!(number("max_torque_nm") > 1.5 * m0 && number("max_current_a") > 1.5 * i0, "{name}: overload");
        // Torque per current: a servo motor of this class gives a few tenths to a few N m per ampere.
        assert!((0.15..=4.5).contains(&(m0 / i0)), "{name}: {} N m/A", m0 / i0);
        assert!(number("rotor_inertia_with_brake_kgcm2") >= number("rotor_inertia_kgcm2"), "{name}: inertia");
        assert!(number("mass_with_brake_kg") > number("mass_kg"), "{name}: mass");
        let mut supplies = 0;
        for (voltage, supply_columns) in [("230v", ["230v_1ac", "230v_3ac"]), ("400v", ["400v_3ac", "400v_3ac"])] {
            let rated = |quantity: &str, unit: &str| format!("rated_{quantity}_{voltage}_{unit}");
            if m[&rated("speed", "rpm")].is_empty() {
                continue;
            }
            supplies += 1;
            let (speed, torque, current) = (number(&rated("speed", "rpm")), number(&rated("torque", "nm")), number(&rated("current", "a")));
            // The catalogue prints a rated current of 3.75 A at 230 V for the 1FT2206-2AC, above its
            // stall current of 2.8 A and above its 3 A converter: most likely a misprint for 2.75 A.
            // The file keeps what is printed, so this one value is not checked.
            let misprint = name == "1FT2206-2AC" && voltage == "230v";
            assert!(speed <= number("max_speed_rpm") && torque <= m0, "{name} at {voltage}");
            assert!(misprint || current <= 1.05 * i0, "{name}: rated current at {voltage}");
            if !m[&rated("power", "w")].is_empty() {
                assert!(close(number(&rated("power", "w")), torque * speed * PI / 30.0, 0.12), "{name}: power at {voltage}");
            }
            // The converters the catalogue pairs with the motor exist and can carry its rated current
            // (the tightest pairing in the catalogue is a converter with 88 % of the motor's rated current).
            let mut paired = 0;
            for series in ["converter_", "converter_new_"] {
                for supply in supply_columns {
                    let article = &m[&format!("{series}{supply}")];
                    if article.is_empty() {
                        continue;
                    }
                    let converter = converters.iter().find(|c| &c["article"] == article).unwrap_or_else(|| panic!("{name}: no {article}"));
                    let available: f64 = converter["rated_output_current_a"].parse().unwrap();
                    assert!(misprint || available >= 0.85 * current, "{name} needs {current} A, {article} gives {available} A");
                    paired += 1;
                }
            }
            assert!(paired > 0, "{name}: no converter at {voltage}");
        }
        assert!(supplies > 0, "{name}: no rated data");
    }
}

#[test]
fn siemens_s120_motor_modules_are_plausible() {
    let modules = csv("siemens/motor_modules_s120.csv");
    assert_eq!(modules.len(), 19);
    let mut articles = HashSet::new();
    for m in &modules {
        let name = &m["article"];
        let number = |column: &str| -> f64 { m[column].parse().unwrap_or_else(|_| panic!("{column} of {name}")) };
        assert!(articles.insert(name), "{name} twice");
        let (rated, base, max) = (number("rated_current_a"), number("base_load_current_a"), number("max_current_a"));
        assert!(base < rated && rated < max && max <= 3.0 * rated, "{name}: currents");
        // The type rating is for an induction motor at 400 V: sqrt(3) 400 V I cos(phi) eta, about 0.54 kW per ampere.
        assert!(close(number("rated_power_kw"), 0.54 * rated, 0.05), "{name}: type rating");
        assert!(number("dc_link_capacitance_uf") >= 110.0, "{name}: capacitance");
        assert!(number("power_loss_typical_kw") <= number("power_loss_max_kw"), "{name}: power loss");
        // The D type is the one for short, high overloads.
        if m["type"] == "D" {
            let same_size = |c: &&std::collections::HashMap<String, String>| {
                c["type"] == "C" && c["axes"] == m["axes"] && c["rated_current_a"] == m["rated_current_a"]
            };
            if let Some(c) = modules.iter().find(same_size) {
                assert!(max > c["max_current_a"].parse::<f64>().unwrap(), "{name}: D type against C type");
            }
        }
    }
}

#[test]
fn siemens_s120_motors_are_plausible_and_their_motor_modules_exist() {
    let (motors, modules) = (csv("siemens/motors_s120.csv"), csv("siemens/motor_modules_s120.csv"));
    assert!(motors.len() >= 160, "{} motors", motors.len());
    let mut articles = HashSet::new();
    for m in &motors {
        let name = &m["article"];
        let number = |column: &str| -> f64 { m[column].parse().unwrap_or_else(|_| panic!("{column} of {name}")) };
        assert!(articles.insert(name), "{name} twice");
        let (speed, m0, rated, i0) = (number("rated_speed_rpm"), number("static_torque_nm"), number("rated_torque_nm"), number("stall_current_a"));
        assert!(rated <= m0, "{name}: rated torque above static torque");
        // Power is torque times speed: the rated power from the rated torque, the calculated power from
        // the static one. Some rated values hold at a lower speed, which a footnote of the catalogue names.
        // The smallest motors are printed to one digit, hence the 15 W of slack.
        let rated_at = if m["rated_values_at_rpm"].is_empty() { speed } else { number("rated_values_at_rpm") };
        assert!(rated_at <= speed, "{name}: reduced speed");
        for (column, torque, at) in [("rated_power_kw", rated, rated_at), ("calculated_power_kw", m0, speed)] {
            let power = torque * at * PI / 30e3;
            assert!((number(column) - power).abs() <= 0.08 * power + 0.015, "{name}: {column} against {power} kW");
        }
        assert!((0.1..=6.0).contains(&(m0 / i0)), "{name}: {} N m/A", m0 / i0);
        assert!(number("pole_pairs").fract() == 0.0 && (2.0..=10.0).contains(&number("pole_pairs")), "{name}: pole pairs");
        assert!((50.0..=98.0).contains(&number("efficiency_percent")), "{name}: efficiency");
        assert!(number("rotor_inertia_kgcm2") > 0.0 && number("mass_kg") > 0.0, "{name}");
        // The Motor Module the catalogue pairs with the motor carries its stall current. In the
        // module's article number "7" stands for Single or Double and the last character is open.
        let current = number("motor_module_rated_current_a");
        assert!(current >= 0.9 * i0, "{name}: {i0} A at standstill on a {current} A Motor Module");
        let article = m["motor_module"].replacen("-7", "-1", 1);
        let known = modules.iter().find(|module| module["article"].starts_with(article.trim_end_matches('.')));
        let rated_current = |module: &std::collections::HashMap<String, String>| module["rated_current_a"].parse::<f64>().unwrap();
        match known {
            Some(module) => assert_eq!(rated_current(module), current, "{name}: {article}"),
            // The 2017 catalogue still pairs some motors with the 45 A and 60 A modules that the
            // Catalog News replaced by C types; a module of that current must exist all the same.
            None => assert!(modules.iter().any(|module| rated_current(module) == current), "{name}: no {current} A Motor Module"),
        }
    }
}

#[test]
fn siemens_gearbox_rows_are_complete_and_plausible() {
    for series in ["nrb", "nrk", "nlc"] {
        let text = fs::read_to_string(catalog().join(format!("siemens/gearbox_{series}.csv"))).unwrap();
        let mut lines = text.lines().filter(|l| !l.starts_with('#'));
        let header: Vec<&str> = lines.next().unwrap().split(',').collect();
        let column = |name: &str| header.iter().position(|h| *h == name).unwrap_or_else(|| panic!("no column {name}"));
        let (mut seen, mut stage_of, mut rows) = (HashSet::new(), std::collections::HashMap::new(), 0);
        for line in lines {
            let fields: Vec<&str> = line.split(',').collect();
            assert_eq!(fields.len(), header.len(), "{line}");
            let text = |name: &str| fields[column(name)];
            let number = |name: &str| -> f64 { text(name).parse().unwrap_or_else(|_| panic!("{name} in {line}")) };
            assert!(text("size").to_lowercase().starts_with(series), "{line}");
            assert!(seen.insert((text("size"), text("motor_frame"), text("ratio"))), "twice: {line}");
            // A ratio belongs to one number of stages, and more stages give higher ratios.
            let (stages, ratio) = (number("stages"), number("ratio"));
            assert_eq!(*stage_of.entry(text("ratio")).or_insert(stages), stages, "{line}");
            assert!((1.0..=3.0).contains(&stages) && ratio >= [3.0, 9.0, 60.0][stages as usize - 1], "{line}");
            let (rated, max, emergency) = (number("rated_torque_nm"), number("max_torque_nm"), number("emergency_off_torque_nm"));
            assert!(0.0 < rated && rated < max && max < emergency, "torques out of order: {line}");
            assert!((0.75..=0.99).contains(&number("efficiency_full_load")), "{line}");
            assert!(number("torsional_stiffness_nm_arcmin") > 0.0, "{line}");
            // The manual leaves a few of these two open; where given they must be sensible.
            for (name, range) in [("backlash_arcmin", 5.0..=25.0), ("inertia_at_input_kgcm2", 0.01..=30.0)] {
                assert!(text(name).is_empty() || range.contains(&number(name)), "{name}: {line}");
            }
            rows += 1;
        }
        assert!(rows > 150, "{series}: only {rows} rows");
    }
}

/// The 1FT2 data sheets of the S120 configuration manual and the S210 catalogue describe the same
/// motors: two documents, read by two scripts, must agree.
#[test]
fn siemens_1ft2_data_agrees_between_the_s120_manual_and_the_s210_catalogue() {
    let (manual, catalogue) = (csv("siemens/motors_1ft2_s120.csv"), csv("siemens/motors_s210.csv"));
    assert!(manual.len() >= 36, "{} motors", manual.len());
    let mut shared = 0;
    for m in &manual {
        let name = &m["article"];
        let number = |row: &std::collections::HashMap<String, String>, column: &str| -> f64 {
            row[column].parse().unwrap_or_else(|_| panic!("{column} of {name}"))
        };
        assert!((5.0..=120.0).contains(&number(m, "thermal_time_constant_min")), "{name}: thermal time constant");
        let power = number(m, "rated_torque_nm") * number(m, "rated_speed_rpm") * PI / 30e3;
        // For three motors the manual prints a rated speed that contradicts its own rated torque and power and
        // the S210 catalogue (3000 for 4500 r/min, 2000 for 3000 r/min). The file keeps what is printed, so
        // these rows are not checked.
        let misprint = ["1FT2105-4AH", "1FT2206-4AH", "1FT2208-4AF"].contains(&name.as_str());
        assert!(misprint || (number(m, "rated_power_kw") - power).abs() <= 0.08 * power + 0.015, "{name}: rated power");
        let Some(c) = catalogue.iter().find(|c| &c["article"] == name) else { continue };
        shared += 1;
        for (column, tolerance) in [("static_torque_nm", 0.05), ("stall_current_a", 0.05), ("max_torque_nm", 0.05), ("max_current_a", 0.05), ("rotor_inertia_kgcm2", 0.06), ("mass_kg", 0.06)] {
            assert!(close(number(m, column), number(c, column), tolerance), "{name}: {column} {} against {}", m[column], c[column]);
        }
    }
    assert!(shared >= 30, "only {shared} motors in both files");
}

/// The real-machine config runs its cycle inside every limit the simulator judges so far.
#[test]
fn exct_30_cycle_stays_within_motor_and_belt_limits() {
    let cfg = sim_core::Config::parse(machine_t_gantry::EXCT_30_CONFIG).unwrap();
    let mut sim = sim_components::Sim::new(&machine_t_gantry::ENTRY, &cfg).unwrap();
    assert!(sim.limit_violations.is_empty(), "{:?}", sim.limit_violations);
    let load = sim_components::analysis::analyse_cycle(&mut sim).unwrap();
    for motor in &load.motors {
        assert!(motor.utilisation < 0.8 && motor.thermal_utilisation < 0.8, "{} of the peak curve, {} thermal", motor.utilisation, motor.thermal_utilisation);
    }
    let (slackest, tightest) = load.belt_tension.unwrap();
    assert!(slackest > 0.0, "a belt section goes slack");
    assert!(tightest < 12.5 / 0.01591, "{tightest} N is more than the gantry's torque limit puts on the belt");
}

#[test]
fn siemens_s120_line_modules_are_plausible() {
    let modules = csv("siemens/line_modules_s120.csv");
    assert_eq!(modules.len(), 13);
    let mut articles = HashSet::new();
    for kind in ["smart", "active", "basic"] {
        let (mut last_power, mut last_capacitance) = (0.0, 0.0);
        for m in modules.iter().filter(|m| m["type"] == kind) {
            let name = &m["article"];
            let number = |column: &str| -> f64 { m[column].parse().unwrap_or_else(|_| panic!("{column} of {name}")) };
            assert!(articles.insert(name), "{name} twice");
            let (rated, s6, max) = (number("rated_power_kw"), number("s6_power_kw"), number("max_power_kw"));
            assert!(rated < s6 && s6 < max, "{name}: powers");
            // Power is DC link voltage times DC link current; the catalogue rounds to whole amperes.
            assert!(close(number("dc_link_current_600v_a"), rated * 1e3 / 600.0, 0.04), "{name}: rated DC link current");
            if !m["dc_link_current_540v_a"].is_empty() {
                assert!(close(number("dc_link_current_540v_a"), rated * 1e3 / 540.0, 0.04), "{name}: DC link current at 540 V");
            }
            // The catalogue prints 138 A and 178 A for the 55 kW Smart Line Module, about 510 V at its
            // 71 kW and 91 kW, where every other module has 600 V. The file keeps what is printed, so
            // these two values are not checked.
            let misprint = name == "6SL3130-6TE25-5AA3";
            for (current, power) in [("s6_dc_link_current_a", s6), ("max_dc_link_current_a", max)] {
                assert!(misprint || close(number(current), power * 1e3 / 600.0, 0.04), "{name}: {current}");
            }
            // A bigger module has more capacitance, and loses one to two percent of what it converts.
            let capacitance = number("dc_link_capacitance_uf");
            assert!(rated > last_power && capacitance > last_capacitance, "{name}: sizes in rising order");
            (last_power, last_capacitance) = (rated, capacitance);
            assert!((0.005..=0.025).contains(&(number("power_loss_kw") / rated)), "{name}: power loss");
            // Only a Basic Line Module has a braking chopper, and the 100 kW one has none.
            if m["braking_threshold_v"].is_empty() {
                assert!(kind != "basic" || rated == 100.0, "{name}: no braking chopper");
                continue;
            }
            assert_eq!(kind, "basic", "{name}");
            // The chopper's peak power is what the smallest resistor takes at the threshold: U^2 / R.
            let (threshold, peak) = (number("braking_threshold_v"), number("braking_power_max_kw"));
            assert!(close(peak, threshold * threshold / number("min_braking_resistance_ohm") / 1e3, 0.03), "{name}: braking power");
            assert!(close(peak, 2.0 * rated, 1e-9) && close(number("braking_power_continuous_kw"), 0.25 * rated, 1e-9), "{name}");
        }
    }
    assert_eq!(articles.len(), modules.len(), "a type other than smart, active or basic");
}

#[test]
fn siemens_s120_braking_resistors_fit_their_modules() {
    let (braking, line_modules) = (csv("siemens/braking_s120.csv"), csv("siemens/line_modules_s120.csv"));
    assert_eq!(braking.len(), 5);
    let mut articles = HashSet::new();
    for b in &braking {
        let name = &b["article"];
        let number = |column: &str| -> f64 { b[column].parse().unwrap_or_else(|_| panic!("{column} of {name}")) };
        assert!(articles.insert(name), "{name} twice");
        let (rated, peak) = (number("rated_power_kw"), number("peak_power_kw"));
        assert!(rated < peak, "{name}: powers");
        if b["kind"] == "braking_module" {
            assert!((700.0..=800.0).contains(&number("activation_threshold_v")) && number("dc_link_capacitance_uf") > 0.0, "{name}");
            continue;
        }
        assert_eq!(b["kind"], "braking_resistor", "{name}");
        // The chopper that switches the resistor is in the Braking Module or in a Basic Line Module.
        let host = &b["for_article"];
        let field = |rows: &[std::collections::HashMap<String, String>], column: &str| -> Option<f64> {
            rows.iter().find(|r| &r["article"] == host).map(|r| r[column].parse().unwrap())
        };
        let (threshold, limit) = match field(&braking, "activation_threshold_v") {
            Some(threshold) => (threshold, field(&braking, "peak_power_kw").unwrap()),
            None => {
                let smallest = field(&line_modules, "min_braking_resistance_ohm").unwrap_or_else(|| panic!("{name}: no {host}"));
                assert!(number("resistance_ohm") >= smallest, "{name}: resistance below what {host} allows");
                (field(&line_modules, "braking_threshold_v").unwrap(), field(&line_modules, "braking_power_max_kw").unwrap())
            }
        };
        assert!(peak <= limit, "{name}: more peak power than {host} switches");
        // At the threshold voltage the resistor takes U^2 / R. The 6SN1113-1AA00-0DA0 is printed with
        // 25 kW at 17 ohm, where 770 V give 35 kW. The file keeps what is printed, so it is not checked.
        let at_threshold = threshold * threshold / number("resistance_ohm") / 1e3;
        assert!(name == "6SN1113-1AA00-0DA0" || close(peak, at_threshold, 0.05), "{name}: {peak} kW against {at_threshold} kW");
        // Over a duty cycle the peak power averages to no more than the continuous power.
        for (load, cycle) in [("load_duration_s", "cycle_duration_s"), ("load_duration_2_s", "cycle_duration_2_s")] {
            if b[load].is_empty() {
                assert!(b[cycle].is_empty() && load != "load_duration_s", "{name}: duty cycle");
                continue;
            }
            assert!(peak * number(load) / number(cycle) <= 1.01 * rated, "{name}: {load}");
        }
    }
}

/// The data sheets and the catalogue describe the same 1FK7 and 1FT7 motors from two Siemens sources:
/// pole count, torque constant and time constants must fit each other and the catalogue's values.
#[test]
fn siemens_s120_data_sheets_agree_with_the_catalogue() {
    let (sheets, motors) = (csv("siemens/motors_s120_electrical.csv"), csv("siemens/motors_s120.csv"));
    assert!(sheets.len() >= 160, "{} data sheets", sheets.len());
    for sheet in &sheets {
        let name = &sheet["article"];
        let motor = motors.iter().find(|m| &m["article"] == name).unwrap_or_else(|| panic!("{name} is not in the catalogue file"));
        let number = |row: &std::collections::HashMap<String, String>, column: &str| -> f64 {
            row[column].parse().unwrap_or_else(|_| panic!("{column} of {name}"))
        };
        assert!(sheet["data_sheet_article"].starts_with(name.as_str()), "{name}");
        // The 1FT7 High Dynamic motors (1FT706x-7 and 1FT708x-7) have 6 poles on their data sheets and 5 pole
        // pairs in the catalogue's table, the value of the Compact motors beside them. Both are kept as printed.
        let high_dynamic = name.starts_with("1FT70") && &name[8..9] == "7";
        assert!(high_dynamic || number(sheet, "poles") == 2.0 * number(motor, "pole_pairs"), "{name}: poles");
        // The torque constant is static torque over stall current, by Siemens' own definition.
        let kt = number(sheet, "torque_constant_nm_a");
        assert!(close(kt, number(motor, "static_torque_nm") / number(motor, "stall_current_a"), 0.08), "{name}: torque constant {kt}");
        // The voltage constant is line to line, RMS, per 1000 r/min; per phase and in V s/rad, three times
        // it is the torque constant again, apart from saturation and temperature.
        let ke = number(sheet, "voltage_constant_v_per_1000rpm") / 1000.0 * 30.0 / PI / 3f64.sqrt();
        // The data sheet of the 1FT7044-1AF prints 49 V per 1000 r/min beside a torque constant of 1.79 N m/A,
        // less than half of what fits. Kept as printed and not checked here.
        assert!(name == "1FT7044-1AF7" || close(3.0 * ke, kt, 0.15), "{name}: voltage constant against torque constant");
        // The electrical time constant is inductance over resistance.
        let (r, l) = (number(sheet, "winding_resistance_20c_ohm"), number(sheet, "inductance_mh"));
        assert!(close(number(sheet, "electrical_time_constant_ms"), l / r, 0.15), "{name}: L / R = {} ms", l / r);
        assert!(number(sheet, "max_torque_nm") > 1.5 * number(motor, "static_torque_nm"), "{name}: maximum torque");
        assert!(number(sheet, "max_speed_inverter_rpm") <= number(sheet, "max_speed_mechanical_rpm"), "{name}: speeds");
        // Water-cooled motors settle within a few minutes.
        assert!((1.0..=200.0).contains(&number(sheet, "thermal_time_constant_min")), "{name}: thermal time constant");
    }
}

/// The motor the EXCT-30 config names gives it the values the config has by hand.
#[test]
fn a_motor_by_name_gives_the_values_of_the_hand_written_config() {
    let by_hand = sim_core::Config::parse(machine_t_gantry::EXCT_30_CONFIG).unwrap();
    let mut cfg = by_hand.clone();
    for wrong in ["motor.torque_constant_nm_a = 0.2", "motor.rated_current_a = 1.0", "drive.peak_torque_nm = 1.0", "drive.rated_torque_nm = 0.1", "drive.rated_speed_rad_s = 10.0", "motor.thermal_time_constant_s = 5.0"] {
        cfg.set_text(wrong).unwrap();
    }
    // The config names its motor in [parts]; that brings the catalogue values back.
    assert_eq!(cfg.get("parts.motor").unwrap().as_str(), Some("1FK2205-4AF"));
    let mut cfg = sim_components::catalog::resolve(&machine_t_gantry::ENTRY, &cfg).unwrap();
    for path in ["motor.rotor_inertia_kgm2", "motor.torque_constant_nm_a", "motor.max_current_a", "motor.rated_current_a", "motor.torque_constant_at_max_current_nm_a", "motor.thermal_time_constant_s", "drive.peak_torque_nm", "drive.rated_torque_nm", "drive.rated_speed_rad_s", "drive.no_load_speed_rad_s", "drive.bus_voltage_v", "ratings.braking_threshold_v", "ratings.dc_link_overvoltage_v"] {
        assert!(close(cfg.f64(path).unwrap(), by_hand.f64(path).unwrap(), 2e-3), "{path}: {} against {} by hand", cfg.f64(path).unwrap(), by_hand.f64(path).unwrap());
    }
    // The numbers of the file are the motor's own, so there is nothing to retune the velocity gain for.
    assert!(close(cfg.f64("drive.velocity_kp_nm_s_rad").unwrap(), 0.6, 1e-9));
    // The rated current is the catalogue's own, not rated torque over the static torque constant: for the
    // 1FK2204-5AK at 400 V 1.95 A, where 1.5 N m / (2.4 N m / 4.4 A) would be 2.75 A * 0.6 = 1.65 A. The one
    // row whose rated current is a misprint (3.75 A at a stall current of 2.8 A) stops 5 % above the stall current.
    for (article, supply, rated) in [("1FK2204-5AK", "400v", 1.95), ("1FT2206-2AC", "230v", 1.05 * 2.8)] {
        sim_components::catalog::apply_motor(&mut cfg, article, supply).unwrap();
        assert!(close(cfg.f64("motor.rated_current_a").unwrap(), rated, 1e-12), "{article}: {} A", cfg.f64("motor.rated_current_a").unwrap());
    }
    // Not offered at 230 V single-phase data, and an unknown article, are refused.
    assert!(sim_components::catalog::apply_motor(&mut cfg, "1FK2205-4AF", "110v").is_err());
    assert!(sim_components::catalog::apply_motor(&mut cfg, "1FK9999-0AA", "400v").is_err());
}

/// A gearbox named by size, motor frame and ratio sets the gearbox of the config from the catalogue row.
#[test]
fn a_gearbox_by_name_sets_ratio_stiffness_backlash_and_ratings() {
    let mut cfg = machine_t_gantry::ENTRY.config();
    let rotor = cfg.f64("motor.rotor_inertia_kgm2").unwrap();
    assert_eq!(cfg.f64("gearbox.inertia_at_input_kgm2").unwrap(), 0.0);
    sim_components::catalog::apply_gearbox(&mut cfg, "NRK070-04-10").unwrap();
    let row = csv("siemens/gearbox_nrk.csv").into_iter().find(|r| r["size"] == "NRK070" && r["motor_frame"] == "04" && r["ratio"] == "10").unwrap();
    let number = |column: &str| -> f64 { row[column].parse().unwrap() };
    let arcmin = PI / 180.0 / 60.0;
    assert_eq!(cfg.f64("gearbox.ratio").unwrap(), 10.0);
    assert!(close(cfg.f64("gearbox.stiffness_nm_rad").unwrap(), number("torsional_stiffness_nm_arcmin") / arcmin, 1e-9));
    assert!(close(cfg.f64("gearbox.backlash_rad").unwrap(), number("backlash_arcmin") * arcmin, 1e-9));
    assert_eq!(cfg.f64("gearbox.efficiency").unwrap(), number("efficiency_full_load"));
    // The gearbox's inertia has a key of its own and leaves the rotor's alone.
    assert!(close(cfg.f64("gearbox.inertia_at_input_kgm2").unwrap(), number("inertia_at_input_kgcm2") * 1e-4, 1e-9) && cfg.f64("motor.rotor_inertia_kgm2").unwrap() == rotor);
    assert_eq!(cfg.f64("ratings.gearbox_max_torque_nm").unwrap(), number("max_torque_nm"));
    assert!(sim_components::catalog::apply_gearbox(&mut cfg, "NRK070-04-11").is_err());
}

/// The search on the EXCT-30: the smallest motors cannot run the cycle, the motor the config was written
/// for can, and among the passing ones the lightest comes first.
#[test]
fn the_motor_search_ranks_passing_motors_by_weight_and_fails_the_small_ones() {
    let cfg = sim_core::Config::parse(machine_t_gantry::EXCT_30_CONFIG).unwrap();
    let wanted = |article: &str| ["1FK2102-1AG", "1FK2203-4AK", "1FK2205-2AF", "1FK2205-4AF", "1FK2206-2AF"].contains(&article);
    let found = sim_components::catalog::search_motors(&machine_t_gantry::ENTRY, &cfg, "400v", &wanted, None);
    let place = |article: &str| found.iter().position(|c| c.article == article);
    let by_hand = &found[place("1FK2205-4AF").unwrap()];
    assert!(by_hand.passes(), "{:?}", by_hand.tightest());
    // The 1FK2102 is offered at 230 V only, so it is not among the 400 V candidates at all.
    assert!(place("1FK2102-1AG").is_none());
    let small = &found[place("1FK2203-4AK").unwrap()];
    assert!(!small.passes(), "a 1.27 N m motor passes: {:?}", small.tightest());
    let passing: Vec<&sim_components::catalog::Candidate> = found.iter().take_while(|c| c.passes()).collect();
    assert!(!passing.is_empty() && passing.windows(2).all(|w| w[0].mass <= w[1].mass));
    assert!(found.iter().skip(passing.len()).all(|c| !c.passes()));
}

/// A motor and a gearbox named in `[parts]` give exactly the machine that the same config gives with the
/// numbers put in by `apply_gearbox` and `apply_motor` and the velocity gain retuned for them, when a
/// simulation starts and when a running one is given the config. An unknown article is refused, with a
/// message in every language.
#[test]
fn parts_named_in_a_config_behave_like_their_numbers() {
    use sim_components::catalog::{apply_gearbox, apply_motor, resolve};
    use sim_components::{Command, Sim};
    for entry in sim_machines::MACHINES {
        let mut named = entry.config();
        named.set_text("parts.motor = \"1FK2104-4AK\"").unwrap();
        named.set_text("parts.gearbox = \"NRK070-04-5\"").unwrap();
        let mut numbers = entry.config();
        apply_gearbox(&mut numbers, "NRK070-04-5").unwrap();
        apply_motor(&mut numbers, "1FK2104-4AK", "400v").unwrap();
        let resolved = resolve(entry, &named).unwrap();
        // The rotor is a little heavier than the config's and the gearbox adds its own inertia: more gain.
        let (gain, own) = (resolved.f64("drive.velocity_kp_nm_s_rad").unwrap(), numbers.f64("drive.velocity_kp_nm_s_rad").unwrap());
        assert!(gain > own && gain < 2.0 * own, "{}: gain {gain}", entry.id);
        numbers.set_text(&format!("drive.velocity_kp_nm_s_rad = {gain:?}")).unwrap();
        assert_eq!(resolved, numbers, "{}", entry.id);
        // Resolved once, a config names no part any more: doing it again changes nothing.
        assert_eq!(resolve(entry, &resolved).unwrap(), resolved, "{}", entry.id);
        assert_eq!(numbers.f64("gearbox.efficiency").unwrap(), 0.97);
        let cycle = |mut sim: Sim| {
            sim.command(Command::StartCycle).unwrap();
            sim.run(0.3, |_| {});
            sim.state.clone()
        };
        let started = cycle(Sim::new(entry, &numbers).unwrap());
        assert_eq!(cycle(Sim::new(entry, &named).unwrap()), started, "{}", entry.id);
        assert_ne!(cycle(Sim::new(entry, &entry.config()).unwrap()), started, "{}: the parts change nothing", entry.id);
        let reconfigured = |cfg: &sim_core::Config| {
            let mut running = Sim::new(entry, &entry.config()).unwrap();
            running.reconfigure(entry, cfg).unwrap();
            cycle(running)
        };
        assert_eq!(reconfigured(&named), reconfigured(&numbers), "{}", entry.id);

        named.set_text("parts.motor = \"1FK9999-0AA\"").unwrap();
        let refused = Sim::new(entry, &named).err().unwrap();
        assert_eq!((refused.key(), refused.detail()), ("error-config-part-motor", "1FK9999-0AA"));
        for lang in sim_machines::LANGUAGES {
            assert!(sim_machines::translations(lang, entry).has(refused.key()), "{lang}");
        }
    }
}

/// Every bound in a config names a value of that config and has it between its low and its high.
#[test]
fn the_bounds_of_every_config_bracket_their_values() {
    let mut configs: Vec<sim_core::Config> = sim_machines::MACHINES.iter().map(|entry| entry.config()).collect();
    configs.push(sim_core::Config::parse(machine_t_gantry::EXCT_30_CONFIG).unwrap());
    for cfg in configs {
        let bounds = sim_components::analysis::bounds(&cfg).unwrap();
        assert!(bounds.len() >= 14, "{} bounds", bounds.len());
        for (path, low, high) in bounds {
            let value = cfg.f64(&path).unwrap();
            assert!(low < high && low <= value && value <= high, "{path}: {value} is not within {low} to {high}");
        }
    }
}

/// Retuning: the 1FK2204-6AF has a third of the rotor inertia of the EXCT-30 config's motor. With the config's
/// velocity gain its loop oscillates and the winding overheats; with the gain scaled by the geometric mean of
/// the ratio of the inertia the motors turn and the ratio of the rotor inertias it runs the cycle with about
/// the torque of the config's own motor. The 1FK2104-5AF has a ninth of the rotor inertia: scaled with the
/// inertia the motors turn alone (the rule before) it still oscillates, with the mean it runs calmly too.
#[test]
fn the_search_retunes_the_velocity_loop_for_a_lighter_rotor() {
    use sim_components::catalog::{search_motors, with_parts};
    let entry = &machine_t_gantry::ENTRY;
    let cfg = sim_core::Config::parse(machine_t_gantry::EXCT_30_CONFIG).unwrap();
    let cycle = |cfg: &sim_core::Config| sim_components::analysis::analyse_cycle(&mut sim_components::Sim::new(entry, cfg).unwrap()).unwrap().motors[0].clone();
    let own = cycle(&cfg);
    // The motor's numbers in the config and the gain left as it is: what naming the motor used to give.
    let mut same_gain = cfg.clone();
    sim_components::catalog::apply_motor(&mut same_gain, "1FK2204-6AF", "400v").unwrap();
    same_gain.set_text("parts.motor = \"\"").unwrap();
    let retuned = with_parts(entry, &cfg, "1FK2204-6AF", "400v", "").unwrap();
    // Naming the motor in the config retunes like the search does, so it runs as calmly.
    let mut named = cfg.clone();
    named.set_text("parts.motor = \"1FK2204-6AF\"").unwrap();
    assert_eq!(sim_components::catalog::resolve(entry, &named).unwrap(), retuned);
    assert!(cycle(&named).winding_rise < 50.0, "{} K with the motor named", cycle(&named).winding_rise);
    // The gain by hand. Per unit of acceleration of X or Z each motor needs (J_rotor + J_pulley) / r for
    // itself and m r / 2 for the load, with 14.86 kg moving in X and 7.7 kg in Z (payload attached). Summed
    // over two axes and two motors: 4 (J_rotor + 2e-5) / r + 22.56 r, with r = 15.91 mm. The belt's own mass
    // adds a little to both sums.
    let seen = |rotor: f64| 4.0 * (rotor + 2.0e-5) / 0.01591 + 22.56 * 0.01591;
    let (gain, by_hand) = (retuned.f64("drive.velocity_kp_nm_s_rad").unwrap(), 0.6 * (seen(1.6e-4) / seen(5.1e-4) * 1.6 / 5.1).sqrt());
    assert!(close(gain, by_hand, 0.02), "gain {gain}, by hand {by_hand}");
    let (oscillating, calm) = (cycle(&same_gain), cycle(&retuned));
    assert!(oscillating.rms_torque > 2.0 * own.rms_torque && oscillating.winding_rise > 100.0, "{} N m, {} K", oscillating.rms_torque, oscillating.winding_rise);
    // The lighter rotor needs a little less torque than the config's own motor (1.74 N m RMS).
    assert!(calm.rms_torque > 0.7 * own.rms_torque && calm.rms_torque < 1.05 * own.rms_torque, "{} N m against {} N m", calm.rms_torque, own.rms_torque);
    assert!(calm.winding_rise < 50.0, "{} K", calm.winding_rise);
    // The lightest rotor that has the torque for this cycle. The rule before gave it 0.6 * 0.77 = 0.46.
    let lightest = with_parts(entry, &cfg, "1FK2104-5AF", "400v", "").unwrap();
    let mut total_only = lightest.clone();
    total_only.set_text(&format!("drive.velocity_kp_nm_s_rad = {:?}", 0.6 * seen(5.6e-5) / seen(5.1e-4))).unwrap();
    let (oscillating, calm) = (cycle(&total_only), cycle(&lightest));
    assert!(oscillating.rms_torque > 2.0 * own.rms_torque && oscillating.winding_rise > 100.0, "{} N m, {} K", oscillating.rms_torque, oscillating.winding_rise);
    assert!(calm.rms_torque > 0.7 * own.rms_torque && calm.rms_torque < 1.05 * own.rms_torque, "{} N m against {} N m", calm.rms_torque, own.rms_torque);
    assert!(calm.winding_rise < 50.0, "{} K", calm.winding_rise);
    // The search does the same, and this machine has no gearbox to exchange (ratio 1), so none is tried.
    let found = search_motors(entry, &cfg, "400v", &|article| article == "1FK2204-6AF", Some("NRK"));
    assert!(found.len() == 1 && found[0].passes() && found[0].gearbox.is_empty(), "{:?}", found[0].tightest());
}

/// The gearbox search on the single axis (ratio 5, 1 m/s over a 20 mm pulley radius: 50 rad/s at the pulley).
/// Two motors with the NRK070 gearboxes for their frame, ratios 3 to 10. The 1FK2104-4AF is rated for
/// 3000 r/min = 314 rad/s, so ratios 7 and above (350 rad/s) are ruled out without a simulation; the
/// 1FK2104-5AK for 6000 r/min, so none is. What was ruled out does run above its rated speed when it is
/// simulated all the same. (Its cycle may still pass the verdicts: above the rated speed the drive has torque
/// left as long as the DC link has voltage. The rated speed is the catalogue's limit, so it stays ruled out.)
#[test]
fn the_gearbox_search_rules_out_by_catalogue_numbers_only_what_would_fail() {
    use sim_components::analysis::analyse_cycle;
    use sim_components::catalog::{search_motors, with_parts};
    let entry = &machine_single_axis::ENTRY;
    let cfg = entry.config();
    let found = search_motors(entry, &cfg, "400v", &|article| ["1FK2104-4AF", "1FK2104-5AK"].contains(&article), Some("NRK070"));
    let ratios = ["3", "4", "5", "7", "8", "9", "10"];
    assert_eq!(found.len(), 2 * ratios.len());
    for c in &found {
        let ratio = c.gearbox.strip_prefix("NRK070-04-").unwrap_or_else(|| panic!("{}", c.gearbox));
        let fast = ratios.iter().position(|r| *r == ratio).unwrap() >= 3;
        assert_eq!(c.refused == "limit-motor-speed", c.article == "1FK2104-4AF" && fast, "{} {}", c.article, c.gearbox);
        assert_eq!(c.result.is_none(), c.refused == "limit-motor-speed");
        if c.result.is_none() {
            let with = with_parts(entry, &cfg, c.article, "400v", &c.gearbox).unwrap();
            let load = analyse_cycle(&mut sim_components::Sim::new(entry, &with).unwrap()).unwrap();
            let (reached, rated) = (load.motors[0].peak_speed, with.f64("drive.rated_speed_rad_s").unwrap());
            assert!(reached > rated, "{} {} was ruled out and stays at {reached} of {rated} rad/s", c.article, c.gearbox);
        }
    }
    // Passing ones first, lightest motor first (the gearbox files have no mass), then the rest.
    let passing = found.iter().take_while(|c| c.passes()).count();
    assert!(passing >= 2 && found[..passing].windows(2).all(|w| w[0].mass <= w[1].mass));
    assert!(found[passing..].iter().all(|c| !c.passes()));
}

/// A motor is applied with the DC link of its supply, so the search on a default config (written for a DC
/// link of 155 V, 190 V at the winding) judges the catalogue's 400 V motors on 540 V. Before, most of them
/// met their own back-EMF, never reached their targets and failed "settling" five hundred to two thousand times over. Now no candidate
/// fails settling, and what fails does so for what the motor is: too slow for the axis, or too much rotor
/// to brake for the config's braking resistor.
#[test]
fn the_search_on_a_default_config_fails_motors_for_physical_reasons() {
    use sim_components::catalog::{apply_motor, search_motors};
    let physical = ["motor-peak", "motor-thermal", "gearbox-peak", "gearbox-continuous", "braking-resistor-continuous", "braking-resistor-peak"];
    for entry in sim_machines::MACHINES {
        let mut cfg = entry.config();
        assert_eq!(cfg.f64("drive.bus_voltage_v").unwrap(), 155.13);
        // The braking threshold and the trip level keep their distance to the bus: 224.65 V and 257.6 V at 155.13 V.
        for (supply, bus) in [("230v", 325.0), ("400v", 540.0)] {
            apply_motor(&mut cfg, "1FK2104-5AK", supply).unwrap();
            assert_eq!(cfg.f64("drive.bus_voltage_v").unwrap(), bus, "{supply}");
            assert!(close(cfg.f64("ratings.braking_threshold_v").unwrap(), bus * 224.65 / 155.13, 1e-12), "{supply}");
            assert!(close(cfg.f64("ratings.dc_link_overvoltage_v").unwrap(), bus * 257.6 / 155.13, 1e-12), "{supply}");
        }
        // So the verdicts of a default config with a catalogue motor have a DC link with room in it, and
        // the simulated DC link takes the config. (The 250 V threshold of the 190 V placeholder drive was
        // below the motor's 540 V and gave a negative limit that passed.)
        let mut named = entry.config();
        named.set_text("parts.motor = \"1FK2104-5AK\"").unwrap();
        named.set_text("effects.dc_link = true").unwrap();
        let load = sim_components::analysis::analyse_cycle(&mut sim_components::Sim::new(entry, &named).unwrap()).unwrap();
        let link = sim_components::analysis::verdicts(&named, &load).unwrap().into_iter().find(|v| v.check == "dc-link-energy").unwrap();
        assert!(link.limit > 0.0 && link.value <= link.limit && link.ok, "{}: {link:?}", entry.id);
        // A threshold below the bus is refused, simulated or not.
        named.set_text("effects.dc_link = false").unwrap();
        named.set_text("parts.motor = \"\"").unwrap();
        named.set_text("drive.bus_voltage_v = 300.0").unwrap();
        assert_eq!(sim_components::analysis::verdicts(&named, &load).unwrap_err().detail(), "ratings.braking_threshold_v", "{}", entry.id);
        let wanted = |article: &str| ["1FK2104-5AF", "1FK2205-4AF", "1FK2206-4AF", "1FK2208-3AC"].contains(&article);
        let found = search_motors(entry, &entry.config(), "400v", &wanted, None);
        assert_eq!(found.len(), 4, "{}", entry.id);
        for c in &found {
            // Rated for 2000 r/min: too slow for either machine behind its 5:1 gearbox.
            assert_eq!(c.refused == "limit-motor-speed", c.article == "1FK2208-3AC", "{} {}: {}", entry.id, c.article, c.refused);
            let Some((load, verdicts)) = &c.result else { continue };
            let settling = verdicts.iter().find(|v| v.check == "settling").unwrap();
            assert!(settling.ok && settling.value < 0.6 * settling.limit, "{} {}: {} m from the target", entry.id, c.article, settling.value);
            for v in verdicts.iter().filter(|v| !v.ok) {
                assert!(physical.contains(&v.check), "{} {} fails {}", entry.id, c.article, v.check);
            }
            // The current loop is calm on the higher bus with the gains the config has.
            assert!(load.motors.iter().all(|m| m.winding_rise < 50.0), "{} {}", entry.id, c.article);
        }
        // The rotor of the lightest of them was 1500 times the settling tolerance away before.
        assert!(found.iter().find(|c| c.article == "1FK2104-5AF").unwrap().passes(), "{}", entry.id);
    }
}

/// Every motor of the S210 catalogue reaches its rated speed at its rated torque inside the voltage limit
/// that `apply_motor` gives it. The back-EMF there is the static torque constant times the rated speed (the
/// catalogue has no winding resistance, so the few volts of R i are not counted). The winding gets
/// sqrt(3/2) times the DC link voltage; with the DC link voltage itself as the limit, five rows (four
/// motors of 2000 r/min and one of 1500 r/min at 400 V, back-EMF 554 V and 558 V against 540 V) could not.
#[test]
fn every_catalogue_motor_reaches_its_rated_point_inside_the_voltage_limit() {
    use sim_components::drive::DriveParams;
    let (mut rows, mut beyond_the_dc_link) = (0, 0);
    for m in csv("siemens/motors_s210.csv") {
        for (supply, bus) in [("230v", 325.0), ("400v", 540.0)] {
            if m[&format!("rated_speed_{supply}_rpm")].is_empty() {
                continue;
            }
            let mut cfg = machine_single_axis::ENTRY.config();
            sim_components::catalog::apply_motor(&mut cfg, &m["article"], supply).unwrap();
            let p = DriveParams::from_config(&cfg, 62.5e-6, 125e-6).unwrap();
            assert_eq!(p.bus_voltage, bus);
            let back_emf = p.back_emf(0.0, p.rated_speed);
            assert!(back_emf < p.max_voltage, "{} at {supply}: {back_emf} V of back-EMF, {} V at the winding", m["article"], p.max_voltage);
            // The torque-speed curve agrees: full torque at the rated speed, and none where the voltage is used up.
            assert!(p.torque_curve(p.rated_speed).1 == p.peak_torque && p.no_load_speed >= 1.1 * p.rated_speed);
            rows += 1;
            beyond_the_dc_link += (back_emf > bus) as usize;
        }
    }
    assert!(rows > 120 && beyond_the_dc_link == 5, "{rows} rows, {beyond_the_dc_link} beyond the DC link voltage");
}

/// A gearbox named for the whole search and the same gearbox found by the search give the same candidate.
/// The first route puts the gearbox into the config before the search puts each motor in; the gearbox's
/// inertia used to be added to the rotor's, where the motor then overwrote it (0.5581 N m of peak torque
/// against 0.5873 N m for the 1FK2104-5AF behind an NRK070-04-5 on the single axis).
#[test]
fn a_gearbox_named_before_the_search_keeps_its_inertia() {
    use sim_components::catalog::{resolve, with_parts};
    let entry = &machine_single_axis::ENTRY;
    let mut named = entry.config();
    named.set_text("parts.gearbox = \"NRK070-04-5\"").unwrap();
    // As the command line does: the config's parts first, then the search's motor.
    let before = with_parts(entry, &resolve(entry, &named).unwrap(), "1FK2104-5AF", "400v", "").unwrap();
    let by_search = with_parts(entry, &entry.config(), "1FK2104-5AF", "400v", "NRK070-04-5").unwrap();
    let inertia = |cfg: &sim_core::Config| cfg.f64("motor.rotor_inertia_kgm2").unwrap() + cfg.f64("gearbox.inertia_at_input_kgm2").unwrap();
    // Rotor 0.56 kg cm2 and gearbox 0.135 kg cm2 (catalog/siemens/gearbox_nrk.csv).
    let row = csv("siemens/gearbox_nrk.csv").into_iter().find(|r| r["size"] == "NRK070" && r["motor_frame"] == "04" && r["ratio"] == "5").unwrap();
    let by_hand = (0.56 + row["inertia_at_input_kgcm2"].parse::<f64>().unwrap()) * 1e-4;
    assert!(close(inertia(&before), by_hand, 1e-12) && close(inertia(&by_search), by_hand, 1e-12), "{} and {} kg m2", inertia(&before), inertia(&by_search));
    for ((path, a), (_, b)) in before.leaves().into_iter().zip(by_search.leaves()) {
        let same = match (a.as_float(), b.as_float()) {
            (Some(a), Some(b)) => close(a, b, 1e-12) || a == b,
            _ => a == b,
        };
        assert!(same, "{path}: {a} with the gearbox named before, {b} with the search's");
    }
    let peak = |cfg: &sim_core::Config| sim_components::analysis::analyse_cycle(&mut sim_components::Sim::new(entry, cfg).unwrap()).unwrap().motors[0].peak_torque;
    assert!(close(peak(&before), peak(&by_search), 1e-6), "{} and {} N m", peak(&before), peak(&by_search));
}

/// The search's "tightest check" leaves out a DC link that is full by design. With a braking resistor the
/// simulated DC link fills to the chopper's threshold in every cycle that brakes hard enough, so its energy
/// stood at 1.0000 for every passing motor (and its voltage at threshold over trip level, 0.939 on the
/// EXCT-30) and hid the check that is really the closest.
#[test]
fn the_tightest_check_leaves_out_a_dc_link_that_a_resistor_backs() {
    use sim_components::analysis::{CycleLoad, Verdict};
    use sim_components::catalog::Candidate;
    let verdict = |check: &'static str, value: f64, limit: f64, ok: bool| Verdict { check, motor: Some(0), value, limit, ok };
    let load = CycleLoad { duration: 1.0, belt_tension: None, belt_slack_time: None, radial_load_peak: None, guide_load: None, path_error_peak: 0.0, path_error_rms: 0.0, tracking_error_peak: 0.0, dwell_error_peak: 0.0, tripped: false, motors: Vec::new() };
    let candidate = |verdicts: Vec<Verdict>| Candidate { article: "", gearbox: String::new(), mass: 1.0, result: Some((load.clone(), verdicts)), refused: "" };
    let full = verdict("dc-link-energy", 6.2, 6.2, true);
    let with_resistor = candidate(vec![verdict("motor-peak", 0.7, 1.0, true), full.clone(), verdict("braking-resistor-continuous", 6.0, 20.0, true), verdict("dc-link-overvoltage", 770.0, 820.0, true)]);
    assert_eq!(with_resistor.tightest(), Some(("motor-peak", 0.7)));
    // Without a resistor a full DC link is the limit, and one that fails is the reason. The voltage, which
    // starts at the bus level, says nothing while it passes.
    assert_eq!(candidate(vec![verdict("motor-peak", 0.7, 1.0, true), verdict("dc-link-energy", 5.0, 6.2, true), verdict("dc-link-overvoltage", 760.0, 820.0, true)]).tightest().unwrap().0, "dc-link-energy");
    assert_eq!(candidate(vec![verdict("motor-peak", 0.7, 1.0, true), verdict("dc-link-overvoltage", 830.0, 820.0, false)]).tightest().unwrap().0, "dc-link-overvoltage");
    assert_eq!(candidate(vec![verdict("motor-peak", 0.7, 1.0, true), verdict("dc-link-energy", 9.3, 6.2, false), verdict("braking-resistor-peak", 1.0, 2.0, true)]).tightest(), Some(("dc-link-energy", 1.5)));
}

/// The EXCT-30 config passes every check with margin: in the worst case of both band modes. One at a time,
/// each of its 61 bounded values at its low and at its high bound (123 cycles); and all of them between their
/// bounds at once, 60 times. Its belt pretension, X jerk limit, input shaper and corner blending were chosen for
/// this (see the comments in the file): the slackest belt section keeps more than 100 N, the pulley shafts
/// carry less than the 1500 N of the lowest shaft rating the file allows for, and the tool is within 0.07 mm
/// in the second half of every dwell, against a tolerance of 0.1 mm.
#[test]
fn the_exct_30_config_passes_every_check_in_the_worst_case_of_both_band_modes() {
    use sim_components::analysis::{Band, analyse_cycle, bounds, verdict_bands, verdict_bands_sampled};
    let cfg = sim_components::load_config(&machine_t_gantry::ENTRY, machine_t_gantry::EXCT_30_CONFIG).unwrap();
    let bounded = bounds(&cfg).unwrap();
    assert!(bounded.len() >= 60 && bounded.contains(&("belt.pretension_n".to_string(), 350.0, 550.0)));
    let band = |bands: &[Band], check: &str| bands.iter().find(|b| b.verdict.check == check).unwrap().clone();
    for (mode, bands) in [("one at a time", verdict_bands(&machine_t_gantry::ENTRY, &cfg, &bounded).unwrap()), ("sampled", verdict_bands_sampled(&machine_t_gantry::ENTRY, &cfg, &bounded, 60, 1).unwrap())] {
        let failed: Vec<&Band> = bands.iter().filter(|b| !b.ok || b.share < 1.0).collect();
        assert!(failed.is_empty(), "{mode}: {failed:?}");
        let (slack, settling, radial) = (band(&bands, "belt-slack"), band(&bands, "settling"), band(&bands, "gearbox-radial-load"));
        assert!(slack.low > 100.0 && settling.high < 0.7e-4 && radial.high < 1500.0, "{mode}: slackest {} N, settling {} m, radial {} N", slack.low, settling.high, radial.high);
        // The highest belt tension is judged too, against the estimate of 2140 N (a 5 mm pitch belt as stiff as Festo's), and passes in the worst case.
        let tension = band(&bands, "belt-tension");
        assert!(tension.verdict.limit == 2140.0 && tension.ok && tension.share == 1.0, "{mode}: {tension:?}");
        println!("{mode}: slackest {:.1} N, settling {:.2e} m, radial load {:.0} N, tightest {:.0} N", slack.low, settling.high, radial.high, tension.high);
    }
    let cycle = analyse_cycle(&mut sim_components::Sim::new(&machine_t_gantry::ENTRY, &cfg).unwrap()).unwrap().duration;
    assert!((cycle - 1.934).abs() < 0.01, "cycle {cycle} s");
}
