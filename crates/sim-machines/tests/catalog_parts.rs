//! Drives, belts and axes by name, the S120 motors, gearbox masses, and the searches over them: each against the
//! catalogue rows or a hand calculation.

use sim_components::catalog::{
    Candidate, apply_belt, apply_drive, apply_motor, paired_drive, resolve, search_axes, search_motors,
};
use sim_core::Config;
use std::path::Path;

fn close(a: f64, b: f64, tolerance: f64) -> bool {
    (a - b).abs() <= tolerance * b.abs()
}

fn exct_30() -> Config {
    Config::parse(machine_t_gantry::EXCT_30_CONFIG).unwrap()
}

fn csv(file: &str) -> Vec<std::collections::HashMap<String, String>> {
    let text = std::fs::read_to_string(Path::new(env!("CARGO_MANIFEST_DIR")).join("../../catalog").join(file)).unwrap();
    let mut lines = text.lines().filter(|l| !l.starts_with('#'));
    let header: Vec<String> = lines.next().unwrap().split(',').map(String::from).collect();
    lines.map(|l| header.iter().cloned().zip(l.split(',').map(String::from)).collect()).collect()
}

/// `axis = "EXCT-30"` on the EXCT-30 config gives the numbers the file was written with by hand from Festo's catalogue
/// and Megadyne's AT5 table: pulley 31.82 mm / 2, the Z axis 4.7 kg of the formula for 250 mm, Festo's EA (300 N at
/// 0.045 %), the AT5 belt at 30 mm (2140 N between 1805 N at 25 mm and 2275 N at 32 mm; 97.86 g/m between 80 and 105),
/// 20 pulley teeth of which 10 in mesh, 12.5 N m at the pulley, and a carriage fitted to Festo's torque formula.
#[test]
fn an_axis_by_name_gives_the_numbers_of_the_hand_written_exct_30() {
    let entry = &machine_t_gantry::ENTRY;
    let by_hand = exct_30();
    let mut named = by_hand.clone();
    for wrong in [
        "geometry.pulley_radius_m = 0.03",
        "mass.carriage_kg = 2.0",
        "mass.z_axis_kg = 1.0",
        "belt.stiffness_ea_n = 1.0e5",
        "ratings.belt_max_tension_n = 100.0",
        "ratings.belt_teeth_in_mesh = 3.0",
    ] {
        named.set_text(wrong).unwrap();
    }
    named.set_text("parts.axis = \"EXCT-30\"").unwrap();
    let resolved = resolve(entry, &named).unwrap();
    for (path, tolerance) in [
        ("geometry.pulley_radius_m", 1e-3),
        ("geometry.stroke_x_m", 0.0),
        ("geometry.stroke_z_m", 0.0),
        ("mass.z_axis_kg", 0.0),
        ("belt.stiffness_ea_n", 1e-3),
        ("belt.tooth_pitch_m", 1e-12),
        ("ratings.belt_max_tension_n", 1e-3),
        ("ratings.belt_width_mm", 0.0),
        ("ratings.belt_teeth_in_mesh", 0.0),
        ("ratings.pulley_max_torque_nm", 0.0),
        ("drive.peak_torque_nm", 0.0),
        // Fitted again from the same formula and machine: 7.136 kg. The file's 7.16 kg was fitted by hand, to the 2 %
        // within which `exct_30_config_agrees_with_the_catalogue` holds the torque.
        ("mass.carriage_kg", 5e-3),
    ] {
        let (got, want) = (resolved.f64(path).unwrap(), by_hand.f64(path).unwrap());
        assert!(close(got, want, tolerance), "{path}: {got} against {want} by hand");
    }
    assert!(close(resolved.f64("belt.mass_kg_m").unwrap(), (80.0 + 25.0 * 5.0 / 7.0) * 1e-3, 1e-9));
    assert_eq!(
        resolved.f64s("ratings.belt_tooth_load_n_cm").unwrap(),
        by_hand.f64s("ratings.belt_tooth_load_n_cm").unwrap()
    );
    assert_eq!(
        resolved.f64s("ratings.belt_tooth_load_rpm").unwrap(),
        by_hand.f64s("ratings.belt_tooth_load_rpm").unwrap()
    );
    // The machine of the resolved config needs Festo's torque at 45 degrees, payload included: M = a (11.3 m_L +
    // 8.89 J_m + 99.1) 1e-3 + 0.08 (4.7 + m_L), J_m = 5.1 kg cm2.
    for (a, load) in [(5.0, 0.0), (25.0, 3.0)] {
        let mut cfg = resolved.clone();
        cfg.set_text(&format!("mass.payload_kg = {load:?}")).unwrap();
        let machine = (entry.build)(&cfg).unwrap();
        let mut torque = [0.0; 2];
        machine.feedforward_torque(&[0.5, 0.1], &[a / 2f64.sqrt(), a / 2f64.sqrt()], &mut torque);
        let festo = a * (11.3 * load + 8.89 * 5.1 + 99.1) * 1e-3 + 0.08 * (4.7 + load);
        assert!(close(torque[0], festo, 0.02), "a = {a}, load = {load}: {} against Festo's {festo}", torque[0]);
    }
}

/// Bahr's X/Z gantry ELZI 40 on the EXCT-30 config: carriage 7.0 - 2.4 kg and Z axis 2.4 kg (the sheet's moved
/// masses), pulley 160 mm per turn / 2 pi, belt 5M25 as Megadyne MTD5 at 25 mm (1805 N at 4 mm/m: EA 451250 N), 32
/// teeth of which 16 in mesh, 1000 N of peak feed force times the radius at the pulley. Bahr's single axis ELZ 60 on
/// the single axis: its own pulley (130 mm per turn) and belt, the carriage as the config has it (the sheet states
/// none). An axis for the other kind of machine, a stroke the axis does not have and an unknown name are refused.
#[test]
fn bahr_axes_by_name_bring_the_sheets_moved_masses_pulley_and_belt() {
    let entry = &machine_t_gantry::ENTRY;
    let mut cfg = exct_30();
    cfg.set_text("parts.axis = \"ELZI-40\"").unwrap();
    let r = resolve(entry, &cfg).unwrap();
    let radius = 0.160 / (2.0 * std::f64::consts::PI);
    for (path, want) in [
        ("mass.carriage_kg", 4.6),
        ("mass.z_axis_kg", 2.4),
        ("geometry.pulley_radius_m", radius),
        ("belt.stiffness_ea_n", 451250.0),
        ("belt.mass_kg_m", 0.08),
        ("belt.tooth_pitch_m", 0.005),
        ("ratings.belt_max_tension_n", 1805.0),
        ("ratings.belt_width_mm", 25.0),
        ("ratings.belt_teeth_in_mesh", 16.0),
        ("ratings.pulley_max_torque_nm", 1000.0 * radius),
    ] {
        assert!(close(r.f64(path).unwrap(), want, 1e-9), "{path}: {} against {want}", r.f64(path).unwrap());
    }
    // The 1FK2205-4AF could give 18 N m; the ELZI 40's pulley allows 25.5 N m, so the motor's own maximum holds now.
    assert!(close(r.f64("drive.peak_torque_nm").unwrap(), 18.0, 1e-9));

    let single = &machine_single_axis::ENTRY;
    let mut cfg = single.config();
    cfg.set_text("parts.axis = \"ELZ-60\"").unwrap();
    let r = resolve(single, &cfg).unwrap();
    assert!(close(r.f64("geometry.pulley_radius_m").unwrap(), 0.130 / (2.0 * std::f64::consts::PI), 1e-12));
    assert_eq!((r.f64("mass.carriage_kg").unwrap(), r.f64("ratings.belt_max_tension_n").unwrap()), (4.0, 1805.0));

    let refused = |entry, cfg: &Config, axis: &str| {
        resolve(entry, &{
            let mut c = cfg.clone();
            c.set_text(&format!("parts.axis = {axis:?}")).unwrap();
            c
        })
        .err()
        .map(|e| (e.key(), e.detail().to_string()))
    };
    assert_eq!(refused(single, &single.config(), "ELZI-40"), Some(("error-config-invalid", "parts.axis".into())));
    assert_eq!(refused(entry, &exct_30(), "ELZ-60"), Some(("error-config-invalid", "parts.axis".into())));
    // The EXCT-15 has Z strokes of 100 and 200 mm; the config's is 250 mm.
    assert_eq!(refused(entry, &exct_30(), "EXCT-15"), Some(("error-config-invalid", "geometry.stroke_x_m".into())));
    assert_eq!(refused(entry, &exct_30(), "EXCT-99"), Some(("error-config-part-axis", "EXCT-99".into())));
    for lang in sim_machines::LANGUAGES {
        for key in ["error-config-part-axis", "error-config-part-belt", "error-config-part-drive"] {
            assert!(sim_machines::translations(lang, entry).has(key), "{lang}: {key}");
        }
    }
}

/// A belt by profile and width: a standard width is the handbook's row, one between two is linear between them,
/// one below the narrowest in proportion to it. EA is the allowable pull over the stretch at it (4 mm/m).
#[test]
fn a_belt_by_name_is_the_megadyne_row_or_between_two() {
    let mut cfg = exct_30();
    for (name, pull, mass, pitch) in [
        ("AT5-25", 1805.0, 0.080, 0.005),
        // MTD8 at 12 mm: between 10 mm (1425 N, 65 g/m) and 15 mm (2135 N, 85 g/m), two fifths of the way.
        ("MTD8-12", 1425.0 + 0.4 * 710.0, 0.065 + 0.4 * 0.020, 0.008),
        // AT10 at 10 mm: its narrowest is 16 mm (2270 N, 90 g/m).
        ("AT10-10", 2270.0 * 10.0 / 16.0, 0.090 * 10.0 / 16.0, 0.010),
    ] {
        apply_belt(&mut cfg, name).unwrap();
        assert!(close(cfg.f64("ratings.belt_max_tension_n").unwrap(), pull, 1e-9), "{name}");
        assert!(close(cfg.f64("belt.stiffness_ea_n").unwrap(), pull / 0.004, 1e-9), "{name}");
        assert!(close(cfg.f64("belt.mass_kg_m").unwrap(), mass, 1e-9), "{name}");
        assert_eq!(cfg.f64("belt.tooth_pitch_m").unwrap(), pitch, "{name}");
    }
    // The AT10's tooth load table: 83 N/cm at standstill, 22 at 8000 r/min.
    let teeth = cfg.f64s("ratings.belt_tooth_load_n_cm").unwrap();
    assert_eq!((teeth[0], teeth[teeth.len() - 1], teeth.len()), (83.0, 22.0, 18));
    for wrong in ["AT5", "AT7-25", "AT5-x", "AT5--3"] {
        assert_eq!(apply_belt(&mut cfg, wrong).err().map(|e| e.key()), Some("error-config-part-belt"), "{wrong}");
    }
}

/// A drive by name sets its rated and maximum output current; the cycle's RMS and peak motor current are judged
/// against them. By hand: an RMS current of 3.2 A against the 5 A of the S210 converter 6SL3210-5HE11-5UF0 is 0.64
/// of it, a peak of 12.4 A against its 19 A 0.653; against the 1.2 A and 4.2 A of the smallest one, 6SL3210-5HE10-4UF0,
/// both fail. The Motor Module 6SL3120-1TE15-0AD0 has 5 A and 15 A.
#[test]
fn a_drive_by_name_judges_the_motor_current_at_the_hand_calculated_share() {
    let entry = &machine_t_gantry::ENTRY;
    let mut cfg = exct_30();
    apply_drive(&mut cfg, "6SL3120-1TE15-0AD0", "s120").unwrap();
    assert_eq!(
        (cfg.f64("ratings.drive_rated_current_a").unwrap(), cfg.f64("ratings.drive_max_current_a").unwrap()),
        (5.0, 15.0)
    );
    assert_eq!(
        apply_drive(&mut cfg, "6SL3210-0XX00-0XX0", "400v").err().map(|e| e.key()),
        Some("error-config-part-drive")
    );
    // The config names its converter; resolved, its currents are in the ratings.
    let resolved = resolve(entry, &exct_30()).unwrap();
    assert_eq!(
        (resolved.f64("ratings.drive_rated_current_a").unwrap(), resolved.f64("ratings.drive_max_current_a").unwrap()),
        (5.0, 19.0)
    );
    let mut load =
        sim_components::analysis::analyse_cycle(&mut sim_components::Sim::new(entry, &resolved).unwrap()).unwrap();
    // The checks read the cycle's own currents.
    let judged = sim_components::analysis::verdicts(&resolved, &load).unwrap();
    let find = |verdicts: &[sim_components::analysis::Verdict], check: &str, motor: usize| {
        verdicts.iter().find(|v| v.check == check && v.motor == Some(motor)).cloned().unwrap()
    };
    assert_eq!(find(&judged, "drive-current", 1).value, load.motors[1].rms_current);
    assert!(load.motors[0].peak_current > load.motors[0].rms_current && find(&judged, "drive-peak-current", 0).ok);
    (load.motors[0].rms_current, load.motors[0].peak_current) = (3.2, 12.4);
    let judged = sim_components::analysis::verdicts(&resolved, &load).unwrap();
    let (rms, peak) = (find(&judged, "drive-current", 0), find(&judged, "drive-peak-current", 0));
    assert!(
        rms.ok
            && peak.ok
            && close(rms.value / rms.limit, 0.64, 1e-12)
            && close(peak.value / peak.limit, 12.4 / 19.0, 1e-12)
    );
    let mut small = resolved.clone();
    apply_drive(&mut small, "6SL3210-5HE10-4UF0", "400v").unwrap();
    let judged = sim_components::analysis::verdicts(&small, &load).unwrap();
    assert!(!find(&judged, "drive-current", 0).ok && !find(&judged, "drive-peak-current", 0).ok);
    assert!(
        close(find(&judged, "drive-current", 0).limit, 1.2, 1e-12)
            && close(find(&judged, "drive-peak-current", 0).limit, 4.2, 1e-12)
    );
}

/// A drive by name brings its DC link data where Siemens publishes it. The S210's internal braking resistor and trip level
/// from its operating instructions (Table 4-4, fault F07432): the EXCT-30's 6SL3210-5HE11-5UF0 has 100 W and 5.7 kW and
/// trips at 820 V; its chopper threshold, which Siemens does not publish, stays the config's 770 V. A 1 AC converter,
/// 6SL3210-5HB10-8UF0: 40 W, 2.4 kW, 410 V; a threshold of 470 V (at or above the trip) becomes
/// (1.1 * 240 * sqrt 2 + 410) / 2 = 391.676 V, one below it stays. A 3 AC converter on 230 V must use an external
/// resistor: the config's powers stay. An S120 Motor Module brings its DC link capacitance: 110 uF for the Single
/// 6SL3120-1TE15-0AD0, 220 / 2 = 110 uF per axis for the Double 6SL3120-2TE13-0AD0; and, having no resistor of its
/// own, the Braking Module's (braking_s120.csv: 1.5 kW, 100 kW, on at 770 V) and the booksize trip at 820 V, not the
/// S210 converter's 100 W and 5.7 kW the EXCT-30 config has.
#[test]
fn a_drive_by_name_brings_its_published_dc_link_data() {
    // That every S210 converter has its braking row is checked by the catalogue's validator
    // (`siemens_s210_braking_rows_are_the_s210_converters` in catalog/tools/cataloglib/checks.py).
    let rating = |cfg: &Config, leaf: &str| cfg.f64(&format!("ratings.{leaf}")).unwrap();
    let exct = resolve(&machine_t_gantry::ENTRY, &exct_30()).unwrap();
    let values = |cfg: &Config| {
        ["braking_resistor_continuous_w", "braking_resistor_peak_w", "dc_link_overvoltage_v", "braking_threshold_v"]
            .map(|leaf| rating(cfg, leaf))
    };
    assert_eq!(values(&exct), [100.0, 5700.0, 820.0, 770.0]);
    let mut cfg = exct_30();
    cfg.set_text("ratings.braking_threshold_v = 470.0").unwrap();
    apply_drive(&mut cfg, "6SL3210-5HB10-8UF0", "230v").unwrap();
    let got = values(&cfg);
    assert_eq!(got[..3], [40.0, 2400.0, 410.0]);
    assert!((got[3] - 391.676).abs() < 1e-3, "{}", got[3]);
    cfg.set_text("ratings.braking_threshold_v = 400.0").unwrap();
    apply_drive(&mut cfg, "6SL3210-5HB10-8UF0", "230v").unwrap();
    assert_eq!(rating(&cfg, "braking_threshold_v"), 400.0);
    let mut cfg = exct_30();
    cfg.set_text("ratings.braking_resistor_peak_w = 1234.0").unwrap();
    apply_drive(&mut cfg, "6SL3210-5HE11-5UF0", "230v").unwrap();
    assert_eq!((rating(&cfg, "braking_resistor_peak_w"), rating(&cfg, "dc_link_overvoltage_v")), (1234.0, 820.0));
    for (module, uf) in [("6SL3120-1TE15-0AD0", 110.0), ("6SL3120-2TE13-0AD0", 110.0)] {
        let mut cfg = exct_30();
        cfg.set_text("ratings.dc_link_capacitance_uf = 1.0").unwrap();
        apply_drive(&mut cfg, module, "s120").unwrap();
        assert_eq!(rating(&cfg, "dc_link_capacitance_uf"), uf, "{module}");
        assert_eq!(values(&cfg), [1500.0, 100_000.0, 820.0, 770.0], "{module}");
    }
    // A capacitance measured on a real drive (dc_link_measured.csv): 139 uF for the 6SL5310-1BE10-8DF0. A converter
    // without a measurement or a published value keeps the config's, as the EXCT-30's 6SL3210-5HE11-5UF0 its 66 uF.
    let mut cfg = exct_30();
    apply_drive(&mut cfg, "6SL5310-1BE10-8DF0", "400v").unwrap();
    assert_eq!(rating(&cfg, "dc_link_capacitance_uf"), 139.0);
    assert_eq!(rating(&exct, "dc_link_capacitance_uf"), 66.0);
}

/// An S120 line-up brakes through one Braking Module (1.5 kW continuous, 100 kW peak) for all its axes, so their
/// braking power is judged together. Two axes each feeding back 900 J in a 1 s cycle, both at 60 kW at the same
/// moment, each more than its DC link takes: by hand 900 + 900 = 1800 W mean against 1500 W and 60 + 60 = 120 kW
/// peak against 100 kW, both FAIL, where each axis alone (900 W, 60 kW) passed before. An S210 has a resistor of its
/// own: the same load on the EXCT-30's 6SL3210-5HE11-5UF0 is judged per axis against 100 W and 5.7 kW. And a run:
/// the peak of the summed signal lies between the highest axis' peak and the sum of the peaks.
#[test]
fn an_s120_line_up_is_judged_against_its_one_braking_module() {
    use sim_components::analysis::{CycleLoad, MotorLoad, verdicts};
    let axis = MotorLoad {
        energy_regenerated: 900.0,
        peak_regenerated_power: 60e3,
        largest_regeneration: 900.0,
        ..Default::default()
    };
    let load = CycleLoad {
        duration: 1.0,
        belt_tension: None,
        belt_slack_time: None,
        belt_tooth_jumps: None,
        belt_span_detuning: None,
        radial_load_peak: None,
        condition_peak: None,
        guide_load: None,
        guide_speed: None,
        path_error_peak: 0.0,
        path_error_rms: 0.0,
        tracking_error_peak: 0.0,
        dwell_error_peak: 0.0,
        path_angle_error_peak: 0.0,
        path_angle_error_rms: 0.0,
        tracking_angle_error_peak: 0.0,
        dwell_angle_error_peak: 0.0,
        tripped: false,
        motors: vec![axis.clone(), axis],
        budget: None,
        part_rises: Vec::new(),
        checks: Vec::new(),
        clearance: None,
        contact: None,
        braking_power_peak: 120e3,
        warm_start: None,
        conveyor: None,
    };
    let braking = |cfg: &Config| -> Vec<(&'static str, Option<usize>, f64, f64, bool)> {
        verdicts(cfg, &load)
            .unwrap()
            .into_iter()
            .filter(|v| v.check.starts_with("braking-resistor"))
            .map(|v| (v.check, v.motor, v.value, v.limit, v.ok))
            .collect()
    };
    let mut cfg = exct_30();
    cfg.set_text("effects.dc_link = false").unwrap();
    cfg.set_text("parts.drive = \"6SL3120-2TE13-0AD0\"").unwrap();
    let s120 = resolve(&machine_t_gantry::ENTRY, &cfg).unwrap();
    assert_eq!(
        braking(&s120),
        [
            ("braking-resistor-continuous", None, 1800.0, 1500.0, false),
            ("braking-resistor-peak", None, 120e3, 100e3, false)
        ]
    );
    let mut s210 = resolve(&machine_t_gantry::ENTRY, &exct_30()).unwrap();
    s210.set_text("effects.dc_link = false").unwrap();
    let per_axis: Vec<_> = (0..2)
        .flat_map(|k| {
            [
                ("braking-resistor-continuous", Some(k), 900.0, 100.0, false),
                ("braking-resistor-peak", Some(k), 60e3, 5700.0, false),
            ]
        })
        .collect();
    assert_eq!(braking(&s210), per_axis);
    let run = sim_components::analysis::analyse_cycle(
        &mut sim_components::Sim::new(&machine_t_gantry::ENTRY, &s120).unwrap(),
    )
    .unwrap();
    let peaks: Vec<f64> = run.motors.iter().map(|m| m.peak_regenerated_power).collect();
    let (highest, sum) = (peaks.iter().cloned().fold(0.0, f64::max), peaks.iter().sum::<f64>());
    assert!(
        highest > 0.0 && run.braking_power_peak >= highest && run.braking_power_peak <= sum * (1.0 + 1e-12),
        "{} W, axes {peaks:?}",
        run.braking_power_peak
    );
}

/// The S120 motors: a 1FK7 brings its data sheet's winding as the model's one winding has it (three times the
/// phase's resistance and inductance) and the current loop retuned for it (Kp = L / T with the config's T = 10 mH /
/// 40 V/A = 0.25 ms, Tn = L / R); a 1FK2 on S120 its manual's rated values. The pairing: D 21.4's Motor Module for
/// the 1FK7, the smallest Single Motor Module carrying the stall current for the 1FK2 (1.06 A: the 3 A one).
#[test]
fn s120_motors_by_name_bring_winding_rated_values_and_their_motor_module() {
    let mut cfg = exct_30();
    apply_motor(&mut cfg, "1FK7060-2AF7", "s120").unwrap();
    // 1FK7060-2AF7: 6 N m at 4.45 A stall, 7.7 kg cm2; data sheet 1.35 ohm and 15.2 mH per phase, 30 min, 18 N m at 15 A.
    for (path, want) in [
        ("motor.torque_constant_nm_a", 6.0 / 4.45),
        ("motor.rotor_inertia_kgm2", 7.7e-4),
        ("motor.resistance_ohm", 3.0 * 1.35),
        ("motor.inductance_h", 3.0 * 15.2e-3),
        ("drive.current_kp_v_a", 3.0 * 15.2e-3 / 2.5e-4),
        ("drive.current_tn_s", 15.2e-3 / 1.35),
        ("motor.thermal_time_constant_s", 1800.0),
        ("motor.max_current_a", 15.0),
        ("drive.rated_torque_nm", 4.7),
        ("drive.rated_speed_rad_s", 3000.0 * std::f64::consts::PI / 30.0),
        ("drive.bus_voltage_v", 540.0),
        // Held within the EXCT-30's 12.5 N m at the pulley.
        ("drive.peak_torque_nm", 12.5),
    ] {
        assert!(close(cfg.f64(path).unwrap(), want, 1e-9), "{path}: {} against {want}", cfg.f64(path).unwrap());
    }
    apply_motor(&mut cfg, "1FK2103-2AH", "s120").unwrap();
    assert!(
        close(cfg.f64("drive.rated_speed_rad_s").unwrap(), 4500.0 * std::f64::consts::PI / 30.0, 1e-12)
            && cfg.f64("drive.rated_torque_nm").unwrap() == 0.59
    );
    assert_eq!(paired_drive("1FK7060-2AF7", "s120"), "6SL3120-1TE15-0AD0");
    assert_eq!(paired_drive("1FK2103-2AH", "s120"), "6SL3120-1TE13-0AD0");
    assert_eq!(paired_drive("1FK2205-4AF", "400v"), "6SL3210-5HE11-5UF0");
    assert!(apply_motor(&mut cfg, "1FK7060-2AF7", "400v").is_err(), "a 1FK7 is no S210 motor");
    // Every S120 motor's pairing exists and carries its stall current.
    let modules = csv("siemens/motor_modules_s120.csv");
    for article in sim_components::catalog::motor_articles_for("s120") {
        let drive = paired_drive(article, "s120");
        let module = modules
            .iter()
            .find(|m| m["part_number"] == drive)
            .unwrap_or_else(|| panic!("{article}: no Motor Module {drive}"));
        let mut probe = exct_30();
        apply_motor(&mut probe, article, "s120").unwrap();
        let rated: f64 = module["rated_output_current_a"].parse().unwrap();
        assert!(rated >= 0.85 * probe.f64("motor.rated_current_a").unwrap(), "{article}: {drive}");
    }
}

/// Three passing motors on the EXCT-30, ranked by hand: two motors of 3.4, 5.2 and 6.3 kg (1FK2104-6AF,
/// 1FK2205-4AF, 1FK2206-2AF, without gearbox), costs 6.8, 10.4 and 12.6 kg.
#[test]
fn the_search_ranks_three_passing_motors_by_their_hand_calculated_weight() {
    let wanted = |a: &str| ["1FK2206-2AF", "1FK2205-4AF", "1FK2104-6AF"].contains(&a);
    let found = search_motors(&machine_t_gantry::ENTRY, &exct_30(), "400v", &wanted, None);
    let order: Vec<(&str, f64, &str)> = found.iter().map(|c| (c.article, c.cost, c.drive)).collect();
    assert_eq!(
        order,
        [
            ("1FK2104-6AF", 6.8, "6SL3210-5HE11-0UF0"),
            ("1FK2205-4AF", 10.4, "6SL3210-5HE11-5UF0"),
            ("1FK2206-2AF", 12.6, "6SL3210-5HE11-5UF0")
        ]
    );
    assert!(found.iter().all(Candidate::passes));
    // A gearbox's mass counts: the single axis with the 1FK2104-5AK (2.85 kg) and the NRK070-04-5 (1.4 kg).
    let single = &machine_single_axis::ENTRY;
    let found = search_motors(single, &single.config(), "400v", &|a| a == "1FK2104-5AK", Some("NRK070"));
    let with = found.iter().find(|c| c.gearbox == "NRK070-04-5").unwrap();
    assert!(close(with.mass, 4.25, 1e-12) && close(with.cost, 4.25, 1e-12), "{} {}", with.mass, with.cost);
    // The S120 family: the 1FK7060-2AF7 runs on the EXCT-30 with its Motor Module.
    let found = search_motors(&machine_t_gantry::ENTRY, &exct_30(), "s120", &|a| a == "1FK7060-2AF7", None);
    assert!(
        found.len() == 1 && found[0].drive == "6SL3120-1TE15-0AD0" && found[0].result.is_some(),
        "{:?}",
        found[0].refused
    );
}

/// The axis search on the EXCT-30 cycle with one motor: the EXCT-15 is ruled out by its stroke (Z 200 mm at most),
/// the ELZI 30 by its speed (2 m/s against the config's 2.5 m/s), without a cycle; the others carry it, and the cost
/// adds the axis's weight at the config's strokes. By hand: EXCT-30 25.38 + 1.48 * 10 + 0.37 * 2.5 = 41.105 kg, ELZI 40
/// 11.5 + 0.68 * 10 + 0.3 * 2.5 = 19.05 kg, plus two motors of 5.2 kg.
#[test]
fn the_axis_search_rules_out_by_catalogue_numbers_and_adds_the_axis_weight() {
    let found = search_axes(&machine_t_gantry::ENTRY, &exct_30(), "400v", &|a| a == "1FK2205-4AF", None);
    let by = |axis: &str| found.iter().find(|c| c.axis == axis).unwrap_or_else(|| panic!("{axis}"));
    assert_eq!((by("EXCT-15").refused, by("EXCT-15").article), ("axis-stroke", ""));
    assert_eq!(by("ELZI-30").refused, "axis-speed");
    assert!(close(by("EXCT-30").cost, 41.105 + 10.4, 1e-9) && close(by("ELZI-40").cost, 19.05 + 10.4, 1e-9));
    assert!(by("EXCT-30").passes(), "{:?}", by("EXCT-30").tightest());
    assert_eq!(found.iter().filter(|c| c.result.is_some()).count(), 4, "EXCT-30, EXCT-100, ELZI-40, ELZI-60");
    let passing: Vec<&Candidate> = found.iter().take_while(|c| c.passes()).collect();
    assert!(passing.windows(2).all(|w| w[0].cost <= w[1].cost));
    // On the single axis only Bahr's single axes are tried; the gantries are of another kind.
    let single = &machine_single_axis::ENTRY;
    let found = search_axes(single, &single.config(), "400v", &|a| a == "1FK2205-4AF", None);
    assert!(
        found.iter().all(|c| !c.axis.starts_with("EXCT") && !c.axis.starts_with("ELZI"))
            && found.iter().any(Candidate::passes)
    );
}
