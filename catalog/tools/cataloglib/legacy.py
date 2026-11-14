"""The column maps from the layouts the extraction scripts write (vendor terms, the units the documents print) to
the catalogue's schema 1 (vendor-neutral names, SI units).

Every extraction goes through `convert`: the script writes its file as before, into a scratch folder, and `convert`
turns it into the table of the catalogue. It refuses a column it has no map for and checks that every value
converts back to the number the script wrote, so nothing is dropped or changed on the way.
"""

import csv
import math

from .units import from_si, to_si

# Columns of the motor tables that recur.
_MOTOR = [
    ("article", "part_number", "same"),
    ("static_torque_nm", "static_torque_nm", "same"),
    ("stall_current_a", "stall_current_a", "same"),
    ("max_torque_nm", "max_torque_nm", "same"),
    ("max_current_a", "max_current_a", "same"),
    ("rotor_inertia_kgcm2", "rotor_inertia_kgm2", "kgcm2"),
    ("rotor_inertia_with_brake_kgcm2", "rotor_inertia_with_brake_kgm2", "kgcm2"),
    ("mass_with_brake_kg", "mass_with_brake_kg", "same"),
    ("mass_kg", "mass_kg", "same"),
]
_MOTOR_MANUAL = _MOTOR + [
    ("max_speed_inverter_rpm", "max_speed_inverter_rad_s", "rpm"),
    ("thermal_time_constant_min", "thermal_time_constant_s", "minute"),
    ("rated_speed_rpm", "rated_speed_rad_s", "rpm"),
    ("rated_torque_nm", "rated_torque_nm", "same"),
    ("rated_current_a", "rated_current_a", "same"),
    ("rated_power_kw", "rated_power_w", "kilo"),
]
_GEARBOX = [
    ("series", "series", "same"),
    ("size", "size", "same"),
    ("motor_frame", "motor_frame", "same"),
    ("stages", "stages", "same"),
    ("ratio", "ratio", "same"),
    ("rated_torque_nm", "rated_output_torque_nm", "same"),
    ("max_torque_nm", "max_output_torque_nm", "same"),
    ("emergency_off_torque_nm", "emergency_stop_output_torque_nm", "same"),
    ("efficiency_full_load", "efficiency_full_load", "same"),
    ("torsional_stiffness_nm_arcmin", "torsional_stiffness_nm_rad", "per_arcmin"),
    ("backlash_arcmin", "backlash_rad", "arcmin"),
    ("inertia_at_input_kgcm2", "inertia_at_input_kgm2", "kgcm2"),
]

# file of the extraction script -> (file in the catalogue, part class, table of the class or None, columns)
# columns: (column of the script, column of the catalogue, conversion of units.CONVERSIONS)
TABLES = {
    "siemens/motors_s210.csv": ("siemens/motors_s210.csv", "servo_motor", None, [
        ("article", "part_number", "same"),
        ("series", "series", "same"),
        ("version", "version", "same"),
        ("shaft_height_mm", "shaft_height_m", "milli"),
        ("cooling", "cooling", "same"),
        ("static_torque_nm", "static_torque_nm", "same"),
        ("stall_current_a", "stall_current_a", "same"),
        ("max_torque_nm", "max_torque_nm", "same"),
        ("max_current_a", "max_current_a", "same"),
        ("max_speed_rpm", "max_speed_rad_s", "rpm"),
        ("rotor_inertia_kgcm2", "rotor_inertia_kgm2", "kgcm2"),
        ("rotor_inertia_with_brake_kgcm2", "rotor_inertia_with_brake_kgm2", "kgcm2"),
        ("mass_with_brake_kg", "mass_with_brake_kg", "same"),
        ("mass_kg", "mass_kg", "same"),
    ] + [(f"rated_{q}_{s}_{u}", f"rated_{q}_{s}_{n}", c) for s in ("230v", "400v")
         for q, u, n, c in (("speed", "rpm", "rad_s", "rpm"), ("torque", "nm", "nm", "same"), ("current", "a", "a", "same"), ("power", "w", "w", "same"))]
      + [(f"converter_{g}{s}", f"paired_drive_{g}{s}", "same") for g in ("", "new_") for s in ("230v_1ac", "230v_3ac", "400v_3ac")]),
    "siemens/motors_1fk2_s120.csv": ("siemens/motors_1fk2_s120.csv", "servo_motor", None, _MOTOR_MANUAL),
    "siemens/motors_1ft2_s120.csv": ("siemens/motors_1ft2_s120.csv", "servo_motor", None, _MOTOR_MANUAL),
    "siemens/motors_s210_oi.csv": ("siemens/motors_s210_oi.csv", "servo_motor", None, [
        ("article", "part_number", "same"),
        ("static_torque_nm", "static_torque_nm", "same"),
        ("max_current_a", "max_current_a", "same"),
        ("thermal_time_constant_min", "thermal_time_constant_s", "minute"),
    ]),
    "siemens/dynamic_braking_1fk2.csv": ("siemens/dynamic_braking_1fk2.csv", "servo_motor", "dynamic_braking", [
        ("article", "part_number", "same"),
        ("optimum_resistance_ohm", "optimum_resistance_ohm", "same"),
        ("braking_torque_rms_nm", "braking_torque_rms_nm", "same"),
        ("braking_torque_rms_with_resistor_nm", "braking_torque_rms_with_resistor_nm", "same"),
        ("max_braking_torque_nm", "max_braking_torque_nm", "same"),
        ("braking_current_rms_a", "braking_current_rms_a", "same"),
        ("braking_current_rms_with_resistor_a", "braking_current_rms_with_resistor_a", "same"),
    ]),
    "siemens/motors_s120.csv": ("siemens/motors_s120.csv", "servo_motor", None, [
        ("article", "part_number", "same"),
        ("series", "series", "same"),
        ("version", "version", "same"),
        ("cooling", "cooling", "same"),
        ("shaft_height_mm", "shaft_height_m", "milli"),
        ("rated_speed_rpm", "rated_speed_rad_s", "rpm"),
        ("rated_values_at_rpm", "rated_values_at_speed_rad_s", "rpm"),
        ("rated_power_kw", "rated_power_w", "kilo"),
        ("static_torque_nm", "static_torque_nm", "same"),
        ("rated_torque_nm", "rated_torque_nm", "same"),
        ("rated_current_a", "rated_current_a", "same"),
        ("pole_pairs", "pole_pairs", "same"),
        ("rotor_inertia_kgcm2", "rotor_inertia_kgm2", "kgcm2"),
        ("mass_kg", "mass_kg", "same"),
        ("efficiency_percent", "efficiency", "percent"),
        ("stall_current_a", "stall_current_a", "same"),
        ("calculated_power_kw", "calculated_power_w", "kilo"),
        ("motor_module_rated_current_a", "paired_drive_rated_current_a", "same"),
        ("motor_module", "paired_drive", "same"),
    ]),
    "siemens/motors_s120_electrical.csv": ("siemens/motors_s120_electrical.csv", "servo_motor", None, [
        ("article", "part_number", "same"),
        ("data_sheet_article", "data_sheet_part_number", "same"),
        ("poles", "poles", "same"),
        ("static_torque_60k_nm", "static_torque_60k_nm", "same"),
        ("stall_current_60k_a", "stall_current_60k_a", "same"),
        ("max_speed_mechanical_rpm", "max_speed_mechanical_rad_s", "rpm"),
        ("max_speed_inverter_rpm", "max_speed_inverter_rad_s", "rpm"),
        ("max_torque_nm", "max_torque_nm", "same"),
        ("max_current_a", "max_current_a", "same"),
        ("torque_constant_nm_a", "torque_constant_nm_a", "same"),
        ("voltage_constant_v_per_1000rpm", "voltage_constant_v_s_rad", "v_per_krpm"),
        ("winding_resistance_20c_ohm", "phase_resistance_20c_ohm", "same"),
        ("inductance_mh", "phase_inductance_h", "milli"),
        ("electrical_time_constant_ms", "electrical_time_constant_s", "milli"),
        ("mechanical_time_constant_ms", "mechanical_time_constant_s", "milli"),
        ("thermal_time_constant_min", "thermal_time_constant_s", "minute"),
        ("shaft_torsional_stiffness_nm_rad", "shaft_torsional_stiffness_nm_rad", "same"),
    ]),
    "siemens/converters_s210.csv": ("siemens/converters_s210.csv", "drive", None, [
        ("article", "part_number", "same"),
        ("series", "series", "same"),
        ("supply", "supply", "same"),
        ("line_current_a", "line_current_a", "same"),
        ("power_loss_w", "power_loss_w", "same"),
        ("rated_power_1ac_230v_kw", "rated_power_1ac_230v_w", "kilo"),
        ("rated_power_3ac_400v_kw", "rated_power_3ac_400v_w", "kilo"),
        ("rated_power_3ac_240v_kw", "rated_power_3ac_240v_w", "kilo"),
        ("rated_output_current_a", "rated_output_current_a", "same"),
        ("max_output_current_a", "max_output_current_a", "same"),
    ]),
    "siemens/motor_modules_s120.csv": ("siemens/motor_modules_s120.csv", "drive", None, [
        ("article", "part_number", "same"),
        ("axes", "axes", "same"),
        ("type", "duty_type", "same"),
        ("rated_current_a", "rated_output_current_a", "same"),
        ("base_load_current_a", "base_load_output_current_a", "same"),
        ("max_current_a", "max_output_current_a", "same"),
        ("rated_power_kw", "rated_power_w", "kilo"),
        ("pulse_frequency_khz", "pulse_frequency_hz", "kilo"),
        ("dc_link_capacitance_uf", "dc_link_capacitance_f", "micro"),
        ("power_loss_typical_kw", "power_loss_typical_w", "kilo"),
        ("power_loss_max_kw", "power_loss_max_w", "kilo"),
    ]),
    "siemens/line_modules_s120.csv": ("siemens/line_modules_s120.csv", "drive", "line_module", [
        ("article", "part_number", "same"),
        ("type", "infeed_type", "same"),
        ("rated_power_kw", "rated_power_w", "kilo"),
        ("s6_power_kw", "s6_power_w", "kilo"),
        ("max_power_kw", "max_power_w", "kilo"),
        ("dc_link_current_540v_a", "dc_link_current_540v_a", "same"),
        ("dc_link_current_600v_a", "dc_link_current_600v_a", "same"),
        ("s6_dc_link_current_a", "s6_dc_link_current_a", "same"),
        ("max_dc_link_current_a", "max_dc_link_current_a", "same"),
        ("dc_link_capacitance_uf", "dc_link_capacitance_f", "micro"),
        ("power_loss_kw", "power_loss_w", "kilo"),
        ("braking_power_max_kw", "max_braking_power_w", "kilo"),
        ("braking_power_continuous_kw", "continuous_braking_power_w", "kilo"),
        ("braking_threshold_v", "braking_threshold_v", "same"),
        ("min_braking_resistance_ohm", "min_braking_resistance_ohm", "same"),
    ]),
    "siemens/braking_s210.csv": ("siemens/braking_s210.csv", "drive", "braking", [
        ("article", "part_number", "same"),
        ("line", "line", "same"),
        ("rated_power_kw", "rated_power_w", "kilo"),
        ("continuous_braking_power_w", "continuous_braking_power_w", "same"),
        ("peak_braking_power_kw", "peak_braking_power_w", "kilo"),
        ("braking_energy_kj", "braking_energy_j", "kilo"),
        ("dc_link_overvoltage_v", "dc_link_overvoltage_v", "same"),
    ]),
    "siemens/dc_link_measured.csv": ("siemens/dc_link_measured.csv", "drive", "dc_link_measurement", [
        ("article", "part_number", "same"),
        ("date", "date", "same"),
        ("firmware", "firmware", "same"),
        ("capacitance_uf", "dc_link_capacitance_f", "micro"),
        ("capacitance_low_uf", "dc_link_capacitance_low_f", "micro"),
        ("capacitance_high_uf", "dc_link_capacitance_high_f", "micro"),
        ("chopper_above_v", "braking_threshold_above_v", "same"),
        ("idle_dc_link_v", "idle_dc_link_v", "same"),
    ]),
    "siemens/braking_s120.csv": ("siemens/braking_s120.csv", "braking_resistor", None, [
        ("article", "part_number", "same"),
        ("kind", "kind", "same"),
        ("for_article", "for_part_number", "same"),
        ("resistance_ohm", "resistance_ohm", "same"),
        ("rated_power_kw", "continuous_power_w", "kilo"),
        ("peak_power_kw", "peak_power_w", "kilo"),
        ("activation_threshold_v", "activation_threshold_v", "same"),
        ("dc_link_capacitance_uf", "dc_link_capacitance_f", "micro"),
        ("load_duration_s", "load_duration_s", "same"),
        ("cycle_duration_s", "cycle_duration_s", "same"),
        ("load_duration_2_s", "load_duration_2_s", "same"),
        ("cycle_duration_2_s", "cycle_duration_2_s", "same"),
    ]),
    "siemens/gearbox_nrb.csv": ("siemens/gearbox_nrb.csv", "gearbox", None, _GEARBOX),
    "siemens/gearbox_nrk.csv": ("siemens/gearbox_nrk.csv", "gearbox", None, _GEARBOX),
    "siemens/gearbox_nlc.csv": ("siemens/gearbox_nlc.csv", "gearbox", None, _GEARBOX),
    "siemens/gearbox_mass.csv": ("siemens/gearbox_mass.csv", "gearbox", "mass", [
        ("series", "series", "same"),
        ("size", "size", "same"),
        ("motor_frames", "motor_frames", "same"),
        ("stages", "stages", "same"),
        ("ratio", "ratio", "same"),
        ("mass_kg", "mass_kg", "same"),
    ]),
    # tools/extract_megadyne.py writes schema columns in SI since 14 November 2026 (with min_pulley_teeth, thickness_m).
    "megadyne/belts.csv": ("megadyne/belts.csv", "belt", None, [
        (c, c, "same") for c in ("profile", "pitch_m", "width_m", "mass_kg_m", "max_pull_n", "breaking_strength_n",
                                 "elongation_at_max_pull", "min_pulley_teeth", "thickness_m")
    ]),
    "megadyne/belt_tooth_load.csv": ("megadyne/belt_tooth_load.csv", "belt", "tooth_load", [
        (c, c, "same") for c in ("profile", "speed_rad_s", "specific_tooth_load_n_m")
    ]),
    "bahr/belt_axes.csv": ("bahr/belt_axes.csv", "belt_axis", None, [
        ("series", "series", "same"),
        ("size", "size", "same"),
        ("kind", "kind", "same"),
        ("belt", "belt", "same"),
        ("belt_pitch_mm", "belt_pitch_m", "milli"),
        ("belt_width_mm", "belt_width_m", "milli"),
        ("feed_per_rev_mm", "feed_per_rev_m", "milli"),
        ("pulley_teeth", "pulley_teeth", "same"),
        ("no_load_torque_nm", "no_load_torque_nm", "same"),
        ("no_load_torque_times", "no_load_torque_count", "same"),
        ("max_speed_m_s", "max_speed_m_s", "same"),
        ("feed_force_continuous_n", "feed_force_continuous_n", "same"),
        ("feed_force_peak_n", "feed_force_peak_n", "same"),
    ] + [(f"{a}_{k}_{u}", f"{a}_{k}_{u}", "same") for a, u in (("fx", "n"), ("fy", "n"), ("fz", "n"), ("mx", "nm"), ("my", "nm"), ("mz", "nm")) for k in ("static", "dynamic")] + [
        ("base_weight_kg", "mass_at_zero_stroke_kg", "same"),
        ("weight_per_100_mm_kg", "mass_per_stroke_kg_m", "per_100_mm"),
        ("weight_per_100_mm_z_kg", "mass_per_stroke_z_kg_m", "per_100_mm"),
        ("moved_mass_x_kg", "moved_mass_x_kg", "same"),
        ("moved_mass_y_kg", "moved_mass_y_kg", "same"),
        ("moved_mass_z_kg", "moved_mass_z_kg", "same"),
        ("max_length_one_piece_mm", "max_length_one_piece_m", "milli"),
        ("max_length_x_mm", "max_length_x_m", "milli"),
        ("max_length_y_mm", "max_length_y_m", "milli"),
        ("max_length_z_mm", "max_length_z_m", "milli"),
        # Where the row was read: a source of the table's metadata, see `source_of_row`.
        ("sheet", None, "same"),
        ("edition", None, "same"),
        ("source_file", None, "same"),
    ]),
}

# The Festo EXCT file had no extraction script; `migrate_v1.py` flattens its TOML into rows with these keys.
FESTO = {
    "belt_axes": [
        ("series", "series", "same"),
        ("size", "size", "same"),
        ("kind", "kind", "same"),
        ("stroke_y_min_mm", "stroke_x_min_m", "milli"),
        ("stroke_y_max_mm", "stroke_x_max_m", "milli"),
        ("rated_load_kg", "rated_payload_kg", "same"),
        ("max_process_force_z_n", "max_process_force_z_n", "same"),
        ("max_torque_nm", "max_pulley_torque_nm", "same"),
        ("max_no_load_torque_nm", "max_no_load_torque_nm", "same"),
        ("max_acceleration_m_s2", "max_acceleration_m_s2", "same"),
        ("max_speed_m_s", "max_speed_m_s", "same"),
        ("repetition_accuracy_mm", "repeatability_m", "milli"),
        ("weight_at_zero_stroke_kg", "mass_at_zero_stroke_kg", "same"),
        ("weight_per_100_mm_stroke_y_kg", "mass_per_stroke_kg_m", "per_100_mm"),
        ("weight_per_100_mm_stroke_z_kg", "mass_per_stroke_z_kg_m", "per_100_mm"),
        ("servo_motor", "motor_part_number", "same"),
        ("axial_kit", "axial_kit_part_number", "same"),
        ("coupling", "coupling_part_number", "same"),
        ("coupling_housing_third_party_motor", "coupling_housing_part_number", "same"),
        ("belt.material", "belt_material", "same"),
        ("belt.pitch_mm", "belt_pitch_m", "milli"),
        ("belt.width_mm", "belt_width_m", "milli"),
        ("belt.elongation_percent", "belt_elongation", "percent"),
        ("belt.elongation_reference_force_n", "belt_elongation_force_n", "same"),
        ("belt.effective_diameter_mm", "pulley_effective_diameter_m", "milli"),
        ("belt.feed_constant_mm_rev", "feed_per_rev_m", "milli"),
    ],
    # M = a (load_factor m_L + motor_inertia_factor J_m + constant) 1e-3 + ... with J_m in kg cm2 becomes, in SI,
    # M = a (load_factor_m m_L + motor_inertia_factor_1_m J_m + constant_kg_m) + ... with J_m in kg m2.
    "belt_axis_torque_formula": [
        ("series", "series", "same"),
        ("size", "size", "same"),
        ("stroke_z_mm", "stroke_z_m", "milli"),
        ("load_factor", "load_factor_m", "milli"),
        ("motor_inertia_factor", "motor_inertia_factor_1_m", "ten"),
        ("constant", "constant_kg_m", "milli"),
        ("gravity_factor", "gravity_factor_nm_kg", "same"),
        ("gravity_mass_kg", "gravity_mass_kg", "same"),
    ],
}


def read_legacy(path):
    """The comment lines (without '# ') and the rows of a file as an extraction script writes it."""
    text = path.read_text(encoding="utf-8")
    notes = [l[2:] if l.startswith("# ") else l[1:] for l in text.splitlines() if l.startswith("#")]
    rows = list(csv.DictReader(l for l in text.splitlines() if not l.startswith("#")))
    return notes, rows


def convert_rows(rows, columns, what):
    """The rows in the catalogue's columns. Fails on a column without a map, and on a value that does not convert
    back to what was written: a conversion that would lose or change a number stops it."""
    if not rows:
        return [], []
    mapped = {old for old, _, _ in columns}
    unknown = [c for c in rows[0] if c not in mapped]
    if unknown:
        raise SystemExit(f"{what}: no column map for {unknown}; add it to cataloglib/legacy.py")
    header = [new for _, new, _ in columns if new]
    out = []
    for row in rows:
        new = {}
        for old, name, conversion in columns:
            value = row.get(old, "")
            if value is None:
                raise SystemExit(f"{what}: a row has fewer values than the header: {row}")
            if not name:
                continue
            new[name] = to_si(value.strip(), conversion)
            if conversion != "same" and value.strip():
                back = from_si(new[name], conversion)
                if not math.isclose(back, float(value), rel_tol=1e-12, abs_tol=1e-300):
                    raise SystemExit(f"{what}: {old} = {value} converts to {new[name]}, which reads back as {back}")
        out.append(new)
    return header, out


def source_of_row(row):
    """Bahr's rows name the sheet (file) they were read from; that becomes the row's source in the metadata."""
    return row["source_file"].removesuffix(".pdf")
