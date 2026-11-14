# Catalogue

[`catalog/`](../catalog) holds vendor data as CSV tables in one vendor-neutral format, schema 1: SI units named in the column (`_m`, `_kgm2`, `_rad_s`, `_w`, ...), the same column name for the same quantity whatever the maker calls it, and beside each `<table>.csv` a `<table>.meta.toml` with its part class, the source of every value (document, page or table, date, extraction script or `[estimate]`), and the notes: what a row is, the conditions of the values, and what was kept as printed although it looks wrong. The numbers are the vendors'; the layout and the column names are ours. The format, the validator and how to add a vendor are in [`catalog/README.md`](../catalog/README.md) and [`catalog/CONTRIBUTING.md`](../catalog/CONTRIBUTING.md); the folder is laid out to become a repository of its own.

The column names and units below are schema 1's. The documents print other units (r/min, kg cm², arcmin, kW, mm); `catalog/tools/cataloglib/legacy.py` maps each extraction script's columns to the schema's, and checks every converted value against the printed one.

- [What the simulator uses](#what-the-simulator-uses)
- [Files](#files)
- [Other makers](#other-makers)
- [How the files were extracted and checked](#how-the-files-were-extracted-and-checked)
- [Misprints and inconsistencies found in the vendor documents](#misprints-and-inconsistencies-found-in-the-vendor-documents)
- [What is missing, and why](#what-is-missing-and-why)
- [Rerun an extraction on a newer edition](#rerun-an-extraction-on-a-newer-edition)

## What the simulator uses

These files are read by the simulator itself. They are compiled into `sim-components` ([`catalog.rs`](../crates/sim-components/src/catalog.rs)), so the binaries need no path to them, and a changed file takes effect after a rebuild. `--catalog DIR` (on `sim-cli` and `sim-app`) reads them from another copy of the catalogue at run time instead; it must have the same `schema_version`.

| File | Used for |
| --- | --- |
| `siemens/motors_s210.csv` | A motor by name (`[parts] motor`, `--motor`) at `230v` or `400v`, its paired S210 converter, and the motor search |
| `siemens/motors_1fk2_s120.csv`, `siemens/motors_1ft2_s120.csv` | The thermal time constant of an S210 motor by name (34 of the 96 are in neither file; for them the config's value stays); the 1FK2 and 1FT2 by name at `s120`, and in the search with `--supply s120` |
| `siemens/motors_s120.csv`, `siemens/motors_s120_electrical.csv` | The 1FK7 and 1FT7 by name at `s120` (with winding resistance and inductance from the data sheets) and their paired Motor Module; the search with `--supply s120` |
| `siemens/converters_s210.csv`, `siemens/motor_modules_s120.csv` | A drive by name (`[parts] drive`, `--drive`): its rated and maximum output current. The search pairs each motor with one (see [pairing](#pairing-a-motor-with-its-drive)) |
| `siemens/gearbox_nrb.csv`, `_nrk.csv`, `_nlc.csv` | A gearbox by name (`[parts] gearbox`, `--gearbox`) and the gearbox search |
| `siemens/gearbox_mass.csv` | The gearbox's mass in the search's `mass_kg` and `cost_kg` |
| `megadyne/belts.csv`, `megadyne/belt_tooth_load.csv` | A belt by name (`[parts] belt`, `--belt`), and the belt of an axis by name |
| `festo/belt_axes.csv`, `festo/belt_axis_torque_formula.csv`, `bahr/belt_axes.csv` | An axis by name (`[parts] axis`, `--axis`) and the axis search (`search --axes`) |
| `<vendor>/motors_*.csv`, `<vendor>/drives_*.csv` (Beckhoff, B&R, Kollmorgen, Lenze, Mitsubishi, Rexroth, SEW, Yaskawa) | A motor by name at `<vendor>-<volts>v` (e.g. `beckhoff-400v`), its paired drive, a drive by name, and the search with `--vendor` (see [other makers](#other-makers)) |

`festo/belt_axes.csv` and `festo/belt_axis_torque_formula.csv` are also the source of the `[catalogue]` values in [`exct-30.toml`](../crates/machine-t-gantry/exct-30.toml); two tests hold the two together (`exct_30_config_agrees_with_the_catalogue`, and `an_axis_by_name_gives_the_numbers_of_the_hand_written_exct_30` for the axis by name). The line module and braking files are data for the DC link estimates; the validator checks them, but no command uses them.

### Pairing a motor with its drive

| Supply | Drive | Rule |
| --- | --- | --- |
| `400v` | S210 converter | The column `converter_400v_3ac` of `motors_s210.csv`, as D 32 lists it beside the motor |
| `230v` | S210 converter | `converter_230v_1ac` where there is one, else `converter_230v_3ac` |
| `s120`, 1FK7 and 1FT7 | Single Motor Module | The column `motor_module` of `motors_s120.csv`, its placeholders read as a Single Motor Module (`6SL3120-1...`) |
| `s210new-230v`, `s210new-400v` | S210 (New) converter, 6SL5310 | The columns `paired_drive_new_*` of `motors_s210.csv`, as D 32 lists them beside the motor (until 14 November 2026 the twelve 6SL5310 converters were in `converters_s210.csv` but never paired) |
| `<vendor>-<volts>v` | That maker's drive at that line voltage | The motor row's `paired_drive`, the maker's own pairing (B&R, Kollmorgen AKM2G, Lenze, Mitsubishi, SEW, Yaskawa); where the maker publishes none (Beckhoff, Kollmorgen AKM, Rexroth, a few rows of the others), the smallest of the maker's drives at that voltage whose rated current carries the motor's stall current, a single-axis one before a multi-axis one |
| `s120`, 1FK2 and 1FT2 | Single Motor Module | The manuals pair none. The smallest Single Motor Module whose rated current carries the motor's stall current, the rule D 21.4 pairs the 1FK7 and 1FT7 by; of two such, the one with the higher maximum current |

A motor named in `[parts]` does not bring its drive: the drive is named on its own. The search names the paired one.

### What stands in for the price

No file has a price. Siemens' list prices are in its Industry Mall, which refuses automated access (HTTP 403), and none of the catalogues read here prints one; Festo's and Bahr's are not public. The search ranks by installed weight instead: `cost_kg` = number of motors × (motor without brake + gearbox) + the axis at the config's strokes (where one of the catalogue is tried). Weight goes roughly with size and so with price within a family; across makers and families it does not, which is why this is a stand-in and called so. The drive's weight is not counted (the converter files have none).

## Files

### `festo/belt_axes.csv` and `festo/belt_axis_torque_formula.csv`

| | |
| --- | --- |
| Source | Festo catalogue "Linear gantries EXCT", 2023/01 |
| A row is | One size of the EXCT T-gantry: series `EXCT`, size `15`, `30`, `100`. The formula table: one size and Z stroke |
| Extraction | There is no extraction script for these files. They have three sizes and were written from the catalogue's tables (until November 2026 as `festo/exct.toml`) |

| Column | Meaning |
| --- | --- |
| `stroke_x_min_m`, `stroke_x_max_m` | Stroke range of the horizontal axis (Festo's Y; the simulator's X). The Z strokes offered are those of the formula table |
| `rated_payload_kg` | Tool load plus payload, at maximum dynamic response |
| `max_process_force_z_n` | Highest process force in Z |
| `max_pulley_torque_nm`, `max_no_load_torque_nm` | At the pulley; they also apply to third-party motors. The no-load torque is at 0.2 m/s and travel at 45 degrees |
| `max_acceleration_m_s2`, `max_speed_m_s` | "Under ideal conditions" |
| `repeatability_m` | |
| `mass_at_zero_stroke_kg`, `mass_per_stroke_kg_m`, `mass_per_stroke_z_kg_m` | Without rated load, motors, axial kits and mounting kits |
| `motor_part_number`, `axial_kit_part_number`, `coupling_part_number`, `coupling_housing_part_number` | Festo part names (the coupling housing for a third-party motor) |
| `belt_material`, `belt_pitch_m`, `belt_width_m` | The belt |
| `belt_elongation`, `belt_elongation_force_n` | The belt stretches by this strain at this force. Gives EA |
| `pulley_effective_diameter_m`, `feed_per_rev_m` | Pulley. The feed per revolution is "at 45 degrees travel" |
| Formula table: `stroke_z_m`, `load_factor_m`, `motor_inertia_factor_1_m`, `constant_kg_m`, `gravity_factor_nm_kg`, `gravity_mass_kg` | Coefficients of Festo's formula for the required motor torque at 45 degrees travel, the heaviest case, in SI |

Festo's formula, in the catalogue's units and in the file's:

```text
M = a · (load_factor · m_L + motor_inertia_factor · J_m + constant) · 1e-3 + gravity_factor · (gravity_mass_kg + m_L) + M_R     [N m], J_m in kg cm²
M = a · (load_factor_m · m_L + motor_inertia_factor_1_m · J_m + constant_kg_m) + gravity_factor_nm_kg · (gravity_mass_kg + m_L) + M_R   [N m], J_m in kg m²
n = v · 60 / feed_per_rev_m · sqrt(2)                                                                                            [1/min]
```

a: acceleration along the path in m/s², v: speed along the path in m/s, m_L: attachment on the Z axis with payload in kg, M_R: friction torque in N m.

The motors couple directly onto the belt pulleys; the catalogue lists no gear unit.

### `siemens/motors_s210.csv`

| | |
| --- | --- |
| Source | Siemens catalogue D 32, SINAMICS S210 servo drive system, January 2024, "Specific technical specifications" of each shaft height |
| A row is | One motor winding: the article number up to the speed letter, e.g. `1FK2104-5AK`. 96 rows: 34 1FK2 and 62 1FT2 |
| Script | `extract_siemens_d32.py` |

Columns: `part_number`, `series`, `version`, `shaft_height_m`, `cooling`, `static_torque_nm`, `stall_current_a`, `max_torque_nm`, `max_current_a`, `max_speed_rad_s`, `rotor_inertia_kgm2`, `rotor_inertia_with_brake_kgm2`, `mass_with_brake_kg`, `mass_kg`; then per supply (`230v` is 200 to 240 V, 1 AC or 3 AC; `400v` is 380 to 480 V 3 AC) `rated_speed_*_rad_s`, `rated_torque_*_nm`, `rated_current_*_a`, `rated_power_*_w`; then the paired converters `paired_drive_230v_1ac`, `paired_drive_230v_3ac`, `paired_drive_400v_3ac` and the same for the "SINAMICS S210 (New)" converters (`paired_drive_new_*`).

Currents are RMS. Static values are at standstill. An empty field means the catalogue lists no value, mostly because the motor is not offered at that supply: 84 motors have 400 V data, 88 have 230 V data.

### `siemens/converters_s210.csv`

| | |
| --- | --- |
| Source | The same catalogue D 32, "Technical specifications" of the servo converters |
| A row is | One converter. 24 rows: 12 S210 and 12 S210 (New) |
| Script | `extract_siemens_d32.py` |

Columns: `part_number`, `series`, `supply`, `line_current_a`, `power_loss_w`, `rated_power_1ac_230v_w`, `rated_power_3ac_400v_w`, `rated_power_3ac_240v_w`, `rated_output_current_a`, `max_output_current_a`. Output currents are RMS; the power unit switches at 8 kHz.

### `siemens/braking_s210.csv`

| | |
| --- | --- |
| Source | Siemens, SINAMICS S210 servo drive system with S-1FK2 and S-1FT2, Operating Instructions, 09/2022, A5E41702836B AL: Table 4-4 (the same values as Table 3-2 of the 06/2020 edition) and the text of fault F07432 for the trip levels |
| A row is | One S210 (6SL3210) converter. 12 rows, the same converters as `converters_s210.csv` (a test checks it) |
| Script | None: typed in from the manual, the table checked against the rendered page |

Columns: `part_number`, `line` (`1ac` or `3ac`), `rated_power_w`, `continuous_braking_power_w`, `peak_braking_power_w`, `braking_energy_j` (the internal braking resistor's), `dc_link_overvoltage_v` (410 V on 1 AC, 820 V on 3 AC). The 100 W converter has no internal resistor (the table's footnote). Not published in any Siemens document found (D 32 07/2021 and 02/2026, these operating instructions 06/2020 and 09/2022, product data sheets, SiePortal): the DC link capacitance and the braking chopper's switch-on threshold.

### `siemens/dc_link_measured.csv`

| | |
| --- | --- |
| Source | Measured on real drives, not catalogue data: one unit each, on one day. The file's header gives the method and the numbers it came from |
| A row is | One converter article with a measured DC link. 1 row: a 6SL5310-1BE10-8DF0 (S210, new generation, firmware 6.4) with a 1FK2204-5AF, 13 November 2026 |
| Script | None: worked out from a Startdrive trace of a quick stop from 3000 rpm |

Columns: `part_number`, `date`, `firmware`, `dc_link_capacitance_f` (139 µF), `dc_link_capacitance_low_f` and `_high_f` (113 and 164 µF: the energy between two ways of integrating it, the winding resistance taken as 3 to 6 ohm), `braking_threshold_above_v` (632: the stop did not reach the chopper), `idle_dc_link_v` (547 on a 400 V line). A drive by name sets `dc_link_capacitance_uf` from it.

### `siemens/motors_1fk2_s120.csv` and `siemens/dynamic_braking_1fk2.csv`

| | |
| --- | --- |
| Source | Siemens, 1FK2 Synchronous Motors for SINAMICS S120, Configuration Manual, 02/2022, A5E46927724B AD. Motors: chapter "Data sheets and characteristics". Braking: section 5.4.3, Table 5-2 |
| A row is | One 1FK2 motor winding as operated on S120 at 380 to 480 V 3 AC. 33 rows in each file |
| Script | `extract_siemens_1fk2_s120.py` |

Motor columns: `part_number`, `static_torque_nm`, `stall_current_a`, `max_speed_inverter_rad_s`, `max_torque_nm`, `max_current_a`, `thermal_time_constant_s`, `rotor_inertia_with_brake_kgm2`, `rotor_inertia_kgm2`, `mass_with_brake_kg`, `mass_kg`, `rated_speed_rad_s`, `rated_torque_nm`, `rated_current_a`, `rated_power_w`.

Braking columns (armature short-circuit braking): `part_number`, `optimum_resistance_ohm`, `braking_torque_rms_nm`, `braking_torque_rms_with_resistor_nm`, `max_braking_torque_nm`, `braking_current_rms_a`, `braking_current_rms_with_resistor_a`. The manual's conditions: braking from the rated speed with an external inertia equal to the motor's own, calculated from the rated values, without production variance and without iron saturation.

28 of the 33 motors are also in `motors_s210.csv`.

### `siemens/motors_1ft2_s120.csv`

| | |
| --- | --- |
| Source | Siemens, 1FT2 synchronous motors for SINAMICS S120, Configuration Manual, 02/2022, A5E50645584B AC, chapter "Data sheets and characteristics" |
| A row is | One 1FT2 motor winding on S120. 37 rows; 34 of them are also in `motors_s210.csv` |
| Script | `extract_siemens_1ft2_s120.py` |

Same columns as `motors_1fk2_s120.csv`.

### `siemens/gearbox_nrb.csv`, `gearbox_nrk.csv`, `gearbox_nlc.csv`

| | |
| --- | --- |
| Source | Siemens, Planetary gearboxes for 1FK2 and 1FT2 servomotors, Equipment Manual, 04/2025, A5E54515894B AA, chapter 6 |
| A row is | One gearbox size, motor frame size and ratio that the manual lists as available. 329 NRB, 252 NRK and 193 NLC rows: 774 in all |
| Script | `extract_siemens_gearboxes.py` |

Columns: `series`, `size`, `motor_frame`, `stages`, `ratio`, `rated_output_torque_nm`, `max_output_torque_nm`, `emergency_stop_output_torque_nm`, `efficiency_full_load`, `torsional_stiffness_nm_rad`, `backlash_rad`, `inertia_at_input_kgm2`.

- `motor_frame` is the last two digits of the motor's frame size in the article number: 1FK2104 and 1FK2204 have frame `04`.
- Torques are at the output shaft. Rated: continuous. Max: for at most 30 000 output revolutions. Emergency off: at most 1000 times in the gearbox's life.
- Stiffness and backlash are referred to the output; inertia to the input.
- Sizes: NRB040 to NRB160, NRK050 to NRK155, NLC060 to NLC120.
- A gearbox is named in a config as size-frame-ratio, e.g. `NRK070-04-5`.

### `siemens/gearbox_sp.csv`

| | |
| --- | --- |
| Source | Siemens, Planetary gearboxes for 1FK2 and 1FT2 servomotors, Equipment Manual, 04/2025, A5E54515894B AA, chapter 6.5 (SP+ precision planetary gearboxes, made by WITTENSTEIN alpha) |
| A row is | The SP100 with two stages for motor frames 03, 04 and 05, at each of its nine ratios: 27 rows |
| Method | Typed in from the rendered tables (6-61, 6-64, 6-66 to 6-68, 6-70 and chapter 6.5.2) and checked against them; no script |

Columns as the other gearbox files, and `backlash_reduced_rad`: the backlash of the reduced-backlash option (order codes M5 to M7), 3 arcmin for two stages against the standard 5; both are maxima.

- Siemens fits the SP+ to 1FT2 motors only ("Coaxial precision gearbox only for 1FT2", chapter 3.1). The frame code is the 1FT2's.
- As printed, the frames 04 and 05 have a rated torque (347 N m, Table 6-61) above their maximum (278 N m, chapter 6.5.2): the maximum is the geared motor's, 15 N m of a 1FT2105 at ratio 20 and 94 %. WITTENSTEIN's own sheet for the SP+ 100 MF gives T2N 259 N m and T2B 347 N m. Both are kept as Siemens prints them.
- Not in the file: the other SP+ sizes, one stage, TP+, masses, forces, reference speeds (the delta configs take 59 r/min from Table 6-74 by hand) and the no-load torque (WITTENSTEIN's sheet, in the delta configs).
- The simulator does not read the file: a gearbox by name and the search take NRB, NRK and NLC (the search pairs by frame code, which would offer an SP+ to a 1FK2). The delta pickers D2 and D3 write its numbers into their configs.

### `siemens/gearbox_mass.csv`

| | |
| --- | --- |
| Source | Siemens, Planetary gearboxes for S-1FK2 and S-1FT2 servomotors, Configuration Manual, 01/2023, A5E51318133B AC, Tables 7-12 (NRB), 7-33 (NRK), 7-54 (NLC), from cache.industry.siemens.com |
| A row is | One gearbox size, group of motor frame sizes and ratio: 445 rows |
| Script | `extract_siemens_gearbox_mass.py` |

Columns: `series`, `size`, `motor_frames` (the frames that share the manual's column, separated by blanks), `stages`, `ratio`, `mass_kg`. This is an older edition than the 04/2025 equipment manual of the other gearbox files; every one of their 774 gearboxes has a mass here (a test checks it). The gearbox is named as in the other files, size-frame-ratio.

### `megadyne/belts.csv` and `megadyne/belt_tooth_load.csv`

| | |
| --- | --- |
| Source | Megadyne (Ammega), MEGALINEAR technical handbook, 02.2026 (megadynegroup.com, `md_manu_en_megalinear_web.pdf`), the page of each profile |
| A row is | Belts: one profile and standard width, open-ended, standard steel cords; 43 rows of T5, AT3, AT5, AT10, MTD3, MTD5, MTD8. Tooth load: one profile and speed, 18 speeds per profile |
| Script | `extract_megadyne.py` |

Belt columns: `profile`, `pitch_m`, `width_m`, `mass_kg_m`, `max_pull_n` (the allowable pull, a quarter of the breaking strength), `breaking_strength_n`, `elongation_at_max_pull` (4 mm/m for all). Tooth load columns: `profile`, `speed_rad_s`, `specific_tooth_load_n_m` (the "Fp spec" per cm of width and tooth in mesh). MTD is Megadyne's HTD-compatible profile: Bahr's 3M, 5M and 8M belts are taken as MTD3, MTD5, MTD8. A belt is named as profile-width, e.g. `AT5-25`; a width between two standard ones is interpolated linearly, one outside them in proportion to the nearest (Bahr's 5M09, 8M12, 8M48 and 8M70 are not standard widths). These are the same figures the EXCT-30 config's belt tension and tooth table were read from by hand.

### `siemens/motors_s120.csv`

| | |
| --- | --- |
| Source | Siemens catalogue D 21.4, SINAMICS S120 and SIMOTICS, 2017, selection and ordering data of the SIMOTICS S synchronous motors |
| A row is | One motor winding: the article number up to the DC link code, e.g. `1FK7042-2AF7`. 167 rows: 62 1FK7 and 105 1FT7. All for a DC link of 510 to 720 V |
| Script | `extract_siemens_d21_4.py` |

Columns: `part_number`, `series`, `version`, `cooling`, `shaft_height_m`, `rated_speed_rad_s`, `rated_values_at_speed_rad_s`, `rated_power_w`, `static_torque_nm`, `rated_torque_nm`, `rated_current_a`, `pole_pairs`, `rotor_inertia_kgm2`, `mass_kg`, `efficiency`, `stall_current_a`, `calculated_power_w`, `paired_drive_rated_current_a`, `paired_drive`.

Ratings are for a winding temperature rise of 100 K. `rated_values_at_speed_rad_s` is filled where a footnote of the catalogue says the rated values refer to another speed. `paired_drive` is the booksize Motor Module the catalogue pairs with the motor; the `7` and the `.` in it are placeholders.

### `siemens/motors_s120_electrical.csv`

| | |
| --- | --- |
| Source | The Siemens data sheet of each motor, generated by Siemens' data sheet service for the full article number in the column `data_sheet_part_number` |
| A row is | One motor of `motors_s120.csv` whose data sheet has been fetched. 166 of the 167 |
| Script | `fetch_siemens_datasheets.py` |

Columns: `part_number`, `data_sheet_part_number`, `poles`, `static_torque_60k_nm`, `stall_current_60k_a`, `max_speed_mechanical_rad_s`, `max_speed_inverter_rad_s`, `max_torque_nm`, `max_current_a`, `torque_constant_nm_a`, `voltage_constant_v_s_rad`, `phase_resistance_20c_ohm`, `phase_inductance_h`, `electrical_time_constant_s`, `mechanical_time_constant_s`, `thermal_time_constant_s`, `shaft_torsional_stiffness_nm_rad`.

The torque constant is at a winding temperature rise of 100 K. The voltage constant is at 20 °C, line to line, RMS, per 1000 r/min. The resistance is of one phase at 20 °C. The inductance is the rotating field inductance. The options in the full article number (encoder, shaft, brake) do not change these values.

This is the only file with winding resistance and inductance. It covers the 1FK7 and 1FT7, which no config uses yet.

### `siemens/motor_modules_s120.csv`

| | |
| --- | --- |
| Source | Siemens Catalog News D 21.4 N, SINAMICS S120 and SIMOTICS, October 2017, chapter 7, "Technical specifications" |
| A row is | One S120 Motor Module in booksize format with internal air cooling, for a DC link of 510 to 720 V. 19 rows |
| Script | `extract_siemens_d21_4n.py` |

Columns: `part_number`, `axes`, `duty_type`, `rated_output_current_a`, `base_load_output_current_a`, `max_output_current_a`, `rated_power_w`, `pulse_frequency_hz`, `dc_link_capacitance_f`, `power_loss_typical_w`, `power_loss_max_w`.

`axes` is 1 for a Single and 2 for a Double Motor Module; currents and rated power of a Double are per axis, its DC link capacitance and power loss are for the module. `duty_type` is C (continuous load) or D (discontinuous load, higher maximum current).

### `siemens/line_modules_s120.csv` and `siemens/braking_s120.csv`

| | |
| --- | --- |
| Source | Siemens Catalog NC 82, SINUMERIK MC, SINAMICS S120 and SIMOTICS, February 2020, chapter 4: Line Modules; DC link components, Braking Module and braking resistors |
| A row is | Line modules: one S120 Line Module in booksize format with internal air cooling, 380 to 480 V 3 AC. 13 rows: 5 Smart, 5 Active, 3 Basic. Braking: the Braking Module and 4 braking resistors |
| Script | `extract_siemens_nc82.py` |

Line module columns: `part_number`, `infeed_type`, `continuous_power_w`, `s6_power_w`, `max_power_w`, `dc_link_current_540v_a`, `dc_link_current_600v_a`, `s6_dc_link_current_a`, `max_dc_link_current_a`, `dc_link_capacitance_f`, `power_loss_w`, `max_braking_power_w`, `continuous_braking_power_w`, `braking_threshold_v`, `min_braking_resistance_ohm`. The `braking_*` columns are the chopper inside the 20 kW and 40 kW Basic Line Modules; empty for all others.

Braking columns: `part_number`, `kind` (`braking_module` or `braking_resistor`), `for_part_number`, `resistance_ohm`, `continuous_power_w`, `peak_power_w`, `activation_threshold_v`, `dc_link_capacitance_f`, `load_duration_s`, `cycle_duration_s`, `load_duration_2_s`, `cycle_duration_2_s`.

These two files are where the DC link estimates of `exct-30.toml` come from: 540 V on a 400 V line, about 44 µF per kW of rated power, a chopper threshold of 770 V. The S210 converter of that config is not in them; see [what is missing](#what-is-missing-and-why).

### `siemens/motor_bearings_1fk2.csv`

| | |
| --- | --- |
| Source | Siemens, operating instructions "SINAMICS S210 servo drive system with S-1FK2 and S-1FT2", 09/2022, A5E41702836B AL, section 12.1.12 (the 1FK2 configuration manual for S120, 02/2022, section 3.6.2, has the same diagrams) |
| A row is | One frame size of the 1FK2, read off its radial force diagram at one distance from the shaft shoulder and one mean speed |
| Extraction | Read off the diagram by hand (`manual:`). Only the 1FK2205 so far: the diagrams are images, and nothing here renders them; the other frame sizes are left out, not guessed |

| Column | Meaning |
| --- | --- |
| `frame` | The frame size, e.g. `1FK2205`: every motor of that size |
| `radial_force_n` | Radial force the bearings allow for `bearing_life_h` at `mean_speed_rad_s`, at `force_distance_m` from the shoulder |
| `force_distance_m`, `mean_speed_rad_s`, `bearing_life_h` | Where, how fast and for how long: 20 mm, 1500 r/min, 25000 h for the 1FK2205's 1000 N |
| `equivalent_load_rating_n` | `[estimate]`: the C of one ball bearing that lasts that long at that force and speed, C = F · (L10h · 60 · n / 10⁶)^(1/3) with n in r/min: 13104 N. Siemens gives no load rating; this C stands for the bearings at that point of force only. The configs' `ratings.motor_bearing_load_rating_n` takes it where a pulley sits on the motor shaft (none of the shipped configs) |

No command reads this file; the simulator takes C from the config. It is data for `motor_bearing_life_h`, checked by the validator like every table.

### `bahr/belt_axes.csv`

| | |
| --- | --- |
| Source | Bahr Modultechnik GmbH (IMI Norgren), data sheets "Linearsystem \<series\>", German editions, from cdn.norgren.com. Editions 11/25, 04/26 and 05/26; each row carries its sheet number, edition and source file |
| A row is | One series and size of a linear axis driven by an HTD toothed belt. 69 rows from 15 series: 59 single axes, 7 XY gantries (ELZU), 3 X/Z gantries (ELZI, a T-gantry) |
| Script | `extract_bahr.py` |

Columns: `series`, `size`, `kind` (`axis`, `xy_gantry`, `xz_gantry`), `belt`, `belt_pitch_m`, `belt_width_m`, `feed_per_rev_m`, `pulley_teeth`, `no_load_torque_nm`, `no_load_torque_count`, `max_speed_m_s`, `feed_force_continuous_n`, `feed_force_peak_n`, `fx_static_n` … `mz_dynamic_nm` (permissible loads on the slide, static and dynamic), `mass_at_zero_stroke_kg`, `mass_per_stroke_kg_m`, `mass_per_stroke_z_kg_m`, `moved_mass_x_kg`, `moved_mass_y_kg`, `moved_mass_z_kg`, `max_length_one_piece_m`, `max_length_x_m`, `max_length_y_m`, `max_length_z_m`, `source` (the data sheet the row was read from, a source in the metadata with its sheet number, edition and file).

- `feed_force_*`: belt pull, continuous and for 0.2 s.
- The sheets want the sum of Fy/Fy_dyn + Fz/Fz_dyn + Mx/Mx_dyn + My/My_dyn + Mz/Mz_dyn to stay at or below 1.
- `no_load_torque_count` is 2 where the sheet prints "2 x 0,4" (the X/Z gantry); whether that is per motor is not said.
- `max_speed_m_s`: the sheets give no conditions.

No config uses a Bahr axis yet. These sheets publish what Festo does not for the EXCT: permissible loads and moments on the slide and the belt's pull.

## Other makers

Added 14 November 2026, so that the search can say which drive train works best across makers and not only among Siemens'. One table of motors and one or two of drives per maker, all in the same columns (the schema's `servo_motor` and `drive` with the columns `vendor`, `line_voltage_v`, `line_resistance_20c_ohm`, `line_inductance_h`, the drives' DC link and braking columns and a `source` per row). Key: part number and line voltage; a motor rated at two voltages has two rows. Every value is from the document named in the table's metadata, page or table given there; none is an estimate. The extraction scripts are `catalog/tools/extract_<vendor>.py` where there is one; Mitsubishi, Yaskawa and the drive tables of Beckhoff, Kollmorgen and Rexroth were typed in from the tables their metadata names.

| Maker | Files (rows) | Documents | Supplies | Pairing |
| --- | --- | --- | --- | --- |
| Beckhoff | `motors_am8000.csv` (300: AM80xx, AM85xx, with and without fan cover, 400 and 480 V), `drives_ax8000.csv` (12), `drives_ax5000.csv` (16) | AM8000/AM8500 operating instructions 4.9.1 (2023-09) ch. 4.4 to 4.9; AX8000 operating instructions 2.4.0 (2026-10) ch. 4.3, 4.4; AX5000 system manual HW2 3.2.3 (2023-09) ch. 6.2 | `beckhoff-400v`, `beckhoff-480v` | None published (Beckhoff's tabular overview needs a login): by stall current |
| B&R | `motors_8ls.csv` (109: 8LSA..-3), `drives_acopos_p3.csv` (34) | 8LS..-3 user's manual V2.51 MAMOT2-ENG (2020-12) ch. 2.14; ACOPOS P3 user's manual V1.40 MAACPP3-ENG (2025-05) ch. 2.2 | `br-400v` | The manual's ACOPOS P3 per motor (the 1-axis module of that size); four motors print "-" |
| Kollmorgen | `motors_akm.csv` (162), `motors_akm2g.csv` (134), 240 and 400 V; `drives_akd.csv` (14) | AKM selection guide KM_SG_000077 Rev F (2025); AKM2G selection guide KM_SG_000315 Rev E (2025); AKD installation manual 903-200003-00 AG (2022-05) ch. 7.6, 7.7, 7.13 | `kollmorgen-240v`, `kollmorgen-400v` | AKM2G: the guide's system performance tables, the smallest AKD that reaches the winding's full continuous and peak torque. AKM: none published, by stall current |
| Lenze | `motors_mcs.csv` (54), `motors_m850.csv` (15), `drives_i950.csv` (18) | MCS and m850 project planning 7.0 (2024-08); i950 cabinet project planning Gen B 3.0 (2026-08) ch. 7.4 | `lenze-400v` | The selection tables: the smallest i950 whose Mmax reaches the motor's |
| Mitsubishi | `motors_j5.csv` (57: HK-KT, HK-ST, 200 and 400 V), `drives_j5.csv` (44: MR-J5-G and -A) | MELSERVO-J5 catalogue L(NA)03179ENG-L (2026-01); the 1-axis combinations web page (read 2026-11-14) | `mitsubishi-200v`, `mitsubishi-400v` | The smallest amplifier of standard torque on the combinations page |
| Rexroth | `motors_ms2n.csv` (155: self-cooled, fan and water cooled), `drives_ctrlx.csv` (59: XCS2, XCD2, XMS2, XMD2) | MS2N project planning R911347583 ed. 11 (2026-03) ch. 6; ctrlX DRIVE modular R911386579 ed. 17 (2026-08) ch. 10; ctrlX DRIVE compact R912009455 ed. 07 (2026-07) ch. 10 | `rexroth-400v` | None published (Rexroth points to its online configurator): by stall current |
| SEW | `motors_cmp.csv` (57: CMP40 to CMP100 per speed class), `drives_movic.csv` (29: MDA90A, MDD90A, MDX90A) | CMP catalogue 27799468/EN (2023-11) ch. 4.5; MOVIDRIVE system manuals 23061715/EN (2017-03, motor-inverter assignment ch. 3.5.6) and 31986765/EN (2025-03); MOVIDRIVE modular 31984061/EN (2025-03) | `sew-400v` | The 2017 MOVIDRIVE system assignment: the smallest MDX90A with an Mpk at the motor's speed class; none for the 6000 r/min classes |
| Yaskawa | `motors_sigma7.csv` (37: SGM7J, SGM7A, 200 and 400 V), `drives_sigma7.csv` (25: SGD7S) | Rotary servomotor product manual SIEP S800001 36J (2025-08); Sigma-7 200 V and 400 V catalogues (2026-03, 2026-10) | `yaskawa-200v`, `yaskawa-400v` | The manual's and catalogue's combination tables |

How the simulator takes them (`catalog.rs`): the bus voltage is 1.35 times the line voltage (a three-phase bridge; a maker's single-phase supply is not modelled); the torque constant is standstill torque over standstill current; the winding, where the maker prints it terminal to terminal, goes in at half of it per phase, the way the 1FK7's does, and the current loop is retuned for it; the thermal time constant and the pole pairs where printed. A maker that prints no standstill values (Yaskawa, Mitsubishi) has its rated torque and current stand for them; one that prints no rated torque or current (SEW CMP; Kollmorgen, which gives no rated current; Beckhoff at 480 V) has its standstill values stand for those, which overstates the continuous current at speed by a few per cent. A drive by name sets its rated and peak current, its base load current at the S210's 0.85 of the rated [estimate], and from its row the DC link capacitance, the internal braking resistor's continuous and peak power, the chopper's switch-on voltage and the trip level, each where printed; a value a maker does not print stays as the config has it (the EXCT-30's S210 estimates). Mitsubishi prints no peak current of its amplifiers: a Mitsubishi drive takes the peak current of the motor put in before it, which its standard-torque pairing is made to supply [estimate]. Lenze does not say whether its inductance is per phase or terminal to terminal; it is read as terminal to terminal.

What the plausibility checks hold them to (`vendor_motors_are_plausible_and_their_drives_exist` and `vendor_drives_are_plausible` in `checks.py`): the vendor column is the folder's; peak torque and current above 1.2 times the standstill ones; rated torque at most 1.05 times the standstill torque; rated speed at most the maximum; torque constant within 20 % of standstill torque over current (15 % would fail Rexroth's water-cooled MS2N, whose Kt is printed at 20 °C beside M0 and I0 at 100 K); torque constant within 20 % of sqrt(3) times the line-to-line RMS voltage constant (a value off by sqrt(2) or sqrt(3) is a peak or phase value read as RMS line to line); rated power within 12 % of rated torque times rated speed; the brake adds inertia and mass; L / R between 0.1 and 100 ms; thermal time constant between 1 and 200 minutes; the paired drive exists at that voltage and has at least 85 % of the printed rated current (SEW pairs by peak torque, with smaller drives than the stall current, and prints no rated current). Drives: peak at least the rated current; DC link above 1.2 times the line voltage and below the trip level (Rexroth's regulated supply holds 750 V on a 400 V line); chopper above the line's peak and below the trip level. The Python test `test_another_makers_motor_that_does_not_fit_its_own_numbers_fails_the_checks` shows each kind of mistake caught. The Rust tests are in [`catalog_vendors.rs`](../crates/sim-machines/tests/catalog_vendors.rs).

## Life and tooth load data in the configs

The life estimates and the belt's tooth load take their figures from the configs' `[ratings]`, not from a file in `catalog/`. Where they come from a maker's document, that is listed here. No vendor document or page is in the repository; the numbers were read from the makers' own servers on 12 November 2026.

| Figure | Value | Source |
| --- | --- | --- |
| Gearbox gearing: maximum torque | M2max for at most 30000 output revolutions; M2N continuous; emergency-off torque at most 1000 times | Siemens, equipment manual "Planetary gearboxes for 1FK2 and 1FT2 servomotors", 04/2025, chapter 6 (the notes of `gearbox_nrb.meta.toml`). `gearbox_gearing_life_h` puts a Wöhler line through M2max at 30000 revolutions with an estimated exponent (8.7, ISO 6336-3's tooth root slope for case-hardened gears) and the knee at M2N, and sums the damage by Palmgren–Miner. Siemens' own damage sum over overload partial cycles (configuration manual 01/2023) is not in the repository |
| Gearbox planet bearings: life formula | L10h = 20000 h · (n2B / n2) · (M2N / M2eq)^3.33, M2eq = (Σ \|M\|^3.33 · \|n\| · t / Σ \|n\| · t)^0.3 | Siemens, configuration manual "Planetary gearboxes for S-1FK2 and S-1FT2 servomotors", 01/2023, A5E51318133B AC, section 6.3.3 (cache.industry.siemens.com). The 04/2025 equipment manual of the gearbox files could not be downloaded; whether it changed these figures was not checked |
| Reference speed n2B of the NRB060, ratio 5 | 175 r/min | The same manual, Table 7-15 (page 99). Only this value went into the configs (`gearbox_reference_speed_rpm`); the gearbox files have no such column, so a gearbox by name does not set it |
| Output bearing of the NRB gearboxes | Average radial force for L10h = 30000 h at 100 r/min, force in the middle of the shaft: NRB040 160 N, NRB060 340 N, NRB080 650 N, NRB120 1500 N, NRB160 4200 N (for 20000 h: 200, 400, 750, 1750, 5000 N); life with another load L10h = 30000 h · (F_R,eq allowed / F_R,eq)³ | The same manual, Table 7-14 (page 95) and section 6.3.4. The default configs' `pulley_bearing_load_rating_n` is the NRB120's, C = 1500 N · (30000 · 60 · 100 / 10⁶)^(1/3) = 8469 N; the 20000 h figure gives 8631 N, 2 % apart |
| Belt teeth: method | Effective pull · service factor ≤ specific tooth load · width in cm · teeth in mesh (at most 12 for open-ended belts) | Megadyne, MEGALINEAR technical handbook, 02.2026, page 9 (megadynegroup.com) |
| Belt teeth: specific tooth load of open-ended AT5 | 35 N/cm at 0 to 40 r/min, 34 at 60 to 100, 32 at 200, 31, 30, 29 at 300, 400, 500, 27 at 750, 26 at 1000, 24 at 1500, 22 at 2000, 19 at 3000, 18 at 4000, 16 at 5000, 13 at 8000 r/min | The same handbook, page 52 (`belt_tooth_load_rpm`, `belt_tooth_load_n_cm` in all three configs) |
| Belt teeth: service factor | 1 steady load; 1.4, 1.7, 2 for low, average and high shock loads | The same handbook, Table 4 (page 13). The configs take 1.4 |
| Belt life | None published | Neither Megadyne's handbook, Gates' "Thermoplastic Polyurethane Belt Program" (catalogue 20166, 2025) nor BRECO's AT5 data sheets give a life in hours or load cycles, or a Wöhler curve, for these belts; they size by tooth load, cord tension and minimum pulley teeth. Optibelt's documents could not be reached (their links redirect). Gates' AT5 figure is of the same size as Megadyne's: an allowable effective force of 1200 N at 25 mm with 12 teeth in mesh is 40 N/cm per tooth at standstill, against Megadyne's 35 |
| 1FK2 motor bearings | Deep-groove ball bearings, lubricated for life, about 25000 h at medium load (less with continuous running at the highest speed, shock or frequent reversing, by up to half); allowable radial force at the shaft end for L10h = 25000 h, e.g. 1FK2205 about 1000 N at 20 mm from the shoulder at 1500 r/min (read off the diagram), an equivalent C of 13104 N | Siemens, operating instructions "SINAMICS S210 servo drive system with S-1FK2 and S-1FT2", 09/2022, A5E41702836B AL, sections 11.1.1 and 12.1.12; `siemens/motor_bearings_1fk2.csv`. Used by `ratings.motor_bearing_load_rating_n` where a pulley sits on the motor shaft; in no shipped config does the belt pull on one |
| Life modification factor a_ISO | a_ISO of radial ball bearings from κ = ν / ν1 and e_C · C_u / P; ν1 from n and D_pw; e_C from Table 13; C_u ≈ C0 / 22 below 100 mm mean diameter | ISO 281:2007, sections 6.3.3.2, 6.3.3.3 and Annex B. The configs' viscosity, contamination factor, C_u and D_pw are estimates (no maker of the pulley bearings publishes them) |
| Rating life of a ball bearing | L10 = (C / P)³ million revolutions, L10h = 10⁶ / (60 n) · L10; for a load that changes, P = cbrt(Σ F³ n t / Σ n t) | ISO 281, as SKF ("Y-bearings and Y-bearing units", 2013) and Schaeffler (TPI 301, and the ball screw catalogue's equivalent load) print it |

## How the files were extracted and checked

**Extraction.** Each Siemens and Bahr file is written by a script in [`catalog/tools/`](../catalog/tools). The vendor PDF is turned into text with `pdftotext -table` (xpdf 4.x), and the script reads the tables from that text. The scripts are strict: a row that does not have one value per column, a motor printed twice with different values, or a motor with only one of its two rows stops the script, so a changed edition cannot produce shifted values silently. The scripts also write the comment header of each file. The vendor documents are not part of the repository.

**Checks.** The catalogue's validator (`python catalog/tools/catalog.py validate`, run by the CI workflow `catalogue.yml`) checks every table against its JSON Schema (types, units, no unknown column, no negative quantity), every value for its source, the key columns for duplicates, and the numbers against their own physics and against each other. A value that landed in the wrong field or row breaks one of them. The physics checks are the functions of [`checks.py`](../catalog/tools/cataloglib/checks.py), named as in the table (until November 2026 they were Rust tests of `sim-machines`; exact row counts became minimums, so that a contributor can add rows):

| File | Checked how | Test |
| --- | --- | --- |
| `festo/exct.toml` | Feed constant = π × effective diameter (0.1 %). The coefficients of the torque formula against the T-gantry's own mechanics at 45 degrees: load factor r/√2, motor factor √2/r (1 %), gravity factor g·r/2 (3 %) | `festo_exct_values_agree_with_its_own_geometry` |
| `motors_s210.csv`, `converters_s210.csv` | No article twice; torque per current between 0.15 and 4.5 N m/A; rated power = rated torque × rated speed (12 %); rated current at most 1.05 × stall current; every paired converter exists and has at least 85 % of the motor's rated current | `siemens_s210_motors_are_plausible_and_their_converters_exist` |
| `motors_s210.csv` through `apply_motor` | Every motor and supply row reaches its rated speed inside the voltage limit: static torque constant × rated speed below sqrt(3/2) × DC link voltage (540 V or 325 V). Five rows are above the DC link voltage itself | `every_catalogue_motor_reaches_its_rated_point_inside_the_voltage_limit` |
| `motors_1fk2_s120.csv` | Rated power = rated torque × rated speed (8 %). Against `motors_s210.csv`, a second document read by a second script: static torque, stall current, maximum torque and current within 5 %, rotor inertia and mass within 6 % | `siemens_1fk2_data_agrees_between_the_s120_manual_and_the_s210_catalogue` |
| `dynamic_braking_1fk2.csv` | The optimum resistor brakes harder than the bare short circuit; a peak is above an average; a resistor lowers the current | `siemens_1fk2_dynamic_braking_fits_the_motors` |
| `motors_1ft2_s120.csv` | As for the 1FK2, against `motors_s210.csv` | `siemens_1ft2_data_agrees_between_the_s120_manual_and_the_s210_catalogue` |
| `gearbox_*.csv` | Rows complete; a ratio belongs to one number of stages; values sensible where given | `siemens_gearbox_rows_are_complete_and_plausible` |
| `motors_s120.csv` | Power = torque × speed; the paired Motor Module exists and carries the stall current | `siemens_s120_motors_are_plausible_and_their_motor_modules_exist` |
| `motors_s120_electrical.csv` | Against `motors_s120.csv`: poles = 2 × pole pairs; torque constant = static torque / stall current (8 %). In itself: three times the per-phase voltage constant = torque constant (15 %); electrical time constant = L / R (15 %) | `siemens_s120_data_sheets_agree_with_the_catalogue` |
| `motor_modules_s120.csv` | Type rating about 0.54 kW per ampere; the D type has the higher maximum current | `siemens_s120_motor_modules_are_plausible` |
| `line_modules_s120.csv` | Power = DC link voltage × DC link current; a bigger module has more capacitance; chopper peak power = U² / R at the threshold | `siemens_s120_line_modules_are_plausible` |
| `braking_s120.csv` | Peak power = U² / R at the threshold; over a duty cycle the peak power averages to no more than the continuous power; each resistor's module exists | `siemens_s120_braking_resistors_fit_their_modules` |
| `bahr/belt_axes.csv` | Rows unique; belt code agrees with pitch and width; feed per revolution = teeth × pitch; continuous feed force against the static Fx; the gantries state their moved masses and lengths | the four `bahr_*` checks |
| `gearbox_mass.csv` | Every gearbox of the three gearbox files has a mass; a bigger size of a series is heavier at the same frame and ratio; NRK070, frame 04, ratio 5 is 1.4 kg as in Table 7-33 | `every_catalogue_gearbox_has_a_mass_and_bigger_ones_weigh_more` |
| `megadyne/*.csv` | Per profile EA per mm of width within 25 % of its mean; breaking strength four times the allowable pull (10 %; 3.5 times at 150 mm, as printed); mass growing with width; tooth load falling with speed | `megadyne_belts_are_plausible` |
| `motor_modules_s120.csv` against the S120 motors | Every S120 motor's paired Motor Module exists and has at least 85 % of its rated current | `s120_motors_by_name_bring_winding_rated_values_and_their_motor_module`, a Rust test in `catalog_parts.rs` |

That the simulator reads the files correctly is checked by Rust tests. [`catalog_golden.rs`](../crates/sim-machines/tests/catalog_golden.rs) holds every value the simulator takes from the catalogue (every motor at every supply, every drive, gearbox, belt and axis by name, pairings, masses) to a snapshot written before the move to schema 1, and so shows that the move changed no result; [`catalog_dir.rs`](../crates/sim-machines/tests/catalog_dir.rs) checks `--catalog DIR`. Besides them: `a_motor_by_name_gives_the_values_of_the_hand_written_config`, `a_gearbox_by_name_sets_ratio_stiffness_backlash_and_ratings`, `parts_named_in_a_config_behave_like_their_numbers` and `exct_30_config_agrees_with_the_catalogue`; for drives, belts, axes and the S120 motors the tests of [`catalog_parts.rs`](../crates/sim-machines/tests/catalog_parts.rs).

What these checks cannot find: a number that was misread into another plausible number in a column with no relation to check it against.

## Misprints and inconsistencies found in the vendor documents

The files keep what is printed. Each of these is named in the header of its file, and the test that would trip on it skips that one value.

| Document | What is printed | Why it looks wrong |
| --- | --- | --- |
| Festo EXCT catalogue 2023/01 | The sample calculation uses a constant of 39.2 and a gravity mass of 2.14 kg for the EXCT-15 with 200 mm Z stroke | Its own formula table gives 47.5 and 2.6. The file has the formula table's values |
| Siemens D 32, 01/2024 | `rated_current_230v_a` of the 1FT2206-2AC is 3.75 A | Above its stall current of 2.8 A and above its converter's 3 A. Most likely a misprint for 2.75 A. A motor by name takes the catalogue's rated current up to 1.05 times the stall current (the most any other row has), so 2.94 A here |
| Siemens D 32, 01/2024 | The 230 V rated power row of the 1FT2 shaft height 100 motors is one value short | The file leaves the field empty |
| Siemens 1FK2 manual 02/2022 | Rotor inertia of the 1FK2308-3AB printed as "1,60,1" | Unreadable. The file leaves the field empty |
| Siemens 1FK2 manual 02/2022 | Braking current with resistor of the 1FK2306-2AC is 6.5 A | Above the 6.1 A without the resistor; for every other motor it is about 10 % below |
| Siemens 1FT2 manual 02/2022 | Rated speed 3000 r/min for the 1FT2105-4AH and 1FT2206-4AH, 2000 r/min for the 1FT2208-4AF | The manual's own rated torque and power, and the S210 catalogue, give 4500 and 3000 |
| Siemens NC 82, 02/2020 | The 55 kW Smart Line Module 6SL3130-6TE25-5AA3 has 138 A for S6 duty and 178 A maximum at 71 kW and 91 kW | That would be a DC link of about 510 V; every other module has power / 600 V in these rows |
| Siemens NC 82, 02/2020 | Peak power of the braking resistor 6SN1113-1AA00-0DA0 is 25 kW at 17 Ω | U² / R at the 770 V threshold would be 35 kW; for the other three resistors it fits |
| Beckhoff AM8000 4.9.1 | Torque constant of the AM8022-0D, AM8022-0E, AM8023-0E 22 to 28 % below sqrt(3) × Ke | Their Kt fits M0 / I0; the AM802x family runs about 20 % low. Not checked (`VENDOR_MISPRINTS` in `checks.py`) |
| Beckhoff AM8000 4.9.1 | Rated power of AM8051-0J-fan, AM8551-0J-fan (480 V), AM8052-0N-fan, AM8552-0N-fan (400 and 480 V), AM8073-0U-fan (480 V) | 14 to 25 % off rated torque × rated speed. Not checked |
| Kollmorgen AKM guide Rev F | Ke of the AKM64Q at 240 V 0.64 V per 1000 r/min | Its Kt of 0.99 N m/A implies about 60. Not checked |
| Kollmorgen AKM guide Rev F | Peak torque of the AKM44G 23.3 N m, peak current of the AKM83T 263.9 A | The other windings of the frame are near 20 N m; the AKM82T and AKM84T have 144 and 201 A. Within the checks' bounds, kept |
| Kollmorgen AKM guide Rev F | The quick-guide table and the frame tables disagree in places (AKM8x masses, AKM52L power, AKM64Q at 400 V) | The rows follow the frame tables; the metadata lists each |
| B&R 8LS V2.51 | Brake mass 0 kg for the 8LSA5A/5B/5C and 8LSA7x, which list a brake inertia | Kept: their mass with brake is their mass |
| Lenze MCS 7.0 | Brake motor inertia of the MCS19J 135 kg cm², less than 105 plus the brake's 31.8 | Kept |
| Lenze i950 Gen B 3.0 | The C130 is rated 209 A, below the smaller C110's 212 A | Kept |
| Rexroth ctrlX DRIVE ed. 17 | XCS2-W0090: internal braking resistor 11 Ω, below its own minimum external resistance of 11.2 Ω | Kept |
| SEW MOVIDRIVE system 2017 | The type list prints MDX90A-0950 and 45 A for the 0460 | Its own technical data and the 2025 manual say 0095 and 46 A, which the file has |
| Siemens data sheet service | Voltage constant of the 1FT7044-1AF7 is 49.0 V per 1000 r/min | Less than half of what its torque constant of 1.79 N m/A implies |
| Siemens data sheets against D 21.4 | The data sheets give the sixteen 1FT7 High Dynamic motors 6 poles; the catalogue table gives them 5 pole pairs | Two Siemens sources disagree. Both files keep what their source prints |
| Siemens D 21.4, 2017 | Some motors are still paired with 45 A and 60 A Motor Modules | The Catalog News D 21.4 N replaced those by C types |
| Bahr data sheets | The continuous feed force is not the static Fx for several series (900 N beside 894 N; ELZU 100: 3600 N beside 4000 N; LLZS 60: 1050 N beside 1073 N) | Rounded differently in places |
| Bahr data sheets | ELZT 80S has a lower no-load torque (1.2) than ELZT 80 (1.3) | Kept as printed |
| Bahr data sheets | The footer of the DLZPVI sheet carries the number of the DSZPVI sheet; page 2 of the ELZU sheet says 21.20.200 | Kept as printed |

Other notes from the headers that are not misprints: three 1FK2 data sheets (1FK2306-4AC, 1FK2308-3AB, 1FK2308-4AB) head their rated data "3AC 400 ... 480 V"; the maximum power of the 6SL3130-7TE25-5AA3 is 91 kW, with 110 kW in brackets for use with an Active Interface Module that is not in the file.

## What is missing, and why

| Missing | Why | What it means |
| --- | --- | --- |
| **Winding resistance and inductance of the 1FK2 and 1FT2** | Siemens publishes them in no catalogue, data sheet or manual on its servers | They are estimates in `exct-30.toml` (2 Ω, 10 mH, bounded 1.4 to 2.6 Ω and 7 to 13 mH), and a motor by name leaves them as the config has them. The current loop's behaviour and the copper loss rest on them. A support request, the drive's parameters p0350 and p0356, or a measurement would give them |
| Torque and voltage constant of the 1FK2 and 1FT2 | Not in the catalogue | The torque constant is taken as static torque over stall current. For the 1FK7 and 1FT7, where the data sheets print it, that agrees within 8 % |
| Rated current of a motor by name | In the catalogue per supply | `motor.rated_current_a` is the catalogue's own figure, not rated torque over the static torque constant, which is up to 18 % lower (1FK2204-5AK at 400 V) and made the winding rise come out up to 40 % too high |
| Thermal time constant of 34 of the 96 S210 motors | They are in neither configuration manual of 02/2022 | A motor by name keeps the config's thermal time constant for them |
| DC link and braking resistor data of the S210 converters | Not in D 32 | The DC link ratings of `exct-30.toml` are estimates by the S120 data |
| The data sheet of the 1FK7022-5AK7 | The data sheet service has none under any article number tried | 166 of 167 rows in `motors_s120_electrical.csv` |
| A complete D 21.4 newer than 2017 | Siemens does not seem to have issued one | The 1FK7 and 1FT7 selection data is from 2017 |
| Gearbox input speed limits, radial and axial forces | Not extracted from the gearbox manual (the mass is, from the 01/2023 edition) | `ratings.gearbox_max_input_speed_rpm` and `ratings.shaft_max_radial_load_n` are placeholders in the configs |
| Backlash of 24 NRB rows, inertia of 3 NRK rows | The manual leaves them open | A gearbox by name keeps the config's value there |
| The angled gearboxes (NRBW, NRKW, NLCW) and the precision gearboxes SP+ and TP+ | Not extracted | |
| EXCT: friction torque over speed, maximum acceleration over load and Z position, permissible support span, dimensions | The Festo catalogue gives them only as graphs or drawings | Friction of the EXCT-30 is an estimate |
| EXCT: allowable belt tension, guide and pulley shaft ratings | Festo does not publish them | All three are estimates in the EXCT-30 config. The belt tension: 2140 N. Festo publishes no allowable belt tension for the EXCT. It states the 5 mm pitch, the steel cords, the 30 mm width and the stretch of the belt: 0.045 % at 300 N, which is 667 kN per unit strain. Megadyne's Megalinear handbook lists for 5 mm pitch steel-cord polyurethane belts, at 4 mm/m stretch: T5 840 N at 25 mm and 1060 N at 32 mm (about 250 kN per unit strain at 30 mm), AT5 1805 N at 25 mm and 2275 N at 32 mm (about 535 kN). Festo's belt is somewhat stiffer than AT5 with standard steel cords and far stiffer than T5, so the limit is the AT5 figure between the two widths: 2140 N at 30 mm. The figures were read from the handbook itself (average values for open-ended belts, a quarter of the breaking strength); that Festo's belt has cords of that class is concluded from the stretch, not stated by Festo. The teeth give way before the cords do: that is judged by `pulley_max_torque_nm`. |
| S120 motors: maximum torque, current and speed in D 21.4 | Given as characteristic curves | Taken from the data sheets instead |
| Bahr: many series | Their tables need special cases: no-load torque with and without cover band, no feed force rows, a second belt width, differently laid out tables. The header of the file lists them | 15 series are in the file |
| Bahr: second moments of area, dimensions, order codes, the second drive version of ELZ and ELVZ | Not extracted | |
| Motor–drive pairing of Beckhoff, Kollmorgen AKM and Rexroth | In no public document (Beckhoff's overview needs a login, Rexroth points to a configurator) | Paired by the stall-current rule, as the 1FK2 on S120 |
| Rated torque, current and power of the SEW CMP; rated current of the Kollmorgen motors; standstill values, winding data and thermal time constants of Yaskawa and Mitsubishi | Not printed | Standstill values stand for rated ones and the other way round (see [other makers](#other-makers)); winding and thermal data stay the config's |
| SEW CM3C, Kollmorgen AKD2G, B&R 8LSA..-4 and 8LSC/O/P, Lenze 230 V windings and the i950 at 480 V, Yaskawa SGM7G/SGM7P, Mitsubishi HK-MT/RT | CM3C: SEW's catalogue links answer 403; the others not extracted yet | |
| DC link capacitance of the Lenze i950 and of the Beckhoff, SEW and smaller Rexroth inverters; switch-on and trip voltages of most makers; braking data of the axis modules of modular systems (AX8000, MOVIDRIVE modular, ctrlX XMS/XMD) | Not printed, or in the supply modules, which are not in the tables | The config's values stay (the EXCT-30's S210 estimates) |
| Peak current of the Mitsubishi amplifiers | Not printed | The paired motor's peak [estimate] |
| Base load current of every other maker's drive | Not printed | 0.85 of the rated current, the S210's [estimate] |
| Prices | In no file: Siemens' Industry Mall refuses automated access, and no catalogue read here prints a price | The search uses installed weight as a stand-in (see [above](#what-stands-in-for-the-price)) |
| Converter and Motor Module weights | Not extracted from D 32 and D 21.4 N | Not counted in the search's `cost_kg` |
| Guides as parts by name | No guide catalogue file | Guide ratings stay as the config has them |
| Bahr: rated payload of the gantries, moved masses of the single axes, belt stretch | Not in the sheets | The axis search judges the payload by the slide's dynamic Fz against Z axis and payload; a Bahr single axis keeps the config's carriage; Bahr's belt EA is Megadyne's for the same profile and width |
| Festo EXCT: moved masses one by one | Festo gives the torque formula's lumped constant and the Z mass | The carriage of an EXCT by name is fitted to the formula with the machine at hand (7.136 kg for the EXCT-30, the hand-written config has 7.16 kg) |

## Rerun an extraction on a newer edition

The vendor documents are not in the repository. Get the PDF from the vendor, turn it into text, and run the script through the catalogue's pipeline, which converts its output to schema 1, writes the tables and validates them. All commands are run from `catalog/`. They need Python 3.11, `jsonschema`, and `pdftotext` from xpdf 4.x.

| File | Commands |
| --- | --- |
| `motors_s210.csv`, `converters_s210.csv` | `pdftotext -table -enc UTF-8 catalogue.pdf catalogue.txt` then `python tools/catalog.py extract siemens_d32 catalogue.txt` |
| `motors_1fk2_s120.csv`, `dynamic_braking_1fk2.csv` | `pdftotext -table -enc UTF-8 manual.pdf manual.txt` then `python tools/catalog.py extract siemens_1fk2_s120 manual.txt` |
| `motors_1ft2_s120.csv` | the same with `siemens_1ft2_s120` |
| `gearbox_nrb.csv`, `_nrk.csv`, `_nlc.csv` | the same with `siemens_gearboxes` |
| `motors_s120.csv` | the same with `siemens_d21_4` |
| `motor_modules_s120.csv` | the same with `siemens_d21_4n` |
| `line_modules_s120.csv`, `braking_s120.csv` | the same with `siemens_nc82` |
| `siemens/gearbox_mass.csv` | The 01/2023 configuration manual from `https://cache.industry.siemens.com/dl/files/596/109802596/att_1132680/v2/Plan_gear_for_servo_motors_config_man_0123_en-US.pdf`, then `pdftotext -table -enc UTF-8 manual.pdf manual.txt` and `python tools/catalog.py extract siemens_gearbox_mass manual.txt` |
| `megadyne/belts.csv`, `belt_tooth_load.csv` | The handbook from `https://megadynegroup.com/files/resources/attachments/md_manu_en_megalinear_web.pdf`, then `pdftotext -table -enc UTF-8 md_manu_en_megalinear_web.pdf megalinear.txt` and `python tools/catalog.py extract megadyne megalinear.txt` |
| `bahr/belt_axes.csv` | Download one PDF per name in the script's `SHEETS` list from `https://cdn.norgren.com/pdf/<name>.pdf`. `pdftotext -q -table -enc UTF-8 <name>.pdf` once per sheet, then `python tools/catalog.py extract bahr folder_with_the_txt_files`. A new sheet needs a source in `belt_axes.meta.toml` |
| `motors_s120_electrical.csv` | `python tools/catalog.py extract fetch_siemens_datasheets <folder for the downloaded PDFs>`. It downloads one data sheet per motor of `motors_s120.csv` from Siemens' service. A full run takes hours; downloaded PDFs are kept and not fetched twice. With a further argument `cached` it fetches nothing and writes the file from the PDFs already in the folder |
| `festo/belt_axes.csv`, `belt_axis_torque_formula.csv` | By hand, from the catalogue's tables |

Then:

1. **If the script stops,** the new edition has a row or table it cannot read. It says which. Fix the script; do not patch the output. If the conversion stops, the script wrote a column the map in `cataloglib/legacy.py` does not know.
2. **Update the source** in the table's `.meta.toml`: `document`, `date`, `url`, and the notes.
3. **Review the diff** of the table.
4. **Validate:** `python tools/catalog.py validate`. A misprint that the new edition has fixed will show as a check that names the old one (for example the check that expects the inertia of the 1FK2308-3AB to be empty says "is readable now, check it"). Remove the exception from the check and the note from the metadata.
5. **In the simulator:** rebuild (the tables are compiled in) and run

   ```sh
   cargo test -p sim-machines --test catalog_golden --test catalog --test catalog_parts
   cargo test -p machine-t-gantry
   ```

   `catalog_golden` lists every value the simulator takes from the catalogue that changed; when the change is meant, write its snapshot anew with `UPDATE_GOLDEN=1`. `exct_30_config_agrees_with_the_catalogue` holds the EXCT-30 config to the files, and the search tests depend on which motors exist.

## Motor and drive data added on 14 November 2026

- `siemens/motors_s210_oi.csv`: the thermal time constants of the 1FK2 and 1FT2 from the S210 operating instructions 09/2022 (A5E41702836B AL, from cache.industry.siemens.com), with the 230 V windings and the 1FT2 the S120 manuals lack; where a manual has the motor too both agree (62 motors; the 06/2020 edition's 22 min for 1FK2205-2AF and 1FK2206-2AF are 29 and 21 min here too). Six 1FT2 of the S210 catalogue got their time constant from it; 22 later 1FT2 windings (1FT2108, the 1FT2208-.S. and some 1FT2206/08/10) are in no edition read and still have none. The motors' torque-speed curves and per-motor R, L, k_E of the 1FK2 and 1FT2 are in none of these documents nor in the configurator's data sheet per article (graphs at most); SIZER, which may export curve points, needs an installation and a login and was not used.
- `siemens/encoders_1fk2_1ft2.csv`: the encoders the 1FK2 and 1FT2 are ordered with (22 bit, 26 bit for the 1FT2 only), for `parts.motor_encoder`.
- `siemens/dc_link_trip_s120.csv`: the S120 DC link's overvoltage trip, 820 V, from the booksize equipment manual (formerly typed into the simulator).
- Brake data (inertia and weight with brake) of `motors_s210.csv` and the S120 manuals now reach the config with `parts.motor_brake = true`; the maximum-speed columns set `drive.max_speed_rad_s`.
- Winding resistance and inductance of the 1FK2 and 1FT2 are still published nowhere Siemens serves (checked: the S120 configuration manuals 02/2022, the S210 operating instructions 06/2020, the data sheet service for S210 and S120 variants). A 1FK2 by name gets an inductance estimated from its row of `dynamic_braking_1fk2.csv` (`catalog::apply_motor`); its resistance stays the config's estimate.
