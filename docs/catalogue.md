# Catalogue

[`catalog/`](../catalog) holds vendor data as plain CSV and TOML files. The numbers are the vendors'; the layout and the column names are ours. Units are in the column names. Each file starts with comment lines (`#`) that name its source, explain its columns, and list what was kept as printed although it looks wrong.

- [What the simulator uses](#what-the-simulator-uses)
- [Files](#files)
- [How the files were extracted and checked](#how-the-files-were-extracted-and-checked)
- [Misprints and inconsistencies found in the vendor documents](#misprints-and-inconsistencies-found-in-the-vendor-documents)
- [What is missing, and why](#what-is-missing-and-why)
- [Rerun an extraction on a newer edition](#rerun-an-extraction-on-a-newer-edition)

## What the simulator uses

Only four kinds of file are read by the simulator itself. They are compiled into `sim-components` ([`catalog.rs`](../crates/sim-components/src/catalog.rs)), so the binaries need no path to them, and a changed file takes effect after a rebuild.

| File | Used for |
| --- | --- |
| `siemens/motors_s210.csv` | A motor by name (`[parts] motor`, `--motor`) and the motor search |
| `siemens/motors_1fk2_s120.csv`, `siemens/motors_1ft2_s120.csv` | The thermal time constant of a motor by name. 34 of the 96 S210 motors are in neither file; for them the config's value stays |
| `siemens/gearbox_nrb.csv`, `_nrk.csv`, `_nlc.csv` | A gearbox by name (`[parts] gearbox`, `--gearbox`) and the gearbox search |

`festo/exct.toml` is the source of the `[catalogue]` values in [`exct-30.toml`](../crates/machine-t-gantry/exct-30.toml); a test holds the two together. All other files are data for later use (drives, line modules, braking, S120 motors, Bahr axes). Tests read and check them, but no command uses them yet.

## Files

### `festo/exct.toml`

| | |
| --- | --- |
| Source | Festo catalogue "Linear gantries EXCT", 2023/01 |
| A table is | One size of the EXCT T-gantry: `EXCT-15`, `EXCT-30`, `EXCT-100` |
| Extraction | There is no extraction script for this file in `catalog/tools/`. It has three sizes and was written from the catalogue's tables |

| Key | Meaning |
| --- | --- |
| `stroke_y_min_mm`, `stroke_y_max_mm`, `strokes_z_mm` | Stroke range of the horizontal axis (Festo's Y; the simulator's X) and the Z strokes offered |
| `rated_load_kg` | Tool load plus payload, at maximum dynamic response |
| `max_process_force_z_n` | Highest process force in Z |
| `max_torque_nm`, `max_no_load_torque_nm` | At the pulley; they also apply to third-party motors. The no-load torque is at 0.2 m/s and travel at 45 degrees |
| `max_acceleration_m_s2`, `max_speed_m_s` | "Under ideal conditions" |
| `repetition_accuracy_mm` | |
| `weight_at_zero_stroke_kg`, `weight_per_100_mm_stroke_y_kg`, `_z_kg` | Without rated load, motors, axial kits and mounting kits |
| `servo_motor`, `axial_kit`, `coupling`, `coupling_housing_third_party_motor` | Festo part names |
| `belt.material`, `.pitch_mm`, `.width_mm` | The belt |
| `belt.elongation_percent`, `.elongation_reference_force_n` | The belt stretches by this much at the reference force. Gives EA |
| `belt.effective_diameter_mm`, `.feed_constant_mm_rev` | Pulley. The feed constant is "at 45 degrees travel" |
| `torque_formula` (one per Z stroke): `load_factor`, `motor_inertia_factor`, `constant`, `gravity_factor`, `gravity_mass_kg` | Coefficients of Festo's formula for the required motor torque at 45 degrees travel, the heaviest case |

Festo's formula, from the file header:

```text
M = a · (load_factor · m_L + motor_inertia_factor · J_m + constant) · 1e-3 + gravity_factor · (gravity_mass_kg + m_L) + M_R     [N m]
n = v · 60000 / feed_constant_mm_rev · sqrt(2)                                                                              [1/min]
```

a: acceleration along the path in m/s², v: speed along the path in m/s, m_L: attachment on the Z axis with payload in kg, J_m: motor inertia in kg cm², M_R: friction torque in N m.

The motors couple directly onto the belt pulleys; the catalogue lists no gear unit.

### `siemens/motors_s210.csv`

| | |
| --- | --- |
| Source | Siemens catalogue D 32, SINAMICS S210 servo drive system, January 2024, "Specific technical specifications" of each shaft height |
| A row is | One motor winding: the article number up to the speed letter, e.g. `1FK2104-5AK`. 96 rows: 34 1FK2 and 62 1FT2 |
| Script | `extract_siemens_d32.py` |

Columns: `article`, `series`, `version`, `shaft_height_mm`, `cooling`, `static_torque_nm`, `stall_current_a`, `max_torque_nm`, `max_current_a`, `max_speed_rpm`, `rotor_inertia_kgcm2`, `rotor_inertia_with_brake_kgcm2`, `mass_with_brake_kg`, `mass_kg`; then per supply (`230v` is 200 to 240 V, 1 AC or 3 AC; `400v` is 380 to 480 V 3 AC) `rated_speed_*_rpm`, `rated_torque_*_nm`, `rated_current_*_a`, `rated_power_*_w`; then the paired converters `converter_230v_1ac`, `converter_230v_3ac`, `converter_400v_3ac` and the same for the "SINAMICS S210 (New)" converters (`converter_new_*`).

Currents are RMS. Static values are at standstill. An empty field means the catalogue lists no value, mostly because the motor is not offered at that supply: 84 motors have 400 V data, 88 have 230 V data.

### `siemens/converters_s210.csv`

| | |
| --- | --- |
| Source | The same catalogue D 32, "Technical specifications" of the servo converters |
| A row is | One converter. 24 rows: 12 S210 and 12 S210 (New) |
| Script | `extract_siemens_d32.py` |

Columns: `article`, `series`, `supply`, `line_current_a`, `power_loss_w`, `rated_power_1ac_230v_kw`, `rated_power_3ac_400v_kw`, `rated_power_3ac_240v_kw`, `rated_output_current_a`, `max_output_current_a`. Output currents are RMS; the power unit switches at 8 kHz.

### `siemens/motors_1fk2_s120.csv` and `siemens/dynamic_braking_1fk2.csv`

| | |
| --- | --- |
| Source | Siemens, 1FK2 Synchronous Motors for SINAMICS S120, Configuration Manual, 02/2022, A5E46927724B AD. Motors: chapter "Data sheets and characteristics". Braking: section 5.4.3, Table 5-2 |
| A row is | One 1FK2 motor winding as operated on S120 at 380 to 480 V 3 AC. 33 rows in each file |
| Script | `extract_siemens_1fk2_s120.py` |

Motor columns: `article`, `static_torque_nm`, `stall_current_a`, `max_speed_inverter_rpm`, `max_torque_nm`, `max_current_a`, `thermal_time_constant_min`, `rotor_inertia_with_brake_kgcm2`, `rotor_inertia_kgcm2`, `mass_with_brake_kg`, `mass_kg`, `rated_speed_rpm`, `rated_torque_nm`, `rated_current_a`, `rated_power_kw`.

Braking columns (armature short-circuit braking): `article`, `optimum_resistance_ohm`, `braking_torque_rms_nm`, `braking_torque_rms_with_resistor_nm`, `max_braking_torque_nm`, `braking_current_rms_a`, `braking_current_rms_with_resistor_a`. The manual's conditions: braking from the rated speed with an external inertia equal to the motor's own, calculated from the rated values, without production variance and without iron saturation.

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

Columns: `series`, `size`, `motor_frame`, `stages`, `ratio`, `rated_torque_nm`, `max_torque_nm`, `emergency_off_torque_nm`, `efficiency_full_load`, `torsional_stiffness_nm_arcmin`, `backlash_arcmin`, `inertia_at_input_kgcm2`.

- `motor_frame` is the last two digits of the motor's frame size in the article number: 1FK2104 and 1FK2204 have frame `04`.
- Torques are at the output shaft. Rated: continuous. Max: for at most 30 000 output revolutions. Emergency off: at most 1000 times in the gearbox's life.
- Stiffness and backlash are referred to the output; inertia to the input.
- Sizes: NRB040 to NRB160, NRK050 to NRK155, NLC060 to NLC120.
- A gearbox is named in a config as size-frame-ratio, e.g. `NRK070-04-5`.

### `siemens/motors_s120.csv`

| | |
| --- | --- |
| Source | Siemens catalogue D 21.4, SINAMICS S120 and SIMOTICS, 2017, selection and ordering data of the SIMOTICS S synchronous motors |
| A row is | One motor winding: the article number up to the DC link code, e.g. `1FK7042-2AF7`. 167 rows: 62 1FK7 and 105 1FT7. All for a DC link of 510 to 720 V |
| Script | `extract_siemens_d21_4.py` |

Columns: `article`, `series`, `version`, `cooling`, `shaft_height_mm`, `rated_speed_rpm`, `rated_values_at_rpm`, `rated_power_kw`, `static_torque_nm`, `rated_torque_nm`, `rated_current_a`, `pole_pairs`, `rotor_inertia_kgcm2`, `mass_kg`, `efficiency_percent`, `stall_current_a`, `calculated_power_kw`, `motor_module_rated_current_a`, `motor_module`.

Ratings are for a winding temperature rise of 100 K. `rated_values_at_rpm` is filled where a footnote of the catalogue says the rated values refer to another speed. `motor_module` is the booksize Motor Module the catalogue pairs with the motor; the `7` and the `.` in it are placeholders.

### `siemens/motors_s120_electrical.csv`

| | |
| --- | --- |
| Source | The Siemens data sheet of each motor, generated by Siemens' data sheet service for the full article number in the column `data_sheet_article` |
| A row is | One motor of `motors_s120.csv` whose data sheet has been fetched. 166 of the 167 |
| Script | `fetch_siemens_datasheets.py` |

Columns: `article`, `data_sheet_article`, `poles`, `static_torque_60k_nm`, `stall_current_60k_a`, `max_speed_mechanical_rpm`, `max_speed_inverter_rpm`, `max_torque_nm`, `max_current_a`, `torque_constant_nm_a`, `voltage_constant_v_per_1000rpm`, `winding_resistance_20c_ohm`, `inductance_mh`, `electrical_time_constant_ms`, `mechanical_time_constant_ms`, `thermal_time_constant_min`, `shaft_torsional_stiffness_nm_rad`.

The torque constant is at a winding temperature rise of 100 K. The voltage constant is at 20 °C, line to line, RMS, per 1000 r/min. The resistance is of one phase at 20 °C. The inductance is the rotating field inductance. The options in the full article number (encoder, shaft, brake) do not change these values.

This is the only file with winding resistance and inductance. It covers the 1FK7 and 1FT7, which no config uses yet.

### `siemens/motor_modules_s120.csv`

| | |
| --- | --- |
| Source | Siemens Catalog News D 21.4 N, SINAMICS S120 and SIMOTICS, October 2017, chapter 7, "Technical specifications" |
| A row is | One S120 Motor Module in booksize format with internal air cooling, for a DC link of 510 to 720 V. 19 rows |
| Script | `extract_siemens_d21_4n.py` |

Columns: `article`, `axes`, `type`, `rated_current_a`, `base_load_current_a`, `max_current_a`, `rated_power_kw`, `pulse_frequency_khz`, `dc_link_capacitance_uf`, `power_loss_typical_kw`, `power_loss_max_kw`.

`axes` is 1 for a Single and 2 for a Double Motor Module; currents and rated power of a Double are per axis, its DC link capacitance and power loss are for the module. `type` is C (continuous load) or D (discontinuous load, higher maximum current).

### `siemens/line_modules_s120.csv` and `siemens/braking_s120.csv`

| | |
| --- | --- |
| Source | Siemens Catalog NC 82, SINUMERIK MC, SINAMICS S120 and SIMOTICS, February 2020, chapter 4: Line Modules; DC link components, Braking Module and braking resistors |
| A row is | Line modules: one S120 Line Module in booksize format with internal air cooling, 380 to 480 V 3 AC. 13 rows: 5 Smart, 5 Active, 3 Basic. Braking: the Braking Module and 4 braking resistors |
| Script | `extract_siemens_nc82.py` |

Line module columns: `article`, `type`, `rated_power_kw`, `s6_power_kw`, `max_power_kw`, `dc_link_current_540v_a`, `dc_link_current_600v_a`, `s6_dc_link_current_a`, `max_dc_link_current_a`, `dc_link_capacitance_uf`, `power_loss_kw`, `braking_power_max_kw`, `braking_power_continuous_kw`, `braking_threshold_v`, `min_braking_resistance_ohm`. The `braking_*` columns are the chopper inside the 20 kW and 40 kW Basic Line Modules; empty for all others.

Braking columns: `article`, `kind` (`braking_module` or `braking_resistor`), `for_article`, `resistance_ohm`, `rated_power_kw`, `peak_power_kw`, `activation_threshold_v`, `dc_link_capacitance_uf`, `load_duration_s`, `cycle_duration_s`, `load_duration_2_s`, `cycle_duration_2_s`.

These two files are where the DC link estimates of `exct-30.toml` come from: 540 V on a 400 V line, about 44 µF per kW of rated power, a chopper threshold of 770 V. The S210 converter of that config is not in them; see [what is missing](#what-is-missing-and-why).

### `bahr/belt_axes.csv`

| | |
| --- | --- |
| Source | Bahr Modultechnik GmbH (IMI Norgren), data sheets "Linearsystem \<series\>", German editions, from cdn.norgren.com. Editions 11/25, 04/26 and 05/26; each row carries its sheet number, edition and source file |
| A row is | One series and size of a linear axis driven by an HTD toothed belt. 69 rows from 15 series: 59 single axes, 7 XY gantries (ELZU), 3 X/Z gantries (ELZI, a T-gantry) |
| Script | `extract_bahr.py` |

Columns: `series`, `size`, `kind` (`axis`, `xy_gantry`, `xz_gantry`), `belt`, `belt_pitch_mm`, `belt_width_mm`, `feed_per_rev_mm`, `pulley_teeth`, `no_load_torque_nm`, `no_load_torque_times`, `max_speed_m_s`, `feed_force_continuous_n`, `feed_force_peak_n`, `fx_static_n` … `mz_dynamic_nm` (permissible loads on the slide, static and dynamic), `base_weight_kg`, `weight_per_100_mm_kg`, `weight_per_100_mm_z_kg`, `moved_mass_x_kg`, `moved_mass_y_kg`, `moved_mass_z_kg`, `max_length_one_piece_mm`, `max_length_x_mm`, `max_length_y_mm`, `max_length_z_mm`, `sheet`, `edition`, `source_file`.

- `feed_force_*`: belt pull, continuous and for 0.2 s.
- The sheets want the sum of Fy/Fy_dyn + Fz/Fz_dyn + Mx/Mx_dyn + My/My_dyn + Mz/Mz_dyn to stay at or below 1.
- `no_load_torque_times` is 2 where the sheet prints "2 x 0,4" (the X/Z gantry); whether that is per motor is not said.
- `max_speed_m_s`: the sheets give no conditions.

No config uses a Bahr axis yet. These sheets publish what Festo does not for the EXCT: permissible loads and moments on the slide and the belt's pull.

## How the files were extracted and checked

**Extraction.** Each Siemens and Bahr file is written by a script in [`catalog/tools/`](../catalog/tools). The vendor PDF is turned into text with `pdftotext -table` (xpdf 4.x), and the script reads the tables from that text. The scripts are strict: a row that does not have one value per column, a motor printed twice with different values, or a motor with only one of its two rows stops the script, so a changed edition cannot produce shifted values silently. The scripts also write the comment header of each file. The vendor documents are not part of the repository.

**Checks.** Tests read every file and check the numbers against their own physics and against each other. A value that landed in the wrong field or row breaks one of them. They are in [`sim-machines/tests/catalog.rs`](../crates/sim-machines/tests/catalog.rs), [`catalog_1fk2.rs`](../crates/sim-machines/tests/catalog_1fk2.rs) and [`catalog_bahr.rs`](../crates/sim-machines/tests/catalog_bahr.rs):

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
| `bahr/belt_axes.csv` | Rows unique; belt code agrees with pitch and width; feed per revolution = teeth × pitch; continuous feed force against the static Fx; the gantries state their moved masses and lengths | the four tests in `catalog_bahr.rs` |

That the simulator reads the files correctly is checked separately: `a_motor_by_name_gives_the_values_of_the_hand_written_config`, `a_gearbox_by_name_sets_ratio_stiffness_backlash_and_ratings`, `parts_named_in_a_config_behave_like_their_numbers` and `exct_30_config_agrees_with_the_catalogue`.

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
| Gearbox mass, input speed limits, radial and axial forces | Not extracted from the gearbox manual | The search ranks by motor weight only; `ratings.gearbox_max_input_speed_rpm` and `ratings.shaft_max_radial_load_n` are placeholders in the configs |
| Backlash of 24 NRB rows, inertia of 3 NRK rows | The manual leaves them open | A gearbox by name keeps the config's value there |
| The angled gearboxes (NRBW, NRKW, NLCW) and the precision gearboxes SP+ and TP+ | Not extracted | |
| EXCT: friction torque over speed, maximum acceleration over load and Z position, permissible support span, dimensions | The Festo catalogue gives them only as graphs or drawings | Friction of the EXCT-30 is an estimate |
| EXCT: allowable belt tension, guide and pulley shaft ratings | Festo does not publish them | All three are estimates in the EXCT-30 config. The belt tension: 2140 N. Festo publishes no allowable belt tension for the EXCT. It states the 5 mm pitch, the steel cords, the 30 mm width and the stretch of the belt: 0.045 % at 300 N, which is 667 kN per unit strain. Megadyne's Megalinear handbook lists for 5 mm pitch steel-cord polyurethane belts, at 4 mm/m stretch: T5 840 N at 25 mm and 1060 N at 32 mm (about 250 kN per unit strain at 30 mm), AT5 1805 N at 25 mm and 2275 N at 32 mm (about 535 kN). Festo's belt is as stiff as the AT5 class and far stiffer than T5, so the limit is the AT5 figure between the two widths: 2140 N at 30 mm. Read from the handbook itself. |
| S120 motors: maximum torque, current and speed in D 21.4 | Given as characteristic curves | Taken from the data sheets instead |
| Bahr: many series | Their tables need special cases: no-load torque with and without cover band, no feed force rows, a second belt width, differently laid out tables. The header of the file lists them | 15 series are in the file |
| Bahr: second moments of area, dimensions, order codes, the second drive version of ELZ and ELVZ | Not extracted | |
| Prices | In no file | The search uses motor weight as a stand-in |
| Belts, guides and drives as parts by name | The files exist for some of them; the simulator does not resolve them yet | Only motors and gearboxes can be named in `[parts]` |

## Rerun an extraction on a newer edition

The vendor documents are not in the repository. Get the PDF from the vendor, turn it into text, and run the script. All commands are run from `catalog/tools/`. They need Python 3 and `pdftotext` from xpdf 4.x.

| File | Commands |
| --- | --- |
| `motors_s210.csv`, `converters_s210.csv` | `pdftotext -table -enc UTF-8 catalogue.pdf catalogue.txt` then `python extract_siemens_d32.py catalogue.txt ../siemens` |
| `motors_1fk2_s120.csv`, `dynamic_braking_1fk2.csv` | `pdftotext -table -enc UTF-8 manual.pdf manual.txt` then `python extract_siemens_1fk2_s120.py manual.txt ../siemens` |
| `motors_1ft2_s120.csv` | the same with `extract_siemens_1ft2_s120.py` |
| `gearbox_nrb.csv`, `_nrk.csv`, `_nlc.csv` | the same with `extract_siemens_gearboxes.py` |
| `motors_s120.csv` | the same with `extract_siemens_d21_4.py` |
| `motor_modules_s120.csv` | the same with `extract_siemens_d21_4n.py` |
| `line_modules_s120.csv`, `braking_s120.csv` | the same with `extract_siemens_nc82.py` |
| `bahr/belt_axes.csv` | Download one PDF per name in the script's `SHEETS` list from `https://cdn.norgren.com/pdf/<name>.pdf`. `pdftotext -q -table -enc UTF-8 <name>.pdf` once per sheet, then `python extract_bahr.py folder_with_the_txt_files ../bahr` |
| `motors_s120_electrical.csv` | `python fetch_siemens_datasheets.py ../siemens <folder for the downloaded PDFs>`. It downloads one data sheet per motor of `motors_s120.csv` from Siemens' service. A full run takes hours; downloaded PDFs are kept and not fetched twice. With a third argument `cached` it fetches nothing and writes the file from the PDFs already in the folder |
| `festo/exct.toml` | By hand, from the catalogue's tables |

Then:

1. **If the script stops,** the new edition has a row or table it cannot read. It says which. Fix the script; do not patch the output.
2. **Review the diff** of the data file. The header of each file, with the document's name and edition, is written by the script from its `SOURCE` line, so update the edition there.
3. **Run the catalogue tests:**

   ```sh
   cargo test -p sim-machines --test catalog --test catalog_1fk2 --test catalog_bahr
   ```

   A misprint that the new edition has fixed will show as a test that names the old one (for example the test that expects the inertia of the 1FK2308-3AB to be empty says "is readable now, check it"). Remove the exception from the test and the note from the script's header.
4. **Rebuild.** The motor and gearbox files are compiled in. Then run `cargo test -p machine-t-gantry` and the catalogue tests again: `exct_30_config_agrees_with_the_catalogue` holds the EXCT-30 config to the files, and the search tests depend on which motors exist.
