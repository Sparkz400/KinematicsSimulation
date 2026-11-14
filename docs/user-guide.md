# User guide

One section per task. Each gives the command, what the output means, and the limits that matter for that task. The physics behind it is in the [model reference](model-reference.md); the config keys are in the [config reference](config-reference.md).
Deutsche Fassung: [Benutzerhandbuch](de/benutzerhandbuch.md).

- [Common to all commands](#common-to-all-commands)
- [Size a motor for a cycle](#size-a-motor-for-a-cycle)
- [Check a real catalogue machine](#check-a-real-catalogue-machine)
- [Find the lightest motor](#find-the-lightest-motor)
- [See which unknown parameters matter, and judge with error bands](#see-which-unknown-parameters-matter-and-judge-with-error-bands)
- [Shorten a cycle](#shorten-a-cycle)
- [Size a delta picker](#size-a-delta-picker)
- [Size a linear delta](#size-a-linear-delta)
- [Run a path and read path error](#run-a-path-and-read-path-error)
- [Tune gains and use input shaping](#tune-gains-and-use-input-shaping)
- [Replay a drive trace and fit parameters](#replay-a-drive-trace-and-fit-parameters)
- [See where torque and energy go](#see-where-torque-and-energy-go)
- [Record raw signals, switch the drives off](#record-raw-signals-switch-the-drives-off)
- [Connect a PLC over OPC UA](#connect-a-plc-over-opc-ua)
- [Use the model in Simulink / Modelica via FMI](#use-the-model-in-simulink--modelica-via-fmi)
- [Use the app](#use-the-app)

The examples write `sim-cli` for `cargo run -q --release -p sim-cli --`. All output shown was produced by the commands shown.

## Common to all commands

```text
sim-cli COMMAND MACHINE [options]
```

`MACHINE` is `t-gantry` or `single-axis` (`sim-cli list` prints them). Options that every command but `list` takes:

| Option | Effect |
| --- | --- |
| `--machine-file FILE` | In place of `MACHINE`: a machine defined in a TOML or JSON file, wired together from components without Rust code (an FMU of it carries the file). See [machine files](machine-files.md) |
| `--config FILE` | Use this config instead of the machine's default. The EXCT-30 config is `crates/machine-t-gantry/exct-30.toml` |
| `--motor ARTICLE` | Put a motor of the Siemens S210 catalogue into the config, e.g. `1FK2205-4AF`. Sets `parts.motor` |
| `--supply 230v` or `400v` | The supply the motor data is for. Sets `parts.motor_supply`. Default as in the config |
| `--gearbox SIZE-FRAME-RATIO` | Put a catalogue gearbox into the config, e.g. `NRK070-04-5`. Sets `parts.gearbox` |
| `--set path=value` | Change one config value, e.g. `--set mass.payload_kg=3.0`. Repeatable. Applied after the parts, so single values can be set on top of a catalogue part |
| `--out FILE` | Write the CSV to a file instead of standard output |
| `--lang en` or `de` | Language of messages. The CSV headers are not translated |
| `--catalog DIR` | Read the component catalogue from DIR, a copy of [`catalog/`](../catalog) of the same schema version, instead of the one built in (see [catalogue](catalogue.md)). `sim-app` takes it too |

Rules that are easy to trip over:

- `--set` only changes values that exist. A misspelt key is refused (`Configuration value is missing: mass.cariage_kg`), and so is a value of the wrong type.
- `--motor`, `--supply` and `--gearbox` need a `[parts]` table in the config. All three shipped configs have one.
- Every option except `--cycle`, `--time-optimal`, `--bands` and `--worst` takes a value. An option without its value is refused (`This option needs a value: --time`), and so is an option the command does not know, such as a misspelt one (`This command has no such option: --cylce`). A number has to be finite: `--time inf` is refused.
- `--cycle`, `--move`, `--trajectory` and `--path` exclude each other (`Only one of these options can be given at a time: --cycle --move`).
- A config file may only have keys that the machine's default config has: a misspelt key is refused (`Configuration has a value this machine does not know: mass.cariage_kg`). The exceptions are `[bounds]`, which has its own check, and `ratings.pulley_max_torque_nm`.
- Numbers are checked when a config is read. NaN and infinity are refused for every value, a negative number where the quantity cannot be negative (a payload, a damping, a backlash, the belt pretension), and a start pose outside the axis ranges. The message names the key: `Configuration value is out of range: gearbox.backlash_rad`.
- A failure prints one line on the error stream and exits with code 2.
- If the axis limits ask more of a motor than the drive data allows, commands that start a simulation directly (`run`) print a warning on the error stream and carry on:

  ```text
  $ sim-cli run t-gantry --set axis.x.max_velocity_m_s=2.0 --time 0.01 --out x.csv
  The axis limits ask for more motor speed than the drive allows: Motor A: 600.0000 > 314.0000
  The axis limits ask for more motor speed than the drive allows: Motor B: 600.0000 > 314.0000
  ```

  The check compares the motor speed at the highest axis speeds with `drive.rated_speed_rad_s`, and the rigid-model torque at the highest axis accelerations with `drive.peak_torque_nm`, for every combination of axis directions.

## Size a motor for a cycle

The cycle is the list of `[[cycle.segment]]` in the config: target pose, dwell time, and whether the payload is attached on arrival. Edit a copy of the config to change it; `--set` cannot edit the cycle.

```text
$ sim-cli analyze t-gantry
Thermal start: warm, at the rises the cycle settles at; largest change of a rise over the measured pass 0.000 K
motor,cycle_s,peak_torque_nm,rms_torque_nm,peak_speed_rad_s,utilisation,thermal_utilisation,belt_tension_min_n,belt_tension_max_n,rms_current_a,winding_rise_k,energy_in_j,energy_regenerated_j,peak_regenerated_power_w,largest_regeneration_j,path_error_peak_m,path_error_rms_m,tracking_error_peak_m,dwell_error_peak_m,cubic_mean_torque_nm,braking_resistor_energy_j,braking_resistor_mean_power_w,dc_link_peak_voltage_v,belt_slack_time_s,radial_load_peak_n,guide_load_peak_n,guide_life_km,mains_energy_j,guide_life_h,pulley_bearing_life_h,gearbox_life_h,belt_tooth_utilisation,gearbox_gearing_life_h,motor_bearing_life_h
a,6.080000,0.559501,0.165359,200.4517,0.1475,0.1302,204.63,334.36,0.3676,1.23,85.040,10.094,28.20,5.864,1.369e-4,4.382e-5,1.706e-4,4.734e-5,0.196870,0.000,0.000,221.40,0.0000e0,6.4481e2,1.7783e2,4.7379e6,0.000,4.9950e6,4.7222e5,6.7360e9,1.9094e-1,,
b,6.080000,0.460104,0.153384,200.4375,0.1211,0.1208,204.63,334.36,0.3410,1.05,74.377,15.292,35.02,7.081,1.369e-4,4.382e-5,1.706e-4,4.734e-5,0.175175,5.343,0.879,224.65,0.0000e0,6.4481e2,1.7783e2,4.7379e6,0.000,4.9950e6,4.7127e5,1.0201e10,1.5155e-1,,
```

The simulator starts the cycle, lets the first pass go by, and measures the second. One row per motor. Columns that describe the machine, not one motor, repeat in every row.

| Column | Unit | Meaning |
| --- | --- | --- |
| `cycle_s` | s | Duration of the measured pass |
| `peak_torque_nm` | N m | Highest absolute motor torque (from the winding current, at the motor shaft) |
| `rms_torque_nm` | N m | RMS of the motor torque over the pass |
| `peak_speed_rad_s` | rad/s | Highest absolute motor speed |
| `utilisation` | fraction | Highest ratio of the torque the velocity loop asks for (before the torque limit) to the torque the torque-speed curve leaves at that speed. Above 1 the cycle asks for more than the curve gives: the drive sits on its torque limit and the axis falls behind |
| `thermal_utilisation` | fraction | RMS torque over `drive.rated_torque_nm`. For information; the verdict `motor-thermal` judges the winding rise, which rests on `motor.rated_current_a` |
| `belt_tension_min_n`, `_max_n` | N | Lowest and highest tension of any belt section, looked at once per millisecond. Machine-wide |
| `rms_current_a` | A | RMS winding current |
| `winding_rise_k` | K | Temperature rise the winding settles at if this cycle repeats for long |
| `energy_in_j` | J | Electrical energy the winding drew while the power was positive |
| `energy_regenerated_j` | J | Energy fed back while braking |
| `peak_regenerated_power_w` | W | Highest power fed back |
| `largest_regeneration_j` | J | Most energy fed back in one braking (one unbroken stretch of negative power) |
| `path_error_peak_m`, `_rms_m` | m | How far the tool was from the path its setpoint drew. Machine-wide |
| `tracking_error_peak_m` | m | How far the tool was from where its setpoint was at the same moment. Machine-wide |
| `dwell_error_peak_m` | m | Most the tool was off its target in the second half of a dwell. Machine-wide |
| `cubic_mean_torque_nm` | N m | Cube root of the mean of torque cubed, weighted with speed, of the motor torque. The check `gearbox-continuous` takes the same mean of the gearbox output torque |
| `braking_resistor_energy_j` | J | Energy the simulated braking resistor burned. 0 without `effects.dc_link` |
| `braking_resistor_mean_power_w` | W | That energy over the cycle time |
| `dc_link_peak_voltage_v` | V | Highest DC link voltage. Stays at `drive.bus_voltage_v` without `effects.dc_link` |
| `belt_slack_time_s` | s | Time any belt section spent without tension. Machine-wide |
| `radial_load_peak_n` | N | Highest pull of the belt on a pulley shaft. Machine-wide |
| `guide_load_peak_n` | N | Highest load on the guide with the highest equivalent load. Machine-wide |
| `guide_life_km` | km | L10 = (C / P)³ · 50 km for that guide. Empty without `ratings.guide_dynamic_load_rating_n` |
| `mains_energy_j` | J | With `effects.regenerative_supply`: energy the line module fed back into the mains over the pass (after its efficiency). 0 without |
| `guide_life_h` | h | `guide_life_km` over the mean speed of that guide's axis. Machine-wide |
| `pulley_bearing_life_h` | h | Rating life of the bearing that takes the belt's pull on this motor's pulley shaft: (C / P)³ · 10⁶ / (60 n), C = `ratings.pulley_bearing_load_rating_n`, P the cubic mean of the radial load weighted with the speed, n the pulley's mean speed in r/min; times ISO 281's a_ISO for lubrication and contamination where the config has the four values it needs |
| `gearbox_life_h` | h | Life of the gearbox's planet bearings by Siemens' formula, 20000 h · (n2B / n2) · (M2N / M2eq)^(10/3). Empty without `ratings.gearbox_rated_torque_nm` and `gearbox_reference_speed_rpm` (the EXCT-30 has no gearbox) |
| `belt_tooth_utilisation` | fraction | The belt's tooth load on this motor's pulley over what its teeth allow at the pulley's speed (Megadyne's method, with `ratings.belt_service_factor`), the highest over the cycle. Not a life: no belt maker publishes one |
| `gearbox_gearing_life_h` | h | Life of the gearbox's gearing from the torque above its rated torque (Palmgren–Miner on Siemens' two torque figures). Empty where the cycle never takes the gearbox above `ratings.gearbox_rated_torque_nm` |
| `motor_bearing_life_h` | h | Life of the motor's bearings under the belt's pull. Only where the pulley sits on the motor shaft (`ratings.motor_bearing_load_rating_n` set, no gearbox); empty in every shipped config |

The exact definitions are in the [model reference](model-reference.md#analysis-quantities).

`--points FILE` also writes the operating points, one per millisecond and motor (`motor,speed_rad_s,torque_nm`), for plotting over the torque-speed curve.

Then judge the cycle:

```text
$ sim-cli verdicts t-gantry
Thermal start: warm, at the rises the cycle settles at; largest change of a rise over the measured pass 0.000 K
check,motor,value,limit,result
motor-peak,a,0.1475,1.000,ok
motor-thermal,a,1.227,100.0,ok
drive-current,a,0.3676,3.000,ok
drive-peak-current,a,1.248,10.90,ok
gearbox-continuous,a,0.7955,40.00,ok
gearbox-input-speed,a,1914,13000,ok
gearbox-peak,a,2.175,64.00,ok
pulley-bearing-life,a,472218,20000,ok
gearbox-life,a,6736010395,20000,ok
belt-tooth-load,a,0.1909,1.000,ok
dc-link-energy,a,5.864,6.205,ok
dc-link-overvoltage,a,221.4,257.6,ok
...
settling,,0.00004734,0.0001000,ok
belt-tension,,334.4,1500,ok
belt-slack,,204.6,0,ok
gearbox-radial-load,,644.8,1500,ok
guide-load,,177.8,2000,ok
guide-life,,4994979,20000,ok
thermal-warm-start,,0.00001281,0.1000,ok
```

| Check | Value | Limit | Unit |
| --- | --- | --- | --- |
| `motor-peak` | `utilisation` (the torque asked for, not the torque delivered) | 1 | fraction |
| `motor-thermal` | `winding_rise_k` | `motor.rated_temperature_rise_k` | K |
| `pulley-torque` | peak of the torque at the gearbox output (signal `gearbox.<motor id>.output_torque`) | `ratings.pulley_max_torque_nm` | N m |
| `gearbox-continuous` | cubic mean of the torque at the gearbox output, weighted with speed | `ratings.gearbox_rated_torque_nm` | N m |
| `gearbox-input-speed` | peak motor speed | `ratings.gearbox_max_input_speed_rpm` | r/min |
| `gearbox-peak` | peak of the torque at the gearbox output, looked at every simulation step | `ratings.gearbox_max_torque_nm` | N m |
| `dc-link-energy` | energy into the DC link capacitors | room between the DC link voltage (`drive.bus_voltage_v`) and the braking threshold | J |
| `braking-resistor-continuous` | mean power of the resistor | `ratings.braking_resistor_continuous_w` | W |
| `braking-resistor-peak` | peak power of the resistor | `ratings.braking_resistor_peak_w` | W |
| `dc-link-overvoltage` | highest DC link voltage (only with `effects.dc_link`) | `ratings.dc_link_overvoltage_v` | V |
| `settling` | `dwell_error_peak_m` | `ratings.settling_tolerance_m` | m |
| `belt-tension` | highest belt tension | `ratings.belt_max_tension_n` | N |
| `belt-slack` | lowest belt tension | `ratings.belt_min_tension_n` | N |
| `gearbox-radial-load` | `radial_load_peak_n` | `ratings.shaft_max_radial_load_n` | N |
| `guide-load` | `guide_load_peak_n` | `ratings.guide_max_load_n` | N |
| `pulley-bearing-life` | `pulley_bearing_life_h` | `ratings.required_life_h` | h |
| `gearbox-life` | `gearbox_life_h` | `ratings.required_life_h` | h |
| `gearbox-gearing-life` | `gearbox_gearing_life_h` (only where the cycle exceeds the rated torque) | `ratings.required_life_h` | h |
| `motor-bearing-life` | `motor_bearing_life_h` | `ratings.required_life_h` | h |
| `guide-life` | `guide_life_h` | `ratings.required_life_h` | h |
| `belt-tooth-load` | `belt_tooth_utilisation` | 1 | fraction |
| `gearbox-thermal` | where the hottest gearbox settles on the cycle (only with `effects.part_heating`; none on a direct drive) | `thermal.gearbox_max_rise_k` | K |
| `guide-thermal` | where the hottest guide settles on the cycle (only with `effects.part_heating`) | `thermal.guide_max_rise_k` | K |
| `belt-tooth-jump` | how often the belt's teeth jumped over a driven pulley in the cycle (only with `effects.belt_tooth_jump`) | 0 | count |
| `belt-span-resonance` | how close a free belt span's first sideways mode came to what the drive excites, at steady speed, as a share of the mode (only with `effects.belt_span_vibration`); passes at or above the limit | `ratings.belt_span_resonance_band` | fraction |
| `thermal-warm-start` | how far the rises the measured pass started at lie from those it settles at (only with `effects.thermal_warm_start`, where something that warms acts back on the run) | 0.1 | K |

Limits for this task:

- Which thermal start was used is on the error stream (`Thermal start: cold` or `warm`) and in the row `thermal-warm-start`. With `effects.thermal_warm_start` (on in the shipped configs) the measured pass runs with the windings and, with `effects.thermal_feedback`, the gearboxes and guides at the rises they settle at when the cycle repeats for long, found by repeating the cycle (at most 4 warm passes, to 0.1 K); off, it starts cold and a cycle of seconds sees every part at ambient. With the feedback on, the EXCT-30's X guide settles at 15.9 K started warm, 23.7 K judged from cold (its oil thins as it warms). See the [model reference](model-reference.md#thermal-warm-start).
- A rating that the config leaves out is not judged. No row is not a pass. The same holds for `gearbox-thermal` and `guide-thermal` with `effects.part_heating` and `effects.thermal_feedback` off, for `belt-tooth-jump` without `effects.belt_tooth_jump` and for `belt-span-resonance` without `effects.belt_span_vibration`. With it on, the EXCT-30's X guide settles at 25 K of 40 K and fails the worst case of `--bands` (51 K at the high bound of its thermal resistance); the thermal resistances are estimates (see the config reference).
- `belt-slack` and the lives are the checks where the value has to stay above the limit. `belt-slack` also fails if any section spent any time without tension.
- The lives are the makers' formulas with ratings that are partly estimates; see the [model reference](model-reference.md#analysis-quantities) for what they leave out. The motors' own bearings have no figure in these configs: the belt does not pull on a motor shaft.
- `dc-link-energy` is `ok` whenever the config has a braking resistor to fall back on, even when the value is at or above the limit. The two resistor rows appear only when the resistor is needed. All three default configs simulate the DC link (`effects.dc_link = true`); with it off the braking rows are estimates from what the motor feeds back.
- A braking threshold at or below `drive.bus_voltage_v` is refused (`Configuration value is out of range: ratings.braking_threshold_v`), simulated or not: it is a rating for another DC link voltage.
- Values are printed with four significant digits and no exponent, so `settling` shows as `0.00004588`.
- `motor-peak` judges what the velocity loop asks for before the torque limit. A cycle that sits on the torque limit fails it; the delivered torque, which never exceeds the curve, is `peak_torque_nm` of `analyze`.
- With input shaping, a dwell and the gripper wait until the shaped setpoint has arrived, one period of the shaper after the plan. `settling` is then judged from that moment.
- Gearbox and pulley torques are the torque at the gearbox output, the signal `gearbox.<motor id>.output_torque`. In the elastic model that is what the gear's spring passes on to the pulley, with the efficiency applied; in the rigid model it is the value the solver works out. The torque that accelerates the rotor itself does not count against the gearbox, and a load that drives the gearbox back (braking, lowering) counts with what it puts on the output. `cubic_mean_torque_nm` of `analyze` is still the motor's.
- The motor, friction and drive values of the default configs are placeholders. See the README on what the absolute numbers rest on.

**One DC link for all drives.** `effects.shared_dc_link` (off by default, needs `effects.dc_link`) gives the drives of a machine one DC link, as with one Line Module on a SINAMICS S120. What a braking motor feeds back, a motoring one draws; only the net power charges the capacitors. The link has the capacitance of all drives together (each brings `ratings.dc_link_capacitance_uf`), one chopper with one braking resistor of the `[ratings]` powers, and one trip. Every motor then shows the same DC link voltage and resistor power, and the braking checks appear once per motor with the same shared values. On a machine with one drive the switch changes nothing.

```text
$ sim-cli verdicts t-gantry --set effects.shared_dc_link=true
...
dc-link-energy,a,6.165,6.205,ok
dc-link-overvoltage,a,224.3,257.6,ok
...
dc-link-energy,b,6.165,6.205,ok
dc-link-overvoltage,b,224.3,257.6,ok
...
```

On the default cycle the highest DC link voltage is 224.65 V with separate links (motor B, at the chopper's threshold) and 224.27 V with the shared one. The resistor is not needed then, so its two rows are gone.

**Feeding back into the mains.** `effects.regenerative_supply` (off by default, needs `effects.dc_link`) puts a regenerative line module in place of the plain rectifier, as an Active Line Module on a SINAMICS S120: it holds the DC link at the bus level from above too and feeds what the motors give back into the mains, up to `ratings.line_module_max_power_w`, of which `ratings.line_module_efficiency` arrives there. The braking resistor then takes only what is beyond the line module's power. `analyze` reports the energy fed back per motor (last column, `mains_energy_j`), the energy budget gets the rows `mains` and `line_module` (its loss), the app shows both, and two checks judge the line module: `line-module-continuous` (its mean power, drawn and fed back, against `line_module_rated_power_w`) and `line-module-peak` (its highest power against `line_module_max_power_w`). The shipped values are the smallest S120 Active Line Module, 16 kW rated and 35 kW at most, far above what these machines need. On the default T-gantry 24.9 J of the 25.4 J the motors regenerate per cycle go back into the mains; the resistor burns nothing and the DC link stays at 155.13 V.

## Check a real catalogue machine

A config can describe a real machine with its values taken from the catalogue. [`exct-30.toml`](../crates/machine-t-gantry/exct-30.toml) does that for a Festo EXCT-30 with two Siemens 1FK2205-4AF motors on S210 converters at 400 V. Every value in it is marked `[catalogue]` or `[estimate]`.

```text
$ sim-cli analyze t-gantry --config crates/machine-t-gantry/exct-30.toml
Thermal start: warm, at the rises the cycle settles at; largest change of a rise over the measured pass 0.000 K
motor,cycle_s,peak_torque_nm,rms_torque_nm,peak_speed_rad_s,utilisation,thermal_utilisation,belt_tension_min_n,belt_tension_max_n,rms_current_a,winding_rise_k,...
a,2.048000,3.659599,1.459732,156.9836,0.2930,0.3173,257.77,554.98,1.1446,6.88,...
b,2.048000,3.036503,1.391427,156.9737,0.2431,0.3025,257.77,554.98,1.0908,6.24,...
```

Five values in this config are choices of the designer, made so that the checks pass in the worst case of the error bands ([below](#see-which-unknown-parameters-matter-and-judge-with-error-bands)): a belt pretension of 450 N, an X jerk limit of 150 m/s³, input shaping at a fixed 25 Hz with a second shaper at 17.5 Hz, and corner blending. The comments in the file give the reasons.

Like the default configs, this one has `effects.dc_link = true`, so the braking rows of `verdicts` are simulated, not estimated: the DC link voltage rises to the chopper threshold of 770 V and the resistor burns the rest.

```text
dc-link-energy,a,9.943,9.943,ok
braking-resistor-continuous,a,8.284,100.0,ok
braking-resistor-peak,a,200.5,5700,ok
dc-link-overvoltage,a,770.0,820.0,ok
```

**Parts by name.** `[parts]` in a config, or `--motor`, `--supply`, `--gearbox`, `--drive`, `--belt` and `--axis` on the command line, name a motor, a gearbox, a drive, a belt and an axis from the catalogue. Their catalogue values then replace the config's numbers. They go in in the order axis, belt, gearbox, motor, drive, so a belt named beside an axis replaces the axis's belt:

| Part | What it sets |
| --- | --- |
| Motor | `motor.rotor_inertia_kgm2`, `motor.torque_constant_nm_a` (static torque over stall current), `motor.max_current_a`, `motor.torque_constant_at_max_current_nm_a`, `motor.rated_current_a` (the catalogue's own, at most 1.05 times the stall current), `motor.thermal_time_constant_s` (where the configuration manual has it), `drive.peak_torque_nm` (capped by `ratings.pulley_max_torque_nm` over the gear ratio), `drive.rated_torque_nm`, `drive.rated_speed_rad_s`, `drive.bus_voltage_v` (the DC link voltage: 540 V for `400v`, 325 V for `230v`), `drive.no_load_speed_rad_s` (sqrt(3/2) × DC link voltage over the torque constant), and `ratings.braking_threshold_v` and `ratings.dc_link_overvoltage_v` scaled with the DC link voltage |
| Motor on S120 (`--supply s120`) | The same keys from the S120 data: for a 1FK7 or 1FT7 [`motors_s120.csv`](../catalog/siemens/motors_s120.csv) and its data sheet (maximum torque and current, thermal time constant), for a 1FK2 or 1FT2 its S120 configuration manual. A 1FK7 or 1FT7 also brings its winding: `motor.resistance_ohm` and `motor.inductance_h` are three times the data sheet's phase values (the model's one winding carries the phase current at three times the phase voltage), and the current controller is retuned with them, `drive.current_kp_v_a` = L / T for the config's own current loop time constant T (its inductance over its gain) and `drive.current_tn_s` = L / R. The DC link is 540 V |
| Gearbox | `gearbox.ratio`, `gearbox.efficiency`, `gearbox.stiffness_nm_rad`, `gearbox.backlash_rad` and `gearbox.inertia_at_input_kgm2` (both where the manual gives them), `ratings.gearbox_rated_torque_nm` and `ratings.gearbox_max_torque_nm` (where the config has them) |
| Drive | An S210 converter ([`converters_s210.csv`](../catalog/siemens/converters_s210.csv)) or an S120 Motor Module ([`motor_modules_s120.csv`](../catalog/siemens/motor_modules_s120.csv)) by article number: `ratings.drive_rated_current_a` and `ratings.drive_max_current_a`, its rated and maximum output current (per axis for a Double Motor Module). `verdicts` judges the cycle's RMS motor current against the first (`drive-current`) and its peak against the second (`drive-peak-current`). With `effects.drive_current_limit` (on in the shipped configs) the drive's maximum current also limits the simulated current: a cycle that asks more lags behind its plan, and `drive-peak-current` judges the current asked for. The DC link: an S210 converter brings its internal braking resistor and trip level ([`braking_s210.csv`](../catalog/siemens/braking_s210.csv)); an S120 Motor Module its DC link capacitance (half per axis for a Double) and, as it has no resistor of its own, the Braking Module's of [`braking_s120.csv`](../catalog/siemens/braking_s120.csv): `braking_resistor_continuous_w` 1.5 kW, `_peak_w` 100 kW, `braking_threshold_v` 770 V, and `dc_link_overvoltage_v` 820 V, the booksize units' trip. One Braking Module serves the whole line-up, so with an S120 Motor Module (`drive.braking_module` on) `verdicts` judges the axes' braking power together, once for the machine: the mean of their sum against 1.5 kW and the peak of their summed power against 100 kW. An S210, with a resistor of its own, is judged per axis |
| Belt | As profile-width, e.g. `AT5-25`: a Megadyne MEGALINEAR belt with standard steel cords ([`megadyne/belts.csv`](../catalog/megadyne/belts.csv); T5, AT3, AT5, AT10 and the HTD-compatible MTD3, MTD5, MTD8). `belt.stiffness_ea_n` (the allowable pull over the stretch at it, 4 mm/m), `belt.mass_kg_m`, `belt.tooth_pitch_m`, and where the config has them `ratings.belt_max_tension_n` (the allowable pull, a quarter of the breaking strength), `ratings.belt_width_mm` and the tooth load table `ratings.belt_tooth_load_rpm` and `_n_cm`. A width between two standard widths is interpolated, one outside them taken in proportion to the nearest |
| Axis | By name: `EXCT-15`, `EXCT-30`, `EXCT-100` (Festo, [`festo/belt_axes.csv`](../catalog/festo/belt_axes.csv)) and Bahr's axes and X/Z gantries as series-size, e.g. `ELZI-40` or `ELZ-60S` ([`bahr/belt_axes.csv`](../catalog/bahr/belt_axes.csv)). An X/Z gantry goes on a config with a Z stroke, a single axis on one with one stroke; the other way round, and a stroke the axis does not have, is refused. Sets `geometry.pulley_radius_m`, the belt (as above, for the axis's profile and width: Festo's EXCT as AT, Bahr's HTD as MTD; Festo's EA from its stated stretch), `ratings.belt_teeth_in_mesh` (half the pulley's teeth), `ratings.pulley_max_torque_nm` where the config has it (Festo's maximum torque; Bahr's peak feed force times the radius), and the moved masses: Bahr's gantry sheets give them per axis (Z axis, and the carriage as X less Z); Festo gives the Z mass of its torque formula, and the carriage is fitted so that the machine needs Festo's torque at 45 degrees, as `exct-30.toml` was written. The strokes stay; an EXCT gets the shortest Z stroke Festo offers that holds the config's. Bahr's single axes state no moved mass, so the config's carriage stays |
| Any but the drive | `drive.velocity_kp_nm_s_rad`, retuned for the part (see below) |

What a part by name does **not** set, and what follows from that:

- **Winding resistance and inductance.** Siemens does not publish them for the 1FK2 and 1FT2. The config's values stay, and the current controller's gains with them. (For the 1FK7 and 1FT7 on S120 they are set, see above.)
- **The drive with the motor.** Naming a motor does not name its drive; the config's drive ratings stay. The search pairs each motor with its drive (below).
- **The DC link capacitance and the braking resistor's powers.** They stay as the config has them. The braking threshold and the trip level keep their ratio to the DC link voltage, which is sound for placeholders and no more: set all five DC link ratings for the real drive before trusting the braking rows.
- **Gearbox input speed, radial load and life.** The gearbox files do not have them. `ratings.gearbox_max_input_speed_rpm`, `ratings.shaft_max_radial_load_n`, `ratings.pulley_bearing_load_rating_n` and `ratings.gearbox_reference_speed_rpm` stay as the config has them.

**The velocity gain is retuned.** A motor with a much lighter rotor oscillates with a gain tuned for a heavier one, which is a failure of the tuning, not of the motor. So wherever a part is named (`[parts]`, `--motor`, `--gearbox`, the search), `drive.velocity_kp_nm_s_rad` is scaled by the square root of (ratio of the inertia the motors turn) × (ratio of the rotor inertias, the gearbox's own included). The reference is the config's own numbers as the file states them before the part goes in: the gain in a config is taken to be tuned for the rotor, gearbox and masses written in that config. `--set drive.velocity_kp_nm_s_rad=...` on the command line is applied after the parts and overrides the retuned gain.

```text
$ sim-cli verdicts t-gantry --config crates/machine-t-gantry/exct-30.toml --motor 1FK2204-6AF
motor-peak,a,0.3636,1.000,ok
motor-thermal,a,12.07,100.0,ok
...
settling,,0.00004418,0.0001000,ok
belt-slack,,252.0,0,ok
```

The rotor of the 1FK2204-6AF has a third of the inertia of the config's 1FK2205-4AF; the gain goes from 0.45 to 0.23 N m s/rad. (0.45 happens to be calm for this motor too.) The 1FK2104-5AF has a ninth of the inertia: with the gain left at 0.45 it oscillates and its winding never settles, retuned to 0.13 it runs the cycle with 1.22 N m RMS.

An unknown article is refused: `The catalogue has no such part: motor 1FK9`. A motor that the catalogue does not offer at the chosen supply is refused the same way, and so are unknown drives, belts and axes.

Naming the axis of `exct-30.toml` gives its own numbers back (the test `an_axis_by_name_gives_the_numbers_of_the_hand_written_exct_30`): the carriage fitted again is 7.136 kg against the file's 7.16 kg, the belt's mass from the AT5 table 97.9 g/m against the file's estimate of 0.1 kg/m.

## Find the lightest motor

```text
$ sim-cli search t-gantry --config crates/machine-t-gantry/exct-30.toml --series 1FK2
Ruled out by catalogue numbers against the axis limits, without simulating a cycle: 13 / 28 candidates
Search time on 16: 0.6 s
axis,motor,gearbox,drive,mass_kg,cost_kg,result,tightest_check,tightest_utilisation,peak_torque_nm,winding_rise_k
,1FK2104-5AF,,6SL3210-5HE10-8UF0,2.85,5.70,ok,settling,0.5312,3.2878,20.44
,1FK2104-5AK,,6SL3210-5HE11-5UF0,2.85,5.70,ok,settling,0.5325,3.2958,41.53
,1FK2204-5AF,,6SL3210-5HE10-8UF0,2.90,5.80,ok,drive-current,0.5142,3.4285,21.54
,1FK2104-6AF,,6SL3210-5HE11-0UF0,3.40,6.80,ok,gearbox-radial-load,0.5018,3.3439,11.27
,1FK2204-6AF,,6SL3210-5HE11-0UF0,3.50,7.00,ok,gearbox-radial-load,0.5018,3.4466,12.07
,1FK2205-2AF,,6SL3210-5HE11-0UF0,3.75,7.50,ok,gearbox-radial-load,0.5007,3.5157,14.67
,1FK2205-4AF,,6SL3210-5HE11-5UF0,5.20,10.40,ok,gearbox-radial-load,0.5002,3.6596,6.88
...
,1FK2204-5AK,,6SL3210-5HE11-5UF0,2.90,5.80,FAIL,motor-thermal,1.6782,3.4287,167.82
,1FK2203-2AK,,6SL3210-5HE10-4UF0,1.16,2.32,FAIL,limit-motor-torque,NaN,NaN,NaN
...
,1FK2208-3AC,,6SL3210-5HE13-5UF0,12.60,25.20,FAIL,limit-motor-speed,NaN,NaN,NaN
```

On the EXCT-30 no passing motor comes near a limit. The tightest check is the dwell for the lightest motors (0.53 of the tolerance), the drive current for the 1FK2204-5AF, and for the others the belt's pull on a pulley shaft (1000 N of 2000 N), which hardly depends on the motor: compare them by `winding_rise_k` and `peak_torque_nm`. The pulley shaft bearings last about 325000 h, against 20000 h required.

The search runs every motor of the catalogue that is offered at the supply on the config's cycle, each with the drive the catalogue pairs with it, on all processor cores. Passing motors come first, cheapest first. Failing ones follow, nearest to passing first. A candidate whose drives trip on DC link overvoltage comes after all of these, with `inf` as its `tightest_utilisation`: its figures end at the trip.

**Cost.** No catalogue file has a price, and Siemens publishes no list prices on a server that can be read automatically (its Industry Mall refuses such access). The installed weight stands in for the price: `cost_kg` is motor and gearbox times the number of motors, plus the axis where one of the catalogue is tried (`--axes`). Passing candidates are ranked by it, then by `mass_kg`. For a motor search on a fixed machine the two give the same order.

| Option | Effect |
| --- | --- |
| `--series PREFIX` | Only article numbers that start this way, e.g. `1FK2`, `1FT2105` or `1FK7` |
| `--supply 230v`, `400v` or `s120` | `230v` and `400v`: the S210 motors ([`motors_s210.csv`](../catalog/siemens/motors_s210.csv)) at that supply, each with the S210 converter the D 32 catalogue lists beside it (at 230 V the single-phase one where there is one). `s120`: the motors on a SINAMICS S120 at 400 V, the 1FK7 and 1FT7 of [`motors_s120.csv`](../catalog/siemens/motors_s120.csv) with a data sheet and the 1FK2 and 1FT2 of their S120 manuals, each with its Single Motor Module (see [parts by name](#check-a-real-catalogue-machine)). Default `400v` |
| `--gearboxes SERIES` | Also try each motor with every gearbox of that series (`NRB`, `NRK`, `NLC`, or a size such as `NRK070`) that fits its frame and has a ratio between half and twice the config's. Only on a machine whose config has a ratio above 1, and only for 1FK2 and 1FT2 motors (the gearboxes are made for them) |
| `--axes` | Also try every axis of the catalogue of the machine's kind (below) |

| Column | Meaning |
| --- | --- |
| `axis` | The axis of the catalogue the motor was tried on; empty for the config's own |
| `motor`, `gearbox` | The candidate. `gearbox` is empty where the config's own gearbox stayed |
| `drive` | The drive the catalogue pairs with the motor; the cycle is judged against its currents (`drive-current`, `drive-peak-current`) |
| `mass_kg` | Mass of the motor without brake and of its gearbox (Siemens' configuration manual 01/2023, see [the catalogue](catalogue.md)) |
| `cost_kg` | The stand-in for the price: installed weight, see above |
| `result` | `ok` if every verdict passes |
| `tightest_check` | The check closest to its limit, or furthest beyond it. Two DC link checks are left out while they pass, because their ratio says nothing about the margin: `dc-link-energy` when a braking resistor is in use (the capacitors then fill to their limit by design) and `dc-link-overvoltage` (the voltage is never below the DC link's own level). For a candidate that was not simulated: why (`limit-motor-speed`, `limit-motor-torque`, `axis-stroke`, `axis-payload`, `axis-speed`) |
| `tightest_utilisation` | Value over limit of that check |
| `peak_torque_nm` | Highest peak torque of any motor of the machine |
| `winding_rise_k` | Highest settled winding rise of any motor |

Before a cycle is simulated, the catalogue numbers are checked against the axis limits: motor speed at the highest axis speed against the rated speed, rigid-model torque at the highest acceleration against the motor's maximum torque. What fails there is listed with `NaN` and not simulated. The error stream says how many candidates that were and how long the search took on how many cores.

Each candidate's velocity gain is retuned by the rule of [parts by name](#check-a-real-catalogue-machine): the square root of (ratio of the inertia the motors turn) × (ratio of the rotor inertias), against the config's own numbers. `--gearbox NAME` for the whole search and `--gearboxes SERIES` give the same result for the same pair: the gearbox's inertia has a key of its own (`gearbox.inertia_at_input_kgm2`) and stays when a motor goes in.

With gearboxes, on the single axis:

```text
$ sim-cli search single-axis --series 1FK2104 --gearboxes NRK070
Ruled out by catalogue numbers against the axis limits, without simulating a cycle: 12 / 35 candidates
Search time on 16: 0.5 s
axis,motor,gearbox,drive,mass_kg,cost_kg,result,tightest_check,tightest_utilisation,peak_torque_nm,winding_rise_k
,1FK2104-4AF,NRK070-04-3,6SL3210-5HE10-4UF0,3.45,3.45,ok,gearbox-radial-load,0.4188,0.7667,3.85
,1FK2104-4AF,NRK070-04-4,6SL3210-5HE10-4UF0,3.45,3.45,ok,settling,0.4567,0.6007,2.50
,1FK2104-4AF,NRK070-04-5,6SL3210-5HE10-4UF0,3.45,3.45,ok,settling,0.4637,0.5294,1.91
...
```

The S120 motors on the EXCT-30:

```text
$ sim-cli search t-gantry --config crates/machine-t-gantry/exct-30.toml --supply s120 --series 1FK7
Ruled out by catalogue numbers against the axis limits, without simulating a cycle: 32 / 61 candidates
Search time on 16: 1.1 s
axis,motor,gearbox,drive,mass_kg,cost_kg,result,tightest_check,tightest_utilisation,peak_torque_nm,winding_rise_k
,1FK7042-2AF7,,6SL3120-1TE13-0AD0,4.60,9.20,ok,gearbox-radial-load,0.5008,3.5050,18.72
,1FK7042-2AK7,,6SL3120-1TE15-0AD0,4.60,9.20,ok,motor-thermal,0.5409,3.5036,54.09
...
```

**Axis combinations.** `--axes` keeps the config's cycle, payload and strokes and tries every motor on every axis of the catalogue of the machine's kind: on the T-gantry the Festo EXCT-15, -30 and -100 and Bahr's X/Z gantries ELZI 30, 40 and 60; on the single axis Bahr's 56 single axes. Each axis goes in as `[parts] axis` does ([parts by name](#check-a-real-catalogue-machine)). Before any motor is tried, the axis's catalogue numbers rule it out where the stroke is longer than it has (`axis-stroke`; an EXCT gets the shortest Z stroke Festo offers that holds the config's), the payload is above its rating (`axis-payload`; Festo's rated load, or Bahr's dynamic vertical load on the slide against the Z axis and payload), or an axis limit of the config above its speed or acceleration (`axis-speed`). Such an axis has one row without motor. On the EXCT-30's cycle, all S210 motors at 400 V:

```text
$ sim-cli search t-gantry --config crates/machine-t-gantry/exct-30.toml --axes
Ruled out by catalogue numbers against the axis limits, without simulating a cycle: 124 / 338 candidates
Search time on 16: 7.8 s
axis,motor,gearbox,drive,mass_kg,cost_kg,result,tightest_check,tightest_utilisation,peak_torque_nm,winding_rise_k
ELZI-40,1FK2104-5AF,,6SL3210-5HE10-8UF0,2.85,24.75,ok,settling,0.9056,3.9951,25.02
ELZI-40,1FK2104-5AK,,6SL3210-5HE11-5UF0,2.85,24.75,ok,settling,0.9200,3.9968,51.69
ELZI-40,1FK2204-5AF,,6SL3210-5HE10-8UF0,2.90,24.85,ok,settling,0.6389,3.8853,26.94
...
EXCT-30,1FK2104-5AF,,6SL3210-5HE10-8UF0,2.85,46.80,ok,settling,0.5314,3.2863,20.38
...
EXCT-100,1FK2104-5AF,,6SL3210-5HE10-8UF0,2.85,56.93,ok,settling,0.7410,5.0097,52.68
...
EXCT-30,1FK2204-5AK,,6SL3210-5HE11-5UF0,2.90,46.90,FAIL,motor-thermal,1.6713,3.4264,167.13
...
ELZI-30,,,,NaN,NaN,FAIL,axis-speed,NaN,NaN,NaN
EXCT-15,,,,NaN,NaN,FAIL,axis-stroke,NaN,NaN,NaN
```

The Bahr ELZI 40 is the lightest axis that carries this cycle (19.05 kg at 1 m by 0.25 m, against 41.1 kg for the EXCT-30), so it ranks first. The EXCT-15 has no 250 mm Z stroke; the ELZI 30 allows 2 m/s, the config asks for 2.5 m/s. Read the ELZI rows with care: the config's shaper, belt pretension, friction, frame and tool estimates are the EXCT-30's, and Bahr publishes neither the gantry's rated payload nor the belt's stretch (the belt is Megadyne's HTD-compatible MTD5 at the sheet's width).

Limits for this task:

- **Weight stands in for price.** See above. The drive's weight is not counted: the converter files have none.
- The retuning is a rule of thumb, not a tuning from the machine's frequency response. A candidate can still fail for its gain. Check a chosen motor with `tuning`.
- The torque screen leaves out gearbox efficiency.
- The axis search keeps everything of the config that the axis does not set: its gearbox, friction, frame, shaper and gains. Bahr's single axes state no moved mass, so the config's carriage stays for them; Bahr's maximum lengths are taken as the longest stroke.
- Belts are not searched; a belt is named with `--belt`. Combinations of different motors per axis are not searched.
- The DC link capacitance and the braking resistor stay as the config has them; the braking threshold and the trip level are scaled to the motor's DC link voltage (see the previous section). On a default config the braking rows of every candidate are judged against placeholders.
- A candidate that asks for more than its rated speed at the axis limits never passes, even if its simulated cycle would.

## See which unknown parameters matter, and judge with error bands

A config gives each uncertain value a low and a high bound in `[bounds]`, a table that mirrors the config:

```toml
[bounds]
belt.pretension_n = [350.0, 550.0]
friction_x.coulomb_n = [12.0, 26.0]
```

**Sensitivity.** The cycle is run with each parameter at its low and at its high value, one at a time, and the rows are sorted by how much the results change. `ratings.bearing_reliability_percent` is left out: it is a choice of how to judge, and 20 % either way leaves its range of 90 to 99.95 %:

```text
$ sim-cli sensitivity t-gantry --config crates/machine-t-gantry/exct-30.toml --group mass --group friction_x --group belt
parameter,value,low,high,peak_torque_low,peak_torque_high,rms_torque_low,rms_torque_high,cycle_time_low,cycle_time_high,belt_tension_min_low_n,belt_tension_min_high_n
friction_x.viscous_n_s_m,20.0,10.0,40.0,-0.0436,+0.0869,-0.0172,+0.0469,+0.0000,+0.0000,264.84,242.04
mass.z_axis_kg,4.7,3.7600000000000002,5.64,-0.0620,+0.0624,-0.0675,+0.0689,+0.0000,+0.0000,268.68,246.11
mass.carriage_kg,7.16,5.728000000000001,8.592,-0.0541,+0.0548,-0.0472,+0.0487,+0.0000,+0.0000,272.13,243.18
mass.payload_kg,3.0,2.4000000000000004,3.5999999999999996,-0.0343,+0.0342,-0.0279,+0.0283,+0.0000,+0.0000,265.01,250.05
friction_x.coulomb_n,20.0,12.0,26.0,-0.0190,+0.0143,-0.0096,+0.0078,+0.0000,+0.0000,263.44,253.31
...
belt.pretension_n,450.0,350.0,550.0,-0.0000,-0.0000,+0.0000,-0.0000,+0.0000,+0.0000,157.77,357.77
...
belt.tooth_mesh_stiffness_n_m,5000000.0,2500000.0,10000000.0,+0.0000,+0.0000,+0.0000,+0.0000,+0.0000,+0.0000,257.77,257.77
```

| Column | Meaning |
| --- | --- |
| `parameter`, `value` | Config path and its value in the config |
| `low`, `high` | The two values the cycle was run with: the config's bounds, or the value times (1 − delta) and (1 + delta) where it has none |
| `peak_torque_low`, `_high` | Relative change of the peak torque (a fraction: `+0.0671` is +6.71 %), of the motor it changes most |
| `rms_torque_low`, `_high` | The same for the RMS torque |
| `cycle_time_low`, `_high` | Relative change of the cycle time |
| `belt_tension_min_low_n`, `_high_n` | Lowest belt tension in N at the low and at the high value. An absolute value, not a change |

| Option | Effect |
| --- | --- |
| `--group TABLE` | Only vary this config table. Repeatable. Without it every table is varied except `sim`, `effects`, `drive`, `profile`, `start`, `axis` and `temperature` |
| `--delta FRACTION` | Width for values without bounds. Default 0.2 |

The masses in the example have no bounds in the config, so they were varied by 20 %. A parameter whose changed value the machine refuses is left out of the table. The pretension hardly moves the torques, so it stands far down, but it sets the lowest belt tension: 158 N at its low bound. The last row shows a parameter of an effect that is switched off: it changes nothing.

**Error bands on the verdicts.**

```text
$ sim-cli verdicts t-gantry --config crates/machine-t-gantry/exct-30.toml --bands
Thermal start: warm, at the rises the cycle settles at; largest change of a rise over the measured pass 0.000 K
check,motor,value,limit,result,low,high,worst_case
motor-peak,a,0.2930,1.000,ok,0.2826,0.3150,ok
motor-thermal,a,6.882,100.0,ok,6.640,7.565,ok
drive-current,a,1.145,5.000,ok,1.125,1.198,ok
drive-peak-current,a,2.876,19.00,ok,2.773,3.093,ok
pulley-torque,a,3.126,12.50,ok,2.993,3.395,ok
pulley-bearing-life,a,325762,20000,ok,73165,1194835,ok
belt-tooth-load,a,0.3529,1.000,ok,0.2941,0.5042,ok
dc-link-energy,a,9.943,9.943,ok,4.971,23.23,ok
braking-resistor-continuous,a,8.284,100.0,ok,4.520,13.14,ok
braking-resistor-peak,a,200.5,5700,ok,138.6,218.0,ok
dc-link-overvoltage,a,770.0,820.0,ok,709.1,790.0,ok
...
settling,,0.00003949,0.0001000,ok,0.00003447,0.00004435,ok
belt-tension,,555.0,2140,ok,455.0,655.0,ok
belt-slack,,257.8,0,ok,157.8,357.8,ok
gearbox-radial-load,,1000,2000,ok,800.4,1200,ok
guide-load,,185.0,2000,ok,177.0,276.7,ok
guide-life,,3024374,20000,ok,378047,24194992,ok
thermal-warm-start,,0.00002993,0.1000,ok,0.00002433,0.0001190,ok
```

`low` and `high` are the lowest and highest value the check took over all runs: the config as it is, and each bounded parameter at its low and at its high bound. `worst_case` is `ok` only if the check passed in every run. Here every check passes in every run: with the belt set to its low bound of 350 N the slackest section keeps 158 N, the tool is at most 0.044 mm off its target in the dwell, and the belt pulls a pulley shaft with at most 1200 N. The pulley shaft bearings last 325800 h as written and at least 73200 h, against 20000 h required. Their load rating and the lubrication values behind a_ISO are estimates (Festo publishes none; see the config's comments). This run simulated 129 cycles (64 bounded values) on all processor cores and took about 4.6 s on the machine this guide was written on.

An earlier state of this config failed here: with 300 N of pretension a belt set to 200 N went slack, and with the frame at the soft end of its bounds the tool was 0.16 mm off in the dwell, against a tolerance of 0.1 mm (the tool position counts the frame's sway). The pretension was raised to 450 N, the X jerk limit lowered from 500 to 150 m/s³, and input shaping and corner blending were switched on; later a second shaper at 17.5 Hz was added for the worst corner (below). The cycle takes 2.048 s now, against 2.136 s before.

**All bounded values at once.** With `--samples N` the cycle is run N times, each time with all bounded values between their bounds:

```text
$ sim-cli verdicts t-gantry --config crates/machine-t-gantry/exct-30.toml --bands --samples 200
Thermal start: warm, at the rises the cycle settles at; largest change of a rise over the measured pass 0.000 K
check,motor,value,limit,result,low,high,worst_case,pass_share
motor-peak,a,0.2930,1.000,ok,0.2799,0.3440,ok,1.000
motor-thermal,a,6.882,100.0,ok,6.423,8.347,ok,1.000
drive-current,a,1.145,5.000,ok,1.107,1.257,ok,1.000
drive-peak-current,a,2.876,19.00,ok,2.747,3.380,ok,1.000
pulley-torque,a,3.126,12.50,ok,2.953,3.717,ok,1.000
pulley-bearing-life,a,325762,20000,ok,33442,4107774,ok,1.000
belt-tooth-load,a,0.3529,1.000,ok,0.2875,0.5661,ok,1.000
dc-link-energy,a,9.943,9.943,ok,4.781,28.60,ok,1.000
braking-resistor-continuous,a,8.284,100.0,ok,0.02972,13.42,ok,1.000
braking-resistor-peak,a,200.5,5700,ok,6.115,231.4,ok,1.000
dc-link-overvoltage,a,770.0,820.0,ok,675.2,789.5,ok,1.000
...
settling,,0.00003949,0.0001000,ok,0.00002566,0.00005005,ok,1.000
belt-tension,,555.0,2140,ok,446.3,665.0,ok,1.000
belt-slack,,257.8,0,ok,139.6,358.1,ok,1.000
gearbox-radial-load,,1000,2000,ok,797.3,1209,ok,1.000
guide-load,,185.0,2000,ok,173.0,308.2,ok,1.000
guide-life,,3024374,20000,ok,187858,23944959,ok,1.000
thermal-warm-start,,0.00002993,0.1000,ok,0.00001024,0.001698,ok,1.000
```

The values are drawn as a Latin hypercube: each value's range is cut into N equal parts, each part is used in exactly one run, and the order of the parts is random per value. The random numbers come from `--seed S` (default 1), so the same call gives the same table. `low`, `high` and `worst_case` are the envelope of these runs and the config as it is. `pass_share` is the share of the runs, the config as it is included, in which the check passed. Here the bands are wider than one at a time (140 N instead of 158 N in the slackest section, 0.050 mm instead of 0.044 mm off in the dwell, 1209 N on a pulley shaft), and every check passes in all 201 runs. The pulley shaft bearings last 33400 h at least. That took about 7 s.

`--samples` without `--bands`, 0 samples, a fraction, and `--seed` without `--samples` are refused (exit code 2). The runs of both kinds use all processor cores.

**The worst corner.** With `--worst` the bands search, per check, for the combination of bounds that makes it worst:

```text
$ sim-cli verdicts t-gantry --config crates/machine-t-gantry/exct-30.toml --bands --worst
Cycles simulated in the search for the worst corners: 2553
check,motor,worst,limit,worst_case,at_high_bound,at_low_bound
motor-peak,a,0.4109,1.000,ok,...
...
pulley-bearing-life,a,10042,20000,FAIL,...
...
dc-link-energy,a,35.66,36.57,ok,...
...
settling,,0.00009437,0.0001000,ok,...
belt-tension,,687.7,2140,ok,...
belt-slack,,86.90,0,ok,...
gearbox-radial-load,,1231,1500,ok,...
guide-load,,356.5,1000,ok,...
guide-life,,115606,20000,ok,...
thermal-warm-start,,0.01726,0.1000,ok,...
```

It starts from the one-at-a-time runs: each check gets the corner where every bounded value is at the bound that made that check worse alone. Then, up to three times, each value that moved the check is flipped to its other bound, one at a time, and what made it worse is kept (all such flips together, or the best one alone, whichever is worse). The last two columns name the values at their high and at their low bound, separated by `;`. "Worse" is judged by the margin to the limit, because some limits are bounded themselves: in this config the DC link's capacitance and threshold, so the `limit` column shows the limit at that corner. `dc-link-energy` is judged by the energy instead: it fails only where the braking resistor is needed and has no rating, and with a resistor the link fills up to its room at every corner that brakes hard, so its margin says nothing (here 35.66 J of 36.57 J of room, with the capacitance at its high bound; 9.94 J as configured). The one-at-a-time runs and the config as it is count as candidates too, so the worst is never better than the bands; where one of them is the worst, the values it leaves as configured are named in neither column. The search took 2553 cycles here.

On the EXCT-30 it found what neither band mode found: with the 25 Hz shaper alone and all values at their worst at once the tool was 0.115 mm off its target in the dwell, against 0.1 mm allowed (0.043 mm one at a time, 0.053 mm in 200 samples). A second shaper at 17.5 Hz, the peak of the tool's response, brings it to 0.094 mm (the comment of `profile.shaper2_frequency_hz` in the file lists what else was tried). The belt keeps 87 N in its slackest section and the shafts carry 1231 N of their 1500 N at the lowest rating bound. The pulley shaft bearings fail at their worst corner, every bounded value against them at once: 10000 h of 20000 h. Their rating and lubrication values are estimates, so this is the part to ask Festo about, not proof that the axis wears out.

`--worst` without `--bands`, or with `--samples` or `--seed`, is refused (exit code 2). The app has it as "worst corner" beside the error bands.

Limits for this task:

- **Plain `--bands` is one at a time.** Two values that are both off can take a result beyond its band. `--samples` shows that.
- **Samples do not find the worst corner.** A sample never sits exactly on a bound (plain `--bands` does that), and a few dozen runs do not find the worst combination of many values. `--worst` searches for it, but it is a search, not a proof: a worst case that no chain of single flips from the corner of the one-at-a-time signs reaches, or one between the bounds (a stiffness that puts a resonance on the cycle), can be missed. Run all three.
- The bounds are themselves estimates. In the default configs only friction, damping, stiffness, pretension, ripple and gearbox efficiency are bounded, not the masses, inertias and backlash.
- Where a bound changes a rating (the EXCT-30 config bounds its DC link ratings), the `limit` column still shows the nominal limit.
- A bound on a key that the config does not have is refused. So is a pair where low is above high.

## Shorten a cycle

```text
$ sim-cli optimise t-gantry --config crates/machine-t-gantry/exct-30.toml
setting,before,after
cycle_s,2.0480,1.5900
dwell_error_peak_m,3.949e-5,5.295e-5
bands_fastest_cycle_s,,1.4920
bands_fastest_fails,,belt-slack settling
bands_steps_back,,1
bands_excused,,
Cycles simulated: search + error bands 40 + 387
axis.x.max_acceleration_m_s2,25.0,53.99999999999999
axis.x.max_jerk_m_s3,150.0,225.0
axis.x.max_velocity_m_s,2.5,3.0
axis.z.max_acceleration_m_s2,15.0,32.4
axis.z.max_jerk_m_s3,500.0,750.0
axis.z.max_velocity_m_s,1.5,1.7999999999999998
effects.input_shaping,true,false
```

The optimiser tries corner blending and higher velocity, acceleration and jerk limits (all axes together, in steps of ×1.5 and ×1.2). It keeps a change if the cycle gets shorter and every verdict still passes, among them `settling`, so a faster move that leaves the tool swinging at the pick does not count. Input shaping makes no cycle shorter by itself, because every stop waits one period of the shaper. So it is tried where it can help: a faster setting that fails on `settling` alone is run again with the shaper (and with `effects.shaper_follows_pose` where the machine reports a resonance) and taken if it then passes and is shorter; one that still fails on `settling` alone with the shaper is run again with a second shaper (see [Tune gains and use input shaping](#tune-gains-and-use-input-shaping)). Where the shaper is on already, switching it off is tried too. The first two rows are the cycle time and the dwell error before and after. The other rows are what to change in the config. No config file is edited.

**In the error bands.** The result also has to pass every check in the error bands of the config's `[bounds]` (one value at a time, as `verdicts --bands`), not only with the config as written. The search itself judges as written; then its fastest setting is judged in the bands, and where it fails there, the steps the search took are gone back, fastest first, until one passes (2n + 1 cycles per step judged, n the bounded values). A check that the cycle as configured fails in its bands already cannot be kept, so it is not judged: it is listed as `bands_excused`. The `bands_*` rows say how fast the search got on the config as written (`bands_fastest_cycle_s`), what that fails in the bands, and how many steps back the result is; the error stream says how many cycles the search and the bands took.

The EXCT-30 config has corner blending and the shaper on already, so here the optimiser raises the limits and switches the shaper off (both shapers). On the config as written that gives 1.49 s, but with the values at their bounds the belt goes slack and the tool is 0.29 mm off in the dwell. One step back, with the jerk of the last step taken back, passes: 2.05 s to 1.59 s, the tool 0.053 mm off in the dwell instead of 0.039 mm. The shipped config passes every check in its bands, so nothing is excused (`bands_excused` is empty). The bands took 387 cycles (three settings at 129 each, 64 bounded values), the search 40; the whole run took about 20 s on 16 threads. The default T-gantry goes from 6.08 s to 5.36 s (5.34 s as written, which fails `settling` in the bands), with 159 cycles of bands.

`--nominal` judges the config as written alone, as before: on the EXCT-30 1.49 s. `--runs N` limits the number of cycles of the search (default 40); the bands come on top.

**Energy against cycle time.** `--pareto FILE` writes the settings that no other beats in cycle time, energy per cycle from the supply and peak motor utilisation at once, fastest first, as CSV: `cycle_s,energy_j,peak_utilisation,velocity_factor,acceleration_factor,jerk_factor,corner_blending,input_shaping` (the factors on the axes' limits as configured). They come from the settings the search ran anyway that pass every check as written, and six slower ones from the result (acceleration and jerk, and all three limits, at 0.8, 0.6 and 0.4 times). The energy is what the drives drew less what they fed back, plus what the braking resistor burned; with no resistor (no DC link simulated) braking energy counts as fed back. On the EXCT-30 the front runs from 1.49 s at 269 J and a peak utilisation of 0.52 to 2.94 s at 118 J and 0.22; the result's 1.59 s takes 254 J. A slower acceleration saves the kinetic energy a stop burns in the resistor, ½ m v² where the move does not reach its top speed (`a_lower_acceleration_takes_the_hand_calculated_braking_energy_less_from_the_supply`). The app draws the front below the optimiser's table.

**Lift heights and rounded lifts.** The cycle's geometry is a variable too. A lift, traverse and lower (lift straight up from the pick, traverse at that height, lower straight down to the place; the default T-gantry has two) is tried first as low as `profile.min_clearance_m` allows: that far above the higher of pick and place, and where the config has `obstacles`, that far above the obstacles in the way of the traverse, measured from the machine's collision bodies (see below). The shipped T-gantry configs set it to the lifts they have, so nothing changes there. With an obstacle of 0.1 m:

```text
$ sim-cli optimise t-gantry --set profile.min_clearance_m=0.1
setting,before,after
cycle_s,6.0800,3.4420
...
cycle.segment[1-2-3].lift_m,0.3500,0.1500
cycle.segment[4-5-0].lift_m,0.3500,0.1500
```

With the other steps the cycle takes 3.44 s (3.36 s with `--nominal`, which fails `settling` in the bands). Lowering alone saves 2.000 s: four Z moves of 0.2 m less at 0.4 m/s (`the_optimiser_lowers_the_lifts_to_the_clearance_and_saves_the_hand_calculated_time`). Segments are counted from 0. Last, once nothing else helps, the optimiser also tries to round a lift's two corners: one segment that goes straight up, through a via point on a spline, and straight down (`via` in [`[[cycle.segment]]`](config-reference.md#cyclesegment)), timed by the path planner, as long as the path keeps the clearance; such a row reads `cycle.segment[1-2-3].via_m,,0.5000 0.3000`. On the default T-gantry it never won: an arch is slower than two stops with corner blending. In the app, "Apply" writes the new cycle into the config (saved, the file is then written without its comments).

**Over obstacles.** Where the config has `obstacles`, each lift is lowered no further than its traverse keeps `profile.min_clearance_m` to them with the machine's collision bodies (the T-gantry's Z axis, flange and payload), looked at every millimetre at rest. A box across the traverse puts the lift at its top plus the clearance plus how far the lowest body reaches below the tool. With a 0.1 m box under the middle of the default T-gantry's traverse:

```text
$ sim-cli optimise t-gantry --set profile.min_clearance_m=0.05 --set 'obstacles=[{ name = "box", centre_m = [0.65, -0.05, -0.65], size_m = [0.1, 0.2, 0.1] }]'
setting,before,after
cycle_s,6.0800,4.0840
...
cycle.segment[1-2-3].lift_m,0.3500,0.2521
cycle.segment[4-5-0].lift_m,0.3500,0.1521
```

By hand: the box's top at -0.6 m in the view, the payload's bottom 0.802 m below the tool on the way there, the flange's 0.702 m on the way back: -0.6 + 0.05 + 0.802 = 0.252 m and 0.152 m (without the box both would go down to 0.1 m). The lifts end 0.1 mm higher: the run sways the payload 0.05 mm nearer than the plan at rest, and the bounds a little more, so the optimiser runs such a setting again with the lifts higher by what was missing (`the_optimiser_lifts_each_traverse_over_a_box_across_it_by_hand`). A lift that an obstacle raised is also tried rounded through a via point over the obstacle, its ends as low as pick and place allow; a via point whose planned path comes nearer to an obstacle than the clearance is not run.

Limits for this task:

- A search by trial, not a proof of the optimum. It stops at the first setting from which no single step is an improvement.
- Lift heights and via points of the cycle are not varied.
- The cycle as configured has to pass every verdict to begin with, or the command is refused.
- A candidate whose axis limits exceed the motor limits (the warning of [Common to all commands](#common-to-all-commands)) is not accepted.
- It judges the error bands one value at a time, not the samples or the worst corner. Run `verdicts --bands --worst` on the result. The result above takes back the margins the config was given for its worst case: at the worst corner it fails `settling` (0.71 mm) and `belt-slack` (0 N), and `pulley-bearing-life` as the shipped config does.

**Corner blending by hand:** `--set effects.corner_blending=true`, with `profile.blend_time_s` as the longest lead. The next segment starts while the current one is still braking. It applies only where the machine need not stop: no dwell, no gripper action, no axis reversing, and only on the ramps. On lift-traverse-lower cycles every axis stays within its limits; for other shapes of cycle the sum of two segments on one axis is not checked against the limits.

## Size a delta picker

`delta-d3` is a three-arm rotary delta (Clavel), sized like the Autonox DELTA RL3-1200-3kg: working range 1200 mm across, 200 mm high at the edge and 386 mm at the centre, 3 kg payload, three Siemens 1FK2105-4AF motors on S210 converters behind planetary gearboxes of ratio 20. Autonox does not publish its arm lengths and masses; [the config](../crates/machine-delta-d3/default.toml) marks every value `[catalogue]` or `[estimate]` and bounds the estimates. Its cycle is the Adept cycle 25/305/25 mm (up 25 mm, across 305 mm, down 25 mm and back), each leg a move of its own with a stop, and 35 ms at pick and place for the gripper.

The tool is the platform's centre, x, y and z in metres, z up from the plane of the arms' shoulder axes (so the platform works at about −0.7 to −1.1 m). A target outside the working range is refused with its reason: outside the cylinder or below its spherical bottom, out of the arms' reach, beyond an arm's stop, or too near a singularity.

```text
$ sim-cli analyze delta-d3
Thermal start: warm, at the rises the cycle settles at; largest change of a rise over the measured pass 0.008 K
motor,cycle_s,peak_torque_nm,rms_torque_nm,peak_speed_rad_s,utilisation,thermal_utilisation,...,rms_current_a,winding_rise_k,...
1,0.720000,7.642678,2.575830,120.0682,0.5101,0.5600,,,2.4400,24.83,...
2,0.720000,3.986994,1.586898,64.8991,0.2677,0.3450,,,1.4824,8.64,...
3,0.720000,3.986994,1.586898,64.8991,0.2677,0.3450,,,1.4824,8.64,...
```

The cycle lies along x, so arm 1 (pointing along +x) works hardest, and arms 2 and 3 alike. Its 0.720 s are the S-curves by hand (traverse d/v + v/a + a/j = 0.1817 s, a 25 mm lift 4 (d / 2j)^(1/3) = 0.0684 s, 0.7069 s with the dwells) on the controller's 1 ms profile cycle (`the_adept_cycle_takes_the_hand_calculated_time`). The makers' figures for the same cycle are shorter (ABB IRB 360-3/1130: 0.40 s at 1 kg, 0.52 s at 3 kg): they round the corners instead of stopping, and run faster than the 3 m/s and 50 m/s² this config allows each axis. `sim-cli optimise delta-d3` rounds the corners through via points (splines on three axes) and raises the limits as far as every check allows.

```text
$ sim-cli verdicts delta-d3
motor-peak,1,0.5101,1.000,ok
motor-thermal,1,24.83,100.0,ok
drive-current,1,2.440,5.000,ok
gearbox-continuous,1,66.79,120.0,ok
gearbox-peak,1,133.9,192.0,ok
gearbox-life,1,671269,20000,ok
gearbox-gearing-life,1,11579,20000,FAIL
...
settling,,0.0006760,0.0005000,FAIL
singularity,,2.072,5.000,ok
thermal-warm-start,,0.007974,0.1000,ok
```

With `--bands` every check but `settling` and `gearbox-gearing-life` passes at both bounds of every estimate (peak gearbox torque 125 to 143 N m); `settling` fails at all of them (0.623 to 0.735 mm), the gearing's life lies between 5100 and 36300 h. What to read from them:

- **Gearbox.** The arm's gearbox carries up to 134 N m of its 192 N m; the cubic mean, 67 N m, is what its 120 N m rating is for. Its planet bearings last 671000 h. The peaks above 120 N m wear the gearing: by Miner's sum on Siemens' two torque figures it lasts about 11600 h of the 20000 h required on arm 1 (arms 2 and 3 stay below 120 N m). That figure is an estimate (see the [model reference](model-reference.md#analysis-quantities)); check the gearbox size with Siemens.
- **Settling fails: a sizing finding.** It is judged against 0.5 mm, an estimate of the positioning accuracy of the class (Festo EXPT: ±0.5 mm), not the 0.1 mm repeatability the makers state. The gearboxes' twist and backlash under the platform's weight leave the tool off where the motors' encoders put it: 0.64 mm at pick and place at rest with the NRB080's catalogue backlash of 9 arcmin, 0.68 mm in the dwell (with the motion before it); up to 2.09 mm at the rim's lower edge. That offset is the same every pass, which repeatability does not see, but it is beyond the class's accuracy: this delta needs a gearbox with less backlash. With 3 arcmin (`--set gearbox.backlash_rad=8.727e-4`, a robot-grade gearbox, not in the catalogue) it passes: 0.29 mm at rest at pick and place, 0.33 mm in the dwell, 0.30 to 0.38 mm over the bounds. The backlash does not make the loops ring: at rest the tool stands still to 2e-8 m with either.
- **Singularity**: the highest condition number of the Jacobian in the cycle, against `ratings.max_condition_number` (5). Near a singularity a little motor angle moves the platform a lot and the motor torques for a force grow without bound; the working range has up to 3.6, the cycle 2.07.
- **No belt, pulley or guide checks**: the delta has none.

To size for another payload, change `mass.payload_kg` (and `ratings` where the parts change); for another motor or gearbox, name them as on the other machines (`--motor`, `--gearbox`, `sim-cli search delta-d3`). Paths on three axes are written `line,x,y,z`, `spline,x,y,z` and `arc,x,y,z,cx,cy,cz,cw` (see [run a path](#run-a-path-and-read-path-error)).

The forearms are rigid rods with their mass and rotational inertia (`effects.forearm_inertia`; off, the lumped model of Codourey and Olsson, which puts 2.7 % more RMS torque on motor 1 in this cycle: 2.645 against 2.576 N m, peak 7.791 against 7.643 N m). With `effects.link_elasticity` (off by default; its stiffness is an estimate) the upper arms bend and the platform rings after a move: the cycle's tracking error grows from 4.0 to 5.3 mm and the dwell error from 0.68 to 0.73 mm. The lowest mode with the motors held (the gearboxes' springs, with the switch the arms' bending too) is what `effects.shaper_follows_pose` sets the input shaper to: at the centre with the arms level the platform's sideways mode, 28 Hz on the gearboxes alone, 23 Hz with bending arms (the forearms lumped). The forearm rods are collision bodies: a forearm that runs into an obstacle is a crash.

What the delta does not have: elastic rods and ball joints, the arms' bending out of the plane they turn in, the ball joints' cone limits (the stops and the working range stand in), a fourth axis, a transition curve where two arcs meet in different planes, and a friction view for its joints (their losses are in the energy budget). Trace replay and fit take it since 13 November, the fit in its motors' coordinates (see [Replay a drive trace and fit parameters](#replay-a-drive-trace-and-fit-parameters)). Held by constant motor torques it is unstable in its vertical mode in the upper part of its range; the drives' position loops hold it (see the [model reference](model-reference.md#delta-picker-d3)).

### The planar delta D2

`delta-d2` is a two-arm delta in the vertical x-z plane: two rotary arms, each with a pair of forearms down to a plate that keeps its orientation, sized like the Codian Robotics D2-800-S020 (working range 800 mm wide and 477 mm high, 5 kg payload, up to 210 picks a minute). Codian names no motors; the config uses the D3's 1FK2105-4AF on S210 converters behind ratio 20, and marks every value `[catalogue]` or `[estimate]` ([the config](../crates/machine-delta-d2/default.toml)). The tool pose is (x, z); paths are written `line,x,z` and `arc,x,z,cx,cz,cw` as on the gantries. The cycle is the D3's Adept cycle in the plane, 100 mm below the top of the range: 0.720 s.

```text
$ sim-cli verdicts delta-d2
motor-peak,1,0.5880,1.000,ok
gearbox-continuous,1,69.95,120.0,ok
gearbox-peak,1,154.3,192.0,ok
...
settling,,0.0003475,0.0005000,ok
singularity,,1.136,5.000,ok
thermal-warm-start,,0.03448,0.1000,ok
```

Every check passes, with `--bands` too (settling 0.31 to 0.41 mm, gearbox peak up to 163 N m); the search for the worst corner of all the estimates at once (`--bands --worst`) fails gearbox peak (202 N m) and settling (0.60 mm). Two arms carry the 5 kg where the D3's three carry 3 kg, so the gearbox works nearer its 192 N m. The settling passes because the gearbox's backlash is an estimate of 1 arcmin, a reduced-backlash precision planetary: with the 9 arcmin of the catalogue's NRB080-05-20 (`--set gearbox.backlash_rad=2.618e-3`) it fails, 0.85 mm, for the D3's reason: both arms always hold the plate up and rest at one side of their backlash. The model keeps the plate level, as the real machine's parallelograms do (see the [model reference](model-reference.md#delta-picker-d2)).

## Size a linear delta

`delta-linear` is a linear delta: three vertical toothed-belt axes on the columns of a frame, each carriage with a pair of rods down to a plate that stays level. It is sized like the Festo EXPT-95 tripod: working range 950 mm across and 100 mm high, EGC-80 belt axes driven without gearbox, 1 kg rated load at its highest dynamics, here with three Siemens 1FK2205-2AF motors on S210 converters. Festo publishes neither rod length nor column layout nor moving masses; [the config](../crates/machine-delta-linear/default.toml) marks every value `[catalogue]` or `[estimate]` and bounds the estimates. Its cycle is the D3's Adept cycle 25/305/25 mm, at the top of the range.

The tool is the plate's centre, x, y and z in metres, z up from the carriages' upper stop (the plate works at −1.12 to −1.22 m). A target is refused with its reason where it lies outside the cylinder, out of the rods' reach, where a rod would stand steeper than 60° from the vertical, where a carriage would leave its stroke, or too near a singularity.

```text
$ sim-cli analyze delta-linear
Thermal start: warm, at the rises the cycle settles at; largest change of a rise over the measured pass 0.030 K
motor,cycle_s,peak_torque_nm,rms_torque_nm,peak_speed_rad_s,utilisation,thermal_utilisation,belt_tension_min_n,belt_tension_max_n,rms_current_a,winding_rise_k,...
1,0.720000,4.775703,2.025348,106.1601,0.4429,0.6751,143.23,527.74,1.6426,35.34,...
2,0.720000,3.925325,1.367014,58.7263,0.3640,0.4557,143.23,527.74,1.1045,14.86,...
3,0.720000,3.925325,1.367014,58.7263,0.3640,0.4557,143.23,527.74,1.1045,14.86,...
```

The cycle lies along x, towards column 1 and away from it, so column 1 works hardest. Its 0.720 s are the D3's: the same cycle and axis limits. Festo times the EXPT-95 at up to 150 of these cycles a minute at light loads (0.4 s each) with rounded corners and up to 7 m/s and 110 m/s²; `sim-cli optimise delta-linear` rounds the corners and raises the limits as far as every check allows.

```text
$ sim-cli verdicts delta-linear
motor-peak,1,0.4429,1.000,ok
motor-thermal,1,35.34,100.0,ok
drive-current,1,1.643,3.000,ok
pulley-torque,1,3.980,10.00,ok
...
settling,,0.00007375,0.0005000,ok
belt-tension,,527.7,1000,ok
belt-slack,,143.2,0,ok
singularity,,3.278,5.000,ok
gearbox-radial-load,,783.0,1500,ok
thermal-warm-start,,0.02976,0.1000,ok
```

Every check passes, with `--bands` at both bounds of every estimate too. What to read from them:

- **Motor.** A Compact motor, for its rotor: the High Dynamic 1FK2104-5AF has the torque (its 7.5 N m against 4.8 N m needed), but 0.65 kg cm² of rotor against about 6 kg cm² of carriage, plate and pulley at its shaft: its velocity loop rang near 450 Hz above 0.1 N m s/rad and the cycle settled to 2.5 mm, not 0.5 mm. The 1FK2205-2AF's 4.0 kg cm² take a gain of 0.25 with at least 38° of phase margin over the bounds, and an integral time of 0.012 s (0.01 s left 34°), short so that the integrator pushes the carriages through their guides' friction soon after a move.
- **Belts.** Each column's belt carries the carriage's weight and its share of the plate's (22 N at the centre) and what accelerating them takes: 143 to 528 N in its two sections, against the pretension of 300 N.
- **Singularity.** The condition number is 3.15 already at the centre: a metre of carriage moves the plate further sideways than up. The range has up to 3.98, at its bottom rim opposite a column.
- **No gearbox, pulley-bearing, guide or tooth-load checks**: the axes have no gearbox, and their pulley bearings, guides and belt teeth have no ratings in the config (the EGC-80's are not in the catalogue).

What the linear delta does not have: the rods' own rotational inertia, elastic frame, rods and joints, a fourth axis and the guides' heat. Trace replay and fit take it, the fit in its motors' coordinates as on the D3. See the [model reference](model-reference.md#linear-delta).

### The four-axis delta D4

`delta-d4` is the D3 with a fourth axis that turns the tool about the vertical, as ABB's IRB 360 with its fourth axis: a fourth 1FK2105-4AF in the centre of the base, behind a planetary gearbox of ratio 8 (Siemens NRB060-05-8), drives a telescopic shaft with a cardan joint at each end down to the tool's shaft in the platform. The tool pose is (x, y, z, c), c in rad; `start.pose` and the cycle's targets have four values. Its cycle is the D3's Adept cycle with the tool turned by 90° on each traverse (+90° to the pick, back to 0° to the place): still 0.720 s, the turn fits into the traverse. The two cardan joints are bent alike (both shafts vertical), so the tool turns exactly with the motor; only the shaft between them turns unevenly (`shaft.speed`). The shaft's weight share and the telescope's friction load the arms ([the config](../crates/machine-delta-d4/default.toml), [model reference](model-reference.md#delta-picker-d4)).

```text
$ sim-cli verdicts delta-d4
motor-peak,1,0.5631,1.000,ok
gearbox-peak,1,149.1,192.0,ok
motor-peak,4,0.05336,1.000,ok
...
settling,,0.0004035,0.0005000,ok
tool-gearbox-peak,4,2.262,29.00,ok
singularity,,2.072,5.000,ok
thermal-warm-start,,0.02011,0.1000,ok
```

Every check passes. Motor 1 carries a little more than on the D3 (peak 0.563 against 0.510 of its limit): the shaft's 0.7 kg at the platform and the telescope's friction. The fourth motor works at 5 % of its peak: the 1FK2105 is far larger than the turn needs, but a machine has one motor type here. The gearboxes' backlash is an estimate of 3 arcmin on the arms and 1 arcmin on the tool (reduced-backlash precision planetaries, as on the D2): with the catalogue's 9 and 10 arcmin (`--set gearbox.backlash_rad=2.618e-3 --set tool_gearbox.backlash_rad=2.909e-3`) settling fails, 1.22 mm, 0.6 mm of it the arms' sag and 1 mrad the tool resting in its backlash: settling counts the turn's error in rad with the position's in m, as on the SCARA. With `--bands` settling fails: it reaches 0.53 mm at a bound of the estimates. A path with a turn is written `line,x,y,z,c`; arcs and splines take three axes and are refused on the D4. `sim-cli modes delta-d4` lists the tool's torsion (41 Hz with the payload: the telescopic shaft twists) beside the arms' modes.

## Run a path and read path error

A path file has one piece per line. The path starts where the machine stands (`start.pose`).

```text
# line, arc, line from the start pose (0.1, 0.35)
line,0.5,0.35
arc,0.6,0.25,0.5,0.25,cw
line,0.6,0.05
```

| Line | Meaning |
| --- | --- |
| `line,a,b` | Straight to this point: one position per axis, in metres |
| `arc,a,b,ca,cb,cw` or `ccw` | On a circle around the centre (ca, cb) to the point (a, b). `cw` and `ccw` are seen with the first axis to the right and the second upwards |
| `spline,a,b` | On a smooth curve through this point. Lines of `spline` that follow one another form one curve, one via point per line |
| `line,x,y,z`, `spline,x,y,z`, `arc,x,y,z,cx,cy,cz,cw` or `ccw` | The same on a machine with three axes (the delta). An arc lies in the plane of its start, its end and its centre, and `cw`/`ccw` is seen from the side of that plane its normal points to, taken upwards (+z), for a vertical plane towards the viewer (−y), for the y-z plane from +x. A half or full turn leaves the plane open and is refused. Transition curves (`profile.path_tolerance_m`) are laid in the plane of the arc they meet; two arcs that meet in different planes keep their junction, and a line has to lie in its arc's plane to 1 nm (write the numbers with all their digits) |
| empty, or starting with `#` | Skipped |

An arc whose end is the end point of the piece before it (the same numbers, to a nanometre) is a full turn. Any other arc turns from that point to its own end, however short: an arc of 80 µm is an arc of 80 µm. The stated end has to lie on the circle around the centre through the end point of the piece before, within 10 µm. The arc is laid from where the path really stands and keeps the radius it has there, so it ends beside its stated end by what the two radii differ. Over arcs that follow one another around one centre these offsets add up if every stated end lies further out than the one before: four arcs, each 9 µm further out, end 36 µm beside the last stated end. Where the path ends beside a stated end, the largest such distance is part of the deviation the planner reports (`The planned path is off ...`, below, and "Planned path off the given one" in the app). The command line says it from 0.05 µm, the smallest it can show.

The first piece has no point before it. A path starts at the setpoint of the machine at rest, which belt stretch and gear twist put up to 0.1 mm from the start pose (some hundredths of a millimetre on the default gantry). So an arc as the first piece is judged against where the machine stands: its end may be 0.1 mm off the circle through that place, and it has to end at least 1 mm from that place. Nearer, a full turn cannot be told from a short arc, so neither is guessed and the path is refused (`The path starts with an arc that ends less than 1 mm from where the machine stands. ...`). To start a path with a full turn or with an arc shorter than that, state its start first with a `line` to the start pose; the arc is then judged against that point. Arcs need a machine with two axes, or three (in a plane, see the table). On the delta the setpoint at rest can be up to 2.3 mm from the start pose (`Machine::setpoint_slack`: what its gearboxes' twist and backlash can sag under the platform's weight anywhere in the range, worked out from stiffness, backlash and masses; 0.6 mm at the default start pose), and a first arc's end may be that far off the circle. That slack is for where a path starts only: a pose further from the start than the slack has to lie within the range (on an axis whose start lies beyond an end, no further beyond than the start), so a path cannot run past the range where a move there is refused.

A spline runs through all its via points with no step in position, direction or curvature at a via point (a cubic spline over the straight distance between the points). It leaves the line or arc before it in that piece's direction and ends in the direction the line or arc after it starts with. At an end of the path, or where there is no line or arc, it ends without curvature. Splines need a machine with two or three axes, like arcs. Via points and directions that would make the curve turn back on the spot are refused (`The path cannot be laid out: ...`).

A spline can swing past its via points: between them the curve bulges outwards, the more the sharper it has to turn. A curve that leaves the travel range although every via point is inside is refused (`The path leaves the travel range. ...`). Move the via points inwards or add one where it bulges.

```text
$ sim-cli analyze t-gantry --path corner.path
Thermal start: warm, at the rises the cycle settles at; largest change of a rise over the measured pass 0.050 K
motor,cycle_s,peak_torque_nm,...,path_error_peak_m,path_error_rms_m,tracking_error_peak_m,dwell_error_peak_m,...
a,2.128000,0.557940,...,1.821e-4,7.442e-5,1.821e-4,0.000e0,...

$ sim-cli analyze t-gantry --path corner.path --time-optimal
a,1.695000,0.646623,...,2.435e-4,9.641e-5,2.435e-4,0.000e0,...
```

For a path, `cycle_s` is the planned duration plus 0.3 s of rest at the end, so that the settling of the tool counts.

Two planners time the path:

| | Default | `--time-optimal` |
| --- | --- | --- |
| Speed and acceleration | The limits of every axis (`[axis.*]`) | What the motors allow: their rated speed, and `profile.torque_reserve` times their torque-speed curve |
| Jerk | The limit of every axis | The same |
| At a corner (direction turns by more than 1 mrad) | Stops | Stops |
| Where a line meets an arc | Slows down so far that the step in acceleration stays within half the jerk limit; with `profile.path_tolerance_m` a transition curve takes the path into the arc at speed (below) | The same |
| On a spline | Each span between two via points gets one speed, one acceleration and one jerk limit, from its tightest curvature and the fastest change of curvature on it (sampled at 257 places, raised by 5 % as margin), shared out as on arcs | The same, from what the motors allow |
| Where a spline meets a line or an arc | The curvature steps, so it slows down as from a line into an arc; with `profile.path_tolerance_m` the spline's end is eased to the curvature of its neighbour, and the junction is passed at speed (below) | The same |
| Model used to plan | None, only the axis limits | The rigid model: no friction, no belt stretch |

The time-optimal plan keeps every motor within the reserve and the rated speed and every axis within its jerk limit at every planned pose. With the default jerk limit of 100 m/s³ the jerk decides nearly everywhere: the example reaches the rated speed on its first line and uses 0.16 of the torque it may. A higher jerk limit makes it faster (0.79 s plus rest at 2000 m/s³, where the belt's tension limits it).

Reading the result:

- `path_error_peak_m` is the distance of the tool tip from the line its setpoint has drawn, wherever along it. Lagging behind along the path does not count. This is the number for contour accuracy.
- `tracking_error_peak_m` is the distance from where the setpoint is at the same moment. It includes the lag.
- On the T-gantry the tool tip is the tip of the leaning Z axis (`effects.tool_flex`) on a frame that sways (`effects.frame_compliance`), seen from the floor where the workpiece stands. It is not the carriage's position along the beam and not what the motor encoders say.

**Arcs entered at speed.** Where a line runs into an arc the acceleration towards the centre steps from 0 to v²/r, so the default planner passes that place at a crawl: 0.07 m/s for a radius of 0.1 m at 100 m/s³. With `profile.path_tolerance_m` above 0, both planners first put transition curves (clothoids) there, and where two arcs meet: curves whose curvature changes evenly with the distance. To fit one in, the arc keeps its centre and gets a slightly smaller radius, so the planned path is off the lines and arcs of the file by a small amount, and the error stream says by how much:

```text
$ sim-cli analyze t-gantry --path corner.path --set profile.path_tolerance_m=0.0001
The planned path is off the given lines and arcs by up to 0.1000 mm (transition curves, and arcs that end beside their stated end).
Thermal start: warm, at the rises the cycle settles at; largest change of a rise over the measured pass 0.056 K
motor,cycle_s,peak_torque_nm,...,path_error_peak_m,...
a,1.890000,0.558121,...,1.466e-4,...
```

1.89 s instead of 2.13 s, with every axis still within its velocity, acceleration and jerk limit. The rules:

| | |
| --- | --- |
| Deviation of an arc of radius r with curves of length L | e = L² / (24 r). The arc moves in by this much; on the curves the path is closer to the given one |
| Length of the curves | Per arc, L = v³ dk / (0.4 j): what the faster of its two junctions needs to pass at the speed v of its slower piece, with 0.4 of the smallest jerk limit j. dk is the step of the curvature, 1 / r for a line into an arc. But no longer than the tolerance allows on that arc: L = sqrt(24 · tolerance · r). Arcs that hand over to one another turning the same way share one L |
| Speed on a curve that is shorter than needed | v = (0.4 j L / dk)^(1/3) |
| Example above (r = 0.1 m, 100 m/s³, arc at 0.4 m/s) | Needed 16 mm, allowed sqrt(24 · 0.0001 · 0.1) = 15.5 mm: the arc moves in by the full 0.1 mm and is entered at 0.396 m/s instead of 0.071 m/s. With a tolerance of 1 mm the curves are the 16 mm they need, and the path is off by 0.107 mm |

Each arc gets the curves its own junctions need. An arc at a corner, at the start or end of the path, or next to a spline has to end there where the file says. If its other end meets a line or an arc, it still gets its curve: its circle shrinks and moves towards the fixed end so that it passes through that end in the same direction, and leaves the room for the curve at the other end. An arc that turns more than half a circle that way moves by at most half the tolerance (halfway round the new circle is twice that inside the old one), so its curve is shorter. An arc fixed at both ends meets nothing in its direction and keeps its radius. If a piece is too short for its curves, or a curve would turn half a circle or more (a tolerance that is large against the radius of a tight arc), every length of the path is halved, up to eight times; after that the path is planned as given. On three axes (the delta) the curves lie in the plane of the arc they meet. With a tolerance of 0 (the default) nothing changes.

**A path with a spline.** Two via points between two lines:

```text
# line, a curve through two via points, line; from the start pose (0.1, 0.35)
line,0.3,0.35
spline,0.45,0.3
spline,0.55,0.15
line,0.55,0.05
```

```text
$ sim-cli analyze t-gantry --path curve.path
Thermal start: warm, at the rises the cycle settles at; largest change of a rise over the measured pass 0.038 K
motor,cycle_s,peak_torque_nm,...,path_error_peak_m,path_error_rms_m,tracking_error_peak_m,dwell_error_peak_m,...
a,2.101000,0.558226,...,1.866e-4,7.199e-5,1.866e-4,0.000e0,...

$ sim-cli analyze t-gantry --path curve.path --time-optimal
a,1.680000,0.593053,...,2.365e-4,9.838e-5,2.365e-4,0.000e0,...
```

2.101 s within the axis limits and 1.680 s as fast as the motors allow, both with 0.3 s of rest. The curve leaves the first line horizontally and arrives on the last one going straight down, but with the curvature its via points ask for, so the path slows down at both junctions as into an arc. With `--set profile.path_tolerance_m=0.0001` the spline's ends are eased: near each junction the spline is bent so that its curvature there is that of the line (0), over a length l = sqrt(tolerance / (0.01728 dk)) for a step dk of the curvature, at most half the span, and only as long as the speed needs. The via points stay where they are; the spline moves by at most the tolerance. Here: 1.985 s within the axis limits and 1.530 s as fast as the motors allow. The eased part changes its curvature about nine times as fast as a clothoid of the same length would, so it is passed more slowly than a transition curve between a line and an arc (see the [model reference](model-reference.md#transition-curves-profilepath_tolerance_m)).

Other ways to give a path:

- `--trajectory FILE`: poses at times, CSV with the time in seconds and one position per axis per line. Lines that do not start with a number are skipped. The first time has to be 0 and the first pose where the setpoint stands, on each axis within the machine's setpoint slack (`Machine::setpoint_slack`: 0.1 mm on the gantries, 2.3 mm on the delta); times have to rise; every pose has to lie within the travel range, except near the start (see the limits below); the path ends at rest. The controller runs a cubic curve through the points.
- `sim-cli run MACHINE --path FILE` or `--trajectory FILE` writes all signals instead of the analysis.

Limits for this task:

- Without a path tolerance a rounded corner is slow with the default planner: 0.07 m/s into an arc of 0.1 m radius at 100 m/s³.
- Splines: without a path tolerance, where one joins a line or an arc the curvature steps, so the path passes that junction slowly, like a line into an arc. With one, the eased junction is passed at speed, but more slowly than a clothoid would allow for the same deviation. A span has one limit, from its tightest place, so a span with one tight bend is slow along its whole length.
- Transition curves between lines and arcs: none where two arcs meet in their direction in different planes (three axes), which the path passes slowly; a piece too short for its curves halves the curves of the whole path, not only its own. Their length follows the speeds the plan allows: the axis limits, or for `--time-optimal` the motors (the example takes 1.397 s with a tolerance of 0.1 mm, 1.695 s without).
- The time-optimal plan knows the belt's tension ratings (`ratings.belt_max_tension_n`, `belt_min_tension_n`): it takes `profile.tension_margin_n` (31 N on the default T-gantry, the guide friction the belt pulls) off both and keeps every section's tension in the rigid model within `profile.tension_reserve` (0.5) of the room to what is left, because the belt's vibration can add as much again as the rigid model's change of tension (see the [model reference](model-reference.md#as-fast-as-the-motors-allow---time-optimal)). It does not know the gearbox and pulley torque ratings, the guides or the DC link. Run `analyze` or `verdicts` on it.
- The time-optimal plan reads the motors' limits point by point (every 2 mm) and takes each where it is, but it is not the fastest one possible: a piece cruises at one speed, and on a curve the ramps get 0.4 of the jerk limit (see the [model reference](model-reference.md#as-fast-as-the-motors-allow---time-optimal)). Planning takes some milliseconds (1 to 21 ms on the shipped machines). On the full model the motors pass the rated speed by what the control loops add (0.1 % in the example).
- A machine that stands on an end of a stroke has its setpoint some hundredths of a millimetre beyond it: the setpoint is where the motors stand, under belt stretch and gear twist. A path from there starts that far outside the travel range and is run. Only near its start may a path leave the range: a pose within the machine's setpoint slack of the start (0.1 mm on the gantries, 2.3 mm on the delta) may lie that far outside it; any other pose has to lie within the range (on an axis whose start lies beyond an end, no further beyond than the start; rounding of 1e-9 m aside). Other poses are refused (`The path leaves the travel range. ...` for a path of pieces, `The trajectory needs at least two points inside the travel range, at rising times from 0.` for poses at times).
- The payload is as the machine has it when planning (attached, at the start).
- `verdicts`, `optimise`, `search` and `sensitivity` work on the cycle only, not on a path.
- In the app paths are under "Path" (see [Use the app](#use-the-app)); a PLC sends them over OPC UA (see [Connect a PLC over OPC UA](#connect-a-plc-over-opc-ua)).

## Check for obstacles and crashes

Put the obstacles of the work area into the config as boxes along the axes of the 3D view (see the [config reference](config-reference.md#top-level) for where each machine's tool is), and say how close the machine may come with `profile.min_clearance_m`:

```text
$ sim-cli run single-axis --move 0.9 --time 1.5 --out run.csv --set "effects.crash=true" \
    --set 'obstacles=[{name="wall",centre_m=[0.75,-0.03,0.06],size_m=[0.1,0.2,0.2]}]'
Planned path: least clearance to the obstacles -85.0 mm
Planned path: first contact with an obstacle at the pose 0.4700
Run: least clearance to the obstacles -0.0 mm
Run: first contact with an obstacle (name: time, speed of the tool, pose) wall: 0.4701 s, 1.0000 /s, 0.4700
```

- `run` says before the run what the move, path or cycle keeps from the obstacles as planned, and after it the run's least clearance and its first contact: when, how fast the tool went and where. Nothing is said without obstacles.
- `verdicts` says the same of the cycle as planned and judges the run with the check `collision`: its least clearance against `profile.min_clearance_m`, failing at any contact. Without obstacles the check is not made.
- `effects.crash` (off by default) makes the obstacles rigid: a part moving into one stops dead at its face, with the drives on or off (a machine coasting or falling with its drives off stops there too), and drives that were on go off. Enabled again, the drives can take the machine out of the obstacle; a move further into it stops it and takes them off again. A PLCopen axis reports such a stop as ErrorID 8. Off, contacts are reported and the machine moves through.
- `--time-optimal` refuses a path that comes nearer than the clearance; `optimise` takes no setting whose run does.
- The app draws the obstacles as translucent boxes with their edges, red while touched, the clearance to the nearest as a red line with its value in mm, and the first contact as a red mark on the timeline.
- The clearance is between boxes: the tool, carriage, Z axis, platform and payload as the 3D view draws them, not their CAD shapes. The delta's forearm rods are each ten boxes along the drawn rod, up to a piece's length (95 mm) wider than the rod where it slants; its upper arms are not checked.

## Tune gains and use input shaping

The gains are in `[drive]`: `kv_1_s` (position), `velocity_kp_nm_s_rad` and `velocity_tn_s` (velocity), `current_kp_v_a` and `current_tn_s` (current). The three commands below put a test signal on the position setpoint only, without feedforward, so they show the feedback loops.

**One line per axis:**

```text
$ sim-cli tuning t-gantry
axis,rise_time_s,overshoot,settling_time_s,bandwidth_hz
x,0.02875,0.0644,0.44750,10.649
z,0.04325,0.0000,0.08550,9.200

$ sim-cli tuning t-gantry --set drive.kv_1_s=150
axis,rise_time_s,overshoot,settling_time_s,bandwidth_hz
x,0.01150,0.3004,0.49875,22.161
z,0.00575,0.1603,0.04175,59.778
```

| Column | Unit | Meaning |
| --- | --- | --- |
| `rise_time_s` | s | Time from 10 % to 90 % of a 1 mm step |
| `overshoot` | fraction | Highest excursion beyond the step, as a fraction of it |
| `settling_time_s` | s | After this time the tool stays within 2 % of the step |
| `bandwidth_hz` | Hz | Where the closed-loop gain first falls below 1/√2. Empty if it never does up to 200 Hz |

With Kv = 50 1/s the position loop behaves like a first-order lag with time constant 1/Kv: bandwidth about Kv / 2π = 8 Hz. Z shows that plainly. In X the tool position counts the frame's sway: the step sets the lightly damped frame mode (about 21 Hz) ringing, which is the 6 % of overshoot and the 0.45 s until the tool stays within 2 %. Tripling Kv makes X overshoot by 30 %.

**Stability margins of the velocity loops.** `margins` measures each motor's velocity loop open: a sine torque is added to what the loop asks for, with the machine holding its start pose, and the loop's answer gives the open-loop response L at 60 frequencies from 10 Hz to 2 kHz (a drive's own measurement function works this way):

```text
$ sim-cli margins t-gantry
run,motor,crossover_hz,phase_margin_deg,gain_margin_db,peak_sensitivity,peak_frequency_hz,result
,a,188.1,56.8,9.05,1.897,277.3,ok
,b,182.1,54.0,8.95,1.895,277.3,ok

$ sim-cli margins single-axis --bands
run,motor,crossover_hz,phase_margin_deg,gain_margin_db,peak_sensitivity,peak_frequency_hz,result
,m,185.6,57.9,11.15,1.665,253.5,ok
belt.damping_n_s_m=50.0,m,190.6,52.1,11.18,1.717,231.7,ok
belt.damping_n_s_m=200.0,m,178.8,65.1,10.97,1.664,253.5,ok
...
```

| Column | Unit | Meaning |
| --- | --- | --- |
| `run` | | Empty for the config as it is; with `--bands` one row per motor for each value of `[bounds]` at its low and at its high bound |
| `crossover_hz` | Hz | Where \|L\| = 1, the crossover with the smallest phase margin |
| `phase_margin_deg` | ° | 180° plus the phase of L there (the phase followed without jumps, taken between −360° and 0° at the first crossover); negative: the loop is unstable |
| `gain_margin_db` | dB | By how much the gain may rise before L reaches −1, at the worst place where the phase falls through −180° that a later rise through −180° with \|L\| > 1 does not undo (a Nyquist count; a fall and rise beyond −1 below the crossover is conditionally stable and not counted); negative where \|L\| > 1 there (unstable); `inf` if no such place up to 2 kHz |
| `peak_sensitivity` | | The highest \|1 / (1 + L)\|: 1 over the shortest distance of L from −1. Up to 2 means at least 6 dB and 29° |
| `peak_frequency_hz` | Hz | Where it is highest |
| `result` | | `ok`, or `FAIL` where a margin is negative or the measurement is not valid: the loop ran into its torque limit or moved on its own (oscillated) during it. Then the loop is unstable and its margins are `NaN` |

The default configs' velocity gains were chosen with it: at least 35° of phase margin and 6 dB of gain margin in every run of `--bands`, also with `--set effects.motor_friction=false --set effects.guide_friction=false` (friction damps the loop near standstill, as the measurement holds the machine, but not in a cruise). Found: T-gantry 0.04 N m s/rad, at least 47.4° and 8.2 dB, 39.1° and 7.3 dB with friction off; single axis 0.025, at least 48.6° and 10.4 dB, 37.8° and 9.6 dB with friction off. The gain of 0.08 the configs had before left the T-gantry 12° and 2 dB (motor B hunted in its gearbox backlash in a cruise) and the single axis 1° and 0.3 dB (it went unstable with the belt at its stiffest bound). The EXCT-30 was tuned the same way, from 0.6 (34.9° and 6.6 dB, 29.6° and 5.8 dB with friction off) to 0.45: at least 47.3° and 9.1 dB, 38.6° and 8.3 dB with friction off (the figures for the gains in use as measured since 13 November, with the phase unwrapped; those for the earlier gains are from before).

**The step response as a trace:**

```text
$ sim-cli step t-gantry --axis-index 0
time_s,setpoint,response
0.000000,1.000000000e-3,0.000000000e0
0.000250,1.000000000e-3,-3.611875521e-10
0.000500,1.000000000e-3,2.716861028e-9
...
```

`--axis-index N` (0 is the first axis; `--axis` names a catalogue axis, as for every command), `--size M` (default 0.001 m), `--time SECONDS` (default 0.5). `response` is the tool position relative to where it stood, four samples per millisecond.

**The closed-loop frequency response:**

```text
$ sim-cli bode t-gantry --axis-index 0 --count 8
frequency_hz,gain,gain_db,phase_deg
1.0000,9.820613e-1,-0.157,-10.626
2.1317,9.730009e-1,-0.238,-19.226
4.5440,8.970231e-1,-0.944,-36.183
9.6863,7.278681e-1,-2.759,-62.980
20.6478,7.982035e-1,-1.958,160.711
...
```

`gain` is tool amplitude over setpoint amplitude. The phase turning past 160° at 20.6 Hz is the frame mode, which the tool position counts. Options: `--from HZ` (1), `--to HZ` (200), `--count N` (40), `--amplitude M` (0.0005 m). Frequencies have to be above zero here too.

**The mechanics without control:**

```text
$ sim-cli sweep single-axis --from 20 --to 200 --count 12
frequency_hz,gain_rad_s_per_nm,phase_deg
20.0000,5.522398e1,-82.358
...
131.5866,9.881837e1,-60.793
```

A sine torque on one motor on top of the holding torques, measured at that motor's speed. Belt resonances show as peaks; the load swinging against the held motor shows as a dip. Options: `--motor N` (index, 0 is the first), `--from` (5), `--to` (500), `--count` (120), `--amplitude NM` (1.5 N m, or 40 % of the drive's peak torque where that is less). Frequencies have to be above zero: `--from 0` is refused (`Configuration value is out of range: frequency`). With friction on, a small amplitude mostly measures stiction: at 0.05 N m neither default machine shows a dip. The default amplitude finds it on every motor of both default configs and of `exct-30.toml` (21.6 Hz on the T-gantry, 20.8 Hz on the EXCT-30, 84.3 Hz on the single axis). A larger one is no better: at 2 N m the single axis shows a second, false dip at 62 Hz. The rows may be in any order of frequency, and `--from` may be above `--to`.

**The notch filter:** `--set effects.notch_filter=true`, with `drive.notch_frequency_hz` and `drive.notch_bandwidth_hz`. It acts on the torque setpoint.

**Input shaping.** A ZVD shaper spreads every setpoint over three steps half a vibration period apart, so that the swings they start cancel:

```text
--set effects.input_shaping=true --set profile.shaper_frequency_hz=25 --set profile.shaper_damping=0.03
```

Set the frequency to the resonance of the tool. On the T-gantry that changes with the Z height, so `--set effects.shaper_follows_pose=true` lets the shaper take the frequency from the machine at the planned pose. The machine reports the lower mode of carriage-on-belt and Z-axis-on-guide (24.6 Hz at z = 0.05 m and 35.6 Hz at z = 0.15 m for the default config at x = 0.9 m). The single axis reports its carriage on the belt (65.8 Hz at x = 0.5 m). A second shaper (`profile.shaper2_frequency_hz` above 0) follows the machine's second mode as well: on the T-gantry carriage and Z axis swinging against each other, 104.2 Hz at z = 0.05 m.

**Resonances over the stroke**, to choose the shaper frequencies: `modes` asks the machine for its modes at `--points` places (21 by default) over the stroke of one axis (`--axis-index`, the first by default), the other axes at the start pose. A machine that reports none writes the column `position` alone. The app's tuning panel plots the same over the axis it measures, under the shaper frequencies configured.

```text
$ sim-cli modes t-gantry --axis-index 1 --points 3
position,mode1_hz,mode1_damping,mode2_hz,mode2_damping
0.000000,21.0375,0.03737,106.9100,0.05326
0.250000,47.7767,0.02417,208.0696,0.02559
0.500000,27.5888,0.02248,86.9524,0.06739
```

The Z axis is stiffest where its centre of mass hangs at the carriage (z = 0.3 m here), and both modes rise there.

What it does on the EXCT-30 cycle. The config has the shaper on, at a fixed 25 Hz with a second one at 17.5 Hz; the first run switches both off:

```text
$ sim-cli analyze t-gantry --config crates/machine-t-gantry/exct-30.toml --set effects.input_shaping=false
Thermal start: warm, at the rises the cycle settles at; largest change of a rise over the measured pass 0.000 K
motor,cycle_s,...,path_error_peak_m,...,dwell_error_peak_m
a,1.852000,...,6.168e-4,...,5.827e-5

$ sim-cli analyze t-gantry --config crates/machine-t-gantry/exct-30.toml
a,2.048000,...,5.378e-4,...,3.949e-5
```

With the shapers the tool is 0.039 mm off its target in the dwell instead of 0.058 mm, and the cycle takes 2.048 s instead of 1.852 s.

The tool swing after a traverse is the signal `axis.x.tool_deflection` of `run`.

**A second resonance.** `--set profile.shaper2_frequency_hz=62` adds a second ZVD shaper (damping `profile.shaper2_damping`), convolved with the first: nine copies of the plan instead of three, and each move ends both periods later. A shaper at f_s leaves cos²(π f / (2 f_s)) of an undamped mode at f: one at 25 Hz leaves 53 % of a mode at 62 Hz, and the two together leave nothing of either (the test `a_second_shaper_suppresses_a_second_resonance_by_the_zvd_formula`). 0, as in the two default configs, is no second shaper; the EXCT-30 has one at 17.5 Hz. It stays at its frequency when the first follows the pose. `optimise` tries it where a faster setting with the shaper still fails `settling` alone, at this frequency or, where it is 0, at the resonance the machine reports at the start pose if that is more than 10 % off the first shaper's.

**Fieldbus delay and jitter.** `--set effects.fieldbus_delay=true` makes the drives use the motor setpoints `profile.fieldbus_delay_cycles` profile cycles late. `--set effects.fieldbus_jitter=true` makes each frame take 0 to `profile.fieldbus_jitter_cycles` cycles more, drawn evenly and the same in every run (from `drive.noise_seed`); a late frame leaves the drives the one before. At 1 m/s on the single axis one cycle of jitter puts the drive's position setpoint 0 or 0.25 rad behind, half the time each: an RMS of 0.18 rad. It works with or without the delay.

Limits for this task:

- The step is 1 mm. With friction on it also shows stiction.
- The shaper handles two resonances at most (`profile.shaper2_frequency_hz`, below). The frame mode and the stiffness of the control loops are not in the frequency the machine reports.
- Each move's setpoint ends one vibration period later. Where the cycle dwells or the gripper acts, both wait for the shaped setpoint, so the cycle above is 82 ms longer: two dwells, 41 ms each. Segments without a dwell and without a gripper action go on with the plan.
- Most of the path error on the EXCT-30 is the frame giving way under the acceleration forces (0.62 mm peak without the shaper, 0.54 mm with it), which a shaper set to the tool's resonance does little about.
- No real drive has been compared. Drive vendors do not document their internal filters, and the delays here are whole cycles. The fieldbus can have jitter (below), the drives' own loops have none.
- The margins are measured with a tenth of the rated torque. Near standstill the motor's Coulomb friction (smoothed over 0.5 rad/s) damps the loop at that amplitude, so the margins are larger with friction on; take the ones with friction off for a cruise.
- A gain that is fine at the nominal values can be unstable when rotor inertia, belt stiffness or inductance are off. That was found on the EXCT-30 config, whose velocity gain was lowered from 1.4 to 0.6 for it (and later to 0.45 for the margins above). Run `sensitivity` after tuning.

## Replay a drive trace and fit parameters

**What a trace is.** A CSV file with a header line. Needed columns, at the motor shaft:

| Column | Unit |
| --- | --- |
| `time` | s |
| `motor.<id>.angle_setpoint` | rad |
| `motor.<id>.angle` | rad |
| `motor.<id>.speed` | rad/s |
| `motor.<id>.torque` | N m |

`<id>` is `a` and `b` on the T-gantry and `m` on the single axis. A unit in brackets after a column name is ignored (`time[s]`), and so are other columns. So a file written by `sim-cli run` is a trace. One row per profile cycle (1 ms) reproduces the position setpoints exactly. At least four rows; a shorter trace is refused.

**Replay.** The position setpoints of the trace are turned into tool poses and run through the simulator:

```text
$ sim-cli run single-axis --cycle --time 2.5 --set effects.payload=false --set mass.carriage_kg=5.5 --out measured.csv
$ sim-cli replay single-axis --trace measured.csv --set effects.payload=false --out overlay.csv
motor,torque_rms_nm,torque_peak_nm,following_error_rms_rad,following_error_peak_rad
m,2.262867e-2,6.527515e-2,6.255957e-4,4.218134e-3
```

Here the "measured" machine is the simulator with a carriage 1.5 kg heavier than the config. The table gives, per motor, the RMS and the peak of the difference between measured and simulated torque and following error. `--out` writes both side by side, one line per row of the trace:

```text
time,motor.m.torque_measured,motor.m.torque_simulated,motor.m.speed_measured,motor.m.speed_simulated,motor.m.following_error_measured,motor.m.following_error_simulated
```

**Fit, stage 1: rigid values from a trace.**

```text
$ sim-cli fit single-axis --trace measured.csv --set effects.payload=false
quantity,of,config,fitted,standard_error
moved_mass_kg,x,6.549800e0,8.067588e0,1.618e-2
viscous_friction_n_s_m,x,2.681385e1,2.621072e1,2.192e-1
coulomb_friction_n,x,2.132336e1,2.196390e1,1.953e-1
constant_force_n,x,-7.757048e-4,2.050984e-3,6.293e-2
Moved mass, axis x: +1.518 kg ± 0.016. Add this to the masses that move with the axis. The trace cannot tell load mass from rotor and pulley inertia.
Viscous friction, axis x: × 0.978 ± 0.008. Scale the viscous friction of the axis's guide and of the motor shafts by this factor; the trace cannot tell the two apart.
Coulomb friction, axis x: × 1.030 ± 0.009. Scale the Coulomb friction and stiction of the axis's guide and of the motor shafts by this factor; the trace cannot tell the two apart.
Constant force, axis x: +0.003 N ± 0.063. The measured axis needs this much more than the config gives: a weight, an incline or a cable pull the config lacks.
```

Per axis, a least-squares fit of force = mass × acceleration + viscous × speed + Coulomb × sign(speed) + constant. The same estimate is run on the trace (`fitted`) and on the config's own simulation of the same setpoints (`config`). The difference between the two is what the config is off by: here +1.518 kg, for a true 1.5 kg. The advice lines go to the error stream. No config is edited.

**Fit, stage 1 in the motors' coordinates: the delta, the SCARA, the arm.** Where the motors do not turn in proportion to the axes (the delta's arms), or the inertia a motor turns changes with the pose (a SCARA's or an arm's elbow; `trace::per_axis_fit` decides), `fit --trace` regresses the motor torques on the machine's own rigid model in regressor form, τ = Y(q, q̇, q̈) θ: per motor k, τ_k − τ_k,rigid = (θ_payload − 1) × (the payload's share of τ_k,rigid) + ΔJ_k q̈_k + b_k q̇_k + c_k sign(q̇_k) + τ0_k, the rigid torques from `Machine::rigid_motor_torque` at the trace's angles, speeds and accelerations (the acceleration from the angle, which an encoder measures far more precisely than the speed it differentiates). A trace that swings every axis at several frequencies (here a file of poses at times: the delta 0.2 m across at 0.6 and 0.9 Hz, 0.06 m up and down at 1.1 Hz, 8 s), of a delta with 4 kg of payload for the config's 3 and twice the joints' friction:

```text
$ sim-cli run delta-d3 --trajectory swing.csv --time 8 --out swing-trace.csv --set mass.payload_kg=4 --set joint.friction_coulomb_nm=0.6 --set joint.friction_viscous_nm_s_rad=0.1
$ sim-cli fit delta-d3 --trace swing-trace.csv
quantity,of,config,fitted,standard_error
payload_factor,payload,9.901197e-1,1.318199e0,1.356e-2
added_inertia_kg_m2,1,3.957651e-7,5.519854e-6,1.260e-5
viscous_friction_nm_s_rad,1,2.581219e-4,3.313259e-4,1.247e-4
coulomb_friction_nm,1,9.511667e-2,1.156876e-1,2.657e-3
constant_torque_nm,1,-6.177851e-3,-8.611906e-3,4.467e-3
...
Payload: × 1.331 ± 0.014. Scale the payload's mass by this factor.
Coulomb friction at motor 1: +0.0206 N m ± 0.0027. Add this, at the motor shaft, to the Coulomb friction of the motor and its joint; the trace cannot tell it from gearbox losses.
...
```

The payload's factor is 1.318 / 0.990 = 1.331 for a true 4 / 3; the joints' Coulomb friction, doubled from 0.3 to 0.6 N m at the joint, is +0.015 N m at the motor behind the gearbox of 20 (0.3 / 20 / 0.96), and comes out 0.019 to 0.021 ± 0.003, the heavier payload loading the joints a little more. Pairs of values whose estimates correlate by more than 0.95 are named on the error stream (`Not told apart by this trace: ...`): only their sum is known. A factor on the rest of the rigid model (arms, platform) is not fitted: on the delta it was one with the motors' added inertias and constant torques. A rotor and its arm's own inertia about the joint are one added inertia. On strain wave gears (the SCARA's, efficiency 0.75) the losses grow with a heavier payload's torque and show as Coulomb friction on the joint that carries it.

**Fit, stage 2: belt stiffness from a frequency sweep.**

```text
$ sim-cli sweep single-axis --from 50 --to 130 --count 30 --amplitude 1.5 --set belt.stiffness_ea_n=3.5e5 --out sweep.csv
$ sim-cli fit single-axis --sweep sweep.csv --amplitude 1.5
quantity,of,config,fitted,standard_error
resonance_hz,m,8.452246e1,7.242377e1,1.285e0
resonance_damping,m,7.932269e-2,7.912026e-2,
Stiffness between motor and load: × 0.734 ± 0.026. Scale the belt stiffness by this factor. It holds only once the masses are right, so correct those first and sweep again.
```

The sweep file needs the columns `frequency_hz` and `gain_rad_s_per_nm` (motor torque to motor speed). `fit` runs the same sweep on the config, at the frequencies of the file, on motor `--sweep-motor N` (default 0) with `--amplitude NM` (default as for `sweep`), and compares where the dip is. Where the response has more than one dip (the T-gantry has three, near 22, 62 and 107 Hz), the one lowest in frequency is compared, in both. The "measured" belt here has 0.7 times the stiffness; the fit says 0.734 ± 0.026.

**Fit, stage 2 without a sweep: belt stiffness from the moves.**

```text
$ sim-cli run single-axis --cycle --time 3 --out cycle.csv --set belt.stiffness_ea_n=3.5e5 --set effects.payload=false
$ sim-cli fit single-axis --trace cycle.csv --stiffness --set effects.payload=false
quantity,of,config,fitted,standard_error
...
stiffness_factor,belt.stiffness_ea_n,1.000000e0,6.937900e-1,2.405e-3
ringing_rms_nm,belt.stiffness_ea_n,,6.790983e-3,
ringing_left_rms_nm,belt.stiffness_ea_n,,8.385708e-5,
...
Stiffness between motor and load, from the moves: × 0.694 ± 0.002. ...
```

`--stiffness` replays the trace with the belts' stiffness (`belt.stiffness_ea_n`, or the key `--stiffness-key` names) times a factor and takes the factor whose motor torques ring as the trace's do: torques less their mean over ±10 ms, least squares over all motors and rows; nine replays from 0.25 to 4, then parabolas through seven replays twice (24 in all, about 1 s on the single axis). The replay has the config's loops, filters and friction, which damp the ringing as they do on the machine. The standard error comes from the curvature of the sum of squares and from how far the replays scatter about the parabola. The rows `ringing_rms_nm` and `ringing_left_rms_nm` are the ringing in the trace and what the fitted replay leaves of it. A trace whose moves do not ring the belt enough is refused, exit code 2: `No stiffness from this trace: its moves do not ring the load on its springs enough against the noise ...`, where the standard error is above a tenth of the factor or the best factor lies at an end of 0.25 to 4. On the single axis with noise a move of 2 mm at 1 m/s³ and 0.2 m/s² is refused; two moves of 0.8 m at the config's limits give 0.708 ± 0.015 for 0.7.

What the fit can and cannot recover:

| Recovered | Not recovered, or not told apart |
| --- | --- |
| Moved mass per axis (rotors and pulleys counted as mass at the axis) | Load mass from rotor and pulley inertia |
| Viscous friction per axis | Guide friction from motor shaft friction |
| Coulomb friction per axis | The belt's damping from friction (the damping of the dip includes friction and the gearbox) |
| Constant force per axis (weight, incline, cable pull) | A rotor from its arm or link on a delta, SCARA or arm: one added inertia per motor |
| One factor on the stiffness between motor and load, from a sweep or from the moves of a trace (`--stiffness`) | Which spring between motor and load is off: the factor counts all of them |
| On a delta, SCARA or arm: the payload's mass as a factor, per motor added inertia, viscous and Coulomb friction and constant torque at the shaft | A factor on the arms' and platform's masses (it is one with the motors' inertias and constant torques); friction from gearbox losses |
| | Drive delays and filters: per motor with `sim-cli identify` (below), not per axis |
| | Backlash, tooth stiffness, frame and Z-axis stiffness |

Limits for this task:

- **The gripper.** A trace does not say when a payload comes and goes, so a cycle trace is replayed with the payload as the config starts it. A cycle trace of a machine that grips, fitted against a config with the payload on, comes out wrong: the single axis's own default cycle fits as −0.837 kg. Record without payload, or switch it off as above.
- `start.pose` of the config has to be where the trace starts.
- An axis that the trace does not move in both directions is not fitted (`Axis x: not fitted. The trace has to move it in both directions.`). One direction cannot tell Coulomb friction from a constant force.
- A wrong friction shows as a few percent of wrong mass and the other way round, because of gearbox losses. Change the config and fit again.
- The stiffness factor holds only once the masses are right.
- A sweep with a small amplitude finds no dip when friction is on. A sweep at 0.05 N m gives `No stiffness fitted: the measured sweep or the config's own shows no dip with both flanks inside the swept range.` That is a failure like any other: `fit` ends with exit code 2, after writing what a trace gave. The sweep also has to contain the dip and both its flanks. Its frequencies have to be above zero.
- All of this was tested on traces the simulator made, with added noise. No trace of a real drive has been read.

**Vendor traces.** `replay`, `fit` and `identify` take `--map FILE` beside `--trace FILE`: a mapping that says which signals of an export of a SINAMICS (Startdrive), TwinCAT Scope or CODESYS trace, or of any CSV, are the position setpoint, position, speed, torque and torque setpoint, and how to convert their units (LU, increments, mm, 1/min, %, ‰) to rad, rad/s and N m at the motor shaft. A template is in [datasets/template/map.toml](../datasets/template/map.toml).

**Identification without a config.** `sim-cli identify --trace FILE --map FILE` fits one motor's drive train from its own trace: total inertia, viscous, Coulomb and (where the trace shows it) Stribeck friction, the LuGre presliding stiffness, a constant torque, and the lag and dead time from torque setpoint to torque and from position setpoint to position. With `--from HZ --to HZ` (a trace with a noise excitation), `--sweep FILE` or `--step-from S` (a torque step) also stiffness and damping between motor and load as a two-mass model. Every value with its standard error and 95 % interval, and how well each model reproduces the trace. Which test moves to record, at which sample rate, and how to read the result: [Measuring a drive train](measurement-procedure.md). Measured datasets are published in [datasets/](../datasets/README.md) and checked with `sim-cli check-dataset DIR`.

## See where torque and energy go

Six views take one effect apart instead of leaving it in the torque trace: where each motor's torque goes, where the energy of a cycle goes, what each guide's friction does, how warm windings, gearboxes and guides get, what each belt section carries, and what one effect changes when it is switched off. How each figure is computed is in the [model reference](model-reference.md#effect-views).

**Torque breakdown.** `--breakdown FILE` writes, per motor, the peak and the RMS over the analysed pass of the four parts of its torque, and of the torque itself:

```text
$ cargo run -q --release -p sim-cli -- analyze t-gantry --breakdown breakdown.csv
$ cat breakdown.csv
motor,part,peak_torque_nm,rms_torque_nm
a,inertia,2.955808e-1,6.164693e-2
a,gravity,1.372931e-1,1.193030e-1
a,friction,1.403399e-1,8.273541e-2
a,rest,6.346283e-2,1.184697e-2
a,torque,5.595012e-1,1.653591e-1
b,...
```

- **inertia**: what accelerates every moved mass, and the rotors and pulleys turning with the load.
- **gravity**: what holds the load against its weight. On the T-gantry each motor holds m_z g r / (2 i) = 0.137 N m, one up, one down.
- **friction**: guides, motor shafts and brakes, gearbox losses, idlers, cable chain.
- **rest**: everything else, mostly the belt and gearbox: the load lagging behind the motors on the belt, ringing, backlash. Also ripple and cogging, tooth meshing, the process force and the end stops.

At every moment the four parts add up to the motor torque. Their peaks and RMS values do not: they come at different times. A rest that is large against the torque means the belt or the gearbox is doing a lot of the work; on the rigid model it is nearly zero.

**Energy budget.** `--budget FILE` writes where the energy drawn from the supply over the pass went, in joules and as a share of the supply:

```text
$ cargo run -q --release -p sim-cli -- analyze t-gantry --budget budget.csv
$ cat budget.csv
item,energy_j,share
supply,1.393748e2,1.000000
copper.a,3.302749e0,0.023697
copper.b,2.839158e0,0.020371
resistor,5.342595e0,0.038333
discharge,0.000000e0,0.000000
power.guide_x,5.101324e1,0.366015
power.guide_z,1.911054e1,0.137116
...
power.shaft_a,2.454226e1,0.176088
power.shaft_b,2.454228e1,0.176088
power.gearbox_a,4.378016e0,0.031412
power.gearbox_b,4.305561e0,0.030892
power.belt,2.861317e-3,0.000021
...
windings,5.249505e-7,0.000000
capacitors,-4.519116e-4,-0.000003
energy.kinetic,3.709163e-8,0.000000
energy.potential,-6.817267e-8,-0.000000
energy.elastic,-2.175817e-7,-0.000000
payload,-2.304591e-4,-0.000002
unaccounted,-8.666425e-4,-0.000006
```

| Item | What |
| --- | --- |
| `supply` | Energy from the supply. With `effects.dc_link` what the rectifier (or the line module) drew from the line into the DC link; without, what the drives drew less what they fed back |
| `copper.<motor>` | Copper loss in each winding |
| `resistor` | The braking resistor (with `effects.dc_link`) |
| `discharge` | With `effects.dc_link`: what the DC links of drives switched off lost through their bleeders (`ratings.dc_link_discharge_time_constant_s`), which the rectifier drew back; 0 while the drives are on |
| `mains`, `line_module` | With `effects.regenerative_supply`: what the line module fed back into the mains, and what it lost doing so |
| `power.<place>` | Each loss of the machine: guide friction per guide, idlers, cable chain, shaft friction and brake per motor, gearbox per motor, belt damping, frame and Z guide damping, end stops, and `power.other` for ripple, cogging, tooth meshing and the process force (these can give energy as well as take it) |
| `windings`, `capacitors` | What the windings' inductance and the DC link capacitors hold more at the end than at the start |
| `energy.kinetic`, `.potential`, `.elastic` | What the machine holds more at the end of the pass than at the start; near zero for a repeating cycle |
| `payload` | Energy that left with the payload put down, less what came with the one picked up |
| `crash` | With `effects.crash`: what the stops at obstacles took out of the machine, lost in the impact |
| `unaccounted` | What the rows leave of the supply. Below 0.1 % of it on every config, the loss-free ones included. The electrical side alone (supply against copper, resistor, mains, line module, windings, capacitors and the work of the torques on the shafts) is checked to a millionth of the supply: the winding current is integrated together with the mechanics |

On the default T-gantry the guides take half the supply, the motor shafts a third, gearboxes, windings and the braking resistor a few percent each; the belt's damping takes almost nothing.

**Friction per guide.** The app draws, per guide, the friction force of steady sliding over the speed (Coulomb level, viscous part and the Stribeck rise towards standstill) at the guide load of the moment, and the point where the guide is now. A point off the curve is a guide that is not sliding steadily: breaking away, reversing, or held. The command line has no such view; `sim-cli run` writes the same signals (`axis.<id>.friction_speed`, `_coulomb`, `_viscous`, `_stiction`, `_load_scale`).

The torque parts, the loss powers and the energy parts are signals too, so `sim-cli run` writes them, the scopes of the app can show them and a PLC can read them.

**Temperatures.** The app draws the temperature rise of each motor winding over the scopes' window and, with the effect "Gearbox and guide heating" (`effects.part_heating`, off by default) on, of each gearbox and guide. Each of these warms with its own loss (`power.gearbox_*`, `power.guide_*`) through a thermal resistance, with a time constant and up to a rated rise, all three in `[thermal]` of the config. The table gives each one's rise now, where it settles when the analysed cycle repeats for long (for a part: thermal resistance times its mean loss), its rated rise and the share of the rating where it settles, marked "(above the rating)" over 100 %. The share is that of the settled rise, because the rise now is a few percent of it for many minutes; the checks `gearbox-thermal` and `guide-thermal` judge the same settled rise. On a direct drive (EXCT-30: gearbox ratio 1) there is no gearbox to warm; its `power.gearbox_*` is the coupling's damping and is labelled so. In the 3D view the motor housings, gearboxes, the carriage and (T-gantry) the Z axis take the colour of their rise against their rated rise: blue cold, yellow at half, red at the rating, magenta above it; the view draws this scale. With the default time constants of 20 to 40 minutes, a minute of the scopes shows a slow climb; to watch a part settle, shorten its time constant. With the effect "Temperatures act on friction, gearbox losses and belt stiffness" (`effects.thermal_feedback`, off by default) the parts warm the same way and feed back: a guide's viscous friction follows the viscosity of its lubricant (0.28 of it at 30 K above 20 °C), a gearbox loses less as its oil thins (a 95 % gearbox has 96.8 % at 30 K, with half its loss taken as viscous), and the belt's stiffness follows the ambient temperature, a little. The values are in `[thermal_feedback]`: the ambient temperature, the reference temperature the config's friction and efficiency hold at, the lubricant's viscosity slope, the viscous share of the gearbox's loss and the belt's loss of stiffness per kelvin. It replaces `effects.temperature`, whose two hand-set factors are not used while it is on. Over one cycle of seconds the parts barely warm with the default time constants of 20 to 40 minutes, so the cycle's figures move only where the ambient differs from the reference; to see a part settle where its loss and its heat flow balance, shorten its time constant. The command line writes the rises with `run` (signals `motor.<id>.winding_temperature`, `temperature.<part>`).

**Belt teeth and spans.** With `effects.belt_tooth_jump` (off by default) the belt's teeth climb out of a motor pulley's grooves when the pull there exceeds what the slack side's tension holds them in with (`belt.tooth_jump_factor` times the teeth in mesh times pitch over radius, times the slack side's tension: where both sides share the pull evenly, about 1.5 times the pretension with the defaults), and the belt slides one tooth on, or back. The signals `pulley.<motor>.tooth_jumps` count the jumps and `pulley.<motor>.tooth_slip` is how far the belt has slipped over the pulley; `power.tooth_jump` is what the jumps take, in the energy budget. Any jump in the cycle fails `belt-tooth-jump`. With `effects.belt_span_vibration` (off by default) each free span of the belt has its first sideways frequency, sqrt(tension / mass per metre) / (2 · length), as a signal `belt.span.<i>.frequency`, and `belt.span_detuning` says how far the closest of them is from what the drive excites: the tooth mesh (belt speed over pitch) and the motors' ripple and cogging. `belt-span-resonance` fails where a span runs within `ratings.belt_span_resonance_band` (10 %) of an excitation while the axes move at a steady speed. Only the frequencies are computed, not how far a span swings.

**Belt sections.** Per section the tension (the pretension dashed) and the stretch in mm over the scopes' window, and a table of both now with the tension allowed (`ratings.belt_max_tension_n`) and the share of it. The rating is not drawn: at five times the pretension it would flatten the lines. The stretch is the section's length less its unstretched belt, so a section at pretension is stretched too (0.18 mm on 0.3 m at 300 N and EA = 5·10⁵ N). The 3D view colours each section by the same tension: the belt's own dark grey at pretension, towards red above it (red at twice the pretension), towards blue below it (blue when slack); the view draws this scale with its numbers. `sim-cli run` writes the signals `belt.<i>.tension` and `belt.<i>.stretch`.

**An effect on and off.** `compare --effect NAME` runs the analysed pass of the cycle twice, with `effects.NAME` on and off, whatever the config has it at, and writes the chosen signals side by side, once per millisecond; `--differences FILE` writes the peak and the RMS of on less off per signal:

```text
$ cargo run -q --release -p sim-cli -- compare t-gantry --effect guide_friction --signal motor.a.torque --signal motor.b.torque --out compare.csv --differences diff.csv
$ head -3 compare.csv
time[s],motor.a.torque.on[N m],motor.a.torque.off[N m],motor.b.torque.on[N m],motor.b.torque.off[N m]
0.0000,5.644070522e-2,1.035051415e-1,-1.449650751e-1,-1.152884059e-1
0.0010,5.731809256e-2,1.039281616e-1,-1.441915809e-1,-1.148902590e-1
$ cat diff.csv
signal,unit,peak_difference,rms_difference
motor.a.torque,N m,9.626565e-2,4.599827e-2
motor.b.torque,N m,9.226389e-2,4.627935e-2
```

Without `--signal` every signal is written. In the app, the part "Effect on and off" of the effect views picks the effect and one quantity of the scopes (torque, speed, following error, ...), runs the comparison in the background on the config as it is, and overlays both runs: on solid, off dashed in the same colour, one colour per axis or motor, with a table of the difference. An effect that changes the cycle's timing (corner blending, input shaping) is compared only over the time both passes have.

## Record raw signals, switch the drives off

```text
$ sim-cli run single-axis --cycle --time 2.5 --out trace.csv
```

Writes every signal as CSV, one column per signal with its unit in the header (`axis.x.position[m]`, `motor.m.torque[N m]`, ...).

| Option | Effect |
| --- | --- |
| `--move a,b` | A point-to-point move to this pose, all axes synchronised |
| `--cycle` | Run the configured cycle |
| `--trajectory FILE`, `--path FILE [--time-optimal]` | See [Run a path](#run-a-path-and-read-path-error) |
| none of these | The machine stands at its start pose |
| `--time SECONDS` | Simulated time. Default 2 |
| `--every STEPS` | Write every N-th base step. Default 16, which is one row per millisecond |
| `--disable-at SECONDS` | Switch the drives off at that time |

The signals:

| Signal | Unit | Meaning |
| --- | --- | --- |
| `time` | s | Simulated time |
| `payload` | 0 or 1 | Whether the payload is attached |
| `axis.<id>.position`, `.velocity` | m, m/s | Tool position and speed (on the T-gantry in X: of the tool tip over the floor, which is the carriage on the beam plus `frame.deflection` plus `axis.x.tool_deflection`) |
| `axis.<id>.position_setpoint`, `.velocity_setpoint`, `.acceleration_setpoint` | m, m/s, m/s² | The controller's setpoints, after input shaping and before the fieldbus delay |
| `axis.<id>.following_error` | m | Motor following errors transformed to the tool axes |
| `motor.<id>.angle`, `.speed` | rad, rad/s | Motor shaft, before the gearbox |
| `motor.<id>.torque`, `.torque_command` | N m | Torque from the winding current, and the velocity loop's setpoint for it after the torque limit |
| `motor.<id>.current`, `.voltage`, `.electrical_power` | A, V, W | Winding |
| `motor.<id>.winding_temperature` | K | Temperature rise of the winding |
| `motor.<id>.dc_link_voltage`, `.braking_resistor_power` | V, W | DC link. With `effects.shared_dc_link` the same for every motor |
| `motor.<id>.speed_command`, `.following_error`, `.angle_setpoint` | rad/s, rad, rad | Position loop |
| `gearbox.<motor id>.output_torque` | N m | Torque at the gearbox output, on the pulley. What the checks `pulley-torque`, `gearbox-peak` and `gearbox-continuous` judge |
| `belt.<n>.tension` | N | Tension of belt section n |
| `axis.<id>.belt_stretch` | m | Tool position minus what the pulley angles say |
| `axis.<id>.friction` | N | Guide friction force |
| `belt.slack_time` | s | Seconds any belt section has spent without tension, counted up |
| `pulley.<id>.radial_load` | N | Pull of the belt on the pulley shaft |
| `axis.<id>.guide_load` | N | Load on the guide |
| `frame.deflection` | m | Frame deflection in X (T-gantry) |
| `axis.x.tool_deflection` | m | Tool tip against the carriage in X (T-gantry) |

**Drives off.** With `--disable-at` the drives give no torque from that time. The motor holding brakes engage (`effects.holding_brake`, on both machines, in the elastic and in the rigid model):

```text
$ sim-cli run t-gantry --move 0.5,0.2 --time 1.5 --disable-at 0.5 --out disable.csv
```

In that run the Z axis is at 0.2276 m at 0.498 s, moving, and rests at 0.2192 m at 1.498 s: a brake that engages at speed slips with its torque (`motor.brake_torque_nm`) and no more until the axis stands. With `--set effects.holding_brake=false` it falls to the lower hard stop (z = −0.00008 m at 1.498 s). A move or cycle under way is dropped, not resumed. While the drives are off their windings cool (with `effects.winding_heating`, by the factor e every `motor.thermal_time_constant_s`), and a DC link above the bus level, after a trip, discharges through its bleeder with `ratings.dc_link_discharge_time_constant_s` (130 s, an estimate) down to the bus level.

## Connect a PLC over OPC UA

```sh
cargo run --release -p sim-opcua -- t-gantry --port 4840 --lang en
```

This prints `OPC UA server listening at opc.tcp://127.0.0.1:4840/` and runs the machine in real time until stopped. The machine name, `--port` (default 4840), `--lang`, `--host` (default 127.0.0.1), `--cycle-ms` (default 10) and the security options `--pki`, `--trust-all` and `--user` are the only arguments. A machine the simulator does not have, a port, language, address or cycle it cannot read and any other word are refused with a message and exit code 2. PLCopen function blocks per axis, the real-time pacing and its measurement, TwinCAT and CODESYS examples and the soak test are in [PLC virtual commissioning](plc.md).

| Property | Value |
| --- | --- |
| Endpoint | `opc.tcp://127.0.0.1:PORT/` |
| Security | None, and Basic256Sha256 with Sign and with SignAndEncrypt; anonymous unless `--user` (see [Security](plc.md#security)) |
| Namespace | `urn:kinematics-simulation` |
| Node ids | Strings in that namespace, as in the table below |
| Update rate | Every signal is published 100 times a second |

| Node | Type | Access | Meaning |
| --- | --- | --- | --- |
| `signal.<signal id>` | Double | read | Every signal of the [signal table](#record-raw-signals-switch-the-drives-off), e.g. `signal.axis.x.position`, `signal.motor.a.torque` |
| `command.target.<axis id>` | Double | write | Target of the next point-to-point move, e.g. `command.target.x`. Reads the planned position until it is written |
| `command.move` | Boolean | write | `true` starts the move to the targets |
| `command.setpoint.<axis id>` | Double | write | Streamed position setpoint. The simulator interpolates towards it over `profile.setpoint_interval_s`. Reads the setpoint the controller follows |
| `command.cycle` | Boolean | write | `true` starts the cycle, `false` stops it after the segment under way |
| `command.disable` | Boolean | write | `true` switches the drives off; brakes engage |
| `command.enable` | Boolean | write | `true` switches the drives on again |
| `command.path` | String | write | A path in the text of a path file (see [Run a path](#run-a-path-and-read-path-error)), one piece per line. Reads back what was written |
| `command.path_time_optimal` | Boolean | write | `true` plans the path as fast as the motors allow (as `--time-optimal`), `false` within the axis limits (the default) |
| `command.run_path` | Boolean | write | `true` plans `command.path` from where the machine stands and runs it |
| `status.busy` | Boolean | read | A move, a path or the cycle is under way |
| `status.refused` | String | read | Key of the last refused command, or empty |

How commands behave:

- The servo loops stay inside the simulator. The PLC sends targets or position setpoints, not torque.
- Numbers may be written as Double, Float, Int16, Int32 or Int64. Anything else is answered with `BadTypeMismatch`.
- Setpoints of several axes written within the same 10 ms are taken together as one setpoint.
- An axis that is not written stays where it is. `command.target.<axis id>` reads the planned position until it is written, and again once the move it was written for has been commanded. `command.setpoint.<axis id>` reads the setpoint the controller follows, which is the planned position whenever no setpoint is on its way. So after a move or a cycle, a write to one axis does not send the others back to where they were before.
- A path is written first and then started: write `command.path` (and `command.path_time_optimal`) together with `command.run_path`, or before it. Written together, `command.path` and `command.path_time_optimal` count in whichever order they come in the write: the path run is the one written last. The commands of one write are carried out in the order written, and the path is planned when its turn comes, from where the machine stands then (after a `command.enable` written before it, for example), exactly as `sim-cli run --path` plans it, and runs in real time.
- `status.refused` holds a key such as `error-busy`, `error-target-out-of-range`, `error-setpoint-too-far` or `error-disabled`. For a path: `opcua-error-path-line` (a line that cannot be read), `error-path` (e.g. an arc on a machine with one axis), `error-path-first-arc`, `error-path-range` (outside the stroke), `error-path-torque` (time-optimal only), `error-busy`, `error-disabled`. It stays until a command of the same kind (move, path, setpoint, cycle start, stop, disable, enable) is accepted. Accepted commands of another kind, such as a stream of setpoints, leave it.
- A streamed setpoint is refused while a move or the cycle runs.
- A setpoint further from the last one (or, with none on its way, from where the axis stands) than the axis travels at its speed limit in one `profile.setpoint_interval_s` (8 mm for the T-gantry's X axis) is refused with `error-setpoint-too-far` and not carried out. A PLC that has fallen further behind catches up with a move. A stream of one setpoint per interval is followed at its own speed, up to the speed limit; a setpoint that comes early finds the axis short of the last one and is followed that much faster.
- If the host cannot keep up with real time, the simulation falls behind instead of piling up work.

Limits for this task:

- **Not tested against a real PLC.** The client in the tests is written with the same library as the server. Interoperability with Siemens, Beckhoff or CODESYS clients, their data types, security settings and cycle jitter are untested.
- The server binds to one address (127.0.0.1 unless `--host` names another) and serves the machine's default config only. There is no `--config` and no `--set`.
- No security profiles, no user accounts, no methods, no config changes.
- Paths: `status.refused` names the key, not the line that could not be read, and there is no `--trajectory` (poses at times) over OPC UA. A path is planned inside the simulation's 10 ms tick: a long time-optimal path that takes longer to plan makes the simulation fall behind real time for that moment.
- The server writes its keys into a folder `pki` in the working directory.

## Use the model in Simulink / Modelica via FMI

```sh
cargo build --release -p sim-fmi                                   # the FMU's binary, target/release/sim_fmi.dll
sim-cli fmu t-gantry --config crates/machine-t-gantry/exct-30.toml --out exct30.fmu
```

`fmu` writes the machine with its config (the default, or `--config`, `--motor`, `--supply`, `--gearbox` and `--set` as for every command) as an FMI 2.0 Co-Simulation FMU. The FMU runs the same simulation as `sim-cli run`: drives, controller, every switched-on effect. Import it in Simulink (FMU block), OpenModelica (`File > Import > FMU`, or `importFMU`), Dymola (`File > Import > FMU`) or any other FMI 2.0 tool; a controller or PLC model there sends the position setpoints and reads back the signals. FMI 2.0 is the default because every one of those tools imports 2.0 Co-Simulation; `--fmi 3` writes an FMI 3.0 Co-Simulation FMU instead, with the same variables and the binaries under `binaries/<architecture>-<system>` (for example `binaries/x86_64-windows`). [FMI export](fmi.md) describes both versions, the FMU state, the derivatives, why there is no Model Exchange, how the FMUs were validated and how to import them.

What is in the FMU:

| File | Content |
| --- | --- |
| `modelDescription.xml` | The variables below, generated from the machine's signal list and config, with units and descriptions in the language of `--lang` |
| `resources/config.toml` | The config as given on the command line, catalogue parts filled in |
| `resources/model.txt` | Machine id and language |
| `binaries/win64/sim_fmi.dll` | The simulator. One binary for every machine; the resources say which machine it runs |

By default `fmu` takes the binary built next to `sim-cli` (`target/release/sim_fmi.dll`, on Linux `libsim_fmi.so`); without it, it stops and says how to build it. `--binary FILE` names it instead and may repeat: a `.dll` goes to `binaries/win64`, a `.so` to `binaries/linux64`, a `.dylib` to `binaries/darwin64` (the extension in any case: `SIM_FMI.DLL` is stored as `sim_fmi.dll`); two for the same platform are refused. An FMU for Windows and Linux is made by building `sim-fmi` on Linux (`cargo build --release -p sim-fmi`) and passing both files: `--binary sim_fmi.dll --binary libsim_fmi.so`. A machine file (`--machine-file`) goes into the FMU as `resources/machine.toml`, and the FMU builds its machine from it.

| Variable | Type | Causality | Meaning |
| --- | --- | --- | --- |
| `time` | Real | independent | Simulated time, s, from the start time of the experiment |
| every other signal of the [signal table](#record-raw-signals-switch-the-drives-off) | Real | output | E.g. `axis.x.position`, `motor.a.torque`, `belt.1.tension`, with its unit |
| `command.setpoint.<axis id>` | Real | input | Position setpoint, as `command.setpoint.<axis id>` over OPC UA. Start value: the start pose |
| `command.enable` | Boolean | input | `false` switches the drives off (brakes engage), `true` on again. Start value `true` |
| `machine` | String | parameter | The machine id |
| `config_file` | String | parameter | A config file to use instead of the one in the FMU; empty (the start value) for the one in the FMU |
| `effects.<name>` | Boolean | parameter | Each effect switch, start value as in the config; a value set here wins over the config file's |

The names are the signal ids as in the CSV columns (naming convention "flat"). The value reference of a variable is its place in the list.

How a step works:

- The FMU steps by itself in its base step `sim.dt_s` (62.5 µs in the shipped configs) with its own integrator. A communication step that is not a whole multiple of it is refused (`fmi2Error`, with a message), as is a step from another time than where the simulation stands, a time or step size that is not a finite number, and a step of more than 4294967295 base steps (about 3 days of simulated time at 62.5 µs). `fmi2SetupExperiment` refuses a start time, and a stop time or tolerance it is told is defined, that is not a finite number. Nothing is interpolated. Outputs are read from initialisation mode on, as FMI 2.0 has it; `fmi2GetReal` before is refused.
- At the start of each communication step a changed setpoint input goes to the controller as one setpoint, which it interpolates towards over `profile.setpoint_interval_s` (10 ms), exactly as a PLC's setpoint over OPC UA. The communication step should therefore be that interval: it is the step size of the model description's default experiment. With shorter steps each new setpoint starts a new interpolation from where the last one had got to, which follows the stream with some lag; with `profile.setpoint_interval_s` set to the communication step in the config the two agree.
- A setpoint the controller refuses (too far for one interval, `error-setpoint-too-far`; outside the stroke) is logged as a warning, the step runs without it, and `fmi2DoStep` returns `fmi2Warning`. An axis whose input is not set stays where the controller has it.
- Parameters can be set until initialisation ends. A config file that changes the list of signals or their units is refused when initialisation ends, with the first signal that differs: export a new FMU for that config.
- Its signals are bit for bit those of `sim-cli run` with the same setpoints at the same times (tested).
- After a DC-link overvoltage trip with a braking resistor too small for the braking energy, `command.enable` set back to `true` right away trips again, and again, until the axis coasting after the trip has stopped: its kinetic energy still flows into the DC link as soon as the drives brake it. Observed 7 to 57 trips again over 6 to 60 ms before the link settles at the bus level and the drives stay on. That is the physics of an undersized resistor, not a defect of the FMU; wait until the axis stands, or size the resistor (`sim-cli verdicts`).

Limits for this task:

- Checked with fmpy and OMSimulator (validation against the FMI schema, simulation compared with `sim-cli run`; [FMI export](fmi.md#validation) has the commands). Run in OpenModelica 1.27.1 through OMSimulator (FMI 2.0; its Modelica import takes Model Exchange only); import into Simulink or Dymola is untested.
- Co-Simulation only, no Model Exchange ([why](fmi.md#why-there-is-no-model-exchange)). The FMU state works by replaying every step since initialisation, so restoring a state takes as long as simulating up to it. The directional derivatives are zero, because the outputs depend on no input directly. No asynchronous steps.
- Inputs are position setpoints only: no move targets, cycle or path commands, and no torque input. The servo loops stay inside the FMU.
- Messages and descriptions in one language, the one of `--lang` when the FMU was written.
- A 64-bit Windows binary unless a Linux one is added as above; no 32-bit binaries.

## Use the app

```sh
cargo run --release -p sim-app
```

The window has the 3D view in the middle, a control panel on the left and the scopes on the right.

**3D view.** Built from simple shapes. Pulleys, motor shafts and idlers turn; belt teeth travel (drawn every 40 mm, not at the tooth pitch); belt sections shift to red above pretension and to blue below. Labels show each motor's speed in rpm and each axis position. Drag with the left mouse button to orbit, use the wheel to zoom. When a path is planned (the Path part of the right panel), the view draws the planned path as a yellow line where the tool will go, and over it in light blue the path the tool tip really travels once the machine runs it.

**Left panel, top to bottom:**

| Control | Does |
| --- | --- |
| Machine | Switches machine. Restarts the simulation with that machine's default config |
| Language | English or Deutsch, switchable while running |
| Pause / Resume | Stops and continues the simulation |
| Speed (1 = real time) | Simulated seconds per real second, 0.01 to 2 |
| View: Front, Side, Top, Angled | Camera presets |
| Motion: one target per axis | Target of the next move |
| Move to target | Point-to-point move, all axes synchronised |
| Start cycle | Runs the configured cycle until stopped |
| Stop after segment | Finishes the segment under way, then stays. There is no fast stop |
| Disable drives / Enable drives | Drives off (brakes engage) and on again |
| Messages | Refused commands and config values, with a Clear button |
| Effects | One checkbox per `[effects]` switch. Takes effect at once, without a restart |
| Parameters | Every numeric config value, grouped by table. Drag to change; applied to the running simulation. A value the simulation refuses is reported under Messages and taken back |

**Right panel:**

| Control | Does |
| --- | --- |
| Time window [s] | Seconds of history the scopes show, 0.2 to 60 |
| Freeze | Keeps the traces as they are; the simulation runs on |
| Export CSV | Writes every signal over the visible window to `traces-<time>.csv` in the working directory |
| Scopes (heading) | Which quantities are plotted |
| Config file: File, Open, Save | A file name, typed in. "Open" loads it as a config of the machine shown, as `--config FILE` does: it replaces the running config and restarts the simulation, as switching machine does. A config of another machine (one the machine shown refuses and another reads) switches to that machine and opens it there. A file that cannot be read or that no machine takes (a misspelt key, a value out of range) is answered under Messages, and the running config stays. "Save" writes the config as it is now, every change of the Effects and Parameters panels and of the part names included. Where only values were changed, the file keeps the comments and the layout of the file the config was read from (the opened file, or the machine's default config); where an array was changed it is written plainly, without comments. Switching machine empties the file field, and "Save" does not write over a file that holds another machine's config: it says so under Messages and leaves the file as it is |
| Config file: Open…, Save as… | The same with the system's file dialog, which starts where the field points. The window waits while the dialog is open |
| Config file: Motor article, Supply, Gearbox, Drive article, Belt, Axis, Use parts | The parts `[parts]` of the config names, shown as the config has them. "Use parts" names what is typed there in the running config, as `--motor`, `--supply`, `--gearbox`, `--drive`, `--belt` and `--axis` do and as "Use" of the motor search does: a motor by its article number (`1FK2104-5AK`), the supply `230v`, `400v` or `s120`, a gearbox as size-frame-ratio (`NRK070-04-5`), a drive by its article number (`6SL3210-5HE11-5UF0`), a belt as profile-width (`AT5-25`), an axis by name (`EXCT-30`, `ELZI-40`); an empty field names none. A name the catalogue does not have is refused under Messages, and so is any name in a config without `[parts]` |
| Analysis: Analyse and judge the cycle | The cycle analysis and the verdicts, as three tables: the figures of the machine (cycle time, path error, off target while dwelling, lowest belt tension, slack time, radial load, guide load, guide life), the load on each motor (peak, RMS and cubic-mean torque, winding rise, energy drawn and fed back, the braking resistor's energy and mean power, highest DC link voltage) and one row per check |
| Analysis: with error bands, samples | Ticked, "Analyse and judge the cycle" also runs the cycle with every value of `[bounds]` at its low and its high bound (two more cycles per value) and adds a fourth table: per check the lowest and the highest value, the worst-case result and "Runs passed [%]". The same as `sim-cli verdicts --bands`. With a number above 0 in the field "samples" beside the tick box, the cycle is instead run that often with all values of `[bounds]` between their bounds at once, and the table is headed "The checks with all uncertain values between their bounds at once". The same as `--bands --samples N`; the app always uses seed 1. With "worst corner" (only together with the error bands) it also searches per check for the worst corner of the bounds and shows a table of the worst value, the limit there, the result and the values at their high and at their low bound. The same as `--bands --worst`; it takes many more cycles |
| Analysis: operating points | After "Analyse and judge the cycle": the motor torque over the motor speed once per millisecond of the cycle, one colour per motor, over the motors' torque-speed curves (dashed: the peak torque both ways up to the rated speed, falling to nothing at the no-load speed; dotted: the rated, S1, torque both ways up to the rated speed), and below it the same points as a table that scrolls. A click on a motor's name in the legend hides its points. The same as `sim-cli analyze --points` |
| Effect views | Opens when a cycle has been analysed; its own button "Analyse and judge the cycle" does the same as the Analysis part's. "Where each motor's torque goes": per motor a plot over the scopes' time window of the four parts of its torque (inertia, gravity, friction, belt and the rest) and, dashed, the torque itself, which they add up to; it shares the scopes' time axis and cursor. "Torque parts over the analysed cycle": per part and motor the peak and the RMS, as `sim-cli analyze --breakdown` writes them. "Energy budget of the analysed cycle": the supply, every loss and store that is not exactly zero, and what is not accounted for, in J and in percent of the supply, as `--budget` writes it, and below it the same as a flow diagram: from the supply (and any store that gave energy back) on the left to what the electrical side lost (copper, braking resistor, mains) and the shaft work in the middle, and from the shaft work on to what the machine lost or stored (guides, shafts, gearboxes, dampers, kinetic and potential energy) on the right, each band as wide as its energy; a bar too thin for its name has it in its tooltip. "Friction per guide": per guide the friction force of steady sliding over the speed (solid) and its Coulomb, viscous and stiction parts (dashed) at the guide load of the moment, the point where the guide is now, and a table of the speed, the three parts, the force and the load factor now. "Temperatures": the rise of each winding and, with gearbox and guide heating on, of each gearbox and guide over the scopes' window, the table of now, where each settles on the analysed cycle, its rated rise and the share of the rating where it settles, and the colour scale of the 3D view. "Belt sections": tension (pretension dashed) and stretch per section, the table of both now, and the belt's colour scale. "Effect on and off": an effect, a quantity and "Compare"; both runs overlaid, off dashed, and per line the peak and RMS of the difference. Above each plot, its lines' names in their colours. See [See where torque and energy go](#see-where-torque-and-energy-go) |
| Analysis: Sensitivity | The eight parameters with `[bounds]` that the cycle depends on most |
| Analysis: Optimise the cycle | The optimiser; with "error bands" ticked, its result has to pass in the bands too (the rows below the settings say how fast it got on the config as written and what that fails there). Below the table the front of energy per cycle against cycle time, the result marked. "Apply these settings to the running machine" puts its result into the config |
| Motor search | Supply (230v, 400v or s120), how the article numbers start (empty: every motor of the catalogue) and optionally a gearbox series such as `NRK` or `NRK070`; ticked, "Every axis of the catalogue" tries each motor on every catalogue axis of the machine's kind as well (`--axes`). "Search" runs every such motor on the cycle and lists them, passing ones first, cheapest first (installed mass stands in for the price), with gearbox, axis, the mass of motor and gearbox, the installed mass, and the check nearest to its limit or the reason a motor or an axis was not run. "Use" in a row names that motor, its supply, its gearbox, its drive and its axis in `[parts]` of the running config. The same as `sim-cli search` |
| Path | A file name, typed in. "Lines, arcs and splines" is a path file, planned within the axis limits or, ticked, "As fast as the motors allow"; "Poses at times" is a trajectory file. "Plan, run and analyse" plans the path from where the machine stands, analyses it on a simulation of its own (duration, deviation by transition curves, path error, distance behind the setpoint, loads, checks), draws the planned path as a plot in the panel and as a line in the 3D view, and has the machine in the 3D view run it. The same as `sim-cli analyze --path` or `--trajectory`. Below the tables, the path's operating points over the torque-speed curves, as for the cycle (`--points` with `--path`). On the delta, the workspace map: a horizontal slice at the height the slider sets, from the travel range to half its width beyond each side, each point coloured by the condition number of the Jacobian (1 dark, the rated `ratings.max_condition_number` light, red beyond it); faint where the arms reach but the machine does not go (outside the working envelope, beyond a joint's stop, above `workspace.max_condition_number`), empty out of reach; the frame is the travel range, and the pointer shows the condition number under it |
| Trace, Mapping | A file name, typed in, and for a vendor's export its mapping file (as `--map` takes it; empty for a trace in the simulator's columns). "Replay and fit" replays the drive trace and shows per motor how far the simulation is from it (torque and following error, RMS and peak), per axis the fitted mass, friction and constant force next to the config's (on the delta, the SCARA and the arm the payload's factor and per motor added inertia, friction and constant torque), with what to change in the config; ticked, "Belt stiffness from the moves" adds the factor on the belts' stiffness (`fit --stiffness`, some twenty replays more); and per motor what the trace says by itself: inertia, friction, the loops' lags, each with its standard error and 95 % interval, and how well each model reproduces the trace. The same as `sim-cli replay`, `fit --trace` and `identify --trace`. The config's start pose has to be where the trace starts. Below, "Trace against replay": motor torque, motor speed and following error at the motor over the time, the trace solid and the simulation dashed, one colour per motor. "Export CSV" there writes them to `replay-overlay.csv` in the working directory, the same file as `sim-cli replay --out` |
| Frequency response of the mechanics | From, to, frequencies and torque amplitude as `sim-cli sweep` takes them (5 to 500 Hz, 120, and 1.5 N m or 40 % of the drive's peak torque where that is less, at the start). "Measure every motor" measures the open-loop response from each motor's torque to its speed, as `sim-cli sweep --motor N` does for one: gain in dB and phase over the frequency, one line per motor, and per motor the dip where the load swings on the belt (resonance and damping ratio of the dip lowest in frequency; empty where the range does not hold a dip with both its flanks). The defaults show it on every motor of the default machines. Before, they did not: the amplitude was 0.05 N m, at which stiction holds the motors, not the range. Switching machine or opening a config sets the amplitude to that config's default unless another one was typed |
| Frequency response: Measured response, taken on, Fit the belt stiffness | A file as `sim-cli sweep` writes it (the columns `frequency_hz` and `gain_rad_s_per_nm`), measured on the motor ticked with the torque amplitude above. "Fit the belt stiffness" runs the config's own response at the file's frequencies and shows resonance and damping ratio of both, and what to change in the config: the factor on the belt stiffness. The same as `sim-cli fit --sweep FILE --sweep-motor N --amplitude A` |
| Tuning: Measure step and frequency response | Step response and closed-loop response of one axis, with rise time, overshoot, settling time and bandwidth; below them the modes the machine reports over that axis's stroke, with the shaper frequencies dashed |
| Scope plots | One plot per quantity, one line per axis or motor. Setpoints are dashed in the colour of their actual value. Shared time axis and cursor; min, max, span and RMS per trace; units switch to mm and µm by themselves |

The right panel scrolls as a whole, and each of its parts (Scopes, Config file, Analysis, Effect views, Motor search, Path, Trace, Frequency response of the mechanics, Tuning) folds away under its heading. A part opens by itself when its result arrives. The tables are laid out for the panel's default width: long names wrap, and numbers have four digits.

The buttons of these parts run in the background on simulations of their own, with the config as it is at that moment; a spinner stands in for the button meanwhile. The running machine is not disturbed. A result is not redone when a parameter changes afterwards. It is marked "computed for an earlier configuration" instead, and nothing of it is applied to the changed config: "Apply these settings", "Use" and the run of a planned path answer with a message then (take the change back, or compute again). Naming a motor with "Use" does not outdate the search it came from, so another row can be tried after it; naming an axis of the axis search does, as the search ran on the config's own axis otherwise.

A file that is missing or cannot be read, a line of it that cannot be read, a supply other than 230v, 400v and s120 and a search that finds no motor, or no gearbox of the series that fits one, are answered under Messages. So is a path that the machine cannot run because it has moved since the plan was made, or is busy.

A drive that trips on DC link overvoltage is reported under Messages, once per trip, for the running machine and for an analysed cycle or path. "Enable drives" brings the drives back.

What the running machine does with a change:

- A changed `gearbox.ratio` or `geometry.pulley_radius_m` rebuilds the machine's state at the tool's pose and speed, with belts and gears as they are at rest there. The belt tensions stay within some percent of what they were. The linear delta keeps its belts' stretch, its couplings' torque and its guides' friction as they were instead: at speed its belts carry 34 N more than at rest.
- A change to the input shaper (its switch, frequency, damping, or following the pose) is taken when the planned motion next stands still, so the setpoint does not jump. A cycle without a dwell and without a gripper action never stands still; there the change waits for Stop.
- A cycle that loses segments while it runs goes on within the segments it has.

Messages: one that only repeats the last one is not added. If the simulation or a background computation ends unexpectedly, a message says so; choosing the machine again starts the simulation anew. If the window stalls for more than ten seconds of simulated time, the scopes lose the samples before that.

Limits for this task:

- The app starts with the machine's default config; another one, such as the EXCT-30 config `crates/machine-t-gantry/exct-30.toml`, is opened under Config file. There is no `--set`: values are changed in the parameter panel.
- A number of a named part that is changed in the parameter panel is overwritten again.
- The cycle itself cannot be edited in the app.
- Not in the app, only on the command line: a sensitivity run over tables without `[bounds]`.
- The search table shows the tightest check only; peak torque and winding rise of a candidate are in `sim-cli search`, or in the cycle analysis after "Use".
- File names of paths, traces and measured frequency responses are typed; only the config file has a file dialog. A relative name is relative to the directory the app was started in.
- The lines of the planned and the travelled path in the 3D view were not checked on screen by a person.
- A path is analysed from where the machine stands when the button is pressed. A machine that is moving then will refuse the run.
- The base time step `sim.dt_s` cannot be changed while running.
- The mouse handling was not checked by a person. Screenshots of the window in both languages were.
- Desktop only.
