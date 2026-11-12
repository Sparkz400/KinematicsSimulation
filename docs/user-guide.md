# User guide

One section per task. Each gives the command, what the output means, and the limits that matter for that task. The physics behind it is in the [model reference](model-reference.md); the config keys are in the [config reference](config-reference.md).
Deutsche Fassung: [Benutzerhandbuch](de/benutzerhandbuch.md).

- [Common to all commands](#common-to-all-commands)
- [Size a motor for a cycle](#size-a-motor-for-a-cycle)
- [Check a real catalogue machine](#check-a-real-catalogue-machine)
- [Find the lightest motor](#find-the-lightest-motor)
- [See which unknown parameters matter, and judge with error bands](#see-which-unknown-parameters-matter-and-judge-with-error-bands)
- [Shorten a cycle](#shorten-a-cycle)
- [Run a path and read path error](#run-a-path-and-read-path-error)
- [Tune gains and use input shaping](#tune-gains-and-use-input-shaping)
- [Replay a drive trace and fit parameters](#replay-a-drive-trace-and-fit-parameters)
- [Record raw signals, switch the drives off](#record-raw-signals-switch-the-drives-off)
- [Connect a PLC over OPC UA](#connect-a-plc-over-opc-ua)
- [Use the app](#use-the-app)

The examples write `sim-cli` for `cargo run -q --release -p sim-cli --`. All output shown was produced by the commands shown.

## Common to all commands

```text
sim-cli COMMAND MACHINE [options]
```

`MACHINE` is `t-gantry` or `single-axis` (`sim-cli list` prints them). Options that every command but `list` takes:

| Option | Effect |
| --- | --- |
| `--config FILE` | Use this config instead of the machine's default. The EXCT-30 config is `crates/machine-t-gantry/exct-30.toml` |
| `--motor ARTICLE` | Put a motor of the Siemens S210 catalogue into the config, e.g. `1FK2205-4AF`. Sets `parts.motor` |
| `--supply 230v` or `400v` | The supply the motor data is for. Sets `parts.motor_supply`. Default as in the config |
| `--gearbox SIZE-FRAME-RATIO` | Put a catalogue gearbox into the config, e.g. `NRK070-04-5`. Sets `parts.gearbox` |
| `--set path=value` | Change one config value, e.g. `--set mass.payload_kg=3.0`. Repeatable. Applied after the parts, so single values can be set on top of a catalogue part |
| `--out FILE` | Write the CSV to a file instead of standard output |
| `--lang en` or `de` | Language of messages. The CSV headers are not translated |

Rules that are easy to trip over:

- `--set` only changes values that exist. A misspelt key is refused (`Configuration value is missing: mass.cariage_kg`), and so is a value of the wrong type.
- `--motor`, `--supply` and `--gearbox` need a `[parts]` table in the config. All three shipped configs have one.
- Every option except `--cycle`, `--time-optimal` and `--bands` takes a value. An option without its value is refused (`This option needs a value: --time`), and so is an option the command does not know, such as a misspelt one (`This command has no such option: --cylce`). A number has to be finite: `--time inf` is refused.
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
motor,cycle_s,peak_torque_nm,rms_torque_nm,peak_speed_rad_s,utilisation,thermal_utilisation,belt_tension_min_n,belt_tension_max_n,rms_current_a,winding_rise_k,energy_in_j,energy_regenerated_j,peak_regenerated_power_w,largest_regeneration_j,path_error_peak_m,path_error_rms_m,tracking_error_peak_m,dwell_error_peak_m,cubic_mean_torque_nm,braking_resistor_energy_j,braking_resistor_mean_power_w,dc_link_peak_voltage_v,belt_slack_time_s,radial_load_peak_n,guide_load_peak_n,guide_life_km
a,6.080000,0.644237,0.166790,200.2390,0.1709,0.1313,202.62,334.58,0.3708,1.25,85.244,10.435,29.12,5.954,1.319e-4,4.409e-5,1.559e-4,4.588e-5,0.198428,0.000,0.000,222.98,0.0000e0,6.4544e2,1.8007e2,4.6452e6
b,6.080000,0.466063,0.154163,200.2192,0.1227,0.1214,202.62,334.58,0.3427,1.07,74.769,15.322,35.58,7.241,1.319e-4,4.409e-5,1.559e-4,4.588e-5,0.176294,5.281,0.869,224.65,0.0000e0,6.4544e2,1.8007e2,4.6452e6
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

The exact definitions are in the [model reference](model-reference.md#analysis-quantities).

`--points FILE` also writes the operating points, one per millisecond and motor (`motor,speed_rad_s,torque_nm`), for plotting over the torque-speed curve.

Then judge the cycle:

```text
$ sim-cli verdicts t-gantry
check,motor,value,limit,result
motor-peak,a,0.1709,1.000,ok
motor-thermal,a,1.248,100.0,ok
gearbox-continuous,a,0.7995,40.00,ok
gearbox-input-speed,a,1912,13000,ok
gearbox-peak,a,2.228,64.00,ok
dc-link-energy,a,6.029,6.205,ok
dc-link-overvoltage,a,223.0,257.6,ok
...
settling,,0.00004588,0.0001000,ok
belt-tension,,334.6,1500,ok
belt-slack,,202.6,0,ok
gearbox-radial-load,,645.4,1500,ok
guide-load,,180.1,2000,ok
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

Limits for this task:

- A rating that the config leaves out is not judged. No row is not a pass.
- `belt-slack` is the one check where the value has to stay above the limit. It also fails if any section spent any time without tension.
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
dc-link-energy,a,6.142,6.205,ok
dc-link-overvoltage,a,224.1,257.6,ok
...
dc-link-energy,b,6.142,6.205,ok
dc-link-overvoltage,b,224.1,257.6,ok
...
```

On the default cycle the highest DC link voltage is 224.65 V with separate links (motor B, at the chopper's threshold) and 224.06 V with the shared one. The resistor is not needed then, so its two rows are gone.

## Check a real catalogue machine

A config can describe a real machine with its values taken from the catalogue. [`exct-30.toml`](../crates/machine-t-gantry/exct-30.toml) does that for a Festo EXCT-30 with two Siemens 1FK2205-4AF motors on S210 converters at 400 V. Every value in it is marked `[catalogue]` or `[estimate]`.

```text
$ sim-cli analyze t-gantry --config crates/machine-t-gantry/exct-30.toml
motor,cycle_s,peak_torque_nm,rms_torque_nm,peak_speed_rad_s,utilisation,thermal_utilisation,belt_tension_min_n,belt_tension_max_n,rms_current_a,winding_rise_k,...
a,1.934000,4.060260,1.547724,157.1821,0.3251,0.3365,239.14,558.35,1.2137,7.76,...
b,1.934000,3.353654,1.483132,157.1735,0.2686,0.3224,239.14,558.35,1.1628,7.11,...
```

Four values in this config are choices of the designer, made so that every check passes in the worst case of the error bands ([below](#see-which-unknown-parameters-matter-and-judge-with-error-bands)): a belt pretension of 450 N, an X jerk limit of 150 m/s³, input shaping at a fixed 25 Hz, and corner blending. The comments in the file give the reasons.

Like the default configs, this one has `effects.dc_link = true`, so the braking rows of `verdicts` are simulated, not estimated: the DC link voltage rises to the chopper threshold of 770 V and the resistor burns the rest.

```text
dc-link-energy,a,9.943,9.943,ok
braking-resistor-continuous,a,8.859,100.0,ok
braking-resistor-peak,a,213.7,3000,ok
dc-link-overvoltage,a,770.0,820.0,ok
```

**Parts by name.** `[parts]` in a config, or `--motor`, `--supply` and `--gearbox` on the command line, name a motor and a gearbox from the catalogue. Their catalogue values then replace the config's numbers:

| Part | What it sets |
| --- | --- |
| Motor | `motor.rotor_inertia_kgm2`, `motor.torque_constant_nm_a` (static torque over stall current), `motor.max_current_a`, `motor.torque_constant_at_max_current_nm_a`, `motor.rated_current_a` (the catalogue's own, at most 1.05 times the stall current), `motor.thermal_time_constant_s` (where the configuration manual has it), `drive.peak_torque_nm` (capped by `ratings.pulley_max_torque_nm` over the gear ratio), `drive.rated_torque_nm`, `drive.rated_speed_rad_s`, `drive.bus_voltage_v` (the DC link voltage: 540 V for `400v`, 325 V for `230v`), `drive.no_load_speed_rad_s` (sqrt(3/2) × DC link voltage over the torque constant), and `ratings.braking_threshold_v` and `ratings.dc_link_overvoltage_v` scaled with the DC link voltage |
| Gearbox | `gearbox.ratio`, `gearbox.efficiency`, `gearbox.stiffness_nm_rad`, `gearbox.backlash_rad` and `gearbox.inertia_at_input_kgm2` (both where the manual gives them), `ratings.gearbox_rated_torque_nm` and `ratings.gearbox_max_torque_nm` (where the config has them) |
| Either | `drive.velocity_kp_nm_s_rad`, retuned for the part (see below) |

What a part by name does **not** set, and what follows from that:

- **Winding resistance and inductance.** Siemens does not publish them for the 1FK2 and 1FT2. The config's values stay, and the current controller's gains with them.
- **The DC link capacitance and the braking resistor's powers.** They stay as the config has them. The braking threshold and the trip level keep their ratio to the DC link voltage, which is sound for placeholders and no more: set all five DC link ratings for the real drive before trusting the braking rows.
- **Gearbox input speed and radial load.** The gearbox files do not have them. `ratings.gearbox_max_input_speed_rpm` and `ratings.shaft_max_radial_load_n` stay as the config has them.

**The velocity gain is retuned.** A motor with a much lighter rotor oscillates with a gain tuned for a heavier one, which is a failure of the tuning, not of the motor. So wherever a part is named (`[parts]`, `--motor`, `--gearbox`, the search), `drive.velocity_kp_nm_s_rad` is scaled by the square root of (ratio of the inertia the motors turn) × (ratio of the rotor inertias, the gearbox's own included). The reference is the config's own numbers as the file states them before the part goes in: the gain in a config is taken to be tuned for the rotor, gearbox and masses written in that config. `--set drive.velocity_kp_nm_s_rad=...` on the command line is applied after the parts and overrides the retuned gain.

```text
$ sim-cli verdicts t-gantry --config crates/machine-t-gantry/exct-30.toml --motor 1FK2204-6AF
motor-peak,a,0.4001,1.000,ok
motor-thermal,a,13.38,100.0,ok
...
settling,,0.00003980,0.0001000,ok
belt-slack,,236.9,0,ok
```

The rotor of the 1FK2204-6AF has a third of the inertia of the config's 1FK2205-4AF; the gain goes from 0.6 to 0.31 N m s/rad. With the gain left at 0.6 the same motor oscillates and its winding would settle thousands of kelvin up.

An unknown article is refused: `The catalogue has no such part: motor 1FK9`. A motor that the catalogue does not offer at the chosen supply is refused the same way.

## Find the lightest motor

```text
$ sim-cli search t-gantry --config crates/machine-t-gantry/exct-30.toml --series 1FK2
Ruled out by catalogue numbers against the axis limits, without simulating a cycle: 13 / 28 candidates
motor,gearbox,mass_kg,result,tightest_check,tightest_utilisation,peak_torque_nm,winding_rise_k
1FK2104-5AF,,2.85,ok,settling,0.5083,3.5749,22.73
1FK2104-5AK,,2.85,ok,settling,0.5081,3.5812,46.58
1FK2204-5AF,,2.90,ok,motor-peak,0.5260,3.7239,23.92
1FK2104-6AF,,3.40,ok,gearbox-radial-load,0.5026,3.6284,12.40
1FK2204-6AF,,3.50,ok,gearbox-radial-load,0.5030,3.7905,13.38
1FK2205-2AF,,3.75,ok,gearbox-radial-load,0.5026,3.9032,16.44
1FK2205-4AF,,5.20,ok,gearbox-radial-load,0.5025,4.0603,7.76
...
1FK2204-5AK,,2.90,FAIL,motor-thermal,1.9796,3.7279,197.96
1FK2103-2AH,,1.18,FAIL,limit-motor-torque,NaN,NaN,NaN
...
1FK2208-3AC,,12.60,FAIL,limit-motor-speed,NaN,NaN,NaN
```

The search runs every motor of the S210 catalogue ([`motors_s210.csv`](../catalog/siemens/motors_s210.csv)) that is offered at the supply on the config's cycle. Passing motors come first, lightest first. Failing ones follow, nearest to passing first. A candidate whose drives trip on DC link overvoltage comes after all of these, with `inf` as its `tightest_utilisation`: its figures end at the trip.

| Option | Effect |
| --- | --- |
| `--series PREFIX` | Only article numbers that start this way, e.g. `1FK2` or `1FT2105` |
| `--supply 230v` or `400v` | Supply to search at. Default `400v` |
| `--gearboxes SERIES` | Also try each motor with every gearbox of that series (`NRB`, `NRK`, `NLC`, or a size such as `NRK070`) that fits its frame and has a ratio between half and twice the config's. Only on a machine whose config has a ratio above 1 |

| Column | Meaning |
| --- | --- |
| `motor`, `gearbox` | The candidate. `gearbox` is empty where the config's own gearbox stayed |
| `mass_kg` | Mass of the motor without brake |
| `result` | `ok` if every verdict passes |
| `tightest_check` | The check closest to its limit, or furthest beyond it. Two DC link checks are left out while they pass, because their ratio says nothing about the margin: `dc-link-energy` when a braking resistor is in use (the capacitors then fill to their limit by design) and `dc-link-overvoltage` (the voltage is never below the DC link's own level). For a candidate that was not simulated: why (`limit-motor-speed`, `limit-motor-torque`) |
| `tightest_utilisation` | Value over limit of that check |
| `peak_torque_nm` | Highest peak torque of any motor of the machine |
| `winding_rise_k` | Highest settled winding rise of any motor |

Before a cycle is simulated, the catalogue numbers are checked against the axis limits: motor speed at the highest axis speed against the rated speed, rigid-model torque at the highest acceleration against the motor's maximum torque. What fails there is listed with `NaN` and not simulated.

Each candidate's velocity gain is retuned by the rule of [parts by name](#check-a-real-catalogue-machine): the square root of (ratio of the inertia the motors turn) × (ratio of the rotor inertias), against the config's own numbers. `--gearbox NAME` for the whole search and `--gearboxes SERIES` give the same result for the same pair: the gearbox's inertia has a key of its own (`gearbox.inertia_at_input_kgm2`) and stays when a motor goes in.

With gearboxes, on the single axis:

```text
$ sim-cli search single-axis --series 1FK2104 --gearboxes NRK070
Ruled out by catalogue numbers against the axis limits, without simulating a cycle: 12 / 35 candidates
motor,gearbox,mass_kg,result,tightest_check,tightest_utilisation,peak_torque_nm,winding_rise_k
1FK2104-4AF,NRK070-04-3,2.05,ok,settling,0.4749,0.8992,3.97
1FK2104-4AF,NRK070-04-4,2.05,ok,settling,0.4632,0.7356,2.50
1FK2104-4AF,NRK070-04-5,2.05,ok,settling,0.4814,0.5543,1.91
...
1FK2104-4AF,NRK070-04-7,2.05,FAIL,limit-motor-speed,NaN,NaN,NaN
```

Limits for this task:

- **Weight stands in for price.** No catalogue file has prices. The weight is the motor's; the gearbox files have no mass.
- The retuning is a rule of thumb, not a tuning from the machine's frequency response. A candidate can still fail for its gain. Check a chosen motor with `tuning`.
- The torque screen leaves out gearbox efficiency.
- Only motors and gearboxes are searched. Drives, belts and axes are not, and neither are combinations of different motors per axis.
- The DC link capacitance and the braking resistor stay as the config has them; the braking threshold and the trip level are scaled to the motor's DC link voltage (see the previous section). On a default config the braking rows of every candidate are judged against placeholders.
- A candidate that asks for more than its rated speed at the axis limits never passes, even if its simulated cycle would.

## See which unknown parameters matter, and judge with error bands

A config gives each uncertain value a low and a high bound in `[bounds]`, a table that mirrors the config:

```toml
[bounds]
belt.pretension_n = [350.0, 550.0]
friction_x.coulomb_n = [12.0, 26.0]
```

**Sensitivity.** The cycle is run with each parameter at its low and at its high value, one at a time, and the rows are sorted by how much the results change:

```text
$ sim-cli sensitivity t-gantry --config crates/machine-t-gantry/exct-30.toml --group mass --group friction_x --group belt
parameter,value,low,high,peak_torque_low,peak_torque_high,rms_torque_low,rms_torque_high,cycle_time_low,cycle_time_high,belt_tension_min_low_n,belt_tension_min_high_n
mass.z_axis_kg,4.7,3.7600000000000002,5.64,-0.0672,+0.0671,-0.0673,+0.0688,+0.0000,+0.0000,251.36,227.13
friction_x.viscous_n_s_m,20.0,10.0,40.0,-0.0341,+0.0683,-0.0161,+0.0444,+0.0000,+0.0000,247.11,223.14
mass.carriage_kg,7.16,5.728000000000001,8.592,-0.0645,+0.0652,-0.0463,+0.0483,+0.0000,+0.0000,254.50,224.13
mass.payload_kg,3.0,2.4000000000000004,3.5999999999999996,-0.0364,+0.0358,-0.0272,+0.0275,+0.0000,+0.0000,247.00,231.17
friction_x.coulomb_n,20.0,12.0,26.0,-0.0222,+0.0167,-0.0086,+0.0072,+0.0000,+0.0000,244.57,235.20
...
belt.pretension_n,450.0,350.0,550.0,-0.0000,-0.0002,+0.0000,-0.0000,+0.0000,+0.0000,139.15,339.14
...
belt.tooth_mesh_stiffness_n_m,5000000.0,2500000.0,10000000.0,+0.0000,+0.0000,+0.0000,+0.0000,+0.0000,+0.0000,239.14,239.14
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

The masses in the example have no bounds in the config, so they were varied by 20 %. A parameter whose changed value the machine refuses is left out of the table. The pretension hardly moves the torques, so it stands far down, but it sets the lowest belt tension: 139 N at its low bound. The last row shows a parameter of an effect that is switched off: it changes nothing.

**Error bands on the verdicts.**

```text
$ sim-cli verdicts t-gantry --config crates/machine-t-gantry/exct-30.toml --bands
check,motor,value,limit,result,low,high,worst_case
motor-peak,a,0.3251,1.000,ok,0.3162,0.3450,ok
motor-thermal,a,7.764,100.0,ok,7.508,8.494,ok
pulley-torque,a,3.405,12.50,ok,3.294,3.635,ok
dc-link-energy,a,9.943,9.943,ok,4.971,23.16,ok
braking-resistor-continuous,a,8.859,100.0,ok,4.999,14.00,ok
braking-resistor-peak,a,213.7,3000,ok,185.9,228.6,ok
dc-link-overvoltage,a,770.0,820.0,ok,708.6,790.0,ok
...
settling,,0.00003505,0.0001000,ok,0.00003164,0.00004055,ok
belt-tension,,558.4,2140,ok,458.4,658.3,ok
belt-slack,,239.1,0,ok,139.1,339.1,ok
gearbox-radial-load,,1005,2000,ok,805.1,1205,ok
guide-load,,185.3,2000,ok,177.3,286.6,ok
```

`low` and `high` are the lowest and highest value the check took over all runs: the config as it is, and each bounded parameter at its low and at its high bound. `worst_case` is `ok` only if the check passed in every run. Here every check passes in every run: with the belt set to its low bound of 350 N the slackest section keeps 139 N, the tool is at most 0.041 mm off its target in the dwell, and the belt pulls a pulley shaft with at most 1205 N. This run simulated 123 cycles (61 bounded values) on all processor cores and took about 2 s on the machine this guide was written on.

An earlier state of this config failed here: with 300 N of pretension a belt set to 200 N went slack, and with the frame at the soft end of its bounds the tool was 0.16 mm off in the dwell, against a tolerance of 0.1 mm (the tool position counts the frame's sway). The pretension was raised to 450 N, the X jerk limit lowered from 500 to 150 m/s³, and input shaping and corner blending were switched on. The cycle takes 1.934 s now, against 2.136 s before.

**All bounded values at once.** With `--samples N` the cycle is run N times, each time with all bounded values between their bounds:

```text
$ sim-cli verdicts t-gantry --config crates/machine-t-gantry/exct-30.toml --bands --samples 200
check,motor,value,limit,result,low,high,worst_case,pass_share
motor-peak,a,0.3251,1.000,ok,0.3063,0.3791,ok,1.000
motor-thermal,a,7.764,100.0,ok,7.310,9.353,ok,1.000
pulley-torque,a,3.405,12.50,ok,3.170,3.992,ok,1.000
dc-link-energy,a,9.943,9.943,ok,4.812,29.92,ok,1.000
braking-resistor-continuous,a,8.859,100.0,ok,0.1327,14.76,ok,1.000
braking-resistor-peak,a,213.7,3000,ok,20.14,240.7,ok,1.000
dc-link-overvoltage,a,770.0,820.0,ok,658.5,789.9,ok,1.000
...
settling,,0.00003505,0.0001000,ok,0.00002482,0.00005487,ok,1.000
belt-tension,,558.4,2140,ok,450.8,668.9,ok,1.000
belt-slack,,239.1,0,ok,121.6,340.1,ok,1.000
gearbox-radial-load,,1005,2000,ok,802.3,1215,ok,1.000
guide-load,,185.3,2000,ok,172.1,324.2,ok,1.000
```

The values are drawn as a Latin hypercube: each value's range is cut into N equal parts, each part is used in exactly one run, and the order of the parts is random per value. The random numbers come from `--seed S` (default 1), so the same call gives the same table. `low`, `high` and `worst_case` are the envelope of these runs and the config as it is. `pass_share` is the share of the runs, the config as it is included, in which the check passed. Here the bands are wider than one at a time (122 N instead of 139 N in the slackest section, 0.055 mm instead of 0.041 mm off in the dwell, 1215 N on a pulley shaft), and every check still passes in all 201 runs. That took about 4 s.

`--samples` without `--bands`, 0 samples, a fraction, and `--seed` without `--samples` are refused (exit code 2). The runs of both kinds use all processor cores.

Limits for this task:

- **Plain `--bands` is one at a time.** Two values that are both off can take a result beyond its band. `--samples` shows that.
- **Samples do not find the worst corner.** A sample never sits exactly on a bound (plain `--bands` does that), and a few dozen runs do not find the worst combination of many values. Run both.
- The bounds are themselves estimates. In the default configs only friction, damping, stiffness, pretension, ripple and gearbox efficiency are bounded, not the masses, inertias and backlash.
- Where a bound changes a rating (the EXCT-30 config bounds its DC link ratings), the `limit` column still shows the nominal limit.
- A bound on a key that the config does not have is refused. So is a pair where low is above high.

## Shorten a cycle

```text
$ sim-cli optimise t-gantry --config crates/machine-t-gantry/exct-30.toml
setting,before,after
cycle_s,1.9340,1.4920
dwell_error_peak_m,3.505e-5,6.965e-5
axis.x.max_acceleration_m_s2,25.0,53.99999999999999
axis.x.max_jerk_m_s3,150.0,337.5
axis.x.max_velocity_m_s,2.5,3.0
axis.z.max_acceleration_m_s2,15.0,32.4
axis.z.max_jerk_m_s3,500.0,1125.0
axis.z.max_velocity_m_s,1.5,1.7999999999999998
effects.input_shaping,true,false
```

The optimiser tries corner blending and higher velocity, acceleration and jerk limits (all axes together, in steps of ×1.5 and ×1.2). It keeps a change if the cycle gets shorter and every verdict still passes, among them `settling`, so a faster move that leaves the tool swinging at the pick does not count. Input shaping makes no cycle shorter by itself, because every stop waits one period of the shaper. So it is tried where it can help: a faster setting that fails on `settling` alone is run again with the shaper (and with `effects.shaper_follows_pose` where the machine reports a resonance) and taken if it then passes and is shorter. Where the shaper is on already, switching it off is tried too. The first two rows are the cycle time and the dwell error before and after. The other rows are what to change in the config. No config file is edited.

The EXCT-30 config has corner blending and the shaper on already, so here the optimiser raises the limits and switches the shaper off: the cycle falls from 1.93 s to 1.49 s, and the tool is 0.070 mm off in the dwell instead of 0.035 mm.

`--runs N` limits the number of cycles simulated (default 40).

Limits for this task:

- A search by trial, not a proof of the optimum. It stops at the first setting from which no single step is an improvement.
- Lift heights and via points of the cycle are not varied.
- The cycle as configured has to pass every verdict to begin with, or the command is refused.
- A candidate whose axis limits exceed the motor limits (the warning of [Common to all commands](#common-to-all-commands)) is not accepted.
- It judges against the nominal config. Run `verdicts --bands` on the result. The result above takes back the margins the config was given for its worst case: with these settings `--bands` fails `settling` (0.35 mm) and `belt-slack` (0 N).

**Corner blending by hand:** `--set effects.corner_blending=true`, with `profile.blend_time_s` as the longest lead. The next segment starts while the current one is still braking. It applies only where the machine need not stop: no dwell, no gripper action, no axis reversing, and only on the ramps. On lift-traverse-lower cycles every axis stays within its limits; for other shapes of cycle the sum of two segments on one axis is not checked against the limits.

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
| empty, or starting with `#` | Skipped |

An arc whose end is the end point of the piece before it (the same numbers, to a nanometre) is a full turn. Any other arc turns from that point to its own end, however short: an arc of 80 µm is an arc of 80 µm. The stated end has to lie on the circle around the centre through the end point of the piece before, within 10 µm. The arc is laid from where the path really stands and keeps the radius it has there, so it ends beside its stated end by what the two radii differ. Over arcs that follow one another around one centre these offsets add up if every stated end lies further out than the one before: four arcs, each 9 µm further out, end 36 µm beside the last stated end. Where the path ends beside a stated end, the largest such distance is part of the deviation the planner reports (`The planned path is off ...`, below, and "Planned path off the given one" in the app). The command line says it from 0.05 µm, the smallest it can show.

The first piece has no point before it. A path starts at the setpoint of the machine at rest, which belt stretch and gear twist put up to 0.1 mm from the start pose (some hundredths of a millimetre on the default gantry). So an arc as the first piece is judged against where the machine stands: its end may be 0.1 mm off the circle through that place, and it has to end at least 1 mm from that place. Nearer, a full turn cannot be told from a short arc, so neither is guessed and the path is refused (`The path starts with an arc that ends less than 1 mm from where the machine stands. ...`). To start a path with a full turn or with an arc shorter than that, state its start first with a `line` to the start pose; the arc is then judged against that point. Arcs need a machine with two axes.

A spline runs through all its via points with no step in position, direction or curvature at a via point (a cubic spline over the straight distance between the points). It leaves the line or arc before it in that piece's direction and ends in the direction the line or arc after it starts with. At an end of the path, or where there is no line or arc, it ends without curvature. Splines need a machine with two axes, like arcs. Via points and directions that would make the curve turn back on the spot are refused (`The path cannot be laid out: ...`).

A spline can swing past its via points: between them the curve bulges outwards, the more the sharper it has to turn. A curve that leaves the travel range although every via point is inside is refused (`The path leaves the travel range. ...`). Move the via points inwards or add one where it bulges.

```text
$ sim-cli analyze t-gantry --path corner.path
motor,cycle_s,peak_torque_nm,...,path_error_peak_m,path_error_rms_m,tracking_error_peak_m,dwell_error_peak_m,...
a,2.128000,0.627751,...,1.545e-4,5.518e-5,1.545e-4,0.000e0,...

$ sim-cli analyze t-gantry --path corner.path --time-optimal
a,1.729000,0.663606,...,1.854e-4,6.665e-5,1.854e-4,0.000e0,...
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
| Where a spline meets a line or an arc | The curvature steps, so it slows down as from a line into an arc. No transition curve is put there | The same |
| Model used to plan | None, only the axis limits | The rigid model: no friction, no belt stretch |

The time-optimal plan keeps every motor within the reserve and the rated speed and every axis within its jerk limit at every planned pose. With the default jerk limit of 100 m/s³ the jerk decides nearly everywhere: the example reaches the rated speed on its first line and uses 0.16 of the torque it may. A higher jerk limit makes it faster (0.78 s plus rest at 2000 m/s³).

Reading the result:

- `path_error_peak_m` is the distance of the tool tip from the line its setpoint has drawn, wherever along it. Lagging behind along the path does not count. This is the number for contour accuracy.
- `tracking_error_peak_m` is the distance from where the setpoint is at the same moment. It includes the lag.
- On the T-gantry the tool tip is the tip of the leaning Z axis (`effects.tool_flex`) on a frame that sways (`effects.frame_compliance`), seen from the floor where the workpiece stands. It is not the carriage's position along the beam and not what the motor encoders say.

**Arcs entered at speed.** Where a line runs into an arc the acceleration towards the centre steps from 0 to v²/r, so the default planner passes that place at a crawl: 0.07 m/s for a radius of 0.1 m at 100 m/s³. With `profile.path_tolerance_m` above 0, both planners first put transition curves (clothoids) there, and where two arcs meet: curves whose curvature changes evenly with the distance. To fit one in, the arc keeps its centre and gets a slightly smaller radius, so the planned path is off the lines and arcs of the file by a small amount, and the error stream says by how much:

```text
$ sim-cli analyze t-gantry --path corner.path --set profile.path_tolerance_m=0.0001
The planned path is off the given lines and arcs by up to 0.1000 mm (transition curves, and arcs that end beside their stated end).
motor,cycle_s,peak_torque_nm,...,path_error_peak_m,...
a,1.890000,0.627751,...,1.133e-4,...
```

1.89 s instead of 2.13 s, with every axis still within its velocity, acceleration and jerk limit. The rules:

| | |
| --- | --- |
| Deviation of an arc of radius r with curves of length L | e = L² / (24 r). The arc moves in by this much; on the curves the path is closer to the given one |
| Length of the curves | L = v³ dk / (0.4 j): what the fastest junction of the path needs to pass at the speed v of its slower piece, with 0.4 of the smallest jerk limit j. dk is the step of the curvature, 1 / r for a line into an arc. But no longer than the tolerance allows on the tightest arc: L = sqrt(24 · tolerance · r_min) |
| Speed on a curve that is shorter than needed | v = (0.4 j L / dk)^(1/3) |
| Example above (r = 0.1 m, 100 m/s³, arc at 0.4 m/s) | Needed 16 mm, allowed sqrt(24 · 0.0001 · 0.1) = 15.5 mm: the arc moves in by the full 0.1 mm and is entered at 0.396 m/s instead of 0.071 m/s. With a tolerance of 1 mm the curves are the 16 mm they need, and the path is off by 0.107 mm |

One length serves all curves of a path. An arc at a corner, or at the start or end of the path, keeps its radius (it has to end where the file says) and so gets no curve towards a line. If a piece is too short for its curves, or a curve would turn half a circle or more (a tolerance that is large against the radius of a tight arc), the length is halved for the whole path, up to eight times; after that the path is planned as given. With a tolerance of 0 (the default) nothing changes.

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
motor,cycle_s,peak_torque_nm,...,path_error_peak_m,path_error_rms_m,tracking_error_peak_m,dwell_error_peak_m,...
a,2.101000,0.626580,...,1.590e-4,5.564e-5,1.590e-4,0.000e0,...

$ sim-cli analyze t-gantry --path curve.path --time-optimal
a,1.974000,0.662041,...,1.703e-4,6.001e-5,1.703e-4,0.000e0,...
```

2.101 s within the axis limits and 1.974 s as fast as the motors allow, both with 0.3 s of rest. The curve leaves the first line horizontally and arrives on the last one going straight down. `profile.path_tolerance_m` changes nothing on this path: it has no arc, and splines get no transition curves.

Other ways to give a path:

- `--trajectory FILE`: poses at times, CSV with the time in seconds and one position per axis per line. Lines that do not start with a number are skipped. The first time has to be 0 and the first pose where the machine stands (within 0.1 mm); times have to rise; every pose has to lie within the travel range (up to 0.1 mm beyond it is taken, see the limits below); the path ends at rest. The controller runs a cubic curve through the points.
- `sim-cli run MACHINE --path FILE` or `--trajectory FILE` writes all signals instead of the analysis.

Limits for this task:

- Without a path tolerance a rounded corner is slow with the default planner: 0.07 m/s into an arc of 0.1 m radius at 100 m/s³.
- Splines: where one joins a line or an arc the curvature steps, so the path passes that junction slowly, like a line into an arc without tolerance. Splines get no transition curves; `profile.path_tolerance_m` leaves them, and the arcs next to them, as given. A span has one limit, from its tightest place, so a span with one tight bend is slow along its whole length.
- Transition curves: one length for the whole path, none at an arc that touches a corner or an end of the path, and their length is worked out with the speeds of the axis limits, also for the time-optimal plan (the example takes 1.499 s with a tolerance of 0.1 mm, 1.729 s without).
- The time-optimal plan does not know the belt, the guides or the DC link. Run `analyze` or `verdicts` on it.
- The time-optimal plan is not the fastest one possible. It has one speed and one acceleration limit per piece, and on an arc it shares the motors' limits out between the directions along the path and towards the centre, which leaves a part unused. On the full model the motors pass the rated speed by what the control loops add (0.1 % in the example).
- A machine that stands on an end of a stroke has its setpoint some hundredths of a millimetre beyond it: the setpoint is where the motors stand, under belt stretch and gear twist. A path from there starts that far outside the travel range and is run. Poses more than 0.1 mm outside the range are refused (`The path leaves the travel range. ...` for a path of pieces, `The trajectory needs at least two points inside the travel range, at rising times from 0.` for poses at times).
- The payload is as the machine has it when planning (attached, at the start).
- `verdicts`, `optimise`, `search` and `sensitivity` work on the cycle only, not on a path.
- Paths are not available over OPC UA. In the app they are under "Path" (see [Use the app](#use-the-app)).

## Tune gains and use input shaping

The gains are in `[drive]`: `kv_1_s` (position), `velocity_kp_nm_s_rad` and `velocity_tn_s` (velocity), `current_kp_v_a` and `current_tn_s` (current). The three commands below put a test signal on the position setpoint only, without feedforward, so they show the feedback loops.

**One line per axis:**

```text
$ sim-cli tuning t-gantry
axis,rise_time_s,overshoot,settling_time_s,bandwidth_hz
x,0.02900,0.0625,0.49450,9.507
z,0.04300,0.0000,0.08525,8.625

$ sim-cli tuning t-gantry --set drive.kv_1_s=150
axis,rise_time_s,overshoot,settling_time_s,bandwidth_hz
x,0.01450,0.2691,0.49725,21.961
z,0.00525,0.0691,0.03100,74.895
```

| Column | Unit | Meaning |
| --- | --- | --- |
| `rise_time_s` | s | Time from 10 % to 90 % of a 1 mm step |
| `overshoot` | fraction | Highest excursion beyond the step, as a fraction of it |
| `settling_time_s` | s | After this time the tool stays within 2 % of the step |
| `bandwidth_hz` | Hz | Where the closed-loop gain first falls below 1/√2. Empty if it never does up to 200 Hz |

With Kv = 50 1/s the position loop behaves like a first-order lag with time constant 1/Kv: bandwidth about Kv / 2π = 8 Hz. Z shows that plainly. In X the tool position counts the frame's sway: the step sets the lightly damped frame mode (about 21 Hz) ringing, which is the 6 % of overshoot and the half second until the tool stays within 2 %. Tripling Kv makes X overshoot by 27 %.

**The step response as a trace:**

```text
$ sim-cli step t-gantry --axis 0
time_s,setpoint,response
0.000000,1.000000000e-3,0.000000000e0
0.000250,1.000000000e-3,3.343570421e-10
0.000500,1.000000000e-3,1.406628745e-8
...
```

`--axis N` (0 is the first axis), `--size M` (default 0.001 m), `--time SECONDS` (default 0.5). `response` is the tool position relative to where it stood, four samples per millisecond.

**The closed-loop frequency response:**

```text
$ sim-cli bode t-gantry --axis 0 --count 8
frequency_hz,gain,gain_db,phase_deg
1.0000,9.750295e-1,-0.220,-10.436
2.1317,9.597195e-1,-0.357,-18.626
4.5440,8.808630e-1,-1.102,-34.525
9.6863,7.038070e-1,-3.051,-59.107
20.6478,1.292231e0,2.227,169.624
...
```

`gain` is tool amplitude over setpoint amplitude. The gain above 1 at 20.6 Hz is the frame mode, which the tool position counts. Options: `--from HZ` (1), `--to HZ` (200), `--count N` (40), `--amplitude M` (0.0005 m). Frequencies have to be above zero here too.

**The mechanics without control:**

```text
$ sim-cli sweep single-axis --from 20 --to 200 --count 12
frequency_hz,gain_rad_s_per_nm,phase_deg
20.0000,2.376702e1,34.641
...
131.5866,5.087283e1,-29.235
```

A sine torque on one motor on top of the holding torques, measured at that motor's speed. Belt resonances show as peaks; the load swinging against the held motor shows as a dip. Options: `--motor N` (index, 0 is the first), `--from` (5), `--to` (500), `--count` (60), `--amplitude NM` (0.05). Frequencies have to be above zero: `--from 0` is refused (`Configuration value is out of range: frequency`). With friction on, a small amplitude mostly measures stiction; the tests use 1.5 N m to find the belt dip.

**The notch filter:** `--set effects.notch_filter=true`, with `drive.notch_frequency_hz` and `drive.notch_bandwidth_hz`. It acts on the torque setpoint.

**Input shaping.** A ZVD shaper spreads every setpoint over three steps half a vibration period apart, so that the swings they start cancel:

```text
--set effects.input_shaping=true --set profile.shaper_frequency_hz=25 --set profile.shaper_damping=0.03
```

Set the frequency to the resonance of the tool. On the T-gantry that changes with the Z height, so `--set effects.shaper_follows_pose=true` lets the shaper take the frequency from the machine at the planned pose. The machine reports the lower mode of carriage-on-belt and Z-axis-on-guide (24.6 Hz at z = 0.05 m and 35.6 Hz at z = 0.15 m for the default config at x = 0.9 m). The single axis reports no resonance, so there the shaper stays at the fixed frequency.

What it does on the EXCT-30 cycle. The config has the shaper on, at a fixed 25 Hz; the first run switches it off:

```text
$ sim-cli analyze t-gantry --config crates/machine-t-gantry/exct-30.toml --set effects.input_shaping=false
motor,cycle_s,...,path_error_peak_m,...,dwell_error_peak_m
a,1.852000,...,5.979e-4,...,7.036e-5

$ sim-cli analyze t-gantry --config crates/machine-t-gantry/exct-30.toml
a,1.934000,...,5.580e-4,...,3.505e-5
```

With the shaper the tool is 0.035 mm off its target in the dwell instead of 0.070 mm.

The tool swing after a traverse is the signal `axis.x.tool_deflection` of `run`.

Limits for this task:

- The step is 1 mm. With friction on it also shows stiction.
- The shaper handles one resonance. The frame mode and the stiffness of the control loops are not in the frequency the machine reports.
- Each move's setpoint ends one vibration period later. Where the cycle dwells or the gripper acts, both wait for the shaped setpoint, so the cycle above is 82 ms longer: two dwells, 41 ms each. Segments without a dwell and without a gripper action go on with the plan.
- Most of the path error on the EXCT-30 is the frame giving way under the acceleration forces (0.60 mm peak without the shaper, 0.56 mm with it), which a shaper set to the tool's resonance does little about.
- No real drive has been compared. Drive vendors do not document their internal filters, and the delays here are whole cycles without jitter.
- A gain that is fine at the nominal values can be unstable when rotor inertia, belt stiffness or inductance are off. That was found on the EXCT-30 config, whose velocity gain was lowered from 1.4 to 0.6 for it. Run `sensitivity` after tuning.

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
m,2.290489e-2,7.933285e-2,1.856409e-4,1.384926e-3
```

Here the "measured" machine is the simulator with a carriage 1.5 kg heavier than the config. The table gives, per motor, the RMS and the peak of the difference between measured and simulated torque and following error. `--out` writes both side by side, one line per row of the trace:

```text
time,motor.m.torque_measured,motor.m.torque_simulated,motor.m.speed_measured,motor.m.speed_simulated,motor.m.following_error_measured,motor.m.following_error_simulated
```

**Fit, stage 1: rigid values from a trace.**

```text
$ sim-cli fit single-axis --trace measured.csv --set effects.payload=false
quantity,of,config,fitted,standard_error
moved_mass_kg,x,6.546751e0,8.060483e0,2.277e-2
viscous_friction_n_s_m,x,2.697711e1,2.633275e1,3.071e-1
coulomb_friction_n,x,2.117146e1,2.185140e1,2.736e-1
constant_force_n,x,3.237100e-3,9.376128e-3,8.854e-2
Moved mass, axis x: +1.514 kg ± 0.023. Add this to the masses that move with the axis. The trace cannot tell load mass from rotor and pulley inertia.
Viscous friction, axis x: × 0.976 ± 0.011. Scale the viscous friction of the axis's guide and of the motor shafts by this factor; the trace cannot tell the two apart.
Coulomb friction, axis x: × 1.032 ± 0.013. Scale the Coulomb friction and stiction of the axis's guide and of the motor shafts by this factor; the trace cannot tell the two apart.
Constant force, axis x: +0.006 N ± 0.089. The measured axis needs this much more than the config gives: a weight, an incline or a cable pull the config lacks.
```

Per axis, a least-squares fit of force = mass × acceleration + viscous × speed + Coulomb × sign(speed) + constant. The same estimate is run on the trace (`fitted`) and on the config's own simulation of the same setpoints (`config`). The difference between the two is what the config is off by: here +1.514 kg, for a true 1.5 kg. The advice lines go to the error stream. No config is edited.

**Fit, stage 2: belt stiffness from a frequency sweep.**

```text
$ sim-cli sweep single-axis --from 50 --to 130 --count 30 --amplitude 1.5 --set belt.stiffness_ea_n=3.5e5 --out sweep.csv
$ sim-cli fit single-axis --sweep sweep.csv --amplitude 1.5
quantity,of,config,fitted,standard_error
resonance_hz,m,8.452246e1,7.242377e1,1.285e0
resonance_damping,m,7.932269e-2,7.912026e-2,
Stiffness between motor and load: × 0.734 ± 0.026. Scale the belt stiffness by this factor. It holds only once the masses are right, so correct those first and sweep again.
```

The sweep file needs the columns `frequency_hz` and `gain_rad_s_per_nm` (motor torque to motor speed). `fit` runs the same sweep on the config, at the frequencies of the file, on motor `--sweep-motor N` (default 0) with `--amplitude NM` (default 0.05), and compares where the dip is. The "measured" belt here has 0.7 times the stiffness; the fit says 0.734 ± 0.026.

What the fit can and cannot recover:

| Recovered | Not recovered, or not told apart |
| --- | --- |
| Moved mass per axis (rotors and pulleys counted as mass at the axis) | Load mass from rotor and pulley inertia |
| Viscous friction per axis | Guide friction from motor shaft friction |
| Coulomb friction per axis | The belt's damping from friction (the damping of the dip includes friction and the gearbox) |
| Constant force per axis (weight, incline, cable pull) | Stiffness from a move's trace: the loops and friction damp the ringing, a sweep is required |
| One factor on the stiffness between motor and load, from a sweep | Which spring between motor and load is off: the factor counts all of them |
| | Drive delays and filters (stage 3 is not built) |
| | Backlash, tooth stiffness, frame and Z-axis stiffness |

Limits for this task:

- **The gripper.** A trace does not say when a payload comes and goes, so a cycle trace is replayed with the payload as the config starts it. A cycle trace of a machine that grips, fitted against a config with the payload on, comes out wrong: the single axis's own default cycle fits as −0.837 kg. Record without payload, or switch it off as above.
- `start.pose` of the config has to be where the trace starts.
- An axis that the trace does not move in both directions is not fitted (`Axis x: not fitted. The trace has to move it in both directions.`). One direction cannot tell Coulomb friction from a constant force.
- A wrong friction shows as a few percent of wrong mass and the other way round, because of gearbox losses. Change the config and fit again.
- The stiffness factor holds only once the masses are right.
- A sweep with a small amplitude finds no dip when friction is on. The same sweep at the default 0.05 N m gives `No stiffness fitted: the measured sweep or the config's own shows no dip with both flanks inside the swept range.` That is a failure like any other: `fit` ends with exit code 2, after writing what a trace gave. The sweep also has to contain the dip and both its flanks. Its frequencies have to be above zero.
- All of this was tested on traces the simulator made, with added noise. No trace of a real drive has been read. Real exports will need their columns renamed and their units converted to rad, rad/s and N m at the motor shaft.

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

**Drives off.** With `--disable-at` the drives give no torque from that time. The motor holding brakes engage (`effects.holding_brake`, on both machines, in the elastic model only):

```text
$ sim-cli run t-gantry --move 0.5,0.2 --time 1.5 --disable-at 0.5 --out disable.csv
```

In that run the Z axis is at 0.2276 m at 0.498 s, moving, and rests at 0.2192 m at 1.498 s: a brake that engages at speed slips with its torque (`motor.brake_torque_nm`) and no more until the axis stands. With `--set effects.holding_brake=false` it falls to the lower hard stop (z = −0.00008 m at 1.498 s). A move or cycle under way is dropped, not resumed.

## Connect a PLC over OPC UA

```sh
cargo run --release -p sim-opcua -- t-gantry --port 4840 --lang en
```

This prints `OPC UA server listening at opc.tcp://127.0.0.1:4840/` and runs the machine in real time until stopped. The machine name, `--port` (default 4840) and `--lang` are the only arguments. A machine the simulator does not have, a port or language it cannot read and any other word are refused with a message and exit code 2.

| Property | Value |
| --- | --- |
| Endpoint | `opc.tcp://127.0.0.1:PORT/` |
| Security | None. Anonymous access |
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
| `status.busy` | Boolean | read | A move or the cycle is under way |
| `status.refused` | String | read | Key of the last refused command, or empty |

How commands behave:

- The servo loops stay inside the simulator. The PLC sends targets or position setpoints, not torque.
- Numbers may be written as Double, Float, Int16, Int32 or Int64. Anything else is answered with `BadTypeMismatch`.
- Setpoints of several axes written within the same 10 ms are taken together as one setpoint.
- An axis that is not written stays where it is. `command.target.<axis id>` reads the planned position until it is written, and again once the move it was written for has been commanded. `command.setpoint.<axis id>` reads the setpoint the controller follows, which is the planned position whenever no setpoint is on its way. So after a move or a cycle, a write to one axis does not send the others back to where they were before.
- `status.refused` holds a key such as `error-busy`, `error-target-out-of-range`, `error-setpoint-too-far` or `error-disabled`. It stays until a command of the same kind (move, setpoint, cycle start, stop, disable, enable) is accepted. Accepted commands of another kind, such as a stream of setpoints, leave it.
- A streamed setpoint is refused while a move or the cycle runs.
- A setpoint further from the last one (or, with none on its way, from where the axis stands) than the axis travels at its speed limit in one `profile.setpoint_interval_s` (8 mm for the T-gantry's X axis) is refused with `error-setpoint-too-far` and not carried out. A PLC that has fallen further behind catches up with a move. A stream of one setpoint per interval is followed at its own speed, up to the speed limit; a setpoint that comes early finds the axis short of the last one and is followed that much faster.
- If the host cannot keep up with real time, the simulation falls behind instead of piling up work.

Limits for this task:

- **Not tested against a real PLC.** The client in the tests is written with the same library as the server. Interoperability with Siemens, Beckhoff or CODESYS clients, their data types, security settings and cycle jitter are untested.
- The server binds to 127.0.0.1 only and serves the machine's default config only. There is no `--config` and no `--set`.
- No security profiles, no user accounts, no methods, no config changes, no paths over OPC UA.
- The server writes its keys into a folder `pki` in the working directory.

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
| Analysis: Analyse and judge the cycle | The cycle analysis and the verdicts, as three tables: the figures of the machine (cycle time, path error, off target while dwelling, lowest belt tension, slack time, radial load, guide load, guide life), the load on each motor (peak, RMS and cubic-mean torque, winding rise, energy drawn and fed back, the braking resistor's energy and mean power, highest DC link voltage) and one row per check |
| Analysis: with error bands, samples | Ticked, "Analyse and judge the cycle" also runs the cycle with every value of `[bounds]` at its low and its high bound (two more cycles per value) and adds a fourth table: per check the lowest and the highest value, the worst-case result and "Runs passed [%]". The same as `sim-cli verdicts --bands`. With a number above 0 in the field "samples" beside the tick box, the cycle is instead run that often with all values of `[bounds]` between their bounds at once, and the table is headed "The checks with all uncertain values between their bounds at once". The same as `--bands --samples N`; the app always uses seed 1 |
| Analysis: Sensitivity | The eight parameters with `[bounds]` that the cycle depends on most |
| Analysis: Optimise the cycle | The optimiser. "Apply these settings to the running machine" puts its result into the config |
| Motor search | Supply (230v or 400v), how the article numbers start (empty: every motor of the catalogue, which takes minutes) and optionally a gearbox series such as `NRK` or `NRK070`. "Search" runs every such motor on the cycle and lists them, passing ones first, lightest first, with the check nearest to its limit or the reason a motor was not run. "Use" in a row names that motor, its supply and its gearbox in `[parts]` of the running config. The same as `sim-cli search` |
| Path | A file name, typed in. "Lines, arcs and splines" is a path file, planned within the axis limits or, ticked, "As fast as the motors allow"; "Poses at times" is a trajectory file. "Plan, run and analyse" plans the path from where the machine stands, analyses it on a simulation of its own (duration, deviation by transition curves, path error, distance behind the setpoint, loads, checks), draws the planned path as a plot in the panel and as a line in the 3D view, and has the machine in the 3D view run it. The same as `sim-cli analyze --path` or `--trajectory` |
| Trace | A file name, typed in. "Replay and fit" replays the drive trace and shows per motor how far the simulation is from it (torque and following error, RMS and peak), and per axis the fitted mass, friction and constant force next to the config's, with what to change in the config. The same as `sim-cli replay` and `fit --trace`. The config's start pose has to be where the trace starts |
| Tuning: Measure step and frequency response | Step response and closed-loop response of one axis, with rise time, overshoot, settling time and bandwidth |
| Scope plots | One plot per quantity, one line per axis or motor. Setpoints are dashed in the colour of their actual value. Shared time axis and cursor; min, max, span and RMS per trace; units switch to mm and µm by themselves |

The right panel scrolls as a whole, and each of its parts (Scopes, Analysis, Motor search, Path, Trace, Tuning) folds away under its heading. A part opens by itself when its result arrives. The tables are laid out for the panel's default width: long names wrap, and numbers have four digits.

The buttons of these parts run in the background on simulations of their own, with the config as it is at that moment; a spinner stands in for the button meanwhile. The running machine is not disturbed. A result is not redone when a parameter changes afterwards. It is marked "computed for an earlier configuration" instead, and nothing of it is applied to the changed config: "Apply these settings", "Use" and the run of a planned path answer with a message then (take the change back, or compute again). Naming a motor with "Use" does not outdate the search it came from, so another row can be tried after it.

A file that is missing or cannot be read, a line of it that cannot be read, a supply other than 230v and 400v and a search that finds no motor, or no gearbox of the series that fits one, are answered under Messages. So is a path that the machine cannot run because it has moved since the plan was made, or is busy.

A drive that trips on DC link overvoltage is reported under Messages, once per trip, for the running machine and for an analysed cycle or path. "Enable drives" brings the drives back.

What the running machine does with a change:

- A changed `gearbox.ratio` or `geometry.pulley_radius_m` rebuilds the machine's state at the tool's pose and speed, with belts and gears as they are at rest there. The belt tensions stay within some percent of what they were.
- A change to the input shaper (its switch, frequency, damping, or following the pose) is taken when the planned motion next stands still, so the setpoint does not jump. A cycle without a dwell and without a gripper action never stands still; there the change waits for Stop.
- A cycle that loses segments while it runs goes on within the segments it has.

Messages: one that only repeats the last one is not added. If the simulation or a background computation ends unexpectedly, a message says so; choosing the machine again starts the simulation anew. If the window stalls for more than ten seconds of simulated time, the scopes lose the samples before that.

Limits for this task:

- The app starts with the default configs only. It cannot load a config file, so the EXCT-30 config is not available in it.
- Parts by name are set by the motor search only; there is no field to type one in. A number of a named part that is changed in the parameter panel is overwritten again.
- The cycle itself cannot be edited in the app.
- Not in the app, only on the command line: the frequency sweep of the mechanics, the fit of the belt stiffness from a sweep (`fit --sweep`), the overlay file of a replay (`replay --out`), the operating points (`analyze --points`), and a sensitivity run over tables without `[bounds]`.
- The search table shows the tightest check only; peak torque and winding rise of a candidate are in `sim-cli search`, or in the cycle analysis after "Use".
- File names are typed; there is no file dialog. A relative name is relative to the directory the app was started in.
- The lines of the planned and the travelled path in the 3D view were not checked on screen by a person.
- A path is analysed from where the machine stands when the button is pressed. A machine that is moving then will refuse the run.
- The base time step `sim.dt_s` cannot be changed while running.
- The mouse handling was not checked by a person. Screenshots of the window in both languages were.
- Desktop only.
