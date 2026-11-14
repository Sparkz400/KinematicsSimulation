# Config reference

One TOML file per machine. Units are in the key names. The key list below was generated from the three shipped configs, so every key they have is here:

| Short | File | Machine |
| --- | --- | --- |
| T | [`crates/machine-t-gantry/default.toml`](../crates/machine-t-gantry/default.toml) | T-gantry, made-up mid-size machine. Placeholder values |
| E | [`crates/machine-t-gantry/exct-30.toml`](../crates/machine-t-gantry/exct-30.toml) | T-gantry: Festo EXCT-30 with Siemens 1FK2205-4AF. Each value marked `[catalogue]` or `[estimate]` in the file |
| S | [`crates/machine-single-axis/default.toml`](../crates/machine-single-axis/default.toml) | Single belt axis. Placeholder values |
| D | [`crates/machine-delta-d3/default.toml`](../crates/machine-delta-d3/default.toml) | Delta picker D3, sized like the Autonox RL3-1200-3kg with Siemens 1FT2105-4AF (the 1FK2105-4AF Autonox names, as a 1FT2 for the SP100 precision gearbox). Each value marked `[catalogue]` or `[estimate]` in the file. Its keys are in [their own section](#delta-picker-d3) |
| D2 | [`crates/machine-delta-d2/default.toml`](../crates/machine-delta-d2/default.toml) | Planar delta picker D2, sized like the Codian D2-800-S020 with Siemens 1FT2105-4AF and the SP100 gearbox. Each value marked `[catalogue]` or `[estimate]` in the file. Its keys are in [their own section](#delta-picker-d2) |
| Parallel SCARA | [`crates/machine-parallel-scara/default.toml`](../crates/machine-parallel-scara/default.toml) | Parallel SCARA: the D2's five-bar in the horizontal plane with a Z stroke on a ball screw and a tool turn at the plate. Each value marked `[catalogue]` or `[estimate]` in the file. Its keys are in [their own section](#parallel-scara) |
| C | [`crates/machine-delta-d3/conveyor.toml`](../crates/machine-delta-d3/conveyor.toml) | The D3's default with a [`[conveyor]`](#conveyor) table: it picks from a belt conveyor instead of running the Adept cycle. So do the conveyor examples beside the T-gantry's, the D2's, the D4's and the linear delta's defaults (`crates/machine-t-gantry/conveyor.toml`, `crates/machine-delta-d2/conveyor.toml`, `crates/machine-delta-d4/conveyor.toml`, `crates/machine-delta-linear/conveyor.toml`) |
| D4 | [`crates/machine-delta-d4/default.toml`](../crates/machine-delta-d4/default.toml) | Four-axis delta picker D4: the D3 with a turning tool on a telescopic shaft, sized after the ABB IRB 360 with its fourth axis. Each value marked `[catalogue]` or `[estimate]` in the file. Its keys are in [their own section](#delta-picker-d4) |
| H4 | [`crates/machine-delta-h4/default.toml`](../crates/machine-delta-h4/default.toml) | Quattro-type four-arm delta H4: four arms at 90°, a plate in two parts that slide against each other and turn the tool through a rack, a pinion and a gear stage; sized near the Omron Quattro s650H class (1300 mm across, 6 kg) with Siemens 1FK2105-6AF. Each value marked `[catalogue]` or `[estimate]` in the file. Its keys are in [their own section](#quattro-type-delta-h4) |

| L | [`crates/machine-delta-linear/default.toml`](../crates/machine-delta-linear/default.toml) | Linear delta, sized like the Festo EXPT-95 with Siemens 1FK2205-2AF. Each value marked `[catalogue]` or `[estimate]` in the file. Its keys are in [their own section](#linear-delta) |
| H | [`crates/machine-hexapod/default.toml`](../crates/machine-hexapod/default.toml) | Hexapod on six electric cylinders, generic proportions with Siemens 1FK2105-4AF. Each value marked `[catalogue]` or `[estimate]` in the file. Its keys are in [their own section](#hexapod) |
| CP, CS | [`crates/machine-cable-robot/config/`](../crates/machine-cable-robot/config/) | Planar (4 cables) and spatial (8 cables) cable robots, generic proportions with Siemens 1FK2105-4AF. Each value marked `[catalogue]` or `[estimate]` in the file. Their keys are in [their own section](#cable-robots) |

The **In** column says which configs have the key. "T E" means the key exists on the T-gantry only; "S" on the single axis only; "E" in the EXCT-30 config only. A key marked for one machine is not read by the other.

The **Source** column says where a value of this kind normally comes from:

- *datasheet*: published by the vendor of the part
- *estimate*: not published; a guess of the right order, to be bounded in `[bounds]` and replaced by a measurement
- *measurement*: can only be had from the machine
- *choice*: a setting, not a property of the machine

For the EXCT-30 config the file itself is the authority: its `[catalogue]` and `[estimate]` comments say for each value which it is.

## Rules

- `format_version = 1` is required. Any other value is refused.
- A config has to be complete. There are no built-in defaults for a missing key: a key that the machine reads and the file lacks is refused (`Configuration value is missing: ...`). Only `[ratings]` entries, `[parts]`, `[bounds]`, `obstacles`, `[conveyor]` and `drive.braking_module` (off) may be left out, and `effects.thermal_warm_start` and `effects.drive_current_limit` (off: a config written before 13 November 2026), and `effects.field_weakening`, `drive_overload`, `motor_thermal_nodes`, `iron_loss`, `magnet_temperature` and `braking_resistor_heating` (off: a config written before 14 November 2026), whose keys are read only with them on, and `effects.drive_monitoring` (off) with the `[drive]` keys from `fault_reaction` to `torque_band` (the values of the shipped files).
- A config file (`--config`) may not have a key that the machine's default config lacks: it is refused by name (`Configuration has a value this machine does not know: ...`), so a misspelt key cannot pass for the default's value. `[bounds]` has its own check, and `ratings.pulley_max_torque_nm` is the one rating no default config has, and `[conveyor]` the table none has. A name in `[parts]` has to be text.
- `--set path=value` and the app's parameter panel can only change values that exist, and cannot change their type. An integer is accepted where a float is expected.
- Most physical values must be strictly positive (masses, radii, stiffnesses, cycle times, friction levels). A zero there is refused as out of range.
- Values that may be zero must not be negative: the payload, dampings, viscous and shaft friction, backlash, belt pretension and belt mass, the gearbox's inertia, idler and cable chain values, feedforward factors, the blend time, the path tolerance, the start temperature rise.
- `gearbox.efficiency`, `profile.torque_reserve` and `profile.tension_reserve` are above 0 and at most 1; `motor.ripple_fraction` and `profile.shaper_damping` are from 0 to below 1; `drive.no_load_speed_rad_s` is above `drive.rated_speed_rad_s`.
- Free in sign: forces (`process.*`, `belt.tooth_force_n`, `motor.cogging_nm`), `geometry.incline_deg`, the two drops of the Z axis, the ripple and cogging orders.
- NaN and infinity are refused for every number. `start.pose` has one position per axis within its range, and a cycle segment's `dwell_s` is finite and not negative.
- Every refusal names the key (`Configuration value is out of range: gearbox.backlash_rad`). Most values are checked whether or not their effect is switched on.
- Loop cycle times must be whole multiples of `sim.dt_s`, and `drive.current_cycle_s` must equal it. `drive.position_cycle_s` may equal `velocity_cycle_s` (DSC) and `profile.cycle_s` may be longer than it: with `effects.fine_interpolation` the drive interpolates between profile cycles.
- A changed config can be applied to a running simulation (the app does), except `sim.dt_s`. That includes `profile.cycle_s` while a motion is under way: the controller counts its own profile cycles and converts what it has stored in them, so the setpoint goes on from where it is. Switching `effects.belt_stretch` while an axis moves carries the tool's position and speed over into the other model. So does a changed `gearbox.ratio` or `geometry.pulley_radius_m`: the state is rebuilt at the tool's pose and speed, with belts and gears as they are at rest there. A changed input shaper (switch, frequency, damping, following the pose) is taken when the plan next stands still. A cycle that loses segments while it runs goes on within those it has, and ends if none is left.

## Top level

| Key | In | Unit | Meaning |
| --- | --- | --- | --- |
| `format_version` | T E S | – | Format of the file. Must be 1 |
| `obstacles` | T E S D | – | Obstacles in the work area: a list of boxes with their edges along the axes of the 3D view (scene coordinates, m: x right, y into the depth, z up), each a table with `name` (text), `centre_m` and `size_m` (three numbers each, the sizes above 0), and optionally its contact `stiffness_n_m` (above 0, [estimate] 1e7: a steel fixture on a machine bed, some 10 N/µm; [bounds] 1e6 to 1e8) and `damping_n_s_m` (not negative, [estimate] 1e4: near critical for a few kilograms), which push back on a part in it with `effects.crash`; another key in an entry is refused. `[]` in all four files: no obstacles, and nothing changes. Write `obstacles = [{ name = "fixture", centre_m = [...], size_m = [...] }]`, or `[[obstacles]]` tables at the end of a file without the `obstacles = []` line. A config without the key has none. The tool is where the 3D view draws it: T and E at (`geometry.span_min_m` + x, −0.05, z − `stroke_z_m` − `span_min_m` − 0.042), S at (`span_min_m` + x, −0.03, 0.06), D at (x, y, z). What can hit an obstacle, the clearance kept (`profile.min_clearance_m`) and the crash are in the [model reference](model-reference.md#obstacles-and-crash) |

## `[sim]`

| Key | In | Unit | Meaning | Source |
| --- | --- | --- | --- | --- |
| `dt_s` | T E S | s | Base time step. Also the current-loop cycle. 62.5 µs in all configs | choice |
| `substeps` | T E S | – | RK4 sub-steps per base step. A whole number. 4 in all configs | choice |

## `[parts]`

Parts by name from the catalogue. The catalogue values of a named part replace the numbers the file has for it when the config is loaded. `""` names none. What each part sets is listed in the [user guide](user-guide.md#check-a-real-catalogue-machine).

| Key | In | Meaning |
| --- | --- | --- |
| `motor` | T E S | Article number of a motor, e.g. `"1FK2205-4AF"`: for `"230v"` and `"400v"` one of [`catalog/siemens/motors_s210.csv`](../catalog/siemens/motors_s210.csv); for `"s120"` a 1FK7 or 1FT7 of `motors_s120.csv` (with its data sheet in `motors_s120_electrical.csv`) or a 1FK2 or 1FT2 of `motors_1fk2_s120.csv`, `motors_1ft2_s120.csv`. E names its motor; T and S name none |
| `motor_supply` | T E S | `"230v"` or `"400v"`: the S210 data at that supply; `"s120"`: the motor on a SINAMICS S120 at 400 V (DC link 540 V); `"s210new-230v"`, `"s210new-400v"`: the S210 data with the new 6SL5310 converters; `"<vendor>-<volts>v"` another maker's motors and drives at that line voltage (`beckhoff-400v`, `beckhoff-480v`, `br-400v`, `kollmorgen-240v`, `kollmorgen-400v`, `lenze-400v`, `mitsubishi-200v`, `mitsubishi-400v`, `rexroth-400v`, `sew-400v`, `yaskawa-200v`, `yaskawa-400v`; DC link 1.35 times the line voltage). See [the catalogue](catalogue.md#other-makers). `"400v"` in all configs |
| `motor_brake` | T E S | `true`: the motor with its holding brake, the catalogue's inertia and weight with brake (refused for a motor without brake data). `false` in all configs; may be left out (false) |
| `motor_encoder` | T E S | The encoder ordered with the motor (`encoders_1fk2_1ft2.csv`): `"AS22DQC"` or `"AM22DQC"` (22 bit), for a 1FT2 also `"AS26DQC"` or `"AM26DQC"` (26 bit); sets `drive.encoder_counts_per_rev`. `""` keeps the config's; may be left out |
| `gearbox` | T E S | A gearbox as size-frame-ratio, e.g. `"NRK070-04-5"`, from the three `gearbox_*.csv` files. None named in any config |
| `drive` | T E S | Article number of an S210 converter ([`converters_s210.csv`](../catalog/siemens/converters_s210.csv)) or an S120 Motor Module ([`motor_modules_s120.csv`](../catalog/siemens/motor_modules_s120.csv)). Sets `ratings.drive_rated_current_a` and `drive_max_current_a`. An S210 converter also sets `dc_link_overvoltage_v` and, unless it is a 3 AC converter on the `"230v"` supply (external resistor required), `braking_resistor_continuous_w` and `_peak_w` ([`braking_s210.csv`](../catalog/siemens/braking_s210.csv)); a `braking_threshold_v` at or above its trip level becomes the midpoint between its highest line peak and the trip. An S120 Motor Module sets `dc_link_capacitance_uf` (half per axis for a Double). E names its converter, `"6SL3210-5HE11-5UF0"`; T and S name none |
| `belt` | T E S | A belt as profile-width, e.g. `"AT5-25"`, from [`megadyne/belts.csv`](../catalog/megadyne/belts.csv) (T5, AT3, AT5, AT10, MTD3, MTD5, MTD8; a width between the standard ones is interpolated). Sets `belt.stiffness_ea_n`, `belt.mass_kg_m`, `belt.tooth_pitch_m` and, where the config has them, `ratings.belt_max_tension_n`, `belt_width_mm` and the tooth load table. None named in any config |
| `axis` | T E S | An axis by name: `"EXCT-15"`, `"EXCT-30"`, `"EXCT-100"` ([`festo/belt_axes.csv`](../catalog/festo/belt_axes.csv)) or a Bahr axis as series-size, e.g. `"ELZI-40"`, `"ELZ-60S"` ([`bahr/belt_axes.csv`](../catalog/bahr/belt_axes.csv); not the XY gantries ELZU and not the DLZPVI). An X/Z gantry needs a config with `geometry.stroke_z_m`, a single axis one with `geometry.stroke_m`. Sets pulley radius, belt, teeth in mesh, the pulley's torque rating and the moved masses; keeps the strokes (an EXCT takes the shortest Z stroke offered that holds the config's) and refuses a stroke the axis does not have. None named in any config: E's numbers are those `"EXCT-30"` gives |

The parts go in in the order axis, belt, gearbox, motor, drive. A part by name other than the drive also retunes `drive.velocity_kp_nm_s_rad`: the gain is scaled by the square root of (ratio of the inertia the motors turn) × (ratio of the rotor inertias, the gearbox's own included), new over old. The reference is the config's own numbers as the file has them before the part goes in, so a config states its gain for the rotor, gearbox and masses it has as numbers, whichever part it names. A motor by name sets `drive.bus_voltage_v` to the DC link voltage of its supply and scales `ratings.braking_threshold_v` and `ratings.dc_link_overvoltage_v` with it. The gearbox is put in before the motor, so the motor's peak torque is capped with the ratio in effect.

`[parts]` is not shown in the app's parameter panel, which shows numbers only; the app's part fields and the "Use" button of a row of its motor search set it. A gain edited there is the gain for the config's own numbers; with a part named, the simulation runs with the retuned value.

## `[effects]`

One switch per effect. "Default" is the value in T, E and S; where they differ it says so. Keys marked "T E" do not exist in the single-axis config.

**Mechanics**

| Switch | In | Default | What it models |
| --- | --- | --- | --- |
| `gravity` | T E S | on | Gravity on the Z axis and payload (T-gantry) or along an inclined axis (single axis) |
| `payload` | T E S | on | The payload mass, attached and released by the cycle. Off: no payload at all |
| `payload_swing` | arms | off | The payload hangs on a pivot at the flange and swings (`arm.payload_swing_*`): two joints no motor drives, their damper's loss `power.payload_swing`, their angles `swing.1.angle`, `swing.2.angle` |
| `belt_stretch` | T E S | on | The elastic model: belt sections as springs, pulleys and motor shafts as separate states. **Off selects the rigid reference model**, in which belt damping, slack, tooth stiffness, backlash, gear stiffness, the frame mode, the leaning Z axis and the holding brakes have no effect |
| `belt_damping` | T E S | on | Damping on the stretch rate of each belt section |
| `belt_slack` | T E S | on | Belt tension cannot go below zero |
| `belt_mass` | T E S | on | Belt mass, lumped on the pulleys |
| `belt_tooth_stiffness` | T E S | off | The teeth in mesh on a driven pulley give way, as a spring in series with each belt section |
| `belt_material_damping` | T E S | off | Damping in proportion to a section's stiffness (`belt.damping_time_s`) instead of one constant. Needs `belt_damping` on |
| `guide_friction` | T E S | on | LuGre friction in the linear guides; in a machine file also a screw nut's preload drag, a linear motor's attraction on its guide and a rotary's bearing friction (`friction_coulomb_nm`, `friction_viscous_nm_s_rad` of its node, see [machine files](machine-files.md#screws-racks-linear-motors-and-rotary-tables)) |
| `load_dependent_friction` | T E S | on | Friction levels scale with the guide load |
| `motor_friction` | T E S | on | Coulomb and viscous friction on the motor shafts |
| `gearbox_backlash` | T S on, E off | | Backlash as a dead zone. Off in E: direct drive |
| `gearbox_efficiency` | T S on, E off | | Torque loss in the direction of power flow. Off in E: direct drive |
| `gearbox_drag` | T E S D | off; on in the delta pickers D2 and D3 (and the planar delta file) | The gearbox's no-load running torque at its input over speed and temperature (`gearbox.no_load_*`, Palmgren's (ν n)^(2/3)); `gearbox.efficiency` then holds at `gearbox.efficiency_torque_nm`, and the efficiency falls at part load, slow and cold (see the [model reference](model-reference.md#gearbox)). Never on a coupling (ratio 1) |
| `motor_stiction` | T E S D | off; on in D2 and D3 | The motor shaft breaks away at `motor.friction_breakaway_nm` and falls to its Coulomb level along a Stribeck curve. Needs `motor_friction` |
| `gearbox_lost_motion` | T E S D | off; on in D2 and D3 | The flanks take the load over `gearbox.lost_motion_band_rad` beyond the backlash, the stiffness rising from 0, and a hysteresis `gearbox.hysteresis_rad` (a Dahl friction state per gearbox: see the model reference) |
| `gearbox_stiffness_curve` | T E S D | off | A strain wave gear's three-segment stiffness, K1 (`stiffness_nm_rad`) to T1, K2 to T2, K3 beyond (`gearbox.stiffness_*`), as Harmonic Drive gives it |
| `torque_ripple` | T E S | on | Torque ripple (proportional to torque) and cogging (independent of it) |
| `tooth_meshing` | T E S | on | A small periodic force at the tooth-pass frequency |
| `frame_compliance` | T E | on | One spring-mass mode of the frame in X: the portal's posts swaying under its beam, coupled to the belt both ways |
| `base_compliance` | every machine and machine file | off | The base the whole machine stands on gives way ([`[base]`](#base)): up to six spring-mass directions under the motors' reactions and the moving masses, the tool moving with it. In series with `frame_compliance` |
| `z_axis_bending` | T E | off | The Z axis bends as a cantilever from the carriage (`tool_flex.z_axis_bending_stiffness_nm2`), in series with its guide's give; with `tool_flex` off it only bends. Elastic model |
| `beam_sag` | T E | off | The X beam sags under the carriage and the Z axis as a simple span ([`[beam]`](#beam)), quasi-static, lowering the tool. Elastic model |
| `tool_flex` | T E | on | The Z axis leans against the carriage; the reported X position is the tool tip. In a machine file the body its `machine.lean` node names leans (elastic model), off in `machines/t-gantry.toml` as compared with its crate |
| `idlers` | T E | off | Inertia and bearing friction of the five idlers |
| `gantry_yaw` | T E | off | Read by machine files only: the bridge of a `machine.gantry` node yaws, one more coordinate held by its drives, belts and joints' spring (values in `[gantry]`, see [machine files](machine-files.md#a-gantry-that-yaws-and-tools-on-rotary-axes)); signals `gantry.<id>.yaw`, `.sync_error`, `.moment`, check `gantry-sync`. On in `machines/dual-gantry.toml`, off in the H-bot, CoreXY and rack gantry files, so that earlier results stay as they were; the crates have no such bridge |
| `cable_chain` | T E S | off | A cable chain whose moving mass grows with x, plus drag. On an inclined single axis the moving run is lifted too |
| `process_force` | T E S | off | A force on the tool (single axis: the carriage) while the cycle dwells |
| `hard_stops` | T E S | on | Spring-damper stops beyond both ends of each stroke |
| `holding_brake` | T E S | on | Motor holding brakes that act while the drives are disabled, in the elastic and in the rigid model |
| `temperature` | T E S | off | Scale factors on friction and belt stiffness, set by hand in `[temperature]`; not used while `thermal_feedback` is on |

**Drive and control**

| Switch | In | Default | What it models |
| --- | --- | --- | --- |
| `velocity_feedforward` | T E S | on | Speed setpoint added to the position loop's output |
| `torque_feedforward` | T E S | on | Rigid-model torque (inertia and gravity, no friction) added to the velocity loop's output |
| `fine_interpolation` | T E S D | on | Between profile cycles the drive runs the position setpoint on from the last setpoints as pos + v t + a t²/2, as it runs the speed feedforward on as v + a t: a SINAMICS drive's fine interpolation, which the position loop needs when it runs faster than the profile (DSC: `drive.position_cycle_s` down to `velocity_cycle_s`, `profile.cycle_s` the controller's IPO cycle of 2 to 8 ms). Off, the setpoint is held for the profile cycle as a position loop closed in the controller sees it, and its steps reach the torque (4 ms IPO: single axis 2.1x the peak torque). On in the shipped configs, where it changes nothing: their position and profile cycles are equal. A config without the switch has it off. See the [model reference](model-reference.md#fine-interpolation-and-precontrol) |
| `friction_feedforward` | T E S D | off | The drive adds the friction characteristic `drive.friction_feedforward_*` at the speed setpoint to the torque feedforward (S120 p3820-p3839, recorded by the drive at constant speeds, p3845). Off in the shipped configs: none has a recorded characteristic |
| `load_encoder` | T E S D | off | The position loop closes on a load-side scale (S120 direct measuring system), the tool axes measured in steps of `drive.load_encoder_step` and turned into motor angles by the rigid kinematics; the speed loop stays on the motor encoder. The following error is then the load's: belt stretch and gear twist are inside the loop. Off in the shipped configs |
| `encoder_resolution` | T E S | on | The position is measured in whole encoder counts; speed is the difference of positions |
| `velocity_filter` | T E S | on | First-order low-pass on the measured speed |
| `torque_filter` | T E S | on | First-order low-pass on the torque setpoint |
| `notch_filter` | T E S | off | A notch on the torque setpoint |
| `torque_limit` | T E S | on | The torque setpoint is limited to the peak torque and the torque-speed curve |
| `drive_current_limit` | T E S D | on | The drive gives no more than its maximum output current `ratings.drive_max_current_a`: the torque setpoint is held to the torque of that current where it is below the motor's limit, with the velocity loop's anti-windup, so a cycle that asks more lags behind (see the [model reference](model-reference.md#drive-current-limit)). Nothing is held without the rating. On in the shipped configs (the joint machines' `common.toml` and the machine files too); it changes nothing there, as every shipped drive gives more than its motor takes. Off, the drive's maximum current is only judged (`drive-peak-current`) |
| `field_weakening` | T E S D | off | The motor as a permanent-magnet synchronous motor in d-q axes (`motor.pole_pairs`, `d_inductance_h`): above base speed the drive sets a negative d current within the voltage and current circles, and the torque-speed curve follows from the model instead of fading between rated and no-load speed (see the [model reference](model-reference.md#field-weakening)). On in the shipped configs since 14 November 2026 (a 1FK2 by name brings an inductance estimated from Siemens' short-circuit braking table; a 1FK7 or 1FT7 its data sheet's). Off, the curve fades linearly between rated and no-load speed, which on the shipped 1FK2205-4AF at 400 V leaves 1.6 N m at 500 rad/s where the d-q model gives 17.8 N m |
| `drive_overload` | T E S D | off | The drive's thermal image (i²t, `ratings.drive_overload_time_constant_s`): once it reaches the level at which the drive has given its maximum current for `drive_max_current_time_s` from the rated load, the drive's current limit falls to its base load current `drive_base_load_current_a` until the image is back at the rated current's; in two nodes since 14 November 2026, the slow heat sink and the fast chips on it (`ratings.drive_chip_share`, `drive_chip_time_constant_s`; a share of 0 is the one node); judged by `drive-overload` (see the [model reference](model-reference.md#drive-overload)). Off in the shipped configs |
| `motor_thermal_nodes` | T E S D | off | With `winding_heating`: the motor's heat in three nodes, winding, stator and housing, surroundings (`motor.winding_resistance_share`, `winding_time_constant_s`); the settled rise `motor-thermal` judges is the winding's and the same as the one node's (see the [model reference](model-reference.md#motor-thermal-nodes)). Off in the shipped configs |
| `iron_loss` | T E S D | off | With `winding_heating`: the stator's iron loss, R (I_0² − I_N²) at the rated speed (what Siemens' rated data leave of the copper loss at the stall current `motor.stall_current_a`), times (speed / rated speed)^1.5, heats the motor (the housing's node with `motor_thermal_nodes`); the copper loss then reaches the rated rise at the stall current, not the rated one. Signal `motor.<id>.iron_loss`, `MotorLoad::iron_loss`. On in the shipped configs (no change where a config's stall current is its rated current) |
| `magnet_temperature` | T E S D | off | With `winding_heating`: the magnets' flux, and so torque constant and back-EMF, change with the motor's temperature by `motor.magnet_temperature_coefficient_1_k` per kelvin from the catalogue's value at the rated rise (a cold motor has about 12 % more). The drive's back-EMF feedforward keeps the catalogue's. Off in the shipped configs |
| `braking_resistor_heating` | T E S D | off | With `dc_link`: the braking resistor's temperature, one heat capacity with `ratings.braking_resistor_time_constant_s` that settles at `braking_resistor_rated_rise_k` at its continuous power; signal `motor.<id>.braking_resistor_temperature`. On in the shipped configs (a signal only; nothing trips on it) |
| `back_emf` | T E S | on | Back-EMF of the turning rotor in the winding |
| `voltage_limit` | T E S | on | The winding voltage is limited to sqrt(3/2) times the DC link voltage |
| `winding_heating` | T E S | on | The winding warms with its copper loss and its resistance rises with it |
| `part_heating` | T E S | off | Each gearbox and guide warms with its loss power, as one thermal mass with the values of `[thermal]`. Only computed and shown (signals `temperature.<part>`, the thermal view, the 3D colours), unless `thermal_feedback` is on |
| `thermal_warm_start` | T E S D | on | The cycle analysis (`analyze`, `verdicts`, the search, the optimiser, the app's analysis) measures its pass with the windings (`winding_heating`) and, with `thermal_feedback`, the gearboxes and guides at the rises they settle at on the cycle, iterated from cold to a fixed point within 0.1 K, at most 4 warm passes; not a physical effect but how the analysis starts (see the [model reference](model-reference.md#thermal-warm-start)). Nothing to do where nothing that warms acts back on the run. On in the shipped configs (the joint machines' `common.toml` and the machine files too), as sizing should judge a machine that has run for a while. Off, the analysis starts from cold |
| `thermal_feedback` | T E S | off | Guide friction and gearbox losses (in a machine file a worm stage's efficiencies too) follow the parts' temperatures, the belt's EA the ambient temperature (`[thermal_feedback]`, see the [model reference](model-reference.md#temperature-feedback)). Warms the parts as `part_heating` does, whether that is on or not. Replaces `temperature`: its hand-set factors are not used while this is on. Off by default, so that earlier results stay as they were |
| `belt_tooth_jump` | T E S | off | Elastic model: the belt's teeth climb out of a driven pulley's grooves when the effective pull there exceeds `belt.tooth_jump_factor` · `ratings.belt_teeth_in_mesh` · pitch / radius times the slack side's tension, and the belt slides over the pulley until it falls into the next groove (a jump) or back. Signals `pulley.<motor>.tooth_jumps`, `.tooth_slip`, `power.tooth_jump`; check `belt-tooth-jump`. Needs `belt_mass`. See the [model reference](model-reference.md#belt-tooth-jump). Off by default |
| `belt_span_vibration` | T E S | off | The first sideways mode of each free belt span, a string at its section's live tension (signals `belt.span.<i>.frequency`), against the tooth mesh and the motors' ripple and cogging (`belt.span_detuning`); check `belt-span-resonance` against `ratings.belt_span_resonance_band` at steady speed. Signals and a check only. Off by default |
| `belt_wrap` | T E S | off | The teeth in mesh on each driven pulley from the angle the belt wraps it at the pose, found from the places of the pulleys and idlers (in a machine file their `at_m` and `clockwise` along the sections' routes, see [machine files](machine-files.md#the-belts-wrap-and-a-leaning-axis); off in `machines/t-gantry.toml`): wrap · r / `belt.tooth_pitch_m`, in place of `ratings.belt_teeth_in_mesh`, in the tooth-jump law (the climb-out limit becomes `belt.tooth_jump_factor` · wrap) and the tooth load (the smallest wrap of the pass). In these machines the spans leave the motor pulleys level at every pose: half a turn, 12.6 teeth (T S) and 10 (E). See the [model reference](model-reference.md#belt-wrap). Off by default, so that earlier results stay as they were |
| `torque_saturation` | T E S | on | The torque constant falls towards the maximum current |
| `drive_delay` | T E S | on | Computation delay of the current and the velocity loop, in whole cycles |
| `dc_link` | T E S | on | A simulated DC link per drive with capacitance, braking chopper, resistor and overvoltage trip, built from `[ratings]`. The ratings of T and S are placeholders. Off, the DC link voltage stays at `drive.bus_voltage_v` and the braking checks are estimates |
| `shared_dc_link` | T E S | off | With `dc_link`: the drives of the machine share one DC link (one Line Module, as on a SINAMICS S120). What a braking motor feeds back a motoring one draws. The link has the capacitance of all drives together (each brings `ratings.dc_link_capacitance_uf`), one chopper with one braking resistor of the `[ratings]` powers, and one trip; every motor reports the same voltage and resistor power. Switched on while the machine runs, links that differ put their energies together. Changes nothing on a machine with one drive. An S120 Motor Module by name switches it on, an S210 converter off |
| `regenerative_supply` | T E S | off | With `dc_link`: a regenerative supply (an Active Line Module, as on a SINAMICS S120). Above the bus level it feeds the DC link's energy back into the mains, up to `ratings.line_module_max_power_w`, of which `ratings.line_module_efficiency` arrives there; the braking resistor takes only what is beyond that power. Reported as energy fed back into the mains and as line module loss (analysis, budget, `analyze` column `mains_energy_j`, the app), and judged by `line-module-continuous` and `line-module-peak`. Off by default, so that earlier results stay as they were |
| `fieldbus_delay` | T E S | off | The motor setpoints reach the drives some profile cycles late |
| `fieldbus_jitter` | T E S | off | Each frame of motor setpoints takes from 0 to `profile.fieldbus_jitter_cycles` more cycles than the delay, evenly and pseudo-random from `drive.noise_seed` (every run the same); the drives use the newest frame that has arrived, so a late frame leaves them the one before. Works with or without `fieldbus_delay`. Off by default, so that earlier results stay as they were |
| `sensor_noise` | T E S | off | Gaussian noise on the measured position and current, repeatable from a seed |
| `crash` | T E S D | off | The obstacles of `obstacles` push back on a part that goes into them, with their contact stiffness and damping, with the drives on or off: the axes in contact are blocked, the others go on until the drives' monitoring (`drive_monitoring`) trips them (see the [model reference](model-reference.md#obstacles-and-crash)). Contacts are found and reported whether it is on or not; off, the machine moves through the obstacle. Changes nothing without obstacles. Off by default, so that earlier results stay as they were |
| `drive_monitoring` | T E S D | off | Each drive watches its following error, its following error at standstill, whether it is held at its torque limit at low speed, and optionally its torque against the model's (the `[drive]` keys from `fault_reaction` on); the first that trips stops all drives, each by its fault reaction. Changes nothing until a drive trips, which no shipped cycle does. Optional in a config file (off: a config written before 14 November 2026). Off by default, so that earlier results stay as they were |

**Motion planning**

| Switch | In | Default | What it does |
| --- | --- | --- | --- |
| `corner_blending` | T S off, E on | | The next cycle segment starts while the current one is still braking |
| `input_shaping` | T S off, E on | | ZVD input shaper on the setpoints of all axes |
| `shaper_follows_pose` | T E S | off | With `input_shaping`: the shaper takes its frequency from the resonance the machine reports for the planned pose, and a second shaper (`profile.shaper2_frequency_hz` above 0) that of the machine's second mode there (`sim-cli modes`); one the machine does not report stays as configured. The T-gantry reports two modes, the single axis one |

## `[geometry]`

| Key | In | Unit | Meaning | Source | Effect switch |
| --- | --- | --- | --- | --- | --- |
| `pulley_radius_m` | T E S | m | Effective radius of the motor pulleys. E: half Festo's effective diameter of 31.82 mm | datasheet | always |
| `stroke_x_m` | T E | m | Stroke of X; the axis runs from 0 to this value | datasheet | always |
| `stroke_z_m` | T E | m | Stroke of Z | datasheet | always |
| `stroke_m` | S | m | Stroke of the single axis | datasheet | always |
| `incline_deg` | S | deg | Incline of the single axis. 0 is level, 90 is vertical; gravity pulls towards −x | choice | `gravity` |
| `span_min_m` | T E S | m | Shortest free belt span. Sets the lengths of the belt sections | estimate (from a drawing) | `belt_stretch` |
| `z_com_drop_m` | T E | m | Centre of mass of the Z axis below the carriage at z = 0; it rises with z. Arm of the tilting moment | estimate | `load_dependent_friction`, `tool_flex` |
| `guide_spacing_m` | T E | m | Distance between the guide blocks that take the tilting moment | estimate (from a drawing) | `load_dependent_friction` |
| `z_tool_drop_m` | T E | m | Tool tip below the carriage at z = 0; it rises with z | estimate | `tool_flex` |

## `[mass]`

| Key | In | Unit | Meaning | Source | Effect switch |
| --- | --- | --- | --- | --- | --- |
| `carriage_kg` | T E S | kg | Carriage. On the T-gantry it moves in X only. E: chosen so that the simulator needs the torque Festo's formula gives | datasheet | always |
| `z_axis_kg` | T E | kg | The vertical axis without payload. It moves in X and Z. E: the mass Festo's formula holds against gravity | datasheet | always |
| `payload_kg` | T E S | kg | Payload, attached and released by the cycle | choice | `payload` |

## `[belt]`

| Key | In | Unit | Meaning | Source | Effect switch |
| --- | --- | --- | --- | --- | --- |
| `stiffness_ea_n` | T E S | N | Specific stiffness EA. A section of length L has the stiffness EA / L. E: from 0.045 % elongation at 300 N | datasheet | `belt_stretch` |
| `pretension_n` | T E S | N | Tension of the belt at rest without load | estimate, measurement | `belt_stretch` |
| `damping_n_s_m` | T E S | N s/m | Damping of every section on its stretch rate, whatever its length | estimate | `belt_damping` (not with `belt_material_damping`) |
| `mass_kg_m` | T E S | kg/m | Belt mass per length | datasheet (estimate in E) | `belt_mass` |
| `tooth_pitch_m` | T E S | m | Tooth pitch | datasheet | `tooth_meshing` |
| `tooth_force_n` | T E S | N | Amplitude of the tooth-meshing force | estimate | `tooth_meshing` |
| `tooth_mesh_stiffness_n_m` | T E S | N/m | Stiffness of the teeth in mesh on one driven pulley | estimate | `belt_tooth_stiffness` |
| `damping_time_s` | T E S | s | A section damps with its stiffness times this time | estimate | `belt_material_damping` |
| `tooth_jump_factor` | T E S | – | k of the climb-out law: the teeth jump when the effective pull exceeds k · z_m · p / r times the slack side's tension; 1 / the tangent of an effective flank angle (flank, friction and the tooth's give together). 2.4: a pulley drive with 10 of 25 teeth in mesh jumps at 1.5 times its pretension of effective pull (E, 10 of 20 teeth: 1.58), between the makers' rule of a pretension at least the effective pull and the slack side going slack at twice the pretension | estimate, bounds 1.2 to 4.8 (runs only with the effect) | `belt_tooth_jump` |

## `[friction_x]` and `[friction_z]`

One table per linear guide. `[friction_z]` exists on the T-gantry only (T E). On the single axis `[friction_x]` is its one guide.

| Key | In | Unit | Meaning | Source | Effect switch |
| --- | --- | --- | --- | --- | --- |
| `coulomb_n` | T E S | N | Coulomb (sliding) friction level, at the nominal guide load | estimate, measurement | `guide_friction` |
| `stiction_n` | T E S | N | Breakaway level. A value below `coulomb_n` counts as `coulomb_n` | estimate, measurement | `guide_friction` |
| `viscous_n_s_m` | T E S | N s/m | Viscous friction | estimate, measurement | `guide_friction` |
| `stribeck_m_s` | T E S | m/s | Speed over which the level falls from stiction to Coulomb. On the T-gantry `friction_x.stribeck_m_s` also sets how the cable chain's drag fades out at rest | estimate | `guide_friction` |
| `bristle_stiffness_n_m` | T E S | N/m | LuGre bristle stiffness σ₀ | estimate | `guide_friction` |
| `bristle_damping_n_s_m` | T E S | N s/m | LuGre bristle damping σ₁ | estimate | `guide_friction` |
| `preload_n` | T E S | N | Guide preload. The friction levels scale with (preload + load) | estimate | `load_dependent_friction` |

Festo gives the friction of the EXCT only as a graph of torque over speed, so all of these are estimates in E.

## `[motor]`

| Key | In | Unit | Meaning | Source | Effect switch |
| --- | --- | --- | --- | --- | --- |
| `rotor_inertia_kgm2` | T E S | kg m² | Rotor inertia, the motor's alone. The gearbox's own inertia is `gearbox.inertia_at_input_kgm2` | datasheet | always |
| `friction_coulomb_nm` | T E S | N m | Coulomb friction on the motor shaft | estimate | `motor_friction` |
| `friction_viscous_nm_s_rad` | T E S | N m s/rad | Viscous friction on the motor shaft | estimate | `motor_friction` |
| `friction_smoothing_rad_s` | T E S | rad/s | Speed over which the Coulomb step is smoothed. On the T-gantry it also smooths the idlers' bearing friction | choice | `motor_friction` |
| `friction_breakaway_nm` | T E S D | N m | The shaft's breakaway torque at rest (seals and bearings). 1.5 times `friction_coulomb_nm` in every shipped config; a value below the Coulomb level counts as that level | estimate, bounds the Coulomb level to twice it | `motor_stiction` |
| `friction_stribeck_rad_s` | T E S D | rad/s | Speed over which the level falls from breakaway to Coulomb, exp(−(ω / ω_st)²). 2 | estimate, bounds 0.5 to 10 | `motor_stiction` |
| `ripple_fraction` | T E S | fraction | Torque ripple as a fraction of the torque | estimate | `torque_ripple` |
| `ripple_order` | T E S | 1/rev | Ripple periods per motor revolution | datasheet (poles and slots) | `torque_ripple` |
| `cogging_nm` | T E S | N m | Cogging torque amplitude | estimate | `torque_ripple` |
| `cogging_order` | T E S | 1/rev | Cogging periods per revolution | datasheet | `torque_ripple` |
| `torque_constant_nm_a` | T E S | N m/A | Torque per current. Also the back-EMF constant. E: static torque 6 N m over stall current 4.7 A | datasheet | always (`back_emf` for the EMF) |
| `resistance_ohm` | T E S | Ω | Winding resistance of the equivalent winding. Siemens does not publish it for the 1FK2 | estimate in E; datasheet where published | always |
| `inductance_h` | T E S | H | Winding inductance. Not published for the 1FK2 either | estimate in E | always |
| `thermal_time_constant_s` | T E S | s | Thermal time constant of the winding. E: 31 min | datasheet | `winding_heating` |
| `pole_pairs` | T E S D | – | Pole pairs: the electrical speed per shaft speed, and with the torque constant the flux linkage ψ = k_t / p. 4 in the shipped configs. Siemens publishes it for the 1FK7 and 1FT7 (a motor by name sets it), not for the 1FK2 | estimate, bounds 3 to 5; catalogue for a 1FK7 or 1FT7 by name | `field_weakening` |
| `d_inductance_h` | T E S D | H | d-axis inductance as the winding has it (three times the phase value); `inductance_h` is the q axis's. The shipped configs and a 1FK7 or 1FT7 by name take it equal to `inductance_h` (surface magnets) | estimate, bounds ±20 % | `field_weakening` |
| `winding_resistance_share` | T E S D | share | The winding's share of the thermal resistance to the surroundings; the rest is the stator and housing's. 0.3 | estimate, bounds 0.2 to 0.5 | `motor_thermal_nodes` |
| `winding_time_constant_s` | T E S D | s | Thermal time constant of the winding against the stator and housing, below `winding_resistance_share` times `thermal_time_constant_s`. 60 s | estimate, bounds 20 to 120 | `motor_thermal_nodes` |
| `stall_current_a` | T E S D | A | Stall current I_0: the static torque is k_t I_0. With `effects.iron_loss` the copper loss at it brings the winding to its rated rise, and the S1 check `motor-s1` starts at k_t I_0. A motor by name sets it; the rated current where the motor is an estimate (no iron loss then) | catalogue (by name); estimate | `iron_loss`, analysis |
| `magnet_temperature_coefficient_1_k` | T E S D | 1/K | The magnets' flux per kelvin, −0.0012 (NdFeB loses about 0.12 % of its remanence per kelvin) | estimate, bounds −0.0013 to −0.0009 | `magnet_temperature` |
| `rated_current_a` | T E S | A | Rated current (RMS): the current at which the winding settles at the rated temperature rise. T S: rated torque over torque constant, 2.822 A. E: the catalogue's 3.75 A at 400 V. A motor by name sets the catalogue's own figure, which is up to 18 % above rated torque over the static torque constant | datasheet | `winding_heating`; `motor-thermal` always |
| `rated_temperature_rise_k` | T E S | K | Temperature rise the winding settles at with the rated current. Also the limit of `motor-thermal` | datasheet | `winding_heating`; the verdict always |
| `start_temperature_rise_k` | T E S | K | Temperature rise at the start of a simulation | choice | `winding_heating` |
| `max_current_a` | T E S | A | Maximum current. Where saturation is referred to, and with saturation on the limit of the current setpoint | datasheet | `torque_saturation` |
| `torque_constant_at_max_current_nm_a` | T E S | N m/A | Torque constant at the maximum current. Must not exceed `torque_constant_nm_a`. E: 18 N m at 15.1 A | datasheet | `torque_saturation` |
| `brake_torque_nm` | T E S | N m | Torque each motor's holding brake holds while the drives are disabled; it slips beyond. The most the brake ever gives, also when it engages at speed | datasheet (estimate in E) | `holding_brake` |

## `[gearbox]`

Between motor shaft and pulley. Stiffness, damping and backlash are given at the output side. In E this is the coupling: ratio 1, no backlash, no loss.

| Key | In | Unit | Meaning | Source | Effect switch |
| --- | --- | --- | --- | --- | --- |
| `ratio` | T E S | – | Motor revolutions per pulley revolution | datasheet | always |
| `efficiency` | T E S | fraction | Efficiency at full load | datasheet | `gearbox_efficiency` |
| `backlash_rad` | T E S | rad | Backlash at the output. 8.7e-4 rad is 3 arcmin | datasheet | `gearbox_backlash` |
| `stiffness_nm_rad` | T E S | N m/rad | Torsional stiffness | datasheet (estimate in E) | `belt_stretch` |
| `damping_nm_s_rad` | T E S | N m s/rad | Torsional damping | estimate | `belt_stretch` |
| `inertia_at_input_kgm2` | T E S | kg m² | The gearbox's own inertia at its input. It turns with the rotor and is added to `motor.rotor_inertia_kgm2` in the model. 0 in all three configs; a gearbox by name sets the catalogue value | datasheet | always |
| `efficiency_torque_nm` | T E S D | N m | The output torque at which `efficiency` holds: the rated torque, where makers give "efficiency at full load" (T S: 40, the NRB060's; D2 D3: 259, the SP+ 100's T2N; the joint kit's strain wave gears their rated torque). With the drag, the teeth's efficiency is what is left of `efficiency` there | datasheet | `gearbox_drag` |
| `no_load_torque_nm` | T E S D | N m | No-load running torque at the input at `no_load_speed_rpm` and 20 °C. T S: 0.125 (Neugart PLE060 one stage, the NRB060: 0.10 to 0.15); D2 D3: 0.93 (WITTENSTEIN SP+ 100 MF two stages, i = 20, T012); D4: 0.2 (PLE080 two stages); the arm 0.5 and SCARA 0.2 (read off Harmonic Drive's chart, sizes 32 and 25 at 2000 r/min); 0 on a coupling | datasheet, bounds the maker's range or ±20 to 25 % | `gearbox_drag` |
| `no_load_speed_rpm` | T E S D | r/min | Input speed at which `no_load_torque_nm` holds. 3000 (2000 for the strain wave gears) | catalogue convention | `gearbox_drag` |
| `lost_motion_band_rad` | T E S D | rad | Twist beyond the backlash over which the flanks' stiffness rises from 0 to `stiffness_nm_rad`. 1.454e-4 (half an arcmin) for the planetary gearboxes, 0 for strain wave gears and couplings | estimate, bounds 0 to 1 arcmin | `gearbox_lost_motion` |
| `hysteresis_rad` | T E S D | rad | Width of the hysteresis loop at zero torque, 2 T_h / K1. 2.909e-4 (1 arcmin) for the strain wave gears (Harmonic Drive's hysteresis loss); 1.454e-4 for the belt machines' planetary gearboxes and the SP+ 100 of D2 and D3 (half an arcmin; WITTENSTEIN publishes none) | datasheet (strain wave), estimate, bounds 0 to 1 arcmin | `gearbox_lost_motion` |
| `stiffness_t1_nm`, `stiffness_t2_nm` | T E S D | N m | The knees T1 and T2 of the three-segment stiffness. Harmonic Drive's Table 43, ratio 80 and up: 29 and 108 (size 32), 14 and 48 (size 25). A planetary gearbox: both at its rated torque | datasheet | `gearbox_stiffness_curve` |
| `stiffness_k2_nm_rad`, `stiffness_k3_nm_rad` | T E S D | N m/rad | K2 between the knees, K3 beyond. 1.1e5 and 1.2e5 (size 32), 5.0e4 and 5.7e4 (size 25). A planetary gearbox: both `stiffness_nm_rad` | datasheet | `gearbox_stiffness_curve` |

## `[pulley]`

| Key | In | Unit | Meaning | Source |
| --- | --- | --- | --- | --- |
| `inertia_kgm2` | T E S | kg m² | Inertia of one motor pulley, without the belt. Estimate in E | datasheet |

## `[frame]`

T-gantry, and machine files whose `machine.frame` names these values. With `effects.frame_compliance`: the beam and its mounting as one mass on a spring in X.

| Key | In | Unit | Meaning | Source |
| --- | --- | --- | --- | --- |
| `mass_kg` | T E | kg | Modal mass of the frame | estimate, measurement |
| `stiffness_n_m` | T E | N/m | Stiffness of the frame in X | estimate, measurement |
| `damping_n_s_m` | T E | N s/m | Damping of the frame mode | estimate, measurement |

## `[tool_flex]`

T-gantry, and machine files whose `machine.lean` node names these values (`machines/t-gantry.toml`). With `effects.tool_flex`: the guide of the Z axis gives way sideways and in tilt.

| Key | In | Unit | Meaning | Source |
| --- | --- | --- | --- | --- |
| `lateral_stiffness_n_m` | T E | N/m | Sideways stiffness of the Z guide | estimate |
| `tilt_stiffness_nm_rad` | T E | N m/rad | Tilt stiffness of the Z guide | estimate |
| `damping_n_s_m` | T E | N s/m | Damping of the leaning motion | estimate |
| `z_axis_bending_stiffness_nm2` | T E | N m² | With `effects.z_axis_bending`: the Z axis's bending stiffness E I. Its centre of mass h below the carriage gives way h³ / (3 E I) more, and the tool at h_t moves h² (3 h_t − h) / (6 E I) per newton there. An aluminium profile of about 60 × 40 mm: 1.4e4; bounds 0.7e4 to 2.8e4 | estimate |

## `[beam]`

T-gantry. With `effects.beam_sag`: the X beam as a simple span between the posts; carriage, Z axis and payload, m g, sag it under the carriage at a from the left post by m g a² b² / (3 E I L), b = L − a (L³ / (48 E I) per newton in the middle), and the tool is that much lower. Quasi-static: the beam's own vertical mode with the carriage (about 100 Hz) lies above the Z axis's on its belt.

| Key | In | Unit | Meaning | Source |
| --- | --- | --- | --- | --- |
| `span_m` | T E | m | Span between the posts. T: the motors at `geometry.span_min_m` beyond each end of the stroke, 1.3; E: Festo's drive heads 1354 mm apart | T geometry, E CAD |
| `carriage_at_zero_m` | T E | m | The carriage's distance from the left post at x = 0. T 0.15, E 0.177 (the stroke in the middle) | T geometry, E estimate |
| `bending_stiffness_nm2` | T E | N m² | The beam's E I: an aluminium profile of about 80 × 80 mm, 8.4e4; bounds 4.2e4 to 1.7e5 | estimate |

## `[base]`

Every machine and machine file. With `effects.base_compliance` the frame or stand under the whole machine is a rigid body on springs: each direction with a stiffness above 0 is a state of its own (`sim_components::base`, model reference "Compliant base"). Directions in the frame of the tool pose's x, y, z (the slides' x along their axis); a tilt turns about `pivot_m`. Off in every shipped config: the values are typical, not measured. Signals `base.<x|y|z|rx|ry|rz>` (m, rad).

| Key | Unit | Meaning |
| --- | --- | --- |
| `pivot_m` | m | The point the base tilts about, (x, y, z) in the tool pose's frame |
| `mass_kg` | kg | Everything that sways with the base, the machine on it included: the frequency of a direction is sqrt(k / M) / 2π |
| `inertia_x_kgm2`, `inertia_y_kgm2`, `inertia_z_kgm2` | kg m² | The same about the axes through the pivot, for the tilts |
| `stiffness_x_n_m`, `stiffness_y_n_m`, `stiffness_z_n_m` | N/m | Translation stiffness; 0 rigid |
| `tilt_stiffness_x_nm_rad`, `tilt_stiffness_y_nm_rad`, `tilt_stiffness_z_nm_rad` | N m/rad | Tilt stiffness; 0 rigid |
| `damping_ratio_x`, `_y`, `_z`, `_rx`, `_ry`, `_rz` | – | Damping ratio of each direction, c = 2 ζ sqrt(k M); 0.02 (welded steel) in all configs |

The shipped values, all estimates with `[bounds]` (stiffness ÷ 2.5 to × 2.5, mass 0.6 to 1.6 times, damping 0.01 to 0.05):

| Machines | Base | Values |
| --- | --- | --- |
| D2, D3 (and its conveyor config), D4, linear delta, `planar-delta.toml` | A welded steel picker frame with the robot, 300 kg | x, y 2.7e6 N/m (15 Hz), z 1.9e7 N/m (40 Hz); tilts rigid |
| T-gantry, EXCT-30, the gantry machine files | The stand under the gantry with it, 150 kg | x, y 2.4e6 N/m (20 Hz), z 1.5e7 N/m (50 Hz); tilts rigid |
| Single axis, screw, rack and linear motor axes, `single-axis.toml` | The machine bed, 100 kg | along the axis 2.5e6 N/m (25 Hz); the rest rigid |
| Rotary table | The table's stand, 20 kg m² about z | twist 7.1e5 N m/rad (30 Hz) |
| Six-axis arm | The pedestal, 60 kg m² about the floor in tilt, 10 about z | tilts 1.5e6 N m/rad (25 Hz), twist 6.3e5 N m/rad (40 Hz) |
| SCARA | The column and its table, 30 kg m² in tilt, 5 about z | tilts 1.1e6 N m/rad (30 Hz), twist 3.2e5 N m/rad (40 Hz) |

## `[gantry]`

Machine files whose `machine.gantry` node names these values (`machines/dual-gantry.toml`, and the H-bot, CoreXY and rack gantry files with the effect off). With `effects.gantry_yaw`: the bridge's yaw. The node may name any keys; these are the shipped files'.

| Key | In | Unit | Meaning | Source |
| --- | --- | --- | --- | --- |
| `width_m` | files | m | Distance between the bridge's two sides (its guides, where its drives are) | geometry |
| `inertia_kgm2` | files | kg m² | The bridge's own inertia about a vertical axis through its centre, without the bodies that move across it | estimate, bounds |
| `stiffness_nm_rad` | files | N m/rad | What holds the yaw beside drives and belts: the bridge's joints on its carriages and their guide blocks | estimate, bounds |
| `damping_nm_s_rad` | files | N m s/rad | Damping of the yaw | estimate, bounds |
| `sync_tolerance_m` | dual gantry | m | The synchronisation error (width times yaw) check `gantry-sync` allows; 0.1 mm | choice |

## `[idler]`

T-gantry, and machine files whose `machine.idler` nodes name these values. With `effects.idlers`: the four carriage idlers and the one at the bottom of the Z axis, all alike. The table has to be present and `radius_m` positive even with the effect off.

| Key | In | Unit | Meaning | Source |
| --- | --- | --- | --- | --- |
| `inertia_kgm2` | T E | kg m² | Inertia of one idler | estimate |
| `radius_m` | T E | m | Radius at which the belt runs | estimate |
| `friction_nm` | T E | N m | Bearing friction torque of one idler | estimate |

## `[cable_chain]`

With `effects.cable_chain`: the chain is fixed at the x = 0 end, so half of every metre the carriage travels joins the run that moves with it. On the single axis with an incline, the weight of that run pulls along the axis like the carriage's.

| Key | In | Unit | Meaning | Source |
| --- | --- | --- | --- | --- |
| `mass_kg_m` | T E S | kg/m | Mass of the chain with its cables per length | estimate |
| `length_at_zero_m` | T E S | m | Length of chain that moves with the carriage at x = 0 | estimate |
| `drag_n` | T E S | N | Constant drag force against the motion in X | estimate |

## `[process]`

With `effects.process_force`: a force on the tool tip (single axis: on the carriage) while the cycle dwells.

| Key | In | Unit | Meaning | Source |
| --- | --- | --- | --- | --- |
| `force_x_n` | T E S | N | Force on the tool in X. May be negative | choice |
| `force_z_n` | T E | N | Force on the tool in Z (positive is upwards) | choice |
| `force_<axis>_n` | machine files | N | Force on the tool along each axis of a machine file (`force_y_n` on the H-bot, the CoreXY and the portal). Read only with the effect on | choice |

## A motor's own tables

In a machine file and on the arms, `[motor.<id>]`, `[gearbox.<id>]`, `[drive.<id>]`, `[ratings.<id>]`, `[strain_wave.<id>]`, `[joint.<id>]` and `[parts.<id>]` give the motor
`<id>` its own values over the shared tables, key by key; each key has to be one the shared table has, with its meaning,
unit and mark there. `[parts.<id>]` takes `motor`, `motor_supply`, `gearbox` and `drive` (not `belt` or `axis`, which
are the machine's). See [machine files](machine-files.md#motors-that-differ). The portal of `machines/xyz-portal.toml`
has them for its Z motor, the arms' configs for their wrists (`[motor.4]` to `[motor.6]` on the six-axis arm, joints 3 and 4 on the SCARA) and the palletiser for each of its joints. A config of a registered machine may give a motor its own tables though the default config has none for it (a seventh joint).

## A machine file's screws, racks, linear motors and strain wave gears

In a machine file, each `screw`, `rack`, `linear_motor` node and each `rotary` node's `strain_wave` names a table of
its values by its path (any name: `[screw_z]` in `machines/screw-portal.toml`, `[rack_x]` in
`machines/rack-gantry.toml`). That table is laid out as the joint kit's `[screw]`, `[rack]`, `[linear_motor]` or
`[strain_wave]` (keys, units and meanings in [mechanisms](mechanisms.md); every key is required there as in the kit),
and is checked against that layout key by key. A screw's and a rack's dimensions and ratings are what its maker's
catalogue lists for the article ([catalogue] where one is named, placeholders in the shipped files); efficiency,
stiffness, preload and the load ratings of an unnamed size are [estimate] and have `[bounds]` entries, as
`screw_z.efficiency = [0.85, 0.95]` and `rack_x.stiffness_n_m = [6.0e7, 1.5e8]`. A rotary node's axis takes its limits
in rad (`[axis.<id>]` below).

## `[hard_stop]`

With `effects.hard_stops`: a spring and a damper at both ends of each stroke that act only beyond it.

| Key | In | Unit | Meaning | Source |
| --- | --- | --- | --- | --- |
| `stiffness_n_m` | T E S | N/m | Stiffness of a stop | estimate |
| `damping_n_s_m` | T E S | N s/m | Damping of a stop | estimate |

## `[temperature]`

Applied only with `effects.temperature` on, and not with `effects.thermal_feedback`, which computes what these set by hand. A cold machine has more friction and a stiffer belt.

| Key | In | Unit | Meaning | Source |
| --- | --- | --- | --- | --- |
| `friction_scale` | T E S | factor | On the Coulomb, stiction and viscous levels of the guides | estimate |
| `belt_stiffness_scale` | T E S | factor | On `belt.stiffness_ea_n` | estimate |

## `[thermal]`

With `effects.part_heating`. Each gearbox and guide is one thermal mass behind a thermal resistance to the air: under a steady loss P its temperature rise settles at resistance × P, with the time constant (see the [model reference](model-reference.md#thermal-model-and-view)). The rated rise is the limit of the checks `gearbox-thermal` and `guide-thermal` (judged only with the effect on), where the 3D view's heat colour reaches red and what the thermal view's share is of. Estimates: no maker of the parts in the configs publishes them. T and S: a size 60 to 70 planetary gearbox (surface about 0.04 m² at 15 W/(m² K) gives 1.7 K/W; 1.5 kg of steel and aluminium, about 800 J/K, gives 20 minutes) and a size 15 ball guide with its rail and the aluminium profile the rail is bolted to: the carriage runs over the whole stroke and spreads its heat along the rail, so about 0.35 m² of profile at 10 W/(m² K) (still air and radiation) with a fin efficiency of about 0.6 gives 0.5 K/W, and about 5 kg of aluminium and steel (5 kJ/K) 40 minutes. The first estimate, the carriage with a piece of rail at 2 K/W, put the guides of two of the three shipped configs above their rated rise (EXCT-30 X guide 101 K, single axis 50 K, against 40 K; default T-gantry 16.8 K). E: the same values; the EXCT has no gearbox (ratio 1: `[gearbox]` is the coupling), so no gearbox warms and its gearbox values are only there for a gearbox put in.

| Key | In | Unit | Meaning | Source | Bounds (T S E) |
| --- | --- | --- | --- | --- | --- |
| `gearbox_resistance_k_w` | T E S | K/W | Thermal resistance of a gearbox to the air | estimate | 0.8 to 3.0 |
| `gearbox_time_constant_s` | T E S | s | Thermal time constant of a gearbox | estimate | 600 to 2400 |
| `gearbox_max_rise_k` | T E S | K | Rated rise of a gearbox: 90 °C on the housing at 40 °C around | estimate (typical maker limit) | none: a limit, like `[ratings]` |
| `guide_resistance_k_w` | T E S | K/W | Thermal resistance of a guide (rail and the profile it is bolted to) to the air. 0.5 | estimate | 0.25 to 1.0 |
| `guide_time_constant_s` | T E S | s | Thermal time constant of a guide. 2400 | estimate | 1200 to 4800 |
| `guide_max_rise_k` | T E S | K | Rated rise of a guide: 80 °C at 40 °C around | estimate (typical maker limit) | none |

With `effects.part_heating` (or `thermal_feedback`, which warms the parts too) the resistances move the two thermal checks and have bands; the time constants move no verdict unless the feedback is on. Without either effect nothing depends on these values, and their bounds get no runs: `verdicts --bands` and the sensitivity leave them out.

## `[thermal_feedback]`

With `effects.thermal_feedback` (see the [model reference](model-reference.md#temperature-feedback)). The config's friction, efficiency and belt stiffness hold at the reference temperature; the machine stands at the ambient temperature; each gearbox and guide is warmer by its rise (`[thermal]`). Without the effect nothing depends on these values, and their bounds get no runs.

| Key | In | Unit | Meaning | Source | Bounds (T S E) |
| --- | --- | --- | --- | --- | --- |
| `ambient_c` | T E S | °C | Temperature of the air around the machine. 20 | estimate | 10 to 40 |
| `reference_c` | T E S | °C | Temperature at which the config's friction, gearbox efficiency and EA hold. 20, as gearbox makers quote their no-load torque at 20 °C | catalogue convention | none |
| `viscosity_slope_k` | T E S | K | B of the lubricant's viscosity, exp(B (1/T − 1/T_ref)). 4000: a mineral oil ISO VG 68 (68 and 8.7 mm²/s at 40 and 100 °C); VG 32 gives 3470, VG 220 4730 | estimate | 3470 to 4730 |
| `gearbox_viscous_loss_share` | T E S | – | Share of a gearbox's loss (1 − η) that is churning and bearing drag and follows the viscosity; the rest, tooth friction, stays. 0.5. No maker publishes it. In E only for a gearbox put in. Taken as 0 with `effects.gearbox_drag`, whose no-load torque is that share | estimate | 0.25 to 0.75 |
| `seal_viscosity_exponent` | T E S D | – | m of the motor shaft's seal friction (`motor.friction_coulomb_nm`, `friction_breakaway_nm`) over the gearbox's lubricant, (ν / ν_ref)^m at the gearbox's temperature. 0.33 | estimate | 0 to 0.5 |
| `belt_stiffness_coefficient_1_k` | T E S | 1/K | How much of its EA the belt loses per kelvin above the reference: the polyurethane softens, the steel cords (about 2.5e-4 per K) carry the load. 3e-4. The belt is taken at ambient | estimate | 0 to 1e-3 |

## `[axis.x]` and `[axis.z]`

Limits of the motion planner per tool axis. `[axis.z]` exists on the T-gantry only (T E).

| Key | In | Unit | Meaning | Source |
| --- | --- | --- | --- | --- |
| `max_velocity_m_s` | T E S | m/s | Highest planned speed | choice, within the datasheet |
| `max_acceleration_m_s2` | T E S | m/s² | Highest planned acceleration | choice, within the datasheet |
| `max_jerk_m_s3` | T E S | m/s³ | Highest planned jerk | choice |
| `max_velocity_rad_s`, `max_acceleration_rad_s2`, `max_jerk_rad_s3` | machine files | rad/s, rad/s², rad/s³ | The same for a rotary axis of a machine file (one a `rotary` node turns), in place of the m keys | choice, within the datasheet |

E stays below Festo's limits of 5 m/s and 50 m/s², which hold "under ideal conditions".

## `[drive]`

One drive per motor, all with the same settings.

| Key | In | Unit | Meaning | Source | Effect switch |
| --- | --- | --- | --- | --- | --- |
| `current_cycle_s` | T E S | s | Current loop cycle. Must equal `sim.dt_s` | datasheet | always |
| `velocity_cycle_s` | T E S | s | Velocity loop cycle | datasheet | always |
| `position_cycle_s` | T E S | s | Position loop cycle | datasheet | always |
| `current_delay_cycles` | T E S | cycles | Current-loop cycles between computing the voltage and it acting. Whole number, 0 to 1000 | estimate | `drive_delay` |
| `velocity_delay_cycles` | T E S | cycles | Velocity-loop cycles between computing the torque setpoint and it acting | estimate | `drive_delay` |
| `kv_1_s` | T E S | 1/s | Position gain Kv | choice | always |
| `velocity_kp_nm_s_rad` | T E S | N m s/rad | Velocity loop gain Kp. 0.04 in T, 0.025 in S (tuned with `sim-cli margins --bands` for at least 35° phase and 6 dB gain margin in every bounds run, with friction as configured and off; 0.08 before, which left T 12° and S 1°), 0.45 in E (tuned the same way: at least 38.6° and 8.3 dB; 0.6 before, 29.6° and 5.8 dB). Stated for the rotor, gearbox and masses the config has as numbers; a part by name retunes it from there (see [`[parts]`](#parts)) | choice | always |
| `velocity_tn_s` | T E S | s | Velocity loop integral time Tn | choice | always |
| `velocity_feedforward_factor` | T E S | factor | Weight of the velocity feedforward | choice | `velocity_feedforward` |
| `torque_feedforward_factor` | T E S | factor | Weight of the torque feedforward | choice | `torque_feedforward` |
| `torque_feedforward_model` | T E S D | `"rigid"` or `"inertia"` | What the torque feedforward is. `rigid`: the rigid model's inverse dynamics, inertia, gravity and the speed terms of a kinematics that is not linear, coupled between the motors: what a controller with a dynamic model (SIMOTION, a robot controller) sends. `inertia`: one moment of inertia per motor, the rigid model's at `start.pose` (its own acceleration, the others standing), times the motor's acceleration setpoint: what a drive does with speed/torque precontrol and the load's inertia set (S120 p1498), as most commissionings of a single axis or a gantry leave it; gravity is left to the speed loop's integrator. `rigid` in the shipped configs; a config without the key has it | choice | `torque_feedforward` |
| `friction_feedforward_speed_rad_s`, `friction_feedforward_torque_nm` | T E S D | rad/s, N m | The friction characteristic: torques at rising speeds above 0, linear between the points, from (0, 0) to the first, held beyond the last, with the sign of the speed setpoint. `[10.0]`, `[0.0]` in the shipped configs: none recorded | measurement | `friction_feedforward` |
| `precontrol_balance_s` | T E S D | s | The position setpoint passes a first-order lag of this time constant before the position loop compares it, so that it arrives with the motor that the speed feedforward moves through the closed speed loop (S120 p1429; the dead time p1428 is not modelled). Typically the speed loop's equivalent time constant, a few ms, with 100 % speed feedforward. The following error is then the one against the balanced setpoint. 0 (off) in the shipped configs and where a config lacks the key | choice | always |
| `load_encoder_step` | T E S D | m, rad | The step of the load-side scales, in each tool axis's unit; 0 is an ideal scale. 1e-6 in the shipped configs | datasheet | `load_encoder` |
| `current_kp_v_a` | T E S | V/A | Current controller gain. Kp = L / T gives the current loop the time constant T | choice | always |
| `current_tn_s` | T E S | s | Current controller integral time. Tn = L / R | choice | always |
| `bus_voltage_v` | T E S | V | The DC link voltage: the level the rectifier holds the DC link at. The winding gets sqrt(3/2) times it at most, because the model's one winding carries the RMS phase current and its voltage times that current is the power of all three phases. T S: 155.13 V, which is 190 V at the winding. E: 540 V, 661 V at the winding. A motor by name sets 540 V or 325 V | datasheet | `voltage_limit`, `dc_link` |
| `peak_torque_nm` | T E S | N m | Highest torque of motor and drive. E: the 12.5 N m the EXCT-30 allows at the pulley; the motor could give 18 N m | datasheet | `torque_limit`; analysis always |
| `rated_torque_nm` | T E S | N m | What the motor can give continuously. The analysis reports the RMS torque over it (`thermal_utilisation`); the winding's heating rests on `motor.rated_current_a` | datasheet | analysis |
| `rated_speed_rad_s` | T E S | rad/s | Full torque up to this speed. 314 rad/s is 3000 r/min | datasheet | `torque_limit`; analysis always |
| `no_load_speed_rad_s` | T E S | rad/s | The torque-speed curve falls linearly from the rated speed to zero here. E: 518 rad/s | datasheet, or sqrt(3/2) × DC link voltage over torque constant | `torque_limit`; analysis always |
| `max_speed_rad_s` | T E S D | rad/s | The highest speed the motor may turn at. E: 628.3 rad/s, the 1FK2205-4AF's 6000 r/min [catalogue]; elsewhere [estimate] the no-load speed, where the torque-speed curve ends. A motor by name sets the catalogue's: the S210 table's mechanical limit, the converter's for a 1FK2 or 1FT2 on S120, the lower of the two for a 1FK7 or 1FT7. The time-optimal plan turns each motor up to it along its torque-speed curve; `check_limits` warns of axis limits beyond it; the verdict `motor-speed` judges the peak speed against it. Per motor in `[drive.<id>]` where the file has the key there. Not in `[bounds]`: a limit, not a property to measure | datasheet | time-optimal plan, `check_limits`, `motor-speed` |
| `encoder_counts_per_rev` | T E S | counts | Encoder resolution. 1048576 is 20 bit, 4194304 is 22 bit | datasheet | `encoder_resolution`, `sensor_noise` |
| `velocity_filter_s` | T E S | s | Time constant of the speed filter | choice | `velocity_filter` |
| `torque_filter_s` | T E S | s | Time constant of the torque setpoint filter | choice | `torque_filter` |
| `notch_frequency_hz` | T E S | Hz | Centre of the notch | choice | `notch_filter` |
| `notch_bandwidth_hz` | T E S | Hz | Width of the notch | choice | `notch_filter` |
| `encoder_noise_counts` | T E S | counts | Standard deviation of the noise on the measured position | estimate | `sensor_noise` |
| `current_noise_a` | T E S | A | Standard deviation of the noise on the measured current | estimate | `sensor_noise` |
| `noise_seed` | T E S | – | Seed of the noise and of the fieldbus jitter. A whole number, not negative. Each motor has a stream of its own, and the jitter one more | choice | `sensor_noise`, `fieldbus_jitter` |
| `fault_reaction` | T E S D | – | With `effects.drive_monitoring`: how the drive stops when the drives trip. `"off1"` ramps the speed setpoint down on `off1_ramp_s` (S120 OFF1, p1121), `"off3"` on `off3_ramp_s` (OFF3, p1135), `"sto"` takes the torque off at once (the motor coasts), `"current_limit"` brakes at the torque limit (OFF3 with no ramp). Per motor as `[drive.<id>]`. `"off3"` in all files | choice | `drive_monitoring` |
| `off1_ramp_s` | T E S D | s | OFF1 ramp: the time from the rated speed to standstill; 0 brakes at the torque limit. 1.0 in all files | estimate | `drive_monitoring` |
| `off3_ramp_s` | T E S D | s | OFF3 ramp, the same. 0.1 in all files, a fast stop as commissioned | estimate | `drive_monitoring` |
| `pulse_off_speed_rad_s` | T E S D | rad/s | Once its ramp is down, the drive takes its pulses off below this speed (p1226). 2.09 (20 r/min); 0.02 m/s on a linear motor | estimate | `drive_monitoring` |
| `following_error_window_rad` | T E S D | rad | The drive trips when its following error (at the motor) is beyond this (p2546). 1.0; 0.005 m on a linear motor. A setting, not a physical value: no `[bounds]` | estimate | `drive_monitoring` |
| `standstill_window_rad` | T E S D | rad | Once the setpoint has stood still `standstill_time_s`, the drive trips when its following error is beyond this (p2542). 0.1; 0.0005 m on a linear motor | estimate | `drive_monitoring` |
| `standstill_time_s` | T E S D | s | How long the setpoint stands still before the standstill window applies (p2543). 1.0 | estimate | `drive_monitoring` |
| `blocked_speed_rad_s` | T E S D | rad/s | A motor slower than this while its velocity loop asks for more than the torque limit counts as blocked (F07900, p2175). 12.57 (120 r/min); 0.05 m/s on a linear motor | estimate | `drive_monitoring` |
| `blocked_time_s` | T E S D | s | A blocked motor trips after this long (p2177). 3.0 | estimate | `drive_monitoring` |
| `torque_band` | T E S D | – | The drive trips when its torque is further from the torque the model expects (its torque feedforward, before its weight) than this share of its peak torque, as a robot controller detects a collision; 0 off. 0 in all files | choice | `drive_monitoring` |
| `braking_module` | T E S D | on/off | The axes brake through one braking resistor for all, as an S120 line-up through its Braking Module: `verdicts` judges the sum of their braking power against `ratings.braking_resistor_continuous_w` (the mean of the sum) and `_peak_w` (the peak of the summed power, step by step), once for the machine. Off in all four files: each axis has a resistor of its own, as an S210, and is judged alone. A drive by name sets it: on for an S120 Motor Module, off for an S210. Optional in a config file (off) | choice | `verdicts` |

## `[ratings]`

What the parts allow. `sim-cli verdicts` judges a cycle against these. **A rating that is left out is not judged.** With `effects.dc_link` the capacitance, the braking threshold, the resistor's peak power, the trip level and the discharge time constant are required, because the simulated DC link is built from them (with `effects.regenerative_supply` also the line module's maximum power and efficiency), and the threshold has to lie between `drive.bus_voltage_v` and `dc_link_overvoltage_v`. `verdicts` refuses a threshold at or below `drive.bus_voltage_v` with the effect off as well: it would leave the capacitors a negative room.

| Key | In | Unit | Meaning | Judged by | Source in the shipped configs |
| --- | --- | --- | --- | --- | --- |
| `belt_max_tension_n` | T E S | N | Allowable belt tension. E: 2140 N. Festo publishes no allowable belt tension for the EXCT. It states the 5 mm pitch, the steel cords, the 30 mm width and the stretch of the belt: 0.045 % at 300 N, which is 667 kN per unit strain. Megadyne's Megalinear handbook lists for 5 mm pitch steel-cord polyurethane belts, at 4 mm/m stretch: T5 840 N at 25 mm and 1060 N at 32 mm (about 250 kN per unit strain at 30 mm), AT5 1805 N at 25 mm and 2275 N at 32 mm (about 535 kN). Festo's belt is somewhat stiffer than AT5 with standard steel cords and far stiffer than T5, so the limit is the AT5 figure between the two widths: 2140 N at 30 mm. The figures were read from the handbook itself (average values for open-ended belts, a quarter of the breaking strength); that Festo's belt has cords of that class is concluded from the stretch, not stated by Festo. The teeth give way before the cords do: that is judged by `pulley_max_torque_nm`. | `belt-tension` | placeholder (T S), estimate (E) |
| `belt_min_tension_n` | T E S | N | Lowest tension allowed. Below it the teeth can jump | `belt-slack` | choice (0) |
| `settling_tolerance_m` | T E S | m | How far the tool's place (its axes in m) may be from its target in the second half of a dwell | `settling` | choice |
| `settling_tolerance_rad` | arms, table, D4 | rad | How far the tool's turn (its axes in rad: an arm's joints or orientation, the SCARA's and the D4's c, the table) may be from its target then, apart from the place: metres and radians are not added. Any config may add it | `settling-angle` | arm 3 mrad in joints, 5 mrad in tool space; SCARA 1 mrad (2 in tool space); table 0.2 mrad; palletiser and D4 5 mrad; estimates |
| `pulley_max_torque_nm` | E | N m | Allowable torque at the pulley. Also caps the peak torque of a motor by name, and the time-optimal plan keeps `profile.torque_reserve` of it at the gearbox output | `pulley-torque` | datasheet (Festo: 12.5 N m) |
| `gearbox_rated_torque_nm` | T S | N m | Gearbox rated (continuous) torque at the output. A gearbox by name sets it | `gearbox-continuous` | datasheet (as Siemens NRB060, ratio 5) |
| `gearbox_max_torque_nm` | T S | N m | Gearbox maximum torque at the output. A gearbox by name sets it. The time-optimal plan keeps `profile.torque_reserve` of it at the output (where the machine reports `gearbox.<motor>.output_torque`) | `gearbox-peak` | datasheet (as Siemens NRB060, ratio 5) |
| `gearbox_max_input_speed_rpm` | T S | r/min | Highest speed at the gearbox input. Not in the catalogue's gearbox files | `gearbox-input-speed` | placeholder |
| `shaft_max_radial_load_n` | T E S | N | Radial load the belt may put on a pulley shaft; in a machine file also what a rack's pinion may put on the gearbox's output shaft (`rack.<motor>.radial_load`). Not in the gearbox files; not published by Festo | `gearbox-radial-load` | placeholder (T S), estimate (E) |
| `guide_max_load_n` | T E S | N | Highest load on a guide. The time-optimal plan keeps the guides' loads in the rigid model within it where the machine gives them (T-gantry, single axis) | `guide-load` | placeholder (T S), estimate (E) |
| `guide_dynamic_load_rating_n` | T E S | N | Dynamic load rating C of the life estimate L10 = (C / P)³ · 50 km | `guide_life_km` of `analyze`; in hours `guide-life` | placeholder (T S), estimate (E) |
| `required_life_h` | T E S | h | The life bearings, gearbox, gearing and guide are to last. 20000 h in all three, the life Siemens rates its planetary gearboxes' planet bearings for | `pulley-bearing-life`, `gearbox-life`, `gearbox-gearing-life`, `motor-bearing-life`, `guide-life` (each passes at or above it) | choice |
| `bearing_reliability_percent` | T E S | % | Reliability of the two bearing lives: the life factor a1 of ISO 281 turns their L10 (90 %) into the life this share of bearings reaches (0.64 at 95 %, 0.25 at 99 %). 90 to 99.95; 90 in all three, which leaves the lives as they were | `pulley-bearing-life`, `gearbox-life` | choice |
| `pulley_bearing_load_rating_n` | T E S | N | Dynamic load rating C of the bearing that takes the belt's pull on a pulley shaft, for the rating life (C / P)³ · 10⁶ / (60 n) h. T S: 8469 N, the output bearing of a Siemens NRB120 (1500 N for 30000 h at 100 r/min, Table 7-14 of the planetary gearbox configuration manual 01/2023; the NRB060 whose torques T S take would allow 340 N, less than their 600 N of pretension on the shaft). E: 9500 N for two deep-groove ball bearings of about 5.85 kN that share the pull (a pair lasts as one bearing of 1.62 C) | `pulley-bearing-life` | placeholder from the catalogue (T S); estimate (E), bounds 8000 to 12000 N |
| `bearing_lubricant_viscosity_mm2_s` | T E S | mm²/s | Kinematic viscosity ν of the pulley shaft bearing's lubricant at operating temperature, for a_ISO of ISO 281:2007 (L_nm = a1 · a_ISO · L10): κ = ν / ν1, ν1 = 45000 · n^−0.83 · D_pw^−0.5 below 1000 r/min, 4500 · n^−0.5 · D_pw^−0.5 above. 60: a grease of ISO VG 100 base oil at about 50 °C. This and the next three go together: with none of them a_ISO is 1, with some but not all `verdicts` refuses the config by the missing key | `pulley-bearing-life` | estimate, bounds 30 to 120 |
| `bearing_contamination_factor` | T E S | – | Contamination factor e_C of ISO 281:2007, Table 13 (mean diameter below 100 mm): 1 extreme cleanliness; 0.8 to 0.6 high (bearings sealed and greased for life); 0.6 to 0.5 normal; 0.5 to 0.3 slight; 0.3 to 0.1 typical; 0.1 to 0 severe; 0 very severe contamination. 0 to 1, others refused. 0.7 | `pulley-bearing-life` | estimate, bounds 0.6 to 0.8 |
| `pulley_bearing_fatigue_limit_n` | T E S | N | Fatigue load limit C_u of the pulley shaft bearing. Where the maker gives none, ISO 281's approximation for ball bearings below 100 mm, C_u = C0 / 22. T S: 193 N for a 6004-like bearing of the placeholder's C (C0 = 0.5 C = 4235 N). E: 259 N for the pair of 6002 (C0 2.85 kN each) | `pulley-bearing-life` | placeholder (T S); estimate (E), bounds 200 to 320 |
| `pulley_bearing_mean_diameter_mm` | T E S | mm | Mean diameter D_pw of the pulley shaft bearing, (bore + outer) / 2, for ν1. T S: 31 mm (20 × 42); E: 23.5 mm (15 × 32) | `pulley-bearing-life` | placeholder (T S); estimate (E), bounds 20 to 28 |
| `motor_bearing_load_rating_n` | in none | N | Dynamic load rating C of the motor's own bearings, where the belt's pulley sits on the motor shaft and `gearbox.ratio` is 1: their rating life (C / P)³ · 10⁶ / (60 n) · a1 with the pulley shaft's radial load. Siemens gives no C for the 1FK2, only the radial force allowed for 25000 h at a speed: C = F · (25000 · 60 · n / 10⁶)^(1/3), the 1FK2205's 13104 N from 1000 N at 1500 r/min, 20 mm from the shoulder (`catalog/siemens/motor_bearings_1fk2.csv`). No shipped config has it: their belts pull on a gearbox output or Festo's drive head. A config file may add it (like `pulley_max_torque_nm`) | `motor-bearing-life` | estimate from the catalogue |
| `gearbox_gearing_wohler_exponent` | T E S | – | Exponent k of the gearing's Wöhler line through Siemens' figures, `gearbox_max_torque_nm` (M2max) for at most 30000 output revolutions and no damage at or below `gearbox_rated_torque_nm` (M2N): a torque M between allows 30000 · (M2max / M)^k output revolutions, and Palmgren–Miner sums the damage over the cycle. 8.7: the tooth root's S-N slope of case-hardened gears in ISO 6336-3, ln(3000) / ln(2.5). No figure where the cycle stays at or below M2N. In E not used: no gearbox | `gearbox-gearing-life` | estimate, bounds 6.6 to 10 |
| `gearbox_reference_speed_rpm` | T E S | r/min | Output speed at which a planetary gearbox's planet bearings last 20000 h at the rated torque, for Siemens' life formula. 175 r/min, the NRB060 with ratio 5 (Table 7-15). A gearbox by name does not set it. In E not used: no gearbox and no `gearbox_rated_torque_nm` | `gearbox-life` | catalogue |
| `belt_tooth_load_rpm`, `belt_tooth_load_n_cm` | T E S | r/min, N/cm | Two arrays of the same length, speeds rising: the specific tooth load the belt's teeth allow per cm of width and tooth in mesh, at the pulley's speed; interpolated, held beyond the ends. Megadyne's table for open-ended AT5 (MEGALINEAR handbook 02.2026, page 52). Not in the parameter panel (arrays) | `belt-tooth-load` | catalogue (E); placeholder (T S) |
| `belt_width_mm` | T E S | mm | Belt width. E: 30 mm (Festo); T S: 25 mm | `belt-tooth-load` | catalogue (E), placeholder (T S) |
| `belt_service_factor` | T E S | – | Factor on the effective pull at the pulley. 1.4, Megadyne's factor for low shock loads (Table 4 of the handbook) | `belt-tooth-load` | choice |
| `belt_teeth_in_mesh` | T E S | – | Teeth in mesh on a driven pulley; at most 12 count, as the handbook says. 10 in all three (half of the EXCT-30's 20-tooth pulley). With `effects.belt_tooth_jump` also z_m of the climb-out law (see `belt.tooth_jump_factor`). Not used with `effects.belt_wrap`, which counts them from the wrap | `belt-tooth-load` | estimate, bounds 6 to 12 (T S), 7 to 12 (E) |
| `drive_rated_current_a` | T E S | A | Rated output current of the drive, against which the RMS motor current of the cycle is judged. A drive by name sets it. E: 5 A, its converter 6SL3210-5HE11-5UF0; T S: 3 A, the S210 converter 6SL3210-5HE11-0UF0, which carries the placeholder motor's rated current (2.8 A) | `drive-current` | catalogue (E), placeholder (T S) |
| `drive_max_current_a` | T E S | A | Maximum output current of the drive, against which the peak motor current is judged; with `effects.drive_current_limit` also the limit of the simulated current and, through the torque constant, of the time-optimal plan's torque. E: 19 A; T S: 10.9 A (above the placeholder motor's 9.5 A) | `drive-peak-current` | catalogue (E), placeholder (T S) |
| `drive_base_load_current_a` | T E S D | A | With `effects.drive_overload`: the current the drive falls back to once its thermal image trips, below `drive_rated_current_a`. A drive by name sets it: an S120 Motor Module's from the catalogue; 0.85 of an S210 converter's rated current, which Siemens does not publish (the share of the S120 Motor Modules up to 30 A). 0.85 of the rated current in the shipped configs | `drive-overload` | catalogue (S120 by name); estimate, bounds 0.8 to 0.9 of the rated current |
| `drive_overload_time_constant_s` | T E S D | s | With `effects.drive_overload`: time constant of the drive's thermal image. 10 s | `drive-overload` | estimate, bounds 2 to 60 |
| `drive_max_current_time_s` | T E S D | s | With `effects.drive_overload`: how long the drive gives its maximum current from the rated load; sets the image's trip level r² − (r² − 1) exp(−t / τ), r = max over rated current. 0.25 s | `drive-overload` | estimate, bounds 0.1 to 1 |
| `drive_chip_share` | T E S D | share | With `effects.drive_overload`: the chips' share of the power unit's rise, the fast node of the two-node image; 0 for one node. 0.3 | `drive-overload` | estimate, bounds 0 to 0.5 |
| `drive_chip_time_constant_s` | T E S D | s | With `effects.drive_overload`: time constant of the chips' node. 1 s | `drive-overload` | estimate, bounds 0.1 to 5 |
| `belt_span_resonance_band` | T E S | share | With `effects.belt_span_vibration`: how close a span's first sideways mode may come to an excitation of the drive at steady speed, as a share of the mode. 0.1: belt makers ask to keep the two apart without a figure; 10 % is the separation usually kept from a resonance | `belt-span-resonance` (passes at or above it) | estimate, a limit, not bounded |
| `dc_link_capacitance_uf` | T E S | µF | DC link capacitance of one drive | `dc-link-energy` | placeholder (T S); estimate from S120 data (E) |
| `braking_threshold_v` | T E S | V | DC link voltage at which the chopper switches the braking resistor in. T S: 224.65 V. A motor by name scales it with the DC link voltage | `dc-link-energy` | placeholder (T S); estimate (E) |
| `braking_resistor_continuous_w` | T E S | W | Continuous power of the braking resistor. A drive by name sets it (S210) | `braking-resistor-continuous` | placeholder (T S); catalogue (E, 100 W, its converter's internal resistor) |
| `braking_resistor_peak_w` | T E S | W | Peak power of the braking resistor. The simulated resistor never takes more. A drive by name sets it (S210) | `braking-resistor-peak` | placeholder (T S); catalogue (E, 5.7 kW) |
| `braking_resistor_time_constant_s` | T E S D | s | With `effects.braking_resistor_heating`: the resistor's thermal time constant. 60 s; Siemens gives duty cycles (`braking_s120.csv`), not this | none (signal) | estimate, bounds 20 to 300 |
| `braking_resistor_rated_rise_k` | T E S D | K | With `effects.braking_resistor_heating`: the resistor's rise at its continuous power. 200 K | none (signal) | estimate, bounds 100 to 300 |
| `dc_link_overvoltage_v` | T E S | V | DC link voltage at which the drive trips. T S: 257.6 V. A motor by name scales it with the DC link voltage; an S210 converter by name sets its trip level (410 V on 1 AC, 820 V on 3 AC), an S120 Motor Module the Active Line Modules' 820 V (`dc_link_trip_s120.csv`, booksize equipment manual) | `dc-link-overvoltage` (only with `effects.dc_link`) | placeholder (T S); catalogue (E, 820 V) |
| `dc_link_discharge_time_constant_s` | T E S | s | With `effects.dc_link`: time constant R C with which a disabled drive's DC link discharges through its bleeder and the drive's own consumption, down to the bus level the rectifier holds. 130 s in all three: Siemens asks to wait 5 minutes after switching off for the DC link to discharge, taken as 600 V falling to 60 V, 300 s / ln 10. Required with `effects.dc_link`. No cycle uses it, so its bounds get no runs | none | estimate, bounds 30 to 300 s |
| `line_module_rated_power_w` | T E S | W | With `effects.regenerative_supply`: rated power of the line module. 16 kW, the smallest S120 Active Line Module 6SL3130-7TE21-6AA4 in [`line_modules_s120.csv`](../catalog/siemens/line_modules_s120.csv) | `line-module-continuous`: the mean power through it, drawn and fed back | catalogue |
| `line_module_max_power_w` | T E S | W | With `effects.regenerative_supply`: the most it feeds back; the simulated line module never feeds back more. 35 kW, the same module's maximum power. Required with the effect | `line-module-peak`: the highest power through it either way | catalogue |
| `line_module_efficiency` | T E S | fraction | With `effects.dc_link`: the line module's (or an S210's rectifier's) efficiency both ways: of what it draws from the line that share arrives in the DC link, of what it feeds back (`effects.regenerative_supply`) that share arrives in the mains; the rest is its loss (budget row `line_module`). Above 0, at most 1. 0.98 from the catalogue's 0.28 kW loss at 16 kW; at the small powers of these machines a guess. Required with `dc_link` since 14 November 2026; its bounds get runs only with it | none | estimate, bounds 0.95 to 0.99 |

The DC link ratings of T and S are placeholders for the drive those configs were written for: a DC link of 155.13 V, with 6.2 J of room in 470 µF up to the threshold. When a catalogue motor goes into such a config, the threshold and the trip level keep their ratio to the DC link voltage (782 V and 897 V at 540 V); capacitance and resistor powers stay. See the [user guide](user-guide.md#check-a-real-catalogue-machine).

## `[profile]`

| Key | In | Unit | Meaning | Source | Effect switch |
| --- | --- | --- | --- | --- | --- |
| `cycle_s` | T E S | s | Cycle of the profile generator. Can be changed while a motion is under way | choice | always |
| `fieldbus_delay_cycles` | T E S | cycles | Whole profile cycles between the controller computing the motor setpoints and the drives using them. 0 to 1000 | datasheet, measurement | `fieldbus_delay` |
| `fieldbus_jitter_cycles` | T E S | cycles | The most whole profile cycles a frame can take beyond the delay; each frame takes 0 to this many, evenly. 0 to 1000; 1 in all three files | measurement | `fieldbus_jitter` |
| `setpoint_interval_s` | T E S | s | Setpoints from the PLC are interpolated over this interval. A whole multiple of `cycle_s` | choice | always (OPC UA) |
| `blend_time_s` | T E S | s | How long before the end of a cycle segment the next one may start | choice | `corner_blending` |
| `shaper_frequency_hz` | T E S | Hz | Resonance the input shaper is set to | measurement, or the machine's own figure | `input_shaping` |
| `shaper_damping` | T E S | fraction | Damping ratio of that resonance. From 0 to below 1 | estimate, measurement | `input_shaping` |
| `shaper2_frequency_hz` | T E S | Hz | A second resonance: a second ZVD shaper at this frequency is convolved with the first, so that both are suppressed; each move then takes both periods longer. Not negative; 0 for none (T S). E: 17.5 Hz, the peak of the tool's response, so that the worst corner of the bounds passes `settling` (0.095 mm against 0.115 mm with the 25 Hz shaper alone). With `effects.shaper_follows_pose` it follows the machine's second mode instead, where the machine reports one | measurement (`sim-cli sweep`), or the machine's own figure | `input_shaping` |
| `shaper2_damping` | T E S | fraction | Damping ratio of the second resonance. From 0 to below 1 | estimate, measurement | `input_shaping`, with `shaper2_frequency_hz` above 0 |
| `shaper_type` | all | – | The shaper of the first and second resonance: `"zv"`, `"zvd"`, `"zvdd"`, `"ei"` (extra-insensitive) or `"ei2"` (two-hump EI); `"zvd"` in every file (as before 14 November 2026). Their weights and what each leaves at ±20 % are in the [model reference](model-reference.md#input-shaper) | choice | `input_shaping` |
| `shaper_ei_tolerance` | all | fraction | The vibration the EI shapers allow, V: EI leaves V at its frequency, the two-hump EI humps of V either side. Above 0, below 1; 0.05 in every file | choice | `input_shaping`, with `ei` or `ei2` |
| `shaper_axes` | all | – | The axes (ids) the first and the second shaper act on; `[]` (in every file) for all | choice | `input_shaping` |
| `shaper_modes` | all | – | Further resonances, each a shaper convolved with the others on its axes: `[{ frequency_hz = 40.0, damping = 0.02, type = "ei", axes = ["z"] }]`, `type` (else `shaper_type`) and `axes` (else all) may be left out. Frequencies above 0, dampings from 0 to below 1. They keep their frequency with `shaper_follows_pose`. `[]` in every file | measurement (`sim-cli modes`, `sweep`) | `input_shaping` |
| `shape` | all | – | The shape of moves (`MoveTo`, cycle segments, the conveyor's are S-curves): `"s-curve"` (seven segments), `"sin2"` (Bestehorn's sin², a = (π/2) v / T_ramp), `"poly345"` or `"poly4567"` (VDI 2143, no cruise). A `[[cycle.segment]]` may name its own. `"s-curve"` in every file | choice | always |
| `deceleration_ratio` | all | fraction | Braking within this share of the axes' acceleration limits; the S-curve and sin² then brake on a ramp of their own, a polynomial law keeps the lower limit. Above 0, at most 1; 1 in every file | choice, measurement (a drive that brakes into a small DC link) | always |
| `vertical_axis` | all | – | The axis that lifts against gravity, by id (`"z"`), with `-` before it where up is its minus direction (`"-z"`); `""` (in every file) for none | the machine | always |
| `up_acceleration_ratio` | all | fraction | The share of the vertical axis's acceleration limit it has for an acceleration upwards (the motor lifts the weight as well); downwards the full limit. Above 0, at most 1; 1 in every file | estimate: (F / m − g) / (F / m + g) of the axis's force and moved mass | always, with `vertical_axis` |
| `trajectory_spline` | all | – | The curve `Command::Trajectory` runs through its poses (an imported trajectory, a planned path, a segment's via points): `"c2"` (in every file), the cubic spline whose acceleration is continuous at every pose, or `"hermite"`, the cubic with the slopes of the neighbours as before 14 November 2026, whose acceleration steps at each pose | choice | always |
| `torque_reserve` | T E S | fraction | Share of the torque-speed curve a time-optimal path may use, and of the torque of the drive's maximum current (with `effects.drive_current_limit`) and of the gearbox's and pulley's torque rating at the output (`ratings.gearbox_max_torque_nm`, `pulley_max_torque_nm`). Above 0, at most 1 | choice | `--time-optimal` |
| `tension_reserve` | T E S | fraction | Share of the room between the belt tension that holds the tool and `ratings.belt_max_tension_n` / `belt_min_tension_n` (each less `tension_margin_n`) that a time-optimal path may use in the plan's rigid model. 0.5 in all three files: a change of acceleration quick against the belt's period makes a lightly damped belt overshoot by up to the change itself, so the full model swings up to twice the rigid model's dynamic tension (0.8 let the default T-gantry pass a 330 N rating by 1.4 N at 100 m/s³ and go slack at 1000 m/s³). Above 0, at most 1. Without a tension rating the belt is no limit of the plan | choice (E: estimate) | `--time-optimal` |
| `tension_margin_n` | T E S D | N | Taken off both belt tension ratings before `tension_reserve`, for the guide friction the belt pulls, which the loss-free rigid model leaves out: the X guide's coulomb friction plus its viscous friction at the axis' top speed (T 15 + 20 · 0.8 = 31 N, E 20 + 20 · 2.5 = 70 N, S 15 + 20 · 1.0 = 35 N; D 0, no belt). At least 0. A path whose holding tension alone is beyond a rating less the margin is refused | estimate | `--time-optimal` |
| `min_clearance_m` | T E S D | m | The least clearance the machine keeps from every obstacle of `obstacles` (surface to surface, between the obstacle and the parts that can hit one): the check `collision` judges the run's least clearance against it, and a time-optimal path that comes nearer is refused (`error-path-collision`). Also, for `sim-cli optimise` and the app's optimiser: how far the tool has to stay beyond the farther of pick and place (along the last axis, the side the lift goes to) while it traverses between them. The optimiser lowers each lift, traverse and lower of the cycle to this, and rounds its corners through a via point only where the planned path keeps it (less the path tolerance). Not negative. T 0.3 and E 0.17: the lifts as configured (no obstacle is known); S 0 (no lift); D 0.025, its 25 mm lift | choice, or the height of an obstacle | `optimise` |
| `path_tolerance_m` | T E S | m | How far a planned path may leave the lines, arcs and splines of its file, so that transition curves can take it from a line into an arc at speed, and a spline's end can be eased to the curvature of the line or arc it meets. The deviation is L² / (24 r) for curves of length L at an arc of radius r, 0.01728 dk l² for a spline eased over l against a step dk of the curvature, and is reported. Not negative; 0 (in all three files) plans the path as given | choice | `--path`, and the app's Path |
| `corner_tolerance_m` | T E S D | m | How far a planned path may leave a corner between two lines (the direction turns by more than 1 mrad) to round it instead of stopping there, as a CNC's G642 or a robot's zone: a curve from d before the corner to d after it that leaves and arrives in the lines' directions with no curvature at either end (no step in any axis's acceleration), d such that the curve is nowhere further than this from the two lines (d = 16/3 of it at a right angle, where the curve's tightest radius is sqrt(2) d / 3 = 2.51 times it), at most half of either line. Applies to paths (`--path`, the app, OPC UA) and to cycle segments with via points. A turn of nearly half a circle, a corner next to an arc or a spline, and a corner on more than six axes stay stops. Whether it is quicker depends on the jerk limit: at the shipped 100 m/s³ a 20 mm rounding of a right angle on the T-gantry is slower than the stop (1.365 s against 1.306 s time-optimal), at 2000 m/s³ quicker (0.920 s against 0.978 s). At least 0; 0 in every shipped file: corners are stopped at | choice | none |
| `cycle_timing` | T E S D | text | How the cycle's segments are timed: `"axis_limits"` (the S-curves within `[axis.*]`, and a segment with via points by `path::plan`), or `"time_optimal"` (each segment rest to rest as fast as the motors allow, `path::plan_time_optimal`: the motors' torque-speed curves within `torque_reserve`, the gearbox's and pulley's ratings, the belt's tensions, the guides' load rating and the jerk limits, the straight way as a line and a segment with via points along its path). With `"time_optimal"` the config is refused (`profile.cycle_timing`) where a segment's straight way cannot be planned so from the segment before, and corner blending (`profile.blend_time_s`) does not apply. `analyze`, `verdicts`, `search` and `optimise` then judge the cycle at the motors' limits. `"axis_limits"` in every shipped file | choice | none |

## `[bounds]`

Low and high bound of the values that are estimates. The table mirrors the config: where the config has `[friction_x] coulomb_n = 15.0`, `[bounds]` has `friction_x.coulomb_n = [10.0, 20.0]`, in the unit of the value. Used by `sim-cli verdicts --bands`, by `sim-cli sensitivity` and by the app's sensitivity ranking.

- A bound on a key the config does not have is refused.
- A bound has to be two numbers with low ≤ high.
- A test holds every bound around its value (`the_bounds_of_every_config_bracket_their_values`).
- Arrays are not shown in the parameter panel, so bounds cannot be edited in the app.
- A bound follows a part named in `[parts]` (or by `--motor`, `--supply`, `--drive`, ...): where the part puts its own value in place of the config's, the bound is scaled by the same factor, the part's value over the config's (a bound of a value that was 0 stays as written). So a band written for the config's own motor and drive keeps its width relative to the value around the named part's: EXCT-30's braking threshold has bounds of 750 to 790 V around its 770 V on a 540 V bus; a 230 V motor and drive by name scale the threshold and the bounds with the bus. Before, the bounds stayed absolute and `verdicts --bands` aborted there (`out of range: ratings.braking_threshold_v`), and the DC link capacitance and base load current bands stayed those of the config's drive.

| Config | Bounded values | What is bounded | What is not |
| --- | --- | --- | --- |
| T | 37 | Belt stiffness, pretension, damping, tooth force, tooth jump factor; guide Coulomb, stiction and viscous friction of both axes; motor shaft friction, ripple, cogging; gearbox efficiency, stiffness, damping; frame stiffness and damping; the three `tool_flex` values; the thermal resistances and time constants of `[thermal]`; the four estimates of `[thermal_feedback]`; the DC link's discharge time constant and the line module's efficiency; the belt's teeth in mesh; the bearing lubricant's viscosity and contamination factor; the gearing's Wöhler exponent | Masses, inertias, backlash; values of effects that are off by default, but for `[thermal]`, `[thermal_feedback]` and the tooth jump factor; the shape of the friction curve (Stribeck speed, bristles, preload); the rated rises of `[thermal]`; the reference temperature; the span resonance band (a limit) |
| S | 29 | As T, without `friction_z`, `frame` and `tool_flex` | As T |
| D | 32 | The geometry and the masses (not the payload), the rods' spin inertia, the joint's friction, the motor's friction, ripple, cogging and winding resistance, the gearbox's stiffness and damping, `[thermal]`, three of `[thermal_feedback]`, the DC link's discharge and the line module's efficiency, the gearing's Wöhler exponent, the upper arms' bending stiffnesses and dampings (in and out of their planes) | The stop angles and stiffness, the working range, the axis limits, the loop gains, every `[catalogue]` value |
| D2 | 26 | As D, with the gearbox's backlash in place of its stiffness (the stiffness is the catalogue's, the backlash an estimate) | As D |
| D4 | 50 | As D, with the arms' backlash, and the estimates of `[gearbox.4]` (backlash, damping), `[motor.4]` (winding, friction), `[ratings.4]`, `[tool]` and `[shaft]` | As D; the shaft's slide smoothing speed |
| H4 | 43 | As D, with the arms' backlash (an estimate here), and the estimates of `[plate]` (pinion radius, gear ratio, the amplifier's stiffness, damping and backlash) and `[tool]` | As D; the upper arms have no bending (no `[upper_arm]`) |
| E | 75 | Every `[estimate]` of the file, including geometry estimates, belt mass, all friction values, winding resistance and inductance, brake torque, pulley inertia, frame mass, idler, cable chain, process force, hard stops, sensor noise, the estimated ratings (the discharge time constant and the line module's efficiency among them), the thermal resistances and time constants, the tooth jump factor and the four estimates of `[thermal_feedback]` | The ripple and cogging orders and the loop gains; all `[catalogue]` values; the rated rises of `[thermal]`; the span resonance band (a limit) |

Keys with bounds in one config only:

| Bounded in T and S, not in E | Bounded in E only |
| --- | --- |
| `belt.stiffness_ea_n` (a catalogue value in E), `gearbox.efficiency` (1.0 in E) | `geometry.span_min_m`, `.z_com_drop_m`, `.guide_spacing_m`, `.z_tool_drop_m`; `belt.mass_kg_m`, `.tooth_mesh_stiffness_n_m`, `.damping_time_s`; `friction_x` and `friction_z`: `.stribeck_m_s`, `.bristle_stiffness_n_m`, `.bristle_damping_n_s_m`, `.preload_n`; `motor.friction_smoothing_rad_s`, `.resistance_ohm`, `.inductance_h`, `.brake_torque_nm`; `pulley.inertia_kgm2`; `frame.mass_kg`; all of `idler`, `cable_chain`, `process`, `hard_stop`; `drive.encoder_noise_counts`, `.current_noise_a`; `ratings.dc_link_capacitance_uf`, `.braking_threshold_v`, `.braking_resistor_continuous_w`, `.braking_resistor_peak_w`, `.dc_link_overvoltage_v`, `.shaft_max_radial_load_n`, `.guide_max_load_n`, `.guide_dynamic_load_rating_n`, `.pulley_bearing_load_rating_n` |

## `[start]`

| Key | In | Unit | Meaning |
| --- | --- | --- | --- |
| `pose` | T E S | m | Tool pose the simulation starts at, at rest: one position per axis. T: `[0.1, 0.35]`, E: `[0.1, 0.2]`, S: `[0.1]`. A path, a trajectory and a replayed trace have to start here |

## Delta picker D3

The D3's config ([`machine-delta-d3/default.toml`](../crates/machine-delta-d3/default.toml)) has the tables every machine has as T and S have them: `[sim]`, `[parts]`, `[motor]`, `[gearbox]`, `[thermal]` (gearbox values only), `[thermal_feedback]`, `[drive]`, `[ratings]` (without the belt, pulley and guide ratings), `[profile]`, `[bounds]`, `[start]`, `[[cycle.segment]]`, and the effects of the drives and the controller. It has no belt, guide, pulley, frame, idler, cable chain, process force or `[temperature]`. Its own keys:

`[effects]`

| Key | Meaning | Default |
| --- | --- | --- |
| `joint_elasticity` | The gearboxes twist (stiffness, damping, backlash): motor and arm are two bodies per joint. Off: the rigid reference model (the switch that `belt_stretch` is on the belt machines) | true |
| `forearm_inertia` | Each forearm pair a rigid rod with its mass and rotational inertia (m/6 at each end, 2m/3 at its middle: exact for a slender rod), and the rods' spin about their own axes as the pair's plane turns (`mass.forearm_rod_spin_inertia_kgm2`; one switch, as it is the rest of the same rigid body's inertia). Off: the lumped model, `mass.forearm_elbow_share` at the elbow, the rest at the platform | true |
| `link_elasticity` | The upper arms bend (`[upper_arm]`): in the plane they turn in, the elbow turns against the arm's root on a spring and damper; out of it, the elbow moves along its axis on the tube's lateral spring and damper. The platform rings after a move. The stiffnesses are estimates | false |
| `arm_tube_inertia` | With `link_elasticity`: the arm tube's own share of the bending modes, as a cantilever's first mode (a quarter of its mass, m L_a² / 4 of its inertia and 0.3915 m L_a of its first moment move from the root to the elbow; a quarter of its mass moves with the elbow out of the plane), and the elbow block's twist as the arm bends out of its plane (`mass.elbow_twist_inertia_kgm2` and the forearms' share at the rods' ends, times (1.3765 / L_a)²) | false |
| `joint_friction` | Friction in the arms' joints at the base, `[joint]` | true |
| `hard_stops` | The stops at the ends of the arms' angle range, `[joint]` | true |
| `gravity`, `payload`, `motor_friction`, `gearbox_backlash`, `gearbox_efficiency`, `torque_ripple`, `holding_brake` | As on the other machines | true |

`[geometry]` (all estimates, bounded)

| Key | Unit | Meaning | Value |
| --- | --- | --- | --- |
| `base_radius_m` | m | Centre of the base to a shoulder axis, R | 0.2 |
| `platform_radius_m` | m | Centre of the platform to a forearm pair's centre, r. The equations use e = R − r | 0.05 |
| `upper_arm_m` | m | Upper arm, shoulder axis to elbow, L_a | 0.4 |
| `forearm_m` | m | Forearm, elbow to platform (the parallelogram's centre line), L_b. Longer than the upper arm | 0.95 |
| `forearm_spacing_m` | m | Distance of the two rods of a pair; for the 3D view only | 0.1 |

`[workspace]`

| Key | Unit | Meaning | Value, source |
| --- | --- | --- | --- |
| `radius_m` | m | Radius of the working cylinder; also the x and y axis ranges ±radius | 0.6, catalogue (Autonox: Ø1200) |
| `top_z_m` | m | Height of its top, negative below the shoulder plane; the upper end of the z axis range | −0.675, estimate (the bottom sphere's centre on the shoulder plane) |
| `height_edge_m` | m | Its height at the edge | 0.2, catalogue |
| `height_centre_m` | m | Its height at the centre, at least the edge's: the bottom is a sphere through both; the lower end of the z axis range is top less this | 0.386, catalogue (Autonox: approx. 386, sphere R1060.74) |
| `max_condition_number` | – | A target whose Jacobian has a higher condition number is refused (`error-delta-singularity`) | 10, estimate |

`[mass]` (all estimates but the payload, bounded)

| Key | Unit | Meaning | Value |
| --- | --- | --- | --- |
| `upper_arm_kg` | kg | An upper arm without the elbow block | 0.9 |
| `upper_arm_cg_m` | m | Its centre of mass from the shoulder axis | 0.2 |
| `upper_arm_inertia_kgm2` | kg m² | Its inertia about the shoulder axis (uniform tube: m L²/3) | 0.048 |
| `elbow_kg` | kg | The elbow block with its ball studs, at the arm's end | 0.3 |
| `forearm_pair_kg` | kg | A pair of forearm rods with their ball cups | 0.5 |
| `forearm_elbow_share` | share | The share of a pair's mass counted at the elbow, the rest at the platform (0 to 1), with `effects.forearm_inertia` off. 2/3 matches the rod's inertia about its end (Codourey, Olsson) | 0.6667 |
| `forearm_rod_spin_inertia_kgm2` | kg m² | Each rod's inertia about its own axis, with `effects.forearm_inertia`. A carbon tube 20 × 1.5 mm of 0.15 kg: m (r_o² + r_i²) / 2 = 1.29e-5; bounds 0.6e-5 to 2.6e-5 | 1.3e-5, estimate |
| `elbow_twist_inertia_kgm2` | kg m² | With `effects.arm_tube_inertia`: the elbow block's inertia about the arm's normal through it. An aluminium block of 0.3 kg, 0.1 m across: m w² / 12 = 2.5e-4; bounds 1.2e-4 to 5e-4 | 2.5e-4, estimate |
| `platform_kg` | kg | The platform with its flange | 1.2 |
| `payload_kg` | kg | Payload, with `effects.payload` | 3.0, catalogue (Autonox rated payload) |

`[upper_arm]` (with `effects.link_elasticity`; estimates, bounded)

| Key | Unit | Meaning | Value |
| --- | --- | --- | --- |
| `bending_stiffness_nm_rad` | N m/rad | The arm's bending as a turn of the elbow about the shoulder: 3 E I / L_a (a cantilever's tip deflection F L³ / (3 E I) over L_a). A carbon tube 50 × 2 mm, E 100 GPa: 65250; bounds 40000 to 100000 | 65000 |
| `bending_damping_nm_s_rad` | N m s/rad | Its damping, about 2 % of critical at the platform's sideways mode; bounds 2 to 10 | 5.0 |
| `lateral_stiffness_n_m` | N/m | The arm's bending out of the plane it turns in, as a move of the elbow along its axis: 3 E I / L_a³ (the same round tube): 407800; bounds 250000 to 620000 | 400000 |
| `lateral_damping_n_s_m` | N s/m | Its damping, about 2 % of critical with the elbow's share of mass (0.47 kg with the rods); bounds 6 to 30 | 15.0 |

`[joint]` (read by the rotary joint component; estimates)

| Key | Unit | Meaning | Value |
| --- | --- | --- | --- |
| `friction_coulomb_nm` | N m | Coulomb friction of an arm's joint, at the arm, with `effects.joint_friction` | 0.3 |
| `friction_viscous_nm_s_rad` | N m s/rad | Viscous friction of the joint | 0.05 |
| `friction_smoothing_rad_s` | rad/s | The Coulomb step is smoothed with tanh over this speed | 0.01 |
| `min_angle_deg`, `max_angle_deg` | ° | The stops: arm angle, 0 horizontal and positive down. A target that asks an arm beyond them is refused (`error-delta-joint-stop`); with `effects.hard_stops` a spring and damper act beyond them | −55, 95 |
| `stop_stiffness_nm_rad`, `stop_damping_nm_s_rad` | N m/rad, N m s/rad | The stops' spring and damper | 2e4, 20 |

`[axis.x]`, `[axis.y]`, `[axis.z]`: the tool's speed, acceleration and jerk limits along x, y and z, as for the other machines' axes: 3 m/s, 50 m/s², 2500 m/s³ (estimates; Autonox publishes none).

`[ratings]`: one more rating, judged on any machine that reports the signal `kinematics.condition`:

| Key | Unit | Meaning | Judged by | Value |
| --- | --- | --- | --- | --- |
| `max_condition_number` | – | The highest condition number of the Jacobian the cycle may reach | `singularity` | 5, estimate (a design rule; the working range has up to 3.6) |

D's values of the shared tables: motor 1FK2105-4AF at 400 V (catalogue: rotor 1.7 kg cm², 1.07527 N m/A, 15 N m peak, 4.6 N m rated at 3000 r/min, 18 A and 4.35 A, 37 min; winding R and L estimates), gearbox Siemens NRB080-05-20 (catalogue: ratio 20, 96 %, 35752.6 N m/rad, 9 arcmin backlash, 0.406 kg cm² at the input), converter 6SL3210-5HE11-5UF0 (5 A, 19 A), DC link capacitance and chopper threshold estimates as the EXCT-30's, the converter's internal braking resistor (100 W, 5.7 kW) and trip level (820 V) from `braking_s210.csv`, `settling_tolerance_m` 0.5 mm (an estimate of the positioning accuracy of the class, Festo EXPT ±0.5 mm; the gearboxes' sag under the platform's weight puts the tool 0.64 mm off at pick and place at rest and 0.68 mm in the dwell, so the cycle fails `settling`: a sizing finding, the NRB080's backlash is too large for the class; 3 arcmin pass with 0.33 mm), `gearbox_reference_speed_rpm` 100 (estimate), `gearbox_gearing_wohler_exponent` 8.7 (estimate, bounds 6.6 to 10; arm 1's NRB080 runs up to 137 N m against its 120 N m rated, and its gearing life comes out at about 9000 h: it fails 20000 h, a finding about the estimate), the velocity loop 0.25 N m s/rad, 0.03 s with 0.1 ms filters (tuned with `sim-cli margins delta-d3 --bands`). Its cycle is the Adept cycle 25/305/25 mm with stops at the corners and 35 ms gripper dwells, one pass = one place and one pick; `profile.min_clearance_m` is its 25 mm lift, `start.pose` above the place position (near the edge of the range one arm carries little of the weight and its velocity loop cannot be measured), `mass.forearm_elbow_share` bounded 0.5 to 0.667. The suite's loss-free configs (`suite_configs`) set `workspace.top_z_m` to −0.85, see the [model reference](model-reference.md#delta-picker-d3).

## Delta picker D2

The D2's config ([`machine-delta-d2/default.toml`](../crates/machine-delta-d2/default.toml)) has the D3's tables and keys, with these differences. The tool pose, `start.pose` and the cycle's targets are (x, z); there is no `[axis.y]`.

| Key | Unit | Meaning | Value, source |
| --- | --- | --- | --- |
| `geometry.base_radius_m`, `platform_radius_m`, `upper_arm_m`, `forearm_m` | m | As on the D3, in the x-z plane: the shoulder axes at x = ±R | 0.15, 0.05, 0.4, 0.7, estimates (the whole range reachable with condition numbers up to 1.56) |
| `geometry.forearm_spacing_m` | m | The rods of a pair apart along y; for the 3D view only | 0.1 |
| `workspace.half_width_m` | m | Half the width of the working range; also the x axis range ±half width. In place of the D3's `radius_m` | 0.4, catalogue (Codian: 800 mm) |
| `workspace.top_z_m` | m | Height of its top below the shoulder axes; the upper end of the z axis range | −0.4, estimate |
| `workspace.height_edge_m`, `height_centre_m` | m | Its height at the edges and at the centre; the bottom is an arc through both | 0.25 estimate, 0.477 catalogue (Codian: 477 mm) |
| `mass.upper_arm_kg`, `upper_arm_cg_m`, `upper_arm_inertia_kgm2`, `elbow_kg`, `forearm_pair_kg`, `forearm_elbow_share`, `platform_kg` | | As on the D3 | 1.2, 0.2, 0.064, 0.4, 0.6, 0.6667, 1.0, estimates |
| `mass.payload_kg` | kg | Payload | 5, catalogue (Codian D2-800-S020) |
| `joint.min_angle_deg`, `max_angle_deg` | ° | The stops | −60, 85, estimates |
| `gearbox.backlash_rad` | rad | Backlash of an arm's gearbox | 2.909e-4 (1 arcmin), estimate, bounded 0.5 to 2 arcmin: a reduced-backlash precision planetary. The NRB080-05-20, whose other values the config has (catalogue), has 9 arcmin, with which the cycle fails `settling` (0.85 mm against 0.5 mm) |

The motor, drive, ratings, axis limits (3 m/s, 50 m/s², 2500 m/s³ along x and z) and loop gains are the D3's.

## Parallel SCARA

The parallel SCARA's config ([`machine-parallel-scara/default.toml`](../crates/machine-parallel-scara/default.toml)) has
the D2's tables and keys with the D2's values, laid in the horizontal x-y plane, with these differences. The tool pose,
`start.pose` and the cycle's targets are (x, y, z, c) with both extra axes on, (x, y, z) or (x, y, c) with one, (x, y)
with none: switching an axis off needs the cycle's targets and `start.pose` changed with it.

| Key | Unit | Meaning | Value, source |
| --- | --- | --- | --- |
| `geometry.forearm_spacing_m` | m | The rods of a pair apart along z (the joints' axis); for the 3D view only | 0.1 |
| `workspace.near_y_m` | m | The range's near edge, y in front of the shoulder axes; the upper end of the y axis range. In place of the D2's `top_z_m` | −0.4, estimate |
| `workspace.depth_edge_m`, `depth_centre_m` | m | The range's depth at the edges and at the centre; the far edge is an arc through both. In place of the D2's `height_edge_m`, `height_centre_m` | 0.25 estimate, 0.477 catalogue (the D2's) |
| `mass.upper_arm_inertia_kgm2` | kg m² | The arm's tube about its joint; the D2's `upper_arm_kg` and `upper_arm_cg_m` are not needed (its weight is out of the plane) | 0.064, estimate |
| `z_axis.enabled` | | The Z stroke at the plate: axis z, motor 3 | true |
| `z_axis.unit_kg` | kg | What the plate carries for it: housing, screw, Z motor with brake | 2.95, estimate (the motor's 1.55 kg catalogue) |
| `z_axis.carriage_kg` | kg | What the screw lifts: quill, flange, gripper (the payload is added while held) | 1.0, estimate |
| `slide.*` | | The Z carriage's stroke (0.2 m), friction and stops, as for the screw axis | estimates, the SCARA's quill |
| `screw.*` | | The Z stroke's ball screw (16 x 20, fixed-free, 0.3 m), as for the screw axis; its checks `screw-*` | estimates, the SCARA's |
| `tool_turn.enabled` | | The tool's turn about z: axis c, motor 4 | true |
| `tool_turn.unit_kg` | kg | What the plate carries for it: C motor with brake and its gearbox | 2.43, catalogue (1FK2103-2AH 1.55 kg, NRB060-03-8 0.88 kg) |
| `tool_turn.inertia_kgm2`, `payload_inertia_kgm2` | kg m² | Quill, flange and gripper about the tool's axis; the payload's, while held | 2e-3, 8.33e-3 (5 kg, 100 x 100 mm), estimates |
| `tool_turn.friction_coulomb_nm`, `friction_viscous_nm_s_rad`, `friction_smoothing_rad_s` | | The quill's spline and bearings (effects.joint_friction) | 0.02, 1e-4, 0.01, estimates |
| `axis.y.*` | | The plate's limits along y | 3 m/s, 30 m/s², 2500 m/s³ |
| `axis.x.max_acceleration_m_s2`, `axis.y.max_acceleration_m_s2` | m/s² | 30, not the D2's 50: the plate carries 12.8 kg against the D2's 6.4 kg; at 50 the arms' gearboxes see 235 N m of their 192 N m | estimate |
| `axis.z.*` | | The stroke's limits | 0.9 m/s (2700 r/min on the 20 mm lead), 15 m/s², 1000 m/s³, estimates |
| `axis.c.*` | | The turn's range and limits: no stops, the range is the controller's | ±360°, 30 rad/s, 600 rad/s², 30000 rad/s³, the D4's |
| `[motor.3]`, `[motor.4]` | | The Z and C motors over `[motor]` | 1FK2103-2AH at 400 V (catalogue), resistance and inductance estimates as the D4's tool motor |
| `[gearbox.3]` | | The Z motor's coupling: ratio 1, nothing warms | estimates |
| `[gearbox.4]` | | The turn's gearbox | NRB060-03-8, ratio 8 (catalogue) |
| `[drive.3]`, `[drive.4]` | | The Z and C drives over `[drive]` | 1.95 N m peak, 0.59 rated (catalogue); velocity gain 0.0443 (Z), 0.1 with 10 ms (C) |
| `drive.velocity_kp_nm_s_rad`, `velocity_tn_s` | | The arms' velocity loop | 0.33, 80 ms: the D2's 0.25 and 30 ms leave arm 2 at 31° phase margin with the heavier plate |
| `[ratings.3]`, `[ratings.4]` | | The S210 6SL3210-5HE10-4UF0's currents and braking resistor; the turn's gearbox | catalogue |

Without `effects.gravity` the Z motor holds nothing either.

## Linear delta

The linear delta's config ([`machine-delta-linear/default.toml`](../crates/machine-delta-linear/default.toml)) has the tables every machine has as T and S have them: `[sim]`, `[parts]`, `[motor]`, `[gearbox]` (here a coupling of ratio 1), `[pulley]`, `[hard_stop]`, `[temperature]`, `[thermal]`, `[thermal_feedback]`, `[drive]`, `[ratings]` (with the guide, pulley-bearing and tooth-load ratings of the EGC-80; no gearbox ratings), `[profile]`, `[bounds]`, `[start]`, `[[cycle.segment]]`, the belt's `[belt]` keys the [belt sections](model-reference.md#belt-sections) read (no tooth meshing or tooth jump), and the effects of the drives and the controller. It has no frame, idler, cable chain or process force. Its own keys:

`[effects]`: `belt_stretch` (off: the rigid reference model), `belt_damping`, `belt_slack`, `belt_mass`, `belt_tooth_stiffness`, `belt_material_damping`, `guide_friction`, `load_dependent_friction` (the guides' levels follow their load across the rail at rest: the rods' pull and, on an inclined column, the carriage's weight across it), `part_heating` (the three guides warm with their loss, and the gearboxes where a config has some; off as shipped), `thermal_feedback` (each guide's viscous friction and each gearbox's efficiency follow its temperature; off as shipped), `motor_friction`, `gearbox_backlash`, `gearbox_efficiency`, `torque_ripple`, `temperature`, `hard_stops` (the carriages' stroke ends), `holding_brake`, as on the single axis; the shipped values as there, `belt_tooth_stiffness`, `belt_material_damping` and `temperature` off.

`[geometry]` (estimates, bounded but the stroke and the stop)

| Key | Unit | Meaning | Value |
| --- | --- | --- | --- |
| `column_radius_m` | m | Centre of the frame to a carriage's joint line, R | 0.52 |
| `plate_radius_m` | m | Centre of the plate to a rod pair's centre, r. The equations use e = R − r | 0.07 |
| `rod_m` | m | Rod length, carriage joint to plate joint (the parallelogram's centre line), L | 1.1 |
| `rod_spacing_m` | m | Distance of the two rods of a pair; for the 3D view only | 0.08 |
| `column_incline_deg` | ° | The columns' lean from the vertical, positive with their tops towards the axis, negative away from it; below 60° either way. The carriage's place (`carriage_top_m`, `stroke_m`) is then its travel along the column, its joint standing at that times cos α; `column_radius_m` is measured at z = 0. Not an estimate: a choice of design, no bound | 0 (upright; Festo gives no angle for the EXPT) |
| `carriage_top_m` | m | The carriages' upper stop: their joints' travel along the column from z = 0 (upright: the joints' height) | 0.0 |
| `stroke_m` | m | The carriages' stroke below it | 0.7 |
| `pulley_radius_m` | m | Drive pulley radius (EGC-80-TB: 90 mm per revolution) | 0.014324 |
| `span_min_m` | m | Shortest free belt span between a carriage and either pulley | 0.1 |

`[workspace]`

| Key | Unit | Meaning | Value, source |
| --- | --- | --- | --- |
| `radius_m` | m | Radius of the working cylinder; also the x and y axis ranges ±radius | 0.475, catalogue (Festo EXPT-95: Ø950) |
| `top_z_m`, `bottom_z_m` | m | Its top and bottom, the z axis range | −1.12, −1.22: 100 mm, catalogue (EXPT nominal height); where it hangs an estimate |
| `max_rod_angle_deg` | ° | A target where a rod stands at a larger angle to its column (upright: from the vertical) is refused (`error-delta-linear-rod-angle`) | 60, estimate (the range needs up to 57.2) |
| `max_condition_number` | – | A target whose Jacobian has a higher condition number is refused (`error-delta-singularity`) | 10, estimate (the range has up to 3.98) |

`[mass]` (estimates but the payload, bounded)

| Key | Unit | Meaning | Value |
| --- | --- | --- | --- |
| `carriage_kg` | kg | A carriage (EGC-80 slide) with its ball studs | 1.3 |
| `rod_pair_kg` | kg | A pair of rods with their ball cups | 0.35 |
| `rod_carriage_share` | share | The share of a pair's mass counted at its carriage, the rest at the plate (0 to 1) | 0.5 |
| `plate_kg` | kg | The plate (front unit without rotary drive) | 0.8 |
| `payload_kg` | kg | Payload, with `effects.payload` | 1.0, catalogue (EXPT rated load at the highest dynamics) |

`[friction]`: each carriage's guide, the keys of the single axis's `[friction_x]`, its values.

`[thermal]` (with effects.part_heating): `guide_resistance_k_w` 0.5 K/W, `guide_time_constant_s` 2400 s, `guide_max_rise_k` 40 K, estimates as the single axis's guide (the heat goes into the rail and the column's EGC-80 profile, about 0.35 m² at 10 W/(m² K), fin efficiency 0.6; about 5 kJ/K), bounded 0.25 to 1 K/W and 1200 to 4800 s; `gearbox_resistance_k_w` 1.5, `gearbox_time_constant_s` 1200, `gearbox_max_rise_k` 50 as the delta picker's, read only where a config has a gearbox (ratio not 1), so not bounded.

`[ratings]` of the EGC-80-TB-KF (Festo, "Toothed belt axes EGC-TB-KF", 2024/01):

| Key | Unit | Meaning | Value, source |
| --- | --- | --- | --- |
| `pulley_max_torque_nm` | N m | The axis's driving torque (`pulley-torque`) | 5, catalogue (max. driving torque, page 12; the max. feed force 350 N is the same at the 28.65 mm pulley). Was 10, an estimate |
| `guide_max_load_n` | N | Each guide's highest load across the column (`guide-load`, per carriage) | 3050, catalogue (Fy max = Fz max for 5000 km, page 16) |
| `guide_dynamic_load_rating_n` | N | C of the guide life L10 = (C / P)³ 50 km (`guide-life`, per carriage, at the carriage's speed) | 14157, from the same figure: 3050 (5000 / 50)^(1/3); Festo's 100 km figure (11236 N, page 17) gives 14156 |
| `pulley_bearing_load_rating_n` | N | C of the bearing that takes the belt's pull on the drive shaft (`pulley-bearing-life`) | 5000, estimate (Festo publishes neither bearing nor radial load; a 6002 of the pulley's size has about 5.9 kN), bounded 3000 to 10000 |
| `belt_tooth_load_rpm`, `belt_tooth_load_n_cm` | r/min, N/cm | The belt's specific tooth load (`belt-tooth-load`) | [0, 3333], [15.1, 15.1], estimate: what gives Festo's 350 N with 12 teeth in mesh on the 19.3 mm belt, 350 / (1.93 · 12), up to the axis's 5 m/s |
| `belt_width_mm` | mm | Belt width | 19.3, catalogue (page 13) |
| `belt_service_factor` | – | Factor on the effective pull | 1.0, estimate (Festo's 350 N is already the limit), bounded 1.0 to 1.4 |
| `belt_teeth_in_mesh` | – | Teeth in mesh on the drive pulley (the method counts at most 12) | 15, half the 30-tooth pulley (90 mm / 3 mm pitch) |

The guide's moments (Mx, My, Mz) are not judged.

L's values of the shared tables: motor 1FK2205-2AF at 400 V with brake (catalogue: rotor 4.0 kg cm², 1.241379 N m/A, 10.8 N m peak, 3.0 N m rated at 3000 r/min, 9.5 A and 2.5 A; R, L, thermal time constant and brake torque estimates; the High Dynamic 1FK2104-5AF, 0.65 kg cm², left the velocity loop ringing near 450 Hz against about 6 kg cm² of load), a coupling of ratio 1 (3000 N m/rad, estimate as the EXCT-30's), converter 6SL3210-5HE11-0UF0 (3 A, 10.9 A; its internal braking resistor 50 W, 3 kW, 820 V), belt EA 6e5 N and 300 N pretension (estimates as the EXCT-30's), `settling_tolerance_m` 0.5 mm (catalogue: EXPT positioning accuracy), `belt_max_tension_n` 1000, `pulley_max_torque_nm` 5 (catalogue, EGC-80) and `shaft_max_radial_load_n` 1500 (estimate), `ratings.max_condition_number` 5, the velocity loop 0.25 N m s/rad, 0.02 s with 0.2 ms filters (tuned with `sim-cli margins delta-linear --bands`). Its cycle is the D3's Adept cycle 25/305/25 mm at the top of the range; `start.pose` above the place position.

## Delta picker D4

The D4's config ([`machine-delta-d4/default.toml`](../crates/machine-delta-d4/default.toml)) has the D3's tables and keys with the D3's values, but for the arms' `gearbox.backlash_rad` (8.727e-4, 3 arcmin, estimate, bounded 2 to 4 arcmin: a reduced-backlash precision planetary; the NRB080-05-20's 9 arcmin fail `settling`, 1.21 mm with the tool's 10). The tool pose, `start.pose` and the cycle's targets are (x, y, z, c), c in rad. These keys are its own:

| Key | Unit | Meaning | Value, source |
| --- | --- | --- | --- |
| `effects.shaft_mass` | – | The shaft's weight share at the platform (on the arms) and its spin inertia at the tool | on |
| `effects.shaft_torsion` | – | The shaft twists in series with the tool's gearbox (with `joint_elasticity`) | on |
| `effects.telescope_friction` | – | The telescope's sliding friction along the shaft, a force on the platform that the arms take | on |
| `axis.c.min_angle_deg`, `max_angle_deg` | ° | The controller's range of the tool's turn (the machine has no stops) | −360, 360, choice |
| `axis.c.max_velocity_rad_s`, `max_acceleration_rad_s2`, `max_jerk_rad_s3` | rad/s, rad/s², rad/s³ | The turn's limits | 30, 600, 30000, estimates (ABB publishes none; 30 rad/s is the motor's 240 rad/s behind ratio 8) |
| `motor.4.rotor_inertia_kgm2`, `torque_constant_nm_a`, `rated_current_a`, `thermal_time_constant_s`, `max_current_a`, `torque_constant_at_max_current_nm_a` | | The tool turn's motor over `[motor]` | 3.5e-5, 1.0672269, 1.19, 1980, 4.2, 0.8928571, catalogue (Siemens 1FK2104-4AF at 400 V) |
| `motor.4.resistance_ohm`, `inductance_h` | Ω, H | Its winding | 6, 0.03, estimates (L/R 5 ms as the arms'), the resistance bounded 4 to 9 |
| `motor.4.friction_coulomb_nm`, `friction_viscous_nm_s_rad` | N m, N m s/rad | Its bearings' friction (`effects.motor_friction`) | 0.02, 8e-5, estimates (1.5 % of the static torque), bounded 0.01 to 0.04 and 4e-5 to 1.6e-4 |
| `motor.4.brake_torque_nm` | N m | Its brake | 2, estimate |
| `gearbox.4.ratio`, `efficiency`, `stiffness_nm_rad`, `inertia_at_input_kgm2` | | The tool's gearbox over `[gearbox]` | 8, 0.97, 8250.592, 8.8e-6, catalogue (Siemens NRB060-04-8) |
| `gearbox.4.backlash_rad` | rad | Its backlash | 2.909e-4 (1 arcmin), estimate, bounded 0.5 to 2 arcmin (catalogue: 10 arcmin) |
| `gearbox.4.damping_nm_s_rad` | N m s/rad | Its damping (replaced by the shaft's with `shaft_torsion`) | 5, estimate, bounded 2 to 10 |
| `drive.4.peak_torque_nm`, `rated_torque_nm`, `rated_speed_rad_s`, `no_load_speed_rad_s` | | Its drive over `[drive]` | 3.75, 1.27, 314.159, 619.7016, catalogue (1FK2104-4AF; no-load speed √(3/2) · 540 V / kt) |
| `drive.4.velocity_kp_nm_s_rad`, `velocity_tn_s`, `kv_1_s`, `current_kp_v_a` | | Its loops | 0.05, 0.02, 80, 120: tuned with `sim-cli margins delta-d4 --bands` (at least 37.7°, 9.4 dB; the 0.081 `catalog::with_motor_part` retunes to keeps 34.6°, 5.4 dB), the shorter integral time and higher position gain for `settling`; the current loop's L over 0.25 ms as the shared |
| `ratings.4.gearbox_rated_torque_nm`, `gearbox_max_torque_nm` | N m | The tool gearbox's limits, judged by the common `gearbox-*` checks of motor 4 | 18, 29, catalogue (NRB060-04-8) |
| `ratings.4.gearbox_reference_speed_rpm` | r/min | Its planet bearings' reference speed (`gearbox-life`) | 110, estimate (NRB060 ratio 5's 175 r/min scaled to ratio 8) |
| `ratings.4.drive_rated_current_a`, `drive_max_current_a`, `braking_resistor_continuous_w`, `braking_resistor_peak_w` | A, W | Its S210 converter | 1.2, 4.2, 50, 1200, catalogue (6SL3210-5HE10-4UF0) |
| `ratings.4.drive_base_load_current_a` | A | Its base load current (`effects.drive_overload`) | 1.02, estimate (0.85 of rated), bounded 0.95 to 1.08 |
| `tool.inertia_kgm2` | kg m² | Flange and gripper about the tool's axis | 2e-3, estimate, bounded 1e-3 to 4e-3 |
| `tool.payload_inertia_kgm2` | kg m² | The payload about the tool's axis (with `effects.payload`) | 5e-3, estimate (3 kg, 100 × 100 mm), bounded 2.5e-3 to 1e-2 |
| `tool.friction_coulomb_nm`, `friction_viscous_nm_s_rad`, `friction_smoothing_rad_s` | | The tool bearing's friction (`effects.joint_friction`) | 0.02, 1e-4, 0.01, estimates; the first two bounded |
| `shaft.upper_joint_z_m` | m | Height of the upper cardan joint on the base's axis | 0, estimate, bounded ±0.05 |
| `shaft.lower_joint_offset_m` | m | The lower cardan joint above the platform's centre | 0.05, estimate, bounded 0.03 to 0.08 |
| `shaft.min_length_m`, `max_length_m` | m | Shortest and longest the telescope can be; beyond: `error-delta-shaft-stroke` | 0.6, 1.1, estimates, bounded |
| `shaft.max_cardan_angle_deg` | ° | Largest bend of a cardan joint; beyond: `error-delta-cardan-angle` | 45, estimate, bounded 40 to 50 |
| `shaft.mass_kg`, `platform_share` | kg, – | The shaft's mass and the share of it the platform carries | 1.4, 0.5, estimates, bounded |
| `shaft.spin_inertia_kgm2` | kg m² | Tube, shaft and joints about the shaft's axis, at the tool | 8e-5, estimate, bounded |
| `shaft.torsional_stiffness_nm_rad`, `torsional_damping_nm_s_rad` | N m/rad, N m s/rad | The shaft's twist (`effects.shaft_torsion`) | 500, 0.075, estimates, bounded 250 to 1000 and 0.03 to 0.2 |
| `shaft.slide_friction_n`, `slide_friction_viscous_n_s_m`, `slide_friction_smoothing_m_s` | N, N s/m, m/s | The telescope's sliding friction (`effects.telescope_friction`) | 5, 2, 0.005, estimates; the first two bounded |

The fourth motor has tables of its own, `[motor.4]`, `[gearbox.4]`, `[drive.4]`, `[ratings.4]`, over the shared ones as in a machine file (`catalog::motor_config`): its joint, its drive and its verdicts read them, and `sim-cli search delta-d4 --motor-id 4` sizes it alone. A key left out is the shared one. The arms share `[motor]` and `[gearbox]` (the D3's arms read them): `[motor.1]` to `[motor.3]`, `[gearbox.1]` to `[gearbox.3]` and `[parts.1]` to `[parts.3]` are refused; their `[drive.<id>]` and `[ratings.<id>]` are taken. Until 14 November the fourth motor was the arms' 1FK2105-4AF with their drive, judged against their gearbox's ratings, and `[tool_gearbox]` with `ratings.tool_gearbox_max_torque_nm` (check `tool-gearbox-peak`) stood for its gearbox; a config with those keys now fails as having unknown keys: rename `[tool_gearbox]` to `[gearbox.4]` and the rating to `ratings.4.gearbox_max_torque_nm`.

## Quattro-type delta H4

The H4's config ([`machine-delta-h4/default.toml`](../crates/machine-delta-h4/default.toml)) follows the D3's section by
section: `[geometry]` (with `plate_radius_m` for the D3's `platform_radius_m`), `[workspace]`, `[mass]`, `[joint]`,
`[motor]`, `[gearbox]`, `[thermal]`, `[thermal_feedback]`, `[axis.x]` to `[axis.z]`, `[drive]`, `[ratings]`,
`[profile]`, `[bounds]`, `[start]` and the Adept cycle, with its own values: four arms of 500 mm, forearms of 1100 mm,
working range 1300 mm across, 250 mm high at the edge and 450 mm at the centre (top 750 mm below the shoulders), 6 kg
payload; motor 1FK2105-6AF at 400 V (catalogue: rotor 2.7 kg cm², 1.19403 N m/A, 24 N m peak, 6.6 N m rated, 5.6 A
rated, 24 A maximum), gearbox NRB120-05-20 (catalogue: ratio 20, 96 %, 75630 N m/rad, 1.57 kg cm², 260 N m rated,
416 N m maximum; backlash 3 arcmin, an estimate for a reduced-backlash planetary, bounded 2 to 4 arcmin), converter
6SL3210-5HE12-0UF0 (7 A, 24 A; its internal braking resistor 100 W, 6 kW), the velocity loop 0.5 N m s/rad, 0.03 s
with 0.1 ms filters (tuned with `sim-cli margins delta-h4 --bands`: at least 46.4°, 8.88 dB). It has no `[upper_arm]` and no
`effects.link_elasticity`: the upper arms do not bend. The tool pose, `start.pose` and the cycle's targets are
(x, y, z, c), c in rad. These keys are its own:

| Key | Unit | Meaning | Value, source |
| --- | --- | --- | --- |
| `effects.amplifier_compliance` | – | The tool turns on the amplifier's spring (rack, pinion and gear stage twist), a body of its own; off, it turns with the slide times the ratio | on |
| `effects.amplifier_backlash` | – | With `amplifier_compliance`: the amplifier's play at the tool, a dead band in its spring | on |
| `geometry.plate_radius_m` | m | From a plate part's centre to each of its two forearm pairs (both parts alike) | 0.07, estimate, bounded 0.06 to 0.08 |
| `mass.plate_part_kg` | kg | Each of the two plate parts, without the forearms' share | 0.7, estimate, bounded 0.5 to 1.0 |
| `mass.carriage_kg` | kg | The carriage between the parts at the tool's point: pinion, gear stage, flange | 1.0, estimate, bounded 0.7 to 1.4 |
| `plate.pinion_radius_m` | m | Pitch radius of the pinion between the two racks; the tool turns by `gear_ratio` h / `pinion_radius_m` | 0.04, estimate, bounded 0.038 to 0.042 |
| `plate.gear_ratio` | – | The gear stage between the pinion and the tool | 1.2, estimate, bounded 1.1 to 1.3 (30 rad/m with the pinion: ±180° are h = ±104.7 mm) |
| `plate.amplifier_stiffness_nm_rad`, `amplifier_damping_nm_s_rad` | N m/rad, N m s/rad | The amplifier's twist at the tool (`effects.amplifier_compliance`): the tool and payload ring at 76.5 Hz on it, 159 Hz without the payload | 3000, 0.25 (2 % of critical), estimates, bounded 1500 to 6000 and 0.1 to 0.5 |
| `plate.amplifier_backlash_rad` | rad | The amplifier's play at the tool (`effects.amplifier_backlash`) | 2.909e-4 (1 arcmin), estimate, bounded 0.5 to 2 arcmin |
| `tool.inertia_kgm2` | kg m² | Flange, gripper and the gear stage, about the tool's axis | 3e-3, estimate, bounded 1.5e-3 to 6e-3 |
| `tool.payload_inertia_kgm2` | kg m² | The payload about the tool's axis (with `effects.payload`) | 1e-2, estimate (6 kg, 100 × 100 mm), bounded 5e-3 to 2e-2 |
| `axis.c.min_angle_deg`, `max_angle_deg` | ° | The tool's turn: the controller's range, and so the plate's slide | −180, 180, estimate |
| `axis.c.max_velocity_rad_s`, `max_acceleration_rad_s2`, `max_jerk_rad_s3` | rad/s, rad/s², rad/s³ | The turn's limits | 30, 300, 15000, estimates |

The ratio kappa = `gear_ratio` / `pinion_radius_m` is a trade: the tool's inertia counts in the slide times kappa², and
the arms' twist and backlash count in the turn times kappa. At 60 rad/m (pinion 25 mm, stage 1.5, the first choice) the
slide weighed 47 kg and the drives' loops, tuned for the rotors, let a turn of 1.5 rad overshoot by 0.27 rad and ring at
6 Hz, growing; at 30 rad/m it settles in about 0.3 s.

## Arms (six-axis arm, SCARA, palletiser)

The configs of `urdf-arm` ([`config/arm.toml`](../crates/machine-joint-kit/config/arm.toml)), `scara`
([`config/scara.toml`](../crates/machine-joint-kit/config/scara.toml)) and `palletiser`
([`config/palletiser.toml`](../crates/machine-joint-kit/config/palletiser.toml)) have these keys. The arm and the SCARA
are shipped with tool space off, so the axes are the joints as before; the palletiser with it on.
[`config/scara-tool.toml`](../crates/machine-joint-kit/config/scara-tool.toml) and
[`config/arm-tool.toml`](../crates/machine-joint-kit/config/arm-tool.toml) are the same configs with it on, a start pose
and a cycle of straight lines in tool space (and their settling tolerances for the place and the turn of the tool);
[`config/arm5-tool.toml`](../crates/machine-joint-kit/config/arm5-tool.toml) and
[`config/arm7-tool.toml`](../crates/machine-joint-kit/config/arm7-tool.toml) the five-axis and the seven-axis example arm
in tool space. See [mechanisms](mechanisms.md#tool-space).

| Key | Unit | Meaning | Value, source |
| --- | --- | --- | --- |
| `arm.payload_inertia_kgm2` | kg m² | The payload's inertia about its centre in the URDF's tip frame: xx, yy, zz, xy, xz, yz. With `effects.payload` | arm 0.03125, 0.01625, 0.025 (a 6 kg box of 0.1 x 0.2 x 0.15 m); SCARA 0.004375, 0.004375, 0.00375 (3 kg, 0.1 m across and high); palletiser 0.41667, 0.75, 0.86667 (a 20 kg carton of 0.6 x 0.4 x 0.3 m); estimates |
| `arm.joint_coupling` | – | The URDF's joints as sums of the motors' joints, one row per URDF joint, one factor per motor (a parallelogram); `[]` one motor per URDF joint | `[]`; palletiser rows (1,0,0,0), (0,1,0,0), (0,−1,1,0), (0,0,−1,0), (0,0,0,1): its geometry, choice |
| `arm.payload_swing_stiffness_nm_rad`, `arm.payload_swing_damping_nm_s_rad` | N m/rad, N m s/rad | With `effects.payload_swing`: the spring and damper per axis of the payload's pivot at the flange | arm and SCARA 20, 0.5 (bounds 10 to 40, 0.2 to 1); palletiser 120, 3 (60 to 240, 1 to 6, its carton at 2.2 Hz); estimates |
| `kinematics.tool_space` | – | On: the axes are the flange's pose (`kinematics.tool_axes`), `start.pose` and the cycle's targets too; off: the motors' joints | arm, SCARA off; palletiser on; choice |
| `kinematics.tool_axes` | – | Of "x", "y", "z", "rx", "ry", "rz" in that order: the tool's axes, as many as motors or one fewer (redundant). ["x", "y", "z", "rz"] on a SCARA chain is its closed form with the axis c | arm all six; SCARA x, y, z, rz; palletiser x, y, z, rz; five-axis x, y, z, rx, ry; choice |
| `kinematics.elbow` | – | SCARA only: "right" (joint 2 counter-clockwise from the stretched arm) or "left" | "right", choice |
| `kinematics.home_joints` | rad | Numerical inverse only: the joint pose whose branch the inverse keeps, which the solver starts from where the last solution does not get there, and which a redundant arm stays nearest | arm 0, 0.4, 0.2, 0, 0.97, 0 (flange 0.99 m out, 0.83 m up, pointing down); palletiser 0, 0.2, 0.3, 0; choice |
| `workspace.tool_min`, `tool_max` | m, rad | In tool space: each axis's range, x, y, z in m, the turns in rad (from the flange's orientation at zero joints); poses in it out of reach or beyond a joint's stop are refused | SCARA 0.15 to 0.6, −0.45 to 0.45, 0 to 0.2, −3 to 3; arm 0.5 to 1.2, −0.6 to 0.6, 0.3 to 1.1, then −0.6 to 0.6, 0.97 to 2.17, −0.6 to 0.6 (pointing down ±0.6 rad); palletiser 1 to 1.8, −0.7 to 0.7, 0.6 to 1.6, −1.7 to 1.7; choice |
| `workspace.max_condition_number` | – | In tool space: a target whose Jacobian has a higher condition number is refused (`error-arm-singularity`); SCARA: of d(x, y)/d(joints 1, 2); others: of the tool's axes, the turns times the flange's distance from joint 2 at the home; also the warm-started inverse's guard | SCARA 25 (11° of elbow), arm 30, palletiser 30, estimates (design rules) |
| `axis.x`, `axis.y`, `axis.z`: `max_velocity_m_s`, `max_acceleration_m_s2`, `max_jerk_m_s3` | m/s, m/s², m/s³ | In tool space: the flange's limits along each axis | SCARA 1, 5, 200 (z 0.8, 8, 400); arm 0.9, 4, 80; palletiser 1.2, 4, 40; estimates |
| `axis.c` (SCARA), `axis.rx`, `axis.ry`, `axis.rz`: `max_velocity_rad_s`, `max_acceleration_rad_s2`, `max_jerk_rad_s3` | rad/s, rad/s², rad/s³ | In tool space: the turn's limits | SCARA 4, 30, 1000; arm 1.5, 8, 150 (2 rad/s with every axis at its limit asked 373 rad/s of a motor); palletiser rz 2.5, 10, 100; estimates |
| `axis.j7` | rad/s, ... | Read only where the URDF has a seventh joint | 3, 20, 400, estimate |
| `[motor.<k>]`, `[drive.<k>]`, `[gearbox.<k>]`, `[strain_wave.<k>]`, `[ratings.<k>]`, `[parts.<k>]` | – | A joint's own motor, drive, gear and ratings over the shared tables, k = 1, 2, ... (see [a motor's own tables](#a-motors-own-tables)) | arm: joints 4 to 6 the 1FK2104-5AF on a 6SL3210-5HE10-8UF0 (catalogue), a size-20 strain wave gear (25000 N m/rad, 40 N m rated, 49 and 76 N m ratings; estimates); SCARA: joints 3 and 4 the 1FK2104-5AF; palletiser: joints 1 to 3 the 1FK2106-4AF on a 6SL3210-5HE15-0UF0, the flange the 1FK2104-5AF |

These are choices and design rules, not properties of a part, so they have no `[bounds]` (the swing's spring and damper
have theirs).

`effects.link_bending` (in all joint kit configs, acting on the arms only; off as shipped) and `[link_bending]` (in the
arms' configs): each link from its joint to the next joint (the flange after the last) bends as a cantilever in its
first Euler-Bernoulli mode, in the plane its joint turns it in; its own URDF mass spread along it, everything beyond at
its tip. Signals `link.<n>.bend` (m, its tip's deflection). Estimates: cast aluminium.

| Key | Unit | Meaning | Value, source |
| --- | --- | --- | --- |
| `outer_diameter_m` | m | Per joint, the link as a round tube: its outer diameter; 0 keeps the link rigid. A link along its joint's axis (the arm's base column) does not bend in that plane | arm 0, 0.14, 0.12, 0.1, 0.08, 0; SCARA 0.12, 0.1, 0, 0; estimates |
| `wall_m` | m | Per joint, the tube's wall | arm 0, 0.008, 0.008, 0.006, 0.005, 0; SCARA 0.008, 0.006, 0, 0; estimates |
| `youngs_modulus_pa` | Pa | E of all links: cast aluminium 7e10; bounds 6.5e10 to 7.2e10 | 7.0e10, estimate |
| `damping_ratio` | – | Of every link's mode; bounds 0.01 to 0.05 | 0.02, estimate |

## Slider-crank, rotary knife, flying saw (`machine-crank-cam`)

What the three machines of `machine-crank-cam` read beside `[motor]`, `[gearbox]`, `[drive]`, `[ratings]` and the rest (the
crank and the knife a `[joint]` as the rotary table's, the saw a `[slide]` and `[rack]` as the rack axis's). Formulas in
[mechanisms](mechanisms.md#crank-and-cam-mechanisms-machine-crank-cam). `effects.process_force` switches the press force and the
cuts (on in all three).

| Key | Unit | Meaning | Mark |
| --- | --- | --- | --- |
| `crank.radius_m`, `rod.length_m` | m | Crank radius r and rod length l (l > r) | [estimate] |
| `crank.inertia_kgm2` | kg m² | The crank disc and shaft about their axis | [estimate], [bounds] |
| `crank.motion` | – | With `cam.run`: `"constant"` (one turn per stroke at constant speed) or `"cam"` (`[[cam.segment]]` or `cam.table_file`) | choice |
| `crank.strokes_per_min` | 1/min | The master's speed with `cam.run`: master periods per minute | choice |
| `rod.mass_kg`, `rod.crank_end_share` | kg, – | The rod's mass and the share of it at the crank pin (0 to 1); the rest moves with the slider | [estimate], [bounds] |
| `slider.mass_kg` | kg | The slider (ram) | [estimate], [bounds] |
| `slider.incline_deg` | ° | The guide from the crank towards the slider against the horizontal, −90 to 90 (−90: a ram below the crank) | choice |
| `mass.payload_kg` | kg | With `effects.payload`: the die on the ram, the knife bars at the knife's radius, the saw on the carriage | [estimate], [bounds] |
| `process.distance_m`, `process.force_n` | m, N | The press force over the ram's distance from the bottom dead centre, linear between the points, 0 outside them | [estimate] |
| `process.smoothing_m_s` | m/s | The press force fades in between this ram speed and twice it | choice |
| `ratings.rod_max_force_n`, `ratings.slider_guide_max_load_n`, `ratings.crank_bearing_max_load_n` | N | The verdicts `rod-force`, `slider-guide-load`, `crank-bearing-load` | [estimate] |
| `knife.radius_m`, `knife.inertia_kgm2`, `knife.knives` | m, kg m², – | Knife radius (the blade's tip), the roll's inertia, blades on the roll (a whole number) | [estimate], inertia [bounds] |
| `knife.sync_angle_deg` | ° | The synchronous zone ± about each cut (2 × it below the blade pitch) | choice |
| `knife.format_length_m` | m | Web per cut (more than the zone, 2 R α) | choice |
| `knife.transition_law`, `saw.return_law` | – | `"3-4-5"`, `"4-5-6-7"`, `"cycloid"` or `"harmonic"` | choice |
| `web.speed_m_s`, `material.speed_m_s` | m/s | The master's speed with `cam.run` | choice |
| `cut.angle_deg`, `cut.force_n` (knife) | °, N | The cut's force at the blade's tip over the angle from the cut | [estimate] |
| `cut.position_m`, `cut.force_n` (saw) | m, N | The cut's force on the carriage along the material over the way into the window | [estimate] |
| `saw.home_m` | m | Where the carriage waits; home + stroke within `slide.stroke_m` | choice |
| `saw.cut_length_m`, `saw.cut_window_m` | m | Material per cut, and the window the carriage runs with it while cutting; L > 2 v² / a + W | choice |
| `saw.sync_acceleration_m_s2` | m/s² | The mean acceleration to the material's speed (peak 1.5 times it) | choice |
| `cam.run` | true/false | The controller runs the cam as the cycle instead of `[[cycle.segment]]`. Off in the shipped configs | choice |
| `cam.ramp_s` | s | The master's sin² ramp from rest to speed and back | choice |
| `cam.table_file` (crank) | – | A CSV cam table (master, slave[, slope], a header and `#` lines skipped; the last row closes the period); `""`: the segments | choice |
| `[[cam.segment]]` `master`, `slave`, `law` (crank) | rad | One segment: master span, slave rise, law (`linear`, `dwell`, `3-4-5`, `4-5-6-7`, `cycloid`, `harmonic`) | choice |

All three have `[axis.crank]`, `[axis.knife]` or `[axis.x]` with `max_velocity_*`, `max_acceleration_*` and `max_jerk_*`
as every machine; with `cam.run` the cam's peaks at the master's speed are checked against them (`limit-cam-speed`,
`limit-cam-acceleration`, `limit-cam-jerk`).

## Hexapod

The hexapod's config ([`machine-hexapod/default.toml`](../crates/machine-hexapod/default.toml)) has the tables of the
ball screw axis (`[parts]`, `[effects]`, `[thermal]`, `[thermal_feedback]`, `[profile]`, `[sim]`, `[screw]`, `[gearbox]`
here a coupling of ratio 1, `[motor]`, `[drive]`, `[ratings]`, `[bounds]`, `[start]`, `[[cycle.segment]]`) with the same
meanings; one motor, coupling, screw and drive per leg, all alike. The screw is a 25 x 10 (`lead_m` 0.01, `length_m`
0.35, estimates, bounded as the axis's); `drive.velocity_kp_nm_s_rad` 0.4 (tuned with `sim-cli margins hexapod`: 145
Hz, 89 degrees, 10.6 dB); `profile.vertical_axis` "z". The tool pose, `start.pose` and the targets are x, y, z (m), rx,
ry, rz (rad, a rotation vector); `[axis.x]`, `[axis.y]`, `[axis.z]` take `max_velocity_m_s`, `max_acceleration_m_s2`,
`max_jerk_m_s3` (0.2, 3, 100, estimates), `[axis.rx]`, `[axis.ry]`, `[axis.rz]` the same in rad (0.8, 10, 400,
estimates). Effects as on the ball screw axis: `joint_elasticity` (each leg's coupling, screw, nut and bearings
stretch; off: the rigid reference model), `joint_friction` (the rod's guide and the nut's preload), `hard_stops` (the
legs' stroke ends), the gearbox and motor effects; shipped as there. Its own keys (all estimates; the masses and the
legs' friction bounded):

| Key | Unit | Meaning | Value |
| --- | --- | --- | --- |
| `hexapod.base_radius_m` | m | Circle of the base joints | 0.4 |
| `hexapod.base_half_angle_deg` | ° | A base pair's joints sit at 120 k ± this | 15 |
| `hexapod.platform_radius_m` | m | Circle of the platform joints (also the length the condition number weighs turns by) | 0.2 |
| `hexapod.platform_half_angle_deg` | ° | A platform pair's joints sit at 120 k ± (60 − this) | 10 |
| `leg.closed_length_m` | m | A leg retracted, joint to joint | 0.45 |
| `leg.stroke_m` | m | Its stroke (the joint's range, its stops) | 0.2 |
| `leg.friction_coulomb_n`, `friction_viscous_n_s_m`, `friction_smoothing_m_s` | N, N s/m, m/s | The rod's guide and seal (`effects.joint_friction`), plus the nut's preload drag | 20, 60, 0.002 |
| `leg.stop_stiffness_n_m`, `stop_damping_n_s_m` | N/m, N s/m | Buffers at the stroke's ends (`effects.hard_stops`) | 2e6, 2000 |
| `leg.mass_kg` | kg | A leg: body, motor, rod and nut | 4 |
| `leg.platform_share` | – | The share of it that moves with the platform joint; the rest sits still at the base | 0.3 |
| `mass.platform_kg` | kg | The platform | 25 |
| `mass.platform_inertia_kgm2` | kg m² | Its inertia about its centre, x, y, z (the legs' shares are added) | 0.4, 0.4, 0.7 |
| `mass.payload_kg` | kg | Payload, a point mass at the centre (`effects.payload`) | 30 |
| `workspace.tool_min`, `tool_max` | m, rad | The tool's box, the axes' ranges | −0.06, −0.06, 0.44, −0.08, −0.08, −0.12 to 0.06, 0.06, 0.54, 0.08, 0.08, 0.12 |
| `workspace.max_joint_angle_deg` | ° | Steepest leg against the base's or the platform's normal; beyond: `error-hexapod-joint-angle` | 40 |
| `workspace.max_condition_number` | – | Of the Jacobian, turns weighed by the platform radius; beyond: `error-hexapod-singularity` | 20 |

## Cable robots

The configs of `cable-planar` and `cable-spatial` ([`machine-cable-robot/config/`](../crates/machine-cable-robot/config/),
`common.toml` and the machine's own file) have the tables of the ball screw axis but `[screw]` and `[slide]`, with
`[gearbox]` a planetary gearbox 5:1 at each winch (estimates: 0.95, 5 arcmin, 3.4e4 N m/rad, bounded) and
`profile.vertical_axis` "z". The tool pose is x, z (m) for the planar one, x, y, z (m), rx, ry, rz (rad, a rotation
vector) for the spatial one; the `[axis.<id>]` tables as on the hexapod (planar 1.5 m/s, 10 m/s², 500 m/s³; spatial 1,
8, 400 and 1 rad/s, 10, 400; estimates). `ratings.settling_tolerance_m` is 0.3 mm (planar) and 1 mm (spatial), a
requirement of their size (estimate). Effects:

| Switch | Default | What it models |
| --- | --- | --- |
| `cable_stretch` | on | The cables stretch (EA / l) and go slack, each winch's gearbox twists: platform, drums and rotors are bodies of their own. **Off selects the rigid reference model**: the cables keep their length, the motors turn with them |
| `cable_damping` | on | With `cable_stretch`: the cables' material damping, `cable.specific_damping_n_s` / l |
| `winch_friction` | on | Friction of the winches' bearings and the pulleys, at the cable |

The gearbox, motor and drive effects as on every machine (`hard_stops`: an empty and a full drum). Own keys (all
estimates, values planar / spatial; bounded where they are a part's property):

| Key | Unit | Meaning | Value |
| --- | --- | --- | --- |
| `platform.spatial` | – | Six degrees of freedom; off: a point in the x-z plane (no `anchors_m`, `inertia_kgm2`) | false / true |
| `platform.mass_kg` | kg | The carriage or platform | 3 / 4 |
| `platform.payload_kg` | kg | Payload, a point mass at the centre (`effects.payload`) | 2 / 4 |
| `platform.inertia_kgm2` | kg m² | Spatial: its inertia about its centre, x, y, z | – / 0.03, 0.03, 0.05 |
| `platform.anchors_m` | m | Spatial: each cable's anchor on the platform, in its frame (the planar one's meet at its centre) | – / the corners of 200 x 200 x 100 mm |
| `frame.exits_m` | m | Each cable's pulley, where the cable runs onto it (the swivel point); 4 or 8 entries, one per winch and motor | the corners of 2 x 1.6 m / of 2 x 1.6 x 1.8 m |
| `frame.winch_directions` | – | The direction from each pulley to its winch (its swivel axis); the pulley faces the frame's centre at rest | up for the top pulleys, down for the bottom ones |
| `pulley.radius_m` | m | The pulleys' radius | 0.025 / 0.03 |
| `cable.ea_n` | N | The cable's EA: a cable of free length l is a spring of EA / l | 1e5 / 3e5 |
| `cable.specific_damping_n_s` | N s | Its damping times its free length: c = this / l (`effects.cable_damping`) | 60 / 150 |
| `cable.breaking_load_n` | N | Its breaking load | 4000 / 10000 |
| `cable.safety_factor` | – | The band's top is the breaking load over this (the rating of `cable-max-tension`) | 10 |
| `cable.min_tension_n` | N | The band's bottom, the least tension that keeps a cable taut (`cable-min-tension`) | 10 / 20 |
| `cable.reference_tension_n` | N | The pretension the distribution keeps the tensions nearest to | 120 / 250 |
| `winch.drum_radius_m` | m | The drum's radius: the lever behind the gearbox | 0.025 / 0.03 |
| `winch.drum_inertia_kgm2` | kg m² | The drum's inertia | 2e-4 / 4e-4 |
| `winch.capacity_m` | m | The cable a drum holds: its stops at 0 and here | 5 |
| `winch.friction_coulomb_n`, `friction_viscous_n_s_m`, `friction_smoothing_m_s` | N, N s/m, m/s | Bearings and pulley at the cable (`effects.winch_friction`) | 2, 1 / 3, 2; 0.002 |
| `winch.stop_stiffness_n_m`, `stop_damping_n_s_m` | N/m, N s/m | The stops (`effects.hard_stops`) | 1e6, 1000 |
| `workspace.tool_min`, `tool_max` | m, rad | The tool's box, the axes' ranges; poses in it outside the wrench-feasible workspace are refused (`error-cable-wrench-infeasible`) | −0.6, 0.3 to 0.6, 1.3 / −0.4, −0.3, 0.8, −0.05 (each turn) to 0.4, 0.3, 1.4, 0.05 |

## `[[cycle.segment]]`

The cycle is a list of segments, run in order and repeated. All three configs have one. It cannot be edited in the parameter panel; `--set "cycle.segment = [{...}, ...]"` replaces it whole (TOML inline tables), and so does "Apply" of the app's optimiser where it changed the cycle's lifts.

| Key | Unit | Meaning |
| --- | --- | --- |
| `target` | m | Tool pose to move to, one position per axis, inside the strokes |
| `dwell_s` | s | Time to stay there. Rounded to whole profile cycles. With input shaping the dwell starts when the shaped setpoint has arrived |
| `payload` | true/false | State of the gripper once the segment has arrived (with input shaping: once the shaped setpoint has): `true` picks up or keeps the payload, `false` releases it |
| `shape` | – | Optional, in no shipped file. The shape of this segment's move, as `profile.shape`, which it replaces for this segment |
| `via` | m | Optional, in no shipped file. Via points, each one position per axis inside the strokes: the segment then runs a path planned within the axis limits (`path::plan`, with the transition curves of `profile.path_tolerance_m` and the rounded corners of `profile.corner_tolerance_m`; with `profile.cycle_timing = "time_optimal"` as fast as the motors allow) instead of the straight move: a line to the first via point, a smooth curve (spline) through the others, and a line from the last to `target`. With via points straight above pick and place and one between, a lift, traverse and lower without a stop; the optimiser writes such segments. One via point is two lines with a stop between. The path is planned from the segment before's target when the config is read, and refused (`cycle.segment[i].via`) where it cannot be planned or leaves the strokes. A segment with via points does not blend (`corner_blending`) with the one before. The first segment of a run starts from the start pose; were its path refused from there, it runs the straight move |

T and E: lower, dwell (grip), lift, traverse, lower, dwell (release), lift, traverse back; six segments. S: a shuttle between x = 0.9 and x = 0.1 with a dwell of 0.1 s at each end; two segments.

## `[conveyor]`

Picking from a moving belt conveyor (goal 3), for every machine whose tool axes are Cartesian positions: the deltas D3, D2 (x and z only: a belt along x), D4 and the linear delta, and the gantries. Only in C and the conveyor examples beside the other machines' defaults (T-gantry: a belt at 0.1 m/s, a part every 5.0 s, its pass about 3.6 s; D2, D4, linear delta: 0.3 m/s, a part every 1.0 s, each picking all 60 a minute); no default config has the table, and a config without it runs its `[[cycle.segment]]` as before. With it (and `enabled = true`) the cycle (`--cycle`, `analyze`, `verdicts`, the app's cycle) is one pass per part instead of the segments, which stay in the file unused: from rest above the place to above the part, meeting it with the belt's speed; down onto it, the grip moving with it, up; to above the place, down, the release (moving with a second belt, if the place is on one), up; and, from a moving place, to rest above it. How each move is planned, and what is a pick or a miss, is in the [model reference](model-reference.md#picking-from-a-conveyor). All keys are choices of the process, not properties of the machine: no `[bounds]`. Poses and directions are tool coordinates, one value per axis (0 on the D4's rotation).

| Key | Unit | Meaning | C |
| --- | --- | --- | --- |
| `enabled` | true/false | The cycle picks from the belt; `false` runs `[[cycle.segment]]` | true |
| `direction` | – | Along the belt, one value per axis; scaled to length 1, not all 0 | [1, 0, 0] |
| `speed_m_s` | m/s | The belt's speed, not negative (0: parts stand where they come) | 0.3 |
| `ramp_s` | s | The belt reaches its speed this long after the start, at a constant acceleration (0: at once). Parts are picked during the ramp too, met where they are with their speed then | 0 |
| `origin` | m | The grip pose of a part where it enters the belt, belt position 0 | [−0.5, 0.25, −0.8] |
| `length_m` | m | The belt's length from the origin, above 0; the 3D view draws it and the parts on it | 1.0 |
| `pitch_s` | s | Parts pass the origin at n `pitch_s` + each of `arrivals_s`, n = 0, 1, …; above 0 | 1.0 |
| `arrivals_s` | s | The times within a pitch, at least one, each from 0 to below `pitch_s` | [0.0] |
| `window_m` | m | The stretch along the belt from the origin in which a grip may run, [start, end], 0 ≤ start ≤ end ≤ `length_m`. A part the tool would meet before the start is waited for; one whose grip would end beyond the end is missed | [0.3, 0.8] |
| `lift` | m | Added to a grip or place pose for the pose above it, which the tool comes from and lifts to | [0, 0, 0.05] |
| `grip_s`, `release_s` | s | How long the tool moves with the part while the gripper closes, and with the place while it opens; the payload is attached at the end of the grip and let go at the end of the release | 0.035, 0.035 |
| `place` | m | The place pose | [0, −0.3, −0.8] |
| `place_direction`, `place_speed_m_s` | –, m/s | A second belt the place lies on, as `direction` and `speed_m_s`; speed 0: a fixed place | [1, 0, 0], 0 |
| `tracking_tolerance_m` | m | The verdict `pick-tracking`: the most the tool may be from the part along the belt during a grip, above 0 | 0.001 |

The results: `analyze` says the picks a minute (60 / the measured pass's time, one part per pass), the tracking error and the parts missed a minute on its error stream; `verdicts` judges `pick-tracking` against `tracking_tolerance_m` and `parts-missed` (parts a minute that pass the window unpicked) against 0.
