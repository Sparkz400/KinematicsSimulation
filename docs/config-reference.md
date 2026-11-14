# Config reference

One TOML file per machine. Units are in the key names. The key list below was generated from the three shipped configs, so every key they have is here:

| Short | File | Machine |
| --- | --- | --- |
| T | [`crates/machine-t-gantry/default.toml`](../crates/machine-t-gantry/default.toml) | T-gantry, made-up mid-size machine. Placeholder values |
| E | [`crates/machine-t-gantry/exct-30.toml`](../crates/machine-t-gantry/exct-30.toml) | T-gantry: Festo EXCT-30 with Siemens 1FK2205-4AF. Each value marked `[catalogue]` or `[estimate]` in the file |
| S | [`crates/machine-single-axis/default.toml`](../crates/machine-single-axis/default.toml) | Single belt axis. Placeholder values |
| D | [`crates/machine-delta-d3/default.toml`](../crates/machine-delta-d3/default.toml) | Delta picker D3, sized like the Autonox RL3-1200-3kg with Siemens 1FK2105-4AF. Each value marked `[catalogue]` or `[estimate]` in the file. Its keys are in [their own section](#delta-picker-d3) |
| D2 | [`crates/machine-delta-d2/default.toml`](../crates/machine-delta-d2/default.toml) | Planar delta picker D2, sized like the Codian D2-800-S020 with Siemens 1FK2105-4AF. Each value marked `[catalogue]` or `[estimate]` in the file. Its keys are in [their own section](#delta-picker-d2) |
| C | [`crates/machine-delta-d3/conveyor.toml`](../crates/machine-delta-d3/conveyor.toml) | The D3's default with a [`[conveyor]`](#conveyor) table: it picks from a belt conveyor instead of running the Adept cycle |
| D4 | [`crates/machine-delta-d4/default.toml`](../crates/machine-delta-d4/default.toml) | Four-axis delta picker D4: the D3 with a turning tool on a telescopic shaft, sized after the ABB IRB 360 with its fourth axis. Each value marked `[catalogue]` or `[estimate]` in the file. Its keys are in [their own section](#delta-picker-d4) |

| L | [`crates/machine-delta-linear/default.toml`](../crates/machine-delta-linear/default.toml) | Linear delta, sized like the Festo EXPT-95 with Siemens 1FK2205-2AF. Each value marked `[catalogue]` or `[estimate]` in the file. Its keys are in [their own section](#linear-delta) |

The **In** column says which configs have the key. "T E" means the key exists on the T-gantry only; "S" on the single axis only; "E" in the EXCT-30 config only. A key marked for one machine is not read by the other.

The **Source** column says where a value of this kind normally comes from:

- *datasheet*: published by the vendor of the part
- *estimate*: not published; a guess of the right order, to be bounded in `[bounds]` and replaced by a measurement
- *measurement*: can only be had from the machine
- *choice*: a setting, not a property of the machine

For the EXCT-30 config the file itself is the authority: its `[catalogue]` and `[estimate]` comments say for each value which it is.

## Rules

- `format_version = 1` is required. Any other value is refused.
- A config has to be complete. There are no built-in defaults for a missing key: a key that the machine reads and the file lacks is refused (`Configuration value is missing: ...`). Only `[ratings]` entries, `[parts]`, `[bounds]`, `obstacles`, `[conveyor]` and `drive.braking_module` (off) may be left out, and `effects.thermal_warm_start` and `effects.drive_current_limit` (off: a config written before 13 November 2026), and `effects.field_weakening`, `drive_overload` and `motor_thermal_nodes` (off: a config written before 14 November 2026), whose keys are read only with them on.
- A config file (`--config`) may not have a key that the machine's default config lacks: it is refused by name (`Configuration has a value this machine does not know: ...`), so a misspelt key cannot pass for the default's value. `[bounds]` has its own check, and `ratings.pulley_max_torque_nm` is the one rating no default config has, and `[conveyor]` the table none has. A name in `[parts]` has to be text.
- `--set path=value` and the app's parameter panel can only change values that exist, and cannot change their type. An integer is accepted where a float is expected.
- Most physical values must be strictly positive (masses, radii, stiffnesses, cycle times, friction levels). A zero there is refused as out of range.
- Values that may be zero must not be negative: the payload, dampings, viscous and shaft friction, backlash, belt pretension and belt mass, the gearbox's inertia, idler and cable chain values, feedforward factors, the blend time, the path tolerance, the start temperature rise.
- `gearbox.efficiency`, `profile.torque_reserve` and `profile.tension_reserve` are above 0 and at most 1; `motor.ripple_fraction` and `profile.shaper_damping` are from 0 to below 1; `drive.no_load_speed_rad_s` is above `drive.rated_speed_rad_s`.
- Free in sign: forces (`process.*`, `belt.tooth_force_n`, `motor.cogging_nm`), `geometry.incline_deg`, the two drops of the Z axis, the ripple and cogging orders.
- NaN and infinity are refused for every number. `start.pose` has one position per axis within its range, and a cycle segment's `dwell_s` is finite and not negative.
- Every refusal names the key (`Configuration value is out of range: gearbox.backlash_rad`). Most values are checked whether or not their effect is switched on.
- Loop cycle times must be whole multiples of `sim.dt_s`, and `drive.current_cycle_s` must equal it.
- A changed config can be applied to a running simulation (the app does), except `sim.dt_s`. That includes `profile.cycle_s` while a motion is under way: the controller counts its own profile cycles and converts what it has stored in them, so the setpoint goes on from where it is. Switching `effects.belt_stretch` while an axis moves carries the tool's position and speed over into the other model. So does a changed `gearbox.ratio` or `geometry.pulley_radius_m`: the state is rebuilt at the tool's pose and speed, with belts and gears as they are at rest there. A changed input shaper (switch, frequency, damping, following the pose) is taken when the plan next stands still. A cycle that loses segments while it runs goes on within those it has, and ends if none is left.

## Top level

| Key | In | Unit | Meaning |
| --- | --- | --- | --- |
| `format_version` | T E S | – | Format of the file. Must be 1 |
| `obstacles` | T E S D | – | Obstacles in the work area: a list of boxes with their edges along the axes of the 3D view (scene coordinates, m: x right, y into the depth, z up), each a table with `name` (text), `centre_m` and `size_m` (three numbers each, the sizes above 0; another key in an entry is refused). `[]` in all four files: no obstacles, and nothing changes. Write `obstacles = [{ name = "fixture", centre_m = [...], size_m = [...] }]`, or `[[obstacles]]` tables at the end of a file without the `obstacles = []` line. A config without the key has none. The tool is where the 3D view draws it: T and E at (`geometry.span_min_m` + x, −0.05, z − `stroke_z_m` − `span_min_m` − 0.042), S at (`span_min_m` + x, −0.03, 0.06), D at (x, y, z). What can hit an obstacle, the clearance kept (`profile.min_clearance_m`) and the crash are in the [model reference](model-reference.md#obstacles-and-crash) |

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
| `motor_supply` | T E S | `"230v"` or `"400v"`: the S210 data at that supply; `"s120"`: the motor on a SINAMICS S120 at 400 V (DC link 540 V). `"400v"` in all configs |
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
| `torque_ripple` | T E S | on | Torque ripple (proportional to torque) and cogging (independent of it) |
| `tooth_meshing` | T E S | on | A small periodic force at the tooth-pass frequency |
| `frame_compliance` | T E | on | One spring-mass mode of the frame in X |
| `tool_flex` | T E | on | The Z axis leans against the carriage; the reported X position is the tool tip. In a machine file the body its `machine.lean` node names leans (elastic model), off in `machines/t-gantry.toml` as compared with its crate |
| `idlers` | T E | off | Inertia and bearing friction of the five idlers |
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
| `encoder_resolution` | T E S | on | The position is measured in whole encoder counts; speed is the difference of positions |
| `velocity_filter` | T E S | on | First-order low-pass on the measured speed |
| `torque_filter` | T E S | on | First-order low-pass on the torque setpoint |
| `notch_filter` | T E S | off | A notch on the torque setpoint |
| `torque_limit` | T E S | on | The torque setpoint is limited to the peak torque and the torque-speed curve |
| `drive_current_limit` | T E S D | on | The drive gives no more than its maximum output current `ratings.drive_max_current_a`: the torque setpoint is held to the torque of that current where it is below the motor's limit, with the velocity loop's anti-windup, so a cycle that asks more lags behind (see the [model reference](model-reference.md#drive-current-limit)). Nothing is held without the rating. On in the shipped configs (the joint machines' `common.toml` and the machine files too); it changes nothing there, as every shipped drive gives more than its motor takes. Off, the drive's maximum current is only judged (`drive-peak-current`) |
| `field_weakening` | T E S D | off | The motor as a permanent-magnet synchronous motor in d-q axes (`motor.pole_pairs`, `d_inductance_h`): above base speed the drive sets a negative d current within the voltage and current circles, and the torque-speed curve follows from the model instead of fading between rated and no-load speed (see the [model reference](model-reference.md#field-weakening)). Off in the shipped configs |
| `drive_overload` | T E S D | off | The drive's thermal image (i²t, `ratings.drive_overload_time_constant_s`): once it reaches the level at which the drive has given its maximum current for `drive_max_current_time_s` from the rated load, the drive's current limit falls to its base load current `drive_base_load_current_a` until the image is back at the rated current's; judged by `drive-overload` (see the [model reference](model-reference.md#drive-overload)). Off in the shipped configs |
| `motor_thermal_nodes` | T E S D | off | With `winding_heating`: the motor's heat in three nodes, winding, stator and housing, surroundings (`motor.winding_resistance_share`, `winding_time_constant_s`); the settled rise `motor-thermal` judges is the winding's and the same as the one node's (see the [model reference](model-reference.md#motor-thermal-nodes)). Off in the shipped configs |
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
| `shared_dc_link` | T E S | off | With `dc_link`: the drives of the machine share one DC link (one Line Module, as on a SINAMICS S120). What a braking motor feeds back a motoring one draws. The link has the capacitance of all drives together (each brings `ratings.dc_link_capacitance_uf`), one chopper with one braking resistor of the `[ratings]` powers, and one trip; every motor reports the same voltage and resistor power. Switched on while the machine runs, links that differ put their energies together. Changes nothing on a machine with one drive |
| `regenerative_supply` | T E S | off | With `dc_link`: a regenerative supply (an Active Line Module, as on a SINAMICS S120). Above the bus level it feeds the DC link's energy back into the mains, up to `ratings.line_module_max_power_w`, of which `ratings.line_module_efficiency` arrives there; the braking resistor takes only what is beyond that power. Reported as energy fed back into the mains and as line module loss (analysis, budget, `analyze` column `mains_energy_j`, the app), and judged by `line-module-continuous` and `line-module-peak`. Off by default, so that earlier results stay as they were |
| `fieldbus_delay` | T E S | off | The motor setpoints reach the drives some profile cycles late |
| `fieldbus_jitter` | T E S | off | Each frame of motor setpoints takes from 0 to `profile.fieldbus_jitter_cycles` more cycles than the delay, evenly and pseudo-random from `drive.noise_seed` (every run the same); the drives use the newest frame that has arrived, so a late frame leaves them the one before. Works with or without `fieldbus_delay`. Off by default, so that earlier results stay as they were |
| `sensor_noise` | T E S | off | Gaussian noise on the measured position and current, repeatable from a seed |
| `crash` | T E S D | off | The obstacles of `obstacles` are rigid: a part moving into one stops dead at its face, with the drives on or off, and drives that were on go off; enabled again, they can take the machine out (see the [model reference](model-reference.md#obstacles-and-crash)). Contacts are found and reported whether it is on or not; off, the machine moves through the obstacle. Changes nothing without obstacles. Off by default, so that earlier results stay as they were |

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

In a machine file, `[motor.<id>]`, `[gearbox.<id>]`, `[drive.<id>]`, `[ratings.<id>]` and `[parts.<id>]` give the motor
`<id>` its own values over the shared tables, key by key; each key has to be one the shared table has, with its meaning,
unit and mark there. `[parts.<id>]` takes `motor`, `motor_supply`, `gearbox` and `drive` (not `belt` or `axis`, which
are the machine's). See [machine files](machine-files.md#motors-that-differ). The portal of `machines/xyz-portal.toml`
has them for its Z motor; no other config has any.

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
| `gearbox_viscous_loss_share` | T E S | – | Share of a gearbox's loss (1 − η) that is churning and bearing drag and follows the viscosity; the rest, tooth friction, stays. 0.5. No maker publishes it. In E only for a gearbox put in | estimate | 0.25 to 0.75 |
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
| `current_kp_v_a` | T E S | V/A | Current controller gain. Kp = L / T gives the current loop the time constant T | choice | always |
| `current_tn_s` | T E S | s | Current controller integral time. Tn = L / R | choice | always |
| `bus_voltage_v` | T E S | V | The DC link voltage: the level the rectifier holds the DC link at. The winding gets sqrt(3/2) times it at most, because the model's one winding carries the RMS phase current and its voltage times that current is the power of all three phases. T S: 155.13 V, which is 190 V at the winding. E: 540 V, 661 V at the winding. A motor by name sets 540 V or 325 V | datasheet | `voltage_limit`, `dc_link` |
| `peak_torque_nm` | T E S | N m | Highest torque of motor and drive. E: the 12.5 N m the EXCT-30 allows at the pulley; the motor could give 18 N m | datasheet | `torque_limit`; analysis always |
| `rated_torque_nm` | T E S | N m | What the motor can give continuously. The analysis reports the RMS torque over it (`thermal_utilisation`); the winding's heating rests on `motor.rated_current_a` | datasheet | analysis |
| `rated_speed_rad_s` | T E S | rad/s | Full torque up to this speed. 314 rad/s is 3000 r/min | datasheet | `torque_limit`; analysis always |
| `no_load_speed_rad_s` | T E S | rad/s | The torque-speed curve falls linearly from the rated speed to zero here. E: 518 rad/s | datasheet, or sqrt(3/2) × DC link voltage over torque constant | `torque_limit`; analysis always |
| `encoder_counts_per_rev` | T E S | counts | Encoder resolution. 1048576 is 20 bit, 4194304 is 22 bit | datasheet | `encoder_resolution`, `sensor_noise` |
| `velocity_filter_s` | T E S | s | Time constant of the speed filter | choice | `velocity_filter` |
| `torque_filter_s` | T E S | s | Time constant of the torque setpoint filter | choice | `torque_filter` |
| `notch_frequency_hz` | T E S | Hz | Centre of the notch | choice | `notch_filter` |
| `notch_bandwidth_hz` | T E S | Hz | Width of the notch | choice | `notch_filter` |
| `encoder_noise_counts` | T E S | counts | Standard deviation of the noise on the measured position | estimate | `sensor_noise` |
| `current_noise_a` | T E S | A | Standard deviation of the noise on the measured current | estimate | `sensor_noise` |
| `noise_seed` | T E S | – | Seed of the noise and of the fieldbus jitter. A whole number, not negative. Each motor has a stream of its own, and the jitter one more | choice | `sensor_noise`, `fieldbus_jitter` |
| `braking_module` | T E S D | on/off | The axes brake through one braking resistor for all, as an S120 line-up through its Braking Module: `verdicts` judges the sum of their braking power against `ratings.braking_resistor_continuous_w` (the mean of the sum) and `_peak_w` (the peak of the summed power, step by step), once for the machine. Off in all four files: each axis has a resistor of its own, as an S210, and is judged alone. A drive by name sets it: on for an S120 Motor Module, off for an S210. Optional in a config file (off) | choice | `verdicts` |

## `[ratings]`

What the parts allow. `sim-cli verdicts` judges a cycle against these. **A rating that is left out is not judged.** With `effects.dc_link` the capacitance, the braking threshold, the resistor's peak power, the trip level and the discharge time constant are required, because the simulated DC link is built from them (with `effects.regenerative_supply` also the line module's maximum power and efficiency), and the threshold has to lie between `drive.bus_voltage_v` and `dc_link_overvoltage_v`. `verdicts` refuses a threshold at or below `drive.bus_voltage_v` with the effect off as well: it would leave the capacitors a negative room.

| Key | In | Unit | Meaning | Judged by | Source in the shipped configs |
| --- | --- | --- | --- | --- | --- |
| `belt_max_tension_n` | T E S | N | Allowable belt tension. E: 2140 N. Festo publishes no allowable belt tension for the EXCT. It states the 5 mm pitch, the steel cords, the 30 mm width and the stretch of the belt: 0.045 % at 300 N, which is 667 kN per unit strain. Megadyne's Megalinear handbook lists for 5 mm pitch steel-cord polyurethane belts, at 4 mm/m stretch: T5 840 N at 25 mm and 1060 N at 32 mm (about 250 kN per unit strain at 30 mm), AT5 1805 N at 25 mm and 2275 N at 32 mm (about 535 kN). Festo's belt is somewhat stiffer than AT5 with standard steel cords and far stiffer than T5, so the limit is the AT5 figure between the two widths: 2140 N at 30 mm. The figures were read from the handbook itself (average values for open-ended belts, a quarter of the breaking strength); that Festo's belt has cords of that class is concluded from the stretch, not stated by Festo. The teeth give way before the cords do: that is judged by `pulley_max_torque_nm`. | `belt-tension` | placeholder (T S), estimate (E) |
| `belt_min_tension_n` | T E S | N | Lowest tension allowed. Below it the teeth can jump | `belt-slack` | choice (0) |
| `settling_tolerance_m` | T E S | m | How far the tool may be from its target in the second half of a dwell | `settling` | choice |
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
| `belt_span_resonance_band` | T E S | share | With `effects.belt_span_vibration`: how close a span's first sideways mode may come to an excitation of the drive at steady speed, as a share of the mode. 0.1: belt makers ask to keep the two apart without a figure; 10 % is the separation usually kept from a resonance | `belt-span-resonance` (passes at or above it) | estimate, a limit, not bounded |
| `dc_link_capacitance_uf` | T E S | µF | DC link capacitance of one drive | `dc-link-energy` | placeholder (T S); estimate from S120 data (E) |
| `braking_threshold_v` | T E S | V | DC link voltage at which the chopper switches the braking resistor in. T S: 224.65 V. A motor by name scales it with the DC link voltage | `dc-link-energy` | placeholder (T S); estimate (E) |
| `braking_resistor_continuous_w` | T E S | W | Continuous power of the braking resistor. A drive by name sets it (S210) | `braking-resistor-continuous` | placeholder (T S); catalogue (E, 100 W, its converter's internal resistor) |
| `braking_resistor_peak_w` | T E S | W | Peak power of the braking resistor. The simulated resistor never takes more. A drive by name sets it (S210) | `braking-resistor-peak` | placeholder (T S); catalogue (E, 5.7 kW) |
| `dc_link_overvoltage_v` | T E S | V | DC link voltage at which the drive trips. T S: 257.6 V. A motor by name scales it with the DC link voltage; an S210 converter by name sets its trip level (410 V on 1 AC, 820 V on 3 AC) | `dc-link-overvoltage` (only with `effects.dc_link`) | placeholder (T S); catalogue (E, 820 V) |
| `dc_link_discharge_time_constant_s` | T E S | s | With `effects.dc_link`: time constant R C with which a disabled drive's DC link discharges through its bleeder and the drive's own consumption, down to the bus level the rectifier holds. 130 s in all three: Siemens asks to wait 5 minutes after switching off for the DC link to discharge, taken as 600 V falling to 60 V, 300 s / ln 10. Required with `effects.dc_link`. No cycle uses it, so its bounds get no runs | none | estimate, bounds 30 to 300 s |
| `line_module_rated_power_w` | T E S | W | With `effects.regenerative_supply`: rated power of the line module. 16 kW, the smallest S120 Active Line Module 6SL3130-7TE21-6AA4 in [`line_modules_s120.csv`](../catalog/siemens/line_modules_s120.csv) | `line-module-continuous`: the mean power through it, drawn and fed back | catalogue |
| `line_module_max_power_w` | T E S | W | With `effects.regenerative_supply`: the most it feeds back; the simulated line module never feeds back more. 35 kW, the same module's maximum power. Required with the effect | `line-module-peak`: the highest power through it either way | catalogue |
| `line_module_efficiency` | T E S | fraction | With `effects.regenerative_supply`: the share of what it feeds back that arrives in the mains; the rest is its loss. Above 0, at most 1. 0.98 from the catalogue's 0.28 kW loss at 16 kW; at the small powers of these machines a guess. Required with the effect; its bounds get runs only with it | none | estimate, bounds 0.95 to 0.99 |

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
| `torque_reserve` | T E S | fraction | Share of the torque-speed curve a time-optimal path may use, and of the torque of the drive's maximum current (with `effects.drive_current_limit`) and of the gearbox's and pulley's torque rating at the output (`ratings.gearbox_max_torque_nm`, `pulley_max_torque_nm`). Above 0, at most 1 | choice | `--time-optimal` |
| `tension_reserve` | T E S | fraction | Share of the room between the belt tension that holds the tool and `ratings.belt_max_tension_n` / `belt_min_tension_n` (each less `tension_margin_n`) that a time-optimal path may use in the plan's rigid model. 0.5 in all three files: a change of acceleration quick against the belt's period makes a lightly damped belt overshoot by up to the change itself, so the full model swings up to twice the rigid model's dynamic tension (0.8 let the default T-gantry pass a 330 N rating by 1.4 N at 100 m/s³ and go slack at 1000 m/s³). Above 0, at most 1. Without a tension rating the belt is no limit of the plan | choice (E: estimate) | `--time-optimal` |
| `tension_margin_n` | T E S D | N | Taken off both belt tension ratings before `tension_reserve`, for the guide friction the belt pulls, which the loss-free rigid model leaves out: the X guide's coulomb friction plus its viscous friction at the axis' top speed (T 15 + 20 · 0.8 = 31 N, E 20 + 20 · 2.5 = 70 N, S 15 + 20 · 1.0 = 35 N; D 0, no belt). At least 0. A path whose holding tension alone is beyond a rating less the margin is refused | estimate | `--time-optimal` |
| `min_clearance_m` | T E S D | m | The least clearance the machine keeps from every obstacle of `obstacles` (surface to surface, between the obstacle and the parts that can hit one): the check `collision` judges the run's least clearance against it, and a time-optimal path that comes nearer is refused (`error-path-collision`). Also, for `sim-cli optimise` and the app's optimiser: how far the tool has to stay beyond the farther of pick and place (along the last axis, the side the lift goes to) while it traverses between them. The optimiser lowers each lift, traverse and lower of the cycle to this, and rounds its corners through a via point only where the planned path keeps it (less the path tolerance). Not negative. T 0.3 and E 0.17: the lifts as configured (no obstacle is known); S 0 (no lift); D 0.025, its 25 mm lift | choice, or the height of an obstacle | `optimise` |
| `path_tolerance_m` | T E S | m | How far a planned path may leave the lines, arcs and splines of its file, so that transition curves can take it from a line into an arc at speed, and a spline's end can be eased to the curvature of the line or arc it meets. The deviation is L² / (24 r) for curves of length L at an arc of radius r, 0.01728 dk l² for a spline eased over l against a step dk of the curvature, and is reported. Not negative; 0 (in all three files) plans the path as given | choice | `--path`, and the app's Path |

## `[bounds]`

Low and high bound of the values that are estimates. The table mirrors the config: where the config has `[friction_x] coulomb_n = 15.0`, `[bounds]` has `friction_x.coulomb_n = [10.0, 20.0]`, in the unit of the value. Used by `sim-cli verdicts --bands`, by `sim-cli sensitivity` and by the app's sensitivity ranking.

- A bound on a key the config does not have is refused.
- A bound has to be two numbers with low ≤ high.
- A test holds every bound around its value (`the_bounds_of_every_config_bracket_their_values`).
- Arrays are not shown in the parameter panel, so bounds cannot be edited in the app.

| Config | Bounded values | What is bounded | What is not |
| --- | --- | --- | --- |
| T | 37 | Belt stiffness, pretension, damping, tooth force, tooth jump factor; guide Coulomb, stiction and viscous friction of both axes; motor shaft friction, ripple, cogging; gearbox efficiency, stiffness, damping; frame stiffness and damping; the three `tool_flex` values; the thermal resistances and time constants of `[thermal]`; the four estimates of `[thermal_feedback]`; the DC link's discharge time constant and the line module's efficiency; the belt's teeth in mesh; the bearing lubricant's viscosity and contamination factor; the gearing's Wöhler exponent | Masses, inertias, backlash; values of effects that are off by default, but for `[thermal]`, `[thermal_feedback]` and the tooth jump factor; the shape of the friction curve (Stribeck speed, bristles, preload); the rated rises of `[thermal]`; the reference temperature; the span resonance band (a limit) |
| S | 29 | As T, without `friction_z`, `frame` and `tool_flex` | As T |
| D | 32 | The geometry and the masses (not the payload), the rods' spin inertia, the joint's friction, the motor's friction, ripple, cogging and winding resistance, the gearbox's stiffness and damping, `[thermal]`, three of `[thermal_feedback]`, the DC link's discharge and the line module's efficiency, the gearing's Wöhler exponent, the upper arms' bending stiffnesses and dampings (in and out of their planes) | The stop angles and stiffness, the working range, the axis limits, the loop gains, every `[catalogue]` value |
| D2 | 26 | As D, with the gearbox's backlash in place of its stiffness (the stiffness is the catalogue's, the backlash an estimate) | As D |
| D4 | 50 | As D, with the arms' backlash, and the estimates of `[tool_gearbox]` (backlash, damping), `[tool]` and `[shaft]` | As D; the shaft's slide smoothing speed |
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

## Linear delta

The linear delta's config ([`machine-delta-linear/default.toml`](../crates/machine-delta-linear/default.toml)) has the tables every machine has as T and S have them: `[sim]`, `[parts]`, `[motor]`, `[gearbox]` (here a coupling of ratio 1), `[pulley]`, `[hard_stop]`, `[temperature]`, `[thermal_feedback]`, `[drive]`, `[ratings]` (without guide, pulley-bearing, gearbox and tooth-load ratings), `[profile]`, `[bounds]`, `[start]`, `[[cycle.segment]]`, the belt's `[belt]` keys the [belt sections](model-reference.md#belt-sections) read (no tooth meshing or tooth jump), and the effects of the drives and the controller. It has no frame, idler, cable chain, process force or `[thermal]`: no part but the motors warms, so `effects.part_heating` and `thermal_feedback` change nothing. Its own keys:

`[effects]`: `belt_stretch` (off: the rigid reference model), `belt_damping`, `belt_slack`, `belt_mass`, `belt_tooth_stiffness`, `belt_material_damping`, `guide_friction`, `load_dependent_friction` (the guides' levels follow their load across the rail, the rods' pull at rest), `motor_friction`, `gearbox_backlash`, `gearbox_efficiency`, `torque_ripple`, `temperature`, `hard_stops` (the carriages' stroke ends), `holding_brake`, as on the single axis; the shipped values as there, `belt_tooth_stiffness`, `belt_material_damping` and `temperature` off.

`[geometry]` (estimates, bounded but the stroke and the stop)

| Key | Unit | Meaning | Value |
| --- | --- | --- | --- |
| `column_radius_m` | m | Centre of the frame to a carriage's joint line, R | 0.52 |
| `plate_radius_m` | m | Centre of the plate to a rod pair's centre, r. The equations use e = R − r | 0.07 |
| `rod_m` | m | Rod length, carriage joint to plate joint (the parallelogram's centre line), L | 1.1 |
| `rod_spacing_m` | m | Distance of the two rods of a pair; for the 3D view only | 0.08 |
| `carriage_top_m` | m | Height of the carriages' joints at their upper stop | 0.0 |
| `stroke_m` | m | The carriages' stroke below it | 0.7 |
| `pulley_radius_m` | m | Drive pulley radius (EGC-80-TB: 90 mm per revolution) | 0.014324 |
| `span_min_m` | m | Shortest free belt span between a carriage and either pulley | 0.1 |

`[workspace]`

| Key | Unit | Meaning | Value, source |
| --- | --- | --- | --- |
| `radius_m` | m | Radius of the working cylinder; also the x and y axis ranges ±radius | 0.475, catalogue (Festo EXPT-95: Ø950) |
| `top_z_m`, `bottom_z_m` | m | Its top and bottom, the z axis range | −1.12, −1.22: 100 mm, catalogue (EXPT nominal height); where it hangs an estimate |
| `max_rod_angle_deg` | ° | A target where a rod stands steeper from the vertical is refused (`error-delta-linear-rod-angle`) | 60, estimate (the range needs up to 57.2) |
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

L's values of the shared tables: motor 1FK2205-2AF at 400 V with brake (catalogue: rotor 4.0 kg cm², 1.241379 N m/A, 10.8 N m peak, 3.0 N m rated at 3000 r/min, 9.5 A and 2.5 A; R, L, thermal time constant and brake torque estimates; the High Dynamic 1FK2104-5AF, 0.65 kg cm², left the velocity loop ringing near 450 Hz against about 6 kg cm² of load), a coupling of ratio 1 (3000 N m/rad, estimate as the EXCT-30's), converter 6SL3210-5HE11-0UF0 (3 A, 10.9 A; its internal braking resistor 50 W, 3 kW, 820 V), belt EA 6e5 N and 300 N pretension (estimates as the EXCT-30's), `settling_tolerance_m` 0.5 mm (catalogue: EXPT positioning accuracy), `belt_max_tension_n` 1000, `pulley_max_torque_nm` 10 and `shaft_max_radial_load_n` 1500 (estimates for the EGC-80), `ratings.max_condition_number` 5, the velocity loop 0.25 N m s/rad, 0.02 s with 0.2 ms filters (tuned with `sim-cli margins delta-linear --bands`). Its cycle is the D3's Adept cycle 25/305/25 mm at the top of the range; `start.pose` above the place position.

## Delta picker D4

The D4's config ([`machine-delta-d4/default.toml`](../crates/machine-delta-d4/default.toml)) has the D3's tables and keys with the D3's values, but for the arms' `gearbox.backlash_rad` (8.727e-4, 3 arcmin, estimate, bounded 2 to 4 arcmin: a reduced-backlash precision planetary; the NRB080-05-20's 9 arcmin fail `settling`, 1.21 mm with the tool's 10). The tool pose, `start.pose` and the cycle's targets are (x, y, z, c), c in rad. These keys are its own:

| Key | Unit | Meaning | Value, source |
| --- | --- | --- | --- |
| `effects.shaft_mass` | – | The shaft's weight share at the platform (on the arms) and its spin inertia at the tool | on |
| `effects.shaft_torsion` | – | The shaft twists in series with the tool's gearbox (with `joint_elasticity`) | on |
| `effects.telescope_friction` | – | The telescope's sliding friction along the shaft, a force on the platform that the arms take | on |
| `axis.c.min_angle_deg`, `max_angle_deg` | ° | The controller's range of the tool's turn (the machine has no stops) | −360, 360, choice |
| `axis.c.max_velocity_rad_s`, `max_acceleration_rad_s2`, `max_jerk_rad_s3` | rad/s, rad/s², rad/s³ | The turn's limits | 30, 600, 30000, estimates (ABB publishes none; 30 rad/s is the motor's 240 rad/s behind ratio 8) |
| `tool_gearbox.ratio`, `efficiency`, `stiffness_nm_rad`, `inertia_at_input_kgm2` | | The tool's gearbox, as `[gearbox]` | 8, 0.95, 8594.4, 3.47e-5, catalogue (Siemens NRB060-05-8) |
| `tool_gearbox.backlash_rad` | rad | Its backlash | 2.909e-4 (1 arcmin), estimate, bounded 0.5 to 2 arcmin (catalogue: 10 arcmin) |
| `tool_gearbox.damping_nm_s_rad` | N m s/rad | Its damping (replaced by the shaft's with `shaft_torsion`) | 5, estimate, bounded 2 to 10 |
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
| `ratings.tool_gearbox_max_torque_nm` | N m | The highest torque the tool's gearbox may pass (`tool-gearbox-peak`) | 29, catalogue (NRB060-05-8) |

The fourth motor is the arms' 1FK2105-4AF with the same drive settings (the drive reads one `[motor]` and `[drive]` per machine); the common gearbox checks of motor 4 are judged against the arms' `[ratings]`, the tool's gearbox by `tool-gearbox-peak`.

## Arms in tool space (SCARA, six-axis arm)

The configs of `scara` ([`config/scara.toml`](../crates/machine-joint-kit/config/scara.toml)) and `urdf-arm`
([`config/arm.toml`](../crates/machine-joint-kit/config/arm.toml)) have these keys; shipped with tool space off, so the
axes are the joints as before. [`config/scara-tool.toml`](../crates/machine-joint-kit/config/scara-tool.toml) and
[`config/arm-tool.toml`](../crates/machine-joint-kit/config/arm-tool.toml) are the same configs with it on, a start pose
and a cycle of straight lines in tool space (and the arm's settling tolerance 5 mm instead of 3 mrad). See
[mechanisms](mechanisms.md#tool-space).

| Key | Unit | Meaning | Value, source |
| --- | --- | --- | --- |
| `kinematics.tool_space` | – | On: the axes are the flange's pose (SCARA x, y, z, c; six joints x, y, z, rx, ry, rz), `start.pose` and the cycle's targets too; off: the joints | off, choice |
| `kinematics.elbow` | – | SCARA only: "right" (joint 2 counter-clockwise from the stretched arm) or "left" | "right", choice |
| `kinematics.home_joints` | rad | Six joints only: the joint pose whose branch the inverse keeps; it follows the line from its flange pose to each target | 0, 0.4, 0.2, 0, 0.97, 0 (flange 0.99 m out, 0.83 m up, pointing down), choice |
| `workspace.tool_min`, `tool_max` | m, rad | In tool space: each axis's range, x, y, z in m, the turn(s) in rad; poses in it out of reach or beyond a joint's stop are refused | SCARA 0.15 to 0.6, −0.45 to 0.45, 0 to 0.2, −3 to 3; arm 0.5 to 1.2, −0.6 to 0.6, 0.3 to 1.1, −3.1 to 3.1 each turn; choice |
| `workspace.max_condition_number` | – | In tool space: a target whose Jacobian has a higher condition number is refused (`error-arm-singularity`); SCARA: of d(x, y)/d(joints 1, 2); arm: the 6 x 6 geometric Jacobian, its turn rows times the flange's distance from joint 2 at the home | SCARA 25 (11° of elbow), arm 30 (5.0 at the home), estimates (design rules) |
| `axis.x`, `axis.y`, `axis.z`: `max_velocity_m_s`, `max_acceleration_m_s2`, `max_jerk_m_s3` | m/s, m/s², m/s³ | In tool space: the flange's limits along each axis | SCARA 1, 5, 200 (z 0.8, 8, 400); arm 1, 4, 80; estimates |
| `axis.c` (SCARA), `axis.rx`, `axis.ry`, `axis.rz` (arm): `max_velocity_rad_s`, `max_acceleration_rad_s2`, `max_jerk_rad_s3` | rad/s, rad/s², rad/s³ | In tool space: the turn's limits | SCARA 4, 30, 1000; arm 2, 8, 150; estimates |

These are choices and design rules, not properties of a part, so they have no `[bounds]`.

## `[[cycle.segment]]`

The cycle is a list of segments, run in order and repeated. All three configs have one. It cannot be edited in the parameter panel; `--set "cycle.segment = [{...}, ...]"` replaces it whole (TOML inline tables), and so does "Apply" of the app's optimiser where it changed the cycle's lifts.

| Key | Unit | Meaning |
| --- | --- | --- |
| `target` | m | Tool pose to move to, one position per axis, inside the strokes |
| `dwell_s` | s | Time to stay there. Rounded to whole profile cycles. With input shaping the dwell starts when the shaped setpoint has arrived |
| `payload` | true/false | State of the gripper once the segment has arrived (with input shaping: once the shaped setpoint has): `true` picks up or keeps the payload, `false` releases it |
| `via` | m | Optional, in no shipped file. Via points, each one position per axis inside the strokes: the segment then runs a path planned within the axis limits (`path::plan`, with the transition curves of `profile.path_tolerance_m`) instead of the straight move: a line to the first via point, a smooth curve (spline) through the others, and a line from the last to `target`. With via points straight above pick and place and one between, a lift, traverse and lower without a stop; the optimiser writes such segments. One via point is two lines with a stop between. The path is planned from the segment before's target when the config is read, and refused (`cycle.segment[i].via`) where it cannot be planned or leaves the strokes. A segment with via points does not blend (`corner_blending`) with the one before. The first segment of a run starts from the start pose; were its path refused from there, it runs the straight move |

T and E: lower, dwell (grip), lift, traverse, lower, dwell (release), lift, traverse back; six segments. S: a shuttle between x = 0.9 and x = 0.1 with a dwell of 0.1 s at each end; two segments.

## `[conveyor]`

Picking from a moving belt conveyor (goal 3), for every machine whose tool axes are Cartesian positions: the deltas D3, D2 (x and z only: a belt along x), D4 and the linear delta, and the gantries. Only in C; no default config has the table, and a config without it runs its `[[cycle.segment]]` as before. With it (and `enabled = true`) the cycle (`--cycle`, `analyze`, `verdicts`, the app's cycle) is one pass per part instead of the segments, which stay in the file unused: from rest above the place to above the part, meeting it with the belt's speed; down onto it, the grip moving with it, up; to above the place, down, the release (moving with a second belt, if the place is on one), up; and, from a moving place, to rest above it. How each move is planned, and what is a pick or a miss, is in the [model reference](model-reference.md#picking-from-a-conveyor). All keys are choices of the process, not properties of the machine: no `[bounds]`. Poses and directions are tool coordinates, one value per axis (0 on the D4's rotation).

| Key | Unit | Meaning | C |
| --- | --- | --- | --- |
| `enabled` | true/false | The cycle picks from the belt; `false` runs `[[cycle.segment]]` | true |
| `direction` | – | Along the belt, one value per axis; scaled to length 1, not all 0 | [1, 0, 0] |
| `speed_m_s` | m/s | The belt's speed, not negative (0: parts stand where they come) | 0.3 |
| `ramp_s` | s | The belt reaches its speed this long after the start, at a constant acceleration; no pass starts before (0: at once) | 0 |
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
