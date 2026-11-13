# Config reference

One TOML file per machine. Units are in the key names. The key list below was generated from the three shipped configs, so every key they have is here:

| Short | File | Machine |
| --- | --- | --- |
| T | [`crates/machine-t-gantry/default.toml`](../crates/machine-t-gantry/default.toml) | T-gantry, made-up mid-size machine. Placeholder values |
| E | [`crates/machine-t-gantry/exct-30.toml`](../crates/machine-t-gantry/exct-30.toml) | T-gantry: Festo EXCT-30 with Siemens 1FK2205-4AF. Each value marked `[catalogue]` or `[estimate]` in the file |
| S | [`crates/machine-single-axis/default.toml`](../crates/machine-single-axis/default.toml) | Single belt axis. Placeholder values |
| D | [`crates/machine-delta-d3/default.toml`](../crates/machine-delta-d3/default.toml) | Delta picker D3, sized like the Autonox RL3-1200-3kg with Siemens 1FK2105-4AF. Each value marked `[catalogue]` or `[estimate]` in the file. Its keys are in [their own section](#delta-picker-d3) |

The **In** column says which configs have the key. "T E" means the key exists on the T-gantry only; "S" on the single axis only; "E" in the EXCT-30 config only. A key marked for one machine is not read by the other.

The **Source** column says where a value of this kind normally comes from:

- *datasheet*: published by the vendor of the part
- *estimate*: not published; a guess of the right order, to be bounded in `[bounds]` and replaced by a measurement
- *measurement*: can only be had from the machine
- *choice*: a setting, not a property of the machine

For the EXCT-30 config the file itself is the authority: its `[catalogue]` and `[estimate]` comments say for each value which it is.

## Rules

- `format_version = 1` is required. Any other value is refused.
- A config has to be complete. There are no built-in defaults for a missing key: a key that the machine reads and the file lacks is refused (`Configuration value is missing: ...`). Only `[ratings]` entries, `[parts]` and `[bounds]` may be left out.
- A config file (`--config`) may not have a key that the machine's default config lacks: it is refused by name (`Configuration has a value this machine does not know: ...`), so a misspelt key cannot pass for the default's value. `[bounds]` has its own check, and `ratings.pulley_max_torque_nm` is the one rating no default config has. A name in `[parts]` has to be text.
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
| `axis` | T E S | An axis by name: `"EXCT-15"`, `"EXCT-30"`, `"EXCT-100"` ([`festo/exct.toml`](../catalog/festo/exct.toml)) or a Bahr axis as series-size, e.g. `"ELZI-40"`, `"ELZ-60S"` ([`bahr/belt_axes.csv`](../catalog/bahr/belt_axes.csv); not the XY gantries ELZU and not the DLZPVI). An X/Z gantry needs a config with `geometry.stroke_z_m`, a single axis one with `geometry.stroke_m`. Sets pulley radius, belt, teeth in mesh, the pulley's torque rating and the moved masses; keeps the strokes (an EXCT takes the shortest Z stroke offered that holds the config's) and refuses a stroke the axis does not have. None named in any config: E's numbers are those `"EXCT-30"` gives |

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
| `guide_friction` | T E S | on | LuGre friction in the linear guides |
| `load_dependent_friction` | T E S | on | Friction levels scale with the guide load |
| `motor_friction` | T E S | on | Coulomb and viscous friction on the motor shafts |
| `gearbox_backlash` | T S on, E off | | Backlash as a dead zone. Off in E: direct drive |
| `gearbox_efficiency` | T S on, E off | | Torque loss in the direction of power flow. Off in E: direct drive |
| `torque_ripple` | T E S | on | Torque ripple (proportional to torque) and cogging (independent of it) |
| `tooth_meshing` | T E S | on | A small periodic force at the tooth-pass frequency |
| `frame_compliance` | T E | on | One spring-mass mode of the frame in X |
| `tool_flex` | T E | on | The Z axis leans against the carriage; the reported X position is the tool tip |
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
| `back_emf` | T E S | on | Back-EMF of the turning rotor in the winding |
| `voltage_limit` | T E S | on | The winding voltage is limited to sqrt(3/2) times the DC link voltage |
| `winding_heating` | T E S | on | The winding warms with its copper loss and its resistance rises with it |
| `part_heating` | T E S | off | Each gearbox and guide warms with its loss power, as one thermal mass with the values of `[thermal]`. Only computed and shown (signals `temperature.<part>`, the thermal view, the 3D colours), unless `thermal_feedback` is on |
| `thermal_feedback` | T E S | off | Guide friction and gearbox losses follow the parts' temperatures, the belt's EA the ambient temperature (`[thermal_feedback]`, see the [model reference](model-reference.md#temperature-feedback)). Warms the parts as `part_heating` does, whether that is on or not. Replaces `temperature`: its hand-set factors are not used while this is on. Off by default, so that earlier results stay as they were |
| `belt_tooth_jump` | T E S | off | Elastic model: the belt's teeth climb out of a driven pulley's grooves when the effective pull there exceeds `belt.tooth_jump_factor` · `ratings.belt_teeth_in_mesh` · pitch / radius times the slack side's tension, and the belt slides over the pulley until it falls into the next groove (a jump) or back. Signals `pulley.<motor>.tooth_jumps`, `.tooth_slip`, `power.tooth_jump`; check `belt-tooth-jump`. Needs `belt_mass`. See the [model reference](model-reference.md#belt-tooth-jump). Off by default |
| `belt_span_vibration` | T E S | off | The first sideways mode of each free belt span, a string at its section's live tension (signals `belt.span.<i>.frequency`), against the tooth mesh and the motors' ripple and cogging (`belt.span_detuning`); check `belt-span-resonance` against `ratings.belt_span_resonance_band` at steady speed. Signals and a check only. Off by default |
| `torque_saturation` | T E S | on | The torque constant falls towards the maximum current |
| `drive_delay` | T E S | on | Computation delay of the current and the velocity loop, in whole cycles |
| `dc_link` | T E S | on | A simulated DC link per drive with capacitance, braking chopper, resistor and overvoltage trip, built from `[ratings]`. The ratings of T and S are placeholders. Off, the DC link voltage stays at `drive.bus_voltage_v` and the braking checks are estimates |
| `shared_dc_link` | T E S | off | With `dc_link`: the drives of the machine share one DC link (one Line Module, as on a SINAMICS S120). What a braking motor feeds back a motoring one draws. The link has the capacitance of all drives together (each brings `ratings.dc_link_capacitance_uf`), one chopper with one braking resistor of the `[ratings]` powers, and one trip; every motor reports the same voltage and resistor power. Switched on while the machine runs, links that differ put their energies together. Changes nothing on a machine with one drive |
| `regenerative_supply` | T E S | off | With `dc_link`: a regenerative supply (an Active Line Module, as on a SINAMICS S120). Above the bus level it feeds the DC link's energy back into the mains, up to `ratings.line_module_max_power_w`, of which `ratings.line_module_efficiency` arrives there; the braking resistor takes only what is beyond that power. Reported as energy fed back into the mains and as line module loss (analysis, budget, `analyze` column `mains_energy_j`, the app), and judged by `line-module-continuous` and `line-module-peak`. Off by default, so that earlier results stay as they were |
| `fieldbus_delay` | T E S | off | The motor setpoints reach the drives some profile cycles late |
| `fieldbus_jitter` | T E S | off | Each frame of motor setpoints takes from 0 to `profile.fieldbus_jitter_cycles` more cycles than the delay, evenly and pseudo-random from `drive.noise_seed` (every run the same); the drives use the newest frame that has arrived, so a late frame leaves them the one before. Works with or without `fieldbus_delay`. Off by default, so that earlier results stay as they were |
| `sensor_noise` | T E S | off | Gaussian noise on the measured position and current, repeatable from a seed |

**Motion planning**

| Switch | In | Default | What it does |
| --- | --- | --- | --- |
| `corner_blending` | T S off, E on | | The next cycle segment starts while the current one is still braking |
| `input_shaping` | T S off, E on | | ZVD input shaper on the setpoints of all axes |
| `shaper_follows_pose` | T E S | off | With `input_shaping`: the shaper takes its frequency from the resonance the machine reports for the planned pose. Only the T-gantry reports one |

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

T-gantry only. With `effects.frame_compliance`: the beam and its mounting as one mass on a spring in X.

| Key | In | Unit | Meaning | Source |
| --- | --- | --- | --- | --- |
| `mass_kg` | T E | kg | Modal mass of the frame | estimate, measurement |
| `stiffness_n_m` | T E | N/m | Stiffness of the frame in X | estimate, measurement |
| `damping_n_s_m` | T E | N s/m | Damping of the frame mode | estimate, measurement |

## `[tool_flex]`

T-gantry only. With `effects.tool_flex`: the guide of the Z axis gives way sideways and in tilt.

| Key | In | Unit | Meaning | Source |
| --- | --- | --- | --- | --- |
| `lateral_stiffness_n_m` | T E | N/m | Sideways stiffness of the Z guide | estimate |
| `tilt_stiffness_nm_rad` | T E | N m/rad | Tilt stiffness of the Z guide | estimate |
| `damping_n_s_m` | T E | N s/m | Damping of the leaning motion | estimate |

## `[idler]`

T-gantry only. With `effects.idlers`: the four carriage idlers and the one at the bottom of the Z axis, all alike. The table has to be present and `radius_m` positive even with the effect off.

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

## `[ratings]`

What the parts allow. `sim-cli verdicts` judges a cycle against these. **A rating that is left out is not judged.** With `effects.dc_link` the capacitance, the braking threshold, the resistor's peak power, the trip level and the discharge time constant are required, because the simulated DC link is built from them (with `effects.regenerative_supply` also the line module's maximum power and efficiency), and the threshold has to lie between `drive.bus_voltage_v` and `dc_link_overvoltage_v`. `verdicts` refuses a threshold at or below `drive.bus_voltage_v` with the effect off as well: it would leave the capacitors a negative room.

| Key | In | Unit | Meaning | Judged by | Source in the shipped configs |
| --- | --- | --- | --- | --- | --- |
| `belt_max_tension_n` | T E S | N | Allowable belt tension. E: 2140 N. Festo publishes no allowable belt tension for the EXCT. It states the 5 mm pitch, the steel cords, the 30 mm width and the stretch of the belt: 0.045 % at 300 N, which is 667 kN per unit strain. Megadyne's Megalinear handbook lists for 5 mm pitch steel-cord polyurethane belts, at 4 mm/m stretch: T5 840 N at 25 mm and 1060 N at 32 mm (about 250 kN per unit strain at 30 mm), AT5 1805 N at 25 mm and 2275 N at 32 mm (about 535 kN). Festo's belt is somewhat stiffer than AT5 with standard steel cords and far stiffer than T5, so the limit is the AT5 figure between the two widths: 2140 N at 30 mm. The figures were read from the handbook itself (average values for open-ended belts, a quarter of the breaking strength); that Festo's belt has cords of that class is concluded from the stretch, not stated by Festo. The teeth give way before the cords do: that is judged by `pulley_max_torque_nm`. | `belt-tension` | placeholder (T S), estimate (E) |
| `belt_min_tension_n` | T E S | N | Lowest tension allowed. Below it the teeth can jump | `belt-slack` | choice (0) |
| `settling_tolerance_m` | T E S | m | How far the tool may be from its target in the second half of a dwell | `settling` | choice |
| `pulley_max_torque_nm` | E | N m | Allowable torque at the pulley. Also caps the peak torque of a motor by name | `pulley-torque` | datasheet (Festo: 12.5 N m) |
| `gearbox_rated_torque_nm` | T S | N m | Gearbox rated (continuous) torque at the output. A gearbox by name sets it | `gearbox-continuous` | datasheet (as Siemens NRB060, ratio 5) |
| `gearbox_max_torque_nm` | T S | N m | Gearbox maximum torque at the output. A gearbox by name sets it | `gearbox-peak` | datasheet (as Siemens NRB060, ratio 5) |
| `gearbox_max_input_speed_rpm` | T S | r/min | Highest speed at the gearbox input. Not in the catalogue's gearbox files | `gearbox-input-speed` | placeholder |
| `shaft_max_radial_load_n` | T E S | N | Radial load the belt may put on a pulley shaft. Not in the gearbox files; not published by Festo | `gearbox-radial-load` | placeholder (T S), estimate (E) |
| `guide_max_load_n` | T E S | N | Highest load on a guide | `guide-load` | placeholder (T S), estimate (E) |
| `guide_dynamic_load_rating_n` | T E S | N | Dynamic load rating C of the life estimate L10 = (C / P)³ · 50 km | `guide_life_km` of `analyze`; in hours `guide-life` | placeholder (T S), estimate (E) |
| `required_life_h` | T E S | h | The life bearings, gearbox and guide are to last. 20000 h in all three, the life Siemens rates its planetary gearboxes' planet bearings for | `pulley-bearing-life`, `gearbox-life`, `guide-life` (each passes at or above it) | choice |
| `bearing_reliability_percent` | T E S | % | Reliability of the two bearing lives: the life factor a1 of ISO 281 turns their L10 (90 %) into the life this share of bearings reaches (0.64 at 95 %, 0.25 at 99 %). 90 to 99.95; 90 in all three, which leaves the lives as they were | `pulley-bearing-life`, `gearbox-life` | choice |
| `pulley_bearing_load_rating_n` | T E S | N | Dynamic load rating C of the bearing that takes the belt's pull on a pulley shaft, for the rating life (C / P)³ · 10⁶ / (60 n) h. T S: 8469 N, the output bearing of a Siemens NRB120 (1500 N for 30000 h at 100 r/min, Table 7-14 of the planetary gearbox configuration manual 01/2023; the NRB060 whose torques T S take would allow 340 N, less than their 600 N of pretension on the shaft). E: 9500 N for two deep-groove ball bearings of about 5.85 kN that share the pull (a pair lasts as one bearing of 1.62 C) | `pulley-bearing-life` | placeholder from the catalogue (T S); estimate (E), bounds 8000 to 12000 N |
| `gearbox_reference_speed_rpm` | T E S | r/min | Output speed at which a planetary gearbox's planet bearings last 20000 h at the rated torque, for Siemens' life formula. 175 r/min, the NRB060 with ratio 5 (Table 7-15). A gearbox by name does not set it. In E not used: no gearbox and no `gearbox_rated_torque_nm` | `gearbox-life` | catalogue |
| `belt_tooth_load_rpm`, `belt_tooth_load_n_cm` | T E S | r/min, N/cm | Two arrays of the same length, speeds rising: the specific tooth load the belt's teeth allow per cm of width and tooth in mesh, at the pulley's speed; interpolated, held beyond the ends. Megadyne's table for open-ended AT5 (MEGALINEAR handbook 02.2026, page 52). Not in the parameter panel (arrays) | `belt-tooth-load` | catalogue (E); placeholder (T S) |
| `belt_width_mm` | T E S | mm | Belt width. E: 30 mm (Festo); T S: 25 mm | `belt-tooth-load` | catalogue (E), placeholder (T S) |
| `belt_service_factor` | T E S | – | Factor on the effective pull at the pulley. 1.4, Megadyne's factor for low shock loads (Table 4 of the handbook) | `belt-tooth-load` | choice |
| `belt_teeth_in_mesh` | T E S | – | Teeth in mesh on a driven pulley; at most 12 count, as the handbook says. 10 in all three (half of the EXCT-30's 20-tooth pulley). With `effects.belt_tooth_jump` also z_m of the climb-out law (see `belt.tooth_jump_factor`) | `belt-tooth-load` | estimate, bounds 6 to 12 (T S), 7 to 12 (E) |
| `drive_rated_current_a` | T E S | A | Rated output current of the drive, against which the RMS motor current of the cycle is judged. A drive by name sets it. E: 5 A, its converter 6SL3210-5HE11-5UF0; T S: 3 A, the S210 converter 6SL3210-5HE11-0UF0, which carries the placeholder motor's rated current (2.8 A) | `drive-current` | catalogue (E), placeholder (T S) |
| `drive_max_current_a` | T E S | A | Maximum output current of the drive, against which the peak motor current is judged. The drive's current limit is not simulated. E: 19 A; T S: 10.9 A (above the placeholder motor's 9.5 A) | `drive-peak-current` | catalogue (E), placeholder (T S) |
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
| `shaper2_frequency_hz` | T E S | Hz | A second resonance: a second ZVD shaper at this frequency is convolved with the first, so that both are suppressed; each move then takes both periods longer. Not negative; 0 for none (T S). E: 17.5 Hz, the peak of the tool's response, so that the worst corner of the bounds passes `settling` (0.095 mm against 0.115 mm with the 25 Hz shaper alone). It does not follow the pose | measurement (`sim-cli sweep`), or the machine's own figure | `input_shaping` |
| `shaper2_damping` | T E S | fraction | Damping ratio of the second resonance. From 0 to below 1 | estimate, measurement | `input_shaping`, with `shaper2_frequency_hz` above 0 |
| `torque_reserve` | T E S | fraction | Share of the torque-speed curve a time-optimal path may use. Above 0, at most 1 | choice | `--time-optimal` |
| `tension_reserve` | T E S | fraction | Share of the room between the belt tension that holds the tool and `ratings.belt_max_tension_n` / `belt_min_tension_n` (each less `tension_margin_n`) that a time-optimal path may use in the plan's rigid model. 0.5 in all three files: a change of acceleration quick against the belt's period makes a lightly damped belt overshoot by up to the change itself, so the full model swings up to twice the rigid model's dynamic tension (0.8 let the default T-gantry pass a 330 N rating by 1.4 N at 100 m/s³ and go slack at 1000 m/s³). Above 0, at most 1. Without a tension rating the belt is no limit of the plan | choice (E: estimate) | `--time-optimal` |
| `tension_margin_n` | T E S D | N | Taken off both belt tension ratings before `tension_reserve`, for the guide friction the belt pulls, which the loss-free rigid model leaves out: the X guide's coulomb friction plus its viscous friction at the axis' top speed (T 15 + 20 · 0.8 = 31 N, E 20 + 20 · 2.5 = 70 N, S 15 + 20 · 1.0 = 35 N; D 0, no belt). At least 0. A path whose holding tension alone is beyond a rating less the margin is refused | estimate | `--time-optimal` |
| `min_clearance_m` | T E S | m | For `sim-cli optimise` and the app's optimiser: how far the tool has to stay beyond the farther of pick and place (along the last axis, the side the lift goes to) while it traverses between them. The optimiser lowers each lift, traverse and lower of the cycle to this, and rounds its corners through a via point only where the planned path keeps it (less the path tolerance). Not negative. T 0.3 and E 0.17: the lifts as configured (no obstacle is known); S 0 (no lift) | choice, or the height of an obstacle | `optimise` |
| `path_tolerance_m` | T E S | m | How far a planned path may leave the lines, arcs and splines of its file, so that transition curves can take it from a line into an arc at speed, and a spline's end can be eased to the curvature of the line or arc it meets. The deviation is L² / (24 r) for curves of length L at an arc of radius r, 0.01728 dk l² for a spline eased over l against a step dk of the curvature, and is reported. Not negative; 0 (in all three files) plans the path as given | choice | `--path`, and the app's Path |

## `[bounds]`

Low and high bound of the values that are estimates. The table mirrors the config: where the config has `[friction_x] coulomb_n = 15.0`, `[bounds]` has `friction_x.coulomb_n = [10.0, 20.0]`, in the unit of the value. Used by `sim-cli verdicts --bands`, by `sim-cli sensitivity` and by the app's sensitivity ranking.

- A bound on a key the config does not have is refused.
- A bound has to be two numbers with low ≤ high.
- A test holds every bound around its value (`the_bounds_of_every_config_bracket_their_values`).
- Arrays are not shown in the parameter panel, so bounds cannot be edited in the app.

| Config | Bounded values | What is bounded | What is not |
| --- | --- | --- | --- |
| T | 34 | Belt stiffness, pretension, damping, tooth force, tooth jump factor; guide Coulomb, stiction and viscous friction of both axes; motor shaft friction, ripple, cogging; gearbox efficiency, stiffness, damping; frame stiffness and damping; the three `tool_flex` values; the thermal resistances and time constants of `[thermal]`; the four estimates of `[thermal_feedback]`; the DC link's discharge time constant and the line module's efficiency; the belt's teeth in mesh | Masses, inertias, backlash; values of effects that are off by default, but for `[thermal]`, `[thermal_feedback]` and the tooth jump factor; the shape of the friction curve (Stribeck speed, bristles, preload); the rated rises of `[thermal]`; the reference temperature; the span resonance band (a limit) |
| S | 26 | As T, without `friction_z`, `frame` and `tool_flex` | As T |
| D | 26 | The geometry and the masses (not the payload), the joint's friction, the motor's friction, ripple, cogging and winding resistance, the gearbox's stiffness and damping, `[thermal]`, three of `[thermal_feedback]`, the DC link's discharge and the line module's efficiency | The stop angles and stiffness, the working range, the axis limits, the loop gains, every `[catalogue]` value |
| E | 74 | Every `[estimate]` of the file, including geometry estimates, belt mass, all friction values, winding resistance and inductance, brake torque, pulley inertia, frame mass, idler, cable chain, process force, hard stops, sensor noise, the estimated ratings (the discharge time constant and the line module's efficiency among them), the thermal resistances and time constants, the tooth jump factor and the four estimates of `[thermal_feedback]` | The ripple and cogging orders and the loop gains; all `[catalogue]` values; the rated rises of `[thermal]`; the span resonance band (a limit) |

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
| `forearm_elbow_share` | share | The share of a pair's mass counted at the elbow, the rest at the platform (0 to 1). 2/3 matches the rod's inertia about its end (Codourey, Olsson) | 0.6667 |
| `platform_kg` | kg | The platform with its flange | 1.2 |
| `payload_kg` | kg | Payload, with `effects.payload` | 3.0, catalogue (Autonox rated payload) |

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

D's values of the shared tables: motor 1FK2105-4AF at 400 V (catalogue: rotor 1.7 kg cm², 1.07527 N m/A, 15 N m peak, 4.6 N m rated at 3000 r/min, 18 A and 4.35 A, 37 min; winding R and L estimates), gearbox Siemens NRB080-05-20 (catalogue: ratio 20, 96 %, 35752.6 N m/rad, 9 arcmin backlash, 0.406 kg cm² at the input), converter 6SL3210-5HE11-5UF0 (5 A, 19 A), DC link capacitance and chopper threshold estimates as the EXCT-30's, the converter's internal braking resistor (100 W, 5.7 kW) and trip level (820 V) from `braking_s210.csv`, `settling_tolerance_m` 0.5 mm (an estimate of the positioning accuracy of the class, Festo EXPT ±0.5 mm; the gearboxes' sag under the platform's weight puts the tool 0.64 mm off at pick and place at rest and 0.68 mm in the dwell, so the cycle fails `settling`: a sizing finding, the NRB080's backlash is too large for the class; 3 arcmin pass with 0.33 mm), `gearbox_reference_speed_rpm` 100 (estimate), the velocity loop 0.25 N m s/rad, 0.03 s with 0.1 ms filters (tuned with `sim-cli margins delta-d3 --bands`). Its cycle is the Adept cycle 25/305/25 mm with stops at the corners and 35 ms gripper dwells, one pass = one place and one pick; `profile.min_clearance_m` is its 25 mm lift, `start.pose` above the place position (near the edge of the range one arm carries little of the weight and its velocity loop cannot be measured), `mass.forearm_elbow_share` bounded 0.5 to 0.667. The suite's loss-free configs (`suite_configs`) set `workspace.top_z_m` to −0.85, see the [model reference](model-reference.md#delta-picker-d3).

## `[[cycle.segment]]`

The cycle is a list of segments, run in order and repeated. All three configs have one. It cannot be edited in the parameter panel; `--set "cycle.segment = [{...}, ...]"` replaces it whole (TOML inline tables), and so does "Apply" of the app's optimiser where it changed the cycle's lifts.

| Key | Unit | Meaning |
| --- | --- | --- |
| `target` | m | Tool pose to move to, one position per axis, inside the strokes |
| `dwell_s` | s | Time to stay there. Rounded to whole profile cycles. With input shaping the dwell starts when the shaped setpoint has arrived |
| `payload` | true/false | State of the gripper once the segment has arrived (with input shaping: once the shaped setpoint has): `true` picks up or keeps the payload, `false` releases it |
| `via` | m | Optional, in no shipped file. Via points, each one position per axis inside the strokes: the segment then runs a path planned within the axis limits (`path::plan`, with the transition curves of `profile.path_tolerance_m`) instead of the straight move: a line to the first via point, a smooth curve (spline) through the others, and a line from the last to `target`. With via points straight above pick and place and one between, a lift, traverse and lower without a stop; the optimiser writes such segments. One via point is two lines with a stop between. The path is planned from the segment before's target when the config is read, and refused (`cycle.segment[i].via`) where it cannot be planned or leaves the strokes. A segment with via points does not blend (`corner_blending`) with the one before. The first segment of a run starts from the start pose; were its path refused from there, it runs the straight move |

T and E: lower, dwell (grip), lift, traverse, lower, dwell (release), lift, traverse back; six segments. S: a shuttle between x = 0.9 and x = 0.1 with a dwell of 0.1 s at each end; two segments.
