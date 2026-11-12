# Config reference

One TOML file per machine. Units are in the key names. The key list below was generated from the three shipped configs, so every key they have is here:

| Short | File | Machine |
| --- | --- | --- |
| T | [`crates/machine-t-gantry/default.toml`](../crates/machine-t-gantry/default.toml) | T-gantry, made-up mid-size machine. Placeholder values |
| E | [`crates/machine-t-gantry/exct-30.toml`](../crates/machine-t-gantry/exct-30.toml) | T-gantry: Festo EXCT-30 with Siemens 1FK2205-4AF. Each value marked `[catalogue]` or `[estimate]` in the file |
| S | [`crates/machine-single-axis/default.toml`](../crates/machine-single-axis/default.toml) | Single belt axis. Placeholder values |

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
- `gearbox.efficiency` and `profile.torque_reserve` are above 0 and at most 1; `motor.ripple_fraction` and `profile.shaper_damping` are from 0 to below 1; `drive.no_load_speed_rad_s` is above `drive.rated_speed_rad_s`.
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
| `motor` | T E S | Article number of a motor in [`catalog/siemens/motors_s210.csv`](../catalog/siemens/motors_s210.csv), e.g. `"1FK2205-4AF"`. E names its motor; T and S name none |
| `motor_supply` | T E S | `"230v"` or `"400v"`: the supply the motor's rated data is taken for. `"400v"` in all configs |
| `gearbox` | T E S | A gearbox as size-frame-ratio, e.g. `"NRK070-04-5"`, from the three `gearbox_*.csv` files. None named in any config |

A part by name also retunes `drive.velocity_kp_nm_s_rad`: the gain is scaled by the square root of (ratio of the inertia the motors turn) × (ratio of the rotor inertias, the gearbox's own included), new over old. The reference is the config's own numbers as the file has them before the part goes in, so a config states its gain for the rotor, gearbox and masses it has as numbers, whichever part it names. A motor by name sets `drive.bus_voltage_v` to the DC link voltage of its supply and scales `ratings.braking_threshold_v` and `ratings.dc_link_overvoltage_v` with it. The gearbox is put in before the motor, so the motor's peak torque is capped with the ratio in effect.

`[parts]` is not shown in the app's parameter panel, which shows numbers only; the app's motor search sets it with the "Use" button of a row. A gain edited there is the gain for the config's own numbers; with a part named, the simulation runs with the retuned value.

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
| `holding_brake` | T E S | on | Motor holding brakes that act while the drives are disabled. Elastic model only |
| `temperature` | T E S | off | Scale factors on friction and belt stiffness, set by hand in `[temperature]` |

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
| `torque_saturation` | T E S | on | The torque constant falls towards the maximum current |
| `drive_delay` | T E S | on | Computation delay of the current and the velocity loop, in whole cycles |
| `dc_link` | T E S | on | A simulated DC link per drive with capacitance, braking chopper, resistor and overvoltage trip, built from `[ratings]`. The ratings of T and S are placeholders. Off, the DC link voltage stays at `drive.bus_voltage_v` and the braking checks are estimates |
| `shared_dc_link` | T E S | off | With `dc_link`: the drives of the machine share one DC link (one Line Module, as on a SINAMICS S120). What a braking motor feeds back a motoring one draws. The link has the capacitance of all drives together (each brings `ratings.dc_link_capacitance_uf`), one chopper with one braking resistor of the `[ratings]` powers, and one trip; every motor reports the same voltage and resistor power. Switched on while the machine runs, links that differ put their energies together. Changes nothing on a machine with one drive |
| `fieldbus_delay` | T E S | off | The motor setpoints reach the drives some profile cycles late |
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

Applied only with `effects.temperature` on. A cold machine has more friction and a stiffer belt. Set by hand; nothing computes these.

| Key | In | Unit | Meaning | Source |
| --- | --- | --- | --- | --- |
| `friction_scale` | T E S | factor | On the Coulomb, stiction and viscous levels of the guides | estimate |
| `belt_stiffness_scale` | T E S | factor | On `belt.stiffness_ea_n` | estimate |

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
| `velocity_kp_nm_s_rad` | T E S | N m s/rad | Velocity loop gain Kp. 0.08 in T and S, 0.6 in E. Stated for the rotor, gearbox and masses the config has as numbers; a part by name retunes it from there (see [`[parts]`](#parts)) | choice | always |
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
| `noise_seed` | T E S | – | Seed of the noise. A whole number, not negative. Each motor has a stream of its own | choice | `sensor_noise` |

## `[ratings]`

What the parts allow. `sim-cli verdicts` judges a cycle against these. **A rating that is left out is not judged.** With `effects.dc_link` the capacitance, the braking threshold, the resistor's peak power and the trip level are required, because the simulated DC link is built from them, and the threshold has to lie between `drive.bus_voltage_v` and `dc_link_overvoltage_v`. `verdicts` refuses a threshold at or below `drive.bus_voltage_v` with the effect off as well: it would leave the capacitors a negative room.

| Key | In | Unit | Meaning | Judged by | Source in the shipped configs |
| --- | --- | --- | --- | --- | --- |
| `belt_max_tension_n` | T E S | N | Allowable belt tension. E: 2140 N. Festo publishes no allowable belt tension for the EXCT. It states the 5 mm pitch, the steel cords, the 30 mm width and the stretch of the belt: 0.045 % at 300 N, which is 667 kN per unit strain. Megadyne's Megalinear handbook lists for 5 mm pitch steel-cord polyurethane belts, at 4 mm/m stretch: T5 840 N at 25 mm and 1060 N at 32 mm (about 250 kN per unit strain at 30 mm), AT5 1805 N at 25 mm and 2275 N at 32 mm (about 535 kN). Festo's belt is as stiff as the AT5 class and far stiffer than T5, so the limit is the AT5 figure between the two widths: 2140 N at 30 mm. Read from the handbook itself. | `belt-tension` | placeholder (T S), estimate (E) |
| `belt_min_tension_n` | T E S | N | Lowest tension allowed. Below it the teeth can jump | `belt-slack` | choice (0) |
| `settling_tolerance_m` | T E S | m | How far the tool may be from its target in the second half of a dwell | `settling` | choice |
| `pulley_max_torque_nm` | E | N m | Allowable torque at the pulley. Also caps the peak torque of a motor by name | `pulley-torque` | datasheet (Festo: 12.5 N m) |
| `gearbox_rated_torque_nm` | T S | N m | Gearbox rated (continuous) torque at the output. A gearbox by name sets it | `gearbox-continuous` | datasheet (as Siemens NRB060, ratio 5) |
| `gearbox_max_torque_nm` | T S | N m | Gearbox maximum torque at the output. A gearbox by name sets it | `gearbox-peak` | datasheet (as Siemens NRB060, ratio 5) |
| `gearbox_max_input_speed_rpm` | T S | r/min | Highest speed at the gearbox input. Not in the catalogue's gearbox files | `gearbox-input-speed` | placeholder |
| `shaft_max_radial_load_n` | T E S | N | Radial load the belt may put on a pulley shaft. Not in the gearbox files; not published by Festo | `gearbox-radial-load` | placeholder (T S), estimate (E) |
| `guide_max_load_n` | T E S | N | Highest load on a guide | `guide-load` | placeholder (T S), estimate (E) |
| `guide_dynamic_load_rating_n` | T E S | N | Dynamic load rating C of the life estimate L10 = (C / P)³ · 50 km | `guide_life_km` of `analyze` | placeholder (T S), estimate (E) |
| `dc_link_capacitance_uf` | T E S | µF | DC link capacitance of one drive | `dc-link-energy` | placeholder (T S); estimate from S120 data (E) |
| `braking_threshold_v` | T E S | V | DC link voltage at which the chopper switches the braking resistor in. T S: 224.65 V. A motor by name scales it with the DC link voltage | `dc-link-energy` | placeholder (T S); estimate (E) |
| `braking_resistor_continuous_w` | T E S | W | Continuous power of the braking resistor | `braking-resistor-continuous` | placeholder (T S); estimate (E) |
| `braking_resistor_peak_w` | T E S | W | Peak power of the braking resistor. The simulated resistor never takes more | `braking-resistor-peak` | placeholder (T S); estimate (E) |
| `dc_link_overvoltage_v` | T E S | V | DC link voltage at which the drive trips. T S: 257.6 V. A motor by name scales it with the DC link voltage | `dc-link-overvoltage` (only with `effects.dc_link`) | placeholder (T S); estimate (E) |

The DC link ratings of T and S are placeholders for the drive those configs were written for: a DC link of 155.13 V, with 6.2 J of room in 470 µF up to the threshold. When a catalogue motor goes into such a config, the threshold and the trip level keep their ratio to the DC link voltage (782 V and 897 V at 540 V); capacitance and resistor powers stay. See the [user guide](user-guide.md#check-a-real-catalogue-machine).

## `[profile]`

| Key | In | Unit | Meaning | Source | Effect switch |
| --- | --- | --- | --- | --- | --- |
| `cycle_s` | T E S | s | Cycle of the profile generator. Can be changed while a motion is under way | choice | always |
| `fieldbus_delay_cycles` | T E S | cycles | Whole profile cycles between the controller computing the motor setpoints and the drives using them. 0 to 1000 | datasheet, measurement | `fieldbus_delay` |
| `setpoint_interval_s` | T E S | s | Setpoints from the PLC are interpolated over this interval. A whole multiple of `cycle_s` | choice | always (OPC UA) |
| `blend_time_s` | T E S | s | How long before the end of a cycle segment the next one may start | choice | `corner_blending` |
| `shaper_frequency_hz` | T E S | Hz | Resonance the input shaper is set to | measurement, or the machine's own figure | `input_shaping` |
| `shaper_damping` | T E S | fraction | Damping ratio of that resonance. From 0 to below 1 | estimate, measurement | `input_shaping` |
| `torque_reserve` | T E S | fraction | Share of the torque-speed curve a time-optimal path may use. Above 0, at most 1 | choice | `--time-optimal` |
| `path_tolerance_m` | T E S | m | How far a planned path of lines and arcs may leave them, so that transition curves can take it from a line into an arc at speed. The deviation is L² / (24 r) for curves of length L at an arc of radius r, and is reported. Not negative; 0 (in all three files) plans the path as given | choice | `--path`, and the app's Path |

## `[bounds]`

Low and high bound of the values that are estimates. The table mirrors the config: where the config has `[friction_x] coulomb_n = 15.0`, `[bounds]` has `friction_x.coulomb_n = [10.0, 20.0]`, in the unit of the value. Used by `sim-cli verdicts --bands`, by `sim-cli sensitivity` and by the app's sensitivity ranking.

- A bound on a key the config does not have is refused.
- A bound has to be two numbers with low ≤ high.
- A test holds every bound around its value (`the_bounds_of_every_config_bracket_their_values`).
- Arrays are not shown in the parameter panel, so bounds cannot be edited in the app.

| Config | Bounded values | What is bounded | What is not |
| --- | --- | --- | --- |
| T | 22 | Belt stiffness, pretension, damping, tooth force; guide Coulomb, stiction and viscous friction of both axes; motor shaft friction, ripple, cogging; gearbox efficiency, stiffness, damping; frame stiffness and damping; the three `tool_flex` values | Masses, inertias, backlash; values of effects that are off by default; the shape of the friction curve (Stribeck speed, bristles, preload) |
| S | 14 | As T, without `friction_z`, `frame` and `tool_flex` | As T |
| E | 61 | Every `[estimate]` of the file, including geometry estimates, belt mass, all friction values, winding resistance and inductance, brake torque, pulley inertia, frame mass, idler, cable chain, process force, hard stops, sensor noise and the estimated ratings | The ripple and cogging orders and the loop gains; all `[catalogue]` values |

Keys with bounds in one config only:

| Bounded in T and S, not in E | Bounded in E only |
| --- | --- |
| `belt.stiffness_ea_n` (a catalogue value in E), `gearbox.efficiency` (1.0 in E) | `geometry.span_min_m`, `.z_com_drop_m`, `.guide_spacing_m`, `.z_tool_drop_m`; `belt.mass_kg_m`, `.tooth_mesh_stiffness_n_m`, `.damping_time_s`; `friction_x` and `friction_z`: `.stribeck_m_s`, `.bristle_stiffness_n_m`, `.bristle_damping_n_s_m`, `.preload_n`; `motor.friction_smoothing_rad_s`, `.resistance_ohm`, `.inductance_h`, `.brake_torque_nm`; `pulley.inertia_kgm2`; `frame.mass_kg`; all of `idler`, `cable_chain`, `process`, `hard_stop`; `drive.encoder_noise_counts`, `.current_noise_a`; `ratings.dc_link_capacitance_uf`, `.braking_threshold_v`, `.braking_resistor_continuous_w`, `.braking_resistor_peak_w`, `.dc_link_overvoltage_v`, `.shaft_max_radial_load_n`, `.guide_max_load_n`, `.guide_dynamic_load_rating_n` |

## `[start]`

| Key | In | Unit | Meaning |
| --- | --- | --- | --- |
| `pose` | T E S | m | Tool pose the simulation starts at, at rest: one position per axis. T: `[0.1, 0.35]`, E: `[0.1, 0.2]`, S: `[0.1]`. A path, a trajectory and a replayed trace have to start here |

## `[[cycle.segment]]`

The cycle is a list of segments, run in order and repeated. All three configs have one. It cannot be edited with `--set` or in the app.

| Key | Unit | Meaning |
| --- | --- | --- |
| `target` | m | Tool pose to move to, one position per axis, inside the strokes |
| `dwell_s` | s | Time to stay there. Rounded to whole profile cycles. With input shaping the dwell starts when the shaped setpoint has arrived |
| `payload` | true/false | State of the gripper once the segment has arrived (with input shaping: once the shaped setpoint has): `true` picks up or keeps the payload, `false` releases it |

T and E: lower, dwell (grip), lift, traverse, lower, dwell (release), lift, traverse back; six segments. S: a shuttle between x = 0.9 and x = 0.1 with a dwell of 0.1 s at each end; two segments.
