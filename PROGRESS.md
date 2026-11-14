# Progress

Status per phase of "T-Gantry Physics Simulator Project Plan.md". A phase is done when its "Done when"
criterion and its Validation-table checks exist as automated tests and `cargo test --workspace` passes.

| Phase | Status | Tests that prove it |
| --- | --- | --- |
| 1. Base and rigid reference | done | `sim-machines/tests/suite.rs` (round trip, static hold, energy, step-size convergence, determinism, for every registered machine), `sim-machines/tests/architecture.rs` (dependency direction, pinned versions, no machine name outside its crate), `sim-machines/tests/i18n.rs`, constant-torque and Z static-hold tests in the two machine crates |
| 2. Full mechanics | done | Component unit tests in `sim-components/src/mech.rs` (belt, LuGre friction, shaft friction, ripple, tooth meshing, gearbox); friction levels, backlash, belt natural frequency and rigid limit in `machine-single-axis/src/tests.rs`; rigid limit, static hold and the SciPy comparison in `machine-t-gantry/src/tests.rs`; energy and step-size convergence of the elastic models in `sim-machines/tests/suite.rs` |
| 3. Control and profiles | done | `sim-machines/tests/control.rs` (following error at constant speed = v / Kv on every machine, feedforward, pick-and-place cycle with payload switching, external setpoint interpolation, axis-against-motor limit check, determinism, step-size convergence with control and all effects on, every effect switch changes the traces); profile limits and synchronised moves in `sim-components/src/profile.rs`; drive loops, torque-speed limit, encoder and notch in `sim-components/src/drive.rs`; CSV runs in `sim-cli/tests/cli.rs` |
| 4. App | done (interactive handling not checked by a person, see below) | `sim-app/tests/app.rs`: every machine runs in the same app model with scopes, scene, effect switches and parameters generated from what the machine supplies; every label exists in German and English and the window code holds no text; switching any effect changes the traces within one cycle without a restart, friction within 1 ms; pause, slow motion and real-time pacing of the simulation thread; the analysis panel (`the_cycle_is_analysed_and_judged_in_the_app`, `the_app_ranks_the_uncertain_parameters`, `the_app_optimises_the_cycle_and_applies_the_settings`), the Disable and Enable buttons (`the_drives_can_be_disabled_and_enabled_from_the_app`), and what the command line could do alone until 12 November (see "The app does what the command line does" below) |
| 5. Analysis | done | `reference_cycle_torque_matches_hand_calculation` (peak and RMS torque, cycle time, peak speed and use of the torque-speed curve against a hand calculation), the over-curve check and the two frequency-sweep checks (pure inertia; belt dip at the hand-calculated frequency) in `machine-single-axis/src/tests.rs`; `analyze` and `sweep` CSV export in `sim-cli/tests/cli.rs` |
| 6. PLC link | done against an OPC UA test client; blocked for a real PLC (see below) | `sim-opcua/tests/plc.rs`: a client-commanded move runs in the simulator in real time and reports back position and following error, for every machine; every signal of the signal list is a readable variable; streamed setpoints are interpolated and followed; a path sent by the client is planned and run; refused commands are reported |

Stages A to D of the plan's "End goals and the stages that reach them" (added Nov 11, 2026):

| Step | Status | Tests that prove it |
| --- | --- | --- |
| A1 gearbox contact cannot pull | done | `gearbox_flanks_in_contact_never_pull` in `sim-components/src/mech.rs` |
| A2 leaning Z axis | done: the Z axis's centre of mass moves sideways against the guide's lateral and tilt stiffness, softer the further it hangs below the carriage (`effects.tool_flex`, `[tool_flex]`); the X position the simulator reports is the tool tip; signal `axis.x.tool_deflection`. Stiffness values are estimates | `leaning_z_axis_matches_the_two_mass_hand_calculation` in `machine-t-gantry/src/tests.rs`; energy and step-size checks of the suite |
| A3 drive delays | done: computation delay of the current and the velocity loop in whole cycles (`effects.drive_delay`); fieldbus delay, whole profile cycles between the controller computing the motor setpoints (position, speed, acceleration, torque feedforward) and the drives using them (`effects.fieldbus_delay`, `profile.fieldbus_delay_cycles`, off by default); sensor noise, Gaussian on the measured position in encoder counts and on the measured current, pseudo-random from a seed so that runs repeat (`effects.sensor_noise`, `drive.encoder_noise_counts`, `drive.current_noise_a`, `drive.noise_seed`, off by default). The setpoint signals and the path error of the analysis are the controller's, before the fieldbus. Missing: jitter (the delay is a fixed number of cycles); noise values are estimates | `computation_delay_makes_the_current_loop_unstable_at_the_calculated_limit`, `the_torque_setpoint_acts_one_velocity_cycle_late`, `speed_from_a_noisy_encoder_has_the_hand_calculated_rms` (two counts of noise: 0.1358 rad/s unfiltered against 0.1356 by hand, 0.0509 rad/s through the velocity filter against 0.0509) in `sim-components/src/drive.rs`; `a_fieldbus_delay_shifts_the_move_by_its_cycles` in `machine-single-axis/src/tests.rs` (three cycles: the trace is the undelayed one 3 ms later to the last bit, 3.00 mm behind at 1 m/s); `the_same_run_twice_is_bit_identical` in `sim-machines/tests/control.rs`, now also with noise |
| A4 motor heating, saturation, energy | done: the winding warms with its copper loss and its resistance rises with it (`effects.winding_heating`), the torque constant falls towards the maximum current (`effects.torque_saturation`), and the cycle analysis reports RMS current, the temperature rise the winding settles at, energy drawn and fed back, peak braking power and the largest single braking. With `effects.dc_link` each drive has a DC link: the voltage starts at `drive.bus_voltage_v`, what the motor feeds back charges the capacitance (C U dU/dt = -P, stepped as energy), the rectifier holds the bus level from below, at `ratings.braking_threshold_v` the chopper switches the braking resistor in, which burns up to `ratings.braking_resistor_peak_w`, and beyond `ratings.dc_link_overvoltage_v` the drive trips: all drives go off as with `Command::Disable`, until `Command::Enable`. The current controller's voltage limit is sqrt(3/2) times the DC link voltage of the moment (see "Review fixes" below). Signals `motor.<id>.dc_link_voltage` and `.braking_resistor_power`; the analysis reports the resistor's energy and mean power and the highest voltage; the verdicts `dc-link-energy`, `braking-resistor-continuous` and `-peak` judge the simulated values, and `dc-link-overvoltage` the highest voltage (without the effect they are estimated from the fed-back energy as before). The DC link is built from the four `[ratings]` values that were there plus the trip level; with the effect on they are required, and a threshold that is not between bus and trip level is refused. On in `exct-30.toml` (values estimated by the S120 data in `catalog/siemens/line_modules_s120.csv`: 540 V, 44 uF per kW, 770 V; there the resistor's mean power falls from 26 W and 28 W estimated to 11 W and 14 W simulated, because the capacitors hand their energy back); on in the two default configs as well since the review fixes below (their DC link placeholders are scaled with the bus when a catalogue motor goes in). A DC link shared by the drives of a machine is `effects.shared_dc_link` (see "Seven gaps closed" below). Missing: feeding back into the line, the resistor's own heating (its continuous rating is judged against the cycle's mean), a disabled drive's DC link discharging | `the_winding_warms_to_the_rated_rise_at_the_rated_current`, `saturation_takes_torque_at_high_current_only`, `braking_energy_raises_the_dc_link_by_the_hand_calculated_voltage` (4.1 J into 470 uF: 190 V to 231.40 V; held at 250 V with the resistor burning 6.096 J of 12.3 J; a 20 W resistor trips after 0.33 s) in `sim-components/src/drive.rs`; energy balance and settled temperature in `reference_cycle_torque_matches_hand_calculation`, and `the_dc_link_takes_braking_energy_the_resistor_the_rest_and_too_small_a_resistor_trips` (cycle: 2.984 J per braking to 220.9 V; with 4.7 uF the resistor's mean power is 2.654 W, by hand 2.655 W; a 5 W resistor trips at 280 V and fails `dc-link-overvoltage`) in `machine-single-axis/src/tests.rs` |
| A5 belt details | done, off by default so that earlier results stay as they were: tooth stiffness in series with each belt section, once per driven pulley the section ends at (`effects.belt_tooth_stiffness`, `belt.tooth_mesh_stiffness_n_m`); damping in proportion to a section's stiffness instead of one constant (`effects.belt_material_damping`, `belt.damping_time_s`); signal `belt.slack_time`, the seconds any section has spent without tension; on the T-gantry the inertia and bearing friction of the five idlers (`effects.idlers`, `[idler]`), 12 idler masses on X and 4 on Z. The 3D view now turns the idlers at these belt speeds. The analysis reports the slack time of the cycle, and `belt-slack` fails on any. Missing: the tooth jump itself; idlers of the single axis. Values are estimates | `teeth_in_mesh_are_a_spring_in_series_with_the_section`, `material_damping_scales_with_the_section_stiffness` in `sim-components/src/mech.rs`; `belt_natural_frequency_with_tooth_stiffness_matches_hand_calculation`, `a_slack_belt_section_is_counted_and_a_tight_one_is_not` in `machine-single-axis/src/tests.rs`; `idlers_add_the_hand_calculated_mass_and_drag` in `machine-t-gantry/src/tests.rs`; energy and step-size checks of the suite with tooth stiffness and idlers on |
| A6 loads and limits | done on both machines (the single axis has cable chain, process force and holding brake since "Seven gaps closed" below, with the T-gantry's code moved to `CableChain` and `HoldingBrake` in `sim-components/src/mech.rs`): cable chain whose moving mass grows with x, plus drag (`effects.cable_chain`, `[cable_chain]`, off by default); a force on the tool while the cycle dwells (`effects.process_force`, `[process]`, off by default); spring-damper stops beyond both ends of each stroke (`effects.hard_stops`, `[hard_stop]`, on); `Command::Disable` and `Command::Enable` on every machine, and motor holding brakes that hold up to `motor.brake_torque_nm` and slip beyond while disabled (`effects.holding_brake`, on). Disable and Enable are in all three front ends: two buttons in the app, `sim-cli run --disable-at SECONDS`, and the OPC UA variables `command.disable` and `command.enable`. Missing: brakes in the rigid model; a disabled drive's winding does not cool. Values are estimates | `cable_chain_adds_the_hand_calculated_force_at_two_positions`, `process_force_changes_the_holding_torque_by_the_hand_calculated_amount`, `a_carriage_pushed_into_the_hard_stop_rests_at_the_hand_calculated_depth`, `holding_brakes_hold_the_z_axis_when_they_are_strong_enough` in `machine-t-gantry/src/tests.rs`; `a_carriage_pushed_into_the_hard_stop_rests_at_the_hand_calculated_depth` in `machine-single-axis/src/tests.rs` (25 N into 1e6 N/m: 25.00 um at both ends); energy check of the suite with the chain on; `the_drives_can_be_disabled_and_enabled_from_the_app` in `sim-app/tests/app.rs`, `a_run_can_disable_the_drives_on_the_way` in `sim-cli/tests/cli.rs` (no torque from the moment of disabling; with brakes the machine stands within 0.1 mm, without them an axis moves more than 10 mm), `the_plc_disables_and_enables_the_drives` in `sim-opcua/tests/plc.rs` |
| B1 verdicts | done for what the configs have ratings for: `sim-cli verdicts` judges a cycle against the torque-speed curve, the settled winding temperature, pulley and gearbox torque, belt tension and slack, and braking energy against the DC link and the braking resistor (`[ratings]` in the config; a rating left out is not judged). Added: `gearbox-input-speed` (peak motor speed against `ratings.gearbox_max_input_speed_rpm`), `gearbox-radial-load` (the belt's pull on a pulley shaft, the sum of the two section tensions there, new signal `pulley.<id>.radial_load`, against `ratings.shaft_max_radial_load_n`), `guide-load` (new signals `axis.<id>.guide_load`, against `ratings.guide_max_load_n`) and, in `sim-cli analyze`, a guide life L10 = (C / P)^3 * 50 km with `ratings.guide_dynamic_load_rating_n` and the cubic mean load over the cycle's time: the catalogue formula, not validated. **`gearbox-continuous` changed its value:** it judges the cubic mean of the torque weighted with the speed, as gearbox makers size, no longer the RMS (single axis 1.11 N m where the RMS gave 1.01; on the loss-free reference cycle 0.486 against 0.426). `belt-slack` also fails when a section spent any time slack. `analyze` has eight more columns (cubic-mean torque, resistor energy and mean power, DC link peak, slack time, radial load, guide load, guide life). **The gearbox and pulley checks judge the torque at the gearbox output** since "Seven gaps closed" below (single axis 0.85 N m peak and 0.337 N m cubic mean on the loss-free reference cycle, where the motor torque times the ratio gave 1.225 and 0.486). Missing: these four ratings are placeholders or estimates (the gearbox files have no input speed or radial load, so a gearbox by name does not set them; Festo publishes no guide or shaft data for the EXCT); the guide loads are those of the friction model (weight, side force and tilting moment from the commanded acceleration), without moments per block; the rigid model reports twice the pretension as radial load; the belt tension rating of the EXCT. The new figures are in the app's tables since 12 November; lives in hours (pulley shaft bearings, gearbox planet bearings, guide) and the belt's tooth load, with verdicts, too (see the end of this file) | `verdicts_match_the_hand_calculation_and_fail_where_a_part_is_too_small` (cubic mean 0.486 N m, by hand 6.125 kg * 0.02 m * (100 / 1.6)^(1/3) m/s^2; 2387 r/min; 600 N; 39.2 N; guide life 47.3 million km for a cubic mean of 50.92 N), `radial_load_on_the_pulley_shaft_is_the_sum_of_both_section_tensions` (vertical axis: 570.58 N), `a_slack_belt_section_is_counted_and_a_tight_one_is_not` (0.87 s of a 2.2 s cycle at 20 N pretension) in `machine-single-axis/src/tests.rs`, `guide_loads_and_shaft_loads_match_the_hand_calculation` in `machine-t-gantry/src/tests.rs` (X guide 203.5 N, Z guide 118.6 N, shafts 593.46 N), `verdicts_judge_the_default_cycle_of_every_machine` in `sim-cli/tests/cli.rs`, `exct_30_cycle_stays_within_motor_and_belt_limits` in `sim-machines/tests/catalog.rs` |
| B2 bounds and sensitivity | done with one-at-a-time runs, and with all bounded values sampled together (`--bands --samples N`, see "Seven gaps closed" below): a config gives each uncertain value a low and a high bound in `[bounds]`, which mirrors the config's tables (`friction_x.coulomb_n = [10.0, 20.0]`, in the value's unit; arrays, so the parameter panel and its translations do not see them). All three configs have them: every `[estimate]` of `exct-30.toml` but the ripple orders and the loop gains (61 values), friction, damping, stiffness, pretension, ripple and gearbox efficiency of the two default configs (14 and 22 values). `sim-cli verdicts --bands` runs the cycle with each value at its low and its high bound and prints per check the lowest and highest value and the worst-case result; a check passes with margin only if every run passes. `sim-cli sensitivity` uses the bounds where the config has them and `--delta` elsewhere, and prints the low and high value it used. On the EXCT-30 config the bands showed the belt going slack at the low pretension bound (200 N) and, once the tool position included the frame, settling out of tolerance; the config was retuned and passes both band modes. A search for the worst corner (`--bands --worst`) was added on 12 November (see the end of this file); it found the EXCT-30 failing `settling` (0.115 mm), which a second input shaper at 17.5 Hz fixed (0.095 mm). The life verdicts added since fail the EXCT-30's pulley shaft bearings in all three band modes (see the end of this file). Missing: bounds on the datasheet-type values. The bands are in the app since 12 November | `a_bound_on_the_moved_mass_gives_the_hand_calculated_band_and_fails_the_verdict_it_reaches` in `machine-single-axis/src/tests.rs` (gearbox peak torque 0.65, 0.85 and 1.25 N m for 3, 4 and 6 kg, within 2 %); `the_bounds_of_every_config_bracket_their_values` in `sim-machines/tests/catalog.rs`; `verdicts_judge_the_default_cycle_of_every_machine`, `sensitivity_writes_one_ranked_row_per_parameter` in `sim-cli/tests/cli.rs`; `sensitivity_shows_that_the_lifted_mass_matters_and_the_tooth_pitch_hardly` in `machine-t-gantry/src/tests.rs` |
| B3 catalogue | done for motors and gearboxes: data files in `catalog/` (below), compiled into `sim-components/src/catalog.rs`. A config names its parts in `[parts]` (`motor = "<article>"`, `motor_supply = "230v"` or `"400v"`, `gearbox = "<size-frame-ratio>"`, `""` for none) and `catalog::resolve` puts their catalogue values in place of the config's numbers. `Sim::new` and `Sim::reconfigure` do that with every config, so the CLI, the app and the OPC UA server get it, and so do the analysis functions that read a config beside a `Sim`; the caller's config keeps the names. `--motor`, `--supply` and `--gearbox` set these keys (a config without `[parts]` refuses them); the CLI resolves before `--set`, so single values can still be set on top. The parameter panel shows numbers only, so `[parts]` is not in it, and a number of a named part that is changed there is overwritten again. `exct-30.toml` names its motor; its motor numbers stay as documentation. A motor is applied with the DC link of its supply as bus voltage: 540 V for "400v" (1.35 times the line voltage, the value the line module data in `catalog/siemens/line_modules_s120.csv` is given for) and 325 V for "230v" (the peak of 230 V). The default configs are written for 190 V at the winding (a DC link of 155.13 V); before, a catalogue motor in them met its own back-EMF and failed "settling" five hundred to two thousand times over. The current controller's gains stay: they follow from resistance and inductance, which a motor by name does not change, and the cycles run calmly on the higher bus. The braking threshold and the trip level of `[ratings]` are scaled with the bus voltage when a motor is applied, and the velocity gain is retuned wherever a part is named (both since the review fixes below; before, the default configs' threshold of 250 V lay below the motor's bus and gave a negative DC link room that passed). Drives, belts and axes by name since 13 November (section "The inverse search made complete" at the end): `[parts] drive`, `belt`, `axis`, `motor_supply = "s120"`, `--drive`, `--belt`, `--axis`, and six part fields in the app. Missing: guides by name (no guide catalogue); the drive's own current limit is judged (`drive-current`, `drive-peak-current`), not simulated | `parts_named_in_a_config_behave_like_their_numbers`, `a_motor_by_name_gives_the_values_of_the_hand_written_config`, `a_gearbox_by_name_sets_ratio_stiffness_backlash_and_ratings` in `sim-machines/tests/catalog.rs`; `the_search_on_a_default_config_fails_motors_for_physical_reasons` in `sim-machines/tests/catalog.rs` (bus 325 V and 540 V; four motors on each default config: none fails settling, the 2000 r/min motor is ruled out by speed, anything else that fails is torque, temperature, gearbox or braking resistor); `parts_are_named_on_the_command_line` in `sim-cli/tests/cli.rs`; `exct_30_config_agrees_with_the_catalogue` in `machine-t-gantry/src/tests.rs` |
| B4 inverse search | done for motors and gearboxes: `sim-cli search` runs every S210 motor offered at the supply on the config's cycle and lists the passing ones first, lightest motor first (weight stands in for price, which no catalogue has; the gearbox files have no mass). Each candidate's velocity gain is scaled with the geometric mean of two ratios: that of the inertia the motors turn (rigid torque per unit acceleration, summed over axes and motors, over the gear ratio) and that of the rotor inertia alone. Below the belt resonance the loop moves rotor and load together, above it the rotor alone; the mean serves both. Scaling with the first ratio only (the rule before) left the lightest rotors oscillating. `--gearboxes SERIES` also tries each motor with the gearboxes of that series that fit its frame, ratios from half to twice the config's, on machines with a gearbox. Before a cycle is simulated the catalogue numbers are checked against the axis limits (motor speed at the highest axis speed against rated speed, rigid torque at the highest acceleration against maximum torque); what fails there is listed, not simulated, and counted on the error stream (13 of 28 1FK2 on the EXCT-30). A candidate that asks more than that at the axis limits never counts as passing, whatever its cycle does (on a 540 V bus a cycle above the rated speed can pass the verdicts, because the drive still has voltage there). Since 13 November (section "The inverse search made complete" at the end): each motor with its paired drive, the S120 motors (`--supply s120`), gearbox masses, ranking by installed weight as the price stand-in (`cost_kg`, beside `mass_kg`), axis combinations (`--axes`: Festo EXCT and Bahr ELZI on the T-gantry, Bahr single axes on the single axis), all on every core. Missing: a retuning from the machine's frequency response in place of the rule of thumb (on the EXCT-30 the mean lands inside the calm range of every 1FK2 that was tried, 0.06 to 0.3 for the lightest at the config's gain of 0.6 then); prices (none public); belts are named, not searched; the torque screen leaves out gearbox efficiency | `the_search_retunes_the_velocity_loop_for_a_lighter_rotor` (1FK2204-6AF on the EXCT-30: gain 0.45 to 0.23 as calculated by hand, 1.34 N m RMS and 13.4 K retuned, against 1.55 N m of the config's own motor; 1FK2104-5AF, a ninth of the rotor inertia: 5.8 N m and a winding that never settles with the config's gain of 0.45, 5.0 N m and never settling with the old rule's 0.35, 1.28 N m and 22.7 K with the mean's 0.13. With the config's earlier gain of 0.6 the 1FK2204-6AF oscillated too; at 0.45 it runs calmly unretuned), `the_gearbox_search_rules_out_by_catalogue_numbers_only_what_would_fail` (two motors, seven NRK070 ratios each: four ruled out by speed as calculated by hand, each of them runs above its rated speed when simulated), `the_motor_search_ranks_passing_motors_by_weight_and_fails_the_small_ones` in `sim-machines/tests/catalog.rs` |
| C1 paths | done for lines, arcs and splines (`spline,x,z`, see "Seven gaps closed" below): a path file (`line,x,z` and `arc,x,z,cx,cz,cw` or `ccw`, one piece per line, from where the machine stands; `sim-components/src/path.rs`) is timed by the simulator and run as a trajectory (`sim-cli run` and `analyze` with `--path FILE`; `Sim::plan_path`). Each piece gets a speed, acceleration and jerk limit along the path that keeps every axis within its own limits, on an arc with the centripetal acceleration counted in; look-ahead lowers the speed at the junctions until every piece has the room to brake; the path stops at corners (direction turning by more than 1 mrad). Where a line meets an arc the acceleration towards the centre steps, and the speed there is held so low that the step within one profile cycle stays within half the jerk limit (0.07 m/s for a radius of 0.1 m at 100 m/s³), so a rounded corner is slow, unless `profile.path_tolerance_m` (0 in the configs) allows transition curves: see "Arcs entered at speed" below. A path given as poses at times still runs as before (`--trajectory`). Paths are in the app and over OPC UA (both since 12 November). Missing: arcs and splines on machines with more than two axes | `a_planned_path_stays_within_the_axis_limits_and_on_its_pieces` (line, arc, line in 1.83 s: speed, acceleration and jerk of both axes within 0.1 % of their limits and reaching them, every pose on the path, 0.4 m/s on the arc and 0.0707 m/s into it as calculated by hand), `a_corner_is_stopped_at`, `a_path_file_is_read_line_by_line` in `sim-components/src/path.rs`; `a_circle_timed_by_the_planner_gives_the_hand_calculated_path_error` and, for poses at times, `path_error_on_a_circle_matches_the_hand_calculation` in `machine-t-gantry/src/tests.rs`; `a_path_file_is_planned_run_and_analysed_for_every_machine` in `sim-cli/tests/cli.rs` |
| C2 path error | done: the analysis reports how far the tool tip was from the path its setpoint drew (peak and RMS) and how far behind its setpoint (peak), for cycles and trajectories | the same test: both figures within 10 % of the hand calculation for a circle at constant speed |
| C4 vibration-free moves | done for one resonance: an input shaper (ZVD: three copies of the plan, half a period apart) on the setpoints of all axes, set by `profile.shaper_frequency_hz` and `profile.shaper_damping` (`effects.input_shaping`, off by default). Each move ends one period later. With `effects.shaper_follows_pose` the shaper takes its frequency from the machine at the planned pose instead (`Machine::resonance_hz`, `None` by default). The T-gantry reports the lower mode of two masses in a row, the carriage on the belt's X stiffness and the Z axis on its guide: 24.6 Hz at z = 0.05 m, 35.6 Hz at z = 0.15 m (x = 0.9 m). The Z axis alone on its guide, with the carriage held, would be 26.2 Hz and 43.4 Hz; the simulation swings at 25 Hz and 37 Hz, and a shaper set to 43.4 Hz left a fifth of the swing. The weights add up to one whatever the period does, so the setpoint ends on the target exactly. Missing: a second resonance; the frame mode and the stiffness of the control loops are not in the reported frequency; while the period changes the speed and acceleration setpoints are the delayed plan's, not the exact rates of the shaped position | `input_shaping_leaves_under_a_tenth_of_the_tool_swing` in `machine-t-gantry/src/tests.rs`: 0.53 mm of swing after a fast traverse falls to 0.015 mm; `a_shaper_that_follows_the_height_leaves_under_a_tenth_at_both_heights`: at z = 0.15 m 0.115 mm falls to 0.010 mm fixed at 35.6 Hz and following alike; at z = 0.05 m 0.527 mm falls to 0.153 mm with the shaper fixed at 35.6 Hz (29 %) and to 0.0085 mm following (1.6 %) |
| C5 cycle optimiser | done as a search by trial: `sim-cli optimise` tries corner blending and higher speed, acceleration and jerk for as long as the cycle gets shorter and every verdict passes, among them the new settling tolerance (`ratings.settling_tolerance_m`: how far the tool may be from its target in the second half of a dwell). Input shaping never shortens a cycle by itself (every stop waits one period of the shaper), so it is tried where it can help: a faster setting that fails on `settling` alone is run again with the shaper, and with the shaper following the pose where the machine reports a resonance, and taken if it then passes and is shorter; where the shaper is on, switching it off is tried. (Before, the shaper was tried only at unchanged limits and so was never taken.) On the EXCT-30 config as it was then the cycle fell from 2.14 s to 1.57 s; on the retuned one it goes from 2.048 s to 1.492 s, a result that fails `settling` and `belt-slack` in the bands, which the optimiser does not look at. Lift heights and via points are variables since 13 November (see "Cycle geometry, belt tension in the plan, faster analyses" below). Missing: a proof of the optimum; the clearance is one height for the whole traverse (no obstacle shape) | `the_optimiser_shortens_the_cycle_within_the_verdicts` in `machine-single-axis/src/tests.rs`; `the_optimiser_switches_input_shaping_on_where_a_faster_cycle_fails_settling` in `machine-t-gantry/src/tests.rs` (a low traverse with a dwell at both ends: at 1.5 times the speed 0.19 mm off against 0.18 mm allowed without the shaper, 0.07 mm with it, cycle 3.69 s to 2.76 s) |
| C3 time-optimal paths | done, as a plan within the motors' limits and the axes' jerk limits (not the fastest plan possible): `--time-optimal` with `--path` times the path as the plan within the axis limits does, with the speed and acceleration limit of each piece taken from the motors. The rigid model gives, at nine poses along a piece, how fast each motor turns per speed of the tool and the torque it needs per acceleration; a motor may turn at its rated speed and give `profile.torque_reserve` (0.8) of its torque-speed curve less the holding torque. On a line that is exact; on an arc the limits are shared out between the directions along the path and towards the centre (3 : 4 : 5), which leaves a part unused. Look-ahead, jerk-limited ramps, junction speeds and transition curves are the shared code. Every planned pose keeps torque, speed and axis jerk within their limits (see "Jerk-limited time-optimal plan" below). Belt tension (max and slack) is a limit of the plan since 13 November (see below). Missing: limits that change within a piece (the classic parameterisation point by point, with a jerk limit); the gearbox and pulley torque ratings, guide loads and DC link are judged afterwards; the payload is as the machine has it when planning | `the_time_optimal_plan_is_shorter_keeps_a_limit_active_and_stays_on_the_torque_speed_curve` in `machine-t-gantry/src/tests.rs` (line, arc, line: 1.429 s against 1.828 s within the axis limits; a limit used to 97 % or more at 64 % of the poses, jerk at 57 % and speed at 10 %; with a 0.6 N m motor and 2000 m/s³ torque at 30 %, speed at 32 %, jerk at 4 %; on the full model with all default effects the motors use 0.18 and 0.09 of the curve), `a_time_optimal_plan_through_a_tight_arc_stays_within_torque_speed_and_jerk`, `time_optimal_plans_of_random_paths_stay_within_torque_speed_and_jerk` in the same file; `a_time_optimal_line_takes_the_hand_calculated_time` in `machine-single-axis/src/tests.rs` (0.714 s = distance / v + v / a + a / j at 1e5 m/s³, 0.861 s = distance / v + 2 sqrt(v / j) at 100 m/s³, within 1.5 ms) |
| D1 trace import and replay | done on the simulator's own traces: a drive trace is a CSV file with a header, the time and per motor position setpoint, position, speed and torque, named like the signals (`time`, `motor.<id>.angle_setpoint`, `.angle`, `.speed`, `.torque`; the setpoint is a new signal), so a file from `sim-cli run` is a trace; other columns are ignored. `sim-cli replay MACHINE --trace FILE` turns the setpoints into tool poses and runs them as a trajectory, prints RMS and peak difference of torque and following error per motor and with `--out` writes measured and simulated torque, speed and following error side by side. Missing: the gripper (a trace does not say when a payload comes and goes, so a cycle trace replays with the payload as the config starts it); the config's start pose has to be set to where the trace starts; a step in the setpoint's acceleration is replayed one cycle late. In the app since 12 November | `a_trace_from_run_replays_onto_itself_and_fits_its_own_config` in `sim-cli/tests/cli.rs`: over one cycle of every machine, torque and following error stay within 0.5 % (RMS) and 5 % (peak) of their largest value |
| D2 test runs on the real axis | needs a real axis | |
| D3 fit in stages | done for two stages, on simulated measurements with noise. Stage 1 (`sim-cli fit MACHINE --trace FILE`): per axis, by least squares, moved mass, viscous friction, Coulomb friction and constant force, with standard errors; motor torques become axis forces through the machine's transformation, so it works where every motor moves every axis. Stage 2 (`--sweep FILE`, a frequency response as `sim-cli sweep` writes it): frequency and damping of the load's resonance on the belt, and the factor on the stiffness that follows. Each value is printed next to what the same estimate gives on the config's own simulation of the same run, and the output says what to change in the config; no config is edited. Cannot be told apart: load mass from rotor and pulley inertia, guide friction from shaft friction, the belt's damping from friction. A wrong friction shows as a few percent of wrong mass and the other way round (gearbox losses); fitting again after the change removes it. The stiffness factor holds once the masses are right and counts every spring between motor and load. Stage 3 (drive delays and filters) is in `sim-cli identify` per motor, with vendor trace import, friction with Stribeck and presliding, and two-mass stiffness from a sweep, an excited trace or a torque step (`crates/sim-components/tests/motor_fit.rs`, `tests/import.rs`, `sim-cli/tests/identify.rs`; synthetic traces and fixtures after vendor layouts, no real drive yet). Not built: stiffness from a move's trace (the loops and friction damp the ringing; a sweep is required) | `the_fit_recovers_changed_masses_friction_and_belt_stiffness_from_noisy_measurements` in `machine-single-axis/src/tests.rs` (1.5 kg more mass found as 1.58 ± 0.03 kg and after a second pass as 1.51 kg, friction factors 1.3 and 1.4 as 1.31 and 1.40 ± 0.02, belt stiffness 0.7 as 0.73 ± 0.02), `the_fit_tells_the_two_axes_apart` in `machine-t-gantry/src/tests.rs`, `resonance_reads_frequency_and_damping_off_a_known_dip` in `sim-components/src/identify.rs` |
| D4 acceptance on a measured cycle | needs a real axis | |

Catalogue data in `catalog/`, each file with the script in `catalog/tools/` that produced it from the vendor document:

| File | Content | Source |
| --- | --- | --- |
| `festo/belt_axes.csv`, `festo/belt_axis_torque_formula.csv` | EXCT-15, -30, -100 T-gantries | Festo catalogue 2023/01 |
| `siemens/gearbox_nrb.csv`, `_nrk`, `_nlc` | 774 planetary gearbox variants for 1FK2 and 1FT2 | Siemens equipment manual 04/2025 |
| `siemens/motors_s210.csv`, `converters_s210.csv` | 96 1FK2 and 1FT2 motors, 24 S210 converters, pairings | Siemens catalogue D 32, 01/2024 |
| `siemens/motors_1ft2_s120.csv` | 37 1FT2 motors on S120, with thermal time constant | Siemens configuration manual 02/2022 |
| `siemens/motors_s120.csv` | 167 1FK7 and 1FT7 motors, with Motor Module pairings | Siemens catalogue D 21.4, 2017 |
| `siemens/motor_modules_s120.csv` | 19 booksize Motor Modules | Siemens Catalog News D 21.4 N, 10/2017 |
| `siemens/motors_1fk2_s120.csv`, `dynamic_braking_1fk2.csv` | 33 1FK2 motors on S120 with thermal time constant; armature short-circuit braking data | Siemens configuration manual 02/2022 |
| `siemens/line_modules_s120.csv`, `braking_s120.csv` | 13 Smart, Active and Basic Line Modules; Braking Module and 4 braking resistors | Siemens catalogue NC 82, 02/2020 |
| `siemens/motors_s120_electrical.csv` | limits and physical constants (torque and voltage constant, resistance, inductance, time constants) of 166 of the 167 1FK7 and 1FT7 (the service has none for the 1FK7022-5AK7); written by `fetch_siemens_datasheets.py`, which keeps the downloaded data sheets and can be rerun | Siemens data sheet service |
| `bahr/belt_axes.csv` | 69 sizes of 15 Bahr belt axis series, among them the X/Z gantry ELZI (a T-gantry) and the XY gantry ELZU | Bahr data sheets 2025/26 on norgren.com |

The tests check the files against their own physics and against each other. They found misprints and gaps in
the vendor documents, which the files keep as printed and name in their headers. Still missing: resistance and
inductance of the 1FK2 and 1FT2, which Siemens publishes in no catalogue, data sheet or manual on its servers
(a support request, the drive's parameters p0350 and p0356, or a measurement would give them); a complete
D 21.4 newer than 2017, which Siemens does not seem to have issued.

The first sensitivity run on the EXCT-30 config showed its velocity loop going unstable when rotor inertia,
belt stiffness or inductance were 20 % off. The gain in the config was lowered from 1.4 to 0.6 N m s/rad.

## Assumptions made for the plan's open questions

- **Reference machine:** a made-up mid-size catalogue-style T-gantry, not a real product. Stroke X 1.0 m,
  Z 0.5 m, pulley radius 20 mm, carriage 4 kg, Z axis 5 kg. All values are placeholders in
  `crates/machine-t-gantry/default.toml`.
- **Payload and cycle time:** payload 2 kg, switchable. No target cycle time; axis limits are X 0.8 m/s,
  8 m/s², Z 0.4 m/s, 6 m/s², jerk 100 m/s³ on both.
- **Motors, gearboxes, drives:** a generic 400 W class servo (rotor 0.3e-4 kg m²) with a 5:1 gearbox.
- **Desktop or browser:** desktop only.
- **PLC link:** the simulator is the OPC UA server; the PLC (or a test client) connects as client.
- **Access to a real axis:** none assumed. Everything that needs measured traces is out of reach.

## Design decisions that differ from or refine the plan

- **Integrator:** classical RK4 with 4 sub-steps per 62.5 µs base step, not a semi-implicit scheme. The plan
  leaves the final choice to the convergence test; RK4 passes it and the energy check, and keeps the core
  free of any knowledge about which states are positions.
- **Registration list:** lives in its own crate `sim-machines`, between the machine crates and the front
  ends. It is the only place outside a machine crate where a machine is named.
- **Motor angles** in the machine interface are motor shaft angles (before the gearbox), because that is
  where the drive's encoder sits. The plan's kinematic formula uses pulley angles; the gear ratio is the
  only difference.
- **Translation files** use the simple-message subset of Fluent (`key = text`, one `{ $arg }` placeholder)
  with a 30-line loader in `sim-core`, not the `fluent` crates. Each crate ships the entries for the keys
  it introduces, so a machine's names stay inside its crate.
- **Energy check:** drift is measured against the energy a torque pulse puts in, not against the
  gravitational potential. That is roughly a thousand times stricter than the plan's wording requires.
- **Belt sections, not spans:** idlers are free, so the tension is uniform between two points that hold
  the belt (a motor pulley or a clamp). The T-gantry has three such sections, each one spring with
  k = EA / L where L changes with x and z. The single spans appear only in the 3D scene.
- **Energy-consistent belt:** the pulleys feel the tension times (1 + strain/2), because stretched belt
  carries its strain energy along. The factor is about 1.0003 and makes the energy check exact. The
  elastic holding torque therefore differs from m_z g r / 2 by less than 0.1 %; the rigid one is exact.
- **Pulley radius holds at pretension,** so x = r/2 (a + b) is exact for an evenly tensioned belt.
- **Guide load for load-dependent friction** uses the X acceleration that the commanded motor torques
  would give a loss-free rigid machine. The true acceleration would make friction depend on itself, and
  its absolute value turned belt vibration into extra friction (found by the rigid-limit check).
- **`effects.belt_stretch = false` selects the rigid reference model.** Belt damping, slack, backlash,
  gear stiffness and the frame mode need the elastic model and have no effect in the rigid one.
- **Belt mass** is lumped on the pulleys (half per pulley on the T-gantry), not spread over the load.
- **Gearbox efficiency** fades out below 0.5 W of transmitted power, so a gearbox at rest has no torque step.
  Motor shaft Coulomb friction is smoothed with tanh for the same reason.
- **SciPy reference:** `crates/machine-t-gantry/reference/scipy_reference.py` re-implements the elastic
  T-gantry and integrates it with LSODA; the trace is committed as `scipy_trace.csv`, so `cargo test`
  needs no Python. Regenerate it when the model or the default config changes (venv in `.venv`, pinned
  in `reference/requirements.txt`). Agreement is within 0.2 % of the travel.
- **Placeholder mechanics:** belt EA 5e5 N, pretension 300 N, gearbox backlash 3 arcmin, friction and
  damping values are estimates. See the two `default.toml` files.
- **Drive placeholders:** Kv 50 1/s, velocity loop Kp 0.08 N m s/rad with Tn 10 ms, current loop tuned
  to 0.25 ms (see below), peak torque 3.8 N m, full torque up to 314 rad/s falling to zero at 420 rad/s, 20-bit
  encoder. Both motors use the same drive settings. Untuned for any real machine.
- **Feedforward model:** torque feedforward is the rigid inverse dynamics the machine supplies (inertia
  and gravity). Friction is not in it, so some following error stays while accelerating.
- **Stop** finishes the segment under way and then stays; there is no fast stop ramp.
- **Cycle** is a list of segments in the machine's config (target, dwell, gripper state on arrival);
  the parameter panel and `--set` cannot edit it, a config file can.
- **External setpoints** (for the PLC link) are interpolated linearly over `profile.setpoint_interval_s`.
- **App versions:** Bevy 0.20.1, bevy_egui 0.43.1 (egui 0.36) and egui_plot 0.37.0, the plot release
  that shares egui 0.36. Desktop only.
- **App structure:** `sim-app` is a library (simulation thread, interface model; no drawing, fully
  tested) plus the window (`main.rs`, `ui.rs`). The simulation thread sends samples through a channel;
  the interface keeps the last 10 s as the scope history.
- **"Within one cycle"** is read as one pass of the pick-and-place cycle. Effects that act all the time
  are also checked within one position-loop cycle (1 ms).
- **Scopes:** one plot per quantity (position, following error, torque, ...) with one line per axis or
  motor. Belt stretch is shown as a colour shift of the belt spans (red above pretension, blue below).
- **The controller's setpoint starts at the motor position,** a few micrometres from the configured
  start pose (belt stretch and gear twist under load), so the first step has no jolt.
- **Speed setpoints are interpolated inside the drive** between profile cycles. Holding them for 1 ms
  made the torque a sawtooth and the peak torque 14 % too high (found by the hand calculation of phase 5).
- **Analysis** measures the second pass of the cycle. Utilisation is torque over the torque the
  torque-speed curve leaves at that speed. The frequency sweep is the open-loop mechanical response
  from one motor's torque to its own speed, on top of the holding torques. The CLI writes it as CSV;
  the app shows the cycle analysis with verdicts, the sensitivity ranking and the optimiser (see "App analysis" below).
- **PLC link:** `sim-opcua` is an OPC UA server (async-opcua 0.19.0 on tokio 1.54.1), endpoint without
  security, anonymous access, default port 4840. Variables are string node ids in the namespace
  `urn:kinematics-simulation`: `signal.<id>` for every signal, `command.target.<axis>`, `command.move`,
  `command.setpoint.<axis>`, `command.cycle`, `command.disable`, `command.enable`, `command.path`, `command.path_time_optimal`, `command.run_path`, `status.busy`, `status.refused`. Signals are published
  100 times a second. No security profiles, no user accounts, no methods, no config changes over OPC UA.
- **Phase 0 leftovers:** the phase 0 commit had no translation files, no dependency check and no window.
  The first two are added in phase 1; the window comes with the app in phase 4.

## Added after phase 6 (on request)

| Addition | Tests |
| --- | --- |
| R-L winding with PI current controller, back-EMF and bus voltage limit, replacing the first-order lag | `current_follows_its_setpoint_with_the_tuned_time_constant`, `back_emf_costs_voltage_and_the_bus_limits_torque_at_speed` in `sim-components/src/drive.rs` |
| Corner blending in the cycle, as an option (`effects.corner_blending`, off by default) | `corner_blending_shortens_the_cycle_and_still_reaches_every_dwell`, `blended_default_cycles_stay_within_the_axis_limits` in `sim-machines/tests/control.rs` |
| Tuning help: step response and closed-loop frequency response, in the CLI (`step`, `bode`, `tuning`) and in the app | `step_and_frequency_response_show_the_position_loop_gain`, `too_much_gain_shows_as_overshoot_in_the_step_response` in `machine-single-axis/src/tests.rs`; `tuning_measures_the_axis_with_the_current_gains` in `sim-app/tests/app.rs` |
| 3D view: pulleys, motor shafts and idlers turn, belt teeth travel around the pulleys, live labels with motor speed in rpm (U/min) and axis position, floor and shadows | `the_scene_shows_pulleys_turning_teeth_travelling_and_motor_speeds` in `sim-machines/tests/suite.rs`; `belt_teeth_keep_their_place_on_the_belt_when_the_path_changes` in `sim-core/src/scene.rs` |
| Scopes: setpoints dashed in the same plot as the actual value, one colour per axis or motor, shared time axis and cursor, time window up to 60 s, freeze, min/max/span/RMS per trace, automatic mm and µm, CSV export | `scopes_have_a_window_freeze_statistics_and_export` in `sim-app/tests/app.rs` |

- **Winding placeholders:** torque constant 0.45 N m/A, 4 Ohm, 10 mH, 190 V at the winding at most (a DC
  link of 155.13 V, `drive.bus_voltage_v`, times sqrt(3/2)). With
  Kp = L / T = 40 V/A and Tn = L / R = 2.5 ms the current follows its setpoint with T = 0.25 ms, as the
  old lag did. The drive adds the expected back-EMF to its voltage (as real drives do); the
  torque-speed curve of the drive settings is kept as a separate limit.
- **Corner blending** starts the next cycle segment while the current one is still braking and adds the
  two. It applies only where the machine need not stop (no dwell, no gripper action), where no axis
  reverses, and only on the ramps. On lift-traverse-lower cycles every axis stays within its limits;
  for other shapes of cycle the sum of two segments on one axis is not checked against the limits.
- **Tuning measurements** put the test signal on the position setpoint only, without feedforward, so
  they show the feedback loops. The step is 1 mm, so with friction on it also shows stiction.
- **Belt teeth in the 3D view** are drawn every 40 mm for visibility, not at the 5 mm tooth pitch.
- **App analysis:** three buttons above the tuning help start the cycle analysis with verdicts, the
  sensitivity ranking and the cycle optimiser. Each runs like the tuning measurement: in a thread of
  its own, on simulations of its own, with the config as it is at that moment; the simulation thread
  and the 3D view run on, and the model picks the result up when it polls. Several can run at once.
  The model hands the window finished tables in the chosen language (`cycle_tables`,
  `sensitivity_table`, `optimised_table`): cycle time, path and dwell error and lowest belt tension;
  per motor peak and RMS torque, winding rise, energy drawn and fed back; one row per check with
  motor, value, limit and result. The ranking varies the values the config gives `[bounds]` and shows
  the eight strongest. "Apply" puts the optimiser's settings into the config and the running
  simulation. A result describes the config it was asked for; it is not redone when a parameter
  changes afterwards. The ranking itself (`Sensitivity::changes`, `strength`) moved from the CLI into
  `sim-components/src/analysis.rs`, so both front ends rank alike.

## Review fixes (11 November 2026)

An independent review confirmed eight defects by experiment and raised further ones. All are fixed; each
has a test that fails without the fix. `cargo test --workspace`: 185 tests.

| Fix | What changed | Test |
| --- | --- | --- |
| 1 profile cycle changed on the way | The controller counts its own profile cycles instead of deriving them from the base step, and when `profile.cycle_s` changes it converts every stored cycle count of the motion under way and reads the shaper's stored plan off at the new spacing. The change is taken at any time (not refused during a motion), so the app's parameter panel stays usable while the cycle runs. Before: X setpoint jumped 0.44 m, a debug build panicked | `changing_the_profile_cycle_on_the_way_does_not_move_the_setpoint` in `sim-machines/tests/control.rs` |
| 2 `motor-peak` on the torque limit | Utilisation is the velocity loop's output before the torque limit (`Drive::torque_demand`) over the curve, no longer the delivered torque, which is clamped to the curve. Single axis with 120 kg payload: 0.97 and `ok` before, 5.6 and `FAIL` now. Search, optimiser and app judge by the same verdict | `a_cycle_on_the_torque_limit_fails_the_peak_verdict` in `machine-single-axis/src/tests.rs`; `a_cycle_beyond_the_torque_speed_curve_shows_as_utilisation_above_one` still holds (limit off: demand is what is delivered) |
| 3 saturation and the current setpoint | The current setpoint is the inverse of `torque_of`, limited to the maximum current; `Drive::at_rest` likewise | `the_maximum_torque_takes_the_maximum_current` in `sim-components/src/drive.rs` |
| 4 holding brake | A spring up to the brake torque (1 mrad), critically damped, with the torque clamped to the brake torque; beyond it the brake slips and forgets the excess deflection within 50 us. No LuGre contact any more. Stable at any speed at the configured step | `a_brake_that_engages_at_speed_stops_the_axis_with_its_torque_and_no_more` in `machine-t-gantry/src/tests.rs` (500 N at the belt stop 15.25 kg from 0.8 m/s over 9.8 mm, within 5 %) |
| 5 gearbox inertia lost in the search | The gearbox's inertia has its own key, `gearbox.inertia_at_input_kgm2` (0 in the three configs), which the machines add to the rotor's. A motor put in later no longer wipes it out, and the gearbox is put in before the motor. `--gearbox NAME` and `--gearboxes SERIES` give the same candidate (0.5905 N m for the 1FK2104-5AF behind an NRK070-04-5; 0.5581 and 0.5873 before) | `a_gearbox_named_before_the_search_keeps_its_inertia` in `sim-machines/tests/catalog.rs` |
| 6 optimiser and input shaping | See C5 above | `the_optimiser_switches_input_shaping_on_where_a_faster_cycle_fails_settling` |
| 7 tool position without the frame | The T-gantry's reported X position and speed are the tool tip over the floor: carriage on the beam + frame deflection + lean of the Z axis. Path error, tracking error, `settling`, step response and closed-loop response see the frame's sway. Both default cycles and the EXCT-30's still pass `settling` (45.9 um and 30.5 um against 100 um), so no tolerance and no cycle was changed. With `--bands` the EXCT-30 now fails `settling` in the worst case (0.16 mm at a bound of the frame estimates) | `the_tool_position_includes_the_frame_deflection` in `machine-t-gantry/src/tests.rs` |
| 8 winding voltage limit | The winding gets sqrt(3/2) times the DC link voltage (`WINDING_PER_DC_LINK` in `drive.rs`, with the reason), also in `apply_motor`'s no-load speed. `drive.bus_voltage_v` is the DC link voltage in every config. The default configs were written with 190 V at the winding: their bus is now 155.13 V, the braking threshold 224.65 V and the trip level 257.6 V, which keeps the voltage limit and the 6.2 J of DC link room as they were. The EXCT-30 (540 V) has 661 V at the winding and a no-load speed of 518 rad/s (423 before) | `every_catalogue_motor_reaches_its_rated_point_inside_the_voltage_limit` in `sim-machines/tests/catalog.rs` |
| a dwell with input shaping | A segment with a dwell or a gripper action has arrived when the shaped setpoint has, one period of the shaper after the plan; dwell, process force and gripper start then. Segments without either go on with the plan | `with_input_shaping_a_dwell_starts_when_the_setpoint_has_arrived` in `sim-machines/tests/control.rs` |
| b rated current | New key `motor.rated_current_a` in all configs; a motor by name sets the catalogue's own rated current (at most 1.05 times the stall current, which catches the one misprinted row). `drive.rated_torque_nm` no longer enters the winding's heating. EXCT-30: winding rise 10.8 K to 9.9 K | `a_motor_by_name_gives_the_values_of_the_hand_written_config` in `sim-machines/tests/catalog.rs` |
| c back-EMF under saturation | The back-EMF constant falls with the torque constant, so electrical power is copper loss plus shaft power with saturation on | `under_saturation_the_electrical_power_is_copper_loss_plus_shaft_power` in `sim-components/src/drive.rs`; the energy balance of `reference_cycle_torque_matches_hand_calculation` runs with saturation on and holds |
| d cycle of one segment | The controller counts the passes of the cycle (`Controller::cycle_passes`); the analysis no longer waits for the segment index to change | `a_cycle_of_one_segment_is_analysed` in `sim-machines/tests/control.rs` |
| e digits of `verdicts` | Four significant digits without exponent for every value, limit and band | read off the output: `settling,,0.00004588,0.0001000,ok` |
| f model switched at speed | `Machine::adopt_state` carries tool position and speed over (new trait method `set_tool_velocity`), and the encoders' last readings move along with the motor angles, so the step in belt stretch does not read as speed | `switching_the_model_at_speed_carries_the_motion_over` in `sim-machines/tests/control.rs` |
| g DC link ratings and the bus | `apply_motor` scales `ratings.braking_threshold_v` and `ratings.dc_link_overvoltage_v` with the bus voltage; `verdicts` refuses a threshold at or below the bus. `effects.dc_link` is on in the default configs | `the_search_on_a_default_config_fails_motors_for_physical_reasons` in `sim-machines/tests/catalog.rs` |
| h retune for a named part | `catalog::resolve` (now with the machine entry) retunes the velocity gain by the search's rule wherever a part is named, relative to the config's own numbers before the part goes in. 1FK2204-6AF on the EXCT-30: winding rise 4634 K before, 17 K now | `the_search_retunes_the_velocity_loop_for_a_lighter_rotor`, `parts_named_in_a_config_behave_like_their_numbers` in `sim-machines/tests/catalog.rs` |
| i tightest check | Leaves out `dc-link-overvoltage` and, with a braking resistor in use, `dc-link-energy` while they pass | `the_tightest_check_leaves_out_a_dc_link_that_a_resistor_backs` in `sim-machines/tests/catalog.rs` |
| j SciPy script | Asserts the fourteen effects it models on and the six machine effects it does not model off; adds the gearbox's inertia to the rotor's. Run with the pinned packages: reproduces the committed trace exactly | `reference_move_matches_scipy` (unchanged trace) |
| k note on resistance and inductance | The header of `catalog/siemens/motors_s210.csv` and the script that writes it say that they are published nowhere | none (a comment) |
| l gearbox verdicts | Comment and documents corrected: motor torque times ratio is on the safe side only while the motor drives; driven back, the output torque is higher by 1 / efficiency. The verdict itself is unchanged: the machines do not report the gearbox torque as a signal | none (a comment) |

Not done then: the gearbox torque as a signal (item l). Done since: see "Seven gaps closed" below.

## Second review fixes (11 November 2026)

A second independent review confirmed twelve defects by experiment and raised five suspicions; all five
were real. All are fixed, each with a test of the review's scenario that was seen to fail with the fix
taken out. `cargo test --workspace`: 211 tests. The tests are in
[`review2.rs`](crates/sim-machines/tests/review2.rs) unless another file is named.

| Fix | What changed | Test |
| --- | --- | --- |
| 1 config values read unchecked | `Config::f64` and `f64s` refuse NaN and infinity; a new `Config::non_negative`; every read in the components and both machines is positive, non-negative, within its range or free in sign (forces, the incline, the Z drops, cogging), see the rules in `docs/config-reference.md`. `drive.no_load_speed_rad_s` has to be above the rated speed, a segment's `dwell_s` finite and not negative, and `start.pose` within the axis ranges, for `Sim::new`, `reconfigure` and the frequency sweep. In the app a simulation thread or a background job that ends without an answer gives a message, once | `bad_config_values_are_refused_with_their_key`, `no_single_bad_number_panics_the_simulation_or_fills_it_with_nan` (every numeric value of both default configs at -1, 0, NaN and infinity), `numbers_that_are_not_finite_or_negative_are_refused_with_their_key` (core), `a_simulation_thread_that_ends_is_reported_once`, `a_job_that_ends_without_an_answer_is_reported` (app model) |
| 2 shaper changed during a motion | A changed shaper setting waits (`Controller::pending_shaper`) until the plan stands still and the shaper's store holds only that pose. A cycle without dwell and gripper action never rests: there it waits for Stop | `a_shaper_changed_at_speed_does_not_make_the_setpoint_jump`: the largest setpoint step stays 0.8 mm |
| 3 reconfigure that changes the kinematics | `Machine::adopt_state` rebuilds the state from the tool's pose and speed whenever the new machine's inverse of that pose differs from the old one's; the position setpoints at the drives and on the fieldbus move along with the encoders (`Controller::motors_shifted`). The rebuilt belt is the one at rest: at speed, the friction it carried (16 N per section on the gantry) is taken up anew | `a_changed_ratio_or_pulley_radius_does_not_tear_the_machine`: belt tension within 10 % of the pretension and the tool within 0.1 mm of a run left alone, at standstill and at speed, with the fieldbus delay on |
| 4 stale OPC UA mirrors | `command.target.*` follows the planned position where the PLC has not written it since the last move; `command.setpoint.*` shows `Controller::external_setpoint`. The controller refuses a `Setpoint` further away than the axis travels at its speed limit in one setpoint interval (`error-setpoint-too-far`), and a followed setpoint ends on the value sent to the last bit | `a_write_to_one_axis_leaves_the_others_where_they_are` (plc.rs), `an_external_setpoint_beyond_the_speed_limit_is_refused` |
| 5 trip before the measured pass | `analyse_cycle` adds up from the start and starts anew when the measured pass begins; `CycleLoad::tripped`; `Candidate::tightest` is infinite for a tripped run | `a_cycle_that_trips_before_the_measured_pass_reports_the_run_up_to_the_trip` |
| 6 refused config value stays in the app | The model keeps the last config the simulation took and goes back to it on a refusal (`Event::Configured` answers every reconfigure); a message that repeats the last one is not added | `a_refused_config_value_is_rolled_back_and_reported_once` (app.rs) |
| 7 optimiser's Apply over a changed config | The result remembers the config it was found for; over another one nothing is applied, with the message `ui-analysis-stale` | `the_app_optimises_the_cycle_and_applies_the_settings` (app.rs) |
| 8 replay of three rows | Fewer than four rows are refused with `error-trace-short` | `replay_refuses_a_trace_too_short_to_replay` |
| 9 frequencies at or below zero | `frequency_sweep` and `closed_loop_response` refuse a frequency that is not finite and above zero; the CLI refuses numbers that are not finite | `frequencies_that_are_not_above_zero_are_refused` (cli.rs) |
| 10 CLI options ignored silently | Every `--word` is checked against the options of the command; an option without its value, two motions at once and an unknown `--lang` are refused, exit code 2 | `misspelt_conflicting_and_incomplete_options_are_refused` (cli.rs) |
| 11 `status.refused` overwritten | A refusal stays until a command of the same kind is accepted | `a_refusal_stays_until_a_command_of_its_kind_is_accepted` (plc.rs) |
| 12 `sim-opcua` falls back silently | An unknown machine, an unreadable port or language and any other word end with a message and exit code 2 | `arguments_that_cannot_be_read_are_refused` (main.rs), `bad_arguments_end_with_exit_code_2_and_a_message` (args.rs) |
| a unbounded event queue | `session::Outbox`: the newest 10 000 samples, the newest scene and every answer; the simulation thread never waits for the reader | `a_reader_that_is_behind_costs_samples_and_not_memory`, `a_session_whose_reader_sleeps_still_delivers_the_newest_state` (app session) |
| b analysis thread that panics | With item 1: a job whose thread is gone answers `ui-job-ended`; the tuning's spinner no longer turns for ever | `a_job_that_ends_without_an_answer_is_reported` |
| c `set_machine` joins the thread | Dropping a session tells the thread to end and does not wait for it | `dropping_a_session_does_not_wait_for_its_thread` (app session) |
| d fewer cycle segments while the cycle runs | `apply_config` keeps the cycle within the segments it has, and ends it if none is left | `a_running_cycle_survives_losing_segments` |
| e `[parts]` and unknown keys | A part name that is not text is refused; `load_config` (used by `--config`) refuses a key the machine's default config lacks, except under `[bounds]` and `ratings.pulley_max_torque_nm`, which the EXCT-30 file has and no default | `a_part_that_is_no_name_and_a_key_the_machine_does_not_know_are_refused`, `a_config_file_with_a_key_the_machine_does_not_know_is_refused` (cli.rs) |
| `fit` without a dip | Exit code 2 with the message, after writing what a trace gave | `fit_without_a_resonance_dip_fails` (cli.rs) |
| German texts | Riemenkraft for the force and Trum for a section; Übertemperatur der Wicklung; "Schlaffes Trum (keine Druckkraft im Riemen)"; µF (also in English); Takt for controller sample times; Bemessungsdrehzahl; 1/min; "Größte Bremsenergie in den Zwischenkreis"; "In Bewegung"; "Die zeitlich vorgegebene Bahn" in `error-trajectory*`; the kind of an unknown part in the user's language (`error-config-part-motor`, `-gearbox`). The German manual follows | the label tests |

Changed for users: bad numbers, unknown options and unknown config keys that used to pass are refused; a
shaper changed at speed acts from the next standstill; unwritten OPC UA targets and setpoints read the
planned position, so a move "back to the targets as they were" has to write them; a setpoint that asks
for a jump is refused; `fit` without a dip and `sim-opcua` with bad arguments exit with 2; the search
table shows `inf` for a tripped candidate.

Not done: nothing of the list. Limits of the fixes: a shaper change never takes effect in a cycle that
never rests; the state rebuilt after a changed ratio is the one at rest, not the stretched one of the
motion; a PLC whose setpoints fall further behind than one interval at full speed is refused until it
catches up with a move; a config value is only as checked as its key (a rating is not).

## The app does what the command line does, and arcs entered at speed (12 November 2026)

`cargo test --workspace`: 220 tests (211 before). New tests are in
[`app.rs`](crates/sim-app/tests/app.rs) unless another file is named.

| Addition | What it is | Test |
| --- | --- | --- |
| Error bands in the app | A tick box beside "Analyse and judge the cycle": the job also runs `verdict_bands` over the config's `[bounds]`, and a fourth table shows per check the lowest and highest value and the worst-case result | `the_checks_get_error_bands_in_the_app` (every row against `verdict_bands`, both machines) |
| Motor search in the app | Supply, start of the article number and an optional gearbox series; the table as `search_motors` ranks it, with the tightest check or the reason a motor was not run; "Use" in a row names motor, supply and gearbox in `[parts]` of the running config. A supply other than 230v and 400v and a search without a motor are refused with a message | `the_app_searches_motors_and_names_one_in_the_config` (five 1FK2104 on both machines, a 2000 r/min motor ruled out by speed, NRK070 gearboxes on the first machine) |
| Paths in the app | A typed file name; lines and arcs (within the axis limits or as fast as the motors allow) or poses at times. The job plans from where the machine stands, analyses on a simulation of its own, and when the answer is back the machine in the app runs the path. Tables as for the cycle, with duration, deviation by transition curves and distance behind the setpoint; the planned path as a plot | `a_path_is_planned_run_and_analysed_in_the_app` (against `plan_path` and `analyse_trajectory`; the machine arrives; poses at times back to the start; missing file, unreadable line, path that starts elsewhere; with a tolerance of 0.1 mm the table shows 0.100 mm and a shorter duration) |
| Trace replay and fit in the app | A typed file name; per motor the difference between trace and replay, per axis the four fitted values next to the config's, and the advice lines. The advice texts moved from the CLI's translations to the components' (`fit-mass` and so on, `AxisFit::advice`), so both front ends say the same | `a_trace_is_replayed_and_fitted_in_the_app` (a trace of one cycle of each machine: torque within 0.5 % RMS and 5 % peak, masses and Coulomb friction within 2 %; missing file, no trace, three rows) |
| The extra analysis figures | The cycle table has slack time, radial load, guide load and guide life; the motor table cubic-mean torque, the braking resistor's energy and mean power and the highest DC link voltage. The tables are turned: one row per figure, one column per motor | `the_cycle_is_analysed_and_judged_in_the_app` |
| A trip is said | The simulation thread reports a drive that trips, once per trip (`ui-drive-tripped`); an analysed cycle or path that trips adds `ui-analysis-tripped` | `a_drive_that_trips_is_reported` (4.7 uF and a 5 W resistor on both machines) |
| Results and the config | Every result remembers the config it was computed for (`Model::outdated`); the window marks it once the config has changed, and "Apply", "Use" and the run of a planned path refuse with `ui-analysis-stale`. A search is not outdated by the part names it sets itself | in each of the tests above |
| Right panel | Scrolls as a whole; Scopes, Analysis, Motor search, Path, Trace and Tuning fold away and open when their result arrives; long names wrap, numbers have four digits, the default width is 720 px; the 3D camera is fitted once the panels are laid out. The screenshot hook runs the cycle analysis with error bands, the ranking, the optimiser and the tuning; with `SIM_APP_SEARCH=<start of the article numbers>`, `SIM_APP_PATH=<file>` or `SIM_APP_TRACE=<file>` it runs that part instead, so that it is in the picture | screenshots (see "Blocked or not verifiable here") |
| Arcs entered at speed | `profile.path_tolerance_m` (0 in all three configs: the path as given). Above 0 the path gets transition curves (clothoids) where a line and an arc or two arcs meet in the same direction, in `geometry` of [`path.rs`](crates/sim-components/src/path.rs), which both planners time. An arc of radius r keeps its centre and moves in by e = L² / (24 r) for curves of length L; one L for the whole path, L = min(v³ dk / (0.4 j) of the fastest junction, sqrt(24 · tolerance · r_min)); each curve is then fitted exactly. `path::deviation`, `Sim::path_deviation`, a line on the CLI's error stream and a row in the app's path table report the largest e | `transition_curves_let_the_path_enter_an_arc_at_speed` in `path.rs`: line, arc of 0.1 m, line with 0.1 mm: 1.590 s against 1.828 s, into the arc at 0.3955 m/s (0.3957 by hand) against 0.0707 m/s, curves 15.49 mm, every pose within 0.1000 mm of the given path, axis rates within 0.1 % of their limits; with 1 mm: curves 15.99 mm (16 by hand), off by 0.10667 mm (0.1067 by hand), 1.589 s. `transition_curves_join_two_arcs`: an S of two arcs 1.539 s against 1.847 s, two arcs turning the same way 1.481 s against 1.824 s, no step in position, direction or curvature. `both_planners_time_the_path_with_its_transition_curves` in `machine-t-gantry/src/tests.rs`; `a_path_tolerance_shortens_a_rounded_path_and_the_deviation_is_reported` in `sim-cli/tests/cli.rs` |

Changed for users: `sim-cli search`-style tables in the app show the tightest check only; the app's numbers
have four digits (1912, not 1912.141); the right panel is 720 px wide by default; `cli-fit-*` translation
keys are `fit-*`.

Limits: see "Not built" for the transition curves and for what the app still lacks. A path in the app is
planned from where the machine stands when the button is pressed, a few micrometres from the configured start
pose, so its figures differ from the command line's in the last digits.

## Third review fixes (12 November 2026)

A third independent review went through the newest code: the transition curves, the app's analysis panel and
the second fix round. Five defects were confirmed by experiment and are fixed, each with a test of the
review's scenario that was seen to fail with the fix taken out. `cargo test --workspace`: 224 tests (220
before). The tests are in [`review3.rs`](crates/sim-machines/tests/review3.rs) unless another file is named.

| Fix | What changed | Test |
| --- | --- | --- |
| 1 transition curves too long for a tight arc | What two curves leave of an arc was taken as the angle between them. Curves that together turn by more than the arc and the rest of its circle read as a short arc again: the planned path went round once (or twice) more than the given one, and was further from it than reported (2.4 mm against 0.9 mm reported, in 3 of 3000 random paths, all with a tolerance near the radius of a tight arc). Now what each curve takes is counted from the arc's own end, and a curve that turns half a circle or more is refused; the length is halved as for a piece that is too short. 16 000 random paths of lines and arcs then come out with no step in position, direction or curvature, the turning of the given path, the reported deviation and every axis within its limits | `curves_too_long_for_a_tight_arc_do_not_add_a_turn` in `sim-components/src/path.rs` (turning by 7.63 rad for 1.35 rad) |
| 2 a planned pose beyond a stroke by rounding | A line that ends on the end of a stroke came out a few 1e-17 m beyond it in 39 of 4000 cases, and the controller refused the whole path (`error-trajectory`). Both planners set a pose that is less than a nanometre outside an axis's range onto its end (`path::within`) | `a_path_onto_the_end_of_a_stroke_stays_within_the_range` in `path.rs` |
| 3 a path from an end of a stroke | The setpoint of a machine at rest is where its motors stand, up to 44 um from the tool pose on the gantry; on an end of a stroke that is beyond it. A path planned from there (the command line with such a start pose; the app after a move to an end of a stroke) was refused, a point-to-point move from the same pose ran. `Command::Trajectory` takes poses up to 0.1 mm beyond the travel range, the margin it already gave the start of a trajectory (`SETPOINT_SLACK`) | `a_path_from_an_end_of_a_stroke_is_run`; `a_path_is_planned_run_and_analysed_in_the_app` (app.rs) |
| 4 a stream of setpoints near the speed limit | Three things. The way to an external setpoint was counted from the profile cycle after it came, so it ended a cycle late; in a stream of one setpoint per interval the plan stood still for a cycle at every setpoint and then ran a ninth faster than the stream (0.889 m/s for 0.8 m/s). The speed guard of the second review measured from where the plan stood, a tenth short of the last setpoint, and so refused every setpoint but the first from 10/11 of the speed limit on (39 of 40). Now the way starts with the cycle the setpoint came in, the guard measures from the last setpoint (`Controller::external_setpoint`), and the speed setpoint stays through the cycle of arrival (zero there would be a dip in every interval of a stream) | `a_stream_of_setpoints_at_the_speed_limit_is_followed` |
| 5 search with a gearbox series the catalogue lacks | The app said that the catalogue had no motor whose article number starts as typed, which it had. New message `ui-search-no-gearbox` with the series | `the_app_searches_motors_and_names_one_in_the_config` (app.rs) |
| documents | The user guides and the model reference said that the time-optimal plan passes the torque reserve "slightly" on an arc. On an arc that takes less time than the smoothing it does so many times over (utilisation 10.5 on a quarter turn of 5 mm radius, with or without transition curves); they now say so. Added: poses up to 0.1 mm beyond the range, the setpoint guard and the interpolation as they are now, curves that are too long. Every command of README.md and docs/user-guide.md was run again: all quoted output is as printed | none (documents) |

Changed for users: a path from an end of a stroke runs; a trajectory may leave the travel range by up to
0.1 mm; an external setpoint arrives one profile cycle earlier than before (its interval after the cycle
in which it came) and a stream at the speed limit is followed; a path with a tolerance near the radius of
one of its arcs gets shorter curves than before, or none.

Found clean: the geometry of the transition curves (end point, direction and curvature at every junction to
1e-9 m, 1e-7 and 1e-6 relative; the arc's radius and what is left of it; the path's end), the axis rates of
planned paths by differences (within 0.1 % of the limits on tight and wide radii, short pieces, arcs of more
than a half turn, full circles, S-curves, closed paths, arcs at corners), the time-optimal plan on blended
paths (no more torque than on the given ones); in the app the stale-result handling of "Use", "Apply" and
the planned path, the answers to missing files, directories, unreadable lines and empty files, the trip
message, the bounded outbox; of the second round the config checks (all three configs load, zero damping,
payload, backlash and pretension and an efficiency of 1 are taken), the unknown-key check (a missing key is
named), the pending shaper, `adopt_state` (a parameter edit that changes no kinematics keeps the state to
the last bit), the option checking of the command line.

Not done, and why:

- The time-optimal plan on arcs shorter than its smoothing (see "documents"): it needs a jerk-limited
  time-optimal plan in place of the moving mean. The analysis shows the overload, and `motor-peak` fails.
  (Done since: see "Jerk-limited time-optimal plan" below.)
- A Coulomb or stiction level of zero is refused (the friction model divides by the level); the effect
  switch `effects.guide_friction` is the way to no friction. `docs/config-reference.md` says so.
- Away from the ends of the strokes the last pose of a planned path can differ from the file's number in
  the last bit (1e-16 m).
- A path asked for in the app while the machine moves is planned from the pose it was passing, analysed and
  shown; the machine refuses to run it, with a message.
- A machine started on an end of a stroke shows its setpoint beyond the range to a PLC, as the target of an
  axis the PLC has not written. No shipped config starts there.

## Jerk-limited time-optimal plan (12 November 2026)

The defect the third review left open. The time-optimal plan had no jerk limit and was smoothed by a moving
mean of the distance along the path over 40 ms, which carried the speed of the lines through any arc shorter
than that: utilisation 10.5 on a quarter turn of 5 mm radius. `cargo test --workspace`: 226 tests (224 before).

What changed, in [`path.rs`](crates/sim-components/src/path.rs): `plan_time_optimal` no longer has a
planner of its own. It works out a speed and an acceleration limit per piece from the motors (rated speed;
`profile.torque_reserve` times the torque-speed curve less the holding torque; both from the rigid model at
nine poses of the piece) and hands them to the timing of the plan within the axis limits (`timed`), with the
jerk limits of the axes. `limits` and `curved` take their bounds as rows (a direction in axis space and the
most a rate may be along it), which an axis limit and a motor limit both are. The grid of points, the
forward and backward pass and the smoothing are gone, and with them `profile.path_smoothing_s` (three
configs, both languages, the documents).

| Test | What it shows |
| --- | --- |
| `a_time_optimal_plan_through_a_tight_arc_stays_within_torque_speed_and_jerk` in `machine-t-gantry/src/tests.rs` | The review's line, quarter turn of 5 mm, line, with a tolerance of 0, 0.1 mm and 1 mm: torque of the rigid model, motor speed and axis jerk of every planned pose, by differences over one cycle, within the reserve, the rated speed and the jerk limits (margin 0.1 %); ends on the end point; utilisation on the full model 0.18. Seen to fail before the change: torque 2.5 times its share, speed 1.13 times the rated speed, jerk 1494 times the limit |
| `time_optimal_plans_of_random_paths_stay_within_torque_speed_and_jerk`, the same file | 200 paths of two to five tangent lines and arcs in the workspace (radius 2 mm to 300 mm, pieces 1 mm to 400 mm, `Noise(3)`), half with a tolerance of 0.01 mm to 1 mm, half with a 0.6 N m motor and 2000 m/s³ so that the torque limits: the same three checks and the end point. Most used: torque 1 + 1.6e-11, speed 1 + 2.5e-13, jerk 1 + 8.4e-9. Seen to fail before the change on its first path (torque 1.34, jerk 494). 6000 paths with another seed, run once, gave margins of the same size |
| `the_time_optimal_plan_is_shorter_keeps_a_limit_active_and_stays_on_the_torque_speed_curve` | Rewritten: see C3 above |
| `a_time_optimal_line_takes_the_hand_calculated_time` in `machine-single-axis/src/tests.rs` | Two hand calculations in place of one, see C3 above |

Changed for users: `--time-optimal` is slower and keeps its promise. Line, arc of 0.1 m, line on the default
T-gantry: 1.429 s (0.689 s before) against 1.828 s within the axis limits; with a tolerance of 0.1 mm 1.199 s
against 1.590 s; analysed with its 0.3 s of rest 1.729 s (0.989 s), path error 0.19 mm (0.71 mm), no slack
belt (13 ms). A config file with `profile.path_smoothing_s` is refused as a key the machine does not know.
`both_planners_time_the_path_with_its_transition_curves` asks for 0.8 of the time where it asked for half,
and the app's path test for less time where it asked for 0.9 (two lines in a row: 1.381 s against 1.473 s).

Left open: the plan is not the fastest one inside its limits. One set of limits per piece, and on arcs the
3 : 4 : 5 shares leave torque, speed and jerk partly unused (at 36 % of the poses of the test path no limit
is within 3 %). With the default jerk limit of 100 m/s³ the motors' torque is hardly used at all (0.16 of
the reserve): the jerk limit decides, as it has to. The length of the transition curves is still worked out
with the speeds of the axis limits, so that both planners time the same path. The last pose of a line can
differ from the file's number in the last bit, as with the other planner.

## Seven gaps closed (12 November 2026)

`cargo test --workspace`: 239 tests (226 before).

| Gap | What was built | Tests and their numbers |
| --- | --- | --- |
| 1 gearbox verdicts | Both machines report the torque at each gearbox output, signal `gearbox.<motor>.output_torque`: in the elastic model what the gear's spring passes on to the pulley (efficiency applied), in the rigid one the value its solver works out, now brought out. The analysis looks at it every step (`MotorLoad::gearbox_torque`: peak and cubic mean weighted with speed), and `pulley-torque`, `gearbox-peak` and `gearbox-continuous` judge it. A machine without the signal does not get these three checks | `the_gearbox_verdicts_judge_the_torque_at_the_gearbox_output` in `machine-single-axis/src/tests.rs` (upright axis, efficiency 0.9: 0.7845 N m at the output lifting and lowering, where the motor torque times the ratio reads 0.8717 and 0.7061; elastic, lowering 6 kg: 1.1768 N m within 1 %; cycle peak 1.642 N m against 1.6345 by hand, motor torque times ratio 2.203 against 2.191); `gearbox_torque_is_the_load_at_the_pulleys_whichever_way_it_is_driven` in `machine-t-gantry/src/tests.rs` (+0.6865 and -0.6865 N m); `verdicts_match_the_hand_calculation_and_fail_where_a_part_is_too_small` with the new hand values |
| 2 bands with interactions | `analysis::verdict_bands_sampled(entry, cfg, bounds, samples, seed)`: a Latin hypercube over all bounded values from the `Noise` generator (new `Noise::uniform`); `Band::share`, the share of runs a check passes. `sim-cli verdicts --bands --samples N [--seed S]` with a ninth column `pass_share`; in the app a number field beside the tick box (`Inputs::samples`, seed 1) and a column "Runs passed [%]". The runs of both modes share the processor cores (61 values one at a time on the EXCT-30: 2.4 s; 200 samples: 3.7 s, debug build, 16 cores) | `sampled_bands_find_the_two_masses_that_fail_only_together` in `machine-single-axis/src/tests.rs` (each mass alone 1.26 to 1.66 N m against 1.75 allowed, passes; sampled 1.26 to 2.06 N m, fails, 70.7 % of 40 runs pass against 72 % by hand; seed 2: 68.3 %); `verdicts_judge_the_default_cycle_of_every_machine` (cli.rs: columns, same seed same table, five refused calls); `the_checks_get_error_bands_in_the_app` (app.rs) |
| 3 single axis | Cable chain (`effects.cable_chain`, `[cable_chain]`; on an incline the moving run's weight is lifted and is in the energy), process force during dwells (`effects.process_force`, `process.force_x_n`), holding brake (`effects.holding_brake`, `motor.brake_torque_nm`; a ninth elastic state). `CableChain` and `HoldingBrake` moved from the T-gantry into `sim-components/src/mech.rs` unchanged in what they compute; their labels moved to the components' translations. The inclined suite configs run with the chain | `cable_chain_adds_the_hand_calculated_force_at_two_positions` (1.4 N and 3.0 N; coasting; 0.858 N and 1.839 N more to hold on 30 degrees), `process_force_changes_the_holding_torque_by_the_hand_calculated_amount` (0.04 N m), `the_holding_brake_holds_the_upright_axis_when_it_is_strong_enough` (falls 136.4 mm against 136.0 by hand) in `machine-single-axis/src/tests.rs`; `a_run_can_disable_the_drives_on_the_way` (cli.rs) now compares where a machine disabled on the way stops with and without brakes |
| 4 shared DC link | `effects.shared_dc_link` in all three configs, off. On (with `dc_link`): one energy state for all drives of a machine, stepped after their current loops (`drive::shared_dc_link`): capacitance of all drives together, one chopper and resistor, one trip; every drive reports the link's values. Switched on in a running machine, the links' energies are put together | `a_shared_dc_link_rises_by_the_net_energy_of_its_drives` in `sim-components/src/drive.rs` (separate 231.40 V and 190 V; shared 199.558 V as by hand; 0.845 J in the one resistor; joined links 211.72 V); `a_shared_dc_link_is_one_voltage_for_all_drives_of_a_machine` in `sim-machines/tests/control.rs` (default T-gantry: 224.65 V separate, 224.06 V shared). The two effect-flip tests skip the switch on machines with one drive |
| 5 path in the 3D view | `Machine::scene_point(tool pose) -> Option<[f64; 3]>`, `None` by default; both machines implement it and draw their tool part with it. `Scene::tool` is where the tool is now, filled in by `Sim::scene`. The app keeps the planned path as scene points (`PathRun::scene_line`) and the travelled one (`Model::trail`, a point per 0.1 mm from the scenes) and draws both as lines (`draw_paths` in `main.rs`) | `the_scene_point_of_a_tool_pose_is_where_the_scene_draws_the_tool` (T-gantry) and `..._the_carriage` (single axis), five poses, rigid and elastic, 1e-12 m; `a_path_is_planned_run_and_analysed_in_the_app` (app.rs: the line ends where the mapping puts the file's end, the trail runs from its start to within 2 mm of its end and stays within 5 mm of it) |
| 6 splines | Path piece `spline,x,z`; consecutive ones are one cubic spline (chord-length parameter, no step in position, direction or curvature at a via point), with the direction of the line or arc before and after, else no curvature at the end. One piece per span for both planners, with limits from the span's tightest curvature and fastest change of curvature (257 places, plus 5 %) | `a_spline_passes_through_its_via_points_within_the_axis_limits` in `sim-components/src/path.rs` (four paths: via points to 1e-12 m, axis rates within 0.1 % of the limits; a spline alone peaks at 0.40 m/s, 2.8 m/s2, 32 m/s3 of 0.4, 6, 100; a via point straight back is refused); `time_optimal_plans_of_random_paths_stay_within_torque_speed_and_jerk` with splines (most used: torque 1 + 2.5e-11, speed 1 + 2.5e-13, jerk 1 + 8.4e-9) |
| 7 EXCT-30 with margin | Chosen in `exct-30.toml`, with the reasons in its comments: belt pretension 450 N with bounds 350 to 550 N (300, 200 to 400), X jerk limit 150 m/s3 (500), input shaper on at a fixed 25 Hz, corner blending on. Cycle 1.934 s (2.136 s before; 2.434 s without the blending). The shaper that follows the pose did worse (0.079 mm against 0.045 mm), because the frequency the machine reports leaves the frame out | `the_exct_30_config_passes_every_check_in_the_worst_case_of_both_band_modes` in `sim-machines/tests/catalog.rs` (one at a time: slackest section 139.1 N, settling 0.041 mm, shaft load 1205 N; 60 samples: 122.7 N, 0.049 mm, 1206 N; 3.3 s) |

Found on the way and fixed: with the X guide's Coulomb level at its low bound and its bristle stiffness at
its high bound together, the friction state settled faster than the integrator can follow and the friction
force grew without bound (one of 200 samples: pulley torque 9 N m, winding 51 K, belt slack). `Lugre::eval`
caps the settling rate at 1e5 1/s (`lugre_with_stiff_bristles_at_speed_stays_stable_and_slides_at_its_level`;
a `shortcut:`). No shipped config reaches the cap with its own values.

Changed for users: the three gearbox and pulley checks read lower where a motor accelerates its own rotor
and higher where a load drives the gearbox back; one more signal per motor; `Band` has a field more and
the app's band table a column more; the single-axis config has two more tables and three more switches,
and its elastic state one more entry; every config has `effects.shared_dc_link`; `Scene` has a field
`tool`; the EXCT-30 config runs another cycle; a path file may hold `spline` lines.

Limits: the sampled bands do not search for the worst corner. A shared link adds the drives' capacitances
and knows no Line Module. The path lines in the 3D view were not looked at on screen. Festo publishes no
belt tension rating for the EXCT; the highest tension of the retuned config (558 N, 658 N with the belt
set 100 N high) is judged against an estimate (2140 N now, see the end of this file). A spline joins a line or an arc with a step in curvature.

## Fourth review fixes (12 November 2026)

A fourth review found the points below. Each code fix has a test that was seen to fail with the fix taken
out. `cargo test --workspace`: 242 tests (239 before).

| Fix | What changed | Test |
| --- | --- | --- |
| 1 an arc as the first piece | The setpoint of a machine at rest is where its motors stand, on the default gantry 12 and 46 um from the start pose (belt stretch under the weight of the Z axis). The arc already took its radius from there, but its end then missed that circle by 12 um, above the 10 um allowed, and a full turn would have read as an arc of 46 um. Both tolerances are now 0.1 mm (`ARC_SLACK` in `path.rs`), what the controller allows between setpoint and pose. An arc 1 mm off is still refused | `a_path_file_is_read_line_by_line` in `path.rs` |
| 2 a path that leaves the travel range | `plan_path` refuses it with `error-path-range` ("The path leaves the travel range ...") instead of the message for poses at times. The guides say that a spline swings past its via points | `path_error_on_a_circle_matches_the_hand_calculation` in `machine-t-gantry` (its last lines), key in `i18n.rs` |
| 3 the travelled path in the 3D view | It came from the scenes, one point per frame. The simulation thread now reports the tool's place from every step, a point per 0.1 mm (`Event::TrailStart`, `Event::Trail`); the queue keeps one such event of at most 100 000 points. The trail starts when the machine has taken the trajectory (not when it refuses it), ends on a reconfigure and on the start of a cycle, and drops its oldest points at the cap | `the_travelled_path_follows_every_step_and_starts_and_ends_with_the_path`, and the spacing in `a_path_is_planned_run_and_analysed_in_the_app`, both in `sim-app/tests/app.rs` |
| 4 label | "Lines and arcs" is "Lines, arcs and splines" / "Geraden, Kreisbögen und Splines", also where the guides quote it | `every_label_exists_in_every_language` |
| 5 comments in `exct-30.toml` | Pretension: the cycle takes up to 239 N off a section (211 N nominal), measured with the belt fixed at 450 N in 200 sampled runs; 111 N are left with the belt at 350 N, 122 N is the lowest with the pretension sampled too. Shaper: passes at 17.5, 25 and 35 Hz, fails off | none (comments) |
| 6 test counts in `docs/validation.md` | 242, 207 and 35 | none |
| 7 share of runs passed | Cut off instead of rounded (`Band::share_shown`), on the command line and in the app: a share below 1 never shows as 1.000 | `a_pass_share_below_one_never_shows_as_one` in `review3.rs` |
| 8 stiction below Coulomb | The bounds of the EXCT-30's X guide overlap (Coulomb up to 26 N, stiction down to 20 N), and the friction model took a stiction level below the Coulomb level as given. `Lugre::from_config` now takes the larger of the two as the stiction level; the bounds are as they were. The EXCT-30 results did not change: every check passes in both band modes (one at a time: 0.041 mm, 139.1 N; 200 samples: 0.055 mm, 121.6 N, every share 1.000) | `lugre_stiction_is_never_below_coulomb` in `mech.rs` |

## Fifth review fixes (12 November 2026)

A fifth review found the points below. Each code fix has a test that was seen to fail with the fix taken
out. `cargo test --workspace`: 243 tests (242 before).

| Fix | What changed | Test |
| --- | --- | --- |
| 1 a short arc ran as a full circle | "Ends where it starts" was judged with the 0.1 mm that only the machine's rest setpoint needs, so every arc with a chord below 0.1 mm went round once, in both planners (a path of 80 um went down to z = 0.15). `given` in `path.rs` now keeps the end the file stated last: an arc is a full turn when its end is that point (to 1e-9 m), and its angle is taken between the two stated points. Only the first piece, which has no point before it, is still judged against where the machine stands, with `ARC_SLACK`: there an arc ending within 0.1 mm is a full turn, and a file that means a shorter arc states its start with a `line` first. Both guides say so | `a_short_arc_is_not_a_full_turn_and_an_arc_off_its_end_is_reported` in `path.rs` |
| 2 an arc off its stated end, unsaid | Later arcs are back at 10 um between the circle through the stated end before and the stated end (`ARC_TOLERANCE`); 0.1 mm holds for the first piece only. What an arc ends off its stated end (above 1 nm) is added to the deviation the planner reports, with or without transition curves; the message `path-deviation` names both causes in both languages | the same test, part (c) |
| 3 the travelled path went on after the path | The trail stops growing when the trajectory is through or any other command is taken (`Runner::recording` in `session.rs`); the finished trail stays shown until the next path, cycle or reconfigure. A move after a path added 4963 points to it | `the_travelled_path_follows_every_step_and_starts_and_ends_with_the_path` in `sim-app/tests/app.rs` |
| 4 comment on the pretension in `exct-30.toml` | Rounded figures with the command that gives them (belt held at 450 N by `--set "bounds.belt.pretension_n=[450.0,450.0]"`: about 240 N off, about 110 N left at 350 N), and that seeds move them by a few newtons | none (comment) |
| 5 a belt tension rating for the EXCT-30 | `ratings.belt_max_tension_n = 2160.0`, an estimate: Festo publishes none; Optibelt lists 1800 N at 25 mm for 5 mm pitch steel-cord polyurethane belts, 72 N per mm, at 30 mm width. The config now has 19 checks. `belt-tension`: 558.4 N as written, 458.4 to 658.3 N one at a time, 450.8 to 668.9 N in 200 samples, every share 1.000. (The limit is 990 N since the sixth review. "All other rows as before", which stood here, was not true of the guides' sampled table: four `low` values of motor a there were stale, see the sixth review) | `the_exct_30_config_passes_every_check_in_the_worst_case_of_both_band_modes` in `catalog.rs` |
| 6 test counts in `docs/validation.md` | 243, 208 and 35 | none |

Limits: an arc shorter than 0.1 mm as the very first piece was still run as a full turn (refused since the
sixth review). The trail ends when the setpoint trajectory is through, so what the tool settles after
that is not drawn. Whether Festo's belt has cords of the class the 2160 N assumed is not known (990 N after the sixth review, 2140 N now, see the end of this file).

## Sixth review fixes (12 November 2026)

A sixth review found the points below. Each code fix has a test that was seen to fail with the fix taken
out. `cargo test --workspace`: 243 tests, as before (tests were extended, none added).

| Fix | What changed | Test |
| --- | --- | --- |
| 1 the travelled path stopped at a stop or an enable | Every accepted command other than a path or a cycle ended the trail, but `Command::Stop` and `Command::Enable` are taken while a trajectory runs and leave it running. `Runner::handle` in `session.rs` now asks the controller (`busy()`): the trail ends with a command only if the trajectory does, which is `Command::Disable` | `the_travelled_path_follows_every_step_and_starts_and_ends_with_the_path` in `sim-app/tests/app.rs` |
| 2 "off by up to 0.0000 mm" | An arc with decimally rounded end coordinates is 1 to 50 nm off its circle, which the command line reported with a figure that shows as zero. It says nothing where the figure would show 0.0000 (below 0.05 um). The app shows the figure with three significant digits and is unchanged | `a_path_tolerance_shortens_a_rounded_path_and_the_deviation_is_reported` in `sim-cli/tests/cli.rs` |
| 3 an arc as the first piece | A 130 um arc written from the start pose (0.1, 0.35) ran as a full turn: it ended 85 um from the rest setpoint, which is 47 um off the pose. A full turn (end up to 0.1 mm from the setpoint) cannot be told from a short arc there, so neither is guessed: a first arc that ends less than 1 mm from where the machine stands is refused (`FIRST_ARC` in `path.rs`, message `error-path-first-arc` in both languages), full turns included, and the file states the start with a `line` first. The reviewer's rule kept the full turn within 0.1 mm; that would still have run the 130 um arc as one. Both guides state the rule, and that the offsets of arcs chained around one centre add up and are reported as the deviation (the code is unchanged there: an arc keeps the radius the path has) | `a_short_arc_is_not_a_full_turn_and_an_arc_off_its_end_is_reported` and `a_path_file_is_read_line_by_line` in `path.rs`; `a_circle_timed_by_the_planner_gives_the_hand_calculated_path_error` now states its start |
| 4 quoted verdict tables | The sampled table of the EXCT-30 in both guides had four stale `low` values of motor a (7.303, 0.1610, 22.17, 657.4; the program gives 7.310, 0.1327, 20.14, 658.5). Both tables in both guides and the one in `README.md` are the program's present output | none (documents) |
| 5 belt tension limit of the EXCT-30 | `ratings.belt_max_tension_n` from 2160 to 990 N: Festo does not state the tooth profile, and makers' figures for 5 mm pitch steel-cord belts are about 72 N per mm of width for AT5 and about 33 N per mm for T5; the weaker class at 30 mm. All 19 checks pass in both band modes with every share 1.000. `belt-tension`: 558.4 N as written, 458.4 to 658.3 N one at a time, 450.8 to 668.9 N in 200 samples | `the_exct_30_config_passes_every_check_in_the_worst_case_of_both_band_modes` in `catalog.rs` |

Limits: a path that starts with a full turn now needs a `line` to the start pose before it. The belt makers'
figures are from search summaries of their data sheets; the sheets were not read. An arc still ends beside
its stated end by what its radius is off, and over chained arcs that adds up; it is reported, not prevented.

## Blocked or not verifiable here

- **Phase 4, a person at the 3D view:** the window was started twice with the built-in screenshot hook
  (`SIM_APP_SCREENSHOT=file.png`, optional `SIM_APP_LANG=de`), and both screenshots were inspected: the
  machine, the panels in English and German and the live scopes render correctly. After the rework of
  the 3D view and the scopes three more screenshots were inspected the same way, and two more (English
  and German) with the analysis panel and the Disable and Enable buttons; the hook now also runs the
  three analyses and waits for them. In those two the analysis tables filled the right panel, and at
  the default width the last column of the widest tables was reached by scrolling sideways. On
  12 November the panel was reworked (parts that fold away, tables laid out for the panel's width) and
  eight more screenshots were inspected, English and German of the analysis with error bands, the motor
  search, a path and a trace: every table's last column and every button is inside the panel, and the
  machine fits the 3D view, whose camera is now fitted after the panels are laid out. Seven more, of the frequency
  response with its fit, the operating points and the replay overlay, in both languages, show every new plot,
  table and button inside the panel. Not checked by anyone:
  dragging the orbit camera, the view presets, the parameter and effect panels by mouse, the buttons
  and text fields of the right panel by mouse and keyboard, and how the motion looks. Those need a person at the screen. The screenshot run is not part of `cargo test`,
  because it opens a window.
- **Phase 6, a real PLC:** no controller is available here. The "PLC" in the test is an OPC UA client
  written with the same library as the server, so interoperability with a Siemens, Beckhoff or CODESYS
  client, their data types, security settings and cycle jitter are untested.
- **Real traces (validation table, stage D):** needs measured drive traces. Import, replay and fit (D1, D3) are built and tested on traces the simulator made itself; D2 and D4 wait for a real axis.

## Not built (named in the plan, outside the phase criteria)

- Of the plan's own list nothing is left: bounds are stage B2, parts by name stage B3.
- Not in the app, available on the command line: `--set`, and the operating points of a path. The sensitivity
  ranking in the app covers the values with `[bounds]` only; the CLI can vary every table.
- Not adjusted when a catalogue motor goes into a default config: the DC link capacitance and the braking
  resistor's powers of `[ratings]`, which are placeholders. The braking threshold and the trip level are
  scaled with the DC link voltage, so `effects.dc_link` is on in the default configs too.
- Of stages A to C: a time-optimal plan whose limits change within a piece (see C3), which for a spline means
  within a span. (The worst-corner search was built on 12 November, see the end of this file. Fieldbus jitter, a second shaper resonance and
  brakes in the rigid models were built on 12 November, see the end of this file.)
- Of the splines: one set of limits per span; an eased junction (spline into a line or an arc, see the end of
  this file) is passed more slowly than a clothoid of the same deviation would allow.
- Of the life estimates (see the end of this file): the motors' own bearings, a life of the belt (no maker publishes a method), the fatigue of the gearbox's gearing above its rated torque (Siemens' damage sum), life factors a1 and a_ISO.
- Of the transition curves: one length for all curves between lines and arcs of a path; none at an arc fixed
  at both ends, at an arc fixed at one end with an arc at its other end, or at an arc fixed at one end that turns
  more than half a circle.

## Belt tension limit read from a maker's handbook (12 November)

`ratings.belt_max_tension_n` of the EXCT-30 is 2140 N, no longer 990 N. The sixth review's figure took the weaker
of two belt classes from search summaries. Megadyne's Megalinear handbook could then be read itself: steel-cord T5
stretches 4 mm/m at 840 N (25 mm) and 1060 N (32 mm), AT5 at 1805 N and 2275 N. Festo's own stretch figure
(0.045 % at 300 N at 30 mm) is 667 kN per unit strain, against about 250 kN for T5 and 535 kN for AT5 with standard steel cords, so by its stretch the
belt is of the AT5 class and the limit is the AT5 figure at 30 mm. The highest tension in the worst case is
669 N. Test: `the_exct_30_config_passes_every_check_in_the_worst_case_of_both_band_modes`.

## The app opens and saves config files and takes part names (12 November)

`cargo test --workspace --release`: 245 tests (243 before), all pass.

| Gap | What was built | Test |
| --- | --- | --- |
| Config file in the app | A part "Config file" in the right panel: a file field, "Open" and "Save", and "Open…" and "Save as…" with the system's file dialog (`rfd` 0.18.0, pinned; it builds beside Bevy 0.20 on Windows). `Model::open_config(path)` loads the file with `load_config`, as `--config` does, and restarts the simulation with it as switching machine does; a file that cannot be read, has a key the machine does not know or does not start is reported under Messages and the running config stays. `Model::save_config(path)` writes the config as edited over the text it was read from (`toml_edit` 0.25.19, pinned, already built for Bevy's macros): only changed values are replaced, so comments and layout stay; where that would not read back as the same config (an array changed), it is written plainly by `Config::to_text` (new in `sim-core`), without comments. The screenshot hook takes `SIM_APP_CONFIG=file` | `a_config_file_is_opened_changed_saved_and_opened_again` in `sim-app/tests/app.rs` (both machines: the default config opened from a file, payload and an effect switch changed, saved: the file differs from the original in those two lines only, and opened again it is the changed config; a changed start pose is written plainly and reads back; a misspelt key, a start pose out of range and a missing file are answered with their messages, in both languages, while the running config stays and runs on) |
| Part names typed in | Three fields under the config file (motor article, supply, gearbox), filled with what `[parts]` names; "Use parts" (`Model::apply_parts`) names them in the running config by the same code as "Use" of the motor search (`name_parts`), which now checks the names with `catalog::resolve` before the config changes. A supply other than 230v and 400v, a name the catalogue does not have and any name in a config without `[parts]` (`error-config-missing parts.motor`, as on the command line) are refused | `typed_part_names_give_the_config_of_the_search_button_and_the_command_line` in `sim-app/tests/app.rs` (1FK2104-5AK at 400v: the same config as "Use" of the search and, resolved, as `--motor 1FK2104-5AK --supply 400v`; with gearbox NRK070-04-5 as `--gearbox`; unknown motor, unknown gearbox, 110v and a config without `[parts]` refused, the config unchanged) |

Left open: there is no `--set` in the app (values are changed in the parameter panel). The file dialog holds up
the window while it is open, and was not used by a person, only built; the screenshots show the controls.

## Frequency response, stiffness fit, replay overlay and operating points in the app (12 November)

`cargo test --workspace --release`: 249 tests (245 before), all pass.

What the command line computed on its own moved to `sim-components`, as `Sensitivity` did earlier, so that both
front ends compute the same thing: `identify::parse_sweep`, `identify::fit_sweep` and `SweepFit` (stiffness factor,
its error and the advice line), `trace::differences` and `trace::overlay_csv`. The CLI calls them; its output
was compared byte for byte before and after (replay, replay --out, fit --trace --sweep, fit --sweep in English
and German, a fit without a dip, analyze --points) and is the same. The advice keys `cli-fit-stiffness` and
`cli-fit-no-resonance` are now `fit-stiffness` and `fit-no-resonance` in the components' translations, with
the same text.

| Gap | What was built | Test |
| --- | --- | --- |
| Frequency sweep in the app | A part "Frequency response of the mechanics": from, to, count and torque amplitude as `sim-cli sweep` takes them (same defaults); "Measure every motor" runs `frequency_sweep` for each motor in the background. Gain in dB and phase over a logarithmic frequency axis, one line per motor, and a table with each motor's resonance and damping ratio (`identify::resonance`; empty where the range lacks the dip) | `the_frequency_response_is_measured_in_the_app` (both machines, 50 to 130 Hz, 30 points, 1.5 N m: every gain and phase equal to `frequency_sweep`, the resonance shown; a sweep from 0 Hz refused). `analyze_and_sweep_write_csv_for_every_machine` in `cli.rs` now also checks the CLI's sweep against `frequency_sweep` |
| Stiffness fit from a sweep in the app | A file field, the motor it was measured on and "Fit the belt stiffness" (`fit_sweep` with the amplitude above). A table as `fit --sweep` writes it (resonance and damping, config against measured, standard error) and the advice line | `the_belt_stiffness_is_fitted_to_a_measured_response_in_the_app` (both machines: a response of the default belt, EA 500 kN, against a config of 400 kN: factor 1.25 by hand, the fit within its standard error of it, which is below 0.05; a file without a dip `fit-no-resonance`, a missing file, a file that is no sweep). `a_trace_from_run_replays_onto_itself_and_fits_its_own_config` in `cli.rs` now checks the CLI's fit rows against `fit_sweep` |
| Replay overlay in the app | Below the trace tables: motor torque, motor speed and following error at the motor over time, trace solid and simulation dashed, one colour per motor (`Model::overlay`); "Export CSV" writes `replay-overlay.csv`, the file `replay --out` writes (`Model::export_overlay`) | `the_replayed_trace_is_overlaid_and_exported_in_the_app` (both machines: every plotted value equal to the trace and to `replay`, the exported file equal to `overlay_csv`). `a_trace_from_run_replays_onto_itself_and_fits_its_own_config` in `cli.rs` checks `replay --out` against `overlay_csv` |
| Operating points in the app | Under the cycle's tables: torque over speed once per millisecond, one colour per motor, over the torque-speed curve (its five corners from `DriveParams::torque_curve` of the analysed simulation), and the points as a table that only makes the rows in sight (`Model::points_table`, `Model::points_plot`) | `the_operating_points_are_shown_over_the_torque_speed_curve` (both machines: every row against `analyse_cycle`'s points; the curve against the hand values peak torque, rated and no-load speed of the resolved config; every point inside it). `analyze_and_sweep_write_csv_for_every_machine` in `cli.rs` checks `--points` against `analyse_cycle` |
| Screenshot hook | `SIM_APP_SWEEP=<file or empty>` runs the frequency response (50 to 130 Hz, 1.5 N m) and with a file its fit; `SIM_APP_POINTS` runs the cycle alone; `SIM_APP_SCROLL=<pixels>` scrolls the right panel | screenshots: sweep, operating points and trace overlay in English and German, at the default width; every new plot, table and button is inside the panel |

Left open: the operating points of a path are not shown (the CLI writes them for `--path` too). The app's own
sweep is not saved as a file; a measured response comes from a file. The overlay and the points plot were
looked at in screenshots only.

## Paths over OPC UA (12 November)

`cargo test --workspace --release`: 250 tests (249 before), all pass.

| Gap | What was built | Test |
| --- | --- | --- |
| Paths over OPC UA | Three variables in `sim-opcua`: `command.path` (String, the text of a path file, reads back what was written), `command.path_time_optimal` (Boolean, as `--time-optimal`) and `command.run_path` (Boolean, `true` starts). The path is planned with `Sim::plan_path` when `command.run_path` arrives, from where the machine stands, and runs as `Command::Trajectory` in real time; `status.busy` is true while it runs. Refusals go to `status.refused` as a path command's (same kind as a trajectory): `opcua-error-path-line` for a line that cannot be read (new key, English and German), otherwise the planner's or controller's key (`error-path`, `error-path-range`, `error-busy`, ...) | `a_path_from_the_plc_is_planned_and_run` in `sim-opcua/tests/plc.rs`, for every machine: a two-line path; every pose the client reads lies on the planned path within the analysis's path error peak for the same plan (T-gantry 0.11 mm against 0.34 mm, single axis 0.009 mm against 0.061 mm), the end within `ratings.settling_tolerance_m`, busy as long as planned within 50 ms; a line that cannot be read, a path beyond the stroke and (single axis) an arc are refused with their keys and nothing moves; a second start while one runs gives `error-busy`; the time-optimal plan of the same path finishes sooner (T-gantry 1.09 s against 1.17 s, single axis 0.74 s against 0.85 s) |

Left open: `status.refused` names the key, not the line that could not be read; poses at times (`--trajectory`)
are not offered over OPC UA. The path is planned inside the simulation's 10 ms tick, so a long time-optimal plan
holds the simulation back for that moment.

## Seventh review fixes (12 November 2026)

A seventh review found the points below by experiment. Each code fix has a test that was seen to fail with the
fix taken out. `cargo test --workspace --release`: 256 tests (250 before), all pass.

| Fix | What changed | Test |
| --- | --- | --- |
| 1 Save wrote over another machine's file | `Model::restart` (machine switch, config opened) kept the config file field, so Save after a switch wrote the new machine's config over the old machine's file. A switch now empties the field (opening a file sets it again), and `save_config` does not write over a file that its machine refuses but another machine loads: message `ui-config-other-machine` (English and German), file untouched. A file that is no config is written as before | `saving_after_a_machine_switch_does_not_overwrite_the_other_machines_file` in `sim-app/tests/app.rs` (failed on the field, and with the field fixed, on the overwritten file) |
| 2 enable, path and run in one OPC UA write | The path was planned while the inputs were read, before the Enable written before it was carried out, so it started from where the machine stood when disabled and was refused with `error-trajectory-start`. The commands of a tick are now carried out in the order written, the path planned when its turn comes. `error-trajectory-start` cannot occur over OPC UA any more (plan and check use the same pose), so it is not added to the documented refusals | `enable_path_and_run_in_one_write_run_the_path` in `sim-opcua/tests/plc.rs` |
| 3 run_path before path in one write | The old path ran while `command.path` read the new one. The path run is the one written last in the tick, whatever the order; both guides say so | `run_path_written_before_the_path_runs_the_new_path` in `plc.rs` |
| 4 the default sweep found no resonance | At 0.05 N m stiction held the motors of both default machines. New shared defaults in `analysis.rs`, used by the app and by `sim-cli sweep` and `fit`: `default_sweep_amplitude` = 1.5 N m, or 40 % of the drive's peak torque where that is less (0.22 N m for the smallest 1FK2, 0.56 N m peak), and `SWEEP_POINTS` = 120. Measured, 5 to 500 Hz, 120 points: T-gantry 21.6 Hz on both motors (0.05 N m: none), EXCT-30 20.8 Hz on both, single axis 84.3 Hz (0.05 N m: none; 0.5 N m: 70.4, 1 N m: 81.4, 2 N m: a false dip at 61.0 Hz). `identify::resonance` reports the dip lowest in frequency, not the deepest: the T-gantry's dips at 21.6 and 62 Hz are about equally deep (458 against 481 in gain times frequency), so the deepest could switch between config and measurement. The app sets the amplitude to the new config's default on a switch unless another one was typed | `the_default_sweep_finds_the_lowest_dip_on_every_config` in `sim-machines/tests/sweep.rs`; `of_two_dips_the_lowest_in_frequency_is_reported` in `identify.rs`; `the_frequency_response_is_measured_in_the_app` checks the app's defaults against the shared ones |
| 5 guides blamed the range | Both guides: the cause was the amplitude, the new defaults, and the 50 to 130 Hz advice removed; the `sweep` example output is the program's present output | none (documents) |
| 6 falling frequencies | `identify::resonance` sorts the points by frequency, so a range typed from high to low and a file with falling rows give the dip of the rising ones (they gave a negative damping) | `resonance_reads_frequency_and_damping_off_a_known_dip` in `identify.rs`; `the_frequency_response_is_measured_in_the_app` (range 130 to 50 Hz); `a_trace_from_run_replays_onto_itself_and_fits_its_own_config` in `cli.rs` (falling file) |
| 7 the sweep motor after a switch | `Model::restart` clamps `inputs.sweep_motor` to the motors of the new machine | `switching_to_a_machine_with_fewer_motors_keeps_the_sweep_motor_in_range` in `app.rs` |
| 8 test counts in `docs/validation.md` | 256, 211 and 45 (counted from the test runs; the 208 and 42 before added up to 250 but split one test wrongly) | none |

Limits: the default amplitude does not follow a motor named later with "Use parts" in the app; it is set when
a machine or config is loaded. On the single axis the dip moves with the amplitude (81.4 Hz at 1 N m, 84.3 at
1.5, a false dip at 2), so a motor with a peak below 3.75 N m there is swept where friction still pulls it low. The 40 % is a judgement, not a figure from a
drive maker.

## Effect views, part 1: torque breakdown, loss budget, friction view (12 November)

`cargo test --workspace --release`: 268 tests (256 before), all pass. Part 2 (thermal view, belt tension per span, effect on/off comparison) is left for later.

Each machine tells the views what it knows through the machine interface: signals with fixed names (`motor.<id>.torque_<part>`, `power.<place>`, `energy.<part>`, `<contact>.friction_<part>`; the conventions are constants in `sim-core/src/machine.rs`) and two methods with defaults, `friction_contacts` and `friction_curve`. A machine without them still runs; app, command line and analysis find the signals by name and name no machine. As signals they also reach the scopes, `sim-cli run` and the OPC UA variables. No config key was added.

| View | What was built | Tests (hand-calculated numbers) |
| --- | --- | --- |
| Torque breakdown | Per motor inertia, gravity, friction and the rest, live in the app (one plot per motor, the torque dashed) and over the analysed cycle as peak and RMS per part (`MotorLoad::torque_parts`; app table, `sim-cli analyze --breakdown FILE`). Forces are mapped onto the motors with the rigid kinematics' transpose; inertia at the load's accelerations, with rotors and pulleys turning as a rigid machine would (taken at the rotors' own accelerations, the backlash knocks of the default T-gantry put 1.3 N m peaks into inertia and the opposite into the rest, with a peak torque of 0.64 N m); the rest is what the other three leave, so the parts add up to the torque to rounding (1e-9 relative is tested) | `a_pure_inertia_move_is_all_inertia` (rigid loss-free, no gravity: J α = 1.22e-4 kg m² · 819.672 rad/s² = 0.1000 N m, gravity, friction, rest 0), `holding_z_is_all_gravity` (± 0.137293 N m; copper loss 0.37235 W), `a_constant_speed_move_is_coulomb_plus_viscous_friction` (r / i · (15 + 20 · 0.8) = 0.124 N m), gantry; `holding_the_vertical_axis_is_all_gravity` (0.235360 N m), axis; `the_torque_parts_add_up_to_the_motor_torque`, effects |
| Loss budget | Every machine reports its loss powers (guides, idlers, chain, shaft and brake, gearbox, belt, frame, Z guide, end stops, and ripple, meshing and process together) and its kinetic, potential and elastic energy; belt, gearbox and end stops got `loss_power`, the drives a copper loss and the simulation the energy the payload brings and takes (`Sim::payload_energy`). `EnergyBudget` per cycle or path: supply (net, with the rectifier's energy when the DC link is simulated), copper per motor, braking resistor, each machine loss, the stores' changes (windings, capacitors, machine), the payload carried off, a "drive model step" row, and what is not accounted for. App table, `sim-cli analyze --budget FILE` | `loss_budget` in the standard suite (the work of the torques against energy change plus losses, 0.1 %; measured 9e-5), `the_energy_budget_closes_on_every_machine_and_config` (18 configs; 0.1 % of the supply, measured at most 0.026 %), `loss_powers_are_the_hand_calculated_ones` (9 W, 10 W, 11.11 W, 5 W, 2 W), mech |
| Friction view | Per guide the steady LuGre curve over ±v_max at the guide load of the moment, its Coulomb, viscous and stiction parts, the live point, and the live split (`Lugre::steady`, `Lugre::split`) | `lugre_steady_curve_and_live_split_follow_the_formula` (15, 0.2, 5/e = 1.83940 N at 0.01 m/s and more), mech; `the_friction_curves_are_the_formula`, gantry; `the_friction_curve_is_the_formula`, axis |
| Front ends | App section "Effect views" (logic in `model.rs`: `breakdown_lines`, `breakdown_table`, `budget_table`, `friction_views`); screenshot hook `SIM_APP_EFFECTS` | `the_effect_views_show_torque_parts_energy_budget_and_friction`, app; `analyze_writes_the_torque_breakdown_and_the_energy_budget`, cli |

Found on the way:

- **The drive model puts energy into undamped modes.** The drive steps the current exactly but takes its power by the trapezoid rule and the back-EMF at the start of the step, while the mechanics turn under the torque of the current at the end of the step. On every config with damping this is below 0.001 % of the supply. On the loss-free elastic configs of the standard suite it is 3 to 5 % of the supply over a cycle (the mechanics get 1 J more than the windings converted in 3 s). It is a row of the budget of its own ("drive model step"); it is not fixed, because it changes no result on a damped machine.
- **Backlash knocks cost energy accuracy.** On the same undamped configs the machine's own balance is off by up to 0.8 %: the ringing knocks through the gearbox backlash, which the fixed step integrates through without events (0.91 J with backlash, 0.013 J without). Those configs get 1 % in the test.
- The energy split of a cycle on the default T-gantry: guides 50 %, motor shafts 35 %, braking resistor 3.8 %, gearboxes 6 %, copper 4.4 %, belt damping 0.004 %.

Left open:

- The motor shafts' friction, idlers and cable chain have no curve in the friction view (no stiction in their models); they are in the breakdown and the budget.
- The friction view is live only; the command line writes its signals with `run`, not a curve.
- Peak and RMS of the parts do not add up to those of the torque (they come at different times); the tables say so only in the guides.
- The live breakdown plots and the friction plots were looked at in screenshots only.

## Effect views, part 2: temperatures, belt sections, an effect on and off (12 November)

`cargo test --workspace --release`: 274 tests (268 before), all pass.

As in part 1, a machine tells the views what it knows through the machine interface: two more methods with defaults, `belt_sections` (id, owner, pretension) and `thermal_parts` (the loss signal `power.<id>` a part warms with, its kind `gearbox` or `guide`, and which scene part shows it), the signals `belt.<i>.stretch`, and the conventions `THERMAL_KINDS` and `TEMPERATURE_PREFIX` in `sim-core/src/machine.rs`. App and command line name no machine.

| View | What was built | Tests (hand-calculated numbers) |
| --- | --- | --- |
| Thermal model | New effect `part_heating` (off by default; nothing feeds back, so no earlier result changes). In `Sim`, not in the machine: each part of `thermal_parts` is one thermal mass, τ d(rise)/dt = R_th · P − rise, stepped exactly for the loss held over each base step; signals `temperature.<id>` (K), kept over a reconfigure. New config table `[thermal]` in all three configs: `gearbox_` and `guide_resistance_k_w`, `_time_constant_s` (estimates, with `[bounds]`) and `_max_rise_k` (the limits). T-gantry: both gearboxes and guides; single axis: gearbox and guide | `a_guide_at_constant_speed_warms_to_resistance_times_its_loss` (from cold, one τ later R P (1 − 1/e) within 1 %; twelve τ later R (λ F_c + b v) v by hand within 1 %, about 51 K and 72 K at 2 K/W), views. The winding's own model was tested before (`the_winding_warms_to_the_rated_rise_at_the_rated_current`: 100 K at the rated current, 51.3 K after one τ, 19.34 K at half the current) |
| 3D colour | `Part::heat`, set by `Sim::scene`: the k-th motor housing (`scene::MOTOR_PART`) by winding rise over `motor.rated_temperature_rise_k` while `winding_heating` is on, each heated part by rise over its rated rise while `part_heating` is on. The app draws it in a heat scale in place of the part's colour (`model::heat_color`: blue, yellow at half, red at the rating, magenta above); a material is updated only when its colour changed, so a part switched back loses the colour | `the_scene_colours_warm_parts_by_their_rise_over_the_rating` (25 K of 100 K is 0.25; half the rated rise 0.5), views |
| Thermal view | Over the scopes' window the rise of each winding and, with `part_heating`, of each part; table: now, where it settles on the analysed cycle (winding: `MotorLoad::winding_rise`; part: R_th times its budget row over the cycle's duration), rated rise, share; the colour scale. With both heating effects off it says so | `the_thermal_belt_and_comparison_views_show_the_simulation`, app |
| Belt view | Per section tension (pretension dashed) and stretch (mm) over the window; table of both now with `ratings.belt_max_tension_n` and the share of it (not drawn: at five times the pretension it flattened the lines); the 3D view's colour scale with its numbers (0 N, belt colour at P, red at 2 P). The 3D colour already used the same tensions as the signals ((T − P) / P per section); now tested | `belt_sections_have_the_pretension_at_rest_and_carry_the_weight_in_a_static_hold` (300 N each at rest; stretch L P / (EA + P), 0.4497 mm and 0.1499 mm; held, 29.42 N = 6 kg g sin 30° on the axis, 2 T₂ − T₁ − T₃ = 68.65 N = 7 kg g and T₁ = T₃ on the gantry; the strips' highlights are (T − P) / P of exactly these), views |
| Effect comparison | `analysis::compare_effect(entry, cfg, effect)`: the config with `effects.<effect>` true and false, each through the cycle on its own simulation (two threads), every signal once per millisecond of the measured (second) pass; `Comparison::difference` = peak and RMS of on − off over the rows both have. App: "Effect on and off" in the effect views (effect and scope quantity chosen, labels in front of the boxes, a background job on the config as it is, on solid and off dashed in the same colour, table of the differences); screenshot hook `SIM_APP_COMPARE=<effect>`. CLI: `sim-cli compare MACHINE --effect NAME [--signal ID]... [--out FILE] [--differences FILE]` | `comparing_friction_on_and_off_gives_the_friction_torque` (the config against itself: zero in every signal; guide friction at a cruise at top speed, r / i (λ F_c + b v) / η, × η for a motor driven back: single axis 0.1510 against 0.1525 N m, T-gantry A 0.0746 against 0.0739, B 0.0595 against 0.0592 N m, 2 %), views; `compare_writes_the_cycle_with_an_effect_on_and_off`, cli |
| Legends | The breakdown and friction plots of part 1 had their legend inside the plot, over the lines. Every effect-view plot now has a row of its lines' names in their colours above it (dashed ones marked "- -") and no legend inside | screenshots |

Found on the way:

- The T-gantry's guide friction on against off at constant X speed is not the X friction alone: the Z guide is switched with it, and while Z stands still its bristles hold about 2 N of what they held when Z stopped. That goes to motor A with +r / (2 i) and to B with −r / (2 i). The test reads it from the signal.
- With torque ripple and tooth meshing on, a single 1 ms sample of the motor torque is off the hand value by about 2.5 % (the controller answers the ripple within the sample); the comparison test switches both off.
- `sim-cli`'s German usage text lacked `--breakdown` and `--budget` of part 1; added.
- The thermal resistances and time constants have `[bounds]` as estimates, so `verdicts --bands` and the sensitivity run their cycle 8 more times on each config, though no verdict depends on them (their bands are zero).

Left open:

- Nothing feeds back: friction, belt stiffness and gearbox efficiency do not change with the computed temperatures (the plan's "computed thermal model" item stays open).
- The default time constants (15 to 30 min) make the live plot a slow climb; the table's "settles at" is what says where a part ends up.
- The EXCT-30 has no gearbox, but the T-gantry still names two gearbox parts; their rise stays 0.
- A disabled drive's winding keeps its temperature (the old shortcut); the parts go on cooling or warming with their losses.
- The compare view overlays one quantity at a time; the CLI writes any number of signals.
- The thermal and belt plots, the colour scales and the 3D colours were looked at in screenshots only.

## Review fixes of the effect views (12 November 2026)

A review of the effect views found the points below by experiment. Each fix has a test that was seen to fail with the fix taken out (the fix put back by hand into the file, the test run, the file restored). `cargo test --workspace --release`: 280 tests (274 before), all pass.

| Fix | What changed | Test |
| --- | --- | --- |
| 1 A direct drive listed gearboxes that warm | The criterion is `Gearbox::is_coupling`: ratio 1, which is how the configs say "no gearbox" (the EXCT-30's `[gearbox]` is the coupling; `[parts] gearbox = ""` does not say it, the default T-gantry has that too with ratio 5). `thermal_parts` of both machines lists gearboxes only where there are some, so the EXCT-30's thermal table and 3D view show no gearbox heating. The 9.1e-5 J per cycle of `power.gearbox_*` on the EXCT-30 is the coupling's damping; its quantity is now "Coupling damping loss (direct drive, no gearbox)" (`quantity-coupling-loss-power`, English and German), and the model reference says so instead of "is zero" | `a_direct_drive_has_no_gearbox_that_warms`, effects (every config, and each machine with `gearbox.ratio = 1`) |
| 2 The thermal share was that of the rise now | The share of the rating is that of the settled rise on the analysed cycle (`CycleLoad::part_rises`, R_th · Σ P dt / cycle time), shown beside the rise now, marked "(above the rating)" over 100 % (`ui-effects-settled-of-rated`, `ui-effects-over-rating`). New checks `gearbox-thermal` and `guide-thermal`: the hottest gearbox and guide where they settle, against `thermal.<kind>_max_rise_k`, judged only with `effects.part_heating` (otherwise not judged, like a rating left out); they reach `sim-cli verdicts`, the app's checks table (`check-gearbox-thermal`, `check-guide-thermal`), the bands, the motor search and the optimiser, which all use `verdicts` | `the_part_thermal_verdicts_judge_where_the_parts_settle`, effects (settled rise = R_th × its budget row / cycle time to 1e-12 on 18 configs; verdicts = the hottest; none without the effect); `the_thermal_share_is_that_of_the_settled_rise_and_marked_above_the_rating`, app |
| 2b The guide estimates failed the shipped configs | With part heating on, 2 K/W put the EXCT-30's X guide at 101.4 K (50.7 W mean loss) and the single axis's at 50.1 K, against 40 K. 2 K/W is a carriage with a piece of rail; a guide's heat goes into its rail and the aluminium profile it is bolted to, over the whole stroke: about 0.35 m² at 10 W/(m² K) (still air and radiation) with a fin efficiency of about 0.6 gives 0.5 K/W, bounds 0.25 to 1; about 5 kJ/K gives τ = 2400 s, bounds 1200 to 4800. No maker publishes these; it is an estimate from a heat balance. Now: EXCT-30 X guide 25.3 K (band 12.7 to 50.7 K: the worst case fails), single axis 12.5 K, default T-gantry 4.2 K. `part_heating` stays off by default | the same tests; values in the three configs, config reference, model reference |
| 3 Effect signals cost every step | The machine interface splits its signals: `base_signals` (what the verdicts look at every step), `effect_signals` (the views, after them) and `loss_powers` (the `power.*` alone, no derivatives); `signals` gives both parts. `Sim::step` takes only the loss powers, and only with part heating. `analyse_cycle_with(sim, views)` adds up breakdown and budget only where asked: the app's analysis and `analyze --breakdown/--budget` ask; bands, sensitivity, optimiser and motor search do not. `run` writes its rows without a string per value. No number changes: the outputs of `analyze`, `verdicts --bands` (default and EXCT-30) and `sensitivity` are byte for byte those of the last commit | `the_cheap_signals_and_the_lean_analysis_change_no_number`, effects (loss powers = the `power.*` signals at 100 points of the cycle on 18 configs; lean and full analysis give the same figures, part rises and verdicts). Seen failing with the lean mode taken out (views forced on) |
| 4 `[thermal]` bounds ran for nothing | `bounds` leaves out a bound on a value only an effect that is off uses (`TABLE_EFFECTS`: `thermal` with `part_heating`; still checked), and `sensitivity` skips such paths. With part heating off, bands and sensitivity are what they were before the effect views (7 all-zero sensitivity rows and 8 band runs gone); with it on, the thermal resistances move the thermal checks | `the_bounds_of_an_effect_that_is_off_get_no_runs`, effects (seen failing both with the bounds filter and with the sensitivity filter taken out) |
| 5 `Lugre::split` at load scale 0 | Divided by the level, NaN at λ = 0. With no level it returns the viscous part alone | `the_split_without_a_friction_level_is_the_viscous_part`, mech |

Timings, best of three, release build, this machine (the last commit built in a `git worktree` in the temp directory, removed afterwards):

| Command | Last commit | Before these fixes | Now (median of 7, interleaved with the last commit) |
| --- | --- | --- | --- |
| `sim-cli analyze t-gantry` | 511 ms | 595 ms (+17 %) | 533 ms (+4 %) |
| `sim-cli run t-gantry --cycle --time 10` | 476 ms | 559 ms (+17 %) | 504 ms (+6 %; about 40 more columns per row written) |
| `sim-cli verdicts t-gantry --bands` | 2731 ms | 3853 ms (+60 %) | 2810 ms (+3 %) |

The rest of `analyze` is the once-per-millisecond sample of every signal (the effect signals among them), which the analysis takes for tensions and loads; `run` with part heating on took 510 ms, no more than with it off.

Left open:

- One thermal resistance per kind: the T-gantry's Z guide, on a smaller profile, is taken with the X guide's value.
- `run` writes the effect signals, about 40 more columns per row than the last commit; that output is new, and costs what it costs.
- The lean analysis is chosen by the caller; a new caller that wants the budget or the breakdown has to ask for them.

## The electrical side of the drive (12 November)

`cargo test --workspace --release`: 287 tests (280 before), all pass.

| Item | What was built | Tests (hand-calculated numbers) |
| --- | --- | --- |
| Winding current in the integrated state | `Sim::step` integrates the machine's state with four entries per drive appended (winding current; electrical energy ∫ u i dt, copper loss ∫ R i² dt, shaft work ∫ τ ω dt from zero each step) in one RK4 vector; the core's integrator stays generic and the machine knows nothing of it. The voltage (and R at the step's temperature) is held over the base step, as an inverter holds it over a PWM period; the torque is k_t · i(t) with saturation (`DriveParams::winding_rates`). The controller now only sets voltages (`Drive::current_control`); `Controller::winding_stepped` takes the step's end: current, mean powers, DC links, trip, heating (from the step's mean copper loss) | the drive's unit tests run the same rates through four RK4 sub-steps (`current_loop` in the test module); every earlier hand calculation holds |
| Electrical side of the budget checked | The row "drive model step" is gone. `EnergyBudget::shaft_work` and `electrical_unaccounted()`: supply − copper − resistor − mains − line module − Δ windings − Δ capacitors − shaft work, within `ELECTRICAL_TOLERANCE` = 1e-6 of the supply on every machine and config, loss-free ones included (measured at most 3.2e-7; falls by 15 per halving of the integration step). With the DC link the supply is the rectifier's draw, counted on its own (`Drive::rectifier_power`), so the DC link's bookkeeping is checked too. Found on the way: the budget read its stores one step after its flows ended; that was the "0.8 % from backlash knocks" on the loss-free configs. Fixed (`Tally::end`): those configs now close to 0.03 %, all within the one 0.1 % tolerance | `the_energy_budget_closes_on_every_machine_and_config`, `the_supply_converges_with_the_integration_step_on_every_config` (4 against 8 sub-steps, 0.5 %, measured at most 0.04 %), effects |
| Cooling of a disabled winding | `Drive::disabled_step` runs the same thermal model with no copper loss; the `shortcut:` comment in `control.rs` is gone | `a_disabled_winding_cools_by_the_exponential` (40 K, τ 0.5 s: 14.715 K after 0.5 s, 5.413 K after 1 s, 0.1 %), electrical |
| Discharge of a disabled DC link | U ← U exp(−dt / τ) each step while disabled, then the DC link step with no power (the rectifier catches it at the bus level). New key `ratings.dc_link_discharge_time_constant_s` = 130 s in all three configs (estimate: Siemens' 5 minutes for 600 V to 60 V), bounds 30 to 300 s; no cycle uses it, so its bounds get no runs (`PATH_EFFECTS`) | `a_disabled_drives_dc_link_discharges_by_the_rc_exponential` (220 V, τ 0.2 s: 171.336 V after 0.05 s, the bus level between 69 and 70.5 ms, by hand 69.9 ms; own and shared links), electrical |
| Regenerative supply | New effect `regenerative_supply` (off by default, needs `dc_link`): above the bus level the line module feeds the DC link's energy back up to `ratings.line_module_max_power_w`, `line_module_efficiency` of it into the mains, before the chopper. Keys in all three configs: rated 16 kW and maximum 35 kW of the S120 Active Line Module 6SL3130-7TE21-6AA4 (catalogue), efficiency 0.98 (estimate, bounds 0.95 to 0.99, runs only with the effect). Reported: `MotorLoad::mains_energy`, `line_module_energy`, `peak_line_module_power`; budget rows `mains` and `line_module`; `analyze` column `mains_energy_j`; app motor table row and budget rows (English and German). Checks `line-module-continuous` and `line-module-peak` | `a_regenerative_supply_feeds_back_into_the_mains_up_to_its_power` (41 W: 4.018 J into the mains, 0.082 J lost; limited to 20 W: 1.96 J and 212.217 V), drive; `the_regenerative_supply_feeds_back_what_the_motors_regenerate` (mains = 0.98 × regenerated on every config), effects; `the_line_module_is_judged_against_its_ratings`, electrical; `the_regenerative_supply_shows_in_the_apps_tables`, app; the CLI budget test |

**What moved.** On the damped configs energies, RMS torques and verdict values move by well under 1 % (default T-gantry supply 139.54 → 139.48 J). Peak torques move more, because the old model's torque of the end-of-step current was a numerical phase lead of half a step that damped the gear and belt ringing: default T-gantry motor A peak 0.644 → 0.666 N m (`motor-peak` 0.1709 → 0.1769; the breakdown's "rest" peak 0.151 → 0.388 N m), single axis 0.600 → 0.647 N m (0.1596 → 0.1726), EXCT-30 4.0603 → 4.0613 N m. Both converge to the same value as the base step shrinks (T-gantry at dt/4: old 0.6487, new 0.6536 N m). Every README and guide example was rerun and updated.

**Found: the default velocity loops have little margin.** Without the phase lead, Kp = 0.08 N m s/rad with one cycle of delay in each loop is at the edge on both default machines: the single axis rings at about 350 Hz with guide friction off and goes unstable at the upper belt-stiffness bound (6e5 N), so `verdicts single-axis --bands` now FAILS `motor-peak` (worst 1.178), `belt-slack` and `settling` in the worst case; the T-gantry's motor B hunts in its gearbox backlash during an X cruise (±0.15 N m sample to sample); both loss-free configs ring up while holding; the single axis's loss-free elastic level config limit-cycles in the cycle (that is the one config the convergence test leaves out, by name). Kp = 0.06 removes the hunting on the T-gantry, 0.05 makes every single-axis case stable. The shipped configs were NOT retuned (a decision for the owner); tests that need a quiet loop set 0.05 or 0.06 (holding brakes on loss-free configs, the friction on/off comparison, the trace replays), each with a comment. The EXCT-30 is unaffected.

Timings (median of 7, interleaved with the binary before this change, release build, this machine): `sim-cli analyze t-gantry` 494 → 535 ms (+8 %), `sim-cli verdicts t-gantry --bands` 2627 → 2815 ms (+7 %). The cost is the integration itself: per drive four more entries in the RK4 vector, and at each of the 16 stages of a base step the machine's motor speeds (`motor_state`) and the winding's rates.

Left open:

- The shipped velocity gains (see above). Retuned since, see the next section.
- The line module draws without loss; its efficiency counts only on the way back. The bleeder draws only while the drive is disabled.
- Step-size convergence is checked in the integrator's sub-steps. Halving `sim.dt_s` (with the current cycle) changes the controller, not only the integration; on the loss-free configs the supply then changes a lot (T-gantry loss-free elastic 77 → 168 → 101 J at dt, dt/2, dt/4), because nothing damps what the loops excite.
- With `effects.back_emf` off the electrical side does not close (the shaft work comes from nowhere); no shipped config has it off.
- No signal for the mains power; it is in the analysis, the budget and the tables only.

## Velocity gains retuned for margin; fieldbus jitter, brakes in the rigid models, a second shaper (12 November)

`cargo test --workspace --release`: 292 tests (287 before), all pass.

| Item | What was built | Tests (hand-calculated numbers) |
| --- | --- | --- |
| Margins of the velocity loops | `Drive::torque_injection`, a test torque added to what the velocity loop asks for, before the limit. `analysis::velocity_loop_response` measures the open-loop response L = -U / E at one motor on the closed loop (machine holding its start pose, whole periods, sampled at the velocity cycle), `loop_margins` reads off phase margin, gain margin (crossings with \|L\| < 1) and the sensitivity peak, `velocity_loop_margins` does every motor for the config and each `[bounds]` value at its low and high bound (threads). `sim-cli margins MACHINE [--bands]` (usage text English and German) | `margins_of_an_integrator_with_dead_time_are_the_hand_calculated_ones` (54 degrees, 7.959 dB), analysis; `the_velocity_loop_response_is_the_hand_calculated_loop` (\|L\| 0.3973 and 0.1898 at 100 and 200 Hz by hand, measured 0.3976 and 0.1914; Kp doubled takes 6.02 dB off the gain margin), axis; the CLI against the library in `step_bode_and_tuning_write_csv_for_every_machine` |
| Default velocity gains | Target: phase margin at least 35 degrees and gain margin at least 6 dB in every one-at-a-time bounds run, with friction as configured and with motor and guide friction off (friction damps the loop near standstill, not in a cruise). T-gantry 0.08 -> 0.04 N m s/rad: at least 47.6 degrees and 8.3 dB, friction off 39.4 degrees and 7.3 dB (0.08 had 12 degrees and 2 dB). Single axis 0.08 -> 0.025: at least 48.8 degrees and 10.4 dB, friction off 38.5 degrees and 9.7 dB (0.08 had 1 degree and 0.3 dB). Reason and margins in a comment above the key in both configs. The EXCT-30 keeps 0.6 (at least 34.2 degrees and 6.4 dB; friction off 28.7 degrees and 5.6 dB, below the target, not retuned: not asked) | `the_shipped_velocity_loops_keep_their_margins_across_the_bounds`, control (35 degrees and 6 dB on both default configs, 27 degrees and 5 dB on the EXCT-30, friction on and off; 57 s) |
| Fieldbus jitter | Effect `fieldbus_jitter` (off by default), key `profile.fieldbus_jitter_cycles` = 1 in all three configs (labels English and German). Each frame of motor setpoints takes the delay plus j cycles, j even on 0..N from its own stream of `drive.noise_seed`; the drives use the newest frame that has arrived (so a late frame repeats the one before, and frames never go backwards). Works with or without `fieldbus_delay` | `a_fieldbus_jitter_spreads_the_setpoints_by_the_hand_calculated_rms`, axis: P(more than i cycles late) = prod (N - k) / (N + 1); RMS of the drive's position setpoint against no jitter at 250 rad/s 0.1768 rad (N = 1) and 0.2887 rad (N = 2), shares 1/2, 1/2 and 1/3, 4/9, 2/9, within 5 % and 0.05 over about 600 cruise cycles; switched on with 0 cycles = off |
| Brakes in the rigid models | The rigid models have a brake deflection state per motor (single axis 4 states, T-gantry 8); the brake torque at the rotor speed comes off the drive force before the gearbox loss, its loss in `power.shaft_*` and the torque breakdown's friction, as in the elastic model. Comments in the configs and the docs no longer say "elastic model only" | `the_holding_brake_holds_the_upright_axis_when_it_is_strong_enough` and `holding_brakes_hold_the_z_axis_when_they_are_strong_enough` now run in both models (0.26 N m holds, 0.2 N m lets the axis fall 136 mm in 0.5 s; 0.15 N m holds Z, 0.12 N m lets it fall 96 mm) |
| Second shaper resonance | Keys `profile.shaper2_frequency_hz` (0 = none, in all three configs) and `shaper2_damping` (labels English and German). A second ZVD shaper is convolved with the first in `Controller::shape`: nine copies at the sums of the delays with the products of the weights; without one its weights are 1 : 0 : 0, so every earlier result is the same to the bit. Dwells wait for both periods. The optimiser tries it where a setting with the shaper still fails `settling` alone, at the configured frequency or, where that is 0, at the resonance the machine reports at the start pose if that is more than 10 % off the first shaper's; switching it off is a candidate | `a_second_shaper_suppresses_a_second_resonance_by_the_zvd_formula`, axis: modes at 25 and 62 Hz excited by the setpoint acceleration, residual cos^2(pi f / (2 f_s)) by hand: one shaper at 25 Hz leaves 0 and 0.5314 (measured 0.0000 and 0.5314), one at 62 Hz 0.6503 and 0 (0.6492, 0.0002), both together 0.0000 and 0.0001 |

**What moved with the gains.** `verdicts --bands` now passes on all three configs, and so does `--bands --samples 200` (every share 1.000). Single axis, one at a time: `motor-peak` worst 1.178 -> 0.1423, `belt-slack` worst 0 -> 120.9 N, `settling` worst 0.1004 -> 0.0322 mm. T-gantry: `motor-peak` A 0.1769 -> 0.1476 (worst 0.1938 -> 0.1545), `settling` 0.0457 -> 0.0473 mm, `guide-load` 181.0 -> 177.8 N. The torque breakdown's "rest" on motor A fell from 0.388 to 0.063 N m peak and the friction comparison's peak difference from 0.28 to 0.096 N m: the hunting in the backlash is gone. The position loop's bandwidth rose a little (X 9.5 -> 10.6 Hz, `tuning`). The EXCT-30 is unchanged. Every example in `README.md`, both guides and `docs/de/README.md` was rerun (a script ran each `sim-cli` example and compared the shown lines) and updated where it moved.

Tests changed with the gains: the test-local gains of 0.05 / 0.06 are gone from the holding-brake tests, the friction comparison and both trace replays (app and CLI). `step_and_frequency_response_show_the_position_loop_gain` sets 0.08 itself (it needs a velocity loop much faster than Kv; at 0.025 the loop's Kp / J is 255 rad/s against Kv = 50). The friction comparison takes the mean over 0.1 s of the cruise (the difference swings by about 3 % with the 22 Hz belt mode). `a_shaper_that_follows_the_height_leaves_about_a_tenth_at_both_heights` (renamed from "under a tenth") allows 0.11: the plain swing is smaller with the lower gain, what is left is not (0.102 at z = 0.15 m; 0.077 at 0.08). The supply-convergence test has no saturated config any more and compares the single axis's loss-free inclined config at 16 sub-steps instead of 8 (20.925, 20.804, 20.924, 20.925 J at 4, 8, 16, 32: the undamped run at 8 alone takes another path through the backlash). Several rigid-model tests that wrote states by hand got the brake state.

Found on the way:

- The fieldbus jitter makes the energy budget miss 0.12 % of the T-gantry's supply (limit 0.1 %): the drives' position setpoint holds for a cycle and then jumps, and the budget's losses are taken once per base step. With half the step it is 0.03 %, so it is the sampling, not a missing loss. The "every effect" config of the effect tests therefore leaves the jitter out, with a comment.
- The margins depend on the amplitude: near standstill the motor's Coulomb friction (smoothed over 0.5 rad/s) is a damping comparable to Kp. Hence the target with friction off.

Left open:

- The EXCT-30 stays at 0.6 N m s/rad with 28.7 degrees and 5.6 dB at worst (friction off); retuning it would move every EXCT-30 number in the documents.
- The margins are taken one bound at a time, not sampled; 60 points can miss a sharp resonance between them; no real drive's measurement was compared.
- Without a configured second frequency the optimiser's second shaper uses the one resonance the machine reports; the T-gantry's upper mode (about 104 Hz at z = 0.05 m) is not reported by the machine interface.
- The second shaper does not follow the pose.
- No test runs the optimiser into the second shaper (it would need a config whose faster settings fail `settling` with one shaper); the candidate logic is covered only by the unchanged optimiser tests.
- The jitter is per frame, evenly spread and independent; real buses have other distributions, and a drive may extrapolate a missing frame instead of holding it.

## EXCT-30 velocity gain retuned; transition curves at splines and fixed arcs; worst-corner search (12 November)

`cargo test --workspace --release`: 297 tests (292 before), all pass.

| Item | What was built | Tests (hand-calculated numbers) |
| --- | --- | --- |
| EXCT-30 velocity gain | `drive.velocity_kp_nm_s_rad` 0.6 -> 0.45 N m s/rad in `exct-30.toml`, to the target of the default configs: at least 35 degrees and 6 dB in every one-at-a-time bounds run, friction as configured and off (`sim-cli margins t-gantry --config crates/machine-t-gantry/exct-30.toml --bands`). Measured now with 0.6: 34.9 degrees and 6.6 dB, friction off 29.6 degrees and 5.8 dB (winding inductance at its upper bound; the section above gives 28.7 degrees and 5.6 dB, which this run does not reproduce, cause not looked into); 0.5 would leave 35.6 degrees friction off; 0.45 has at least 47.7 degrees and 9.2 dB, friction off 39.2 degrees (belt damping at its lower bound) and 8.3 dB. `verdicts --bands` and `--bands --samples 200` (seeds 1 to 3) pass. Reason and figures in a comment above the key | `the_shipped_velocity_loops_keep_their_margins_across_the_bounds` now holds the EXCT-30 to 35 degrees and 6 dB too; `the_search_retunes_the_velocity_loop_for_a_lighter_rotor` reads the config's gain (by hand 0.45 * 0.517 = 0.23 for the 1FK2204-6AF) and shows the oscillation with the 1FK2104-5AF, because at 0.45 the 1FK2204-6AF runs calmly unretuned (1.34 N m RMS, 13.4 K; at 0.6 it oscillated) |
| Arc fixed at one end | An arc at a corner, at an end of the path or beside a spline, whose other end meets a line in its direction, gets its transition curve there: its circle shrinks by s and its centre moves towards the fixed end by s, so it passes through that end in the same direction and is s (1 - cos phi) from the line; that gap is made e = L^2 / (24 r), s = e / (1 - cos phi), for arcs of up to half a turn whose radius stays above half. The reported deviation is the larger of e and the circles' distance on the arc, r - sqrt(r^2 - 2 s (r - s)(1 - cos phi)) | `an_arc_at_an_end_or_a_corner_gets_its_transition_curve`, path: a quarter arc of 0.1 m that ends the path, and the same with a corner after it, 0.1 mm: s = e, L = sqrt(24 e r) = 15.49 mm, 0.3957 m/s into the arc by hand (measured 0.3955; 0.0707 m/s without), the arc ending on (0.6, 0.25) in its direction to 1e-12, every pose within the reported 0.1 mm, the axis limits held |
| Spline eased into a line or an arc | `ease` in `path.rs`: where a spline meets a line or an arc without a corner, the span next to the junction gets delta(u) = D l^2 w^2 (1 - w)^3 / 2 added over l from the junction (w = u / l), D the step of the second derivative across the direction (|D| = dk): the junction keeps place and direction, its curvature becomes the neighbour's, the rest of the spline and every via point stay. The eased part is a polynomial of the fifth degree (`Geo::Spline` now holds six coefficients). It moves the spline by at most 0.01728 dk l^2, so l = min(sqrt(e / (0.01728 dk)), 9 * 1.05 dk v^3 / (0.4 j), half the span), per junction | `a_spline_eased_into_a_line_is_entered_at_the_hand_calculated_speed`, path: curvature 3 dz / h^2 = 10 1/m at the junction by hand (spline formulas), eased over 24.06 mm at 0.1 mm, junction speed cbrt(0.4 j / (1.05 (9 dk / l + d0 x 6 c3))) = cbrt(40 / (1.05 * 3635.2)) = 0.2189 m/s by hand, measured 0.2188 (0.0707 without); every pose within the reported 0.1 mm of the spline as given and above 0.09 mm; the axis limits. `a_spline_passes_through_its_via_points_within_the_axis_limits` now also with 0.1 mm on its four paths: no step in position, direction or curvature anywhere, poses within the reported deviation, via points passed, faster (2.129 s against 2.251 s, 2.415 s against 2.701 s, 2.093 s against 2.184 s) |
| Transition lengths of the time-optimal plan | `geometry` takes the speed the plan allows per piece: the axis limits for `plan`, the motors' limits for `plan_time_optimal` (the per-piece motor limits are now `motor_limits`). `path::deviation_time_optimal`, and `Sim::path_deviation(pieces, time_optimal)` (CLI, app and tests pass the flag) | existing time-optimal tests; on the user guide's line, arc, line the time-optimal plan's curves were already at the tolerance's length (1.499 s unchanged) |
| Worst-corner search | `analysis::verdict_worst` (and `Worst`, `WORST_PASSES` = 3): the one-at-a-time runs; per verdict the corner where each parameter is at the bound that made it worse alone; then up to three passes of flipping, one at a time, each parameter that moved the verdict, taking the flips that made it worse together or the best alone, whichever is worse, until a pass finds nothing. "Worse" is the margin to the limit (limits such as the DC link's are bounded too). A corner is run once for all verdicts. `sim-cli verdicts --bands --worst` (columns `check,motor,worst,limit,worst_case,at_high_bound,at_low_bound`, the number of cycles on the error stream; refused without `--bands` or with `--samples`/`--seed`, `cli-error-worst`, `cli-worst-runs`, English and German). App: tick box "worst corner" beside the error bands (`Inputs::worst`, `Model::worst`, `Model::worst_table`; labels English and German) | `the_worst_corner_search_finds_the_hand_calculated_corner`, axis: carriage 4 to 6 kg, payload 2 to 4 kg, gearbox efficiency 0.85 to 0.95, belt damping (no effect): `gearbox-peak` at both masses high 0.2 (6 + 4 + 0.25) = 2.05 N m by hand, found 2.070; `motor-peak` at both masses high and the efficiency low, (2.05 / 0.85 + 0.375) / 5 / 3.8 = 0.1467 by hand, found 0.1481; every check at least as bad as any of 40 samples, seeds 1 and 2. `verdicts_search_the_worst_corner_of_the_bounds`, cli; `the_app_searches_the_worst_corner_of_the_bounds`, app |

**What moved with the gain** (EXCT-30, every example in `README.md`, both guides and the config reference rerun and updated): `motor-peak` A 0.3255 -> 0.3222, `settling` 0.0351 -> 0.0385 mm (bands 0.043 mm, 200 samples 0.053 mm), lowest belt tension 239 -> 241 N (bands 141 N, samples 123 N), the motor search's figures by a few per cent (the 1FK2104-6AF's tightest check is now `settling`), the optimiser's result 1.934 -> 1.492 s as before but 0.095 mm in the dwell instead of 0.070 mm, the analysis without the shaper 0.058 mm instead of 0.071 mm. The sweep's resonance (20.8 Hz) is unchanged.

**Found: the EXCT-30 fails `settling` at its worst corner.** `verdicts --bands --worst` on the EXCT-30 (1884 cycles, 33 s) finds the tool 0.115 mm off its target in the dwell against 0.1 mm, with all bounded values at once on the side that makes it worse; one at a time it is 0.043 mm, in 200 samples 0.053 mm. With the old gain of 0.6 it is 0.101 mm, so it failed before the retuning too, by less. Every other check passes at its worst corner (belt 65 N in its slackest section, the shafts 1238 N of the 1500 N of their lowest rating bound). The config was NOT changed for it (not asked; it would move every EXCT-30 number again). The default T-gantry (1183 cycles, 69 s) and the single axis (311 cycles, 3.5 s) pass every check at their worst corners.

Left open:

- The worst corner is a search: a worst case that no chain of single flips from the sign corner reaches, or one inside the bounds (a resonance hit by a stiffness between them), can be missed. The CLI has no option for the number of passes. Its cost is about 15 times that of `--bands` on the EXCT-30.
- The eased spline junction changes its curvature at the junction about nine times as fast as a clothoid of the same length, and overshoots it by up to 0.36 dk on the way, so it is passed about 1.8 times more slowly than a clothoid of the same deviation would allow. A clothoid itself does not fit there without moving the spline the way an arc is moved, which a spline cannot be without moving its via points.
- One length for all curves between lines and arcs of a path (each spline junction has its own). An arc fixed at both ends, fixed at one end with an arc at the other, or fixed at an end and turning more than half a circle, stays as given.
- The app's worst-corner search runs beside the bands, so the one-at-a-time runs are done twice. The screenshot hook does not tick it.

## EXCT-30 worst corner fixed; life estimates and the belt's tooth load (12 November)

`cargo test --workspace --release`: 301 tests (297 before), all pass.

**The EXCT-30 passes `settling` at its worst corner.** `profile.shaper2_frequency_hz` 0 -> 17.5 Hz in `exct-30.toml`: a second ZVD shaper at the peak of the tool's closed-loop response, which the file's comment already named. `verdicts --bands --worst`: 0.095 mm against 0.1 mm (0.115 mm before); one at a time 0.044 mm, 200 samples 0.051 mm. The cycle takes 2.048 s instead of 1.934 s (each stop waits 57 ms more). Tried at the worst corner and not taken: the first shaper at 17.5 or 20 Hz alone (0.114, 0.110 mm in the full search), Z jerk 300 m/s³ (0.099 mm, too close), X jerk 100 m/s³ (worse), velocity gain 0.4 or 0.5 (worse, and the margins test holds 0.45). The tolerance and the gain are unchanged; the shaper does not touch the loop margins. Moved: `motor-peak` A 0.3222 -> 0.2930, `settling` 0.0385 -> 0.0395 mm, lowest belt tension 241 -> 258 N (bands 158 N, samples 140 N, worst corner 87 N); the optimiser's result is the same 1.492 s (it switches the shapers off); the search's figures. Every EXCT-30 example in `README.md`, both guides and the config reference was rerun and updated.

| Item | What was built | Tests (hand-calculated numbers) |
| --- | --- | --- |
| Pulley shaft bearings | ISO 281 rating life L10h = (C / P)³ · 10⁶ / (60 n) per motor's pulley, P = cbrt(Σ F³ \|ω\| / Σ \|ω\|) of `pulley.<motor>.radial_load` once per ms, n the pulley's mean speed (motor over `gearbox.ratio`). Key `ratings.pulley_bearing_load_rating_n`: T S 8469 N (output bearing of a Siemens NRB120: 1500 N for 30000 h at 100 r/min, Table 7-14 of the planetary gearbox configuration manual 01/2023; the NRB060 whose torque ratings the default configs take allows 340 N, less than their 600 N of pretension: about 2500 h on this cycle); E 9500 N [estimate, 8000 to 12000] for two bearings of about 5.85 kN sharing the pull (a pair = 1.62 C). Verdict `pulley-bearing-life` | `the_lives_and_the_tooth_load_are_the_hand_calculated_ones` (468696 h), analysis; `the_life_loads_are_the_signals_added_up` (1e-9), life |
| Gearbox planet bearings | Siemens' formula L10h = 20000 h · (n2B / n2) · (M2N / M2eq)^(10/3), M2eq the speed-weighted mean of the output torque with the exponent 10/3, every step. Key `ratings.gearbox_reference_speed_rpm` = 175 r/min (NRB060, ratio 5, Table 7-15) in all three; not judged on the EXCT-30 (no gearbox, no rated torque). Verdict `gearbox-life` | 352778 h by hand, analysis; the plumbing test, life |
| Guide life in hours | L10 km over the mean speed of the guide's axis (`CycleLoad::guide_speed`). Verdict `guide-life` | 6944 h by hand, analysis; the plumbing test |
| Belt teeth | No belt maker publishes a life (Megadyne MEGALINEAR 02.2026, Gates TPU catalogue 2025, BRECO AT5 sheets read; Optibelt unreachable). Instead Megadyne's rating: max over the cycle of τ_out / r · c_s / (F_spec(n) · b / 10 · min(z_m, 12)), the output torque's peak per 5 rad/s speed band, each band at its upper speed. Keys `ratings.belt_tooth_load_rpm` and `_n_cm` (Megadyne's AT5 table, arrays), `belt_width_mm` (30 E, 25 T S), `belt_service_factor` 1.4, `belt_teeth_in_mesh` 10 [estimate, bounds]. At standstill the EXCT-30's teeth carry 35 · 3 · 10 / 1.4 = 750 N, near Festo's 786 N (12.5 N m). Verdict `belt-tooth-load` | 0.13707 and 0.14359 by hand, analysis |
| Required life | `ratings.required_life_h` = 20000 h in all three. The lives are judged from below (`analysis::AT_LEAST`, used by the worst-corner search's badness, `analysis::utilisation` and the motor search's tightest check) | `the_life_verdicts_judge_the_lives_against_the_required_life` (0.1 % either side), life; `the_worst_corner_search_finds_the_hand_calculated_corner` now judges the lives from below |
| Front ends | `sim-cli analyze`: columns `guide_life_h`, `pulley_bearing_life_h`, `gearbox_life_h`, `belt_tooth_utilisation`. App: a summary row and three motor rows (only where a motor has the figure), four check labels, English and German. Bands, samples and the worst corner include the new verdicts on their own | `analyze_and_sweep_write_csv_for_every_machine`, cli; `the_cycle_is_analysed_and_judged_in_the_app`, app |
| EXCT-30 regression | `verdict_worst` on the EXCT-30 in the suite (about 47 s on all cores, not ignored) | `the_exct_30_config_passes_settling_at_the_worst_corner_of_its_bounds`, catalog; the cheaper `the_exct_30_config_passes_every_check_in_the_worst_case_of_both_band_modes` (cycle 2.048 s) |

Figures on the shipped cycles:

| Config | Pulley shaft bearing | Gearbox planet bearings | Guide | Belt tooth load |
| --- | --- | --- | --- | --- |
| T-gantry default | 212500 h (A), 212200 h (B); bands 89800 h | 6.7e9 h, 1.0e10 h (torques far below the 40 N m rating) | 5.0e6 h | 0.19, 0.15 |
| Single axis | 136000 h; bands 57300 h | 4.3e9 h | 1.8e7 h | 0.17 |
| EXCT-30 | 31800 h; bands 17300 h (belt set 100 N high), 200 samples 11900 h (86 % pass), worst corner 10300 h: **fails 20000 h in all three band modes** | not judged | 3.0e6 h; worst corner 115600 h | 0.35, 0.28; worst corner 0.73 |

**Found: the EXCT-30's pulley shaft bearings.** With the pretension alone pulling about 900 N on each shaft and the pulleys turning all cycle long, the bearing pair's rating decides: 31800 h with 9500 N, 19000 h at the 8000 N bound. Festo publishes nothing about them; this is the part to ask Festo about. The existing band test holds this as a finding (every other check passes; `pulley-bearing-life` passes as written and fails in the worst case). Every passing motor of the EXCT-30's search now has `pulley-bearing-life` (0.631) as its tightest check, which the motor hardly changes.

Left open:

- The motors' own bearings: in all three configs the belt pulls on a gearbox output or on Festo's drive head, not on a motor shaft. Siemens gives 25000 h for the 1FK2's bearings at medium load and allowable radial forces for 25000 h (S210 operating instructions 09/2022, 12.1.12), no load rating.
- The gearing of the gearbox (Siemens' damage sum over partial cycles above M2N) and the output bearing's own formula (30000 h at the diagram's force) are not computed separately; on the default configs the output bearing is the pulley shaft bearing.
- The reference speed is not in the gearbox files, so a gearbox by name keeps the config's 175 r/min. Siemens' 04/2025 manual could not be downloaded; the figures are from the 01/2023 edition.
- One bearing takes all of a pulley shaft's radial load; no axial load, no life factors; the radial load is the sum of the two section tensions (the machines' shortcut).
- Tooth load: the teeth in mesh are a fixed estimate, not the wrap angle of the machine; beyond the table's last speed its last value holds.
- No real part was compared; the formulas are checked, not the lives.

## FMI export (13 November)

`cargo test --workspace --release`: 308 tests (301 before), all pass.

The plan's "FMI export" item: a new front-end crate `sim-fmi` and `sim-cli fmu MACHINE [--config FILE] --out FILE.fmu [--binary FILE]...`. How to use it: user guide, "Use the model in Simulink / Modelica via FMI".

| Decision | Why |
| --- | --- |
| FMI 2.0 Co-Simulation, not 3.0, not Model Exchange | Simulink, OpenModelica and Dymola all import 2.0 CS; nothing the simulator needs is 3.0-only. Model Exchange would hand the integration to the tool, but the drives' cascaded loops run on their own fixed cycles inside `Sim::step` |
| Inputs: one position setpoint per axis and a drive enable | The same interface as a PLC over OPC UA (`Command::Setpoint`, interpolated over `profile.setpoint_interval_s`); the servo loops stay inside, as the plan says. No torque mode: it would bypass the drive model that the simulator is about |
| Outputs: every signal of `Sim::signal_infos`, with its unit; `time` as the independent variable | Generated, never written per machine. Naming convention "flat": ids such as `belt.1.tension` are not "structured" identifiers |
| Parameters: `machine`, `config_file` (a path; empty for the config in the FMU), `effects.<name>` | Fixed (settable until initialisation ends). A config that changes the signal list is refused then, since the variables are fixed by the model description |
| Stepping: whole base steps only | A communication step that is not a whole multiple of `sim.dt_s`, or starts elsewhere than the simulation stands, is refused with `fmi2Error`; nothing is interpolated. Default experiment step: the setpoint interval |
| One generic binary for all machines | `resources/model.txt` (machine id, language) and `resources/config.toml`; `fmi2Instantiate` regenerates the model description from them and refuses a GUID that does not match it. GUID: two FNV-1a 64-bit hashes of the description (stable across builds, unlike `std`'s hasher) |
| No new dependency | A stored zip with CRC-32 (`sim_fmi::zip`, `crc32`) and the XML written by hand. The C types follow the FMI 2.0 headers (BSD-2-Clause), noted in `api.rs` |
| `sim-cli` depends on `sim-fmi` | The FMU's description and archive come from the same code as its binary. The architecture test gives `sim-fmi` its own front-end layer (4) below the other front ends (5) |

| Item | Tests |
| --- | --- |
| C functions against a plain `Sim`, every machine: a ramp of setpoints then a hold, all signals bit for bit at every communication point | `a_move_through_the_fmu_matches_a_plain_simulation_bit_for_bit` (fmi) |
| Effect parameter and drive enable, bit for bit against the config switch and `Command::Disable` | `effect_switches_and_the_drive_enable_act_on_the_simulation` (fmi) |
| Error paths: wrong GUID, wrong FMU type, missing resources, bad config file, step not a multiple, wrong communication point, parameter after initialisation, output set, unknown value reference, Integer access, unsupported calls, reset | `what_the_standard_forbids_is_refused` (fmi) |
| GUID changes with language and start values, not between runs | `the_guid_changes_with_the_model_description` (fmi) |
| CRC-32 check value; German and English for every message | `crc32_of_the_standard_check_string`, `every_export_message_exists_in_every_language` (sim-fmi lib) |
| The written `.fmu`: zip headers, CRC and central directory, layout, balanced XML, every signal with unit and unique value reference, `Outputs`; the default binary and refusals | `fmu_writes_an_archive_with_the_fmi_layout_and_every_signal` (cli) |

Left open:

- **No independent FMI tool.** `fmpy` is not in the project's `.venv` and nothing was installed. By hand, the FMUs of both machines were opened with Python's `zipfile` and `xml.etree` and the DLL driven through `ctypes` (instantiate with a logger, a ramp of setpoints, a refused step with its logged message): all as expected. Not imported into Simulink, OpenModelica or Dymola; the description is not checked against the FMI XSD.
- Only a win64 binary is built here. A linux64 one needs `cargo build --release -p sim-fmi` on Linux and `--binary libsim_fmi.so`; not tried.
- No FMU state (get/set/serialize), no directional derivatives, no asynchronous steps; move targets, the cycle and paths are not inputs.
- Messages are logged at warning and error level only, always; `fmi2SetDebugLogging` changes nothing.

## Eighth review fixes (13 November 2026)

Two independent reviews found the points below by experiment. Each fix has a test that was seen to fail with the fix taken out (the fix taken out of the file by hand, the test run, the file restored). `cargo test --workspace --release`: 315 tests (308 before), all pass.

| Fix | What changed | Test |
| --- | --- | --- |
| 1 Loop margins could not be negative | `loop_margins` took the phase margin as the unsigned angle between L and −1 and dropped every −180° crossing with \|L\| > 1, so L = K e^(−sT) / s with K = 2π · 400, T = 1 ms read +53.95° and +10.28 dB (exact −54°, −4.08 dB) and passed the 35° / 6 dB target. Now the phase is followed from the lowest point up without jumps (taken between −360° and 0° there), the phase margin is 180° + ∠L at the crossover (signed), and the gain margin is the worst −20 log \|L\| where the phase falls through −180° (negative where \|L\| > 1). Where the phase rises back through −180° with \|L\| > 1 the loop is conditionally stable (a falling gain is the danger there); those crossings are still not counted, as counting every crossing, as the review asked, would make every loop with integrators and a position loop read unstable. `velocity_loop_response` marks a point NaN where the torque limit was hit or the loop moved on its own (the part of e that is neither the injected sine nor a constant larger in RMS than the injection); `LoopMargins::valid` and `stable()`. `sim-cli margins` has a column `result` (ok, or FAIL for invalid or unstable); the shipped-margins test requires `stable()` | `margins_are_signed_and_say_an_unstable_loop_is_unstable` (72° / 13.979 dB, 54° / 7.959 dB, −18° / −1.584 dB, −54° / −4.082 dB), analysis; `an_unstable_velocity_loop_is_reported_unstable` (single axis at 0.3 N m s/rad: every point hits the torque limit; with the fix out it read 194.9° and +0.38 dB, which passed as stable), axis; the CLI's FAIL rows at that gain on every machine in `step_bode_and_tuning_write_csv_for_every_machine` |
| 2 Electrical energy lost on disable and enable | `disabled_step` set the winding current to zero in one step (½ L i² gone) and `hold_here` reset the DC link to the bus voltage on Enable (EXCT-30 249.7 V → 155.13 V, about 18 to 20 J gone). Now the current in the winding when the pulses stop freewheels through the diodes into the DC link (as the drive's power over the first disabled step, −½ L i² / dt; against the link voltage it is gone within microseconds, well inside a step, so it is not decayed over the winding's time constant), and Enable keeps each drive's DC link charge, as it keeps the winding temperature, less the ½ L i² of the holding current (below the bus level the rectifier makes that up at the next step). A drive enabled after a trip keeps its charge too: the single axis's trip test said "enabling starts with the DC link at the bus level" and now checks that the charge is kept and the chopper takes it below the trip level within the next step. Model reference updated | `disable_and_enable_conserve_the_electrical_energy` (windings plus capacitors across Disable and Enable, every machine, own and shared link, 1e-9 relative; seen failing for each half: 1.6e-5 J lost on disable, 3.1 J lost on enable), electrical; `a_disabled_drives_dc_link_discharges_by_the_rc_exponential` now steps once before setting 220 V |
| 3 Two arcs on one circle lost a whole turn | Their radii differed by rounding, so a degenerate transition curve was built between them and what it took of the arcs wrapped by 2π: the reviewer's path (0.05 m circle, 4 rad then π + 0.05 rad) was planned 0.1953 m long for 0.5096 m. `blend_with` now treats curvatures equal within 1e-9 relative and gaps below 1e-12 m as no junction to blend; refuses (shorter curves, then the path as given) where a curve would take more of an arc than its length times its curvature can reach; and checks the blended path's length against the given one's, allowed to differ by the curves' lengths plus each moved arc's shrink times its angle | `two_arcs_on_one_circle_keep_their_whole_turn`, `blended_random_paths_keep_their_length_and_turning` (400 random paths of tangent lines and arcs, half the arcs continuing on the circle before, three tolerances: length within that allowance, turning to 1e-6 rad), path |
| 4 `fmi2DoStep` with an infinite size hung the importer | It looped u64::MAX steps, and a NaN communication point passed every comparison. Both, and NaN sizes and infinite points, are refused with `fmi2Error` (`fmi-error-step`) | `early_reads_and_steps_that_are_not_finite_are_refused`, fmi |
| 5 The worst corner better than the bands | `dc-link-energy` passes or fails by whether the braking resistor is needed and has a rating, not by value against limit; with a resistor the link fills to its room at every corner that brakes hard, so its badness (the margin) was 0 at all of them and the corner with the least room came out: 4.47 J on the EXCT-30 against bands up to 23 J. It is now ranked by the energy in the capacitors, a failing run above any passing one. And `verdict_worst` counts the one-at-a-time runs and the config as it is as candidates, so the worst is never better than the bands; `Worst::high` is `Vec<Option<bool>>` (`None`: as configured, named in neither column of the CLI and the app). EXCT-30 `--worst`: `dc-link-energy` a 4.470 → 35.66 J (limit 36.57 J), b 36.57 J; 2429 → 2428 cycles; no other row moved. Both guides' example and text updated | `the_exct_30_config_passes_settling_at_the_worst_corner_of_its_bounds` now also checks the worst of every check against its one-at-a-time band, catalog |
| 6 FMU time output stale | `fmi2GetReal` in the Instantiated state built the simulation and sampled it at start time 0; a later `fmi2SetupExperiment(start = 5)` left the `time` output at 0. Both remedies: `fmi2GetReal` is refused before initialisation mode (FMI 2.0 allows getting values from there on), and `fmi2ExitInitializationMode` takes a fresh sample | `early_reads_and_steps_that_are_not_finite_are_refused` (5 s after setup, 5 s + dt after one step), fmi |
| 7a Time-optimal deviation of a refused path | `deviation_time_optimal` reported a deviation for a path `plan_time_optimal` refuses with `error-path-torque`. Both now go through `time_optimal_geometry`, which refuses | `the_time_optimal_deviation_refuses_what_the_plan_refuses` (torque reserve 0.1 %: refused on the T-gantry, whose Z motors hold the load), control |
| 7b Two `--binary` for one platform | They went into the archive as two entries of one name. Refused with `cli-error-conflict`, no file written | `fmu_writes_an_archive_with_the_fmi_layout_and_every_signal`, cli |

**What moved.** The margins move a little with the new interpolation (in log \|L\| and phase instead of in L): `sim-cli margins t-gantry` a 57.0° / 9.09 dB → 56.8° / 9.05 dB, b 54.2° / 8.99 dB → 54.0° / 8.95 dB. The shipped minima over the bounds runs, friction on and off: T-gantry 47.6° / 8.3 dB → 47.4° / 8.2 dB and 39.4° / 7.3 dB → 39.1° / 7.3 dB; single axis 48.8° / 10.4 dB → 48.6° / 10.4 dB and 38.5° / 9.7 dB → 37.8° / 9.6 dB; EXCT-30 47.7° / 9.2 dB → 47.3° / 9.1 dB and 39.2° / 8.3 dB → 38.6° / 8.3 dB. All above 35° and 6 dB, all valid. Updated in both guides (examples and text), the config reference, validation, the three configs' comments and the margins test's comment. The EXCT-30 worst-corner `dc-link-energy` row (fix 5). No other verdict moved.

Left open:

- Without the DC link simulated, the holding current's ½ L i² on Enable (about 1 mJ) comes from nowhere (`shortcut:` in `hold_here`); with it, it is taken from the link.
- The oscillation test of the loop measurement is a judgement (the rest of e above the injection's RMS); a loop that limit-cycles below that is measured as if linear.
- `fmi2GetBoolean` and `fmi2GetString` are still answered before initialisation (they read parameters, nothing that depends on the start time).

## Ninth review fixes (13 November 2026)

A review of the eighth round found the points below by experiment. Each fix has a test that was seen to fail with the fix taken out (the fix taken out of the file by hand, the test run, the file restored); the long-step test hung instead (stopped after 90 s). `cargo test --workspace --release`: 320 tests (315 before), all pass.

| Fix | What changed | Test |
| --- | --- | --- |
| 1 A conditionally stable loop read unstable | `loop_margins` counted every fall of the phase through −180° with \|L\| > 1, also one a later rise undoes: L = K (1 + s/a)² / (s (1 + s/b)² (1 + s/c)) (a, b, c = 2π · 300, 30, 3000 Hz; K = 3e5, 6.1e5, 1.2e6) read −53.7, −59.8, −65.7 dB and unstable, while its closed loops are stable. Now a Nyquist count: crossings of the negative real axis beyond −1 in frequency order, a rising one undoing the last falling one; the count starts at a phase of 0 at the lowest point's gain (the small circle around s = 0, −90° per integrator), so the shipped loops whose integrators lag past −180° below 10 Hz count that fall and the rise that undoes it. Stable when none is left; the gain margin is the worst falling crossing not undone (negative where \|L\| > 1). A rising crossing with nothing to undo (an unstable open loop) is not counted (`shortcut:` comment) | `a_conditionally_stable_loop_is_stable` (stable, 30.7°, 41.6°, 42.4° at 589, 1002, 1710 Hz, gain margin infinite), analysis |
| 2 Phase margin −360° off for a positive first phase | The first point was taken in (−360°, 0°], so a type-0 lead loop, L = 3 (1 + s/ω5) / ((1 + s/ω50) (1 + s/ω200) (1 + s/ω400)), read −336.0° at 1516 Hz. Now the phase is unwrapped freely and shifted by whole turns so that it lies in (−360°, 0°] at the first gain crossover (without one: the first point in (−270°, 90°]). With a point not measured the margins are NaN (they were read from the points left) | `a_lead_with_positive_phase_at_the_first_point_keeps_its_phase_margin` (24.0°), analysis; `margins_are_signed_and_say_an_unstable_loop_is_unstable` now also requires NaN margins for the invalid measurement |
| 3 FMU: a NaN start time, a huge step, an uppercase `--binary` | `fmi2SetupExperiment` took a NaN start time, after which every communication point passed (`(point - now).abs() > 0.5 dt` is false for NaN). It now refuses a start time, and a defined stop time or tolerance, that is not finite (`fmi-error-experiment`), and the time check is written so that NaN fails. A finite step of 1e20 s saturated `steps as u64` and ran for ever: more base steps than a u32 holds (about 3 days at 62.5 µs) are refused with `fmi-error-step` (message says so). `binary_path` compares the extension in any case and stores it lower case (`SIM_FMI.DLL` → `binaries/win64/sim_fmi.dll`) | `an_experiment_that_is_not_finite_is_refused`, `a_step_too_long_to_run_is_refused`, fmi; `a_binary_is_known_by_its_extension_in_any_case`, sim-fmi lib |
| 4 Deviation of a path out of the range | `path_deviation` reported a deviation for a path `plan_path` refuses with `error-path-range` (line, clockwise arc out of the range, line, 0.1 mm tolerance: `sim-cli analyze --path` printed the deviation, then the refusal). It now plans the path through `plan_path` first and refuses alike, for both planners; `sim-cli analyze` prints only the refusal | `path_error_on_a_circle_matches_the_hand_calculation` (the spline bulge and the reviewer's path, both planners: plan and deviation `error-path-range`), gantry |
| 5 Re-trips after a DC-link overvoltage trip | Documented, not changed: with an undersized braking resistor, re-enabling right after the trip trips again until the coasting axis has stopped (observed 7 to 57 re-trips over 6 to 60 ms, settling at the bus level). Physics, not a defect. Both guides' FMI section | — |

**What moved.** No shipped number: `sim-cli margins --bands` for the T-gantry, the single axis and the EXCT-30 gives the same CSV before and after, byte for byte. Only an invalid measurement now prints `NaN` margins (single axis at 0.3 N m s/rad: `NaN,NaN,NaN`, `FAIL` as before). Docs: the margin method in the model reference and both guides' margins tables, the FMI step rules and `--binary` in both guides, validation (rows and count).

Left open:

- `path_deviation` now plans the path as well, so the app and `sim-cli analyze` plan it twice; the time-optimal plan is the slower one.
- The no-crossover branch of the phase ((−270°, 90°] at the first point) is a judgement for loops with up to three integrators or a lead; a gain margin of a falling crossing with \|L\| < 1 that a rising one with larger \|L\| would undo is still counted (the lower bound of the true margin).

## Release basics (13 November)

| Item | What exists |
| --- | --- |
| Licence | MIT OR Apache-2.0: `LICENSE-MIT` (Copyright (c) 2026 Nico Schuermann), `LICENSE-APACHE`; `license = "MIT OR Apache-2.0"` in `[workspace.package]`, `license.workspace = true` in all nine crates. README section "Licence" |
| Version | `[workspace.package] version = "1.0.0-rc.1"` (was 0.1.0); all nine crates inherit it. The FMU's `generationTool` reads `sim-fmi 1.0.0-rc.1`, so the GUIDs change with it |
| Third-party licences | `THIRD-PARTY-LICENSES.md`, written by `tools/third_party_licenses.py` from `cargo metadata` for Windows and Linux: 546 crates, 31 licence expressions, none GPL, LGPL or unknown (the script fails on those). MPL-2.0: the nine `async-opcua` crates, only in `sim-opcua`; file-level copyleft, explained in the file and the README. `self_cell` is `Apache-2.0 OR GPL-2.0-only`, used under Apache-2.0. Fonts of `epaint_default_fonts` under OFL-1.1 and Ubuntu-font-1.0; ICU4X under Unicode-3.0 |
| Docs | `CHANGELOG.md` (Keep a Changelog: 0.1.0 phases 0 to 6, 1.0.0-rc.1 stages A to D, effect views, FMI), `CONTRIBUTING.md` (house rules), `ROADMAP.md` (from "Not built" and "Blocked") |
| CI | `.github/workflows/ci.yml`: Windows and Linux, Rust 1.98.1, build, `cargo test --workspace --release`, clippy without `-D warnings` (34 warnings on 13 November); Linux installs Bevy's system libraries. Not run yet: the repository has no remote |
| Packages | `tools/package.ps1` (zip) and `tools/package.sh` (tar.gz) build sim-app, sim-cli, sim-opcua and the sim-fmi library and pack them with README, CHANGELOG, licences, `docs/`, the three configs (`configs/t-gantry/default.toml`, `exct-30.toml`, `configs/single-axis/default.toml`) and THIRD-PARTY-LICENSES into `dist/` (ignored by git). Unsigned. The Windows one was run on a clean copy of the commit: 35 MB zip, the packed `sim-cli` lists the machines and writes an FMU. The CAD meshes are not packed (vendor CAD). No MSI (WiX is not installed). `package.sh` not run (no Linux here) |

**FMU checked with fmpy.** fmpy 0.3.32, installed into the project's `.venv` (Python 3.14). FMUs of the T-gantry, the single axis and the EXCT-30 (English and German), win64:

- `fmpy.validation.validate_fmu` (FMI 2.0 XSD, variable names, model structure, start values, causality and variability, units): no problem in any of the four.
- `simulate_fmu`, co-simulation at the default step (10 ms), no input: every output equals `sim-cli run` of the same config to the 9 digits the CLI prints (largest relative difference 5e-10), no log message.
- A setpoint ramp (X +0.1 m from 0.1 s to 0.6 s; EXCT-30 also Z −0.05 m from 0.2 s to 0.7 s; single axis X +0.2 m), against `sim-cli run --trajectory` with the same points every 10 ms, 1.5 s: end positions within 16 µm (T-gantry X 0.200039 against 0.199988 m, EXCT-30 X 0.199988 against 0.199998 m, single axis 0.299984 against 0.299985 m). On the way the two differ by design: the FMU's setpoint reaches its value one setpoint interval later and is interpolated linearly (`Command::Setpoint`), the trajectory's with cubic Hermite through the points (`Command::Trajectory`). With the CLI shifted by 10 ms the setpoints differ by at most 0.21 mm, positions by 0.35 mm (T-gantry), 0.26 mm (EXCT-30), 1.6 mm (single axis); peak torque differences 0.93, 0.56 and 2.6 N m (of 1.3, 2.2 and 2.8 N m): the ramp's corners are a speed step for the linear interpolation and smoothed for the Hermite one. Not an FMU defect.
- Parameters and inputs from fmpy: `effects.gravity = false` takes the Z holding torque away (motor b −0.143 → −0.010 N m); `command.enable` false at 0.2 s gives zero torque and the brakes hold Z; a step of 0.1 ms is refused with `fmi2Error` and the message logged; a setpoint out of range is a warning per step.
- Scripts, kept outside the repository (scratchpad): `validate_fmu.py`, `compare_fmu_cli.py`, `idle_check.py`, `api_checks.py`.

Left open: Linux CI and `package.sh` never run; clippy warnings not fixed; no signed installer; no import into Simulink, OpenModelica or Dymola.

## Temperature feedback, belt tooth jump, belt span vibration (13 November)

`cargo test --workspace --release`: 331 tests (8 new: 3 in mech, 5 in `sim-machines/tests/belt_and_heat.rs`; 323 before with the other session's), all pass.

Three new effects, each with its own switch, off by default, so every earlier result stays. The machine interface has two more methods with defaults: `after_step` (what happens at once after a base step, here a tooth jump; it reports what that took out of the energy among the loss powers) and `set_part_temperatures`. New config keys in all three configs, with notes and `[bounds]` for the estimates; labels English and German in the crates that introduce them.

| Effect | What was built | Tests (hand-calculated numbers) |
| --- | --- | --- |
| `thermal_feedback` | `ThermalFeedback` in `mech.rs`, table `[thermal_feedback]`: ambient 20 °C (estimate, 10 to 40), reference 20 °C (the temperature gearbox makers quote their no-load torque at), viscosity slope B = 4000 K (Andrade/Arrhenius, ISO VG 68; VG 32 to VG 220 give 3470 to 4730 K, the bounds), viscous share of the gearbox loss 0.5 (estimate, 0.25 to 0.75), belt stiffness coefficient 3e-4 per K (estimate, 0 to 1e-3). Guide viscous coefficient × ν(T)/ν_ref, Coulomb and stiction unchanged; gearbox 1 − η = (1 − η_cfg)(1 − s + s ν/ν_ref); belt EA × (1 − α (T_ambient − T_ref)), the belt at ambient (no loss of its own, and an EA that changed while running would change the stored energy). The feedback warms the parts as `part_heating` does (`sim::heats`), and `Sim` feeds the rises the last step ended with back before each step. It replaces `effects.temperature`: the hand-set factors are not used while it is on. `[thermal]` bounds run with either effect; the thermal checks are judged with either; the app's thermal view shows the parts with either | `the_temperature_laws_follow_their_formulas`, mech (0.281749 at +30 K, 4.73776 at −10 °C; gearbox 0.967956 and 0.856556; belt 0.994 at 40 °C); `with_thermal_feedback_friction_and_gearbox_loss_follow_the_temperature`, belt_and_heat (both machines at 30 K: guide viscous 0.281749 of configured, Coulomb unchanged, the 1.5 of `[temperature]` not used; gearbox loss in a cruise 0.64088 of the cold one, 1 %); `a_guide_settles_where_its_loss_and_heat_flow_balance` (single axis, 2 K/W at 1 m/s: rise = 2 (16.23 + 20 ν(rise)/ν_ref) settles at 40.18 K, 72.46 K without; 1 %); app: the thermal view lists the parts with the feedback alone |
| `belt_tooth_jump` | `ToothJump` in `mech.rs`, key `belt.tooth_jump_factor` = 2.4 (estimate, 1.2 to 4.8). Climb-out at a driven pulley when \|T_a − T_b\| > k z_m p / r · min(T_a, T_b), z_m = `ratings.belt_teeth_in_mesh`: the tension's radial pull T p / r holds each tooth, the pull times tan(effective flank angle) pushes it out, k = 1 / that tangent. 2.4 makes a pulley drive with 10 of 25 teeth jump at 1.5 times its pretension of effective pull. Per driven pulley six more states (slip u, its rate, direction, start, count, ridge force). While the teeth are out the belt (the mass the machine lumps on that pulley; needs `belt_mass`) slides on its own against a ridge F_s sin(2π d / p), F_s the climb-out pull, which is what the pulley feels meanwhile; it lands where it crosses the next groove (counted) or the one it left, belt and pulley sharing their momentum. Energy exact while sliding; what changes at once is `power.tooth_jump` over that step. Signals `pulley.<motor>.tooth_jumps`, `.tooth_slip`; check `belt-tooth-jump` (any jump fails). Elastic model only | `a_tooth_jump_starts_at_the_climb_limit_and_ends_one_pitch_on`, mech (climb at 6 × the slack side; ridge 600 N, 0.954930 J; landing at 0.793651 m/s); `the_belt_teeth_jump_at_the_hand_calculated_pull`, belt_and_heat (single axis upright at x = 0.1 m: W = 6 P / (1 + 6 L₂ / (L₁ + L₂)) = 280.24 N; no jump in a second at 98 %, at 102 % the first jump ends exactly one pitch on, −5 mm); `jumping_teeth_fail_the_check_and_close_the_energy_budget` (90 N pretension: 240 jumps, 11.3 J, budget within 0.5 %, measured 0.33 %, 0.21 %, 0.14 % at the step, its half and quarter; the default config: 0 jumps) |
| `belt_span_vibration` | `span_frequency`, `span_detuning`, `drive_excitations` in `mech.rs`; per machine `span_signals`. Each free span (single axis 3, T-gantry 8) a string, f1 = sqrt(T / μ) / (2 L) at its section's live tension (the belt speed's correction, below 0.2 %, left out); excitations: tooth mesh \|v\| / p per driven pulley and, with `torque_ripple`, the ripple and cogging orders. Signals `belt.span.<i>.frequency` and `belt.span_detuning` = min \|f1 − f_e\| / f1 (capped at 1). Check `belt-span-resonance`: the lowest detuning at steady speed (acceleration setpoints 0, a speed setpoint not), at or above `ratings.belt_span_resonance_band` = 0.1 (estimate). No amplitude: a lumped first-mode oscillator would need a sideways damping and a forcing that nothing here can be checked against | `a_span_vibrates_as_a_string_and_is_judged_against_the_excitations`, mech (109.545 Hz for 0.25 m at 300 N and 0.1 kg/m; 0.087129 against 100 Hz; 200 Hz mesh and 954.930 Hz ripple at 1 m/s and 250 rad/s); `a_span_vibrates_at_its_string_frequency_and_a_cruise_on_resonance_fails`, belt_and_heat (109.545, 21.066, 26.082 Hz at rest; T-gantry span 1; 1.109e-3 kg/m puts the 1.3 m span at 200 Hz, the cruise's mesh: fails; 0.1 kg/m passes with 1) |

**What moves in the shipped verdicts with each effect on** (`sim-cli verdicts`, the three configs):

- `thermal_feedback`: in a cycle of seconds the parts warm by fractions of a kelvin (time constants 20 to 40 min), and ambient = reference, so the verdicts move in the fourth digit at most (default T-gantry `motor-peak` b 0.1211 → 0.1212, `gearbox-continuous` a 0.7955 → 0.7954; single axis `gearbox-continuous` 0.8058 → 0.8056; EXCT-30 `braking-resistor-continuous` b 12.77 → 12.78). New rows, because the parts now warm: `gearbox-thermal` 1.080 K and `guide-thermal` 4.194 K (T-gantry), 2.076 K and 12.51 K (single axis), `guide-thermal` 23.68 K (EXCT-30), all ok.
- `belt_tooth_jump`: no tooth climbs out on any shipped config; every verdict is as it was, and the new row `belt-tooth-jump` is 0, ok, on all three. On the single axis the teeth hold down to 100 N of pretension (slack side 20.9 N against 119.5 N of pull); at 90 N they ratchet.
- `belt_span_vibration`: single axis and EXCT-30 pass with 1 (no span within 100 % of an excitation at steady speed). The default T-gantry FAILS `belt-span-resonance` with 5e-6: while its Z axis runs at 0.4 m/s, the tooth mesh at 80 Hz meets spans 4 and 5 (down to and up from the Z-bottom idler), whose frequency changes with z. A sweep in a cruise counts: it is slow enough for a lightly damped span to ring up. The default T-gantry is a placeholder machine.

Found on the way:

- The first jump model (the slip with zero force while the teeth were out, and a stall snapped back into a groove) created energy: snapping a belt back against the tension, and the belt's mass counted twice from the start of a slide. The ridge force, the inertia split between pulley and belt and the momentum-sharing landing make every landing lose energy; the budget then closed.
- `on_pulley = r · (belt torque / r)` differed from the belt torque by a rounding; the default T-gantry (which knocks through its gearbox backlash) then moved in the fourth digit with the jump effect on and no jump. Now the torque passes unchanged while the teeth are in mesh.
- The single axis's loss-free inclined config is chaotic (no damping, backlash knocks): with the new effects on, a tooth climbing out and falling back sent it another way, and the supply's step convergence failed. The new effects are on in the level elastic loss-free config instead (energy, step size and determinism of the standard suite hold there and on the T-gantry's loaded config); `the_supply_converges_with_the_integration_step_on_every_config` runs with them through its "every effect" config.
- `belt.tooth_jump_factor`'s bound stands among the other `belt.*` bounds: the app's save groups dotted keys by their table, and a key after the `ratings.*` bounds moved and shifted every later line.

Left open:

- The feedback over a cycle: the analysis runs from cold, so with the shipped time constants the cycle sees the parts at ambient; no "warm start" at the settled rises.
- One viscosity law for grease and oil; no no-load torque of its own for the gearbox, only its efficiency; the windings' feedback stays the one of `winding_heating`.
- The tooth jump law per pulley with the configured teeth in mesh, not the wrap of the pose; a tooth pressed back into its groove by the wrap's tension is not modelled; a jump is found at the end of a base step.
- Span amplitudes; harmonics n f1; the belt speed's correction.
- Not in the app beyond what is generic: no extra panel rows (the switches, signals, scopes and checks come through the generic lists; the check labels are in sim-components' .ftl).

## The inverse search made complete: drives, belts, axes, S120, gearbox mass, axis combinations (13 November)

`cargo test --workspace --release`: 341 tests (10 new: 9 in `sim-machines/tests/catalog_parts.rs`, 1 in `sim-app/tests/app.rs`; 331 before), all pass.

| Item | What was built | Tests (hand-calculated numbers) |
| --- | --- | --- |
| Drives by name | `[parts] drive`, `--drive`: an S210 converter (`converters_s210.csv`) or S120 Motor Module (`motor_modules_s120.csv`) sets `ratings.drive_rated_current_a` and `drive_max_current_a` (new keys in all three configs: E 5 A / 19 A, its converter 6SL3210-5HE11-5UF0, now named in E's `[parts]`; T S 3 A / 10.9 A, placeholders). New checks `drive-current` (cycle RMS current against rated) and `drive-peak-current` (new `MotorLoad::peak_current` against maximum); labels EN and DE in sim-components. The drive's current limit is judged, not simulated | `a_drive_by_name_judges_the_motor_current_at_the_hand_calculated_share` (3.2 A of 5 A = 0.64, 12.4 A of 19 A; both fail against 1.2 A / 4.2 A) |
| Pairing | `catalog::paired_drive`: D 32's converter column for S210; D 21.4's Motor Module for 1FK7 and 1FT7; for 1FK2 and 1FT2 on S120 the smallest Single Motor Module carrying the stall current. The search puts the paired drive into every candidate (column `drive`) | `s120_motors_by_name_...` (1FK7060-2AF7 -> 6SL3120-1TE15-0AD0; 1FK2103-2AH, 1.06 A -> 6SL3120-1TE13-0AD0; every S120 pairing exists and carries 85 % of the rated current) |
| Belts by name | New `catalog/megadyne/belts.csv` (43 rows: T5, AT3, AT5, AT10, MTD3, MTD5, MTD8, standard steel cords: width, g/m, allowable pull, breaking strength, stretch) and `belt_tooth_load.csv` (Fp spec over r/min), from Megadyne's MEGALINEAR handbook 02.2026 by `catalog/tools/extract_megadyne.py`. `[parts] belt = "AT5-25"`, `--belt`: EA = pull / 4 mm/m, mass, pitch, and where present belt tension rating, width and tooth table; widths between standard ones interpolated | `a_belt_by_name_is_the_megadyne_row_or_between_two` (AT5-25; MTD8-12 = 1425 + 0.4 * 710 N; AT10-10 = 2270 * 10/16 N), `megadyne_belts_are_plausible` |
| Axes by name | `[parts] axis`, `--axis`: `EXCT-15/-30/-100` (`festo/exct.toml`) and Bahr series-size (`bahr/belt_axes.csv`, not ELZU, not DLZPVI). Kind by keys (Z stroke: X/Z gantry; one stroke: single axis), so no machine is named. Sets pulley radius, belt (EXCT as AT, Bahr HTD as MTD; Festo's EA from its stretch), teeth in mesh (half the pulley), pulley torque rating, moved masses (Bahr gantry sheets; EXCT Z mass from the formula and carriage fitted with the machine to Festo's torque at 45 degrees). Strokes kept; EXCT gets the shortest offered Z stroke that holds the config's; wrong kind or stroke refused | `an_axis_by_name_gives_the_numbers_of_the_hand_written_exct_30` (all [catalogue] numbers of exct-30.toml; carriage 7.136 kg against 7.16 kg; Festo torque 2 %), `bahr_axes_by_name_bring_the_sheets_moved_masses_pulley_and_belt` (ELZI 40: 4.6 kg, 2.4 kg, 160 mm / 2 pi, EA 451250 N, 16 teeth, 25.46 N m) |
| S120 motors | `motor_supply = "s120"` / `--supply s120`: 1FK7 and 1FT7 (`motors_s120.csv` + data sheets; 3 x phase R and L, current loop retuned Kp = L / T, Tn = L / R) and 1FK2/1FT2 from their S120 manuals; DC link 540 V. `search --supply s120` (S210 stays the default) | `s120_motors_by_name_...` (1FK7060-2AF7: 4.05 ohm, 45.6 mH, Kp 182.4 V/A, Tn 11.26 ms; 1FK2103-2AH 4500 r/min) |
| Gearbox mass | New `catalog/siemens/gearbox_mass.csv` (445 rows) from the Siemens configuration manual 01/2023 A5E51318133B AC, Tables 7-12, 7-33, 7-54 (cache.industry.siemens.com), by `extract_siemens_gearbox_mass.py`; every one of the 774 gearboxes has a mass | `every_catalogue_gearbox_has_a_mass_and_bigger_ones_weigh_more` (NRK070-04-5 1.4 kg) |
| Cost stand-in and ranking | No public price (Industry Mall 403, no catalogue prints prices). `Candidate::mass` = motor + gearbox, `cost` = motors x mass + axis weight at the config's strokes; passing ranked by cost, then mass. CLI columns `axis,motor,gearbox,drive,mass_kg,cost_kg,...` | `the_search_ranks_three_passing_motors_by_their_hand_calculated_weight` (6.8, 10.4, 12.6 kg; 2.85 + 1.4 kg with NRK070-04-5) |
| Axis combinations | `catalog::search_axes`, `search --axes`: every axis of the machine's kind with every motor (and fitting gearboxes with `--gearboxes`), screened first by stroke, payload (Festo rated load; Bahr slide Fz against Z axis + payload) and speed/acceleration (`axis-stroke`, `axis-payload`, `axis-speed`, one row per ruled-out axis). Searches now run on all cores (motor search too); the CLI reports the run time | `the_axis_search_rules_out_by_catalogue_numbers_and_adds_the_axis_weight` (EXCT-15 stroke, ELZI 30 speed; 41.105 + 10.4 and 19.05 + 10.4 kg) |
| App | Six part fields (motor, supply, gearbox, drive, belt, axis), supply s120, an "Every axis of the catalogue" tick box, search table with axis, motor-and-gearbox mass and installed mass; "Use" names motor, supply, gearbox, drive and axis. EN and DE | `drives_belts_and_axes_are_named_in_the_app_and_searched_over`; `typed_part_names_...` and `the_app_searches_motors_...` updated |

Search run times (16 cores, release): EXCT-30 config, 28 1FK2: 0.3 s; S120 1FK7 (61): 0.7 s; `--axes` with all S210 motors at 400 V: 338 candidates (214 simulated, 124 screened) in 4.6 s; `--axes --supply s120`: 942 in 13.5 s; single axis `--axes`, all S210 motors on 56 Bahr axes: 4704 in 34.2 s. On the EXCT-30's cycle the Bahr ELZI 40 ranks first (19.05 kg axis), the EXCT-30 after it (41.1 kg); EXCT-15 has no 250 mm Z stroke and ELZI 30 allows 2 m/s of the config's 2.5 m/s.

Moved in shipped outputs: every `verdicts` table has the two drive rows per motor (E: 1.145 A of 5 A, 2.874 A of 19 A); the search's columns. The docs' examples were updated.

Left open:

- Prices: none public; installed weight stands in. Drive weights are not counted (not extracted).
- Belts are named, not searched; no guide catalogue.
- Axis search keeps everything the axis does not set (gearbox, friction, frame, shaper, gains): Bahr rows run on the EXCT-30's estimates. Bahr single axes have no moved mass; Bahr's maximum length is taken as the stroke limit; Bahr's belt EA is Megadyne's.
- The drive's current limit is not simulated; `--motor` does not bring the paired drive (the search does).

## Cycle geometry, belt tension in the plan, faster analyses (13 November)

`cargo test --workspace --release`: 345 tests (4 new: 3 in `machine-t-gantry/src/tests.rs`, 1 in `sim-app/tests/app.rs`; 341 before), all pass.

| Item | What was built | Tests (hand-calculated numbers) |
| --- | --- | --- |
| Lift heights as variables (C5) | `analysis::lifts` finds each lift, traverse and lower of the cycle (three segments: along the last axis away from both ends, across at that height, back along the last axis; the first two without dwell or gripper action). New key `profile.min_clearance_m` (all three configs: T 0.3, E 0.17 = the lifts as configured, S 0; label EN/DE in sim-components): the lowest height is the farther of pick and place plus it. The optimiser tries all lifts at their lowest first, then each alone at its lowest and halfway. `Optimised::lifts` reports them, `changes` carries the new cycle as `cycle.segment` (TOML), which `set_text` applies: the app's "Apply" writes it into the config; the CLI prints `cycle.segment[a-b-c].lift_m` rows, the app a row per lift ("Lift height, segments 2, 3, 4 [m]", `optimised-lift` / `optimised-via` in sim-components' .ftl) | `the_optimiser_lowers_the_lifts_to_the_clearance_and_saves_the_hand_calculated_time` (clearance 0.1 m: both lifts 0.35 -> 0.15 m; four jerk-limited Z moves, t = d/v + v/a + a/j = 0.8767 s and 0.3767 s, save 4 * 0.2 / 0.4 = 2.000 s, within 5 ms; the applied cycle runs to the optimiser's time); `the_app_shows_and_applies_lower_lifts` |
| Via points (C5) | `via = [[...], ...]` in a `[[cycle.segment]]`: the segment runs a path (`control::via_pieces`: a line to the first via point, a spline through the rest, a line to the target), planned by `path::plan` within the axis limits with the config's path tolerance, from where the segment starts; checked when the config is read (`cycle.segment[i].via`); it waits for the shaped setpoint and dwells as a move does, and does not blend. The optimiser, after every other step, rounds a lift through a via point at the middle, 1/2, 1/4, 1/8 of the width above the lift height, moves it by 1/8 and 1/32 of the width, or drops it; a candidate whose planned path comes nearer pick and place than the clearance (less the path tolerance) is not run | `a_cycle_segment_runs_its_path_through_via_points` (up, through (0.5, 0.3) and down without a stop, highest 0.29 to 0.35 m, arrives on (0.9, 0.05) before the dwell; a via point outside the stroke refused by name) |
| Belt tension as a limit of the time-optimal plan (C3) | `Machine::rigid_tensions` (T-gantry and single axis): section tensions of the loss-free rigid model, linear in the acceleration (the belt pulls the moved masses and keeps its length, sum (T_i - P) L_i = 0). Each section and pose is a row of the acceleration bounds with the room `profile.tension_reserve` (new, 0.8 in all three configs, label EN/DE) * min(T_max - c, c - T_min); `ratings.belt_max_tension_n` / `belt_min_tension_n`. A path whose holding tension alone is beyond a limit: `error-path-tension` (EN/DE). The reserve is needed: friction and belt vibration put the full model 15 to 70 N above the rigid tension (300 to 2000 m/s^3). No shipped example is belt-limited; their plans are unchanged (diffed) | `the_time_optimal_plan_keeps_the_belt_within_its_tension_limit` (line x 0.7 -> 0.95 at 300 m/s^3, 400 N: a = 0.8 * 115.53 / 6.681 = 13.84 m/s^2 within 0.5 %; rigid tension within 3 % of the 376.9 N allowed; full model 396 N, passes, within 3 % of 400 N; without the limit 19.3 m/s^2 and 425 N, fails) |
| Faster analyses | Profiled: integration 90 % of a cycle run (16 derivative evaluations per base step), tally 10 % (path error 6 %, machine signals per step 2 %, full sample per ms 1 %). Sensitivity on all cores (was one after another); tally samples only `base_signals` where that is all it reads (`Sim::sample_base`); path error skips setpoint pieces the tool is provably further from than the nearest so far (exact), and reuses storage; the app's bands and worst-corner search share their first 2 n + 1 runs (`verdict_worst_and_bands`) | `the_worst_corner_search_finds_the_hand_calculated_corner` (bands from the search == `verdict_bands`, to the bit); `the_app_searches_the_worst_corner_of_the_bounds`; outputs diffed byte for byte before and after (below) |

Run times, release, 8 cores / 16 threads, before and after alternately, three rounds each (s; median, then the three). The machine was shared with other work in rounds 2 and 3, so only the sensitivity's gain stands out of the noise; the single-cycle profile (one run, quiet) is the better measure of the rest: a T-gantry cycle 668 -> 640 ms, EXCT-30 215 -> 206 ms, single axis 94 -> 89 ms (-4 to -6 %).

| Command | Before | After |
| --- | --- | --- |
| `sensitivity t-gantry` | 151.5 (136.4, 151.5, 200.5) | 23.9 (12.4, 23.9, 25.4) |
| `verdicts t-gantry --config exct-30.toml --bands --worst` (2650 cycles) | 101.8 (61.8, 101.8, 111.7) | 74.2 (57.7, 74.2, 75.1) |
| `verdicts t-gantry --config exct-30.toml --bands` | 5.73 (2.80, 5.73, 6.33) | 3.09 (2.92, 3.12, 3.09) |
| `search single-axis --axes` (4704 candidates) | 61.3 (35.9, 61.3, 66.5) | 50.0 (62.2, 49.9, 34.0) |

No number changed: the four outputs of the old and the new binary are byte for byte the same (`cmp`; 94, 29, 29 and 4705 lines), and so are `analyze` of all three configs and the time-optimal path examples of the user guide. The old binary got the EXCT-30 file without the two new keys, which it refuses.

Left open:

- Limits that change within a piece (the classic point-by-point parameterisation with a jerk limit): not built; the plan keeps one limit per piece, so on a long line the belt limit is set by the line's worst pose (a line 0.1 -> 0.9 m at 300 m/s^3 reaches 349 N in the rigid model where it accelerates, of the 377 N the plan allows at its far end).
- Gearbox and pulley torque ratings are not limits of the plan (the gearbox output torque needs the pulley and rotor inertia split, which the planner does not get from the machine); judged afterwards as before.
- A rounded lift never won on the default T-gantry (300 runs, path tolerance 0 and 2 mm): an arch with vertical ends is slower than two stops with corner blending. The clearance is one height over the whole traverse, not an obstacle shape.
- The app's save writes a config whose cycle changed without its comments (an array changed, `saved_text`); the app's optimiser table rows use sim-components labels, no other UI was changed (the app is being redesigned in another session).
- Early stop of a failing candidate in the search: not built. On `search single-axis --axes` 189 of the 3279 simulated candidates fail, mostly on means (braking resistor) or in the dwell (settling), which a stop cannot judge early; at most about 3 % to gain, and failing rows would print other numbers.
- No cross-command cache of runs: within one CLI command nothing is run twice; resolving a config costs 0.05 to 0.3 ms of a 100 to 700 ms run.

## FMU check in CI, linux64 FMU, MSI and .deb (13 November)

| Item | What was built | Checked |
| --- | --- | --- |
| Independent FMU check | `tools/check_fmu.py`: fmpy 0.3.32's validator must find no problem, and every output of a co-simulation at the default step with no input must equal `sim-cli run` of the same config (relative to the output's peak, 1e-8; the CLI prints 9 digits). Exits 1 otherwise. A step of the `ci.yml` test job on Windows and Linux, for the three example configs | All three configs pass on Windows (win64) and in WSL Ubuntu (linux64), worst 5.0e-10 (`axis.x.position`). Seen failing: the EXCT-30 FMU against the default config, 7.2 relative (`motor.a.torque_inertia`), exit 1 |
| linux64 binary in the shipped FMU | `.github/workflows/release.yml` (on a `v*` tag): packages on Windows and Linux, then the Linux `sim-cli` writes one FMU per example config with `--binary sim_fmi.dll --binary libsim_fmi.so`, checks each with `check_fmu.py`, and attaches everything to a draft GitHub release | A two-binary FMU made in WSL passes `check_fmu.py` on Linux and on Windows. The workflow itself has not run (no remote) |
| MSI | `tools/installer.wxs` (WiX 5), built by `package.ps1` when `wix` is on the PATH: per machine into Program Files, Start menu entry for the app, `bin` on the system PATH. MSI version is the numeric part (1.0.0-rc.1 installs as 1.0.0). Unsigned | Built from a staged folder; an administrative extract (`msiexec /a`) has the expected files. Not installed (it changes the system PATH) |
| .deb | `package.sh`, when `dpkg-deb` is there: `/opt/kinematics-simulation`, links in `/usr/bin`, a desktop entry; version 1.0.0~rc.1. Built in a temp folder (on a Windows drive under WSL, dpkg-deb refuses the 777 modes) | Built in WSL with sim-cli and libsim_fmi.so and empty placeholders for sim-app and sim-opcua (sim-app did not compile at the time, another session was mid-edit); `dpkg-deb -I/-c` as expected. Not installed |

Left open: DC link capacitance and braking resistor from the catalogue (the S210 data is not extracted; D 32 is needed); package.ps1 and package.sh not run end to end with a compiling sim-app.

## Life factor a1 (13 November)

| Item | What was built | Tests (hand-calculated numbers) |
| --- | --- | --- |
| Life factor a1 of ISO 281 | New key `ratings.bearing_reliability_percent` (90 in all three configs, which leaves every result as it was; label EN/DE; config reference). `analysis::reliability_factor`: a1 = 0.95 · (ln(100 / R) / ln(100 / 90))^(2/3) + 0.05, the Weibull form behind the standard's table. `pulley_bearing_life_h` and `gearbox_life_h` (Siemens' figure is an L10 too) are multiplied by it; the guide life is not (its makers use their own factors). A reliability outside 90 to 99.95 % gives no life, and `verdicts` refuses it with `ratings.bearing_reliability_percent` | `the_lives_and_the_tooth_load_are_the_hand_calculated_ones`: a1 = 0.637912 at 95 %, 0.248332 at 99 %, 0.076832 at 99.95 % (the table's 0.64, 0.25, 0.077; worked out separately in Python), both lives scale by it, 1 without the key; `the_life_verdicts_judge_the_lives_against_the_required_life`: 99.99 % refused by name on every shipped config |

Left open: a_ISO (the configs have no lubricant viscosity, contamination or fatigue load limit); the motors' own bearings (Siemens gives their radial force limits only as diagrams in the motor configuration manuals, not in D 32); gearing fatigue above rated torque (the method is in the gearbox configuration manual, not in the repository). D 32 (07/2021 and 02/2026 editions, downloaded to look) has neither the S210 DC link capacitance nor its integrated braking resistor's ratings, only the limits for an external resistor.

## S210 braking data from the operating instructions (13 November)

| Item | What was built | Tests (hand-calculated numbers) |
| --- | --- | --- |
| `catalog/siemens/braking_s210.csv` | The internal braking resistor of each S210 (6SL3210) converter (continuous and peak power, braking energy) from Table 4-4 of the operating instructions 09/2022 (Table 3-2 of 06/2020, checked against the rendered page), and the trip level from the text of fault F07432 (410 V on 1 AC, 820 V on 3 AC). Typed in, no script | Its 12 converters are those of `converters_s210.csv` (in `a_drive_by_name_brings_its_published_dc_link_data`) |
| A drive by name brings its DC link data | `apply_drive(cfg, article, supply)` (the supply is new: the search sets it in `Parts`, not in the config). S210: `dc_link_overvoltage_v`, and `braking_resistor_continuous_w` / `_peak_w` except for a 3 AC converter on "230v" (Siemens requires an external resistor there; the config's values stand). A `braking_threshold_v` at or above the trip becomes (1.1 · line max · sqrt 2 + trip) / 2, an estimate; otherwise the config's estimate stays. S120 Motor Module: `dc_link_capacitance_uf` from `motor_modules_s120.csv`, half per axis for a Double. The search pairs every motor with its drive, so its candidates now carry these values | `a_drive_by_name_brings_its_published_dc_link_data`: EXCT-30 resolved 100 W, 5.7 kW, 820 V, threshold 770 V kept; 6SL3210-5HB10-8UF0 on 230 V 40 W, 2.4 kW, 410 V, a threshold of 470 V becomes 391.676 V, one of 400 V stays; 5HE11-5UF0 on 230 V keeps the config's resistor; 110 µF for 6SL3120-1TE15-0AD0, 220 / 2 for 6SL3120-2TE13-0AD0 |
| EXCT-30 | `braking_resistor_peak_w` 3000 → 5700 W (the converter's internal resistor; the continuous 100 W and the 820 V trip it had already); those three marked `[catalogue]` and their `[bounds]` removed (a named drive overrides them, so a bound would show no effect). Capacitance and threshold stay `[estimate]` with their bounds | |

Searched and not found (so they stay estimates): the S210's DC link capacitance and chopper threshold. Looked in catalogue D 32 (07/2021, 02/2026), the operating instructions (06/2020, 09/2022), the TED and Industry Mall data sheets of the converters, and SiePortal's specifications. D 32's "external braking resistor" table gives limits for a resistor one adds, not the internal one's ratings, and was not used.

## DC link capacitance measured on a real S210 (13 November)

| Item | What was built | Tests |
| --- | --- | --- |
| `catalog/siemens/dc_link_measured.csv` | The user traced a quick stop (OFF3) from 3000 rpm on a 6SL5310-1BE10-8DF0 (S210, new generation, firmware 6.4) with a 1FK2204-5AF on a belt conveyor: the DC link rose from 551 V to 632 V while the motor braked at 2.38 N m for 22 ms. Energy into the DC link 7.4 to 9.1 J less the copper loss (0.34 J per ohm; p0350 is not shown on the S210, 3 to 6 ohm taken): C = 139 uF, between 113 and 164 uF. The chopper was not reached (above 632 V). `apply_drive` sets `dc_link_capacitance_uf` for an article with a measurement | `a_drive_by_name_brings_its_published_dc_link_data`: 139 uF for the 6SL5310-1BE10-8DF0; the EXCT-30's 6SL3210-5HE11-5UF0, which has none, keeps 66 uF |

Also seen in the traces, not put into any config (the conveyor is none of the repository's machines): total inertia at that motor 2.2 to 2.6e-4 kg m^2 (rotor 1.23e-4), friction 0.24 N m + 0.0018 N m s/rad; on a gentle stop from 521 rpm friction took all of the braking energy (1 mJ fed back), so the DC link never matters on that axis. With the pulses off after the stop, the DC link fell at about 21 V/s near 630 V, much faster than `dc_link_discharge_time_constant_s` = 130 s implies; the trace was too short (0.37 s) for a time constant.

## Delta picker D3, rotary joint, paths in 3D (13 November)

The first of the further kinematics: a three-arm rotary delta (Clavel), sized like the Autonox DELTA RL3-1200-3kg with
the Siemens 1FK2105-4AF Autonox recommends. Equations and parameter sources: the research note of 13 November and the
[model reference](docs/model-reference.md#delta-picker-d3); config keys in the [config reference](docs/config-reference.md#delta-picker-d3).

| Item | What was built | Tests (hand-calculated numbers) |
| --- | --- | --- |
| Rotary joint component | `sim-components/src/joint.rs`: gearbox (stiffness, backlash, efficiency), motor friction, ripple, brake, joint friction (`effects.joint_friction`, `[joint]`), stops on the angle (`effects.hard_stops`); elastic and rigid methods, losses | `a_joint_adds_friction_stops_and_the_rotor_as_calculated` |
| Machine `machine-delta-d3` (id `delta-d3`) | Inverse (elbow out), forward (trilateration), Jacobian, its condition number (signal `kinematics.condition`), working range as Autonox draws it (cylinder with a spherical bottom) with refusals of their own (`error-delta-outside-envelope`, `-out-of-reach`, `-joint-stop`, `-singularity`); Codourey/Olsson dynamics with the forearm split as a parameter (2/3), elastic (`effects.joint_elasticity`, 15 states) and rigid (9 states) models, payload switching, scene. Config with every value `[catalogue]` or `[estimate]`, 26 bounds, and the Adept cycle 25/305/25 | `the_kinematics_round_trip_and_the_jacobian_is_the_derivative_of_the_forward_kinematics` (1e-12 m; J against central differences to 1e-7), `the_holding_torque_at_the_centre_is_the_platforms_weight_shared_out_and_the_arms_own` (−10.40 N m per gearbox), `the_holding_torque_at_the_edge_is_the_weight_through_the_jacobians_transpose` (−22.2 / −1.35 N m), `a_vertical_acceleration_at_the_centre_takes_the_hand_calculated_inertia_torque` (−0.6053 N m at the motor), `the_feedforward_with_the_speed_gives_the_asked_acceleration`, `the_adept_cycle_takes_the_hand_calculated_time` (0.70693 s continuous, 0.720 s on the 1 ms profile cycle; ABB 0.40 to 0.52 s printed beside it), `the_vertical_mode_under_constant_torques_is_unstable_high_up`, `targets_outside_the_working_range_are_refused_with_their_reason`, `the_cycle_is_judged_against_the_condition_number_limit`, `the_condition_number_is_the_ratio_of_the_singular_values`; the standard suite and every cross-machine test |
| Interface additions (defaulted, the gantries unchanged) | `Machine::unreachable(pose, slack)` (the controller refuses targets and checks every planned pose with it), `Machine::setpoint_slack()` (how far a setpoint at rest may be from the tool; 0.1 mm by default, 1.5 mm on the delta, whose gearboxes sag up to 1.01 mm at the edge of its range; since the tenth review fixes worked out from the sag, 2.30 mm with 9 arcmin), `Machine::feedforward_torque_moving` (with the tool's speed; the controller uses it, the delta adds its velocity term) | the standard suite's `round_trip` runs over the poses the machine can go to |
| Paths on three axes | `line,x,y,z`, `spline,x,y,z` (the spline code in three components), `arc,x,y,z,cx,cy,cz,cw|ccw` in the plane of start, end and centre, cw/ccw seen from +z, −y or +x. Not on three axes: transition curves (`path_tolerance_m` is not used), half and full arcs (start, end and centre on one line) | `arcs_on_three_axes_lie_in_the_plane_of_their_points`, `a_spline_on_three_axes_passes_through_its_points_within_the_limits` |
| Check `singularity` | The cycle's highest condition number against `ratings.max_condition_number`, for any machine that reports `kinematics.condition` | `the_cycle_is_judged_against_the_condition_number_limit` |

Findings:

- Held by constant torques the delta is unstable in its vertical mode in the upper part of its range (−1.68 N m/rad at
  the middle of the default range): the suite's loss-free configs put the range 175 mm lower, where it is stable.
- ~~The catalogue NRB080's 9 arcmin backlash let the arm that carries little of the weight float, and the tool rang ±4 mm at
  rest: the config takes 3 arcmin (a robot-grade gearbox, estimate).~~ Not found again (tenth review fixes): with 9 arcmin
  the tool stands still to 2e-8 m at rest at five poses. The config takes the catalogue's 9 arcmin since, and fails
  settling with it (a sizing finding, see there).
- The gearboxes' twist and backlash under the platform's weight put the tool off the motors' pose: with 3 arcmin 0.29 mm
  at rest at pick and place (0.33 mm was the dwell figure, with the motion before it) and up to 1.02 mm at the rim's lower
  edge; with 9 arcmin 0.64 mm and 2.09 mm. The encoders do not see it. Settling is judged against 0.5 mm, the
  positioning accuracy of the class, not the 0.1 mm repeatability.
- The velocity loop crosses over above the gearboxes' resonance (0.25 N m s/rad, 0.03 s, 0.1 ms filters: at least 39.6°
  and 8.2 dB across the bounds). Its margins cannot be measured at the far edge of the range, where one arm carries little
  of the weight; the start pose is above the place position for that.
- The tool's following error in the cycle is 1.2 mm at the motors with the velocity term in the feedforward and without
  (1.20 mm before, 1.23 mm after; on the PLC test's diagonal move 1.49 mm before, 1.40 mm after): it comes from the
  gearboxes' springs, which the rigid feedforward leaves out.

Cross-machine tests that assumed belts or guides now ask the machine (`belt_sections`, `thermal_parts`,
`friction_contacts`, the config's tables, a linear transformation for replay and fit, `setpoint_slack`) instead of
assuming. Left open: the friction view for joints (N m over rad/s), replay and fit for a kinematics that is not linear,
the forearms' inertia and the links' elasticity, D2, D4 and the linear delta.

## Tenth review fixes (13 November 2026)

Two independent reviews found the points below by experiment. Each fix has a test that was seen to fail with the fix taken out (the fix taken out of the file by hand, the test run, the file restored), except where the row says otherwise. `cargo test --workspace --release`: 374 (6 new: 1 gantry, 3 delta, 1 life, 1 app) tests (368 before), all pass.

| Fix | What changed | Test |
| --- | --- | --- |
| 1 The time-optimal plan passed the belt's rating on the full model | The review took the full model's excess over the rigid tension to be roughly absolute (15 to 70 N). Measured, it is not only that: after a quick ramp of acceleration the belt rings (about 18 Hz on the default T-gantry) by up to the rigid model's change of tension itself, and on top comes the guide friction the loss-free rigid model leaves out (with `belt_slack` off the slack side went to −117 N where the rigid plan kept about 100 N). So both: new key `profile.tension_margin_n` (the X guide's coulomb plus viscous friction at the axis' top speed: T 31 N, E 70 N, S 35 N, D 0; label EN/DE; config reference) comes off both ratings, and `profile.tension_reserve` goes 0.8 → 0.5 in the three belt configs (a step on a lightly damped mode overshoots by up to itself). `error-path-tension` names the margin (EN/DE). The reviewer's cases now: 330 N (default limits) and 340 N (40 m/s², 2000 m/s³, 2 m/s) refused, the holding tension (up to about 310 N) leaving no room; 360 N → 325.7 N, 400 N → 344.4 N, 1500 N → 43.4 to 428.5 N (slackest side 43 N, was 0 N). Swept 100 to 5000 m/s³ and 340 to 1000 N on two paths: no rating passed. A judgement backed by those runs, not a proof (quick ramps one after another can add their swings). The user guide's corner at 2000 m/s³ is now belt-limited: 0.78 → 0.80 s plus rest (it went slack before); no other shipped example moved | `the_full_model_keeps_the_belt_ratings_on_a_time_optimal_plan_at_high_jerk` (new, gantry; seen failing with 0.8 and no margin: 331.40 N on 330 N); `the_time_optimal_plan_keeps_the_belt_within_its_tension_limit` (hand calculation now 0.5 · (400 − 31 − 284.47) / 6.679 = 6.328 m/s², full model 358 N) |
| 2 S120 drives judged against the S210's internal resistor | `apply_drive` for a Motor Module: `braking_resistor_continuous_w` 1500, `_peak_w` 100000, `braking_threshold_v` 770 from the Braking Module row of `braking_s120.csv` (6SL3100-1AE31-0AB1: 1.5 kW, 100 kW, 770 V, checked in the file), `dc_link_overvoltage_v` 820 V (the booksize units' trip, S120 manual; not in the repository's catalogue files, a constant in `catalog.rs`). The reviewer's command: limits 1500 W and 100000 W, 770 V of 820 V. The S120 search example of the guide is unchanged. Guides' Drive row EN/DE | `a_drive_by_name_brings_its_published_dc_link_data` (both S120 modules; seen failing: 100, 5700) |
| 3 `sensitivity` varied the bearing reliability to 72 and 108 % | Left out in `analysis::sensitivity` (`NOT_MACHINE_DATA`: a choice of how to judge, with a range of its own). Guides EN/DE | `sensitivity_leaves_the_bearing_reliability_alone` (life; seen failing: (72, 108)) |
| 4 A first arc from rest refused on the delta | The planners take the slack for a first arc as an argument (`path::plan`, `deviation`, `plan_time_optimal`, `deviation_time_optimal`); the controller passes `Machine::setpoint_slack()`, a stated start `path::ARC_SLACK` (0.1 mm). `FIRST_ARC` is at least ten times the slack. The reviewer's arc runs; it ends 0.60 mm beside its stated end (the sag at the start pose), which the deviation reports | `a_first_arc_from_rest_is_laid_out_with_the_machines_slack` (delta; seen failing with `ARC_SLACK` passed) |
| 5 The setpoint slack applied to every planned pose | `control::near_start`: a pose within the slack of where the path starts keeps the slack; any other has to be in the range, except that on an axis whose start lies beyond an end it may stay as far beyond as the start is (a gantry path along an end it stands beyond), and in the envelope (with the slack only if the start is outside it). Used for plans, trajectories and via-point segments. And the slack itself: the delta works it out when built (`Delta::rest_sag`: each gearbox's twist under its holding torque plus half its backlash either way, all eight combinations, through the forward kinematics, over a grid of the range, payload on and off, plus a tenth): 2.30 mm with 9 arcmin (it was a fixed 1.5 mm; the simulation at rest shows up to 2.09 mm at the rim's lower edge between two arms) | `a_path_may_not_use_the_setpoint_slack_away_from_its_start` (0.6014 refused as the move is, both planners and a trajectory; seen failing: accepted), `the_setpoint_slack_covers_the_sag_at_rest` (five poses; seen failing with 1.5 mm) |
| 6 The delta's backlash | No limit cycle with the catalogue's 9 arcmin: peak-to-peak below 2e-8 m at rest at five poses, as the reviewer found. The config takes the NRB080's 9 arcmin. **Sizing finding: `settling` fails** (0.677 mm against 0.5 mm; over the bounds 0.624 to 0.736 mm, all FAIL); static sag at pick and place 0.64 mm. With 3 arcmin it passes (0.29 mm static, 0.33 mm in the dwell). Other numbers: arm 1 peak 7.786 N m (was 7.662), RMS 2.645 N m, gearbox peak 136.6 N m (bands 127 to 146), cubic mean 68.7 N m, gearbox life 613609 h; loop margins at least 47.0° / 8.6 dB with friction, 41.7° / 8.1 dB without (the config's comment had 39.6° / 8.2 dB). Wording corrected: the 0.33 mm was the dwell figure; static at pick and place it is 0.29 mm (3 arcmin). The rim: at its lower edge (z = −0.874 m) 0.45 to 1.02 mm with 3 arcmin over 12 directions, so the 1.01 mm stands there; the reviewer's 0.84 to 0.90 mm are from higher up. Updated: config comments, guides EN/DE (Size a delta picker, first arc), config and model reference, the delta findings above | the settling FAIL is shown in both guides' example; `a_path_from_the_plc_is_planned_and_run` takes the end within the larger of settling tolerance and setpoint slack (commented) |
| 7 Flaky `a_path_from_the_plc_is_planned_and_run` | The plans are compared (time-optimal below 0.95 of the other); each run's busy time against its own plan within the largest polled `signal.time` step plus 1 ms; busy and time read in one request (read apart, a slow answer put up to 24 ms between them); at least half the polls the planned time takes at the mean poll step; refusals and busy polled for up to 5 s instead of fixed sleeps. Loops below | (the test itself) |
| 8 Flaky `the_session_runs_in_real_time_and_can_pause_and_slow_down` | Only upper bounds and order: sim time ≤ wall + 0.05 s at speed 1, ≤ 0.1 wall + 0.05 s at 0.1, nothing while paused; up to 10 s for the first sample, for time after the pause, and for the refusal of an empty move (that one failed once in the first quiet loop) | (the test itself) |
| 9 `fit` and `replay` on the delta printed numbers without meaning | `trace::linear_transformation` (the tests' check, now over every axis); `trace::replay` (so `fit_rigid` and the app) refuses with `error-trace-not-linear` (EN/DE), the CLI's `replay` and `fit` before reading any file. The two tests use the library check | `a_trace_from_run_replays_onto_itself_and_fits_its_own_config` (cli: replay, fit --trace, fit --sweep refused on the delta in EN and DE; seen failing: it went on to read the trace file instead of refusing) |
| 10 Comment on `to_low_end` | It moves the gantries 50 mm (0.1 → 0.05 m), not "no length": control.rs, review2.rs, review3.rs | — |
| 11 `step --axis 1` taken as a catalogue axis | `step` and `bode` take `--axis-index N`; `--axis` stays the catalogue's on every command. Usage EN/DE, guides EN/DE | `step_bode_and_tuning_write_csv_for_every_machine` (the last axis by `--axis-index`, steps unlike the first; without the fix `--axis-index` is an unknown option, not run separately) |
| 12 A delta config in the app's screenshot hook | `Model::open_config` opens a config of another machine (one the shown machine refuses and another reads) on that machine; the "Open" action also forgets the picked part. `SIM_APP_MACHINE=<id>` switches machine before `SIM_APP_CONFIG`. Guides EN/DE | `a_config_of_another_machine_opens_that_machine` (app; seen failing: stayed on the first machine) |

**What the delta's 9 arcmin moved in other tests.** Tests that took every shipped cycle to pass, or the tool to rest within 0.1 mm of its setpoint, now ask the library or the machine: the CLI's and the app's verdict tables are compared with `verdicts` (ok or FAIL as judged), the app's optimiser expects its refusal for a cycle that fails a verdict, rest positions are judged within the larger of 0.1 mm and the machine's setpoint slack (`a_move_arrives_and_comes_to_rest`, the PLC move, the app's path trail), the cruise following error within 1 % plus the sag at rest, a 1 % ratio change within a tenth of the slack (0.13 mm on the delta), the scene's rpm label above 10 rpm instead of 50 (43 rpm on the delta at that moment). Replay tests skip or expect the refusal on the delta. Found on the way: a band whose runs all failed had a share of −5e-16 (1 − n · 1/n in floating point); `bands_of` now counts the failed runs and divides once.

**Flaky-test loops** (release, 10 runs each, alone and with `sim-cli sensitivity t-gantry` running on all cores beside them). First round, with fixes 7 and 8 as the review asked: alone PLC 10/10, app 9/10 (the empty move's refusal read after a fixed 100 ms); loaded PLC 5/10 (refusals read after fixed 100 ms sleeps, and busy and time read in two requests: 24 ms apart once). After polling those and reading busy and time together: alone PLC 10/10, app 10/10; loaded PLC 10/10, app 10/10.

Left open:

- The tension margin and share are a judgement backed by sweeps of the default T-gantry, not a bound: the plan has no belt frequency to compute the swing from; an elastic estimate of the plan (a mode filter on the rigid tensions) would need one per machine and pose.
- The S120 trip level 820 V is typed from the manual into `catalog.rs`, not extracted into a catalogue file.
- The delta with the catalogue gearbox fails `settling`; no low-backlash gearbox is in the catalogue to put in its place.

## Obstacles and crash detection (13 November)

| Item | What was built | Test |
| --- | --- | --- |
| Config | `obstacles`: a list of boxes along the axes of the 3D view (`name`, `centre_m`, `size_m`), `[]` in all four configs, optional in a config file; an entry with another key, a size of 0 or not three numbers is refused (`obstacles[i]`). No second clearance key: `profile.min_clearance_m` was the optimiser's lift above pick and place, "an obstacle there"; it is now the one clearance the machine keeps, from the modelled obstacles too. Its values stay (T 0.3, E 0.17, S 0, D 0.025) | `a_bad_obstacle_is_refused` |
| Collision bodies | `Machine::collision_bodies(state)`, by default the tool as a point at `scene_point`; `scene::scene_boxes` takes named boxes of the machine's own scene. T: Z axis, flange, payload; S: carriage, payload; D: platform, payload (no forearms). `Aabb::gap`: nearest points and clearance, negative depth in an overlap | `gap_between_boxes_by_hand` (sim-core) |
| Checks | Before a run: `collision::check_path` (every mm, machine at rest at each pose) and `check_cycle` (start, each segment with the payload as the segment before left it, round again; via points as planned; blending not followed). While it runs: every base step in `Sim::step`, only with obstacles. Check `collision` (in `AT_LEAST`): least clearance of the run against `profile.min_clearance_m`, failing at contact; not made without obstacles | `a_box_beside_a_straight_move_keeps_the_clearance_by_hand` (0.13 m by hand, planned and run), `a_box_under_the_traverse_is_judged_by_the_collision_check` (0.048 m by hand on the T-gantry; fails at 0.3, passes at 0.04; no check and no change on any machine without obstacles) |
| Crash | `effects.crash`, off: on contact the machine is put at rest at the tool's pose and the drives go off (plastic impact on a rigid obstacle; a contact spring would need a stiffness, a damping and a smaller step). Contact time, pose, impact speed and obstacle in `Sim::contact` and `CycleLoad::contact`, with the crash off too | `a_move_into_a_box_stops_at_its_face_at_the_planned_speed` (face at x = 0.47 by hand, 1 m/s planned and at impact, stopped within one step's travel; through it without the crash) |
| Optimiser | A time-optimal path nearer than the clearance is refused (`error-path-collision`); `optimise` judges candidates by the verdicts, so `collision` keeps its lifts off the obstacles | `a_time_optimal_path_into_an_obstacle_is_refused` |
| Front ends | CLI `run` and `verdicts` say the planned least clearance and first contact pose, `run` the run's, on the error stream; the verdict table has the `collision` row. App: obstacles as see-through wire boxes, red in contact; the clearance as a red line with an mm label among the view's labels; the first contact as a red mark on the timeline. Labels EN/DE in sim-components, sim-cli and sim-app | the workspace's key and string checks |

Left open:

- ~~The app draws the obstacles as see-through wireframe boxes (gizmos), not filled translucent meshes.~~ Filled (app views, 13 November).
- ~~The crash's kinetic energy is not booked as a loss; the energy budget of a run with a crash does not close.~~ Booked (eleventh review fixes), with the bleeder loss of the drives switched off.
- ~~The live check builds the machine's scene every base step while obstacles exist: 4 s of the T-gantry's cycle take 0.61 s instead of 0.23 s on the CLI (the planned check included). Without obstacles nothing changes. Worth a cheaper body list per machine if obstacles are used in searches.~~ The machines build only their collision boxes now (eleventh review fixes).
- The delta's forearms are not collision bodies; turned parts count as their unturned boxes.

## Eleventh review fixes (13 November 2026)

A review confirmed the points below by experiment. Each fix has a test that was seen to fail with the fix taken out (the fix taken out of the file by hand, the test run, the file restored), except where the row says otherwise. `cargo test --workspace --release --exclude sim-py`: 414 tests (4 new: delta rim arc, CLI axis index, S120 line-up, collision bodies; the crash budget and tightest-check cases extend existing tests), all pass; one run had `a_config_file_is_opened_changed_saved_and_opened_again` (app) fail on `wait_for_data` timing out under load (another test process at full CPU), which passed 3 of 3 alone and in the app suite again. `sim-fmi`'s `the_guid_changes_with_the_model_description` took the first boolean of the config for an effect switch; it now takes an effect (`drive.braking_module` sorts first).

| Fix | What changed | Test |
| --- | --- | --- |
| 1 A delta path along the rim of its range refused by rounding | `control::near_start` allows a pose away from the start 1e-9 m (`ROUNDING`) beyond the axis range and the envelope (`unreachable`). The reviewer's `line,0.6,0.0,-0.775` then `arc,0.0,0.6,-0.775,0.0,0.0,-0.775,ccw` runs again on `analyze delta-d3 --path` | `an_arc_along_the_rim_of_the_range_is_planned` (delta, both planners; seen failing with `ROUNDING` 0) |
| 2 PLC tests took the end within 2.3 mm on the delta | The three tests read the controller's setpoint (`signal.axis.<id>.position_setpoint`) and want it on the target to 1e-6 m; the tool's actual position within 1.2 times the sag at rest at that pose (0.5 s at rest with it as the start pose), at least 0.2 mm (`at_rest_within`). On the gantries that is the 0.2 mm, on the delta the sag of the pose, not the 2.3 mm slack | `a_plc_commanded_move_runs_and_reports_position_and_following_error`, `a_path_from_the_plc_is_planned_and_run`, `plcopen_power_home_and_moves_reach_their_targets` (each seen failing with the commanded target, or the path's end it is judged against, 1 mm off: "set to 0.30099 m, target 0.29999 m") |
| 3 Guides said poses up to 0.1 mm outside the range are taken | The limits bullet and the `--trajectory` bullet, EN and DE: only near its start may a path leave the range, by the machine's setpoint slack (0.1 mm gantries, 2.3 mm delta); the first pose of a trajectory within that slack of the setpoint | — |
| 4 German `error-path-tension` named the margin "Reserve" | "abzüglich des Abstands (profile.tension_margin_n)", as its label | — |
| 5 `step`/`bode --axis-index` took -1 as 0 and 1.7 as 1 | `axis_index` in the CLI reads a whole number below the machine's axis count; otherwise `cli-error-axis-index` (EN/DE) names `--axis-index` and the highest index | `a_bad_axis_index_is_refused` (cli; -1, 1.7 and one past the last axis, both commands, both languages; seen failing: -1 ran) |
| 6 S120 axes each judged against the whole Braking Module | New key `drive.braking_module` (all four configs off, optional in a config file, label EN/DE, config reference): the axes brake through one resistor for all. `apply_drive` sets it for an S120 Motor Module and clears it for an S210. `verdicts` then judges `braking-resistor-continuous` and `-peak` once, without a motor: the mean of the summed braking power and the peak of the sum step by step (`CycleLoad::braking_power_peak`: the resistors' power with the DC link simulated, a shared one once, or the power fed back without). The search's "tightest check" takes such a verdict as backing every axis's DC link (without that the S120 search example of the guide showed `dc-link-energy` 1.0000; it is unchanged now). Guides' Drive row EN/DE | `an_s120_line_up_is_judged_against_its_one_braking_module` (catalog_parts; two axes each 900 J in 1 s, 60 kW at once: by hand 1800 W against 1500 W and 120 kW against 100 kW, both FAIL; the same load on the EXCT-30's S210 per axis; and a run's summed peak between the highest axis peak and the sum of the peaks; seen failing: per axis, 900 W and 60 kW, ok). `the_tightest_check_leaves_out_a_dc_link_that_a_resistor_backs` (one more case; seen failing: dc-link-energy 1.0) |
| 7 Comments | The gantry test's doc says 330 to 1500 N (330 N at the default limits); the delta config says `sim-cli margins delta-d3` | — |
| 8 Crash energy not booked | `Sim::crash_energy`: what each stop at an obstacle took (`Machine::energy` before less after), budget row `crash` with effects.crash ("Crash"/"Aufprall", sim-components, app key list). That alone left 0.074 J of 22.7 J (0.32 %) open on the move into a box: the DC link of the drives, off from the stop on, discharges through its bleeder and the rectifier draws it back, a loss no row had (on any run with the drives off: `--disable-at` as well). `Drive::discharge_power` and budget row `discharge` ("DC link discharge of drives switched off"/"Zwischenkreisentladung abgeschalteter Antriebe"), counted in `electrical_unaccounted`. Guides' budget table and example, model reference | `a_move_into_a_box_stops_at_its_face_at_the_planned_speed` (now also: budget within `BUDGET_TOLERANCE`, 0.0016 J of 22.7 J, and the electrical side within `ELECTRICAL_TOLERANCE`; seen failing without the `crash` row, and without the discharge: 0.0738 J of 22.73 J) |
| 9 The live collision check drew the whole scene every base step | `scene::scene_boxes` takes parts; each machine builds only the parts it can hit with: T-gantry `hanging` (Z axis, clamp, flange, payload, the bottom idler, leaned with tool flex; plus the frame's sway), single axis `on_carriage`, delta `on_platform`, which `scene` draws too. `sim-cli run t-gantry --cycle --time 4 --every 100000` with the reviewer's box under the traverse: before 1.67 s, after 0.89 s, without obstacles 0.55 s (the least of 7 runs each, on a machine with another test process running; the planned check `check_cycle` included). The CSV and the reported clearances (48.0 mm planned and run) are the same byte for byte | `the_collision_bodies_are_boxes_of_the_scene` (every machine, during its cycle, with and without the payload: each body is a box the scene draws, to the bit; seen failing with the T-gantry's sway left out); the existing collision tests unchanged |

Left open:

- The S120 line-up's Braking Module is judged against the summed braking power, but with `effects.dc_link` and `shared_dc_link` off each axis still simulates a DC link and resistor of its own; `shared_dc_link` is not switched on by an S120 drive.
- The PLC tests' bound on the delta rests on the sag at rest at the pose, which the backlash side of the approach can change; 1.2 times it held on every run here.

## Faster tests (13 November 2026)

The suite runs under [cargo-nextest](https://nexte.st): every test in a process of its own, the tests of all binaries at
once, where `cargo test` ran the binaries one after the other. `.config/nextest.toml`: a test running over 60 s is
reported as slow; no retries; the tests against the wall clock (`sim-opcua` plc and soak, `sim-app` app) form the group
`real-time`, at most 4 at a time; the eight longest tests start first. CI installs nextest with
`taiki-e/install-action` and runs the doctests on their own (`cargo test --doc`; there are none yet). CONTRIBUTING shows
the commands.

Measured on the 16-core development machine, release, warm build (nothing to compile), 414 tests:

| | Before (`cargo test`) | After (`cargo nextest run`) |
| --- | --- | --- |
| Build and tests, wall clock | 9 min 53 s | 4 min 24 s to 4 min 57 s (two runs) |
| Tests alone | 580 s (the binaries' own times added up) | 263 s to 293 s |

The suite is now limited by processor time, not by waiting: two tests that sweep every bound use all cores between
them, and the run ends when the longer one does. Shortened, each without a weaker assertion:

| Test | Change | Alone, before → after | How it was shown to still catch its defect |
| --- | --- | --- | --- |
| `every_effect_switch_changes_the_traces` (sim-machines control) | Each flipped run is compared sample by sample with the reference and stops at the first difference, instead of recording 4 s and comparing at the end; same samples, same 4 s for a switch that changes nothing, the same answer to the bit | 34.5 s → 1.3 s | `effects.torque_ripple` read as always on in `Ripple::from_config`: fails "t-gantry: flipping effects.torque_ripple changed nothing"; restored |
| `the_shipped_velocity_loops_keep_their_margins_across_the_bounds` (sim-machines control) | In the pass with friction off, the bounds of the friction values (`friction_*`, `motor.friction_*`, `joint.friction_*`) are left out: they act on nothing there | 128 s → 111 s | A run of every bound on all four configs, both passes, compared each run's margins with those of the config as it is: all 68 friction runs of the pass without friction were the same to the bit (and no other friction run was left out) |

The real-time waits of the app tests poll with a bound of 60 s instead of 5 s (`wait_for_data`, the refusals of
`editing_a_parameter_reaches_the_running_simulation`, the trail of `a_path_is_planned_run_and_analysed_in_the_app`), and
two fixed sleeps before an assertion that new samples arrived became polls (`editing_a_parameter...`, the frozen scopes
of `scopes_have_a_window_freeze_statistics_and_export`). They return as soon as the data is there; only a failing run
waits longer. `a_config_file_is_opened_changed_saved_and_opened_again`, which timed out once under load, waits like that.

The 15 slowest tests, each alone (`--test-threads 1`; most of them use every core themselves):

| Test | s |
| --- | --- |
| sim-machines control `the_shipped_velocity_loops_keep_their_margins_across_the_bounds` | 128 (111 now) |
| sim-machines catalog `the_exct_30_config_passes_settling_at_the_worst_corner_of_its_bounds` | 123 |
| sim-cli `step_bode_and_tuning_write_csv_for_every_machine` | 73 |
| sim-app `the_app_optimises_the_cycle_and_applies_the_settings` | 39 |
| sim-machines control `every_effect_switch_changes_the_traces` | 34 (1.3 now) |
| sim-machines effects `the_cheap_signals_and_the_lean_analysis_change_no_number` | 34 |
| sim-machines effects `the_part_thermal_verdicts_judge_where_the_parts_settle` | 31 |
| sim-machines sweep `the_default_sweep_finds_the_lowest_dip_on_every_config` | 30 |
| sim-cli `verdicts_search_the_worst_corner_of_the_bounds` | 29 |
| sim-app `the_frequency_response_is_measured_in_the_app` | 28 |
| sim-app `the_checks_get_error_bands_in_the_app` | 27 |
| sim-app `the_app_searches_the_worst_corner_of_the_bounds` | 24 |
| sim-app `the_app_shows_and_applies_lower_lifts` | 23 |
| sim-opcua soak `the_link_runs_move_cycles_for_a_long_time_without_errors_or_drift` | 23 |
| sim-app `a_path_is_planned_run_and_analysed_in_the_app` | 22 |

Left as they are, and why:

- The worst-corner search of the EXCT-30 (about 2400 cycles) and the margins over every bound (about 830 loop
  measurements of 60 frequencies each) are what they test; fewer runs or frequencies would test less. Of the margin
  runs, 174 of 468 give the margins of the config as it is to the bit (ratings, hard stops, the brake, process forces,
  cable chain, idlers, noise: values the loop never meets at standstill), but which ones is known only by running them.
  Leaving them out of `analysis::bounds` (as `PATH_EFFECTS` does for values of effects that are off) would also speed up
  the app's bands; that is a change to the product, not to a test.
- `step_bode_and_tuning...`: 25 of its 73 s are `sim-cli tuning` (17 s on the delta), which has no shorter mode.
- The soak test runs 20 s by design (`SOAK_SECONDS`).
- The effects tests repeat one cycle analysis between them (`part_heating` on, every config); sharing it would mean
  merging two tests. They are single-threaded and run beside the others, so they do not lengthen the suite.
- The dev profile already builds with `opt-level = 1` and the dependencies with 3, so nothing was added for quick
  debug runs (not measured here: a debug build of the whole workspace with Bevy takes several minutes).

## Twelfth review fixes (13 November 2026)

A review confirmed the points below by experiment. Each fix has a test that was seen to fail with the fix taken out
(the change undone in the file, the test run, the file restored), except where the row says otherwise.

| Fix | What changed | Test |
| --- | --- | --- |
| 1 After a crash the machine could never be powered up again | The stop left the carriage up to one step's travel inside the obstacle (clearance -26.8 um), and every step after `Enable` found the contact and tripped the drives again. `Sim::check_obstacles` now stops a part only when it moves further into an obstacle: its clearance more than `CREEP` (1 um, in all while in contact) less than where it was judged from; a part that stands or moves out is left to move. The stop puts the machine at the pose where the clearance reached 0, linearly along the step that went in, so at the face (0.7 to 1e-9 m on the single axis), or back where it stood if it was in contact already. `CREEP`: with a strict comparison, drives enabled at rest against the wall tripped after 69 steps on 4.4e-16 m of rounding, and they settle some 25 nm in. `Sim::crashed` says the drives went off at an obstacle (until they are enabled again). PLCopen: ErrorID 8 (`ERR_CRASH`) for a crash instead of 7, and a setpoint streamed in the tick in which the drives went off is no longer refused as ErrorID 6 (seen once: the stream ran before the check of the drives); MC_Reset switches the drives on again as after 7. FMI needs nothing: `command.enable` held true switches the drives on at the next communication step, and they now stay on unless the setpoint drives the tool further in. Guides EN/DE, PLC pages EN/DE, model and config reference | `after_a_crash_the_drives_are_enabled_again_and_move_out` (single axis: the reviewer's wall, 1 s, `Enable`, 0.25 s at rest, `MoveTo(0.1)` reaches it, `MoveTo(0.9)` stops at the face again and trips; seen failing with the old stop: "the drives tripped at rest at 0.4700268"). `plcopen_a_crash_is_reported_reset_and_left` (sim-opcua plc: ErrorID 8, MC_Reset, Standstill, MoveAbsolute to 0.1 within 0.2 mm; seen failing with the old stop, Power.Status never true after the reset, and with ErrorID 7 reported). `a_move_into_a_box_stops_at_its_face_at_the_planned_speed` now wants the face to 1e-9 m (seen failing with the stop at the pose of the step: 0.7000268) |
| 2 Obstacles stopped the machine only while its drives were on | The stop acts with the drives on or off; drives that were on go off. Its energy is booked in the `crash` row as before | `obstacles_stop_the_machine_with_its_drives_off`: the single axis coasting from x = 0.44 with the drives and brake off stops at the face (seen failing: 81 mm in); the T-gantry without a holding brake, its payload into a table at the pick position, rests on it at z = 0.102 within 2 um (seen failing: it fell 0.098 m through), the budget closes to `BUDGET_TOLERANCE` of the supply plus the 17 J of potential energy (the drives draw 0.18 J for a lower that gravity drives; 1.1 mJ open). With brake on the same run is 6.1 mJ open, so the rest is not the stops' |
| 3 The search's tightest check hid a failing collision at `min_clearance_m` 0 | `Candidate::tightest` counts a verdict with a limit of 0 when it fails (`utilisation` is infinite for a contact). The reviewer's `search single-axis --series 1FK2104 --gearboxes NRK070` with the wall now names `collision,inf` | `the_tightest_check_leaves_out_a_dc_link_that_a_resistor_backs` (two cases: a contact against limit 0 is the tightest, a clearance against limit 0 has no share; seen failing: gearbox-radial-load 0.42) |
| 4 Stale nextest comment | `every_effect_switch_changes_the_traces` (1.3 s now) left out of the tests started first; seven remain, each 30 s to 4 min | — |
| 5 Docs | The `obstacles` example of each config lies by its tool path, checked with `sim-cli verdicts`: T-gantry under the traverse, 48 mm below the payload (`collision` fails at 0.3); EXCT-30 under the traverse, 221 mm (passes at 0.17); single axis beside the stroke, 130 mm; delta between pick and place, 50 mm below the payload (passes at 0.025). Config reference: `min_clearance_m` is "T E S D" (D 0.025, its lift); the `crash` rows and the configs' `crash` comments say the obstacles are rigid | — |
| 6 The faster margins test could skip a bound that acts | The friction bounds are left out of the pass with friction off only while `effects.cable_chain` is off too: the chain's drag takes `friction_x.stribeck_m_s` | — (no shipped config has the chain on, so nothing changes; not seen failing) |
| 7 `unused mut` in plcopen.rs | Removed | the build has no warning |

Left open:

- The stop puts the whole machine at rest, not only the motion into the obstacle: a part sliding along a face is not
  stopped, but one stopped in a corner of two motions loses both. Drives that push on an obstacle trip once the part
  has gone `CREEP` (1 um) further in, not on their torque or following error.
- The PLC test's crash and reset run against the wall clock like the other PLC tests.

## Resonances and the belt's wrap (13 November)

| Item | What was built | Test |
| --- | --- | --- |
| Modes through the machine interface | `Machine::modes(tool_pose) -> Vec<Mode>` (`sim_core::Mode { hz, damping }`, lowest first); the default is the one of `resonance_hz` with damping 0, so every other machine keeps working. T-gantry: both roots of the two-mass equation of carriage-on-belt and Z-axis-on-guide, each with the damping ratio of its shape (v'Cv / (2 w v'Mv), belt and guide damping, cross terms left out); `resonance_hz` is now the first of them. Single axis: the carriage on its two sections with the pulley held, f = sqrt(k / m) / (2 pi) (it reported none before) | `both_modes_are_the_roots_of_the_two_mass_hand_calculation` (24.6303 / 104.1875 Hz at z = 0.05 m, 35.6220 / 109.5573 Hz at 0.15 m, damping 0.029751 / 0.055161 / 0.017328 / 0.053512), `the_carriage_mode_is_the_hand_calculated_one_at_two_places` (65.8025 Hz at x = 0.5 m, 96.6523 Hz at 0.1 m) |
| Second shaper | With `effects.shaper_follows_pose` the second shaper (`profile.shaper2_frequency_hz` above 0) takes the machine's second mode at the planned pose, as the first takes `resonance_hz`; one the machine does not report stays as configured (`Controller::shaper_halves`, now public). The optimiser's second shaper without a configured frequency uses the second mode at the start pose, else the one resonance as before | `the_second_shaper_follows_the_second_mode_at_each_pose` (104.1875 / 109.5573 Hz, not following 25 / 100 Hz) |
| Resonance over a stroke | `analysis::modes_along(machine, pose, axis, points)`; `sim-cli modes MACHINE [--axis-index N] [--points N]`: CSV position, then frequency and damping of each mode (empty where a place has fewer). App: the tuning panel's run also asks for the modes at 41 places over the measured axis (`Tuning::modes`), plotted under the configured shaper frequencies (`modes_plot` in ui.rs, its own function). Generic for any machine that reports modes. Labels EN/DE | `modes_write_the_resonances_over_the_stroke` (every machine; single axis 122.2046 / 65.8025 / 57.3698 Hz), `tuning_measures_the_axis_with_the_current_gains` (41 places, two modes each, the middle one the machine's own) |
| Belt wrap | `effects.belt_wrap`, off in all shipped configs (results stay as they were). `Machine::belt_wraps(tool_pose)`: the angle per driven pulley from the places of pulleys and idlers (`mech::wrap_angle`: pi less the lean of both spans). Teeth in mesh = wrap r / p; the tooth-jump law's climb = k z_m p / r becomes k wrap (`ToothJump::set_wrap`, every step in `after_step`), the tooth load counts the teeth of the smallest wrap of the pass (`MotorLoad::belt_wrap`). In both machine crates the spans leave the motor pulleys level at every pose, so the wrap is pi wherever the carriage is: 12.6 teeth on the default pulley (10 configured), 10 of the EXCT-30's 20 | `the_wrap_angle_is_half_a_turn_less_the_lean_of_both_spans` (pi; pi / 2 + asin(0.04) = 1.610807; 2.4 pi = 7.539822), `the_belt_wraps_both_motor_pulleys_by_half_a_turn_and_the_teeth_climb_out_at_k_pi` (pi at x = 0.1 and 0.9 m; at T2 = 8 T1 the teeth jump with the configured teeth and hold with the wrap), `with_the_wrap_the_belt_teeth_jump_at_the_pull_of_half_a_turn` (single axis upright: 289.44 N), `the_tooth_load_counts_the_teeth_in_mesh_from_the_wrap` (8 teeth: 0.205610) |
| Docs | Model reference (Belt wrap, Leaning Z axis's modes, Input shaper), config reference (`belt_wrap`, `shaper_follows_pose`, `shaper2_frequency_hz`, `belt_teeth_in_mesh`), user guide and Benutzerhandbuch (the second shaper, `modes`, the tuning panel), validation | -- |

Left open:

- The wrap is pi at every pose in both machine crates: their idlers stand at the pulleys' tangent heights. The brief
  expected it to change with the pose; it would only with a wheel at another height (an offset tensioner), which no
  config describes. Machine files (`sim-netlist`) give no belt geometry and keep `ratings.belt_teeth_in_mesh`; their
  `UNSUPPORTED_EFFECTS` does not list `belt_wrap` yet, so it is quietly ignored there.
- The T-gantry's modes leave out the frame mode (q) and the control loops, as `resonance_hz` did; the single axis's
  leaves out the gearbox's twist and the motor's inertia. The modal damping leaves out the cross terms.
- The shaper's damping ratio stays the configured one when it follows the pose; the machine's modal damping is reported
  but not used.
- With the shaper following, the EXCT-30's second shaper (17.5 Hz, the tool's response peak) would move to the upper
  two-mass mode (373 Hz there, `sim-cli modes`); it does not follow in the shipped config.

## Delta picker D3: forearm inertia, bending arms, forearms as collision bodies (13 November)

What, how, test:

- **Forearm inertia** (`effects.forearm_inertia`, on in the shipped config). Each forearm pair is a rigid slender rod.
  Its kinetic energy (m/6)(|c'|^2 + c'.p' + |p'|^2) is exactly that of m/6 at each end and 2m/3 at its middle, so the
  existing model holds with the elbow share 1/6 and the platform share 1/6, plus per leg a point mass 2 m_fb / 3 at the
  rod's middle with its Jacobian (J + dc_i/dtheta e_i^T) / 2 (`Legs::rod_middle`, `Delta::load_at`): mass matrix,
  velocity term and gravity. The feedforward with the speed now takes `load` whole. Off, the lumped Codourey/Olsson
  model as before (`mass.forearm_elbow_share`). In the Adept cycle (`sim-cli analyze delta-d3`, rods against lumped):
  motor 1 peak 7.640 against 7.786 N m (-1.9 %), RMS 2.575 against 2.645 N m (-2.6 %); motors 2 and 3 peak 3.987
  against 3.989 N m, RMS 1.587 against 1.615 N m (-1.7 %). Tests: `with_the_arms_horizontal_a_vertical_acceleration_
  takes_the_hand_calculated_torque_with_rods` (arms level at the centre, z = -sqrt(0.6): every rod translates, -10.0 N m
  at the arm, -0.6053 N m at the motor, both models), `the_rods_kinetic_energy_is_the_hand_calculated_one` (sideways
  1 m/s there: 0.31302 J rods against 0.37604 J lumped, plus 2.52596 J of arms, rotors and platform),
  `a_free_loss_free_motion_keeps_its_energy` (E less the constant torques' work over 0.1 s of RK4 at 10 us, to 1e-7 of
  the kinetic energy: seen 3e-14 J of 3.7 J, with rods, bending, both, rigid joints).
- **Bending arms** (`effects.link_elasticity`, off in the shipped config: the stiffness is an estimate). The drive
  train's torsion was already the gearbox's stiffness (`joint_elasticity`), so only the upper arms' bending is new:
  6 states after the brakes (bend delta_i and its rate), the elbow at theta + delta on a spring k_b = 3 E I / L_a with a
  damper (`[upper_arm] bending_stiffness_nm_rad` 65000 = a carbon tube 50 x 2 mm at E 100 GPa, bounds 40000 to 100000;
  `bending_damping_nm_s_rad` 5, bounds 2 to 10). The arm's tube stays with the root. Loss `power.arm_bending`, the
  spring's energy in the elastic energy, the bend at rest Q_d / k_b in `equilibrium` and `rest_sag`. In the Adept cycle:
  tracking error 4.0 to 5.3 mm, dwell error 0.68 to 0.73 mm, motor 1 peak 7.83 N m.
- **Resonance** through `Machine::resonance_hz` and `mode_shape` (the delta had none, so `effects.shaper_follows_pose`
  did nothing on it): the lowest mode with the motors held, the gearboxes' springs (`joint_elasticity`) and the bending
  springs (`link_elasticity`) in series with the arm's tube between them, the rest the mass matrix at the elbows.
  Inverse iteration with the flexibility per arm [[1/k_g, 1/k_g], [1/k_g, 1/k_g + 1/k_b]] (`lowest_mode`, pub). Test
  `the_lowest_mode_is_the_hand_calculated_one` (arms level at the centre, forearms lumped, the sideways mode
  M_s = 1.09572 kg m^2: bending alone 38.76 Hz, gearboxes alone 28.14 Hz, both 22.88 Hz from the two-mass quadratic;
  the shape moves the platform sideways by 1 m). With the shipped config (gearboxes elastic, rods) the delta now reports
  a frequency, so `shaper_follows_pose` follows it.
- **Forearms as collision bodies**: each of the six rods is ten boxes along the drawn segment (`Legs::rods`, shared
  with the scene), each the box around its piece widened by the rod's 12 mm. `the_collision_bodies_are_boxes_of_the_
  scene` (sim-machines) now also accepts a box around a piece of a drawn segment. Test
  `a_forearm_running_into_an_obstacle_is_a_crash` (a 30 mm box at arm 1's rod middle at the centre, 0.36 m above the
  platform: the plan finds the contact, the traverse with `effects.crash` trips the drives at x = 0.137, the platform's
  boxes stay more than 0.3 m away).
- The three hand calculations of the lumped model (holding torque at the centre and the edge, vertical acceleration at
  the centre) and the constant-torque stability test set `effects.forearm_inertia = false`. Docs: model, config and
  validation references, user guide EN/DE (new `analyze` and `verdicts` figures of the default cycle).

Left open:

- The rods' spin about their own axis; the arms' bending out of the plane they turn in, and the tube's own share of the
  bending mode (it stays with the root; 33/140 of a beam's mass would go to its tip).
- The resonance leaves out gravity's stiffness (a few N m/rad) and the backlash; it is the lowest mode only, for the
  one-mode `resonance_hz` (see the coordination note in the report: several modes are a small change of `lowest_mode`).
- Forearm boxes are up to a piece's length (95 mm) wider than a slanted rod; the upper arms are not collision bodies.
- The feedforward stays rigid (no bend under acceleration); the suite configs keep `link_elasticity` off.
- `verdicts --bands` with the rods: gearbox peak 124.7 to 143.3 N m, settling 0.624 to 0.736 mm (fails at all bounds,
  as before); the bending keys' bounds only act with the switch on.

## Thermal warm start and the drive's current limit (13 November)

Two items that make the sizing verdicts honest (goal 2). Not run here: the test suites (the coordinating session runs
them); `cargo check --tests` of sim-components, sim-machines, machine-single-axis, sim-cli and sim-app is clean.

| Item | What was built | Tests (hand-calculated numbers) |
| --- | --- | --- |
| Thermal warm start | `effects.thermal_warm_start` (read with a default of off, so a config of before stays cold), on in every shipped config (T-gantry default and EXCT-30, single axis, delta D3, the joint machines' `common.toml`, `machines/*.toml`). An `effects.` key, not an analysis key: every switch of a run is there, the app lists it with the others, and the configs have no analysis table. `analyse_cycle_with`: where windings (`winding_heating`) or parts (`thermal_feedback`) act back with their warmth, each measured pass is finished, its settled rises (`MotorLoad::winding_rise`, `CycleLoad::part_rises`) compared with those it started at, and the next pass started at a secant step per rise (plain substitution on the first warm pass); stop within `WARM_START_TOLERANCE_K` = 0.1 K or after `WARM_START_PASSES` = 4. `analyse_trajectory_with`: the same, with the machine sent back to the path's start and rested 0.3 s between passes. `CycleLoad::warm_start` (passes, last change); verdict row `thermal-warm-start` (change against 0.1 K; no row: cold); `sim-cli analyze` and `verdicts` print "Thermal start: cold / warm / NOT converged" on the error stream (EN, DE); labels EN, DE | `a_winding_started_warm_loses_the_hand_calculated_copper_power_at_its_settled_rise` (single axis, rated 0.5 A: q = 100/1.39 (0.4463/0.5)² = 57.32 K, settled 73.82 K, copper 4 (1 + 0.0039 · 73.82) 0.4463² = 1.0261 W, cold 0.7967 W); `a_guide_started_warm_has_the_hand_calculated_friction_at_its_settled_rise` (single axis with the feedback: guide at 10.03 K, not 12.51; viscous at 0.5 m/s 10 N · ν(10.03)/ν_ref = 6.3673 N); `the_warm_start_converges_on_the_default_t_gantry_within_its_pass_limit` (windings alone: 1 pass; with the feedback: 2) |
| Drive current limit | `effects.drive_current_limit` (same default rule), on in the shipped configs. `DriveParams::drive_max_current` = `ratings.drive_max_current_a` (infinite without the rating or the effect); `torque_range` narrows the velocity loop's clamp to ±τ(I_drive) (the saturation curve), so the current setpoint stays within min(motor, drive) and the integrator's anti-windup acts as at the motor's limit, also with `torque_limit` off. `torque_curve` (what `motor-peak` judges) stays the motor's. `MotorLoad::peak_current` takes, with the limit simulated, the current of `torque_demand` where higher, so `drive-peak-current` still fails a cycle held at the limit. The search is unchanged: a candidate with its paired drive is limited by that drive; `--motor` without `--drive` keeps the config's drive ratings as before, and is now also limited by them | `the_drive_current_limit_holds_the_torque_to_the_lower_of_motor_and_drive` (drive.rs: 5 A · 0.45 = 2.25 N m below 3.8; 10.9 A leaves 3.8); `a_move_held_at_the_drive_current_limit_accelerates_at_the_hand_calculated_rate` (single axis rigid, 0.5 A, 15 N friction: a = (56.25 − 15) / 6.125 = 6.7347 m/s², 6.733 by sim-cli; motor at 0.18 N m: 4.8980, 4.894; shipped cycle at 70 % of its peak current fails `drive-peak-current`, tracking error 0.2 → 12.7 mm at 0.78 A) |

What moved in the shipped outputs (`sim-cli verdicts`, warm start on, as shipped): the windings settle at 1 to 2 K
(T, S) and 6 to 7 K (E), so verdicts move in the fourth digit (default T-gantry `drive-peak-current` a 1.246 → 1.248:
the current asked for counts now). Every verdict table has one more row, `thermal-warm-start`; golden snapshots and the
docs' example outputs need regenerating. With part heating and the feedback on: single axis guide 12.51 → 10.03 K,
`gearbox-continuous` 0.806 → 0.695 N m; T-gantry guide 4.19 → 3.86 K; EXCT-30 X guide 23.68 → 15.90 K (3 warm passes,
0.045 K). Cost: one more cycle per analysis on the shipped configs, two or three with the feedback (`verdicts` T-gantry
with feedback 2.9 → 3.8 s), so searches, bands and the optimiser take up to twice as long.

Found on the way: plain substitution converged too slowly where the warmth thins a lubricant (EXCT-30's guide, slope
about −0.4: 0.30 K off after the 4 passes); the secant step brings every config within 0.1 K in at most 3.

Left open:

- The trajectory's warm passes start after a way back and 0.3 s of rest, not from the machine's start state (`Sim`
  cannot be copied): what still rings in a lightly damped machine is in the next pass.
- `check_limits` (the warning that the axis limits ask more of a motor than it has) knows only the motor's peak torque,
  not the drive's current; the time-optimal planner plans with the motor's torque too.
- A cycle held at the drive's limit builds a following error whose demand can fail `motor-peak` as well.
- No ramp or i²t of the drive's current (its overload capacity over time): one maximum current.
- Not in the app beyond what is generic (the two switches, the verdict row and its label come through the lists).

## Transition curves on three axes, one length per arc, the planar corner cases (13 November)

Closes the "Not built" line of the transition curves and the "Missing: arcs and splines on machines with more than
two axes" of C1 (lines, arcs and splines on three axes were built with the delta; their transition curves now too).

| What | How | Test |
| --- | --- | --- |
| Curves on three axes | `blend3` in `path.rs`: a line that meets an arc in its direction lies in its plane; a run of lines and arcs in one plane (to 1 nm) is mapped onto the plane's coordinates (along the first arc's start and its direction), blended by the planar `blend` with the speeds of the pieces in space, and laid back: lines and `Arc3` as such, `Geo::Clothoid` with a new field `plane` (point and two directions; `None` on two axes) that `at` lifts into space. A line between two arcs in different planes is cut in half, each half with its arc. `ease` now runs on three axes too. `path::deviation`, `Sim::path_deviation`, the CLI's error-stream line and the app's path table take it as on two axes (the plane keeps distances) | `transition_curves_on_three_axes_lie_in_the_plane_of_the_arc`, path: line, quarter arc of 0.1 m, line in a plane tilted by 30°, 0.1 mm, delta limits: needed 80 mm, allowed L = sqrt(24 e r) = 15.49 mm, so e = L² / (24 r) = 0.1 mm, radius 99.9 mm; curves 15.49 mm; every pose in the plane to 1e-12 m; no steps; every pose within 0.1 mm; 0.353 s against 0.418 s; with a 0.05 m arc in the plane x = 0.1 after the last line all four junctions blended. `a_path_in_a_tilted_plane_gets_its_transition_curves`, delta-d3: both planners through `Sim`, 0.1 mm reported (1e-9), every pose within it, 0.551 s against 0.616 s (time-optimal 0.516 s against 0.578 s) |
| Arcs in different planes | Not blended: a curve between them turns from one plane into the other, with torsion, and the planar construction (each circle moved in within its plane, the curve fitted exactly between two circles) has no counterpart; the path passes at `junction_speed` | — (documented, `shortcut:` in `blend3`) |
| One length per arc | Each blended arc takes L = min(max over its two junctions of v³ dk / (0.4 j), sqrt(24 · tolerance · r)); arcs that hand over to one another turning the same way share one L (longest need, shortest limit along the chain), since their gap is the difference of their e. Opposite turns and lines fit whatever the L. A piece too short still halves every L | `each_arc_gets_the_curve_its_own_junctions_need`: line, 0.1 m quarter, line, 20 mm quarter, line, 1 mm: wide arc at 0.4 m/s, L = 0.064 · 10 / 40 = 16 mm (15.99 fitted); tight arc at (0.4 j r²)^(1/3) = 0.252 m/s, L = 0.016 · 50 / 40 = 20 mm (19.67 fitted; first-order formula at L/r = 1); reported 0.02² / 0.48 = 0.833 mm. One length made both 20 mm |
| Arc fixed at one end, an arc at the other | The fixed arc's circle shrinks by s and moves towards the fixed end as before; its gap to the neighbouring arc is taken as e for the insets, and `transition` fits the curve to the real circles. Same-turn pairs share their L | `an_arc_fixed_at_one_end_gets_its_curve_towards_an_arc`: the path starts on a 0.1 m quarter that hands over to a 50 mm quarter (an S and the same way) and a line, 0.1 mm: two curves, the first arc still leaves the start along +x, no steps, every pose within the reported deviation, faster |
| Arc fixed at one end that turns more than half a circle | At the angle θ from the fixed end the circles are r − sqrt(r² − 2 s (r − s)(1 − cos θ)) apart, 2 s at θ = π; so s ≤ tolerance / 2, e = (tolerance / 2)(1 − cos φ), and the reported distance takes θ up to π. A whole turn has no free end and stays | `an_arc_fixed_at_one_end_that_turns_three_quarters_gets_its_curve`: 270°, r = 0.1 m, 0.1 mm: s = e = 0.05 mm, L = sqrt(24 e r) = 10.95 mm, reported 2 s = 0.1 mm, ends on (0.4, 0.25) heading +y, 1.922 s against 2.025 s |
| Arc fixed at both ends | Meets nothing in its direction (corners, path ends; a spline end is eased by `ease`): no curve is needed, nothing to build | — |

Changed tests: `a_spline_on_three_axes_passes_through_its_points_within_the_limits` said a tolerance changes nothing on three axes; its junctions are now eased (deviation above 0, within 1 mm). The test helper `off_line` works on any number of axes. No config key, no new effect switch (the curves stay under `profile.path_tolerance_m`, 0 in every shipped config, so no golden snapshot changes).

Left open:

- Two arcs that meet in their direction in different planes (a helix-like path) keep their junction and are passed
  slowly.
- A piece more than 1 nm off the plane of its run (a tilted path written with few digits) gets no curves; the
  deviation then stays 0, so the user sees that nothing was blended.
- A piece too short for its curves halves every length of the path, not only the curves at that piece.
- Next to an arc, a fixed arc's gap is taken as its e; the exact gap between the moved circles differs slightly, and
  a curve that would then take too much of an arc is refused (shorter curves, then the path as given).

## App views: workspace map, torque-speed curves, energy flow, filled obstacles (13 November)

| Item | What was built | Test |
| --- | --- | --- |
| Workspace map | `Machine::condition(tool_pose)`: the 2-norm condition number of the kinematics' Jacobian, `INFINITY` out of reach, `None` by default (no map). The delta gives that of `condition_number` (the signal `kinematics.condition`). `Model::workspace_slice(n)`: an n x n grid at the height `inputs.workspace_z` (the middle of the z range by default, clamped to it), over the x and y ranges and half their width beyond each end, per node the condition and whether `unreachable` (no slack) lets the machine go there; the colour scale ends at `ratings.max_condition_number`. App, Path tab: a height slider, the slice drawn with the painter (dark at 1, light at the limit, red beyond, faint where reached but not gone to, empty out of reach), the travel range framed, the condition under the pointer | `the_workspace_map_ends_at_the_hand_calculated_reach_of_the_delta`: at z = -1.0 m the middle row ends towards -x at 0.15 - sqrt(1.35^2 - 1) = -0.7569 m (arm 1 stretched; arm 2 still reaches, 0.906 m of its 0.95 m forearm), the first node of the 0.01 m grid at -0.75 m; the envelope's bottom sphere (R 1.0607 m) is 0.3545 m wide there: x = 0.3 gone to, 0.5 reached but not; no map on the other machines |
| Torque-speed curves | `model::torque_speed_curves(drive)`: the peak curve (as before) and the S1 line, the rated torque both ways up to the rated speed, for the cycle and now for a path (`PathRun::curves`). `points_plot(kind)` takes `Analysis::Cycle` or `Analysis::Path`; the Path tab draws the path's operating points over both curves (PROGRESS "the operating points of a path are not shown") | `the_operating_points_are_shown_over_the_torque_speed_curve` (S1 corners [-rated speed, rated torque], [rated speed, rated torque] from the config; none for a path before one ran), and in `a_path_is_planned_run_and_analysed_in_the_app` the path's points equal its `CycleLoad`'s, its curves `torque_speed_curves` of its drive |
| Energy flow | `Model::budget_flows()`: three columns of positive energies: supply plus what stores gave back; the electrical rows (copper, resistor, discharge, mains, line module, windings, capacitors) with what `electrical_unaccounted` leaves, and the shaft work plus what the machine's stores gave back; the machine's rows with the rest of `unaccounted`. Drawn under the budget table (which stays) with the painter as a Sankey: bars per column, bands from slices of one column to the next in order, as wide as the energy; names beside bars taller than a line, all in the tooltip | in `the_effect_views_show_torque_parts_energy_budget_and_friction`: first column = second, the shaft work = third, second + third - (first - supply) = supply to 1e-9 of the supply, one bar per non-zero row and per non-zero side rest, every bar positive; "Wellenarbeit" in German |
| Filled obstacles | `fill_obstacles`: a translucent box (alpha 0.22, rough, casts no shadow) per obstacle of the scene shown, in the colour of its edges (grey, red in contact); the gizmo edges stay. A click goes through them to the parts | screenshot |
| Screenshot hook | `SIM_APP_TAB` (cycle, effects, search, path, trace, sweep, tuning) opens a tab of the tools panel; `SIM_APP_WORKSPACE_Z` sets the map's height | the screenshots |

Left open:

- The map is a slice and not a volume in the 3D view; the joint-kit machines have no `condition` (the arms' tool pose is
  their joints, where the Jacobian is the unit matrix, and a condition over metres and radians says little; the SCARA
  could have one).
- The S1 line ends at the rated speed: the motor data has no S1 curve above it. One plot for all motors (they share the
  drive data), each motor's points hidden by its legend entry, not one plot per motor.
- The map builds the machine every frame the Path tab is open (as the friction view does): 61 x 61 nodes, cheap on
  the delta.
- A store that gave energy back is drawn as a source of the first column whichever side it is on; its band goes to the
  next bars in order, not to its own side's.

## Planar delta picker D2 (13 November)

A new machine, `machine-delta-d2` (id `delta-d2`): the planar 2-axis delta, two motor-driven arms in the vertical x-z
plane with forearm pairs down to a plate that keeps its orientation. Built on the D3 throughout, with its own copy of
the code (the D3 is being changed by another session): the D3's equations with two legs at 0 and 180 degrees, a 2 x 2
Jacobian with its condition number in closed form, the forward kinematics as two circles' lower crossing, the same
lumped dynamics (Codourey, Olsson; m_nt = plate + payload + 2 (1 - s) m_fb), `RotaryJoint`s elastic or rigid, the D3's
signals, effect views, part heating, refusals and `rest_sag` slack. Tool pose (x, z); the working range a band 800 mm
wide with an arc for its bottom. Scene in the plane y = 0; collision bodies plate and payload. Registered in
`sim-machines`.

Sized like the Codian Robotics D2-800-S020 (codian-robotics.com/robots, read 13 November 2026: range 800 x 477 mm,
5 kg, up to 210 picks a minute, 0.1 mm repeatability); the arm lengths (R 0.15, r 0.05, L_a 0.4, L_b 0.7 m) from a
search over the geometry for the lowest condition number that reaches the whole range (1.56); the D3's 1FK2105-4AF and
S210, ratio 20. The cycle is the D3's Adept cycle in the plane, 0.720 s, and passes all its verdicts, also with
`--bands` one estimate at a time; loop margins at least 41.7 degrees and 8.1 dB over the bands with friction off.

Tests (`crates/machine-delta-d2/src/tests.rs`, hand numbers from a Python mirror of the equations): the standard suite;
round trip and Jacobian against finite differences over the range; holding torque at the centre (-18.358 N m) and at
the edge (-25.43, -4.19 N m); the inertia torque of 10 m/s^2 straight up (-0.9929 N m at the motor); the range's edge
at z = -0.8 (x = 0.256560 m from the bottom arc); refusals with reasons; the cycle's 720 profile cycles; the default
cycle's verdicts; the condition number; the vertical mode's stiffness under constant torques (-3.49 at the default
range's middle, +7.52 N m/rad 150 mm lower, where the suite's configs lie); the feedforward with the speed; the
setpoint slack (0.420 mm by hand, 0.29 to 0.37 mm simulated); a path beyond the range; the collision bodies. The
tests were not run here (by the coordinator's rule); `cargo check --tests` passes and `sim-cli verdicts`, `margins`
and `analyze` were run.

Left open:

- The gearbox's backlash is an estimate (1 arcmin, a reduced-backlash precision planetary): with the 9 arcmin of the
  catalogue's NRB080-05-20 the cycle fails settling (0.85 mm against 0.5 mm), as the D3 does, and every catalogue
  gearbox of this size and ratio has 7 to 9 arcmin. A precision gearbox series in the catalogue would make it
  `[catalogue]`.
- `--bands --worst` (every estimate at its worst at once) fails gearbox peak (201 against 192 N m) and settling
  (0.61 mm).
- Codian publishes no arm lengths, masses, range shape below its two figures, speeds or motors: all estimates.
- The model keeps the plate level; the parallelogram in the x-z plane that does it on the real machine is not drawn (the
  view draws the pairs' rods apart along y, as on the D3), and its rods' mass is in the forearm pairs'.
- Not in the app's own lists or the CAD mass tool (`tools/cad/cadmass.py` knows the D3 only); not in
  docs/integrations.md.

## Public repository polish (13 November)

The repository is public on GitHub since 13 November; its front pages caught up with what is built.

- README and docs/de/README.md: nine registered machines (three belt and delta machines, six from the joint kit) and
  machine files such as the H-bot; six front ends (FMI 2.0 and 3.0, the Python module, the browser page, PLCopen over
  OPC UA). The tour's `sim-cli list` output is the real one of all nine (English and `--lang de`). The limits keep
  every one that is still true; the FMU limit now says what it was checked with (fmpy, OMSimulator) and what not
  (Simulink, Dymola, `fmusim`, the FMU Compliance Checker), and the browser's four-to-five-digit agreement is listed.
  The crate table has `machine-joint-kit` and `sim-netlist`; the test line names them.
- `CITATION.cff` (CFF 1.2.0, version 1.0.0-rc.1). ROADMAP: the "never ran, no remote" item is gone.
- `deny.toml` and a `deny` job in ci.yml (`EmbarkStudios/cargo-deny-action@v2.1.1`). Licences: exactly the 13 the tree
  uses (cargo-deny 0.20.2 reports none allowed and unused). Duplicate versions warn (30 crates, from Bevy and
  async-opcua). Sources: crates.io only. `cargo deny check` locally: advisories ok, bans ok, licenses ok, sources ok,
  with four advisories ignored, each with its reason in the file: RUSTSEC-2023-0071 (rsa, Marvin attack, via
  async-opcua; sim-opcua has only a SecurityPolicy::None endpoint, so no RSA decryption runs) and three unmaintained
  crates with no release to move to (atomic-polyfill and unsafe-libyaml via async-opcua 0.19, ttf-parser via winit
  in Bevy 0.20).
- `.github/workflows/pages.yml`: on push to main and by hand, builds `sim-wasm` for wasm32 as the `wasm` job does and
  deploys `index.html` and `sim_wasm.wasm` to GitHub Pages. Linked from both READMEs and docs/web.md (EN, DE).
- README images: `docs/images/t-gantry-cycle.gif` (800 x 532, 8.6 s, 6.3 MB) from the app's own recorder
  (`SIM_APP_SCREENSHOT` with `SIM_APP_RECORD=9`, frames every second one at 15 fps, made into a GIF with Pillow; no
  ffmpeg here) and `docs/images/delta-d3-app.png` (`SIM_APP_MACHINE=delta-d3`, `SIM_APP_VIEW=30,20`). No app code
  changed: the existing hooks were enough.

Test: no code changed, so no new test. `cargo deny check` as above; `sim-cli list` and `sim-cli verdicts
--machine-file machines/h-bot.toml` run for the README's text; both images looked at.

Left open:

- The shipped `delta-d3` config fails `settling` (0.677 mm against 0.5 mm, `sim-cli verdicts delta-d3`); the
  screenshot and its caption show it as it is.
- The Pages workflow has not run: Pages must be switched on in the repository settings (source: GitHub Actions).
- The advisory ignores in deny.toml go once async-opcua and Bevy are updated past them.

## Time-optimal plan point by point (13 November)

Not run by this session: the test suites (the coordinating session runs them). Run here, release, filtered to the tests
this work touches: `machine-t-gantry` `time_optimal` (6 tests, among them the 200 random paths), all 15 of
`machine-delta-d3`, `a_time_optimal_line_takes_the_hand_calculated_time`, `sim-machines` `time_optimal` (collision,
control) and `sim-cli` `a_path_file_is_planned_run_and_analysed_for_every_machine`: all pass.

| Item | What was built | Tests (hand-calculated numbers) |
| --- | --- | --- |
| Limits point by point (C3) | `path::plan_time_optimal` keeps the path, the look-ahead, the junction speeds and the stops of the plan per piece, and times each piece under limits read at places 2 mm apart (0.02 rad on a curve; 32 to 1000 per piece, `piece_limits`): per motor its torque c a + d v^2 + e from `feedforward_torque_moving` (so centripetal and Coriolis torque count, and gravity each way as it pushes) within `profile.torque_reserve` of the peak, its speed within the rated speed, per belt section its rigid change of tension within the room both ways, per axis its jerk (ramps get all of it on a line, 0.4 on a curve, the rest bounds 3 p'' v a + p''' v^3). A ramp (`climb`) is integrated in quarter cycles at constant jerk: up at the jerk limit, along the most acceleration its places allow (each bound at its worst speed of the ramp, lowered ahead of where it falls so the jerk can follow), down so that it reaches zero as the speed reaches the top; the ramp down is the same from the end backwards. The top speed is searched up to the fastest place's: a piece may cruise faster than its slowest place where the ramps pass that place (at most 0.9 of its speed there, so its torque keeps room). 0.02 % of torque and tension kept back. The per-piece timing (`timed`) stays for the plan within the axis limits and for the transition-curve lengths | `the_time_optimal_plan_takes_the_belt_limit_where_it_is_along_a_line` (T-gantry, x 0.1 -> 0.9, 300 m/s^3, 400 N: bound 16.137 / max(2.6 - x, 1.6 + x) m/s^2 by hand; per piece 0.8 / 1.2559 + 1.2559 / 6.455 + 6.455 / 300 = 0.8531 s, now 0.8517 s; 6.455 -> 6.73 m/s^2 along the ramp, no pose above its bound, the rigid change of tension within 1 % of 42.26 N); `the_time_optimal_plan_takes_the_motor_limits_where_they_are_along_a_line` (delta, (0.5, 0, -0.85) -> (-0.5, 0, -0.85): rated speed allows 7.270 to 13.31 m/s along it by the inverse kinematics; 8.1 m/s planned, 0.411 s against 0.4290 s, torque with Coriolis, speed and jerk within 0.1 %); `the_time_optimal_plan_keeps_the_belt_within_its_tension_limit` changed (7.02 m/s^2 at x = 0.7 where it accelerates instead of the far end's 6.328 everywhere); the hand-calculated single-axis times hold unchanged |

Planned durations, old (one limit per piece) and new, from `sim-cli run --path ... --time-optimal` (exact, before
rounding up to the profile cycle), and the planning time (release, one run of the plan):

| Path | Old | New | Planning old / new |
| --- | --- | --- | --- |
| T-gantry default cycle as lines | 3.574198 s | 3.574199 s | 0.9 / 3-6 ms |
| Delta default cycle (Adept, lines) | 0.589106 s | 0.589066 s | 0.4 / 7-12 ms |
| SCARA default cycle (joint space) | 1.173275 s | 1.173275 s | 0.5 / 8-13 ms |
| URDF arm default cycle (joint space) | 1.657660 s | 1.657660 s | 0.9 / 19 ms |
| T-gantry line x 0.1 -> 0.9, 300 m/s^3, belt 400 N | 0.853080 s | 0.851693 s | 0.2 / 1 ms |
| Delta, line to (0.5, 0, -0.85), across to (-0.5, 0, -0.85) | 0.429020 s | 0.411345 s | 0.2 / 7 ms |
| Delta, lines to (0, -0.4, -0.9) and (0.4, 0.3, -0.8) | 0.405526 s | 0.392167 s | 0.2 / 16 ms |
| User guide line, arc, line | 1.428429 s | 1.394717 s | - / 17 ms |
| User guide spline example | 1.673957 s | 1.379974 s | - / 21 ms |

The default cycles are limited by jerk and rated speed (the delta's 305 mm traverse is a pure jerk ramp, 4 (d / 2 j)^(1/3)
= 0.1575 s; the arms are in joint space, where the speed limit does not change along a line), so point by point gives
them nothing; the T-gantry's is 1 us longer (the ramps' quarter-cycle steps).

Left open:

- One cruise speed per piece: where a piece's speed limit rises and falls between its ramps, it cruises at the lowest
  of it there. A cruise that follows the limit needs ramps with acceleration across places (tried as sixteen
  sub-stretches with a stop of the acceleration at each: gained 0.009 m/s on the delta line, dropped).
- The top speed above the slowest place's is searched by trying 16 + 16 speeds (which top speeds fit is not one range),
  not proven best; a junction is passed no faster than the slowest place of either piece.
- A ramp's acceleration bound is taken at the worst speed of the ramp (on a curve at low speed some torque is left);
  on a curve the jerk is shared 0.4 for the ramps, and the jerk's terms are bounded each by its size.
- Planning is 5 to 40 times slower than per piece (1 to 21 ms); `sim-opcua` plans inside its 10 ms tick (documented).
- The per-piece limits still size the transition curves and decide the refusals before the places are read.
- Motor speed by differences reaches 1 + 3e-5 of the rated speed on the random paths (1 + 2e-13 before), within the
  tests' 0.1 %; the speed bound has no part kept back, so that a speed-limited line stays as fast as before.

## Life estimates: a_ISO, the gearing above its rated torque, the motors' bearings (13 November)

Not run here (the coordinating session runs the suites): only the three new unit tests and the changed one in `analysis.rs`, which pass. Everything else was checked with `cargo check --tests` and by running `sim-cli`.

| Item | What was built | Tests (hand-calculated numbers) |
| --- | --- | --- |
| a_ISO of ISO 281:2007 | `analysis::reference_viscosity` (ν1 = 45000 · n^−0.83 · D_pw^−0.5 below 1000 r/min, 4500 · n^−0.5 · D_pw^−0.5 from it), `life_modification_factor` (radial ball bearings, the three κ ranges, at most 50; κ below 0.1 taken as 0.1, a `shortcut:`), `lubrication` and `pulley_bearing_a_iso`. `pulley_bearing_life_h` is now L_nm = a1 · a_ISO · L10, κ at the pulley's mean speed. New keys in T E S (and the three machine files): `ratings.bearing_lubricant_viscosity_mm2_s` 60 (ISO VG 100 grease base oil at about 50 °C) and `bearing_contamination_factor` 0.7 (sealed, greased for life, Table 13), both `[estimate]` with bounds; `pulley_bearing_fatigue_limit_n` and `pulley_bearing_mean_diameter_mm`: T S 193 N and 31 mm (a 6004-like bearing of the placeholder's C, C_u = C0 / 22), E 259 N and 23.5 mm (the pair of 6002, C 5.85 kN, C0 2.85 kN each), estimates with bounds in E. With none of the four a_ISO is 1; with some, `verdicts` refuses the config by the missing key; e_C above 1 is refused. a_ISO only on the pulley shaft bearings: Siemens' planet and motor bearing figures include their lubrication | `a_iso_is_the_hand_calculated_one_of_iso_281`: ν1 164.082 and 23.717 mm²/s; a_ISO 3.27647 at κ 1 with e_C · C_u / P = 0.125, 0.198152 at 0.2, 0.610291 at 0.5, 6.03577 at 2, 11.0606 at 4 and 8, 0.100176 at 0.05, 50 when the bracket goes below 0; the pulley bearing with ν 120 mm²/s: 728945 h; refusals by name. Worked out separately in Python from the standard's formulae |
| Gearing above the rated torque | Siemens' damage sum is in the configuration manual, not in the repository. Instead Palmgren–Miner on Siemens' own two figures: M2max for at most 30000 output revolutions (the 04/2025 equipment manual, already in `gearbox_nrb.meta.toml`) and no damage at or below M2N, a Wöhler line N = 30000 · (M2max / M)^k between, k = `ratings.gearbox_gearing_wohler_exponent` 8.7 `[estimate]` (ISO 6336-3's tooth root slope for case-hardened gears, ln 3000 / ln 2.5), bounds 6.6 to 10, in T E S D, the rack axis and the machine files. The analysis adds the load spectrum up every step (`MotorLoad::gearbox_torque_spectrum`: the motor's speed per 1 % torque band from 0.1 N m), `gearing_life_h` sums the damage with each band at its upper torque (at most 9 % too much at k 8.7), `gearbox_gearing_life_h` gives hours; no figure where the cycle never exceeds M2N (an infinite life broke the numeric comparisons). Verdict `gearbox-gearing-life` against `required_life_h`. Not on the arm, SCARA and rotary table: their strain wave gears have their own `strain-wave-life` | `the_gearing_life_is_the_hand_calculated_damage_sum`: two levels (48 and 60 N m above 40, M2max 64, k 6) 122.015 h; through a spectrum 118.971 h; at M2max 30000 rev/h last 1 h; no figure without torque, without the key, or below M2N. `the_life_loads_are_the_signals_added_up` now also sums the spectrum by hand (1e-9) |
| Motors' own bearings | `motor_bearing_life_h`: (C / P)³ · 10⁶ / (60 n) · a1 with `ratings.motor_bearing_load_rating_n`, the pulley shaft's radial load and the motor's mean speed, only with `gearbox.ratio` 1 (behind a gearbox the output bearing takes the pull and there is no figure). Verdict `motor-bearing-life`. C from Siemens' allowable radial force for 25000 h: C = F · (25000 · 60 · n / 10⁶)^(1/3). New catalogue table `siemens/motor_bearings_1fk2.csv` (schema `servo_motor` `$defs.bearing_radial_force`, validator passes): the 1FK2205's 1000 N at 20 mm, 1500 r/min, C 13104 N `[estimate]`. The key is in no shipped config (their belts pull on a gearbox or Festo's drive head); it is in `RATINGS_BEYOND_DEFAULT`, so a config file may add it | `the_motor_bearing_life_is_the_hand_calculated_one`: C 13103.7 N, 600 N at 1000 r/min: 25000 · (1000 / 600)³ · 1.5 = 173611 h; none behind a gearbox |
| Front ends | `sim-cli analyze`: columns `gearbox_gearing_life_h`, `motor_bearing_life_h` at the end. App: two more life rows (where a motor has the figure) and two check labels; the pulley bearing's label says "modified rating life" (DE "erweiterte modifizierte Lebensdauer"). Report: the verdicts carry them. EN and DE labels, 6 new parameter labels | `analyze_and_sweep_write_csv_for_every_machine`, `the_cycle_is_analysed_and_judged_in_the_app`, `the_life_verdicts_judge_the_lives_against_the_required_life` extended |

Figures on the shipped cycles (`sim-cli verdicts`):

| Config | Pulley shaft bearing, a1 · a_ISO · L10 | Gearing |
| --- | --- | --- |
| T-gantry default | 472200 h (A), 471300 h (B); 212500 h without a_ISO | no figure (below M2N) |
| Single axis | 1008500 h; 136000 h without | no figure |
| EXCT-30 | 325800 h (31800 h without); bands one at a time 73200 h at the lowest, 60 samples 51200 h, both pass; worst corner 10000 h, **fails** (pretension high, the bearings' C, C_u, the viscosity and the cleanliness at their bad bounds) | no gearbox |
| Delta D3 | no pulley | arm 1 **8981 h, fails 20000 h** (its NRB080 up to 137 N m against 120 N m rated, M2max 192); arms 2 and 3 no figure |
| Rack axis | no pulley | no figure |

The EXCT-30's band test no longer holds the pulley bearings as a failing exception (they pass both band modes, above 40000 h); the worst-corner test still excepts them. The EXCT-30's motor search no longer has `pulley-bearing-life` as every motor's tightest check: now `settling` (1FK2104-5AF 0.530), `drive-current` (1FK2204-5AF 0.514) or `gearbox-radial-load` (about 0.50). The user guides' examples (EN and DE) were rerun: analyze, verdicts, the four searches, bands, 200 samples (33400 h at the lowest), the worst corner (2656 cycles), the delta (`gearbox-gearing-life` 4100 to 24100 h in its bands).

Left open:

- Siemens' own damage sum for the gearing (configuration manual 01/2023, section 6.3) was not read; the Wöhler exponent and the knee at M2N are estimates. The delta's arm 1 failure is a finding about them as much as about the gearbox.
- a_ISO: κ at the mean speed, not per load step; κ below 0.1 taken as 0.1; radial ball bearings only; C_u and D_pw of the pulley bearings are guesses (no maker publishes the bearings). Not on the planet bearings or the guides.
- The 1FK2 radial force diagrams are images: only the 1FK2205 is in the catalogue. Reading the other frame sizes needs the pages rendered (the 1FK2 S120 configuration manual 02/2022, pages 43 to 49, has them). The C stands for the force at 20 mm from the shoulder; no axial load. No config has a pulley on a motor shaft, so `motor-bearing-life` is checked by its unit test only.
- Rating-only bounds (the viscosity, contamination, C_u, D_pw, exponent) rerun the whole cycle in the bands and the margins test although they change only the verdicts, as the existing rating bounds did: two more cycles per bound in `--bands` (T 6, S 6, E 8, D 2 more).

## Robust optimiser, lifts over obstacles, energy front (13 November)

Tests not run here (the coordinating session runs them); `cargo check --tests` of every crate touched and its dependants passes. The numbers below are from `sim-cli` (release) on this branch.

| Item | What was built | Tests (hand-calculated numbers) |
| --- | --- | --- |
| Robust result (C5) | `optimise_cycle_with(entry, cfg, &OptimiseOptions { max_runs, robust, pareto })`; `optimise_cycle` is the nominal one as before. The search keeps the chain of settings it accepted. With `robust`, the fastest is judged with `verdict_bands` over the config's `[bounds]` (one at a time, 2n + 1 cycles); where it fails a check there, the accepted steps are judged from the fastest down until one passes, no further (pass or fail in the bands need not follow the cycle time, so no bisection). Checks the configured cycle fails in its bands already (judged once, only where the fastest fails) cannot be kept and are not judged (`Robust::excused`); the configured cycle is the floor. A step that fails `collision` alone in the bands by less than 1 mm is judged again with the obstacle lifts higher by what the bands lacked. `Optimised::robust` says the fastest's cycle time and failures, the excused checks, the steps back and the band cycles. Default: robust on in `sim-cli optimise` (`--nominal` to skip), and in the app when "with error bands" is ticked (off by default there, so the app's analysis stays at 40 cycles): goals 1 and 2 are about results that hold within the uncertainty of the data, and a nominal winner that fails in the bands is the result the C5 row warned about. Rows `bands_fastest_cycle_s`, `bands_fastest_fails`, `bands_steps_back`, `bands_excused`; cycles on the error stream (`cli-optimise-runs`, EN/DE); three rows in the app's table (`optimised-fastest`, `-fastest-fails`, `-excused` in sim-components' .ftl). **EXCT-30: 2.048 -> 1.590 s** (the search's 1.492 s fails `belt-slack`, `settling` and `pulley-bearing-life` in the bands; the shipped config fails `pulley-bearing-life` there already, 17 349 h at the low bound against 20 000 h, so it is excused; one step back, the jerk of the last step taken back, passes; 3 x 121 = 363 band cycles plus 40 of search, about 30 s on 16 threads). **Default T-gantry: 6.080 -> 5.364 s** (5.340 s fails `settling` in the bands; 3 x 47 = 141 band cycles). Default single axis: 1.732 s, passes at once (31). With a clearance of 0.1 m: 3.442 s robust (3.364 s nominal fails `settling`, two steps back, 188) | `the_robust_optimiser_steps_back_to_the_fastest_setting_that_passes_in_the_bands` (single axis, one bound, payload 2 to 20 kg: 2 * 1 + 1 = 3 cycles per setting judged; the fastest fails `belt-slack`, nothing excused, one step back passes: 3 settings, 9 cycles; the nominal result fails `verdict_bands`, the robust one passes) |
| Lifts over obstacles (C5) | `over_obstacles`: per lift, from the height above pick and place, the traverse at that height is checked as `collision::check_path` does (at rest, every mm, the payload as the segments carry it) and the height moved by what the clearance lacks (or has too much, never below that height) until it is the clearance to 1 nm, at most 40 steps within the axis range (none: the lift is not lowered). The bodies move with the last axis one to one, so a box across the traverse is cleared in one step. `Lift::needed` replaces `lowest` as the target of the lower-lift candidates; without obstacles it is `lowest`, so nothing changes. `PlanCheck::nearest`: the pose at the least clearance; a lift an obstacle raised is also tried rounded through a via point over that pose (at the needed height and 1/8 of the width above), its ends at the height above pick and place. `clears` checks every via candidate's planned path against the obstacles as well. The run sways the payload 0.05 mm nearer than the plan at rest (T-gantry), so a candidate that fails `collision` alone by less than 1 mm is run again, twice at most, with those lifts higher by what it lacked and 1 um | `the_optimiser_lifts_each_traverse_over_a_box_across_it_by_hand` (collision.rs; T-gantry, clearance 0.05 m, 0.1 m box at x = 0.65 with its top at -0.6: lifts -0.6 + 0.05 + 0.802 = 0.252 m with the payload, -0.6 + 0.05 + 0.702 = 0.152 m without (flange), at or above and within 0.2 mm; 0.1 m without the box; the optimised cycle with `effects.crash` touches nothing and passes every verdict). CLI: 0.2520/0.1520 m nominal (3 cycles), 0.2521/0.1521 m robust, 6.08 -> 4.084 s |
| Energy against cycle time (goal 3) | `OptimiseOptions::pareto`: every setting the search runs that passes every verdict as written, plus from the result its acceleration and jerk, and all three limits, at 0.8, 0.6 and 0.4 times (six cycles), as `FrontPoint` (cycle time, `supply_energy`, highest `MotorLoad::utilisation`, factors, blending, shaping); `non_dominated` keeps those no other is as good as in all three and better in one, fastest first, duplicates once. `supply_energy(cfg, load)`: drawn less fed back, plus what the braking resistor burned (a shared one once); from the lean analysis, no budget run needed. `sim-cli optimise --pareto FILE` writes it as CSV; the app plots energy over cycle time below the optimiser's table, the result as a vertical line (`optimised_front` in ui.rs, labels EN/DE). EXCT-30: 1.492 s at 268 J and 0.52 peak utilisation to 3.652 s at 109 J and 0.17 (12 rows); single axis 1.732 s at 88 J to 3.732 s at 57 J (13 rows) | `a_lower_acceleration_takes_the_hand_calculated_braking_energy_less_from_the_supply` (loss-free rigid axis, DC link at 4.7 uF: 0.8 m moves peak at 0.88944 and 0.63121 m/s with 1 and 0.5 m/s^2 (v^2 + (a^2/j) v = a d), 1/2 6.125 kg v^2 = 2.4228 and 1.2202 J per stop; fed back within 2 % below (CLI: 4.827 of 4.846 J, 2.435 of 2.440 J), 2.405 J less per cycle within 2 % (2.392 J), from the supply within 3 % (2.418 J)); `every_row_of_the_energy_front_is_beaten_by_no_other` (single axis, 14 + 6 cycles); `the_front_keeps_the_points_no_other_beats_in_every_figure` (analysis.rs, six points by hand) |

Left open:

- The robust check is the one-at-a-time bands, not the worst corner (`verdict_worst`, about 2400 cycles on the EXCT-30 per setting judged against 121); a step that passes the bands can fail at a corner.
- The search itself judges as written and only its accepted steps are judged in the bands: a setting it passed over (slower but robust, or a robust branch the search never took) is not looked for. A search that judges every candidate in the bands would cost 2n + 1 times as much.
- The shipped EXCT-30 fails `pulley-bearing-life` in its bands (see the worst-corner row in validation.md); the robust optimiser excuses it there. The README's optimiser example (repo-polish's area) still shows the nominal 1.492 s and says the optimiser does not look at the bounds; with this branch it prints the `bands_*` rows and 1.590 s.
- Obstacle lifts: one height per traverse (the highest the obstacles need along it), plus the via candidate over the nearest pose; no profile over several obstacles of different heights. Obstacles above the traverse (a ceiling) are not lifted under; such a lift is left as configured. The vertical legs at pick and place are not searched (their obstacles are the configured cycle's, which `collision` judges).
- The front's energy leaves out the line module's loss (regenerative supply) and the bleeder; the lifts are not columns of the CSV. The front comes from the search's runs and a fixed sweep, not a search of its own along the front.

## Identification for every machine, and belt stiffness without a sweep (13 November)

Goal 1. Two gaps of the D3 row closed: the fit of a kinematics that is not linear, and the belt's stiffness from a move's
trace. No new config key and no new effect switch (both are analyses of a trace, not physics); no golden snapshot
changes.

| What | How | Test |
| --- | --- | --- |
| The rigid model in the motors' coordinates | `Machine::rigid_motor_torque(angle, speed, acc)`: the motor torques of the rigid, loss-free model at motor angles, speeds and accelerations. Default through the tool: pose by `forward`, its speed and acceleration by central differences along the motors' motion over ±0.1 ms, then `feedforward_torque_moving` (exact for the delta, whose `feedforward_torque_moving` has the velocity terms, and for the linear machines). `JointMachine` (SCARA, arm, screw, rack, linear motor, table) overrides it exactly with M q'' + h at the joint over the ratio plus J_rotor i q'', velocity terms included, which its `feedforward_torque` leaves out. The controller is unchanged | through the two fits below; checked by hand on the delta: the rigid torque fed to `derivatives` gives back the asked arm accelerations to 1e-7 |
| Which machines are fitted per axis | `trace::per_axis_fit`: a linear transformation (`linear_transformation`) and a rigid model whose torques per unit of each axis's acceleration agree at 35 % and 60 % of every range (not 40 and 60: a SCARA's inertia, even in its elbow angle, agrees there). Per axis: T-gantry, EXCT, single axis, screw, rack, linear motor, table. Not: the delta (not linear), the SCARA and the arm (inertia changes with the pose). `fit_rigid` refuses the others with `error-trace-not-linear` (text changed in EN/DE to say they are fitted in the motors' coordinates) | `the_delta_fit_finds_...` checks the refusal; `a_trace_from_run_replays_onto_itself_and_fits_its_own_config` (cli) and `a_trace_is_replayed_and_fitted_in_the_app` take the table by it |
| Fit in the motors' coordinates | `identify::fit_joints`: per motor k, τ_k − τ_k,rigid = (θ_p − 1)(τ_k,rigid − τ_k,rigid without payload) + ΔJ_k q̈_k + b_k q̇_k + c_k sign q̇_k + τ0_k, least squares over every motor's rows faster than 5 % of its top speed, columns scaled to unit length, the payload left out where the config has none. Torque and speed averaged over ±5 rows as stage 1; the acceleration from the angle's second difference across them: the speed's slope (as stage 1 takes it) put noise on the regressor and gave 14 % too little payload on the delta (errors in variables). Standard errors as stage 1 (variance widened by the window); pairs whose estimates correlate by more than 0.95 are `alike` and named (`fit-alike`). Run on the trace and on the config's replay of it, side by side, as stage 1. A factor on the rest of the rigid model (arms, platform, rotors) was tried and dropped: on the delta it correlated above 0.95 with all three added inertias and constant torques together and made all of them unknown | `the_delta_fit_finds_a_heavier_payload_and_more_joint_friction` (sim-machines/tests/fit_joints.rs): the delta with every shipped effect, an 8 s swing (0.2 m at 0.6 and 0.9 Hz, 0.06 m at 1.1 Hz), noise 0.01 N m, 0.1 rad/s, 10 urad, payload 4 kg for 3, joint friction doubled; by hand at the motor: × 4/3, Coulomb + 0.3/20/0.96 = 0.0156 N m, viscous + 0.05/400/0.96 = 1.30e-4, inertia and constant 0; found × 1.329 ± 0.014, Coulomb + 0.021, 0.019, 0.020 ± 0.003, all within 3 standard errors, nothing alike. `the_scara_fit_finds_a_heavier_payload_and_more_joint_friction`: gearbox losses off; Coulomb + 0.5/ratio (0.005, 0.00625, 0.01 N m) found 0.00514, 0.00621, 0.00999; viscous + 0.2/ratio² found 1.9e-5, 3.3e-5, 7.9e-5 (2.0e-5, 3.1e-5, 8.0e-5 by hand); payload × 1.335 ± 0.006. `a_joint_fit_needs_every_motor_to_turn_both_ways` |
| Replay of a kinematics that is not linear | `trace::replay` no longer refuses the delta: its setpoints become tool poses by `forward` and run as a trajectory as on the other machines. Delta cycle replayed onto itself: torque RMS 4 to 5 mN m, following error RMS 3e-5 rad | `a_trace_from_run_replays_onto_itself_and_fits_its_own_config` (cli) and `a_trace_is_replayed_and_fitted_in_the_app`, `the_replayed_trace_is_overlaid_and_exported_in_the_app` (app) now include the delta, the SCARA and the arm; `replay_refuses_a_trace_too_short_to_replay` (review2) replays four rows on the delta too |
| Belt stiffness from a move's trace | `identify::fit_move_stiffness(entry, cfg, trace, key)`: an output-error fit through the simulator. The config is replayed with the number at `key` (`BELT_STIFFNESS_KEY` = `belt.stiffness_ea_n`, a key, not a machine) times a factor; the cost is the sum of squares of the difference of the motor torques less their mean over ±10 ms (the ringing, without the rigid motion that a wrong mass would put in). Nine replays from 0.25 to 4, then a parabola through seven replays between the best one's neighbours and another through seven within ±0.1 of its vertex (24 replays, about 1 s on the single axis): the cost is jagged on a 1e-4 scale (a belt tooth or the play meets a step earlier or later), which golden sections fell into. Standard error: var(ln f) = 2 σ² / curvature, σ² = S/(N − 1) widened by (1 + ρ)/(1 − ρ), plus the vertex's variance from the replays' scatter about the parabola. Refused (`fit-move-not-excited`) where that error is above `MOVE_STIFFNESS_LIMIT` = 0.1 of the factor, the parabola has no minimum, or the best factor is at an end of 0.25 to 4; `fit-move-no-key` without the key. First tried: a free two-mass model of the torque, τ = J_m q̈ + J_l H(q̈) + friction with H the spring's low-pass, fitted per motor without a config; on the single axis it locked onto a 51 Hz feature that is in the config's replay too (the drive's own lag) and not the belt (85 Hz), confidently; removed | `the_belt_stiffness_is_fitted_to_a_moves_trace_and_a_tiny_move_is_refused` (sim-machines/tests/move_stiffness.rs): the single axis, belt × 0.7, two moves of 0.8 m at the config's limits, noise as stage 1: 0.708 ± 0.015 (0.7 within 3 standard errors, band below 0.03); the replay leaves 4.9 mN m of the 7.8 mN m of ringing, the noise 0.005 × sqrt(20/21) by hand (within 5 %); 2 mm at 1 m/s³ and 0.2 m/s² refused; an unknown key refused. Noise-free cycle on the CLI: 0.694 ± 0.002 |
| Command line | `fit --trace` writes the joint table (`payload_factor,payload,...`, per motor `added_inertia_kg_m2`, `viscous_friction_nm_s_rad`, `coulomb_friction_nm`, `constant_torque_nm`) where `per_axis_fit` is false, with the advice (`fit-payload`, `fit-joint-*`, `fit-alike`, `fit-joints-skipped`); `replay` and `fit` no longer refuse the delta. `--stiffness [--stiffness-key KEY]` adds `stiffness_factor`, `ringing_rms_nm`, `ringing_left_rms_nm` and `fit-move-stiffness`; a refusal ends with exit code 2 after the table, as `fit --sweep` without a dip | `the_belt_stiffness_is_fitted_to_a_moves_trace_on_the_command_line` (cli: rows equal `fit_move_stiffness`, refusal in EN and DE with code 2, unknown key), `a_trace_from_run_replays_onto_itself_and_fits_its_own_config` |
| App | The trace panel's fit table shows the joint fit's rows (`Payload · Factor on its mass`, `<motor> · Coulomb friction [N m]`, ...) and its advice; a checkbox "Belt stiffness from the moves" (off by default, some twenty replays more) adds the factor's row and its advice or the refusal | `a_trace_is_replayed_and_fitted_in_the_app` (every machine, the joint rows within 2 % of the config's own on its trace, EN and DE); `every_label_exists_in_every_language` |

Docs: measurement procedure (the swing for a delta, SCARA or arm, the moves for stiffness, the joint table), user guide
EN/DE (stage 1 in the motors' coordinates with a delta example, stage 2 from the moves, the table of what is and is not
recovered, the app's trace row; the delta's "does not have" line), validation (three rows).

Run here, beside `cargo check`: the new tests, `a_trace_from_run_replays_onto_itself_and_fits_its_own_config`,
`replay_refuses_a_trace_too_short_to_replay`, the two app trace tests and the two translation tests, in release; all
pass. Not run: the rest of the suites.

Left open:

- The stiffness from the moves is an output-error fit of the whole config: what rings in the trace (the jerk, the
  motor's ripple and cogging sweeping through the mode at speed, the belt's teeth) has to be in the config as it is on
  the machine, and a trace without noise gets an error from the replays' jaggedness alone (0.694 ± 0.002 for 0.7 on a
  noise-free cycle, 2.6 standard errors). One factor on one key; several springs at once (belt and gearbox) are not
  told apart. `sim-cli identify` (no config) has no stiffness from a move: the free two-mass fit was wrong.
- Per motor added inertia stands for rotor and arm together; a factor per link of the URDF (its mass and inertia
  scaled) would need the chain's bodies through the `Machine` trait. Not built.
- A payload that comes and goes in the trace is still not followed: record a swing with the payload held (or none).
- On strain wave gears (efficiency 0.75) the losses on a heavier payload's torque show as Coulomb friction on joint 1
  (twice the true change on the SCARA); the test with all effects runs on the delta (0.96), the SCARA's with losses off.
- German measurement procedure: there is none (docs/de has no counterpart of measurement-procedure.md).

## Machine files: per-motor parts, the eight effects, belt mode, FMU, belt drawing, two new files (13 November)

What, how, test:

- **Different motors per motor.** `catalog::motor_config(cfg, id)` lays `[motor.<id>]`, `[gearbox.<id>]`,
  `[drive.<id>]`, `[ratings.<id>]` and `[parts.<id>]` over the shared tables (only keys the shared table has; a part
  named there is applied from the catalogue, `belt` and `axis` refused). The `ConfigMachine` reads each motor's drive
  train from it (ratio, rotor, gearbox, motor friction, ripple, brake; `ratio` and `j_rotor` are per motor now), the
  `Controller` keeps one `DriveParams` per motor (`drive_params_of(k)`, `motor_torque_curve`) for its loops, windings
  and DC links, and `analysis` takes each motor's for utilisation, winding rise and thermal utilisation. `MotorLoad`
  has the motor's `id`, and `verdicts` judges each motor's checks with its own config. The loader's key schema has
  the per-motor tables of each motor. Test: `the_portal_moves_and_holds_as_worked_out_by_hand` (Z motor angle and
  speed with ratio 10 against 5: 50 rad and 100 rad/s at 0.1 m and 0.2 m/s against 25 and 50; its own rated torque
  0.635 against 1.27; drive-current limit 1.5 A and gearbox-continuous 18 N m against 3 A and 40 N m; every verdict
  passes), `a_motors_own_tables_are_checked` (a misspelt key refused by name, a belt in `[parts.mz]` refused).
- **The eight refused effects.** Seven implemented for any routing, as the T-gantry crate does them:
  `frame_compliance` (node `frame`: a horizontal axis, mass, stiffness, damping; elastic model; bodies not moving
  along it ride on the frame; the tool moves with it), `idlers` (node `idler`: belt speed per axis speed, radius,
  inertia, friction; mass m s s^T, which makes the axes' mass matrix full, and the elastic model now solves it),
  `cable_chain` (an axis's `cable_chain = true`), `process_force` (`process.force_<axis>_n`), `thermal_feedback`
  (`set_part_temperatures`: each gearbox's efficiency per its own law, each guide's viscous friction),
  `belt_tooth_jump` (per motor pulley between its +1 and -1 section; a drum refuses it) and `belt_span_vibration`
  (node `span`: section, length, axes). Switches left out of an older file read as off. `tool_flex` stays refused:
  it needs where the leaning axis's centre of mass and tool tip sit below its guide as it travels, geometry the
  netlist has not. `machines/t-gantry.toml` names the crate's five idlers (speeds z, z, 2x + z, 2x - z, 2x), its frame,
  its cable chain on X and its eight spans, and has `[frame]` and `[idler]`. Test:
  `the_effects_of_a_machine_file_are_its_crates`: each effect on (thermal feedback with the parts 10 to 40 K warm), and
  all together, elastic and rigid: kinematics, rest state, derivatives, energy, feedforward, rigid tensions, every
  shared signal and the eight span frequencies agree with the crate to 1e-9 (the crate's own tests hold the hand
  checks). Before writing it, `sim-cli verdicts` of the crate and the file gave the same digits with each effect on
  (encoder counts off); idlers and chain first differed in the fourth digit, because the overhang's commanded
  acceleration took their mass, which the crate does not: now it does not either.
- **Structural resonance.** `resonance_hz` and `mode_shape` (elastic model): the axes, and the frame where it sways,
  on K = sum EA / L_s c_s c_s^T with the pulleys held, against their mass matrix; Cholesky and Jacobi for the lowest
  mode, scaled to 1 m of the tool's largest travel. Test: `a_machine_file_reports_its_belt_mode` (single axis at
  x = 0.5 m: sqrt(5e5 (1/0.65 + 1/1.95) / 6 kg) / 2 pi = 65.80 Hz; none in the rigid model; the T-gantry file's mode
  drops with the frame swaying).
- **FMU.** `Model::files` adds `resources/machine.toml` for a machine that is not registered; instantiation
  (`machine_of`) builds it with `sim_machines::load_text`; `ensure_sim` takes the FMU's own entry. `sim-cli fmu
  --machine-file` no longer refuses (its message removed). `tools/check_fmu.py` takes a machine file in place of the
  id; ci.yml checks the H-bot's FMI 3.0 FMU. Test: both FMI move tests run on the H-bot, CoreXY and portal files too
  (bit for bit against a plain `Sim`), `a_machine_file_is_run_judged_and_checked` writes the FMU and finds the file in it.
- **Belt drawing.** The scene draws each section as strips through its `route` (pulleys, idlers, bodies), else its
  pulleys and the body it is clamped to; pulleys at `at_m`, idlers at theirs (from their body). Test:
  `the_3d_view_draws_the_belt_of_a_machine_file` (H-bot 4 strips, T-gantry 2 + 4 + 2 and five idlers).
- **Two files.** `machines/corexy.toml` (a = (x + y) / r, b = (x - y) / r; gantry in Y, head in X; four idler nodes)
  and `machines/xyz-portal.toml` (three belt loops; Z vertical with `[motor.mz]`, `[drive.mz]`, `[gearbox.mz]` ratio 10,
  `[ratings.mz]`). Both pass every verdict (`sim-cli verdicts --machine-file`). Tests:
  `the_corexy_moves_as_worked_out_by_hand` (100 and 50 rad at (0.3, 0.1); a diagonal 1 m/s2 asks motor B for
  0.02 / 10 (4 - 8) = -0.008 N m), the portal's holding torque 3.5 * 9.80665 * 0.02 / 10 = 0.0686466 N m, and both in
  `every_machine_file_passes_the_standard_suite` (without backlash, lying flat like the H-bot). The H-bot's `[process]`
  has `force_y_n` in place of the unused `force_z_n`.
- Docs: machine-files.md and maschinendateien.md (nodes, "Motors that differ", the two examples, the model, what is
  left), config reference (`[frame]`, `[idler]`, `force_<axis>_n`, a motor's own tables), user guide EN/DE, fmi.md EN/DE,
  validation.md. `schema/machine-file.schema.json` regenerated.

Not run here (the coordinating session runs the suites): the tests above. `fmpy` is not installed here, so the CI line
of the FMU check was not run either.

Left open:

- `tool_flex` in machine files (see above).
- The time-optimal planner (`path::plan_time_optimal`, planner-topp's area) and `report`'s torque-curve plot take the
  shared `[drive]` for every motor; a smaller motor of its own can be over-asked by a time-optimal plan.
- `sim-cli catalog`'s search sizes the shared motor only; a motor with its own part keeps it, but is not searched.
- The drives' cycle times, noise seed, shared DC link and braking module stay shared; a part named in `[parts.<id>]`
  does not retune that motor's velocity gain (set `drive.<id>.velocity_kp_nm_s_rad` with it).
- A chain on the axis the frame sways along takes its inertia from the axis speed relative to the frame in the rigid
  sense only as the crate does (the crate's own shortcut); idler inertia rides on the axes' speed over ground.
- The belt mode holds the pulleys and leaves out the guides' and motors' own stiffness; the CoreXY's idler nodes are
  one per corner for both belts.
- The belt is drawn as straight strips through the route, not round the pulleys.

## Linear delta (13 November)

A new machine, `delta-linear` in the crate `machine-delta-linear`: three vertical toothed-belt axes on the columns of a
frame at 120 degrees, each carriage with a parallelogram of rods down to a plate that stays level (Festo EXPT tripod,
Kossel printers). Registered in `sim-machines`.

- **Sizing.** Like the Festo EXPT-95 (datasheet EXPT, 2022/02 edition; the 2016 edition for the size table): working
  range 950 mm across and 100 mm high, EGC-80 toothed-belt axes without gearbox, 1 kg rated load at the highest
  dynamics, positioning accuracy +-0.5 mm (the settling limit). Festo publishes no rod length, column layout or masses:
  vertical columns at 0.52 m, rods of 1.1 m, a 0.7 m stroke, carriage 1.3 kg, rods 0.35 kg a pair, plate 0.8 kg, all
  marked and bounded estimates, chosen so that the range is reached with rods at most 57.2 degrees from the vertical and
  condition numbers up to 3.98. Motors: the High Dynamic 1FK2104-5AF has the torque but 0.65 kg cm2 of rotor against
  about 6 kg cm2 of load at its shaft; its velocity loop rang near 450 Hz above 0.1 N m s/rad and the cycle settled to
  2.5 mm. The Compact 1FK2205-2AF (4.0 kg cm2, with brake) on the 6SL3210-5HE11-0UF0 takes 0.25 N m s/rad with Tn 0.02 s:
  at least 41.0 degrees and 8.5 dB over the bounds with friction off (`sim-cli margins delta-linear --bands`).
- **Kinematics** after the D3's (copied, not shared: the machine crates stay apart): inverse h_i = z + sqrt(L^2 -
  rho_i^2), forward by trilateration (the lower point), J = J_x^-1 J_q with J_q = diag(u_i,z), the velocity term
  w_i = -|p' - e_z h_i'|^2, the condition number. The working range refuses targets outside its cylinder, out of reach,
  with a rod steeper than `workspace.max_rod_angle_deg` or a carriage beyond its stroke (two new error keys), or near a
  singularity.
- **Dynamics** at the D3's level: rods split between carriage and plate (`mass.rod_carriage_share`),
  M = m_c 1 + m_nt J^T J, Q_g = -g m_c - m_nt g J_z. Each column is the single axis's belt axis turned upright with the
  motor at the top: two belt sections as springs (`Belt`), pulley, coupling (`Gearbox` of ratio 1), motor shaft friction,
  ripple, brake, stops, a LuGre guide whose levels follow its load across the rail (the rods' pull at rest). Elastic
  (24 states) and rigid (12) models, the feedforward with the speed, `rigid_tensions` for the time-optimal planner,
  `rest_sag` for the setpoint slack (0.1 mm: the belts sag 0.01 to 0.02 mm).
- **Signals, views, scene**: carriage heights, six belt tensions and stretches, radial loads, guide loads, coupling
  torques, the condition number; torque breakdown, loss powers, energies, three friction contacts, six belt sections; a
  scene with columns, belt loops, pulleys, carriages, rods and plate; the plate and payload as collision bodies.
- **Default cycle**: the D3's Adept cycle 25/305/25 mm at the top of the range, 0.720 s; every verdict passes (settling
  0.06 mm against 0.5 mm, belts 141 to 529 N, condition 3.28 against 5), and with `--bands` at both bounds of every
  estimate.
- `sim-machines/tests/views.rs`: the hand calculation of `belt_sections_have_the_pretension_at_rest_and_carry_the_weight_in_a_static_hold`
  is the single axis's and the T-gantry's geometry; the linear delta is left out of it (its own test checks its belts).

Tests (`crates/machine-delta-linear/src/tests.rs`): the standard suite; inverse against forward over 1000 and more poses
and the Jacobian against central differences (its z row summing to 1); the belts' holding forces by hand at the centre
(22.065 N, 0.31606 N m) and off it through a finite-difference Jacobian (32.784, 19.257, 14.154 N, summing to
66.195 N), in the rigid and the elastic model; the range's edges at a height by hand (rod tilt at x = -0.502628 m,
carriage stop at x = 0.302014 m); the feedforward with the speed and an inertia torque by hand (0.63355 N m); the
default cycle's verdicts and its 720 profile cycles; the setpoint slack at rest. The test suites were not run here (the
coordinating session runs them); `cargo check --tests` passes and the numbers above are from `sim-cli`.

Left open:

- The guides' load from accelerating the plate (the load at rest stands in), and with it guide ratings and life.
- No part warms but the motors: the guides' heat is not modelled, so `effects.part_heating` and `thermal_feedback`
  change nothing on this machine.
- Pulley bearing life and the belt teeth's load: the EGC-80's ratings are not in the catalogue, nor the axis itself.
- Inclined columns (some linear deltas lean them inwards): the columns are vertical.
- The rods' rotational inertia, elastic frame, columns and rods, a fourth axis, tooth meshing, tooth jump and belt-span
  vibration on this machine.
- The cross-machine tests in `sim-machines` (effect switches, budgets, scene, lives, ratio change) run on it for the
  first time with the coordinator's test run.

## Four-axis delta picker D4 (13 November)

A new machine, `delta-d4` (crate `machine-delta-d4`): the D3 with a fourth axis that turns the tool about the
vertical, as ABB's IRB 360 with its fourth axis. A fourth motor with its gearbox in the base's centre drives a
telescopic shaft through a cardan joint, and a second cardan joint drives the tool's shaft in the platform.

- Built on the D3: `Delta4` holds a `machine_delta_d3::Delta` built from its own config with the shaft's share of mass
  added to `mass.platform_kg`, and delegates the arms' kinematics, dynamics, rods, bending arms, rest sag, refusals,
  modes, scene and collision rods to it; the tool's turn is a `RotaryJoint` behind them (states: the D3's, then c, its
  speed, rotor angle and speed when elastic, brake). The one change in machine-delta-d3: `pub struct PlatformDrag`
  and `Delta::set_platform_drag`, a force on the platform along the line from a fixed point (the telescope's sliding
  friction), added as J^T F to the velocity term of `load_at` and moved to the friction part of the torque breakdown;
  `None` on the D3, which is unchanged.
- The cardans: one joint at a bend beta turns its output by tan chi = tan phi / cos beta; the motor's and the tool's
  shafts are both vertical, so the two joints are bent alike in one plane (Z arrangement) and with the intermediate
  shaft's yokes in phase the second undoes the first: the tool turns exactly as the gearbox's output at any pose. Only
  the intermediate shaft turns unevenly (signal `shaft.speed`); its spin inertia counts at the tool's speed (its mean
  over a turn is 1 % more at 30 degrees, of a part that is 1 % of the tool's inertia).
- Three switches, all on in the shipped config: `effects.shaft_mass` (the shaft's `platform_share` of `mass_kg` on the
  arms, its spin inertia at the tool), `effects.shaft_torsion` (the shaft's 500 N m/rad in series with the tool
  gearbox's spring, damped by the shaft: the tool rings at 41 Hz with the payload, among `Machine::modes`),
  `effects.telescope_friction` (5 N Coulomb and 2 N s/m along the shaft, a loss `power.telescope`).
- Sized after the ABB IRB 360 datasheet (ROB0082EN: 3 or 4 axes, axis 4 angular repeatability 0.4 degrees; older
  brochures: axis 4 unlimited) on the D3's Autonox RL3-1200 arms. The fourth motor is the arms' 1FK2105-4AF (one motor
  type per machine), behind a Siemens NRB060-05-8 (catalogue; 29 N m maximum, judged by the new check
  `tool-gearbox-peak`). Shaft, tool and joint positions are estimates with bounds; with them the whole working range is
  reachable by the shaft (0.625 to 1.020 m, at most 43.8 degrees bend; beyond: `error-delta-shaft-stroke`,
  `error-delta-cardan-angle`).
- The default cycle is the D3's Adept cycle with the tool turned by +90 degrees on the traverse to the pick and back on
  the return: 0.720 s, every verdict passes (`sim-cli verdicts delta-d4`: settling 0.39 mm, motor 1 at 0.563 of its
  peak against the D3's 0.510, the tool's gearbox at 2.26 of 29 N m). That needs reduced-backlash gearboxes, as on the
  D2: 3 arcmin on the arms and 1 arcmin on the tool, estimates; with the catalogue's 9 and 10 arcmin settling fails,
  1.21 mm (0.6 mm the arms' sag, the D3's finding, and 1 mrad of the tool resting in its backlash, which settling
  counts as 1 mm).
- Scene: the fourth motor, gearbox and rotor bar in the base's centre, the telescope's tube and inner shaft, both
  joints, a bar under the platform turned with the tool; collision bodies: the D3's and six boxes along each of tube
  and inner shaft. Labels en and de; registered in sim-machines.

Test (`crates/machine-delta-d4/src/tests.rs`, hand calculations in each test's comment): the shaft's length, bend and
bending plane at (0.3, 0.4, -0.85) (0.943398 m, 32.0054 degrees, 53.1301 degrees) and over the range, with both
refusals; one cardan joint's angle (34.2486 degrees at 30), the pair giving the angle back over a turn, the speed
ratio as the derivative, tool 10 rad/s with motor 80 rad/s and the shaft at 10.3937 rad/s; the tool motor's torque
for 100 rad/s^2 (0.25226 N m, 0.18976 without the payload); the arms' holding torques at the centre equal the D3's
plus 0.7 kg through dz/dtheta (-0.04580 N m per motor); the telescope's friction (6.696 N, 5.678 W); the tool's
torsional mode (41.116 Hz, 75.857 Hz empty, 2.05 %); the standard suite; the default cycle (the tool at 90 degrees in
the pick, all verdicts pass, with catalogue backlash settling fails alone). Before committing the standard suite, the
i18n, scene and collision-body checks of sim-machines and the hand figures were run once for the D4 alone from a
throwaway example (not kept); `sim-cli margins delta-d4 --bands` with friction on and off: at least 41.7 degrees and
8.1 dB, the fourth motor 48.1 degrees.

Left open:

- With `--bands` settling reaches 0.52 mm at the high end of all estimates at once (the tool's bearing friction and
  backlash, which settling counts in rad as m); the default passes.
- Settling, path and tracking errors take the turn's rad as m (as on the SCARA). A separate angular tolerance (ABB
  states 0.4 degrees for axis 4) would judge the turn as makers do; it is a change in analysis.rs.
- The fourth motor has to be the arms' motor and drive (one `[motor]`/`[drive]` per machine), and the common gearbox
  checks of motor 4 take the arms' `[gearbox]` ratings; `tool-gearbox-peak` alone judges the tool's gearbox.
- Arcs and splines on four axes: the path module takes two and three; `line,x,y,z,c` works.
- Not modelled: cardan yokes out of phase, the joints' friction and play, the shaft's swing about the upper joint and
  its bending, the shaft's own torsional mode between gearbox and tool (gearbox and shaft are one spring in series).
- Not in the app's own lists, the CAD mass tool, README's machine list or docs/integrations.md; not run in the app.

## Machine files: screws, racks, linear motors and rotary tables (13 November)

What, how, test:

- **Four node kinds** in `sim-netlist` (`netlist::NODES`, so the parser and the JSON schema have them): `screw`
  (gearbox, axis, `values`), `rack` (gearbox, axis, `values`), `linear_motor` (motor, axis, `values`; the motor has no
  gearbox node) and `rotary` (gearbox, axis, `inertia_kgm2`, `payload_inertia_kgm2`, `strain_wave`). `values` names a
  table of the file laid out as the joint kit's `[screw]`, `[rack]`, `[linear_motor]`, `[strain_wave]`; the key check
  lays that layout over it (a misspelt key is refused by name and line). An axis turned by a rotary has `stroke_rad` for
  `stroke_m` and limits `max_velocity_rad_s`...; no body, guide, incline or chain on it. A gearbox drives one pulley,
  screw, rack or rotary; an axis has at most one of them; belts and these mix. `pulley`, `section`, `body` and
  `gearbox` are no longer required lists (JSON schema regenerated).
- **The joint kit's drive train, reused.** For such a motor `ConfigMachine` builds a `RotaryJoint` from its own config
  and puts the kit's `transmission::Converter` behind it (`BallScrew::converter(softest_position)`,
  `RackPinion::converter`), exactly as `machine-joint-kit` does: the gearbox becomes one in the axis's unit (ratio
  i / lever, stiffness in series, efficiency multiplied, converter inertia on the rotor, brake from that rotor). Its
  "pulley" is the axis: K gets the row e_i, the axis takes the gearbox output as its force, the belt's rest-state and
  rigid-tension equations leave that axis out, the elastic pulley states follow the axis, no tooth mesh. Linear motor:
  ratio 1 required, cogging order 2 pi / pole pitch, attraction and forcer weight added to the axis's guide load
  signal; nut preload drag and the attraction's friction at the motor with `effects.guide_friction`. Rotary: its
  inertia (and payload's) is the axis's mass. Thermal feedback keeps the converter's share of the efficiency; a coupling
  stays a coupling (`coupled`) after the converter changes the ratio. Signals and checks named by the motor as the kit
  names them by its joint: `joint.<m>.force`, `screw.<m>.speed/.dn/.damage_rate`, `rack.<m>.feed_force`,
  `strain_wave.<m>.damage_rate`, `gearbox.<m>.output_torque` ahead of the converter; `rating_checks` gives
  `screw-speed`, `screw-dn`, `screw-buckling`, `screw-static`, `screw-life`, `rack-feed-force`, `strain-wave-life`.
  The belt mode includes their gearboxes' stiffness (rotors held); the 3D view draws a screw, rack or magnet track
  along the axis and a rotary as a disc.
- **Fixes on the way**: the least-squares solves for K and for the rest state's pulley angles ran over every motor, and
  a motor without a pulley (a zero column) made them singular; they run over the belt motors now. A file with no belt
  section at all builds.
- **Verdicts**: a motor that turns no pulley reports no `pulley.<m>.radial_load`; `analysis::belt_tooth_utilisation`
  passes over a motor that reports neither that nor a belt wrap (else the rack gantry's X motor was judged on belt
  teeth it has not: 1.18, FAIL). Pulley- and motor-bearing lives drop for such a motor the same way.
- **Two files**: `machines/screw-portal.toml` (the XYZ portal with Z on a ball screw 16 x 10 behind a bellows coupling,
  the Z motor's brake holding 0.055 N m of its 0.5) and `machines/rack-gantry.toml` (3 m X on rack and pinion behind
  ratio 10, Y on a belt). `sim-cli verdicts --machine-file`: every check passes (screw-speed 2402 of 9833 r/min,
  rack-feed-force 180 of 2400 N); `--bands --worst` runs on both with their new bounds. The rack gantry first took the belt motors'
  small gearbox for X too: at its upper stiffness bound X rang (settling 1.2 mm, 4863 N on the rack, 7616 N in the worst
  band); X now has the joint kit rack axis's planetary values (35752.6 N m/rad, damping 5) in `[gearbox.mx]` with bounds,
  and settles to 0.015 mm across 25000 to 45000 N m/rad.
- Before the tests: the joint kit's screw, rack, linear motor and rotary table axes written as files (the kit's config,
  the belt and guide tables a file needs, a netlist; joint friction off, hard stops as the slide's) gave the kit's
  `sim-cli verdicts` digit for digit, elastic; rigid and vertical the screw differs only in the warm-start residual
  (1.864e-5 against 1.883e-5: the file's gearbox loss takes a third pass, the kit's two).

Tests (`crates/sim-machines/tests/machine_file_mechanisms.rs`): the screw portal's Z by hand (T = J a / lever +
F lever / eta = 0.0536936 + 0.0730756 N m gives a = 2 m/s2 with F = 3.5 kg (a + g); force, torque ahead of the screw,
1800 r/min, feedforward 0.1194617 N m, holding m g lever) and its critical speed (12291.2 r/min, check limit 0.8 of it);
the rack gantry's motor angle i x / r = 314.159 rad at 1 m and its feed force 34 / 0.97 N for 17 kg at 2 m/s2 with
T = 0.1369228 N m; a linear motor's 130 N at 10 m/s2 drawing 2 A at 65 N/A and its guide load 2627.486 N; a rotary
table's 0.76 N m at 10 rad/s2 in rad with its strain-wave-life check; the joint kit's screw axis as a file against its
crate to 1e-9 (elastic and rigid, payload on, set down, braked: kinematics, rest state, derivatives, energy, feedforward,
14 signals, the checks); both files through the standard suite; a broken screw refused by key and line (unknown field,
misspelt value key, a screw shorter than its stroke, a gearbox with a pulley and a screw). Only the schema test was run
here (`MACHINE_SCHEMA_WRITE=1`); `cargo check --tests` passes, the numbers above are from `sim-cli`.

Left open:

- Linkages and worm gears in files; a body riding on a rotary axis; bearing friction of a rotary axis of its own.
- `effects.belt_tooth_jump` beside a screw, rack, linear motor or rotary is refused (its states are per motor).
- A screw motor's coupling is judged with whatever gearbox ratings the motor's `[ratings]` has (the screw portal's Z:
  a size 40 gearbox's placeholders); a coupling has no ratings of its own.
- The rack's pinion load on the gearbox's output bearing is not modelled: no pulley/output-bearing life for a rack.
- One elastic switch for the whole machine (`effects.belt_stretch`), not the kit's `joint_elasticity`.
- The comparison with the kit covers the screw axis; rack, linear motor and table are compared by `sim-cli` (above)
  and by hand in the tests, not to 1e-9 in a test.

## Faster tests again (14 November 2026)

With the four new machines in every "for every machine" loop, the thermal warm start and the life bounds, `cargo nextest
run --workspace --release --exclude sim-py` had grown from about 4.5 to over 20 minutes warm (22 min 36 s measured
here, with the first of the changes below already in: 1344 s of tests, 16950 s of processor time). Measured this time
per test by its processor time (its process and every process it starts, in a job object of its own), not by its wall
time in the full run: there a test of 30 s on one thread took up to 550 s, waiting for a processor while the tests that
spread over every core held them. One test, the velocity loops' margins across the bounds of every shipped config, took
8590 s of processor, more than all the others together (608 s alone).

Now 7 min 38 s, twice (457 s of tests, 6230 s of processor), 580 tests with 13 left out by the default filter (below),
of which 9 failed in every run before and after: the paths on the D2 and D4 in the CLI, the app and the PLC, the
architecture's two, the direct drive's gearbox, switching an effect and ranking the parameters in the app, repaired
meanwhile on main (a7586c6). With those repairs merged in and all 580 passing: 7 min 59 s (477 s of tests); planning a
path in the app (112 s alone on one thread, 432 s in the run) and the PLC's two long tests, which the group `real-time`
runs last, end the run.

Production code, every result the same to the bit (compared on all twelve machines against the code before: every
motor's sweep at 9 frequencies, the closed loop at 5, the margins with one bound at both ends):

- `frequency_sweep`, `closed_loop_response` and `velocity_loop_margins` spread their frequencies over the processors
  (`on_all_cores`). Each frequency already started from the same rest on a machine or simulation of its own; the
  margins went from one job per run and motor (three on a three-motor machine as configured) to one per frequency.
- `sim-cli tuning` measures the closed loop a batch of frequencies at a time, as many as there are processors, lowest
  first, and stops after the batch with the first point below -3 dB: `bandwidth` reads nothing beyond it.
- The linear delta works out its setpoint slack (`rest_sag`, 600 equilibria, 8 ms) when first asked for and not in
  every build (0.15 ms without it): a loop measurement builds a simulation per frequency and never asks.
- `testkit::run_standard_suite` runs the loss-free configs on a thread each.

Tests (processor time in the full run before → after; alone on the quiet machine before → after):

| Test | What took the time | Change | Processor, s | Alone, s |
| --- | --- | --- | --- | --- |
| margins across the bounds (control) | 13 configs, friction on and off, every bound at both ends, every motor, 60 frequencies | `the_shipped_velocity_loops_keep_their_margins`: every config as configured without friction, the EXCT-30 with it too (the machines' default configs with friction are judged by `margins` in the CLI's step, Bode and tuning test). The sweep is `across_the_bounds::*`, a test per config: the default filter leaves it out, CI runs it in a step of its own, one at a time; `every_shipped_config_is_swept_across_its_bounds` keeps the list whole | 8590 → 220 | 608 → 14 |
| `step_bode_and_tuning_write_csv_for_every_machine` (CLI) | `tuning` on 35 axes, 30 frequencies each one after the other; the margins measured twice per machine | the production changes above; the margins' figures against the library on the one-axis belt machine, every machine's against 35 degrees and 6 dB as printed | 641 → 494 | about 300 (its commands added up) → 59 |
| `the_supply_converges_with_the_integration_step_on_every_config` (effects) | 3 × 100 warm cycles at 4, 8 and 16 sub-steps; the 16 are compared on one config | from cold; 16 sub-steps on that config only | 857 → 253 | 21 now |
| `the_checks_get_error_bands_in_the_app` | the app's bands and the library's again, warm, one machine after the other | from cold, a machine to a thread; the library's on the machine with the fewest bounds, on every machine the table against the app's bands, and the width and worst case of the peak torque's band (from the CLI test) | 862 → 333 | 24 now |
| `verdicts_judge_the_default_cycle_of_every_machine` (CLI) | the bands and three sets of samples on every machine | the bands' printing on the machine with the fewest bounds, from cold | 546 → 32 | 15 now |
| `the_app_optimises_the_cycle_and_applies_the_settings` | 12 optimisers one after the other; failed at the 300 s of `wait_for_jobs` under load | a machine to a thread, from cold; `wait_for_jobs` waits up to 20 minutes for the job to end | 373 → 311 | 43 now |
| EXCT-30 worst corner (catalog) | about 2400 cycles of four passes each with the warm start | the bands from the search's own first runs (`verdict_worst_and_bands`, 2 n + 1 cycles fewer), and the bands test of both modes judges its one-at-a-time bands on them instead of running them again; still warm, it judges the thermal checks | 1230 → 1230 (the bands test 135 cycles fewer of its 196) | 95 → 85 |
| CLI worst corner, CLI sensitivity, the part thermal verdicts, the regenerative supply, the app's worst corner, motor search and lower lifts | the warm start | from cold: they judge nothing thermal (the app's worst corner also takes its bands from the search) | 295 → 169, 131 → 87, 186 → 141, 121 → 77, 235 → 129, 165 → 118, 44 → 32 | |
| the default sweep (sweep.rs) | 5 sweeps of 120 frequencies on one thread | `frequency_sweep` in parallel | 25 → 24 | 23 → 2 |
| the frequency response in the app | every motor's sweep in the app and again in the test | against `frequency_sweep` on the first machine, on every machine the table against the app's sweep: half the sweeps | 216 → (not measured) | |
| loops over every machine on one thread: profile cycle changed on the way, the cycle visits every segment, every effect switch, corner blending, the shaper's dwell (control); `analyze --breakdown`, `analyze` and `sweep`, the trace (CLI); trace, trip, motor search, cycle, effect views (app); the standard suite of every machine | up to a minute alone, up to ten in the full run | a machine to a thread (`on_every_machine`, the thread named after the machine) | unchanged | profile cycle 47 → 9, CLI trace 66 → 7, `analyze --breakdown` 37 → 9, app trace 34 → 5, trip 31 → 4, every machine's standard suite 16 now |

`.config/nextest.toml`: the default filter; the app's nine long analyses and switching an effect (stepped on the test's
own thread) out of the group `real-time` (they wait for their jobs, up to 20 minutes, and judge nothing against the
clock; in the group they held one of its four places for 5 of the 8 minutes); the 21 longest tests first. CONTRIBUTING
and the CI workflow (a step "Margin sweeps") say how the sweeps run.

The 15 slowest of the full run (s): EXCT-30 worst corner 380, the app's optimiser 301, every machine file's standard
suite 299, a path from an end of a stroke 292, the netlist machines' cycles 244, step, Bode and tuning 241, a changed
ratio or pulley radius 181, the replayed trace overlaid (app) 139, lower lifts (app) 131, operating points (app) 124,
the supply's step convergence 111, the six-axis arm's standard suite 111, the regenerative supply's tables (app) 108,
switching an effect (app, failing) 107, the verdicts of every default cycle 103.

Left as they are:

- The EXCT-30's worst corner, 1230 s of processor (a fifth of the run): every cycle takes four passes with the warm
  start, whose windings settle at 6 to 7 K and whose resistance's change takes a fourth pass to 0.1 K; it judges the
  thermal checks too, so it stays warm.
- The margin sweeps run in CI only: some 6500 s of processor, the arms' and the D4's 1100 to 1700 s each (estimated), about
  7 minutes each on 4 cores.
- `every_machine_passes_the_standard_suite` repeats the standard suite of the 9 machines whose crates run it too (150 s
  of processor): it is the one test that holds for every registered machine.
- The loss budget on the integrator's sub-steps costs 0.1 to 1.1 s per config, a tenth of a standard suite: left.
- Long tests on one thread that are not loops over the machines (every machine file's standard suite, a path from an end
  of a stroke, the netlist machines' cycles: 30 to 46 s alone) start first instead.
- The tests repaired on main meanwhile were not changed here (only scheduled): planning a path in the app goes through
  the machines one after the other against the clock (112 s alone), switching an effect too (73 s, no clock).
- The PLC tests run last in the group `real-time`, in the order of the binaries; started early they would run beside
  the busiest tests, against the clock. Their two longest (50 s each) end the run.

## Merging fifteen branches, and what the merged code got wrong (14 November 2026)

Fifteen branches of 13 November were built side by side and merged into main without their test suites run. The first
full run of the merged code failed 43 tests and timed out 4; after six fix rounds all 580 pass (and the 13 margin sweeps
that only CI runs by default). What was found, beyond tests whose expectations an intended change had outdated:

| Defect | Cause | Fix | Test |
| --- | --- | --- | --- |
| Every move of the linear delta refused | Its start pose lay on the working range's top face; the pose it settles at (belts stretch a few um) lay outside | Start and cycle 10 mm lower | the control tests, which now name the machine they fail on |
| Linear delta: 3 m/s on all axes asked 337.7 rad/s of motors rated 314.16 | z turns all carriages one to one | `axis.z.max_velocity_m_s = 2.5` (302.8 rad/s); the cycle's lifts never reach it | `axis_limits_are_checked_against_motor_limits` |
| Linear delta: belt tension 33 N off after a ratio change at speed, 20 um creep at rest | The default state carry-over rebuilt the belts at rest; the integrator worked against stiff guide bristles through a soft loop | `adopt_state` keeps stretch, coupling torque and friction states; `velocity_tn_s` 0.012 (margins at least 38 deg, 8.8 dB) | `a_changed_ratio_or_pulley_radius_does_not_tear_the_machine`, `a_move_arrives_and_comes_to_rest` |
| Energy check off by 0.042 J on the linear delta | A hard stop's damper jumps to c v^2 at contact; the check summed powers at the ends of 62.5 us steps | `testkit::loss_budget` sums on the integrator's sub-steps (error at most 0.011 J of 0.018 J allowed) | the standard suite |
| A test hung on the planar delta | It stepped until the setpoint reached the axis's top speed, which a 0.68 m move never does | The test bounds its loop; tests that need a cruise lower the speed limit where the stroke is short (`cruising`) | `with_thermal_feedback_friction_and_gearbox_loss_follow_the_temperature` |
| The app's saved config reordered dotted keys | `saved_text` rewrote the document with toml_edit, which groups dotted keys | Only each changed value's text is replaced | `a_config_file_is_opened_changed_saved_and_opened_again` |
| A setpoint that had arrived moved during the delta's dwell | The shaper following the pose lengthened its period when the gripper picked (41.5 to 27.2 Hz), reaching back into the lift | The shaper's half periods are held while the plan stands still | `with_input_shaping_a_dwell_starts_when_the_setpoint_has_arrived` |
| Tuning panel and `sim-cli modes` measured 46 um off the start pose they state | They took the rigid kinematics' pose of the motors | Both take `start.pose` | `tuning_measures_the_axis_with_the_current_gains` |
| Machine files named their span signals unlike the crates | `belt.span.span_1.*` against `belt.span.1.*` | Numbered from 1 in file order | `the_effects_of_a_machine_file_are_its_crates` |
| The D4's time-optimal plan slower than the plan within the axis limits | The D3's feedforward counted the telescope shaft's friction in its speed term, which the planner reads as d v^2: ninefold at 3 m/s | The feedforward, the rigid loss-free model, leaves the drag out | `a_path_file_is_planned_run_and_analysed_for_every_machine` |
| A run that tripped also failed `thermal-warm-start` | A stopped run got a warm-start record with an infinite change | No row when the change has no value; the trip's own check fails | the single axis's DC link test |
| `effects.belt_wrap` silently ignored in machine files; two deltas and four machine files without the warm start and the drive's current limit | Written before those switches existed | `belt_wrap` refused like `tool_flex`; the switches added | `every_machine_file_passes_the_standard_suite` |
| A pose read back at a stroke's end refused as out of range | 0.40000000000000013 against 0.4 | Range checks allow `ROUNDING` (1e-9), as path checks did | `a_path_is_planned_run_and_analysed_in_the_app` |

Repository: every workflow action pinned to a commit; a test stopped after 30 minutes in CI (nextest profile `ci`), the
test job after 4 hours; the PLC tests in a CI step of their own, one at a time (six missed their wall-clock times beside
the simulation tests on the Windows runner); clippy clean and `-D warnings` in CI, with four lints allowed on purpose in
the root Cargo.toml; rustfmt at 120 characters (`rustfmt.toml`), checked in CI, the formatting commit in
`.git-blame-ignore-revs`.

Left open:

- The app plans a path from the live setpoint as `start.pose`; its planning model then puts the setpoint one sag higher,
  about 0.26 mm on the planar delta, within the slack.
- Building in one target directory from several worktrees can run another worktree's binaries (cargo gives workspace
  crates the same artifact names); agents built in cargo profiles of their own.

## FMUs in OpenModelica (14 November 2026)

OpenModelica 1.27.1 (64-bit, Windows, installed for the user only from build.openmodelica.org, checksum and signature
checked) against the FMUs of `sim-cli fmu`:

| What | Result | Test |
| --- | --- | --- |
| OMSimulator 2.1.3 (ships with OpenModelica 1.27.1), FMI 2.0 Co-Simulation: T-gantry, delta D3, H-bot machine file, 1 s at 0.01 s | Runs; 96, 94 and 90 outputs agree with `sim-cli run` to 5.0e-10, 4.3e-10 and 4.9e-10 of their peaks (the CLI's 9 digits) | by hand, as `tools/check_fmu.py` compares |
| The same OMSimulator, FMI 3.0 | Refused: "unsupported FMI version" (OMSimulator 3.0 from PyPI runs it, 13 November) | -- |
| OMEdit's Import FMU (`importFMU` of omc) | Refused: "Only FMI 2.0 ModelExchange is supported"; this export writes Co-Simulation only (docs/fmi.md, why there is no Model Exchange) | -- |
| FMIL reading the FMI 2.0 description | Reported "Start attribute is required" for the independent variable `time`, which the standard forbids a start; with a start added the error went, confirming the cause. The 2.0 description now leaves `time` out (implicit by the standard), the ModelStructure counts the listed variables; FMIL reads it without a message, fmpy validates it, the FMI 3.0 description keeps `time` | `fmu_writes_an_archive_with_the_fmi_layout_and_every_signal` (no independent variable declared; outputs by their place), `tools/check_fmu.py` on the T-gantry and the H-bot |

Left open:

- OMEdit's graphical SSP editor (File > New SSP Model) was not clicked through; OMSimulator was run from its command line.
- Simulink and Dymola stay untested.

## The OPC UA server prepared for real PLCs (14 November)

What: `sim-opcua` made ready for a Siemens, Beckhoff or CODESYS OPC UA client, without one here (ROADMAP: "A real PLC
against the OPC UA server: data types, security, cycle jitter").

- Security: three endpoints at the same URL, None and Basic256Sha256 with Sign and with SignAndEncrypt. The application
  instance certificate (self-signed, RSA 2048, 365 days, alternative names the application URI, localhost, the host
  name and its addresses, checked with openssl) is made on the first start in `--pki` (default `pki`). A client
  certificate not in `pki/trusted` is refused on the secured endpoints and lands in `pki/rejected`, from where moving it
  trusts it; `--trust-all` (off by default) accepts and stores every one. `--user NAME` asks for that user and the
  password from `SIM_OPCUA_PASSWORD` (not on the command line); the None endpoint is then not offered, because
  async-opcua accepts a password sent in clear (empty encryption algorithm) whatever the endpoint's password policy. All of it is async-opcua's own (`ServerEndpoint`, `ServerUserToken`, its certificate
  store); `sim_opcua::Security` and `serve_secured` carry it, `serve` keeps the old defaults for the other tests.
- Data types: every variable checked against the IEC 61131-3 type the PLC clients map it to. Nothing a PLC could not
  read or write was found: all variables are scalars (ValueRank -1) with types the clients know (Double LREAL, Boolean
  BOOL, UInt16 UINT, Int32 DINT, UInt64 ULINT, String STRING), and the server already lets a scalar of another type
  through to the write callback, which takes Float and the integers for a number. The node id table, with what S7-1500,
  TF6100 and CODESYS need (S7 String 254 characters, no ULInt on the S7-1200), is in docs/plc.md; the namespace index
  is 2 and the string ids are built from names, so stable across restarts.
- The clients' cycle: async-opcua's simple node manager wrapped in `Timed`, which times each Read and Write request of
  the simulator's namespace (one request counts once; reads counted after they are answered, so that a read of the
  figures is not in its own answer). Count, mean, standard deviation and largest interval are `status.client_*` and
  printed at shutdown; `command.reset_pacing` clears them in the write itself. async-trait (=0.1.92, already in the lock
  through async-opcua) became a direct dependency for the wrapper's trait impl.
- Structured Text: `sim_cycle.st` for TwinCAT (TF6100) and CODESYS beside the existing `sim_plcopen.st`: a secured
  connect with optional user, namespace by URI, MC_Power, MC_MoveAbsolute, a path through `command.path` and
  `command.run_path`, position and following error read every cycle through handles kept; docs/plc.md (and
  docs/de/sps.md) step by step for running them against `sim-opcua` on the same PC, and reading the jitter off.
- CI: the new wall-clock tests run in the serial PLC step (`binary(interop)` in ci.yml and nextest.toml).

Test (`sim-opcua/tests/interop.rs`, async-opcua's client; no independent client was on this machine: no Python
`asyncua` or `opcua` in the repo's .venv or the system Pythons, no UaExpert, TwinCAT or CODESYS):
`a_trusted_client_reads_and_writes_with_basic256sha256_sign_and_encrypt` (the three endpoints offered; an unknown client
refused, its certificate in rejected/; moved to trusted/, it reads and writes with SignAndEncrypt, and connects with
Sign), `trust_all_lets_an_unknown_client_in`, `a_user_name_and_password_keep_others_out` (anonymous and a wrong
password refused, the right one in on SignAndEncrypt; None not offered), `the_type_table_matches_every_exposed_variable`
(every variable of every machine browsed; DataType, ValueRank, AccessLevel and value against the table parsed from
docs/plc.md; every row met; namespace index 2), `the_server_measures_the_cycle_of_a_client_writing_every_10_ms`
(intervals of nine times 10 ms and one of 40 ms, three times, by spinning on the wall clock: by hand 30 intervals,
mean 390/30 = 13 ms, largest 40 ms, standard deviation sqrt(250 - 13^2) = 9 ms; for writes and for reads, within 0.5,
5 and 2 ms). In the crate: `intervals_give_count_mean_standard_deviation_and_largest` (10, 10, 10, 30 ms: mean 15 ms,
largest 30 ms, standard deviation sqrt(75) = 8.660 ms) and `security_options_are_read`. `tests/interop.rs` passed here, 5 of 5 in 37 s
(release, one thread); the crate's unit tests were not run (the coordinating session runs the suites); the server was started with and without `--user` and `--trust-all` and
made its certificate.

Left open:

- No real PLC has connected, and the ST files are not compiled; names that can differ between library versions are
  marked `CHECK`, among them the user identity token members of TF6100 and CODESYS.
- Aes128-Sha256-RsaOaep and Aes256-Sha256-RsaPss endpoints, X.509 user tokens, roles (read only), certificate
  revocation lists: not served.
- The client timing counts all clients together and leaves subscriptions out; a PLC that subscribes instead of polling
  shows nothing there.
- The certificate does not renew itself after 365 days.

## 3D view in the browser page (14 November)

The browser page (`crates/sim-wasm/web/index.html`, GitHub Pages) shows the selected machine in 3D running its cycle,
beside the sizing check as before. Request 3 sets the machine at its start pose, commands its cycle and answers the base
step and the sphere round the start scene (`centre`, `radius`) the camera is fitted to; then `frame(seconds)` steps the
`Sim` and hands back a flat float array, no JSON per frame: a head (time, cycle time, tool x y z, part count) and per
part 20 floats, the mesh (unit cube or unit cylinder along y), the 4 × 4 matrix taking it to the part and its colour.
The shape rides in the matrix with the pose, because belt spans change length and the number of belt strips can change
between frames; the page draws whatever a frame holds. The cycle time is the latest pass from one start of the cycle
to the next, counted as `analyse_cycle` counts passes. `heat_color` and `part_color` moved from sim-app's model.rs to
`sim_core::scene` (sim-app re-exports them), so the page colours belt tension and warmth as the app does.

Rendering is plain WebGL2 in the page (no library: nothing loaded from elsewhere, no SRI hash to keep, works offline;
two meshes, one light and an orbit camera are about a hundred lines). Drag turns, the wheel zooms, Pause/Play and
Restart (with the config in the editor); light and dark follow the page's colour scheme; a legend gives time, cycle
time and tool position, an orange cube marks the tool. Real time, falling behind (at most 50 ms simulated per frame) on
a machine the browser cannot step that fast. The module grew from 1,979,671 to 2,001,607 bytes.

Looked at in the desktop app's built-in Chromium pane, served from 127.0.0.1: all twelve machines start without error;
the T-gantry's view shows a cycle of 6.079 s against the sizing check's 6.080 s; drag, wheel, light and dark work; the
sizing check still runs beside the view.

Tests (`crates/sim-wasm/tests/wasm.rs`): the frame of every machine against its native scene at the start pose and
after 0.5 s of its cycle (mesh, centre, stretch along each axis and colour of every part, tool position, time); two
records and a bounding box by hand (a 0.2 × 0.4 × 0.6 m box at (1, 2, 3) turned a quarter turn: x side along +z, z side
along -x, reaching 0.3, 0.2, 0.1 m out; a strip from the origin to (3, 0, 4): 5 m along (0.6, 0, 0.8), thickness along
(-0.8, 0, 0.6), centred at (1.5, 0, 2)); the view's cycle time against the sizing check's to one base step.

Left open:

- The T-gantry (236 parts, its belts) runs at about a third of real time in the browser pane (measured with the pane
  hidden, so throttled; not measured in the foreground). Instanced drawing or fewer belt teeth would help if it is slow
  in a normal browser.
- No obstacles, arrows, labels or tool trail in the page's view; no pinch zoom on touch screens.

## Identification of links and of a payload that comes and goes (14 November)

Goal 1. The two limits the 13 November fit left: one added inertia per motor for rotor and link together, and a payload
held all through the trace. No new config key and no effect switch (an analysis of a trace); no golden snapshot changes.

| What | How | Test |
| --- | --- | --- |
| A link's inertial parameters through the `Machine` trait | `Machine::links()` (names) and `set_link_change(link, factor, moment)`, default none; `Mechanism` the same, `JointMachine` passes them on, `Arm` (URDF arm and SCARA) changes its body from the one it was built with: mass, first moment m c and inertia about the frame times the factor, then `moment` added to the first moment with mass and inertia about the frame kept (`rigid::Inertial::changed`; the torques are linear in each) | `a_links_change_scales_its_parameters_about_its_frame_and_moves_its_centre` (rigid.rs): 2 kg at (0.1, 0, 0), diag(0.01, 0.02, 0.03), × 3 with 0.2 kg m along y: by hand 6 kg, centre (0.1, 1/30, 0), inertia about it xx 0.07/3, yy 0.06, zz 0.25/3, xy 0.02; × 1 gives the body back |
| Per-link columns in the regressor | `identify::regressor`: after the payload's and the motors' columns, per link the change of the rigid torques with the factor 2 (`link_mass_factor`) and with 0.01 kg m more first moment along x, y, z of its frame, per unit (`link_first_moment_*_kg_m`). A column that changes the stacked rigid torque by less than 1e-9 of it is zeros (rounding of a parameter without effect) | the tests below |
| Base parameters | `identify::solve`: the columns in their order (payload, motors, links), each taken where with it no value's variance inflation (diagonal of the inverse normal matrix, columns scaled to unit length) is above `WEAK` = 100 or below 1 − 1e-6 (an exactly dependent pair gave a negative one by rounding); every other column j is folded: X_j = Σ X_i B_ij + r_j, B the projection, r_j orthogonal to the basis, so its value enters base value i with B_ij (`JointValue::combines`, shares under 1 % of the column left out) and r_j only the residual. The config's replay is fitted on the trace's basis. Tried first: only exact dependence (R_jj² below 1e-9, the QR in column order); on the SCARA the payload, arm 2 and the quill (masses at the end of arm 2) then all stayed, nearly alike, the payload ± 0.73 instead of ± 0.007 | `the_scara_fit_finds_heavier_links_in_the_values_they_combine_with`: arm 2 × 1.25, quill × 1.5, noise as stage 1, gearbox losses off; by hand arm 1's factor into motor 1's inertia with (0.1 + 8 · 0.16²) / 100² = 3.048e-5 kg m² (to 1e-6), quill's and flange's into the payload's with 1.5 / 3 and 1 / 3 (found 0.5010, 0.3297), every value's change equal to the truth through what it combines within 3 standard errors (payload + 0.445 ± 0.007 for 0.764 · 0.25 + 0.501 · 0.5 = 0.441). `the_arm_fit_finds_heavier_links_and_folds_one_its_trace_does_not_excite`: six-axis arm, gears rigid, link 2 × 1.2, link 3 × 1.3; link 1's factor into motor 1's inertia with 0.25 / 120² (to 1e-6); link 2's factor a value of its own, error below 0.02 (+ 0.577 ± 0.013 for 0.2 + 1.217 · 0.3 = 0.565); with joint 2 held within 0.01 rad link 2's factor is folded |
| Payload picked or dropped during the trace | Traces have no gripper column (`Trace` holds time and per motor setpoint, angle, speed, torque), so from the torques: `payload_step` splits the payload's column at every 11th row between a tenth and nine tenths of the trace, then at every row about the best, the sums of squares from running sums along the rows; the least is the step. Its error: how far the sum of squares stays within one variance of its least. It counts where the trace's factor after less before differs from the replay's (which holds the payload all along) by more than `PAYLOAD_STEP_SIGMAS` = 6 standard errors: on the six-axis arm with elastic gears the trace alone showed a "step" at 1.69 s of a trace without one, the replay the same | `a_payload_picked_during_the_trace_is_found_with_when`: the delta with every shipped effect, gripper empty until 3.5 s; step within 5 ms and its error at most 5 ms (found 3.503 ± 0.001 s), × 0 before and × 1 after within 3 standard errors with bands below 0.02 (× −0.014 ± 0.010, × 0.988 ± 0.010). The delta's and the SCARA's friction tests now also check that no step is found |
| Command line | `fit --trace` writes the base values as before (link rows `link_mass_factor,<link>,...`), `payload_factor_before`/`_after` and `payload_step_s,payload,,<time>,<error>` where a step was found; advice `fit-payload-step`, `fit-link-mass`, `fit-link-moment`, `fit-combined`, `fit-no-effect` (EN, DE). `identify` (no config) has no rigid model and gets neither | `a_trace_from_run_replays_onto_itself_and_fits_its_own_config` counts the motors' rows apart from the links' (changed) |
| App | The trace panel's fit table names a link's rows by the link's name, the step by its own row (`ui-trace-payload-step`); labels for the new quantities in EN and DE | `a_trace_is_replayed_and_fitted_in_the_app` counts the motors' rows apart from the links' and takes at least four advice lines per motor (changed); `every_label_exists_in_every_language` takes the new keys |

Shared files touched, small: `sim-core/src/machine.rs` (two trait methods with defaults), `sim-components/src/chain.rs`
(the same on `Mechanism`, passed on), `sim-components/src/rigid.rs` (`Inertial::changed`), `machine-joint-kit/src/arm.rs`
(the built inertials and the two methods), `sim-app/src/model.rs` (the row names, one row, the label keys).

Docs: measurement procedure (links, the payload step, the new rows), user guide EN/DE (a paragraph after the delta's fit
example), validation (three rows). The German measurement procedure still does not exist.

Run here: the six tests of `fit_joints.rs` (the three new ones and the three old) as plain functions in a release build,
all pass (3 to 15 s each); `cargo check`, clippy `-D warnings` and rustfmt on every touched crate; `sim-cli fit` on the
own cycle traces of the SCARA, the arm and the delta (config and fitted agree to the fourth digit). Not run: the unit
test of `Inertial::changed` (its numbers worked by hand), any test
suite, the changed app and CLI tests included.

Left open:

- On the six-axis arm with its gears elastic (as shipped), a trace of an arm with heavier links fits badly even without
  the links' columns: a payload "step" at 6.17 s and joint 2's Coulomb friction 0.08 N m off. The rigid model at the
  motor's angle misses what the gears' twist does on that arm; the arm test runs with `joint_elasticity` off. On the
  SCARA and the delta (elastic too) the fits hold.
- The links' values are the shares the trace tells apart, not a URDF: a folded link's change shows only through the
  values it enters, and the advice says how. A first moment along a link's own joint's axis has no effect and is named
  so. Inertia off the diagonal is not fitted on its own (it moves with the factor only).
- One step of the payload per trace, between a tenth and nine tenths of it; several picks and drops would need a split
  per gripper event (a gripper column in the trace format, where the drive or PLC records one).
- The per-axis fit (`fit_rigid`, the belt machines) has neither: its payload is in the axis's moved mass.

## Machine files: a linkage node, the planar delta as a file (14 November)

What: machine files describe a planar parallel linkage. A `[[machine.linkage]]` node names the two gearboxes that turn
the arms (arm 1 at +x, arm 2 at -x), the tool's two axes (across, up), the geometry (base and platform radius, upper
arm, forearm, rod spacing), the masses (arm with its centre of mass and inertia, elbow, forearm pair with its elbow
share, plate, payload) and the working range (half width, top, heights at edge and centre, highest condition number),
each a number or a path. `machines/planar-delta.toml` is the planar delta's default config with such a node.

How: `crates/sim-netlist/src/linkage.rs`. `Netlist::parse` hands a file with a linkage to `linkage::read` right after
the generic node checks (the only change to the belt parser besides the node list and moving the `machine.id`/`name`
check up); a linkage file has axes without stroke, two motors, a gearbox on each and the linkage, and anything else
beside it is refused by name. `sim_netlist::build` builds a `LinkageMachine` for it: the planar delta crate's model
line for line (kinematics, Jacobian, rest sag, rigid and elastic dynamics, losses, torque breakdown, scene), with the
values from the node, each arm's `RotaryJoint` from its motor's own config (`catalog::motor_config`), axis and motor
ids and keys from the netlist, the gearboxes' warmth under their nodes' ids. The crate stays as it was; the
architecture forbids sim-netlist to depend on it, so the formulas are copied, not shared. No new config key, effect
or label (the crate's translation keys serve, as all machines' are loaded). JSON schema regenerated.

Tests (`crates/sim-machines/tests/machine_file_linkage.rs`):
- `the_planar_delta_file_is_its_crate_with_every_effect`: default and each of the crate's 36 effects switched from its
  default: axes, motors, thermal parts, state length, setpoint slack, inverse and forward kinematics, rest state and
  holding torques, moving state, derivatives off rest, energy, both feedforwards, every signal id and value, the
  working range's refusals, at three poses (payload held, set down, braked), to 1e-9.
- `the_planar_delta_file_runs_its_crates_cycle`: elastic, rigid, and every effect that is off by default on at once:
  every signal at every 16th step of the first 0.5 s, and the cycle's motor loads and verdicts, to 1e-9. Seen first
  with `sim-cli verdicts` on the crate and on the file in those three variants: the outputs are identical.
- `the_planar_delta_file_passes_the_standard_suite`: the crate's four loss-free configs, applied to the file.
- `the_linkage_inverse_kinematics_at_a_pose_is_the_hand_calculation`: at (0.1, -0.5) m arm 1 sees x' = e, so A = 0,
  sin t = -C / B = -0.08 / 0.4: t_1 = -asin 0.2 = -0.2013579 rad; arm 2 solves 0.16 cos t - 0.4 sin t = 0.04,
  t_2 = acos(0.04 / 0.430813) - atan2(0.4, 0.16) = 0.287525 rad; both elbows 0.7 m from the plate; motors at 20 t.
- `the_linkage_holds_the_plate_at_the_centre_with_the_hand_calculated_torque`: the crate's hand calculation at
  (0, -0.5) m, t = 1.4290 deg, -g m_LG cos t + m_nt g dz/dt = -5.490 - 12.868 = -18.358 N m per gearbox.
- `a_broken_linkage_is_refused_with_its_key_and_line`: a forearm shorter than the arm (with its line), an unknown and a
  repeated arm, a stroke on a linkage axis (with its line), a body beside the linkage, a misspelt field.

Left open:
- A scissor lift: not a case of this node. One motor lifts one vertical axis through a screw or cylinder on the
  scissor's foot (z = n L sin phi); it needs a screw that drives a linkage instead of an axis, a node of its own.
- One linkage, the planar delta's topology; it does not mix with belts or screws in one file, and the three-arm delta
  (D3) is no file yet. A spatial linkage node would generalise `Legs` to three legs at azimuths.
- The linkage machine is a copy of the crate's model; a change to one has to be made to the other (the 1e-9 test
  catches a difference). Moving the shared model into sim-components would end that, against the rule that the D2
  crate stays as it is.
- The belt effects are not refused in a linkage file, only not read.

## Delta D3: the rods' spin, the arms' bending out of their planes, the upper arms as collision bodies (14 November)

Three items left open by the D3's forearm-inertia work. Crates `machine-delta-d3` (all of it), `machine-delta-d4`
(config keys, its lossless config), docs. Run here: the D3's and D4's unit tests (24 and 8, all pass); the other suites
not (the coordinating session runs them).

What, how, test:

- **The rods' spin** (with `effects.forearm_inertia`, no switch of its own: it is the rest of the same rigid body's
  inertia tensor, the switch already says "rotational inertia"). The parallelogram keeps a pair's plane through the
  elbow's axis a (fixed) and the forearm u = p - c, so the rods turn about u as that plane does: with sigma = a . u,
  n = a . (u' x u), s = -sigma n / (L (L^2 - sigma^2)) = g . q'. By Kane's equations (angular velocity e x e' + s e,
  angular momentum I s e about the axis) I g g^T in the mass matrix and I (s0' g + s J_u^T (u' x u) / L^3) in the
  velocity term (`Delta::add_spin`, `rod_spin`). New key `mass.forearm_rod_spin_inertia_kgm2` 1.3e-5 per rod (a
  carbon tube 20 x 1.5 mm of 0.15 kg, bounds 0.6e-5 to 2.6e-5). Tests: `the_rods_spin_is_the_hand_calculated_one`
  (leg 1 at (0, 0.1, -0.8) moving 1 m/s square to its forearm, arm 1 still: 0.1 / (0.95 sqrt(0.8925)) =
  0.111422 rad/s; all three legs against the pair's plane turning by central differences, 1e-6; the spin's kinetic
  energy with 0.01 kg m^2 a rod, I sum s^2 = 1.363e-4 J, 1e-6); `a_free_loss_free_motion_keeps_its_energy` now also
  with 0.05 kg m^2 a rod (rigid and elastic with bending: drift 1.4e-14 J of 3.72 J).
- **Bending out of the plane** (in `effects.link_elasticity`'s model): each elbow moves by y_i along its axis on
  `upper_arm.lateral_stiffness_n_m` 400000 (3 E I / L_a^3 of the same tube: 407800; bounds 250000 to 620000) with
  `lateral_damping_n_s_m` 15 (about 2 % of critical, bounds 6 to 30): 6 more states after the in-plane bend. The
  kinematics take the moved elbows (`Legs::forward_moved`, `inverse_moved`, `jacobians`: J_y = J_x^-1 diag(u_i . a_i),
  `velocity_term_moved`); the distal dynamics are 6 x 6 (`Delta::load_n::<6>`, every point mass from its Jacobian,
  solved by `chain::solve`); at rest y = Q_y / k_l; loss in `power.arm_bending`, energy in the elastic energy.
  `load_at` is `load_n::<3>`, so the rigid path keeps its 3 x 3 sizes.
- **Modes**: `arm_modes` (pub, in place of `lowest_mode`): the coordinates (roots, elbow angles, moves) with the
  flexibility per arm and 1/k_l, M = L L^T, the eigenvalues of L^T K^-1 L by Jacobi rotations, rigid connections
  (mu = 0) left out; `Machine::modes` now gives all of them with their damping ratio v'Cv / (2 w v'Mv); `resonance_hz`
  and `mode_shape` the lowest (the shape with the moves in their states). Test
  `the_arms_bending_out_of_their_planes_rings_at_the_hand_calculated_frequency` (arms level at the centre, every
  forearm square to its axis: lumped sqrt(400000 / 0.63335) / 2 pi = 126.48 Hz three times, damping
  15 / (2 sqrt(400000 * 0.63335)) = 0.014901; with rods 0.3 + 0.5/6 + (2/3)(0.5)/4 = 0.46667 kg, 147.35 Hz for the
  three elbows moving the same way round, the other two couple to the sideways modes through the rods' middles and
  ring 0.24 % higher; 3 / 6 / 9 modes with gearboxes / bending / both). `the_lowest_mode_is_the_hand_calculated_one`
  unchanged and passing.
- **Upper arms as collision bodies**: ten boxes along each drawn arm (40 mm wide), after the rods' (`segment_boxes`,
  shared). Test `an_upper_arm_running_into_an_obstacle_is_a_crash` (a 20 mm box at arm 1's middle at the centre,
  0.75 m down: 5.9 mm from the arm's boxes at the start pose, the plan finds the contact, `effects.crash` trips the
  drives at x = 0.123; the rods and platform stay 0.08 m away).
- **The default cycle** (`sim-cli analyze delta-d3`, before and after): the spin changes motor 1's peak from 7.642678
  to 7.642679 N m and motors 2, 3 from 3.986994 to 3.986993 N m, RMS unchanged to 1e-6 (2.575830, 1.586898): the cycle
  runs on y = 0, where leg 1's forearm is square to its axis, and the others spin at a few 0.01 rad/s. With
  `link_elasticity` the moves out of the planes take motor 1 from 7.836258 / 2.612054 to 7.836320 / 2.612210 N m
  (peak / RMS), motors 2, 3 from 4.039997 / 1.621916 to 4.038092 / 1.622322 N m, the dwell error from 0.7295 to
  0.7313 mm, the tracking error 5.307 to 5.309 mm.
- The D4 takes all of it through the D3's `Delta` (its config has the three keys, its lossless config zeroes the
  lateral damping); its `modes` now holds the arms' three (shipped config) besides the tool's torsion.
- Pub API: `Legs::rods` takes the elbow's move y; `Legs::rod_middle` and `lowest_mode` are gone (nothing outside the
  crate used them); new `Legs::tangent`, `elbows_moved`, `inverse_moved`, `forward_moved`, `jacobians`,
  `velocity_term_moved`, `Delta::rod_spin`, `arm_modes`.

Left open:

- The arm tube's own share of the bending modes (it stays with the root in the plane and does not move out of it);
  the elbow block's twist as the arm bends out of its plane (the elbow's axis a stays as it is).
- `modes` now gives the D3 three modes with the shipped config (it gave one): at the centre the second is the
  sideways double of the first, which the optimiser takes as the second shaper's frequency where
  `profile.shaper2_frequency_hz` is 0 (`analysis.rs`, `modes().get(1)`). Not touched (optimiser's area); a second
  shaper at a double mode does nothing new.
- Upper-arm boxes are up to a piece's length (40 mm) wider than a slanted arm; no capsules.

## Optimiser at the worst corner, lift height profiles (14 November)

Goal 3. Tests not run here (the coordinating session runs them); `cargo check --workspace --tests`, clippy of the
crates touched and `cargo fmt --check` pass. The numbers are from `sim-cli` (release) on this branch, 16 threads. No
new config key, no new effect switch.

| Item | What was built | Tests (hand-calculated numbers) |
| --- | --- | --- |
| Robust at the worst corner | `OptimiseOptions::worst` (with `robust`): the result judged with the worst-corner search in place of the bands. `worst_search` (behind `verdict_worst_and_bands`, unchanged for it) takes the previous step's worst corners as a warm start (per verdict its corner is run beside the sign corner, the worse starts the passes) and a `stop`: the search ends after the first batch of runs in which a check judged on fails. The fastest is judged first, stopping at any failure; where it fails, the configured cycle is searched whole (the excused checks), the fastest judged again on the rest where it failed only excused checks or `collision` (the lift-raising retry as in the bands), then the accepted steps bisected, each search started from the one before. `Robust::worst`; CLI rows `worst_*`, `cli-optimise-runs-worst`; app: the existing "worst corner" checkbox applies to the optimiser too (hint EN/DE), rows `optimised-fastest-fails-worst`, `optimised-excused-worst`. **EXCT-30: 2.048 -> 2.048 s**: the fastest (1.492 s) fails `belt-slack` and `settling` at the corner, `pulley-bearing-life` excused (the config fails it there), steps 3 and 1 of the seven fail: no step of the search holds; 3001 corner cycles, 2 min 54 s. **Default T-gantry: 6.080 -> 5.442 s** (bands 5.364 s; 5.340 s fails `settling`), three steps back, 4964 corner cycles, 16 min 28 s. One whole search costs 2553 cycles (2 min 43 s) on the default T-gantry, 1619 (5 min 51 s) on the EXCT-30. **Not the default**: a quarter of an hour on the default T-gantry is not a time a user waits on each try (the bands take 30 s); it is the check to run on a result before building it | `the_optimiser_at_the_worst_corner_steps_back_to_a_setting_that_passes_there` (single axis, bounds pretension 270 to 300 N and payload 2 to 18 kg: the bands pass the fastest of 14 search cycles; per search at most 2 * 2 + 1 = 5 runs alone and 2^2 = 4 corners; fastest 5 + 3 (stops at the slack corner), configured 5 + 4, bisection step 1 of 0..3 5 + 4 passes, step 2 5 + 4 fails: 35 cycles, two steps back; `verdict_worst` fails the bands' result and passes this one) |
| Lift height profile | `profile`: per obstacle, `over_obstacles` with it alone; those it raises are in the way, with the stretch of the traverse at the lowest height (every mm, at rest) nearer to it than the clearance; overlapping stretches merged at the higher height; with two or more, two via points per stretch at its height. `smoothed_profile`: the planner's spline through them sags (11.6 mm on the two boxes below), so the points are raised by what the planned path lacks and 1 um, up to three times; dropped if it still does not clear. A candidate beside the via over the nearest pose, kept or dropped, not moved. `LiftChange::via` and the optimiser's via of a lift are lists of points now (CLI `via_m` row: points apart by `; `, app row likewise). `lift_profiles(entry, cfg)` gives each lift's profile and its raise. Two boxes under the default T-gantry's traverse (the test's): profile 0.152 m from 0.151 to 0.449 m, 0.252 m from 0.551 to 0.849 m, raised 11.6 mm; the cycle through it keeps 51.05 mm with the crash on (CLI `verdicts`). The optimiser does not take it there: 4.72 s against 3.87 s flat with corner blending (200 runs) | `the_lift_passes_two_boxes_each_at_its_own_height` (collision.rs; boxes 0.1 m at 0.3 and 0.7 m of the axis, tops -0.7 and -0.6, clearance 0.05: via heights -0.7 + 0.05 + 0.802 = 0.152 and -0.6 + 0.05 + 0.802 = 0.252 m, two each, in order around the boxes, to 1 um; on the way back the flange keeps 0.1 - 0.702 + 0.7 = 0.098 m from the low box, so one box is in the way: no profile; the low box alone: none; the raised profile as a cycle with `effects.crash`: no contact, clearance above 0.049 m; the optimised cycle passes every verdict with the crash on) |

Docs: user guide EN/DE ("At the worst corner", "A height profile", limits), model reference (optimiser row), validation
(two rows).

Left open:

- `--worst` costs one whole search of the configured cycle wherever the fastest fails (for the excused checks), and a
  whole one per passing step; bisection takes pass and fail to follow the steps (`// shortcut:` in the code). On the
  EXCT-30 nothing the search found holds at the worst corner: a search that judges its candidates at the corner (or
  with a margin learnt from it) would find slower settings that do; not built.
- The profile's spline is checked and raised as a whole; points per stretch at its ends only, so the spline sags
  between them (11.6 mm here). A third point per stretch, or straight lines with transition curves, would sag less.
  The profile goes down between the obstacles only as far as the spline does, not to the lowest height. On the
  T-gantry it never won against flat lifts with corner blending.
- Ceilings, and obstacles beside the traverse that reach above it: no profile (as for the lift).

## Motor and drive model for sizing: field weakening, drive overload, three thermal nodes (14 November 2026)

Three switches for goal 2, each read as off where a config has no switch (a config of before keeps working) and each
reading its keys only when on; all three are off in every shipped config (T-gantry default and EXCT-30, single axis, the
four deltas, the joint machines' `common.toml` and `config/*.toml`, `machines/*.toml`), which carry the keys with their
`[estimate]` marks and `[bounds]`. The bounds count only with their effect on (`PATH_EFFECTS`).

| What | How | Test |
| --- | --- | --- |
| Field weakening | `effects.field_weakening`, `drive::FieldWeakening` (`motor.pole_pairs`, `motor.d_inductance_h`). The one winding is the q axis of a d-q model of the PMSM, scaled as it was (RMS phase current, three times the RMS phase voltage), the d axis alike; ψ = k_e / p. Every current cycle the drive sets the least negative d current that brings the steady voltage vector within sqrt(3/2) U_dc (`d_current_for`, a quadratic in i_d), adds ω_e L_d i_d to the back-EMF it feeds forward and gives the q axis what u_d leaves. The d current is held over the base step like the voltage: `winding_rates` adds u_d i_d, R i_d² and the reluctance torque, so the energy still closes. The torque-speed curve is the most torque inside the voltage and current circles over i_d in [−I_max, 0] (golden section), motoring and braking apart, up to the peak torque (`torque_curve`; `motor-peak`, the report's and the app's curve). The RMS current of the analysis, the heating and the drive's image use sqrt(i_q² + i_d²) (`Drive::phase_current`). A 1FK7 or 1FT7 by name sets the pole pairs (`motors_s120.csv`) and L_d = L_q from its data sheet | `field_weakening_gives_the_hand_calculated_torque_above_the_voltage_limit` (0.45 N m/A, 190 V, 9.5 A, p = 4, 10 mH, R → 0, 500 rad/s: i_d = −50625 / 9000 = −5.625 A, i_q = sqrt(90.25 − 31.64) = 7.6557 A, 3.4451 N m where the fading curve gives 0; the current loop settles there on the voltage circle, power = copper + shaft); `the_d_q_model_holds_the_published_points_of_a_1fk7` (1FK7042-2AF7 at 600 V from its data sheet: rated 2.6 N m at 2.0 A within 6 % and inside the curve with no d current, 527 V of 734.8 V; torque at n_max,inv = 670.2 rad/s where the fade ends at 538.9; best i_d at 942.5 rad/s within 1 % of −ψ / L_d = −3.247 A) |
| Drive overload | `effects.drive_overload`, `drive::Overload` from `ratings.drive_rated_current_a`, `drive_max_current_a`, `drive_base_load_current_a`, `drive_overload_time_constant_s`, `drive_max_current_time_s`. The power unit's i²t image x' = ((i / I_rated)² − x) / τ, stepped exactly each base step; trip = r² − (r² − 1) e^(−t_max / τ), the level the maximum current reaches in t_max from the rated load. At the trip the velocity loop's torque range falls to the torque of the base load current until x is back at 1. A drive by name sets the base load current: an S120 Motor Module's from the catalogue, an S210's as 0.85 of its rated current (`catalog::S210_BASE_LOAD_SHARE`, the share of the S120 modules up to 30 A). Verdict `drive-overload`: the image's peak when the cycle repeats (`analysis::repeating_peak`: the image is linear in its start, the repeating start x₀ = f / (1 − e^(−T/τ))) against the trip; `MotorLoad::drive_overload` | `a_step_at_twice_the_rated_current_clamps_after_the_hand_calculated_time` (3 A, 2.6 A, 9 A for 0.25 s, τ 10 s: trip 1.19752; 6 A from cold clamps after −10 ln(1 − 1.19752/4) = 3.558 s, released 5.842 s later); `the_overload_image_of_a_repeated_cycle_peaks_at_the_hand_calculated_level` (2.25 held; 2.04999 for 2 I_rated half the time) |
| Three thermal nodes | `effects.motor_thermal_nodes`, `drive::ThermalNodes` (`motor.winding_resistance_share` s, `motor.winding_time_constant_s` τ_w): winding and housing as two heat capacities in a chain to the surroundings; the resistances add up to the one node's, rated rise over rated loss, the capacities too (housing (τ_th − τ_w / s) / R, refused unless τ_w < s τ_th). The settled winding rise, which `motor-thermal` judges, is the one node's; `MotorLoad::housing_rise`, `Drive::housing_rise`; the warm start and a rebuild keep or set the housing. The one node stays the fallback | `three_thermal_nodes_settle_at_the_sum_of_the_resistances_and_step_as_by_hand` (s 0.25, 60 s, 1800 s: eigenvalues −7.362e-4 and −1.9349e-2 1/s, winding 16.18 K at 60 s and 78.32 K at 1800 s against the one node's 3.28 and 63.21 K; settled 100 K and 75 K) |

Shown: `sim-cli analyze` has the columns `housing_rise_k` and `drive_overload` at the end; `verdicts`, the report and the
app's verdict table have `drive-overload` (the app's `CHECKS`, now 40); the app's motor table has the two rows; the report's
and the app's torque-speed curves follow the d-q model with field weakening on. `sim-cli search` simulates every motor with
its paired drive through `apply_motor`, `apply_drive` and `verdicts`, so with the switches on it ranks by the new curve,
the overload verdict and the base load currents of the catalogue. Labels EN and DE. Docs: model reference (three
sections), config reference, user guide and Benutzerhandbuch (columns, verdict), validation.

Not checked against Siemens' torque-speed curves themselves: they are printed as graphs in the data sheets and manuals,
which are not in the repository and which were not to be downloaded; the 1FK2 (the shipped motors) have neither R, L, k_E
nor pole pairs in any catalogue file. The load duty cycles of the S120 and S210 are not in the catalogue files either, so
the image's time constant and the maximum current's time are `[estimate]`s (10 s, 0.25 s).

Ran here: the five new tests and the rest of `drive::tests` (25 pass); `cargo check` of sim-components, sim-cli, sim-app
and sim-machines with their tests; clippy and fmt. Not run: the suites (the coordinator's). Changed outside drive.rs:
`sim.rs` (the d current into the integration), `control.rs` (a rebuild keeps housing and image), `analysis.rs` (phase
current, `repeating_peak`, the verdict, `PATH_EFFECTS`), `catalog.rs`, sim-cli's analyze CSV and its header test in
`tests/cli.rs`, sim-app's `CHECKS` and motor table.

Left open:

- The d current has no winding state of its own: it follows its setpoint at once, and the back-EMF in its choice and in
  the curve is unsaturated. `check_limits` still holds the axis speed to the rated speed, so a cycle cannot plan into
  field weakening yet; the planner's torque (`path.rs`, at the rated speed) is untouched.
- The image is one node; real drives have a fast one (the semiconductors) and a slow one (the heat sink), which no
  single τ fits for both the 0.25 s and the 60 s load cycles. With Siemens' duty cycles in the catalogue, fit two.
- No signals for the d current, the housing and the image (scopes, CSV, PLC): adding signals changes every signal list
  the FMI and OPC UA tests count.
- Switching the three on in the shipped configs, and the example outputs and test numbers that would change with it.

## The app's --set, and paths from the live setpoint (14 November)

- `sim-app --set path=value`, repeatable, as `sim-cli --set`: any config value by its dotted path, the value in TOML.
  `Model::set_at_start` applies them in turn to the config (after `--machine-file`) and restarts the model with it; the
  first refusal (`error-config-missing` for a key the config does not have, `error-config-type`, `error-config-parse`,
  or the simulation's own, e.g. `error-config-invalid`) is printed in the app's language and the start ends with
  code 2, nothing applied. The config text stays the file's, so "Save" writes only the changed values in place.
  As on the command line, a value of a part the config names in `[parts]` is overwritten by the catalogue's when
  the simulation resolves the parts (`Sim::new` does that with every config, so `sim-cli --set` behaves the same).
- The parameter panel has a field "Set a value: key = value" (DE "Wert setzen: Schlüssel = Wert") with a button
  "Set"/"Setzen" and Enter: `Model::set_value` sets the key and sends the config to the simulation; a refusal is
  shown right below the field (the newest message), and a value the simulation refuses is rolled back as any edit.
  Screenshot hook: `SIM_APP_SET="key = value"` opens the panel with that typed and applied.
- Path plan offset: the app plans with `start.pose` set to the live setpoint, but its planning model puts the tool
  there and the setpoint one rest sag off (0.26 mm on the planar delta). The path job now moves the planning
  model's tool by what is left between its setpoint and the live one and builds it again, until they agree to
  1e-10 (a fixed point: the sag barely changes over a sag's distance; at most 8 rounds). The first pose of the plan
  is the live setpoint on every machine.

Tests (`crates/sim-app/tests/app.rs`; not run here, the coordinator runs the suites):
`values_set_at_start_change_the_config_and_the_run`: the single axis upright (`geometry.incline_deg = 90.0`),
frictions, gearbox loss, ripple and tooth meshing off, all by `--set`: the motor holds (4 + 2) kg * 9.80665 * 0.02 m
/ 5 = 0.235360 N m, and with `mass.payload_kg = 7.0` 5 kg * 9.80665 * 0.02 / 5 = 0.196133 N m more (`sim-cli run`
with the same `--set`s: 0.235361 and 0.43129 N m at 0.3 s); an unknown key or a wrong type is refused with nothing
applied. `the_panel_field_sets_any_key_and_refuses_as_the_command_line_does`: a key is set; `mass.nope_kg` gives
"Configuration value is missing: mass.nope_kg" (the CLI's message), in German its German one; a string for a number,
a line without `=`, and a negative carriage mass (refused by the simulation, rolled back) each their message.
`a_planned_path_starts_at_the_live_setpoint`: on every machine the plan's first pose is the live setpoint within 1 um.
Screenshots of the parameter panel with `--set mass.payload_kg=3.0` and `SIM_APP_SET` in EN and DE were looked at.

Left open:

- A setpoint within a rest sag of a stroke's end can push the planning model's tool out of range; the plan then keeps
  the last tool pose that started, up to one sag off (`// shortcut:` in the path job).

## Tool-space paths for the SCARA and the six-axis arm (14 November)

Goal 3. The SCARA and the URDF arm moved in joint space only, so no line of the tool was straight. Now
`kinematics.tool_space = true` makes their axes the flange's pose, behind the `sim_core::Machine` interface as on the
deltas: every planner, the cycle, the CLI and the app work on tool poses unchanged. Off in the shipped configs; two new
configs have it on (`config/scara-tool.toml`, `config/arm-tool.toml`, also `SCARA_TOOL_CONFIG`, `ARM_TOOL_CONFIG`).
Chosen over mapping tool-space path files in the planner: the `Mechanism` trait of `chain::JointMachine` already had
`forward`, `inverse` and `unreachable`, and the joint machine already turned tool speeds into joint speeds through the
Jacobian of `forward`; only the mechanism's side was missing.

How (`machine-joint-kit/src/tool.rs`, wired in `arm.rs`):

- SCARA: x, y, z (m), c (rad) in closed form from the URDF at zero (arm lengths and directions, joint 1's place, the
  quill's direction, the flange's turn), checked against the chain at three poses; the elbow by `kinematics.elbow`
  ("right": joint 2 counter-clockwise from the stretched arm). Condition number of d(x, y)/d(q1, q2).
- Six joints: x, y, z (m) and the rotation vector rx, ry, rz (rad, from the base frame). Damped least squares on the
  geometric Jacobian (lambda 1 mm, turn rows times the flange's distance from joint 2 at the home), followed from
  `kinematics.home_joints` along the line (orientation by slerp) to the target in steps of 20 mm / 0.05 rad, settled to
  1 um, the last to 1e-13 m: the solution stays on the home's branch; a step that does not settle refuses the pose.
  About 90 us per inverse in release. Condition number of the scaled 6 x 6 geometric Jacobian, by Jacobi rotations.
- Refusals: `error-arm-out-of-reach`, `error-arm-singularity` (above `workspace.max_condition_number`: SCARA 25, arm
  30), `error-arm-orientation-half-turn` (|r| within 0.01 rad of pi, where the rotation vector jumps); the joint
  machine's `error-joint-beyond-stop`. Ranges `workspace.tool_min` / `tool_max`, limits `[axis.x]` ... `[axis.c]` /
  `[axis.rz]`.
- `sim-components/src/chain.rs` (shared, small): `Mechanism::condition` (passed on as `Machine::condition`, so the
  workspace map and `ratings.max_condition_number` work), `Mechanism::tool_is_joints`, and where the tool is not the
  joints `feedforward_torque_moving` with the speed terms (q'' = J^-1 (a - J' q'), J' q' by a second difference of
  `forward`, then `rigid_motor_torque` with Coriolis and centripetal torques); in joint space unchanged. `JointSpec`
  got the joint's `unit`, which the joint signals used to take from the axes (wrong once the axes are the tool's).
- `sim-components/src/control.rs` (shared, small): `plan_path` with `time_optimal` first checks the path as planned
  within the axis limits for poses the machine cannot go to. A line ending at the SCARA's stretched arm made the
  time-optimal timing slow to a standstill and ask for 64 GB of points; now it is refused with `error-path-range`.

Measured (`sim-cli verdicts`, release): the SCARA's tool cycle (pick 100 mm down, 500 mm across turning 1.5 rad, back)
2.542 s, every verdict passes, settling 0.04 mm; the arm's (the same, flange pointing down) 3.774 s, every verdict
passes with `settling_tolerance_m` 5 mm: under the payload its flange rests 4.0 mm below the encoders' pose (0.2 mm
rigid). The time-optimal line across the SCARA: 1.542 s against 1.750 s within the axis limits, motor 1 at 307.5 of its
rated 314.16 rad/s.

Tests (`machine-joint-kit`):

- `the_scara_closed_form_gives_the_joints_of_a_pose_by_hand`: at (0.325, 0.275, 0.1, 0.3), cos(elbow) = 0: right elbow
  pi/2 with joint 1 = 0, left -pi/2 with joint 1 = 2 atan2(0.275, 0.325) = 1.404514; joint 3 = 0.1 m; 1e-12.
- `inverse_then_forward_returns_sampled_tool_poses_within_1e_9`: the SCARA over its tool range (standard suite's grid),
  the arm at 40 poses around its home (turns up to 0.3 rad), its wrist on the home's side; static hold of both.
- `every_planned_pose_of_a_straight_tool_line_lies_within_1_um_of_it`: the SCARA's line (x = 0.4, z = 0.15,
  c = 3 (y + 0.25)) within the axis limits and time-optimal; the arm's line with (0, pi/2, 0) held within 1e-6 rad, read
  back through the URDF chain.
- `the_joint_speeds_along_a_tool_line_are_the_inverse_jacobian_times_the_tool_speed`: at (0.325, 0.275), J =
  [[-0.275, -0.275], [0.325, 0]], 0.5 m/s along y: joint 1 0.5 / 0.325 = 1.538462 rad/s, joint 2 the opposite, joint 4
  0, the motors times 100 and 80; condition number 2.4691 from the eigenvalues of J^T J.
- `a_tool_path_into_the_stretched_scara_arm_is_refused_with_its_reason`: x = 0.598 (elbow 0.1639 rad) refused with
  `error-arm-singularity`, 0.59 (0.3669 rad) goes, 0.61 `error-arm-out-of-reach`, a line ending at 0.6 `error-path-range`
  (both planners).
- `the_time_optimal_plan_keeps_the_scara_motors_within_their_rated_speed_along_a_tool_line`: no motor above 314.159
  rad/s (+0.1 %), the fastest above 0.9 of it and faster than within the axis limits.
- `the_tool_configs_are_the_defaults_but_for_tool_space_start_cycle_and_settling`,
  `the_tool_space_cycles_pass_their_verdicts`.

Left open:

- Orientation along a line is the linear interpolation of the rotation vector: the slerp only where both ends' vectors
  are parallel (held, or a turn about one axis through the base frame's orientation); a turn about the vertical of a
  flange pointing down is close to the slerp, not it. A slerp needs the planner to interpolate orientations.
- Arcs and splines take two or three axes, so on the arm (six axes) and the SCARA (four) only lines; an arc in x, y with
  the rest held would need the planner to carry the other axes along.
- The six-axis inverse follows the line from the home on every call (90 us); a warm start from the last solution would
  make the arm's tool-space cycle (8 s of computing) several times faster.
- The time-optimal planner itself still asks for unbounded points where a reachable pose on a path slows a motor to
  nearly zero speed (only poses the machine refuses are caught before it); planner-optimal's area.
- In joint space the controller's feedforward still leaves out the arms' Coriolis and centripetal torques; switching
  them on there changes the shipped arms' figures.
- The friction view spans each joint's friction over its axis's speed limit, which in tool space is the tool's (m/s for
  joint 1 of the SCARA).

## Picking from a moving conveyor (14 November)

Goal 3: a cycle that picks parts riding on a belt conveyor, for every machine whose tool axes are Cartesian positions
(the four deltas, the gantries), generic in sim-components (`conveyor.rs`, no machine named).

- A `[conveyor]` table (optional, in no default config; `analysis::RATINGS_BEYOND_DEFAULT` lets a config file have it):
  the belt's direction in tool coordinates, speed and ramp, the grip pose where parts enter, parts at a pitch with
  listed times within it, the pick window along the belt, the lift, grip and release times, the place (fixed, or on a
  second belt with its direction and speed), and the tracking tolerance. Keys in docs/config-reference.md.
- Each move of a pass is a jerk-limited `profile::Move` to Q plus a belt term whose velocity blends from va to vb with
  h(u) = 3u^2 - 2u^3 (no jump in acceleration at either end). A target at P moving with vb is met at T with its velocity
  where Q = P + T (vb - va) / 2; the move runs with each axis's limits less what the belt term takes (max(|va|, |vb|),
  1.5 |dv| / T, 6 |dv| / T^2), and T, on which both depend, is found by bisection (`conveyor::intercept`).
- A pass: to above the part, down, the grip moving with it (payload attached at its end), up, to above the place, down,
  the release, up, and from a moving place to rest. A part the tool would meet before the window is waited for (the
  pass starts later, by bisection on its start); one gripped beyond the window, or whose pass leaves the range, is
  missed. The controller runs a pass as `Motion::Conveyor` on the simulated time; one pass is one cycle pass, so
  `analyse_cycle`, the CLI (`analyze`, `verdicts`, `run --cycle`) and the app run it unchanged.
- Results (`CycleLoad::conveyor`): picks a minute = 60 / the measured pass; the tracking error (measured tool against
  the part along the belt during the grip) as the verdict `pick-tracking` against `conveyor.tracking_tolerance_m`;
  parts missed a minute (beyond the picks a minute, or counted in the pass, whichever is more) as `parts-missed`
  against 0. `analyze` says the three on its error stream.
- The scene draws the belt as a band, slats that run with it and the parts on it; the app needs nothing of its own
  (the belt gets the rubber finish, the two checks their labels).
- `crates/machine-delta-d3/conveyor.toml`: the D3 picking from a belt at 0.3 m/s, a part a second: 60 picks a minute,
  0.444 mm tracking error, none missed. Run as well on the D2, D4 (onto a second belt), linear delta and T-gantry
  from scratch configs.

Found: a faster belt need not raise the RMS torque against a stationary pick, as the task expected. The moves give the
belt term its share of the limits for their whole length, so they get gentler and longer; on the single axis with
inertia alone, a belt at 0.3 m/s lowers the RMS torque by 11 % (hand 0.8876, simulated 0.8877), and on the D3 motor 1
falls 8 % and motor 3 rises 3 %. The test checks the share by hand, whichever sign it has.

Tests: `conveyor.rs` (unit): the intercept against the closed form of a trapezoidal move (0.89395 s), arrival on the
part with its speed within the limits, the plan on the part at the belt's speed during the grip (beginning at 1 s, the
window's start), two parts beyond the window missed, every part missed with the place out of reach.
`sim-machines/tests/conveyor.rs`: the D3 example at a pass a pitch (1 s, 60 / min = 60 / cycle time, none missed),
the measured speed along the belt during the grip 0.3 m/s within tolerance / grip time, a belt at 0.5 s pitch missing at
least 120 - picks a minute, the RMS share on the single axis, belt, slats and parts in the scene, an obstacle at the place
failing `collision`.

Left open:

- Passes start once the belt has its speed (no pick during the ramp); the gripper acts on the plan's time, so with input
  shaping on it closes up to a shaper period early.
- The reserved limits are conservative: a planner that shares the limits along the move would be faster.
- The second belt carries no parts in the scene and has no slots (the part goes wherever the place pose is when the tool
  leaves for it); a vision system's latency or a part's position error are not modelled.
- `check_cycle` checks no conveyor pass before the run (they are planned as the parts come); the live check judges them.
- `sim-cli optimise` changes the unused `[[cycle.segment]]` of a conveyor config, not the conveyor's passes.
- Only the D3 ships a conveyor config.

