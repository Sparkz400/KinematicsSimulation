# Validation

What the simulator has been compared with, and what it has not.

**The short version.** Every check below proves one of two things: that the equations are solved correctly, or that the catalogue data was read correctly. None proves that the parameters describe a real machine. No real machine has been measured.

All checks are automated tests. Run them with `cargo test --workspace`. The workspace has 243 tests. All of them passed on 12 November 2026 (`cargo test --workspace`): 208 in the six crates without the app and the OPC UA link, 35 in `sim-app` and `sim-opcua`.

Test files, abbreviated in the tables:

| Short | File |
| --- | --- |
| core | [`crates/sim-core/src/`](../crates/sim-core/src) (`integrate.rs`, `testkit.rs`) |
| mech, drive, profile, path, identify | [`crates/sim-components/src/`](../crates/sim-components/src) `mech.rs`, `drive.rs`, `profile.rs`, `path.rs`, `identify.rs` |
| axis | [`crates/machine-single-axis/src/tests.rs`](../crates/machine-single-axis/src/tests.rs) |
| gantry | [`crates/machine-t-gantry/src/tests.rs`](../crates/machine-t-gantry/src/tests.rs) |
| suite, control, catalog, review2, review3 | [`crates/sim-machines/tests/`](../crates/sim-machines/tests) `suite.rs`, `control.rs`, `catalog.rs`, `catalog_1fk2.rs`, `catalog_bahr.rs`, `review2.rs`, `review3.rs` |
| cli | [`crates/sim-cli/tests/cli.rs`](../crates/sim-cli/tests/cli.rs) |
| plc | [`crates/sim-opcua/tests/plc.rs`](../crates/sim-opcua/tests/plc.rs) |

- [The standard suite](#the-standard-suite-every-machine)
- [Independent reference](#independent-reference)
- [Hand calculations: mechanics](#hand-calculations-mechanics)
- [Hand calculations: drive](#hand-calculations-drive)
- [Hand calculations: control, profiles and paths](#hand-calculations-control-profiles-and-paths)
- [Hand calculations: analysis and verdicts](#hand-calculations-analysis-and-verdicts)
- [Vendor formulas and vendor documents](#vendor-formulas-and-vendor-documents)
- [Self-consistency](#self-consistency)
- [What is not validated](#what-is-not-validated)

## The standard suite, every machine

`every_machine_passes_the_standard_suite` in suite runs these from core `testkit.rs` on every registered machine: on the default config (round trip, static hold, determinism) and on its loss-free configs (all five). The loss-free configs are the rigid and the elastic model; for the single axis level and inclined by 30° (the inclined ones with tooth stiffness), for the T-gantry also with tooth stiffness, idlers and cable chain on.

| What is compared | Against | Tolerance | Check in `testkit.rs` |
| --- | --- | --- | --- |
| Forward of inverse transformation, on a grid over the workspace | The starting pose | 1e-9 m | `round_trip` |
| The machine at its equilibrium state under the holding torques, for 1 s | It does not move | 1e-7 m, 1e-6 m/s | `static_hold` |
| Total energy minus the work of the holding torques, for 10 s after a torque pulse, losses off | Constant | 0.1 % of the energy the pulse put in | `energy_conservation` |
| Travel after 1 s with the time step halved | Travel with the full step | 1 % | `step_convergence` |
| Two identical runs | Each other | Bit-identical | `determinism` |

The integrator itself: a harmonic oscillator against its closed form (1e-12), and the error falling by a factor of 14 to 18 when the step is halved, as fourth order demands (`harmonic_oscillator_matches_closed_form`, `error_falls_with_fourth_order`, core `integrate.rs`).

## Independent reference

| What is compared | Against | Tolerance | Test |
| --- | --- | --- | --- |
| x and z of the elastic T-gantry over an open-loop move of 0.6 s (a torque push, a pause, a pull back), default config, 601 samples | A second implementation of the same equations in Python, integrated with SciPy's LSODA (rtol 1e-10) instead of fixed-step RK4. Trace committed as [`scipy_trace.csv`](../crates/machine-t-gantry/reference/scipy_trace.csv) | 0.2 % of the travel per axis | `reference_move_matches_scipy`, gantry |

What this covers: belt sections with the energy-consistent form, LuGre friction with load dependence, gearbox with backlash and efficiency, ripple, cogging, tooth meshing, shaft friction, frame mode, gravity. No controller, no drive.

What it does not cover:

- The leaning Z axis. The Rust side of the comparison runs with `effects.tool_flex = false`, because the Python model has none.
- Everything added later and off by default: tooth stiffness, material damping, idlers, cable chain, process force. The hard stops and brakes are in the Rust run but never act in this move.
- The script [`scipy_reference.py`](../crates/machine-t-gantry/reference/scipy_reference.py) runs against the current default config again. It asserts that the fourteen effects it models are on and that the six effects of the machine it does not model are off (tooth stiffness, material damping, idlers, cable chain, process force, temperature); the leaning Z axis is switched off on the Rust side; the switches of drive and controller, the hard stops and the brakes take no part in a move driven by given torques. Run on 11 November 2026 with the pinned packages, it reproduced the committed trace exactly (all 601 samples to the last printed digit).
- It is the same person's reading of the same equations, written twice. It catches integration and coding errors, not a wrong equation.

## Hand calculations: mechanics

| What is compared | Against | Tolerance | Test |
| --- | --- | --- | --- |
| Acceleration under constant torque, rigid model | Force over effective mass | 1e-9 | `constant_torque_gives_force_over_effective_mass`, axis and gantry |
| Holding torque of an inclined axis | m g sin · r / i | exact rigid; 0.1 % elastic | `inclined_axis_needs_holding_torque`, axis |
| Holding torques on Z | ± m_z g r / (2i) | exact rigid; 0.1 % elastic | `static_hold_on_z_needs_opposite_torques`, gantry |
| Belt tension from stretch; damping on the stretch rate | EA · strain; c · rate | 1e-6 N; 1e-9 N | `belt_tension_is_ea_strain_and_pretension_at_rest`, `belt_damping_acts_on_the_stretch_rate`, mech |
| Belt forces | The gradient of the belt's energy | 1e-3 N | `belt_forces_are_the_gradient_of_its_energy`, mech |
| Tooth stiffness in series | 1 / (1/k + 1/k_tooth) | 0.1 % | `teeth_in_mesh_are_a_spring_in_series_with_the_section`, mech |
| Material damping | Four times as hard for a section a quarter as long | 1e-9 N | `material_damping_scales_with_the_section_stiffness`, mech |
| Belt natural frequency (carriage against pulley) | Two masses on a spring k from EA / L | 5 % | `belt_natural_frequency_matches_hand_calculation`, axis |
| The same with tooth stiffness | 1/k = L/EA + 1/k_tooth | 5 % | `belt_natural_frequency_with_tooth_stiffness_matches_hand_calculation`, axis |
| Elastic model with a very stiff belt | The rigid model | 1 % of the travel | `stiff_belt_converges_to_the_rigid_reference`, axis and gantry |
| Slack time of a belt at 20 N pretension | Between 0.5 s and 1 s of a 1 s move; none at 300 N | Range | `a_slack_belt_section_is_counted_and_a_tight_one_is_not`, axis |
| Breakaway force | The stiction level, 20 N | 2 % | `breakaway_force_equals_the_stiction_level`, axis |
| Steady sliding force | Coulomb + viscous · v | 0.1 % | `steady_sliding_force_is_coulomb_plus_viscous`, axis; `lugre_steady_sliding_is_coulomb_plus_viscous`, mech |
| Friction factor with guide load | (preload + load) / (preload + nominal) | 1e-12 | `friction_grows_with_payload_when_load_dependent`, axis; `guide_friction_grows_with_acceleration_and_overhang`, gantry |
| Lost motion on reversal | The configured backlash, 8.7e-4 rad | 1 % | `lost_motion_on_reversal_equals_the_backlash`, axis |
| Gearbox: dead zone, flanks that never pull, efficiency both ways | τη driving, τ/η driven | 1e-9 N m | `gearbox_transmits_nothing_inside_the_backlash`, `gearbox_flanks_in_contact_never_pull`, `gearbox_efficiency_loses_torque_in_the_direction_of_power_flow`, mech |
| Leaning Z axis at two heights: restoring acceleration and tool-tip offset | Two masses on k = 1 / (1/k_lat + h²/k_tilt) | 1e-6 relative; 1e-12 m | `leaning_z_axis_matches_the_two_mass_hand_calculation`, gantry |
| Resonance the machine reports at two heights | 24.6 Hz and 35.6 Hz by hand | 0.05 Hz | `a_shaper_that_follows_the_height_leaves_under_a_tenth_at_both_heights`, gantry |
| Idlers: added mass and drag in X and Z | 12 and 4 idler masses; 6 and 4 friction forces | 1e-9 rigid; 1e-6 elastic | `idlers_add_the_hand_calculated_mass_and_drag`, gantry |
| Cable chain at two positions | Mass 0.175 kg and 0.375 kg; drag and pick-up force | 1e-9 | `cable_chain_adds_the_hand_calculated_force_at_two_positions`, gantry |
| Process force: change of the holding torques | −0.06 N m and +0.02 N m | 2 % of 0.06 N m | `process_force_changes_the_holding_torque_by_the_hand_calculated_amount`, gantry |
| Depth in a hard stop | Force / stiffness: 25 µm and 50 µm | 1 % | `a_carriage_pushed_into_the_hard_stop_rests_at_the_hand_calculated_depth`, axis and gantry |
| Holding brakes: a brake of 0.15 N m holds Z; one of 0.12 N m lets it fall | Holds within 0.1 mm; falls 96 mm in 0.5 s | 3 % | `holding_brakes_hold_the_z_axis_when_they_are_strong_enough`, gantry |
| Single axis, cable chain: extra force to accelerate at 8 m/s² at x = 0.1 m and 0.9 m; coasting at 0.5 m/s; holding torque on a 30° incline | 1.4 N and 3.0 N; (3 + 0.25 · 0.5² / 2) N over the moved mass, rigid and elastic; 0.175 kg and 0.375 kg · g / 2 more to hold | 1e-9; 1e-6 m/s²; 1e-12 and 2e-6 N m | `cable_chain_adds_the_hand_calculated_force_at_two_positions`, axis |
| Single axis, process force of 10 N during a dwell | Holding torque 0.04 N m lower | 2 % | `process_force_changes_the_holding_torque_by_the_hand_calculated_amount`, axis |
| Single axis set upright, holding brake: 0.26 N m holds 6 kg; 0.2 N m lets it fall | Holds within 0.1 mm; falls 136 mm in 0.5 s (136.4 mm simulated) | 3 % | `the_holding_brake_holds_the_upright_axis_when_it_is_strong_enough`, axis |
| Torque at the gearbox output with an efficiency of 0.9, lifting and lowering at a steady speed | The load's torque both ways: 0.7845 N m (axis, rigid), 1.1768 N m (axis, elastic, lowering), ±0.6865 N m (gantry, rigid, and elastic at rest); the motor torque times the ratio reads 1/0.9 or 0.9 of it | 1e-9 N m rigid; 1 % and 0.1 % elastic | `the_gearbox_verdicts_judge_the_torque_at_the_gearbox_output`, axis; `gearbox_torque_is_the_load_at_the_pulleys_whichever_way_it_is_driven`, gantry |
| Friction with stiff bristles on a light contact at speed (1e6 N/m, 12 N, 2.5 m/s) | Stable at the simulator's step; sliding takes 12 N + 20 N s/m · 2.5 m/s = 62 N | 1e-6 N | `lugre_with_stiff_bristles_at_speed_stays_stable_and_slides_at_its_level`, mech |
| Radial load on the pulley shaft | Sum of both section tensions: 570.58 N (vertical axis), 593.46 N (gantry) | 0.02 N | `radial_load_on_the_pulley_shaft_is_the_sum_of_both_section_tensions`, axis; `guide_loads_and_shaft_loads_match_the_hand_calculation`, gantry |
| Guide loads under acceleration | X guide 203.5 N, Z guide 118.6 N | 0.01 N | `guide_loads_and_shaft_loads_match_the_hand_calculation`, gantry |

## Hand calculations: drive

All in drive unless noted.

| What is compared | Against | Tolerance | Test |
| --- | --- | --- | --- |
| Current step response | 63.2 % after the tuned time constant of 0.25 ms; no overshoot | 0.05 A of 1 A; peak below 1.01 | `current_follows_its_setpoint_with_the_tuned_time_constant` |
| Voltage at speed; current at the voltage limit of the winding (190 V) | R i + k_e ω = 49 V; (U − k_e ω) / R = 2.5 A | 1e-3 V; 1e-6 A | `back_emf_costs_voltage_and_the_bus_limits_torque_at_speed` |
| Current for the maximum torque with saturation (1FK2205-4AF: 18 N m at 15.1 A) | The maximum current and that torque; the inverse of the curve at five currents; the top of a curve that turns down | 1e-6 A, 1e-6 N m; 1e-9 A | `the_maximum_torque_takes_the_maximum_current` |
| Voltage and power at the maximum current and 100 rad/s with saturation | R i + (18 / 15.1) · 100 = 179.6 V; copper loss 912 W plus shaft power 1800 W | 1e-3 V; 1e-2 W | `under_saturation_the_electrical_power_is_copper_loss_plus_shaft_power` |
| Every motor and supply row of the S210 catalogue at its rated speed | Back-EMF (static torque constant × rated speed) below sqrt(3/2) × DC link voltage; five rows are above the DC link voltage itself | All 100-odd rows | `every_catalogue_motor_reaches_its_rated_point_inside_the_voltage_limit`, catalog |
| Current loop with computation delay | Stability limit by hand at 5.8 cycles: stable with 4, unstable with 7 | Either side of the limit | `computation_delay_makes_the_current_loop_unstable_at_the_calculated_limit` |
| Speed noise from encoder noise of two counts | 0.1356 rad/s unfiltered, 0.0509 rad/s filtered | 2 % | `speed_from_a_noisy_encoder_has_the_hand_calculated_rms` |
| Winding temperature at the rated current | 100 K in the end; 51.3 K after one time constant; 19.34 K at half the current | 0.2 K; 0.5 K; 0.05 K | `the_winding_warms_to_the_rated_rise_at_the_rated_current` |
| Torque at the maximum current | The lower torque constant | 1e-12 | `saturation_takes_torque_at_high_current_only` |
| DC link voltage after 4.1 J of braking into 470 µF, from 190 V | 231.40 V; held at 250 V with the resistor burning 6.096 J; a 20 W resistor trips between 0.32 s and 0.34 s; the winding then gets sqrt(3/2) × 231.40 V | 0.01 V; 1e-3 J | `braking_energy_raises_the_dc_link_by_the_hand_calculated_voltage` |
| DC link over a cycle of the single axis, from 155.13 V | 2.983 J per braking to 191.7 V; resistor mean power with 4.7 µF; a 5 W resistor trips at 257.6 V | 0.01 J; 0.1 V; 1 % | `the_dc_link_takes_braking_energy_the_resistor_the_rest_and_too_small_a_resistor_trips`, axis |
| Two drives, one braking 41 W and one drawing 23.5 W for 0.1 s, 470 µF each, from 190 V | Separate links: 231.40 V and 190 V. Shared: sqrt(2 · (16.967 + 1.75) J / 940 µF) = 199.56 V; with the chopper at 195 V the one resistor burns 0.845 J; one trip for both; links joined at 231.40 V and 190 V settle at 211.72 V | 0.01 V; 0.002 J | `a_shared_dc_link_rises_by_the_net_energy_of_its_drives`, drive |
| Shared DC link over the default cycle of a machine with two drives | The same voltage and resistor power on every drive at every step; highest voltage no higher than with separate links (224.06 V against 224.65 V on the gantry) | Exact | `a_shared_dc_link_is_one_voltage_for_all_drives_of_a_machine`, control |
| Notch filter | Removes its centre frequency, passes the rest | Gain below 0.01 at 60 Hz; within 2 % of 1 at 5 Hz and 600 Hz | `notch_removes_its_centre_frequency_and_passes_the_rest` |
| Position loop, velocity loop, torque limit, encoder steps, delay of one velocity cycle | Their defining formulas | 1e-12 | `position_loop_is_kv_times_following_error_plus_feedforward`, `velocity_loop_is_pi_with_torque_feedforward`, `torque_limit_follows_the_torque_speed_curve`, `encoder_resolution_quantises_position_and_speed`, `the_torque_setpoint_acts_one_velocity_cycle_late` |

## Hand calculations: control, profiles and paths

| What is compared | Against | Tolerance | Test |
| --- | --- | --- | --- |
| Following error at constant speed, no feedforward, every machine | v / Kv | 1 % | `following_error_at_constant_speed_is_v_over_kv`, control |
| The same with velocity feedforward | Zero; half with a factor of 0.5 | 2 % of v / Kv | `velocity_feedforward_removes_the_following_error`, control |
| Step response and bandwidth of the position loop | First-order lag with 1 / Kv: 63.2 % after 1 / Kv; bandwidth Kv / 2π; −45° there | 0.05; 10 %; 10° | `step_and_frequency_response_show_the_position_loop_gain`, axis |
| `profile.cycle_s` changed to 2 ms and to 0.5 ms while an axis moves, with and without input shaping, every machine | The setpoint of an untouched run, within three cycles of travel; no step above one cycle's travel | As stated | `changing_the_profile_cycle_on_the_way_does_not_move_the_setpoint`, control |
| Dwell and gripper with input shaping, every machine | They start with the shaped setpoint on the target | 1 µm | `with_input_shaping_a_dwell_starts_when_the_setpoint_has_arrived`, control |
| Rigid and elastic model exchanged at cruise speed, every machine | Tool position and speed carried over; torque below half the limit; the tool stays with its setpoint | 1e-12; 0.2 mm | `switching_the_model_at_speed_carries_the_motion_over`, control |
| A cycle of one segment | One pass lasts the dwell | 2.1 ms | `a_cycle_of_one_segment_is_analysed`, control |
| A move with a fieldbus delay of three cycles | The undelayed move 3 ms later; 3.00 mm behind at 1 m/s | 1e-12 m; 1 % | `a_fieldbus_delay_shifts_the_move_by_its_cycles`, axis |
| S-curve: velocity, acceleration, jerk in every regime | Their limits | 1e-9 relative | `velocity_acceleration_and_jerk_stay_within_limits`, profile |
| Duration of a long move | 4 t_j + 2 t_a + cruise by hand | 1e-12 s | `long_moves_use_the_limits_fully`, profile |
| Synchronised axes | Finish together, on a straight line, each within its limits | 1e-12 | `synchronised_axes_finish_together_within_their_limits`, profile |
| Blended cycles of every machine | Axis speed and acceleration within their limits | 1e-9 relative | `blended_default_cycles_stay_within_the_axis_limits`, control |
| Planned path (line, arc, line): rates of both axes by differences | The axis limits; every pose on the path | 0.1 %; 1e-12 m | `a_planned_path_stays_within_the_axis_limits_and_on_its_pieces`, path |
| Speed on the arc and into it | 0.4 m/s; 0.0707 m/s by hand | 1e-3 m/s | the same test |
| The same path with transition curves, tolerance 0.1 mm and 1 mm: deviation, length of the curves and speed on them | e = L² / (24 r): 0.1 mm with L = sqrt(24 e r) = 15.49 mm and (0.4 j r L)^(1/3) = 0.3957 m/s; 0.1067 mm with L = v³ / (0.4 j r) = 16 mm at 0.4 m/s | 0.2 %; 0.5 %; 2 mm/s | `transition_curves_let_the_path_enter_an_arc_at_speed`, path |
| The same: rates of both axes by differences; every pose against the given line, arc and line; duration | The axis limits; within the reported deviation and reaching it; 1.590 s against 1.828 s | 0.1 %; 1 nm; below 0.9 | the same test |
| Two arcs between two lines, turning opposite ways and the same way, tolerance 0.1 mm | No step in position, direction or curvature at any junction; the axis limits; ends on the target; 1.539 s against 1.847 s and 1.481 s against 1.824 s | 1e-11 m, 1e-9, 1e-7 1/m; 0.1 % | `transition_curves_join_two_arcs`, path |
| A wide arc (0.25 m) into a tight one (1.72 mm, 1 rad) with a tolerance of 1 mm, which allows curves longer than the tight arc can take | The planned path turns as far as the given one, 1.35 rad (it turned a whole circle more) | 1e-9 rad | `curves_too_long_for_a_tight_arc_do_not_add_a_turn`, path |
| A line that ends on the end of a stroke | Every pose within the range, the last one the given point (it ended 2.8e-17 m beyond) | Exact | `a_path_onto_the_end_of_a_stroke_stays_within_the_range`, path |
| Both planners on the path with transition curves | Every pose within the reported 0.1 mm of the given path, and reaching it | 10 µm (the start pose) | `both_planners_time_the_path_with_its_transition_curves`, gantry |
| Path error and distance behind the setpoint on a circle at constant speed, no feedforward | R (1 − gain) and R (ω / Kv) gain, with gain = 1 / sqrt(1 + (ω / Kv)²) | 10 % | `path_error_on_a_circle_matches_the_hand_calculation`, `a_circle_timed_by_the_planner_gives_the_hand_calculated_path_error`, gantry |
| Time-optimal line, jerk limit 1e5 m/s³ and 100 m/s³ | Distance / v + v / a + a / j = 0.714 s; distance / v + 2 sqrt(v / j) = 0.861 s, with the acceleration peaking at sqrt(v j) = 11.2 m/s² | 1.5 ms; 0.5 % | `a_time_optimal_line_takes_the_hand_calculated_time`, axis |
| Time-optimal line-arc-line: torque of the rigid model, motor speed and axis jerk at every planned pose, by differences | Within the torque reserve, the rated speed and the jerk limits; one of them used to 97 % or more at over 60 % of the poses (64 %: jerk 57 %, speed 10 %; with a 0.6 N m motor and 2000 m/s³: torque 30 %, speed 32 %, jerk 4 %); 1.429 s against 1.828 s within the axis limits | 0.1 %; below 0.8 | `the_time_optimal_plan_is_shorter_keeps_a_limit_active_and_stays_on_the_torque_speed_curve`, gantry |
| The same plan on the full model | Utilisation below 0.6 (0.18); speed below 1.03 × rated; path error below 1 mm | As stated | the same test |
| Time-optimal line, quarter turn of 5 mm radius, line, with a path tolerance of 0, 0.1 mm and 1 mm | The same three limits at every planned pose; ends on the end point; utilisation on the full model below 1 (0.18; it was 10.5) | 0.1 %; 1 nm | `a_time_optimal_plan_through_a_tight_arc_stays_within_torque_speed_and_jerk`, gantry |
| 200 random paths of two to five tangent lines, arcs and splines (radius 2 mm to 300 mm, pieces 1 mm to 400 mm, splines of one to three via points turning by up to 0.6 rad each), half with a tolerance, half with a 0.6 N m motor and 2000 m/s³ | The same three limits at every planned pose (most used: torque 1 + 2e-11, speed 1 + 2e-13, jerk 1 + 8e-9); ends on the end point | 0.1 %; 1 nm | `time_optimal_plans_of_random_paths_stay_within_torque_speed_and_jerk`, gantry |
| A single spline piece between two points at rest | The straight line between them, the distance along it exact | 1e-12 m; 1e-14 m | `a_spline_passes_through_its_via_points_within_the_axis_limits`, path |
| Four spline paths (between two lines; alone; out of an arc into an arc; an S of via points 20 mm apart): geometry | Every via point is the end of a span; no step in position or direction anywhere, none in curvature between spans | 1e-12 m; 1e-9; 1e-6 1/m | the same test |
| The same: rates of both axes by differences; nearest planned pose to every via point | The axis limits; half of what the tool travels in a cycle | 0.1 %; 0.4 mm | the same test |
| Tool swing after a traverse with input shaping | Under a tenth of the swing without | Factor 0.1 | `input_shaping_leaves_under_a_tenth_of_the_tool_swing`, `a_shaper_that_follows_the_height_leaves_under_a_tenth_at_both_heights`, gantry |

## Hand calculations: analysis and verdicts

| What is compared | Against | Tolerance | Test |
| --- | --- | --- | --- |
| Cycle time, peak torque, RMS torque, peak speed, utilisation of a reference cycle (rigid, loss-free, ideal drive) | The S-curve cycle by hand | 2 ms; 2 %; 2 %; 1 %; 2 % | `reference_cycle_torque_matches_hand_calculation`, axis |
| Energy in minus energy fed back | The copper loss | 3 % | the same test |
| Settled winding rise | The formula from the RMS current | 1e-9 K | the same test |
| Frequency sweep of the rigid machine | A pure inertia: gain 1 / (J ω), phase −90° | 1 %; 1° | `frequency_sweep_of_the_rigid_machine_is_a_pure_inertia`, axis |
| Dip in the frequency sweep | The load's resonance on the belt by hand | 5 % | `frequency_sweep_shows_the_belt_resonance`, axis |
| Verdict values of the reference cycle | Peak torque at the gearbox output 0.85 N m and its cubic mean 0.337 N m by hand (4.25 kg · 0.02 m · 10 m/s² and · (100 / 1.6)^(1/3) m/s²; the motor torque times the ratio gives 1.225 N m); input speed 2387 r/min; radial load 600 N; guide load 4 kg · g; DC link room 6.204 J | 2 %; 1 %; exact; 1e-9; 1e-3 J | `verdicts_match_the_hand_calculation_and_fail_where_a_part_is_too_small`, axis |
| Guide life | 47.3 million km for a cubic mean of 50.92 N | 1 % | the same test |
| Undersized parts | The matching verdicts fail, and only those | Exact list | the same test |
| Band on the gearbox peak torque for a moved mass of 3, 4 and 6 kg | 0.65, 0.85, 1.25 N m by hand | 2 % | `a_bound_on_the_moved_mass_gives_the_hand_calculated_band_and_fails_the_verdict_it_reaches`, axis |
| Sampled bands: carriage 4 to 6 kg and payload 2 to 4 kg, a gearbox that allows 1.75 N m | Each mass alone at its bound: 1.65 N m, passes. Both sampled together, 40 runs: up to 2.05 N m, fails; 72 % of the runs pass by hand (70.7 % and 68.3 % with two seeds) | 2 %; 0.1 | `sampled_bands_find_the_two_masses_that_fail_only_together`, axis |
| The EXCT-30 config in the worst case of both band modes (61 bounded values one at a time; 60 samples) | Every check passes; slackest belt section above 100 N (139 N and 123 N), settling below 0.07 mm (0.041 and 0.049), shaft load below 1500 N (1205 and 1206); cycle 1.934 s | As stated | `the_exct_30_config_passes_every_check_in_the_worst_case_of_both_band_modes`, catalog |
| A tool pose mapped into the 3D scene, at five poses of each machine, rigid and elastic | Where the scene draws the tool part (flange, carriage); by hand (0.45, −0.05, −0.592) m and (0.45, −0.03, 0.06) m | 1e-12 m | `the_scene_point_of_a_tool_pose_is_where_the_scene_draws_the_tool`, gantry; `..._the_carriage`, axis |
| Fit of a noisy simulated trace of a changed machine (1.5 kg heavier, friction × 1.4 and × 1.3, belt stiffness × 0.7) | The changes that were made | Within 3 standard errors; stiffness band below 0.03 | `the_fit_recovers_changed_masses_friction_and_belt_stiffness_from_noisy_measurements`, axis |
| Fit on the T-gantry, where both motors move both axes | The changes per axis | Within 3 standard errors | `the_fit_tells_the_two_axes_apart`, gantry |
| Resonance read off a synthetic dip | The frequency and damping it was built with | The sweep's spacing; 5 % | `resonance_reads_frequency_and_damping_off_a_known_dip`, identify |

The guide life row checks that the code computes the catalogue formula. It does not check the formula against a guide.

## Vendor formulas and vendor documents

| What is compared | Against | Tolerance | Test |
| --- | --- | --- | --- |
| Motor torque of the EXCT-30 config at 45 degrees travel, from the simulator's rigid model | Festo's torque formula with the catalogue's coefficients, for several accelerations and loads | 2 % | `exct_30_config_agrees_with_the_catalogue`, gantry |
| The `[catalogue]` values of `exct-30.toml` | `catalog/festo/exct.toml` and `catalog/siemens/motors_s210.csv` | 0.1 % or exact | the same test |
| Festo's formula coefficients | The T-gantry's own mechanics from the pulley diameter | 1 % to 3 % | `festo_exct_values_agree_with_its_own_geometry`, catalog |
| 1FK2 data from the S120 configuration manual | The same motors in the S210 catalogue, read by another script | 5 % to 6 % | `siemens_1fk2_data_agrees_between_the_s120_manual_and_the_s210_catalogue`, catalog |
| 1FT2 data from the S120 configuration manual | The same motors in the S210 catalogue | 5 % to 6 % | `siemens_1ft2_data_agrees_between_the_s120_manual_and_the_s210_catalogue`, catalog |
| 1FK7 and 1FT7 data sheets | Catalogue D 21.4: pole count; torque constant = static torque / stall current | Exact; 8 % | `siemens_s120_data_sheets_agree_with_the_catalogue`, catalog |
| The data sheets in themselves | Voltage constant against torque constant; L / R against the electrical time constant | 15 % | the same test |
| All other catalogue files | Their own physics; see [catalogue](catalogue.md#how-the-files-were-extracted-and-checked) | Per test | catalog |
| A motor and a gearbox by name | The same values put in by hand | Identical machine | `a_motor_by_name_gives_the_values_of_the_hand_written_config`, `a_gearbox_by_name_sets_ratio_stiffness_backlash_and_ratings`, `parts_named_in_a_config_behave_like_their_numbers`, catalog |
| Velocity gain the search gives a lighter motor | The gain by hand, 0.6 → 0.31 | Per test | `the_search_retunes_the_velocity_loop_for_a_lighter_rotor`, catalog |
| What the gearbox search rules out by catalogue numbers | Ratios 7 and above for a 3000 r/min motor, by hand; each runs above its rated speed when simulated | Exact list | `the_gearbox_search_rules_out_by_catalogue_numbers_only_what_would_fail`, catalog |

The Festo comparison is the one place where the simulator's numbers meet a vendor's numbers for a whole machine. It covers the rigid torque (inertia and gravity), with the carriage mass of the config chosen so that it fits. It does not cover friction, which Festo gives only as a graph.

## Self-consistency

These compare the simulator with itself. They prove repeatability and plumbing, not physics.

| What | Test |
| --- | --- |
| The same run twice is bit-identical, with control and with sensor noise | `the_same_run_twice_is_bit_identical`, control |
| Halving the integration step changes a controlled move with all effects on by less than 1 % | `halving_the_integration_step_changes_a_controlled_move_by_less_than_one_percent`, control |
| Every effect switch changes the traces | `every_effect_switch_changes_the_traces`, control |
| A trace written by `run` replays onto itself: torque and following error within 0.5 % (RMS) and 5 % (peak) of their largest value; the fit finds the config's own masses and friction within 2 % | `a_trace_from_run_replays_onto_itself_and_fits_its_own_config`, cli |
| The EXCT-30 cycle stays below 0.8 of the torque-speed curve and of the rated torque, and the belt does not go slack | `exct_30_cycle_stays_within_motor_and_belt_limits`, catalog |
| The optimiser's result is shorter by at least 5 % and passes every verdict when run again | `the_optimiser_shortens_the_cycle_within_the_verdicts`, axis |
| A PLC-commanded move runs and reports position and following error, for every machine | `a_plc_commanded_move_runs_and_reports_position_and_following_error`, plc |
| Every numeric value of both default configs at -1, 0, NaN and infinity, one at a time: the config is refused or the cycle runs with finite signals; nothing panics | `no_single_bad_number_panics_the_simulation_or_fills_it_with_nan`, review2 |
| A shaper changed at speed leaves the largest setpoint step at speed times profile cycle (0.8 mm on the gantry's X axis) | `a_shaper_changed_at_speed_does_not_make_the_setpoint_jump`, review2 |
| A gear ratio or pulley radius changed by 1 % at standstill and at speed: belt tensions within 10 % of the pretension and the tool within 0.1 mm of a run left alone | `a_changed_ratio_or_pulley_radius_does_not_tear_the_machine`, review2 |
| A path planned from every corner of the workspace, where the setpoint stands up to 44 µm beyond the range, is run and arrives; one that leaves the range by 1 mm is refused | `a_path_from_an_end_of_a_stroke_is_run`, review3 |
| A stream of setpoints at the speed limit and at 0.95 of it, one per interval: none is refused, the setpoint moves at the stream's speed (1e-9 relative) and ends on the last value sent; the same with the setpoints a cycle and a half early and late in turn; a jump of two intervals is refused | `a_stream_of_setpoints_at_the_speed_limit_is_followed`, review3 |
| The app's tables show what the library functions behind the command line give, for every machine: the cycle with its extra figures, the error bands, the ranked motor search, a planned path, a replayed and fitted trace | `the_cycle_is_analysed_and_judged_in_the_app`, `the_checks_get_error_bands_in_the_app`, `the_app_searches_motors_and_names_one_in_the_config`, `a_path_is_planned_run_and_analysed_in_the_app`, `a_trace_is_replayed_and_fitted_in_the_app`, app |
| A result of the app is not applied to a config changed since: a motor of the search, a planned path, the optimiser's settings | the same tests and `the_app_optimises_the_cycle_and_applies_the_settings`, app |
| A drive that trips is reported once per trip, for the running machine and for an analysed cycle | `a_drive_that_trips_is_reported`, app |
| Dependencies point downwards; every version is pinned; no machine is named outside its crate | [`architecture.rs`](../crates/sim-machines/tests/architecture.rs) |
| Every label exists in German and English | [`i18n.rs`](../crates/sim-machines/tests/i18n.rs), and in the app's and the link's tests |

## What is not validated

**Against a real machine: nothing.**

- No measured drive trace has been compared with the simulation. The "Real traces" check of the project plan (stage D4) needs a real axis and does not exist.
- The test runs on a real axis (stage D2) have not been made.
- Trace import, replay and fit have only seen traces the simulator made itself, with added noise. Column names, units and sampling of a real drive's export are untested.
- The following error in particular is harder to match than torque, because drive vendors do not document their internal filters.

**Parameters.** These are estimates in every config, and nothing checks them:

- guide friction (Coulomb, stiction, viscous, Stribeck speed, bristles, preload)
- belt damping, pretension, tooth-mesh stiffness, tooth force
- frame mass, stiffness and damping; lateral and tilt stiffness of the Z guide
- motor shaft friction, ripple, cogging
- winding resistance and inductance of the 1FK2 (Siemens does not publish them)
- DC link capacitance, braking threshold, resistor powers and trip level of the S210
- idler, cable chain, hard stop and brake values; sensor noise levels
- guide and pulley shaft ratings and the belt tension rating of the EXCT (Festo publishes none; the belt figure is from another maker's belts of the same build)

The default configs of both machines are placeholders throughout.

**Models that are implemented but not compared with anything outside the code:**

- Guide life. It is the catalogue formula L10 = (C / P)³ · 50 km with an estimated load. No endurance data.
- The thermal model beyond its own steady state: one thermal mass, no measured heating curve.
- Torque saturation: the shape of the curve between zero and maximum current (a square law) is an assumption; only the end point is from the catalogue.
- The DC link: checked against energy bookkeeping by hand, not against a drive. The S210's real chopper behaviour is not known.
- Drive delays and filters: checked against control theory, not against a drive.
- Sensor noise: Gaussian by assumption.
- The rule that scales the velocity gain for a part by name and in the motor search: a rule of thumb, checked on the EXCT-30 for the 1FK2 only.
- The holding brake: a spring up to the brake torque, then slipping with it. Checked against its own hand calculation, not against a brake.
- The shared DC link: energy bookkeeping by hand, with the capacitances simply added. Not against an S120 line-up; a Line Module's own capacitance, its regenerative feedback and the busbars are not modelled.
- Splines: the geometry and the axis rates are checked, not that this is the curve a given controller would lay through the same points.
- The resonance the T-gantry reports for the input shaper leaves out the frame mode and the control loops. The simulation swings at 25 Hz and 37 Hz where it reports 24.6 Hz and 35.6 Hz.

**The SciPy reference** does not include the leaning Z axis or any effect added after phase 2 (see [above](#independent-reference)).

**Combinations.** Each effect has a test of its own. That all effects together give the right result is checked only by energy conservation (losses off), step-size convergence and determinism, not against an independent number.

**Error bands** vary one parameter at a time, or with `--samples` all at once in a Latin hypercube. The sampled mode covers interactions as far as its runs reach: it is tested on one case with a known answer (two masses), and no number of samples proves a worst case. On the EXCT-30 it found one that the one-at-a-time mode cannot: the friction model blowing up with two of the X guide's values at their bounds together (fixed, see the model reference).

**Front ends:**

- OPC UA: tested against a client written with the same library. Not against a Siemens, Beckhoff or CODESYS controller; no security profiles; no check of timing jitter.
- App: orbit camera, view presets, panels and buttons by mouse, typing into the file and search fields, and how the motion looks, were not checked by a person. Neither were the two lines the 3D view draws for a planned path (planned, and travelled): their points are tested in the model, the drawing was not looked at. Screenshots in English and German of every part of the right panel were inspected. What the app computes is tested without its window, against the library functions the command line uses (`sim-app/tests/app.rs`).

**Catalogue data.** Cross-checks find values in the wrong row or column. They cannot find a number misread into another plausible number where nothing ties it to a second value. The misprints found are listed in the [catalogue](catalogue.md#misprints-and-inconsistencies-found-in-the-vendor-documents) document; there may be more.
