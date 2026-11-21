# Validation

What the simulator has been compared with, and what it has not.

**The short version.** Every check below proves one of two things: that the equations are solved as documented, or that the catalogue data was read correctly. The independent implementations by other people (MuJoCo for the arms and the D3's mass split) can also catch a wrong equation for the model as documented. None proves that the parameters describe a real machine. The project has measured no machine of its own; one public measurement of a ball screw axis, made by ISW Stuttgart, is compared [below](#against-measured-data), and it is the only measured machine.

All checks are automated tests. Run them with `cargo test --workspace`, or as [CONTRIBUTING.md](../CONTRIBUTING.md) does with cargo-nextest (the Python tests with pytest, see the CI workflow). The workspace has **1979 tests**: in the final full run of 16 November 2026 (`cargo nextest run --workspace --cargo-profile quick --exclude sim-py --profile full`) 1920 of 1920 passed, the 29 OPC UA tests against the wall clock passed run one at a time, and the 30 long sweeps passed (see CONTRIBUTING.md); the Python tests are counted apart. Beside them: property tests (proptest) of every reader of untrusted input, fuzz targets in [`fuzz/`](../fuzz) run weekly by a workflow, and benchmarks per machine (`crates/sim-machines/benches/simulate.rs`, not run in CI).

Test files, abbreviated in the tables:

| Short | File |
| --- | --- |
| core, sim-core | [`crates/sim-core/src/`](../crates/sim-core/src) (`integrate.rs`, `testkit.rs`, `config.rs`), [`crates/sim-core/tests/config_props.rs`](../crates/sim-core/tests/config_props.rs) |
| mech, drive, profile, path, identify, analysis, joint, base, fault, gripper, ... | [`crates/sim-components/src/`](../crates/sim-components/src) `mech.rs`, `drive.rs`, `profile.rs`, `path.rs`, `identify.rs`, `analysis.rs`, `joint.rs`, `base.rs`, `fault.rs`, `gripper.rs`, `stepper.rs`, `induction.rs`, `pneumatic.rs`, `stop.rs` and the other modules' own tests; [`crates/sim-components/tests/`](../crates/sim-components/tests) |
| axis, single axis | [`crates/machine-single-axis/src/tests.rs`](../crates/machine-single-axis/src/tests.rs) |
| gantry | [`crates/machine-t-gantry/src/tests.rs`](../crates/machine-t-gantry/src/tests.rs) |
| delta | [`crates/machine-delta-d3/src/tests.rs`](../crates/machine-delta-d3/src/tests.rs) |
| delta2 | [`crates/machine-delta-d2/src/tests.rs`](../crates/machine-delta-d2/src/tests.rs) |
| linear delta | [`crates/machine-delta-linear/src/tests.rs`](../crates/machine-delta-linear/src/tests.rs) |
| delta4 | [`crates/machine-delta-d4/src/tests.rs`](../crates/machine-delta-d4/src/tests.rs) |
| H4, parallel SCARA, hexapod, tripod, cable robot, wafer handler | `src/tests.rs` of [`machine-delta-h4`](../crates/machine-delta-h4/src/tests.rs), [`machine-parallel-scara`](../crates/machine-parallel-scara/src/tests.rs), [`machine-hexapod`](../crates/machine-hexapod/src/tests.rs), [`machine-tripod`](../crates/machine-tripod/src/tests.rs), [`machine-cable-robot`](../crates/machine-cable-robot/src/tests.rs), [`machine-wafer-handler`](../crates/machine-wafer-handler/src/tests.rs) |
| joint kit | [`crates/machine-joint-kit/src/tests.rs`](../crates/machine-joint-kit/src/tests.rs) |
| overhead crane | [`crates/machine-overhead-crane/src/tests.rs`](../crates/machine-overhead-crane/src/tests.rs) |
| crank, toggle | [`crates/machine-crank-cam/src/tests.rs`](../crates/machine-crank-cam/src/tests.rs), [`toggle_tests.rs`](../crates/machine-crank-cam/src/toggle_tests.rs) |
| indexing table | [`crates/machine-crank-cam/src/indexer_tests.rs`](../crates/machine-crank-cam/src/indexer_tests.rs) |
| winder | [`crates/machine-winder/src/tests.rs`](../crates/machine-winder/src/tests.rs) |
| reference (per crate) | `tests/reference.rs` of [`machine-single-axis`](../crates/machine-single-axis/tests/reference.rs), [`machine-delta-d3`](../crates/machine-delta-d3/tests/reference.rs), [`machine-delta-d2`](../crates/machine-delta-d2/tests/reference.rs), [`machine-delta-linear`](../crates/machine-delta-linear/tests/reference.rs), [`machine-joint-kit`](../crates/machine-joint-kit/tests/reference.rs) |
| sim-netlist | [`crates/sim-netlist/src/`](../crates/sim-netlist/src) |
| suite, control, catalog, review2, review3, sweep, effects, views, electrical, belt_and_heat, life, conveyor, collision, fit_joints, machine_files, motor_kinds, pneumatic, stop, time_optimal, bad_numbers, ... | [`crates/sim-machines/tests/`](../crates/sim-machines/tests): one file per name (`suite.rs`, `control.rs`, ...), the catalogue checks in `catalog*.rs`, the machine files' in `machine_file*.rs`, the property tests in `*_props.rs` |
| cli | [`crates/sim-cli/tests/`](../crates/sim-cli/tests) `cli.rs`, `identify.rs`, `examples.rs` |
| app, window | [`crates/sim-app/tests/app.rs`](../crates/sim-app/tests/app.rs), [`crates/sim-app/src/window_tests.rs`](../crates/sim-app/src/window_tests.rs) |
| plc | [`crates/sim-opcua/tests/`](../crates/sim-opcua/tests) `plc.rs`, `plcopen.rs`, `interop.rs`, `args.rs`, `soak.rs` |
| fmi, sim-fmi lib | [`crates/sim-fmi/tests/`](../crates/sim-fmi/tests) `fmi.rs`, `fmi3.rs`, `fmi_props.rs`, and `crates/sim-fmi/src/lib.rs` |
| Python | [`crates/sim-py/tests/`](../crates/sim-py/tests) (pytest) |

- [The standard suite](#the-standard-suite-every-machine)
- [Independent reference](#independent-reference)
- [Hand calculations: mechanics](#hand-calculations-mechanics)
- [Hand calculations: drive](#hand-calculations-drive)
- [Hand calculations: control, profiles and paths](#hand-calculations-control-profiles-and-paths)
- [Hand calculations: analysis and verdicts](#hand-calculations-analysis-and-verdicts)
- [Hand calculations: delta picker D3](#hand-calculations-delta-picker-d3)
- [Hand calculations: planar delta picker D2](#hand-calculations-planar-delta-picker-d2)
- [Hand calculations: linear delta](#hand-calculations-linear-delta)
- [Hand calculations: four-axis delta picker D4](#hand-calculations-four-axis-delta-picker-d4)
- [Hand calculations: wafer handler](#hand-calculations-wafer-handler)
- [Hand calculations: arms in tool space](#hand-calculations-arms-in-tool-space)
- [Hand calculations: tripod](#hand-calculations-tripod)
- [Hand calculations: arm on a track](#hand-calculations-arm-on-a-track)
- [Hand calculations: picking from a conveyor](#hand-calculations-picking-from-a-conveyor)
- [Hand calculations: pneumatic cylinder](#hand-calculations-pneumatic-cylinder)
- [Hand calculations: toggle press](#hand-calculations-toggle-press)
- [Hand calculations: winder with dancer](#hand-calculations-winder-with-dancer)
- [Hand calculations: overhead crane](#hand-calculations-overhead-crane)
- [Friction feedforward in the shipped configs](#friction-feedforward-in-the-shipped-configs)
- [Vendor formulas and vendor documents](#vendor-formulas-and-vendor-documents)
- [Self-consistency](#self-consistency)
- [Against measured data](#against-measured-data)
- [What is not validated](#what-is-not-validated)

## The standard suite, every machine

`every_machine_passes_the_standard_suite` in suite runs these from core `testkit.rs` on every registered machine: on the default config (round trip, static hold, determinism, loss budget) and on its loss-free configs (all six). The loss-free configs are the rigid and the elastic model; for the single axis level and inclined by 30° (the inclined ones with tooth stiffness), for the T-gantry also with tooth stiffness, idlers and cable chain on.

| What is compared | Against | Tolerance | Check in `testkit.rs` |
| --- | --- | --- | --- |
| Forward of inverse transformation, on a grid over the workspace (the poses of the axes' box the machine can go to: on the delta its cylinder with the spherical bottom) | The starting pose | 1e-9 m | `round_trip` |
| The machine at its equilibrium state under the holding torques, for 1 s | It does not move | 1e-7 m, 1e-6 m/s | `static_hold` |
| Total energy minus the work of the holding torques, for 10 s after a torque pulse, losses off | Constant | 0.1 % of the energy the pulse put in | `energy_conservation` |
| Travel after 1 s with the time step halved | Travel with the full step | 1 % | `step_convergence` |
| Two identical runs | Each other | Bit-identical | `determinism` |
| Work of the motor torques (Σ τ Δφ) over 1 s after a strong pulse (half the acceleration limit for 0.1 s), with the default config's losses on | Change of the machine's energy plus the time integral of every loss power it reports (`power.*`), summed on the integrator's sub-steps of 15.6 µs (on whole steps, the linear delta's stop damper, which takes c v² the moment a carriage touches, put 0.042 J of 18.3 J into one step); the energy parts (`energy.*`) add up to the energy | 0.1 % of the work either way (measured 9e-5 on the T-gantry, 1.4e-6 on the single axis); 1e-9 | `loss_budget` |

The integrator itself: a harmonic oscillator against its closed form (1e-12), and the error falling by a factor of 14 to 18 when the step is halved, as fourth order demands (`harmonic_oscillator_matches_closed_form`, `error_falls_with_fourth_order`, core `integrate.rs`).

## Independent reference

| What is compared | Against | Tolerance | Test |
| --- | --- | --- | --- |
| x and z of the elastic T-gantry over an open-loop move of 0.6 s (a torque push, a pause, a pull back), default config, 601 samples | A second implementation of the same equations in Python, integrated with SciPy's LSODA (rtol 1e-10) instead of fixed-step RK4. Trace committed as [`scipy_trace.csv`](../crates/machine-t-gantry/reference/scipy_trace.csv) | 0.2 % of the travel per axis | `reference_move_matches_scipy`, gantry |

What this covers: belt sections with the energy-consistent form, LuGre friction with load dependence, gearbox with backlash and efficiency, ripple, cogging, tooth meshing, shaft friction, frame mode, gravity. No controller, no drive.

What it does not cover:

- The leaning Z axis. The Rust side of the comparison runs with `effects.tool_flex = false`, because the Python model has none.
- Everything added later and off by default: tooth stiffness, material damping, idlers, cable chain, process force. The hard stops and brakes are in the Rust run but never act in this move.
- The script [`scipy_reference.py`](../crates/machine-t-gantry/reference/scipy_reference.py) runs against the current default config again. It asserts that the fourteen effects it models are on and that the six effects of the machine it does not model are off (tooth stiffness, material damping, idlers, cable chain, process force, temperature); the leaning Z axis is switched off on the Rust side (the gearboxes' mesh friction is off in its config); the switches of drive and controller, the hard stops and the brakes take no part in a move driven by given torques. Run on 11 November 2026 with the pinned packages, it reproduced the committed trace exactly (all 601 samples to the last printed digit). On 16 November it took the belt's runs (`belt_mass`: their mass beyond the pulleys' lumps, their speed terms, the whole belt swaying with the frame, so carriage, Z axis and frame are solved together) and was run again to write the trace.
- It is the same person's reading of the same equations, written twice. It catches integration and coding errors, not a wrong equation.

### The overhead crane's sway against SciPy

| What is compared | Against | Tolerance | Test |
| --- | --- | --- | --- |
| The load's y and its sway (its offset from under the sheave) over a trapezoidal trolley move driven open loop by torque steps (speed up 1.5 s, cruise 2 s, brake 1.5 s, let go; 10 s on a 3 m rope with the 265 kg load), default config but ripple, 201 samples | The same equations written again in Python ([`scipy_reference.py`](../crates/machine-overhead-crane/reference/scipy_reference.py)) and integrated with SciPy's LSODA (rtol 1e-11) instead of fixed-step RK4: the elastic rope with its damping and slack, the spherical pendulum, the three rigid drives with their gearboxes' loss factor and the motors' friction, the wheels' load-dependent rolling. Trace committed as [`scipy_trace.csv`](../crates/machine-overhead-crane/reference/scipy_trace.csv) (largest sway 0.313 m, 1.05 m of travel) | 1e-4 of the travel for y, 1e-3 of the largest sway | `a_trapezoidal_trolley_move_sways_as_scipy_says`, overhead crane |

The script asserts that the six effects it models are on (gravity, payload, rope damping, rolling resistance, motor
friction, gearbox efficiency) and four it does not are off (gearbox drag, motor stiction, thermal feedback, base
compliance); the Rust side switches the torque ripple off. Run on 16 November 2026 with the pinned packages
(numpy 2.5.4, scipy 1.18.1). As the T-gantry's, it is the same person's reading of the same equations written twice: it
catches integration and coding errors, not a wrong equation. The rigid rope (`effects.rope_stretch` off) is checked
against the elastic one instead: at the same pose a rope a hundred times stiffer sways within 4.5e-5 of the rigid
one's largest sway, the shipped rope 4.75e-3 off (its stretch hangs the load 5.1 mm lower)
(`a_stiff_rope_converges_to_the_rigid_reference`).

### The arms against MuJoCo

| What is compared | Against | Tolerance | Test |
| --- | --- | --- | --- |
| Per URDF (`six-axis`, `scara`, `five-axis`, `seven-axis`, `palletiser`, `arm-on-track`), at 60 random states (q, q̇, q̈) inside the joints' limits, seed 20261114: the joint torques of inverse dynamics with gravity, the gravity torques alone, the mass matrix, the flange's position and rotation | [MuJoCo](https://mujoco.org) 4.0.1 reading the same URDF files (`mj_inverse`, `mj_fullM`, the bias forces at rest, the flange body's pose), written by [`mujoco_reference.py`](../crates/machine-joint-kit/reference/mujoco_reference.py) into one committed CSV per URDF in [`reference/`](../crates/machine-joint-kit/reference/) | 1e-9 of the largest magnitude of each quantity in a state (the worst seen is 2e-14) | `*_matches_mujoco` in `machine-joint-kit/tests/reference.rs` |

The crate's side is the machine as built from its config (the default configs; `arm5-tool.toml` and `arm7-tool.toml` with joint space for the five- and seven-axis arms): its loss-free rigid torques with the rotors held still (`Machine::rigid_split_torque` with no rotor acceleration), so the Newton-Euler chain of `rigid::Chain`, the mass matrix from the bodies' Jacobians, the motors' ratios and the arm's wiring are all in it; the flange from `rigid::Chain::tip`. MuJoCo is an independent implementation by other people: unlike the SciPy reference it can catch a wrong equation, not only a coding slip. Pinocchio, the first choice, has no wheel for Windows; MuJoCo has, and runs the same on Linux from the pinned [`requirements.txt`](../crates/machine-joint-kit/reference/requirements.txt). Run on 14 November 2026, the script reproduced the committed CSVs exactly. With one link's mass (six-axis, link 3) or one inertia (palletiser, forearm iyy) changed by 1 % the tests fail by 6e-3 and 6e-4.

The arm on its track (`arm-on-track.csv`, added on 15 November 2026 at the end of the script's list, so that the random states of the files before it stay theirs: run again, the script left the five earlier CSVs unchanged to the byte) has the track's prismatic joint first in the chain: its force with the arm's reaction along the rails and every arm joint's torque with the track's acceleration, the whole coupling between them, are compared with MuJoCo's, through the rack's ratio (motor radians per metre). Its worst relative error is that of the other arms.

The palletiser's parallelogram is the config's `arm.joint_coupling` (the URDF's five joints as sums of the four motors' joints, q_urdf = C q). MuJoCo works on the serial URDF chain at C q, C q̇, C q̈; the script takes its numbers to the motors' joints, M = Cᵀ M_urdf C and τ = Cᵀ τ_urdf, as a mimic joint would. That map is written twice (here and in `arm.rs`), so it is checked as the T-gantry's equations are: a slip, not a wrong idea.

What it does not cover: the motors' rotors, the gearboxes' inertia, efficiency, mesh friction and elasticity, friction, the drives, the payload (and its swing), bending links and the compliant base; the parallelogram's bars as bodies of their own (they are in the links they move with, in the URDF as in the reference); the tool-space kinematics (`tool::Pose`, `tool::Scara`) beyond the flange's pose.

**Delta picker D3** ([`crates/machine-delta-d3/reference/`](../crates/machine-delta-d3/reference), tests in [`tests/reference.rs`](../crates/machine-delta-d3/tests/reference.rs)). The rigid, loss-free model of the [model reference](model-reference.md#delta-picker-d3), written again from the document (not from the Rust source) by another route: the arm angles and the platform's position are the coordinates, every body's energy is written out, SymPy differentiates the Lagrangian, and the legs' constraints enter with Lagrange multipliers; the crate instead eliminates the platform through its Jacobian. The inverse kinematics is a root search on each leg's constraint, the forward kinematics a Newton iteration. The rods' spin is worked out from the frame of the pair's plane (n′ · (e × n)), not from the document's closed form.

| What is compared | Against | Tolerance | Test |
| --- | --- | --- | --- |
| Motor torques of the rigid model (`feedforward_torque_moving`) at 60 random states in the working range (seed 2026; speeds and accelerations up to the axes' 3 m/s and 50 m/s²), forearms lumped and as rods | The Lagrangian with multipliers ([`lagrange_reference.py`](../crates/machine-delta-d3/reference/lagrange_reference.py), [`inverse_dynamics.csv`](../crates/machine-delta-d3/reference/inverse_dynamics.csv)) | 1e-8 of the state's largest torque | `delta_inverse_dynamics_matches_the_lagrangian`, delta reference |
| x, y, z of the platform over an open-loop move of 0.5 s from (0.05, 0.02, −0.92) m: the reference's holding torques, plus its push for (5, 3, −4) m/s² as a sin² pulse of 0.15 s and back at 0.25 s (travel 79, 48, 72 mm); forearms as rods (shipped), RK4 at the simulator's step | The same Lagrangian integrated by SciPy's DOP853 (rtol 1e-10), driven by the same torques ([`open_loop_move.csv`](../crates/machine-delta-d3/reference/open_loop_move.csv), [`open_loop_torques.csv`](../crates/machine-delta-d3/reference/open_loop_torques.csv)) | 0.2 % of the travel per axis, every ms | `open_loop_move_matches_scipy`, delta reference |

Both fail when one value of the config changes by 1 % (tried on 14 November 2026 with `mass.forearm_pair_kg` 0.5 → 0.505 kg and `geometry.upper_arm_m` 0.4 → 0.404 m: the torques differ at the first state; the move's z first leaves the allowed 0.14 mm at 148 ms and at 60 ms). The open-loop move is driven by the reference's torques, not the crate's feedforward, so that a model wrong by a factor cannot push itself just as far. The document and the code agreed: no difference was found.

Effects: the Rust tests run the crate's `lossless_config` with `effects.joint_elasticity = false`, i.e. off are the gearboxes' and the arms' elasticity (`joint_elasticity`, `link_elasticity`, `arm_tube_inertia`), every loss (`joint_friction`, `motor_friction`, `gearbox_drag`, `gearbox_lost_motion`, `gearbox_efficiency`, `torque_ripple`, `holding_brake`) and the base (`base_compliance`). Gravity and the payload are on, `forearm_inertia` both ways. The hard stops are on but every state stays inside the stop angles (the script asserts it); drive and controller take no part in given torques. The script asserts the switches it names are still in the config.

**What the mass split costs** ([`multibody_reference.py`](../crates/machine-delta-d3/reference/multibody_reference.py), MuJoCo 4.0.1; printed, not a test). A full multibody model: each upper arm a body on a hinge, the elbow block on it, each of the six forearms a rigid tube of half the pair's mass with its true inertia (m L²/12 + I_axial/2 across, `forearm_rod_spin_inertia_kgm2` along) on a ball joint at its elbow, the platform with the payload a free body, the rotors as armature. Its motor torques come from MuJoCo's inverse dynamics of the open tree mapped onto the arms by virtual work. Over the 638 states (every ms) of the Adept cycle 25/305/25 mm of the default config, the S-curve at 3 m/s, 50 m/s², 2500 m/s³ without the dwells:

| Model | Largest motor-torque difference from the multibody model |
| --- | --- |
| The same pipeline with the lumped masses, against the Lagrangian (a check of the pipeline) | 5.0e-6 N m |
| Forearms lumped, s = 2/3 at the elbow (Codourey/Olsson, `effects.forearm_inertia = false`) | **0.160 N m**, 2.5 % of the cycle's peak 6.44 N m; RMS motor 1 3.290 against 3.217 N m (+2.3 %), motors 2 and 3 1.933 against 1.900 N m (+1.7 %) |
| Forearms as rods (`effects.forearm_inertia = true`, shipped) | 5.5e-5 N m (the tubes' own cross-section, which the slender rods leave out) |

The payload does not change the difference (it sits at the platform in every model).

**Planar delta D2 and linear delta**: their inverse dynamics by the same route (`reference/lagrange_reference.py` in [machine-delta-d2](../crates/machine-delta-d2/reference) and [machine-delta-linear](../crates/machine-delta-linear/reference), tests in each crate's `tests/reference.rs`), lumped forearms and rods as their models have them.

| What is compared | Against | Tolerance | Test |
| --- | --- | --- | --- |
| D2: motor torques of the rigid model at 60 random states in the working range (seed 2026, up to the axes' limits) | The Lagrangian in the arm angles and the plate's (x, z) with multipliers | 1e-8 of the state's largest torque | `d2_inverse_dynamics_matches_the_lagrangian`, D2 reference |
| Linear delta: motor torques of the rigid model (`effects.belt_stretch = false`) at 30 random states with the columns upright and 30 leaning 10° inwards, with the pulley, the belt's mass on it and the rotor | The Lagrangian in the carriages' travels and the plate's position with multipliers | 1e-8 of the state's largest torque | `linear_delta_inverse_dynamics_matches_the_lagrangian`, linear delta reference |

Each fails with a mass or a length changed by 1 % (14 November 2026: `mass.forearm_pair_kg`, `geometry.forearm_m` on the D2; `mass.rod_pair_kg`, `geometry.rod_m` on the linear delta). Effects as on the D3 (each crate's `lossless_config`, rigid; on the linear delta also no guide friction). Not covered: their open-loop motion, the elastic models, the belts and guides of the linear delta beyond the belt's mass on the pulley.

What the D3 references do not cover: the elastic model (gearboxes, bending arms), friction and every other loss, the drives and the controller, the planner (the multibody script writes the cycle's S-curve itself), the hard stops. Both references take the model's assumptions about the real machine (the parallelogram's rods turning with the pair's plane, rigid rods and ball joints, the estimated masses), so like the T-gantry's they catch coding and integration errors and, here, a wrong equation for the model as documented, not a wrong model of a real delta.

### The closed loop of the drive, single axis

| What is compared | Against | Tolerance | Test |
| --- | --- | --- | --- |
| Setpoint, position, speed, motor current, torque, torque command, voltage, speed command and following error (and in stage 4 the DC link voltage, the winding's temperature and the braking resistor's power) of the single axis over one point-to-point move, 0.100625 m to 0.4 m in 0.7 s, every fourth base step (2801 samples), in four stages | A second implementation of drive, controller, profile and mechanics in Python, written from this document's [model reference](model-reference.md) (structure of a step, drive and motor, motion controller, profile generator, single axis and its components), not from the Rust source; the mechanics and the winding between drive samples integrated with SciPy's DOP853 (rtol 1e-10). Traces committed as [`stage1.csv` to `stage4.csv`](../crates/machine-single-axis/reference/), each with the switches and values it ran with | 1e-8 of each signal's peak in stages 1 and 2 (reached: 5e-10, the CSV's digits); 2e-4 in stages 3 and 4 (reached: 7e-5 and 1.1e-4) | `the_rigid_closed_loop_matches_the_python_reference`, `the_drive_with_delays_filters_feedforward_and_limits_matches_the_python_reference`, `the_drive_on_the_elastic_belt_axis_matches_the_python_reference`, `the_drive_with_winding_heating_and_dc_link_matches_the_python_reference` (`machine-single-axis/tests/reference.rs`) |

The stages, each with every other effect off: (1) the rigid loss-free axis, position, velocity and current loop with the back-EMF; (2) plus the drive delays, velocity and torque filters, speed and torque feedforward with fine interpolation, torque saturation, the torque-speed curve, the drive's current limit and the voltage limit, with the drive's values lowered (0.5 A, 0.3 N m, a 90 V bus) so that each limit acts within the move; (3) plus the elastic belt (stretch, damping, slack, mass), the gearbox (backlash, stiffness, damping, efficiency), guide friction (LuGre), motor friction, torque ripple, cogging and tooth meshing; (4) plus the winding's heating (started 80 K warm, with a 0.5 s time constant so that it moves) and the DC link with its braking chopper (a tenth of its capacitance, so that braking charges it to the threshold and the chopper burns).

Why the tolerances: the drive is discrete (cycle times, zero-order hold of the voltage), so both implementations take the same decisions at the same samples, and what is left is the integrators' error between them. On the smooth rigid axis that is below the twelve digits of the CSV. In stage 3 the gearbox's backlash puts a kink into the equations where the flanks meet; the simulator's fixed RK4 step (15.6 µs) goes through it less exactly than an adaptive solver, and the difference peaks at 7e-5 of the peak torque where the torque reverses at the end of the braking ramp. Away from it, and with the backlash off, the two agree to 1e-8.

The check bites: the velocity gain 1 % higher fails all four stages (stage 3: 1.2e-3 of the peak torque command, 1.3e-2 of the peak following error), the velocity filter's time constant 1 % longer fails stages 2 to 4.

Differences between the model reference and the code that it found (14 November 2026), each settled:

- The current loop's expected back-EMF is saturated, k_e ω_f (1 − s min(1, (i / i_max)²)) at the winding's current; the model reference said only "at its measured speed". The code is right (the winding's back-EMF falls with the torque constant); the reference is corrected.
- A winding started or enabled warm began with the current loop's integrator, output and delay at the cold resistance's R i: a voltage step of (R(ϑ) − R) i. The code was wrong and is fixed (`Drive::hold_current`); stage 4 fails without the fix (6e-4 of the peak torque).
- The drive started with the torque that holds the pose its motor encoder reads, not the pose the carriage stands at; with the backlash the two are 8.7 µm apart and the torques 5.3e-4 N m and 0.8e-4 N m. Resolved for a machine built at rest (15 November 2026, `Controller::new` holds at the tool's pose); the Python reference now starts with the torque that holds the carriage where it stands, and its stages 3 and 4 were made again (with the old reference they differed by up to 0.0054 of the peak, following error). Open: `Command::Enable` has no machine state and still holds at the setpoint, the pose the encoders read.
- At the start the DC link is at the bus voltage; the holding current's ½ L i² is taken from it only at an Enable. Not stated before; the model reference now says so (whether the start's ½ L i² belongs in the energy budget is left to it).
- With no torque at rest the gearbox's twist is 0 (`static_twist`), so at a pose where the tooth meshing is exactly zero the side its flanks rest on is left to rounding. The move therefore starts an eighth of a tooth pitch past a whole number of them.

What it does not cover: field weakening, iron loss, the drive's overload, the motor's thermal nodes, magnet temperature, the notch filter, encoder resolution and noise (encoder resolution matches too, until at standstill a count the two integrations put 1e-9 apart rounds the other way), precontrol balancing, the friction feedforward, the fieldbus delay and jitter, the load encoder, a regenerative supply, the shared DC link, gravity and payload, the gearbox's lost motion, drag, mesh friction and stiffness curve, motor stiction, belt tooth stiffness and material damping, the hard stops and the brake, the stepper and the induction motor (`motor.kind`; the reference runs the servo, which they leave bit for bit as it was). Profiles other than the S-curve, and an S-curve that does not reach its cruise speed. And, as for the T-gantry, it is one reading of one document written twice: it catches places where code and document disagree and integration errors, not a wrong equation in both.

## Hand calculations: mechanics

| What is compared | Against | Tolerance | Test |
| --- | --- | --- | --- |
| Acceleration under constant torque, rigid model | Force over effective mass | 1e-9 | `constant_torque_gives_force_over_effective_mass`, axis and gantry |
| Holding torque of an inclined axis | m g sin · r / i | exact rigid; 0.1 % elastic | `inclined_axis_needs_holding_torque`, axis |
| Holding torques on Z | ± W_z r / (2i), W_z = (m_z + ρ (4s + 2s_z)) g = 7.16 kg · g with the belt's vertical runs | exact rigid; 0.1 % elastic | `static_hold_on_z_needs_opposite_torques`, gantry |
| A slide's presliding friction (`effects.slide_presliding`): Dahl's force from its anchor, held at rest, turned back, the Stribeck level; off, the smoothed step to the bit | g (1 − e^−1) = 94.8181 N; −59.9364 N; 118.394 N | 1e-4 N | `a_presliding_friction_holds_at_rest_and_turns_over_its_travel`, mech |
| The screw axis inclined by 5°, drive off: held by its presliding friction, crept with the smoothed step | −g / σ ln(1 − F / g) = 2.40 µm taken up, < 1 µm in the second half second (Dahl ratchets 10 µm more while the screw rings); 0.002 · atanh(21.37 / 89.47) = 0.487 mm/s | 1 µm; 0.03 mm | `a_presliding_slide_holds_its_load_without_creeping`, joint kit |
| Belt tension from stretch; damping on the stretch rate | EA · strain; c · rate | 1e-6 N; 1e-9 N | `belt_tension_is_ea_strain_and_pretension_at_rest`, `belt_damping_acts_on_the_stretch_rate`, mech |
| Belt forces | The gradient of the belt's energy | 1e-3 N | `belt_forces_are_the_gradient_of_its_energy`, mech |
| Belt sections at rest and in a static hold (belt view) | Without gravity every section at the pretension 300 N, section 1 stretched by L · P / (EA + P) (0.4497 mm on 0.75 m, gantry; 0.1499 mm on 0.25 m, axis). Held: single axis at 30°, T₂ − T₁ = 6 kg · g · sin 30° = 29.42 N; T-gantry, 2 T₂ − T₁ − T₃ = (7 + 0.16) kg · g = 70.22 N (the Z axis and the belt's vertical runs) and T₁ = T₃. The 3D view's strips carry (T − P) / P of exactly these tensions | 1e-6 N; 1e-12 m; exact | `belt_sections_have_the_pretension_at_rest_and_carry_the_weight_in_a_static_hold`, views |
| Tooth stiffness in series | 1 / (1/k + 1/k_tooth) | 0.1 % | `teeth_in_mesh_are_a_spring_in_series_with_the_section`, mech |
| Material damping | Four times as hard for a section a quarter as long | 1e-9 N | `material_damping_scales_with_the_section_stiffness`, mech |
| Torque breakdown, pure inertia: rigid loss-free T-gantry without gravity, 0.1 N m on both motors | J α per motor with J = (0.02/5)² (11 + 2 · 1e-4/0.02²)/2 + 0.3e-4 = 1.22e-4 kg m² and α = 819.672 rad/s²: 0.1000 N m, all inertia; gravity, friction and rest 0. With 0.15 and 0.05 N m: the torques | 1e-12 N m | `a_pure_inertia_move_is_all_inertia`, gantry |
| Torque breakdown, vertical hold: T-gantry holding Z, single axis at 90° (elastic, controlled, ripple and meshing off) | Gravity part ± 7.16 · g · 0.02 / 10 = 0.140431 N m (the Z axis and the belt's vertical runs) and 6 · g · 0.02 / 5 = 0.235360 N m; motor torque equal to it, the other parts below a fraction of it; copper loss R i² = 4 · (0.140431 / 0.45)² = 0.38955 W | 1e-12 N m; 0.2 % (mean over 50 ms; one step dithers 0.5 %); 0.2 % and 0.5 %; 0.2 % | `holding_z_is_all_gravity`, gantry; `holding_the_vertical_axis_is_all_gravity`, axis |
| Torque breakdown, constant speed: rigid T-gantry with guide friction alone, X at 0.8 m/s | Friction parts of both motors together r / i · (15 + 20 v) = 0.124 N m; the motor torques add up to it; the guide's split 15 N Coulomb, 20 v viscous, 0 stiction | 1e-9 N m; 2 %; 1e-6 N | `a_constant_speed_move_is_coulomb_plus_viscous_friction`, gantry |
| Friction view's curve | [λ F_c, b v, λ (F_s − F_c) e^(−(v/v_s)²)] · sign v by hand: 15, 10, 0 N at 0.5 m/s; 15, 0.2, 5/e = 1.83940 N at 0.01 m/s; −15, −0.1, −3.89400 N at −0.005 m/s; 30, 0.2, 3.67879 N with λ = 2; T-gantry Z guide −20, −3, 0 and 10, 0.075, 3.11520 N; single axis −15, −0.2, −1.83940 and 22.5, 20, 0 N | 1e-5 N | `lugre_steady_curve_and_live_split_follow_the_formula`, mech; `the_friction_curves_are_the_formula`, gantry; `the_friction_curve_is_the_formula`, axis |
| Live friction split | In steady sliding the curve's parts; held at rest by 5 N of bristle force 3.75 N Coulomb and 1.25 N stiction (15 : 5); the parts add up to the friction force | 1e-5 N; 1e-9 N | the same test, mech |
| Loss powers | Belt damping 100 N s/m at 0.3 m/s: 9 W; gearbox 2 N m at 50 rad/s and 90 %: 10 W, driven back 11.11 W; gear damping 5 N m s/rad at 1 rad/s: 5 W; end stop 200 N s/m at 0.1 m/s: 2 W | 1e-9 W | `loss_powers_are_the_hand_calculated_ones`, mech |
| Belt natural frequency (carriage against pulley) | Two masses on a spring k from EA / L | 5 % | `belt_natural_frequency_matches_hand_calculation`, axis |
| The same with tooth stiffness | 1/k = L/EA + 1/k_tooth | 5 % | `belt_natural_frequency_with_tooth_stiffness_matches_hand_calculation`, axis |
| Elastic model with a very stiff belt | The rigid model | 1 % of the travel | `stiff_belt_converges_to_the_rigid_reference`, axis and gantry |
| Slack time of a belt at 20 N pretension | Between 0.5 s and 1 s of a 1 s move; none at 300 N | Range | `a_slack_belt_section_is_counted_and_a_tight_one_is_not`, axis |
| Breakaway force | The stiction level, 20 N | 2 % | `breakaway_force_equals_the_stiction_level`, axis |
| Steady sliding force | Coulomb + viscous · v | 0.1 % | `steady_sliding_force_is_coulomb_plus_viscous`, axis; `lugre_steady_sliding_is_coulomb_plus_viscous`, mech |
| Friction factor with guide load | (preload + load) / (preload + nominal) | 1e-12 | `friction_grows_with_payload_when_load_dependent`, axis; `guide_friction_grows_with_acceleration_and_overhang`, gantry |
| Lost motion on reversal | The configured backlash, 8.7e-4 rad | 1 % | `lost_motion_on_reversal_equals_the_backlash`, axis |
| Gearbox: dead zone, flanks that never pull, efficiency both ways | τη driving, τ/η driven | 1e-9 N m | `gearbox_transmits_nothing_inside_the_backlash`, `gearbox_flanks_in_contact_never_pull`, `gearbox_efficiency_loses_torque_in_the_direction_of_power_flow`, mech |
| Friction feedforward from the machine's model (`drive.friction_feedforward_model = "machine"`), single axis in its cruise at 1 m/s | Guide 15 N · 1.082012 (load scale with the 2 kg payload) + 20 N s/m · 1 m/s through 0.02 m / (5 · 0.95), shaft 0.02 + 1e-4 · 250: 0.197548 N m; the velocity integrator's mean carries under 1 % of it (without the feedforward, all of it) | 1e-4 relative; 1 %; 2 % | `the_machine_friction_feedforward_is_the_steady_friction_and_the_integrator_carries_nothing`, axis |
| The same on every other machine crate | Screw axis 0.529807 N m at 0.8 m/s; parallel SCARA tool turn 0.878065 N m at 30 rad/s; flying saw 1.440917 N m at 1.2 m/s (its shipped characteristic 1.440936); T-gantry 0.114638 / 0.107704 N m at 0.8 m/s in X (the gearboxes' loss on the Z axis's weight in it); linear delta −0.267867 N m; D3 0.233030, D2 0.223461, H4 0.225393 N m going down at 0.5 m/s; D4 arms 0.138255 (the gearbox loss on the weight in it, 0.114165 without), tool 0.025778 N m; hexapod 0.254378 N m rising at 0.1 m/s; planar cable robot −0.0960803 / 0.0946509 N m hauling in and paying out; in the cruises the integrators carry under 1 to 3 % of it | 1e-4 relative (hand values), 1e-5 N m (hexapod), 1e-6 N m (cable); 1 to 3 % | `the_machine_friction_feedforward_is_the_steady_friction_and_the_integrator_carries_nothing` in each crate; `the_flying_saw_s_machine_friction_feedforward_is_its_characteristic_and_the_integrator_carries_nothing`; `the_steady_friction_of_a_rising_platform_is_the_hand_calculated_one`; `the_planar_winches_steady_friction_at_the_centre_is_the_hand_calculated_one`, `the_planar_machine_friction_feedforward_takes_the_friction_off_the_integrators` |
| Friction feedforward against the model's own steady state: the torques under which the rigid model's derivatives give no acceleration at a steady speed (Newton, the Jacobian by differences) against the loss-free feedforward plus `steady_friction` | T-gantry at (0.5, 0.25) m, Z up and down at 0.5 m/s, η 0.95, no guide friction: ±0.147822 and ±0.133410 N m; D4 at z = −0.85 m down and up at 0.5 m/s with the tool turning, and sideways: arms −0.470195 and −0.753110 N m (the arms' gearbox loss on the weight in `steady_friction`, as on the planar linkage) | 1e-9 N m (T-gantry); 5e-4 of the torque (D4: the loss on the velocity terms, which the feedforward leaves loss-free) | `the_steady_friction_has_the_gearbox_loss_on_the_weight`, gantry; `the_friction_feedforward_takes_the_loss_on_the_weight`, D4 |
| Gripper times | Vacuum: V/Q ln(p_v / (p_v − p_hold)) = 0.12 ln 4 = 0.166355 s to grip, V/Q_vent ln(p_v / p_release) = 0.03 ln 8 = 0.062383 s to let go, hold A p_hold = 42.4116 N; jaws as configured | 1e-12 s | `the_vacuum_gripper_takes_the_hand_calculated_times_and_holds_area_times_pressure`, gripper |
| Vacuum cup's hold against a part accelerated sideways | Slips beyond a = μ (A Δp − m g) / m = 5.69958 m/s² (2 kg, μ 0.5): there the part asks exactly A Δp | 1e-9 N | `the_part_asks_the_hold_exactly_at_the_hand_calculated_slip_acceleration`, gripper |
| Gripper time on the single axis's shuttle cycle | Jaws opening in 0.25 s and closing in 0.3 s against dwells of 0.1 s: the cycle 0.35 s longer, the payload let go 0.25 s after arrival; jaws of 0.05 s and the switch off change nothing | 1e-9 s; 1.5 ms; exact | `the_gripper_time_lengthens_the_dwells_and_moves_the_payload_switch_by_hand`, axis |
| `gripper-hold` on the rigid single axis at 10 m/s² | m g + m a / μ = 59.6133 N (seen 61.86 N: the servo's overshoot); passes a 70 N cup, fails a 55 N one; the optimiser stops where a 63 N cup still holds, short of where it goes with the hold not judged | 5 % | `the_gripper_hold_judges_the_held_part_against_the_cup_and_the_optimiser_keeps_to_it`, axis |
| Gearbox mesh friction (`effects.gearbox_mesh_friction`): each way, rated point and part load, holding and breaking away, ring-down, energy | Ratio 10, K1 1e4 N m/rad, η 0.8: 100 N m passed gives 80 N m driving and takes 120 N m driven back (1 / (2 − η) = 0.833); with T0 = 0.1 N m and η 0.9 at 50 N m the teeth's 0.916497, 0.9 at 50 N m and 0.774527 at 5 N m; a 100 N m weight held stands until 125 N m lifting and 83.33 N m lowering (within 3 N m, the bristle's give 2.5e-5 rad); a ring around zero load keeps (1 − μ) / (1 + μ) = 0.666667 a cycle with a stiff bristle (5 %), 7/9 = 0.777778 with the shipped one (1e-4), its loss power integrating to the energy gone (1 %); every machine with a gearbox closes its cycle's budget with it on (0.1 %), the T-gantry too (4e-6; off in its config as on the other belt machines); a gear at the bristle's cap on a light drum (0.32 kg at 3.2e8 N/m) stays bounded and closes its energy to 1e-6 of its loss | as stated | `the_mesh_friction_passes_the_teeth_efficiency_each_way`, `mesh_friction_and_drag_give_the_catalogue_efficiency_and_less_at_part_load`, `a_held_gear_stands_inside_its_friction_band_and_breaks_away_at_its_ends`, `a_ring_around_zero_load_decays_by_the_hand_calculated_damping`, `the_shipped_bristle_rings_down_by_its_hand_calculated_ratio`, `a_gear_at_the_bristle_cap_of_a_light_drum_stays_stable_and_closes_its_energy` (sim-components); `the_energy_budget_closes_with_the_mesh_friction_on_every_gearbox` (sim-machines) |
| An arm's joints' modes with the motors held (`Machine::modes`, `resonance_hz`) | Two joints on 100 and 50 N m/rad, M = [[2, 0.5], [0.5, 1]] kg m²: det(K − λM) = 1.75 λ² − 200 λ + 5000 = 0, 0.967314 and 1.399712 Hz; uncoupled √(k / J) / 2π = 1.125395 Hz and the damping c / (2 √(k J)) = 0.070711; the palletiser's turret at full reach 5.95 Hz held, its measured ring 5.30 Hz, and the velocity loop's dynamic stiffness Kp (1 + 1/(jω Tn)) (Kv + jω) − ω² J in series with the gear 5.36 Hz | 1e-6 Hz; the loop's estimate within 2 % of the ring (1.0 %) | `held_joint_modes_are_the_hand_solved_eigenvalues` (sim-components), `the_palletisers_turret_rings_at_its_held_mode_softened_by_the_velocity_loop` (machine-joint-kit) |
| Leaning Z axis at two heights: restoring acceleration and tool-tip offset | Two masses on k = 1 / (1/k_lat + h²/k_tilt), the carriage with the belt's runs it moves, 4 + 0.8 (0.65 − z) kg (6.96 m/s² at z = 0.05 m for u = 0.1 mm) | 1e-6 relative; 1e-12 m | `leaning_z_axis_matches_the_two_mass_hand_calculation`, gantry |
| Resonance the machine reports at two heights (the lowest mode that moves the tool; frame off) | 24.344 Hz and 34.590 Hz at x = 0.9 m, z = 0.05 and 0.15 m; by hand two masses in a row (carriage with the belt's runs on the belt, Z axis on its guide) 24.62 and 35.52 Hz, which the X-Z coupling at x = 0.9 m lowers | 0.005 Hz; 1 Hz | `a_shaper_that_follows_the_height_leaves_about_a_tenth_at_both_heights`, gantry |
| Modes of the carriage, the frame and the Z axis on the belt | Generalised eigenvalues by hand at x = 0.5 m, z = 0.25 m without the lean, in (x on the frame, frame): M = [[11.32, 11.16], [11.16, 41.42]] kg (11 kg, the belt's runs 0.32 kg on x, the whole belt's 0.42 kg swaying and its 1.6 m of vertical runs coupling), K = diag(k_b, 7.4e5), k_b = 2 × 5e5 / l with l = 1.05 / (1 + 282.838 / 5e5) the unstretched belt: 20.60 and 55.64 Hz; without the frame 46.18 Hz | 1e-3 Hz | `the_modes_are_the_generalised_eigenvalues_of_the_carriage_the_frame_and_the_z_axis`, gantry |
| Modes against the model's own linearised equations (pulleys held, loss-free, backlash off) at (0.9, 0.05), (0.5, 0.25), (0.5, 0.05), (0.1, 0.45) | K = −M J from the derivatives' accelerations per unit of x, z, q, u: each mode's K v = ω² M v. 18.748 / 30.684 / 76.046 / 116.303 Hz; 20.531 / 53.225 / 83.047 / 201.018; 18.689 / 30.623 / 84.595 / 92.683; 19.888 / 39.813 / 82.117 / 103.463. Without the belt's mass 18.809 / 30.786 / 78.517 / 119.468 Hz and so on (the audit's 18.81 / 30.77 / 78.51 / 119.46 had no pendulum stiffness of the Z axis's weight). Both from an independent reference, `crates/machine-t-gantry/reference/modes_reference.py` | 1e-6 of ω² M v; 1e-5 Hz | `the_modes_are_those_of_the_linearised_derivatives_with_the_pulleys_held`, gantry |
| T-gantry loss-free, rigid, rigid with idlers and chain, elastic (frame, lean), elastic with idlers, chain, tooth stiffness, Z bending and beam sag, two moving states each | The motors' power Σ τ ω is the rate of the energy along the derivatives (the belt's kinetic energy summed run by run); the rigid feedforward at speed gives back (3, −2) m/s² | 1e-7 of the power (measured at most 8.8e-9); 1e-9 m/s² | `the_motors_power_is_the_rate_of_the_energy_in_every_loss_free_configuration`, `the_feedforward_at_speed_gives_the_commanded_acceleration`, gantry |
| The second shaper following the pose | The first shaper takes the lowest mode that moves the tool where the plan stands, the second the next one: with the default frame at x = 0.9 m 18.7481 and 30.6839 Hz at z = 0.05 m, 20.1208 and 41.5342 Hz at 0.15 m; not following, 25 and 100 Hz as configured | 1e-3 Hz | `the_second_shaper_follows_the_second_mode_at_each_pose`, gantry |
| Single axis: its carriage mode at two places, and the wrap | sqrt(k / m) / (2π), k = EA / l of the unstretched belt: 65.8222 Hz at x = 0.5 m, 96.6813 Hz at 0.1 m; damping 0.040299 and 0.027436; π at the motor pulley | 1e-3 Hz; 1e-5; 1e-12 | `the_carriage_mode_is_the_hand_calculated_one_at_two_places`, axis |
| Modes over the stroke (`sim-cli modes`), single axis at x = 0, 0.5, 1 m | 122.2412, 65.8222, 57.3870 Hz; damping 0.021700, 0.040299, 0.046223 (k = EA / l of the unstretched belt; 0.03 % above EA / L of the stretched length) | 0.1 % | `modes_write_the_resonances_over_the_stroke`, cli |
| Idlers: added mass and drag in X and Z | 12 and 4 idler masses; 6 and 4 friction forces | 1e-9 rigid; 1e-6 elastic | `idlers_add_the_hand_calculated_mass_and_drag`, gantry |
| Cable chain at two positions | Mass 0.175 kg and 0.375 kg; drag and pick-up force | 1e-9 | `cable_chain_adds_the_hand_calculated_force_at_two_positions`, gantry |
| Process force: change of the holding torques | −0.06 N m and +0.02 N m | 2 % of 0.06 N m | `process_force_changes_the_holding_torque_by_the_hand_calculated_amount`, gantry |
| Depth in a hard stop | Force / stiffness: 25 µm and 50 µm | 1 % | `a_carriage_pushed_into_the_hard_stop_rests_at_the_hand_calculated_depth`, axis and gantry |
| Holding brakes: a brake of 0.15 N m holds Z; one of 0.12 N m lets it fall; in the elastic and in the rigid model | Holds within 0.1 mm; falls 96 mm in 0.5 s | 3 % | `holding_brakes_hold_the_z_axis_when_they_are_strong_enough`, gantry |
| Single axis, cable chain: extra force to accelerate at 8 m/s² at x = 0.1 m and 0.9 m; coasting at 0.5 m/s; holding torque on a 30° incline | 1.4 N and 3.0 N; (3 + 0.25 · 0.5² / 2) N over the moved mass, rigid and elastic; 0.175 kg and 0.375 kg · g / 2 more to hold | 1e-9; 1e-6 m/s²; 1e-12 and 2e-6 N m | `cable_chain_adds_the_hand_calculated_force_at_two_positions`, axis |
| Single axis, process force of 10 N during a dwell | Holding torque 0.04 N m lower | 2 % | `process_force_changes_the_holding_torque_by_the_hand_calculated_amount`, axis |
| Single axis set upright, holding brake: 0.26 N m holds 6 kg; 0.2 N m lets it fall; in the elastic and in the rigid model | Holds within 0.1 mm; falls 136 mm in 0.5 s (136.4 mm simulated) | 3 % | `the_holding_brake_holds_the_upright_axis_when_it_is_strong_enough`, axis |
| Torque at the gearbox output with an efficiency of 0.9, lifting and lowering at a steady speed | The load's torque both ways: 0.7845 N m (axis, rigid), 1.1768 N m (axis, elastic, lowering), ±0.6865 N m (gantry, rigid, and elastic at rest); the motor torque times the ratio reads 1/0.9 or 0.9 of it | 1e-9 N m rigid; 1 % and 0.1 % elastic | `the_gearbox_verdicts_judge_the_torque_at_the_gearbox_output`, axis; `gearbox_torque_is_the_load_at_the_pulleys_whichever_way_it_is_driven`, gantry |
| Friction with stiff bristles on a light contact at speed (1e6 N/m, 12 N, 2.5 m/s) | Stable at the simulator's step; sliding takes 12 N + 20 N s/m · 2.5 m/s = 62 N | 1e-6 N | `lugre_with_stiff_bristles_at_speed_stays_stable_and_slides_at_its_level`, mech |
| Radial load on the pulley shaft | Sum of both section tensions: 570.58 N (vertical axis), 593.46 N (gantry) | 0.02 N | `radial_load_on_the_pulley_shaft_is_the_sum_of_both_section_tensions`, axis; `guide_loads_and_shaft_loads_match_the_hand_calculation`, gantry |
| Guide loads under acceleration | X guide 203.5 N, Z guide 118.6 N | 0.01 N | `guide_loads_and_shaft_loads_match_the_hand_calculation`, gantry |
| Temperature feedback laws at fixed temperatures (B 4000 K, 20 °C reference) | Viscosity factor 0.281749 at +30 K, 4.73776 at −10 °C; gearbox of 95 % with half its loss viscous: 0.967956 and 0.856556; belt at 40 °C: 0.994 of EA | 1e-6 | `the_temperature_laws_follow_their_formulas`, mech |
| Temperature feedback in the machines at a rise of 30 K | Guide viscous friction 0.281749 of the configured, Coulomb unchanged, the hand-set `[temperature]` factor not used; gearbox loss in a cruise 0.64088 of the cold one | 1e-5 N; 1 % | `with_thermal_feedback_friction_and_gearbox_loss_follow_the_temperature`, belt_and_heat |
| A guide at 1 m/s with the feedback settles where its loss and heat flow balance (R 2 K/W) | rise = 2 (λ F_c + b ν(rise)/ν_ref v) v: 40.18 K (72.46 K without the feedback) | 1 % | `a_guide_settles_where_its_loss_and_heat_flow_balance`, belt_and_heat |
| Warm start, single axis winding rated 0.5 A (copper 0.39 %/K) | Settled at q / (1 − 0.0039 q), q = 100 / 1.39 · (0.4463 / 0.5)² = 57.32 K: 73.82 K; copper loss of the measured pass 4 Ω · (1 + 0.0039 · 73.82) · 0.4463² = 1.0261 W (cold: 0.7967 W) | 0.2 % against R(rise) I²; 0.5 % against 1.0261 W | `a_winding_started_warm_loses_the_hand_calculated_copper_power_at_its_settled_rise`, belt_and_heat |
| Warm start, single axis guide with the feedback on its shipped cycle | Settles at 10.03 K (12.51 K from cold); viscous friction at 0.5 m/s 10 N · ν(10.03 K)/ν_ref = 6.3673 N | 0.05 K; 1 % | `a_guide_started_warm_has_the_hand_calculated_friction_at_its_settled_rise`, belt_and_heat |
| Warm start on the default T-gantry, windings alone and with the feedback | Converges (within 0.1 K) in at most 4 warm passes (1 and 2 measured) | – | `the_warm_start_converges_on_the_default_t_gantry_within_its_pass_limit`, belt_and_heat |
| Tooth climb-out and jump logic | Climb at a pull above 6 × the slack side; ridge 600 N at a quarter pitch, 0.954930 J at its top; one pitch on and counted, or back uncounted; momentum shared at landing, 0.793651 m/s | 1e-6 | `a_tooth_jump_starts_at_the_climb_limit_and_ends_one_pitch_on`, mech |
| Tooth jump threshold, single axis upright holding at x = 0.1 m | W = 6 P / (1 + 6 L₂ / (L₁ + L₂)) = 280.24 N: no jump at 98 %, a jump at 102 %, the first one exactly one pitch (−5 mm) | 2 % either side; exact | `the_belt_teeth_jump_at_the_hand_calculated_pull`, belt_and_heat |
| The wrap angle, and the tooth load it gives | π less the lean of both spans: π for level spans, π/2 + asin(0.04) = 1.610807 rad for one straight up; climb = k φ = 2.4 π = 7.539822; 2 rad on r = 20 mm, p = 5 mm: 8 teeth, tooth load 0.205610 | 1e-6; 1e-5 relative | `the_wrap_angle_is_half_a_turn_less_the_lean_of_both_spans`, mech; `the_tooth_load_counts_the_teeth_in_mesh_from_the_wrap`, analysis |
| T-gantry: the wrap at both motor pulleys at x = 0.1 m and 0.9 m, and the climb-out that follows | The spans to the carriage idlers leave level: π on both; at T₂ = 8 T₁ the teeth jump with the configured 10 teeth (limit 6) and hold with the wrap (limit 7.54) | 1e-12 | `the_belt_wraps_both_motor_pulleys_by_half_a_turn_and_the_teeth_climb_out_at_k_pi`, gantry |
| Tooth jump threshold with the wrap, single axis as above | c = 2.4 π: W = 289.44 N; no jump at 98 % (where the configured teeth let go at 280.24 N), a jump at 102 % | 2 % either side; exact | `with_the_wrap_the_belt_teeth_jump_at_the_pull_of_half_a_turn`, belt_and_heat |
| Jumping teeth in a cycle (single axis, 90 N pretension) | `belt-tooth-jump` fails; the jumps' energy closes the budget | 0.5 % of the supply (measured 0.33 %, falling with the step) | `jumping_teeth_fail_the_check_and_close_the_energy_budget`, belt_and_heat |
| Belt span as a string | f1 = sqrt(T/μ) / (2 L): 109.545, 21.066, 26.082 Hz at 300 N, 0.1 kg/m; T-gantry span 1 at its start pose; a cruise at 200 Hz of tooth mesh on a 200 Hz span fails `belt-span-resonance` | 1e-3 Hz | `a_span_vibrates_as_a_string_and_is_judged_against_the_excitations`, mech; `a_span_vibrates_at_its_string_frequency_and_a_cruise_on_resonance_fails`, belt_and_heat |
| Two-way base (`effects.base_reaction`): a slider of 4 kg on a spring of 2e5 N/m to a base of 200 kg on 8e5 N/m, the drive off, started in either mode shape | The two-mass system's closed-form modes, 10.057 Hz and 35.981 Hz, the slider at 1.0868 and −45.09 times the base | 1e-6 of the amplitude over 0.2 s | `a_slider_on_a_spring_on_a_coupled_base_swings_in_the_two_mass_modes`, base |
| Two-way base free along x, z and in tilt about y (springs at zero), an arm swung on it by a torque pulse, no outside force | Momentum M_B d' + p and I_B θ' + L stay zero | 1e-9 of the arm's own | `a_free_coupled_base_and_the_machine_on_it_keep_their_momentum`, base |
| Two-way base under an arm in gravity (x, z and a soft tilt), and under the H4 (shipped base plus tilts about x and y): losses off, then the shipped damping | The energy is conserved (the suite's `energy_conservation`; one-way the H4 drifts by 4.5 %), the loss budget closes with `power.base` and the base in `energy.*` | 0.1 % of what the pulse put in; `LOSS_BUDGET_TOLERANCE` | `a_coupled_base_keeps_the_energy_and_closes_the_loss_budget`, base; `on_a_coupled_base_the_four_arm_delta_keeps_its_energy_and_budget`, H4 |
| T-gantry: `frame_compliance` off and a two-way base along X of the frame's 30 kg plus carriage, Z axis, payload and belt (41.42 kg), the frame's 7.4e5 N/m and damping ratio 0.01716 | The frame mode (`effects.frame_compliance`): the same equations, the belt's runs in the base's momentum as in the frame's mass matrix; a move from (0.1, 0.35) to (0.9, 0.05) | 1e-9 of the sway's 0.149 mm (measured 5.1e-15 m) | `a_coupled_base_along_x_moves_as_the_frame_mode`, T-gantry |

## Hand calculations: drive

All in drive unless noted.

| What is compared | Against | Tolerance | Test |
| --- | --- | --- | --- |
| Current step response | 63.2 % after the tuned time constant of 0.25 ms; no overshoot | 0.05 A of 1 A; peak below 1.01 | `current_follows_its_setpoint_with_the_tuned_time_constant` |
| Drive current limit (effects.drive_current_limit): 5 A on a 0.45 N m/A motor of 3.8 N m; 10.9 A | Torque range ±2.25 N m, integrator held, current setpoint 5 A; ±3.8 N m (the motor's) | exact | `the_drive_current_limit_holds_the_torque_to_the_lower_of_motor_and_drive` |
| Field weakening (effects.field_weakening): 0.45 N m/A, 190 V, 9.5 A, 4 pole pairs, L_d = L_q = 10 mH, R → 0, at 500 rad/s | The circles cross at i_d = −50625 / 9000 = −5.625 A, i_q = sqrt(9.5² − 5.625²) = 7.6557 A: 3.4451 N m (the fading curve: 0); the current loop settles there with sqrt(u_q² + u_d²) = 190 V, power = copper loss + shaft power | 1e-3 N m, 1e-3 A, 0.1 V | `field_weakening_gives_the_hand_calculated_torque_above_the_voltage_limit` |
| d-q model on the 1FK7042-2AF7 at 600 V (R, L, k_E, pole pairs from its Siemens data sheet) | The rated 2.6 N m at 2.0 A and 3000 r/min inside the curve and without d current; torque left at n_max,inv 670.2 rad/s, where the fading curve ends at 538.9 rad/s; best d current at 942.5 rad/s → −ψ / L_d = −3.247 A. Siemens' torque-speed curves are graphs, not in the catalogue files: no curve points are compared | 6 % (rated torque); 1 % (d current) | `the_d_q_model_holds_the_published_points_of_a_1fk7` |
| Drive overload (effects.drive_overload): 3 A rated, 2.6 A base load, 9 A for 0.25 s, τ 10 s; a step to 6 A from cold | Trip 9 − 8 e^(−0.025) = 1.19752; clamps to 2.6 A after −10 s · ln(1 − 1.19752 / 4) = 3.558 s; the image back at 1 after −10 s · ln(0.2489 / 0.4464) = 5.842 s more | 2 ms; 3 ms (a velocity cycle's overshoot of the trip) | `a_step_at_twice_the_rated_current_clamps_after_the_hand_calculated_time` |
| Image peak of a repeated cycle (τ 10 s): 1.5 I_rated held; 2 I_rated for 0.5 s of 1 s | 2.25; x₀ = 4 (1 − e) e / (1 − e²) = 1.95001, peak x₀ e + 4 (1 − e) = 2.04999 (e = e^(−0.05)) | 1e-9; 1e-4 | `the_overload_image_of_a_repeated_cycle_peaks_at_the_hand_calculated_level` |
| Three thermal nodes (effects.motor_thermal_nodes): s = 0.25, τ_w = 60 s, τ_th = 1800 s, a loss the one node settles at 100 K | Eigenvalues −7.36204e-4 and −1.934927e-2 1/s: winding 100 − 81.5696 e^(λ₁t) − 18.4304 e^(λ₂t), 16.18 K at 60 s, 78.32 K at 1800 s; settled 100 K (P · (R_w + R_h)) and housing 75 K | 1e-3 K | `three_thermal_nodes_settle_at_the_sum_of_the_resistances_and_step_as_by_hand` |
| Single axis held at a 0.5 A drive (rigid, payload off, 15 N Coulomb friction) | a = (0.45 · 0.5 · 5 / 0.02 − 15) / 6.125 = 6.7347 m/s²; with the motor's 0.18 N m below it, (45 − 15) / 6.125 = 4.8980 m/s² (6.733 and 4.894 simulated); the shipped cycle at 70 % of its peak current fails `drive-peak-current` and lags further | 1 % | `a_move_held_at_the_drive_current_limit_accelerates_at_the_hand_calculated_rate`, axis |
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
| Regenerative supply: a motor feeding back 41 W for 0.1 s at 190 V; the line module limited to 20 W; a motor drawing 49 W | 4.018 J into the mains and 0.082 J lost (efficiency 0.98), the link held at 190 V, nothing burned; limited: 1.96 J into the mains and the other 21 W charge 470 µF to sqrt(2 · 10.5835 J / 470 µF) = 212.217 V; drawing: 4.9 J from the line | 1e-9 J; 1e-3 V | `a_regenerative_supply_feeds_back_into_the_mains_up_to_its_power`, drive |
| Regenerative supply on every config of both machines (as the energy budget below, but without DC link) | Fed back into the mains = 0.98 × what the motors regenerated, line module loss the rest, drawn = what the motors drew (per motor exactly; the budget, one step shorter, to one step at the line module's peak power); no resistor; a shared link feeds back less; the budget closes | 1e-9 relative; one step; 0.1 % and 1e-6 | `the_regenerative_supply_feeds_back_what_the_motors_regenerate`, effects |
| Line module checks with the regenerative supply, every machine | `line-module-continuous` = line module energy / cycle time against 16 kW, `line-module-peak` = its peak against 35 kW; neither without the effect; a 1 W rating fails | exact | `the_line_module_is_judged_against_its_ratings`, electrical |
| A disabled winding, 40 K at the start, thermal time constant 0.5 s, every machine | 40 / e = 14.715 K after 0.5 s, 40 / e² = 5.413 K after 1 s; kept on Enable; without winding heating it stays | 0.1 % | `a_disabled_winding_cools_by_the_exponential`, electrical |
| A disabled drive's DC link at 220 V, R C = 0.2 s, every machine, own and shared links | 220 V · exp(−0.25) = 171.336 V after 0.05 s; above the bus level at 69 ms, at it (155.13 V) from 70.5 ms (by hand 69.9 ms); a regenerative supply takes 160 V to the bus level in one step | 1e-3 V | `a_disabled_drives_dc_link_discharges_by_the_rc_exponential`, electrical |
| Disable mid-move and Enable, every machine, own and shared links, link at bus + 20 V, no bleeder | Windings' ½ L i² plus capacitors' ½ C U² the same before Disable, after the first disabled step (the winding's energy is in the link) and after Enable (the holding current's energy taken from the link) | 1e-9 relative | `disable_and_enable_conserve_the_electrical_energy`, electrical |
| Two drives, one braking 41 W and one drawing 23.5 W for 0.1 s, 470 µF each, from 190 V | Separate links: 231.40 V and 190 V. Shared: sqrt(2 · (16.967 + 1.75) J / 940 µF) = 199.56 V; with the chopper at 195 V the one resistor burns 0.845 J; one trip for both; links joined at 231.40 V and 190 V settle at 211.72 V | 0.01 V; 0.002 J | `a_shared_dc_link_rises_by_the_net_energy_of_its_drives`, drive |
| Shared DC link over the default cycle of a machine with two drives | The same voltage and resistor power on every drive at every step; highest voltage no higher than with separate links (224.06 V against 224.65 V on the gantry) | Exact | `a_shared_dc_link_is_one_voltage_for_all_drives_of_a_machine`, control |
| Notch filter | Removes its centre frequency, passes the rest | Gain below 0.01 at 60 Hz; within 2 % of 1 at 5 Hz and 600 Hz | `notch_removes_its_centre_frequency_and_passes_the_rest` |
| Position loop, velocity loop, torque limit, encoder steps, delay of one velocity cycle | Their defining formulas | 1e-12 | `position_loop_is_kv_times_following_error_plus_feedforward`, `velocity_loop_is_pi_with_torque_feedforward`, `torque_limit_follows_the_torque_speed_curve`, `encoder_resolution_quantises_position_and_speed`, `the_torque_setpoint_acts_one_velocity_cycle_late` |
| Fine interpolation, friction feedforward, balancing | pos 1 rad, 10 rad/s, 100 rad/s² after 3 ms: 1.03045 rad, Kv part 1.5225 rad/s; characteristic (10, 0.2), (100, 0.3): 0.1, −0.25, 0.3 N m at 5, −55, 200 rad/s; PT1 1 ms: 1 − e⁻¹, 1 − e⁻² | 1e-9 | `fine_interpolation_runs_the_setpoint_on_between_profile_cycles`, `the_friction_feedforward_is_the_characteristic_at_the_speed_setpoint`, `balancing_lags_the_position_setpoint_by_its_time_constant`, drive |
| Constant-inertia feedforward, load-side encoder, one encoder sample, single axis | J = 9.8e-5 + 4.16e-6 (belt) + 3.2e-5 (payload) kg m², torque_ff = J · 250 · a; following error = setpoint − 250 · round(x / 0.1 mm) · 0.1 mm; position loop's measured position = the encoder's reading with noise | 1e-9 | `the_constant_inertia_feedforward_is_j_times_the_motor_acceleration`, `a_load_side_encoder_measures_the_following_error_at_the_load`, `both_loops_read_one_encoder_sample_per_step`, control |
| 4 ms profile cycle on the single axis; D3 motor acceleration in a cruise | Peak torque within 5 % of the 1 ms one with fine interpolation, above 1.5x held; the motor acceleration setpoint equals the speed setpoints' central difference | 5 %, 1 % | `a_long_profile_cycle_with_fine_interpolation_keeps_the_peak_torque`, `the_motor_acceleration_setpoint_has_the_speed_term_of_the_kinematics`, control |

Stepper (`motor.kind = "stepper"`), in stepper: N = 50, k_t = k_E = 0.5 N m/A, R = 1 Ω, L = 2.5 mH, 2 A on 24 V, integrated with a rigid inertia as `Sim::step` integrates.

| What is compared | Against | Tolerance | Test |
| --- | --- | --- | --- |
| Holding torque; a load of 0.99 of it, brought on slowly | k_t I = 1 N m at a load angle of 90°; the rotor at asin(0.99) electrical behind its setpoint, 0.02858 rad, the current at 2 A | 1e-12; 2e-3 rad electrical, 1e-4 rad | `the_holding_torque_is_kt_times_the_rated_current` |
| Small-angle resonance, J = 1e-4 kg m², after one microstep | sqrt(N T_hold / J) / 2π = 112.54 Hz | 1 % | `the_rotor_rings_at_the_hand_calculated_resonance` |
| Corner speeds | Unloaded current: sqrt(24² − 2²) / (50 · 2.5e-3 · 2 + 0.5) = 31.89 rad/s, the current at 0.9 of it ≥ 0.99 I, at 1.3 of it < 0.95 I; torque: (R I + k_E ω)² + (N ω L I)² = 24², 39.70 rad/s, the pull-out torque 1 N m below, < 0.98 N m at 1.1 times | 0.01 rad/s; 1e-12 N m | `the_current_falls_short_above_the_hand_calculated_corner_speed` |
| A load of 1.3 T_hold for 30 ms, detent 0.05 N m | The rotor settles a whole number of electrical turns behind: lost full steps a multiple of four, at least four | 1 % of four steps | `a_load_above_the_holding_torque_slips_by_whole_multiples_of_four_full_steps` |
| Jerk-limited ramps on 0.01 kg m² to 20 rad/s and to 55 rad/s | 80 % of the pull-out torque keeps in step, 110 % falls out (load angle past π); at 55 rad/s the pull-out torque 0.793 N m by hand (the voltage disc's centre (−0.570, 3.917) A, radius 3.454 A on the 2 A circle: Re I = 1.586 A) | 1e-3 N m | `a_ramp_beyond_the_pull_out_torque_stalls_and_one_at_80_percent_does_not` |
| The stepper belt axis (`machines/stepper-axis.toml`), its cycle; with a 25 kg payload | Every check passes, load angle 0.42 rad, 44 % of the pull-out curve, no `motor-s1`; 26 kg · 5 m/s² · 0.016 m = 2.08 N m over the holding 1.11 N m: `stepper-synchronism` fails above π; the energy budget closes | 1e-6 (electrical), 1e-3 | `the_stepper_axis_runs_its_cycle_in_step_and_a_heavy_payload_makes_it_lose_steps`, motor_kinds |

Induction motor (`motor.kind = "induction"`), in induction: four poles, 400 V, 50 Hz, 1440 rpm, R_s = 4.5 Ω, R_r = 3.5 Ω, L_m = 0.45 H, both leakages 0.02 H, 600 V DC link.

| What is compared | Against | Tolerance | Test |
| --- | --- | --- | --- |
| Synchronous speed, rated slip; torque at the rated slip | 1500 rpm, 4 %; about the rated 9.95 N m | 1e-12; 1 N m (round numbers, not fitted) | `synchronous_speed_and_rated_slip_come_from_the_name_plate` |
| Torque-slip curve with R_s = 0 at s = 0.02, s_b and 0.6; breakdown with R_s | Kloss, T_b = 3/2 p U_th² / (2 ω (X_th + X_σr)) = 37.96 N m at s_b = R_r / (X_th + X_σr) = 0.2846; the Thevenin form with R_th = 4.121 Ω, X_th = 6.141 Ω, U_th = 312.55 V: 27.10 N m, the curve's top over slips 0.20 to 0.28 | 0.2 %; 0.01 N m, 0.5 % | `the_torque_slip_curve_is_kloss_and_the_breakdown_is_the_equivalent_circuits` |
| Starting current, rotor held, rated voltage and frequency | Z = (7.7071 + j 12.388) Ω: 326.60 / 14.590 = 22.385 A amplitude, 15.83 A RMS | 0.2 % | `the_starting_current_is_the_equivalent_circuits` |
| V/f, a 5 N m load step at 50 Hz, J = 0.05 kg m² | Slip T = 3/2 p ψ_r² ω_slip / R_r: 2.947 rad/s at the shaft (ψ_r = 0.995 Wb unloaded); compensated less than 0.4 rad/s | 6 %; 0.4 rad/s | `under_v_f_a_load_step_costs_the_hand_calculated_slip` |
| Field-oriented flux build-up from no flux | L_r / R_r = 134.3 ms: 63.2 % at one, 95.0 % at three | 0.01 | `the_rotor_flux_builds_with_l_r_over_r_r` |
| Field-oriented torque step of 5 N m, rotor held | 63.2 % after the current loop's T = σ L_s / Kp = 0.5 ms; 5 N m held | 0.05; 0.2 % | `under_field_orientation_a_torque_step_answers_with_the_current_loops_time_constant` |
| The induction lift (`machines/induction-lift.toml`) field-oriented; flat on V/f | Every check passes, no `motor-s1`; on V/f the motor checks pass and the carriage stops 2 to 20 mm off its targets (open loop); the energy budgets close | 1e-6 (electrical), 1e-3 | `the_induction_lift_runs_its_cycle_field_oriented`, `the_induction_axis_runs_its_cycle_on_v_f`, motor_kinds |
Thermal model of gearboxes and guides (`effects.part_heating`), in views:

| What is compared | Against | Tolerance | Test |
| --- | --- | --- | --- |
| A guide at constant speed: its rise one time constant after starting from cold | R_th · P · (1 − 1/e), P the guide's loss power | 1 % | `a_guide_at_constant_speed_warms_to_resistance_times_its_loss`, views |
| The same guide twelve time constants later | R_th · (λ F_c + b v) · v by hand from the config, 2 K/W (about 51 K on the T-gantry at 0.8 m/s, about 72 K on the single axis at 1 m/s) | 1 % | the same test |
| The 3D colour of a motor housing and a heated part | Rise over rated rise: 25 K of 100 K is 0.25, half the rated rise 0.5 | exact | `the_scene_colours_warm_parts_by_their_rise_over_the_rating`, views |

The winding's own model (settles at the rated rise at the rated current, 51.3 K after one time constant from cold, 19.34 K at half the current) is `the_winding_warms_to_the_rated_rise_at_the_rated_current`, drive.

### Stops and brake delays

On the rigid, loss-free single axis (`ideal_rigid`: 6.125 kg reflected, 4 kg carriage, r = 0.02 m, i = 5), stopped from 1 m/s (machine-single-axis). Each was seen to fail with its piece taken out: the OFF3 ramp (braking at the torque limit instead), category 2's braking plan, the brake delay.

| What is compared | Against | Tolerance | Test |
| --- | --- | --- | --- |
| Category 1 on an OFF3 ramp of 0.2512 s from 314 rad/s (5 m/s² at the carriage) | v² / (2 a) = 0.1 m, v / a = 0.2 s; peak deceleration over 1 ms above 5 m/s² (the speed loop overshoots the ramp's start: 6.0) | 3 %; peak below 6.5 | `the_stops_travel_the_hand_calculated_distance_in_the_hand_calculated_time` |
| Category 2 at the axis' 10 m/s² | 0.05 m; the plan in 0.1 s, the tool below 1 mm/s within 0.12 s (the position loop's lag) | 3 %; 20 % on the time; peak below 15 | same |
| Category 0, 10 N guide friction (Coulomb = stiction), 0.2 N m brake (50 N), 0.03 s engagement delay | v t − F t² / (2 m) + m v₁² / (2 (F + F_b)) = 0.075435 m; without the delay 0.051042 m | 1 % | `a_category_0_stop_coasts_the_hand_calculated_distance_with_the_brake_delay` |
| A hanging axis (upright) taken off in category 0, 0.03 s engagement delay, 10 N friction | (39.227 − 10) N / 6.125 kg · 0.03² / 2 = 2.1473 mm before the brake holds, after 480 steps; none without the delay | 2 % | `a_hanging_axis_sags_by_hand_while_its_brake_engages` |
| The DC link in a category 1 stop | 3.0625 J less 0.0593 J copper loss: 470 µF from 155.13 to 191.95 V; with 4.7 µF the resistor takes 2.9412 J | 0.3 V; 2 % | `a_category_1_stop_brings_the_hand_calculated_energy_to_the_dc_link` |

The stop analysis (`sim-cli stop`, `stop::study`) reports a worst stop with finite figures on every registered machine in categories 0, 1 and 2 (`the_stop_study_reports_the_worst_stop_on_every_machine`, sim-machines, category 1; the command line `a_run_can_stop_the_machine_and_stop_finds_the_worst_moment`). Nothing compares its distances with a real machine's stop: no measured stopping distance is in the repository.

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
| Fieldbus jitter of N cycles at 250 rad/s (1 m/s), over the ~600 cycles of a cruise | Frames 0 to N cycles late, the newest arrived used: P(more than i late) = prod (N − k)/(N + 1); setpoint RMS 0.25 rad · sqrt(E[c²]) = 0.1768 rad (N = 1), 0.2887 rad (N = 2); shares 1/2, 1/2 and 1/3, 4/9, 2/9; switched on with 0 cycles the same setpoints as off | 5 %; 0.05; exact | `a_fieldbus_jitter_spreads_the_setpoints_by_the_hand_calculated_rms`, axis |
| A second ZVD shaper, modes at 25 and 62 Hz (undamped shapers) | Residual cos²(π f / (2 f_s)): shaper at 25 Hz leaves 0 and 0.5314, at 62 Hz 0.6503 and 0; both together under a tenth of each (measured 0.0000 and 0.0001) | 0.01 (0.03 at 62 Hz), 0.02; 0.1 | `a_second_shaper_suppresses_a_second_resonance_by_the_zvd_formula`, axis |
| Open-loop response of the velocity loop, rigid loss-free axis | \|L\| = Kp \|1 + 1/(jωTn)\| \|1 + Kv e^(−jω 0.5 ms)/(jω)\| / (J ω \|1 + jωT\|), J = 9.8e-5 kg m², T = 0.25 ms: 0.3973 at 100 Hz, 0.1898 at 200 Hz (measured 0.3976, 0.1914); twice the gain takes 6.02 dB off the gain margin | 2 %; 0.1 dB | `the_velocity_loop_response_is_the_hand_calculated_loop`, axis |
| Margins read off an integrator with dead time, L = (ωc / jω) e^(−jωτ), 100 Hz, 1 ms | Phase margin 90° − 360° fc τ = 54°; gain margin 20 log 2.5 = 7.959 dB at 250 Hz | 0.1°; 0.01 dB | `margins_of_an_integrator_with_dead_time_are_the_hand_calculated_ones`, analysis |
| The same loop stable and unstable, 10 Hz to 2 kHz | Phase margin 90° − 360° fc τ, gain margin −20 log (4 fc τ), signed: 100 Hz, 0.5 ms 72°, 13.979 dB; 200 Hz, 0.5 ms 54°, 7.959 dB; 300 Hz, 1 ms −18°, −1.584 dB; 400 Hz, 1 ms −54°, −4.082 dB; a point without a measurement makes them invalid and NaN | 0.1°; 0.01 dB | `margins_are_signed_and_say_an_unstable_loop_is_unstable`, analysis |
| A conditionally stable loop, L = K (1 + s/a)² / (s (1 + s/b)² (1 + s/c)), a, b, c = 2π · 300, 30, 3000 Hz, K = 3e5, 6.1e5, 1.2e6, 60 points 10 Hz to 2 kHz | Stable (closed-loop poles' largest real parts −977, −1355, −1459 1/s): the fall through −180° beyond −1 is undone by the rise; phase margins 30.7°, 41.6°, 42.4° at 589, 1002, 1710 Hz, gain margin infinite (it read −53.7, −59.8, −65.7 dB and unstable) | 0.3°; 1 % | `a_conditionally_stable_loop_is_stable`, analysis |
| A type-0 lead loop with a positive phase at the first point, L = 3 (1 + s/ω5) / ((1 + s/ω50) (1 + s/ω200) (1 + s/ω400)) | Phase margin 24.0° at 1516 Hz by hand (it read −336.0°), stable, gain margin infinite | 0.2°; 1 % | `a_lead_with_positive_phase_at_the_first_point_keeps_its_phase_margin`, analysis |
| A loop whose first crossover rises out of an anti-resonance with a phase lead, L = (2π · 9.7 / s) (1 + 0.1 s/ωz + s²/ωz²) / (1 + 0.1 s/ωp + s²/ωp²) e^(−s · 1 ms), ωz, ωp = 2π · 14, 40 Hz, 60 points 10 Hz to 2 kHz (the parallel SCARA's arm 2 as measured) | First crossover 22.5 Hz at +71.4° (a margin of 251.4°); phase margin 58.1° at 94.3 Hz, by hand from the formula on a fine grid (it read 58.1° − 360°: the phase had been turned at the first crossover) | 0.5°; 2 % | `a_crossover_with_phase_lead_before_the_one_that_counts_does_not_turn_the_phase`, analysis |
| Single axis with a velocity gain of 0.3 N m s/rad (twelve times the shipped one) | Not stable (the loop runs into its torque limit; the shipped gain is stable); `sim-cli margins` says FAIL, on every machine at that gain | exact | `an_unstable_velocity_loop_is_reported_unstable`, axis; `step_bode_and_tuning_write_csv_for_every_machine`, cli |
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
| Two arcs on one circle (0.05 m) that turn 4 rad and π + 0.05 rad, between two lines, tolerance 0.1 mm | The planned path as long as the given one, 0.05 + 0.05 (4 + π + 0.05) + 0.1 = 0.5096 m (it was 0.1953 m: a whole turn lost), but for what the curves may change, and turning as far | 1e-9 m; curves' length plus deviation times angle; 1e-6 rad | `two_arcs_on_one_circle_keep_their_whole_turn`, path |
| 400 random paths of tangent lines and arcs (radius 2 mm to 0.2 m, 0.05 to 6 rad either way, half the arcs continuing on the circle before), tolerance 0.01, 0.1 and 1 mm | Length as given but for the curves' lengths and the moved arcs' deviation times angle; turning as given | As stated; 1e-6 rad | `blended_random_paths_keep_their_length_and_turning`, path |
| The time-optimal plan and its deviation with a torque reserve of 0.1 %, every machine | Both refused alike (`error-path-torque` where a motor holds the load against gravity) | Exact | `the_time_optimal_deviation_refuses_what_the_plan_refuses`, control |
| A spline that swings out of the range, and a line, a clockwise arc out of the range and a line with 0.1 mm of tolerance, both planners | Plan and deviation both refused with `error-path-range` (the deviation was reported, then the plan refused) | Exact | `path_error_on_a_circle_matches_the_hand_calculation`, gantry |
| A line into a quarter arc of 0.1 m that ends the path, and the same with a corner after the arc, tolerance 0.1 mm | The arc's end fixed: circle shrunk and moved by s = e / (1 − cos 90°) = e, so L = sqrt(24 e r) = 15.49 mm and 0.3957 m/s into the arc (0.0707 m/s without); the arc ends exactly on (0.6, 0.25) in its direction; every pose within the reported 0.1 mm; the axis limits | 0.5 %; 2 mm/s; 1e-12 m; 1 nm; 0.1 % | `an_arc_at_an_end_or_a_corner_gets_its_transition_curve`, path |
| A line, a quarter arc of 0.1 m and a line in a plane tilted by 30° about x, three axes (3 m/s, 50 m/s², 2500 m/s³), tolerance 0.1 mm; then on into a 0.05 m arc in the plane x = 0.1 | Needed v³ dk / (0.4 j) = 2³ · 10 / 1000 = 80 mm, allowed L = sqrt(24 e r) = 15.49 mm, so e = L² / (24 r) = 0.1 mm and the arc's radius 99.9 mm; every pose in the tilted plane; no step in position, direction or curvature; every pose within the reported deviation; 0.353 s against 0.418 s; with the second arc all four junctions blended (the line between the planes shared out) | 5e-3 relative (curves), 1e-12 m (plane) | `transition_curves_on_three_axes_lie_in_the_plane_of_the_arc`, path |
| A right-angled corner between two lines, `profile.corner_tolerance_m` = 1 mm | d = 16 e / 3 = 5.333 mm, the curve's middle 3 d / 16 = 1 mm off each line, radius there sqrt(2) d / 3 = 2.514 mm; lines cut back by d; no stop at the corner | 1e-12; 1e-9 m | `a_right_angled_corner_is_rounded_within_the_tolerance_at_the_hand_calculated_radius`, path |
| A quarter arc of 0.1 m on four axes, the fourth carried 0 -> 1 | Length sqrt((0.05 pi)^2 + 1) = 1.01226; at a share w of it the arc at w pi / 2 and the fourth axis at w | 1e-12 | `a_four_axis_arc_keeps_its_fourth_axis_linear_in_the_arc_length`, path |
| Two tangent arcs of 0.2 and 0.1 m radius, the second slow only at its far end, time-optimal | Junction at sqrt(0.5 j T / dk) = sqrt(0.5 * 100 * 0.001 / 5) = 0.1 m/s, not at the 0.05 m/s of the slowest place | 1e-12 | `the_junction_of_two_arcs_is_passed_at_the_hand_calculated_speed`, path |
| A line into a line whose speed limit falls from 1 to 0.3 m/s, time-optimal | Joined on the speed curve: quicker than on a cruise at the slowest place | Comparison | `a_path_without_the_forced_cruise_is_quicker_than_with_it`, path |
| Single axis, `profile.cycle_timing = "time_optimal"`, first segment 0.1 -> 0.9 m with 0.8 * 0.5 N m | 0.8 / v + v / a + a / j = 0.71970 s (a = 16.327 m/s^2, v = 0.99 · 1.256 = 1.2434 m/s, j = 1e5 m/s^3), quicker than the S-curve within the axis limits | 2.5 ms | `a_time_optimal_cycle_segment_takes_the_hand_calculated_time`, single axis |
| Single axis, time-optimal line, gearbox efficiency 0.9 | 0.0075 + 0.017 / 0.9 = 0.026389 N m per m/s^2 accelerating (15.158 m/s^2) and 0.0075 + 0.017 * 0.9 = 0.0228 braking (17.544 m/s^2): 0.71999 s | 1e-3 relative; 1.5 ms | `a_lossy_gearbox_lowers_the_time_optimal_acceleration_by_its_loss`, single axis |
| The same tilted path on the delta, both planners, `profile.path_tolerance_m` = 0.1 mm | Reported 0.1 mm = L² / (24 r) within the axis limits; every pose within it of the given path; 0.551 s against 0.616 s, time-optimal 0.474 s (0.516 s before 16 November) against 23.6 s as given (0.578 s before; the given line at z = −0.9 m crawls where the D3's inverse kinematics gives motor 2 at 0 at x = −0.15 m exactly, an open fault of `spatial::Legs::inverse_moved`) | 1e-9 relative; 0.1 µm | `a_path_in_a_tilted_plane_gets_its_transition_curves`, delta-d3 |
| A line, a quarter arc of 0.1 m, a line, a quarter arc of 20 mm, a line, tolerance 1 mm: one length per arc | Wide arc at 0.4 m/s: L = 0.4³ · 10 / (0.4 · 100) = 16 mm (15.99 fitted); tight arc at (0.4 j r²)^(1/3) = 0.252 m/s: L = 0.016 · 50 / 40 = 20 mm (19.67 fitted: e = L² / (24 r) is the first term, and the curve turns 0.5 rad); reported 0.02² / 0.48 = 0.833 mm (one length for the path made both 20 mm) | 5e-3 and 2e-2 relative; 2e-3 | `each_arc_gets_the_curve_its_own_junctions_need`, path |
| An arc fixed at the path's start that hands over to an arc (an S and the same way) and a line, tolerance 0.1 mm | Both junctions blended, the first arc leaves the start along +x, no steps, every pose within the reported deviation (≤ 0.1 mm), faster | 1e-12 | `an_arc_fixed_at_one_end_gets_its_curve_towards_an_arc`, path |
| A line into a 270° arc of 0.1 m that ends the path, tolerance 0.1 mm | s ≤ tolerance / 2 = 0.05 mm, e = s (1 − cos 270°) = 0.05 mm, L = sqrt(24 e r) = 10.95 mm, reported 2 s = 0.1 mm; the arc ends on (0.4, 0.25) heading +y | 5e-3 relative; 1e-12 | `an_arc_fixed_at_one_end_that_turns_three_quarters_gets_its_curve`, path |
| A line into one spline span ending without curvature, curvature 3 dz / h² = 10 1/m at the junction, tolerance 0.1 mm | Eased over l = sqrt(e / (0.01728 dk)) = 24.06 mm, moved by 0.1 mm; junction speed cbrt(0.4 j / (1.05 (9 dk / l + d0 × 6 c3))) = cbrt(40 / (1.05 · 3635.2)) = 0.2189 m/s (measured 0.2188), 0.0707 m/s without; every pose within the reported 0.1 mm of the spline as given (and above 0.09 mm); the axis limits | 10 nm; 1 %; 0.1 um sampling; 0.1 % | `a_spline_eased_into_a_line_is_entered_at_the_hand_calculated_speed`, path |
| Four spline paths (between lines, alone, between arcs, a tight S) with a tolerance of 0.1 mm | No step in position, direction or curvature at any junction; reported deviation at most 0.1 mm, zero for the spline alone; every pose within it of the path as given; the axis limits; via points passed; faster where eased (2.129 s against 2.251 s, 2.415 s against 2.701 s, 2.093 s against 2.184 s) | 1e-11 m, 1e-9, 1e-6 1/m; 0.1 um; 0.1 % | `a_spline_passes_through_its_via_points_within_the_axis_limits`, path |
| A line that ends on the end of a stroke | Every pose within the range, the last one the given point (it ended 2.8e-17 m beyond) | Exact | `a_path_onto_the_end_of_a_stroke_stays_within_the_range`, path |
| Both planners on the path with transition curves | Every pose within the reported 0.1 mm of the given path, and reaching it | 10 µm (the start pose) | `both_planners_time_the_path_with_its_transition_curves`, gantry |
| Path error and distance behind the setpoint on a circle at constant speed, no feedforward | R (1 − gain) and R (ω / Kv) gain, with gain = 1 / sqrt(1 + (ω / Kv)²) | 10 % | `path_error_on_a_circle_matches_the_hand_calculation`, `a_circle_timed_by_the_planner_gives_the_hand_calculated_path_error`, gantry |
| Time-optimal line, jerk limit 1e5 m/s³ and 100 m/s³ | Distance / v + v / a + a / j = 0.720 s; distance / v + 2 sqrt(v / j) = 0.866 s (v = 1.2434 m/s, the rated speed less the 1 % kept back), with the acceleration peaking at sqrt(v j) = 11.2 m/s² | 1.5 ms; 0.5 % | `a_time_optimal_line_takes_the_hand_calculated_time`, axis |
| Belt tension as a limit of the time-optimal plan: default T-gantry, line along X from x = 0.7 to 0.95 at 300 m/s³, 400 N allowed | Rigid tensions T = P + d_i with Σ (T_i − P) L_i = 0 (by hand at x = 0.9, 10 m/s²); acceleration 0.5 · (400 − 31 − 284.12) / 6.679 = 6.355 m/s² at the far end (share 0.5, margin 31 N, the weight on Z 7.16 kg), 7.05 m/s² at x = 0.7 where it accelerates: the plan takes the bound at each pose (none above it, one within 0.5 %); rigid tension within 1 % of the 326.7 N the plan allows; the full model passes `belt-tension`; without the limit 19.3 m/s² fails it | 1e-9 N; 0.2 % / 0.5 %; 1 % | `the_time_optimal_plan_keeps_the_belt_within_its_tension_limit`, gantry |
| The belt's ratings on the full model, time-optimal at 40 m/s², 2000 m/s³, 2 m/s: three lines, rated 340 to 1500 N, and a five-line path at 400 N; 330 N at the default limits | Both ratings kept (highest ≤ rating, slackest > 0 N), or refused (`error-path-tension`) where the holding tension leaves no room (330 and 340 N); with share 0.8 and no margin 331.4 N on 330 N | exact | `the_full_model_keeps_the_belt_ratings_on_a_time_optimal_plan_at_high_jerk`, gantry |
| A cycle segment with via points | Runs up, through the via point and down without a stop, arrives on its target before the dwell; a via point outside the stroke is refused by name | 1e-9 m | `a_cycle_segment_runs_its_path_through_via_points`, gantry |
| Time-optimal line-arc-line: torque of the rigid model, motor speed and axis jerk at every planned pose, by differences | Within the torque reserve, the rated speed and the jerk limits; one of them used to 97 % or more at over 60 % of the poses (65 %: jerk 58 %, speed 10 %; with a 0.6 N m motor and 2000 m/s³ 75 %: torque 33 %, speed 38 %, jerk 5 %); 1.397 s (1.395 s before 16 November) against 1.828 s within the axis limits (1.429 s with one limit per piece) | 0.1 %; below 0.8 | `the_time_optimal_plan_is_shorter_keeps_a_limit_active_and_stays_on_the_torque_speed_curve`, gantry |
| The same plan on the full model | Utilisation below 0.6 (0.18); speed below 1.03 × rated; path error below 1 mm | As stated | the same test |
| Limits that change within a piece: default T-gantry, line along X from x = 0.1 to 0.9 at 300 m/s³, belt rated 400 N | The belt allows 16.205 / max(2.6 − x, 1.6 + x) m/s² at x (by hand; 16.137 before the belt's vertical runs weighed on Z); one limit per piece takes 6.482 m/s² all along, 0.8583 s at the speed the plan keeps (0.8531 s at 1.2559 m/s and 6.455 m/s² before); point by point 6.455 to 6.73 m/s² along the ramp, 1.0 to 2.5 ms shorter (0.8517 s), no pose above its bound, one within 0.5 %, the sections' rigid change of tension within 1 % of the 42.26 N allowed, torque, speed and jerk within their limits; following the limit (14 November) faster than the plan of 13 November (0.851565 s against 0.851693 s; on a 125 µs profile cycle it ends before 0.851693 s); since the belt's own mass (16 November) 0.857 s against 0.8576 s with one limit per piece | 0.2 %; 0.1 % | `the_time_optimal_plan_takes_the_belt_limit_where_it_is_along_a_line`, gantry |
| Limits that change within a piece on the delta: a line across the range, (0.5, 0, −0.85) to (−0.5, 0, −0.85) | The rated speed allows the tool 7.270 m/s where the motors turn fastest per tool speed (the line's end) and 13.31 m/s where slowest (inverse kinematics at 801 places); the plan goes above 1.1 times the slowest place's speed (8.1 m/s), keeps torque (with the speed's centripetal and Coriolis torque), motor speed and axis jerk within their limits, and takes under 0.419 s against 0.4290 s with one limit per piece (gearboxes' M2max lifted); with M2max 192 N m each gearbox's output torque, ratio · (motor torque − 2.106e-4 kg m² · the motor's angular acceleration), within 0.8 · 192 N m and up to it, and the line takes longer (0.434 s) | 1e-3 m/s; 0.1 % | `the_time_optimal_plan_takes_the_motor_limits_where_they_are_along_a_line`, delta |
| The drive's maximum current as a limit of the time-optimal plan: rigid single axis, 30° incline, 2 A on a 0.45 N m/A motor (3.8 N m peak), 1e5 m/s³ | a = (0.8 · 0.9 N m · 5 / 0.02 m ∓ 19.613 N) / 6.125 kg = 26.186 m/s² up, 32.590 m/s² down; 0.8 / v + (v / a1 + a1 / j) / 2 + (v / a2 + a2 / j) / 2 = 0.68649 s at v = 1.2434 m/s, with `effects.drive_current_limit` and without it (the verdict judges the rating either way); with the rating lifted the motor's (0.8 · 3.8 · 250 − 19.613) / 6.125 | 1.5 ms; 0.1 % | `the_drive_current_limits_the_time_optimal_acceleration_to_the_hand_calculated_one`, axis |
| The gearbox's M2max and the pulley's torque rating as limits of the time-optimal plan: rigid single axis, 1e5 m/s³ | The output drives carriage and pulley, 0.085 N m per m/s² (the motor's 0.0245 less the rotor's 0.0075, times 5): a = 0.8 · 10 / 0.085 = 94.118 m/s² at M2max 10 N m, 0.8 · 8 / 0.085 = 75.294 m/s² with the pulley rated 8 N m, both ways; 0.8 / v + v / a + a / j | 1.5 ms; 0.1 % | `a_gearbox_or_pulley_rating_that_binds_limits_the_time_optimal_acceleration`, axis |
| A short time-optimal line, rest to rest: rigid single axis, 0.05 m, 0.8 · 0.5 N m, 1e5 m/s³ | T = τ + sqrt(τ² + 4 s / a), τ = a / j, a = 16.327 m/s² (the bang-bang 2 sqrt(s / a) = 0.11068 s with no jerk limit): 0.11084 s, peaking at 0.902 m/s | 1.5 ms; 1 % | `a_short_time_optimal_line_takes_the_hand_calculated_bang_bang_time`, axis |
| A hanging axis, time-optimal up and down: the single axis upright, 0.8 · 0.5 N m | Speeding up at (100 − 39.227) / 6.125 = 9.922 m/s², braking at (100 + 39.227) / 6.125 = 22.731 m/s² on the way up (swapped on the way down): the speed follows min(sqrt(2 a d), 1.2434 m/s) at 0.05 m from the start, in the middle and 0.02 m before the end, both ways | 0.5 % | `on_a_hanging_axis_the_time_optimal_speed_follows_the_hand_calculated_limit_curve`, axis |
| The braking resistor's peak power as a limit of the time-optimal plan: 50 W on the rigid single axis | Braking at no more than P / (m v) = 50 / (6.125 v): 6.565 m/s² at 1.2434 m/s; at 1.2, 1.0 and 0.8 m/s within 5 % below (the speed levels) and 1 % above; speeding up at the motor's 16.327 m/s² | 5 % / 1 %; 0.1 % | `the_braking_resistor_bounds_the_time_optimal_deceleration`, axis |
| The gearbox's maximum input speed as a limit of the time-optimal plan: 2000 r/min on the single axis | Top speed 0.99 · 2000 · π / 30 · 0.02 / 5 = 0.82938 m/s | 0.1 % | `the_gearbox_input_speed_caps_the_time_optimal_top_speed`, axis |
| `profile.time_optimal_margin` 0.9 on the single axis | Top speed and acceleration 0.9 of 1.2434 m/s and 16.327 m/s², 0.8 / v + v / a + a / j | 1.5 ms; 0.1 % | `the_time_optimal_margin_plans_at_its_share_of_the_limits`, axis |
| A screw's speed rating as a limit of the time-optimal plan: the 1.6 m screw of the screw axis | Whirling at 1250 r/min on the 20 mm lead: top speed 0.99 · 1250 / 60 · 0.02 = 0.41250 m/s | 0.1 % | `the_time_optimal_plan_keeps_the_screws_speed_rating`, joint kit |
| A rack's feed force as a limit of the time-optimal plan: rated 500 N, service factor 1.25 | 0.8 · 400 N / 80 kg = 4 m/s² | 0.2 % | `the_time_optimal_plan_keeps_the_racks_feed_force`, joint kit |
| Every machine's cycle timed time-optimal, run with the shipped config | Every verdict the plan keeps passes ("motor-peak", "motor-speed", "drive-peak-current", "gearbox-peak", "pulley-torque", "gearbox-input-speed", "guide-load", "belt-tension", "belt-least-tension", "braking-resistor-peak", the machine's own checks with a rigid value); where "motor-thermal" fails (linear motor axis, flying saw, parallel SCARA, four-arm delta), with the winding at its starting temperature | exact | `time_optimal::machine_NN`, sim-machines |
| The guides' load rating as a limit of the time-optimal plan: default T-gantry, line along X at z = 0.35 m, rated 150 N | X guide 11 kg · 9.80665 + 5.833 kg · a_x, Z guide 12.833 kg · a_x (`guide_loads`); a = (150 − 107.87) / 5.833 = 7.222 m/s², 0.8 / 1.2559 + 1.2559 / 7.222 + 7.222 / 100 = 0.8831 s; no pose's load above 150 N; torque, speed and jerk within their limits | 0.2 %; 1.5 ms; 0.05 % | `the_guide_load_rating_limits_the_time_optimal_acceleration`, gantry |
| The speed curve a piece follows (`path::smoothed`): places at 1 m/s and one at 0.5 m/s, 10 m/s², 10 mm apart | x = v² rises 0.25, 0.43, 0.61, 0.79, 0.97, 1 m²/s² from the slow place (0.18 per place = 2 · 0.01 · 0.9 · 10); at 50 m/s³ also x'' within ± 2 · 0.5 · 50 · 0.01² / v, never above a top or below the slow place | 1e-12; 1e-4 | `the_speed_curve_comes_down_to_a_slow_place_within_its_acceleration_and_jerk`, path |
| A piece under a speed curve that falls faster than the jerk can follow (tops 4 to 2 m/s over 0.4 m, 50 m/s², 20 m/s³), rest to rest | The jerk alone bounds it: four phases of T = (d / 2j)^(1/3) = 0.215443 s, 0.861774 s in all, 0.928 m/s at most. Before 15 November the run stayed at rest under the falling curve and the piece was refused as a crawl (a turn about z alone of the hexapod and the platform on cranks: `a_turn_about_one_rotary_axis_alone_is_timed_on_every_machine`, time_optimal) | 2 ms; 5 mm/s | `a_piece_under_a_steeply_falling_speed_curve_takes_the_hand_calculated_time`, path |
| A turn about one axis (no turn to (0.075, 0.075, 0.1) rad, and on along that axis) | Turns about one axis add: the rotation vector runs along the straight line at the line's rate, with no second derivative (by differences it had 2e-8 rad/m² of rounding, and the time-optimal plan gave the piece 0.4 of its axes' jerk); a turn that changes its axis still bends | 1e-15 | `a_turn_about_one_axis_does_not_bend_its_rotation_vector`, path |
| Time-optimal line, quarter turn of 5 mm radius, line, with a path tolerance of 0, 0.1 mm and 1 mm | The same three limits at every planned pose; ends on the end point; utilisation on the full model below 1 (0.18; it was 10.5) | 0.1 %; 1 nm | `a_time_optimal_plan_through_a_tight_arc_stays_within_torque_speed_and_jerk`, gantry |
| 200 random paths of two to five tangent lines, arcs and splines (radius 2 mm to 300 mm, pieces 1 mm to 400 mm, splines of one to three via points turning by up to 0.6 rad each), half with a tolerance, half with a 0.6 N m motor and 2000 m/s³ | The same three limits at every planned pose (most used: torque 1 − 2e-4, speed 1 + 3e-5, jerk 1 + 1e-8; the longest plan 1.368 s (1.47 s before 16 November), 1.73 s with one limit per piece); ends on the end point | 0.1 %; 1 nm | `time_optimal_plans_of_random_paths_stay_within_torque_speed_and_jerk`, gantry |
| A single spline piece between two points at rest | The straight line between them, the distance along it exact | 1e-12 m; 1e-14 m | `a_spline_passes_through_its_via_points_within_the_axis_limits`, path |
| Four spline paths (between two lines; alone; out of an arc into an arc; an S of via points 20 mm apart): geometry | Every via point is the end of a span; no step in position or direction anywhere, none in curvature between spans | 1e-12 m; 1e-9; 1e-6 1/m | the same test |
| The same: rates of both axes by differences; nearest planned pose to every via point | The axis limits; half of what the tool travels in a cycle | 0.1 %; 0.4 mm | the same test |
| Tool swing after a traverse with input shaping | Under a tenth of the swing without (following the height: about a tenth, 0.102 at z = 0.15 m with the velocity gain of 0.04) | Factor 0.1 (following: 0.11) | `input_shaping_leaves_under_a_tenth_of_the_tool_swing`, `a_shaper_that_follows_the_height_leaves_about_a_tenth_at_both_heights`, gantry |

## Hand calculations: analysis and verdicts

| What is compared | Against | Tolerance | Test |
| --- | --- | --- | --- |
| Cycle time, peak torque, RMS torque, peak speed, utilisation of a reference cycle (rigid, loss-free, ideal drive) | The S-curve cycle by hand | 2 ms; 2 %; 2 %; 1 %; 2 % | `reference_cycle_torque_matches_hand_calculation`, axis |
| Energy in minus energy fed back | The copper loss | 3 % | the same test |
| Settled winding rise | The formula from the RMS current | 1e-9 K | the same test |
| Frequency sweep of the rigid machine | A pure inertia: gain 1 / (J ω), phase −90° | 1 %; 1° | `frequency_sweep_of_the_rigid_machine_is_a_pure_inertia`, axis |
| Phase of the sweeps at the time the signal has | The pure inertia at −90° at 320 Hz and 1 kHz (half a base step of misalignment would be 3.6° and 11.25°); the closed loop at 1 Hz, Kv 50/s, at −atan(2π / 50) = −7.1615° | 0.1° | `the_sweeps_measure_the_phase_at_the_time_the_signal_has`, axis |
| Dip in the frequency sweep | The load's resonance on the belt by hand | 5 % | `frequency_sweep_shows_the_belt_resonance`, axis |
| Verdict values of the reference cycle | Peak torque at the gearbox output 0.85 N m and its cubic mean 0.337 N m by hand (4.25 kg · 0.02 m · 10 m/s² and · (100 / 1.6)^(1/3) m/s²; the motor torque times the ratio gives 1.225 N m); input speed 2387 r/min; radial load 600 N; guide load 4 kg · g; DC link room 6.204 J | 2 %; 1 %; exact; 1e-9; 1e-3 J | `verdicts_match_the_hand_calculation_and_fail_where_a_part_is_too_small`, axis |
| Guide life | 47.3 million km for a cubic mean of 50.92 N | 1 % | the same test |
| Lives and tooth load from given loads | Pulley shaft bearing (ISO 281): C 8469 N, P 600 N, pulley at 100 r/min (motor 500 r/min, ratio 5): 468696 h. Gearbox planet bearings (Siemens): 20000 h · 175 / 100 · (40 / 20)^(10/3) = 352778 h. Guide: (5000 / 1000)³ · 50 km = 6250 km, at 0.25 m/s 6944 h. Belt teeth (Megadyne): 2 N m up to 95.49 r/min of the pulley, 34.045 N/cm allowed, 100 N · 1.4 / (34.045 · 2.5 · 12, 14 teeth capped) = 0.13707; beyond the table's end its last value: 0.14359. A life judged from below: utilisation 2.88, badness 13056 h | 1e-5 relative | `the_lives_and_the_tooth_load_are_the_hand_calculated_ones`, analysis |
| a_ISO of ISO 281:2007 | ν1 = 45000 · 100^−0.83 / 6 = 164.082 mm²/s (100 r/min, D_pw 36 mm), 4500 / sqrt(1000) / 6 = 23.717 mm²/s (1000 r/min). With e_C · C_u / P = 0.5 · 150 / 600 = 0.125: a_ISO 3.27647 at κ = 1 (0.1 · (1 − (2.5671 − 1.9987)^0.83 · 0.5)^−9.3), 0.198152 at 0.2, 0.610291 at 0.5, 6.03577 at 2, 11.0606 at 4 and above, 0.100176 at 0.05 (taken as 0.1); capped at 50; the three branches continuous. The pulley bearing above with ν 120 mm²/s: κ 0.731341, a_ISO 1.555262, 728945 h. Some keys without the others, or e_C above 1, refused by name | 1e-5 relative | `a_iso_is_the_hand_calculated_one_of_iso_281`, analysis |
| Gearing fatigue above the rated torque | Palmgren–Miner on a line through M2max 64 N m at 30000 revolutions, exponent 6, M2N 40 N m: 1000 rev/h at 48 N m (30000 · (64 / 48)^6 = 168560 rev allowed) and 100 rev/h at 60 N m (44187 rev), 5000 rev/h at 30 N m no damage: 122.015 h. Through a cycle's spectrum (1 % torque bands at their upper torque, ratio 5): 118.971 h. No figure without output torque, without the exponent, or never above M2N | 1e-6 relative | `the_gearing_life_is_the_hand_calculated_damage_sum`, analysis |
| Motor bearings | C = 1000 N · (25000 · 60 · 1500 / 10⁶)^(1/3) = 13103.7 N (the 1FK2205's figure); 600 N at 1000 r/min: 25000 h · (1000 / 600)³ · 1.5 = 173611 h; none behind a gearbox | 1e-5 relative | `the_motor_bearing_life_is_the_hand_calculated_one`, analysis |
| What the analysis adds up for the lives, on a move of every machine | The same move again, its signals summed by hand: mean motor speed, the speed-weighted cubic mean of the pulley shaft's radial load, the speed-weighted mean of the gearbox torque with the exponent 10/3, its peak per speed band and its load spectrum (speed per 1 % torque band), the guide's mean speed | 1e-9 relative | `the_life_loads_are_the_signals_added_up`, life |
| Life verdicts on the three shipped configs | Each value the function's; a required life 0.1 % above the pulley bearing's life fails it, 0.1 % below passes; no gearbox life on the EXCT-30 (no gearbox); the gearing's life where it has one (the delta's first arm), no motor bearing life anywhere; e_C 1.5 refused by name | exact | `the_life_verdicts_judge_the_lives_against_the_required_life`, life |
| Effect comparison: guide friction on against off, the mean over 0.1 s of a cruise at top speed | Motor torque difference r / i · (λ F_c + b v) / η (× η on a motor driven back); T-gantry r / (2 i) per motor with the force the Z guide's bristles hold added for A and taken off for B. Single axis 0.1525 against 0.1525 N m; T-gantry A 0.0749 against 0.0747 N m, B 0.0584 against 0.0583 N m (ripple and tooth meshing off) | 2 % | `comparing_friction_on_and_off_gives_the_friction_torque`, views |
| Effect comparison of a config with itself | Zero difference in every signal | exact | the same test |
| Undersized parts | The matching verdicts fail, and only those | Exact list | the same test |
| Band on the gearbox peak torque for a moved mass of 3, 4 and 6 kg | 0.65, 0.85, 1.25 N m by hand | 2 % | `a_bound_on_the_moved_mass_gives_the_hand_calculated_band_and_fails_the_verdict_it_reaches`, axis |
| Sampled bands: carriage 4 to 6 kg and payload 2 to 4 kg, a gearbox that allows 1.75 N m | Each mass alone at its bound: 1.65 N m, passes. Both sampled together, 40 runs: up to 2.05 N m, fails; 72 % of the runs pass by hand (70.7 % and 68.3 % with two seeds) | 2 %; 0.1 | `sampled_bands_find_the_two_masses_that_fail_only_together`, axis |
| Worst corner: loss-free rigid axis with payload, carriage 4 to 6 kg, payload 2 to 4 kg, gearbox efficiency 0.85 to 0.95, belt damping 50 to 200 N s/m (no effect) | `gearbox-peak` worst with both masses high: 0.2 (6 + 4 + 0.25) = 2.05 N m (found 2.070); `motor-peak` worst with both masses high and the efficiency low: (2.05 / 0.85 + 0.375) / 5 / 3.8 = 0.1467 (found 0.1481); every check at least as bad as in any of 40 samples, seeds 1 and 2 | 2 %; exact corner | `the_worst_corner_search_finds_the_hand_calculated_corner`, axis |
| The optimiser lowers the default T-gantry's lifts to a clearance of 0.1 m | Four jerk-limited Z moves of 0.2 m less: 4 · 0.2 / 0.4 = 2.000 s (t = d/v + v/a + a/j, 0.8767 s and 0.3767 s); "Apply" writes the cycle it ran; in the app a row per lift, EN and DE | 5 ms | `the_optimiser_lowers_the_lifts_to_the_clearance_and_saves_the_hand_calculated_time`, gantry; `the_app_shows_and_applies_lower_lifts`, app |
| The optimiser lifts each traverse of the T-gantry over a 0.1 m box across it, clearance 0.05 m | Top of the box + clearance + the lowest body below the tool: -0.6 + 0.05 + 0.802 = 0.252 m with the payload, -0.6 + 0.05 + 0.702 = 0.152 m without (flange); 0.1 m without the box, as before; the optimised cycle runs with the crash on without contact and passes every verdict | at or above, within 0.2 mm (the run sways 0.05 mm nearer than the plan at rest) | `the_optimiser_lifts_each_traverse_over_a_box_across_it_by_hand`, collision |
| The robust optimiser on the single axis with one bound, payload 2 to 20 kg | The search's fastest leaves the belt slack in the bands; one step back passes them; three settings judged at 2 · 1 + 1 cycles: 9 band runs; the nominal result fails in the bands, the robust one passes | exact count | `the_robust_optimiser_steps_back_to_the_fastest_setting_that_passes_in_the_bands`, axis |
| The optimiser judged at the worst corner on the single axis with two bounds, pretension 270 to 300 N and payload 2 to 18 kg | The bands pass the fastest (each bound alone keeps the belt taut), the corner of low pretension and heavy payload does not; at most 2 · 2 + 1 + 2² = 9 cycles per search: the fastest 5 + 3 (stops at the failing corner), the configured cycle 5 + 4, bisection steps 1 (passes) and 2 (fails) 5 + 4 each: 35; two steps back; the bands' result fails `verdict_worst`, this one passes | exact count | `the_optimiser_at_the_worst_corner_steps_back_to_a_setting_that_passes_there`, axis |
| A lift's height profile over two boxes of different heights, T-gantry, clearance 0.05 m | Via heights top + clearance + payload below the tool: -0.7 + 0.05 + 0.802 = 0.152 m and -0.6 + 0.05 + 0.802 = 0.252 m, two each, in order around the boxes; one box: no profile; the cycle through the profile (raised for the spline's sag) runs with the crash on without contact | 1 µm; clearance within 1 mm (sway) | `the_lift_passes_two_boxes_each_at_its_own_height`, collision |
| Braking energy of a 0.8 m move that does not reach its top speed, at 1 and 0.5 m/s² (loss-free rigid axis, DC link at 4.7 µF) | Peak of the jerk-limited triangle v² + (a²/j) v = a d: 0.88944 and 0.63121 m/s; ½ · 6.125 kg · v² per stop, 2.4228 and 1.2202 J; 2.405 J less per cycle fed back, and from the supply (the resistor burns all but 0.062 J of a stop) | 2 % (fed back), 3 % (supply) | `a_lower_acceleration_takes_the_hand_calculated_braking_energy_less_from_the_supply`, axis |
| The EXCT-30 config in the worst case of both band modes (67 bounded values one at a time; 60 samples) | Every check passes, `pulley-bearing-life` too since a_ISO is in it (above 40000 h in both modes; it failed before); slackest belt section above 100 N (158 N and 137 N), settling below 0.07 mm (0.044 and 0.050), shaft load below 1500 N (1200 and 1201); cycle 2.048 s | As stated | `the_exct_30_config_passes_every_check_in_the_worst_case_of_both_band_modes`, catalog |
| The EXCT-30 config at the worst corner of its bounds (`verdict_worst`; 3101 cycles on the command line on 21 November 2026, 3062 on 17 November, 2428 before the stage of modelled effects) | `settling` below 0.1 mm (0.087 mm, with the second shaper at 17.5 Hz; 0.115 mm without, not re-run since 17 November); every other check passes but `pulley-bearing-life` (9955 h on motor B there, with a_ISO's estimates at their bad bounds) | As stated | `the_exct_30_config_passes_settling_at_the_worst_corner_of_its_bounds`, catalog |
| A tool pose mapped into the 3D scene, at five poses of each machine, rigid and elastic | Where the scene draws the tool part (flange, carriage); by hand (0.45, −0.05, −0.592) m and (0.45, −0.03, 0.06) m | 1e-12 m | `the_scene_point_of_a_tool_pose_is_where_the_scene_draws_the_tool`, gantry; `..._the_carriage`, axis |
| Fit of a noisy simulated trace of a changed machine (1.5 kg heavier, friction × 1.4 and × 1.3, belt stiffness × 0.7) | The changes that were made | Within 3 standard errors plus what the rigid model mixes up on a changed machine, 5 % of the mass's change and 0.02 on the friction's factors (found + 1.570 kg, × 1.313, × 1.395); stiffness band below 0.03 | `the_fit_recovers_changed_masses_friction_and_belt_stiffness_from_noisy_measurements`, axis |
| Fit on the axis's own config with the test noise, 40 seeds (by hand: `--run-ignored only`) | (fitted − config) / error has mean 0 and standard deviation 1 for mass, viscous and Coulomb friction | Mean within 0.4, standard deviation 0.8 to 1.25 (found 1.01, 1.13, 1.15; seeds 41 to 80: 0.88, 0.94, 0.90) | `the_fit_s_errors_cover_its_noise`, axis |
| Fit on the T-gantry, where both motors move both axes | The changes per axis | Within 3 standard errors plus 20 % of the mass's change, 0.03 on the Coulomb factor and 3 % of the Coulomb friction on the constant force (found X + 2.32 kg, Z + 1.01 kg) | `the_fit_tells_the_two_axes_apart`, gantry |
| Resonance read off a synthetic dip | The frequency and damping it was built with, also with the points from high to low frequency | The sweep's spacing; 5 % | `resonance_reads_frequency_and_damping_off_a_known_dip`, identify |
| Of two synthetic dips, the deeper one higher | The lower one, 22 Hz, damping 0.05 | 0.5 Hz; 0.01 | `of_two_dips_the_lowest_in_frequency_is_reported`, identify |
| Fit in the motors' coordinates of a noisy simulated swing of the delta with all its effects: 4 kg of payload for 3, the joints' Coulomb and viscous friction doubled | Payload × 4 / 3; at the motor behind the gearbox of 20 and efficiency 0.96, Coulomb + 0.3 / 20 / 0.96 = 0.0156 N m, viscous + 0.05 / 400 / 0.96 = 1.30e-4 N m s/rad; no added inertia or constant torque; nothing the trace cannot tell apart | Within 3 standard errors; payload band below 2 % (found × 1.329 ± 0.014) | `the_delta_fit_finds_a_heavier_payload_and_more_joint_friction`, fit_joints |
| The same on the SCARA, gearbox losses off | Payload × 4 / 3; Coulomb + 0.5 / ratio (0.005, 0.00625, 0.01 N m), viscous + 0.2 / ratio² on joints 1, 2 and 4; the quill unchanged | Within 3 standard errors (found 0.00514, 0.00621, 0.00999 N m) | `the_scara_fit_finds_a_heavier_payload_and_more_joint_friction`, fit_joints |
| Links of the SCARA: arm 2 × 1.25, quill × 1.5 (mass and inertia), gearbox losses off | Arm 1's factor folded into motor 1's inertia with (0.1 + 8 · 0.16²) / 100² = 3.048e-5 kg m²; quill's and flange's into the payload's with 1.5 / 3 and 1 / 3; every value's change equal to the truth through what it combines | Coefficients to 1e-6 and 2 %; changes within 3 standard errors (payload + 0.445 ± 0.007 for 0.441) | `the_scara_fit_finds_heavier_links_in_the_values_they_combine_with`, fit_joints |
| Links of the six-axis arm, gears rigid: link 2 × 1.2, link 3 × 1.3; then the shoulder held | Link 1's factor into motor 1's inertia with 0.25 / 120² = 1.736e-5 kg m²; link 2's factor a value of its own, its change the truth through what it combines; folded with the shoulder held | Within 3 standard errors (+ 0.577 ± 0.013 for 0.565); error below 0.02 | `the_arm_fit_finds_heavier_links_and_folds_one_its_trace_does_not_excite`, fit_joints |
| Links of the six-axis arm as shipped, gears elastic: link 2 × 1.2, link 3 × 1.3 | As with rigid gears: link 1's factor into motor 1's inertia with 1.736e-5 kg m²; every value's change the truth through what it combines; no payload step | Within 3 standard errors (+ 0.558 ± 0.025 for 0.565); error below 0.04 | `the_arm_fit_with_elastic_gears_finds_heavier_links`, fit_joints |
| Payload picked at 2.5 s and dropped at 5.5 s during a swing of the delta | Factor 0, 1, 0; steps at 2.5 and 5.5 s | Steps within 3 of their errors, which stay below 20 ms (2.489 ± 0.019, 5.502 ± 0.002 s); factors within 3 standard errors, bands below 0.02 | `two_payload_steps_in_one_trace_are_found_with_when`, fit_joints |
| Payload picked at rest during moves of the single axis, fit per axis | One step in the rest before the third move; factor change + 1; mass before the step 2 kg below the config's | Within 3 standard errors (+ 1.022 ± 0.017; − 2.037 ± 0.024 kg) | `a_payload_picked_during_the_trace_is_found_on_the_axis`, axis |
| Payload picked at 3.5 s during a swing of the delta, all its effects | Factor 0 before, 1 after; the step at 3.5 s | Step within 5 ms, its error at most 5 ms (3.503 ± 0.001 s); factors within 3 standard errors, bands below 0.02 | `a_payload_picked_during_the_trace_is_found_with_when`, fit_joints |
| Belt stiffness from a noisy simulated move's trace of the single axis with its belt × 0.7, by replaying the config with the stiffness scaled | 0.7; what the fitted replay leaves of the ringing is the noise, 0.005 × sqrt(20 / 21) N m; a move of 2 mm at 1 m/s³ and 0.2 m/s² refused | Within 3 standard errors, band below 0.03 (found 0.708 ± 0.015); 5 % | `the_belt_stiffness_is_fitted_to_a_moves_trace_and_a_tiny_move_is_refused`, move_stiffness |
| The default sweep (5 to 500 Hz, 120 points, 1.5 N m or 40 % of the peak torque) on every motor of both default configs and `exct-30.toml` | A dip at 21.6, 20.8 and 84.3 Hz, the figures the simulation gave when the default was chosen | 3 %, damping above 0 | `the_default_sweep_finds_the_lowest_dip_on_every_config`, sweep |

| Energy budget of the cycle, 9 configs per machine (default, rigid, without DC link, with a shared one, every remaining effect on, the loss-free configs of the suite, and on the T-gantry the EXCT-30) | The supply equals copper loss, resistor, every loss of the machine and the change of every store; and on its own the electrical side: supply = copper + resistor + Δ windings + Δ capacitors + the shaft work integrated with the windings | 0.1 % of the supply on every config, loss-free ones included (measured at most 0.03 %, T-gantry loss-free elastic; 0.0005 % default T-gantry, `sim-cli analyze t-gantry --budget FILE`, 17 November 2026); the electrical side 1e-6 (measured at most 3.2e-7) | `the_energy_budget_closes_on_every_machine_and_config`, effects |
| Step-size convergence of the supply: the same cycles at 4 and at 8 integrator sub-steps | Each other | 0.5 % (measured at most 0.06 %); the single axis's loss-free elastic inclined config at 4 against 16 sub-steps (undamped, it knocks through its backlash, and the run at 8 alone takes another path: 20.925, 20.804, 20.924 J at 4, 8, 16); no config runs into its torque limit since the velocity gains were lowered | `the_supply_converges_with_the_integration_step_on_every_config`, effects |
| The four torque parts at every 5 ms of a cycle, same configs | Add up to the motor torque | 1e-9 relative | `the_torque_parts_add_up_to_the_motor_torque`, effects |

The guide life row checks that the code computes the catalogue formula. It does not check the formula against a guide. The same holds for the life rows: they check that the code computes ISO 281 (a1 and a_ISO), Siemens' gearbox formula, Palmgren–Miner on Siemens' gearbox figures and Megadyne's tooth load method; no bearing, gearbox or belt was run to failure, and the load ratings of the EXCT-30's pulley shafts are estimates.
| A slanted capsule's clearance to a box | A rod of radius 6 mm from (0, 0, 0) to (0.3, 0, 0.3) and a 0.1 m cube at (0.25, 0, 0.05): the cube's top edge through (0.2, y, 0.1) is (0.2 − 0.1) / √2 = 70.711 mm from the rod's line, so 64.711 mm; the cube at (0.15, 0, 0.1) holds the rod's point (0.125, 0, 0.125) 25 mm deep, an overlap of 31 mm | 1e-12 m | `a_slanted_capsule_keeps_the_hand_calculated_clearance_to_a_box`, sim-core |
| A box inside another | A point at (0.02, 0, 0.01) in a 0.1 m cube leaves by the face at x = 0.05, 30 mm deep; a 0.02 m cube there 40 mm | 1e-12 m | `a_box_inside_another_is_as_deep_as_its_way_out`, sim-core |
| Each drive monitor's trip, steps of 1/1024 s | Following error 1.01 rad beyond the 1 rad window at the first step; 0.2 rad beyond the 0.1 rad standstill window after 1 s (step 1024), never while moving; at the torque limit at standstill after 3 s (step 3072); torque 1.5 N m from the expected, beyond 0.3 · 3.8 N m, within 0.4 · 3.8 | exact | `each_monitor_trips_at_its_hand_counted_step`, fault |
| A move into a box, single axis, `effects.crash` and `drive_monitoring` | Contact at x = 0.47 at 1 m/s; the blocked motor's following error grows at 1 · 5 / 0.02 = 250 rad/s and crosses 1 rad 4 ms after the contact; it trips on it then (within one position cycle and the motor's last turn), stops, and rests at the face | trip 0 to 2 ms after the crossing; 3 mm in | `a_move_into_a_box_is_blocked_at_its_face_and_trips_on_the_following_error`, collision |
| A blocked axis at its current limit, single axis | F = T_max i / r = 3.8 · 5 / 0.02 = 950 N at rest against the box | 5 N (the motor's Coulomb friction, 0.02 N m) | `a_blocked_axis_pushes_with_the_hand_calculated_force_at_the_current_limit`, collision |
| The axis not in contact stops on its fault reaction, xyz portal on a slab | Z blocked trips 1 / (0.25 · 10 / 0.02) = 8 ms after the contact; X from v = 0.8 m/s on OFF3 (3140 rad/s² at the motor, 12.56 m/s² at the carriage) in v² / (2a) = 25.48 mm, on OFF1 (1.256 m/s²) 254.8 mm; STO without a brake coasts more than 3 times as far | trip within 2 ms; distance within 2 ms at v of v² / (2a) | `the_axis_not_in_contact_stops_on_its_fault_reaction_ramp`, collision |
| A D3 forearm rod's collision body | A 0.1 mm cube 20 mm from each rod's middle, square to it: clearance 20 − 6 = 14 mm, less the cube's half diagonal at most | 0.1 mm | `a_forearm_rod_is_no_wider_than_its_radius`, delta |

## Hand calculations: delta picker D3

The delta's geometry and masses are estimates (see [its config](../crates/machine-delta-d3/default.toml)), so these check the equations, not the machine.

| What is compared | Against | Tolerance | Test |
| --- | --- | --- | --- |
| Inverse and forward kinematics over a 13 × 13 × 9 grid of the working range (500 and more poses) | Each other | 1e-12 m | `the_kinematics_round_trip_and_the_jacobian_is_the_derivative_of_the_forward_kinematics`, delta |
| The Jacobian J = J_x⁻¹ J_q | Central differences of the forward kinematics, 1e-6 rad | 1e-7 m/rad | same |
| Holding torque at the centre, 100 mm below the top (forearms lumped, as in the next two) | The leg equation solved by halving, dz/dθ = −L_a cos θ + (e + L_a cos θ) L_a sin θ / (z + L_a sin θ) over 3 per arm: −g m_LG cos θ + m_nt g dz/dθ = −4.25 − 6.15 = −10.40 N m per gearbox, −0.520 N m per motor | 1e-4 | `the_holding_torque_at_the_centre_is_the_platforms_weight_shared_out_and_the_arms_own`, delta |
| Holding torque at the edge, x = 0.6 m | The weight through the transpose of a Jacobian from finite differences: −22.887 N m on the arm pointing there, −1.129 N m on the others (the forearms' weight a half at each end); rigid and elastic models stay put there | 1e-6; 1e-6 rad/s² | `the_holding_torque_at_the_edge_is_the_weight_through_the_jacobians_transpose`, delta |
| A vertical acceleration of 10 m/s² at the centre | θ̈ = a / (3 dz/dθ) = −24.98 rad/s², I_b θ̈ + m_nt a dz/dθ = −10.00 N m at the arm, −0.6053 N m at the motor; the rigid model given that torque accelerates the platform at 10 m/s² straight down | 1e-6; 1e-9 m/s² | `a_vertical_acceleration_at_the_centre_takes_the_hand_calculated_inertia_torque`, delta |
| The forearm rods (`effects.forearm_inertia`), arms level at the centre (z = −√0.6) | A vertical 10 m/s²: every rod translates, (I_arm + L_a² m_elbow) θ̈ − L_a m_fb a − (L_a/3) m_p a = −10.0 N m at the arm, −0.6053 N m at the motor, in both models; sideways at 1 m/s the rods turn: (m/6)(\|ċ\|² + ċ·ṗ + \|ṗ\|²) = 0.31302 J for the three pairs against 0.37604 J lumped | 1e-6 | `with_the_arms_horizontal_a_vertical_acceleration_takes_the_hand_calculated_torque_with_rods`, `the_rods_kinetic_energy_is_the_hand_calculated_one`, delta |
| The rods' spin about their own axis (`effects.forearm_inertia`) | Leg 1 with the platform at (0, 0.1, −0.8) moving 1 m/s square to its forearm in the plane y = 0.1 (arm 1 still): s = 0.1 / (L_b √(L_b² − 0.01)) = 0.111422 rad/s; every leg's spin from its pair's plane turning, by central differences; the spin's kinetic energy with 0.01 kg m² a rod, I Σ s_i² | 1e-9; 1e-6; 1e-6 | `the_rods_spin_is_the_hand_calculated_one`, delta |
| The arms' bending out of their planes (`effects.link_elasticity`), arms level at the centre | Every forearm square to its axis, so each elbow rings on k_l alone: lumped √(400000 / 0.63335) / 2π = 126.48 Hz three times, damped 15 / (2 √(k_l m)) = 0.014901; with the rods 147.35 Hz for the three elbows moving the same way round, the other two within 0.5 % above; 3 modes with the gearboxes alone, 6 with the bending, 9 with both, the lowest `resonance_hz` | 1e-6 | `the_arms_bending_out_of_their_planes_rings_at_the_hand_calculated_frequency`, delta |
| An upper arm running into an obstacle | A 20 mm box at the middle of arm 1 at the centre, 0.75 m down: more than 4 mm from the arm's capsule at the start pose, the plan finds the contact, the traverse with `effects.crash` and `drive_monitoring` trips the drives, the rods' capsules and the platform's box never touch it | exact | `an_upper_arm_running_into_an_obstacle_is_a_crash`, delta |
| A free, loss-free motion with rods, bending arms (in and out of their planes), both, on elastic and rigid joints, and with a rod spin inertia of 0.05 kg m² | Energy less the constant motor torques' work stays put over 0.1 s of RK4 at 10 µs | 1e-7 of the kinetic energy | `a_free_loss_free_motion_keeps_its_energy`, delta |
| The lowest mode with the motors held (`resonance_hz`), arms level at the centre, forearms lumped | The sideways mode, M_s = L_a² (m_elbow + s m_fb) + m_nt / Σθ̇² = 1.09572 kg m²: bending alone √(k_b / M_s) / 2π = 38.76 Hz, gearboxes alone √(k_g / (I_arm + M_s)) / 2π = 28.14 Hz, both the lower root of I_arm M_s ω⁴ − (k_g M_s + k_b M_s + k_b I_arm) ω² + k_g k_b = 0, 22.88 Hz; the mode's shape moves the platform sideways | 1e-6 | `the_lowest_mode_is_the_hand_calculated_one`, delta |
| A forearm running into an obstacle | A 30 mm box at the middle of arm 1's rod at the centre, 0.36 m above the platform: the plan finds the contact, the traverse with `effects.crash` and `drive_monitoring` trips the drives, the platform's boxes stay more than 0.3 m from it | exact | `a_forearm_running_into_an_obstacle_is_a_crash`, delta |
| The feedforward with the speed | The rigid loss-free model at (2, −1, 0.5) m/s accelerates the platform at the asked (10, 5, −20) m/s²; without the speed it misses by more than 1 m/s² | 1e-9 m/s² | `the_feedforward_with_the_speed_gives_the_asked_acceleration`, delta |
| The Adept cycle 25/305/25 mm with stops and 35 ms dwells | S-curve by hand: traverse d/v + v/a + a/j = 0.18167 s, 25 mm lift 4 (d / 2j)^(1/3) = 0.06840 s, pass 0.70693 s; on the 1 ms profile cycle 720 cycles. The vendors' 0.35 to 0.52 s (ABB IRB 360, rounded paths) is printed, not asserted | exact (1e-9 s) | `the_adept_cycle_takes_the_hand_calculated_time`, delta |
| Stability under constant torques | Second differences of gravity less torque times angle: −1.68 N m/rad in the vertical mode at 868 mm, +2.64 at 1000 mm | signs | `the_vertical_mode_under_constant_torques_is_unstable_high_up`, delta |
| Targets outside the working range, out of reach, beyond a stop, near a singularity | Refused with their reason | exact | `targets_outside_the_working_range_are_refused_with_their_reason`, delta |
| Condition number | 4 for a turned diag(1, 2, 4), 1 for 2 · 1; the cycle's peak judged against `ratings.max_condition_number` | 1e-12 | `the_condition_number_is_the_ratio_of_the_singular_values`, `the_cycle_is_judged_against_the_condition_number_limit`, delta |
| The setpoint slack at rest (`rest_sag`) | The simulation's setpoint-to-tool offset after 0.5 s at rest at five poses (0.75 to 2.09 mm with 9 arcmin): covered, and the slack (2.30 mm) at most 1.4 times the largest; 0.1 mm without backlash and with stiff gearboxes | exact | `the_setpoint_slack_covers_the_sag_at_rest`, delta |
| A first arc from rest; a path beyond the range | `arc,0.2525,0.0,-0.85,0.1525,0.0,-0.85,cw` from rest is laid out (refused before), an end 2 slacks off its circle is not; a line to x = 0.6014 is refused as the move there (both planners), one to 0.6 runs; a trajectory to 0.6014 is refused | exact | `a_first_arc_from_rest_is_laid_out_with_the_machines_slack`, `a_path_may_not_use_the_setpoint_slack_away_from_its_start`, delta |
| Rotary joint | Joint friction 2 + 0.5 · 4 = 4 N m, stop 1e5 · 0.01 = 1000 N m, rotor 2e-4 · 20² = 0.08 kg m², 1 N m less 0.1 N m friction through ratio 20 = 18 N m, losses 16 W and 8 W | 1e-9 | `a_joint_adds_friction_stops_and_the_rotor_as_calculated`, joint |
| Arcs on three axes | A quarter circle of 0.1 m in the x-z plane is π/20 m long, passes (0.0707, 0, 0.0707) about its centre halfway with its curvature 1/r towards the centre; clockwise the other way round; in the x-y plane seen from +z; a half turn laid in the x-y, else x-z, else y-z plane that holds its radius (refused where none does) | 1e-12 | `arcs_on_three_axes_lie_in_the_plane_of_their_points`, path |
| Splines on three axes | Through their via points, out of and into the lines beside them in their directions, the plan within every axis's speed, acceleration and jerk | 1e-9; 0.1 % | `a_spline_on_three_axes_passes_through_its_points_within_the_limits`, path |

## Hand calculations: planar delta picker D2

The planar delta's geometry and masses are estimates (see [its config](../crates/machine-delta-d2/default.toml)), so these check the equations, not the machine. All in `delta2`.

| What is compared | Against | Tolerance | Test |
| --- | --- | --- | --- |
| The standard suite, in the crate as well as in `suite` | As every machine | – | `the_planar_delta_passes_the_standard_suite` |
| Inverse and forward kinematics over the working range in steps of 10 mm (2500 and more poses); the Jacobian | Each other; central differences of the forward kinematics | 1e-12 m; 1e-7 m/rad | `the_kinematics_round_trip_and_the_jacobian_is_the_derivative_of_the_forward_kinematics` |
| Holding torque at the centre, 100 mm below the top | The leg equation solved by halving: θ = 1.4290°, dz/dθ per arm = −0.20503 m/rad, m_nt = 6.6 kg, m_LG = 0.520 kg m (the forearms' weight a half at each end): −5.098 − 13.270 = −18.368 N m at each gearbox | 1e-4 | `the_holding_torque_at_the_centre_is_the_plates_weight_shared_out_and_the_arms_own` |
| Holding torque at the edge, x = 0.4 m | The weight through the transpose of a Jacobian from finite differences: −25.73 N m on arm 1, −4.06 N m on arm 2 | 1e-6 | `the_holding_torque_at_the_edge_is_the_weight_through_the_jacobians_transpose` |
| A vertical acceleration of 10 m/s² at the centre | θ̈ = a / (2 dz/dθ) = −24.39 rad/s², I_b θ̈ + m_nt a dz/dθ = −17.80 N m at the arm, −0.9929 N m at the motor; the rigid model given it accelerates the plate at 10 m/s² straight up | 1e-6; 1e-9 m/s² | `a_vertical_acceleration_at_the_centre_takes_the_hand_calculated_inertia_torque` |
| The edge of the working range at a height | The bottom arc r = (0.4² + 0.227²) / (2 · 0.227) = 0.465923 m about z_c = −0.411077 m: at z = −0.8, x = ±0.256560 m; at z = −0.6 the half width 0.4 m | 1e-5 m | `the_edge_of_the_working_range_at_a_height_is_where_the_bottom_arc_cuts_it` |
| Targets outside the range, beyond a stop, out of reach, near a singularity | Refused with their reason | exact | `targets_outside_the_working_range_are_refused_with_their_reason` |
| The Adept cycle in the plane | As on the D3: 720 profile cycles, 0.720 s | 1e-9 s | `the_adept_cycle_takes_the_hand_calculated_time` |
| The default cycle's verdicts | All pass; the condition number's peak between the poses' 1.02 and 1.11 by hand and the lifts' tops (1.136) | – | `the_default_cycle_passes_its_verdicts` |
| Condition number | 4 for a turned diag(1, 4), 1 for 2 · 1 | 1e-12 | `the_condition_number_is_the_ratio_of_the_singular_values` |
| Stability under constant torques | Second differences of gravity less torque times angle: −3.778 N m/rad in the vertical mode at 638.5 mm, +7.428 at 788.5 mm; sideways 13.714 and 19.125 (the forearms' weight a half at each end) | 0.005 N m/rad | `the_vertical_mode_under_constant_torques_is_unstable_high_up` |
| The feedforward with the speed | The rigid loss-free model at (2, −1) m/s accelerates the plate at the asked (10, −20) m/s² | 1e-9 m/s² | `the_feedforward_with_the_speed_gives_the_asked_acceleration` |
| The setpoint slack at rest | 0.420 mm by hand (`rest_sag`'s grid, twist and half of 1 arcmin either way, plus a tenth), covering the simulation's 0.29 to 0.37 mm at six poses after 0.5 s | 2e-6 m; covered, within 1.4 times | `the_setpoint_slack_covers_the_sag_at_rest` |
| A path beyond the range; collision bodies | A line to x = 0.4014 refused, to 0.4 laid out; plate and payload boxes at (x, 0, z) and 70 mm below | exact; 1e-12 m | `a_path_may_not_use_the_setpoint_slack_away_from_its_start`, `the_collision_bodies_are_the_plate_and_the_payload_at_the_tool` |

## Hand calculations: linear delta

The linear delta's geometry and masses are estimates (see [its config](../crates/machine-delta-linear/default.toml)), so these check the equations, not the machine. It passes the standard suite as every machine does, also from its own tests.

| What is compared | Against | Tolerance | Test |
| --- | --- | --- | --- |
| Inverse and forward kinematics over a 20 × 20 × 5 grid of the working range (1000 and more poses) | Each other | 1e-12 m | `the_kinematics_round_trip_and_the_jacobian_is_the_derivative_of_the_forward_kinematics`, linear delta |
| The Jacobian J = J_x⁻¹ J_q | Central differences of the forward kinematics, 1e-6 m; its z row sums to 1, its x and y rows to 0 | 1e-7; 1e-12 | same |
| The belts' holding forces | g (m_c + m_nt dz/dh_k), m_c = 1.475 kg, m_nt = 2.325 kg: 22.0650 N per column at the centre (0.31606 N m at the motor), 32.784, 19.257 and 14.154 N at (0.3, 0.1, −1.17) with dz/dh from finite differences, summing to 3 m_c g + m_nt g = 66.1949 N; the rigid model's holding torques and the elastic one's T₁ − T₂; both stay put there | 1e-6; 1e-6 | `the_belts_hold_the_carriages_and_the_plates_weight_through_the_jacobian`, linear delta |
| The working range's edges at a height | At z = −1.17 a rod reaches 60° at x = e − L sin 60° = −0.502628 m; at z = −1.09 a carriage reaches its upper stop at x = e − sqrt(L² − 1.09²) = 0.302014 m; refused 1e-6 m beyond, not before; outside, out of reach, near a singularity refused with their reasons | 1e-6 m | `the_working_range_ends_where_a_rod_tilts_too_far_or_a_carriage_meets_its_stop`, linear delta |
| The feedforward | With the speed the rigid loss-free model accelerates the plate at the asked (10, 5, −20) m/s² at (2, −1, 0.5) m/s; at rest 10 m/s² up at the centre takes ((m_c + (J_pulley + belt + J_rotor) / r²) 10 + m_nt 10 / 3) r = 0.63355 N m per motor | 1e-9 m/s²; 1e-9 | `the_feedforward_gives_the_asked_acceleration`, linear delta |
| The default cycle | Every verdict passes; 720 profile cycles as the D3's Adept cycle by hand; condition number 3.28 | exact | `the_default_cycle_passes_its_verdicts`, linear delta |
| The setpoint slack at rest | The simulation's setpoint-to-tool offset after 0.5 s at rest at four poses (0.010 to 0.016 mm) within the slack (0.1 mm) | exact | `the_setpoint_slack_covers_the_sag_at_rest`, linear delta |

## Hand calculations: four-axis delta picker D4

The D4's arms are the D3's (checked above); these check what the fourth axis adds. Its shaft and tool values are estimates (see [its config](../crates/machine-delta-d4/default.toml)). All in `delta4`.

| What is compared | Against | Tolerance | Test |
| --- | --- | --- | --- |
| The standard suite, in the crate as well as in `suite` | As every machine | – | `the_d4_passes_the_standard_suite` |
| The shaft at (0.3, 0.4, −0.85) | d = (0.3, 0.4, −0.8): L = √0.89 = 0.943398 m, β = atan(0.5/0.8) = 32.0054°, bending plane at 53.1301°; over the range 0.625 to 1.0201 m and at most 43.8309°, so every pose the arms reach the shaft reaches (grid of 10 cm); beyond the stroke or the bend refused with the reason | 1e-6 relative | `the_shafts_length_and_cardan_angles_are_the_hand_calculated` |
| The cardan joints | One joint at β = 32.0054°, φ = 30°: tan χ = tan φ / cos β, χ = 34.2486°; the pair in phase gives φ back over a turn; the speed ratio cos β / (1 − sin²β cos²φ) is the derivative of χ; tool 10 rad/s and motor 80 rad/s at the pose, rigid and elastic, the shaft between at 10.3937 rad/s (between 8.480 and 11.792) | 1e-12 rad; 1e-7; 1e-5 | `the_tool_turns_with_the_motor_through_the_cardan_pair_and_the_shaft_between_unevenly` |
| The tool motor's torque for 100 rad/s² | J_c = 2e-3 + 8e-5 + 5e-3 = 7.08e-3 kg m²: 7.08e-3 · 100 / 8 + (3.5e-5 + 8.8e-6) · 8 · 100 = 0.12354 N m (0.06104 N m without the payload; motor 4's own rotor and gearbox, `[motor.4]`, `[gearbox.4]`); the rigid model given it turns the tool at 100 rad/s² | 1e-9 | `the_tool_motors_torque_for_a_turn_is_the_hand_calculated` |
| The arms' holding torques at the centre, 100 mm below the top | The D3's plus the shaft's 0.7 kg through dz/dθ = −0.13343 m/rad: 0.7 · 9.80665 · (−0.13343) / 20 = −0.04580 N m more at each motor; the D3's alone without `effects.shaft_mass`; the tool's motor holds nothing | 1e-6 | `the_arms_hold_the_d3s_torques_plus_the_shafts_weight_share` |
| The telescope's friction, the platform sinking at 1 m/s at (0.3, 0.4, −0.85) | L̇ = 0.8/0.943398 = 0.848 m/s: 5 N + 2 · 0.848 = 6.696 N along the shaft, 5.678 W | 1e-5 | `the_telescopes_friction_is_the_hand_calculated` |
| The tool's torsional mode | 1 / (1/8250.592 + 1/500) = 471.43 N m/rad against 7.08e-3 kg m²: 41.069 Hz (75.770 Hz without the payload), 2.05 % damping; among `modes` | 1e-5 | `the_tools_torsional_mode_is_the_hand_calculated` |
| The default cycle with a 90° turn | The tool at 90° in the pick to 1.5 mrad; every verdict passes (settling 0.39 mm); with the catalogue's 9 and 10 arcmin backlash settling fails alone (1.21 mm) | – | `the_default_cycle_turns_the_tool_and_passes_its_verdicts` |
| Motor 4's own tables | The tool's joint turns 3.5e-5 + 8.8e-6 = 4.38e-5 kg m² behind ratio 8 at 97 %; drive 4 gives 3.75 N m (the arms' 15); `ratings.gearbox_max_torque_nm` 29 for motor 4, 192 for motor 1 | 1e-12 | `the_tool_motors_own_tables_are_read` |
| Motor 4's values are the catalogue's | `[motor.4]`, `[gearbox.4]`, `[drive.4]`, `[ratings.4]` equal what `catalog::with_motor_part` puts in for 1FK2104-4AF, NRB060-04-8, 6SL3210-5HE10-4UF0 (backlash apart); its retuned velocity gain 0.25 · √(0.676413 · 0.207977) = 0.093768, above the shipped 0.05 (27.5°, 4.1 dB there) | 1e-6 (gain 1e-5) | `the_tool_motors_values_are_the_catalogues` |
| An arm's own gearbox table | `[gearbox.2]` ratio 10 against the shared 20: motor 2's angle for one pose is half, motors 1 and 3 unchanged; the tool motor and gearbox above the base are collision boxes at z 0.16 and 0.065 m (`the_motor_and_gearbox_above_the_base_are_collision_bodies`) | 1e-12 | `an_arms_own_gearbox_table_is_read_by_that_arm_alone` |

## Hand calculations: wafer handler

The R-θ-Z arm with banded links (`machine-wafer-handler`). L = 0.32 m, e = 0.18 m, the links' and effector's masses,
inertias and centres of [its config](../crates/machine-wafer-handler/default.toml) (estimates), the wafer 0.1276 kg
(SEMI M1), gearboxes 50 with rotor and input 2.86e-5 kg m². All in the crate's `tests.rs`.

| What is compared | Against | Tolerance | Test |
| --- | --- | --- | --- |
| The standard suite, rigid and elastic gearboxes and bands with and without the wafer | As every machine | – | `the_wafer_handler_passes_the_standard_suite` |
| The kinematics at φ = 60°, θ = 30° | Elbow (0, 0.32), wrist L u(90°) + L u(−30°) = (0.277128, 0.16) = 2 L cos φ u(θ) on the radial line, wafer centre (0.433013, 0.25), r = 0.5, the effector along θ; dr/dφ = −2 L sin φ = −0.554256 m/rad; condition 1 / sin φ = 1.154701; forward of inverse; motors 50 θ, 50 φ, 2π z / 0.01 | 1e-6; 1e-12 | `the_wrist_runs_on_the_radial_line_and_the_effector_keeps_its_direction` |
| Refusals | Beyond 2 L + e out of reach; φ beyond 10° and 110° and θ beyond ±200° at a stop; 1 / sin φ above 5 (r above 0.806926 m) near the full extension | – | `targets_beyond_the_arm_its_stops_or_near_the_full_extension_are_refused` |
| The R motor's inertia, retracted and extended | M_φφ = m₁ c₁² + I₁ + m₂ (L² + c₂² − 2 L c₂ cos 2φ) + I₂ + (m₃ + m_w) 4 L² sin² φ: 0.5499656 kg m² at r = 0.12 (1.354595 kg along r, 2.485862e-4 kg m² at the motor), 0.1876677 at r = 0.75 (2.215676 kg, 1.036671e-4); M_θφ = 0.13312 at every pose; the feedforward for r̈ = 1 m/s² | 1e-6 (the rounded figures 1e-5); 1e-9 | `the_r_motor_feels_the_hand_calculated_inertia_retracted_and_extended` |
| The θ motor's inertia at two reaches | J_c + Σ (I + m |centre|²): 0.153033 kg m² at r = 0.12 (8.981318e-5 at the motor), 0.604751 at r = 0.75 (2.705002e-4), growing with r²; the feedforward for θ̈ = 1 rad/s² | 1e-5; 1e-9 | `the_theta_motor_feels_an_inertia_growing_with_the_reach` |
| The wafer pushed out in a turn | At r = 0.75 turning at √(μ g / r) = 1.980571 rad/s (μ 0.3) the wafer accelerates at μ g = 2.941995 m/s², 1.2 times the limit μ g / 1.2 = 2.451663; motor 2 holds the arm against −½ ∂M_θθ/∂φ θ̇² = 0.814776 N m (∂M_θθ/∂φ = −0.415420), 0.0162955 N m at the motor, and the rigid model under that feedforward does not accelerate | 1e-9; 1e-5 | `a_fast_turn_pushes_the_wafer_outwards_and_it_slips_at_the_hand_calculated_rate` |
| Coriolis, extending during a turn | r = 0.5, ṙ = 0.5 m/s, θ̇ = 1 rad/s, no acceleration: the wafer at (r̈ − r θ̇², 2 ṙ θ̇ + r θ̈) = (−0.5, 1.0) m/s², 1.118034 | 1e-9; 1e-6 | `extending_during_a_turn_gives_the_wafer_the_coriolis_acceleration` |
| The bands under 1 N m on the effector | K₁ = 2 k r₁² = 1692, K₂ = 2303 N m/rad; β = T / 2K₁ = 2.955083e-4, γ = T / 4K₁ + T / K₂ = 5.819704e-4 rad; the bands push back with exactly −T on γ and nothing on β; swings −T / (4 r₁ T₀) = −0.0555556 and −T / (2 r₂ T₀) = −0.0952381; at r = 0.75 the wafer centre at (0.750043, 1.889809e-4) and the effector 3.300103e-4 rad out of its radial line (the signal, and at a station only while dwelling) | 1e-6 | `the_bands_stretch_under_a_torque_and_turn_the_effector_out_of_its_line` |
| A span runs slack | 1e-4 m: 188 N difference, k s² = 9.4e-3 J; 2e-4 m: one span slack, 338 N, 0.0488 − T₀² / 2k = 0.0368319 J | 1e-12; 1e-6 | `a_band_span_runs_slack_beyond_its_pretension` |
| The rigid bands' forces | Wrist over the column, θ̈ = 10 rad/s²: M_γθ = m₃ c₃² + I₃ + m_w e² + I_w = 0.01532974, M_βθ = m₂ (c₂² − L c₂) + I₂ = −0.02048 kg m²; ΔT₂ = 4.379926 N, ΔT₁ = −4.271710 N, swings −0.0142390 and 0.0145998 of the pretension; the plan's rows (`rigid_checks`) and the signals give them | 1e-5; 1e-9 | `the_rigid_bands_carry_the_hand_calculated_forces` |
| The Z holding torque | 11.2276 kg · 9.80665 · 0.01 / 2π = 0.1752378 N m (0.1732462 without the wafer); the arm's motors hold nothing; nothing moves, rigid or elastic, friction on | 1e-9; rates 1e-7 | `the_z_motor_holds_what_it_lifts_and_the_arm_motors_nothing` |
| The modes, motors held | numpy's eigenvalues of the same K and M (scratch script): 18.1952, 51.3514, 70.6454, 186.4465 Hz at r = 0.12, 16.7646, 42.3221, 83.6696, 175.3323 Hz at r = 0.75; the bands alone 21.0252, 62.4203 and 18.5831, 83.1553 Hz | 1e-5 | `the_modes_are_those_of_the_gearboxes_and_bands` |
| The hold | Pads μ g = 2.941995 m/s², lowering at 1 m/s² 2 · 9.80665 / 8.80665 = 2.227100 asked for 2; vacuum 18 N / 0.1276 kg: μ (g + 141.065831) = 45.261744; the edge 9.81 whatever the lift | 1e-6 | `the_hold_asks_the_hand_calculated_share_on_pads_vacuum_and_edge` |
| The default cycle | Every verdict passes (`wafer-slip` 1.83 of 2.45 m/s², settling 0.048 mm and 0.113 mrad, `effector-direction` 12.7 µrad, the bands' swings 6.5 % and 1.2 %); the reach speeding up at 3 m/s² fails `wafer-slip` | – | `the_default_cycle_passes_its_verdicts_and_a_faster_reach_slips_the_wafer` |

## Hand calculations: arms in tool space

The SCARA and the six-axis arm with `kinematics.tool_space` on ([mechanisms](mechanisms.md#tool-space)), on the configs `config/scara-tool.toml` and `config/arm-tool.toml`. All in `machine-joint-kit`.

| What is compared | Against | Tolerance | Test |
| --- | --- | --- | --- |
| The SCARA's closed form at (0.325, 0.275, 0.1, 0.3) | cos β = (0.18125 − 0.325² − 0.275²) / (2 · 0.325 · 0.275) = 0: right elbow β = π/2, joint 1 = 0; left β = −π/2, joint 1 = 2 atan2(0.275, 0.325) = 1.404514; joint 3 = 0.2 − z = 0.1 m, joint 4 = c − q1 − q2; the URDF chain puts the flange there | 1e-12 | `the_scara_closed_form_gives_the_joints_of_a_pose_by_hand` |
| Forward of inverse | The pose itself, over the SCARA's tool range (the standard suite's grid) and at 40 poses around the arm's home (turns up to 0.3 rad), the arm's wrist on the home's side (joint 5 > 0); both hold still at rest | 1e-9 | `inverse_then_forward_returns_sampled_tool_poses_within_1e_9` |
| A line of the tool | Every planned pose (within the axis limits and time-optimal) on the SCARA's line x = 0.4, z = 0.15, c = 3 (y + 0.25), and the flange where the inverse's joints put it; the arm's line along y with its orientation (0, π/2, 0) held, read back through the URDF chain | 1 µm, 1e-6 rad | `every_planned_pose_of_a_straight_tool_line_lies_within_1_um_of_it` |
| The joint speeds at (0.325, 0.275) along y at 0.5 m/s | J = [[−0.275, −0.275], [0.325, 0]]: joint 1 0.5 / 0.325 = 1.538462 rad/s, joint 2 the opposite, joint 4 0; motors × 100, 80; condition number from the eigenvalues of Jᵀ J = [[0.18125, 0.075625], [0.075625, 0.075625]], 2.4691 | 1e-4 rad/s, 1e-9 | `the_joint_speeds_along_a_tool_line_are_the_inverse_jacobian_times_the_tool_speed` |
| The stretched SCARA arm | At x = 0.598 the elbow acos((0.598² − L1² − L2²) / (2 L1 L2)) = 0.1639 rad, condition above 25: the move refused with `error-arm-singularity`; at 0.59 (0.3669 rad) it goes; at 0.61 `error-arm-out-of-reach`; a line ending at 0.6 refused with `error-path-range`, within the axis limits and time-optimal | – | `a_tool_path_into_the_stretched_scara_arm_is_refused_with_its_reason` |
| The time-optimal plan of a line of the tool | No motor faster than its rated 314.159 rad/s (from differences of the planned poses' joints); the fastest above 0.9 of it and faster than within the axis limits, the plan shorter | 0.1 % | `the_time_optimal_plan_keeps_the_scara_motors_within_their_rated_speed_along_a_tool_line` |
| The tool-space configs | The defaults but for `kinematics.tool_space`, `start.pose`, the cycle and the arm's settling tolerance; both cycles of straight lines pass every verdict (SCARA 2.542 s, settling 0.03 mm; arm 3.858 s, settling 2.3 of 5 mm) | – | `the_tool_configs_are_the_defaults_but_for_tool_space_start_cycle_and_settling`, `the_tool_space_cycles_pass_their_verdicts` |
| A link that twists and bends (`link_bending.directions` all three): a 60 × 4 mm steel tube, 0.8 m, 4.5 kg, with 10 kg and 0.5 kg m² about its axis at its tip | Twist sqrt(G I_p / (L (J + J_own / 3))) / 2π = sqrt(56146.0 / 0.501182) / 2π = 53.2699 Hz (the tube's share J_own / 3 = 1.182e-3 kg m²; the exact root of β tan β = J_own / J 53.26988 Hz); both bends sqrt(3.0906 E I / L³ / (m / 4 + M)) / 2π = 28.2887 Hz | 1e-5 | `a_tube_twists_at_the_frequency_of_its_tip_inertia_and_bends_alike_in_both_planes` |
| Two-way bending (`effects.link_bending_reaction`): the tube of 0.45 kg with 20 kg at its tip on a gearbox of 2e5 N m/rad, the motor held | The two-coordinate mode det(diag(k_g, k_b) − ω² M) = 0, M_qq = 12.897 kg m², B = 16.07219 kg m, M_d = 20.11546 kg: 14.43363 Hz, the lowest in `modes` (the series spring on the modal mass 14.43276 Hz); static sag L τ / k_g + Q_g / k_b = 1.197642 mm (M g / k_s = 1.185666 mm with the tip mass alone); let go from straight it rings about the sag with that amplitude at that frequency | 1e-5 (modes), 1e-4 (sag), 0.2 % (ringing), 1 % (amplitude, middle) | `a_bending_link_on_an_elastic_joint_rings_as_the_two_springs_in_series` |
| Two-way bending, energy at an instant | The six-axis arm under gravity, loss-free, no motor torque, all three directions, links bent by 1 mm (1 mrad) and moving, joints turning: dE/dt along the state's own rates by central differences is zero (1e-5 W left; without G 0.22 W, without ½ Ṁ_d ḋ 0.012 W), rigid and elastic | 1e-4 W | `two_way_bending_trades_energy_without_loss_at_every_instant` |
| Two-way bending, energy over a run | The standard suite's energy conservation (0.1 % of the pulse's energy over 10 s): the tube hanging under gravity with all three directions, elastic and rigid; the six-axis arm's loss-free elastic config with the payload, gravity off, its links bending in their joints' planes | 0.1 % | `two_way_bending_keeps_the_energy_of_a_hanging_link`, `two_way_bending_keeps_the_six_axis_arms_energy` |
| Two-way bending at rest and with losses | The shipped six-axis arm, all three directions: the standard suite's static hold (1e-7 m for 1 s, the gears holding the bent links' weight shift) and loss budget (the links' damping as `power.link_bending`) | suite's | `two_way_bending_holds_still_and_balances_the_losses` |

## Hand calculations: tripod

The 3-RPS tilting table ([mechanisms](mechanisms.md#tripod)), its default config: rb 0.3 m, rp 0.2 m, legs 0.45 m
retracted, 62.7 kg moving (20 kg platform, 3 x 0.9 kg of legs, 40 kg payload at (0.05, 0, 0.1) m). All in
`machine-tripod`.

| What is compared | Against | Tolerance | Test |
| --- | --- | --- | --- |
| The standard suite | As every machine; its loss-free configs without gravity, as the hexapod's | – | `the_tripod_passes_the_standard_suite` |
| Leg lengths | Level at z = 0.54: √(0.54² + 0.1²) = 0.5491812 m, 0.0991812 m out; tilted 0.15 rad about x (p_x = rp (1 − cos 0.15) / 2 = 0.0011229 m): 0.5489779, 0.5750463, 0.5241851 m | 1e-7 m | `the_legs_have_the_hand_calculated_lengths` |
| The parasitic motion at (0.15, 0.1) rad | Closed form g = −0.0075203 rad, p_x = 0.00061207 m, p_y = −0.00149654 m; second order rp (rx² − ry²)/4, −rp rx ry / 2, −rx ry / 2; each platform joint in its leg's plane; the normal the pure tilt's; tilted about x alone the pose error is p_x | 1e-8 m, 1e-7 rad (second order 2e-5 m, 1e-4 rad) | `the_parasitic_motion_is_the_hand_calculated_one` |
| Forward of inverse | 9³ poses over the working range; legs no pose fits (one 1 m out) refused | 1e-12 | `the_forward_kinematics_undoes_the_inverse_over_the_working_range` |
| Convergence of the forward kinematics | The chord step's contraction over the box grown by r = 0.04 m (weighed): at most 0.595 on 17³ points, changing by under 0.07 between neighbours, so below 0.7 | < 0.7 | `the_chord_iteration_contracts_over_the_grown_working_range` |
| The Jacobian, level | Rows u_z (1, rp sin t_i, −rp cos t_i), u_z = 0.9832820: (0.983282, 0, −0.196656), (0.983282, ±0.170309, 0.098328); condition number √2 (u_z² diag(3, 3/2, 3/2)); below 1.7 at the box's corners | 1e-6; 1e-9 | `the_jacobian_at_the_level_pose_is_the_geometrys` |
| Holding a centred load, level | m g / (3 u_z) = 62.7 · 9.80665 / (3 · 0.983282) = 208.4437 N per leg; 0.331749 N m per motor (f lead / 2π) | 1e-3 N, 1e-5 N m | `the_legs_hold_a_centred_load_with_the_hand_calculated_forces` |
| Holding the offset load, level | Moment balance: centre of mass 0.031898 m out, V_i = m g (1/3 + 2 c cos t_i / (3 rp)), f_i = V_i / u_z: 274.9330 N (leg 1), 175.1991 N (legs 2, 3) | 1e-3 N | `the_legs_hold_an_offset_load_with_the_hand_calculated_forces` |
| A leg's force rising at 2 m/s² | m (g + a) / (3 u_z) = 250.9544 N with q'' = u_z a, from M_q q'' + h | 1e-3 N | `a_leg_pushes_the_hand_calculated_force_for_an_acceleration_along_z` |
| Momentum on a compliant base, rising at 0.1 m/s | m v = (0, 0, 6.27) N s, s × v = (0, −0.2, 0) N m s from the payload's offset, first moment (2.0, 0, 37.858) kg m | 1e-5 | `the_platform_has_the_hand_calculated_momentum` |
| The new verdicts at their limits | Level: `tripod-leg-force` the offset load's forces against 1500 N, `tripod-leg-stroke` −0.0008188 m against 0.1 m, `tripod-revolute-angle` −10.4915° and `tripod-cone-angle` 10.4915° against 30° and 35°, `tripod-parasitic` 0 against 5 mm; the base joints limited to 10.4° refuse the level pose, 10.5° take it; the cone at 10.4° refuses it; no tolerance, no parasitic check | 1e-7 (forces 1e-3 N) | `the_new_verdicts_have_the_hand_calculated_values_and_limits` |
| The default cycle | Every verdict passes: legs to 382 N, swing 12.0°, cone 21.6°, pose error 2.87 of 5 mm, settling 3.1 µm of 50 µm | – | `the_default_cycle_passes_its_verdicts` |

## Hand calculations: arm on a track

The six-axis arm on its track (`arm-on-track`, [mechanisms](mechanisms.md#an-arm-on-a-track), [model reference](model-reference.md#arm-on-a-track)), rigid and loss-free, the arm stretched across the track (joint 2 at π/2, the others at 0). From `urdf/arm-on-track.urdf`: Σ m y = 12 · 0.15 + 7 · 0.23 + 4 · 0.60 + 2 · 0.83 + 1.5 · 0.89 + 6 · 0.95 = 14.505 kg m (the payload 0.05 m beyond the flange), Σ m z = 80 · 0.1 + 18 · 0.70 + 12 · 1.15 + 7 · 1.53 + 13.5 · 1.57 = 66.305 kg m above the blocks' plane, 130.5 kg in all. All in `machine-joint-kit`; the track's ratio 10 / (0.002 · 30 / (2 cos 19.5283°)) motor radians per metre.

| What is compared | Against | Tolerance | Test |
| --- | --- | --- | --- |
| The arm's swing on the track | Joint 2 at 3 rad/s²: each mass moves along the track by −3 y, the track pushes −3 · 14.505 = −43.515 N; at rest nothing | 1e-6 | `the_track_and_the_arm_act_on_each_other_as_by_hand` |
| The track's acceleration on the arm | Track at 2 m/s²: joint 2 holds −2 · 14.505 = −29.01 N m (motor 2 times 120), the track pushes 130.5 · 2 = 261 N (with the rotor, gearbox and pinion of motor 1 apart) | 1e-6 | as above |
| The guide blocks at rest | Four blocks 0.4 m by 0.5 m: 4 (130.5 g / 4 + g · 14.505 / (2 · 0.5)) = 4 (319.942 + 142.2455) = 1848.75 N | 1e-7 | `the_guide_blocks_carry_the_weight_and_the_tilting_moments_by_hand` |
| The guide blocks with the track at 2 m/s² | Pitch M_y = 2 · 66.305 = 132.61 N m over 2 · 0.4 m, yaw M_z = 2 · 14.505 = 29.01 N m over 2 · 0.4 m on top: 4 (462.188 + 165.7625 + 36.2625) = 2656.85 N; the rack's teeth 261 N, the pinion's shaft 261 · √(1 + (tan 20° / cos 19.5283°)²) = 261 · 1.071978 N | 1e-6 | as above |
| The guide verdicts | `guide-load` fails with the rating at 0.999 of the cycle's peak and passes at 1.001; `guide-life` fails with the required life at 1.001 of what the blocks give | – | `the_guide_verdicts_judge_the_track_at_their_limits` |
| Tool space with the track | The track exactly where the pose says (1e-12 m) and forward of inverse the pose (1e-9) at 0.3, 1.5, 2.8 and 3.7 m; the flange held while the track moves 0.2 m turns the arm's base; the machine holds still in tool space | 1e-12, 1e-9 | `the_track_is_a_given_axis_beside_the_tool_pose` |
| The default cycles | Both pass every verdict: in joints and in tool space | – | `every_default_cycle_passes_its_verdicts`, `the_tool_space_cycles_pass_their_verdicts` |

Taken out by hand on 15 November 2026: with the moments dropped from the block loads the guide test fails (1279.77 N at rest against 1848.75 N).

## Hand calculations: hexapod with rotary actuators

The six-axis platform on cranks (`hexapod-rotary`, [mechanisms](mechanisms.md#hexapod-with-rotary-actuators-6-rss)),
on its default config; leg 1 at the level pose (0, 0, 0.4 m): B at 0.5 m and −5°, P at 0.3 m and −55°, d = (−0.573576,
−0.819152, 0). All in `machine-hexapod-rotary`; the numbers also by an independent numpy script of the same formulas.

| What is compared | Against | Tolerance | Test |
| --- | --- | --- | --- |
| The crank angle at the level pose | e = P − B = (−0.326024, −0.202168, 0.4): a = 0.352606, b = 0.4, c = 0.111659, √(a² + b²) = 0.533227; θ = 0.848288 − 1.359832 = −0.5115438 rad for all six ("in": 2.208120 rad); the rod 0.55 m long | 1e-7 rad | `every_crank_has_the_hand_calculated_angle_at_the_level_pose` |
| Forward of inverse | Three poses (level, shifted and tilted, a far corner with all three turns); a rod that cannot reach refused | 1e-12 | `the_forward_kinematics_undoes_the_inverse_at_three_poses` |
| Holding torque, level, centred load | f = (115 + 6 · 0.2) g / (6 u_z) = 209.7935 N, u_z = 0.905281; τ = f m + cos θ g (m_c r_cg + m_rod r / 2) = 39.7772 + 1.7103 = 41.4875 N m, 0.829750 N m at the motor (ratio 50) | 1e-3 N, 1e-5 N m | `the_cranks_hold_the_level_platform_with_the_hand_calculated_torque` |
| The torque arm at the dead centre | m = r √(a² + b²) sin δ / L: 0.185772 m at z = 0.5 m; zero at z = 0.6378641 m, the root of (w + K)² = 4 r² (a² + w), K = −0.115336; 2 mm lower the pose is refused (`error-rss-dead-centre`) | 1e-6 m, 1e-5 m | `the_torque_arm_goes_to_zero_at_the_dead_centre` |
| The inverse Jacobian at the level pose | Row 1 (u, b × u) / m = (−2.167162, −0.568745, 4.774645, −1.173348, −0.821587, −0.630436); the closed form's slope along z 4.774645 rad/m | 2e-6 | `the_jacobian_at_the_level_pose_is_the_hand_calculated_one` |
| A heave of 1 m/s² from rest | θ'' = 4.774645 rad/s²; the rods' compression + 21.3807 N; at the crank 0.095493 + 0.037088 + 4.053815 = 4.186396 N m; at the motor / 50 plus 1.1e-4 · 50 · θ'' = 0.109989 N m | 2e-5 N m | `a_heave_acceleration_takes_the_hand_calculated_torque` |
| A rod's Euler load | π² E I / L², I = π (0.02⁴ − 0.016⁴) / 64 = 4.636991e-9 m⁴, E 70 GPa, L 0.55 m: 10590.31 N, over 3: 3530.10 N | 0.01 N | `the_rods_buckle_at_the_hand_calculated_euler_load` |
| The new verdicts | At the level pose the rod 18.5567° from the crank's tangent, 15.9463° out of the crank's plane, 25.1388° from the platform's normal; `crank-dead-centre`, `rod-buckling`, `crank-ball-joint`, `platform-ball-joint` pass at their limits (70°, 3530.10 N, 35°, 40°) and fail 1e-9 beyond | 1e-4° | `the_new_verdicts_pass_at_their_limits_and_fail_beyond` |
| Virtual work against Newton-Euler | Off centre, moving and accelerating: each crank's M θ'' + h equals J_c θ'' + its weight + the rod's lower node t · m (a − g) + f_i m_i with the rods' forces from the platform's side | 1e-9 | `the_virtual_work_agrees_with_the_forces_along_the_rods` |

## Hand calculations: picking from a conveyor

| What is compared | Against | Tolerance | Test |
| --- | --- | --- | --- |
| Intercept time of a trapezoidal move (v_max 2 m/s, a 20 m/s², jerk 1e9) from rest at 0 to a part at 1 m moving at 0.5 m/s | The root of the quadratic of T = D / v′ + v′ / a′, D = 1 + v T / 2, v′ = 1.5 m/s, a′ = a − 1.5 v / T: 0.89395 s | 1e-5 s | `the_intercept_of_a_trapezoidal_move_is_the_closed_form`, conveyor (unit) |
| Arrival on the part, speed and limits on the way | The part's pose and speed, the axis limits | 1e-9 | `the_tool_arrives_on_the_part_with_the_belts_speed_within_its_limits`, conveyor (unit) |
| Plan during the grip; when it begins | The part's pose and the belt's speed; the part at the window's start, 0.5 m at 0.5 m/s: 1 s | 1e-9; 1e-6 s | `during_the_grip_the_plan_moves_with_the_part`, conveyor (unit) |
| Parts beyond the window or out of reach | Parts at 0, 2 and 4 s are 2.5, 1.5 and 0.5 m on at 5 s, the window ending at 1 m: two missed; a place out of reach: all missed | exact | `a_part_beyond_the_window_counts_as_missed`, `a_part_out_of_reach_counts_as_missed`, conveyor (unit) |
| Picks a minute of the D3 example | 60 / cycle time; the passes a pitch apart, 1 s: 60 a minute, none missed | 1e-12; 1 ms | `the_example_picks_a_part_each_pitch_and_misses_none`, conveyor |
| The D3's speed along the belt during the grip, measured | 0.3 m/s ± tolerance / grip time = 0.0286 m/s; the plan exactly 0.3 m/s | as said | `during_the_grip_the_tool_moves_along_the_belt_at_its_speed`, conveyor |
| Parts missed a minute at a part every 0.5 s | At least 120 − picks a minute | – | `a_belt_faster_than_the_picks_misses_parts`, conveyor |
| RMS torque of a belt pick at 0.3 m/s against a stationary pick, single axis with inertia alone | sqrt of the ratio of ∫a² dt of the bang-bang moves with the reduced acceleration plus 1.2 v² / T per blend: 0.8876 (measured 0.8877) | 0.02 | `a_belt_pick_changes_the_rms_torque_by_the_hand_estimated_share`, conveyor |
| Belt, slats and parts in the scene; an obstacle at the place | 1 band, 10 slats, part 0 at 0.15 m at 0.5 s; `collision` fails | 1e-9 m | `the_scene_draws_the_belt_its_slats_and_its_parts`, `an_obstacle_at_the_place_fails_the_collision_check`, conveyor |

## Hand calculations: pneumatic cylinder

The cylinder of `sim_components::pneumatic` (unit tests there, a DSBC-32-like cylinder: A1 = 8.0425e-4 m², A2 =
6.9115e-4 m², 0.5 kg, 6 bar gauge) and the pneumatic Z of `machines/pneumatic-axis.toml` (tests in
`sim-machines/tests/pneumatic.rs`).

| What is compared | Against | Tolerance | Test |
| --- | --- | --- | --- |
| Theoretical force at 6 bar, extending and retracting | p_g A1 = 482.55 N, p_g A2 = 414.69 N (Festo prints 483 and 415 N; the catalogue check holds every DSBC size to p A) | 1e-5 | `the_theoretical_force_is_p_a_on_both_sides` |
| Choked and subsonic mass flow through C = 1 l/(s bar), b = 0.3; two elements in series | C p ρ0 = 8.3107e-3 kg/s from 7.013 bar; ψ(0.65) = 0.86603; C_s = 2^(−1/3) C, b_s = 1 − C_s²/C² (1.4) | 1e-5 | `the_flow_through_a_conductance_is_choked_below_b_and_elliptic_above` |
| Steady speed of a meter-out throttled cylinder, C = 0.1 l/(s bar), 50 N load | v = C ρ0 R T0 / A2 = 0.14428 m/s, whatever p2 while choked (p2 = 7.27 bar) | 1 % | `a_meter_out_throttled_cylinder_runs_at_the_choked_flow_s_speed` |
| The same in the machine file: Z extending downwards with its seals and the payload | C_s = 0.14943 l/(s bar) (throttle, valve, tube in series): 0.2156 m/s | 3 % | `the_servo_x_and_the_pneumatic_z_run_their_cycle` |
| Natural frequency on the two air springs at mid-stroke, closed centre | sqrt(n (p1 A1²/V1 + p2 A2²/V2) / m) / 2π: adiabatic (n = 1.4) 29.64 Hz, isothermal limit (500 W/K, n = 1) 25.05 Hz | 2 % | `the_air_springs_ring_at_the_hand_frequency_adiabatic_and_isothermal` |
| Air for a double stroke, long dwells | Σ (p_s (V0 + A s) − p_atm V0) / (R T) = 1.31757e-3 kg, 1.1119 normal litres (ANR) | 0.5 % | `a_double_stroke_draws_the_hand_air_consumption` |
| Filling a closed 49 cm³ chamber, adiabatic, while choked | t = (b p_s − p_atm) V / (κ R T ṁ) = 2.555 ms (C 0.9615 l/(s bar), b 0.2142) | 0.5 % | `filling_a_closed_volume_takes_the_closed_form_time_while_choked` |
| Energy of a closed cylinder (both chambers shut, adiabatic, no friction, stops undamped), vertical, 1 s | U1 + U2 + kinetic + gravity + the atmosphere's work on the rod + the stops' springs: constant | 1e-9 of U | `a_closed_cylinder_keeps_its_energy` |
| Balance of the open cylinder over an extend and a retract stroke with seals, cushions and heat transfer | Δ(energy) = enthalpy from the supply − to the exhaust + heat from the wall − seal and damper work, each integrated as a state | 1e-6 of the enthalpy drawn | `the_enthalpy_flows_balance_the_energy_of_an_open_cylinder` |
| Step convergence: stroke time at the sub-step 15.6 µs and half of it; the machine file's stroke times at 4 and 8 sub-steps | the same | 1 % | `the_stroke_time_converges_with_the_step`, `halving_the_step_leaves_the_stroke_time` |
| Checks of the shipped cycle | Impact energy ½ · 1.0 kg · 0.2156² = 0.023 J (within 30 %: the piston still swings about its steady speed) against 0.4 J; load factor 9.807 N / 414.7 N = 0.0236 against 0.7; a rating of 0.01 J fails | as said | `the_cylinder_s_checks_are_judged` |
| A cylinder too stiff for the step | refused with `error-cylinder-unstable` (1e-9 m³ dead volume) | – | `a_cylinder_too_stiff_for_the_step_is_refused`, `..._in_a_file` |

## Hand calculations: winder with dancer

The winder of `machine-winder` (tests in its `src/tests.rs`): PET film 50 µm × 0.4 m, 1390 kg/m³, EA = 80 kN; rolls
from r_c = 45 mm to r_f = 150 mm, core and shaft 0.01 kg m²; gearbox i = 7, rotor and gearbox input 8.8e-4 kg m²; spans
of 1 m; dancer 4 kg plus 2 kg, roller R = 40 mm, J_d = 2e-3 kg m², air spring 1000 N/m; tension 100 N at the core,
taper 0.3.

| What is compared | Against | Tolerance | Test |
| --- | --- | --- | --- |
| Radius after a wound length of 640 m; the angles; the roll's capacity | r² = r_c² + h L / π: rewind 0.1105030 m, unwind 0.1109688 m; φ = 2 L / (r_0 + r): 8231.352 and 4904.800 rad; π (r_f² − r_c²) / h = 1286.482 m; forward of inverse, the dancer's trim in the unwind's wound length | 1e-7 m, 1e-3 rad, 1e-9 m | `the_radius_after_a_wound_length_matches_the_hand_calculation` |
| Roll inertia at a radius; inertia ratio | ½ ρ w π (r⁴ − r_c⁴) + J_core: 0.1366427 kg m² at 110.503 mm, 0.4485586 full; ratio over i² J_m: 10.4026 full, 0.2319 on the core | 1e-7, 1e-4 | `the_roll_inertia_at_a_radius_matches_the_hand_calculation` |
| Steady torque for a tension through the ratio, holding and feedforward | T r / i: −2.142857 and 0.642857 N m at the roll's start (100 N), −0.507857 and 1.692857 N m at its end (taper 79 N) | 1e-6 N m | `the_steady_torque_for_a_tension_through_the_ratio_matches_the_hand_calculation` |
| Torque to speed the line up by 2 m/s² at 640 m, with the tension | ((J + i² J_m) (a ∓ h ω² / 2π) / r ± T r) / i, T = 82.21686 ∓ 1.25 N (the roller's J_d a / 2R²): rewind 1.7824107 N m at rest, 1.7818049 at 2 m/s; unwind −0.8150135, −0.8144080; the rigid model under them accelerates the line at 2 m/s² with the dancer at rest | 1e-6 N m, 1e-9 m/s² | `the_torque_to_accelerate_the_line_matches_the_hand_calculation` |
| A span's steady tension for a speed difference, 1 against 1.0005 m/s, unwind wound slack | EA (v2 / v1 − 1) = 40.000 N in both spans (the first-order span equation gives 39.980 N) | 1e-6 N | `a_span_settles_at_the_tension_of_its_speed_difference` |
| The dancer's static position for a tension and a load: cylinder set for 120 N, rolls held at 100 N | (2 · 120 − 2 T) / k_air with T = 100 r0 / (r0 − 2.88 µm) = 100.0026 N: 39.9948 mm | 2e-7 m | `the_dancer_settles_where_the_web_carries_its_load` |
| Natural frequencies and damping on the web's stiffness | sqrt((2 EA / L + k_air) / m) / 2π = 26.0710 Hz, ζ = 0.026454; the roller sqrt(R² 2 EA / L / J_d) / 2π = 56.9410 Hz | 1e-4 Hz | `the_dancer_frequency_on_the_web_stiffness_matches_the_hand_calculation` |
| Conservation of web length over a 3 m run | paid out / (1 + ε_0) = taken up (trapezoid of the rewind's flow) + Δ(M1 + M2) | 1e-7 m | `the_web_that_leaves_the_unwind_arrives_at_the_rewind_or_stays_in_the_spans` |
| Round trip and static hold at a radius | at L = 0, a quarter and the capacity: still for 1 s under the holding torques | 1e-7 m (line), 1e-9 m (dancer) | `the_winder_holds_at_rest_at_both_ends_of_the_roll` |
| The draw and the dancer's loop: roll wound at 100 N, web at 80 N, 2 m/s | without the loop y' = v (ε − ε_0) / (2 (1 + ε_0)) = −0.24969 mm/s; with it the trim settles at 0.49938 mm/s and the dancer stands | 1 % | `the_dancer_loop_takes_out_the_draw_of_a_roll_wound_at_another_tension` |
| The own verdicts on the shipped cycle (4 m/s²) | inertia ratio 10.4026; speed at the core 7 · 2 / 0.045 = 311.11 rad/s; tension's torque at the full roll with the roller's J_d a / 2R² = 2.5 N: (100 + 2.5) 0.15 / 7 = 2.1964, (79 + 2.5) 0.15 / 7 = 1.7464 N m; each fails with its rating just below | 1e-4, 0.05 rad/s, 0.01 N m | `the_own_verdicts_judge_the_cycle_and_fail_at_their_limits` |
| Emergency stops from 2 m/s | category 2 at both ends and in the middle, category 1 in the middle: web between 30 and 250 N, dancer within 40 mm, standing after 0.5 s (4 m/s²) and 2.0 s (the OFF3 ramp, 0.99 m/s²); category 1 near the ends: the surfaces part at 1.346 − 0.404 m/s², the dancer meets its stop after sqrt(2 · 0.05 / 0.471) = 0.46 s, the web above 250 N; category 0: slack | 0.05 s | `an_emergency_stop_along_the_line_keeps_the_web_within_its_limits`, `a_stop_on_each_drives_own_ramp_overloads_the_web_where_the_radii_differ` |

The standard suite runs on its four loss-free configs (rigid and elastic, with and without the dancer's weight; the
taper off, as it feeds energy in through the cylinder) in the middle of the roll: there the radii are equal, the
tension the suite's constant holding torques keep stays at the tension the unwind was wound at, and the strain the web
carries in equals the strain it carries out. At the roll's start the radii part as the rolls turn, the tension drifts
off the wound tension, and the elastic runs lose 0.36 % of the pulse's energy over the 10 s, exactly what
`power.web_transport` books (the loss budget of the default config, at the roll's start, closes with it).

Of the standard suite, round trip, static hold, determinism, energy conservation, the loss budget and step convergence
run on `machines/pneumatic-axis.toml` as on every machine file (`every_machine_file_passes_the_standard_suite`); they
pulse the servo X, and the cylinder rests in its end stop, so they check that it does rest there exactly
(the equilibrium is the stop's deflection under the net pressure force). For the cylinder itself energy conservation
does not apply (it is fed by the supply): the closed-cylinder check and the enthalpy balance above take its place, and
the stroke-time convergence that of the travel.

## Hand calculations: toggle press

The knuckle-joint press `toggle-press` of `machine-crank-cam` (tests in its `src/toggle_tests.rs`): links a = b =
0.15 m, crank r = 0.05 m at 120° from the rod at the dead centre, rod c = 0.25 m, the crank's centre at (0.275,
−0.1933013) m from the top pivot. The independent numbers come from a circle intersection for the knee, the ram by
Pythagoras and central differences for the rates, not from the closed-form chain of `toggle.rs`.

| What is compared | Against | Tolerance | Test |
| --- | --- | --- | --- |
| Ram and ratio at three crank angles and the dead centre | −90°: knee (0.0709256, −0.1321724), φ = 0.4925083, x = 0.2643449 m (35.655 mm above the bottom dead centre), x' = 0.0364766 m/rad; −20°: x = 0.2982085 m, x' = 0.0108256; −2°: x = 0.2999844 m, x' = 0.00090095; 0°: x = a + b, x' = 0, φ' = D·P_q / D·K_φ = −r sin(t0) / a = −0.2886751 (D = (−c, 0)), x'' = −(a + a²/b) φ'² = −0.025 m/rad²; every rate the difference of its position, through the dead centre too | 1e-7 | `the_toggle_kinematics_match_the_hand_worked_linkage` |
| The inverse's two branches | 1 mm above the bottom dead centre cos φ = x / 2a = 0.9966667, φ = ±0.0816724: the crank at −0.2646374 rad before, as far past after; 0.1 mm: −0.0871754 rad; the round trip over the stroke; beyond the links' reach none | 1e-7, 1e-8 | `the_inverse_has_a_branch_on_each_side_of_the_dead_centre` |
| The force amplification | 10 N m of the motor through 10, loss-free: 100 N m at the crank hold 100 / x' = 2741.48 N at −90° and 42530.15 N at −5°, 15.5 times as much | 1e-3 N m | `the_toggle_amplifies_the_crank_torque_towards_the_dead_centre` |
| Holding torque of 10 kN | F x' at the crank: 79.3873 N m 1 mm before the dead centre (8.18 N m at the motor), 23.4866 N m 0.1 mm before it, 0 at it | 1e-3 N m | `the_holding_torque_falls_towards_the_dead_centre` |
| The frame's stretch (5e7 N/m) | 20000 / 5e7 = 0.4 mm under the rated force, the `frame-deflection` limit; at the dead centre the table's 12000 N − 2e6 N/m · F / k: F = 12000 / 1.04 = 11538.46 N, the ram 0.230769 mm short of the links' bottom (signals `ram.height`, `frame.deflection`); rigid 12000 N at the bottom | 1e-3 N, 1e-9 m | `the_frame_stretches_under_the_press_force_and_the_ram_stops_short` |
| Reflected inertia | J = J_crank + m_pin r² + (m_up/3 + m_knee) a² φ'² + m_x x'²: 0.01 + 0.003125 + 2.9166667 · 0.0225 / 12 = 0.01859375 kg m² at the dead centre, 0.05605024 at −90°; the speed term dJ/dq q'² / 2 and gravity's dV/dq by differences | 1e-9, 1e-7 | `the_reflected_inertia_far_from_and_at_the_dead_centre_match_the_hand_calculation` |
| Force capacity and link forces | 15 N m · 10 · 0.97 = 145.5 N m at the crank: 3988.85 N at −90°, the rated 20 kN at −5°, the capacity's use 1 at its limit; at the dead centre both links carry F, the rod nothing; at −20° the rod's torque about the crank's centre is −F x' = −108.256 N m (virtual work) | 1e-3, 1e-6 | `the_force_capacity_and_the_link_forces_match_the_hand_calculation` |
| A forming force early in the stroke | The 12 kN table 20 mm up asks about 380 N m of the crank's 145.5: `press-capacity` and `motor-peak` fail | – | `a_forming_force_early_in_the_stroke_overloads_the_drive` |
| The dwell at the dead centre | The press force 11538 N held while the motor gives under a tenth of the 9.44 N m it would take 1 mm before the dead centre | 5 N; 10 % | `the_toggle_holds_the_press_force_at_the_dead_centre_with_little_motor_torque` |
| Energy of a forming stroke | Into the part 22 J (the table's area) less 12000 u − 1e6 u² over the last u = 0.2308 mm, 19.284 J; into the frame F² / 2k = 1.331 J; 20.615 J; the motor's Σ τ Δφ from rest 5.5 mm above to rest at the dead centre, rigid and loss-free | 1 % | `a_forming_stroke_takes_the_hand_worked_energy_from_the_motor` |
| Friction feedforward from the model | (1 N m + 0.2 · 2 rad/s) / (10 · 0.97) + 0.05 + 2e-4 · 20 = 0.1983299 N m | 1e-6 | `the_steady_friction_is_the_hand_calculated_one` |
| Standard suite, verdicts | The suite (with gravity: the middle of the crank's range lies where the ram's weight pulls towards the dead centre, stable); the shipped cycle and `toggle-press-cam.toml` pass every verdict | suite's | `toggle_press_passes_the_standard_suite`, `every_default_cycle_and_every_cam_passes_its_verdicts` |

## Hand calculations: indexing table

The rotary indexing tables `geneva-drive` and `cam-indexer` of `machine-crank-cam` (tests in its
`src/indexer_tests.rs`). The Geneva's numbers come from the textbook closed forms (the peak acceleration where
cos x = sqrt(k² + 2) - k, k = (1 + lambda²) / (4 lambda)), the code's from a grid of 200001 points over the index.

| What is compared | Against | Tolerance | Test |
| --- | --- | --- | --- |
| Geneva's motion share and dwell share, n = 4, 6, 8 | (n - 2) / (2n) = 1/4, 1/3, 3/8 and (n + 2) / (2n) = 3/4, 2/3, 5/8 | 1e-12 | `the_geneva_s_dwell_share_and_peak_speed_and_acceleration_ratios_match_the_textbook` |
| Geneva's peak speed ratio | lambda / (1 - lambda) = 2.41421356, 1, 0.61991440 | 1e-8 | same |
| Geneva's peak acceleration ratio | 5.406981, 1.349637, 0.699757 (at 11.46°, 22.90°, 31.64°; the textbooks' 5.41, 1.35, 0.700) | 1e-5 | same |
| The step at the pin's entry | phi'' from 0 to tan(pi / 6) = 0.57735; on the shipped table at 60 turns a minute the wheel's torque 0.55 · 0.57735 · 39.4784 = 12.536 N m, the pin's force over C cos(pi / 6) = 0.103923 m 120.63 N, the crank's torque 0 | 1e-3 N m, 1e-2 N | `the_geneva_s_acceleration_jumps_at_the_pin_s_entry_and_the_torque_on_the_wheel_with_it` |
| Modified sine | peak acceleration 4 pi² / (4 + pi) = 5.527957, peak speed 4 pi / (4 + pi) = 1.759603, s, s', s'' continuous at 1/8 and 7/8 | 1e-6 | `the_modified_sine_s_peak_acceleration_is_5_53_and_its_peak_speed_1_76` |
| Cam indexer's peaks over 120° | 1.047198 · 5.527957 / 2.094395² = 1.319707, 1.047198 · 1.759603 / 2.094395 = 0.879802 | 1e-5 | `the_cam_indexer_s_peaks_are_the_modified_sine_s_over_its_index_angle` |
| The table's ring after an index (the table side, 0.52 kg m², twisting against the wheel the drive holds: w_n = sqrt(2e5 / 0.52) = 620.17 rad/s, zeta 0.03) | Geneva: the exit step 22.794 rad/s² over w_n² = 5.927e-5 rad, settled to 5e-5 rad in ln(5.927 / 5) / (0.03 · 620.17) = 9.1 ms, 83.7 ms at twice the speed; cam: its jerk 1964.1 rad/s³ over w_n³ = 8.23e-6 rad | 3 %, 2 ms, 3 ms; 5 % | `the_geneva_leaves_the_table_ringing_by_its_acceleration_step_and_the_cam_indexer_does_not` |
| Energy, step convergence, determinism | the standard suite | as for every machine | `geneva_drive_passes_the_standard_suite`, `cam_indexer_passes_the_standard_suite` |

The redundant arm's `kinematics.redundancy = "joint_limits"` (`machine-joint-kit`): on the seven-axis URDF the pose is
reached to 1e-9, the weighted offset from the ranges' middles has no part along the null space (below 1e-6), its
weighted sum is below the home solution's, and the machine built with the key solves the same joints
(`the_seven_joint_arm_can_spend_its_spare_joint_on_staying_off_its_stops`). Its dynamics are the seven-axis URDF's,
compared with MuJoCo above (`seven_axis_arm_matches_mujoco`); the option changes only which joints solve a pose.

## Hand calculations: overhead crane

The crane of `machine-overhead-crane` (tests in its `src/tests.rs`): sheave at H = 5 m, hook 15 kg and payload 250 kg
(m g = 2598.762 N), rope EA = 1.44e6 N; the trolley 120 kg, its motor and gearbox input 1.34e-4 kg m² at 500 rad/m; the
hoist 40:1 on an 80 mm drum (500 rad/m).

| What is compared | Against | Tolerance | Test |
| --- | --- | --- | --- |
| The rope at rest on 4 m: tension, free length, stretch, elastic energy, the drum's pull and the hoist's holding torque; the brake's check | t = m g = 2598.762 N; the drum at the pose's free length l = 4 / (1 + 15 g / EA) = 3.999591 m, the load on it L = l (1 + m g / EA) = 4.006809 m, e = 7.2180 mm; t e / 2 = 9.3790 J; t (1 + e / (2 l)) = 2601.107 N, / 500 = 5.20221 N m; 12 N m / 1.6 = 7.5 N m | 1e-3 N, 1e-6 m, 1e-4 J, 1e-5 N m | `the_static_stretch_tension_and_holding_torque_are_the_hand_calculated_ones` |
| The payload's own stretch below the motors' pose (they count on the empty hook's), the tool pose, the setpoint slack | 4 m (s / s_h − 1) = 6.8095 mm on 4 m; `forward` and `tool_state` give the pose; 4.5 m × 250 g / EA + 0.1 mm = 7.762 mm | 1e-7 m, 1e-12 m, 1e-6 m | same test |
| The sway's period, small amplitude, the pose 3 m below the sheave (L = 3.005107 m with the payload's stretch) | Trolley braked: 2π √(L / g) = 3.47816 s; trolley free: two masses, 3.47816 / √(1 + 265 / 153.5) = 2.10647 s (153.5 kg the trolley with its rotor at the rail) | 2e-4 relative | `the_sway_period_is_the_hand_calculated_pendulum_s` |
| The energy budget of a loss-free lift, 265 kg from the pose z = 1 to 3 m | m g h with h = 2.00340474 m (the load hangs 6.8095 mm below the pose on 4 m, 3.4047 mm on 2 m) = 5206.373 J, less the rope's t e / 2 at 4 m and 2 m (9.3790 − 4.6895 J): 5201.683 J; the motors' work ∫ T ω dt equals the machine's energy gained | 1e-4, 1e-3 | `a_loss_free_lift_takes_the_hand_calculated_work` |
| Signals at rest, the pose 2 m below the sheave | `rope.tension` 2598.76 N, `rope.length` 2.0034047 m, `hoist.brake_load` and the hoist's gravity torque 5.2 N m, `energy.elastic` 4.6895 J | 0.5 N, 1e-6 m, 0.01 | `the_signals_at_rest_are_the_hand_calculated_ones` |
| Input shaping against the sway | A travel on 2 m of rope: the ZVD leaves under a tenth of the unshaped sway (0.072 m unshaped, 3e-5 m shaped). Hoisting from 4 to 2 m while travelling 7 m: following the rope's length 0.012 m, under a third of a ZVD set for either end's length (0.047, 0.041 m), under a tenth of unshaped (0.43 m) | as stated | `the_shaper_cancels_the_sway_and_following_the_rope_length_beats_a_fixed_period` |
| The default cycle | Shaped, every verdict passes, the rope within 7 % of the load's weight and never slack; unshaped, `residual-sway` fails | as stated | `the_default_cycle_passes_with_the_shaper_and_leaves_the_load_swinging_without` |

## Friction feedforward in the shipped configs

`sim-cli analyze` of every shipped config with `effects.friction_feedforward = false` and with it on and `drive.friction_feedforward_model = "machine"` (15 November 2026, re-run 17 November after the stage of modelled effects, and on 21 November 2026 after the screw and rack damping, the screw stiffness that follows the nut, the T-gantry's belt runs' weight and the XYZ portal's Z belt loop: `sim-cli analyze ID [--config FILE | --machine-file FILE] --set effects.friction_feedforward=false`, and `=true --set 'drive.friction_feedforward_model="machine"'`): the following error while moving (`tracking_error_peak`, the tool's distance from the setpoint; `_angle_` for the axes in rad) and the settling at pick and place (`dwell_error_peak`, the second half of the dwells). Place in um, turn in mrad; "–" where the machine has none (no dwell in a conveyor or cam cycle). **On** marks where it is switched on as shipped. The velocity loop's margins do not change with a feedforward (`sim-cli margins` gives the same figures with it on and off), so every config keeps the margins it was tuned to: `sim-cli margins MACHINE --bands` of every config switched on gives (17 November, the same after the stage of modelled effects) the rotary knife 45.6° and 10.98 dB, the T-gantry 46.2° and 8.38 dB, the D4 45.9° and 8.92 dB (35.8° and 8.0 dB, 48.9° and 8.63 dB, 40.4° and 8.92 dB on 15 November; the other configs not re-run).

| Config | Following off → on | Settling off → on | On |
| --- | --- | --- | --- |
| T-gantry | 199.9 → 182.0 | 48.4 → 43.4 | yes |
| T-gantry conveyor | 676.8 → 664.0 | – | yes |
| T-gantry EXCT-30 | 543.3 → 519.6 | 40.5 → 35.4 | yes |
| Single axis | 201.1 → 126.3 (characteristic 118.5) | 20.8 → 42.5 (characteristic 38.8) | no |
| D3 | 1460 → 1130 | 305.2 → 179.3 | yes |
| D3 conveyor | 1428 → 1194 | – | yes |
| D2 | 1061 → 1046 | 369.6 → 210.5 | yes |
| D2 conveyor | 1119 → 1080 | – | yes |
| D4 | 3655 → 3438; 5.831 → 5.371 mrad | 387.9 → 308.8; 0.192 → 0.187 mrad | yes |
| D4 conveyor | 3156 → 3046; 3.320 → 2.860 mrad | – | yes |
| H4 | 2477 → 2304; 24.51 → 26.54 mrad | 435.9 → 366.1; 3.589 → 3.043 mrad | no (turn follows worse) |
| Linear delta | 700.4 → 334.0 | 73.5 → 109.8 | no |
| Linear delta conveyor | 730.5 → 371.8 | – | yes |
| Parallel SCARA | 1012 → 1039; 4.702 → 1.131 mrad | 334.7 → 239.5; 0.760 → 0.095 mrad | no (place follows worse) |
| Screw axis | 79.9 → 31.1 | 45.2 → 7.4 | yes |
| Rack axis | 118.4 → 50.3 | 61.7 → 7.9 | yes |
| Linear motor axis | 43.5 → 39.6 | 24.5 → 6.6 | yes |
| Rotary table | 2.584 → 2.506 mrad | 0.0341 → 0.0224 mrad | yes |
| Six-axis arm (joints) | 4.453 → 4.413 mrad | 1.968 → 1.921 mrad | no (gain under 3 %; its tool-space configs follow worse) |
| Arm, 5- and 7-axis arm in tool space | 5869 → 5856, 6154 → 6136, 5868 → 5855; 7.910 → 7.946, 7.940 → 7.988, 7.920 → 7.960 mrad | 2303 → 2278, 2301 → 2285, 2309 → 2283; 3.089 → 3.163, 2.892 → 3.004, 3.072 → 3.147 mrad | no |
| SCARA | 166.5 → 28.6; 1.776 → 1.701 mrad | 70.9 → 8.3; 0.270 → 0.080 mrad | yes |
| SCARA in tool space | 542.5 → 512.8; 1.566 → 1.264 mrad | 84.3 → 30.6; 0.0215 → 0.0020 mrad | yes |
| Palletiser | 6181 → 6185; 2.684 → 2.637 mrad | 2312 → 2308; 0.0038 → 0.0036 mrad | no |
| Slider crank (and cam) | 4.444 → 4.467 mrad (2.265 → 2.269) | 0.134 → 0.047 mrad | no |
| Rotary knife | 12.31 → 12.07 mrad | 0.248 → 0.072 mrad | yes |
| Rotary knife cam | 5.694 → 5.697 mrad | – | yes (as the rotary knife: the cam configs are the shipped one running its cam) |
| Flying saw (and cam) | 381.4 → 369.6 (428.9 → 342.1) | 179.9 → 25.7 | characteristic, as before: 369.3 and 26.3 (341.9) |
| Disc cam | 0.319 → 0.232 mrad | 0.228 → 0.024 mrad | yes |
| Disc cam cam | 0.158 → 0.158 mrad | – | yes |
| Hexapod | 25.4 → 11.0; 0.077 → 0.025 mrad | 15.5 → 1.3; 0.034 → 0.004 mrad | yes |
| Planar cable robot | 714.2 → 649.9 | 157.7 → 148.5 | yes |
| Spatial cable robot | 297.7 → 204.5; 1.441 → 1.672 mrad | 36.7 → 35.7; 0.722 → 1.030 mrad (the turn follows and settles worse with it) | yes |

The machine files (`machines/*.toml`, `sim-cli analyze --machine-file`) by the same rule. The T-gantry, D3 and planar delta files are their crates' machines and follow their configs (on); the single axis's stays off as its crate's. The netlist machine tells the same steady friction as the crate it mirrors, to 1e-9 (`the_netlist_machines_are_their_crates`).

| Machine file | Following off → on | Settling off → on | On |
| --- | --- | --- | --- |
| `t-gantry.toml` (no frame, no leaning Z) | 115.6 → 105.6 | 47.1 → 41.8 | yes |
| `delta-d3.toml`, `planar-delta.toml` | as the D3 and D2 above | | yes |
| `screw-portal.toml` | 143.5 → 99.5 | 31.3 → 22.1 | yes |
| `xyz-portal.toml` | 142.7 → 107.3 (141.9 → 104.4 before its Z belt was a loop, 21 November) | 30.6 → 22.7 (29.9 → 22.3) | yes |
| `corexy.toml` | 77.0 → 49.1 | 15.6 → 22.0 | no |
| `h-bot.toml` | 121.8 → 111.2 | 13.2 → 18.9 | no |
| `dual-gantry.toml` | 165.5 → 109.8 | 10.2 → 31.7 | no |
| `rack-gantry.toml` | 81.6 → 56.8 | 14.6 → 22.2 | no |
| `portal-c.toml` | 177.4 → 144.9; 0.576 → 0.588 mrad | 34.0 → 23.6; 7.3e-5 → 8.0e-5 mrad | no |
| `five-axis.toml` | 201.1 → 132.1; 0.151 → 0.155 mrad | 33.6 → 32.3; 0.055 → 0.059 mrad | no |
| `five-axis-table.toml` | 189.0 → 112.6; 0.088 → 0.099 mrad | 30.3 → 34.3; 0.022 → 0.010 mrad | no |
| `induction-lift.toml` | 284.6 → 291.5 | 217.9 → 202.8 | no |
| `stepper-axis.toml`, `pneumatic-axis.toml`, `scissor-lift.toml` | unchanged (15.2 to 206.2) | unchanged | no |

Where it settles worse (the single axis, the linear delta: belt axes whose guides have a LuGre contact), the cause is not worked out; the single axis does the same with its shipped characteristic, so it is not the machine model's value. The guides' feedforward fades in over their Stribeck speed (`Lugre::feedforward`), so that a stop puts no torque step on the axis (the plain steady value changed the figures above by under 10 %). The arm's joints, the slider crank and the palletiser change by under 3 %.

## Vendor formulas and vendor documents

| What is compared | Against | Tolerance | Test |
| --- | --- | --- | --- |
| Motor torque of the EXCT-30 config at 45 degrees travel, from the simulator's rigid model | Festo's torque formula with the catalogue's coefficients, for several accelerations and loads | 2 % | `exct_30_config_agrees_with_the_catalogue`, gantry |
| The `[catalogue]` values of `exct-30.toml` | `catalog/festo/belt_axes.csv`, `belt_axis_torque_formula.csv` and `catalog/siemens/motors_s210.csv` | 0.1 % or exact | the same test |
| Festo's formula coefficients | The T-gantry's own mechanics from the pulley diameter | 1 % to 3 % | `festo_exct_values_agree_with_its_own_geometry`, catalogue validator (`catalog/tools/cataloglib/checks.py`) |
| 1FK2 data from the S120 configuration manual | The same motors in the S210 catalogue, read by another script | 5 % to 6 % | `siemens_1fk2_data_agrees_between_the_s120_manual_and_the_s210_catalogue`, catalogue validator (`catalog/tools/cataloglib/checks.py`) |
| 1FT2 data from the S120 configuration manual | The same motors in the S210 catalogue | 5 % to 6 % | `siemens_1ft2_data_agrees_between_the_s120_manual_and_the_s210_catalogue`, catalogue validator (`catalog/tools/cataloglib/checks.py`) |
| 1FK7 and 1FT7 data sheets | Catalogue D 21.4: pole count; torque constant = static torque / stall current | Exact; 8 % | `siemens_s120_data_sheets_agree_with_the_catalogue`, catalogue validator (`catalog/tools/cataloglib/checks.py`) |
| The data sheets in themselves | Voltage constant against torque constant; L / R against the electrical time constant | 15 % | the same test |
| All other catalogue files | Their own physics; see [catalogue](catalogue.md#how-the-files-were-extracted-and-checked) | Per test | catalog |
| A motor and a gearbox by name | The same values put in by hand | Identical machine | `a_motor_by_name_gives_the_values_of_the_hand_written_config`, `a_gearbox_by_name_sets_ratio_stiffness_backlash_and_ratings`, `parts_named_in_a_config_behave_like_their_numbers`, catalog |
| Velocity gain the search gives a lighter motor | The gain by hand, 0.6 → 0.31 | Per test | `the_search_retunes_the_velocity_loop_for_a_lighter_rotor`, catalog |
| What the gearbox search rules out by catalogue numbers | Ratios 7 and above for a 3000 r/min motor, by hand; each runs above its rated speed when simulated | Exact list | `the_gearbox_search_rules_out_by_catalogue_numbers_only_what_would_fail`, catalog |
| The axis EXCT-30 by name | The hand-written `exct-30.toml`: pulley, strokes, Z mass 4.7 kg, Festo's EA, AT5 at 30 mm (2140 N, 97.86 g/m interpolated by hand), 10 teeth in mesh, 12.5 N m; the carriage fitted again 7.136 kg against 7.16 kg; the machine's torque at 45 degrees against Festo's formula | 0.1 %, exact; carriage 0.5 %; torque 2 % | `an_axis_by_name_gives_the_numbers_of_the_hand_written_exct_30`, catalog_parts |
| Bahr axes by name | ELZI 40: carriage 7.0 − 2.4 kg, Z 2.4 kg, radius 160 mm / 2π, EA 1805 N / 0.004, 16 teeth in mesh, 1000 N × r at the pulley; ELZ 60 on the single axis; wrong kind, too short a stroke and an unknown name refused | 1e-9 | `bahr_axes_by_name_bring_the_sheets_moved_masses_pulley_and_belt`, catalog_parts |
| A belt by name | AT5-25 as printed; MTD8-12 two fifths between 10 and 15 mm; AT10-10 as 10/16 of the 16 mm row | 1e-9 | `a_belt_by_name_is_the_megadyne_row_or_between_two`, catalog_parts |
| The drive's current checks | RMS 3.2 A against 5 A is 0.64, peak 12.4 A against 19 A is 0.653; both fail against the 1.2 A and 4.2 A converter; the checks read the cycle's own currents | 1e-12 | `a_drive_by_name_judges_the_motor_current_at_the_hand_calculated_share`, catalog_parts |
| An S120 motor by name | 1FK7060-2AF7: torque constant 6 / 4.45, resistance and inductance three times the data sheet's 1.35 Ω and 15.2 mH, current loop Kp = L / 0.25 ms, Tn = L / R; 1FK2103-2AH rated speed 4500 r/min; the pairings 6SL3120-1TE15-0AD0 and (stall current 1.06 A) 6SL3120-1TE13-0AD0 | 1e-9 | `s120_motors_by_name_bring_winding_rated_values_and_their_motor_module`, catalog_parts |
| The search's order | Three passing motors on the EXCT-30 by their costs by hand, 6.8, 10.4, 12.6 kg (two motors each); motor and gearbox 2.85 + 1.4 kg | Exact | `the_search_ranks_three_passing_motors_by_their_hand_calculated_weight`, catalog_parts |
| The axis search | EXCT-15 ruled out by stroke, ELZI 30 by speed, without a cycle; costs EXCT-30 41.105 + 10.4 kg, ELZI 40 19.05 + 10.4 kg by hand | 1e-9 | `the_axis_search_rules_out_by_catalogue_numbers_and_adds_the_axis_weight`, catalog_parts |

The Festo comparison is the one place where the simulator's numbers meet a vendor's numbers for a whole machine. It covers the rigid torque (inertia and gravity), with the carriage mass of the config chosen so that it fits. It does not cover friction, which Festo gives only as a graph.

## Self-consistency

These compare the simulator with itself. They prove repeatability and plumbing, not physics.

| What | Test |
| --- | --- |
| The same run twice is bit-identical, with control and with sensor noise | `the_same_run_twice_is_bit_identical`, control |
| Halving the integration step changes a controlled move with all effects on by less than 1 % | `halving_the_integration_step_changes_a_controlled_move_by_less_than_one_percent`, control |
| Every effect switch changes the traces | `every_effect_switch_changes_the_traces`, control |
| A trace written by `run` replays onto itself: torque and following error within 0.5 % (RMS) and 5 % (peak) of their largest value; the fit finds the config's own masses and friction within 2 % | `a_trace_from_run_replays_onto_itself_and_fits_its_own_config`, cli |
| `sensitivity` leaves `ratings.bearing_reliability_percent` out (it varied it to 72 % and 108 %, which `verdicts` refuses) | `sensitivity_leaves_the_bearing_reliability_alone`, life |
| `replay` and `fit` refuse a machine whose transformation is not linear (they printed numbers without meaning on the delta), with a message in both languages; `step` and `bode` take the axis as `--axis-index`, `--axis` names a catalogue axis | `a_trace_from_run_replays_onto_itself_and_fits_its_own_config`, `step_bode_and_tuning_write_csv_for_every_machine`, cli |
| A config file of another machine opens that machine in the app; one no machine takes leaves the model as it was | `a_config_of_another_machine_opens_that_machine`, app |
| The EXCT-30 cycle stays below 0.8 of the torque-speed curve and of the rated torque, and the belt does not go slack | `exct_30_cycle_stays_within_motor_and_belt_limits`, catalog |
| The optimiser's result is shorter by at least 5 % and passes every verdict when run again | `the_optimiser_shortens_the_cycle_within_the_verdicts`, axis |
| No row of the optimiser's energy front is beaten by another in cycle time, energy and peak utilisation at once; it is sorted and reaches from the result to slower than the configured cycle; by hand on six points | `every_row_of_the_energy_front_is_beaten_by_no_other`, axis; `the_front_keeps_the_points_no_other_beats_in_every_figure`, analysis |
| A PLC-commanded move runs and reports position and following error, for every machine | `a_plc_commanded_move_runs_and_reports_position_and_following_error`, plc |
| A move commanded through the FMU's C functions (instantiate, initialise, a ramp of setpoints, one per setpoint interval, then a hold) gives every signal bit for bit as a plain simulation given the same setpoints, for every machine; an effect parameter and the drive enable input act as the config switch and `Command::Disable` do, bit for bit | `a_move_through_the_fmu_matches_a_plain_simulation_bit_for_bit`, `effect_switches_and_the_drive_enable_act_on_the_simulation`, fmi |
| The single axis and the T-gantry rewired as machine files (`machines/*.toml`) are their crates: kinematics, rest state, derivatives, energy, feedforward, tensions and signals to 1e-9 in five sets of effects; a whole cycle without encoder counts to 1e-5; with them the same verdicts within the crate's own spread under a 1e-9 change (see [machine files](machine-files.md#how-it-is-validated)) | `the_netlist_machines_are_their_crates`, `the_netlist_machines_run_their_crates_cycle`, `the_netlist_machines_give_their_crates_verdicts`, machine_files |
| Every shipped machine file passes the standard suite (the H-bot's without its gearbox backlash, see [machine files](machine-files.md#how-it-is-validated)), and the H-bot's motor angles and torques match a hand calculation | `every_machine_file_passes_the_standard_suite`, `the_h_bot_moves_as_worked_out_by_hand`, machine_files |
| The T-gantry's machine file with each of frame, idlers, cable chain, process force, thermal feedback, belt tooth jump, span vibration, tool flex (its `machine.lean` node) and belt wrap (from the places of its pulleys and idlers) switched on, and with all of them, elastic and rigid, is its crate to 1e-9, its wraps too | `the_effects_of_a_machine_file_are_its_crates`, machine_files |
| A machine file's belt wrap by hand (`mech::wrap_angle`'s formula, to 1e-12), from an offset idler and a clamp that is not level: π less the lean of both runs, 2.9991496 rad at x = 0 and 3.0389854 rad at 0.8 m (pulley r = 0.02 m at (-0.2, 0.03); clamp at height 0.06; idler r = 0.012 m 0.3 along and 0.06 below) | `the_belt_wraps_a_pulley_as_far_as_the_places_of_its_neighbours_say`, machine_file_geometry |
| A worm node is the joint kit's rotary table with `table.gear = "worm"` (efficiency 0.762938 both ways) to 1e-9, rigid and elastic, driving and driven back; by hand, a turning worm (back efficiency 0.5, ratio 40) back-driven by -10 N m at -10 rad/s accelerates the table by -10 / (4.5 + 1600 · 3.1e-4 / 0.5) = -1.820830 rad/s²; a self-locking one (efficiency 0.4, back efficiency -0.5) holds it at rest with no torque (the gear taking 10 N m), brakes it lowering at -10 rad/s with no torque ((15 - 10) / 4.996 = 1.000801 rad/s², the drag (1 + 0.5) 10 N m), needs a push of 0.5 · 10 / 40 = 0.125 N m to lower it at a steady speed, and from rest sticks at -0.1 and +0.6 N m and breaks loose at -0.15 and +0.7 N m (-0.200160 and 0.600480 rad/s²); its rotor stands where a turning one's turns back at 0.25 / 3.1e-4 = 806.45 rad/s². A worm's efficiencies warmed so that it loses half: 0.9 and 0.75 from 0.8 and 0.5, a locking one staying at 0 | `a_worm_gear_in_a_file_drives_as_the_joint_kits_table`, `a_self_locking_worm_holds_its_load_and_one_that_turns_back_loses_by_its_back_efficiency`, machine_file_geometry; `a_warm_worm_loses_less_and_a_locking_one_stays_locked`, sim-netlist |
| The delta crates and their machine files are one implementation: `machines/planar-delta.toml` and `machines/delta-d3.toml` are their crates to 1e-9 with each effect switched from its default, and their cycles give the crates' signals and verdicts | `the_planar_delta_file_is_its_crate_with_every_effect`, `the_spatial_delta_file_is_its_crate_with_every_effect`, `the_spatial_delta_file_runs_its_crates_cycle`, machine_file_linkage |
| A scissor lift (2 stages, 1 m arms, lead 10 mm): the force on its foot W n cot(alpha) at 0.6 m (6359.59 N per 1000 N, cot 3.179797) and 1.6 m (1500 N, cot 0.75), equal to W dh/dtheta / c; the shipped file holds 280 kg with 1.389627 and 0.445847 N m at 0.6 and 1.4 m (17462.6 and 5602.7 N on the foot), and passes the standard suite without gravity | `the_scissor_force_ratio_is_n_cot_alpha_at_two_heights`, sim-netlist; `the_scissor_lift_holds_its_platform_with_the_hand_calculated_force_at_two_heights`, `the_scissor_lift_file_passes_the_standard_suite`, machine_file_linkage |
| A rack's pinion loads the gearbox shaft with 35.2165 · sqrt(1 + (tan 20° / cos 19.5283°)²) = 37.7513 N, checked against `shaft_max_radial_load_n`; a rotary's bearing friction (2 N m + 0.5 N m s/rad at 1 rad/s) takes (2 + 0.5) / 100 = 0.025 N m at the motor behind ratio 100 | `a_rack_axis_turns_its_motor_by_x_over_r_and_feeds_its_force`, `a_rotary_axis_has_the_hand_calculated_bearing_friction`, machine_file_mechanisms |
| The belts wrap their pulleys beside a screw: the screw portal's X and Y, run from a clamp round the pulley and back, pi + 2 asin(r / D) = 3.2416344 and 3.3419275 rad, its screw's motor none; the belt drawn round two wheels meets them at the tangents ((0.02, -0.0979796) and (0.98, 0.0979796) for a crossed belt) | `the_belts_wrap_their_pulleys_beside_a_screw`, machine_file_geometry; `the_belt_meets_its_wheels_at_the_tangents_and_runs_round_them_in_their_sense`, sim-netlist |
| The catalogue search for one motor of a machine file (the XYZ portal's Z motor, a 1FK2103-2AH): it becomes `[parts.mz]`, X and Y keep theirs, its gain is retuned down, and it costs its own 1.18 kg (the catalogue's mass), not three motors' | `the_search_sizes_one_motor_of_a_machine_file`, machine_file_geometry |
| The CoreXY's motor angles i (x ± y) / r and its torque on a diagonal, r / (2 i) (m_x - m_y) a; the portal's Z motor with its own gearbox (twice the angle and speed of X and Y per metre), its holding torque m g r / i = 0.0686 N m, its own drive and ratings; the single axis's belt mode sqrt(EA (1 / l1 + 1 / l2) / m) / 2 pi = 65.82 Hz (l the sections' unstretched belt under the pretension); the belt strips the 3D view draws | `the_corexy_moves_as_worked_out_by_hand`, `the_portal_moves_and_holds_as_worked_out_by_hand`, `a_machine_file_reports_its_belt_mode`, `the_3d_view_draws_the_belt_of_a_machine_file`, machine_files |
| An FMU of a machine file builds its machine from the file it carries, and gives the signals of a plain `Sim` bit for bit (FMI 2.0 and 3.0) | `a_move_through_the_fmu_matches_a_plain_simulation_bit_for_bit`, `a_move_through_the_fmi3_functions_matches_a_plain_simulation_bit_for_bit`, sim-fmi |
| The FMU refuses what the standard forbids: another GUID, another FMU type, missing resources, a config file that is not one, a communication step that is not a whole number of base steps or starts elsewhere, a parameter set after initialisation, an output set, an unknown value reference, an unsupported call; a reset starts over. The GUID changes with language and start values | `what_the_standard_forbids_is_refused`, `the_guid_changes_with_the_model_description`, fmi |
| The FMU refuses `fmi2GetReal` before initialisation mode (read there, the time output stayed 0 after a start time of 5 s), reads 5 s and 5 s + dt after a start time of 5 s and one step; refuses a step at a time or of a size that is NaN or infinite (an infinite step looped for ever) | `early_reads_and_steps_that_are_not_finite_are_refused`, fmi |
| `fmi2SetupExperiment` refuses a start time that is NaN (after it every communication point passed) and a defined stop time or tolerance that is not finite, and reads neither when not defined; a step of 1e20 s or of 2^32 base steps is refused (it ran u64::MAX steps: the importer hung); `--binary SIM_FMI.DLL` is the Windows binary, stored as `sim_fmi.dll` | `an_experiment_that_is_not_finite_is_refused`, `a_step_too_long_to_run_is_refused`, fmi; `a_binary_is_known_by_its_extension_in_any_case`, sim-fmi lib |
| `fmu` writes a zip whose local headers, CRC-32 and central directory agree (CRC-32 of "123456789" is 0xCBF43926), with the FMI layout; its model description is balanced XML that lists every signal with its unit (defined in `UnitDefinitions`), unique value references, the setpoint inputs and the parameters, and `Outputs` naming exactly the outputs; two `--binary` for one platform are refused (they were two entries of one name) | `fmu_writes_an_archive_with_the_fmi_layout_and_every_signal`, cli; `crc32_of_the_standard_check_string`, fmi |
| Every numeric value of both default configs at -1, 0, NaN and infinity, one at a time: the config is refused or the cycle runs with finite signals; nothing panics | `no_single_bad_number_panics_the_simulation_or_fills_it_with_nan`, review2 |
| A shaper changed at speed leaves the largest setpoint step at speed times profile cycle (0.8 mm on the gantry's X axis) | `a_shaper_changed_at_speed_does_not_make_the_setpoint_jump`, review2 |
| A gear ratio or pulley radius changed by 1 % at standstill and at speed: belt tensions within 10 % of the pretension and the tool within 0.1 mm of a run left alone | `a_changed_ratio_or_pulley_radius_does_not_tear_the_machine`, review2 |
| A path planned from every corner of the workspace, where the setpoint stands up to 44 µm beyond the range, is run and arrives; one that leaves the range by 1 mm is refused | `a_path_from_an_end_of_a_stroke_is_run`, review3 |
| A stream of setpoints at the speed limit and at 0.95 of it, one per interval: none is refused, the setpoint moves at the stream's speed (1e-9 relative) and ends on the last value sent; the same with the setpoints a cycle and a half early and late in turn; a jump of two intervals is refused | `a_stream_of_setpoints_at_the_speed_limit_is_followed`, review3 |
| The app's tables show what the library functions behind the command line give, for every machine: the cycle with its extra figures, the error bands, the ranked motor search, a planned path, a replayed and fitted trace | `the_cycle_is_analysed_and_judged_in_the_app`, `the_checks_get_error_bands_in_the_app`, `the_app_searches_motors_and_names_one_in_the_config`, `a_path_is_planned_run_and_analysed_in_the_app`, `a_trace_is_replayed_and_fitted_in_the_app`, app |
| The worst corner of the bounds in the app and on the command line is what `verdict_worst` finds: per check the worst value, limit, result, and every bounded value named once at its bound; English and German; refused without `--bands` or with `--samples`/`--seed`. The bands beside it, taken from the search's first runs, are those of `verdict_bands` (in the library to the last bit) | `the_app_searches_the_worst_corner_of_the_bounds`, app; `verdicts_search_the_worst_corner_of_the_bounds`, cli; `the_worst_corner_search_finds_the_hand_calculated_corner`, axis |
| On the EXCT-30 the worst of every check is at least as bad as its band of one value at a time (`dc-link-energy` was reported at 4.47 J against a band up to 23.2 J) | `the_exct_30_config_passes_settling_at_the_worst_corner_of_its_bounds`, catalog |
| The app's frequency response, stiffness fit, replay overlay and operating points are those of the shared functions the command line prints (`frequency_sweep`, `fit_sweep`, `overlay_csv`, `MotorLoad::points`), and the command line's output is checked against the same functions. A response of a belt with 500 kN against a config of 400 kN gives the factor 1.25 worked out by hand, within its standard error; the torque-speed curve's corners are the config's peak torque, rated and no-load speed, the S1 line's the verdict's (`s1_torque`: k_t I_0 at standstill, the rated torque at the rated speed, and on along that line); a path's operating points are those of its analysis. The delta's workspace map at z = -1.0 m ends at x = 0.15 - sqrt(1.35^2 - 1.0^2) = -0.757 m, where arm 1's forearm stands in line with its upper arm, and the working envelope's bottom sphere cuts it at 0.355 m | `the_workspace_map_ends_at_the_hand_calculated_reach_of_the_delta`, `the_frequency_response_is_measured_in_the_app`, `the_belt_stiffness_is_fitted_to_a_measured_response_in_the_app`, `the_replayed_trace_is_overlaid_and_exported_in_the_app`, `the_operating_points_are_shown_over_the_torque_speed_curve`, app; `analyze_and_sweep_write_csv_for_every_machine`, `a_trace_from_run_replays_onto_itself_and_fits_its_own_config`, cli |
| A result of the app is not applied to a config changed since: a motor of the search, a planned path, the optimiser's settings | the same tests and `the_app_optimises_the_cycle_and_applies_the_settings`, app |
| A drive that trips is reported once per trip, for the running machine and for an analysed cycle | `a_drive_that_trips_is_reported`, app |
| The app's effect views are the analysis's and the machine's: live parts adding up to the torque at every point, peak and RMS per part and the budget row by row as `analyse_cycle` gives them, and its flow diagram's columns adding up (the supply and what stores gave back to the electrical losses and the shaft work, the shaft work to the machine's losses and stores), the friction curve as `friction_curve` at the newest load factor with the newest point on it; `analyze --breakdown` and `--budget` write the same | `the_effect_views_show_torque_parts_energy_budget_and_friction`, app; `analyze_writes_the_torque_breakdown_and_the_energy_budget`, cli |
| The thermal, belt and comparison views are the simulation's: winding and part rises and the belt tension and stretch as their newest samples, settling rises as `MotorLoad::winding_rise` and as R_th times the budget row over the cycle's duration, the comparison's differences as `Comparison::difference`; the colour scales of the 3D view at their ends and middle; `compare` writes the same as `compare_effect` | `the_thermal_belt_and_comparison_views_show_the_simulation`, app; `compare_writes_the_cycle_with_an_effect_on_and_off`, cli |
| Dependencies point downwards; every version is pinned; no machine is named outside its crate | [`architecture.rs`](../crates/sim-machines/tests/architecture.rs) |
| Every label exists in German and English | [`i18n.rs`](../crates/sim-machines/tests/i18n.rs), and in the app's and the link's tests |

## Against measured data

One public test bench, not built or measured by this project: the ball screw axis of ISW, University of Stuttgart, in
three datasets on DaRUS, all CC BY 4.0 (licence checked on each dataset's page): C. Hinze, L. Zeh 2022,
[doi:10.18419/darus-3003](https://doi.org/10.18419/darus-3003) (the three recordings below), C. Hinze et al. 2023,
[doi:10.18419/darus-3252](https://doi.org/10.18419/darus-3252) (velocity sweeps, the speed loop's gains), and
[doi:10.18419/darus-2563](https://doi.org/10.18419/darus-2563) (the linear direct drive's force compensation; read, not
brought in: its constant-speed runs carry nothing the three recordings and their nut force do not, its README explains
the drive's force column). Converted into [datasets/isw-ball-screw-axis](../datasets/isw-ball-screw-axis/). A 1FT7085
motor on a SINAMICS S120 turns a 40 mm ball screw that moves a 400 kg table; speed and position loops on a dSPACE board.
The machine is the `screw-axis` of the joint kit with the dataset's `machine.toml`.

**The recordings and their roles.** The S-profile (10 to 700 mm and back, 0.3 m/s, 1 m/s², 6.2 s) is the one everything
is fitted on. The sine (±100 mm at 0.1 Hz, 21 s) and the run at 0.1 m/s out and back (15 s, recorded two months
earlier) are checks: nothing was set from them. The velocity sweeps (darus-3252, at 0, 350 and 700 mm) check the screw's
stiffness and the lead, and are compared, not fitted.

**Compared**: the measured motor torque (`M_KGT_act`) with the simulated one when the trace's position setpoints are
replayed (`sim-cli replay`), as written for 1.0.0 and now. Differences as a share of the recording's peak measured
torque; "as written" is the 1.0.0 table (the config before any fit, the replay from rest at the configured pose), "now"
is `machine.toml` as committed:

| Recording | Role | Peak torque | RMS, 1.0.0 | RMS, now | Peak difference, 1.0.0 | now |
| --- | --- | --- | --- | --- | --- | --- |
| S-profile | fit | 8.07 N m | 0.857 N m, 10.6 % | 0.368 N m, 4.6 % | 7.39 N m, 92 % | 2.20 N m, 27 % |
| Sine, 0.1 to 19.9 s | check | 3.52 N m | 0.502 N m, 14.3 % | 0.461 N m, 13.1 % | 1.88 N m, 53 % | 1.20 N m, 34 % |
| Sine, whole file | check | 34.16 N m | 1.62 N m, 4.7 % | 0.709 N m, 2.1 % | 95 N m, 278 % | 24.5 N m, 72 % |
| Constant speed 0.1 m/s | check | 14.63 N m | 0.746 N m, 5.1 % | 0.896 N m, 6.1 % | 6.17 N m, 42 % | 3.65 N m, 25 % |

On the S-profile, step by step (RMS, peak): the setpoint on the simulator's clock (below) 0.842 N m, 7.23 N m; the
torque feedforward off 0.711, 3.85; the speed loop's gains 0.686, 2.75; the guides' presliding friction 0.425, 2.24; the
fitted friction and constant force 0.430, 2.27; the screw's efficiency from its lead angle
(`effects.screw_lead_efficiency`, 0.990 in place of 0.9) 0.441, 2.26; its Coulomb friction fitted again 0.370, 2.19.
Before the last two steps (16 November 2026) the table's "now" read 0.430 N m, 5.3 % and 2.27 N m, 28 % on the S-profile;
0.470, 13.3 % and 1.23, 35 % on the sine; 0.895, 6.1 % and 3.59, 25 % at constant speed. Replaying from the trace's own state changes nothing here (the
S-profile starts at rest) and the whole sine's jolt from 95 to 30 N m. Since 17 November 2026 the replay runs a drive's
trace with speeds and accelerations derived per row from the rows around it (`trace::replay`), not a cubic whose
acceleration steps at every row: S-profile 0.370 to 0.368 N m and 2.19 to 2.17 N m, the sine's window 0.462 to 0.461 and
1.20 as it was, the whole sine 0.789 to 0.719 and 30.0 to 27.5 N m, constant speed unchanged. Since 17 November 2026
the motor's winding is the model's one winding: three times the data sheet's phase values, 0.36 Ω and 9.3 mH for 0.12 Ω
and 3.1 mH, and its printed voltage constant (77 V per 1000 r/min, line to line): S-profile 0.368 N m as it was and 2.17 to 2.20 N m, the whole
sine 0.719 to 0.709 and 27.5 to 24.5 N m (the jolt at its end: L di/dt against the same DC link on three times the
inductance), the sine's window and constant speed unchanged. Since 17 November 2026 the screw driven back by the load
loses by its own efficiency, THK's (1 − μ / tan β) / (1 + μ tan β) = 0.98963 in place of the driving 0.98972: the
constant speed's worst sample 3.65 to 3.62 N m, the rest within their last digit. That one sample is as sensitive to
anything near it: 3.615 and 3.616 N m with the screw's μ 1 % lower or higher. Since 20 November 2026 the coupling's damper bridges its own spring, not the
screw's too (two damped springs in series, `Converter::apply_back`), and the screw has its own damping ratio 0.04
(`screw.axial_damping_ratio`): the first alone gives 3.6156 N m at constant speed and an S-profile RMS of 0.3716 N m, both
together 3.6487 and 0.3681 (3.62 to 3.65 N m, the rest within their last digit). That worst sample lies at 14.5 s, where
the replay gives −1.1 N m against the measured +2.5 N m: a gap the damping moves by 0.03 N m, not one it makes.

**Following error**, now on the source's clock: S-profile measured 0.0411 rad RMS, simulated 0.0399, their difference
0.0022 rad (5 % of the measured); sine (0.1 to 19.9 s) 0.0078 against 0.0070, difference 0.0018 rad (23 %); constant
speed 0.0165 against 0.0153, difference 0.0039 rad (24 %). With the bench's position gain (35 1/s) and speed feedforward,
and the speed loop's gains read off the S-profile (below), not the dSPACE board's own law.

**What was changed in machine.toml, and why** (each change marked in the file):

- `effects.torque_feedforward` off. The recordings' torque command holds no term of the acceleration setpoint: regressed
  on the speed error, its integral and the acceleration setpoint, the S-profile gives −0.9 N m per m/s² where a torque
  feedforward would give +3.1. Off, the S-profile's peak difference falls from 92 % to 48 %: the bench's speed loop lags
  at the start of each ramp and overshoots, the simulated one did not.
- The speed loop's gains: 3.9 N m s/rad and Tn 14.45 ms for the estimates 6.0 and 10 ms. The recordings name them "Gain"
  166 and 69.2 without units; the S-profile's torque command regressed on the speed error and its integral, with Ki / Kp
  = 69.2 1/s, gives Kp = 616 N m per m/s at the table, 3.9 at the motor (3.3 to 4.0 with Ki / Kp free). darus-3252,
  recorded a year later, gives Kp = 175, Ki = 11 and a closed speed loop of 215 rad/s at a damping of 0.44, which on the
  fitted 0.0199 kg m² is Kp = 2 · 0.44 · 215 · 0.0199 = 3.77 N m s/rad: the same within the regression's spread.
- `effects.slide_presliding` on (new, shared physics behind its own switch, off in every shipped config): the guides'
  and nut's friction as a Dahl state ([model reference](model-reference.md#slide-presliding-friction)), σ = 2e7 N/m, an
  estimate. Without it the replay holds nothing at rest; on the S-profile 16 % of the rows are rest, and the bench holds
  −1.44 N m there. With it the replay holds −0.99 N m.
- Friction from `sim-cli fit` on the S-profile: guides' Coulomb 100 → 87.6 N and viscous 500 → 723 N s/m, the motor
  shaft's 0.3 → 0.263 N m and 1e-3 → 1.446e-3 N m s/rad, and the nut's preload 2000 → 1752 N (its drag scaled with the
  rest: the trace cannot tell the guides from the nut, both act as a sign of the speed). The constant force of +15.4 N
  as an incline of 0.225° (`slide.incline_deg`).
- The mapping (`map.toml`): the setpoint 1 ms later (`time_offset_s`), see below.
- `effects.screw_lead_efficiency` on (16 November 2026; new, behind its own switch, off in every other shipped config):
  the screw's efficiency from its lead angle, THK's (1 − μ tan β) / (1 + μ / tan β) with μ = 0.003 (an estimate, THK's
  general value): 0.990 at β = 17.7° (`the_screw_efficiency_follows_the_lead_angle_with_its_switch`, sim-components:
  0.976607 and 0.989717 by hand, and off the switch keeps `screw.efficiency` to the bit). `screw.efficiency` 0.9 is a catalogue's figure that holds the nut's preload drag,
  and the preload drags apart in the model (`screw.preload_n`, fitted): the 0.9 counted it twice, and on the inertial
  force too. Then the S-profile's fit asked for Coulomb × 1.115 ± 0.022 (the 0.9 had lifted every friction force by 1 /
  0.9): guides 87.6 → 97.7 N (breakaway with them), nut preload 1752 → 1953 N, motor shaft 0.263 → 0.293 N m.

Fitted again on the S-profile, the config lands on the measurement within its standard errors but for the mass and the
held force: viscous 764 against 773 ± 24 N s/m, Coulomb 320 against 324 ± 7 N, constant 15.7 against 15.2 ± 1.5 N;
moved mass 550 against 496 ± 3 kg, held force −162 against −242 ± 6 N (before the screw's efficiency: viscous 792,
Coulomb 325, moved mass 584 kg). The checks (re-run 17 November 2026, `sim-cli fit screw-axis --config machine.toml --trace FILE --map map.toml`):
moved mass 545 against 499 ± 135 kg on the sine, 545 against 510 ± 11 kg at constant speed (562 against 578 ± 8 and
557 against 529 ± 3 as written on 16 November, before the error became that of fitted − config and the gearbox and
friction fixes since; 555 and 572 before the screw's efficiency).

**Not changed, and why.** The moved mass: the fit says 54 kg less than the config's 550 kg (92 kg less than 584 before
the screw's efficiency). The table's 400 kg is the source's, and the sweeps put the load side at 0.0173 to 0.0214 kg m²
(427 to 528 kg at the table), so the table is not lighter; the excess is on the screw's side. Both parts of the model
that the 1.0 work named were already in it: `BallScrew::inertia` takes the shaft as a solid steel cylinder at its
nominal diameter over its whole length, ρ π d⁴ L / 32 = 0.00355 kg m² (88 kg at the table; at its root 0.00179, 44 kg),
and the efficiency acts on the whole force the screw passes, inertial force included, η driving and 1 / η driven back
(`Gearbox::loss_factor`). Neither is missing; both make the model heavier, not lighter. The efficiency was the wrong
figure (above). The shaft: the remaining 54 kg is three fifths of the shaft's 88 kg, which fits a threaded part lighter
than a solid shaft at the nominal diameter and journals thinner than it, but the source gives neither, and the three
recordings do not pin it closer (moved mass 496 ± 3, 500 ± 135 and 510 ± 11 kg). A carriage of 342 kg would make the torques fit and
would contradict the sweeps. The first mode stays at 137.02 Hz (`sim-cli modes`): the efficiency does not enter it.
Against it, the torque per force at the table (below) reads as an efficiency of 0.77 to 0.81; at that efficiency the
table alone would be 494 to 519 kg at the motor, more than the fit's 496 kg for everything, so the inertia does not
support it.
The screw's stiffness: see the sweeps. The lead: see the sweeps.

**The friction the data can tell** (`fit`, the rows after the rigid values; `friction_details`, tested on excerpts of
these files against the same estimate worked out by hand, `held_force_breakaway_and_presliding_of_the_measured_ball_screw`
in sim-components):

| Value | S-profile (fit) | Sine (check) | Constant speed (check) | Replay of the config now (S-profile, sine, constant) |
| --- | --- | --- | --- | --- |
| Coulomb, guides, nut and motor together, at the table | 324 ± 7 N | 317 ± 10 N | 427 ± 1 N (with the viscous share at 0.1 m/s) | 320, 319, 396 N |
| Viscous | 773 ± 24 N s/m | 1169 ± 211 N s/m | (one speed: in the Coulomb value) | 764, 778 |
| Held at rest after the last move | −242 ± 6 N | +231 ± 7 N | +355 ± 1 N | −162, +217, −184 N |
| Breakaway (1 to 5 % of top speed, slowing) | (too few rows) | 252 ± 1 N | (too few rows) | (−), 304 |
| Presliding stiffness σ, from the turn at reversals | (one reversal at 1 m/s², crossed in 2 ms: no row; 7.0e6 N/m from it alone) | 3.2e7 ± 0.04e7 N/m (four) | (one reversal: no row; 1.9e7 N/m from it alone) | (−), 1.8e7, (−) |

- **Guides against nut.** The motor torque cannot tell them apart. The source's nut force sensors (`F_nut_sum`, now a
  column of the CSV) can: regressed on the S-profile, the torque is 0.00829 N m per N of nut force plus 1.48 N m of
  Coulomb friction that does not pass the nut's sensors (the nut's preload drag, the screw's bearings, the motor), and
  the nut force carries 59 N of Coulomb friction from the table (the guides): 1.48 and 0.49 N m of the 1.99 N m that
  `identify` finds, 74 and 26 %. The config as fitted has 1.36 and 0.62 N m (69 and 31 %). The sensors are not
  referenced (the source says so): the nut force's acceleration term is 264 kg where the table has 400, and scaled by
  400 / 264 the guides' share is 89 N, 0.63 N m.
- **Stiction above Coulomb: none the data show.** On the sine, with the linear drive's force (below) and the inertia
  taken off by hand, the friction while slowing is 1.97 to 2.25 N m between 0.5 and 8 mm/s and 2.02 to 2.50 N m from
  8 to 65 mm/s; the breakaway row's 252 N is the drive's force at the sine's ends (−75 N at 350 mm, +67 N at 150 mm,
  both against the motion there), which the fit does not know. The S-profile has too few slow rows. So
  `slide.friction_breakaway_n` is the Coulomb level, and its Stribeck speed plays no part.
- **The turn at a reversal.** At the sine's four reversals the friction (inertia and the drive's force off by hand)
  turns from +2.0 to −1.5 N m over 80 ms and 125 µm, through zero after 13.5 to 15.6 µm: σ = 315 N ln 2 / x = 1.4e7 to
  1.6e7 N/m. The fit's 3.2e7 does not take the drive's force off (it pushes against the motion at both ends of the sine and
  moves the crossing to 7 µm); the config's 2e7 lies between, and its replay gives 1.8e7 by the same estimate (2.0e7 before the screw's lead
  efficiency). Dahl's law
  with 1.5e7 would be within 1 % of the far level after 125 µm; the measured friction is at 68 %: it approaches the
  level more slowly, a second, longer scale that one σ does not have.
- **The held force.** It is the friction holding what the speed loop's integrator had when the axis stopped, so its sign
  depends on the last deceleration: the S-profile and the constant-speed run both stop moving backwards and hold −242 and
  +354 N. The model holds the right sign and two thirds on the S-profile, the right sign on the sine, the wrong one on
  the constant-speed run (−184 N): its last stop is a ramp at 0.1 m/s whose loop dynamics the stand-in loops do not
  reproduce. On that recording the rest rows (6 %) leave 3.29 N m RMS, the moving rows 0.347 N m (2.4 %); without the
  presliding state it held nothing, and its whole RMS was 0.66 N m (4.5 %): that check is worse with the new friction.

**The setpoint's clock.** The source records the setpoint its loop compares with the position of the same cycle; a
simulated trace holds in each row the setpoint the drive worked with up to that row, the one of the cycle before. The
mapping's existing `time_offset_s` on the setpoint (1 ms here) puts the source on the simulator's clock: the following
errors then compare (above), where the simulated one was the speed times one cycle (0.047 rad at 0.3 m/s) against the
bench's 0.001.

**The velocity sweeps** (darus-3252, `velocity-sweep-s0.csv`, `-s350`, `-s700`; the frequency response from motor torque
to motor speed, computed by `tools/darus_ball_screw_to_csv.py --sweep`). `sim-cli identify --sweep` fits the two-mass
model: at 0, 350 and 700 mm a stiffness at the motor of 3240 ± 420, 2590 ± 300, 2830 ± 370 N m/rad, the first mode
(resonance) at 134 ± 13, 123 ± 10, 126 ± 12 Hz, the dip (antiresonance) at 63, 62, 58 Hz. The model (`sim-cli modes`):
137 Hz, its stiffness at the motor 3210 N m/rad (coupling 50000 N m/rad in series with the screw, nut and bearings,
8.5e7 N/m at the softest nut position), its dip 70 Hz; `fit --sweep` puts the config's dip against the measured one at a
stiffness factor of 0.80, 0.76, 0.66 if the table's mass is right. So the model's screw is as stiff as the bench near the
motor and up to a third too stiff at the far end (one stiffness for the whole stroke, `BallScrew::converter`); not
changed, as the closed speed loop under the sweep and the load side's mass (the sweeps' own 427 to 528 kg) each move
the dip as much. **The lead**: the sweeps' ratio of load side to motor side, 3.0 to 3.7, is what a lead of 40 mm gives
(table 0.0162 kg m² over a rotor side of 0.0039 to 0.0060: 2.7 to 4.2); a lead of 25 mm would give 1.0 to 1.6. This
checks the lead read from the article number without the torque fit.

**What still does not match.**

- The sine: half of its remaining difference is the linear direct drive's force. Its setpoint (`F_des_LDD`, −113 to
  +123 N, now a column of the CSV) shows in the nut force one for one (regression coefficient 1.00) and in the motor torque
  at 0.0078 N m per N; added to the simulated torque by hand, the sine's RMS falls from 0.470 to 0.233 N m (6.6 %). The
  simulator has no input for a force on the table from outside, so the replay leaves it out. (On the S-profile the same
  column's coefficient is 0.0007 N m per N, 0.10 in the nut force: the drive's compensation there did what it should.)
- The whole sine's start and end: its setpoint starts at 63 mm/s from rest and stops from it at 20 s; the bench jolts
  with 34 N m, the replay with up to 37 N m.
- The moved mass (above), and the S-profile's peak difference of 2.20 N m at the start of the return (2.62 s), where
  the bench's speed loop overshoots more than the stand-in.
- The torque per force at the table is 0.0078 to 0.0083 N m per N in both nut and drive regressions, 1.23 to 1.30 times
  the lead's 0.04 / 2π = 0.00637: an efficiency of 0.77 to 0.81, or a torque reading, sensor or lead off by that factor.
  The config's 0.9 is the makers' figure; with `effects.screw_lead_efficiency` the config uses 0.990 (above).
- The held force on the constant-speed run, and the second length scale of the turn at a reversal (above).

**Found in the tools** by this data: the replay forced a start from rest; it now runs up to the trace's own first
position and speed (`a_replay_starts_from_the_traces_own_speed`, cli: the sine from 3 s on, measured within ±3.42 N m,
jolted the replay with 86 N m before). From 1.0.0: on the constant-speed run the speed is its sign times 0.1 m/s, so
viscous and Coulomb friction cannot be told apart, and the fit reports both as one Coulomb value
(`at_one_speed_each_way_viscous_friction_is_left_in_the_coulomb_value`); `check-dataset .` inside a dataset's folder
(`the_checker_takes_the_name_of_the_folder_it_runs_in`); the inertia `identify` reads off the S-profile lies between the
bounds the source's masses give by hand (`the_measured_ball_screw_has_the_inertia_its_masses_give`).

**What this does not show.**

- One machine, a horizontal ball screw axis. Nothing about belts, gearboxes, deltas, arms or gravity loads.
- Not the drive model: the bench's speed and position loops ran on a dSPACE board, not in the S120; the simulated S120
  loops have the bench's position gain and speed gains read off its signals, not its control law.
- Not the current loop, the DC link, the motor's heating, the noise, or the screw's ratings and life.
- Friction at the speeds of these recordings (0.5 mm/s to 0.3 m/s), its turn at slow reversals, and what it holds at rest
  after a stop; not its dwell time, nor at the speeds of darus-3003's other constant-speed runs (1 to 750 mm/s, not
  brought in).
- A fit on one recording checked on two others and three sweeps, all from one bench, in 2022 and 2023.

## Hand calculations: positioning accuracy

| What is compared | Against | Tolerance | Test |
| --- | --- | --- | --- |
| ISO 230-2 figures from given deviations | Two targets, two approaches each way, by hand: x̄↑ = 4 µm, x̄↓ = −4 µm, s = √2 µm; B 8 µm, R 4√2 + 8 µm, E 8 µm, M 0, A 8 + 4√2 µm | 1e-15 m | `iso_230_2_figures_match_a_hand_calculation`, accuracy |
| ISO 9283 figures from given poses | Barycentre (2, 0) µm from (1, 0) and (3, 0): AP 2 µm, RP 1 µm; vAP √5 µm against (0, −1) | 1e-15 m | `iso_9283_figures_match_a_hand_calculation`, accuracy |
| Thermal growth of a screw | 11.5e-6 / K · 5 K · 0.4 m = 23 µm from the fixed bearing; nothing at the bearing | 1e-12 m | `a_warm_screw_grows_from_its_fixed_end`, accuracy |
| Lead error table | Halfway between (0.1 m, +4 µm) and (0.2 m, −2 µm): +1 µm; beyond the table the end row | 1e-15 m | `the_lead_error_is_interpolated_between_its_rows`, accuracy |
| Backlash in the reversal error, single axis | 8.7e-4 rad at the gearbox output on a 20 mm pulley: 17.4 µm more reversal than without backlash (simulated 18.4 µm: the rest is the guide's friction held through the belt, which ends at another force with the gap) | 10 % | `the_reversal_error_takes_the_gearbox_backlash`, sim-machines accuracy |
| Pure-stiffness deflection, screw axis standing up, no friction | 25 kg · 9.81 m/s² over screw (E A / 1 m = 7.272e7 N/m), nut 4e8, bearings 7.5e8 and the coupling 9000 N m/rad over (lead / 2π)²: 4.589 µm sag from either side (simulated 4.587 µm) | 1 % | `a_vertical_screw_sags_by_its_load_over_its_stiffness`, sim-machines accuracy |
| Thermal growth and lead error in a run, screw axis | 10 K: each target reads 11.5e-6 · 10 · x further out; with the table (0, 0)–(0.8 m, 16 µm) 20 µm/m more; nothing with `effects.load_encoder` | 1e-9 m | `a_warm_screw_and_its_lead_error_move_the_real_position`, sim-machines accuracy |
| The test on the single axis, screw axis, T-gantry and D3 | Finite figures within a millimetre; a rating gives a verdict | — | `the_test_runs_on_the_axes_the_gantry_and_the_delta`, sim-machines accuracy |

## Hand calculations: global sensitivity

`sim-cli sensitivity --method morris|sobol`, `ks.global_sensitivity` (`sim_components::sensitivity`). The estimators are
checked on functions whose indices are known in closed form; on the machines only what can be said exactly.

| What is compared | Against | Tolerance | Test |
| --- | --- | --- | --- |
| Sobol' first-order and total indices of the Ishigami function (a = 7, b = 0.1), 16384 rows | V1 = ½ (1 + b π⁴ / 5)² = 4.3459, V2 = a² / 8 = 6.125, V13 = b² π⁸ (1/18 − 1/50) = 3.3737, V = 13.8446: S = 0.3139, 0.4424, 0; S_T = 0.5576, 0.4424, 0.2437 | 0.03 absolute; bootstrap interval narrower than 0.06 and around the estimate | `sobol_finds_the_ishigami_indices`, sensitivity |
| Sobol' indices of a linear model Y = Σ a_i X_i, a = (1, −2, 0.5, 0), ranges 1, 1, 4, 3 | S_i = S_Ti = a_i² Var_i / Σ with Var_i = range² / 12: 1/9, 4/9, 4/9, 0; the value without effect exactly 0 | 0.03 (first), 0.02 (total); exact | `sobol_finds_the_shares_of_a_linear_model`, sensitivity |
| Morris on the same linear model, 10 trajectories | Every elementary effect is a_i × range: μ* = 1, 2, 2, 0, μ with the sign, σ = 0 | 1e-9 | `morris_on_a_linear_model_gives_the_slope_times_the_range_with_no_spread`, sensitivity |
| Morris on the Ishigami function, 200 trajectories | On the grid (−π, −π/3, π/3, π) every step of x2 changes 7 sin² x2 by 7 · 3/4: μ*2 = 5.25 / (2/3) = 7.875; x3 acts only with sin x1 ≠ 0 (half the steps): μ*3 ≈ ½ · 0.1 · (π⁴ − π⁴/81) · 0.866 / (2/3) = 6.25 and σ3 > 1, though S3 = 0 | μ*3 > 1, σ3 > 1, μ*2 > μ*3 | `morris_sees_the_interaction_of_the_ishigami_function`, sensitivity |
| Morris points | Every point on the four-level grid, every step moves one value by 2/3 of its range, every value once per trajectory | 1e-12 | `every_morris_point_lies_on_the_grid_and_every_step_moves_one_parameter_by_the_step`, sensitivity |
| Determinism | The same seed gives the same points and indices, another seed other points; on the single axis the same rankings on one thread and on five, to the bit | exact | `the_same_seed_gives_the_same_points_and_another_seed_others`, sensitivity; `the_same_seed_gives_the_same_ranking_on_one_thread_as_on_many`, global_sensitivity |
| Single axis: slide friction against the lubricant's viscosity (which only the bearings' life reads) | RMS torque: both frictions μ* > 0 with μ = μ* (more friction, more torque in every step); the lubricant μ* = σ = 0 and Sobol' S_T = 0 with interval (0, 0) | exact | `friction_moves_the_rms_torque_and_the_lubricant_does_not`, global_sensitivity |
| T-gantry, Morris 10 trajectories: the slackest belt section's margin against the pretension (200 to 400 N) | The section's tension rises one to one with the pretension: μ* = 200 N, σ ≈ 0 (found 200.0 N, σ 0.002 N) | as found, shown in the user guide | run by hand, not a test |
| Python against the command line | The same rows and μ* | 1e-3 relative | `test_global_sensitivity_matches_the_cli`, sim-py |

## What is not validated

- **The delta picker D3's geometry, masses and stiffnesses.** Autonox keeps them behind a registration; the arm lengths are scaled from Williams' estimate of an ABB IRB 360, the masses estimated. Its cycle time is compared with ABB's and Omron's printed figures, not asserted (they round the corners and run faster). Its forearms are rigid rods (their spin about their own axis with an estimated inertia), its upper arms' bending stiffnesses (in and out of their planes) estimates for a carbon tube; neither is compared with a measured delta.
- **The overhead crane against a real crane.** No crane has been measured: its masses, the rope's EA and damping, the
  rolling coefficient, the brake's torque and the drives' tuning are estimates. As shipped the load is one point mass on a
  massless rope, the bridge one drive, the drives rigid, and pick and set-down are quasi-static. The switches
  `effects.rope_mass`, `hook_block` (hook and load a double pendulum), `joint_elasticity` (elastic drives),
  `bridge_skew` and `floor_contact` (the load standing on the floor, a snatch lift, a slack rope at set-down) add those
  effects; each is checked by its own tests, not against a crane (the hook block's sling is a rope that goes slack on the
  floor). Not modelled either way: the rope's bending at the sheave and its twist.
  The residual sway is the amplitude of a free
  pendulum at the dwell, not what a sway sensor of a real crane reads; the input shaper is checked against the model's
  own pendulum only. The SciPy reference checks the integration, not the equations.

**Against a real machine: one public measurement.**

- One measured machine has been compared with the simulation: a ball screw axis of ISW Stuttgart, three recordings and three velocity sweeps ([above](#against-measured-data)). The "Real traces" check of the project plan (stage D4) on the project's own axes does not exist.
- The test runs on a real axis (stage D2) have not been made.
- Trace import, replay and fit have seen one real machine's recordings, converted to CSV by a script; a drive's own export (Startdrive, TwinCAT, CODESYS) with its column names, units and sampling is still untested.
- The following error is compared on that one bench only, whose loops ran on a dSPACE board; drive vendors do not document their internal filters.
- The screw's efficiency from its lead angle (`effects.screw_lead_efficiency`): μ = 0.003 is THK's general value, not measured on any screw; the bench's fit prefers it to 0.9 through the moved mass and friction, which does not measure the efficiency by itself. Driven back the screw loses by its own efficiency, THK's back-drive formula (1 − μ / tan β) / (1 + μ tan β) with the same μ (0.98963 at the bench's lead against 0.98972 driving). The screw shaft's inertia is a solid cylinder at the nominal diameter over `screw.length_m`, journals and threads not told apart; the bench's fit leaves 57 kg of it unexplained.

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
- The temperature feedback: the viscosity law is a mineral oil's, the share of a gearbox's loss that follows it and the belt's loss of stiffness per kelvin are estimates; no gearbox maker's curve of no-load torque over temperature was found.
- The belt tooth jump: the climb-out factor is an estimate (no maker's ratcheting chart was read), the ridge the sliding belt rides over is a sine of the climb-out pull, and a tooth pressed back into its groove by the wrap's tension is not modelled. Only the threshold and the bookkeeping are checked by hand, not a real jump.
- The belt spans: string frequencies only, no amplitude; the 10 % band is an estimate.
- Torque saturation: the shape of the curve between zero and maximum current (a square law) is an assumption; only the end point is from the catalogue.
- The DC link: checked against energy bookkeeping by hand, not against a drive. The S210's real chopper behaviour is not known.
- Drive delays and filters: checked against control theory, not against a drive.
- Sensor noise: Gaussian by assumption.
- The rule that scales the velocity gain for a part by name and in the motor search: a rule of thumb, checked on the EXCT-30 for the 1FK2 only.
- The holding brake: a spring up to the brake torque, then slipping with it. Checked against its own hand calculation, not against a brake.
- The shared DC link: energy bookkeeping by hand, with the capacitances simply added. Not against an S120 line-up; a Line Module's own capacitance and the busbars are not modelled. The regenerative supply is an ideal power limiter with one efficiency, checked by hand, not against an Active Line Module.
- Splines: the geometry and the axis rates are checked, not that this is the curve a given controller would lay through the same points.
- The modes the T-gantry reports for the input shaper have the frame mode, the X-Z belt coupling, the Z axis's own mode on the belt and the belt's runs' mass (pulleys held), with `effects.beam_vertical_mode` the beam's vertical mode, and leave out the control loops. Under the servo the swing is a little faster: at z = 0.15 m with the frame off a shaper at 36 Hz leaves least of the swing after a traverse (0.111 of it), where the machine reports 34.59 Hz (0.121 left).

**The SciPy reference** does not include the leaning Z axis or any effect added after phase 2 (see [above](#independent-reference)).

**Effect views.** The torque parts are a definition: the rest is what inertia, gravity and friction leave, so the sum cannot be wrong, and inertia is taken at the load's acceleration (see the model reference for why). The hand calculations check the three named parts where they can be worked out; a measurement could not check them, since no sensor sees a torque split into parts. The energy budget's electrical side is checked against itself (the integration's error), not against a measured supply.

**Drive stability.** With the winding current integrated together with the mechanics (12 November), the old velocity gain of 0.08 N m s/rad left the default configs little margin (single axis 1° and 0.3 dB, T-gantry 12° and 2 dB, measured with `sim-cli margins`); the old drive model had hidden this behind a numerical phase lead. The default configs now have 0.025 (single axis) and 0.04 (T-gantry), with at least 35° and 6 dB in every one-at-a-time bounds run, with friction on and off; `across_the_bounds::*` (control, a test per config, run by CI; `the_shipped_velocity_loops_keep_their_margins` checks the configs as they are) holds that, and since its retuning from 0.6 to 0.45 the EXCT-30 too (measured 39.4° and 8.46 dB on 17 November, `sim-cli margins t-gantry --config crates/machine-t-gantry/exct-30.toml --bands --set effects.motor_friction=false --set effects.guide_friction=false`; 38.6° and 8.3 dB before; it had 29.6° and 5.8 dB). The margins are measured on the simulated loop, at one amplitude, with the machine holding its start pose; 60 points from 10 Hz to 2 kHz (and below 10 Hz to a crossover there, to 1 Hz) can miss a sharp resonance between them. They are not checked against a real drive's measurement function. The sampled bands (`--samples`) are not run for the margins.

**Combinations.** Each effect has a test of its own. That all effects together give the right result is checked only by energy conservation (losses off), step-size convergence and determinism, not against an independent number.

**Error bands** vary one parameter at a time, or with `--samples` all at once in a Latin hypercube. The sampled mode covers interactions as far as its runs reach: it is tested on one case with a known answer (two masses), and no number of samples proves a worst case. On the EXCT-30 it found one that the one-at-a-time mode cannot: the friction model blowing up with two of the X guide's values at their bounds together (fixed, see the model reference).

**Front ends:**

- OPC UA: tested against a client written with the same library, the secured endpoints (`interop.rs`) and the pacing and client jitter figures (`plc.rs`, `soak.rs`, [PLC](plc.md#real-time-pacing-and-jitter)) included. Not against a Siemens, Beckhoff or CODESYS controller; the jitter a real PLC brings is not measured.
- FMI: checked once (13 November) with an independent tool, fmpy 0.3.32 (in the project's `.venv`), on the win64 FMUs of the T-gantry, the single axis and the EXCT-30 config (English and German): `fmpy.validation.validate_fmu` (FMI 2.0 XSD, variable names, model structure, start values, causality and variability, units) found no problem in any; `simulate_fmu` in co-simulation ran each with no log message. At rest, every output agreed with `sim-cli run` to the 9 digits the CLI prints (relative 5e-10). A setpoint ramp (X 0.1 m in 0.5 s; on the EXCT-30 also Z) ends where `sim-cli run --trajectory` with the same points ends, within 16 µm of position; on the way, the FMU's setpoint arrives one setpoint interval (10 ms) later and is interpolated linearly, the trajectory's with cubic Hermite, so the setpoints differ by up to 0.2 mm shifted by that interval and peak torques by up to 0.9 N m (single axis 2.6 N m of 2.8): a difference of the input path, not of the model. fmpy's error paths: a communication step of 0.1 ms (not a multiple of 62.5 µs) refused with `fmi2Error` and its message logged; a setpoint out of range logged as a warning per step. The same check is now `tools/check_fmu.py`, a step of CI on Windows and Linux (linux64 binary). Run in OpenModelica 1.27.1 through OMSimulator ([FMI export](fmi.md#openmodelica)); not imported into Simulink or Dymola.
- App: what the app computes is tested without its window, against the library functions the command line uses (`sim-app/tests/app.rs`). The window itself is driven by synthetic input (`sim-app/src/window_tests.rs`, run with `cargo nextest run -p sim-app -E 'binary(sim-app)'`): the app as `main` builds it, Bevy without winit and without a renderer (so the tests need no screen or graphics card and run in CI as anywhere), fed the messages winit sends for cursor moves, mouse buttons, the wheel, keys, typed text and window resizes, its widgets found by their labels in egui's accessibility tree. They cover the orbit camera (a drag turns the view by 0.005 rad a pixel about its target; the wheel zooms by 0.9 a notch within 0.2 to 20 m, a touchpad's pixels 50 to a notch; the right or middle button pans with the point under the pointer staying there; a drag that starts on a panel or a control turns nothing; the presets front, side, top and angled put the camera on the stated side with the machine's box in view; a resize keeps it in view), the top bar (pause, speed, units, theme, machine and language lists), the motion tab (start cycle, stop, drives off and on, a target typed, move), the parameter panel (a value typed reaches the running simulation and stays; a word for a number, a key the config does not have and a value the simulation refuses are each refused with their message and the old value kept; a drag on a number changes it), an effect switch, the config file field (a file that does not exist is said and changes nothing), every tool tab's run button (cycle with error bands, sensitivity, optimiser, motor search, path from `examples/corner.path`, trace from a missing file, frequency response, tuning), the keys (Space replays and the arrows step the timeline over the view, but not while a field has the keys, where a space is typed; Tab moves to the next field), five clicks on a long analysis start it once, switching the machine during an analysis leaves no result of the old one, and closing the window during the optimiser exits within seconds. Every label of the panels changes with the language but a listed few that German shares (units, "Motor", "Pause", "Plan", "Live", "Position"); no widget overlaps another or sticks out of its panel in English or German at 1600 × 900 and 1280 × 720 (as far as the accessibility tree tells: a wrapped label's box is not its text, and plots and the 3D view's own labels are painted, not widgets). Not covered by the tests: the file dialogs (Open…, Save as…, Export, the sizing report and a plot's picture), which the tests cannot close; dragging a panel's edge; the right-click menu of a plot. How it looks was checked by eye (15 November) on pictures saved by the app's screenshot hook (`SIM_APP_SCREENSHOT`, with `SIM_APP_SIZE` and `SIM_APP_SETUP` added for this) of the T-gantry in every tools tab and setup tab, English and German, 1600 × 900 and 1280 × 720, with the motion, a planned path and the overlays: the defects found were fixed (texts and controls running out of the view or the window in German, the timeline's buttons, the scopes panel shrinking with the kept runs, plots with 0 alone on their axis across, the part labels' -0, the config tab's part names wrapped to the end of the line before their fields, the motor search's list names after their lists where the fields' names come before). Not fixed: at 1280 × 720 the view's controls cover much of the machine, and some tools tabs (motor search, path, tuning) widen the tools panel by up to 160 pixels. Since then the planned path is drawn as a wide amber line with the travelled path as a cyan line over it; before, the travelled line lay at the same depth, lost the depth test to the plan and was never seen. Only the T-gantry was looked at; the other 21 machines' 3D views were not. How the motion looks over time was seen in single pictures, not watched.

**Dataset reports** (`sim-cli fit --dataset --report`, [datasets/RESULTS.md](../datasets/RESULTS.md)). The report
reproduces the figures of [Against measured data](#against-measured-data) for the ISW bench (tested), and nothing more:
it has one measured dataset to run on. It takes the first trace recording as the fit and the rest as checks, whatever
they are; it does not cut a recording (the sine's 0.1 to 19.9 s window above is not in it). The fitted values are
lumped at each axis and are not mapped onto the config's `[bounds]` keys: the report lists the bounds beside them, it
does not say whether a fit lies inside them. The model's first mode is taken at the config's start pose only, not at
the position of each frequency response. The contribution form and guide have not been used by anyone outside the
project yet. The report's texts are in English only.

**Warm-started kinematics (16 November 2026).** The hexapod's and the platform on cranks' dynamics start Newton's forward kinematics from the pose at the nearest point of a grid of joint positions (1 mm of leg, 1 mrad of crank), and the cable robots' elastic `forward` from the pose and Jacobian at the nearest point of a 1 mm grid of free lengths, for speed; the pose is a function of the joints alone, not of what was called before (an earlier version started from the last call's pose, which made a run, its replay and an analysis with and without its effect views differ at the rounding, and a fit of a trace onto its own run diverge) (see the [mechanisms](mechanisms.md)). The tests `the_default_cycle_keeps_its_figures` (hexapod, platform on cranks) and `the_default_cycles_keep_their_figures` (both cable robots) compare each motor's peak and RMS torque, the energy it drew and fed back, and the tool's path, tracking and angle errors over the default cycle with the figures recorded before. The cable robots' moved by up to 1.5e-5. The hexapods' moved by up to 1.1e-4, about as far as moving only Newton's start by 1e-9 to 1e-7 m in the old code moved them (up to 7e-5): the closed loop amplifies rounding-level changes of the pose that far. The tests allow 1e-3, also for other platforms, whose maths library rounds sine and arc tangent differently in the last bit; that difference was not measured (no Linux run here). Not checked: that the warm start converges to the same pose far from the default cycle (it falls back to the old start where it does not converge).

**Catalogue data.** Cross-checks find values in the wrong row or column. They cannot find a number misread into another plausible number where nothing ties it to a second value. The misprints found are listed in the [catalogue](catalogue.md#misprints-and-inconsistencies-found-in-the-vendor-documents) document; there may be more.

**The validation dashboard.** It shows the outcome of each test this page names and, for the tests that call `record_check`, their values; it checks nothing these tests do not. Most rows carry no recorded values, the Python tests are not run for it, and a row whose tests did not run shows as "not run", not as passed ([dashboard](dashboard.md)).

**Positioning accuracy** (`sim-cli accuracy`). The ISO figures are computed as the standards define them and checked by hand; what the simulated axes give is not compared with a laser measurement of any machine. Thermal growth and the lead error are a map applied to the tool pose at rest, not part of the dynamics; the belt's expansion coefficient is an estimate (no belt datasheet gives one); the frame's growth, a gradient along the screw and the screw's own heating from its nut are not modelled; the screw's stiffness is the one at its softest place over the whole stroke (the model's shortcut), so a sag that grows along the screw does not show. The D3's systematic deviation of up to 0.57 mm along y comes from its estimated stiffnesses and is not checked against a real delta. The machine tests run one or two approaches per target and direction, not ISO 230-2's five.

**Global sensitivity** (`sensitivity --method morris|sobol`). The estimators are checked on analytic functions, not on
a machine whose true indices are known: on the machines the tests check only determinism, a value that cannot act and
one that must. The ranking is only as good as the `[bounds]` it starts from, which are mostly estimates, and it ranks
only bounded values (the shipped T-gantry bounds no mass). Sobol' uses plain Monte Carlo rows, not a quasi-random
sequence; with a few dozen rows its first-order indices scatter around 0 (below 0 too) and the bootstrap intervals are
wide; no rule says how many rows are enough. Morris' μ* is in the result's unit per whole range of the value and does not
say which share of the variance a value explains. A result missing from a run (a check only some values bring up) or not
finite in one (a life where nothing wore) is left out. Integer values (pole pairs) are rounded, so their steps are not
the grid's.

**Release packaging and benchmarks.** The macOS build (universal tarball, darwin64 in the FMUs) is made on GitHub's
runner and was never run on a Mac; its FMU binary is not run by `check_fmu.py`, which runs on Linux. The attestations
and `SHA256SUMS` were not produced by a real release before 1.0.0 was tagged. The benchmark check
(`.github/workflows/bench.yml`) compares each run on main with the one before, on shared runners that differ by 10 to
20 % between runs: it warns only, and a slowdown spread over several pushes of under 15 % each is never warned about.

**Indexing tables** (`geneva-drive`, `cam-indexer`). The Geneva and modified-sine kinematics are checked against the
textbook formulas, not against a measured indexer; every mass, inertia, stiffness, play and rating is an estimate. The
pin and the rollers carry the wheel's torque without friction; the cam's pressure angle is in the roller's force
(`indexer.cam_radius_m`, an estimate: 35° at the fastest point, 2.1 % more peak contact force), checked by hand
(`the_cam_s_pressure_angle_raises_the_contact_force_by_one_over_its_cosine`). The locking disc holds the wheel rigidly:
its play gives the index accuracy, but the wheel does not rattle in it, and the lock's own load is not judged. As
shipped the table's twist on its shaft is worked out beside the dynamics for an index run at the driver's speed of the
moment (exact under `cam.run`, an estimate on a servo cycle that speeds up through the index) and does not act back on
the drive; `effects.table_twist` makes it a joint of its own, the table on its spring and damper
(`the_table_twist_is_a_joint_of_its_own_with_the_matrix_of_the_two_inertias`). The seven-axis arm's joint-limit option is checked for the pose it reaches and the
optimum it settles at, not for how far from its stops a whole cycle keeps the arm.

**Loop response (velocity-loop Bode and Nyquist, refined sweep).** The refinement is tested against a two-mass system worked out by hand (`refinement_finds_a_sharp_resonance_the_coarse_sweep_misses`: resonance 318.31 Hz with 1 % damping, missed by a factor of four by the 60 points, found within 0.2 % and 3 % of its peak) and the margins, phase crossover and closed loop against a hand-calculated loop (`phase_crossover_and_closed_loop_of_a_hand_calculated_loop`). Not checked: a resonance whose peak and dip both fall between two neighbouring points of the 60 (and is not one the machine reports in `modes`) shows no fast change and is not refined; the plant is the air-gap torque to the motor's speed with both sampled once per simulation step, so its phase near the top of the range is uncertain by up to half a step (31 µs, 22° at 2 kHz) and is not checked there against a hand calculation; and the plots themselves are checked by the window tests for their labels and results, not by eye against a measured Bode plot of a real drive.

**App layout (16 November).** The window test at 1280 x 720 checks the panels' widths and how much of the 3D view the
floating controls leave free, by the accessibility tree without a screen; it does not check what the machine's parts
fill of the view, nor other window sizes than 1280 x 720 and the default 1600 x 900. The 3D view of each machine was
looked at in screenshots at 1280 x 720 at its start pose, not in motion.

**Mover track (`mover-track`).** Checked by hand calculations and the standard suite only; no XTS or ACOPOStrak has been
measured. Not published, so estimated: the force constant, coil resistance and inductance, the magnets' attraction, the
pole pitch and cogging, the rollers' resistance and spread, how the 267 mm curve splits into arc and clothoid, the
segments' thermal resistance and time constant. Not modelled: the rollers' resistance changing with the curve's load
(it is the straight's level throughout), the mover's yaw and its rollers' own compliance in a curve, the coils'
commutation across a segment joint (a mover's force is the same wherever its plate lies), the movers' coupling through a
shared supply beyond the DC link, the gap in a curve measured across the chord (it is measured along the centre line),
the tilt of a mover on a vertical track (the track lies level). The segment's settled rise is the loss of about the last
pass times its thermal resistance, not a thermal network of coils, frame and air. The station error reads the cam's
master as the controller last set it (once a profile cycle); a stop and restart part-way through an index leaves the
setpoints off a station, which the check then counts.
