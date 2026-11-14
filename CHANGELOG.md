# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the project uses
[Semantic Versioning](https://semver.org/spec/v2.0.0.html). Details, test names and the numbers behind each
item are in [PROGRESS.md](PROGRESS.md).

## [Unreleased]

### Added

- `verdicts`, `search` and `optimise --path FILE [--time-optimal]` (and Python's `path=`): the path or program is
  judged, searched on and shortened in place of the cycle, through an optional `[path]` config table.
- Corners next to an arc or a spline are rounded within `profile.corner_tolerance_m` (G2), no longer stopped at; the
  optimiser tries a lift with its two corners rounded (`cycle.segment[i].corners_m`).
- The time-optimal plan takes each motor's shaft friction, gearbox drag and (machine files) guide friction off its
  torque at the planned speed (`Machine::rigid_friction`).
- G-code F is the speed along the path; RAPID speeds (`v100`, speeddata) and KRL `$VEL.CP` are read, and on arms in tool
  space RAPID's quaternions and KRL's A, B, C set the orientation.
- Drive monitoring and fault reactions (`effects.drive_monitoring`, off by default): each drive trips on its
  following error window (S120 p2546), a standstill window (p2542/p2543), a motor blocked at its torque limit (F07900)
  and optionally a band around the expected torque, and every drive then stops by its own `drive.fault_reaction`
  (`off1`, `off3`, `sto`, `current_limit`; per motor as `[drive.<id>]`). `Controller::fault` reports the monitor, the
  drive, the time and each axis' stopping distance; `sim-cli run` and the app's messages say them.
- Capsules as collision bodies (`Machine::collision_capsules`, `sim_core::Capsule`) with an exact segment-to-box
  clearance; the D3's rods and arms and the D4's telescope are capsules.
- Obstacles take a contact `stiffness_n_m` and `damping_n_s_m`.

- Catalogue: gearboxes of WITTENSTEIN (SP+, TP+), Neugart (PLE, PLN, PLFE), Apex (PEII, AF) and Harmonic Drive (CSF,
  CSG, SHG); HIWIN ball screws (FDW, FDC, FSH) and guides (HG, EG); Siemens 1FN3 and Beckhoff AL8000 linear motors;
  Gates belts; Festo EGC/ELGA and Rexroth CKR single belt axes; motor shafts of Siemens, Beckhoff, SEW and Yaskawa.
  `search --gearboxes` takes a maker or series (`wittenstein-sp`), `search --axes` tries screws, guide sets and linear
  motors on screw, rack and linear motor machines; `catalog::gearboxes_for_motor` and `catalog::axes_for` for the search.

- Machines `hexapod` (crate `machine-hexapod`): a Stewart-Gough platform on six ball-screw cylinders, closed-form inverse
  and Newton forward kinematics, dynamics through the platform's twist, joint-angle, stroke and singularity limits, each
  leg's screw judged under the leg's force (buckling, static load, speed, life).
- Machines `cable-planar` and `cable-spatial` (crate `machine-cable-robot`): cable robots on 4 and 8 winches with pulley
  kinematics, a QP tension distribution within each cable's band, the wrench-feasible workspace
  (`error-cable-wrench-infeasible`), elastic cables (EA / l, `effects.cable_stretch`, `cable_damping`,
  `winch_friction`), natural frequencies on the cables, and the checks `cable-max-tension` and `cable-min-tension`.
- FMI: inputs `command.cycle`, `command.target.<axis>` with `command.move`, and `command.path` / `command.path_file`
  with `command.run_path`, which start the cycle, a move and a path as `sim-cli run --cycle`, `--move` and `--path`
  do; log categories `logEvents`, `logStatusWarning`, `logStatusError`, `logAll` in the model description, switched by
  `fmi2SetDebugLogging` / `fmi3SetDebugLogging` (which did nothing before).
- FMI: an FMU state holds a snapshot of the simulation (`Sim::snapshot`, `Sim::restore`) and is set at once instead of
  replaying every step since initialisation; a serialized state still replays (format `sim-fmi state 2`).
- Python: `Config(machine_file=...)`, `sweep`, `optimise`, `fit` and `identify`, as the command line has them.
- CI: clippy of sim-py in the job "python", and the CAD mass tests (`tools/cad/test_cadmass.py`) with cadquery-ocp.
- Python: `step`, `bode`, `tuning`, `margins`, `modes`, `sensitivity`, `replay`, `compare`, `report`, `check_dataset`
  and `fmu`, `verdicts(bands=True, worst=True)` and `sizing(motor_id=...)` (with the column `winding_rise_k`), as the
  command line has them; every command of `sim-cli` is now a function.
- FMI: the parameter `time_optimal` (paths timed as `--time-optimal` does) and the input `command.trajectory_file`
  (poses at times, as `--trajectory`); `command.path_file` is read by its extension as `--path` reads it (G-code, KRL,
  RAPID, robot moves as CSV). `tools/check_fmu.py` checks a time-optimal path against `sim-cli run --path --time-optimal`.

- Search: the wide table (cycle time, energy per cycle, thermal and drive current utilisation, following and settling
  error, inertia ratio load/motor, failing checks, what the catalogue left to estimates, one `u_<check>` column per
  check), `--rank-by` (cost, margin, energy, error, thermal, inertia, drive or a weighted mix), the non-dominated front,
  `--drives paired|neighbours|all` (the next drive sizes by rated current), a duty mix (several `--config` with
  `--weights`: worst result per check, heat and energy over the mix) and `--robust N` (the best ones judged again at
  their worst corner). Python `sizing` gives the same columns and options.
- `sim-cli transmission`: a sweep of the gear ratio or pulley radius with the inertia ratio and the margins, the classic
  optimum, and the catalogue gearboxes around the best ratio.
- `sim-cli compare-kinematics TASK.toml --machines ...`: one tool-space pick-and-place task (examples/pick-place-task.toml)
  mapped onto each machine's cycle, unreachable poses refused with the reason, and the best combination per kinematics.

### Changed

- `effects.crash`: an obstacle pushes back on a part in it with its contact stiffness and damping, blocking the axes
  in contact, instead of stopping the whole machine dead and switching the drives off on 1 µm of creep; the drives'
  monitoring trips them. The budget's `crash` row is the work of the pushes, integrated with the motion. The contact
  record has the largest force and the impact energy.
- The conveyor's slats are drawn square across the belt however it runs.

- FMI: a serialized FMU state carries the simulation's snapshot as bytes (format `sim-fmi state 3`: the FMU's version,
  then `postcard` 1.1.3 over serde 1.0.229 derives on the snapshot's types in sim-core and sim-components) and is set
  at once, however long the simulation ran; it no longer replays every step. States of format 2 or of another version
  are refused with a message naming it.
- Python: `fit` and `identify` return what they found with the fits that found nothing in `errors`, where they raised
  at the first; `sizing` takes the config's own `parts.motor_supply` where `supply` is left out, as `search` does.

- A gearbox by name also sets its efficiency torque, no-load torque and stiffness curve where the config has the keys;
  `-r` after the name takes the reduced backlash. Siemens' SP100 is read, on 1FT2 motors only.
- A linear motor machine's search tries linear motors in place of rotary ones.
- FMI 2.0 log messages go to the logger as the format string with `%` doubled, not as an argument of `"%s"`, so loggers
  that print the format as it is (fmpy's Python callbacks) show the text.

- Gearbox no-load running torque over speed and temperature (`effects.gearbox_drag`, `gearbox.no_load_torque_nm`,
  `no_load_speed_rpm`; Palmgren's (nu n)^(2/3)) and with it an efficiency map that falls at part load, at speed and
  cold (`gearbox.efficiency_torque_nm`); catalogue values from Neugart, WITTENSTEIN and Harmonic Drive.
- Motor shaft breakaway along a Stribeck curve (`effects.motor_stiction`, `motor.friction_breakaway_nm`,
  `friction_stribeck_rad_s`); with `effects.thermal_feedback` the shaft's seals follow the gearbox's lubricant
  (`thermal_feedback.seal_viscosity_exponent`).
- Gearbox lost motion: a band over which the flanks' stiffness rises and a hysteresis (`effects.gearbox_lost_motion`,
  `gearbox.lost_motion_band_rad`, `hysteresis_rad`); Harmonic Drive's three-segment stiffness for strain wave gears
  (`effects.gearbox_stiffness_curve`, `gearbox.stiffness_t1_nm`, `_t2_nm`, `_k2_nm_rad`, `_k3_nm_rad`).
- `catalog/siemens/gearbox_sp.csv`: Siemens' SP100 precision planetary gearbox (WITTENSTEIN SP+ 100), two stages,
  with its reduced backlash (schema column `backlash_reduced_rad`).
- `search --gearboxes` offers gearboxes on a direct drive too (ratio 1), around each motor's classic optimum ratio,
  beside the motor without one.
- `gearbox-radial-load` is judged per motor against that motor's own `ratings.shaft_max_radial_load_n`; once for the
  machine only where no motor's shaft reports its own radial load.

### Changed

- Delta pickers D3 and D2 (and `machines/planar-delta.toml`): the SP100-05-20 with reduced backlash (typical
  2.25 arcmin, bounds 1.5 to 3) on the 1FT2105-4AF, with the new friction and gearbox effects on; the D2's velocity
  gain 0.24.

- Machine files: a linkage node of three arms is the spatial delta (`machines/delta-d3.toml`, the D3 crate's machine);
  a `scissor` node and `machines/scissor-lift.toml`, a two-stage scissor lift table on a ball screw; a rotary node's
  bearing friction (`friction_coulomb_nm`, `friction_viscous_nm_s_rad`, with `effects.guide_friction`); a rack
  pinion's radial load on the gearbox shaft (`rack.<motor>.radial_load`, checked as `gearbox-radial-load`);
  `effects.belt_wrap` beside a screw, rack, linear motor or rotary (`belt_wraps` gives NaN for a motor without a
  belt); a worm's efficiencies follow its temperature with `effects.thermal_feedback`.

### Changed

- The D2 and D3 crates are the linkage machines of `sim-netlist` read from their tables: one implementation for crate
  and file (the D3's model moved to `sim-netlist/src/spatial.rs`). A linkage's gearbox warmth is named after its
  loss, `gearbox_<motor>`.
- The 3D view draws a machine file's belts round their pulleys and idlers, from tangent to tangent.

- Motor model: `effects.iron_loss` (iron loss from Siemens' stall and rated data, on in the shipped configs),
  `effects.magnet_temperature` (torque constant over the magnets' temperature, off), `effects.braking_resistor_heating`
  (the braking resistor's temperature, on); the drive's thermal image in two nodes, heat sink and chips
  (`ratings.drive_chip_share`, `drive_chip_time_constant_s`); the d current follows its setpoint with the current loop's
  time constant. New checks `motor-s1` (RMS torque at the mean speed against the S1 curve) and `motor-speed` (peak
  speed against `drive.max_speed_rad_s`); new signals per motor `d_current`, `housing_temperature`,
  `drive_overload_image`, `drive_overload_chip`, `iron_loss`, `braking_resistor_temperature`.
- Catalogue: thermal time constants of the 1FK2 from the S210 operating instructions (`motors_s210_oi.csv`, with the
  230 V windings), the 1FK2/1FT2 encoders (`encoders_1fk2_1ft2.csv`), the S120 DC link trip (`dc_link_trip_s120.csv`);
  motor options `parts.motor_brake` and `parts.motor_encoder`; `drive.max_speed_rad_s` and `motor.stall_current_a` from
  the catalogue; a 1FK2's inductance estimated from Siemens' short-circuit braking table.

### Changed

- `effects.field_weakening` is on in the shipped configs: the torque-speed curve is the d-q model's at the voltage
  limit, not the linear fade from rated to no-load speed.
- The line module (or an S210's rectifier) loses `ratings.line_module_efficiency` when drawing too; the key is read
  whenever `effects.dc_link` is on. An S120 Motor Module by name switches `effects.shared_dc_link` on, an S210 off.
- The d current's setpoint and u_d use the winding's warm resistance.

- A compliant base under every machine and machine file (`effects.base_compliance`, `[base]`, off): up to six
  spring-mass directions taking the rate of change of the momentum of everything that moves on it
  (`Machine::frame_momentum`, written for every machine), the tool moving with it (`Machine::base_shift`); signals
  `base.<dof>`, its modes in `modes`. Typical frame stiffnesses as estimates in `[bounds]`.
- The URDF arms' links bend, one Euler-Bernoulli cantilever mode each (`effects.link_bending`, `[link_bending]`, off;
  `sim_components::beam`, `chain::Mechanism::flex_modes`); signals `link.<n>.bend`.
- T-gantry: the Z axis's bending (`effects.z_axis_bending`) and the X beam's sag under the carriage
  (`effects.beam_sag`, `[beam]`), both off.
- D3 and D4: the arm tube's own share of the bending modes and the elbow block's twist (`effects.arm_tube_inertia`,
  with `link_elasticity`; off).

- Arms: generic tool-space kinematics for any URDF chain of 4 to 7 joints (`kinematics.tool_axes`, damped least squares,
  warm-started from the last solution, a seven-joint arm resolved towards its home posture in the Jacobian's null
  space); example configs `config/arm5-tool.toml` and `config/arm7-tool.toml` with `urdf/five-axis.urdf` and
  `urdf/seven-axis.urdf`.
- A palletiser (`palletiser`, `urdf/palletiser.urdf`): four motors, a parallelogram keeping the flange level as
  `arm.joint_coupling` (the URDF's joints as sums of the motors'), shipped in tool space with catalogue motors.
- `arm.payload_inertia_kgm2`: the payload's own inertia; `effects.payload_swing` (`arm.payload_swing_*`): a payload
  swinging on the flange, two joints no motor drives (`chain::Mechanism::passive_joints`), signals `swing.<k>.angle`.
- Each arm joint's own motor, drive, gear and ratings (`[motor.<k>]`, `[drive.<k>]`, `[gearbox.<k>]`,
  `[strain_wave.<k>]`, `[ratings.<k>]`, `[parts.<k>]`); the shipped arms' wrists sized with smaller catalogue motors.
- `ratings.settling_tolerance_rad`, the check `settling-angle`, and the turn's own path, tracking and dwell errors
  (`path_angle_error_peak_rad`, `_rms_rad`, `tracking_angle_error_peak_rad`, `dwell_angle_error_peak_rad` in
  `sim-cli analyze` and the app).

### Changed

- A line of a tool whose axes are x, y, z, rx, ry, rz turns its orientation by slerp (`path`, `rigid::slerp`).
- The arms' orientation in tool space is the turn from the flange's orientation at zero joints (the base frame's on
  the shipped six-axis URDF): a URDF whose flange points down at zero is no longer refused everywhere.
- The six-axis arm's tool-space limits: the orientation range around pointing down, 0.9 m/s and 1.5 rad/s, so the axis
  limits ask no motor beyond its rated speed (it warned "Motor joint 5: 370.5 > 314.2" and more).
- `[strain_wave]` and `[joint]` are per-motor tables too (`catalog::PER_MOTOR_TABLES`); a config of a registered machine may
  give a motor its own tables though the default config has none for it (`sim_components::schema`).

### Fixed

- The arms in joint space leave the Coriolis and centripetal torques out of the feedforward and the time-optimal plan
  no longer; `Machine::feedforward_torque_moving`'s doc no longer calls the default exact where the motor angles are
  linear in the tool pose.
- The tool's error metric added metres and radians (the D4's path error 3.53 mm with its 90° turn, 1.53 mm without);
  the place's errors and `settling` take the axes in m only, the turn's its own in rad.
- `path::plan_time_optimal` refuses a plan of more than ten million points instead of filling the memory where the
  motors allow only a crawl (a line onto the stretched SCARA arm asked some 64 GB).
- `lossless_config` of the joint machines zeroes a joint's own gearbox damping too.

### Fixed

- A drive enabled without the DC link simulated drew its winding's first L i^2 / 2 from nowhere; it is booked on the
  supply now.

### Fixed

- `sim-netlist` compiles again: the scissor's free joint is `ShaftFriction::NONE` (the literal missed the fields the
  friction branch added).

- `sim-netlist` scissor node: the free joint's shaft friction (fields added by the friction merge), so main compiles.

- sim-netlist did not compile at 04e5ecd (a `ShaftFriction` in scissor.rs without the fields the friction work added).
- A self-locking worm in a machine file holds its load: driven back the motor has to push to lower it, a lowering load
  with no torque brakes to a stop, and in the rigid model its output sticks at rest within what the worm can take
  (before, it lowered the load with no torque and crept).
- A belt effect switched on in a linkage file was silently ignored; it is refused.

### Fixed

- The D3 fails no check any more: settling 0.676 mm (FAIL against 0.5 mm) with the NRB080's 9 arcmin is 0.298 mm, and
  the gearing life no longer fails. The D2 no longer rests on an estimated 1 arcmin, and its lines planned within the
  axis limits (247 N m) stay within its gearbox's 278 N m.
- The setpoint slack of the deltas and linkages takes the gearbox's lost-motion band.

- Crank and cam mechanisms (`machine-crank-cam`): `slider-crank` (a servo press with J(θ), press force over the stroke, rod,
  guide and crank bearing loads), `rotary-knife` (cross cutter, synchronous zone and format length) and `flying-saw`
  (synchronise, cut in a window, return). An electronic cam (`sim_components::cam`: VDI 2143 laws, CSV cam tables, master
  ramp, slave limits at the master's speed) that the controller runs as the cycle with `cam.run = true`
  (`Machine::cam`); `limit-cam-speed`, `-acceleration`, `-jerk`; a mechanism's process force (`power.process`).

### Fixed

- `Aabb::gap`: a box inside another (or a point, the default collision body) had the width of the intersection, 0 for a
  point, as its depth; now it is how far it has to move to part.
- The D3's rods and upper arms were boxes up to 95 mm and 40 mm wider than they are where they slant.
- Drive loops: the position setpoint is interpolated between profile cycles (`effects.fine_interpolation`, as a SINAMICS
  drive in DSC does); a 4 ms IPO no longer doubles the peak torque (single axis 1.02 N m held, 0.507 N m interpolated,
  0.496 N m at 1 ms). The analysis measures the tool against the plan of the moment, not the setpoint held for the
  cycle. Both loops read one encoder sample per base step; the current loop's back-EMF feedforward and field weakening
  use the measured speed; the motor acceleration setpoint has the d(J^-1)/dt v term of a kinematics that is not linear.

- FMI: setting an FMU state whose simulation cannot be built again (its config file gone) left the instance in step
  mode without a simulation, refusing every later step; now the state is refused and the instance goes on.
- Python: an int beyond 64 bits set on a config became a float silently; now `OverflowError`. `run(time=1e12)` passed
  its checks and never finished; a run beyond 1e8 base steps or 2.5e8 sampled values is now refused with `ValueError`.

- The time-optimal plan holds each motor to its own drive and motor (`[drive.<id>]`, `[motor.<id>]`, `[parts.<id>]`),
  not the shared `[drive]`, and turns it up to its maximum speed along its torque-speed curve instead of stopping at
  the rated speed; above the rated speed it keeps back the same torque for friction as at a standstill.
- `check_limits` judges each motor against its maximum speed and its curve at that speed, and only warns: the
  catalogue search and the optimiser no longer use it as a screen (it ruled out 32 of 84 motors on EXCT-30 at 400 V
  that pass every verdict, 21 on the CoreXY).
- A motor's peak current is computed with its own motor and drive.
- `[bounds]` follow a part named in `[parts]` or on the command line: scaled by the part's value over the config's
  (`verdicts --bands` with a 230 V motor on EXCT-30 aborted on the braking threshold's bounds).
- `optimise` says which check the cycle as configured fails, or why it does not run.
- `search --gearboxes` keeps a motor that no gearbox of the series fits, with the config's own gearbox, and says how
  many; `search --motor-id` shows that motor's tightest check, peak torque and winding rise, and its mass with its own
  gearbox; the cost counts only the motors that take the candidate.
- The tightest check counts a failing belt-slack; a check judged from below at or below zero (a pushed belt section)
  is infinitely far over, not a margin above 100 %.

- Imported trajectories run a C2 cubic spline: the acceleration no longer steps at each pose
  (`profile.trajectory_spline`, `"hermite"` for the old curve).
- Conveyor: parts are picked during the belt's ramp; with input shaping a pass is planned for the shaped setpoint, so
  the gripper no longer closes up to a shaper period early and the tool no longer trails the part by the shaper's mean
  delay; `check_cycle` checks the passes; `optimise` no longer edits the unused `[[cycle.segment]]`.


- The search never paired the twelve SINAMICS S210 (New) converters 6SL5310 that `converters_s210.csv` has: the supplies
  `s210new-230v` and `s210new-400v` take the S210 motors with the 6SL5310 that D 32 lists beside each.

- `profile.cycle_timing = "time_optimal"`: every cycle segment timed rest to rest as fast as the motors, the gearbox,
  pulley, belt and guide ratings and the jerk limits allow (`path::plan_time_optimal`), so `analyze`, `verdicts`,
  `search` and `optimise` judge a cycle at the motors' limits. `"axis_limits"` (as before) in every shipped file.
- `profile.corner_tolerance_m`: a corner between two lines is rounded by a G2 curve within the tolerance (as a CNC's
  G642 or a robot's zone) instead of stopped at. 0 (stop) in every shipped file.
- Arcs on more than three axes (the other axes carried evenly with the distance) and splines on up to six: curves for
  the SCARA, the D4 and the arm in tool space.
- `Machine::rigid_guide_loads` for the linear delta, the linear motor axis and machine files: their guides' ratings
  limit the time-optimal plan.

### Changed

- The time-optimal plan passes a junction as fast as the junction and the speed curves beside it allow, no longer no
  faster than the slowest place of either piece, and joins the runs of a piece on the speed curve where that is
  quicker than on a cruise.
- The time-optimal plan keeps `profile.torque_reserve` of the guides' load rating, and its motors give a lossy
  gearbox's loss on top (`gearbox.efficiency` with `effects.gearbox_efficiency`).

### Fixed

- The time-optimal plan refuses a path through a singularity or one that would crawl longer than 1000 s
  (`error-path-singular`) instead of timing it on rows that are not numbers or handing on a pose per cycle without
  bound.

- A config reload (`Sim::reconfigure`) refused by a key read late (`profile.min_clearance_m`, `thermal.*`, obstacles,
  `effects.crash`, the conveyor's) no longer leaves the new controller and state beside the old machine: everything is
  checked before anything changes (also in `Controller::apply_config`, whose profile-cycle rescale ran before the
  conveyor was checked).
- Trace import: a time column without a unit whose step is a drive cycle in ms is read in ms, with a note
  (`time_unit = "s"` keeps seconds); lines of numbers above the header (metadata such as `Sample time;0,001`) are
  skipped with a note instead of being taken for data; times that do not rise are refused naming the signal and the
  row, also in a trace in the simulator's own columns; the friction fit no longer overflows on them.
- Error bands: a bounded value whose key is written as an integer no longer ends the band run with a Type error; it
  is written in the key's type (`analysis::set_bounded`), and bounds on such a key have to be whole numbers.
- The fit in the motors' coordinates on elastic gears: the six-axis arm as shipped fitted a link's factor −9.4 for
  +0.57; the load is now put where each gear's twist holds it (`Machine::twist`, `Machine::rigid_split_torque`) and the
  regressor's columns and torque are smoothed alike over ±40 ms.


- App: a run store (up to eight runs: analysed cycles and paths with their checks, a trace and its replay, the scopes'
  or the ghost's window, the cycle of any config or machine file with `--set`-style changes), overlaid per quantity
  with a difference trace against a reference and a table of key figures side by side; runs of different machines
  are aligned by motor and quantity.
- App: a light theme, one unit choice (rpm, mm) for the plots, and saving a scope, run, overlay, operating-point, path
  or effect-view plot as SVG or PNG by a right click; a fixed colour scale and a tolerance for the travelled path.

### Changed

- App: long lines are drawn with the lowest and highest sample of each stretch, so a peak one sample wide shows (the
  sizing report's plots too); the machine-derived views, the workspace map, the scopes' statistics and the trace
  overlay are worked out once per change instead of every frame; the history is one ring buffer and Freeze keeps the
  window only; the CSV exports run in the background to a file the dialog asks for.

### Fixed

- App: the travelled path's error colour measured from the plan's newest setpoint, which moves once per 1 ms profile
  cycle, and read up to speed times cycle too high (1.09 mm against a path error of 0.76 mm); it is now the distance
  from the path the plan drew, as the path error. The path and the ghost follow the scrubbed timeline; one colour per
  motor in every plot; trace and sweep files with a byte order mark are read; ticks on small plots; legends no longer
  cover the traces; the user guide's layout and `--set`.

- Browser page: machine files (the shipped `machines/*.toml`, and a TOML file opened or pasted), path runs within the
  axis limits or time-optimal, obstacles and a trail coloured by following error on a set scale in the 3D view, time
  plots of torque, speed and following error per motor with setpoints dashed and min/max decimation, operating points
  over the torque-speed curve, a frequency sweep, a mass and friction fit to a trace, every `sim-cli analyze` column in
  the sizing check, and German texts with a language switch. The module runs in Web Workers.

### Fixed

- Browser module: `frame(seconds)` with an infinite or huge time hung the page; it is refused with an error.
- FMI 2.0: the model description no longer declares the independent variable `time` (the standard then has it
  implicitly). FMIL, the FMI library of OpenModelica and other importers, reported the declared one as an error for
  want of a start value, which the standard forbids there. Tested in OpenModelica 1.27.1: its OMSimulator 2.1.3 runs
  the FMI 2.0 FMUs and agrees with `sim-cli run` to 5.0e-10.

- `sim-opcua`: Basic256Sha256 endpoints with Sign and SignAndEncrypt beside None; a self-signed certificate made in
  `--pki` on the first start; client certificates trusted from `pki/trusted` (refused ones in `pki/rejected`), or all
  with `--trust-all`; `--user NAME` with the password from `SIM_OPCUA_PASSWORD`.
- `sim-opcua`: the clients' read and write intervals (count, mean, standard deviation, largest) as `status.client_*`
  and at shutdown, to read a PLC's cycle jitter off.
- docs/plc.md: node ids with their OPC UA and IEC 61131-3 types (checked by a test), security, and a step-by-step run of
  the new `sim_cycle.st` programs for TwinCAT 3 and CODESYS against `sim-opcua` on the same PC.

- The browser page shows the selected machine in 3D running its cycle (WebGL2, orbit camera, legend with tool position and cycle time), beside the sizing check; the wasm module exports the scene per frame as a flat float array (`frame`, `frame_len`, request 3).
- `heat_color` and `part_color` moved from sim-app to `sim_core::scene`.

### Added

- Drive precontrol as commissioned on an S120/S210: `drive.torque_feedforward_model` (`rigid` or `inertia`, one
  moment of inertia per motor as p1498), friction feedforward from a characteristic (`effects.friction_feedforward`,
  p3820-p3839), balancing of the position setpoint (`drive.precontrol_balance_s`, p1429), a load-side encoder
  (`effects.load_encoder`, direct measuring system), and per motor the signals `torque_feedforward`,
  `torque_feedback` and `speed_from_position`.

- `drive.max_speed_rad_s`: the motor's highest speed, in every config (the no-load speed, [estimate], where no
  catalogue motor is named; EXCT-30 the 1FK2205-4AF's 628.3 rad/s); a motor by name sets the catalogue's. The verdict
  `motor-speed` judges the cycle's peak speed against it.

- Motion laws: sin² (Bestehorn) and the polynomial laws 3-4-5 and 4-5-6-7 of VDI 2143 beside the S-curve
  (`profile.shape`, or `shape` per `[[cycle.segment]]`), braking with a limit of its own (`profile.deceleration_ratio`),
  less acceleration upwards on a vertical axis (`profile.vertical_axis`, `up_acceleration_ratio`), and time scaling
  (`Profile::stretched`).
- Input shapers ZV, ZVD, ZVDD, EI and two-hump EI (`profile.shaper_type`, `shaper_ei_tolerance`), any number of modes
  convolved (`profile.shaper_modes`), each on its own axes (`axes`, `profile.shaper_axes`).
- Program import (`program.rs`): a G-code subset (G0-G3, F, G17-G19, G20/G21, G90/G91), robot moves as CSV (MoveL,
  MoveJ, MoveC), KUKA KRL (PTP, LIN, CIRC) and ABB RAPID (MoveL, MoveJ, MoveC) into path pieces, by extension, for
  `sim-cli --path` and the app's path field; runs at one feed planned with the axes capped at the feed.
- Conveyor examples beside the T-gantry's, D2's, D4's and linear delta's default configs.


- The catalogue has the servo motors and drives of eight more makers, from their public datasheets with page and
  table: Beckhoff AM8000/AM8500 with AX8000 and AX5000, B&R 8LS with ACOPOS P3, Kollmorgen AKM and AKM2G with AKD,
  Lenze MCS and m850 with i950, Mitsubishi HK-KT/HK-ST with MR-J5, Bosch Rexroth MS2N with ctrlX DRIVE, SEW CMP with
  MOVIDRIVE, Yaskawa SGM7J/SGM7A with SGD7S (about 1,100 motor rows, 230 drive rows). A motor is named at the supply
  `<vendor>-<volts>v`; the search pairs it with the maker's drive, and `sim-cli search --vendor NAME|all` and Python
  `sizing(vendor=...)` search across makers, with the columns `vendor` and `supply`. Schema 1 gains the optional
  columns `vendor`, `line_voltage_v`, `line_resistance_20c_ohm`, `line_inductance_h` and the drives' DC link and
  braking columns; the validator checks the new makers' physics (`vendor_motors_are_plausible_and_their_drives_exist`,
  `vendor_drives_are_plausible`).

- Several payload steps per trace (`JointFit::payload_steps`, up to eight), and payload steps on the fit per axis
  (`AxisFit::payload_steps`, `fit-axis-payload-step`, CLI rows `payload_step_s` and `payload_factor_change`).
- `Imported::notes`: how an export was read other than it says, printed by `sim-cli` (`note-import-skipped`,
  `note-import-time-ms`).

- Machine files: a gantry bridge that yaws (`machine.gantry`, `effects.gantry_yaw`): its yaw as a coordinate of its own,
  held by drives at its sides, belts that pull its ends and its joints' spring; the carriage's lever on it. Two drives
  on one axis make a dual-drive gantry (one motor more than axes) whose motors share the bridge by the lever rule;
  signals `gantry.<id>.yaw`, `.sync_error`, `.moment`, check `gantry-sync`, `ConfigMachine::racking_moment`. The H-bot,
  CoreXY and rack gantry files have it (off as shipped): the H-bot racks with the head's pull, the CoreXY's belts cancel
  it. New file `machines/dual-gantry.toml` (two racks, catalogue 1FK2103-4AH motors).
- Machine files: bodies riding on rotary axes (`offset_m`, rotary axes tilted by `incline_deg`), with their full mass
  matrix, gravity and centripetal and Coriolis forces; a tool centre point (`machine.tcp`) whose tip the path moves
  while the rotaries turn and the straight axes make up for them, with the feedforward's speed part. New files
  `machines/portal-c.toml` (a C axis at the tool) and `machines/five-axis.toml` (a fork head, A and C).

- Identification of links and of a payload picked during a trace (`identify::fit_joints`): on the URDF arm and the
  SCARA each link's mass factor and first moment where the trace tells them apart (base parameters: a value is fitted
  only where no value's variance is inflated by more than 100; the others are folded in, with their shares named),
  and a payload picked or dropped during the trace found in the torques and fitted before and after
  (`payload_step_s`). `Machine::links` and `Machine::set_link_change`. `sim-cli fit` and the app's trace panel show
  both.

### Added

- Machine files: a `linkage` node, a planar parallel linkage (two driven arms, forearm pairs, a plate that keeps its
  orientation), with inverse and forward kinematics, Jacobian, working range, condition-number check and the planar
  delta's dynamics; `machines/planar-delta.toml`, the planar delta as a file, gives the crate's signals and verdicts to
  the bit with each of its effects.

### Added

- Delta D3 (and D4): the forearm rods' spin about their own axis as the pair's plane turns, in the mass matrix and
  the velocity term with `effects.forearm_inertia` (`mass.forearm_rod_spin_inertia_kgm2`); the upper arms' bending
  out of the plane they turn in, a lateral spring and damper at each elbow with `effects.link_elasticity`
  (`upper_arm.lateral_stiffness_n_m`, `lateral_damping_n_s_m`), 6 more states; `Machine::modes` with every arm mode
  and its damping (3, 6 or 9); the upper arms as collision bodies (ten boxes each).

### Changed

- Delta D3: the modes are found by Jacobi rotations of the whole eigenproblem (`arm_modes`, in place of the inverse
  iteration `lowest_mode`); `Legs::rods` takes the elbow's move out of the arm's plane; `Legs::rod_middle` is gone
  (`Delta::load_n` builds every point mass from its Jacobian).

### Added

- `sim-cli optimise --worst` (in the app: "worst corner" beside "with error bands"): the optimiser's result judged at the
  worst corner of all bounds at once (`verdict_worst`) instead of the one-at-a-time bands, rows `worst_*`. The
  worst-corner search stops at the first failing batch of runs, starts from the previous step's worst corners, and the
  search's steps are bisected (`OptimiseOptions::worst`, `Robust::worst`). Not the default: 16 min on the default
  T-gantry.
- Lift height profiles: where two or more obstacles are in a traverse's way, the optimiser also tries a path through
  two via points per obstacle at its own height (`analysis::lift_profiles`), and takes it where it is faster.

### Changed

- `LiftChange::via` is a list of via points (`Option<Vec<Vec<f64>>>`); the CLI's `via_m` row puts `; ` between them.

### Added

- Field weakening (`effects.field_weakening`, off in the shipped configs): a d-q model of the permanent-magnet motor in
  the drive's current control, with the voltage and current circles and a negative d current above base speed; the
  torque-speed curve follows from it. New keys `motor.pole_pairs`, `motor.d_inductance_h` (set for a 1FK7 or 1FT7 by name).
- Drive overload (`effects.drive_overload`, off in the shipped configs): the drive's i²t image lowers its current limit
  to the base load current; verdict `drive-overload`. New keys `ratings.drive_base_load_current_a` (set by a drive by
  name), `drive_overload_time_constant_s`, `drive_max_current_time_s`.
- Three thermal nodes for the motor (`effects.motor_thermal_nodes`, off in the shipped configs): winding, stator and
  housing, surroundings; the one node stays the fallback. New keys `motor.winding_resistance_share`,
  `winding_time_constant_s`.
- `sim-cli analyze`: columns `housing_rise_k` and `drive_overload`; the app's motor table has both.

### Added

- `sim-app --set path=value` (repeatable) and a "Set a value: key = value" field in the app's parameter panel: any
  config key by its dotted path, refused as `sim-cli --set` refuses it (unknown key, wrong type, out of range).

### Fixed

- A path planned in the app starts at the live setpoint; on the planar delta it started 0.26 mm off it, one rest sag.

- SCARA and six-axis arm in tool space (`kinematics.tool_space`, off in the shipped configs): the axes are the
  flange's pose (SCARA x, y, z, c in closed form with `kinematics.elbow`; arm x, y, z and a rotation vector, damped
  least squares followed from `kinematics.home_joints`), so moves, paths and cycles are straight lines of the tool;
  refusals `error-arm-out-of-reach`, `error-arm-singularity` (`workspace.max_condition_number`),
  `error-arm-orientation-half-turn`; new configs `config/scara-tool.toml` and `config/arm-tool.toml` with cycles of
  straight lines. `Mechanism::condition`, `Mechanism::tool_is_joints` and `JointSpec::unit` in `chain.rs`; in tool
  space the joint machine's feedforward takes the speed terms.
- Fixed: a time-optimal path through a pose the machine cannot go to (an arm's singularity) slowed to a standstill and
  asked for unbounded memory; it is refused with `error-path-range` first.

- Picking from a moving conveyor: a `[conveyor]` table makes the cycle pick parts from a belt (intercept with the belt's
  speed, tracking during the grip, lift-off, place fixed or onto a second belt), for every Cartesian machine; picks a
  minute, the verdicts `pick-tracking` and `parts-missed`, the belt and its parts in the 3D view.
- `crates/machine-delta-d3/conveyor.toml`: the D3 picking from a belt.

### Changed

- The time-optimal plan follows the highest speed a piece allows within it, up and down (a smoothed speed curve per
  piece, runs from both ends that rise along the acceleration ceilings at the speed they reach and follow the curve),
  instead of one cruise speed per piece and 16 + 16 trial top speeds; the user guide's spline example 1.380 -> 1.349 s.
- The time-optimal plan keeps the drive's maximum current (`effects.drive_current_limit`), `profile.torque_reserve` of
  the gearbox's and the pulley's torque rating at the output, and the guides' load rating (T-gantry, single axis) at
  every planned pose. The delta's and the arm's plans are slower where their gearboxes' M2max binds (a line across the
  delta's range 0.411 -> 0.434 s). `path::plan_time_optimal` and `deviation_time_optimal` take a `path::PlanLimits`.

### Added

- `Machine::rigid_guide_loads`, and the refusal `error-path-guide` (EN/DE) for a path whose guide load at rest is
  beyond its rating.

### Added

- Machine files: `effects.belt_wrap` from the places of pulleys and idlers (`at_m`, new `clockwise` field) along the
  sections' routes; `effects.tool_flex` with a `lean` node (a body leaning on its carriage, as the T-gantry's Z axis);
  `worm` nodes (ratio, forward and back efficiency, self-locking at a back efficiency of 0 or less). The T-gantry file
  has both effects' geometry and is its crate to 1e-9 with either on.
- `sim-cli search --motor-id ID`: the catalogue search for one motor of a machine file, the others kept
  (`catalog::search_motor`, `catalog::with_motor_part`).

### Added

- `sim-opcua`: `--config FILE`, `--set PATH=VALUE` and `--trajectory FILE`, read as `sim-cli` reads them
  (`sim_opcua::configured`, `Link::run_trajectory`).
- `sim-opcua`: Aes128_Sha256_RsaOaep and Aes256_Sha256_RsaPss endpoints (Sign, SignAndEncrypt) beside Basic256Sha256;
  X.509 user tokens with `--user-cert FILE`; a warning from 30 days before the server's certificate expires and
  `--renew-certificate` (`certificate_days_left`, `renew_certificate`).

### Fixed

- `sim-opcua` PLCopen: a block started again while busy whose new inputs are refused showed Busy beside Error and then
  Done beside Error. It now shows Error alone and the axis brakes to a stop with the motion's deceleration.
- `sim-opcua` PLCopen: `Power.Error` and `Power.ErrorID` were hidden while `Power.Enable` was false, although the axis
  stays in ErrorStop until MC_Reset; they now show the axis error as State does.
- `sim-opcua`: writes of SByte, Byte, UInt16, UInt32 and UInt64 (IEC SINT, USINT, UINT, UDINT, ULINT) to numeric
  inputs were refused with BadTypeMismatch; 64-bit integers beyond 2^53 are refused now.

### Changed

- `deny.toml`: the reason for ignoring RUSTSEC-2023-0071 (rsa, Marvin) was false since the secured endpoints came; it
  now says the advisory applies, with the residual risk in docs/plc.md and SECURITY.md.
- PLC Structured Text examples: TwinCAT names checked against Beckhoff Infosys and Beckhoff's samples (attribute id,
  identifier type, session timeout, security mode and policy, user identity token, TF6100's PKI folder); CODESYS
  names aligned with the PLCopen specification v1.02.

### Added

- A Quattro-type four-arm delta, `delta-h4`: four arms at 90 degrees, a plate in two parts that slide against each
  other and turn the tool through a pinion between two racks and a gear stage (x, y, z, c; four motors). The tool's
  turn on the amplifier's spring and its backlash are effects of their own (`effects.amplifier_compliance`,
  `effects.amplifier_backlash`). Sized near the Omron Quattro s650H class with Siemens 1FK2105-6AF and NRB120-05-20;
  its cycle fails `settling` (the turn amplifies the arms' twist and backlash 30 times), a finding.
- Machine `parallel-scara` (crate `machine-parallel-scara`): a five-bar in the horizontal plane (the D2's, laid flat,
  as Mitsubishi's RP series) with an optional Z stroke on a ball screw and an optional tool turn at the plate, each with
  its own motor, gearbox, drive and ratings tables; the arms' motors carry no weight, the Z motor holds the carriage.
- Linear delta: inclined columns (`geometry.column_incline_deg`, positive with the tops towards the axis), a tripod as
  Festo's EXPT; kinematics, dynamics (the carriage's weight m g cos alpha on the belt, the rest on the guide), scene.
  Upright it is the vertical machine to the bit.
- Linear delta: guide, pulley-bearing and belt-tooth ratings of the EGC-80 from Festo's datasheet (guide 3050 N for
  5000 km, C 14157 N; estimates with bounds for the bearing and the teeth); per-carriage `guide-load` and `guide-life`
  verdicts.
- Linear delta: the guides (and gearboxes, where configured) warm with their loss (effects.part_heating, `[thermal]`), and
  with thermal_feedback each guide's friction and gearbox's efficiency follow its temperature.

### Changed

- D4: the tool turn's motor has its own tables, `[motor.4]`, `[gearbox.4]`, `[drive.4]`, `[ratings.4]` (a 1FK2104-4AF on
  an NRB060-04-8 and its own S210); `[tool_gearbox]` is now `[gearbox.4]` and `ratings.tool_gearbox_max_torque_nm` (check
  `tool-gearbox-peak`) is `ratings.4.gearbox_max_torque_nm`, judged by the common gearbox checks. Tables of an arm's own
  motor or gearbox are refused (the arms share the D3's).

### Fixed

- D4: the common gearbox checks of motor 4 judged the arms' gearbox and ratio, and its drive had the arms' motor; and
  `sim-cli search delta-d4 --motor-id 4` changed the tool's drive but not its joint.
- Linear delta: the guide load was the rods' pull at rest; it now follows the motion (the rods' pull from the plate's
  acceleration), so the verdicts judge the cycle's highest load (116 N on the default cycle, 3.4 N at rest).
- Linear delta: `ratings.pulley_max_torque_nm` is Festo's 5 N m for the EGC-80, not an estimate of 10.

## [1.0.0-rc.1] - 2026-11-14

Stages A to D of the plan's end goals, the effect views, the electrical drive and the FMI export.
Then four more delta pickers (D2, D4, linear), machine files with screws, racks and mechanisms, the warm start
and the drive's current limit, life estimates with a_ISO, a point-by-point time-optimal plan, transition curves on
three axes, a robust optimiser, modes, identification of non-linear machines and new app views.

### Added

- Four-axis delta picker D4 (`delta-d4`, crate `machine-delta-d4`): the D3 with a turning tool, driven by a fourth
  motor through a telescopic shaft with two cardan joints (the pair in phase cancels: the tool turns with the motor),
  tool pose (x, y, z, c); switches `effects.shaft_mass`, `shaft_torsion`, `telescope_friction` (on); the check
  `tool-gearbox-peak`; its default cycle (the Adept cycle with a 90 degree turn) passes every verdict. The D3 gained
  `PlatformDrag` (`Delta::set_platform_drag`), a force on its platform along a line, for the telescope's friction.
- Machine files: ball screws, racks, linear motors and rotary tables beside belts (nodes `screw`, `rack`,
  `linear_motor`, `rotary`; value tables laid out as the joint kit's `[screw]`, `[rack]`, `[linear_motor]`,
  `[strain_wave]`; a rotary axis in rad with `stroke_rad`), built with the joint kit's `RotaryJoint` and
  `transmission::Converter`, with the kit's signals and checks (`screw-speed`, `screw-dn`, `screw-buckling`,
  `screw-static`, `screw-life`, `rack-feed-force`, `strain-wave-life`). New files `machines/screw-portal.toml` (Z on a
  ball screw with a brake) and `machines/rack-gantry.toml` (3 m X on rack and pinion, Y on a belt). The JSON schema no
  longer requires `pulley`, `section`, `body` and `gearbox`. Verdicts no longer judge belt-tooth load for a motor that
  turns no pulley (it reports neither a pulley radial load nor a belt wrap).
- Delta picker D3: forearms as rigid rods with their inertia (`effects.forearm_inertia`, on; off is the lumped model),
  bending upper arms (`effects.link_elasticity`, `[upper_arm]`, off), the lowest mode with the motors held through
  `resonance_hz` and `mode_shape` (so `shaper_follows_pose` follows it), and the forearm rods as collision bodies.
- Thermal warm start (`effects.thermal_warm_start`, on in the shipped configs): the cycle analysis measures its pass
  with the windings and, with `effects.thermal_feedback`, the gearboxes and guides at the rises they settle at,
  iterated from cold to within 0.1 K (secant step, at most 4 warm passes); verdict row `thermal-warm-start`, and
  `analyze`/`verdicts` say on the error stream which start was used. Shipped verdicts move in the fourth digit (windings
  at 1 to 7 K); with the feedback on, the EXCT-30's X guide settles at 15.9 K instead of 23.7 K.
- The drive's current limit simulated (`effects.drive_current_limit`, on in the shipped configs): the torque setpoint is
  held to the torque of `ratings.drive_max_current_a` where that is below the motor's limit, with anti-windup; a cycle
  that asks more lags behind, and `drive-peak-current` judges the current asked for. Nothing changes on the shipped
  configs, whose drives give more than their motors take.
- Transition curves on three axes (the delta): with `profile.path_tolerance_m` the curves between lines and arcs are
  laid in the plane of the arc they meet, a line between arcs in two planes is shared out, and deviation, plan and
  reports hold in space; spline junctions on three axes are eased as on two. Two arcs meeting in different planes
  keep their junction.
- Transition curves: one length per arc (from its own junctions' speed and its own tolerance limit) instead of one
  for the whole path; curves at an arc fixed at one end with an arc at its other end, and at one that turns more
  than half a circle (moved by at most half the tolerance).
- Planar delta picker D2 (`delta-d2`, crate `machine-delta-d2`): two rotary arms in the x-z plane with forearm pairs
  to a plate that keeps its orientation, sized like the Codian D2-800-S020 (800 x 477 mm, 5 kg) with Siemens
  1FK2105-4AF behind ratio 20; kinematics, Jacobian and condition number, the D3's dynamics, scene, collision bodies,
  a default Adept cycle that passes its verdicts. The gearbox backlash is an estimate (1 arcmin): with the catalogue's
  9 arcmin the cycle fails settling.
- `CITATION.cff`; `deny.toml` with a cargo-deny job in CI (licences, advisories, duplicate versions, sources).
- GitHub Pages workflow (`pages.yml`) that publishes the browser sizing page: https://sparkz400.github.io/KinematicsSimulation/.
- README: an animated GIF of the T-gantry's cycle and a screenshot of the delta picker; intro, machine list, limits
  and crate table brought up to date (nine machines, machine files, FMI 3.0, Python, WebAssembly), also in German.
- Fit of a delta, SCARA or arm from a drive trace, in the motors' coordinates: the payload's mass (a factor) and per
  motor added inertia, viscous and Coulomb friction and constant torque, by least squares on the machine's own rigid
  model (`identify::fit_joints`, `Machine::rigid_motor_torque`), with the pairs the trace cannot tell apart; `sim-cli
  fit --trace` and the app's trace panel. Trace replay takes the delta.
- The belts' stiffness from an ordinary move's trace, without a sweep: `sim-cli fit --trace FILE --stiffness
  [--stiffness-key KEY]` and a checkbox in the app's trace panel (`identify::fit_move_stiffness`), refused with a
  reason where the moves do not ring the belt enough.
- Linear delta (`delta-linear`, crate `machine-delta-linear`): three vertical toothed-belt axes with parallelogram rods
  to a level plate, sized like the Festo EXPT-95 with Siemens 1FK2205-2AF motors on S210 converters. Inverse, forward
  kinematics, Jacobian, working range with the rods' tilt and the carriages' stroke, condition number; elastic belt
  axes (sections as springs, couplings, LuGre guides loaded by the rods' pull, stops, brakes) and a rigid model; its
  default Adept cycle passes every verdict, also over the estimates' bounds.
- Machine files: a machine defined in a TOML or JSON file as a netlist of components (motor, gearbox, pulley,
  belt sections, bodies, guides) with the tables of a machine config, loaded at run time by `sim-cli --machine-file`,
  the app (Open, or `sim-app --machine-file`) and `sim-opcua --machine-file`; versioned, checked with file, line and
  key in every error, with a generated JSON Schema (`schema/machine-file.schema.json`). The single axis and the
  T-gantry as machine files reproduce their crates (`machines/`, `docs/machine-files.md`).
- Machine files: a motor's own `[motor.<id>]`, `[gearbox.<id>]`, `[drive.<id>]`, `[ratings.<id>]` and `[parts.<id>]`
  over the shared tables, for its drive train, its drive and its verdicts (`catalog::motor_config`).
- Machine files: the effects frame compliance, idlers, cable chain, process force, thermal feedback, belt tooth jump and
  span vibration, with the node kinds `idler`, `frame` and `span` and an axis's `cable_chain`; tool flex stays refused.
- Machine files report the belt's lowest mode per pose (`resonance_hz`, `mode_shape`), so `shaper_follows_pose` works
  and the app shows it; the 3D view draws their belt (`route`, `at_m`).
- `sim-cli fmu --machine-file`: the FMU carries the file (`resources/machine.toml`) and builds its machine from it, FMI
  2.0 and 3.0; `tools/check_fmu.py` takes a machine file, and CI checks the H-bot's FMU.
- `machines/corexy.toml` (CoreXY) and `machines/xyz-portal.toml` (three-axis portal with a smaller Z motor).
- Licence: dual MIT OR Apache-2.0 (`LICENSE-MIT`, `LICENSE-APACHE`), and `THIRD-PARTY-LICENSES.md` generated from
  `cargo metadata` by `tools/third_party_licenses.py`.
- `CONTRIBUTING.md`, `ROADMAP.md`, this changelog.
- CI on Windows and Linux (`.github/workflows/ci.yml`): build, tests, clippy.
- App: a workspace map of the delta in the Path tab (a horizontal slice at a chosen height, coloured by the
  Jacobian's condition number; `Machine::condition`), the S1 line beside the peak torque-speed curve, the operating
  points of a path, the energy budget as a flow diagram, and the obstacles as filled translucent boxes.
- Unsigned release packages: `tools/package.ps1` (zip) and `tools/package.sh` (tar.gz) write `dist/`.
- The FMUs of both machines and of the EXCT-30 config checked with an independent tool (fmpy).
- Mechanisms and transmissions (`docs/mechanisms.md`, German `docs/de/mechanismen.md`): ball screw, rack and pinion,
  linear motor, worm and strain wave gear as components (`sim_components::transmission`), a URDF reader with
  recursive Newton-Euler dynamics (`rigid`), a machine of joints (`chain::JointMachine`), and six machines in
  `machine-joint-kit`: `screw-axis`, `rack-axis`, `linear-motor-axis`, `rotary-table`, `urdf-arm`, `scara`, with
  their own rating checks (`screw-speed`, `screw-dn`, `screw-buckling`, `screw-static`, `screw-life`,
  `rack-feed-force`, `strain-wave-life`).
- `Machine::modes`: every mode a machine knows at a pose, with its damping (T-gantry both two-mass modes, the single
  axis its carriage on the belt); `sim-cli modes` and a plot in the app's tuning panel show them over an axis's stroke.
- With `effects.shaper_follows_pose` the second input shaper follows the machine's second mode; the optimiser's second
  shaper uses it.
- `effects.belt_wrap` (off): the teeth in mesh from the belt's wrap at the pose, for the tooth-jump law and the tooth
  load.
- Life estimates: the life modification factor a_ISO of ISO 281:2007 on the pulley shaft bearings (keys
  `ratings.bearing_lubricant_viscosity_mm2_s`, `bearing_contamination_factor`, `pulley_bearing_fatigue_limit_n`,
  `pulley_bearing_mean_diameter_mm`); the gearing's fatigue above the gearbox's rated torque by Palmgren–Miner on
  Siemens' M2N and M2max figures (`gearbox-gearing-life`, key `ratings.gearbox_gearing_wohler_exponent`); the motor's
  own bearings where a pulley sits on its shaft (`motor-bearing-life`, key `ratings.motor_bearing_load_rating_n`, C
  from Siemens' allowable radial force, `catalog/siemens/motor_bearings_1fk2.csv`). `sim-cli analyze` has the columns
  `gearbox_gearing_life_h` and `motor_bearing_life_h`.
- Cycle optimiser: lifts placed over the config's obstacles (each traverse no lower than keeps the clearance with the
  machine's collision bodies, a via point over the obstacle tried too); `sim-cli optimise --pareto FILE` writes the
  front of cycle time, energy per cycle from the supply and peak motor utilisation, which the app plots below the
  optimiser's table.
- **Physics (stage A):** gearbox contact that cannot pull; the leaning Z axis (tool flex); drive computation delay,
  fieldbus delay and jitter, sensor noise; winding heating, torque saturation, a DC link with braking chopper and
  trip, shared or per drive; belt tooth stiffness, material damping, slack time, idler inertia and friction;
  cable chain, process force, hard stops, Disable / Enable and motor holding brakes on every machine.
- **Sizing (stage B):** verdicts against `[ratings]` (`sim-cli verdicts`), including gearbox input speed, radial
  shaft load, guide load and life, bearing lives and the belt's tooth load; `[bounds]` with one-at-a-time and
  sampled error bands and a worst-corner search; sensitivity ranking.
- **Vendor catalogue (stage B):** Siemens S210 and S120 motors, converters, Motor and Line Modules, braking
  resistors, NRB/NRK/NLC planetary gearboxes; Festo EXCT and Bahr belt axes; parts named in a config's `[parts]`;
  `sim-cli search` for motors and gearboxes; the Festo EXCT-30 config (`crates/machine-t-gantry/exct-30.toml`).
- **Paths (stage C):** lines, arcs and splines with look-ahead and jerk limits, transition curves within
  `profile.path_tolerance_m`, a time-optimal plan within the motors' torque-speed curves, path error, ZVD input
  shaping (fixed, following the pose, a second resonance), and a cycle optimiser.
- **Traces (stage D):** trace import and replay, a two-stage fit (masses and friction; belt stiffness from a sweep).
- **Effect views:** torque breakdown, loss budget, friction view, temperatures, belt sections, one effect on
  against off (`sim-cli compare`).
- **Electrical drive:** R-L winding with PI current control, back-EMF and the DC link voltage as the limit.
- **Tuning:** step, closed-loop frequency response and loop margins (`step`, `bode`, `tuning`, `margins`).
- **FMI export:** `sim-fmi`, an FMI 2.0 Co-Simulation FMU per machine and config (`sim-cli fmu`).
- **App:** everything the command line does, config files opened and saved, CAD meshes in the 3D view.
- **OPC UA:** paths, Disable and Enable.

### Changed

- The time-optimal plan (`--time-optimal`) reads the motors' limits point by point, every 2 mm along a piece (and
  every 0.02 rad on a curve), instead of one speed and one acceleration limit per piece from nine poses: each ramp
  accelerates by what the torque, the belt's tension and the axes' jerk allow where it is, gravity counts each way
  as it pushes, the centripetal and Coriolis torque of the speed counts, and a piece may cruise faster than its
  slowest place where its ramps pass that place more slowly. Arcs no longer share the limits 3 : 4 : 5. The user
  guide's line, arc, line takes 1.395 s instead of 1.429 s, its spline example 1.380 s instead of 1.674 s, a line
  across the delta's range 0.411 s instead of 0.429 s; the default cycles of the T-gantry, the delta and the arms
  are limited by jerk and speed and plan as before. Planning takes 1 to 21 ms instead of 0.2 to 1 ms.
- The pulley shaft bearings' life now includes a_ISO: the shipped configs' lives grow (EXCT-30 31800 h to 325800 h,
  which now passes in both band modes; it still fails at the worst corner). The delta's first arm fails the new
  gearing check (about 9000 h against 20000 h).
- `sim-cli optimise` judges its result in the error bands of `[bounds]` too and steps back along the search to the
  fastest setting that passes there (checks the configured cycle fails there already are listed, not judged);
  `--nominal` gives the old result. EXCT-30: 1.590 s instead of 1.492 s. The app does so with "error bands" ticked.

### Fixed

- Nine rounds of review by experiment; each fix has a test that was seen failing without it (PROGRESS.md,
  "Review fixes" to "Ninth review fixes").
- Two rounds of fixes after merging fifteen branches: every test of the merged code passes, and the suite runs in
  about 8 minutes again (PROGRESS.md, the sections of 14 November).

## [0.1.0] - 2026-11-11

Phases 0 to 6 of the project plan.

### Added

- **Phase 0:** Rust workspace with the layers as separate crates.
- **Phase 1:** core with fixed-step RK4 integrator, machine interface, config loading, German and English
  translations; rigid single axis and T-gantry; the standard machine test suite (round trip, static hold, energy,
  step-size convergence, determinism); dependency-direction check.
- **Phase 2:** belt sections, LuGre friction, shaft friction, ripple, gearbox with backlash and stiffness, frame
  mode, each with its own switch; a SciPy reference for the elastic T-gantry.
- **Phase 3:** cascaded drive loops, feedforward, jerk-limited profiles, the pick-and-place cycle with payload
  switching, external setpoint interpolation.
- **Phase 4:** desktop app (Bevy, egui) with 3D view, scopes, parameter and effect panels, two languages.
- **Phase 5:** cycle analysis for motor sizing (peak and RMS torque, utilisation of the torque-speed curve) and
  frequency sweeps, as CSV from `sim-cli`.
- **Phase 6:** PLC link as an OPC UA server (`sim-opcua`).
- After phase 6: winding model, corner blending, tuning help, a reworked 3D view and scopes.

<!-- Version links (compare/tag URLs) go here once the repository has a public remote. -->
