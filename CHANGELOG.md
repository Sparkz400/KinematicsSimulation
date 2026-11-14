# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the project uses
[Semantic Versioning](https://semver.org/spec/v2.0.0.html). Details, test names and the numbers behind each
item are in [PROGRESS.md](PROGRESS.md).

## [Unreleased]

### Fixed

- Drive loops: the position setpoint is interpolated between profile cycles (`effects.fine_interpolation`, as a SINAMICS
  drive in DSC does); a 4 ms IPO no longer doubles the peak torque (single axis 1.02 N m held, 0.507 N m interpolated,
  0.496 N m at 1 ms). The analysis measures the tool against the plan of the moment, not the setpoint held for the
  cycle. Both loops read one encoder sample per base step; the current loop's back-EMF feedforward and field weakening
  use the measured speed; the motor acceleration setpoint has the d(J^-1)/dt v term of a kinematics that is not linear.

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
