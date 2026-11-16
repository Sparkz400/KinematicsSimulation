# Roadmap

What is not built yet, or built and left open, collected from the "Left open" lists of [PROGRESS.md](docs/development/PROGRESS.md), the
commit messages since the 1.0.0 cut, and the "not covered", "shortcut" and "assumptions" passages of the
[model reference](docs/model-reference.md), [mechanisms](docs/mechanisms.md) and [validation](docs/validation.md).
No dates. What to verify before the release is in [TODO.md](docs/development/TODO.md).

## Needs a real machine or data

- Measured machines. One exists: the ball screw bench of ISW, University of Stuttgart
  (`datasets/isw-ball-screw-axis`, CC BY 4.0). After the fit on its S-profile, with the screw's efficiency from its
  lead angle, the replayed motor torque is 4.6 % of the peak off in RMS (27 % at the worst sample); on the two
  recordings kept as checks 13.1 % and 6.1 % (34 % and 25 %). The fitted moved mass is 57 kg below the config's 549 kg
  (92 kg before the efficiency): what is left is the screw shaft's inertia, a solid cylinder at its nominal diameter
  over 1.8 m in the model, whose threaded part and journals the source does not give; the held force at rest has the
  wrong sign on the constant-speed run; the model's first mode is 137 Hz against 123 to 134 Hz measured
  ([validation](docs/validation.md#against-measured-data)). No other machine has been measured, and the project's own
  test runs on a real axis (stages D2, D4 of the plan) have not been made.
- A drive's own trace export. The importers for SINAMICS, TwinCAT and CODESYS traces are tested on files built after
  the vendors' layouts and on the converted ISW recordings; no real export has been read. The layout of the
  Startdrive CSV export is not documented by Siemens and is assumed.
- A real PLC (Siemens, Beckhoff, CODESYS) against the OPC UA server. The server is prepared (secured endpoints, trusted
  client certificates, user and password, a checked type table, the clients' cycle jitter, Structured Text programs
  for TwinCAT and CODESYS, docs/plc.md); no PLC has connected yet and the ST files are not compiled.
- A measured stop: the stop categories and brake delays are checked against hand calculations only.
- The S210's DC link capacitance and chopper threshold: Siemens publishes neither. The capacitance is measured for
  one converter (6SL5310-1BE10-8DF0, `dc_link_measured.csv`); for every other converter both stay estimates.
- The motors' own bearings: Siemens gives their allowable radial forces only as diagrams; one frame size
  (1FK2205) is read into the catalogue, the others are not.
- The planar, linear and four-axis deltas: their makers publish no geometry or masses; those are estimates.
- Catalogue data for steppers, induction motors, vacuum cups and valve flow values (ISO 6358 C and b): none is in
  `catalog/`; the shipped stepper, induction motor, gripper and valve values are estimates. Festo prints no C and b
  for its VUVG valves.

## Physics and planning

- Time-optimal plan: it keeps every limit a verdict judges at a point of the path, not those judged over the whole
  cycle (winding and part warming, S1, RMS current, the lives, the mean braking power). So the linear motor axis, the
  flying saw, the parallel SCARA and the four-arm delta fail "motor-thermal" on their time-optimal cycles. Also not in
  the plan: settling, the cable robots' tensions, the parallel SCARA's and the machine files' own checks, the wafer
  handler's wafer slip (an acceleration's length, no linear row), the arm on a track's guide load. A short piece
  whose speed curve rises into its end faster than the jerk allows is refused and timed within the axis limits.
- Compliant base, two-way (`effects.base_reaction`): the gyroscopic and centrifugal terms of the base's tilt rates and
  the change of the base's inertia with the pose are left out; refused for the six-axis arm and the palletiser, whose
  pedestal's estimated inertia (60 kg m²) is below what the arm moves on it; the cable robots' winches' spin is left
  out of their momentum.
- Coupled link bending (`effects.link_bending_reaction`): Lagrange's terms of second order in the small bending
  (centrifugal stiffening, gyroscopic terms) are left out; one mode per direction and link.
- Stops: category 2 brakes along the direction of motion, not along a curved path, and without a jerk limit; one
  engage and one release delay serve all brakes of a machine; the verdicts `stopping-distance` and `stop-dc-link`
  appear only in `sim-cli stop`, not in `verdicts`, the search or the optimiser.
- The drive's start: a machine built at rest is held where its tool stands, but `Command::Enable` still holds at the
  setpoint (the pose the encoders read), since it has no machine state.
- Gearbox mesh friction (`effects.gearbox_mesh_friction`): on in the joint kit's and cable robots' configs; off on the
  T-gantry, whose energy budget misses its 0.1 % tolerance with it (0.163 %); not in the deltas' configs, and on the
  four-arm delta's arms it would not act (they do not commit their gearboxes, so the faded efficiency stays).
- Friction feedforward from the machine's model: off on the single axis and the linear delta, whose settling got worse
  with it (also with the single axis's recorded characteristic) for a reason not found; off where some axis follows
  worse (four-arm delta, parallel SCARA, palletiser, arms) and in most machine files.
- Steppers: open loop only, no closed-loop stepper; sinusoidal phases, no standstill current reduction.
- Pneumatics: no proportional valve and no servo-pneumatic position loop; the cylinder is no tool axis and does not act
  back on the axis carrying it.
- Gripper: the payload's mass switches as a step; no gripper dynamics or compliance; the vacuum cup's hold is judged
  with its axis vertical.
- The palletiser ships without input shaping, although its turret rings at about 5.3 Hz after a long move.
- Winder: the drive does not adapt its speed controller to the roll's inertia (45-fold from core to full), so the
  shipped gains fit the stretch the cycle runs, 3 m at the roll's start; the other end of the roll is judged through
  the machine's own verdicts, not run. A category 1 stop (each drive on its own OFF3 ramp) overloads the web near
  either end of the roll, where the radii differ; only a category 2 stop keeps the web within its limits at every
  radius. Both rolls have the same core, full radius, motor and gearbox; the web's mass in the spans and the roll's own
  compliance are left out.
- Six-axis platform on cranks (`hexapod-rotary`): the time-optimal plan refuses a pure yaw from the level pose as
  singular, while roll and pitch plan (its shipped cycle yaws from the roll; the planner is unchanged). It ships with a
  settling tolerance of 1 mm. The rods' spin and the ball joints' friction are left out; one crank branch for all legs.
- The rotary knife was retuned for its margins across the bounds with friction off at a cost in following error (its
  cycle 8.86 -> 12.1 mrad); the flying saw and the toggle press were retuned for the same margins.
- The toggle press is crank-driven only; the screw-driven toggle (the knee on a nut) is not built.
- The measured ball screw's remaining differences (above): the screw shaft's inertia and the gearbox loss on the
  inertial force are model code left to a later piece of work.
- Speed: the hexapod and the spatial cable robot simulate slower than real time (about a quarter and a third of it on
  a 16-thread desktop busy with other work, 15 November 2026).
- Life estimates: no belt life (no maker publishes a method); the gearbox's gearing by Palmgren-Miner on Siemens'
  two torque figures, not Siemens' own damage method.
- Linear delta: the guides' moments (Mx, My, Mz), and warming belts and pulleys.
- Machine files: a self-locking worm's stick in the elastic model and once moving (it fades in over 0.5 W there); a
  scissor lift with its arms' turning in its mass, or driven by a cylinder between the arms; a linkage or scissor lift
  beside other mechanisms in one file.

## Front ends

- Labels such as `joint-1` are defined by several machine crates (`Joint 1`, `Leg 1`, `Crank 1`); a machine that defines
  none of its own shows the wording of the last registered machine that does (the selected machine's own wording wins).

- App: its layout at 1280 x 720 (the view's controls cover much of the machine, some tools tabs widen the tools panel
  by up to 160 pixels); the file dialogs, dragging a panel's edge and a plot's right-click menu are not tested; only
  the T-gantry's 3D view was looked at, and how the motion looks over time was seen in single pictures, not watched.
- FMI: asynchronous steps, a darwin64 binary in the shipped FMUs; import tested in Simulink or Dymola (checked with fmpy
  in CI, run in OpenModelica 1.27.1 through OMSimulator); the Modelica Association's `fmusim` and the FMU Compliance
  Checker. A Model Exchange FMU, which Modelica tools import as a block, is not written (docs/fmi.md says why).

## Release

- The Python package `kinematics-sim` is prepared for PyPI (metadata, trusted-publishing workflow) but not published.
- The Linux .deb and the MSI were built and inspected but never installed; signing them; packages for Linux
  distributions other than Debian and Ubuntu.
- The GitHub workflows restructured since the 1.0.0 cut (CI as partitions, fuzzing, docs site, Python publishing)
  have not run on GitHub yet.
