# Roadmap

What is not built yet, from the "Left open" lists at the end of each section of [PROGRESS.md](PROGRESS.md). No dates.

## Needs a real machine or data

- Test runs on a real axis and acceptance on a measured cycle (stages D2, D4). The importers for SINAMICS,
  TwinCAT and CODESYS traces, the identification (`sim-cli identify`, `sim-cli fit` for linear machines and, in the
  motors' coordinates, for the deltas, the SCARA and the arm) and the belt stiffness from a move's trace are built
  and tested on traces the simulator made and on files built after the vendors' layouts; none has read a real
  drive's trace yet. The layout of the Startdrive CSV export is not documented by Siemens and is assumed.
- A real PLC (Siemens, Beckhoff, CODESYS) against the OPC UA server. The server is prepared (Basic256Sha256
  endpoints, trusted client certificates, user and password, a checked type table, the clients' cycle jitter, Structured
  Text programs for TwinCAT and CODESYS, docs/plc.md); no PLC has connected yet and the ST files are not compiled.
- The S210's DC link capacitance and chopper threshold: Siemens publishes neither. The capacitance is measured for
  one converter (6SL5310-1BE10-8DF0, `dc_link_measured.csv`); for every other converter both stay estimates.
- The motors' own bearings: Siemens gives their allowable radial forces only as diagrams; one frame size
  (1FK2205) is read into the catalogue, the others are not.
- The planar, linear and four-axis deltas: their makers publish no geometry or masses; those are estimates.

## Physics and planning

- Time-optimal plan: a piece still cruises at one speed between its ramps, and the top speed above its slowest place
  is found by trial; the drive's current limit, the gearbox and pulley torque ratings and the guide loads are judged
  after planning, not limits of the plan.
- Life estimates: no belt life (no maker publishes a method); the gearbox's gearing by Palmgren-Miner on Siemens'
  two torque figures, not Siemens' own damage method.
- The drive: one maximum current, no overload over time (i2t).
- Delta D3: the arm tube's own share of the bending modes and the elbow block's twist as an arm bends out of its
  plane. Linear delta: vertical columns only; no guide, pulley-bearing or belt-tooth ratings (its axes are
  not in the catalogue).
- Fit: one added inertia per motor, not a mass factor per link of a URDF arm; a payload picked or dropped during a
  trace is not followed.
- Optimiser: the robust check uses the error bands one bound at a time, not the worst corner of all of them; one lift
  height per traverse, not a height profile over the obstacles.
- Machine files: a self-locking worm's stick in the elastic model and once moving (it fades in over 0.5 W there); a
  scissor lift with its arms' turning in its mass, or driven by a cylinder between the arms; a linkage or scissor lift
  beside other mechanisms in one file.

## Front ends

- In the app: `--set` (values are changed in the parameter panel).
- FMI: asynchronous steps, a darwin64 binary; import tested in Simulink or Dymola (checked with fmpy in CI, run in
  OpenModelica 1.27.1 through OMSimulator). A Model Exchange FMU, which Modelica tools import as a block, is not
  written (docs/fmi.md says why). Restoring an FMU state replays every step from initialisation, so it costs as long
  as simulating to that point.

## Release

- Signing the MSI and the .deb; packages for Linux distributions other than Debian and Ubuntu.
