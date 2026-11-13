# Roadmap

What is not built yet, from the "Not built" and "Blocked" lists of [PROGRESS.md](PROGRESS.md). No dates.

## Needs a real machine

- Test runs on a real axis and acceptance on a measured cycle (stages D2, D4). The importers for SINAMICS,
  TwinCAT and CODESYS traces and the machine-free identification (`sim-cli identify`: friction, inertia,
  two-mass stiffness and damping, and the fit's stage 3, drive delays and filters) are built and tested on
  synthetic traces and on files built after the vendors' layouts; none has read a real drive's trace yet.
  The layout of the Startdrive CSV export is not documented by Siemens and is assumed.
- A real PLC (Siemens, Beckhoff, CODESYS) against the OPC UA server: data types, security, cycle jitter.

## Physics and planning

- A time-optimal plan whose limits change within a piece (within a spline span).
- Transition curves: one length per path today; none at arcs fixed at both ends.
- Life estimates: the motors' own bearings (Siemens publishes their radial force limits only as diagrams in the
  configuration manuals), belt life, gearbox gearing fatigue above rated torque, the life factor a_ISO.
- The S210's DC link capacitance and chopper threshold: Siemens publishes neither. The capacitance is measured for
  one converter (6SL5310-1BE10-8DF0, `dc_link_measured.csv`); for every other converter both stay estimates. The
  chopper threshold needs a stop that reaches it (more braking energy, or a smaller DC link).

## Front ends

- In the app: `--set` and the operating points of a path (command line only today).
- FMI: FMU state, directional derivatives, a darwin64 binary, import tested in Simulink,
  OpenModelica or Dymola.

## Release

- Signing the MSI and the .deb; packages for Linux distributions other than Debian and Ubuntu.
- The CI and release workflows have never run: the repository has no GitHub remote yet.
