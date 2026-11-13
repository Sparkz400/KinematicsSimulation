# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the project uses
[Semantic Versioning](https://semver.org/spec/v2.0.0.html). Details, test names and the numbers behind each
item are in [PROGRESS.md](PROGRESS.md).

## [Unreleased]

### Added

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
- Machine files: a machine defined in a TOML or JSON file as a netlist of components (motor, gearbox, pulley,
  belt sections, bodies, guides) with the tables of a machine config, loaded at run time by `sim-cli --machine-file`,
  the app (Open, or `sim-app --machine-file`) and `sim-opcua --machine-file`; versioned, checked with file, line and
  key in every error, with a generated JSON Schema (`schema/machine-file.schema.json`). The single axis and the
  T-gantry as machine files reproduce their crates (`machines/`, `docs/machine-files.md`).
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

## [1.0.0-rc.1] - 2026-11-13

Stages A to D of the plan's end goals, the effect views, the electrical drive and the FMI export.

### Added

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

### Fixed

- Nine rounds of review by experiment; each fix has a test that was seen failing without it (PROGRESS.md,
  "Review fixes" to "Ninth review fixes").

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
