# What is estimated, assumed, unverified or left open

Written 2026-11-16, after the physics audit and the fix batch that followed it (16 fix branches
merged into `main`, starting from commit `1d098351`). This file lists everything in that batch,
and everything the audit met, that is **not taken from reality**: values without a source, rules
somebody chose, behaviour assumed without a manual, numbers nobody has run, and model
simplifications that are still in the code.

None of the open items is impossible. Each row says why it was left and what closing it takes.
Remove a row when it is closed, with the commit that closed it.

Status words: **made up** (no source at all), **chosen** (a judgement, defensible but not derived),
**assumed** (believed, not checked against the source), **unverified** (derived, never run),
**simplified** (the model knowingly leaves something out).

## 1. Values with no source (made up or from memory)

| What | Where | Status | What is true instead | Closing it |
| --- | --- | --- | --- | --- |
| Cushion energy "about 0.10 to 0.20 J" on the hanging pneumatic axis | `docs/mechanisms.md`, `docs/de/mechanismen.md` | **made up**: a hand estimate written before any run | Whatever the shipped cycle gives with the weight's work included | Run the shipped pneumatic axis hanging and write the measured figure |
| Mover's mass centre sits on the track centre line (`mass.centre_offset_m = 0.0`) | `crates/machine-mover-track/default.toml`, `flying.toml` | **made up** default: the mover is drawn 35 mm outside the centre line, the real mass offset is not known | With 35 mm the lateral acceleration at 2 m/s is 38.9 m/s², not 30.7 | Take the centre-of-mass offset from the vendor's mover data (or CAD) and set it |
| FMU output counts 114 / 60 / 108 | `docs/fmi.md` | **assumed**: copied from the audit, the FMUs were not regenerated | Whatever the current export gives | Export the three FMUs and count |
| Catalogue rows equal the vendors' datasheets | `catalog/**` | **assumed**: the audit checked internal consistency and a few values from memory, never the PDFs | Unknown per row | Spot-check rows per vendor against the source PDFs the extract scripts read |

## 2. Rules somebody chose (not derived from physics or a standard)

| What | Where | Why it is only a choice | Closing it |
| --- | --- | --- | --- |
| The mesh-friction bristle behind a screw, rack or drum is capped at a quarter of the integrator's rate bound | `transmission.rs` (`Converter::apply`) | A numerical cap, softer than the physical bristle; chosen so RK4 stays stable, never tuned by a run | Run cable robot and screw axis, check the energy budget, then either keep the cap with its measured error or integrate the bristle implicitly |
| Tolerances of new tests (settled rise 0.5 K, seven-joint inverse 1e-4) | `machine-mover-track`, `machine-joint-kit` tests | Picked before any run | Set from the first run |
| The binormal part v³κτ of a 3D spline's jerk may take at most 0.3 of an axis's jerk limit in the plan within the axis limits | `path.rs` (`BINORMAL_SHARE`, `curved`) | A choice: the other two parts then keep sqrt(1 − 0.3²) = 0.954 of theirs. Measured: on the 3D spline of `a_spline_on_three_axes_passes_through_its_points_within_the_limits` (0.697 s, axis jerk used to 1.0000) the time is the same at 0.15, 0.3 and 0.6: the cap does not bind there; in the time-optimal plan the share plays no part (its bend rows take the whole p''' by differences) | Keep; derive it only if a machine's 3D splines come out jerk-limited by it |
| Friction is not counted as help while a motor brakes in the time-optimal plan | `path.rs` (`MotorCurve`, `Place::bounds`) | A choice: friction is nil at a standstill and the bounds hold at every speed up to v. Measured: the friction left unused where the braking torque binds is 0.0213 N m of 0.48 N m on the T-gantry with a 0.6 N m motor (4.4 %), 0.0237 of 0.4 N m on the single axis (5.9 %), 0.34 of 12 N m on the delta-d3 (2.8 %), so braking is planned that much below what the motor could do | Keep; count it above a speed where it is sure to be there if the few per cent matter |

## 3. Behaviour assumed without the manual

| What | Where | Assumed from | Closing it |
| --- | --- | --- | --- |
| A late fieldbus frame makes a drive with fine interpolation carry on extrapolating the old frame | `control.rs` (`profile_tick`) | Reasoning; no drive manual was read | Read one drive's behaviour on a missed setpoint (SINAMICS DSC, CiA 402 interpolated position) |

## 4. Derived but never run (as of this writing nothing in the batch has been executed)

Every fix was compile-checked only. The whole suite is being run once after the merge; until its
result is in, treat **every number below as unverified**.

- **Back-EMF constant apart from the torque constant** (`drive.rs`, `emf_loss`, budget row `emf.<motor>`): the
  row is the winding's step energies less copper, shaft and stored energy, derived by hand. A catalogue motor's
  printed constant now reaches the drive (every shipped config and machine file carries
  `drive.back_emf_constant_v_s_rad = 0.0`, which `apply_motor` overwrites; checked for a Kollmorgen AKM and a
  Beckhoff AM8000), but no cycle's energy budget has been run with it, and the sign in braking (the row is
  negative) is a model limit, not physics.
- **Torque band with the inertia feedforward** (`control.rs`, `holding_torque`): the rigid model's torque at rest
  at the planned pose stands in for the held load; friction and the integrator's rest are not in it.
- **The time-optimal planner keeps every limit: verified by run on 16 November** (`crates/sim-machines/tests/planned_limits.rs`):
  T-gantry, delta-d3, linear delta, SCARA in tool space and single axis, loss-free, shipped and shipped with twenty times
  the jerk, on lines, arcs (radius 100 m, half turns), 3D splines, a helix, corners, and the single axis braking
  through a gearbox of efficiency 0.9 with a heavy rotor and on ±30°: tool jerk, motor speed, motor torque by the
  machine's own inverse dynamics through η the way the power flows, gearbox output, belt tension with the speed,
  guide loads and own checks, all at most 1.0001 of their planned bounds. The run found the braking rows unsafe where
  the rotor's inertia outweighs the load (fixed: `Place::against`). A nearly straight arc (100 m) now takes 0.620 s
  against 0.619 s for its chord (0.684 s against 0.504 s before; the chord itself is slower now on the batch's
  T-gantry).
- **Found by that run, open:** the D3's inverse kinematics (`sim-netlist/src/spatial.rs`, `Legs::inverse_moved`,
  `2 atan((−b − √d) / (c − a))`) gives motor 2 an angle of 0 at (−0.15, −0.0866, −0.9) exactly (c = a), so a
  time-optimal line through that point crawls (23.6 s for 0.2 m; `a_path_in_a_tilted_plane_gets_its_transition_curves`
  still passes). The palletiser's time-optimal cycle fails its `tool-angular-acceleration` check (13.5 against 10):
  the check's value is a vector's length, which the plan's fit c a + b v + d v² + e cannot hold (as `wafer-slip`);
  `rigid_checks` should give NaN for it or its components.
- **Mesh friction with the bristle in joint units**: the cable robots blew up numerically and the screw
  axis lost 1.6 % of its energy budget in a trial at full stiffness; behaviour with the cap is unknown.
- **All hand numbers in the new regression tests** (guide loads 11.414 N and 2.762 N, momentum
  (1.875, 0.856, 5.693) N s, reactance limits 3.474 / 2.709 / 1.956 N m, steady friction totals,
  and so on; the full circle's length is now run).
- **The 2-axis delta's reference file** `crates/machine-delta-d2/reference/inverse_dynamics.csv` is
  stale: its generator was changed but could not be run (no sympy).
- **The video tool** `crates/sim-app/tools/frames_to_mp4.py` (holds frames for dropped numbers).
- **`catalog/tools/extract_rexroth_ckr.py`** (writes `non_stroke_length_m`): changed, not run, its source PDF
  is not in the repository. (`catalog.py validate` and the tools' unit tests pass.)

## 5. Model simplifications still in the code (not true to the real machine)

| What the model leaves out | Where | Size of the error | Why it was left | Closing it |
| --- | --- | --- | --- | --- |
| Pre-existing estimates the audit did not judge: tooth-jump factor, brake deflection and slip times, the efficiency fade constant 0.5 W, placeholder drive currents in shipped configs | `mech.rs`, shipped `default.toml` files | unknown | Outside the audit's reach without measurements | Measure, or mark each in its config comment |

## 6. Documents that do not match the program

- `docs/user-guide.md` and `docs/de/benutzerhandbuch.md`: 41 of 51 worked command outputs no longer
  match the program (before this batch already), and the batch changes more numbers. They are
  regenerated after the suite is green.
- `docs/validation.md`: rows 212 and 214 still quote the old T-gantry mode numbers.
- `docs/de/*`: not checked for the statements changed in the English docs (modes, belt mass, ramp share).
- `docs/fmi.md`: the validation logs are from 14 November 2026.

## 7. What "validated" means in this project today

As far as this batch knows, no drive trace measured on one of this project's own machines exists; the
one measured data set in the repository is the external ISW set under `datasets/`. Apart from what is
compared with that set, a "matches" in `docs/validation.md` compares the program with a hand
calculation or with itself, not with hardware. (Not re-read line by line for this file: check
`docs/validation.md` before quoting this.)

## 8. Vendor facts looked up on 2026-11-16

`docs/development/vendor-facts.md` holds what the vendors' own documents say, with document, page
and link. What it changes in the tables above (rows stay until the code follows):

- **KRL CIRC** (section 3): sourced. KUKA System Software 8.3, section 11.5.3: missing end-point
  components take the previous position's values. The code now does this.
- **G-code feed** (section 3): sourced for Siemens SINUMERIK (Fundamentals manual, section 8.1): a
  rotary-only block reads F in degrees per minute; with the default FGROUP (geometry axes) a rotary
  axis in a mixed block is a synchronised axis and follows in the linear axes' time. Fanuc: no primary
  source found.
- **Rexroth CKR and Bahr stroke** (section 5): sourced. Lca and Lad per CKR size and the basic length
  per Bahr ELZ size are in the vendor-facts file, and the code follows: the stroke limit is the module
  length minus `non_stroke_length_m`. The other Bahr series keep the module length, flagged `axis-stroke`.
- **Harmonic Drive efficiency**: read off the vendor's graphs (0.699 to 0.800, about 1 point reading error)
  for every CSF / CSG / SHG row; the flat 0.70 estimate applies to no shipped row.
- **Mover's mass centre** (section 1): not published by Beckhoff in anything readable; needs the
  vendor's load diagrams, CAD or a question to their support. Mass 651 g is sourced.
- **Late fieldbus frame** (section 3): no primary text found; SINAMICS only documents the tolerated
  number of missed sign-of-life counts (p0925, fault F01912), not what the setpoint does meanwhile.
- **Stepper current convention** (section 3): the catalogue has no stepper vendor, so there is nothing
  to check it against.
