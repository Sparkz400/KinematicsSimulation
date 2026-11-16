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
| "Strain-wave gears run at 70 to 85 %" | the audit's finding K5 | **from memory** of the auditor | same as above | same as above |
| Cushion energy "about 0.10 to 0.20 J" on the hanging pneumatic axis | `docs/mechanisms.md`, `docs/de/mechanismen.md` | **made up**: a hand estimate written before any run | Whatever the shipped cycle gives with the weight's work included | Run the shipped pneumatic axis hanging and write the measured figure |
| Mover's mass centre sits on the track centre line (`mass.centre_offset_m = 0.0`) | `crates/machine-mover-track/default.toml`, `flying.toml` | **made up** default: the mover is drawn 35 mm outside the centre line, the real mass offset is not known | With 35 mm the lateral acceleration at 2 m/s is 38.9 m/s², not 30.7 | Take the centre-of-mass offset from the vendor's mover data (or CAD) and set it |
| FMU output counts 114 / 60 / 108 | `docs/fmi.md` | **assumed**: copied from the audit, the FMUs were not regenerated | Whatever the current export gives | Export the three FMUs and count |
| Catalogue rows equal the vendors' datasheets | `catalog/**` | **assumed**: the audit checked internal consistency and a few values from memory, never the PDFs | Unknown per row | Spot-check rows per vendor against the source PDFs the extract scripts read |

## 2. Rules somebody chose (not derived from physics or a standard)

| What | Where | Why it is only a choice | Closing it |
| --- | --- | --- | --- |
| The mesh-friction bristle behind a screw, rack or drum is capped at a quarter of the integrator's rate bound | `transmission.rs` (`Converter::apply`) | A numerical cap, softer than the physical bristle; chosen so RK4 stays stable, never tuned by a run | Run cable robot and screw axis, check the energy budget, then either keep the cap with its measured error or integrate the bristle implicitly |
| The error band of "fitted minus config" is scaled by the ratio of residual norms | `identify.rs` (`unshared`) | The implementer's own estimate of the difference's error; not derived, not checked for coverage | Monte Carlo on a machine with known truth: (fitted − config)/error must have a standard deviation near 1 |
| Tolerances of new tests (settled rise 0.5 K, seven-joint inverse 1e-4) | `machine-mover-track`, `machine-joint-kit` tests | Picked before any run | Set from the first run |

## 3. Behaviour assumed without the manual

| What | Where | Assumed from | Closing it |
| --- | --- | --- | --- |
| KRL `CIRC`: components left out of the end point come from the previous position | `program.rs` | The module's own doc sentence and RAPID's MoveC; no KUKA manual was read | Read the KUKA System Software manual's CIRC section |
| G-code: a rotary-only block reads F in degrees per minute, a mixed block takes its time from the linear axes | `program.rs` (`path_feed`) | Common control behaviour (Siemens); Fanuc counts degrees as mm | State the dialect, or make it a setting |
| A late fieldbus frame makes a drive with fine interpolation carry on extrapolating the old frame | `control.rs` (`profile_tick`) | Reasoning; no drive manual was read | Read one drive's behaviour on a missed setpoint (SINAMICS DSC, CiA 402 interpolated position) |

## 4. Derived but never run (as of this writing nothing in the batch has been executed)

Every fix was compile-checked only. The whole suite is being run once after the merge; until its
result is in, treat **every number below as unverified**.

- **T-gantry belt mass, run by run** (`machine-t-gantry/src/lib.rs`: `belt_mass`, `belt_speed_terms`).
  The implementer's derivation gives a cross term 2·μ·(2x − sx)·vx·vz in the kinetic energy; the audit
  stated 4·μ. One of the two is wrong. Energy conservation of the lossless model after this change has
  not been shown.
- **T-gantry modes with the frame** (`modal`): expected 18.81 / 30.77 / 78.51 / 119.46 Hz at
  (0.9, 0.05) and 20.64 / 53.91 / 83.05 / 206.52 Hz at (0.5, 0.25) from the audit's linearisation; the
  new code holds the pulleys and leaves the belt's own mass out of the mass matrix.
- **Field-weakening braking runaway** (`drive.rs`, `current_control`): verified by run. The audit's case
  reproduced on commit `1d098351` (test motor of the t-gantry, saturation off, step to −3.8 N m held at
  400 rad/s: −19.11 N m at 42.47 A); now, stepped to ±3.8 N m and to the curve's torques at 380 to
  450 rad/s, saturation on and off, the torque stays within 3.8 N m (0.06 % step overshoot), the phase
  current within 9.5 A and the voltage vector within 190 V, and the torque settles at the curve
  (`braking_at_the_voltage_limit_does_not_run_away`). A command above the curve used to take a d current
  filling the current circle and give no torque; the d current is now the one of the most q current that
  fits. The shipped t-gantry and delta-d3 cycles keep every verdict (motor peaks 0.5406 / 0.4545 N m
  against 0.5400 / 0.4542 on `1d098351`, the change from the T-gantry belt mass; delta-d3 identical).
- **Induction torque curve and delivered torque agree** (`induction.rs`): verified by run. The control
  keeps the rated flux until the voltage binds at the torque commanded (`control_flux`, the largest flux
  that leaves room for it); only the curve takes the flux of the most torque. Asked for the curve's torque
  at 110 to 150 rad/s on the 540 V link the rotor-held simulation delivers it within 1 %
  (`asked_for_the_curves_torque_the_field_oriented_control_delivers_it`).
- **Induction flux of the most torque** (`induction.rs`, `flux_setpoint`): 27.29 N m at 0.774 Wb and
  115 rad/s and 20.68 N m at 0.650 Wb and 140 rad/s are now asserted by the Rust tests. The shipped
  `induction-lift` cycle is unchanged against `1d098351` (same verdicts, peak torque 5.79934 N m, RMS
  2.48832 N m, tracking error peak 0.28457 mm, all to the printed digits). Cost: the voltage circle's
  q current is solved by safeguarded Newton instead of a 40-step bisection (the curve at 120 rad/s took
  52.7 µs per call, now 3.5 µs); the lift cycle's run time is within the noise of the old code (fastest of
  six runs 5.90 s against 5.79 s).
- **V/f curve over the slip** (`induction.rs`, `vf_extreme`): the scan is right. A sweep of 200 000
  stator frequencies through an independently written per-phase circuit gives 17.949 N m at 10 rad/s and
  19.552 at 25 (22.247 at 50, 26.365 at 100, 20.683 at 140; generating breakdown at rated V and f
  −52.006 N m); the audit's 18.88 / 20.53 / 23.06 / 26.49 were not the circuit's maximum. The test now
  asserts 17.949 and 19.552.
- **Back-EMF constant apart from the torque constant** (`drive.rs`, `emf_loss`, budget row `emf.<motor>`): the
  row is the winding's step energies less copper, shaft and stored energy, derived by hand; nothing has run
  with a printed voltage constant, and the sign in braking (the row is negative) is a model limit, not physics.
- **Torque band with the inertia feedforward** (`control.rs`, `holding_torque`): the rigid model's torque at rest
  at the planned pose stands in for the held load; friction and the integrator's rest are not in it.
- **The time-optimal planner still keeps every limit** with the ramp share following curvature
  (`path.rs`, `ramp_share`): argued, not run.
- **Mesh friction with the bristle in joint units**: the cable robots blew up numerically and the screw
  axis lost 1.6 % of its energy budget in a trial at full stiffness; behaviour with the cap is unknown.
- **All hand numbers in the new regression tests** (guide loads 11.414 N and 2.762 N, momentum
  (1.875, 0.856, 5.693) N s, reactance limits 3.474 / 2.709 / 1.956 N m, steady friction totals,
  the full circle's length, and so on).
- **The 2-axis delta's reference file** `crates/machine-delta-d2/reference/inverse_dynamics.csv` is
  stale: its generator was changed but could not be run (no sympy).
- **The video tool** `crates/sim-app/tools/frames_to_mp4.py` (holds frames for dropped numbers).
- **The Python catalogue checks** after the linear-motor back-EMF column was converted.

## 5. Model simplifications still in the code (not true to the real machine)

| What the model leaves out | Where | Size of the error | Why it was left | Closing it |
| --- | --- | --- | --- | --- |
| Feed-forward of the T-gantry has the belt's mass terms but not its speed terms | `machine-t-gantry/src/lib.rs` (`feedforward_torque`) | small (belt mass 0.1 kg/m) | The method takes no velocity | Use the moving feed-forward where it exists |
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
  per Bahr ELZ size are in the vendor-facts file; the stroke limit can now be module length minus those.
- **Mover's mass centre** (section 1): not published by Beckhoff in anything readable; needs the
  vendor's load diagrams, CAD or a question to their support. Mass 651 g is sourced.
- **Late fieldbus frame** (section 3): no primary text found; SINAMICS only documents the tolerated
  number of missed sign-of-life counts (p0925, fault F01912), not what the setpoint does meanwhile.
- **Stepper current convention** (section 3): the catalogue has no stepper vendor, so there is nothing
  to check it against.
