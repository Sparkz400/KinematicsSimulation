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
| The error band of "fitted minus config" is scaled by the ratio of residual norms | `identify.rs` (`unshared`) | The implementer's own estimate of the difference's error; not derived, not checked for coverage | Monte Carlo on a machine with known truth: (fitted − config)/error must have a standard deviation near 1 |
| A retuned velocity gain that grows is bounded by Kp / (rotor + gearbox inertia) <= 2 pi f_res / 3, never below the config's own | `catalog.rs` (`retuned`, `bounded_gain`) | The third is the usual separation of a speed loop from a resonance; the rotor-side inertia was chosen because the load-side one bounded nothing. Checked on one case only: the single axis with the Bahr ELZ-125 (0.0573 oscillated, 0.045 too, 0.0368 passed; the bound gives 0.025, which passes). In effect a gain grows with the load only on a machine that reports no resonance | Find the stable gain by run for a few axes on several machines and fit the rule to it |
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
- **Field-weakening braking runaway** (`drive.rs`, `current_control`): fixed by a different route than
  planned; the original case (−19.1 N m at 42.5 A near 400 rad/s with saturation off) was never
  reproduced before or after.
- **Induction torque curve and delivered torque agree** (`induction.rs`): by construction, not by run.
- **Induction flux of the most torque** (`induction.rs`, `flux_setpoint`): 27.29 N m at 0.774 Wb and 115 rad/s
  (540 V link) come from an independent scan in Python, not from the Rust; the shipped `induction-lift` cycle
  now runs with a weaker flux from about 100 rad/s on, and nobody has seen it run. Cost per base step in the
  weakened range (a golden section of about 1700 voltage evaluations) unmeasured.
- **Back-EMF constant apart from the torque constant** (`drive.rs`, `emf_loss`, budget row `emf.<motor>`): the
  row is the winding's step energies less copper, shaft and stored energy, derived by hand. A catalogue motor's
  printed constant now reaches the drive (every shipped config and machine file carries
  `drive.back_emf_constant_v_s_rad = 0.0`, which `apply_motor` overwrites; checked for a Kollmorgen AKM and a
  Beckhoff AM8000), but no cycle's energy budget has been run with it, and the sign in braking (the row is
  negative) is a model limit, not physics.
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
- **`catalog/tools/extract_rexroth_ckr.py`** (writes `non_stroke_length_m`): changed, not run, its source PDF
  is not in the repository. (`catalog.py validate` and the tools' unit tests pass.)

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
