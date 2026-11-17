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
| FMU output counts 114 / 60 / 108 | `docs/fmi.md` | **assumed**: copied from the audit, the FMUs were not regenerated | Whatever the current export gives; each has one output more since the signal `setpoint_age` (2026-11-17) | Export the three FMUs and count |
| Stall current of the arms' 1FK2105-4AF in the parallel SCARA and the 4-axis delta (`[motor] stall_current_a` 4.35 A, the rated current) | `crates/machine-parallel-scara/default.toml`, `crates/machine-delta-d4/*.toml` | **chosen**: the comment calls the motor an estimate, but it is the catalogue's 1FK2105-4AF, whose printed stall current is 4.65 A (static torque 5 N m) | 4.65 A: S1 line from 5.0 N m, and a different iron-loss split | Set 4.65 A and re-check `motor-s1`, `motor-thermal` of motors 1 to 3 (the small motors were fixed on 2026-11-17: their own 1.06 A and 1.19 A from `motors_s210.csv`) |
| The D4 tool-turn motor 1FK2104-4AF's static torque equals its rated torque (1.27 N m at 1.19 A) | `catalog/siemens/motors_s210.csv`, `crates/machine-delta-d4/*.toml` `[motor.4]` | **assumed**: the CSV row prints both the same, so its S1 line is flat at 1.27 N m instead of starting a little above the rated torque as on the other motors | Unknown until read against D 32 | Read M0 and I0 of the 1FK2104-4AF in Siemens D 32 |
| Catalogue rows equal the vendors' datasheets | `catalog/**` | **assumed**: the audit checked internal consistency and a few values from memory, never the PDFs | Unknown per row | Spot-check rows per vendor against the source PDFs the extract scripts read |

## 2. Rules somebody chose (not derived from physics or a standard)

| What | Where | Why it is only a choice | Closing it |
| --- | --- | --- | --- |

## 3. Behaviour assumed without the manual

| What | Where | Assumed from | Closing it |
| --- | --- | --- | --- |
| A late fieldbus frame makes a drive with fine interpolation carry on extrapolating the old frame | `control.rs` (`profile_tick`) | Reasoning; no drive manual was read | Read one drive's behaviour on a missed setpoint (SINAMICS DSC, CiA 402 interpolated position) |

## 4. Derived but never run (as of this writing nothing in the batch has been executed)

Every fix was compile-checked only. The whole suite is being run once after the merge; until its
result is in, treat **every number below as unverified**.

- **The video tool** `crates/sim-app/tools/frames_to_mp4.py` (holds frames for dropped numbers).
- **`catalog/tools/extract_rexroth_ckr.py`** (writes `non_stroke_length_m`): changed, not run, its source PDF
  is not in the repository. (`catalog.py validate` and the tools' unit tests pass.)

## 5. Model simplifications still in the code (not true to the real machine)

| What the model leaves out | Where | Size of the error | Why it was left | Closing it |
| --- | --- | --- | --- | --- |
| The tool position error of a run file recorded before `setpoint_age` existed (2026-11-17) | `charts::axis_errors` | Up to a profile cycle's way at speed: 0.79 mm against 0.18 mm on the T-gantry's default cycle | Such a file has no age column; it falls back to the held setpoint | Record the run again |
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

## 9. Tests that depend on the host

| What | Where | Status | Closing it |
| --- | --- | --- | --- |
| OPC UA tests wait in wall time only against a hang (60 to 120 s); everything they assert is on the loop's ticks (`status.cycles`), the simulated time and the pacing's bookkeeping (`Pacing::lost_s`: simulated + dropped = wall time). The interop client still takes "not connected within 10 s" as refused | `crates/sim-opcua/tests/*.rs` | **chosen**: under a CPU-heavy build, 5 runs gave 10 failures before (5 of them from the load) and 5 after, all of them the palletiser row below (2026-11-17) | Nothing, unless the 10 s connect ever reads a slow accept as a refusal |
| `a_path_from_the_plc_is_planned_and_run` fails on the palletiser on any host: its time-optimal plan of the test path is 1.477 s against 1.517 s within the axis limits (1.392 s with the jerk limits alone), 2.6 % sooner where the test asks 4.1 % | `crates/sim-opcua/tests/plc.rs`, the planner (`path.rs`) and the palletiser config | **unverified** which changed: the planner or the palletiser config since the merges of 2026-11-16 | The owner of `path.rs` / the joint kit decides whether the plan or the test's expectation is wrong |
