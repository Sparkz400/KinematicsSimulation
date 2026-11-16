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
| FMU output counts 114 / 60 / 108 | `docs/fmi.md` | **assumed**: copied from the audit, the FMUs were not regenerated | Whatever the current export gives | Export the three FMUs and count |
| Catalogue rows equal the vendors' datasheets | `catalog/**` | **assumed**: the audit checked internal consistency and a few values from memory, never the PDFs | Unknown per row | Spot-check rows per vendor against the source PDFs the extract scripts read |

## 2. Rules somebody chose (not derived from physics or a standard)

| What | Where | Why it is only a choice | Closing it |
| --- | --- | --- | --- |
| A catalogue swap's speed-loop gain keeps the config's crossover over the rotor-side inertia: on an elastic model at most Kp × J_M,new / J_M,old (J_M rotor and gearbox input), else the geometric-mean scaling | `catalog.rs` (`retuned`, `bounded_gain`), `docs/catalogue.md` | Fitted on 13 swept cases (single axis with radius, ratio, three ELZ axes and two motors; T-gantry with two motors; D3): their largest gain at 45° phase margin is Kp / J_M = 870 to 1550 rad/s, a fixed fraction of the first resonance does not fit (1.0 to 12 times it). It takes the config's own Kp / J_M as tuned; it is up to 0.6 times the largest stable gain (lighter rotor), and at an inertia ratio near 50 (1FK2103-4AH on ELZ-125) no gain passes under either rule | Sweep further machines (a delta and an arm with swapped motors); derive the drive's delay-limited crossover from its cycle times and filters instead of the config's |
| cable-spatial's speed loops at Tn 5 ms (was 30 ms) | `machine-cable-robot/config/spatial.toml` | The gears' mesh friction under the cables' pretension is a real dead band: by hand (eta, 250 N, drum, structure matrix at home) a 0.5 mrad turn needs 1.0, 0.8 and 0.4 times the band's torque from the loop at 1 Hz for rx, ry, rz, which matches the simulated 1.41 / 1.25 Hz / none at 30 ms; Tn by the symmetric optimum (80 Hz crossover, 60° margin) gives 5.6 / 5.2 / 4.4 Hz. The pretension stays 250 N: below 200 N the cycle takes a cable under 20 N. With it the machine friction feedforward has the meshes' friction on the tension at rest, and the config states a turn tolerance of 2 mrad (an estimate, as its 1 mm; the cycle's dwells reach 1.03 mrad): the PLCopen rz move ends 0.31 mrad past its target, 0.17 mrad 1 s later (was 0.55 / 0.20) | A real winch's mesh friction under tension and its presliding (`mech.rs` `MESH_STIFFNESS`, K1 by choice; the bands moved by under 10 % from 0.1 to 10 times it) |

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
