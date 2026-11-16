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
| A move timed by the axis limits keeps the flange's spin with half its angular acceleration limit at most for the g₂ ṡ² part, g₁ and g₂ sampled at 65 places along the move | `profile.rs` (`spin_rates`, `Move::with_law`) | Any split keeps the limit; this one is conservative (the six-axis arm's pure 0.5 rad turn at the home spins at 1.35 of 1.5 rad/s) | Time the move along s against the exact bound, as the time-optimal planner does |

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
| The flange's spin on a path of an arm with seven tool axes (arm on track): its lines are not slerps, so `path::plan_within` bounds only the rotation vector's rates there | `path.rs` (`shape`, `plan_within`) | as before the fix: up to 1.27 times the limit at the home | Only six-axis tool poses turn by slerp | Slerp the orientation whatever axes stand beside it |
| The ram's acceleration of the toggle press leaves out the stretch's share that follows the crank's jerk, where the press force fades as the ram rises (in the press zone going down it is the derivative of `ram.speed`) | `toggle.rs` (`signals`) | only in the fade-out band of `process.smoothing_m_s` | The signals get no jerk | Pass the jerk, or difference the speed signal over time |
| The torque-band monitor expects the friction at the planned speed (characteristic or `steady_friction`), none at rest: a breakaway or stiction at a standstill is not expected | `control.rs` (`setpoints`), `fault.rs` | up to the breakaway torque at a reversal or a start | `steady_friction` is a sliding level | Expect the breakaway band at zero speed, or widen the band there |
| The DC link's running account keeps every stretch until the link first empties or fills; a link that does neither over a long run (net zero power at rest, no braking reaching its room) still keeps them all. The room is the run's config's; a verdict with another room falls back to `regeneration_events`, which a run leaves empty | `analysis.rs` (`LinkAccount`, `Tally`) | none in the shipped runs (the T-gantry's and the single axis's figures unchanged) | Exact for every room needs the whole list | Merge stretches below a share of the room once that has no merge |
| The H4's small-amplitude modes take Dahl's σ as K1 (√2 above `modes`), as the D3's do | `machine-delta-h4/src/lib.rs` (`modes_at`), `mech.rs` | unknown; no maker publishes the loop's shape | Ships with hysteresis 0, so nothing changes today | A measured hysteresis loop, then σ as a config key |
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
