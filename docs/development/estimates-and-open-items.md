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
| The mesh friction's bristle is K1 (`MESH_STIFFNESS`) | `mech.rs` | Taken as the hysteresis's Dahl stiffness. The energy budget no longer chooses it: with the friction booked as its whole power the single axis and the T-gantry close their passes at K1, 3 K1 and 10 K1 alike (under 1e-6 and 4e-6 unaccounted; gearbox loss 3.04659 J and 8.70004 to 8.70029 J), so the T-gantry is back in `the_energy_budget_closes_with_the_mesh_friction_on_every_gearbox` and its config has the switch (off, as on the other belt machines) | Measure a gear's presliding |
| The split of a machine file's section lengths into runs (`machine.span`) on the CoreXY and the portals | `machines/corexy.toml`, `xyz-portal.toml`, `portal-c.toml`, `dual-gantry.toml`, `rack-gantry.toml` | Their section lengths are placeholders shorter than the drawn loops, so the runs share them out (the table in each file); the CoreXY's -0.01 kg between X and Y comes from that split (equal belts would give 0). The portals' moved masses do not depend on it | Lengths from a real machine's drawing |

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
| The belt's mass run by run is in the T-gantry and in every machine file whose belt runs move over ground (H-bot, CoreXY, XYZ and C portal, dual and rack gantry); the dual gantry's runs do not move with the bridge's yaw | `machines/dual-gantry.toml` | the yaw's inertia of the 0.11 kg belt at its lever, under 0.1 % | A span's speed over ground is constant per axis speed; the yaw's lever depends on the pose | Let a span's speed depend on the pose |
| The guide load's acceleration (`commanded_acc`) leaves the belt's runs out, as it leaves chains and idlers | `sim-netlist/src/machine.rs` | the H-bot's X: 0.32 kg on 11 kg, 3 % of the tilt | The crates take it the same way; the true acceleration would make friction depend on itself | Add `runs_mass` in file and crates together |
| The friction feedforward's velocity terms are loss-free (`feedforward_torque_moving`); `steady_friction` carries the loss on the weight and the friction (planar linkage, spatial D3/D4, T-gantry, single axis) | `linkage.rs`, `spatial.rs` | 3.5e-4 of the torque on the D4 at 0.5 m/s, 4e-5 on the linkage at 0.05 m/s (against a Newton solve of the rigid model) | Small at the speeds the deltas run | Put the loss factor on the velocity terms in the feedforward |
| The mesh bristle's cap behind a screw, rack or drum knows the output's inertia only where the machine passes it (`Converter::apply_to`: the cable robots' drums); `apply` caps by the rotor's alone | `transmission.rs` | none on a shipped machine (no gear reaches either cap) | The screw, rack and crane builders do not pass their load's mass | Pass the carried mass in each builder |
| Energy rate against the motors' power, loss-free, off by up to 9e-7 where the machine itself is (with the belt's mass at 0 too): the C portal (rigid 1.6e-7, elastic 7.5e-7; its rigid feedforward misses 4e-6 of the acceleration) and the dual gantry (elastic 9e-7; feedforward 5.5e-6) | `sim-netlist/src/machine.rs` (`body_jacobian` differences places over 1e-6; the yawing bridge) | below 1e-6 of the power | Found while checking the belt's runs, which add nothing to it (`the_belts_runs_keep_the_energy_and_give_the_moved_mass_by_hand`) | Analytic body Jacobian and its rate; find the dual gantry's share |

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
