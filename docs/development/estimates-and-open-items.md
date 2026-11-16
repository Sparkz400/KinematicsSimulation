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
| Harmonic Drive strain-wave efficiency 0.70 for every CSF/CSG/SHG size and ratio | `crates/sim-components/src/catalog.rs` (`apply_gearbox`), flagged `gearbox-efficiency` in the search's `estimated` column | **made up**: typed from memory by the implementer, who could not open the vendor's charts | The vendor prints efficiency charts by ratio, speed, temperature and load; real values differ by ratio and are mostly above 0.70 at rated torque | Read the chart values from the Harmonic Drive catalogue into an efficiency column per series and ratio |
| "Strain-wave gears run at 70 to 85 %" | the audit's finding K5 | **from memory** of the auditor | same as above | same as above |
| Cushion energy "about 0.10 to 0.20 J" on the hanging pneumatic axis | `docs/mechanisms.md`, `docs/de/mechanismen.md` | **made up**: a hand estimate written before any run | Whatever the shipped cycle gives with the weight's work included | Run the shipped pneumatic axis hanging and write the measured figure |
| The cushion must absorb the weight's work over the cushion length | `crates/sim-components/src/pneumatic.rs` (`Cylinder::stepped`) | **assumed**: physically sound, but whether Festo's 0.4 J rating is meant to include it was never read in the catalogue | Unknown until the Festo DSNU/ADN sizing rule is read | Read the vendor's definition of the permissible impact energy |
| Mover's mass centre sits on the track centre line (`mass.centre_offset_m = 0.0`) | `crates/machine-mover-track/default.toml`, `flying.toml` | **made up** default: the mover is drawn 35 mm outside the centre line, the real mass offset is not known | With 35 mm the lateral acceleration at 2 m/s is 38.9 m/s², not 30.7 | Take the centre-of-mass offset from the vendor's mover data (or CAD) and set it |
| FMU output counts 114 / 60 / 108 | `docs/fmi.md` | **assumed**: copied from the audit, the FMUs were not regenerated | Whatever the current export gives | Export the three FMUs and count |
| Catalogue rows equal the vendors' datasheets | `catalog/**` | **assumed**: the audit checked internal consistency and a few values from memory, never the PDFs | Unknown per row | Spot-check rows per vendor against the source PDFs the extract scripts read |

## 2. Rules somebody chose (not derived from physics or a standard)

| What | Where | Why it is only a choice | Closing it |
| --- | --- | --- | --- |
| The induction motor's flux is held so the no-load voltage uses 0.9 of the supply | `induction.rs` (`flux_setpoint`) | 0.9 is a round number, not the torque-optimal flux (near 1/√2 of the voltage for the stator's share) | Derive the flux that gives the most torque inside the voltage and current limits at each speed |
| The mesh-friction bristle behind a screw, rack or drum is capped at a quarter of the integrator's rate bound | `transmission.rs` (`Converter::apply`) | A numerical cap, softer than the physical bristle; chosen so RK4 stays stable, never tuned by a run | Run cable robot and screw axis, check the energy budget, then either keep the cap with its measured error or integrate the bristle implicitly |
| The error band of "fitted minus config" is scaled by the ratio of residual norms | `identify.rs` (`unshared`) | The implementer's own estimate of the difference's error; not derived, not checked for coverage | Monte Carlo on a machine with known truth: (fitted − config)/error must have a standard deviation near 1 |
| V/f breakdown torque by a 96-point scan over slip | `induction.rs` (`vf_extreme`) | Resolution chosen by eye | Compare with the closed-form Thevenin breakdown at fixed frequency |
| A half-turn arc is laid in the first coordinate plane that contains its start radius, x-y first | `path.rs` (`arc3`) | A convention: a G18 or G19 semicircle whose start radius lies in two coordinate planes lands in the wrong plane | Carry the plane on the arc piece, set by G17/G18/G19 in `program.rs` |
| The pinion's separating force always loads the worst guide block | `machine-joint-kit/src/arm.rs` (`guide_load`) | The config does not say on which side the rack sits, so the sign is taken as worst case | Add the rack side to the config |
| Reported delta modes use the gearbox's large-amplitude stiffness K1 | `sim-netlist/src/spatial.rs` (`modes_at`) | Small vibrations see K1 + σ = 2·K1 (49 / 51 / 84 Hz become 69 / 73 / 119 Hz). Which one a shaper should aim at depends on amplitude | Report both, and let the shaper use the one for the residual amplitude it fights |
| Tolerances of new tests (settled rise 0.5 K, seven-joint inverse 1e-4) | `machine-mover-track`, `machine-joint-kit` tests | Picked before any run | Set from the first run |

## 3. Behaviour assumed without the manual

| What | Where | Assumed from | Closing it |
| --- | --- | --- | --- |
| KRL `CIRC`: components left out of the end point come from the previous position | `program.rs` | The module's own doc sentence and RAPID's MoveC; no KUKA manual was read | Read the KUKA System Software manual's CIRC section |
| G-code: a rotary-only block reads F in degrees per minute, a mixed block takes its time from the linear axes | `program.rs` (`path_feed`) | Common control behaviour (Siemens); Fanuc counts degrees as mm | State the dialect, or make it a setting |
| A late fieldbus frame makes a drive with fine interpolation carry on extrapolating the old frame | `control.rs` (`profile_tick`) | Reasoning; no drive manual was read | Read one drive's behaviour on a missed setpoint (SINAMICS DSC, CiA 402 interpolated position) |
| Stepper drive current ratings are the phase-peak amplitude | `docs/config-reference.md` | The model's own convention; datasheets often print RMS | Check per catalogue vendor and convert where they print RMS |

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
| Simulated back-EMF uses the torque constant, not the printed voltage constant (only the reported no-load speed uses the printed one) | `drive.rs` | 10 to 15 % on the voltage limit for Beckhoff, Kollmorgen, SEW, Rexroth; about 5 % of that is real magnet temperature | The drive model has one constant; the implementer's group did not own `drive.rs` | Give the drive a separate back-EMF constant |
| Binormal jerk v³κτ on 3D spline spans | `path.rs` (`limits`) | worst 0.71 of the jerk limit seen, none exceeded | Needs torsion stored on the spline piece; more than the brief's "few lines" | Store torsion, add the term |
| rx / ry / rz limits act on rotation-vector rates, not angular velocity | `machine-joint-kit/src/tool.rs` | at the home pose a 1.5 rad/s limit allows a 1.91 rad/s spin | Converting the limit check was not a local change | Check limits on the geometric angular velocity |
| Feed-forward of the T-gantry has the belt's mass terms but not its speed terms | `machine-t-gantry/src/lib.rs` (`feedforward_torque`) | small (belt mass 0.1 kg/m) | The method takes no velocity | Use the moving feed-forward where it exists |
| 3-axis delta's lumped forearm model weights the forearm with the 2/3 share | `sim-netlist/src/spatial.rs` | 1 to 3 % of holding torque; the shipped config uses the exact rod model | The finding named the 2-axis delta only | Apply the half split there too |
| Braking is planned with the gearbox loss against the motor | `path.rs` (`piece_limits`) | 0.8 % of the time on a single-axis line (conservative) | Documented choice | Take the braking side with η |
| Guide life comes from one guide | `analysis.rs` (`Tally::finish`) | can differ from the true minimum when guides run at very different speeds | The result structure carries one pair | Carry a pair per guide |
| "motor-speed" label says rad/s on linear motors | `sim-components/i18n/*.ftl`, `sim-app/src/model.rs` | label only | The text has the unit baked in and three groups owned the three files | Build the label from the motor's kind |
| CLI, Python and browser column names say kg, N, N m, rad/s when the axis is rotary or the motor linear | `sim-cli`, `sim-py`, `sim-wasm` | names only; usage text and docstrings now say so | Names are an interface | Add a unit column |
| Torque-band monitor trips on a held load with the "inertia" feed-forward | `fault.rs` | 11 % of peak torque on the lift | Adding the integrator would blind the monitor | Compare against feed-forward plus gravity |
| A full circle given with R is refused; consecutive mixed linear/rotary G-code blocks stop between each other | `program.rs` | behaviour, not a wrong number | An R circle is ambiguous; mixed blocks get different caps | Blend mixed blocks |
| With iron loss off, the drawn S1 line may be flat while the verdict reads the stall current | `charts.rs` / `analysis.rs` | chart only | Needs one field from `drive.rs` | Use the same source in both |
| Tool position error falls back to the motor-side following error when a run has no setpoint signal | `sim-app/src/runs.rs` | 2.4 to 15 times too small when it falls back | Old recordings lack the signal | Check every machine emits `axis.<id>.position_setpoint` |
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
- **Festo cushioning** (section 1): the vendor's rule for the cylinder's own cushioning is kinetic
  energy only, v = sqrt(2E/(m1 + m2)); the weight term belongs to Festo's rule for external shock
  absorbers. The weight term added to `pneumatic.rs` in this batch is therefore NOT the vendor's check
  and is to be taken out again (keep it, if at all, as a separate signal).
- **G-code feed** (section 3): sourced for Siemens SINUMERIK (Fundamentals manual, section 8.1): a
  rotary-only block reads F in degrees per minute; with the default FGROUP (geometry axes) a rotary
  axis in a mixed block is a synchronised axis and follows in the linear axes' time. Fanuc: no primary
  source found.
- **Rexroth CKR and Bahr stroke** (section 5): sourced. Lca and Lad per CKR size and the basic length
  per Bahr ELZ size are in the vendor-facts file; the stroke limit can now be module length minus those.
- **Harmonic Drive efficiency** (section 1): the vendor prints graphs only, no table. Two printed
  anchors: CSF-20-80 at 1000 r/min, 20 °C, grease: 78 % at rated torque; SHF-20-80: 82 %. The flat
  0.70 stays unsourced until the graphs are digitised.
- **Mover's mass centre** (section 1): not published by Beckhoff in anything readable; needs the
  vendor's load diagrams, CAD or a question to their support. Mass 651 g is sourced.
- **Late fieldbus frame** (section 3): no primary text found; SINAMICS only documents the tolerated
  number of missed sign-of-life counts (p0925, fault F01912), not what the setpoint does meanwhile.
- **Stepper current convention** (section 3): the catalogue has no stepper vendor, so there is nothing
  to check it against.
