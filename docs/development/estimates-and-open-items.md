# What is estimated, chosen, left out or unverified

State: 17 November 2026, after the physics audit (about 95 findings), its fixes, and the stage that
modelled the effects the code had only named in `shortcut:` comments (133 comments, 24 left). This file
lists everything that is **not taken from reality or not derived**: values without a source, rules
somebody chose, what the models still leave out and how large that is, and what no run has checked.
It replaces the list of 16 November.

Words used: **estimate** (a value with no source, chosen to be plausible), **chosen** (a rule that is
defensible but not derived from physics or a standard), **left out** (the model knowingly omits it;
the size is given where it was measured or worked out by hand), **unverified** (written, not run or
not checked against the source).

Vendor facts with document and page are in [vendor-facts.md](vendor-facts.md).

## 1. What code cannot close

These need data that no vendor publishes, or a measurement on hardware.

| What | Where | Why it stays |
| --- | --- | --- |
| No measured trace of one of this project's own machines | `docs/validation.md` | The only measured data is the external ISW ball-screw set under `datasets/`. Everything else in "validation" compares the program with a hand calculation, an independent script or itself. |
| S3 duty line in the torque-speed chart | `charts.rs` | No catalogue the simulator reads prints S3 data. |
| Standstill values of Yaskawa and Mitsubishi motors, rated torque of SEW CMP | `catalog.rs` | The makers print only one of the two; the other stands in (overstates what the motor carries at speed by a few per cent). |
| Winding resistance of motors whose maker prints none; base-load current of drives | `catalog.rs` | Not published; an estimate from the braking table, and the S210's share of the rated current, stand in. |
| Winding resistance and inductance of the Siemens 1FK2 / 1FT2 in the shipped configs (10 mH) | shipped configs | Siemens does not print them in catalogue D 32; naming the motor gives another estimate (28.9 mH for the 1FK2105-4AF, from the braking table). Closing it: the per-motor data sheet from Siemens' data-sheet service. |
| The split of a motor's iron loss into eddy and hysteresis parts (one exponent 1.5 for both) | `drive.rs` | No maker prints iron loss over speed. |
| The magnets' temperature (eddy heating of the rotor) | `drive.rs` | No maker gives the rotor's loss. |
| The shape of a gear's hysteresis loop at a reversal, its presliding stiffness | `mech.rs` | Not published; both default to the gear's stiffness K1 (keys `gearbox.hysteresis_stiffness_factor`, `gearbox.mesh_stiffness_factor`). |
| Motor-specific adapter plates and shaft lengths for a named axis | `catalog/mechanics.rs` | Needs the makers' configurators. |
| Bearing life factor below κ = 0.1 | `analysis.rs` | ISO 281 gives no factor there; 0.1 is taken. |
| The Beckhoff XTS mover's centre of mass | `machine-mover-track` (`mass.centre_offset_m` = 0) | Not published; needs the vendor's CAD or load diagrams. Mass 651 g is sourced. |
| What a drive does with its setpoint on a late fieldbus frame | `control.rs` | No primary text found (SINAMICS documents only the tolerated number of missed sign-of-life counts). The model extrapolates the old frame. |
| Fanuc's feed rule for a block with rotary axes; a stepper's current convention | `program.rs`, `stepper.rs` | No primary source found; the catalogue has no stepper vendor. |
| The 1FT7085-7WF7's data sheet (the measured dataset's motor) | `datasets/isw-ball-screw-axis` | Siemens' service did not answer on 17 November; the values rest on the repository's earlier fetch and an independent data sheet copy of the same motor size. |
| RAPID's `v_ori` in speeddata and KRL's `$VEL.ORI1` | `program.rs` | Not read; the config key `program.orientation_speed_rad_s` (500 °/s, ABB's default) is used for every move. |

## 2. Estimates in the shipped files

Every shipped machine is a generic machine: its proportions are made up unless a part is named from the
catalogue. The configs mark each such value with `# [estimate]` (about 2,400 marks). The ones that
matter most, and the ones added in November 2026:

| What | Value | From |
| --- | --- | --- |
| Placeholder motor in the single axis, the T-gantry default and conveyor configs and 12 machine files | 0.45 N m/A | No maker's motor; marked in the config comment. |
| Spatial cable robot's settling tolerance for turns | 2 mrad (0.28 mm at the platform's corners) | A requirement, not physics. The backlash floor is about 0.25 to 0.6 mrad; the default rule's 0.2 mrad cannot be held with this gearbox. |
| Cable mass per metre | 3 g/m (planar), 10 g/m (spatial) | Rope diameters 2 mm and 4 mm. |
| Hexapod and tripod leg parts (mass, centre, inertia) | rod 0.35 m bar, body 0.5 m bar, m l²/12 | Bar model of the actuator and tube sizes in the config. |
| H4 upper-arm tube | carbon 60 × 2.5 mm, 100 GPa; bending damping 2 % of critical; elbow twist inertia 2.9e-4 kg m² | The D3's tube scaled; the H4 config had no tube. |
| D4 tool gearbox output inertia | 1e-4 kg m² | Steel disc 50 × 15 mm (7.2e-5) plus the carrier's share. |
| Five-bar arm tube | 1.2 kg, centre 0.2 m | The uniform tube with the config's inertia. |
| Five-bar tool turn's velocity gain | 0.0443 N m s/rad | Chosen after the turn became elastic: 0.08 hammers across the play, 0.06 does not. |
| Linear delta belt | 1500 J/(kg K), 15 W/(m² K), 60 K allowed rise | Polyurethane with steel cords; a moving belt in still air. |
| Linear delta rod-joint offsets from the slide | 0 | Festo's moment ratings are not shipped; the guide moments are therefore not judged. |
| T-gantry beam | 3.5 kg/m, damping 1 % | A light 80 × 80 mm aluminium profile, a bolted structure. |
| Cam indexer's cam radius | 0.10 m | Gives a pressure angle of 35° at the fastest point (peak contact force +2.1 %). |
| Disc cam follower: roller width, contact damping, guide friction | 14 mm, 0.5, 5 N | A KR 30 track roller. |
| Crane: hook block, sling, gauge, wheelbase, wheel lateral stiffness, skew damping, floor friction, floor height | 15 kg, 1 m, 5.6 m, 2 m, 2e6 N/m, 5 %, 0.3, 0.5 m | A crane of this load class. Rope mass 0.126 kg/m is sourced (EN 12385-4 tables: 0.125 to 0.129 at 6 mm). |
| Gantries' yaw inertia including the belt's share | 2.657 and 1.339 kg m² | The belt loop taken symmetric about the bridge centre; the files' span lengths are placeholders. |
| Rotary hexapod's clearance of rods to platform and base | 0.02 m | Rod radius plus the plate's half thickness. |
| Joint kit: the worm rotor's axis on the rotary table | along x | No layout given. |
| Pre-existing: tooth-jump factor, brake deflection and slip times, the efficiency fade's 0.5 W | `mech.rs`, configs | Never judged against a source; measure or mark. |

## 3. Rules somebody chose

| Rule | Where | What it costs or assumes |
| --- | --- | --- |
| Retuned velocity gain keeps the config's crossover over the rotor-side inertia; a linear motor by the whole moving mass | `catalog.rs` (`bounded_gain`, `retuned`) | Fitted on a sweep of 12 cases; knows inertias only. Motors with an inertia ratio above about 25 (EXCT-30, 360 candidates swept) get no stable gain and are rejected by the verdicts. The integral time and position gain did not follow the gain until the fix of 17 November (see section 8). |
| Iron loss: the shaft's drag is the printed gap k_t·I_N − M_N less the motor's friction; the winding's heat is the thermal split R(I0² − I_N²); both scale with (n/n_N)^1.5 | `drive.rs` | Backed by the printed efficiencies of 144 1FK7/1FT7 rows (the gap is a median 2.9 times the thermal split). The friction subtracted is the config's estimate. |
| The field-weakening d current uses the unsaturated back-EMF | `drive.rs` | With the saturated one braking beyond the curve ran away; costs 3.6 % torque at 380 rad/s on the test motor. |
| Overmodulation carries the fundamental only (2√3/π three-phase, 4/π stepper bridges) | `drive.rs` | No harmonic loss. |
| S1 curve of a Siemens S210 motor: the vendor's graph, scaled to the printed table when within 4 %; beyond its last point the last torque | `catalog/siemens/s1_curves_s210.csv`, `drive.rs` | 74 curves as read, 14 scaled by 0.985 to 1.036, 8 motors keep the straight line. |
| A duty mix runs each duty in the order given, repeated; S1 and the thermal figure go through the motor's one node | `search.rs` | In the middle regime the order matters. Without `ratings.drive_overload_time_constant_s` the drive current of a mix is the time-weighted RMS. |
| Planner shares: 0.3 of the jerk for the binormal; ½ of the acceleration limit for a slerp's bend; 0.8 torque and tension reserve; 1 % of the speed kept back at least (`speed_kept_of`); 32 grid steps for the tops; interpolation allowance 0.0851·J·h (C2) or 2/3·J·h (Hermite) | `path.rs` | A torsioned corner runs 9 % slower than a planar one; the tension reserve costs 1.9 % (planar) and 5.6 % (spatial) of the cable robots' time-optimal cycle; a fitting range narrower than 1/32 of the span can be missed. |
| Friction is not counted as helping while braking | `path.rs` | Conservative. |
| Fit statistics: `PER_AXIS_SPREAD` 0.05, `REPEATABILITY` 1e-3, the larger of two error estimates, a non-central window only when 10 times smoother | `identify.rs`, `trace.rs` | The row-wise error is conservative (about 8 times the seed scatter on the elastic arm). |
| Sweep fit: the probe's exponent, at least half its step; S1 curve tolerance 2 % or half the last printed digit | `identify.rs`, `catalog/tools` | — |
| Self-locking worm: Coulomb friction on both flanks; in the rigid model the overhauling flank takes the statically held torque (a Painlevé regularisation); stick below 1e-3 rad/s over 1 ms | `sim-netlist/src/worm.rs` | Rigid and elastic models agree at steady speed and differ in a transient: the elastic worm jams within milliseconds when a lowering load is left to stop. |
| Mesh friction of a gear with no back efficiency of its own gives 1/(2 − η) driven back, the loss factor η | `mech.rs` | About (1 − η)², 1 % at 0.9. |
| Behind a converter Dahl's law acts on the combined twist | `transmission.rs` | Only the small-amplitude stiffness and the sliding level are exact. |
| The linear delta's guide friction scales its bristles' stiffness and damping with the load, as its levels | `machine-delta-linear` | Needed for the friction and the acceleration to have a fixed point. |
| Guide load law `preload + |tilt|` | gantries | Has a corner at zero acceleration; preloaded carriages stay flat until the preload is exceeded. |
| Joints that turn on (crank, knife, cam disc, indexer driver) have no end stops unless the config gives them | `machine-crank-cam` | A limit switch that trips the drive is not modelled. |
| Cam follower: Palmgren's line contact, a constant Coulomb guide friction | `machine-crank-cam/src/disc.rs` | The contact ignores the curvatures (logarithmic). |
| Crane: the free rope's speed varies linearly along it; the shaper uses the double pendulum's lower root | `machine-overhead-crane` | The rope's mass in the shaper's frequency is ignored (about 0.05 %, an estimate). |
| Cable robot: the winch's spin axis horizontal and square to the winch direction | `machine-cable-robot` | No layout given. |
| A strain wave gear's rotor turns against its output; a rack's pinion rides above the fixed rack; an idler's positive speed follows its section's first pulley | joint kit, `sim-netlist` | Conventions for the frame's reaction. |
| The cylinder check refuses every rotary axis that carries the cylinder | `pneumatic.rs` | Conservative. |
| Modes shown at 5 evenly spaced poses of a run; a refused start push keeps the last start | `sim-app` | The tool is then up to one sag off, on the envelope only. |
| A machine-file table's reach box | `sim-netlist` | Conservative: a pose outside it may be reachable. |
| The DC-link account switches to 257 passes after 131,072 stretches | `analysis.rs` | Error at most 1/512 of the link's room. |
| The torque-band monitor expects the model's own friction state | `control.rs` | An idealised observer; a real controller would drive the same model from measured speeds. |
| OPC UA tests read the end of a path 1 s after it; timing tests assert the pacing's books, not wall time | `sim-opcua/tests` | — |

## 4. What the models still leave out

Sizes are measured by a run where it says so, otherwise worked out by hand.

| Left out | Machine | Size |
| --- | --- | --- |
| The cables' own inertia; the lift of cable from the drum to the pulley; the sag's weight changing with the pose | cable robots | 0.5 % of the spatial platform's mass; 0.05 N; 5e-5 N. The sag model is a shallow parabola, 8.7e-5 to 1.7e-4 N from the exact catenary (measured). |
| The rotors' gyroscopic coupling with the legs' swing (it is in the frame's reaction only); the rods' faces against platform and base | hexapod, tripod, rotary hexapod | About 0.015 N m (hand estimate). |
| A slack sling: with the hook block and the floor contact on, the sling is a rod | crane | Not measured; the sling pushes where it should go slack. Only the hook block was run on a full cycle; rope mass, elastic drives and bridge skew passed their own tests and a 0.2 s run. |
| Link bending together with parallelogram bars of their own (refused); the palletiser's rod and crank (shipped bar arrays empty) | joint kit | The bend-mode method overestimates by 1.2 to 2.6 % against the exact beam (measured in the test case). |
| The belt runs' sideways offset on the leaning Z axis; the belt runs riding on the beam's vertical mode | T-gantry | A first-order moment (T₁ − T₃)·0.04 m; the beam mode would fall 0.9 % (59.32 to 58.77 Hz). |
| Angular momentum of the T-gantry crate against its machine file is not compared | T-gantry | Different origins and belt-run geometry. |
| Idler signs of the y belts of the dual gantry and the rack gantry | machine files | Not checked: the 3D drawing does not model those routes faithfully. |
| One screw stiffness for the whole stroke, at the far end | wafer handler, screws | 321 Hz at the far end, 386 Hz with the nut 0.2 m from the bearing, 438 Hz at 0.1 m. |
| Belt-to-pulley heat conduction | linear delta | Not sized. |
| The tube's single-mode shape for inertia with the static stiffness 3EI/L | D3, D4, H4 arm bending | Not sized. |
| A contact that is not at the tool is pushed at the motors (machine crates, linkage, delta, scissor files) | obstacle contact | A load on a soft belt swings on by the belt's stretch (millimetres at 1 m/s). Contact needs a load above c·h/2.79 (about 56 g) to stay stable. |
| The stepper's detent while disabled | stepper | Up to T_d/(4N) appears or disappears from the budget at an enable. |
| A loop that crosses a backlash, in the margin measurement | `sim-cli margins` | It measures the motor alone inside its play and called a hammering loop stable (8.1 dB): such a loop needs a time run. |
| `mode_shape` of the five-bar | five-bar | Display only. |
| Run files recorded before the signal `setpoint_age` | `charts.rs` | The tool error falls back to the held setpoint: up to one cycle's travel (0.79 against 0.18 mm on the T-gantry). |

## 5. The 24 `shortcut:` comments still in the code

`git grep "shortcut:"` lists them. None is an effect that could be modelled with data at hand.

- **Need data (section 1):** standstill values, winding resistance, base-load current, adapter plates
  (`catalog.rs`, `catalog/mechanics.rs`); the S3 line (`charts.rs`); the rotor's temperature
  (`drive.rs`); the bearing factor below κ 0.1 (`analysis.rs`).
- **Limits of the fixed-step solver:** bristles faster than the step (`mech.rs`); the last 0.5 % of
  the pneumatic flow law (`pneumatic.rs`); the speed curve's sweeps to 1e-7 (`path.rs`); a chain's
  drag on an axis without a guide below 1 cm/s (`sim-netlist`).
- **Conservative or without effect on a figure:** the thermal image after it tripped (the verdict
  fails anyway); an unstable open loop's crossing (`analysis.rs`); the swing joints' unit inertia
  without a payload (`machine-joint-kit`); the shared cycle times and DC link of per-motor parts
  (`catalog.rs`).
- **App and tooling:** the CAD mesh's size after an edit, three display heuristics (`sim-app`); bold
  text widths in the report; the PDF reader's limits; a hand-kept summary and the GIL in the Python
  package; loss-free configs for machine files in the test suite.

## 6. New effects and their switches

Effects that add states are behind a switch, off in the shipped configs. "Fit" means the reviewer
verified the model by hand values and runs and found it ready to switch on.

| Switch | Machine | State |
| --- | --- | --- |
| `effects.tool_gearbox_body` | D4 | Fit (tool mode 41.07 Hz, a second overdamped motion). |
| `effects.arm_bending`, `effects.arm_tube_inertia` | H4 (and the corrected tube model on the D3) | Fit; gearbox modes about 945 Hz with the tube on. |
| `effects.table_twist` | cam and Geneva indexers | Fit. |
| `effects.follower_lift_off` | disc cam | Fit; adds 5 N of guide friction that the model without it does not have. |
| `effects.beam_vertical_mode` | T-gantry, its machine file | Fit; the beam's mass and stiffness are estimates. |
| `effects.rope_mass`, `effects.hook_block`, `effects.joint_elasticity`, `effects.bridge_skew`, `effects.floor_contact` | crane | Verified by their own tests; see section 4 for what was not run. |
| `drive.overmodulation` | every drive | Off; verified. |

Always on since November 2026 (they changed shipped figures): the legs' bodies on the hexapod and
tripod; cable weight and sag; the elastic Z screw and tool turn of the five-bar and the wafer handler's
lift; the linear delta's friction load with the acceleration; the pressure angle in the cam indexer; the
indexer's ring on the table's own inertia; the rectifier's loss in the supply energy (about +2 % energy
per cycle with a DC link); the shaft drag from the iron loss on three machines; printed S1 curves for
named S210 motors; back-drive efficiency of screws and of the table's worm; gravity in the guides'
commanded acceleration.

## 7. The catalogue against the vendors' documents

- **Checked row by row against the PDF, none wrong:** Bahr 23 rows, Festo DSBC 7, Harmonic Drive SHG
  44, Rexroth CKR 11, Siemens S210 8 (shaft height 40). The Rexroth extraction script reproduces the
  CSV from the vendor's PDF.
- **Never checked against a PDF (none on this machine):** Apex, Beckhoff, B&R, Gates, HIWIN,
  Kollmorgen, Lenze, Megadyne, Mitsubishi, Neugart, SEW, Wittenstein, Yaskawa; Festo EGC / ELGA;
  Harmonic Drive CSF / CSG; Rexroth MS2N / ctrlX.
- **Known disagreement inside Siemens' documents:** catalogue D 21.4 prints 5 pole pairs for all
  sixteen 1FT7 High Dynamic motors; the per-motor data sheets say 8 poles (6 for the smaller sizes).
  The catalogue code takes the data sheet's.
- **Harmonic Drive efficiency:** read off the vendor's graphs (0.699 to 0.800), about 1 point of
  reading error.
- **43 shipped motor blocks** are tested against their catalogue rows.

## 8. Verification state

- The audit's fixes: one full run of the suite on 17 November (2,265 tests; 2,255 passed at once, the
  other ten fixed or passing alone), plus format, clippy, doc tests, the web view's and the Python
  package's tests and the catalogue checks.
- The modelled effects: written by implementers who ran nothing, then reviewed item by item by
  reviewers who redid the derivations and ran each group's tests. The reviews found and corrected,
  among others: the iron loss's definition, the cable's sag law, the belt runs' geometric stiffness,
  the rope's kinetic energy, the parallelogram bars, the duty mix's thermal model, a transposed matrix
  in the machine files' lowest mode (existing code: the five-axis file's resonance was 71.3 Hz, is
  43.6 Hz), the D3's tube inertia (existing code: gearbox modes about 40 % low), a config helper that
  silently dropped all but its first line.
- **The final full run after all of this: see the end of this file.**

## 9. Documents that do not match the program yet

- `docs/user-guide.md`, `docs/de/benutzerhandbuch.md`: worked command outputs (many were stale before
  November 2026, and the figures moved again).
- `docs/config-reference.md`, `docs/validation.md`, `docs/model-reference.md`: figures quoted from
  runs (corner times, the delta line, the optimiser, T-gantry worst-corner figures, mode tables).
- `docs/fmi.md`: FMU output counts and validation logs (signals were added).
- `docs/de/*`: statements changed in the English docs.

These are regenerated after the final run.
