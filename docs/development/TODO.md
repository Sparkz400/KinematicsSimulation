# Open after the 1.1.0 release

What is not built is in [ROADMAP.md](../../ROADMAP.md); what is estimated, chosen or left out, with its size, is in
[estimates-and-open-items.md](estimates-and-open-items.md). This file lists the work left over from the physics audit of
21 and 22 November 2026 (state of 22 November 2026), most urgent first.

## Unfinished work (partly written)

- **Stiff joint stops on every machine.** The step is cut finer where a stop is met (`HardStop::met_within`,
  `mech::STOP_SUBSTEPS`) only on the joint machines and the linear delta. The rest (single axis, T-gantry, hexapod,
  tripod, crane, cable robot, D4, parallel SCARA, the netlist machines, the scissor) still lose or make about 5e-6 J per
  stop contact at 62.5 us. Work in progress (not compiled, not tested) is on the branch
  `wip/stops-all-machines`; finish it with one shared test (drive each machine into
  a stop, energy drift under 1e-5 of the input), measure the cost of an axis resting in a stop being cut 16 times.
- **`across_the_bounds::cable_spatial::as_configured`** runs past nextest's 900 s (about 1059 s on 4 cores). Split it
  into one test per pose (`loops_keep_their_margins_at`, a pose name per config in the macro); a draft is on the branch
  `wip/cable-spatial-split` (not compiled).
- **Palletiser after its retune (base turn 1.2 / Kv 20, arm Kv 26):** run `across_the_bounds::palletiser::with_friction`
  and `without_friction` once; re-run `the_app_optimises_the_cycle_and_applies_the_settings::machine_17` (an earlier
  gain set timed out at 900 s: find out why if it still does).

## Documents

- Re-run the figures this round moved but did not re-run: the T-gantry's optimise, sensitivity, stop, accuracy and path
  blocks in both guides (still from velocity gain 0.04, Kv 50); its `optimise --worst` (about 5000 corner runs) and
  `--robust`; the two-box height profile; the D3's Morris figures; the base table's rows other than the five re-run on
  21 November. Check every figure quoted for a machine whose gains changed (T-gantry, URDF arm, arm on a track,
  palletiser, rotary knife) or whose belts became loops (XYZ, screw and C portals, five-axis, dual and rack gantries).
- The German guides: check the crash paragraph and the new effects' paragraphs against the English ones.

## Physics still open

- Two-way link bending keeps the single exact first mode (up to 15.7 % off with a tip inertia): a Ritz pair beside a
  joint lets the joint turn on the link's small inertia and makes the gearbox's switch too stiff for RK4. Needs a
  static-correction coordinate without inertia (a coordinate without a state in `chain.rs`).
- The rocking forearm (`effects.bar_rocking`): the tool pose ignores the rocking (about 30 um on the palletiser at
  rest); the planner's trains and the held modes of a multi-motor arm hold it still; its damping is not in the modes'
  damping figures.
- The cheaper treatment of a gearbox's drive/driven-back switch (semi-implicit or a stick decision): the adaptive cuts
  are exact but cost about 5 times in long holds against an obstacle (the hexapod's slab test 209 s). Only the joint
  machines and the scissor file refine; other machines' gears and the winder's own copy of the switch do not.
- An obstacle's push acts on the contact coordinates (the tool's, the crane's load, the winder's dancer), not at the
  touching part's own point (a payload under a leaning Z axis is held through the tool).
- One-way base compliance (without `effects.base_reaction`) leaves the push's work on the base in no budget row.
- The palletiser's turret ring with the rotor's Coulomb friction (it sticks for part of each swing and rings stiffer)
  is described, not modelled in the hand model.
- `effects.rotor_gyroscopic` (hexapod) ships off: it moves the default cycle's peak torques by up to 0.08 %; decide
  whether to switch it on and re-record the figures.
- The `y-z` belt plane of machine files has no shipped file and no test.
- The following error rose with the belt loops (C portal 177 to 253 um, screw portal 100 to 152 um); no verdict judges
  following error.
- The rest of section 4 of the open-items list (cable sag model's catenary error, the stepper's convention, ...).

## Data and verification (no code can close these)

- Section 1 of the open-items list: measured traces of the project's own machines, unpublished motor data.
- Section 7: the catalogue rows never checked against a vendor PDF (Apex, Beckhoff, B&R, Gates, HIWIN, Kollmorgen,
  Lenze, Megadyne, Mitsubishi, Neugart, SEW, Wittenstein, Yaskawa; Festo EGC / ELGA; Harmonic Drive CSF / CSG; Rexroth
  MS2N / ctrlX): fetch the PDFs and check row by row.
- Install the MSI and the .deb once each on a clean machine; watch the release and publish workflows of 1.1.0.

## Carried over from 1.0.0, not rechecked

- Orientation rates on a turning slerp in the planner come from difference quotients (`Geo::at`).
- The FMU state's format is still "state 3" although the snapshot layout changed since 14 November.
- Check the D3/D4 arm-angle cancellation and the app's budget-row counts (`line_module` row with any DC link).
