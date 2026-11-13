# Integrations: CAD, FreeCAD, Onshape and the sizing report

The simulator sizes a machine from a config file (see [config-reference.md](config-reference.md)). This page is
about getting numbers into that file from CAD, and getting the verdicts out as a document a customer signs.

| Piece | Where | Status |
|---|---|---|
| STEP → mass, centre of mass, inertia → config | `tools/cad/step_mass.py` | working, tested |
| FreeCAD macro: tag parts, export, run the sizing check | `tools/cad/SimSizing.FCMacro` | working, tested headless and once in the GUI |
| Onshape mass properties → config | `tools/cad/onshape_mass.py` | prototype: offline path tested, API path untested |
| Sizing report, PDF and HTML | `sim-cli report`, app button "Sizing report…" | working, tested |

## Where a value comes from

Every value in a config can say where it comes from, in a comment right above it:

- `# [catalogue] …` read or derived from a maker's document;
- `# [estimate] …` not published, our estimate (with a low and a high value in `[bounds]`);
- `# [cad] <source>: m=…, COM=(…) m, I_com=[[…]] kg m2` computed from a CAD model by the tools below.

A tag marks the keys right below it, up to a blank line or the next comment, so a group of values shares one tag;
a `[cad]` line, which the CAD tools write per key, marks only the key below it. A tag in a comment at the end of a
key's line marks that key. A tag right above or after a table header (`[mass]`) marks every key of the table that
has no tag of its own. Where a comment block has tags on several lines, the line nearest the key counts; a line that
names two tags ("[catalogue] … its efficiency [estimate]") gives both, and its words say which value is which. A
value in an inline table (`mass = { … }`) gets no tag. The sizing report lists every input with its source.

## Roles: which part goes where

Parts are tagged with a role. Several parts with one role are combined: masses added, a common centre of mass,
the tensors moved to it with the parallel-axis theorem.

| Machine | `carriage` | `arm` | `payload` |
|---|---|---|---|
| `t-gantry` | `mass.carriage_kg` | `mass.z_axis_kg` | `mass.payload_kg` |
| `single-axis` | `mass.carriage_kg` | — | `mass.payload_kg` |
| `delta-d3` | `mass.platform_kg` | `mass.upper_arm_kg`, `upper_arm_cg_m`, `upper_arm_inertia_kgm2` | `mass.payload_kg` |

Delta arm: model one upper arm with its shoulder axis on the CAD x-axis through the origin. `upper_arm_cg_m` is
the distance of the arm's centre of mass from that axis, `upper_arm_inertia_kgm2` its inertia about it.

Only `[mass]` values are written. The full tensor and the centre of mass are in the `[cad]` comment; the models
use no more of them today (for example the T-gantry's `geometry.z_com_drop_m` is not set from CAD). Two things the
tools leave to you: `[bounds]` of a value now from CAD stay as they were, so `--bands` still varies it over the old
estimate's range (narrow or remove them); and the delta's `upper_arm_*` values are the arm without its elbow block,
which is `mass.elbow_kg`: tag the elbow block as nothing, or it counts twice. `delta-d3` is offered as soon as the
simulator registers that machine (`sim-cli list`).

## (a) STEP files

OpenCascade comes from PyPI (`cadquery-ocp`, which has wheels for Python 3.10 to 3.14):

```bash
.venv/Scripts/python -m pip install cadquery-ocp
```

```bash
python tools/cad/step_mass.py MODEL.step --map tools/cad/example-map.toml --machine t-gantry --out cad.toml --report
```

```bash
sim-cli verdicts t-gantry --config cad.toml
```

The map file gives densities (`[materials]`, kg/m³; steel, aluminium, stainless, POM and carbon are built in) and
rules `[[part]]` with `match` (a glob on the STEP product name), `role` and `material`; the first matching rule
wins, unmatched parts are listed on the error stream and not counted. The file's length unit is converted to metres
by the reader. `--config` picks the config to start from (default: the machine's `default.toml`); it must be complete,
and only the mapped `[mass]` lines and their `[cad]` comments change, every other line stays byte for byte.

Tested against hand-computed solids: a box, a solid cylinder, a hollow tube, a part placed off the
origin, a file in metres, and a compound against OpenCascade's own result (`tools/cad/test_cadmass.py`).

## (b) FreeCAD

FreeCAD 1.x: run `tools/cad/SimSizing.FCMacro` (Macro → Macros…, with the macro folder pointing at `tools/cad`, or
copy the whole folder). In its dialog:

1. Select parts in the tree and tag them with a role and a material, or a density. The tags are properties of the
   object (`SimRole`, `SimMaterial`, `SimDensity`), shown in the property editor and saved in the `.FCStd`.
2. Settings: machine, the `sim-cli` program, the base config and the output file; kept in FreeCAD's parameters
   (`Macros/SimSizing`).
3. "Export and check" adds up the tagged parts per role (shapes in mm, converted to SI, each in its global
   placement), writes the config with `[cad]` lines (source: the document's name), runs `sim-cli verdicts` on it and
   shows each check with value, limit, margin and result, failed ones in red, in a table and in the Report view.

The logic is in `tools/cad/simsizing.py` and runs without the GUI; `tools/cad/test_freecad_macro.py` runs it under
`freecadcmd` on a tagged box and cylinder and compares masses and inertias with hand values.

Margin in the macro: `sim-cli verdicts` gives no direction, so where "value ≤ limit" agrees with the result the check
is an at-most one and the margin is (limit − value) / limit; otherwise it is an at-least one (lives, belt slack) and
the margin is (value − limit) / value, as in the sizing report.

## (c) Onshape: design note and prototype

**Goal.** The same as for FreeCAD: masses and inertias of tagged parts into a config, and the verdicts back.

**Options.**

- *REST app (chosen).* Onshape's endpoint `GET /api/v9/partstudios/d/{did}/{w|v|m}/{id}/e/{eid}/massproperties`
  returns mass, volume, centroid and inertia of a Part Studio, computed by Onshape with the materials assigned there.
  A small client reads it and writes the config with `tools/cad/cadmass.py`, as the other two tools do. Runs anywhere
  Python runs, needs no Onshape-side code.
- *FeatureScript feature.* Runs inside Onshape and could store a role as an attribute on parts, but FeatureScript
  cannot call the simulator, and whether it can evaluate an inertia tensor was not checked here. Useful later at
  least for tagging.
- *Onshape app with OAuth in the browser*, showing verdicts inside Onshape: needs a server and an app registration;
  the step after the REST client works.

**Roles.** One Part Studio per role (or, later, a part filter): the `-all-` entry of its mass properties is that role.

**What is known.** From Onshape's documentation (Part Studios guide, API keys guide): the response has
`bodies["-all-"]` with `mass` (3 numbers), `centroid` (9), `inertia` (27), `hasMass` and `massMissingCount`; the
numbers fit SI units (the sample's mass over volume is 7850 kg/m³), and each quantity appears to be its value
followed by a lower and an upper bound. Requests are signed with an API key: HMAC-SHA256 over method, nonce, date,
content type, path and query, each followed by a newline, lowercased, base64.

**What is not confirmed**, because it needs an account and keys this project does not have: whether the tensor is
about the centroid or about the origin (`--inertia-about centroid|origin`, default centroid), the bound order, and
whether a signed request built this way is accepted. A part without a material makes `massMissingCount` non-zero; the
prototype refuses it rather than leaving the part out.

**Prototype.** `tools/cad/onshape_mass.py`:

```bash
python tools/cad/onshape_mass.py --machine t-gantry --role carriage=carriage.json --role payload=payload.json --out cad.toml
```

A source is a saved massproperties response (save it from Onshape's API Explorer while signed in: works without
keys) or a Part Studio URL, fetched with `ONSHAPE_ACCESS_KEY` and `ONSHAPE_SECRET_KEY` (not tested).
`tools/cad/test_onshape.py` tests the conversion on a response shaped like the documented sample, both tensor
references, the refusal of parts without material, and the signature against the documented scheme.

**To finish it, with keys:** model a 100 × 200 × 300 mm steel box away from the origin, fetch it, and compare with
the hand values; that settles the tensor's reference point and the signature. Then a part filter so that one Part
Studio can hold all roles.

## (d) The sizing report

```bash
sim-cli report t-gantry --config crates/machine-t-gantry/exct-30.toml --bands --out report.pdf
```

`--out` ending in `.pdf` writes a PDF (A4), anything else self-contained HTML (plots as inline SVG; print it from a
browser if needed). `--lang de` writes it in German, `--bands` adds the error bands, and every option of the other
commands (`--motor`, `--set`, …) works as there. The result (all checks pass or how many fail) goes to the error
stream; the exit code is 0 either way. In the app: "Sizing report…" beside the config file buttons writes the report
of the config as it is now, with the error bands when they are ticked in the analysis panel.

Contents:

1. **Machine**: name, config file, date, simulator version, cycle time and the overall result; the parts named in
   `[parts]` and the `[mass]` and `[geometry]` values, each with its source.
2. **Cycle**: motor torque and speed over the measured pass, and the operating points against the drive's
   torque-speed curve.
3. **Verdicts**: every check with value, limit, margin and result; failed rows in red. Margin is one minus the check's
   utilisation (value over limit; for lives and belt slack limit over value), so negative means exceeded. The values
   are the ones `sim-cli verdicts` prints.
4. **Error bands** (with `--bands`): per check the lowest and highest value with each bounded parameter at its
   bounds, and whether the worst case passes.
5. **Inputs and their sources**: every value the run used with `[catalogue]`, `[estimate]` or `[cad]` from the
   comments, "part in [parts]" where a part named from the catalogue brought it, "set for this run" where it differs
   from the file (`--set`, a part name on the command line, an edit in the app), "not stated" where the file says
   nothing.
6. **Sign-off**: prepared by, checked by and accepted by the customer, each with name, company, date and signature.

The PDF is written by the simulator itself (`sim_components::report`), with the standard Helvetica fonts and no
outside library; text outside Latin-1 shows as "?".

## Tests

```bash
cargo test -p sim-components report
```

```bash
cargo test -p sim-cli --test cli report
```

```bash
SIM_CLI=target/release/sim-cli.exe .venv/Scripts/python -m unittest discover tools/cad
```

The Python tests skip what they cannot run (no OpenCascade, no FreeCAD, no `sim-cli` that reads the current configs)
and say why. `FREECADCMD` names `freecadcmd` where it is not found by itself.
