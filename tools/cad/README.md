# CAD -> machine config masses

Mass, centre of mass and inertia of CAD parts, written into a copy of a machine config as `[mass]` values with
a `# [cad] <source>: m=..., COM=(...) m, I_com=[[...]] kg m2` line above each changed key. SI units throughout.

Install (OpenCascade from PyPI, into the repo's `.venv`): `.venv/Scripts/python -m pip install cadquery-ocp`

Commands:

    python tools/cad/step_mass.py MODEL.step --map tools/cad/example-map.toml --machine t-gantry --out cad.toml --report
    sim-cli verdicts t-gantry --config cad.toml
    python -m unittest discover tools/cad          # SIM_CLI=<fresh sim-cli>, FREECADCMD=<freecadcmd> if not found

`--config IN.toml` picks the base config (default: the machine's `default.toml`); it must be complete.

Map file (`example-map.toml`): `[materials]` name = density in kg/m3 (adds to steel 7850, aluminium 2700,
stainless 7900, pom 1410, carbon 1550); `[[part]]` rules with `match` (glob on the STEP product name,
`solid_N` when unnamed), `role` and `material`. The first matching rule wins; unmatched parts go to stderr.

Roles: t-gantry carriage -> `carriage_kg`, arm -> `z_axis_kg`, payload -> `payload_kg`; single-axis carriage,
payload; delta-d3 carriage -> `platform_kg`, arm -> `upper_arm_kg`, `upper_arm_cg_m`, `upper_arm_inertia_kgm2`,
payload. Several parts with one role are combined (summed mass, joint COM, tensor by the parallel-axis theorem).

Delta arm convention: model one upper arm with the shoulder axis on the CAD x-axis through the origin.
`upper_arm_cg_m` is the distance of the arm's COM from that axis, `upper_arm_inertia_kgm2` its inertia about it.

FreeCAD: run `SimSizing.FCMacro` from this folder (Macro > Macros..., or copy all of `tools/cad`). Select parts,
tag them with role and material (stored as `SimRole`, `SimMaterial`, `SimDensity` in the document), then
"Export and check": it writes the config (source = document name) and shows `sim-cli verdicts` in a table and
the Report view. Settings are kept in FreeCAD's parameters (Macros/SimSizing).

Margin: the CSV gives no direction, so if (value <= limit) agrees with ok the check is at-most and
margin = (limit - value) / limit, else at-least and margin = (value - limit) / value; none for a zero divisor.
