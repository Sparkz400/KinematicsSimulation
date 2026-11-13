"""FreeCAD side of the sizing check, without any GUI: tag objects, collect them as bodies, write the config,
run sim-cli. SimSizing.FCMacro is the dialog on top; test_freecad_macro.py runs this under freecadcmd."""

import tempfile
from pathlib import Path

import FreeCAD as App

import cadmass

GROUP = "Sim sizing"
PARAMS = "User parameter:BaseApp/Preferences/Macros/SimSizing"


def tag(objs, role, material="", density=0.0):
    """Role "" untags. density in kg/m3 overrides the material when > 0."""
    for o in objs:
        for prop, kind, doc in (("SimRole", "App::PropertyString", "carriage, arm or payload (empty: not counted)"),
                                ("SimMaterial", "App::PropertyString", "material name from the list in cadmass.py"),
                                ("SimDensity", "App::PropertyFloat", "kg/m3; overrides SimMaterial when > 0")):
            if prop not in o.PropertiesList:
                o.addProperty(kind, prop, GROUP, doc)
        o.SimRole, o.SimMaterial, o.SimDensity = role, material, float(density)


def density_of(o):
    if o.SimDensity > 0:
        return o.SimDensity
    if o.SimMaterial not in cadmass.MATERIALS:
        raise ValueError(f"{o.Label}: unknown material {o.SimMaterial!r} and no SimDensity; known: {', '.join(cadmass.MATERIALS)}")
    return cadmass.MATERIALS[o.SimMaterial]


def bodies(doc):
    """One Body per tagged object with solids, in SI, placed where the whole document has it."""
    out = []
    for o in doc.Objects:
        if not getattr(o, "SimRole", "") or not hasattr(o, "Shape"):
            continue
        shape = o.Shape.copy()
        # Inside an App::Part the shape is in the container's coordinates.
        shape.Placement = o.getGlobalPlacement()
        if not shape.Solids:
            raise ValueError(f"{o.Label}: tagged {o.SimRole} but has no solid")
        rho = density_of(o)
        parts = []
        for s in shape.Solids:
            c, t = s.CenterOfMass, s.MatrixOfInertia
            # FreeCAD gives mm, mm3 and mm5 for unit density.
            tensor = [[rho * 1e-15 * t.A[4 * i + j] for j in range(3)] for i in range(3)]
            parts.append(cadmass.Body(o.Label, o.SimRole, rho * 1e-9 * s.Volume, (c.x * 1e-3, c.y * 1e-3, c.z * 1e-3), tensor))
        out.append(cadmass.combine(parts, name=o.Label, role=o.SimRole))
    return out


def settings():
    p = App.ParamGet(PARAMS)
    return {"machine": p.GetString("Machine", "t-gantry"), "sim_cli": p.GetString("SimCli", ""),
            "base_config": p.GetString("BaseConfig", ""), "out_file": p.GetString("OutFile", "")}


def save_settings(s):
    p = App.ParamGet(PARAMS)
    p.SetString("Machine", s["machine"])
    p.SetString("SimCli", s["sim_cli"])
    p.SetString("BaseConfig", s["base_config"])
    p.SetString("OutFile", s["out_file"])


def out_path(doc, s):
    if s["out_file"]:
        return Path(s["out_file"])
    folder = Path(doc.FileName).parent if doc.FileName else Path(tempfile.gettempdir())
    return folder / f"{doc.Name}-{s['machine']}.toml"


def export(doc, s):
    """Writes the config copy and checks it: (bodies per role, config path, verdict rows)."""
    found = bodies(doc)
    if not found:
        raise ValueError("no object is tagged with a role")
    roles = cadmass.by_role(found)
    base = Path(s["base_config"] or cadmass.default_config(s["machine"]))
    with open(base, encoding="utf-8", newline="") as f:
        text = cadmass.apply(f.read(), s["machine"], roles, doc.Name)
    out = out_path(doc, s)
    with open(out, "w", encoding="utf-8", newline="") as f:
        f.write(text)
    sim = cadmass.find_sim_cli(s["sim_cli"] or None)
    if not sim:
        raise RuntimeError("no sim-cli: set its path in the settings or the SIM_CLI environment variable")
    return roles, out, cadmass.parse_verdicts(cadmass.run_verdicts(sim, s["machine"], out))


def report_lines(machine, roles, out, rows):
    lines = [f"SimSizing: {machine} -> {out}"]
    for key, v in cadmass.values(machine, roles).items():
        lines.append(f"  mass.{key} = {v:.6g}")
    for r in rows:
        margin = "" if r["margin"] is None else f"{100 * r['margin']:+.1f} %"
        lines.append(f"  {r['result']:<4} {r['check']:<28} {r['motor']:<3} {r['value']:>12.4g} {r['limit']:>12.4g} {margin:>9}")
    failed = sum(not r["ok"] for r in rows)
    lines.append(f"  {len(rows)} checks, {failed} FAIL")
    return lines

