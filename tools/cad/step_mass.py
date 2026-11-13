"""STEP model -> mass properties per part and role -> a machine config with [cad] values.

    python tools/cad/step_mass.py MODEL.step --map MAP.toml --machine t-gantry [--config IN.toml] --out OUT.toml [--report]

Needs OpenCascade from PyPI: pip install cadquery-ocp
"""

import argparse
import fnmatch
import sys
import tomllib
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import cadmass  # noqa: E402

DEFAULT_MATERIALS = cadmass.MATERIALS


def read_parts(path):
    """[(name, shape)] of the STEP file's leaf parts, placed, in metres."""
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.STEPCAFControl import STEPCAFControl_Reader
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDataStd import TDataStd_Name
    from OCP.TDF import TDF_Label
    from OCP.collections import Sequence_TDF_Label as TDF_LabelSequence
    from OCP.TDocStd import TDocStd_Document
    from OCP.TopLoc import TopLoc_Location
    from OCP.UnitsMethods import UnitsMethods_LengthUnit_Meter
    from OCP.XCAFDoc import XCAFDoc_DocumentTool

    doc = TDocStd_Document(TCollection_ExtendedString("XmlOcaf"))
    # The reader converts the file's units to the document's: make that metres.
    XCAFDoc_DocumentTool.SetLengthUnit_s(doc, 1.0, UnitsMethods_LengthUnit_Meter)
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    if reader.ReadFile(str(path)) != IFSelect_RetDone or not reader.Transfer(doc):
        raise RuntimeError(f"cannot read STEP file {path}")
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())

    def name(label):
        attr = TDataStd_Name()
        return attr.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(), attr) else ""

    parts = []

    def walk(label, loc):
        if tool.IsAssembly_s(label):
            comps = TDF_LabelSequence()
            tool.GetComponents_s(label, comps)
            for i in range(1, comps.Length() + 1):
                comp = comps.Value(i)
                ref = TDF_Label()
                tool.GetReferredShape_s(comp, ref)
                walk(ref, loc.Multiplied(tool.GetLocation_s(comp)))
        else:
            parts.append((name(label), tool.GetShape_s(label).Moved(loc)))

    free = TDF_LabelSequence()
    tool.GetFreeShapes(free)
    for i in range(1, free.Length() + 1):
        walk(free.Value(i), TopLoc_Location())
    return parts


def unit_props(shape):
    """(volume m3, COM m, inertia tensor about the COM for density 1) of a shape in metres."""
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps

    g = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, g)
    c, t = g.CentreOfMass(), g.MatrixOfInertia()
    return g.Mass(), (c.X(), c.Y(), c.Z()), [[t.Value(i, j) for j in (1, 2, 3)] for i in (1, 2, 3)]


def load_map(path):
    with open(path, "rb") as f:
        m = tomllib.load(f)
    materials = {**DEFAULT_MATERIALS, **m.get("materials", {})}
    rules = m.get("part", [])
    for r in rules:
        if r.get("material") not in materials:
            raise ValueError(f"map: part rule {r.get('match')!r} has unknown material {r.get('material')!r}")
    return materials, rules


def bodies_from_step(step, materials, rules):
    """(bodies, unmatched names). The first rule whose glob matches a part's name wins."""
    bodies, unmatched = [], []
    for k, (name, shape) in enumerate(read_parts(step), 1):
        name = name or f"solid_{k}"
        rule = next((r for r in rules if fnmatch.fnmatchcase(name, r["match"])), None)
        if rule is None:
            unmatched.append(name)
            continue
        rho = materials[rule["material"]]
        v, com, tensor = unit_props(shape)
        bodies.append(cadmass.Body(name, rule["role"], rho * v, com, [[rho * x for x in row] for row in tensor]))
    return bodies, unmatched


def report(bodies, roles, out=sys.stdout):
    fmt = "{:<28} {:<9} {:>10} {:>32} {:>12} {:>12} {:>12}"
    print(fmt.format("part", "role", "m kg", "COM m", "Ixx kg m2", "Iyy kg m2", "Izz kg m2"), file=out)
    for b in list(bodies) + [None] + list(roles.values()):
        if b is None:
            print("-- per role", file=out)
            continue
        com = "(" + ", ".join(f"{c:.4g}" for c in b.com) + ")"
        print(fmt.format(b.name, b.role, f"{b.mass:.4g}", com, *(f"{b.inertia[i][i]:.4g}" for i in range(3))), file=out)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("step")
    p.add_argument("--map", required=True)
    p.add_argument("--machine", required=True, choices=sorted(cadmass.ROLE_KEYS))
    p.add_argument("--config", help="complete base config (default: the machine's default.toml)")
    p.add_argument("--out", required=True)
    p.add_argument("--report", action="store_true")
    a = p.parse_args(argv)

    try:
        materials, rules = load_map(a.map)
        bodies, unmatched = bodies_from_step(a.step, materials, rules)
        for n in unmatched:
            print(f"step_mass: not counted, no rule matches: {n}", file=sys.stderr)
        if not bodies:
            raise ValueError("no part matched a rule in the map")
        roles = cadmass.by_role(bodies)
        base = Path(a.config) if a.config else cadmass.default_config(a.machine)
        # newline="" keeps the file's line endings.
        with open(base, encoding="utf-8", newline="") as f:
            text = f.read()
        text = cadmass.apply(text, a.machine, roles, Path(a.step).name)
    except (OSError, ValueError, RuntimeError, KeyError, tomllib.TOMLDecodeError) as e:
        p.exit(2, f"step_mass: {e}\n")
    with open(a.out, "w", encoding="utf-8", newline="") as f:
        f.write(text)
    if a.report:
        report(bodies, roles)
    for key, v in cadmass.values(a.machine, roles).items():
        print(f"mass.{key} = {v:.6g}")


if __name__ == "__main__":
    main()
