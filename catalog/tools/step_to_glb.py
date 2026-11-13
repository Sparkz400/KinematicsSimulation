"""Vendor STEP file to one part's mesh for the app's 3D view (`cad_mesh` in crates/sim-app/src/main.rs).

Runs inside FreeCAD, which reads STEP:

    freecadcmd catalog/tools/step_to_glb.py --pass <step> <out.glb> [keep=A,B] [origin=x,y,z] [turn=rx,ry,rz]
    freecadcmd catalog/tools/step_to_glb.py --pass <step> list

(key=value, since FreeCAD takes any --option for its own.) "list" prints the file's parts, to choose keep from. keep takes the parts whose label contains any of the
words (all parts without it). origin is the point in the STEP file, in mm, that becomes the part's
reference point in the simulation (a box part's centre). turn turns the parts about x, then y, then z, in
degrees, so that x runs along the beam and z up, as in the simulation's scene. The file is written in metres
with y up, which the app reads as is.

Then thin the mesh out, without quantising or compressing it (the app's glTF reader reads neither):

    gltfpack -noq -si 0.3 -i <out.glb> -o crates/sim-app/assets/meshes/<config name>/<part id>.glb

Vendor CAD files may be used but usually not passed on: keep the STEP files and the meshes out of git.
"""

import os
import sys

import FreeCAD
import Import
import Part


def fail(message):
    # FreeCAD drops what sys.exit would print: print it first.
    print(message, flush=True)
    sys.exit(1)

given = sys.argv[sys.argv.index("--pass") + 1 :]
options = {"keep": "", "origin": "0,0,0", "turn": "0,0,0"}
options.update(g.split("=", 1) for g in given if "=" in g)
plain = [g for g in given if "=" not in g]
if not plain or set(options) != {"keep", "origin", "turn"}:
    fail(__doc__)
step, out_file = plain[0], (plain[1:] or [None])[0]
listing = out_file == "list"
if not os.path.isfile(step):
    fail(f"step_to_glb: no file {step}")

doc = FreeCAD.newDocument("source")
Import.insert(step, doc.Name)


def placed(o):
    # A part inside an assembly is in its parent's coordinates: put it where the whole assembly has it.
    s = o.Shape.copy()
    s.Placement = o.getGlobalPlacement()
    return s


solids = [o for o in doc.Objects if o.TypeId == "Part::Feature" and not o.Shape.isNull() and o.Shape.Volume > 0]
if listing:
    for o in solids:
        box = placed(o).BoundBox
        print(f"{o.Label!r}: x {box.XMin:.1f}..{box.XMax:.1f}  y {box.YMin:.1f}..{box.YMax:.1f}  z {box.ZMin:.1f}..{box.ZMax:.1f} mm")
else:
    words = [w for w in options["keep"].split(",") if w]
    kept = [placed(o) for o in solids if not words or any(w in o.Label for w in words)]
    if not kept:
        fail(f"step_to_glb: no part's label contains any of {words}; see list")

    shape = Part.makeCompound(kept)
    origin = FreeCAD.Vector(*(float(v) for v in options["origin"].split(",")))
    rx, ry, rz = (float(v) for v in options["turn"].split(","))
    move = FreeCAD.Placement(FreeCAD.Vector(), FreeCAD.Rotation(rz, ry, rx)).multiply(FreeCAD.Placement(-origin, FreeCAD.Rotation()))
    shape = shape.transformGeometry(move.toMatrix())
    # The glTF writer only writes faces that are already meshed: 0.1 mm deviation is finer than the view shows.
    shape.tessellate(0.1)
    out = FreeCAD.newDocument("out").addObject("Part::Feature", "part")
    out.Shape = shape
    Import.export([out], out_file)
    box = shape.BoundBox
    print(f"step_to_glb: {len(kept)} parts, x {box.XMin:.1f}..{box.XMax:.1f}  y {box.YMin:.1f}..{box.YMax:.1f}  z {box.ZMin:.1f}..{box.ZMax:.1f} mm -> {out_file}")
