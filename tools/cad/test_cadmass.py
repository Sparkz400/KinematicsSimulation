"""python -m unittest discover tools/cad"""

import difflib
import math
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import cadmass  # noqa: E402
import step_mass  # noqa: E402

try:
    import OCP  # noqa: F401
    HAVE_OCP = True
except ImportError:
    HAVE_OCP = False

RHO = 2700.0
FREECADCMD = os.environ.get("FREECADCMD") or shutil.which("freecadcmd") or next(
    (p for p in [Path(os.environ.get("LOCALAPPDATA", "")) / "Programs/FreeCAD 1.1/bin/freecadcmd.exe"] if p.is_file()), None)


def write_step(path, parts, unit="MM"):
    """parts: [(name, shape in mm, (dx, dy, dz) mm)] as components of one assembly."""
    from OCP.Interface import Interface_Static
    from OCP.STEPCAFControl import STEPCAFControl_Writer
    from OCP.STEPControl import STEPControl_AsIs, STEPControl_Controller
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDataStd import TDataStd_Name
    from OCP.TDocStd import TDocStd_Document
    from OCP.TopLoc import TopLoc_Location
    from OCP.XCAFApp import XCAFApp_Application
    from OCP.XCAFDoc import XCAFDoc_DocumentTool
    from OCP.gp import gp_Trsf, gp_Vec

    doc = TDocStd_Document(TCollection_ExtendedString("XmlOcaf"))
    XCAFApp_Application.GetApplication_s().InitDocument(doc)
    if unit == "M":
        # Shapes are taken in the document's unit: metres here, so the file holds metre values.
        from OCP.UnitsMethods import UnitsMethods_LengthUnit_Meter
        XCAFDoc_DocumentTool.SetLengthUnit_s(doc, 1.0, UnitsMethods_LengthUnit_Meter)
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    asm = tool.NewShape()
    TDataStd_Name.Set_s(asm, TCollection_ExtendedString("assembly"))
    for name, shape, offset in parts:
        label = tool.AddShape(shape, False)
        TDataStd_Name.Set_s(label, TCollection_ExtendedString(name))
        t = gp_Trsf()
        t.SetTranslation(gp_Vec(*offset))
        tool.AddComponent(asm, label, TopLoc_Location(t))
    tool.UpdateAssemblies()
    STEPControl_Controller.Init_s()
    Interface_Static.SetCVal_s("write.step.unit", unit)
    w = STEPCAFControl_Writer()
    w.SetNameMode(True)
    w.Transfer(doc, STEPControl_AsIs)
    w.Write(str(path))
    Interface_Static.SetCVal_s("write.step.unit", "MM")


def box(a, b, c):
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    return BRepPrimAPI_MakeBox(a, b, c).Shape()


def cylinder(r, h, axis="z"):
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
    from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt
    d = {"x": (1, 0, 0), "y": (0, 1, 0), "z": (0, 0, 1)}[axis]
    return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(*d)), r, h).Shape()


def tube(r1, r2, h):
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
    return BRepAlgoAPI_Cut(cylinder(r2, h), cylinder(r1, h)).Shape()


def diag(a, b, c):
    return [[a, 0, 0], [0, b, 0], [0, 0, c]]


class Close:
    def assertTensor(self, got, want, rel=1e-6):
        scale = max(abs(want[i][i]) for i in range(3))
        for i in range(3):
            for j in range(3):
                self.assertLessEqual(abs(got[i][j] - want[i][j]), rel * scale, f"I[{i}][{j}] {got[i][j]} != {want[i][j]}")

    def assertRel(self, got, want, rel=1e-6):
        self.assertLessEqual(abs(got - want), rel * max(abs(want), 1e-30), f"{got} != {want}")

    def assertVec(self, got, want, tol=1e-9):
        for g, w in zip(got, want):
            self.assertLessEqual(abs(g - w), tol, f"{got} != {want}")


@unittest.skipUnless(HAVE_OCP, "OCP missing: pip install cadquery-ocp")
class StepTests(unittest.TestCase, Close):
    def setUp(self):
        self.dir = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, self.dir, True)

    def bodies(self, parts, unit="MM", rules=None):
        path = self.dir / "model.step"
        write_step(path, parts, unit)
        rules = rules or [{"match": "*", "role": "carriage", "material": "aluminium"}]
        bodies, unmatched = step_mass.bodies_from_step(path, step_mass.DEFAULT_MATERIALS, rules)
        return {b.name: b for b in bodies}, unmatched

    def test_primitives_against_formulas(self):
        a, b, c = 0.04, 0.06, 0.1
        r, h = 0.03, 0.12
        r1, r2 = 0.02, 0.025
        got, _ = self.bodies([("box", box(40, 60, 100), (0, 0, 0)), ("cyl", cylinder(30, 120), (0, 0, 0)),
                              ("tube", tube(20, 25, 120), (0, 0, 0))])
        m = RHO * a * b * c
        self.assertRel(got["box"].mass, m)
        self.assertVec(got["box"].com, (a / 2, b / 2, c / 2))
        self.assertTensor(got["box"].inertia, diag(m * (b * b + c * c) / 12, m * (a * a + c * c) / 12, m * (a * a + b * b) / 12))
        m = RHO * math.pi * r * r * h
        self.assertRel(got["cyl"].mass, m)
        self.assertVec(got["cyl"].com, (0, 0, h / 2))
        side = m * (3 * r * r + h * h) / 12
        self.assertTensor(got["cyl"].inertia, diag(side, side, m * r * r / 2))
        m = RHO * math.pi * (r2 * r2 - r1 * r1) * h
        self.assertRel(got["tube"].mass, m)
        side = m * (3 * (r1 * r1 + r2 * r2) + h * h) / 12
        self.assertTensor(got["tube"].inertia, diag(side, side, m * (r1 * r1 + r2 * r2) / 2))

    def test_metre_step_reads_the_same(self):
        mm, _ = self.bodies([("box", box(40, 60, 100), (0, 0, 0))])
        m, _ = self.bodies([("box", box(0.04, 0.06, 0.1), (0, 0, 0))], unit="M")
        self.assertRel(m["box"].mass, mm["box"].mass)
        self.assertTensor(m["box"].inertia, mm["box"].inertia)

    def test_placed_parts_and_combine_against_occ(self):
        from OCP.TopoDS import TopoDS_Compound
        from OCP.BRep import BRep_Builder
        from OCP.TopLoc import TopLoc_Location
        from OCP.gp import gp_Trsf, gp_Vec

        got, _ = self.bodies([("box", box(40, 60, 100), (100, 50, -20)), ("cyl", cylinder(30, 120), (-30, 20, 10))])
        self.assertVec(got["box"].com, (0.12, 0.08, 0.03))
        self.assertVec(got["cyl"].com, (-0.03, 0.02, 0.07))
        both = cadmass.combine(list(got.values()))
        # OCC's own properties of both shapes together, scaled to metres, are the reference for the parallel axis.
        comp, bld = TopoDS_Compound(), BRep_Builder()
        bld.MakeCompound(comp)
        for shape, off in ((box(40, 60, 100), (100, 50, -20)), (cylinder(30, 120), (-30, 20, 10))):
            t = gp_Trsf()
            t.SetTranslation(gp_Vec(*off))
            bld.Add(comp, shape.Moved(TopLoc_Location(t)))
        v, com, tensor = step_mass.unit_props(comp)
        self.assertRel(both.mass, RHO * v * 1e-9)
        self.assertVec(both.com, [c * 1e-3 for c in com])
        self.assertTensor(both.inertia, [[RHO * x * 1e-15 for x in row] for row in tensor])
        self.assertGreater(abs(both.inertia[0][1]), 1e-6 * both.inertia[0][0], "test needs off-diagonal terms")

    def test_delta_arm_about_shoulder_axis(self):
        r, L, y0 = 0.02, 0.4, 0.01
        got, unmatched = self.bodies(
            [("arm tube", cylinder(20, 400, "y"), (0, 10, 0)), ("bolt", box(5, 5, 5), (0, 0, 0))],
            rules=[{"match": "arm*", "role": "arm", "material": "carbon"}])
        self.assertEqual(unmatched, ["bolt"])
        m = 1550 * math.pi * r * r * L
        vals = cadmass.values("delta-d3", cadmass.by_role(got.values()))
        self.assertRel(vals["upper_arm_kg"], m)
        self.assertRel(vals["upper_arm_cg_m"], y0 + L / 2)
        self.assertRel(vals["upper_arm_inertia_kgm2"], m * (3 * r * r + L * L) / 12 + m * (y0 + L / 2) ** 2)

    def test_cli_writes_config(self):
        write_step(self.dir / "m.step", [("Carriage plate", box(200, 100, 10), (0, 0, 0)), ("Screw", box(5, 5, 5), (0, 0, 0))])
        (self.dir / "map.toml").write_text('[materials]\nmystery = 1000.0\n\n[[part]]\nmatch = "Carriage*"\nrole = "carriage"\nmaterial = "steel"\n')
        out = self.dir / "out.toml"
        step_mass.main([str(self.dir / "m.step"), "--map", str(self.dir / "map.toml"), "--machine", "t-gantry",
                        "--out", str(out)])
        text = out.read_text(encoding="utf-8")
        self.assertIn("carriage_kg = 1.57\n", text)
        self.assertIn("# [cad] m.step: m=1.57 kg", text)


T_GANTRY = cadmass.default_config("t-gantry")
DELTA = cadmass.default_config("delta-d3")


def body(role, mass, com=(0.0, 0.0, 0.0), inertia=None):
    return cadmass.Body(role, role, mass, com, inertia or diag(1e-3, 2e-3, 3e-3))


class PureTests(unittest.TestCase, Close):
    def test_combine_parallel_axis(self):
        a = body("a", 2.0, (1.0, 0.0, 0.0))
        b = body("a", 2.0, (-1.0, 0.0, 2.0))
        c = cadmass.combine([a, b])
        self.assertVec(c.com, (0.0, 0.0, 1.0))
        # d = (+-1, 0, -+1): each adds m(|d|^2 E - d d^T) = 2*[[1,0,1],[0,2,0],[1,0,1]].
        self.assertTensor(c.inertia, [[2e-3 + 4, 0, 4], [0, 4e-3 + 8, 0], [4, 0, 6e-3 + 4]])

    def test_about_axis(self):
        cg, i = cadmass.about_axis(body("arm", 2.0, (5.0, 0.3, 0.4)))
        self.assertRel(cg, 0.5)
        self.assertRel(i, 1e-3 + 2.0 * 0.25)

    def test_apply_changes_only_the_mapped_lines(self):
        old = T_GANTRY.read_text(encoding="utf-8")
        roles = {"carriage": body("carriage", 3.25), "arm": body("arm", 6.5), "payload": body("payload", 1.0)}
        new = cadmass.apply(old, "t-gantry", roles, "gantry.step")
        diff = [l for l in difflib.unified_diff(old.splitlines(), new.splitlines(), lineterm="", n=0) if l[:1] in "+-" and l[:3] not in ("+++", "---")]
        self.assertEqual([l for l in diff if l.startswith("-")], ["-carriage_kg = 4.0", "-z_axis_kg = 5.0", "-payload_kg = 2.0"])
        added = [l for l in diff if l.startswith("+")]
        self.assertEqual([l for l in added if not l.startswith("+# [cad] gantry.step: ")],
                         ["+carriage_kg = 3.25", "+z_axis_kg = 6.5", "+payload_kg = 1.0"])
        self.assertEqual(len(added), 6)
        again = cadmass.apply(new, "t-gantry", roles, "gantry.step")
        self.assertEqual(again, new, "a second apply replaces the [cad] lines, not stacks them")
        crlf = cadmass.apply(old.replace("\n", "\r\n"), "t-gantry", roles, "gantry.step")
        self.assertEqual(crlf, new.replace("\n", "\r\n"))

    def test_apply_delta_keeps_catalogue_comment(self):
        new = cadmass.apply(DELTA.read_text(encoding="utf-8"), "delta-d3",
                            {"payload": body("payload", 2.5), "arm": body("arm", 1.0, (0, 0.2, 0))}, "arm.step")
        lines = new.splitlines()
        i = lines.index("payload_kg = 2.5")
        self.assertTrue(lines[i - 1].startswith("# [cad] arm.step: m=2.5 kg"))
        self.assertTrue(lines[i - 2].startswith("# [catalogue]"))
        self.assertIn("upper_arm_cg_m = 0.2", lines)
        self.assertIn("mass.upper_arm_kg = [0.6, 1.2]", lines, "keys outside [mass] stay")

    def test_apply_errors(self):
        with self.assertRaises(ValueError):
            cadmass.apply("[mass]\ncarriage_kg = 1.0\n", "single-axis", {"arm": body("arm", 1.0)}, "x")
        with self.assertRaises(ValueError):
            cadmass.apply("[mass]\ncarriage_kg = 1.0\n", "single-axis", {"payload": body("payload", 1.0)}, "x")

    def test_apply_crlf_without_last_newline_and_old_trailing_tag(self):
        text = "[mass]\r\ncarriage_kg = 1.0   # [estimate] a guess\r\npayload_kg = 2.0"
        out = cadmass.apply(text, "single-axis", {"carriage": body("carriage", 3.0), "payload": body("payload", 4.0)}, "x")
        self.assertNotIn("[estimate]", out)
        self.assertEqual(out.count("\n"), out.count("\r\n"), repr(out))
        self.assertTrue(out.endswith("payload_kg = 4.0"))

    def test_parse_verdicts_margin_direction(self):
        rows = cadmass.parse_verdicts("check,motor,value,limit,result\nmotor-peak,a,0.5,1.000,ok\n"
                                      "guide-life,,40000,20000,ok\ngearbox-peak,a,80,64,FAIL\nbelt-slack,,10,0,ok\n")
        self.assertEqual([r["margin"] for r in rows], [0.5, 0.5, -0.25, 1.0])
        self.assertEqual([r["ok"] for r in rows], [True, True, False, True])

    def test_applied_config_loads_in_sim_cli(self):
        sim = cadmass.find_sim_cli()
        if not sim:
            self.skipTest("no sim-cli: set SIM_CLI or build target/release/sim-cli")
        try:
            cadmass.run_verdicts(sim, "t-gantry", T_GANTRY)
        except RuntimeError as e:
            self.skipTest(f"{sim} cannot read the unchanged default.toml, so it is older than the config (set SIM_CLI to a fresh build): {e}")
        roles = {"carriage": body("carriage", 3.25), "payload": body("payload", 1.0)}
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "cad.toml"
            path.write_text(cadmass.apply(T_GANTRY.read_text(encoding="utf-8"), "t-gantry", roles, "x.step"), encoding="utf-8")
            rows = cadmass.parse_verdicts(cadmass.run_verdicts(sim, "t-gantry", path))
        self.assertTrue(rows)
        self.assertTrue(all(r["result"] in ("ok", "FAIL") for r in rows))


class FreeCadTests(unittest.TestCase):
    @unittest.skipUnless(FREECADCMD, "FreeCAD missing: set FREECADCMD to freecadcmd")
    def test_macro_logic_under_freecadcmd(self):
        r = subprocess.run([str(FREECADCMD), str(HERE / "test_freecad_macro.py")], capture_output=True, text=True, timeout=300)
        out = r.stdout + r.stderr
        self.assertIn("FREECAD MACRO TESTS PASSED", out, out[-3000:])
        if "(verdicts skipped" in out:
            self.skipTest(out[out.index("(verdicts skipped"):].splitlines()[0])


if __name__ == "__main__":
    unittest.main()
