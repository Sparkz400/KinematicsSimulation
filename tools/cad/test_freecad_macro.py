"""Runs under freecadcmd: freecadcmd tools/cad/test_freecad_macro.py. Prints FREECAD MACRO TESTS PASSED at the end."""

import math
import os
import sys
import tempfile
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
try:
    import FreeCAD as App
except ImportError:  # imported by unittest discover in plain Python: test_cadmass runs this file under freecadcmd
    App = None
else:
    import cadmass
    import simsizing


def close(got, want, what, rel=1e-6):
    assert abs(got - want) <= rel * max(abs(want), 1e-30), f"{what}: {got} != {want}"


def check_tensor(got, want, what):
    scale = max(want[i][i] for i in range(3))
    for i in range(3):
        for j in range(3):
            assert abs(got[i][j] - want[i][j]) <= 1e-6 * scale, f"{what} I[{i}][{j}]: {got[i][j]} != {want[i][j]}"


def main():
    doc = App.newDocument("SizingTest")
    box = doc.addObject("Part::Box", "Carriage")
    box.Length, box.Width, box.Height = 100, 50, 20
    box.Placement.Base = App.Vector(10, 20, 30)
    # The cylinder sits in an App::Part that is moved: its global placement must count.
    part = doc.addObject("App::Part", "Tool")
    part.Placement.Base = App.Vector(200, 0, 0)
    cyl = doc.addObject("Part::Cylinder", "Payload")
    cyl.Radius, cyl.Height = 20, 50
    part.addObject(cyl)
    other = doc.addObject("Part::Box", "Untagged")
    doc.recompute()
    simsizing.tag([box], "carriage", "aluminium")
    simsizing.tag([cyl], "payload", "", density=7850.0)
    simsizing.tag([other], "")

    got = {b.role: b for b in simsizing.bodies(doc)}
    assert set(got) == {"carriage", "payload"}, got
    a, b, c = 0.1, 0.05, 0.02
    m = 2700 * a * b * c
    close(got["carriage"].mass, m, "box mass")
    for g, w in zip(got["carriage"].com, (0.06, 0.045, 0.04)):
        close(g, w, "box COM")
    check_tensor(got["carriage"].inertia, [[m * (b * b + c * c) / 12, 0, 0], [0, m * (a * a + c * c) / 12, 0],
                                           [0, 0, m * (a * a + b * b) / 12]], "box")
    r, h = 0.02, 0.05
    m = 7850 * math.pi * r * r * h
    close(got["payload"].mass, m, "cylinder mass")
    for g, w in zip(got["payload"].com, (0.2, 0.0, 0.025)):
        assert abs(g - w) < 1e-9, f"cylinder COM {got['payload'].com}"
    side = m * (3 * r * r + h * h) / 12
    check_tensor(got["payload"].inertia, [[side, 0, 0], [0, side, 0], [0, 0, m * r * r / 2]], "cylinder")

    # The tags are document properties: they survive saving and loading.
    path = os.path.join(tempfile.mkdtemp(), "sizing.FCStd")
    doc.saveAs(path)
    App.closeDocument(doc.Name)
    doc = App.openDocument(path)
    assert doc.getObject("Carriage").SimRole == "carriage"

    skipped = ""
    sim = cadmass.find_sim_cli()
    try:
        sim and cadmass.run_verdicts(sim, "t-gantry", cadmass.default_config("t-gantry"))
    except RuntimeError as e:
        skipped = f" (verdicts skipped: {sim} is older than the config, set SIM_CLI: {e})"
    if not sim:
        skipped = " (verdicts skipped: no sim-cli, set SIM_CLI)"
    if not skipped:
        s = {"machine": "t-gantry", "sim_cli": "", "base_config": "", "out_file": ""}
        roles, out, rows = simsizing.export(doc, s)
        assert out.is_file() and rows, rows
        text = out.read_text(encoding="utf-8")
        assert f"# [cad] {doc.Name}: m=0.27 kg" in text, text
        assert "z_axis_kg = 5.0" in text, "untouched role keeps its value"
        lines = simsizing.report_lines("t-gantry", roles, out, rows)
        print("\n".join(lines))

    # The dialog without a FreeCAD window: offscreen Qt, the macro loaded under another name.
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PySide import QtGui
    app = QtGui.QApplication.instance() or QtGui.QApplication([])
    ns = {"__name__": "simsizing_macro", "__file__": os.path.join(HERE, "SimSizing.FCMacro")}
    with open(ns["__file__"], encoding="utf-8") as f:
        exec(compile(f.read(), ns["__file__"], "exec"), ns)
    dlg = ns["Dialog"]()
    rows = cadmass.parse_verdicts("check,motor,value,limit,result\nmotor-peak,a,0.5,1.000,ok\ngearbox-peak,a,80,64,FAIL\n")
    dlg.show_rows(rows)
    assert dlg.table.rowCount() == 2 and dlg.table.item(0, 4).text() == "+50.0 %" and dlg.table.item(1, 5).text() == "FAIL"
    assert dlg.table.item(1, 5).background().color().name() == "#f0a0a0"
    print("FREECAD MACRO TESTS PASSED" + skipped)


if App is not None:
    try:
        main()
    except Exception:
        traceback.print_exc()
        print("FREECAD MACRO TESTS FAILED", flush=True)
