"""Independent check of an FMU written by `sim-cli fmu` (FMI 2.0 or, with --fmi 3, FMI 3.0), with fmpy (CI runs it on
Windows and Linux).

usage: python tools/check_fmu.py SIM_CLI FMU MACHINE [--config TOML] [--stop S]

1. fmpy's validator (the XSD of the FMU's FMI version, names, model structure, start values, causality, units) finds
   no problem.
2. fmpy runs the FMU as co-simulation at its default step with no input; every output must equal
   `sim-cli run` of the same machine and config at the same instants (the CLI prints 9 digits).
3. Through fmpy's own bindings of the C functions: an FMU state taken halfway brings back bit for bit the outputs of
   the steps after it, set on the same instance and serialized onto a second one; the directional derivative of
   the outputs by the first setpoint input is zero, as is the finite difference by hand.
Exits 1 on any failure.
"""
import argparse, os, shutil, subprocess, sys, tempfile
import numpy as np
from fmpy import extract, read_model_description, simulate_fmu
from fmpy.fmi2 import FMU2Slave
from fmpy.fmi3 import FMU3Slave
from fmpy.validation import validate_fmu

REL_TOL = 1e-8  # the CLI's 9 printed digits; the hand check found 5e-10

ap = argparse.ArgumentParser()
ap.add_argument("cli"); ap.add_argument("fmu"); ap.add_argument("machine")
ap.add_argument("--config"); ap.add_argument("--stop", type=float, default=1.0)
a = ap.parse_args()

failed = False
problems = validate_fmu(a.fmu)
for p in problems:
    print("validator:", p)
failed |= bool(problems)

md = read_model_description(a.fmu)
v3 = md.fmiVersion.startswith("3")
step = float(md.defaultExperiment.stepSize)
outputs = [v.name for v in md.modelVariables if v.causality == "output"]
logs = []
fmu = simulate_fmu(a.fmu, fmi_type="CoSimulation", stop_time=a.stop, step_size=step, output_interval=step,
                   output=outputs, logger=lambda *m: logs.append(m))
for m in logs:
    print("fmu log:", m)

with tempfile.TemporaryDirectory() as d:
    out = os.path.join(d, "cli.csv")
    cmd = [os.path.abspath(a.cli), "run", a.machine, "--time", str(a.stop), "--every", "1", "--out", out]
    if a.config:
        cmd[3:3] = ["--config", a.config]
    subprocess.run(cmd, check=True)
    cli = np.genfromtxt(out, delimiter=",", names=True, deletechars="")
cols = {n.split("[")[0]: n for n in cli.dtype.names}
rows = np.searchsorted(cli[cols["time"]], fmu["time"] - 1e-9)
if not np.allclose(cli[cols["time"]][rows], fmu["time"], atol=1e-9):
    print("time grids differ"); sys.exit(1)

worst = (0.0, "")
for name in outputs:
    if name not in cols:
        print(f"output {name} not in sim-cli run"); failed = True; continue
    f, c = fmu[name].astype(float), cli[cols[name]][rows]
    scale = max(np.max(np.abs(c)), 1e-12)
    rel = np.max(np.abs(f - c)) / scale
    worst = max(worst, (rel, name))
    if rel > REL_TOL:
        print(f"{name}: FMU differs from sim-cli by {rel:.2e} of its peak"); failed = True

# 3. FMU state and derivatives, called the way an importing tool calls them.
out_refs = [v.valueReference for v in md.modelVariables if v.causality == "output"]
setpoint = next(v for v in md.modelVariables if v.name.startswith("command.setpoint."))
unzipped = extract(a.fmu)


def started():
    cls = FMU3Slave if v3 else FMU2Slave
    s = cls(guid=md.guid, unzipDirectory=unzipped, modelIdentifier=md.coSimulation.modelIdentifier, instanceName="check")
    s.instantiate()
    if v3:
        s.enterInitializationMode(startTime=0.0)
    else:
        s.setupExperiment(startTime=0.0)
        s.enterInitializationMode()
    s.exitInitializationMode()
    return s


get = lambda s, refs: list(s.getFloat64(refs) if v3 else s.getReal(refs))
put = lambda s, refs, values: s.setFloat64(refs, values) if v3 else s.setReal(refs, values)


def run(s, ks):
    """Steps k with the first setpoint 1 mm on per step (8 mm at most); the outputs after each."""
    rows = []
    for k in ks:
        put(s, [setpoint.valueReference], [float(setpoint.start) + 0.001 * min(k, 8)])
        s.doStep(k * step, step)
        rows.append(get(s, out_refs))
    return rows


s = started()
run(s, range(0, 5))
state = s.getFMUState() if v3 else s.getFMUstate()
after = run(s, range(5, 12))
(s.setFMUState if v3 else s.setFMUstate)(state)
same = run(s, range(5, 12)) == after
data = (s.serializeFMUState if v3 else s.serializeFMUstate)(state)
(s.freeFMUState if v3 else s.freeFMUstate)(state)
s.terminate(); s.freeInstance()
t = started()
restored = (t.deserializeFMUState if v3 else t.deSerializeFMUstate)(data)
(t.setFMUState if v3 else t.setFMUstate)(restored)
elsewhere = run(t, range(5, 12)) == after
base = get(t, out_refs)
derivative = t.getDirectionalDerivative(out_refs, [setpoint.valueReference], [1.0])
u = get(t, [setpoint.valueReference])[0]
put(t, [setpoint.valueReference], [u + 1e-4])
by_hand = [(x - y) / 1e-4 for x, y in zip(get(t, out_refs), base)]
t.terminate(); t.freeInstance()
shutil.rmtree(unzipped, ignore_errors=True)
zero = all(x == 0.0 for x in derivative) and all(x == 0.0 for x in by_hand)
print(f"FMU state: restored {'bit for bit' if same else 'DIFFERENT'}, serialized {len(data)} bytes and restored "
      f"{'bit for bit' if elsewhere else 'DIFFERENT'}; directional derivative and finite difference "
      f"{'zero' if zero else 'NOT ZERO'}")
failed |= not (same and elsewhere and zero)

print(f"{a.machine} {a.config or ''} (FMI {md.fmiVersion}): {len(problems)} validator problems, {len(outputs)} outputs, "
      f"{len(fmu)} rows, worst {worst[0]:.1e} ({worst[1]}): {'FAIL' if failed else 'ok'}")
sys.exit(1 if failed else 0)
