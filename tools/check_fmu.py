"""Independent check of an FMU written by `sim-cli fmu` (FMI 2.0 or, with --fmi 3, FMI 3.0), with fmpy (CI runs it on
Windows and Linux).

usage: python tools/check_fmu.py SIM_CLI FMU MACHINE [--config TOML] [--stop S]

MACHINE is a registered machine's id, or a machine file (.toml or .json) that `sim-cli run --machine-file` reads.

1. fmpy's validator (the XSD of the FMU's FMI version, names, model structure, start values, causality, units) finds
   no problem.
2. fmpy runs the FMU as co-simulation at its default step with no input; every output must equal
   `sim-cli run` of the same machine and config at the same instants (the CLI prints 9 digits).
3. Through fmpy's own bindings of the C functions: an FMU state taken halfway brings back bit for bit the outputs of
   the steps after it, set on the same instance and serialized onto a second one; the directional derivative of
   the outputs by the first setpoint input is zero, as is the finite difference by hand.
4. A move through the inputs command.target.* and command.move gives the outputs of `sim-cli run --move`; with the
   log category logEvents switched on it is one message of that category, and with all switched off nothing is logged.
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
    machine = ["--machine-file", a.machine] if a.machine.endswith((".toml", ".json")) else [a.machine]
    cmd = [os.path.abspath(a.cli), "run", *machine, "--time", str(a.stop), "--every", "1", "--out", out]
    if a.config:
        cmd[2 + len(machine):2 + len(machine)] = ["--config", a.config]
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

# 4. The move inputs and the log categories: the first axis moved 10 mm (or rad) through command.target and
#    command.move gives the outputs of `sim-cli run --move` at the same instants; with logEvents switched on the move
#    is one message of that category, and with every category off a refused call gives none.
targets = [v for v in md.modelVariables if v.name.startswith("command.target.")]
move = next(v for v in md.modelVariables if v.name == "command.move")
messages = []


def started_logging():
    cls = FMU3Slave if v3 else FMU2Slave
    s = cls(guid=md.guid, unzipDirectory=unzipped, modelIdentifier=md.coSimulation.modelIdentifier, instanceName="check")
    if v3:
        s.instantiate(logMessage=lambda env, status, category, message: messages.append((category.decode(), message.decode())))
        s.enterInitializationMode(startTime=0.0)
    else:
        from fmpy.fmi2 import defaultCallbacks, fmi2CallbackFunctions, fmi2CallbackLoggerTYPE
        cb = fmi2CallbackFunctions()
        cb.logger = fmi2CallbackLoggerTYPE(lambda env, name, status, category, message: messages.append((category.decode(), message.decode())))
        cb.allocateMemory, cb.freeMemory = defaultCallbacks.allocateMemory, defaultCallbacks.freeMemory
        s.instantiate(callbacks=cb)
        s.setupExperiment(startTime=0.0)
        s.enterInitializationMode()
    s.exitInitializationMode()
    return s


unzipped = extract(a.fmu)
moved = None
for offset in (0.01, -0.01):
    target = [float(v.start) for v in targets]
    target[0] += offset
    with tempfile.TemporaryDirectory() as d:
        out = os.path.join(d, "move.csv")
        machine = ["--machine-file", a.machine] if a.machine.endswith((".toml", ".json")) else [a.machine]
        cmd = [os.path.abspath(a.cli), "run", *machine, *(["--config", a.config] if a.config else []), "--move",
               ",".join(repr(x) for x in target), "--time", str(a.stop), "--every", "1", "--out", out]
        if subprocess.run(cmd, capture_output=True).returncode == 0:
            moved = np.genfromtxt(out, delimiter=",", names=True, deletechars="")
            break
if moved is None:
    print("sim-cli refuses a move of the first axis by 10 mm either way"); failed = True
else:
    s = started_logging()
    s.setDebugLogging(True, ["logEvents"])
    put(s, [v.valueReference for v in targets], target)
    s.setBoolean([move.valueReference], [True])
    steps = int(round(a.stop / step))
    rows = []
    for k in range(steps):
        s.doStep(k * step, step)
        rows.append(get(s, out_refs))
    events = [m for m in messages if m[0] == "logEvents"]
    one_event = len(messages) == 1 and len(events) == 1 and "MoveTo" in events[0][1]
    s.setDebugLogging(False, [])
    try:
        get(s, [max(v.valueReference for v in md.modelVariables) + 1])
    except Exception:
        pass
    quiet = len(messages) == 1
    s.terminate(); s.freeInstance()
    mcols = {n.split("[")[0]: n for n in moved.dtype.names}
    times = np.array([(k + 1) * step for k in range(steps)])
    mrows = np.searchsorted(moved[mcols["time"]], times - 1e-9)
    move_worst = 0.0
    for i, name in enumerate(outputs):
        f, c = np.array([r[i] for r in rows]), moved[mcols[name]][mrows]
        move_worst = max(move_worst, np.max(np.abs(f - c)) / max(np.max(np.abs(c)), 1e-12))
    print(f"move input: worst {move_worst:.1e} against sim-cli run --move; log categories: "
          f"{'one logEvents message' if one_event else f'UNEXPECTED {messages}'}, "
          f"{'none with logging off' if quiet else 'MESSAGES WITH LOGGING OFF'}")
    failed |= move_worst > REL_TOL or not one_event or not quiet
shutil.rmtree(unzipped, ignore_errors=True)

print(f"{a.machine} {a.config or ''} (FMI {md.fmiVersion}): {len(problems)} validator problems, {len(outputs)} outputs, "
      f"{len(fmu)} rows, worst {worst[0]:.1e} ({worst[1]}): {'FAIL' if failed else 'ok'}")
sys.exit(1 if failed else 0)
