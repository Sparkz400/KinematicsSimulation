"""Drive an FMU from a controller of your own: a PD on the motor angle, written here in Python with fmpy, closes the
loop through the drive's torque mode (`command.mode.<motor>` = "torque", `command.torque.<motor>` in N m) at 1 ms.
The torque still passes the drive's torque limit, current loop, winding and DC link, so the current is as real.
Any importer (Simulink, Dymola, OpenModelica, a PLC runtime) does the same with these inputs (docs/fmi.md).

    sim-cli fmu single-axis --out target/single-axis.fmu
    python examples/fmu_controller.py target/single-axis.fmu

Needs fmpy (pip install fmpy==0.3.32). The machine needs one motor.
"""

import math
import sys

from fmpy import extract, read_model_description
from fmpy.fmi2 import FMU2Slave

CYCLE = 0.001  # the PD's sampling time and the communication step, s


def start(path):
    md = read_model_description(path)
    s = FMU2Slave(guid=md.guid, unzipDirectory=extract(path), modelIdentifier=md.coSimulation.modelIdentifier,
                  instanceName="pd")
    s.instantiate()
    s.setupExperiment(startTime=0.0)
    s.enterInitializationMode()
    s.exitInitializationMode()
    return s, {v.name: v.valueReference for v in md.modelVariables}


def run(path, torque_of, seconds):
    """Steps the FMU in torque mode for `seconds`; `torque_of(t, angle, speed)` is the controller, the angle counted
    from where the motor started. Rows of t, angle, speed, current."""
    s, ref = start(path)
    motor = next(n.split(".")[1] for n in ref if n.startswith("motor.") and n.endswith(".angle"))
    out = [ref[f"motor.{motor}.{q}"] for q in ("angle", "speed", "current")]
    s.setString([ref[f"command.mode.{motor}"]], ["torque"])
    zero, speed, current = s.getReal(out)
    angle, rows, t = 0.0, [], 0.0
    while t < seconds - 1e-9:
        s.setReal([ref[f"command.torque.{motor}"]], [torque_of(t, angle, speed)])
        s.doStep(t, CYCLE)
        t += CYCLE
        angle, speed, current = s.getReal(out)
        angle -= zero
        rows.append((t, angle, speed, current))
    s.terminate()
    s.freeInstance()
    return rows


def main(path):
    # The inertia at the motor from two torques held from rest, J = (T2 - T1) / (a2 - a1) between 10 and 30 ms, so
    # that a friction the same at both cancels.
    rates = []
    for torque in (0.5, 1.0):
        rows = run(path, lambda t, a, w, torque=torque: torque, 0.03)
        rates.append((rows[29][2] - rows[9][2]) / 0.02)
    j = 0.5 / (rates[1] - rates[0])
    # PD for a double pole at 20 Hz on that inertia; the move: 20 rad in 0.4 s with a cosine speed, then held.
    w = 2.0 * math.pi * 20.0
    kp, kd = j * w * w, 2.0 * j * w
    goal = lambda t: 20.0 * min(t / 0.4, 1.0) - 20.0 / (2 * math.pi) * math.sin(2 * math.pi * min(t / 0.4, 1.0))
    rate = lambda t: 20.0 / 0.4 * (1 - math.cos(2 * math.pi * t / 0.4)) if t < 0.4 else 0.0
    rows = run(path, lambda t, a, w_: kp * (goal(t) - a) + kd * (rate(t) - w_), 0.6)
    print(f"inertia at the motor {j:.4g} kg m^2, kp {kp:.4g} N m/rad, kd {kd:.4g} N m s/rad")
    print("time_s,setpoint_rad,angle_rad,error_rad,current_a")
    for t, a, _, i in rows[99::100]:
        print(f"{t:.3f},{goal(t):.4f},{a:.4f},{goal(t) - a:.2e},{i:.3f}")
    print(f"peak following error {max(abs(goal(t) - a) for t, a, _, _ in rows):.3e} rad")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "target/single-axis.fmu")
