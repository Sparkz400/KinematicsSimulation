"""Test a controller of your own on the simulated mechanics: a PD position controller written in Python closes the
loop through torque mode, and is compared with the drive's built-in cascade on the same move.

    python crates/sim-py/examples/own_controller.py [MACHINE]

MACHINE is a machine with one motor (default: the first of the registry). Each `sim.step(torque=[...], time=CYCLE)`
runs one cycle of the PD (1 ms) with the torque held; the torque still passes the drive's torque limit, current loop,
winding and DC link, so `sim.verdicts()` judges the run as it judges the built-in drive's.
"""

import math
import sys

import kinematics_sim as ks

CYCLE = 0.001  # the PD's sampling time, s


def one_motor_machine() -> str:
    """The first machine of the registry with one motor."""
    for m in ks.machines():
        ids = ks.signals(ks.Config(m["id"]))["id"]
        if sum(1 for i in ids if i.startswith("motor.") and i.endswith(".angle")) == 1:
            return m["id"]
    raise LookupError("no machine with one motor")


def move(start: float, distance: float, duration: float, t: float) -> float:
    """A smooth move (cosine velocity) of `distance` in `duration`, then held."""
    s = min(max(t / duration, 0.0), 1.0)
    return start + distance * (s - math.sin(2.0 * math.pi * s) / (2.0 * math.pi))


def built_in(cfg: ks.Config, seconds: float):
    """The built-in cascade following the move as position setpoints: the motor angle setpoint per cycle, and the sim."""
    sim = ks.Sim(cfg)
    axis = next(i for i in sim.signals() if i.startswith("axis.") and i.endswith(".position"))
    start = sim.signals()[axis]
    motor = next(i for i in sim.signals() if i.startswith("motor.") and i.endswith(".angle")).rsplit(".", 1)[0]
    angles = []
    while sim.time < seconds - 1e-9:
        out = sim.step(setpoint=[move(start, 0.1, 0.3, sim.time + CYCLE)], time=CYCLE)
        angles.append(out[f"{motor}.angle_setpoint"])
    return sim, motor, angles


def inertia(cfg: ks.Config, motor: str) -> float:
    """The moment of inertia at the motor, measured with two torques from rest: each one's acceleration between 10
    and 30 ms, J = (T2 - T1) / (a2 - a1), so that a friction that is the same at both cancels."""
    rates = []
    for torque in (0.5, 1.0):
        sim = ks.Sim(cfg)
        w1 = sim.step(torque=[torque], time=0.01)[f"{motor}.speed"]
        w2 = sim.step(time=0.02)[f"{motor}.speed"]
        rates.append((w2 - w1) / 0.02)
    return 0.5 / (rates[1] - rates[0])


def own_pd(cfg: ks.Config, motor: str, angles: list[float], bandwidth_hz: float = 20.0):
    """A PD on the motor angle, tuned on the measured inertia for a double pole at `bandwidth_hz`."""
    j = inertia(cfg, motor)
    w = 2.0 * math.pi * bandwidth_hz
    kp, kd = j * w * w, 2.0 * j * w
    sim = ks.Sim(cfg)
    out, last = sim.signals(), angles[0]
    for target in angles:
        rate = (target - last) / CYCLE
        torque = kp * (target - out[f"{motor}.angle"]) + kd * (rate - out[f"{motor}.speed"])
        out, last = sim.step(torque=[torque], time=CYCLE), target
    return sim, j


def errors(sim: ks.Sim) -> str:
    load = sim.analyze()
    failing = [c for c, ok in zip(sim.verdicts()["check"], sim.verdicts()["ok"]) if not ok]
    return (
        f"peak torque {load['peak_torque_nm'][0]:.3f} N m, RMS {load['rms_torque_nm'][0]:.3f} N m, "
        f"RMS current {load['rms_current_a'][0]:.3f} A, failing checks: {failing or 'none'}"
    )


def main(machine: str | None = None, seconds: float = 0.5) -> dict[str, float]:
    cfg = ks.Config(machine or one_motor_machine())
    reference, motor, angles = built_in(cfg, seconds)
    own, j = own_pd(cfg, motor, angles)
    off = abs(own.signals()[f"{motor}.angle"] - angles[-1])
    print(f"{cfg.machine}: inertia at the motor {j:.4g} kg m^2")
    print(f"built-in cascade: {errors(reference)}")
    print(f"own PD:           {errors(own)}; {off:.2e} rad from the target at the end")
    return {"inertia": j, "off": off, "built_in_rms": reference.analyze()["rms_torque_nm"][0],
            "own_rms": own.analyze()["rms_torque_nm"][0]}


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else None)
