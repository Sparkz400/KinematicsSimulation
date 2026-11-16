"""Independent reference for the overhead crane: the same equations, written again in Python and integrated by SciPy's
LSODA instead of the simulator's fixed-step RK4.

Reads ../config/crane.toml, runs the reference move (a trapezoidal trolley move, driven open loop by torque steps:
speed up, cruise, brake, then let go) and writes scipy_trace.csv next to this file: the load's y and its sway, the
horizontal offset from under the trolley's sheave. The Rust test `a_trapezoidal_trolley_move_sways_as_scipy_says`
compares against it.

Run from the repository root, with the pinned packages of requirements.txt in a venv:

    python -m venv .venv
    .venv/Scripts/python -m pip install -r crates/machine-overhead-crane/reference/requirements.txt
    .venv/Scripts/python crates/machine-overhead-crane/reference/scipy_reference.py
"""

import math
import tomllib
from pathlib import Path

import numpy as np
from scipy.integrate import solve_ivp

HERE = Path(__file__).parent
cfg = tomllib.loads((HERE.parent / "config" / "crane.toml").read_text(encoding="utf-8"))

# The effects this script models: they have to be on.
MODELLED = {"gravity", "payload", "rope_damping", "guide_friction", "motor_friction", "gearbox_efficiency"}
# Effects of the machine it does not model: they have to be off in the config, or the Rust test switches them off.
OFF_IN_THE_RUST_TEST = {"torque_ripple"}
NOT_MODELLED = {"gearbox_drag", "motor_stiction", "thermal_feedback", "base_compliance"}
# The hard stops and the holding brakes act only beyond the travel and with the drives disabled; neither happens here.
# Every other switch belongs to the drives and the controller, which take no part in a move driven by given torques.
effects = cfg["effects"]
assert MODELLED | NOT_MODELLED | OFF_IN_THE_RUST_TEST <= effects.keys(), "an effect this reference names is gone"
assert all(effects[name] for name in MODELLED), "an effect this reference implements is switched off"
assert not any(effects[name] for name in NOT_MODELLED), "an effect this reference does not implement is switched on"

G = 9.80665
crane, hoist, rope = cfg["crane"], cfg["hoist"], cfg["rope"]
H = crane["rope_exit_height_m"]
EA, C_ROPE = rope["ea_n"], rope["specific_damping_n_s"]
M_BRIDGE, M_TROLLEY = crane["bridge_mass_kg"], crane["trolley_mass_kg"]
M_LOAD = crane["hook_mass_kg"] + crane["payload_kg"]
W_ROLL, V_ROLL = crane["rolling_coefficient"], crane["rolling_smoothing_m_s"]
FADE_W = 0.5  # the gearbox's efficiency fades in over this power (mech::EFFICIENCY_FADE_W)


def own(table, motor):
    """A motor's own table over the shared one (catalog::motor_config)."""
    merged = {k: v for k, v in cfg[table].items() if not isinstance(v, dict)}
    merged.update(cfg[table].get(motor, {}))
    return merged


DRIVES = []
for motor, lever in (("bridge", crane["bridge_wheel_radius_m"]), ("trolley", crane["trolley_wheel_radius_m"]),
                     ("hoist", hoist["drum_radius_m"])):
    mot, gear = own("motor", motor), own("gearbox", motor)
    DRIVES.append({
        "ratio": gear["ratio"] / lever,  # rad of the motor per m at the axis
        "rotor": mot["rotor_inertia_kgm2"] + gear["inertia_at_input_kgm2"],
        "coulomb": mot["friction_coulomb_nm"], "viscous": mot["friction_viscous_nm_s_rad"],
        "smoothing": mot["friction_smoothing_rad_s"], "efficiency": gear["efficiency"],
    })
MASSES = (M_BRIDGE + M_TROLLEY, M_TROLLEY, hoist["drum_inertia_kgm2"] / hoist["drum_radius_m"] ** 2)


def rope_forces(y):
    """Tension on the ends, pull on the drum, unit vector sheave -> load."""
    x, yt, l = y[0], y[1], y[2]
    d = np.array([y[6] - x, y[7] - yt, y[8] - H])
    length = np.linalg.norm(d)
    u = d / length
    rate = u @ np.array([y[9] - y[3], y[10] - y[4], y[11]])
    e = length - l
    if e <= 0.0:
        return 0.0, 0.0, u
    spring = EA * e / l
    tension = max(spring + C_ROPE / l * (rate - y[5]), 0.0)
    return tension, spring * (1 + e / (2 * l)) + tension - spring, u


def rhs(t, y, torque):
    tension, drum, u = rope_forces(y)
    down = tension * -u[2]
    wheels = ((M_BRIDGE + M_TROLLEY) * G + down, M_TROLLEY * G + down)
    pull = (tension * u[0], tension * u[1], drum)
    acc = []
    for k, d in enumerate(DRIVES):
        v, i = y[3 + k], d["ratio"]
        w = i * v
        shaft = d["coulomb"] * math.tanh(w / d["smoothing"]) + d["viscous"] * w
        drive = i * (torque[k] - shaft)
        rolling = W_ROLL * wheels[k] * math.tanh(v / V_ROLL) if k < 2 else 0.0
        force = pull[k] - rolling
        reflected = i * i * d["rotor"]
        first = (drive + force) / (MASSES[k] + reflected)
        loss = d["efficiency"] ** math.tanh((drive - reflected * first) * v / FADE_W)
        acc.append((loss * drive + force) / (MASSES[k] + loss * reflected))
    load = [-tension * u[k] / M_LOAD for k in range(3)]
    load[2] -= G
    return [y[3], y[4], y[5], *acc, y[9], y[10], y[11], *load]


START = (5.0, 1.0, 2.0)  # the load's pose: a 3 m rope
# The trolley's torque steps over its holding torque (zero), N m at its motor: speed up, cruise against the rolling
# resistance and the motor's friction, brake, let go. The Rust test has the same table.
STEPS = ((0.0, 0.40), (1.5, 0.05), (3.5, -0.30), (5.0, 0.0))
END, SAMPLE = 10.0, 0.05

# At rest: the drum where the motors stand for the pose, counting on the empty hook's stretch; the load under the sheave
# on that rope stretched by its own weight; the hoist holds the drum against the rope.
l0 = (H - START[2]) / (1 + crane["hook_mass_kg"] * G / EA)
y0 = [START[0], START[1], l0, 0, 0, 0, START[0], START[1], H - l0 * (1 + M_LOAD * G / EA), 0, 0, 0]
_, drum0, _ = rope_forces(y0)
hold = [0.0, 0.0, -drum0 / DRIVES[2]["ratio"]]

times, rows, y = [], [], np.array(y0, dtype=float)
for n, (t0, step) in enumerate(STEPS):
    t1 = STEPS[n + 1][0] if n + 1 < len(STEPS) else END
    torque = [hold[0], hold[1] + step, hold[2]]
    first = round(t0 / SAMPLE)
    t_eval = [k * SAMPLE for k in range(first, round(t1 / SAMPLE) + 1)]
    sol = solve_ivp(rhs, (t0, t1), y, method="LSODA", t_eval=t_eval, args=(torque,), rtol=1e-11, atol=1e-13)
    assert sol.success, sol.message
    for k, t in enumerate(sol.t):
        if not times or t > times[-1] + 1e-9:
            times.append(t)
            rows.append(sol.y[:, k])
    y = sol.y[:, -1]

with open(HERE / "scipy_trace.csv", "w", encoding="utf-8", newline="\n") as f:
    f.write("# Written by scipy_reference.py from config/crane.toml. Regenerate when the model or the config changes.\n")
    f.write("t,load_y,sway_y\n")
    for t, r in zip(times, rows):
        f.write(f"{t:.3f},{r[7]:.12f},{r[7] - r[1]:.12f}\n")
print(f"{len(times)} samples, largest sway {max(abs(r[7] - r[1]) for r in rows):.6f} m")
