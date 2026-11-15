"""Independent reference for the D2's rigid, loss-free inverse dynamics: the model of docs/model-reference.md
("Delta picker D2": the D3's lumped dynamics with two legs in the x-z plane) written again as the D3's reference is
(the script of the same name beside the three-arm delta's crate): the arm angles and the plate's position (x, z) are the
coordinates, SymPy differentiates the Lagrangian, the legs' constraints |P_i - E_i|^2 = L_b^2 enter with Lagrange
multipliers. The crate eliminates the plate through its Jacobian instead.

Writes inverse_dynamics.csv next to this file: motor torques at 60 random poses, speeds and accelerations in the
working range (seed 2026), read by the Rust test `d2_inverse_dynamics_matches_the_lagrangian`.

Effects: the Rust test runs the crate's `lossless_config` with `effects.joint_elasticity = false` (no losses, rigid
gearboxes, no base); gravity and the payload (attached) are on. Every state keeps the arms inside their stops.

    .venv/Scripts/python -m pip install -r crates/machine-delta-d2/reference/requirements.txt
    .venv/Scripts/python crates/machine-delta-d2/reference/lagrange_reference.py
"""

import math
import tomllib
from pathlib import Path

import numpy as np
import sympy as sp
from scipy.optimize import brentq

HERE = Path(__file__).parent
cfg = tomllib.loads((HERE.parent / "default.toml").read_text(encoding="utf-8"))
effects = cfg["effects"]
assert effects["gravity"] and effects["payload"], "this reference has gravity and the payload on"
assert "forearm_inertia" not in effects, "the D2 has forearms as rods now: this reference lumps them"

G = 9.80665
geo, mass, joint = cfg["geometry"], cfg["mass"], cfg["joint"]
R, r, La, Lb = geo["base_radius_m"], geo["platform_radius_m"], geo["upper_arm_m"], geo["forearm_m"]
ratio = cfg["gearbox"]["ratio"]
j_rotor = cfg["motor"]["rotor_inertia_kgm2"] + cfg["gearbox"]["inertia_at_input_kgm2"]
m_fb, share = mass["forearm_pair_kg"], mass["forearm_elbow_share"]
SIDE = [1.0, -1.0]
STOPS = (math.radians(joint["min_angle_deg"]), math.radians(joint["max_angle_deg"]))

th, thd = sp.symbols("th0:2"), sp.symbols("thd0:2")
x, z, xd, zd = sp.symbols("x z xd zd")
q, qd = sp.Matrix([*th, x, z]), sp.Matrix([*thd, xd, zd])
rate = lambda e: e.jacobian(q) * qd

# Arm i turns about the axis along y at x = s_i R; its elbow E_i = (s_i (R + L_a cos th), -L_a sin th) (th down from
# horizontal); the forearm pair joins it to P_i = (x + s_i r, z) on the plate, which keeps its orientation.
legs = [(sp.Matrix([s * (R + La * sp.cos(th[i])), -La * sp.sin(th[i])]), sp.Matrix([x + s * r, z])) for i, s in enumerate(SIDE)]

T = (mass["platform_kg"] + mass["payload_kg"] + 2 * (1 - share) * m_fb) * (xd**2 + zd**2) / 2
V = (mass["platform_kg"] + mass["payload_kg"] + 2 * (1 - share) * m_fb) * G * z
for i, (elbow, _) in enumerate(legs):
    ve = rate(elbow)
    T += (mass["upper_arm_inertia_kgm2"] + j_rotor * ratio**2) * thd[i] ** 2 / 2 + (mass["elbow_kg"] + share * m_fb) * ve.dot(ve) / 2
    V += -G * mass["upper_arm_kg"] * mass["upper_arm_cg_m"] * sp.sin(th[i]) + (mass["elbow_kg"] + share * m_fb) * G * elbow[1]

T = sp.Matrix([T])
t_qd = T.jacobian(qd).T
phi_q = sp.Matrix([(p - e).dot(p - e) - Lb**2 for e, p in legs]).jacobian(q)
f = lambda e: sp.lambdify((*q, *qd), e, "numpy", cse=True)
f_m, f_rest = f(t_qd.jacobian(qd)), f(t_qd.jacobian(q) * qd - T.jacobian(q).T + sp.Matrix([V]).jacobian(q).T)
f_phiq, f_conv = f(phi_q), f((phi_q * qd).jacobian(q) * qd)
call = lambda fn, qv, qdv: np.array(fn(*qv, *qdv), dtype=float).squeeze()


def leg_angle(i, pos):
    """Arm i's angle: the root of its leg's constraint with the elbow further out."""
    xp, zp = SIDE[i] * pos[0] + r - R, pos[1]
    g = lambda t: (xp - La * math.cos(t)) ** 2 + (zp + La * math.sin(t)) ** 2 - Lb**2
    grid = np.linspace(-math.pi, math.pi, 721)
    roots = [brentq(g, a, b, xtol=1e-16, rtol=1e-15) for a, b in zip(grid[:-1], grid[1:]) if g(a) * g(b) < 0]
    assert len(roots) == 2, f"pose {pos} out of reach"
    return max(roots, key=math.cos)


def motor_torques(pos, vel, acc):
    theta = np.array([leg_angle(i, pos) for i in range(2)])
    assert all(STOPS[0] < t < STOPS[1] for t in theta), "an arm beyond its stop"
    qv = np.concatenate([theta, pos])
    jq = call(f_phiq, qv, np.zeros(4))
    a_th, a_p = np.diag(jq[:, :2]), jq[:, 2:]
    qdv = np.concatenate([-a_p @ vel / a_th, vel])
    qdd = np.concatenate([-(a_p @ acc + call(f_conv, qv, qdv)) / a_th, acc])
    lhs = call(f_m, qv, qdv) @ qdd + call(f_rest, qv, qdv)
    lam = np.linalg.solve(a_p.T, lhs[2:])
    return (lhs[:2] - jq[:, :2].T @ lam) / ratio


def in_range(pos):
    """The band |x| <= half_width below the top, its bottom an arc through the edges and the centre's depth."""
    ws = cfg["workspace"]
    sag = ws["height_centre_m"] - ws["height_edge_m"]
    radius = (ws["half_width_m"] ** 2 + sag**2) / (2 * sag)
    centre = ws["top_z_m"] - ws["height_centre_m"] + radius
    return abs(pos[0]) <= ws["half_width_m"] and pos[1] <= ws["top_z_m"] and pos[0] ** 2 + (pos[1] - centre) ** 2 <= radius**2


def main():
    rng = np.random.default_rng(2026)
    ax = cfg["axis"]
    v_max = np.array([ax[a]["max_velocity_m_s"] for a in "xz"])
    a_max = np.array([ax[a]["max_acceleration_m_s2"] for a in "xz"])
    ws = cfg["workspace"]
    low, high = [-ws["half_width_m"], ws["top_z_m"] - ws["height_centre_m"]], [ws["half_width_m"], ws["top_z_m"]]
    rows = []
    while len(rows) < 60:
        pos, vel, acc = rng.uniform(low, high), rng.uniform(-v_max, v_max), rng.uniform(-a_max, a_max)
        if in_range(pos):
            rows.append((*pos, *vel, *acc, *motor_torques(pos, vel, acc)))
    with open(HERE / "inverse_dynamics.csv", "w", encoding="utf-8", newline="\n") as out:
        out.write("# Written by lagrange_reference.py from default.toml. Regenerate when the model or the config changes.\n")
        out.write("x,z,vx,vz,ax,az,tau_1,tau_2\n")
        for row in rows:
            out.write(",".join(repr(float(v)) for v in row) + "\n")
    print(f"inverse dynamics: {len(rows)} states, largest motor torque {max(abs(v) for row in rows for v in row[6:]):.3f} N m")


if __name__ == "__main__":
    main()
