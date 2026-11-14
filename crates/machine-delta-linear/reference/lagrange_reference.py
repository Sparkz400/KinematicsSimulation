"""Independent reference for the linear delta's rigid, loss-free inverse dynamics: the model of
docs/model-reference.md ("Linear delta", lumped rods) written again as the D3's reference is
(crates/machine-delta-d3/reference/lagrange_reference.py): the carriages' travels and the plate's position are the
coordinates, SymPy differentiates the Lagrangian, the legs' constraints |P_i - C_i|^2 = L^2 enter with Lagrange
multipliers. The crate eliminates the plate through its Jacobian instead.

Writes inverse_dynamics.csv next to this file: motor torques at 30 random states in the working range of the default
config (columns upright) and 30 with the columns leaning 10 degrees inwards (seed 2026), read by the Rust test
`linear_delta_inverse_dynamics_matches_the_lagrangian`.

Effects: the Rust test runs the crate's `lossless_config` with `effects.belt_stretch = false` (the rigid model: no
losses, no guide friction, no base); gravity, the payload (attached) and the belt's mass (on the pulley) are on.
Every state keeps the carriages inside their stroke, so the stops take no part.

    .venv/Scripts/python -m pip install -r crates/machine-delta-d3/reference/requirements.txt
    .venv/Scripts/python crates/machine-delta-linear/reference/lagrange_reference.py
"""

import math
import tomllib
from pathlib import Path

import numpy as np
import sympy as sp

HERE = Path(__file__).parent
cfg = tomllib.loads((HERE.parent / "default.toml").read_text(encoding="utf-8"))
effects = cfg["effects"]
assert effects["gravity"] and effects["payload"] and effects["belt_mass"], "this reference has these on"
assert not effects["belt_tooth_stiffness"], "this reference has no teeth in the belt's length"
assert cfg["geometry"]["column_incline_deg"] == 0.0, "the first half of the states is for upright columns"

G = 9.80665
geo, mass, ws = cfg["geometry"], cfg["mass"], cfg["workspace"]
R, r, L = geo["column_radius_m"], geo["plate_radius_m"], geo["rod_m"]
radius, ratio = geo["pulley_radius_m"], cfg["gearbox"]["ratio"]
j_rotor = cfg["motor"]["rotor_inertia_kgm2"] + cfg["gearbox"]["inertia_at_input_kgm2"]
# The whole belt runs at carriage speed: sections span + stroke - x and 3 span + stroke + x.
j_pulley = cfg["pulley"]["inertia_kgm2"] + cfg["belt"]["mass_kg_m"] * (4 * geo["span_min_m"] + 2 * geo["stroke_m"]) * radius**2
m_c = mass["carriage_kg"] + mass["rod_carriage_share"] * mass["rod_pair_kg"]
m_nt = mass["plate_kg"] + mass["payload_kg"] + 3 * (1 - mass["rod_carriage_share"]) * mass["rod_pair_kg"]
PHI = [0.0, 2 * math.pi / 3, 4 * math.pi / 3]

h, hd = sp.symbols("h0:3"), sp.symbols("hd0:3")
pp, ppd = sp.symbols("x y z"), sp.symbols("xd yd zd")
q, qd = sp.Matrix([*h, *pp]), sp.Matrix([*hd, *ppd])
alpha = sp.Symbol("alpha")


def column(i, a):
    """Column i's direction: upright, or leaning by a with its top towards the axis."""
    c, s = math.cos(PHI[i]), math.sin(PHI[i])
    return [-sp.sin(a) * c, -sp.sin(a) * s, sp.cos(a)]


# Carriage i's joint C_i = (R cos phi, R sin phi, 0) + h_i d_i, the plate's P_i = p + r (cos phi, sin phi, 0).
joints = [sp.Matrix([R * math.cos(f), R * math.sin(f), 0]) + h[i] * sp.Matrix(column(i, alpha)) for i, f in enumerate(PHI)]
plates = [sp.Matrix(pp) + sp.Matrix([r * math.cos(f), r * math.sin(f), 0]) for f in PHI]
T = m_nt * sum(v**2 for v in ppd) / 2
V = m_nt * G * pp[2]
for i, c in enumerate(joints):
    vc = c.jacobian(q) * qd
    T += m_c * vc.dot(vc) / 2 + (j_pulley + ratio**2 * j_rotor) / radius**2 * hd[i] ** 2 / 2
    V += m_c * G * c[2]
T = sp.Matrix([T])
t_qd = T.jacobian(qd).T
phi_q = sp.Matrix([(p - c).dot(p - c) - L**2 for c, p in zip(joints, plates)]).jacobian(q)
f = lambda e: sp.lambdify((*q, *qd, alpha), e, "numpy", cse=True)
f_m, f_rest = f(t_qd.jacobian(qd)), f(t_qd.jacobian(q) * qd - T.jacobian(q).T + sp.Matrix([V]).jacobian(q).T)
f_phiq, f_conv = f(phi_q), f((phi_q * qd).jacobian(q) * qd)


def travel(i, pos, a):
    """Carriage i's travel along its column: the larger root of |q - h d|^2 = L^2 (carriage above the plate)."""
    d = np.array([float(v) for v in column(i, a)])
    qv = pos + (r - R) * np.array([math.cos(PHI[i]), math.sin(PHI[i]), 0.0])
    roots = np.roots([1.0, -2 * qv @ d, qv @ qv - L**2])
    assert np.isreal(roots).all(), "out of reach"
    return max(roots.real), np.linalg.norm(qv - (qv @ d) * d)


def motor_torques(pos, vel, acc, a):
    hv = np.array([travel(i, pos, a)[0] for i in range(3)])
    qv = np.concatenate([hv, pos])
    call = lambda fn, qdv: np.array(fn(*qv, *qdv, a), dtype=float).squeeze()
    jq = call(f_phiq, np.zeros(6))
    a_h, a_p = np.diag(jq[:, :3]), jq[:, 3:]
    qdv = np.concatenate([-a_p @ vel / a_h, vel])
    qdd = np.concatenate([-(a_p @ acc + call(f_conv, qdv)) / a_h, acc])
    lhs = call(f_m, qdv) @ qdd + call(f_rest, qdv)
    lam = np.linalg.solve(a_p.T, lhs[3:])
    return (lhs[:3] - jq[:, :3].T @ lam) * radius / ratio


def admissible(pos, a):
    """In the cylinder of [workspace], the rods within their angle, the carriages within their stroke."""
    if math.hypot(pos[0], pos[1]) > ws["radius_m"] or not ws["bottom_z_m"] <= pos[2] <= ws["top_z_m"]:
        return False
    top = geo["carriage_top_m"]
    for i in range(3):
        hv, rho = travel(i, pos, a)
        if rho > L * math.sin(math.radians(ws["max_rod_angle_deg"])) or not top - geo["stroke_m"] <= hv <= top:
            return False
    return True


def main():
    rng = np.random.default_rng(2026)
    ax = cfg["axis"]
    v_max = np.array([ax[k]["max_velocity_m_s"] for k in "xyz"])
    a_max = np.array([ax[k]["max_acceleration_m_s2"] for k in "xyz"])
    rows = []
    for incline in (0.0, 10.0):
        a, n = math.radians(incline), 0
        while n < 30:
            pos = rng.uniform([-ws["radius_m"], -ws["radius_m"], ws["bottom_z_m"]], [ws["radius_m"], ws["radius_m"], ws["top_z_m"]])
            vel, acc = rng.uniform(-v_max, v_max), rng.uniform(-a_max, a_max)
            if admissible(pos, a):
                rows.append((incline, *pos, *vel, *acc, *motor_torques(pos, vel, acc, a)))
                n += 1
    with open(HERE / "inverse_dynamics.csv", "w", encoding="utf-8", newline="\n") as out:
        out.write("# Written by lagrange_reference.py from default.toml. Regenerate when the model or the config changes.\n")
        out.write("incline_deg,x,y,z,vx,vy,vz,ax,ay,az,tau_1,tau_2,tau_3\n")
        for row in rows:
            out.write(",".join(repr(float(v)) for v in row) + "\n")
    print(f"inverse dynamics: {len(rows)} states, largest motor torque {max(abs(v) for row in rows for v in row[10:]):.3f} N m")


if __name__ == "__main__":
    main()
