"""Independent reference for the D3's rigid, loss-free dynamics: the model of docs/model-reference.md ("Delta picker D3",
lumped forearms and forearms as rods), written again by another route than the crate's.

The crate eliminates the platform through the Jacobian (M = I_b + m_nt J^T J, the velocity term through J_x^-1 w).
Here the arm angles AND the platform's position are the coordinates, q = (theta_1..3, x, y, z), every body's kinetic
and potential energy is written out in them, SymPy differentiates the Lagrangian, and the three legs' constraints
|P_i - E_i|^2 = L_b^2 enter with Lagrange multipliers:

    M(q) q'' + h(q, q') + dV/dq = Q + Phi_q^T lambda,     Phi_q q'' + (d/dq (Phi_q q')) q' = 0

The inverse kinematics is a root search on each leg's constraint (not the half-angle formula), the forward
kinematics a Newton iteration on all three.

Writes, next to this file:
  inverse_dynamics.csv  motor torques at 60 random poses, speeds and accelerations in the working range (seed 2026),
                        lumped forearms and forearms as rods (`delta_inverse_dynamics_matches_the_lagrangian`)
  open_loop_move.csv    the platform under given motor torques (hold, a push, a pause, a pull back) over 0.5 s, the
                        shipped model (rods), integrated by DOP853 at rtol 1e-10 (`open_loop_move_matches_scipy`)
  open_loop_torques.csv the holding and push torques of that move, which the Rust test drives its model with

Run from the repository root, with the pinned packages of requirements.txt in a venv:

    python -m venv .venv
    .venv/Scripts/python -m pip install -r crates/machine-delta-d3/reference/requirements.txt
    .venv/Scripts/python crates/machine-delta-d3/reference/lagrange_reference.py
"""

import math
import tomllib
from pathlib import Path

import numpy as np
import sympy as sp
from scipy.integrate import solve_ivp
from scipy.optimize import brentq

HERE = Path(__file__).parent
cfg = tomllib.loads((HERE.parent / "default.toml").read_text(encoding="utf-8"))

# The effects this reference models: gravity, the payload (attached), the forearms lumped or as rods (both written).
MODELLED = {"gravity", "payload", "forearm_inertia"}
# Everything else the Rust tests switch off (the crate's `lossless_config`, and the rigid model): losses, the
# gearboxes' and the arms' elasticity, the base. The hard stops stay on; they act only beyond the stop angles, which
# every pose here keeps inside (asserted). The drives and the controller take no part in given torques.
OFF_IN_THE_RUST_TEST = {
    "joint_elasticity", "link_elasticity", "arm_tube_inertia", "base_compliance", "joint_friction", "motor_friction",
    "gearbox_drag", "gearbox_lost_motion", "gearbox_efficiency", "torque_ripple", "holding_brake",
}
effects = cfg["effects"]
assert MODELLED | OFF_IN_THE_RUST_TEST <= effects.keys(), "an effect this reference names is gone from the config"
assert effects["gravity"] and effects["payload"], "this reference has gravity and the payload on"

G = 9.80665
geo, mass, joint = cfg["geometry"], cfg["mass"], cfg["joint"]
R, r, La, Lb = geo["base_radius_m"], geo["platform_radius_m"], geo["upper_arm_m"], geo["forearm_m"]
ratio = cfg["gearbox"]["ratio"]
j_rotor = cfg["motor"]["rotor_inertia_kgm2"] + cfg["gearbox"]["inertia_at_input_kgm2"]
m_fb = mass["forearm_pair_kg"]
m_plate = mass["platform_kg"] + mass["payload_kg"]
PHI = [0.0, 2 * math.pi / 3, 4 * math.pi / 3]
STOPS = (math.radians(joint["min_angle_deg"]), math.radians(joint["max_angle_deg"]))


def rz(phi):
    c, s = sp.cos(phi), sp.sin(phi)
    return sp.Matrix([[c, -s, 0], [s, c, 0], [0, 0, 1]])


th = sp.symbols("th0:3")
pp = sp.symbols("x y z")
thd = sp.symbols("thd0:3")
ppd = sp.symbols("xd yd zd")
q, qd = sp.Matrix([*th, *pp]), sp.Matrix([*thd, *ppd])
p, pd = sp.Matrix(pp), sp.Matrix(ppd)


def rate(expr):
    """Time derivative of an expression in q along q'."""
    return expr.jacobian(q) * qd


# The bodies, in the base frame (z up, theta down from horizontal). Arm i turns about the axis through
# Rz(phi_i) (R, 0, 0) along a_i = Rz(phi_i) (0, 1, 0); its elbow E_i is L_a out along the arm; the pair of forearms
# joins it to P_i = p + Rz(phi_i) (r, 0, 0) on the platform, which keeps its orientation.
legs = []
for i in range(3):
    turn = rz(PHI[i])
    elbow = turn * sp.Matrix([R + La * sp.cos(th[i]), 0, -La * sp.sin(th[i])])
    plate = p + turn * sp.Matrix([r, 0, 0])
    axis = turn * sp.Matrix([0, 1, 0])
    legs.append((elbow, plate, axis))


def lagrangian(rods):
    """Kinetic energy T(q, q') and potential energy V(q) of the whole delta."""
    T = sp.Integer(0)
    V = sp.Integer(0)
    share = 1 / sp.Integer(6) if rods else mass["forearm_elbow_share"]
    plate_mass = m_plate + (0 if rods else 3 * (1 - share) * m_fb)
    T += plate_mass * pd.dot(pd) / 2
    # The weight, unlike the inertia, splits each forearm a half to the elbow and a half to the plate (a rod's mass is at
    # its middle), whatever the inertia share.
    V += (m_plate + (0 if rods else 3 * m_fb / 2)) * G * p[2]
    for i, (elbow, plate, axis) in enumerate(legs):
        # The upper arm about its joint (its centre of mass upper_arm_cg_m out), the rotor through the gearbox.
        T += (mass["upper_arm_inertia_kgm2"] + j_rotor * ratio**2) * thd[i] ** 2 / 2
        V += -G * mass["upper_arm_kg"] * mass["upper_arm_cg_m"] * sp.sin(th[i])
        ve = rate(elbow)
        if rods:
            # Elbow block alone; each forearm a uniform rod from E to P: integral of (m/L) |E'(1-s) + P' s|^2 / 2 ds
            # over the rod = (m/6) (|E'|^2 + E'.P' + |P'|^2); its centre is the middle.
            T += mass["elbow_kg"] * ve.dot(ve) / 2
            V += mass["elbow_kg"] * G * elbow[2]
            vp = rate(plate)
            T += m_fb / 6 * (ve.dot(ve) + ve.dot(vp) + vp.dot(vp))
            V += m_fb * G * (elbow[2] + plate[2]) / 2
            # Each rod's spin about its own axis e: the pair's plane holds the elbow's axis a and e, so the rod's
            # frame is (e, n, e x n) with n = a x e / |a x e|, and the spin is omega . e = n' . (e x n).
            e = (plate - elbow) / Lb
            n = axis.cross(e)
            n = n / sp.sqrt(n.dot(n))
            spin = rate(n).dot(e.cross(n))
            T += 2 * mass["forearm_rod_spin_inertia_kgm2"] * spin**2 / 2
        else:
            # Codourey/Olsson: the elbow block and the share s of the pair at the elbow, the rest at the platform.
            m_e = mass["elbow_kg"] + share * m_fb
            T += m_e * ve.dot(ve) / 2
            V += (mass["elbow_kg"] + m_fb / 2) * G * elbow[2]
    return T, V


def equations(rods):
    """Numeric functions for M(q), h(q, q') + dV/dq, Phi_q(q) and the constraints' convective term."""
    T, V = lagrangian(rods)
    T = sp.Matrix([T])
    t_qd = T.jacobian(qd).T
    mass_matrix = t_qd.jacobian(qd)
    rest = t_qd.jacobian(q) * qd - T.jacobian(q).T + sp.Matrix([V]).jacobian(q).T
    phi = sp.Matrix([(plate - elbow).dot(plate - elbow) - Lb**2 for elbow, plate, _ in legs])
    phi_q = phi.jacobian(q)
    convective = (phi_q * qd).jacobian(q) * qd
    args = (*q, *qd)
    lam = lambda e: sp.lambdify(args, e, "numpy", cse=True)
    f_m, f_rest, f_phi, f_phiq, f_conv = (lam(e) for e in (mass_matrix, rest, phi, phi_q, convective))

    def call(f, qv, qdv):
        return np.array(f(*qv, *qdv), dtype=float).squeeze()

    return (
        lambda qv: call(f_m, qv, np.zeros(6)),
        lambda qv, qdv: call(f_rest, qv, qdv),
        lambda qv: call(f_phi, qv, np.zeros(6)),
        lambda qv: call(f_phiq, qv, np.zeros(6)),
        lambda qv, qdv: call(f_conv, qv, qdv),
    )


def leg_angle(i, pos):
    """Arm i's angle at the platform's position: the root of its leg's constraint with the elbow further out."""
    c, s = math.cos(PHI[i]), math.sin(PHI[i])
    x, y, z = c * pos[0] + s * pos[1] + r - R, -s * pos[0] + c * pos[1], pos[2]
    f = lambda t: (x - La * math.cos(t)) ** 2 + y**2 + (z + La * math.sin(t)) ** 2 - Lb**2
    grid = np.linspace(-math.pi, math.pi, 721)
    roots = [brentq(f, a, b, xtol=1e-16, rtol=1e-15) for a, b in zip(grid[:-1], grid[1:]) if f(a) * f(b) < 0]
    assert len(roots) == 2, f"pose {pos} out of reach"
    return max(roots, key=math.cos)


def kinematics(phi_q, conv, pos, vel, acc):
    """q, q', q'' from the platform's position, speed and acceleration."""
    theta = np.array([leg_angle(i, pos) for i in range(3)])
    assert all(STOPS[0] < t < STOPS[1] for t in theta), "an arm beyond its stop"
    qv = np.concatenate([theta, pos])
    jq = phi_q(qv)
    a_th, a_p = np.diag(jq[:, :3]), jq[:, 3:]
    qdv = np.concatenate([-a_p @ vel / a_th, vel])
    qdd = np.concatenate([-(a_p @ acc + conv(qv, qdv)) / a_th, acc])
    return qv, qdv, qdd


def motor_torques(model, pos, vel, acc):
    """Inverse dynamics: the platform's equations give the multipliers, the arms' the torques."""
    m, rest, _, phi_q, conv = model
    qv, qdv, qdd = kinematics(phi_q, conv, pos, vel, acc)
    lhs = m(qv) @ qdd + rest(qv, qdv)
    jq = phi_q(qv)
    lam = np.linalg.solve(jq[:, 3:].T, lhs[3:])
    return (lhs[:3] - jq[:, :3].T @ lam) / ratio


def forward(phi, phi_q, theta, guess):
    """The platform's position at the arm angles: Newton on the three constraints from a guess below."""
    pos = np.array(guess, dtype=float)
    for _ in range(50):
        qv = np.concatenate([theta, pos])
        step = np.linalg.solve(phi_q(qv)[:, 3:], phi(qv))
        pos -= step
        if np.abs(step).max() < 1e-15:
            return pos
    raise RuntimeError("forward kinematics did not converge")


def in_range(pos):
    """The working range of [workspace]: a cylinder below the top, its bottom a sphere."""
    ws = cfg["workspace"]
    sag = ws["height_centre_m"] - ws["height_edge_m"]
    sphere = (ws["radius_m"] ** 2 + sag**2) / (2 * sag)
    rho = math.hypot(pos[0], pos[1])
    centre = ws["top_z_m"] - ws["height_centre_m"] + sphere
    return rho <= ws["radius_m"] and pos[2] <= ws["top_z_m"] and (pos[0] ** 2 + pos[1] ** 2 + (pos[2] - centre) ** 2) <= sphere**2


def write_inverse_dynamics(models):
    rng = np.random.default_rng(2026)
    ax = cfg["axis"]
    v_max = np.array([ax[a]["max_velocity_m_s"] for a in "xyz"])
    a_max = np.array([ax[a]["max_acceleration_m_s2"] for a in "xyz"])
    rows = []
    while len(rows) < 60:
        pos = rng.uniform([-0.6, -0.6, -1.07], [0.6, 0.6, -0.675])
        vel, acc = rng.uniform(-v_max, v_max), rng.uniform(-a_max, a_max)
        if not in_range(pos):
            continue
        rows.append((pos, vel, acc, [motor_torques(m, pos, vel, acc) for m in models]))
    with open(HERE / "inverse_dynamics.csv", "w", encoding="utf-8", newline="\n") as f:
        f.write("# Written by lagrange_reference.py from default.toml. Regenerate when the model or the config changes.\n")
        f.write("x,y,z,vx,vy,vz,ax,ay,az,lumped_1,lumped_2,lumped_3,rods_1,rods_2,rods_3\n")
        for pos, vel, acc, (lumped, rods) in rows:
            f.write(",".join(repr(float(v)) for v in (*pos, *vel, *acc, *lumped, *rods)) + "\n")
    peak = max(np.abs(t).max() for *_, ts in rows for t in ts)
    print(f"inverse dynamics: {len(rows)} states, largest motor torque {peak:.3f} N m")


def pulse(t):
    bump = lambda t0: math.sin(math.pi * (t - t0) / 0.15) ** 2 if t0 < t < t0 + 0.15 else 0.0
    return bump(0.0) - bump(0.25)


def write_open_loop_move(model, start, push):
    """Hold at `start`, then the torques that accelerate the platform at `push` there, as a pulse there and back."""
    m, rest, phi, phi_q, conv = model
    zero = np.zeros(3)
    hold = motor_torques(model, start, zero, zero)
    delta = motor_torques(model, start, zero, push) - hold
    guess = [np.array(start, dtype=float)]

    def rhs(t, y):
        theta, w = y[:3], y[3:]
        pos = forward(phi, phi_q, theta, guess[0])
        guess[0] = pos
        qv = np.concatenate([theta, pos])
        jq = phi_q(qv)
        pv = -np.linalg.solve(jq[:, 3:], jq[:, :3] @ w)
        qdv = np.concatenate([w, pv])
        torque = hold + delta * pulse(t)
        force = np.concatenate([ratio * torque, zero]) - rest(qv, qdv)
        # [M, -Phi_q^T; Phi_q, 0] [q''; lambda] = [Q - h - dV/dq; -convective]
        a = np.block([[m(qv), -jq.T], [jq, np.zeros((3, 3))]])
        sol = np.linalg.solve(a, np.concatenate([force, -conv(qv, qdv)]))
        return np.concatenate([w, sol[:3]])

    theta0 = np.array([leg_angle(i, start) for i in range(3)])
    times = np.round(np.arange(501) * 1e-3, 9)
    sol = solve_ivp(rhs, (0.0, 0.5), np.concatenate([theta0, zero]), method="DOP853", t_eval=times,
                    rtol=1e-10, atol=1e-13, max_step=1e-3)
    assert sol.success, sol.message
    poses = []
    for theta in sol.y[:3].T:
        assert all(STOPS[0] < t < STOPS[1] for t in theta), "an arm beyond its stop"
        poses.append(forward(phi, phi_q, theta, poses[-1] if poses else start))
    poses = np.array(poses)
    assert all(in_range(pos) for pos in poses), "the move leaves the working range"
    with open(HERE / "open_loop_move.csv", "w", encoding="utf-8", newline="\n") as f:
        f.write("# Written by lagrange_reference.py from default.toml. Regenerate when the model or the config changes.\n")
        f.write(f"# start {start.tolist()}, push {push.tolist()} m/s^2, pulse sin^2 over 0.15 s at 0 and back at 0.25 s\n")
        f.write("t,x,y,z\n")
        for t, pos in zip(sol.t, poses):
            f.write(f"{t:.3f},{pos[0]:.12f},{pos[1]:.12f},{pos[2]:.12f}\n")
    # The torques the move is driven by, for the Rust test to drive its model with (its own would follow its config).
    with open(HERE / "open_loop_torques.csv", "w", encoding="utf-8", newline="\n") as f:
        f.write("# Written by lagrange_reference.py: motor torques of open_loop_move.csv, hold + push * pulse(t).\n")
        f.write("hold_1,hold_2,hold_3,push_1,push_2,push_3\n")
        f.write(",".join(repr(float(v)) for v in (*hold, *delta)) + "\n")
    travel = np.abs(poses - poses[0]).max(axis=0)
    print(f"open loop: travel {travel} m, {sol.nfev} evaluations")


def main():
    lumped, rods = equations(False), equations(True)
    write_inverse_dynamics((lumped, rods))
    write_open_loop_move(rods, np.array([0.05, 0.02, -0.92]), np.array([5.0, 3.0, -4.0]))


if __name__ == "__main__":
    main()
