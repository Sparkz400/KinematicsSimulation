"""Independent reference for the elastic model: the same equations, written again in Python and
integrated by SciPy's LSODA instead of the simulator's fixed-step RK4.

Reads ../default.toml, runs the reference move (a push, a pause, a pull back) and writes
scipy_trace.csv next to this file. The Rust test `reference_move_matches_scipy` compares against it.

Run from the repository root, with the pinned packages of requirements.txt in a venv:

    python -m venv .venv
    .venv/Scripts/python -m pip install -r crates/machine-t-gantry/reference/requirements.txt
    .venv/Scripts/python crates/machine-t-gantry/reference/scipy_reference.py
"""

import math
import tomllib
from pathlib import Path

import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import brentq

HERE = Path(__file__).parent
cfg = tomllib.loads((HERE.parent / "default.toml").read_text(encoding="utf-8"))

# The effects of the machine that this script models: they have to be switched on.
MODELLED = {
    "gravity", "payload", "belt_stretch", "belt_damping", "belt_slack", "belt_mass", "guide_friction",
    "load_dependent_friction", "motor_friction", "gearbox_backlash", "gearbox_efficiency", "torque_ripple",
    "tooth_meshing", "frame_compliance",
}
# The effects of the machine that it does not model: they have to be switched off, or the Rust test has to
# switch them off for the comparison, as it does with the leaning Z axis.
NOT_MODELLED = {"belt_tooth_stiffness", "belt_material_damping", "idlers", "cable_chain", "process_force", "temperature"}
OFF_IN_THE_RUST_TEST = {"tool_flex"}
# Every other switch belongs to the drives and the controller, which take no part in a move driven by given
# torques, or to the hard stops and the holding brakes, which act only beyond the stroke and with the drives
# disabled.
effects = cfg["effects"]
assert MODELLED | NOT_MODELLED | OFF_IN_THE_RUST_TEST <= effects.keys(), "an effect this reference names is gone from the config"
assert all(effects[name] for name in MODELLED), "an effect this reference implements is switched off"
assert not any(effects[name] for name in NOT_MODELLED), "an effect this reference does not implement is switched on"

G = 9.80665
geo_, mass, belt, motor, gear, frame = (cfg[k] for k in ("geometry", "mass", "belt", "motor", "gearbox", "frame"))
r, sx, sz, span = geo_["pulley_radius_m"], geo_["stroke_x_m"], geo_["stroke_z_m"], geo_["span_min_m"]
EA, T0, c_belt = belt["stiffness_ea_n"], belt["pretension_n"], belt["damping_n_s_m"]
ratio, k_gear, c_gear = gear["ratio"], gear["stiffness_nm_rad"], gear["damping_nm_s_rad"]
half_backlash, eta = gear["backlash_rad"] / 2, gear["efficiency"]
m_z = mass["z_axis_kg"] + mass["payload_kg"]
m_x = mass["carriage_kg"] + m_z
m_x_nominal = mass["carriage_kg"] + mass["z_axis_kg"]
j_rotor = motor["rotor_inertia_kgm2"] + gear["inertia_at_input_kgm2"]


def lengths(x, z):
    return np.array([2 * span + x + z, 4 * span + sx + 2 * sz - 2 * z, 2 * span + sx - x + z])


rest0 = lengths(0, 0) / (1 + T0 / EA)
r_mat = r / (1 + T0 / EA)
j_pulley = cfg["pulley"]["inertia_kgm2"] + 0.5 * belt["mass_kg_m"] * lengths(0, 0).sum() * r * r
m_motors = 2 * (j_pulley + ratio**2 * j_rotor) / r**2


def material(a, b):
    return rest0 + r_mat * np.array([a, b - a, -b])


def section(length, rest, length_rate, rest_rate):
    """Tension on the ends, and force on belt fed in (tension times 1 + strain/2)."""
    e = length - rest
    elastic = EA * e / rest
    tension = max(elastic + c_belt * (length_rate - rest_rate), 0.0)
    return (tension, tension) if e < 0 else (tension, tension + elastic * e / (2 * rest))


def lugre(p, v, s, scale):
    level = scale * (p["coulomb_n"] + (p["stiction_n"] - p["coulomb_n"]) * math.exp(-((v / p["stribeck_m_s"]) ** 2)))
    s_dot = v - p["bristle_stiffness_n_m"] * abs(v) * s / level
    return p["bristle_stiffness_n_m"] * s + p["bristle_damping_n_s_m"] * s_dot + p["viscous_n_s_m"] * v, s_dot


def guide_friction(z, vx, vz, s_x, s_z, torque):
    ax = ratio * (torque[0] + torque[1]) / r / (m_x + m_motors)
    side = m_z * ax
    tilt = 2 * side * (geo_["z_com_drop_m"] - z) / geo_["guide_spacing_m"]
    fx, fz = cfg["friction_x"], cfg["friction_z"]
    scale_x = (fx["preload_n"] + m_x * G + abs(tilt)) / (fx["preload_n"] + m_x_nominal * G)
    scale_z = (fz["preload_n"] + abs(side) + abs(tilt)) / fz["preload_n"]
    return lugre(fx, vx, s_x, scale_x) + lugre(fz, vz, s_z, scale_z)


def gear_torque(phi, w_phi, theta, w_theta):
    twist = phi / ratio - theta
    twist -= min(max(twist, -half_backlash), half_backlash)
    if twist == 0.0:
        return 0.0, 0.0
    torque = k_gear * twist + c_gear * (w_phi / ratio - w_theta)
    return torque * eta ** math.tanh(torque * w_theta / 0.5), torque / ratio


def ripple(torque, angle):
    return (torque * (1 + motor["ripple_fraction"] * math.sin(motor["ripple_order"] * angle))
            + motor["cogging_nm"] * math.sin(motor["cogging_order"] * angle))


def mesh(travel):
    return belt["tooth_force_n"] * math.sin(2 * math.pi * travel / belt["tooth_pitch_m"])


def shaft_friction(w):
    return motor["friction_coulomb_nm"] * math.tanh(w / motor["friction_smoothing_rad_s"]) + motor["friction_viscous_nm_s_rad"] * w


# State: x, z, a, b, pa, pb, q, their rates, sx, sz.
def rhs(t, y, torque_at):
    x, z, a, b, pa, pb, q, vx, vz, va, vb, wa, wb, vq, s_x, s_z = y
    torque = torque_at(t)
    length, rest = lengths(x, z), material(a, b)
    length_rate = (vx + vz, -2 * vz, vz - vx)
    rest_rate = (r_mat * va, r_mat * (vb - va), -r_mat * vb)
    tension, feed = zip(*(section(length[i], rest[i], length_rate[i], rest_rate[i]) for i in range(3)))
    ffx, dsx, ffz, dsz = guide_friction(z, vx, vz, s_x, s_z, torque)
    fx = tension[2] - tension[0] - ffx
    aq = (-frame["stiffness_n_m"] * q - frame["damping_n_s_m"] * vq - fx) / frame["mass_kg"]
    ax = fx / m_x - aq
    az = (2 * tension[1] - tension[0] - tension[2] - ffz) / m_z - G
    belt_torque = (r_mat * (feed[0] - feed[1]), r_mat * (feed[1] - feed[2]))
    acc = []
    for k, (theta, w_theta, phi, w_phi) in enumerate(((a, va, pa, wa), (b, vb, pb, wb))):
        out, back = gear_torque(phi, w_phi, theta, w_theta)
        acc.append(((out + belt_torque[k] + r * mesh(r * theta)) / j_pulley,
                    (ripple(torque[k], phi) - back - shaft_friction(w_phi)) / j_rotor))
    return [vx, vz, va, vb, wa, wb, vq, ax, az, acc[0][0], acc[1][0], acc[0][1], acc[1][1], aq, dsx, dsz]


def equilibrium(x, z):
    """State at rest and holding torques. X balance: T1 = T3; Z balance: 2 T2 - T1 - T3 = m_z g."""
    length = lengths(x, z)
    offsets = np.array([0.0, m_z * G / 2, 0.0])
    rest_at = lambda base: length / (1 + np.maximum(base + offsets, 0.0) / EA)
    base = brentq(lambda t: rest_at(t).sum() - rest0.sum(), -0.5 * EA, EA, xtol=1e-12, rtol=1e-15)
    rest = rest_at(base)
    a, b = (rest[0] - rest0[0]) / r_mat, (rest0[2] - rest[2]) / r_mat
    feed = [section(length[i], rest[i], 0.0, 0.0)[1] for i in range(3)]
    y, hold = [x, z, a, b, 0.0, 0.0] + [0.0] * 10, []
    for k, (theta, belt_torque) in enumerate(((a, r_mat * (feed[0] - feed[1])), (b, r_mat * (feed[1] - feed[2])))):
        through = -belt_torque - r * mesh(r * theta)
        phi = ratio * (theta + through / k_gear + math.copysign(half_backlash, through))
        y[4 + k] = phi
        hold.append((through / ratio - motor["cogging_nm"] * math.sin(motor["cogging_order"] * phi))
                    / (1 + motor["ripple_fraction"] * math.sin(motor["ripple_order"] * phi)))
    return y, np.array(hold)


def pulse(t):
    bump = lambda t0: math.sin(math.pi * (t - t0) / 0.15) ** 2 if t0 < t < t0 + 0.15 else 0.0
    return bump(0.0) - bump(0.3)


def main():
    pose = (0.5, 0.25)
    y0, hold = equilibrium(*pose)
    # The push asks for each axis's acceleration limit, through the rigid model.
    fx = (m_x + m_motors) * cfg["axis"]["x"]["max_acceleration_m_s2"]
    fz = (m_z + m_motors) * cfg["axis"]["z"]["max_acceleration_m_s2"]
    delta = r / (2 * ratio) * np.array([fx + fz, fx - fz])
    torque_at = lambda t: hold + delta * pulse(t)
    assert max(abs(v) for v in rhs(0.0, y0, torque_at)) < 1e-6, "the start state is not at rest"

    times = np.round(np.arange(601) * 1e-3, 9)
    sol = solve_ivp(rhs, (0.0, 0.6), y0, method="LSODA", t_eval=times, args=(torque_at,),
                    rtol=1e-10, atol=1e-13, max_step=5e-5)
    assert sol.success, sol.message
    with open(HERE / "scipy_trace.csv", "w", encoding="utf-8", newline="\n") as f:
        f.write("# Written by scipy_reference.py from default.toml. Regenerate when the model or the config changes.\n")
        f.write("t,x,z\n")
        for t, x, z in zip(sol.t, sol.y[0], sol.y[1]):
            f.write(f"{t:.3f},{x:.12f},{z:.12f}\n")
    print(f"travel x {np.ptp(sol.y[0]):.4f} m, z {np.ptp(sol.y[1]):.4f} m, {sol.nfev} evaluations")


if __name__ == "__main__":
    main()
