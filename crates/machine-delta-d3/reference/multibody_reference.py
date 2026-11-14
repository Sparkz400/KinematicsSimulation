"""What the lumped forearms cost: the D3 as a full multibody model in MuJoCo against the mass split of
Codourey/Olsson (effects.forearm_inertia off), over the states of the Adept cycle 25/305/25 mm of default.toml.

The multibody model: each upper arm a body on a hinge (its mass, centre of mass and inertia about the joint), the
elbow block a point mass on it, each of the six forearms a rigid tube of half the pair's mass with its true inertia
(m L^2 / 12 + I_axial / 2 across, mass.forearm_rod_spin_inertia_kgm2 along) on a ball joint at its upper end, the
platform with the payload a free body, the rotors as the hinges' armature. MuJoCo's own inverse dynamics (mj_inverse)
gives the generalised forces of that open tree at a state of the closed chain; the motors' torques are those forces
by virtual work, tau = G^T f with G = dq_tree/dtheta (the closing ball joints at the platform do no work along the
chain's motion). G and its rate by central differences of MuJoCo's own coordinates (mj_differentiatePos).

The same pipeline with the lumped masses instead of the tubes is checked against lagrange_reference.py first, so that
the difference printed is the model's and not the pipeline's. The crate's forearms as rods
(effects.forearm_inertia, shipped) are compared too: they leave out only the tubes' own cross-section. Nothing is written; the numbers go into
docs/validation.md ("Independent reference").

    .venv/Scripts/python crates/machine-delta-d3/reference/multibody_reference.py
"""

import math
import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import lagrange_reference as ref  # noqa: E402  (the config, the inverse kinematics and the Lagrangian)

cfg, mass, G = ref.cfg, ref.mass, ref.G
R, r, La, Lb, ratio = ref.R, ref.r, ref.La, ref.Lb, ref.ratio
spacing = cfg["geometry"]["forearm_spacing_m"]
m_fb = ref.m_fb
TINY = 1e-9  # MuJoCo wants positive inertia; a point mass gets this


def mjcf(rods):
    share = 0.0 if rods else mass["forearm_elbow_share"]
    m_plate = ref.m_plate + (0.0 if rods else 3 * (1 - share) * m_fb)
    i_cg = mass["upper_arm_inertia_kgm2"] - mass["upper_arm_kg"] * mass["upper_arm_cg_m"] ** 2
    axial = mass["forearm_rod_spin_inertia_kgm2"]
    m_rod = m_fb / 2
    across = m_rod * Lb**2 / 12 + axial / 2
    arms = []
    for i, phi in enumerate(ref.PHI):
        c, s = math.cos(phi), math.sin(phi)
        rods_xml = "".join(
            f'<body name="rod{i}{k}" pos="{La} {side * spacing / 2} 0"><joint name="ball{i}{k}" type="ball"/>'
            f'<inertial pos="{Lb / 2} 0 0" mass="{m_rod}" diaginertia="{axial} {across} {across}"/></body>'
            for k, side in enumerate((-1, 1))
        ) if rods else ""
        arms.append(
            f'<body name="arm{i}" pos="{R * c} {R * s} 0" quat="{math.cos(phi / 2)} 0 0 {math.sin(phi / 2)}">'
            f'<joint name="hinge{i}" type="hinge" axis="0 1 0" armature="{ref.j_rotor * ratio**2}"/>'
            f'<inertial pos="{mass["upper_arm_cg_m"]} 0 0" mass="{mass["upper_arm_kg"]}" diaginertia="{TINY} {i_cg} {i_cg}"/>'
            f'<body name="elbow{i}" pos="{La} 0 0"><inertial pos="0 0 0" mass="{mass["elbow_kg"] + share * m_fb}" '
            f'diaginertia="{TINY} {TINY} {TINY}"/></body>{rods_xml}</body>'
        )
    return (
        f'<mujoco><option gravity="0 0 {-G}"/><worldbody>{"".join(arms)}'
        f'<body name="platform"><freejoint/><inertial pos="0 0 0" mass="{m_plate}" diaginertia="{TINY} {TINY} {TINY}"/></body>'
        "</worldbody></mujoco>"
    )


def rotation_y(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rotation_z(phi):
    c, s = math.cos(phi), math.sin(phi)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def tree_pose(model, theta, pos):
    """MuJoCo's coordinates of the tree at the arm angles and the platform's position."""
    qpos = np.zeros(model.nq)
    for i, phi in enumerate(ref.PHI):
        qpos[model.jnt_qposadr[model.joint(f"hinge{i}").id]] = theta[i]
        arm = rotation_z(phi) @ rotation_y(theta[i])
        axis = rotation_z(phi) @ np.array([0.0, 1.0, 0.0])
        elbow = rotation_z(phi) @ np.array([R + La * math.cos(theta[i]), 0.0, -La * math.sin(theta[i])])
        plate = pos + rotation_z(phi) @ np.array([r, 0.0, 0.0])
        if model.njnt > 4:
            # The rod's frame: along it, and square to the pair's plane through the elbow's axis (its spin as the
            # crate counts it; negligible either way, see the docs).
            e = (plate - elbow) / Lb
            n = np.cross(axis, e)
            n /= np.linalg.norm(n)
            rel = arm.T @ np.column_stack([e, n, np.cross(e, n)])
            quat = np.zeros(4)
            mujoco.mju_mat2Quat(quat, rel.flatten())
            for k in range(2):
                adr = model.jnt_qposadr[model.joint(f"ball{i}{k}").id]
                qpos[adr:adr + 4] = quat
    adr = model.jnt_qposadr[model.joint(model.njnt - 1).id]
    qpos[adr:adr + 3] = pos
    qpos[adr + 3] = 1.0
    return qpos


def chain(model, phi, phi_q, theta):
    """G = dq_tree/dtheta at the arm angles, by central differences."""
    h = 1e-6
    out = np.zeros((3, model.nv))
    for k in range(3):
        up, down = theta.copy(), theta.copy()
        up[k] += h
        down[k] -= h
        q_up = tree_pose(model, up, ref.forward(phi, phi_q, up, [0, 0, -0.9]))
        q_down = tree_pose(model, down, ref.forward(phi, phi_q, down, [0, 0, -0.9]))
        mujoco.mj_differentiatePos(model, out[k], 2 * h, q_down, q_up)
    return out.T


def motor_torques(model, data, lag, pos, vel, acc):
    _, _, phi, phi_q, conv = lag
    qv, qdv, qdd = ref.kinematics(phi_q, conv, pos, vel, acc)
    theta, w, wd = qv[:3], qdv[:3], qdd[:3]
    g = chain(model, phi, phi_q, theta)
    eps = 1e-5
    g_rate = (chain(model, phi, phi_q, theta + eps * w) - chain(model, phi, phi_q, theta - eps * w)) / (2 * eps)
    data.qpos[:] = tree_pose(model, theta, pos)
    data.qvel[:] = g @ w
    data.qacc[:] = g @ wd + g_rate @ w
    mujoco.mj_inverse(model, data)
    return g.T @ data.qfrc_inverse / ratio


def s_curve(d, v, a, j):
    """The profile generator's symmetric S-curve over a distance d: [(duration, jerk)], stopping at the end."""
    tj = min(a / j, math.sqrt(v / j))
    ta = v / (j * tj) + tj if tj == a / j else 2 * tj
    if v * ta > d:  # no cruise: the largest top speed that fits
        tj = (d / (2 * j)) ** (1 / 3)
        if j * tj > a:  # reaches the acceleration: solve d = a ta^2 / 2 ... with ta = v'/a + a/j
            tj = a / j
            vp = (-a * tj + math.sqrt((a * tj) ** 2 + 4 * a * d)) / 2
            ta = vp / a + tj
        else:
            ta = 2 * tj
        cruise = 0.0
        vp = j * tj * (ta - tj)
    else:
        vp = v
        cruise = (d - v * ta) / v
    flat = ta - 2 * tj
    up = [(tj, j), (flat, 0.0), (tj, -j)]
    return up + [(cruise, 0.0)] + [(t, -jj) for t, jj in up]


def cycle_states(dt=2e-4):
    """Position, speed and acceleration along the Adept cycle of default.toml, each leg along one axis."""
    ax = cfg["axis"]
    corners = [np.array(cfg["start"]["pose"])] + [np.array(s["target"]) for s in cfg["cycle"]["segment"]]
    states = []
    for a, b in zip(corners[:-1], corners[1:]):
        axis = int(np.argmax(np.abs(b - a)))
        d = abs(b[axis] - a[axis])
        lim = ax["xyz"[axis]]
        segments = s_curve(d, lim["max_velocity_m_s"], lim["max_acceleration_m_s2"], lim["max_jerk_m_s3"])
        x = v = acc = 0.0
        for duration, jerk in segments:
            n = max(1, round(duration / dt))
            h = duration / n
            for _ in range(n):
                x, v, acc = x + v * h + acc * h * h / 2 + jerk * h**3 / 6, v + acc * h + jerk * h * h / 2, acc + jerk * h
                unit = np.sign(b[axis] - a[axis]) * np.eye(3)[axis]
                states.append((a + unit * x, unit * v, unit * acc))
        assert abs(x - d) < 1e-9 and abs(v) < 1e-9, (x, d, v)
    return states


def main():
    lumped, rods = ref.equations(False), ref.equations(True)
    pipeline_check, full = mujoco.MjModel.from_xml_string(mjcf(False)), mujoco.MjModel.from_xml_string(mjcf(True))
    data_check, data_full = mujoco.MjData(pipeline_check), mujoco.MjData(full)
    states = cycle_states()
    worst_check = worst = worst_rods = 0.0
    rms = np.zeros((2, 3))
    peak = np.zeros((2, 3))
    for pos, vel, acc in states[::5]:
        lag = ref.motor_torques(lumped, pos, vel, acc)
        check = motor_torques(pipeline_check, data_check, lumped, pos, vel, acc)
        multibody = motor_torques(full, data_full, lumped, pos, vel, acc)
        worst_check = max(worst_check, np.abs(check - lag).max())
        worst = max(worst, np.abs(multibody - lag).max())
        worst_rods = max(worst_rods, np.abs(multibody - ref.motor_torques(rods, pos, vel, acc)).max())
        rms += np.array([lag, multibody]) ** 2
        peak = np.maximum(peak, np.abs([lag, multibody]))
    rms = np.sqrt(rms / len(states[::5]))
    print(f"{len(states[::5])} states of the Adept cycle")
    print(f"pipeline: MuJoCo with the lumped masses against the Lagrangian, largest difference {worst_check:.3e} N m")
    print(f"mass split: lumped against the multibody model, largest difference {worst:.4f} N m")
    print(f"rods (the shipped effects.forearm_inertia) against the multibody model, largest difference {worst_rods:.3e} N m")
    print(f"RMS per motor lumped {rms[0]}, multibody {rms[1]} N m; peak lumped {peak[0]}, multibody {peak[1]} N m")
    assert worst_check < 1e-5, "the pipeline does not reproduce the lumped model"


if __name__ == "__main__":
    main()
