"""Independent reference for the arms' rigid-body dynamics: MuJoCo reads the same URDF files as the simulator and
gives, at random states inside the joints' limits, the inverse dynamics with gravity (mj_inverse), the mass matrix
(mj_fullM), the gravity torques alone (the bias forces at rest) and the flange's position and rotation.

Mechanism only: no motor rotors, no gearboxes, no friction, no elasticity, no payload. The palletiser's parallelogram
is the config's arm.joint_coupling (the URDF's joints as sums of the motors' joints, q_urdf = C q): MuJoCo works on the
serial five-joint URDF chain and the numbers are taken to the motors' joints here, M = C^T M_urdf C, tau = C^T tau_urdf
(C constant, so its constraint forces do no work), as a mimic joint would.

Writes <urdf>.csv next to this file, one per URDF; the Rust test tests/reference.rs compares the crate against them.
Pinocchio, the first choice, has no wheel for Windows; MuJoCo has, and runs the same on Linux.

Run from the repository root, with the pinned packages of requirements.txt in a venv:

    python -m venv .venv
    .venv/Scripts/python -m pip install -r crates/machine-joint-kit/reference/requirements.txt
    .venv/Scripts/python crates/machine-joint-kit/reference/mujoco_reference.py
"""

import tomllib
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).parent
URDF = HERE.parent / "urdf"
CONFIG = HERE.parent / "config"
G = 9.80665  # the crate's (lib.rs)
STATES = 60
SEED = 20261114

# URDF file and the config with its arm.joint_coupling ("" for one motor per joint). New files go at the end: the
# random states are drawn in this order from one generator, so that the files before keep theirs.
ARMS = [
    ("six-axis", ""),
    ("scara", ""),
    ("five-axis", ""),
    ("seven-axis", ""),
    ("palletiser", "palletiser.toml"),
    ("arm-on-track", ""),
]

# Keep the fixed flange frame as a body of its own (MuJoCo fuses fixed links by default), every inertia as written.
COMPILER = '<mujoco><compiler fusestatic="false" discardvisual="true" balanceinertia="false"/></mujoco>'


def load(name):
    text = (URDF / f"{name}.urdf").read_text(encoding="utf-8")
    head, tail = text.split("<robot", 1)
    head_end = tail.index(">") + 1
    model = mujoco.MjModel.from_xml_string(head + "<robot" + tail[:head_end] + COMPILER + tail[head_end:])
    model.opt.gravity[:] = [0.0, 0.0, -G]
    # Inside the limits no constraint acts; switched off so that none can.
    model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONSTRAINT
    assert model.nq == model.nv, "only revolute and prismatic joints"
    assert not model.dof_armature.any() and not model.dof_damping.any() and not model.jnt_stiffness.any()
    return model


def coupling(config, u):
    if not config:
        return np.eye(u)
    rows = tomllib.loads((CONFIG / config).read_text(encoding="utf-8"))["arm"]["joint_coupling"]
    return np.array(rows, dtype=float)


def write(name, config, rng):
    model = load(name)
    data = mujoco.MjData(model)
    u = model.nq
    c = coupling(config, u)
    n = c.shape[1]
    # The URDF's limits; a continuous joint turns over one turn either way.
    low = np.where(model.jnt_limited, model.jnt_range[:, 0], -np.pi)
    high = np.where(model.jnt_limited, model.jnt_range[:, 1], np.pi)
    prismatic = model.jnt_type == mujoco.mjtJoint.mjJNT_SLIDE
    # Coupled motor joints: drawn in a box around the coupled ones, kept only where every URDF joint is in its limits.
    if config:
        span = np.abs(np.linalg.pinv(c)) @ np.maximum(np.abs(low), np.abs(high))
        low_m, high_m, speed = -span, span, np.full(n, 2.0)
    else:
        low_m, high_m, speed = low, high, np.where(prismatic, 0.5, 2.0)
    tip = model.nbody - 1  # serial chain: the last body is the flange
    rows = []
    while len(rows) < STATES:
        q = rng.uniform(low_m, high_m)
        qu = c @ q
        if not np.all((low <= qu) & (qu <= high)):
            continue
        qd = rng.uniform(-speed, speed)
        qdd = rng.uniform(-5.0 * speed, 5.0 * speed)
        data.qpos[:], data.qvel[:], data.qacc[:] = qu, c @ qd, c @ qdd
        mujoco.mj_inverse(model, data)
        tau = c.T @ data.qfrc_inverse
        mass = np.zeros((u, u))
        mujoco.mj_fullM(model, data, mass)
        mass = c.T @ mass @ c
        pos, rot = data.xpos[tip].copy(), data.xmat[tip].copy()
        data.qvel[:] = 0.0
        mujoco.mj_forward(model, data)
        gravity = c.T @ data.qfrc_bias
        rows.append(np.concatenate([q, qd, qdd, tau, gravity, mass.ravel(), pos, rot]))
    head = (
        [f"q{k}" for k in range(n)] + [f"qd{k}" for k in range(n)] + [f"qdd{k}" for k in range(n)]
        + [f"tau{k}" for k in range(n)] + [f"g{k}" for k in range(n)]
        + [f"m{r}{k}" for r in range(n) for k in range(n)] + ["x", "y", "z"] + [f"r{r}{k}" for r in range(3) for k in range(3)]
    )
    lines = [
        f"# MuJoCo {mujoco.__version__}, urdf/{name}.urdf, {STATES} states, seed {SEED}, g = {G} m/s^2: joints (rad, m),"
        " their speeds and accelerations, the joint torques (N m, N) of inverse dynamics with gravity, the gravity"
        " torques alone, the mass matrix row by row, the flange's position (m) and rotation row by row in the base frame."
        + (f" Motor joints: the URDF's are config/{config}'s arm.joint_coupling of them." if config else ""),
        ",".join(head),
    ] + [",".join(repr(float(v)) for v in r) for r in rows]
    (HERE / f"{name}.csv").write_text("\n".join(lines) + "\n", encoding="utf-8", newline="\n")
    print(f"{name}: {u} URDF joints, {n} motor joints, flange body {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, tip)}")


rng = np.random.default_rng(SEED)
for name, config in ARMS:
    write(name, config, rng)
