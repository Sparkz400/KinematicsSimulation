"""A --set sweep in Python: the single belt axis's default cycle at three payloads, one row per payload with the
motor's peak and RMS torque, its thermal utilisation and whether every check passes. Each value is set as
`sim-cli --set mass.payload_kg=...` would set it. Needs the module kinematics_sim (docs/python.md).

    python examples/payload_sweep.py
"""

import kinematics_sim as ks

print("payload_kg,peak_torque_nm,rms_torque_nm,thermal_utilisation,all_checks_pass")
for payload in [0.0, 1.0, 2.0]:
    cfg = ks.Config("single-axis")
    cfg.set(f"mass.payload_kg = {payload}")
    load = ks.analyze(cfg)
    passed = bool(ks.verdicts(cfg)["ok"].all())
    print(f"{payload},{load['peak_torque_nm'][0]:.4f},{load['rms_torque_nm'][0]:.4f},"
          f"{load['thermal_utilisation'][0]:.4f},{passed}")
