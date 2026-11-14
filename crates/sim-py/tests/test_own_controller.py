"""`kinematics_sim.Sim`: a controller of the user's own closes the loop through torque or velocity mode, and the run is
judged by the same analysis and verdicts as the built-in drive's. The tests name no machine (a rule of the
architecture test): they take the first machine of the registry with one motor."""

import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest

import kinematics_sim as ks

EXAMPLE = Path(__file__).resolve().parents[1] / "examples" / "own_controller.py"


def example():
    spec = importlib.util.spec_from_file_location("own_controller", EXAMPLE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


MACHINE = example().one_motor_machine()


def ideal() -> ks.Config:
    """Rigid, no friction, no saturation, no iron loss: the torque constant holds at every current and nothing but
    the inertia takes torque."""
    cfg = ks.Config(MACHINE)
    words = ("friction", "saturation", "iron_loss", "ripple", "meshing", "efficiency", "drag", "stretch", "encoder")
    for key, value in cfg.leaves().items():
        if key.startswith("effects.") and isinstance(value, bool) and any(word in key for word in words):
            cfg[key] = False
    return cfg


def motor_signal(out: dict, name: str) -> float:
    return next(v for k, v in out.items() if k.startswith("motor.") and k.endswith("." + name))


def test_a_constant_torque_gives_current_torque_over_kt_and_the_analysis_sees_it():
    # By hand: 0.3 N m in torque mode is 0.3 / k_t A in the winding, and the load over the run peaks at 0.3 N m.
    cfg = ideal()
    sim = ks.Sim(cfg)
    out = sim.step(torque=[0.3], time=0.02)
    assert sim.modes == ["torque"]
    kt = cfg["motor.torque_constant_nm_a"]
    assert motor_signal(out, "torque_command") == pytest.approx(0.3, abs=1e-12)
    assert motor_signal(out, "current") == pytest.approx(0.3 / kt, rel=1e-3)
    load = sim.analyze()
    assert load["peak_torque_nm"][0] == pytest.approx(0.3, rel=1e-3)
    assert load["cycle_s"][0] == pytest.approx(sim.time)
    assert load["rms_current_a"][0] < 0.3 / kt * (1 + 1e-3)
    verdicts = sim.verdicts()
    assert len(verdicts["check"]) > 0 and verdicts["ok"].dtype == bool


def test_a_speed_setpoint_settles_and_reset_goes_back_to_the_start():
    sim = ks.Sim(ideal())
    start = sim.signals()
    out = sim.step(velocity=[20.0], time=0.5)
    assert sim.modes == ["velocity"]
    assert motor_signal(out, "speed") == pytest.approx(20.0, abs=1e-6)
    assert motor_signal(out, "following_error") == 0.0
    sim.reset()
    assert sim.time == 0.0 and sim.modes == ["position"] and sim.signals() == start


def test_position_mode_steps_are_the_run_of_the_built_in_drive():
    # Stepping without commands is the plain simulation: bit for bit what `run` samples at the same time.
    cfg = ks.Config(MACHINE)
    sim = ks.Sim(cfg)
    for _ in range(5):
        out = sim.step()
    ran = ks.run(cfg, time=5 * sim.cycle, every=1)
    assert [out[k] for k in ran] == [ran[k][-1] for k in ran]


def test_what_cannot_be_stepped_is_refused():
    sim = ks.Sim(ks.Config(MACHINE))
    with pytest.raises(ValueError, match="at most one"):
        sim.step(torque=[0.0], velocity=[0.0])
    with pytest.raises(ValueError, match="one per motor"):
        sim.step(torque=[0.0, 0.0])
    with pytest.raises(ValueError, match="one per motor"):
        sim.step(torque=[math.nan])
    with pytest.raises(ValueError, match="base steps"):
        sim.step(time=0.0)
    sim.step(torque=[0.0])
    # A plan command waits for position mode: a setpoint puts the drive back in it.
    sim.step(setpoint=[sim.signals()[next(k for k in sim.signals() if k.endswith(".position_setpoint"))]])
    assert sim.modes == ["position"]


def test_the_example_pd_follows_the_move_and_is_judged_like_the_built_in_drive():
    found = example().main(MACHINE, seconds=0.4)
    assert np.isfinite(found["inertia"]) and found["inertia"] > 0
    # The PD needs about the torque the cascade needs for the same move (both move the same inertia).
    assert found["own_rms"] == pytest.approx(found["built_in_rms"], rel=0.2)
