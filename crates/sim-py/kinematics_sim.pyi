"""Type stubs of the native module kinematics_sim (crates/sim-py/src/lib.rs)."""

from os import PathLike
from typing import Any, Literal, Sequence, TypedDict, final

import numpy as np
import numpy.typing as npt

__version__: str
__all__ = [
    "Config", "machines", "signals", "run", "analyze", "verdicts", "sizing", "sweep", "optimise", "fit", "identify",
    "__version__",
]

FloatArray = npt.NDArray[np.float64]
BoolArray = npt.NDArray[np.bool_]

class MachineRow(TypedDict):
    id: str
    name: str

@final
class Config:
    """A machine's config: its default, or TOML text, or a config file; values set on top as `sim-cli --set` does."""

    def __new__(
        cls,
        machine: str | None = None,
        text: str | None = None,
        path: str | PathLike[str] | None = None,
        *,
        machine_file: str | PathLike[str] | None = None,
    ) -> Config:
        """A registered machine by id, or a machine file (docs/machine-files.md); one of the two."""
    @property
    def machine(self) -> str: ...
    def __getitem__(self, path: str, /) -> Any: ...
    def __setitem__(self, path: str, value: bool | int | float | str | Sequence[Any] | dict[str, Any], /) -> None:
        """An int beyond 64 bits raises OverflowError."""
    def set(self, assignment: str) -> None:
        """One `path = value` assignment with the value in TOML syntax, exactly as `sim-cli --set` takes it."""
    def copy(self) -> Config: ...
    def leaves(self) -> dict[str, Any]: ...
    def to_toml(self) -> str: ...

def machines(lang: Literal["en", "de"] = "en") -> list[MachineRow]: ...
def signals(config: Config) -> dict[str, list[str]]:
    """{"id": [...], "unit": [...]} in the order `run` gives the signals."""

def run(
    config: Config,
    *,
    cycle: bool = False,
    move_to: Sequence[float] | None = None,
    path: str | None = None,
    trajectory: str | None = None,
    time_optimal: bool = False,
    time: float = 2.0,
    every: int = 16,
    disable_at: float | None = None,
) -> dict[str, FloatArray]:
    """Every signal over `time` seconds, sampled every `every` base steps; "time" first. At most 1e8 base steps and
    2.5e8 values sampled: beyond, ValueError."""

def analyze(
    config: Config,
    *,
    path: str | None = None,
    trajectory: str | None = None,
    time_optimal: bool = False,
    points: bool = False,
) -> dict[str, Any]:
    """The load per motor over one pass of the cycle (or a path), one row per motor."""

def verdicts(config: Config, *, bands: bool = False, samples: int | None = None, seed: int = 1) -> dict[str, Any]:
    """The checks of the cycle, one row per check and motor; with bands their low, high and worst case."""

def sizing(
    config: Config,
    *,
    supply: str | None = None,
    vendor: str | None = None,
    series: str = "",
    gearboxes: str | None = None,
    axes: bool = False,
) -> dict[str, Any]:
    """Every catalogue motor on the config's cycle, passing and cheapest first."""

def sweep(
    config: Config,
    *,
    motor: int = 0,
    from_hz: float = 5.0,
    to_hz: float = 500.0,
    count: int = 120,
    amplitude: float | None = None,
) -> dict[str, FloatArray]:
    """The frequency response torque to speed of one motor: frequency_hz, gain_rad_s_per_nm, phase_deg."""

def optimise(
    config: Config, *, runs: int = 40, nominal: bool = False, worst: bool = False, pareto: bool = False
) -> dict[str, Any]:
    """A shorter cycle that passes every verdict: cycle_s (before, after), changes, robust, pareto."""

def fit(
    config: Config,
    *,
    trace_file: str | PathLike[str] | None = None,
    map_file: str | PathLike[str] | None = None,
    stiffness: bool = False,
    stiffness_key: str | None = None,
    sweep_file: str | PathLike[str] | None = None,
    sweep_motor: int = 0,
    amplitude: float | None = None,
) -> dict[str, Any]:
    """What a measurement says the config should be: quantity, of, config, fitted, standard_error, advice."""

def identify(
    *,
    trace_file: str | PathLike[str] | None = None,
    map_file: str | PathLike[str] | None = None,
    config: Config | None = None,
    motor: int = 0,
    from_hz: float | None = None,
    to_hz: float | None = None,
    step_from: float | None = None,
    step_to: float | None = None,
    motor_inertia: float | None = None,
    sweep_file: str | PathLike[str] | None = None,
) -> dict[str, Any]:
    """One motor's drive train from its trace: quantity, of, value, standard_error, ci95_low, ci95_high, notes, overlay."""
