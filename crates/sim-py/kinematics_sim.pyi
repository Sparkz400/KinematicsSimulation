"""Type stubs of the native module kinematics_sim (crates/sim-py/src/lib.rs)."""

from os import PathLike
from typing import Any, Literal, Sequence, TypedDict, final

import numpy as np
import numpy.typing as npt

__version__: str
__all__ = ["Config", "machines", "signals", "run", "analyze", "verdicts", "sizing", "__version__"]

FloatArray = npt.NDArray[np.float64]
BoolArray = npt.NDArray[np.bool_]

class MachineRow(TypedDict):
    id: str
    name: str

@final
class Config:
    """A machine's config: its default, or TOML text, or a config file; values set on top as `sim-cli --set` does."""

    def __new__(cls, machine: str, text: str | None = None, path: str | PathLike[str] | None = None) -> Config: ...
    @property
    def machine(self) -> str: ...
    def __getitem__(self, path: str, /) -> Any: ...
    def __setitem__(self, path: str, value: bool | int | float | str | Sequence[Any] | dict[str, Any], /) -> None: ...
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
    """Every signal over `time` seconds, sampled every `every` base steps; "time" first."""

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
