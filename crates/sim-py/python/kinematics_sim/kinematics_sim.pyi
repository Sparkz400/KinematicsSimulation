"""Type stubs of the native module kinematics_sim.kinematics_sim (crates/sim-py/src/lib.rs), re-exported by the package."""

from os import PathLike
from typing import Any, Callable, Literal, Sequence, TypeVar, TypedDict, final

import numpy as np
import numpy.typing as npt

__version__: str
__all__ = [
    "Config", "Sim", "Job", "Cancelled", "machines", "signals", "label", "run", "analyze", "verdicts", "sizing", "sweep", "optimise", "fit", "identify",
    "step", "bode", "tuning", "margins", "modes", "sensitivity", "global_sensitivity", "replay", "compare", "report", "check_dataset", "fmu",
    "accuracy", "__version__",
]

T = TypeVar("T")
FloatArray = npt.NDArray[np.float64]
BoolArray = npt.NDArray[np.bool_]

class MachineRow(TypedDict):
    id: str
    name: str

class Cancelled(Exception):
    """A `Job` was cancelled while its callable ran."""

@final
class Job:
    """A long analysis to watch and stop from another thread: every function `run`'s callable calls belongs to it."""

    def __init__(self) -> None: ...
    def cancel(self) -> None:
        """Stops the job from any thread: no new run starts, `run` raises `Cancelled` when the ones under way end."""
    @property
    def done(self) -> int:
        """Runs finished so far."""
    @property
    def total(self) -> int:
        """Runs planned so far (later batches add to it)."""
    @property
    def cancelled(self) -> bool: ...
    def run(self, callable: Callable[[], T]) -> T:
        """`callable()` on this thread as part of the job; raises `Cancelled` where the job was cancelled."""

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

@final
class Sim:
    """A simulation stepped from Python, for a controller of your own (docs/python.md, "Test your own controller")."""

    def __new__(cls, config: Config) -> Sim:
        """At rest at the start pose, drives enabled, every motor in position mode."""
    def step(
        self,
        *,
        torque: Sequence[float] | None = None,
        velocity: Sequence[float] | None = None,
        setpoint: Sequence[float] | None = None,
        time: float | None = None,
    ) -> dict[str, float]:
        """One controller cycle (`cycle`), or `time` seconds in whole base steps, with at most one of: `torque` (N m
        per motor, torque mode), `velocity` (rad/s per motor, velocity mode), `setpoint` (per tool axis, position
        mode). Every signal at its end. A refused mode or setpoint raises ValueError."""
    def stop(self, category: Literal[0, 1, 2]) -> None:
        """Stop category 0 (torque off), 1 (quick-stop ramp, then off) or 2 (brake the plan, hold with torque on);
        it runs in the steps that follow. A refused stop raises ValueError."""
    def enable(self) -> None:
        """The drives on again after a stop, holding where the motors stand."""
    def signals(self) -> dict[str, float]: ...
    @property
    def time(self) -> float: ...
    @property
    def dt(self) -> float: ...
    @property
    def cycle(self) -> float:
        """profile.setpoint_interval_s: what a step runs without `time`."""
    @property
    def modes(self) -> list[Literal["position", "velocity", "torque"]]: ...
    def reset(self) -> None: ...
    def analyze(self, *, points: bool = False) -> dict[str, Any]:
        """The columns of `analyze` over the run since the start or the last `reset`."""
    def verdicts(self) -> dict[str, Any]:
        """"check", "motor", "value", "limit", "ok" on the run since the start or the last `reset`."""

def machines(lang: Literal["en", "de"] = "en") -> list[MachineRow]: ...
def signals(config: Config) -> dict[str, list[str]]:
    """{"id": [...], "unit": [...]} in the order `run` gives the signals."""
def label(config: Config, key: str, lang: Literal["en", "de"] = "en") -> str:
    """The text of a message key for the config's machine, e.g. "check-motor-thermal"; a key with none comes back as is."""

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
    """The load per motor over one pass of the cycle (or a path), one row per motor; torque_unit and speed_unit give
    each motor's units ("N" and "m/s" on a linear motor)."""

def verdicts(
    config: Config,
    *,
    bands: bool = False,
    samples: int | None = None,
    seed: int = 1,
    worst: bool = False,
    path: str | None = None,
    path_file: str | None = None,
    time_optimal: bool = False,
) -> dict[str, Any]:
    """The checks of the cycle, one row per check and motor; with bands their low, high and worst case; with
    bands and worst the worst corner of the bounds per check (worst, worst_ok, at_high_bound, at_low_bound, runs)."""

def sizing(
    config: Config,
    *,
    supply: str | None = None,
    vendor: str | None = None,
    series: str = "",
    gearboxes: str | None = None,
    axes: bool = False,
    motor_id: str | None = None,
    drives: str = "paired",
    rank_by: str = "cost",
    duties: list[tuple[Config, float]] | None = None,
    path: str | None = None,
    path_file: str | None = None,
    time_optimal: bool = False,
) -> dict[str, Any]:
    """Every catalogue motor on the config's cycle, passing first, ranked by `rank_by`; the columns of `sim-cli search`,
    with torque_unit the unit of peak_torque_nm ("N" on a machine of linear motors)."""

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
    config: Config,
    *,
    runs: int = 40,
    nominal: bool = False,
    worst: bool = False,
    pareto: bool = False,
    path: str | None = None,
    path_file: str | None = None,
    time_optimal: bool = False,
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
    """What a measurement says the config should be: quantity, of, config, fitted, standard_error, units (per row, ""
    where none), advice, and errors (the fits that found nothing; their rows are missing)."""

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
    """One motor's drive train from its trace: quantity, of, value, standard_error, ci95_low, ci95_high, notes, overlay,
    and errors (the fits that found nothing; their rows are missing)."""

def step(config: Config, *, axis_index: int = 0, size: float = 0.001, time: float = 0.5) -> dict[str, Any]:
    """One axis's answer to a setpoint step: time_s, setpoint, response; rise_time_s, overshoot, settling_time_s."""

def bode(
    config: Config,
    *,
    axis_index: int = 0,
    from_hz: float = 1.0,
    to_hz: float = 200.0,
    count: int = 40,
    amplitude: float = 0.0005,
) -> dict[str, FloatArray]:
    """The closed-loop response of one axis's position loop: frequency_hz, gain, gain_db, phase_deg."""

def tuning(config: Config, *, size: float = 0.001, amplitude: float = 0.0005) -> dict[str, Any]:
    """Per axis: axis, rise_time_s, overshoot, settling_time_s, bandwidth_hz (NaN where none below 200 Hz)."""

def margins(config: Config, *, bands: bool = False) -> dict[str, Any]:
    """Per run and motor the velocity loop's margins: run, motor, crossover_hz, phase_margin_deg, gain_margin_db,
    peak_sensitivity, peak_frequency_hz, ok. crossover_hz and phase_margin_deg are NaN without a gain crossover in the
    band (not measurable; ok then judges the gain margin alone)."""

def accuracy(config: Config, *, points: int = 5, repeats: int = 5, dwell: float = 0.3) -> dict[str, Any]:
    """Static positioning accuracy along ISO 230-2 and ISO 9283: per axis and target axis, target, mean_up, mean_down,
    std_up, std_down; "summary" per axis (reversal, repeatability, systematic, mean_range, accuracy); tool_accuracy,
    tool_repeatability, tool_direction_variation; "verdicts" (check, value, limit, ok)."""

def modes(config: Config, *, axis_index: int = 0, points: int = 21) -> dict[str, FloatArray]:
    """The modes over one axis's stroke: position, mode<i>_hz, mode<i>_damping (NaN where a place lacks one), and
    mode<i>_small_hz where the small-amplitude modes differ (a gearbox's hysteresis)."""

def sensitivity(config: Config, *, delta: float = 0.2, groups: Sequence[str] | None = None) -> dict[str, Any]:
    """Each parameter at its low and high value, strongest first: parameter, value, low, high, relative changes."""

def global_sensitivity(
    config: Config,
    *,
    method: Literal["morris", "sobol"] = "morris",
    samples: int | None = None,
    seed: int = 1,
    groups: Sequence[str] | None = None,
    results: Sequence[str] | None = None,
) -> dict[str, Any]:
    """All values of [bounds] moving at once, ranked per result: result, rank, parameter, and mu_star, mu, sigma
    (morris) or first, first_low, first_high, total, total_low, total_high (sobol)."""

def replay(
    config: Config, *, trace_file: str | PathLike[str], map_file: str | PathLike[str] | None = None
) -> dict[str, Any]:
    """A trace's setpoints through the simulator: per motor the torque and following error apart, and overlay."""

def compare(config: Config, *, effect: str, signals: Sequence[str] | None = None) -> dict[str, Any]:
    """The cycle with effects.<effect> on and off: time, <id>.on, <id>.off, and differences."""

def report(config: Config, path: str | PathLike[str], *, bands: bool = False, lang: Literal["en", "de"] = "en") -> bool:
    """The sizing report as PDF (a path ending in .pdf) or HTML; True where every check passed."""

def check_dataset(path: str | PathLike[str]) -> list[str]:
    """What keeps a dataset folder from being published; empty where nothing does."""

def fmu(
    config: Config,
    path: str | PathLike[str],
    *,
    binaries: Sequence[str | PathLike[str]],
    fmi: Literal[2, 3] = 2,
    lang: Literal["en", "de"] = "en",
) -> None:
    """The machine as an FMI 2.0 or 3.0 Co-Simulation FMU carrying the given sim-fmi binaries."""
