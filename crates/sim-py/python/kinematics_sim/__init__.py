"""The simulator from Python: the native module (crates/sim-py/src/lib.rs, re-exported here) and, with the extra `mcp`,
an MCP server for LLM clients (`kinematics_sim.mcp`, docs/mcp.md)."""

from .kinematics_sim import *  # noqa: F403
from .kinematics_sim import __all__, __version__  # noqa: F401
