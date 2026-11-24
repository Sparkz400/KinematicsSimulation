# MCP server and coding agents

Two ways for an LLM to drive the simulator: the **MCP server** for chat clients (Claude Desktop, Claude Code, Cursor,
any client of the Model Context Protocol), and **`sim-cli`** for coding agents that have a shell. Both give the
numbers of the [Python module](python.md) and the [command line](user-guide.md); neither makes them more certain:
read [what it can and cannot tell you](overview.md#what-it-can-and-cannot-tell-you) first.

## The MCP server

`kinematics_sim.mcp` is a server on top of the Python module, written with the official MCP Python SDK (`mcp` 2.x,
`MCPServer`), on stdio. It keeps configs as named sessions in its memory, so a conversation can build variants and
compare them, and it gives compact JSON: units in the key names, tables cut to their top rows with the number left
out, and a `notes` field in every result with what the numbers cannot claim (which values are estimates, what was not
validated), so that the model passes the limits on instead of overclaiming.

### Install

The extra `mcp` comes with the release after 1.0.0 on PyPI. With [uv](https://docs.astral.sh/uv/) nothing has to be
installed by hand:

```bash
uvx --from "kinematics-sim[mcp]" kinematics-sim-mcp
```

or into an environment of your own: `pip install "kinematics-sim[mcp]"`, then `kinematics-sim-mcp` (or
`python -m kinematics_sim.mcp`). From the repository: `pip install maturin==1.16.0 "mcp>=2.3,<3"`, then
`maturin develop --release` in `crates/sim-py` ([Python module](python.md#install)).

A call that runs longer than 900 s is given up with a message (set `KINEMATICS_SIM_MCP_TIMEOUT_S` to change it); the
simulator cannot be stopped half-way, so its thread runs to its end in the background. Long calls report progress every
5 s to a client that asks for it.

### Client configuration

**Claude Code:**

```bash
claude mcp add kinematics-sim -e KINEMATICS_SIM_MCP_OUT_DIR="$HOME/kinematics-sim" -- uvx --from "kinematics-sim[mcp]" kinematics-sim-mcp
```

The variables are in [the output folder](#files-the-output-folder) below.

**Claude Desktop** (`claude_desktop_config.json`; Settings, Developer, Edit Config) and **Cursor** (`.cursor/mcp.json`
in a project, or `~/.cursor/mcp.json`) take the same entry:

```json
{
  "mcpServers": {
    "kinematics-sim": {
      "command": "uvx",
      "args": ["--from", "kinematics-sim[mcp]", "kinematics-sim-mcp"],
      "env": { "KINEMATICS_SIM_MCP_OUT_DIR": "C:\\Users\\me\\Documents\\kinematics-sim" }
    }
  }
}
```

With the package installed in an environment, `"command": "kinematics-sim-mcp"` (the full path to the script in that
environment) and no `args` does the same.

### Tools

| Tool | What it gives |
| --- | --- |
| `list_machines` | Every machine: id and name |
| `describe_machine` | A machine or session: name, motors and axes, the settings that matter with their labels and units, the estimates with their `[bounds]` (value, low, high), the effects on and off |
| `create_session` | A named config in the server's memory: a machine's default, TOML text, or a copy of another session |
| `list_sessions` | The sessions, their machines and how many values differ from the default |
| `get_config` | Values by dotted path, every value under a prefix (`motor.`), or the whole config as TOML |
| `set_config_values` | Values by dotted path, all or nothing; a misspelt key is refused with the keys it may have meant |
| `config_diff` | What differs from the machine's default, or from another session |
| `check_sizing` | Pass or fail per check and motor: value, limit, `margin_share` (share of the limit left, negative where exceeded) and a one-line meaning with the unit; tightest first. `bands=true` adds the range over the estimates' bounds and `worst_ok` |
| `analyze_cycle` | Per motor peak and RMS torque, peak speed, utilisation of the torque-speed curve, thermal utilisation, winding rise, RMS current, energy in and regenerated; more columns on request |
| `run_path` | A path given inline (lines, arcs, splines; the format of a [path file](user-guide.md#run-a-path-and-read-path-error)): duration, path error, tracking error, load per motor |
| `find_motor` | The catalogue search: the top N motors with drive and gearbox, passing and cheapest first, with what each fails and which of its values are estimates |
| `optimise_cycle` | A shorter cycle that passes every check: cycle time before and after and the settings changed, as values `set_config_values` takes |
| `loop_margins` | Per motor the velocity loop's crossover, phase and gain margin, peak sensitivity |
| `positioning_accuracy` | ISO 230-2 per axis and ISO 9283 at the tool (simulated) |
| `sensitivity` | Which uncertain values move the results most (one at a time, or Morris / Sobol' over all bounds): what to measure first |
| `write_report` | The sizing report as HTML or PDF, into the output folder under a file name ending in `.html` or `.pdf` |
| `export_fmu` | An FMI 2.0 or 3.0 FMU, into the output folder under a file name ending in `.fmu`, carrying the sim-fmi library the operator set up |

`session` takes a session's name or a machine id (its default config, not stored).

### Files: the output folder

A tool is called by a model, and a model can be steered by what it reads (a web page, a file, a message: prompt
injection). So the two tools that write files write only where the operator, who starts the server, says:

| Variable | Effect |
| --- | --- |
| `KINEMATICS_SIM_MCP_OUT_DIR` | The output folder (created if missing). Default: `kinematics-sim-output` in the server's working directory, which is wherever the client starts it: set it |
| `KINEMATICS_SIM_MCP_OVERWRITE` | `1` lets a tool replace an existing file; otherwise an existing name is refused |
| `KINEMATICS_SIM_FMI_BINARIES` | The sim-fmi libraries an FMU carries (`sim_fmi.dll`, `libsim_fmi.so`, `libsim_fmi.dylib`), separated by `;` on Windows and `:` elsewhere. Without it, as `sim-cli fmu` does, this platform's library beside a `sim-cli` on `PATH` (a release ships them together; `cargo build -p sim-fmi --release` builds it) |
| `KINEMATICS_SIM_MCP_TIMEOUT_S` | Seconds before a call is given up (default 900) |

The model gives a file name, or a relative path inside the folder (`variants/gantry-3kg.html`). A name that leads out
of the folder is refused: `..`, an absolute path, a drive (`C:x.html`), a network path, a link or junction inside the
folder that points out of it. So is a name with another extension, and an existing file unless overwriting is allowed.
The result gives the full path written, to show the user. The FMU's binaries never come from the call.

The resource `kinematics-sim://limits` is a summary of [what it can and cannot tell you](overview.md#what-it-can-and-cannot-tell-you);
the prompt `size_a_motor` (argument: a description of the machine) walks a model through describe, configure,
analyse, check, search, bands and sensitivity.

### A worked example

What a conversation in Claude Code looks like with the server added (the tool results are real, trimmed; the model's
wording is an example):

> **You:** The T-gantry from the simulator with 3.5 kg instead of 2 kg: do the motors still pass?

The model calls `create_session {"name": "heavier", "machine": "t-gantry"}` and then
`set_config_values {"session": "heavier", "values": {"mass.paylod_kg": 3.5}}` with a typo. The server refuses:

```text
Error executing tool set_config_values: nothing was changed. mass.paylod_kg: Configuration value is missing:
mass.paylod_kg (did you mean mass.payload_kg, mass.z_axis_kg, mass.carriage_kg?)
```

It retries with `mass.payload_kg`, then calls `analyze_cycle {"session": "heavier"}`:

```json
{"motors": [{"motor": "a", "cycle_s": 6.08, "peak_torque_nm": 0.6235, "rms_torque_nm": 0.1857,
             "peak_speed_rad_s": 200.5, "utilisation": 0.1645, "thermal_utilisation": 0.1462,
             "winding_rise_k": 1.55, "rms_current_a": 0.4129, "energy_in_j": 94.9, "energy_regenerated_j": 12.2},
            {"motor": "b", "peak_torque_nm": 0.4617, "thermal_utilisation": 0.1303, "...": "..."}],
 "notes": "utilisation and thermal_utilisation above 1 fail. ... Only one machine (a ball screw bench) was measured; ..."}
```

and `check_sizing {"session": "heavier", "bands": true, "top": 3}`:

```json
{"passed": true, "failed": [], "worst_case_failed": [],
 "rows": [{"check": "dc-link-energy", "motor": "a", "meaning": "Largest braking into the DC link [J]",
           "value": 6.205, "limit": 6.205, "ok": true, "low": 6.205, "high": 6.205, "worst_ok": true,
           "margin_share": 0.0},
          {"check": "dc-link-overvoltage", "motor": "a", "meaning": "Highest DC link voltage [V]",
           "value": 224.7, "limit": 257.6, "ok": true, "worst_ok": true, "margin_share": 0.1277, "...": "..."}],
 "row_count": 40, "rows_left_out": 37,
 "notes": "margin_share: share of the limit left (negative: exceeded by). ... worst_ok judges the worst case within the bounds. ..."}
```

> **Model:** Yes. With 3.5 kg every one of the 40 checks passes, also at the worst corner of the estimated values'
> bounds. The motors are lightly loaded (motor A at 16 % of its torque-speed curve and 15 % thermally); the rows
> closest to their limits are the DC link's (braking energy at its limit, voltage 13 % below it), not the motors.
> These numbers rest on estimated friction, damping and stiffness (this default config is a placeholder machine, not
> a product), so treat them as a comparison with the 2 kg case rather than as a guarantee.

Asked for a smaller motor, it calls `find_motor {"session": "heavier", "series": "1FK2", "top": 2}`: all 28 candidates
of the series pass, the lightest first, 1FK2203-2AK with the drive 6SL3210-5HE10-4UF0 (1.16 kg, tightest check
`settling` at 0.53, three of its values `estimated`).

### Tests

`crates/sim-py/tests/test_mcp.py` drives the server through the SDK's own client, in process for every tool and once
over stdio as a client starts it (`python -m kinematics_sim.mcp`): on the first two machines of the registry (the
T-gantry and the single axis) it checks the tools against the module's own results (verdicts, analysis, path error,
search order, margins, accuracy), the sessions and the diff, the refusal of a misspelt key with the key meant, the
all-or-nothing set, a path that leaves the travel range, the report and the FMU written into the output folder (and
`..`, an absolute path, another extension, an existing file and, where the system lets a test make one, a link out of
the folder refused; the FMU's library only from the operator's variable), the timeout, the resource and the prompt. 10 tests pass on Windows with CPython 3.14 and `mcp` 2.3.0 in about 3 minutes (16 November 2026).

Not tried: the server in Claude Desktop, Cursor or another client's own process (the tests use the SDK's client, as
the clients do, but not the clients themselves); Linux and macOS; Python 3.10 to 3.13; the HTTP transports.

## Using sim-cli from a coding agent

An agent with a shell (Claude Code, Cursor, Codex) can run `sim-cli` from a [release](install.md) or a build
(`cargo build -p sim-cli --release`, then `target/release/sim-cli`). What it needs to know:

- **Output:** results are CSV on standard output, one header line, units in the column names (`peak_torque_nm`,
  `winding_rise_k`); messages, warnings and summaries (thermal start, accuracy figures, search time) go to the error
  stream. `--out FILE` writes the CSV to a file instead.
- **Exit codes:** 0 when the command ran and everything it judged passes; 1 when it ran and printed a failing row
  (`verdicts`, `margins`, `stop`, `accuracy`, `report`): the table is whole, read it for which check says `FAIL`, and
  do not treat the 1 as a crash; 2 for a refused option, config, key or value, with one line on the error stream saying
  what (`Configuration value is missing: mass.paylod_kg`, `This command has no such option: --band`). A result that is
  `inf` or `nan` comes with a note on the error stream: the loop is unstable, `sim-cli margins MACHINE` measures it.
- **Values:** `--set path=value` (repeatable, dotted path as in the [config reference](config-reference.md), a misspelt
  key is refused), `--config FILE` for a whole config, `--motor ARTICLE`, `--gearbox`, `--drive` for catalogue parts.
- **Help:** `sim-cli help` (or `sim-cli` alone) prints the usage of every command in one paragraph and exits 2; there is
  no `--help` and no help per command: the [user guide](user-guide.md) has the details.
- **Keep the output small:** write long tables with `--out FILE` and read the head; pick columns with `cut -d, -f...`.
  `run` writes every signal (115 columns on the T-gantry) at every 16th base step: use `--every 160` or more, or
  `analyze` for the summary. Piping into `head` can end `sim-cli` with a "failed printing to stdout" panic once
  `head` closes the pipe: harmless, but write to a file to keep the log clean.

The commands that answer the common questions (outputs from a Windows build of 16 November 2026, trimmed; the times
were taken on a 16-thread machine with other work running and vary with load):

| Question | Command | Time |
| --- | --- | --- |
| Which machines are there? | `sim-cli list` | instant |
| Does it pass? | `sim-cli verdicts t-gantry [--set mass.payload_kg=3.5]` | 5 to 45 s |
| Does it pass with the estimates at their bounds? | `sim-cli verdicts t-gantry --bands` | 80 s |
| How loaded is each motor? | `sim-cli analyze t-gantry` | 5 to 45 s |
| Which catalogue motor? | `sim-cli search t-gantry --series 1FK2` | 15 s |
| How much faster can the cycle be? | `sim-cli optimise t-gantry --runs 8` | 60 s |
| Is the velocity loop stable enough? | `sim-cli margins t-gantry` | 3 s |
| How accurate is it at rest? | `sim-cli accuracy t-gantry --points 3 --repeats 2` | 7 s |
| What should I measure first? | `sim-cli sensitivity single-axis` | 13 s |
| How well does it follow a path? | `sim-cli analyze t-gantry --path corner.path` | 13 s |
| An FMU for Simulink or Modelica | `sim-cli fmu t-gantry --out t.fmu --binary sim_fmi.dll` | 1 s |

```text
$ sim-cli list
t-gantry	T-gantry
single-axis	Single axis
delta-d3	Delta picker (3 arms)
...

$ sim-cli verdicts t-gantry
check,motor,value,limit,result
motor-peak,a,0.1482,1.000,ok
motor-thermal,a,1.279,100.0,ok
motor-s1,a,0.1688,1.270,ok
...                                      (40 rows)

$ sim-cli verdicts t-gantry --set mass.payload_kg=40 | grep -v ',ok$'
check,motor,value,limit,result
braking-resistor-continuous,b,26.86,20.00,FAIL
settling,,0.0003396,0.0001000,FAIL
belt-least-tension,,0,0,FAIL

$ sim-cli verdicts t-gantry --bands
check,motor,value,limit,result,low,high,worst_case
motor-peak,a,0.1482,1.000,ok,0.1436,0.1554,ok
motor-thermal,a,1.279,100.0,ok,1.205,1.453,ok
...

$ sim-cli analyze t-gantry --set mass.payload_kg=3.5 --out load.csv
$ cut -d, -f1-7,11 load.csv
motor,cycle_s,peak_torque_nm,rms_torque_nm,peak_speed_rad_s,utilisation,thermal_utilisation,winding_rise_k
a,6.080000,0.630199,0.188265,200.5251,0.1664,0.1482,1.59
b,6.080000,0.467014,0.168400,200.5256,0.1230,0.1326,1.27

$ sim-cli search t-gantry --series 1FK2 --out search.csv
Not simulated (a part the config cannot take, or a cycle that does not run; the reason is in the table): 0 / 28 candidates
$ cut -d, -f2,4,7,9-11 search.csv | head -3
motor,drive,mass_kg,result,tightest_check,tightest_utilisation
1FK2203-2AK,6SL3210-5HE10-4UF0,1.16,ok,settling,0.4324
1FK2103-2AH,6SL3210-5HE10-4UF0,1.18,ok,gearbox-radial-load,0.4302

$ sim-cli optimise t-gantry --runs 8
setting,before,after
cycle_s,6.0800,3.6220
axis.x.max_velocity_m_s,0.8,1.44
axis.z.max_velocity_m_s,0.4,0.72
effects.corner_blending,false,true
...

$ sim-cli margins t-gantry
run,motor,crossover_hz,phase_margin_deg,gain_margin_db,peak_sensitivity,peak_frequency_hz,play_margin_db,result
,a,185.6,54.9,9.19,1.882,277.3,,ok
,b,179.8,52.1,9.09,1.878,277.3,,ok

$ sim-cli accuracy t-gantry --points 3 --repeats 2 --out acc.csv
Accuracy along ISO 230-2, axis x [m]: B 2.657e-5, R 2.658e-5, E 5.119e-5, M 3.850e-5, A 5.120e-5 (...)
Accuracy at the tool along ISO 9283, over the axes in metres: AP 5.625e-5, RP 1.569e-7, vAP 2.657e-5 (...)

$ sim-cli sensitivity single-axis | cut -d, -f1-8 | head -3
parameter,value,low,high,peak_torque_low,peak_torque_high,rms_torque_low,rms_torque_high
friction_x.viscous_n_s_m,20.0,10.0,40.0,-0.0443,+0.0883,-0.1381,+0.3016
gearbox.ratio,5.0,4.0,6.0,+0.1650,-0.0906,+0.1631,-0.0982

$ sim-cli analyze t-gantry --path corner.path --out path.csv
$ cut -d, -f1-3,16-18 path.csv
motor,cycle_s,peak_torque_nm,path_error_peak_m,path_error_rms_m,tracking_error_peak_m
a,2.128000,0.546983,1.324e-4,4.412e-5,1.779e-4
b,2.128000,0.303047,1.324e-4,4.412e-5,1.779e-4

$ sim-cli fmu t-gantry --out t.fmu --binary target/release/sim_fmi.dll
```

`corner.path` is [`examples/corner.path`](../examples/corner.path) (a line, an arc and a line). In `sensitivity` the
`_low` and `_high` columns are relative changes (`+0.0883`: 8.8 % more); in `accuracy` the figures are in metres.
Every column is explained in the [user guide](user-guide.md).
