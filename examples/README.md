# Examples

One small example per task of the [user guide](../docs/user-guide.md), each with the command that runs it and what to
look at in its output. Run them from the repository's root, with `sim-cli` built (`cargo build --release -p sim-cli`,
then `target/release/sim-cli`, or `cargo run -q --release -p sim-cli -- ...` in place of `sim-cli`). Every `sim-cli`
command in a `bash` block below is run by the test `crates/sim-cli/tests/examples.rs`, and the Python script by
`crates/sim-py/tests/test_examples.py`, so they keep working as the program changes.

Where a task needs a whole config, the example takes a shipped one and changes it with `--set`: a config file has to be
complete (some 700 lines, [config reference](../docs/config-reference.md#rules)), and a copy here would fall behind the
shipped one.

## A path on two axes

[`corner.path`](corner.path): line, arc, line on the T-gantry, the first path printed in the README.

```bash
sim-cli analyze t-gantry --path examples/corner.path
```

Look at `cycle_s` (the path plus 0.3 s of rest) and `path_error_peak_m`, the tool tip's largest distance from the path.

[`curve.path`](curve.path): a line, a spline through two via points, a line.

```bash
sim-cli analyze t-gantry --path examples/curve.path
```

The same columns; the curve is slow where it joins the lines, because the curvature steps there.

## A path on three axes

[`delta-3d.path`](delta-3d.path): the delta picker D3 goes down, round a quarter arc in the vertical plane, across and
through a curve in space.

```bash
sim-cli analyze delta-d3 --path examples/delta-3d.path
```

One row per arm motor: `peak_torque_nm` and `utilisation` show which arm the path loads most.

## A G-code program

[`pocket.nc`](pocket.nc): the corner of `corner.path` in G-code (G18 plane, mm, feed per minute), read by its extension.

```bash
sim-cli analyze t-gantry --path examples/pocket.nc
```

`cycle_s` follows from the feeds: F is the speed along the path, with a stop between moves at different feeds.

## A trajectory file

[`move.traj`](move.traj): poses at times on the single belt axis; the controller runs a cubic curve through them.

```bash
sim-cli analyze single-axis --trajectory examples/move.traj
```

`tracking_error_peak_m` is how far the tool lags the poses at their times.

## A cam table

[`press-cam.csv`](press-cam.csv): a servo press cam for the slider-crank, slave = master + 0.6 sin(master), in place of
the config's cam segments.

```bash
sim-cli analyze slider-crank --set cam.run=true --set 'cam.table_file="examples/press-cam.csv"'
```

`peak_speed_rad_s` is about 100.5: the slope's peak 1.6 times the master's pi rad/s (30 strokes a minute) times the
gearbox's 20.

## Obstacles and a crash

A wall in the single axis's way, given with `--set obstacles=...` (in a config file it is the key `obstacles`), with the
crash and the drives' monitoring on.

```bash
sim-cli run single-axis --move 0.9 --time 1.5 --every 100 --set effects.crash=true --set effects.drive_monitoring=true --set 'obstacles=[{name="wall",centre_m=[0.75,-0.03,0.06],size_m=[0.1,0.2,0.2]}]'
```

The lines on the error stream: the planned clearance, the first contact (0.47 s), the contact force and how the drives
tripped and stopped. The signals go to the standard output as CSV (`--out FILE` writes them to a file).

## Pick from a moving conveyor

The shipped conveyor config of the planar delta D2: a part a second on a belt at 0.3 m/s.

```bash
sim-cli analyze delta-d2 --config crates/machine-delta-d2/conveyor.toml
```

The line `Conveyor:` says the picks a minute (60, all the belt brings), the tracking error during the grip and the parts
missed (0). Add `--set conveyor.pitch_s=0.5` and the delta misses parts.

## A duty mix for the motor search

The D2's own cycle three times to one conveyor shift, every 1FK2105 motor with its drive.

```bash
sim-cli search delta-d2 --config crates/machine-delta-d2/default.toml --config crates/machine-delta-d2/conveyor.toml --weights 3,1 --series 1FK2105
```

One row per candidate, passing and cheapest first: `thermal_utilisation` is the heat over the mix, the peaks the worst
of the two cycles. Leave out `--series` for every motor of the catalogue (some 40 s).

## A drive trace with its map file

The synthetic trace of [`datasets/example-synthetic-belt-axis`](../datasets/example-synthetic-belt-axis/): positions as
encoder counts, speed in 1/min; its `map.toml` says which column is which and how to convert it.

```bash
sim-cli replay single-axis --trace datasets/example-synthetic-belt-axis/trace.csv --map datasets/example-synthetic-belt-axis/map.toml --set effects.payload=false
```

Per motor the RMS and peak difference between trace and simulation, in torque and following error; small here, since
the trace was made by the simulator. `fit` and `identify` take the same two options.

## A `--set` sweep in Python

[`payload_sweep.py`](payload_sweep.py): the single axis at three payloads with the module `kinematics_sim`
([docs/python.md](../docs/python.md)).

```bash
python examples/payload_sweep.py
```

One CSV row per payload: peak and RMS torque rise with it, and `all_checks_pass` says whether every check passes.

## A task on several kinematics

[`pick-place-task.toml`](pick-place-task.toml) is the task file of `sim-cli compare-kinematics` (user guide, "Which
kinematics for a task"). It runs a motor search per machine, minutes in all, so the test leaves it out.
