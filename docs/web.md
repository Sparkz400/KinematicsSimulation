# The simulator in the browser (WebAssembly): 3D view, plots and sizing check

The core, the machines, machine files, the catalogue and the analysis build for `wasm32-unknown-unknown`.
`crates/sim-wasm` wraps them in a few C-ABI functions, and `crates/sim-wasm/web/index.html` is a static page that loads
the module: pick a machine (registered, one of the machine files of `machines/`, or a TOML machine file opened or pasted
into the editor), edit its config, run its cycle or a path, and the page shows the machine in 3D with its obstacles and
the tool's trail, time plots of torque, speed and following error per motor, and a sizing check with every verdict,
every column of `sim-cli analyze` and the operating points over the motor's torque-speed curve. A frequency sweep and a
fit of masses and friction to a measured trace are there too. Nothing leaves the browser; there is no server beyond one
that hands out two files. The page is in English and German (the switch at the top; the browser's language first).

**Live page:** https://sparkz400.github.io/KinematicsSimulation/, built from `main` by the workflow
[`pages.yml`](../.github/workflows/pages.yml) on every push.

## Build and open

```bash
rustup target add wasm32-unknown-unknown
cargo build -p sim-wasm --profile wasm --target wasm32-unknown-unknown
cp target/wasm32-unknown-unknown/wasm/sim_wasm.wasm crates/sim-wasm/web/
python -m http.server 8765 -d crates/sim-wasm/web
```

Then open http://localhost:8765/. Browsers load WebAssembly only over HTTP, not from a `file://` page, so any static file
server will do. To publish it, copy `index.html` and `sim_wasm.wasm` to any static host. The CI job `wasm` builds both
and keeps them as the artifact `sizing-page`; the workflow `pages.yml` builds them the same way and publishes them on
GitHub Pages.

The profile `wasm` (root `Cargo.toml`) is the release profile with link-time optimisation and one codegen unit: on the
T-gantry some 5 % faster and 10 % smaller than `--release` (14 November 2026). No wasm-bindgen, no wasm-opt and no
JavaScript toolchain: the page writes a request into the module's memory and reads JSON back, so `cargo` is the only
build tool. The module is 2,887,035 bytes (14 November 2026, with the machine files built in).

## Speed

The module runs in two Web Workers, so that the page stays responsive: one steps the 3D view, the other runs the sizing
check, the sweep and the fit, which before blocked the page for seconds. Each worker has its own copy of the module and
so its own machine files; the page hands an opened file to both. The view's worker answers one request at a time with
all the simulated time owed since the last one (at most 0.25 s, so a machine too slow for real time falls behind
instead of piling up work), and hands the frame back as a transferred copy.

Measured on 14 November 2026 on the development machine while other builds ran: the T-gantry's default config
simulates 10 s in 0.70 s of processor time in the module (Node 24, the same wasm), about 14 times real time with the
plots' samples; the module of the release profile without LTO took 0.86 to 1.05 s. In headless Chrome (software WebGL)
the page ran the T-gantry at 1.01 times real time, the sizing check alongside taking 4.0 s. The earlier page's third of
real time could not be reproduced: the module itself was some ten times faster than real time then too; what the page
did on the main thread around it (the sizing check blocked it, and every frame waited for the step) is now in workers.

## Requests

The page writes a request into memory from `alloc`, calls `call(kind, ptr, len)` and reads the answer, JSON, from
`out_ptr()` and `out_len()`; `call` returns 0 for a result and 1 for `{"error": "..."}` with the simulator's message.

A request has lines: the machine, the language (`en`, `de`) with options after it separated by spaces, then the config
as TOML (empty: the machine's default), and after a line `---` the text an option names (a path, a trace). A machine
is a registered id (`t-gantry`), a machine file of `machines/` built into the module (`file:corexy.toml`), or one the page
handed in (`upload:<file name>`).

| kind | request | answer (JSON) |
| --- | --- | --- |
| 0 | language | every machine: `[{"id", "name", "file"}]`, the registered ones, then the machine files |
| 1 | machine | `{"config"}`: the default config (of a machine file: the file) |
| 2 | machine, language and options, config | the sizing check: `machine`, `cycle_s`, `tripped`, `warnings`, the machine-wide columns of `sim-cli analyze`, `motors` (every per-motor column of `sim-cli analyze` under its name, `points` the operating points, `curve` the peak torque-speed outline, `s1` the continuous rating), `verdicts` (check, label, motor, value, limit, ok) |
| 3 | machine, language and options, config | starts the 3D view: `{"dt_s", "centre", "radius", "motors", "sample"}` |
| 4 | file name, language, machine file (TOML) | `{"id", "name", "config"}`: the file checked as `sim-cli --machine-file` checks it (an error names the file and the line) |
| 5 | machine, language and options, config | `{"motor", "amplitude_nm", "points": [[frequency Hz, gain rad/s per N m, phase °]]}` as `sim-cli sweep` |
| 6 | machine, language, config, `---`, trace (CSV) | `{"rows": [[quantity, of, config, fitted, standard error]], "advice"}` as `sim-cli fit --trace` without `--map` |
| 7 | language, then translation keys, one a line | `{key: text}`: the page's texts (`crates/sim-wasm/i18n/*.ftl`) and those keys from the simulator's translations |

Options: `path` runs the path after `---` (format: `sim_components::path`, the file `sim-cli --path` reads) from the
start pose instead of the cycle, timed within the axis limits or with `time-optimal` as fast as the motors allow
(kinds 2 and 3); kind 2 warns, as `sim-cli` does, where the planned path is off the given lines and arcs. `motor=K`,
`from=HZ`, `to=HZ`, `count=N`, `amplitude=NM` set the sweep as `sim-cli sweep` takes them (the page asks for 40 points
instead of 120: each is a run of its own).

## What it computes

The same computation as `sim-cli verdicts` and `sim-cli analyze` for the cycle of the config or the path: one pass
measured after the machine has settled into the cycle, or the path run through, judged against the motor and drive data
and the `[ratings]` of the config, with every column `sim-cli analyze` writes (housing rise, drive overload, RMS and peak
current, the energies, DC link, lives, belt tension, path and tracking errors, the warm start's change). What the page
does not give: `analyze`'s `--breakdown` and `--budget` (the effect views; the page runs without them, as `verdicts`
does), the error bands and samples of `verdicts --bands`, and the catalogue search.

The numbers agree with the command line's to four or five significant digits, not bit for bit: WebAssembly computes
`sin`, `exp` and the like with Rust's libm, the native build with the platform's maths library, and the simulation
amplifies the differences in the last bit. One measured example: a peak torque of 0.4600761 N m in the browser against
0.4601061 N m from `sim-cli`. A verdict close to its limit can therefore come out the other way. Parts named under
`[parts]` come from the built-in catalogue.

The operating points are those of the analysis (motor speed and torque, sampled once a millisecond), over the outline of
`DriveParams::torque_curve` (peak torque of motor and drive both ways, at its corners and 40 speeds between, so a
field-weakened curve bends) and the S1 box (rated torque up to the rated speed), as in the app.

## The 3D view, plots and trail

Request 3 sets the machine at rest at its start pose, starts its cycle or path and answers the base step, the sphere
round the scene to fit the camera to, the motors and the floats per sample. Then `frame(seconds)` runs the simulation on
by that many seconds (whole base steps) and returns a pointer to `frame_len()` 32-bit floats, no JSON:

| floats | what |
| --- | --- |
| 8, the head | simulated time (s), the cycle time (s, the latest whole pass from one start of the cycle to the next; NaN before one), the tool's x, y, z in the scene (m; NaN where the machine has no mapping), the number of parts, of obstacles, of samples |
| 20 per part | the mesh (0: a cube of side 1, 1: a cylinder of diameter and length 1 along y, both centred on the origin), the 4 × 4 matrix that takes the mesh to the part (column by column, scene coordinates, z up), the colour (red, green, blue in 0..1) |
| 20 per obstacle | as a part, mesh 2: the cube drawn see-through, stretched to the obstacle's box; grey, red while a part touches it |
| `sample` per sample | one per millisecond since the last frame: time, the tool's x, y, z, its distance from where the plan has it then (m), then per motor the lowest and highest torque, torque command (N m), speed, speed command (rad/s) and following error (rad) over the base steps of that millisecond |

`frame` refuses a time that is not finite, below 0 or above 60 s: it returns null and leaves the error as the answer
at `out_ptr`. (An infinite time ran the step count up to the largest integer and hung the page.)

The matrix carries the part's shape with its pose: a belt span changes its length as the machine moves, and the number of
belt strips can change from one frame to the next, so the page draws whatever the frame holds. Colours are the desktop
app's (`scene::part_color`).

The page draws with plain WebGL2 written in the page, no library. Drag to turn, wheel to zoom; Pause/Play and Restart
(which takes the config, the motion and the path in the editor). The legend shows the simulated time, the cycle time and
the tool position; an orange cube marks the tool. The trail is the tool's path, one point a millisecond, coloured by its
distance from the plan (the app's "following error" trail) on a fixed scale set on the page: blue at none, yellow at
half, red at the value set (2 mm unless changed), magenta beyond it. The last 40,000 points are kept.

The time plots show the last few seconds of the view's run (the window is set on the page): torque with the torque
command dashed, speed with the speed command dashed, and the following error, one colour per motor. The page draws them
with its own canvas plotter of some hundred lines, no library. Each pixel column draws the lowest and the highest value
of the samples it spans, and each sample holds the lowest and highest over its base steps, so a peak of one base step
stays visible however long the window. The same plotter draws the operating points and the sweep.

## Differences from the native build

WebAssembly starts no threads. The analyses that share the processors natively (error bands, sampled runs, the
catalogue search, the stability margins, the comparison of an effect on and off) run one after another there; the code
falls back to one thread whenever `std::thread::available_parallelism` says one or knows none. The page offers the
sizing check, the sweep and the fit, each one run or a set of short runs. Machine files are TOML; a JSON machine file
is read by `sim-cli` and the app but not yet by the page. The fit takes a trace in the simulator's own columns, not a
vendor export with its mapping file, and fits masses and friction only (no stiffness or sweep fit).

## Tests

`cargo test -p sim-wasm --release` runs the module's requests natively: every machine listed, the sizing check giving the
same checks, values and cycle time as the analysis and verdict functions of the command line, a config from the page
read as a config file (a misspelt key refused by name), and the C functions handing back JSON. Machine files: the eight
of `machines/` listed and run, one handed in giving the same sizing check as the built-in copy, a misspelt key refused
with the file and its line. Paths: the sizing check of a path against `analyse_trajectory` of the same planned poses,
time-optimal too, the view standing where a native run of the path stands, an unreadable and an empty path refused.
The frame: refused for an infinite, a NaN, a negative and an over-long time, and running afterwards. For the 3D view:
the frame of every machine against its scene from a native simulation, at the start pose and after 0.5 s of its cycle
(mesh, centre, stretch along each axis and colour of every part, the tool position), two records and a bounding box and
an obstacle's record worked out by hand, an obstacle of the config in the frame, the view's cycle time against the
sizing check's, and the samples of 0.5 s (500 at 1 ms, with the highest torque and lowest following error of a native
run step by step). The sizing check's analyze columns against the native analysis, the torque-speed curve's corners by
hand, the sweep against `frequency_sweep`, a trace without the simulator's columns refused, and the page's texts in both
languages with every English key in German.

The page itself was run in headless Chrome (puppeteer-core driving the installed Chrome, 14 November 2026): the
T-gantry's cycle at real time with its plots, the sizing check passing every check with a cycle of 6.080 s; in German,
the CoreXY machine file running a path of a line, an arc and a line with its trail, its sizing check (1.661 s) and a
40-point sweep (22 s).
