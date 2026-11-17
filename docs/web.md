# The simulator in the browser (WebAssembly): 3D view, plots and sizing check

The core, the machines, machine files, the catalogue and the analysis build for `wasm32-unknown-unknown`.
`crates/sim-wasm` wraps them in a few C-ABI functions, and `crates/sim-wasm/web/index.html` is a static page that loads
the module: pick a machine (registered, one of the machine files of `machines/`, or a TOML or JSON machine file opened or pasted
into the editor), edit its config, run its cycle or a path, and the page shows the machine in 3D with its obstacles and
the tool's trail, time plots of torque, speed and following error per motor, and a sizing check with every verdict,
every column of `sim-cli analyze` and the operating points over the motor's torque-speed curve. A frequency sweep and a
fit of masses and friction to a measured trace are there too. Nothing leaves the browser; there is no server beyond one
that hands out two files. The page is in English and German (the switch at the top; the browser's language first).

It is the workbench without an install: every machine of the registry and every machine file of `machines/` in the
selector; a form over the config (the motor, supply, gearbox, drive, belt and axis from the catalogue, the masses and
payload, the axes' profile limits, the velocity loop's gains, and every other number behind a filter), which writes a
change into the config's own TOML with its comments kept; the config saved as and opened from a TOML file; the verdict
table of `sim-cli verdicts` with each check's margin; scopes of position, speed, torque, current, following error and
winding temperature rise per motor over one pass of the cycle or the run of a path, with zoom and the values under the pointer; and a share
link that carries the changed values in the URL's fragment.

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
| 2 | machine, language and options, config | the sizing check: `machine`, `cycle_s`, `tripped`, `warnings`, the machine-wide columns of `sim-cli analyze`, `motors` (every per-motor column of `sim-cli analyze` under its name, `points` the operating points, `curve` the peak torque-speed outline, `s1` the continuous rating), `verdicts` (check, label, motor, value, limit, margin, ok) |
| 3 | machine, language and options, config | starts the 3D view: `{"dt_s", "centre", "radius", "motors", "sample"}` |
| 4 | file name, language, machine file (TOML, or JSON by the name's extension `.json`) | `{"id", "name", "config"}`: the file checked as `sim-cli --machine-file` checks it (an error names the file and the line; a JSON file is converted to TOML first, as `sim_machines::load_file` does) |
| 5 | machine, language and options, config | `{"motor", "amplitude_nm", "points": [[frequency Hz, gain rad/s per N m, phase °]]}` as `sim-cli sweep` |
| 6 | machine, language and options, config, `---`, trace, a line `%%%`, mapping file, a line `%%%`, measured sweep (each may be empty, not both trace and sweep) | `{"rows": [[quantity, of, config, fitted, standard error]], "advice", "units"}` as `sim-cli fit`: the trace in the simulator's own columns, or through the mapping file (`--map`); option `stiffness` (`stiffness-key=` for another key than the belts') as `--stiffness`; the sweep with `motor=K`, `amplitude=NM` as `--sweep` |
| 7 | language, then translation keys, one a line | `{key: text}`: the page's texts (`crates/sim-wasm/i18n/*.ftl`) and those keys from the simulator's translations |
| 8 | machine, language, config | the form: `params` (`path`, `group`, `label`, `value` as TOML text: every number but `[effects]` and `[bounds]`, labelled as the app's parameter panel) and `parts` (`key`, `value`, `options`: the supplies, the motors of the config's supply, the gearboxes that fit the named motor, the drives of the supply, the belt axes) |
| 9 | machine, language, config, `---`, lines `path = value` | `{"config", "changed", "compact"}`: the values set over their own text in the config (comments and layout kept; an unknown key or another type refused, as `Config::set_text` refuses it), every value that differs from the machine's default as a line `path = value`, and whether those lines on the default give the config back |
| 10 | machine, language, config | the scopes: `step_s`, `time`, `signals` (`id`, `label`, `unit`) and per motor `data`, one array per signal: `angle`, `speed`, `torque`, `current`, `following_error`, `winding_temperature` of one pass of the cycle as `record_cycle` (`sim-cli compare`) keeps it, or with the option `path` of the path's run from rest to 0.3 s after its end, once a millisecond, 6 significant digits |

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

The verdict table prints value and limit as `sim-cli verdicts` does (four significant digits, no exponent) and
"Save as CSV" writes its lines `check,motor,value,limit,result`. The margin is the sizing report's: what is left of the
limit, 1 − value / limit, or 1 − limit / value for a check that must stay above its limit (`analysis::utilisation`);
a check passes exactly where it is not below 0, and the page marks one below 10 % in red.

## Scopes, the form and the share link

"Run sizing check" runs the scopes after the verdicts: the config's cycle once more, recorded over its second pass as
`sim-cli compare` records it, one row a millisecond (a pass of more than 30 s keeps every n-th row, 30,000 at most).
One plot a signal, every motor in its colour. Drag across a scope to zoom into that span, the wheel zooms about the
pointer, a double-click or "Reset zoom" shows the whole pass; the pointer's time and every motor's value at it stand in
the corner of each scope. Each pixel column draws the lowest and highest value of its rows, as the time plots do.
With a path instead of the cycle they record the path's run from the start pose, from its start to 0.3 s after its end (as the sizing check runs it).

The form is request 8 over the config in the editor. A field changed goes to request 9, which writes the value over its
own text in the config (the app's way of saving a config over its file), and the 3D view restarts with it; the sizing
check is run again by hand. A part's field takes any name and offers the catalogue's (a gearbox only for a named
motor); empty keeps the config's own values. "Open config…" reads a TOML config into the editor, "Save config" writes
the editor's text as `<machine>.toml`.

The share link is the page's address with a fragment (after `#`, which browsers do not send to a server):
`m=<machine>`, then `set=<path> = <value>` for every value that differs from the machine's default, or, where those
lines do not give the config back (a changed array, such as a cycle segment), `toml=` the whole config deflated and in
base64url; `path=` and `optimal=1` for a path. The page keeps its address up to date as the config changes, and
"Copy share link" copies it; opening it sets machine, config and path. A machine file opened from disk has no link: its
text is not in the module of the one who opens the link.

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
catalogue search, the stability margins, the comparison of an effect on and off, the optimiser's candidates) run one
after another there; the code
falls back to one thread whenever `std::thread::available_parallelism` says one or knows none. The page offers the
sizing check, the sweep and the fit, each one run or a set of short runs. Machine files are TOML or JSON. The fit takes
a trace in the simulator's own columns or a vendor export through its mapping file (opened first; a UTF-16 file such as
a CODESYS `.trace` is read by its byte order mark), and fits masses and friction, with the box ticked the stiffness from
the trace's own moves, and from a measured sweep (`frequency_hz`, `gain_rad_s_per_nm`) the load's resonance on the motor
chosen for the sweep.

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
hand, the sweep against `frequency_sweep`, a trace without the simulator's columns refused, a JSON machine file giving
the sizing check of its TOML text, the scopes of a path running to 0.3 s after its end, the fit of the dataset
`example-synthetic-belt-axis` through its mapping file number for number as `import` and `fit_rigid` (with the stiffness
option as `fit_move_stiffness`), a measured sweep fitted as `fit_sweep` fits it, and the page's texts in both
languages with every English key in German. The workbench: every verdict's margin (one by hand, and a check passing
exactly where its margin is not below 0), the scopes against `record_cycle`'s rows, the form's values, labels and
catalogue names, a change written with every comment of the default kept and refused for an unknown key or another
type, the share link's lines giving the same config back and an array's change making it carry the whole text. And
the built `sim_wasm.wasm` run in Node against the native numbers (`the_wasm_build_agrees_with_the_native_numbers`):
verdicts' values, limits and margins, cycle time and the scopes' peak and RMS per signal of the T-gantry and the CoreXY
file to 1e-4 relative; it passes with a note where the module is not built or Node is missing (CI's `wasm` job builds
the module before its tests).

The page itself was run in headless Chrome (puppeteer-core driving the installed Chrome, 14 November 2026): the
T-gantry's cycle at real time with its plots, the sizing check passing every check with a cycle of 6.080 s; in German,
the CoreXY machine file running a path of a line, an arc and a line with its trail, its sizing check (1.661 s) and a
40-point sweep (22 s).
