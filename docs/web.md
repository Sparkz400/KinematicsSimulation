# The simulator in the browser (WebAssembly): 3D view and sizing check

The core, the machines, the catalogue and the analysis build for `wasm32-unknown-unknown`. `crates/sim-wasm` wraps them
in six C-ABI functions, and `crates/sim-wasm/web/index.html` is a static page that loads the module: pick a machine,
edit its config (TOML), and the page shows the machine in 3D running its cycle, and runs a sizing check that shows every
verdict and the load per motor. Nothing leaves the browser; there is no server beyond one that hands out two files.

**Live page:** https://sparkz400.github.io/KinematicsSimulation/, built from `main` by the workflow
[`pages.yml`](../.github/workflows/pages.yml) on every push.

## Build and open

```bash
rustup target add wasm32-unknown-unknown
cargo build -p sim-wasm --release --target wasm32-unknown-unknown
cp target/wasm32-unknown-unknown/release/sim_wasm.wasm crates/sim-wasm/web/
python -m http.server 8765 -d crates/sim-wasm/web
```

Then open http://localhost:8765/. Browsers load WebAssembly only over HTTP, not from a `file://` page, so any static file
server will do. To publish it, copy `index.html` and `sim_wasm.wasm` to any static host. The CI job `wasm` builds both
and keeps them as the artifact `sizing-page`; the workflow `pages.yml` builds them the same way and publishes them on
GitHub Pages.

No wasm-bindgen and no JavaScript toolchain: the page writes a request into the module's memory and reads JSON back, so
`cargo` is the only build tool. The module is about 2.0 MB (1,979,671 bytes before the 3D view, 2,001,607 bytes with it,
14 November 2026).

## What it computes

The same computation as `sim-cli verdicts` and `sim-cli analyze` for the cycle of the config: one pass measured after the
machine has settled into the cycle, judged against the motor and drive data and the `[ratings]` of the config. The
numbers agree with the command line's to four or five significant digits, not bit for bit: WebAssembly computes `sin`,
`exp` and the like with Rust's libm, the native build with the platform's maths library, and the simulation amplifies
the differences in the last bit. One measured example: a peak torque of 0.4600761 N m in the browser against 0.4601061
N m from `sim-cli`. A verdict close to its limit can therefore come out the other way. Parts
named under `[parts]` come from the built-in catalogue. On the T-gantry's default config the check takes 0.9 s in Chrome
on the development machine (13 November 2026).

| request (`call(kind, ...)`) | lines of the request | answer (JSON) |
| --- | --- | --- |
| 0 | language (`en`, `de`) | every machine: `[{"id", "name"}]` |
| 1 | machine id | `{"config"}`: the default config as TOML |
| 2 | machine id, language, then the config as TOML (empty: the default) | `machine`, `cycle_s`, `tripped`, `warnings`, `motors` (peak and RMS torque, peak speed, use of the torque-speed curve, thermal utilisation, winding rise), `verdicts` (check, label, motor, value, limit, ok) |

An error comes back as `{"error": "..."}` with the simulator's message, and `call` returns 1.

## The 3D view

Request 3 (machine id, language, config as in request 2) sets the machine at rest at its start pose, starts its cycle and
answers `{"dt_s", "centre", "radius"}`: the base step and the sphere round the scene at the start, which the camera is
fitted to. Each frame the page then calls `frame(seconds)`, which runs the simulation on by that many seconds (whole base
steps) and returns a pointer to `frame_len()` 32-bit floats, no JSON:

| floats | what |
| --- | --- |
| 6, the head | simulated time (s), the cycle time (s, the latest whole pass from one start of the cycle to the next; NaN before one), the tool's x, y, z in the scene (m; NaN where the machine has no mapping), the number of parts |
| 20 per part | the mesh (0: a cube of side 1, 1: a cylinder of diameter and length 1 along y, both centred on the origin), the 4 × 4 matrix that takes the mesh to the part (column by column, scene coordinates, z up), the colour (red, green, blue in 0..1) |

The matrix carries the part's shape with its pose: a belt span (a segment) changes its length as the machine moves, and
the number of belt strips can change from one frame to the next, so the page draws whatever the frame holds. Colours are
the desktop app's (`scene::part_color`): the parts' own, belt tension and warmth as there.

The page draws with plain WebGL2 written in the page, no library: nothing is loaded from elsewhere, and two meshes, one
light and an orbit camera are a hundred lines. Drag to turn, wheel to zoom; Pause/Play and Restart (which takes the config
in the editor). The page runs the simulation at real time; a machine the browser cannot step that fast runs slower, since
a frame runs at most 50 ms of simulated time. The legend shows the simulated time, the cycle time and the tool position;
an orange cube marks the tool.

## Differences from the native build

WebAssembly starts no threads. The analyses that share the processors natively (error bands, sampled runs, the
catalogue search, the stability margins, the comparison of an effect on and off) run one after another there; the code
falls back to one thread whenever `std::thread::available_parallelism` says one or knows none. The page offers only the
single sizing check, which needs one run.

## Tests

`cargo test -p sim-wasm --release` runs the module's requests natively: every machine listed, the sizing check giving the
same checks, values and cycle time as the analysis and verdict functions of the command line, a config from the page
read as a config file (a misspelt key refused by name), and the C functions handing back JSON. For the 3D view: the frame
of every machine against its scene from a native simulation, at the start pose and after 0.5 s of its cycle (mesh,
centre, stretch along each axis and colour of every part, the tool position), two records and a bounding box worked out
by hand, and the view's cycle time against the sizing check's. The page itself was run
in Chrome by hand: the T-gantry's default config passes every check with a cycle of 6.080 s.
