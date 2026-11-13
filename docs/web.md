# Sizing check in the browser (WebAssembly)

The core, the machines, the catalogue and the analysis build for `wasm32-unknown-unknown`. `crates/sim-wasm` wraps them
in four C-ABI functions, and `crates/sim-wasm/web/index.html` is a static page that loads the module and runs a sizing
check in the browser: pick a machine, edit its config (TOML), and the page runs the cycle and shows every verdict and the
load per motor. Nothing leaves the browser; there is no server beyond one that hands out two files.

## Build and open

```bash
rustup target add wasm32-unknown-unknown
cargo build -p sim-wasm --release --target wasm32-unknown-unknown
cp target/wasm32-unknown-unknown/release/sim_wasm.wasm crates/sim-wasm/web/
python -m http.server 8765 -d crates/sim-wasm/web
```

Then open http://localhost:8765/. Browsers load WebAssembly only over HTTP, not from a `file://` page, so any static file
server will do. To publish it, copy `index.html` and `sim_wasm.wasm` to any static host. The CI job `wasm` builds both
and keeps them as the artifact `sizing-page`.

No wasm-bindgen and no JavaScript toolchain: the page writes a request into the module's memory and reads JSON back, so
`cargo` is the only build tool. The module is about 1.1 MB.

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

## Differences from the native build

WebAssembly starts no threads. The analyses that share the processors natively (error bands, sampled runs, the
catalogue search, the stability margins, the comparison of an effect on and off) run one after another there; the code
falls back to one thread whenever `std::thread::available_parallelism` says one or knows none. The page offers only the
single sizing check, which needs one run.

## Tests

`cargo test -p sim-wasm --release` runs the module's requests natively: every machine listed, the sizing check giving the
same checks, values and cycle time as the analysis and verdict functions of the command line, a config from the page
read as a config file (a misspelt key refused by name), and the C functions handing back JSON. The page itself was run
in Chrome by hand: the T-gantry's default config passes every check with a cycle of 6.080 s.
