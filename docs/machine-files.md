# Machine files

A machine file defines a machine without Rust code and without a rebuild: the components of `sim-components` wired
together like a netlist (motor, gearbox, pulley, belt sections, carriages, guides), with the axes, ratings, bounds,
control loops and cycle of a machine config. CLI, app and PLC link load it from a path at run time.
Deutsche Fassung: [Maschinendateien](de/maschinendateien.md).

- [What a machine file is](#what-a-machine-file-is)
- [Use one](#use-one)
- [Worked example: an H-bot](#worked-example-an-h-bot)
- [The nodes](#the-nodes)
- [What is checked, and how errors read](#what-is-checked-and-how-errors-read)
- [The model behind it](#the-model-behind-it)
- [What machine files cannot do yet](#what-machine-files-cannot-do-yet)
- [How it is validated](#how-it-is-validated)
- [JSON, the JSON Schema and editors](#json-the-json-schema-and-editors)
- [Rust API](#rust-api)

## What a machine file is

A machine config as the [config reference](config-reference.md) describes it (`[sim]`, `[parts]`, `[effects]`,
`[belt]`, `[motor]`, `[gearbox]`, `[drive]`, `[ratings]`, `[profile]`, `[bounds]`, `[start]`, `[[cycle.segment]]`, ...),
plus one table `[machine]` that says what is connected to what:

```toml
format_version = 1          # of the config, as in every config

# ... the value tables of a machine config ...

[machine]
version = 1                 # of [machine]; a file of a later version is refused, not misread
id = "h-bot"                # how the front ends list it; default: the file name without extension
name = "H-bot (machine file)"   # a translation key or plain text; default: the id

[[machine.axis]]            # one table per node, each with a stable id
id = "x"
stroke_m = "geometry.stroke_x_m"   # a number, or the path of a number in the file
# ...
```

The values stay in the value tables, under the same keys as for the shipped machines. That is what makes everything
else work on a machine file as it is: the parameter panel, `--set`, the catalogue (`--motor`, `--axis`, ...), the
error bands of `[bounds]`, the sensitivity analysis, saving the config. A number field of a node can name such a value
by its path (`radius_m = "geometry.pulley_radius_m"`); then changing the value changes the machine. A plain number is
fixed.

Three files ship in `machines/`:

| file | what it is |
| ---- | ---------- |
| `machines/single-axis.toml` | the single axis of `crates/machine-single-axis`, rewired as a netlist |
| `machines/t-gantry.toml` | the T-gantry of `crates/machine-t-gantry`, rewired as a netlist (frame compliance and tool flex off, see [below](#what-machine-files-cannot-do-yet)) |
| `machines/h-bot.toml` | the worked example below: an X-Y table, not a real machine |

The easiest way to a new machine is to copy the one nearest to it and change `[machine]`.

## Use one

Command line: `--machine-file FILE` takes the place of the machine's id, for every command but `fmu`.

```
sim-cli verdicts --machine-file machines/h-bot.toml
sim-cli run --machine-file machines/t-gantry.toml --cycle --time 2 --out run.csv
sim-cli verdicts --machine-file machines/t-gantry.toml --bands --set belt.pretension_n=350
```

App: **Open** in the config file row (or **Open…** for the file dialog) opens a machine file like a config, and the machine changes with it.
**Save** writes the edited values back over the file, keeping its comments; a JSON file is not written over. The app
can also start with one: `sim-app --machine-file machines/h-bot.toml`.

PLC link: `sim-opcua --machine-file machines/h-bot.toml [--port N] [--lang en|de]`, in place of a machine's id.

## Worked example: an H-bot

An H-bot is an X-Y table whose two motors stand still at the ends of the X beam and drive one belt that runs around the
carriage and the head. Both motors turning the same way move X, turning against each other move Y. It is the T-gantry's
belt routing with the second axis lying flat, so `machines/h-bot.toml` is the T-gantry's file with these changes:

1. In `[geometry]`, `stroke_z_m` becomes `stroke_y_m`, and `[axis.z]` becomes `[axis.y]` (the limits of axis `y`).
2. `[machine]` names the axes `x` and `y`, gives `y` no incline (flat; the T-gantry's Z has `incline_deg = 90`), and
   drops the overhang: the head runs on its own guide on the carriage.

The belt first. The two motor pulleys cut it into three sections, each with one tension along it. Section 1 runs from
its clamp to motor A, and gets longer by 1 m for every metre of x and of y; motor A turning forward feeds belt into it
(`+1`). Section 2 runs from A around the head to B, shortens by 2 m per metre of y; A takes belt out of it (`-1`), B
feeds it (`+1`). Section 3 runs from B to its clamp:

```toml
[[machine.section]]
id = "1"
length_m = 0.3                      # with x = y = 0
axes = { x = 1, y = 1 }             # how much longer per metre of each axis
pulleys = { pulley_a = 1 }          # +1: this pulley's forward turn feeds belt in
pos = [450, -80]                    # where a node editor draws it; not read

[[machine.section]]
id = "2"
length_m = 2.6
axes = { y = -2 }
pulleys = { pulley_a = -1, pulley_b = 1 }

[[machine.section]]
id = "3"
length_m = 1.3
axes = { x = -1, y = 1 }
pulleys = { pulley_b = -1 }
```

From these alone the loader works out the kinematics: the belt has to keep its length, so x = r/2 (a + b) and
y = r/2 (a - b) for pulley angles a and b. A routing whose sections cannot all keep their length for some motion of the
axes, or whose motors do not set every axis, is refused.

The drive trains: each motor turns one gearbox, each gearbox one pulley. The motor and gearbox data are in `[motor]`,
`[drive]` and `[gearbox]`, shared by all motors:

```toml
[[machine.motor]]
id = "a"

[[machine.gearbox]]
id = "gearbox_a"
motor = "a"

[[machine.pulley]]
id = "pulley_a"
gearbox = "gearbox_a"
radius_m = "geometry.pulley_radius_m"
inertia_kgm2 = "pulley.inertia_kgm2"
```

(and the same for `b`). Then what moves, and what guides it. A body moves with the axes it lists; the head moves with
X and Y and carries the payload:

```toml
[[machine.body]]
id = "carriage"
mass_kg = "mass.carriage_kg"
axes = ["x"]

[[machine.body]]
id = "head"
mass_kg = "mass.z_axis_kg"
payload_kg = "mass.payload_kg"
axes = ["x", "y"]

[[machine.guide]]
id = "guide_x"
axis = "x"
friction = "friction_x"             # the table its friction values are in
carries = ["carriage", "head"]      # whose weight across the axis loads it

[[machine.guide]]
id = "guide_y"
axis = "y"
friction = "friction_z"
carries = ["head"]
```

Run it:

```
sim-cli verdicts --machine-file machines/h-bot.toml
```

```
check,motor,value,limit,result
motor-peak,a,0.09917,1.000,ok
motor-thermal,a,0.4428,100.0,ok
drive-current,a,0.2212,3.000,ok
drive-peak-current,a,0.8356,10.90,ok
gearbox-continuous,a,0.3766,40.00,ok
gearbox-input-speed,a,1914,13000,ok
gearbox-peak,a,1.326,64.00,ok
pulley-bearing-life,a,213589,20000,ok
gearbox-life,a,74415368568,20000,ok
belt-tooth-load,a,0.1164,1.000,ok
dc-link-energy,a,1.106,6.205,ok
dc-link-overvoltage,a,169.6,257.6,ok
motor-peak,b,0.09105,1.000,ok
motor-thermal,b,0.4562,100.0,ok
drive-current,b,0.2245,3.000,ok
drive-peak-current,b,0.7675,10.90,ok
gearbox-continuous,b,0.3850,40.00,ok
gearbox-input-speed,b,1914,13000,ok
gearbox-peak,b,1.182,64.00,ok
pulley-bearing-life,b,213351,20000,ok
gearbox-life,b,70668656880,20000,ok
belt-tooth-load,b,0.1038,1.000,ok
dc-link-energy,b,1.128,6.205,ok
dc-link-overvoltage,b,169.9,257.6,ok
settling,,0.00001314,0.0001000,ok
belt-tension,,345.7,1500,ok
belt-slack,,229.2,0,ok
gearbox-radial-load,,637.1,1500,ok
guide-load,,107.9,2000,ok
guide-life,,6789511,20000,ok
```

Every check passes. The motors run cooler than on the T-gantry (`motor-thermal` 0.44 against its 1.23): nothing has to
be held up against gravity. Signals follow the ids: `axis.y.position`, `motor.a.torque`, `belt.2.tension`, `power.guide_y`, ...

## The nodes

Every node has an `id`: letters, digits, `_` and `-`, unique in the file (not only in its list), so that a reference
cannot mean two things and an editor can keep it when the node moves. Every node may have a `pos = [x, y]`, where a
node editor draws it; the simulation does not read it. A number field takes a number or the dotted path of a number in
the file. The order of the motors in the file is the order of the motor signals and of `--motor N`.

| list | field | | meaning |
| ---- | ----- | - | ------- |
| `axis` | `id` | required | the axis; signals `axis.<id>.*`, its limits in `[axis.<id>]` |
| | `key` | | translation key of its name; default `axis-<id>` |
| | `stroke_m` | required | stroke, m; the axis runs from 0 to it |
| | `incline_deg` | | angle above the horizontal, deg, 90 vertical and rising towards +; default 0 |
| `motor` | `id` | required | a servo motor with its drive; signals `motor.<id>.*` |
| | `key` | | translation key of its name; default `motor-<id>` |
| `gearbox` | `motor` | required | the motor that drives it; one gearbox per motor; ratio 1 is a coupling |
| `pulley` | `gearbox` | required | the gearbox whose output turns it; one pulley per gearbox |
| | `radius_m` | required | pitch radius, m |
| | `inertia_kgm2` | required | inertia without the belt, kg m² |
| `section` | `length_m` | required | geometric length with every axis at 0, m |
| | `axes` | required | per axis id: how much longer the section gets per metre of that axis |
| | `pulleys` | required | per pulley id: `+1` where its forward turn feeds belt in, `-1` where it takes belt out |
| | `key` | | translation key of its name; default `belt-section-<id>`; signals `belt.<id>.*` |
| `body` | `mass_kg` | required | mass, kg |
| | `payload_kg` | | payload it carries while `effects.payload` is on and the gripper holds it, kg |
| | `axes` | required | the axes it moves with |
| `guide` | `axis` | required | the axis it guides; one guide per axis at most; its loss is `power.<id>` |
| | `friction` | required | the table of its friction values, laid out as `[friction_x]` |
| | `carries` | | bodies whose weight across the axis it carries |
| | `overhang` | | a body that overhangs the guide: accelerated along `overhang_axis` it tilts the guide's carriages |
| | `overhang_axis`, `lever_m`, `spacing_m` | with `overhang` | the axis, the lever of the overhang's centre of mass and the spacing of the guide's two carriages, m; the tilt puts 2 m a lever / spacing on them |
| | `lever_axis` | | an axis whose travel shortens the lever |
| | `side_load` | | the guide also carries the overhang's inertia force m a; default false |

There are as many motors as axes. A file names each list it uses; `guide` may be left out (no guide friction).

## What is checked, and how errors read

A machine file is checked in full when it is loaded, before anything runs:

- the TOML or JSON itself, `format_version` and `machine.version`;
- every node: no field its kind does not have, every required field there, each value of the right type, ids valid
  and unique, every reference to a node that exists, every path to a number that exists;
- the machine: masses, radii and lengths positive, one gearbox per motor and one pulley per gearbox, as many motors as
  axes, a routing that keeps the belt's length and sets every axis;
- every other key against the keys the registered machines' configs have (so `pretensoin_n` is refused by name), plus
  `[axis.<id>]` for each axis, a friction table for each guide, and every key the netlist names by path;
- then everything the simulation reads at its start, as for a config: the drive, the cycle, the start pose, the
  catalogue parts the file names.

An error names the file, the line and the key, in the front end's language:

```
$ sim-cli verdicts --machine-file broken.toml
Configuration has a value this machine does not know: broken.toml:604: machine.guide[guide_z].side_lod
```

A node is named by its list and its id, `machine.guide[guide_z]`; before its id is read, by its place in the list,
`machine.gearbox[#2]` (from 1). Where an error is about a value a node names by its path, the line is that of the
node's field. The line is found for TOML files; for JSON the key alone is given.

## The model behind it

A machine file builds a `ConfigMachine`: a belt gantry of any routing with n axes q and as many motors. It is the model
of the single-axis and T-gantry crates, written for any routing ([model reference](model-reference.md) has their
derivations):

- Belt section s has the length L_s = L0_s + Σ_i c_si q_i, plus the teeth in mesh at each driven pulley it ends at
  (`Belt::tooth_length`), and pulley k feeds f_sk r_k of belt into it per radian.
- The belt pulls axis i with -Σ_s c_si T_s and turns pulley k with r_k Σ_s f_sk T_s (of the belt's material, with the
  elastic law of `Belt`).
- Rigid (effects.belt_stretch off), the belt keeps its length: C q = F diag(r) a, so a = K q with K the least-squares
  solution of F diag(r) K = C. The loader refuses a routing where K does not reproduce C exactly, or is not invertible.
- At rest the tensions hold every axis against gravity, and each closed belt keeps its material: sections that
  share a pulley are one belt, and it is closed where each of its pulleys feeds it on one side and takes it out on
  another. A machine may have several belts (one per axis, say); a belt that ends on a drum has no such sum.
- The mass on axis i is the sum of the bodies that move with it; the axes are taken square to each other, so the
  bodies' mass matrix is diagonal. Pulleys and rotors (J_pulley + i² J_rotor per motor, the belt's mass shared out on
  the pulleys equally) add K^T J K. Gravity pulls axis i with m_i g sin(incline_i).
- Gearbox (backlash, stiffness, efficiency), motor friction, torque ripple and cogging, tooth meshing, holding brake,
  hard stops at both ends of every stroke, LuGre guide friction with its load scaling: the components of
  `sim-components::mech`, as the crates use them.

The controller, drives, DC link, winding heat, analyses, verdicts, sensitivity and error bands are those of every
machine; nothing about them is particular to machine files.

## What machine files cannot do yet

- These effects: `frame_compliance`, `tool_flex`, `idlers`, `cable_chain`, `process_force`, `thermal_feedback`,
  `belt_tooth_jump`, `belt_span_vibration`. A file that switches one on is refused (`effects.<name> (not in machine files
  yet)`), so a result never leaves out an effect that was asked for. `part_heating` works (it only shows).
- Mechanisms other than belts: screws, racks, linkages, rotary joints. The node kinds are the ones above.
- Different motors, gearboxes or drives per axis: `[motor]`, `[gearbox]` and `[drive]` hold for all.
- A structural resonance: the machine reports none, so `effects.shaper_follows_pose` keeps the fixed shaper frequency
  and the app has no mode shape to show.
- Section lengths are numbers (or paths), not formulas: a section whose length depends on a stroke does not follow a
  change of that stroke by itself.
- The 3D view draws a plain machine (a beam, a box per body, a drive unit per motor), not the belt.
- `sim-cli fmu` refuses a machine file: the FMU finds its machine among the registered ones.
- `sim-cli list` and the app's machine list show the registered machines only; a file is opened, not listed.

## How it is validated

`crates/sim-machines/tests/machine_files.rs` loads `machines/single-axis.toml` and `machines/t-gantry.toml` and holds
them against their crates:

- the machine itself, in five sets of effects (default, rigid, without losses, without gravity, without belt details
  and payload), at three poses with the payload held, set down and with the brakes on: kinematics, the state at rest
  and its holding torques, the derivatives of a state moved off rest under torques, energy, feedforward, rigid
  tensions, and every signal both have: agree to 1e-9;
- the full cycle (controller, drives, DC link, winding), elastic and rigid, without the encoder's counts: every motor
  load and verdict value agrees to 1e-5. Closer is not possible: the two sum in another order, and while a guide
  breaks away from stiction a difference of 1e-36 grows tenfold every 30 steps before it closes again in the cruise;
- the same with the encoder's counts as shipped: the same verdicts, limits and pass or fail. There a last-bit
  difference moves one count by a step, and switching thresholds (the braking chopper) carry that on: the crate run
  with its carriage 1e-9 heavier spreads its peak braking power by 0.5 %, and the file is held to three times the
  crate's own spread, or 1e-4.

The H-bot has no crate to be held against. All three files pass the standard suite every machine passes (round trip,
static hold, determinism, energy conservation without losses, the loss budget with them), and the H-bot's motor angles
and torques are checked against a hand calculation. Its suite run has the gearbox backlash off: nothing pulls its
gears to one flank, and the suite's small pulse knocking them across the backlash drifts the energy by 0.17 %, over
the 0.1 % allowed. That is the gearbox contact at the suite's step size, not the netlist: the T-gantry crate with
gravity off drifts by 0.19 % the same way.

`crates/sim-machines/tests/machine_file_review.rs` holds what an independent review found by experiment and what was
fixed: a machine with two or three separate belt loops (its rest state was wrong where an axis is vertical), a
guide on an overhang where the drive masses couple the axes, an axis no body moves, the H-bot's Y drawn along X,
unchecked `machine.id` and `machine.name`, error lines that pointed elsewhere, and a JSON version written `1.0`.

The tests also check that a JSON file gives the same machine as its TOML, that `schema/machine-file.schema.json` is
current, that nine kinds of broken file are refused with the right key and line, and (in `sim-cli`, `sim-app` and
`sim-opcua`) that each front end loads, runs and refuses machine files.

## JSON, the JSON Schema and editors

A file whose name ends in `.json` is read as JSON: the same tables as objects, the node lists as arrays of objects.
`format_version` and `machine.version` are integers (`1`, not `1.0`).

`schema/machine-file.schema.json` (JSON Schema, draft 2020-12) describes `[machine]` in full, with every field's
meaning; the value tables it lists as objects (their keys are in the [config reference](config-reference.md)). It is
written from the same list the loader checks against (`sim_netlist::netlist::NODES`). After a change to that list,
rewrite it with

```
MACHINE_SCHEMA_WRITE=1 cargo test -p sim-machines --test machine_files the_json_schema_file_is_current
```

The format is meant for a node editor too: ids are stable and unique in the file, references are by id, `pos` holds a
node's place on the canvas and is otherwise ignored, and nothing depends on the order of the nodes but the order of the
motors and axes.

## Rust API

- `sim_machines::load_file(path) -> Result<&'static MachineEntry, FileError>`: what the front ends call. The entry is
  used like one of `MACHINES` (`Sim::new(entry, &entry.config())`, the analyses, `translations`). It lives as long as
  the program.
- `sim_netlist::from_text(toml_text, lines, fallback_id, known)`: the same on a file's TOML text, with the configs
  whose keys a file may use given (`known`); it returns the error with its line where `lines` is set. JSON is
  converted in `sim_machines`, the only layer that reads it; the machine crates' layer has `toml` alone.
- `FileError { file, line, error }`, `error` a `ConfigError` (a translation key and the path it is about), as for any
  config.
- `sim_netlist::build(&Config)`: the `MachineEntry::build` of every machine file; the netlist is in the config.
- `sim_netlist::Netlist::parse(&Config)`, `ConfigMachine::new(&Config, &Netlist)`, `json_schema()` (as a value; `sim_machines::json_schema()` writes it),
  `line_of(text, path)`.
