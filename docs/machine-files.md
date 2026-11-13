# Machine files

A machine file defines a machine without Rust code and without a rebuild: the components of `sim-components` wired
together like a netlist (motor, gearbox, pulley, belt sections, carriages, guides, and ball screws, racks, linear motors
and rotary tables beside the belts), with the axes, ratings, bounds,
control loops and cycle of a machine config. CLI, app and PLC link load it from a path at run time.
Deutsche Fassung: [Maschinendateien](de/maschinendateien.md).

- [What a machine file is](#what-a-machine-file-is)
- [Use one](#use-one)
- [Worked example: an H-bot](#worked-example-an-h-bot)
- [The nodes](#the-nodes)
- [Motors that differ](#motors-that-differ)
- [Screws, racks, linear motors and rotary tables](#screws-racks-linear-motors-and-rotary-tables)
- [The other examples: a CoreXY and an XYZ portal](#the-other-examples-a-corexy-and-an-xyz-portal)
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

Five files ship in `machines/`:

| file | what it is |
| ---- | ---------- |
| `machines/single-axis.toml` | the single axis of `crates/machine-single-axis`, rewired as a netlist |
| `machines/t-gantry.toml` | the T-gantry of `crates/machine-t-gantry`, rewired as a netlist, with its idlers, frame, cable chain and belt spans (tool flex is not in machine files, see [below](#what-machine-files-cannot-do-yet)) |
| `machines/h-bot.toml` | the worked example below: an X-Y table, not a real machine |
| `machines/corexy.toml` | a CoreXY X-Y table: two fixed motors, two crossed belts ([below](#the-other-examples-a-corexy-and-an-xyz-portal)) |
| `machines/xyz-portal.toml` | a three-axis cartesian portal, X, Y and Z each on its own belt, with a smaller Z motor of its own |

The easiest way to a new machine is to copy the one nearest to it and change `[machine]`.

## Use one

Command line: `--machine-file FILE` takes the place of the machine's id, for every command.

```
sim-cli verdicts --machine-file machines/h-bot.toml
sim-cli run --machine-file machines/t-gantry.toml --cycle --time 2 --out run.csv
sim-cli verdicts --machine-file machines/t-gantry.toml --bands --set belt.pretension_n=350
sim-cli fmu --machine-file machines/h-bot.toml --fmi 3 --out h-bot.fmu
```

An FMU of a machine file carries the file among its resources (`resources/machine.toml`) and builds the machine from it
when it is instantiated, for FMI 2.0 and 3.0 alike; the tool that imports it needs nothing else.

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
| | `stroke_m` | required | stroke, m; the axis runs from 0 to it (a rotary axis has `stroke_rad` instead) |
| | `stroke_rad` | | a rotary axis's range from 0, rad, in place of `stroke_m` |
| | `incline_deg` | | angle above the horizontal, deg, 90 vertical and rising towards +; default 0 |
| `motor` | `id` | required | a servo motor with its drive; signals `motor.<id>.*` |
| | `key` | | translation key of its name; default `motor-<id>` |
| `gearbox` | `motor` | required | the motor that drives it; one gearbox per motor (none for a linear motor); ratio 1 is a coupling |
| `pulley` | `gearbox` | required | the gearbox whose output turns it; one pulley, screw, rack or rotary per gearbox |
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
| `axis` | `cable_chain` | | the axis drags a cable chain fixed at its 0 end (`effects.cable_chain`, values in `[cable_chain]`); default false |
| `pulley` | `at_m` | | where the 3D view draws it, `[x, y, z]`, m; default at an end of the first axis |
| `section` | `route` | | the pulleys, idlers and bodies (a clamp on it) the section runs past, in order, for the 3D view; default its pulleys and the body it is clamped to |
| `idler` | `speed` | required | belt speed past it per speed of each axis, m/s per m/s (`effects.idlers`) |
| | `radius_m`, `inertia_kgm2`, `friction_nm` | required | its radius, inertia and bearing friction torque |
| | `body`, `at_m` | | a body it rides on and where the 3D view draws it (from that body) |
| `frame` | `axis` | required | the horizontal axis the frame sways along (`effects.frame_compliance`, elastic model); at most one |
| | `mass_kg`, `stiffness_n_m`, `damping_n_s_m` | required | the frame's mass, stiffness and damping in its sway |
| `span` | `section` | required | a free belt span of that section, for its sideways mode (`effects.belt_span_vibration`) |
| | `length_m` | required | its length with every axis at 0, m |
| | `axes` | | how much longer it gets per metre of each axis |
| `screw` | `gearbox`, `axis` | required | a ball screw on that gearbox's output, its nut driving that axis ([below](#screws-racks-linear-motors-and-rotary-tables)) |
| | `values` | required | the table of its values, laid out as `[screw]` of the joint kit's screw axis |
| `rack` | `gearbox`, `axis` | required | a pinion on that gearbox's output, driving that axis along a rack |
| | `values` | required | the table of its values, laid out as `[rack]` of the joint kit's rack axis |
| `linear_motor` | `motor`, `axis` | required | that motor is a linear motor driving that axis; it has no gearbox node |
| | `values` | required | the table of its values, laid out as `[linear_motor]` of the joint kit's linear motor axis |
| `rotary` | `gearbox`, `axis` | required | a rotary table on that gearbox's output: the axis turns, in rad |
| | `inertia_kgm2` | required | the inertia it turns, kg m² |
| | `payload_inertia_kgm2` | | its payload's while `effects.payload` is on and the gripper holds it, kg m² |
| | `strain_wave` | | the table of its strain wave gear's life values, laid out as `[strain_wave]` |

There are as many motors as axes. A file names each list it uses; `axis` and `motor` are always there, `gearbox`,
`pulley` and `section` where belts are driven, `body` where an axis moves one, and the others may be left out.
An effect that needs nodes the file has not (`frame_compliance` without a `frame`, `belt_span_vibration` without a
`span`) refuses the file with `machine.<list>`.

## Motors that differ

`[motor]`, `[gearbox]`, `[drive]`, `[ratings]` and `[parts]` hold for every motor. A table `[<table>.<motor id>]` gives
one motor its own values over them, key by key; it may only hold keys the shared table has:

```toml
[gearbox.mz]          # the Z motor of machines/xyz-portal.toml has its own gearbox ratio
ratio = 10.0

[motor.mz]            # and a smaller motor
rotor_inertia_kgm2 = 0.15e-4
rated_current_a = 1.411

[drive.mz]            # its drive retuned for it
peak_torque_nm = 1.9
rated_torque_nm = 0.635
velocity_kp_nm_s_rad = 0.01

[ratings.mz]          # and what its smaller gearbox and drive allow
gearbox_rated_torque_nm = 18.0
drive_rated_current_a = 1.5

[parts.mz]            # or a motor, gearbox and drive from the catalogue, for this motor alone
motor = "1FK2203-2AG"
```

That motor's drive train in the machine (rotor, gearbox, motor friction, ripple, brake), its drive in the controller
(current and velocity loops, torque-speed curve, winding, DC link) and its verdicts (motor, drive current, gearbox,
bearing, belt teeth, braking) take its own values; the machine-wide checks keep the shared ones. A part named in
`[parts.<id>]` sets that motor's values over the numbers, as `[parts]` does; `belt` and `axis` are the machine's and are
refused there. What stays shared: the drives' cycle times, the noise seed, a shared DC link and the braking module.
`sim-cli catalog` searches the shared motor; a motor with its own `[parts.<id>]` keeps its part through the search.
The time-optimal path planner takes the shared `[drive]`'s torque curve for every motor.

## Screws, racks, linear motors and rotary tables

A motor may drive an axis without a belt: through a `screw`, `rack` or `rotary` node on its gearbox's output (a gearbox
of ratio 1 is the coupling), or as a `linear_motor`, which is the motor itself and has no gearbox node. Each drives one
axis, an axis has at most one, and belts and these mix in one file:

```toml
[[machine.screw]]          # machines/screw-portal.toml: X and Y on belts, Z on a ball screw 16 x 10
id = "screw_z"
gearbox = "gearbox_z"      # the coupling: [gearbox.mz] has ratio = 1
axis = "z"
values = "screw_z"         # the table [screw_z], laid out as [screw] of the joint kit's screw axis
```

The values are the joint kit's ([mechanisms](mechanisms.md)), in a table the node names, so that two screws have two
tables: `[screw]`'s layout for a screw (lead, nominal and root diameter, length between the bearings, mounting,
efficiency, nut and bearing stiffness, preload, load ratings and factors, `max_dn`), `[rack]`'s for a rack (module,
teeth, helix angle, efficiency, stiffness, backlash, pinion inertia, rated feed force, service factor),
`[linear_motor]`'s for a linear motor (attraction force, pole pitch, guide friction coefficient), `[strain_wave]`'s for
a rotary's strain wave gear (rated torque, input speed and life). The drive train is built as the joint kit builds it
(`transmission::Converter` behind the gearbox): the screw's lead / 2 pi or the pinion's radius as a lever, its stiffness
in series with the gearbox's, its efficiency on the gearbox's, the screw shaft's or pinion's inertia on the rotor. So a
screw axis's motor needs T = J a / lever + F lever / eta to lift a load F at an acceleration a (J the rotor, coupling
and shaft), and a rack's motor turns i x / r.

- With `effects.guide_friction` the nut's preload drags at the motor, and a linear motor's attraction adds its guide
  friction coefficient times itself; the attraction and the forcer's weight also press on the axis's guide
  (`axis.<id>.guide_load`, judged by the guide checks).
- A linear motor's `[motor.<id>]` and `[drive.<id>]` read in N for N m and m for rad (force constant in N/A, the
  forcer's mass as `rotor_inertia_kgm2`); its `[gearbox.<id>]` has ratio 1 and is the forcer's mounting.
- A rotary turns its axis in rad: the axis has `stroke_rad` for `stroke_m`, its limits are `[axis.<id>]`'s
  `max_velocity_rad_s`, `max_acceleration_rad_s2` and `max_jerk_rad_s3`, and what it moves is the rotary's
  `inertia_kgm2` (and `payload_inertia_kgm2`). No body moves with it; it has no guide, no incline and no cable chain.
- Signals and checks are named by the motor, as the joint kit's are by its joint: `joint.<motor>.force` (what the
  screw, rack, linear motor or rotary passes to its axis), `screw.<motor>.speed`, `.dn` and `.damage_rate`,
  `rack.<motor>.feed_force`, `strain_wave.<motor>.damage_rate`; `gearbox.<motor>.output_torque` is the torque ahead of
  the screw or rack. The checks are the kit's: `screw-speed`, `screw-dn`, `screw-buckling`, `screw-static`,
  `screw-life`, `rack-feed-force`, `strain-wave-life` (the lives where `ratings.required_life_h` is set). A motor that
  turns no pulley reports no `pulley.<motor>.radial_load`, so the belt-tooth and pulley-bearing checks pass it over.
- `effects.belt_stretch` switches the elastic model of the whole machine: with it the screws', racks' and rotaries'
  gearboxes twist too (the converter's stiffness in series), and the belt's mode at a pose includes them, with the
  rotors held.

`machines/screw-portal.toml` is the XYZ portal with its Z belt replaced by a ball screw 16 x 10 behind a bellows coupling,
the Z motor's brake holding the slide (0.055 N m of its 0.5); `machines/rack-gantry.toml` is a two-axis gantry with a
3 m X on a rack and pinion (module 2, 200 mm per turn) behind a gearbox of ratio 10 and Y on a belt. Both pass every
verdict on their cycles (`screw-speed` 2402 of 9833 r/min allowed, `rack-feed-force` 422 of 2400 N). The joint kit's
screw, rack, linear motor and rotary table axes written as files give the kit's verdicts digit for digit.

## The other examples: a CoreXY and an XYZ portal

`machines/corexy.toml` is a CoreXY table. Its two motors stand at the front corners; each drives a belt clamped to the
head at both ends, so each motor's pulley cuts its belt into two sections. Belt A gets longer by x + y on one side of
its pulley, belt B by x - y, so the pulley angles are a = (x + y) / r and b = (x - y) / r: the loader finds that from
the sections alone, and one motor turning alone moves the head along a diagonal. The gantry moves in Y, the head on it
in X. Its idler nodes (four, off by default) and the `route` of each section are what the 3D view draws.

`machines/xyz-portal.toml` is a three-axis portal: X, Y and Z each on a belt loop of its own, round its motor's pulley
and an idler at the far end, clamped to its carriage, Z vertical. Its Z motor is smaller, with a gearbox of ratio 10,
its drive tuned for it and its own ratings ([above](#motors-that-differ)); it holds the slide and payload,
m g r / i = 3.5 kg * 9.81 m/s² * 0.02 m / 10 = 0.0686 N m.

Both pass every verdict on their cycles with the T-gantry's placeholder values (`sim-cli verdicts --machine-file ...`);
neither is a real machine.

## What is checked, and how errors read

A machine file is checked in full when it is loaded, before anything runs:

- the TOML or JSON itself, `format_version` and `machine.version`;
- every node: no field its kind does not have, every required field there, each value of the right type, ids valid
  and unique, every reference to a node that exists, every path to a number that exists;
- the machine: masses, radii and lengths positive, one gearbox per motor (none for a linear motor) and one pulley,
  screw, rack or rotary per gearbox, at most one of these per axis, a screw at least as long as its stroke, as many
  motors as axes, a routing that keeps the belt's length and sets every axis;
- every other key against the keys the registered machines' configs have (so `pretensoin_n` is refused by name), plus
  `[axis.<id>]` for each axis, a friction table for each guide, the value tables of screws, racks, linear motors and
  strain wave gears laid out as the joint kit's, and every key the netlist names by path;
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
  `sim-components::mech`, as the crates use them. Each motor's from its own config ([above](#motors-that-differ)).
- Idlers (`machine.idler`, `effects.idlers`): idler d passes belt at s_d . v (its `speed`), so its inertia J / r² adds
  m_d s_d s_d^T to the axes' mass matrix (which is then full), and its bearing pulls the axes back with
  F_d tanh(s_d . v / (w r)) s_d, F_d its friction torque over its radius. They carry no weight.
- A cable chain on an axis: its moving mass m(q) = mu (L0 + q / 2) takes m dv/dt + m' v² / 2, and it drags.
- A process force per axis, `process.force_<axis>_n`, on the tool while the cycle dwells.
- The frame (`machine.frame`, elastic model): it sways by u along a horizontal axis f; the bodies move at v + e_f du/dt
  over ground, and the frame takes back the forces its belt, guides, idlers, chain and stops put on them, but not the
  process force, which the workpiece on the floor takes. The tool moves with the frame.
- Thermal feedback: each gearbox's efficiency and each guide's viscous friction follow their temperature, the belt's
  stiffness the ambient one (`mech::ThermalFeedback`).
- Belt tooth jump (elastic model): at each motor pulley between the section it feeds and the one it takes from
  (`mech::ToothJump`); a pulley with a section on one side only (a drum) refuses the effect.
- Span vibration (`machine.span`): each span's first sideways mode at its section's tension, against what the drives
  excite (`mech::span_frequency`).
- The belt's mode (elastic model): the axes, and the frame where it sways, on the belt's stiffness with the pulleys
  held, K = Σ_s EA / L_s c_s c_s^T, against their mass matrix; the lowest is `resonance_hz` at a pose, so
  `effects.shaper_follows_pose` sets the shaper to it, and the app shows its shape (scaled to 1 m of tool travel).
- A screw, rack, linear motor or rotary (`machine.screw`, ...): its motor's gearbox becomes one in the axis's unit, of
  ratio i / lever (the joint kit's `RotaryJoint` with the `transmission::Converter` behind it), and its "pulley" is the
  axis itself: K has the row e_i for it, the axis takes the gearbox's output as its force, the belt's equations leave
  the axis to it (at rest and in the rigid tensions), and in the elastic model the pulley's states follow the axis. A
  rotary axis's mass is its rotary's inertia, with no gravity along it.

The controller, drives, DC link, winding heat, analyses, verdicts, sensitivity and error bands are those of every
machine; nothing about them is particular to machine files.

## What machine files cannot do yet

- `tool_flex`: the T-gantry's Z axis leaning on its carriage. It needs where that axis's centre of mass and tool tip
  sit below its guide and how that changes with its travel, geometry a netlist of belts and bodies does not have; a node
  for it would be the T-gantry's special case. A file that switches it on is refused (`effects.tool_flex (not in machine
  files yet)`), so a result never leaves out an effect that was asked for.
- Linkages (a five-bar, a scissor lift) and worm gears: the mechanisms beside belts are screws, racks, linear motors
  and rotary tables, each driving one axis straight. A body cannot ride on a rotary axis (it would move on a circle,
  not along square axes), and a rotary axis has no bearing friction of its own (only its motor's and gearbox's).
- `effects.belt_tooth_jump` together with a screw, rack, linear motor or rotary: its states are laid out per motor, and
  a motor without a pulley has none; the file is refused.
- A screw's stiffness is the one at its softest nut position for the whole stroke, as in the joint kit; its preload's
  drag sits at the motor with a smoothing in rad/s.
- Modes other than the belt's: the frame enters the belt's mode where it sways, but the guides' and the motors' own
  stiffness do not, and a body's bending is not modelled.
- Section and span lengths are numbers (or paths), not formulas: one whose length depends on a stroke does not follow a
  change of that stroke by itself.
- The 3D view draws the belt as straight strips from point to point of each section's route, not round its pulleys.
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

The same comparison runs with each effect machine files have had since 13 November switched on (frame, idlers, cable
chain, process force, thermal feedback with warm parts, belt tooth jump, span vibration), elastic and rigid, and with
all of them at once: the T-gantry's file names its five idlers, frame and eight spans, and gives the crate's numbers to
1e-9 (`the_effects_of_a_machine_file_are_its_crates`). The effects' own hand checks are the crate's.

The H-bot, the CoreXY and the portal have no crate to be held against. All five files pass the standard suite every machine passes (round trip,
static hold, determinism, energy conservation without losses, the loss budget with them), and hand calculations check the H-bot's and
the CoreXY's motor angles and torques, the portal's per-motor gearbox ratio (in its Z motor's angle and speed), the
torque its Z motor holds the slide with, its Z motor's own drive and ratings, the single axis's belt mode, and the belt
strips the 3D view draws. The CoreXY and the portal lie flat (in X and Y) as the H-bot does and run the suite without
backlash for the same reason. Its suite run has the gearbox backlash off: nothing pulls its
gears to one flank, and the suite's small pulse knocking them across the backlash drifts the energy by 0.17 %, over
the 0.1 % allowed. That is the gearbox contact at the suite's step size, not the netlist: the T-gantry crate with
gravity off drifts by 0.19 % the same way.

`crates/sim-machines/tests/machine_file_mechanisms.rs` holds the screws, racks, linear motors and rotary tables: the
joint kit's screw axis written as a file is its crate to 1e-9 (elastic and rigid, payload held, set down and braked:
kinematics, rest state, derivatives, energy, feedforward, the screw's signals and torque breakdown, its checks); by
hand, the screw portal's Z motor torque for a load and acceleration and its screw's critical speed, the rack gantry's
motor angle x i / r and feed force, a linear motor's current for a force and its guide load, a rotary table's torque in
rad; both new files pass the standard suite; and a broken screw is refused by key and line.

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
- `sim_components::catalog::motor_config(&Config, motor_id)`: the config one motor sees ([motors that
  differ](#motors-that-differ)); `Controller::drive_params_of(k)` is its drive.
- `sim_machines::load_text(toml_text, id)`: a machine file's text as `load_file` reads it (what an FMU does with its
  `resources/machine.toml`).
