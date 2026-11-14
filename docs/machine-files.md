# Machine files

A machine file defines a machine without Rust code and without a rebuild: the components of `sim-components` wired
together like a netlist (motor, gearbox, pulley, belt sections, carriages, guides, and ball screws, racks, linear motors,
rotary tables and worm gears beside the belts, or a parallel linkage of two or three arms, or a scissor lift), with the
axes, ratings, bounds,
control loops and cycle of a machine config. CLI, app and PLC link load it from a path at run time.
Deutsche Fassung: [Maschinendateien](de/maschinendateien.md).

- [What a machine file is](#what-a-machine-file-is)
- [Use one](#use-one)
- [Worked example: an H-bot](#worked-example-an-h-bot)
- [The nodes](#the-nodes)
- [Motors that differ](#motors-that-differ)
- [Screws, racks, linear motors and rotary tables](#screws-racks-linear-motors-and-rotary-tables)
- [Linkages: the planar and the spatial delta](#linkages-the-planar-and-the-spatial-delta)
- [A scissor lift](#a-scissor-lift)
- [Worm gears](#worm-gears)
- [The belt's wrap and a leaning axis](#the-belts-wrap-and-a-leaning-axis)
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

Fourteen files ship in `machines/`:

| file | what it is |
| ---- | ---------- |
| `machines/single-axis.toml` | the single axis of `crates/machine-single-axis`, rewired as a netlist |
| `machines/t-gantry.toml` | the T-gantry of `crates/machine-t-gantry`, rewired as a netlist, with its idlers, frame, cable chain, belt spans and its leaning Z axis (a `lean` node and `[tool_flex]`, off as shipped; [below](#the-belts-wrap-and-a-leaning-axis)) |
| `machines/h-bot.toml` | the worked example below: an X-Y table, not a real machine |
| `machines/corexy.toml` | a CoreXY X-Y table: two fixed motors, two crossed belts ([below](#the-other-examples-a-corexy-and-an-xyz-portal)) |
| `machines/xyz-portal.toml` | a three-axis cartesian portal, X, Y and Z each on its own belt, with a smaller Z motor of its own |
| `machines/screw-portal.toml` | the XYZ portal with its Z on a ball screw ([below](#screws-racks-linear-motors-and-rotary-tables)) |
| `machines/rack-gantry.toml` | a two-axis gantry, X on a rack and pinion, Y on a belt |
| `machines/planar-delta.toml` | the planar delta of `crates/machine-delta-d2` as a linkage of two arms ([below](#linkages-the-planar-and-the-spatial-delta)) |
| `machines/delta-d3.toml` | the spatial delta of `crates/machine-delta-d3` as a linkage of three arms |
| `machines/scissor-lift.toml` | a two-stage scissor lift table on a ball screw ([below](#a-scissor-lift)) |
| `machines/dual-gantry.toml` | a dual-drive gantry: a yawing bridge on two racks, its drives in plain, master/slave or cross-coupled synchronisation ([below](#a-gantry-that-yaws-and-tools-on-rotary-axes)) |
| `machines/portal-c.toml` | the screw portal with a C axis turning a gripper, its tip the tool centre point |
| `machines/five-axis.toml` | the screw portal with a fork head (C and A), its tool's tip the tool centre point |
| `machines/five-axis-table.toml` | the screw portal over a tilting rotary table (A and C carry the workpiece, table/table), the tip on the workpiece the tool centre point |

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
| | `stroke_m` | | stroke, m; the axis runs from 0 to it. Every axis has it but a rotary one (which has `stroke_rad`) and a linkage's or scissor lift's (whose range the node sets) |
| | `stroke_rad` | | a rotary axis's range from 0, rad, in place of `stroke_m` |
| | `incline_deg` | | angle above the horizontal, deg, 90 vertical and rising towards +; default 0; on a rotary axis the tilt of the axis it turns about from the vertical (0, a C axis) towards x (90, an A axis) |
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
| | `axes` | required | the axes it moves with; rotary axes among them carry it round, the first outermost |
| | `offset_m` | | on rotary axes: its centre of mass from where their axes meet, `[x, y, z]` at angle 0, m ([below](#a-gantry-that-yaws-and-tools-on-rotary-axes)) |
| `guide` | `axis` | required | the axis it guides; one guide per axis at most; its loss is `power.<id>` |
| | `friction` | required | the table of its friction values, laid out as `[friction_x]` |
| | `carries` | | bodies whose weight across the axis it carries |
| | `overhang` | | a body that overhangs the guide: accelerated along `overhang_axis` it tilts the guide's carriages |
| | `overhang_axis`, `lever_m`, `spacing_m` | with `overhang` | the axis, the lever of the overhang's centre of mass and the spacing of the guide's two carriages, m; the tilt puts 2 m a lever / spacing on them |
| | `lever_axis` | | an axis whose travel shortens the lever |
| | `side_load` | | the guide also carries the overhang's inertia force m a; default false |
| `axis` | `cable_chain` | | the axis drags a cable chain fixed at its 0 end (`effects.cable_chain`, values in `[cable_chain]`); default false |
| `pulley` | `at_m` | | where the 3D view draws it, `[x, y, z]`, m; default at an end of the first axis; with `effects.belt_wrap` its place in the belt's plane (x, z) |
| `pulley`, `idler` | `clockwise` | | with `effects.belt_wrap`: the belt runs round it clockwise, seen from the front (x right, z up) and followed along the routes in their order; default false |
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
| | `friction_coulomb_nm`, `friction_viscous_nm_s_rad` | | its bearing's Coulomb and viscous friction at the axis, with `effects.guide_friction`; default 0 |
| | `strain_wave` | | the table of its strain wave gear's life values, laid out as `[strain_wave]` |
| `worm` | `gearbox` | required | that gearbox is a worm stage ([below](#worm-gears)); one worm per gearbox |
| | `ratio` | required | its ratio, in place of the gearbox's |
| | `efficiency`, `back_efficiency` | required | driven by the worm (above 0, at most 1) and back by the wheel (at most 1; 0 or less locks itself) |
| `lean` | `body`, `axis` | required | a body leaning on its carriage along a horizontal axis it moves with (`effects.tool_flex`, elastic model); at most one ([below](#the-belts-wrap-and-a-leaning-axis)) |
| | `com_drop_m`, `tool_drop_m` | required | its centre of mass and the tool tip below the guide with `lever_axis` at 0, m |
| | `lever_axis` | | an axis whose travel raises both towards the guide |
| | `lateral_stiffness_n_m`, `tilt_stiffness_nm_rad`, `damping_n_s_m` | required | the guide's stiffness sideways and in tilt, the lean's damping |
| `gantry` | `axis` | required | a bridge travelling along this horizontal straight axis yaws (`effects.gantry_yaw`); at most one ([below](#a-gantry-that-yaws-and-tools-on-rotary-axes)) |
| | `across` | | the straight axis along the bridge; its bodies' lever about the bridge's centre changes with it |
| | `width_m`, `inertia_kgm2` | required | the distance between the bridge's two sides, m; its own inertia about a vertical axis through its centre, kg m² |
| | `stiffness_nm_rad`, `damping_nm_s_rad` | required | what holds the yaw beside drives and belts (its joints and guides), and its damping |
| | `across_start_m` | | where `across` is at 0 from the first side, m; default its stroke in the middle |
| | `drives` | | per motor whose screw, rack or linear motor drives `axis`: its place across the bridge from the centre, m; two make a dual-drive gantry |
| | `sections` | | per belt section that pulls the bridge's ends: its length per rad of yaw, m |
| | `tolerance_m` | | the synchronisation error (width x yaw) check `gantry-sync` allows, m |
| `tcp` | `axes` | required | the tool centre point: the three straight axes along x (the file's first), y (horizontal) and z (vertical); at most one |
| | `rotaries` | | the rotary axes that carry the tool (a head), outermost first; `rotaries` or `table` or both |
| | `table` | | the rotary axes that carry the workpiece (a rotary or tilting table), outermost first: the tool's x, y and z are the tip's on the workpiece |
| | `table_pivot_m` | | with `table`: where the table's axes meet, in the straight axes' coordinates, `[x, y, z]`, m; default `[0, 0, 0]` |
| | `tool_m` | required | the tool's tip from where the head's rotaries' axes meet (from the straight axes' point without a head), `[x, y, z]` at angle 0, m |

There are as many motors as axes, and one more for each drive of a gantry beyond the first. A file names each list it uses; `axis` and `motor` are always there, `gearbox`,
`pulley` and `section` where belts are driven, `body` where an axis moves one, and the others may be left out.
An effect that needs nodes the file has not (`frame_compliance` without a `frame`, `belt_span_vibration` without a
`span`, `tool_flex` without a `lean`) refuses the file with `machine.<list>`; `belt_wrap` without the places it is worked
out from is refused with what is missing.

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
`sim-cli search` searches the shared motor; a motor with its own `[parts.<id>]` keeps its part through the search.
`sim-cli search --machine-file FILE --motor-id ID` searches one motor alone: each candidate (with its paired drive, and
with `--gearboxes` the fitting gearboxes for that motor's own ratio) becomes `[parts.<ID>]`, the other motors stay as
the file has them, that motor's velocity gain `drive.<ID>.velocity_kp_nm_s_rad` is retuned for it as the shared one is
for a shared part (by the inertia that motor turns and its rotor's), and the cost is that one motor's weight. On the
XYZ portal, `--motor-id mz` sizes the Z motor behind its gearbox of ratio 10 while X and Y keep theirs.
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
  `inertia_kgm2` (and `payload_inertia_kgm2`), and the bodies that ride on it ([below](#a-gantry-that-yaws-and-tools-on-rotary-axes)).
  Its `incline_deg` tilts the axis it turns about; it has no guide and no cable chain.
  Its bearing drags with the rotary's `friction_coulomb_nm` + `friction_viscous_nm_s_rad` w at the axis while
  `effects.guide_friction` is on, taken at the rotor as c / i + v w_rotor / i² (in the motor's shaft friction and its
  loss); no shipped file sets it.
- Signals and checks are named by the motor, as the joint kit's are by its joint: `joint.<motor>.force` (what the
  screw, rack, linear motor or rotary passes to its axis), `screw.<motor>.speed`, `.dn` and `.damage_rate`,
  `rack.<motor>.feed_force`, `strain_wave.<motor>.damage_rate`; `gearbox.<motor>.output_torque` is the torque ahead of
  the screw or rack. The checks are the kit's: `screw-speed`, `screw-dn`, `screw-buckling`, `screw-static`,
  `screw-life`, `rack-feed-force`, `strain-wave-life` (the lives where `ratings.required_life_h` is set). A motor that
  turns no pulley reports no `pulley.<motor>.radial_load`, so the belt-tooth and pulley-bearing checks pass it over.
- A rack's pinion loads the gearbox's output shaft sideways: the tooth force F, tangential, and the force that pushes
  pinion and rack apart, F tan(alpha_n) / cos(beta) (alpha_n = 20 degrees, beta the helix angle), at right angles to it,
  |F| sqrt(1 + (tan(alpha_n) / cos(beta))²) together: `rack.<motor>.radial_load`, judged as `gearbox-radial-load`
  against the motor's `ratings.shaft_max_radial_load_n` where that is set. The helical pinion's axial force and the
  tilt it puts on the shaft are left out. The rack gantry's X: 192.9 N of 1500.
- `effects.belt_stretch` switches the elastic model of the whole machine: with it the screws', racks' and rotaries'
  gearboxes twist too (the converter's stiffness in series), and the belt's mode at a pose includes them, with the
  rotors held.

`machines/screw-portal.toml` is the XYZ portal with its Z belt replaced by a ball screw 16 x 10 behind a bellows coupling,
the Z motor's brake holding the slide (0.055 N m of its 0.5); `machines/rack-gantry.toml` is a two-axis gantry with a
3 m X on a rack and pinion (module 2, 200 mm per turn) behind a gearbox of ratio 10 and Y on a belt. Both pass every
verdict on their cycles (`screw-speed` 2402 of 9833 r/min allowed, `rack-feed-force` 180 of 2400 N). The joint kit's
screw, rack, linear motor and rotary table axes written as files give the kit's verdicts digit for digit.

## Linkages: the planar and the spatial delta

A `linkage` node is a parallel linkage. With two arms it is planar: two arms in the vertical x-z plane, one at each
side of the base, each
turned by a motor through its gearbox and carrying a pair of forearms down to a plate that keeps its orientation (a
five-bar whose lower links are parallelograms). The tool is the plate's centre, x across and z up from the shoulders'
axis. `machines/planar-delta.toml` is the registered planar delta (`delta-d2`) written so, its values unchanged:

```toml
[[machine.axis]]           # no stroke: the linkage's working range sets the range
id = "x"

[[machine.axis]]
id = "z"

[[machine.motor]]          # and "2"; key = "motor-arm-1" names it as the crate does
id = "1"

[[machine.gearbox]]        # and gearbox_2 on motor 2
id = "gearbox_1"
motor = "1"

[[machine.linkage]]
id = "delta"
arms = ["gearbox_1", "gearbox_2"]   # the arm at +x, then the one at -x
axes = ["x", "z"]                    # across, then up
base_radius_m = "geometry.base_radius_m"
upper_arm_m = "geometry.upper_arm_m"
forearm_m = "geometry.forearm_m"
upper_arm_kg = "mass.upper_arm_kg"
# ... the other lengths, masses and the working range, see the file
```

Its fields: the geometry (`base_radius_m`, `platform_radius_m`, `upper_arm_m`, `forearm_m` longer than the upper arm,
`forearm_spacing_m` for the 3D view), the masses (`upper_arm_kg` with its `upper_arm_cg_m` and
`upper_arm_inertia_kgm2` about the shoulder, `elbow_kg`, `forearm_pair_kg` with the `forearm_elbow_share` counted at the
elbow, `platform_kg`, `payload_kg`) and the working range (`half_width_m`, `top_z_m`, `height_edge_m`,
`height_centre_m`: a band below the top whose bottom is an arc through both edges and the centre) with the highest
condition number of the Jacobian a target may have, `max_condition_number`. Each is a number or the path of one; the
planar delta's file names the crate's `[geometry]`, `[mass]` and `[workspace]`.

The model is the crate's ([model reference](model-reference.md)): inverse kinematics elbow out, forward kinematics as
the lower meeting point of two circles of the forearm's length round the elbows, the Jacobian J = J_x^-1 J_q of the
leg constraints, and the dynamics of two arms about their joints plus a point mass at the plate (each forearm pair's
mass split, the share r at the elbow): M(theta) theta'' = tau_gear + Q_g - m_nt J^T J_x^-1 w + tau_joint with
M = I_b 1 + m_nt J^T J. Each arm's joint is a `RotaryJoint`, elastic or rigid (`effects.joint_elasticity`), its drive
train from its motor's own config ([above](#motors-that-differ)), its bearing friction and stops from `[joint]`. A
target outside the working range, out of reach, beyond a joint's stop or above the condition number is refused with
the crate's reasons, and the `singularity` verdict judges the cycle against `ratings.max_condition_number`. The signals
are the crate's: `arm.<motor>.angle`, `gearbox.<motor>.output_torque`, `kinematics.condition`, the torque breakdown, loss
powers and energies; the gearboxes warm as `gearbox_<motor>`, the name of their loss.

**Three arms** make the spatial delta (Clavel's): the arms at 0, 120 and 240 degrees round the base, each with a
parallelogram of two forearms down to a platform that stays level; the tool is x, y and z, so the node names three
axes (x, y, then up) and three gearboxes. Its fields are the planar one's with the working range a cylinder about z
(`radius_m` in place of `half_width_m`, the bottom a sphere's cap through the edge and the centre), and three more:
`forearm_rod_spin_inertia_kgm2` (each rod's inertia about its own axis, with `effects.forearm_inertia`), and the upper
arms' bending with `effects.link_elasticity`, `bending_stiffness_nm_rad` and `lateral_stiffness_n_m` (required) with
`bending_damping_nm_s_rad` and `lateral_damping_n_s_m`. A planar linkage refuses the spatial values and the spatial one
`half_width_m`. `machines/delta-d3.toml` is the registered spatial delta (`delta-d3`) written so, its values
unchanged, naming the crate's `[geometry]`, `[mass]`, `[upper_arm]` and `[workspace]`.

One implementation: the delta crates are these linkages read from their own tables (`sim_netlist::linkage` and
`sim_netlist::spatial`; `crates/machine-delta-d2` and `crates/machine-delta-d3` are built on `sim-netlist`), so a fix
to the arms reaches the crates and the files at once, and each file gives its crate's numbers to the bit.

A file with a linkage has as many axes as arms (with a `key` at most: no stroke, incline or cable chain), one motor
per arm, a gearbox on each and the linkage; a pulley, section, body, guide, idler, frame, span, screw, rack, linear
motor, rotary, worm, lean or scissor beside it is refused, as is a second linkage. The effects are the crate's
(`[effects]` as in its config); a belt effect switched on (`belt_wrap`, `belt_stretch`, ...) is refused, as the
linkage has no belt to apply it to.

## A scissor lift

A `scissor` node is a scissor lift table: `stages` stages of crossed arm pairs, each arm `arm_m` from pin to pin,
stand on the base; a screw of lead `lead_m`, turned by the motor through its gearbox, pulls the sliding foot of the
lowest stage towards the fixed one and so raises the platform. The tool is the platform's height z, from
`min_height_m` to `max_height_m`.

```toml
[[machine.axis]]           # no stroke: min_height_m and max_height_m set the range
id = "z"

[[machine.motor]]
id = "lift"

[[machine.gearbox]]
id = "gearbox"
motor = "lift"

[[machine.scissor]]        # machines/scissor-lift.toml
id = "scissor"
gearbox = "gearbox"
axis = "z"
stages = 2.0
arm_m = 1.0
lead_m = 0.01
min_height_m = 0.6
max_height_m = 1.4
arms_kg = 40.0
platform_kg = 60.0
payload_kg = 200.0
screw_efficiency = 0.9     # with effects.gearbox_efficiency; default 1
screw_inertia_kgm2 = 6.47e-4
```

With u the span between the feet a stage stands s = sqrt(L² - u²) high and the platform h = n s; the screw turned by
theta moves the foot by c theta, c = lead / 2 pi. By virtual work the screw pulls the foot with
F = W dh/du = W n u / s = W n cot(alpha) for a weight W on the platform, alpha the arms' angle to the base: the lift
needs three times the force at 0.6 m that it needs at 1.4 m. The dynamics are in theta as the linkage's:
M(theta) theta'' + m h' h'' theta'² = tau_gear + Q_g, M = J_screw + m h'², Q_g = -g m_g h', m the platform, the
payload and the arms' vertical motion ((4 n² - 1) / (12 n²) of them), m_g the platform, the payload and half the
arms. The joint is a `RotaryJoint` on the motor's config, elastic or rigid (`effects.joint_elasticity`), the screw's
efficiency on the gearbox's. Signals: `gearbox.<motor>.output_torque`, `scissor.<id>.actuator_force` (the force on the
foot), the torque breakdown, loss powers and energies. A file with a scissor lift has one axis, one motor, its gearbox
and the scissor; anything else beside it is refused, as is a belt effect switched on.

Held by a constant torque the lift is unstable at every height: it needs less torque the higher it stands (gravity's
stiffness -g m_g h'' is negative), so a little too much torque lifts it ever faster; the drive's position loop holds
it. Its standard suite runs without gravity for that reason.

## Worm gears

A `worm` node makes a gearbox a worm stage: its `ratio` replaces the gearbox's, and with `effects.gearbox_efficiency`
it loses by its own efficiencies, the gearbox's stiffness, backlash and inertia staying:

```toml
[[machine.worm]]
id = "worm_c"
gearbox = "gear_c"         # the gearbox in front of a rotary table, say
ratio = 40.0
efficiency = 0.76          # driven by the worm: tan(gamma) / tan(gamma + rho')
back_efficiency = 0.5      # driven back by the wheel: tan(gamma - rho') / tan(gamma); 0 or less locks itself
```

While the motor drives it passes its spring's torque t on as t eta to the output and t / i to the rotor, as a gearbox
does (`mech::Gearbox`, the efficiency fading in over 0.5 W). Driven back by the load it passes t / eta_back: a back
efficiency of 0.5 has the rotor take half the load's power. A back efficiency of 0 or less locks it: the load cannot
turn the worm. Driven back it passes t to the output and eta_back t / i to the rotor, against the motion, so the motor
has to push to lower a load, by -eta_back of the load's torque over i, and the load's and the motor's power both go
into the worm's friction; with no motor torque a lowering load brakes to a stop. At rest it holds any load with no
motor torque (its holding torque is 0). In the rigid model its output sticks at rest while the worm can take what the
motor and the load ask of it: from -(1 / eta - 1) of the held torque (lifting) to (1 - eta_back) of it (lowering);
beyond, it breaks loose. With the same efficiency both ways it is the joint kit's rotary table with
`table.gear = "worm"`. With `effects.thermal_feedback` its efficiencies follow its temperature as a gearbox's does:
1 - eta becomes (1 - eta) (1 - s + s nu / nu_ref) both ways, a self-locking worm staying locked.

## The belt's wrap and a leaning axis

**The belt's wrap** (`effects.belt_wrap`). With it the teeth in mesh at each driven pulley are wrap r / p (p the tooth
pitch) in place of `ratings.belt_teeth_in_mesh`, in the tooth-jump law and the belt's tooth load. The wrap is worked out
at the pose from the places of the wheels in the belt's plane (x and z of the 3D view): the pulley's `at_m`, and the
wheels before and after it along the belt, the last point but one of the route of the section that ends at the pulley
and the second of the route of the section that starts there (an idler with its `at_m`, from its body where it rides on
one; a body, a clamp on it, where the 3D view draws it; another pulley). Each wheel's `clockwise` says which way the
belt runs round it, followed along the routes; the belt's straight runs are the tangents that fit, and the wrap is how
far the belt turns round the pulley between them. For the shipped crates' layouts that is `mech::wrap_angle`, half a
turn less the lean of both runs. A motor with a screw, rack, linear motor or rotary has no wrap (NaN in
`Machine::belt_wraps`; the tooth load passes it over). A file is refused where a belt pulley does not end one route
and start another, or it or a wheel next to it has no `at_m`. The screw portal with the wrap on: X's belt runs from the
bridge's clamp round its pulley and back to the same clamp, pi + 2 asin(r / D) round it.

The 3D view draws each belt section round its wheels, as the wrap sees them: from tangent to tangent between the wheels
of its route and round each wheel inside the route in the wheel's sense, and round each motor pulley between the
section that ends there and the one that starts there (`geometry::contacts`, `geometry::arc`).

**A leaning axis** (`effects.tool_flex`, elastic model): a `lean` node is a body that leans on its carriage along a
horizontal axis, as the T-gantry's Z axis does on its X carriage. Its centre of mass moves sideways by w against the
guide, which gives way sideways and in tilt: with h = `com_drop_m` less the `lever_axis`'s travel, the stiffness at the
centre of mass is k = 1 / (1 / lateral + h² / tilt), and the tool tip, h_tool below, moves
k (1 / lateral + h h_tool / tilt) per metre of w. The body moves at its axis's speed plus w' over ground, so its mass
couples the axes and w; the spring acts inside, between carriage and body, the process force on the tool tip, and as
the lever axis travels the stiffer spring pushes back on it by -w² / 2 dk/dz. The tool is at the tip (`tool_state`),
the lean's damping is `power.tool_flex`, its tip travel `axis.<axis>.tool_deflection`, and the belt's mode includes it.

```toml
[[machine.lean]]           # machines/t-gantry.toml
id = "z_lean"
body = "z_axis"
axis = "x"
com_drop_m = "geometry.z_com_drop_m"
tool_drop_m = "geometry.z_tool_drop_m"
lever_axis = "z"
lateral_stiffness_n_m = "tool_flex.lateral_stiffness_n_m"
tilt_stiffness_nm_rad = "tool_flex.tilt_stiffness_nm_rad"
damping_n_s_m = "tool_flex.damping_n_s_m"
```

The T-gantry's file has both, off as shipped (the crate has its tool flex on): its pulleys at the ends of X at the
carriage's height and its carriage idlers r + r_idler above and below, so the belt leaves both pulleys level, half a
turn at every pose, as in the crate; with either effect on it is its crate to 1e-9.

## A gantry that yaws, and tools on rotary axes

**A yawing bridge** (`machine.gantry`, `effects.gantry_yaw`). A gantry's bridge travels along a horizontal axis and
yaws a little about the vertical: its two sides, `width_m` apart, run apart by the width times the yaw angle psi (small
angles). psi is one more coordinate beside the axes. What holds it: the bridge's joints on its carriages and their
guides, a spring `stiffness_nm_rad` with `damping_nm_s_rad`; screws, racks or linear motors at its sides (`drives`,
each with its place across the bridge from the centre, so that its output travels x + side psi); and belt sections
that pull its ends (`sections`, m of length per rad: the section's pull along the axis times the end's place across).
What it turns: its own `inertia_kgm2`, and the bodies that move with the axis and along the bridge (`across`). Such a
body sits e = q_across + shift from the centre and moves along the axis by e psi, so its mass couples the axis and the
yaw (m e) and adds m e² to the yaw's inertia, through the Jacobian of its place (below); the tool on it moves with it.
Signals `gantry.<id>.yaw` (rad), `.sync_error` (width x yaw, m: how far the sides are apart) and `.moment` (k psi +
c psi', N m, what the joints carry); loss `power.gantry_<id>`; check `gantry-sync`, the highest |sync error| against
`tolerance_m`. `ConfigMachine::racking_moment` gives the racking moment at a pose and acceleration in the loss-free
rigid model: what the belts, the drives and gravity put on the yaw beyond what turns the bodies with the bridge square.
The effect is off in the H-bot, CoreXY and rack gantry files as shipped, so that their results stay, and on in the
dual-drive gantry's.

```toml
[[machine.gantry]]         # machines/dual-gantry.toml
id = "bridge_yaw"
axis = "x"
across = "y"
width_m = "gantry.width_m"
inertia_kgm2 = "gantry.inertia_kgm2"
stiffness_nm_rad = "gantry.stiffness_nm_rad"
damping_nm_s_rad = "gantry.damping_nm_s_rad"
drives = { mx1 = -0.6, mx2 = 0.6 }
tolerance_m = "gantry.sync_tolerance_m"
```

- **Two drives.** Two racks on the gantry's axis, one at each end: the file has one motor more than axes, and the two
  set x and psi together. As shipped (`gantry.sync = "plain"`) each drive follows the axis's setpoint on its own encoder
  (setpoint coupling), so the two position loops keep the bridge square, and their torques share the load by the lever
  rule: F2 - F1 = 2 m e a / B for a carriage m at e accelerated at a. With the effect off the guides hold the bridge square and the two share equally.
  `machines/dual-gantry.toml` has a 3 m X on two racks with catalogue motors (1FK2103-4AH on S210) and a 0.8 m Y belt
  on a 1.2 m bridge. It passes every verdict on its cycle: `gantry-sync` 16 um of 100, and the end the carriage starts
  near carries more (`rack-feed-force` 109 against 95 N). Its yaw rings at sqrt((k + 2 k_end (B / 2)²) / J) / 2 pi,
  155.0 Hz with J raised to 20 kg m² and the carriage in the middle, 149.5 Hz with it at an end (`sim-cli modes`).
- **Synchronising the two drives** (`gantry.sync`, `gantry.sync_gain_1_s`, read where a gantry has two drives; the
  first is the master). `"master_slave"`: the slave's position loop follows the master's actual position, its
  setpoint less the master's following error, so the slave's lag comes on top of the master's: at a steady speed v with
  the speed feedforward off and the same loops the ends run v / Kv apart, the master's following error (2 mm at
  0.1 m/s and Kv 50 1/s), where the plain scheme keeps them together. With the feedforward on both lag little, and what
  the scheme changes is how a disturbance at the master reaches the slave. `"cross_coupled"`: the synchronisation error
  e (the slave's actual position less the master's, in the slave's units) is fed back to both speed commands with the
  gain k, the slave's losing k e and the master's gaining k e; unequal loops then run apart by e0 / (1 + k (1 / Kv1 +
  1 / Kv2)) instead of e0 = v (1 / Kv2 - 1 / Kv1) at a steady speed, so k = Kv1 Kv2 / (Kv1 + Kv2) halves the yaw
  (16.7 1/s for Kv 50 and 25). Tests: `the_master_slave_sync_error_is_the_master_s_following_error`,
  `the_cross_coupled_gain_halves_the_yaw_of_unequal_loops` (sim-machines `machine_file_gantries.rs`).
- **One drive.** The rack gantry's one rack drives its bridge at one end (`drives = { mx = -0.5 }`): the joints' spring
  alone holds the far end, which lags while the bridge accelerates; with the effect on the path error of its cycle
  grows from 0.07 to 0.20 mm.
- **H-bot and CoreXY.** Belts pull the bridge's ends. In the H-bot (`sections = { "1" = -0.3, "2" = 0.6, "3" = -0.3 }`)
  the belt's moment on the bridge is -0.3 m times its pull on the head along the bridge: it racks whenever the head is
  pushed along the bridge, wherever the head is (-4.2 N m for the 7 kg head at 2 m/s²), and a head off the centre racks
  it as the bridge accelerates (-m e a). A rigid belt lets the H-bot rack (the head can move along the bridge with the
  motors held), so it racks in both models; with the effect on the path error of its cycle grows from 0.09 to 0.80 mm.
  In the CoreXY each belt pulls both ends: its moment is the one belt's tension sum against the other's, -0.19 N m for
  the 8 N pull that would rack an H-bot with -2.4, and its path error stays 0.06 mm. A rigid belt holds a CoreXY's yaw:
  its rigid model has none.

**Bodies on rotary axes, and the tool centre point.** A body may move with rotary axes: `offset_m` is its centre of
mass from where their axes meet at angle 0, and it rides round on them, the first in its `axes` outermost. A rotary
axis's `incline_deg` tilts the axis it turns about from the vertical (0, a C axis) towards x (90, an A axis). Such a
body's place p(q) is the straight axes' travels plus its offset turned by its rotaries; its share of the mass matrix is
m J^T J (J = dp/dq, by central differences), of gravity -m g J_z, and of the forces its speed asks -m J^T (dJ/dt q'),
the centripetal and Coriolis forces: Lagrange's equations, so the energy keeps. A `tcp` node makes the tool's
coordinates those of the tip: three straight axes along x (the file's first), y and z, the `rotaries` that carry the
tool, and `tool_m`, the tip from where their axes meet. The axes stand at the tip less `tool_m` turned by the rotaries,
so a path moves the tip straight while the rotaries turn, the straight axes making up for them, and the feedforward has
the speed's part (`feedforward_torque_moving`). The tool's ranges are where its tip can be whatever the head's angles
(the strokes shifted by `tool_m` turned by the rotaries, at their least), and a pose beyond the straight axes' strokes
is refused (`error-tcp-joint-range`). The 3D view draws the tool from the rotaries to the
tip, and the tool's path at the tip.

```toml
[[machine.tcp]]            # machines/five-axis.toml
id = "tool_tip"
axes = ["x", "y", "z"]
rotaries = ["c", "a"]      # C about the vertical carries A about x
tool_m = [0.0, 0.0, -0.2]
```

`machines/portal-c.toml` is the screw portal with a C axis on its Z slide turning a gripper whose tip is 0.1 m off the
axis: its cycle carries a part while C turns it a quarter turn, the tip on a straight line. `machines/five-axis.toml` is
the screw portal with a fork head, C about the vertical and A about x inside it, the tip 0.2 m below where they meet: its
cycle tilts A and swings C half a turn round the still tip, X and Y running on a circle, and the spindle's weight on A.
Both pass every verdict on their cycles. Their rotaries' gearboxes, inertias and the heads' masses are estimates.

**A rotary table that carries the workpiece** (`table`, `table_pivot_m`). The tool's x, y and z are then the tip's
place on the workpiece, as it lies at the table's angles 0; a point w of it lies in the room at p + R_table (w - p), p
where the table's axes meet and R_table its rotaries' turn (the outermost last). So the straight axes stand at
q = p + R_table (w - p) - R_head `tool_m`: a path on the workpiece is cut while the table turns it, the straight axes
making up for the table, and the head's rotaries (`rotaries`) may tilt the tool beside it (head/table). The tool's
ranges are the box about p as wide as p's least distance from an end of X's and Y's strokes, the workpiece the table
can turn under the tool; a pose beyond the straight axes' strokes is refused (`error-tcp-joint-range`).
`machines/five-axis-table.toml` is the screw portal over a trunnion: A about x carries C, the table, about its own axis,
meeting at (0.5, 0.25, -0.2) m, the tool hanging 0.2 m below the Z slide. Its cycle tilts the table with the tip held on
a point of the workpiece, turns the table half a turn under the still tip (X and Y on a circle round C's axis), and
moves the tip on the workpiece with all five axes. The trunnion and the table ride on A and C (`offset_m`), so tilting
A lifts their weight on A's motor. Test `a_table_table_tcp_keeps_the_tool_on_a_line_while_the_table_turns`: the tip on
a straight line of the workpiece while C turns 2 rad at A 0.3 rad, every motor against its hand-worked angle; a
head/table variant (A in the head, C in the table) in
`a_head_table_tcp_turns_the_tool_in_the_head_and_the_workpiece_on_the_table`.

```toml
[[machine.tcp]]            # machines/five-axis-table.toml
id = "tool_tip"
axes = ["x", "y", "z"]
table = ["a", "c"]         # A about x carries C, the table
table_pivot_m = [0.5, 0.25, -0.2]
tool_m = [0.0, 0.0, -0.2]  # without a head: from the straight axes' point
```

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
  screw, rack or rotary per gearbox, at most one of these per axis (but two at a gantry's sides), a screw at least as
  long as its stroke, as many motors as axes and one more per gantry drive beyond the first, a gantry's drives on its
  axis, a tool centre point's axes along x, y and z, a body's `offset_m` on rotary axes only, a routing that keeps the
  belt's length and sets every axis;
- every other key against the keys the registered machines' configs have (so `pretensoin_n` is refused by name), plus
  `[axis.<id>]` for each axis, a friction table for each guide, the value tables of screws, racks, linear motors and
  strain wave gears laid out as the joint kit's, and every key the netlist names by path;
- then everything the simulation reads at its start, as for a config: the drive, the cycle, the start pose, the
  catalogue parts the file names.

An error names the file, the line and the key, in the front end's language:

```
$ sim-cli verdicts --machine-file broken.toml
Configuration has a value this machine does not know: broken.toml:674: machine.guide[guide_z].side_lod
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
  bodies' mass matrix is diagonal, but for the bodies on rotary axes or across a yawing bridge, which add m J^T J, their
  share of gravity and the forces of their speed ([above](#a-gantry-that-yaws-and-tools-on-rotary-axes)). Pulleys and rotors (J_pulley + i² J_rotor per motor, the belt's mass shared out on
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
  rotary axis's mass is its rotary's inertia and what rides on it. A drive at a yawing gantry's side has the row
  e_i + side e_psi.
- A gantry's yaw (`machine.gantry`, `effects.gantry_yaw`): one more coordinate psi after the axes, with its spring and
  damper, the belt sections' yaw coefficients in C, the drives' sides in K. Where fewer motors than coordinates set it
  (one drive, or belts), the motors set the axes and the spring the yaw (`forward` and the feedforward take the axes'
  rows); where belts cannot keep their length with it (a CoreXY), the rigid model has no yaw.
- A tool centre point (`machine.tcp`): the tool's pose maps to the axes and back through the rotaries' turn of
  `tool_m`; speeds and accelerations by differentiating that map.
- A worm stage (`machine.worm`): its gearbox's output gets t eta^x and its rotor t / i, x = tanh(t w / 0.5 W), with the
  back efficiency where the load drives (x below 0); a self-locking one gives the rotor x t / i driving and
  -x eta_back t / i driven back, against the motion. In the rigid model, with a self-locking worm, the gearboxes'
  outputs are solved for beside the accelerations; driven back its drag at the wheel is (1 - rotor factor) times the
  torque the output holds at rest against the axes' forces (a drag in proportion to the dynamic torque has no
  consistent rigid solution there), and at rest the output sticks within the worm's bounds
  (`ConfigMachine::rigid_locking`).
- A leaning body (`machine.lean`, elastic model): one more coordinate w beside the axes, the body's mass coupling it to
  its axis, [[M, m e_a], [m e_a^T, m]] [a; w''] = [F; tip p_a - k w - c w'], k and tip from `geometry::flex_at`.
- The belt's wrap (`effects.belt_wrap`): `geometry::wrap` at each pulley from its neighbours' tangents, at the pose.
- A linkage of two or three arms builds a `LinkageMachine` or a `spatial::Delta`, a scissor a `ScissorMachine`, not a
  `ConfigMachine`: their models are above.

The controller, drives, DC link, winding heat, analyses, verdicts, sensitivity and error bands are those of every
machine; nothing about them is particular to machine files.

## What machine files cannot do yet

- A pulley stands where its `at_m` puts it (it rides on no body), and the wrap's places are the 3D view's: the section
  lengths are not checked against them.
- `tool_flex`: one leaning body, along a horizontal axis; it leans in the elastic model only (as in the crate).
- A self-locking worm's lock fades in over the gearbox's 0.5 W once it moves: from rest it sticks (rigid model), but a
  motor torque short of lifting the load, or a push short of lowering it, creeps it at the speed whose power is about
  0.5 W atanh(the share of the torque it would take). In the elastic model the stick is the fade alone.
- A rotary's bearing friction acts at the rotor, before the gearbox's spring and efficiency, as a screw's preload does.
- A rack's pressure angle is 20 degrees for every rack, and its pinion's axial force is left out of the shaft's load.
- The scissor lift's arms count with their vertical motion only (their turning and the feet's sideways motion are left
  out of its mass), it has one screw on the lowest stage's foot (no cylinder between the arms), and it does not mix with
  belts, screws or bodies. A linkage's or scissor lift's file cannot carry other mechanisms beside it. The spatial
  delta's modes take its first arm's gearbox for all three.
- A gantry's yaw: one per file, small angles (the sides move by the width times the angle); the drives each follow the
  common setpoint, so there is no master/slave or cross-coupled synchronisation in the drives (the controller's to add)
  and the sync error is what two equal position loops leave. A section's yaw coefficients are numbers from its routing,
  not worked out from its `route`; a body on the bridge that does not move across sits at its centre; idlers and spans
  do not follow the yaw.
- A body on rotary axes is a point mass at its centre of mass: its own inertia about that is in the rotary's
  `inertia_kgm2`. A tool centre point needs three straight axes square to each other (the 3D view's x, y, z) and the
  rotaries on the tool's side (a head): a rotary table carrying the workpiece is not one yet. The analysis's settling
  and path errors add a rotary axis's error in rad to the straight axes' in m.
- `effects.belt_tooth_jump` together with a screw, rack, linear motor or rotary: its states are laid out per motor, and
  a motor without a pulley has none; the file is refused.
- A screw's stiffness is the one at its softest nut position for the whole stroke, as in the joint kit; its preload's
  drag sits at the motor with a smoothing in rad/s.
- Modes other than the belt's: the frame enters the belt's mode where it sways, but the guides' and the motors' own
  stiffness do not, and a body's bending is not modelled.
- Section and span lengths are numbers (or paths), not formulas: one whose length depends on a stroke does not follow a
  change of that stroke by itself.

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
1e-9 (`the_effects_of_a_machine_file_are_its_crates`); since 14 November also with the leaning Z axis (`machine.lean`)
and the belt's wrap from the places of its pulleys and idlers, whose wraps agree too. The effects' own hand checks are
the crate's.

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
rad, a rotary's bearing friction at the motor ((2 + 0.5) / 100 = 0.025 N m), the rack pinion's radial load on the
gearbox shaft (37.5745 N for 35.05 N on the teeth); both new files pass the standard suite; and a broken screw is
refused by key and line.

`crates/sim-machines/tests/machine_file_linkage.rs` holds the linkage: `machines/planar-delta.toml` is the planar delta's
crate to 1e-9 in its default config and with each of its 36 effects switched from its default, one at a time (axes and
motors, kinematics, rest state and holding torque, the derivatives of a state moved off rest, energy, both
feedforwards, every signal, the parts' warmth, what the working range refuses); its cycle, elastic, rigid and with
every effect that is off by default switched on, gives the crate's signals at every 16th step of the first half second
and the crate's motor loads and verdicts, to 1e-9 (the two sum in the same order, so they agree to the bit, encoder
counts and all); the file passes the standard suite with the crate's loss-free configs; by hand, the arm angles at
(0.1, -0.5) m (-asin 0.2 = -11.537 degrees and 16.474 degrees) and the holding torque at the centre (-18.358 N m at
each gearbox); and a broken linkage is refused by key and line (a belt effect switched on and a spatial value on the
planar linkage among them). `machines/delta-d3.toml` is the spatial delta's crate the same way, with each of its
effects switched from its default, and its cycle gives the crate's signals and verdicts to 1e-9. The scissor lift's
holding torque and the force on its foot at 0.6 and 1.4 m are the hand's (W n cot(alpha): 17462.6 and 5602.7 N), as
its force ratio n cot(alpha) at two heights is (`scissor.rs`), and its file passes the standard suite without gravity.

`crates/sim-machines/tests/machine_file_geometry.rs` holds the belt's wrap from the places of a pulley's neighbours by
hand at two poses (an offset idler and a clamp that is not level), the worm gear against the joint kit's table with its
worm (rigid and elastic, driving and driven back), a self-locking worm's hold and a turning worm's back-driven
acceleration by hand, the self-locking worm braking a lowering load with no torque, needing 0.05 N m of push to lower it
and sticking from rest short of what moves it, the belts' wraps of the screw portal beside its Z screw, and the
catalogue search for the XYZ portal's Z motor alone.

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
