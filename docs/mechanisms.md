# Mechanisms and transmissions

Beside the belt machines the simulator has machines built from one motor-driven joint per axis: a ball screw axis, a
rack and pinion axis, a linear motor axis, a rotary indexing table, a six-axis arm and a SCARA arm, both read from URDF
files. Their parts are components in `sim-components`, so that a machine described in a config file can wire them
later; the six machines live in the crate `machine-joint-kit` and are run and sized like every other machine.

The German version of this document is [de/mechanismen.md](de/mechanismen.md).

- [The machines](#the-machines)
- [How they are built](#how-they-are-built)
- [Ball screw](#ball-screw)
- [Rack and pinion](#rack-and-pinion)
- [Linear motor](#linear-motor)
- [Rotary table: strain wave and worm gear](#rotary-table-strain-wave-and-worm-gear)
- [Arms from URDF files](#arms-from-urdf-files)
- [What is checked by hand](#what-is-checked-by-hand)
- [Limits](#limits)

## The machines

| Id | Mechanism | Transmission | Tool pose | Own checks |
| --- | --- | --- | --- | --- |
| `screw-axis` | carriage, 800 mm | ball screw 25 x 20 behind a coupling | x (m) | `screw-speed`, `screw-dn`, `screw-buckling`, `screw-static`, `screw-life` |
| `rack-axis` | carriage, 3 m | helical pinion on a rack behind a planetary gearbox of ratio 10, backlash-free | x (m) | `rack-feed-force` |
| `linear-motor-axis` | carriage, 1 m | ironcore linear motor, none | x (m) | `guide-load`, `guide-life` from the attraction |
| `rotary-table` | table of 3 kg m^2 plus the work | strain wave gear of ratio 100 (or worm, or planetary) | c (rad) | `strain-wave-life` |
| `urdf-arm` | six-axis arm, `urdf/six-axis.urdf` | strain wave gears of ratio 120 and 80 | six joint angles (rad), or x, y, z, rx, ry, rz in tool space | `strain-wave-life` per joint |
| `scara` | SCARA arm, `urdf/scara.urdf` | strain wave gears; the quill on a ball screw | joints (rad, m, rad), or x, y, z, c in tool space | `strain-wave-life`, the screw's checks on joint 3 |

Every one of them also gets the checks every machine gets: motor peak and RMS torque (`motor-peak`, `motor-thermal`),
drive currents, gearbox peak, cubic-mean torque, input speed and life where `[ratings]` gives them, DC link and braking
resistor, settling. `sim-cli list` names them, and every command takes them:

```text
sim-cli verdicts screw-axis
sim-cli run rack-axis --set rack.service_factor=1.5
sim-cli margins linear-motor-axis
```

Each default config is a sized example: its cycle passes all its verdicts (test
`every_default_cycle_passes_its_verdicts`). Its values are marked `[catalogue]` where they come from a maker's document
(the Siemens 1FK2105-4AF motor and the S210 drive the delta picker uses) and `[estimate]` everywhere else: a screw, rack,
linear motor or strain wave gear of this size as the makers' catalogues list them, not one maker's article. Replace them
with your part's data sheet.

## How they are built

A joint machine (`chain::JointMachine`) is a mechanism plus one `joint::RotaryJoint` per joint. The mechanism gives the
load's dynamics in joint coordinates, M(q) q'' + h(q, q') = tau, and its own signals, rating checks and scene. The joint
gives everything between the motor and the joint: motor friction, ripple and cogging, holding brake, the gearbox
(ratio, torsional stiffness, damping, backlash, efficiency), friction in the joint and its end stops. With
`effects.joint_elasticity` the motors are bodies of their own behind the gearboxes' springs; without, the rigid model,
the rotor turns with the joint times the ratio and each gearbox loses its efficiency on what passes through it.

A joint's coordinate is an angle (rad) or a travel (m). A screw or rack turns the gearbox's output rotation into travel
(`transmission::Converter`): put behind the gearbox, the pair becomes one gearbox in N and m with

- ratio i / lever, the lever the travel per radian (lead / 2 pi, or the pinion's pitch radius);
- stiffness 1 / (lever^2 / k_gearbox + 1 / k_converter): the gearbox's torsional stiffness seen at the joint, in series
  with the screw's or rack's own; damping likewise over lever^2;
- backlash: the gearbox's seen at the joint plus the converter's (with `effects.gearbox_backlash`);
- efficiency: the product (with `effects.gearbox_efficiency`);
- the screw shaft's or pinion's inertia added at the rotor over i^2.

The signal `gearbox.<motor>.output_torque` is the joint force times the lever, over the converter's efficiency while
the motor drives and times it while the load drives back, so the gearbox ratings judge the torque at the gearbox
output, before the screw or pinion. The converter's efficiency is kept apart from the gearbox's
(`RotaryJoint::extra_efficiency`): with `effects.thermal_feedback` the gearbox's temperature changes only its own
share. A sliding joint reads its guides' friction, stroke and stops from
`[slide]` (`RotaryJoint::linear_from_config`).

## Ball screw

`transmission::BallScrew`, config `[screw]`. The formulas are those of ISO 3408-5 and of the screw makers' catalogues:

| Quantity | Formula |
| --- | --- |
| Critical speed | n_cr = 60 / (2 pi) (lambda L)^2 / L^2 sqrt(E I / (rho A)), a uniform steel beam of the root diameter; lambda L = 1.875, pi, 3.927, 4.730 for fixed-free, supported-supported, fixed-supported, fixed-fixed. Allowed: `critical_speed_factor` (0.8) times it. |
| Buckling | F_k = factor pi^2 E I / L^2, factor 1/4, 1, 2.046, 4; allowed F_k / `buckling_safety_factor`. |
| Static load | C0a / `static_safety_factor`. |
| d n | nominal diameter (mm) times speed (r/min), against `max_dn` (0 judges none). |
| Axial stiffness | shaft A E / x (fixed-fixed: A E L / (x (L - x))), nut and fixed bearing in series; taken at the softest nut position. |
| Preload drag | T = K F_pr l / (2 pi), K = 0.05 / sqrt(tan beta) (THK); added to the guides' friction as T 2 pi / l. |
| Life | L10 = (Ca / (fw F_m))^3 10^6 turns. A preloaded nut carries F_pr (1 + F / (2.83 F_pr))^(3/2) until the force lifts the preload (Hiwin). |

The screw's signals: `screw.1.speed` (r/min), `screw.1.dn`, `screw.1.damage_rate` (the share of its life used per hour,
60 |n| (fw F_eq / Ca)^3 / 10^6). The life a check reports is one over the damage rate's mean over the measured pass
(Palmgren-Miner), times the life factor a1 of `ratings.bearing_reliability_percent` (0.25 at 99 %, as for every
bearing life), against `ratings.required_life_h`. The axial force checks take |force|, pulling too, which is on the
safe side for buckling.

A vertical axis: `slide.incline_deg = 90` (gravity pulls along the travel by the sine of the incline).

## Rack and pinion

`transmission::RackPinion`, config `[rack]`: normal module, teeth and helix angle give the pitch radius m z / (2 cos
beta). The check `rack-feed-force` holds the highest force on the teeth (signal `rack.1.feed_force`: the force the load
gets over the rack's efficiency while the pinion drives, times it while the load drives back) against
`rated_feed_force_n` over the application factor K_A (`service_factor`). Rack makers publish the permissible feed force of a pair, not a life formula, so the rack
has no life check; the planetary gearbox before it has its own (`gearbox-life`).

## Linear motor

No transmission: the joint's gearbox has ratio 1 and stands for the forcer's mounting (its stiffness in N/m). The
motor's `[motor]` and `[drive]` values read in N for N m and m for rad: force constant in N/A, the forcer's mass as the
rotor's inertia (not part of `mass.carriage_kg`), speeds in m/s, encoder counts per 2 pi m. The cogging repeats once per pole pitch: `motor.cogging_order`
is replaced by 2 pi / `linear_motor.pole_pitch_m`.

The attraction between forcer and magnet track (`attraction_force_n`) presses on the guides: the signal
`axis.x.guide_load` is the attraction plus the weight of carriage, payload and forcer across the guide, and the guide checks (`guide-load` against
`ratings.guide_max_load_n`, `guide-life` from `ratings.guide_dynamic_load_rating_n`, L10 = (C / P)^3 50 km) judge it.
`guide_friction_coefficient` times the attraction is added to the guides' Coulomb friction.

The scene labels the motor's speed in m/s; it has no shaft to turn.

## Rotary table: strain wave and worm gear

The table turns about a vertical axis: M = table plus work inertia, no gravity torque. `table.gear` selects the gear:

- `"strain-wave"`: `transmission::StrainWave`, Harmonic Drive's catalogue method: L10 = L_n (T_r / T_av)^3 (n_r /
  n_av), T_av the cubic mean output torque weighted with the input speed. Signal `strain_wave.1.damage_rate`, check
  `strain-wave-life`.
- `"worm"`: `transmission::WormGear` gives the efficiency from lead angle gamma, flank friction mu and normal pressure
  angle alpha_n: tan gamma / tan(gamma + rho'), rho' = atan(mu / cos alpha_n). Its back efficiency, tan(gamma - rho') /
  tan gamma, is computed (zero or below: the gear locks itself) but not yet used by the dynamics, see the limits.
  The worm's efficiency is kept apart from the configured gearbox's, as a converter's is. A machine file's `worm` node
  is this drive with a back efficiency of its own, which locks itself at 0 or less ([machine files](machine-files.md#worm-gears)).
- `"planetary"`: the `[gearbox]` as it is.

## Arms from URDF files

`rigid::Chain` reads a URDF file (links with their inertials, joints of type revolute, continuous, prismatic and fixed;
fixed joints merge into the body before them; no xacro, no branches) and computes the dynamics by the recursive
Newton-Euler algorithm (Luh, Walker and Paul 1980): the joint torques for given accelerations, the mass matrix column by
column, gravity and velocity terms. The payload is a point mass at `arm.payload_offset_m` from the URDF's tip frame.

Config `[arm]`: `urdf_file` is a path, or `builtin:six-axis` / `builtin:scara` for the examples in
`crates/machine-joint-kit/urdf/`. `gear_ratios` has one entry per joint; each joint's limits are in `[axis.j1]`,
`[axis.j2]`, ... (`max_velocity_rad_s`, or `max_velocity_m_s` for a prismatic joint, and so on). The URDF's limits are the joints' stops; a continuous joint takes `[joint]` `min_angle_deg` and
`max_angle_deg`. A prismatic joint slides on the `[screw]`, coupled to its motor (its ratio must be 1; the
`[gearbox]`'s stiffness and damping stand for the coupling, its efficiency and inertia do not count), with the `[slide]`
friction (the SCARA's quill). A URDF limit left out is 0; an empty range is refused. `ratings.gearbox_reference_speed_rpm`
is refused where the joints' ratios differ, as the gearbox life takes every motor's speed through `gearbox.ratio`. With a
`[strain_wave]` table each revolute joint's gear gets the life check.

The tool pose is the joint positions: cycles are given in joints and the sizing judges motors, gears and screws along
them. The settling tolerance is then in rad (`ratings.settling_tolerance_m` keeps its name); it counts the gears' twist
under the arm's weight, which the motors' encoders do not see (2.6 mrad at joint 2 of the six-axis example reaching
out).

### Tool space

With `kinematics.tool_space = true` the machine's axes are the flange's pose, and every planner, the cycle, the CLI
and the app move the tool as they move a delta's: a line of a path is a straight line of the tool. The joint machine
maps the pose to the joints through the mechanism's `inverse` and back through `forward`, the tool's speed to the
joints' through the Jacobian of `forward` (central differences). The controller's feedforward then takes the speed
terms: the joints' accelerations J^-1 (a - J' q'), J' q' the second difference of `forward` along q', and the
centripetal and Coriolis torques of the Newton-Euler model (in joint space it leaves them out, as before). Targets are
refused out of reach (`error-arm-out-of-reach`), beyond a joint's stop (`error-joint-beyond-stop`) or where the
Jacobian's condition number is above `workspace.max_condition_number` (`error-arm-singularity`); a path through such a
pose with `error-path-range`, as on the deltas. The workspace map and `ratings.max_condition_number` take the same
number.

- SCARA (four joints: two arms about vertical axes, a vertical quill, a turning flange): the axes are x, y, z of the
  flange (m) and its turn c about the vertical (rad: joints 1 + 2 + 4 plus the turn at zero), in closed form. The arms'
  lengths and directions at zero, joint 1's place, the quill's direction and the flange's turn come from the URDF at
  zero and are checked against the chain at three poses (a URDF that is no SCARA is refused). Joint 2 by the cosine law,
  cos(beta) = (r^2 - L1^2 - L2^2) / (2 L1 L2), with `kinematics.elbow` "right" (beta > 0: joint 2 counter-clockwise
  from the stretched arm, the elbow to the right of the line from the base to the tool, seen from above) or "left";
  joint 1 = atan2(y, x) - atan2(L2 sin beta, L1 + L2 cos beta). The condition number is that of d(x, y)/d(q1, q2):
  infinite at the stretched and the folded arm (25 at 11 degrees of elbow, the example's limit).
- Six joints: x, y, z of the flange (m) and its orientation as a rotation vector rx, ry, rz (rad: axis times angle,
  from the base frame), numerically. The joints are followed from `kinematics.home_joints` along the straight line from
  the home's flange pose to the target, the orientation by slerp, in steps of at most 20 mm and 0.05 rad, each by
  damped least squares on the geometric Jacobian (dq = J^T (J J^T + lambda^2 I)^-1 e, lambda 1 mm, the turn rows times
  the flange's distance from joint 2 at the home), settled to 1 um, the last to 1e-13 m. So the solution stays on the
  home's branch (shoulder, elbow, wrist; on an arm whose branches are parted by singularities, as the usual ones are,
  it does not depend on the way there), and a step that does not settle (out of reach, or a singularity the branch
  cannot pass) refuses the pose. A flange turned within 0.01 rad of half a turn from the base frame is refused
  (`error-arm-orientation-half-turn`): there the rotation vector jumps. The condition number is that of the 6 x 6
  geometric Jacobian with its turn rows scaled as above.

The planners interpolate each axis linearly: a line holds the orientation where both ends have the same, and turns it
by the slerp where the two rotation vectors are parallel (for a flange pointing down, a turn about the vertical is not:
there the turn is close to the slerp, not it). `config/scara-tool.toml` and `config/arm-tool.toml` are the default
configs with tool space on, a start pose and a cycle of straight lines (the arm's settling tolerance 5 mm: under the
payload its flange rests 4.0 mm below the encoders' pose).

## Telescopic shaft with two cardan joints (delta-d4)

The four-axis delta `delta-d4` (its own crate, built on the D3, not a joint machine) turns its tool through a telescopic
shaft with a cardan joint at each end. One cardan joint bent by beta turns its output unevenly: tan chi = tan phi /
cos beta, the speed between cos beta and 1 / cos beta of the input's, twice a turn. Two joints bent alike in one plane,
the intermediate shaft's yokes in phase (the Z arrangement), cancel: the second gives phi back. On the delta both the
motor's shaft and the tool's are vertical (the platform stays level), so the pair cancels at every pose and the tool
turns with the gearbox's output; only the shaft between turns unevenly. The shaft twists in series with the tool's
gearbox (`effects.shaft_torsion`), its weight share loads the arms (`effects.shaft_mass`) and the telescope's sliding
friction pulls on the platform along the shaft (`effects.telescope_friction`). See the
[model reference](model-reference.md#delta-picker-d4).

## Crank and cam mechanisms (`machine-crank-cam`)

The core cases of a servo sizing tool (Siemens SIZER) that a joint alone does not cover: a crank that turns a rotary motion
into a stroke, and two axes that work on a moving web or bar through an electronic cam. Each is one motor-driven joint
(`chain::JointMachine`), so the motor, gearbox, drive loops, heating and every common check are the same as above.

| Id | Mechanism | Transmission | Tool pose | Own checks |
| --- | --- | --- | --- | --- |
| `slider-crank` | servo press: crank 50 mm, rod 200 mm, ram 15 kg and die 10 kg below it, press force up to 3 kN | planetary gearbox of ratio 20 | crank angle (rad), 0 at the bottom dead centre | `rod-force`, `slider-guide-load`, `crank-bearing-load` |
| `rotary-knife` | cross cutter: knife roll of 100 mm radius, one blade, cuts a web at 1 m/s into 0.5 m formats | planetary gearbox of ratio 10 | roll angle (rad), 0 with a blade at the cut | |
| `flying-saw` | carriage 40 kg with a saw of 25 kg, material at 1 m/s cut into 1.5 m lengths in a window of 0.2 m | helical pinion on a rack behind a planetary gearbox of ratio 5 | x (m) | `rack-feed-force` |

**Slider-crank.** The slider's distance from the crank's centre is x(θ) = r cos θ + √(l² − r² sin² θ), its speed ratio
x'(θ) = −r sin θ − r² sin θ cos θ / w with w = √(l² − r² sin² θ), and x''(θ) = −r cos θ − r² cos 2θ / w −
r⁴ sin² θ cos² θ / w³. The rod's mass is split into a share at the crank pin (`rod.crank_end_share`) and the rest at the
slider. The inertia the crank feels changes with the angle: J(θ) = J_crank + m_pin r² + m_x x'², m_x the slider, the die
(payload) and the rod's slider end; its speed term is m_x x' x'' θ'² (dJ/dθ θ'² / 2), which the feedforward takes too.
Gravity along the guide (`slider.incline_deg`, −90 for a ram below the crank) and on the pin. The process force
(`effects.process_force`) is a table over the ram's distance from the bottom dead centre (`[process]`); it pushes back
while the ram moves out (forming), not on the way back, and reaches the crank as −F x'(θ): near the dead centre a large
force costs little torque. Signals: the ram's position, speed and acceleration, the process force, the rod force
T = (F_along − m_x ẍ) l / w, the guide's load across |m_x g cos β − T r sin θ / l|, the crank bearing's load (the rod's
pull plus the pin's mass on its circle) and J(θ). Judged against `ratings.rod_max_force_n`,
`slider_guide_max_load_n` and `crank_bearing_max_load_n`.

**Rotary knife.** The cam's master is the web, in metres. In the synchronous zone ±α about each cut
(`knife.sync_angle_deg`) the blade's tip runs with the web, slope 1 / R; between cuts a motion law takes the roll the rest
of a blade pitch, 2π / n − 2α, over the rest of the format, L − 2Rα. A format shorter than the circumference over n speeds
the roll up between cuts, a longer one slows it: on average (2π / n − 2α) / (L − 2Rα) rad per metre of web, and with
3-4-5 at mid-span 1 / R + 1.875 × (that − 1 / R). The cut (`effects.process_force`) brakes the roll with a force at the
tip over the angle from the cut (`[cut]`).

**Flying saw.** The cam's master is the material. Per cut length: the carriage comes up to the material's speed v while
the material runs v² / a (`saw.sync_acceleration_m_s2` = a, the mean acceleration), its speed blending as 3u² − 2u³ (no
jump in acceleration, peak 1.5 a), so it covers the synchronisation distance v² / (2a); it runs with the material through
the cut window W for W / v seconds; it leaves the material over another v² / (2a), and returns over the stroke W + v² / a
by a motion law in the rest of the cut length, L − 2v² / a − W of material. A cut length that leaves no room is refused
(`saw.cut_length_m`). The cut pushes on the carriage along the material over the way into the window (`[cut]`).

**Electronic cam.** `sim_components::cam` holds a cam as the slave's position over a master's, periodic, the slave rising
by its advance per master period (a crank or knife one turn, the saw 0). Segments take the motion laws of VDI 2143:
`linear` (a synchronous run), `dwell`, `3-4-5` and `4-5-6-7` (the 5th and 7th order polynomials), `cycloid` and
`harmonic`. Where a law meets a linear segment or a dwell it takes its neighbour's slope and has no acceleration there
(velocity in velocity); between two laws the slave rests (rest in rest). A cam table (`cam.table_file`, CSV of master,
slave and an optional slope) runs a cubic through its points: with the slopes given between each two points, without them
the periodic cubic spline (the slave's acceleration continuous). With `cam.run = true` the controller runs the cam as the
cycle instead of `[[cycle.segment]]`: the master comes up to speed by a sin² ramp in `cam.ramp_s`, the slave's speed is
s'(m) m' and its acceleration s''(m) m'² + s'(m) m''; a pass is one master period at speed, so `analyze` and `verdicts`
measure the steady state; a stop runs the master down over the same ramp. A cam that asks more slave speed, acceleration or
jerk at the master's speed than the axis limits allow is reported as `limit-cam-speed`, `-acceleration`, `-jerk` (with the
motor limits). The slider-crank runs either at constant speed (`crank.motion = "constant"`, one turn per stroke, as a
mechanical press's flywheel) or by the servo-press cam of `[[cam.segment]]` (fast approach, slow through the forming
zone, fast return) at `crank.strokes_per_min`.

The shipped configs keep `cam.run = false`: the cycle swings the crank from the top dead centre down to the bottom and
back (a servo press in pendulum mode), indexes the knife by half a turn and back, moves the saw's carriage over its
stroke. Run the cams with `--set cam.run=true` (app: the cam group's switch):

```text
sim-cli verdicts slider-crank --set cam.run=true
sim-cli analyze rotary-knife --set cam.run=true --set knife.format_length_m=0.4
sim-cli verdicts flying-saw --set cam.run=true --set material.speed_m_s=1.2
```

The torque and following error come out of the drive model as on every machine; the cams' and the mechanisms' formulas are
checked by hand in `cam.rs` and `machine-crank-cam`'s tests (below).

## What is checked by hand

Component tests (`sim-components`): the screw's critical speed, buckling load, inertia, stiffness, preload drag, life
and its signals and checks; the converter behind a gearbox (ratio, series stiffness, backlash, efficiency, inertia);
the slide's friction and stops; pitch radius and feed force of the rack; worm efficiencies; strain wave life; the
textbook pendulum and energy conservation of the Newton-Euler chain.

Machine tests (`machine-joint-kit`): every machine passes the standard suite (round trip, static hold, energy
conservation without losses, energy budget with them, step convergence, determinism) and its default cycle passes all
verdicts. On the rigid, loss-free models: the screw reaches 2400 r/min at 0.8 m/s and pushes with the mass times the
acceleration the run reached; the rack's feed force and gearbox torque are m a and m a r; the linear motor's guides carry
2500 + 13 g = 2627.49 N; the table's gear passes J alpha; the six-axis arm reaching out holds 177.304 N m at joint 2
(-1.47753 N m at its motor) and 21.3785 N m at joint 3, summed from the URDF's masses by hand; the SCARA's screw holds
5.5 g = 53.937 N, -0.171686 N m at its motor. In tool space: the SCARA's closed form at (0.325, 0.275) gives the elbow
at ±pi/2 and joint 1 at 0 or 1.404514 rad by hand; forward of inverse returns poses within 1e-9 (the SCARA's whole
range, the arm's sampled around its home, its wrist on the home's side); every planned pose of a line lies within 1 um
of it and the arm's orientation within 1e-6 rad; the joint speeds are J^-1 v by hand (1.538462 rad/s); a line into the
stretched SCARA arm is refused; the time-optimal plan keeps the motors within their rated speed along a line; both tool
cycles pass their verdicts. A screw 1.6 m long fails `screw-speed`, and a life asked for beyond the
screw's fails `screw-life`. With losses: the rack's gearbox torque and feed force are the load's force over 0.97, the
screw's life shrinks by a1 at 99 % reliability, and the temperature feedback keeps the efficiency of screw, rack and
worm.

Crank and cam tests (`sim-components` cam.rs, `machine-crank-cam`): a 3-4-5 segment of rise 0.1 m over 2 rad at 4 rad/s
peaks at 1.875 h V / T = 0.375 m/s, 5.7735 h V² / T² = 2.3094 m/s² and a jerk of 60 h V³ / T³ = 48 m/s³; every law rises
from rest to rest with rates that are its differences; a law between two runs of slope 10 keeps it at its ends and has
10 + 1.875 (12 − 10) at mid-span; a sine table with slopes, and its periodic spline without, run through the sine; the
master's sin² ramp covers ramp V / 2. The slider-crank at 60° (r 50 mm, l 200 mm): x = 0.2202562 m, x' = −0.0488454 m/rad,
x'' = −0.0187556 m/rad²; J = 0.0869757 kg m², and turning back at 2 rad/s, speeding up by 10 rad/s² against 1 kN,
−47.878563 N m at the crank; at −90° the rod pushes with 1032.796 N, the guide takes 258.199 N. The knife (R 0.1 m, ±10°)
on a 0.5 m format turns 12.758984 rad/m between cuts on average and 15.173095 at mid-span, 7.756072 on 0.8 m. The saw (1 m/s,
5 m/s²) synchronises over 0.1 m at 7.5 m/s² peak, runs 0.2 m with the material and returns at 0.8333 m/s. Each machine passes
the standard suite, its default cycle and its cam pass all verdicts, a cam pass lasts one master period (2, 0.5 and
1.5 s) and a stop ends it within the ramp; the knife's cam at a 10 rad/s limit is reported at 15.17 rad/s.

## Limits

- The arms are sized in joint space: no Cartesian paths and no inverse kinematics of the tool.
- Every joint of an arm has the same motor, drive and gearbox (only the ratio differs per joint), as the drive reads
  one `[motor]` and `[drive]` per machine; no gearbox life for arms whose joints have different ratios.
- A screw's stiffness is taken at its softest nut position for the whole stroke.
- A worm gear driven back from the load loses as any gearbox does (1 / efficiency) and never locks itself.
- No unbalance on the rotary table, no rack life formula (none published), no cross roller bearing check.
- Not built yet: mechanical (disc) cams with a follower and their contact force; a cam's master cannot yet be the
  conveyor of `[conveyor]` (the cam's master is its own, with the conveyor's speed-and-ramp model); no MC_CamIn over
  OPC UA; a changed master speed is taken at once, not ramped; the crank's and the knife's joint has no stops while
  the cam turns it on; the press force acts only on the way out (no springback) and the knife's and saw's cut force
  only forwards.
