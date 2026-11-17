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
- [Banded R-theta-Z arm (wafer-handler)](#banded-r-theta-z-arm-wafer-handler)
- [Parallel kinematics: hexapod and cable robots](#parallel-kinematics-hexapod-and-cable-robots)
- [Pneumatic cylinder beside servo axes](#pneumatic-cylinder-beside-servo-axes-machinecylinder)
- [Winder and unwinder with a dancer](#winder-and-unwinder-with-a-dancer-machine-winder)
- [Overhead crane](#overhead-crane-machine-overhead-crane)
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
| `palletiser` | four-axis palletiser, `urdf/palletiser.urdf`, a parallelogram keeping the flange level | gears of ratio 160, the flange's strain wave gear 50 | x, y, z, rz (shipped in tool space), or the four motors' joints | `strain-wave-life` per joint |
| `arm-on-track` | the six-axis arm on an 80 kg carriage on a 4 m floor track, `urdf/arm-on-track.urdf` | the track: a pinion on a rack behind a planetary gearbox of ratio 10; the arm as `urdf-arm` | the track (m) and the six joints (rad), or the track beside x, y, z, rx, ry, rz in tool space | `rack-feed-force`, `gearbox-radial-load` on the track; `guide-load`, `guide-life` of its guide blocks; `strain-wave-life` per arm joint |
| `parallel-scara` | five-bar with parallelogram forearms in the horizontal plane (its own crate, not a joint machine); a Z stroke and a tool turn at the plate | planetary gearboxes of ratio 20 on the arms; the Z stroke on a ball screw 16 x 20 behind a coupling; the turn behind a planetary gearbox of ratio 8 | x, y (m), z (m), c (rad) | `screw-speed`, `screw-dn`, `screw-buckling`, `screw-static`, `screw-life` on the Z stroke |
| `wafer-handler` | R-theta-Z arm of two equal links in the horizontal plane, its forearm and effector coupled to the shoulder by bands at 2:1 and 1:2 (its own crate, not a joint machine); a column that turns and lifts | strain wave gears of ratio 50 on the column and the upper arm; the lift on a ball screw 16 x 10 behind a coupling | r (m), theta (rad), z (m) | `wafer-slip`, `effector-direction`, `elbow-band-slack`, `wrist-band-slack`; the screw's checks on the lift |

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

With `effects.slide_presliding` (in the joint kit's `common.toml`, off as shipped; a config without the key has it off)
the guides' friction, with a screw nut's preload drag and a linear motor's attraction friction, is a presliding state
(model reference, "Slide: presliding friction") in place of the smoothed Coulomb step. Its keys in `[slide]`, in every
slide config of the kit (screw, rack, linear motor, the SCARA's quill):

| Key | Unit | What | Shipped |
| --- | --- | --- | --- |
| `friction_breakaway_n` | N | The guides' breakaway force at rest; the nut's drag and the attraction friction are added to it as to the Coulomb level; below the Coulomb level it counts as that level | 1.5 times `friction_coulomb_n`, estimate, bounds the Coulomb level to twice it |
| `friction_stribeck_m_s` | m/s | The speed over which the level falls from breakaway to Coulomb, exp(−(v / v_st)²) | 0.001, estimate, bounds 2e-4 to 5e-3 |
| `presliding_stiffness_n_m` | N/m | Dahl's σ: the friction's slope over the travel where it turns at a reversal; from one sliding level it passes zero after level · ln 2 / σ | 1e7, estimate, bounds 2e6 to 5e7 |

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
has no life check; the planetary gearbox before it has its own (`gearbox-life`). `pressure_angle_deg` (normal section, optional, 20 as DIN 867 where
absent, below 45) gives the force pushing pinion and rack apart, F tan(alpha) / cos(beta), that the pinion's radial load on the
gearbox shaft counts.

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
Newton-Euler algorithm (Luh, Walker and Paul 1980): the joint torques for given accelerations, the mass matrix, gravity
and velocity terms. The payload is a rigid body on the last link: its mass `arm.payload_kg`, its centre
`arm.payload_offset_m` from the URDF's tip frame and its inertia about the centre `arm.payload_inertia_kgm2` (xx, yy, zz,
xy, xz, yz in the tip frame), merged into the last body by the parallel axis theorem (`Inertial::merged`).

Config `[arm]`: `urdf_file` is a path, or `builtin:six-axis`, `builtin:scara`, `builtin:five-axis`,
`builtin:seven-axis`, `builtin:palletiser` or `builtin:arm-on-track` for the examples in `crates/machine-joint-kit/urdf/`. `gear_ratios` has one
entry per motor; each motor joint's limits are in `[axis.j1]`, `[axis.j2]`, ... (`max_velocity_rad_s`, or
`max_velocity_m_s` for a prismatic joint, and so on). The URDF's limits are the joints' stops; a continuous joint takes
`[joint]` `min_angle_deg` and `max_angle_deg`. A prismatic joint slides on the `[screw]`, coupled to its motor (its
ratio must be 1; the `[gearbox]`'s stiffness and damping stand for the coupling, its efficiency and inertia do not
count), with the `[slide]` friction (the SCARA's quill); where the config has a `[rack]`, it runs on a pinion on that rack
behind its own gearbox instead (ratio `gear_ratios`', any), as the rack axis does, with `rack-feed-force` and the
pinion's radial load on the gearbox shaft (`rack.<k>.radial_load`, the tooth force times sqrt(1 + (tan 20° / cos
beta)^2), against `ratings.shaft_max_radial_load_n`: `gearbox-radial-load`, as on the machine files' racks). A URDF
limit left out is 0; an empty range is refused. With a `[strain_wave]` table each revolute joint's gear gets the life
check.

Each joint has its own motor, drive and gear where the config says so: `[motor.<k>]`, `[drive.<k>]`, `[gearbox.<k>]`,
`[strain_wave.<k>]`, `[joint.<k>]`, `[ratings.<k>]` and `[parts.<k>]` over the shared tables, k = 1, 2, ... (`catalog::motor_config`,
as on the machine files). The shipped arms are sized so: the six-axis arm's wrist (joints 4 to 6) has the catalogue's
1FK2104-5AF on an S210 6SL3210-5HE10-8UF0 behind a size-20 strain wave gear, its joints 1 to 3 the 1FK2105-4AF behind
size 32; the SCARA's quill and flange the 1FK2104-5AF. `ratings.gearbox_reference_speed_rpm` is refused where a motor's
ratio differs from its own `gearbox.ratio`, which the gearbox life takes the motor's speed through.

The tool pose is the joint positions: cycles are given in joints and the sizing judges motors, gears and screws along
them. Their dwell is judged by `ratings.settling_tolerance_rad` for the joints in rad and `settling_tolerance_m` for those
in m (the SCARA's quill); it counts the gears' twist under the arm's weight, which the motors' encoders do not see (2.6
mrad at joint 2 of the six-axis example reaching out). The controller's feedforward takes the velocity terms in joint
space too: the Coriolis and centripetal torques of M(q) changing with the pose (two links of a planar arm at q2 = 90
degrees and q' = (1, 2) rad/s ask -6 and 0.75 N m that the feedforward left out before; on the shipped cycles, which
turn their joints at up to 2 rad/s, the motors' peak and RMS torque change by less than 0.3 %).

### Coupled joints: a parallelogram

URDF has no closed chains. A parallelogram that couples joints, as a palletiser's keeps its flange level, is
`arm.joint_coupling`: one row per URDF joint, the URDF's joints as sums of the motors' joints, q_urdf = C q. The machine
has one motor per column; the dynamics are the chain's taken to the motors' joints, M = C^T M_urdf C and h = C^T h_urdf
(C is constant, so it adds no term; the coupling's forces do no work). A motor joint alone in a row of factor c takes
that URDF joint's limits over c. `[]` is one motor per URDF joint.
shortcut: the parallelogram's bars are in the links they move with, not bodies of their own.

The palletiser (`palletiser`, `urdf/palletiser.urdf`, about 20 kg payload, 1.6 m reach) has five URDF joints and four
motors: the turret, the upper arm, the forearm's angle against the base (a parallelogram from the base drives it: joint 3
= motor 3 - motor 2) and the flange's turn, the wrist turning back the forearm's angle (joint 4 = - motor 3), so that the
flange stays level everywhere. Its joints 1 to 3 have the catalogue's 1FK2106-4AF on 6SL3210-5HE15-0UF0 drives behind
gears of 600 N m rated (an estimate), the flange a 1FK2104-5AF. Shipped in tool space: x, y, z and the flange's turn rz.
The app draws it as it draws the arms.

### A swinging payload

With `effects.payload_swing` (off in every shipped config) the payload hangs on a pivot at the flange, about two axes
across the line from the flange to its centre (`arm.payload_offset_m`, the pendulum's length), held towards its rest by
`arm.payload_swing_stiffness_nm_rad` and damped by `arm.payload_swing_damping_nm_s_rad` per axis: a carton in a soft
vacuum gripper or a sack in a sling. The two swing joints are joints no motor drives (`chain::Mechanism::passive_joints`):
their angles and speeds follow the joint machine's other states, their dynamics are the chain's with the swing's bodies,
the damper's power is the loss `power.payload_swing` and the angles the signals `swing.1.angle`, `swing.2.angle`. The
feedforward and the poses at rest see the payload fixed to the flange. The palletiser's carton swings at 1.61 Hz
about the axis across its 0.6 m side and 1.82 Hz about the other (sqrt((k + m g l) / (m l^2 + I)) / 2 pi with 120 N m/rad,
20 kg, 0.2 m and the carton's own inertia I = 0.75 and 0.4167 kg m^2: 159.2 / (0.8 + 0.75), 159.2 / (0.8 + 0.4167)).
shortcut: without the payload the swing's joints carry a unit inertia of their own, apart from the arm.

### An arm on a track

`arm-on-track` (`urdf/arm-on-track.urdf`, `config/arm-on-track.toml`) is the six-axis example arm on a carriage that
runs 4 m along x on two rails: a prismatic first joint, then the arm's six. The carriage (80 kg: plate, riser, guide
blocks, the track's motor, gearbox and pinion) is a body of the chain like any link, so the track is one more joint of
the Newton-Euler model and the coupling between track and arm is in the chain's equations: the arm's swing pushes on
the track (joint 2 turning at 3 rad/s² with the arm stretched across the track asks -3 · Σ m y = -43.5 N of the track,
Σ m y = 14.505 kg m of the arm and payload), and the track's acceleration reaches every joint of the arm (at 2 m/s²
joint 2 holds -2 · 14.505 = -29.0 N m). The track's motor is the catalogue's 1FK2105-4AF on an S210, behind a Siemens
NRB080 of ratio 10 (catalogue values, its backlash left out for a preloaded pinion) and the rack axis's pinion of 200 mm a
turn; 130.5 kg move on it at up to 1 m/s and 3 m/s².

`[track]` puts the carriage on four guide blocks, `block_spacing_m` (0.4 m) apart along the rails and `rail_spacing_m`
(0.5 m) across them. Their load comes from the chain's reaction at its first joint: what the base passes to the chain,
F = dP/dt + Σ m g z and M_O = dH_O/dt + Σ c × m g z (the rates of the chain's momentum P and its angular momentum H
about the base's origin by central differences), taken to the carriage's frame between the blocks. Its push along the
rails is the rack's; the rest the blocks share as rigid supports, each F_z / 4 ± M_x / (2 w) ∓ M_y / (2 l) on its rail and
F_y / 4 ± M_z / (2 l) across it, its load the sum of both magnitudes (the catalogue's equivalent load). The rack's side (`track.rack_side`, ±1 along the carriage's y) decides on which side the helical pinion's separating force adds to F_y, and its lever (`track.rack_lever_m`) turns the drive's push into a yaw moment on the blocks, M_z + side · lever · F_x; the defaults, +1 and 0, are the loads as they were counted. The signal
`axis.j1.guide_load` is the most loaded block's load times four, so that the guide verdicts, which share a load equally
among a set of four and rate the set as one (`parts.axis = "<block>x4"`), judge that block: `guide-load` against
`ratings.guide_max_load_n`, `guide-life` from `ratings.guide_dynamic_load_rating_n` and the track's speed. The moments
decide the size: on the default cycle the most loaded block carries 895 N at its peak against the 320 N a quarter of
the weight gives. Shipped: four HIWIN HGH20CA (catalogue), 3580 N against 73360 N, a life of 1.4 million hours.

Redundancy: seven joints for a tool pose of six. In tool space (`config/arm-on-track-tool.toml`) the track's position
is an axis of its own, `"j1"` ahead of the pose's in `kinematics.tool_axes`: seven axes, the track given and the arm's
six joints solved by damped least squares for the flange's pose with the track where it is. A line of a path moves the
track and the flange together, each straight; the flange held while the track moves makes the arm reach back. The
default cycle in joints picks at 0.5 m on the arm's left and places at 3.5 m on its right, the arm swinging round and
reaching out while the track travels; the tool-space cycle picks and places with straight lines of the flange, the
track carrying it 3 m while the flange moves from 0.9 m ahead and 0.6 m left of the arm to 0.7 m ahead and 0.3 m right,
150 mm up and turned by 0.5 rad.

### Tool space

With `kinematics.tool_space = true` the machine's axes are the flange's pose, and every planner, the cycle, the CLI
and the app move the tool as they move a delta's: a line of a path is a straight line of the tool. The joint machine
maps the pose to the joints through the mechanism's `inverse` and back through `forward`, the tool's speed to the
joints' through the Jacobian of `forward` (central differences). The controller's feedforward takes the speed terms:
the joints' accelerations J^-1 (a - J' q'), J' q' the second difference of `forward` along q', and the centripetal and
Coriolis torques of the Newton-Euler model. Targets are refused out of reach (`error-arm-out-of-reach`), beyond a joint's
stop (`error-joint-beyond-stop`) or where the Jacobian's condition number is above `workspace.max_condition_number`
(`error-arm-singularity`); a path through such a pose with `error-path-range`, as on the deltas. The workspace map and
`ratings.max_condition_number` take the same number. The time-optimal planner refuses a plan of more than ten million
points (`error-path-torque`): where the motors allow only a crawl it would fill the memory.

- SCARA (four joints: two arms about vertical axes, a vertical quill, a turning flange, `kinematics.tool_axes = ["x",
  "y", "z", "rz"]`): the axes are x, y, z of the flange (m) and its turn c about the vertical (rad: joints 1 + 2 + 4 plus
  the turn at zero), in closed form. The arms' lengths and directions at zero, joint 1's place, the quill's direction
  and the flange's turn come from the URDF at zero and are checked against the chain at three poses. Joint 2 by the
  cosine law, cos(beta) = (r^2 - L1^2 - L2^2) / (2 L1 L2), with `kinematics.elbow` "right" (beta > 0: joint 2
  counter-clockwise from the stretched arm, the elbow to the right of the line from the base to the tool, seen from
  above) or "left"; joint 1 = atan2(y, x) - atan2(L2 sin beta, L1 + L2 cos beta). The condition number is that of
  d(x, y)/d(q1, q2): infinite at the stretched and the folded arm (25 at 11 degrees of elbow, the example's limit).
- Any other chain of 4 to 7 joints: the axes are those of x, y, z (m) and rx, ry, rz (rad) that `kinematics.tool_axes`
  names, as many as the motors or one fewer (a redundant arm); `"j1"` ahead of them makes the first joint's own position
  an axis (a track's: see [An arm on a track](#an-arm-on-a-track)). The orientation is the rotation vector (axis times angle,
  base frame) of the turn from the flange's orientation at zero joints: a URDF whose flange points down at zero is at
  0 there, not half a turn from the base frame. With two of the three turns (the five-axis arm's rx, ry) they are where
  the tool points: the shortest turn from the flange's direction along the missing axis at zero joints to where it
  points now, whose missing part is 0, whatever the joints turn the tool about that direction; the flange's angular
  checks judge that turn alone, and a move keeps them with the missing turn at 0. A pose within 0.01 rad of half a turn from that is refused
  (`error-arm-orientation-half-turn`): there the rotation vector jumps. The joints by damped least squares on the
  Jacobian of the tool's axes (central differences; dq = J^T (J J^T + lambda^2 I)^-1 e, lambda 1 mm, the turns times
  the flange's distance from joint 2 at `kinematics.home_joints`), along the straight line from the last solution to
  the target in steps of at most 20 mm and 0.05 rad, each settled to 1 um, the last to 1e-13 m. A walk from the last
  solution that ends a step beyond `workspace.max_condition_number` may have changed the branch and is walked again
  from the home, as is one that does not get there; so the solution stays on the home's branch (shoulder, elbow,
  wrist). From the last solution an inverse along a cycle takes 7.7 us on the six-axis arm in release, from the home
  74 us. A redundant arm (seven joints, six axes) spends its spare joint on staying nearest its home posture: each
  iteration also moves within the null space of J, by (I - J^T (J J^T)^-1 J)(home - q), until that move is below 1e-11
  rad; its joint speeds and accelerations are those of the inverse itself, by central differences along the tool's speed
  and acceleration. With `kinematics.redundancy = "joint_limits"` (`"home"` as shipped) the spare joint keeps the
  joints off their stops instead: the null space moves them by (I - J^T (J J^T)^-1 J) W (m - q) towards the middles m of
  their ranges s, W = (s_min / s)², so the arm settles where sum W (q - m)² is least among the poses near it that reach
  the target (Liégeois 1977), each offset counted as a share of its joint's range. The condition number is that of the
  weighted Jacobian (of its transpose on a redundant arm).

A line of a path in x, y, z, rx, ry, rz moves the place straight and turns the orientation by slerp (`path`, the
quaternions' spherical interpolation, Shoemake 1985): about one fixed axis at a steady rate, also between rotation
vectors that are not parallel, where the straight line between them is another turn (a quarter turn about x to one about
y: 70.5 degrees about their bisector halfway, the straight line's 63.6 degrees). Lines in fewer orientation axes (the
SCARA's c, the palletiser's rz, the five-axis arm's rx, ry) run each axis straight.

`config/scara-tool.toml` and `config/arm-tool.toml` are the default configs with tool space on, a start pose and a cycle
of straight lines (the arm's settling tolerances 5 mm and 5 mrad: under the payload its flange rests 2.4 mm off the
encoders' pose and turned 3.2 mrad). `config/arm5-tool.toml` and `config/arm7-tool.toml` are the five-axis example (`urdf/five-axis.urdf`:
the six-axis arm without the flange's roll, pointing down at zero; axes x, y, z, rx, ry, where the tool points) and the
seven-axis one (`urdf/seven-axis.urdf`: a roll joint in the upper arm; axes x, y, z, rx, ry, rz), each with a cycle of
straight lines, for `--config` with the `urdf-arm` machine. Their proportions are generic, every value an estimate.

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

## Bending links and the base under the machine

With `effects.link_bending` (off as shipped) every link of an arm with a section in `[link_bending]` (a round tube per
joint: outer diameter, wall, Young's modulus) bends as a cantilever from its joint to the next in its first
Euler-Bernoulli mode, in the plane its joint turns it in: modal mass a quarter of its URDF mass plus everything beyond
it, stiffness 1.875104^4 / 4 E I / L^3, driven by what the bodies beyond need from its tip (their m (a - g), its moment
through the tip's slope) and its own weight and inertia spread along it. The tool rings with it in tool space; the
joints do not feel it (one-way). A 60 x 4 mm steel tube of 0.8 m and 4.5 kg alone rings at the Euler-Bernoulli 88.96 Hz
and sags under its weight by 49.2 um, 1.3 % above the exact q L^4 / (8 E I). `link_bending.directions` adds the bend
across that plane (`out_of_plane`) and the twist about the link's length (`torsion`, shear modulus
`link_bending.shear_modulus_pa`), a first mode each, with the bodies beyond as a tip mass and a tip inertia. With
`effects.link_bending_reaction` the bending acts back: joints, gearboxes and links are one coupled system, the motors
feel the links ring and excite them through their loops, the bending's energy and damping are in the machine's, and
`modes` gives the coupled modes with the motors held (a link on a gearbox rings as the two springs in series).

With `effects.base_compliance` (every machine and machine file, off as shipped) the machine stands on a base of up to
six spring-mass directions (`[base]`): it takes the rate of change of the momentum of everything that moves on it,
rotors included (`Machine::frame_momentum`; the slides, table and arms give theirs through `Mechanism::frame_momentum`),
and the tool moves with it. The arm on its pedestal tilts, the table's stand twists, a slide's bed sways along it. See
the [model reference](model-reference.md#compliant-base).

## Crank and cam mechanisms (`machine-crank-cam`)

The core cases of a servo sizing tool (Siemens SIZER) that a joint alone does not cover: a crank that turns a rotary motion
into a stroke, and two axes that work on a moving web or bar through an electronic cam. Each is one motor-driven joint
(`chain::JointMachine`), so the motor, gearbox, drive loops, heating and every common check are the same as above.

| Id | Mechanism | Transmission | Tool pose | Own checks |
| --- | --- | --- | --- | --- |
| `slider-crank` | servo press: crank 50 mm, rod 200 mm, ram 15 kg and die 10 kg below it, press force up to 3 kN | planetary gearbox of ratio 20 | crank angle (rad), 0 at the bottom dead centre | `rod-force`, `slider-guide-load`, `crank-bearing-load` |
| `rotary-knife` | cross cutter: knife roll of 100 mm radius, one blade, cuts a web at 1 m/s into 0.5 m formats | planetary gearbox of ratio 10 | roll angle (rad), 0 with a blade at the cut | |
| `flying-saw` | carriage 40 kg with a saw of 25 kg, material at 1 m/s cut into 1.5 m lengths in a window of 0.2 m | helical pinion on a rack behind a planetary gearbox of ratio 5 | x (m) | `rack-feed-force` |
| `disc-cam` | disc cam: base circle 60 mm, a roller follower of 2 kg (and a 1 kg tool) lifted 30 mm against its return spring, 60 turns a minute | planetary gearbox of ratio 10 | disc angle (rad) | `disc-contact-force`, `disc-pressure-angle` |
| `toggle-press` | knuckle-joint press: links of 150 mm in line between crown and ram, a crank of 50 mm pushing the knee through a rod of 250 mm, ram 20 kg and die 8 kg, stroke 57.7 mm, rated 20 kN, press force up to 12 kN | planetary gearbox of ratio 10 | crank angle (rad), 0 at the links' dead centre | `press-capacity`, `upper-link-force`, `lower-link-force`, `rod-force`, `frame-deflection` |
| `geneva-drive` | rotary indexing table of 6 stations by a Geneva wheel of 6 slots, crank 60 mm, centre distance 120 mm; table 0.4 kg m² with six parts of 0.5 kg at 200 mm; an index a second | planetary gearbox of ratio 20 | crank angle (rad), 0 halfway through an index | `indexer-contact-force`, `index-accuracy`, `index-settling` |
| `cam-indexer` | the same table stepped by a cam index drive (roller-gear cam, modified sine over 120° of the cam's turn, rollers at 80 mm) | planetary gearbox of ratio 20 | cam angle (rad), 0 halfway through an index | `indexer-contact-force`, `index-accuracy`, `index-settling` |

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

**Disc cam.** A mechanical cam: the servo turns the disc, and the disc lifts a roller follower on a straight slide,
held on it by a return spring (F0 + k s, `follower.spring_preload_n`, `spring_n_m`) and its weight. The follower's lift
s(θ) is the disc's lift law, segments of the laws of VDI 2143 over one turn (`[[disc.segment]]`, a 3-4-5 rise of 30 mm
over 120°, a dwell, the return, a dwell). The follower's axis is off the disc's centre by e (`follower.offset_m`), its
roller of radius r_r rolls on the disc of base circle r_b: the roller's centre is s0 + s above the centre,
s0 = √((r_b + r_r)² − e²), and the pressure angle, between the follower's axis and the common normal at the contact, is
tan φ = (s' − e) / (s0 + s), s' = ds/dθ (a positive offset lowers it on the rise). The contact force along the normal is
N = (m s̈ + F0 + k s + m g) / cos φ with s̈ = s''(θ) θ'² + s'(θ) θ''; the guide takes N sin φ across. The disc feels
J(θ) = J_disc + m s'², the speed term m s' s'' θ'², and the spring's and the weight's torque (F0 + k s + m g) s' (their
potential F0 s + k s² / 2 + m g s): the torque swings with the lift law, driving the disc back on the return. Signals:
the follower's lift, speed and acceleration, the contact force, the pressure angle, the guide's load and J(θ); judged
against `ratings.disc_max_contact_force_n` and `disc_max_pressure_angle_deg`. With `cam.run` the disc turns on at
`disc.turns_per_min`, its master virtual. With `effects.follower_lift_off` the follower leaves the disc where N would fall below
zero: its gap g to the cam is a passive joint (lift s + g), the contact the roller's steel line contact (Palmgren's d =
3.84e-5 Q^0.9 / l^0.8 in mm and N over its width l, `follower.roller_width_m`: 4e8 N/m at 160 N) and a damper at
`contact_damping_ratio` of critical with that stiffness, in compression only, so the force on the disc is never negative
and does not jump at the touch, and the guide's friction (`follower.guide_friction_n`) acts. At rest the roller sits
pressed in by its load (0.45 µm). Lagrange's equations: (J + m s'²) θ'' + m s' g'' + m s' s'' θ'² + (F0 + k (s + g) + m g + F_f) s' = torque
and m s' θ'' + m g'' + m s'' θ'² + F0 + k (s + g) + m g + F_f = F_c; the damper's power is booked as a loss, so the budget closes through an
impact (`power.follower_contact`). Signals `follower.gap` (negative: pressed in) and `follower.off_disc` (the contact
carries nothing). Off, the roller stays on the disc and the signal alone shows where N would
fall below zero. The middle of the disc's
range lies on the first half of the rise, where s'' > 0 and spring and weight hold the disc where it is held against
them; past the rise's middle they turn it away.

**Toggle press.** A knuckle-joint press: two links in line between the crown's pivot O and the ram, the upper one a, the
lower one b, their joint the knee; a crank of radius r pushes the knee across through a rod of length c. The crank and
not a ball screw: the crank-driven knuckle joint is the classic form and keeps the crate's turning drive and cam; the
screw-driven toggle (the knee on the nut) is not built. With the upper link at φ from straight down, the ram stands
x = a cos φ + √(b² − a² sin² φ) below O (the slider-crank's formula, the upper link its crank), x = a + b where the
links stand stretched (φ = 0, the dead centre). The joint is the crank angle q from that dead centre; the crank's
centre lies where the rod is level at it, at crank.angle_at_dead_centre_deg from the rod (90: across it, the ratio falls
linearly into the dead centre; towards 180 in line with it, a long flat dwell). The knee closes the four-bar O-knee-pin-
crank centre: φ from the two circles in closed form, φ' = D·P_q / D·K_φ and φ'' = −(E·E + D·(K_φφ φ'² − P_qq)) / D·K_φ
from the rod's constant length (D = K − P, E = K_φ φ' − P_q), so x' = x_φ φ' and x'' = x_φφ φ'² + x_φ φ''. The ratio
x' (m/rad) is zero at the dead centre: a crank torque τ holds the ram force τ / x', which grows without bound into
it. The inverse from the ram's height has two branches, before and after the dead centre
(`TogglePress::crank_angle`); the stops (`joint.max_angle_deg`) say whether the crank may go through it. The masses:
the crank and the rod's crank end at the pin, the upper link a uniform bar (m a² / 3 about O), the knee with the rod's
other end and a share of the lower link, the ram with the die and the rest:
J(q) = J_crank + m_pin r² + (m_up / 3 + m_knee) a² φ'² + m_x x'², with its speed term and the weights' torque. The
process force (`[process]`, over the ram's height above the bottom dead centre) pushes the ram up while it moves down
or stands, so that it holds through a dwell at the dead centre, and fades out over `process.smoothing_m_s` as the ram
rises. The frame stretches under it (`effects.frame_stretch`, `frame.stretch_stiffness_n_m` = k), a spring in
series: the ram stops F / k short of where the links put it, and the force is the root of F = table(d + F / k), solved
segment by segment of the table. Signals: the ram's height above the bottom dead centre (with the stretch), speed and
acceleration, the ratio x', the press force, the force capacity at the pose and its use, the frame's stretch, the
forces of the two links (two-force members, so their pins carry the same) and of the rod, and J(q). Checks:
`press-capacity`, the press force within min(`ratings.rated_press_force_n`, τ_max / |x'|) at every pose, τ_max the
drive's peak torque through the gearbox's ratio and efficiency (at most `gearbox_max_torque_nm`): the rated force
only over the last millimetres, falling with the distance from the dead centre, so that a forming force early in the
stroke fails it (and the motor's `motor-peak`); `upper-link-force` and `lower-link-force` against
`ratings.link_max_force_n`, `rod-force` against `rod_max_force_n`, `frame-deflection` against
`frame_max_deflection_m`. The shipped cycle goes from the ram's top dead centre (the crank in line with the rod, where
it holds the ram's weight without torque) into the dead centre in one move, the ratio slowing the ram over the last
millimetres, dwells there under 11.5 kN while the motor gives almost nothing, and returns; `config/toggle-press-cam.toml` does the same by a cam at 30
strokes a minute.

**Electronic cam.** `sim_components::cam` holds a cam as the slave's position over a master's, periodic, the slave rising
by its advance per master period (a crank or knife one turn, the saw 0). Segments take the motion laws of VDI 2143:
`linear` (a synchronous run), `dwell`, `3-4-5` and `4-5-6-7` (the 5th and 7th order polynomials), `cycloid` and
`harmonic`. Where a law meets a linear segment or a dwell it takes its neighbour's slope and has no acceleration there
(velocity in velocity); between two laws the slave rests (rest in rest). A cam table (`cam.table_file`, CSV of master,
slave and an optional slope) runs a cubic through its points: with the slopes given between each two points, without them
the periodic cubic spline (the slave's acceleration continuous). With `cam.run = true` the controller runs the cam as the
cycle instead of `[[cycle.segment]]`: the master comes up to speed by a sin² ramp in `cam.ramp_s`, the slave's speed is
s'(m) m' and its acceleration s''(m) m'² + s'(m) m''; a pass is one master period at speed, so `analyze` and `verdicts`
measure the steady state; a stop runs the master down over the same ramp, and a new master speed under a running cam
(another `web.speed_m_s` applied while it runs) ramps from the speed the master has to the new one over `cam.ramp_s`,
from where it stands: travel v0 t + (v1 − v0) U(t), U the ramp of unit height (`cam::master_change`). A cam that asks
more slave speed, acceleration or jerk at the master's speed than the axis limits allow is reported when the config is
applied as `limit-cam-speed`, `-acceleration`, `-jerk` (with the motor limits), and judged as the cycle's verdicts
`cam-speed`, `cam-acceleration`, `cam-jerk`, per slave axis, against the axis limits. The slider-crank runs either at constant speed (`crank.motion = "constant"`, one turn per stroke, as a
mechanical press's flywheel) or by the servo-press cam of `[[cam.segment]]` (fast approach, slow through the forming
zone, fast return) at `crank.strokes_per_min`.

The shipped configs keep `cam.run = false`: the cycle swings the crank from the top dead centre down to the bottom and
back (a servo press in pendulum mode), indexes the knife by half a turn and back, moves the saw's carriage over its
stroke, turns the disc half a turn and back, takes the toggle press's ram from its top dead centre into the bottom one and back. Beside each lies `config/<id>-cam.toml`, the same config with
`cam.run = true`, which passes every verdict (the cam's limits among them); or run the cams with `--set cam.run=true`
(app: the cam group's switch):

```text
sim-cli verdicts slider-crank --config crates/machine-crank-cam/config/slider-crank-cam.toml
sim-cli verdicts slider-crank --set cam.run=true
sim-cli analyze rotary-knife --set cam.run=true --set knife.format_length_m=0.4
sim-cli verdicts flying-saw --set cam.run=true --set material.speed_m_s=1.2
```

The torque and following error come out of the drive model as on every machine; the cams' and the mechanisms' formulas are
checked by hand in `cam.rs` and `machine-crank-cam`'s tests (below).

### Indexing table: Geneva drive and cam index drive (`geneva-drive`, `cam-indexer`)

A rotary indexing table steps on by one of its n stations per turn of its driver and stands still between (the dwell)
while the stations work. The joint is the driver's angle theta, 0 halfway through an index; the table's angle phi(theta)
follows it over |theta - 2 pi k| < h and stands outside, a station on per turn (`src/indexer.rs`).

- **Geneva wheel** (`geneva-drive`): n radial slots, the pin on a crank of radius a entering a slot radially at the centre
  distance C = a / sin(pi / n). With lambda = sin(pi / n) and x the crank's angle from the line of centres:
  tan(phi) = lambda sin x / (1 - lambda cos x), phi' = lambda (cos x - lambda) / (1 - 2 lambda cos x + lambda²),
  phi'' = -lambda (1 - lambda²) sin x / (1 - 2 lambda cos x + lambda²)². The pin is in a slot over 2h = pi - 2 pi / n of
  the turn: the table moves over (n - 2) / (2n) of it and dwells over (n + 2) / (2n) (n = 6: a third and two thirds; the
  "(n - 2) / (2n)" sometimes quoted as the dwell is the motion's share). Peak speed ratio lambda / (1 - lambda) (2.414,
  1, 0.620 for n = 4, 6, 8), peak acceleration ratio 5.41, 1.35, 0.700. The wheel starts and stops at rest but with
  phi'' = ±tan(pi / n): its acceleration jumps, the jerk is infinite, and the torque on the wheel and the pin's force on
  the slot step by J tan(pi / n) theta'² (12.5 N m and 121 N on the shipped table at 60 turns a minute). The crank does
  not feel the step (its torque is J phi' phi'' theta'², phi' = 0 there); the table does. The pin's force is the wheel's
  torque over its distance from the wheel's centre, C sqrt(1 - 2 lambda cos x + lambda²), across the radial slot. The
  locking disc holds the wheel during the dwell.
- **Cam index drive** (`cam-indexer`), the modern alternative: a roller-gear (globoid) cam moves the turret by the
  modified sine (VDI 2143: a quarter sine of acceleration over the first and last eighth, a half sine of a third the
  frequency between; peak acceleration 4 pi² / (4 + pi) = 5.528, peak speed 1.760) over `indexer.index_angle_deg` of its
  turn, and holds it on its dwell ridge over the rest. No step in acceleration at either end: its jerk is finite. The
  rollers on the turret's pitch radius take the turret's torque over that radius, along the normal with `indexer.cam_radius_m`
(R_c): tan α = r_p φ' / R_c, N = F / cos α (shipped: α = 35° at the fastest; the peak force, in the deceleration,
2.1 % more: 359.8 N against 352.5 N at 60 indexes a minute).

The wheel, the table and its parts (`mass.payload_kg` at `table.station_radius_m`) give J(theta) = J_driver + J_t phi'²
and the speed term J_t phi' phi'' theta'²; the table is horizontal, so gravity does nothing (as process force and frame
stretch, a switch these machines do not read). **Index accuracy** in the dwell is the lock's: half
`indexer.locking_play_rad` at the station radius (`index-accuracy` against `ratings.index_max_error_m`), whatever the
motor's following error, as the locking disc or the dwell ridge holds the table, not the servo. **Settling**: the table
twists on its shaft against the held wheel (`table.stiffness_nm_rad` k, `table.damping_ratio`); through the index its
twist follows x'' + 2 zeta w_n x' + w_n² x = -phi'' theta'², w_n² = k / J_s with J_s the table and its parts (the
wheel, held by the cam or the pin, is not in the mode), and the ring left at the index's end dies as exp(-zeta w_n t). The
signal `table.settling_time` is the time the table rings beyond `ratings.index_settling_tolerance_m` (at the station
radius) after an index run at the driver's present speed, solved over 2000 steps by the oscillator's exact solution for a
held force; `index-settling` judges it against `ratings.index_max_settling_s`. On the shipped table (w_n = 620 rad/s,
zeta 0.03) the Geneva's step leaves 59 µrad, 11.9 µm, settling to 10 µm in 9.1 ms at 60 indexes a minute (9.6 ms in the
run) and 60 ms on the servo cycle that runs the index at up to 10 rad/s; the cam indexer's modified sine leaves 8 µrad
(its jerk over w_n³) and is settled when the index ends. The ring is worked out beside the dynamics, not in them: the drive does not see
it. With `effects.table_twist` the twist is a passive joint x of its own: (J_d + J_t φ'²) θ'' + J_s φ' x'' + J_t φ' φ'' θ'²
= torque and J_s (φ' θ'' + x'' + φ'' θ'²) = −k x − c x', J_s the table with its parts, J_t with the wheel, c = 2 ζ √(k J_s); the
ring then acts back on the driver (signal `table.twist`, the damper's loss `power.table_twist`), and the cam's torque on
the wheel is J_w a_w less the shaft's k x + c x'. The ring as a joint is the one worked out beside: 5.930e-5 rad against
5.940e-5 after the Geneva's index at 60 a minute.

The shipped cycle turns the driver one turn (one index) from the middle of a dwell and back; `config/geneva-drive-cam.toml`
and `config/cam-indexer-cam.toml` turn it on at 60 turns a minute:

```text
sim-cli verdicts geneva-drive --config crates/machine-crank-cam/config/geneva-drive-cam.toml
sim-cli verdicts cam-indexer --set cam.run=true --set indexer.turns_per_min=90
```

## Five-bar in the horizontal plane (parallel-scara)

The parallel SCARA `parallel-scara` (its own crate, not a joint machine) is a five-bar lying flat: two arms turn about
vertical axes at x = ±R, each carries a pair of forearms (a parallelogram, so the plate keeps its orientation) to the
plate, as on Mitsubishi's RP series. Its kinematics and dynamics are the planar delta D2's, laid flat (copied into the
crate: a machine may not use another machine's crate); the plate's coordinates are x and y. Gravity acts along -z, out
of the plane, on the joints' bearings, so the arms' motors carry no weight at all: they accelerate the plate and what it
carries, and hold nothing at rest. At the plate two axes can be switched on: a Z stroke (`[z_axis]`, motor 3), a quill
on a ball screw behind a coupling, which holds the carriage and the payload against gravity (m g lead / 2 pi, 0.187 N m
for 6 kg on a 20 mm lead), and a turn of the tool about z (`[tool_turn]`, motor 4) behind its own gearbox. Each has
its own motor, gearbox, drive and ratings tables (`[motor.3]`, `[gearbox.4]`, ...). Everything at the plate (the units
with their motors, the carriage, the payload) adds to the plate's mass in the plane; the stroke and the turn are
perpendicular to the plane, so they do not couple with it otherwise. The tool pose is (x, y), (x, y, z), (x, y, c) or
(x, y, z, c), as the two switches say. The stroke and the turn are rigid joints: the screw's and the turn gearbox's
twist and backlash are not modelled. See the [model reference](model-reference.md).

## Two-part plate with a rack-and-pinion amplifier (delta-h4)

The four-arm delta `delta-h4` (its own crate, not a joint machine) is of the Quattro / H4 type (Pierrot's H4, the
Adept/Omron Quattro): four arms on the base at 90 degrees, each with a parallelogram of forearms, down to a travelling
plate in two parts. Arms 1 and 2 carry part A, arms 3 and 4 part B: two neighbouring arms, whose elbow axes are at
right angles, so that each part cannot turn at all (a parallelogram leaves its plate end one turn, about its elbow's
axis; two opposite arms' axes are parallel and would leave a part free to turn about them). The parts slide against each
other along the horizontal d = (1, -1, 0) / sqrt 2, each by the half slide h from the tool's point the other way; along
the line between them, d = (1, 1, 0) / sqrt 2, the slide would move the four forearms' ends as the plate's descent does
at the centre, and the Jacobian would be singular there. A pinion between a rack on each part rolls by h / r_p (its
centre, on a carriage the parts guide between them, stays at the tool's point), and a gear stage of ratio i_p turns the
tool: c = kappa h, kappa = i_p / r_p (30 rad/m shipped: +-180 degrees are h = +-104.7 mm).

The kinematics are the D3's per leg at the forearm's end p -+ h d (closed-form inverse; forward by Newton on the four
constraints from the D3's trilateration), the dynamics the D3's by virtual work with each body's Jacobian: the two parts,
the carriage and payload at p, and the tool's turn I_t c'^2 / 2 = I_t kappa^2 h'^2 / 2. That last term is the
machine's character: the tool's inertia counts in the slide times kappa squared (the shipped tool and payload's
0.013 kg m^2 as 11.7 kg), and the arms' gearbox twist and backlash count in the tool's turn times kappa. With
`effects.amplifier_compliance` the tool is a body of its own on the amplifier's spring (with
`effects.amplifier_backlash` a dead band in it), as the D4's tool on its shaft. See the
[config reference](config-reference.md#quattro-type-delta-h4).

## Banded R-theta-Z arm (wafer-handler)

The wafer handler `wafer-handler` (its own crate, not a joint machine) is the R-θ-Z arm of semiconductor and flat-panel
handling. A column lifts (z, motor 3, a ball screw 16 x 10 behind a coupling) and turns (θ, motor 1) an arm of two links
of the same length L = 0.32 m. Motor 2 rides on the column and turns the upper arm by φ, its angle from the radial line.
Inside the upper arm a steel band runs from a pulley fixed to the column to a pulley on the forearm of half its radius:
the forearm turns against the upper arm by −2 φ. Inside the forearm a band from a pulley fixed to the upper arm to one on
the effector of twice its radius turns the effector against the forearm by φ. So the wrist stays on the radial line at
2 L cos φ from the column's axis, and the effector keeps pointing along it: one motor moves the wafer straight in and out
(r = 2 L cos φ + e, e = 0.18 m from the wrist to the wafer centre). The tool pose is (r, θ, z).

dr/dφ = −2 L sin φ is largest with the wrist over the column (φ = 90°) and vanishes at full extension (φ = 0), the
singular pose: the condition number 1 / sin φ is refused above `workspace.max_condition_number` (5: φ above 11.5°, r up
to 0.807 m), and the upper arm's stops are at 10° and 110°. The R motor's inertia changes strongly with the reach: the
links' 0.55 kg m² at r = 0.12 m is 1.35 kg along r, their 0.19 kg m² at r = 0.75 m 2.22 kg (and without bound towards the
full extension), while the θ motor feels the whole arm, 0.15 kg m² at r = 0.12 m and 0.60 kg m² at r = 0.75 m, growing with
r². The two motors are coupled through the arm's motion: turning pulls the arm outwards (motor 2 holds it, the wafer is
accelerated towards the axis at r θ̇²), extending during a turn drags on motor 1 (the wafer's Coriolis acceleration
2 ṙ θ̇).

The wafer lies on the effector's friction pads (or vacuum pads, or an edge grip, `wafer.hold`). Its in-plane
acceleration over the cycle is the verdict `wafer-slip`, against μ g (μ (g + A p / m) on vacuum pads, the grip's
acceleration at the edge) over a safety factor: what limits a handler's cycle time. The shipped cycle between a load port and a chamber 90° apart at r = 0.55 m and 0.2 m apart in height (turn to the chamber
while rising, extend and settle, lower the wafer onto it, retract, turn back while lowering, extend under the next
wafer and settle, lift it, retract: 7.76 s) is sized for it: 0.3 g over 1.2 allows 2.45 m/s² (2.04 at the low bound of
μ), the reach's limit is 1.5 m/s² and the turn's 4 rad/s and 36 rad/s² with the wafer drawn back to r = 0.05 m, near the
column's axis (0.8 m/s² outwards and 1.8 m/s² along); the wafer reaches 1.83 m/s². The bands are springs with pretension (each span
k = 9.4e5 N/m, 150 N): their stretch lets the forearm and the effector lag behind the column and the upper arm, the
wafer lags in r and the effector turns out of its radial line (`effector.direction_error`; judged while it dwells at a
station, `effector-direction`); a span that loses all its pretension runs slack (`elbow-band-slack`, `wrist-band-slack`
judge the swing against half of it). With the gearboxes elastic (`effects.joint_elasticity`) the lift's coupling and screw
are a spring too (in series 4.56e7 N/m at the nut's far end): the column rings on it at 321 Hz with the rotor held, which
moves the default cycle by under 0.1 % (the screw's peak force 172.4 to 172.5 N). See the [model reference](model-reference.md#wafer-handler) and the
[config reference](config-reference.md#wafer-handler).

## Parallel kinematics: hexapod and cable robots

Three parallel kinematics have crates of their own, `machine-hexapod`, `machine-tripod` and `machine-cable-robot`, run
and sized like every other machine (`sim-cli analyze hexapod`, `sim-cli verdicts cable-planar`, the app's 3D view,
`search`):

| Id | Mechanism | Drives | Tool pose | Own checks |
| --- | --- | --- | --- | --- |
| `hexapod` | Stewart-Gough platform, 25 kg plus up to 30 kg, on six electric cylinders (450 mm retracted, 200 mm stroke) | per leg a ball screw 25 x 10 behind a coupling on a 1FK2105-4AF | x, y, z (m), rx, ry, rz (rad, a rotation vector) | the screw's `screw-speed`, `screw-dn`, `screw-buckling`, `screw-static`, `screw-life` per leg, under the leg's force |
| `hexapod-rotary` | Stewart platform on six cranks (6-RSS), 35 kg plus a seat of 80 kg, cranks 200 mm, rods 550 mm | per crank a planetary gearbox SP100 50:1 on a 1FT2104-5AF | x, y, z (m), rx, ry, rz (rad, a rotation vector) | `crank-dead-centre`, `rod-buckling`, `crank-ball-joint`, `platform-ball-joint` per leg, the gearbox's torque |
| `cable-planar` | a carriage of 3 kg plus up to 2 kg, a point in a vertical plane, on 4 cables from a frame 2 m x 1.6 m | per cable a winch: 25 mm drum behind a planetary gearbox 5:1 | x, z (m) | `cable-max-tension`, `cable-min-tension` per cable |
| `cable-spatial` | a platform of 4 kg plus up to 4 kg, six degrees of freedom, on 8 cables from the corners of a frame 2 x 1.6 x 1.8 m | as the planar one, 30 mm drum | x, y, z (m), rx, ry, rz (rad) | as the planar one |
| `tripod` | 3-RPS tilting table, 20 kg plus 40 kg off its centre, on three electric cylinders (450 mm retracted, 200 mm stroke), a revolute joint at the base and a spherical one at the platform | per leg a ball screw 20 x 10 (Hiwin 20-10K3-FDC) behind a coupling on a 1FK2104-4AF | z (m), rx, ry (rad, tilts about x and y) | the screw's checks per leg; `tripod-leg-force`, `tripod-leg-stroke`, `tripod-revolute-angle`, `tripod-cone-angle` per leg; `tripod-parasitic` |

Their proportions are generic, every value an `[estimate]` but the motor's and drive's (catalogue, as above). Each
default cycle passes all its verdicts; the cable robots' settling tolerances are requirements of their size (0.3 mm
planar, 1 mm and 2 mrad spatial), not the ball screw axis's 0.05 mm.

### Hexapod

Each leg is a joint of `chain::JointMachine` whose coordinate is the cylinder's extension q_i, so the screw, its nut and
bearings, the coupling, backlash, the friction of the rod's guide and the nut's preload, the stops at the stroke's ends
and the motor are those of the ball screw axis above. The mechanism gives the platform's kinematics and dynamics:

- Inverse kinematics in closed form: leg i runs from its base joint B_i to the platform joint p + R a_i, its length L_i =
  `leg.closed_length_m` + q_i. The joints sit in pairs: base pairs at 120 k ± `hexapod.base_half_angle_deg`, platform
  pairs at 120 k ± (60 − `platform_half_angle_deg`), each leg to the neighbouring pair.
- Forward kinematics by Newton's method on the platform's twist, from the centre of the tool's box: xi = J^-1 (q −
  q(x)), p += v, R = exp(w) R, until the legs miss by less than 1e-13 m; legs that no pose fits (no convergence within 40
  steps) give NaN, which every caller refuses. The dynamics, four times an integration step, starts Newton instead from
  the pose at the nearest point of a 1 mm grid of leg extensions (found from the box's centre, kept until the legs leave
  its cell) moved by J^-1 (q − q_grid): two steps instead of about five. That pose differs from the one from the box's
  centre by Newton's tolerance, and depends on q alone, not on what was called before (a run, its replay and a fit's
  perturbed runs give the same numbers); the default cycle's figures move with it as much as a change at the rounding of
  the pose moves them anyway.
- J's rows are (u_i, b_i × u_i), u_i the leg's direction and b_i = R a_i. Dynamics by the platform's twist: M_xi xi' +
  c + g = J^T f, so in leg space M_q = J^-T M_xi J^-1 and h = J^-T (c + g − M_xi J^-1 a0), a0 the legs' length
  acceleration at the twist (checked against the Lagrange equations of M_q by differences: equal to 1e-9). Each leg is
  two bodies swinging with its direction: the rod part (`leg.platform_share` of `leg.mass_kg`, centre
  `leg.platform_part_centre_m` from the platform joint) and the body part (the rest, centre `leg.base_part_centre_m`
  from the base joint), each with its inertia about its centre across the leg; their kinetic energy enters M_xi and c,
  their weight g. The payload may have an inertia of its own (`mass.payload_inertia_kgm2`, 0: a point mass).
- Refused: a leg beyond its stroke (`error-joint-beyond-stop`), a leg steeper against the base's or the platform's
  normal than `workspace.max_joint_angle_deg` (`error-hexapod-joint-angle`), a pose whose Jacobian has a condition
  number above `workspace.max_condition_number`, the turns weighed by the platform radius
  (`error-hexapod-singularity`). The workspace map shows that condition number; the signal `platform.condition` follows
  it along the cycle.
- Verdicts: each leg's force (`joint.<i>.force`) against its screw's buckling load and static rating over their safety
  factors, the screw's speed, d n and life; the motors and drives as on every machine.

Under its weight the platform held by constant leg forces is unstable (d²/dz² of m g z − Σ f_i L_i < 0), a real
property that the position loops hold; the standard suite's loss-free configs therefore run without gravity, as the
six-axis arm's do, and gravity is checked by the hand-calculated leg forces and the default config.

### Tripod

A platform on three legs of variable length: z and two tilts, as tilting tables, tool heads and motion bases use them.
The form is 3-RPS: each leg an electric cylinder as on the hexapod (motor in line, ball screw, coupling), on a trunnion
at the base (a revolute joint about the tangent of the base's circle, so the leg swings in the vertical plane through
the base's centre) and on a ball joint at the platform. The other common form, 3-PRS (vertical slides carrying struts of
fixed length, as in the Sprint Z3 head), has the same constraints and the same dependent motion; legs of variable length
are the hexapod's electric cylinders, parts the simulator has. Each leg is a joint of `chain::JointMachine` as on the
hexapod; the mechanism gives:

- The pose: z, the platform centre's height, and the tilts rx, ry, with R = Rx(rx) Ry(ry) Rz(g): the pure tilt R0 =
  Rx(rx) Ry(ry), then a turn g about the platform's normal. The three constraints (each platform joint stays in its
  leg's plane, e_i · (p + R a_i) = 0 with the joints at 0, 120 and 240 degrees) give the three dependent coordinates in
  closed form: tan g = −sin rx sin ry / (cos rx + cos ry), p_x = rp (cos ry cos g + sin rx sin ry sin g − cos rx cos g) /
  2, p_y = rp cos ry sin g; to second order p_x ≈ rp (rx² − ry²) / 4, p_y ≈ −rp rx ry / 2, g ≈ −rx ry / 2. The platform's
  normal is exactly the pure tilt's: the turn is about it alone.
- Inverse kinematics in closed form (the legs' lengths |p + R a_i − B_i|). The twist (v, w) of the platform solves G xi =
  (q', 0), G's rows (u_i, b_i × u_i) for the legs and (e_i, b_i × e_i) for the constraints; the tool's rates are S xi
  (z' = v_z and the tilts' rates from w), so the Jacobian is K = (S A)^-1, A the first three columns of G^-1.
- Forward kinematics by chord iterations x += K(home)^-1 (q − q(x)) from the centre of the tool's box; after the first,
  steps with K at that first iterate (far faster near the solution: 5 to 6 evaluations of the legs over the box) as
  long as each halves the legs' miss, else the chord again for good; until the legs miss by less than 1e-13 m, NaN
  (refused) after 100 steps. Why it converges over the working range: weighing the error as (dz, rp drx, rp dry) in the max norm, every pose of the box
  lies within r = 0.04 m of its centre, and a chord step shrinks the error by at least kappa = max ||W (1 − K0^-1 K(x))
  W^-1|| over the box grown by r (z 0.46 to 0.62 m, tilts ±0.4 rad), which holds every segment the iterations run along.
  On a grid of 17³ points there kappa is at most 0.595 and changes by less than 0.07 between neighbours, so kappa < 0.7:
  the iterations converge from the centre to every pose of the box (test
  `the_chord_iteration_contracts_over_the_grown_working_range`); the faster steps cannot stop that, as each must halve
  the miss or hand back to the chord.
- Dynamics by the twist with the constraint forces projected out: G xi' = (q'' − a0, −c0) gives xi' = A q'' + beta; the
  platform M_xi xi' + c + g = J^T f + C^T lambda, M_xi with the first moment s = R s_b of a payload off centre; A^T takes
  lambda out (C A = 0) and A^T J^T = 1, so M_q = A^T M_xi A, h = A^T (M_xi beta + c + g). The payload is a body of its own
  (`mass.payload_kg`, `payload_offset_m`, `payload_inertia_kgm2`); each leg's mass split as on the hexapod.
- Signals: per leg `leg.<i>.from_middle` (its extension from the middle of the stroke), `leg.<i>.swing` (its angle in its
  plane from the base's normal, at the revolute joint), `leg.<i>.cone` (its angle against the platform's normal, the
  spherical joint's cone); the platform's `platform.condition`, the parasitic `platform.x`, `platform.y`, `platform.turn`
  and `platform.pose_error`, the farthest any point of the joint circle lies from where the pure tilt about (0, 0, z)
  puts it, sqrt(|d|² + c² + 2 c |(R0^T d)_xy|) with d = (p_x, p_y, 0) and c = 2 rp |sin(g/2)|.
- Refused: a leg beyond its stroke (`error-joint-beyond-stop`), a swing beyond `workspace.max_revolute_angle_deg`
  (`error-tripod-revolute-angle`), a cone angle beyond `workspace.max_cone_angle_deg` (`error-tripod-cone-angle`), a
  condition number of K (tilts weighed by rp) above `workspace.max_condition_number` (`error-tripod-singularity`). Level
  the condition number is √2; over the shipped box at most 1.62.
- Verdicts: the screws' as on the hexapod; `tripod-leg-force` (`joint.<i>.force` against `leg.max_force_n`),
  `tripod-leg-stroke` (`from_middle` against half the stroke), `tripod-revolute-angle`, `tripod-cone-angle` (against the
  joints' limits), and `tripod-parasitic` (`platform.pose_error` against `ratings.parasitic_tolerance_m`, where given).

### Hexapod with rotary actuators (6-RSS)

`hexapod-rotary` (crate `machine-hexapod-rotary`) is the build of motion simulators and fast six-axis positioners: six
servo motors with planetary gearboxes on the base, each turning a crank of 200 mm, each crank's tip joined to the
platform by a rod of 550 mm with a ball joint at both ends. The default is a small motion base: a platform of 35 kg
carrying a seat with its occupant, 80 kg (its centre of mass 300 mm above the platform joints, with its own inertia), on
six 1FT2104-5AF with SP100-04-50 gearboxes on S210 converters 6SL3210-5HE10-8UF0 (catalogue). Tool pose and axes as on the hexapod; the joints are the crank
angles (rad, 0 horizontal, positive lifting the tip). Each crank is a joint of `chain::JointMachine` with a
`joint::RotaryJoint` (gearbox stiffness, backlash, efficiency, the crank bearing's friction, stops), as on the delta
pickers. See the [model reference](model-reference.md#hexapod-with-rotary-actuators-6-rss) and the
[config reference](config-reference.md#hexapod-with-rotary-actuators).

- Geometry: the crank axes' centres in pairs at 120 k ± `platform.base_half_angle_deg` on `platform.base_radius_m`,
  the platform joints in pairs at 120 k ± (60 − `platform_half_angle_deg`) on `platform_radius_m`; each crank axis
  horizontal, the base radius turned by ± `crank.axis_turn_deg` (mirrored in a pair) so that the rod leans little out
  of the crank's plane.
- Inverse kinematics per leg in closed form: with e = P_i − B_i, a cos θ + b sin θ = c, a = e · d_i, b = e_z,
  c = (|e|² + r² − L²) / 2r; θ = atan2(b, a) ∓ acos(c / √(a² + b²)), the branch fixed once by `crank.branch` ("out":
  the tip away from the pair's partner; "in" the other); no solution where |c| > √(a² + b²).
- Forward kinematics by Newton's method on the platform's twist from the centre of the tool's box, as on the hexapod;
  the dynamics too starts from the pose at the nearest point of a 1 mrad grid of crank angles, moved by K (q − q_grid).
- Jacobian: A ξ = diag(u_i · t_i) θ', A's rows (u_i, b_i × u_i), u_i the rod's direction and t_i the crank tip's
  tangent; u_i · t_i is the motor's torque arm. The condition number of the inverse Jacobian (turns weighed by the
  platform radius) is the workspace map's and the signal `kinematics.condition`; it grows without bound as one crank nears its dead centre (all six there at once
  scale its rows alike and leave it as it is: the dead centre's own margin refuses that pose).
- Refused: a rod that cannot reach (`error-target-out-of-range`), a crank nearer its dead centre (the rod in the plane
  of the crank and its axis, the torque arm zero) than `workspace.min_dead_centre_margin_deg` (`error-rss-dead-centre`),
  a ball joint beyond `workspace.crank_joint_max_angle_deg` (the rod out of the crank's plane) or
  `platform_joint_max_angle_deg` (against the platform's normal; `error-rss-ball-joint`), two rods nearer than
  `workspace.min_rod_clearance_m` axis to axis (`error-rss-rods-cross`), a rod nearer than `workspace.min_body_clearance_m` to an edge of the platform's or the base's hexagon (`error-rss-rods-cross-body`), a condition number above
  `workspace.max_condition_number` (`error-rss-singularity`), a crank beyond its stops.
- Dynamics by virtual work in the crank angles: the platform and payload as one rigid body with its centre of mass off
  the platform centre, each rod as a slender rod (a sixth of its mass at each end and two thirds at its middle: exact
  for a uniform rod's mass, centre and inertia across, which the lumped split of the delta crates is not), each crank
  with its inertia and weight, the rotors through the ratio; gravity; energy and frame momentum for the standard
  suite and the compliant base.
- Verdicts beyond the motors': the gearboxes' torques (`gearbox-peak`, `gearbox-continuous`, `gearbox-input-speed`
  against the SP100's ratings), per leg `crank-dead-centre` (the rod's angle from the crank's tangent against 90° less
  `ratings.crank_dead_centre_margin_deg`, 20°), `rod-buckling` (the rod's compression against its Euler load π² E I /
  L² of the tube in `[rod]`, over `ratings.rod_buckling_safety_factor`), `crank-ball-joint` and
  `platform-ball-joint` (the joints' angles against their limits), and `singularity`.
- The default cycle: heave up 50 mm, then surge and sway 60 mm, roll and pitch
  0.12 rad, yaw 0.15 rad, each from the pose before (one turns back while the next sets in), and back to the level pose,
  where the seat is taken off and put on again (the payload's switch).

As on the hexapod, the platform held by constant crank torques runs away from its pose under its weight, so the standard
suite's loss-free configs run without gravity (and without the gearboxes' backlash, which no load then holds against a
flank); gravity is checked by the hand-calculated holding torques and the default config.

### Cable robots

A cable leaves its winch along the swivel axis of a pulley of radius r (`frame.exits_m`, `frame.winch_directions`,
`pulley.radius_m`), wraps it and runs straight to its anchor on the platform (`platform.anchors_m`; the planar one's
cables meet at the carriage's centre). The pulley swivels into the plane of the anchor; in that plane the length is the
arc r beta plus the tangent l_f = sqrt(m² − r²), and dL/dP = −u, u the unit vector from the anchor to the exit point.

- Statics: the structure matrix A (n x m) has the columns (u_k, b_k × u_k) in the platform's degrees of freedom; the
  cables hold a wrench w when A t = w with every tension in the band from `cable.min_tension_n` to the rating
  `cable.breaking_load_n` / `cable.safety_factor`. Cables only pull, so the tensions are distributed by a small QP: those
  nearest `cable.reference_tension_n` that hold the wrench within the band. Where no cable sits at a bound that is the
  closed form t = t_ref + A^+ (w − A t_ref); where one does, the QP holds it there and shares the rest out, and unlike the
  improved closed-form method (which fixes one cable at a time and never frees it again) its tensions change
  continuously with the pose. It is solved by an active-set method on the penalised problem (which also says which
  cables sit at a bound) and then exactly on the free cables.
- Wrench-feasible workspace: a pose whose platform weight (with the payload) no tensions in the band hold is refused
  (`error-cable-wrench-infeasible`), also as a target, a path or a cycle pose. At the planar robot's centre the four
  cables carry 139.81 N (top) and 100.19 N (bottom); 5 cm below the top pulleys the top cables would need 1284 N each.
- Elastic (`effects.cable_stretch`, on): a cable of free length l is a spring of EA / l while taut (`cable.ea_n`), slack
  below; its stiffness changes over the workspace. The platform's stiffness on its cables, elastic and geometric part
  with the drums held, gives its natural frequencies (`sim-cli modes`, the input shaper's resonance): 24.6 Hz at the
  planar robot's centre. `effects.cable_damping` adds `cable.specific_damping_n_s` / l. Rigid (off), the cables keep
  their length.
- The cables' own mass (`cable.mass_per_m_kg`, 3 g/m and 10 g/m shipped, [estimate]; 0 leaves it out): half of each
  span's weight hangs on the platform, and a cable sags (shallow parabola): at the chord tension T it is longer than its
  chord by (w l_h)² l_f / (24 T²), so with t0 = EA (L − l) / l the straight cable's tension, t0 = T − c / (2 T²),
  c = (w l_h)² EA l_f / (12 L); its stiffness dT/dt0 is Ernst's equivalent modulus E / (1 + (w l_h)² EA / (12 T³)), and
  a slack cable still hangs with a small tension. Against the exact elastic catenary the pull on the anchor agrees to
  9e-5 N at 20 N and 2e-4 N at 5 N (the straight cable misses it by 0.17 N and 1.6 N there).
- The winches' drums and rotors spin about `frame.winch_spin_axes` (default horizontal and square to the cable's
  direction at the winch; with a vertical one the pulley's axle at rest), and their spin is in the frame's load.
- The time-optimal plan keeps every cable's tension of the rigid model within the band (`cable.min_tension_n` to the
  rating, `profile.tension_reserve` of the room): the planar default cycle takes 1.684 s instead of 1.653 s without the
  band, where a cable went down to 0.63 N; cable-spatial 1.207 s instead of 1.143 s.
- Each winch is a `joint::RotaryJoint` in metres of cable: motor, gearbox with the drum's radius as its lever, the drum's
  inertia, friction of bearings and pulley (`effects.winch_friction`), stops at an empty and a full drum. Elastic, drum
  and rotor are bodies of their own behind the gearbox's spring; rigid, the cables keep their length and the winches'
  inertia joins the platform's.
- The gears' mesh friction (`effects.gearbox_mesh_friction`) holds (1 - eta) of the torque each gear passes, and a
  winch's gear always passes the cable's pretension: at cable-spatial's 250 N, 30 mm drum, ratio 5 and eta 0.95 a band
  of +-12.5 N at the cable (+-0.075 N m at the motor) inside which the drum stands still. A turn of the platform moves
  each cable by little (at the home pose 35, 28 and 14 mm per rad for rx, ry, rz), so the sum of the eight bands along
  a turn, 3.5, 2.8 and 1.4 N m, against the position loop's proportional stiffness through the cables (830, 530 and
  130 N m/rad) is a dead band of 4, 5 and 11 mrad without the integrator: by hand, a 0.5 mrad sine of rx at 1 Hz
  asks the loop for 1.0 to 1.1 times the band's torque, rz for 0.4, so with the speed loop's Tn at 30 ms rx and ry
  followed to 1.41 and 1.25 Hz and rz had no band (simulated alike; 4.44, 4.37 and 4.06 Hz with the friction off). The
  friction is physical, so the tuning changed: Tn 5 ms, by the symmetric optimum from the speed loop's 80 Hz crossover
  (phase margin 60 degrees), lets the integrator break the meshes away within the turn: 5.6, 5.2 and 4.4 Hz. A lower
  pretension would narrow the band too, but below 200 N the cycle takes cable 7 under its 20 N. The machine friction
  feedforward (`drive.friction_feedforward_model = "machine"`) now has the meshes' friction on the tension at rest,
  (1 - eta) of each gear's held torque along the planned motion, so that the integrator no longer carries it through a
  move and takes it past the target when the move stops; it acts on the planned speed and does not reach a step or a
  sine on the setpoint. In the PLCopen test's 5 mrad move of rz the platform then stands 0.31 mrad past the target
  0.3 s after Done and 0.17 mrad 1 s later (0.55 and 0.20 before), inside the 2 mrad turn tolerance the spatial config
  now states (`ratings.settling_tolerance_rad`). The planar robot moves its cables by its translations alone and keeps
  30 ms (9.3 and 9.8 Hz).
- The motors' angles of a pose are those of the free lengths that hold the platform there at rest, l = L / (1 + t /
  EA), t the tension distribution's wherever one exists (the closed form's only beyond the wrench-feasible workspace),
  so position setpoints and the
  torque feedforward (the distribution's tensions for the platform's wrench, plus the winches' inertia) ask the same
  pretension of the cables. The forward kinematics is Gauss-Newton on these lengths; the controller uses it twice a
  position cycle (the following error in tool space), so it takes cheap steps first and the numerical Jacobian only near
  the workspace's edge. With stretching cables it then finds the platform's rest on them by Newton, from the pose at
  the nearest point of a 1 mm grid of free lengths (found as before, kept until the lengths leave its cell) with its
  Jacobian (a chord method, the Jacobian recomputed where a step fails), which saves
  the Gauss-Newton and most residuals. The pose differs by Newton's tolerance (1e-12 m) and depends on the motor angles
  alone; the default cycles' figures move by up to 1.5e-5 with it.
- Signals per cable: `cable.<k>.tension`, `cable.<k>.shortfall` (how far it is below the least tension), `cable.<k>.length`;
  `platform.condition`, the structure matrix's condition number. Verdicts: `cable-max-tension` (highest tension against
  the rating) and `cable-min-tension` (any shortfall fails), the motors and drives as on every machine.

The press's cam changes speed by 4-5-6-7 laws: a 3-4-5 into the slow zone asks 6 Δv / Δm² + 60 c / Δm³ = 67.4 rad/rad³
of jerk at its ends (Δv 1.468, Δm 0.5 rad, c −0.067 rad), at π rad/s of master 2090 rad/s³, more than the crank's
2000; the 4-5-6-7 has no jerk of its own there. The four machines' speed loops have Tn 0.01 s (the knife's Kp 0.25):
the integrator takes up the friction at the end of a move three times as fast as with 0.03 s, and the dwell errors fall
to 0.13 mrad (crank), 0.16 mrad (knife), 0.14 mm (saw) and 0.23 mrad (disc), with at least 35.8° and 8 dB of margin in
every run across the bounds (`sim-cli margins <id> --bands`); the settling tolerances are 0.3 mrad, 0.3 mrad, 0.25 mm and
0.5 mrad. Retuned on 15 November so that the margins hold with friction off too (the sweep `across_the_bounds`): the
knife's Kp 0.18 (at the softest gearbox damping of its bounds 0.25 left 29.7°; now at least 37.8° and 9.9 dB; its
cycle's following error 8.86 → 12.1 mrad, the cam's 4.72 → 5.70 mrad, settling 0.042 → 0.072 mrad), the saw's Tn
0.04 s, position loop kv 57 1/s, Kp 0.25 with a notch at its gearbox and rack's resonance (360 Hz, 250 Hz wide;
the friction-free 45 kg carriage left 33.9° with Tn 0.03 s and no notch; now 40.2° and 11.51 dB; it comes to rest
moving 12.3 µm in 0.2 s after a long move, 13.5 before, under the test's 20; at 400 Hz the notch's phase lag
below its centre drove the undamped gearbox mode at 367 Hz unstable in the loss-free configs). The notch's switch is each crank and cam
machine's own (`effects.notch_filter`, the first key of its file, which follows the `[effects]` table that ends
`config/common.toml`); the other four keep it off.

## Pneumatic cylinder beside servo axes (`[[machine.cylinder]]`)

Most pick-and-place machines carry a pneumatic actuator beside their servo axes: a Z stroke, a gripper slide, a
stopper. A machine file adds one with a `[[machine.cylinder]]` node whose `values` table holds the cylinder, its valve,
tube, throttles and cushions (`sim_components::pneumatic`, keys in [config-reference.md](config-reference.md), model
in [model-reference.md](model-reference.md)). It is no tool axis: no drive moves it and the controller plans nothing
for it. The cycle commands its valve once a segment has arrived (`valves = { z = "extend" }`) and waits for its
end-position sensors (`wait = ["z.extended"]`) before the dwell counts and the next segment starts.

`machines/pneumatic-axis.toml`: the belt-driven servo X of `single-axis.toml` carrying a Festo DSBC-32-100-PPVA-N3
(32 mm bore, 12 mm rod, 100 mm stroke, PPV cushions 17 mm, 0.4 J) hanging downwards on a 5/2 valve with meter-out
throttles; 0.5 kg moved, 0.5 kg picked. The cycle: X to the pick, Z down, wait, grip, Z up, wait, X to the place, Z
down, release, Z up. With the shipped settings Z extends in 0.55 to 0.56 s and retracts in 0.65 s (valve 7 ms, the rod side
exhausting from supply, 0.216 m/s throttled, the cushion's bleed at the end), enters the cushion at 0.22 to 0.24 m/s
with a kinetic energy of 0.012 to 0.029 J against the 0.4 J rated (`cushion-energy`). The check is the vendor's rule for
the cylinder's own cushioning, the kinetic energy at the cushion's entry against the maximum impact energy of the
table (Festo DSBC data sheet, "Forces and impact energy", v = sqrt(2 E / (m1 + m2)), p. 13); the weight's work over
the cushion length, m g L, belongs to Festo's rule for external shock absorbers and is not added (hanging 1 kg over
the 17 mm: 0.167 J more, which this check does not count). It stops at 0.06 to 0.09 m/s on the bumper without
bouncing out of the sensor band, and has a load factor of 0.024 (`load-factor`, the 1 kg weight against 415 N
retracting). A double stroke draws 1.10 normal litres, 474 J at the compressor (0.12 kWh/m³); by hand at the supply
pressure and the wall's temperature 1.15 l, the 4 % less because the dwells are too short for the hot air to cool.

What the cylinder gives for sizing: stroke times out and back, the speed profile, the speed and kinetic energy
entering the cushion against its rating, the speed at the end stop, bounce and settling, the load factor, the air per
double stroke and its energy at the compressor, the pressure and temperature traces of both chambers.

## Winder and unwinder with a dancer (`machine-winder`)

The classic servo sizing problem of converting machines, and the one machine whose motors a material couples and whose
load changes over the run: `winder` runs a web (film, paper, foil) from an unwind roll over a dancer roller to a rewind
roll, at a line speed, under a tension. The shipped config: a PET film of 50 µm, 400 mm wide, at up to 2 m/s (up to speed
in half a second, 4 m/s²) and 100 N, on 3-inch cores (r_c 45 mm) growing to 300 mm (r_f 150 mm; 1286 m of film, 36 kg),
each roll on a 1FK2206-2AF with a holding brake behind a planetary gearbox of ratio 7, on an S120 Motor Module of 5 A
with a Braking Module for both; the dancer a roller of
80 mm on a vertical slide, 4 kg plus a 2 kg weight, pushed down by a pneumatic cylinder, ±50 mm of travel. The line
position runs over the roll's first 6 m (`roll.run_from_m`, `run_m`), where the unwind is full and the rewind on its
core; the cycle runs 3 m forward and back. Model in the [model reference](model-reference.md#winder-with-dancer).

| Id | Mechanism | Transmission | Tool pose | Own checks |
| --- | --- | --- | --- | --- |
| `winder` | unwind roll, two web spans of 1 m over a dancer roller, rewind roll | planetary gearbox of ratio 7 per roll | line position (m), the web wound up | `web-tension`, `web-slack`, `dancer-travel`; per motor `inertia-ratio`, `torque-at-full-roll`, `torque-at-core`, `speed-at-core`, `tension-torque-at-full-roll` |

**What changes over a run.** A roll's radius grows by the web's thickness per turn, r² = r_c² + h s / π; the motor's speed
for a line speed v is i v / r, highest at the core (311 rad/s at 2 m/s), the torque for a tension T r / i, highest at the
full roll (2.14 N m at 100 N), and the inertia it sees grows with r⁴: from 0.23 to 10.4 times the rotor's and gearbox's
over the roll. A sizing has to hold both ends; a cycle runs metres of a roll of 1286 m, so the own verdicts carry the
cycle's line speed, acceleration and tension to the full roll and to the core (model reference, "Own verdicts"). With the
shipped motor the inertia ratio (10.4 of 15 allowed) decides the motor, not its torque: a 1FK2205-2AF (3.15 kg cm²) has
the torque but 22 times its inertia at the full roll.

**The web** stretches in two spans of EA / L (80 kN/m each), carries its strain from span to span, and goes slack below
zero tension. The dancer on them rings at 26.1 Hz, its roller at 56.9 Hz. The rolls follow the line position by
electronic gearing at the radius of their wound length, with the tension's torque and the roll's inertia fed forward;
the dancer's position loop trims the unwind's speed: a roll wound at 100 N run at 80 N would drift the dancer by 0.25 mm/s
at 2 m/s (the draw), which the loop takes out. The tension setpoint tapers over the rewind's radius, 100 N at the core to
79 N at the full roll, through the cylinder's force.

**Stops.** A category 2 stop (`sim-cli stop winder --category 2`, the line braked along its path at the axis's 4 m/s²)
keeps the web between 30 and 250 N at every radius. A category 1 stop brakes each motor on its own OFF3 ramp: in
the middle of the roll, where the radii are equal, the web keeps its tension; near either end the two surfaces slow in
the ratio of their radii (1.35 against 0.40 m/s² on the full roll and the core), the dancer runs into its stop after
0.46 s and the web is stretched beyond 1200 N. Category 0 lets the holding brakes take the rolls within some 50 ms and the
web goes slack. On a winder only a stop along the line keeps the web; after a category 1 stop the holding brakes keep
the tension the dancer holds.

**Speed loops.** The rewind on its core has a tenth of the full unwind's inertia at the motor, so each has its own gain
(1.2 and 3.0 N m s/rad, Tn 30 ms): `sim-cli margins winder` gives 73.6° and 20.5 dB on the unwind, 58.8° and 9.56 dB on
the rewind (52.0° and 9.19 dB without friction), at least 54.9° and 8.91 dB across the bounds (`--bands`).

```text
sim-cli verdicts winder
sim-cli run winder --cycle --time 10 --stop-at 7.5 --stop-category 2 --out winder.csv
sim-cli stop winder --category 2
sim-cli verdicts winder --set web.tension_n=150 --set axis.web.max_velocity_m_s=3.0
```

## Movers on a closed track (`machine-mover-track`)

A linear transport system as Beckhoff XTS or B&R ACOPOStrak: motor segments of linear-motor coils in a closed loop, and
on them independent movers, each a mass on rollers carrying a magnet plate, each driven by the coils under it with a
position loop of its own. The shipped config: six movers of 651 g (AT9014) with 600 g each, on a loop of two 1 m
straights of four 250 mm modules (AT2000) and two 180° curves 267 mm across, a 100 mm clothoid at each end of an arc of
130.3 mm (the split is an estimate; Beckhoff publishes the width and that it is a clothoid); 3.019 m round. A mover gives
100 N peak and 30 N continuous, 4 m/s, 10 g unloaded (the 5-pin magnet plate on 48 V; with the payload the plate moves
1.25 kg at 79.9 m/s², the axes are limited to 30 m/s²); force constant, coil resistance, the
attraction (300 N) and the pole pitch are estimates.

| Id | Mechanism | Transmission | Tool pose | Own checks |
| --- | --- | --- | --- | --- |
| `mover-track` | 1 to 7 movers on a closed loop of straights and curves (6 shipped) | linear motor per mover: the coils under it | each mover's distance along the loop (m), unwrapped | per mover `mover-guide-load`, `curve-holding`, `station-settling` (index cycle); `mover-gap`; per segment `segment-current`, `segment-temperature` |

**Two cycles, both electronic cams.** A closed track only goes forward, so the cycle is a cam (`[cam]`): `track.cycle =
"index"` moves every mover one station pitch (the loop over the movers, 0.503 m) by a 4-5-6-7 law in 0.4 s and
stands it at its station for the rest of the period, 1.5 times a second (2.75 m/s, 23.6 m/s² peak); `flying.toml` runs
`"flying"`: each mover in turn rides with a conveyor at 1.0 m/s over 0.3 m of the first straight, synchronous with it,
then back round the loop and into the process again (6 products of 0.3 m per loop: the movers close up to a product
pitch over the process and run back at up to 2.78 m/s). `default.toml` ships `cam.run = false`, as the crank and cam
machines do: there the movers jog 0.25 to 0.3 m forward to take a product and back with it through a point they pass
without stopping, to hand it over at the start, by the `[[cycle.segment]]`, the cycle the generic tests and tools run;
`--set cam.run=true` runs the index cycle.

**Curves.** A mover's rollers span its wheelbase w, so the curvature its body follows is the curvature's mean over it:
the lateral acceleration v² k rises over the wheelbase where a constant-radius curve begins and over the clothoid where
there is one; its rate, the lateral jerk 2 v a k + v³ dk/ds, is what the clothoid takes down (at 2 m/s 1228 m/s³ into a
circle, 614 m/s³ into the clothoid). The movers run outside the loop with the plate facing the coils, so the centrifugal
force m v² k pulls a mover off the coils against the attraction: the rollers carry sqrt((F_a − m v² k)² + (m g)²)
(`mover-guide-load`), and the share m v² k / F_a (`curve-holding`) must stay below 1, where the mover lifts off.
`mass.centre_offset_m` = e puts the movers' mass centre outside the centre line (the drawn mover's is 35 mm out; 0
ships): in a curve it runs on R + e at v (1 + e k), its lateral acceleration v² k (1 + e k) (38.9 m/s² mid-arc at 2 m/s
with e = 35 mm, 30.7 at e = 0) and its kinetic energy ½ m v² (1 + e k)², with the tangential force of that mass changing
over the clothoid in the dynamics (the energy is kept). The offset moves the mover body and payload, not the plate.

**Coils and segments.** A mover's current is its force over the force constant; it flows in the coils under its magnet
plate, shared among the segments by the plate's length on each. A segment's current is the sum of its movers' shares
(`segment-current`, against the module's 48 A peak), its copper loss R I² of each; it warms through one thermal
resistance and time constant (`segment.<j>.temperature_rise`), and `segment-temperature` judges the rise its loss settles
at averaged over a pass: movers stopping on one segment heat it. The gap between neighbours along the centre line, less
a body's length, against the 2 mm least distance (`mover-gap`); a standing mover's error at its station against the
±10 µm repeatability (`station-settling`).

Model and limits in the crate's documentation (`crates/machine-mover-track/src/lib.rs`).

```text
sim-cli verdicts mover-track --set cam.run=true
sim-cli verdicts mover-track --config crates/machine-mover-track/flying.toml
sim-cli verdicts mover-track --set cam.run=true --set track.index_rate_hz=2.5 --set track.clothoid_m=0.0
```

## Overhead crane (`machine-overhead-crane`)

`overhead-crane` is a single-girder overhead crane of 250 kg safe working load: a bridge travelling 10 m on its runway
(x), a trolley travelling 5 m along the girder (y), and a hoist on the trolley whose drum winds a single fall of 6 mm
steel wire rope down to the hook. The tool is the hook as the drives place it: the bridge's x, the trolley's y and the load's height z over the floor
(m); the load's sway is not in it (a crane is positioned by its bridge and trolley) but has signals and a verdict of its own. Each drive
is a servo motor behind a planetary gearbox (Siemens 1FK2104-6AF on the bridge and the trolley, 1FK2105-6AF on the
hoist, S210 drives), its wheel or drum the lever: 30:1 on 80 mm wheels, 25:1 on 50 mm wheels, 40:1 on an 80 mm drum.

- **The rope.** It leaves the trolley's sheave at T = (x, y, H), H = 5 m, and runs straight to the load's point P (hook
  and load one point mass). Its length L = |P − T| over its free length l (paid out by the drum) is a stretch e = L − l;
  while taut it pulls with t = EA e / l + c / l (L' − l') (`rope.ea_n`, `rope.specific_damping_n_s` with
  `effects.rope_damping`), its potential EA e² / (2 l), and it cannot push: below zero it is slack. It pulls the load
  with −t u (u the unit vector from the sheave to the load), the trolley with t u (x on the bridge's drive, y on the
  trolley's, z into the rails) and the drum, through the potential's slope over l, with t (1 + e / (2 l)): the load's
  sway acts back on both travel drives, and its swing stretches the rope on the hoist.
- **The load** is a spherical pendulum on that rope, P'' = −t u / m − g e_z. Its small swing has the period
  2π √(L / g): 2.84 s on a 2 m rope, 4.01 s on 4 m (`sim-cli modes overhead-crane` reports the frequency over z).
  With the trolley free to roll it is a two-mass pendulum, ω² = g / L (1 + m / M), M the trolley with its rotor at the
  rail. Rigid (`effects.rope_stretch` off) the rope keeps its free length: its tension is the constraint force that
  keeps L'' = l'', solved in closed form each step; that model's rope also pushes (it is the reference for a taut rope).
- **The drives** are rigid (`joint::RotaryJoint` with a `transmission::Converter`; the gearboxes' backlash and twist are
  read and not used, the crane's give is its rope's), the gearbox's efficiency on the power it passes, the motors' own
  friction, ripple and holding brake as on every machine. The wheels roll against `crane.rolling_coefficient` (0.006)
  times their load, the bridge's and trolley's weight and the rope's pull down (`effects.guide_friction`). Buffers
  at the rails' ends and stops at an empty and a full drum (`effects.hard_stops`).
- **The motors' angles of a pose** are the empty hook's: l = (H − z) / (1 + m_hook g / EA). The payload hangs lower than
  the pose by its own stretch (6.8 mm on 4 m), and the drum stands still when the hook takes the load or lets it go, as
  a hoist's does (`setpoint_slack` 7.8 mm). Pick and set-down are quasi-static: the payload goes on (off) with the rope
  at once at the stretch of the new weight, as a load standing on the floor that the hoist takes up slowly; without that
  the rope would snatch the load from rest at twice its weight and fling the empty hook up off a slack rope.
- **Input shaping against the sway.** The machine reports the sway's frequency √(g / L) / 2π at a pose
  (`Machine::resonance_hz`), and with `effects.shaper_follows_pose` (on in the shipped config) the input shaper's
  period follows the rope's length where the plan stands, so a move that hoists while it travels is shaped for the
  length as it changes; `profile.shaper_axes = ["x", "y"]` leaves the hoist unshaped. `effects.input_shaping` itself is
  off in the shipped config (as on every machine, so that a setpoint from outside arrives as sent); switch it on with
  `--set effects.input_shaping=true` (`machine_overhead_crane::shaped_config()`). Measured on the empty hook's way back,
  which hoists from 4 m to 2 m of rope while it travels 7 m: unshaped 0.43 m of sway left, the following ZV (the shipped
  `profile.shaper_type`, half a sway period behind the plan) 0.017 m, a ZV set for the length at the move's end 0.124 m,
  one set for its start 0.105 m (ZVD, a whole period behind: 0.012, 0.047 and 0.041 m). A move without a dwell starts
  when the plan arrives, the shaper's tail still under way, so a segment that lowers right after a travel has the sway's
  period change under that tail: the default cycle travels and lowers in one move instead.
- **The time-optimal plan** (`profile.cycle_timing = "time_optimal"`) knows the load as carried along with the trolley,
  not swinging: the config keeps 30 % of every limit for the swing (`profile.time_optimal_margin` 0.7; planned at the
  motors' full speed, the swing ran the bridge's motor past its no-load speed). It is no sway-aware plan.
- **The default cycle** (travel and lower, back, pick, hoist): a short positioning cycle, 7.0 s a pass (12.8 s shaped),
  so that every check of every machine's cycle sees it whole. From the travel height (z = 3 m, a 2 m rope) over the
  pick place the load travels 0.2 m and 0.1 m to the place while it is lowered by 0.15 m, and is let go; the empty hook
  goes up and back in one move, down to the next load, 0.3 s to sling it, and hoists it. Every move but the pick's
  lowering and hoisting travels and changes the rope's length at once, so every drive moves from the start. Shaped,
  every verdict passes (residual sway 4.3 mm, the rope 2727 N at most and never below its least tension, the hoist brake
  loaded to 5.5 of 7.5 N m); unshaped, the load is left swinging by 0.32 m and `residual-sway` fails, and so does
  `thermal-warm-start` (the sway left from one pass changes the next, so the passes never repeat: a winding's rise
  1.1 K apart). A workshop crane's real cycle over the runway takes a minute: a segment's target of [8.0, 4.0, 1.0].
- **The drives' velocity loops** are tuned as stiff as the other machines' (bridge and trolley some 60 Hz, 2.0 and 0.24
  N m s/rad). The hoist's is measured at rest under the 265 kg (`sim-cli margins`) with the gearbox's efficiency effect off: with it on, the 92 % gearbox's efficiency, faded in below 0.5 W, is a damper of 2601² ln(1 / 0.92) / 0.5 W = 1.1e6 N s/m at the rope, 4.5 N m s/rad at the motor, which the test torque does not break away from, and the loop shows no gain crossover. Without the gear's losses, with a tenth of the hoist's own rated torque (0.66 N m) as the test torque, it crosses over at 50.4 Hz with 73.7° and 21.0 dB (at 0.32 N m, a tenth of the shared drive's rating: 48.5 Hz, 83.1°, 21.3 dB, where the motor's friction damps the smaller signal more).
- **Verdicts of its own:** `rope-max-tension` (the highest tension against `rope.breaking_load_n` / `safety_factor`,
  4444 N), `rope-min-tension` (any fall below `rope.min_tension_n`, 50 N, fails: no slack rope), `residual-sway` (the
  load's sway amplitude √(d² + (d' / ω)²) at every dwell, d its horizontal offset from under the sheave, against
  `crane.residual_sway_max_m`, 50 mm), `hoist-brake` (the rope's torque at the hoist's motor, t (1 + e / (2 l)) r / i,
  against `motor.hoist.brake_torque_nm` / `hoist.brake_safety_factor`, 12 / 1.6 = 7.5 N m). Signals `rope.tension`,
  `rope.shortfall`, `rope.length`, `load.sway_angle`, `load.sway`, `load.residual_sway`, `hoist.brake_load`.
- **3D view:** the runway's rails on their columns, the girder, the trolley with its drum, each drive's motor and
  gearbox, the rope coloured by its tension against the load's weight, the hook and the load; labels with the motors'
  speeds, the rope's tension and the sway in mm.

```text
sim-cli verdicts overhead-crane --set effects.input_shaping=true
sim-cli verdicts overhead-crane
sim-cli modes overhead-crane --axis-index 2
```

Left out (shortcuts in the code): the rope's own mass (0.14 kg/m against 265 kg) and its bending at the sheave, a hook
block and a load that swing on their own (a double pendulum) or twist, a bridge on two drives that can skew, and the
load standing on the floor during pick and set-down.

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
stretched SCARA arm is refused; the time-optimal plan keeps the motors within their maximum speed and torque-speed curve along a line; both tool
cycles pass their verdicts. The arms of 14 November: the feedforward in joint space has a two-link arm's Coriolis and
centripetal torques by hand (-6 and 0.75 N m at q2 = 90 degrees, q' = (1, 2) rad/s); a payload's own inertia counts
about the last joint (1.2 N m against the point mass's 1.15 at 1 rad/s^2); the slerp midpoint of two quarter turns is
70.53 degrees about their bisector, and a planned line of the arm's tool passes it; the warm-started inverse gives the
joints a walk from the home gives (1e-9 rad); the seven-joint arm reaches a pose to 1e-9 with its joints' offset from
the home free of the Jacobian's null space; a flange pointing down at zero is at no turn; the palletiser's flange stays
level over its joints' ranges and its swinging carton has m g l (1 - cos 0.1) + k 0.1^2 / 2 = 0.79597 J turned by 0.1
rad and (m l^2 + I) / 2 = 0.775 J swinging at 1 rad/s, and keeps the energy loss-free; each arm joint has its own
catalogue motor; a time-optimal plan of a crawl is refused beyond ten million points. The arm on its track: the track
pushes -3 · 14.505 = -43.515 N while joint 2 swings the stretched arm at 3 rad/s², joint 2 holds -2 · 14.505 =
-29.01 N m while the track accelerates at 2 m/s² and the track pushes 130.5 · 2 = 261 N; the guide blocks carry
4 (1279.768 / 4 + 142.2455 / 1) = 1848.75 N at rest with the arm stretched across and 2656.85 N accelerating, the
pinion's shaft 261 · 1.071978 N; the guide verdicts flip at their limits; in tool space the track is where the pose
says to 1e-12 m and forward of inverse returns the pose to 1e-9 along the whole track. A screw 1.6 m long fails `screw-speed`, and a life asked for beyond the
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
1.5 s) and a stop ends it within the ramp; the knife's cam at a 10 rad/s limit is reported at 15.17 rad/s and fails its
verdict `cam-speed` there. A new web speed under the knife's running cam ramps the master from 1 to 0.8 m/s: 0.9 m/s at
mid-ramp, 0.8 m/s after 0.5 s, 0.45 m on the way, no step in speed. The disc cam at 60° (mid-rise of a 3-4-5 of 30 mm
over 120°, r_b 60 mm, r_r 15 mm, e 10 mm): s0 = 0.0743303 m, s' = 0.0268574 m/rad, tan φ = 0.188709, φ = 10.687°; at
2π rad/s spring and weight push with 159.420 N, the contact force is 162.234 N, the guide's load 30.084 N and the disc's
torque 4.28161 N m. The toggle press (links 150 mm, crank 50 mm at 120° from the level rod at the dead centre, rod
250 mm), against a circle intersection for the knee and central differences: at −90° of the crank the ram is
35.655 mm above the bottom dead centre with x' = 0.0364766 m/rad, at −20° 1.792 mm and 0.0108256, at −2° 0.0156 mm and
0.00090095, at the dead centre x' = 0, φ' = −0.2886751, x'' = −0.025 m/rad²; 1 mm above it the crank stands at
−0.2646374 rad on one branch and as far past it on the other; 10 N m of the motor (100 N m at the crank) hold 2741.5 N
at −90° and 42530 N at −5°; 10 kN take 79.387 N m at the crank 1 mm before the dead centre and 23.487 N m 0.1 mm
before it; the frame (5e7 N/m) stretches 0.4 mm under the rated 20 kN, the table's 12 kN become 11538.46 N with
0.2308 mm of stretch; J = 0.01859375 kg m² at the dead centre and 0.05605024 far from it; the force capacity at −90° is
145.5 / 0.0364766 = 3988.85 N; a forming stroke from rest to rest takes 20.615 J from the motor in a loss-free run
(19.284 J into the part, 1.331 J into the frame); in the dwell the motor gives under a tenth of the 9.44 N m the same
force would take 1 mm before the dead centre; a forming force moved 20 mm up fails `press-capacity` and
`motor-peak`.

Machine tests (`machine-hexapod`, `machine-cable-robot`): all three pass the standard suite and their default cycles
pass all verdicts. Hexapod: every leg 0.5471158 m long at the home pose (0, 0, 0.48 m), 0.0971158 m out, from the
joints' circles by hand; forward of inverse returns three poses within 1e-12; the legs hold 62.2 kg at home with
115.8772 N each, m g / (6 u_z), and the motors with 0.184424 N m (f lead / 2 pi); off centre the forces still sum to the
weight. Hexapod with rotary actuators (`machine-hexapod-rotary`): at the level pose (0, 0, 0.4 m) every crank stands at
−0.5115438 rad from the closed form by hand (2.208120 rad on the branch "in"); forward of inverse returns three poses
within 1e-12; the rods carry (115 + 6 × 0.2) g / (6 u_z) = 209.7935 N of compression and the cranks hold the platform
with 41.4875 N m, 0.829750 N m at each motor (the rod's force times the torque arm 0.189602 m, the crank's and the rod's
weight); the torque arm r √(a² + b²) sin δ / L is 0.185772 m 100 mm up and zero at the dead centre 0.6378641 m up;
the inverse Jacobian's row (u, b × u) / (u · t) by hand, 4.774645 rad/s of crank per m/s of heave; a heave of 1 m/s²
takes 0.109989 N m more at each motor; the rods' Euler load 10590.31 N, 3530.10 N over the safety; the four new
verdicts pass at their limits and fail beyond; the virtual work's torques equal the crank's Newton-Euler balance with
the rods' forces to 1e-9. Cable robots: at the planar centre each cable is 1.283539 m long (arc and tangent by hand), the tensions are
139.808 N and 100.192 N, the cables' stiffness is 192512 N/m across and 119500 N/m up, the lowest natural frequency
24.605 Hz, the geometric stiffness 235 N/m more; 5 cm below the top pulleys the pose is refused; a cable the closed form
takes below its band is held at the bound and the others carry the rest (8.485 N each by hand); the spatial robot's
forward of inverse returns its poses within 1e-10.

Machine tests (`machine-tripod`): the standard suite and the default cycle with all verdicts. Level at z = 0.54 m every
leg is 0.5491812 m long; tilted 0.15 rad about x the centre shifts by rp (1 − cos 0.15) / 2 = 1.1229 mm and the legs are
0.5489779, 0.5750463 and 0.5241851 m; at (0.15, 0.1) rad the closed form's turn and shift (−7.5203 mrad, 0.61207 and
−1.49654 mm) match their second order and keep each platform joint in its leg's plane; forward of inverse to 1e-12 on 9³
poses; the chord step contracts by under 0.7 over the grown box; level the Jacobian's rows are u_z (1, rp sin t_i, −rp
cos t_i) and its condition number √2; the legs hold the centred 62.7 kg with m g / (3 u_z) = 208.4437 N (0.331749 N m
at the motors), the offset load by moment balance with 274.9330 and 175.1991 N, and push 250.9544 N each rising at 2
m/s²; the platform's momentum on a compliant base carries the payload's offset; each new verdict's value and limit by
hand, and the joints' limits refuse the level pose 0.1° below its 10.49°.


Machine tests (`machine-wafer-handler`): the standard suite with every combination of rigid and elastic gearboxes and
bands; at φ = 60°, θ = 30° the wrist at (0.277128, 0.16) on the radial line and the wafer centre at (0.433013, 0.25),
dr/dφ = −0.554256 m/rad; the R motor's inertia by hand M_φφ = m₁ c₁² + I₁ + m₂ (L² + c₂² − 2 L c₂ cos 2φ) + I₂ + (m₃ + m_w)
4 L² sin² φ, 0.5499656 kg m² at r = 0.12 m (2.485862e-4 kg m² at the motor) and 0.1876677 at r = 0.75 m (1.036671e-4); the
θ motor's 0.153033 and 0.604751 kg m²; turning at r = 0.75 m at sqrt(μ g / r) = 1.980571 rad/s the wafer accelerates at
μ g = 2.941995 m/s², 1.2 times the limit, and motor 2 holds the arm with 0.0162955 N m; extending at 0.5 m/s during a
turn at 1 rad/s at r = 0.5 m the wafer's acceleration is (−0.5, 1.0) m/s²; a torque of 1 N m on the effector stretches
the bands by β = 2.955083e-4 and γ = 5.819704e-4 rad (K₁ = 1692, K₂ = 2303 N m/rad) and turns it out of its radial line by
3.300103e-4 rad; the rigid bands carry dT₁ = −4.271710 N and dT₂ = 4.379926 N at θ̈ = 10 rad/s² with the wrist over the
column; the Z motor holds 11.2276 kg with 0.1752378 N m; the modes against numpy's (18.20 to 186.45 Hz) and the column on its
screw and coupling, 320.8907 Hz by hand; the hold on
pads, vacuum and edge; a span runs slack beyond its pretension; the default cycle passes all verdicts and the reach
speeding up at 3 m/s² slips the wafer.

Pneumatic cylinder tests (`sim-components` pneumatic.rs, `sim-machines/tests/pneumatic.rs`): p A at 6 bar on both
sides (482.5 and 414.7 N, Festo's 483 and 415); the ISO 6358 choked flow C p ρ0 and the elliptic law; the meter-out
throttled speed C ρ0 R T0 / A2 (0.1443 m/s within 1 %, the shipped Z's 0.2156 m/s within 3 %); the air springs' natural
frequency adiabatic (29.64 Hz) and isothermal (25.05 Hz); the air of a double stroke (1.1119 normal litres); the
closed-form filling time while choked (2.555 ms); energy kept in a closed cylinder and the enthalpy balance of an open
one; the stroke time within 1 % at half the step; the checks `cushion-energy` and `load-factor` on the shipped cycle;
a cylinder too stiff for the step refused. The file passes the standard suite with the cylinder at rest.

Winder tests (`machine-winder`): the standard suite (rigid and elastic, with and without the dancer's weight), and holding
at rest at both ends of the roll and a quarter of it. After 640 m the rewind's radius is 110.503 mm and the unwind's
110.969 mm (8231.352 and 4904.800 rad turned); the roll holds 1286.482 m. The roll's inertia at 110.503 mm is 0.1366427
kg m², full 0.4485586 kg m², the inertia ratio 10.4026 at the full roll and 0.2319 on the core. The tension's torque through
the ratio, −2.142857 and 0.642857 N m at the roll's start, −0.507857 and 1.692857 N m at its end (the taper's 79 N). The
torque to speed the line up by 2 m/s² at 640 m: 1.7824107 N m on the rewind at rest, 1.7818049 N m at 2 m/s, −0.8150135 and
−0.8144080 N m on the unwind, the roller's 2.5 N between the spans included; in the rigid model these torques move the line
at exactly 2 m/s² with the dancer at rest. The suite's loss-free runs are made in the middle of the roll, where the radii
are equal; at the roll's start the tension its constant holding torques keep drifts off the wound tension as the radii
part, and the strain the web carries in and out (`power.web_transport`) is what the energy changes by. A span's steady tension for 1 against 1.0005 m/s: EA (v2 / v1 − 1) = 40.000 N
(the span equation to first order 39.980 N). The dancer at a load of 120 N with the rolls held at 100 N stands 39.9948 mm
down (the rolls' radii 2.88 µm smaller after giving the web). The dancer rings on the web at 26.071 Hz, its roller at
56.941 Hz. What leaves the unwind arrives at the rewind or stays in the spans within 1e-7 m over a 3 m run. Without the
dancer's loop a roll wound at 100 N run at 80 N drifts the dancer by −0.24969 mm/s at 2 m/s, with it the trim settles at
0.49938 mm/s. The own verdicts: the inertia ratio 10.4026, the speed at the core 311.11 rad/s, each failing with its rating
just below its value. A category 2 stop and, in the middle of the roll, a category 1 stop keep the web between 30 and
250 N; a category 1 stop near either end of the roll overloads it.

## Limits

- A screw's stiffness is taken at its softest nut position for the whole stroke.
- The arm on its track: the rack's push is taken through the carriage's origin (no moment of its own on the blocks),
  the arm's motor rotors' gyroscopic moments are left out of the guide load, and the blocks are rigid supports (no
  guide stiffness: the carriage does not tilt on them); the guide load is not told to the time-optimal plan
  (`rigid_guide_loads`), which keeps the motors' and gears' limits only.
- A pneumatic cylinder is no tool axis and does not act back on the axis that carries it; its valve switches on/off
  (no proportional valve, no servo-pneumatic position loop); a tube is a flow element whose volume goes into the dead
  volumes; Festo prints no ISO 6358 C and b for the VUVG valves, so the shipped valve's are estimates.
- A link bends in one mode per direction (in its joint's plane; across it and in torsion with
  `link_bending.directions`); with `effects.link_bending_reaction` the bending acts back on the joints, without
  Lagrange's terms of second order in the small bending.
- A worm gear driven back from the load loses as any gearbox does (1 / efficiency) and never locks itself.
- No unbalance on the rotary table, no rack life formula (none published), no cross roller bearing check.
- A cam's master cannot yet be the conveyor of `[conveyor]` (the cam's master is its own, with the conveyor's
  speed-and-ramp model); a new master speed starts its ramp from the master's speed without its acceleration (a step in
  acceleration where it comes during a ramp); the disc cam's follower is a translating one (no swinging lever); the slider-crank's press force acts only on the way out (no springback), the toggle press's also at rest and fades as the ram rises (its frame's stretch is a massless spring, the links are rigid) and the knife's and saw's cut force
  only forwards.
- The linear delta (`delta-linear`, three belt axes on columns that may lean, `geometry.column_incline_deg`) is a
  parallel machine, not one of these; see the [model reference](model-reference.md#linear-delta).

- Hexapod: the legs' own turning about their base joints and the payload's inertia are left out; one stiffness per leg
  (the screw's softest nut position). Its modes are reported with the motors held, as the arms' (`sim-cli modes hexapod`).
- Tripod: the legs' swing about their base joints is left out, as on the hexapod; no friction or play in the revolute
  and spherical joints; one stiffness per leg (the screw's softest nut position); the forward kinematics' convergence is
  shown for the shipped box (a config with another geometry or box should run that test's check again).

- Winder: both rolls have the same core, full radius, motor and gearbox; the web's mass in the spans, the change of the
  spans' length with the rolls' radii, the web's Poisson thinning and the roll's own compliance (its layers sliding,
  telescoping) are left out; the roll's radius is computed from its wound length (a measured diameter or the speed ratio,
  `rewind.radius_from_speeds`, is a signal only); the shaft seals' friction does not follow the gearboxes' temperature
  (their efficiency and drag do); the shipped cycle runs 3 m of the roll's 1286 m at its start, and the other end of the
  roll is judged through the own verdicts, not run; corner blending has nothing to blend on its one reversing axis; the
  drive does not adapt its speed controller to the roll's inertia (45-fold from core to full), so the shipped gains are
  those of the stretch shipped (`[drive.unwind]` its own) and a run near the roll's end needs them swapped.

- Hexapod with rotary actuators: the rods' spin about their own axes and the ball joints' friction are left out; of the
  parts that could cross only the rods are checked against each other; one crank branch for all legs.
- Cable robots: the cables' own inertia is left out (a third of each cable's mass moving with its anchor, 0.5 % of the
  spatial platform's mass), and the sag's weight moving with the pose (5e-5 N); the wrench-feasible workspace of the
  poses is the static one (a cycle's accelerations are held to the band by the time-optimal plan; within the axis limits
  they are judged by the tension verdicts); near the workspace's edge the motors'
  stretch compensation counts on the closed form's tensions where the distribution holds a cable at a bound.
