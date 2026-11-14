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
| `palletiser` | four-axis palletiser, `urdf/palletiser.urdf`, a parallelogram keeping the flange level | gears of ratio 160, the flange's strain wave gear 50 | x, y, z, rz (shipped in tool space), or the four motors' joints | `strain-wave-life` per joint |
| `parallel-scara` | five-bar with parallelogram forearms in the horizontal plane (its own crate, not a joint machine); a Z stroke and a tool turn at the plate | planetary gearboxes of ratio 20 on the arms; the Z stroke on a ball screw 16 x 20 behind a coupling; the turn behind a planetary gearbox of ratio 8 | x, y (m), z (m), c (rad) | `screw-speed`, `screw-dn`, `screw-buckling`, `screw-static`, `screw-life` on the Z stroke |

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
Newton-Euler algorithm (Luh, Walker and Paul 1980): the joint torques for given accelerations, the mass matrix, gravity
and velocity terms. The payload is a rigid body on the last link: its mass `arm.payload_kg`, its centre
`arm.payload_offset_m` from the URDF's tip frame and its inertia about the centre `arm.payload_inertia_kgm2` (xx, yy, zz,
xy, xz, yz in the tip frame), merged into the last body by the parallel axis theorem (`Inertial::merged`).

Config `[arm]`: `urdf_file` is a path, or `builtin:six-axis`, `builtin:scara`, `builtin:five-axis`,
`builtin:seven-axis` or `builtin:palletiser` for the examples in `crates/machine-joint-kit/urdf/`. `gear_ratios` has one
entry per motor; each motor joint's limits are in `[axis.j1]`, `[axis.j2]`, ... (`max_velocity_rad_s`, or
`max_velocity_m_s` for a prismatic joint, and so on). The URDF's limits are the joints' stops; a continuous joint takes
`[joint]` `min_angle_deg` and `max_angle_deg`. A prismatic joint slides on the `[screw]`, coupled to its motor (its
ratio must be 1; the `[gearbox]`'s stiffness and damping stand for the coupling, its efficiency and inertia do not
count), with the `[slide]` friction (the SCARA's quill). A URDF limit left out is 0; an empty range is refused. With a
`[strain_wave]` table each revolute joint's gear gets the life check.

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
feedforward and the poses at rest see the payload fixed to the flange. The palletiser's carton swings at 2.2 Hz
(sqrt(k / (m l^2) + g / l) / 2 pi with 120 N m/rad, 20 kg, 0.2 m).
shortcut: without the payload the swing's joints carry a unit inertia of their own, apart from the arm.

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
  names, as many as the motors or one fewer (a redundant arm). The orientation is the rotation vector (axis times angle,
  base frame) of the turn from the flange's orientation at zero joints: a URDF whose flange points down at zero is at
  0 there, not half a turn from the base frame. A pose within 0.01 rad of half a turn from that is refused
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
  and acceleration. The condition number is that of the weighted Jacobian (of its transpose on a redundant arm).

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
and sags under its weight by 49.2 um, 1.3 % above the exact q L^4 / (8 E I).

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
catalogue motor; a time-optimal plan of a crawl is refused beyond ten million points. A screw 1.6 m long fails `screw-speed`, and a life asked for beyond the
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
- A link bends in one mode in the plane its joint turns it in; out of that plane and in torsion it is rigid, and the
  bending and the base do not act back on the joints (one-way).
- A worm gear driven back from the load loses as any gearbox does (1 / efficiency) and never locks itself.
- No unbalance on the rotary table, no rack life formula (none published), no cross roller bearing check.
- Not built yet: mechanical (disc) cams with a follower and their contact force; a cam's master cannot yet be the
  conveyor of `[conveyor]` (the cam's master is its own, with the conveyor's speed-and-ramp model); no MC_CamIn over
  OPC UA; a changed master speed is taken at once, not ramped; the crank's and the knife's joint has no stops while
  the cam turns it on; the press force acts only on the way out (no springback) and the knife's and saw's cut force
  only forwards.
- The linear delta (`delta-linear`, three belt axes on columns that may lean, `geometry.column_incline_deg`) is a
  parallel machine, not one of these; see the [model reference](model-reference.md#linear-delta).
