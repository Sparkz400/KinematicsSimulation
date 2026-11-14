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
- [Parallel kinematics: hexapod and cable robots](#parallel-kinematics-hexapod-and-cable-robots)
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

## Parallel kinematics: hexapod and cable robots

Two parallel kinematics have crates of their own, `machine-hexapod` and `machine-cable-robot`, run and sized like every
other machine (`sim-cli analyze hexapod`, `sim-cli verdicts cable-planar`, the app's 3D view, `search`):

| Id | Mechanism | Drives | Tool pose | Own checks |
| --- | --- | --- | --- | --- |
| `hexapod` | Stewart-Gough platform, 25 kg plus up to 30 kg, on six electric cylinders (450 mm retracted, 200 mm stroke) | per leg a ball screw 25 x 10 behind a coupling on a 1FK2105-4AF | x, y, z (m), rx, ry, rz (rad, a rotation vector) | the screw's `screw-speed`, `screw-dn`, `screw-buckling`, `screw-static`, `screw-life` per leg, under the leg's force |
| `cable-planar` | a carriage of 3 kg plus up to 2 kg, a point in a vertical plane, on 4 cables from a frame 2 m x 1.6 m | per cable a winch: 25 mm drum behind a planetary gearbox 5:1 | x, z (m) | `cable-max-tension`, `cable-min-tension` per cable |
| `cable-spatial` | a platform of 4 kg plus up to 4 kg, six degrees of freedom, on 8 cables from the corners of a frame 2 x 1.6 x 1.8 m | as the planar one, 30 mm drum | x, y, z (m), rx, ry, rz (rad) | as the planar one |

Their proportions are generic, every value an `[estimate]` but the motor's and drive's (catalogue, as above). Each
default cycle passes all its verdicts; the cable robots' settling tolerances are requirements of their size (0.3 mm
planar, 1 mm spatial), not the ball screw axis's 0.05 mm.

### Hexapod

Each leg is a joint of `chain::JointMachine` whose coordinate is the cylinder's extension q_i, so the screw, its nut and
bearings, the coupling, backlash, the friction of the rod's guide and the nut's preload, the stops at the stroke's ends
and the motor are those of the ball screw axis above. The mechanism gives the platform's kinematics and dynamics:

- Inverse kinematics in closed form: leg i runs from its base joint B_i to the platform joint p + R a_i, its length L_i =
  `leg.closed_length_m` + q_i. The joints sit in pairs: base pairs at 120 k ± `hexapod.base_half_angle_deg`, platform
  pairs at 120 k ± (60 − `platform_half_angle_deg`), each leg to the neighbouring pair.
- Forward kinematics by Newton's method on the platform's twist, from the centre of the tool's box: xi = J^-1 (q −
  q(x)), p += v, R = exp(w) R, until the legs miss by less than 1e-13 m; legs that no pose fits (no convergence within 40
  steps) give NaN, which every caller refuses.
- J's rows are (u_i, b_i × u_i), u_i the leg's direction and b_i = R a_i. Dynamics by the platform's twist: M_xi xi' +
  c + g = J^T f, so in leg space M_q = J^-T M_xi J^-1 and h = J^-T (c + g − M_xi J^-1 a0), a0 the legs' length
  acceleration at the twist (checked against the Lagrange equations of M_q by differences: equal to 1e-9). Each leg's
  mass is split between its ends: `leg.platform_share` moves with the platform joint (a point mass in the platform's
  mass and inertia), the rest sits still at the base.
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
- Each winch is a `joint::RotaryJoint` in metres of cable: motor, gearbox with the drum's radius as its lever, the drum's
  inertia, friction of bearings and pulley (`effects.winch_friction`), stops at an empty and a full drum. Elastic, drum
  and rotor are bodies of their own behind the gearbox's spring; rigid, the cables keep their length and the winches'
  inertia joins the platform's.
- The motors' angles of a pose are those of the free lengths that hold the platform there at rest, l = L / (1 + t /
  EA), t the closed form's tensions (the distribution's wherever no cable sits at a bound), so position setpoints and the
  torque feedforward (the distribution's tensions for the platform's wrench, plus the winches' inertia) ask the same
  pretension of the cables. The forward kinematics is Gauss-Newton on these lengths; the controller uses it twice a
  position cycle (the following error in tool space), so it takes cheap steps first and the numerical Jacobian only near
  the workspace's edge.
- Signals per cable: `cable.<k>.tension`, `cable.<k>.shortfall` (how far it is below the least tension), `cable.<k>.length`;
  `platform.condition`, the structure matrix's condition number. Verdicts: `cable-max-tension` (highest tension against
  the rating) and `cable-min-tension` (any shortfall fails), the motors and drives as on every machine.

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
cycles pass their verdicts. A screw 1.6 m long fails `screw-speed`, and a life asked for beyond the
screw's fails `screw-life`. With losses: the rack's gearbox torque and feed force are the load's force over 0.97, the
screw's life shrinks by a1 at 99 % reliability, and the temperature feedback keeps the efficiency of screw, rack and
worm.

Machine tests (`machine-hexapod`, `machine-cable-robot`): all three pass the standard suite and their default cycles
pass all verdicts. Hexapod: every leg 0.5471158 m long at the home pose (0, 0, 0.48 m), 0.0971158 m out, from the
joints' circles by hand; forward of inverse returns three poses within 1e-12; the legs hold 62.2 kg at home with
115.8772 N each, m g / (6 u_z), and the motors with 0.184424 N m (f lead / 2 pi); off centre the forces still sum to the
weight. Cable robots: at the planar centre each cable is 1.283539 m long (arc and tangent by hand), the tensions are
139.808 N and 100.192 N, the cables' stiffness is 192512 N/m across and 119500 N/m up, the lowest natural frequency
24.605 Hz, the geometric stiffness 235 N/m more; 5 cm below the top pulleys the pose is refused; a cable the closed form
takes below its band is held at the bound and the others carry the rest (8.485 N each by hand); the spatial robot's
forward of inverse returns its poses within 1e-10.

## Limits

- The arms are sized in joint space: no Cartesian paths and no inverse kinematics of the tool.
- Every joint of an arm has the same motor, drive and gearbox (only the ratio differs per joint), as the drive reads
  one `[motor]` and `[drive]` per machine; no gearbox life for arms whose joints have different ratios.
- A screw's stiffness is taken at its softest nut position for the whole stroke.
- A worm gear driven back from the load loses as any gearbox does (1 / efficiency) and never locks itself.
- No unbalance on the rotary table, no rack life formula (none published), no cross roller bearing check.
- Not built yet: mechanical cams, electronic cams and flying saw profiles.
- Hexapod: the legs' own turning about their base joints and the payload's inertia are left out; one stiffness per leg
  (the screw's softest nut position); no modes reported (`resonance_hz`) for it yet.
- Cable robots: the cables' mass and sag are left out; the wrench-feasible workspace is the static one (the cycle's
  accelerations are judged by the tension verdicts, not planned around); near the workspace's edge the motors'
  stretch compensation counts on the closed form's tensions where the distribution holds a cable at a bound.
