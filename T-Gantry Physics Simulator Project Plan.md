# T-Gantry Physics Simulator: Project Plan

Nov 11, 2026 · @Nico

## Goal and scope

The goal is a simulator of a two-motor T-gantry in which torque, speed and following error come out of a physical model, not from scripted values. It should be good enough to size motors, tune gains and compare motion profiles before touching hardware. The T-gantry is the first machine on a modular base, so other kinematics can be added later without rewriting the physics, the control or the app.

In scope:

- Coupled belt kinematics and dynamics for X and Z, with gravity on Z
- Two servo drives with cascaded position, velocity and current control
- Jerk-limited motion profiles for point-to-point moves and pick-and-place cycles
- An animated 3D view of the machine next to live scopes, per motor and per axis
- A German and an English interface, switchable while the program runs
- Peak and RMS torque per cycle, for motor sizing
- Belt stretch, pretension, detailed friction, backlash and torque ripple in the first version, each with its own switch
- A PLC link over OPC UA as a later stage, with the PLC sending setpoints

Out of scope for the first version:

- Matching one specific machine to the percent. That needs measured drive traces.
- Detailed CAD models and collision checks. The first version builds the 3D view from simple shapes.
- Electrical detail below the current loop, such as PWM and inverter switching
- Closing a servo loop through the PLC. OPC UA is too slow for that, so the fast loops stay in the simulator.

## Technology

The simulator is written in Rust. Only the app depends on the graphics and interface libraries; core, components and machines are plain Rust with no such dependencies.

| Part | Choice | Used for |
| --- | --- | --- |
| Language | Rust, stable toolchain | Everything |
| App shell and 3D | Bevy | Window, 3D scene, camera, lighting, desktop and browser builds |
| Panels and scopes | egui through [bevy\_egui](https://github.com/mvlabat/bevy_egui/releases), with [egui\_plot](https://github.com/emilk/egui_plot) | Parameter panel, effect switches, live scopes |
| Interface languages | One translation file per language (Fluent format), German and English | Every text the user sees |
| Configuration | TOML | Machine parameters, one file per machine |
| PLC link | [async-opcua](https://github.com/freeopcua/async-opcua) | OPC UA client and server |
| Tests | cargo test in CI | Component tests and the standard machine suite |

All library versions are pinned in the workspace. Bevy and egui change their APIs between releases, so upgrades happen deliberately between phases.

## Architecture

The simulator is built in four layers, and only one of them knows what a T-gantry is. Adding another kinematics means adding one machine module; core, components and front ends stay untouched.

&#91;embedded content: architecture · 3 front ends, 1 core\]

Each layer uses only the layer below it. The T-gantry module is the single place where its transformation, belt routing and 3D scene live.

**What each layer owns**

| Layer | Crates | Owns | Must not know |
| --- | --- | --- | --- |
| Front ends | sim-app, sim-cli, sim-opcua | Windows, 3D rendering, scopes, both interface languages, files, the PLC connection | Any machine by name |
| Machines | machine-t-gantry, machine-single-axis | Transformation, which components it has and how they connect, its 3D scene description, default config | How time stepping or the interface works |
| Components | sim-components | Belt span, friction contact, gearbox, motor, drive, profile generator, encoder | Which machine they sit in |
| Core | sim-core | Fixed-step integrator, state vector, signals with names and units, config loading, test harness | Any physics |

**What a machine module supplies**

- Its tool axes and its motors, with names and units
- Forward and inverse transformation between the two
- Its assembly: masses, belt spans with their length as a function of pose, friction contacts, gearboxes
- Accelerations from the current state and the motor torques
- The signals it exposes to the scopes and to the PLC
- A 3D scene of itself: parts with shape, size and pose for the current state
- A config schema with default values

As a Rust sketch, to be refined in phase 1:

```rust
pub trait Machine {
    fn axes(&self) -> &[AxisInfo];      // tool side, e.g. X and Z
    fn motors(&self) -> &[MotorInfo];
    fn inverse(&self, tool: &[f64], motors: &mut [f64]);
    fn forward(&self, motors: &[f64], tool: &mut [f64]);
    fn derivatives(&self, state: &State, torque: &[f64], out: &mut State);
    fn signals(&self, state: &State, out: &mut SignalFrame);
    fn scene(&self, state: &State, out: &mut Scene);
}
```

**Adding a kinematics later**

1. Create a new machine crate from the template.
2. Write its forward and inverse transformation.
3. Assemble it from existing components. Add a new component only when a part is truly new, such as a rotary link.
4. Add its 3D scene, its German and English names, and its default config.
5. Run the standard machine test suite. It must pass unchanged.
6. Register the machine in one list. App, CLI and PLC link pick it up from there.

**Rules that keep it maintainable**

- Dependencies point one way, downwards, and CI checks it.
- No machine name appears outside its own crate, except in the registration list.
- Every component has its own unit tests and its own on/off switch.
- Every machine passes the same test suite: transformation round trip, energy conservation, static hold, step-size convergence.
- Scopes and the PLC link are generated from the signal list, never hand-written per machine.
- A machine never renders. It only describes its parts, so the renderer can change without touching any machine.
- No text for the user is written in code. Every signal, parameter and label has a fixed key and a German and an English entry, and CI fails when one is missing.
- One config file per machine, with a format version.

**Runtime behaviour**

- **Time.** The core advances in fixed steps and never looks at the wall clock. The app decides how many steps to run per frame, which gives pause, slow motion and faster-than-real-time for free.
- **Threads.** In the app, the simulation runs in its own thread and writes samples to a ring buffer. The interface reads that buffer at screen rate and never blocks the physics.
- **Determinism.** The same config and the same commands give bit-identical traces. That makes the tests exact and makes a bug reproducible from a config file.
- **Batch runs.** The CLI runs the same core as fast as the CPU allows and writes CSV, for sweeps over payload, gains or jerk.

## Physics model

The model includes every effect that visibly shapes torque, speed and following error on a belt-driven gantry. Each effect has its own switch, so its contribution can be seen in isolation.

**Effects in the model**

| Effect | How it is modelled | What it does to the traces |
| --- | --- | --- |
| Belt stretch | Every belt span is a spring with k = EA / L; span lengths change with x and z | Oscillation after each acceleration change, resonance that shifts with position, lag between motor and load |
| Belt damping | Viscous term per span | How quickly the oscillation dies out |
| Belt pretension and slack | Span tension cannot go below zero; pretension sets the margin | Loss of stiffness and risk of tooth jump at high acceleration |
| Belt mass | Added to the moving masses and pulley inertias | More torque at high acceleration |
| Guide friction on X and Z | LuGre model: Coulomb, viscous, stiction and the Stribeck drop between them | Direction-dependent torque offset, reversal spikes, stick-slip at low speed, standstill error |
| Load-dependent friction | Coulomb level scales with guide load, including the tilting moment of the overhanging Z axis | Friction that grows with payload and acceleration |
| Motor and bearing friction | Coulomb plus viscous on each motor shaft | Torque floor and speed-dependent losses |
| Gearbox | Ratio, efficiency, backlash as a dead zone, torsional stiffness | Lost motion on reversal, limit cycles at high gain |
| Motor torque ripple and cogging | Position-dependent torque harmonics | Speed ripple, most visible at low speed |
| Tooth meshing | Small periodic force at the tooth-pass frequency | Speed-dependent vibration line |
| Gravity | On vertical axis and payload | Holding torque, different behaviour up and down |
| Payload | Mass that can switch mid-cycle | Load step at pick and at release |
| Frame compliance | One spring-mass mode for the beam and its mounting | A low structural resonance |
| Temperature | Scale factors on friction and belt stiffness, set by hand | Cold machine versus warm machine |

**Left out of the first version (see Optional additions)**

- Sideways string vibration of the free belt spans: audible on a real machine, but barely visible in drive signals
- Tooth-level contact mechanics and wear: this is finite-element territory and far too slow for real time
- Air drag: negligible next to friction and inertia at gantry speeds
- A computed thermal model: temperature is a manual setting instead

**Kinematics.** With pulley radius r and motor angles A and B at the pulley (after the gearbox), for one choice of belt routing and sign convention:

```latex
x = \frac{r}{2}(\theta_A + \theta_B), \qquad z = \frac{r}{2}(\theta_A - \theta_B)
```

With a stretching belt this holds only on average. Motor angles and load positions are separate states, and the difference between them is the belt stretch.

**Belt.** Each span i between two pulleys carries a tension:

```latex
T_i = \max\!\left(0,\; T_0 + k_i\,\Delta L_i + c_i\,\Delta\dot{L}_i\right), \qquad k_i = \frac{EA}{L_i(x,z)}
```

**Motors.** Each motor is accelerated by its own torque against the two belt tensions at its pulley:

```latex
J\,\ddot{\theta}_A = \tau_A - r\,(T_{A,\mathrm{in}} - T_{A,\mathrm{out}}) - \tau_{f,A}
```

**Load.** Carriage and vertical axis are accelerated by the net belt forces on them, minus guide friction and, on Z, gravity.

**Friction.** The LuGre model adds one internal state s per friction contact, which is what produces stiction and stick-slip:

```latex
\dot{s} = v - \frac{\sigma_0\,|v|}{g(v)}\,s, \qquad F_f = \sigma_0 s + \sigma_1 \dot{s} + b\,v
```

```latex
g(v) = F_c + (F_s - F_c)\,e^{-(v/v_s)^2}
```

**Rigid limit.** With an infinitely stiff belt the model collapses to two independent axes. This simple form is kept as the reference the full model is tested against:

```latex
\left(m_x + \frac{2J}{r^2}\right)\ddot{x} = F_x - F_{f,x}, \qquad \left(m_z + \frac{2J}{r^2}\right)\ddot{z} = F_z - m_z g - F_{f,z}
```

**Motor and drive.**

- Rotor inertia, torque limit from the current limit, torque-speed curve from the voltage limit
- Current loop as a first-order lag at first; an R-L winding model with a PI current controller is a later refinement

**Integration.** Belt springs and the friction states make the equations stiff. The default is a 62.5 µs base step with sub-steps and a semi-implicit integrator. The final choice is made by the convergence test in the validation section.

**Parameters.** All of them live in one config file (TOML) with units in the field names. Masses, inertias, backlash and the belt's specific stiffness come from datasheets. Friction and damping values do not; they start as estimates with a low and a high bound.

## Control and motion model

The simulator copies the structure of a real system: a motion controller plans in X and Z, transforms to motor setpoints, and each drive closes its own loops on its own motor encoder.

**Signal chain per cycle**

1. The profile generator produces position, velocity and acceleration setpoints for X and Z.
2. The inverse transformation turns them into setpoints for motors A and B.
3. Each drive runs position control (P gain, the Kv factor), then velocity control (PI), then the current loop.
4. Velocity and torque feedforward are added, each with an on/off switch and a weighting factor.
5. The resulting motor torques go into the physics model.

**Cycle times.** Configurable, with these defaults as typical servo-drive values:

| Loop | Default cycle |
| --- | --- |
| Current | 62.5 µs |
| Velocity | 125 µs |
| Position | 1 ms |
| Profile generator | 1 ms |

**Following error.** Position setpoint minus actual position, computed per motor in the position loop and transformed back to X and Z for display. No extra modelling is needed; it appears as soon as the loops and the mechanics run together.

**Motion profiles.**

- Jerk-limited seven-segment S-curve per axis, with limits for velocity, acceleration and jerk
- Synchronised X and Z moves, so both axes finish together
- A pick-and-place cycle built from segments: lift, traverse, lower, dwell
- Corner blending between segments as a later addition
- A check that axis limits do not exceed motor limits, since a diagonal move loads one motor with both axes

**Measurement effects (optional switches).** Encoder resolution, velocity-filter delay, torque-setpoint filter and a notch filter for the belt resonance. These are what limit achievable gains on a real drive, so the simulation is too optimistic without them.

## Milestones

Seven phases, each ending in something that runs and can be checked. Phases 0 to 5 give the full-physics T-gantry on the modular base in roughly 28 to 43 working days; effort figures are estimates for one person.

The first task is phases 0 and 1: the base. That means the workspace, the core as our own small physics engine (fixed-step integrator, state, signals) and the machine interface, proven with the single-axis machine and the rigid T-gantry.

| Phase | Delivers | Done when | Effort (working days) |
| --- | --- | --- | --- |
| 0. Setup | Rust workspace with the four layers as separate crates; config file format; German and English translation files; CI running tests and the dependency-direction check | An empty window opens and one dummy test passes | 1 to 2 |
| 1. Base and rigid reference | Core with integrator and signal list; the machine interface including the 3D scene description; the single-axis test machine; the T-gantry as a rigid model | Both machines run on the same core with no machine-specific code outside their crates; energy, static-hold and constant-torque tests pass | 5 to 7 |
| 2. Full mechanics | Belt span, friction contact, gearbox and motor as reusable components with one switch each; the T-gantry assembled from them, with payload switching and frame mode | Belt natural frequency matches the hand calculation within 5 %; with very high stiffness the traces converge to the rigid reference | 8 to 12 |
| 3. Control and profiles | Drive and profile generator as components: cascade loops, feedforward, notch filter, S-curves, pick-and-place cycle | Following error at constant speed matches v / Kv without feedforward | 4 to 6 |
| 4. App | Machine picker; 3D view built from the machine's scene, with orbit camera, view presets and visible belt stretch; scopes built from the signal list; parameter panel; effect switches; German and English; pause and slow motion | Switching machine or language needs no code change; switching one effect off changes the traces within one cycle | 8 to 12 |
| 5. Analysis | Peak and RMS torque per cycle, operating points on the torque-speed curve, frequency sweep, CSV export | Numbers for a reference cycle match a hand calculation | 2 to 4 |
| 6. PLC link | OPC UA process whose variables are generated from the signal list | A PLC-commanded move runs in the simulator and reports back position and following error | 6 to 10 |

The rigid model in phase 1 is not what ships. It exists because the full model needs something exact to be tested against.

A possible phase 7 is parameter identification from real drive traces. It is not planned in detail because it depends on access to a machine.

The first usable result comes after phase 3, as a command-line run that writes traces to CSV. That is the point to check the physics before investing in the interface.

## Validation

Every phase has a test that compares the simulation against something known independently, and these run automatically on each change.

| Check | Expected result | Phase |
| --- | --- | --- |
| Transformation round trip, every machine | Forward of inverse returns the starting pose within 1e-9 | 1 |
| Modularity | Single-axis machine and T-gantry run on the same core and front ends; no machine name outside its crate | 1 |
| Energy conservation with friction and control off, every machine | Total energy drifts less than 0.1 % over 10 s | 1 and 2 |
| Static hold on Z | Each motor holds m\_z g r / 2, with opposite signs | 1 |
| Constant torque | Acceleration equals force divided by effective mass | 1 |
| Step-size convergence, every machine | Halving the time step changes results by less than 1 % | 1 and 2 |
| Rigid limit | With very high belt stiffness, the full model matches the rigid reference within 1 % | 2 |
| Belt natural frequency | Matches the hand calculation from k and m within 5 % | 2 |
| Friction levels | Breakaway force equals the stiction level; steady sliding force equals Coulomb plus viscous | 2 |
| Backlash | Lost motion on reversal equals the configured backlash | 2 |
| Independent reference | One reference move matches a SciPy implementation of the same equations | 2 |
| Following error at constant speed, no feedforward | Equals v / Kv | 3 |
| Profile limits | Velocity, acceleration and jerk never exceed their limits | 3 |
| Real traces | Simulated torque and following error overlay measured drive traces | 7 |

Checks marked "every machine" form the standard suite that a new kinematics must pass unchanged. Component checks such as friction levels and backlash run once on the single-axis machine and then hold for every machine that uses the component.

Every check except the last proves the equations are solved correctly. Only the last one proves the parameters describe a real machine.

## Risks and open questions

The biggest risk is not the code but the parameters: friction and damping cannot be read off a datasheet, and a fuller model has more of them.

| Risk | Effect | Mitigation |
| --- | --- | --- |
| More effects mean more unknown parameters | A detailed model with guessed values is no more accurate than a simple one | Sensitivity sweep to find which parameters matter; low and high bounds on each; identify the important ones from traces |
| Friction and damping values unknown | Right trends, wrong absolute numbers | Run sizing with low and high values; identify from traces when a machine is available |
| Machine interface shaped around one machine | The next kinematics does not fit and the base has to be reworked | Build the single-axis machine alongside the T-gantry from phase 1; treat the interface as provisional until one further kinematics is in |
| Too much abstraction too early | Slower progress and harder debugging for no gain | Only four layers and one interface; a part becomes a shared component when the second machine needs it |
| Stiff equations from belt springs and friction states | Unstable or slow simulation | Step-size convergence test; semi-implicit integrator with sub-steps |
| Effects interact and hide bugs | Wrong results that look plausible | One switch per effect; each component has its own test before they are combined |
| Rust learning curve | Phases 0 to 2 take longer | The machine interface is the one trait to learn early; keep async and lifetimes out of the core |
| Bevy and egui change their APIs between releases | Upgrade work | Pin versions; only the app depends on them; upgrade deliberately between phases |
| UI frame rate couples to simulation time | Physics changes with window load | Simulation runs in its own thread on its own clock; the UI only reads buffers |
| OPC UA timing jitter | Jerky setpoints from the PLC | Interpolate setpoints in the simulator; keep all servo loops local |
| 3D polish eats time meant for physics | A good-looking app on an unverified model | Simple shapes first; the physics checks of phases 1 to 3 run from the command line before the app exists |

Open questions to settle before phase 1:

- [ ] Which machine is the reference: a catalogue T-gantry or a custom build? Strokes in X and Z?
- [ ] Payload range and target cycle time?
- [ ] Which motors, gearboxes and drives, so limits and cycle times match?
- [ ] Desktop app only, or should it also run in the browser?
- [ ] For the PLC link: which controller, and is the PLC the OPC UA server with the simulator as client?
- [ ] Is there access to a real axis for traces later?

## Optional additions

None of these is needed for the first full-physics version. Each can be added on its own afterwards, with its own switch like the other effects.

**Extra physics**

- [ ] Sideways vibration of the free belt spans, as a string model per span
- [ ] Tooth contact and wear: a simplified tooth stiffness and a wear estimate from load cycles
- [ ] Air drag on carriage and vertical axis
- [ ] Computed thermal model: winding and friction heat that feed back into friction and belt stiffness, replacing the manual temperature setting

**Further kinematics**

Belt-driven machines reuse the existing components. Machines with arms need one new component first, a rotary link with a joint.

- [ ] H-gantry: the same single-belt principle in a flat XY plane
- [ ] Line gantry: X and Z with one motor per axis
- [ ] Area gantry: X, Y and Z
- [ ] CoreXY
- [ ] Rotary link and joint component
- [ ] Delta robot
- [ ] SCARA

**To do: a separate view per effect**

The goal is that a user can look at one physical effect alone, instead of reading it out of the torque trace.

- [ ] Friction view: force-over-speed curve per contact with the live operating point, and the split into Coulomb, viscous and stiction
- [ ] Thermal view: temperatures of windings, gearboxes and guides over time, plus colour on the 3D model
- [ ] Belt view: tension and stretch per span, drawn as colour on the belt
- [ ] Torque breakdown: how much of each motor's torque goes to inertia, gravity, friction and belt forces
- [ ] Loss budget: energy per cycle lost to each friction contact, belt damping and the motors
- [ ] Effect comparison: the same move with one effect on and off, overlaid in one scope

**Visual extras**

- [ ] Detailed 3D model files per machine part, in place of simple shapes
- [ ] Further interface languages

## Open-source release

Six additions take the simulator from a personal tool to a public open-source project. Three are needed before the first public release; the other three are features that follow it.

**Before the first public release**

- [ ] Open-source basics: dual MIT / Apache-2.0 licence, contribution guide, public roadmap, signed installers for Windows and Linux
- [ ] Dependency licence check, since async-opcua is under MPL-2.0
- [ ] Documentation: user manual in German and English, plus a model reference that states every equation and assumption
- [ ] Component catalogue: real motors, gearboxes, belts and guides as data files in an open format that anyone can contribute to

**Features after the release**

- [ ] Energy and electrical: consumption per cycle, DC link, braking resistor sizing
- [ ] Lifetime estimates: belts, bearings and guides from the simulated loads
- [ ] FMI export, so the models plug into Simulink and other simulation tools

**Where each one fits in the architecture**

| Addition | Lives in | Change to the base |
| --- | --- | --- |
| Component catalogue | One data file per part, loaded by the components layer; a machine config names its parts | Config loading learns to resolve part names |
| Energy and electrical | Drive component, plus new signals | None; scopes pick up the signals automatically |
| Lifetime estimates | Analysis on recorded traces | None; the physics is untouched |
| FMI export | A further front end next to app, CLI and PLC link | None, if the machine interface holds |
| Documentation | Model reference written per component and per machine, next to the code | None |

The catalogue is the only one that should influence the base early: the config format in phase 0 should already allow a part to be given by name instead of by raw numbers.
