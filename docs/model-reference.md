# Model reference

The equations the simulator solves, taken from the code and its comments. Each section names the source file. Config keys are written as `table.key`; their units are in the [config reference](config-reference.md).

- [Structure of one time step](#structure-of-one-time-step)
- [Integrator](#integrator)
- Components: [belt](#belt-sections), [guide friction](#guide-friction-lugre), [motor shaft](#motor-shaft-friction-ripple-cogging), [tooth meshing](#tooth-meshing), [gearbox](#gearbox), [hard stops](#hard-stops), [drive and motor](#drive-and-motor)
- Machines: [T-gantry](#t-gantry), [single axis](#single-axis), [delta picker D3](#delta-picker-d3), [linear delta](#linear-delta)
- Motion: [controller](#motion-controller), [profile generator](#profile-generator), [corner blending](#corner-blending), [path planners](#path-planners), [input shaper](#input-shaper)
- [Analysis quantities](#analysis-quantities), [verdicts](#verdicts), [other analyses](#other-analyses), [effect views](#effect-views), [fit](#fit)
- [Known shortcuts](#known-shortcuts), [other simplifications](#other-simplifications)

Notation: x, z tool axes; a, b pulley angles (after the gearbox); φ motor shaft angle; r pulley radius; i gear ratio; T belt tension; τ torque; ω speed; g = 9.80665 m/s².

## Structure of one time step

Source: [`sim.rs`](../crates/sim-components/src/sim.rs), [`control.rs`](../crates/sim-components/src/control.rs).

One base step `sim.dt_s` (62.5 µs):

1. The controller reads the motor angles and speeds from the state at the start of the step and runs the loops that are due:
   - profile generator, every `profile.cycle_s` (1 ms): new tool setpoints, input shaping, inverse transformation, fieldbus delay
   - position loop, every `drive.position_cycle_s` (1 ms)
   - velocity loop, every `drive.velocity_cycle_s` (125 µs)
   - current loop, every base step: the voltage each winding gets over the step
2. The machine is told whether the payload is attached, whether the cycle dwells and whether the drives are disabled.
3. The mechanics and the winding currents are integrated together over the base step with the voltages held constant (zero-order hold, as an inverter holds its voltage over a PWM period); the motor torque follows the current within the step. With the drives disabled, the mechanics alone without torque.
4. The drives take the step's end: current, electrical energy, copper loss and shaft work; then the DC links (a shared one after all drives), the trip, and the winding's heating.

The simulation never looks at the wall clock. The same config and the same commands give bit-identical results.

## Integrator

Source: [`integrate.rs`](../crates/sim-core/src/integrate.rs).

Classical fourth-order Runge-Kutta with `sim.substeps` (4) sub-steps per base step, so the mechanical step is 15.6 µs. The integrator treats the state as one vector and does not know which entries are positions. The simulation (`Sim::step` in [`sim.rs`](../crates/sim-components/src/sim.rs)) hands it the machine's state with four entries per drive appended: the winding current and, from zero at the start of the step, the electrical energy ∫ u i dt, the copper loss ∫ R i² dt and the shaft work ∫ τ ω dt. The machine and the core know nothing of the windings; the drive gives their rates (`DriveParams::winding_rates`).

Assumptions: the winding voltages are held over a base step (zero-order hold), as an inverter would; the torque follows the current within the step. Friction, backlash and belt slack are not handled as events; they are integrated through with the fixed step. The step-size test (halving the step changes the travel by less than 1 %) is what justifies that.

## Belt sections

Source: `Belt` in [`mech.rs`](../crates/sim-components/src/mech.rs).

A *section* is the belt between two points that hold it: a driven pulley or a clamp. Idlers in between are free, so the tension is the same along the whole section. A section has a geometric length `geo` (set by the pose) and holds `mat` of unstretched belt (changed by the pulleys feeding belt in or out).

```text
e        = geo − mat                                   stretch
k        = EA / mat                                    stiffness
T_el     = EA · e / mat                                elastic tension
T        = T_el + c_s · (d geo/dt − d mat/dt)          with damping
tension  = max(0, T)                                   with effects.belt_slack
```

**Damping variants.** `c_s` is one of:

| Switches | c_s |
| --- | --- |
| `belt_damping` on, `belt_material_damping` off | `belt.damping_n_s_m`, the same for every section whatever its length |
| both on | `belt.damping_time_s` · EA / mat: in proportion to the section's stiffness, so a short section damps more |
| `belt_damping` off | 0 |

**Slack.** With `effects.belt_slack` the tension cannot go below zero. With the switch off the belt can push. Each machine counts the seconds during which any section has a tension of zero or less (signal `belt.slack_time`); the tooth jump itself is not modelled.

**Energy-consistent form.** The force that pulls on the clamps and the load is `tension`. The force on the belt material that a pulley feeds into the section is larger:

```text
material = tension + T_el · e / (2 · mat)  =  tension + T_el · strain / 2
V        = ½ · EA · e² / mat                           stored energy
```

Stretched belt carries its strain energy along when it passes a pulley, and `material` is −∂V/∂mat. The pulleys are driven with `material`, the load with `tension`. The factor is about 1.0003 and makes the model conserve energy exactly. A slack section (e < 0) has tension and material force both max(0, T) and no stored energy.

**Pretension and pulley radius.** A section of length `geo` at pretension T₀ holds geo / (1 + T₀/EA) of unstretched belt. The pulley radius is taken to hold at pretension, so a pulley feeds `r_mat = r / (1 + T₀/EA)` of unstretched belt per radian. That makes the kinematics exact for an evenly tensioned belt.

**Tooth stiffness** (`effects.belt_tooth_stiffness`). The teeth in mesh on a driven pulley are a spring `belt.tooth_mesh_stiffness_n_m` in series with the section. It is modelled as extra belt length EA / k_tooth, added to the geometric length once per driven pulley the section ends at. Then k = EA / mat is belt and teeth in series: 1/k = L/EA + 1/k_tooth.

**Belt mass** (`effects.belt_mass`). Lumped on the pulleys as inertia, not spread along the belt: see each machine.

**Temperature** (`effects.temperature`). EA is multiplied by `temperature.belt_stiffness_scale`.

**At rest.** The unstretched belt per section at an equilibrium is found by bisection from two conditions: the tension differences the load needs, and the total amount of belt, which is fixed.

Not modelled: sideways (string) vibration of the free spans, tooth contact, wear, a belt mass that moves with the load.

## Belt tooth jump

Source: `ToothJump` in [`mech.rs`](../crates/sim-components/src/mech.rs); `effects.belt_tooth_jump`, off by default, elastic model only (the rigid model has no tensions). Needs `effects.belt_mass`: the belt's mass is what slides.

**Climbing out.** At a driven pulley with the sections a and b on either side (tensions T_a, T_b), the teeth climb out of the grooves when

```text
|T_a − T_b| > c · max(0, min(T_a, T_b))        c = k · z_m · p / r
```

Each of the z_m teeth in mesh (`ratings.belt_teeth_in_mesh`) is held in its groove by the tension's radial pull over one pitch, T · p / r, and pushed out by its share of the effective pull times the tangent of an effective flank angle; k = 1 / that tangent is `belt.tooth_jump_factor` (an estimate, 2.4, bounds 1.2 to 4.8: flank, friction and the tooth's give together, not derived from the tooth profile). Summed over the teeth the pull the pulley takes is proportional to the slack side's tension, as the torque at which timing belts jump is reported to rise with their installation tension (no maker's chart was read for the factor). On a pulley drive with both sides at P ± F/2 the teeth jump at F = c P / (1 + c/2): with 10 of 25 teeth in mesh, c = 2.4 · 10 · 5 mm / 20 mm = 6 and F = 1.5 P, between the makers' rule of a pretension at least the effective pull (slack side at P/2) and the slack side going slack at 2 P. With the EXCT-30's 10 of 20 teeth c = 7.54 and F = 1.58 P.

**The jump.** Per driven pulley the elastic state carries six more entries (`JUMP_STATES`): the belt's slip over the pulley u (m of belt), its rate w, the direction of a jump under way (0 while in mesh), u where it began, the jumps counted and the ridge force F_s. The sections see the pulley angle plus u / r, so a slip of u feeds u · r_mat / r of material from one section into the other. While the teeth are out (decided after each base step, `Machine::after_step`):

```text
m · (r · d²a/dt² + d²u/dt²) = F − F_s · sin(2π d / p)     belt, F = belt torque / r its pull towards +u, d = u − u_start
J' · d²a/dt² = gear + r · F_s · sin(2π d / p) + mesh       pulley, J' = J_pulley − m r²
F_s = c · min(T_a, T_b) at the moment it climbed out, held over the jump
```

m is the belt's mass that the machine lumps on that pulley (single axis: the whole belt; T-gantry: half of it on each). The belt moves on its own and the pulley holds it back with a ridge force, the potential F_s p / (2π) (1 − cos(2π d / p)); with its kinetic energy ½ m ((v + w)² − v²) on top of what the pulley's lumped inertia counts (v = r da/dt), the energy is exact. Where the belt crosses the next groove (|d| = p) the teeth fall in: u is set to u_start ± p, the jump is counted (`pulley.<motor>.tooth_jumps`), and belt and pulley run on at the speed that keeps their momentum (a plastic impact). Where it slides back over the groove it left, it falls in there, uncounted. Where the pull settles below the ridge the belt stays half out, carrying the load through the ridge. What changes the energy at once (the start, the landing) is reported as `power.tooth_jump` over that base step, so the energy budget closes; a jump is found at the end of a step and lands up to one step late, so in a run that ratchets the budget's remainder grows (0.33 % at the base step, 0.21 % at half, 0.14 % at a quarter, on the single axis at 90 N of pretension; `jumping_teeth_fail_the_check_and_close_the_energy_budget`).

Signals: `pulley.<motor>.tooth_jumps` (count) and `pulley.<motor>.tooth_slip` (u, m), `power.tooth_jump` (W). Verdict `belt-tooth-jump`: the jumps during the measured pass, which must be 0.

Assumptions: one jump law per pulley, with the configured teeth in mesh or, with `effects.belt_wrap`, those of the wrap at the pose ([Belt wrap](#belt-wrap)); no radial motion of the belt, so the energy with which the tension presses a tooth back into its groove is not modelled (a tooth that climbs out and falls back takes nothing); the teeth themselves are not hurt (their load is `belt-tooth-load`).

## Belt wrap

Source: `wrap_angle` in [`mech.rs`](../crates/sim-components/src/mech.rs), `Machine::belt_wraps`; `effects.belt_wrap`, off by default (so that earlier results stay as they were). The teeth in mesh on a driven pulley are counted from the angle φ by which the belt wraps it at the pose, in place of `ratings.belt_teeth_in_mesh`:

```text
z_m  = φ / (2π) · 2π r / p = φ · r / p                    teeth in mesh
lean = atan2(h, d) − asin((r + r₂) / sqrt(d² + h²))       per span, to the next wheel at (d, h), radius r₂
φ    = π − lean_upper − lean_lower
```

r₂ is positive where pulley and wheel lie on either side of the belt, negative where on the same side, 0 for a clamp; a span that leaves level leans by 0. With it the tooth-jump law's c = k · z_m · p / r becomes **c = k · φ**, whatever the pitch and radius, and the belt's tooth load (`belt-tooth-load`) counts φ · r / p teeth (at most 12) at the smallest wrap of the measured pass.

Both machine crates give their geometry: the T-gantry its carriage idlers (`idler.radius_m`) beside the carriage at the heights ±(r + r_i), the single axis the clamp at the pulley's height and the idler as large as the pulley at the far end. In both the spans leave the motor pulleys level at every pose: **φ = π wherever the carriage is**, 12.6 teeth on the default configs' pulley (r = 20 mm, p = 5 mm, against the 10 configured), 10 of the EXCT-30's 20. The tooth-jump limit is then 2.4 π = 7.54 times the slack side, against 6 with the configured 10 teeth. Machine files give no geometry and keep the configured value.

Assumptions: the wheels stand where the scene draws them; the belt's sag and the teeth's give do not change the wrap.

A machine file works the wrap out for any layout (`geometry::wrap` in [`sim-netlist`](../crates/sim-netlist/src/geometry.rs)): each wheel has a radius signed by the way the belt runs round it (+ anticlockwise, − clockwise, 0 a clamp), the straight run from wheel a to wheel b leaves in the direction t = cos α e + sin α e⊥ with e the unit vector between the centres, D their distance and sin α = −(ρ_b − ρ_a) / D, and the wrap is how far t turns round the pulley, in its sense, from 0 to 2π. For the crates' layouts that is the φ above ([machine files](machine-files.md#the-belts-wrap-and-a-leaning-axis)).

## Sideways vibration of the belt spans

Source: `span_frequency`, `span_detuning`, `drive_excitations` in [`mech.rs`](../crates/sim-components/src/mech.rs); each machine's `span_signals`; `effects.belt_span_vibration`, off by default. Signals and a check only: nothing acts back on the motion.

Each free span between two of the points that guide the belt (pulleys, idlers, clamps) is a string at the live tension of its section:

```text
f1 = sqrt(T / μ) / (2 L)          μ = belt.mass_kg_m, L the span's length, T its section's tension; 0 when slack
```

The belt's own speed would lower it by 1 − v² / c² (c = sqrt(T / μ), 55 m/s at 300 N and 0.1 kg/m), below 0.2 % here, left out. The drives excite the belt at the tooth mesh |v_pulley| / p and, with `effects.torque_ripple`, at the motors' ripple and cogging orders times their turns per second. `belt.span_detuning` is the smallest min |f1 − f_e| / f1 over spans and excitations, capped at 1 (a span far from every excitation, a slack span or a machine at rest). Spans: single axis 3 (motor pulley to carriage, motor pulley to idler, idler to carriage), T-gantry 8 (clamp to carriage idler, carriage idler to motor A, motor A to the lower left carriage idler, down to the Z-bottom idler and up again, to motor B, motor B to the carriage idler, carriage idler to the clamp), their lengths those of the sections' geometry without the teeth in mesh.

Verdict `belt-span-resonance`: the lowest `belt.span_detuning` while the axes run at a steady speed (every acceleration setpoint 0, some speed setpoint not), sampled once per millisecond, must stay at or above `ratings.belt_span_resonance_band` (0.1). A sweep through a resonance while speeding up passes in moments; a cruise on one does not. On a linear axis the spans' lengths change during a cruise, so their frequencies sweep, and a sweep across an excitation in a cruise fails the check as well: it is slow, the ±10 % band of a 200 Hz span 0.5 m long takes 0.2 L / v = 0.1 s at 1 m/s, 20 periods, enough for a lightly damped span to ring up. The default T-gantry fails it so (detuning 5e-6 when its Z axis runs at 0.4 m/s: the tooth mesh at 0.4 m / 5 mm = 80 Hz meets spans 4 and 5, down to and up from the Z-bottom idler, whose frequency changes with z). Belt makers ask to keep a span's natural frequency away from the drive's excitations without a figure; 10 % is the separation usually kept from a resonance (an estimate).

Not built: the amplitude. A lumped first-mode oscillator driven by the tension's fluctuation would need a damping of the sideways motion that nobody publishes and a forcing term (tension times the span's curvature) that has no measurement to be checked against; the frequency and the check need neither. Only the first mode; its harmonics n f1 are not checked.

## Guide friction (LuGre)

Source: `Lugre` in [`mech.rs`](../crates/sim-components/src/mech.rs).

One internal state s per contact, the bristle deflection:

```text
g(v)   = λ · ( F_c + (F_s − F_c) · exp(−(v / v_s)²) )
ds/dt  = v − σ₀ · |v| · s / g(v)
F_f    = σ₀ · s + σ₁ · ds/dt + b · v
```

F_c `coulomb_n`, F_s `stiction_n`, v_s `stribeck_m_s`, σ₀ `bristle_stiffness_n_m`, σ₁ `bristle_damping_n_s_m`, b `viscous_n_s_m`. In steady sliding F_f = g(v)·sign(v) + b·v. A `stiction_n` below `coulomb_n` counts as `coulomb_n` (F_s = max(F_s, F_c)): breaking away never takes less than sliding. That matters where the uncertainty bounds of the two overlap, as for the X guide in `exct-30.toml`. With `effects.temperature`, F_c, F_s and b are multiplied by `temperature.friction_scale`.

**Load dependence** (`effects.load_dependent_friction`). The factor λ scales the Coulomb and stiction levels with the guide load:

```text
λ = (preload + |load|) / (preload + |nominal load|)
```

The nominal load is the load the levels were given for. What the load is, per machine:

| Guide | Load | Nominal load |
| --- | --- | --- |
| Single axis | m · g · cos(incline), m with the payload if attached | carriage alone |
| T-gantry X | (carriage + Z axis + payload) · g + \|tilt\| | (carriage + Z axis) · g |
| T-gantry Z | \|side\| + \|tilt\| | 0 |

The bristle state settles at the rate σ₀ |v| / (λ g(v)). Stiff bristles on a light contact at speed take that beyond what the integrator follows (1e6 N/m, 12 N, 2.5 m/s: 2.1e5 1/s, against 1.8e5 1/s for RK4 at the default step), and the friction force then grew without bound. Above 1e5 1/s the bristles therefore settle at 1e5 1/s, with their force scaled by 1e5 / rate so that sliding still takes the level. No shipped config reaches that with its own values; the EXCT-30's bounds do, with the X guide's Coulomb level low and its bristle stiffness high together.

with, on the T-gantry, side = m_z · a_x and tilt = 2 · side · (z_com_drop − z) / guide_spacing. The viscous term does not scale.

**Simplification.** a_x here is not the true acceleration. It is the X acceleration that the commanded motor torques would give a loss-free rigid machine: a_x = i (τ_A + τ_B) / r / (m_x + 2J/r²). The true acceleration would make friction depend on itself, and its absolute value would turn every belt vibration into extra friction. The same loads are what the signals `axis.<id>.guide_load` report.

## Motor shaft: friction, ripple, cogging

Source: `ShaftFriction`, `Ripple` in [`mech.rs`](../crates/sim-components/src/mech.rs).

```text
τ_shaft = T_c · tanh(ω / ω_s) + b · ω                                  friction opposing the speed
τ_out   = τ · (1 + ρ · sin(n_r · φ)) + T_cog · sin(n_c · φ)           torque at the shaft for a torque τ from the current
```

T_c `friction_coulomb_nm`, b `friction_viscous_nm_s_rad`, ω_s `friction_smoothing_rad_s`; ρ `ripple_fraction`, n_r `ripple_order`, T_cog `cogging_nm`, n_c `cogging_order`. The Coulomb step is smoothed with tanh so that a shaft at rest sees no chattering torque; this friction has no stiction. Ripple and cogging are one harmonic each.

## Tooth meshing

Source: `ToothMesh` in [`mech.rs`](../crates/sim-components/src/mech.rs).

A force along the belt at each driven pulley, periodic in the belt travel:

```text
F_mesh = F_t · sin(2π · travel / pitch)
```

F_t `belt.tooth_force_n`, pitch `belt.tooth_pitch_m`. It acts on the pulley as the torque r · F_mesh(r · angle).

## Gearbox

Source: `Gearbox` in [`mech.rs`](../crates/sim-components/src/mech.rs).

Between motor shaft (angle φ) and pulley (angle θ). Stiffness, damping and backlash are at the output side.

```text
twist    = φ / i − θ
engaged  = twist − clamp(twist, −β/2, +β/2)            twist beyond the dead zone; β = backlash_rad
τ        = k_g · engaged + c_g · (dφ/dt / i − dθ/dt)   while engaged ≠ 0, else 0
τ_out    = τ · η^tanh(τ · ω_out / 0.5 W)               on the pulley
τ_rotor  = τ / i                                       reaction on the motor shaft
```

- **Stiffness and backlash.** Inside the dead zone nothing is transmitted. With `effects.gearbox_backlash` off, β = 0.
- **Flanks cannot pull.** With backlash on, if the damping term would reverse the torque while the flanks separate (τ · engaged < 0), the torque is zero. With backlash off the connection is a plain spring-damper and can pull, as a coupling does.
- **Efficiency.** The loss is on the output torque. When the gearbox drives the load (power positive) the output gets τ · η; when the load drives back it gets τ / η. The exponent fades with tanh below 0.5 W of transmitted power, so a gearbox at rest has no torque step. With `effects.gearbox_efficiency` off, η = 1.
- **At rest** under an output torque the twist is τ / k_g + β/2 on the matching side.

Assumptions: one constant efficiency (the catalogue's full-load value; with `effects.thermal_feedback` it follows the gearbox's temperature, see [temperature feedback](#temperature-feedback)), no speed- or load-dependent loss, no no-load torque, constant stiffness, backlash as a hard dead zone. The gearbox's own inertia (`gearbox.inertia_at_input_kgm2`) turns with the rotor: J_rotor in the equations is `motor.rotor_inertia_kgm2` plus it.

## Hard stops

Source: `HardStop` in [`mech.rs`](../crates/sim-components/src/mech.rs).

For an axis whose stroke runs from 0 to S:

```text
beyond = pos − clamp(pos, 0, S)
F      = −k_s · beyond − c_s · v        only while F pushes back towards the stroke, else 0
```

A stop never holds on. With `effects.hard_stops` off there is none.

## Drive and motor

Source: [`drive.rs`](../crates/sim-components/src/drive.rs). One drive per motor; all have the same settings. The loops close on the motor's own encoder.

### Winding

The motor is one equivalent winding with resistance R, inductance L and the back-EMF of the turning rotor:

```text
L · di/dt = u − R · i − k_e · ω
τ         = k_t · i
```

k_e = k_t in SI units (`effects.back_emf` off: k_e = 0). Each base step the voltage u (and the resistance R, at the temperature the step starts with) is held, and the current is integrated together with the mechanics (see [integrator](#integrator)), with ω the motor's speed at each stage of the integration:

```text
di/dt = (u − R · i − k_e · ω) / L           in the integrated state, with the torque k_t · i(t) on the shaft
E_el  = ∫ u · i dt                           electrical energy of the step; P = E_el / dt, negative while the motor feeds back
E_cu  = ∫ R · i² dt,   W = ∫ τ · ω dt         copper loss and shaft work of the step
```

With the back-EMF on, u · i = R i² + d(½ L i²)/dt + τ · ω at every moment (also under saturation, see below), so the electrical energy, the copper loss, the winding's stored energy and the work on the shaft close to the integration's error (the [energy budget](#energy-budget) checks it). Before 12 November the current was stepped exactly over a base step with the speed of its start, its power taken by the trapezoid rule, and the mechanics turned under the torque of the current at the end of the step. That held torque was a numerical phase lead of half a step: on undamped elastic configs it put 3 to 5 % of the supply into the mechanics, and on damped ones it hid part of the ringing (peak motor torque 3 % lower on the default T-gantry, 8 % on the single axis; see [PROGRESS](../PROGRESS.md)).

Assumptions: no three phases, no d/q axes, no PWM, no inverter dead time, no field weakening, no iron loss. Below the current loop there is nothing.

### Torque saturation

`effects.torque_saturation`: the torque constant falls with the square of the current to `torque_constant_at_max_current_nm_a` at `max_current_a`:

```text
τ = k_t · i · (1 − s · min(1, (i / i_max)²)),      s = 1 − k_t,max / k_t
```

The current setpoint is the inverse of this curve, limited to `max_current_a` (Newton's method from below; exact to rounding). So the motor gives the torque that is commanded, up to its maximum torque at its maximum current. A curve that loses more than a third of its torque constant (s > 1/3) turns down before the maximum current; the setpoint then stops at the top of the curve, i_max / sqrt(3 s).

The back-EMF constant falls with the torque constant: e = k_e · ω · (1 − s · min(1, (i / i_max)²)). The electrical power u · i is then the copper loss plus the shaft power, with saturation as without.

### Winding heating

`effects.winding_heating`: one thermal mass. ϑ is the temperature rise of the winding above its surroundings.

```text
R(ϑ)     = R · (1 + 0.0039 · ϑ)                          copper
I_rated  = motor.rated_current_a
P_rated  = R(ϑ_rated) · I_rated²
dϑ/dt    = ( ϑ_rated · P_cu / P_rated − ϑ ) / τ_th            P_cu = E_cu / dt, the mean R(ϑ) · i² of the step
```

At the rated current the winding settles at `rated_temperature_rise_k`. The warmer resistance is used in the winding equation above. A disabled drive has no current, so its winding cools by the same equation with P_cu = 0: ϑ falls by the factor e every τ_th (`a_disabled_winding_cools_by_the_exponential`). Stepped once per base step (explicit Euler; with dt / τ_th of 3.5e-8 its error is far below anything that shows). The thermal time constant is half an hour, so a simulated cycle hardly warms the winding; the analysis therefore reports the rise the winding would *settle* at (see [analysis quantities](#analysis-quantities)).

Assumptions: only copper loss heats; the surroundings are at a fixed temperature; no housing, no second thermal mass (but see the three nodes below); the torque constant does not change with temperature.

### Motor thermal nodes

`effects.motor_thermal_nodes` (off in the shipped configs; read as off where a config has no switch): three nodes, the winding ϑ_w, the stator and housing ϑ_h, and the surroundings, as two heat capacities in a chain. The resistances add up to the one-node model's, R = ϑ_rated / P_rated, and the heat capacities too, C = τ_th / R; the winding has the share s = `motor.winding_resistance_share` of the resistance and the time constant τ_w = `motor.winding_time_constant_s` against the housing (both `[estimate]`: Siemens publishes one thermal time constant per motor, the catalogue's τ_th):

```text
R_w = s · R            C_w = τ_w / R_w
R_h = (1 − s) · R      C_h = C − C_w = (τ_th − τ_w / s) / R          τ_w < s · τ_th, or the config is refused
τ_w · dϑ_w/dt               = s · P_cu · R − (ϑ_w − ϑ_h)
(τ_th − τ_w / s) · dϑ_h/dt  = (ϑ_w − ϑ_h) / s − ϑ_h / (1 − s)
```

The steady state is ϑ_w = P · (R_w + R_h), the one-node model's, and ϑ_h = P · R_h: the settled rise the analysis reports and `motor-thermal` judges is the winding's and does not change; the analysis reports the housing's settled rise beside it (`MotorLoad::housing_rise`). On the way the winding warms much faster than the one node: with s = 0.25, τ_w = 60 s, τ_th = 1800 s it is at 16.2 K after 60 s (one node: 3.3 K) and 78.3 K after 1800 s (one node: 63.2 K) of a loss that settles at 100 K. A winding started warm (`start_temperature_rise_k`, the warm start) starts with its housing where that rise settles it. The one-node model stays the fallback with the switch off.

Test: `three_thermal_nodes_settle_at_the_sum_of_the_resistances_and_step_as_by_hand` (drive.rs; the step response by its two eigenvalues, −7.36e-4 and −1.935e-2 1/s, at 60 s and 1800 s, and the steady state 100 K / 75 K).

### Current loop

Every base step:

```text
i_meas   = i + noise
e_i      = i(τ_cmd) − i_meas                       i(τ): τ / k_t, or the inverse of the saturation curve
u_wanted = K_pi · e_i + I_i + e                    the drive adds the back-EMF e it expects
u        = clamp(u_wanted, −U_max, +U_max)
I_i     += (K_pi / T_ni) · e_i · dt                only while u is not clamped (anti-windup)
```

U_max is sqrt(3/2) times `drive.bus_voltage_v`, or sqrt(3/2) times the DC link voltage of the moment with `effects.dc_link`, or unlimited with `effects.voltage_limit` off. The factor: the one winding carries the RMS phase current I, and u · i is the power of all three phases, 3 · U_phase · I, so u is three times the RMS phase voltage. A DC link U gives a phase amplitude of U / sqrt(3), which is U / sqrt(6) RMS, and 3 / sqrt(6) = sqrt(3/2). `drive.bus_voltage_v` is the DC link voltage itself, and the DC link's energy bookkeeping below works with it unscaled. With K_pi = L / T and T_ni = L / R the current follows its setpoint with the time constant T (0.25 ms in the default configs).

### Velocity loop

Every velocity cycle T_v:

```text
ω_raw    = (encoder − encoder_before) / T_v        or the true speed with an ideal encoder
ω_f     += α_v · (ω_raw − ω_f)                     α_v = 1 − exp(−T_v / velocity_filter_s); 1 with the filter off
ω_cmd    = K_v · e_pos + w_v · (ω_set + α_set · t) speed setpoint interpolated between profile cycles
e_ω      = ω_cmd − ω_f
τ_wanted = K_p · e_ω + I_ω + w_τ · τ_ff
τ_filt  += α_τ · (τ_wanted − τ_filt)               torque setpoint filter
τ_notch  = notch(τ_filt)                           with effects.notch_filter
τ_cmd    = clamp(τ_notch, torque range at ω_f)
I_ω     += (K_p / T_n) · e_ω · T_v                 unless that pushes further into the limit (anti-windup)
```

### Drive current limit

`effects.drive_current_limit` (on in the shipped configs): the drive gives no more than its maximum output current `ratings.drive_max_current_a` (a drive by name sets it). The velocity loop's torque range above is narrowed to the torque of that current, where that is below the motor's:

```text
torque range = [low, high] ∩ [−τ(I_drive), +τ(I_drive)]      τ(i) = k_t · i · (1 − s · min(1, (i / i_max)²)), the saturation curve
```

So the current setpoint i(τ_cmd) stays within min(motor, drive maximum current), and the velocity integrator stops while the setpoint sits on the limit, the same anti-windup as at the motor's limit; the same with `effects.torque_limit` off. A cycle that asks more than the drive gives then shows it: the axis accelerates at what the drive's current gives, a = (k_t · I_drive · i / r − F_friction) / m_total, falls behind its plan and catches up later. `drive-peak-current` judges the current the velocity loop asks for (i(τ_demand), at most the motor's maximum current under saturation), so a cycle held at the drive's limit fails it, as one held at the motor's limit fails `motor-peak`. The demand that a following error built up may then fail `motor-peak` too. Without the rating, or with the effect off, nothing is held and the drive's maximum current is only judged. `check_limits` (the warning that the axis limits ask more of a motor than it has) takes the torque of that current too, where it is below the curve.

Tests: `the_drive_current_limit_holds_the_torque_to_the_lower_of_motor_and_drive` (drive.rs: 5 A at 0.45 N m/A holds 3.8 N m to 2.25 N m; 10.9 A leaves the motor's 3.8), `a_move_held_at_the_drive_current_limit_accelerates_at_the_hand_calculated_rate` (single axis, 0.5 A: a = (0.225 · 250 − 15) / 6.125 = 6.735 m/s²; with the motor at 0.18 N m, below the drive, 4.898 m/s²; the shipped cycle at 70 % of its peak current fails `drive-peak-current` and lags further).

### Drive overload

`effects.drive_overload` (off in the shipped configs; read as off where a config has no switch): SINAMICS drives give their maximum current only for a short time and then fall back to their base load current. The drive keeps a thermal image of its power unit (i²t), stepped exactly every base step with the RMS phase current i (with field weakening the hypotenuse of q and d current):

```text
x' = ((i / I_rated)² − x) / τ_drive            x = 1 at the rated current held
trip = r² − (r² − 1) · exp(−t_max / τ_drive)   r = I_max / I_rated: the maximum current from the rated load for t_max
x ≥ trip  → limit = I_base, until x ≤ 1 again
```

I_rated, I_max and I_base are `ratings.drive_rated_current_a`, `drive_max_current_a` and `drive_base_load_current_a` (a drive by name sets them: an S120 Motor Module's base load current is the catalogue's, about 0.85 of its rated current; Siemens publishes none for the S210, so 0.85 of its rated current is taken, `[estimate]`); τ_drive = `drive_overload_time_constant_s` and t_max = `drive_max_current_time_s` are `[estimate]`s (10 s and 0.25 s in the shipped configs: the Siemens load duty cycles are not in the catalogue files). While tripped, the velocity loop's torque range is narrowed to the torque of I_base, as for the drive current limit above.

`drive-overload` judges the image's peak when the cycle repeats for long against the trip level. The image is linear in its start: one started at x₀ lies (x₀ − x_start) · exp(−t / τ) above the measured pass's at every t, and the start a repeating pass returns to is x₀ = f / (1 − exp(−T / τ)), with f = x_end − x_start · exp(−T / τ) what the pass added (`analysis::repeating_peak`; sampled once per millisecond). Where the image tripped in the pass the drive was not linear any more; the check fails there anyway.

Tests: `a_step_at_twice_the_rated_current_clamps_after_the_hand_calculated_time` (drive.rs: 3 A rated, 2.6 A base load, 9 A for 0.25 s, τ = 10 s: trip 1.19752; 6 A from cold clamps to 2.6 A after −10 s · ln(1 − 1.19752 / 4) = 3.558 s, and the image is back at 1 after another 5.842 s), `the_overload_image_of_a_repeated_cycle_peaks_at_the_hand_calculated_level` (1.5 I_rated held: 2.25; 2 I_rated for 0.5 s of 1 s: 2.04999).

w_v and w_τ are the feedforward factors (0 with the switch off). The notch is a biquad with centre `notch_frequency_hz` and Q = frequency / bandwidth.

**Margins.** `Drive::torque_injection` is a test torque d added to τ_notch before the limit; a drive's own measurement function works the same way. With the machine holding its start pose, a sine d on one motor gives the loop's answer u = τ_notch and the torque that acts e = u + d, both correlated with sine and cosine at the velocity cycle over whole periods (at least 0.1 s to settle, 0.1 s to measure). Then L = −U / E is the open-loop response at that motor, with everything in it: velocity filter, PI, torque filter, delays, current loop, the elastic mechanics (the other motors' loops closed) and the position loop around it; and |S| = |E| / |D| = 1 / |1 + L|. `analysis::loop_margins` follows the phase of L along the frequency axis without jumps and shifts it by whole turns so that it lies between −360° and 0° at the first gain crossover (without a crossover: between −270° and 90° at the lowest point). It reads off, interpolated in log |L| and phase between points, the phase margin 180° + ∠L at each gain crossover (the smallest counts; negative: unstable) and the peak of |S|. Stability is a Nyquist count of the crossings of the negative real axis (the phase through −180° or an odd multiple) beyond −1, |L| > 1: a falling one is an encirclement of −1, a later rising one undoes the last falling one not yet undone. The count starts at a phase of 0 at the gain of the lowest point, as the small circle of the Nyquist contour around s = 0 turns the phase by −90° per integrator, so a loop whose integrators lag past −180° below the measured range counts that crossing too. The loop is stable when no falling crossing is left. The gain margin is −20 log |L| at the worst falling crossing not undone: those with |L| < 1 (in practice above the crossover) give a positive margin, one left with |L| > 1 a negative one (L encircles −1, unstable). A falling crossing beyond −1 that a rising one undoes (the phase turned past −180° and back below the crossover) is the conditionally stable kind: a gain that falls would be the danger there, and the margin for that is not given. A point at which the torque limit was hit, or at which the loop moved on its own (what of e is neither the injected sine nor a constant is larger in RMS than the injection), is no measurement of L: it is NaN, the margins are not valid and NaN (`LoopMargins::valid`, `stable`); `sim-cli margins` says `FAIL` for an invalid or unstable loop. Tests: `margins_are_signed_and_say_an_unstable_loop_is_unstable` (L = ωc e^(−jωτ) / (jω): 100 Hz, 0.5 ms: 72°, 13.979 dB; 200 Hz, 0.5 ms: 54°, 7.959 dB; 300 Hz, 1 ms: −18°, −1.584 dB; 400 Hz, 1 ms: −54°, −4.082 dB), `a_conditionally_stable_loop_is_stable` (L = K (1 + s/a)² / (s (1 + s/b)² (1 + s/c)), a, b, c = 2π · 300, 30, 3000 Hz: stable at K = 3e5, 6.1e5, 1.2e6 with 30.7°, 41.6°, 42.4°), `a_lead_with_positive_phase_at_the_first_point_keeps_its_phase_margin` (a type-0 lead loop: 24.0° at 1516 Hz, not −336°), analysis; `an_unstable_velocity_loop_is_reported_unstable` (single axis at 0.3 N m s/rad), axis. `velocity_loop_margins` does this for every motor at 60 points from 10 Hz to a quarter of the velocity loop's rate (2 kHz), with a tenth of the rated torque, for the config and for each value of `[bounds]` at its low and at its high bound (`sim-cli margins MACHINE [--bands]`). The velocity gains of all three configs were chosen with it (see the [user guide](user-guide.md#tune-gains-and-use-input-shaping)). Friction near standstill damps the loop at this amplitude (the motor's Coulomb friction is smoothed over 0.5 rad/s), so the margins are lower with friction off, as in a cruise.

### Position loop

Every position cycle:

```text
e_pos = φ_set − φ_measured
K_v · e_pos is held until the next position cycle
```

### Torque-speed curve

```text
fade     = clamp( (ω₀ − |ω|) / (ω₀ − ω_rated), 0, 1 )
motoring : |τ| ≤ peak_torque · fade        in the direction of rotation
braking  : |τ| ≤ peak_torque
```

ω₀ `no_load_speed_rad_s`. Full torque up to the rated speed, falling linearly to zero at the no-load speed. With `effects.torque_limit` off the torque setpoint is not limited; the analysis still judges against the curve. What it judges is the velocity loop's output before this limit (`torque_demand`), so a drive that sits on the limit shows as a utilisation above 1. This curve is a separate, configured limit; the voltage limit of the winding acts as well. With field weakening the curve follows from the d-q model instead (below).

### Field weakening

`effects.field_weakening` (off in the shipped configs; read as off where a config has no switch): the permanent-magnet synchronous motor in d-q axes. The one winding above is the q axis, scaled as it is (current the RMS phase current, voltage three times the RMS phase voltage, so R, L and U_max are the winding's: three times the data sheet's phase values); the d axis is scaled alike. The flux linkage is ψ = k_e / p, from the torque constant; p = `motor.pole_pairs`, L_q = `motor.inductance_h`, L_d = `motor.d_inductance_h`. With the electrical speed ω_e = p · ω:

```text
u_q = R · i_q + L_q · di_q/dt + k_e · ω + ω_e · L_d · i_d
u_d = R · i_d − ω_e · L_q · i_q
u_q² + u_d² ≤ U_max²                    voltage circle (the DC link through the modulation, as above)
i_q² + i_d² ≤ I_max²                    current circle (the motor's maximum current)
τ = k_t · i_q + p · (L_d − L_q) · i_d · i_q
```

Above base speed the drive sets a negative d current: the least that brings the steady voltage within the circle for the q setpoint, from (c₀ + a · i_d)² + (R · i_d − b · i_q)² = U_max² with c₀ = R · i_q + k_e · ω, a = ω_e · L_d, b = ω_e · L_q (`DriveParams::d_current_for`). The q axis's current controller adds ω_e · L_d · i_d to the back-EMF it expects and gets what u_d leaves of the voltage, sqrt(U_max² − u_d²). The d current is held over the base step like the voltage: u_d · i_d joins the electrical power, R · i_d² the copper loss (and so the heating), the reluctance torque the shaft, so u_q · i_q + u_d · i_d = R (i_q² + i_d²) + L_q · i_q · di_q/dt + τ · ω still closes. The drive's thermal image, the RMS current of the analysis and `drive-current` see the phase current sqrt(i_q² + i_d²).

The torque-speed curve is then the most torque within both circles at each speed, over d currents from 0 to −I_max (golden section), up to `drive.peak_torque_nm`, motoring and braking apart; `motor-peak`, the report's curve and the app's plot use it. Towards high speed the best d current tends to the characteristic current −ψ / L_d.

Catalogue: Siemens publishes R, L, k_E and the pole pairs of the 1FK7 and 1FT7 (`motors_s120_electrical.csv`, `motors_s120.csv`); a motor by name sets `pole_pairs` and takes L_d = L_q (surface magnets, `[estimate]`). For the 1FK2 and 1FT2 none of these is published; the config's `[estimate]`s stay.

Tests: `field_weakening_gives_the_hand_calculated_torque_above_the_voltage_limit` (drive.rs: 0.45 N m/A, 190 V, 9.5 A, 4 pole pairs, 10 mH, R → 0, at 500 rad/s: the circles cross at i_d = −5.625 A, i_q = 7.6557 A, 3.4451 N m, where the fading curve has none; the current loop settles there with |u| = 190 V and the power balance closes), `the_d_q_model_holds_the_published_points_of_a_1fk7` (1FK7042-2AF7 at 600 V: the rated 2.6 N m at 2.0 A and 314.16 rad/s within 6 % and inside the curve without d current; torque left at n_max,inv = 670.2 rad/s, where the fading curve ends at 538.9 rad/s; the best d current at 942.5 rad/s within 1 % of −ψ / L_d = −3.247 A). Siemens' torque-speed curves themselves are printed only as graphs and are not in the catalogue files, so the model is not checked against curve points.

Assumptions: the d current follows its setpoint at once (no d-axis winding state); the back-EMF in the d-current choice and the curve is unsaturated; the curve takes the static voltage (the bus voltage), the current loop the DC link's voltage of the moment; no MTPA below base speed for L_d < L_q beyond what the curve's search finds; `check_limits` judges the axis limits against the maximum speed and the curve at that speed, as a warning only (see [the axis-limit check](#axis-limits-and-the-motors)).

### Axis limits and the motors

`check_limits` (`Sim::limit_violations`, the warning at start-up) judges each motor by its own drive and motor (`[drive.<id>]`, `[motor.<id>]`, `[parts.<id>]`): the motor speed with every axis at its highest speed, in every combination of directions, at the middle pose, against its maximum speed (`drive.max_speed_rad_s`), and the rigid torque with every axis at its highest acceleration against its torque-speed curve at that speed (and the torque of the drive's maximum current, where lower). It is a warning, not a verdict: it takes every axis at its limits at once, which a cycle need not ask. The catalogue search and the optimiser used it as a screen before 14 November, against the shared rated speed and peak torque, and so ruled out motors that passed every verdict (on EXCT-30 at 400 V 32 of 84, the 1FT2205-2AC among them, which turns 157 of its 209 rad/s rated). Now they simulate every candidate, and the verdicts judge what the cycle asks: `motor-peak` against the curve, `motor-speed` against the maximum speed.

### Measurement effects

| Effect | Model |
| --- | --- |
| `encoder_resolution` | The position is rounded to whole counts of 2π / `encoder_counts_per_rev`. The speed is the difference of two positions per velocity cycle |
| `sensor_noise` | Gaussian noise (twelve uniform numbers summed) on the position, `encoder_noise_counts` counts standard deviation, before rounding; and on the measured current, `current_noise_a`. Pseudo-random from `noise_seed`, one stream per motor, so runs repeat |
| `velocity_filter`, `torque_filter` | First-order low-passes, computed once per velocity cycle |
| `drive_delay` | The voltage acts `current_delay_cycles` current cycles after it was computed, the torque setpoint `velocity_delay_cycles` velocity cycles after. Whole cycles |

### DC link and braking chopper

`effects.dc_link`: one DC link per drive, built from `[ratings]`.

```text
E      = ½ · C · U² − P · dt                            the capacitors give what the motor draws and take what it feeds back
fed    = clamp( E − ½ · C · U_bus², 0, P_line · dt )    with effects.regenerative_supply: the line module feeds back
burned = clamp( E − fed − ½ · C · U_th², 0, P_peak · dt )   the chopper switches the resistor in above the threshold
U      = sqrt( 2 · (E − fed − burned) / C )             but never below the bus voltage: the rectifier feeds from the line
drawn  = ½ · C · U_bus² − (E − fed − burned)            what the rectifier draws from the line, where that is positive
trip   : U > dc_link_overvoltage_v
```

Stepping the energy instead of the voltage is exact for any step size. P is the step's electrical energy E_el over dt (see [winding](#winding)). When a drive trips, all drives go off as with a Disable command, and stay off until Enable. Without the effect the DC link voltage stays at `drive.bus_voltage_v`.

**Regenerative supply.** `effects.regenerative_supply` (with `effects.dc_link`, off by default): the line module, an Active Line Module as on a SINAMICS S120, holds the DC link at the bus level from above as the rectifier does from below. What the motors feed back goes into the mains up to `ratings.line_module_max_power_w`; of it `line_module_efficiency` arrives there (`Drive::mains_power`) and the rest is the line module's loss (`Drive::line_loss`). Only what is beyond its power charges the capacitors and reaches the chopper. The shipped values are the smallest S120 Active Line Module of the catalogue (16 kW rated, 35 kW maximum) and an efficiency of 0.98 from its 0.28 kW loss at rated power. Hand calculation: `a_regenerative_supply_feeds_back_into_the_mains_up_to_its_power` (41 W fed back: 4.018 J into the mains and 0.082 J lost in 0.1 s at 190 V; limited to 20 W, the other 21 W charge 470 µF to 212.217 V); on every config the energy fed back is the efficiency times what the motors regenerated (`the_regenerative_supply_feeds_back_what_the_motors_regenerate`).

**Disabled.** A disabled drive's DC link discharges through its bleeder and the drive's own consumption, C dU/dt = −U / R_b: every step U ← U · exp(−dt / τ) with τ = `ratings.dc_link_discharge_time_constant_s`, exact for any step, then the step above with P = 0, so the rectifier catches it at the bus level and a regenerative supply takes what is above at once (`a_disabled_drives_dc_link_discharges_by_the_rc_exponential`: 220 V with τ = 0.2 s are 171.336 V after 0.05 s and the bus level after 69.9 ms).

Assumptions: the rectifier is an ideal source at the bus voltage; no ripple; the chopper is an ideal power limiter up to `braking_resistor_peak_w`, not a switched resistor; the resistor itself does not heat (its continuous rating is judged against the cycle's mean power); the line module is an ideal power limiter with one efficiency at every power and feeds in without loss (its efficiency counts only on the way back); the bleeder draws only while the drive is disabled (enabled, its power, U² / R_b ≈ 0.1 W at 155 V, is left out).

**Shared DC link.** `effects.shared_dc_link` (with `effects.dc_link`): the drives of a machine have one DC link, as with one Line Module on a SINAMICS S120. After the current loops of all drives, one step of the same energy balance runs with the sum of their powers:

```text
E = Σ ½ C U_k² − Σ P_k · dt        C_link = n · C
```

then chopper, rectifier and trip as above, once, with one resistor of `braking_resistor_peak_w`. Every drive then has the link's voltage and reports the resistor's power and the trip as its own, so the braking checks show the same shared values for each motor; `dc-link-energy` compares one drive's share of the stored energy with one drive's share of the room, which is the same ratio as for the whole link. What a braking motor feeds back, a motoring one draws: only the net power charges the capacitors. Switched on while the machine runs, the sum above puts the energies of links at different voltages together (the voltages step to the common one; no energy appears or is lost). Assumptions beyond those above: the capacitance is the sum of the drives' (a Line Module's own is not counted), and the busbars have no resistance.

### Start, disable, enable

A drive starts at rest already delivering the torque that holds the load, so a hanging axis does not sag. Disabled drives give zero torque, current and voltage; their windings cool (with `effects.winding_heating`) and their DC links discharge (with `effects.dc_link`). The current in a winding when the pulses stop freewheels through the inverter's diodes into the DC link within microseconds, so in the first disabled step its energy ½ L i² is fed back (the drive's power for that step is −½ L i² / dt; without the DC link simulated it goes back to the supply). Enable restarts the loops holding where the motors stand; a winding keeps the temperature it has and a DC link the charge it has, less the ½ L i² of the current that holds the load (below the bus level the rectifier makes that up at the next step). A drive that tripped and is enabled again while its link is still above the trip level trips again unless the chopper brings it below within the step. Test: `disable_and_enable_conserve_the_electrical_energy` (windings plus capacitors across Disable and Enable, to 1e-9 relative).

## T-gantry

Source: [`machine-t-gantry/src/lib.rs`](../crates/machine-t-gantry/src/lib.rs).

Two fixed motors A (left) and B (right) at the ends of the X beam drive one belt. The belt ends are clamped at the top of the Z axis, and it runs around an idler at the bottom of the Z axis:

```text
clamp → carriage idler → motor A → carriage idler → Z-bottom idler → carriage idler → motor B → carriage idler → clamp
```

### Kinematics

With pulley angles a and b after the gearbox:

```text
x = r/2 · (a + b)          z = r/2 · (a − b)
φ_A = i/r · (x + z)        φ_B = i/r · (x − z)         motor shaft angles
```

Both motors the same way moves X, against each other moves Z. With a stretching belt this holds only on average: motor angles, pulley angles and load positions are separate states.

### The three belt sections

The two motor pulleys cut the belt into three sections of uniform tension. With s = `span_min_m`, S_x and S_z the strokes:

| Section | From → to | Geometric length | Belt fed in |
| --- | --- | --- | --- |
| 1 | clamp → motor A | 2s + x + z | + r_mat · a |
| 2 | motor A → motor B | 4s + S_x + 2 S_z − 2z | − r_mat · a + r_mat · b |
| 3 | motor B → clamp | 2s + S_x − x + z | − r_mat · b |

The total length does not depend on the pose. With tooth stiffness, sections 1 and 3 get one tooth length and section 2 two.

```text
F_x = T₃ − T₁                          belt force on the carriage
F_z = 2 T₂ − T₁ − T₃                   belt force on the Z axis
τ_A = r_mat · (M₁ − M₂)                belt torque on pulley A (M: material force)
τ_B = r_mat · (M₂ − M₃)                belt torque on pulley B
```

Belt mass: half of the belt is lumped on each motor pulley, J_pulley += ½ · ρ · L_total · r².

### States of the elastic model

21 states: positions x, z, a, b, φ_A, φ_B, q, u; their eight rates; the two LuGre states of the guides; the slack-time counter; the two brake states.

- x is the carriage relative to the frame. q is the frame deflection in X.
- u is the Z axis's centre of mass in X relative to the carriage.

### Equations of motion

Masses: m_z = Z axis + payload (if attached); m_c = carriage + idler mass on X + moving chain mass.

Carriage, frame and leaning Z axis:

```text
f_x   = F_x − F_f,x − F_idler,x − F_chain drag + F_stop,x
a_q   = ( −k_f · q − c_f · dq/dt − f_x − ½ · m′ · w² ) / m_f              frame; w = dx/dt + dq/dt
spring = k(z) · u + c_u · du/dt
d²x/dt² = ( f_x + chain_x + (1 − tip) · F_p,x + spring ) / m_c − a_q
d²u/dt² = ( tip · F_p,x − spring ) / m_z − ( d²x/dt² + a_q )
```

Without `effects.tool_flex`: d²x/dt² = (f_x + chain_x + F_p,x) / (m_c + m_z) − a_q, and u stays 0. Without `effects.frame_compliance`: q stays 0 and a_q = 0.

Z axis:

```text
d²z/dt² = ( F_z − F_f,z + f_z,flex − F_idler,z + F_p,z + F_stop,z ) / (m_z + m_idler,z)  −  g · m_z / (m_z + m_idler,z)
```

Pulleys and motor shafts, for k = A, B:

```text
J_pulley · d²θ_k/dt² = τ_gear,out + τ_belt,k + r · F_mesh(r · θ_k)
J_rotor  · d²φ_k/dt² = ripple(τ_motor,k, φ_k) − τ_gear,rotor − τ_shaft(dφ_k/dt) − τ_brake,k
```

### Frame mode

`effects.frame_compliance`: the beam and its mounting are one mass `frame.mass_kg` on a spring `frame.stiffness_n_m` with damping, in X only. Belt, guide, idlers, chain and end stop push back on the beam with the force they put on the carriage. This is the only structural mode of the frame.

### Leaning Z axis

`effects.tool_flex`: the guide of the Z axis gives way sideways and in tilt. A force at the distance h from the carriage tilts with the arm h, so the compliance at the centre of mass is the sum of both:

```text
h      = z_com_drop_m − z                 centre of mass below the carriage
h_tool = z_tool_drop_m − z                tool tip below the carriage
k(z)   = 1 / ( 1/k_lat + h²/k_tilt )
tip    = k · ( 1/k_lat + h · h_tool / k_tilt )           tool-tip travel per u
f_z,flex = −½ · u² · dk/dz,     dk/dz = 2 · k² · h / k_tilt
```

The axis is softer the further it hangs below the carriage. The spring gets stiffer as the Z axis rises, which pushes back on Z (the last line; it keeps the energy balance exact). **The X position the simulator reports is the tool tip over the floor: x + q + tip · u**, and its speed likewise. The workpiece stands on the floor, not on the frame, so the frame's sway is part of path error, tracking error and the settling verdict. q and tip · u are also signals of their own. The process force acts at the tool tip: the share `tip` of it bends the Z axis, the rest pushes the carriage.

A machine file's `machine.lean` node is the same for any body leaning along a horizontal axis, its lean w beside the axes with the body's mass coupling it to its axis, [[M, m e_a], [m e_a^T, m]] · [a; ẅ] = [F; tip · F_p,a − k w − c ẇ] ([machine files](machine-files.md#the-belts-wrap-and-a-leaning-axis)); the T-gantry's file gives this crate's numbers to 1e-9.

Assumptions: the Z axis is a rigid body on a compliant guide; it does not bend. Only its lean in X is a degree of freedom, not in Y and not in rotation as a state of its own. The payload sits at the centre of mass used for h.

The modes the machine reports (`modes`; the lower one is `resonance_hz`, the input shaper's) are those of two masses in a row: the carriage on the belt's X stiffness k_b = EA/L₁ + EA/L₃ (pulleys taken as held), and the Z axis on k(z):

```text
ω⁴ − ( k_b/m_c + k/m_c + k/m_z ) · ω² + k_b · k / (m_c · m_z) = 0
v = (k − ω² m_z, ω² m_z)                                    shape in (x, u)
ζ = (c_b v_x² + c_z v_u²) / (2 ω (m_c v_x² + m_z (v_x + v_u)²))
```

Both roots: in the lower mode carriage and Z axis swing together (24.6 Hz at x = 0.9 m, z = 0.05 m), in the upper one against each other (104.2 Hz). c_b is the damping of the two sections (2 · `belt.damping_n_s_m`, or with material damping `belt.damping_time_s` · k_b), c_z `tool_flex.damping_n_s_m`; the cross terms of a damping that is not proportional are left out. The frame mode and the stiffness of the control loops are not in it.

The single axis reports one mode, its carriage on the belt with the pulley held: f = sqrt(k / m) / (2π), k = EA/L₁ + EA/L₂, m the carriage with payload and cable chain, ζ = c / (2 m ω) (65.8 Hz at x = 0.5 m). None in the rigid model.

### Idlers

`effects.idlers`: the four carriage idlers and the one at the bottom of the Z axis, all alike, turn with the belt that passes them. Belt passes the two upper carriage idlers at v_z, the lower carriage idlers at 2v_x + v_z (left) and 2v_x − v_z (right), and the Z-bottom idler at 2v_x. With m = J_idler / r_idler² per idler that is

```text
½ · m · ( 2 v_z² + (2v_x + v_z)² + (2v_x − v_z)² + (2v_x)² ) = ½ · (12 m) · v_x² + ½ · (4 m) · v_z²
```

so 12 idler masses on X and 4 on Z. They add inertia in Z but no weight. Bearing friction: each idler holds back the belt that passes it with the force `friction_nm` / `radius_m`, smoothed with tanh around rest:

```text
F_idler,x = 2 d · ( sg(2v_x + v_z) + sg(2v_x − v_z) + sg(2v_x) )
F_idler,z = d · ( 2 sg(v_z) + sg(2v_x + v_z) − sg(2v_x − v_z) )
```

### Cable chain

`effects.cable_chain`: the chain is fixed at the x = 0 end, so half of every metre the carriage travels joins the run that moves with it.

```text
m_chain(x) = mass_kg_m · ( length_at_zero_m + x/2 ),      m′ = mass_kg_m / 2
drag       = drag_n · tanh( v_x / friction_x.stribeck_m_s )
```

The mass grows as the carriage moves, so the equation of motion is d/dt(m · w) − ½ · m′ · w² = force. In the carriage equation that appears as chain_x = m′ · w · (w/2 − v_x), and the frame feels −½ · m′ · w².

### Process force

`effects.process_force`: the force (`force_x_n`, `force_z_n`) acts on the tool tip while the cycle dwells at a target, and not otherwise. It is constant.

### Hard stops

One per axis, at 0 and at the stroke, as [above](#hard-stops).

### Holding brakes

`effects.holding_brake`: while the drives are disabled, a brake on each motor shaft holds up to `motor.brake_torque_nm` and slips with that torque beyond. Each brake has one state, its deflection s:

```text
ds/dt   = ω − (s − clamp(s, −s_max, +s_max)) / T_slip        s_max = 1 mrad, T_slip = 50 µs
τ_brake = clamp( (τ_max / s_max) · s + c · ds/dt, −τ_max, +τ_max )
```

Held, the shaft sits on a spring that reaches the brake torque at 1 mrad, with the damping c critical for the rotor on that stiffness. The torque never exceeds the brake torque: beyond it the brake slips, and what s is deflected beyond 1 mrad is forgotten within 50 µs. A brake that engages at speed therefore decelerates the rotor with brake torque over inertia until it stands, at any speed. (The brake was a LuGre contact before; its bristle damping gave many times the brake torque when it engaged at speed, and above 178 rad/s its state was too stiff for the integration step.) A released brake forgets its deflection within 1 ms. Brakes exist in the elastic and in the rigid model, and on both motors (there is no separate "brake on Z"; holding Z takes both).

### Rigid reference model

`effects.belt_stretch = false`. Eight states: x, z, their rates, the two LuGre states, the two brake deflections. With belt force F_k = i · (ripple(τ_k) − τ_shaft − τ_brake,k) / r per motor (τ_brake,k the [holding brake](#holding-brakes) at the rotor speed i (v_x ± v_z) / r), reflected rotor mass m_r = i² J_rotor / r² and the gearbox loss factors λ_A, λ_B:

```text
( m_X + (λ_A + λ_B) m_r ) · a_x + (λ_A − λ_B) m_r · a_z = λ_A F_A + λ_B F_B + mesh − F_f,x + extras_x
(λ_A − λ_B) m_r · a_x + ( m_Z + (λ_A + λ_B) m_r ) · a_z = λ_A F_A − λ_B F_B + mesh − F_f,z − m_z g + extras_z
```

m_X and m_Z include both pulleys (2 J_pulley / r²), the idlers and the chain. The gearbox loses torque only on what it transmits, which is the motor torque minus what accelerates the rotor itself; that depends on the acceleration being solved for, and three passes settle it. With a loss-free gearbox this collapses to two independent axes:

```text
( m_x + 2J/r² ) · a_x = F_x − F_f,x          ( m_z + 2J/r² ) · a_z = F_z − m_z g − F_f,z
```

The rigid model has no belt stretch, slack, damping, tooth stiffness, backlash, gear stiffness, frame mode or leaning Z axis. Its brakes hold the whole machine on their spring (the damping is critical for the rotor alone, so held it rings a little before it stands). It reports the pretension as belt tension and twice the pretension as radial load.

### Feedforward torque and static hold

The torque feedforward and the motor-limit check use the loss-free rigid model:

```text
f_x = ( m_x + 2J/r² + m_idler,x + m_chain(x) ) · a_x
f_z = ( m_z + 2J/r² + m_idler,z ) · a_z + m_z g
τ_A = r/(2i) · (f_x + f_z)         τ_B = r/(2i) · (f_x − f_z)
```

Friction is not in it. At rest each motor holds m_z g r / (2i), with opposite signs.

### Signals

Besides the signals every machine gets: `belt.1/2/3.tension`, `axis.x/z.belt_stretch` (tool position minus r/2 (a ± b)), `axis.x/z.friction`, `frame.deflection` (q), `axis.x.tool_deflection` (tip · u), `belt.slack_time`, `pulley.a/b.radial_load`, `axis.x/z.guide_load`, `gearbox.a/b.output_torque`. And those of the [effect views](#effect-views): torque parts, loss powers, energy parts, the friction split per guide and `belt.1/2/3.stretch`; the simulation adds `temperature.gearbox_a/b` and `temperature.guide_x/z`.

The gearbox output torque is the torque that turns the pulley, positive forwards. Elastic model: τ_gear,out, what the gear's spring and damper pass on, times the loss factor. Rigid model: λ_k · (F_drive,k − m_r · (a_x ± a_z)) · r from the solver's last pass, the motor's force less what its own rotor takes, times the loss factor.

**Scene.** `Machine::scene_point` maps a tool pose to the place where the scene draws the tool, so that a front end can draw a path there: (span + x, −0.05, z − stroke_z − span − 0.042), the flange under the Z axis. The scene itself draws the flange at the carriage's place on the beam, so at rest the two coincide; in motion the tool tip is off the drawn flange by the frame's deflection and the lean of the Z axis.

The belt wraps each motor pulley by half a turn, so the two sections that end there pull its shaft the same way: radial load A = T₁ + T₂, B = T₂ + T₃. The guide loads are those of the [friction model](#guide-friction-lugre): weight, side force and tilting moment from the commanded acceleration, without moments on single blocks.

## Single axis

Source: [`machine-single-axis/src/lib.rs`](../crates/machine-single-axis/src/lib.rs).

A motor at the left end drives a carriage through a belt loop around an idler at the right end. x = r · a. The axis can be inclined (`geometry.incline_deg`).

| Section | From → to | Geometric length | Belt fed in |
| --- | --- | --- | --- |
| 1 | pulley → carriage, straight | s + x | + r_mat · a |
| 2 | pulley → idler → carriage | 3s + 2S − x | − r_mat · a |

Elastic model, 9 states (x, a, φ, their rates, the LuGre state, the slack-time counter, the deflection of the holding brake):

```text
m · d²x/dt²        = T₂ − T₁ − F_f + F_stop + F_extra − m · g · sin(incline)
J_pulley · d²a/dt² = τ_gear,out + r_mat · (M₁ − M₂) + r · F_mesh(r · a)
J_rotor · d²φ/dt²  = ripple(τ_motor, φ) − τ_gear,rotor − τ_shaft(dφ/dt) − τ_brake
F_extra            = F_process (while dwelling) − drag − ½ · m′ · v²
```

m is the carriage plus the payload if attached, plus the moving run of the [cable chain](#cable-chain) m_chain(x) with `effects.cable_chain`; drag and m′ are the chain's, with the Stribeck speed of `friction_x`. The chain's run has weight along an inclined axis like the carriage (lifting it from 0 to x takes mass_kg_m · (length_at_zero_m · x + x²/4) · g · sin(incline)), and does not load the guide. F_process is `process.force_x_n` with `effects.process_force`. τ_brake is the [holding brake](#holding-brakes) of the T-gantry, the same component (`HoldingBrake` in `mech.rs`, as `CableChain`), on the one motor shaft; in the rigid model (states x, v, the LuGre state and the brake deflection) it takes i · τ_brake / r off the drive force. The whole belt runs at carriage speed, so all of its mass sits on the pulley: J_pulley += ρ · L_total · r². The idler at the far end is free and has no inertia.

Rigid model, 3 states (x, v, LuGre state), with m_r = i² J_rotor / r²:

```text
a = ( λ · F_drive + F_mesh − F_f − m g sin(incline) + F_stop + F_extra ) / ( m + J_pulley/r² + λ · m_r )
```

λ is the gearbox loss factor on the transmitted force, found in two passes.

Signals: `belt.1/2.tension`, `axis.x.belt_stretch` (x − r · a), `axis.x.friction`, `belt.slack_time`, `pulley.m.radial_load` (T₁ + T₂), `axis.x.guide_load` (m · g · cos(incline), without the chain), `gearbox.m.output_torque` (elastic: τ_gear,out; rigid: λ · (F_drive − m_r · a) · r). And those of the [effect views](#effect-views), with `belt.1/2.stretch`; the simulation adds `temperature.gearbox_m` and `temperature.guide_x`.

`Machine::scene_point` maps x to where the scene draws the carriage: (span + x, −0.03, 0.06).

Not on the single axis: frame mode, leaning tool, idler inertia and friction, a resonance for the input shaper. In the rigid model, as in the elastic one, disabled drives leave it to the holding brake.

## Delta picker D3

Source: [`machine-delta-d3/src/lib.rs`](../crates/machine-delta-d3/src/lib.rs), with the joint of [`joint.rs`](../crates/sim-components/src/joint.rs). Equations after the research note of 13 November 2026: Williams, "The Delta Parallel Robot: Kinematics Solutions" (Ohio University, 2016), for the kinematics; Codourey (IJRR 17(12), 1998) as used by Olsson ("Modeling and control of a Delta-3 robot", Lund 2009) for the dynamics.

Three arms at the azimuths φ_i = 0°, 120°, 240° turn about horizontal axes at the base radius R; each carries a parallelogram of two forearms down to a platform of radius r, which stays level. The tool is the platform's centre p = (x, y, z), in a frame with its origin at the centre of the shoulder plane and z up (the platform works below 0). θ_i is the arm angle, 0 horizontal and positive downwards. Because the platform never turns, its offset folds into the base: e = R − r.

### Kinematics

```text
elbow       c_i(θ) = R_z(φ_i) · (e + L_a cos θ, 0, −L_a sin θ)
constraint  |p − c_i|² = L_b²                                  (the parallelogram's centre line)
inverse     (x′, y′, z) = R_z(−φ_i) p,   A = −2 L_a (x′ − e),   B = 2 L_a z,   C = (x′ − e)² + y′² + z² + L_a² − L_b²
            A cos θ + B sin θ + C = 0  →  θ = 2 atan( (−B − sqrt(A² + B² − C²)) / (C − A) )     (elbow out)
forward     the point at L_b from all three elbows (trilateration), the lower of the two
Jacobian    J_x ṗ = J_q θ̇,   J_x rows u_i = p − c_i,   J_q = diag(u_i · ∂c_i/∂θ_i),   ṗ = J θ̇,   J = J_x⁻¹ J_q
            J_x p̈ = J_q θ̈ + w,   w_i = θ̇_i² (u_i · ∂²c_i/∂θ²) − |ṗ − ∂c_i/∂θ θ̇_i|²
```

No solution of the inverse (A² + B² < C²): out of reach. The forward kinematics fails where the elbows cannot be joined (z² < 0) and at a direct singularity (J_x singular). The condition number of J (2-norm: the square root of the largest over the smallest eigenvalue of JᵀJ, by the closed form for a symmetric 3 × 3) is the signal `kinematics.condition`, and the check `singularity` judges its highest value in the cycle against `ratings.max_condition_number`. `Machine::condition` gives it at any pose (infinite out of reach) for the app's workspace map; the other machines have none to give.

**Working range** (`Machine::unreachable`): a cylinder of `workspace.radius_m` below `workspace.top_z_m`, its bottom a sphere through the edge at `height_edge_m` and the centre at `height_centre_m` below the top (Autonox's drawing: R 1060.74 mm = (600² + 186²) / (2 · 186)). The axis ranges are the box around it. A target outside the cylinder or the sphere, out of reach, with an arm beyond `joint.min_angle_deg` / `max_angle_deg`, or with a condition number above `workspace.max_condition_number` is refused with its reason (`error-delta-outside-envelope`, `-out-of-reach`, `-joint-stop`, `-singularity`). The controller checks moves, setpoints, cycle targets and every pose of a planned path against it; a straight move between two poses in the range stays in it (the range is convex). The standard suite's round trip runs over the poses of its grid that are in the range.

### Dynamics

Lumped (`effects.forearm_inertia` off), each forearm pair's mass m_fb is split: a share s (`mass.forearm_elbow_share`, default 2/3, which matches the rod's inertia about its end, m L²/3) turns with the arm at the elbow, the rest moves with the platform. The forearms' own rotational inertia is left out. Then the machine is three arms about their joints and a point mass at the platform:

```text
m_nt = m_platform + m_payload + 3 (1 − s) m_fb
I_b  = I_arm + L_a² (m_elbow + s m_fb)             (I_arm about the joint; rotor and gearbox not in it)
m_LG = m_arm l_cg + L_a (m_elbow + s m_fb)
M(θ) θ̈ = τ_out + Q_g − m_nt Jᵀ J_x⁻¹ w + τ_joint
M = I_b · 1 + m_nt JᵀJ,   Q_g,i = g m_LG cos θ_i − m_nt g J_zi
```

This is exact for the lumped model: the platform's kinetic energy ½ m_nt |J θ̇|² is what M holds, the velocity term is the platform's m_nt p̈ through Jᵀ with p̈ = J θ̈ + J_x⁻¹ w, and Q_g = −∂V/∂θ with V = m_nt g z − g m_LG Σ sin θ_i. τ_joint is the joint's friction (Coulomb, smoothed with tanh over `joint.friction_smoothing_rad_s`, and viscous; `effects.joint_friction`) and its stops (springs and dampers beyond the stop angles, `effects.hard_stops`).

**Elastic model** (`effects.joint_elasticity`), 15 states: θ, θ̇, rotor angles φ, rotor speeds, brake deflections. Each joint is a `RotaryJoint`: the gearbox's spring, damper, backlash and efficiency (the [gearbox](#gearbox)) pass τ_out to the arm, and

```text
J_rotor φ̈_i = ripple(τ_motor,i, φ_i) − τ_gear,rotor,i − τ_shaft(φ̇_i) − τ_brake,i
```

with J_rotor the motor's rotor and the gearbox's input inertia. **Rigid model**, 9 states (θ, θ̇, brakes): φ = i θ, and with the torque the motor puts on its arm through a loss-free gearbox with a massless rotor, d_i = i (ripple(τ_motor) − τ_shaft − τ_brake),

```text
(M + diag(λ_i i² J_rotor)) θ̈ = λ ⊙ d + Q_g − m_nt Jᵀ J_x⁻¹ w + τ_joint
```

λ_i the gearbox loss factor on what passes through it (d_i − i² J_rotor θ̈_i), found in two passes as on the other machines. Feedforward: the rigid, loss-free model, τ_motor = (I_b θ̈ + m_nt Jᵀ p̈ − Q_g) / i + J_rotor i θ̈ with θ̈ = J⁻¹ (p̈ − J_x⁻¹ w) and θ̇ = J⁻¹ ṗ (`feedforward_torque_moving`, which the controller uses; `feedforward_torque` is the same at rest). The velocity term changes the tool's following error in the Adept cycle little (1.20 mm without, 1.23 mm with; on a diagonal move 1.49 mm to 1.40 mm): what is left comes from the gearboxes' springs, which the rigid feedforward leaves out.

**Static hold, and stability.** At rest each gearbox holds −Q_g,i (with the twist τ / k plus half the backlash in the elastic model). Held by constant motor torques, as the standard suite holds it after a pulse, the D3 is unstable in its vertical mode in the upper part of its range: the stiffness of V − τ θ in the symmetric mode is −1.68 N m/rad at the middle of the default range (868 mm below the shoulders) and +2.64 N m/rad at 1000 mm (`the_vertical_mode_under_constant_torques_is_unstable_high_up`). The drives' position loops hold it; the suite's loss-free configs put the range, the start pose and the cycle 175 mm lower, where it is stable, so that the suite's 10 s energy check measures the integration and not this. Their stops keep their springs and lose their damping. The gearboxes' twist and backlash under the weight put the motors' pose (the setpoint of the machine at rest) up to 2.09 mm from the tool at the lower edge of the range (with the catalogue's 9 arcmin; 1.02 mm with 3 arcmin), 0.64 mm at pick and place (0.29 mm). `Machine::setpoint_slack` follows from that: the delta works it out when it is built (`rest_sag`) as the most that each gearbox's twist under its holding torque plus half its backlash either way, all eight combinations, moves the forward kinematics over a grid of the range (5 radii, 24 directions, 9 heights and the bottom, payload on and off), plus a tenth: 2.30 mm (0.1 mm on the belt machines, and in the rigid model). The controller lets a path or trajectory start that far from the tool and beyond the range or the envelope, and no further: a pose further from the start than the slack has to lie in the range (`near_start`). No limit cycle at rest with either backlash: the tool stands still to 2e-8 m peak to peak at five poses.

**Forearm inertia** (`effects.forearm_inertia`, on in the shipped config). Each forearm pair is a rigid slender rod from the elbow c_i to the platform p (the two rods of a pair move alike, the platform never turning; their spin about their own axis below). Its kinetic energy (m/6)(|ċ|² + ċ·ṗ + |ṗ|²) is exactly that of m/6 at each end and 2m/3 at its middle, and so is its weight (its centre is the middle). So the model above holds with s = 1/6 and m_nt = m_platform + m_payload + 3 m_fb / 6, plus per leg a point mass m_mid = 2 m_fb / 3 at (c_i + p) / 2:

```text
J_m,i = (J + ∂c_i/∂θ_i e_iᵀ) / 2                 (the middle's Jacobian; e_i the unit vector of arm i)
M    += Σ_i m_mid J_m,iᵀ J_m,i
v    += Σ_i m_mid J_m,iᵀ (J_x⁻¹ w + ∂²c_i/∂θ² θ̇_i²) / 2
Q_g  −= Σ_i m_mid g (J_m,i)_z
```

(`Delta::load_n`, which builds every point mass's share from its Jacobian). With the arms level at the centre the rods translate and both models give the same torque for a vertical acceleration; sideways at 1 m/s the rods' kinetic energy is 0.313 J against the lumped 0.376 J. In the Adept cycle the lumped model puts 2.6 % more RMS torque on motor 1 (2.645 against 2.575 N m; peak 7.786 against 7.640 N m) and 1.8 % more on motors 2 and 3 (1.615 against 1.587 N m).

**The rods' spin** (with `effects.forearm_inertia`: the rest of the same rigid body's inertia, so no switch of its own). The parallelogram keeps the plane of a pair through the elbow's axis a_i (fixed, along the base's tangent) and the forearm u_i = p − c_i, so each rod turns about its own axis as that plane does. With σ = a_i · u_i, n = a_i · (u̇_i × u_i) and L = L_b the spin is

```text
s_i = −σ n / (L (L² − σ²)) = g_i · q̇,   g_i = −σ / (L (L² − σ²)) J_u,iᵀ (u_i × a_i)     (J_u,i = J − ∂c_i/∂q)
M += Σ_i I_s g_i g_iᵀ,   v += Σ_i I_s (ṡ_i,0 g_i + s_i J_u,iᵀ (u̇_i × u_i) / L³)
```

by Kane's equations with the rod's angular velocity e × ė + s e (e = u / L) and its angular momentum about its axis I s e; ṡ_i,0 is the spin's rate at no acceleration of the coordinates. I_s is both rods' `mass.forearm_rod_spin_inertia_kgm2`, the kinetic energy I_s s² / 2 (`Delta::add_spin`, `rod_spin`). Where a forearm lies square to its elbow's axis (σ = 0: at the centre, or a leg whose plane holds the platform) its rods do not spin. At (0, 0.1, −0.8) moving 1 m/s square to leg 1's forearm, leg 1's rods spin at 0.1114 rad/s. In the Adept cycle (on y = 0, so leg 1's rods never spin) the 1.3e-5 kg m² change the motors' peak torque by 1e-6 N m and their RMS torque by less: negligible, as expected of a slender rod.

**Bending arms** (`effects.link_elasticity`, off in the shipped config), 12 more states after the brakes: the bend δ_i and its rate, then the move y_i and its rate. Each upper arm bends in the plane it turns in; the elbow's angle is θ_i + δ_i, and a spring k_b = 3 E I / L_a (a cantilever's tip deflection F L³ / (3 E I) as a turn about the shoulder) with a damper c_b (`[upper_arm]`) joins it to the arm's root. The arm's tube stays with the root, the elbow and all beyond move with the elbow angle:

```text
I_arm θ̈_i = τ_out,i + g m_arm l_cg cos θ_i + k_b δ_i + c_b δ̇_i + τ_joint,i
M_d(θ + δ, y) (θ̈ + δ̈, ÿ) = Q_d − v_d − (k_b δ + c_b δ̇, k_l y + c_l ẏ)   (of the elbows and beyond, at the elbow angles and moves)
```

and out of that plane the elbow moves by y_i along its axis a_i, on the tube's lateral stiffness k_l = 3 E I / L_a³ with a damper c_l (`upper_arm.lateral_stiffness_n_m`, `lateral_damping_n_s_m`). The elbow is then c_i + y_i a_i: the kinematics take it (the forward and inverse kinematics with the elbows moved, `Legs::forward_moved`, `inverse_moved`), and the platform's Jacobian by the moves is J_x⁻¹ diag(u_i · a_i) (`Legs::jacobians`), so a move pushes the platform only where the forearm is not square to the axis. M_d is 6 × 6: every point mass (platform, rods' middles, the elbow mass on the moves) with its Jacobian in (θ + δ, y), the rods' spin too. The kinematics (tool, condition number, scene, collision bodies) take the elbow angles and moves; the arm angle signal, the stops and the gearbox the joint's. At rest δ = Q_d / k_b and y = Q_y / k_l (three rounds from y = 0; Q_y hardly changes with y). Both dampers' loss is `power.arm_bending`, both springs' energy is elastic energy. The feedforward stays the rigid model's. In the Adept cycle the tracking error grows from 4.0 to 5.3 mm, the dwell error from 0.68 to 0.73 mm; the moves out of the plane add little (motor 1 RMS 2.6121 to 2.6122 N m, motors 2 and 3 peak 4.0400 to 4.0381 N m, dwell error 0.7295 to 0.7313 mm: the cycle runs on y = 0, where leg 1's forearm is square to its axis).

**Resonance** (`Machine::modes`, `resonance_hz`, `mode_shape`; what `effects.shaper_follows_pose` sets the input shaper to). The modes with the motors held by their position loops: per arm the gearbox's spring k_g (with `joint_elasticity`) from the held rotor to the arm's root, the bending spring k_b (with `link_elasticity`) from the root to the elbow, the lateral spring k_l on the elbow's move, the root I_arm and everything beyond the mass matrix M_d at the elbow angles (and moves). The coordinates (roots, elbow angles, moves) with the flexibility per arm K⁻¹ = [[1/k_g, 1/k_g], [1/k_g, 1/k_g + 1/k_b]] and 1/k_l (a rigid spring 0): with M = L Lᵀ (Cholesky) the eigenvalues μ = 1/ω² of the symmetric Lᵀ K⁻¹ L by Jacobi rotations, a rigid connection's μ = 0 left out (`arm_modes`). Each mode's damping ratio is vᵀCv / (2 ω vᵀMv) with C the gearboxes', the bending's and the lateral dampers (the cross terms of a damping that is not proportional left out). 3 modes with the gearboxes alone, 6 with the bending alone, 9 with both; `resonance_hz` is the lowest. Gravity's stiffness (a few N m/rad) and backlash are left out. With rigid gearboxes and arms there is none. At the centre with the arms level the lowest is the platform's sideways mode (twice): the forearms lumped, 28.14 Hz on the gearboxes alone, 38.76 Hz on the bending alone, 22.88 Hz with both (vertical: 68.39 Hz on the bending alone). There every forearm is square to its axis, so the elbows' moves ring on their own: lumped sqrt(k_l / (m_elbow + s m_fb)) / 2π = 126.48 Hz three times; with the rods, whose middles also move with the platform, 147.35 Hz for all three elbows moving the same way round and up to 0.3 % more for the two that couple to the sideways modes.

### Signals

`arm.<k>.angle` (rad), `gearbox.<k>.output_torque` (N m; elastic: the spring's torque on the arm, rigid: λ (d − i² J_rotor θ̈)), `kinematics.condition`. Effect views: the [torque breakdown](#torque-breakdown) per motor with inertia = ((M θ̈ + velocity term)_k + i² J_rotor θ̈_k) / i, gravity = −Q_g,k / i, friction = shaft + brake + gearbox loss at the rotor + joint friction / i; the loss powers `power.joint_<k>`, `power.shaft_<k>`, `power.gearbox_<k>`, `power.hard_stops`, `power.other` (ripple), with bending arms `power.arm_bending`; the energies. With bending arms the joint carries the tube and the spring: gravity = −(g m_arm l_cg cos θ + Q_d) / i, inertia the rest of the spring's torque. The joints are not offered as friction contacts: the friction view draws a guide's force over its speed along an axis. Each gearbox warms with `effects.part_heating` (`temperature.gearbox_<k>`) and follows its temperature with `effects.thermal_feedback`.

`Machine::scene_point` is the platform's centre: the machine's frame is the scene's. The scene: base plate, per arm a motor and a gearbox box with a bar turned by the rotor angle, the arm (to its elbow, bent or not) and the two rods of its forearm pair as segments, the platform as a box, the payload under it. The collision bodies are the platform, the payload, each forearm rod as ten boxes along it and each upper arm as ten boxes along it (each the box around its piece of the drawn segment, widened by the segment's width: 12 mm for a rod, 40 mm for an arm). An obstacle an upper arm runs into is a crash as any other.

Not modelled: the tube's own share of the bending modes (it stays with the root in the plane and does not move out of it; 33/140 of a beam's mass would go to its tip), the elbow block's twist as the arm bends out of its plane (the elbow's axis stays as it is), elasticity of rods and ball joints (no figures published for any product), friction in the ball joints (lumped into the joint friction), the ball joints' cone angles (the stops and the working range stand in), collisions between the links, a fourth axis (see the [D4](#delta-picker-d4)).

## Delta picker D2

Source: [`machine-delta-d2/src/lib.rs`](../crates/machine-delta-d2/src/lib.rs). The planar delta: two arms in the vertical x-z plane, their shoulder axes along y at x = ±R, each with a pair of forearms down to a plate that keeps its orientation (a five-bar whose lower links are parallelograms, as in Codian's D2 or ABB's two-axis FlexPicker). The tool is the plate's centre p = (x, z), z up from the shoulder axes. The equations are the [D3's](#delta-picker-d3) with two legs at φ = 0° and 180° and y = 0; leg i points along s_i x (s_1 = +1, s_2 = −1):

```text
elbow       c_i(θ) = (s_i (e + L_a cos θ), −L_a sin θ)
inverse     x′ = s_i x,  A = −2 L_a (x′ − e),  B = 2 L_a z,  C = (x′ − e)² + z² + L_a² − L_b²,  θ = 2 atan((−B − sqrt(A² + B² − C²)) / (C − A))
forward     the lower of the two points at L_b from both elbows (two circles)
Jacobian    2 × 2: J_x rows u_i = p − c_i, J_q = diag(u_i · ∂c_i/∂θ_i), J = J_x⁻¹ J_q; condition number from the eigenvalues of JᵀJ in closed form
dynamics    m_nt = m_plate + m_payload + 2 (1 − s) m_fb,  M = I_b · 1 + m_nt JᵀJ,  Q_g,i = g m_LG cos θ_i − m_nt g J_zi   (I_b, m_LG as on the D3)
```

The joints, the elastic and rigid models (10 and 6 states), the feedforward, the signals (`arm.1/2.angle`, `gearbox.1/2.output_torque`, `kinematics.condition`), the effect views, part heating and the refusals (`error-delta-*`) are the D3's. **Working range**: the band |x| ≤ `workspace.half_width_m` below `top_z_m`, its bottom an arc through both edges at `height_edge_m` and the centre at `height_centre_m` below the top. **Static hold, and stability**: held by constant motor torques the plate's vertical mode is unstable in the upper part of the range, −3.49 N m/rad at the middle of the default range (638.5 mm below the shoulders), +7.52 N m/rad at 788.5 mm; the sideways mode is stable (13.5 and 18.9 N m/rad). The standard suite's loss-free configs put range, start and cycle 150 mm lower. `Machine::setpoint_slack` is `rest_sag` over 41 columns of the range: 0.42 mm with 1 arcmin backlash (at most 0.37 mm found at rest in the simulation).

The model takes the plate as kept level. The 3D view draws each forearm pair as two rods apart along y, as on the D3; in the plane, a plate on two legs needs a parallelogram in the x-z plane (or a third link) to keep it from turning about y, which real D2s have. The scene: a base beam, per arm a motor and gearbox box with the rotor bar, the arm, the two rods, the plate and the payload; the collision bodies are the plate and the payload. Not modelled, as on the D3: the forearms' rotational inertia, elastic links, friction in the ball joints, collisions between links.

## Linear delta

Source: [`machine-delta-linear/src/lib.rs`](../crates/machine-delta-linear/src/lib.rs), with the belt, guide, coupling and motor components of the single axis. The parallel kinematics are the D3's with the arms replaced by carriages on vertical columns (as the Festo EXPT tripod and the Kossel printers); the dynamics are at the D3's level (Codourey 1998, Olsson 2009).

Three columns stand at the azimuths φ_i = 0°, 120°, 240° at the column radius R. Each carries a carriage on a vertical toothed-belt axis; a parallelogram of two rods of length L joins it to a plate of radius r that stays level. The tool is the plate's centre p = (x, y, z), in a frame with its origin on the column axis at the carriages' upper stop and z up (the plate works below 0). h_i is carriage i's joint height, x_i = h_i − h_min its place in its stroke. Because the plate never turns, e = R − r.

### Kinematics

```text
joint       c_i = (e cos φ_i, e sin φ_i, h_i)
constraint  |p − c_i|² = L²
inverse     h_i = z + sqrt(L² − ρ_i²),   ρ_i² = (x − e cos φ_i)² + (y − e sin φ_i)²       (carriage above the plate)
forward     the point at L from all three joints (trilateration), the lower of the two
Jacobian    J_x ṗ = J_q ḣ,   J_x rows u_i = p − c_i,   J_q = diag(u_i,z),   ṗ = J ḣ,   J = J_x⁻¹ J_q
            J_x p̈ = J_q ḧ + w,   w_i = −|ṗ − e_z ḣ_i|²
```

Raising all three carriages alike lifts the plate alike: the z row of J sums to 1, the x and y rows to 0. The condition number of J is the signal `kinematics.condition`, judged by `singularity` as on the D3; the default range has up to 3.98 (at its bottom rim, opposite a column), the centre 3.15: a metre of carriage moves the plate further sideways than up.

**Working range** (`Machine::unreachable`): a cylinder of `workspace.radius_m` from `workspace.bottom_z_m` to `top_z_m`; the axis ranges are the box around it. A target outside it (`error-delta-outside-envelope`), out of the rods' reach (`-out-of-reach`), where a rod would stand steeper from the vertical than `workspace.max_rod_angle_deg` (`error-delta-linear-rod-angle`: ρ_i > L sin of it), where a carriage would leave its stroke from `geometry.carriage_top_m` − `stroke_m` to `carriage_top_m` (`error-delta-linear-carriage-stop`), or with a condition number above `workspace.max_condition_number` (`-singularity`) is refused with its reason.

### Dynamics

Each rod pair's mass m_rod is split: a share s (`mass.rod_carriage_share`, default 1/2) moves with its carriage, the rest with the plate. The rods' rotational inertia is left out. Then

```text
m_c  = m_carriage + s m_rod,   m_nt = m_plate + m_payload + 3 (1 − s) m_rod
M(h) ḧ = F_belt + Q_g − m_nt Jᵀ J_x⁻¹ w − F_guide + F_stop
M = m_c · 1 + m_nt JᵀJ,   Q_g,i = −g m_c − m_nt g J_zi
```

exact for the lumped model as on the D3 (V = g (m_nt z + m_c Σ h_i)). Each column is a belt axis as on the [single axis](#single-axis), upright with its motor at the top: section 1 runs from the motor pulley straight down to the carriage (length span + stroke − x), section 2 from the motor pulley down round an idler and up to the carriage (3 span + stroke + x); the belt pulls the carriage up with T₁ − T₂ and turns the pulley with r (m₂ − m₁), the material forces of the [belt sections](#belt-sections). The guide of each carriage is a [LuGre contact](#guide-friction-lugre); with `effects.load_dependent_friction` its levels scale with its load across the rail, the horizontal part of the rods' pull that holds the plate at rest, |λ_i| ρ_i with J_xᵀ λ = −m_nt g e_z (the levels hold for the guide under its preload alone). The stops are springs and dampers beyond the stroke.

**Elastic model** (`effects.belt_stretch`), 25 states: h, pulley angles, rotor angles, their speeds, the guides' friction states, the brakes and the time a belt section has spent without tension (`belt.slack_time`, as on the single axis). The pulley: J_pulley ä = τ_gear,out + r (m₂ − m₁); the rotor as on the single axis, behind the coupling (or gearbox). **Rigid model**, 12 states (h, ḣ, friction, brakes): x = r a, rotor = i a, and

```text
(M + diag(J_pulley / r² + λ_i i² J_rotor / r²)) ḧ = λ ⊙ d + Q_g − m_nt Jᵀ J_x⁻¹ w − F_guide + F_stop,   d_i = i (ripple(τ_i) − τ_shaft − τ_brake) / r
```

λ_i the gearbox loss factor, in two passes as on the other machines. Feedforward: the rigid, loss-free model, τ_k = ((m_c + J_pulley / r²) ḧ_k + m_nt (Jᵀ p̈)_k − Q_g,k) r / i + J_rotor i ḧ_k / r with ḧ = J⁻¹ (p̈ − J_x⁻¹ w) (`feedforward_torque_moving`). `rigid_tensions` shares each carriage's force F = m_c ḧ_k + m_nt (Jᵀ p̈)_k − Q_g,k between its two sections so that the belt keeps its length: T₁ = P + F L₂ / (L₁ + L₂), T₂ = P − F L₁ / (L₁ + L₂).

**Static hold.** At rest each belt holds −Q_g,i: at the centre the carriage's weight and a third of the plate's, 22.07 N with the default masses (`the_belts_hold_the_carriages_and_the_plates_weight_through_the_jacobian`). Held by constant motor torques the hanging plate is stable sideways (it rises to second order when two carriages move apart) and neutral in the vertical mode, so the standard suite's loss-free configs keep the default range. The belts' stretch and the couplings' twist put the motors' pose 0.01 to 0.02 mm off the tool at rest; `setpoint_slack` is worked out over a grid of the range as on the D3 (`rest_sag`) and stays at the default 0.1 mm.

### Signals

Per column k: `carriage.<k>.height` (m), `belt.<2k−1>.tension` and `belt.<2k>.tension` (N; rigid: the pretension), `pulley.<k>.radial_load` (T₁ + T₂, judged by `gearbox-radial-load`), `carriage.<k>.guide_load` (N), `gearbox.<k>.output_torque` (N m, judged by `pulley-torque`); `kinematics.condition`; `belt.slack_time` (s, elastic model). Effect views: the [torque breakdown](#torque-breakdown) per motor with inertia = r / i ((M ḧ + velocity term)_k + (J_pulley + i² J_rotor) / r² ḧ_k), gravity = −r / i Q_g,k, friction = shaft + brake + gearbox loss at the rotor + r / i guide friction; the loss powers `power.guide_<k>`, `power.shaft_<k>`, `power.gearbox_<k>`, `power.belt_<k>`, `power.hard_stops`, `power.other`; the energies; the friction contacts `carriage.<k>` with their `friction_*` signals; `belt.<i>.stretch`. The six belt sections are offered to the belt view.

`Machine::scene_point` is the plate's centre. The scene: a frame plate, per column a column, its belt loop with motor, pulley and idler (drawn in the plane parallel to x–z through the column), the carriage and the two rods of its pair, the plate and the payload under it.

Not modelled: the rods' rotational inertia; elasticity of frame, columns, rods and ball joints; friction in the ball joints; collisions between the links; a fourth axis; the guides' heat (no part warms but the motors); what accelerating the plate adds to the guides' load (the load at rest stands in); tooth meshing, tooth jump and belt-span vibration.

## Delta picker D4

Source: [`machine-delta-d4/src/lib.rs`](../crates/machine-delta-d4/src/lib.rs). The [D3](#delta-picker-d3) with a fourth axis that turns the tool about the vertical, as on ABB's IRB 360 with its fourth axis: a fourth motor with its gearbox in the centre of the base drives a telescopic shaft through a cardan (universal) joint, and the shaft drives the tool's shaft in a bearing of the platform through a second cardan joint. The tool pose is (x, y, z, c), c the tool's turn about +z from +x (rad). The arms are the D3's `Delta`, built from the D4's config with the shaft's share of mass added to `mass.platform_kg`: its kinematics, Jacobian, dynamics, forearm rods and their spin, bending arms (in and out of their planes), modes, rest sag, refusals and collision bodies of rods and arms are the D3's, unchanged. The D3 gained one public hook for it, `PlatformDrag` (`Delta::set_platform_drag`): a force on the platform along the line from a fixed point, against its lengthening, added as Jᵀ F to the velocity term of `load_at` (so to the dynamics and the bending arms'; the feedforward, the rigid loss-free model, leaves it out, so that its speed terms stay quadratic in the speed as the time-optimal planner reads them) and moved to the friction part of the torque breakdown; the D3 has none.

```text
shaft       d = p + (0, 0, h) − (0, 0, z_u)     upper joint on the base's axis at z_u = shaft.upper_joint_z_m, lower one h = lower_joint_offset_m above the platform
            L = |d|,  β = angle(d, −z),  ψ = atan2(d_y, d_x)            length, the bend of both joints, the bending plane's azimuth
cardan      tan χ = tan φ / cos β,  χ′/φ′ = cos β / (1 − sin²β cos²φ)   one joint, φ and χ from the bending plane (the input yoke's pin in it at φ = 0)
pair        tan φ_out = cos β tan χ = tan φ                            the second joint, yokes of the intermediate shaft in phase: undoes the first
tool        J_c c̈ = τ_gear + τ_bearing,  J_c = J_tool + J_payload + J_s
arms        the D3's, with m_platform + r_s m_s at the platform, and Jᵀ F, F = −f(L̇) u, u = d / L, L̇ = u · ṗ (the telescope's friction)
```

**Why the pair cancels.** A single cardan joint bent by β turns its output unevenly, between cos β and 1/cos β of the input's speed, twice per turn. The D4's motor shaft and the tool's shaft are both vertical (the parallelograms keep the platform level), so the two joints are bent by the same β in the same plane: the Z arrangement. With the yokes of the intermediate shaft in phase the second joint undoes the first, and the tool turns exactly as the gearbox's output, c = θ_out, at any pose and while the platform moves. Only the shaft between the joints turns unevenly: the signal `shaft.speed` is the tool's speed times cos β / (1 − sin²β cos²(c − ψ)) (the platform's own motion adds a part to it that is left out). Its spin inertia J_s counts at the tool's speed: its mean over a turn would be J_s (1 + cos²β) / (2 cos β), 1 % more at 30° and of a part that is 1 % of J_c. Yokes out of phase (by 90°) would double the unevenness at the tool; that is a fault of assembly, not modelled.

**The tool's joint** is a `RotaryJoint` ([`joint.rs`](../crates/sim-components/src/joint.rs)): the machine's `[motor]` (every motor of a machine has the same motor and drive), the gearbox of `[tool_gearbox]` (read as a `[gearbox]`), brake and ripple as on the arms, the tool bearing's friction (`[tool]`, effects.joint_friction), no stops (the turn is endless; `axis.c.min_angle_deg`/`max_angle_deg` are the controller's range). Rigid (effects.joint_elasticity off) the rotor turns with the tool times the ratio, with the gearbox's efficiency as on the arms; elastic, rotor and tool are two bodies behind the gearbox's spring, and with effects.shaft_torsion the shaft's torsional stiffness is in series with it, 1 / (1/k_gear + 1/k_shaft), damped by the shaft's `torsional_damping_nm_s_rad` (the shaft is the softer by far: 500 against 8594 N m/rad). The tool's torsional mode with the motor held, √(k/J_c), is among `Machine::modes`: 41.1 Hz with the payload, 75.9 Hz without, 2 % damping. `resonance_hz` and `mode_shape` stay the arms' (what an input shaper on the path is for). States: the D3's, then c, ċ, (rotor angle and speed, elastic), brake: 3 or 5 more.

**Coupling to the arms.** Nothing turns the parallelograms, so the shaft acts on the arms only through its weight (effects.shaft_mass: `shaft.platform_share` of `shaft.mass_kg` at the platform, the inner shaft and the lower joint; the outer tube hangs from the upper joint) and through the telescope's sliding friction (effects.telescope_friction, `shaft.slide_friction_n` Coulomb, smoothed over `slide_friction_smoothing_m_s`, and a viscous part), a force along the shaft against its lengthening which the arms take through Jᵀ. The torque the shaft passes and the joints' bending moments on the platform do no work on the arms: the platform cannot tilt or turn. The shaft's own swing about the upper joint is not counted (as the D3 leaves out the forearms' spin).

**Signals** beyond the D3's: `tool.angle`, `gearbox.4.output_torque` (judged by `tool-gearbox-peak` against `ratings.tool_gearbox_max_torque_nm`), `shaft.length`, `shaft.cardan_angle`, `shaft.speed`, and in the effect views `motor.4.torque_*`, `power.joint_4` (tool bearing), `power.shaft_4`, `power.gearbox_4`, `power.telescope`, `power.other_4`; the energies include the tool's. The tool's gearbox warms as a fourth `gearbox_4`. **Refusals** beyond the D3's: `error-delta-shaft-stroke` (the telescope shorter than `min_length_m` or longer than `max_length_m`), `error-delta-cardan-angle` (a joint bent beyond `max_cardan_angle_deg`); with the defaults every pose the arms reach the shaft reaches too (0.625 m at the top of the centre, 1.020 m at the edge's bottom, at most 43.8° at the top of the edge). **Scene**: the D3's, then the fourth motor, gearbox and rotor bar in the base's centre, the telescope as an outer tube from the upper joint and an inner shaft from the lower one, the two joints, and a bar under the platform turned with the tool. **Collision bodies**: the D3's and the tube and the inner shaft as six boxes each along them.

**Paths with a turn**: `line,x,y,z,c` (lines work on any number of axes); arcs and splines are on two and three axes only, so they are refused on the D4. Not modelled: yokes out of phase, the cardan joints' friction and play, the shaft's bending and swing, a different motor for the fourth axis (the drive reads one `[motor]` per machine), the gearbox checks of motor 4 with its own gearbox (`gearbox-*` of every motor take `[gearbox]` and `[ratings]`: read them for motors 1 to 3; `tool-gearbox-peak` judges the tool's).

## Arms in tool space (SCARA, six-axis arm)

Source: [`machine-joint-kit/src/tool.rs`](../crates/machine-joint-kit/src/tool.rs), wired in `arm.rs`; the joint machine's side in [`chain.rs`](../crates/sim-components/src/chain.rs). With `kinematics.tool_space` the axes of `scara` and `urdf-arm` are the flange's pose; the joints' dynamics (Newton-Euler on the URDF chain), drives, losses and signals are unchanged, in joint coordinates. Details, refusals and the branch kept by the six-axis inverse: [mechanisms](mechanisms.md#tool-space).

```text
SCARA       x = x1 + L1 cos(q1 + φ1) + L2 cos(q1 + q2 + φ2),  y likewise with sin,  z = z0 + s q3,  c = c0 + q1 + q2 + q4
            cos β = (r² − L1² − L2²) / (2 L1 L2),  β = ±acos (elbow),  q1 = atan2(y, x) − atan2(L2 sin β, L1 + L2 cos β) − φ1,  q2 = β − φ2 + φ1
six joints  (x, y, z) = p_tip(q),  (rx, ry, rz) = log R_tip(q);  Δq = Jᵀ (J Jᵀ + λ² I)⁻¹ e,  e = (p* − p, ℓ log(R* Rᵀ)),  λ = 1 mm,  ℓ = |p_tip − p_joint2| at the home
speeds      q̇ = J⁻¹ v,  q̈ = J⁻¹ (a − J̇ q̇),  J̇ q̇ = [f(q + h q̇) − 2 f(q) + f(q − h q̇)] / h²,  h |q̇| = 1e-4,  J = ∂f/∂q by central differences
feedforward τ = (M q̈ + h(q, q̇)) / i + J_rotor i q̈   per motor (in joint space the speed terms are left out, as before)
condition   κ = √(λ_max / λ_min) of Jᵀ J (Jacobi rotations): SCARA of ∂(x, y)/∂(q1, q2), six joints of the geometric 6 × 6 with turn rows × ℓ
```

The joints' units are their own (`JointSpec::unit`), no longer the axes': a joint's position signal, its force's unit and whether it has a gearbox output torque follow the joint. The time-optimal planner reads the motors' limits through `feedforward_torque_moving`, so along a line of the tool it slows where a motor reaches its rated speed (on the SCARA's example line, joint 1 near the middle, 307.5 of 314.2 rad/s). Before timing a path as fast as the motors allow, `Controller::plan_path` checks the path as planned within the axis limits: a pose on it the machine cannot go to (a singularity, where the motors would have to turn infinitely fast) is refused with `error-path-range` at once, where the time-optimal timing would slow to a standstill there.

## Motion controller

Source: [`control.rs`](../crates/sim-components/src/control.rs).

The controller plans in tool coordinates and runs one drive per motor. Each profile cycle:

1. The motion under way gives position, velocity and acceleration setpoints per tool axis: a point-to-point move, the cycle, a trajectory, or an interpolated external setpoint.
2. The [input shaper](#input-shaper) spreads them over time, if on.
3. The inverse transformation gives the motor position setpoints. Motor speed and acceleration setpoints are the inverse transformation applied a moment ahead and behind (central difference).
4. The machine's rigid model gives the torque feedforward for the setpoint pose and acceleration.
5. With `effects.fieldbus_delay` the drives get what was computed `profile.fieldbus_delay_cycles` cycles ago. With `effects.fieldbus_jitter` each frame takes j more cycles, j drawn evenly from 0 to `profile.fieldbus_jitter_cycles` per frame (pseudo-random from `drive.noise_seed`, a stream of its own, so every run is the same), and the drives use the newest frame that has arrived: a late frame leaves them the one before, and frames never arrive out of order. With N cycles of jitter the frame in use is older than the delay by more than i cycles with probability Π_{k=0..i} (N − k) / (N + 1): N = 1 gives 0 or 1 cycle, half each.

The setpoint signals and the path error of the analysis are the controller's, before the fieldbus delay.

| Command | Behaviour |
| --- | --- |
| Move | Synchronised point-to-point move. Refused while busy or outside the strokes |
| Cycle | Runs the configured segments in order, repeating. On arrival the payload switches to the segment's state and the dwell runs. With input shaping, a segment with a dwell or a gripper action has arrived when the shaped setpoint has |
| Stop | Finishes the segment under way, then stays. There is no fast stop ramp |
| Setpoint (PLC) | Linear interpolation from the present setpoint to the new one over `profile.setpoint_interval_s`, counted from the profile cycle in which the setpoint came, so that a stream of one setpoint per interval is followed without a pause. The speed setpoint is that of the interpolation, up to and including the cycle of arrival; the acceleration setpoint is 0. A setpoint further from the last one than the axis travels in one interval at its speed limit is refused |
| Trajectory | Poses at times. A cubic curve through the points (Hermite), with the speed at each point the slope between its neighbours and rest at both ends. Poses up to 0.1 mm beyond the travel range are taken: the setpoint of a machine at rest is where its motors stand, which at an end of a stroke is some hundredths of a millimetre beyond it. A planned path's poses that rounding puts beyond the range (by less than a nanometre) are set onto its end |
| Disable / Enable | Drives off: the motion under way is dropped. Every other command is refused until Enable |

The controller's setpoint starts where the motors stand, a few micrometres from the configured start pose (belt stretch and gear twist under load), so the first step has no jolt.

The following error per tool axis (signal `axis.<id>.following_error`) is the motor following errors transformed with the forward transformation. It is an encoder-side figure and does not see belt stretch or the leaning tool.

**Check of axis limits against motor limits.** When a config is applied, for every combination of axis directions at the middle of the workspace: the motor speed at the axis speed limits against `drive.rated_speed_rad_s`, and the feedforward torque at the axis acceleration limits against `drive.peak_torque_nm`. A diagonal move loads one motor with both axes. Violations are reported, not enforced.

## Profile generator

Source: [`profile.rs`](../crates/sim-components/src/profile.rs).

A jerk-limited seven-segment S-curve from rest to rest: jerk up, constant acceleration, jerk down, cruise, and the mirror image. With limits v, a, j:

```text
t_j = a / j,  t_a = v/a − a/j         if v · j ≥ a²        (acceleration limit reached)
t_j = sqrt(v / j),  t_a = 0           otherwise
```

If the distance is too short to reach the cruise speed, the peak speed is lowered: the constant-acceleration phase is shortened, and for very short moves dropped (t_j = cbrt(d / 2j)).

A move of several axes is one S-curve that runs a straight line from 0 to 1. Its limits are the tightest of all moving axes (limit / |distance|), so all axes start and finish together and none exceeds its own limits.

## Corner blending

`effects.corner_blending`: the next cycle segment starts while the current one still has its braking ahead, and the remaining distance of the old move is added on top of the new one. The lead is the smallest of `profile.blend_time_s` and the ramp times of both moves. It happens only where the machine need not stop: the segment has no dwell and no gripper action, no axis reverses, and no other blend is under way.

Because both moves are on their ramps, the speeds add up to no more than either cruise speed on lift-traverse-lower cycles. For other shapes of cycle the sum of two segments on one axis is not checked against the limits.

## Path planners

Source: [`path.rs`](../crates/sim-components/src/path.rs). Both planners turn lines, arcs and splines into poses at times, one per profile cycle, which then run as a trajectory. A *corner* is a junction where the direction turns by more than 1 mrad; both planners stop there.

### Within the axis limits (default)

Each piece gets one speed, acceleration and jerk limit along the path that hold on all of it:

- **Line:** the tightest of limit / |direction component| over the axes.
- **Arc of radius R:** the direction turns through the axes, so the smallest limit of any axis holds for the length of the vector. Along the path (t) and towards the centre (n), at the speed v:

  ```text
  acceleration = t · dv/dt + n · v²/R
  jerk         = t · (d²v/dt² − v³/R²) + n · 3 · v · dv/dt / R
  ```

  The limits are shared out 3 : 4 : 5. Acceleration: 0.6 along, 0.8 towards the centre. Jerk: 0.4 for the ramps and 0.4 for v³/R², and 0.6 towards the centre. So v ≤ min(v_max, sqrt(0.8 a R), cbrt(0.4 j R²)).

- **Junction** (no corner): the direction may turn a little and the curvature may jump (line into arc), which steps the axis speeds and accelerations. Taken over one profile cycle, each of the two steps may use half an axis's jerk limit. For a line into an arc that is v = sqrt(0.5 j T R): 0.07 m/s for 0.1 m at 100 m/s³ and a cycle of 1 ms. Transition curves (below) take the jump away.
- **Transition curve:** counts as an arc of its tightest radius R. Its curvature changes along it by dk per distance, which adds n · v³ · dk to the jerk; that gets 0.4 of the 0.6 towards the centre, and the 3 v dv/dt / R term the other 0.2. So v ≤ cbrt(0.4 j / dk) in addition, and the acceleration along the curve is at most 0.2 j R / (3 v).

- **Spline span:** consecutive `spline` pieces are one cubic spline through their points: the parameter u runs with the straight distance between the via points, each span is a cubic in u, and first and second derivative by u agree at every via point, so position, direction and curvature have no step there. At an end the curve takes the direction of the line or arc it joins (the first derivative is that unit direction); without one it ends with no second derivative. The derivatives at the points follow from a system with three entries per row. Each span is one piece for the timing. Distance along it is a table at 65 values of u (five-point Gauss-Legendre per part), inverted by Newton's method. Its curvature k = (p′ × p″) / |p′|³ and the change of curvature per distance are taken at 257 places; the largest of each, raised by 5 % for what lies between, give the span its limits like a transition curve: an arc of radius 1 / k_max with dk = max |dk/ds|. So a span runs no faster than its tightest place allows, and the speed changes from span to span, not within one. Where a spline joins a line or an arc the curvature steps, and the junction rule above slows the path there, unless a path tolerance lets the spline's end be eased (below). A span is a polynomial of the fifth degree where it was eased, a cubic elsewhere. A curve whose |p′| falls below 0.05 anywhere (1 on average) is refused: its via points and end directions would make it turn back on the spot.

Look-ahead then lowers the junction speeds until every piece is long enough to change from the speed at its start to the speed at its end. On each piece the speed goes by a jerk-limited ramp to the highest speed the piece has room for, stays there, and goes by a second ramp to the speed at its end. Two cycles at steady speed are kept on either side of a junction.

### Transition curves (`profile.path_tolerance_m`)

With a tolerance above zero, both planners time a changed path: where a line and an arc, or two arcs, meet in the same direction, a curve is put in whose curvature k goes evenly with the distance from that of the one piece to that of the other (a clothoid). At the speed v the acceleration towards the centre then changes at the rate v³ dk / L instead of stepping (L: length of the curve, dk: step of the curvature, 1/R for a line into an arc).

Such a curve does not fit between a line and a circle that touch. It fits when the circle is moved in by

```text
e = L² / (24 R)          first term; the code solves the exact length by halving
```

So every blended arc keeps its centre and gets the radius R − e, and the curves replace about L/2 of the pieces on either side of each junction. The planned path is nowhere further from the given one than the largest e, which `path::deviation` reports. Each arc gets its own L:

```text
L = min( max over the arc's two junctions of v³ dk / (0.4 j),  sqrt(24 · tolerance · R) )
```

with v the speed limit of the slower of the two pieces of a junction and j the smallest jerk limit of any axis. With an arc moved in by its e, a line beside it is apart from it by e and the curve there comes out about L long. Two arcs turning opposite ways are apart by the sum of their e, which fits whatever their L. Two arcs turning the same way are apart by the difference of their e (the tighter one moves in more), which is positive only for one L: arcs that hand over to one another turning the same way share one L, the longest any of them needs and the shortest any of them allows. Each curve is then fitted exactly: its length is the one for which the circles that fit its two ends are as far apart as the real ones, and it is turned and placed so that they coincide. A point on the curve is the direction added up along it (Simpson's rule, 64 intervals).

**An arc with one end fixed.** An arc at a corner, at an end of the path, or where a spline joins it has to end at that end where it is given to, in the given direction. If its other end meets a line or an arc in its direction, its circle shrinks by s and its centre moves towards the fixed end by s: the new circle passes through the fixed end in the same direction (it touches the old one there from inside), and is inside the old one by s (1 − cos φ) at the other end, φ being the angle the arc turns through. That gap is made e = L² / (24 R), so s = e / (1 − cos φ), and the curve towards the line or the next arc is fitted as above (next to an arc the gap is taken as e for that arc's moving in, and the curve is fitted to where the circles are). At the angle θ from the fixed end the new circle is r − sqrt(r² − 2 s (r − s)(1 − cos θ)) from the old one, which grows up to θ = π, where it is 2 s: about e at the free end of an arc of up to half a turn. An arc that turns further gets s ≤ tolerance / 2, so e = (tolerance / 2)(1 − cos φ) and a shorter L (`an_arc_fixed_at_one_end_that_turns_three_quarters_gets_its_curve`: 270°, r = 0.1 m, 0.1 mm: s = e = 0.05 mm, L = sqrt(24 e r) = 10.95 mm, 0.1 mm reported). The larger of e and that distance is reported. This is done for arcs whose radius stays above half of what it was, not for a whole turn (no free end); for a quarter turn s = e, and the numbers are those of a free arc (`an_arc_at_an_end_or_a_corner_gets_its_transition_curve`: 15.49 mm, 0.3957 m/s into the arc instead of 0.0707 m/s at 0.1 mm).

**On three axes.** A line that meets an arc in its direction lies in the arc's plane, and so does a line or an arc that meets that line in its direction and lies in the plane (to 1 nm). Such a run of pieces is blended in its plane, on the coordinates along the first arc's start and its direction there, with the speeds of the pieces in space, and laid back into space; the curves keep their plane, and the deviation is the one in the plane. A line between two arcs in different planes lies in both and is cut in half, each half going with its arc. Two arcs that meet in their direction in different planes keep their junction, which the path passes at the junction speed: a curve between them would have to turn from one plane into the other (with torsion), and the planar fit has no counterpart there. Spline junctions are eased as on two axes. `a_path_in_a_tilted_plane_gets_its_transition_curves` (delta, a quarter arc of 0.1 m in a plane tilted by 30°, 0.1 mm): L = sqrt(24 e r) = 15.49 mm, e = 0.1 mm reported, 0.551 s against 0.616 s within the axis limits.

**A spline eased into a line or an arc.** A spline leaves a line or an arc in its direction at a speed by u of 1, but with the curvature its via points ask for, so the curvature steps by dk at the junction. With a tolerance the span next to the junction gets, over a length l from the junction (u from 0 to l, w = u / l),

```text
delta(u) = D · l² · w² (1 − w)³ / 2
```

added, D being the change of the second derivative across the direction that makes the curvature at the junction that of the neighbour (|D| = dk). delta has no value, first or second derivative at w = 1 and only the second (D) at w = 0, so the junction keeps its place and direction, the curvature there becomes the neighbour's, the rest of the spline stays as it was, and the eased part is a polynomial of the fifth degree, timed as a span. The largest of w² (1 − w)³ / 2 is 0.01728 (at w = 0.4), so the spline moves by at most 0.01728 dk l², and

```text
l = min( sqrt(tolerance / (0.01728 dk)),  9 · 1.05 · dk · v³ / (0.4 j),  half the span )
```

The second term is the length needed: at the junction the curvature changes per distance by 9 dk / l (the third derivative of w² (1 − w)³ / 2 at 0 is −9, and nowhere larger), and with the margin of the spans' limits that has to allow the speed v of the slower of the two pieces. The third keeps the via points where they are. Each junction gets its own l. The curvature overshoots on the eased part by up to 0.36 dk (the second derivative of w² (1 − w)³ / 2 falls to −0.36), and the change of curvature at the junction is about nine times that of a clothoid of the same length, so an eased junction is passed more slowly than a clothoid would allow: about 1.8 times for the same deviation (a clothoid between a spline and a line would need the spline moved as an arc is, which a spline cannot be without changing its via points). Example (`a_spline_eased_into_a_line_is_entered_at_the_hand_calculated_speed`): a span leaving a line with 10 1/m is eased over 24.06 mm at 0.1 mm and entered at 0.2189 m/s instead of 0.0707 m/s.

Limits: an arc with both ends fixed, or with one end fixed and an arc at the other, keeps its radius and gets no curve; where a curve would not fit (a wider arc moved in beside a tighter one that was not) both arcs stay as given; pieces too short for their curves halve L for the whole path, up to eight times, and then the path stays as given. What a curve takes of an arc is the angle from the arc's own end to where the curve meets it; curves that together take more than the arc has are too long, and so is a curve that turns half a circle or more (a tolerance that is large against a tight radius). The time-optimal planner times the same kind of path, with L and l worked out with the speeds its motors allow, so its curves can be longer than those of the plan within the axis limits, and its deviation (`path::deviation_time_optimal`) larger, up to the tolerance. One L still serves all curves between lines and arcs of a path: an arc whose own junctions would need less is moved in all the same by its e = L² / (24 R).

### As fast as the motors allow (`--time-optimal`)

The path the timing within the axis limits lays out (above, transition curves included, their lengths for the speeds the motors allow), timed point by point within what the motors allow. The jerk limits stay those of the axes, and so do the look-ahead over the junctions, the speed at a junction and the stop at a corner (`path::plan_time_optimal`).

**Places.** Each piece is read at places about 2 mm apart (at least 32 and at most 1000 per piece). With p′, p″ and p‴ the derivatives of the pose by the distance along the path there, the speed v and the acceleration a along the path give

```text
tool velocity      = v p′
tool acceleration  = a p′ + v² p″
tool jerk          = j p′ + 3 v a p″ + v³ p‴        (j: the jerk along the path)
```

and the rigid model gives each motor's torque as τ = c a + d v² + e: c per acceleration along the path, d per speed squared (the centripetal and Coriolis torque, `Machine::feedforward_torque_moving`, and the part of p″), e the torque that holds the tool there. With w the motor's angle per distance (central differences of the inverse kinematics) each place bounds:

```text
| w | v                                ≤  maximum speed (`drive.max_speed_rad_s`)
−T_b(| w | v)  ≤  c a + d v² + e      ≤  T_m(| w | v),   T(ω) = min(curve(ω), k_t I_drive) − (1 − profile.torque_reserve) · T(0)
3 | p″_i | v | a | + | p‴_i | v³       ≤  J_i − j_ramp | p′_i |      per axis i, J_i its jerk limit
```

Each motor has its own drive and motor (`[drive.<id>]`, `[motor.<id>]`, `[parts.<id>]`; before 14 November every motor had the shared `[drive]` here). T_m is what the curve gives in the direction the motor turns and T_b as it brakes, at the speed it turns: level up to the rated speed, then falling (linearly to the no-load speed, or as the d-q model has it with field weakening). The plan keeps back (1 − reserve) times the torque at a standstill at every speed, for friction and elasticity, which the rigid model leaves out and which do not shrink with the speed: up to the rated speed that is the reserve times the curve, as before; above it the room falls faster than the curve, and where the curve has less than that left the rigid torque has to stay as far on the braking side. Within a place the bound is taken at the highest speed of the range, as the curve does not rise (`MotorCurve`, read at 257 speeds up to the maximum and taken at the next one up). Before 14 November the plan stopped every motor at its rated speed with its full torque: no plan used the curve above it. The single axis now cruises a 1 m move at up to 398.8 rad/s, 420 − 106 · 0.76 / 3.8 (`the_time_optimal_plan_turns_a_motor_above_its_rated_speed_along_its_curve`). With `effects.drive_current_limit` the drive holds the current to `ratings.drive_max_current_a`, so T is also no more than the reserve times the torque of that current (`DriveParams::torque_of`, with saturation where it is on): a single axis whose drive gives 2 A to a 0.45 N m/A motor accelerates at (0.8 · 0.9 N m · i / r ∓ F) / m up and down a 30° incline (`the_drive_current_limits_the_time_optimal_acceleration_to_the_hand_calculated_one`: 26.186 and 32.590 m/s², 0.68049 s). Gravity counts for each direction as it is (before 13 November | e | was taken off both). The plan keeps 0.02 % of T back for what falls between two places. A place's **top speed** is the highest at which it holds all of its bounds without acceleration.

**Gearbox and pulley.** Where the machine reports a gearbox's output torque (`gearbox.<motor>.output_torque`) and the config rates it, `ratings.gearbox_max_torque_nm` (M2max) or `pulley_max_torque_nm` (the pulley sits on the output; the lower of the two counts), the output torque is a row too: the ratio i times what the motor gives less what turns the inertia at the gearbox's input (rotor and gearbox, J_in), with the motor's angular acceleration w a + w′ v² (w′ by second differences of the inverse kinematics along the path):

```text
−M  ≤  i ((c − J_in w) a + (d − J_in w′) v² + e)  ≤  M,      M = profile.torque_reserve · min(M2max, pulley rating)
```

The reserve is the motor's: the rigid model leaves out the output's friction and the elasticity's swing there as well. Gearbox losses are left out (a lossless output torque), as in the rest of the rigid model. Where it binds, a single axis accelerates at 0.8 · M / 0.085 N m per m/s² (`a_gearbox_or_pulley_rating_that_binds_limits_the_time_optimal_acceleration`: 94.118 m/s² at M2max 10 N m, 75.294 m/s² with the pulley rated 8 N m). On the shipped delta it binds (0.8 · 192 N m at the output against the motors' 0.8 · 15 N m · 20): its line across the range takes 0.434 s instead of 0.412 s, and each output torque stays within 0.8 · M2max (0.1 %). A holding torque beyond M is refused (`error-path-torque`).

**Guides.** Where the machine gives its guides' loads in the rigid model (`Machine::rigid_guide_loads`: a load at rest plus the size of a part linear in the tool's acceleration; the T-gantry and the single axis) and the config rates them (`ratings.guide_max_load_n`), each guide is a row with the same room both ways: | g · (a p′ + v² p″) | ≤ L − rest. On the T-gantry the X guide carries m_X g + | 2 m_Z a_x (z_com − z) / s | and the Z guide | m_Z a_x | (1 + | 2 (z_com − z) / s |); rated 150 N, a line along X at z = 0.35 m accelerates at (150 − 107.87) / 5.833 = 7.222 m/s² (`the_guide_load_rating_limits_the_time_optimal_acceleration`). A load at rest beyond the rating is refused (`error-path-guide`). No reserve: what friction adds to the acceleration the motors command (`commanded_ax`) is not in it.

**Jerk along the path.** On a line the ramps may use the axes' jerk limits in full: j_ramp = min over the axes of J_i / | p′_i |. On a curve they get 0.4 of the least of that over the piece, and the rest of each axis's limit is for the terms of speed and acceleration (the third bound), as `curved` shares it out for the plan within the axis limits.

**The speed curve.** Each piece gets a curve of the highest speed it may follow (`path::smoothed`, as squared speeds x at its places): it starts at the places' top speeds and is only lowered, in sweeps forwards and backwards, until its acceleration x′ / 2 is within 0.9 of what the places allow at its speed (where a slow place would come too fast to brake for, the curve comes down to it in time) and its jerk, v times the change of x′ / 2 per metre, within half of j_ramp (a peak too sharp is lowered onto the bound, a dip too sharp gets its sides lowered). It never falls below the slowest place's top. The rest of the acceleration and of the jerk is for a run to catch up with the curve and follow its bends (`the_speed_curve_comes_down_to_a_slow_place_within_its_acceleration_and_jerk`: from a place at 0.5 m/s among places at 1 m/s, 10 m/s² and 10 mm apart, x rises 0.25, 0.43, 0.61, 0.79, 0.97, 1 m²/s²).

**The runs.** Between the speeds at its junctions a piece runs: two cycles steady where a junction is passed at speed, a run up from its start to its top speed, cruise, a run down to its end (`path::follow`; the run down is planned from the end backwards under the bounds for braking). A run is integrated in steps of a quarter of the profile cycle at a constant jerk per step. Each step the acceleration rises at j_ramp as far as three things allow: the most acceleration the places allow ahead (worked out at 32 speed levels between zero and the curve's highest, each torque row at its worst speed up to the level, the jerk bound at the level; the run takes the level of the highest speed it reaches if its acceleration falls to zero from there, v + a² / (2 j_ramp); lowered ahead of where it falls by j_ramp times the least time between two places, so that the jerk can follow it); that it can still fall to zero just as the speed reaches the top speed ((a + Δa)² ≤ 2 j_ramp (top − v)); and that it can still come down onto the speed curve with the jerk left beside the curve's own (y² ≤ 2 k g for the acceleration y above the curve's and the gap g to it), unless falling to zero at once keeps the speed below the curve all the way. So a run rises along the most acceleration, meets the speed curve, follows it up and down, and goes on to the top speed; until it brakes for the top speed it is the same run whatever the top speed. On a line with constant limits this is the jerk-limited ramp of the plan within the axis limits, to a fraction of a step.

**The top speed** of a piece is the highest, up to the curve's highest, for which both runs reach it before they meet and the cruise between them holds at every place it passes, found by halving to 1e-9 of it. A higher top that fits is the quicker, since the runs are the same until they brake for it. A junction is passed no faster than the slowest place of either piece, so that the look-ahead always has a speed that fits.

So every planned pose keeps each motor's torque from the rigid model within its curve less what the reserve keeps back, each motor's speed within its maximum speed, each rated gearbox, pulley and guide within its rating, and each axis's jerk within its limit. The tests check that on the planned poses by differences over one cycle, with a margin of 0.1 %. Planning a path takes 2 to 21 ms (release build; see the table in PROGRESS.md), against 0.2 to 1 ms with one limit per piece.

Where the holding torque alone is above the reserve at a place of the path, the path is refused (`error-path-torque`).

**Belt tension as a limit.** Where the config rates the belt (`ratings.belt_max_tension_n`, `belt_min_tension_n`), the rigid model also gives each belt section's tension at each place (`Machine::rigid_tensions`). It is linear in the acceleration, T = c + g · acceleration: the belt pulls the moved masses (not pulleys and rotors) and keeps its length, Σ (T_i − P) L_i = 0 over the geometric section lengths (the elastic law to first order in T / EA). T-gantry, with F_x = (m_X + idlers + chain) a_x and F_z = (m_Z + idler) a_z + m_Z g:

```text
d1 = −((F_x + F_z) L2 / 2 + F_x L3) / (L1 + L2 + L3)      d3 = d1 + F_x      d2 = d1 + (F_x + F_z) / 2      T_i = P + d_i
```

Single axis, F = (m + chain)(a + g sin incline): T1 = P − F L2 / (L1 + L2), T2 = P + F L1 / (L1 + L2). Each section and place is one more row of the bounds, with the same room on both sides (the belt's vibration after a change of tension swings as far the other way, below):

```text
| g · (a p′ + v² p″) |  ≤  profile.tension_reserve · min( T_max − m − c,  c − T_min − m ),   m = profile.tension_margin_n
```

The margin m and the share are for what the rigid model leaves out. The margin is the guide friction the belt pulls, roughly absolute (coulomb plus viscous at top speed: 31 N on the default T-gantry). The share (0.5) is for the belt's vibration: a change of acceleration Δa in a ramp of time T_r on a lightly damped mode of angular frequency ω leaves a swing of g Δa |sin(ω T_r / 2) / (ω T_r / 2)|, up to g Δa itself when the ramp is quick against the period, so the full model's dynamic tension is up to twice the rigid one. A share of 0.8 without a margin (until 13 November 2026) let the full model pass the rating: 331.4 N on a 330 N rating at the default 100 m/s³, 405.5 N on 400 N at 2000 m/s³, and the slack side reached 0 N (−117 N with `belt_slack` off: the belt rings at about 18 Hz after a corner). With 0.5 and the margin the full model kept both ratings on every path, rating (330 to 1500 N) and jerk (100 to 5000 m/s³) tried on the default T-gantry, 40 m/s² and 2 m/s (`the_full_model_keeps_the_belt_ratings_on_a_time_optimal_plan_at_high_jerk`; `the_time_optimal_plan_keeps_the_belt_within_its_tension_limit`: 6.328 m/s² at the line's far end by hand, 7.02 m/s² where it accelerates, each taken where it is, 358 N on the full model; without the limit 19.3 m/s² and 423 N). That is a judgement backed by those runs, not a proof: several quick ramps one after another can add their swings. A path on which the holding tension alone is beyond a rating less the margin is refused (`error-path-tension`). The shipped examples are not limited by the belt at their axis limits; the user guide's corner at 2000 m/s³ is (0.78 s → 0.80 s plus rest with one limit per piece, 0.79 s point by point; it went slack before).

Assumptions and limits: the rigid model, so no friction and no elasticity (that is what the margin and the share are for); the velocity and acceleration limits of the axes play no part; the plan is not proven the fastest one possible: the two runs of a piece meet on a cruise at no acceleration, so where they meet while the speed curve still rises or falls the top is a little below where they would meet; the speed curve keeps 0.1 of the acceleration and half of the jerk for catching up with it, and its sweeps stop at a change of 1e-7 of it (at most 300); a run's acceleration bound is taken at the worst speed up to its speed level (32 levels); on a curve the ramps get 0.4 of the jerk and the jerk's three terms are bounded each by its size; the bounds are read at places 2 mm apart and taken as straight between them, with 0.02 % kept back; without a path tolerance it crawls into an arc as the plan within the axis limits does; gearbox losses and the DC link are not in the plan (the belt's tension, the drive's current, the gearbox's and pulley's torque and the guides' load are, above); the payload is as the machine has it when planning. Until 14 November 2026 a piece cruised at one speed between two ramps (where its speed limit rose and fell between them, at the lowest of it), passed a place slower than its top at no more than 0.9 of that place's speed, took the quickest of 16 + 16 top speeds, read each ramp's bounds at its worst speed up to the top, and took the lowest of three neighbouring places; the drive's current, the gearbox, the pulley and the guides were judged after planning only. The user guide's spline example then took 1.380 s (1.349 s now), the T-gantry line at 300 m/s³ and a 400 N belt 0.851693 s (0.851565 s now), the delta's line across its range 0.411 s, its motors' 0.8 · 15 N m through the ratio 20 allowed up to 240 N m on gearboxes rated 192 N m M2max (0.434 s now). Until 13 November 2026 the plan had one speed and one acceleration limit per piece, from nine poses, shared out 3 : 4 : 5 on arcs: the line, arc, line of the user guide took 1.429 s (1.395 s now), its spline example 1.674 s (1.381 s now), a line across the delta's range 0.429 s (0.415 s now). Until 12 November 2026 this was the classic time-optimal parameterisation on a grid of points, without a jerk limit and smoothed by a moving mean (`profile.path_smoothing_s`, which no longer exists). It was faster (0.69 s against 1.43 s on the line, arc, line of the user guide) and passed the motors' limits on arcs, many times over on short ones.

## Input shaper

Source: `shape` in [`control.rs`](../crates/sim-components/src/control.rs).

`effects.input_shaping`: a ZVD shaper on the position, velocity and acceleration setpoints of all axes. The shaped setpoint is the sum of three copies of the plan: now, half a damped period ago and a whole period ago. With `profile.shaper2_frequency_hz` above 0 a second ZVD shaper (its damping `shaper2_damping`) is convolved with the first: nine copies, at the sums of the two shapers' delays with the products of their weights. For an undamped mode at f a ZVD shaper at f_s with no damping leaves |H| = cos²(π f / (2 f_s)) of the vibration, two convolved the product of theirs.

```text
K       = exp( −ζ · π / sqrt(1 − ζ²) )
T_half  = 1 / ( 2 · f · sqrt(1 − ζ²) )
shaped(t) = ( plan(t) + 2K · plan(t − T_half) + K² · plan(t − 2 T_half) ) / (1 + K)²
```

f `shaper_frequency_hz`, ζ `shaper_damping`. The delays are interpolated between profile cycles. The weights add up to one, so once the plan has stood still for a whole period the setpoint is the plan exactly. Where a cycle segment has a dwell or switches the payload, both wait until the shaped setpoint has arrived, ceil(2 · T_half / cycle) profile cycles after the plan (with a second shaper ceil(2 · (T_half + T_half,2) / cycle)); the process force starts with the dwell. Segments without a dwell and without a gripper action go on with the plan. A plain move (`Move`) is no longer busy when its plan has arrived.

With `effects.shaper_follows_pose` the half period of the first shaper is that of the resonance the machine reports for the planned pose, and that of a second shaper (`shaper2_frequency_hz` above 0) the machine's second mode there (`Machine::modes`); one the machine does not report stays at its frequency. The T-gantry reports two modes, the single axis one. `sim-cli modes` and the app's tuning panel show them over an axis's stroke. Its resonance also depends on the payload: when the gripper closes at the start of a dwell it drops (on the delta D3 from 41.5 to 27.2 Hz). While the plan stands still the period stays the one it had on arrival, so a setpoint that has arrived stays on the target (the longer period reached back into the lift and took it 0.1 mm off); the next move starts with the new period (on the D3 the standstill, 25 ms of shaper lag and the 35 ms dwell, is longer than its 37 ms).

A changed shaper setting in a running simulation (the switch, the frequency, the damping, following the pose) is held back until the plan stands still and the shaper's store holds nothing but that pose. Then the setpoint is the plan with the old shaper and with the new one, and does not step. Taken at speed, the three copies of the plan were suddenly read at other delays, or two of them dropped, and the setpoint jumped by up to 16 mm on the T-gantry's X axis at 0.8 m/s.

When `profile.cycle_s` is changed in a running simulation, the stored plan of the shaper is read off again at the new spacing and every stored cycle count of the motion under way is converted, so the setpoint does not jump. The fieldbus queue keeps its count of cycles.

Limits: two resonances at most; the same shapers on every axis.

## Analysis quantities

Source: [`analysis.rs`](../crates/sim-components/src/analysis.rs).

**What is measured.** For a cycle: the simulation starts the cycle and measures its second pass, from the moment the first segment starts for the second time to the moment it starts for the third time. If a drive trips on DC link overvoltage before that, the drives are off and the cycle never gets there: the figures are then those of the run from its start to the trip, `duration` is that time, and the load is marked as tripped. For a path or trajectory: from its start to 0.3 s after its planned end.

Per motor, summed over every base step of the measured pass (N steps):

| Quantity | Definition |
| --- | --- |
| Peak torque | max \|τ\|. τ is the drive's torque from the winding current, before ripple and shaft friction |
| RMS torque | sqrt( Σ τ² / N ) |
| Cubic-mean torque | cbrt( Σ \|τ\|³ · \|ω\| / Σ \|ω\| ): weighted with the speed, so that a turn under load counts and standing still does not |
| Peak speed | max \|ω\| of the motor shaft |
| Utilisation | max of \|τ_demand\| / (torque the [torque-speed curve](#torque-speed-curve) leaves at that speed in the direction of τ_demand). τ_demand is the velocity loop's output after its filters and before the torque limit. The delivered torque is clamped to the curve and could never exceed it |
| Thermal utilisation | RMS torque / `drive.rated_torque_nm`. Not judged; `motor-thermal` judges the winding rise |
| RMS current | sqrt( Σ i² / N ) |
| Peak current | max \|i\| |
| Winding rise (settled) | With q = ϑ_rated / (1 + 0.0039 ϑ_rated) · (I_rms / I_rated)²: ϑ = q / (1 − 0.0039 q). The copper loss with the resistance at that temperature, against the loss at the rated current `motor.rated_current_a`. Infinite if 0.0039 q ≥ 1 |
| Energy in | Σ P · dt over the steps with P ≥ 0 |
| Energy regenerated | −Σ P · dt over the steps with P < 0 |
| Peak regenerated power | max(−P) |
| Largest regeneration | Most energy fed back in one unbroken stretch of negative power |
| Resistor energy, peak power | Σ and max of the simulated braking resistor's power (0 without `effects.dc_link`) |
| DC link peak voltage | max of the DC link voltage |
| Mains energy | With `effects.regenerative_supply`: Σ of the power fed back into the mains, after the line module's efficiency (`analyze` column `mains_energy_j`) |
| Line module energy, peak power | With `effects.regenerative_supply`: Σ and max of the power through the line module either way, drawn from the line plus fed back (before its efficiency) |
| Mean speed | Σ \|ω\| / N, for the lives in hours |
| Gearbox life torque | Where the machine reports `gearbox.<motor>.output_torque`: ( Σ \|τ_out\|^(10/3) · \|ω\| / Σ \|ω\| )^0.3, Siemens' equivalent torque M2eq of its planet bearings' life |
| Output torque per speed band | Where the machine reports the output torque: its peak in each band of 5 rad/s of \|ω\| (`SPEED_BAND`), for the belt's tooth load |
| Radial load, equivalent | Where the machine reports `pulley.<motor>.radial_load`: cbrt( Σ F³ · \|ω\| / Σ \|ω\| ), sampled once per millisecond: the equivalent load P of a bearing whose load and speed change (ISO 281) |

With a shared DC link every motor reports the link's resistor, rectifier and line module, as above.

For the machine, sampled once per millisecond:

| Quantity | Definition |
| --- | --- |
| Belt tension, lowest and highest | Over all signals `belt.<n>.tension` |
| Belt slack time | Increase of `belt.slack_time` over the pass. Counted every integration step, so it also catches what falls between the samples |
| Radial load peak | max over all signals `pulley.<id>.radial_load` |
| Guide load | Per guide the peak and the cubic mean over time, cbrt(Σ load³ / samples). Reported for the guide with the highest cubic mean |
| Guide life | L10 = (C / P)³ · 50 km with C = `ratings.guide_dynamic_load_rating_n` and P that cubic mean |
| Guide speed, guide life in hours | The mean \|speed\| of that guide's axis (the tool's speed along it, once per millisecond); L10 · 1000 / (v · 3600) h |
| Path error | Distance from the tool position to the polyline of the setpoints of the last 0.5 s (the nearest of its pieces). Peak and RMS. The tool position is the machine's tool state: on the T-gantry the tool tip over the floor, frame deflection included |
| Tracking error | Distance from the tool to the setpoint of the same moment. Peak |
| Dwell error | In the second half of a dwell: distance from the tool to the planned target. Peak. With input shaping the dwell starts when the shaped setpoint has arrived |

Notes:

- Path error looks only at the setpoints of the last half second: the piece of the path the tool can be near.
- The setpoints used are the controller's (after shaping, before the fieldbus delay), so a fieldbus delay shows in the tracking error.
- The lives (`pulley_bearing_life_h`, `gearbox_life_h`, `gearbox_gearing_life_h`, `motor_bearing_life_h`, `guide_life_h` in `analysis.rs`):

  | Life | Formula | Source |
  | --- | --- | --- |
  | Pulley shaft bearing | L10h = (C / P)³ · 10⁶ / (60 n): C = `ratings.pulley_bearing_load_rating_n`, P the equivalent radial load above, n the pulley's mean speed in r/min (the motor's over `gearbox.ratio`) | ISO 281 basic rating life of a ball bearing (as SKF and Schaeffler print it); Siemens uses the same form for its gearboxes' output bearings, L10h = 30000 h · (F_R,eq allowed / F_R,eq)³ |
  | Pulley shaft bearing, a_ISO | L_nm = a1 · a_ISO · L10. a_ISO = 0.1 · [1 − x · (e_C · C_u / P)^(1/3)]^−9.3, at most 50, x = (2.5671 − 2.2649 / κ^0.054381)^0.83 for 0.1 ≤ κ < 0.4, (2.5671 − 1.9987 / κ^0.19087)^0.83 for 0.4 ≤ κ < 1, (2.5671 − 1.9987 / κ^0.071739)^0.83 for 1 ≤ κ ≤ 4 (κ above 4 as 4, below 0.1 as 0.1: a shortcut, the standard gives none there). κ = ν / ν1, ν1 = 45000 · n^−0.83 · D_pw^−0.5 below 1000 r/min and 4500 · n^−0.5 · D_pw^−0.5 from 1000 r/min (mm²/s, n in r/min, D_pw in mm). ν = `ratings.bearing_lubricant_viscosity_mm2_s`, e_C = `bearing_contamination_factor`, C_u = `pulley_bearing_fatigue_limit_n`, D_pw = `pulley_bearing_mean_diameter_mm`; κ at the pulley's mean speed. Without all four, a_ISO = 1 | ISO 281:2007, sections 6.3.3.2 and 6.3.3.3 (radial ball bearings), Table 13 for e_C; C_u ≈ C0 / 22 for ball bearings below 100 mm (its Annex B) where the maker gives none |
  | Gearbox planet bearings | L10h = 20000 h · (n2B / n2) · (M2N / M2eq)^(10/3): M2N = `ratings.gearbox_rated_torque_nm`, n2B = `ratings.gearbox_reference_speed_rpm`, n2 the output's mean speed, M2eq the gearbox life torque above | Siemens, configuration manual "Planetary gearboxes for S-1FK2 and S-1FT2 servomotors", 01/2023, section 6.3.3 |
  | Gearbox gearing | Palmgren–Miner: life = 1 / Σ (n_i / N_i) h over the cycle's torque spectrum above M2N, n_i the output revolutions per hour in torque band i (1 % wide from 0.1 N m, each at its upper torque; `MotorLoad::gearbox_torque_spectrum`, every step), N_i = 30000 · (M2max / M_i)^k; at or below M2N no damage, and no figure where the cycle never exceeds it. M2max = `ratings.gearbox_max_torque_nm`, k = `ratings.gearbox_gearing_wohler_exponent` (8.7, an estimate: the tooth root's S-N slope of case-hardened gears in ISO 6336-3, ln(3000) / ln(2.5)) | Siemens' two figures: M2N continuous, M2max for at most 30000 output revolutions (equipment manual 04/2025, chapter 6). Siemens' own damage sum is in its configuration manual, not in the repository: this is the textbook method on its figures, an estimate |
  | Motor bearings | L10h = (C / P)³ · 10⁶ / (60 n) · a1 with C = `ratings.motor_bearing_load_rating_n`, P the pulley shaft's equivalent radial load, n the motor's mean speed; only where the pulley sits on the motor shaft (the key set, and `gearbox.ratio` 1). Behind a gearbox the gearbox's output bearing takes the belt's pull and the motor's bearings carry none. C from Siemens' allowable radial force F for 25000 h at a mean speed n: C = F · (25000 · 60 · n / 10⁶)^(1/3), e.g. the 1FK2205's 1000 N at 1500 r/min and 20 mm from the shoulder, 13104 N (`catalog/siemens/motor_bearings_1fk2.csv`) | Siemens, S210 operating instructions 09/2022, sections 11.1.1 (about 25000 h at medium load) and 12.1.12 (the diagrams) |
  | Guide | L10 in km over the mean speed of its axis | The catalogue formula above |

  One bearing per pulley shaft takes all of the radial load (a config can fold a pair into one C, as the EXCT-30's does); no axial load. The bearing lives are multiplied by the life factor a1 of ISO 281 for `ratings.bearing_reliability_percent` R: a1 = 0.95 · (ln(100 / R) / ln(100 / 90))^(2/3) + 0.05, the Weibull form behind the standard's table (1 at 90 %, 0.64 at 95 %, 0.25 at 99 %); R from 90 to 99.95 %, others refused by `verdicts`. The guide life is not, its makers use factors of their own. a_ISO is on the pulley shaft bearing only: Siemens' figures for the planet bearings (20000 h) and the motor bearings (25000 h) are for their own lubrication already. The motors' own bearings have no figure in any shipped config: the belt does not pull on a motor shaft there (a gearbox, or Festo's drive head with a coupling, is between). The gearing's figure appears on the delta's first arm only (about 11600 h: its NRB080 runs up to 134 N m against 120 N m rated); the belt machines' cycles stay below M2N.
- The belt's teeth (`belt_tooth_utilisation`): no belt maker publishes a life in hours or load cycles, or a Wöhler curve, for polyurethane steel-cord belts (Megadyne's MEGALINEAR handbook, Gates' polyurethane belt catalogue and BRECO's data sheets were read; Optibelt's could not be reached). They size by the tooth load and the cord tension with a service factor, so this is a rating, not a life: per motor the highest over the cycle of τ_out / r · c_s / (F_spec(n) · b / 10 · min(z_m, 12)), with r = `geometry.pulley_radius_m`, c_s = `ratings.belt_service_factor`, F_spec the allowed specific tooth load at the pulley's speed n (`ratings.belt_tooth_load_n_cm` over `belt_tooth_load_rpm`, interpolated, held beyond the ends; each speed band taken at its upper speed), b = `ratings.belt_width_mm`, z_m = `ratings.belt_teeth_in_mesh`. Megadyne's method (handbook 02.2026, page 9). The cords are judged by `belt-tension`.
- The guide life is the catalogue formula and nothing more: no load factors for shock, temperature or hardness, no moments on single blocks, the mean taken over time and not over distance, and a guide load that is itself an estimate from the rigid model. It is not validated.

## Verdicts

Source: `verdicts` in [`analysis.rs`](../crates/sim-components/src/analysis.rs). Each verdict is a value against a limit; it passes if value ≤ limit, except where noted. A rating that the config leaves out gives no verdict.

Per motor:

| Check | Value | Limit |
| --- | --- | --- |
| `motor-peak` | Utilisation | 1 |
| `motor-thermal` | Settled winding rise | `motor.rated_temperature_rise_k` |
| `motor-speed` | Peak motor speed in rad/s, where the config gives a maximum speed (a catalogue motor does) | `drive.max_speed_rad_s` |
| `drive-current` | RMS current | `ratings.drive_rated_current_a`: the drive's rated output current (a drive by name sets it). The cycle's RMS against the rated current: the power unit warms in seconds to minutes, about as long as a cycle |
| `drive-peak-current` | Peak current; with `effects.drive_current_limit` the current the velocity loop asks for where that is higher (see [drive current limit](#drive-current-limit)) | `ratings.drive_max_current_a`: the drive's maximum output current. With the effect the simulated current stays within it, and a cycle that needs more lags and fails here; without, the motor's maximum current stays the limit of the run |
| `pulley-torque` | Peak torque at the gearbox output | `ratings.pulley_max_torque_nm` |
| `gearbox-continuous` | Cubic mean of the torque at the gearbox output, weighted with the speed | `ratings.gearbox_rated_torque_nm` |
| `gearbox-input-speed` | Peak speed in r/min | `ratings.gearbox_max_input_speed_rpm` |
| `gearbox-peak` | Peak torque at the gearbox output | `ratings.gearbox_max_torque_nm` |
| `dc-link-energy` | See below | room = ½ C (U_th² − U_bus²). A threshold at or below U_bus is refused as out of range |
| `braking-resistor-continuous` | See below | `ratings.braking_resistor_continuous_w` |
| `braking-resistor-peak` | See below | `ratings.braking_resistor_peak_w` |
| `dc-link-overvoltage` | Peak DC link voltage; only with `effects.dc_link` | `ratings.dc_link_overvoltage_v` |
| `line-module-continuous` | Line module energy / cycle time: its mean power either way; only with `effects.regenerative_supply` | `ratings.line_module_rated_power_w` |
| `line-module-peak` | Line module peak power either way; only with `effects.regenerative_supply` | `ratings.line_module_max_power_w` |
| `pulley-bearing-life` | Rating life of the pulley shaft bearing, h. Passes at or **above** the limit | `ratings.required_life_h` |
| `gearbox-life` | Life of the gearbox's planet bearings, h; needs `gearbox_rated_torque_nm` and `gearbox_reference_speed_rpm`. Passes at or **above** the limit | `ratings.required_life_h` |
| `belt-tooth-load` | The belt's tooth load over its rating (see [analysis quantities](#analysis-quantities)) | 1 |

The braking checks need `dc_link_capacitance_uf` and `braking_threshold_v`. They come in two forms:

| | Without `effects.dc_link` (estimated) | With it (simulated) |
| --- | --- | --- |
| `dc-link-energy` value | Largest regeneration | ½ C (U_peak² − U_bus²) |
| Resistor needed | Largest regeneration > room | The resistor burned any energy |
| `braking-resistor-continuous` value | All regenerated energy / cycle time | Resistor energy / cycle time |
| `braking-resistor-peak` value | Peak regenerated power | Peak resistor power |

`dc-link-energy` passes if the resistor is not needed, or if the config has a continuous resistor rating to fall back on. The two resistor checks appear only when the resistor is needed. The estimate is the cautious one: it sends all regenerated energy through the resistor, where the simulated capacitors hand part of it back to the next acceleration.

For the machine:

| Check | Value | Limit |
| --- | --- | --- |
| `settling` | Dwell error peak | `ratings.settling_tolerance_m` |
| `belt-tension` | Highest belt tension | `ratings.belt_max_tension_n` |
| `belt-slack` | Lowest belt tension | Passes if it is **above** `ratings.belt_min_tension_n` and no section spent any time slack |
| `gearbox-radial-load` | Radial load peak | `ratings.shaft_max_radial_load_n` |
| `guide-load` | Peak load of the reported guide | `ratings.guide_max_load_n` |
| `guide-life` | Guide life in hours. Passes at or **above** the limit | `ratings.required_life_h` |
| `belt-tooth-jump` | With `effects.belt_tooth_jump`: how often the belt's teeth jumped over a driven pulley in the cycle (see [belt tooth jump](#belt-tooth-jump)) | 0 |
| `belt-span-resonance` | With `effects.belt_span_vibration`: the lowest distance of a span's first sideways mode from an excitation at steady speed, as a share of the mode (see [belt spans](#sideways-vibration-of-the-belt-spans)). Passes at or **above** the limit | `ratings.belt_span_resonance_band` |
| `gearbox-thermal` | With `effects.part_heating`: where the hottest gearbox settles on the cycle, R_th × its mean loss (`CycleLoad::part_rises`, see [thermal model](#thermal-model-and-view)); not judged on a direct drive, which has no gearbox | `thermal.gearbox_max_rise_k` |
| `guide-thermal` | With `effects.part_heating`: where the hottest guide settles on the cycle | `thermal.guide_max_rise_k` |
| `thermal-warm-start` | With `effects.thermal_warm_start`, where something that warms acts back on the run: the most a rise the measured pass started at lies from the one it settles at, K (see [thermal warm start](#thermal-warm-start)). No row: the pass started cold, or the change has no value (a run that stopped, a winding that does not settle), whose cause fails its own row (the trip's, `collision`, `motor-thermal`) | 0.1 K (`WARM_START_TOLERANCE_K`) |
| `collision` | Where the config has obstacles: the least clearance to them over the run, m (see [obstacles and crash](#obstacles-and-crash)). Passes at or **above** the limit, never at a contact (clearance 0 or less). Without obstacles it is not judged, so it cannot fail | `profile.min_clearance_m` |

Without `effects.part_heating` (or `effects.thermal_feedback`, which warms the parts too) the two thermal checks are not judged, as a rating left out is not.

Gearbox and pulley torques are those at the gearbox output, the signals `gearbox.<motor>.output_torque`, looked at every simulation step: peak, and the cubic mean cbrt( Σ |τ_out|³ · |ω| / Σ |ω| ). The rotor's own acceleration torque is not in them (it never passes through the gearbox), and the efficiency is: driving, the output carries η times what the gear's spring transmits, driven back 1/η times. The motor torque times the ratio, which these checks used before, reads too high while the motor accelerates its own rotor and too low while the load drives the gearbox back. On a machine that does not report the signal the three checks are not made. In the elastic model the peak includes the ringing of the gear's spring and the knocks of its backlash.

## Obstacles and crash

Source: [`collision.rs`](../crates/sim-components/src/collision.rs), `Sim::check_obstacles` in [`sim.rs`](../crates/sim-components/src/sim.rs). The config's `obstacles` are boxes with their edges along the axes of the 3D view. What can hit one is `Machine::collision_bodies`: boxes of the machine's scene at its state. By default the tool alone, as a point where the scene draws it; the T-gantry gives its Z axis, flange and payload, the single axis its carriage and payload, the D3 its platform and payload (the forearms are left out). The payload counts only while it is attached.

- **Clearance** between two boxes: the distance of their nearest points; where they overlap, minus the depth of the overlap along the axis where it is least. A contact is a clearance of 0 or less. The clearance of the machine is the least over its parts and the obstacles.
- **Before a run** (`check_path`, `check_cycle`): the path is followed straight from pose to pose, at least every millimetre, with the machine at rest at each pose (`Machine::equilibrium`). The cycle goes from the start pose to its first target with the payload attached, then target to target with the payload as the segment before left it, and round from the last target to the first; a segment with via points follows the path the planner times through them. Corner blending is not followed: it cuts the corners, and the run is checked anyway. Reported are the least clearance and the first pose in contact.
- **While it runs**: after every base step. The simulation keeps the clearance now and the least so far (`Sim::clearance`, `least_clearance`), marks the obstacles touched, and records the first contact: the time at the end of the step that found it, the tool's pose and its speed (the length of its speed vector) then, and the obstacle.
- **The check** `collision` judges the run's least clearance (the way to the cycle and its first pass included) against `profile.min_clearance_m`. That key was the optimiser's clearance over pick and place; it is now the one clearance the machine keeps, from the obstacles as well. A time-optimal path that comes nearer to an obstacle than it, or touches one, is refused (`error-path-collision`); a path planned within the axis limits is planned all the same and reported. The cycle optimiser judges every candidate by the verdicts, so it takes no lower lift that comes too near, and lowers each lift no further than its traverse keeps the clearance (see Optimiser below).
- **The crash** (`effects.crash`): the obstacles are rigid. A part that touches one and moves further into it (its clearance less than after the base step before) is stopped by a perfectly plastic impact, with the drives on or off: the machine is put at rest (its equilibrium, belts and gears holding what they hold at rest) at the pose along the step where the clearance reached 0, taken linearly between the tool's poses before and after the step, so at the face where the parts move straight. Drives that were on go off, as a real drive trips on the following error of a blocked axis; holding brakes engage. A part in contact that stands or moves out is left to move: drives enabled again take the machine out of the obstacle, and a move further in stops it and trips them again (`Sim::crashed`; a PLCopen axis shows ErrorID 8). A machine coasting or falling with its drives off (after a crash, a trip or `--disable-at`, without a holding brake) stops at the face and rests there. A stiff contact spring would need a stiffness, a damping and a step small enough for them, and would bounce the tool back into a drive still pushing; the stop needs none of these. The energy it takes (kinetic, and what springs held beyond their rest) is booked as the budget's `crash` row (`Sim::crash_energy`), and the bleeders of the drives switched off then as its `discharge` row; the budget of the move into a box closes as the others do. Off, contacts are still found and reported, and the machine moves through the obstacle.

Hand calculations in [`sim-machines/tests/collision.rs`](../crates/sim-machines/tests/collision.rs): a box beside a straight move of the single axis keeps 0.13 m, planned and run; a move into a box at 1 m/s stops at its face at x = 0.47 with an impact speed of 1 m/s; a box under the T-gantry's traverse keeps 0.048 m, which fails at the default 0.3 m and passes at 0.04 m.

## Picking from a conveyor

Source: [`conveyor.rs`](../crates/sim-components/src/conveyor.rs), the conveyor cycle in [`control.rs`](../crates/sim-components/src/control.rs). With a [`[conveyor]`](config-reference.md#conveyor) table the cycle picks parts from a moving belt, one pass per part, on any machine whose tool axes are Cartesian positions; it knows the machine through its tool pose, axis limits and range alone.

- **The belt and its parts**: the belt has run S(t) = v t from the start, or v t² / (2 t_r) during a ramp of t_r (then v (t − t_r / 2)); part k passes the origin at its arrival time and is S(t) − S(arrival) along the belt from there, its grip pose the origin plus that times the belt's direction.
- **A move of a pass** is a superposition: a jerk-limited straight move (`profile::Move`, as a cycle segment's) from where the phase starts to a point Q, plus a belt term whose velocity blends from the start velocity va to the end velocity vb as va + (vb − va) h(u), h(u) = 3u² − 2u³, u = t / T. The blend has no acceleration at either end, so the phases join without a jump in acceleration; it covers T (va + vb) / 2. A target that is at P now and moves with vb is met at T, with its velocity, where Q = P + T (vb − va) / 2.
- **The intercept**: the move is planned with each axis's limits less what the belt term takes: max(|va|, |vb|) of the speed, 1.5 |vb − va| / T of the acceleration (the blend's peak, at u = 1/2) and 6 |vb − va| / T² of the jerk (at its ends). Since these and Q depend on T, T is the shortest time in which the move to Q fits into T, found by bisection (60 halvings from a bracket found by doubling); with va = vb at once, the move's own time. For a trapezoidal move with cruise it is the root of a α T² − (α c + a β + v′) T + β c = 0 (α = 1 − v / (2 v′), β = D / v′, c = 1.5 v, v′ = v_max − v), which the test checks. The sum of move and belt term keeps the axis limits; as the reservations hold over the whole move, it may use less than them: a faster belt makes the moves gentler and longer, and so need not raise the RMS torque (the validation has an 11 % fall).
- **A pass**, from rest above the place (or the start pose): to above the part (va = 0, vb = the belt's velocity), down onto it, the grip (`grip_s`) and up (all with the belt's velocity, the move only along the lift), to above the place (vb = the place's velocity), down, the release (`release_s`), up, and from a moving place to rest above it. The payload is attached at the end of the grip and let go at the end of the release.
- **Waiting and missing**: the passes start once the belt has its speed. The first part not yet planned for is taken; where the grip would begin before the window, the pass starts later, at the time found by bisection at which it begins at the window's start; on a belt that stands, not before the part has arrived. Where the grip would end beyond the window, or a pose of the pass (every 5 ms) leaves the machine's range, the part is missed and the next is taken. After a thousand misses in a row the cycle ends, and the analysis says that no part can be picked (`error-conveyor-unreachable`).
- **Results** (`CycleLoad::conveyor`): the measured pass is one pick, so the picks a minute are 60 / its time; a machine faster than the parts waits, and its passes come a pitch apart. The tracking error is the most the tool's measured pose, taken along the belt, was from the part's during the grip. The parts missed a minute are those the belt brings beyond the picks a minute, or those missed in the measured pass, whichever is more: one pass cannot see misses that come in bursts. The verdicts are `pick-tracking` (against `conveyor.tracking_tolerance_m`) and `parts-missed` (against 0).
- **Obstacles**: the passes are planned as the parts come, so `check_cycle` does not check them before the run; the live check (see above) judges the run, and `collision` fails where a pass comes too near.
- **The scene** draws the belt as a band under the grip poses (300 mm wide), slats every 100 mm that run with the belt, and each part on it that is not in the gripper as a 50 mm cube.

Shortcuts: a pass starts only once the belt has its speed (a pick during the ramp would need the part's acceleration in the intercept); the gripper acts on the plan's time, not once a shaped setpoint has arrived, so with input shaping the grip begins up to a shaper period early; the reserved limits are conservative (the moves are slower than a planner that shares the limits along the move would make them); the second belt carries no parts in the scene, and its slot is wherever the place pose is when the tool leaves for it.

## Other analyses

| Analysis | Definition |
| --- | --- |
| Error bands | The cycle is run with the config as it is and with each parameter of `[bounds]` at its low and at its high value, one at a time. A bound on a value that only an effect that is off uses (`[thermal]` without `effects.part_heating` or `thermal_feedback`, `[thermal_feedback]` without `effects.thermal_feedback`, `belt.tooth_jump_factor` without `effects.belt_tooth_jump`, `ratings.line_module_efficiency` without `effects.regenerative_supply`), or that no cycle uses (`ratings.dc_link_discharge_time_constant_s`, which acts only while the drives are disabled), gets no runs, here and in the sensitivity. A band is the envelope of the verdict's value over all runs; the worst case passes only if every run passes. The runs share the processor cores |
| Sampled error bands | `--bands --samples N`: N runs with every parameter of `[bounds]` between its bounds at once, as a Latin hypercube: the range of each parameter is cut into N equal parts, each part is used in exactly one run at a random place inside it, and each parameter has its parts in a random order of its own. Random numbers from the simulator's own generator and a seed, so a call repeats. Reported per check: the envelope, the worst case, and the share of the runs (the config as it is among them) that pass; a run in which a check does not come up has passed it. It finds what two or more values do together; it does not put a value exactly on a bound, and N runs do not find the worst of the 2ⁿ corners |
| Worst corner | `--bands --worst` (`analysis::verdict_worst`): per verdict a search among the 2ⁿ corners. (1) The one-at-a-time runs above. (2) The corner where each parameter is at the bound that made this verdict worse alone (at its high bound where it moved the verdict not at all); for a verdict that is monotone in each parameter and whose effects do not depend on each other, that is the worst corner. (3) Up to `WORST_PASSES` = 3 passes: every parameter that moved the verdict alone is flipped to its other bound, one at a time; the flips that made it worse are taken together, and the worse of that corner and the best single flip becomes the corner if it is worse than before; a pass that finds nothing worse ends it. "Worse" is the margin, value − limit (limit − value for `belt-slack` and the lives, `AT_LEAST`), not the value, because some limits are bounded too (DC link capacitance and threshold). `dc-link-energy` is the exception: it fails only where the braking resistor is needed and has no rating, and with a resistor the link fills up to its room at every corner that brakes hard (margin 0 at all of them), so it is ranked by the energy in the capacitors, a failing run above any passing one. A corner is run once for all verdicts that land on it; reported per verdict is the worst at any run, the one-at-a-time runs and the config as it is included (so the worst is never better than the bands), with its value, limit, result and where the parameters were (a parameter left as configured is at neither bound). Cost: 2n + 1 runs, plus per pass up to the number of parameters per verdict; on the EXCT-30 (63 bounded values, 24 checks) about 2400 cycles in about 47 s on all cores, on the single axis (14) about 300 in 3.5 s. A search, not a proof: a worst case that no chain of single flips from the sign corner reaches, or one inside the bounds (a resonance hit by a stiffness between them), can be missed |
| Sensitivity | The same runs, reported as relative changes of peak torque and RMS torque (each of the motor it changes most) and of the cycle time. Ranked by the largest of these six changes |
| Optimiser | Starts from the config. Candidates: toggle corner blending, switch input shaping off where it is on, raise velocity, acceleration or jerk of all axes by 1.5 or 1.2. A candidate without input shaping that fails `settling` and nothing else is run again with the shaper, and with the shaper following the pose where the machine reports a resonance; one with the shaper that still fails `settling` alone, with a second shaper at `profile.shaper2_frequency_hz`, or where that is 0 at the resonance the machine reports at the start pose if that is more than 10 % off `shaper_frequency_hz`. Where a second shaper is on, switching it off is a candidate too. The first of these that is shorter by more than 0.1 ms, has no motor-limit violation and passes every verdict is taken; repeat until none is. **Geometry** (`analysis::lifts`): a lift, traverse and lower is three segments in a row of which the first moves along the last axis only, away from both the start (the target before) and the end of the three, the second along the other axes only at that height, the third along the last axis only; the first two neither dwell nor change the payload. Its lowest height is the farther of the two ends plus `profile.min_clearance_m`. Candidates, before all others: every lift at its lowest, then each alone at its lowest and halfway there. After all others: a lift rounded through a via point (`control::via_pieces`: a line straight up from the start to the lift height, a spline through the via point to the point straight above the end, a line down; timed by `path::plan` within the axis limits, with the transition curves of `profile.path_tolerance_m`), the via point at the middle and 1/2, 1/4, 1/8 of the width above the lift height; a via point moved by 1/8 or 1/32 of the width along each axis; the rounding dropped. A via candidate whose planned poses between the two ends (along the other axes) come nearer to the ends than the lowest height, less the path tolerance, is not run. The new cycle is reported as a whole (`changes` entry `cycle.segment`, applied by `set_text`) and per lift (`Optimised::lifts`). On the default T-gantry with a clearance of 0.1 m the lifts go to 0.15 m and the cycle from 6.08 s to 3.36 s with the other steps (3.44 s robust); a rounded lift never won there (300 runs, with and without a path tolerance of 2 mm): the arch's curvature and its slower speed cost more than the two stops it saves. **Obstacles** (`over_obstacles`): where the config has obstacles, a lift's lowest height is the lowest at which its traverse, followed straight at that height with the machine at rest every millimetre (as `check_path`), keeps the clearance to them with the collision bodies; from the height above, it is moved by what the clearance lacks (or has too much, never below the height above) until it is the clearance to 1 nm, at most 40 times within the axis range (none found: the lift is not lowered). The bodies move with the last axis one to one, so a box across the traverse is cleared in one step: lift = top of the box + clearance + how far the lowest body reaches below the tool. The run sways a little (0.05 mm on the T-gantry), so a candidate that fails `collision` alone by less than 1 mm is run again, at most twice, with the lifts the obstacles raised higher by what it lacked and 1 µm. A lift an obstacle raised is also tried rounded through a via point over the obstacle's nearest pose at that height and 1/8 of the width above it, its ends at the height above pick and place; every via candidate's planned path is checked against the obstacles (clearance kept) before it runs. **Profile** (`profile`, `lift_profiles`): per obstacle, `over_obstacles` with it alone; those it raises are in the way, with the stretch of the traverse (every mm, at the lowest height, at rest) nearer to it than the clearance; overlapping stretches merged at the higher height; with two or more, two via points per stretch at its height, at its ends. The ends of the path at the height above pick and place. The planned spline is checked as every via candidate, and where it lacks clearance, all points are raised by what it lacks and 1 µm, three times at most; a profile that still lacks it is dropped. It is a candidate beside the via over the nearest pose; a profile is not moved, only kept or dropped. **Robust** (`OptimiseOptions::robust`; `sim-cli optimise` without `--nominal`, the app with "error bands" ticked): the fastest setting found is judged with the error bands (one bound at a time, 2n + 1 runs); where it fails a check there that the cycle as configured passes there, the accepted steps of the search are judged from the fastest down, each 2n + 1 runs, until one passes; the cycle as configured passes by definition. A step that fails `collision` alone in the bands, by less than 1 mm, is judged again (one run and its bands) with the lifts the obstacles raised higher by what the bands lacked and 1 µm. Checks the configured cycle fails in its bands already are reported (`Robust::excused`) and not judged. EXCT-30: the search's 1.492 s fails `belt-slack`, `settling` and `pulley-bearing-life` in the bands, the configured cycle `pulley-bearing-life` already; one step back, 1.590 s, passes the rest; 3 × 121 = 363 band runs. Default T-gantry: 5.340 s fails `settling`, 5.364 s passes, 3 × 47 = 141. **Worst corner** (`OptimiseOptions::worst`; `sim-cli optimise --worst`, the app with "worst corner" ticked): the same, judged with `verdict_worst` in place of the bands. The search stops after the first batch of runs (the 2n + 1 runs alone, the corners, a pass's single flips, its combined flips) in which a check judged on fails; it starts per verdict from the sign corner and from the previous judged step's worst corner (its parameters left as configured at the sign corner's bound), the worse of the two. The fastest is judged first, stopping at any failure; where it fails, the configured cycle is searched whole for the excused checks, the fastest judged again on the rest where it failed only excused checks or `collision`, then the accepted steps bisected (step 0 passes by definition): log2 of the steps searches instead of up to one per step. EXCT-30: the fastest (1.492 s) fails `belt-slack` and `settling` at the corner, steps 3 and 1 of the seven fail as well, `pulley-bearing-life` excused; the result is the configured 2.048 s, 3001 corner runs, 2.9 min on 16 threads. Default T-gantry: the search's 5.340 s fails `settling` there; three steps back, 5.442 s; 4964 corner runs, 16.5 min (bands: 5.364 s). Not the default for that time. **Front** (`OptimiseOptions::pareto`): every setting run that passes every verdict, and from the result its acceleration and jerk, and all three limits, times 0.8, 0.6 and 0.4 (six runs), as (cycle time, energy per cycle from the supply, highest `MotorLoad::utilisation`); kept are those no other is as good as in all three and better in one (`non_dominated`). Energy from the supply (`supply_energy`): what the drives drew less what they fed back, plus what the braking resistor burned (a shared one once); the line module's loss and the bleeder are left out |
| Frequency sweep | No control. A sine torque on one motor on top of the holding torques; that motor's speed is correlated with sine and cosine over whole periods (at least 0.2 s) after settling (at least 0.3 s). Gain in rad/s per N m |
| Step response | The machine settles for 0.05 s, then the position setpoint of one axis is offset by the step size, without feedforward. Rise time 10 % to 90 %; overshoot as a fraction; settling to within 2 % |
| Closed-loop response | A sine offset on the position setpoint, measured at the tool. Bandwidth: where the gain first falls below 1/√2, interpolated on the logarithmic frequency axis |
| Motor search | See the [user guide](user-guide.md#find-the-lightest-motor). Velocity gain of a candidate = the config's gain × sqrt( ratio of inertia seen × ratio of rotor inertias ), rotor inertia with the gearbox's own, against the config's own numbers. The same rule applies wherever a part is named (`[parts]`, `--motor`, `--gearbox`). "Tightest check" leaves out `dc-link-overvoltage` and, with a braking resistor in use, `dc-link-energy` while they pass |

## Effect views

Sources: `losses` and `view_signals` (the machine's `loss_powers` and `effect_signals`) in each machine's `lib.rs`; `Lugre::steady`, `Lugre::split` and the `loss_power` functions in [`mech.rs`](../crates/sim-components/src/mech.rs); `EnergyBudget`, `Tally`, `Comparison` and `compare_effect` in [`analysis.rs`](../crates/sim-components/src/analysis.rs); `HeatedPart` and `Sim::scene` in [`sim.rs`](../crates/sim-components/src/sim.rs); the conventions `TORQUE_PARTS`, `LOSS_PREFIX`, `ENERGY_PARTS`, `FRICTION_SIGNALS`, `THERMAL_KINDS`, `TEMPERATURE_PREFIX` and the methods `friction_contacts`, `friction_curve`, `belt_sections` and `thermal_parts` in [`machine.rs`](../crates/sim-core/src/machine.rs).

A machine tells the effect views what it knows through signals with fixed names and four methods with defaults; a machine that has none of them still runs, and the views say nothing about it. Front ends and the analysis find the signals by their names only.

**Cost.** The torque breakdown needs the machine's accelerations, one more evaluation of its derivatives. So the machine gives its signals in two parts: `base_signals` (tensions, guide loads, gearbox torques: what the verdicts look at every step) and `effect_signals` (the signals of the views, after them), and the loss powers alone as `loss_powers`, without derivatives. `Sim::sample` and `run` take both parts; `Sim::step` takes only the loss powers, and only with `effects.part_heating` or `thermal_feedback`; the analysis adds up breakdown and budget only where they are asked for (`analyse_cycle_with(sim, views)`: the app's analysis and `sim-cli analyze --breakdown/--budget` ask, bands, sensitivity, optimiser and motor search do not). Every number is the same either way (`the_cheap_signals_and_the_lean_analysis_change_no_number`).

**Run time of the analyses** (13 November). Where a cycle run's time goes, release build, measured on one run each: the integration of the machine (16 evaluations of its derivatives per base step: RK4 with 4 sub-steps, 176 ns each on the T-gantry) is 90 %; the analysis's tally 10 %, of which the path error (the tool against the last 500 setpoints, every millisecond) was 6 %, the machine's own signals every step 2 %, the sample of all signals every millisecond 1 %. Changed without changing a number: the sensitivity's runs share the processor cores (they ran one after the other); the tally samples only the signals of `base_signals` where every signal it looks at is among them (`Sim::sample_base`); the path error skips a piece of setpoint path that the tool is beyond along one axis by more than the nearest distance found so far (plus 1 pm), looking at the newest pieces first, and reuses the oldest setpoint's storage; the bands and the worst-corner search in the app share their first 2 n + 1 runs (`verdict_worst_and_bands`). The searches were on all cores already, and 8 cores with 16 threads are the limit there.

### Torque breakdown

Per motor four signals `motor.<id>.torque_inertia`, `_gravity`, `_friction`, `_rest`. The machine's own forces are mapped onto the motors with the transpose of its rigid kinematics, the mapping the torque feedforward uses. T-gantry, with k_r = r / (2 i) and s = +1 for motor A, −1 for B:

```text
inertia  = k_r · (I_x + s · I_z)
I_x      = m_c · (ẍ + q̈) + m_z · (ẍ + q̈ + ü) + m_M · ẍ + m′ · w · (v_x − w/2)       elastic
I_x      = (m_x + m_M + m_idler,x + m_chain(x)) · a_x + ½ · m′ · v_x²                   rigid
I_z      = (m_z + m_idler,z + m_M) · z̈
gravity  = s · k_r · m_z · g
friction = τ_shaft(ω) + τ_brake + τ_gear loss + k_r · (F_f,x + F_idler,x + drag + s · (F_f,z + F_idler,z))
rest     = τ − inertia − gravity − friction
```

m_M = 2 (J_pulley + i² J_rotor) / r² is both pulleys and both rotors seen at the tool; m_c is the carriage with its idler and chain mass; w the carriage's speed over ground. The gearbox loss at the rotor is τ_g / i − τ_out / i in the elastic model (what the rotor gives the gear less what reaches the pulley, over the ratio) and (τ_through − τ_out) / i in the rigid one. Single axis, with r / i in place of k_r: inertia = r / i · ((m + J / r²) · a + ½ m′ v²) with J = J_pulley + i² J_rotor, gravity = r / i · m g sin(incline), friction = τ_shaft + τ_brake + τ_gear loss + r / i · (F_f + drag).

**Inertia is taken at the load's acceleration**, with rotors and pulleys turning as a rigid machine would turn them at it. Rotors and pulleys that ring against the load on the belt or knock through the gearbox backlash are then in the rest, together with the belt force that drives the ringing. Taken at the rotors' own accelerations instead, the backlash knocks of the default T-gantry put peaks of 1.3 N m into the inertia part and the opposite into the rest, with a peak motor torque of 0.64 N m (measured before the winding was integrated with the mechanics; 0.67 N m now).

**The rest** holds: the belt's and the gearbox's elastic forces beyond what the load needs (stretch, ringing, backlash), the force of the leaning Z axis's spring on Z, ripple and cogging (τ − ripple(τ)), tooth meshing, the process force and the end stops. In the rigid model only ripple, meshing, process force and end stops are left; with those off, the rest of a rigid model is zero to rounding (`a_pure_inertia_move_is_all_inertia`).

**The parts add up to the motor torque** at every step to rounding (1e-9 relative is tested), because the rest is defined as what the other three leave. A breakdown that left the rest out would have to add up by itself; this one cannot fail to, so what the tests check is that the three named parts are right (hand calculations) and that the rest is nothing where nothing elastic is.

The analysis looks at the parts every step and reports per part its peak of |value| and its RMS (`MotorLoad::torque_parts`). Peaks and RMS values of the parts do not add up to those of the torque: they come at different times.

### Loss powers and energy parts

Signals `power.<place>` in W. Each is what goes into a place of the machine less what that place stores, so that together they close the machine's balance: the work of the motor torques Σ τ_k Δφ_k equals the change of `Machine::energy` plus the time integral of all `power.*` (`testkit::loss_budget`, 0.1 %; measured 1e-4).

| Signal | Power |
| --- | --- |
| `power.guide_x`, `power.guide_z` | F_f · v, the LuGre force times the guide's speed. Counts what the bristles store as lost; that is below a millijoule |
| `power.idlers` | F_idler,x · v_x + F_idler,z · v_z |
| `power.cable_chain` | drag · v_x |
| `power.shaft_<motor>` | (τ_shaft(ω) + τ_brake) · ω |
| `power.gearbox_<motor>` | Elastic: (τ_g / i) · ω − τ_out · θ̇ − k_g · twist · (ω / i − θ̇), the efficiency loss and the damping. Rigid: (τ_through − τ_out) / i · ω. On a direct drive (ratio 1, `Gearbox::is_coupling`) the `[gearbox]` table is the coupling, and this is the coupling's damping c_g · (ω − θ̇)², labelled "Coupling damping loss": the EXCT-30's 9.1e-5 J per cycle |
| `power.belt` | Per section T · d(geo)/dt − M · d(mat)/dt − dV/dt; c_s · (rate)² while taut |
| `power.frame`, `power.tool_flex` | c_f · q̇², c_u · u̇² |
| `power.hard_stops` | −(F_stop + k_s · beyond) · v per axis |
| `power.other` | (τ − ripple(τ)) · ω per motor, less what the tooth meshing and the process force put in. What the process takes is work done on the workpiece; ripple and meshing can give or take |

Signals `energy.kinetic`, `energy.potential`, `energy.elastic` in J; they add up to `Machine::energy`. Elastic is belt, gearbox springs, frame spring, the leaning Z axis's spring and the end stops.

### Energy budget

`EnergyBudget` of a cycle or path, over the measured pass (for a cycle from the state after the step in which the pass begins):

```text
supply = Σ P · dt                                         drives' electrical power, net; without the DC link
       = Σ drawn                                          with effects.dc_link: what the rectifier (or line module) drew
rows   = copper loss per motor  Σ E_cu                    integrated with the mechanics (see the winding)
       + braking resistor (with the DC link; a shared link's one resistor once)
       + DC link discharge of drives switched off           with the DC link: ½ C (U² − U'²) per step through the bleeder
       + fed back into the mains, line module loss         with effects.regenerative_supply
       + every power.* · dt
       + ΔE windings (½ L i² of all motors)
       + ΔE capacitors (with the DC link)
       + Δenergy.kinetic, Δenergy.potential, Δenergy.elastic
       + carried off with the payload = −(sum of the steps of Machine::energy when the payload switched)
       + crash = sum of the steps of Machine::energy at each stop at an obstacle   with effects.crash
unaccounted = supply − rows
electrical  = supply − copper − resistor − discharge − mains − line module − ΔE windings − ΔE capacitors − Σ W   (W: shaft work)
```

The rectifier's energy is counted on its own, every step, as what lifts the capacitor energy back to that of the bus voltage. A payload picked up brings its kinetic and potential energy into the machine and one put down takes it away; the simulation adds up these steps (`Sim::payload_energy`). The stores are read after the last step counted: a cycle's measured pass leaves out the step in which it begins and the one in which the next begins, flows and stores alike. (Until 12 November the stores were read one step later than the flows ended, which on the loss-free elastic configs put up to 0.8 % into "not accounted for", taken then for backlash knocks.)

**The electrical side is checked on its own** (`EnergyBudget::electrical_unaccounted`): the supply against copper loss, resistor, the discharge of drives switched off, mains, line module, the windings' and the capacitors' stored energy and the work the torques did on the shafts, integrated with the windings (`EnergyBudget::shaft_work`). Within `ELECTRICAL_TOLERANCE` = 1e-6 of the supply on every machine and config, the loss-free ones included: measured at most 3.2e-7 (single axis, loss-free inclined), 1e-12 on the default configs; it falls by 15 with each halving of the integration step, as fourth order does. With `effects.back_emf` off the shaft work comes from nowhere and this does not close; no shipped config has it off. What `unaccounted` holds beyond it is the machine's own balance.

**Tolerance.** What is not accounted for stays below `BUDGET_TOLERANCE` = 0.1 % of the supply on every config, the loss-free ones included: measured 0.009 % on the default T-gantry (1.2e-2 J of 139.5 J), 0.00001 % on the single axis, 0.0004 % on the EXCT-30, at most 0.03 % on the loss-free elastic T-gantry (`the_energy_budget_closes_on_every_machine_and_config`). It is the machine's losses taken at the end of each base step and summed times the step, the trapezoid rule but for the ends of the pass, and the undamped ringing knocking through the gearbox backlash on the loss-free configs, which the fixed step integrates through without events.

### Friction view

Per contact that `friction_contacts` names (the X and Z guides of the T-gantry, the X guide of the single axis), `friction_curve(i, v, λ)` is the steady sliding force of the LuGre model split into three parts:

```text
[ λ · F_c · sign(v),   b · v,   λ · (F_s − F_c) · exp(−(v / v_s)²) · sign(v) ]       sum: g(v) · sign(v) + b · v
```

Its signals `<id>.friction_speed`, `_coulomb`, `_viscous`, `_stiction` and `_load_scale` give where the contact is now: the guide's own speed (on the T-gantry the carriage along the beam, not the tool tip over the floor), λ, and the force of the LuGre state split as b · v and the bristle force σ₀ s + σ₁ ds/dt shared in the proportion λ F_c : λ (F_s − F_c) e^(−(v/v_s)²). In steady sliding these are the curve's parts; held at rest the bristle force is shared in the proportion of F_c to F_s − F_c. The three add up to the signal `<id>.friction`. The app draws the curve at the load factor of the newest sample, at 200 speeds from −v_max to v_max of the axis, closer together near standstill (v_max · (j/100)³), and the point (speed, force) of the newest sample on it; off the curve means the bristles are not in steady sliding (breaking away, reversing).

Not in the friction view: the motor shafts' friction (Coulomb and viscous, no stiction), idlers and cable chain. They are in the breakdown's friction part and in the energy budget.

### Thermal model and view

**Windings** (`effects.winding_heating`, on by default): one thermal mass per winding, as before (see the drive). At a constant current i the rise follows τ dϑ/dt = R(ϑ) i² · R_th − ϑ with R_th = ϑ_rated / (R(ϑ_rated) i_rated²), so it settles at the rated rise at the rated current and rises with the time constant τ = `motor.thermal_time_constant_s`.

**Gearboxes and guides** (`effects.part_heating`, off by default): `thermal_parts` names the parts that warm, each by the id of its loss signal `power.<id>` and its kind (`gearbox` or `guide`). Each is one thermal mass with a thermal resistance R_th to its surroundings:

```text
tau · d(rise)/dt = R_th · P − rise              P = power.<id>, the loss the energy budget counts
rise(t + dt)     = rise + (1 − exp(−dt / tau)) · (R_th · P − rise)        every base step, P at the end of the step
```

The step is the exact answer of the first-order lag to a loss held over the step. Under a steady loss the rise settles at R_th · P, and the gap to it shrinks by the factor e every τ. R_th, τ and the rated rise come per kind from `thermal.<kind>_resistance_k_w`, `_time_constant_s` and `_max_rise_k`. The model lives in the simulation (`Sim`), not in the machine: the machine's state, its energy and the standard suite are untouched, and nothing feeds back (friction and efficiency do not change with the temperature; `effects.temperature` stays the hand-set factors). The rises are the signals `temperature.<id>` (K), present whether or not the effect is on; switched off, they stay where they are. A reconfigure keeps the rise of every part that is still there.

| Machine | Parts | Shown in the 3D view by |
| --- | --- | --- |
| T-gantry | `gearbox_a`, `gearbox_b` (`power.gearbox_a/b`), `guide_x`, `guide_z` (`power.guide_x/z`) | the gearboxes, the carriage, the Z axis's column |
| Single axis | `gearbox_m`, `guide_x` | the gearbox, the carriage |

A direct drive (`gearbox.ratio` = 1, as the EXCT-30) has no gearbox: its gearboxes are not among the parts that warm, are not in the thermal view and not judged; what `power.gearbox_*` counts there is the coupling's damping (see the loss powers).

**The view** draws, over the scopes' window, the rise of each winding and, with `effects.part_heating`, of each heated part, and a table: now, where it settles on the analysed cycle, its rated rise and the share of the rating where it settles (marked "above the rating" over 100 %). The share is that of the settled rise, not of the rise now: with time constants of 20 to 40 minutes the rise now is a few percent of where the part ends up. A winding settles at `MotorLoad::winding_rise` (the copper loss of the cycle's RMS current at the resistance of that temperature); a part at R_th times its mean loss over the measured pass, `CycleLoad::part_rises` = R_th · Σ P dt / cycle time, the same as its row of the energy budget over the cycle's duration. Winding rises are rated by `motor.rated_temperature_rise_k`. The checks `gearbox-thermal` and `guide-thermal` judge the same settled rises.

**Estimates.** The guide's R_th is that of the rail and the aluminium profile it is bolted to, not of the carriage alone: the carriage runs over the whole stroke and spreads its heat along the rail, and aluminium conducts well. About 0.35 m² of profile at 10 W/(m² K) (still air about 5, radiation of anodised aluminium about 5) with a fin efficiency of about 0.6 gives 0.5 K/W (bounds 0.25 to 1); about 5 kJ/K of aluminium and steel gives τ ≈ 40 min. The carriage alone, 2 K/W as first estimated, put the EXCT-30's X guide at 101 K (50.7 W mean loss) and the single axis's at 50 K, above the 40 K rating with the effect on (the default T-gantry's 16.8 K passed). With 0.5 K/W they settle at 25.3 K and 12.5 K; at the high bound of 1 K/W the EXCT-30's X guide reaches 50.7 K and fails the band's worst case. One R_th per kind: the T-gantry's Z guide sits on a smaller profile and is taken with the X guide's value.

**Colour.** `Sim::scene` gives a part a heat value h = rise / rated rise (`Part::heat`): the k-th motor housing (`scene::MOTOR_PART`, drawn by `drive_unit`) with motor k's winding while `effects.winding_heating` is on, and the part `thermal_parts` names with its rise while `effects.part_heating` is on. The 3D view then draws the part in the heat scale in place of its own colour: blue at 0, yellow at 0.5, red at 1, magenta above 1 (`model::heat_color`).

### Temperature feedback

Source: `ThermalFeedback` in [`mech.rs`](../crates/sim-components/src/mech.rs); `effects.thermal_feedback`, off by default. It replaces `effects.temperature`: while it is on, the hand-set factors of `[temperature]` are not used. It warms the parts as `effects.part_heating` does (`sim::heats`), whether that is on or not, and before each base step `Sim` tells the machine the rises the last one ended with (`Machine::set_part_temperatures`). The config's friction, efficiency and EA hold at `thermal_feedback.reference_c` (20 °C, the temperature gearbox makers quote their no-load torque at); the machine stands at `ambient_c`, and a part is at ambient plus its rise.

```text
nu(T) / nu_ref = exp(B · (1/T − 1/T_ref))                 T in K, B = thermal_feedback.viscosity_slope_k
guide:   b = b_config · nu(T_guide) / nu_ref              Coulomb and stiction levels stay
gearbox: 1 − eta = (1 − eta_config) · (1 − s + s · nu(T_gearbox) / nu_ref),   at least 0.01       s = gearbox_viscous_loss_share
belt:    EA = EA_config · (1 − alpha · (T_ambient − T_ref))                    alpha = belt_stiffness_coefficient_1_k
```

- **Viscosity.** Andrade's (Arrhenius) law for a lubricant's viscosity. B from a mineral oil's viscosities at 40 and 100 °C (viscosity index about 100): ISO VG 32 (32 and 5.4 mm²/s) gives 3470 K, VG 68 (68 and 8.7) 4000 K, VG 220 (220 and 19.4) 4730 K; 4000 K, bounds 3470 to 4730. 30 K above the reference the oil has 0.2817 of its viscosity, at −10 °C 4.74 times.
- **Guide.** The viscous coefficient is the lubricant's drag and follows the viscosity; the Coulomb level of a rolling contact and its seals is taken as constant.
- **Gearbox.** The model has one efficiency and no no-load torque. Makers quote the no-load running torque at 20 °C and it falls as the gearbox warms (churning and bearing drag in thinner oil), while the tooth friction under load does not. So a share s of the loss (1 − η) follows the viscosity, the rest stays: an estimate, s = 0.5, bounds 0.25 to 0.75. 30 K warm, a 95 % gearbox has 96.80 %. No maker publishes the curve; a search found the 20 °C convention and the direction, nothing more.
- **Belt.** Polyurethane softens as it warms, but the steel cords carry EA, and steel's modulus falls by about 2.5e-4 per K: alpha = 3e-4 per K (bounds 0 to 1e-3), 0.6 % at 20 K. The belt is taken at ambient: it has no loss of its own in the model, and an EA that changed while running would change the stored energy without a loss to account for it. So the belt's factor acts only when ambient differs from the reference.
- Without part heating's losses the parts stay at ambient; the windings are not part of this (their resistance already follows their temperature, `effects.winding_heating`).

Tests: `the_temperature_laws_follow_their_formulas` (the factors at fixed temperatures), `with_thermal_feedback_friction_and_gearbox_loss_follow_the_temperature` (the machines at 30 K), `a_guide_settles_where_its_loss_and_heat_flow_balance` (single axis, R 2 K/W at 1 m/s: rise = 2 (λ F_c + b ν/ν_ref v) v settles at 40.2 K instead of 72.5 K).

### Thermal warm start

Source: `analyse_cycle_with` and `analyse_trajectory_with` in [`analysis.rs`](../crates/sim-components/src/analysis.rs); `effects.thermal_warm_start`, on in the shipped configs. It is not a physical effect but how the analysis starts; it sits among the effects because every switch of a run does, the app shows it there, and a config without it keeps starting cold. With thermal time constants of 20 minutes to an hour, a cycle of seconds analysed from cold sees every part at the temperature it started with: the winding's resistance at ambient, and with `effects.thermal_feedback` the guides' friction and the gearboxes' losses cold. A machine that sizing judges has run for a while.

The rises that act back on the run are the windings' (with `effects.winding_heating`: their resistance) and the gearboxes' and guides' (with `effects.thermal_feedback`). Where there are such rises, the cycle repeats:

```text
pass 1                 from cold, not measured (as before)
pass 2                 measured from where pass 1 left the rises r_2: settles them at s_2
                       (winding: settled_rise(I_rms); part: R_th × its mean loss, as the thermal checks judge)
pass 3                 measured with every rise put at s_2 at its start: r_3 = s_2
pass k + 1             a secant step per rise: r_(k+1) = r_k + (s_k − r_k) / (1 − m),  m = (s_k − s_(k−1)) / (r_k − r_(k−1))
                       clamped to [−20, 0.5]
stop                   when max |s_k − r_k| ≤ 0.1 K (WARM_START_TOLERANCE_K), or after 4 warm passes (WARM_START_PASSES)
```

The secant step: a warmer guide or gearbox loses less (its lubricant thins), so the rise a pass settles at falls with the rise it starts at, and plain substitution r = s oscillates towards the fixed point; on the EXCT-30's guide (slope about −0.4) it was still 0.3 K off after four passes. A winding's settled rise depends on its current alone (m = 0).

The last measured pass is the result; `CycleLoad::warm_start` has the warm passes and the last change, and the verdicts a row `thermal-warm-start` (that change against 0.1 K). A rise that does not settle (a winding loaded beyond what its copper can shed) ends the iteration at once, not converged. So does a run that stops (a drive trips, the machine hits an obstacle) in a warm pass; one that stops before a warm pass began was measured from cold, as `CycleLoad::warm_start` then says. Neither has a `thermal-warm-start` row: the trip, the crash or `motor-thermal` fails for it. Over a measured pass the rises drift with their time constants, by a hundredth of a kelvin in a cycle of seconds. A trajectory or path (`analyze --path`) counts as a cycle that repeats: between passes the machine goes back to its start (`Command::MoveTo`) and rests 0.3 s; the rises are set after that, and what the way back left ringing in a lightly damped machine is in the next pass. Each warm pass costs one more cycle. On the shipped configs one (the windings alone converge in one: their settled rise depends only on the current); with part heating and the feedback on, two (T-gantry, delta picker) or three (single axis, EXCT-30, which ends 0.045 K off). `verdicts` on the default T-gantry with the feedback on took 3.8 s against 2.9 s from cold.

What it moves: the windings settle at 1 to 2 K on the shipped cycles, so the shipped verdicts move in the fourth digit. With part heating and the feedback on, the single axis's guide settles at 10.03 K instead of 12.51 K (its viscous friction is 0.637 of the cold one there, not 1) and its `gearbox-continuous` falls from 0.806 to 0.695 N m; the T-gantry's guide at 3.86 K instead of 4.19 K; the EXCT-30's X guide at 15.90 K instead of 23.68 K, its windings at 6.62 and 6.08 K instead of 6.88 and 6.24 K. A 0.5 A-rated winding (single axis) settles at 73.8 K, and the measured pass's copper loss is R (1 + 0.0039 · 73.8) I² = 1.026 W instead of the cold 0.797 W.

Tests: `a_winding_started_warm_loses_the_hand_calculated_copper_power_at_its_settled_rise`, `a_guide_started_warm_has_the_hand_calculated_friction_at_its_settled_rise`, `the_warm_start_converges_on_the_default_t_gantry_within_its_pass_limit` (belt_and_heat).

### Belt view

`belt_sections` names the belt sections, each with its pretension. Signals `<id>.tension` (N, the existing `belt.<i>.tension`) and `<id>.stretch` (m):

```text
stretch = geo − mat         the section's length less its unstretched belt; T = EA · stretch / mat (+ damping)
rigid model: stretch = geo − geo / (1 + P / EA) = geo · P / (EA + P), tension = P
```

At rest without a load every section has the pretension P and the stretch geo · P / (EA + P). Held against gravity the sections balance the weight: on the single axis T₂ − T₁ = m g sin(incline); on the T-gantry, whose Z axis hangs in the loop of section 2, 2 T₂ − T₁ − T₃ = m_z g and T₁ = T₃. The view draws tension (with the pretension as a dashed line) and stretch (mm) per section over the scopes' window, and a table of the newest sample with `ratings.belt_max_tension_n` and the share of it.

**Colour.** Each machine's `scene` colours the strips of a section by h = (T − P) / P, clamped to ±1, from the same tensions as the signals. The 3D view moves the belt's own colour towards red by h above 0 (red at T = 2 P) and towards blue below (blue at T = 0, slack). The view draws this scale as its legend.

### Effect comparison

`compare_effect(entry, cfg, effect)` copies the config twice, sets `effects.<effect>` true in one and false in the other, and runs each on a simulation of its own (two threads) through the configured cycle. As in `analyse_cycle`, the second pass is the one measured: every signal is kept once per millisecond, with the time counted from the start of that pass. `Comparison::difference(i)` is the peak of |on − off| and the RMS of on − off of signal i over the rows both runs have; where the effect changes the timing (corner blending, input shaping), the longer pass's rest is not compared. A config compared with itself differs in nothing (the simulation is deterministic).

At constant speed the difference of a friction effect is the friction force through the drive train: guide friction on against off changes the motor torque by r / i · (λ F_c + b v) / η on a motor that drives its gearbox (× η on one that is driven back); on the T-gantry each motor carries half (r / (2 i)), plus or minus the force the Z guide's bristles hold while Z stands still.

## Fit

Source: [`identify.rs`](../crates/sim-components/src/identify.rs), [`trace.rs`](../crates/sim-components/src/trace.rs).

**Replay.** The motor position setpoints of the trace become tool poses through the forward transformation and run as a trajectory. Each row is commanded one row earlier, because a run writes a row before it computes the step at that time. The simulator's own controller makes the speed and torque feedforward from the poses.

**Stage 1, rigid values.** Motor torques become axis forces through the machine's transformation (motor angle per axis travel; the same power on both sides), so the fit works per axis even where every motor moves every axis. Per axis, least squares of

```text
force = mass · acceleration + viscous · speed + coulomb · sign(speed) + constant
```

Force and speed are averaged over 11 rows; the acceleration is the slope of the speed across them. Rows slower than 5 % of the top speed are left out. The standard errors count only every 11th row as a measurement, because neighbouring rows share their window. The same estimate runs on the trace and on the config's own simulation of the trace's setpoints; what the simple estimate gets wrong (gearbox losses that grow with torque, load-dependent friction) it gets wrong on both sides, so the difference is what the config is off by.

**Stage 2, resonance.** In a sweep of motor torque to motor speed, the load resonating on the belt shows as a dip. Gain times frequency is level for a pure inertia; the dip's flanks are where that has twice the power of the lowest point. Frequency = geometric mean of the two flanks; damping ratio = (high − low) / (2 · frequency); frequency error = half the spacing of the sweep's points there. With the masses right, (measured / config frequency)² is the factor the stiffness is off by.

## Known shortcuts

Places where the code carries a `shortcut:` comment.

| Where | Shortcut | When to upgrade |
| --- | --- | --- |
| [`machine-t-gantry/src/lib.rs`](../crates/machine-t-gantry/src/lib.rs), `elastic_derivatives` (line 381) | Idler and cable-chain inertia ride on the carriage's speed over ground, not on its speed along the beam. The two differ by the frame's own motion, which is a thousand times smaller | If the frame moves by more than a small fraction of the carriage's travel |
| [`sim-components/src/control.rs`](../crates/sim-components/src/control.rs), `Controller::rescale_ticks` (line 302) | When `profile.cycle_s` changes, setpoints on the fieldbus keep their count of cycles, so the fieldbus delay changes with the cycle at once instead of after the queue has run through | Resample the queue, if that step ever matters |
| [`sim-components/src/control.rs`](../crates/sim-components/src/control.rs), `Controller::shape` (line 707) | While the shaper's period changes (`shaper_follows_pose`), the speed and acceleration setpoints are the delayed plan's, not the exact rates of the shaped position. Off by the rate of the delay, a few percent in a Z move; they are feedforward only | Shape the position alone and difference it, if that matters |
| [`sim-components/src/mech.rs`](../crates/sim-components/src/mech.rs), `Lugre::eval` (line 290) | Bristles that would settle faster than 1e5 1/s settle at that rate, with their force scaled so that sliding still takes the level | Step the friction state implicitly, if what bristles do within 10 µs ever matters |

## Other simplifications

Not marked as shortcuts in the code, but worth knowing. Most are stated in the sections above.

| Area | Simplification |
| --- | --- |
| Motor | One equivalent winding. No phases, PWM, field weakening or iron loss |
| Drive | Delays are whole cycles; the fieldbus jitter is whole profile cycles, evenly spread, frame by frame independent. The same settings for every drive. The filters are generic, not a vendor's |
| Thermal | One thermal mass per winding, gearbox and guide (`effects.part_heating`); no belt temperature (the belt is at ambient). With `effects.thermal_feedback` guide friction and gearbox losses follow the temperatures (one viscosity law for all lubricants); without it `effects.temperature` is two factors set by hand |
| DC link | One per drive, or one for all with the sum of their capacitances; feedback into the line only through an ideal line module of one efficiency (`effects.regenerative_supply`); no resistor heating, no Line Module capacitance or busbar resistance; the bleeder draws only while the drives are disabled |
| Belt | Mass lumped on the pulleys. Sideways span vibration as first-mode frequencies only, no amplitude (`effects.belt_span_vibration`); tooth jump with a quasi-static climb-out law and the belt sliding over a sinusoidal ridge (`effects.belt_tooth_jump`); no wear |
| Friction | Guide loads from the commanded acceleration of a loss-free rigid machine, not from the true one. No moments on single guide blocks |
| Gearbox | Constant efficiency, constant stiffness, hard dead zone |
| Frame | One mode, in X only |
| Z axis | Rigid body leaning on a compliant guide, in X only |
| Stop | No fast stop ramp, no emergency stop |
| Brakes | A spring up to the brake torque, then slipping with it; no wear, no engagement delay. In the rigid model the spring holds the whole machine with the damping meant for the rotor |
| Cycle | The payload switches as a step on arrival; no gripper time, no swinging payload |
| Feedforward | Rigid model with inertia and gravity. No friction feedforward, so some following error stays while moving |
