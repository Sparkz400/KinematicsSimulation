# Model reference

The equations the simulator solves, taken from the code and its comments. Each section names the source file. Config keys are written as `table.key`; their units are in the [config reference](config-reference.md).

- [Structure of one time step](#structure-of-one-time-step)
- [Integrator](#integrator)
- Components: [belt](#belt-sections), [guide friction](#guide-friction-lugre), [motor shaft](#motor-shaft-friction-ripple-cogging), [slide presliding](#slide-presliding-friction), [tooth meshing](#tooth-meshing), [gearbox](#gearbox), [hard stops](#hard-stops), [drive and motor](#drive-and-motor)
- Machines: [T-gantry](#t-gantry), [single axis](#single-axis), [delta picker D3](#delta-picker-d3), [linear delta](#linear-delta), [toggle press](#toggle-press)
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
g(ω)    = s_seal · (T_c + (T_s − T_c) · exp(−(ω / ω_st)²))            sliding level, from breakaway to Coulomb
τ_shaft = g(ω) · tanh(ω / ω_s) + b · ω + T_0(ω, ν)                     friction opposing the speed
τ_out   = τ · (1 + ρ · sin(n_r · φ)) + T_cog · sin(n_c · φ)           torque at the shaft for a torque τ from the current
```

T_c `friction_coulomb_nm`, b `friction_viscous_nm_s_rad`, ω_s `friction_smoothing_rad_s`; ρ `ripple_fraction`, n_r `ripple_order`, T_cog `cogging_nm`, n_c `cogging_order`. The Coulomb step is smoothed with tanh so that a shaft at rest sees no chattering torque. Ripple and cogging are one harmonic each.

- **Breakaway** (`effects.motor_stiction`). The level falls from the breakaway torque T_s `friction_breakaway_nm` at rest to T_c along a Stribeck curve over ω_st `friction_stribeck_rad_s`. A breakaway below T_c counts as T_c, as for the guides; off, T_s = T_c. With the tanh smoothing the shaft holds up to T_s near rest as a steep viscous slope, not as a stuck state: a drive that pushes below T_s creeps at a fraction of ω_s.
- **The gearbox's no-load torque** T_0 (`effects.gearbox_drag`, see [gearbox](#gearbox)) acts on the shaft too: the motor turns the gearbox's input.
- **Seals over temperature** (`effects.thermal_feedback`). On a geared motor the shaft's seal is the gearbox's input seal and runs in its lubricant: s_seal = (ν / ν_ref)^m with m `thermal_feedback.seal_viscosity_exponent` and ν the gearbox's lubricant at its temperature (see [temperature feedback](#temperature-feedback)); 1 without the feedback.

Assumptions: the tanh smoothing stands for the bristles of a friction state (no presliding displacement, no dwell-time growth of the breakaway); the seal exponent is an estimate (0.33, bounds 0 to 0.5): no seal maker's curve of breakaway over temperature was read.

## Slide: presliding friction

Source: `ShaftFriction::presliding_force`, `torque_at`, `commit` in [`mech.rs`](../crates/sim-components/src/mech.rs); read by `RotaryJoint::linear_from_config` in [`joint.rs`](../crates/sim-components/src/joint.rs), committed by `JointMachine::after_step` in [`chain.rs`](../crates/sim-components/src/chain.rs).

The joint kit's slides (screw, rack and linear motor axis, the SCARA's quill) smooth their guides' Coulomb step with tanh, as the motor shafts do. A load below the level then makes the carriage creep at v_s · atanh(F / g) (0.49 mm/s for a fifth of the level at v_s = 2 mm/s), and a drive at rest holds no force against it: its integrator unwinds as the carriage creeps. With `effects.slide_presliding` the level is a state of its own, Dahl's law over the travel x:

```text
g(v)   = F_c + (F_s − F_c) · exp(−(v / v_st)²)                      level, from breakaway to Coulomb
z(x)   = s g − (s g − z0) · exp(−σ |x − x0| / g)                     from the anchor [x0, z0], s = sign(x − x0)
F_f    = z(x) + b v                                                  friction opposing the motion
```

F_c is the Coulomb level of `[slide]` with the nut's preload drag (screw) or the attraction friction (linear motor), F_s `friction_breakaway_n` plus the same, v_st `friction_stribeck_m_s`, σ `presliding_stiffness_n_m`, b the viscous friction. After each base step the anchor moves to where the step ended, [x, z(x)] (`Machine::after_step`; `Machine::memory` carries it, so a snapshot restores it). Sliding one way the force goes to ±g within a few g / σ; at rest it stays where it is, however long, since the travel and not the time moves it; at a reversal it turns over a presliding travel, through zero after g ln 2 / σ from a sliding level. Before the first step it is 0. Off, the friction is the smoothed step as before, to the bit.

Assumptions and shortcuts: Dahl's law closes no small loop, so a carriage that rings on its screw while a load pulls it moves on a little with each swing until the ringing has died (12 µm on the screw axis inclined by 5° with its drive disabled, `a_presliding_slide_holds_its_load_without_creeping`); no damping of the bristles (LuGre's σ₁), no dwell-time growth of the breakaway; the level is evaluated at the speed within the step; the motor shaft's own friction stays smoothed. A reload (`Sim::reconfigure`) starts the state again from 0.

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
e        = max(|twist| − β/2, 0)                       twist beyond the dead zone; β = backlash_rad
ẇ        = dφ/dt / i − dθ/dt                           the twist's rate
τ        = sign(twist) · f(e) + c_g · ẇ + T_h · tanh(ẇ / 1e-3 rad/s)    while e > 0, else 0
τ_out    = τ · η^tanh(τ · ω_out / 0.5 W)               on the pulley
τ_rotor  = τ / i                                       reaction on the motor shaft
```

The spring f(e), as gearbox makers measure the torsional characteristic with the input locked:

```text
f(e) = K1 · e² / (2 b)                                 e < b: the lost-motion band, the flanks come into full contact
f(e) = F(e − b/2)                                      beyond it
F(x) = K1 · x                                          x ≤ T1 / K1
F(x) = T1 + K2 · (x − T1 / K1)                         up to T2
F(x) = T2 + K3 · (x − T1 / K1 − (T2 − T1) / K2)        beyond
```

b `lost_motion_band_rad` and T_h = K1 · w / 2 with w `hysteresis_rad` (`effects.gearbox_lost_motion`, else both 0); T1, T2, K2, K3 `stiffness_t1_nm`, `stiffness_t2_nm`, `stiffness_k2_nm_rad`, `stiffness_k3_nm_rad` (`effects.gearbox_stiffness_curve`, else T1 = T2 = ∞: one stiffness K1 = `stiffness_nm_rad`). The twist at a torque T is then Harmonic Drive's T / K1, T1 / K1 + (T − T1) / K2 or T1 / K1 + (T2 − T1) / K2 + (T − T2) / K3 (plus b/2 past the band). T_h is a friction in the flanks and bearings: loading and unloading run on curves apart by 2 T_h / K1 = w in angle, Harmonic Drive's "hysteresis loss" (1 arcmin for sizes 25 and 32 at ratio 50 and up). It is a state, Dahl's friction model on the contact twist x (the twist beyond the backlash, signed), in parallel with the spring:

```text
dz/dx = σ · (1 − sign(ẋ) · z / T_h)        |z| < T_h, σ = K1
τ     = f(e) · sign(twist) + c · ẋ + z
```

Each gearbox keeps the anchor (x₀, z₀) where the last base step ended (`Gearbox::commit` from every machine's `after_step`); within a step z = s T_h − (s T_h − z₀) exp(−σ |x − x₀| / T_h), s the sign of x − x₀, which is Dahl's law solved exactly for a twist that runs one way. Loaded far one way z reaches ±T_h and the loop is w wide at zero torque; at rest the gear holds the z it stopped with; at the small amplitudes of a frequency response z is a spring of σ beside K1 (the gear is K1 + σ stiff there, not locked). The energy is the spring's plus z² / (2σ), the hysteresis takes s · ẋ · z² / T_h, so the energy balance holds with all of it. Tests: `the_hysteresis_loop_is_as_wide_as_given_and_holds_its_torque_at_rest` (T_h = 1.4545 N m on K1 = σ = 1e4 N m/rad: zero torque at ±1.4545e-4 rad after loading to ±0.01 rad, −1.4545 N m held at zero twist, z = T_h (1 − 1/e) = 0.919419 N m at x = T_h / σ, 0.290592 W lost at 0.5 rad/s either way), `a_gear_with_hysteresis_answers_small_signals_as_its_stiffness` (1e-7 rad at 100 Hz: (K1 + σ) in phase within 1 %, almost nothing in quadrature; the old rate-switched friction gave 45 times the spring's torque in quadrature).

No-load running torque (`GearDrag`, `effects.gearbox_drag`), at the input, added to the motor shaft's friction:

```text
T_0(ω, ν)    = T_0,ref · (ν / ν_ref · |ω| / ω_ref)^(2/3) · sign(ω)
η_teeth      = 1 / (1 / η − i · T_0,ref / T_η)                at most 1
η(T2, ω, ν)  = T2 / (i · (T2 / (i · η_teeth) + T_0(ω, ν)))    the gearbox's efficiency map
```

T_0,ref `no_load_torque_nm` at ω_ref `no_load_speed_rpm` and 20 °C (`thermal_feedback.reference_c`), as makers quote it (Neugart: "average idle torque"; WITTENSTEIN: T012 "at n1 = 3000 rpm and 20 °C gearbox temperature"; Harmonic Drive: charts over speed and temperature). The exponent 2/3 is Palmgren's law for the drag of rolling bearings in oil, M0 ∝ (ν n)^(2/3) (SKF's rolling bearing catalogue), which carries both speed and viscosity. Harmonic Drive's chart of a size 32 gear at 2000 r/min falls from about 50 to 27 N cm from 20 to 40 °C, a factor 0.54; the law with B = 4000 K gives 0.56. With the drag the configured efficiency η (`gearbox.efficiency`) holds at the output torque T_η `efficiency_torque_nm` at ω_ref and 20 °C: the teeth's own efficiency η_teeth is what is left of it. Below T_η, at speed and in the cold the drag takes a larger share, so the efficiency falls, as the makers' charts show (Siemens' gearbox manual: in the partial load range, especially cold, the efficiency is always lower). This one map serves planetary, strain wave and worm gears; its points are tested in `the_efficiency_map_falls_at_part_load_slow_speed_and_in_the_cold_as_calculated`. With the drag on, [temperature feedback](#temperature-feedback) takes the viscous share of the efficiency as 0: the drag is that share.

- **Stiffness and backlash.** Inside the dead zone nothing is transmitted. With `effects.gearbox_backlash` off, β = 0.
- **Flanks cannot pull.** With backlash on, if the damping term would reverse the torque while the flanks separate (τ · engaged < 0), the torque is zero. With backlash off the connection is a plain spring-damper and can pull, as a coupling does.
- **Efficiency.** The loss is on the output torque. When the gearbox drives the load (power positive) the output gets τ · η; when the load drives back it gets τ / η. The exponent fades with tanh below 0.5 W of transmitted power, so a gearbox at rest has no torque step. With `effects.gearbox_efficiency` off, η = 1.
- **Mesh friction** (`effects.gearbox_mesh_friction`, in place of the faded factor). The efficiency's loss is a friction in the mesh, at the output against the housing, in proportion to the torque the gear passes and against the output's motion at any speed, and it sticks at rest:

  ```text
  τ_s     = sign(twist) · f(e) + z                          what the spring and hysteresis pass (the damping left out)
  μ       = 1 − η                                           η the teeth's efficiency (with the drag: η_teeth)
  F       = clamp(σ_m · (θ − a), −μ |τ_s|, +μ |τ_s|)        σ_m = K1, a the anchor where the last step ended
  τ_out   = τ − F
  a      ← θ − F / σ_m                                       at each step's end (`Gearbox::commit`)
  ```

  Sliding forward the output gets τ_s (1 − μ) = η τ_s, as with the efficiency; driven back it takes τ_s (1 + μ), so the back-driving efficiency follows as 1 / (1 + μ) = 1 / (2 − η): 0.833 at η = 0.8, where the faded factor gives η either way. At rest a gear holding a load L stands while τ_s lies between L / (2 − η) and L / η: it holds anything in between, and breaks away lifting at L / (i η) at the motor and lowering at L / (i (2 − η)) (`a_held_gear_stands_inside_its_friction_band_and_breaks_away_at_its_ends`). Around zero load the friction μ K |x| takes, from one turning point A₁ to the next, μ K (A₁² + A₂²) / 2 of the spring's K (A₁² − A₂²) / 2: the friction law alone keeps (1 − μ) / (1 + μ) of the amplitude a cycle, ζ = ln((1 + μ) / (1 − μ)) / 2π = 6.45 % at η = 0.8 (`a_ring_around_zero_load_decays_by_the_hand_calculated_damping`, with a stiff bristle). With the bristle of K1 the friction first turns round by sticking on 2 K, from the bound at A to the other at x₁ = A (1 − μ) / (1 + μ), then slides on K (1 − μ) and K (1 + μ): A₂² / A² = ((1 + μ²)(1 − r²) + (1 − μ) r²) / (1 + μ) with r = (1 − μ) / (1 + μ), 7/9 a half cycle at η = 0.8, so a cycle keeps 7/9 and ζ = ln(9/7) / 2π = 4.0 % (`the_shipped_bristle_rings_down_by_its_hand_calculated_ratio`), where the faded factor leaves a small ring (below its 0.5 W) about 1 %. With the no-load running torque (`effects.gearbox_drag`) the two make the efficiency map above: the catalogue's η at its rated point, less at part load (`mesh_friction_and_drag_give_the_catalogue_efficiency_and_less_at_part_load`: 0.9 at 50 N m, 0.774527 at 5 N m). The friction's power F ω is booked as its loss, where the gearbox's loss goes (its loss row, `part_heating`, `thermal_feedback`, which sets η and so μ); the bristle's store F² / (2 σ_m), at most (μ τ)² / (2 σ_m) (4e-7 J on the planar cable robot's winches), is counted with it and not in `Gearbox::energy`. Booking the store's rate instead, sticking or sliding as decided at a step's end, missed 1.80 J of the planar cable robot's 38.17 J: within a step the anchor stands and the friction slides at its bound throughout, also where the ends said it sticks. Shortcuts: σ_m is K1, as the hysteresis's Dahl stiffness (presliding takes μ of the twist). The energy budget does not choose it: with the friction booked as its whole power the belt axes, whose light pulleys stick to the housing first, close their passes at K1, 3 K1 and 10 K1 alike (the single axis under 1e-6 of its pass unaccounted, the T-gantry 4e-6, as with the friction off; gearbox loss per pass 3.04659 J on the single axis at all three, 8.70004, 8.70027 and 8.70029 J on the T-gantry, against 3.04944 and 8.82445 J with the friction off); the 1.4 and 2.0 % they missed at 10 K1 were the earlier bookkeeping's. It is off on the belt machines as shipped, and on in the arms, cable robots and mover. Behind a screw, rack or drum (`Converter::apply_to`) σ_m is capped where the bristle on the lighter of the rotor's inertia seen at the load and the output's own would settle faster than a quarter of the bristle rate limit (2e8 N/m for the cable robots' 0.32 kg drums; no shipped gear reaches it); before a gearbox's first step and in a machine that does not commit its gearboxes after each step (the four-arm delta's arms, the gearboxes of a rigid model) the faded factor stays; the rigid models' gearboxes keep it.
- **At rest** under an output torque the twist is τ / k_g + β/2 on the matching side; without one it is 0, the middle of the backlash (`Gearbox::static_twist`).

Assumptions: the hysteresis's stiffness at a reversal σ is taken as K1 (`shortcut`: no maker publishes the loop's shape at a reversal), so the D3 and D2 gearboxes (half an arcmin) are twice as stiff at small amplitudes: the D3's velocity loop crosses over at 305 Hz with it and 259 Hz without (the old rate-switched friction locked it at 19 Hz). Inside one base step a twist that turns back follows the curve from the step's start, not Dahl's reversal (the steps are 62.5 µs). The machine's state vector does not carry z: a state set from outside (a crash's stop, `adopt_state`) keeps the gearbox's anchor from before. The lost-motion band is in the gearbox's angle; behind a screw or rack its stretch by the converter's spring is left out. The drag acts on the motor shaft with its friction, but its loss is the gearbox's (`ShaftFriction::drag_power`): it is in the gearbox's loss row and with `part_heating` it warms the gearbox (`the_gearbox_drag_warms_the_gearbox_to_its_resistance_times_the_drag_power`: the single axis cruising at 250 rad/s at the motor, 0.107342 N m of drag, 26.8355 W, 53.671 K at 2 K/W). A worm's tooth efficiency does not fall with sliding speed. The tooth efficiency is at most 1: where the catalogue's no-load torque at its reference speed is more than its whole loss (the SP+ 100: 0.93 N m at 3000 r/min, where 6 % of 259 N m at ratio 20 is 0.78 N m), the gearbox reaches 93.3 % instead of 94 % at T2N and 20 °C. The gearbox's own inertia (`gearbox.inertia_at_input_kgm2`) turns with the rotor: J_rotor in the equations is `motor.rotor_inertia_kgm2` plus it.

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

Assumptions: no three phases, no PWM, no inverter dead time. Field weakening (the winding as the q axis of a d-q model) and iron loss are effects of their own, below ([field weakening](#field-weakening)). Below the current loop there is nothing.

### Torque saturation

`effects.torque_saturation`: the torque constant falls with the square of the current to `torque_constant_at_max_current_nm_a` at `max_current_a`:

```text
τ = k_t · i · (1 − s · min(1, (i / i_max)²)),      s = 1 − k_t,max / k_t
```

The current setpoint is the inverse of this curve, limited to `max_current_a` (Newton's method from below; exact to rounding). So the motor gives the torque that is commanded, up to its maximum torque at its maximum current. A curve that loses more than a third of its torque constant (s > 1/3) turns down before the maximum current; the setpoint then stops at the top of the curve, i_max / sqrt(3 s).

The back-EMF constant falls with the torque constant: e = k_e · ω · (1 − s · min(1, (i / i_max)²)). The electrical power u · i is then the copper loss plus the shaft power, with saturation as without.

**A back-EMF constant of its own.** `drive.back_emf_constant_v_s_rad` (optional; absent or 0: equal to the torque constant, the shipped configs' behaviour) is the maker's printed voltage constant in the one winding's scale (√3 times the line-to-line RMS constant, as `catalog` converts it). The servo winding then has k_e ≠ k_t: the winding equation, the back-EMF feedforward of the current loop, the voltage limits (`voltage_limited_current`, field weakening's `d_current_for` and `field_weakened_torque`) and the DC link's power u · i use k_e, the torque stays k_t i. The electrical power is then the copper loss plus the shaft power plus (k_e − k_t) · i · ω (times the saturation and magnet factors both constants share). A real motor has one flux linkage: the two printed constants differ because they are measured apart, the voltage constant at no load with the magnets cold (about 20 °C), the torque constant at the rated point with the magnets warm (their flux falls by about 0.1 % per kelvin) and with the iron loss and saturation of the load taken off the shaft. (k_e − k_t) · i · ω is therefore not a loss the motor has but the model's bookkeeping of using both constants at once: the voltage side (conservative for the voltage limits) and the torque side each as printed. The drive books it as `Drive::emf_loss` (the winding's step energies less copper, shaft and stored energy, exact) and the energy budget shows it as the row `emf.<motor>` next to the copper loss, only where a constant is set apart from k_t; it is signed as i ω is: motoring it is positive (it then stands roughly for the iron loss), braking negative, the winding returning (k_e − k_t) |i| ω more than the shaft gives, which no real motor does; over a cycle the row is (k_e / k_t − 1) times the shaft's net work (the single axis with k_e 15 % above k_t: 11.34 J of a 90.50 J supply, the budget closing to 2e-9). It does not warm the winding model (the rated rise of the catalogue holds the real losses already). A stepper keeps one constant for both and an induction motor has none: the key is read for the servo only. Test: `drive::tests::a_back_emf_constant_apart_from_the_torque_constant_costs_voltage_and_books_the_difference` (k_e 0.5, k_t 0.45, R 4 Ω: 2.25 N m at 340 rad/s on 190 V where equal constants give 4.1625 N m; 100 rad/s at 2 A takes 116 W = 16 W copper + 90 W shaft + 10 W loss).

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
u_wanted = K_pi · e_i + I_i + e                    the drive adds the back-EMF e it expects at its measured speed ω_f:
                                                   e = k_e · ω_f · (1 − s · min(1, (i / i_max)²)), saturated (below) at
                                                   the winding's current i of the step's start, not the measured one
u        = clamp(u_wanted, −U_q, +U_q)             U_q = sqrt(U_max² − u_d²): the q axis gets what the d axis leaves of the voltage circle
u_applied = u, `drive.current_delay_cycles` base steps later    (effects.drive_delay; 0 without)
I_i     += (K_pi / T_ni) · e_i · dt                only while u is not clamped (anti-windup)
```

The d axis comes first. Its steady voltage is u_d = R · i_d − p · ω · L · i, the reactance of the q current with p the pole pairs (`motor.pole_pairs`; no reactance where the config has none) and, with `effects.field_weakening`, the d current i_d of the winding's own model (below). Without field weakening i_d = 0 and the winding meets (R i + k_E ω)² + (p ω L i)² ≤ U_max² at most (the torque-speed chart's voltage limit, `DriveParams::voltage_limited_current`; the kt = kE = 0.45, R = 4 Ω, L = 10 mH, 4 pole pairs, 190 V test winding has 3.474 N m at 300 rad/s, 2.709 at 330 and 1.956 at 360 where the reactance-free R i + k_E ω ≤ U gave the current limit's 4.275 at 300), and braking at the peak torque is held to the current that circle allows (beyond it the q voltage cannot oppose the back-EMF and the current runs away). With field weakening the d current follows its setpoint with the voltage that moves it, held within the circle first (d axis first), and the q current within the phase current's circle, i_q² + i_d² ≤ min(motor, drive maximum current)². `motor.<id>.voltage` is, for every motor kind, the amplitude of the phase voltage's space vector: the induction motor's and the stepper's vectors are phase amplitudes, the servo's (u, u_d), in the winding's units of three times the RMS phase voltage, is multiplied by sqrt(2) / 3 (a DC link U gives sqrt(3/2) · U at most, which is U / sqrt(3) at the phase).

U_max is sqrt(3/2) times `drive.bus_voltage_v`, or sqrt(3/2) times the DC link voltage of the moment with `effects.dc_link`, or unlimited with `effects.voltage_limit` off. The factor: the one winding carries the RMS phase current I, and u · i is the power of all three phases, 3 · U_phase · I, so u is three times the RMS phase voltage. A DC link U gives a phase amplitude of U / sqrt(3), which is U / sqrt(6) RMS, and 3 / sqrt(6) = sqrt(3/2). `drive.bus_voltage_v` is the DC link voltage itself, and the DC link's energy bookkeeping below works with it unscaled. With K_pi = L / T and T_ni = L / R the current follows its setpoint with the time constant T (0.25 ms in the default configs).

### Velocity loop

Every velocity cycle T_v:

```text
ω_raw    = (encoder − encoder_before) / T_v        or the true speed with an ideal encoder
ω_f     += α_v · (ω_raw − ω_f)                     α_v = 1 − exp(−T_v / velocity_filter_s); 1 with the filter off
ω_cmd    = K_v · e_pos + w_v · (ω_set + α_set · t) speed setpoint interpolated between profile cycles
e_ω      = ω_cmd − ω_f
τ_wanted = K_p · e_ω + I_ω + w_τ · τ_ff + τ_fr(ω_set + α_set · t)
                                                   PI part and feedforward part are the signals
                                                   motor.<id>.torque_feedback and .torque_feedforward
τ_filt  += α_τ · (τ_wanted − τ_filt)               torque setpoint filter
τ_notch  = notch(τ_filt)                           with effects.notch_filter
τ_cmd    = clamp(τ_notch + τ_inj, torque range at ω_f), `drive.velocity_delay_cycles` velocity cycles later
                                                   (effects.drive_delay; τ_inj is the analysis's test injection, 0 otherwise)
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

`effects.drive_overload` (off in the shipped configs; read as off where a config has no switch): SINAMICS drives give their maximum current only for a short time and then fall back to their base load current. Since 14 November 2026 the image has two nodes, as the SINAMICS power units model a slow heat sink and the fast chips on it: x = x_s + x_c, x_s' = ((1 − w) q − x_s) / τ_drive, x_c' = (w q − x_c) / τ_c, q = (i / I_rated)², w = `ratings.drive_chip_share` and τ_c = `drive_chip_time_constant_s` (`[estimate]`s, 0.3 and 1 s; w = 0 is the one node below). The trip level becomes r² − (r² − 1) ((1 − w) exp(−t_max / τ_drive) + w exp(−t_max / τ_c)); `repeating_peak` repeats each node by itself. Hand calculation: `the_two_node_overload_image_trips_after_the_hand_calculated_time` (3 A module, 9 A for 0.25 s: trip 1.66914, at 6 A from the rated load after 0.8203 s, where one node takes 0.6811 s). The drive keeps a thermal image of its power unit (i²t), stepped exactly every base step with the RMS phase current i (with field weakening the hypotenuse of q and d current):

```text
x' = ((i / I_rated)² − x) / τ_drive            x = 1 at the rated current held
trip = r² − (r² − 1) · exp(−t_max / τ_drive)   r = I_max / I_rated: the maximum current from the rated load for t_max
x ≥ trip  → limit = I_base, until x ≤ 1 again
```

I_rated, I_max and I_base are `ratings.drive_rated_current_a`, `drive_max_current_a` and `drive_base_load_current_a` (a drive by name sets them: an S120 Motor Module's base load current is the catalogue's, about 0.85 of its rated current; Siemens publishes none for the S210, so 0.85 of its rated current is taken, `[estimate]`); τ_drive = `drive_overload_time_constant_s` and t_max = `drive_max_current_time_s` are `[estimate]`s (10 s and 0.25 s in the shipped configs: the Siemens load duty cycles are not in the catalogue files). While tripped, the velocity loop's torque range is narrowed to the torque of I_base, as for the drive current limit above.

`drive-overload` judges the image's peak when the cycle repeats for long against the trip level. The image is linear in its start: one started at x₀ lies (x₀ − x_start) · exp(−t / τ) above the measured pass's at every t, and the start a repeating pass returns to is x₀ = f / (1 − exp(−T / τ)), with f = x_end − x_start · exp(−T / τ) what the pass added (`analysis::repeating_peak`; sampled once per millisecond). Where the image tripped in the pass the drive was not linear any more; the check fails there anyway.

Tests: `a_step_at_twice_the_rated_current_clamps_after_the_hand_calculated_time` (drive.rs: 3 A rated, 2.6 A base load, 9 A for 0.25 s, τ = 10 s: trip 1.19752; 6 A from cold clamps to 2.6 A after −10 s · ln(1 − 1.19752 / 4) = 3.558 s, and the image is back at 1 after another 5.842 s), `the_overload_image_of_a_repeated_cycle_peaks_at_the_hand_calculated_level` (1.5 I_rated held: 2.25; 2 I_rated for 0.5 s of 1 s: 2.04999).

w_v and w_τ are the feedforward factors (0 with the switch off). The notch is a biquad with centre `notch_frequency_hz` and Q = frequency / bandwidth.

**Margins.** `Drive::torque_injection` is a test torque d added to τ_notch before the limit; a drive's own measurement function works the same way. With the machine holding its start pose, a sine d on one motor gives the loop's answer u = τ_notch and the torque that acts e = u + d, both correlated with sine and cosine at the velocity cycle over whole periods (at least 0.1 s to settle, 0.1 s to measure). Then L = −U / E is the open-loop response at that motor, with everything in it: velocity filter, PI, torque filter, delays, current loop, the elastic mechanics (the other motors' loops closed) and the position loop around it; and |S| = |E| / |D| = 1 / |1 + L|. `analysis::loop_margins` follows the phase of L along the frequency axis without jumps and shifts it by whole turns so that it lies between −360° and 0° at the first gain crossover (without a crossover: between −270° and 90° at the lowest point). It reads off, interpolated in log |L| and phase between points, the phase margin 180° + ∠L at each gain crossover (the smallest counts; negative: unstable) and the peak of |S|. Stability is a Nyquist count of the crossings of the negative real axis (the phase through −180° or an odd multiple) beyond −1, |L| > 1: a falling one is an encirclement of −1, a later rising one undoes the last falling one not yet undone. The count starts at a phase of 0 at the gain of the lowest point, as the small circle of the Nyquist contour around s = 0 turns the phase by −90° per integrator, so a loop whose integrators lag past −180° below the measured range counts that crossing too. The loop is stable when no falling crossing is left. The gain margin is −20 log |L| at the worst falling crossing not undone: those with |L| < 1 (in practice above the crossover) give a positive margin, one left with |L| > 1 a negative one (L encircles −1, unstable). A falling crossing beyond −1 that a rising one undoes (the phase turned past −180° and back below the crossover) is the conditionally stable kind: a gain that falls would be the danger there, and the margin for that is not given. A point at which the torque limit was hit, or at which the loop moved on its own (what of e is neither the injected sine nor a constant is larger in RMS than the injection), is no measurement of L: it is NaN, the margins are not valid and NaN (`LoopMargins::valid`, `stable`); `sim-cli margins` says `FAIL` for an invalid or unstable loop. Tests: `margins_are_signed_and_say_an_unstable_loop_is_unstable` (L = ωc e^(−jωτ) / (jω): 100 Hz, 0.5 ms: 72°, 13.979 dB; 200 Hz, 0.5 ms: 54°, 7.959 dB; 300 Hz, 1 ms: −18°, −1.584 dB; 400 Hz, 1 ms: −54°, −4.082 dB), `a_conditionally_stable_loop_is_stable` (L = K (1 + s/a)² / (s (1 + s/b)² (1 + s/c)), a, b, c = 2π · 300, 30, 3000 Hz: stable at K = 3e5, 6.1e5, 1.2e6 with 30.7°, 41.6°, 42.4°), `a_lead_with_positive_phase_at_the_first_point_keeps_its_phase_margin` (a type-0 lead loop: 24.0° at 1516 Hz, not −336°), analysis; `an_unstable_velocity_loop_is_reported_unstable` (single axis at 0.3 N m s/rad), axis. `velocity_loop_margins` does this for every motor at 60 points from 10 Hz to a quarter of the velocity loop's rate (2 kHz), with a tenth of each motor's own rated torque, for the config and for each value of `[bounds]` at its low and at its high bound (`sim-cli margins MACHINE [--bands]`). The velocity gains of all three configs were chosen with it (see the [user guide](user-guide.md#tune-gains-and-use-input-shaping)). The measurement runs with `effects.gearbox_efficiency` off: at standstill the efficiency's fade with the transmitted power (η^tanh(P / 0.5 W)) acts as a large damper on the small test torque and would hide the loop. Without a gain crossover in the measured band the phase margin is NaN (not measurable), the gain margin infinite. Friction near standstill damps the loop at this amplitude (the motor's Coulomb friction is smoothed over 0.5 rad/s), so the margins are lower with friction off, as in a cruise.

### Position loop

Every position cycle T_p, t the time since the profile cycle wrote the setpoints:

```text
φ_target = φ_set + ω_set · t + α_set · t² / 2      effects.fine_interpolation; φ_set held without it
φ_cmd   += β · (φ_target − φ_cmd)                  β = 1 − exp(−T_p / precontrol_balance_s); 1 with it 0
e_pos    = φ_cmd − φ_measured                      motor encoder, or the load-side scale (effects.load_encoder)
K_v · e_pos is held until the next position cycle  signal motor.<id>.speed_from_position
```

One encoder sample per base step serves both loops: the position loop and the velocity loop of the same step see the same reading, with the same noise.

#### Fine interpolation and precontrol

A SINAMICS S120 or S210 in DSC (dynamic servo control) closes the position loop in the drive at the speed loop's cycle, with Kv, the controller's position setpoint and its speed precontrol, while the controller's interpolator (IPO) sends them every 2 to 8 ms. Between two setpoints the drive interpolates. `effects.fine_interpolation` does so here: the position setpoint runs on as pos + v t + a t²/2 from the last frame, the derivative of which is the speed feedforward the velocity loop already ran on as v + a t, so the two agree and the following error does not see the setpoint's steps. At the next frame the extrapolation is off by the jerk's j T³/6 (4 ms, 1000 rad/s³: 11 µrad), against v T of a held setpoint (100 rad/s: 0.4 rad). Siemens' cubic fine interpolator needs the next frame and delays by one IPO cycle; this one needs none and is as smooth where the acceleration is continuous. `drive.position_cycle_s` may equal `velocity_cycle_s`. Without the switch (a position loop in the controller, at the IPO cycle) the setpoint is held, and its steps of v T go through Kv into the torque (14 November measurements in PROGRESS).

The motor speed and acceleration setpoints come from the inverse kinematics at the setpoint pose moved a moment h = 1 ms along the plan, x ± v h + a h²/2: the first difference is the speed, the second the whole acceleration, J⁻¹ a plus the term d(J⁻¹)/dt · v that a kinematics that is not linear (the deltas, the arms) adds at speed.

The torque feedforward (`effects.torque_feedforward`, weight `drive.torque_feedforward_factor`) is, by `drive.torque_feedforward_model`:

- `rigid`: the rigid model's inverse dynamics at the setpoint pose, speed and acceleration, gravity included and coupled between the motors. What a controller with a dynamic model sends as torque precontrol. The shipped configs use it.
- `inertia`: J_k · α_set,k per motor, J_k the torque the rigid model needs for a unit acceleration of motor k alone at `start.pose` less what it needs at rest. What a drive's own speed/torque precontrol does with the load inertia entered (S120 p1498), as the commissioning of a belt axis or a gantry usually leaves it: no gravity (the speed loop's integrator takes it; on a vertical axis the controller often adds it as a supplementary torque), no speed terms, no coupling, one inertia for the whole workspace.
- off (`effects.torque_feedforward = false`): the speed loop alone.

`effects.friction_feedforward` adds the friction characteristic τ_fr at the speed setpoint, S120 p3820-p3839: points (ω_i, τ_i) at rising speeds, linear between them and from (0, 0) to the first, held beyond the last, with the sign of the speed. A real drive records them at ten constant speeds (p3845); a config takes the recorded points. With `drive.friction_feedforward_model = "machine"` it adds in place of the characteristic the friction of the machine's own model at the planned pose and speed (`Machine::steady_friction`), worked out by the controller every profile cycle and sent with the setpoints: the guides at their steady sliding level, faded in over the Stribeck speed so that a stop puts no step on the axis (LuGre: g(v) tanh(v / v_s) + b v, under the load at rest; `Lugre::feedforward`), the motor shaft's friction with the gearbox's drag, the joints' friction, idlers, the cable chain and the drums, each at its own speed through the kinematics' Jacobian and carried to the motor through the ratios, and through the gearbox's efficiency in the direction of what the gearbox passes at rest (friction plus the weight it holds, or a cable's tension). What it leaves out: the gearbox's efficiency on the inertia and gravity torques (the rigid feedforward is loss-free), extra guide load from acceleration, the brake and the stops, and damping, which does no work at steady speed. Every registered machine tells it, and so do the machine files' belt, screw and rack machines, linkages and deltas (a self-locking worm's drag left out); the scissor lift's file tells none. At a constant speed the velocity loop's integrator then carries nothing (`the_machine_friction_feedforward_is_the_steady_friction_and_the_integrator_carries_nothing`, single axis: 0.197548 N m at 1 m/s by hand; one such test per machine crate). Where it is on in the shipped configs, and what it does there, is in [validation](validation.md#friction-feedforward-in-the-shipped-configs).

`drive.precontrol_balance_s` balances the speed precontrol (S120 p1429): the position setpoint goes through a first-order lag before the position loop compares it. The speed feedforward moves the motor through the closed speed loop, which lags by about its equivalent time constant; delaying the setpoint by as much leaves the position loop less to correct. The following error is then the one against the balanced setpoint. The dead time p1428 is not modelled. Off (0) in the shipped configs, as a drive comes commissioned.

`effects.load_encoder` closes the position loop on a load-side measuring system (S120 direct measuring system): the tool axes as the elastic model has them, in steps of `drive.load_encoder_step`, turned into motor angles by the inverse kinematics. The speed loop stays on the motor encoder. Belt stretch, gear twist and backlash are then inside the position loop, and the following error is the load's.

Tests: `fine_interpolation_runs_the_setpoint_on_between_profile_cycles`, `the_friction_feedforward_is_the_characteristic_at_the_speed_setpoint`, `balancing_lags_the_position_setpoint_by_its_time_constant` (drive.rs); `the_constant_inertia_feedforward_is_j_times_the_motor_acceleration`, `a_load_side_encoder_measures_the_following_error_at_the_load`, `both_loops_read_one_encoder_sample_per_step`, `a_long_profile_cycle_with_fine_interpolation_keeps_the_peak_torque` (sim-machines control tests).

### Electronic cam

A machine that has a cam (`Machine::cam`, the crank and cam machines of `machine-crank-cam` with `cam.run = true`) runs it as
its cycle: the controller moves a master m from rest to its speed V by a sin² ramp of `cam.ramp_s` (m' = V sin²(π t /
2T), m'' = V π / (2T) sin(π t / T)), steady after, and down by the mirror ramp after a stop. The slave's setpoint is
s(m) + offset, its speed s'(m) m' and acceleration s''(m) m'² + s'(m) m'' (`cam::slave_motion`), s from
`sim_components::cam` (VDI 2143 segments or a cubic through a table). The offset keeps the slave where it stands when the
cam starts; the master keeps its position between runs. These setpoints go through the inverse kinematics, the torque
feedforward and the drives as every other motion's. One pass of the cycle is one master period once the master is up to
speed, so the analysis measures the second steady period. The cam's peaks at speed V, |s'| V, |s''| V² and |s'''| V³, are
checked against the axis limits (`limit-cam-*`). A mechanism's process force (`Mechanism::process_torque`: a press force,
a cut) is a generalized force at the joint outside the rigid model: in the dynamics, not in the feedforward, among the
rest in the torque breakdown, and its power given to the process is `power.process`.

### Torque-speed curve

```text
fade     = clamp( (ω₀ − |ω|) / (ω₀ − ω_rated), 0, 1 )
motoring : |τ| ≤ peak_torque · fade        in the direction of rotation
braking  : |τ| ≤ peak_torque
```

ω₀ `no_load_speed_rad_s`. Full torque up to the rated speed, falling linearly to zero at the no-load speed. With `effects.torque_limit` off the torque setpoint is not limited; the analysis still judges against the curve. What it judges is the velocity loop's output before this limit (`torque_demand`), so a drive that sits on the limit shows as a utilisation above 1. This curve is a separate, configured limit; the voltage limit of the winding acts as well. With field weakening the curve follows from the d-q model instead (below).

### Field weakening

`effects.field_weakening` (on in the shipped configs since 14 November 2026; read as off where a config has no switch): the permanent-magnet synchronous motor in d-q axes. The one winding above is the q axis, scaled as it is (current the RMS phase current, voltage three times the RMS phase voltage, so R, L and U_max are the winding's: three times the data sheet's phase values); the d axis is scaled alike. The flux linkage is ψ = k_e / p, from the torque constant; p = `motor.pole_pairs`, L_q = `motor.inductance_h`, L_d = `motor.d_inductance_h`. With the electrical speed ω_e = p · ω:

```text
u_q = R · i_q + L_q · di_q/dt + k_e · ω + ω_e · L_d · i_d
u_d = R · i_d − ω_e · L_q · i_q
u_q² + u_d² ≤ U_max²                    voltage circle (the DC link through the modulation, as above)
i_q² + i_d² ≤ I_max²                    current circle (the motor's maximum current)
τ = k_t · i_q + p · (L_d − L_q) · i_d · i_q
```

Above base speed the drive sets a negative d current: the least that brings the steady voltage within the circle for the q setpoint, from (c₀ + a · i_d)² + (R · i_d − b · i_q)² = U_max² with c₀ = R · i_q + k_e · ω, a = ω_e · L_d, b = ω_e · L_q (`DriveParams::d_current_for`). The q axis's current controller adds ω_e · L_d · i_d to the back-EMF it expects and gets what u_d leaves of the voltage, sqrt(U_max² − u_d²). The d current is held over the base step like the voltage: u_d · i_d joins the electrical power, R · i_d² the copper loss (and so the heating), the reluctance torque the shaft, so u_q · i_q + u_d · i_d = R (i_q² + i_d²) + L_q · i_q · di_q/dt + τ · ω still closes. The drive's thermal image, the RMS current of the analysis and `drive-current` see the phase current sqrt(i_q² + i_d²).

The torque-speed curve is then the most torque within both circles at each speed, over d currents from 0 to −I_max (golden section), up to `drive.peak_torque_nm`, motoring and braking apart; `motor-peak`, the report's curve and the app's plot use it. Towards high speed the best d current tends to the characteristic current −ψ / L_d.

Catalogue: Siemens publishes R, L, k_E and the pole pairs of the 1FK7 and 1FT7 (`motors_s120_electrical.csv`, `motors_s120.csv`); a motor by name sets `pole_pairs` and takes L_d = L_q (surface magnets, `[estimate]`). For the 1FK2 and 1FT2 none of these is published (not in the S120 configuration manuals 02/2022, the S210 operating instructions 06/2020 and 09/2022, nor the configurator's data sheet per article); the config's `[estimate]`s stay.

Tests: `field_weakening_gives_the_hand_calculated_torque_above_the_voltage_limit` (drive.rs: 0.45 N m/A, 190 V, 9.5 A, 4 pole pairs, 10 mH, R → 0, at 500 rad/s: the circles cross at i_d = −5.625 A, i_q = 7.6557 A, 3.4451 N m, where the fading curve has none; the current loop settles there with |u| = 190 V and the power balance closes), `the_d_q_model_holds_the_published_points_of_a_1fk7` (1FK7042-2AF7 at 600 V: the rated 2.6 N m at 2.0 A and 314.16 rad/s within 6 % and inside the curve without d current; torque left at n_max,inv = 670.2 rad/s, where the fading curve ends at 538.9 rad/s; the best d current at 942.5 rad/s within 1 % of −ψ / L_d = −3.247 A). Siemens' torque-speed curves themselves are printed only as graphs and are not in the catalogue files, so the model is not checked against curve points.

Assumptions: the d current follows its setpoint at once (no d-axis winding state); the back-EMF in the d-current choice and the curve is unsaturated; the velocity loop's torque limit and the current loop take the DC link's voltage of the moment (`Drive::max_voltage`), the planner's and the analysis's curves the nominal bus; no MTPA below base speed for L_d < L_q beyond what the curve's search finds; `check_limits` judges the axis limits against the maximum speed and the curve at that speed, as a warning only (see [the axis-limit check](#axis-limits-and-the-motors)).

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


### Motor model additions of 14 November 2026

**d-axis dynamics and warm resistance.** With field weakening the d current no longer jumps to its setpoint: it follows it as the tuned current loop does, with the time constant L_d / K_p (the controller's T_n = L / R cancels the winding's own), exactly per base step, as far as the voltage circle lets it: the d voltage that moves it is held within U_max (d axis first), and the q axis gets the rest. A q setpoint beyond what the two circles leave together (above the curve) gets the d current of the most q current that fits, not one that fills the current circle and leaves no torque. The d current's setpoint and u_d take the winding's resistance at its temperature, as the winding integration does. Tests: `the_d_current_follows_its_setpoint_with_the_current_loop_time_constant` (0.25 ms: −3.5289 A of −5.625 A after four cycles, the first cycle's 199 V held to the 190 V circle; −3.5557 A unbounded), `the_d_current_takes_the_warm_resistance`, `braking_at_the_voltage_limit_does_not_run_away` (the audit's −19.1 N m at 42.5 A at 400 rad/s on the old code, now within 3.8 N m, 9.5 A and 190 V at 380 to 450 rad/s, motoring and braking, saturation on and off).

**Voltage limit from R, L and the DC link.** `the_voltage_limit_gives_the_hand_calculated_torque_from_r_l_and_the_dc_link`: the 1FK2205-4AF at 540 V (R = 2 Ω, L = 15.67 mH from Siemens' short-circuit braking table, 4 pole pairs) has 15.614 N m at 600 rad/s and 17.818 N m at 500 rad/s by a hand search over the d current, where the old linear fade gave none and 1.595 N m. The current loop and the velocity loop's torque limit use the DC link's voltage at each moment (`DriveParams::torque_range_at`), so a sagging DC link limits the torque at speed: `a_dc_link_ten_percent_low_lowers_the_velocity_loops_torque_limit_at_speed` (the field weakening test's motor at 500 rad/s, 171 V of 190: i_d = −6.38711 A, i_q = 7.03241 A, 3.16459 N m against 3.4451 N m; without field weakening the fade's speeds scale with the voltage, 1.91195 against 3.22642 N m at 330 rad/s). The planner's curve uses the nominal bus.

**Iron loss.** `effects.iron_loss` (on in the shipped configs): Siemens' static data (M_0 at I_0) and rated data (M_N at I_N, n_N) both hold for the rated rise of 100 K, so at the rated speed iron and the other speed-dependent losses take P_Fe,N = R(ϑ_rated) (I_0² − I_N²). Over speed P_Fe = P_Fe,N (n / n_N)^1.5 (hysteresis with the frequency, eddy currents with its square; one exponent for both is a `shortcut`). The copper loss is then referred to the stall current, at which it alone brings the winding to its rated rise. The iron loss heats the one node, or with `motor_thermal_nodes` the housing's; it is not a drag torque on the shaft (`shortcut`). Test: `iron_loss_at_a_speed_is_the_hand_calculated_share_of_the_rated_loss` (R 4 Ω, I_0 3 A, I_N 2.5 A: 15.29 W at the rated speed, 1.911 W at a quarter; settled 100 K at the rated current and speed, and at the stall current at standstill).

**Magnet temperature.** `effects.magnet_temperature` (off): k_t and k_e scale with 1 + α (ϑ − ϑ_rated), α = `motor.magnet_temperature_coefficient_1_k` (−0.0012/K), ϑ the winding's rise (the housing's with `motor_thermal_nodes`). The catalogue's k_t holds at the rated rise; a cold motor has 12 % more. With `motor_thermal_nodes` the magnets sit at the housing's rise, which at the rated state is (1 − s) times the rated rise, and the flux is compared with that: 1 at the rated state (it compared the housing's rise with the winding's rated one before, 1.03 at s = 0.25). Test: `cold_magnets_give_the_hand_calculated_torque_per_amp`.

**S1 check.** `motor-s1` (servo motors only) judges the RMS torque at the mean speed against the S1 curve: the maker's graph where the config has it (`motor.s1_curve_*`; an S210 motor named in `[parts]` brings the one `catalog/siemens/s1_curves_s210.csv` has for its supply's voltage, 88 of the 96), else the line from k_t I_0 at standstill to the rated torque at the rated speed; `motor-speed` the peak speed against `drive.max_speed_rad_s`. Test: `the_s1_check_judges_the_rms_torque_at_the_mean_speed_by_hand` (1FK2205-4AF: 5.4832 N m at 157.08 rad/s on the graph's curve, 5.3 N m on the line).

**Braking resistor heating.** `effects.braking_resistor_heating` (with `dc_link`): ϑ' = (ϑ_rated P / P_continuous − ϑ) / τ_R, stepped exactly. Test: `the_braking_resistor_warms_by_the_hand_calculated_rise_in_a_pulse` (100 kW for 1 s on the 1.5 kW resistor, τ_R 60 s, 200 K: 220.38 K, 72.15 K at the end of its 68 s cycle).

**Line module loss when drawing.** With the DC link simulated, the line module (an S210's rectifier alike) draws P / η from the line for P into the DC link; the difference is its loss (`Drive::line_loss`, budget row `line_module`). Test: `a_regenerative_supply_feeds_back_into_the_mains_up_to_its_power` (49 W motoring: 50 W drawn, 0.1 J lost in 0.1 s).

**Enable without DC link.** A drive enabled while holding a load draws its winding's L i² / 2 from the supply in its first step (`Drive::enable_energy`); with the DC link simulated it comes out of the link, as before.

Signals per motor: `d_current`, `housing_temperature`, `drive_overload_image`, `drive_overload_chip`, `iron_loss`, `braking_resistor_temperature` (0 where their effect is off).

**Regenerative supply.** `effects.regenerative_supply` (with `effects.dc_link`, off by default): the line module, an Active Line Module as on a SINAMICS S120, holds the DC link at the bus level from above as the rectifier does from below. What the motors feed back goes into the mains up to `ratings.line_module_max_power_w`; of it `line_module_efficiency` arrives there (`Drive::mains_power`) and the rest is the line module's loss (`Drive::line_loss`). Only what is beyond its power charges the capacitors and reaches the chopper. The shipped values are the smallest S120 Active Line Module of the catalogue (16 kW rated, 35 kW maximum) and an efficiency of 0.98 from its 0.28 kW loss at rated power. Hand calculation: `a_regenerative_supply_feeds_back_into_the_mains_up_to_its_power` (41 W fed back: 4.018 J into the mains and 0.082 J lost in 0.1 s at 190 V; limited to 20 W, the other 21 W charge 470 µF to 212.217 V); on every config the energy fed back is the efficiency times what the motors regenerated (`the_regenerative_supply_feeds_back_what_the_motors_regenerate`).

**Disabled.** A disabled drive's DC link discharges through its bleeder and the drive's own consumption, C dU/dt = −U / R_b: every step U ← U · exp(−dt / τ) with τ = `ratings.dc_link_discharge_time_constant_s`, exact for any step, then the step above with P = 0, so the rectifier catches it at the bus level and a regenerative supply takes what is above at once (`a_disabled_drives_dc_link_discharges_by_the_rc_exponential`: 220 V with τ = 0.2 s are 171.336 V after 0.05 s and the bus level after 69.9 ms).

Assumptions: the rectifier is an ideal source at the bus voltage; no ripple; the chopper is an ideal power limiter up to `braking_resistor_peak_w`, not a switched resistor; the resistor itself does not heat (its continuous rating is judged against the cycle's mean power); the line module is an ideal power limiter with one efficiency at every power and feeds in without loss (its efficiency counts only on the way back); the bleeder draws only while the drive is disabled (enabled, its power, U² / R_b ≈ 0.1 W at 155 V, is left out).

**Shared DC link.** `effects.shared_dc_link` (with `effects.dc_link`): the drives of a machine have one DC link, as with one Line Module on a SINAMICS S120. After the current loops of all drives, one step of the same energy balance runs with the sum of their powers:

```text
E = Σ ½ C U_k² − Σ P_k · dt        C_link = n · C
```

then chopper, rectifier and trip as above, once, with one resistor of `braking_resistor_peak_w`. Every drive then has the link's voltage and reports the resistor's power and the trip as its own, so the braking checks show the same shared values for each motor; `dc-link-energy` compares one drive's share of the stored energy with one drive's share of the room, which is the same ratio as for the whole link. What a braking motor feeds back, a motoring one draws: only the net power charges the capacitors. Switched on while the machine runs, the sum above puts the energies of links at different voltages together (the voltages step to the common one; no energy appears or is lost). Assumptions beyond those above: the capacitance is the sum of the drives' (a Line Module's own is not counted), and the busbars have no resistance.

### Stepper motor

`motor.kind = "stepper"`. Source: [`stepper.rs`](../crates/sim-components/src/stepper.rs). A two-phase hybrid stepper in its usual sinusoidal form behind an open-loop step and direction drive. With N rotor teeth (`motor.rotor_teeth`, 50 for 1.8°) and the rotor's electrical angle e = N (θ − θ₀), counted from where the drive was enabled:

```text
τ         = k_t (i_b cos e − i_a sin e) − T_d sin 4e        T_d: effects.detent_torque, motor.detent_torque_nm (4 N periods a turn)
L di_a/dt = u_a − R i_a + k_e ω sin e
L di_b/dt = u_b − R i_b − k_e ω cos e
```

R, L and k_t are a phase's (`resistance_ohm`, `inductance_h`, `torque_constant_nm_a`), k_e = k_t (0 with `effects.back_emf` off). Both phase currents are integrated with the mechanics as the servo's winding is; the electrical power is u_a i_a + u_b i_b, the copper loss R (i_a² + i_b²), the shaft work the electromagnetic torque's (the detent's is the magnets' and left out: over a full step it is nothing, within one at most T_d / 2N).

**Drive.** The step generator runs the position setpoint on between profile cycles (pos + v t + a t²/2; in velocity mode or a fault's stop ramp it integrates the speed setpoint) and counts it in microsteps of 2π / (4 N `drive.microsteps`). The current vector of amplitude I = `drive.stepper_current_a` stands at the commanded electrical angle e_c: i_a = I cos e_c, i_b = I sin e_c, so τ = k_t I sin(e_c − e): the rotor sits on the magnetic spring of the holding torque T_hold = k_t I, at the load angle δ = e_c − e. The small-angle stiffness is N T_hold, the resonance sqrt(N T_hold / J) / 2π. Beyond the pull-out angle δ = π/2 the spring weakens; past π the rotor falls into the next equilibrium, four full steps (one electrical turn) on, and the open-loop drive goes on counting: the position is lost for good. There is no speed loop: the drive takes no torque setpoint (torque mode is refused, `error-open-loop`).

The chopper sets each base step the voltages that bring both currents to their setpoints by the step's end against the back-EMF of the step's start (the exact solution over the step, as a current-regulated chopper does over its PWM periods), the voltage vector held within the supply U (`drive.bus_voltage_v`, the simulated DC link with `effects.dc_link`). So the current falls short at speed and the torque with it.

**Pull-out curve.** In steady state at the electrical frequency N ω, with phasors (the back-EMF E = k_e ω as the real axis, Z = R + j N ω L): τ = k_t Re(I) and V = Z I + E. The pull-out torque is k_t times the highest Re(I) within |I| ≤ I and |Z I + E| ≤ U, the rightmost point of the intersection of two discs (braking: E with the other sign). It is the holding torque up to (R I + k_e ω)² + (N ω L I)² = U² and falls above. It is the stepper's torque-speed curve: "motor-peak" judges the delivered torque against it (open loop nothing asks for a torque), and the chart shows it.

Assumptions: sinusoidal phases (no full-step or half-step current shapes, no square-wave overmodulation, which gives up to 4/π more at the top of the curve); no standstill current reduction; the electrical zero is put on the rotor at each enable; no iron loss; the detent torque's shape is its first harmonic. Tests (`stepper::tests`): holding torque k_t I and the load angle asin(0.99) under 0.99 of it; the resonance 112.5 Hz of N = 50, T_hold = 1 N m, J = 1e-4 kg m² within 1 %; the corner speeds 31.89 rad/s (current, unloaded) and 39.70 rad/s (torque); a load of 1.3 T_hold slips by whole multiples of four full steps; jerk-limited ramps at 80 % of the pull-out torque keep in step and at 110 % fall out, below the corner and at 55 rad/s (0.793 N m by hand).

### Induction motor

`motor.kind = "induction"`. Source: [`induction.rs`](../crates/sim-components/src/induction.rs). The standard d-q model with stator and rotor flux as states, in the frame the converter turns at the electrical speed ω_k, amplitude-invariant (a vector's length is a phase's amplitude), p pole pairs, ω_r = p ω:

```text
dψ_s/dt = u_s − R_s i_s − j ω_k ψ_s           ψ_s = L_s i_s + L_m i_r,   L_s = L_m + L_σs
dψ_r/dt = −R_r i_r − j (ω_k − ω_r) ψ_r         ψ_r = L_m i_s + L_r i_r,   L_r = L_m + L_σr
τ       = 3/2 p (ψ_s × i_s)                    P = 3/2 u_s · i_s,   copper loss 3/2 (R_s |i_s|² + R_r |i_r|²)
```

R_s is `motor.resistance_ohm` (warming with `effects.winding_heating`), R_r, L_m, L_σs, L_σr are `motor.rotor_resistance_ohm`, `magnetizing_inductance_h`, `stator_leakage_inductance_h`, `rotor_leakage_inductance_h`. The four fluxes are integrated with the mechanics; the energy closes with the magnetic energy 3/4 (ψ_s · i_s + ψ_r · i_r). The drive's current is the RMS phase current |i_s| / √2; the phase voltage's amplitude is at most U_dc / √3.

**V/f** (`drive.induction_control = "vf"`): open loop, the frequency follows the speed setpoint, ω_k = p ω_set, and the voltage rises from the boost (`drive.vf_boost_v`, line-to-line volts at 0 Hz) to the rated voltage at the rated frequency (`motor.rated_voltage_v`, `rated_frequency_hz`) and stays there above. With `drive.vf_slip_compensation` the frequency is raised by the rated slip (from the name-plate speed `drive.rated_speed_rad_s`) times the estimated torque over the rated torque; the converter estimates the torque from its own air-gap power, 3/2 u · i less the stator's copper loss, over the synchronous speed, filtered with L_r / R_r. The rotor slips behind on the equivalent circuit's torque-slip curve; Kloss's formula τ = 2 τ_b / (s / s_b + s_b / s) holds when R_s is neglected.

**Field-oriented** (`"foc"`): the drive models the rotor flux, dψ̂/dt = (R_r / L_r)(L_m i_sd − ψ̂), turns its frame with it by the slip the model gives, ω_k = ω_r + R_r L_m i_sq / (L_r ψ̂), and controls i_sd for the flux (the rated flux up to the rated frequency, falling with 1/ω above: field weakening, and weaker only where the voltage and current circles leave no room at that flux for the torque asked for: the largest flux that does, `control_flux`) and i_sq for the torque τ = 3/2 p (L_m / L_r) ψ i_sq, both within the current limit (the smaller of `motor.max_current_a` and the drive's). PI current controllers with the decoupling the model gives: σ L_s di/dt = u − R_σ i + coupling, σ L_s = L_s − L_m² / L_r, R_σ = R_s + (L_m / L_r)² R_r; `drive.current_kp_v_a` = σ L_s / T and `current_tn_s` = σ L_s / R_σ give the current loop the time constant T. The servo's speed and position loops run on top.

**Torque curve.** The breakdown curve: the Thevenin form of the equivalent circuit with R_s, τ_b = 3/2 p / ω_e · U_th² / (2 (R_th + |R_th + j (X_th + X_σr)|)), at the V/f voltage (V/f) or the converter's (field-oriented, then also the current limit at the flux setpoint), and the peak torque `drive.peak_torque_nm`. V/f takes the maximum over the slip: the stator frequency runs ahead of the rotor by it, so the voltage is the V/f law's at ω_r + slip and the curve is the best torque of the circuit over that slip, motoring and generating apart (at 10 rad/s 17.95 N m motoring, where the breakdown at the rotor's own frequency gave 7.79; generating at the rated voltage and frequency −52.0 N m against −27.1). Field-oriented, the curve takes the flux of the most torque (`flux_setpoint`): the rated one up to the rated frequency and falling with 1/ω above, and weaker where the converter's voltage circle U_dc / sqrt(3) and the current circle leave more torque at a weaker flux, the top of ψ · i_q(ψ) (i_q the most both circles leave at ψ, i_d = ψ / L_m, motoring; a golden section finds the top, and none is searched where the rated flux is it: the voltage not binding and i_d at most the current amplitude over √2). The control keeps the rated flux (above the rated frequency its 1/ω cap) while the circles leave room at it for the torque asked for, and weakens it only as far as that torque needs, to the largest flux that leaves it (`control_flux`, a bracketed secant, Illinois, between the cap and the flux of the most torque: a handful of evaluations where a bisection took 30); asked for the curve's torque it sets the curve's flux, and the q current is held to what the circles leave at the flux set, so curve and delivered torque agree (by a run within 1 % at 110 to 150 rad/s on the 540 V link, `asked_for_the_curves_torque_the_field_oriented_control_delivers_it`). On a 540 V link the rated flux 0.995 Wb leaves 23.05 N m at 115 rad/s and 0.774 Wb 27.29 N m, which the control sets when asked for that torque (the curve said 25 N m before, and a flux held at 0.9 of the voltage on the d axis was no round number's worth of torque); at 140 rad/s 7.24 N m against 20.68 N m at 0.650 Wb; at 60 rad/s the current circle binds and the rated flux stays. The V/f scan is checked against the Thevenin breakdown at fixed frequency and a dense sweep (`the_v_f_scan_agrees_with_the_thevenin_breakdown_and_a_dense_sweep`). The drive's maximum current limits the curve through the currents, not through the servo torque constant. **Heat**: the copper loss of stator and rotor warms the one winding; the reference is the copper loss at the name-plate point (rated voltage, frequency and slip) from the equivalent circuit, which brings it to its rated rise (`Drive::thermal_current`).

Assumptions: no saturation of L_m, no iron loss, no skin effect in the rotor bars (R_r constant); V/f without a current limit controller; the field-oriented drive knows the motor's parameters exactly (no detuning unless the config's values differ from the motor's, which they cannot here). Tests (`induction::tests`, a 1.5 kW four-pole motor of round numbers): synchronous speed 1500 rpm and rated slip 4 % from the name plate; Kloss with R_s = 0 at three slips within 0.2 % (τ_b = 37.96 N m at s_b = 0.2846) and the breakdown with R_s, 27.10 N m, against the curve's top; the starting current 22.385 A amplitude (15.83 A RMS); a 5 N m load step under V/f costs 2.947 rad/s of slip by hand (within 6 %) and less than 0.4 rad/s compensated; the flux builds with L_r / R_r = 134 ms (63.2 % at one, 95 % at three); a torque step answers with the current loop's 0.5 ms.

**Machine files and verdicts.** [`machines/stepper-axis.toml`](../machines/stepper-axis.toml) (a NEMA 23 stepper on a small belt axis, 48 V, 16 microsteps) and [`machines/induction-lift.toml`](../machines/induction-lift.toml) (a 1.5 kW four-pole motor, gearbox 10, vertical belt lift, field-oriented), [estimate] values both. Siemens' S1 check ("motor-s1") is a servo motor's and not made for the other kinds; "stepper-synchronism" judges a stepper's highest load angle over the pass against the pull-out angle π/2 (above π it has lost steps). Tests: `crates/sim-machines/tests/motor_kinds.rs`.
### Start, disable, enable

A drive starts at rest already delivering the torque that holds the load, so a hanging axis does not sag: the velocity integrator, the torque filter and delay hold that torque, the current loop's integrator, output and delay the voltage R(ϑ) · i that holds its current, with the winding's resistance at its start temperature (before 14 November 2026 the cold one: a winding started or enabled warm began with a step of (R(ϑ) − R) i). At the start the holding torque is the machine's equilibrium at the pose the tool stands at (since 15 November 2026; before, at the pose the motor encoders read, the forward transformation of the motor angles, which the gearbox's twist or a belt's stretch puts some micrometres away: on the single axis with its backlash 8.7 µm, 5.3e-4 N m against 0.8e-4 N m of cogging and tooth meshing). An Enable has no machine state and holds at the pose the encoders read. At the start the DC link is at the bus voltage: the ½ L i² of the holding current is taken from it only at an Enable (below). Disabled drives give zero torque, current and voltage; their windings cool (with `effects.winding_heating`) and their DC links discharge (with `effects.dc_link`). The current in a winding when the pulses stop freewheels through the inverter's diodes into the DC link within microseconds, so in the first disabled step its energy ½ L i² is fed back (the drive's power for that step is −½ L i² / dt; without the DC link simulated it goes back to the supply). Enable restarts the loops holding where the motors stand; a winding keeps the temperature it has and a DC link the charge it has, less the ½ L i² of the current that holds the load (below the bus level the rectifier makes that up at the next step). A drive that tripped and is enabled again while its link is still above the trip level trips again unless the chopper brings it below within the step. Test: `disable_and_enable_conserve_the_electrical_energy` (windings plus capacitors across Disable and Enable, to 1e-9 relative).

## Stops

`Command::Halt(category)` stops the machine as a servo drive does, by the stop categories of IEC 60204-1 (`sim-cli run --stop-at SECONDS --stop-category N`, `sim-cli stop`, the FMU's `command.stop_category`, `ks.Sim.stop`, the OPC UA node `command.stop_category`). A move, path or cycle under way is dropped in each.

- **Category 0** (SINAMICS STO / OFF2): the torque off at once, as `Command::Disable`. The motors coast against what holds them (friction, gravity, the process) until the holding brakes engage.
- **Category 1** (OFF3, then off; as SS1): every drive's speed setpoint ramps down from the speed it measures on its OFF3 ramp, ω_rated / `drive.off3_ramp_s` (the same ramp as the fault reaction `"off3"`, whatever `drive.fault_reaction` is), the velocity and current loops following it with their torque and current limits, the DC link and the braking chopper as in any move (the position loop open). Once every ramp is at zero and every motor below `drive.pulse_off_speed_rad_s`, the brakes are asked to hold with the torque still on; after `brake.engage_delay_s` the torque goes off. If `drive.ss1_time_s` runs out first, the torque goes off then, the brakes asked to hold at the same moment, as SS1's timer does.
- **Category 2**: the controller brakes the plan, the drives stay on. From where the plan stands at its speed v, every axis decelerates evenly to rest in T = max_i |v_i| / a_max,i (the axis limits `axis.*.max_acceleration_*`), so the tool keeps the direction it moved in and stops |v| T / 2 further on; then it holds there with torque on. A stop that would end outside the axes' range ends in T halved until it is inside (at most 8 times). The deceleration steps in and out (no jerk limit).

```text
category 1: s = v² / (2 a) + lag      t = v / a + lag      a = ω_rated / off3_ramp_s · (load per motor angle)
category 2: s = v T / 2               T = max_i |v_i| / a_max,i
category 0: s = v t_d − F t_d² / (2 m) + m v₁² / (2 (F + F_b))      v₁ = v − F t_d / m
```

with F what holds the axis (friction), F_b the brake's torque at the carriage and t_d the brake's engagement delay. The velocity loop follows a ramp somewhat behind, so the peak deceleration over 1 ms overshoots the ramp's at its start (6.0 against 5 m/s² on the single axis below); in category 2 the position loop follows the plan's steps in deceleration the same way (13.3 against 10 m/s²).

**Brake delays** (`effects.brake_delay`, off by default): the brakes engage `brake.engage_delay_s` after they are asked to hold (the drives off, or the end of a category 1 stop) and release `brake.release_delay_s` after they are asked to let go (the drives on again), to the nearest base step; the brake model of [Holding brakes](#holding-brakes) is unchanged. Meanwhile a disabled hanging axis falls: g_eff t_d² / 2, g_eff the weight less what friction holds over the moved mass; a drive enabled again holds against the brake until it lets go. Off, or with both delays 0, the brakes follow the drives within the step as before.

The stop's state (the SS1 timer, the brakes' timer) is the controller's and goes with its snapshot. Tests on the rigid, loss-free single axis (`ideal_rigid`: 6.125 kg reflected, 4 kg carriage), each seen to fail with its part taken out:

| Test | By hand | Simulated |
| --- | --- | --- |
| `the_stops_travel_the_hand_calculated_distance_in_the_hand_calculated_time` | category 1 at 1 m/s, ramp 0.2512 s from 314 rad/s = 5 m/s²: 0.1 m in 0.2 s; category 2 at 10 m/s²: 0.05 m in 0.1 s | within 3 %; the tool's last 1 mm/s in category 2 within 20 % of the time (the position loop's lag) |
| `a_category_0_stop_coasts_the_hand_calculated_distance_with_the_brake_delay` | 10 N friction, 50 N brake, 0.03 s delay: 0.075435 m; no delay 0.051042 m | within 1 % |
| `a_hanging_axis_sags_by_hand_while_its_brake_engages` | upright, 39.227 N weight less 10 N friction on 6.125 kg: 2.1473 mm in 0.03 s (480 steps) | within 2 % |
| `a_category_1_stop_brings_the_hand_calculated_energy_to_the_dc_link` | 3.0625 J less 0.0593 J copper loss: 470 µF to 191.95 V; 4.7 µF: the resistor 2.9412 J | within 0.3 V and 2 % |

**Stopping distance** (`sim-cli stop MACHINE [--category N] [--points N]`, `stop::study`): the cycle (or the `[path]`, or `--path FILE`) runs once; the moments where each tool axis is fastest, where the machine's kinetic plus potential energy (`Machine::energy`) is largest, and N moments evenly over the pass (20 by default) are kept as snapshots of a second, identical run, and from each the stop is run until every axis stays below a hundredth of its speed limit for 0.1 s (at most 3 s; a carriage ringing on its belt and brake after the torque is off is not travelling any more). Per stop: how far the tool got from where it was at the trigger, over the axes in metres (over all axes on a machine without one), and how far each axis got, the farthest (not the path, so that ringing does not add up), the stopping time, the peak deceleration over 1 ms, whether an axis went beyond its range by more than `Machine::setpoint_slack` (a hard stop), the DC link's peak voltage and the braking resistors' energy, and whether a drive tripped (it then coasts, as a disabled drive). The worst is the stop with the longest tool distance; `stopping-distance` judges it against `ratings.max_stopping_distance_m` where the config has it (on a machine without an axis in m, where the distance is in rad, against `ratings.max_stopping_angle_rad`), and `stop-dc-link` (with `effects.dc_link`) the stops' highest DC link voltage against the trip level, failed when a stop tripped a drive.

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

Belt mass: half of the free belt is lumped on each motor pulley, J_pulley += ½ · ρ · L_total · r², which is exact for the runs that move at pulley speed. Summed run by run along the belt path (`belt_mass`), the runs at the clamp move with it, (v_x, v_z), the upper runs at the pulleys' surface speeds v_x ± v_z and the lower ones back at −(v_x ± v_z), and the runs down the Z axis, which pass the lower carriage idlers at 2v_x ± v_z, at (v_x, 2v_x + v_z) and (v_x, −(2v_x − v_z)): 2T / ρ = L_total (v_x² + v_z²) + 8 (s + s_z − z) v_x² + 4 (2x − s_x) v_x v_z (s the free span at each end, s_x, s_z the strokes). Beyond the lumps that is T = ½ m_xx v_x² + m_xz v_x v_z with m_xx = 8ρ (s + s_z − z) and m_xz = 2ρ (2x − s_x): the X coefficient ρ (L_total + 8 (s + s_z − z)) in all and the cross term 2ρ (2x − s_x) v_x v_z. Lagrange's equations give the speed terms 8ρ v_x v_z on X and −8ρ v_x² on Z. Where the frame sways at q' the whole belt sways with it, which adds ρ (4s + 2s_z) q' v_x (the vertical runs; the horizontal ones cancel) and ½ ρ L_total q'². Both models, the energy, the modes, the frame's momentum (for a [base](#compliant-base)) and the feedforward at speed (`feedforward_torque_moving`) have all of it; the machine file sums it the same way from its spans' speeds over ground ([machine files](machine-files.md)).

### States of the elastic model

21 states: positions x, z, a, b, φ_A, φ_B, q, u; their eight rates; the two LuGre states of the guides; the slack-time counter; the two brake states.

- x is the carriage relative to the frame. q is the frame deflection in X.
- u is the Z axis's centre of mass in X relative to the carriage.

### Equations of motion

Masses: m_z = Z axis + payload (if attached); m_c = carriage + idler mass on X + moving chain mass m(x).

Carriage, Z axis, frame and leaning Z axis, Lagrange's equations solved together over (x, z, q, u):

```text
T = ½ m_c w² + ½ m_z (w + u')² + ½ (m_z + m_idler,z) z'² + ½ m_f q'²                 w = x' + q'
  + ½ m_xx x'² + m_xz x' z' + ρ (4s + 2s_z) q' x' + ½ ρ L_total q'²                 the belt's runs (above)
M · (x'', z'', q'', u'') = (F_x, F_z, F_q, F_u)                                       M the mass matrix of T
F_x = T₃ − T₁ − F_f,x − F_idler,x − F_chain drag + F_stop,x + F_p,x + m′ w (½ w − x') + 8ρ x' z'
F_z = 2T₂ − T₁ − T₃ − F_f,z + f_z,flex − F_idler,z + F_p,z + F_stop,z − m_z g − 8ρ x'²
F_q = −k_f q − c_f q' + F_p,x − m′ x' w
F_u = tip · F_p,x − k(z) u − c_u u'
```

The process force acts at the tool tip, at x + q + tip · u, so on each of them; the lean's spring acts between carriage and Z axis, on u alone. Without `effects.tool_flex` m_z rides on w and u stays 0; without `effects.frame_compliance` q stays 0.

Pulleys and motor shafts, for k = A, B:

```text
J_pulley · d²θ_k/dt² = τ_gear,out + τ_belt,k + r · F_mesh(r · θ_k)
J_rotor  · d²φ_k/dt² = ripple(τ_motor,k, φ_k) − τ_gear,rotor − τ_shaft(dφ_k/dt) − τ_brake,k
```

### Frame mode

`effects.frame_compliance`: the beam and its mounting are one mass `frame.mass_kg` on a spring `frame.stiffness_n_m` with damping, in X only. Belt, guide, idlers, chain and end stop push back on the beam with the force they put on the carriage. This is the portal's own sway, coupled to the belt both ways. What the whole gantry stands on is the [compliant base](#compliant-base) (`effects.base_compliance`), in series with it.

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

`effects.z_axis_bending`: the Z axis also bends, a cantilever from the carriage of bending stiffness E I (`tool_flex.z_axis_bending_stiffness_nm2`), in series with the guide's give (alone with `tool_flex` off, whose stiffnesses are then infinite):

```text
k(z)   = 1 / ( 1/k_lat + h²/k_tilt + h³/(3 E I) )
tip    = k · ( 1/k_lat + h · h_tool / k_tilt + h² (3 h_tool − h) / (6 E I) )
dk/dz  = k² · ( 2 h / k_tilt + h² / (E I) )
```

The Z axis alone at z = 0: k = 3 · 1.4e4 / 0.3³ = 1.556e6 N/m, the tool moving (3 h_t − h)/(2 h) = 3 times as far as the centre of mass; with the guide 1.2206e5 N/m against 1.3333e5 without the bending.

Assumptions: the Z axis is a rigid body on a compliant guide (with `z_axis_bending` one bending cantilever lumped at its centre of mass, the same state u). Only its lean in X is a degree of freedom, not in Y and not in rotation as a state of its own. The payload sits at the centre of mass used for h.

### Beam sag

`effects.beam_sag` (elastic model): the X beam is a simple span `beam.span_m` between the posts, E I `beam.bending_stiffness_nm2`, the carriage `beam.carriage_at_zero_m` from the left post at x = 0. Carriage, Z axis and payload sag it under the carriage, and the tool is that much lower:

```text
a = carriage_at_zero + x,  b = L − a
sag(x) = (m_c + m_z) g · a² b² / (3 E I L)          (L³ / (48 E I) per newton in the middle)
z_tool = z − sag(x),   ż_tool = ż − sag'(x) ẋ
```

At rest the Z axis stands sag(x) higher on its belt so that the tool is at its pose. With the default values: 107.87 N, L = 1.3 m, E I = 8.4e4 N m²: 58.8 µm in the middle, 9.8 µm at the ends of the stroke. This is quasi-static.

With `effects.beam_vertical_mode` the beam's first vertical mode with the carriage on it is a coordinate w, the beam's deflection under the carriage (down), and the Z axis's z is relative to the carriage:

```text
shape φ: the span's static deflection under a point load at the carriage, φ = 1 there
k(x) = 1 / span_compliance = 3 E I L / (a² b²),   M(x) = μ ∫ φ² ds   (17 μ L / 35 in the middle)
T = ½ (m_c + m_z + M(x)) w'² − m_z z' w' + …,   V = ½ k(x) w² − (m_c + m_z) g w,   damping c = 2 ζ √(k M)
z_tool = z − w (+ the lean's rise)
```

So the Z axis's acceleration drives the beam through −m_z, and the carriage's weight riding along the beam drives it through k(x) (a force on x of −½ k' w², and ½ M' w'² from the modal mass). At rest w is the quasi-static sag. By hand in the middle (k = 1.835e6 N/m, M = 2.21 kg, m_c + m_z = 11 kg): 59.3 Hz (the former estimate of about 100 Hz left out the carriage and the Z axis). The Lagrangian drops what the shape's change with x adds to the kinetic energy (second order in w), and the belt's vertical runs (0.5 kg) are not on w. The damping is the loss row `power.beam`; the signal `beam.deflection` is w.

The belt's vertical runs hold the leaning Z axis back with their geometric stiffness: the clamp (s + z above the carriage's idlers) and the Z-bottom idler (s + sz − z below them) stand out sideways by k u g (g the guide's give, tilt and bending at that distance), a run of length l under tension T is longer by δ²/(2l) and holds the end back with T δ / l, so K_uu gains Σ T_i c_i with c = (g_clamp²/(s + z), 2 g_bottom²/(s + sz − z), g_clamp²/(s + z)) (a few per cent of the lean's spring). The process force's vertical part works on the tip's rise a(z) u² (the leaning bar is shortened by θ² d / 2): it is in the lean's and the Z axis's equations, in the modes and in the process row of the budget, and `tool_state` has the rise in the tool's height.

The modes the machine reports (`modes`; the lowest is `resonance_hz`, the input shaper's) are the generalised eigenvalues K v = ω² M v over the elastic coordinates the model has at that pose (pulleys taken as held): the carriage x on the frame, the Z axis's z on its belt, with `effects.frame_compliance` the frame's sway q, with the leaning Z axis its lean u:

```text
T as in the equations of motion, belt's runs and all; the pulleys and the belt lumped on them stand
K = Σ k_i g_i g_iᵀ + k_f e_q e_qᵀ + k_u e_u e_uᵀ,  k_i = EA / l_i (l_i the section's unstretched belt at rest)
g₁ = (1, 1), g₂ = (0, −2), g₃ = (−1, 1)          in (x, z): the sections' lengths change with x + z, −2z, −x + z
K_xx = k₁ + k₃,  K_xz = k₁ − k₃,  K_zz = k₁ + 4k₂ + k₃
ζ = vᵀ C v / (2 ω vᵀ M v)
```

Only modes that move the tool (along X by x + q + tip · u, or along Z) are reported, so that the input shaper aims at the lowest that does and the second shaper at the next one. With the default frame and the leaning Z axis that is four modes: the frame swaying with the carriage on it, the carriage swaying against the frame with the Z axis leaning, the Z axis on its belt, the Z axis's lean against the carriage. By hand without the lean at x = 0.5 m, z = 0.25 m, in (x, q): M = [[11.32, 11.16], [11.16, 41.42]] kg (11 kg, the belt's runs 0.32 kg on x, the whole belt's 0.42 kg swaying, its 1.6 m of vertical runs coupling), K = diag(k_b, 7.4·10⁵) with k_b = 2 · 5·10⁵ / l, l = 1.05 / (1 + 282.84 / 5·10⁵) m: 20.60 Hz and 55.64 Hz (the Z axis's mode on the belt above them). Against the model's own linearised equations at rest (`the_modes_are_those_of_the_linearised_derivatives_with_the_pulleys_held`), loss-free with the backlash off: 18.74 / 30.67 / 76.04 / 116.30 Hz at x = 0.9 m, z = 0.05 m and 20.53 / 53.22 / 83.05 / 200.98 Hz at (0.5, 0.25); without the belt's mass 18.81 / 30.77 / 78.51 / 119.46 and 20.64 / 53.91 / 83.05 / 206.52 Hz. C holds the two sections' damping on the same g_i (`belt.damping_n_s_m` each, or with material damping `belt.damping_time_s` · k_i more), `frame.damping_n_s_m` on q and `tool_flex.damping_n_s_m` on u; the cross terms of a damping that is not proportional are left out. The stiffness of the control loops is not in it: under the servo the swing is a little faster (about 36 Hz where 34.58 Hz is reported, frame off, x = 0.9 m, z = 0.15 m).

The single axis reports one mode, its carriage on the belt with the pulley held: f = sqrt(k / m) / (2π), k = EA/l₁ + EA/l₂ (dT/dL of the belt section, l its unstretched belt at rest there: 1 + T/EA shorter than the geometric length, 0.06 % at 300 N on 5·10⁵ N), m the carriage with payload and cable chain, ζ = c / (2 m ω) (65.82 Hz at x = 0.5 m). The machine file takes the same l (`rest_tensions` in sim-netlist), so file and crate agree to rounding. None in the rigid model.

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

Held, the shaft sits on a spring that reaches the brake torque at 1 mrad, with the damping c critical for the rotor on that stiffness. The torque never exceeds the brake torque: beyond it the brake slips, and what s is deflected beyond 1 mrad is forgotten within 50 µs. A brake that engages at speed therefore decelerates the rotor with brake torque over inertia until it stands, at any speed. (The brake was a LuGre contact before; its bristle damping gave many times the brake torque when it engaged at speed, and above 178 rad/s its state was too stiff for the integration step.) A released brake forgets its deflection within 1 ms. With `effects.brake_delay` a brake engages and releases its delay after it is asked to (see [Stops](#stops)). Brakes exist in the elastic and in the rigid model, and on both motors (there is no separate "brake on Z"; holding Z takes both).

### Rigid reference model

`effects.belt_stretch = false`. Eight states: x, z, their rates, the two LuGre states, the two brake deflections. With belt force F_k = i · (ripple(τ_k) − τ_shaft − τ_brake,k) / r per motor (τ_brake,k the [holding brake](#holding-brakes) at the rotor speed i (v_x ± v_z) / r), reflected rotor mass m_r = i² J_rotor / r² and the gearbox loss factors λ_A, λ_B:

```text
( m_X + (λ_A + λ_B) m_r ) · a_x + (λ_A − λ_B) m_r · a_z = λ_A F_A + λ_B F_B + mesh − F_f,x + extras_x
(λ_A − λ_B) m_r · a_x + ( m_Z + (λ_A + λ_B) m_r ) · a_z = λ_A F_A − λ_B F_B + mesh − F_f,z − m_z g + extras_z
```

m_X and m_Z include both pulleys (2 J_pulley / r²), the idlers and the chain. The gearbox loses torque only on what it transmits, which is the motor torque minus what accelerates the rotor itself; that depends on the acceleration being solved for, and the loss factors are iterated to their fixed point (`mech::settled_loss`; three passes left up to 1.5e-4 of the acceleration). With a loss-free gearbox this collapses to two independent axes:

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

Source: [`sim-netlist/src/spatial.rs`](../crates/sim-netlist/src/spatial.rs) (the crate reads its tables into it; a machine file's linkage node of three arms builds the same machine), with the joint of [`joint.rs`](../crates/sim-components/src/joint.rs). Equations after the research note of 13 November 2026: Williams, "The Delta Parallel Robot: Kinematics Solutions" (Ohio University, 2016), for the kinematics; Codourey (IJRR 17(12), 1998) as used by Olsson ("Modeling and control of a Delta-3 robot", Lund 2009) for the dynamics.

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

Lumped (`effects.forearm_inertia` off), each forearm pair's mass m_fb is split: a share s (`mass.forearm_elbow_share`, default 2/3, which matches the rod's inertia about its end, m L²/3) turns with the arm at the elbow, the rest moves with the platform. The forearms' own rotational inertia is left out. The share is the inertia's; a rod's weight acts at its middle, half at each end (exact with the rods, `effects.forearm_inertia`). Then the machine is three arms about their joints and a point mass at the platform:

```text
m_nt = m_platform + m_payload + 3 (1 − s) m_fb
I_b  = I_arm + L_a² (m_elbow + s m_fb)             (I_arm about the joint; rotor and gearbox not in it)
m_LG = m_arm l_cg + L_a (m_elbow + s m_fb)
M(θ) θ̈ = τ_out + Q_g − m_nt Jᵀ J_x⁻¹ w + τ_joint
M = I_b · 1 + m_nt JᵀJ,   Q_g,i = g m_LG cos θ_i − m_nt g J_zi
```

This is exact for the lumped model's kinetic energy (its weight keeps the share too, as an approximation; the D2 weights with 1/2, see there): the platform's kinetic energy ½ m_nt |J θ̇|² is what M holds, the velocity term is the platform's m_nt p̈ through Jᵀ with p̈ = J θ̈ + J_x⁻¹ w, and Q_g = −∂V/∂θ with V = m_nt g z − g m_LG Σ sin θ_i. τ_joint is the joint's friction (Coulomb, smoothed with tanh over `joint.friction_smoothing_rad_s`, and viscous; `effects.joint_friction`) and its stops (springs and dampers beyond the stop angles, `effects.hard_stops`).

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

**The tube's own share and the elbow's twist** (`effects.arm_tube_inertia`, with `link_elasticity`; off as shipped). The tube bends in its first cantilever mode, φ(L) = 1 at the elbow: a quarter of its mass moves with the elbow's bend, so m L_a²/4 of its inertia about the shoulder and 0.3915 m L_a of its first moment (what its weight drives the mode with, ∫φ) go from the root to the elbow; the arm turned whole keeps both sums. Out of the plane a quarter of its mass moves with the elbow's move y, and the elbow block turns with the tube's tip slope, 1.3765 y / L_a, about the arm's normal: its twist inertia (`mass.elbow_twist_inertia_kgm2`, plus the forearms' share at the rods' ends, s m_fb (spacing/2)²) counts J (1.3765 / L_a)² on y. Default arm: 0.036 kg m² and 0.1409 kg m to the elbow (the root keeps 0.012 and 0.0391), the lateral mass 0.383 + 0.225 + 0.0054 = 0.614 kg, which lowers the elbows' lateral mode from sqrt(4e5 / 0.383) / 2π = 162.6 Hz to 128.5 Hz. The twist turns the rods' ends at the elbow in opposite senses along the arm; their mean, which the platform follows, stays (to first order the platform does not move).

**Resonance** (`Machine::modes`, `resonance_hz`, `mode_shape`; what `effects.shaper_follows_pose` sets the input shaper to). The modes with the motors held by their position loops: per arm the gearbox's spring k_g (with `joint_elasticity`) from the held rotor to the arm's root, the bending spring k_b (with `link_elasticity`) from the root to the elbow, the lateral spring k_l on the elbow's move, the root I_arm and everything beyond the mass matrix M_d at the elbow angles (and moves). The coordinates (roots, elbow angles, moves) with the flexibility per arm K⁻¹ = [[1/k_g, 1/k_g], [1/k_g, 1/k_g + 1/k_b]] and 1/k_l (a rigid spring 0): with M = L Lᵀ (Cholesky) the eigenvalues μ = 1/ω² of the symmetric Lᵀ K⁻¹ L by Jacobi rotations, a rigid connection's μ = 0 left out (`arm_modes`). Each mode's damping ratio is vᵀCv / (2 ω vᵀMv) with C the gearboxes', the bending's and the lateral dampers (the cross terms of a damping that is not proportional left out). 3 modes with the gearboxes alone, 6 with the bending alone, 9 with both; `resonance_hz` is the lowest. Gravity's stiffness (a few N m/rad) and backlash are left out. With rigid gearboxes and arms there is none. The gearbox's k_g is K1, its stiffness for the moves that excite the mode (twists above about 1e-3 rad); with `gearbox_lost_motion` and a non-zero `hysteresis_rad` its small-amplitude stiffness is K1 + σ = 2 K1 ([gearbox](#gearbox)), so a small vibration rings about 41 % higher: on the D3 at its start pose (0.1525, 0, -0.75) m 49.07 / 51.48 / 84.02 Hz become 69.39 / 72.80 / 118.82 Hz. `Machine::modes` reports the large-amplitude K1 (what a shaper for the move's own swing takes), `Machine::modes_small_amplitude` the same modes with K1 + σ (what a shaper for the residual ringing below about 1e-4 rad at the joint takes; the four-arm delta H4 does the same for its arms' four modes, its amplifier's mode unchanged, though it ships with no hysteresis, `the_small_amplitude_modes_stand_sqrt_two_above_the_large_ones_with_hysteresis` in its crate; the other machines return `modes`). At the centre with the arms level the lowest is the platform's sideways mode (twice): the forearms lumped, 28.14 Hz on the gearboxes alone, 38.76 Hz on the bending alone, 22.88 Hz with both (vertical: 68.39 Hz on the bending alone). There every forearm is square to its axis, so the elbows' moves ring on their own: lumped sqrt(k_l / (m_elbow + s m_fb)) / 2π = 126.48 Hz three times; with the rods, whose middles also move with the platform, 147.35 Hz for all three elbows moving the same way round and up to 0.3 % more for the two that couple to the sideways modes.

### Signals

`arm.<k>.angle` (rad), `gearbox.<k>.output_torque` (N m; elastic: the spring's torque on the arm, rigid: λ (d − i² J_rotor θ̈)), `kinematics.condition`. Effect views: the [torque breakdown](#torque-breakdown) per motor with inertia = ((M θ̈ + velocity term)_k + i² J_rotor θ̈_k) / i, gravity = −Q_g,k / i, friction = shaft + brake + gearbox loss at the rotor + joint friction / i; the loss powers `power.joint_<k>`, `power.shaft_<k>`, `power.gearbox_<k>`, `power.hard_stops`, `power.other` (ripple), with bending arms `power.arm_bending`; the energies. With bending arms the joint carries the tube and the spring: gravity = −(g m_arm l_cg cos θ + Q_d) / i, inertia the rest of the spring's torque. The joints are offered as rotary friction contacts `joint.<k>` (signals `joint.<k>.friction_speed` in rad/s at the arm, `_coulomb`, `_viscous`, `_stiction` in N m, `_load_scale` 1): the friction view draws their torque over the angular speed up to the motor top speed over the gearbox ratio. Each gearbox warms with `effects.part_heating` (`temperature.gearbox_<k>`) and follows its temperature with `effects.thermal_feedback`.

`Machine::scene_point` is the platform's centre: the machine's frame is the scene's. The scene: base plate, per arm a motor and a gearbox box with a bar turned by the rotor angle, the arm (to its elbow, bent or not) and the two rods of its forearm pair as segments, the platform as a box, the payload under it. The collision bodies are the platform and the payload as boxes, and each forearm rod and each upper arm as a capsule along its drawn segment, of half its drawn width (6 mm for a rod, 20 mm for an arm). An obstacle an upper arm runs into is a crash as any other.

Not modelled: the tube's own share of the bending modes (it stays with the root in the plane and does not move out of it; 33/140 of a beam's mass would go to its tip), the elbow block's twist as the arm bends out of its plane (the elbow's axis stays as it is), elasticity of rods and ball joints (no figures published for any product), friction in the ball joints (lumped into the joint friction), the ball joints' cone angles (the stops and the working range stand in), collisions between the links, a fourth axis (see the [D4](#delta-picker-d4)).

## Delta picker D2

Source: [`sim-netlist/src/linkage.rs`](../crates/sim-netlist/src/linkage.rs) (the crate reads its tables into it; a machine file's linkage node of two arms builds the same machine). The planar delta: two arms in the vertical x-z plane, their shoulder axes along y at x = ±R, each with a pair of forearms down to a plate that keeps its orientation (a five-bar whose lower links are parallelograms, as in Codian's D2 or ABB's two-axis FlexPicker). The tool is the plate's centre p = (x, z), z up from the shoulder axes. The equations are the [D3's](#delta-picker-d3) with two legs at φ = 0° and 180° and y = 0; leg i points along s_i x (s_1 = +1, s_2 = −1):

```text
elbow       c_i(θ) = (s_i (e + L_a cos θ), −L_a sin θ)
inverse     x′ = s_i x,  A = −2 L_a (x′ − e),  B = 2 L_a z,  C = (x′ − e)² + z² + L_a² − L_b²,  θ = 2 atan((−B − sqrt(A² + B² − C²)) / (C − A))
forward     the lower of the two points at L_b from both elbows (two circles)
Jacobian    2 × 2: J_x rows u_i = p − c_i, J_q = diag(u_i · ∂c_i/∂θ_i), J = J_x⁻¹ J_q; condition number from the eigenvalues of JᵀJ in closed form
dynamics    m_nt = m_plate + m_payload + 2 (1 − s) m_fb,  M = I_b · 1 + m_nt JᵀJ   (I_b as on the D3)
            weight: Q_g,i = g m_LG,w cos θ_i − m_w g J_zi,  m_LG,w = m_arm l_cg + L_a (m_elbow + m_fb / 2),  m_w = m_plate + m_payload + m_fb
```

The joints, the elastic and rigid models (10 and 6 states), the feedforward, the signals (`arm.1/2.angle`, `gearbox.1/2.output_torque`, `kinematics.condition`), the effect views, part heating and the refusals (`error-delta-*`) are the D3's. **Working range**: the band |x| ≤ `workspace.half_width_m` below `top_z_m`, its bottom an arc through both edges at `height_edge_m` and the centre at `height_centre_m` below the top. **Static hold, and stability**: held by constant motor torques the plate's vertical mode is unstable in the upper part of the range, −3.778 N m/rad at the middle of the default range (638.5 mm below the shoulders), +7.428 N m/rad at 788.5 mm; the sideways mode is stable (13.714 and 19.125 N m/rad; the forearms' weight a half at each end). The standard suite's loss-free configs put range, start and cycle 150 mm lower. `Machine::setpoint_slack` is `rest_sag` over 41 columns of the range: 0.42 mm with 1 arcmin backlash (at most 0.37 mm found at rest in the simulation).

The model takes the plate as kept level. The 3D view draws each forearm pair as two rods apart along y, as on the D3; in the plane, a plate on two legs needs a parallelogram in the x-z plane (or a third link) to keep it from turning about y, which real D2s have. The scene: a base beam, per arm a motor and gearbox box with the rotor bar, the arm, the two rods, the plate and the payload; the collision bodies are the plate and the payload. Not modelled: the forearms' rotational inertia (the D3's rods carry it, `effects.forearm_inertia`), elastic links, friction in the ball joints, collisions between links.

## Linear delta

Source: [`machine-delta-linear/src/lib.rs`](../crates/machine-delta-linear/src/lib.rs), with the belt, guide, coupling and motor components of the single axis. The parallel kinematics are the D3's with the arms replaced by carriages on vertical columns (as the Festo EXPT tripod and the Kossel printers); the dynamics are at the D3's level (Codourey 1998, Olsson 2009).

Three columns stand at the azimuths φ_i = 0°, 120°, 240° at the column radius R. Each carries a carriage on a toothed-belt axis; a parallelogram of two rods of length L joins it to a plate of radius r that stays level. The columns stand upright or lean by α = `geometry.column_incline_deg`, positive with their tops towards the machine's axis (a tripod whose columns meet above it), negative away from it. The tool is the plate's centre p = (x, y, z), in a frame with its origin on the column axis at z = 0 and z up (the plate works below 0). h_i is carriage i's travel along its column from z = 0 (upright: its joint's height), x_i = h_i − h_min its place in its stroke. Because the plate never turns, e = R − r, both measured at z = 0.

### Kinematics

```text
column      d_i = (−sin α cos φ_i, −sin α sin φ_i, cos α)
joint       c_i = (e cos φ_i, e sin φ_i, 0) + h_i d_i
constraint  |p − c_i|² = L²
inverse     h_i = q_i · d_i + sqrt(L² − ρ_i²),   q_i = p − c_i(0),   ρ_i = |q_i − (q_i · d_i) d_i|   (carriage above the plate)
forward     the point at L from all three joints (trilateration), the lower of the two
Jacobian    J_x ṗ = J_q ḣ,   J_x rows u_i = p − c_i,   J_q = diag(u_i · d_i),   ṗ = J ḣ,   J = J_x⁻¹ J_q
            J_x p̈ = J_q ḧ + w,   w_i = −|ṗ − d_i ḣ_i|²
```

ρ_i is the plate's distance from column i's joint line; upright (α = 0, d_i = e_z) it is the horizontal distance sqrt((x − e cos φ_i)² + (y − e sin φ_i)²) and h_i = z + sqrt(L² − ρ_i²), the formulas of the vertical machine to the bit (a test compares them).

Upright, raising all three carriages alike lifts the plate alike: the z row of J sums to 1, the x and y rows to 0 (inclined, not: the joints then also move towards or away from the axis). The condition number of J is the signal `kinematics.condition`, judged by `singularity` as on the D3; the default range has up to 3.98 (at its bottom rim, opposite a column), the centre 3.15: a metre of carriage moves the plate further sideways than up.

**Working range** (`Machine::unreachable`): a cylinder of `workspace.radius_m` from `workspace.bottom_z_m` to `top_z_m`; the axis ranges are the box around it. A target outside it (`error-delta-outside-envelope`), out of the rods' reach (`-out-of-reach`), where a rod would stand at a larger angle to its column than `workspace.max_rod_angle_deg` (`error-delta-linear-rod-angle`: ρ_i > L sin of it; upright, the angle from the vertical), where a carriage would leave its stroke from `geometry.carriage_top_m` − `stroke_m` to `carriage_top_m` (`error-delta-linear-carriage-stop`), or with a condition number above `workspace.max_condition_number` (`-singularity`) is refused with its reason.

### Dynamics

Each rod pair's mass m_rod is split: a share s (`mass.rod_carriage_share`, default 1/2) moves with its carriage, the rest with the plate. The rods' rotational inertia is left out. Then

```text
m_c  = m_carriage + s m_rod,   m_nt = m_plate + m_payload + 3 (1 − s) m_rod
M(h) ḧ = F_belt + Q_g − m_nt Jᵀ J_x⁻¹ w − F_guide + F_stop
M = m_c · 1 + m_nt JᵀJ,   Q_g,i = −g m_c cos α − m_nt g J_zi
```

exact for the lumped model as on the D3 (V = g (m_nt z + m_c cos α Σ h_i)): the carriage's weight acts on its belt with m_c g cos α, the rest, m_c g sin α, on its guide.Each column is a belt axis as on the [single axis](#single-axis), upright with its motor at the top: section 1 runs from the motor pulley straight down to the carriage (length span + stroke − x), section 2 from the motor pulley down round an idler and up to the carriage (3 span + stroke + x); the belt pulls the carriage up with T₁ − T₂ and turns the pulley with r (m₂ − m₁), the material forces of the [belt sections](#belt-sections). The guide of each carriage is a [LuGre contact](#guide-friction-lugre). Its load across the rail is what the rods' pull and the carriage's weight have at right angles to the column:

```text
plate        m_nt p̈ = −m_nt g e_z − J_xᵀ λ        (force of rod pair i on its carriage: λ_i u_i)
guide load   G_i = |f_i − (f_i · d_i) d_i|,   f_i = λ_i u_i − m_c g e_z
```

with p̈ = J ḧ + J_x⁻¹ w from the carriages' acceleration in the state (`carriage.<i>.guide_load`, so that the verdicts judge the cycle's highest load). The carriage moves along its column only: its own acceleration loads the belt, not the guide. With `effects.load_dependent_friction` the friction levels scale with the load that holds the plate at rest (p̈ = 0; the moving load would make the friction depend on the acceleration it changes), the levels holding for the guide under its preload alone. Upright at rest G_i = |λ_i| ρ_i, as before. The load is a force through the guide's centre: the moments of the rods' joints standing off the slide (Festo's Mx, My, Mz) are not modelled. The stops are springs and dampers beyond the stroke.

**Elastic model** (`effects.belt_stretch`), 25 states: h, pulley angles, rotor angles, their speeds, the guides' friction states, the brakes and the time a belt section has spent without tension (`belt.slack_time`, as on the single axis). The pulley: J_pulley ä = τ_gear,out + r (m₂ − m₁); the rotor as on the single axis, behind the coupling (or gearbox). **Rigid model**, 12 states (h, ḣ, friction, brakes): x = r a, rotor = i a, and

```text
(M + diag(J_pulley / r² + λ_i i² J_rotor / r²)) ḧ = λ ⊙ d + Q_g − m_nt Jᵀ J_x⁻¹ w − F_guide + F_stop,   d_i = i (ripple(τ_i) − τ_shaft − τ_brake) / r
```

λ_i the gearbox loss factor, in two passes as on the other machines. Feedforward: the rigid, loss-free model, τ_k = ((m_c + J_pulley / r²) ḧ_k + m_nt (Jᵀ p̈)_k − Q_g,k) r / i + J_rotor i ḧ_k / r with ḧ = J⁻¹ (p̈ − J_x⁻¹ w) (`feedforward_torque_moving`). `rigid_tensions` shares each carriage's force F = m_c ḧ_k + m_nt (Jᵀ p̈)_k − Q_g,k between its two sections so that the belt keeps its length: T₁ = P + F L₂ / (L₁ + L₂), T₂ = P − F L₁ / (L₁ + L₂).

**Static hold.** At rest each belt holds −Q_g,i: at the centre the carriage's weight and a third of the plate's, 22.07 N with the default masses (`the_belts_hold_the_carriages_and_the_plates_weight_through_the_jacobian`). Held by constant motor torques the hanging plate is stable sideways (it rises to second order when two carriages move apart) and neutral in the vertical mode, so the standard suite's loss-free configs keep the default range. The belts' stretch and the couplings' twist put the motors' pose 0.01 to 0.02 mm off the tool at rest; `setpoint_slack` is worked out over a grid of the range as on the D3 (`rest_sag`) and stays at the default 0.1 mm.

**Guide ratings** (`Machine::rating_checks`): `guide-load` per carriage, the highest `carriage.<i>.guide_load` against `ratings.guide_max_load_n`, and `guide-life`, from the damage rate `carriage.<i>.guide_damage_rate` = 3600 |ḣ_i| (G_i / C)³ / 50 km per hour (the catalogue life L10 = (C / G)³ 50 km used up at the carriage's speed, C `ratings.guide_dynamic_load_rating_n`), the life 1 / its mean over the pass, against `ratings.required_life_h`. The generic guide checks follow a tool axis (`axis.<id>.guide_load`), which a carriage is not. The pulley shaft's bearing (`pulley-bearing-life`) and the belt's teeth (`belt-tooth-load`) are judged by the generic checks from `pulley.<i>.radial_load` and the coupling's output torque.

**Part heating** (effects.part_heating): each guide warms with its friction loss `power.guide_<i>`, and each gearbox, where a config has one in place of the couplings, with `power.gearbox_<i>` (`thermal_parts`); the 3D view colours the carriage (and the gearbox) by its rise. With effects.thermal_feedback each guide's viscous friction follows its own temperature, each gearbox's efficiency its own (one `Lugre` and one `Gearbox` per column). Belts and pulleys do not warm.

### Signals

Per column k: `carriage.<k>.height` (m), `belt.<2k−1>.tension` and `belt.<2k>.tension` (N; rigid: the pretension), `pulley.<k>.radial_load` (T₁ + T₂, judged by `gearbox-radial-load`), `carriage.<k>.guide_load` (N), `gearbox.<k>.output_torque` (N m, judged by `pulley-torque`); `kinematics.condition`; `belt.slack_time` (s, elastic model). Effect views: the [torque breakdown](#torque-breakdown) per motor with inertia = r / i ((M ḧ + velocity term)_k + (J_pulley + i² J_rotor) / r² ḧ_k), gravity = −r / i Q_g,k, friction = shaft + brake + gearbox loss at the rotor + r / i guide friction; the loss powers `power.guide_<k>`, `power.shaft_<k>`, `power.gearbox_<k>`, `power.belt_<k>`, `power.hard_stops`, `power.other`; the energies; the friction contacts `carriage.<k>` with their `friction_*` signals; `belt.<i>.stretch`. The six belt sections are offered to the belt view.

`Machine::scene_point` is the plate's centre. The scene: a frame plate, per column a column, its belt loop with motor, pulley and idler (drawn in the plane parallel to x–z through the column), the carriage and the two rods of its pair, the plate and the payload under it. An inclined column's belt loop is drawn upright in its x-z plane with z the travel along the column, then sheared onto the column, (x, y, z) → (x, y, 0) + z d_i: pulleys, belt, carriage and label follow the column, the pulleys' axes stay along y; the column is a strip from its foot to its head. The collision bodies (plate, payload) follow the plate.

Not modelled: the rods' rotational inertia; elasticity of frame, columns, rods and ball joints; friction in the ball joints; collisions between the links; a fourth axis; the guides' moments (Mx, My, Mz); the warming of belts and pulleys; tooth meshing, tooth jump and belt-span vibration.

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

**The tool's joint** is a `RotaryJoint` ([`joint.rs`](../crates/sim-components/src/joint.rs)): motor 4's own `[motor.4]` and `[gearbox.4]` over the shared tables (`catalog::motor_config`, as its drive and verdicts read them: a 1FK2104-4AF on an NRB060-04-8), brake and ripple as on the arms, the tool bearing's friction (`[tool]`, effects.joint_friction), no stops (the turn is endless; `axis.c.min_angle_deg`/`max_angle_deg` are the controller's range). Rigid (effects.joint_elasticity off) the rotor turns with the tool times the ratio, with the gearbox's efficiency as on the arms; elastic, rotor and tool are two bodies behind the gearbox's spring, and with effects.shaft_torsion the shaft's torsional stiffness is in series with it, 1 / (1/k_gear + 1/k_shaft), damped by the shaft's `torsional_damping_nm_s_rad` (the shaft is the softer by far: 500 against 8251 N m/rad). The tool's torsional mode with the motor held, √(k/J_c), is among `Machine::modes`: 41.1 Hz with the payload, 75.8 Hz without, 2 % damping. `resonance_hz` and `mode_shape` stay the arms' (what an input shaper on the path is for). States: the D3's, then c, ċ, (rotor angle and speed, elastic), brake: 3 or 5 more.

**Coupling to the arms.** Nothing turns the parallelograms, so the shaft acts on the arms only through its weight (effects.shaft_mass: `shaft.platform_share` of `shaft.mass_kg` at the platform, the inner shaft and the lower joint; the outer tube hangs from the upper joint) and through the telescope's sliding friction (effects.telescope_friction, `shaft.slide_friction_n` Coulomb, smoothed over `slide_friction_smoothing_m_s`, and a viscous part), a force along the shaft against its lengthening which the arms take through Jᵀ. The torque the shaft passes and the joints' bending moments on the platform do no work on the arms: the platform cannot tilt or turn. The shaft's own swing about the upper joint is not counted (as the D3 leaves out the forearms' spin).

**Signals** beyond the D3's: `tool.angle`, `gearbox.4.output_torque` (judged by the common `gearbox-*` checks of motor 4 against `[ratings.4]`), `shaft.length`, `shaft.cardan_angle`, `shaft.speed`, and in the effect views `motor.4.torque_*`, `power.joint_4` (tool bearing), `power.shaft_4`, `power.gearbox_4`, `power.telescope`, `power.other_4`; the energies include the tool's. The tool's gearbox warms as a fourth `gearbox_4`. **Refusals** beyond the D3's: `error-delta-shaft-stroke` (the telescope shorter than `min_length_m` or longer than `max_length_m`), `error-delta-cardan-angle` (a joint bent beyond `max_cardan_angle_deg`); with the defaults every pose the arms reach the shaft reaches too (0.625 m at the top of the centre, 1.020 m at the edge's bottom, at most 43.8° at the top of the edge). **Scene**: the D3's, then the fourth motor, gearbox and rotor bar in the base's centre, the telescope as an outer tube from the upper joint and an inner shaft from the lower one, the two joints, and a bar under the platform turned with the tool. **Collision bodies**: the D3's and the tube and the inner shaft as capsules along them, of half their drawn widths (15 and 8 mm).

**Paths with a turn**: `line,x,y,z,c` (lines work on any number of axes); arcs and splines are on two and three axes only, so they are refused on the D4. Not modelled: yokes out of phase, the cardan joints' friction and play, the shaft's bending and swing; arms with motors or gearboxes of their own (the three share `[motor]` and `[gearbox]`; `[motor.1]` to `[motor.3]` are refused).

## Arms in tool space (SCARA, six-axis arm)

Source: [`machine-joint-kit/src/tool.rs`](../crates/machine-joint-kit/src/tool.rs), wired in `arm.rs`; the joint machine's side in [`chain.rs`](../crates/sim-components/src/chain.rs). With `kinematics.tool_space` the axes of `scara` and `urdf-arm` are the flange's pose; the joints' dynamics (Newton-Euler on the URDF chain), drives, losses and signals are unchanged, in joint coordinates. Details, refusals and the branch kept by the six-axis inverse: [mechanisms](mechanisms.md#tool-space).

```text
SCARA       x = x1 + L1 cos(q1 + φ1) + L2 cos(q1 + q2 + φ2),  y likewise with sin,  z = z0 + s q3,  c = c0 + q1 + q2 + q4
            cos β = (r² − L1² − L2²) / (2 L1 L2),  β = ±acos (elbow),  q1 = atan2(y, x) − atan2(L2 sin β, L1 + L2 cos β) − φ1,  q2 = β − φ2 + φ1
six joints  (x, y, z) = p_tip(q),  r = (rx, ry, rz) = log(R_tip(q) R_0ᵀ);  Δq = Jᵀ (J Jᵀ + λ² I)⁻¹ e,  e = (p* − p, ℓ (r* − r)), J = ∂(p, ℓ r)/∂q,  λ = 1 mm,  ℓ = |p_tip − p_joint2| at the home
speeds      q̇ = J⁻¹ v,  q̈ = J⁻¹ (a − J̇ q̇),  J̇ q̇ = [f(q + h q̇) − 2 f(q) + f(q − h q̇)] / h²,  h |q̇| = 1e-4,  J = ∂f/∂q by central differences
feedforward τ = (M q̈ + h(q, q̇)) / i + J_rotor i q̈   per motor (in joint space the speed terms are left out, as before)
condition   κ = √(λ_max / λ_min) of Jᵀ J (Jacobi rotations): SCARA of ∂(x, y)/∂(q1, q2), six joints of the geometric 6 × 6 (turn rows the angular velocity, × ℓ, not the rotation vector's rate)
```

A redundant arm (fewer axes than joints) takes q̇ and q̈ from central differences of its inverse over 1e-3 of the tool's units, the null space settled by Newton steps to 1e-13 rad (to the rounding of J where that is not reached), so that the feed-forward does not depend on the call before.

The limits of `rx`, `ry`, `rz` (`max_velocity_rad_s` and the rest) act on the rates of the rotation vector, not on the angular velocity: they differ by (θ/2) / sin(θ/2) about the vector's own axis and more across it, so at the home (θ = π/2) a limit of 1.5 rad/s on `rz` allows a spin of 1.91 rad/s. The machine's own checks `tool-angular-speed` and `tool-angular-acceleration` therefore hold the flange's angular speed |ω| and acceleration |dω/dt| (from the geometric Jacobian, `Pose::angular_motion`; signals `tool.angular_speed`, `tool.angular_acceleration`) to the smallest of the picked `rx`, `ry`, `rz` velocity and acceleration limits in the time-optimal plan, which fits them as c·a + b·v + d·v² of the rigid model (exact on a line, whose slerp turns about one axis: |ω| = |J_ω p′| v, |dω/dt| = |J_ω p′| |a|); the checks name the signals `plan.tool.angular_speed` and `plan.tool.angular_acceleration`, and the verdicts judge them on the setpoint's rigid motion (the plan's pose, speed and acceleration through `rigid_checks` once per millisecond), not on the simulated pass (there the gears' ringing alone gave 22 rad/s² on the arm's tool cycle, whose plan holds the orientation; 11.74 against 10 rad/s² on the palletiser): a cycle planned within them passes, one planned faster fails (`the_flanges_angular_limits_are_judged_on_the_planned_motion`). The signals show the simulated values. A cycle timed by `axis_limits` keeps them too (`MotionLaw::spin`, `profile::spin_rates`): a move straight in the rotation vector bounds its speed by |ω| ≤ g₁ ṡ and its acceleration by |dω/dt| ≤ g₁ |s̈| + g₂ ṡ² (g₁, g₂ the most |J ṙ| and |d(J ṙ)/ds| along it, sampled at 65 places; half the acceleration limit at most for g₂ ṡ²), then the acceleration raised while the move's own |dω/dt| = |J ṙ s̈ + d(J ṙ)/ds ṡ²| (the two as vectors, at 1024 times of the profile) stays within the limit, a slerp of `path::plan_within` (also on the arm on a track, whose rotation vector follows its track axis) by the angle over its length. At the home a pure 0.5 rad turn on rz spins at 1.351 rad/s: the rz limit binds there, as the flange turns at |J e_z| = 2√2/π = 0.900 of the rate across its rotation vector; 0.5 rad on rx and rz at once spun at 1.91 rad/s without the flange's limits and keeps 1.499 rad/s and 7.99 rad/s² with them (`the_flanges_spin_on_a_move_uses_its_limits`); moves of the place alone are timed as before (`a_reorientation_timed_by_the_axis_limits_keeps_the_flanges_angular_limits`).

**Link bending** (`effects.link_bending`, the arms only; off as shipped). Each link with a section in `[link_bending]` bends as a cantilever from its joint to the next (the flange after the last) in its first Euler-Bernoulli mode, in the plane its joint turns it in (across = axis × along), one state d (its tip's deflection) and its rate per link, after the joint machine's states (`chain::Mechanism::flex_modes`, `beam::cantilever_mode`):

```text
E I = E π (D⁴ − d⁴) / 64,   L the link's length,   m its URDF mass (spread evenly),   M everything beyond it (payload too)
m_mod = m / 4 + M,   k_mod = 1.875104⁴ / 4 · E I / L³ = 3.0906 E I / L³,   c = 2 ζ sqrt(k m)
m_mod d̈ + c ḋ + k_mod d = −across · F_beyond − 0.3915 across · m (a_c − g) − 1.3765 / L · turn · M_beyond
F_beyond = Σ_beyond m_k (a_k − g),   M_beyond = Σ_beyond (c_k − tip) × m_k (a_k − g),   turn = along × across
```

so a link alone rings at the Euler-Bernoulli frequency 1.875104² / 2π · sqrt(E I / (m L³)) (a 60 × 4 mm steel tube of 0.8 m and 4.5 kg: 88.96 Hz), and a tip load F bends it F L³ / (3.09 E I), the static cantilever's within 3 %. The bodies' accelerations come from the joints' (central differences over ±1 µs along q̇ and q̈ of `Chain::body_states`). One-way, as the [compliant base](#compliant-base): the joints do not feel the links' bending (two-way with `effects.link_bending_reaction`, below). In tool space the flange moves with each link's tip and turns with its slope, d · across + 1.3765 d / L · turn × (flange − tip), its turn added to the rotation vector through exp and log; in joint space the tool pose is the joints, which the bending does not move (its signals `link.<n>.bend` show it). Default six-axis arm: the upper arm, 0.6 m, 12 kg with 20.5 kg beyond it, as a 140 × 8 mm aluminium tube rings at 88.5 Hz. A link along its joint's axis (the base column) has no bend (shortcut in `arm.rs`).

**Directions** (`link_bending.directions`, shipped `["in_plane"]`, the model above). `out_of_plane` adds the bend across that plane (across' = along × across, turning about along × across' = −across), `torsion` the twist about the link's length: one more first mode each per link, states after the others in the order of the list. The twist is a shaft from the joint, its tip turning by d (rad) about the link's line, stiffness k = G I_p / L, I_p = 2 I = π (D⁴ − d⁴) / 32 (`link_bending.shear_modulus_pa`), its shape growing evenly along the link. Signals `link.<n>.bend_across` (m) and `link.<n>.twist` (rad).

With any direction beyond `in_plane`, or with the reaction (below), every mode takes the bodies beyond its link whole: each rides on the tip, its centre moving by t_k = u + r × (c_k − tip) and turning by r per unit of d (a bend: u = across, r = 1.3765 / L · turn; a twist: u = 0, r = along), its turning inertia counted. The link's own body takes its share: a bend 0.3915 u (its turning left out), a twist half of r and of r × (c − origin). Per mode

```text
p_i   = Σ_k m_k t_ik · v_k + r_ik · I_k ω_k            the mode's momentum (B q̇: B's column j is p at unit speed of joint j)
Q_g,i = Σ_k m_k t_ik · g                               its load by gravity
M_d   = Σ_k m_k t_i · t_j + r_i · I_k r_j  + own       own: a bend m (1/4 − 0.3915²), a twist J_line / 12 (J_line about the link's line)
```

so that the own body's modal mass is the mode shape's (m / 4, J_line / 3) and M_d stays positive definite with the joints'. One-way the load is Q_g − ṗ (ṗ by central differences along q̇ and q̈) and the modal mass M_d's diagonal at zero joints. A tube with a tip inertia J twists at sqrt(G I_p / (L (J + J_own / 3))) / 2π: 60 × 4 mm steel (G 8.1e10 Pa), 0.8 m, J = 0.5 kg m², J_own = 3.546e-3 kg m²: 53.2699 Hz, 53.3328 Hz without the tube's share, the exact root of β tan β = J_own / J 53.26988 Hz; a round tube bends alike in both planes.

**Two-way** (`effects.link_bending_reaction`; off as shipped, then all of the above is unchanged to the bit). The modes are coordinates of their own beside the joints, one mass matrix for both (`chain::FlexCoupling`, `Mechanism::flex_coupling`):

```text
T = ½ q̇ᵀ M q̇ + ḋᵀ B q̇ + ½ ḋᵀ M_d ḋ,   V = V(q) − dᵀ Q_g(q) + ½ dᵀ K d
M q̈ + Bᵀ d̈ = τ − h + G                 G_j = ∂(dᵀ Q_g) / ∂q_j = Σ_i d_i Σ_k m_k g · ∂t_ik/∂q_j   (the bent links' weight shift)
B q̈ + M_d d̈ = Q_g − Ḃ q̇ − ½ Ṁ_d ḋ − K d − C ḋ
```

solved together (in the rigid model with the rotors' reflected inertia, in the elastic one behind the gearboxes' springs). Ḃ q̇ and Ṁ_d by central differences over ±1 µs along q̇; ∂t/∂q_j from the bodies' speeds at unit speed of joint j. With those terms T + V changes by the joints' work less the links' damping C ḋ² alone, which the machine reports as `power.link_bending`; T, V and ½ dᵀ K d are in `energy.*`. (Lagrange's terms of second order in the small d, ḋ — the centrifugal stiffening and the gyroscopic terms in ḋ q̇ — are left out; dropping the ½ Ṁ_d ḋ and G as well would leave a drift.) At rest the gears hold the weight less G. The joints feel the bending: the bending link's spring force reaches the joint through B, the motors see it in their following error and excite it through the loop.

`modes` lists, two-way, the modes of joints and links together with the motors held: K x = ω² M x over the joints (on their gearboxes' stiffness where elastic; held where rigid; passive joints held) and the modes, M of the load (`chain::generalized_modes`: Cholesky, Jacobi rotations), each with its damping ratio xᵀ C x / 2ω (C the gearboxes' and the links' dampers). A tube of 0.45 kg with 20 kg at its tip on a gearbox of k_g = 2e5 N m/rad, its motor held: joint and link are two springs in series for the tip, L² / k_g + 1 / k_b, k_s = 165420 N/m; the coupled mode 14.43363 Hz, the series spring on the modal mass sqrt(k_s / M_d) / 2π 14.43276 Hz (one-way the joint rings alone at 19.82 Hz and the tube clamped at 21.04 Hz); let go from straight the tip rings about its static sag L τ / k_g + Q_g / k_b = 1.1976 mm at that frequency.

Without the links two-way an arm (more than one joint, elastic) lists its joints' modes the same way with the motors held, K x = ω² M x over the joints alone on their gearboxes' stiffnesses with the gearboxes' dampers (`JointMachine::held_modes`), beside its links' own bending modes; `resonance_hz`, what `effects.shaper_follows_pose` sets the input shaper to, is the lowest (with the links two-way, the lowest of the coupled modes). Before 15 November 2026 an arm reported no resonance: the two-mass formula, motor and load on the gear with the motor free, is one joint's and stays so. Checks: two joints on 100 and 50 N m/rad with M = [[2, 0.5], [0.5, 1]] kg m² ring at 0.967314 and 1.399712 Hz, uncoupled each at √(k / J) / 2π (`held_joint_modes_are_the_hand_solved_eigenvalues`). The palletiser's turret at its arm's full reach reports 5.95 Hz (8.30 Hz at the long move's start); after the move it rings at 5.30 Hz. The motor is not held rigidly: its cascade holds the rotor with Z = Kp (1 + 1 / (jω Tn)) (Kv + jω) − ω² J, i² Z at the joint in series with the gear's k, which lowers the mode to 5.36 Hz, 1.0 % from the ring (`the_palletisers_turret_rings_at_its_held_mode_softened_by_the_velocity_loop`): the reported mode is the upper bound a stiff loop approaches. With the gears' mesh friction the ring rises again as it dies away (5.60 Hz over its first 3 s), the turret stuck to its housing for more of each cycle.


The six-axis arm's cycle in tool space (`arm-tool.toml`, the shipped `[link_bending]` estimates, `sim-cli analyze`), m and rad:

| Bending | Path error peak | Path error RMS | Tracking peak | Dwell peak (settling) | Dwell angle peak |
| --- | --- | --- | --- | --- | --- |
| off (shipped) | 3.781e-3 | 1.718e-3 | 5.869e-3 | 2.303e-3 | 3.089e-3 |
| one-way, in_plane | 3.879e-3 | 1.777e-3 | 6.019e-3 | 2.397e-3 | 3.227e-3 |
| two-way, in_plane | 3.817e-3 | 1.774e-3 | 5.962e-3 | 2.389e-3 | 3.215e-3 |
| two-way, all three directions | 3.823e-3 | 1.778e-3 | 5.962e-3 | 2.389e-3 | 3.215e-3 |

The links' modes (the upper arm's bend at 88 Hz) lie well above the gears' modes and the position loops, so the reaction changes the tool's errors by 2 to 3 % (re-run 17 November 2026; under 1 % on 15 November) and the other directions by less again: the shipped default stays as it was (bending off, one-way when switched on); two-way is for links soft enough to come near the gears' or the loops' frequencies, and where the bending's energy or its load on the motors matters.

The joints' units are their own (`JointSpec::unit`), no longer the axes': a joint's position signal, its force's unit and whether it has a gearbox output torque follow the joint. The time-optimal planner reads the motors' limits through `feedforward_torque_moving`, so along a line of the tool it slows where a motor reaches its rated speed (on the SCARA's example line, joint 1 near the middle, 307.5 of 314.2 rad/s). Before timing a path as fast as the motors allow, `Controller::plan_path` checks the path as planned within the axis limits: a pose on it the machine cannot go to (a singularity, where the motors would have to turn infinitely fast) is refused with `error-path-range` at once, where the time-optimal timing would slow to a standstill there.

## Tripod

Source: [`machine-tripod/src/lib.rs`](../crates/machine-tripod/src/lib.rs). A 3-RPS platform: three electric cylinders,
each a revolute joint at the base (about the tangent of the base's circle, radius rb), a ball screw leg of length
L_i = L_closed + q_i, and a spherical joint at the platform (radius rp), the joints at t_i = 0, 120, 240 degrees. Each
leg is a joint of `chain::JointMachine` (screw, coupling, friction, stops, motor; rigid and elastic as there). The tool
pose is x = (z, rx, ry); the platform's place follows in closed form:

```text
pose        R = Rx(rx) Ry(ry) Rz(g),  p = (p_x, p_y, z)
constraints e_i . (p + R a_i) = 0   (e_i = (−sin t_i, cos t_i, 0), each platform joint in its leg's plane)
            ⇔ R12 = R21,  p_x = rp (R11 − R22) / 2,  p_y = −rp R21
closed form tan g = −sin rx sin ry / (cos rx + cos ry),  p_x = rp (cos ry cos g + sin rx sin ry sin g − cos rx cos g) / 2,
            p_y = rp cos ry sin g      (second order: p_x ≈ rp (rx² − ry²)/4,  p_y ≈ −rp rx ry / 2,  g ≈ −rx ry / 2)
inverse     d_i = p + R a_i − B_i,  L_i = |d_i|,  u_i = d_i / L_i,  b_i = R a_i
twist       G xi = (q', 0),  G = [u_i, b_i × u_i ; e_i, b_i × e_i],  A = G⁻¹[:, 0..3],  xi = A q'
rates       z' = v_z,  rx' = w_x + tan ry (sin rx w_y − cos rx w_z),  ry' = cos rx w_y + sin rx w_z;  K⁻¹ = S A
forward     chord x += K(home)⁻¹ (q − q(x)), then K(x1)⁻¹ while each step halves the miss; to 1e-13 m
accelerat.  G xi' = (q'' − a0, −c0),  a0_i = (|w_i|² − (u_i · w_i)²) / L_i + u_i · (w × (w × b_i)),  w_i = v + w × b_i,
            c0_i = e_i · (w × (w × b_i));  xi' = A q'' + β,  β = G⁻¹ (−a0, −c0)
platform    M_xi = [[m 1, −[s]×], [[s]×, I_p]],  s = R s_b,  I_p = R I_b Rᵀ   (about the platform centre)
            c = (w × (w × s), w × I_p w),  g = (0, 0, m g, g s_y, −g s_x, 0)
joints      M_q = Aᵀ M_xi A,  h = Aᵀ (M_xi β + c + g),  M_q q'' + h = f      (Aᵀ Cᵀ = 0, Aᵀ Jᵀ = 1)
potential   V = g (m p_z + s_z)
momentum    L = m v + w × s,  H_O = s × v + I_p w + p × L,  first moment m p + s
```

m, s_b and I_b (platform frame, about the centre) sum the platform (`mass.platform_kg`, `platform_inertia_kgm2`), the
and, attached, the payload (`mass.payload_kg` at `payload_offset_m` with its own `payload_inertia_kgm2`, by the
parallel axis theorem). Each leg is two more bodies swinging with its direction u, added to M_xi and c + g by d'Alembert:
the rod part (`leg.platform_share` of `leg.mass_kg`, centre `leg.platform_part_centre_m` from the platform joint) at
X = P − c u and the body part (the rest, centre `leg.base_part_centre_m` from the base joint) at X = B + c u, each with
its inertia about its centre across the leg (`leg.*_part_inertia_kgm2`; u' = (1 − u uᵀ) d' / L, d' the platform joint's
velocity). The
constraint forces at the base joints (along e_i) do no work along the platform's admissible motions, so projecting the
Newton-Euler equations with Aᵀ removes them; at rest f = Aᵀ g. Level with the load centred each leg carries m g / (3
u_z); off centre by c the vertical shares are m g (1/3 + 2 c · â_i / (3 rp)).

**Signals**: the screws' per leg; `leg.<i>.from_middle` (q_i − stroke/2), `leg.<i>.swing` (atan2(u_i · r̂_i, u_iz), the
revolute joint's angle from the base's normal), `leg.<i>.cone` (acos(u_i · R e_z)); `platform.condition` (of diag(1,
rp, rp) K⁻¹, which K shares), `platform.x`, `platform.y`, `platform.turn` (p_x, p_y, g) and `platform.pose_error`, the
largest distance of a point of the joint circle from the pure tilt's place, sqrt(|d|² + c² + 2 c |(R0ᵀ d)_xy|), d = (p_x,
p_y, 0), c = 2 rp |sin(g/2)|. **Verdicts**: `tripod-leg-force`, `tripod-leg-stroke`, `tripod-revolute-angle`,
`tripod-cone-angle` per leg and `tripod-parasitic` (with `ratings.parasitic_tolerance_m`). **Refusals**:
`error-tripod-revolute-angle`, `error-tripod-cone-angle`, `error-tripod-singularity`, and the joint machine's
`error-joint-beyond-stop`. **Base**: `frame_momentum` as above with the rotors' spin along u_i, for `base::mount`.

Not modelled: the legs' swing inertia about their base joints, friction and play in the revolute and spherical joints,
the joints' and the platform's elasticity (the legs' screws are the only springs).

## Wafer handler

Source: [`machine-wafer-handler/src/lib.rs`](../crates/machine-wafer-handler/src/lib.rs). The R-θ-Z arm of semiconductor and flat-panel handling: a column lifts (z, motor 3, a ball screw behind a coupling) and turns (θ, motor 1) an arm of two links of equal length L in the horizontal plane. Motor 2 rides on the column and turns the upper arm; a band in the upper arm from a pulley fixed to the column (radius 2 r₁) to one on the forearm (r₁) turns the forearm against the upper arm at 2:1, a band in the forearm from a pulley fixed to the upper arm (r₂ / 2) to one on the effector (r₂) turns the effector against the forearm at 1:2. One motor runs the wrist along the radial line and the effector keeps pointing outwards. The tool pose is (r, θ, z): r the wafer centre's distance from the column's axis along the effector, θ the effector's direction, z the stroke.

```text
coordinates  q = [θ, φ, β, γ]: θ the column, φ the upper arm from the radial line, β and γ the bands' stretch (0 when rigid)
angles       a₁ = θ + φ,  a₂ = θ − φ + β,  a₃ = θ + γ                    upper arm, forearm, effector
wafer        C = L u(a₁) + L u(a₂) + e u(a₃) = (2 L cos φ + e) u(θ)       β = γ = 0; e the effector's length to the wafer centre
reach        r = 2 L cos φ + e,  dr/dφ = −2 L sin φ,  condition 1 / sin φ  (1 with the wrist over the column, ∞ at full extension)
inverse      φ = acos((r − e) / 2 L),  motors i₁ θ, i₂ φ, i₃ z
dynamics     M(q) q̈ + h(q, q̇) = Q,  M = J_c e₀e₀ᵀ + Σ m J_vᵀ J_v + I a_k a_kᵀ,  h = Σ m J_vᵀ (J̇_v q̇)
             J_v = Σ over the segments to a body's centre l n(a_j) a_jᵀ,  J̇_v q̇ = −Σ l (a_j · q̇)² u(a_j)
bands        s₁ = −r₁ β,  s₂ = r₂ (β / 2 − γ);  span tensions T₀ ± k s + c ṡ (0 once a span is slack)
             Q_β = r₁ ΔT₁ − r₂ ΔT₂ / 2,  Q_γ = r₂ ΔT₂;  stored ∫ T over both spans: k s² while both are taut
```

The bodies are the upper arm, the forearm, the effector and the wafer while it is held (each with its mass, its inertia about its centre and its centre on its link's line, `[mass]`), and the column's inertia J_c (`mass.column_inertia_kgm2`: the column's head, motor 2, the shoulder pulley) on θ. h carries the coupling of the two motors: turning at θ̇ pulls the arm outwards, h_φ = −½ ∂M_θθ/∂φ θ̇² (M_θφ is the same at every pose, so no other term at rest), which motor 2 holds; extending during a turn gives the wafer the Coriolis acceleration 2 ṙ θ̇ across its line, which motor 1 takes. M_θθ grows with r² (the wafer's m r², the effector's m (2 L cos φ + c₃)²); M_φφ seen along r, M_φφ / (dr/dφ)², grows without bound towards the full extension. Gravity acts along −z, on the joints' bearings: the plane's motors hold nothing at rest, the Z motor everything it lifts (column, motors 1 and 2, links, effector and wafer: m g lead / 2π).

Motors 1 and 2 drive θ and φ through `RotaryJoint`s (`[joint]`, and `[joint.2]` for the upper arm's stops over it), elastic with `effects.joint_elasticity` (rotor and link two bodies on the gearbox's spring) or rigid (the rotor at i q, its inertia i² J_rotor on the diagonal of M), with friction, efficiency, brake and ripple as on every joint. With `effects.band_elasticity` β and γ are states on the bands' springs (`[elbow_band]`, `[wrist_band]`: the distal pulley's radius, a span's stiffness and damping, the pretension); off, the bands are rigid and the arm has θ and φ alone. The stroke is a rigid `RotaryJoint` on the shared `BallScrew` (`[slide]`, `[screw]`, motor 3's `[gearbox.3]` coupling). States: θ, φ (β, γ) and their rates; with elastic gearboxes the two rotors' angles and speeds; the two brakes; then z, ż and the Z brake.

**Signals**: `arm.1.angle` (θ), `arm.2.angle` (φ), `gearbox.1.output_torque`, `gearbox.2.output_torque`, `kinematics.condition`, `effector.direction_error` (the effector's angle a₃ less the wafer centre's polar angle: its turn out of the radial line through its wafer, from the bands' stretch; 0 with rigid bands), `effector.direction_error_at_station` (the same while the cycle dwells, else 0), `wafer.acceleration` (the wafer's in-plane acceleration from the arm's accelerations, as the in-plane acceleration that asks the same share of the hold at rest: |a_xy| (g + F/m) / (g + a_z + F/m) on pads and vacuum, |a_xy| on an edge grip), `band.elbow.pretension_share` and `band.wrist.pretension_share` (k s / T₀ of a span from the stretch; with rigid bands ΔT / 2T₀ from the forces G = M q̈ + h the rigid model asks on β and γ: ΔT₂ = G_γ / r₂, ΔT₁ = (G_β + G_γ / 2) / r₁), `joint.z.force` and the screw's; in the effect views the torque parts, losses (`power.band_elbow`, `power.band_wrist`: the spans' dampers) and energies.

**Verdicts** beyond the common ones and the screw's: `wafer-slip` (the highest `wafer.acceleration` against the hold at rest over `ratings.wafer_slip_safety_factor`: μ g on friction pads, μ (g + A p / m) on vacuum pads, `wafer.edge_grip_acceleration_m_s2` on an edge grip; `[wafer]`), `effector-direction` (`effector.direction_error_at_station` against `ratings.direction_tolerance_rad`), `elbow-band-slack` and `wrist-band-slack` (the tension swing against `ratings.band_max_pretension_share` of the pretension). The settling verdicts judge the wafer centre's r and z (`settling`) and θ (`settling-angle`). The `gripper-hold` verdict of `gripper.rs` judges a cup above its part, which gravity pulls off; a wafer lies on its pads, which gravity presses on, so the hold is the machine's own (it takes `gripper::gravity` and the wafer's mass from `mass.payload_kg`). `rigid_checks` gives the time-optimal plan the screw's rows and the bands' swings (linear in the acceleration); `wafer-slip` is no row of it: the wafer's acceleration is a vector's length, and the plan's rows are linear in the acceleration and the speed squared along the path. The shipped cycle is timed within axis limits sized for the hold, and `sim-cli optimise` judges it with every verdict.

**Modes**: with the motors held, the generalised eigenvalues of K and M over the coordinates that move (Cholesky, Jacobi rotations): the bands' K = [[K₁ + K₂/4, −K₂/2], [−K₂/2, K₂]] on β, γ (K = 2 k r²), the gearboxes' stiffness on θ and φ when they are elastic. Shipped: 18.2, 51.4, 70.6, 186.4 Hz at r = 0.12 m, 16.8, 42.3, 83.7, 175.3 Hz at r = 0.75 m.

**Shortcuts**: motor 2's rotor counts at its speed against the column (its own spin with the column, J_rotor θ̈, 1.0e-3 N m at the turn's 36 rad/s², is left out); the stroke is rigid; the wafer is fixed to the effector (its slip is judged, not simulated); the links' centres lie on their lines; the bands' own mass is in the links'.

## Toggle press

Source: [`toggle.rs`](../crates/machine-crank-cam/src/toggle.rs); the mechanism in
[mechanisms](mechanisms.md#crank-and-cam-mechanisms-machine-crank-cam). One motor-driven joint (`chain::JointMachine`,
as every machine of the crate): the crank angle q from the links' dead centre is the joint and the tool axis, so the
motor, gearbox (twist and backlash in the elastic model), drive and checks are the common ones, and the friction
feedforward (`steady_friction`), the time-optimal plan's friction and checks (`rigid_friction`, `rigid_checks`), the
two-mass mode (`resonance_hz`, `modes`) come from the joint machine. No condition number: the tool is the joint.

**Kinematics.** Top pivot O at the origin, x across, z up. Pin P(q) = C + r (cos t, sin t), t = t0 + q; knee
K = (a sin φ, −a cos φ) with φ = atan2(P_z, P_x) − acos((a² + |P|² − c²) / (2 a |P|)) + π/2 (the branch below O-P);
ram x = a cos φ + √(b² − a² sin² φ) below O. Rates from |K − P| = c: φ' = D·P_q / D·K_φ,
φ'' = −(E·E + D·(K_φφ φ'² − P_qq)) / D·K_φ, D = K − P, E = K_φ φ' − P_q; x' = x_φ φ', x'' = x_φφ φ'² + x_φ φ''. The
build refuses a geometry whose four-bar does not close over the joint's range or comes near its own dead point
(|φ'| ≥ 50). The inverse from the ram's distance d above the bottom dead centre: cos φ = (a² + x² − b²) / (2 a x),
φ = ±acos (before, after the dead centre), then the crank on the circle of radius c about K, its assembly the one that
puts it at t0 with the links stretched.

**Dynamics.** Lumped masses: the rod's crank end m_pin at P, the upper link a uniform bar (J = m_up a² / 3 about O,
first moment at a / 2), the knee m_knee (the rod's other end and `toggle.lower_link_knee_share` of the lower link), the
ram m_x (ram, die, the rest of the lower link). M(q) = J_crank + m_pin r² + (m_up / 3 + m_knee) a² φ'² + m_x x'²,
h = ((m_up / 3 + m_knee) a² φ' φ'' + m_x x' x'') q'² + g (m_pin r cos t + (m_knee + m_up / 2) a sin φ φ' − m_x x').
`frame_momentum` gives the same bodies (pin, knee, the bar's centre and its spin about −y, ram, crank, rotor) and the
press force on the ram, so the machine stands on the compliant base as the others of the crate.

**Process force and frame.** d = a + b − x. Without `effects.frame_stretch` F = table(d); with it the frame (stiffness
k) is a massless spring in series and F the root of F = table(d + F / k): on the table's segment i (from d_i, F_i, slope
s_i), F = (F_i + s_i (d − d_i)) / (1 − s_i / k), the first segment on which d + F / k falls. The force acts while the
ram moves down or stands, times clamp(1 + v / v_s, 0, 1) with v = x' q' the ram's speed down and v_s =
`process.smoothing_m_s`: a dwell holds it, and it is gone once the ram rises at v_s (the part springs back). Its torque
at the crank −F x' (`process_torque`, outside the feedforward); the frame's stretch F / k is the signal
`frame.deflection`, and the ram's height above the bottom dead centre `ram.height` = d + F / k; `ram.speed` and `ram.acceleration` are its first and second derivatives, the stretch's included (by differences of F / k over the crank's angle and speed; where the force fades as the ram rises the stretch also follows the crank's jerk, which is left out). The energy the frame
takes on the way down is lost when the force fades on the way up (no rebound into the drive).

**Loads.** Ram along z: C_b w / b = m_x (ẍ − g) + F, w = x − a cos φ, C_b the lower link's compression. Upper link about
O: across it at the knee T = (−m_up g a sin φ / 2 − J φ̈) / a. Knee: m_knee K̈ = C_a u_a + T n_a − C_b u_b + C_rod u_rod +
m_knee g, u_a = K / a, n_a = K_φ / a, u_b = (R − K) / b, u_rod = (K − P) / c, solved for C_a and C_rod. The links and the
rod are two-force members, so their pins carry the same. Capacity at q: min(F_rated, τ_max / |x'|), τ_max = peak torque ·
ratio · efficiency, at most the gearbox's maximum torque.

**Limits.** The crank-driven form only (the screw-driven toggle is the same links with a linear joint at the knee, not
built); the links, the pins and the crank are rigid (only the gearbox and the frame give); the frame's stretch is
quasi-static (no frame mode); the part's springback is a fade over a speed, not a stiffness.

## Arm on a track

Source: [`machine-joint-kit/src/arm.rs`](../crates/machine-joint-kit/src/arm.rs) (`Arm::guide_load`, the rack in `build`), the tool axis in [`tool.rs`](../crates/machine-joint-kit/src/tool.rs); the machine `arm-on-track`, [`urdf/arm-on-track.urdf`](../crates/machine-joint-kit/urdf/arm-on-track.urdf), [`config/arm-on-track.toml`](../crates/machine-joint-kit/config/arm-on-track.toml). The six-axis arm on a carriage whose prismatic joint along x comes first in the URDF chain: the track is a joint of the Newton-Euler model like the arm's, so M(q) couples them both ways (the arm's swing in the track's force, the track's acceleration in every arm joint's torque) with nothing added. The track's joint runs on a pinion and rack behind its gearbox (`[rack]`, the rack axis's `transmission::RackPinion` as a converter), its tooth force `rack.1.feed_force` and the pinion's radial load on the gearbox shaft `rack.1.radial_load` = |F| √(1 + (tan 20° / cos β)²) (`RackPinion::radial_load`, as the machine files' racks).

**Guide blocks** (`[track]`: `block_spacing_m` l, `rail_spacing_m` w; four blocks at (±l/2, ±w/2) of the first joint's frame, its origin between them):

```text
F   = dP/dt + Σ_k m_k g ẑ                    the base's force on the chain (P its momentum; payload with the last body)
M_O = dH_O/dt + Σ_k c_k × m_k g ẑ            its moment about the base's origin (H_O the angular momentum about it)
M   = M_O − o × F,   (F, M) into the carriage's frame (o, R its origin and rotation)
F_y += s F_sep,  M_z += s λ F_x               s = `rack_side` (±1), λ = `rack_lever_m`, F_sep the helical pinion's separating force
R_i = F_z/4 + s_y M_x/(2w) − s_x M_y/(2l)     on the rail, block at (s_x l/2, s_y w/2)
T_i = F_y/4 + s_x M_z/(2l)                    across it
P_i = |R_i| + |T_i|,   axis.j1.guide_load = 4 max_i P_i
```

dP/dt and dH/dt by central differences over ±1 µs along q̇ and q̈ (as the link bending's loads); F_x is the rack's. Four times the most loaded block, because the guide verdicts share a load equally among a set of `catalog::GUIDE_CARRIAGES` = 4 and rate the set (`guide-load` against `ratings.guide_max_load_n`, `guide-life` (C/P)³ 50 km over the track's mean speed). Shortcuts: the rack's force through o, the arm's rotors' gyroscopic moments left out, the blocks rigid and statically shared; `rigid_guide_loads` is not given, so the time-optimal plan does not keep the guide rating.

**Tool space with the track** (`kinematics.tool_axes = ["j1", "x", …, "rz"]`, `tool::FIRST_JOINT`): the first joint's position is a tool axis of its own ahead of the pose's, its row of the Jacobian (1, 0, …, 0) unweighted, so seven axes for seven joints: not redundant, the track given and the arm solved by the damped least squares above with it where it is. A line moves the track and the flange straight together.

## Winder with dancer

Source: [`machine-winder/src/lib.rs`](../crates/machine-winder/src/lib.rs). A web runs from an unwind roll down over a
dancer roller and up to a rewind roll; both rolls are centre-driven by a servo motor through a gearbox of ratio i; the
dancer is a roller on a vertical slide pushed down by a pneumatic cylinder. The tool axis is the line position L (m), the
web wound onto the rewind since it was empty, which is the web the unwind has paid out since it was full when the dancer
stands in the middle: 0 has the unwind full and the rewind on its core, the roll's capacity π (r_f² − r_c²) / h the other
way round. The axis runs over a stretch of the roll (`roll.run_from_m`, `run_m`; shipped its first 6 m). Of a state the
line position is the mean of the two rolls', ½ (s_u − trim + s_r) (the dancer's trim below); in the rigid model the web's
travel at the dancer roller.

**Rolls.** A roll of web of thickness h gains h per turn, an Archimedean spiral: r = r_c + h φ / 2π (rewind, wound from the
core), r = r_f − h φ / 2π (unwind, paid out from full), and r² = r_c² + h s / π with s the web on it, s(φ) = φ (r_0 + r) / 2.
Its inertia is the web's as a thick-walled cylinder plus core and shaft, J(r) = ½ ρ w π (r⁴ − r_c⁴) + J_core; the motor's
rotor and gearbox input add i² J_m. A roll that gains or loses web where the web moves with its surface gets no torque from
the mass flow, so

```text
unwind   (J_u + λ_u i² J_m) ω_u' = λ_u i (τ_u − τ_f) + T1 r_u        (ω_u in the paying-out sense)
rewind   (J_r + λ_r i² J_m) ω_r' = λ_r i (τ_r − τ_f) − T2 r_r
```

not d(J ω)/dt: the kinetic energy ½ λ v³ per second the web carries from roll to roll (λ = ρ w h) is booked with the
web's transport below. λ_k is the gearbox's factor, η^tanh(P / 0.5 W) on the power P it passes (`effects.gearbox_efficiency`,
as every machine's gearbox; with `effects.thermal_feedback` η and the drag at the gearbox's temperature), τ_f the shaft's
friction, the gearbox's no-load drag (`effects.gearbox_drag`, booked as the gearbox's loss) and, while the drives are
off, the holding brake (`mech::HoldingBrake`, a deflection state per motor). The torque each gearbox passes on,
λ i τ_in, is the signal `gearbox.<motor>.output_torque`.

**Web** (`effects.web_stretch`). Two spans, unwind to dancer and dancer to rewind, each vertical, of length L_k0 + y (y the
dancer's travel down from the middle), each holding M_k of web unstretched; T = EA (L / M − 1) + EA τ_d ε̇ while L > M, 0
while slack (`web.slack_time` counts the seconds). The web carries its strain: what crosses a boundary crosses it at the
strain of the side it comes from,

```text
dM1/dt = v_u / (1 + ε_0) − v_d / (1 + ε_1),     dM2/dt = v_d / (1 + ε_1) − v_r / (1 + ε_2)
```

ε_0 = `roll.wound_tension_n` / EA the strain the unwind roll was wound at, v_d the roller's surface speed; backwards the
upstream side is the other one (the rewind giving off web at the tension setpoint's strain). To first order this is the
span equation L dT2/dt = EA (v2 − v1) + v1 T1 − v2 T2 (+ EA dL/dt); the form in M keeps the web's length exactly. The span
lengths do not change with the rolls' radii (the web leaves each roll over a fixed idler, not modelled), and the web's own
mass in the spans is left out.

**Dancer.** The roller turns with the web, J_d ω_d' = R (T2 − T1) − drag; the slide carries it, m y'' = F_cyl − k_air y + m g
− T1 − T2 − friction + stops, m the slide and roller (`mass.dancer_kg`) and a weight a cycle segment's `payload` hangs on
it (`mass.payload_kg`). The cylinder's force is set to hold the tension setpoint, F_cyl = 2 T_set − m g, the dancer's
weight (with the weight on it) taken off, with the taper T_set = T0 (1 − k (1 − r_c / r_rewind))
(`effects.tension_taper`). Its potential is −(F_cyl + m g) y + ½ k_air y²; while the taper changes F_cyl the loss row
`power.cylinder` = dF_cyl/dt · y books the energy that puts in.

**Rigid model** (`effects.web_stretch` off): the web does not stretch, the dancer stands at y = (s_u − s_r) / 2 and the
roller turns at (v_u + v_r) / 2R. With a_k = b_k ± k_k T_k the surfaces' accelerations (b from the drive, k = r² / J), y'' =
(a_u − a_r) / 2 and R ω_d' = (a_u + a_r) / 2 into the dancer's and the roller's balance give two linear equations in T1 and
T2, solved loss-free and again with the gearboxes' factors.

**Energy.** `energy.*`: the rolls with their rotors, the dancer and its roller; the dancer's weight and the cylinder; the
spans' EA (L − M)² / 2M and the stops. The loss rows: each shaft, each gearbox, the dancer (slide and roller), the web's
damping, the stops, `power.web_transport` (the balance of what the tensions do on the bodies, what the spans store and
what their damping takes, plus the kinetic energy the web carries from roll to roll: ½ λ (v_u³ − v_r³) and the strain
energy the web brings into and out of the spans, T v (ε_in − ε) to first order) and `power.cylinder`.

**Control.** Both motors follow the line position by electronic gearing: `inverse` gives each roll's angle at the radius
of its wound length (2 s / (r_0 + r)), so the speed setpoint is i v / r and the position loop holds the roll on the line.
The torque feedforward (`feedforward_torque_moving`) is the tension's torque and the inertia compensation at that radius,

```text
τ = (J(r) ω' ∓ T r) / i,   ω' = (a ± h ω² / 2π) / r      (unwind −T r and +; rewind +T r and −)
T1, T2 = T_set ∓ J_d a / (2 R²)                            (the roller's inertia between the spans)
```

The dancer's position loop (`dancer.position_loop`), a PI as a winder's technology controller runs it, trims the
unwind: trim = −(Kp y + Kp / Tn ∫ y dt), within ± `trim_max_m_s`, every base step while the drives are on (`after_step`).
The trim's integral, the web it has had the unwind pay out beyond the line, goes into the unwind's wound length in
`inverse`: the unwind runs on the line speed over its radius plus the trim, and its radius follows the web it gave.
Without the loop a roll wound at another tension than the web runs at drifts the dancer at v (ε − ε_0) / 2 (the draw).

**Signals.** The spans' tensions `web.span.<k>.tension`, the highest `web.tension_peak`, `web.slack_ratio` (the minimum
tension over the lower span's), `web.tension_setpoint`, `web.slack_time`; `dancer.position`, `speed`, `load_force`,
`trim_speed`; `rewind.radius_from_speeds` (r_u ω_u / ω_r, as a drive works the radius out); per roll its radius, inertia
ratio now and at the full roll, and the figures of the sizing at both ends of the roll.

**Own verdicts.** `web-tension` (highest span tension, `ratings.web_max_tension_n`), `web-slack` (the minimum over the
lowest, at most 1), `dancer-travel` (`ratings.dancer_max_travel_m`), and per motor: `inertia-ratio` (the full roll's
J / (i² J_m), `ratings.max_inertia_ratio`), `torque-at-full-roll` and `torque-at-core` (the torque of the formula above at
the full roll and at the core, at the line's speed and acceleration of the moment and the tension the span would have
there: the setpoint at that end of the roll plus what the span is off its setpoint now; against `drive.peak_torque_nm`),
`speed-at-core` (i v / r_c, `drive.max_speed_rad_s`) and `tension-torque-at-full-roll` (T r_f / i, `drive.rated_torque_nm`).
A run takes metres of a roll of hundreds; these carry the cycle's line speed, acceleration and tension to both ends, where
the speed is highest at the core and the torque at the full roll. `rigid_checks` gives them for the time-optimal plan
(the dancer's travel: none in the rigid model).

**Modes.** The dancer on the spans with the rolls held, k = EA / L1 + EA / L2 + k_air on the dancer and what it carries;
the roller turning on the spans, R² (EA / L1 + EA / L2) on J_d.

**Stops.** A category 2 stop brakes the line along its path and keeps the web's tension at every radius. A category 1 stop
brakes each motor on its own OFF3 ramp, the same angular deceleration: the surfaces slow in proportion to their radii, so
where the radii differ the rolls part and the web is overloaded or goes slack (tests
`an_emergency_stop_along_the_line_keeps_the_web_within_its_limits`,
`a_stop_on_each_drives_own_ramp_overloads_the_web_where_the_radii_differ`). Once the drives are off, the holding brakes
keep the rolls and with them the tension; meanwhile the trim takes up what the unwind paid out more than the rewind took
(the dancer's travel), so that enabled again each motor holds where it stands and the dancer's loop brings the dancer
back.

**Drives.** The shipped config has the two Motor Modules of an S120 line-up with a Braking Module (`drive.braking_module`),
each with its own 110 µF of DC link; with `effects.shared_dc_link` (off as shipped, as on every machine) the two share
one link and what the unwind brakes, the rewind takes. On S210 converters of 66 µF the unwind's DC link tripped on
overvoltage under a stream of setpoints at the speed limit, its roll's inertia braked in steps.

## Hexapod with rotary actuators (6-RSS)

`machine-hexapod-rotary` (id `hexapod-rotary`): six cranks on the base, each turned by its motor through a gearbox
(`joint::RotaryJoint` in `chain::JointMachine`, as the arms), each crank's tip joined to the platform by a rod with a
ball joint at both ends. The joints are the crank angles θ_i; the tool pose is the platform centre p and its rotation
vector.

### Kinematics

Crank i turns about the horizontal axis k_i = d_i × z through B_i, d_i its zero direction (horizontal, in its plane):

```text
tip      C_i = B_i + r (cos θ d_i + sin θ z),  t_i = dC/dθ,  t2_i = d²C/dθ²
rod      |P_i − C_i| = L,  P_i = p + b_i,  b_i = R a_i,  u_i = (P_i − C_i) / L
inverse  e = P_i − B_i:  a cos θ + b sin θ = c,  a = e · d_i,  b = e_z,  c = (|e|² + r² − L²) / 2r
         θ = atan2(b, a) ∓ acos(c / √(a² + b²))   ("out" −, "in" +; refused where |c| > √(a² + b²))
rates    A ξ = diag(m) θ',  A row i = (u_i, b_i × u_i),  m_i = u_i · t_i  (the torque arm),  ξ = K θ',  K = A⁻¹ diag(m)
         A ξ' = diag(m) θ'' + s,  s_i = u_i · t2_i θ_i'² − |v + w × b_i − t_i θ_i'|² / L − u_i · (w × (w × b_i))
forward  Newton on the twist from the centre of the tool's box: ξ = A⁻¹ (L − |P_i − C_i|), p += v, R = exp(w) R
```

The torque arm is m_i = e · t_i / L = r √(a² + b²) sin δ / L, δ the closed form's acos: zero at the dead centres δ = 0
and π, where the rod lies in the plane of the crank and its axis. `condition` is the condition number of diag(m)⁻¹ A
with the turns' columns over the platform radius.

### Dynamics

By virtual work in θ (M θ'' + h = τ at the cranks):

- the platform with the payload, one rigid body: mass m, first moment S = R s and inertia I = R I_p R^T about p, s and
  I_p from both bodies' centres of mass (`mass.platform_cg_m`, `payload_cg_m`) and inertias; M_ξ = [m 1, −[S]×; [S]×,
  I], c = (w × (w × S), w × I w), G = (m g, S × g): M += K^T M_ξ K, h += K^T (M_ξ A⁻¹ s + c − G);
- each rod a uniform slender rod as three points, m/6 at C_i and at P_i and 2m/3 at the middle (exact in mass, centre
  and the inertia across; the spin about its own axis, free between two ball joints, is left out), each with its
  Jacobian P in θ and its acceleration a0 at θ'' = 0: M += m P^T P, h += m P^T (a0 − g);
- each crank its inertia about its axis and its weight at `crank.cg_m`: h_i += m_c g r_cg cos θ_i.

The rods' forces (signal `rod.<i>.force`, compression positive): A^T f = W, W the body's required wrench plus each rod's
upper sixth and half its middle at P_i; the test `the_virtual_work_agrees_with_the_forces_along_the_rods` checks that
each crank's torque from M and h equals its own Newton-Euler balance with f_i m_i (to 1e-9).

### Signals and verdicts

Per leg `crank.<i>.dead_centre_angle` (acos(|m_i| / r), 90° at the dead centre), `rod.<i>.force`, `rod.<i>.compression`
(its positive part), `crank.<i>.ball_joint_angle` (asin |u_i · k_i|) and `platform.<i>.ball_joint_angle` (acos(u_i ·
R z)); `kinematics.condition`. The verdicts `crank-dead-centre`, `rod-buckling` (π² E I / L² of the tube over
`ratings.rod_buckling_safety_factor`), `crank-ball-joint` and `platform-ball-joint` judge their peaks; the gearboxes'
output torque (`gearbox.<i>.output_torque`) is judged by the common gearbox verdicts.

## Compliant base

Source: [`sim-components/src/base.rs`](../crates/sim-components/src/base.rs). With `effects.base_compliance` every machine and machine file stands on a base that gives way: a rigid body on springs in up to six directions, translation along x, y, z and a small tilt about x, y, z through `base.pivot_m`, in the frame of the tool pose (x, y, z of the tool; the slides' x along their axis). Each direction with a stiffness above 0 is a state d and its rate after the machine's own:

```text
M d'' + 2 ζ sqrt(k M) d' + k d = L                          (per direction; M = base.mass_kg, or base.inertia_*_kgm2 for a tilt)
F = −dp/dt + m g + F_ext,   T = −dL/dt + c × g + r_ext × F_ext   (about the origin; about the pivot T − P × F)
L = (F, T) − (F, T) at rest at the start pose
```

p and L are the momentum and the angular momentum of everything that moves on the base (`Machine::frame_momentum`: bodies, arms, rotors and pulleys spinning), m and c its mass and first moment, F_ext a process force from a workpiece on the floor; the motors push on their stators as they push the load, so the base takes dp/dt back. dp/dt is taken along the machine's own state rates, by central differences over ±1 µs. What stays put cancels against the start pose's load, at which the base was set up: a machine's weight does not sag it, its weight moving out does. M is everything that sways with the base, the machine on it included, so a direction's frequency is sqrt(k / M) / 2π (the modes list them, and `resonance_hz` takes the lowest that moves the tool along its first axis, for the input shaper).

The tool rides on the base and the workpiece stands on the floor: the tool pose is the machine's plus the base's move at the tool, u + θ × (r − P) for the axes x, y, z and θ for the turns (rx, ry, rz, c about z; `Machine::base_shift`); the motors' angles are measured on the base and do not see it. At rest the machine stands off by the base's deflection under its load so that the tool is at its pose. The scene moves as a whole by the base's move at the tool, exaggerated. Signals `base.<x|y|z|rx|ry|rz>`.

Checks: a force held at rest moves the base and the tool by F / k (single axis, 50 N on 2.5e6 N/m: 20 µm); after a move an undamped base rings at sqrt(k / M) / 2π (100 kg on 2.5e6 N/m: 25.165 Hz, half period 19.869 ms, in the tool's position); every machine's momentum is the rate of its first moment (`testkit::frame_momentum_moves_its_first_moment`). Two-way: a slider (4 kg) on a spring (2e5 N/m) to a base of 200 kg on 8e5 N/m swings in the two-mass system's closed-form modes, 10.057 Hz and 35.981 Hz, each in its shape (to 1e-6 of the amplitude; one-way the base rings at 10.066 Hz whatever the slider does); a free base and an arm swung on it keep their momentum (to 1e-9); the arm in gravity on a base along x, z and in tilt keeps its energy (`testkit::energy_conservation`; without the tilt's turn of gravity it drifts by 0.32 %) and closes the loss budget, and so does the H4 (one-way it drifts by 4.5 %); on the T-gantry a two-way base along X of the frame's mass plus what rides on it (30 + 11 kg and the belt's 0.42 kg), its stiffness and damping, gives the frame mode's motion (`effects.frame_compliance`) to 5e-15 m against a sway of 0.149 mm.

**Two-way** (`effects.base_reaction`, with `base_compliance`; off in every shipped config). Without it the coupling is one-way: the machine's own dynamics do not feel the base's acceleration, the motors neither see the sway in their following error nor hold against it, and the base's energy is not in the budget; exact where the base is much heavier than what moves on it or the servo holds the axes stiffly to it. With it the base's acceleration acts on everything that moves on it as an inertial force, derived from the same momentum, not written per machine. The machine's momentum is linear in its speeds v (the states its momentum moves with and whose change moves more than speeds: a frame mode or a tool flex switched off, held at zero, is none; found at the start pose), P = A(q) v, linear and angular about the pivot; A turns the base's acceleration a into the generalised forces −Aᵀ a on the speeds, as it turns gravity into A_linᵀ g. The machine's mass matrix over its speeds is the Hessian of its kinetic energy, M = ∂²E/∂v², from `Machine::energy` at unit speeds (by polarisation: exact, E being quadratic in v). Base and machine are solved together at every evaluation of the derivatives, no lag:

```text
(M_B − A M⁻¹ Aᵀ) a = L − k d − c d' − A M⁻¹ Aᵀ w        (over the compliant directions; L as above, with the machine's own rates)
v'' = v₀'' + M⁻¹ Aᵀ (w − a)                            (v₀'': the machine's own accelerations)
w = (g × θ, 0)                                          (the base's tilt θ turns gravity in the machine's frame)
E_B = ½ M_B d'² + d' · P + ½ k d² − θ · (c × g − c₀ × g)
```

E_B, the base's energy (its sway, its sway with the machine's momentum, its springs, the weight's work on its tilt), is in `Machine::energy` and in the signals `energy.kinetic`, `energy.elastic` and `energy.potential`; its damping c d'² is the signal `power.base`, so the loss budget closes with the base on. M_B − A M⁻¹ Aᵀ has to be positive: a base lighter than what the machine moves on it is refused (`base.mass_kg`, `base.inertia_*_kgm2`). The cost: each evaluation takes the machine's energy at 1 + n + n(n − 1)/2 speed settings and its momentum at n + 3 (n speeds: 4 on the rigid H4, 9 elastic with its amplifier).

Assumptions (shortcut in `base.rs`): the gyroscopic and centrifugal terms of the base's tilt rates on the machine (second order in the small tilt; they do no work, so the energy stays conserved) and the change of the base's inertia with the machine's pose are left out; the process force's work on the moving base is not in the budget. The frame of the T-gantry and the machine files (`frame_compliance`) is the two-way base along one axis, in series with this one. The machine files' belt drives count their pulleys', rotors' and idlers' spin about the normal of the belt's plane (a screw's rotor about its axis); the table's worm motor is taken about z, the T-gantry's idlers and the H4's forearm spin are left out; the parallel SCARA's arm is taken as a uniform rod for its first moment (its config gives its inertia, its weight being out of the plane), the crank-cam machines' rotors turn about −y with their crank, roll or disc (the saw's pinion about y), and the knife's cut acts at the cut point.

The cycles of the default configs with the shipped estimates (`[base]` of each config): base off → on one-way → on two-way (`base_reaction`), tracking error peak and dwell error peak (what the settling verdict judges), m (rad where the tool is a turn or the joints). Measured on 15 November 2026, re-run 17 November (`sim-cli analyze ID --set effects.friction_feedforward=false`, with `--set effects.base_compliance=true` and then `--set effects.base_reaction=true`); the two-way runs took 3 to 10 times as long as the one-way ones (the H4 161 s against 17 s, the SCARA 301 s against 34 s).

| Machine | Base | Tracking peak off → one-way → two-way | Dwell peak off → one-way → two-way |
| --- | --- | --- | --- |
| D2 | 300 kg, 15/15/40 Hz | 1.062e-3 → 1.275e-3 → 1.283e-3 | 3.694e-4 → 3.682e-4 → 3.541e-4 |
| D3 | 300 kg, 15/15/40 Hz | 1.461e-3 → 1.601e-3 → 1.626e-3 | 3.052e-4 → 3.177e-4 → 3.002e-4 |
| D4 | 300 kg, 15/15/40 Hz | 3.656e-3 → 3.808e-3 → 3.845e-3 | 3.880e-4 → 3.463e-4 → 3.707e-4 |
| H4 | 300 kg, 15/15/40 Hz | 2.485e-3 → 2.758e-3 → 2.778e-3 | 4.359e-4 → 4.254e-4 → 4.308e-4 |
| H4, its turn c (rad) | | 2.448e-2 → 2.448e-2 → 2.402e-2 | 3.655e-3 → 3.655e-3 → 4.212e-3 |
| Linear delta | 300 kg, 15/15/40 Hz | 6.093e-4 → 6.179e-4 → 5.886e-4 | 7.418e-5 → 9.054e-5 → 6.663e-5 |
| Parallel SCARA | 300 kg, 15/15/40 Hz | 1.021e-3 → 1.231e-3 → 1.358e-3 | 3.030e-4 → 5.850e-4 → 3.127e-4 |
| T-gantry | 150 kg, 20/20/50 Hz | 1.995e-4 → 2.729e-4 → 2.346e-4 | 4.76e-5 → 4.92e-5 → 5.43e-5 |
| Single axis | 100 kg, 25 Hz along x | 2.011e-4 → 2.115e-4 → 2.078e-4 | 2.076e-5 → 2.534e-5 → 2.542e-5 |
| Screw axis | 100 kg, 25 Hz along x | 8.31e-5 → 1.805e-4 → 1.859e-4 | 4.51e-5 → 1.181e-4 → 1.040e-4 |
| Rack axis | 100 kg, 25 Hz along x | 1.182e-4 → 4.008e-4 → 3.640e-4 | 6.17e-5 → 1.633e-4 → 9.30e-5 |
| Linear motor axis | 100 kg, 25 Hz along x | 4.351e-5 → 2.469e-4 → 2.656e-4 | 2.453e-5 → 1.260e-4 → 1.362e-4 |
| Rotary table (rad) | 20 kg m², 30 Hz twist | 2.584e-3 → 2.754e-3 → 2.796e-3 | 3.40e-5 → 9.17e-5 → 4.91e-5 |
| SCARA (rad, joints) | 30 kg m² tilts, 5 twist | 1.776e-3 → 1.776e-3 → 1.834e-3 | 2.699e-4 → 2.699e-4 → 2.717e-4 |
| Six-axis arm (rad, joints) | 60 kg m² tilts, 10 twist | 4.453e-3 → 4.453e-3 → refused | 1.968e-3 → 1.968e-3 → refused |
| Palletiser (m) | as the six-axis arm | 6.173e-3 → 7.364e-3 → refused | 2.309e-3 → 2.809e-3 → refused |
| Slider-crank, rotary knife, flying saw, disc cam | 20 kg m², 30 Hz twist about z | unchanged (4.444e-3 rad, 12.31e-3 rad, 3.815e-4 m, 3.189e-4 rad off; on not re-run 17 November) | unchanged |
| Hexapod | 150 kg, 20/20/50 Hz | 2.722e-5 → 1.480e-4 → 1.492e-4 | 1.566e-5 → 4.127e-5 → 4.65e-5 |
| Planar cable robot | 20 kg m², 30 Hz twist about z | 7.140e-4, unchanged | 1.576e-4, unchanged |
| Spatial cable robot, its turn (rad) | as the planar | 2.149e-3 → 2.156e-3 → 2.154e-3 (place 3.569e-4 m unchanged) | 8.281e-4 → 8.295e-4 → 8.309e-4 |

The deltas' dwell error is their own settling, which the base's 0.1 mm sway shifts in phase; the T-gantry's tracking error grows by 37 % one-way, its traverse pushing the stand along x, and by 18 % two-way, where the drives hold the carriage against the stand's sway, but its dwell error is 14 % higher. The axes on their bed (25 Hz) feel it most: one-way the bed's sway is all in the tool's error, two-way the rack axis's dwell error is 43 % lower, as its drive holds the carriage still over the moving bed. Where the tool is the joints (SCARA, six-axis arm) the one-way base moves nothing (`base_shift`) and only the two-way base shows, in the motors. Two-way refused the six-axis arm and the palletiser: their pedestal's 60 kg m² about x is less than what the arm moves on it there (64.6 kg m²; the palletiser 110 and 185 kg m² about x and y), so `base.inertia_*_kgm2` of those configs has to be raised before they can stand on a two-way base. The crank and cam machines' base turns about z only, and their motion is in the x-z plane: it does not move. The hexapod's two-way cycle (elastic legs), stopped after 20 minutes on 15 November, ran on 17 November.

## Motion controller

Source: [`control.rs`](../crates/sim-components/src/control.rs).

The controller plans in tool coordinates and runs one drive per motor. Each profile cycle:

1. The motion under way gives position, velocity and acceleration setpoints per tool axis: a point-to-point move, the cycle, a trajectory, or an interpolated external setpoint.
2. The [input shaper](#input-shaper) spreads them over time, if on.
3. The inverse transformation gives the motor position setpoints. Motor speed and acceleration setpoints are the inverse transformation along the plan a moment ahead and behind (first and second central difference, see [fine interpolation](#fine-interpolation-and-precontrol)).
4. The machine's rigid model gives the torque feedforward for the setpoint pose, speed and acceleration, or each motor's constant inertia times its acceleration (`drive.torque_feedforward_model`).
5. With `effects.fieldbus_delay` the drives get what was computed `profile.fieldbus_delay_cycles` cycles ago. With `effects.fieldbus_jitter` each frame takes j more cycles, j drawn evenly from 0 to `profile.fieldbus_jitter_cycles` per frame (pseudo-random from `drive.noise_seed`, a stream of its own, so every run is the same), and the drives use the newest frame that has arrived: a late frame leaves them the one before, which they carry on from by its speed and acceleration (the time since the setpoint runs on, so the extrapolated setpoint does not stand still for a cycle and then jump two), and frames never arrive out of order. With N cycles of jitter the frame in use is older than the delay by more than i cycles with probability Π_{k=0..i} (N − k) / (N + 1): N = 1 gives 0 or 1 cycle, half each.

The setpoint signals and the path error of the analysis are the controller's, before the fieldbus delay. The signals `axis.<id>.position_setpoint`, `.velocity_setpoint` and `.acceleration_setpoint` are those the last profile cycle set, held until the next; `setpoint_age` is how long ago that was at the sample (0 to one profile cycle).

### Tool position error

The tool position error is the plan at the moment of the sample less where the tool is, the length over the axes in m (the axes in rad apart, in rad): per axis p + (v + a t / 2) t − x, with p, v, a the held setpoints above, t = `setpoint_age` and x = `axis.<id>.position` (the tool as the machine has it: belt stretch, sag and frame deflection included, unlike the drives' encoder-side `following_error`). The plan is the controller's, after shaping and before the fieldbus delay, so a fieldbus delay is part of the error. It is the analysis's "Tool position error, peak" (`tracking_error_peak_m`, `Controller::plan_now` at every sample of the pass), the desktop app's run figure, path colouring and Views tab chart (`charts::axis_errors`, from the run's columns; a run file without `setpoint_age` falls back to the held setpoint) and the report's chart. Not against the setpoint held for the cycle: that counts the tool off by up to a cycle's way at speed, 0.79 mm at 1 m/s on the T-gantry's 1 ms profile cycle where the error is 0.18 mm. Test: `the_tool_error_is_one_number_in_the_run_the_chart_and_the_analysis` (sim-app).

| Command | Behaviour |
| --- | --- |
| Move | Synchronised point-to-point move. Refused while busy or outside the strokes |
| Cycle | Runs the configured segments in order, repeating. On arrival the payload switches to the segment's state and the dwell runs. With `effects.gripper_time` a segment that changes the gripper's state dwells at least the gripper's close or open time (in whole profile cycles), and the payload comes or goes when the gripper is done, not on arrival (see "Gripper" below). With input shaping, a segment with a dwell or a gripper action has arrived when the shaped setpoint has |
| Stop | Finishes the segment under way, then stays. There is no fast stop ramp |
| Setpoint (PLC) | Linear interpolation from the present setpoint to the new one over `profile.setpoint_interval_s`, counted from the profile cycle in which the setpoint came, so that a stream of one setpoint per interval is followed without a pause. The speed setpoint is that of the interpolation, up to and including the cycle of arrival; the acceleration setpoint is 0. A setpoint further from the last one than the axis travels in one interval at its speed limit is refused |
| Trajectory | Poses at times. A cubic spline through the points with a continuous acceleration (C2, `profile.trajectory_spline = "c2"`), at rest at both ends: the speeds at the points solve m_{i−1}/h + 2 m_i (1/h + 1/h′) + m_{i+1}/h′ = 3 ((p_i − p_{i−1})/h² + (p_{i+1} − p_i)/h′²), h and h′ the spacings before and after point i (a tridiagonal system). With `"hermite"` as before 14 November 2026: the speed at each point the slope between its neighbours (of five where evenly spaced), the acceleration stepping at each point. Poses up to 0.1 mm beyond the travel range are taken: the setpoint of a machine at rest is where its motors stand, which at an end of a stroke is some hundredths of a millimetre beyond it. A planned path's poses that rounding puts beyond the range (by less than a nanometre) are set onto its end |
| Disable / Enable | Drives off: the motion under way is dropped. Every other command is refused until Enable |

**Gripper** (`effects.gripper_time`, `[gripper]`, `crates/sim-components/src/gripper.rs`). Jaws (`kind = "mechanical"`) close in `close_s` and open in `open_s`. A vacuum gripper (`kind = "vacuum"`): the generator of suction flow Q evacuates the cup of volume V as a first-order lag towards its vacuum p_v, Δp(t) = p_v (1 − e^(−tQ/V)), and the grip is made when the vacuum switch sees p_hold: t_grip = V/Q · ln(p_v / (p_v − p_hold)). Vented with Q_vent it lets go at p_release: t_release = V/Q_vent · ln(p_v / p_release). The cup of area A holds A · p_hold along its axis. While the cycle stands at the pick or place for that time the settling verdict judges it as before (the second half of the dwell, which now lasts at least the gripper's time).

The verdict `gripper-hold` (vacuum gripper, with `ratings.gripper_safety_factor` S): the cup sits above the part, its axis vertical. The held part of mass m (the payload) at the tool's acceleration a pulls off the cup with m (a_z + g) and pushes sideways with m |a_xy|, which the cup holds by friction μ (`gripper.friction_coefficient`) on what the pull leaves of its force; it slips when m |a_xy| > μ (A p_hold − m (a_z + g)). The verdict judges the most the part asks over the measured pass, m (a_z + g) + m |a_xy| / μ, against A p_hold / S. The acceleration is the tool's along its axes x, y, z (in m), from its speed sampled a millisecond apart while the gripper holds and is not acting; a machine without such axes is not judged. The optimiser judges it as any other verdict. Tests: `the_vacuum_gripper_takes_the_hand_calculated_times_and_holds_area_times_pressure`, `the_part_asks_the_hold_exactly_at_the_hand_calculated_slip_acceleration` (gripper.rs); `the_gripper_time_lengthens_the_dwells_and_moves_the_payload_switch_by_hand`, `the_gripper_hold_judges_the_held_part_against_the_cup_and_the_optimiser_keeps_to_it` (single axis).

The controller's setpoint starts where the motors stand, a few micrometres from the configured start pose (belt stretch and gear twist under load), so the first step has no jolt.

The following error per tool axis (signal `axis.<id>.following_error`) is the motor following errors transformed with the forward transformation. It is an encoder-side figure and does not see belt stretch or the leaning tool, unless the position loops close on load-side scales (`effects.load_encoder`).

**Check of axis limits against motor limits.** When a config is applied, for every combination of axis directions at the middle of the workspace: the motor speed at the axis speed limits against `drive.rated_speed_rad_s`, and the feedforward torque at the axis acceleration limits against `drive.peak_torque_nm`. A diagonal move loads one motor with both axes. Violations are reported, not enforced.

## Profile generator

Source: [`profile.rs`](../crates/sim-components/src/profile.rs).

A jerk-limited seven-segment S-curve from rest to rest: jerk up, constant acceleration, jerk down, cruise, and the mirror image. With limits v, a, j:

```text
t_j = a / j,  t_a = v/a − a/j         if v · j ≥ a²        (acceleration limit reached)
t_j = sqrt(v / j),  t_a = 0           otherwise
```

If the distance is too short to reach the cruise speed, the peak speed is lowered: the constant-acceleration phase is shortened, and for very short moves dropped (t_j = cbrt(d / 2j)).

A move of several axes is one profile that runs a straight line from 0 to 1. Its limits are the tightest of all moving axes (limit / |distance|), so all axes start and finish together and none exceeds its own limits.

**Shapes** (`profile.shape`, or `shape` of one `[[cycle.segment]]`):

| Shape | Law | Peaks over a move of h in T |
|---|---|---|
| `s-curve` | seven segments (above) | the limits |
| `sin2` | speed v·sin²(π t / (2 T_r)) over each ramp of T_r, cruise between (Bestehorn) | a = (π/2) v / T_r at the ramp's middle, jerk (π²/2) v / T_r² at its ends; T_r = max(π v / (2a), π sqrt(v / (2j))) |
| `poly345` | s = h (10u³ − 15u⁴ + 6u⁵), u = t / T (VDI 2143), no cruise | v 1.875 h/T, a 5.7735 h/T², jerk 60 h/T³ (at the ends) |
| `poly4567` | s = h (35u⁴ − 84u⁵ + 70u⁶ − 20u⁷), no cruise | v 2.1875 h/T, a 7.5132 h/T², jerk 52.5 h/T³ (zero at the ends) |

A polynomial law takes the shortest T that keeps all three peaks: T = max(c_v h/v, sqrt(c_a h/a), cbrt(c_j h/j)).

**Braking of its own**: with `profile.deceleration_ratio` below 1 the braking ramp has that share of the acceleration limits; the S-curve and sin² then speed up on one ramp and brake on another, each the shortest within its limit, and the cruise speed is the limit where both ramps fit the distance, else the speed at which they cover it alone (bisection: they cover v (T_up(v) + T_down(v)) / 2). A polynomial law keeps the lower of the two. With the ratio 1 the S-curve is the closed form above.

**Gravity** (`profile.vertical_axis`, `up_acceleration_ratio`): the axis that lifts has that share of its acceleration limit for an acceleration upwards. Speeding up, each axis accelerates towards its target, braking away from it; so a move up speeds up within the reduced limit and brakes (accelerating downwards) within the full one, a move down the other way round.

**Time scaling** (`Profile::stretched`): a profile stretched to k times its shortest duration has the same shape with the limits v/k, a/k², j/k³, for a planner that synchronises moves to one duration.

## Programs: G-code and robot programs

Source: [`program.rs`](../crates/sim-components/src/program.rs). `sim-cli run|analyze --path FILE` and the app's path field read a file by its extension into path pieces (lines and arcs, `path::Piece`) for the path planner below: `.nc`, `.ngc`, `.gcode`, `.g`, `.cnc`, `.tap` as G-code, `.src` as KUKA KRL, `.mod` and `.prg` as ABB RAPID, a file with lines starting `MoveL`, `MoveJ`, `MoveC` as robot moves in CSV, and anything else as a path file. The subsets, and what is refused with its line, are listed in the [user guide](user-guide.md#programs-g-code-and-robot-programs).

- **Arcs**: G2/G3 in the plane of G17 (X, Y), G18 (Z, X) or G19 (Y, Z), clockwise as seen from the plus side of the normal (Z, Y, X). The centre is the start plus I, J, K, or from R: on the chord's perpendicular bisector at sqrt(R² − d²/4) from its middle, to the right of the chord for G2 with R > 0 (the shorter arc), to the left with R < 0. Start and end must lie within 1e-4 of the radius (and 1 µm) of the centre. The direction is turned into the planner's (`cw` seen with the first axis right and the second up on two axes; from +z, -y or +x on three) by the orientation of the plane's axes among the machine's. The plane's normal goes with the arc (`Piece::Arc::plane`, from G17/G18/G19 by position, from the three points of a MoveC or CIRC): on three axes or more a half turn, whose end lies opposite its start, is in that plane, where start, end and centre alone leave it open; a piece without a plane takes the first of the x-y, x-z and y-z planes that holds the start's radius.
- **MoveC and CIRC**: the circle through start, via and end, its centre s + ((|u|² w − |w|² u) × n) / (2 |n|²) with u = via − s, w = end − s, n = u × w; the direction by n against the planner's viewing side.
- **Feeds**: each move has the speed along the path it is capped at (G1/G2/G3 with F per minute, in length units, in degrees for a block that moves only rotary axes, and for a block of both the linear axes' distance at F sets the time and the rotary axes follow in it, so `G1 C90 F3600` takes 1.5 s and `G1 X10 C90 F600` 1 s; G2/G3 with I, J, K and no end is a full circle, planned as three arcs of 120 degrees; RAPID's speed, `v100` in mm/s or a speeddata's TCP speed; KRL's `$VEL.CP` in m/s for LIN and CIRC); the reading of F is Siemens SINUMERIK's with its default FGROUP (the geometry axes; Fanuc's counting of degrees like mm in a mixed block is not taken, no primary source). A run is planned within the axis limits with each piece's speed along the path capped at its own feed (`path::plan_feeds`: each piece's speed limit, and the speed its transition curves are sized for, no higher than its feed). In G-code F is modal and the moves between two G0 are one run, so a feed that changes between blocks changes the cap with no stop (`F600 G1 X10`, `F1200 G1 X20`, `F600 G1 X30` run 10 mm at 10 mm/s, 10 mm at 20 mm/s, 10 mm at 10 mm/s; the junctions are passed at the lower feed, and no corner is rounded or transition curve laid across the change); a corner stops the path as it does without a feed. A KRL or RAPID run follows the speed it was read with and the runs follow one another with a stop between; G0, `vmax`, PTP and robot CSV run at the axes' limits, as fast as the motors allow with `--time-optimal`. Before 14 November the feed capped each axis, and a diagonal ran up to sqrt 2 faster. The speed along the path counts all axes, as the path's length does (shortcut: on an arm in tool space the rotation vector's radians count with the metres, where a robot's TCP speed counts the position alone).
- **Orientations**: on a machine with the axes rx, ry, rz (an arm in tool space) RAPID's quaternion [q1, q2, q3, q4] (q1 the scalar part, normalised) and KRL's A, B, C (R = Rz(A) Ry(B) Rx(C), components left out kept) are turned into the rotation vector those axes take (`rigid::from_quaternion`, `rigid::log`).

## Corner blending

`effects.corner_blending`: the next cycle segment starts while the current one still has its braking ahead, and the remaining distance of the old move is added on top of the new one. The lead is the smallest of `profile.blend_time_s`, the braking time of the current move and the ramp time of the next. It happens only where the machine need not stop: the segment has no dwell and no gripper action, no axis reverses, and no other blend is under way.

Because both moves are on their ramps, the speeds add up to no more than either cruise speed on lift-traverse-lower cycles. For other shapes of cycle the sum of two segments on one axis is not checked against the limits.

## Path planners

Source: [`path.rs`](../crates/sim-components/src/path.rs). Both planners turn lines, arcs and splines into poses at times, one per profile cycle, which then run as a trajectory. A *corner* is a junction where the direction turns by more than 1 mrad; both planners stop there, unless `profile.corner_tolerance_m` rounds it (below).

**Corner rounding (`profile.corner_tolerance_m`, `round_corners`).** A corner between two lines with directions t_a and t_b is replaced by a curve from d before the corner to d after it, a polynomial in u from 0 to 2d:

```text
p(u) = corner − d t_a + t_a u + (t_b − t_a) (u³ / (4 d²) − u⁴ / (16 d³))
```

It leaves and arrives in the lines' directions at a speed by u of 1 and with no second derivative at either end, so the path has no step in position, direction or curvature (G2), and no axis's acceleration steps. Its middle is 3 d / 16 · (t_b − t_a) from the corner, 3 d sin θ / 16 off each line for a turn θ; there it turns at a curvature of 3 |t_b − t_a| / (d |t_a + t_b|²), for a right angle 3√2 / (2d), a radius of √2 d / 3. d is chosen so that the curve's largest distance from the two lines (measured at 65 places, it scales with d) is the tolerance: d = 16/3 of it at a right angle, where the tightest radius is 2.51 times the tolerance (a circular fillet of the same deviation would have 3.41 times: the curve pays for G2 with a tighter middle). d is no more than half of either line, so two rounded corners never overlap. The curve is a spline span for the planners. A turn of nearly half a circle (the curve would nearly stop) and a corner on more than six axes stay stops.

A corner next to an arc or a spline (`round_corner`) is rounded by the quintic Hermite curve in u from 0 to h = 2d that leaves the piece before d back from the corner with its direction t₀ and curvature vector k₀ and arrives d on along the piece after with t₁ and k₁, at a speed by u of 1 at both ends (in τ = u / h):

```text
p = p₀ + h t₀ τ + h² k₀ τ² / 2 + (10 (p₁ − p₀) − h (6 t₀ + 4 t₁) − h² (3 k₀ − k₁) / 2) τ³
    + (15 (p₀ − p₁) + h (8 t₀ + 7 t₁) + h² (3 k₀ / 2 − k₁)) τ⁴ + (6 (p₁ − p₀) − 3 h (t₀ + t₁) − h² (k₀ − k₁) / 2) τ⁵
```

so the path keeps its place, direction and curvature at both ends (G2); between two lines (k = 0) it is the curve above. Its deviation is measured at 65 places against the parts of the two pieces it replaces (as broken lines of 64 parts each), and d is scaled by the tolerance over the deviation until the deviation is within 2 % below the tolerance (at most 30 times, at most half of either piece). A line into an arc of 1 m radius at a right angle with 0.1 mm starts the curve 0.533 mm before the corner, as between two lines, the arc's bend over d being d² / (2R) = 0.14 µm; the curve ends with the arc's curvature of 1 / m (`a_line_arc_corner_is_rounded_within_the_corner_tolerance`). What remains of the arc is the same arc from the curve's end on; of a spline span the polynomial from that parameter on. Rounding is not always quicker than stopping: the curve has to be passed at a speed whose centripetal acceleration and its change keep within the limits, which on the T-gantry at 100 m/s³ is slower than the stop and the start (the square path `line,0.5,0.35` `line,0.5,0.05` from (0.1, 0.35), `sim-cli analyze t-gantry --path FILE --time-optimal [--set profile.corner_tolerance_m=0.02]`, `cycle_s` with its 0.3 s of rest: 1.262 s time-optimal stopped, 1.365 s rounded within 20 mm, 17 November 2026; 1.306 s stopped before). At 2000 m/s³ (`--set axis.x.max_jerk_m_s3=2000 --set axis.z.max_jerk_m_s3=2000`) rounding was quicker on 15 November (0.978 s against 0.920 s) and is no longer: 0.869 s stopped against 0.888 s rounded. Test: `a_right_angled_corner_is_rounded_within_the_tolerance_at_the_hand_calculated_radius`.

**More than three axes.** A line runs on any number of axes. A spline runs on two to six: the cubic spline through all positions of its via points, its parameter the straight distance over all axes. An arc on three axes or more (`arc,` the end with a position per axis, the centre with three) lies on the first three as on the delta, and the other axes (a SCARA's or the D4's turn, an arm's orientation in tool space) go evenly with the distance along the arc (`Geo::Carried`): the piece counts the distance over all axes, sqrt(L² + |Δ|²) for the arc's L and the change Δ of the other axes, so that its direction has length 1 as on a line. Test: `a_four_axis_arc_keeps_its_fourth_axis_linear_in_the_arc_length`. An arm's orientation goes along as its rotation vector, evenly per component, which turns the tool at an even rate only about a fixed axis (shortcut in `arc_on`: take the arm's quaternion slerp there). Transition curves and eased splines stay two- and three-axis things.

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

Each motor has its own drive and motor (`[drive.<id>]`, `[motor.<id>]`, `[parts.<id>]`; before 14 November every motor had the shared `[drive]` here). T_m is what the curve gives in the direction the motor turns and T_b as it brakes, at the speed it turns: level up to the rated speed, then falling (linearly to the no-load speed, or as the d-q model has it with field weakening). The plan keeps back (1 − reserve) times the torque at a standstill at every speed, for friction and elasticity, which the rigid model leaves out and which do not shrink with the speed: up to the rated speed that is the reserve times the curve, as before; above it the room falls faster than the curve, and where the curve has less than that left the rigid torque has to stay as far on the braking side. Within a place the bound is taken at the highest speed of the range, as the curve does not rise (`MotorCurve`, read at 257 speeds up to the maximum and taken at the next one up). **Friction** comes off the motoring side of T at each speed: the shaft's steady friction and the gearbox's drag, c tanh(ω / ω_s) + b ω + T₀ (ω / n₀)^(2/3) (`PlanLimits::friction`, from each motor's own config as `mech::ShaftFriction` reads it, without the breakaway's Stribeck rise), and per place the guides' friction as torque at the motors where the machine gives it (`Machine::rigid_friction`: machine files, each guide's Coulomb and viscous force on its axis through the map of the rigid forces, behind a lossy gearbox over its efficiency), c + b v at the speed v along the path, taken at its largest over the speeds up to the place's (none at a standstill) off the room on its side. As the motor brakes, friction would help; that is not counted. Before 14 November only the reserve stood for friction: the XYZ portal's Z motor held to 0.2 N m (0.04 N m kept back) has 0.02 + 1e-4 · 400 = 0.06 N m of shaft friction at 400 rad/s and 0.02 N m of the Z guide's, and its run asked for more than its peak; now the path down and across (0.1, 0.1, 0.02), (0.3, 0.2, 0.2) takes 1.064 s instead of 0.886 s without friction, and at its fastest upwards (249 rad/s) the holding torque 0.0686 N m and the friction fit under the curve less the reserve (`a_motor_whose_friction_exceeds_its_reserve_is_planned_slower_within_its_peak`). Before 14 November the plan stopped every motor at its rated speed with its full torque: no plan used the curve above it. The single axis now cruises a 1 m move at up to 398.8 rad/s, 420 − 106 · 0.76 / 3.8 (`the_time_optimal_plan_turns_a_motor_above_its_rated_speed_along_its_curve`). With `effects.drive_current_limit` the drive holds the current to `ratings.drive_max_current_a`, so T is also no more than the reserve times the torque of that current (`DriveParams::torque_of`, with saturation where it is on): a single axis whose drive gives 2 A to a 0.45 N m/A motor accelerates at (0.8 · 0.9 N m · i / r ∓ F) / m up and down a 30° incline (`the_drive_current_limits_the_time_optimal_acceleration_to_the_hand_calculated_one`: 26.186 and 32.590 m/s², 0.68649 s at the top speed less 1 %, below). Gravity counts for each direction as it is (before 13 November | e | was taken off both). The plan keeps 0.02 % of T back for what falls between two places. A place's **top speed** is the highest at which it holds all of its bounds without acceleration.

**Gearbox and pulley.** Where the machine reports a gearbox's output torque (`gearbox.<motor>.output_torque`) and the config rates it, `ratings.gearbox_max_torque_nm` (M2max) or `pulley_max_torque_nm` (the pulley sits on the output; the lower of the two counts), the output torque is a row too: the ratio i times what the motor gives less what turns the inertia at the gearbox's input (rotor and gearbox, J_in), with the motor's angular acceleration w a + w′ v² (w′ by second differences of the inverse kinematics along the path):

```text
−M  ≤  i ((c − J_in w) a + (d − J_in w′) v² + e)  ≤  M,      M = profile.torque_reserve · min(M2max, pulley rating)
```

The reserve is the motor's: the rigid model leaves out the output's friction and the elasticity's swing there as well. The output torque is what the load takes, whatever the gearbox loses. What it loses the motor gives on top: with the efficiency η (`gearbox.efficiency`, with `effects.gearbox_efficiency`) the motor gives J_in (w a + w′ v²) + (τ − J_in (w a + w′ v²)) / η while it drives the load (the load's torque turns the way the motor turns) and J_in (w a + w′ v²) + η (τ − J_in (w a + w′ v²)) while the load drives it (it brakes the load). For a motor that turns forwards along the path that is the larger of the two forms, for one that turns backwards the smaller, whatever the sign of the load's torque and of the rotor's own J_in α: so the η form holds against both bounds, and the 1 / η form against the high bound where the motor turns forwards and the low bound where it turns backwards (`Place::against`). Friction, which helps braking, is not counted: it is nil at a standstill, and the bound holds at every speed. With the 1 / η form alone against the high bound, a rotor whose inertia outweighs a load that pushes on would have been planned over its torque (0.4227 N m on a 0.4 N m bound in the unit test). On the single axis with η = 0.9 accelerating takes 0.0075 + 0.017 / 0.9 = 0.026389 N m per m/s² against 0.0245 lossless: 15.158 m/s² instead of 16.327, and braking 0.0075 + 0.017 · 0.9 = 0.0228 N m per m/s²: 17.544 m/s² (`a_lossy_gearbox_lowers_the_time_optimal_acceleration_by_its_loss`). A gearbox enters the plan where it is rated or lossy. Where it binds, a single axis accelerates at 0.8 · M / 0.085 N m per m/s² (`a_gearbox_or_pulley_rating_that_binds_limits_the_time_optimal_acceleration`: 94.118 m/s² at M2max 10 N m, 75.294 m/s² with the pulley rated 8 N m). On the shipped delta it binds (0.8 · 192 N m at the output against the motors' 0.8 · 15 N m · 20): its line across the range took 0.434 s instead of 0.412 s with the gearbox rated 192 N m (`--set ratings.gearbox_max_torque_nm=192`; the shipped SP+ 100 is rated 278 N m since, and its line takes 0.412 s), and each output torque stays within 0.8 · M2max (0.1 %). A holding torque beyond M is refused (`error-path-torque`).

**Guides.** Where the machine gives its guides' loads in the rigid model (`Machine::rigid_guide_loads`: a load at rest plus the size of a part linear in the tool's acceleration; the T-gantry, the single axis, the linear delta's carriages (the part of the rods' pull and the carriage's weight at right angles to its column, at rest and from what accelerating the plate takes, m a, through the same rods), the linear motor axis (attraction and weight, nothing from the acceleration along the rail) and machine files (weight and press, and an overhang's tilt and side load, m a (2 |lever| / spacing (+ 1)))) and the config rates them (`ratings.guide_max_load_n`), each guide is a row with the same room both ways: | g · (a p′ + v² p″) | ≤ R L − rest, R = `profile.torque_reserve`, as the motors keep R of their torque: the rest is for what the rigid model leaves out (vibration, friction). Before 14 November the guides used their whole rating. On the T-gantry the X guide carries m_X g + | 2 m_Z a_x (z_com − z) / s | and the Z guide | m_Z a_x | (1 + | 2 (z_com − z) / s |); rated 150 N, a line along X at z = 0.35 m accelerates at (150 − 107.87) / 5.833 = 7.222 m/s² (`the_guide_load_rating_limits_the_time_optimal_acceleration`). A load at rest beyond the rating is refused (`error-path-guide`). No reserve: what friction adds to the acceleration the motors command (`commanded_ax`) is not in it.

**The rest of what the verdicts judge (since 15 November 2026).** The plan reads every rating whose verdict depends on the motion at a point of the path, so that a time-optimal plan, run, passes its own checks:

- **The drive's maximum current**, `ratings.drive_max_current_a` ("drive-peak-current"), whether or not `effects.drive_current_limit` holds the drive to it (before, the plan read it only with the effect on, while the verdict judges it always): T is no more than the torque of that current.
- **The gearbox's input speed**, `ratings.gearbox_max_input_speed_rpm` ("gearbox-input-speed"): the motor's maximum speed is the lower of its own and that.
- **Speed ratings keep 1 % back** (`SPEED_KEPT`) for the loops' following: a plan at the motors' 420 rad/s ran the T-gantry's up to 420.66 rad/s, and "motor-speed" failed by 0.16 %. So the top speed of the single axis is 0.99 · 314 · 0.02 / 5 = 1.2434 m/s where it was 1.256 m/s.
- **The braking power**, `ratings.braking_resistor_peak_w` where the config rates the DC link (`dc_link_capacitance_uf`; "braking-resistor-peak"): as a motor brakes at the speed ω its torque is held to P / |ω| as well, the mechanical power, which while braking exceeds the electrical by the losses, so the bound is on the safe side; a line-up that brakes through one Braking Module (`drive.braking_module`) shares P out evenly over its motors. Up to the speed P / (m a_max) the motor's torque binds, above it the power: the single axis held to 50 W brakes at no more than 50 / (6.125 kg · v), 6.565 m/s² from 1.2434 m/s (`the_braking_resistor_bounds_the_time_optimal_deceleration`; the speed levels of the runs make it up to 4 % softer). A braking that the DC link cannot take is not available. The resistor's mean power ("braking-resistor-continuous", over the whole cycle with its dwells) is not a bound at a point of the path and is not in the plan.
- **The machine's own parts** (`Machine::rigid_checks`): for each check of `Machine::rating_checks` the machine gives the value of its signal in the rigid model at a tool pose, speed and acceleration (NaN for a life or a signal the rigid model does not give). The joint machines give their mechanism's signals and each joint's force M q″ + h: a screw's speed and d n ("screw-speed", "screw-dn"), its axial force ("screw-buckling", "screw-static"), a rack's feed force ("rack-feed-force"). The plan reads each at rest, at the acceleration 1 and at the speeds 1 and 2 along the path, which gives c a + b v + d v² + e, and keeps | c a + b v + d v² + e | within the limit: the reserve's share of it where it grows with the acceleration (as the torques), `SPEED_KEPT` off it where it is a speed alone, and all of it (times the margin) for a limit of the plan itself (a `plan.` signal: the flange's angular speed and acceleration, judged on the planned motion, which the rigid model gives exactly), less, where it grows with the acceleration, what the controller's cubics between the plan's points add: where a ramp of jerk J ends in a plateau the Hermite cubic with the neighbours' slopes overshoots the plateau by up to 2/3 J h (the C2 spline by 0.085 J h), so the plan keeps back 2/3 of the quantity's change per acceleration times the ramps' jerk (and the bends' on a curve) times the cycle; the palletiser's turn uses 9.93 of its 10 rad/s² (2/3 · 100 rad/s³ · 1 ms = 0.067 back; before 17 November the reserve's 8, and with nothing kept back for the cubics its cycle timed time-optimal reached 10.003). The part b v takes its room off its side at its most over the speeds up to the place's, as the guides' friction does. A value at rest beyond the limit is refused (`error-path-rating`). The 1.6 m screw whirls at 1250 r/min: the plan runs it at 0.99 · 1250 / 60 · 0.02 m = 0.4125 m/s (`the_time_optimal_plan_keeps_the_screws_speed_rating`); a rack rated 500 N over the service factor 1.25 pushes 80 kg at 0.8 · 400 / 80 = 4 m/s² (`the_time_optimal_plan_keeps_the_racks_feed_force`). shortcut: a rack's feed force is the force over the efficiency while the pinion drives and times it while the load drives it back; with the load at rest pulling against the motion the three-point fit is not exact there.
- **The joints' friction** (`Machine::rigid_friction` of the joint machines: each joint's Coulomb level and viscous part through its ratio and what a screw or rack passes on), taken off the motor's room as the guides' friction is. Before, the plan of the joint machines left it out: the linear motor axis's friction took some 210 N of its 900 N at 6 m/s, and the run asked 944 N of the motor ("motor-peak" 1.05).
- **A margin** `profile.time_optimal_margin` (1 in the shipped configs; at most 1): the share of every limit the plan may use, on top of the reserves. It multiplies the torque reserve, the belt's share, the speed ratings and the braking power, so 0.9 plans at 90 % of the limits (`the_time_optimal_margin_plans_at_its_share_of_the_limits`).

Not in the plan: what is judged over the whole cycle (the winding's and the parts' warming, "motor-s1", "drive-current", the lives, the mean braking power), the settling at the targets, the cable robots' cable tensions, the parallel SCARA's and the netlist machines' own checks (they give no rigid values yet), and whatever the full model adds to the rigid one beyond the reserves. Found on the way, not solved: a short piece whose speed curve rises into its end faster than a run from the end can follow at the jerk (the four-arm delta's 25 mm descent from its place pose, the curve rising at 195 m/s² against a jerk of 2500 m/s³) has no top speed that fits, as the run from the end is held at rest; the search for its top then looped for ever (the planar delta's time-optimal cycle hung before 15 November; `the_search_for_a_top_speed_ends_where_nothing_above_zero_fits`). Now the piece is refused (`error-path-singular`), and a cycle times that segment within the axis limits. A winding that overheats on the cycle repeated runs away in temperature and its resistance takes the torque the plan counts on (the linear motor axis, the flying saw, the parallel SCARA and the four-arm delta on their time-optimal cycles): there "motor-peak" can fail as a result of "motor-thermal".

Test over the machines (`crates/sim-machines/tests/time_optimal.rs`, one test per machine of `MACHINES`): each machine's cycle, timed with `profile.cycle_timing = "time_optimal"`, run and judged with the shipped config, passes every verdict the plan keeps ("motor-peak", "motor-speed", "drive-peak-current", "gearbox-peak", "pulley-torque", "gearbox-input-speed", "guide-load", "belt-tension", "belt-slack", "braking-resistor-peak" and the machine's own checks with a rigid value); where "motor-thermal" fails, with the winding at its starting temperature (`effects.winding_heating` off).

**Jerk along the path.** On a line the ramps may use the axes' jerk limits in full: j_ramp = min over the axes of J_i / | p′_i |. On a curve they get 0.4 of the least of that over the piece, and the rest of each axis's limit is for the terms of speed and acceleration (the third bound), as `curved` shares it out for the plan within the axis limits.

**The speed curve.** Each piece gets a curve of the highest speed it may follow (`path::smoothed`, as squared speeds x at its places): it starts at the places' top speeds and is only lowered, in sweeps forwards and backwards, until its acceleration x′ / 2 is within 0.9 of what the places allow at its speed (where a slow place would come too fast to brake for, the curve comes down to it in time) and its jerk, v times the change of x′ / 2 per metre, within half of j_ramp (a peak too sharp is lowered onto the bound, a dip too sharp gets its sides lowered). It never falls below the slowest place's top. The rest of the acceleration and of the jerk is for a run to catch up with the curve and follow its bends (`the_speed_curve_comes_down_to_a_slow_place_within_its_acceleration_and_jerk`: from a place at 0.5 m/s among places at 1 m/s, 10 m/s² and 10 mm apart, x rises 0.25, 0.43, 0.61, 0.79, 0.97, 1 m²/s²).

**The runs.** Between the speeds at its junctions a piece runs: two cycles steady where a junction is passed at speed, a run up from its start to its top speed, cruise, a run down to its end (`path::follow`; the run down is planned from the end backwards under the bounds for braking). A run is integrated in steps of a quarter of the profile cycle at a constant jerk per step. Each step the acceleration rises at j_ramp as far as three things allow: the most acceleration the places allow ahead (worked out at 32 speed levels between zero and the curve's highest, each torque row at its worst speed up to the level, the jerk bound at the level; the run takes the level of the highest speed it reaches if its acceleration falls to zero from there, v + a² / (2 j_ramp); lowered ahead of where it falls by j_ramp times the least time between two places, so that the jerk can follow it); that it can still fall to zero just as the speed reaches the top speed ((a + Δa)² ≤ 2 j_ramp (top − v)); and that it can still come down onto the speed curve with the jerk left beside the curve's own (y² ≤ 2 k g for the acceleration y above the curve's and the gap g to it), unless falling to zero at once keeps the speed below the curve all the way. So a run rises along the most acceleration, meets the speed curve, follows it up and down, and goes on to the top speed; until it brakes for the top speed it is the same run whatever the top speed. On a line with constant limits this is the jerk-limited ramp of the plan within the axis limits, to a fraction of a step.

**The top speed** of a piece is the highest, up to the curve's highest, for which both runs reach it before they meet and the cruise between them holds at every place it passes, found by halving to 1e-9 of it. A higher top that fits is the quicker, since the runs are the same until they brake for it.

**Joining on the curve** (`curve_join`). Where the runs would meet while the speed curve still rises or falls, a top with no acceleration makes the plan cruise below where they meet, and a piece that has to slow down below both its end speeds somewhere has no top at all. So each piece is also timed by two runs with no top, each following the curve as far as the piece goes (one that gets more than 0.1 % above the curve started faster than the curve lets it slow down, and is refused), joined at the first place where the run from the start is as fast as the run from the end: there both are on the curve, at its speed and acceleration (to 0.02 % and four steps' jerk), and the plan switches from one to the other directly. The quicker of the two timings is taken. Test: `a_path_without_the_forced_cruise_is_quicker_than_with_it`.

**Junction speed.** A junction is passed no faster than `junction_speed` allows (the step of the axes' rates, as within the axis limits) and than the speed curve of either piece allows at the junction (its last or its first place, from where the curve can still slow down), and no faster than a run into either piece from there can follow its curve; the look-ahead then lowers it until both pieces fit. Before 14 November it was no faster than the slowest place of either piece: two tangent arcs of 0.2 and 0.1 m radius, the second slow (0.05 m/s) only at its far end, are now joined at sqrt(0.5 · 100 · 0.001 / 5) = 0.1 m/s, the junction's own limit, not at 0.05 m/s (`the_junction_of_two_arcs_is_passed_at_the_hand_calculated_speed`). Where the speeds the look-ahead gives leave a piece that does not fit, that piece's two junctions fall back to the rule before (where that is slower) and the look-ahead runs again, as long as pieces are left that do not fit (at most once per piece; then the whole path as before 14 November, which until then was taken at the first such piece).

**Singularities.** Where the inverse kinematics gives no finite angle, or a motor turns more than 1e6 rad per metre along the path, or the plan would take more than 1000 s (a crawl near a singularity, a pose per cycle without bound), the path is refused (`error-path-singular`; `the_time_optimal_plan_refuses_a_crawl_that_would_take_unbounded_poses`).

**Long plans.** A plan within the axis limits, as a program with a feed gets, of more than ten million poses (2.8 hours at a profile cycle of 1 ms) is refused as well (`error-path-long`; `programs_beyond_any_plan_are_refused_in_bounded_time` in sim-machines): a feed of 0.000001 mm/min would otherwise be planned until the memory is full.

**Cycles** (`profile.cycle_timing = "time_optimal"`). Each cycle segment is planned so, rest to rest, from where the segment before ended: the straight way as one line, a segment with via points along its path. A config whose segment cannot be planned so from the segment before is refused (`profile.cycle_timing`); the first segment, from the start pose, falls back to the timing within the axis limits where it is refused. The single axis's segment 0.1 → 0.9 m takes 0.8 / v + v / a + a / j = 0.71970 s with 0.8 · 0.5 N m and v = 1.2434 m/s (0.71404 s before 15 November, at 1.256 m/s) (`a_time_optimal_cycle_segment_takes_the_hand_calculated_time`).

So every planned pose keeps each motor's torque from the rigid model within its curve less what the reserve keeps back, each motor's speed within its maximum speed, each rated gearbox, pulley and guide within its rating, and each axis's jerk within its limit. The tests check that on the planned poses by differences over one cycle, with a margin of 0.1 %. Planning a path takes 2 to 21 ms (release build; see the table in docs/development/PROGRESS.md), against 0.2 to 1 ms with one limit per piece.

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

Assumptions and limits: the rigid model, so no friction and no elasticity (that is what the margin and the share are for); the velocity and acceleration limits of the axes play no part; the plan is not proven the fastest one possible: the two runs of a piece meet on a cruise at no acceleration, so where they meet while the speed curve still rises or falls the top is a little below where they would meet; the speed curve keeps 0.1 of the acceleration and half of the jerk for catching up with it, and its sweeps stop at a change of 1e-7 of it (at most 300); a run's acceleration bound is taken at the worst speed up to its speed level (32 levels); on a curve the ramps get the share of the jerk its bends leave at the piece's own speed and acceleration ceilings, at least 0.4 (a slight curve keeps nearly all of it), and the jerk's three terms are bounded each by its size; the bounds are read at places 2 mm apart and taken as straight between them, with 0.02 % kept back; without a path tolerance it crawls into an arc as the plan within the axis limits does; what is judged over a whole cycle is not in the plan (the belt's tension, the drive's current, the gearbox's and pulley's torque and input speed, the guides' load, the braking power, the joints' friction and the machine's own parts are, above); the payload is as the machine has it when planning. Until 14 November 2026 a piece cruised at one speed between two ramps (where its speed limit rose and fell between them, at the lowest of it), passed a place slower than its top at no more than 0.9 of that place's speed, took the quickest of 16 + 16 top speeds, read each ramp's bounds at its worst speed up to the top, and took the lowest of three neighbouring places; the drive's current, the gearbox, the pulley and the guides were judged after planning only. The user guide's spline example then took 1.380 s (1.340 s on 17 November 2026, 1.349 s on 16 November), the T-gantry line at 300 m/s³ and a 400 N belt 0.851693 s (0.851565 s on 14 November, 0.857 s since the belt's own mass), the delta's line across its range 0.411 s, its motors' 0.8 · 15 N m through the ratio 20 allowed up to 240 N m on gearboxes rated 192 N m M2max (0.434 s at that rating; 0.412 s with the shipped SP+ 100's 278 N m). Until 13 November 2026 the plan had one speed and one acceleration limit per piece, from nine poses, shared out 3 : 4 : 5 on arcs: the line, arc, line of the user guide took 1.429 s (1.397 s now), its spline example 1.674 s (1.381 s now), a line across the delta's range 0.429 s (0.415 s now). Until 12 November 2026 this was the classic time-optimal parameterisation on a grid of points, without a jerk limit and smoothed by a moving mean (`profile.path_smoothing_s`, which no longer exists). It was faster (0.69 s against 1.43 s on the line, arc, line of the user guide) and passed the motors' limits on arcs, many times over on short ones.

## Input shaper

Source: `shape` in [`control.rs`](../crates/sim-components/src/control.rs).

Source: the impulses in [`shaper.rs`](../crates/sim-components/src/shaper.rs).

`effects.input_shaping`: a shaper on the position, velocity and acceleration setpoints. The shaped setpoint is a weighted sum of copies of the plan at multiples of half a damped period (`profile.shaper_type`, ZVD by default: three copies, now, half a period ago and a whole period ago). With `profile.shaper2_frequency_hz` above 0 a second shaper of the same type (its damping `shaper2_damping`) is convolved with the first: copies at the sums of the two shapers' delays with the products of their weights; every entry of `profile.shaper_modes` adds one more. The first and the second act on the axes of `profile.shaper_axes` (all where it is empty), an entry of `shaper_modes` on its own `axes` (all where it has none): each axis convolves the modes that act on it, so axes can have shapers of their own frequency and damping. For an undamped mode at f a ZVD shaper at f_s with no damping leaves |H| = cos²(π f / (2 f_s)) of the vibration, two convolved the product of theirs.

| Type | Undamped weights at 0, T/2, T, 3T/2 | Left at the design frequency, at ±20 % |
|---|---|---|
| `zv` | 1 : 1 | 0, cos(0.4 π) = 0.309 |
| `zvd` | 1 : 2 : 1 | 0, 0.095 |
| `zvdd` | 1 : 3 : 3 : 1 | 0, 0.030 |
| `ei` | (1+V)/4 : (1−V)/2 : (1+V)/4 | V, \|0.475 + 0.525 cos(0.8 π)\| = 0.0503 at V = 5 % |
| `ei2` (two-hump EI) | A : 1/2 − A : 1/2 − A : A, A = (3X² + 2X + 3V²)/(16X), X = (V² (sqrt(1 − V²) + 1))^(1/3) | 0, humps of V either side |

V is `profile.shaper_ei_tolerance`. Damped, weight i is multiplied by K^i (and all divided by their sum): that cancels the decay between the copies. For ZV, ZVD and ZVDD that is the textbook damped shaper, which leaves nothing at the design frequency; an EI shaper of undamped weights a_i then leaves V K^n / Σ a_i K^i there (n its last copy), less than V: 0.0424 for V = 0.05 at ζ = 0.05. The residual vibration of a shaper is checked against Singer and Seering's e^(−ζωt_n) sqrt(C² + S²), C and S the sums of A_i e^(ζωt_i) cos and sin(ω_d t_i). Shortcut: the damped EI shapers are these weighted ones, not Singhose's fitted damped EI, whose insensitivity is a little wider at dampings above 0.1.

```text
K       = exp( −ζ · π / sqrt(1 − ζ²) )
T_half  = 1 / ( 2 · f · sqrt(1 − ζ²) )
shaped(t) = ( plan(t) + 2K · plan(t − T_half) + K² · plan(t − 2 T_half) ) / (1 + K)²
```

f `shaper_frequency_hz`, ζ `shaper_damping` (the ZVD; the other types likewise with their weights). The delays are interpolated between profile cycles. The weights add up to one, so once the plan has stood still for a whole shaper the setpoint is the plan exactly. Where a cycle segment has a dwell or switches the payload, both wait until the shaped setpoint has arrived, the ceiling of the longest axis's shaper in profile cycles after the plan (ZVD ceil(2 · T_half / cycle), with a second ZVD ceil(2 · (T_half + T_half,2) / cycle)); the process force starts with the dwell. Segments without a dwell and without a gripper action go on with the plan. A plain move (`Move`) is no longer busy when its plan has arrived.

With `effects.shaper_follows_pose` the half period of the first shaper is that of the resonance the machine reports for the planned pose, and that of a second shaper (`shaper2_frequency_hz` above 0) the machine's second mode there (`Machine::modes`); one the machine does not report stays at its frequency. The T-gantry reports two modes, the single axis one. `sim-cli modes` and the app's tuning panel show them over an axis's stroke. Its resonance also depends on the payload: when the gripper closes at the start of a dwell it drops (on the delta D3 from 41.5 to 27.2 Hz). While the plan stands still the period stays the one it had on arrival, so a setpoint that has arrived stays on the target (the longer period reached back into the lift and took it 0.1 mm off); the next move starts with the new period (on the D3 the standstill, 25 ms of shaper lag and the 35 ms dwell, is longer than its 37 ms).

A changed shaper setting in a running simulation (the switch, the frequency, the damping, following the pose) is held back until the plan stands still and the shaper's store holds nothing but that pose. Then the setpoint is the plan with the old shaper and with the new one, and does not step. Taken at speed, the three copies of the plan were suddenly read at other delays, or two of them dropped, and the setpoint jumped by up to 16 mm on the T-gantry's X axis at 0.8 m/s.

When `profile.cycle_s` is changed in a running simulation, the stored plan of the shaper is read off again at the new spacing and every stored cycle count of the motion under way is converted, so the setpoint does not jump. The fieldbus queue keeps its count of cycles.

Following the pose applies to the first and the second shaper; the modes of `shaper_modes` keep their frequency. The optimiser (`sim-cli optimise`) switches the first and second shaper as before; it does not choose per-axis frequencies.

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
| Guide load | Per guide the peak and the cubic mean over distance, cbrt(Σ load³ |v| / Σ |v|) with v the guide's axis speed (as the pulley's radial load). The peak is the highest of all guides; the mean and the life are those of the guide that wears out first |
| Guide life | L10 = (C / P)³ · 50 km with C = `ratings.guide_dynamic_load_rating_n` and P that cubic mean |
| Guide speed, guide life in hours | The mean \|speed\| of that guide's axis (the tool's speed along it, once per millisecond); L10 · 1000 / (v · 3600) h |
| Path error | Distance from the tool position to the polyline of the setpoints of the last 0.5 s (the nearest of its pieces). Peak and RMS. The tool position is the machine's tool state: on the T-gantry the tool tip over the floor, frame deflection included |
| Tracking error | The [tool position error](#tool-position-error): distance from the tool to the plan of the same moment. Peak |
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
- The guide life is the catalogue formula and nothing more: no load factors for shock, temperature or hardness, no moments on single blocks (but on the [arm on a track](#arm-on-a-track), which reports its most loaded block), and a guide load that is itself an estimate from the rigid model. It is not validated.

## Verdicts

Source: `verdicts` in [`analysis.rs`](../crates/sim-components/src/analysis.rs). Each verdict is a value against a limit; it passes if value ≤ limit, except where noted. A rating that the config leaves out gives no verdict.

Per motor:

| Check | Value | Limit |
| --- | --- | --- |
| `motor-peak` | Utilisation | 1 |
| `motor-thermal` | Settled winding rise | `motor.rated_temperature_rise_k` |
| `stepper-synchronism` | A stepper's highest load angle over the pass, electrical rad (above π it has lost steps); only for `motor.kind = "stepper"` (see [stepper motor](#stepper-motor)) | π/2, the pull-out angle |
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
| `dc-link-energy` value | Most the link would hold (the fill below, before the resistor) | ½ C (U_peak² − U_bus²) |
| Resistor needed | That fill > room | The resistor burned any energy |
| `braking-resistor-continuous` value | What spills over the room per cycle / cycle time | Resistor energy / cycle time |
| `braking-resistor-peak` value | Peak regenerated power | Peak resistor power |

`dc-link-energy` passes if the resistor is not needed, or if the config has a continuous resistor rating to fall back on. The two resistor checks appear only when the resistor is needed. The estimate keeps the link's energy account over the cycle's stretches of one sign (`MotorLoad::regeneration_events`): what a braking feeds back fills the capacitors, what is above their room goes to the resistor, and what the motor draws between brakings empties them first (down to the bus, no lower). Brakings with too little drawn between them add up, and the cycle is counted as repeated (the link enters it as the cycle before left it). A run keeps this account online (`analysis::LinkAccount`, the motor's room from its ratings at the run): it keeps the stretches only until the link has emptied or filled whatever it held (from then on one fill stands for any entry), so the list no longer grows by one per sign change of the power (about 90 per cycle, without bound at rest with noisy power); a load built by hand gives the list. It leaves out the link's own losses, so it errs high; on the T-gantry's motor b it gives the simulated link's 0.958 W, where all that is fed back would be 2.61 W (`the_estimated_braking_resistor_power_is_not_below_the_simulated_one`; single axis with 47 µF: 2 (2.983 − 0.620) J / 2.202 s = 2.146 W, `verdicts_match_the_hand_calculation_and_fail_where_a_part_is_too_small`).

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

Source: [`collision.rs`](../crates/sim-components/src/collision.rs), `Sim::check_obstacles` and `Sim::pushes` in [`sim.rs`](../crates/sim-components/src/sim.rs), the drives' monitoring and fault reactions in [`fault.rs`](../crates/sim-components/src/fault.rs) and `Controller::react` in [`control.rs`](../crates/sim-components/src/control.rs). The config's `obstacles` are boxes with their edges along the axes of the 3D view. What can hit one is `Machine::collision_bodies`, boxes of the machine's scene at its state, and `Machine::collision_capsules`, its slanted parts as capsules (every point within a radius of a segment). By default the tool alone, as a point where the scene draws it; the T-gantry gives its Z axis, flange and payload, the single axis its carriage and payload, the D3 its platform and payload as boxes and its six forearm rods (radius 6 mm, half the 12 mm drawn) and three upper arms (20 mm) as capsules along the segments it draws, the D4 the D3's and its telescope's tube (15 mm) and inner shaft (8 mm) as capsules. The payload counts only while it is attached.

- **Clearance** between two boxes: the distance of their nearest points; where they overlap, minus the depth of the overlap: how far one would have to move to part from the other, along the axis where that is least. Between a capsule and a box: the least signed distance of the capsule's segment to the box, less the radius. The signed distance of a point to a box is convex along a line, so its least along the segment is found by a golden-section search to the last bit of the segment's parameter: exact, not a box around the slanted part (which made the D3's rods up to 95 mm and its arms 40 mm wider than they are until 14 November). A contact is a clearance of 0 or less. The clearance of the machine is the least over its parts and the obstacles.
- **Before a run** (`check_path`, `check_cycle`): the path is followed straight from pose to pose, at least every millimetre, with the machine at rest at each pose (`Machine::equilibrium`). The cycle goes from the start pose to its first target with the payload attached, then target to target with the payload as the segment before left it, and round from the last target to the first; a segment with via points follows the path the planner times through them. Corner blending is not followed: it cuts the corners, and the run is checked anyway. Reported are the least clearance and the first pose in contact.
- **While it runs**: after every base step. The simulation keeps the clearance now and the least so far (`Sim::clearance`, `least_clearance`), marks the obstacles touched, and records the first contact: the time at the end of the step that found it, the tool's pose and its speed (the length of its speed vector) then, and the obstacle; with the crash also the largest contact force since and the impact energy (below).
- **The check** `collision` judges the run's least clearance (the way to the cycle and its first pass included) against `profile.min_clearance_m`. That key was the optimiser's clearance over pick and place; it is now the one clearance the machine keeps, from the obstacles as well. A time-optimal path that comes nearer to an obstacle than it, or touches one, is refused (`error-path-collision`); a path planned within the axis limits is planned all the same and reported. The cycle optimiser judges every candidate by the verdicts, so it takes no lower lift that comes too near, and lowers each lift no further than its traverse keeps the clearance (see Optimiser below).
- **The crash** (`effects.crash`): an obstacle pushes back on a part that goes into it, with its contact stiffness k and damping c (`stiffness_n_m`, `damping_n_s_m` in its entry; [estimate] 1e7 N/m, a steel fixture on a machine bed, and 1e4 N s/m, near critical for a few kilograms; [bounds] 1e6 to 1e8 N/m), F = k d + c ḋ for a depth d, pushing only. The axes in contact are blocked by it; the others go on, until the drives' monitoring trips them (below) and each stops by its fault reaction. The push is a spring on the motors' angles θ: before each base step, for every obstacle a part has come within 10 mm of, the depth d₀ of the rigid kinematics at the motors' angles (`forward`, then `equilibrium`, then the clearance, negated) and its slope g = ∂clearance/∂θ by central differences of 1 µrad are taken, and over the step d = d₀ − g · (θ − θ₀) and ḋ = −g · θ̇ at every stage of the integration, with the torques τ = F g added to the motors' torques; the work −∫ τ · θ̇ dt it takes out of the machine is integrated with the motion and booked as the budget's `crash` row (`Sim::crash_energy`: what the damping lost and the spring holds), and the budget of the move into a box closes as the others do. At rest against a box a drive at its current limit T_max with ratio i and pulley radius r pushes F = T_max i / r (the single axis: 3.8 · 5 / 0.02 = 950 N). Reported: the force now (`Sim::contact_force`), and in the first contact the largest force since (`peak_force`) and the impact energy (`impact_energy`: the most work the pushes had taken out of the machine at any time since, at the deepest point of an impact the kinetic energy it took). The single axis at 1 m/s into a box: 7.2 kN at the impact, 1.3 J. *Shortcut*: the push acts on the motors through the rigid kinematics, not on the part that touches; a load on a soft belt or gearbox swings on into the obstacle by its transmission's give (2.6 mm on the single axis's belt at 1 m/s) and springs back. Pushing on the part itself needs a `Machine` method for an external force at a body. Off, contacts are still found and reported, and the machine moves through the obstacle.
- **Drive monitoring** (`effects.drive_monitoring`): each drive watches, every base step after its loops (`fault::Watch`), in this order: its following error against `drive.following_error_window_rad` (S120 p2546); once its setpoint has stood still for `standstill_time_s`, the following error against the tighter `standstill_window_rad` (p2542, p2543); whether its velocity loop has asked for more than the torque limit while the motor turned slower than `blocked_speed_rad_s` for `blocked_time_s` ("motor blocked", F07900 with p2175 and p2177); and, where `torque_band` is above 0, its torque against the torque the model expects (its torque feedforward, before its weight), out by more than that share of the peak torque, as a robot controller detects a collision (with `drive.torque_feedforward_model = "inertia"` that feedforward has no gravity, so the expected torque adds what the rigid model needs to hold the planned pose, its torque at that pose at rest, which the controller works out with the setpoints: a held load is expected, a lasting extra load is not; and the friction the machine's own model takes of the motor's torque at the machine's state, its signal `motor.<id>.torque_friction`, so the guides' bristles, the shaft's breakaway and the gearbox's mesh friction count as they act through a start or a reversal, where the steady friction at the planned speed, which the monitor took before 17 November 2026, was off by nine tenths of the friction as the axis broke away behind the plan (`the_torque_band_expects_the_model_s_friction_through_a_reversal`: within 0.038 N m against 0.089 of a 0.098 N m friction, the rest the axis catching up with the plan; a machine that does not report the signal still gets the friction at the planned speed, the drive's characteristic where it has one, else `steady_friction`), so a band of the friction's size trips nothing on the hanging single axis with its friction on, `the_torque_band_expects_the_friction_at_the_planned_speed`). The first monitor that trips stops every drive by its fault reaction and drops the move or cycle; `Controller::fault` records which monitor of which drive, when, the tool pose then, how far each tool axis has gone since (the stopping distance) and when the machine stood. A blocked axis trips on its following error when that crosses the window: the setpoint runs on at its speed ω while the motor stands, so some window / ω after the contact (the single axis at 1 m/s: 1 rad / 250 rad/s = 4 ms, seen at the position loop's next cycle). Enabling the drives acknowledges the trip. The windows are settings, not physical values: [estimate] 1 rad (5 mm on a linear motor), 0.1 rad after 1 s, 3 s below 12.57 rad/s (120 r/min, the S120 defaults), no torque band; no shipped cycle trips with them.
- **Fault reactions** (`drive.fault_reaction`, per motor as `[drive.<id>]`): `off1` ramps the speed setpoint down from the speed at the trip at the rated speed over `off1_ramp_s` (p1121), `off3` over `off3_ramp_s` (p1135), with the velocity and current loops following it and the position loop open; `current_limit` drops the setpoint to zero at once, so the velocity loop brakes at the torque limit; `sto` takes the pulses off at once and the motor coasts. A ramping drive takes its pulses off once its ramp is down and its motor below `pulse_off_speed_rad_s` (p1226). A drive with its pulses off gives no torque and its winding is left out of the integration, while the others may still ramp. When the last is off the drives are disabled and holding brakes engage (`effects.holding_brake`; the brake is the machine's, for all axes at once). An axis not in contact, on OFF3 with ramp time t and rated speed n, stops within v² / (2a), a = n / t · r / i at the carriage, plus the speed loop's lag behind the ramp (about a millisecond): the xyz portal's X axis from 0.8 m/s in 26.3 mm against 25.5 mm by hand, on OFF1 in 254.9 mm against 254.8 mm, on STO without a brake coasting 148 mm on its friction. *Shortcut*: the holding brake is the machine's, engaged once all drives are off, not per axis as each stops.

Hand calculations in [`sim-machines/tests/collision.rs`](../crates/sim-machines/tests/collision.rs), [`sim-core/src/scene.rs`](../crates/sim-core/src/scene.rs), [`fault.rs`](../crates/sim-components/src/fault.rs) and [`machine-delta-d3/src/tests.rs`](../crates/machine-delta-d3/src/tests.rs): a box beside a straight move of the single axis keeps 0.13 m, planned and run; a move into a box at 1 m/s is blocked at its face at x = 0.47 with an impact speed of 1 m/s and trips on the following error 4 to 6 ms after the contact; unmonitored, it pushes the box with 950 N at its current limit; the xyz portal's X axis stops on OFF3 and OFF1 within v² / (2a) while its Z axis is blocked on a slab; a capsule slanted at 45 degrees keeps (0.2 − 0.1) / √2 − 0.006 = 64.711 mm from a cube; a D3 rod is no wider than its 6 mm radius plus 0.1 mm; a point inside a cube is as deep as its way out; each monitor trips at the step counted by hand; a box under the T-gantry's traverse keeps 0.048 m, which fails at the default 0.3 m and passes at 0.04 m.

## Picking from a conveyor

Source: [`conveyor.rs`](../crates/sim-components/src/conveyor.rs), the conveyor cycle in [`control.rs`](../crates/sim-components/src/control.rs). With a [`[conveyor]`](config-reference.md#conveyor) table the cycle picks parts from a moving belt, one pass per part, on any machine whose tool axes are Cartesian positions; it knows the machine through its tool pose, axis limits and range alone.

- **The belt and its parts**: the belt has run S(t) = v t from the start, or v t² / (2 t_r) during a ramp of t_r (then v (t − t_r / 2)); part k passes the origin at its arrival time and is S(t) − S(arrival) along the belt from there, its grip pose the origin plus that times the belt's direction.
- **A move of a pass** is a superposition: a jerk-limited straight move (`profile::Move`, as a cycle segment's) from where the phase starts to a point Q, plus a belt term whose velocity blends from the start velocity va to the end velocity vb as va + (vb − va) h(u), h(u) = 3u² − 2u³, u = t / T. The blend has no acceleration at either end, so the phases join without a jump in acceleration; it covers T (va + vb) / 2. A target that is at P now and moves with vb is met at T, with its velocity, where Q = P + T (vb − va) / 2.
- **The intercept**: the move is planned with each axis's limits less what the belt term takes: max(|va|, |vb|) of the speed, 1.5 |vb − va| / T of the acceleration (the blend's peak, at u = 1/2) and 6 |vb − va| / T² of the jerk (at its ends). Since these and Q depend on T, T is the shortest time in which the move to Q fits into T, found by bisection (60 halvings from a bracket found by doubling); with va = vb at once, the move's own time. For a trapezoidal move with cruise it is the root of a α T² − (α c + a β + v′) T + β c = 0 (α = 1 − v / (2 v′), β = D / v′, c = 1.5 v, v′ = v_max − v), which the test checks. The sum of move and belt term keeps the axis limits; as the reservations hold over the whole move, it may use less than them: a faster belt makes the moves gentler and longer, and so need not raise the RMS torque (the validation has an 11 % fall).
- **A pass**, from rest above the place (or the start pose): to above the part (va = 0, vb = the belt's velocity), down onto it, the grip (`grip_s`) and up (all with the belt's velocity, the move only along the lift), to above the place (vb = the place's velocity), down, the release (`release_s`), up, and from a moving place to rest above it. The payload is attached at the end of the grip and let go at the end of the release.
- **During the belt's ramp** the part speeds up: the intercept meets it where it is at T, P(T), with its velocity then, vb(T), by Q = P(T) − T (va + vb(T)) / 2 (the same bisection), and the grip follows the belt's own motion S(t). The setpoint's acceleration steps by the belt's, v / t_r, where the grip begins and ends.
- **With input shaping** the shaped setpoint of a plan at the belt's speed lags it by the mean of the shaper's delays, Σ A_i t_i, and is on the plan only once the whole shaper (its last delay) has passed. So the pass is planned for the part that mean ahead on each axis, and the grip and the release last their time plus the shaper's span; the grip as the gripper and the tracking error count it begins once the shaped setpoint is on the part, and lasts `grip_s`. Planned without this, the tool trailed a part on a belt at v by v Σ A_i t_i through the grip (10 mm for a 25 Hz ZVD at 0.5 m/s).
- **Waiting and missing**: passes start also while the belt speeds up. The first part not yet planned for is taken; where the grip would begin before the window, the pass starts later, at the time found by bisection at which it begins at the window's start; on a belt that stands, not before the part has arrived. Where the grip would end beyond the window, or a pose of the pass (every 5 ms) leaves the machine's range, the part is missed and the next is taken. After a thousand misses in a row the cycle ends, and the analysis says that no part can be picked (`error-conveyor-unreachable`).
- **Results** (`CycleLoad::conveyor`): the measured pass is one pick, so the picks a minute are 60 / its time; a machine faster than the parts waits, and its passes come a pitch apart. The tracking error is the most the tool's measured pose, taken along the belt, was from the part's during the grip. The parts missed a minute are those the belt brings beyond the picks a minute, or those missed in the measured pass, whichever is more: one pass cannot see misses that come in bursts. The verdicts are `pick-tracking` (against `conveyor.tracking_tolerance_m`) and `parts-missed` (against 0).
- **Obstacles**: `check_cycle` plans the passes for two pitches' worth of parts from the start pose at time 0 (without input shaping) and checks them every 5 ms with the payload as each holds it; the live check (see above) judges the run, and `collision` fails where a pass comes too near.
- **The optimiser** leaves `[[cycle.segment]]` alone with a conveyor (it does not run): it tries no lifts or via points, only the limits and switches.
- **The scene** draws the belt as a band under the grip poses (300 mm wide), slats every 100 mm that run with the belt, and each part on it that is not in the gripper as a 50 mm cube.

Shortcuts: the acceleration step at the grip's ends during the belt's ramp; the shaper's lead is the mean delay at the pose the pass starts from (a shaper that follows the pose may differ at the part); the reserved limits are conservative (the moves are slower than a planner that shares the limits along the move would make them); the second belt carries no parts in the scene, and its slot is wherever the place pose is when the tool leaves for it.

## Other analyses

| Analysis | Definition |
| --- | --- |
| Error bands | The cycle is run with the config as it is and with each parameter of `[bounds]` at its low and at its high value, one at a time. A bound on a value that only an effect that is off uses (`[thermal]` without `effects.part_heating` or `thermal_feedback`, `[thermal_feedback]` without `effects.thermal_feedback`, `belt.tooth_jump_factor` without `effects.belt_tooth_jump`, `ratings.line_module_efficiency` without `effects.regenerative_supply`), or that no cycle uses (`ratings.dc_link_discharge_time_constant_s`, which acts only while the drives are disabled), gets no runs, here and in the sensitivity. A band is the envelope of the verdict's value over all runs; the worst case passes only if every run passes. The runs share the processor cores |
| Sampled error bands | `--bands --samples N`: N runs with every parameter of `[bounds]` between its bounds at once, as a Latin hypercube: the range of each parameter is cut into N equal parts, each part is used in exactly one run at a random place inside it, and each parameter has its parts in a random order of its own. Random numbers from the simulator's own generator and a seed, so a call repeats. Reported per check: the envelope, the worst case, and the share of the runs (the config as it is among them) that pass; a run in which a check does not come up has passed it. It finds what two or more values do together; it does not put a value exactly on a bound, and N runs do not find the worst of the 2ⁿ corners |
| Worst corner | `--bands --worst` (`analysis::verdict_worst`): per verdict a search among the 2ⁿ corners. (1) The one-at-a-time runs above. (2) The corner where each parameter is at the bound that made this verdict worse alone (at its high bound where it moved the verdict not at all); for a verdict that is monotone in each parameter and whose effects do not depend on each other, that is the worst corner. (3) Up to `WORST_PASSES` = 3 passes: every parameter that moved the verdict alone is flipped to its other bound, one at a time; the flips that made it worse are taken together, and the worse of that corner and the best single flip becomes the corner if it is worse than before; a pass that finds nothing worse ends it. "Worse" is the margin, value − limit (limit − value for `belt-slack` and the lives, `AT_LEAST`), not the value, because some limits are bounded too (DC link capacitance and threshold). `dc-link-energy` is the exception: it fails only where the braking resistor is needed and has no rating, and with a resistor the link fills up to its room at every corner that brakes hard (margin 0 at all of them), so it is ranked by the energy in the capacitors, a failing run above any passing one. A corner is run once for all verdicts that land on it; reported per verdict is the worst at any run, the one-at-a-time runs and the config as it is included (so the worst is never better than the bands), with its value, limit, result and where the parameters were (a parameter left as configured is at neither bound). Cost: 2n + 1 runs, plus per pass up to the number of parameters per verdict; on the EXCT-30 (63 bounded values, 24 checks) about 2400 cycles in about 47 s on all cores, on the single axis (14) about 300 in 3.5 s. A search, not a proof: a worst case that no chain of single flips from the sign corner reaches, or one inside the bounds (a resonance hit by a stiffness between them), can be missed |
| Sensitivity | The same runs, reported as relative changes of peak torque and RMS torque (each of the motor it changes most) and of the cycle time. Ranked by the largest of these six changes |
| Optimiser | Starts from the config. Candidates: toggle corner blending, switch input shaping off where it is on, raise velocity, acceleration or jerk of all axes by 1.5 or 1.2. A candidate without input shaping that fails `settling` and nothing else is run again with the shaper, and with the shaper following the pose where the machine reports a resonance; one with the shaper that still fails `settling` alone, with a second shaper at `profile.shaper2_frequency_hz`, or where that is 0 at the resonance the machine reports at the start pose if that is more than 10 % off `shaper_frequency_hz`. Where a second shaper is on, switching it off is a candidate too. The first of these that is shorter by more than 0.1 ms, has no motor-limit violation and passes every verdict is taken; repeat until none is. **Geometry** (`analysis::lifts`): a lift, traverse and lower is three segments in a row of which the first moves along the last axis only, away from both the start (the target before) and the end of the three, the second along the other axes only at that height, the third along the last axis only; the first two neither dwell nor change the payload. Its lowest height is the farther of the two ends plus `profile.min_clearance_m`. Candidates, before all others: every lift at its lowest, then each alone at its lowest and halfway there. After all others: a lift rounded through a via point (`control::via_pieces`: a line straight up from the start to the lift height, a spline through the via point to the point straight above the end, a line down; timed by `path::plan` within the axis limits, with the transition curves of `profile.path_tolerance_m`), the via point at the middle and 1/2, 1/4, 1/8 of the width above the lift height; a via point moved by 1/8 or 1/32 of the width along each axis; the rounding dropped. A via candidate whose planned poses between the two ends (along the other axes) come nearer to the ends than the lowest height, less the path tolerance, is not run. The new cycle is reported as a whole (`changes` entry `cycle.segment`, applied by `set_text`) and per lift (`Optimised::lifts`). On the default T-gantry with a clearance of 0.1 m the lifts go to 0.15 m and the cycle from 6.08 s to 2.228 s with the other steps (2.344 s robust; `sim-cli optimise t-gantry --set profile.min_clearance_m=0.1 [--nominal]`, 17 November 2026; 3.36 and 3.44 s before); a rounded lift never won there (300 runs, with and without a path tolerance of 2 mm): the arch's curvature and its slower speed cost more than the two stops it saves. **Obstacles** (`over_obstacles`): where the config has obstacles, a lift's lowest height is the lowest at which its traverse, followed straight at that height with the machine at rest every millimetre (as `check_path`), keeps the clearance to them with the collision bodies; from the height above, it is moved by what the clearance lacks (or has too much, never below the height above) until it is the clearance to 1 nm, at most 40 times within the axis range (none found: the lift is not lowered). The bodies move with the last axis one to one, so a box across the traverse is cleared in one step: lift = top of the box + clearance + how far the lowest body reaches below the tool. The run sways a little (0.05 mm on the T-gantry), so a candidate that fails `collision` alone by less than 1 mm is run again, at most twice, with the lifts the obstacles raised higher by what it lacked and 1 µm. A lift an obstacle raised is also tried rounded through a via point over the obstacle's nearest pose at that height and 1/8 of the width above it, its ends at the height above pick and place; every via candidate's planned path is checked against the obstacles (clearance kept) before it runs. **Profile** (`profile`, `lift_profiles`): per obstacle, `over_obstacles` with it alone; those it raises are in the way, with the stretch of the traverse (every mm, at the lowest height, at rest) nearer to it than the clearance; overlapping stretches merged at the higher height; with two or more, two via points per stretch at its height, at its ends. The ends of the path at the height above pick and place. The planned spline is checked as every via candidate, and where it lacks clearance, all points are raised by what it lacks and 1 µm, three times at most; a profile that still lacks it is dropped. It is a candidate beside the via over the nearest pose; a profile is not moved, only kept or dropped. **Robust** (`OptimiseOptions::robust`; `sim-cli optimise` without `--nominal`, the app with "error bands" ticked): the fastest setting found is judged with the error bands (one bound at a time, 2n + 1 runs); where it fails a check there that the cycle as configured passes there, the accepted steps of the search are judged from the fastest down, each 2n + 1 runs, until one passes; the cycle as configured passes by definition. A step that fails `collision` alone in the bands, by less than 1 mm, is judged again (one run and its bands) with the lifts the obstacles raised higher by what the bands lacked and 1 µm. Checks the configured cycle fails in its bands already are reported (`Robust::excused`) and not judged. EXCT-30: the search's 1.492 s fails `belt-slack`, `settling` and `pulley-bearing-life` in the bands, the configured cycle `pulley-bearing-life` already; one step back, 1.590 s, passes the rest; 3 × 121 = 363 band runs. Default T-gantry: 5.340 s fails `settling`, 5.364 s passes, 3 × 47 = 141. **Worst corner** (`OptimiseOptions::worst`; `sim-cli optimise --worst`, the app with "worst corner" ticked): the same, judged with `verdict_worst` in place of the bands. The search stops after the first batch of runs (the 2n + 1 runs alone, the corners, a pass's single flips, its combined flips) in which a check judged on fails; it starts per verdict from the sign corner and from the previous judged step's worst corner (its parameters left as configured at the sign corner's bound), the worse of the two. The fastest is judged first, stopping at any failure; where it fails, the configured cycle is searched whole for the excused checks, the fastest judged again on the rest where it failed only excused checks or `collision`, then the accepted steps bisected (step 0 passes by definition): log2 of the steps searches instead of up to one per step. EXCT-30: the fastest (1.492 s) fails `belt-slack` and `settling` at the corner, steps 3 and 1 of the seven fail as well, `pulley-bearing-life` excused; the result is the configured 2.048 s, 3001 corner runs, 2.9 min on 16 threads. Default T-gantry: the search's 5.340 s fails `settling` there; three steps back, 5.442 s; 4964 corner runs, 16.5 min (bands: 5.364 s). Not the default for that time. **Front** (`OptimiseOptions::pareto`): every setting run that passes every verdict, and from the result its acceleration and jerk, and all three limits, times 0.8, 0.6 and 0.4 (six runs), as (cycle time, energy per cycle from the supply, highest `MotorLoad::utilisation`); kept are those no other is as good as in all three and better in one (`non_dominated`). Energy from the supply (`supply_energy`): what the drives drew less what they fed back, plus what the braking resistor burned (a shared one once); the line module's loss and the bleeder are left out |
| Frequency sweep | No control. A sine torque on one motor on top of the holding torques; that motor's speed is correlated with sine and cosine over whole periods (at least 0.2 s) after settling (at least 0.3 s). Gain in rad/s per N m |
| Step response | The machine settles for 0.05 s, then the position setpoint of one axis is offset by the step size, without feedforward. Rise time 10 % to 90 %; overshoot as a fraction; settling to within 2 % |
| Closed-loop response | Without feedforward, as the step response: a sine offset on the position setpoint, measured at the tool. Bandwidth: where the gain first falls below 1/√2, interpolated on the logarithmic frequency axis |
| Motor search | See the [user guide](user-guide.md#find-the-lightest-motor). Velocity gain of a candidate = the config's gain × sqrt( ratio of inertia seen × ratio of rotor inertias ), rotor inertia with the gearbox's own, against the config's own numbers. The same rule applies wherever a part is named (`[parts]`, `--motor`, `--gearbox`). "Tightest check" leaves out `dc-link-overvoltage` and, with a braking resistor in use, `dc-link-energy` while they pass |

## Positioning accuracy

`accuracy::study` (`sim-cli accuracy`, `ks.accuracy`) runs each tool axis through ISO 230-2's linear cycle and judges the real tool pose at rest; see the [user guide](user-guide.md#how-accurate-and-repeatable-is-an-axis) for the procedure.

| Quantity | Definition |
| --- | --- |
| Deviation x_ij | Real tool position less the target, averaged over the last tenth of the rest at target j, approach i |
| x̄↑, x̄↓, s↑, s↓ | Mean and sample standard deviation (n − 1) of the approaches from below and from above |
| B | max_j \|x̄↑ − x̄↓\| |
| R | max_j max(2s↑ + 2s↓ + \|x̄↑ − x̄↓\|, 4s↑, 4s↓) |
| E | max of all x̄↑, x̄↓ − min of them |
| M | max_j (x̄↑ + x̄↓)/2 − min_j of the same |
| A | max_j max(x̄↑ + 2s↑, x̄↓ + 2s↓) − min_j min(x̄↑ − 2s↑, x̄↓ − 2s↓) |
| AP, RP, vAP (tool) | Over the axes in metres, per target and direction: the distance of the barycentre of the attained poses from the target; l̄ + 3 S_l, l the distances from the barycentre; the distance between the two directions' barycentres. Each the largest over targets and directions |

**Thermal growth** (`effects.thermal_growth`) and **lead error** (`effects.lead_error`), both optional keys, off where a config does not have them, for the axes of `accuracy.axes`: the real position is x + α (ΔT_config + ΔT_part) (x − x_fixed) + e(x), x the position the controller holds, α `accuracy.screw_expansion_per_k` (11.5e-6 / K, hardened bearing steel) or `accuracy.belt_expansion_per_k` (11.5e-6 / K, [estimate]: the steel tension cords set the pitch; no belt datasheet found gives a coefficient; a glass-fibre cord would be about 5e-6, aramid about −2e-6), ΔT_part the rise of the named heated part (`Sim::heated`), e the lead error table linear between its rows and constant beyond them. The map is applied to the measured tool pose, not inside the machine's equations: a lead changed by 1e-4 changes the inertia the motor sees by 2e-4, far below what matters for a static test. With `effects.load_encoder` neither is applied.

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
       + crash = work the obstacles' pushes took out of the machine                with effects.crash
unaccounted = supply − rows
electrical  = supply − copper − resistor − discharge − mains − line module − ΔE windings − ΔE capacitors − Σ W   (W: shaft work)
```

The rectifier's energy is counted on its own, every step, as what lifts the capacitor energy back to that of the bus voltage. A payload picked up brings its kinetic and potential energy into the machine and one put down takes it away; the simulation adds up these steps (`Sim::payload_energy`). The stores are read after the last step counted: a cycle's measured pass leaves out the step in which it begins and the one in which the next begins, flows and stores alike. (Until 12 November the stores were read one step later than the flows ended, which on the loss-free elastic configs put up to 0.8 % into "not accounted for", taken then for backlash knocks.)

**The electrical side is checked on its own** (`EnergyBudget::electrical_unaccounted`): the supply against copper loss, resistor, the discharge of drives switched off, mains, line module, the windings' and the capacitors' stored energy and the work the torques did on the shafts, integrated with the windings (`EnergyBudget::shaft_work`). Within `ELECTRICAL_TOLERANCE` = 1e-6 of the supply on every machine and config, the loss-free ones included: measured at most 3.2e-7 (single axis, loss-free inclined), 1e-12 on the default configs; it falls by 15 with each halving of the integration step, as fourth order does. With `effects.back_emf` off the shaft work comes from nowhere and this does not close; no shipped config has it off. What `unaccounted` holds beyond it is the machine's own balance.

**Tolerance.** What is not accounted for stays below `BUDGET_TOLERANCE` = 0.1 % of the supply on every config, the loss-free ones included: measured 0.0004 % on the default T-gantry (−5.4e-4 J of 143.0 J, `sim-cli analyze t-gantry --budget FILE`, 17 November 2026; 0.009 %, 1.2e-2 J of 139.5 J before), 0.00001 % on the single axis, 0.0004 % on the EXCT-30, at most 0.03 % on the loss-free elastic T-gantry (`the_energy_budget_closes_on_every_machine_and_config`). It is the machine's losses taken at the end of each base step and summed times the step, the trapezoid rule but for the ends of the pass, and the undamped ringing knocking through the gearbox backlash on the loss-free configs, which the fixed step integrates through without events.

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
- **Gearbox.** Without `effects.gearbox_drag` the model has one efficiency and no no-load torque. Makers quote the no-load running torque at 20 °C and it falls as the gearbox warms (churning and bearing drag in thinner oil), while the tooth friction under load does not. So a share s of the loss (1 − η) follows the viscosity, the rest stays: an estimate, s = 0.5, bounds 0.25 to 0.75. 30 K warm, a 95 % gearbox has 96.80 %. With `effects.gearbox_drag` the churning and bearing drag is the [no-load torque](#gearbox) itself, T_0 ∝ (ν / ν_ref)^(2/3) at the gearbox's temperature, and s is taken as 0: the efficiency is the teeth's. Harmonic Drive's chart of a size 32 gear (20 to 40 °C: 0.54) agrees with this law at B = 4000 K (0.56).
- **Shaft seals.** The motor shaft's Coulomb and breakaway levels are its seals' and bearings'; on a geared motor the seal is the gearbox's input seal in its lubricant: (ν / ν_ref)^m at the gearbox's temperature, m `seal_viscosity_exponent` (estimate 0.33, bounds 0 to 0.5). At −10 °C (ν 4.74 times) that is 1.67 times the level, at 30 K above the reference 0.66.
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

What it moves: the windings settle at 1 to 2 K on the shipped cycles, so the shipped verdicts move in the fourth digit. With part heating and the feedback on, the single axis's guide settles at 10.03 K instead of 12.51 K (its viscous friction is 0.637 of the cold one there, not 1) and its `gearbox-continuous` falls from 0.806 to 0.695 N m; the T-gantry's guide at 3.86 K instead of 4.20 K; the EXCT-30's X guide at 15.90 K instead of 23.70 K, its windings at 8.81 and 8.46 K instead of 8.98 and 8.56 K (`sim-cli verdicts` with `--set effects.part_heating=true`, and `--set effects.thermal_feedback=true` added, 17 November 2026; 6.62 and 6.08 K instead of 6.88 and 6.24 K before). A 0.5 A-rated winding (single axis) settles at 73.8 K, and the measured pass's copper loss is R (1 + 0.0039 · 73.8) I² = 1.026 W instead of the cold 0.797 W.

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

**Replay.** The motor position setpoints of the trace become tool poses through the forward transformation and run as a trajectory. Each row is commanded one row earlier, because a run writes a row before it computes the step at that time. The simulator's own controller makes the speed and torque feedforward from the poses. The simulation starts from the trace's own state: at rest at p0 − v0 T / 2 (p0 the first row's setpoint pose, v0 its measured speed in tool units), it runs up along p0 − v0 T / 2 + v0 T (2.5 u⁴ − 3 u⁵ + u⁶), u = t / T, a smootherstep of the speed with zero acceleration at both ends, over T = max(0.25 s, 1.875 |v0| / (a_max / 4)) per axis, and reaches p0 at v0 one row before the first row; the rows are recorded from there. The run-up starts at the setpoint the machine has at rest there (its motors, a sag away from the tool) and takes that difference up along a smoothstep. A trace that starts at rest where the config's machine stands at rest (its drives' setpoints those of the first row, as in a trace a run of the config wrote) runs no run-up: the replay starts from that state, row 0 the machine at rest. Behind the last row two points carry the motion on with the last rows' speed and acceleration before it stops, unrecorded.

**Friction beyond the sliding level** (`identify::friction_details`, rows of `fit`). From an axis's force F, speed v and travel, with its rigid values m, b, c (Coulomb), k (constant): the force held at rest after the last motion, the mean of F − k over the last stretch whose 11-row mean speed is within 1 % of the top speed, after its first 0.1 s; the breakaway level, the mean of sign(v) (F − m a − b v − k) on the rows between 1 and 5 % of the top speed whose speed falls in size and keeps its sign across the window; the presliding stiffness, σ = c ln 2 / x0 with x0 the travel from the turning point at which sign(v_after) (F − m a − k) passes zero, the median over the reversals (Dahl's law above). Means with the standard error of n rows widened by (1 + ρ) / (1 − ρ) for their lag-one correlation ρ; the median with 1.25 times the reversals' spread over the root of their count. The same on the replay, as for the rigid values.

**Stage 1, rigid values.** Motor torques become axis forces through the machine's transformation (motor angle per axis travel; the same power on both sides), so the fit works per axis even where every motor moves every axis. Per axis, least squares of

```text
force = mass · acceleration + viscous · speed + coulomb · sign(speed) + constant
```

The acceleration is the second difference of the axis's travel (an encoder's position is far less noisy than the speed differentiated, and noise on a regressor pulls its value towards zero); force, speed and every column are smoothed alike by a triangle of half-width h (`identify::smoothing_span`, 40 ms where the trace allows), the same as the fit in the motors' coordinates. Rows slower than 5 % of the top speed, or whose window holds a reversal, are left out. A payload picked or dropped during the trace (found as in the fit in the motors' coordinates) adds a column per step, the payload's force on the axis from the step on. The same estimate runs on the trace and on the config's own simulation of the trace's setpoints, on the rows both take and on the trace's columns; what the simple estimate gets wrong (gearbox losses that grow with torque, load-dependent friction) it gets wrong on both sides, so the difference is what the config is off by. Its standard errors are those of that difference: the noise's variance from the residual of (trace − replay), Σ (r_trace − r_replay)² / (N − m · per) · per, where per ≈ 1.5 h rows count as one measurement and each of the m values fitted takes per rows' worth of freedom; at least 0.001 of the trace's own residual, the floor a replay of the config's own run, without noise, would otherwise fall to. On the config's own trace with noise the bands cover it (standard deviation of (fitted − config) / error 0.88 to 1.15 over 80 seeds); on a machine that differs from the config the rigid model mixes the changes a little (on the single axis 1.5 kg more mass shows as 1.57 kg), which a second pass with the config corrected takes out. The same errors in the motors' coordinates.

**Stage 2, resonance.** In a sweep of motor torque to motor speed, the load resonating on the belt shows as a dip. Gain times frequency is level for a pure inertia; the dip's flanks are where that has twice the power of the lowest point. Frequency = geometric mean of the two flanks; damping ratio = (high − low) / (2 · frequency); frequency error = half the spacing of the sweep's points there. The measured sweep's lowest dip is set against the config's dip nearest to it in frequency, the config's sweep run up to a factor of √2 beyond the measured range so that a dip near its end keeps both flanks. With the masses right, (measured / config frequency)² is the factor the stiffness is off by.

## Pneumatic cylinder

Source: [`pneumatic.rs`](../crates/sim-components/src/pneumatic.rs). A machine file attaches one with a
`[[machine.cylinder]]` node ([machine-files.md](machine-files.md)); the cylinder is no tool axis and has no drive: the
cycle commands its valve and waits for its end-position sensors (`[[cycle.segment]]` `valves` and `wait`, see
[config-reference.md](config-reference.md)). Its states follow the machine's: piston position x (0 retracted, s
extended) and speed v, and per chamber the air's mass m and internal energy U, then running totals for the balances.

**Chambers.** Cap side 1 (area A1 = π D²/4), rod side 2 (A2 = A1 − π d²/4), each a control volume of an ideal gas
(R = 287.058 J/(kg K), κ = 1.4, c_p = κ R/(κ − 1)):

```text
V1 = V01 + A1 x,  V2 = V02 + A2 (s − x),  p = (κ − 1) U / V,  T = p V / (m R)
dm/dt = ṁ,   dU/dt = c_p T_in ṁ + α A (T_wall − T) − p dV/dt
```

T_in is the port's temperature for air flowing in, the chamber's for air flowing out. **Thermal assumption:**
adiabatic with heat transfer to the wall, `heat_transfer_w_k` = α A per chamber. 0 is the adiabatic limit (a closed
chamber's air spring has the exponent κ); a heat transfer quick against the process (α A ≫ m c_v / t) is the
isothermal limit (exponent 1). Both limits are tested on the natural frequency. The wall and the supply are at
`ambient_temperature_k`.

**Flow elements (ISO 6358).** Every flow path is a sonic conductance C (m³/(s Pa), ANR; the config gives l/(s bar))
and a critical pressure ratio b:

```text
ṁ = C p_up ρ0 sqrt(T0 / T_up) ψ(r),  r = p_down / p_up,  ρ0 = 1.185 kg/m³, T0 = 293.15 K (ANR, ISO 8778)
ψ = 1 for r ≤ b (choked),  ψ = sqrt(1 − ((r − b) / (1 − b))²) above
```

The last 0.5 % of r before 1 is a straight line through zero (the root's slope is infinite there, which no fixed step
integrates; a shortcut, see below). Elements in series combine to C = (Σ C_i⁻³)^(−1/3) and 1 − b = C² Σ (1 − b_i)/C_i²:
the cube rule for the choked flow, and b so that at small pressure drops (where the elliptic law is Δp ∝ ṁ² (1 − b)/C²)
the drops add up.

**Valve and paths.** A 5/2 valve (or 5/3 with a closed centre) switches `valve_switch_time_s` after its command
changes; extending, chamber 1 is on the supply (`supply_pressure_bar` gauge plus ambient) and chamber 2 on the exhaust
(ambient), retracting the other way. Air into a chamber passes valve and tube (the throttles' and cushions' check
valves open); air out of it passes valve, tube and its meter-out throttle (`throttle_extend_*` on the rod side's port,
`throttle_retract_*` on the cap side's). Within `cushion_length_m` of its end (effects.cylinder_cushion), the cushion
spear closes the main bore and the chamber exhausts through valve, tube and the cushion bleed alone, blended in over the
first millimetre.

**Piston.**

```text
(m + payload) dv/dt = p1 A1 − p2 A2 − p_atm (A1 − A2) − F_load − (m + payload) g sin(incline) − F_seal + F_stop
F_seal = (F_c + k_p p̄_g) (1 + (r_s − 1) exp(−(v/v_s)²)) tanh(v / v_ε) + b v      (effects.cylinder_friction)
```

p̄_g is the chambers' mean gauge pressure (`friction_coulomb_n_bar` per bar), r_s the breakaway ratio, v_s the Stribeck
speed, v_ε the smoothing; a Stribeck curve rather than the LuGre bristles of the guides, whose bristle stiffness would
add a stiff state for no gain at a cylinder's speeds. F_stop is `mech::HardStop` with the cylinder's own
`stop_stiffness_n_m` and `stop_damping_n_s_m` (the elastic bumpers), always on. The payload rides on the rod while the
gripper holds it (effects.payload).

**What it reports** (`cylinder.<id>.*`): position, speed, both pressures (absolute) and temperatures, the spool, the two
end sensors (on once the piston has been within `sensor_band_m` of the end for `sensor_delay_s`), the supply's flow and
the air drawn in normal litres (ANR), the air of the last double stroke (between two switches to extend), the
compressor's energy for it (`compressor_energy_kwh_m3`), the last stroke times out and back (from the spool switching
to within the sensor band), the speed and kinetic energy entering the cushion (at the end stop without a cushion), the
speed at the end stop, the bounce back from it and the settling time (until the piston last left the sensor band), and
the load factor. Checks: `cushion-energy` (the highest kinetic energy entering the cushion against
`rated_cushion_energy_j`) and `load-factor` (the force the cylinder works against, load and weight, over p_g A1
extending or p_g A2 retracting, the worse, against `load_factor_limit`, 0.7 by the usual rule).

**Energy.** The cylinder is an open system: `Machine::energy` and the energy view leave it out. Its own balance: the
change of U1 + U2 + ½ m v² + (m g sin + F_load + p_atm (A1 − A2)) x + the stops' springs equals the enthalpy from the
supply less that to the exhaust, plus the heat from the wall, less the seals' and the stops' dampers' work, each
integrated as a state.

**Step.** The model is stiff where a chamber is small: the flow law's straight end, the air spring of the dead volume,
the stops, the seal's smoothing and the heat transfer each give a rate; a cylinder whose largest rate times the
integrator's sub-step (`sim.dt_s / sim.substeps`) exceeds 2 is refused with `error-cylinder-unstable`, naming the rate
and the largest sub-step that would do (RK4 is stable to 2.8). Not modelled: a proportional valve with a position loop
(servo-pneumatics), the valve's spool travel (it switches at once after its delay), tube dynamics (a tube is a flow
element; its volume belongs in the dead volumes), the cylinder's reaction on the axis that carries it.

## Known shortcuts

Places where the code carries a `shortcut:` comment.

| Where | Shortcut | When to upgrade |
| --- | --- | --- |
| [`sim-components/src/pneumatic.rs`](../crates/sim-components/src/pneumatic.rs), `psi` | The ISO 6358 flow law's last 0.5 % of pressure ratio before 1 is a straight line, not the root | ISO 6358-3's laminar region, if a test against measured data needs the last 0.5 % of a pressure |
| [`sim-components/src/pneumatic.rs`](../crates/sim-components/src/pneumatic.rs), module | A cylinder rides on the machine without acting back on it; its weight belongs in the carrying body's mass | A cylinder not at right angles to the axes that carry it |
| [`machine-t-gantry/src/lib.rs`](../crates/machine-t-gantry/src/lib.rs), `elastic_derivatives` (line 381) | Idler and cable-chain inertia ride on the carriage's speed over ground, not on its speed along the beam. The two differ by the frame's own motion, which is a thousand times smaller | If the frame moves by more than a small fraction of the carriage's travel |
| [`sim-components/src/control.rs`](../crates/sim-components/src/control.rs), `Controller::rescale_ticks` (line 302) | When `profile.cycle_s` changes, setpoints on the fieldbus keep their count of cycles, so the fieldbus delay changes with the cycle at once instead of after the queue has run through | Resample the queue, if that step ever matters |
| [`sim-components/src/control.rs`](../crates/sim-components/src/control.rs), `Controller::shape` (line 707) | While the shaper's period changes (`shaper_follows_pose`), the speed and acceleration setpoints are the delayed plan's, not the exact rates of the shaped position. Off by the rate of the delay, a few percent in a Z move; they are feedforward only | Shape the position alone and difference it, if that matters |
| [`sim-components/src/mech.rs`](../crates/sim-components/src/mech.rs), `Lugre::eval` (line 290) | Bristles that would settle faster than 1e5 1/s settle at that rate, with their force scaled so that sliding still takes the level | Step the friction state implicitly, if what bristles do within 10 µs ever matters |

## Other simplifications

Not marked as shortcuts in the code, but worth knowing. Most are stated in the sections above.

| Area | Simplification |
| --- | --- |
| Motor | One equivalent winding; with `effects.field_weakening` (on in the shipped configs) the q axis of a d-q model. No phases, PWM or inverter dead time; the iron loss (`effects.iron_loss`) heats the motor but is no drag on the shaft. The stepper and the induction motor are models of their own |
| Drive | Delays are whole cycles; the fieldbus jitter is whole profile cycles, evenly spread, frame by frame independent. The same settings for every drive. The filters are generic, not a vendor's |
| Thermal | One thermal mass per winding, gearbox and guide (`effects.part_heating`); no belt temperature (the belt is at ambient). With `effects.thermal_feedback` guide friction and gearbox losses follow the temperatures (one viscosity law for all lubricants); without it `effects.temperature` is two factors set by hand |
| DC link | One per drive, or one for all with the sum of their capacitances; feedback into the line only through an ideal line module of one efficiency (`effects.regenerative_supply`); no resistor heating, no Line Module capacitance or busbar resistance; the bleeder draws only while the drives are disabled |
| Belt | Mass lumped on the pulleys, or summed run by run where the runs move over ground otherwise (T-gantry, the coupled-belt and portal machine files). Sideways span vibration as first-mode frequencies only, no amplitude (`effects.belt_span_vibration`); tooth jump with a quasi-static climb-out law and the belt sliding over a sinusoidal ridge (`effects.belt_tooth_jump`); no wear |
| Friction | Guide loads from the commanded acceleration of a loss-free rigid machine, not from the true one. No moments on single guide blocks |
| Gearbox | One teeth's efficiency (with `effects.gearbox_drag` the efficiency falls at part load and with speed and the cold; with `effects.gearbox_mesh_friction` its loss is a mesh friction that sticks, in the elastic models only); the friction's bound in proportion to the torque the spring passes, not to the mean of input and output; constant stiffness unless `effects.gearbox_stiffness_curve`; hard dead zone |
| Frame | One mode, in X only |
| Z axis | Rigid body leaning on a compliant guide, in X only |
| Stop | Categories 0, 1 and 2 ([Stops](#stops)); category 2 brakes along the direction of motion, not along a curved path, and without a jerk limit; the OFF3 ramp is a time from the rated speed; no safe torque monitoring beyond the SS1 timer |
| Brakes | A spring up to the brake torque, then slipping with it; no wear. Engagement and release delays one for all brakes of a machine (`effects.brake_delay`); during the release delay the drive holds against the brake (a drive's brake control would wait for it). In the rigid model the spring holds the whole machine with the damping meant for the rotor |
| Cycle | The payload's mass switches as a step, on arrival or, with `effects.gripper_time`, when the gripper is done; the gripper's times are fixed or a first-order vacuum build-up, no gripper dynamics or compliance. The vacuum cup's hold is judged with its axis vertical and its vacuum at the switch level. A swinging payload only on the joint machines (`effects.payload_swing`) |
| Feedforward | Rigid model with inertia and gravity, loss-free: the gearbox's efficiency on them stays in the integrator. The friction feedforward (a recorded characteristic, or the machine's own steady friction) is on where it improves following and settling, off elsewhere ([validation](validation.md#friction-feedforward-in-the-shipped-configs)) |
