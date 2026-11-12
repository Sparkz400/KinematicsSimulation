# Model reference

The equations the simulator solves, taken from the code and its comments. Each section names the source file. Config keys are written as `table.key`; their units are in the [config reference](config-reference.md).

- [Structure of one time step](#structure-of-one-time-step)
- [Integrator](#integrator)
- Components: [belt](#belt-sections), [guide friction](#guide-friction-lugre), [motor shaft](#motor-shaft-friction-ripple-cogging), [tooth meshing](#tooth-meshing), [gearbox](#gearbox), [hard stops](#hard-stops), [drive and motor](#drive-and-motor)
- Machines: [T-gantry](#t-gantry), [single axis](#single-axis)
- Motion: [controller](#motion-controller), [profile generator](#profile-generator), [corner blending](#corner-blending), [path planners](#path-planners), [input shaper](#input-shaper)
- [Analysis quantities](#analysis-quantities), [verdicts](#verdicts), [other analyses](#other-analyses), [fit](#fit)
- [Known shortcuts](#known-shortcuts), [other simplifications](#other-simplifications)

Notation: x, z tool axes; a, b pulley angles (after the gearbox); φ motor shaft angle; r pulley radius; i gear ratio; T belt tension; τ torque; ω speed; g = 9.80665 m/s².

## Structure of one time step

Source: [`sim.rs`](../crates/sim-components/src/sim.rs), [`control.rs`](../crates/sim-components/src/control.rs).

One base step `sim.dt_s` (62.5 µs):

1. The controller reads the motor angles and speeds from the state at the start of the step and runs the loops that are due:
   - profile generator, every `profile.cycle_s` (1 ms): new tool setpoints, input shaping, inverse transformation, fieldbus delay
   - position loop, every `drive.position_cycle_s` (1 ms)
   - velocity loop, every `drive.velocity_cycle_s` (125 µs)
   - current loop and winding, every base step
2. The machine is told whether the payload is attached, whether the cycle dwells and whether the drives are disabled.
3. The mechanics are integrated over the base step with the motor torques held constant.

The simulation never looks at the wall clock. The same config and the same commands give bit-identical results.

## Integrator

Source: [`integrate.rs`](../crates/sim-core/src/integrate.rs).

Classical fourth-order Runge-Kutta with `sim.substeps` (4) sub-steps per base step, so the mechanical step is 15.6 µs. The integrator treats the state as one vector and does not know which entries are positions.

Assumptions: the torques are held over a base step (zero-order hold), as a drive would. Friction, backlash and belt slack are not handled as events; they are integrated through with the fixed step. The step-size test (halving the step changes the travel by less than 1 %) is what justifies that.

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

Assumptions: one constant efficiency (the catalogue's full-load value), no speed- or load-dependent loss, no no-load torque, constant stiffness, backlash as a hard dead zone. The gearbox's own inertia (`gearbox.inertia_at_input_kgm2`) turns with the rotor: J_rotor in the equations is `motor.rotor_inertia_kgm2` plus it.

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

k_e = k_t in SI units (`effects.back_emf` off: k_e = 0). Each base step the voltage u and the speed ω are held and the current is stepped exactly:

```text
i_∞ = (u − k_e · ω) / R
i   ← i_∞ + (i − i_∞) · exp(−R · dt / L)
P   = u · (i_before + i_after) / 2          electrical power into the winding; negative while the motor feeds back
```

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
dϑ/dt    = ( ϑ_rated · R(ϑ) · i² / P_rated − ϑ ) / τ_th
```

At the rated current the winding settles at `rated_temperature_rise_k`. The warmer resistance is used in the winding equation above. The thermal time constant is half an hour, so a simulated cycle hardly warms the winding; the analysis therefore reports the rise the winding would *settle* at (see [analysis quantities](#analysis-quantities)).

Assumptions: only copper loss heats; the surroundings are at a fixed temperature; no housing, no second thermal mass; the torque constant does not change with temperature.

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

w_v and w_τ are the feedforward factors (0 with the switch off). The notch is a biquad with centre `notch_frequency_hz` and Q = frequency / bandwidth.

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

ω₀ `no_load_speed_rad_s`. Full torque up to the rated speed, falling linearly to zero at the no-load speed. With `effects.torque_limit` off the torque setpoint is not limited; the analysis still judges against the curve. What it judges is the velocity loop's output before this limit (`torque_demand`), so a drive that sits on the limit shows as a utilisation above 1. This curve is a separate, configured limit; the voltage limit of the winding acts as well.

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
burned = clamp( E − ½ · C · U_th², 0, P_peak · dt )     the chopper switches the resistor in above the threshold
U      = sqrt( 2 · (E − burned) / C )                   but never below the bus voltage: the rectifier feeds from the line
trip   : U > dc_link_overvoltage_v
```

Stepping the energy instead of the voltage is exact for any step size. When a drive trips, all drives go off as with a Disable command, and stay off until Enable. Without the effect the DC link voltage stays at `drive.bus_voltage_v`.

Assumptions: the rectifier is an ideal source at the bus voltage; no ripple; the chopper is an ideal power limiter up to `braking_resistor_peak_w`, not a switched resistor; the resistor itself does not heat (its continuous rating is judged against the cycle's mean power); nothing is fed back into the line; a disabled drive's DC link does not discharge.

**Shared DC link.** `effects.shared_dc_link` (with `effects.dc_link`): the drives of a machine have one DC link, as with one Line Module on a SINAMICS S120. After the current loops of all drives, one step of the same energy balance runs with the sum of their powers:

```text
E = Σ ½ C U_k² − Σ P_k · dt        C_link = n · C
```

then chopper, rectifier and trip as above, once, with one resistor of `braking_resistor_peak_w`. Every drive then has the link's voltage and reports the resistor's power and the trip as its own, so the braking checks show the same shared values for each motor; `dc-link-energy` compares one drive's share of the stored energy with one drive's share of the room, which is the same ratio as for the whole link. What a braking motor feeds back, a motoring one draws: only the net power charges the capacitors. Switched on while the machine runs, the sum above puts the energies of links at different voltages together (the voltages step to the common one; no energy appears or is lost). Assumptions beyond those above: the capacitance is the sum of the drives' (a Line Module's own is not counted), and the busbars have no resistance.

### Start, disable, enable

A drive starts at rest already delivering the torque that holds the load, so a hanging axis does not sag. Disabled drives give zero torque, current and voltage. Enable restarts the loops holding where the motors stand; a warm winding stays warm.

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

Assumptions: the Z axis is a rigid body on a compliant guide; it does not bend. Only its lean in X is a degree of freedom, not in Y and not in rotation as a state of its own. The payload sits at the centre of mass used for h.

The resonance the machine reports for the input shaper (`resonance_hz`) is the lower mode of two masses in a row: the carriage on the belt's X stiffness k_b = EA/L₁ + EA/L₃ (pulleys taken as held), and the Z axis on k(z):

```text
ω⁴ − ( k_b/m_c + k/m_c + k/m_z ) · ω² + k_b · k / (m_c · m_z) = 0
```

The frame mode and the stiffness of the control loops are not in it.

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

Held, the shaft sits on a spring that reaches the brake torque at 1 mrad, with the damping c critical for the rotor on that stiffness. The torque never exceeds the brake torque: beyond it the brake slips, and what s is deflected beyond 1 mrad is forgotten within 50 µs. A brake that engages at speed therefore decelerates the rotor with brake torque over inertia until it stands, at any speed. (The brake was a LuGre contact before; its bristle damping gave many times the brake torque when it engaged at speed, and above 178 rad/s its state was too stiff for the integration step.) A released brake forgets its deflection within 1 ms. Brakes exist in the elastic model only, and on both motors (there is no separate "brake on Z"; holding Z takes both).

### Rigid reference model

`effects.belt_stretch = false`. Six states: x, z, their rates, the two LuGre states. With belt force F_k = i · (ripple(τ_k) − τ_shaft) / r per motor, reflected rotor mass m_r = i² J_rotor / r² and the gearbox loss factors λ_A, λ_B:

```text
( m_X + (λ_A + λ_B) m_r ) · a_x + (λ_A − λ_B) m_r · a_z = λ_A F_A + λ_B F_B + mesh − F_f,x + extras_x
(λ_A − λ_B) m_r · a_x + ( m_Z + (λ_A + λ_B) m_r ) · a_z = λ_A F_A − λ_B F_B + mesh − F_f,z − m_z g + extras_z
```

m_X and m_Z include both pulleys (2 J_pulley / r²), the idlers and the chain. The gearbox loses torque only on what it transmits, which is the motor torque minus what accelerates the rotor itself; that depends on the acceleration being solved for, and three passes settle it. With a loss-free gearbox this collapses to two independent axes:

```text
( m_x + 2J/r² ) · a_x = F_x − F_f,x          ( m_z + 2J/r² ) · a_z = F_z − m_z g − F_f,z
```

The rigid model has no belt stretch, slack, damping, tooth stiffness, backlash, gear stiffness, frame mode, leaning Z axis or brakes. It reports the pretension as belt tension and twice the pretension as radial load.

### Feedforward torque and static hold

The torque feedforward and the motor-limit check use the loss-free rigid model:

```text
f_x = ( m_x + 2J/r² + m_idler,x + m_chain(x) ) · a_x
f_z = ( m_z + 2J/r² + m_idler,z ) · a_z + m_z g
τ_A = r/(2i) · (f_x + f_z)         τ_B = r/(2i) · (f_x − f_z)
```

Friction is not in it. At rest each motor holds m_z g r / (2i), with opposite signs.

### Signals

Besides the signals every machine gets: `belt.1/2/3.tension`, `axis.x/z.belt_stretch` (tool position minus r/2 (a ± b)), `axis.x/z.friction`, `frame.deflection` (q), `axis.x.tool_deflection` (tip · u), `belt.slack_time`, `pulley.a/b.radial_load`, `axis.x/z.guide_load`, `gearbox.a/b.output_torque`.

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

m is the carriage plus the payload if attached, plus the moving run of the [cable chain](#cable-chain) m_chain(x) with `effects.cable_chain`; drag and m′ are the chain's, with the Stribeck speed of `friction_x`. The chain's run has weight along an inclined axis like the carriage (lifting it from 0 to x takes mass_kg_m · (length_at_zero_m · x + x²/4) · g · sin(incline)), and does not load the guide. F_process is `process.force_x_n` with `effects.process_force`. τ_brake is the [holding brake](#holding-brakes) of the T-gantry, the same component (`HoldingBrake` in `mech.rs`, as `CableChain`), on the one motor shaft; elastic model only. The whole belt runs at carriage speed, so all of its mass sits on the pulley: J_pulley += ρ · L_total · r². The idler at the far end is free and has no inertia.

Rigid model, 3 states (x, v, LuGre state), with m_r = i² J_rotor / r²:

```text
a = ( λ · F_drive + F_mesh − F_f − m g sin(incline) + F_stop + F_extra ) / ( m + J_pulley/r² + λ · m_r )
```

λ is the gearbox loss factor on the transmitted force, found in two passes.

Signals: `belt.1/2.tension`, `axis.x.belt_stretch` (x − r · a), `axis.x.friction`, `belt.slack_time`, `pulley.m.radial_load` (T₁ + T₂), `axis.x.guide_load` (m · g · cos(incline), without the chain), `gearbox.m.output_torque` (elastic: τ_gear,out; rigid: λ · (F_drive − m_r · a) · r).

`Machine::scene_point` maps x to where the scene draws the carriage: (span + x, −0.03, 0.06).

Not on the single axis: frame mode, leaning tool, idler inertia and friction, a resonance for the input shaper. In the rigid model disabled drives let it coast.

## Motion controller

Source: [`control.rs`](../crates/sim-components/src/control.rs).

The controller plans in tool coordinates and runs one drive per motor. Each profile cycle:

1. The motion under way gives position, velocity and acceleration setpoints per tool axis: a point-to-point move, the cycle, a trajectory, or an interpolated external setpoint.
2. The [input shaper](#input-shaper) spreads them over time, if on.
3. The inverse transformation gives the motor position setpoints. Motor speed and acceleration setpoints are the inverse transformation applied a moment ahead and behind (central difference).
4. The machine's rigid model gives the torque feedforward for the setpoint pose and acceleration.
5. With `effects.fieldbus_delay` the drives get what was computed `profile.fieldbus_delay_cycles` cycles ago. A fixed delay; no jitter.

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

- **Spline span:** consecutive `spline` pieces are one cubic spline through their points: the parameter u runs with the straight distance between the via points, each span is a cubic in u, and first and second derivative by u agree at every via point, so position, direction and curvature have no step there. At an end the curve takes the direction of the line or arc it joins (the first derivative is that unit direction); without one it ends with no second derivative. The derivatives at the points follow from a system with three entries per row. Each span is one piece for the timing. Distance along it is a table at 65 values of u (five-point Gauss-Legendre per part), inverted by Newton's method. Its curvature k = (p′ × p″) / |p′|³ and the change of curvature per distance are taken at 257 places; the largest of each, raised by 5 % for what lies between, give the span its limits like a transition curve: an arc of radius 1 / k_max with dk = max |dk/ds|. So a span runs no faster than its tightest place allows, and the speed changes from span to span, not within one. Where a spline joins a line or an arc the curvature steps, and the junction rule above slows the path there. Splines get no transition curves, and an arc beside one keeps its radius. A curve whose |p′| falls below 0.05 anywhere (1 on average) is refused: its via points and end directions would make it turn back on the spot.

Look-ahead then lowers the junction speeds until every piece is long enough to change from the speed at its start to the speed at its end. On each piece the speed goes by a jerk-limited ramp to the highest speed the piece has room for, stays there, and goes by a second ramp to the speed at its end. Two cycles at steady speed are kept on either side of a junction.

### Transition curves (`profile.path_tolerance_m`)

With a tolerance above zero, both planners time a changed path: where a line and an arc, or two arcs, meet in the same direction, a curve is put in whose curvature k goes evenly with the distance from that of the one piece to that of the other (a clothoid). At the speed v the acceleration towards the centre then changes at the rate v³ dk / L instead of stepping (L: length of the curve, dk: step of the curvature, 1/R for a line into an arc).

Such a curve does not fit between a line and a circle that touch. It fits when the circle is moved in by

```text
e = L² / (24 R)          first term; the code solves the exact length by halving
```

So every blended arc keeps its centre and gets the radius R − e, and the curves replace about L/2 of the pieces on either side of each junction. The planned path is nowhere further from the given one than the largest e, which `path::deviation` reports. One L serves the whole path:

```text
L = min( max over junctions of v³ dk / (0.4 j),  sqrt(24 · tolerance · R_min) )
```

with v the speed limit of the slower of the two pieces and j the smallest jerk limit of any axis. With every arc moved in by its e, the junctions come out with curves of about this one L: two arcs turning the same way are apart by the difference of their e (the tighter one moves in more), two turning opposite ways by the sum. Each curve is then fitted exactly: its length is the one for which the circles that fit its two ends are as far apart as the real ones, and it is turned and placed so that they coincide. A point on the curve is the direction added up along it (Simpson's rule, 64 intervals).

Limits: an arc at a corner or at an end of the path has to end where it is given to, so it keeps its radius and gets no curve towards a line; where a curve would not fit (a wider arc moved in beside a tighter one that was not) both arcs stay as given; pieces too short for their curves halve L for the whole path, up to eight times, and then the path stays as given. What a curve takes of an arc is the angle from the arc's own end to where the curve meets it; curves that together take more than the arc has are too long, and so is a curve that turns half a circle or more (a tolerance that is large against a tight radius). The time-optimal planner times the same path. The length L is worked out with the speeds of the axis limits for both planners, so a time-optimal plan that could pass a junction faster finds its curves shorter than it would need and slows down on them.

### As fast as the motors allow (`--time-optimal`)

The timing within the axis limits (above), with the speed and the acceleration limits of each piece taken from the motors instead of from `[axis.*]`. The jerk limits stay those of the axes, and so do the look-ahead, the jerk-limited ramps, the speed at a junction and the stop at a corner.

At nine poses along each piece the machine's rigid model gives, per motor, the angle it turns per distance of each axis (a row w of the inverse transformation's derivative) and the torque it needs per acceleration of each axis (a row g), plus the torque T_hold that holds the tool there. Every pose gives two bounds per motor:

```text
| w · velocity |      ≤  rated speed
| g · acceleration |  ≤  profile.torque_reserve · peak torque − | T_hold |
```

The torque-speed curve is level up to the rated speed, so its peak torque holds for the whole plan. The rest of the curve is for friction and elasticity, which the rigid model leaves out. Taking | T_hold | off for both directions costs the plan a little where gravity would help.

- **Line** in the direction d: speed ≤ rated speed / | w · d | and acceleration ≤ bound / | g · d |, the smallest over all rows. That is exact: on a line the plan uses the motors' speed and torque in full.
- **Arc and transition curve:** the direction turns, so the bounds have to hold in any direction: speed ≤ rated speed / | w | and acceleration ≤ bound / | g |. These two and the smallest jerk limit are then shared out between the directions along the path and towards the centre as for the axis limits (3 : 4 : 5, above). That is safe and leaves a part of the limits unused.

So every planned pose keeps each motor's torque from the rigid model within the reserve, each motor's speed within the rated speed and each axis's jerk within its limit. The tests check that on the planned poses by differences over one cycle, with a margin of 0.1 %.

Where the holding torque alone is above the reserve at a pose of the path, the path is refused (`error-path-torque`).

Assumptions and limits: the rigid model, so no friction and no elasticity (that is what the reserve is for); the velocity and acceleration limits of the axes play no part; one set of limits per piece, so the plan is not the fastest one possible: on an arc the shares leave a part of torque, speed and jerk unused, and a piece has one top speed however its direction turns; without a path tolerance it crawls into an arc as the plan within the axis limits does; belt tension and the other ratings are not limits of the plan; the payload is as the machine has it when planning. Until 12 November 2026 this was the classic time-optimal parameterisation on a grid of points, without a jerk limit and smoothed by a moving mean (`profile.path_smoothing_s`, which no longer exists). It was faster (0.69 s against 1.43 s on the line, arc, line of the user guide) and passed the motors' limits on arcs, many times over on short ones.

## Input shaper

Source: `shape` in [`control.rs`](../crates/sim-components/src/control.rs).

`effects.input_shaping`: a ZVD shaper on the position, velocity and acceleration setpoints of all axes. The shaped setpoint is the sum of three copies of the plan: now, half a damped period ago and a whole period ago.

```text
K       = exp( −ζ · π / sqrt(1 − ζ²) )
T_half  = 1 / ( 2 · f · sqrt(1 − ζ²) )
shaped(t) = ( plan(t) + 2K · plan(t − T_half) + K² · plan(t − 2 T_half) ) / (1 + K)²
```

f `shaper_frequency_hz`, ζ `shaper_damping`. The delays are interpolated between profile cycles. The weights add up to one, so once the plan has stood still for a whole period the setpoint is the plan exactly. Where a cycle segment has a dwell or switches the payload, both wait until the shaped setpoint has arrived, ceil(2 · T_half / cycle) profile cycles after the plan; the process force starts with the dwell. Segments without a dwell and without a gripper action go on with the plan. A plain move (`Move`) is no longer busy when its plan has arrived.

With `effects.shaper_follows_pose` the half period is that of the resonance the machine reports for the planned pose. Only the T-gantry reports one. Its resonance also depends on the payload: when the gripper closes at the start of a dwell the period grows a little, and the setpoint leaves the target once more by less than a micrometre.

A changed shaper setting in a running simulation (the switch, the frequency, the damping, following the pose) is held back until the plan stands still and the shaper's store holds nothing but that pose. Then the setpoint is the plan with the old shaper and with the new one, and does not step. Taken at speed, the three copies of the plan were suddenly read at other delays, or two of them dropped, and the setpoint jumped by up to 16 mm on the T-gantry's X axis at 0.8 m/s.

When `profile.cycle_s` is changed in a running simulation, the stored plan of the shaper is read off again at the new spacing and every stored cycle count of the motion under way is converted, so the setpoint does not jump. The fieldbus queue keeps its count of cycles.

Limits: one resonance; the same shaper on every axis.

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
| Winding rise (settled) | With q = ϑ_rated / (1 + 0.0039 ϑ_rated) · (I_rms / I_rated)²: ϑ = q / (1 − 0.0039 q). The copper loss with the resistance at that temperature, against the loss at the rated current `motor.rated_current_a`. Infinite if 0.0039 q ≥ 1 |
| Energy in | Σ P · dt over the steps with P ≥ 0 |
| Energy regenerated | −Σ P · dt over the steps with P < 0 |
| Peak regenerated power | max(−P) |
| Largest regeneration | Most energy fed back in one unbroken stretch of negative power |
| Resistor energy, peak power | Σ and max of the simulated braking resistor's power (0 without `effects.dc_link`) |
| DC link peak voltage | max of the DC link voltage |

For the machine, sampled once per millisecond:

| Quantity | Definition |
| --- | --- |
| Belt tension, lowest and highest | Over all signals `belt.<n>.tension` |
| Belt slack time | Increase of `belt.slack_time` over the pass. Counted every integration step, so it also catches what falls between the samples |
| Radial load peak | max over all signals `pulley.<id>.radial_load` |
| Guide load | Per guide the peak and the cubic mean over time, cbrt(Σ load³ / samples). Reported for the guide with the highest cubic mean |
| Guide life | L10 = (C / P)³ · 50 km with C = `ratings.guide_dynamic_load_rating_n` and P that cubic mean |
| Path error | Distance from the tool position to the polyline of the setpoints of the last 0.5 s (the nearest of its pieces). Peak and RMS. The tool position is the machine's tool state: on the T-gantry the tool tip over the floor, frame deflection included |
| Tracking error | Distance from the tool to the setpoint of the same moment. Peak |
| Dwell error | In the second half of a dwell: distance from the tool to the planned target. Peak. With input shaping the dwell starts when the shaped setpoint has arrived |

Notes:

- Path error looks only at the setpoints of the last half second: the piece of the path the tool can be near.
- The setpoints used are the controller's (after shaping, before the fieldbus delay), so a fieldbus delay shows in the tracking error.
- The guide life is the catalogue formula and nothing more: no load factors for shock, temperature or hardness, no moments on single blocks, the mean taken over time and not over distance, and a guide load that is itself an estimate from the rigid model. It is not validated.

## Verdicts

Source: `verdicts` in [`analysis.rs`](../crates/sim-components/src/analysis.rs). Each verdict is a value against a limit; it passes if value ≤ limit, except where noted. A rating that the config leaves out gives no verdict.

Per motor:

| Check | Value | Limit |
| --- | --- | --- |
| `motor-peak` | Utilisation | 1 |
| `motor-thermal` | Settled winding rise | `motor.rated_temperature_rise_k` |
| `pulley-torque` | Peak torque at the gearbox output | `ratings.pulley_max_torque_nm` |
| `gearbox-continuous` | Cubic mean of the torque at the gearbox output, weighted with the speed | `ratings.gearbox_rated_torque_nm` |
| `gearbox-input-speed` | Peak speed in r/min | `ratings.gearbox_max_input_speed_rpm` |
| `gearbox-peak` | Peak torque at the gearbox output | `ratings.gearbox_max_torque_nm` |
| `dc-link-energy` | See below | room = ½ C (U_th² − U_bus²). A threshold at or below U_bus is refused as out of range |
| `braking-resistor-continuous` | See below | `ratings.braking_resistor_continuous_w` |
| `braking-resistor-peak` | See below | `ratings.braking_resistor_peak_w` |
| `dc-link-overvoltage` | Peak DC link voltage; only with `effects.dc_link` | `ratings.dc_link_overvoltage_v` |

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

Gearbox and pulley torques are those at the gearbox output, the signals `gearbox.<motor>.output_torque`, looked at every simulation step: peak, and the cubic mean cbrt( Σ |τ_out|³ · |ω| / Σ |ω| ). The rotor's own acceleration torque is not in them (it never passes through the gearbox), and the efficiency is: driving, the output carries η times what the gear's spring transmits, driven back 1/η times. The motor torque times the ratio, which these checks used before, reads too high while the motor accelerates its own rotor and too low while the load drives the gearbox back. On a machine that does not report the signal the three checks are not made. In the elastic model the peak includes the ringing of the gear's spring and the knocks of its backlash.

## Other analyses

| Analysis | Definition |
| --- | --- |
| Error bands | The cycle is run with the config as it is and with each parameter of `[bounds]` at its low and at its high value, one at a time. A band is the envelope of the verdict's value over all runs; the worst case passes only if every run passes. The runs share the processor cores |
| Sampled error bands | `--bands --samples N`: N runs with every parameter of `[bounds]` between its bounds at once, as a Latin hypercube: the range of each parameter is cut into N equal parts, each part is used in exactly one run at a random place inside it, and each parameter has its parts in a random order of its own. Random numbers from the simulator's own generator and a seed, so a call repeats. Reported per check: the envelope, the worst case, and the share of the runs (the config as it is among them) that pass; a run in which a check does not come up has passed it. It finds what two or more values do together; it does not put a value exactly on a bound, and N runs do not find the worst of the 2ⁿ corners |
| Sensitivity | The same runs, reported as relative changes of peak torque and RMS torque (each of the motor it changes most) and of the cycle time. Ranked by the largest of these six changes |
| Optimiser | Starts from the config. Candidates: toggle corner blending, switch input shaping off where it is on, raise velocity, acceleration or jerk of all axes by 1.5 or 1.2. A candidate without input shaping that fails `settling` and nothing else is run again with the shaper, and with the shaper following the pose where the machine reports a resonance. The first of these that is shorter by more than 0.1 ms, has no motor-limit violation and passes every verdict is taken; repeat until none is |
| Frequency sweep | No control. A sine torque on one motor on top of the holding torques; that motor's speed is correlated with sine and cosine over whole periods (at least 0.2 s) after settling (at least 0.3 s). Gain in rad/s per N m |
| Step response | The machine settles for 0.05 s, then the position setpoint of one axis is offset by the step size, without feedforward. Rise time 10 % to 90 %; overshoot as a fraction; settling to within 2 % |
| Closed-loop response | A sine offset on the position setpoint, measured at the tool. Bandwidth: where the gain first falls below 1/√2, interpolated on the logarithmic frequency axis |
| Motor search | See the [user guide](user-guide.md#find-the-lightest-motor). Velocity gain of a candidate = the config's gain × sqrt( ratio of inertia seen × ratio of rotor inertias ), rotor inertia with the gearbox's own, against the config's own numbers. The same rule applies wherever a part is named (`[parts]`, `--motor`, `--gearbox`). "Tightest check" leaves out `dc-link-overvoltage` and, with a braking resistor in use, `dc-link-energy` while they pass |

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
| [`sim-components/src/control.rs`](../crates/sim-components/src/control.rs), `Controller::update` (line 482) | A disabled drive's winding keeps its temperature instead of cooling | When long stops matter |
| [`sim-components/src/control.rs`](../crates/sim-components/src/control.rs), `Controller::shape` (line 691) | While the shaper's period changes (`shaper_follows_pose`), the speed and acceleration setpoints are the delayed plan's, not the exact rates of the shaped position. Off by the rate of the delay, a few percent in a Z move; they are feedforward only | Shape the position alone and difference it, if that matters |
| [`sim-components/src/mech.rs`](../crates/sim-components/src/mech.rs), `Lugre::eval` (line 273) | Bristles that would settle faster than 1e5 1/s settle at that rate, with their force scaled so that sliding still takes the level | Step the friction state implicitly, if what bristles do within 10 µs ever matters |

## Other simplifications

Not marked as shortcuts in the code, but worth knowing. Most are stated in the sections above.

| Area | Simplification |
| --- | --- |
| Motor | One equivalent winding. No phases, PWM, field weakening or iron loss |
| Drive | Delays are whole cycles; no jitter. The same settings for every drive. The filters are generic, not a vendor's |
| Thermal | One thermal mass per winding. No gearbox, guide or belt temperature. `effects.temperature` is two factors set by hand |
| DC link | One per drive, or one for all with the sum of their capacitances; no feedback into the line, no resistor heating, no Line Module capacitance or busbar resistance |
| Belt | Mass lumped on the pulleys. No string vibration, no tooth jump, no wear |
| Friction | Guide loads from the commanded acceleration of a loss-free rigid machine, not from the true one. No moments on single guide blocks |
| Gearbox | Constant efficiency, constant stiffness, hard dead zone |
| Frame | One mode, in X only |
| Z axis | Rigid body leaning on a compliant guide, in X only |
| Stop | No fast stop ramp, no emergency stop |
| Brakes | Elastic model only. A spring up to the brake torque, then slipping with it; no wear, no engagement delay |
| Cycle | The payload switches as a step on arrival; no gripper time, no swinging payload |
| Feedforward | Rigid model with inertia and gravity. No friction feedforward, so some following error stays while moving |
