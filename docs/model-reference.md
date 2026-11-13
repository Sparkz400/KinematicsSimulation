# Model reference

The equations the simulator solves, taken from the code and its comments. Each section names the source file. Config keys are written as `table.key`; their units are in the [config reference](config-reference.md).

- [Structure of one time step](#structure-of-one-time-step)
- [Integrator](#integrator)
- Components: [belt](#belt-sections), [guide friction](#guide-friction-lugre), [motor shaft](#motor-shaft-friction-ripple-cogging), [tooth meshing](#tooth-meshing), [gearbox](#gearbox), [hard stops](#hard-stops), [drive and motor](#drive-and-motor)
- Machines: [T-gantry](#t-gantry), [single axis](#single-axis)
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

So every blended arc keeps its centre and gets the radius R − e, and the curves replace about L/2 of the pieces on either side of each junction. The planned path is nowhere further from the given one than the largest e, which `path::deviation` reports. One L serves the whole path:

```text
L = min( max over junctions of v³ dk / (0.4 j),  sqrt(24 · tolerance · R_min) )
```

with v the speed limit of the slower of the two pieces and j the smallest jerk limit of any axis. With every arc moved in by its e, the junctions come out with curves of about this one L: two arcs turning the same way are apart by the difference of their e (the tighter one moves in more), two turning opposite ways by the sum. Each curve is then fitted exactly: its length is the one for which the circles that fit its two ends are as far apart as the real ones, and it is turned and placed so that they coincide. A point on the curve is the direction added up along it (Simpson's rule, 64 intervals).

**An arc with one end fixed.** An arc at a corner, at an end of the path, or where a spline joins it has to end at that end where it is given to, in the given direction. If its other end meets a line in its direction, its circle shrinks by s and its centre moves towards the fixed end by s: the new circle passes through the fixed end in the same direction (it touches the old one there from inside), and is apart from the line by s (1 − cos φ), φ being the angle the arc turns through. That gap is made e = L² / (24 R), so s = e / (1 − cos φ), and the curve towards the line is fitted as above. On the arc the new circle is at most r − sqrt(r² − 2 s (r − s)(1 − cos φ)) from the old one, about e; the larger of the two is reported. This is done for arcs of up to half a turn whose radius stays above half of what it was; for a quarter turn s = e, and the numbers are those of a free arc (`an_arc_at_an_end_or_a_corner_gets_its_transition_curve`: 15.49 mm, 0.3957 m/s into the arc instead of 0.0707 m/s at 0.1 mm).

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

`effects.input_shaping`: a ZVD shaper on the position, velocity and acceleration setpoints of all axes. The shaped setpoint is the sum of three copies of the plan: now, half a damped period ago and a whole period ago. With `profile.shaper2_frequency_hz` above 0 a second ZVD shaper (its damping `shaper2_damping`) is convolved with the first: nine copies, at the sums of the two shapers' delays with the products of their weights. For an undamped mode at f a ZVD shaper at f_s with no damping leaves |H| = cos²(π f / (2 f_s)) of the vibration, two convolved the product of theirs.

```text
K       = exp( −ζ · π / sqrt(1 − ζ²) )
T_half  = 1 / ( 2 · f · sqrt(1 − ζ²) )
shaped(t) = ( plan(t) + 2K · plan(t − T_half) + K² · plan(t − 2 T_half) ) / (1 + K)²
```

f `shaper_frequency_hz`, ζ `shaper_damping`. The delays are interpolated between profile cycles. The weights add up to one, so once the plan has stood still for a whole period the setpoint is the plan exactly. Where a cycle segment has a dwell or switches the payload, both wait until the shaped setpoint has arrived, ceil(2 · T_half / cycle) profile cycles after the plan (with a second shaper ceil(2 · (T_half + T_half,2) / cycle)); the process force starts with the dwell. Segments without a dwell and without a gripper action go on with the plan. A plain move (`Move`) is no longer busy when its plan has arrived.

With `effects.shaper_follows_pose` the half period of the first shaper is that of the resonance the machine reports for the planned pose; the second stays at its frequency. Only the T-gantry reports one. Its resonance also depends on the payload: when the gripper closes at the start of a dwell the period grows a little, and the setpoint leaves the target once more by less than a micrometre.

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
- The lives (`pulley_bearing_life_h`, `gearbox_life_h`, `guide_life_h` in `analysis.rs`):

  | Life | Formula | Source |
  | --- | --- | --- |
  | Pulley shaft bearing | L10h = (C / P)³ · 10⁶ / (60 n): C = `ratings.pulley_bearing_load_rating_n`, P the equivalent radial load above, n the pulley's mean speed in r/min (the motor's over `gearbox.ratio`) | ISO 281 basic rating life of a ball bearing (as SKF and Schaeffler print it); Siemens uses the same form for its gearboxes' output bearings, L10h = 30000 h · (F_R,eq allowed / F_R,eq)³ |
  | Gearbox planet bearings | L10h = 20000 h · (n2B / n2) · (M2N / M2eq)^(10/3): M2N = `ratings.gearbox_rated_torque_nm`, n2B = `ratings.gearbox_reference_speed_rpm`, n2 the output's mean speed, M2eq the gearbox life torque above | Siemens, configuration manual "Planetary gearboxes for S-1FK2 and S-1FT2 servomotors", 01/2023, section 6.3.3 |
  | Guide | L10 in km over the mean speed of its axis | The catalogue formula above |

  One bearing per pulley shaft takes all of the radial load (a config can fold a pair into one C, as the EXCT-30's does); no axial load, no life factors a1 or a_ISO (lubrication, contamination). The gearbox's own gearing (fatigue only above M2N, by Siemens' damage sum over overload partial cycles) is not computed; `gearbox-continuous` and `gearbox-peak` judge the torques. The motors' own bearings are not estimated: in all three configs the belt does not pull on the motor shaft (a gearbox or Festo's drive head with a coupling is between), and Siemens gives no load rating for them, only allowable radial forces for 25000 h (S210 operating instructions 09/2022, 12.1.12).
- The belt's teeth (`belt_tooth_utilisation`): no belt maker publishes a life in hours or load cycles, or a Wöhler curve, for polyurethane steel-cord belts (Megadyne's MEGALINEAR handbook, Gates' polyurethane belt catalogue and BRECO's data sheets were read; Optibelt's could not be reached). They size by the tooth load and the cord tension with a service factor, so this is a rating, not a life: per motor the highest over the cycle of τ_out / r · c_s / (F_spec(n) · b / 10 · min(z_m, 12)), with r = `geometry.pulley_radius_m`, c_s = `ratings.belt_service_factor`, F_spec the allowed specific tooth load at the pulley's speed n (`ratings.belt_tooth_load_n_cm` over `belt_tooth_load_rpm`, interpolated, held beyond the ends; each speed band taken at its upper speed), b = `ratings.belt_width_mm`, z_m = `ratings.belt_teeth_in_mesh`. Megadyne's method (handbook 02.2026, page 9). The cords are judged by `belt-tension`.
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
| `gearbox-thermal` | With `effects.part_heating`: where the hottest gearbox settles on the cycle, R_th × its mean loss (`CycleLoad::part_rises`, see [thermal model](#thermal-model-and-view)); not judged on a direct drive, which has no gearbox | `thermal.gearbox_max_rise_k` |
| `guide-thermal` | With `effects.part_heating`: where the hottest guide settles on the cycle | `thermal.guide_max_rise_k` |

Without `effects.part_heating` the two thermal checks are not judged, as a rating left out is not.

Gearbox and pulley torques are those at the gearbox output, the signals `gearbox.<motor>.output_torque`, looked at every simulation step: peak, and the cubic mean cbrt( Σ |τ_out|³ · |ω| / Σ |ω| ). The rotor's own acceleration torque is not in them (it never passes through the gearbox), and the efficiency is: driving, the output carries η times what the gear's spring transmits, driven back 1/η times. The motor torque times the ratio, which these checks used before, reads too high while the motor accelerates its own rotor and too low while the load drives the gearbox back. On a machine that does not report the signal the three checks are not made. In the elastic model the peak includes the ringing of the gear's spring and the knocks of its backlash.

## Other analyses

| Analysis | Definition |
| --- | --- |
| Error bands | The cycle is run with the config as it is and with each parameter of `[bounds]` at its low and at its high value, one at a time. A bound on a value that only an effect that is off uses (`[thermal]` without `effects.part_heating`, `ratings.line_module_efficiency` without `effects.regenerative_supply`), or that no cycle uses (`ratings.dc_link_discharge_time_constant_s`, which acts only while the drives are disabled), gets no runs, here and in the sensitivity. A band is the envelope of the verdict's value over all runs; the worst case passes only if every run passes. The runs share the processor cores |
| Sampled error bands | `--bands --samples N`: N runs with every parameter of `[bounds]` between its bounds at once, as a Latin hypercube: the range of each parameter is cut into N equal parts, each part is used in exactly one run at a random place inside it, and each parameter has its parts in a random order of its own. Random numbers from the simulator's own generator and a seed, so a call repeats. Reported per check: the envelope, the worst case, and the share of the runs (the config as it is among them) that pass; a run in which a check does not come up has passed it. It finds what two or more values do together; it does not put a value exactly on a bound, and N runs do not find the worst of the 2ⁿ corners |
| Worst corner | `--bands --worst` (`analysis::verdict_worst`): per verdict a search among the 2ⁿ corners. (1) The one-at-a-time runs above. (2) The corner where each parameter is at the bound that made this verdict worse alone (at its high bound where it moved the verdict not at all); for a verdict that is monotone in each parameter and whose effects do not depend on each other, that is the worst corner. (3) Up to `WORST_PASSES` = 3 passes: every parameter that moved the verdict alone is flipped to its other bound, one at a time; the flips that made it worse are taken together, and the worse of that corner and the best single flip becomes the corner if it is worse than before; a pass that finds nothing worse ends it. "Worse" is the margin, value − limit (limit − value for `belt-slack` and the lives, `AT_LEAST`), not the value, because some limits are bounded too (DC link capacitance and threshold). `dc-link-energy` is the exception: it fails only where the braking resistor is needed and has no rating, and with a resistor the link fills up to its room at every corner that brakes hard (margin 0 at all of them), so it is ranked by the energy in the capacitors, a failing run above any passing one. A corner is run once for all verdicts that land on it; reported per verdict is the worst at any run, the one-at-a-time runs and the config as it is included (so the worst is never better than the bands), with its value, limit, result and where the parameters were (a parameter left as configured is at neither bound). Cost: 2n + 1 runs, plus per pass up to the number of parameters per verdict; on the EXCT-30 (63 bounded values, 24 checks) about 2400 cycles in about 47 s on all cores, on the single axis (14) about 300 in 3.5 s. A search, not a proof: a worst case that no chain of single flips from the sign corner reaches, or one inside the bounds (a resonance hit by a stiffness between them), can be missed |
| Sensitivity | The same runs, reported as relative changes of peak torque and RMS torque (each of the motor it changes most) and of the cycle time. Ranked by the largest of these six changes |
| Optimiser | Starts from the config. Candidates: toggle corner blending, switch input shaping off where it is on, raise velocity, acceleration or jerk of all axes by 1.5 or 1.2. A candidate without input shaping that fails `settling` and nothing else is run again with the shaper, and with the shaper following the pose where the machine reports a resonance; one with the shaper that still fails `settling` alone, with a second shaper at `profile.shaper2_frequency_hz`, or where that is 0 at the resonance the machine reports at the start pose if that is more than 10 % off `shaper_frequency_hz`. Where a second shaper is on, switching it off is a candidate too. The first of these that is shorter by more than 0.1 ms, has no motor-limit violation and passes every verdict is taken; repeat until none is |
| Frequency sweep | No control. A sine torque on one motor on top of the holding torques; that motor's speed is correlated with sine and cosine over whole periods (at least 0.2 s) after settling (at least 0.3 s). Gain in rad/s per N m |
| Step response | The machine settles for 0.05 s, then the position setpoint of one axis is offset by the step size, without feedforward. Rise time 10 % to 90 %; overshoot as a fraction; settling to within 2 % |
| Closed-loop response | A sine offset on the position setpoint, measured at the tool. Bandwidth: where the gain first falls below 1/√2, interpolated on the logarithmic frequency axis |
| Motor search | See the [user guide](user-guide.md#find-the-lightest-motor). Velocity gain of a candidate = the config's gain × sqrt( ratio of inertia seen × ratio of rotor inertias ), rotor inertia with the gearbox's own, against the config's own numbers. The same rule applies wherever a part is named (`[parts]`, `--motor`, `--gearbox`). "Tightest check" leaves out `dc-link-overvoltage` and, with a braking resistor in use, `dc-link-energy` while they pass |

## Effect views

Sources: `losses` and `view_signals` (the machine's `loss_powers` and `effect_signals`) in each machine's `lib.rs`; `Lugre::steady`, `Lugre::split` and the `loss_power` functions in [`mech.rs`](../crates/sim-components/src/mech.rs); `EnergyBudget`, `Tally`, `Comparison` and `compare_effect` in [`analysis.rs`](../crates/sim-components/src/analysis.rs); `HeatedPart` and `Sim::scene` in [`sim.rs`](../crates/sim-components/src/sim.rs); the conventions `TORQUE_PARTS`, `LOSS_PREFIX`, `ENERGY_PARTS`, `FRICTION_SIGNALS`, `THERMAL_KINDS`, `TEMPERATURE_PREFIX` and the methods `friction_contacts`, `friction_curve`, `belt_sections` and `thermal_parts` in [`machine.rs`](../crates/sim-core/src/machine.rs).

A machine tells the effect views what it knows through signals with fixed names and four methods with defaults; a machine that has none of them still runs, and the views say nothing about it. Front ends and the analysis find the signals by their names only.

**Cost.** The torque breakdown needs the machine's accelerations, one more evaluation of its derivatives. So the machine gives its signals in two parts: `base_signals` (tensions, guide loads, gearbox torques: what the verdicts look at every step) and `effect_signals` (the signals of the views, after them), and the loss powers alone as `loss_powers`, without derivatives. `Sim::sample` and `run` take both parts; `Sim::step` takes only the loss powers, and only with `effects.part_heating`; the analysis adds up breakdown and budget only where they are asked for (`analyse_cycle_with(sim, views)`: the app's analysis and `sim-cli analyze --breakdown/--budget` ask, bands, sensitivity, optimiser and motor search do not). Every number is the same either way (`the_cheap_signals_and_the_lean_analysis_change_no_number`).

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
       + fed back into the mains, line module loss         with effects.regenerative_supply
       + every power.* · dt
       + ΔE windings (½ L i² of all motors)
       + ΔE capacitors (with the DC link)
       + Δenergy.kinetic, Δenergy.potential, Δenergy.elastic
       + carried off with the payload = −(sum of the steps of Machine::energy when the payload switched)
unaccounted = supply − rows
electrical  = supply − copper − resistor − mains − line module − ΔE windings − ΔE capacitors − Σ W   (W: shaft work)
```

The rectifier's energy is counted on its own, every step, as what lifts the capacitor energy back to that of the bus voltage. A payload picked up brings its kinetic and potential energy into the machine and one put down takes it away; the simulation adds up these steps (`Sim::payload_energy`). The stores are read after the last step counted: a cycle's measured pass leaves out the step in which it begins and the one in which the next begins, flows and stores alike. (Until 12 November the stores were read one step later than the flows ended, which on the loss-free elastic configs put up to 0.8 % into "not accounted for", taken then for backlash knocks.)

**The electrical side is checked on its own** (`EnergyBudget::electrical_unaccounted`): the supply against copper loss, resistor, mains, line module, the windings' and the capacitors' stored energy and the work the torques did on the shafts, integrated with the windings (`EnergyBudget::shaft_work`). Within `ELECTRICAL_TOLERANCE` = 1e-6 of the supply on every machine and config, the loss-free ones included: measured at most 3.2e-7 (single axis, loss-free inclined), 1e-12 on the default configs; it falls by 15 with each halving of the integration step, as fourth order does. With `effects.back_emf` off the shaft work comes from nowhere and this does not close; no shipped config has it off. What `unaccounted` holds beyond it is the machine's own balance.

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
| Thermal | One thermal mass per winding, gearbox and guide (`effects.part_heating`); no belt temperature. Nothing feeds back: `effects.temperature` is two factors set by hand |
| DC link | One per drive, or one for all with the sum of their capacitances; feedback into the line only through an ideal line module of one efficiency (`effects.regenerative_supply`); no resistor heating, no Line Module capacitance or busbar resistance; the bleeder draws only while the drives are disabled |
| Belt | Mass lumped on the pulleys. No string vibration, no tooth jump, no wear |
| Friction | Guide loads from the commanded acceleration of a loss-free rigid machine, not from the true one. No moments on single guide blocks |
| Gearbox | Constant efficiency, constant stiffness, hard dead zone |
| Frame | One mode, in X only |
| Z axis | Rigid body leaning on a compliant guide, in X only |
| Stop | No fast stop ramp, no emergency stop |
| Brakes | A spring up to the brake torque, then slipping with it; no wear, no engagement delay. In the rigid model the spring holds the whole machine with the damping meant for the rotor |
| Cycle | The payload switches as a step on arrival; no gripper time, no swinging payload |
| Feedforward | Rigid model with inertia and gravity. No friction feedforward, so some following error stays while moving |
