# Single belt axis, pick-and-place cycle (synthetic)

**Not a measurement.** The simulator's `single-axis` machine running its default cycle for 6 s with the
payload off, recorded at 1 ms. The columns are written the way a drive records them, so that the format,
the mapping and the checker have something to run on:

| Column | Unit | Made from |
| --- | --- | --- |
| `time` | ms | the run's time |
| `position setpoint` | encoder counts, 2^20 per motor turn | `motor.m.angle_setpoint`, rounded |
| `position actual` | encoder counts | `motor.m.angle`, rounded down |
| `speed actual` | 1/min | `motor.m.speed` |
| `torque actual` | N m | `motor.m.torque` plus normal noise of 0.003 N m |
| `torque setpoint` | N m | `motor.m.torque_command` |

Semicolons and decimal commas, as a German Windows writes CSV.

```text
sim-cli identify --trace trace.csv --map map.toml
```

gives a total inertia of 1.049e-4 kg m² ± 0.1 % and Coulomb friction of 0.0848 N m. These are the
simulator's own values, reflected to the motor, with its gearbox losses and belt included.
