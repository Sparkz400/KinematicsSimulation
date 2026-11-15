# Ball screw axis with a 400 kg table (ISW Stuttgart, measured)

**Measured on a real machine.** The first measured data of this project, taken from a public source:

> C. Hinze, L. Zeh: *Identification data for plant and friction identification*. DaRUS, V1, 2022.
> https://doi.org/10.18419/darus-3003. Institute for Control Engineering of Machine Tools and Manufacturing Units
> (ISW), University of Stuttgart. Licence: [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/).

The data here are three of that dataset's recordings, converted (below) and cut to the motor's signals. Use them under
CC BY 4.0: name the authors and the DOI above. ISW has not reviewed this conversion.

## Machine

From the source's README and metadata:

- A table of 400 kg, 0.72 m of travel, on a ball screw Steinmeyer 3526/40.40.6,3E, 40 mm, spindle 1800 mm long.
- Motor SIEMENS 1FT7085-7WF71 (water cooled) on a SINAMICS S120; encoder AM2048S/R on the motor, a Heidenhain
  LS 187C glass scale on the table.
- A Rexroth linear direct drive under the table, used to push disturbance forces in the source's other recordings;
  in these three it applies no force (`F_des_LDD` is zero), its magnets' pull is compensated by a lookup table.
- Speed loop (PI) and position loop (P, 35 1/s) on a dSPACE DS1006; the drive gets a torque setpoint.

Not stated by the source, our estimates (machine.toml says the same where it uses them):

- **The lead, 40 mm.** Read from the article number: Steinmeyer numbers give lead.diameter (its page for the
  3526/25.40.7,5.3NE lists lead 25 mm, diameter 40 mm). The recordings give the motor's position only as table
  travel, converted by the source with its lead, so the lead cannot be read from them. The torque per acceleration
  fits 40 mm (moved mass 492 kg against 400 kg of table, the rest rotor and screw) but would fit 25 mm with a heavier
  rotor side too.
- Horizontal mounting; whether the motor has a brake (machine.toml takes the rotor without).

## Files

| File | Source file | Test | What |
| --- | --- | --- | --- |
| `s-profile-v300.csv` | `Validation/SProfile/SPP_v300_xS10_xE700_a1000.mat` | cycle | 10 mm to 700 mm and back: 0.3 m/s, 1 m/s², 100 m/s³, 6.2 s |
| `sine-a100-v100.csv` | `Validation/SineTrajectory/Sine_xS250_t20_A100_v100_f0-1.mat` | friction-map | sine of ±100 mm about 250 mm at 0.1 Hz, 21 s |
| `constant-speed-v100.csv` | `const_v_100.mat` | friction-map | 0 to 700 mm and back at 0.1 m/s, 15 s |
| `map.toml` | | | the columns at the motor shaft |
| `machine.toml` | | | the bench as the simulator's `screw-axis` machine, every value marked with its origin |

Columns: `time [s]`; `position setpoint [m]` (`traj_KGT[0]`, the table's planned position); `position actual [m]`
(`x_meas_motor`, the motor encoder as table travel); `speed actual [m/s]` (`v_meas_motor`); `torque actual [Nm]`
(`M_KGT_act`, the motor torque the S120 reports from its current). The map turns metres into motor radians through the
lead (`per_rev = 0.04`).

## Conversion

The source files are MATLAB v7.3 structs written by dSPACE ControlDesk, signals at 1 ms and at 4 kHz.
`tools/darus_ball_screw_to_csv.py` writes the CSV: the 1 ms grid, and of the 4 kHz signals every fourth sample (the
grids meet there; the script checks it). Nothing is filtered or averaged; the operator's user name and the rest of the
signals are left out. With the source files downloaded from DaRUS (no account needed):

```text
python -I tools/darus_ball_screw_to_csv.py SPP_v300_xS10_xE700_a1000.mat s-profile-v300.csv
python -I tools/darus_ball_screw_to_csv.py Sine_xS250_t20_A100_v100_f0-1.mat sine-a100-v100.csv
python -I tools/darus_ball_screw_to_csv.py const_v_100.mat constant-speed-v100.csv
```

## Use

The simulator replays a trace from rest at `start.pose`; each file starts elsewhere:

```text
sim-cli identify --trace s-profile-v300.csv --map map.toml
sim-cli replay screw-axis --config machine.toml --set "start.pose=[0.01003171]" --trace s-profile-v300.csv --map map.toml
sim-cli fit screw-axis --config machine.toml --set "start.pose=[0.01003171]" --trace s-profile-v300.csv --map map.toml
```

Start poses: `s-profile-v300` 0.01003171, `sine-a100-v100` 0.25002924, `constant-speed-v100` 0.00095521. What came out
is in docs/validation.md, "Against measured data".

## What is odd

- The torque is quantised in steps of 3.05 mN m and sampled every 1 ms from a 4 kHz record without filtering.
- The sine starts in motion: the setpoint jumps to 0.063 m/s at the first row and the measured torque reaches 34 N m in
  the first 10 ms. A replay starts at rest and makes a larger jolt there.
- After the S-profile's last move the axis holds about −1.5 N m at standstill (friction holding the integrator's
  torque); a model without stiction gives zero.
