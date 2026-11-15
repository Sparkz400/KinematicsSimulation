# Ball screw axis with a 400 kg table (ISW Stuttgart, measured)

**Measured on a real machine.** The first measured data of this project, taken from a public source:

> C. Hinze, L. Zeh: *Identification data for plant and friction identification*. DaRUS, V1, 2022.
> https://doi.org/10.18419/darus-3003. Institute for Control Engineering of Machine Tools and Manufacturing Units
> (ISW), University of Stuttgart. Licence: [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/).

The data here are three of that dataset's recordings, converted (below) and cut to the motor's signals, and three
frequency responses computed from velocity sweeps of the same bench published with

> C. Hinze et al.: *Replication Data for: Robust µ parameterization with low tuning complexity of cascaded control for
> feed drives*. DaRUS, 2023. https://doi.org/10.18419/darus-3252. ISW, University of Stuttgart. Licence: CC BY 4.0.

(the same sweep files are in darus-3003 too). Use them under CC BY 4.0: name the authors and the DOIs above. ISW has not
reviewed this conversion.

## Machine

From the source's README and metadata:

- A table of 400 kg, 0.72 m of travel, on a ball screw Steinmeyer 3526/40.40.6,3E, 40 mm, spindle 1800 mm long.
- Motor SIEMENS 1FT7085-7WF71 (water cooled) on a SINAMICS S120; encoder AM2048S/R on the motor, a Heidenhain
  LS 187C glass scale on the table.
- A Rexroth linear direct drive under the table, used to push disturbance forces in the source's other recordings;
  in these three it pushes no disturbance (`isLDDmoving` is 0), but its magnets' pull is compensated by a lookup table
  (darus-2563): `F_des_LDD`, the force it is told to push for that, runs from −113 to +123 N and is a column of the CSV.
  On the sine it shows in the nut force one for one and in the motor torque (docs/validation.md).
- Speed loop (PI) and position loop (P, 35 1/s) on a dSPACE DS1006; the drive gets a torque setpoint. The recordings
  name the speed loop's gains 166 and 69.2 without units (darus-3252: Kp = 175, Ki = 11); machine.toml takes them as
  read off the S-profile's signals.

Not stated by the source, our estimates (machine.toml says the same where it uses them):

- **The lead, 40 mm.** Read from the article number: Steinmeyer numbers give lead.diameter (its page for the
  3526/25.40.7,5.3NE lists lead 25 mm, diameter 40 mm). The recordings give the motor's position only as table
  travel, converted by the source with its lead, so the lead cannot be read from them. The velocity sweeps check it
  without the torque fit: their load side is 3.0 to 3.7 times their motor side, which a lead of 40 mm gives (2.7 to 4.2)
  and one of 25 mm does not (1.0 to 1.6).
- Horizontal mounting; whether the motor has a brake (machine.toml takes the rotor without).

## Files

| File | Source file | Test | What |
| --- | --- | --- | --- |
| `s-profile-v300.csv` | darus-3003 `Validation/SProfile/SPP_v300_xS10_xE700_a1000.mat` | cycle | 10 mm to 700 mm and back: 0.3 m/s, 1 m/s², 100 m/s³, 6.2 s. The recording the config is fitted on |
| `sine-a100-v100.csv` | darus-3003 `Validation/SineTrajectory/Sine_xS250_t20_A100_v100_f0-1.mat` | friction-map | sine of ±100 mm about 250 mm at 0.1 Hz, 21 s. A check |
| `constant-speed-v100.csv` | darus-3003 `const_v_100.mat` | friction-map | 0 to 700 mm and back at 0.1 m/s, 15 s. A check |
| `velocity-sweep-s0.csv`, `-s350.csv`, `-s700.csv` | darus-3252 `Identification/VelocitySweeps/vel_sweep_s_0_dir_pos.mat`, `_s_350_dir_pos`, `_s_700_dir_neg` | frequency-response | motor torque to motor speed at the table's 0, 350 and 700 mm, the speed loop closed, 15 to 250 Hz |
| `map.toml` | | | the columns at the motor shaft, the setpoint on the simulator's clock |
| `machine.toml` | | | the bench as the simulator's `screw-axis` machine, every value marked with its origin |

Columns: `time [s]`; `position setpoint [m]` (`traj_KGT[0]`, the table's planned position); `position actual [m]`
(`x_meas_motor`, the motor encoder as table travel); `speed actual [m/s]` (`v_meas_motor`); `torque actual [Nm]`
(`M_KGT_act`, the motor torque the S120 reports from its current); `LDD force setpoint [N]` (`F_des_LDD`, above);
`nut force [N]` (`F_nut_sum`, four KISTLER sensors in the nut, not referenced: their zero drifts, and their force per
acceleration is 264 kg on the S-profile where the table has 400). The map turns metres into motor radians through the
lead (`per_rev = 0.04`); the last two columns it does not read. The source records its setpoint in the same cycle as the
position it compares it with, one cycle ahead of a simulated trace: the map puts it 1 ms later (`time_offset_s`), so that
the following errors compare.

## Conversion

The source files are MATLAB v7.3 structs written by dSPACE ControlDesk, signals at 1 ms and at 4 kHz.
`tools/darus_ball_screw_to_csv.py` writes the CSV: the 1 ms grid, and of the 4 kHz signals every fourth sample (the
grids meet there; the script checks it). Nothing is filtered or averaged; the operator's user name and the rest of the
signals are left out. With `--sweep` it computes a velocity sweep's frequency response from the 4 kHz torque and speed
(Welch, Hann windows of 8192 samples overlapping by half, five neighbouring frequencies averaged). With the source files
downloaded from DaRUS (no account needed):

```text
python -I tools/darus_ball_screw_to_csv.py SPP_v300_xS10_xE700_a1000.mat s-profile-v300.csv
python -I tools/darus_ball_screw_to_csv.py Sine_xS250_t20_A100_v100_f0-1.mat sine-a100-v100.csv
python -I tools/darus_ball_screw_to_csv.py const_v_100.mat constant-speed-v100.csv
python -I tools/darus_ball_screw_to_csv.py --sweep vel_sweep_s_0_dir_pos.mat velocity-sweep-s0.csv
python -I tools/darus_ball_screw_to_csv.py --sweep vel_sweep_s_350_dir_pos.mat velocity-sweep-s350.csv
python -I tools/darus_ball_screw_to_csv.py --sweep vel_sweep_s_700_dir_neg.mat velocity-sweep-s700.csv
```

## Use

A replay starts from the trace's own first position and speed, whatever `start.pose` says:

```text
sim-cli identify --trace s-profile-v300.csv --map map.toml
sim-cli replay screw-axis --config machine.toml --trace s-profile-v300.csv --map map.toml
sim-cli fit screw-axis --config machine.toml --trace s-profile-v300.csv --map map.toml
sim-cli identify --sweep velocity-sweep-s350.csv
sim-cli fit screw-axis --config machine.toml --set "start.pose=[0.35]" --sweep velocity-sweep-s350.csv
sim-cli modes screw-axis --config machine.toml
```

What came out is in docs/validation.md, "Against measured data".

## What is odd

- The torque is quantised in steps of 3.05 mN m and sampled every 1 ms from a 4 kHz record without filtering.
- The sine starts in motion: the setpoint jumps to 0.063 m/s at the first row and the measured torque reaches 34 N m in
  the first 10 ms; it stops as abruptly at 20 s.
- After a move the axis holds a torque at standstill (−1.44 N m after the S-profile, +2.25 N m after the constant-speed
  run): friction holding the speed loop's integral. A friction smoothed through zero holds none; machine.toml has
  `effects.slide_presliding` on for it.
- The constant-speed run was recorded in June 2022, the S-profile and the sine in August. The linear drive's
  compensation force (`F_des_LDD`) runs in all three, but shows in the motor torque on the sine only (0.0078 N m per N;
  below 0.003 on the other two).
