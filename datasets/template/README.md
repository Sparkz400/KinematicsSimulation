# Title of the dataset

What machine, what was measured, and why. Keep it to what a reader needs to use the data; the step-by-step guide is
docs/measurement-procedure.md, "Contributing a measured machine".

## Source and licence

Who measured it, when (year and month), and the licence (CC BY 4.0 suggested). If the data were published elsewhere
first: the authors, the title, the DOI, and that the licence allows this copy.

## Machine

What the motor drives, transmission (ratio, lead, pulley), moved mass and payload, mounting, condition (new, worn,
re-tensioned). Which of the simulator's machines it is (`machine.simulator`) and what `machine.toml` changes from
that machine's default config, each value marked `[source]`, `[catalogue]` or `[estimate]`.

## Drive settings

The loop gains, feedforward and filters in force during the recording, and anything switched off for the test (the
speed loop's gain lowered for the torque step, for example). The drive's current, speed and position cycles.

## Files

| File | Test | What |
| --- | --- | --- |
| `cycle.csv` | cycle | the machine's cycle, fitted on: speeds, accelerations, travel, duration |
| `friction-map.csv` | friction-map | a check: the constant speeds, both ways |
| `map.toml` | | which column is which signal, in which unit |

## What is odd

A signal that clipped, a glitch at 3.2 s, a temperature drift, a force on the axis from outside.

## Check and report

Before publishing, read the anonymisation list in datasets/README.md, then run:

```text
sim-cli check-dataset datasets/<id>
sim-cli fit --dataset datasets/<id> --report report/
```
