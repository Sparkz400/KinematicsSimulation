# Title of the dataset

What machine, what was measured, and why. Keep it to what a reader needs to use the data:

- **Machine**: what the motor drives, transmission, mounting, condition (new, worn, re-tensioned).
- **Drive settings**: the loop gains and filters in force during the recording, and anything switched off
  for the test (speed loop gain lowered for the torque step, for example).
- **Tests**: per file, which test move of docs/measurement-procedure.md, with its speeds, amplitudes and times.
- **What is odd**: a signal that clipped, a glitch at 3.2 s, a temperature drift.

Before publishing, read the anonymisation list in datasets/README.md, then run:

```text
sim-cli check-dataset datasets/<id>
```
