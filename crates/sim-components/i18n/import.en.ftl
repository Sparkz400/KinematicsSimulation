# Vendor trace import, the machine-free identification and the dataset checker.
error-import-layout = The trace file does not have the layout of its format: { $arg }
error-import-unit = Unknown unit, give `unit`, or `scale` to N m, in the mapping: { $arg }
error-import-per-rev = A unit that is not an angle needs `per_rev` (units per motor revolution) in the mapping: { $arg }
error-import-reference = A torque in percent or per mille needs `reference` (N m at 100 %) in the mapping: { $arg }
error-import-signal = The trace file has no signal { $arg }
error-import-ambiguous = More than one signal of the trace file matches { $arg }; give its full name.
error-import-map = The mapping file is missing or has a wrong value: { $arg }
error-import-time = The times of the trace cannot be put on one grid: { $arg }
error-identify-friction = Motor { $arg }: no inertia or friction fitted. The trace has to turn it both ways for at least 0.1 s each.
error-identify-lag = Motor { $arg }: no lag fitted. The trace has to start at rest and the setpoint has to change.
error-identify-two-mass = No two-mass model: the response needs a dip, a peak above it and three points below a third of the dip.
error-identify-ringing = No ringing found between the given times.
dataset-ok = Dataset { $arg }: no problems found.
dataset-problem = Dataset problem: { $arg }
error-identify-response = The trace is too short for a frequency response down to --from: it needs eight windows of four periods of the lowest frequency.
