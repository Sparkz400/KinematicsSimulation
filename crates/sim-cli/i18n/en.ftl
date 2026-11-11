cli-usage = Usage: sim-cli list | run MACHINE [--move a,b | --cycle] [--time SECONDS] [--every STEPS] | analyze MACHINE [--points FILE] | sweep MACHINE [--motor N] [--from HZ] [--to HZ] [--count N] [--amplitude NM] | step MACHINE [--axis N] [--size M] [--time SECONDS] | bode MACHINE [--axis N] [--from HZ] [--to HZ] [--count N] [--amplitude M] | tuning MACHINE; for all: [--config FILE] [--set path=value]... [--out FILE] [--lang en|de]
cli-error-unknown-machine = Unknown machine: { $arg }
cli-error-bad-argument = Cannot read this argument: { $arg }
cli-error-io = File error: { $arg }
