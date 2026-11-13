# Measured datasets

Drive traces of real machines, published so that the simulator's models and the identification can be
checked against more than the simulator's own output. Each dataset is a folder `datasets/<id>/`:

| File | What |
| --- | --- |
| `dataset.toml` | The metadata: what was measured, on what, under which license. |
| `README.md` | What a reader needs to use the data: settings, test moves, anything odd. |
| one file per recording | The trace as the drive's tool exported it (CSV of Startdrive, TwinCAT Scope, CODESYS, ...). Keep the export as it is; the mapping converts it. |
| one mapping per format | A TOML file that names the signals and their units ([template/map.toml](template/map.toml)). |

Nothing else goes in the folder; the checker refuses files no recording names. Start from
[template/](template/), and record as [docs/measurement-procedure.md](../docs/measurement-procedure.md)
describes.

## dataset.toml

Every key is described in [template/dataset.toml](template/dataset.toml). In short:

- `[dataset]`: `id` (the folder's name), `title`, `description`, `license` (CC0-1.0, CC-BY-4.0,
  CC-BY-SA-4.0, ODbL-1.0 or PDDL-1.0), `contributor` (an organisation or `anonymous`), `recorded` (year and
  month, `YYYY-MM`), `synthetic` (true only for data a simulator made).
- `[machine]`: `kind`, `transmission`, `mounting`; optional `travel_per_motor_rev_m`, `payload_kg`.
- `[drive]`: `vendor`, `family`; optional `current_cycle_s`, `speed_cycle_s`, `position_cycle_s`.
- `[motor]`: `family`, `rated_torque_nm`; optional `rotor_inertia_kg_m2`, `encoder_counts_per_rev`.
- `[[recording]]`, one per file: `file`, `map` (not for a frequency response), `test` (one of the test
  moves: `inertia-sines`, `friction-map`, `slow-reversal`, `torque-step`, `noise-excitation`,
  `frequency-response`, `cycle`), `sample_time_s`; optional `notes`.

A frequency response measured by the drive (the SINAMICS measuring function, the TwinCAT or CODESYS Bode
plot) is a CSV with the columns `frequency_hz` and `gain_rad_s_per_nm` (motor torque to motor speed),
converted by hand: Siemens does not document the export of its measuring function.

## Anonymisation

Before a dataset is published, remove what identifies people, sites, networks or machines:

- Names and e-mail addresses of people, in the files and in the exports' headers.
- IP addresses and TwinCAT AMS Net IDs (a TwinCAT Scope export lists the Net ID of every channel).
- Paths of the project or the user (a CODESYS `.txt` has the project's path in its second line; a TwinCAT
  export the file's path in its first lines).
- Serial numbers of drives, motors and encoders; order numbers of whole machines.
- Customer, site and project names in channel names and comments.
- The recording date beyond year and month.

Replace a line rather than deleting it where a format expects it (the importers skip header lines they do
not read). `sim-cli check-dataset` finds e-mail addresses, IP addresses and Net IDs, absolute paths and the
word "serial" in every file; it cannot find a name. Read the files once more yourself.

## The checker

```text
sim-cli check-dataset datasets/<id>
```

prints every problem and ends with exit code 2 if there is one: a missing or unknown key, a value of the
wrong kind, an `id` that is not the folder's name, a license not on the list, a date with a day, a file
that is missing or not named by a recording, a mapping that does not load, a trace that does not import
through its mapping or has fewer than 100 rows, a `sample_time_s` that is not the trace's own (1 %), and
whatever the anonymisation check finds. The tests run it on every folder here except `template/`.

## Datasets

| Folder | What | Measured |
| --- | --- | --- |
| [example-synthetic-belt-axis](example-synthetic-belt-axis/) | The simulator's single axis in its cycle, written like a drive trace. Shows the format. | No |

No measured dataset has been published yet.
