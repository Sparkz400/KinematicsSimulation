# Open drive-train component catalogue

Data of servo motors, drives, gearboxes, belts and belt axes, read from the makers' public documents, in one vendor-neutral format: SI units, the same column name for the same quantity whatever the maker calls it, and for every value the document, page or table, date and method it came from. The kinematics simulator in the parent repository reads it; anything else can too, it is CSV.

- [Layout](#layout)
- [Schema 1](#schema-1)
- [Provenance](#provenance)
- [Validating](#validating)
- [Using it from the simulator](#using-it-from-the-simulator)
- [Versioning](#versioning)
- [Its own repository](#its-own-repository)

Adding data: [CONTRIBUTING.md](CONTRIBUTING.md). Where each table comes from, what it holds and what the documents get wrong: [docs/catalogue.md](../docs/catalogue.md) of the simulator.

## Layout

```text
catalog.toml                 schema_version and the list of tables
<maker>/<table>.csv          one row per part, a header line, nothing else
<maker>/<table>.meta.toml    part class, key columns, sources, provenance, notes
schema/v1/<class>.schema.json   JSON Schema (draft 2020-12) of a row of each part class
schema/v1/meta.schema.json      JSON Schema of a .meta.toml
tools/catalog.py             the one entry point: validate, list, extract
tools/cataloglib/            shared helpers: tables, units, column maps, validator, checks
tools/extract_*.py, tools/fetch_*.py   one script per vendor document
tools/tests/                 tests of the tools
.github/workflows/validate.yml   CI once this folder is a repository of its own
```

`tools/step_to_glb.py`, `tools/glb_part.py` and `tools/exct30_meshes.sh` turn CAD models into meshes for the simulator's 3D view. They are not part of the data pipeline and belong to the simulator; move them out before a split (see below).

## Schema 1

Part classes, one schema file each: `servo_motor`, `drive`, `braking_resistor`, `gearbox`, `belt`, `belt_axis`, `ball_screw`, `linear_guide`, `linear_motor` (the last three have a schema and no data yet). A class can have further tables for data that is not one row per part, in the schema's `$defs`: `servo_motor/dynamic_braking`, `drive/braking`, `drive/dc_link_measurement`, `drive/line_module`, `gearbox/mass`, `belt/tooth_load`, `belt_axis/torque_formula`. A table's metadata names its `class` and, for these, its `table`.

The rules every table keeps:

- **SI units, named in the column**: `_m`, `_kg`, `_s`, `_a`, `_v`, `_w`, `_j`, `_f`, `_h`, `_ohm`, `_hz`, `_rad`, `_rad_s`, `_nm` (newton metre), `_n`, `_kgm2`, `_kg_m` (per metre), `_nm_rad`, `_v_s_rad`. No unit suffix: a ratio or a count. Each number's unit is also in the schema as `"unit"`. Speeds are in rad/s, so 3000 r/min is `314.1592653589793`.
- **Vendor-neutral names**: `part_number` (what the maker calls article, order or MLFB number), `rated_output_torque_nm` of a gearbox, `paired_drive` of a motor, `mass_at_zero_stroke_kg` of an axis. A column means the same in every table of a class.
- **An empty field** is a value the source does not give. Nothing is filled in by guessing; an estimate is a value with an `[estimate]` source.
- **Values as printed**, converted to SI. What a document prints wrongly is kept and named in the notes; the checks skip that one value and say why.
- **Key columns** (`key` in the metadata) name a row; no two rows have the same.

## Provenance

Every value has a source: a `[[sources]]` entry of its table's metadata with

| Field | |
| --- | --- |
| `document` | Maker, title, document number |
| `date` | Edition (`YYYY`, `YYYY-MM` or `YYYY-MM-DD`), or the day of a measurement |
| `locator` | Chapter, page, table |
| `method` | `script:tools/<script>.py` (extracted by that script), `manual: <how>` (typed in), `measured: <how>`, or `[estimate] <reason>` |
| `url`, `retrieved` | Where it was downloaded, and when (optional) |

Which source a value has, in this order: a `[[provenance]]` entry naming its row (by key, `|` between key columns; `"*"` for all rows) and column; the row's `source` column; the table's `default_source`. The validator fails a table where any value has none, where a source is named but missing, or where a source is never used. Example: `siemens/braking_s210.meta.toml` takes its braking powers from Table 4-4 of the S210 operating instructions and the trip level `dc_link_overvoltage_v` from the text of fault F07432; `bahr/belt_axes.csv` names the data sheet of each row in its `source` column.

## Validating

```sh
pip install jsonschema
python tools/catalog.py validate          # schema, provenance, keys, physics and cross-document checks
python -m unittest discover -s tools/tests
```

Run from this folder. The physics and cross-document checks are in `tools/cataloglib/checks.py`: rated power against rated torque and speed, torque constant against static torque and stall current, every paired drive exists and carries the motor's current, the same motor in two documents agrees, a bigger gearbox weighs more, and so on (the list with the reasoning is in the simulator's docs/catalogue.md).

## Using it from the simulator

The simulator compiles in the copy of this folder it was built with (`crates/sim-components/src/catalog.rs`). Another copy, a newer edition or a fork, is read at run time with

```sh
sim-cli search t-gantry --config exct-30.toml --catalog path/to/catalog
sim-app --catalog path/to/catalog
```

The directory must have `catalog.toml` with the `schema_version` the simulator reads (1) and every table the simulator reads, under the same paths; extra tables are ignored. A local directory only: the simulator fetches nothing.

## Versioning

`schema_version` in `catalog.toml` and in every metadata file is the schema's major version, and `schema/v<n>/` holds its files. Adding a table, a row, a source, or an optional column to a class is not a new version. Renaming or removing a column, changing a unit, or changing what a column means is: it gets `schema/v2/`, a migration script in `tools/` (as `tools/migrate_v1.py` did from the old layout) and a simulator that reads it. The simulator refuses a catalogue of another version.

Data changes go by git history: a table's diff shows what changed, and its sources say from which edition.

## Its own repository

The folder is laid out to be split off with its history:

```sh
# in the simulator's repository, on the branch to split from
git subtree split --prefix=catalog -b catalogue-split
git push git@github.com:<owner>/<catalogue-repo>.git catalogue-split:main
```

Then in the new repository `.github/workflows/validate.yml` runs on every push. Before the split, move the 3D mesh tools (`tools/step_to_glb.py`, `tools/glb_part.py`, `tools/exct30_meshes.sh`) to the simulator, and decide the data's licence (the numbers are the makers'; the layout and the tools are the project's).

To keep the simulator on the split repository afterwards, either

- pull it back as a subtree, so the compiled-in copy keeps working without changes:
  ```sh
  git subtree pull --prefix=catalog git@github.com:<owner>/<catalogue-repo>.git main --squash
  ```
  and contribute with `git subtree push --prefix=catalog <remote> <branch>`, or
- check it out anywhere and run the simulator with `--catalog <checkout>`.

The simulator's own tests that pin the catalogue's numbers (`crates/sim-machines/tests/catalog_golden.rs`) are what tells it a pulled edition changes results; write their snapshot anew when the change is meant (`UPDATE_GOLDEN=1`).
