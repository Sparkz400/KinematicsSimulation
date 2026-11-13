# Contributing to the catalogue

Data comes from the makers' public documents, read by a script wherever the document has tables, and every number carries where it came from. Read [README.md](README.md) for the format first.

What you need: Python 3.11 or newer, `pip install jsonschema`, and for PDF documents `pdftotext` from xpdf 4.x (`-table` mode keeps table rows on one line). Commands run from this folder.

## The rules

1. **Never commit the vendor's document.** Not the PDF, not its text, not a page copied. Only the numbers go in, in this format.
2. **A script, not typing**, wherever the document has tables: it can be run again on the next edition, and it fails loudly instead of shifting a column. Type in by hand only what has no table (a figure in running text, three sizes of one product), and say so in the source's `method` (`manual: ...`).
3. **Values as printed.** Do not correct a misprint in the data. Name it in the table's `notes`, and if a check trips on it, make that check skip exactly that value with a comment saying why.
4. **No guessed values.** Leave a field empty when the document does not give it. A value that is an estimate gets a source of its own with `method = "[estimate] <reason>"` and a `[[provenance]]` entry for exactly the values it covers.
5. **SI units and the class's column names.** If a column you need is not in the class schema, add it there (an optional property with `"unit"`) in the same change.

## Adding a vendor (or a document of a known vendor)

1. **Find the tables** in the document and the part class they belong to (`schema/v1/*.schema.json`). Pick the class's column names for the quantities; add missing ones to the schema.
2. **Write the extraction script** `tools/extract_<vendor>_<document>.py`, taking the text file (or folder) and an output folder:

   ```sh
   pdftotext -table -enc UTF-8 document.pdf document.txt
   python tools/extract_<vendor>_<document>.py document.txt out/
   ```

   Model it on an existing one (`extract_megadyne.py` is short). It writes one CSV per table into the output folder, in the document's own terms and units if that is easier: the conversion happens next. Make it strict: stop with a message when a row has the wrong number of values, a part appears twice with different values, or a table is missing.
3. **Map its columns**: add an entry per table to `TABLES` in `tools/cataloglib/legacy.py`, keyed by `<vendor>/<file>`: the catalogue file, the class, the class's table (or `None`), and per column of the script `(script's column, catalogue column, conversion)`. Conversions are in `tools/cataloglib/units.py` (`milli`, `kilo`, `rpm`, `kgcm2`, `arcmin`, ...). A column the script writes and the map does not name stops the conversion; so does a value that does not convert back to what was written. If the script already writes schema columns in SI, map each column to itself with `"same"`.
4. **Write the metadata** `<vendor>/<table>.meta.toml` (copy one, e.g. `siemens/converters_s210.meta.toml`): `schema_version = 1`, `class`, `table` if any, `key`, `default_source`, `notes`, and the `[[sources]]` with `document`, `date`, `locator`, `method = "script:tools/extract_<...>.py"`, `url`. Add the table's path to `tables` in `catalog.toml`.
5. **Run it through the pipeline**, which runs the script into a scratch folder, converts, writes the table and validates:

   ```sh
   python tools/catalog.py extract <vendor>_<document> document.txt
   ```

   (For the first run the table must exist so its metadata can be found: create the `.csv` with just the header, or run the script and the conversion once by hand.)
6. **Add checks** for what physics ties together in the new data, to `CHECKS` in `tools/cataloglib/checks.py`: a function taking the tables and returning a list of messages. Check against the document's own other columns and against other documents of the same part. Then break one value on purpose and see the check fail.
7. **Validate and test**:

   ```sh
   python tools/catalog.py validate
   python -m unittest discover -s tools/tests
   ```

## Rerunning an extraction on a newer edition

```sh
python tools/catalog.py extract <script> <input> [more arguments]
```

`<script>` is the name after `extract_` (e.g. `siemens_d32`), or the script's file name (`fetch_siemens_datasheets`). The commands per table, and where each document is, are in the simulator's [docs/catalogue.md](../docs/catalogue.md#rerun-an-extraction-on-a-newer-edition). Then:

1. If the script stops, the new edition has a row or table it cannot read; it says which. Fix the script, not the output.
2. Update `date`, `document`, `url` of the source in the table's metadata, and the notes if a misprint is gone or a new one came.
3. Review the diff of the table.
4. Validate. A misprint the new edition fixed shows as a check that names the old exception; remove the exception.
5. In the simulator: `cargo test -p sim-machines --test catalog_golden` shows every value the simulator takes from the catalogue that changed. If the change is meant, write the snapshot anew with `UPDATE_GOLDEN=1`.

## A change to the schema

Adding an optional column, a table, or a class is schema 1. Renaming or removing a column, changing a unit or a meaning is a schema 2: it needs `schema/v2/`, a migration script, the `schema_version` raised everywhere, and the simulator's loader updated to read it. Discuss it first.
