"""The catalogue's one entry point for contributors. Run from anywhere; Python 3.11+ and `jsonschema`.

    python tools/catalog.py validate [--root DIR] [--schema-only]
        Checks every table: its JSON Schema, the provenance of every value, keys, and the physics and
        cross-document checks. Exit code 1 on any error. This is what CI runs.

    python tools/catalog.py list
        The tables, their part class and number of rows.

    python tools/catalog.py extract <script> <input> [more arguments]
        Runs tools/extract_<script>.py (or tools/<script>.py) on <input> into a scratch folder, converts what it
        writes to schema 1 (cataloglib/legacy.py), puts the tables in place and validates. The tables a script
        writes are those whose metadata names it as `method = "script:..."`. Their metadata is kept: update the
        source's edition and date there when the document is a new edition.

See CONTRIBUTING.md.
"""

import argparse
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from cataloglib import legacy  # noqa: E402
from cataloglib.table import ROOT, manifest, read_meta, read_rows, write_rows  # noqa: E402
from cataloglib.validate import main as validate_main  # noqa: E402


def script_path(name):
    tools = ROOT / "tools"
    for candidate in (tools / f"extract_{name}.py", tools / f"{name}.py", tools / name):
        if candidate.is_file():
            return candidate
    raise SystemExit(f"no script {name} in {tools}")


def extract(name, arguments):
    script = script_path(name)
    method = "script:" + str(script.relative_to(ROOT)).replace("\\", "/")
    tables = [t for t in manifest()["tables"] if any(s["method"] == method for s in read_meta(ROOT / t)["sources"])]
    if not tables:
        raise SystemExit(f"no table names {method} as its source")
    with tempfile.TemporaryDirectory() as scratch:
        if len(arguments) < 1:
            raise SystemExit("extract needs the input of the script")
        subprocess.run([sys.executable, str(script), arguments[0], scratch, *arguments[1:]], check=True)
        for table in tables:
            written = Path(scratch) / Path(table).name
            if not written.is_file():
                raise SystemExit(f"{script.name} did not write {written.name}")
            _, cls, kind, columns = legacy.TABLES[table]
            notes, rows = legacy.read_legacy(written)
            header, out = legacy.convert_rows(rows, columns, table)
            existing = (ROOT / table).read_text(encoding="utf-8").splitlines()[:1] if (ROOT / table).is_file() else []
            if "source" in "".join(existing).split(","):
                # Rows that carry their own source (Bahr: one per data sheet) keep the id of the sheet they came from.
                header.append("source")
                for row, new in zip(rows, out):
                    new["source"] = legacy.source_of_row(row)
            write_rows(ROOT / table, header, out)
            print(f"{table}: {len(out)} rows; check the edition and date of its source in {Path(table).stem}.meta.toml")
    return validate_main(ROOT)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    v = sub.add_parser("validate", help="validate the catalogue")
    v.add_argument("--root", default=str(ROOT), help="the catalogue's folder (default: the one this script is in)")
    v.add_argument("--schema-only", action="store_true", help="skip the physics and cross-document checks")
    sub.add_parser("list", help="list the tables")
    e = sub.add_parser("extract", help="run an extraction script and convert its output")
    e.add_argument("script")
    e.add_argument("arguments", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    if args.command == "validate":
        return validate_main(args.root, not args.schema_only)
    if args.command == "list":
        for table in manifest()["tables"]:
            meta = read_meta(ROOT / table)
            print(f"{table:40} {meta['class'] + ('/' + meta['table'] if 'table' in meta else ''):32} {len(read_rows(ROOT / table)):4} rows")
        return 0
    return extract(args.script, args.arguments)


if __name__ == "__main__":
    sys.exit(main())
