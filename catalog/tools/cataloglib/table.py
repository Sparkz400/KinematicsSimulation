"""Reading and writing the catalogue's tables: `<name>.csv` (a header and one row per part) beside
`<name>.meta.toml` (part class, sources, provenance, notes), listed in `catalog.toml`."""

import csv
import io
import json
import tomllib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SCHEMA_VERSION = 1


def manifest(root=ROOT):
    return tomllib.loads((root / "catalog.toml").read_text(encoding="utf-8"))


def meta_path(csv_path):
    return csv_path.with_name(csv_path.stem + ".meta.toml")


def read_meta(csv_path):
    return tomllib.loads(meta_path(csv_path).read_text(encoding="utf-8"))


def read_rows(csv_path):
    """The rows as dicts of text; an empty field is ""."""
    with open(csv_path, encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))


def write_rows(csv_path, header, rows):
    out = io.StringIO()
    writer = csv.DictWriter(out, fieldnames=header, lineterminator="\n")
    writer.writeheader()
    writer.writerows(rows)
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    csv_path.write_text(out.getvalue(), encoding="utf-8", newline="\n")


def _toml_value(value):
    if isinstance(value, list):
        return "[" + ", ".join(_toml_value(v) for v in value) + "]"
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, str) and "\n" in value and "'''" not in value:
        return "'''\n" + value + "'''"
    return json.dumps(value, ensure_ascii=False)


def write_meta(csv_path, meta):
    """Writes the metadata: plain keys first, then `[[sources]]` and `[[provenance]]`."""
    lines = []
    for key, value in meta.items():
        if not isinstance(value, list) or not value or not isinstance(value[0], dict):
            lines.append(f"{key} = {_toml_value(value)}")
    for key, value in meta.items():
        if isinstance(value, list) and value and isinstance(value[0], dict):
            for item in value:
                lines += ["", f"[[{key}]]"] + [f"{k} = {_toml_value(v)}" for k, v in item.items()]
    meta_path(csv_path).write_text("\n".join(lines) + "\n", encoding="utf-8", newline="\n")
