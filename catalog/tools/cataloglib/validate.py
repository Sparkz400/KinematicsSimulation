"""The catalogue's validator: the JSON Schema of each table and its metadata, the provenance of every value, and
the physics and cross-document checks of `checks.py`."""

import csv
import json
from pathlib import Path

import jsonschema


from .table import SCHEMA_VERSION, manifest, meta_path, read_meta

PROVENANCE_ROW_SEPARATOR = "|"


def schema(root, name):
    return json.loads((root / "schema" / f"v{SCHEMA_VERSION}" / f"{name}.schema.json").read_text(encoding="utf-8"))


def typed(row, properties):
    """The row as JSON Schema sees it: numbers parsed, empty fields left out. Unparsable numbers stay text, so that
    the schema reports them."""
    out = {}
    for column, text in row.items():
        if text == "":
            continue
        if properties.get(column, {}).get("type") == "number":
            try:
                out[column] = float(text)
                continue
            except ValueError:
                pass
        out[column] = text
    return out


def validate_table(root, name, meta_schema):
    """Errors of one table: its metadata, its rows against the class schema, its keys and its provenance."""
    errors = []
    path = root / name
    try:
        meta = read_meta(path)
    except (OSError, ValueError) as e:
        return [f"{meta_path(path).name}: {e}"]
    errors += [f"metadata: {e.message} at {'/'.join(map(str, e.absolute_path))}" for e in jsonschema.Draft202012Validator(meta_schema).iter_errors(meta)]
    if errors:
        return errors
    try:
        class_schema = schema(root, meta["class"])
    except OSError as e:
        return [f"no schema for class {meta['class']}: {e}"]
    row_schema = class_schema["$defs"][meta["table"]] if "table" in meta else class_schema
    if "table" in meta and meta["table"] not in class_schema.get("$defs", {}):
        return [f"class {meta['class']} has no table {meta['table']}"]
    validator = jsonschema.Draft202012Validator(row_schema)

    with open(path, encoding="utf-8", newline="") as f:
        reader = csv.reader(f)
        header = next(reader, [])
        raw = list(reader)
    if len(set(header)) != len(header):
        errors.append(f"a column is named twice in {header}")
    rows = []
    for number, values in enumerate(raw, start=2):
        if len(values) != len(header):
            errors.append(f"line {number}: {len(values)} fields, the header has {len(header)}")
            continue
        rows.append(dict(zip(header, values)))

    sources = {s["id"] for s in meta["sources"]}
    used = set()
    for key in [k for k in meta["key"] if k not in header]:
        errors.append(f"key column {key} is not in the table")
    seen = {}
    for number, row in enumerate(rows, start=2):
        for e in validator.iter_errors(typed(row, row_schema["properties"])):
            errors.append(f"line {number}: {e.message}" + (f" ({'/'.join(map(str, e.absolute_path))})" if e.absolute_path else ""))
        key = PROVENANCE_ROW_SEPARATOR.join(row.get(k, "") for k in meta["key"])
        if key in seen:
            errors.append(f"line {number}: the key {key} is also on line {seen[key]}")
        seen.setdefault(key, number)
        row_source = row.get("source") or meta.get("default_source")
        if not row_source:
            errors.append(f"line {number}: no source (no source column and no default_source)")
        elif row_source not in sources:
            errors.append(f"line {number}: unknown source {row_source}")
        else:
            used.add(row_source)

    for entry in meta.get("provenance", []):
        if entry["source"] not in sources:
            errors.append(f"provenance: unknown source {entry['source']}")
        used.add(entry["source"])
        errors += [f"provenance: no column {c}" for c in entry["fields"] if c not in header]
        errors += [f"provenance: no row {r}" for r in entry["rows"] if r != "*" and r not in seen]
    if meta.get("default_source") and meta["default_source"] not in sources:
        errors.append(f"unknown default_source {meta['default_source']}")
    errors += [f"source {s} is not used by any value" for s in sorted(sources - used)]
    for s in meta["sources"]:
        if s["method"].startswith("script:") and not (root / s["method"][len("script:"):]).is_file():
            errors.append(f"source {s['id']}: no script {s['method'][len('script:'):]}")
    return errors


def validate(root, run_checks=True, report=print):
    """Validates the catalogue at `root`; returns the number of errors and reports each."""
    count = 0

    def fail(where, message):
        nonlocal count
        count += 1
        report(f"{where}: {message}")

    try:
        man = manifest(root)
    except (OSError, ValueError) as e:
        fail("catalog.toml", e)
        return count
    if man.get("schema_version") != SCHEMA_VERSION:
        fail("catalog.toml", f"schema_version {man.get('schema_version')}, this validator reads {SCHEMA_VERSION}")
        return count
    listed = man.get("tables", [])
    on_disk = {str(p.relative_to(root)).replace("\\", "/") for p in root.rglob("*.csv") if not {"tools", "schema"} & set(p.relative_to(root).parts)}
    for name in sorted(on_disk - set(listed)):
        fail(name, "not listed in catalog.toml")
    meta_schema = schema(root, "meta")
    tables = {}
    for name in listed:
        if not (root / name).is_file():
            fail(name, "listed in catalog.toml but missing")
            continue
        for message in validate_table(root, name, meta_schema):
            fail(name, message)
        with open(root / name, encoding="utf-8", newline="") as f:
            tables[name] = list(csv.DictReader(f))
    if run_checks:
        from . import checks
        for check in checks.CHECKS:
            for message in check(tables):
                fail(check.__name__, message)
    return count


def main(root, run_checks=True):
    count = validate(Path(root), run_checks)
    print(f"{count} error(s)" if count else "catalogue valid")
    return 1 if count else 0
