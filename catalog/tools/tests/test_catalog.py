"""Tests of the catalogue's tools: python -m unittest discover -s tools/tests (from the catalogue's folder)."""

import math
import shutil
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from cataloglib import legacy  # noqa: E402
from cataloglib.table import ROOT, read_rows  # noqa: E402
from cataloglib.units import CONVERSIONS  # noqa: E402
from cataloglib.validate import validate  # noqa: E402


def errors(root, run_checks=True):
    found = []
    validate(root, run_checks, report=found.append)
    return found


class Catalogue(unittest.TestCase):
    def test_the_catalogue_is_valid(self):
        self.assertEqual(errors(ROOT), [])

    def test_a_broken_copy_fails_for_the_reason_it_is_broken(self):
        def broken(edit, expected):
            with tempfile.TemporaryDirectory() as scratch:
                root = Path(scratch) / "catalog"
                shutil.copytree(ROOT, root, ignore=shutil.ignore_patterns("__pycache__"))
                edit(root)
                found = errors(root, run_checks=False)
                self.assertTrue(any(expected in e for e in found), f"expected '{expected}' in {found}")

        def replace(file, old, new):
            def edit(root):
                text = (root / file).read_text(encoding="utf-8")
                self.assertIn(old, text)
                (root / file).write_text(text.replace(old, new, 1), encoding="utf-8")
            return edit

        broken(replace("siemens/gearbox_nrk.meta.toml", 'default_source = "gearboxes"\n', ""), "no source")
        broken(replace("siemens/gearbox_nrk.csv", ",0.98,", ",ninety-eight,"), "is not of type 'number'")
        broken(replace("siemens/gearbox_nrk.csv", ",0.98,", ",-0.98,"), "less than the minimum")
        broken(replace("siemens/converters_s210.csv", "part_number,", "article,"), "Additional properties")
        broken(replace("siemens/converters_s210.csv", "6SL3210-5HB10-8UF0", "6SL3210-5HB10-1UF0"), "is also on line")
        broken(replace("siemens/braking_s210.meta.toml", 'fields = ["dc_link_overvoltage_v"]', 'fields = ["trip_v"]'), "provenance: no column trip_v")
        broken(replace("siemens/dc_link_measured.meta.toml", 'method = "measured: worked out from the trace as the notes describe"', 'method = "[estimate]"'), "does not match")
        broken(replace("siemens/motors_s210.meta.toml", 'date = "2024-01"', 'date = "January 2024"'), "does not match")
        broken(replace("bahr/belt_axes.csv", "de_21_30_100_ELZI\n", "de_21_30_100_ELZX\n"), "unknown source de_21_30_100_ELZX")
        broken(lambda root: shutil.copy(root / "megadyne/belts.csv", root / "megadyne/belts_copy.csv"), "not listed in catalog.toml")
        broken(replace("catalog.toml", "schema_version = 1", "schema_version = 2"), "schema_version 2")

    def test_every_table_converts_back_to_what_its_extraction_writes(self):
        # A row of the catalogue, turned back into the units of its document, comes out of `convert_rows` as it is:
        # the column maps the extractions go through hold for the data that is there.
        for name, (table, _, _, columns) in legacy.TABLES.items():
            rows = read_rows(ROOT / table)
            back = []
            for row in rows:
                old = {}
                for before, after, conversion in columns:
                    if after is None:
                        old[before] = "x"
                    elif conversion == "same" or row[after] == "":
                        old[before] = row[after]
                    else:
                        old[before] = repr(float(row[after]) / float(CONVERSIONS[conversion][0]))
                back.append(old)
            header, again = legacy.convert_rows(back, columns, name)
            for row, new in zip(rows, again):
                for column in header:
                    a, b = row[column], new[column]
                    same = a == b or (a and b and math.isclose(float(a), float(b), rel_tol=1e-12))
                    self.assertTrue(same, f"{table} {column}: {a} became {b}")

    def test_a_column_without_a_map_is_refused(self):
        with self.assertRaises(SystemExit):
            legacy.convert_rows([{"article": "1FK2", "colour": "blue"}], [("article", "part_number", "same")], "test")


if __name__ == "__main__":
    unittest.main()
