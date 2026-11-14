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

    def test_another_makers_motor_that_does_not_fit_its_own_numbers_fails_the_checks(self):
        from cataloglib import checks
        # A made-up motor that fits: Kt = 2 N m / 1.6 A = 1.25, Ke = 1.25 / sqrt(3) = 0.7217 V s/rad line to line,
        # rated 1.8 N m x 314.16 rad/s = 565 W, L / R = 0.02 H / 5 ohm = 4 ms.
        motor = {"part_number": "X1", "vendor": "maker", "series": "X", "line_voltage_v": "400", "static_torque_nm": "2",
                 "stall_current_a": "1.6", "rated_torque_nm": "1.8", "rated_current_a": "1.5", "rated_speed_rad_s": "314.16",
                 "rated_power_w": "565", "max_torque_nm": "8", "max_current_a": "7", "max_speed_rad_s": "600",
                 "torque_constant_nm_a": "1.25", "voltage_constant_v_s_rad": "0.7217", "line_resistance_20c_ohm": "5",
                 "line_inductance_h": "0.02", "pole_pairs": "4", "rotor_inertia_kgm2": "1e-4",
                 "rotor_inertia_with_brake_kgm2": "1.2e-4", "mass_kg": "2", "mass_with_brake_kg": "2.4",
                 "thermal_time_constant_s": "1200", "paired_drive": "D1", "source": ""}
        drive = {"part_number": "D1", "vendor": "maker", "series": "D", "supply": "3 AC 400 V", "line_voltage_v": "400",
                 "axes": "1", "rated_output_current_a": "2", "max_output_current_a": "6", "max_current_duration_s": "2",
                 "dc_link_voltage_v": "565", "dc_link_capacitance_f": "", "braking_resistor_internal_ohm": "",
                 "braking_resistor_continuous_w": "", "braking_resistor_peak_w": "", "min_braking_resistance_ohm": "",
                 "braking_threshold_v": "780", "dc_link_overvoltage_v": "850", "source": ""}

        def found(**changes):
            tables = {"maker/motors_x.csv": [dict(motor, **changes)], "maker/drives_d.csv": [drive]}
            return checks.vendor_motors_are_plausible_and_their_drives_exist(tables) + checks.vendor_drives_are_plausible(tables)

        self.assertEqual(found(), [])
        # Ke printed per phase or as a peak: sqrt(3) x 0.7217 x sqrt(2) = 1.77, 41 % off Kt.
        self.assertTrue(any("voltage_constant" in e for e in found(voltage_constant_v_s_rad="1.0206")))
        self.assertTrue(any("rated_power_w" in e for e in found(rated_power_w="5650")))
        self.assertTrue(any("not in a maker drive table" in e for e in found(paired_drive="D2")))
        self.assertTrue(any("needs 3.5 A" in e for e in found(rated_current_a="3.5", stall_current_a="", torque_constant_nm_a="")))
        self.assertTrue(any("L / R" in e for e in found(line_inductance_h="2")))

    def test_a_column_without_a_map_is_refused(self):
        with self.assertRaises(SystemExit):
            legacy.convert_rows([{"article": "1FK2", "colour": "blue"}], [("article", "part_number", "same")], "test")


if __name__ == "__main__":
    unittest.main()
