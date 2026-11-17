"""Tests of siemens/s1_curves_s210.csv against the printed values of motors_s210.csv, and of the digitising helpers:
python -m unittest discover -s tools/tests (from the catalogue's folder)."""

import csv
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import extract_siemens_s210_s1 as extract  # noqa: E402

SIEMENS = Path(__file__).resolve().parents[2] / "siemens"


def read(name):
    with open(SIEMENS / name, encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))


def curves():
    out = {}
    for r in read("s1_curves_s210.csv"):
        out.setdefault(r["article"], []).append((float(r["speed_rad_s"]), float(r["torque_nm"])))
    return out


class S1Curves(unittest.TestCase):
    def test_every_curve_starts_at_the_static_torque_and_passes_the_rated_point(self):
        motors = {r["part_number"]: r for r in read("motors_s210.csv")}
        found = curves()
        self.assertGreaterEqual(len(found), 50)
        for article, pts in found.items():
            row = motors[article]  # a curve of a motor that is not in the table fails here
            self.assertEqual(pts, sorted(pts, key=lambda p: p[0]), article)
            self.assertEqual(pts[0][0], 0.0, article)
            self.assertTrue(5 <= len(pts) <= 12, f"{article}: {len(pts)} points")
            self.assertTrue(all(b[0] > a[0] for a, b in zip(pts, pts[1:])), f"{article}: speeds do not rise")
            self.assertLessEqual(abs(pts[0][1] - float(row["static_torque_nm"])), extract.tolerance(row["static_torque_nm"]), article)
            self.assertLessEqual(pts[-1][0], 1.03 * float(row["max_speed_rad_s"]), f"{article}: curve beyond the maximum speed")
            supply = "400v" if row["rated_speed_400v_rad_s"] else "230v"  # the supply the curve is read for
            w, q = float(row[f"rated_speed_{supply}_rad_s"]), row[f"rated_torque_{supply}_nm"]
            t = extract.at_speed(pts, w)
            self.assertIsNotNone(t, f"{article}: the curve ends before the rated speed")
            self.assertLessEqual(abs(t - float(q)), extract.tolerance(q), f"{article}: {t} N m at {w} rad/s, printed {q}")
            self.assertTrue(all(0 <= y <= 1.03 * float(row["max_torque_nm"]) for _, y in pts), article)

    def test_the_helpers_do_what_their_comments_say(self):
        self.assertAlmostEqual(extract.tolerance("1.7"), 0.05)  # half the last digit beats 2 %
        self.assertAlmostEqual(extract.tolerance("40"), 0.8)  # 2 % beats half the last digit
        self.assertEqual(extract.labels_x("0100020003000"), (1000, 3))
        self.assertEqual(extract.labels_x("01000200030008000400050006000"), (1000, 8))  # the label 7000 is missing
        self.assertAlmostEqual(extract.at_speed([(0, 4.0), (10, 2.0)], 5), 3.0)
        line = [(x, 0.0 if x < 50 else 1.0 + x / 100) for x in range(0, 101)]  # a step: it must be kept
        red = extract.reduce(line, 5, 12, 100, 1.0)
        self.assertLessEqual(len(red), 12)
        self.assertTrue(all(any(p == q for q in red) for p in (line[0], line[-1])))
        self.assertGreaterEqual(len(extract.reduce([(0, 1.0), (100, 1.0)], 5, 12, 100, 1.0)), 5)


if __name__ == "__main__":
    unittest.main()
