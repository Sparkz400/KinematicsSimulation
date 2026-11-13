"""The Onshape prototype on a response shaped like Onshape's documented sample: no credentials needed."""

import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import cadmass  # noqa: E402
import onshape_mass  # noqa: E402

# A steel box 0.1 x 0.2 x 0.3 m centred at (1, 2, 3): m = 7850 * 0.006 = 47.1 kg.
M = 47.1
I = [M * (0.2**2 + 0.3**2) / 12, M * (0.1**2 + 0.3**2) / 12, M * (0.1**2 + 0.2**2) / 12]
COM = [1.0, 2.0, 3.0]


def response(tensor):
    bounds = lambda v: v + [x * 0.999 for x in v] + [x * 1.001 for x in v]  # value, lower, upper
    return {"microversionId": "x", "bodies": {"-all-": {
        "mass": bounds([M]), "volume": bounds([0.006]), "centroid": bounds(COM), "inertia": bounds(tensor),
        "hasMass": True, "massMissingCount": 0, "principalInertia": I}}}


class OnshapeTests(unittest.TestCase):
    def test_centroid_tensor(self):
        b = onshape_mass.body(response([I[0], 0, 0, 0, I[1], 0, 0, 0, I[2]]), "carriage")
        self.assertEqual((b.mass, b.com), (M, tuple(COM)))
        self.assertEqual([b.inertia[i][i] for i in range(3)], I)

    def test_origin_tensor_is_moved_to_the_centroid(self):
        d2 = sum(c * c for c in COM)
        about_origin = [(I[i] if i == j else 0.0) + M * ((d2 if i == j else 0.0) - COM[i] * COM[j]) for i in range(3) for j in range(3)]
        b = onshape_mass.body(response(about_origin), "carriage", "origin")
        for i in range(3):
            for j in range(3):
                self.assertAlmostEqual(b.inertia[i][j], I[i] if i == j else 0.0, places=9)

    def test_missing_material_is_refused(self):
        r = response([I[0], 0, 0, 0, I[1], 0, 0, 0, I[2]])
        r["bodies"]["-all-"]["massMissingCount"] = 1
        with self.assertRaises(ValueError):
            onshape_mass.body(r, "payload")

    def test_cli_writes_cad_values(self):
        with tempfile.TemporaryDirectory() as d:
            src, out = Path(d, "carriage.json"), Path(d, "out.toml")
            src.write_text(json.dumps(response([I[0], 0, 0, 0, I[1], 0, 0, 0, I[2]])))
            self.assertEqual(onshape_mass.main(["--machine", "t-gantry", "--role", f"carriage={src}", "--out", str(out)]), 0)
            text = out.read_text(encoding="utf-8")
        self.assertIn("# [cad] onshape: m=47.1 kg", text)
        self.assertIn("carriage_kg = 47.1", text)

    def test_signature_follows_the_documented_scheme(self):
        h = onshape_mass.sign("GET", "https://cad.onshape.com/api/v9/partstudios/d/A/w/B/e/C/massproperties?x=1",
                              "ACCESS", "secret", date="Mon, 11 Apr 2016 20:08:56 GMT", nonce="0123456789abcdef0")
        import base64, hashlib, hmac
        text = "get\n0123456789abcdef0\nmon, 11 apr 2016 20:08:56 gmt\napplication/json\n/api/v9/partstudios/d/a/w/b/e/c/massproperties\nx=1\n"
        expected = base64.b64encode(hmac.new(b"secret", text.encode(), hashlib.sha256).digest()).decode()
        self.assertEqual(h["Authorization"], f"On ACCESS:HmacSHA256:{expected}")
        self.assertEqual(onshape_mass.endpoint("https://cad.onshape.com/documents/A1/w/B2/e/C3"),
                         "https://cad.onshape.com/api/v9/partstudios/d/A1/w/B2/e/C3/massproperties")


if __name__ == "__main__":
    unittest.main()
