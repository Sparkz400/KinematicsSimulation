"""Prototype: Onshape Part Studio mass properties -> a machine config with [cad] values. See docs/integrations.md.

    python tools/cad/onshape_mass.py --machine t-gantry --role carriage=carriage.json --role payload=URL ... --out OUT.toml

One Part Studio per role: its "-all-" body (every part with a material in it) becomes that role. A source is either
a saved response of the massproperties endpoint (works without credentials: save it from Onshape's API Explorer
while signed in) or a Part Studio URL https://cad.onshape.com/documents/<did>/w/<wid>/e/<eid>, fetched with the API
keys in ONSHAPE_ACCESS_KEY and ONSHAPE_SECRET_KEY.

shortcut: the URL path is untested (no keys in this project), and Onshape's documentation does not say whether the
tensor is about the centroid or the origin; --inertia-about picks it. Verify both against a box once keys exist.
"""

import argparse
import base64
import email.utils
import hashlib
import hmac
import json
import os
import re
import secrets
import sys
import urllib.parse
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import cadmass  # noqa: E402

API = "https://cad.onshape.com/api/v9"
URL = re.compile(r"https://([^/]+)/documents/(\w+)/([wvm])/(\w+)/e/(\w+)")


def sign(method, url, access, secret, date=None, nonce=None, content_type="application/json"):
    """Headers of Onshape's API key scheme: HMAC-SHA256 of method, nonce, date, content type, path and query,
    each followed by a newline, all lowercase, keyed with the secret key, base64."""
    date = date or email.utils.formatdate(usegmt=True)
    nonce = nonce or secrets.token_hex(13)
    parts = urllib.parse.urlsplit(url)
    text = "\n".join([method, nonce, date, content_type, parts.path, parts.query, ""]).lower()
    digest = hmac.new(secret.encode(), text.encode(), hashlib.sha256).digest()
    return {"Date": date, "On-Nonce": nonce, "Content-Type": content_type, "Accept": "application/json",
            "Authorization": f"On {access}:HmacSHA256:{base64.b64encode(digest).decode()}"}


def endpoint(document_url):
    m = URL.match(document_url)
    if not m:
        raise ValueError(f"not a Part Studio URL: {document_url}")
    _, did, wvm, wvmid, eid = m.groups()
    return f"{API}/partstudios/d/{did}/{wvm}/{wvmid}/e/{eid}/massproperties"


def fetch(document_url):
    access, secret = os.environ.get("ONSHAPE_ACCESS_KEY"), os.environ.get("ONSHAPE_SECRET_KEY")
    if not access or not secret:
        raise RuntimeError("set ONSHAPE_ACCESS_KEY and ONSHAPE_SECRET_KEY, or pass a saved response file")
    url = endpoint(document_url)
    with urllib.request.urlopen(urllib.request.Request(url, headers=sign("GET", url, access, secret))) as r:
        return json.load(r)


def body(response, role, inertia_about="centroid"):
    """A Body of the "-all-" entry. Each quantity is a value followed by its lower and upper bound: the value is
    the first number (mass), the first 3 (centroid) or the first 9 (inertia, row by row). SI units."""
    entry = response["bodies"]["-all-"]
    if not entry.get("hasMass", True) or entry.get("massMissingCount", 0):
        raise ValueError(f"{role}: {entry.get('massMissingCount')} part(s) without a material; Onshape gives no mass for them")
    mass, com = entry["mass"][0], tuple(entry["centroid"][:3])
    inertia = [entry["inertia"][3 * i:3 * i + 3] for i in range(3)]
    if inertia_about == "origin":
        # Back from the origin to the centroid: the parallel-axis theorem the other way.
        d2 = sum(c * c for c in com)
        inertia = [[inertia[i][j] - mass * ((d2 if i == j else 0.0) - com[i] * com[j]) for j in range(3)] for i in range(3)]
    return cadmass.Body(role, role, mass, com, inertia)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--machine", required=True)
    p.add_argument("--role", action="append", required=True, help="ROLE=FILE.json or ROLE=Part Studio URL")
    p.add_argument("--inertia-about", choices=["centroid", "origin"], default="centroid")
    p.add_argument("--config")
    p.add_argument("--out", required=True)
    a = p.parse_args(argv)
    try:
        roles = {}
        for given in a.role:
            role, _, source = given.partition("=")
            response = fetch(source) if source.startswith("https://") else json.loads(Path(source).read_text(encoding="utf-8"))
            roles[role] = body(response, role, a.inertia_about)
        config = Path(a.config) if a.config else cadmass.default_config(a.machine)
        text = config.read_bytes().decode("utf-8")
        Path(a.out).write_bytes(cadmass.apply(text, a.machine, roles, "onshape").encode("utf-8"))
    except (ValueError, RuntimeError, OSError, KeyError) as e:
        print(f"onshape_mass: {e}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
