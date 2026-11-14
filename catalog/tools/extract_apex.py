"""Extracts two Apex Dynamics planetary gearbox series into catalog/apex/:

- gearbox_peii.csv: PEII, the economy line ("economic", New Generation P series), from the catalogue
  "Planetary Gearbox New Generation P-Series", APEX-2025-06-PII/PIIR/PIIS SERIES-TWN, printed pages 02 to 05
  (PEII / PEIIR Gearbox Performance, PEII Gearbox Inertia).
  https://www.apexdyna.com/uploadimages/catalog/11/Gearbox-PII-PIIR-Series%20(1)-tw-20260729-EV1.pdf
- gearbox_af.csv: AF, a "high precision high speed" line, from the catalogue "AF / AFR Series",
  APEX-2025-02-AF/AFR SERIES-1.0-TWN, printed pages 2 to 6 (Specifications / AF Series, Gearbox Inertia,
  Dimensions 1-stage and 2-stage, row C3).
  https://www.apexdyna.com/uploadimages/catalog/15/Gearbox-AF-AFR-Series-tw-20260729-EV1.pdf

The PDFs are not part of this repository. Get them from the URLs above, then (xpdf 4.x):

    pdftotext -table -enc UTF-8 pii.pdf pii.txt
    pdftotext -table -enc UTF-8 af.pdf af.txt
    python tools/extract_apex.py pii.txt af.txt apex

Max. acceleration torque and emergency stop torque are not printed per row: both catalogues give
T2NOT = 3 x T2N and T2B = 60 % of T2NOT; the script applies these rules. Any row with the wrong number of
values, a missing table or an unexpected ratio list stops the script.
"""
import csv
import re
import sys
from decimal import Decimal
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from cataloglib.units import to_si  # noqa: E402

COLUMNS = ["series", "size", "motor_frame", "clamping_hub_diameter_m", "stages", "ratio",
           "rated_output_torque_nm", "max_output_torque_nm", "emergency_stop_output_torque_nm",
           "rated_input_speed_rad_s", "max_input_speed_rad_s", "no_load_torque_nm", "no_load_torque_speed_rad_s",
           "backlash_rad", "backlash_reduced_rad", "torsional_stiffness_nm_rad", "max_radial_force_n",
           "max_axial_force_n", "max_tilting_moment_nm", "efficiency_full_load", "mass_kg", "inertia_at_input_kgm2"]

NUMBER = re.compile(r"^\d[\d,]*(\.\d+)?$")
PEII_SIZES = ["PEII050", "PEII070", "PEII090", "PEII120", "PEII155"]
PEII_RATIOS = {1: [3, 4, 5, 7, 10], 2: [12, 15, 16, 20, 25, 30, 35, 40, 50, 70, 100]}  # ordering code, PEII
PEII_BORES = ["8", "11", "14", "19", "24", "28", "32", "35", "38", "42"]
AF_SIZES = ["AF042", "AF060", "AF060A", "AF075", "AF075A", "AF100", "AF100A", "AF140", "AF140A", "AF180", "AF220"]
AF_ONE_STAGE = ["AF042", "AF060", "AF075", "AF100", "AF140", "AF180", "AF220"]  # the "A" sizes are 2-stage only
AF_RATIOS = {1: [3, 4, 5, 6, 7, 8, 9, 10],
             2: [12, 15, 16, 20, 25, 28, 30, 32, 35, 40, 45, 50, 60, 70, 80, 90, 100]}


def clean(line):
    return line.replace("≦", " ").replace("≤", " ").replace("≧", " ").replace("≥", " ")


def trailing(line):
    """The run of numbers at the end of a line, thousands commas removed."""
    out = []
    for token in reversed(clean(line).split()):
        if not NUMBER.match(token):
            break
        out.append(token.replace(",", ""))
    return out[::-1]


def section(lines, start, end):
    i = next((k for k, l in enumerate(lines) if start in l), None)
    if i is None:
        raise ValueError(f"table '{start}' missing")
    j = next((k for k in range(i + 1, len(lines)) if end in lines[k]), None)
    if j is None:
        raise ValueError(f"end '{end}' of table '{start}' missing")
    return lines[i:j]


def labelled(lines, label, n):
    for k, line in enumerate(lines):
        if line.strip().startswith(label):
            vals = trailing(line)
            if len(vals) != n:
                raise ValueError(f"{label}: {len(vals)} values {vals}, expected {n}")
            return k, vals
    raise ValueError(f"row '{label}' missing")


def ratio_rows(lines, wide, narrow):
    """Rows 'ratio v1 .. vn' (a stage number may come before): 1-stage rows have `narrow` values,
    2-stage rows `wide` values. Returns {ratio: values}."""
    out = {}
    for line in lines:
        t = trailing(line.replace(" All ", " "))
        if len(t) >= wide + 1:
            n = wide
        elif len(t) >= narrow + 1:
            n = narrow
        else:
            continue
        ratio, vals = int(t[-n - 1]), t[-n:]
        if ratio in out:
            raise ValueError(f"ratio {ratio} twice")
        out[ratio] = vals
    return out


def check_ratios(found, expected, where):
    want = expected[1] + expected[2]
    if sorted(found) != want:
        raise ValueError(f"{where}: ratios {sorted(found)}, expected {want}")


def times(text, factor):
    return format((Decimal(text) * Decimal(factor)).normalize(), "f")


def stage(ratio):
    return 1 if ratio <= 10 else 2


def peii(lines):
    perf = section(lines, "PEII / PEIIR Gearbox Performance", "(1) Ratio")
    k_em = next(k for k, l in enumerate(perf) if "Emergency Stop Torque" in l)
    torque = ratio_rows(perf[:k_em], 5, 5)
    check_ratios(torque, PEII_RATIOS, "PEII T2N")
    get = lambda label: labelled(perf, label, 5)[1]
    rigidity, n1n, n1b = get("Torsional Rigidity"), get("Nominal Input Speed"), get("Max. Input Speed")
    radial, axial, tilt = get("Max. Radial Load"), get("Max. Axial Load"), get("Max. Tilting Moment")
    # Lines '<stage> <ratios> PE II <values>': no-load torque, backlash, efficiency, each for stages 1 and 2.
    staged = [m for m in (re.match(r"^\s*([12])\s+(3~10|12~100)\s+PE II\s+(?!R)(.*)$", l) for l in perf) if m]
    if [m[1] for m in staged] != ["1", "2"] * 3:
        raise ValueError(f"PEII: staged rows {[m[0] for m in staged]}")
    no_load = {int(m[1]): trailing(m[3]) for m in staged[0:2]}
    backlash = {int(m[1]): trailing(m[3]) for m in staged[2:4]}
    if not all("≦" in m[3] for m in staged[2:4]):
        raise ValueError("PEII: backlash rows not where expected")
    eff = {int(m[1]): re.fullmatch(r"\s*≧\s*(\d+)\s*[％%]\s*", m[3]) for m in staged[4:6]}
    for s in (1, 2):
        if len(no_load[s]) != 5 or len(backlash[s]) != 5 or not eff[s]:
            raise ValueError(f"PEII stage {s}: no-load {no_load[s]}, backlash {backlash[s]}, efficiency {eff[s]}")

    inertia_lines = section(lines, "PEII          Gearbox Iner", "PEIIR Gearbox Inertia")
    inertia, pending = {}, []
    for line in inertia_lines:
        tokens = line.replace("kg.cm", " ").split()
        if tokens and all(t == "-" for t in tokens):
            pending = tokens  # a row broken over two lines: its dashes come first
            continue
        if tokens and tokens[0] in PEII_BORES:
            vals = pending + tokens[1:]
            if tokens[0] == "24":
                vals = pending + tokens[2:]  # '24 kg.cm 2 ...': drop the unit's exponent
            pending = []
            if len(vals) != 10:
                raise ValueError(f"PEII inertia bore {tokens[0]}: {len(vals)} values {vals}")
            inertia[tokens[0]] = vals
    if list(inertia) != PEII_BORES:
        raise ValueError(f"PEII inertia bores {list(inertia)}")

    rows = []
    for c, size in enumerate(PEII_SIZES):
        for ratio in PEII_RATIOS[1] + PEII_RATIOS[2]:
            s = stage(ratio)
            t2n = torque[ratio][c]
            for bore in PEII_BORES:
                j = inertia[bore][2 * c + s - 1]
                if j == "-":
                    continue  # this bore is not offered for this size and stage
                rows.append({
                    "series": "PEII", "size": size, "motor_frame": bore,
                    "clamping_hub_diameter_m": to_si(bore, "milli"), "stages": str(s), "ratio": str(ratio),
                    "rated_output_torque_nm": t2n, "max_output_torque_nm": times(t2n, "1.8"),
                    "emergency_stop_output_torque_nm": times(t2n, "3"),
                    "rated_input_speed_rad_s": to_si(n1n[c], "rpm"), "max_input_speed_rad_s": to_si(n1b[c], "rpm"),
                    "no_load_torque_nm": no_load[s][c], "no_load_torque_speed_rad_s": "",
                    "backlash_rad": to_si(backlash[s][c], "arcmin"), "backlash_reduced_rad": "",
                    "torsional_stiffness_nm_rad": to_si(rigidity[c], "per_arcmin"),
                    "max_radial_force_n": radial[c], "max_axial_force_n": axial[c], "max_tilting_moment_nm": tilt[c],
                    "efficiency_full_load": to_si(eff[s][1], "percent"), "mass_kg": "",
                    "inertia_at_input_kgm2": to_si(j, "kgcm2"),
                })
    return rows


def af(lines):
    perf = section(lines, "Specifications / AF Series", "(1) Ratio")
    k_em = next(k for k, l in enumerate(perf) if "Emergency Stop Torque" in l)
    torque = ratio_rows(perf[:k_em], 11, 7)
    check_ratios(torque, AF_RATIOS, "AF T2N")
    get = lambda label: labelled(perf, label, 11)[1]
    n1n, n1b, rigidity = get("Nominal Input Speed"), get("Max. Input Speed"), get("Torsional Rigidity")
    radial, axial, tilt = get("Max. Radial Load"), get("Max. Axial Load"), get("Max.Tilting Moment")

    def two_stage(label):
        k, one = labelled(perf, label, 7)
        two = trailing(perf[k + 1])
        if len(two) != 11 or not perf[k + 1].split()[0] == "2":
            raise ValueError(f"{label}: 2-stage row {perf[k + 1]!r}")
        return {1: dict(zip(AF_ONE_STAGE, one)), 2: dict(zip(AF_SIZES, two))}

    reduced, standard, mass = two_stage("Reduced Backlash P1"), two_stage("Standard Backlash P2"), two_stage("Weight")
    k = next(k for k, l in enumerate(perf) if l.strip().startswith("Efﬁciency") or l.strip().startswith("Efficiency"))
    eff = {}
    for line in perf[k:k + 2]:
        m = re.search(r"\b([12])\s+(3~10|12~100)\s+≥\s*(\d+)%", line)
        if not m:
            raise ValueError(f"AF efficiency row {line!r}")
        eff[int(m[1])] = m[3]

    inertia = ratio_rows(section(lines, "Gearbox Inertia", "Dimensions (1-stage"), 11, 7)
    check_ratios(inertia, AF_RATIOS, "AF inertia")

    # C3, the clamping hub bore, standard value first; '/ <option>' and footnote digits dropped.
    c3 = [l for l in lines if re.match(r"^\s*C3\s*\d?\s+≤", l)][:2]
    bores = [re.findall(r"≤\s*(\d+(?:\.\d+)?)", re.sub(r"/\s*≤\s*[\d.]+", "", l)) for l in c3]
    if len(c3) != 2 or len(bores[0]) != 7 or len(bores[1]) != 11:
        raise ValueError(f"AF C3 rows {c3}")
    bore = {1: dict(zip(AF_ONE_STAGE, bores[0])), 2: dict(zip(AF_SIZES, bores[1]))}

    rows = []
    for c, size in enumerate(AF_SIZES):
        for ratio in AF_RATIOS[1] + AF_RATIOS[2]:
            s = stage(ratio)
            if size not in bore[s]:
                continue
            col = AF_ONE_STAGE.index(size) if s == 1 else c
            t2n, b = torque[ratio][col], bore[s][size]
            rows.append({
                "series": "AF", "size": size, "motor_frame": b, "clamping_hub_diameter_m": to_si(b, "milli"),
                "stages": str(s), "ratio": str(ratio),
                "rated_output_torque_nm": t2n, "max_output_torque_nm": times(t2n, "1.8"),
                "emergency_stop_output_torque_nm": times(t2n, "3"),
                "rated_input_speed_rad_s": to_si(n1n[c], "rpm"), "max_input_speed_rad_s": to_si(n1b[c], "rpm"),
                "no_load_torque_nm": "", "no_load_torque_speed_rad_s": "",
                "backlash_rad": to_si(standard[s][size], "arcmin"),
                "backlash_reduced_rad": to_si(reduced[s][size], "arcmin"),
                "torsional_stiffness_nm_rad": to_si(rigidity[c], "per_arcmin"),
                "max_radial_force_n": radial[c], "max_axial_force_n": axial[c], "max_tilting_moment_nm": tilt[c],
                "efficiency_full_load": to_si(eff[s], "percent"), "mass_kg": mass[s][size],
                "inertia_at_input_kgm2": to_si(inertia[ratio][col], "kgcm2"),
            })
    return rows


def write(rows, path, columns):
    keys = [(r["size"], r["motor_frame"], r["ratio"]) for r in rows]
    if len(set(keys)) != len(keys):
        raise ValueError(f"{path.name}: duplicate keys")
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, columns, lineterminator="\n", extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)
    print(f"{path.name}: {len(rows)} rows")


def main():
    pii_txt, af_txt, out = Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3])
    out.mkdir(parents=True, exist_ok=True)
    used = lambda rows: [c for c in COLUMNS if any(r[c] for r in rows)]
    read = lambda path: [l for l in path.read_text(encoding="utf-8").splitlines() if l.strip()]
    p, a = peii(read(pii_txt)), af(read(af_txt))
    write(p, out / "gearbox_peii.csv", used(p))
    write(a, out / "gearbox_af.csv", used(a))


if __name__ == "__main__":
    main()
