"""Extracts the Kollmorgen AKM and AKM2G servo motors from their selection guides into
catalog/kollmorgen/motors_akm.csv and motors_akm2g.csv.

The guides are not part of this repository. Get the PDFs from Kollmorgen:
    AKM:   https://www.kollmorgen.com/sites/default/files/2025-04/AKM-KM_SG_000077_RevF_EN_0.pdf
    AKM2G: https://www.kollmorgen.com/sites/default/files/2025-04/AKM2G-KM_SG_000315_RevE_EN_0.pdf
then:

    pdftotext -q -table -enc UTF-8 AKM-KM_SG_000077_RevF_EN_0.pdf akm.txt          (xpdf 4.x)
    pdftotext -q -table -enc UTF-8 AKM2G-KM_SG_000315_RevE_EN_0.pdf akm2g.txt
    python extract_kollmorgen.py akm.txt akm2g.txt ../kollmorgen

Each frame family has a "Performance Data" table and a "Motor Parameters" table with one column per winding
(frame-wide values such as inertia have one column per frame). The AKM2G guide also has the "System Performance"
tables that pair each winding with AKD drives per line voltage. A row that does not have one value per column
stops the script.
"""
import csv
import math
import re
import sys
from pathlib import Path

HEADER = ["part_number", "vendor", "series", "line_voltage_v", "static_torque_nm", "stall_current_a",
          "rated_torque_nm", "rated_current_a", "rated_speed_rad_s", "rated_power_w", "max_torque_nm",
          "max_current_a", "max_speed_rad_s", "torque_constant_nm_a", "voltage_constant_v_s_rad",
          "line_resistance_20c_ohm", "line_inductance_h", "pole_pairs", "rotor_inertia_kgm2",
          "rotor_inertia_with_brake_kgm2", "mass_kg", "mass_with_brake_kg", "thermal_time_constant_s",
          "paired_drive", "source"]
RPM = math.pi / 30
FRAME = re.compile(r"^AKM(2G-)?\d\d$")
VOLTAGE = re.compile(r"^(\d+) V(ac|dc)")
NUMBER = re.compile(r"^-?\d*\.?\d+(E[-+]?\d+)?$")
PRIVATE_USE = re.compile("[%s-%s]" % (chr(0xE000), chr(0xF8FF)))  # footnote glyphs glued to a few values
BRAKE_MASS = re.compile(r"(Brake option increases weight by|Brake motor adds) ([\d.]+) kg")
SYSTEM_ROW = re.compile(r"^(?:(\d+) V )?(AKM2G-\d\d[A-Z]) (\S+) \d+ ([\d.]+) \([\d.]+\) ([\d.]+) \(")


def last(line, count, what):
    """The last `count` values of a table row ("-": not given)."""
    tokens = line.split()[-count:]
    if len(tokens) != count or not all(t == "-" or NUMBER.match(t) for t in tokens):
        raise ValueError(f"{what}: expected {count} values, found {line.split()}")
    return [None if t == "-" else float(t) for t in tokens]


def columns(lines, at, known):
    """The windings (part numbers) of the table whose header line is lines[at], the frame of each, the frames.
    `known`: {frames: (windings, frame of each)} of the performance tables read so far; a parameter table with as
    many columns takes the windings of the performance table of its frames (the AKM2G-7x one misprints a letter)."""
    words = lines[at].split()
    letters = [t.rstrip("*") for t in words[words.index("Units") + 1:]]  # "N*": a UL footnote
    frames = next(l.split() for l in reversed(lines[at - 3:at]) if l.split() and all(FRAME.match(t) for t in l.split()))
    if tuple(frames) in known and len(known[tuple(frames)][0]) == len(letters):
        windings, frame_of = known[tuple(frames)]
        if [w[len(f):] for w, f in zip(windings, frame_of)] != letters:
            print(f"line {at + 1}: header {letters} read as the performance table's {windings}")
        return windings, frame_of, frames
    if all(re.match(r"^AKM\d\d[A-Z]$", t) for t in letters):  # AKM8x: the full name in the header
        return letters, [t[:5] for t in letters], frames
    out, frame_of, i = [], [], -1
    for j, letter in enumerate(letters):
        if j == 0 or letter <= letters[j - 1]:  # the letters ascend within a frame
            i += 1
        out.append(frames[i] + letter)
        frame_of.append(frames[i])
    if i != len(frames) - 1:
        raise ValueError(f"line {at + 1}: windings {letters} do not fit frames {frames}")
    known.setdefault(tuple(frames), (out, frame_of))
    return out, frame_of, frames


def table(lines, at):
    """The non-empty lines of the table from its header to the next header, with single spaces."""
    end = next((k for k in range(at + 1, len(lines)) if re.search(r"Parameters\s+.*\bUnits\b", lines[k])), len(lines))
    return [" ".join(PRIVATE_USE.sub("", line).split()) for line in lines[at + 1:end] if line.strip()]


def performance(lines, at, motors, known):
    windings, _, _ = columns(lines, at, known)
    n = len(windings)
    body = table(lines, at)
    get = {}
    for line in body:
        s = line.split()
        # by symbol and unit: a label may stand on the line before or after its values
        unit = s[-n - 1] if len(s) > n else None
        if "Nmax" in s and unit == "rpm":
            get["nmax"] = last(line, n, "Nmax")
        elif unit == "Nm" and "tcs" not in get:  # the first Nm row: continuous torque at 100 K
            get["tcs"] = last(line, n, "Tcs")
        elif "Ics" in s and unit == "Arms":
            get["ics"] = last(line, n, "Ics")
        elif "Tp" in s and unit == "Nm":
            get["tp"] = last(line, n, "Tp")
        elif "Ip" in s and unit == "Arms":
            get["ip"] = last(line, n, "Ip")
    for key in ("nmax", "tcs", "ics", "tp", "ip"):
        if key not in get:
            raise ValueError(f"line {at + 1}: no {key} row")
    rated = {}
    for b in [i for i, line in enumerate(body) if "Rated Torque" in line and line.split()[-n - 1] == "Nm"]:
        voltage = next(int(m.group(1)) for line in body[b:b + 5] if (m := VOLTAGE.match(line)))
        speed = next((line for line in body[b:b + 5] if "rpm" in line.split()), None)
        power = next((line for line in body[b:b + 5] if "kW" in line.split()), None)
        if speed is None or power is None:
            raise ValueError(f"line {at + 1}: the {voltage} V block has no speed or power row")
        rated[voltage] = (last(body[b], n, "Trtd"), last(speed, n, "Nrtd"), last(power, n, "Prtd"))
    for j, w in enumerate(windings):
        m = motors.setdefault(w, {})
        for key, values in get.items():
            m[key] = values[j]
        m["rated"] = {v: tuple(x[j] for x in r) for v, r in rated.items()}


def parameters(lines, at, motors, known):
    windings, frame_of, frames = columns(lines, at, known)
    n, f = len(windings), len(frames)
    get, inertia = {}, []
    for line in table(lines, at):
        s = line.split()
        if "Nm/Arms" in s:
            get["kt"] = last(line, n, "Kt")
        elif "Vrms/krpm" in s or "Vrms/" in s:  # the unit may wrap after "Vrms/"
            get["ke"] = last(line, n, "Ke")
        elif "Resistance" in line and "ohm" in line.lower().split():
            get["r"] = last(line, n, "Rm")
        elif "Inductance" in line and "mH" in s and "l" not in get:  # AKM2G: the Q-axis one comes first
            get["l"] = last(line, n, "L")
        elif "kg-cm2" in s:
            inertia.append(last(line, f, "Jm"))  # the rotor's, then the brake's (additional)
        elif "Weight" in line and "kg" in s:
            get["mass"] = last(line, f, "W")
        elif "minutes" in s:
            get["tct"] = last(line, f, "TCT")
        elif line.startswith("Pole Pairs"):
            get["pp"] = last(line, f, "pole pairs")
        elif m := BRAKE_MASS.search(line):
            get["brake_mass"] = float(m.group(2))
            break
    if len(inertia) != 2 or len(get) != 8:
        raise ValueError(f"line {at + 1}: parameter rows missing, found {sorted(get)} and {len(inertia)} inertia rows")
    for j, w in enumerate(windings):
        k = frames.index(frame_of[j])
        motors.setdefault(w, {}).update(
            kt=get["kt"][j], ke=get["ke"][j], r=get["r"][j], l=get["l"][j], jm=inertia[0][k], jb=inertia[1][k],
            mass=get["mass"][k], tct=get["tct"][k], pp=get["pp"][k], brake_mass=get["brake_mass"])


def pairings(lines):
    """{(winding, line voltage): AKD drive} from the AKM2G "System Performance" tables: of the AKD drives listed
    for the winding in the table of that voltage, the smallest that reaches the full continuous and peak torque
    of the winding's rows (within 1 %: the table prints the same torque rounded differently). A table runs on
    over pages; the next one starts where the windings start again from the smallest; its voltage is printed once,
    in front of one of its rows."""
    sections, current, previous = [], None, None
    for line in lines:
        m = SYSTEM_ROW.match(" ".join(line.split()))
        if not m:
            continue
        voltage, winding, drives, tcs, tp = m.groups()
        if current is None or winding < previous:
            current = {"voltage": None, "rows": []}
            sections.append(current)
        previous = winding
        if voltage:
            current["voltage"] = int(voltage)
        akd = drives.split("/")[-1]
        if akd.startswith("AKD-"):
            current["rows"].append((winding, akd, float(tcs), float(tp)))
    out = {}
    for section in sections:
        if section["voltage"] is None:
            raise ValueError(f"a system performance table without voltage: {section['rows'][:3]}")
        for winding in {r[0] for r in section["rows"]}:
            listed = sorted((r for r in section["rows"] if r[0] == winding), key=lambda r: int(r[1][-4:-2]))
            tcs, tp = max(r[2] for r in listed), max(r[3] for r in listed)
            out[winding, section["voltage"]] = next(r[1] for r in listed if r[2] >= 0.99 * tcs and r[3] >= 0.99 * tp)
    return out


def g(x):
    """A number as Python prints it, integers without ".0"."""
    if x is None:
        return ""
    x = float(x)
    return str(int(x)) if x.is_integer() else repr(x)


def rows(motors, series, paired):
    out = []
    for w in sorted(motors):
        m = motors[w]
        rated = m["rated"]
        voltages = [v for v in (240, 400) if v in rated and rated[v][0] is not None]
        if 400 not in voltages and 480 in rated and rated[480][0] is not None:
            voltages.append(480)
        for v in voltages:
            t, nr, p = rated[v]
            out.append({
                "part_number": w, "vendor": "kollmorgen", "series": series, "line_voltage_v": v,
                "static_torque_nm": g(m["tcs"]), "stall_current_a": g(m["ics"]), "rated_torque_nm": g(t),
                "rated_current_a": "", "rated_speed_rad_s": g(nr * RPM), "rated_power_w": g(round(p * 1000, 6)),
                "max_torque_nm": g(m["tp"]), "max_current_a": g(m["ip"]), "max_speed_rad_s": g(m["nmax"] * RPM),
                "torque_constant_nm_a": g(m["kt"]), "voltage_constant_v_s_rad": g(m["ke"] / (1000 * RPM)),
                "line_resistance_20c_ohm": g(m["r"]), "line_inductance_h": g(round(m["l"] * 1e-3, 12)),
                "pole_pairs": g(m["pp"]), "rotor_inertia_kgm2": g(round(m["jm"] * 1e-4, 12)),
                "rotor_inertia_with_brake_kgm2": g(round((m["jm"] + m["jb"]) * 1e-4, 12)),
                "mass_kg": g(m["mass"]), "mass_with_brake_kg": g(round(m["mass"] + m["brake_mass"], 6)),
                "thermal_time_constant_s": g(m["tct"] * 60), "paired_drive": paired.get((w, v), ""), "source": ""})
    return out


def guide(text_file):
    lines = Path(text_file).read_text(encoding="utf-8").splitlines()
    motors, known = {}, {}
    for i, line in enumerate(lines):
        if not re.search(r"Parameters\s+Tol\s+Sym(bol)?\s+Units", line):
            continue
        following = " ".join(lines[i + 1:i + 6])
        if "Max Rated" in following:
            performance(lines, i, motors, known)
        elif "Torque Constant" in following:
            parameters(lines, i, motors, known)
    incomplete = [w for w, m in motors.items() if "rated" not in m or "kt" not in m]
    if incomplete:
        raise ValueError(f"windings in only one of the two tables: {incomplete}")
    return lines, motors


def write(path, out):
    with open(path, "w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, HEADER, lineterminator="\n")
        writer.writeheader()
        writer.writerows(out)
    print(f"{path}: {len(out)} rows")


def main(akm_text, akm2g_text, out_dir):
    _, motors = guide(akm_text)
    write(Path(out_dir) / "motors_akm.csv", rows(motors, "AKM", {}))
    lines, motors = guide(akm2g_text)
    write(Path(out_dir) / "motors_akm2g.csv", rows(motors, "AKM2G", pairings(lines)))


if __name__ == "__main__":
    main(*sys.argv[1:4])
