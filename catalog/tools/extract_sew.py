"""Extracts the SEW-EURODRIVE CMP40 - CMP100 synchronous servomotors (400 V system voltage) into
catalog/sew/motors_cmp.csv.

The documents are not part of this repository. Get them from SEW-EURODRIVE, then:

    curl -L -o 27799468.pdf https://download.sew-eurodrive.com/download/pdf/27799468.pdf   (catalogue CMP40 - CMP112, 11/2023)
    curl -L -o 23061715.pdf https://download.sew-eurodrive.com/download/pdf/23061715.pdf   (product manual MOVIDRIVE system, 03/2017)
    pdftotext -raw 27799468.pdf cmp.txt                                  (xpdf 4.x)
    pdftotext -table -f 98 -l 101 23061715.pdf assign.txt
    python extract_sew.py cmp.txt assign.txt ../sew

Motor data: catalogue chapter 4.5 "Design for system voltage 400 V", the "Technical data" table of each size
(one column per motor and speed class, per-motor values printed once per motor). A row without one value per
column (or per motor) stops the script.
paired_drive: product manual chapter 3.5.6 "Motor-inverter assignments CMP.. motors, PWM 4 kHz", the smallest
MDX90A the table gives a peak torque for, at the motor's speed class.
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
SIZES = ["CMP40", "CMP50", "CMP63", "CMP71", "CMP80", "CMP100"]
# Catalogue 3.11 "Maximum motor speeds", motor without brake (min-1).
MAX_SPEED = {"CMP40": 9000, "CMP50": 9000, "CMP63": 7500, "CMP71": 7500, "CMP80": 7500, "CMP100": 5600}
# Row label -> key; "col": one value per motor and speed class, "motor": one per motor.
ROWS = [("Speed class nC", "n", "col"), ("Standstill torque M0", "m0", "motor"), ("Standstill current I0", "i0", "col"),
        ("Dynamic limit torque Mpk", "mpk", "motor"), ("Maximum motor current Imax", "imax", "col"),
        ("Inductance (phase) L1", "l1", "col"), ("Resistance (phase) at 20", "r1", "col"),
        ("Internal voltage at 1000 min-1 Vp0 cold", "vp0", "col"), ("Number of poles", "poles", "one"),
        ("Mass of the motor mmot", "mass", "motor"), ("Mass moment of inertia Jmot", "j", "motor"),
        ("Mass of the brakemotor mbmot", "massb", "motor"), ("Jbmot", "jb", "motor")]
NUMBER = re.compile(r"^\d+([.,]\d+)?$")
DRIVE = "MDX90A-{}-{}-4-S00"


def numbers(line):
    return [float(t.replace(",", ".")) for t in line.split() if NUMBER.match(t)]


def size_table(lines, size):
    start = next(i for i, l in enumerate(lines) if re.match(rf"^4\.5\.\d {size}$", l.strip()))
    end = next(i for i in range(start, len(lines)) if lines[i].startswith("Technical data of the brake"))
    block = lines[start:end]
    motors = block[2].split()  # "CMP50S CMP50M CMP50L"
    get = {}
    for label, key, kind in ROWS:
        at = next(i for i, l in enumerate(block) if l.startswith(label))
        values = numbers(block[at] if key == "poles" else block[at + 1])
        want = {"col": None, "motor": len(motors), "one": 1}[kind]
        get[key] = values
        if want and len(values) != want:
            raise ValueError(f"{size} {label}: {values}")
    cols = len(get["n"])
    for key in ("i0", "imax", "l1", "r1", "vp0"):
        if len(get[key]) != cols:
            raise ValueError(f"{size} {key}: {get[key]}")
    per = cols // len(motors)
    return motors, per, get


def pairings(lines):
    """{(motor, speed class): smallest MDX90A type with a peak torque in the assignment table}"""
    out, speed, cols = {}, None, None
    for line in lines:
        m = re.search(r"rated\s+speed\s+(\d+)\s+1/min", line)
        if m:
            speed = int(m.group(1))
            continue
        if "Inverter" in line and "MDX90A" in line:
            cols = [(t.start(), t.group()) for t in re.finditer(r"\b\d{4}\b", line)]
            continue
        m = re.match(r"^\s*(CMP\d+[SML])\s+Mpk\s+Nm\s+(\S+)", line)
        if m and cols:
            pos = m.start(2)
            first = min(cols, key=lambda c: abs(c[0] - pos))[1]
            out[(m.group(1), speed)] = first
    return out


def g(x):
    if x is None or x == "":
        return ""
    x = float(x)
    return str(int(x)) if x.is_integer() else repr(x)


def main(cmp_text, assign_text, out_dir):
    lines = Path(cmp_text).read_text(encoding="latin-1").splitlines()
    pairs = pairings(Path(assign_text).read_text(encoding="latin-1").splitlines())
    out = []
    for size in SIZES:
        motors, per, v = size_table(lines, size)
        for k, motor in enumerate(motors):
            for c in range(k * per, (k + 1) * per):
                n = v["n"][c]
                drive = pairs.get((motor, int(n)), "")
                if drive:
                    drive = DRIVE.format(drive, "5E3" if int(drive) <= 160 else "503")
                out.append({
                    "part_number": f"{motor}/{int(n)}", "vendor": "sew", "series": "CMP", "line_voltage_v": 400,
                    "static_torque_nm": g(v["m0"][k]), "stall_current_a": g(v["i0"][c]), "rated_torque_nm": "",
                    "rated_current_a": "", "rated_speed_rad_s": g(n * RPM), "rated_power_w": "",
                    "max_torque_nm": g(v["mpk"][k]), "max_current_a": g(v["imax"][c]),
                    "max_speed_rad_s": g(MAX_SPEED[size] * RPM), "torque_constant_nm_a": "",
                    "voltage_constant_v_s_rad": g(v["vp0"][c] / (1000 * RPM)),
                    "line_resistance_20c_ohm": g(round(2 * v["r1"][c], 9)),
                    "line_inductance_h": g(round(2 * v["l1"][c] * 1e-3, 12)),
                    "pole_pairs": g(v["poles"][0] / 2), "rotor_inertia_kgm2": g(round(v["j"][k] * 1e-4, 12)),
                    "rotor_inertia_with_brake_kgm2": g(round(v["jb"][k] * 1e-4, 12)),
                    "mass_kg": g(v["mass"][k]), "mass_with_brake_kg": g(v["massb"][k]),
                    "thermal_time_constant_s": "", "paired_drive": drive, "source": ""})
    with open(Path(out_dir) / "motors_cmp.csv", "w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, HEADER, lineterminator="\n")
        writer.writeheader()
        writer.writerows(out)
    print(f"{len(out)} rows, {sum(1 for r in out if r['paired_drive'])} paired")


if __name__ == "__main__":
    main(*sys.argv[1:4])
