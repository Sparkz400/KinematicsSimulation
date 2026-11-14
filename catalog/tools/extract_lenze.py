"""Extracts the Lenze MCS and m850 servo motors and the i950 servo inverters into catalog/lenze/.

The documents are not part of this repository. Get them from Lenze's documentation finder
(lenze.com, Services, Knowledge base, Product-related documentation), then:

    pdftotext -table "Projektierung_Servo-Synchronmotoren MCS_EN.pdf" mcs.txt              (xpdf 4.x)
    pdftotext -table "Projektierung_Servo-Synchronmotoren m850_EN.pdf" m850.txt
    pdftotext -table "Projektierung_Servoumrichter i950 cabinet_EN_Gen_B.pdf" i950.txt
    python extract_lenze.py mcs.txt m850.txt i950.txt ../lenze

The rated data tables (one column per motor or inverter) are read from the text. Cells the PDF merges
over several columns (brake inertia, additional weights, the chopper's minimum resistance) and the
m850 selection table, whose rows pdftotext puts out of order, are typed in below with their page.
A table row without one value per column stops the script.
"""
import math
import re
import sys
from decimal import Decimal
from pathlib import Path

MOTOR_HEADER = ("part_number,vendor,series,line_voltage_v,static_torque_nm,stall_current_a,rated_torque_nm,"
                "rated_current_a,rated_speed_rad_s,rated_power_w,max_torque_nm,max_current_a,max_speed_rad_s,"
                "torque_constant_nm_a,voltage_constant_v_s_rad,line_resistance_20c_ohm,line_inductance_h,pole_pairs,"
                "rotor_inertia_kgm2,rotor_inertia_with_brake_kgm2,mass_kg,mass_with_brake_kg,"
                "thermal_time_constant_s,paired_drive,source").split(",")
DRIVE_HEADER = ("part_number,vendor,series,supply,line_voltage_v,axes,rated_output_current_a,max_output_current_a,"
                "max_current_duration_s,dc_link_voltage_v,dc_link_capacitance_f,braking_resistor_internal_ohm,"
                "braking_resistor_continuous_w,braking_resistor_peak_w,min_braking_resistance_ohm,braking_threshold_v,"
                "dc_link_overvoltage_v,source").split(",")
# Rated data rows: label at the start of the line -> key.
ROWS = {"Standstill torque": "m0", "Rated torque": "mn", "Max. torque": "mmax", "Rated speed": "nn",
        "Max. speed": "nmax", "Rated power": "pn", "Standstill current": "i0", "Rated current": "in",
        "Max. current": "imax", "Moment of inertia": "j", "Torque constant": "kt", "Voltage constant": "ke",
        "Stator terminal resistance  RUV   20": "r20", "Stator inductance": "l", "Weight": "m"}

# MCS, pages 132-133 "Permanent magnet brakes": JMB (brake motor, kg cm2) of the standard brake (motor code P1);
# MCS19J and MCS19P have only the increased brake (P2). Page 117 "Additional weights" (kg): standard brake,
# MCS19J/P the increased one (the only one they have).
MCS_JMB = {"06C": "0.26", "06F": "0.34", "06I": "0.42", "09D": "2.17", "09F": "2.57", "09H": "2.97", "09L": "3.87",
           "12D": "5.07", "12H": "8.4", "12L": "11.7", "14D": "11.3", "14H": "17.4", "14L": "26.6", "14P": "37.9",
           "19F": "77", "19J": "135", "19P": "190"}
MCS_BRAKE_KG = {"06": "0.3", "09": "0.8", "12": "0.9", "14": "1.9", "19F": "3.1", "19J": "4.3", "19P": "4.3"}
# m850, pages 67-68 "Permanent magnet brakes" / "Spring-applied brakes": JMB; page 57 "Additional weights".
M850_JMB = {"S40/S9000": "0.055", "S40/L9000": "0.079", "S60/S6000": "0.319", "S60/L6000": "0.512",
            "S80/S5500": "1.68", "S80/L5500": "2.2", "S120/S3960": "7.5", "S120/M3960": "13.3", "S120/L3960": "19.2",
            "S140/S3240": "18.1", "S140/M3240": "32.5", "S140/L3240": "46.9", "S190/S3000": "72.6",
            "S190/M3000": "128.9", "S190/L2520": "204.3"}
M850_BRAKE_KG = {"S40": "0.085", "S60": "0.2", "S80": "0.4", "S120": "2.0", "S140": "3.0", "S190": "6.70"}
# m850, pages 37-38, i950 selection table at 3x 400 V: the smallest inverter whose Mmax reaches the motor's Mmax.
# The table has no m850-S40, S60, S80.
M850_PAIR = {"S120/S3960": "i950-C4.0/400-3", "S120/M3960": "i950-C7.5/400-3", "S120/L3960": "i950-C11/400-3",
             "S140/S3240": "i950-C7.5/400-3", "S140/M3240": "i950-C11/400-3", "S140/L3240": "i950-C22/400-3",
             "S190/S3000": "i950-C15/400-3", "S190/M3000": "i950-C22/400-3", "S190/L2520": "i950-C30/400-3"}
# i950 Gen B, pages 130-131 "Brake chopper" (400 V): minimum brake resistance; the PDF merges equal cells.
I950_RMIN = {"0.55": "390", "0.75": "390", "2.2": "150", "4.0": "47", "7.5": "27", "11": "27", "15": "18",
             "22": "15", "30": "7.5", "45": "7.5", "55": "4.7", "75": "4.7", "90": "2.3", "110": "2.3",
             "130": "2.8", "160": "2.3", "200": "1.8", "250": "1.4"}


def lines(path):
    """The non-empty lines of a pdftotext -table file."""
    return [l for l in Path(path).read_text(encoding="latin-1").split("\n") if l.strip()]


def si(value, factor):
    """value * factor as a float without binary noise (0.14 kg cm2 -> 1.4e-05)."""
    return repr(float(Decimal(value) * Decimal(factor)))


def rated_blocks(text, name_re, stop):
    """Rated data blocks: {motor: {key: value}} up to the line containing `stop`."""
    motors = {}
    names = []
    for i, line in enumerate(text):
        if stop in line and motors:  # the stop text is in the contents too
            break
        if line.startswith("Motor "):
            names = re.findall(name_re, line)
            if names and names[0].endswith("/"):  # m850: "m850-S40/" with the winding on the next line
                names = [n + w for n, w in zip(names, text[i + 1].split())]
            names = [n.replace(" ", "") for n in names]
            if not any(l.startswith("Standstill torque") for l in text[i + 1:i + 3]):
                names = []  # not a rated data table (e.g. the product overview, which rounds)
            continue
        for label, key in ROWS.items():
            if names and line.startswith(label) and (key != "mn" or "Mrated" in line):
                tokens = line.split()[-len(names):]
                if not all(re.fullmatch(r"\d+(\.\d+)?", t) for t in tokens):
                    raise ValueError(f"{label}: {tokens} for {names}")
                for n, t in zip(names, tokens):
                    motors.setdefault(n, {}).setdefault(key, t)
    return motors


def motor_row(part, series, d, jmb, brake_kg, pair):
    mass_brake = repr(float(Decimal(d["m"]) + Decimal(brake_kg)))
    return [part, "lenze", series, "400", d["m0"], d["i0"], d["mn"], d["in"],
            repr(float(d["nn"]) * math.pi / 30), si(d["pn"], "1000"), d["mmax"], d["imax"],
            repr(float(d["nmax"]) * math.pi / 30), d["kt"], repr(float(d["ke"]) / (1000 * math.pi / 30)),
            d["r20"], si(d["l"], "0.001"), "", si(d["j"], "0.0001"), si(jmb, "0.0001"), d["m"], mass_brake,
            "", pair, ""]


def mcs_pairs(text):
    """MCS i950 selection tables at 3x 400 V: smallest inverter whose Mmax reaches the motor's Mmax."""
    start = next(i for i, l in enumerate(text) if l.strip() == "i950 cabinet servo inverter")
    end = next(i for i, l in enumerate(text) if i > start and "mains  voltage 3x 230 V" in l)
    groups, labels, prev = [], [], None
    for n, l in enumerate(text[start:end]):
        lab = re.search(r"MCS +(\d\d[A-Z]\d\d)-", l)
        if lab:
            labels.append((n, "MCS" + lab.group(1)))
        m = re.search(r"(i950-C([\d.]+)/400-3)(.*)$", l)
        if not m:
            continue
        if prev is None or float(m.group(2)) <= prev:  # inverter sizes rise within one motor
            groups.append([])
        prev = float(m.group(2))
        groups[-1].append((n, m.group(1), float(m.group(3).split()[-1])))
    if len(groups) != len(labels):
        raise ValueError(f"{len(groups)} inverter groups but {len(labels)} motors")
    return {name: g for (pos, name), g in zip(labels, groups) if g[0][0] <= pos <= g[-1][0]}


def drives(text):
    """i950 rated data and brake chopper blocks of the 3-phase 400 V chapter (not the 480 V one)."""
    rows, voltage = {}, None
    for i, l in enumerate(text):
        m = re.search(r"phase mains\s+connection\s+(\d+)\s*V", l)
        if m:
            voltage = m.group(1)
        if voltage != "400" or not (l.strip().startswith("Product name") and "/400-3" in l):
            continue
        names = re.findall(r"i950-C([\d.]+)/400-3", l)
        block, rec, sect = text[i + 1:i + 60], {}, None
        if text[i - 1].strip() == "Brake chopper":
            thr = re.search(r"Switch-on threshold\s+V\s+(\d+)\s*$", text[i + 1]).group(1)
            for n in names:
                rows.setdefault(n, {})["thr"] = thr
            continue
        for b in block:
            s = b.strip()
            if s.startswith("Product name") or s.startswith("Max. motor cable length"):
                break
            if s.startswith("Rated output current"):
                sect = "rated"
            elif s.startswith("Power loss"):
                sect = None
            elif s.startswith("Overcurrent cycle"):
                sect = s
            v = s.split()
            if sect == "rated" and s.startswith("4 kHz"):
                rec["rated"] = v[-len(names):]
            elif sect == "Overcurrent cycle 15 s" and s.startswith("Max. output current"):
                rec.setdefault("max", v[-len(names):])  # not "Max. output current during the recovery time"
            elif sect == "Overcurrent cycle 15 s" and s.startswith("Overload time"):
                rec["t"] = v[-len(names):]
        for key in ("rated", "max", "t"):
            if len(rec.get(key, [])) != len(names):
                raise ValueError(f"i950 {names}: {key} = {rec.get(key)}")
        for k, n in enumerate(names):
            rows.setdefault(n, {}).update(rated=rec["rated"][k], max=rec["max"][k], t=rec["t"][k])
    return rows


def write(path, header, rows):
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        for r in [header] + rows:
            if any("," in c for c in r):
                raise ValueError(f"comma in {r}")
            f.write(",".join(r) + "\n")


def main(mcs_txt, m850_txt, i950_txt, out):
    out = Path(out)
    mcs_text = lines(mcs_txt)
    mcs = rated_blocks(mcs_text, r"MCS \d\d[A-Z]\d\d(?=-)", "Inverter mains connection 230 V")
    pairs = {n: next(inv for _, inv, mmax in g if mmax >= float(mcs[n]["mmax"])) for n, g in mcs_pairs(mcs_text).items()}
    rows = []
    for n, d in mcs.items():
        frame = n[3:6]
        kg = MCS_BRAKE_KG.get(n[3:6], MCS_BRAKE_KG.get(n[3:5]))
        rows.append(motor_row(n, "MCS", d, MCS_JMB[frame], kg, pairs[n]))
    write(out / "motors_mcs.csv", MOTOR_HEADER, rows)

    m850 = rated_blocks(lines(m850_txt), r"m850-S\d+/(?:[SML]\d+)?", "Notes on the selection tables")
    rows = []
    for n, d in m850.items():
        key = n[len("m850-"):]
        rows.append(motor_row(n, "m850", d, M850_JMB[key], M850_BRAKE_KG[key.split("/")[0]], M850_PAIR.get(key, "")))
    write(out / "motors_m850.csv", MOTOR_HEADER, rows)

    rows = []
    for size, d in sorted(drives(lines(i950_txt)).items(), key=lambda kv: float(kv[0])):
        rows.append([f"i950-C{size}/400-3", "lenze", "i950", "3 AC 340 V ... 440 V", "400", "1", d["rated"],
                     d["max"], d["t"], "", "", "", "", "", I950_RMIN[size], d["thr"], "", ""])
    write(out / "drives_i950.csv", DRIVE_HEADER, rows)


if __name__ == "__main__":
    main(*sys.argv[1:5])
