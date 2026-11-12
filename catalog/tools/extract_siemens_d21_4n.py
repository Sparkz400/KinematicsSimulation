"""Extracts the SINAMICS S120 Motor Modules in booksize format from the Siemens Catalog News
D 21.4 N, October 2017, into catalog/siemens/motor_modules_s120.csv.

The catalogue is not part of this repository. Get the PDF from Siemens, then:

    pdftotext -table -enc UTF-8 catalogue.pdf catalogue.txt      (xpdf 4.x)
    python extract_siemens_d21_4n.py catalogue.txt ../siemens

The data sits in four "Technical specifications" tables, one column per module size. A row that
does not have one value per column stops the script.
"""
import re
import sys
from pathlib import Path

SOURCE = "Siemens, Catalog News D 21.4 N, SINAMICS S120 and SIMOTICS, October 2017"
HEADER = ["article", "axes", "type", "rated_current_a", "base_load_current_a", "max_current_a", "rated_power_kw",
          "pulse_frequency_khz", "dc_link_capacitance_uf", "power_loss_typical_kw", "power_loss_max_kw"]
# Row label -> column. Values that differ between the C and the D type follow in rows of their own.
ROWS = {"Rated current Irated": "rated_current_a", "Base-load current IH": "base_load_current_a", "Imax": "max_current_a",
        "Based on Irated": "rated_power_kw", "Rated pulse frequency": "pulse_frequency_khz",
        "DC link capacitance": "dc_link_capacitance_uf", "Power loss": "power_loss"}
VALUE = re.compile(r"^(–|\d+(\.\d+)?(/\d+(\.\d+)?)?)$")


def modules(lines):
    out, problems = [], []
    for start, line in enumerate(lines):
        head = re.search(r"DC link voltage 510 \.\.\. 720\s+V\s+DC\s+(Single|Double)\s+Motor\s+Module", line)
        if not head:
            continue
        axes = 1 if head.group(1) == "Single" else 2
        articles, data, section = {}, {}, None  # {type: [article per column]}, {(column name, type): values}
        for raw in lines[start + 1:]:
            # A Double Motor Module prints "2 × 9": the value per axis is what counts. Imperial
            # values in brackets and footnote marks like "8)" go.
            text = re.sub(r"\([^)]*\)|\b\d\)", " ", re.sub(r"2\s*×\s*", "", raw))
            tokens = text.split()
            label = " ".join(tokens)
            if label.startswith("Cooling air requirement"):
                break
            if "6SL3120-..." in tokens:
                kind = re.search(r"\b([CD]) type", label)
                articles[kind.group(1) if kind else ""] = tokens[tokens.index("6SL3120-...") + 1:]
                continue
            if not articles:
                continue
            columns = len(next(iter(articles.values())))
            name = next((c for l, c in ROWS.items() if l in label), None)
            kind = re.match(r"- ([CD]) type", label)
            if name:
                section = name
            elif kind and section:
                name = section
            else:
                section = None if label.startswith("•") else section
                continue
            # The micro sign of the capacitance comes out of the PDF as a symbol-font character.
            unit = next((i for i, t in enumerate(tokens) if t in ("A", "kW", "kHz") or len(t) <= 2 and t.endswith("F")), None)
            if unit is None:
                continue  # the label alone: its values follow per type
            values = [t for t in tokens[unit + 1:] if VALUE.match(t)][:columns]
            if len(values) != columns:
                problems.append(label)
                continue
            data[(name, kind.group(1) if kind else "")] = values
        for kind, column_articles in articles.items():
            for column, suffix in enumerate(column_articles):
                if suffix == "–":
                    continue
                value = lambda name: (data.get((name, kind)) or data.get((name, "")) or [""] * columns)[column]
                typical, _, most = value("power_loss").partition("/")
                row = ["6SL3120-" + suffix, str(axes), kind] + [value(c) for c in HEADER[3:9]] + [typical, most]
                # The table of the Double Motor Modules has no pulse frequency row.
                if "" in row[3:7] + row[8:] or "–" in row:
                    problems.append(f"{row[0]}: a value is missing: {row}")
                out.append(row)
    return out, problems


def main():
    lines = Path(sys.argv[1]).read_text(encoding="utf-8").splitlines()
    rows, problems = modules(lines)
    if problems or not rows:
        sys.exit("Motor Module tables not read:\n" + "\n".join(problems))
    notes = f"""# Source: {SOURCE}, chapter 7, "Technical specifications".
# Extracted by catalog/tools/extract_siemens_d21_4n.py; the numbers are the catalogue's, the layout is ours.
# One row per SINAMICS S120 Motor Module in booksize format with internal air cooling, for a DC link of 510 ... 720 V.
# axes: 1 for a Single, 2 for a Double Motor Module; currents, rated power and losses of a Double are per axis
#   as printed ("2 x 9 A"), its DC link capacitance and power loss are for the module.
# type: C (continuous load) or D (discontinuous load, higher maximum current); empty where the catalogue makes no difference.
# Currents are RMS. base_load_current: the current that leaves room for the overload cycles.
# rated_power: the catalogue's type rating based on the rated current, for a standard induction motor at 400 V.
# pulse_frequency: the rated one; the catalogue allows others with derating. Empty for the Double Motor Modules,
#   whose table does not state it.
# Not extracted: currents for S6 duty, DC link current, busbar ratings, derating curves, overload cycles,
#   dimensions, and everything but booksize Motor Modules (Line Modules, chassis and blocksize formats).
"""
    out = Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
    (out / "motor_modules_s120.csv").write_text(notes + ",".join(HEADER) + "\n" + "".join(",".join(r) + "\n" for r in rows),
                                               encoding="utf-8", newline="\n")
    print(f"{len(rows)} motor modules")


if __name__ == "__main__":
    main()
