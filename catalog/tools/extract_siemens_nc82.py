"""Extracts the SINAMICS S120 Line Modules in booksize format, the Braking Module and the braking
resistors from the Siemens catalogue NC 82, February 2020, into catalog/siemens/line_modules_s120.csv
and catalog/siemens/braking_s120.csv.

The catalogue is not part of this repository. Get the PDF from Siemens, then:

    pdftotext -table -enc UTF-8 catalogue.pdf catalogue.txt      (xpdf 4.x)
    python extract_siemens_nc82.py catalogue.txt ../siemens

The Line Modules and the resistors sit in "Technical specifications" tables with one column per
article, the Braking Module in a table of its own. A row that does not have one value per column
stops the script.
"""
import re
import sys
from pathlib import Path

SOURCE = "Siemens, Catalog NC 82, SINUMERIK MC, SINAMICS S120 and SIMOTICS, February 2020"
LINE_HEADER = ["article", "type", "rated_power_kw", "s6_power_kw", "max_power_kw", "dc_link_current_540v_a",
               "dc_link_current_600v_a", "s6_dc_link_current_a", "max_dc_link_current_a", "dc_link_capacitance_uf",
               "power_loss_kw", "braking_power_max_kw", "braking_power_continuous_kw", "braking_threshold_v",
               "min_braking_resistance_ohm"]
BRAKING_HEADER = ["article", "kind", "for_article", "resistance_ohm", "rated_power_kw", "peak_power_kw",
                  "activation_threshold_v", "dc_link_capacitance_uf", "load_duration_s", "cycle_duration_s",
                  "load_duration_2_s", "cycle_duration_2_s"]
# "For S6 duty" and "Maximum" appear under the power, the DC link current and the input current:
# the heading above a row decides. Heading as printed -> our name for it.
SECTIONS = {"Infeed/regenerative": "power", "Power": "power", "Braking power": "braking", "DC link current": "dc_current",
            "Input current": "input_current", "DC link capacitance": "capacitance"}
# (section or None for any, start of the row label, unit, column)
LINE_ROWS = [("power", "At 380 V 3 AC", "kW", "rated_power_kw"), ("power", "For S6 duty", "kW", "s6_power_kw"),
             ("power", "Pmax", "kW", "max_power_kw"), ("braking", "PBmax", "kW", "braking_power_max_kw"),
             ("braking", "Continuous braking power", "kW", "braking_power_continuous_kw"),
             ("dc_current", "At 540 V/600 V DC", "A", "dc_link_current_540v_a/dc_link_current_600v_a"),
             ("dc_current", "At 600 V DC", "A", "dc_link_current_600v_a"),
             ("dc_current", "For S6 duty", "A", "s6_dc_link_current_a"), ("dc_current", "Maximum", "A", "max_dc_link_current_a"),
             ("capacitance", "Smart Line Module", "F", "dc_link_capacitance_uf"),
             ("capacitance", "Active Line Module", "F", "dc_link_capacitance_uf"),
             ("capacitance", "Basic Line Module", "F", "dc_link_capacitance_uf"), (None, "Power loss", "kW", "power_loss_kw"),
             (None, "Activation threshold", "V", "braking_threshold_v"), (None, "Resistance value", "Ω", "min_braking_resistance_ohm")]
VALUE = re.compile(r"^(–|\d+(\.\d+)?(/\d+(\.\d+)?)?)$")
ARTICLE = r"6S[A-Z]\d{4}-\w{5}-\w{4}"


def line_modules(lines):
    """{article: {column: value}} and what could not be read."""
    out, problems = {}, []
    for start, line in enumerate(lines):
        articles = re.findall(r"6SL3130-\w{5}-\w{4}", line)
        if "Article No." not in line or len(articles) < 2:
            continue
        kind = re.search(r"(Smart|Active|Basic) Line Module in\s+booksize\s+format with internal air cooling", " ".join(lines[start:start + 8]))
        if not kind:
            continue
        for article in articles:
            out.setdefault(article, {})["type"] = kind.group(1).lower()
        section = None
        for raw in lines[start + 1:]:
            if "Siemens NC 82" in raw or re.match(r"\s*\d\)\s", raw):
                break  # end of the page or the footnotes: the table goes on under a new header
            # Imperial values and the peak power with another Active Interface Module stand in brackets,
            # footnote marks follow as "2)". A lone "4" at the margin is the chapter tab of the page.
            text = re.sub(r"\([^)]*\)|\d?\)|[•≥]|^\s*4\s", " ", raw)
            tokens = text.split()
            label = " ".join(tokens).lstrip("- ")
            if label in SECTIONS:
                section = SECTIONS[label]
                continue
            row = next((r for r in LINE_ROWS if r[0] in (None, section) and label.startswith(r[1])), None)
            if not row:
                continue
            # The micro sign may come out of the PDF as a symbol-font character.
            is_unit = (lambda t: len(t) <= 2 and t.endswith("F")) if row[2] == "F" else (lambda t: t == row[2])
            unit = next((i for i, t in enumerate(tokens) if i >= len(row[1].split()) and is_unit(t)), None)
            values = [] if unit is None else [t for t in tokens[unit + 1:] if VALUE.match(t)][:len(articles)]
            if len(values) != len(articles):
                problems.append(f"{kind.group(1)}: {label}")
                continue
            for article, value in zip(articles, values):
                # "9.3/8.3" under "At 540 V/600 V DC": one value per voltage.
                for column, part in zip(row[3].split("/"), value.split("/") if "/" in row[3] else [value]):
                    if out[article].setdefault(column, part) != part:
                        problems.append(f"{article}: {column} twice")
    for article, data in out.items():
        missing = [c for c in ("rated_power_kw", "max_power_kw", "dc_link_current_600v_a", "max_dc_link_current_a",
                               "dc_link_capacitance_uf", "power_loss_kw") if not data.get(c)]
        if missing:
            problems.append(f"{article}: no {missing}")
    return out, problems


def braking_module(lines):
    """The Braking Module's table has one column: label, value and unit on a line."""
    start = next((i for i, l in enumerate(lines) if re.search(r"Article No\.\s+6SL3100-1AE\w\w-\w{4}\s*$", l)), None)
    if start is None:
        return None, ["no Braking Module table"]
    end = next(i for i in range(start, len(lines)) if "Digital inputs" in lines[i])
    block = "\n".join(lines[start:end])
    row = {"article": re.search(ARTICLE, lines[start]).group(), "kind": "braking_module"}
    for column, pattern in [("rated_power_kw", r"Rated power PDB\s+([\d.]+) kW"), ("peak_power_kw", r"Peak power Pmax\s+([\d.]+) kW"),
                            ("activation_threshold_v", r"Activation threshold\s+([\d.]+) V"),
                            ("dc_link_capacitance_uf", r"DC link capacitance\s+([\d.]+) \SF")]:
        found = re.search(pattern, block)
        if not found or "Braking Module" not in block:
            return None, [f"Braking Module: no {column}"]
        row[column] = found.group(1)
    return row, []


def braking_resistors(lines):
    start = next((i for i, l in enumerate(lines) if "Article No." in l and len(re.findall(ARTICLE, l)) >= 2
                  and "Braking resistor for" in " ".join(lines[i:i + 4])), None)
    if start is None:
        return [], ["no braking resistor table"]
    heads = list(re.finditer(ARTICLE, lines[start]))
    rows = [{"article": h.group(), "kind": "braking_resistor"} for h in heads]
    problems = []
    for raw in lines[start + 1:]:
        if "Power connections" in raw:
            break
        for label, unit, columns in [("Resistance", "Ω", ["resistance_ohm"]), ("Rated power PDB", "kW", ["rated_power_kw"]),
                                     ("Peak power Pmax", "kW", ["peak_power_kw"]),
                                     ("Load duration ta", "s", ["load_duration_s", "load_duration_2_s"]),
                                     ("Cycle duration t", "s", ["cycle_duration_s", "cycle_duration_2_s"])]:
            found = re.match(rf"\s*{label}\s+{unit}\s", raw)
            if not found:
                continue
            # The resistors of the Braking Module have two duty cycles, so a column can hold two values:
            # a value belongs to the article whose number starts at or left of it.
            per_column = [[] for _ in heads]
            for value in re.finditer(r"\S+", raw[found.end():]):
                position = found.end() + value.start()
                owner = max((i for i, h in enumerate(heads) if h.start() <= position + 2), default=None)
                if owner is None or not VALUE.match(value.group()):
                    problems.append(f"resistors: {label}: {value.group()}")
                else:
                    per_column[owner].append(value.group())
            for row, values in zip(rows, per_column):
                if not 1 <= len(values) <= len(columns):
                    problems.append(f"{row['article']}: {label}: {values}")
                row.update(zip(columns, values))
    # "Selection and ordering data" above the table says which module a resistor is for; the module
    # is printed once for the resistors that follow it. Its powers must be the table's.
    host = None
    for raw in lines[start - 60:start]:
        tokens = raw.split()
        row = next((r for r in rows if tokens and tokens[-1] == r["article"]), None)
        if not row or len(tokens) < 3:
            continue
        if len(tokens) >= 4 and re.fullmatch(ARTICLE, tokens[-4]):
            host = tokens[-4]
        if tokens[-3:-1] != [row.get("rated_power_kw"), row.get("peak_power_kw")]:
            problems.append(f"{row['article']}: ordering data says {tokens[-3:-1]} kW")
        row["for_article"] = host
    for row in rows:
        missing = [c for c in ("for_article", "resistance_ohm", "rated_power_kw", "peak_power_kw", "load_duration_s", "cycle_duration_s")
                   if not row.get(c)]
        if missing or ("load_duration_2_s" in row) != ("cycle_duration_2_s" in row):
            problems.append(f"{row['article']}: no {missing} or duty cycles do not pair")
    return rows, problems


def write(path, notes, header, rows):
    table = "".join(",".join(r.get(c, "").replace("–", "") for c in header) + "\n" for r in rows)
    path.write_text(notes + ",".join(header) + "\n" + table, encoding="utf-8", newline="\n")


def main():
    lines = Path(sys.argv[1]).read_text(encoding="utf-8").splitlines()
    modules, problems = line_modules(lines)
    module, more = braking_module(lines)
    resistors, even_more = braking_resistors(lines)
    problems += more + even_more
    if problems or not modules:
        sys.exit("NC 82 tables not read:\n" + "\n".join(problems))
    out = Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
    extracted = "Extracted by catalog/tools/extract_siemens_nc82.py; the numbers are the catalogue's, the layout is ours."
    write(out / "line_modules_s120.csv", f"""# Source: {SOURCE}, chapter 4, Line Modules, "Technical specifications".
# {extracted}
# One row per SINAMICS S120 Line Module in booksize format with internal air cooling, for a line voltage of 380 ... 480 V 3 AC.
# type: smart (unregulated infeed with regenerative feedback), active (regulated DC link, regenerative feedback) or
#   basic (diode infeed without feedback, which needs a braking resistor to brake).
# rated_power: at 380 V 3 AC. s6_power: for S6 duty with 40 % on-time. max_power: the catalogue's Pmax.
# dc_link_current_540v/_600v: the rated DC link current at a DC link voltage of 540 V and of 600 V; the catalogue
#   gives the value at 540 V for the Smart Line Modules only. s6_ and max_dc_link_current: as printed, without a voltage.
# dc_link_capacitance: of the module itself. power_loss: at rated power, with the losses of the 24 V electronics supply.
# braking_*: the braking chopper inside the 20 kW and 40 kW Basic Line Modules for an external resistor: peak and
#   continuous braking power, the DC link voltage at which it switches on, the smallest resistance allowed.
#   Empty for all other modules, which have none.
# An empty field means the catalogue does not state the value.
# max_power of the 6SL3130-7TE25-5AA3 is 91 kW (152 A); the catalogue adds 110 kW (176 A) in brackets for use with the
#   Active Interface Module 6SL3100-0BE25-5AB0, which is not in this file.
# Inconsistent in the catalogue and kept as printed: the 6SL3130-6TE25-5AA3 has 138 A for S6 duty and 178 A maximum
#   at 71 kW and 91 kW, which would be a DC link of about 510 V; every other module, the Active Line Module of the same
#   power included (121 A, 152 A), has power / 600 V in these two rows.
# Not extracted: input currents, 24 V current, busbar ratings, the largest capacitance of a drive line-up, cooling air,
#   connections, cable lengths, dimensions, derating, and the line-side components (reactors, filters, Active Interface Modules).
""", LINE_HEADER, [dict(data, article=article) for article, data in modules.items()])
    write(out / "braking_s120.csv", f"""# Source: {SOURCE}, chapter 4, DC link components, Braking Module and Braking resistors.
# {extracted}
# One row for the SINAMICS S120 Braking Module in booksize format and one per braking resistor, for a DC link of 510 ... 720 V.
# kind: braking_module (the chopper, without a resistor) or braking_resistor.
# for_article: the module the catalogue's ordering data lists the resistor for: the Braking Module, or a Basic Line
#   Module of catalog/siemens/line_modules_s120.csv, which has a chopper of its own.
# rated_power: the continuous braking power PDB. peak_power: Pmax, allowed for load_duration within every cycle_duration.
# load_duration_2/cycle_duration_2: the resistors of the Braking Module have a second duty cycle with a longer load.
# activation_threshold: the DC link voltage at which the Braking Module switches the resistor on; for the resistors of
#   the Basic Line Modules see braking_threshold_v there.
# dc_link_capacitance: of the Braking Module.
# An empty field means the catalogue does not state the value.
# The peak power of the 6SN1113-1AA00-0DA0 (25 kW at 17 ohm) is not U^2/R at the threshold of 770 V, which would be
#   35 kW; for the other three resistors it is. Kept as printed.
# Not extracted: cable lengths, digital inputs and outputs, connections, thermostatic switch, degree of protection,
#   dimensions, weight.
""", BRAKING_HEADER, [module] + resistors)
    print(f"{len(modules)} line modules, 1 braking module, {len(resistors)} braking resistors")


if __name__ == "__main__":
    main()
