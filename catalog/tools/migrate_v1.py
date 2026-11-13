"""One-off move of the catalogue to schema 1, kept as the record of how it was done (November 2026).

It read the files as they were before (vendor column names and units, a '#' comment header, the Festo EXCT as
TOML) and wrote each as a schema 1 table: the CSV in SI units with vendor-neutral names, and beside it the
metadata with the sources, the provenance and the old header as notes. It cannot be run again on the tree as it is
now; check out the commit before it to see its input:

    python tools/migrate_v1.py

`cataloglib.legacy.convert_rows` checks every converted value against the old one, and the golden test of the
simulator (`crates/sim-machines/tests/catalog_golden.rs`) checks that nothing it computes from the catalogue changed.
"""

import tomllib
from pathlib import Path

from cataloglib import legacy
from cataloglib.table import ROOT, SCHEMA_VERSION, write_meta, write_rows

SIEMENS = "https://cache.industry.siemens.com/dl/files"


def source(id, document, date, locator, method, url=None, **more):
    out = {"id": id, "document": document, "date": date, "locator": locator, "method": method}
    if url:
        out["url"] = url
    out.update(more)
    return out


D32 = dict(document="Siemens, catalogue D 32, SINAMICS S210 servo drive system", date="2024-01",
           url=f"{SIEMENS}/381/109754381/att_1261452/v1/motion-control-drives-D32-complete-English-2024-01.pdf",
           method="script:tools/extract_siemens_d32.py")
FK2 = dict(document="Siemens, 1FK2 Synchronous Motors for SINAMICS S120, Configuration Manual, A5E46927724B AD", date="2022-02",
           url=f"{SIEMENS}/757/109769757/att_1102511/v1/1FK2_S120_config_man_0222_en-US.pdf",
           method="script:tools/extract_siemens_1fk2_s120.py")
NC82 = dict(document="Siemens, Catalog NC 82, SINUMERIK MC, SINAMICS S120 and SIMOTICS", date="2020-02",
            url=f"{SIEMENS}/977/109746977/att_1014595/v1/motion-control-NC82-complete-English-2020-02.pdf",
            method="script:tools/extract_siemens_nc82.py")
GEARBOXES = dict(document="Siemens, Planetary gearboxes for 1FK2 and 1FT2 servomotors, Equipment Manual, A5E54515894B AA", date="2025-04",
                 url="https://docs.rs-online.com/f6f5/A700000014746827.pdf", locator="chapter 6",
                 method="script:tools/extract_siemens_gearboxes.py")
MEGADYNE = dict(document="Megadyne (Ammega), MEGALINEAR technical handbook", date="2026-02",
                url="https://megadynegroup.com/files/resources/attachments/md_manu_en_megalinear_web.pdf", locator="the page of each profile",
                method="script:tools/extract_megadyne.py")
S210_OI = dict(document="Siemens, SINAMICS S210 servo drive system with S-1FK2 and S-1FT2, Operating Instructions, A5E41702836B AL", date="2022-09",
               method="manual: typed in from the manual; the table checked against the rendered page of the 06/2020 edition (Table 3-2), which has the same values")
FESTO = dict(document='Festo, catalogue "Linear gantries EXCT"', date="2023-01",
             url="https://ftp.festo.com/Public/PNEUMATIC/SOFTWARE_SERVICE/Documentation/2023/EN/EXCT_EN.PDF",
             method="manual: written from the catalogue's tables; there are three sizes")

# file -> (key columns, sources, provenance); the first source is the default of every row
META = {
    "siemens/motors_s210.csv": (["part_number"], [source("d32", locator='"Specific technical specifications" of each shaft height', **D32)], []),
    "siemens/converters_s210.csv": (["part_number"], [source("d32", locator='"Technical specifications" of the servo converters', **D32)], []),
    "siemens/motors_1fk2_s120.csv": (["part_number"], [source("1fk2-s120", locator='chapter "Data sheets and characteristics"', **FK2)], []),
    "siemens/dynamic_braking_1fk2.csv": (["part_number"], [source("1fk2-s120", locator='section 5.4.3, Table 5-2 "Dynamic braking for 1FK2"', **FK2)], []),
    "siemens/motors_1ft2_s120.csv": (["part_number"], [source(
        "1ft2-s120", "Siemens, 1FT2 synchronous motors for SINAMICS S120, Configuration Manual, A5E50645584B AC", "2022-02",
        'chapter "Data sheets and characteristics"', "script:tools/extract_siemens_1ft2_s120.py",
        f"{SIEMENS}/387/109794387/att_1102452/v1/1FT2_S120_config_man_0222_en-US.pdf")], []),
    "siemens/motors_s120.csv": (["part_number"], [source(
        "d21-4", "Siemens, catalogue D 21.4, SINAMICS S120 and SIMOTICS", "2017", "selection and ordering data of the SIMOTICS S synchronous motors",
        "script:tools/extract_siemens_d21_4.py",
        "https://assets.new.siemens.com/siemens/assets/api/uuid:ef8b99bf-acb1-4efd-bada-76f046c0ccc3/catalogo-simotics-1fk7-1ft7-1ph8-1pn-1fw.pdf")], []),
    "siemens/motors_s120_electrical.csv": (["part_number"], [source(
        "data-sheets", "Siemens data sheet service, one data sheet per motor", "2026-11", "the data sheet of the full part number in data_sheet_part_number",
        "script:tools/fetch_siemens_datasheets.py", "https://apim.industry.siemens.cloud/ted/datasheet?format=pdf&mlfbs=<data_sheet_part_number>&language=en&caller=SIOS")], []),
    "siemens/motor_modules_s120.csv": (["part_number"], [source(
        "d21-4n", "Siemens, Catalog News D 21.4 N, SINAMICS S120 and SIMOTICS", "2017-10", 'chapter 7, "Technical specifications"',
        "script:tools/extract_siemens_d21_4n.py", f"{SIEMENS}/875/109751875/att_934896/v1/motion-control-drives-D21-4-N-complete-English-2017-10.pdf")], []),
    "siemens/line_modules_s120.csv": (["part_number"], [source("nc82", locator='chapter 4, Line Modules, "Technical specifications"', **NC82)], []),
    "siemens/braking_s120.csv": (["part_number"], [source("nc82", locator="chapter 4, DC link components, Braking Module and Braking resistors", **NC82)], []),
    "siemens/braking_s210.csv": (["part_number"], [
        source("s210-oi", locator='Table 4-4 "Braking power and braking energy with the internal braking resistor"', **S210_OI),
        source("s210-oi-f07432", locator="the text of fault F07432 (DC link overvoltage: 410 V with a 1 AC line, 820 V with a 3 AC line)", **S210_OI),
    ], [{"rows": ["*"], "fields": ["dc_link_overvoltage_v"], "source": "s210-oi-f07432"}]),
    "siemens/dc_link_measured.csv": (["part_number"], [source(
        "measured-2026-11-13", "Measurement on one 6SL5310-1BE10-8DF0 (SINAMICS S210, new generation, firmware 6.4) with a 1FK2204-5AF", "2026-11-13",
        "a Startdrive trace of a quick stop from 3000 r/min; the notes give the method and the numbers",
        "measured: worked out from the trace as the notes describe")], []),
    "siemens/gearbox_nrb.csv": (["size", "motor_frame", "ratio"], [source("gearboxes", **GEARBOXES)], []),
    "siemens/gearbox_nrk.csv": (["size", "motor_frame", "ratio"], [source("gearboxes", **GEARBOXES)], []),
    "siemens/gearbox_nlc.csv": (["size", "motor_frame", "ratio"], [source("gearboxes", **GEARBOXES)], []),
    "siemens/gearbox_mass.csv": (["size", "motor_frames", "ratio"], [source(
        "gearbox-config-manual", "Siemens, Planetary gearboxes for S-1FK2 and S-1FT2 servomotors, Configuration Manual, A5E51318133B AC", "2023-01",
        'Tables 7-12 (NRB), 7-33 (NRK) and 7-54 (NLC), "Mass mG of ... coaxial planetary gearboxes"', "script:tools/extract_siemens_gearbox_mass.py",
        f"{SIEMENS}/596/109802596/att_1132680/v2/Plan_gear_for_servo_motors_config_man_0123_en-US.pdf")], []),
    "megadyne/belts.csv": (["profile", "width_m"], [source("megalinear", **MEGADYNE)], []),
    "megadyne/belt_tooth_load.csv": (["profile", "speed_rad_s"], [source("megalinear", **MEGADYNE)], []),
}


def migrate_csv(old_name):
    new_name, cls, table, columns = legacy.TABLES[old_name]
    notes, rows = legacy.read_legacy(ROOT / old_name)
    header, out = legacy.convert_rows(rows, columns, old_name)
    meta = {"schema_version": SCHEMA_VERSION, "class": cls}
    if table:
        meta["table"] = table
    if old_name == "bahr/belt_axes.csv":
        # The sheet, edition and file of each row become its source.
        sources = {}
        for row, new in zip(rows, out):
            id = legacy.source_of_row(row)
            month, year = row["edition"].split("/")
            sources.setdefault(id, source(
                id, f'Bahr Modultechnik GmbH (IMI Norgren), data sheet "Linearsystem {row["series"]}", German edition', f"20{year}-{month}",
                f"sheet {row['sheet']} (as its footer prints it)", "script:tools/extract_bahr.py", f"https://cdn.norgren.com/pdf/{row['source_file']}"))
            new["source"] = id
        header.append("source")
        meta.update(key=["series", "size"], sources=list(sources.values()))
    else:
        key, sources, provenance = META[old_name]
        meta.update(key=key, default_source=sources[0]["id"], sources=sources)
        if provenance:
            meta["provenance"] = provenance
    meta["notes"] = "\n".join(notes) + "\n"
    write_rows(ROOT / new_name, header, out)
    write_meta(ROOT / new_name, {k: meta[k] for k in ["schema_version", "class", "table", "key", "default_source", "notes", "sources", "provenance"] if k in meta})
    if new_name != old_name:
        (ROOT / old_name).unlink()
    return new_name


def migrate_festo():
    path = ROOT / "festo/exct.toml"
    text = path.read_text(encoding="utf-8")
    data = tomllib.loads(text)
    axes, formulas = [], []
    for name, size in data.items():
        if not isinstance(size, dict):
            continue
        series, number = name.split("-")
        if sorted(f["stroke_z_mm"] for f in size["torque_formula"]) != sorted(size["strokes_z_mm"]):
            raise SystemExit(f"{name}: the Z strokes are not those of the torque formula; they would be lost")
        row = {"series": series, "size": number, "kind": "xz_gantry"}
        row.update({k: str(v) for k, v in size.items() if not isinstance(v, (dict, list))})
        row.update({f"belt.{k}": str(v) for k, v in size["belt"].items()})
        axes.append(row)
        formulas += [{"series": series, "size": number, **{k: str(v) for k, v in f.items()}} for f in size["torque_formula"]]
    notes = "\n".join(l[2:] if l.startswith("# ") else l[1:] for l in text.splitlines() if l.lstrip().startswith("#")) + "\n"
    for file, rows, table, key, locator in [
        ("festo/belt_axes.csv", axes, None, ["series", "size"], "technical data of each size"),
        ("festo/belt_axis_torque_formula.csv", formulas, "torque_formula", ["series", "size", "stroke_z_m"], '"Load values": the required motor torque at 45 degrees travel'),
    ]:
        header, out = legacy.convert_rows(rows, legacy.FESTO[Path(file).stem], file)
        meta = {"schema_version": SCHEMA_VERSION, "class": "belt_axis"}
        if table:
            meta["table"] = table
        meta.update(key=key, default_source="exct", notes=notes, sources=[source("exct", locator=locator, **FESTO)])
        write_rows(ROOT / file, header, out)
        write_meta(ROOT / file, meta)
    path.unlink()
    return ["festo/belt_axes.csv", "festo/belt_axis_torque_formula.csv"]


def main():
    tables = [migrate_csv(name) for name in legacy.TABLES] + migrate_festo()
    lines = [f"schema_version = {SCHEMA_VERSION}", "tables = ["] + [f'    "{t}",' for t in sorted(tables)] + ["]"]
    (ROOT / "catalog.toml").write_text("\n".join(lines) + "\n", encoding="utf-8", newline="\n")
    print(f"{len(tables)} tables written")


if __name__ == "__main__":
    main()
