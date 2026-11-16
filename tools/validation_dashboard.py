#!/usr/bin/env python3
"""The validation dashboard: every check of docs/validation.md as one static HTML page.

The rows are the tables of docs/validation.md (what is compared, against what, the tolerance, the tests). Each test
named there is found in the sources (a link to its line), its outcome is read from nextest's JUnit file, and the values
the tests wrote with `sim_core::testkit::record_check` (a JSON line each, to the file `KS_VALIDATION_OUT` names) are
shown under the row: the reference, the simulated value, the difference allowed, the margin left. Standard library only.

    python tools/validation_dashboard.py --nextest-filter > filter.txt
    KS_VALIDATION_OUT=$PWD/target/validation.jsonl cargo nextest run --workspace --exclude sim-py --profile full -E "$(cat filter.txt)"
    python tools/validation_dashboard.py --junit target/nextest/full/junit.xml --records target/validation.jsonl --out site/validation
"""

import argparse
import html
import json
import os
import re
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NAME = re.compile(r"^[a-z_][a-z0-9_]*(::[a-z0-9_]+)*$")


def split_row(line):
    """The cells of a Markdown table row; a | inside backticks stays in its cell."""
    cells, cell, code = [], "", False
    body = line.strip()
    body = body[1:] if body.startswith("|") else body
    body = body[:-1] if body.endswith("|") and not body.endswith("\\|") else body
    for c in body:
        if c == "`":
            code = not code
        if c == "|" and not code:
            cells.append(cell.strip())
            cell = ""
        else:
            cell += c
    return cells + [cell.strip()]


def tables(markdown):
    """Every table row of the document as (section, header, cells). A table whose header has no separator line under
    it continues the table before it (validation.md splits long tables with a paragraph)."""
    lines = markdown.splitlines()
    section, header, out, i = "", None, [], 0
    while i < len(lines):
        line = lines[i]
        if line.startswith("#"):
            section = line.lstrip("#").strip()
            header = None
        if line.startswith("|"):
            block = []
            while i < len(lines) and lines[i].startswith("|"):
                block.append(lines[i])
                i += 1
            if len(block) > 1 and re.fullmatch(r"[\s|:-]+", block[1]):
                header, block = split_row(block[0]), block[2:]
            for row in block:
                out.append((section, header or [], split_row(row)))
            continue
        i += 1
    return out


def column(header, *starts):
    for k, h in enumerate(header):
        if any(h.lower().startswith(s) for s in starts):
            return k
    return None


def checks(markdown):
    """The checks: dicts with section, what, against, tolerance, test cell and the test names in it."""
    out, last_tests = [], []
    for section, header, cells in tables(markdown):
        if header[:2] == ["Short", "File"]:
            continue  # the table of file abbreviations
        cell = lambda k: cells[k] if k is not None and k < len(cells) else ""
        t, a, tol = column(header, "test", "check in"), column(header, "against"), column(header, "tolerance")
        if t is None:
            # A table of figures (the shipped configs, the measured recordings): its columns as they are.
            against = "; ".join(f"{h}: {c}" for h, c in zip(header[1:], cells[1:]))
            out.append(dict(section=section, what=cell(0), against=against, tolerance="", test_cell="", tests=[]))
            continue
        names = [n for n in re.findall(r"`([^`]+)`", cell(t)) if NAME.match(n)]
        if not names and "same test" in cell(t):
            names = last_tests
        last_tests = names
        out.append(dict(section=section, what=cell(0), against=cell(a), tolerance=cell(tol), test_cell=cell(t), tests=names))
    return out


def find_sources(names):
    """Where each name is defined: `fn name` in the Rust sources, `def name` in the Python ones."""
    wanted = {n.split("::")[-1] for n in names}
    found = defaultdict(list)
    # A function, or a test a macro makes one per machine of (`per_machine!(name, ...)`).
    pattern = re.compile(r"^\s*(?:(?:pub(?:\([a-z]+\))?|async)\s+)*(?:fn|def)\s+([a-z_][a-z0-9_]*)|^[a-z_]+!\(([a-z_][a-z0-9_]*),")
    for top in ("crates", "catalog"):
        for dirpath, dirs, files in os.walk(os.path.join(ROOT, top)):
            dirs[:] = [d for d in dirs if d not in ("target", "node_modules", ".venv", "__pycache__")]
            for f in files:
                if not f.endswith((".rs", ".py")):
                    continue
                path = os.path.join(dirpath, f)
                with open(path, encoding="utf-8", errors="replace") as fh:
                    for number, line in enumerate(fh, 1):
                        m = pattern.match(line)
                        name = m and (m.group(1) or m.group(2))
                        if name in wanted:
                            found[name].append((os.path.relpath(path, ROOT).replace("\\", "/"), number))
    return found


def outcomes(junit):
    """Pass or fail per test name, from nextest's JUnit file. A test is filed under every segment of its path, so
    `the_check` finds `tests::the_check` and the per-machine split `the_check::machine_06`.
    shortcut: a name defined in two crates gets both crates' outcomes together; the source links tell them apart."""
    seen = defaultdict(list)
    if not junit:
        return seen
    for case in ET.parse(junit).getroot().iter("testcase"):
        failed = case.find("failure") is not None or case.find("error") is not None
        if case.find("skipped") is not None:
            continue
        for segment in case.get("name", "").split("::"):
            seen[segment].append(not failed)
    return seen


def records(path):
    """The recorded values per test name, the worst of each (test, what) with the count of its samples."""
    worst = {}
    if not path or not os.path.exists(path):
        return defaultdict(list)
    with open(path, encoding="utf-8") as fh:
        for line in fh:
            if not line.strip():
                continue
            r = json.loads(line)
            ref, sim, allowed = r.get("reference"), r.get("simulated"), r.get("allowed")
            # The share of the tolerance used; a value that is not a number fails.
            if None in (ref, sim, allowed):
                r["used"] = float("inf")
            elif allowed <= 0:
                r["used"] = 0.0 if sim == ref else float("inf")
            else:
                r["used"] = abs(sim - ref) / allowed
            key = (r["test"], r["what"])
            r["samples"] = worst.get(key, {}).get("samples", 0) + 1
            if key not in worst or r["used"] > worst[key]["used"]:
                worst[key] = r
            else:
                worst[key]["samples"] = r["samples"]
    by_test = defaultdict(list)
    for r in worst.values():
        test = r["test"].split("::")
        # libtest and nextest name the thread after the test; the split per machine adds a segment after it.
        by_test[test[-1]].append(r)
        if len(test) > 1:
            by_test[test[-2]].append(r)
    return by_test


def inline(text, repo):
    """The little Markdown the tables use: links (their text may be code), code, bold."""

    def plain(t):
        out = []
        for p in re.split(r"(`[^`]*`)", t):
            if p.startswith("`") and p.endswith("`") and len(p) > 1:
                out.append(f"<code>{html.escape(p[1:-1])}</code>")
            else:
                out.append(re.sub(r"\*\*(.+?)\*\*", r"<b>\1</b>", html.escape(p, quote=False)))
        return "".join(out)

    out, at = [], 0
    for m in re.finditer(r"\[([^\]]+)\]\(([^)\s]+)\)", text):
        target = m.group(2)
        if target.startswith("../"):
            target = f"{repo}/{target[3:]}"
        elif target.startswith("#"):
            target = f"../docs/validation.html{target}"
        elif not re.match(r"https?://", target):
            target = "../docs/" + re.sub(r"\.md\b", ".html", target)
        out.append(plain(text[at:m.start()]) + f'<a href="{html.escape(target)}">{plain(m.group(1))}</a>')
        at = m.end()
    return "".join(out) + plain(text[at:])


def fmt(v):
    return "—" if v is None else f"{v:.6g}"


def status_of(results):
    if not results:
        return "notrun"
    return "pass" if all(results) else "fail"


CSS = """
:root { --bg:#fbfbfa; --fg:#1d1d1b; --muted:#6a6a64; --line:#e3e2dc; --card:#ffffff; --pass:#1f7a3d; --passbg:#e3f3e8;
  --fail:#b3261e; --failbg:#fbe4e2; --none:#6a6a64; --nonebg:#efeeea; --link:#1f5fa8; --code:#f1f0ec; }
@media (prefers-color-scheme: dark) { :root:not([data-theme="light"]) { --bg:#161615; --fg:#e8e7e2; --muted:#a09f98;
  --line:#33322f; --card:#1e1e1c; --pass:#7fd497; --passbg:#173a22; --fail:#ff9a90; --failbg:#45201d; --none:#b0afa8;
  --nonebg:#2a2a27; --link:#8ab8f0; --code:#2a2a27; } }
:root[data-theme="dark"] { --bg:#161615; --fg:#e8e7e2; --muted:#a09f98; --line:#33322f; --card:#1e1e1c; --pass:#7fd497;
  --passbg:#173a22; --fail:#ff9a90; --failbg:#45201d; --none:#b0afa8; --nonebg:#2a2a27; --link:#8ab8f0; --code:#2a2a27; }
* { box-sizing: border-box; }
body { margin:0; background:var(--bg); color:var(--fg); font:14px/1.45 system-ui, -apple-system, "Segoe UI", sans-serif; }
main { max-width: 1400px; margin: 0 auto; padding: 24px 16px 64px; }
h1 { font-size: 24px; margin: 0 0 4px; } h2 { font-size: 17px; margin: 32px 0 8px; }
p.lede { color: var(--muted); margin: 0 0 16px; max-width: 70ch; }
a { color: var(--link); } code { background: var(--code); padding: 0 3px; border-radius: 3px; font-size: 12.5px; }
.tiles { display:flex; flex-wrap:wrap; gap:8px; margin: 16px 0; }
.tile { background:var(--card); border:1px solid var(--line); border-radius:8px; padding:8px 14px; min-width:110px; }
.tile b { display:block; font-size:22px; font-variant-numeric: tabular-nums; } .tile span { color:var(--muted); font-size:12px; }
.bar { display:flex; flex-wrap:wrap; gap:8px; align-items:center; margin: 8px 0 0; position:sticky; top:0; background:var(--bg); padding:8px 0; z-index:1; }
.bar input, .bar select { font:inherit; padding:5px 8px; border:1px solid var(--line); border-radius:6px; background:var(--card); color:var(--fg); }
.bar input { flex: 1 1 240px; }
.wrap { overflow-x: auto; }
table { border-collapse: collapse; width: 100%; background: var(--card); border: 1px solid var(--line); }
th, td { text-align: left; vertical-align: top; padding: 6px 8px; border-top: 1px solid var(--line); }
th { font-size: 12px; color: var(--muted); font-weight: 600; }
td.what { min-width: 220px; } td.against { min-width: 220px; } td.tests { min-width: 210px; } td.recs { min-width: 420px; }
table.rec td:not(:first-child) { white-space: nowrap; } table.rec td:first-child { min-width: 180px; }
.badge { display:inline-block; font-size:11.5px; font-weight:600; border-radius:10px; padding:0 7px; white-space:nowrap; }
.pass { color:var(--pass); background:var(--passbg); } .fail { color:var(--fail); background:var(--failbg); }
.notrun, .doc { color:var(--none); background:var(--nonebg); }
ul.tests { margin:0; padding:0; list-style:none; } ul.tests li { margin: 0 0 3px; word-break: break-word; }
table.rec { margin-top: 6px; border: none; font-size: 12.5px; font-variant-numeric: tabular-nums; }
table.rec th, table.rec td { padding: 2px 6px; }
.muted { color: var(--muted); }
@media (max-width: 700px) { td.what, td.against { min-width: 160px; } }
"""

SCRIPT = """
const q = document.getElementById('q'), s = document.getElementById('s');
function filter() {
  const text = q.value.toLowerCase(), want = s.value;
  document.querySelectorAll('tr.check').forEach(tr => {
    const ok = (!text || tr.textContent.toLowerCase().includes(text)) && (!want || tr.dataset.status === want);
    tr.hidden = !ok;
  });
  document.querySelectorAll('section').forEach(sec => {
    sec.hidden = !sec.querySelector('tr.check:not([hidden])');
  });
}
q.addEventListener('input', filter); s.addEventListener('change', filter);
"""


def page(rows, sources, seen, recorded, repo, stamp):
    counts = defaultdict(int)
    body = []
    sections = defaultdict(list)
    for row in rows:
        sections[row["section"]].append(row)
    test_names = set()
    for title, group in sections.items():
        anchor = re.sub(r"[^a-z0-9]+", "-", title.lower()).strip("-")
        body.append(f'<section><h2 id="{anchor}">{html.escape(title)} '
                    f'<a class="muted" href="../docs/validation.html#{anchor}">§</a></h2><div class="wrap"><table>')
        body.append("<tr><th>Status</th><th>What is compared</th><th>Reference</th><th>Tolerance</th>"
                    "<th>Tests</th><th>Recorded values</th></tr>")
        for row in group:
            results, items, recs = [], [], []
            for name in row["tests"]:
                short = name.split("::")[-1]
                test_names.add(short)
                mine = seen.get(short, [])
                results += mine
                st = status_of(mine)
                where = sources.get(short, [])
                links = " ".join(f'<a href="{repo}/{p}#L{n}" title="{p}:{n}">{html.escape(os.path.basename(p))}:{n}</a>'
                                 for p, n in where) or '<span class="muted">not found in the sources</span>'
                label = {"pass": "pass", "fail": "fail", "notrun": "not run"}[st]
                if not where:
                    label = "?"
                items.append(f'<li><span class="badge {st}">{label}</span> <code>{html.escape(name)}</code> {links}</li>')
                recs += recorded.get(short, [])
            if row["tests"]:
                status = status_of(results)
                if any(r["used"] > 1 for r in recs):
                    status = "fail"
            else:
                status = "doc"
            counts[status] += 1
            label = {"pass": "pass", "fail": "fail", "notrun": "not run", "doc": "figure"}[status]
            tests_html = f'<ul class="tests">{"".join(items)}</ul>' if items else ""
            if row["test_cell"]:
                rest = re.sub(r"`[^`]+`", "", row["test_cell"]).strip(" ,;")
                if rest:
                    tests_html += f'<div class="muted">{inline(rest, repo)}</div>'
            rec_html = ""
            if recs:
                lines = []
                for r in sorted(recs, key=lambda r: (r["test"], r["what"])):
                    ok = r["used"] <= 1
                    margin = None if r["used"] == float("inf") else 1 - r["used"]
                    lines.append(
                        f'<tr><td>{html.escape(r["what"])}{" (worst of " + str(r["samples"]) + ")" if r["samples"] > 1 else ""}</td>'
                        f'<td>{fmt(r["reference"])}</td><td>{fmt(r["simulated"])}</td><td>±{fmt(r["allowed"])}</td>'
                        f'<td>{"—" if margin is None else f"{margin:.0%}"}</td>'
                        f'<td><span class="badge {"pass" if ok else "fail"}">{"pass" if ok else "fail"}</span></td></tr>')
                rec_html = ('<table class="rec"><tr><th>Value</th><th>Reference</th><th>Simulated</th><th>Allowed</th>'
                            '<th title="Share of the tolerance left: 1 − |simulated − reference| / allowed">Margin</th><th></th></tr>'
                            + "".join(lines) + "</table>")
            body.append(
                f'<tr class="check" data-status="{status}"><td><span class="badge {status}">{label}</span></td>'
                f'<td class="what">{inline(row["what"], repo)}</td><td class="against">{inline(row["against"], repo)}</td>'
                f'<td>{inline(row["tolerance"], repo)}</td><td class="tests">{tests_html}</td>'
                f'<td{" class=recs" if rec_html else ""}>{rec_html}</td></tr>')
        body.append("</table></div></section>")
    n_rec = sum(len(v) for k, v in recorded.items())
    tiles = [("checks", len(rows)), ("passed", counts["pass"]), ("failed", counts["fail"]),
             ("not run", counts["notrun"]), ("figures without a test", counts["doc"]),
             ("tests named", len(test_names))]
    tiles_html = "".join(f'<div class="tile"><b>{v}</b><span>{k}</span></div>' for k, v in tiles)
    return f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Validation dashboard</title><style>{CSS}</style></head>
<body><main>
<h1>Validation dashboard</h1>
<p class="lede">Every check of <a href="../docs/validation.html">docs/validation.md</a> against an outside number (hand
calculation, SciPy, SymPy, MuJoCo, vendor formula, the ISW measurement), with the outcome of the test that makes it in
this run. Where a test records its values (<code>sim_core::testkit::record_check</code>) the reference, the simulated
value, the difference allowed and the margin left are shown; the margin is the share of the tolerance not used, of the
worst sample. A check passes when all its tests pass. These checks prove that the equations are solved as documented;
see <a href="../docs/validation.html#what-is-not-validated">what is not validated</a>. {html.escape(stamp)}</p>
<div class="tiles">{tiles_html}</div>
<div class="bar"><input id="q" type="search" placeholder="Filter by text, test name, section" aria-label="Filter">
<select id="s" aria-label="Status"><option value="">Every status</option><option value="pass">Passed</option>
<option value="fail">Failed</option><option value="notrun">Not run</option><option value="doc">Figures</option></select>
<span class="muted">{n_rec} recorded values · <a href="../api/">API docs</a> · <a href="../docs/">book</a></span></div>
{"".join(body)}
</main><script>{SCRIPT}</script></body></html>
"""


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--validation", default=os.path.join(ROOT, "docs", "validation.md"))
    ap.add_argument("--junit", help="nextest's JUnit file of the run")
    ap.add_argument("--records", help="the JSON lines the tests wrote (KS_VALIDATION_OUT)")
    ap.add_argument("--out", default=os.path.join(ROOT, "target", "validation"), help="folder for index.html")
    ap.add_argument("--repo", default="https://github.com/Sparkz400/KinematicsSimulation/blob/main",
                    help="the base of the source links")
    ap.add_argument("--stamp", default="", help="a line on the run (commit, date)")
    ap.add_argument("--nextest-filter", action="store_true",
                    help="print a nextest filterset of the Rust tests the tables name, and nothing else")
    ap.add_argument("--strict", action="store_true", help="exit 1 when a check failed or a named test is not found")
    a = ap.parse_args()

    with open(a.validation, encoding="utf-8") as fh:
        rows = checks(fh.read())
    names = sorted({n for r in rows for n in r["tests"]})
    sources = find_sources(names)
    if a.nextest_filter:
        rust = sorted({n.split("::")[-1] for n in names if any(p.endswith(".rs") for p, _ in sources.get(n.split("::")[-1], []))})
        # One regular expression: some 20 kB, too long for a Windows command line, fine on Linux.
        print(f"test(/(^|::)({'|'.join(rust)})($|::)/)")
        return 0
    seen, recorded = outcomes(a.junit), records(a.records)
    os.makedirs(a.out, exist_ok=True)
    with open(os.path.join(a.out, "index.html"), "w", encoding="utf-8", newline="\n") as fh:
        fh.write(page(rows, sources, seen, recorded, a.repo.rstrip("/"), a.stamp))
    missing = [n for n in names if not sources.get(n.split("::")[-1])]
    failed = [r["what"][:80] for r in rows if r["tests"] and status_of([o for n in r["tests"] for o in seen.get(n.split("::")[-1], [])]) == "fail"]
    print(f"{len(rows)} checks, {len(names)} test names ({len(missing)} not found in the sources), "
          f"{sum(len(v) for v in recorded.values())} recorded values -> {os.path.join(a.out, 'index.html')}")
    for n in missing:
        print(f"  not found: {n}", file=sys.stderr)
    for w in failed:
        print(f"  failed: {w}", file=sys.stderr)
    return 1 if a.strict and (missing or failed) else 0


if __name__ == "__main__":
    sys.exit(main())
