//! The sizing report a customer signs: the machine, plots of the cycle, every verdict with its value, limit and
//! margin, the error bands, where each input comes from (`[catalogue]`, `[estimate]`, `[cad]` in the config's
//! comments) and a sign-off block. One document model, written as self-contained HTML (plots as inline SVG) or as
//! PDF. The PDF is written here: A4, the standard Helvetica fonts, no compression, so no dependency is needed.

use crate::analysis::{Band, bounds, record_cycle, utilisation, verdict_bands};
use crate::{Sim, catalog};
use sim_core::{Config, ConfigError, MachineEntry, Translations};
use std::collections::BTreeMap;
use std::fmt::Write as _;

/// One part of the report, in reading order.
#[derive(Debug, Clone, PartialEq)]
pub enum Block {
    Heading(String),
    Text(String),
    /// Column titles, and rows with whether the row reports a failed check.
    Table(Vec<String>, Vec<(Vec<String>, bool)>),
    Plot(Plot),
    /// The roles that sign, each with the lines to fill in.
    SignOff(Vec<String>, Vec<String>),
}

#[derive(Debug, Clone, PartialEq)]
pub struct Plot {
    pub title: String,
    pub x_label: String,
    pub y_label: String,
    /// Name, points, and whether they are drawn as dots (operating points) rather than a line.
    pub series: Vec<(String, Vec<[f64; 2]>, bool)>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Report {
    pub title: String,
    pub blocks: Vec<Block>,
    /// Every check passed (with `bands`: in the worst case too).
    pub passed: bool,
}

/// The source of each value of a config text, by dotted path, as its comments say: a comment line naming
/// `[catalogue]`, `[estimate]` or `[cad]` (any case) marks the keys right below it, up to a blank line or the next
/// comment; a `[cad]` line, which the CAD tools write per key, marks only the key right below it. A tag in a comment
/// at the end of a key's line marks that key. One right above or after a table header marks every key of the table
/// that has none of its own. A comment naming two tags gives both, "[catalogue] / [estimate]": which value is which
/// is in its words.
pub fn sources(text: &str) -> BTreeMap<String, String> {
    let tags = |comment: &str| {
        let lower = comment.to_lowercase();
        let mut found: Vec<(usize, &str)> = ["catalogue", "estimate", "cad"]
            .into_iter()
            .filter_map(|t| lower.find(&format!("[{t}]")).map(|at| (at, t)))
            .collect();
        found.sort();
        (!found.is_empty()).then(|| found.iter().map(|(_, t)| format!("[{t}]")).collect::<Vec<_>>().join(" / "))
    };
    let (mut out, mut table, mut table_tag) = (BTreeMap::new(), Some(String::new()), None);
    // The tags of the comment block just read (`Some(None)`: a block without one), and those the keys below carry.
    let (mut block, mut current): (Option<Option<String>>, Option<String>) = (None, None);
    for line in text.lines().map(str::trim) {
        let (code, comment) = line.split_once('#').map_or((line, ""), |(code, comment)| (code.trim(), comment));
        if line.is_empty() {
            (block, current) = (None, None);
        } else if code.is_empty() {
            // The tag nearest the key wins.
            block = Some(tags(comment).or(block.flatten()));
        } else if code.starts_with("[[") {
            // Arrays of tables (the cycle's segments) are the motion, not machine data.
            (table, block, current) = (None, None, None);
        } else if let Some(name) = code.strip_prefix('[').and_then(|l| l.split(']').next()) {
            (table, table_tag, block, current) =
                (Some(name.trim().to_string()), tags(comment).or(block.flatten()), None, None);
        } else if let (Some(table), Some((key, _))) = (&table, code.split_once('=')) {
            if let Some(found) = block.take() {
                current = found;
            }
            let key = key.trim().trim_matches('"');
            let path = if table.is_empty() { key.to_string() } else { format!("{table}.{key}") };
            if let Some(t) = tags(comment).or_else(|| current.clone()).or_else(|| table_tag.clone()) {
                out.insert(path, t);
            }
            if current.as_deref() == Some("[cad]") {
                current = None;
            }
        }
    }
    out
}

/// The report of a config: `text` is the config file as read (for its comments), `source` names where it came from,
/// `named` is the file with the parts named for this run in `[parts]` and not yet resolved, and `cfg` the config the
/// run uses (`named`, resolved or not, with whatever was set on top). With `bands`, the error bands of `verdict_bands`
/// over the config's `[bounds]` too.
pub fn make(
    entry: &MachineEntry,
    text: &str,
    source: &str,
    named: &Config,
    cfg: &Config,
    bands: bool,
    tr: &Translations,
) -> Result<Report, ConfigError> {
    let resolved = catalog::resolve(entry, cfg)?;
    let sim = Sim::new(entry, &resolved)?;
    let motors = sim.machine.motors().to_vec();
    let signals = sim.signal_infos();
    let drive = sim.control.drive_params().clone();
    let judged = verdict_bands(entry, &resolved, &if bands { bounds(&resolved)? } else { Vec::new() })?;
    let rows = record_cycle(entry, &resolved)?;
    let t = |key: &str| tr.tr(key).to_string();
    let motor_name =
        |k: Option<usize>| k.map_or(String::new(), |k| format!("{} ({})", motors[k].id, tr.tr(motors[k].key)));
    let failed = judged.iter().filter(|b| !(if bands { b.ok } else { b.verdict.ok })).count();
    let passed = failed == 0;

    let mut blocks = vec![Block::Heading(t("report-machine"))];
    let summary = if passed {
        tr.tr_arg("report-pass", &judged.len().to_string())
    } else {
        tr.tr_arg("report-fail", &format!("{failed} / {}", judged.len()))
    };
    let duration = rows.last().map_or(0.0, |r| r[0]);
    let facts = [
        ("report-machine-name", tr.tr(entry.key).to_string()),
        ("report-config", source.to_string()),
        ("report-date", today()),
        ("report-version", env!("CARGO_PKG_VERSION").to_string()),
        ("report-cycle-time", format!("{duration:.3} s")),
        ("report-result", summary),
    ];
    blocks.push(Block::Table(
        vec![t("report-item"), t("report-value")],
        facts.into_iter().map(|(k, v)| (vec![t(k), v], false)).collect(),
    ));
    let source_of = provenance(entry, text, named, &resolved, tr)?;
    // Resolving empties `[parts]`: its names come from `named`.
    let parts =
        named.leaves().into_iter().filter(|(path, value)| path.starts_with("parts.") && value.as_str() != Some(""));
    let described: Vec<(Vec<String>, bool)> = parts
        .chain(
            resolved
                .leaves()
                .into_iter()
                .filter(|(path, _)| path.starts_with("mass.") || path.starts_with("geometry.")),
        )
        .map(|(path, value)| (vec![label(&path, tr), shown(&value), source_of(&path)], false))
        .collect();
    blocks.push(Block::Table(vec![t("report-quantity"), t("report-value"), t("report-source")], described));

    blocks.push(Block::Heading(t("report-cycle")));
    blocks.push(Block::Text(t("report-cycle-text")));
    let column = |id: String| signals.iter().position(|s| s.id == id);
    let series = |quantity: &str| -> Vec<(String, Vec<[f64; 2]>, bool)> {
        motors
            .iter()
            .filter_map(|m| {
                column(format!("motor.{}.{quantity}", m.id))
                    .map(|c| (m.id.to_string(), rows.iter().map(|r| [r[0], r[c]]).collect(), false))
            })
            .collect()
    };
    blocks.push(Block::Plot(Plot {
        title: t("report-plot-torque"),
        x_label: "t [s]".into(),
        y_label: "M [N m]".into(),
        series: series("torque"),
    }));
    blocks.push(Block::Plot(Plot {
        title: t("report-plot-speed"),
        x_label: "t [s]".into(),
        y_label: "n [1/min]".into(),
        series: series("speed")
            .into_iter()
            .map(|(n, p, d)| (n, p.iter().map(|q| [q[0], q[1] * 60.0 / std::f64::consts::TAU]).collect(), d))
            .collect(),
    }));
    // The operating points against the drive's torque-speed curve (`DriveParams::torque_curve`, straight between these speeds).
    let mut points: Vec<(String, Vec<[f64; 2]>, bool)> = motors
        .iter()
        .filter_map(|m| {
            let (speed, torque) = (column(format!("motor.{}.speed", m.id))?, column(format!("motor.{}.torque", m.id))?);
            Some((m.id.to_string(), rows.iter().map(|r| [r[speed], r[torque]]).collect(), true))
        })
        .collect();
    let rpm = 60.0 / std::f64::consts::TAU;
    points.iter_mut().for_each(|s| s.1.iter_mut().for_each(|p| p[0] *= rpm));
    let corners = [-drive.no_load_speed, -drive.rated_speed, 0.0, drive.rated_speed, drive.no_load_speed];
    for (name, side) in [(t("report-curve-max"), 1), (t("report-curve-min"), 0)] {
        points.push((
            name,
            corners
                .iter()
                .map(|&s| [s * rpm, if side == 1 { drive.torque_curve(s).1 } else { drive.torque_curve(s).0 }])
                .collect(),
            false,
        ));
    }
    blocks.push(Block::Plot(Plot {
        title: t("report-plot-curve"),
        x_label: "n [1/min]".into(),
        y_label: "M [N m]".into(),
        series: points,
    }));

    blocks.push(Block::Heading(t("report-verdicts")));
    blocks.push(Block::Text(t("report-verdicts-text")));
    let head = ["report-check", "report-motor", "report-value", "report-limit", "report-margin", "report-result"]
        .map(t)
        .to_vec();
    let result = |ok: bool| t(if ok { "report-ok" } else { "report-not-ok" });
    let check = |b: &Band| tr.tr(&format!("check-{}", b.verdict.check)).to_string();
    let verdict_rows = judged.iter().map(|b| {
        let v = &b.verdict;
        (
            vec![
                check(b),
                motor_name(v.motor),
                number(v.value),
                number(v.limit),
                margin(1.0 - utilisation(v)),
                result(v.ok),
            ],
            !v.ok,
        )
    });
    blocks.push(Block::Table(head, verdict_rows.collect()));
    if bands {
        blocks.push(Block::Heading(t("report-bands")));
        blocks.push(Block::Text(tr.tr_arg("report-bands-text", &bounds(&resolved)?.len().to_string())));
        let head = ["report-check", "report-motor", "report-low", "report-high", "report-limit", "report-worst-case"]
            .map(t)
            .to_vec();
        let band_rows = judged.iter().map(|b| {
            (
                vec![
                    check(b),
                    motor_name(b.verdict.motor),
                    number(b.low),
                    number(b.high),
                    number(b.verdict.limit),
                    result(b.ok),
                ],
                !b.ok,
            )
        });
        blocks.push(Block::Table(head, band_rows.collect()));
    }

    blocks.push(Block::Heading(t("report-inputs")));
    blocks.push(Block::Text(t("report-inputs-text")));
    let value = |path: &str, value: toml::Value| {
        if path.starts_with("parts.") { named.get(path).map_or(String::new(), shown) } else { shown(&value) }
    };
    let inputs = resolved
        .leaves()
        .into_iter()
        .filter(|(path, _)| path != "format_version")
        .map(|(path, v)| (vec![path.clone(), value(&path, v), source_of(&path)], false));
    blocks.push(Block::Table(vec![t("report-parameter"), t("report-value"), t("report-source")], inputs.collect()));

    blocks.push(Block::Heading(t("report-sign-off")));
    blocks.push(Block::Text(t("report-sign-off-text")));
    let roles = ["report-prepared", "report-checked", "report-accepted"].map(t).to_vec();
    blocks.push(Block::SignOff(
        roles,
        ["report-name", "report-company", "report-date", "report-signature"].map(t).to_vec(),
    ));
    Ok(Report { title: format!("{} – {}", t("report-title"), tr.tr(entry.key)), blocks, passed })
}

/// Where each value the run used comes from: set on top of the file, brought by a part named in `[parts]`, or the
/// file's own comment tag (see `sources`).
fn provenance<'a>(
    entry: &MachineEntry,
    text: &str,
    named: &'a Config,
    resolved: &'a Config,
    tr: &'a Translations,
) -> Result<impl Fn(&str) -> String + 'a, ConfigError> {
    let (tags, file, from_parts) = (sources(text), Config::parse(text).ok(), catalog::resolve(entry, named)?);
    Ok(move |path: &str| {
        let (in_file, before, brought, used) = (
            file.as_ref().and_then(|f| f.get(path).ok()),
            named.get(path).ok(),
            from_parts.get(path).ok(),
            resolved.get(path).ok(),
        );
        // `[parts]` itself is emptied by resolving: its names are judged before.
        let parts = path.starts_with("parts.");
        if (!parts && used != brought) || (brought == before && before != in_file) || (parts && before != in_file) {
            tr.tr("report-source-set").to_string()
        } else if !parts && brought != before {
            tr.tr("report-source-parts").to_string()
        } else {
            tags.get(path).cloned().unwrap_or_else(|| tr.tr("report-source-none").to_string())
        }
    })
}

/// A config path with its parameter label where the translations have one (`param-<leaf>`).
fn label(path: &str, tr: &Translations) -> String {
    let key = format!("param-{}", path.rsplit('.').next().unwrap_or(path));
    if tr.has(&key) { format!("{} ({path})", tr.tr(&key)) } else { path.to_string() }
}

fn shown(value: &toml::Value) -> String {
    value.as_str().map_or_else(|| value.to_string(), str::to_string)
}

/// Four significant digits, as `sim-cli verdicts` prints them.
fn number(value: f64) -> String {
    let decimals = if value == 0.0 || !value.is_finite() { 0 } else { (3 - value.abs().log10().floor() as i32).max(0) };
    format!("{value:.*}", decimals as usize)
}

fn margin(share: f64) -> String {
    if share.is_finite() { format!("{:+.1} %", share * 100.0) } else { "–".into() }
}

/// Today's date (UTC) as YYYY-MM-DD, from the days since 1970 (H. Hinnant's civil-from-days).
fn today() -> String {
    let days =
        std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map_or(0, |d| d.as_secs() / 86_400) as i64;
    let z = days + 719_468;
    let (era, doe) = (z.div_euclid(146_097), z.rem_euclid(146_097));
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let (d, m) = (doy - (153 * mp + 2) / 5 + 1, if mp < 10 { mp + 3 } else { mp - 9 });
    format!("{:04}-{m:02}-{d:02}", yoe + era * 400 + i64::from(m <= 2))
}

const COLORS: [(f64, f64, f64); 6] = [
    (0.16, 0.38, 0.72),
    (0.85, 0.42, 0.10),
    (0.20, 0.60, 0.30),
    (0.55, 0.30, 0.65),
    (0.45, 0.45, 0.45),
    (0.45, 0.45, 0.45),
];

/// What a plot is drawn with, in a box of `w` by `h` with y downward: lines, dots, and text with its anchor (0 start,
/// 0.5 middle, 1 end).
enum Mark {
    Line(Vec<[f64; 2]>, (f64, f64, f64), f64),
    Dot([f64; 2], (f64, f64, f64)),
    Text([f64; 2], String, f64, f64),
}

/// At most `2 buckets` of the `n` points `point(0)` .. `point(n - 1)`, in their order, for drawing a long line: the
/// points cut into `buckets` stretches of about equal count, and of each the lowest and the highest point (one, where
/// they are the same). A stride would step over a peak one sample wide; this keeps every stretch's extremes, so the
/// drawn line reaches every peak there is. Fewer points than `2 buckets` are given as they are.
pub fn peak_keeping(n: usize, point: impl Fn(usize) -> [f64; 2], buckets: usize) -> Vec<[f64; 2]> {
    if n <= 2 * buckets {
        return (0..n).map(point).collect();
    }
    let mut out = Vec::with_capacity(2 * buckets);
    for b in 0..buckets {
        let (from, to) = (b * n / buckets, (b + 1) * n / buckets);
        let (mut low, mut high) = ((from, point(from)[1]), (from, point(from)[1]));
        for i in from + 1..to {
            let y = point(i)[1];
            // NaN is neither: a gap stays out of the extremes.
            if y < low.1 || low.1.is_nan() {
                low = (i, y);
            }
            if y > high.1 || high.1.is_nan() {
                high = (i, y);
            }
        }
        let (low, high) = (low.0, high.0);
        out.push(point(low.min(high)));
        if low != high {
            out.push(point(low.max(high)));
        }
    }
    out
}

/// A step of 1, 2 or 5 times a power of ten that cuts `span` into about five.
fn tick_step(span: f64) -> f64 {
    let raw = (span / 5.0).max(1e-12);
    let power = 10f64.powf(raw.log10().floor());
    [1.0, 2.0, 5.0, 10.0].into_iter().map(|f| f * power).find(|s| *s >= raw).unwrap_or(10.0 * power)
}

/// A tick's value with as many decimals as its step needs.
fn tick(value: f64, step: f64) -> String {
    let decimals = (-step.log10().floor()).max(0.0) as usize;
    // "-0" where a sum of steps lands a rounding below zero.
    format!("{:.*}", decimals, if value.abs() < step * 1e-6 { 0.0 } else { value })
}

fn marks(plot: &Plot, w: f64, h: f64) -> Vec<Mark> {
    let (left, right, top, bottom) = (52.0, w - 10.0, 34.0, h - 40.0);
    let all = || plot.series.iter().flat_map(|s| s.1.iter());
    let range = |i: usize| {
        let (lo, hi) = all().fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), p| (lo.min(p[i]), hi.max(p[i])));
        if lo.is_finite() && hi > lo {
            (lo, hi)
        } else if lo.is_finite() {
            (lo - 1.0, lo + 1.0)
        } else {
            (0.0, 1.0)
        }
    };
    let ((x0, x1), (y0, y1)) = (range(0), range(1));
    let (sx, sy) = (tick_step(x1 - x0), tick_step(y1 - y0));
    let (x0, x1, y0, y1) =
        ((x0 / sx).floor() * sx, (x1 / sx).ceil() * sx, (y0 / sy).floor() * sy, (y1 / sy).ceil() * sy);
    let at = |p: &[f64; 2]| {
        [left + (p[0] - x0) / (x1 - x0) * (right - left), bottom - (p[1] - y0) / (y1 - y0) * (bottom - top)]
    };
    let grey = (0.75, 0.75, 0.75);
    let mut out = vec![Mark::Text([left, 12.0], plot.title.clone(), 10.0, 0.0)];
    let mut x = x0;
    while x <= x1 + sx * 1e-6 {
        out.push(Mark::Line(vec![at(&[x, y0]), at(&[x, y1])], grey, 0.4));
        out.push(Mark::Text([at(&[x, y0])[0], bottom + 12.0], tick(x, sx), 7.0, 0.5));
        x += sx;
    }
    let mut y = y0;
    while y <= y1 + sy * 1e-6 {
        out.push(Mark::Line(vec![at(&[x0, y]), at(&[x1, y])], grey, 0.4));
        out.push(Mark::Text([left - 4.0, at(&[x0, y])[1] + 2.5], tick(y, sy), 7.0, 1.0));
        y += sy;
    }
    out.push(Mark::Text([(left + right) / 2.0, bottom + 24.0], plot.x_label.clone(), 8.0, 0.5));
    out.push(Mark::Text([left, top - 6.0], plot.y_label.clone(), 8.0, 0.5));
    let mut legend = left;
    for (i, (name, points, dots)) in plot.series.iter().enumerate() {
        let color = COLORS[i.min(COLORS.len() - 1)];
        // At most about 800 points a series: a cycle sampled each millisecond would make the PDF large for nothing. The
        // lowest and highest of each stretch stay, so that a peak one sample wide still shows.
        let shown: Vec<[f64; 2]> = if *dots {
            points.iter().step_by((points.len() / 800).max(1)).map(at).collect()
        } else {
            peak_keeping(points.len(), |i| points[i], 400).iter().map(at).collect()
        };
        if *dots {
            out.extend(shown.into_iter().map(|p| Mark::Dot(p, color)));
        } else {
            out.push(Mark::Line(shown, color, 1.0));
        }
        out.push(Mark::Line(vec![[legend, h - 6.0], [legend + 14.0, h - 6.0]], color, 2.0));
        out.push(Mark::Text([legend + 18.0, h - 3.5], name.clone(), 7.0, 0.0));
        legend += 30.0 + width(name, 7.0, false);
    }
    out
}

impl Plot {
    /// The plot as one SVG picture of 720 by 300, as the report draws it; the app saves its plots with it too.
    pub fn svg(&self) -> String {
        let plot = self;
        let mut out = String::new();
        let (w, h) = (720.0, 300.0);
        _ = write!(
            out,
            "<svg viewBox=\"0 0 {w} {h}\" xmlns=\"http://www.w3.org/2000/svg\" font-family=\"Helvetica,Arial,sans-serif\">"
        );
        for mark in marks(plot, w, h) {
            match mark {
                Mark::Line(points, c, width) => {
                    let p: Vec<String> = points.iter().map(|p| format!("{:.1},{:.1}", p[0], p[1])).collect();
                    _ = write!(
                        out,
                        "<polyline fill=\"none\" stroke=\"{}\" stroke-width=\"{width}\" points=\"{}\"/>",
                        rgb(c),
                        p.join(" ")
                    );
                }
                Mark::Dot(p, c) => {
                    _ = write!(out, "<circle cx=\"{:.1}\" cy=\"{:.1}\" r=\"1\" fill=\"{}\"/>", p[0], p[1], rgb(c))
                }
                Mark::Text(p, text, size, anchor) => {
                    let anchor = if anchor == 0.0 {
                        "start"
                    } else if anchor == 1.0 {
                        "end"
                    } else {
                        "middle"
                    };
                    _ = write!(
                        out,
                        "<text x=\"{:.1}\" y=\"{:.1}\" font-size=\"{size}\" text-anchor=\"{anchor}\">{}</text>",
                        p[0],
                        p[1],
                        escape(&text)
                    );
                }
            }
        }
        out + "</svg>"
    }
}

fn escape(text: &str) -> String {
    text.replace('&', "&amp;").replace('<', "&lt;").replace('>', "&gt;").replace('"', "&quot;")
}

fn rgb(c: (f64, f64, f64)) -> String {
    format!("rgb({},{},{})", (c.0 * 255.0) as u8, (c.1 * 255.0) as u8, (c.2 * 255.0) as u8)
}

impl Report {
    /// One HTML file with no outside references, for a browser or for printing.
    pub fn html(&self, lang: &str) -> String {
        let mut out = format!(
            "<!doctype html>\n<html lang=\"{lang}\"><head><meta charset=\"utf-8\"><title>{0}</title><style>\
             @page{{size:A4;margin:18mm}} body{{font:13px/1.45 Helvetica,Arial,sans-serif;color:#111;max-width:960px;margin:24px auto;padding:0 16px}}\
             h1{{font-size:22px}} h2{{font-size:16px;margin-top:28px;border-bottom:1px solid #999}} table{{border-collapse:collapse;width:100%;margin:8px 0}}\
             th,td{{border:1px solid #bbb;padding:3px 6px;text-align:left;vertical-align:top}} th{{background:#eee}} tr.fail td{{background:#fbe3e3;color:#900}}\
             svg{{width:100%;max-width:720px;display:block;margin:8px 0}} .sign{{display:grid;grid-template-columns:repeat(3,1fr);gap:16px}}\
             .sign div{{border:1px solid #999;padding:8px}} .sign p{{margin:18px 0 0;border-bottom:1px solid #333;font-size:11px;color:#555}}\
             tr{{break-inside:avoid}} .sign,svg{{break-inside:avoid}}</style></head><body>\n<h1>{0}</h1>\n",
            escape(&self.title)
        );
        for block in &self.blocks {
            match block {
                Block::Heading(text) => _ = writeln!(out, "<h2>{}</h2>", escape(text)),
                Block::Text(text) => _ = writeln!(out, "<p>{}</p>", escape(text)),
                Block::Table(head, rows) => {
                    out += "<table><tr>";
                    head.iter().for_each(|h| _ = write!(out, "<th>{}</th>", escape(h)));
                    out += "</tr>\n";
                    for (cells, fail) in rows {
                        out += if *fail { "<tr class=\"fail\">" } else { "<tr>" };
                        cells.iter().for_each(|c| _ = write!(out, "<td>{}</td>", escape(c)));
                        out += "</tr>\n";
                    }
                    out += "</table>\n";
                }
                Block::Plot(plot) => out += &(plot.svg() + "\n"),
                Block::SignOff(roles, fields) => {
                    out += "<div class=\"sign\">";
                    for role in roles {
                        _ = write!(out, "<div><b>{}</b>", escape(role));
                        fields.iter().for_each(|f| _ = write!(out, "<p>{}</p>", escape(f)));
                        out += "</div>";
                    }
                    out += "</div>\n";
                }
            }
        }
        out + "</body></html>\n"
    }

    /// The same report as a PDF of A4 pages.
    pub fn pdf(&self) -> Vec<u8> {
        let mut pdf = Pdf::new();
        pdf.text(MARGIN, &self.title, 16.0, true);
        pdf.y += 8.0;
        for block in &self.blocks {
            match block {
                Block::Heading(text) => {
                    pdf.room(60.0);
                    pdf.y += 10.0;
                    pdf.text(MARGIN, text, 12.0, true);
                    pdf.rule(pdf.y + 2.0, 0.6);
                    pdf.y += 6.0;
                }
                Block::Text(text) => {
                    for line in wrap(text, WIDTH, 9.0, false) {
                        pdf.room(12.0);
                        pdf.text(MARGIN, &line, 9.0, false);
                    }
                    pdf.y += 4.0;
                }
                Block::Table(head, rows) => pdf.table(head, rows),
                Block::Plot(plot) => {
                    let (w, h) = (WIDTH, 230.0);
                    pdf.room(h + 6.0);
                    let (x0, y0) = (MARGIN, pdf.y);
                    for mark in marks(plot, w, h) {
                        match mark {
                            Mark::Line(points, c, width) => {
                                pdf.line(&points.iter().map(|p| [x0 + p[0], y0 + p[1]]).collect::<Vec<_>>(), c, width)
                            }
                            Mark::Dot(p, c) => {
                                pdf.line(&[[x0 + p[0] - 0.6, y0 + p[1]], [x0 + p[0] + 0.6, y0 + p[1]]], c, 1.2)
                            }
                            Mark::Text(p, text, size, anchor) => pdf.text_at(
                                x0 + p[0] - anchor * width(&text, size, false),
                                y0 + p[1],
                                &text,
                                size,
                                false,
                            ),
                        }
                    }
                    pdf.y += h + 6.0;
                }
                Block::SignOff(roles, fields) => {
                    let column = WIDTH / roles.len() as f64;
                    pdf.room(40.0 + 30.0 * fields.len() as f64);
                    let top = pdf.y;
                    for (i, role) in roles.iter().enumerate() {
                        let x = MARGIN + i as f64 * column;
                        pdf.text_at(x, top + 12.0, role, 10.0, true);
                        for (j, field) in fields.iter().enumerate() {
                            let y = top + 46.0 + 30.0 * j as f64;
                            pdf.line(&[[x, y], [x + column - 14.0, y]], (0.2, 0.2, 0.2), 0.5);
                            pdf.text_at(x, y + 9.0, field, 7.0, false);
                        }
                    }
                    pdf.y = top + 46.0 + 30.0 * fields.len() as f64;
                }
            }
        }
        pdf.finish(&self.title)
    }
}

const PAGE: (f64, f64) = (595.0, 842.0);
const MARGIN: f64 = 40.0;
const WIDTH: f64 = PAGE.0 - 2.0 * MARGIN;
const BOTTOM: f64 = PAGE.1 - 50.0;

/// Helvetica's advance widths of the printable ASCII characters (AFM, per 1000 units of the font size).
const HELVETICA: [u16; 95] = [
    278, 278, 355, 556, 556, 889, 667, 191, 333, 333, 389, 584, 278, 333, 278, 278, 556, 556, 556, 556, 556, 556, 556,
    556, 556, 556, 278, 278, 584, 584, 584, 556, 1015, 667, 667, 722, 722, 667, 611, 778, 722, 278, 500, 667, 556, 833,
    722, 778, 667, 778, 722, 667, 611, 722, 667, 944, 667, 667, 611, 278, 278, 278, 469, 556, 333, 556, 556, 500, 556,
    556, 278, 556, 556, 222, 222, 500, 222, 833, 556, 556, 556, 556, 333, 500, 278, 556, 500, 722, 500, 500, 500, 334,
    260, 334, 584,
];

/// Text width in points. Bold is taken as 6 % wider: shortcut: the bold metrics are not in the table, upgrade if a
/// bold column ever needs to fit exactly.
fn width(text: &str, size: f64, bold: bool) -> f64 {
    let units: f64 =
        text.chars().map(|c| HELVETICA.get((c as usize).wrapping_sub(32)).copied().unwrap_or(556) as f64).sum();
    units * size / 1000.0 * if bold { 1.06 } else { 1.0 }
}

/// `text` broken at spaces into lines no wider than `room`; a word wider than the room is cut.
fn wrap(text: &str, room: f64, size: f64, bold: bool) -> Vec<String> {
    let mut lines = vec![String::new()];
    for word in text.split(' ') {
        let last = lines.last_mut().unwrap();
        let joined = if last.is_empty() { word.to_string() } else { format!("{last} {word}") };
        if width(&joined, size, bold) <= room || last.is_empty() {
            *last = joined;
        } else {
            lines.push(word.to_string());
        }
        while width(lines.last().unwrap(), size, bold) > room && lines.last().unwrap().chars().count() > 1 {
            let long = lines.pop().unwrap();
            let fit = (1..=long.chars().count())
                .take_while(|&n| width(&long.chars().take(n).collect::<String>(), size, bold) <= room)
                .last()
                .unwrap_or(1);
            lines.push(long.chars().take(fit).collect());
            lines.push(long.chars().skip(fit).collect());
        }
    }
    lines
}

/// A text as a PDF string in WinAnsiEncoding, the encoding of the standard fonts: Latin-1 as it is, the few
/// typographic characters it has elsewhere, anything else as "?".
fn pdf_string(text: &str) -> Vec<u8> {
    let mut out = vec![b'('];
    for c in text.chars() {
        let byte = match c {
            ' '..='~' | '\u{a0}'..='\u{ff}' => c as u8,
            '–' => 0x96,
            '—' => 0x97,
            '‚' => 0x82,
            '„' => 0x84,
            '‘' => 0x91,
            '’' => 0x92,
            '“' => 0x93,
            '”' => 0x94,
            '•' => 0x95,
            '…' => 0x85,
            '€' => 0x80,
            '−' => b'-',
            _ => b'?',
        };
        if matches!(byte, b'(' | b')' | b'\\') {
            out.push(b'\\');
        }
        out.push(byte);
    }
    out.push(b')');
    out
}

/// Pages being written; `y` is the next line's top, from the top of the page.
struct Pdf {
    pages: Vec<Vec<u8>>,
    y: f64,
}

impl Pdf {
    fn new() -> Self {
        Pdf { pages: vec![Vec::new()], y: MARGIN }
    }

    /// A new page unless `height` still fits on this one.
    fn room(&mut self, height: f64) {
        if self.y + height > BOTTOM {
            self.pages.push(Vec::new());
            self.y = MARGIN;
        }
    }

    fn page(&mut self) -> &mut Vec<u8> {
        self.pages.last_mut().unwrap()
    }

    fn text_at(&mut self, x: f64, y: f64, text: &str, size: f64, bold: bool) {
        let page = self.page();
        _ = write!(Bytes(page), "BT /{} {size} Tf {x:.2} {:.2} Td ", if bold { "F2" } else { "F1" }, PAGE.1 - y);
        page.extend(pdf_string(text));
        page.extend(b" Tj ET\n");
    }

    /// A line of text at the left edge `x`, moving down by its leading.
    fn text(&mut self, x: f64, text: &str, size: f64, bold: bool) {
        self.y += size * 1.3;
        self.text_at(x, self.y - size * 0.3, text, size, bold);
    }

    fn line(&mut self, points: &[[f64; 2]], c: (f64, f64, f64), width: f64) {
        let Some(first) = points.first() else { return };
        let page = self.page();
        _ = write!(Bytes(page), "{width} w {} {} {} RG {:.2} {:.2} m", c.0, c.1, c.2, first[0], PAGE.1 - first[1]);
        points[1..].iter().for_each(|p| _ = write!(Bytes(page), " {:.2} {:.2} l", p[0], PAGE.1 - p[1]));
        page.extend(b" S\n");
    }

    fn rule(&mut self, y: f64, width: f64) {
        self.line(&[[MARGIN, y], [MARGIN + WIDTH, y]], (0.5, 0.5, 0.5), width);
    }

    fn fill(&mut self, x: f64, y: f64, w: f64, h: f64, c: (f64, f64, f64)) {
        _ = writeln!(
            Bytes(self.page()),
            "{} {} {} rg {x:.2} {:.2} {w:.2} {h:.2} re f 0 0 0 rg",
            c.0,
            c.1,
            c.2,
            PAGE.1 - y - h
        );
    }

    /// A table across the page: columns as wide as their widest cell where all fit, the wide ones narrowed in
    /// proportion where not; cells wrap. The header repeats on each new page.
    fn table(&mut self, head: &[String], rows: &[(Vec<String>, bool)]) {
        let (size, pad) = (7.5, 3.0);
        let natural: Vec<f64> = (0..head.len())
            .map(|i| {
                std::iter::once(width(&head[i], size, true))
                    .chain(rows.iter().map(|r| width(&r.0[i], size, false)))
                    .fold(0.0, f64::max)
                    + 2.0 * pad
            })
            .collect();
        let fair = WIDTH / head.len() as f64;
        let narrow: f64 = natural.iter().filter(|w| **w <= fair).sum();
        let wide: f64 = natural.iter().filter(|w| **w > fair).sum();
        let scale = if narrow + wide <= WIDTH { 1.0 } else { (WIDTH - narrow) / wide };
        let widths: Vec<f64> = natural.iter().map(|w| if *w <= fair { *w } else { w * scale }).collect();
        let draw = |pdf: &mut Pdf, cells: &[String], bold: bool, shade: Option<(f64, f64, f64)>| {
            let lines: Vec<Vec<String>> =
                cells.iter().zip(&widths).map(|(c, w)| wrap(c, w - 2.0 * pad, size, bold)).collect();
            let height = lines.iter().map(Vec::len).max().unwrap_or(1) as f64 * size * 1.25 + 2.0 * pad;
            pdf.room(height);
            let top = pdf.y;
            if let Some(color) = shade {
                pdf.fill(MARGIN, top, widths.iter().sum(), height, color);
            }
            let mut x = MARGIN;
            for (cell, w) in lines.iter().zip(&widths) {
                for (n, line) in cell.iter().enumerate() {
                    pdf.text_at(x + pad, top + pad + size * (1.0 + 1.25 * n as f64) - 1.0, line, size, bold);
                }
                x += w;
            }
            pdf.line(
                &[[MARGIN, top + height], [MARGIN + widths.iter().sum::<f64>(), top + height]],
                (0.7, 0.7, 0.7),
                0.4,
            );
            pdf.y = top + height;
        };
        self.room(30.0);
        draw(self, head, true, Some((0.92, 0.92, 0.92)));
        for (cells, fail) in rows {
            if self.y + 20.0 > BOTTOM {
                self.room(BOTTOM);
                draw(self, head, true, Some((0.92, 0.92, 0.92)));
            }
            draw(self, cells, false, fail.then_some((0.98, 0.88, 0.88)));
        }
        self.y += 6.0;
    }

    /// The pages with a footer each, in a PDF file.
    fn finish(mut self, title: &str) -> Vec<u8> {
        let count = self.pages.len();
        for n in 0..count {
            let footer = format!("{title} – {} / {count}", n + 1);
            let page = &mut self.pages[n];
            _ = write!(Bytes(page), "BT /F1 7 Tf {MARGIN} 28 Td ");
            page.extend(pdf_string(&footer));
            page.extend(b" Tj ET\n");
        }
        // Objects: 1 catalogue, 2 page tree, 3 and 4 fonts, then per page its page object and its contents.
        let mut objects: Vec<Vec<u8>> = vec![
            b"<< /Type /Catalog /Pages 2 0 R >>".to_vec(),
            format!(
                "<< /Type /Pages /Count {count} /Kids [{}] >>",
                (0..count).map(|n| format!("{} 0 R", 5 + 2 * n)).collect::<Vec<_>>().join(" ")
            )
            .into_bytes(),
            b"<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica /Encoding /WinAnsiEncoding >>".to_vec(),
            b"<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica-Bold /Encoding /WinAnsiEncoding >>".to_vec(),
        ];
        for (n, content) in self.pages.into_iter().enumerate() {
            let page = format!(
                "<< /Type /Page /Parent 2 0 R /MediaBox [0 0 {} {}] /Resources << /Font << /F1 3 0 R /F2 4 0 R >> >> /Contents {} 0 R >>",
                PAGE.0,
                PAGE.1,
                6 + 2 * n
            );
            objects.push(page.into_bytes());
            let mut stream = format!("<< /Length {} >>\nstream\n", content.len()).into_bytes();
            stream.extend(content);
            stream.extend(b"\nendstream");
            objects.push(stream);
        }
        let mut out = b"%PDF-1.4\n%\xe2\xe3\xcf\xd3\n".to_vec();
        let mut offsets = Vec::new();
        for (i, object) in objects.iter().enumerate() {
            offsets.push(out.len());
            out.extend(format!("{} 0 obj\n", i + 1).bytes());
            out.extend(object);
            out.extend(b"\nendobj\n");
        }
        let xref = out.len();
        out.extend(format!("xref\n0 {}\n0000000000 65535 f \n", objects.len() + 1).bytes());
        offsets.iter().for_each(|o| out.extend(format!("{o:010} 00000 n \n").bytes()));
        out.extend(
            format!("trailer\n<< /Size {} /Root 1 0 R >>\nstartxref\n{xref}\n%%EOF\n", objects.len() + 1).bytes(),
        );
        out
    }
}

/// `write!` into a byte buffer.
struct Bytes<'a>(&'a mut Vec<u8>);

impl std::fmt::Write for Bytes<'_> {
    fn write_str(&mut self, s: &str) -> std::fmt::Result {
        self.0.extend(s.as_bytes());
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sources_follow_the_comments() {
        let text = "format_version = 1
# [estimate] all of these
[mass]
# [catalogue] from the data sheet
a_kg = 1.0
b_kg = 2.0

c_kg = 3.0
# [CAD] model.step: m=4
d_kg = 4.0
d2_kg = 4.5
# a note without a tag
e_kg = 5.0
f_kg = 6.0   # [catalogue] at the end of the line
# [catalogue] a line module; its efficiency [estimate]
g = 0.9
[belt]
f = 1
[thermal] # [estimate]
h = 2
[[cycle.segment]]
# [cad]
g = 1
";
        let s = sources(text);
        let get = |path: &str| s.get(path).map(String::as_str);
        assert_eq!(get("mass.a_kg"), Some("[catalogue]"));
        assert_eq!(get("mass.b_kg"), Some("[catalogue]"), "a tag holds for the keys right below it");
        assert_eq!(get("mass.c_kg"), Some("[estimate]"), "after a blank line the table's tag holds");
        assert_eq!(get("mass.d_kg"), Some("[cad]"));
        assert_eq!(get("mass.d2_kg"), Some("[estimate]"), "a [cad] line marks one key");
        assert_eq!(
            get("mass.e_kg"),
            Some("[estimate]"),
            "a comment without a tag ends the one above, the table's holds"
        );
        assert_eq!(get("mass.f_kg"), Some("[catalogue]"), "a tag at the end of the line");
        assert_eq!(get("mass.g"), Some("[catalogue] / [estimate]"));
        assert_eq!(get("belt.f"), None);
        assert_eq!(get("thermal.h"), Some("[estimate]"), "a tag after the header");
        assert_eq!(s.len(), 9, "nothing from the cycle's segments: {s:?}");
    }

    #[test]
    fn dates_and_numbers() {
        assert_eq!(today().len(), 10);
        assert_eq!(number(0.012345), "0.01235");
        assert_eq!(margin(0.25), "+25.0 %");
        assert_eq!(margin(f64::INFINITY), "–");
        assert_eq!(tick_step(10.0), 2.0);
        assert_eq!(tick_step(0.7), 0.2);
        assert_eq!(
            (tick(0.5, 0.5), tick(2.0, 1.0), tick(-1e-17, 0.2), tick(0.4, 0.2)),
            ("0.5".into(), "2".into(), "0.0".into(), "0.4".into())
        );
    }

    #[test]
    fn pdf_is_well_formed() {
        let rows: Vec<(Vec<String>, bool)> = (0..200)
            .map(|i| {
                (
                    vec![
                        format!(
                            "row {i} (with brackets) and a long text that wraps over more than one line of its column"
                        ),
                        "1.0".into(),
                    ],
                    i == 3,
                )
            })
            .collect();
        let plot = Plot {
            title: "p".into(),
            x_label: "x".into(),
            y_label: "y".into(),
            series: vec![("a".into(), vec![[0.0, 1.0], [1.0, -2.0]], false), ("b".into(), vec![[0.5, 0.5]], true)],
        };
        let report = Report {
            title: "Größe – test".into(),
            blocks: vec![
                Block::Heading("h".into()),
                Block::Table(vec!["a".into(), "b".into()], rows),
                Block::Plot(plot),
                Block::SignOff(vec!["x".into()], vec!["Name".into()]),
            ],
            passed: true,
        };
        let pdf = report.pdf();
        // One character per byte, so that offsets in the text are offsets in the file.
        let text: String = pdf.iter().map(|&b| if b.is_ascii() { b as char } else { '?' }).collect();
        assert!(text.starts_with("%PDF-1.4") && text.ends_with("%%EOF\n"));
        // Every object starts where the cross-reference table says.
        let xref: usize = text.rsplit("startxref\n").next().unwrap().lines().next().unwrap().parse().unwrap();
        let entries: Vec<usize> =
            text[xref..].lines().skip(3).take_while(|l| l.ends_with(" n ")).map(|l| l[..10].parse().unwrap()).collect();
        for (i, offset) in entries.iter().enumerate() {
            assert!(pdf[*offset..].starts_with(format!("{} 0 obj", i + 1).as_bytes()), "object {}", i + 1);
        }
        let pages = text.matches("/Type /Page ").count();
        assert!(pages >= 3, "200 wrapped rows take several pages: {pages}");
        assert!(text.contains(&format!("/Count {pages}")));
        // Each stream's length is right.
        for part in text.split("<< /Length ").skip(1) {
            let length: usize = part.split_whitespace().next().unwrap().parse().unwrap();
            let body = part.split_once("stream\n").unwrap().1;
            assert_eq!(&body.as_bytes()[length..length + 10], b"\nendstream");
        }
        assert!(pdf.windows(4).any(|w| w == b"Gr\xf6\xdf"), "German letters in WinAnsi");
        assert!(text.contains("\\(with brackets\\)"));
        let html = report.html("en");
        assert!(html.contains("<tr class=\"fail\">") && html.contains("<svg") && html.contains("Größe"));
    }
}
