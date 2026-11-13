//! Drive traces as the drives' and controllers' own tools export them, turned into a `Trace` at the motor
//! shaft (rad, rad/s, N m, s) by a mapping file that names the signals and says how to scale them.
//!
//! Formats read (see docs/measurement-procedure.md for where each comes from and what is not confirmed):
//! - `csv`: any table with one header line of names (a unit in `[..]` or `(..)` after a name is read),
//!   optionally a line of units under it; comma, semicolon or tab; decimal point or comma.
//! - `sinamics`: the CSV export of a Startdrive / TIA Portal trace. Siemens does not document its layout;
//!   it is read as `csv`, with the time column found by name. The native `.ttrecx` is not documented and
//!   not read.
//! - `twincat`: the CSV export of TwinCAT 3 Scope View: a header block of name/value pairs per channel
//!   (Name, SymbolName, SampleTime[ms], Unit, ...), then a time and a value column per channel, or one time
//!   column shared by channels of the same sample time. Times in ms from the start, or full FILETIME stamps.
//!   The binary `.svdx` is not documented and not read.
//! - `codesys`: the three exports of a CODESYS trace: `.trace` (XML, UTF-16, values and timestamps per
//!   variable), `.txt` (one time column in ms or µs, space-separated) and `.trace.csv` (a block per
//!   variable with its own times).

use crate::trace::{MotorTrace, Trace, TraceError};
use std::f64::consts::TAU;
use toml::{Table, Value};

/// One recorded signal with its own time base, in the units of the file.
#[derive(Debug, Clone, PartialEq)]
pub struct Channel {
    /// The names it can be found by: its name, and in a TwinCAT export also its symbol name.
    pub names: Vec<String>,
    /// As the file states it; empty where the file does not.
    pub unit: String,
    /// s, from the start of the recording.
    pub time: Vec<f64>,
    pub values: Vec<f64>,
}

fn bad(key: &'static str, detail: impl Into<String>) -> TraceError {
    TraceError { key, detail: detail.into() }
}

/// Text from a file's bytes: UTF-16 (little or big endian) with its byte order mark, as a CODESYS `.trace`
/// is written, otherwise UTF-8 (a byte order mark dropped, invalid bytes replaced).
pub fn decode(bytes: &[u8]) -> String {
    let utf16 = |big: bool| {
        let units: Vec<u16> = bytes[2..].chunks_exact(2).map(|p| if big { u16::from_be_bytes([p[0], p[1]]) } else { u16::from_le_bytes([p[0], p[1]]) }).collect();
        String::from_utf16_lossy(&units)
    };
    match bytes {
        [0xff, 0xfe, ..] => utf16(false),
        [0xfe, 0xff, ..] => utf16(true),
        [0xef, 0xbb, 0xbf, rest @ ..] => String::from_utf8_lossy(rest).into_owned(),
        _ => String::from_utf8_lossy(bytes).into_owned(),
    }
}

/// The separator of a line: tab if it has one, else semicolon, else comma. Not the most frequent: a row
/// `0,125;2,5` has more decimal commas than separators.
fn separator(line: &str) -> char {
    ['\t', ';'].into_iter().find(|&c| line.contains(c)).unwrap_or(',')
}

/// The separator of a table: tab or semicolon where at least half of its lines have one, else comma.
fn table_separator(lines: &[&str]) -> char {
    ['\t', ';'].into_iter().find(|&c| 2 * lines.iter().filter(|l| l.contains(c)).count() >= lines.len()).unwrap_or(',')
}

/// The fields of a line, without the empty one a trailing separator leaves.
fn fields(line: &str, sep: char) -> Vec<&str> {
    let line = line.trim_end();
    line.strip_suffix(sep).unwrap_or(line).split(sep).collect()
}

/// A number in the file's notation: with a decimal comma where the separator is not a comma.
fn number(field: &str, comma: bool) -> Option<f64> {
    let field = field.trim().trim_matches('"').trim();
    let parsed = if comma { field.replace(',', ".").parse::<f64>() } else { field.parse::<f64>() };
    parsed.ok().filter(|v| v.is_finite())
}

/// A header name and the unit after it in brackets or parentheses: `speed [1/min]`, `Time (ms)`.
fn name_and_unit(header: &str) -> (String, String) {
    let header = header.trim().trim_matches('"').trim();
    for (open, close) in [('[', ']'), ('(', ')')] {
        if let (Some(a), Some(b)) = (header.rfind(open), header.rfind(close)) {
            if a < b && b == header.len() - 1 {
                return (header[..a].trim().to_string(), header[a + 1..b].trim().to_string());
            }
        }
    }
    (header.to_string(), String::new())
}

/// Seconds per unit of time; `None` for a unit that is not one.
fn seconds_per(unit: &str) -> Option<f64> {
    match normal(unit).as_str() {
        "" | "s" | "sec" => Some(1.0),
        "ms" => Some(1e-3),
        "us" => Some(1e-6),
        "ns" => Some(1e-9),
        "min" => Some(60.0),
        _ => None,
    }
}

/// A unit written without spaces, in lower case, with µ as u and ° as deg.
fn normal(unit: &str) -> String {
    unit.chars().filter(|c| !c.is_whitespace() && *c != '·' && *c != '*').collect::<String>().to_lowercase().replace(['µ', 'μ'], "u").replace('°', "deg")
}

/// A table with one shared time column: `csv` and `sinamics`. `time` names the time column; without it the
/// first one whose name has "time" or "zeit" in it, else the first column. `time_unit` overrides the unit
/// the file gives the time column.
fn read_table(text: &str, time: Option<&str>, time_unit: Option<&str>) -> Result<Vec<Channel>, TraceError> {
    let lines: Vec<&str> = text.lines().map(|l| l.trim_end_matches('\r')).filter(|l| !l.trim().is_empty()).collect();
    let sep = table_separator(&lines);
    let comma = sep != ',';
    let numeric = |l: &str| {
        let f = fields(l, sep);
        f.len() > 1 && f.iter().all(|f| f.trim().is_empty() || number(f, comma).is_some())
    };
    let first_data = lines.iter().position(|l| numeric(l));
    let Some(first_data) = first_data.filter(|&i| i > 0) else { return Err(bad("error-import-layout", "no header line followed by numbers")) };
    let width = fields(lines[first_data], sep).len();
    // Names, and under them maybe a line of units: short fields without brackets (a line of names with
    // units in brackets, or a metadata line, is not one).
    let mut names_at = first_data - 1;
    let mut units: Option<Vec<String>> = None;
    let unit_like = |l: &str| fields(l, sep).iter().all(|f| f.trim().trim_matches('"').chars().count() <= 8 && !f.contains(['[', '(']));
    if first_data >= 2 && fields(lines[first_data - 2], sep).len() == width && unit_like(lines[first_data - 1]) {
        units = Some(fields(lines[first_data - 1], sep).iter().map(|u| u.trim().trim_matches('"').to_string()).collect());
        names_at = first_data - 2;
    }
    let mut channels: Vec<Channel> = fields(lines[names_at], sep)
        .into_iter()
        .enumerate()
        .map(|(i, h)| {
            let (name, unit) = name_and_unit(h);
            let unit = units.as_ref().and_then(|u| u.get(i)).filter(|u| !u.is_empty()).cloned().unwrap_or(unit);
            Channel { names: vec![name], unit, time: Vec::new(), values: Vec::new() }
        })
        .collect();
    let mut rows: Vec<Vec<f64>> = Vec::new();
    for line in &lines[first_data..] {
        let row: Option<Vec<f64>> = fields(line, sep).iter().map(|f| number(f, comma)).collect();
        match row {
            Some(row) if row.len() == channels.len() => rows.push(row),
            _ => return Err(bad("error-trace-row", *line)),
        }
    }
    let lower = |c: &Channel| c.names[0].to_lowercase();
    let t = match time {
        Some(name) => find(&channels, name)?,
        None => channels.iter().position(|c| lower(c).contains("time") || lower(c).contains("zeit")).unwrap_or(0),
    };
    let unit = time_unit.map_or(channels[t].unit.clone(), str::to_string);
    let scale = seconds_per(&unit).ok_or_else(|| bad("error-import-unit", format!("{}: {unit}", channels[t].names[0])))?;
    let times: Vec<f64> = rows.iter().map(|r| r[t] * scale).collect();
    for (i, channel) in channels.iter_mut().enumerate() {
        channel.time.clone_from(&times);
        channel.values = rows.iter().map(|r| r[i]).collect();
    }
    channels.remove(t);
    Ok(channels)
}

/// TwinCAT 3 Scope View CSV export.
fn read_twincat(text: &str) -> Result<Vec<Channel>, TraceError> {
    let lines: Vec<&str> = text.lines().map(|l| l.trim_end_matches('\r')).collect();
    let sep = separator(lines.first().copied().unwrap_or(""));
    let comma = sep != ',';
    // The channel block starts at the second line that begins with "Name" (the first names the export).
    let start = lines.iter().enumerate().filter(|(_, l)| l.split(sep).next().map(str::trim) == Some("Name")).nth(1).map(|(i, _)| i);
    let Some(start) = start else { return Err(bad("error-import-layout", "no channel header (a second line starting with Name)")) };
    let end = (start..lines.len()).find(|&i| lines[i].trim().is_empty()).unwrap_or(lines.len());
    let header: Vec<Vec<&str>> = lines[start..end].iter().map(|l| l.split(sep).collect()).collect();
    let row = |key: &str| header.iter().find(|r| r.first().map(|k| k.trim()) == Some(key));
    let names = row("Name").ok_or_else(|| bad("error-import-layout", "Name"))?;
    // Full header: key and value per channel. Shared time columns: the key once, then the values.
    let paired = names.get(2).map(|k| k.trim()) == Some("Name");
    let value_at = |r: &Vec<&str>, c: usize| -> String { r.get(if paired { 2 * c + 1 } else { c + 1 }).map_or(String::new(), |v| v.trim().to_string()) };
    let count = if paired { names.len() / 2 } else { names.len() - 1 };
    let mut channels: Vec<Channel> = (0..count)
        .map(|c| {
            let mut found = vec![value_at(names, c)];
            if let Some(symbol) = row("SymbolName").map(|r| value_at(r, c)).filter(|s| !s.is_empty()) {
                found.push(symbol);
            }
            let unit = row("Unit").map(|r| value_at(r, c)).unwrap_or_default();
            // "(None)", or a symbol and its quantity: "V (Voltage)".
            let unit = if unit.starts_with('(') { String::new() } else { unit.split(" (").next().unwrap_or("").to_string() };
            Channel { names: found, unit, time: Vec::new(), values: Vec::new() }
        })
        .collect();
    // Data: with the full header a time and a value column per channel. With shared time columns the
    // layout of columns is not in the header; one time column followed by every channel is read.
    for line in lines[end..].iter().filter(|l| !l.trim().is_empty() && l.trim() != "EOF") {
        let fields: Vec<&str> = line.split(sep).collect();
        for (c, channel) in channels.iter_mut().enumerate() {
            let (t, v) = if paired { (fields.get(2 * c), fields.get(2 * c + 1)) } else { (fields.first(), fields.get(c + 1)) };
            // Shorter channels are padded with blank cells.
            match (t.and_then(|t| number(t, comma)), v.and_then(|v| number(v, comma))) {
                (Some(t), Some(v)) => {
                    channel.time.push(t);
                    channel.values.push(v);
                }
                _ if t.is_none_or(|t| t.trim().is_empty()) => {}
                _ => return Err(bad("error-trace-row", *line)),
            }
        }
    }
    for channel in &mut channels {
        // Full timestamps are FILETIME (100 ns since 1601); otherwise ms from the start of the export.
        let full = channel.time.first().is_some_and(|&t| t > 1e15);
        let first = channel.time.first().copied().unwrap_or(0.0);
        channel.time.iter_mut().for_each(|t| *t = if full { (*t - first) * 1e-7 } else { *t * 1e-3 });
    }
    Ok(channels)
}

/// CODESYS trace exports: `.trace` XML, `.txt` or `.trace.csv`, told apart by their content.
fn read_codesys(text: &str) -> Result<Vec<Channel>, TraceError> {
    if text.contains("<TraceVariable") {
        return read_codesys_xml(text);
    }
    let lines: Vec<&str> = text.lines().map(|l| l.trim_end_matches('\r')).collect();
    if lines.first().is_some_and(|l| l.trim_start().starts_with("[key]")) {
        return read_codesys_csv(&lines);
    }
    // .txt: a title, the project, then a header of names separated by spaces, the first the time with its unit.
    let first_data = lines.iter().position(|l| !l.trim().is_empty() && l.split_whitespace().all(|f| number(f, false).is_some()));
    let Some(first_data) = first_data.filter(|&i| i > 0) else { return Err(bad("error-import-layout", "no header line followed by numbers")) };
    let header: Vec<&str> = lines[first_data - 1].split_whitespace().collect();
    let (_, unit) = name_and_unit(header.first().copied().unwrap_or(""));
    let scale = seconds_per(&unit).ok_or_else(|| bad("error-import-unit", format!("{}: {unit}", header.first().copied().unwrap_or(""))))?;
    let mut channels: Vec<Channel> = header[1..].iter().map(|n| Channel { names: vec![n.to_string()], unit: String::new(), time: vec![], values: vec![] }).collect();
    for line in lines[first_data..].iter().filter(|l| !l.trim().is_empty()) {
        let row: Option<Vec<f64>> = line.split_whitespace().map(|f| number(f, false)).collect();
        let row = row.filter(|r| r.len() == channels.len() + 1).ok_or_else(|| bad("error-trace-row", *line))?;
        for (c, channel) in channels.iter_mut().enumerate() {
            channel.time.push(row[0] * scale);
            channel.values.push(row[c + 1]);
        }
    }
    Ok(channels)
}

/// The text between `<tag>` and `</tag>` from `from` on, and where it ends.
fn element<'a>(text: &'a str, tag: &str, from: usize) -> Option<(&'a str, usize)> {
    let open = text[from..].find(&format!("<{tag}>"))? + from + tag.len() + 2;
    let close = text[open..].find(&format!("</{tag}>"))? + open;
    Some((&text[open..close], close))
}

fn read_codesys_xml(text: &str) -> Result<Vec<Channel>, TraceError> {
    // Timestamps in µs where the configuration says so, otherwise in ms.
    let micro = text.find("Name=\"MicroSeconds\"").is_some_and(|i| text[i..].split('<').next().unwrap_or("").contains(">True"));
    let scale = if micro { 1e-6 } else { 1e-3 };
    let mut channels = Vec::new();
    let mut at = 0;
    while let Some(found) = text[at..].find("<TraceVariable") {
        let start = at + found;
        let name = text[start..].split("VarName=\"").nth(1).and_then(|r| r.split('"').next()).unwrap_or("").to_string();
        let end = text[start..].find("</TraceVariable>").map_or(text.len(), |e| start + e);
        let list = |tag: &str| -> Result<Vec<f64>, TraceError> {
            let (body, _) = element(&text[..end], tag, start).ok_or_else(|| bad("error-import-layout", format!("{name}: {tag}")))?;
            body.split(',').filter(|v| !v.trim().is_empty()).map(|v| number(v, false).ok_or_else(|| bad("error-trace-row", v.trim()))).collect()
        };
        let (values, time) = (list("Values")?, list("Timestamps")?);
        if values.len() != time.len() {
            return Err(bad("error-import-layout", format!("{name}: {} values, {} timestamps", values.len(), time.len())));
        }
        channels.push(Channel { names: vec![name], unit: String::new(), time: time.iter().map(|t| t * scale).collect(), values });
        at = end;
    }
    Ok(channels)
}

fn read_codesys_csv(lines: &[&str]) -> Result<Vec<Channel>, TraceError> {
    let field = |line: &str, i: usize| line.split(';').nth(i).map(str::trim).unwrap_or("").to_string();
    // Flags 32 was set in the one µs export seen, 16 in the ms one: an inference, see the module comment.
    let flags = lines.iter().find(|l| field(l, 0) == "Flags").map(|l| field(l, 1).parse::<u32>().unwrap_or(0)).unwrap_or(0);
    let scale = if flags & 32 != 0 { 1e-6 } else { 1e-3 };
    let mut channels: Vec<Channel> = Vec::new();
    let mut current: Option<usize> = None;
    for line in lines.iter().filter(|l| !l.trim().is_empty()) {
        let key = field(line, 0);
        if key.ends_with(".Variable") {
            channels.push(Channel { names: vec![field(line, 1)], unit: String::new(), time: vec![], values: vec![] });
            current = None;
        } else if key.ends_with(".Data") {
            current = Some(channels.len().checked_sub(1).ok_or_else(|| bad("error-import-layout", *line))?);
        } else if key.is_empty() {
            if let Some(c) = current {
                let (t, v) = (number(&field(line, 1), false), number(&field(line, 2), false));
                let (t, v) = t.zip(v).ok_or_else(|| bad("error-trace-row", *line))?;
                channels[c].time.push(t * scale);
                channels[c].values.push(v);
            }
        } else {
            current = None;
        }
    }
    Ok(channels)
}

/// The index of the channel named `name`: an exact match (case and surrounding space ignored) of any of a
/// channel's names, otherwise the one channel with `name` in one of its names.
fn find(channels: &[Channel], name: &str) -> Result<usize, TraceError> {
    let wanted = name.trim().to_lowercase();
    let exact: Vec<usize> = (0..channels.len()).filter(|&i| channels[i].names.iter().any(|n| n.trim().to_lowercase() == wanted)).collect();
    let partial: Vec<usize> = (0..channels.len()).filter(|&i| channels[i].names.iter().any(|n| n.to_lowercase().contains(&wanted))).collect();
    match (exact.as_slice(), partial.as_slice()) {
        ([i], _) | ([], [i]) => Ok(*i),
        ([], []) => {
            let all: Vec<&str> = channels.iter().map(|c| c.names[0].as_str()).collect();
            Err(bad("error-import-signal", format!("{name} (the file has: {})", all.join(", "))))
        }
        _ => Err(bad("error-import-ambiguous", name)),
    }
}

/// What a quantity is, for its units.
#[derive(Clone, Copy, PartialEq)]
enum Quantity {
    Angle,
    Speed,
    Torque,
}

/// How one signal of the file becomes a motor quantity in SI units:
/// `(raw + offset) × unit factor × scale`, with the time shifted by `time_offset_s`.
#[derive(Debug, Clone, PartialEq)]
pub struct Signal {
    pub signal: String,
    /// Overrides the file's unit; required where the file has none.
    pub unit: Option<String>,
    /// Units of the signal per motor revolution: for increments the encoder resolution (counts per turn),
    /// for a length the lead (mm of travel per motor turn). Not needed for rad, deg, rev.
    pub per_rev: Option<f64>,
    /// N m at 100 % (unit `%`) or at 1000 ‰ (unit `‰`, as CiA 402 drives give torque in per mille of rated).
    pub reference: Option<f64>,
    pub scale: f64,
    pub offset: f64,
    /// The raw value wraps around at this span (a 32-bit encoder count: 4294967296); it is unwrapped.
    pub wrap: Option<f64>,
    /// Added to the signal's times: where a device's clock or a bus delay puts it behind the others.
    pub time_offset_s: f64,
}

/// The mapping file: which signals of the file are which motor quantities.
#[derive(Debug, Clone, PartialEq)]
pub struct Mapping {
    /// csv, sinamics, twincat or codesys.
    pub format: String,
    /// The time column of a `csv`/`sinamics` table, and its unit, which overrides the one the file states.
    pub time: Option<String>,
    pub time_unit: Option<String>,
    /// The step the signals are resampled to; without it, that of the first motor's angle.
    pub sample_time_s: Option<f64>,
    pub motors: Vec<MotorMapping>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct MotorMapping {
    pub id: String,
    pub setpoint: Signal,
    pub angle: Signal,
    /// Without it, the speed is the slope of the angle.
    pub speed: Option<Signal>,
    pub torque: Signal,
    pub torque_setpoint: Option<Signal>,
}

pub const FORMATS: [&str; 4] = ["csv", "sinamics", "twincat", "codesys"];

impl Mapping {
    /// Reads a mapping file (TOML; see datasets/template/map.toml). Unknown keys are refused, so a
    /// misspelt one does not pass unnoticed.
    pub fn parse(text: &str) -> Result<Self, TraceError> {
        let table: Table = text.parse().map_err(|e: toml::de::Error| bad("error-import-map", e.to_string()))?;
        known(&table, "", &["format", "time", "time_unit", "sample_time_s", "motor"])?;
        let text_of = |t: &Table, key: &str| -> Result<Option<String>, TraceError> {
            t.get(key).map(|v| v.as_str().map(str::to_string).ok_or_else(|| bad("error-import-map", key.to_string()))).transpose()
        };
        let format = text_of(&table, "format")?.ok_or_else(|| bad("error-import-map", "format"))?;
        if !FORMATS.contains(&format.as_str()) {
            return Err(bad("error-import-map", format!("format = {format}")));
        }
        // TwinCAT and CODESYS state their time column and unit in the file's layout.
        if ["twincat", "codesys"].contains(&format.as_str()) && (table.contains_key("time") || table.contains_key("time_unit")) {
            return Err(bad("error-import-map", format!("time / time_unit with format = {format}")));
        }
        let positive = |t: &Table, key: &str| -> Result<Option<f64>, TraceError> {
            t.get(key).map(|v| v.as_float().or(v.as_integer().map(|i| i as f64)).filter(|v| *v > 0.0 && v.is_finite()).ok_or_else(|| bad("error-import-map", key.to_string()))).transpose()
        };
        let motors = table.get("motor").and_then(Value::as_array).ok_or_else(|| bad("error-import-map", "[[motor]]"))?;
        let motors = motors
            .iter()
            .enumerate()
            .map(|(i, m)| {
                let m = m.as_table().ok_or_else(|| bad("error-import-map", "[[motor]]"))?;
                known(m, "motor.", &["id", "setpoint", "angle", "speed", "torque", "torque_setpoint"])?;
                let signal = |key: &str| -> Result<Option<Signal>, TraceError> { m.get(key).map(|v| Signal::parse(v, key)).transpose() };
                let needed = |key: &str| signal(key)?.ok_or_else(|| bad("error-import-map", format!("motor.{key}")));
                Ok(MotorMapping {
                    id: text_of(m, "id")?.unwrap_or_else(|| i.to_string()),
                    setpoint: needed("setpoint")?,
                    angle: needed("angle")?,
                    speed: signal("speed")?,
                    torque: needed("torque")?,
                    torque_setpoint: signal("torque_setpoint")?,
                })
            })
            .collect::<Result<Vec<_>, TraceError>>()?;
        if motors.is_empty() {
            return Err(bad("error-import-map", "[[motor]]"));
        }
        Ok(Self { format, time: text_of(&table, "time")?, time_unit: text_of(&table, "time_unit")?, sample_time_s: positive(&table, "sample_time_s")?, motors })
    }
}

fn known(table: &Table, prefix: &str, keys: &[&str]) -> Result<(), TraceError> {
    match table.keys().find(|k| !keys.contains(&k.as_str())) {
        Some(k) => Err(bad("error-import-map", format!("{prefix}{k}"))),
        None => Ok(()),
    }
}

impl Signal {
    fn parse(value: &Value, key: &str) -> Result<Self, TraceError> {
        let wrong = |what: &str| bad("error-import-map", format!("{key}.{what}"));
        // A bare name is a signal without options.
        if let Some(name) = value.as_str() {
            return Ok(Self { signal: name.into(), unit: None, per_rev: None, reference: None, scale: 1.0, offset: 0.0, wrap: None, time_offset_s: 0.0 });
        }
        let t = value.as_table().ok_or_else(|| wrong("signal"))?;
        known(t, &format!("{key}."), &["signal", "unit", "per_rev", "reference", "scale", "offset", "wrap", "time_offset_s"])?;
        let float = |k: &str| -> Result<Option<f64>, TraceError> { t.get(k).map(|v| v.as_float().or(v.as_integer().map(|i| i as f64)).filter(|v| v.is_finite()).ok_or_else(|| wrong(k))).transpose() };
        let positive = |k: &str| -> Result<Option<f64>, TraceError> { float(k)?.map(|v| if v > 0.0 { Ok(v) } else { Err(wrong(k)) }).transpose() };
        Ok(Self {
            signal: t.get("signal").and_then(Value::as_str).ok_or_else(|| wrong("signal"))?.into(),
            unit: t.get("unit").map(|v| v.as_str().map(str::to_string).ok_or_else(|| wrong("unit"))).transpose()?,
            per_rev: positive("per_rev")?,
            reference: positive("reference")?,
            scale: float("scale")?.unwrap_or(1.0),
            offset: float("offset")?.unwrap_or(0.0),
            wrap: positive("wrap")?,
            time_offset_s: float("time_offset_s")?.unwrap_or(0.0),
        })
    }

    /// The factor from the signal's unit to rad, rad/s or N m.
    fn factor(&self, unit: &str, quantity: Quantity) -> Result<f64, TraceError> {
        let unknown = || bad("error-import-unit", format!("{}: {unit}", self.signal));
        let per_rev = |angle: &str| -> Result<f64, TraceError> {
            match angle {
                "rad" => Ok(TAU),
                "deg" => Ok(360.0),
                "rev" | "r" | "u" | "turn" | "turns" | "1" => Ok(1.0),
                _ => self.per_rev.ok_or_else(|| bad("error-import-per-rev", format!("{}: {unit}", self.signal))),
            }
        };
        let u = normal(unit);
        match quantity {
            Quantity::Angle => Ok(TAU / per_rev(&u)?),
            Quantity::Speed => {
                let (angle, time) = match u.as_str() {
                    "rpm" => ("rev", "min"),
                    _ => u.split_once('/').ok_or_else(unknown)?,
                };
                Ok(TAU / per_rev(angle)? / seconds_per(time).ok_or_else(unknown)?)
            }
            Quantity::Torque => match u.as_str() {
                "nm" => Ok(1.0),
                "mnm" => Ok(1e-3),
                "knm" => Ok(1e3),
                "%" => Ok(self.reference.ok_or_else(|| bad("error-import-reference", format!("{}: %", self.signal)))? / 100.0),
                "‰" | "permille" => Ok(self.reference.ok_or_else(|| bad("error-import-reference", format!("{}: ‰", self.signal)))? / 1000.0),
                // Any other unit (A, a raw count) only with a scale that turns it into N m.
                _ if self.scale != 1.0 => Ok(1.0),
                _ => Err(unknown()),
            },
        }
    }

    /// The signal's times and SI values.
    fn convert(&self, channels: &[Channel], quantity: Quantity) -> Result<(Vec<f64>, Vec<f64>), TraceError> {
        let channel = &channels[find(channels, &self.signal)?];
        let unit = self.unit.clone().unwrap_or_else(|| channel.unit.clone());
        let factor = self.factor(&unit, quantity)? * self.scale;
        let mut raw = channel.values.clone();
        if let Some(span) = self.wrap {
            let mut turns = 0.0;
            for i in 1..raw.len() {
                let jump = channel.values[i] - channel.values[i - 1];
                turns -= span * (jump / span).round();
                raw[i] = channel.values[i] + turns;
            }
        }
        if channel.time.len() < 2 {
            return Err(bad("error-trace-short", self.signal.clone()));
        }
        let time = channel.time.iter().map(|t| t + self.time_offset_s).collect();
        Ok((time, raw.iter().map(|v| (v + self.offset) * factor).collect()))
    }
}

/// `values` at `time`, linear between samples; times must rise.
fn resample(time: &[f64], values: &[f64], at: &[f64]) -> Vec<f64> {
    at.iter()
        .map(|&t| {
            let k = time.partition_point(|&x| x <= t).clamp(1, time.len() - 1);
            let share = (t - time[k - 1]) / (time[k] - time[k - 1]);
            values[k - 1] + share * (values[k] - values[k - 1])
        })
        .collect()
}

/// A trace read through a mapping, and per motor the torque setpoint where the mapping names one.
#[derive(Debug, Clone, PartialEq)]
pub struct Imported {
    pub trace: Trace,
    pub ids: Vec<String>,
    pub torque_setpoints: Vec<Option<Vec<f64>>>,
}

/// The channels of a file in one of the `FORMATS`.
pub fn channels(text: &str, mapping: &Mapping) -> Result<Vec<Channel>, TraceError> {
    let channels = match mapping.format.as_str() {
        "twincat" => read_twincat(text)?,
        "codesys" => read_codesys(text)?,
        _ => read_table(text, mapping.time.as_deref(), mapping.time_unit.as_deref())?,
    };
    Ok(channels)
}

/// Reads a file through a mapping: every signal converted to SI units, shifted by its time offset, and all
/// resampled (linear) onto one grid that spans the time all of them cover, from 0.
pub fn import(text: &str, mapping: &Mapping) -> Result<Imported, TraceError> {
    let channels = channels(text, mapping)?;
    let mut signals = Vec::new();
    for m in &mapping.motors {
        signals.push(Some(m.setpoint.convert(&channels, Quantity::Angle)?));
        signals.push(Some(m.angle.convert(&channels, Quantity::Angle)?));
        signals.push(m.speed.as_ref().map(|s| s.convert(&channels, Quantity::Speed)).transpose()?);
        signals.push(Some(m.torque.convert(&channels, Quantity::Torque)?));
        signals.push(m.torque_setpoint.as_ref().map(|s| s.convert(&channels, Quantity::Torque)).transpose()?);
    }
    for (time, _) in signals.iter().flatten() {
        if time.windows(2).any(|w| w[1] <= w[0]) {
            return Err(bad("error-import-time", "times do not rise"));
        }
    }
    let from = signals.iter().flatten().map(|(t, _)| t[0]).fold(f64::NEG_INFINITY, f64::max);
    let to = signals.iter().flatten().map(|(t, _)| t[t.len() - 1]).fold(f64::INFINITY, f64::min);
    let reference = &signals[1].as_ref().expect("angle").0;
    let step = mapping.sample_time_s.unwrap_or_else(|| {
        let mut steps: Vec<f64> = reference.windows(2).map(|w| w[1] - w[0]).collect();
        steps.sort_by(f64::total_cmp);
        steps[steps.len() / 2]
    });
    if !(to > from) {
        return Err(bad("error-import-time", "the signals do not overlap in time"));
    }
    let count = ((to - from) / step + 1e-9).floor() + 1.0;
    // A grid far finer than any signal is a mistake in `sample_time_s`, and would not fit in memory.
    let longest = signals.iter().flatten().map(|(t, _)| t.len()).max().unwrap_or(0) as f64;
    if count > 10.0 * longest + 10.0 {
        return Err(bad("error-import-time", format!("sample_time_s = {step} gives {count} rows for {longest} samples")));
    }
    let count = count as usize;
    let grid: Vec<f64> = (0..count).map(|i| from + i as f64 * step).collect();
    let on_grid = |s: &Option<(Vec<f64>, Vec<f64>)>| s.as_ref().map(|(t, v)| resample(t, v, &grid));
    let mut trace = Trace { time: grid.iter().map(|t| t - from).collect(), motors: Vec::new() };
    let mut torque_setpoints = Vec::new();
    for k in 0..mapping.motors.len() {
        let angle = on_grid(&signals[5 * k + 1]).expect("angle");
        let speed = on_grid(&signals[5 * k + 2]).unwrap_or_else(|| slope(&trace.time, &angle));
        trace.motors.push(MotorTrace { setpoint: on_grid(&signals[5 * k]).expect("setpoint"), angle, speed, torque: on_grid(&signals[5 * k + 3]).expect("torque") });
        torque_setpoints.push(on_grid(&signals[5 * k + 4]));
    }
    if trace.time.len() < 4 {
        return Err(bad("error-trace-short", String::new()));
    }
    Ok(Imported { trace, ids: mapping.motors.iter().map(|m| m.id.clone()).collect(), torque_setpoints })
}

/// The slope of a signal: central differences, one-sided at the ends.
fn slope(time: &[f64], values: &[f64]) -> Vec<f64> {
    let n = values.len();
    (0..n).map(|i| {
        let (a, b) = (i.saturating_sub(1), (i + 1).min(n - 1));
        (values[b] - values[a]) / (time[b] - time[a])
    })
    .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn units_convert_to_the_motor_shaft() {
        let s = |unit: &str| Signal { signal: "x".into(), unit: Some(unit.into()), per_rev: Some(10.0), reference: Some(2.0), scale: 1.0, offset: 0.0, wrap: None, time_offset_s: 0.0 };
        let close = |a: f64, b: f64| assert!((a - b).abs() < 1e-12 * b.abs(), "{a} {b}");
        close(s("1/min").factor("1/min", Quantity::Speed).unwrap(), TAU / 60.0);
        close(s("rpm").factor("rpm", Quantity::Speed).unwrap(), TAU / 60.0);
        close(s("").factor("°/s", Quantity::Speed).unwrap(), TAU / 360.0);
        close(s("").factor("mm/s", Quantity::Speed).unwrap(), TAU / 10.0);
        close(s("").factor("mm", Quantity::Angle).unwrap(), TAU / 10.0);
        close(s("").factor("inc", Quantity::Angle).unwrap(), TAU / 10.0);
        close(s("").factor("%", Quantity::Torque).unwrap(), 0.02);
        close(s("").factor("‰", Quantity::Torque).unwrap(), 0.002);
        close(s("").factor("N m", Quantity::Torque).unwrap(), 1.0);
        let bare = Signal { per_rev: None, ..s("") };
        assert_eq!(bare.factor("LU", Quantity::Angle).unwrap_err().key, "error-import-per-rev");
        assert_eq!(bare.factor("A", Quantity::Torque).unwrap_err().key, "error-import-unit");
    }

    #[test]
    fn a_wrapping_counter_is_unwrapped() {
        let channel = Channel { names: vec!["c".into()], unit: "inc".into(), time: vec![0.0, 1.0, 2.0, 3.0], values: vec![4294967000.0, 4294967200.0, 104.0, 400.0] };
        let s = Signal { signal: "c".into(), unit: None, per_rev: Some(1.0), reference: None, scale: 1.0, offset: 0.0, wrap: Some(4294967296.0), time_offset_s: 0.0 };
        let (_, v) = s.convert(&[channel], Quantity::Angle).unwrap();
        let counts: Vec<f64> = v.iter().map(|v| (v / TAU).round()).collect();
        assert_eq!(counts, vec![4294967000.0, 4294967200.0, 4294967400.0, 4294967696.0]);
    }

    #[test]
    fn utf16_is_decoded() {
        let mut bytes = vec![0xff, 0xfe];
        "<a>ä</a>".encode_utf16().for_each(|u| bytes.extend(u.to_le_bytes()));
        assert_eq!(decode(&bytes), "<a>ä</a>");
    }
}
