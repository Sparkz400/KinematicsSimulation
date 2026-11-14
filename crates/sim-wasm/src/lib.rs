//! The simulator in the browser: core, machines and analysis built for `wasm32-unknown-unknown`, behind a few C-ABI
//! functions that take and give UTF-8 text. `web/index.html` loads the module and runs a sizing check with it.
//!
//! The page writes a request into memory from `alloc`, calls `call(kind, ptr, len)` and reads the answer, JSON, from
//! `out_ptr()` and `out_len()`. `call` returns 0 when the answer is a result and 1 when it is `{"error": "..."}`.
//!
//! | kind | request lines                                   | answer                                              |
//! | ---- | ----------------------------------------------- | --------------------------------------------------- |
//! | 0    | language                                        | `[{"id", "name"}]`, every machine                   |
//! | 1    | machine id                                      | `{"config"}`, the machine's default config (TOML)   |
//! | 2    | machine id, language, then a config (TOML)      | the cycle's load per motor and its verdicts         |
//!
//! An empty config in kind 2 is the machine's default one. Parts the config names (`[parts]`) come from the built-in
//! catalogue, as in every front end. WebAssembly starts no threads, so analyses that share the processors natively
//! run one after another here.

use sim_components::analysis::{analyse_cycle_with, verdicts};
use sim_components::{Sim, load_config};
use sim_core::Config;
use sim_machines::{LANGUAGES, MACHINES, find, translations};
use std::fmt::Write;

/// The verdicts' labels are the app's: one translation source for every front end.
static CHECK_LABELS: &[(&str, &str)] =
    &[("en", include_str!("../../sim-app/i18n/en.ftl")), ("de", include_str!("../../sim-app/i18n/de.ftl"))];

thread_local! {
    static OUT: std::cell::RefCell<Vec<u8>> = const { std::cell::RefCell::new(Vec::new()) };
}

/// Room for a request of `len` bytes, which the caller writes and hands to `call`; `call` frees it.
#[unsafe(no_mangle)]
pub extern "C" fn alloc(len: usize) -> *mut u8 {
    let mut buffer = Vec::<u8>::with_capacity(len.max(1));
    let ptr = buffer.as_mut_ptr();
    std::mem::forget(buffer);
    ptr
}

/// Where the last answer is.
#[unsafe(no_mangle)]
pub extern "C" fn out_ptr() -> *const u8 {
    OUT.with(|o| o.borrow().as_ptr())
}

/// How long the last answer is, in bytes.
#[unsafe(no_mangle)]
pub extern "C" fn out_len() -> usize {
    OUT.with(|o| o.borrow().len())
}

/// # Safety
/// `ptr` comes from `alloc(len)` and holds `len` bytes written by the caller; it is not used afterwards.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn call(kind: u32, ptr: *mut u8, len: usize) -> u32 {
    let request = unsafe { Vec::from_raw_parts(ptr, len, len.max(1)) };
    let (answer, code) = match handle(kind, &String::from_utf8_lossy(&request)) {
        Ok(json) => (json, 0),
        Err(message) => (format!("{{\"error\":{}}}", quote(&message)), 1),
    };
    OUT.with(|o| *o.borrow_mut() = answer.into_bytes());
    code
}

/// The answer to a request as `call` gets it, or the message of why there is none.
pub fn handle(kind: u32, request: &str) -> Result<String, String> {
    let mut lines = request.splitn(3, '\n');
    let (first, second, rest) =
        (lines.next().unwrap_or("").trim(), lines.next().unwrap_or("").trim(), lines.next().unwrap_or(""));
    match kind {
        0 => Ok(list(language(first))),
        1 => {
            let entry = find(first).ok_or_else(|| format!("unknown machine: {first}"))?;
            Ok(format!("{{\"config\":{}}}", quote(entry.default_config)))
        }
        2 => size(first, language(second), rest),
        _ => Err(format!("unknown request kind: {kind}")),
    }
}

fn language(asked: &str) -> &'static str {
    LANGUAGES.iter().copied().find(|l| *l == asked).unwrap_or("en")
}

fn list(lang: &str) -> String {
    let rows: Vec<String> = MACHINES
        .iter()
        .map(|m| format!("{{\"id\":{},\"name\":{}}}", quote(m.id), quote(translations(lang, m).tr(m.key))))
        .collect();
    format!("[{}]", rows.join(","))
}

/// The cycle of the config, one pass measured by `analyse_cycle`, and judged as `sim-cli verdicts` judges it.
fn size(id: &str, lang: &str, text: &str) -> Result<String, String> {
    let entry = find(id).ok_or_else(|| format!("unknown machine: {id}"))?;
    let mut tr = translations(lang, entry);
    CHECK_LABELS.iter().filter(|(l, _)| *l == lang).for_each(|(_, t)| tr.add(t));
    let refused = |e: sim_core::ConfigError| tr.tr_arg(e.key(), e.detail());
    let cfg: Config =
        if text.trim().is_empty() { entry.config() } else { load_config(entry, text).map_err(refused)? };
    let cfg = sim_components::catalog::resolve(entry, &cfg).map_err(refused)?;
    let mut sim = Sim::new(entry, &cfg).map_err(refused)?;
    let motors = sim.machine.motors().to_vec();
    let warnings: Vec<String> = sim
        .limit_violations
        .iter()
        .map(|v| {
            quote(&tr.tr_arg(v.key, &format!("{}: {:.4} > {:.4}", tr.tr(motors[v.motor].key), v.needed, v.allowed)))
        })
        .collect();
    let load = analyse_cycle_with(&mut sim, false).map_err(|e| tr.tr(e.key).to_string())?;
    let judged = verdicts(&cfg, &load).map_err(refused)?;
    let mut json = format!(
        "{{\"machine\":{},\"cycle_s\":{},\"tripped\":{},\"warnings\":[{}],\"motors\":[",
        quote(entry.id),
        number(load.duration),
        load.tripped,
        warnings.join(",")
    );
    for (k, (motor, m)) in motors.iter().zip(&load.motors).enumerate() {
        let _ = write!(
            json,
            "{}{{\"id\":{},\"name\":{},\"peak_torque_nm\":{},\"rms_torque_nm\":{},\"peak_speed_rad_s\":{},\"utilisation\":{},\"thermal_utilisation\":{},\"winding_rise_k\":{}}}",
            if k == 0 { "" } else { "," },
            quote(motor.id),
            quote(tr.tr(motor.key)),
            number(m.peak_torque),
            number(m.rms_torque),
            number(m.peak_speed),
            number(m.utilisation),
            number(m.thermal_utilisation),
            number(m.winding_rise)
        );
    }
    json.push_str("],\"verdicts\":[");
    for (i, v) in judged.iter().enumerate() {
        let _ = write!(
            json,
            "{}{{\"check\":{},\"label\":{},\"motor\":{},\"value\":{},\"limit\":{},\"ok\":{}}}",
            if i == 0 { "" } else { "," },
            quote(v.check),
            quote(tr.tr(&format!("check-{}", v.check))),
            v.motor.map_or("null".into(), |k| quote(motors[k].id)),
            number(v.value),
            number(v.limit),
            v.ok
        );
    }
    json.push_str("]}");
    Ok(json)
}

/// JSON has no NaN or infinity: those are `null`.
fn number(v: f64) -> String {
    if v.is_finite() { format!("{v:?}") } else { "null".into() }
}

fn quote(text: &str) -> String {
    let mut out = String::with_capacity(text.len() + 2);
    out.push('"');
    for c in text.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if (c as u32) < 0x20 => {
                let _ = write!(out, "\\u{:04x}", c as u32);
            }
            c => out.push(c),
        }
    }
    out.push('"');
    out
}
