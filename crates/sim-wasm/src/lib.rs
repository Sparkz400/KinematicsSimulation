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
//! | 3    | machine id, language, then a config (TOML)      | `{"dt_s", "centre", "radius"}`: the 3D view starts  |
//!
//! An empty config in kind 2 and 3 is the machine's default one. Parts the config names (`[parts]`) come from the
//! built-in catalogue, as in every front end. WebAssembly starts no threads, so analyses that share the processors
//! natively run one after another here.
//!
//! The 3D view: kind 3 sets the machine at its start pose and starts its cycle; `centre` and `radius` are the sphere
//! round its scene there, to fit the camera to. Then each frame the page calls `frame(seconds)`, which runs the
//! simulation on by that many seconds and returns where `frame_len()` 32-bit floats lie, no JSON: a head of `HEAD`
//! floats (`time_s`, `cycle_s`, the tool's x, y, z, the number of parts; NaN for what is not known yet) and then
//! `RECORD` floats per part, as `record` lays them out.

use sim_components::analysis::{analyse_cycle_with, verdicts};
use sim_components::{Command, Sim, load_config};
use sim_core::scene::{Part, Scene, Shape, part_color};
use sim_core::{Config, MachineEntry, Translations};
use sim_machines::{LANGUAGES, MACHINES, find, translations};
use std::fmt::Write;

/// Floats in front of the parts in a frame.
pub const HEAD: usize = 6;
/// Floats per part in a frame.
pub const RECORD: usize = 20;

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
        3 => play(first, language(second), rest),
        _ => Err(format!("unknown request kind: {kind}")),
    }
}

/// The machine, its translations with the checks' labels, and the config of the request, read and resolved.
fn configured(id: &str, lang: &str, text: &str) -> Result<(&'static MachineEntry, Translations, Config), String> {
    let entry = find(id).ok_or_else(|| format!("unknown machine: {id}"))?;
    let mut tr = translations(lang, entry);
    CHECK_LABELS.iter().filter(|(l, _)| *l == lang).for_each(|(_, t)| tr.add(t));
    let refused = |e: sim_core::ConfigError| tr.tr_arg(e.key(), e.detail());
    let cfg: Config =
        if text.trim().is_empty() { entry.config() } else { load_config(entry, text).map_err(refused)? };
    let cfg = sim_components::catalog::resolve(entry, &cfg).map_err(refused)?;
    Ok((entry, tr, cfg))
}

/// The machine of the 3D view: its simulation, the frame it hands the page, and the cycle passes it has seen.
struct Player {
    sim: Sim,
    scene: Scene,
    frame: Vec<f32>,
    passes: u64,
    /// When the latest pass began, once one has been seen to begin.
    pass_began: Option<f64>,
    /// How long the latest whole pass took, s; NaN before one has run.
    cycle_s: f64,
}

thread_local! {
    static PLAYER: std::cell::RefCell<Option<Player>> = const { std::cell::RefCell::new(None) };
}

/// Sets up the 3D view (kind 3): the machine at rest at its start pose, its cycle commanded.
fn play(id: &str, lang: &str, text: &str) -> Result<String, String> {
    let (entry, tr, cfg) = configured(id, lang, text)?;
    let mut sim = Sim::new(entry, &cfg).map_err(|e| tr.tr_arg(e.key(), e.detail()))?;
    sim.command(Command::StartCycle).map_err(|e| tr.tr(e.key).to_string())?;
    let mut player = Player {
        passes: sim.control.cycle_passes(),
        sim,
        scene: Scene::default(),
        frame: Vec::new(),
        pass_began: None,
        cycle_s: f64::NAN,
    };
    player.advance(0.0);
    let (lo, hi) = bounds(&player.frame[HEAD..]);
    let centre: [f32; 3] = std::array::from_fn(|i| (lo[i] + hi[i]) / 2.0);
    let radius = (0..3).map(|i| (hi[i] - lo[i]).powi(2)).sum::<f32>().sqrt() / 2.0;
    let json = format!(
        "{{\"dt_s\":{},\"centre\":[{},{},{}],\"radius\":{}}}",
        number(player.sim.dt),
        number(centre[0].into()),
        number(centre[1].into()),
        number(centre[2].into()),
        number(radius.into())
    );
    PLAYER.with(|p| *p.borrow_mut() = Some(player));
    Ok(json)
}

impl Player {
    /// Runs on by `seconds` (whole base steps) and writes the frame.
    fn advance(&mut self, seconds: f64) {
        for _ in 0..(seconds / self.sim.dt).round() as u64 {
            self.sim.step();
            // The controller counts the passes, as `analyse_cycle` does: from one count to the next is a pass.
            let passes = self.sim.control.cycle_passes();
            if passes != self.passes {
                let now = self.sim.time();
                if let Some(began) = self.pass_began {
                    self.cycle_s = now - began;
                }
                (self.passes, self.pass_began) = (passes, Some(now));
            }
        }
        self.scene = Scene::default();
        self.sim.scene(&mut self.scene);
        let tool = self.scene.tool.map_or([f32::NAN; 3], |p| p.map(|v| v as f32));
        self.frame.clear();
        let head =
            [self.sim.time() as f32, self.cycle_s as f32, tool[0], tool[1], tool[2], self.scene.parts.len() as f32];
        self.frame.extend(head);
        self.scene.parts.iter().for_each(|part| record(part, &mut self.frame));
    }
}

/// One part as the page draws it, `RECORD` floats: the mesh (0 a cube of side 1, 1 a cylinder of diameter and length
/// 1 along y, both centred on the origin), the matrix that takes the mesh to the part in scene coordinates (4 × 4,
/// column by column, as WebGL takes it), and the colour (`part_color`, red, green, blue in 0..1).
/// A segment is a cube stretched from `pos` to `to`, `width` wide in the depth (y) and 4 mm thick, as in sim-app.
pub fn record(part: &Part, out: &mut Vec<f32>) {
    // Sizes are drawn as their size, whatever the sign, and never flat, so that the shader can invert the matrix.
    let size = |v: f64| v.abs().max(1e-6);
    let (mesh, columns, at) = match &part.shape {
        Shape::Box { size: s } => {
            let (sin, cos) = part.angle.sin_cos();
            // The angle turns from +x towards +z, about y.
            let x = [cos, 0.0, sin].map(|v| v * size(s[0]));
            let z = [-sin, 0.0, cos].map(|v| v * size(s[2]));
            (0.0, [x, [0.0, size(s[1]), 0.0], z], part.pos)
        }
        Shape::Cylinder { radius, length } => {
            let d = size(2.0 * radius);
            (1.0, [[d, 0.0, 0.0], [0.0, size(*length), 0.0], [0.0, 0.0, d]], part.pos)
        }
        Shape::Segment { to, width } => {
            let along: [f64; 3] = std::array::from_fn(|i| to[i] - part.pos[i]);
            let length = size(along.iter().map(|v| v * v).sum::<f64>().sqrt());
            let u = along.map(|v| v / length);
            // The depth axis, or x for a segment along the depth, made square to the segment.
            let near = if u[1].abs() > 0.99 { [1.0, 0.0, 0.0] } else { [0.0, 1.0, 0.0] };
            let dot = (0..3).map(|i| near[i] * u[i]).sum::<f64>();
            let w: [f64; 3] = std::array::from_fn(|i| near[i] - dot * u[i]);
            let w_len = w.iter().map(|v| v * v).sum::<f64>().sqrt();
            let w = w.map(|v| v / w_len);
            let n = [u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0]];
            let mid = std::array::from_fn(|i| part.pos[i] + along[i] / 2.0);
            (0.0, [u.map(|v| v * length), w.map(|v| v * size(*width)), n.map(|v| v * 0.004)], mid)
        }
    };
    out.push(mesh);
    for c in columns {
        out.extend([c[0] as f32, c[1] as f32, c[2] as f32, 0.0]);
    }
    out.extend([at[0] as f32, at[1] as f32, at[2] as f32, 1.0]);
    out.extend(part_color(part.color, part.highlight, part.heat));
}

/// The box with edges along the axes round every part of `records` (`RECORD` floats each): lowest and highest corner.
pub fn bounds(records: &[f32]) -> ([f32; 3], [f32; 3]) {
    let (mut lo, mut hi) = ([f32::INFINITY; 3], [f32::NEG_INFINITY; 3]);
    for r in records.as_chunks::<RECORD>().0 {
        for i in 0..3 {
            // The mesh fits in the unit cube; that cube's corners reach half of each column out from its centre.
            let half = (r[1 + i].abs() + r[5 + i].abs() + r[9 + i].abs()) / 2.0;
            lo[i] = lo[i].min(r[13 + i] - half);
            hi[i] = hi[i].max(r[13 + i] + half);
        }
    }
    (lo, hi)
}

/// Runs the 3D view's machine on by `seconds` and returns its frame (see the module's documentation); null before
/// kind 3 has set one up.
#[unsafe(no_mangle)]
pub extern "C" fn frame(seconds: f64) -> *const f32 {
    PLAYER.with(|p| {
        p.borrow_mut().as_mut().map_or(std::ptr::null(), |player| {
            player.advance(seconds.max(0.0));
            player.frame.as_ptr()
        })
    })
}

/// How many floats the last frame has.
#[unsafe(no_mangle)]
pub extern "C" fn frame_len() -> usize {
    PLAYER.with(|p| p.borrow().as_ref().map_or(0, |player| player.frame.len()))
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
    let (entry, tr, cfg) = configured(id, lang, text)?;
    let refused = |e: sim_core::ConfigError| tr.tr_arg(e.key(), e.detail());
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
