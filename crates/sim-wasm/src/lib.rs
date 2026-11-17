//! The simulator in the browser: core, machines and analysis built for `wasm32-unknown-unknown`, behind a few C-ABI
//! functions that take and give UTF-8 text. `web/index.html` loads the module and runs a sizing check with it.
//!
//! The page writes a request into memory from `alloc`, calls `call(kind, ptr, len)` and reads the answer, JSON, from
//! `out_ptr()` and `out_len()`. `call` returns 0 when the answer is a result and 1 when it is `{"error": "..."}`.
//!
//! | kind | request lines                                        | answer                                            |
//! | ---- | ---------------------------------------------------- | ------------------------------------------------- |
//! | 0    | language                                             | `[{"id", "name", "file"}]`, every machine         |
//! | 1    | machine id                                           | `{"config"}`, the machine's default config (TOML) |
//! | 2    | machine id, language and options, a config (TOML)    | the run's load per motor and its verdicts         |
//! | 3    | machine id, language and options, a config (TOML)    | `{"dt_s", "centre", "radius", "motors", "sample"}`: the 3D view starts |
//! | 4    | file name, language, then a machine file (TOML)      | `{"id", "name", "config"}`: the machine to use as an id |
//! | 5    | machine id, language and options, a config (TOML)    | `{"motor", "points"}`: frequency response of one motor |
//! | 6    | machine id, language, a config, `---`, a trace (CSV) | `{"rows", "advice"}`: masses and friction fitted to the trace |
//! | 7    | language, then translation keys, one a line          | `{key: text}`: the page's texts and those keys    |
//! | 8    | machine id, language, a config (TOML)                | `{"params", "parts"}`: the values the page's form edits |
//! | 9    | machine id, language, a config, `---`, `path = value` lines | `{"config", "changed", "compact"}`: the values set in the config's own text |
//! | 10   | machine id, language, a config (TOML)                | `{"step_s", "time", "signals", "motors"}`: the scopes of one pass of the cycle |
//!
//! An empty config is the machine's default one. Parts the config names (`[parts]`) come from the built-in catalogue,
//! as in every front end. WebAssembly starts no threads, so analyses that share the processors natively run one after
//! another here.
//!
//! Options follow the language on its line, separated by spaces: `path` runs the path (format: `sim_components::path`)
//! that follows the config after a line `---` instead of the cycle, timed within the axis limits or with
//! `time-optimal` as fast as the motors allow (kinds 2 and 3); `motor=K`, `from=HZ`, `to=HZ`, `count=N` and
//! `amplitude=NM` set the sweep of kind 5 as `sim-cli sweep` takes them. A line `---` is never part of a TOML config.
//!
//! Machines: the registered ones by their id, the machine files of `machines/` built into the module as
//! `file:<name>.toml`, and a file the page hands in (kind 4) as `upload:<name>`.
//!
//! The 3D view: kind 3 sets the machine at its start pose and starts its cycle or path; `centre` and `radius` are the
//! sphere round its scene there, to fit the camera to. Then the page calls `frame(seconds)`, which runs the simulation
//! on by that many seconds and returns where `frame_len()` 32-bit floats lie, no JSON: a head of `HEAD` floats
//! (`time_s`, `cycle_s`, the tool's x, y, z, the number of parts, of obstacles and of samples; NaN for what is not
//! known yet), `RECORD` floats per part and per obstacle, as `record` lays them out, and `sample` floats per sample as
//! `Player::sample` lays them out: the plots' and the trail's data since the last frame. A request that is not a
//! finite time from 0 to `MAX_FRAME_S` is refused: `frame` returns null and the error is the answer at `out_ptr`.

use sim_components::analysis::{
    SWEEP_POINTS, analyse_cycle_with, analyse_trajectory_with, default_sweep_amplitude, frequency_sweep, log_spaced,
    verdicts,
};
use sim_components::{Command, Sim, load_config};
use sim_core::scene::{Obstacle, Part, Scene, Shape, part_color};
use sim_core::{Config, MachineEntry, Translations};
use sim_machines::{LANGUAGES, MACHINES, find, translations};
use std::cell::RefCell;
use std::fmt::Write;

/// Floats in front of the parts in a frame.
pub const HEAD: usize = 8;
/// Floats per part in a frame.
pub const RECORD: usize = 20;
/// The mesh number of an obstacle: a cube drawn see-through.
pub const OBSTACLE_MESH: f32 = 2.0;
/// The most simulated time one `frame` runs, s: more is refused rather than left to hang the page.
pub const MAX_FRAME_S: f64 = 60.0;
/// Floats of a sample in front of its motors: time, the tool's x, y, z, and its distance from the plan.
pub const SAMPLE_HEAD: usize = 5;
/// Signals per motor in a sample, each as its lowest and highest value over the sample: torque, torque command,
/// speed, speed command, following error.
pub const MOTOR_SIGNALS: usize = 5;
/// How long a sample spans, s: the plots get every peak, as the lowest and highest value within it.
pub const SAMPLE_S: f64 = 1e-3;

/// The verdicts' labels are the app's: one translation source for every front end.
static CHECK_LABELS: &[(&str, &str)] =
    &[("en", include_str!("../../sim-app/i18n/en.ftl")), ("de", include_str!("../../sim-app/i18n/de.ftl"))];

/// The page's own texts; kind 7 hands them out.
static PAGE_TEXTS: &[(&str, &str)] = &[("en", include_str!("../i18n/en.ftl")), ("de", include_str!("../i18n/de.ftl"))];

/// The machine files shipped in `machines/`, built in so that the page needs no third file.
static SHIPPED: &[(&str, &str)] = &[
    ("corexy.toml", include_str!("../../../machines/corexy.toml")),
    ("h-bot.toml", include_str!("../../../machines/h-bot.toml")),
    ("planar-delta.toml", include_str!("../../../machines/planar-delta.toml")),
    ("rack-gantry.toml", include_str!("../../../machines/rack-gantry.toml")),
    ("screw-portal.toml", include_str!("../../../machines/screw-portal.toml")),
    ("single-axis.toml", include_str!("../../../machines/single-axis.toml")),
    ("t-gantry.toml", include_str!("../../../machines/t-gantry.toml")),
    ("xyz-portal.toml", include_str!("../../../machines/xyz-portal.toml")),
];

thread_local! {
    /// The machine files read so far, by the id the page uses for them.
    static FILES: RefCell<Vec<(String, &'static MachineEntry)>> = const { RefCell::new(Vec::new()) };
}

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
    let r = Request::parse(request);
    match kind {
        0 => Ok(list(language(r.id))),
        1 => Ok(format!("{{\"config\":{}}}", quote(machine(r.id, "en")?.default_config))),
        2 => size(&r),
        3 => play(&r),
        4 => upload(r.id, r.lang, r.config),
        5 => sweep(&r),
        6 => fit(&r),
        7 => Ok(texts(language(r.id), request.lines().skip(1))),
        8 => form(&r),
        9 => apply(&r),
        10 => scopes(&r),
        _ => Err(format!("unknown request kind: {kind}")),
    }
}

/// A request's lines: the machine, the language with options after it, the config, and after a line `---` the text
/// the options name (a path, a trace).
struct Request<'a> {
    id: &'a str,
    lang: &'static str,
    options: Vec<&'a str>,
    config: &'a str,
    extra: &'a str,
}

impl<'a> Request<'a> {
    fn parse(request: &'a str) -> Self {
        let mut lines = request.splitn(3, '\n');
        let (id, second, rest) =
            (lines.next().unwrap_or("").trim(), lines.next().unwrap_or(""), lines.next().unwrap_or(""));
        let mut words = second.split_whitespace();
        let lang = language(words.next().unwrap_or(""));
        let (config, extra) = match rest.strip_prefix("---\n").or(rest.strip_prefix("---\r\n")) {
            Some(extra) => ("", extra),
            None => rest.split_once("\n---\n").or(rest.split_once("\n---\r\n")).unwrap_or((rest, "")),
        };
        Request { id, lang, options: words.collect(), config, extra }
    }

    fn flag(&self, name: &str) -> bool {
        self.options.contains(&name)
    }

    /// The number of an option `name=value`; `default` without it.
    fn number(&self, name: &str, default: f64) -> Result<f64, String> {
        let prefix = format!("{name}=");
        match self.options.iter().find_map(|o| o.strip_prefix(prefix.as_str())) {
            None => Ok(default),
            Some(v) => v.parse::<f64>().ok().filter(|v| v.is_finite()).ok_or_else(|| format!("{name}={v}")),
        }
    }
}

/// A machine by the id the page uses: a registered one, a machine file of `machines/` (read the first time it is
/// asked for) or one the page handed in.
fn machine(id: &str, lang: &str) -> Result<&'static MachineEntry, String> {
    if let Some(entry) = find(id) {
        return Ok(entry);
    }
    if let Some(entry) = FILES.with(|f| f.borrow().iter().find(|(k, _)| k == id).map(|(_, e)| *e)) {
        return Ok(entry);
    }
    match id.strip_prefix("file:").and_then(|name| SHIPPED.iter().find(|(f, _)| *f == name)) {
        Some((name, text)) => read_file(id, name, lang, text),
        None => Err(format!("unknown machine: {id}")),
    }
}

/// A machine file read and checked as `sim_machines::load_file` checks a file (TOML, or JSON by the extension of
/// `name`), kept under `id`. An error names the file and the line (a JSON file: the key alone).
fn read_file(id: &str, name: &str, lang: &str, text: &str) -> Result<&'static MachineEntry, String> {
    let known: Vec<Config> = MACHINES.iter().map(|m| m.config()).collect();
    let json = name.to_ascii_lowercase().ends_with(".json");
    let stem = name.rsplit_once('.').map_or(name, |(stem, _)| stem);
    let tr = translations(lang, MACHINES[0]);
    let converted = if json {
        sim_machines::json_to_toml(text).map_err(|e| tr.tr_arg(e.key(), e.detail()))?
    } else {
        text.to_string()
    };
    let entry = sim_netlist::from_text(&converted, !json, stem, &known).map_err(|(e, line)| {
        let at = line.map_or(String::new(), |line| format!(":{line}"));
        tr.tr_arg(e.key(), &format!("{name}{at}: {}", e.detail()))
    })?;
    FILES.with(|f| {
        let mut files = f.borrow_mut();
        files.retain(|(k, _)| k != id);
        files.push((id.to_string(), entry));
    });
    Ok(entry)
}

/// A machine file the page hands in (kind 4), TOML or JSON by its file name's extension; one of the same name before is
/// replaced.
fn upload(name: &str, lang: &str, text: &str) -> Result<String, String> {
    let name = if name.is_empty() { "machine.toml" } else { name };
    let id = format!("upload:{name}");
    let entry = read_file(&id, name, lang, text)?;
    Ok(format!(
        "{{\"id\":{},\"name\":{},\"config\":{}}}",
        quote(&id),
        quote(translations(lang, entry).tr(entry.key)),
        quote(entry.default_config)
    ))
}

/// The machine, its translations with the checks' labels, and the config of the request, read and resolved.
fn configured(r: &Request) -> Result<(&'static MachineEntry, Translations, Config), String> {
    let entry = machine(r.id, r.lang)?;
    let mut tr = translations(r.lang, entry);
    CHECK_LABELS.iter().chain(PAGE_TEXTS).filter(|(l, _)| *l == r.lang).for_each(|(_, t)| tr.add(t));
    let refused = |e: sim_core::ConfigError| tr.tr_arg(e.key(), e.detail());
    let cfg: Config =
        if r.config.trim().is_empty() { entry.config() } else { load_config(entry, r.config).map_err(refused)? };
    let cfg = sim_components::catalog::resolve(entry, &cfg).map_err(refused)?;
    Ok((entry, tr, cfg))
}

/// The poses at times of the request's path (option `path`), planned from where `sim` stands, with what the planned
/// path is off the given one, mm, as `sim-cli` warns of it; `None` without the option.
fn planned_path(r: &Request, sim: &Sim, tr: &Translations) -> Result<Option<(Vec<(f64, Vec<f64>)>, f64)>, String> {
    if !r.flag("path") {
        return Ok(None);
    }
    let pieces = sim_components::path::parse(r.extra).map_err(|line| tr.tr_arg("web-path-unreadable", &line))?;
    if pieces.is_empty() {
        return Err(tr.tr("web-path-empty").to_string());
    }
    let optimal = r.flag("time-optimal");
    let refused = |e: sim_components::CommandError| tr.tr(e.key).to_string();
    let deviation = sim.path_deviation(&pieces, optimal).map_err(refused)?;
    Ok(Some((sim.plan_path(&pieces, optimal).map_err(refused)?, deviation * 1e3)))
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
    /// Base steps per sample, and how many of the sample being gathered have run.
    every: u64,
    gathered: u64,
    /// The sample being gathered: per motor and signal the lowest and highest value so far.
    range: Vec<f32>,
    /// The samples since the last frame, `sample_len` floats each.
    samples: Vec<f32>,
    /// The motors' angles and speeds, room kept from one step to the next.
    angle: Vec<f64>,
    speed: Vec<f64>,
}

thread_local! {
    static PLAYER: RefCell<Option<Player>> = const { RefCell::new(None) };
}

/// Floats per sample for a machine of `motors` motors.
pub fn sample_len(motors: usize) -> usize {
    SAMPLE_HEAD + 2 * MOTOR_SIGNALS * motors
}

/// Sets up the 3D view (kind 3): the machine at rest at its start pose, its cycle or its path commanded.
fn play(r: &Request) -> Result<String, String> {
    let (entry, tr, cfg) = configured(r)?;
    let mut sim = Sim::new(entry, &cfg).map_err(|e| tr.tr_arg(e.key(), e.detail()))?;
    let command = match planned_path(r, &sim, &tr)? {
        Some((points, _)) => Command::Trajectory(points),
        None => Command::StartCycle,
    };
    sim.command(command).map_err(|e| tr.tr(e.key).to_string())?;
    let motors = sim.machine.motors().to_vec();
    let mut player = Player {
        passes: sim.control.cycle_passes(),
        every: ((SAMPLE_S / sim.dt).round() as u64).max(1),
        gathered: 0,
        range: Vec::new(),
        samples: Vec::new(),
        angle: vec![0.0; motors.len()],
        speed: vec![0.0; motors.len()],
        sim,
        scene: Scene::default(),
        frame: Vec::new(),
        pass_began: None,
        cycle_s: f64::NAN,
    };
    player.advance(0.0);
    let drawn = (player.frame[5] + player.frame[6]) as usize * RECORD;
    let (lo, hi) = bounds(&player.frame[HEAD..HEAD + drawn]);
    let centre: [f32; 3] = std::array::from_fn(|i| (lo[i] + hi[i]) / 2.0);
    let radius = (0..3).map(|i| (hi[i] - lo[i]).powi(2)).sum::<f32>().sqrt() / 2.0;
    let names: Vec<String> =
        motors.iter().map(|m| format!("{{\"id\":{},\"name\":{}}}", quote(m.id), quote(tr.tr(m.key)))).collect();
    let json = format!(
        "{{\"dt_s\":{},\"centre\":[{},{},{}],\"radius\":{},\"motors\":[{}],\"sample\":{}}}",
        number(player.sim.dt),
        number(centre[0].into()),
        number(centre[1].into()),
        number(centre[2].into()),
        number(radius.into()),
        names.join(","),
        sample_len(motors.len())
    );
    PLAYER.with(|p| *p.borrow_mut() = Some(player));
    Ok(json)
}

impl Player {
    /// Runs on by `seconds` (whole base steps) and writes the frame.
    fn advance(&mut self, seconds: f64) {
        self.samples.clear();
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
            self.sample();
        }
        self.scene = Scene::default();
        self.sim.scene(&mut self.scene);
        let tool = self.scene.tool.map_or([f32::NAN; 3], |p| p.map(|v| v as f32));
        self.frame.clear();
        let samples = self.samples.len() / sample_len(self.speed.len());
        let (parts, obstacles) = (self.scene.parts.len(), self.scene.obstacles.len());
        let head = [self.sim.time(), self.cycle_s, tool[0].into(), tool[1].into(), tool[2].into()];
        self.frame.extend(head.map(|v| v as f32));
        self.frame.extend([parts, obstacles, samples].map(|n| n as f32));
        self.scene.parts.iter().for_each(|part| record(part, &mut self.frame));
        self.scene.obstacles.iter().for_each(|o| record_obstacle(o, &mut self.frame));
        self.frame.extend_from_slice(&self.samples);
    }

    /// Takes this base step's motor signals into the sample being gathered, and closes the sample after `every`
    /// steps: `SAMPLE_HEAD` floats (time, the tool's x, y, z in the scene, its distance from where the plan has it then,
    /// m; NaN where the machine has no mapping), then per motor the lowest and highest torque, torque command (N m),
    /// speed, speed command (rad/s) and following error (rad) over the sample.
    fn sample(&mut self) {
        let sim = &self.sim;
        sim.machine.motor_state(&sim.state, &mut self.angle, &mut self.speed);
        if self.gathered == 0 {
            self.range.clear();
            self.range.resize(2 * MOTOR_SIGNALS * self.speed.len(), f32::NAN);
        }
        for (k, d) in sim.control.drives.iter().enumerate() {
            let values = [d.torque, d.torque_command, self.speed[k], d.speed_command, d.following_error];
            for (i, v) in values.into_iter().enumerate() {
                let at = 2 * (MOTOR_SIGNALS * k + i);
                // NaN loses to any number in `min` and `max`: the first value starts the range.
                self.range[at] = self.range[at].min(v as f32);
                self.range[at + 1] = self.range[at + 1].max(v as f32);
            }
        }
        self.gathered += 1;
        if self.gathered < self.every {
            return;
        }
        self.gathered = 0;
        let tool = sim.tool_point();
        let planned = sim.machine.scene_point(&sim.control.set_pos);
        let gap = match (tool, planned) {
            (Some(t), Some(p)) => (0..3).map(|i| (t[i] - p[i]).powi(2)).sum::<f64>().sqrt(),
            _ => f64::NAN,
        };
        let tool = tool.unwrap_or([f64::NAN; 3]);
        self.samples.extend([sim.time(), tool[0], tool[1], tool[2], gap].map(|v| v as f32));
        self.samples.extend_from_slice(&self.range);
    }
}

/// An obstacle as the page draws it, `RECORD` floats as `record` lays them out: mesh `OBSTACLE_MESH` (the unit cube,
/// drawn see-through), stretched to the obstacle's box, grey, or red while a part of the machine touches it.
pub fn record_obstacle(obstacle: &Obstacle, out: &mut Vec<f32>) {
    let (c, s) = (obstacle.aabb.centre, obstacle.aabb.size.map(|v| v.abs().max(1e-6)));
    out.push(OBSTACLE_MESH);
    out.extend([s[0], 0.0, 0.0, 0.0, 0.0, s[1], 0.0, 0.0, 0.0, 0.0, s[2], 0.0].map(|v| v as f32));
    out.extend([c[0] as f32, c[1] as f32, c[2] as f32, 1.0]);
    out.extend(if obstacle.contact { [0.9, 0.15, 0.1] } else { [0.55, 0.57, 0.62] });
}

/// One part as the page draws it, `RECORD` floats: the mesh (0 a cube of side 1, 1 a cylinder of diameter and length
/// 1 along y, both centred on the origin; turned as `Part::flat` says), the matrix that takes the mesh to the part in scene coordinates (4 × 4,
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
            let norm = along.iter().map(|v| v * v).sum::<f64>().sqrt();
            let length = size(norm);
            // A segment of no length (an arm's last link where the flange sits at its last joint) has no direction: it
            // lies along x, so that its width and thickness keep theirs and the matrix stays invertible.
            let u = if norm > 0.0 { along.map(|v| v / norm) } else { [1.0, 0.0, 0.0] };
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
    // A segment is never flat (`Part::flat`), so this turns only boxes and cylinders.
    let columns = columns.map(|c| part.oriented(c));
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
/// kind 3 has set one up, and null with the error as the answer at `out_ptr` for a time that is not finite, below 0
/// or over `MAX_FRAME_S` (an infinite one ran the steps' count to the largest integer and hung the page).
#[unsafe(no_mangle)]
pub extern "C" fn frame(seconds: f64) -> *const f32 {
    if !(0.0..=MAX_FRAME_S).contains(&seconds) {
        let error = format!("{{\"error\":{}}}", quote(&format!("frame: {seconds} s is not from 0 to {MAX_FRAME_S} s")));
        OUT.with(|o| *o.borrow_mut() = error.into_bytes());
        return std::ptr::null();
    }
    PLAYER.with(|p| {
        p.borrow_mut().as_mut().map_or(std::ptr::null(), |player| {
            player.advance(seconds);
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

/// The registered machines, then the machine files of `machines/` and those the page handed in (`"file": true`).
fn list(lang: &str) -> String {
    let mut rows: Vec<String> = MACHINES
        .iter()
        .map(|m| {
            format!("{{\"id\":{},\"name\":{},\"file\":false}}", quote(m.id), quote(translations(lang, m).tr(m.key)))
        })
        .collect();
    let shipped = SHIPPED.iter().map(|(name, _)| (format!("file:{name}"), name.to_string()));
    let uploaded = FILES.with(|f| {
        let files = f.borrow();
        let ids: Vec<String> = files.iter().map(|(id, _)| id.clone()).filter(|id| id.starts_with("upload:")).collect();
        ids.into_iter().map(|id| (id.clone(), id["upload:".len()..].to_string())).collect::<Vec<_>>()
    });
    for (id, name) in shipped.chain(uploaded) {
        rows.push(format!("{{\"id\":{},\"name\":{},\"file\":true}}", quote(&id), quote(&name)));
    }
    format!("[{}]", rows.join(","))
}

/// The page's texts (`i18n/*.ftl` of this crate) in `lang`, and those of `keys` from the simulator's translations
/// and the app's (axis, quantity and check names), as one JSON object.
fn texts<'a>(lang: &str, keys: impl Iterator<Item = &'a str>) -> String {
    let mut tr = translations(lang, MACHINES[0]);
    CHECK_LABELS.iter().chain(PAGE_TEXTS).filter(|(l, _)| *l == lang).for_each(|(_, t)| tr.add(t));
    let own = PAGE_TEXTS[0].1.lines().filter_map(|l| l.split_once(" = ").map(|(k, _)| k.trim()));
    let keys: Vec<&str> =
        own.filter(|k| !k.starts_with('#')).chain(keys.map(str::trim).filter(|k| !k.is_empty())).collect();
    let rows: Vec<String> = keys.iter().map(|k| format!("{}:{}", quote(k), quote(tr.tr(k)))).collect();
    format!("{{{}}}", rows.join(","))
}

/// The cycle of the config, one pass measured by `analyse_cycle`, or the path of the request run through by
/// `analyse_trajectory`, and judged as `sim-cli verdicts` judges it; per motor every column of `sim-cli analyze`, with
/// the operating points and the torque-speed curve. The column names are fixed: a motor's `torque_unit` and
/// `speed_unit` say what its `*_torque_nm` and `peak_speed_rad_s` hold (N m and rad/s, on a linear motor N and m/s).
fn size(r: &Request) -> Result<String, String> {
    let (entry, tr, cfg) = configured(r)?;
    let refused = |e: sim_core::ConfigError| tr.tr_arg(e.key(), e.detail());
    let mut sim = Sim::new(entry, &cfg).map_err(refused)?;
    let motors = sim.machine.motors().to_vec();
    let infos = sim.signal_infos();
    let unit_of = |motor: &str, signal: &str, rotary: &str| {
        let id = format!("motor.{motor}.{signal}");
        infos.iter().find(|s| s.id == id).map_or(rotary, |s| s.unit.as_str()).to_string()
    };
    let mut warnings: Vec<String> = sim
        .limit_violations
        .iter()
        .map(|v| tr.tr_arg(v.key, &format!("{}: {:.4} > {:.4}", tr.tr(motors[v.motor].key), v.needed, v.allowed)))
        .collect();
    let path = planned_path(r, &sim, &tr)?;
    let load = match path {
        Some((points, deviation)) => {
            // As `sim-cli`: not below what the figure can show.
            let shown = format!("{deviation:.4}");
            if shown != "0.0000" {
                warnings.push(tr.tr_arg("path-deviation", &shown));
            }
            analyse_trajectory_with(&mut sim, points, false)
        }
        None => analyse_cycle_with(&mut sim, false),
    }
    .map_err(|e| tr.tr(e.key).to_string())?;
    let judged = verdicts(&cfg, &load).map_err(refused)?;
    let or_null = |v: Option<f64>| v.map_or("null".to_string(), number);
    let belt = load.belt_tension;
    let machine_wide = [
        ("belt_tension_min_n", belt.map(|b| b.0)),
        ("belt_tension_max_n", belt.map(|b| b.1)),
        ("path_error_peak_m", Some(load.path_error_peak)),
        ("path_error_rms_m", Some(load.path_error_rms)),
        ("tracking_error_peak_m", Some(load.tracking_error_peak)),
        ("dwell_error_peak_m", Some(load.dwell_error_peak)),
        ("belt_slack_time_s", load.belt_slack_time),
        ("radial_load_peak_n", load.radial_load_peak),
        ("guide_load_peak_n", load.guide_load.map(|g| g.0)),
        ("guide_life_km", sim_components::analysis::guide_life_km(&cfg, &load)),
        ("guide_life_h", sim_components::analysis::guide_life_h(&cfg, &load)),
        // effects.thermal_warm_start: the largest change of a rise over the measured pass, K; null from cold.
        ("warm_start_change_k", load.warm_start.map(|w| w.change)),
    ];
    let machine_wide: Vec<String> = machine_wide.iter().map(|(k, v)| format!("{}:{}", quote(k), or_null(*v))).collect();
    let warnings: Vec<String> = warnings.iter().map(|w| quote(w)).collect();
    let mut json = format!(
        "{{\"machine\":{},\"cycle_s\":{},\"tripped\":{},\"warnings\":[{}],{},\"motors\":[",
        quote(entry.id),
        number(load.duration),
        load.tripped,
        warnings.join(","),
        machine_wide.join(",")
    );
    for (k, (motor, m)) in motors.iter().zip(&load.motors).enumerate() {
        let columns = [
            ("peak_torque_nm", Some(m.peak_torque)),
            ("rms_torque_nm", Some(m.rms_torque)),
            ("peak_speed_rad_s", Some(m.peak_speed)),
            ("utilisation", Some(m.utilisation)),
            ("thermal_utilisation", Some(m.thermal_utilisation)),
            ("rms_current_a", Some(m.rms_current)),
            ("peak_current_a", Some(m.peak_current)),
            ("winding_rise_k", Some(m.winding_rise)),
            ("housing_rise_k", Some(m.housing_rise)),
            ("drive_overload", Some(m.drive_overload)),
            ("energy_in_j", Some(m.energy_in)),
            ("energy_regenerated_j", Some(m.energy_regenerated)),
            ("peak_regenerated_power_w", Some(m.peak_regenerated_power)),
            ("largest_regeneration_j", Some(m.largest_regeneration)),
            ("cubic_mean_torque_nm", Some(m.cubic_mean_torque)),
            ("braking_resistor_energy_j", Some(m.resistor_energy)),
            ("braking_resistor_mean_power_w", Some(m.resistor_energy / load.duration)),
            ("dc_link_peak_voltage_v", Some(m.peak_dc_link_voltage)),
            ("mains_energy_j", Some(m.mains_energy)),
            ("pulley_bearing_life_h", sim_components::analysis::pulley_bearing_life_h(&cfg, m)),
            ("gearbox_life_h", sim_components::analysis::gearbox_life_h(&cfg, m)),
            ("belt_tooth_utilisation", sim_components::analysis::belt_tooth_utilisation(&cfg, m).map_err(refused)?),
            ("gearbox_gearing_life_h", sim_components::analysis::gearbox_gearing_life_h(&cfg, m)),
            ("motor_bearing_life_h", sim_components::analysis::motor_bearing_life_h(&cfg, m)),
        ];
        let _ = write!(
            json,
            "{}{{\"id\":{},\"name\":{},\"torque_unit\":{},\"speed_unit\":{}",
            if k == 0 { "" } else { "," },
            quote(motor.id),
            quote(tr.tr(motor.key)),
            quote(&unit_of(motor.id, "torque", "N m")),
            quote(&unit_of(motor.id, "speed", "rad/s"))
        );
        for (key, value) in columns {
            let _ = write!(json, ",{}:{}", quote(key), or_null(value));
        }
        let pair = |(s, t): (f64, f64)| format!("[{},{}]", number(s), number(t));
        let points: Vec<String> = m.points.iter().copied().map(pair).collect();
        let (curve, s1) = torque_speed_curve(sim.control.drive_params_of(k));
        let (curve, s1): (Vec<String>, Vec<String>) =
            (curve.into_iter().map(pair).collect(), s1.into_iter().map(pair).collect());
        let _ = write!(
            json,
            ",\"points\":[{}],\"curve\":[{}],\"s1\":[{}]}}",
            points.join(","),
            curve.join(","),
            s1.join(",")
        );
    }
    json.push_str("],\"verdicts\":[");
    for (i, v) in judged.iter().enumerate() {
        let _ =
            write!(
                json,
                "{}{{\"check\":{},\"label\":{},\"motor\":{},\"value\":{},\"limit\":{},\"margin\":{},\"ok\":{}}}",
                if i == 0 { "" } else { "," },
                quote(v.check),
                quote(tr.tr(&sim_components::report::check_key(
                    v.check,
                    v.motor.is_some_and(|k| sim.machine.motor_linear(k))
                ))),
                v.motor.map_or("null".into(), |k| quote(motors[k].id)),
                number(v.value),
                number(v.limit),
                // As the sizing report has it: the share of the limit left, 1 - value / limit (limit / value for a check
                // that must stay above its limit); null where there is none (0 of an at-least check).
                number(1.0 - sim_components::analysis::utilisation(v)),
                v.ok
            );
    }
    json.push_str("]}");
    Ok(json)
}

/// The torque-speed curve of a motor's drive (`DriveParams::torque_curve`) as an outline of (speed rad/s, torque N m):
/// the highest torque from the no-load speed backwards to the no-load speed forwards, then the lowest torque back,
/// at the curve's corners and 40 speeds between (a field-weakened curve bends). And the continuous (S1) rating as the
/// app and the motor-s1 verdict have it (`charts::torque_speed`): its line for positive torque, then the one for
/// negative torque back, closed.
pub fn torque_speed_curve(drive: &sim_components::drive::DriveParams) -> (Vec<(f64, f64)>, Vec<(f64, f64)>) {
    let (top, rated) = (drive.no_load_speed, drive.rated_speed);
    let mut speeds: Vec<f64> = (0..=40).map(|i| top * (i as f64 / 20.0 - 1.0)).chain([-rated, 0.0, rated]).collect();
    speeds.sort_by(f64::total_cmp);
    speeds.dedup();
    let high = speeds.iter().map(|&s| (s, drive.torque_curve(s).1));
    let curve = high.chain(speeds.iter().rev().map(|&s| (s, drive.torque_curve(s).0))).collect();
    let [high, low] = sim_components::charts::torque_speed(drive, drive.bus_voltage).s1;
    let s1: Vec<(f64, f64)> = high.iter().chain(low.iter().rev()).chain(high.first()).map(|p| (p[0], p[1])).collect();
    (curve, s1)
}

/// The frequency response from one motor's torque to its speed (kind 5), as `sim-cli sweep` gives it. On a linear
/// motor `amplitude_nm` is a force in N and the gain is in (m/s)/N.
fn sweep(r: &Request) -> Result<String, String> {
    let (entry, tr, cfg) = configured(r)?;
    let refused = |e: sim_core::ConfigError| tr.tr_arg(e.key(), e.detail());
    let motor = r.number("motor", 0.0)?;
    let count = r.number("count", SWEEP_POINTS as f64)?;
    if motor < 0.0 || motor.fract() != 0.0 || !(2.0..=1000.0).contains(&count) {
        return Err(format!("motor={motor} count={count}"));
    }
    let frequencies = log_spaced(r.number("from", 5.0)?, r.number("to", 500.0)?, count as usize);
    let amplitude = r.number("amplitude", default_sweep_amplitude(entry, &cfg).map_err(refused)?)?;
    let points = frequency_sweep(entry, &cfg, motor as usize, &frequencies, amplitude).map_err(refused)?;
    let rows: Vec<String> = points
        .iter()
        .map(|p| format!("[{},{},{}]", number(p.frequency), number(p.gain), number(p.phase_deg)))
        .collect();
    Ok(format!("{{\"motor\":{motor},\"amplitude_nm\":{},\"points\":[{}]}}", number(amplitude), rows.join(",")))
}

/// What the fit (kind 6) is given after the request's `---`: the trace, then a mapping file for a vendor export, then a
/// measured sweep, split by lines of `%%%`; an empty part is none.
const FIT_PARTS: &str = "\n%%%\n";

/// Masses and friction fitted to a measured trace (kind 6), as `sim-cli fit` fits them: the trace in the simulator's own
/// columns, or a vendor export through its mapping file (`import`); per axis on a machine whose axes are its motors'
/// (`fit_rigid`), else per joint (`fit_joints`). With the option `stiffness` also the factor on the stiffness at the key
/// `stiffness-key=` (the belts' by default) from the trace's own moves (`fit_move_stiffness`); with a sweep (options
/// `motor=`, `amplitude=` as kind 5) the load's resonance (`fit_sweep`). Rows of quantity, of, config, fitted and
/// standard error, and the advice translated.
fn fit(r: &Request) -> Result<String, String> {
    use sim_components::identify::{
        BELT_STIFFNESS_KEY, fit_joints, fit_move_stiffness, fit_rigid, fit_sweep, parse_sweep,
    };
    let (entry, tr, cfg) = configured(r)?;
    let refused = |e: sim_components::trace::TraceError| tr.tr_arg(e.key, &e.detail);
    let machine = (entry.build)(&cfg).map_err(|e| tr.tr_arg(e.key(), e.detail()))?;
    let parts: Vec<&str> = r.extra.split(FIT_PARTS).map(str::trim).collect();
    let part = |i: usize| parts.get(i).copied().filter(|p| !p.is_empty());
    if part(0).is_none() && part(2).is_none() {
        return Err(tr.tr("web-fit-nothing").to_string());
    }
    let (mut rows, mut advice, mut units) = (Vec::new(), Vec::new(), Vec::new());
    let row = |q: &str, of: &str, config: f64, fitted: f64, error: f64| {
        format!("[{},{},{},{},{}]", quote(q), quote(of), number(config), number(fitted), number(error))
    };
    let trace = match part(0) {
        None => None,
        Some(text) => Some(match part(1) {
            Some(map) => {
                use sim_components::import::{Mapping, import};
                let imported = import(text, &Mapping::parse(map).map_err(refused)?).map_err(refused)?;
                let motors = machine.motors().len();
                if motors != 0 && imported.trace.motors.len() != motors {
                    let detail = format!("{} [[motor]] for {motors} motors", imported.trace.motors.len());
                    return Err(tr.tr_arg("error-import-map", &detail));
                }
                advice.extend(imported.notes.iter().map(|(key, detail)| tr.tr_arg(key, detail)));
                imported.trace
            }
            None => sim_components::trace::Trace::parse(text, machine.motors()).map_err(refused)?,
        }),
    };
    if let Some(trace) = &trace {
        if sim_components::trace::per_axis_fit(machine.as_ref()) {
            for (axis, fit) in machine.axes().iter().zip(fit_rigid(entry, &cfg, trace).map_err(refused)?) {
                let Some(fit) = fit else {
                    advice.push(tr.tr_arg("fit-axis-skipped", axis.id));
                    continue;
                };
                let names = ["moved_mass_kg", "viscous_friction_n_s_m", "coulomb_friction_n", "constant_force_n"];
                for (q, name) in names.iter().enumerate() {
                    rows.push(row(name, axis.id, fit.config[q], fit.fitted[q], fit.error[q]));
                    units.push(quote(fit.units[q]));
                }
                advice.extend(fit.advice(axis.id).iter().map(|(key, detail)| tr.tr_arg(key, detail)));
            }
        } else {
            match fit_joints(entry, &cfg, trace).map_err(refused)? {
                Some(fit) => {
                    rows.extend(fit.values.iter().map(|v| row(v.quantity, &v.of, v.config, v.fitted, v.error)));
                    units.extend(fit.values.iter().map(|_| quote("")));
                    advice.extend(fit.advice().iter().map(|(key, detail)| tr.tr_arg(key, detail)));
                }
                None => advice.push(tr.tr("fit-joints-skipped").to_string()),
            }
        }
        if r.flag("stiffness") {
            let key = r.options.iter().find_map(|o| o.strip_prefix("stiffness-key=")).unwrap_or(BELT_STIFFNESS_KEY);
            let fit = fit_move_stiffness(entry, &cfg, trace, key).map_err(refused)?;
            rows.push(row("stiffness_factor", key, 1.0, fit.factor, fit.error));
            rows.push(row("ringing_rms_nm", key, f64::NAN, fit.ringing, f64::NAN));
            rows.push(row("ringing_left_rms_nm", key, f64::NAN, fit.residual, f64::NAN));
            units.extend(["", "", ""].map(quote));
            let (key, detail) = fit.advice();
            advice.push(tr.tr_arg(key, &detail));
        }
    }
    if let Some(text) = part(2) {
        let measured = parse_sweep(text).map_err(refused)?;
        let motor = r.number("motor", 0.0)?;
        if motor < 0.0 || motor.fract() != 0.0 || motor as usize >= machine.motors().len() {
            return Err(format!("motor={motor}"));
        }
        let from_config = |e: sim_core::ConfigError| tr.tr_arg(e.key(), e.detail());
        let amplitude = r.number("amplitude", default_sweep_amplitude(entry, &cfg).map_err(from_config)?)?;
        let fit = fit_sweep(entry, &cfg, motor as usize, &measured, amplitude)
            .map_err(from_config)?
            .ok_or_else(|| tr.tr("fit-no-resonance").to_string())?;
        let id = machine.motors()[motor as usize].id;
        rows.push(row("resonance_hz", id, fit.config.frequency, fit.fitted.frequency, fit.fitted.frequency_error));
        rows.push(row("resonance_damping", id, fit.config.damping, fit.fitted.damping, f64::NAN));
        units.extend(["Hz", ""].map(quote));
        let (key, detail) = fit.advice();
        advice.push(tr.tr_arg(key, &detail));
    }
    let advice: Vec<String> = advice.iter().map(|a| quote(a)).collect();
    // `units`: the unit of each row's values, in the order of `rows` (empty for a joint value, which has none of its own).
    Ok(format!("{{\"rows\":[{}],\"advice\":[{}],\"units\":[{}]}}", rows.join(","), advice.join(","), units.join(",")))
}

/// The values of the page's form (kind 8): every number of the config but the effect switches and the bounds, with
/// its label (`param-<key>` and `group-<table>`, as the app's parameter panel), and the `[parts]` names with what the
/// catalogue offers for each: the supplies, the motors of the config's supply, the gearboxes that fit its motor, the
/// drives of its supply and the belt axes. Booleans, arrays and the other names are edited in the TOML.
fn form(r: &Request) -> Result<String, String> {
    use sim_components::catalog::{self, PART_KEYS, SUPPLIES};
    let entry = machine(r.id, r.lang)?;
    let tr = translations(r.lang, entry);
    let refused = |e: sim_core::ConfigError| tr.tr_arg(e.key(), e.detail());
    let cfg =
        if r.config.trim().is_empty() { entry.config() } else { load_config(entry, r.config).map_err(refused)? };
    let mut params = Vec::new();
    for (path, value) in cfg.leaves() {
        let Some((table, leaf)) = path.rsplit_once('.') else { continue };
        let skipped = table == "effects" || table == "bounds" || table.starts_with("bounds.");
        if skipped || !(value.is_float() || value.is_integer()) {
            continue;
        }
        params.push(format!(
            "{{\"path\":{},\"group\":{},\"label\":{},\"value\":{}}}",
            quote(&path),
            quote(tr.tr(&format!("group-{}", table.replace('.', "-")))),
            quote(tr.tr(&format!("param-{leaf}"))),
            quote(&value.to_string())
        ));
    }
    let named = |key: &str| cfg.get(&format!("parts.{key}")).ok().and_then(|v| v.as_str()).unwrap_or("").to_string();
    let (supply, motor) = (named("motor_supply"), named("motor"));
    let mut parts = Vec::new();
    for key in PART_KEYS {
        if cfg.get(&format!("parts.{key}")).is_err() {
            continue;
        }
        let options: Vec<String> = match key {
            "motor_supply" => SUPPLIES.iter().map(|s| s.to_string()).collect(),
            "motor" => catalog::motor_articles_for(&supply).into_iter().map(String::from).collect(),
            "gearbox" if !motor.is_empty() => catalog::gearboxes_for_motor(&motor, ""),
            "drive" => catalog::drives_for(&supply).into_iter().map(|d| d.0.to_string()).collect(),
            "axis" => catalog::axis_names(),
            _ => Vec::new(),
        };
        let options: Vec<String> = options.iter().map(|o| quote(o)).collect();
        parts.push(format!(
            "{{\"key\":{},\"value\":{},\"options\":[{}]}}",
            quote(key),
            quote(&named(key)),
            options.join(",")
        ));
    }
    Ok(format!("{{\"params\":[{}],\"parts\":[{}]}}", params.join(","), parts.join(",")))
}

/// The page's changes set in its config (kind 9): each `path = value` line after `---` (TOML syntax; an unknown key or
/// another type refused, as `Config::set_text` refuses them) written over the value's own text, so that the comments
/// and the layout stay. Answers the text, every value that differs from the machine's default config as such a line
/// (what a share link carries), and whether those lines on the default config give this config back (`compact`: a
/// changed array, such as a cycle segment, is not among them, so the link then carries the whole text).
fn apply(r: &Request) -> Result<String, String> {
    let entry = machine(r.id, r.lang)?;
    let tr = translations(r.lang, entry);
    let refused = |e: sim_core::ConfigError| tr.tr_arg(e.key(), e.detail());
    let text = if r.config.trim().is_empty() { entry.default_config } else { r.config };
    let mut cfg = Config::parse(text).map_err(refused)?;
    for line in r.extra.lines().filter(|l| !l.trim().is_empty()) {
        cfg.set_text(line).map_err(refused)?;
    }
    let text = edited(text, &cfg).map_err(refused)?;
    let default = entry.config();
    let changed: Vec<String> = cfg
        .leaves()
        .into_iter()
        .filter(|(path, value)| default.get(path).ok() != Some(value))
        .map(|(path, value)| format!("{path} = {value}"))
        .collect();
    let mut rebuilt = default;
    let compact = changed.iter().all(|line| rebuilt.set_text(line).is_ok()) && rebuilt == cfg;
    let changed: Vec<String> = changed.iter().map(|c| quote(c)).collect();
    Ok(format!("{{\"config\":{},\"changed\":[{}],\"compact\":{compact}}}", quote(&text), changed.join(",")))
}

/// `text` with every value that `cfg` has changed written in place, so that its comments and layout stay; where that
/// does not give `cfg` back (an array was changed, a value is not where the text has it), `cfg` written plainly.
/// The app saves a config over its file the same way (`sim-app` `saved_text`).
fn edited(text: &str, cfg: &Config) -> Result<String, sim_core::ConfigError> {
    use toml::Value;
    let plain = cfg.to_text()?;
    let (Ok(doc), Ok(old)) = (toml_edit::Document::parse(text), Config::parse(text)) else { return Ok(plain) };
    let mut edits = Vec::new();
    for (path, value) in cfg.leaves() {
        if old.get(&path).ok() == Some(&value) {
            continue;
        }
        let new = match value {
            Value::Float(x) => toml_edit::Value::from(x),
            Value::Integer(n) => toml_edit::Value::from(n),
            Value::Boolean(on) => toml_edit::Value::from(on),
            Value::String(name) => toml_edit::Value::from(name),
            _ => return Ok(plain),
        };
        // The span is the value's own text: the comment behind it stays.
        let span = path
            .split('.')
            .try_fold(doc.as_item(), |item, key| item.get(key))
            .and_then(toml_edit::Item::as_value)
            .and_then(toml_edit::Value::span);
        let Some(span) = span else { return Ok(plain) };
        edits.push((span, new.to_string()));
    }
    edits.sort_by_key(|(span, _)| std::cmp::Reverse(span.start));
    let mut kept = text.to_string();
    for (span, new) in edits {
        kept.replace_range(span, new.trim());
    }
    Ok(if Config::parse(&kept).as_ref() == Ok(cfg) { kept } else { plain })
}

/// The signals of the scopes (kind 10), per motor, by the end of their id `motor.<id>.<signal>`.
pub const SCOPE_SIGNALS: [&str; 6] = ["angle", "speed", "torque", "current", "following_error", "winding_temperature"];
/// The most rows a scope answer has: one a millisecond up to a pass of 30 s.
pub const SCOPE_ROWS: usize = 30_000;

/// A path's run (`Command::Trajectory`, from rest) with 0.3 s of rest after its end as the sizing check gives it, every
/// signal once a millisecond, the time counted from the start: the rows `record_cycle` gives for the cycle.
fn record_path(sim: &mut Sim, points: Vec<(f64, Vec<f64>)>) -> Result<Vec<Vec<f64>>, sim_components::CommandError> {
    let end = points.last().map_or(0.0, |p| p.0) + 0.3;
    sim.command(Command::Trajectory(points))?;
    let every = ((1e-3 / sim.dt).round() as u64).max(1);
    let (first, start) = (sim.step, sim.time());
    let (mut rows, mut sample) = (Vec::new(), Vec::new());
    // A tripped drive never finishes the path.
    while sim.time() - start < end && sim.control.enabled() {
        sim.step();
        if (sim.step - first).is_multiple_of(every) {
            sim.sample(&mut sample);
            sample[0] = (sim.step - first) as f64 * sim.dt;
            rows.push(sample.clone());
        }
    }
    Ok(rows)
}

/// One pass of the configured cycle, once a millisecond, as `sim-cli compare` records it (`record_cycle`: the second
/// pass, the time from its start), or with the option `path` the run of the request's path (`record_path`): per motor
/// its angle, speed, torque, current, following error and winding temperature rise. A longer pass keeps every n-th row,
/// `SCOPE_ROWS` at most. Numbers to 6 significant digits.
fn scopes(r: &Request) -> Result<String, String> {
    let (entry, tr, cfg) = configured(r)?;
    let refused = |e: sim_core::ConfigError| tr.tr_arg(e.key(), e.detail());
    let mut sim = Sim::new(entry, &cfg).map_err(refused)?;
    let signals = sim.signal_infos();
    let motors = sim.machine.motors().to_vec();
    let rows = match planned_path(r, &sim, &tr)? {
        Some((points, _)) => record_path(&mut sim, points).map_err(|e| tr.tr(e.key).to_string())?,
        None => sim_components::analysis::record_cycle(entry, &cfg).map_err(refused)?,
    };
    let every = rows.len().div_ceil(SCOPE_ROWS).max(1);
    let rows: Vec<&Vec<f64>> = rows.iter().step_by(every).collect();
    let column = |id: String| signals.iter().position(|s| s.id == id);
    let series = |c: Option<usize>| -> String {
        let values: Vec<String> = rows.iter().map(|row| c.map_or("null".into(), |c| short(row[c]))).collect();
        format!("[{}]", values.join(","))
    };
    // The label is the first motor's; each unit is listed per motor (a rotary and a linear motor may share a machine).
    let head = SCOPE_SIGNALS.map(|name| {
        let info = motors.first().and_then(|m| column(format!("motor.{}.{name}", m.id))).map(|c| &signals[c]);
        let (key, unit) = info.map_or((name.to_string(), ""), |s| (s.quantity_key.clone(), s.unit.as_str()));
        let units: Vec<String> = motors
            .iter()
            .map(|m| quote(column(format!("motor.{}.{name}", m.id)).map_or("", |c| signals[c].unit.as_str())))
            .collect();
        format!(
            "{{\"id\":{},\"label\":{},\"unit\":{},\"units\":[{}]}}",
            quote(name),
            quote(tr.tr(&key)),
            quote(unit),
            units.join(",")
        )
    });
    let mut json = format!(
        "{{\"step_s\":{},\"time\":{},\"signals\":[{}],\"motors\":[",
        number(1e-3 * every as f64),
        series(Some(0)),
        head.join(",")
    );
    for (k, m) in motors.iter().enumerate() {
        let data = SCOPE_SIGNALS.map(|name| series(column(format!("motor.{}.{name}", m.id))));
        let _ = write!(
            json,
            "{}{{\"id\":{},\"name\":{},\"data\":[{}]}}",
            if k == 0 { "" } else { "," },
            quote(m.id),
            quote(tr.tr(m.key)),
            data.join(",")
        );
    }
    json.push_str("]}");
    Ok(json)
}

/// A number to 6 significant digits, `null` where it is not finite: the scopes' rows, a third the size of `number`'s.
fn short(v: f64) -> String {
    if v.is_finite() { format!("{v:.5e}") } else { "null".into() }
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
