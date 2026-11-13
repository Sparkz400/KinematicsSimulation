//! The panels around the 3D view: a bar on top with the machine, the run controls and how close the analysed
//! cycle runs each part to its limit; the setup on the left (motion, effects, parameters, config file); the tools on
//! the right (analysis and the other computations, with the check picked in the top bar); the scopes below; and the
//! view's own controls over the view. Everything shown comes from the model and the translation files; no text for
//! the user is written here.

use bevy_egui::egui::{
    self, Align, Checkbox, Color32, ComboBox, CornerRadius, DragValue, FontData, FontDefinitions, FontFamily, FontId, Frame, Grid, Label, Layout, Margin, Rect, RichText,
    ScrollArea, Sense, Slider, Stroke, TextEdit, TextStyle, TextWrapMode, Ui, vec2,
};
use egui_plot::{GridMark, HLine, Legend, Line, LineStyle, Plot, Points, VLine, log_grid_spacer};
use sim_app::model::{Analysis, Inputs, Model, SUPPLIES, Table, display_scale, heat_color, part_color};
use sim_components::Command;
use sim_components::analysis::{AT_LEAST, Verdict, utilisation};
use sim_core::scene::MOTOR_PART;
use sim_core::{MachineEntry, Scene};
use sim_machines::{LANGUAGES, MACHINES};
use std::sync::Arc;
use toml::Value;

// The palette. Each colour stands for one thing, the same in the panels as in the 3D view: amber what is planned,
// cyan what is measured or travelled, red what is over its limit. Green only marks a check that passes.
pub const GRAPHITE: Color32 = Color32::from_rgb(36, 40, 45);
pub const MACHINED: Color32 = Color32::from_rgb(59, 65, 72);
pub const INSET: Color32 = Color32::from_rgb(28, 31, 35);
pub const CHALK: Color32 = Color32::from_rgb(214, 218, 223);
pub const DIM: Color32 = Color32::from_rgb(141, 148, 156);
pub const PLAN: Color32 = Color32::from_rgb(227, 179, 65);
pub const ACTUAL: Color32 = Color32::from_rgb(76, 201, 217);
pub const LIMIT: Color32 = Color32::from_rgb(242, 124, 112);
pub const PASS: Color32 = Color32::from_rgb(109, 179, 122);

/// Messages, and the mark on a result that is not of the present config: something to look at, not a failure.
const NOTE: Color32 = PLAN;

/// One colour per axis or motor, the same in every scope. None is amber, cyan, green or red: those mean planned,
/// measured, passing and over the limit.
/// Each at least 24 apart (CIE76) from the others and from the reserved colours and the text colours.
const PALETTE: [Color32; 6] = [
    Color32::from_rgb(90, 140, 235),
    Color32::from_rgb(232, 124, 186),
    Color32::from_rgb(184, 148, 242),
    Color32::from_rgb(204, 172, 132),
    Color32::from_rgb(160, 205, 250),
    Color32::from_rgb(200, 90, 200),
];

/// The look of every panel: the palette above, Atkinson Hyperlegible Next (made so that 1, l and I and 0 and O
/// cannot be mixed up, which matters in tables of article numbers and values) and a calm spacing.
pub fn apply_theme(ctx: &egui::Context) {
    let mut fonts = FontDefinitions::default();
    fonts.font_data.insert("atkinson".into(), Arc::new(FontData::from_static(sim_app::FONT)));
    // One family for everything, numbers in fields and tables too.
    for family in [FontFamily::Proportional, FontFamily::Monospace] {
        fonts.families.entry(family).or_default().insert(0, "atkinson".into());
    }
    ctx.set_fonts(fonts);

    let mut style = (*ctx.global_style()).clone();
    style.text_styles = [
        (TextStyle::Heading, FontId::proportional(17.0)),
        (TextStyle::Body, FontId::proportional(14.0)),
        (TextStyle::Button, FontId::proportional(14.0)),
        (TextStyle::Small, FontId::proportional(11.5)),
        (TextStyle::Monospace, FontId::monospace(14.0)),
    ]
    .into();
    style.spacing.item_spacing = vec2(8.0, 6.0);
    style.spacing.button_padding = vec2(8.0, 3.0);
    style.spacing.indent = 14.0;
    let v = &mut style.visuals;
    *v = egui::Visuals::dark();
    (v.panel_fill, v.window_fill, v.extreme_bg_color, v.faint_bg_color) = (GRAPHITE, GRAPHITE, INSET, Color32::from_rgb(42, 46, 52));
    (v.hyperlink_color, v.warn_fg_color, v.error_fg_color) = (PLAN, PLAN, LIMIT);
    v.selection.bg_fill = Color32::from_rgba_unmultiplied(76, 201, 217, 70);
    v.selection.stroke = Stroke::new(1.0, ACTUAL);
    v.window_stroke = Stroke::new(1.0, MACHINED);
    let w = &mut v.widgets;
    w.noninteractive.bg_stroke = Stroke::new(1.0, MACHINED);
    w.noninteractive.fg_stroke = Stroke::new(1.0, CHALK);
    (w.inactive.bg_fill, w.inactive.weak_bg_fill) = (MACHINED, MACHINED);
    w.inactive.fg_stroke = Stroke::new(1.0, CHALK);
    (w.hovered.bg_fill, w.hovered.weak_bg_fill) = (Color32::from_rgb(74, 81, 90), Color32::from_rgb(74, 81, 90));
    w.hovered.bg_stroke = Stroke::new(1.0, DIM);
    (w.active.bg_fill, w.active.weak_bg_fill) = (Color32::from_rgb(52, 92, 100), Color32::from_rgb(52, 92, 100));
    w.active.bg_stroke = Stroke::new(1.0, ACTUAL);
    for state in [&mut w.noninteractive, &mut w.inactive, &mut w.hovered, &mut w.active, &mut w.open] {
        state.corner_radius = CornerRadius::same(3);
    }
    ctx.set_global_style(style);
}

/// What the 3D view draws besides the parts. Kept by the window, not the model.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Overlays {
    /// Parts drawn from CAD meshes see-through, to show the belt and pulleys inside them.
    pub xray: bool,
    /// The machine's force arrows (`Scene::arrows`).
    pub forces: bool,
    /// Where the plan has the tool, with the way to the tool drawn as many times longer as the deflections.
    pub planned: bool,
    /// Each part coloured by how close the analysed cycle runs it to its limits (`part_load`).
    pub load: bool,
    /// The machine swinging in its mode in place of the live state, which waits (`session::mode_scene`).
    pub mode: bool,
    /// What the travelled path is coloured by.
    pub trail: TrailColor,
    /// The travelled path drawn from the plan with its gap to it as many times larger as the deflections, and the
    /// gap as a line every few centimetres.
    pub tube: bool,
    /// A small picture from in front of the tool, in the view's corner.
    pub tool_view: bool,
}

/// What the travelled path is coloured by: cyan as everything travelled, or by a value along it (`trail_color`).
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub enum TrailColor {
    #[default]
    Plain,
    Speed,
    FollowingError,
}

/// The scale the travelled path is coloured by, low (0) to high (1): dark blue through violet and pink to a light
/// rose. Sequential, and none of the palette's meaning colours (amber, cyan, green, red).
pub fn trail_color(f: f32) -> [f32; 3] {
    const STOPS: [[f32; 3]; 4] = [[0.16, 0.2, 0.55], [0.5, 0.25, 0.7], [0.88, 0.4, 0.68], [1.0, 0.88, 0.9]];
    let at = f.clamp(0.0, 1.0) * (STOPS.len() - 1) as f32;
    let (i, t) = ((at as usize).min(STOPS.len() - 2), at - (at as usize).min(STOPS.len() - 2) as f32);
    [0, 1, 2].map(|c| STOPS[i][c] + (STOPS[i + 1][c] - STOPS[i][c]) * t)
}

/// What the view's controls show besides the overlays, which the window works out.
pub struct ViewInfo<'a> {
    /// The machine tells a mode to show.
    pub has_mode: bool,
    /// The lowest and highest value the travelled path is coloured by, in SI units.
    pub range: Option<(f64, f64)>,
    /// The run compared beside the machine's: whether it runs, and the effect (its config path) switched in it.
    pub compare: (bool, &'a str),
    /// The view is being recorded (`sim_app::video`).
    pub recording: bool,
}

/// The timeline under the 3D view: which of the recorded scenes the view draws, and whether they replay.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Timeline {
    /// The simulated time shown, in s; `None` for the newest scene, live.
    pub time: Option<f64>,
    pub playing: bool,
    /// Simulated seconds replayed per second.
    pub speed: f64,
}

impl Default for Timeline {
    fn default() -> Self {
        Self { time: None, playing: false, speed: 1.0 }
    }
}

impl Timeline {
    /// Replay started or stopped. Started live or at the end, it starts from the oldest scene of `span`, as there is
    /// nothing after the newest to replay.
    pub fn toggled(self, span: (f64, f64)) -> Self {
        if self.playing {
            return Self { playing: false, ..self };
        }
        let time = self.time.filter(|t| *t < span.1).unwrap_or(span.0);
        Self { time: Some(time), playing: true, ..self }
    }
}

/// The tabs of the setup panel on the left.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub enum SetupTab {
    #[default]
    Motion,
    Effects,
    Parameters,
    Config,
}

/// The tabs of the tools panel on the right: one per computation, each showing its inputs and results.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub enum ToolTab {
    #[default]
    Cycle,
    EffectViews,
    Search,
    Path,
    Trace,
    Sweep,
    Tuning,
}

/// What the window keeps for the panels between frames.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct PanelState {
    pub setup: SetupTab,
    pub tools: ToolTab,
    /// The check picked in the top bar, shown at the top of the tools panel.
    pub check: Option<&'static str>,
    /// The part clicked in the 3D view: its id and how many parts of that id come before it in the scene.
    pub part: Option<(&'static str, usize)>,
}

/// What the user did this frame. Collected while drawing, applied afterwards.
pub enum Action {
    Machine(&'static MachineEntry),
    Language(&'static str),
    Pause(bool),
    Speed(f64),
    /// Camera yaw and pitch in degrees.
    View(f32, f32),
    Overlays(Overlays),
    /// Start or stop recording the view.
    Record(bool),
    /// The timeline was scrubbed, replayed, stopped or sent live.
    Timeline(Timeline),
    /// One recorded scene on (`true`) or back.
    Step(bool),
    /// How many times larger the deflections are drawn.
    Exaggeration(f64),
    /// The run beside the machine's with one effect switched: on or off, and the effect (its config path).
    Compare(bool, String),
    Panels(PanelState),
    Target(usize, f64),
    Command(Command),
    Param(String, Value),
    Scope(usize, bool),
    /// Seconds of history the scopes show.
    Window(f64),
    Freeze(bool),
    Export,
    /// The overlay of the replayed trace goes to a CSV file.
    ExportOverlay,
    /// The sizing report goes to a file the system's dialog asks for.
    ExportReport,
    TuningAxis(usize),
    RunTuning,
    CloseTuning,
    /// What the search, path and trace fields hold after an edit.
    Inputs(Inputs),
    RunAnalysis(Analysis),
    ApplyOptimised,
    /// A row of the motor search goes into the config.
    UseCandidate(usize),
    /// The config file field: open it, or save the config to it; with `true`, a file dialog asks for the file first.
    OpenConfig(bool),
    SaveConfig(bool),
    /// The typed part names go into the config.
    ApplyParts,
    CloseAnalysis,
    Clear(Analysis),
    ClearMessages,
}

/// How much of its limit a check uses (`analysis::utilisation`): 1 is at the limit, above 1 over it. `None` where
/// the share says nothing: a limit of 0 (a failing check is then simply over, `INFINITY`; a passing one has used
/// none of it), or a check whose result is not its value against its limit (the DC link's energy passes as long as
/// the chopper copes, whatever it is against the capacitors' room).
fn usage(v: &Verdict) -> Option<f64> {
    // shortcut: named, as `Verdict` does not say how it judges. At its limit the chopper holds the voltage, so a
    // share there only says that the resistor took over. Upgrade when verdicts carry how they judge.
    if v.check == "dc-link-energy" {
        return None;
    }
    if v.limit == 0.0 {
        return Some(if v.ok { 0.0 } else { f64::INFINITY });
    }
    let share = utilisation(v);
    // "belt-slack" passes only above its limit, the others at it too.
    let judged_by_share = v.ok == if v.check == "belt-slack" { share < 1.0 } else { share <= 1.0 };
    (share.is_finite() && share >= 0.0 && judged_by_share).then_some(share)
}

/// Per check of the analysed cycle, its worst motor: (check, share or `None`, passes), failed ones first, then the
/// tightest. A check with no share (see `usage`) counts in the summary but gets no gauge.
fn margins(model: &Model) -> Vec<(&'static str, Option<f64>, bool)> {
    let Some((_, verdicts, _)) = &model.cycle_report else { return Vec::new() };
    let mut worst: Vec<(&'static str, Option<f64>, bool)> = Vec::new();
    for v in verdicts {
        let used = usage(v);
        match worst.iter_mut().find(|w| w.0 == v.check) {
            Some(w) => (w.1, w.2) = (w.1.zip(used).map(|(a, b)| a.max(b)), w.2 && v.ok),
            None => worst.push((v.check, used, v.ok)),
        }
    }
    let key = |share: Option<f64>| share.unwrap_or(-1.0);
    worst.sort_by(|a, b| a.2.cmp(&b.2).then(key(b.1).total_cmp(&key(a.1))));
    worst
}

/// A share as the gauges write it: a percentage, or "over" for a check over a limit of 0.
fn share_text(model: &Model, used: f64) -> String {
    if used.is_finite() { format!("{:.0} {}", used * 100.0, "%") } else { model.tr.tr("ui-verdict-over").to_string() }
}

/// A check's colour: red over its limit, amber within a tenth of it, green with room.
fn margin_color(used: f64, passes: bool) -> Color32 {
    if !passes {
        LIMIT
    } else if used > 0.9 {
        PLAN
    } else {
        PASS
    }
}

/// How much of the limit a check uses, as a bar: the share filled, coloured by the margin left.
fn gauge_bar(ui: &mut Ui, width: f32, used: f64, passes: bool) {
    let (rect, _) = ui.allocate_exact_size(vec2(width, 5.0), Sense::hover());
    let painter = ui.painter();
    painter.rect_filled(rect, 2.0, INSET);
    let filled = Rect::from_min_size(rect.min, vec2(rect.width() * used.clamp(0.0, 1.0) as f32, rect.height()));
    painter.rect_filled(filled, 2.0, margin_color(used, passes));
}

/// Whether verdict `v` concerns the part `id`, the `nth` part of that id in the scene: a motor's and its drive's
/// checks its motor housing, a gearbox's or pulley's that of its motor, the belt's every strip and tooth, the
/// guides' the carriage and the Z axis.
/// shortcut: relies on the scene's part ids, and on the k-th "motor", "gearbox" and "pulley" being motor k's as
/// `scene::drive_unit` draws them; a machine without such parts gets no load colours. Upgrade to the machine naming
/// the parts each check concerns when one has no such parts.
pub fn concerns(v: &Verdict, id: &str, nth: usize) -> bool {
    // A check of no particular motor concerns that part of every motor.
    let motor = v.motor.is_none_or(|k| k == nth);
    let is = |prefix: &str| v.check.starts_with(prefix);
    if is("motor-") || is("drive-") {
        id == MOTOR_PART && motor
    } else if is("gearbox-") {
        id == "gearbox" && motor
    } else if is("pulley-") {
        id == "pulley" && motor
    } else if is("belt-") {
        id == "belt" || id == "belt-tooth"
    } else if is("guide-") {
        id == "carriage" || id == "z-axis"
    } else {
        false
    }
}

/// The colour of a part in the load view: that of the gauge (`margin_color`) of the tightest check of the analysed
/// cycle that concerns it (`concerns`). `None` where none does, or none is judged by its share (`usage`).
pub fn load_color(model: &Model, id: &str, nth: usize) -> Option<Color32> {
    let (_, verdicts, _) = model.cycle_report.as_ref()?;
    let shares = verdicts.iter().filter(|v| concerns(v, id, nth)).filter_map(|v| Some((usage(v)?, v.ok)));
    shares.reduce(|a, b| (a.0.max(b.0), a.1 && b.1)).map(|(used, passes)| margin_color(used, passes))
}

/// The bar on top: machine and language, run and speed, the open config file, and how close the analysed cycle
/// runs each part to its limit, the tightest checks as gauges. A gauge opens its check in the tools panel. While the
/// view is `recording`, a sign of it and its Stop button, out of the frames (the view's own controls are hidden then).
pub fn top_bar(ui: &mut Ui, model: &Model, panels: PanelState, recording: bool, actions: &mut Vec<Action>) {
    let tr = |key: &str| model.tr.tr(key).to_string();
    ui.horizontal_centered(|ui| {
        ComboBox::from_id_salt("machine").selected_text(RichText::new(tr(model.entry.key)).strong()).show_ui(ui, |ui| {
            for entry in MACHINES {
                if ui.selectable_label(entry.id == model.entry.id, tr(entry.key)).clicked() {
                    actions.push(Action::Machine(entry));
                }
            }
        });
        ComboBox::from_id_salt("language").width(90.0).selected_text(tr(&format!("lang-{}", model.lang))).show_ui(ui, |ui| {
            for lang in LANGUAGES {
                if ui.selectable_label(*lang == model.lang, tr(&format!("lang-{lang}"))).clicked() {
                    actions.push(Action::Language(lang));
                }
            }
        });
        ui.separator();
        if ui.button(tr(if model.paused { "ui-resume" } else { "ui-pause" })).clicked() {
            actions.push(Action::Pause(!model.paused));
        }
        let mut speed = model.speed;
        ui.spacing_mut().slider_width = 90.0;
        if ui.add(Slider::new(&mut speed, 0.01..=2.0).logarithmic(true).text(tr("ui-speed"))).changed() {
            actions.push(Action::Speed(speed));
        }
        if recording {
            ui.separator();
            ui.label(RichText::new(tr("ui-recording")).color(LIMIT).strong());
            if ui.button(tr("ui-recording-stop")).clicked() {
                actions.push(Action::Record(false));
            }
        }
        if let Some(name) = std::path::Path::new(&model.inputs.config_file).file_name() {
            ui.separator();
            ui.label(RichText::new(name.to_string_lossy()).color(DIM)).on_hover_text(&model.inputs.config_file);
        }
        ui.separator();

        let checks = margins(model);
        if model.running(Analysis::Cycle) {
            ui.spinner();
        }
        if checks.is_empty() {
            if !model.running(Analysis::Cycle) && ui.button(tr("ui-analysis-run-cycle")).clicked() {
                actions.push(Action::RunAnalysis(Analysis::Cycle));
                actions.push(Action::Panels(PanelState { tools: ToolTab::Cycle, ..panels }));
            }
            ui.label(RichText::new(tr("ui-verdicts-none")).color(DIM));
            return;
        }
        let failed = checks.iter().filter(|c| !c.2).count();
        let count = |key: &str, n: usize| if n == 1 { model.tr.tr(&format!("{key}-one")).to_string() } else { model.tr.tr_arg(key, &n.to_string()) };
        let (summary, color) = if failed == 0 { (count("ui-verdicts-pass", checks.len()), PASS) } else { (count("ui-verdicts-fail", failed), LIMIT) };
        let all = ui.add(egui::Button::new(RichText::new(summary).color(color).strong()).frame(false)).on_hover_text(tr("ui-verdicts-all-hint"));
        if all.clicked() {
            actions.push(Action::Panels(PanelState { tools: ToolTab::Cycle, check: None, ..panels }));
        }
        if model.outdated(Analysis::Cycle) {
            ui.label(RichText::new(tr("ui-analysis-outdated")).color(NOTE));
        }
        // As many of the tightest checks as there is room for, each a name over its bar.
        let room = ((ui.available_width() - 8.0) / 178.0).floor().max(0.0) as usize;
        for (check, used, passes) in checks.iter().filter_map(|&(check, used, passes)| Some((check, used?, passes))).take(room) {
            let name = tr(&format!("check-{check}"));
            let picked = panels.check == Some(check);
            let frame = Frame::new()
                .inner_margin(Margin::symmetric(6, 3))
                .corner_radius(3)
                .fill(if picked { Color32::from_rgb(48, 53, 60) } else { Color32::TRANSPARENT })
                .stroke(if picked { Stroke::new(1.0, ACTUAL) } else { Stroke::NONE });
            let response = frame
                .show(ui, |ui| {
                    ui.set_width(158.0);
                    ui.vertical(|ui| {
                        ui.spacing_mut().item_spacing.y = 3.0;
                        ui.horizontal(|ui| {
                            ui.add(Label::new(RichText::new(&name).small().color(CHALK)).truncate().selectable(false));
                        });
                        ui.horizontal(|ui| {
                            gauge_bar(ui, 112.0, used, passes);
                            ui.label(RichText::new(share_text(model, used)).small().color(margin_color(used, passes)));
                        });
                    });
                })
                .response
                .interact(Sense::click())
                .on_hover_text(format!("{name}\n{}", tr("ui-verdicts-gauge-hint")));
            if response.clicked() {
                let check = if picked { None } else { Some(check) };
                actions.push(Action::Panels(PanelState { tools: ToolTab::Cycle, check, ..panels }));
            }
        }
    });
}

/// The view's own controls, over the 3D view: camera presets, what is drawn besides the parts, and how much
/// larger the deflections are drawn; the mode, the run compared, the tool's picture, and what the travelled path is
/// coloured by.
pub fn view_controls(ui: &mut Ui, model: &Model, overlays: Overlays, view: ViewInfo, actions: &mut Vec<Action>) {
    let tr = |key: &str| model.tr.tr(key).to_string();
    Frame::new().fill(Color32::from_rgba_unmultiplied(36, 40, 45, 215)).corner_radius(4).inner_margin(Margin::same(8)).show(ui, |ui| {
        ui.horizontal(|ui| {
            for (key, yaw, pitch) in [("ui-view-front", 0.0, 0.0), ("ui-view-side", 90.0, 0.0), ("ui-view-top", 0.0, 89.0), ("ui-view-iso", 30.0, 20.0)] {
                if ui.button(tr(key)).clicked() {
                    actions.push(Action::View(yaw, pitch));
                }
            }
            // A button like the presets beside it, shown pressed while recording.
            if ui.add(egui::Button::new(tr("ui-view-record")).selected(view.recording)).on_hover_text(tr("ui-view-record-hint")).clicked() {
                actions.push(Action::Record(!view.recording));
            }
        });
        ui.horizontal(|ui| {
            let mut shown = overlays;
            ui.checkbox(&mut shown.xray, tr("ui-view-xray")).on_hover_text(tr("ui-view-xray-hint"));
            ui.checkbox(&mut shown.forces, tr("ui-view-forces")).on_hover_text(tr("ui-view-forces-hint"));
            ui.checkbox(&mut shown.planned, tr("ui-view-planned")).on_hover_text(tr("ui-view-planned-hint"));
            ui.checkbox(&mut shown.load, tr("ui-view-load")).on_hover_text(tr("ui-view-load-hint"));
            if shown != overlays {
                actions.push(Action::Overlays(shown));
            }
        });
        ui.horizontal(|ui| {
            let mut shown = overlays;
            let mode = ui.add_enabled(view.has_mode, Checkbox::new(&mut shown.mode, tr("ui-view-mode")));
            mode.on_hover_text(tr("ui-view-mode-hint")).on_disabled_hover_text(tr("ui-view-mode-none"));
            ui.checkbox(&mut shown.tool_view, tr("ui-view-tool")).on_hover_text(tr("ui-view-tool-hint"));
            if shown != overlays {
                actions.push(Action::Overlays(shown));
            }
            let (mut on, effect) = view.compare;
            if ui.checkbox(&mut on, tr("ui-view-compare")).on_hover_text(tr("ui-view-compare-hint")).changed() {
                actions.push(Action::Compare(on, effect.to_string()));
            }
            let name = |path: &str| tr(&format!("effect-{}", path.strip_prefix("effects.").unwrap_or(path)));
            ComboBox::from_id_salt("view-compare").selected_text(name(effect)).width(150.0).show_ui(ui, |ui| {
                for (path, key, _) in model.effects() {
                    if ui.selectable_label(path == effect, tr(&key)).clicked() && path != effect {
                        actions.push(Action::Compare(on, path));
                    }
                }
            });
        });
        ui.horizontal(|ui| {
            let mut shown = overlays;
            ui.label(tr("ui-view-trail"));
            for (color, key) in [(TrailColor::Plain, "ui-view-trail-plain"), (TrailColor::Speed, "ui-view-trail-speed"), (TrailColor::FollowingError, "ui-view-trail-error")] {
                ui.radio_value(&mut shown.trail, color, tr(key));
            }
            ui.checkbox(&mut shown.tube, tr("ui-view-tube")).on_hover_text(tr("ui-view-tube-hint"));
            if shown != overlays {
                actions.push(Action::Overlays(shown));
            }
        });
        // The scale the path is coloured by, between the lowest and highest value along it.
        if let Some((low, high)) = view.range.filter(|_| overlays.trail != TrailColor::Plain) {
            let (factor, unit) = if overlays.trail == TrailColor::Speed { (1.0, tr("unit-m-per-s")) } else { (1e3, tr("unit-mm")) };
            scale_bar(ui, &format!("{:.3} {unit}", low * factor), &format!("{:.3} {unit}", high * factor), trail_color);
        }
        if overlays.load {
            ui.horizontal(|ui| match model.cycle_report {
                Some(_) => {
                    for (color, key) in [(PASS, "ui-load-room"), (PLAN, "ui-load-close"), (LIMIT, "ui-load-over")] {
                        ui.label(RichText::new(format!("■ {}", tr(key))).color(color));
                    }
                }
                None => drop(ui.label(RichText::new(tr("ui-verdicts-none")).small().color(DIM))),
            });
        }
        let mut factor = model.exaggeration;
        ui.spacing_mut().slider_width = 120.0;
        let slider = Slider::new(&mut factor, 1.0..=10_000.0).logarithmic(true).text(tr("ui-view-exaggeration"));
        if ui.add(slider).on_hover_text(tr("ui-view-exaggeration-hint")).changed() {
            actions.push(Action::Exaggeration(factor));
        }
    });
}

/// A row of tabs: the one shown filled, the others plain.
fn tabs<T: Copy + PartialEq>(ui: &mut Ui, model: &Model, shown: T, all: &[(T, &str)]) -> Option<T> {
    let mut picked = None;
    ui.horizontal_wrapped(|ui| {
        ui.spacing_mut().item_spacing.x = 2.0;
        for &(tab, key) in all {
            if ui.selectable_label(tab == shown, model.tr.tr(key)).clicked() && tab != shown {
                picked = Some(tab);
            }
        }
    });
    ui.separator();
    picked
}

/// The setup on the left: where the machine goes and how to drive it, which effects are modelled, the parameters,
/// and the config file with the parts it names.
pub fn setup(ui: &mut Ui, model: &Model, panels: PanelState, actions: &mut Vec<Action>) {
    let tr = |key: &str| model.tr.tr(key).to_string();
    let all = [(SetupTab::Motion, "ui-motion"), (SetupTab::Effects, "ui-effects"), (SetupTab::Parameters, "ui-parameters"), (SetupTab::Config, "ui-config")];
    if let Some(tab) = tabs(ui, model, panels.setup, &all) {
        actions.push(Action::Panels(PanelState { setup: tab, ..panels }));
    }
    ScrollArea::vertical().show(ui, |ui| match panels.setup {
        SetupTab::Motion => {
            for (i, axis) in model.session.axes.iter().enumerate() {
                let mut value = model.target[i];
                ui.horizontal(|ui| {
                    if ui.add(DragValue::new(&mut value).speed(0.005).range(axis.min..=axis.max)).changed() {
                        actions.push(Action::Target(i, value));
                    }
                    ui.label(format!("{} [{}]", tr(axis.key), axis.unit));
                });
            }
            ui.horizontal_wrapped(|ui| {
                if ui.button(tr("ui-move")).clicked() {
                    actions.push(Action::Command(Command::MoveTo(model.target.clone())));
                }
                if ui.button(tr("ui-start-cycle")).clicked() {
                    actions.push(Action::Command(Command::StartCycle));
                }
                if ui.button(tr("ui-stop")).clicked() {
                    actions.push(Action::Command(Command::Stop));
                }
            });
            // Drives off: the holding brakes engage where the machine has them. Until enabled, nothing moves it.
            ui.horizontal_wrapped(|ui| {
                if ui.button(tr("ui-disable")).clicked() {
                    actions.push(Action::Command(Command::Disable));
                }
                if ui.button(tr("ui-enable")).clicked() {
                    actions.push(Action::Command(Command::Enable));
                }
            });
            if !model.messages.is_empty() {
                ui.separator();
                ui.horizontal(|ui| {
                    ui.label(tr("ui-messages"));
                    if ui.small_button(tr("ui-clear")).clicked() {
                        actions.push(Action::ClearMessages);
                    }
                });
                for (key, detail) in &model.messages {
                    ui.colored_label(NOTE, model.tr.tr_arg(key, detail));
                }
            }
        }
        SetupTab::Effects => {
            for (path, key, mut on) in model.effects() {
                if ui.checkbox(&mut on, tr(&key)).changed() {
                    actions.push(Action::Param(path, Value::Boolean(on)));
                }
            }
        }
        SetupTab::Parameters => {
            for group in model.param_groups() {
                ui.collapsing(tr(&group.key), |ui| {
                    for (path, key, value) in group.params {
                        ui.horizontal(|ui| {
                            match value {
                                Value::Integer(mut n) => {
                                    if ui.add(DragValue::new(&mut n)).changed() {
                                        actions.push(Action::Param(path, Value::Integer(n)));
                                    }
                                }
                                Value::Float(mut x) => {
                                    // One percent of the value per pixel dragged.
                                    let speed = if x == 0.0 { 1e-3 } else { x.abs() * 0.01 };
                                    if ui.add(DragValue::new(&mut x).speed(speed).min_decimals(2).max_decimals(8)).changed() {
                                        actions.push(Action::Param(path, Value::Float(x)));
                                    }
                                }
                                _ => {}
                            }
                            ui.label(tr(&key));
                        });
                    }
                });
            }
        }
        SetupTab::Config => {
            let mut inputs = model.inputs.clone();
            config(ui, model, &mut inputs, actions);
            if inputs != model.inputs {
                actions.push(Action::Inputs(inputs));
            }
        }
    });
}

/// The check picked in the top bar: per motor its value, limit and how much of the limit it uses.
fn picked_check(ui: &mut Ui, model: &Model, check: &'static str, panels: PanelState, actions: &mut Vec<Action>) {
    let Some((_, verdicts, _)) = &model.cycle_report else { return };
    let tr = |key: &str| model.tr.tr(key).to_string();
    Frame::new().fill(INSET).corner_radius(4).inner_margin(Margin::same(10)).show(ui, |ui| {
        ui.set_width(ui.available_width());
        ui.horizontal(|ui| {
            ui.heading(tr(&format!("check-{check}")));
            ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                if ui.small_button(tr("ui-close")).clicked() {
                    actions.push(Action::Panels(PanelState { check: None, ..panels }));
                }
            });
        });
        Grid::new("picked-check").num_columns(4).spacing(vec2(14.0, 6.0)).show(ui, |ui| {
            for key in ["ui-verdict-motor", "ui-verdict-value", "ui-verdict-limit", "ui-verdict-used"] {
                ui.label(RichText::new(tr(key)).small().color(DIM));
            }
            ui.end_row();
            for v in verdicts.iter().filter(|v| v.check == check) {
                ui.label(v.motor.map_or(String::new(), |k| tr(model.session.motors[k].key)));
                verdict_cells(ui, model, v);
            }
        });
        let judged_otherwise = verdicts.iter().any(|v| v.check == check && v.limit != 0.0 && usage(v).is_none());
        let direction = if judged_otherwise {
            "ui-verdict-judged-otherwise"
        } else if check == "belt-slack" {
            "ui-verdict-above"
        } else if AT_LEAST.contains(&check) {
            "ui-verdict-at-least"
        } else {
            "ui-verdict-at-most"
        };
        ui.label(RichText::new(tr(direction)).small().color(DIM));
    });
    ui.add_space(6.0);
}

/// A verdict's value, limit and share of the limit as cells of a grid row, and the row's end.
fn verdict_cells(ui: &mut Ui, model: &Model, v: &Verdict) {
    ui.label(sim_app::model::short(v.value));
    ui.label(sim_app::model::short(v.limit));
    match usage(v) {
        Some(used) => drop(ui.horizontal(|ui| {
            gauge_bar(ui, 90.0, used, v.ok);
            ui.label(RichText::new(share_text(model, used)).color(margin_color(used, v.ok)));
        })),
        None => drop(ui.label(RichText::new(model.tr.tr(if v.ok { "ui-verdict-passes" } else { "ui-verdict-fails" })).color(if v.ok { PASS } else { LIMIT }))),
    }
    ui.end_row();
}

/// "Motor A  1250 1/min"
pub fn label_text(model: &Model, label: &sim_core::Label) -> String {
    format!("{}  {:.*} {}", model.tr.tr(label.owner_key), label.decimals, label.value, model.tr.tr(label.unit_key))
}

/// The part clicked in the 3D view, in the `scene` it is drawn from: its name, the live values written next to it,
/// and the checks of the analysed cycle that concern it (`concerns`).
fn inspector(ui: &mut Ui, model: &Model, scene: &Scene, (id, nth): (&'static str, usize), panels: PanelState, actions: &mut Vec<Action>) {
    let tr = |key: &str| model.tr.tr(key).to_string();
    Frame::new().fill(INSET).corner_radius(4).inner_margin(Margin::same(10)).show(ui, |ui| {
        ui.set_width(ui.available_width());
        ui.horizontal(|ui| {
            ui.heading(tr(&format!("part-{id}")));
            ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                if ui.small_button(tr("ui-close")).clicked() {
                    actions.push(Action::Panels(PanelState { part: None, ..panels }));
                }
            });
        });
        // The labels drawn near the part are its live values.
        // shortcut: near is within 0.2 m of the part's middle, which fits drive units of a few hundred millimetres; upgrade to
        // labels naming their part when a machine puts them further away.
        if let Some(part) = scene.parts.iter().filter(|p| p.id == id).nth(nth) {
            let near = |l: &&sim_core::Label| l.pos.iter().zip(part.pos).map(|(a, b)| (a - b).powi(2)).sum::<f64>() < 0.2f64.powi(2);
            for label in scene.labels.iter().filter(near) {
                ui.label(RichText::new(label_text(model, label)).color(ACTUAL));
            }
        }
        let Some((_, verdicts, _)) = &model.cycle_report else {
            ui.label(RichText::new(tr("ui-verdicts-none")).small().color(DIM));
            return;
        };
        let mine: Vec<_> = verdicts.iter().filter(|v| concerns(v, id, nth)).collect();
        if mine.is_empty() {
            ui.label(RichText::new(tr("ui-inspect-no-checks")).small().color(DIM));
            return;
        }
        // Long check names wrap, so that the panel keeps its width.
        Grid::new("inspector").num_columns(4).max_col_width(170.0).spacing(vec2(14.0, 6.0)).show(ui, |ui| {
            for key in ["ui-inspect-check", "ui-verdict-value", "ui-verdict-limit", "ui-verdict-used"] {
                ui.label(RichText::new(tr(key)).small().color(DIM));
            }
            ui.end_row();
            for v in mine {
                ui.label(tr(&format!("check-{}", v.check)));
                verdict_cells(ui, model, v);
            }
        });
    });
    ui.add_space(6.0);
}

/// The timeline over the bottom of the 3D view, `width` wide: back to live, a step back, replay, a step on, the
/// recorded `span` of simulated time to scrub through, and the replay speed.
pub fn timeline(ui: &mut Ui, model: &Model, timeline: Timeline, span: (f64, f64), width: f32, actions: &mut Vec<Action>) {
    let tr = |key: &str| model.tr.tr(key).to_string();
    Frame::new().fill(Color32::from_rgba_unmultiplied(36, 40, 45, 215)).corner_radius(4).inner_margin(Margin::symmetric(8, 5)).show(ui, |ui| {
        ui.set_width(width);
        ui.horizontal(|ui| {
            let live = timeline.time.is_none();
            if ui.selectable_label(live, tr("ui-timeline-live")).on_hover_text(tr("ui-timeline-live-hint")).clicked() && !live {
                actions.push(Action::Timeline(Timeline { time: None, playing: false, ..timeline }));
            }
            if ui.button("|◀").on_hover_text(tr("ui-timeline-step-hint")).clicked() {
                actions.push(Action::Step(false));
            }
            if ui.button(if timeline.playing { "❚❚" } else { "▶" }).on_hover_text(tr("ui-timeline-play-hint")).clicked() {
                actions.push(Action::Timeline(timeline.toggled(span)));
            }
            if ui.button("▶|").on_hover_text(tr("ui-timeline-step-hint")).clicked() {
                actions.push(Action::Step(true));
            }
            let mut time = timeline.time.unwrap_or(span.1);
            ui.spacing_mut().slider_width = (ui.available_width() - 250.0).max(60.0);
            let slider = ui.add(Slider::new(&mut time, span.0..=span.1).show_value(false));
            if slider.changed() {
                actions.push(Action::Timeline(Timeline { time: Some(time), playing: false, ..timeline }));
            }
            // The first contact with an obstacle as a red mark on the slider, where it lies in the recorded span.
            if let Some(contact) = model.scene.first_contact.filter(|t| *t >= span.0 && *t <= span.1 && span.1 > span.0) {
                let rect = slider.rect;
                let x = rect.left() + (rect.width() * ((contact - span.0) / (span.1 - span.0)) as f32);
                let mark = egui::Rect::from_center_size(egui::pos2(x, rect.center().y), egui::vec2(3.0, rect.height()));
                ui.painter().rect_filled(mark, 1.0, LIMIT);
                ui.interact(mark.expand(2.0), ui.id().with("contact-mark"), egui::Sense::hover()).on_hover_text(model.tr.tr_arg("ui-timeline-contact-hint", &format!("{contact:.3}")));
            }
            // The time shown, and how far back from the newest scene: the scopes' right edge.
            ui.label(RichText::new(format!("{time:.3} {}  {:+.3} {}", "s", time - span.1, "s")).color(if live { CHALK } else { ACTUAL }));
            for speed in [0.1, 0.25, 1.0] {
                if ui.selectable_label(timeline.speed == speed, format!("{speed}×")).on_hover_text(tr("ui-timeline-speed-hint")).clicked() {
                    actions.push(Action::Timeline(Timeline { speed, ..timeline }));
                }
            }
        });
    });
}

/// The tools on the right, one tab per computation, each with its run button and its results. A result arrives in
/// the tab its run was started from, so the tabs stay where the user put them.
/// The part clicked in the 3D view is inspected at the top of every tab, in the `scene` the view draws.
pub fn tools(ui: &mut Ui, model: &Model, scene: &Scene, panels: PanelState, actions: &mut Vec<Action>, scroll: Option<f32>) {
    let all = [
        (ToolTab::Cycle, "ui-analysis"),
        (ToolTab::EffectViews, "ui-effect-views"),
        (ToolTab::Search, "ui-search"),
        (ToolTab::Path, "ui-path"),
        (ToolTab::Trace, "ui-trace"),
        (ToolTab::Sweep, "ui-sweep-tab"),
        (ToolTab::Tuning, "ui-tuning"),
    ];
    if let Some(tab) = tabs(ui, model, panels.tools, &all) {
        actions.push(Action::Panels(PanelState { tools: tab, ..panels }));
    }
    // `scroll`: the screenshot hook's way to get a part further down into the picture.
    let area = ScrollArea::vertical().id_salt(("tools", panels.tools as u8));
    let area = if let Some(offset) = scroll { area.vertical_scroll_offset(offset) } else { area };
    area.show(ui, |ui| {
        let mut inputs = model.inputs.clone();
        if let Some(part) = panels.part {
            inspector(ui, model, scene, part, panels, actions);
        }
        match panels.tools {
            ToolTab::Cycle => {
                if let Some(check) = panels.check {
                    picked_check(ui, model, check, panels, actions);
                }
                analysis(ui, model, &mut inputs, actions);
            }
            ToolTab::EffectViews => effects(ui, model, &mut inputs, actions),
            ToolTab::Search => search(ui, model, &mut inputs, actions),
            ToolTab::Path => path(ui, model, &mut inputs, actions),
            ToolTab::Trace => trace(ui, model, &mut inputs, actions),
            ToolTab::Sweep => sweep(ui, model, &mut inputs, actions),
            ToolTab::Tuning => tuning(ui, model, actions),
        }
        if inputs != model.inputs {
            actions.push(Action::Inputs(inputs));
        }
    });
}

fn number(value: f64) -> String {
    format!("{value:.4}")
}

/// The scopes, under the view: one plot per quantity with actual values solid and their setpoints dashed, one
/// colour per axis or motor, a shared time axis and cursor, readable units, and what each trace did in the window.
/// Side by side as the panel's width allows. All of it is built from the signal list.
/// The time the timeline shows is a line in every scope, and a click in a scope scrubs the timeline to there.
pub fn scopes(ui: &mut Ui, model: &Model, timeline: Timeline, actions: &mut Vec<Action>) {
    let tr = |key: &str| model.tr.tr(key).to_string();
    let signals = &model.session.signals;
    // Owners in order of first appearance; their place decides the colour.
    let mut owners: Vec<&str> = Vec::new();
    for s in signals {
        if !owners.contains(&s.owner_key.as_str()) {
            owners.push(&s.owner_key);
        }
    }
    let color = |signal: usize| PALETTE[owners.iter().position(|o| *o == signals[signal].owner_key).unwrap_or(0) % PALETTE.len()];

    ui.horizontal(|ui| {
        ui.menu_button(tr("ui-scopes"), |ui| {
            for (i, scope) in model.scopes.iter().enumerate() {
                let mut visible = scope.visible;
                if ui.checkbox(&mut visible, format!("{} [{}]", tr(&scope.quantity_key), scope.unit)).changed() {
                    actions.push(Action::Scope(i, visible));
                }
            }
        });
        let mut window = model.window_s;
        if ui.add(DragValue::new(&mut window).speed(0.1).range(0.2..=60.0)).changed() {
            actions.push(Action::Window(window));
        }
        ui.label(tr("ui-window"));
        let mut frozen = model.frozen.is_some();
        if ui.checkbox(&mut frozen, tr("ui-freeze")).changed() {
            actions.push(Action::Freeze(frozen));
        }
        if ui.button(tr("ui-export")).clicked() {
            actions.push(Action::Export);
        }
        if model.scopes.iter().any(|s| s.lines.iter().any(|l| l.reference)) {
            ui.label(RichText::new(tr("ui-dashed")).color(DIM));
        }
    });
    let rows: Vec<&Vec<f64>> = model.visible_rows().collect();
    // At most about three thousand points per line, however long the window is.
    let stride = (rows.len() / 3000).max(1);
    let shown: Vec<_> = model.scopes.iter().filter(|s| s.visible).collect();
    let per_row = ((ui.available_width() / 420.0).floor() as usize).clamp(1, 4);
    ScrollArea::vertical().id_salt("scopes").show(ui, |ui| {
        for chunk in shown.chunks(per_row) {
            ui.columns(per_row, |columns| {
                for (scope, ui) in chunk.iter().zip(columns.iter_mut()) {
                    let stats: Vec<_> = scope.lines.iter().map(|l| model.stats(l.signal)).collect();
                    let peak = stats.iter().flatten().fold(0f64, |p, s| p.max(s.min.abs()).max(s.max.abs()));
                    let (scale, prefix) = display_scale(peak, &scope.unit);
                    let unit = format!("{prefix}{}", scope.unit);
                    ui.label(RichText::new(format!("{} [{unit}]", tr(&scope.quantity_key))).strong());
                    let plot = Plot::new((&scope.quantity_key, &scope.unit))
                        .height(150.0)
                        .legend(Legend::default())
                        .link_axis("scopes", [true, false])
                        .link_cursor("scopes", [true, false])
                        .x_axis_label(tr("ui-time"))
                        .show(ui, |plot| {
                            for line in &scope.lines {
                                let points: Vec<[f64; 2]> = rows.iter().step_by(stride).map(|row| [row[0], row[line.signal] * scale]).collect();
                                let info = &signals[line.signal];
                                // A setpoint carries the name of its actual value: one legend entry switches both.
                                let drawn = Line::new(tr(&info.owner_key), points).color(color(line.signal));
                                plot.line(if line.reference { drawn.style(LineStyle::dashed_dense()).width(1.2) } else { drawn.width(1.8) });
                            }
                            // Unnamed, so that it stays out of the legend.
                            if let Some(time) = timeline.time {
                                plot.vline(VLine::new("", time).color(CHALK).width(1.5));
                            }
                            plot.pointer_coordinate()
                        });
                    if plot.response.clicked()
                        && let Some(at) = plot.inner
                    {
                        actions.push(Action::Timeline(Timeline { time: Some(at.x), playing: false, ..timeline }));
                    }
                    // What each actual trace did in the window.
                    for (line, stats) in scope.lines.iter().zip(&stats).filter(|(l, _)| !l.reference) {
                        let Some(s) = stats else { continue };
                        ui.horizontal_wrapped(|ui| {
                            ui.colored_label(color(line.signal), tr(&signals[line.signal].owner_key));
                            let values = [("ui-stat-min", s.min), ("ui-stat-max", s.max), ("ui-stat-span", s.max - s.min), ("ui-stat-rms", s.rms)];
                            for (key, value) in values {
                                ui.label(RichText::new(tr(key)).small().color(DIM));
                                ui.label(RichText::new(number(value * scale)).small());
                            }
                            ui.label(RichText::new(&unit).small().color(DIM));
                        });
                    }
                }
            });
            ui.add_space(4.0);
        }
    });
}

/// A table of the model, drawn as it is: the header weak, numbers and words as they
/// are. Long names wrap, so that the last column stays inside the panel. With `pick`, every row starts with
/// a button under that label, which reports the row.
fn table(ui: &mut Ui, id: &str, table: &Table, mut pick: Option<(&str, &mut dyn FnMut(usize))>) {
    ScrollArea::horizontal().id_salt(id).show(ui, |ui| {
        // A table with a button has the most columns, and so the narrowest.
        Grid::new(id).striped(true).max_col_width(if pick.is_some() { 150.0 } else { 230.0 }).show(ui, |ui| {
            for (i, row) in table.iter().enumerate() {
                match (&mut pick, i) {
                    (Some(_), 0) => drop(ui.label("")),
                    (Some((label, picked)), _) if ui.small_button(*label).clicked() => picked(i - 1),
                    _ => {}
                }
                for (j, cell) in row.iter().enumerate() {
                    match (i, j) {
                        (0, _) => ui.weak(cell),
                        _ => ui.label(cell),
                    };
                }
                ui.end_row();
            }
        });
    });
}

/// The heading of a result, marked where the result is of a config that has changed since.
fn heading(ui: &mut Ui, model: &Model, key: &str, kind: Analysis) {
    ui.horizontal_wrapped(|ui| {
        ui.strong(model.tr.tr(key));
        if model.outdated(kind) {
            ui.colored_label(NOTE, model.tr.tr("ui-analysis-outdated"));
        }
    });
}

/// The button that starts a computation, or a spinner while it runs.
fn run_button(ui: &mut Ui, model: &Model, key: &str, kind: Analysis, actions: &mut Vec<Action>) {
    if model.running(kind) {
        ui.spinner();
    } else if ui.button(model.tr.tr(key)).clicked() {
        actions.push(Action::RunAnalysis(kind));
    }
}

/// What the command line computes about the cycle, as tables: the loads and the checks against the
/// parts' limits, which uncertain values matter most, and a shorter cycle that still passes.
fn analysis(ui: &mut Ui, model: &Model, inputs: &mut Inputs, actions: &mut Vec<Action>) {
    let tr = |key: &str| model.tr.tr(key).to_string();
    let tables = model.cycle_tables();
    let (ranking, optimised) = (model.sensitivity_table(), model.optimised_table());
    ui.horizontal_wrapped(|ui| {
        run_button(ui, model, "ui-analysis-run-cycle", Analysis::Cycle, actions);
        ui.checkbox(&mut inputs.bands, tr("ui-analysis-bands"));
        ui.add_enabled(inputs.bands, DragValue::new(&mut inputs.samples).range(0..=1000)).on_hover_text(tr("ui-analysis-samples-hint"));
        ui.label(tr("ui-analysis-samples"));
        ui.add_enabled(inputs.bands, Checkbox::new(&mut inputs.worst, tr("ui-analysis-worst"))).on_hover_text(tr("ui-analysis-worst-hint"));
        run_button(ui, model, "ui-analysis-run-sensitivity", Analysis::Sensitivity, actions);
        run_button(ui, model, "ui-analysis-run-optimise", Analysis::Optimise, actions);
        if (!tables.is_empty() || ranking.is_some() || optimised.is_some()) && ui.button(tr("ui-clear")).clicked() {
            actions.push(Action::CloseAnalysis);
        }
    });
    let banded = if model.band_samples > 0 { "ui-analysis-sampled" } else { "ui-analysis-banded" };
    for (rows, key) in tables.iter().zip(["ui-analysis-cycle", "ui-analysis-motors", "ui-analysis-verdicts", banded]) {
        heading(ui, model, key, Analysis::Cycle);
        table(ui, key, rows, None);
    }
    if let Some(rows) = model.worst_table() {
        heading(ui, model, "ui-analysis-worst-corners", Analysis::Cycle);
        table(ui, "ui-analysis-worst-corners", &rows, None);
    }
    points(ui, model);
    if let Some(rows) = &ranking {
        heading(ui, model, "ui-analysis-sensitivity", Analysis::Sensitivity);
        table(ui, "ui-analysis-sensitivity", rows, None);
    }
    if let Some(rows) = &optimised {
        heading(ui, model, "ui-analysis-optimised", Analysis::Optimise);
        table(ui, "ui-analysis-optimised", rows, None);
        optimised_front(ui, model);
        if ui.button(tr("ui-analysis-apply")).clicked() {
            actions.push(Action::ApplyOptimised);
        }
    }
}

/// The optimiser's front: energy per cycle against cycle time of the settings no other one beats (in cycle time, energy
/// and peak utilisation at once), the result marked by a vertical line.
fn optimised_front(ui: &mut Ui, model: &Model) {
    let Some((points, result)) = model.optimised_front() else { return };
    let tr = |key: &str| model.tr.tr(key).to_string();
    ui.label(tr("ui-optimised-front"));
    Plot::new("optimised-front").height(160.0).x_axis_label(tr("ui-analysis-cycle-time")).y_axis_label(tr("ui-optimised-energy")).show(ui, |plot| {
        plot.points(Points::new(tr("ui-optimised-front"), points).color(PALETTE[0]).radius(3.0));
        plot.vline(VLine::new(tr("ui-optimised-result"), result).color(PALETTE[1]));
    });
}

/// What the lines of the plot below are, each in its colour, dashed ones marked so: in place of a legend in the plot,
/// which would cover the lines.
fn key_row(ui: &mut Ui, keys: &[(String, Color32, bool)]) {
    ui.horizontal_wrapped(|ui| {
        for (label, color, dashed) in keys {
            // Each name whole on its line: the row wraps between names, not inside one.
            ui.add(Label::new(RichText::new(format!("{} {label}", if *dashed { "- -" } else { "—" })).color(*color)).wrap_mode(TextWrapMode::Extend));
        }
    });
}

/// A colour scale as a bar between two labels.
fn scale_bar(ui: &mut Ui, low: &str, high: &str, color: impl Fn(f32) -> [f32; 3]) {
    ui.horizontal(|ui| {
        ui.weak(low);
        let (rect, _) = ui.allocate_exact_size(vec2(160.0, 12.0), Sense::hover());
        const STEPS: usize = 40;
        for i in 0..STEPS {
            let [r, g, b] = color((i as f32 + 0.5) / STEPS as f32).map(|c| (c.clamp(0.0, 1.0) * 255.0) as u8);
            let left = rect.left() + rect.width() * i as f32 / STEPS as f32;
            let piece = Rect::from_min_max([left, rect.top()].into(), [left + rect.width() / STEPS as f32 + 0.5, rect.bottom()].into());
            ui.painter().rect_filled(piece, 0.0, Color32::from_rgb(r, g, b));
        }
        ui.weak(high);
    });
}

/// The effect views: the parts of each motor's torque, live over the scopes' window and over the analysed cycle;
/// where the energy of the analysed cycle went; per friction contact its curve of steady sliding with the point
/// it is at now; the temperatures; the belt sections; and the comparison of the cycle with an effect on and off.
fn effects(ui: &mut Ui, model: &Model, inputs: &mut Inputs, actions: &mut Vec<Action>) {
    let tr = |key: &str| model.tr.tr(key).to_string();
    run_button(ui, model, "ui-analysis-run-cycle", Analysis::Cycle, actions);
    let lines = model.breakdown_lines();
    if !lines.is_empty() {
        ui.horizontal_wrapped(|ui| {
            ui.strong(tr("ui-effects-breakdown"));
            ui.weak(tr("ui-effects-breakdown-hint"));
        });
    }
    for (title, parts) in lines {
        ui.label(&title);
        let keys: Vec<_> = parts.iter().enumerate().map(|(i, (label, _))| if i == TOTAL { (label.clone(), Color32::GRAY, true) } else { (label.clone(), PALETTE[i], false) }).collect();
        key_row(ui, &keys);
        Plot::new(("breakdown", &title)).height(150.0).link_axis("scopes", [true, false]).link_cursor("scopes", [true, false]).x_axis_label(tr("ui-time")).show(ui, |plot| {
            for (i, (label, points)) in parts.into_iter().enumerate() {
                // The torque itself, which the parts add up to, dashed.
                let line = Line::new(label, points);
                plot.line(if i == TOTAL { line.color(Color32::GRAY).style(LineStyle::dashed_dense()).width(1.5) } else { line.color(PALETTE[i]).width(1.6) });
            }
        });
    }
    for (rows, key) in [(model.breakdown_table(), "ui-effects-breakdown-cycle"), (model.budget_table(), "ui-effects-budget")] {
        if let Some(rows) = rows {
            heading(ui, model, key, Analysis::Cycle);
            table(ui, key, &rows, None);
        }
    }
    let views = model.friction_views();
    if !views.is_empty() {
        ui.horizontal_wrapped(|ui| {
            ui.strong(tr("ui-effects-friction"));
            ui.weak(tr("ui-effects-friction-hint"));
        });
    }
    for view in views {
        ui.label(&view.title);
        let mut keys: Vec<_> = view.curves.iter().enumerate().map(|(i, (label, _))| (label.clone(), PALETTE[(i + 1) % PALETTE.len()], i > 0)).collect();
        keys.push((tr("ui-effects-now"), PALETTE[0], false));
        key_row(ui, &keys);
        // Powers of five on the speed axis: with powers of ten a speed limit of 0.8 m/s has no labelled tick but 0.
        Plot::new(("friction", &view.title)).height(180.0).x_grid_spacer(log_grid_spacer(5)).x_axis_label(tr("ui-effects-speed")).y_axis_label(tr("ui-effects-force")).show(ui, |plot| {
            for (i, (label, points)) in view.curves.into_iter().enumerate() {
                // The friction force solid, its parts dashed.
                let line = Line::new(label, points).color(PALETTE[(i + 1) % PALETTE.len()]);
                plot.line(if i == 0 { line.width(2.0) } else { line.style(LineStyle::dashed_dense()).width(1.0) });
            }
            if let Some(now) = view.now {
                plot.points(Points::new(tr("ui-effects-now"), vec![now]).color(PALETTE[0]).radius(5.0));
            }
        });
        table(ui, &format!("friction-{}", view.title), &view.table, None);
    }
    thermal(ui, model);
    belt(ui, model);
    compare(ui, model, inputs, actions);
}

/// The temperatures of windings, gearboxes and guides over the scopes' window, the table of where they are and settle,
/// and the scale the 3D view colours them by.
fn thermal(ui: &mut Ui, model: &Model) {
    let tr = |key: &str| model.tr.tr(key).to_string();
    let view = model.thermal_view();
    ui.separator();
    ui.horizontal_wrapped(|ui| {
        ui.strong(tr("ui-effects-thermal"));
        ui.weak(tr(if view.off { "ui-effects-thermal-off" } else { "ui-effects-thermal-hint" }));
    });
    key_row(ui, &view.lines.iter().zip(PALETTE.iter().cycle()).map(|((label, _), color)| (label.clone(), *color, false)).collect::<Vec<_>>());
    Plot::new("thermal").height(150.0).link_axis("scopes", [true, false]).link_cursor("scopes", [true, false]).x_axis_label(tr("ui-time")).y_axis_label(tr("ui-effects-rise")).show(ui, |plot| {
        for ((label, points), color) in view.lines.into_iter().zip(PALETTE.iter().cycle()) {
            plot.line(Line::new(label, points).color(*color).width(1.8));
        }
    });
    table(ui, "thermal", &view.table, None);
    ui.horizontal(|ui| {
        scale_bar(ui, &tr("ui-effects-cold"), &tr("ui-effects-rated"), heat_color);
        let [r, g, b] = heat_color(2.0).map(|c| (c * 255.0) as u8);
        ui.colored_label(Color32::from_rgb(r, g, b), format!("■ {}", tr("ui-effects-above")));
    });
}

/// Tension and stretch of each belt section over the scopes' window, against pretension and rating, the table of the
/// newest sample, and the scale the 3D view colours the belt by.
fn belt(ui: &mut Ui, model: &Model) {
    let tr = |key: &str| model.tr.tr(key).to_string();
    let Some(view) = model.belt_view() else { return };
    ui.separator();
    ui.horizontal_wrapped(|ui| {
        ui.strong(tr("ui-effects-belt"));
        ui.weak(tr("ui-effects-belt-hint"));
    });
    let mut keys: Vec<_> = view.tension.iter().zip(PALETTE.iter().cycle()).map(|((label, _), color)| (label.clone(), *color, false)).collect();
    // The rating is in the table, not in the plot: at five times the pretension it would flatten the lines.
    keys.push((tr("ui-effects-pretension"), Color32::GRAY, true));
    ui.label(&view.titles[0]);
    key_row(ui, &keys);
    Plot::new("belt-tension").height(150.0).link_axis("scopes", [true, false]).link_cursor("scopes", [true, false]).x_axis_label(tr("ui-time")).show(ui, |plot| {
        for ((label, points), color) in view.tension.into_iter().zip(PALETTE.iter().cycle()) {
            plot.line(Line::new(label, points).color(*color).width(1.8));
        }
        plot.hline(HLine::new(tr("ui-effects-pretension"), view.pretension).color(Color32::GRAY).style(LineStyle::dashed_dense()));
    });
    ui.label(&view.titles[1]);
    Plot::new("belt-stretch").height(130.0).link_axis("scopes", [true, false]).link_cursor("scopes", [true, false]).x_axis_label(tr("ui-time")).show(ui, |plot| {
        for ((label, points), color) in view.stretch.into_iter().zip(PALETTE.iter().cycle()) {
            plot.line(Line::new(label, points).color(*color).width(1.8));
        }
    });
    table(ui, "belt", &view.table, None);
    // The 3D view's belt colour: from no tension through pretension (the belt's own colour) to twice the pretension.
    scale_bar(ui, &view.scale[0], &view.scale[1], |f| part_color(sim_core::scene::BELT_COLOR, 2.0 * f - 1.0, None));
}

/// The cycle with one effect on and off: the effect and the quantity to overlay, the overlay (off dashed) and the
/// difference per line.
fn compare(ui: &mut Ui, model: &Model, inputs: &mut Inputs, actions: &mut Vec<Action>) {
    let tr = |key: &str| model.tr.tr(key).to_string();
    let view = model.compare_view();
    ui.separator();
    ui.horizontal_wrapped(|ui| {
        ui.strong(tr("ui-effects-compare"));
        ui.weak(tr("ui-effects-compare-hint"));
    });
    ui.horizontal_wrapped(|ui| {
        let effect_label = |name: &str| tr(&format!("effect-{name}"));
        ui.label(tr("ui-effects-effect"));
        ComboBox::from_id_salt("compare-effect").selected_text(effect_label(&inputs.compare_effect)).width(180.0).show_ui(ui, |ui| {
            for (path, key, _) in model.effects() {
                ui.selectable_value(&mut inputs.compare_effect, path["effects.".len()..].to_string(), tr(&key));
            }
        });
        ui.label(tr("ui-effects-quantity"));
        ComboBox::from_id_salt("compare-quantity").selected_text(tr(&inputs.compare_quantity)).width(140.0).show_ui(ui, |ui| {
            for scope in &model.scopes {
                ui.selectable_value(&mut inputs.compare_quantity, scope.quantity_key.clone(), format!("{} [{}]", tr(&scope.quantity_key), scope.unit));
            }
        });
        run_button(ui, model, "ui-effects-run-compare", Analysis::Compare, actions);
        if view.is_some() && ui.button(tr("ui-clear")).clicked() {
            actions.push(Action::Clear(Analysis::Compare));
        }
    });
    let Some(view) = view else { return };
    ui.horizontal_wrapped(|ui| {
        ui.label(format!("{} · {}", view.effect, view.title));
        if model.outdated(Analysis::Compare) {
            ui.colored_label(NOTE, tr("ui-analysis-outdated"));
        }
    });
    let mut keys: Vec<_> = view.lines.iter().zip(PALETTE.iter().cycle()).map(|((label, ..), color)| (label.clone(), *color, false)).collect();
    keys.push((tr("ui-effects-off"), Color32::GRAY, true));
    key_row(ui, &keys);
    Plot::new("compare").height(170.0).x_axis_label(tr("ui-time")).show(ui, |plot| {
        for ((label, on, off), color) in view.lines.into_iter().zip(PALETTE.iter().cycle()) {
            plot.line(Line::new(label.clone(), on).color(*color).width(1.8));
            plot.line(Line::new(label, off).color(*color).style(LineStyle::dashed_dense()).width(1.2));
        }
    });
    table(ui, "compare", &view.table, None);
}

/// Where the torque itself is among the breakdown's lines, after its parts.
const TOTAL: usize = 4;

/// The config file: open one for the machine shown, or save the config as edited; and the parts it names,
/// typed by name.
fn config(ui: &mut Ui, model: &Model, inputs: &mut Inputs, actions: &mut Vec<Action>) {
    let tr = |key: &str| model.tr.tr(key).to_string();
    ui.horizontal_wrapped(|ui| {
        ui.label(tr("ui-path-file"));
        ui.add(TextEdit::singleline(&mut inputs.config_file).desired_width(260.0));
        for (key, action) in [("ui-config-open", Action::OpenConfig(false)), ("ui-config-save", Action::SaveConfig(false)), ("ui-config-open-dialog", Action::OpenConfig(true)), ("ui-config-save-dialog", Action::SaveConfig(true))] {
            if ui.button(tr(key)).clicked() {
                actions.push(action);
            }
        }
        let report = ui.add_enabled(!model.report_running(), egui::Button::new(tr("ui-report-export"))).on_hover_text(tr("ui-report-export-hint"));
        if report.clicked() {
            actions.push(Action::ExportReport);
        }
    });
    ui.horizontal_wrapped(|ui| {
        let [motor, supply, gearbox, drive, belt, axis] = &mut inputs.parts;
        ui.label(tr("ui-parts-motor"));
        ui.add(TextEdit::singleline(motor).desired_width(110.0));
        ui.label(tr("ui-search-supply"));
        ui.add(TextEdit::singleline(supply).desired_width(40.0));
        ui.label(tr("ui-search-gearbox"));
        ui.add(TextEdit::singleline(gearbox).desired_width(100.0));
        ui.label(tr("ui-parts-drive"));
        ui.add(TextEdit::singleline(drive).desired_width(130.0));
        ui.label(tr("ui-parts-belt"));
        ui.add(TextEdit::singleline(belt).desired_width(70.0));
        ui.label(tr("ui-search-axis"));
        ui.add(TextEdit::singleline(axis).desired_width(80.0));
        if ui.button(tr("ui-parts-apply")).clicked() {
            actions.push(Action::ApplyParts);
        }
    });
}

/// The motor search: every catalogue motor the fields ask for on the cycle, and a button per motor that
/// names it in the running config.
fn search(ui: &mut Ui, model: &Model, inputs: &mut Inputs, actions: &mut Vec<Action>) {
    let tr = |key: &str| model.tr.tr(key).to_string();
    let found = model.search_table();
    ui.horizontal_wrapped(|ui| {
        ComboBox::from_label(tr("ui-search-supply")).selected_text(&inputs.supply).width(60.0).show_ui(ui, |ui| {
            for supply in SUPPLIES {
                ui.selectable_value(&mut inputs.supply, supply.to_string(), supply);
            }
        });
        ui.label(tr("ui-search-series"));
        ui.add(TextEdit::singleline(&mut inputs.series).desired_width(90.0));
        ui.label(tr("ui-search-gearboxes"));
        ui.add(TextEdit::singleline(&mut inputs.gearboxes).desired_width(60.0));
        ui.checkbox(&mut inputs.axes, tr("ui-search-axes"));
        run_button(ui, model, "ui-search-run", Analysis::Search, actions);
        if found.is_some() && ui.button(tr("ui-clear")).clicked() {
            actions.push(Action::Clear(Analysis::Search));
        }
    });
    if let Some(rows) = &found {
        heading(ui, model, "ui-search-ranked", Analysis::Search);
        table(ui, "ui-search-ranked", rows, Some((&tr("ui-search-use"), &mut |row| actions.push(Action::UseCandidate(row)))));
    }
}

/// A path from a file: planned, run by the machine and analysed, with the planned path drawn.
fn path(ui: &mut Ui, model: &Model, inputs: &mut Inputs, actions: &mut Vec<Action>) {
    let tr = |key: &str| model.tr.tr(key).to_string();
    let tables = model.path_tables();
    ui.horizontal_wrapped(|ui| {
        ui.label(tr("ui-path-file"));
        ui.add(TextEdit::singleline(&mut inputs.path_file).desired_width(260.0));
        ui.radio_value(&mut inputs.path_timed, false, tr("ui-path-lines"));
        ui.radio_value(&mut inputs.path_timed, true, tr("ui-path-timed"));
        // Poses at times are timed already.
        ui.add_enabled(!inputs.path_timed, Checkbox::new(&mut inputs.time_optimal, tr("ui-path-time-optimal")));
        run_button(ui, model, "ui-path-run", Analysis::Path, actions);
        if !tables.is_empty() && ui.button(tr("ui-clear")).clicked() {
            actions.push(Action::Clear(Analysis::Path));
        }
    });
    if let Some((across, up, points)) = model.path_plot() {
        heading(ui, model, "ui-path-planned", Analysis::Path);
        let plot = Plot::new("path-planned").height(200.0).x_axis_label(across).y_axis_label(up);
        // Two axes of the machine are drawn to the same scale, so that a circle is one.
        let plot = if model.session.axes.len() > 1 { plot.data_aspect(1.0) } else { plot };
        plot.show(ui, |plot| plot.line(Line::new(tr("ui-path-planned"), points).color(PALETTE[1]).width(1.8)));
    }
    for (rows, key) in tables.iter().zip(["ui-path-result", "ui-analysis-motors", "ui-analysis-verdicts"]) {
        heading(ui, model, key, Analysis::Path);
        table(ui, &format!("path-{key}"), rows, None);
    }
}

/// A drive trace from a file, replayed through the simulator, and what it says the config should be.
fn trace(ui: &mut Ui, model: &Model, inputs: &mut Inputs, actions: &mut Vec<Action>) {
    let tr = |key: &str| model.tr.tr(key).to_string();
    let tables = model.trace_tables();
    ui.horizontal_wrapped(|ui| {
        ui.label(tr("ui-path-file"));
        ui.add(TextEdit::singleline(&mut inputs.trace_file).desired_width(260.0));
        ui.label(tr("ui-trace-map")).on_hover_text(tr("ui-trace-map-hint"));
        ui.add(TextEdit::singleline(&mut inputs.trace_map).desired_width(200.0));
        run_button(ui, model, "ui-trace-run", Analysis::Trace, actions);
        if !tables.is_empty() && ui.button(tr("ui-clear")).clicked() {
            actions.push(Action::Clear(Analysis::Trace));
        }
    });
    for (rows, key) in tables.iter().zip(["ui-trace-replay", "ui-trace-fit", "ui-trace-identify"]) {
        heading(ui, model, key, Analysis::Trace);
        table(ui, key, rows, None);
    }
    for line in model.trace_advice() {
        ui.label(line);
    }
    let overlay = model.overlay();
    if !overlay.is_empty() {
        ui.horizontal_wrapped(|ui| {
            heading(ui, model, "ui-overlay", Analysis::Trace);
            ui.weak(tr("ui-overlay-dashed"));
            if ui.button(tr("ui-export")).clicked() {
                actions.push(Action::ExportOverlay);
            }
        });
    }
    for (key, motors) in overlay {
        ui.strong(tr(key));
        let plot = Plot::new(key).height(150.0).legend(Legend::default()).link_axis("overlay", [true, false]).link_cursor("overlay", [true, false]);
        plot.x_axis_label(tr("ui-time")).show(ui, |plot| {
            for ((motor, [measured, simulated]), color) in model.session.motors.iter().zip(motors).zip(PALETTE.iter().cycle()) {
                // Both carry the motor's name: one legend entry switches both.
                plot.line(Line::new(tr(motor.key), measured).color(*color).width(1.8));
                plot.line(Line::new(tr(motor.key), simulated).color(*color).style(LineStyle::dashed_dense()).width(1.2));
            }
        });
    }
}

/// The operating points of the analysed cycle over the motors' torque-speed curve, and as a table.
fn points(ui: &mut Ui, model: &Model) {
    let tr = |key: &str| model.tr.tr(key).to_string();
    let Some((points, [high, low])) = model.points_plot() else { return };
    heading(ui, model, "ui-points", Analysis::Cycle);
    Plot::new("ui-points").height(220.0).legend(Legend::default()).x_axis_label(tr("ui-points-speed")).y_axis_label(tr("ui-points-torque")).show(ui, |plot| {
        for ((motor, points), color) in model.session.motors.iter().zip(points).zip(PALETTE.iter().cycle()) {
            plot.points(Points::new(tr(motor.key), points).color(*color).radius(1.0));
        }
        for line in [high, low] {
            plot.line(Line::new(tr("ui-points-curve"), line).color(Color32::GRAY).style(LineStyle::dashed_dense()).width(1.5));
        }
    });
    // Thousands of rows: only those in sight are made. Row 0 is the header.
    let Some((count, _)) = model.points_table(0..0) else { return };
    let height = ui.text_style_height(&bevy_egui::egui::TextStyle::Body) + ui.spacing().item_spacing.y;
    ScrollArea::vertical().id_salt("points-table").max_height(160.0).auto_shrink([false, true]).min_scrolled_height(160.0).show_rows(ui, height, count + 1, |ui, rows| {
        if let Some((_, shown)) = model.points_table(rows.start.saturating_sub(1)..rows.end.saturating_sub(1)) {
            let body = if rows.start == 0 { shown } else { shown[1..].to_vec() };
            table(ui, "points-rows", &body, None);
        }
    });
}

/// Frequency on a logarithmic axis: the plot runs over log10(f) and the ticks show f.
fn hertz(mark: GridMark, _: &std::ops::RangeInclusive<f64>) -> String {
    format!("{:.3}", 10f64.powf(mark.value)).trim_end_matches('0').trim_end_matches('.').to_string()
}

/// A phase over log10(f), unwrapped, so that a lag beyond half a turn does not jump to the top of the plot.
fn phase_line(points: &[sim_components::analysis::SweepPoint]) -> Vec<[f64; 2]> {
    let mut last = 0.0;
    points
        .iter()
        .map(|p| {
            let mut phase = p.phase_deg;
            while phase > last + 180.0 {
                phase -= 360.0;
            }
            last = phase;
            [p.frequency.log10(), phase]
        })
        .collect()
}

/// The open-loop frequency response of the mechanics per motor, its resonance, and the belt stiffness fitted
/// to a measured response from a file.
fn sweep(ui: &mut Ui, model: &Model, inputs: &mut Inputs, actions: &mut Vec<Action>) {
    let tr = |key: &str| model.tr.tr(key).to_string();
    let found = model.sweep_table();
    ui.horizontal_wrapped(|ui| {
        ui.label(tr("ui-sweep-from"));
        ui.add(DragValue::new(&mut inputs.sweep_from).speed(0.5).range(0.1..=10000.0));
        ui.label(tr("ui-sweep-to"));
        ui.add(DragValue::new(&mut inputs.sweep_to).speed(1.0).range(0.1..=10000.0));
        ui.label(tr("ui-sweep-count"));
        ui.add(DragValue::new(&mut inputs.sweep_count).range(2..=400));
        ui.label(tr("ui-sweep-amplitude"));
        ui.add(DragValue::new(&mut inputs.sweep_amplitude).speed(0.01).range(1e-4..=100.0));
        run_button(ui, model, "ui-sweep-run", Analysis::Sweep, actions);
        if found.is_some() && ui.button(tr("ui-clear")).clicked() {
            actions.push(Action::Clear(Analysis::Sweep));
        }
    });
    if let (Some(sweeps), Some(rows)) = (&model.sweeps, &found) {
        heading(ui, model, "ui-sweep-gain", Analysis::Sweep);
        Plot::new("sweep-gain").height(160.0).legend(Legend::default()).link_axis("sweep", [true, false]).x_axis_formatter(hertz).show(ui, |plot| {
            for ((motor, points), color) in model.session.motors.iter().zip(sweeps).zip(PALETTE.iter().cycle()) {
                let line: Vec<[f64; 2]> = points.iter().map(|p| [p.frequency.log10(), 20.0 * p.gain.log10()]).collect();
                plot.line(Line::new(tr(motor.key), line).color(*color).width(1.8));
            }
        });
        ui.strong(format!("{} [°], {} [{}]", tr("ui-tuning-phase"), tr("ui-tuning-frequency"), tr("unit-hz")));
        Plot::new("sweep-phase").height(120.0).legend(Legend::default()).link_axis("sweep", [true, false]).x_axis_formatter(hertz).show(ui, |plot| {
            for ((motor, points), color) in model.session.motors.iter().zip(sweeps).zip(PALETTE.iter().cycle()) {
                plot.line(Line::new(tr(motor.key), phase_line(points)).color(*color).width(1.8));
            }
        });
        heading(ui, model, "ui-sweep-found", Analysis::Sweep);
        table(ui, "ui-sweep-found", rows, None);
    }
    ui.separator();
    ui.horizontal_wrapped(|ui| {
        ui.label(tr("ui-sweep-file"));
        ui.add(TextEdit::singleline(&mut inputs.sweep_file).desired_width(260.0));
        ui.label(tr("ui-sweep-motor"));
        for (i, motor) in model.session.motors.iter().enumerate() {
            ui.radio_value(&mut inputs.sweep_motor, i, tr(motor.key));
        }
        run_button(ui, model, "ui-sweep-fit-run", Analysis::SweepFit, actions);
        if model.sweep_fit.is_some() && ui.button(tr("ui-clear")).clicked() {
            actions.push(Action::Clear(Analysis::SweepFit));
        }
    });
    if let Some(rows) = model.sweep_fit_table() {
        heading(ui, model, "ui-sweep-fit", Analysis::SweepFit);
        table(ui, "ui-sweep-fit", &rows, None);
    }
    if let Some(line) = model.sweep_fit_advice() {
        ui.label(line);
    }
}

/// Tuning help: step response and closed-loop frequency response of one axis's position loop,
/// measured on a copy of the machine with the parameters as they are now.
fn tuning(ui: &mut Ui, model: &Model, actions: &mut Vec<Action>) {
    let tr = |key: &str| model.tr.tr(key).to_string();
    {
        ui.horizontal_wrapped(|ui| {
            for (i, axis) in model.session.axes.iter().enumerate() {
                if ui.selectable_label(i == model.tuning_axis, tr(axis.key)).clicked() {
                    actions.push(Action::TuningAxis(i));
                }
            }
            if model.tuning_running() {
                ui.spinner();
            } else if ui.button(tr("ui-tuning-run")).clicked() {
                actions.push(Action::RunTuning);
            }
            if model.tuning.is_some() && ui.button(tr("ui-clear")).clicked() {
                actions.push(Action::CloseTuning);
            }
        });
        let Some(result) = &model.tuning else { return };
        let Some(axis) = model.session.axes.get(result.axis) else { return };
        let (scale, prefix) = display_scale(result.step.size, axis.unit);
        ui.strong(format!("{} · {} [{prefix}{}]", tr(axis.key), tr("ui-tuning-step"), axis.unit));
        ui.horizontal_wrapped(|ui| {
            let values = [
                ("ui-tuning-rise", result.step.rise_time * 1e3, tr("unit-ms")),
                ("ui-tuning-overshoot", result.step.overshoot * 100.0, tr("unit-percent")),
                ("ui-tuning-settling", result.step.settling_time * 1e3, tr("unit-ms")),
                ("ui-tuning-bandwidth", result.bandwidth.unwrap_or(f64::NAN), tr("unit-hz")),
            ];
            for (key, value, unit) in values {
                ui.weak(tr(key));
                ui.monospace(format!("{value:.1} {unit}"));
            }
        });
        Plot::new("tuning-step").height(150.0).x_axis_label(tr("ui-time")).show(ui, |plot| {
            let size = result.step.size * scale;
            let end = result.step.trace.last().map_or(0.0, |p| p.0);
            plot.line(Line::new(tr("quantity-position-setpoint"), vec![[0.0, 0.0], [0.0, size], [end, size]]).style(LineStyle::dashed_dense()).color(Color32::GRAY));
            let points: Vec<[f64; 2]> = result.step.trace.iter().map(|(t, y)| [*t, y * scale]).collect();
            plot.line(Line::new(tr("quantity-position"), points).color(PALETTE[1]).width(1.8));
        });
        ui.strong(format!("{} · {} [{}]", tr(axis.key), tr("ui-tuning-gain"), tr("unit-db")));
        Plot::new("tuning-gain").height(130.0).link_axis("tuning", [true, false]).x_axis_formatter(hertz).show(ui, |plot| {
            let limit = [result.response.first(), result.response.last()].map(|p| [p.map_or(0.0, |p| p.frequency.log10()), -3.0]);
            plot.line(Line::new(format!("-3 {}", tr("unit-db")), limit.to_vec()).style(LineStyle::dashed_dense()).color(Color32::GRAY));
            let points: Vec<[f64; 2]> = result.response.iter().map(|p| [p.frequency.log10(), 20.0 * p.gain.log10()]).collect();
            plot.line(Line::new(tr("ui-tuning-gain"), points).color(PALETTE[0]).width(1.8));
        });
        ui.strong(format!("{} · {} [°], {} [{}]", tr(axis.key), tr("ui-tuning-phase"), tr("ui-tuning-frequency"), tr("unit-hz")));
        Plot::new("tuning-phase").height(110.0).link_axis("tuning", [true, false]).x_axis_formatter(hertz).show(ui, |plot| {
            plot.line(Line::new(tr("ui-tuning-phase"), phase_line(&result.response)).color(PALETTE[2]).width(1.8));
        });
    }
}
