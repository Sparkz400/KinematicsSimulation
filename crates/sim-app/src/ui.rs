//! The panels: machine and language pickers, run controls, view presets, motion commands, effect
//! switches, the parameter panel, the scopes, the tuning help and the analysis tables. Everything shown comes from the model and the
//! translation files; no text for the user is written here.

use bevy_egui::egui::{Checkbox, CollapsingHeader, Color32, ComboBox, DragValue, Grid, ScrollArea, Slider, TextEdit, Ui};
use egui_plot::{Legend, Line, LineStyle, Plot};
use sim_app::model::{Analysis, Inputs, Model, SUPPLIES, Table, display_scale};
use sim_components::Command;
use sim_core::MachineEntry;
use sim_machines::{LANGUAGES, MACHINES};
use toml::Value;

/// What the user did this frame. Collected while drawing, applied afterwards.
pub enum Action {
    Machine(&'static MachineEntry),
    Language(&'static str),
    Pause(bool),
    Speed(f64),
    /// Camera yaw and pitch in degrees.
    View(f32, f32),
    Target(usize, f64),
    Command(Command),
    Param(String, Value),
    Scope(usize, bool),
    /// Seconds of history the scopes show.
    Window(f64),
    Freeze(bool),
    Export,
    TuningAxis(usize),
    RunTuning,
    CloseTuning,
    /// What the search, path and trace fields hold after an edit.
    Inputs(Inputs),
    RunAnalysis(Analysis),
    ApplyOptimised,
    /// A row of the motor search goes into the config.
    UseCandidate(usize),
    CloseAnalysis,
    Clear(Analysis),
    ClearMessages,
}

pub fn controls(ui: &mut Ui, model: &Model, actions: &mut Vec<Action>) {
    let tr = |key: &str| model.tr.tr(key).to_string();
    ScrollArea::vertical().show(ui, |ui| {
        ComboBox::from_label(tr("ui-machine")).selected_text(tr(model.entry.key)).show_ui(ui, |ui| {
            for entry in MACHINES {
                if ui.selectable_label(entry.id == model.entry.id, tr(entry.key)).clicked() {
                    actions.push(Action::Machine(entry));
                }
            }
        });
        ComboBox::from_label(tr("ui-language")).selected_text(tr(&format!("lang-{}", model.lang))).show_ui(ui, |ui| {
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
        if ui.add(Slider::new(&mut speed, 0.01..=2.0).logarithmic(true).text(tr("ui-speed"))).changed() {
            actions.push(Action::Speed(speed));
        }

        ui.separator();
        ui.horizontal(|ui| {
            ui.label(tr("ui-view"));
            for (key, yaw, pitch) in [("ui-view-front", 0.0, 0.0), ("ui-view-side", 90.0, 0.0), ("ui-view-top", 0.0, 89.0), ("ui-view-iso", 30.0, 20.0)] {
                if ui.button(tr(key)).clicked() {
                    actions.push(Action::View(yaw, pitch));
                }
            }
        });

        ui.separator();
        ui.label(tr("ui-motion"));
        for (i, axis) in model.session.axes.iter().enumerate() {
            let mut value = model.target[i];
            ui.horizontal(|ui| {
                if ui.add(DragValue::new(&mut value).speed(0.005).range(axis.min..=axis.max)).changed() {
                    actions.push(Action::Target(i, value));
                }
                ui.label(format!("{} [{}]", tr(axis.key), axis.unit));
            });
        }
        ui.horizontal(|ui| {
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
        ui.horizontal(|ui| {
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
                if ui.button(tr("ui-clear")).clicked() {
                    actions.push(Action::ClearMessages);
                }
            });
            for (key, detail) in &model.messages {
                ui.colored_label(NOTE, model.tr.tr_arg(key, detail));
            }
        }

        ui.separator();
        ui.collapsing(tr("ui-effects"), |ui| {
            for (path, key, mut on) in model.effects() {
                if ui.checkbox(&mut on, tr(&key)).changed() {
                    actions.push(Action::Param(path, Value::Boolean(on)));
                }
            }
        });
        ui.collapsing(tr("ui-parameters"), |ui| {
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
        });
    });
}

/// Messages, and the mark on a result that is not of the present config.
const NOTE: Color32 = Color32::from_rgb(230, 150, 40);

/// One colour per axis or motor, the same in every scope.
const PALETTE: [Color32; 6] = [
    Color32::from_rgb(235, 110, 70),
    Color32::from_rgb(80, 160, 240),
    Color32::from_rgb(110, 200, 120),
    Color32::from_rgb(230, 190, 70),
    Color32::from_rgb(190, 130, 230),
    Color32::from_rgb(90, 210, 210),
];

fn number(value: f64) -> String {
    format!("{value:.4}")
}

/// The scopes: one plot per quantity with actual values solid and their setpoints dashed, one
/// colour per axis or motor, a shared time axis and cursor, readable units, and what each trace
/// did in the window. All of it is built from the signal list.
pub fn scopes(ui: &mut Ui, model: &Model, actions: &mut Vec<Action>) {
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
    });
    ScrollArea::vertical().show(ui, |ui| {
        ui.collapsing(tr("ui-scopes"), |ui| {
            for (i, scope) in model.scopes.iter().enumerate() {
                let mut visible = scope.visible;
                if ui.checkbox(&mut visible, format!("{} [{}]", tr(&scope.quantity_key), scope.unit)).changed() {
                    actions.push(Action::Scope(i, visible));
                }
            }
        });
        // Everything else folds away, so that every part of the panel can be reached at any window size.
        let has = |kinds: &[Analysis]| kinds.iter().any(|kind| model.has_result(*kind));
        let mut inputs = model.inputs.clone();
        section(ui, "ui-analysis", model, has(&[Analysis::Cycle, Analysis::Sensitivity, Analysis::Optimise]), |ui| analysis(ui, model, &mut inputs, actions));
        section(ui, "ui-search", model, has(&[Analysis::Search]), |ui| search(ui, model, &mut inputs, actions));
        section(ui, "ui-path", model, has(&[Analysis::Path]), |ui| path(ui, model, &mut inputs, actions));
        section(ui, "ui-trace", model, has(&[Analysis::Trace]), |ui| trace(ui, model, &mut inputs, actions));
        section(ui, "ui-tuning", model, model.tuning.is_some(), |ui| tuning(ui, model, actions));
        if inputs != model.inputs {
            actions.push(Action::Inputs(inputs));
        }

        let rows: Vec<&Vec<f64>> = model.visible_rows().collect();
        // At most about three thousand points per line, however long the window is.
        let stride = (rows.len() / 3000).max(1);
        for scope in model.scopes.iter().filter(|s| s.visible) {
            let stats: Vec<_> = scope.lines.iter().map(|l| model.stats(l.signal)).collect();
            let peak = stats.iter().flatten().fold(0f64, |p, s| p.max(s.min.abs()).max(s.max.abs()));
            let (scale, prefix) = display_scale(peak, &scope.unit);
            let unit = format!("{prefix}{}", scope.unit);
            ui.separator();
            ui.horizontal(|ui| {
                ui.strong(format!("{} [{unit}]", tr(&scope.quantity_key)));
                if scope.lines.iter().any(|l| l.reference) {
                    ui.weak(tr("ui-dashed"));
                }
            });
            Plot::new((&scope.quantity_key, &scope.unit))
                .height(190.0)
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
                });
            // What each actual trace did in the window.
            for (line, stats) in scope.lines.iter().zip(&stats).filter(|(l, _)| !l.reference) {
                let Some(s) = stats else { continue };
                ui.horizontal(|ui| {
                    ui.colored_label(color(line.signal), tr(&signals[line.signal].owner_key));
                    let values = [("ui-stat-min", s.min), ("ui-stat-max", s.max), ("ui-stat-span", s.max - s.min), ("ui-stat-rms", s.rms)];
                    for (key, value) in values {
                        ui.weak(tr(key));
                        ui.monospace(number(value * scale));
                    }
                    ui.weak(&unit);
                });
            }
        }
    });
}

/// A part of the panel under a heading that folds it away. It opens by itself when its content arrives, so
/// that a result is seen, and is otherwise as the user left it.
fn section(ui: &mut Ui, key: &str, model: &Model, filled: bool, body: impl FnOnce(&mut Ui)) {
    let id = ui.make_persistent_id(key);
    let was_filled = ui.data_mut(|data| std::mem::replace(data.get_temp_mut_or(id, false), filled));
    CollapsingHeader::new(model.tr.tr(key)).id_salt(key).open((filled && !was_filled).then_some(true)).show(ui, body);
}

/// A table of the model, drawn as it is: the header weak, numbers in digits of one width, words as they
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
                    match (i, j, cell.parse::<f64>()) {
                        (0, _, _) => ui.weak(cell),
                        (_, 0, _) | (_, _, Err(_)) => ui.label(cell),
                        _ => ui.monospace(cell),
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
    if let Some(rows) = &ranking {
        heading(ui, model, "ui-analysis-sensitivity", Analysis::Sensitivity);
        table(ui, "ui-analysis-sensitivity", rows, None);
    }
    if let Some(rows) = &optimised {
        heading(ui, model, "ui-analysis-optimised", Analysis::Optimise);
        table(ui, "ui-analysis-optimised", rows, None);
        if ui.button(tr("ui-analysis-apply")).clicked() {
            actions.push(Action::ApplyOptimised);
        }
    }
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
        run_button(ui, model, "ui-trace-run", Analysis::Trace, actions);
        if !tables.is_empty() && ui.button(tr("ui-clear")).clicked() {
            actions.push(Action::Clear(Analysis::Trace));
        }
    });
    for (rows, key) in tables.iter().zip(["ui-trace-replay", "ui-trace-fit"]) {
        heading(ui, model, key, Analysis::Trace);
        table(ui, key, rows, None);
    }
    for line in model.trace_advice() {
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
        ui.horizontal(|ui| {
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
        // Frequency on a logarithmic axis: the plot runs over log10(f) and the ticks show f.
        let hertz = |mark: egui_plot::GridMark, _: &std::ops::RangeInclusive<f64>| format!("{:.3}", 10f64.powf(mark.value)).trim_end_matches('0').trim_end_matches('.').to_string();
        ui.strong(format!("{} · {} [{}]", tr(axis.key), tr("ui-tuning-gain"), tr("unit-db")));
        Plot::new("tuning-gain").height(130.0).link_axis("tuning", [true, false]).x_axis_formatter(hertz).show(ui, |plot| {
            let limit = [result.response.first(), result.response.last()].map(|p| [p.map_or(0.0, |p| p.frequency.log10()), -3.0]);
            plot.line(Line::new(format!("-3 {}", tr("unit-db")), limit.to_vec()).style(LineStyle::dashed_dense()).color(Color32::GRAY));
            let points: Vec<[f64; 2]> = result.response.iter().map(|p| [p.frequency.log10(), 20.0 * p.gain.log10()]).collect();
            plot.line(Line::new(tr("ui-tuning-gain"), points).color(PALETTE[0]).width(1.8));
        });
        ui.strong(format!("{} · {} [°], {} [{}]", tr(axis.key), tr("ui-tuning-phase"), tr("ui-tuning-frequency"), tr("unit-hz")));
        Plot::new("tuning-phase").height(110.0).link_axis("tuning", [true, false]).x_axis_formatter(hertz).show(ui, |plot| {
            // Unwrapped, so that a lag beyond half a turn does not jump to the top of the plot.
            let mut last = 0.0;
            let points: Vec<[f64; 2]> = result
                .response
                .iter()
                .map(|p| {
                    let mut phase = p.phase_deg;
                    while phase > last + 180.0 {
                        phase -= 360.0;
                    }
                    last = phase;
                    [p.frequency.log10(), phase]
                })
                .collect();
            plot.line(Line::new(tr("ui-tuning-phase"), points).color(PALETTE[2]).width(1.8));
        });
    }
}
