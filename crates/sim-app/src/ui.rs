//! The panels: machine and language pickers, run controls, view presets, motion commands, effect
//! switches, the parameter panel and the scopes. Everything shown comes from the model and the
//! translation files; no text for the user is written here.

use bevy_egui::egui::{Color32, ComboBox, DragValue, ScrollArea, Slider, Ui};
use egui_plot::{Legend, Line, LineStyle, Plot};
use sim_app::model::{Model, display_scale};
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

        if !model.messages.is_empty() {
            ui.separator();
            ui.horizontal(|ui| {
                ui.label(tr("ui-messages"));
                if ui.button(tr("ui-clear")).clicked() {
                    actions.push(Action::ClearMessages);
                }
            });
            for (key, detail) in &model.messages {
                ui.colored_label(Color32::from_rgb(230, 150, 40), model.tr.tr_arg(key, detail));
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
        tuning(ui, model, actions);

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

/// Tuning help: step response and closed-loop frequency response of one axis's position loop,
/// measured on a copy of the machine with the parameters as they are now.
fn tuning(ui: &mut Ui, model: &Model, actions: &mut Vec<Action>) {
    let tr = |key: &str| model.tr.tr(key).to_string();
    {
        ui.horizontal(|ui| {
            ui.label(tr("ui-tuning"));
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
