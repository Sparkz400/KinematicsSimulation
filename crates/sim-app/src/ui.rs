//! The panels: machine and language pickers, run controls, view presets, motion commands, effect
//! switches, the parameter panel and the scopes. Everything shown comes from the model and the
//! translation files; no text for the user is written here.

use bevy_egui::egui::{Color32, ComboBox, DragValue, ScrollArea, Slider, Ui};
use egui_plot::{Legend, Line, Plot};
use sim_app::model::Model;
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

/// The scopes: one plot per quantity, one line per axis or motor, all built from the signal list.
pub fn scopes(ui: &mut Ui, model: &Model, actions: &mut Vec<Action>) {
    let tr = |key: &str| model.tr.tr(key).to_string();
    ScrollArea::vertical().show(ui, |ui| {
        ui.collapsing(tr("ui-scopes"), |ui| {
            for (i, scope) in model.scopes.iter().enumerate() {
                let mut visible = scope.visible;
                if ui.checkbox(&mut visible, format!("{} [{}]", tr(&scope.quantity_key), scope.unit)).changed() {
                    actions.push(Action::Scope(i, visible));
                }
            }
        });
        // At most about two thousand points per line, however long the history is.
        let stride = (model.history.len() / 2000).max(1);
        for scope in model.scopes.iter().filter(|s| s.visible) {
            ui.label(format!("{} [{}]", tr(&scope.quantity_key), scope.unit));
            Plot::new((&scope.quantity_key, &scope.unit)).height(150.0).legend(Legend::default()).x_axis_label(tr("ui-time")).show(ui, |plot| {
                for &signal in &scope.signals {
                    let points: Vec<[f64; 2]> = model.history.iter().step_by(stride).map(|row| [row[0], row[signal]]).collect();
                    plot.line(Line::new(tr(&model.session.signals[signal].owner_key), points));
                }
            });
        }
    });
}
