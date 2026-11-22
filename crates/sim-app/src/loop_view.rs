//! The loop-response tab: per motor the Bode plot of its velocity loop (open loop L, closed loop T = L / (1 + L), and
//! the plant from the motor's torque to its speed), the margins marked on it, the machine's resonances and the input
//! shaper's and notch filter's frequencies as vertical lines, and the Nyquist plot of L around the critical point.
//! The measurement is `sim_app::frequency`'s, in the background with progress and a cancel button.

use crate::ui::{Action, LIMIT, PALETTE, PASS, PLAN, hertz, key_row, plot};
use bevy_egui::egui::{self, Color32, RichText, Ui};
use egui_plot::{HLine, Line, LineStyle, MarkerShape, PlotUi, Points, VLine};
use sim_app::frequency::{Response, closed_loop, phase_crossover, phases};
use sim_app::model::Model;
use sim_components::analysis::LoopPoint;

/// What the tab asks of the model.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LoopAction {
    Run,
    Cancel,
    Motor(usize),
    Clear,
}

pub fn apply(model: &mut Model, action: LoopAction) {
    match action {
        LoopAction::Run => model.loop_response.start(model.entry, &model.config),
        LoopAction::Cancel => model.loop_response.cancel(),
        LoopAction::Motor(motor) => model.loop_response.motor = motor,
        LoopAction::Clear => model.loop_response.result = None,
    }
}

const OPEN: Color32 = PALETTE[0];
const CLOSED: Color32 = PALETTE[2];
const PLANT: Color32 = PALETTE[3];
const NOTCH: Color32 = PALETTE[1];
const MODE: Color32 = Color32::GRAY;

/// The Nyquist plot shows L where |L| is at most this: the neighbourhood of -1, where stability is decided.
const NYQUIST_REACH: f64 = 4.0;

pub fn loop_response(ui: &mut Ui, model: &Model, actions: &mut Vec<Action>) {
    let tr = |key: &str| model.tr.tr(key).to_string();
    let state = &model.loop_response;
    ui.horizontal_wrapped(|ui| {
        for (i, motor) in model.session.motors.iter().enumerate() {
            if ui.selectable_label(i == state.motor, tr(motor.key)).clicked() {
                actions.push(Action::Loop(LoopAction::Motor(i)));
            }
        }
        if let Some((done, total)) = state.progress() {
            ui.spinner();
            let share = if total > 0 { done as f32 / total as f32 } else { 0.0 };
            ui.add(egui::ProgressBar::new(share).desired_width(120.0).text(format!("{done} / {total}")));
            if ui.button(tr("ui-loop-cancel")).clicked() {
                actions.push(Action::Loop(LoopAction::Cancel));
            }
        } else if ui.button(tr("ui-loop-run")).on_hover_text(tr("ui-loop-run-hint")).clicked() {
            actions.push(Action::Loop(LoopAction::Run));
        }
        if state.result.is_some() && ui.button(tr("ui-clear")).clicked() {
            actions.push(Action::Loop(LoopAction::Clear));
        }
    });
    let Some((config, response)) = &state.result else { return };
    if *config != model.config {
        ui.label(RichText::new(tr("ui-analysis-outdated")).color(PLAN));
    }
    let (Some(points), Some(m)) = (response.curves.get(state.motor), response.margins.get(state.motor)) else {
        return;
    };
    let colour = if m.stable() { PASS } else { LIMIT };
    ui.horizontal_wrapped(|ui| {
        let values = [
            ("ui-loop-crossover", m.crossover_hz, tr("unit-hz")),
            ("ui-loop-phase-margin", m.phase_margin_deg, "°".to_string()),
            ("ui-loop-gain-margin", m.gain_margin_db, tr("unit-db")),
            ("ui-loop-peak", m.peak_sensitivity, String::new()),
        ];
        for (key, value, unit) in values {
            ui.weak(tr(key));
            if value.is_nan() && m.valid {
                // No gain crossover in the band: nothing to measure, the verdict is the gain margin's alone.
                ui.label(RichText::new("–").monospace().color(colour)).on_hover_text(tr("ui-loop-no-crossover"));
            } else {
                ui.label(RichText::new(format!("{value:.1} {unit}")).monospace().color(colour));
            }
        }
        ui.weak(format!("{} {}", points.len(), tr("ui-loop-points")));
    });
    if let Some(h) = m.hammering {
        let mut at = format!("{:.0} {}, {:.2} N m", h.frequency_hz, tr("unit-hz"), h.torque_nm);
        // Away from the start pose the curves are not: the cycle's pose it hammers at (`margin_poses`).
        if let Some(pose) = sim_components::analysis::margin_poses(config).ok().and_then(|p| p.get(h.pose).cloned())
            && h.pose > 0
        {
            at += &format!(", start.pose = {pose:?}");
        }
        ui.label(RichText::new(model.tr.tr_arg("loop-hammers", &at)).color(LIMIT));
    } else if !m.stable() {
        ui.label(RichText::new(tr("ui-loop-unstable")).color(LIMIT));
    }
    let mut keys = vec![
        (tr("ui-loop-open"), OPEN, false),
        (tr("ui-loop-closed"), CLOSED, false),
        (tr("ui-loop-plant"), PLANT, false),
        (tr("ui-loop-mode"), MODE, true),
    ];
    for (key, colour) in [("ui-loop-shaper", PLAN), ("ui-loop-notch", NOTCH)] {
        if response.markers.iter().any(|(k, _)| *k == key) {
            keys.push((tr(key), colour, true));
        }
    }
    key_row(ui, &keys);
    let at = |hz: f64| hz.log10();
    let crossing = phase_crossover(points);
    // Magnitude: L and T in dB, the plant in dB of 1 rad/s per N m.
    ui.strong(format!("{} [{}]", tr("ui-loop-magnitude"), tr("unit-db")));
    plot(("loop-gain", state.motor)).height(160.0).link_axis("loop", [true, false]).x_axis_formatter(hertz).show(
        ui,
        |plot| {
            markers(plot, model, response);
            plot.hline(HLine::new("0 dB", 0.0).color(MODE).style(LineStyle::dashed_dense()));
            let db = |[re, im]: [f64; 2]| 20.0 * re.hypot(im).log10();
            let line = |value: fn(&LoopPoint) -> [f64; 2]| -> Vec<[f64; 2]> {
                points.iter().map(|p| [at(p.frequency), db(value(p))]).collect()
            };
            plot.line(Line::new(tr("ui-loop-open"), line(|p| [p.re, p.im])).color(OPEN).width(1.8));
            plot.line(Line::new(tr("ui-loop-closed"), line(closed_loop)).color(CLOSED).width(1.8));
            plot.line(Line::new(tr("ui-loop-plant"), line(|p| p.plant)).color(PLANT).width(1.4));
            if let Some(f) = crossing.filter(|_| m.gain_margin_db.is_finite()) {
                let gm = format!("{} {:.1} {}", tr("ui-loop-gain-margin"), m.gain_margin_db, tr("unit-db"));
                plot.vline(VLine::new(gm.clone(), at(f)).color(colour));
                plot.points(Points::new(gm, vec![[at(f), -m.gain_margin_db]]).color(colour).radius(5.0));
            }
        },
    );
    ui.strong(format!("{} [°], {} [{}]", tr("ui-loop-phase"), tr("ui-tuning-frequency"), tr("unit-hz")));
    plot(("loop-phase", state.motor)).height(160.0).link_axis("loop", [true, false]).x_axis_formatter(hertz).show(
        ui,
        |plot| {
            markers(plot, model, response);
            plot.hline(HLine::new("-180°", -180.0).color(MODE).style(LineStyle::dashed_dense()));
            let line = |values: Vec<f64>| -> Vec<[f64; 2]> {
                points.iter().zip(values).map(|(p, phase)| [at(p.frequency), phase]).collect()
            };
            let open = line(phases(points.iter().map(|p| [p.re, p.im])));
            plot.line(Line::new(tr("ui-loop-open"), open).color(OPEN).width(1.8));
            plot.line(Line::new(tr("ui-loop-closed"), line(phases(points.iter().map(closed_loop)))).color(CLOSED));
            plot.line(Line::new(tr("ui-loop-plant"), line(phases(points.iter().map(|p| p.plant)))).color(PLANT));
            if m.crossover_hz.is_finite() {
                let pm = format!("{} {:.1}°", tr("ui-loop-phase-margin"), m.phase_margin_deg);
                plot.vline(VLine::new(pm.clone(), at(m.crossover_hz)).color(colour));
                let phase = -180.0 + m.phase_margin_deg;
                plot.points(Points::new(pm, vec![[at(m.crossover_hz), phase]]).color(colour).radius(5.0));
            }
        },
    );
    nyquist(ui, model, points);
}

/// The resonances (dotted), the shaper's and the notch filter's frequencies (dashed) as vertical lines.
fn markers(plot: &mut PlotUi, model: &Model, response: &Response) {
    let tr = |key: &str| model.tr.tr(key).to_string();
    for hz in &response.modes {
        let name = format!("{} {hz:.1} {}", tr("ui-loop-mode"), tr("unit-hz"));
        plot.vline(VLine::new(name, hz.log10()).color(MODE).style(LineStyle::dotted_dense()));
    }
    for (key, hz) in &response.markers {
        let colour = if *key == "ui-loop-shaper" { PLAN } else { NOTCH };
        let name = format!("{} {hz:.1} {}", tr(key), tr("unit-hz"));
        plot.vline(VLine::new(name, hz.log10()).color(colour).style(LineStyle::dashed_dense()));
    }
}

/// L in the complex plane where |L| <= `NYQUIST_REACH`, the critical point -1 and the unit circle (where L crosses it
/// the angle to -1 is the phase margin).
fn nyquist(ui: &mut Ui, model: &Model, points: &[LoopPoint]) {
    let tr = |key: &str| model.tr.tr(key).to_string();
    ui.strong(tr("ui-loop-nyquist"));
    key_row(ui, &[(tr("ui-loop-critical"), LIMIT, false), (tr("ui-loop-unit-circle"), MODE, true)]);
    plot(("loop-nyquist", model.loop_response.motor)).height(260.0).data_aspect(1.0).show(ui, |plot| {
        let circle: Vec<[f64; 2]> =
            (0..=120).map(|i| (i as f64 * std::f64::consts::TAU / 120.0).sin_cos()).map(|(s, c)| [c, s]).collect();
        plot.line(Line::new(tr("ui-loop-unit-circle"), circle).color(MODE).style(LineStyle::dashed_dense()));
        // The curve in pieces: a stretch beyond the reach (the integrators' large |L| at low frequency) is left out.
        let mut piece: Vec<[f64; 2]> = Vec::new();
        for p in points.iter().chain(std::iter::once(&LoopPoint { re: f64::NAN, ..points[0] })) {
            if p.re.hypot(p.im) <= NYQUIST_REACH {
                piece.push([p.re, p.im]);
            } else if !piece.is_empty() {
                plot.line(Line::new(tr("ui-loop-open"), std::mem::take(&mut piece)).color(OPEN).width(1.8));
            }
        }
        plot.points(
            Points::new(tr("ui-loop-critical"), vec![[-1.0, 0.0]]).color(LIMIT).shape(MarkerShape::Cross).radius(7.0),
        );
    });
}
