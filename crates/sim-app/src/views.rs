//! The views for seeing a drive's real torque and the tool's real error, drawn from the kept runs (`sim_app::runs`) with
//! the numbers of `sim_components::charts`: the torque-speed chart per motor (its peak, S1, voltage and current limits,
//! the operating points coloured by time and the RMS point, for every shown run); the tool's error normal and tangential
//! to the path with the tolerance and a histogram; each motor's torque split stacked, and feedforward against feedback;
//! the drive's utilisation over time; the spectra of following error and torque with the machine's resonances; and the
//! trace's residual (measured less simulated). No text for the user is written here.

use crate::ui::{ACTUAL, Action, LIMIT, PALETTE, PLAN, chalk, dim, key_row, plot, save_menu, scale_bar, trail_color};
use bevy_egui::egui::{self, Color32, ComboBox, RichText, Ui};
use egui_plot::{Bar, BarChart, HLine, Heatmap, Line, LineStyle, MarkerShape, PlotPoint, Points, VLine};
use sim_app::model::{Model, display_scale};
use sim_app::runs::{Run, plot_line, scaled_speeds};
use sim_components::charts::{self, TorqueSpeed};

/// The marker of each run's operating points, by its place in the store.
const MARKERS: [MarkerShape; 8] = [
    MarkerShape::Circle,
    MarkerShape::Square,
    MarkerShape::Diamond,
    MarkerShape::Cross,
    MarkerShape::Up,
    MarkerShape::Down,
    MarkerShape::Plus,
    MarkerShape::Asterisk,
];

/// The unit of torque, as the runs give it.
const TORQUE: &str = sim_app::runs::Quantity::Torque.unit();

/// How many colours the time scale of the operating points has.
const TIME_STEPS: usize = 8;

/// The most points of one run drawn per chart: a run has one per millisecond.
const MOST_POINTS: usize = 4000;

/// What one run did on one motor's chart: its label, its operating points (speed, torque) in time order and its RMS
/// point (`charts::rms_point`), speeds in the unit shown.
#[derive(Clone)]
pub struct Operating {
    pub run: usize,
    pub label: String,
    pub points: Vec<[f64; 2]>,
    pub rms: [f64; 2],
}

fn color(rgb: [f32; 3]) -> Color32 {
    let [r, g, b] = rgb.map(|c| (c.clamp(0.0, 1.0) * 255.0) as u8);
    Color32::from_rgb(r, g, b)
}

/// How many plots side by side fit the room, each at least `width` wide.
fn per_row(ui: &Ui, width: f32) -> usize {
    ((ui.available_width() / width).floor() as usize).clamp(1, 3)
}

/// Per motor its torque-speed chart (`charts.len()` of them, small multiples): the peak curve dashed, S1 dotted, the
/// voltage limit at the DC link and the drive's current limit in red, and each run's operating points coloured from
/// the start (dark) to the end (light) of the run, with its RMS point as a large marker, each run in its own marker.
/// Speeds in the unit chosen (`Model::speed_label`). `names` per motor.
pub fn torque_speed_grid(
    ui: &mut Ui,
    model: &Model,
    id: &str,
    names: &[String],
    charts: &[TorqueSpeed],
    operating: &[Vec<Operating>],
    actions: &mut Vec<Action>,
) {
    let tr = |key: &str| model.tr.tr(key).to_string();
    key_row(
        ui,
        &[
            (tr("charts-peak"), Color32::GRAY, true),
            (tr("charts-s1"), chalk(), true),
            (tr("charts-voltage-limit"), LIMIT, false),
            (tr("charts-current-limit"), LIMIT, true),
            (tr("charts-rms-point"), chalk(), false),
        ],
    );
    scale_bar(ui, &tr("charts-run-start"), &tr("charts-run-end"), trail_color);
    let columns = per_row(ui, 300.0);
    for (row, chunk) in charts.chunks(columns).enumerate() {
        ui.columns(columns, |cols| {
            for (i, (chart, ui)) in chunk.iter().zip(cols.iter_mut()).enumerate() {
                let k = row * columns + i;
                let runs = operating.get(k).map_or(&[][..], Vec::as_slice);
                let title = format!(
                    "{} · {} {:.0} {}",
                    names.get(k).cloned().unwrap_or_default(),
                    tr("charts-dc-link-at"),
                    chart.dc_voltage,
                    "V"
                );
                ui.label(RichText::new(&title).strong());
                let drawn = plot((id, k))
                    .height(200.0)
                    .x_axis_label(model.speed_label())
                    .y_axis_label(tr("ui-points-torque"))
                    .show(ui, |plot| {
                        for line in &chart.peak {
                            plot.line(
                                Line::new(tr("charts-peak"), line.clone())
                                    .color(Color32::GRAY)
                                    .style(LineStyle::dashed_dense())
                                    .width(1.5),
                            );
                        }
                        for line in &chart.s1 {
                            plot.line(
                                Line::new(tr("charts-s1"), line.clone())
                                    .color(chalk())
                                    .style(LineStyle::dotted_dense())
                                    .width(1.5),
                            );
                        }
                        for line in &chart.voltage {
                            plot.line(Line::new(tr("charts-voltage-limit"), line.clone()).color(LIMIT).width(1.2));
                        }
                        if let Some(limit) = chart.current_limit {
                            for sign in [1.0, -1.0] {
                                plot.hline(
                                    HLine::new(tr("charts-current-limit"), sign * limit)
                                        .color(LIMIT)
                                        .style(LineStyle::dashed_loose()),
                                );
                            }
                        }
                        for run in runs {
                            let shape = MARKERS[run.run % MARKERS.len()];
                            let every = run.points.len().div_ceil(MOST_POINTS).max(1);
                            let n = run.points.len().max(1);
                            for step in 0..TIME_STEPS {
                                let (from, to) = (step * n / TIME_STEPS, (step + 1) * n / TIME_STEPS);
                                let points: Vec<[f64; 2]> = run.points
                                    [from.min(run.points.len())..to.min(run.points.len())]
                                    .iter()
                                    .step_by(every)
                                    .copied()
                                    .collect();
                                let tint = color(trail_color((step as f32 + 0.5) / TIME_STEPS as f32));
                                plot.points(Points::new(&run.label, points).color(tint).shape(shape).radius(1.2));
                            }
                            plot.points(
                                Points::new(format!("{} · {}", tr("charts-rms-point"), run.label), vec![run.rms])
                                    .color(chalk())
                                    .shape(shape)
                                    .filled(true)
                                    .radius(6.0),
                            );
                        }
                    });
                save_menu(
                    &drawn.response,
                    model,
                    &title,
                    &model.speed_label(),
                    || {
                        let mut series = Vec::new();
                        for (key, lines) in [("charts-peak", &chart.peak), ("charts-s1", &chart.s1)]
                            .into_iter()
                            .chain([("charts-voltage-limit", &chart.voltage)])
                        {
                            series.extend(lines.iter().map(|l| (tr(key), l.clone(), false)));
                        }
                        for run in runs {
                            series.push((run.label.clone(), run.points.clone(), true));
                            series.push((format!("{} · {}", tr("charts-rms-point"), run.label), vec![run.rms], true));
                        }
                        series
                    },
                    actions,
                );
            }
        });
    }
}

/// The operating points of a run per motor in the speed unit `rpm` (a factor), with its RMS point; `run` its place.
fn operating_of(run: &Run, place: usize, rpm: f64) -> Vec<Operating> {
    (0..run.motors.len())
        .map(|k| {
            let get = |name: &str| run.column(&format!("motor.{}.{name}", run.motors[k])).unwrap_or(&[]);
            let (speed, torque) = (get("speed"), get("torque"));
            let [w, t] = charts::rms_point(speed, torque);
            Operating {
                run: place,
                label: run.label.clone(),
                points: speed.iter().zip(torque).map(|(w, t)| [w * rpm, *t]).collect(),
                rms: [w * rpm, t],
            }
        })
        .collect()
}

/// The torque-speed chart of the analysed cycle or path (`Model::points_plot`), per motor from its own drive.
pub fn analysis_torque_speed(
    ui: &mut Ui,
    model: &Model,
    id: &str,
    kind: sim_app::model::Analysis,
    actions: &mut Vec<Action>,
) {
    let Some((points, curves)) = model.points_plot(kind) else { return };
    let names: Vec<String> = model.session.motors.iter().map(|m| model.tr.tr(m.key).to_string()).collect();
    let operating: Vec<Vec<Operating>> = points
        .into_iter()
        .map(|points| {
            let (speed, torque): (Vec<f64>, Vec<f64>) = points.iter().map(|p| (p[0], p[1])).unzip();
            vec![Operating {
                run: 0,
                label: model.tr.tr("ui-points").to_string(),
                rms: charts::rms_point(&speed, &torque),
                points,
            }]
        })
        .collect();
    torque_speed_grid(ui, model, id, &names, &curves, &operating, actions);
}

/// The views tab: the run they show (the reference run of the store), and each view in a section of its own.
/// `tolerance` is the path error the 3D view's overlay marks (m), drawn as the error's band.
pub fn views(ui: &mut Ui, model: &Model, tolerance: Option<f64>, actions: &mut Vec<Action>) {
    let tr = |key: &str| model.tr.tr(key).to_string();
    ui.label(RichText::new(tr("ui-views-intro")).color(dim()));
    ui.horizontal_wrapped(|ui| {
        if ui.button(tr("ui-runs-keep-live")).on_hover_text(tr("ui-runs-keep-live-hint")).clicked() {
            actions.push(Action::KeepLive);
        }
        if model.runs.is_empty() {
            return;
        }
        let reference = model.runs.reference();
        ui.label(tr("ui-views-run"));
        ComboBox::from_id_salt("views-run")
            .width(260.0)
            .selected_text(model.runs.get(reference).map_or(String::new(), |r| r.label.clone()))
            .show_ui(ui, |ui| {
                for (i, run) in model.runs.iter().enumerate() {
                    if ui.selectable_label(i == reference, &run.label).clicked() {
                        actions.push(Action::RunReference(i));
                    }
                }
            });
    });
    let Some(run) = model.runs.get(model.runs.reference()) else {
        ui.label(RichText::new(tr("ui-runs-none")).color(dim()));
        return;
    };
    let names: Vec<String> = run
        .motors
        .iter()
        .map(|id| {
            let owner = run.signals.iter().find(|s| s.id == format!("motor.{id}.torque")).map(|s| s.owner_key.clone());
            owner.map_or(id.clone(), |key| model.tr.tr(&key).to_string())
        })
        .collect();
    let section =
        |key: &str, open: bool| egui::CollapsingHeader::new(tr(key)).id_salt(key.to_string()).default_open(open);
    section("charts-torque-speed", true).show(ui, |ui| {
        if run.drives.is_empty() {
            ui.label(RichText::new(tr("ui-views-no-drives")).color(dim()));
            return;
        }
        let rpm = model.units.scale(sim_app::runs::Quantity::Speed.unit()).0;
        let curves: Vec<TorqueSpeed> = run.torque_speed().iter().map(|c| scaled_speeds(c, rpm)).collect();
        // Every shown run's points over the reference run's charts, motor by motor by their place.
        let mut operating: Vec<Vec<Operating>> = vec![Vec::new(); curves.len()];
        for (i, shown) in
            model.runs.iter().enumerate().filter(|(i, _)| model.runs.shown(*i) || *i == model.runs.reference())
        {
            for (k, points) in operating_of(shown, i, rpm).into_iter().enumerate().take(curves.len()) {
                operating[k].push(points);
            }
        }
        torque_speed_grid(ui, model, "views-ts", &names, &curves, &operating, actions);
    });
    section("charts-tool-error", true).show(ui, |ui| tool_error(ui, model, run, tolerance, actions));
    section("charts-torque-split", false).show(ui, |ui| torque_split(ui, model, run, &names, actions));
    section("charts-utilisation", false).show(ui, |ui| utilisation(ui, model, run, &names, actions));
    section("charts-spectrum", false).show(ui, |ui| spectrum(ui, model, run, &names, actions));
}

/// A plot over the run's time with lines (name, colour, style, points); with `bands`, horizontal lines in red at those
/// values. Saved by a right click.
#[allow(clippy::type_complexity)]
fn time_plot(
    ui: &mut Ui,
    model: &Model,
    id: impl std::hash::Hash + std::fmt::Debug + Copy,
    title: &str,
    lines: &[(String, Color32, LineStyle, Vec<[f64; 2]>)],
    bands: &[f64],
    actions: &mut Vec<Action>,
) {
    let tr = |key: &str| model.tr.tr(key).to_string();
    ui.label(RichText::new(title).strong());
    key_row(ui, &lines.iter().map(|(n, c, s, _)| (n.clone(), *c, *s != LineStyle::Solid)).collect::<Vec<_>>());
    let drawn = plot(id)
        .height(150.0)
        .link_axis("views", [true, false])
        .link_cursor("views", [true, false])
        .x_axis_label(tr("ui-time"))
        .show(ui, |plot| {
            for (name, c, style, points) in lines {
                plot.line(Line::new(name, points.clone()).color(*c).style(*style).width(1.6));
            }
            for band in bands {
                plot.hline(HLine::new("", *band).color(LIMIT).style(LineStyle::dashed_loose()));
            }
        });
    save_menu(
        &drawn.response,
        model,
        title,
        &tr("ui-time"),
        || lines.iter().map(|(n, _, _, p)| (n.clone(), p.clone(), false)).collect(),
        actions,
    );
}

/// The tool's error over the run, normal (across the path) and tangential (along it, positive lagging) in m, with the
/// distance to the path the analysis measured where the run has it; the tolerance band; the angle error apart; and a
/// histogram of the normal error.
fn tool_error(ui: &mut Ui, model: &Model, run: &Run, tolerance: Option<f64>, actions: &mut Vec<Action>) {
    let tr = |key: &str| model.tr.tr(key).to_string();
    let Some(error) = charts::tool_error(&run.signals, &run.columns) else {
        ui.label(RichText::new(tr("ui-views-no-error")).color(dim()));
        return;
    };
    let time = &error.time;
    if !error.normal.is_empty() {
        let peak = error.normal.iter().chain(&error.tangential).fold(0f64, |p, v| p.max(v.abs()));
        let (scale, prefix) = display_scale(peak.max(tolerance.unwrap_or(0.0)), "m");
        let line = |values: &[f64]| plot_line(values.len(), |n| [time[n], values[n] * scale]);
        let mut lines = vec![
            (tr("charts-normal"), ACTUAL, LineStyle::Solid, line(&error.normal)),
            (tr("charts-tangential"), PLAN, LineStyle::Solid, line(&error.tangential)),
        ];
        if !run.off_path.is_empty() {
            let off: Vec<[f64; 2]> =
                plot_line(run.off_path.len(), |n| [run.off_path[n][0], run.off_path[n][1] * scale]);
            lines.push((tr("charts-off-path"), chalk(), LineStyle::dotted_dense(), off));
        }
        let bands: Vec<f64> = tolerance.map_or(Vec::new(), |t| vec![t * scale, -t * scale]);
        time_plot(
            ui,
            model,
            "views-error",
            &format!("{} [{prefix}{}]", tr("charts-tool-error"), "m"),
            &lines,
            &bands,
            actions,
        );
        // How often each size of normal error occurred, the tolerance marked.
        let bins = charts::histogram(&error.normal, 40);
        let bars: Vec<Bar> = bins
            .iter()
            .map(|(low, high, n)| Bar::new((low + high) / 2.0 * scale, *n as f64).width((high - low) * scale))
            .collect();
        let title = format!("{} [{prefix}{}]", tr("charts-histogram"), "m");
        ui.label(RichText::new(&title).strong());
        let drawn = plot("views-histogram")
            .height(110.0)
            .x_axis_label(format!("{} [{prefix}{}]", tr("charts-normal"), "m"))
            .show(ui, |plot| {
                plot.bar_chart(BarChart::new(tr("charts-normal"), bars).color(ACTUAL));
                if let Some(t) = tolerance {
                    plot.vline(VLine::new("", t * scale).color(LIMIT).style(LineStyle::dashed_loose()));
                }
            });
        save_menu(
            &drawn.response,
            model,
            &title,
            &tr("charts-normal"),
            || {
                vec![(
                    tr("charts-normal"),
                    bins.iter().map(|(l, h, n)| [(l + h) / 2.0 * scale, *n as f64]).collect(),
                    false,
                )]
            },
            actions,
        );
    }
    if !error.angle.is_empty() {
        let peak = error.angle.iter().fold(0f64, |p, v| p.max(*v));
        let (scale, prefix) = display_scale(peak, "rad");
        let lines = [(
            tr("charts-angle-error"),
            ACTUAL,
            LineStyle::Solid,
            plot_line(time.len(), |n| [time[n], error.angle[n] * scale]),
        )];
        time_plot(
            ui,
            model,
            "views-angle",
            &format!("{} [{prefix}{}]", tr("charts-angle-error"), "rad"),
            &lines,
            &[],
            actions,
        );
    }
}

/// Per motor its torque's parts stacked (`charts::stack`; the last line is the motor torque), and the velocity loop's
/// feedforward against its feedback (the PI part) with the torque.
fn torque_split(ui: &mut Ui, model: &Model, run: &Run, names: &[String], actions: &mut Vec<Action>) {
    let tr = |key: &str| model.tr.tr(key).to_string();
    let time = run.time();
    let line = |values: &[f64]| plot_line(values.len().min(time.len()), |n| [time[n], values[n]]);
    let mut none = true;
    for (k, motor) in run.motors.iter().enumerate() {
        let name = names.get(k).cloned().unwrap_or_default();
        let parts = charts::torque_parts(&run.signals, &run.columns, motor);
        if !parts.is_empty() {
            none = false;
            let stacked = charts::stack(&parts.iter().map(|(_, v)| *v).collect::<Vec<_>>());
            let mut label = String::new();
            let lines: Vec<_> = parts
                .iter()
                .zip(&stacked)
                .enumerate()
                .map(|(i, ((part, _), values))| {
                    label = if label.is_empty() {
                        tr(&format!("quantity-torque-{part}"))
                    } else {
                        format!("+ {}", tr(&format!("quantity-torque-{part}")))
                    };
                    (label.clone(), PALETTE[i % PALETTE.len()], LineStyle::Solid, line(values))
                })
                .collect();
            time_plot(
                ui,
                model,
                ("views-split", k),
                &format!("{name} · {} [{TORQUE}]", tr("charts-torque-split")),
                &lines,
                &[],
                actions,
            );
        }
        let get = |signal: &str| run.column(&format!("motor.{motor}.{signal}"));
        if let (Some(ff), Some(fb), Some(torque)) = (get("torque_feedforward"), get("torque_feedback"), get("torque")) {
            none = false;
            let lines = [
                (tr("charts-feedforward"), PLAN, LineStyle::Solid, line(ff)),
                (tr("charts-feedback"), ACTUAL, LineStyle::Solid, line(fb)),
                (tr("quantity-torque"), chalk(), LineStyle::dashed_dense(), line(torque)),
            ];
            time_plot(
                ui,
                model,
                ("views-ff", k),
                &format!("{name} · {} [{TORQUE}]", tr("charts-feedforward-feedback")),
                &lines,
                &[],
                actions,
            );
        }
    }
    if none {
        ui.label(RichText::new(tr("ui-views-no-split")).color(dim()));
    }
}

/// Per motor its utilisation over the run (`charts::utilisation`) in % of each limit, 100 % in red, the DC link's
/// braking threshold dotted.
fn utilisation(ui: &mut Ui, model: &Model, run: &Run, names: &[String], actions: &mut Vec<Action>) {
    let tr = |key: &str| model.tr.tr(key).to_string();
    if run.drives.is_empty() {
        ui.label(RichText::new(tr("ui-views-no-drives")).color(dim()));
        return;
    }
    let time = run.time();
    for (k, (motor, drive)) in run.motors.iter().zip(&run.drives).enumerate() {
        let shares = charts::utilisation(drive, &run.signals, &run.columns, motor);
        let lines: Vec<_> = shares
            .iter()
            .enumerate()
            .map(|(i, (key, values))| {
                let points = plot_line(values.len().min(time.len()), |n| [time[n], (values[n] * 100.0).min(1e4)]);
                (tr(key), PALETTE[i % PALETTE.len()], LineStyle::Solid, points)
            })
            .collect();
        let mut bands = vec![100.0];
        bands.extend(charts::braking_share(drive).map(|s| s * 100.0));
        let name = names.get(k).cloned().unwrap_or_default();
        time_plot(
            ui,
            model,
            ("views-use", k),
            &format!("{name} · {} [%]", tr("charts-utilisation")),
            &lines,
            &bands,
            actions,
        );
    }
}

/// The spectra of each motor's following error and torque over the run (`charts::spectrum`, amplitudes on a log
/// scale) with the machine's resonances (`Run::modes`) as amber lines, and per motor the following error's spectrogram.
fn spectrum(ui: &mut Ui, model: &Model, run: &Run, names: &[String], actions: &mut Vec<Action>) {
    let tr = |key: &str| model.tr.tr(key).to_string();
    let time = run.time();
    let dt = if time.len() > 1 { (time[time.len() - 1] - time[0]) / (time.len() - 1) as f64 } else { 1e-3 };
    if !run.modes.is_empty() {
        key_row(ui, &[(tr("charts-modes"), PLAN, true)]);
    }
    for (signal, key) in [("following_error", "quantity-following-error"), ("torque", "quantity-torque")] {
        let lines: Vec<(String, Vec<[f64; 2]>)> = run
            .motors
            .iter()
            .enumerate()
            .filter_map(|(k, m)| {
                let values = run.column(&format!("motor.{m}.{signal}"))?;
                let s = charts::spectrum(values, dt);
                let points = s.iter().skip(1).map(|[f, a]| [*f, a.max(1e-12).log10()]).collect();
                Some((names.get(k).cloned().unwrap_or_default(), points))
            })
            .collect();
        if lines.is_empty() {
            continue;
        }
        let title = format!("{} · {}", tr(key), tr("charts-spectrum"));
        ui.label(RichText::new(&title).strong());
        let drawn = plot(("views-fft", signal))
            .height(150.0)
            .x_axis_label(tr("charts-frequency"))
            .y_axis_label(tr("charts-log-amplitude"))
            .show(ui, |plot| {
                for (k, (name, points)) in lines.iter().enumerate() {
                    plot.line(Line::new(name, points.clone()).color(PALETTE[k % PALETTE.len()]).width(1.4));
                }
                for hz in &run.modes {
                    plot.vline(VLine::new(tr("charts-modes"), *hz).color(PLAN).style(LineStyle::dashed_dense()));
                }
            });
        save_menu(
            &drawn.response,
            model,
            &title,
            &tr("charts-frequency"),
            || lines.iter().map(|(n, p)| (n.clone(), p.clone(), false)).collect(),
            actions,
        );
    }
    // The spectrogram: frames of 256 ms every 128 ms, at most `COLUMNS` of them (the newest), up to `ROWS` bins.
    const COLUMNS: usize = 120;
    const ROWS: usize = 128;
    for (k, m) in run.motors.iter().enumerate() {
        let Some(values) = run.column(&format!("motor.{m}.following_error")) else { continue };
        let g = charts::spectrogram(values, dt, 0.0, 256);
        let frames = &g.frames[g.frames.len().saturating_sub(COLUMNS)..];
        let times = &g.times[g.times.len().saturating_sub(COLUMNS)..];
        if frames.len() < 2 {
            continue;
        }
        let rows = ROWS.min(frames[0].len());
        let top = frames.iter().flatten().fold(1e-30f64, |p, v| p.max(*v)).log10();
        // Row by row from the lowest bin; each tile the frame's amplitude, 4 decades below the highest dark.
        let values: Vec<f64> =
            (0..rows).flat_map(|bin| frames.iter().map(move |f| f[bin].max(1e-30).log10())).collect();
        let hop = if times.len() > 1 { times[1] - times[0] } else { 0.128 };
        let title = format!(
            "{} · {} · {}",
            names.get(k).cloned().unwrap_or_default(),
            tr("quantity-following-error"),
            tr("charts-spectrogram")
        );
        ui.label(RichText::new(&title).strong());
        plot(("views-spectrogram", k))
            .height(140.0)
            .x_axis_label(tr("ui-time"))
            .y_axis_label(tr("charts-frequency"))
            .show(ui, |plot| {
                let heat = Heatmap::new(values, frames.len())
                    .show_labels(false)
                    .at(PlotPoint::new(times[0] - hop / 2.0, -g.bin_hz / 2.0))
                    .tile_size(hop as f32, g.bin_hz as f32)
                    .custom_mapping(Box::new(move |v| color(trail_color(((v - top + 4.0) / 4.0) as f32))));
                plot.heatmap(heat);
                for hz in run.modes.iter().filter(|hz| **hz < rows as f64 * g.bin_hz) {
                    plot.hline(HLine::new(tr("charts-modes"), *hz).color(PLAN).style(LineStyle::dashed_dense()));
                }
            });
    }
}

/// The trace's residual, measured less simulated, per motor for the torque and the following error, on the same time
/// axis and cursor as the overlay above it (`ui::trace`).
pub fn trace_residual(ui: &mut Ui, model: &Model, actions: &mut Vec<Action>) {
    let tr = |key: &str| model.tr.tr(key).to_string();
    let Some(fit) = &model.trace_fit else { return };
    let (measured, simulated) = (&fit.measured, &fit.simulated);
    // shortcut: sample by sample, as the replay runs on the trace's own time stamps; align by time if a replay ever
    // samples differently.
    let n = measured.time.len().min(simulated.time.len());
    let time = &measured.time[..n];
    for (key, unit, of) in [
        ("quantity-torque", TORQUE, (|m: &sim_components::trace::MotorTrace| m.torque.clone()) as fn(&_) -> Vec<f64>),
        ("quantity-following-error", "rad", |m| m.following_error()),
    ] {
        let lines: Vec<(String, Color32, LineStyle, Vec<[f64; 2]>)> = model
            .session
            .motors
            .iter()
            .zip(measured.motors.iter().zip(&simulated.motors))
            .enumerate()
            .map(|(k, (motor, (a, b)))| {
                let (a, b) = (of(a), of(b));
                let residual: Vec<f64> = a.iter().zip(&b).take(n).map(|(a, b)| a - b).collect();
                let points = plot_line(residual.len(), |i| [time[i], residual[i]]);
                (tr(motor.key), PALETTE[k % PALETTE.len()], LineStyle::Solid, points)
            })
            .collect();
        let title = format!("{} · {} [{unit}]", tr(key), tr("charts-residual"));
        ui.label(RichText::new(&title).strong());
        let drawn = plot(("trace-residual", key))
            .height(120.0)
            .link_axis("overlay", [true, false])
            .link_cursor("overlay", [true, false])
            .x_axis_label(tr("ui-time"))
            .show(ui, |plot| {
                for (name, c, _, points) in &lines {
                    plot.line(Line::new(name, points.clone()).color(*c).width(1.4));
                }
                plot.hline(HLine::new("", 0.0).color(dim()));
            });
        save_menu(
            &drawn.response,
            model,
            &title,
            &tr("ui-time"),
            || lines.iter().map(|(n, _, _, p)| (n.clone(), p.clone(), false)).collect(),
            actions,
        );
    }
}
