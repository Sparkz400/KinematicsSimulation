//! The search tab (every option of `sim-cli search`, the ranked table, the heat map of the checks, cost against margin
//! with the front, the top candidates' operating points, why each failed, the ratio sweep and the kinematics
//! comparison) and the path tab's maps of peak torque and following error. What they show comes from
//! `sim_app::search_views`; the grid painter is the workspace map's as well.

use super::*;
use egui_plot::MarkerShape;
use sim_app::search_views::{ViewJob, failure_line, failures, heatmap, scatter, vendors};
use sim_components::catalog::Drives;

/// A square map of `n` × `n` cells over the plane from `x` to `y` (low, high each): node (i, j) at the middle of its
/// cell, y upwards as in the plots.
pub(super) struct GridMap {
    pub rect: Rect,
    pub response: egui::Response,
    x: (f64, f64),
    y: (f64, f64),
    n: usize,
}

impl GridMap {
    /// Draws the map `side` pixels wide on the inset ground, cell k (row after row, j outer) in the colour `color`
    /// gives it, and nothing where it gives none.
    pub fn draw(
        ui: &mut Ui,
        side: f32,
        (x, y): ((f64, f64), (f64, f64)),
        n: usize,
        color: impl Fn(usize) -> Option<Color32>,
    ) -> Self {
        let (rect, response) = ui.allocate_exact_size(vec2(side, side), Sense::hover());
        let map = Self { rect, response, x, y, n };
        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 0.0, inset());
        let cell = side / n as f32;
        for k in 0..n * n {
            if let Some(color) = color(k) {
                let centre = map.place(map.at(x, k % n), map.at(y, k / n));
                painter.rect_filled(Rect::from_center_size(centre, vec2(cell + 0.5, cell + 0.5)), 0.0, color);
            }
        }
        map
    }

    /// The value of node i along a side from low to high.
    pub fn at(&self, (low, high): (f64, f64), i: usize) -> f64 {
        low + (high - low) * i as f64 / (self.n - 1) as f64
    }

    /// Where a point of the plane is on the screen.
    pub fn place(&self, x: f64, y: f64) -> egui::Pos2 {
        let (side, cell) = (self.rect.width(), self.rect.width() / self.n as f32);
        let (fx, fy) = ((x - self.x.0) / (self.x.1 - self.x.0), (y - self.y.0) / (self.y.1 - self.y.0));
        egui::pos2(
            self.rect.left() + cell / 2.0 + (side - cell) * fx as f32,
            self.rect.bottom() - cell / 2.0 - (side - cell) * fy as f32,
        )
    }

    /// The node (i, j) under the pointer, where it is over the map.
    pub fn hovered(&self) -> Option<(usize, usize)> {
        let pointer = self.response.hover_pos()?;
        let side = self.rect.width();
        let (fx, fy) =
            (((pointer.x - self.rect.left()) / side) as f64, ((self.rect.bottom() - pointer.y) / side) as f64);
        let n = self.n;
        Some((((fx * n as f64) as usize).min(n - 1), ((fy * n as f64) as usize).min(n - 1)))
    }

    /// The span of each side, written in its corners.
    pub fn label_axes(&self, ui: &Ui, across: String, up: String) {
        let painter = ui.painter_at(self.rect);
        let label = |pos: egui::Pos2, align: egui::Align2, text: String| {
            painter.text(pos, align, text, FontId::proportional(11.0), dim())
        };
        label(
            self.rect.left_bottom() + vec2(2.0, -2.0),
            egui::Align2::LEFT_BOTTOM,
            format!("{across} {} … {}", short(self.x.0), short(self.x.1)),
        );
        label(
            self.rect.left_top() + vec2(2.0, 2.0),
            egui::Align2::LEFT_TOP,
            format!("{up} {} … {}", short(self.y.0), short(self.y.1)),
        );
    }
}

/// A colour of the trail's scale for `f` between 0 and 1.
fn scale_color(f: f64) -> Color32 {
    let [r, g, b] = trail_color(f.clamp(0.0, 1.0) as f32).map(|c| (c.clamp(0.0, 1.0) * 255.0) as u8);
    Color32::from_rgb(r, g, b)
}

/// The colour of a check's utilisation: the scale up to 1, red above it.
fn utilisation_color(u: f64) -> Option<Color32> {
    u.is_finite().then(|| if u > 1.0 { LIMIT } else { scale_color(u) }).or((u == f64::INFINITY).then_some(LIMIT))
}

/// The motor search: its options as `sim-cli search` takes them, the ranked table with a button per row that names
/// the candidate in the running config, and the views of what it found; the ratio sweep and the kinematics comparison.
pub(super) fn search(ui: &mut Ui, model: &Model, inputs: &mut Inputs, actions: &mut Vec<Action>) {
    let tr = |key: &str| model.tr.tr(key).to_string();
    let found = model.search_table();
    ui.horizontal_wrapped(|ui| {
        ComboBox::from_label(tr("ui-search-supply")).selected_text(&inputs.supply).width(60.0).show_ui(ui, |ui| {
            for &supply in SUPPLIES {
                ui.selectable_value(&mut inputs.supply, supply.to_string(), supply);
            }
        });
        let vendor = |v: &str| {
            if v.is_empty() {
                tr("ui-search-vendor-none")
            } else if v == "all" {
                tr("ui-search-vendor-all")
            } else {
                v.to_string()
            }
        };
        let s = &mut inputs.search;
        ComboBox::from_label(tr("ui-search-vendor")).selected_text(vendor(&s.vendor)).width(90.0).show_ui(ui, |ui| {
            for v in ["", "all"].into_iter().chain(vendors()) {
                ui.selectable_value(&mut s.vendor, v.to_string(), vendor(v));
            }
        });
        ui.label(tr("ui-search-series"));
        ui.add(TextEdit::singleline(&mut inputs.series).desired_width(90.0));
        ui.label(tr("ui-search-gearboxes"));
        ui.add(TextEdit::singleline(&mut inputs.gearboxes).desired_width(60.0));
        ui.checkbox(&mut inputs.axes, tr("ui-search-axes"));
    });
    ui.horizontal_wrapped(|ui| {
        let s = &mut inputs.search;
        let drives = [
            (Drives::Paired, "ui-search-drives-paired"),
            (Drives::Neighbours, "ui-search-drives-neighbours"),
            (Drives::All, "ui-search-drives-all"),
        ];
        let named = drives.iter().find(|d| d.0 == s.drives).map_or(String::new(), |d| tr(d.1));
        ComboBox::from_label(tr("ui-search-drives")).selected_text(named).width(110.0).show_ui(ui, |ui| {
            for (drives, key) in drives {
                ui.selectable_value(&mut s.drives, drives, tr(key));
            }
        });
        ui.label(tr("ui-search-rank-by")).on_hover_text(tr("ui-search-rank-by-hint"));
        ui.add(TextEdit::singleline(&mut s.rank_by).desired_width(110.0));
        let motor = if s.motor.is_empty() { tr("ui-search-motor-all") } else { s.motor.clone() };
        ComboBox::from_label(tr("ui-search-motor-id")).selected_text(motor).width(80.0).show_ui(ui, |ui| {
            ui.selectable_value(&mut s.motor, String::new(), tr("ui-search-motor-all"));
            for m in &model.session.motors {
                ui.selectable_value(&mut s.motor, m.id.to_string(), format!("{} ({})", tr(m.key), m.id));
            }
        });
        ui.label(tr("ui-search-duties")).on_hover_text(tr("ui-search-duties-hint"));
        ui.add(TextEdit::singleline(&mut s.duties).desired_width(160.0));
        ui.label(tr("ui-search-top"));
        ui.add(DragValue::new(&mut s.top).range(0..=6));
        run_button(ui, model, "ui-search-run", Analysis::Search, actions);
        if found.is_some() && ui.button(tr("ui-clear")).clicked() {
            actions.push(Action::Clear(Analysis::Search));
        }
    });
    if let Some(rows) = &found {
        heading(ui, model, "ui-search-ranked", Analysis::Search);
        table(
            ui,
            "ui-search-ranked",
            rows,
            Some((&tr("ui-search-use"), &mut |row| actions.push(Action::UseCandidate(row)))),
        );
    }
    if let Some((candidates, _)) = &model.candidates {
        check_map(ui, model, candidates);
        cost_margin(ui, model, candidates);
        operating_points(ui, model, candidates);
        failed(ui, model, candidates);
    }
    ui.separator();
    ratio_sweep(ui, model, inputs, actions);
    ui.separator();
    kinematics(ui, model, inputs, actions);
}

/// Rows of the heat map: the first of the ranking.
const MAP_ROWS: usize = 40;

/// The candidates × checks heat map: per candidate in the order of the ranking, per check its utilisation in the
/// trail's colours up to 1 and red above; the checks' names written upwards over their columns.
fn check_map(ui: &mut Ui, model: &Model, candidates: &[sim_components::catalog::Candidate]) {
    let tr = |key: &str| model.tr.tr(key).to_string();
    let map = heatmap(candidates);
    if map.checks.is_empty() {
        return;
    }
    heading(ui, model, "ui-search-heatmap", Analysis::Search);
    let rows = map.rows.len().min(MAP_ROWS);
    let (label_w, head_h, row_h) = (130.0, 150.0, 14.0);
    let col_w = ((ui.available_width() - label_w) / map.checks.len() as f32).clamp(8.0, 22.0);
    let size = vec2(label_w + col_w * map.checks.len() as f32, head_h + row_h * rows as f32);
    let (rect, response) = ui.allocate_exact_size(size, Sense::hover());
    let painter = ui.painter_at(rect);
    let font = FontId::proportional(10.0);
    for (j, check) in map.checks.iter().enumerate() {
        let galley = painter.layout_no_wrap(tr(&format!("check-{check}")), font.clone(), dim());
        let pos = egui::pos2(rect.left() + label_w + col_w * (j as f32 + 0.5) - 6.0, rect.top() + head_h - 2.0);
        painter.add(egui::epaint::TextShape::new(pos, galley, dim()).with_angle(-std::f32::consts::FRAC_PI_2));
    }
    for (i, (row, c)) in map.rows.iter().zip(candidates).take(rows).enumerate() {
        let top = rect.top() + head_h + row_h * i as f32;
        let name = if c.article.is_empty() { c.axis.clone() } else { format!("{} {}", i + 1, c.article) };
        painter.text(
            egui::pos2(rect.left(), top + row_h / 2.0),
            egui::Align2::LEFT_CENTER,
            name,
            font.clone(),
            chalk(),
        );
        for (j, &u) in row.iter().enumerate() {
            if let Some(color) = utilisation_color(u) {
                let left = rect.left() + label_w + col_w * j as f32;
                painter.rect_filled(
                    Rect::from_min_size(egui::pos2(left, top), vec2(col_w - 1.0, row_h - 1.0)),
                    0.0,
                    color,
                );
            }
        }
    }
    if let Some(pointer) = response.hover_pos() {
        let (i, j) = (
            ((pointer.y - rect.top() - head_h) / row_h).floor(),
            ((pointer.x - rect.left() - label_w) / col_w).floor(),
        );
        if i >= 0.0 && j >= 0.0 && (i as usize) < rows && (j as usize) < map.checks.len() {
            let (i, j) = (i as usize, j as usize);
            let u = map.rows[i][j];
            response.on_hover_text(format!(
                "{} · {}: {}",
                candidates[i].article,
                tr(&format!("check-{}", map.checks[j])),
                if u.is_nan() { String::new() } else { short(u) }
            ));
        }
    }
    ui.horizontal(|ui| {
        scale_bar(ui, "0", "1", trail_color);
        ui.colored_label(LIMIT, format!("■ {}", tr("ui-search-heatmap-over")));
    });
}

/// Cost (installed mass) against the worst margin, coloured by the energy per cycle; the front (`search::front`) drawn
/// larger and joined by a line, the failing ones hollow.
fn cost_margin(ui: &mut Ui, model: &Model, candidates: &[sim_components::catalog::Candidate]) {
    let tr = |key: &str| model.tr.tr(key).to_string();
    let points = scatter(candidates);
    if points.is_empty() {
        return;
    }
    heading(ui, model, "ui-search-scatter", Analysis::Search);
    ui.weak(tr("ui-search-scatter-hint"));
    let energies = points.iter().map(|p| p.energy).filter(|e| e.is_finite());
    let (low, high) = energies.clone().fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), e| (l.min(e), h.max(e)));
    let mut front: Vec<[f64; 2]> = points.iter().filter(|p| p.front).map(|p| [p.cost, p.margin]).collect();
    front.sort_by(|a, b| a[0].total_cmp(&b[0]));
    plot("search-scatter").height(200.0).x_axis_label(tr("ui-search-cost")).y_axis_label(tr("ui-search-margin")).show(
        ui,
        |plot| {
            plot.hline(HLine::new(tr("ui-search-margin"), 0.0).color(LIMIT).style(LineStyle::dashed_dense()));
            plot.line(Line::new(tr("ui-search-front"), front.clone()).color(chalk()).width(1.2));
            for p in &points {
                let f = if high > low { (p.energy - low) / (high - low) } else { 0.5 };
                let color = if p.energy.is_finite() { scale_color(f) } else { dim() };
                let c = &candidates[p.index];
                plot.points(
                    Points::new(format!("{} {} · {}", p.index + 1, c.article, c.drive), vec![[p.cost, p.margin]])
                        .color(color)
                        .shape(if p.front { MarkerShape::Diamond } else { MarkerShape::Circle })
                        .filled(p.passes)
                        .radius(if p.front { 5.0 } else { 3.0 }),
                );
            }
        },
    );
    ui.horizontal(|ui| {
        ui.weak(tr("ui-search-energy"));
        scale_bar(ui, &short(low), &short(high), trail_color);
    });
}

/// The best candidates' operating points on their motor's torque-speed curves, a plot each. A simple version of the
/// cycle's (`torque_speed`): the curves of the candidate's motor and drive, its points per motor.
fn operating_points(ui: &mut Ui, model: &Model, candidates: &[sim_components::catalog::Candidate]) {
    let tr = |key: &str| model.tr.tr(key).to_string();
    let ran = candidates.iter().enumerate().filter(|(_, c)| c.result.is_some());
    let shown: Vec<_> =
        ran.zip(&model.views.curves).filter_map(|((i, c), curves)| Some((i, c, curves.as_ref()?))).collect();
    if shown.is_empty() {
        return;
    }
    heading(ui, model, "ui-search-curves", Analysis::Search);
    key_row(ui, &[(tr("ui-points-curve"), Color32::GRAY, true), (tr("ui-points-s1"), chalk(), true)]);
    let rpm = model.units.scale(sim_app::runs::Quantity::Speed.unit()).0;
    let scaled = |line: &Vec<[f64; 2]>| line.iter().map(|[s, t]| [s * rpm, *t]).collect::<Vec<_>>();
    for (i, c, curves) in shown {
        ui.label(format!("{} {} · {} · {}", i + 1, c.article, c.drive, c.supply));
        let Some((load, _)) = &c.result else { continue };
        plot(("search-points", i))
            .height(140.0)
            .x_axis_label(model.speed_label())
            .y_axis_label(tr("ui-points-torque"))
            .show(ui, |plot| {
                let motors = load.motors.iter().filter(|m| c.motor.is_empty() || m.id == c.motor);
                for (m, color) in motors.zip(PALETTE.iter().cycle()) {
                    let points: Vec<[f64; 2]> = m.points.iter().map(|(s, t)| [s * rpm, *t]).collect();
                    plot.points(Points::new(m.id.to_string(), points).color(*color).radius(1.0));
                }
                for (k, line) in curves.iter().flat_map(|chart| chart.peak.iter().chain(chart.s1.iter()).enumerate()) {
                    let (key, color, style) = if k < 2 {
                        ("ui-points-curve", Color32::GRAY, LineStyle::dashed_dense())
                    } else {
                        ("ui-points-s1", chalk(), LineStyle::dotted_dense())
                    };
                    plot.line(Line::new(tr(key), scaled(line)).color(color).style(style).width(1.5));
                }
            });
    }
}

/// Failing candidates listed: the first ten that ran, each with its failing checks and their values.
fn failed(ui: &mut Ui, model: &Model, candidates: &[sim_components::catalog::Candidate]) {
    let keys: Vec<&str> = model.session.motors.iter().map(|m| m.key).collect();
    let failing: Vec<_> = candidates.iter().enumerate().filter(|(_, c)| !failures(c).is_empty()).take(10).collect();
    if failing.is_empty() {
        return;
    }
    heading(ui, model, "ui-search-failures", Analysis::Search);
    for (i, c) in failing {
        egui::CollapsingHeader::new(format!("{} {} · {}", i + 1, c.article, c.drive)).id_salt(("failed", i)).show(
            ui,
            |ui| {
                for v in failures(c) {
                    ui.colored_label(LIMIT, failure_line(&model.tr, v, &keys));
                }
            },
        );
    }
}

/// The ratio sweep: the inertia ratio and the tightest check over the swept gear ratio or pulley radius, with the
/// classic optimum, the best value for the cycle and the catalogue gearboxes around it marked.
fn ratio_sweep(ui: &mut Ui, model: &Model, inputs: &mut Inputs, actions: &mut Vec<Action>) {
    let tr = |key: &str| model.tr.tr(key).to_string();
    ui.horizontal_wrapped(|ui| {
        ui.strong(tr("ui-ratio"));
        ui.weak(tr("ui-ratio-hint"));
    });
    let s = &mut inputs.search;
    ui.horizontal_wrapped(|ui| {
        for (key, label) in [("gearbox.ratio", "ui-ratio-gear"), ("geometry.pulley_radius_m", "ui-ratio-pulley")] {
            ui.radio_value(&mut s.sweep_key, key.to_string(), tr(label));
        }
        ui.label(tr("ui-ratio-from"));
        ui.add(DragValue::new(&mut s.sweep_from).speed(0.01).range(0.0..=1000.0));
        ui.label(tr("ui-ratio-to"));
        ui.add(DragValue::new(&mut s.sweep_to).speed(0.01).range(0.0..=1000.0));
        ui.label(tr("ui-ratio-count"));
        ui.add(DragValue::new(&mut s.sweep_count).range(2..=40));
        ui.label(tr("ui-search-gearboxes")).on_hover_text(tr("ui-ratio-snap-hint"));
        ui.add(TextEdit::singleline(&mut s.sweep_snap).desired_width(90.0));
        view_button(ui, model, ViewJob::Sweep, actions);
    });
    let Some((config, sweep)) = &model.views.sweep else { return };
    outdated_mark(ui, model, config);
    ui.label(format!(
        "{}: {}   {}: {}",
        tr("ui-ratio-classic"),
        short(sweep.classic),
        tr("ui-ratio-best"),
        short(sweep.best)
    ));
    let line = |f: &dyn Fn(&sim_components::search::SweepRow) -> f64| -> Vec<[f64; 2]> {
        sweep.rows.iter().map(|r| [r.value, f(r)]).filter(|p| p[1].is_finite()).collect()
    };
    let marks = |plot: &mut egui_plot::PlotUi| {
        plot.vline(VLine::new(tr("ui-ratio-classic"), sweep.classic).color(chalk()).style(LineStyle::dashed_dense()));
        plot.vline(VLine::new(tr("ui-ratio-best"), sweep.best).color(PASS));
    };
    let tightest = |r: &sim_components::search::SweepRow| r.tightest.map_or(f64::NAN, |t| t.1);
    let across = tr(if inputs.search.sweep_key == "gearbox.ratio" { "ui-ratio-gear" } else { "ui-ratio-pulley" });
    plot("ratio-inertia").height(140.0).x_axis_label(across.clone()).y_axis_label(tr("ui-kinematics-inertia")).show(
        ui,
        |plot| {
            plot.line(Line::new(tr("ui-kinematics-inertia"), line(&|r| r.inertia_ratio)).color(PALETTE[0]).width(1.8));
            marks(plot);
        },
    );
    plot("ratio-margin").height(140.0).x_axis_label(across).y_axis_label(tr("ui-search-share")).show(ui, |plot| {
        plot.line(Line::new(tr("ui-search-share"), line(&tightest)).color(PALETTE[1]).width(1.8));
        plot.hline(HLine::new(tr("ui-search-share"), 1.0).color(LIMIT).style(LineStyle::dashed_dense()));
        for r in &sweep.snapped {
            plot.points(
                Points::new(r.gearbox.clone(), vec![[r.value, tightest(r)]])
                    .color(PALETTE[2])
                    .shape(MarkerShape::Square)
                    .radius(4.0),
            );
        }
        marks(plot);
    });
    if !sweep.snapped.is_empty() {
        key_row(ui, &[(tr("ui-ratio-catalogue"), PALETTE[2], true)]);
    }
}

/// The kinematics comparison: one task on several machines, the best candidate of each side by side.
fn kinematics(ui: &mut Ui, model: &Model, inputs: &mut Inputs, actions: &mut Vec<Action>) {
    let tr = |key: &str| model.tr.tr(key).to_string();
    ui.horizontal_wrapped(|ui| {
        ui.strong(tr("ui-kinematics"));
        ui.weak(tr("ui-kinematics-hint"));
    });
    let s = &mut inputs.search;
    ui.horizontal_wrapped(|ui| {
        ui.label(tr("ui-kinematics-task"));
        ui.add(TextEdit::singleline(&mut s.task_file).desired_width(200.0));
        ui.label(tr("ui-kinematics-machines")).on_hover_text(tr("ui-kinematics-machines-hint"));
        ui.add(TextEdit::singleline(&mut s.machines).desired_width(160.0));
        view_button(ui, model, ViewJob::Kinematics, actions);
    });
    let Some((config, rows)) = &model.views.kinematics else { return };
    outdated_mark(ui, model, config);
    table(ui, "kinematics", &sim_app::search_views::kinematics_table(&model.tr, rows), None);
}

/// The button that starts one of these views' computations, or a spinner while it runs.
fn view_button(ui: &mut Ui, model: &Model, kind: ViewJob, actions: &mut Vec<Action>) {
    if model.views.running(kind) {
        ui.spinner();
    } else if ui.button(model.tr.tr("ui-view-run")).clicked() {
        actions.push(Action::RunView(kind));
    }
}

/// The mark on a result of a config that has changed since.
fn outdated_mark(ui: &mut Ui, model: &Model, config: &sim_core::Config) {
    if *config != model.config {
        ui.colored_label(NOTE, model.tr.tr("ui-analysis-outdated"));
    }
}

/// The path tab's maps of peak torque and following error: from each node of a grid over a plane of two axes a short
/// move along one axis (`search_views::move_map`), run in the background.
pub(super) fn move_maps(ui: &mut Ui, model: &Model, inputs: &mut Inputs, actions: &mut Vec<Action>) {
    let tr = |key: &str| model.tr.tr(key).to_string();
    let axes = &model.session.axes;
    if axes.len() < 2 {
        return;
    }
    ui.separator();
    ui.horizontal_wrapped(|ui| {
        ui.strong(tr("ui-map"));
        ui.weak(tr("ui-map-hint"));
    });
    let name = |i: usize| format!("{} [{}]", tr(axes[i].key), axes[i].unit);
    let m = &mut inputs.map;
    ui.horizontal_wrapped(|ui| {
        let planes: Vec<[usize; 2]> =
            [[0, 1], [0, 2], [1, 2]].into_iter().filter(|p| p[1] < axes.len().min(3)).collect();
        let plane_name = |p: [usize; 2]| format!("{} – {}", tr(axes[p[0]].key), tr(axes[p[1]].key));
        ComboBox::from_label(tr("ui-map-plane")).selected_text(plane_name(m.plane)).show_ui(ui, |ui| {
            for p in planes {
                ui.selectable_value(&mut m.plane, p, plane_name(p));
            }
        });
        ComboBox::from_label(tr("ui-map-axis")).selected_text(tr(axes[m.axis.min(axes.len() - 1)].key)).show_ui(
            ui,
            |ui| {
                for (i, a) in axes.iter().enumerate() {
                    ui.selectable_value(&mut m.axis, i, tr(a.key));
                }
            },
        );
        ui.label(format!("{} [{}]", tr("ui-map-distance"), axes[m.axis.min(axes.len() - 1)].unit))
            .on_hover_text(tr("ui-map-distance-hint"));
        ui.add(DragValue::new(&mut m.distance).speed(0.001).range(0.0..=10.0));
        ui.label(tr("ui-map-nodes"));
        ui.add(DragValue::new(&mut m.n).range(2..=15));
        view_button(ui, model, ViewJob::Map, actions);
    });
    let Some((config, map)) = &model.views.map else { return };
    outdated_mark(ui, model, config);
    ui.weak(format!("{}: {} {}", tr("ui-map-move"), tr(axes[map.axis].key), short(map.distance)));
    let unit = axes[map.axis].unit;
    let (error_scale, error_prefix) =
        display_scale(map.error.iter().copied().filter(|e| e.is_finite()).fold(0.0, f64::max), unit);
    let torque = model.session.signals.iter().find(|s| s.quantity_key == "quantity-torque");
    for (values, key, scale, unit) in [
        (&map.torque, "ui-map-torque", 1.0, torque.map_or(String::new(), |s| s.unit.clone())),
        (&map.error, "ui-map-error", error_scale, format!("{error_prefix}{unit}")),
    ] {
        ui.label(format!("{} [{unit}]", tr(key)));
        let finite = values.iter().copied().filter(|v| v.is_finite());
        let (low, high) = finite.fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), v| (l.min(v), h.max(v)));
        let side = ui.available_width().min(300.0);
        let grid = GridMap::draw(ui, side, (map.x, map.y), map.n, |k| {
            let v = values[k];
            v.is_finite().then(|| scale_color(if high > low { (v - low) / (high - low) } else { 0.5 }))
        });
        grid.label_axes(ui, name(map.axes[0]), name(map.axes[1]));
        if let Some((i, j)) = grid.hovered() {
            let v = values[j * map.n + i];
            let shown = if v.is_finite() { short(v * scale) } else { tr("ui-map-not-made") };
            grid.response.clone().on_hover_text(format!(
                "{} {}, {} {}: {shown}",
                tr(axes[map.axes[0]].key),
                short(grid.at(map.x, i)),
                tr(axes[map.axes[1]].key),
                short(grid.at(map.y, j)),
            ));
        }
        scale_bar(ui, &short(low * scale), &short(high * scale), trail_color);
    }
    ui.weak(tr("ui-map-empty"));
}
