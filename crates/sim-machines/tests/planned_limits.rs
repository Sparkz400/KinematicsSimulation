//! The time-optimal plan keeps every limit it plans with, measured on the plan itself (16 November 2026): per profile
//! cycle the tool's speed, acceleration and jerk by differences of the planned poses, the motors' angles by the
//! machine's inverse kinematics and their speed and acceleration by differences, the motor torques by the machine's own
//! rigid inverse dynamics (`Machine::rigid_motor_torque`), each motor's torque through its gearbox's efficiency the way
//! the power flows, the gearbox's output torque, the belt's tensions (with the speed), the guides' loads and the
//! machine's own checks (`Machine::rigid_checks`), each against its limit as the plan holds it (the reserve, the margin
//! and the share kept back for the speed). The worst share of a limit must stay at 1 within 1e-4 (the rounding of the
//! differences over 1 ms). Lines, arcs (one of radius 100 m, a half turn), splines on three axes, a helix with a carried
//! axis and a path with corners on the T-gantry, the delta-d3, the SCARA in tool space, the linear delta and the single
//! axis, each loss-free and as shipped; the single axis also behind a gearbox of efficiency 0.9 braking, with a rotor
//! whose inertia outweighs the load's and on an incline.

use sim_components::path::{Piece, PlanLimits};
use sim_components::{Sim, load_config};
use sim_core::{Config, MachineEntry};

/// The share of every limit the plan may use: the plan keeps 2e-4 back, the differences over 1 ms round by less.
const SLACK: f64 = 1e-4;
/// The share of each speed rating the plan keeps back (`path::SPEED_KEPT`).
const SPEED_KEPT: f64 = 0.01;

/// The worst share of each kind of limit over a plan, with the time and what it was.
#[derive(Default)]
struct Worst(Vec<(&'static str, f64, f64, String)>);

impl Worst {
    fn note(&mut self, what: &'static str, share: f64, t: f64, detail: impl FnOnce() -> String) {
        match self.0.iter_mut().find(|w| w.0 == what) {
            Some(w) if share > w.1 => (w.1, w.2, w.3) = (share, t, detail()),
            Some(_) => {}
            None => self.0.push((what, share, t, detail())),
        }
    }
    fn highest(&self) -> f64 {
        self.0.iter().map(|w| w.1).fold(0.0, f64::max)
    }
}

/// The drive as the plan holds it (`PlanLimits::rated`): the drive's current rating and the gearbox's input speed rating.
fn rated(sim: &Sim, limits: &PlanLimits, m: usize) -> sim_components::drive::DriveParams {
    let [current, speed, _] = limits.ratings.get(m).copied().unwrap_or([f64::INFINITY; 3]);
    let mut drive = sim.control.drive_params_of(m).clone();
    drive.drive_max_current = drive.drive_max_current.min(current);
    drive.max_speed = drive.max_speed.min(speed) * limits.margin * (1.0 - SPEED_KEPT);
    drive
}

/// Every limit of the plan `points` measured on it, worst share per kind.
fn measure(sim: &Sim, points: &[(f64, Vec<f64>)]) -> Worst {
    let machine = sim.machine.as_ref();
    let (axes, motors) = (machine.axes(), machine.motors().len());
    let limits = sim.control.plan_limits();
    let reserve = limits.reserve * limits.margin;
    let dt = points[1].0 - points[0].0;
    // At rest before and after the plan.
    let mut poses: Vec<Vec<f64>> = vec![points[0].1.clone(); 3];
    poses.extend(points.iter().map(|(_, p)| p.clone()));
    poses.extend(std::iter::repeat_n(points[points.len() - 1].1.clone(), 3));
    let angles: Vec<Vec<f64>> = poses
        .iter()
        .map(|p| {
            let mut out = vec![0.0; motors];
            machine.inverse(p, &mut out);
            out
        })
        .collect();
    let drives: Vec<_> = (0..motors).map(|m| rated(sim, limits, m)).collect();
    let checks = machine.rating_checks();
    let mut worst = Worst::default();
    let rate = |x: &[Vec<f64>], i: usize| -> [Vec<f64>; 3] {
        let n = x[i].len();
        let v = (0..n).map(|k| (x[i + 1][k] - x[i - 1][k]) / (2.0 * dt)).collect();
        let a = (0..n).map(|k| (x[i + 1][k] - 2.0 * x[i][k] + x[i - 1][k]) / (dt * dt)).collect();
        let j = (0..n)
            .map(|k| (x[i + 2][k] - 2.0 * x[i + 1][k] + 2.0 * x[i - 1][k] - x[i - 2][k]) / (2.0 * dt.powi(3)))
            .collect();
        [v, a, j]
    };
    for i in 2..poses.len() - 2 {
        let t = (i as f64 - 3.0) * dt;
        let p = &poses[i];
        let [v, a, j] = rate(&poses, i);
        for (k, axis) in axes.iter().enumerate() {
            worst.note("axis jerk", j[k].abs() / axis.max_jerk, t, || {
                format!("{} {:.4} of {}", axis.id, j[k], axis.max_jerk)
            });
        }
        let [w, alpha, _] = rate(&angles, i);
        let mut torque = vec![0.0; motors];
        machine.rigid_motor_torque(&angles[i], &w, &alpha, &mut torque);
        let speed = v.iter().map(|x| x * x).sum::<f64>().sqrt();
        let mut guide = Vec::new();
        if speed > 1e-9 {
            machine.rigid_friction(p, &v.iter().map(|x| x / speed).collect::<Vec<_>>(), &mut guide);
        }
        for m in 0..motors {
            let drive = &drives[m];
            worst.note("motor speed", w[m].abs() / drive.max_speed, t, || {
                format!("motor {m} {:.3} of {:.3}", w[m], drive.max_speed)
            });
            // Through the gearbox: the load's torque over eta where the motor drives it, times eta where it drives the motor.
            let gear = limits.gearbox.get(m).copied().flatten();
            let (inertia, eta) = gear.map_or((0.0, 1.0), |g| (g[1], g[3]));
            let load = torque[m] - inertia * alpha[m];
            let real = inertia * alpha[m] + if load * w[m] >= 0.0 { load / eta } else { load * eta };
            let friction = w[m].signum() * limits.shaft_friction(m, w[m].abs())
                + guide.get(m).map_or(0.0, |[c, b]| (c + b * speed) / eta);
            let with_friction = real + friction;
            let most = if drive.drive_max_current.is_finite() {
                drive.torque_of(drive.drive_max_current)
            } else {
                f64::INFINITY
            };
            let curve = |s: f64| {
                let (low, high) = drive.torque_curve(s);
                (high.min(most), (-low).min(most))
            };
            let (still_high, still_low) = curve(0.0);
            let (high, low) = curve(w[m].abs());
            let (drives_room, brakes_room) = (high - (1.0 - reserve) * still_high, low - (1.0 - reserve) * still_low);
            let (up, down) = (real.max(with_friction), real.min(with_friction));
            let (up_room, down_room) =
                if w[m] >= 0.0 { (drives_room, brakes_room) } else { (brakes_room, drives_room) };
            let share = (up / up_room).max(-down / down_room);
            worst.note("motor torque", share, t, || {
                format!(
                    "motor {m} {real:.4} (friction {friction:.4}) at {:.2} rad/s of +{up_room:.4} / -{down_room:.4}",
                    w[m]
                )
            });
            let power = limits.ratings.get(m).map_or(f64::INFINITY, |r| r[2] * limits.margin);
            if real * w[m] < 0.0 && power.is_finite() {
                worst.note("braking power", (real * w[m]).abs() / power, t, || {
                    format!("motor {m} {:.1} W of {power}", real * w[m])
                });
            }
            if let Some([ratio, _, rating, _]) = gear.filter(|g| g[2].is_finite()) {
                worst.note("gearbox output", (ratio * load).abs() / (reserve * rating), t, || {
                    format!("motor {m} {:.3} of {:.3}", ratio * load, reserve * rating)
                });
            }
        }
        let [lowest, highest, share] = limits.tension;
        if lowest.is_finite() || highest.is_finite() {
            let zero = vec![0.0; axes.len()];
            let (mut still, mut moving) = (Vec::new(), Vec::new());
            machine.rigid_tensions_moving(p, &zero, &zero, &mut still);
            machine.rigid_tensions_moving(p, &v, &a, &mut moving);
            for (k, (c, now)) in still.iter().zip(&moving).enumerate() {
                let room = share * limits.margin * (highest - c).min(c - lowest);
                worst.note("belt tension", (now - c).abs() / room, t, || {
                    format!("section {k} {now:.2} N, at rest {c:.2}, room {room:.2}")
                });
            }
        }
        if limits.guide.is_finite() {
            let mut loads = Vec::new();
            machine.rigid_guide_loads(p, &a, &mut loads);
            for (k, [rest, moving]) in loads.iter().enumerate() {
                let room = reserve * limits.guide - rest;
                worst.note("guide load", moving.abs() / room, t, || {
                    format!("guide {k} {rest:.1} + {moving:.1} of {}", limits.guide)
                });
            }
        }
        if !checks.is_empty() {
            let zero = vec![0.0; axes.len()];
            let (mut values, mut still, mut pushed) = (Vec::new(), Vec::new(), Vec::new());
            machine.rigid_checks(p, &v, &a, &mut values);
            machine.rigid_checks(p, &zero, &zero, &mut still);
            machine.rigid_checks(p, &zero, &axes.iter().map(|_| 1.0).collect::<Vec<_>>(), &mut pushed);
            for (k, check) in checks.iter().enumerate().take(values.len()) {
                if !values[k].is_finite() {
                    continue;
                }
                let share = if pushed[k] != still[k] { reserve } else { limits.margin * (1.0 - SPEED_KEPT) };
                worst.note("own checks", values[k].abs() / (share * check.limit), t, || {
                    format!("{} {:.4} of {:.4}", check.check, values[k], share * check.limit)
                });
            }
        }
    }
    worst
}

/// Plans `pieces` time-optimal, measures it and returns its duration and worst share; prints both.
fn run(name: &str, sim: &Sim, pieces: &[Piece]) -> (f64, f64) {
    let points = sim.plan_path(pieces, true).unwrap_or_else(|e| panic!("{name}: refused, {}", e.key));
    let worst = measure(sim, &points);
    let duration = points[points.len() - 1].0;
    eprintln!("[{name}] {duration:.4} s, worst {:.5}", worst.highest());
    for (what, share, t, detail) in &worst.0 {
        eprintln!("    {what:<15} {share:.5} at {t:.3} s: {detail}");
    }
    (duration, worst.highest())
}

/// The shipped config with every effect the loss-free config of the machine switches off, as shipped, and as shipped
/// with twenty times the jerk on every axis (so that the torques, not the jerk, bound the plan: braking through the
/// gearboxes' efficiency).
fn both(shipped: Config, lossless: &Config) -> Vec<(&'static str, Config)> {
    let mut free = shipped.clone();
    if let Some(effects) = lossless.get("effects").ok().and_then(|e| e.as_table().cloned()) {
        for (key, value) in effects.iter().filter(|(_, v)| v.as_bool() == Some(false)) {
            free.set_text(&format!("effects.{key} = {value}")).ok();
        }
    }
    let mut quick = shipped.clone();
    if let Some(axes) = shipped.get("axis").ok().and_then(|a| a.as_table().cloned()) {
        for (id, axis) in &axes {
            for key in ["max_jerk_m_s3", "max_jerk_rad_s3"] {
                if let Some(jerk) = axis.get(key).and_then(|j| j.as_float().or(j.as_integer().map(|i| i as f64))) {
                    quick.set_text(&format!("axis.{id}.{key} = {:.1}", 20.0 * jerk)).unwrap();
                }
            }
        }
    }
    vec![("lossless", free), ("shipped", shipped), ("shipped jerk x20", quick)]
}

fn plus(start: &[f64], by: &[f64]) -> Vec<f64> {
    start.iter().zip(by.iter().chain(std::iter::repeat(&0.0))).map(|(s, b)| s + b).collect()
}

fn arc(to: Vec<f64>, centre: Vec<f64>, clockwise: bool) -> Piece {
    Piece::Arc { to, centre, clockwise, plane: None }
}

/// The cases of one machine: name and pieces from its start pose.
type Cases = Vec<(&'static str, Vec<Piece>)>;

fn check_machine(
    entry: &MachineEntry,
    configs: Vec<(&'static str, Config)>,
    cases: impl Fn(&[f64]) -> Cases,
) -> Vec<String> {
    let mut failed = Vec::new();
    for (kind, cfg) in configs {
        let sim = Sim::new(entry, &cfg).unwrap();
        let start = sim.control.planned_pos().to_vec();
        let ranges: Vec<_> = sim.machine.axes().iter().map(|a| (a.id, a.min, a.max)).collect();
        eprintln!("{} {kind}: start {start:?}, axes {ranges:?}", entry.id);
        for (name, pieces) in cases(&start) {
            let label = format!("{} {kind} {name}", entry.id);
            let (_, worst) = run(&label, &sim, &pieces);
            if worst > 1.0 + SLACK {
                failed.push(format!("{label}: {worst:.5}"));
            }
        }
    }
    failed
}

/// The arc of radius `r` from `from` to `to` on two axes, clockwise (its centre to the right of the chord).
fn wide_arc(from: &[f64], to: [f64; 2], r: f64) -> Piece {
    let (dx, dz) = (to[0] - from[0], to[1] - from[1]);
    let chord = dx.hypot(dz);
    let rise = (r * r - chord * chord / 4.0).sqrt() / chord;
    let centre = vec![0.5 * (from[0] + to[0]) + rise * dz, 0.5 * (from[1] + to[1]) - rise * dx];
    arc(to.to_vec(), centre, true)
}

/// The T-gantry (x, z), from its start (0.1, 0.35): the line to (0.7, 0.15) and the arc of radius 100 m beside it (the
/// audit's), a half turn, a quarter, a spline and a path with corners and an arc.
fn t_gantry_cases(s: &[f64]) -> Cases {
    vec![
        ("line", vec![Piece::Line(vec![0.7, 0.15])]),
        ("arc r = 100 m", vec![wide_arc(s, [0.7, 0.15], 100.0)]),
        ("half turn", vec![arc(plus(s, &[0.3, 0.0]), plus(s, &[0.15, 0.0]), false)]),
        ("quarter", vec![arc(plus(s, &[0.15, -0.15]), plus(s, &[0.0, -0.15]), true)]),
        (
            "spline",
            vec![
                Piece::Spline(plus(s, &[0.15, 0.1])),
                Piece::Spline(plus(s, &[0.35, -0.05])),
                Piece::Spline(plus(s, &[0.55, 0.12])),
            ],
        ),
        (
            "corners",
            vec![
                Piece::Line(plus(s, &[0.3, 0.0])),
                Piece::Line(plus(s, &[0.3, -0.15])),
                Piece::Line(plus(s, &[0.55, -0.15])),
                arc(plus(s, &[0.55, 0.0]), plus(s, &[0.55, -0.075]), true),
            ],
        ),
    ]
}

/// The delta-d3 and the linear delta (x, y, z): a line, a quarter and a half turn about the centre, a spline through
/// points at three heights (torsion), a path with corners; all no more than 0.09 m below the start.
fn delta_cases(s: &[f64]) -> Cases {
    let r = (s[0] * s[0] + s[1] * s[1]).sqrt();
    let z = s[2];
    vec![
        ("line", vec![Piece::Line(plus(s, &[-2.0 * s[0], 0.0, 0.0]))]),
        ("quarter", vec![arc(vec![0.0, r, z], vec![0.0, 0.0, z], false)]),
        ("half turn", vec![arc(vec![-s[0], -s[1], z], vec![0.0, 0.0, z], false)]),
        (
            "spline 3d",
            vec![
                Piece::Spline(vec![0.05, 0.1, z - 0.05]),
                Piece::Spline(vec![-0.05, 0.05, z - 0.01]),
                Piece::Spline(vec![-0.1, -0.08, z - 0.08]),
            ],
        ),
        (
            "corners",
            vec![Piece::Line(vec![0.0, 0.0, z]), Piece::Line(vec![0.0, 0.1, z]), Piece::Line(vec![0.0, 0.1, z - 0.06])],
        ),
    ]
}

/// The SCARA in tool space (x, y, z, c): a line, an arc, a helix (an arc with c carried along), a spline, corners.
fn scara_cases(s: &[f64]) -> Cases {
    let centre = plus(s, &[-0.08, 0.0, 0.0]);
    let centre3 = centre[..3].to_vec();
    vec![
        ("line", vec![Piece::Line(plus(s, &[-0.15, 0.1, 0.0, 0.5]))]),
        ("arc", vec![arc(plus(s, &[-0.08, 0.08, 0.0, 0.0]), centre3.clone(), false)]),
        ("helix", vec![arc(plus(s, &[-0.08, 0.08, 0.0, 1.0]), centre3.clone(), false)]),
        ("half turn", vec![arc(plus(s, &[-0.16, 0.0, 0.0, 0.0]), centre3, false)]),
        (
            "spline",
            vec![
                Piece::Spline(plus(s, &[-0.05, 0.05, 0.02, 0.2])),
                Piece::Spline(plus(s, &[-0.12, 0.02, -0.02, 0.4])),
                Piece::Spline(plus(s, &[-0.18, 0.08, 0.0, 0.0])),
            ],
        ),
        ("corners", vec![Piece::Line(plus(s, &[-0.1, 0.0, 0.0, 0.0])), Piece::Line(plus(s, &[-0.1, 0.1, 0.0, 0.0]))]),
    ]
}

/// The single axis: a line out along the axis and back.
fn single_cases(s: &[f64]) -> Cases {
    vec![("out", vec![Piece::Line(vec![0.9])]), ("out and back", vec![Piece::Line(vec![0.9]), Piece::Line(vec![s[0]])])]
}

fn scara() -> (&'static MachineEntry, Config) {
    let entry = sim_machines::find("scara").unwrap();
    let root = std::path::Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/../.."));
    let text = std::fs::read_to_string(root.join("crates/machine-joint-kit/config/scara-tool.toml")).unwrap();
    (entry, load_config(entry, &text).unwrap())
}

#[test]
fn the_t_gantry_keeps_its_limits() {
    let entry = &machine_t_gantry::ENTRY;
    let failed = check_machine(entry, both(entry.config(), &machine_t_gantry::lossless_config()), t_gantry_cases);
    assert!(failed.is_empty(), "{failed:?}");
}

#[test]
fn the_delta_d3_keeps_its_limits() {
    let entry = &machine_delta_d3::ENTRY;
    let failed = check_machine(entry, both(entry.config(), &machine_delta_d3::lossless_config()), delta_cases);
    assert!(failed.is_empty(), "{failed:?}");
}

#[test]
fn the_linear_delta_keeps_its_limits_with_the_belts_speed_term() {
    let entry = &machine_delta_linear::ENTRY;
    let failed = check_machine(entry, both(entry.config(), &machine_delta_linear::lossless_config()), delta_cases);
    assert!(failed.is_empty(), "{failed:?}");
}

#[test]
fn the_scara_in_tool_space_keeps_its_limits() {
    let (entry, cfg) = scara();
    let failed = check_machine(entry, both(cfg, &machine_joint_kit::lossless_config(entry)), scara_cases);
    assert!(failed.is_empty(), "{failed:?}");
}

/// An arc of radius 100 m on the T-gantry, from (0.1, 0.15) to (0.7, 0.15) (0.45 mm off its chord), takes about as long
/// as its chord: its ramps keep nearly all of the axes' jerk (`path::ramp_share`). Before 16 November they kept 0.4 of
/// it on every curve: 0.684 s against 0.504 s for the chord.
#[test]
fn a_nearly_straight_arc_takes_about_as_long_as_its_chord() {
    let entry = &machine_t_gantry::ENTRY;
    let sim = Sim::new(entry, &machine_t_gantry::lossless_config()).unwrap();
    let machine = sim.machine.as_ref();
    let drives = sim.control.motor_drives(machine);
    let start = [0.1, 0.15];
    let plan = |name: &str, pieces: &[Piece]| {
        let limits = sim.control.plan_limits();
        let points = sim_components::path::plan_time_optimal(
            machine,
            &drives,
            limits,
            &start,
            pieces,
            1e-3,
            0.0,
            machine.setpoint_slack(),
        )
        .unwrap_or_else(|e| panic!("{name}: {}", e.key));
        let worst = measure(&sim, &points).highest();
        let duration = points[points.len() - 1].0;
        eprintln!("[t-gantry {name} from (0.1, 0.15)] {duration:.4} s, worst {worst:.5}");
        assert!(worst <= 1.0 + SLACK, "{name}: {worst}");
        duration
    };
    let chord = plan("chord", &[Piece::Line(vec![0.7, 0.15])]);
    let bent = plan("arc r = 100 m", &[wide_arc(&start, [0.7, 0.15], 100.0)]);
    assert!(bent < 1.01 * chord, "{bent} s against {chord} s");
}

/// The single axis loss-free and shipped, then behind a gearbox of efficiency 0.9: as it is, with a rotor ten times
/// as heavy (its inertia then outweighs the load's), and on an incline of 30 degrees with the heavy rotor (the weight
/// pushes the carriage down while the motor brakes or accelerates the rotor).
#[test]
fn the_single_axis_keeps_its_limits_braking_through_a_lossy_gearbox() {
    let entry = &machine_single_axis::ENTRY;
    let mut failed = check_machine(entry, both(entry.config(), &machine_single_axis::lossless_config()), single_cases);
    // The torque binds, not the jerk: 0.5 N m peak, jerk 1e5 m/s^3.
    let lossy = |extra: &[&str]| {
        let mut cfg = entry.config();
        let base = [
            "effects.gearbox_efficiency = true",
            "gearbox.efficiency = 0.9",
            "drive.peak_torque_nm = 0.5",
            "axis.x.max_jerk_m_s3 = 1.0e5",
        ];
        for e in base.iter().chain(extra) {
            cfg.set_text(e).unwrap();
        }
        cfg
    };
    for (name, cfg) in [
        ("eta 0.9", lossy(&[])),
        ("eta 0.9, heavy rotor", lossy(&["motor.rotor_inertia_kgm2 = 3.0e-4"])),
        ("eta 0.9, heavy rotor, 30 deg", lossy(&["motor.rotor_inertia_kgm2 = 3.0e-4", "geometry.incline_deg = 30.0"])),
        (
            "eta 0.9, heavy rotor, -30 deg",
            lossy(&["motor.rotor_inertia_kgm2 = 3.0e-4", "geometry.incline_deg = -30.0"]),
        ),
    ] {
        let sim = Sim::new(entry, &cfg).unwrap();
        let start = sim.control.planned_pos().to_vec();
        for (case, pieces) in single_cases(&start) {
            let label = format!("single-axis {name} {case}");
            let (_, worst) = run(&label, &sim, &pieces);
            if worst > 1.0 + SLACK {
                failed.push(format!("{label}: {worst:.5}"));
            }
        }
    }
    assert!(failed.is_empty(), "{failed:?}");
}
