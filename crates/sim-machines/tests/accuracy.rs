//! The positioning accuracy test (`sim_components::accuracy::study`) against hand calculations: a gearbox's backlash, the
//! sag of a vertical ball screw under its load, the growth of a warm screw and a lead error table; and the test run on
//! the single axis, the screw axis, the T-gantry and the D3 delta.

use sim_components::accuracy::{STEEL_EXPANSION_PER_K, Settings, Study, axis_errors, study};
use sim_components::{Sim, load_config};
use sim_core::Config;

fn config(id: &str, effects: &str, extra: &str) -> Config {
    let entry = sim_machines::find(id).unwrap();
    let text = entry.default_config.replacen("\n[effects]", &format!("\n[effects]\n{effects}\n"), 1) + "\n" + extra;
    load_config(entry, &text).unwrap()
}

fn run(id: &str, cfg: &Config, points: usize, repeats: usize) -> Study {
    study(sim_machines::find(id).unwrap(), cfg, Settings { points, repeats, dwell_s: 0.3 }).unwrap()
}

/// The single axis's gearbox has a backlash of 8.7e-4 rad at its output, on a pulley of 20 mm: a gap of 8.7e-4 * 0.02 =
/// 17.4 um at the carriage, which the reversal error takes on whole. The rest of the reversal (11 um at the first
/// target) is the guide's friction held through the belt and gearbox where the carriage comes to rest; it changes a
/// little with the gap (the bristles stop at another force), so 10 %.
#[test]
fn the_reversal_error_takes_the_gearbox_backlash() {
    let with = run("single-axis", &config("single-axis", "", ""), 3, 1);
    let without = run("single-axis", &config("single-axis", "", "").with("effects.gearbox_backlash", false), 3, 1);
    let gap = 8.7e-4 * 0.02;
    let reversal = |s: &Study| (s.axes[0].mean_up[0] - s.axes[0].mean_down[0]).abs();
    let found = reversal(&with) - reversal(&without);
    assert!((found - gap).abs() < 0.1 * gap, "{found} against {gap}");
}

/// The screw axis standing up (slide.incline_deg 90) with no friction, no efficiency loss and no ripple: the encoder
/// holds the motor at the target and the carriage sags by its weight over the drive's stiffness, from either side.
/// Weight 25 kg * 9.81 = 245.25 N. Screw (root 21.2 mm, A = 3.530e-4 m^2, E A = 7.272e7 N, over its whole 1 m: the
/// model takes the softest place for the stroke) in series with nut 4e8 and bearings 7.5e8 N/m: 1.7585e-8 m/N;
/// coupling 9000 N m/rad seen at the nut over (lead / 2 pi)^2 = (3.1831e-3 m)^2: 1.1258e-9 m/N; together 1.8711e-8
/// m/N, a sag of 4.589 um.
#[test]
fn a_vertical_screw_sags_by_its_load_over_its_stiffness() {
    let mut cfg = config("screw-axis", "", "");
    for (key, on) in [
        ("effects.gearbox_efficiency", false),
        ("effects.joint_friction", false),
        ("effects.motor_friction", false),
        ("effects.gearbox_mesh_friction", false),
        ("effects.torque_ripple", false),
    ] {
        cfg = cfg.with(key, on);
    }
    cfg.set("slide.incline_deg", toml::Value::Float(90.0)).unwrap();
    let s = run("screw-axis", &cfg, 3, 1);
    let sag =
        25.0 * 9.81 * (1.0 / 7.2716e7 + 1.0 / 4e8 + 1.0 / 7.5e8 + (0.02 / std::f64::consts::TAU).powi(2) / 9000.0);
    for x in s.axes[0].mean_up.iter().chain(&s.axes[0].mean_down) {
        assert!((x + sag).abs() < 0.01 * sag, "{x} against {}", -sag);
    }
}

/// The screw 10 K warm (accuracy.axes rise_k), fixed at x = 0: each target x reads 11.5e-6 * 10 * x further out than
/// with effects.thermal_growth off, 4.6 um at 0.4 m; the run is otherwise the same to the bit (the growth is taken at
/// the real position, a micrometre off the target: 1e-10 m more). With a lead error table
/// (0, 0) to (0.8 m, 16 um) on top: 20 um per metre more, 8 um at 0.4 m. With effects.load_encoder neither: the
/// scale reads the carriage itself.
#[test]
fn a_warm_screw_and_its_lead_error_move_the_real_position() {
    let table = "[accuracy]\naxes = [{ axis = \"x\", drive = \"screw\", rise_k = 10.0, lead_error = [[0.0, 0.0], [0.8, \
                 16.0e-6]] }]\n";
    let cold = run("screw-axis", &config("screw-axis", "", table), 3, 1);
    let warm = run("screw-axis", &config("screw-axis", "thermal_growth = true", table), 3, 1);
    let both = run("screw-axis", &config("screw-axis", "thermal_growth = true\nlead_error = true", table), 3, 1);
    let (c, w, b) = (&cold.axes[0], &warm.axes[0], &both.axes[0]);
    for (j, x) in c.targets.iter().enumerate() {
        let growth = STEEL_EXPANSION_PER_K * 10.0 * x;
        assert!((w.mean_up[j] - c.mean_up[j] - growth).abs() < 1e-9, "{x}: {} {}", w.mean_up[j], c.mean_up[j]);
        assert!((b.mean_down[j] - c.mean_down[j] - growth - 20e-6 * x).abs() < 1e-9, "{x}");
    }
    let mut scale = config("screw-axis", "thermal_growth = true\nlead_error = true", table);
    scale = scale.with("effects.load_encoder", true);
    let sim = Sim::new(sim_machines::find("screw-axis").unwrap(), &scale).unwrap();
    assert!(axis_errors(&scale, &sim).unwrap().is_empty());
}

/// A belt axis takes accuracy.belt_expansion_per_k, a heated part by its id, and refuses one the machine lacks.
#[test]
fn a_belt_axis_grows_with_its_heated_part() {
    let table = |part: &str| {
        format!(
            "[accuracy]\nbelt_expansion_per_k = 2.0e-5\naxes = [{{ axis = \"x\", drive = \"belt\", fixed_end = 0.1, \
             heated_part = \"{part}\" }}]\n"
        )
    };
    let entry = sim_machines::find("t-gantry").unwrap();
    let cfg = config("t-gantry", "thermal_growth = true", &table("guide_x"));
    let sim = Sim::new(entry, &cfg).unwrap();
    let errors = axis_errors(&cfg, &sim).unwrap();
    // 2e-5 / K * 4 K * (0.6 - 0.1) m = 40 um.
    assert!((errors[0].offset(0.6, 4.0) - 40e-6).abs() < 1e-15);
    assert_eq!(sim.heated[errors[0].part.unwrap()].part.id, "guide_x");
    let cfg = config("t-gantry", "thermal_growth = true", &table("guide_q"));
    assert!(axis_errors(&cfg, &sim).is_err());
}

/// The test runs on the four machines it was made for; its figures are finite, the reversal and accuracy within the
/// millimetre, and a tolerance in the ratings gives a verdict.
#[test]
fn the_test_runs_on_the_axes_the_gantry_and_the_delta() {
    std::thread::scope(|scope| {
        for id in ["single-axis", "screw-axis", "t-gantry", "delta-d3"] {
            scope.spawn(move || {
                let entry = sim_machines::find(id).unwrap();
                let text = entry.default_config.replacen(
                    "\n[ratings]",
                    "\n[ratings]\nmax_positioning_error_m = 1.0e-3\nmax_repeatability_m = 1.0e-9\n",
                    1,
                );
                let s = run(id, &load_config(entry, &text).unwrap(), 2, 1);
                assert!(!s.axes.is_empty(), "{id}");
                for a in &s.axes {
                    let figures = [a.reversal, a.repeatability, a.systematic, a.mean_range, a.accuracy];
                    assert!(figures.iter().all(|v| v.is_finite() && (0.0..1e-3).contains(v)), "{id}: {a:?}");
                }
                assert!(s.tool.accuracy.is_finite() && s.tool.repeatability.is_finite(), "{id}");
                let check = |c: &str| s.verdicts.iter().find(|v| v.check == c).unwrap().ok;
                assert!(check("positioning-accuracy"), "{id}: {:?}", s.verdicts);
                // Every one reverses by more than a nanometre.
                assert!(!check("positioning-repeatability"), "{id}: {:?}", s.verdicts);
            });
        }
    });
}

/// A machine without an axis in m (the rotary table, in rad) is judged against the `_rad` ratings and never against a
/// metre limit: its accuracy of some 5e-6 rad (4.58e-6 in the audit's run) passes 1e-4 rad, while metre limits of a
/// nanometre, which the rad figures would fail, give no verdict.
#[test]
fn a_rotary_axis_is_judged_in_rad() {
    let ratings = "
[ratings]
max_positioning_error_rad = 1.0e-4
max_repeatability_rad = 1.0e-4
                   max_positioning_error_m = 1.0e-9
max_repeatability_m = 1.0e-9
";
    let entry = sim_machines::find("rotary-table").unwrap();
    let cfg = load_config(
        entry,
        &entry.default_config.replacen(
            "
[ratings]",
            ratings,
            1,
        ),
    )
    .unwrap();
    let s = run("rotary-table", &cfg, 2, 1);
    let accuracy = s.axes.iter().map(|a| a.accuracy).fold(0.0, f64::max);
    println!("rotary table: accuracy {accuracy:e} rad, {:?}", s.verdicts);
    assert!(accuracy > 0.0 && accuracy < 1e-4, "{accuracy} rad");
    assert_eq!(s.verdicts.len(), 2, "{:?}", s.verdicts);
    for v in &s.verdicts {
        assert!(v.limit == 1e-4 && v.ok, "{v:?}");
    }
    let found = s.verdicts.iter().find(|v| v.check == "positioning-accuracy").unwrap();
    assert_eq!(found.value, accuracy);
}

trait With {
    fn with(self, key: &str, on: bool) -> Self;
}

impl With for Config {
    fn with(mut self, key: &str, on: bool) -> Self {
        self.set(key, toml::Value::Boolean(on)).unwrap();
        self
    }
}
