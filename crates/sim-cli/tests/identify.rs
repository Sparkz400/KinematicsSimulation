//! Vendor traces through `--map`, `identify` and `check-dataset` on the command line.

use std::path::{Path, PathBuf};
use std::process::{Command, Output};

fn cli(args: &[&str]) -> Output {
    Command::new(env!("CARGO_BIN_EXE_sim-cli")).args(args).output().unwrap()
}

fn text(bytes: &[u8]) -> String {
    String::from_utf8_lossy(bytes).into_owned()
}

fn datasets() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../datasets")
}

fn tmp(name: &str) -> PathBuf {
    Path::new(env!("CARGO_TARGET_TMPDIR")).join(name)
}

/// quantity -> (value, standard error) of an `identify` or `fit` table.
fn table(csv: &str, value: usize) -> Vec<(String, f64, f64)> {
    csv.lines()
        .skip(1)
        .map(|l| l.split(',').collect::<Vec<_>>())
        .map(|f| (f[0].to_string(), f[value].parse().unwrap(), f[value + 1].parse().unwrap_or(f64::NAN)))
        .collect()
}

#[test]
fn identify_reads_the_example_dataset_and_writes_an_overlay() {
    let dir = datasets().join("example-synthetic-belt-axis");
    let overlay = tmp("identify-overlay.csv");
    let out = cli(&[
        "identify",
        "--trace",
        dir.join("trace.csv").to_str().unwrap(),
        "--map",
        dir.join("map.toml").to_str().unwrap(),
        "--overlay",
        overlay.to_str().unwrap(),
    ]);
    assert!(out.status.success(), "{}", text(&out.stderr));
    let rows = table(&text(&out.stdout), 2);
    let get = |q: &str| rows.iter().find(|r| r.0 == q).unwrap_or_else(|| panic!("{q} missing: {rows:?}")).clone();
    // The one-motor machine at the motor: rotor 0.3e-4 kg m² plus pulleys, belt and carriage reflected.
    let inertia = get("inertia_kg_m2");
    assert!((inertia.1 - 1.049e-4).abs() < 0.01e-4 && inertia.2 < 0.01 * inertia.1, "{inertia:?}");
    assert!(get("friction_model_fit_percent").1 > 95.0);
    assert!(get("current_loop_gain").1 > 0.99);
    let header = std::fs::read_to_string(&overlay).unwrap().lines().next().unwrap().to_string();
    assert_eq!(
        header,
        "time,torque_measured,torque_model,current_loop_measured,current_loop_model,position_loop_measured,position_loop_model"
    );
}

#[test]
fn fit_through_a_mapping_gives_what_the_simulators_own_columns_give() {
    let run = tmp("map-run.csv");
    // The machine with one motor, by the registry: no machine is named outside its crate.
    let entry = sim_machines::MACHINES.iter().find(|m| (m.build)(&m.config()).unwrap().motors().len() == 1).unwrap();
    let out = cli(&[
        "run",
        entry.id,
        "--cycle",
        "--time",
        "2.5",
        "--set",
        "effects.payload=false",
        "--set",
        "mass.carriage_kg=5.5",
        "--out",
        run.to_str().unwrap(),
    ]);
    assert!(out.status.success(), "{}", text(&out.stderr));
    let map = tmp("map-run.toml");
    let id = (entry.build)(&entry.config()).unwrap().motors()[0].id;
    let mapping = format!(
        "format = \"csv\"\ntime = \"time\"\n[[motor]]\nsetpoint = \"motor.{id}.angle_setpoint\"\nangle = \"motor.{id}.angle\"\nspeed = \"motor.{id}.speed\"\ntorque = \"motor.{id}.torque\"\n"
    );
    std::fs::write(&map, mapping).unwrap();
    let fit = |extra: &[&str]| {
        let mut args = vec!["fit", entry.id, "--trace", run.to_str().unwrap(), "--set", "effects.payload=false"];
        args.extend(extra);
        let out = cli(&args);
        assert!(out.status.success(), "{}", text(&out.stderr));
        table(&text(&out.stdout), 3)
    };
    let (own, mapped) = (fit(&[]), fit(&["--map", map.to_str().unwrap()]));
    assert_eq!(own.len(), 4);
    for (a, b) in own.iter().zip(&mapped) {
        assert!((a.1 - b.1).abs() <= 1e-6 * a.1.abs().max(1e-3), "{a:?} {b:?}");
    }
}

#[test]
fn identify_fits_the_two_mass_model_to_a_sweep_file() {
    // Torque to speed of motor 1.2e-4 and load 4.8e-4 kg m² on 300 N m/rad, 0.01 N m s/rad.
    let (jm, jl, k, c) = (1.2e-4f64, 4.8e-4f64, 300.0f64, 0.01f64);
    let mut csv = String::from("frequency_hz,gain_rad_s_per_nm\n");
    for i in 0..150 {
        let f = 5.0 * (80.0f64).powf(i as f64 / 149.0);
        let w = std::f64::consts::TAU * f;
        let gain = (k - jl * w * w).hypot(c * w) / (w * (k * (jm + jl) - jm * jl * w * w).hypot(c * (jm + jl) * w));
        csv += &format!("{f},{gain}\n");
    }
    let sweep = tmp("two-mass-sweep.csv");
    std::fs::write(&sweep, csv).unwrap();
    let out = cli(&["identify", "--sweep", sweep.to_str().unwrap()]);
    assert!(out.status.success(), "{}", text(&out.stderr));
    let rows = table(&text(&out.stdout), 2);
    let get = |q: &str| rows.iter().find(|r| r.0 == q).unwrap().1;
    assert!((get("stiffness_nm_rad") - k).abs() < 0.01 * k, "{rows:?}");
    assert!((get("motor_side_inertia_kg_m2") - jm).abs() < 0.01 * jm, "{rows:?}");
}

#[test]
fn every_dataset_passes_the_checker_and_the_template_lacks_only_its_trace() {
    for entry in std::fs::read_dir(datasets()).unwrap().flatten().filter(|e| e.path().is_dir()) {
        let out = cli(&["check-dataset", entry.path().to_str().unwrap()]);
        let stdout = text(&out.stdout);
        if entry.file_name() == "template" {
            assert_eq!(stdout.lines().count(), 1, "{stdout}");
            assert!(stdout.contains("recording 1 (friction-map.csv): the file is missing"), "{stdout}");
        } else {
            assert!(out.status.success(), "{stdout}");
        }
    }
}

/// The measured ball screw bench (datasets/isw-ball-screw-axis): `identify` and `fit` on its S-profile give an inertia
/// at the motor between the hand calculation's bounds from the source's masses. Table 400 kg on a lead of 40 mm:
/// 400 (0.04 / 2 pi)^2 = 0.016211 kg m^2; rotor (D 21.4) 0.00207; the screw, steel 1.8 m long, between a shaft at its root
/// (33.7 mm, pi/32 7850 1.8 0.0337^4 = 0.00179) and at its nominal diameter (40 mm, 0.00355), and a coupling of up to
/// 0.0003. Low 0.016211 + 0.00207 = 0.01828 (no screw), high 0.016211 + 0.00207 + 0.00355 + 0.0003 = 0.02213 kg m^2;
/// as moved mass at the table (times (2 pi / 0.04)^2 = 24674) 451 to 546 kg. Measured 0.0199 kg m^2 and 492 kg.
#[test]
fn the_measured_ball_screw_has_the_inertia_its_masses_give() {
    let dir = datasets().join("isw-ball-screw-axis");
    let path = |name: &str| dir.join(name).to_str().unwrap().to_string();
    let trace = ["--trace", &path("s-profile-v300.csv"), "--map", &path("map.toml")];
    let out = cli(&[&["identify"][..], &trace].concat());
    assert!(out.status.success(), "{}", text(&out.stderr));
    let rows = table(&text(&out.stdout), 2);
    let inertia = rows.iter().find(|r| r.0 == "inertia_kg_m2").unwrap();
    assert!((0.01828..0.02213).contains(&inertia.1) && inertia.2 < 0.01 * inertia.1, "{inertia:?}");
    let machine = ["screw-axis", "--config", &path("machine.toml"), "--set", "start.pose=[0.01003171]"];
    let out = cli(&[&["fit"][..], &machine, &trace].concat());
    assert!(out.status.success(), "{}", text(&out.stderr));
    let rows = table(&text(&out.stdout), 3);
    let mass = rows.iter().find(|r| r.0 == "moved_mass_kg").unwrap();
    assert!((451.0..546.0).contains(&mass.1), "{rows:?}");
    let out = cli(&[&["replay"][..], &machine, &trace].concat());
    assert!(out.status.success(), "{}", text(&out.stderr));
}

/// Run from inside the folder, `check-dataset .` checks the id against the folder's own name. It checked it against "",
/// the name of `.`, and refused every dataset.
#[test]
fn the_checker_takes_the_name_of_the_folder_it_runs_in() {
    let out = Command::new(env!("CARGO_BIN_EXE_sim-cli"))
        .current_dir(datasets().join("isw-ball-screw-axis"))
        .args(["check-dataset", "."])
        .output()
        .unwrap();
    assert!(out.status.success(), "{}", text(&out.stdout));
}

#[test]
fn the_checker_finds_personal_data_and_stray_files() {
    let source = datasets().join("example-synthetic-belt-axis");
    let copy = tmp("example-synthetic-belt-axis");
    let _ = std::fs::remove_dir_all(&copy);
    std::fs::create_dir_all(&copy).unwrap();
    for name in ["dataset.toml", "map.toml", "trace.csv", "README.md"] {
        std::fs::copy(source.join(name), copy.join(name)).unwrap();
    }
    std::fs::write(copy.join("README.md"), "Recorded by jane.doe@example.com on the line at 10.1.2.3.\n").unwrap();
    std::fs::write(copy.join("project.zip"), "x").unwrap();
    let meta = std::fs::read_to_string(copy.join("dataset.toml")).unwrap();
    let meta = meta.replace("[machine]\n", "[machine]\nconfig = \"machine.toml\"\n");
    std::fs::write(copy.join("dataset.toml"), meta).unwrap();
    let out = cli(&["check-dataset", copy.to_str().unwrap(), "--lang", "de"]);
    assert_eq!(out.status.code(), Some(2));
    let stdout = text(&out.stdout);
    for expected in [
        "README.md, line 1: an e-mail address",
        "README.md, line 1: an IP address",
        "project.zip is not part of any recording",
        "machine.config: machine.toml is missing",
        "Problem im Datensatz",
    ] {
        assert!(stdout.contains(expected), "{expected} not in {stdout}");
    }
}
