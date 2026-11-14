//! The catalogue's mechanics as the simulator uses them: other makers' gearboxes and which motor each fits, ball
//! screws, profile rail guides and linear motors, each against a figure worked out by hand from the catalogue's rows.

use sim_components::catalog::{
    GUIDE_CARRIAGES, GUIDE_STATIC_SAFETY, apply_axis, apply_gearbox, apply_motor, axes_for, gearbox_mass,
    gearboxes_for_motor, linear_motor_articles_for, mech_axes, paired_drive,
};
use sim_components::drive::DriveParams;
use sim_components::transmission::BallScrew;
use sim_core::Config;

fn close(a: f64, b: f64, tolerance: f64) -> bool {
    (a - b).abs() <= tolerance * b.abs()
}

fn screw_axis() -> Config {
    machine_joint_kit::SCREW.config()
}

fn linear_axis() -> Config {
    machine_joint_kit::LINEAR_MOTOR.config()
}

/// HIWIN's FDW 32-10B2 (ball_screws.csv): root diameter 27.324 mm... read from the row so the figure follows the data;
/// the hand calculation is in the comments.
#[test]
fn a_catalogue_screws_allowed_speed_agrees_with_the_makers_critical_speed_formula() {
    let mut cfg = screw_axis();
    apply_axis(&mut cfg, "25-5B2-FDW", None).unwrap();
    let screw = BallScrew::from_config(&cfg).unwrap();
    // 25-5B2-FDW: root diameter 22.324 mm; the screw axis: 1000 mm between the bearings, fixed-supported.
    assert!(close(screw.root_diameter, 0.022324, 1e-12));
    // HIWIN (ball_screw_mountings.csv, fixed-supported): Nc = f dr / L^2 10^7 = 18.6719 * 22.324 / 1000^2 * 1e7
    //   = 4168.3 r/min, and the permissible Np = 0.8 Nc = 3334.6 r/min.
    // The simulator: the first bending mode of a steel beam of the root diameter, (lambda / L)^2 sqrt(E I / (rho A))
    //   = 3.926602^2 * 0.022324 / 4 * sqrt(2.06e11 / 7850) = 15.4182 * 0.005581 * 5122.75 = 440.8 rad/s
    //   = 4209.4 r/min: 1.0 % above HIWIN's, whose factor holds its own E and rho.
    assert!(close(screw.critical_speed_rpm(), 4209.4, 1e-3), "{}", screw.critical_speed_rpm());
    assert!(close(screw.allowed_speed_rpm(), 0.8 * 4168.3, 0.015), "{}", screw.allowed_speed_rpm());
}

#[test]
fn a_guide_sets_life_is_the_cube_of_its_carriages_rating_over_the_load() {
    let mut cfg = linear_axis();
    apply_axis(&mut cfg, "HGH25CAx4", None).unwrap();
    // HGH25CA: C 34900 N for 50 km, C0 52820 N (linear_guides.csv); four carriages.
    let rating = cfg.f64("ratings.guide_dynamic_load_rating_n").unwrap();
    assert_eq!(GUIDE_CARRIAGES, 4.0);
    assert!(close(rating, 4.0 * 34900.0, 1e-12));
    assert!(close(cfg.f64("ratings.guide_max_load_n").unwrap(), 4.0 * 52820.0 / GUIDE_STATIC_SAFETY, 1e-12));
    // L10 = (C / P)^3 * 50 km (analysis::guide_life_km) at a mean load of 20000 N on the set:
    //   (139600 / 20000)^3 * 50 = 6.98^3 * 50 = 340.07 * 50 = 17003.6 km.
    let life = (rating / 20000.0_f64).powi(3) * 50.0;
    assert!(close(life, 17003.6, 1e-4), "{life}");
}

#[test]
fn a_screw_too_fast_for_the_axis_is_refused_before_any_motor_is_tried() {
    let cfg = screw_axis();
    let axes = mech_axes(&cfg);
    let refusal = |name: &str| axes.iter().find(|(n, ..)| n == name).map(|(_, r, _)| *r);
    // 0.8 m/s on a 5 mm lead is 9600 r/min: above 0.8 x 4168 r/min of the 25 x 5 at 1 m (and its d n of 70000).
    assert_eq!(refusal("25-5B2-FDW"), Some("axis-speed"));
    // A 25 x 25... the longest lead of the 25 mm screws carries it where the catalogue has one: 1920 r/min at 25 mm.
    assert!(axes.iter().any(|(_, r, _)| r.is_empty()), "{axes:?}");
    // The screw config rates no guides: the screws are tried alone, and `axes_for` lists them.
    assert!(axes.iter().all(|(n, ..)| !n.contains('+')));
    assert_eq!(axes_for(&cfg).len(), axes.len());
}

#[test]
fn a_linear_motors_force_follows_its_current_and_force_constant() {
    let mut cfg = linear_axis();
    let article = "1FN3050-2WC00-0xAx";
    assert!(linear_motor_articles_for("s120").contains(&article));
    apply_motor(&mut cfg, article, "s120").unwrap();
    // linear_motors_1fn3.csv: kF 73.6 N/A, FN 200 N at IN 2.72 A, FMAX 550 N at IMAX 8.15 A, coil 3 kg.
    assert!(close(cfg.f64("motor.torque_constant_nm_a").unwrap(), 73.6, 1e-12));
    assert!(close(cfg.f64("motor.rotor_inertia_kgm2").unwrap(), 3.0, 1e-12));
    assert!(close(cfg.f64("linear_motor.attraction_force_n").unwrap(), 996.0, 1e-12));
    let drive = DriveParams::from_config(&cfg, 62.5e-6, 125e-6).unwrap();
    // Saturation s = 1 - FMAX / (kF IMAX) = 1 - 550 / 599.84 = 0.08309; F(I) = kF I (1 - s (I / IMAX)^2).
    // At IMAX: 73.6 * 8.15 * (1 - 0.08309) = 550 N. At IN: 73.6 * 2.72 * (1 - 0.08309 * 0.11139) = 198.3 N, within 1 %
    // of the printed FN.
    assert!(close(drive.torque_of(8.15), 550.0, 1e-9), "{}", drive.torque_of(8.15));
    assert!(close(drive.torque_of(2.72), 198.34, 1e-3), "{}", drive.torque_of(2.72));
    // The S120 Motor Module for it: the smallest Single Motor Module whose rated current carries IN.
    assert!(!paired_drive(article, "s120").is_empty());
}

#[test]
fn siemens_offers_its_sp_gearbox_on_the_1ft2_only() {
    // The equipment manual, chapter 3.1: "Coaxial precision gearbox only for 1FT2".
    let on_1fk2 = gearboxes_for_motor("1FK2105-4AF", "siemens-sp");
    assert!(on_1fk2.is_empty(), "{on_1fk2:?}");
    let on_1ft2 = gearboxes_for_motor("1FT2105-4AF", "siemens-sp");
    assert!(on_1ft2.contains(&"SP100-05-20".to_string()), "{on_1ft2:?}");
    // The same frame takes the NRB on either motor.
    assert!(!gearboxes_for_motor("1FK2105-4AF", "siemens-nrb").is_empty());
    // As a size prefix, as the search took it before: the same.
    assert!(gearboxes_for_motor("1FK2105-4AF", "SP100").is_empty());
}

#[test]
fn the_reduced_backlash_option_and_the_sp_mass_come_from_the_table() {
    let mut cfg = linear_axis();
    apply_gearbox(&mut cfg, "SP100-05-20-r").unwrap();
    // gearbox_sp.csv: 3 arcmin reduced (Table 6-68), 5 standard; 7.3 kg (Table 6-72, the 1FT2105's column).
    assert!(close(cfg.f64("gearbox.backlash_rad").unwrap(), 3.0 / 60.0 * std::f64::consts::PI / 180.0, 1e-9));
    apply_gearbox(&mut cfg, "SP100-05-20").unwrap();
    assert!(close(cfg.f64("gearbox.backlash_rad").unwrap(), 5.0 / 60.0 * std::f64::consts::PI / 180.0, 1e-9));
    assert_eq!(gearbox_mass("SP100-05-20"), Some(7.3));
    assert_eq!(gearbox_mass("SP100-05-20-r"), Some(7.3));
    // A strain wave gear brings its stiffness curve: K1 for the stiffness, K2, K3 and the knees.
    apply_gearbox(&mut cfg, "CSF32-14-100").unwrap();
    assert!(cfg.f64("gearbox.stiffness_k3_nm_rad").unwrap() > cfg.f64("gearbox.stiffness_nm_rad").unwrap());
}
