//! The frequency sweep as the app and `sim-cli sweep` run it unless told otherwise.

use sim_components::analysis::{SWEEP_POINTS, default_sweep_amplitude, frequency_sweep, log_spaced};
use sim_components::identify::resonance;
use sim_core::Config;

/// The default sweep (5 to 500 Hz, `SWEEP_POINTS` frequencies, `default_sweep_amplitude`) finds the load's
/// lowest dip on every motor of every machine's default config and of the EXCT-30. With 0.05 N m and 60
/// frequencies, the default before, stiction held the motors of both default machines and nothing was found.
/// Measured: T-gantry 21.6 Hz on both motors, EXCT-30 20.8 Hz, single axis 84.3 Hz; the T-gantry also has dips
/// near 62 and 107 Hz (the one at 62 Hz about as deep), which are not the one reported.
#[test]
fn the_default_sweep_finds_the_lowest_dip_on_every_config() {
    let gantry = sim_machines::find("t-gantry").unwrap();
    let configs = [
        (gantry, gantry.config(), 21.6),
        (gantry, Config::parse(machine_t_gantry::EXCT_30_CONFIG).unwrap(), 20.8),
        (sim_machines::find("single-axis").unwrap(), sim_machines::find("single-axis").unwrap().config(), 84.3),
    ];
    let frequencies = log_spaced(5.0, 500.0, SWEEP_POINTS);
    for (entry, cfg, expected) in configs {
        let amplitude = default_sweep_amplitude(entry, &cfg).unwrap();
        // Within 40 % of the drive's peak torque.
        let resolved = sim_components::catalog::resolve(entry, &cfg).unwrap();
        let peak = resolved.f64("drive.peak_torque_nm").unwrap();
        assert!(amplitude <= 0.4 * peak, "{}: {amplitude} N m of {peak}", entry.id);
        let motors = (entry.build)(&resolved).unwrap().motors().len();
        for motor in 0..motors {
            let found = resonance(&frequency_sweep(entry, &cfg, motor, &frequencies, amplitude).unwrap());
            let found = found.unwrap_or_else(|| panic!("{} motor {motor}: no dip at {amplitude} N m", entry.id));
            assert!(
                (found.frequency / expected - 1.0).abs() < 0.03 && found.damping > 0.0,
                "{} motor {motor}: {found:?}",
                entry.id
            );
        }
    }
}

/// The sweep fit tells the measured mode by how the config's dips move with the belt's stiffness
/// (`identify::fit_sweep`): on the T-gantry swept from 30 to 200 Hz, "measured" on the config with its belt 0.5, 1.8 and
/// 2.5 times as stiff, it takes the config's 60 Hz dip, the one the measured lowest dip is (it is the config's own at 1).
/// At 2.5 the measured dip (84 Hz) lies nearer the config's 102 Hz dip, which the nearest dip took before (factor 0.68).
/// The factor it then gives, (f_measured / f_config)^2, is the pure belt law's: the mode moves as k^0.37 to k^0.45 here,
/// so it reads 0.539 for 0.5 and below 1.8 and 2.5 (`SweepFit::stiffness`).
#[test]
fn the_sweep_fit_tells_the_belt_mode_by_how_it_moves() {
    let entry = sim_machines::find("t-gantry").unwrap();
    let cfg = entry.config();
    let frequencies = log_spaced(30.0, 200.0, 50);
    let base = cfg.f64("belt.stiffness_ea_n").unwrap();
    let fit = |factor: f64| {
        let mut measured = cfg.clone();
        measured.set_text(&format!("belt.stiffness_ea_n = {:?}", base * factor)).unwrap();
        let measured = frequency_sweep(entry, &measured, 0, &frequencies, 1.5).unwrap();
        sim_components::identify::fit_sweep(entry, &cfg, 0, &measured, 1.5).unwrap().unwrap()
    };
    let own = fit(1.0);
    assert!((own.config.frequency - 59.91).abs() < 0.01 && own.stiffness().0 == 1.0, "{own:?}");
    for factor in [0.5, 1.8, 2.5] {
        let found = fit(factor);
        eprintln!("belt x {factor}: {:?} -> {:?}", (found.config.frequency, found.fitted.frequency), found.stiffness());
        assert_eq!(found.config.frequency, own.config.frequency, "x {factor}: {found:?}");
        let (read, _) = found.stiffness();
        assert!(
            if factor < 1.0 { read > factor && read < 1.0 } else { read > 1.0 && read < factor },
            "x {factor}: {read}"
        );
    }
}
