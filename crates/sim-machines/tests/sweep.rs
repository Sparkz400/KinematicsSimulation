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

/// Over the default sweep the measured lowest dip (21.6 Hz on the T-gantry, 20.7 Hz on the EXCT-30) is a mode the belt
/// hardly moves; the fit sets a measured dip against the config's dip of the same mode, not the lowest against the belt
/// mode (before: × 0.103 and × 0.400 for a config swept against itself). "Measured" on the config itself and with the
/// belt 1.5 times as stiff, the factor read is 1 and 1.5 within the standard error the fit states.
#[test]
fn the_sweep_fit_compares_like_modes_over_the_default_sweep() {
    let gantry = sim_machines::find("t-gantry").unwrap();
    let frequencies = log_spaced(5.0, 500.0, SWEEP_POINTS);
    for (cfg, factors) in
        [(gantry.config(), &[1.0, 1.5][..]), (Config::parse(machine_t_gantry::EXCT_30_CONFIG).unwrap(), &[1.0][..])]
    {
        let amplitude = default_sweep_amplitude(gantry, &cfg).unwrap();
        let base = cfg.f64("belt.stiffness_ea_n").unwrap();
        for &factor in factors {
            let mut measured = cfg.clone();
            measured.set_text(&format!("belt.stiffness_ea_n = {:?}", base * factor)).unwrap();
            let measured = frequency_sweep(gantry, &measured, 0, &frequencies, amplitude).unwrap();
            let fit = sim_components::identify::fit_sweep(gantry, &cfg, 0, &measured, amplitude).unwrap().unwrap();
            let (read, error) = fit.stiffness();
            assert!(fit.config.frequency > 30.0, "x {factor}: {fit:?}");
            assert!((read - factor).abs() <= error && error < 0.1 * factor, "x {factor}: {read} +- {error}, {fit:?}");
        }
    }
}

/// The sweep fit tells the measured mode by how the config's dips move with the belt's stiffness
/// (`identify::fit_sweep`): on the T-gantry swept from 30 to 200 Hz, "measured" on the config with its belt 0.5, 1.8 and
/// 2.5 times as stiff, it sets a belt mode of the config against the measured dip of the same mode, never the measured
/// 60 Hz mode moved to 84 Hz (at 2.5) against the config's 102 Hz dip, which the nearest dip took before (factor 0.68).
/// The factor is the one at which the config's dip is the measured one, found by sweeping the config at it: the mode
/// moves as k^0.37 to k^0.45 here, so the pure belt law (f_measured / f_config)^2 read 0.539, 1.58 and 1.98. Each true
/// factor is recovered within the standard error the fit states. (The config's own dip at 59.90 Hz; 59.91 Hz before the
/// belt's vertical runs weighed on Z, 21 November.)
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
    assert!((own.config.frequency - 59.90).abs() < 0.01 && own.stiffness().0 == 1.0, "{own:?}");
    for factor in [0.5, 1.8, 2.5] {
        let found = fit(factor);
        let (read, error) = found.stiffness();
        eprintln!("belt x {factor}: {:?} -> x {read} +- {error}, exponent {}", found.fitted.frequency, found.exponent);
        // A belt mode of the config, the 60 Hz one or the 102 Hz one set against the measured dip of the same mode:
        // both read the same factor (at 0.5 the 102 Hz one against 72.6 Hz, × 0.500).
        assert!(found.config.frequency > 55.0, "x {factor}: {found:?}");
        assert!((read - factor).abs() < error && error < 0.1 * factor, "x {factor}: {read} +- {error}");
        assert!(found.exponent > 0.3 && found.exponent < 0.5, "x {factor}: {}", found.exponent);
    }
}
