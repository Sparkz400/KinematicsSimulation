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
            assert!((found.frequency / expected - 1.0).abs() < 0.03 && found.damping > 0.0, "{} motor {motor}: {found:?}", entry.id);
        }
    }
}
