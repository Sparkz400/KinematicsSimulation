//! Input shapers: trains of impulses that, convolved with a setpoint, cancel the vibration it starts in a mode of known
//! frequency and damping (Singer and Seering 1990, Singhose 1994–1997). Times are in half damped periods of the mode,
//! T_d / 2 = 1 / (2 f sqrt(1 - zeta^2)); amplitudes add up to one.
//!
//! Undamped, each shaper is a sum of impulses at multiples of the half period: ZV 1 : 1, ZVD 1 : 2 : 1, ZVDD
//! 1 : 3 : 3 : 1, EI (1+V)/4 : (1-V)/2 : (1+V)/4, the two-hump EI A : 1/2 - A : 1/2 - A : A with
//! A = (3X^2 + 2X + 3V^2) / (16 X), X = (V^2 (sqrt(1 - V^2) + 1))^(1/3), V the vibration the EI shapers allow: EI leaves
//! V at the design frequency and none a little either side of it, the two-hump EI none at it and humps of V either
//! side, so both stay at or below V over a wider band than ZVD (Singhose, Seering and Singer 1994–1997).
//!
//! Damped, impulse i is weighted by K^i, K = exp(-zeta pi / sqrt(1 - zeta^2)), and all by their sum: that cancels the
//! mode's decay between the impulses. For ZV, ZVD and ZVDD it is the textbook damped shaper, which leaves nothing at
//! the design frequency; an EI shaper of undamped weights a_i then leaves V K^n / sum(a_i K^i) there (n its last
//! impulse), less than V (0.0424 for V = 0.05 at zeta = 0.05).
// shortcut: the damped EI shapers are the undamped ones weighted by K^i (less than V at the design frequency, a
// slightly different insensitivity from Singhose's fitted damped EI); fit the damped EI if a damping above 0.1 matters.

use std::f64::consts::PI;

/// The kinds of shaper (`profile.shaper_type`, `type` of a `profile.shaper_modes` entry).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum Kind {
    Zv,
    #[default]
    Zvd,
    Zvdd,
    Ei,
    TwoHumpEi,
}

impl Kind {
    pub const NAMES: [&str; 5] = ["zv", "zvd", "zvdd", "ei", "ei2"];

    pub fn from_name(name: &str) -> Option<Self> {
        [Self::Zv, Self::Zvd, Self::Zvdd, Self::Ei, Self::TwoHumpEi].into_iter().find(|k| k.name() == name)
    }

    pub fn name(&self) -> &'static str {
        Self::NAMES[*self as usize]
    }
}

/// The impulses of a shaper of `kind` for a mode of damping ratio `damping`: (time in half damped periods, amplitude),
/// the amplitudes adding up to one. `tolerance`: the vibration V the EI shapers allow (0 to 1).
pub fn impulses(kind: Kind, damping: f64, tolerance: f64) -> Vec<(f64, f64)> {
    let v = tolerance;
    let undamped: Vec<f64> = match kind {
        Kind::Zv => vec![1.0, 1.0],
        Kind::Zvd => vec![1.0, 2.0, 1.0],
        Kind::Zvdd => vec![1.0, 3.0, 3.0, 1.0],
        Kind::Ei => vec![(1.0 + v) / 4.0, (1.0 - v) / 2.0, (1.0 + v) / 4.0],
        Kind::TwoHumpEi => {
            let x = (v * v * ((1.0 - v * v).sqrt() + 1.0)).cbrt();
            // At V = 0 it is ZVDD's 1/8 : 3/8 : 3/8 : 1/8.
            let a = if x > 0.0 { (3.0 * x * x + 2.0 * x + 3.0 * v * v) / (16.0 * x) } else { 0.125 };
            vec![a, 0.5 - a, 0.5 - a, a]
        }
    };
    let k = (-damping * PI / (1.0 - damping * damping).sqrt()).exp();
    let weighted: Vec<f64> = undamped.iter().enumerate().map(|(i, a)| a * k.powi(i as i32)).collect();
    let sum: f64 = weighted.iter().sum();
    weighted.iter().enumerate().map(|(i, a)| (i as f64, a / sum)).collect()
}

/// The shaper of two shapers in a row: every pair of impulses, at the sum of their times with the product of their
/// amplitudes. Times must be in the same unit.
pub fn convolve(a: &[(f64, f64)], b: &[(f64, f64)]) -> Vec<(f64, f64)> {
    a.iter().flat_map(|(ta, aa)| b.iter().map(move |(tb, ab)| (ta + tb, aa * ab))).collect()
}

/// The vibration a shaper leaves in a mode of `hz` and `damping`, as a share of what a single impulse leaves
/// (Singer and Seering): e^(-zeta w t_n) sqrt(C^2 + S^2), C and S the sums of A_i e^(zeta w t_i) cos and sin of
/// w_d t_i. `impulses` in seconds.
pub fn residual(impulses: &[(f64, f64)], hz: f64, damping: f64) -> f64 {
    let w = 2.0 * PI * hz;
    let wd = w * (1.0 - damping * damping).sqrt();
    let last = impulses.iter().fold(0.0, |m: f64, (t, _)| m.max(*t));
    let (c, s) = impulses.iter().fold((0.0, 0.0), |(c, s), (t, a)| {
        let e = a * (damping * w * (t - last)).exp();
        (c + e * (wd * t).cos(), s + e * (wd * t).sin())
    });
    (c * c + s * s).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A shaper designed for 10 Hz, its impulses in seconds: half periods of 1 / (2 f sqrt(1 - zeta^2)).
    fn at_10_hz(kind: Kind, damping: f64) -> Vec<(f64, f64)> {
        let half = 1.0 / (2.0 * 10.0 * (1.0 - damping * damping).sqrt());
        impulses(kind, damping, 0.05).into_iter().map(|(t, a)| (t * half, a)).collect()
    }

    /// ZV, undamped: two halves half a period apart leave |1 + e^(i pi r)| / 2 = |cos(pi r / 2)| at r times the design
    /// frequency: 0 at it, cos(0.4 pi) = 0.3090 at 0.8 and at 1.2. ZVD the square: 0.0955.
    #[test]
    fn zv_cancels_its_frequency_and_leaves_a_third_at_20_percent_off() {
        let zv = at_10_hz(Kind::Zv, 0.0);
        assert!(residual(&zv, 10.0, 0.0) < 1e-12);
        for hz in [8.0, 12.0] {
            assert!((residual(&zv, hz, 0.0) - 0.309_017).abs() < 1e-6, "{hz}: {}", residual(&zv, hz, 0.0));
            assert!((residual(&at_10_hz(Kind::Zvd, 0.0), hz, 0.0) - 0.309_017f64.powi(2)).abs() < 1e-6);
        }
    }

    /// EI with V = 5 %, undamped: 0.2625 : 0.475 : 0.2625 at 0, T/2, T leave |0.475 + 0.525 cos(pi r)|: 0.05 at the design
    /// frequency, |0.475 + 0.525 cos(0.8 pi)| = |0.475 - 0.42473| = 0.05027 at 0.8 and 1.2: within the 5 % over +-20 %, where
    /// ZV leaves 31 %.
    #[test]
    fn ei_leaves_its_tolerance_at_its_frequency_and_about_that_at_20_percent_off() {
        let ei = at_10_hz(Kind::Ei, 0.0);
        assert!(ei.iter().zip([0.2625, 0.475, 0.2625]).all(|(i, a)| (i.1 - a).abs() < 1e-15), "{ei:?}");
        assert!((residual(&ei, 10.0, 0.0) - 0.05).abs() < 1e-12);
        for hz in [8.0, 12.0] {
            let expected = (0.475 + 0.525 * (0.8 * PI).cos()).abs();
            assert!((expected - 0.050_266).abs() < 1e-6);
            assert!((residual(&ei, hz, 0.0) - expected).abs() < 1e-9, "{hz}: {}", residual(&ei, hz, 0.0));
        }
        // Damped (zeta = 0.05), K = exp(-0.05 pi / sqrt(1 - 0.0025)) = 0.85447: ZV still leaves nothing at its
        // frequency; EI leaves 0.05 K^2 / (0.2625 + 0.475 K + 0.2625 K^2) = 0.05 * 0.73012 / 0.86003 = 0.04245.
        assert!(residual(&at_10_hz(Kind::Zv, 0.05), 10.0, 0.05) < 1e-12);
        let k = (-0.05 * PI / (1.0f64 - 0.0025).sqrt()).exp();
        let expected = 0.05 * k * k / (0.2625 + 0.475 * k + 0.2625 * k * k);
        assert!((k - 0.85447).abs() < 1e-5 && (expected - 0.04245).abs() < 1e-5, "{k} {expected}");
        assert!((residual(&at_10_hz(Kind::Ei, 0.05), 10.0, 0.05) - expected).abs() < 1e-12);
    }

    /// The two-hump EI at V = 5 %: X = (0.0025 (sqrt(0.9975) + 1))^(1/3) = 0.17096, A = (3 X^2 + 2 X + 0.0075) / (16 X)
    /// = 0.15980. Undamped it leaves |8 A c^3 + (1 - 8 A) c|, c = cos(pi r / 2): nothing at the design frequency (c = 0),
    /// and humps where 24 A c^2 = 8 A - 1, of (2/3) (8 A - 1) sqrt((8 A - 1) / (24 A)) = (2/3) 0.27838 * 0.26942 = 0.0500:
    /// V, at c = 0.26942: r = (2 / pi) acos(c) = 0.8263 and 1.1737. From 7 to 13 Hz (c = cos(0.35 pi) = 0.45399 at the
    /// ends, |1.27838 * 0.09357 - 0.27838 * 0.45399| = 0.0068) the humps are the most it leaves; at 5 Hz (c = 0.7071) it
    /// leaves 0.255, outside the band it is insensitive over. At 0 vibration it is ZVDD.
    #[test]
    fn the_two_hump_ei_leaves_its_tolerance_at_its_humps_and_is_zvdd_without_one() {
        let x = (0.0025f64 * (0.9975f64.sqrt() + 1.0)).cbrt();
        let a = (3.0 * x * x + 2.0 * x + 0.0075) / (16.0 * x);
        assert!((x - 0.17096).abs() < 1e-5 && (a - 0.15980).abs() < 1e-5, "{x} {a}");
        let two = at_10_hz(Kind::TwoHumpEi, 0.0);
        assert!((two[0].1 - a).abs() < 1e-12 && (two[1].1 - (0.5 - a)).abs() < 1e-12, "{two:?}");
        assert!(residual(&two, 10.0, 0.0) < 1e-12, "{}", residual(&two, 10.0, 0.0));
        let hump = (0..=2000).map(|i| residual(&two, 7.0 + 6.0 * i as f64 / 2000.0, 0.0)).fold(0.0, f64::max);
        assert!((hump - 0.05).abs() < 1e-4, "{hump}");
        let zvdd: Vec<f64> = impulses(Kind::TwoHumpEi, 0.0, 0.0).iter().map(|i| i.1).collect();
        assert_eq!(zvdd, [0.125, 0.375, 0.375, 0.125]);
    }

    /// Two shapers convolved cancel both modes: ZV at 10 Hz and ZV at 25 Hz leave nothing at either.
    #[test]
    fn convolved_shapers_cancel_every_mode() {
        let b: Vec<(f64, f64)> = impulses(Kind::Zv, 0.0, 0.0).into_iter().map(|(t, a)| (t / 50.0, a)).collect();
        let both = convolve(&at_10_hz(Kind::Zv, 0.0), &b);
        assert_eq!(both.len(), 4);
        assert!((both.iter().map(|i| i.1).sum::<f64>() - 1.0).abs() < 1e-15);
        assert!(residual(&both, 10.0, 0.0) < 1e-12 && residual(&both, 25.0, 0.0) < 1e-12);
    }
}
