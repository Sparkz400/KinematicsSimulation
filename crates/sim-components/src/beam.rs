//! Euler-Bernoulli beams for the structure's bending (effects.link_bending, the gantry's beam and Z axis): a tube's
//! second moment of area, a cantilever's first bending mode as one modal coordinate (its tip's deflection), and the
//! compliance of a simply supported span under a point load.
//!
//! The cantilever's first mode shape (Clough and Penzien, "Dynamics of Structures", 18.4), clamped at x = 0:
//!
//! ```text
//! phi(x) = cosh bx - cos bx - s (sinh bx - sin bx),   b L = 1.875104,   s = 0.734096,   phi(L) = 2
//! f = (b L)^2 / (2 pi) sqrt(E I / (m L^3))            m the beam's mass
//! ```
//!
//! Taken to its tip (phi(L) = 1): modal mass m / 4 plus a mass at the tip, modal stiffness (b L)^4 / 4 E I / L^3 =
//! 3.0906 E I / L^3 (the static 3 E I / L^3 within 3 %); a load spread evenly over the beam drives it with 0.3915 of its
//! sum, a moment at the tip with 1.3765 / L of it, and the tip turns by 1.3765 / L of its deflection.

/// b L of a cantilever's first mode.
pub const CANTILEVER_BETA_L: f64 = 1.875_104_068_711_961;
/// The share of an evenly spread load that drives the first mode taken to its tip: the integral of phi over L.
pub const CANTILEVER_PARTICIPATION: f64 = 0.391_496;
/// The first mode's slope at the tip times L, taken to its tip: what a moment at the tip drives it with, times 1 / L.
pub const CANTILEVER_TIP_SLOPE: f64 = 1.376_51;

/// The cantilever's first mode shape taken to its tip, phi(xi) / phi(1) at xi = x / L in [0, 1] (the module's phi).
pub fn cantilever_shape(xi: f64) -> f64 {
    let (b, s) = (CANTILEVER_BETA_L, 0.734_095_514);
    let phi = |x: f64| (b * x).cosh() - (b * x).cos() - s * ((b * x).sinh() - (b * x).sin());
    phi(xi) / phi(1.0)
}

/// A cantilever's static deflection shape under a tip load, taken to its tip: psi(xi) = (3 xi^2 - xi^3) / 2, xi = x / L.
/// With the static stiffness 3 E I / L^3 it is the consistent single mode (Rayleigh): the tip deflects as the exact
/// beam's under a tip load and under its own weight (int psi = 3/8: q L^4 / (8 E I)), and the first frequency is the
/// exact one's from above, +1.5 % for the bare beam and under +0.26 % with the delta arms' elbows (the exact mode's
/// shape with 3 E I / L^3 reads it 0.8 to 1 % low there and the sag under the tube's weight 4.4 % high,
/// `cantilever_first_frequency`). Its mass share: int psi^2.
pub const STATIC_SHAPE_MASS: f64 = 33.0 / 140.0;
/// The static shape's int psi: what an evenly spread load drives it with.
pub const STATIC_SHAPE_PARTICIPATION: f64 = 3.0 / 8.0;
/// The static shape's int(xi psi): the first moment about the root of what moves with it, over m L.
pub const STATIC_SHAPE_MOMENT: f64 = 11.0 / 40.0;
/// The static shape's slope at the tip times L: the tip turns by 3/2 of its deflection over L.
pub const STATIC_SHAPE_TIP_SLOPE: f64 = 1.5;

/// The first bending frequency (rad/s) of a uniform Euler-Bernoulli cantilever, exactly: bending stiffness `ei`, its own
/// mass `beam` over `length`, a mass `tip` and a rotary inertia `tip_inertia` (about the axis the tip turns about) at its
/// free end. The lowest root z = b L of the clamped-free characteristic equation with the tip's shear -M w^2 w(L) and
/// moment J w^2 w'(L), r = M / m, j = J / (m L^2):
///
/// ```text
/// [(ch + c) - j z^3 (sh + s)] [(ch + c) + r z (sh - s)] - [(sh + s) - j z^3 (ch - c)] [(sh - s) + r z (ch - c)] = 0
/// w = z^2 sqrt(E I / (m L^3))           (ch, sh, c, s of z)
/// ```
///
/// by bisection below 1.875104 (the bare beam's); `beam` > 0.
pub fn cantilever_first_frequency(ei: f64, beam: f64, tip: f64, tip_inertia: f64, length: f64) -> f64 {
    let (r, j) = (tip / beam, tip_inertia / (beam * length * length));
    let det = |z: f64| {
        let (ch, sh, (s, c)) = (z.cosh(), z.sinh(), z.sin_cos());
        let (j3, rz) = (j * z.powi(3), r * z);
        ((ch + c) - j3 * (sh + s)) * ((ch + c) + rz * (sh - s))
            - ((sh + s) - j3 * (ch - c)) * ((sh - s) + rz * (ch - c))
    };
    let (mut low, mut high) = (1e-6, CANTILEVER_BETA_L);
    let below = det(low) > 0.0;
    for _ in 0..200 {
        let mid = 0.5 * (low + high);
        if (det(mid) > 0.0) == below { low = mid } else { high = mid }
    }
    low * low * (ei / (beam * length.powi(3))).sqrt()
}

/// The second moment of area of a round tube, outer diameter `outer` and wall `wall` (m): pi / 64 (D^4 - d^4). A wall of
/// half the diameter or more is a solid bar.
pub fn tube_second_moment(outer: f64, wall: f64) -> f64 {
    let inner = (outer - 2.0 * wall).max(0.0);
    std::f64::consts::PI / 64.0 * (outer.powi(4) - inner.powi(4))
}

/// A cantilever's first bending mode in its tip's deflection: modal mass (kg) and stiffness (N/m), for bending
/// stiffness `ei` (N m^2), its own mass `beam` spread evenly over `length` and `tip` at its end.
pub fn cantilever_mode(ei: f64, beam: f64, tip: f64, length: f64) -> (f64, f64) {
    (0.25 * beam + tip, CANTILEVER_BETA_L.powi(4) / 4.0 * ei / length.powi(3))
}

/// The deflection under a unit load at `at` along a simply supported span `span` (m/N): a^2 b^2 / (3 E I L), b = L - a.
/// L^3 / (48 E I) in the middle.
pub fn span_compliance(ei: f64, span: f64, at: f64) -> f64 {
    let a = at.clamp(0.0, span);
    let b = span - a;
    a * a * b * b / (3.0 * ei * span)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_uniform_cantilever_alone_rings_at_the_euler_bernoulli_frequency() {
        // Steel tube 60 x 4 mm, 0.8 m, 4.5 kg: I = pi/64 (0.06^4 - 0.052^4) = 2.77264e-7 m^4, EI = 2.1e11 I = 58225 N m^2.
        let ei = 2.1e11 * tube_second_moment(0.06, 0.004);
        assert!((ei - 58_225.52).abs() < 0.01, "EI {ei}");
        let (m, k) = cantilever_mode(ei, 4.5, 0.0, 0.8);
        // f = 1.875104^2 / (2 pi) sqrt(EI / (m L^3)) = 3.51602 / 6.28319 sqrt(58225.52 / (4.5 * 0.512)) = 88.96 Hz.
        let euler_bernoulli = 3.516_015 / std::f64::consts::TAU * (ei / (4.5 * 0.8f64.powi(3))).sqrt();
        let f = (k / m).sqrt() / std::f64::consts::TAU;
        assert!((f - euler_bernoulli).abs() < 1e-3, "{f} Hz against {euler_bernoulli} Hz");
        assert!((f - 89.0).abs() < 0.1, "{f}");
    }

    #[test]
    fn a_tip_load_bends_the_mode_as_far_as_the_static_cantilever_within_three_per_cent() {
        let (_, k) = cantilever_mode(1000.0, 1.0, 0.0, 0.5);
        // F L^3 / (3 EI) = 0.125 / 3000 per newton.
        assert!(((1.0 / k) / (0.125 / 3000.0) - 1.0).abs() < 0.03);
    }

    #[test]
    fn a_central_load_sags_a_simple_span_by_f_l3_over_48_ei() {
        // 1.2 m span, EI = 4e5 N m^2, 500 N in the middle: 500 * 1.728 / (48 * 4e5) = 45 um.
        assert!((500.0 * span_compliance(4.0e5, 1.2, 0.6) - 45.0e-6).abs() < 1e-12);
        // At a quarter: a^2 b^2 / (3 EI L) = 0.09 * 0.81 / (3 * 4e5 * 1.2) per newton, 9/16 of the middle's.
        assert!((span_compliance(4.0e5, 1.2, 0.3) / span_compliance(4.0e5, 1.2, 0.6) - 0.5625).abs() < 1e-12);
    }

    #[test]
    fn the_mode_constants_follow_from_its_shape() {
        let (b, s) = (CANTILEVER_BETA_L, 0.734_095_514);
        let phi = |x: f64| (b * x).cosh() - (b * x).cos() - s * ((b * x).sinh() - (b * x).sin());
        let tip = phi(1.0);
        assert!((tip - 2.0).abs() < 1e-5);
        // Integrals over x from 0 to 1 by the midpoint rule.
        let n = 100_000;
        let (mut area, mut square) = (0.0, 0.0);
        for i in 0..n {
            let x = (i as f64 + 0.5) / n as f64;
            area += phi(x) / tip / n as f64;
            square += (phi(x) / tip).powi(2) / n as f64;
        }
        assert!((area - CANTILEVER_PARTICIPATION).abs() < 1e-5, "{area}");
        assert!((square - 0.25).abs() < 1e-5, "{square}");
        let slope = (phi(1.0) - phi(1.0 - 1e-6)) / 1e-6 / tip;
        assert!((slope - CANTILEVER_TIP_SLOPE).abs() < 1e-4, "{slope}");
    }

    /// The exact first frequency against the textbook's b L: 1.875104 bare, 1.247917 with a tip mass equal to the
    /// beam's (Blevins, "Formulas for natural frequency and mode shape", table 8-8: lambda = 1.24792). And the static
    /// shape's integrals by the midpoint rule: 3/8, 11/40, 33/140, its tip slope 3/2.
    #[test]
    fn the_exact_cantilever_with_a_tip_mass_has_the_textbook_frequency() {
        let w = |tip: f64, j: f64| cantilever_first_frequency(1.0, 1.0, tip, j, 1.0);
        assert!((w(0.0, 0.0) - CANTILEVER_BETA_L.powi(2)).abs() < 1e-9, "{}", w(0.0, 0.0));
        assert!((w(1.0, 0.0).sqrt() - 1.247_917).abs() < 2e-6, "{}", w(1.0, 0.0).sqrt());
        // A rotary inertia at the tip lowers it further; scaling: z^2 sqrt(EI / (m L^3)).
        assert!(w(1.0, 0.05) < w(1.0, 0.0));
        let scaled = cantilever_first_frequency(4.0, 2.0, 2.0, 0.0, 0.5);
        assert!((scaled - w(1.0, 0.0) * (4.0f64 / (2.0 * 0.125)).sqrt()).abs() < 1e-9, "{scaled}");
        let psi = |x: f64| (3.0 * x * x - x * x * x) / 2.0;
        let n = 100_000;
        let (mut area, mut moment, mut square) = (0.0, 0.0, 0.0);
        for i in 0..n {
            let x = (i as f64 + 0.5) / n as f64;
            (area, moment, square) =
                (area + psi(x) / n as f64, moment + x * psi(x) / n as f64, square + psi(x).powi(2) / n as f64);
        }
        assert!((area - STATIC_SHAPE_PARTICIPATION).abs() < 1e-9 && (moment - STATIC_SHAPE_MOMENT).abs() < 1e-9);
        assert!((square - STATIC_SHAPE_MASS).abs() < 1e-9 && psi(1.0) == 1.0);
        assert!(((psi(1.0) - psi(1.0 - 1e-7)) / 1e-7 - STATIC_SHAPE_TIP_SLOPE).abs() < 1e-6);
    }
}
