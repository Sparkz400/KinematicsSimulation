//! Geometry a machine file gives beside its netlist: the belt's wrap at a driven pulley from where the wheels stand
//! (effects.belt_wrap) and the belt drawn round its wheels, and a body leaning on its carriage (effects.tool_flex).

use crate::netlist::LeanNode;
use std::f64::consts::TAU;

/// A wheel the belt runs round, in the belt's plane (x, z of the 3D view): its centre, and its radius signed by the
/// sense the belt runs round it, + anticlockwise, - clockwise (seen from the front, x to the right, z up). A clamp is a
/// wheel of radius 0.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Wheel {
    pub centre: [f64; 2],
    pub radius: f64,
}

/// The direction (unit vector) of the belt's straight run from wheel `a` to wheel `b`. Where the belt touches a wheel,
/// the wheel's centre lies to its left (anticlockwise) or right (clockwise): the point of contact is c - rho l, with rho
/// the signed radius and l the run's direction t turned left by 90 degrees. Both points lie on one line along t, so
/// d = c_b - c_a = lambda t + (rho_b - rho_a) l: with D = |d| and e = d / D, t = cos(alpha) e + sin(alpha) e_left,
/// sin(alpha) = -(rho_b - rho_a) / D. Wheels too close for a run (|rho_b - rho_a| > D) get the nearest direction.
fn run(a: Wheel, b: Wheel) -> [f64; 2] {
    let d = [b.centre[0] - a.centre[0], b.centre[1] - a.centre[1]];
    let length = d[0].hypot(d[1]);
    let e = [d[0] / length, d[1] / length];
    let sin = (-(b.radius - a.radius) / length).clamp(-1.0, 1.0);
    let cos = (1.0 - sin * sin).sqrt();
    [cos * e[0] - sin * e[1], cos * e[1] + sin * e[0]]
}

/// Where the belt's straight run from wheel `a` to wheel `b` leaves `a` and meets `b`: c - rho l on each, l the run's
/// direction (`run`) turned left by 90 degrees.
pub fn contacts(a: Wheel, b: Wheel) -> [[f64; 2]; 2] {
    let t = run(a, b);
    let left = [-t[1], t[0]];
    [a, b].map(|w| [w.centre[0] - w.radius * left[0], w.centre[1] - w.radius * left[1]])
}

/// Points on wheel `at` from the belt's contact `from` round to its contact `to`, in the wheel's sense (anticlockwise
/// for a positive radius), `pieces` + 1 of them, both ends included. A clamp (radius 0) gives its centre.
pub fn arc(at: Wheel, from: [f64; 2], to: [f64; 2], pieces: usize) -> Vec<[f64; 2]> {
    let angle = |p: [f64; 2]| (p[1] - at.centre[1]).atan2(p[0] - at.centre[0]);
    let (start, sense) = (angle(from), at.radius.signum());
    let span = ((angle(to) - start) * sense).rem_euclid(TAU);
    (0..=pieces)
        .map(|k| {
            let a = start + sense * span * k as f64 / pieces as f64;
            [at.centre[0] + at.radius.abs() * a.cos(), at.centre[1] + at.radius.abs() * a.sin()]
        })
        .collect()
}

/// The angle (rad) by which the belt wraps wheel `at`, coming from wheel `from` and going on to wheel `to`: how far the
/// belt's direction turns round it, in the wheel's sense, from 0 to a whole turn. For the shipped crates' layouts this is
/// `mech::wrap_angle`: half a turn less the lean of the two runs.
pub fn wrap(from: Wheel, at: Wheel, to: Wheel) -> f64 {
    let (a, b) = (run(from, at), run(at, to));
    let turn = (a[0] * b[1] - a[1] * b[0]).atan2(a[0] * b[0] + a[1] * b[1]);
    (turn * at.radius.signum()).rem_euclid(TAU)
}

/// The spring that holds a leaning body upright at the pose `q`: (its stiffness k at the centre of mass, dk per metre of
/// the lever axis, the tool tip's travel per metre the centre of mass moves). The guide gives way sideways (lateral) and
/// in tilt; a force at the distance h below it tilts it with the arm h, so the compliance at the centre of mass is
/// 1 / lateral + h^2 / tilt, and the tool tip, h_tool below, moves k (1 / lateral + h h_tool / tilt) per metre of it.
/// Both distances shrink as the lever axis travels: dk/dz = 2 k^2 h / tilt, the leaning axis's own law (`flex_at` in its crate).
pub fn flex_at(lean: &LeanNode, q: &[f64]) -> (f64, f64, f64) {
    let z = lean.lever_axis.map_or(0.0, |a| q[a]);
    let (h, h_tool) = (lean.com_drop - z, lean.tool_drop - z);
    let k = 1.0 / (1.0 / lean.lateral + h * h / lean.tilt);
    (k, 2.0 * k * k * h / lean.tilt, k * (1.0 / lean.lateral + h * h_tool / lean.tilt))
}

/// How far a point at distance `d` below the carriage rises as the leaning body is held out by a sideways force F at its
/// centre of mass, `h` below the carriage, per F^2 (`lean_rise` of the crate with the guide's tilt only: a hanging bar
/// turned by theta is shortened by theta^2 / 2 per length, and theta = F h / tilt along the whole of it, so the rise is
/// 1/2 d (h / tilt)^2; where `h` is negative nothing hangs below the carriage and only the length beyond counts).
pub fn lean_rise(lean: &LeanNode, d: f64, h: f64) -> f64 {
    let (theta, hanging) = (h / lean.tilt, h.max(0.0));
    let inside = d.min(hanging);
    0.5 * (inside + (d - inside).max(0.0)) * theta * theta
}

/// The weight's geometric stiffness of the lean, and its rate per metre of the lever axis: the body's weight m g at its
/// centre of mass rises by F^2 `lean_rise` as the spring's force F = k u holds it out, V = m g F^2 r = 1/2 c u^2 with
/// c = 2 k^2 m g r (the body hangs like a pendulum, c > 0 where its centre of mass is below the carriage). The rate is
/// the central difference over 2e-6 m, as the crate has it. Both are 0 without gravity.
pub fn lean_weight(lean: &LeanNode, q: &[f64], mass: f64, g: f64) -> (f64, f64) {
    let at = |step: f64| {
        let z = lean.lever_axis.map_or(0.0, |a| q[a]) + step;
        let (k, h) = (1.0 / (1.0 / lean.lateral + (lean.com_drop - z).powi(2) / lean.tilt), lean.com_drop - z);
        2.0 * k * k * mass * g * lean_rise(lean, h, h)
    };
    let step = 1e-6;
    let rate = if lean.lever_axis.is_some() && g != 0.0 { (at(step) - at(-step)) / (2.0 * step) } else { 0.0 };
    (at(0.0), rate)
}

/// How far a point `d` below the carriage (above it where negative) is out sideways, per newton at the leaning body's
/// centre of mass `h` below the carriage (`lean_reach` of the crate without bending): the guide's give 1 / lateral plus
/// its tilt h d / tilt.
pub fn lean_reach(lean: &LeanNode, d: f64, h: f64) -> f64 {
    1.0 / lean.lateral + h * d / lean.tilt
}

/// The tool tip's rise as the body leans, rho = a(z) u^2 (`rise_coefficients` of the crate): the tip, `tool_drop - z`
/// below the carriage, rises by k^2 `lean_rise` per u^2 (the leaning bar is shortened by theta^2 / 2 per length, with the
/// spring's force F = k u). Returns [a, da/dz, d2a/dz2] by central differences over 1e-4 m of the lever axis; zeros
/// without one (a body that leans with no axis to rise along).
pub fn rise_coefficients(lean: &LeanNode, q: &[f64]) -> [f64; 3] {
    let Some(axis) = lean.lever_axis else { return [0.0; 3] };
    let a = |z: f64| {
        let (h, h_tool) = (lean.com_drop - z, lean.tool_drop - z);
        let k = 1.0 / (1.0 / lean.lateral + h * h / lean.tilt);
        k * k * lean_rise(lean, h_tool, h)
    };
    let (z, step) = (q[axis], 1e-4);
    let (here, ahead, behind) = (a(z), a(z + step), a(z - step));
    [here, (ahead - behind) / (2.0 * step), (ahead - 2.0 * here + behind) / (step * step)]
}

/// How the tool tip's travel per metre of lean (`flex_at`) changes along the lever axis: [d tip / dz, d2 tip / dz2] by
/// central differences over 1e-4 m (`tip_slope` of the crate); zeros without a lever axis. The process force's sideways
/// part works on tip(z) u, so it pushes on the lever axis with F tip' u.
pub fn tip_slope(lean: &LeanNode, q: &[f64]) -> [f64; 2] {
    let Some(axis) = lean.lever_axis else { return [0.0; 2] };
    let tip = |step: f64| {
        let mut at = q.to_vec();
        at[axis] += step;
        flex_at(lean, &at).2
    };
    let (here, ahead, behind, step) = (tip(0.0), tip(1e-4), tip(-1e-4), 1e-4);
    [(ahead - behind) / (2.0 * step), (ahead - 2.0 * here + behind) / (step * step)]
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn close(a: f64, b: f64) {
        assert!((a - b).abs() < 1e-12, "{a} against {b}");
    }

    /// The belt drawn round its wheels, by hand. Two wheels of r = 0.1 m 1 m apart, both anticlockwise: the run
    /// leaves the first at its bottom, (0, -0.1), and meets the second at (1, -0.1). Crossed, the second clockwise:
    /// the run leans by asin(0.2 / 1) (0.2 the sum of the radii), its direction (0.979796, 0.2), so it leaves the first
    /// at -0.1 (-0.2, 0.979796) = (0.02, -0.0979796) and meets the second at (1, 0) + 0.1 (-0.2, 0.979796) =
    /// (0.98, 0.0979796), each 0.1 from its centre and at right angles to the run. Round the first wheel
    /// anticlockwise from its bottom to its top the belt passes (0.1, 0), half way.
    #[test]
    fn the_belt_meets_its_wheels_at_the_tangents_and_runs_round_them_in_their_sense() {
        let near = |a: [f64; 2], b: [f64; 2]| assert!((a[0] - b[0]).hypot(a[1] - b[1]) < 1e-6, "{a:?} against {b:?}");
        let (one, two) = (Wheel { centre: [0.0, 0.0], radius: 0.1 }, Wheel { centre: [1.0, 0.0], radius: 0.1 });
        let [a, b] = contacts(one, two);
        near(a, [0.0, -0.1]);
        near(b, [1.0, -0.1]);
        let [a, b] = contacts(one, Wheel { radius: -0.1, ..two });
        near(a, [0.02, -0.097_979_6]);
        near(b, [0.98, 0.097_979_6]);
        for (p, c) in [(a, one.centre), (b, two.centre)] {
            close((p[0] - c[0]).hypot(p[1] - c[1]), 0.1);
        }
        close((b[0] - a[0]) * 0.2 - (b[1] - a[1]) * 0.96f64.sqrt(), 0.0);
        let round = arc(one, [0.0, -0.1], [0.0, 0.1], 4);
        near(round[0], [0.0, -0.1]);
        near(round[2], [0.1, 0.0]);
        near(round[4], [0.0, 0.1]);
        let back = arc(Wheel { radius: -0.1, ..one }, [0.0, -0.1], [0.0, 0.1], 4);
        near(back[2], [-0.1, 0.0]);
    }

    /// The shipped crates' layout (`mech::wrap_angle`): spans leaving level give half a turn; an inner tangent to a wheel
    /// 0.06 m down and 0.3 m along, of radius 0.012, on a pulley of 0.02 leans by atan2(0.06, 0.3) - asin(0.032 /
    /// hypot(0.3, 0.06)) = 0.1973956 - 0.1047869 = 0.0926086 rad, which the wrap loses.
    #[test]
    fn the_wrap_is_half_a_turn_less_the_lean_of_both_runs() {
        let pulley = Wheel { centre: [0.0, 0.0], radius: 0.02 };
        let clamp = Wheel { centre: [0.5, 0.02], radius: 0.0 };
        let level_below = Wheel { centre: [0.8, -0.032], radius: -0.012 };
        close(wrap(clamp, pulley, level_below), PI);
        let lower = Wheel { centre: [0.3, -0.06], radius: -0.012 };
        let lean = 0.06f64.atan2(0.3) - (0.032 / 0.3f64.hypot(0.06)).asin();
        close(wrap(clamp, pulley, lower), PI - lean);
        assert!((wrap(clamp, pulley, lower) - (PI - 0.092_608_6)).abs() < 1e-7);
        // Mirrored, the belt runs round each wheel the other way, and wraps as far; so it does followed backwards.
        let mirror = |w: Wheel| Wheel { centre: [-w.centre[0], w.centre[1]], radius: -w.radius };
        close(wrap(mirror(clamp), mirror(pulley), mirror(lower)), PI - lean);
        let back = |w: Wheel| Wheel { radius: -w.radius, ..w };
        close(wrap(back(lower), back(pulley), back(clamp)), PI - lean);
    }
}
