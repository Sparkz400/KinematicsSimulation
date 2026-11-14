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
