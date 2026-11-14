//! Geometry a machine file gives beside its netlist: the belt's wrap at a driven pulley from where the wheels stand
//! (effects.belt_wrap), and a body leaning on its carriage (effects.tool_flex).

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
/// Both distances shrink as the lever axis travels: dk/dz = 2 k^2 h / tilt. As the T-gantry crate's `flex_at`.
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
