//! The machine's own parts against each other along a path (`PlanCheck::own`): a SCARA's forearm folded over its column,
//! two movers of the track brought to one place, and the shipped cycles of every machine.

use sim_components::collision::{Body, PlanCheck, check_cycle, check_path};
use sim_machines::{MACHINES, find};

/// The SCARA (config/scara.toml, urdf/scara.urdf, payload off so that its 0.08 m cube is not there) draws its links as
/// bars of 0.05 m (radius 0.025): capsule 0 the column from the floor to joint 1 at (0, 0, 0.4); capsule 1 the first
/// arm to joint 2 at (0.325 cos q1, 0.325 sin q1, 0.4); capsule 2 the second arm to the quill at 0.275 further along
/// the angle q1 + q2, q3 below 0.4; capsule 3 the quill, 0.2 down to the flange; capsule 4 a point at the flange. Its
/// flange is box 0, 0.06 m on a side, at the end of the quill.
///
/// Folded back with q = (0, pi, 0.1, 0): the second arm runs from joint 2 at (0.325, 0, 0.4) to the quill's top at
/// (0.325 - 0.275, 0, 0.4 - 0.1) = (0.05, 0, 0.3), the quill from there to (0.05, 0, 0.1), where the flange is, its box
/// x from 0.02 to 0.08, z from 0.07 to 0.13. The column's axis is at x = 0, radius 0.025: the flange box is
/// 0.02 away from the axis, 0.02 - 0.025 = -0.005 m (an overlap of 5 mm) from the column. The second arm's end and the
/// quill touch it (the bars' axes 0.05 m apart, radii summing to 0.05: 0), the flange point is 0.05 - 0.025 - 0.015 =
/// 0.01 m clear, the first arm is 0.3 - 0.1 = 0.05 m clear of the quill's top (0.1 between the axes, less the radii
/// 0.05). The nearest are the flange box and the column: -0.005 m, at the last pose; the two are not joined (the
/// flange is joined to the second arm, the quill and its point, the column to the first arm only). Opened by 0.4 rad
/// (23 degrees: q2 = pi - 0.4): the quill's top at (0.325 - 0.275 cos 0.4, 0.275 sin 0.4, 0.3) = (0.07171, 0.10709,
/// 0.3), the flange box x 0.04171 and y 0.07709 clear of the column's axis: sqrt(0.04171^2 + 0.07709^2) - 0.025 =
/// 0.06265 m, the second arm's end and the quill 0.1289 - 0.05 = 0.0789 m, the first arm and the quill 0.1465 - 0.05 =
/// 0.0965 m: the least 0.06265 m, which passes a clearance of 10 mm.
#[test]
fn a_scara_folded_over_its_column_is_reported_with_the_two_parts_and_opened_it_passes() {
    let entry = find("scara").unwrap();
    let mut machine = (entry.build)(&entry.config()).unwrap();
    machine.set_payload_attached(false);
    let path = |q2: f64| vec![vec![0.0, 0.5, 0.1, 0.0], vec![0.0, q2, 0.1, 0.0]];

    let mut folded = PlanCheck::keeping(0.01);
    check_path(machine.as_ref(), &[], &path(std::f64::consts::PI), &mut folded);
    let own = folded.own.clone().expect("the SCARA has parts to check");
    assert_eq!(own.bodies, [Body::Box(0), Body::Capsule(0)], "{own:?}");
    assert!((own.clearance + 0.005).abs() < 1e-9, "{own:?}");
    assert!((own.pose[1] - std::f64::consts::PI).abs() < 1e-9, "{own:?}");
    assert!(folded.own_fails(0.01) && folded.own_fails(0.0));

    let mut opened = PlanCheck::keeping(0.01);
    check_path(machine.as_ref(), &[], &path(std::f64::consts::PI - 0.4), &mut opened);
    let own = opened.own.clone().unwrap();
    assert!((own.clearance - 0.06265).abs() < 1e-4, "{own:?}");
    assert!(!opened.own_fails(0.01), "{own:?}");
}

/// The mover track's six movers (default.toml, six boxes of 0.06 x 0.06 x 0.05 m, payload off) stand 0.5 m apart along
/// the loop; none of the boxes touches another at the start, so none is joined. Mover 2 driven from s = 0.628 to 0.125, where mover 1 stands, meets the same box:
/// identical boxes overlap by their least size, 0.05 m in z, so -0.05 m, between boxes 0 and 1, at the last pose.
#[test]
fn two_movers_brought_to_the_same_place_are_reported() {
    let entry = find("mover-track").unwrap();
    let mut machine = (entry.build)(&entry.config()).unwrap();
    machine.set_payload_attached(false);
    let start = vec![0.125, 0.628116, 1.131233, 1.634349, 2.137465, 2.640582];
    let mut together = start.clone();
    together[1] = start[0];
    let mut check = PlanCheck::keeping(0.0);
    check_path(machine.as_ref(), &[], &[start.clone(), together.clone()], &mut check);
    let own = check.own.clone().expect("six boxes to check");
    assert_eq!(own.bodies, [Body::Box(0), Body::Box(1)], "{own:?}");
    assert!((own.clearance + 0.05).abs() < 1e-9 && (own.pose[1] - together[1]).abs() < 1e-9, "{own:?}");
    assert!(check.own_fails(0.0));

    // The start pose alone keeps them apart.
    let mut apart = PlanCheck::keeping(0.0);
    check_path(machine.as_ref(), &[], std::slice::from_ref(&start), &mut apart);
    let own = apart.own.clone().unwrap();
    assert!(own.clearance > 0.0 && !apart.own_fails(0.0), "{own:?}");
}

/// The shipped cycle of every machine keeps its parts clear of each other by `profile.min_clearance_m` (0 as shipped:
/// no overlap), or says it did not check. Expected above 20 s: every machine's cycle every millimetre. A failure here
/// is a finding or a body drawn too coarse; the check is not to be loosened for it.
#[test]
fn the_shipped_cycle_of_every_machine_keeps_its_parts_apart_or_is_not_checked() {
    for entry in MACHINES {
        let cfg = entry.config();
        let limit = cfg.non_negative("profile.min_clearance_m").unwrap();
        // Without obstacles `None` is "not checked"; with obstacles `own` None is.
        let Some(check) = check_cycle(entry, &cfg).unwrap() else { continue };
        assert!(!check.own_fails(limit), "{}: {:?}", entry.id, check.own);
    }
}
