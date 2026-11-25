//! The machine's own parts against each other along a path (`PlanCheck::own`): a SCARA's forearm folded over its column,
//! two movers of the track brought to one place, and the shipped cycles of every machine.

use sim_components::collision::{Body, PlanCheck, check_cycle, check_path};
use sim_components::{Sim, analysis};
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
/// 0.0965 m: the least 0.06265 m, which passes a clearance of 10 mm. Both poses lie beyond joint 2's range in the URDF
/// (-2.5 to 2.5 rad), which the check does not judge: they show the check, not a collision the SCARA can reach.
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

/// The same SCARA on a compliant base (effects.base_compliance: the arm wrapped in the base's model, `base::Based`)
/// keeps its links and their joints: folded back it names the flange box and the column at -5 mm, as the bare arm does
/// (the base tilts under the arm's weight and the box keeps its axes while the column's capsule leans: 1.2 um more,
/// within 10 um). Fails if the wrapper does not forward
/// `self_collision_capsules` and `connected_bodies`.
#[test]
fn a_scara_on_a_compliant_base_keeps_its_links() {
    let entry = find("scara").unwrap();
    let mut cfg = entry.config();
    cfg.set_text("effects.base_compliance = true").unwrap();
    let mut machine = (entry.build)(&cfg).unwrap();
    machine.set_payload_attached(false);
    let path = [vec![0.0, 0.5, 0.1, 0.0], vec![0.0, std::f64::consts::PI, 0.1, 0.0]];
    let mut folded = PlanCheck::keeping(0.01);
    check_path(machine.as_ref(), &[], &path, &mut folded);
    let own = folded.own.expect("the SCARA on its base has parts to check");
    assert_eq!(own.bodies, [Body::Box(0), Body::Capsule(0)], "{own:?}");
    assert!((own.clearance + 0.005).abs() < 1e-5, "{own:?}");
}

/// The mover track's six movers (default.toml, six boxes of 0.06 x 0.06 x 0.05 m, payload off) stand 0.5 m apart along
/// the loop; none of the boxes touches another at the start, so none is joined. Mover 2 driven from s = 0.628 to 0.125,
/// where mover 1 stands, runs into its box: the overlap is the least of the two boxes' overlaps along each axis
/// (`Aabb::gap`), 0.05 m in z once the 0.06 m along the track overlap by 0.05 or more, from 0.01 m short of mover 1,
/// s = 0.135: -0.05 m between boxes 0 and 1, first at the last sample (1 mm apart) up to s = 0.135.
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
    assert!((own.clearance + 0.05).abs() < 1e-9 && (0.134..=0.135 + 1e-9).contains(&own.pose[1]), "{own:?}");
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
    let mut failed = Vec::new();
    for entry in MACHINES {
        let cfg = entry.config();
        let limit = cfg.non_negative("profile.min_clearance_m").unwrap();
        // Without obstacles `None` is "not checked"; with obstacles `own` None is.
        let Some(check) = check_cycle(entry, &cfg).unwrap() else { continue };
        if check.own_fails(limit) {
            failed.push(format!("{} (limit {limit}): {:?}", entry.id, check.own));
        }
    }
    assert!(failed.is_empty(), "{failed:#?}");
}

/// Parts are joined as they are at the machine's start pose, not at the first pose of a path. The mover track's movers
/// stand 0.5 m apart at the start, so nothing is joined there; a path that begins with mover 2 on mover 1 (s = 0.125
/// both) is reported at that first pose, -0.05 m between boxes 0 and 1. Taken at the first pose, the two would be
/// joined and nothing said.
#[test]
fn parts_are_joined_at_the_start_pose_not_at_a_paths_first() {
    let entry = find("mover-track").unwrap();
    let cfg = entry.config();
    let mut machine = (entry.build)(&cfg).unwrap();
    let start = cfg.f64s("start.pose").unwrap();
    let (mut state, mut torque) = (Vec::new(), vec![0.0; machine.motors().len()]);
    machine.equilibrium(&start, &mut state, &mut torque);
    let mut check = PlanCheck::at_start(0.0, machine.as_ref(), &state, &start, || (entry.build)(&cfg)).unwrap();
    machine.set_payload_attached(false);
    let mut together = start.clone();
    together[1] = start[0];
    check_path(machine.as_ref(), &[], &[together.clone(), start], &mut check);
    let own = check.own.expect("the movers are not joined at the start");
    assert_eq!(own.bodies, [Body::Box(0), Body::Box(1)], "{own:?}");
    assert!((own.clearance + 0.05).abs() < 1e-9 && own.pose == together, "{own:?}");
}

/// A slide's payload sits on its carriage (carriage 0.06 m high from z = 0, payload 0.08 m from z = 0.06): the two
/// faces meet, 7e-18 m apart by rounding, and are joined. The screw axis has no other parts, so its own parts are not
/// checked, and the run has no verdict "self-collision". Fails if the rounding leaves the two parts apart, to be judged
/// on that noise.
#[test]
fn a_payload_on_its_carriage_is_joined_and_the_slide_is_not_checked() {
    for id in ["screw-axis", "rack-axis", "linear-motor-axis"] {
        let entry = find(id).unwrap();
        let cfg = entry.config();
        assert_eq!(check_cycle(entry, &cfg).unwrap(), None, "{id}");
        let load = analysis::analyse_cycle(&mut Sim::new(entry, &cfg).unwrap()).unwrap();
        assert_eq!(load.own_clearance, None, "{id}");
    }
}

/// Two parts brought nearer than `profile.min_clearance_m` are judged and refused as an obstacle hit is. The six-axis
/// arm's shipped cycle (config/arm.toml) brings box 2 within 13.4 mm of the link of capsule 5 at its first target, the
/// joint angles (0.8, 0.45, -0.55, 0.3, 0.5, 1.0) (`sim-cli verdicts urdf-arm` tells that planned figure); the two are
/// more than 50 mm apart at the start, or they would be joined. With 50 mm asked for, the cycle's verdict "self-collision" fails, its
/// value the run's least own clearance (below 50 mm); and the move from the start to that target, planned
/// time-optimal, is refused with `error-path-self-collision`. At the shipped 0 it passes.
#[test]
fn parts_nearer_than_the_clearance_fail_the_verdict_and_a_time_optimal_plan_is_refused() {
    let entry = find("urdf-arm").unwrap();
    let mut cfg = entry.config();
    let load = analysis::analyse_cycle(&mut Sim::new(entry, &cfg).unwrap()).unwrap();
    let verdicts = analysis::verdicts(&cfg, &load).unwrap();
    assert!(verdicts.iter().any(|v| v.check == "self-collision" && v.ok), "{verdicts:?}");

    cfg.set_text("profile.min_clearance_m = 0.05").unwrap();
    let mut sim = Sim::new(entry, &cfg).unwrap();
    let line = [sim_components::path::Piece::Line(vec![0.8, 0.45, -0.55, 0.3, 0.5, 1.0])];
    assert_eq!(sim.plan_path(&line, true).unwrap_err().key, "error-path-self-collision");
    let load = analysis::analyse_cycle(&mut sim).unwrap();
    let own = load.own_clearance.unwrap();
    let verdicts = analysis::verdicts(&cfg, &load).unwrap();
    let verdict = verdicts.iter().find(|v| v.check == "self-collision").unwrap();
    assert!(own < 0.05 && !verdict.ok && verdict.value == own && verdict.limit == 0.05, "{verdict:?}");
}

/// The point the SCARA draws at its flange (capsule 4, of no length) sits at the quill's end, the quill's 0.2 m from the
/// second arm's end (capsule 2) whatever the pose: 0.2 - 0.025 - 0.015 = 0.16 m between the two always, which a clearance
/// above it would fail on every cycle. The two are joined, as the quill (capsule 3) is to both. Fails if a link of no
/// length is joined only to the link it ends.
#[test]
fn the_scaras_flange_point_is_joined_to_the_arm_before_its_quill() {
    let entry = find("scara").unwrap();
    let mut machine = (entry.build)(&entry.config()).unwrap();
    machine.set_payload_attached(false);
    let (mut state, mut torque) = (Vec::new(), vec![0.0; machine.motors().len()]);
    machine.equilibrium(&[0.3, 1.0, 0.1, 0.0], &mut state, &mut torque);
    let (boxes, capsules) = (machine.collision_bodies(&state).len(), machine.self_collision_capsules(&state).len());
    assert_eq!((boxes, capsules), (1, 5));
    let joined = machine.connected_bodies(&state, boxes, capsules).unwrap();
    for pair in [(1 + 2, 1 + 3), (1 + 3, 1 + 4), (1 + 2, 1 + 4)] {
        assert!(joined.contains(&pair), "{pair:?} not in {joined:?}");
    }
    assert!(!joined.contains(&(1 + 1, 1 + 3)), "{joined:?}");
}

/// The run's least own clearance goes with a snapshot and stays over a reconfiguration, as the obstacles' does: the
/// six-axis arm looks at its parts after its first step (the start pose), a simulation restored from that snapshot has
/// the same figure, and one reconfigured with the same config keeps it.
#[test]
fn the_own_clearance_goes_with_a_snapshot_and_stays_over_a_reconfiguration() {
    let entry = find("urdf-arm").unwrap();
    let cfg = entry.config();
    let mut sim = Sim::new(entry, &cfg).unwrap();
    sim.step();
    let seen = sim.own_parts.own.clone().expect("the arm has links to look at");
    let mut other = Sim::new(entry, &cfg).unwrap();
    assert_eq!(other.own_parts.own, None);
    other.restore(&sim.snapshot()).unwrap();
    assert_eq!(other.own_parts.own.as_ref(), Some(&seen));
    sim.reconfigure(entry, &cfg).unwrap();
    assert_eq!(sim.own_parts.own, Some(seen));
}
