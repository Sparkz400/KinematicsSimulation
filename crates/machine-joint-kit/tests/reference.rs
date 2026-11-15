//! The arms' rigid-body dynamics against an independent library: MuJoCo reads the same URDF files
//! (`reference/mujoco_reference.py`, pinned in `reference/requirements.txt`) and gives, at 60 random states inside the
//! joints' limits per URDF, the inverse dynamics with gravity, the mass matrix, the gravity torques alone and the
//! flange's pose, in `reference/<urdf>.csv`. Here the same from the machine as built from its config: its loss-free
//! rigid torques with the rotors held still (`Machine::rigid_split_torque` with no rotor acceleration: the mechanism's
//! M q'' + h alone, through the motors' ratios and the palletiser's `arm.joint_coupling`), and the flange from the
//! URDF's chain (`rigid::Chain::tip`). Payload, swing, bending links and base compliance off: none is in the reference.

use machine_joint_kit::{ARM_ON_TRACK, ARM5_TOOL_CONFIG, ARM7_TOOL_CONFIG, PALLETISER, SCARA, URDF_ARM};
use sim_components::load_config;
use sim_components::rigid::Chain;
use sim_core::{Config, Machine};

/// Relative to the largest magnitude of the quantity in the row; MuJoCo and the chain differ by rounding only.
const TOLERANCE: f64 = 1e-9;

fn rows(csv: &str) -> Vec<Vec<f64>> {
    csv.lines()
        .filter(|l| !l.starts_with('#') && !l.starts_with('q'))
        .map(|l| l.split(',').map(|v| v.parse().unwrap()).collect())
        .collect()
}

fn mechanism_only(mut cfg: Config) -> Config {
    for line in [
        "effects.gravity = true",
        "effects.payload = false",
        "effects.payload_swing = false",
        "effects.link_bending = false",
        "effects.base_compliance = false",
        "kinematics.tool_space = false",
    ] {
        cfg.set_text(line).unwrap();
    }
    cfg
}

/// The worst of |a - b| over the largest |b|.
fn worst(a: &[f64], b: &[f64]) -> f64 {
    let scale = b.iter().fold(1e-12_f64, |m, v| m.max(v.abs()));
    a.iter().zip(b).map(|(x, y)| (x - y).abs()).fold(0.0, f64::max) / scale
}

fn compare(name: &str, cfg: Config, urdf: &str, csv: &str) {
    let m: Box<dyn Machine> = (machine_entry(name).build)(&cfg).unwrap();
    let chain = Chain::from_urdf(urdf).unwrap();
    let n = m.motors().len();
    let coupling: Vec<Vec<f64>> = match cfg.get("arm.joint_coupling").unwrap().as_array().unwrap() {
        rows if rows.is_empty() => (0..n).map(|r| (0..n).map(|c| (r == c) as u8 as f64).collect()).collect(),
        rows => rows.iter().map(|r| r.as_array().unwrap().iter().map(|v| v.as_float().unwrap()).collect()).collect(),
    };
    // Motor radians per joint unit: the tool is the joints.
    let mut ratio = vec![0.0; n];
    m.inverse(&vec![1.0; n], &mut ratio);
    let torque = |q: &[f64], qd: &[f64], qdd: &[f64]| -> Vec<f64> {
        let at_motor = |x: &[f64]| -> Vec<f64> { x.iter().zip(&ratio).map(|(v, r)| v * r).collect() };
        let mut out = vec![0.0; n];
        m.rigid_split_torque(&at_motor(q), &at_motor(qd), &at_motor(qdd), &vec![0.0; n], &mut out);
        out.iter().zip(&ratio).map(|(t, r)| t * r).collect()
    };
    let reference = rows(csv);
    assert!(reference.len() >= 50, "{name}: {} states", reference.len());
    let mut worst_seen = [0.0_f64; 5];
    for row in &reference {
        let (q, qd, qdd) = (&row[..n], &row[n..2 * n], &row[2 * n..3 * n]);
        let (tau, gravity, mass) = (&row[3 * n..4 * n], &row[4 * n..5 * n], &row[5 * n..5 * n + n * n]);
        let (pos, rot) = (&row[5 * n + n * n..5 * n + n * n + 3], &row[5 * n + n * n + 3..]);

        let zero = vec![0.0; n];
        let at_rest = torque(q, &zero, &zero);
        let ours_mass: Vec<f64> = (0..n * n)
            .map(|rc| {
                let mut unit = zero.clone();
                unit[rc % n] = 1.0;
                torque(q, &zero, &unit)[rc / n] - at_rest[rc / n]
            })
            .collect();
        let full: Vec<f64> = coupling.iter().map(|r| r.iter().zip(q).map(|(c, v)| c * v).sum()).collect();
        let (tip_rot, tip_pos) = chain.tip(&full);
        let ours_rot: Vec<f64> = tip_rot.iter().flatten().copied().collect();
        let errors = [
            worst(&torque(q, qd, qdd), tau),
            worst(&at_rest, gravity),
            worst(&ours_mass, mass),
            worst(&tip_pos, pos),
            worst(&ours_rot, rot),
        ];
        for (w, e) in worst_seen.iter_mut().zip(errors) {
            *w = w.max(e);
        }
    }
    println!("{name}: worst relative error of torque, gravity, mass matrix, flange position, rotation: {worst_seen:?}");
    assert!(worst_seen.iter().all(|e| *e < TOLERANCE), "{name} disagrees with MuJoCo: {worst_seen:?}");
}

fn machine_entry(name: &str) -> &'static sim_core::MachineEntry {
    match name {
        "scara" => &SCARA,
        "palletiser" => &PALLETISER,
        "arm-on-track" => &ARM_ON_TRACK,
        _ => &URDF_ARM,
    }
}

#[test]
fn six_axis_arm_matches_mujoco() {
    compare(
        "six-axis",
        mechanism_only(URDF_ARM.config()),
        include_str!("../urdf/six-axis.urdf"),
        include_str!("../reference/six-axis.csv"),
    );
}

#[test]
fn scara_matches_mujoco() {
    compare(
        "scara",
        mechanism_only(SCARA.config()),
        include_str!("../urdf/scara.urdf"),
        include_str!("../reference/scara.csv"),
    );
}

#[test]
fn five_axis_arm_matches_mujoco() {
    compare(
        "five-axis",
        mechanism_only(load_config(&URDF_ARM, ARM5_TOOL_CONFIG).unwrap()),
        include_str!("../urdf/five-axis.urdf"),
        include_str!("../reference/five-axis.csv"),
    );
}

#[test]
fn seven_axis_arm_matches_mujoco() {
    compare(
        "seven-axis",
        mechanism_only(load_config(&URDF_ARM, ARM7_TOOL_CONFIG).unwrap()),
        include_str!("../urdf/seven-axis.urdf"),
        include_str!("../reference/seven-axis.csv"),
    );
}

/// The track's prismatic joint in one chain with the arm: its force carries the arm's reaction along the rails, and its
/// acceleration reaches every joint of the arm. Through the rack's ratio (motor radians per metre).
#[test]
fn arm_on_track_matches_mujoco() {
    compare(
        "arm-on-track",
        mechanism_only(ARM_ON_TRACK.config()),
        include_str!("../urdf/arm-on-track.urdf"),
        include_str!("../reference/arm-on-track.csv"),
    );
}

#[test]
fn palletiser_matches_mujoco() {
    compare(
        "palletiser",
        mechanism_only(PALLETISER.config()),
        include_str!("../urdf/palletiser.urdf"),
        include_str!("../reference/palletiser.csv"),
    );
}
