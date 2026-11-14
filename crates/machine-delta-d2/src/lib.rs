//! Delta picker D2, the planar delta: two motor-driven arms in the vertical x-z plane, one at each side of the base,
//! each carrying a pair of forearms down to a plate that keeps its orientation (a five-bar whose lower links are
//! parallelograms, as in Codian's D2 or ABB's two-axis FlexPicker). The tool is the plate's centre, x and z.
//!
//! The D3's equations with two legs at the azimuths 0 and 180 degrees and the plate in the plane y = 0. Leg i points
//! along s_i x (s_1 = +1, s_2 = -1); e = base radius - platform radius (the plate's offset folded into the base, as it
//! never turns), the arm angle theta_i (0 horizontal, positive down), upper arm L_a, forearm L_b:
//!
//! ```text
//! elbow     c_i = (s_i (e + L_a cos theta), -L_a sin theta)
//! constraint |p - c_i|^2 = L_b^2
//! inverse   A cos theta + B sin theta + C = 0, A = -2 L_a (x' - e), B = 2 L_a z, C = (x' - e)^2 + z^2 + L_a^2 - L_b^2
//!           theta = 2 atan((-B - sqrt(A^2 + B^2 - C^2)) / (C - A))     (elbow out; x' = s_i x)
//! forward   p = the lower of the two points at L_b from both elbows (two circles)
//! Jacobian  J_x p' = J_q theta', J_x rows u_i = p - c_i, J_q = diag(u_i . dc_i/dtheta_i); p' = J theta', J = J_x^-1 J_q
//!           J_x p'' = J_q theta'' + w, w_i = theta_i'^2 (u_i . c_i'') - |p' - c_i' theta_i'|^2
//! ```
//!
//! Dynamics as on the D3 (Codourey 1998, Olsson 2009): each forearm pair's mass split, a share r at the elbow and
//! 1 - r at the plate; two arms about their joints plus a point mass at the plate, m_nt = plate + payload +
//! 2 (1 - r) m_fb, and
//!
//! ```text
//! M(theta) theta'' = tau_gear + Q_g - m_nt J^T J_x^-1 w + tau_joint,   M = I_b 1 + m_nt J^T J
//! I_b = I_arm + L_a^2 (m_elbow + r m_fb),   Q_g,i = g m_LG cos theta_i - m_nt g J_zi,   m_LG = m_arm l_cg + L_a (m_elbow + r m_fb)
//! ```
//!
//! Each joint is a `RotaryJoint`, elastic or rigid (effects.joint_elasticity), as on the D3.
//!
//! The machine is the planar linkage of machine files (`sim_netlist::linkage::LinkageMachine`), read from this crate's
//! tables (`[geometry]`, `[mass]`, `[workspace]`, one `[joint]` for both arms): one implementation, so that a fix
//! reaches both, and machines/planar-delta.toml gives this crate's numbers.

use sim_core::{Config, ConfigError, Machine, MachineEntry};
pub use sim_netlist::linkage::{Legs, SIDE, condition_number};

/// The planar delta: the linkage machine of `sim_netlist`, built by `Delta2::new` from this crate's config.
pub type Delta2 = sim_netlist::linkage::LinkageMachine;

pub static ENTRY: MachineEntry = MachineEntry {
    id: "delta-d2",
    key: "machine-delta-d2",
    default_config: include_str!("../default.toml"),
    i18n: &[("en", include_str!("../i18n/en.ftl")), ("de", include_str!("../i18n/de.ftl"))],
    build,
    suite_configs,
};

/// Effects that take energy out or put it in; the standard suite runs with them off. The stops stay, without their
/// damping.
pub const LOSSY_EFFECTS: [&str; 7] = [
    "joint_friction",
    "motor_friction",
    "gearbox_drag",
    "gearbox_lost_motion",
    "gearbox_efficiency",
    "torque_ripple",
    "holding_brake",
];

fn build(cfg: &Config) -> Result<Box<dyn Machine>, ConfigError> {
    Ok(Box::new(Delta2::new(cfg)?))
}

/// The default config with every loss switched off.
pub fn lossless_config() -> Config {
    let mut cfg = ENTRY.config();
    for effect in LOSSY_EFFECTS {
        cfg.set_text(&format!("effects.{effect} = false")).unwrap();
    }
    cfg.set_text("gearbox.damping_nm_s_rad = 0.0").unwrap();
    cfg.set_text("joint.stop_damping_nm_s_rad = 0.0").unwrap();
    cfg
}

/// The start pose and the cycle's targets moved by `dz` in z (see `suite_configs`).
fn lowered(cfg: &mut Config, dz: f64) {
    let start = cfg.f64s("start.pose").unwrap();
    cfg.set_text(&format!("start.pose = [{:?}, {:?}]", start[0], start[1] + dz)).unwrap();
    let segments: Vec<String> = cfg
        .array("cycle.segment")
        .unwrap()
        .iter()
        .map(|s| {
            let t: Vec<f64> = s["target"].as_array().unwrap().iter().map(|v| v.as_float().unwrap()).collect();
            format!(
                "{{ target = [{:?}, {:?}], dwell_s = {:?}, payload = {} }}",
                t[0],
                t[1] + dz,
                s["dwell_s"].as_float().unwrap(),
                s["payload"].as_bool().unwrap()
            )
        })
        .collect();
    cfg.set_text(&format!("cycle.segment = [{}]", segments.join(", "))).unwrap();
}

/// Loss-free rigid and elastic models, with and without the payload, and the elastic one with part heating and the
/// temperature feedback (no loss, so the gearboxes stay at ambient).
///
/// As on the D3, their working range, start pose and cycle lie 150 mm lower than the default's (top 550 mm below the
/// shoulders): held by constant motor torques, as the suite holds it after a pulse, the plate's vertical mode is
/// unstable at the default's middle pose, 638.5 mm down, and stable at 788.5 mm
/// (`the_vertical_mode_under_constant_torques_is_unstable_high_up`). The drives' position loops hold it.
fn suite_configs() -> Vec<Config> {
    let mut out = Vec::new();
    for (elastic, payload) in [(false, true), (false, false), (true, true), (true, false)] {
        let mut cfg = lossless_config();
        cfg.set_text("workspace.top_z_m = -0.55").unwrap();
        lowered(&mut cfg, -0.15);
        cfg.set_text(&format!("effects.joint_elasticity = {elastic}")).unwrap();
        cfg.set_text(&format!("effects.payload = {payload}")).unwrap();
        cfg.set_text(&format!("effects.part_heating = {}", elastic && payload)).unwrap();
        cfg.set_text(&format!("effects.thermal_feedback = {}", elastic && payload)).unwrap();
        out.push(cfg);
    }
    out
}

#[cfg(test)]
mod tests;
