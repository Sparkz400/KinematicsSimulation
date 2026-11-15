//! Property tests of the URDF reader (`Chain::from_urdf`) that machine-joint-kit builds its arms with: whatever the text,
//! a chain or a `ConfigError`, never a panic or a hang; a chain read holds finite numbers and unit joint axes. Seeds:
//! crates/machine-joint-kit/urdf.

#[path = "../../sim-core/tests/edits/mod.rs"]
mod edits;

use proptest::prelude::*;
use sim_components::rigid::Chain;

fn check(text: &str) -> Result<(), TestCaseError> {
    if let Ok(chain) = Chain::from_urdf(text) {
        prop_assert!(!chain.bodies.is_empty());
        for b in &chain.bodies {
            let i = &b.inertial;
            let numbers = b.origin_rot.iter().flatten().chain(&b.origin_pos).chain(&b.axis).chain(&i.com);
            prop_assert!(numbers.chain(i.inertia.iter().flatten()).all(|v| v.is_finite()), "{b:?}");
            prop_assert!(i.mass >= 0.0 && i.mass.is_finite(), "{b:?}");
            prop_assert!((b.axis.iter().map(|a| a * a).sum::<f64>() - 1.0).abs() < 1e-9, "{b:?}");
        }
    }
    Ok(())
}

proptest! {
    #[test]
    fn any_text_is_read_or_refused(text in "[<>/=\"' a-z0-9.-]{0,400}") {
        check(&text)?;
    }

    #[test]
    fn edited_urdf_files_are_read_or_refused((_, text) in edits::edited(edits::files("crates/machine-joint-kit/urdf", ".urdf"))) {
        check(&text)?;
    }
}

fn link(name: &str, com: &str) -> String {
    format!(r#"<link name="{name}"><inertial><origin xyz="{com}"/><mass value="1"/></inertial></link>"#)
}

fn joint(kind: &str, parent: &str, child: &str, axis: &str) -> String {
    format!(
        r#"<joint name="{parent}{child}" type="{kind}"><parent link="{parent}"/><child link="{child}"/><axis xyz="{axis}"/></joint>"#
    )
}

/// Joints that lead back to a link already passed (a to b, b to a) made the reader walk the loop for ever, its chain
/// growing until memory ran out.
#[test]
fn a_loop_of_joints_is_refused() {
    let parts = [
        link("base", "0 0 0"),
        link("a", "0 0 0"),
        link("b", "0 0 0"),
        joint("revolute", "base", "a", "0 0 1"),
        joint("revolute", "a", "b", "0 0 1"),
        joint("revolute", "b", "a", "0 0 1"),
    ];
    assert!(Chain::from_urdf(&format!("<robot>{}</robot>", parts.concat())).is_err());
}

/// An axis of 1e200 squared to infinity and became (0, 0, 0); one of 1e-170 squared to 0 and was refused. Both are
/// the unit axis.
#[test]
fn huge_and_tiny_axes_are_unit_axes() {
    for axis in ["0 1e200 0", "0 1e-170 0"] {
        let parts = [link("base", "0 0 0"), link("a", "0 0 0"), joint("revolute", "base", "a", axis)];
        let chain = Chain::from_urdf(&format!("<robot>{}</robot>", parts.concat())).unwrap();
        assert_eq!(chain.bodies[0].axis, [0.0, 1.0, 0.0], "{axis}");
    }
}

/// A tool 1e200 m out, merged into the last body by a fixed joint: its inertia about that body is infinite.
#[test]
fn a_merged_link_whose_inertia_overflows_is_refused() {
    let parts = [
        link("base", "0 0 0"),
        link("a", "0 0 0"),
        link("tool", "1e200 0 0"),
        joint("revolute", "base", "a", "0 0 1"),
        joint("fixed", "a", "tool", "1 0 0"),
    ];
    assert!(Chain::from_urdf(&format!("<robot>{}</robot>", parts.concat())).is_err());
}
