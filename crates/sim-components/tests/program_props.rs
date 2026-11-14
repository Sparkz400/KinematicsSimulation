//! Property tests of the path and program readers (`path::parse`, `path::parse_trajectory`, `program::read`: G-code,
//! KRL, RAPID, robot CSV) and of the cam tables (`cam::parse_table`, `Cam::from_table`): whatever the text, an `Ok` or
//! an error value, never a panic; and an `Ok` holds finite numbers only. Seeds: fuzz/seeds.

#[path = "../../sim-core/tests/edits/mod.rs"]
mod edits;

use proptest::prelude::*;
use sim_components::cam::{Cam, parse_table};
use sim_components::path::{Piece, parse_trajectory};
use sim_components::program::read;
use sim_core::AxisInfo;

fn axis(id: &'static str) -> AxisInfo {
    AxisInfo { id, key: "", unit: "m", min: -1.0, max: 1.0, max_vel: 1.0, max_acc: 10.0, max_jerk: 1000.0 }
}

/// A gantry, a three-axis portal and an arm in tool space.
fn machines() -> [Vec<AxisInfo>; 3] {
    [
        ["x", "y"].map(axis).to_vec(),
        ["x", "y", "z"].map(axis).to_vec(),
        ["x", "y", "z", "rx", "ry", "rz"].map(axis).to_vec(),
    ]
}

fn finite(pieces: &[Piece]) -> bool {
    pieces.iter().all(|p| match p {
        Piece::Line(v) | Piece::Spline(v) => v.iter().all(|x| x.is_finite()),
        Piece::Arc { to, centre, .. } => to.iter().chain(centre).all(|x| x.is_finite()),
    })
}

fn check_program(name: &str, text: &str) -> Result<(), TestCaseError> {
    for axes in machines() {
        let start = vec![0.0; axes.len()];
        if let Ok(runs) = read(name, text, &axes, &start) {
            for run in &runs {
                prop_assert!(finite(&run.pieces), "{name}: {run:?}");
                prop_assert!(run.feed.is_none_or(|f| f.is_finite() && f > 0.0), "{name}: feed {:?}", run.feed);
            }
        }
    }
    Ok(())
}

const NAMES: [&str; 7] = ["p.path", "p.csv", "p.nc", "p.src", "p.mod", "p.prg", "p.txt"];

proptest! {
    #[test]
    fn any_text_is_read_or_refused_by_every_reader(text in ".{0,300}", name in prop::sample::select(NAMES.to_vec())) {
        check_program(name, &text)?;
    }

    #[test]
    fn edited_programs_are_read_or_refused((name, text) in edits::edited(edits::files("fuzz/seeds/program", ""))) {
        check_program(&name, &text)?;
    }

    #[test]
    fn edited_trajectories_are_read_or_refused((_, text) in edits::edited(edits::files("fuzz/seeds/trajectory", ""))) {
        if let Ok(points) = parse_trajectory(&text) {
            prop_assert!(points.iter().all(|(t, p)| t.is_finite() && p.iter().all(|x| x.is_finite())), "{points:?}");
        }
    }

    #[test]
    fn edited_cam_tables_are_read_or_refused((_, text) in edits::edited(edits::files("fuzz/seeds/cam", ""))) {
        if let Ok(rows) = parse_table(&text) {
            prop_assert!(rows.iter().all(|r| r.0.is_finite() && r.1.is_finite() && r.2.is_none_or(f64::is_finite)), "{rows:?}");
            if let Ok(cam) = Cam::from_table(&rows) {
                let _ = cam.peaks(1.0);
                for k in 0..=8 {
                    let s = cam.at(k as f64 * 0.25 * (rows[rows.len() - 1].0 - rows[0].0));
                    prop_assert!(s.iter().all(|v| v.is_finite()), "{rows:?}: {s:?}");
                }
            }
        }
    }
}

/// What the properties found, kept as cases of their own.
mod found {
    use super::*;

    /// A G-code line starting with a letter of more than one byte panicked on slicing inside it.
    #[test]
    fn gcode_with_a_non_ascii_letter_is_refused() {
        let axes = &machines()[0];
        assert_eq!(read("p.nc", "¡", axes, &[0.0; 2]).unwrap_err().key, "program-error-word");
        assert_eq!(read("p.nc", "G1 X1 F60 é", axes, &[0.0; 2]).unwrap_err().key, "program-error-word");
    }

    /// `1e999` reads as infinity, which a trajectory, a cam table and a KRL position took as a number.
    #[test]
    fn infinite_numbers_are_refused() {
        assert!(parse_trajectory("time,x\n1e999,0.0\n0.001,0.1").is_err());
        assert!(parse_trajectory("0.0,NaN").is_err());
        assert_eq!(parse_table("0,0\n0.5,inf\n1,0"), Err(2));
        let axes = &machines()[0];
        let krl = "DEF demo()\n  $VEL.CP = 0.2\n  LIN {X 1e999}\nEND";
        assert_eq!(read("p.src", krl, axes, &[0.0; 2]).unwrap_err().line, 3);
    }

    /// A slave of 1e308 next to one of 1 overflows the spline's slopes: the cam's rates were infinite.
    #[test]
    fn a_cam_table_whose_slopes_overflow_is_refused() {
        let rows = parse_table("0,1e308,0\n0.5,1,2\n1,2,0").unwrap();
        assert!(Cam::from_table(&rows).is_err());
        assert!(Cam::from_table(&[(0.0, 0.0, None), (1e-300, 1e300, None), (1.0, 0.0, None)]).is_err());
    }
}
