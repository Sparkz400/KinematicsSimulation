//! Path files and programs (G-code, KRL, RAPID, robot CSV): the input read by every reader, on two, three and six axes.
#![no_main]

use libfuzzer_sys::fuzz_target;
use sim_components::path::Piece;
use sim_components::program::read;
use sim_core::AxisInfo;

fn axis(id: &'static str) -> AxisInfo {
    AxisInfo { id, key: "", unit: "m", min: -1.0, max: 1.0, max_vel: 1.0, max_acc: 10.0, max_jerk: 1000.0 }
}

fuzz_target!(|text: &str| {
    let machines = [
        ["x", "y"].map(axis).to_vec(),
        ["x", "y", "z"].map(axis).to_vec(),
        ["x", "y", "z", "rx", "ry", "rz"].map(axis).to_vec(),
    ];
    for name in ["p.path", "p.csv", "p.nc", "p.src", "p.mod"] {
        for axes in &machines {
            let Ok(runs) = read(name, text, axes, &vec![0.0; axes.len()]) else { continue };
            for run in runs {
                assert!(run.feed.is_none_or(|f| f.is_finite() && f > 0.0));
                for piece in run.pieces {
                    let finite = match piece {
                        Piece::Line(v) | Piece::Spline(v) => v.iter().all(|x| x.is_finite()),
                        Piece::Arc { to, centre, .. } => to.iter().chain(&centre).all(|x| x.is_finite()),
                    };
                    assert!(finite);
                }
            }
        }
    }
});
