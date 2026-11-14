//! The 3D scene beyond the tool's place: deflections drawn larger, the cable chain, the force arrows and the
//! posts, each against a hand calculation.

use super::*;

/// The default config with frame compliance and tool flex on (as it has them), at rest at `pose`, with the frame
/// swayed by `q` and the Z axis's centre of mass moved by `u`.
fn deflected(pose: [f64; 2], q: f64, u: f64) -> (TGantry, Vec<f64>) {
    let m = TGantry::new(&ENTRY.config()).unwrap();
    let mut state = Vec::new();
    m.equilibrium(&pose, &mut state, &mut [0.0; 2]);
    (state[Q], state[U]) = (q, u);
    (m, state)
}

fn drawn(m: &TGantry, state: &[f64], exaggeration: f64) -> Scene {
    let mut scene = Scene { exaggeration, ..Scene::default() };
    m.scene(state, &mut scene);
    scene
}

fn part<'a>(scene: &'a Scene, id: &str, nth: usize) -> &'a Part {
    scene.parts.iter().filter(|p| p.id == id).nth(nth).unwrap()
}

/// By hand at (0.3, 0.1) with q = 20 um and u = 10 um: the tool sits tip u + q to the right of where the
/// carriage puts it (`tool_state`), tip = flex_at(z).2. Drawn as it is, the flange is exactly there; drawn 100
/// times larger, 100 times as far. The Z axis turns about the carriage, so its top moves the other way, and the
/// posts lean from the floor: their middle moves by half the sway.
#[test]
fn deflections_drawn_larger_move_the_tool_by_the_factor() {
    let (q, u) = (2.0e-5, 1.0e-5);
    let (m, state) = deflected([0.3, 0.1], q, u);
    let tip = m.flex_at(0.1).2;
    assert!(tip > 0.5, "tool flex must be on in the default config: {tip}");
    let at = m.scene_point(&[0.3, 0.1]).unwrap();
    let (mut tool, mut speed) = ([0.0; 2], [0.0; 2]);
    m.tool_state(&state, &mut tool, &mut speed);
    for k in [1.0, 100.0] {
        let scene = drawn(&m, &state, k);
        let flange = part(&scene, "flange", 0).pos;
        let expected = at[0] + k * (tip * u + q);
        assert!((flange[0] - expected).abs() < 1e-12, "x{k}: flange at {}, by hand {expected}", flange[0]);
        if k == 1.0 {
            let mapped = m.scene_point(&tool).unwrap();
            assert!(
                (flange[0] - mapped[0]).abs() < 1e-12,
                "as it is, the flange is where the tool is: {} {}",
                flange[0],
                mapped[0]
            );
        }
        // The column's top is above the carriage: it moves against the tool.
        let clamp = part(&scene, "clamp", 0).pos[0];
        let carriage = part(&scene, "carriage", 0).pos[0];
        assert!(clamp < carriage, "x{k}: the top leans the other way: clamp {clamp}, carriage {carriage}");
        assert!((carriage - (0.15 + 0.3 + k * q)).abs() < 1e-12, "x{k}: carriage at {carriage}");
        let post = part(&scene, "post", 0);
        assert!((post.pos[0] - (0.15 + k * q / 2.0)).abs() < 1e-12, "x{k}: post at {}", post.pos[0]);
        let Shape::Box { size } = &post.shape else { panic!("a post is a box") };
        assert!((post.angle + (k * q / size[2]).atan()).abs() < 1e-15, "x{k}: post leans by {}", post.angle);
    }
}

/// The chain's length stays: as many links at every pose, its bracket 0.1 m left of the carriage, and its loop
/// (the leftmost link, half a link inside the loop's radius of 0.09 m) 0.02 m + x / 2 left of the bracket, so at
/// span + x - 0.1 - 0.02 - x / 2 - 0.09. Also at the shortest span the EXCT-30's bounds allow, 0.10 m.
#[test]
fn the_cable_chain_keeps_its_length_and_its_loop_moves_half_as_far() {
    for span in [0.15, 0.10] {
        let mut cfg = ENTRY.config();
        cfg.set_text(&format!("geometry.span_min_m = {span}")).unwrap();
        let m = TGantry::new(&cfg).unwrap();
        let mut links = Vec::new();
        for x in [0.0, 0.05, 0.4, 1.0] {
            let (mut state, mut scene) = (Vec::new(), Scene::default());
            m.equilibrium(&[x, 0.2], &mut state, &mut [0.0; 2]);
            m.scene(&state, &mut scene);
            let chain: Vec<&Part> = scene.parts.iter().filter(|p| p.id == "cable-chain").collect();
            links.push(chain.len());
            let bracket = part(&scene, "chain-bracket", 0).pos[0];
            let carriage = part(&scene, "carriage", 0).pos[0];
            assert!((bracket - (carriage - 0.1)).abs() < 1e-12, "at {x}: bracket {bracket}, carriage {carriage}");
            // The last link ends at the bracket: its middle at most a link short of it (just inside the loop where the
            // upper run is shorter than a link).
            let last = chain.last().unwrap();
            assert!(
                (bracket - last.pos[0]).abs() < 0.035 && last.pos[0] < bracket,
                "at {x}: last link at {}, bracket {bracket}",
                last.pos[0]
            );
            let loop_left = chain.iter().map(|p| p.pos[0]).fold(f64::MAX, f64::min);
            let by_hand = span + x - 0.1 - 0.02 - x / 2.0 - 0.09;
            assert!((loop_left - by_hand).abs() < 0.002, "span {span} at {x}: loop at {loop_left}, by hand {by_hand}");
        }
        assert!(links.iter().all(|&n| n == links[0]) && links[0] > 40, "span {span}: links {links:?}");
    }
}

/// The rigid model keeps every section at pretension: each motor pulley's shaft carries two of them towards the
/// carriage (600 N at 300 N pretension), and carriage and Z axis get none. The posts stand under the ends of the
/// travel, 0.15 m in from each pulley.
#[test]
fn the_arrows_show_the_belt_pull_and_the_posts_stand_under_the_travel_ends() {
    let m = TGantry::new(&{
        let mut cfg = ENTRY.config();
        cfg.set_text("effects.belt_stretch = false").unwrap();
        cfg
    })
    .unwrap();
    let (mut state, mut scene) = (Vec::new(), Scene::default());
    m.equilibrium(&[0.5, 0.2], &mut state, &mut [0.0; 2]);
    m.scene(&state, &mut scene);
    let pull = 2.0 * m.belt.pretension;
    assert!((pull - 600.0).abs() < 1e-9, "pretension {}", m.belt.pretension);
    let forces: Vec<[f64; 3]> = scene.arrows.iter().map(|a| a.force).collect();
    assert_eq!(forces, vec![[0.0; 3], [0.0; 3], [pull, 0.0, 0.0], [-pull, 0.0, 0.0]]);
    let end = 2.0 * 0.15 + 1.0;
    let posts: Vec<f64> = scene.parts.iter().filter(|p| p.id == "post").map(|p| p.pos[0]).collect();
    assert!((posts[0] - 0.15).abs() < 1e-12 && (posts[1] - (end - 0.15)).abs() < 1e-12, "posts at {posts:?}");
}

/// The mode of `resonance_hz` (see `mode_shape`), by hand from the default config at x = 0.9 m: mc = 4 kg, mz = 5 + 2
/// = 7 kg, kb = EA / L1 + EA / L3 with L1 = 0.3 + x + z, L3 = 0.3 + 1 - x + z, EA = 5e5 N; kz = 1 / (1 / 2e7 + h^2 /
/// 1.2e4) and tip = kz (1 / 2e7 + h (0.7 - z) / 1.2e4) with h = 0.3 - z. In (x, u): M = [[mc + mz, mz], [mz, mz]],
/// K = [[kb, 0], [0, kz]]. At z = 0.05 m: w^2 = 2.394e4, tip = 2.585, v = (0.050, 0.368), the tool moving 1 m. At
/// z = 0.362 m (x = 0.9) tip has turned negative and the tool hardly moves in the mode: the shape stays finite there,
/// the carriage's travel being the larger.
#[test]
fn the_mode_shape_is_the_eigenvector_of_the_carriage_and_the_leaning_z_axis() {
    let m = TGantry::new(&ENTRY.config()).unwrap();
    let x = 0.9;
    for z in [0.05, 0.15, 0.362] {
        let (hz, shape) = m.mode_shape(&[x, z]).unwrap();
        assert_eq!(hz, m.resonance_hz(&[x, z]).unwrap());
        assert_eq!(shape.len(), m.state_len());
        assert!(shape.iter().enumerate().all(|(i, &v)| i == X || i == U || v == 0.0), "only x and u move: {shape:?}");
        let (mc, mz) = (4.0, 7.0);
        let kb = 5e5 / (0.3 + x + z) + 5e5 / (0.3 + 1.0 - x + z);
        let h = 0.3 - z;
        let kz = 1.0 / (1.0 / 2e7 + h * h / 1.2e4);
        let tip = kz * (1.0 / 2e7 + h * (0.7 - z) / 1.2e4);
        let w2 = (std::f64::consts::TAU * hz).powi(2);
        let (vx, vu) = (shape[X], shape[U]);
        let residual = [kb * vx - w2 * ((mc + mz) * vx + mz * vu), kz * vu - w2 * (mz * vx + mz * vu)];
        let scale = kb * vx.abs() + kz * vu.abs();
        assert!(residual.iter().all(|r| r.abs() < 1e-9 * scale), "z = {z}: (K - w^2 M) v = {residual:?}");
        let largest = vx.abs().max((vx + tip * vu).abs());
        assert!((largest - 1.0).abs() < 1e-12 && vu.is_finite(), "z = {z}: the larger travel is {largest}, lean {vu}");
        assert!(vx * vu > 0.0, "z = {z}: the lower mode, both the same way: {vx} {vu}");
        if z < 0.2 {
            assert!(vu > vx, "z = {z}: low on the Z axis mostly lean: {vx} {vu}");
        }
        if z == 0.05 {
            assert!(
                (w2 - 2.394e4).abs() < 10.0
                    && (tip - 2.585).abs() < 1e-3
                    && (vx - 0.050).abs() < 1e-3
                    && (vu - 0.368).abs() < 1e-3,
                "{w2} {tip} {vx} {vu}"
            );
        }
    }
    // No resonance, no mode.
    assert!(
        TGantry::new(&{
            let mut cfg = ENTRY.config();
            cfg.set_text("effects.tool_flex = false").unwrap();
            cfg
        })
        .unwrap()
        .mode_shape(&[x, 0.05])
        .is_none()
    );
}

/// Equilibrium plus 1e-3 times the shape moves the tool by 1e-3 m (`tool_state`) and, drawn as it is, the flange
/// too. Drawn k times larger, only the lean is: the carriage's x is where it is, so the flange moves by
/// 1e-3 (vx + k tip vu), not 1e-3 k.
#[test]
fn the_mode_shape_drawn_moves_the_flange_by_its_amplitude() {
    let m = TGantry::new(&ENTRY.config()).unwrap();
    let pose = [0.9, 0.05];
    let (_, shape) = m.mode_shape(&pose).unwrap();
    let (mut rest, mut torque) = (Vec::new(), [0.0; 2]);
    m.equilibrium(&pose, &mut rest, &mut torque);
    let moved: Vec<f64> = rest.iter().zip(&shape).map(|(s, v)| s + 1e-3 * v).collect();
    let (mut before, mut after, mut speed) = ([0.0; 2], [0.0; 2], [0.0; 2]);
    m.tool_state(&rest, &mut before, &mut speed);
    m.tool_state(&moved, &mut after, &mut speed);
    assert!((after[0] - before[0] - 1e-3).abs() < 1e-12, "the tool moves {}", after[0] - before[0]);
    let tip = m.flex_at(pose[1]).2;
    for k in [1.0, 100.0] {
        let shift = part(&drawn(&m, &moved, k), "flange", 0).pos[0] - part(&drawn(&m, &rest, k), "flange", 0).pos[0];
        let by_hand = 1e-3 * (shape[X] + k * tip * shape[U]);
        assert!((shift - by_hand).abs() < 1e-12, "x{k}: flange moves {shift}, by hand {by_hand}");
    }
}

/// The drawn belt's points, in order, and its teeth.
fn belt_drawn(scene: &Scene) -> (Vec<[f64; 3]>, usize) {
    let strips: Vec<&Part> = scene.parts.iter().filter(|p| p.id == "belt").collect();
    let mut points: Vec<[f64; 3]> = strips.iter().map(|p| p.pos).collect();
    if let Some(Shape::Segment { to, .. }) = strips.last().map(|p| &p.shape) {
        points.push(*to);
    }
    (points, scene.parts.iter().filter(|p| p.id == "belt-tooth").count())
}

fn belt_length(points: &[[f64; 3]]) -> f64 {
    points.windows(2).map(|p| (p[1][0] - p[0][0]).hypot(p[1][2] - p[0][2])).sum()
}

/// The sag of the upper left span (from the left motor pulley's top to the carriage idler's bottom, both at
/// z = r = 0.02 m), at x = 0.4: L = 0.15 + 0.4 - 0.03 = 0.52 m, w = 0.1 kg/m * 9.80665 = 0.981 N/m, and T that
/// section's tension (about the 300 N pretension): s = w L^2 / (8 T) = 1.1e-4 m, the parabola's depth at its middle.
/// The span is drawn leaving the pulley level, as off its wrap, so it is that deep a third of the way from the
/// idler (a drawn point: the second of six). Drawn 100 times larger, 100 times as deep. With pulley A turned
/// 0.1 rad on, 2 mm more belt feeds section 1 than its 0.36 mm stretch: slack, drawn 15 mm deep.
#[test]
fn a_belt_span_sags_by_its_catenary() {
    let (m, rest) = deflected([0.4, 0.2], 0.0, 0.0);
    let mut slack = rest.clone();
    slack[A] += 0.1;
    assert_eq!(m.sections(&slack)[0].tension, 0.0);
    let (span, r) = (0.52, 0.02);
    // The deepest drawn point of the upper left span.
    let middle = |scene: &Scene| {
        let (points, _) = belt_drawn(scene);
        points
            .iter()
            .filter(|p| p[0] > 1e-9 && p[0] < span - 1e-9 && p[2] > 0.0)
            .map(|p| r - p[2])
            .fold(f64::MIN, f64::max)
    };
    let tension = m.sections(&rest)[0].tension;
    assert!((tension - 300.0).abs() < 60.0, "{tension} N");
    let by_hand = 0.1 * 9.80665 * span * span / (8.0 * tension);
    assert!((by_hand - 1.1e-4).abs() < 0.3e-4, "{by_hand} m");
    for k in [1.0, 100.0] {
        let sag = middle(&drawn(&m, &rest, k));
        assert!((sag - k * by_hand).abs() < 1e-12, "x{k}: drawn {sag}, by hand {}", k * by_hand);
    }
    let sag = middle(&drawn(&m, &slack, 1.0));
    assert!((sag - 0.015).abs() < 1e-12, "slack: {sag}");
}

/// The teeth keep their places on the belt: as many at every pose, taut or slack, drawn as it is or larger. The
/// slack span lengthens the drawn belt by less than its smooth sag does and by more than 0.9 of it. That sag,
/// s 27/4 t (1 - t)^2 over the span, is longer than the chord by (s^2 / L) (1/2) (27/4)^2 integral of
/// (1 - 4t + 3t^2)^2 = (s^2 / L) (1/2) (729/16) (2/15) = 3.0375 s^2 / L.
#[test]
fn belt_teeth_stay_put_and_the_sag_lengthens_the_belt_as_its_shape_says() {
    let mut counts = Vec::new();
    for pose in [[0.0, 0.0], [0.4, 0.2], [1.0, 0.5]] {
        let (m, rest) = deflected(pose, 0.0, 0.0);
        let mut slack = rest.clone();
        slack[A] += 0.1;
        for state in [&rest, &slack] {
            for k in [1.0, 100.0] {
                counts.push(belt_drawn(&drawn(&m, state, k)).1);
            }
        }
    }
    assert!(counts.iter().all(|&n| n == counts[0]) && counts[0] > 100, "teeth: {counts:?}");

    let (m, rest) = deflected([0.4, 0.2], 0.0, 0.0);
    let mut slack = rest.clone();
    slack[A] += 0.1;
    let (span, s0, s1) = (0.52, span_sag(0.1 * 9.80665, 0.52, m.sections(&rest)[0].tension), 0.015);
    let longer =
        belt_length(&belt_drawn(&drawn(&m, &slack, 1.0)).0) - belt_length(&belt_drawn(&drawn(&m, &rest, 1.0)).0);
    let smooth = 3.0375 * (s1 * s1 - s0 * s0) / span;
    assert!(longer < smooth && longer > 0.9 * smooth, "drawn {longer} m longer, the smooth sag {smooth} m");
}
