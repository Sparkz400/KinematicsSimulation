//! The app window: a 3D view built from the machine's scene description, next to the panels.
//! It knows no machine; it draws whatever parts the selected machine describes.

mod ui;
mod views;

use bevy::anti_alias::smaa::Smaa;
use bevy::asset::RenderAssetUsages;
use bevy::asset::io::file::FileAssetReader;
use bevy::camera::visibility::RenderLayers;
use bevy::camera::{CameraOutputMode, Hdr, RenderTarget, Viewport};
use bevy::gltf::GltfAssetLabel;
use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll};
use bevy::light::{CascadeShadowConfigBuilder, NotShadowCaster};
use bevy::pbr::ScreenSpaceAmbientOcclusion;
use bevy::picking::mesh_picking::ray_cast::{MeshRayCast, MeshRayCastSettings};
use bevy::post_process::bloom::Bloom;
use bevy::prelude::*;
use bevy::render::render_resource::{
    BlendState, Extent3d, TextureDimension, TextureFormat, TextureViewDescriptor, TextureViewDimension,
};
use bevy::render::view::Tonemapping;
use bevy::render::view::screenshot::{Screenshot, save_to_disk};
use bevy::window::PrimaryWindow;
use bevy::world_serialization::WorldAssetRoot;
use bevy_egui::{
    EguiContexts, EguiGlobalSettings, EguiPlugin, EguiPrimaryContextPass, EguiTextureHandle, EguiUserTextures,
    PrimaryEguiContext, egui,
};
use sim_app::model::{Analysis, Message, Model};
use sim_app::runs::{History, Run, RunKind, trail_until};
use sim_app::session::{Event, Request, Session, TrailPoint};
use sim_app::video;
use sim_components::Command;
use sim_core::{Config, Part, Scene, Shape};
use sim_machines::{LANGUAGES, MACHINES};
use std::collections::VecDeque;
use ui::Action;

/// Orbit camera: angles in radians around a target point.
struct Orbit {
    yaw: f32,
    pitch: f32,
    distance: f32,
    target: Vec3,
}

#[derive(Resource)]
struct State {
    model: Model,
    orbit: Orbit,
    /// The entities drawing the scene's parts, in the scene's order, with the part ids they were made for.
    parts: Vec<(Entity, Handle<StandardMaterial>)>,
    part_ids: Vec<&'static str>,
    /// The floor under the machine, placed when the machine's first scene arrives: its entity, the middle of its
    /// top and how wide it is.
    floor: Option<(Entity, Vec3, f32)>,
    /// The box the machine's parts filled when its first scene arrived, which the camera frames.
    bounds: Option<(Vec3, Vec3)>,
    /// The camera is to frame the machine anew: a new machine, or a view preset was picked.
    fit: bool,
    overlays: ui::Overlays,
    /// The tabs shown and the check picked.
    panels: ui::PanelState,
    /// The panels' look is set: once, on the first frame.
    themed: bool,
    /// The config file the drawn parts were made for, which decides which CAD meshes they use (`cad_mesh`).
    mesh_config: String,
    /// The 3D view's rectangle in logical pixels: (left, top, right, bottom).
    view_rect: [f32; 4],
    frame: u32,
    /// Where the screenshot hook scrolls the right panel to, in pixels from its top.
    scroll: Option<f32>,
    /// The scenes as they arrived, each with the simulated time of the newest sample then, over the scopes' window
    /// and at most one per `RECORDED`th of it: what the timeline scrubs and replays.
    recorded: VecDeque<(f64, Scene)>,
    timeline: ui::Timeline,
    /// Whether the simulation was paused when the timeline left the newest scene, to be so again when it returns.
    was_paused: bool,
    /// The travelled path's points with their speed and plan (`session::ViewNews`), as the model's `trail` has them.
    trail_points: Vec<TrailPoint>,
    /// The machine tells a mode to show, and whether the session was last asked to show it.
    has_mode: bool,
    mode_sent: bool,
    /// The effect (its config path) the compare switch picks, and the run with it switched while compared, drawn
    /// by `ghost_parts`.
    compare_effect: String,
    ghost: Option<Ghost>,
    ghost_parts: Vec<(Entity, bool)>,
    /// The view's recording, while the Record button is on, and the videos still being made from stopped ones.
    recording: Option<video::Recording>,
    encodings: Vec<video::Encoding>,
    /// What the plot pictures saved say, from the screenshot's observer, for the messages.
    notes: std::sync::Arc<std::sync::Mutex<Vec<Message>>>,
}

/// A second run beside the machine's with one effect switched the other way, drawn as a see-through ghost.
struct Ghost {
    session: Session,
    /// The effect's config path, and whether it is on in this run.
    effect: String,
    on: bool,
    /// The machine and config the run was made from: with another, the comparison says nothing and ends.
    entry: &'static str,
    base: Config,
    scene: Scene,
    /// Paused, speed and exaggeration as last sent, to follow the machine's run.
    sent: Option<(bool, f64, f64)>,
    /// Its samples, for keeping its window as a run (`Action::KeepGhost`), and its scenes at the machine's time when
    /// they arrived, as `State::recorded` has the machine's, so that the timeline shows both at the same time.
    history: History,
    recorded: VecDeque<(f64, Scene)>,
}

/// The ghost's colour: a light neutral grey, see-through, none of the palette's meaning colours.
const GHOST: Color = Color::srgba(0.86, 0.88, 0.92, 0.25);

/// The gap between the tool and the plan, as the plan overlay and the deviation tube draw it.
const GAP: Color = Color::srgb(0.9, 0.3, 0.9);

/// Where the tool view's camera stands from the tool: in front, a little above, a quarter of a metre away.
const TOOL_VIEW_FROM: Vec3 = Vec3::new(0.0, 0.07, 0.24);
/// The tool view's size in the window, in logical pixels; its picture is rendered at twice that, to stay sharp on
/// screens that scale.
const TOOL_VIEW_SIZE: egui::Vec2 = egui::vec2(260.0, 180.0);

/// The picture the tool view's camera renders into, and its id in the panels.
#[derive(Resource)]
struct ToolView(egui::TextureId);

#[derive(Component)]
struct ToolCamera;

/// A part of the compared run's ghost (`Ghost`).
#[derive(Component)]
struct GhostPart;

/// At most this many scenes are recorded, spread over the scopes' window: a scene of a few hundred parts is some
/// 50 kB, so this holds the recording to about 50 MB however long the window.
const RECORDED: usize = 1000;

#[derive(Resource)]
struct Shapes {
    cube: Handle<Mesh>,
    cylinder: Handle<Mesh>,
}

#[derive(Component)]
struct WorldCamera;

fn main() {
    let lang = std::env::var("SIM_APP_LANG")
        .ok()
        .and_then(|l| LANGUAGES.iter().copied().find(|k| *k == l))
        .unwrap_or(LANGUAGES[0]);
    let mut model = Model::new(MACHINES[0], lang);
    let args: Vec<String> = std::env::args().collect();
    // `sim-app --catalog DIR`: the catalogue from a directory instead of the compiled-in one. Before a machine file is
    // opened, which may name parts.
    if let Some(dir) = args.windows(2).find(|w| w[0] == "--catalog").map(|w| &w[1])
        && let Err(e) = sim_components::catalog::use_directory(std::path::Path::new(dir))
    {
        eprintln!("{}", model.tr.tr_arg(e.key(), e.detail()));
        std::process::exit(2);
    }
    // `sim-app --machine-file FILE` starts with a machine defined in a file (docs/machine-files.md).
    if let Some(file) = args.windows(2).find(|w| w[0] == "--machine-file").map(|w| w[1].clone()) {
        model.open_config(&file);
    }
    // `sim-app --set path=value`, repeatable: any config value by its dotted path, as `sim-cli --set` takes it.
    let sets: Vec<String> = args.windows(2).filter(|w| w[0] == "--set").map(|w| w[1].clone()).collect();
    if let Err((key, detail)) = model.set_at_start(&sets) {
        eprintln!("{}", model.tr.tr_arg(key, &detail));
        std::process::exit(2);
    }
    let window =
        Window { title: model.tr.tr("ui-window-title").into(), resolution: (1600u32, 900u32).into(), ..default() };
    App::new()
        .insert_resource(ClearColor(Color::srgb_u8(29, 32, 37)))
        .insert_resource(State {
            model,
            orbit: Orbit { yaw: 0.35, pitch: 0.2, distance: 2.5, target: Vec3::ZERO },
            parts: Vec::new(),
            floor: None,
            bounds: None,
            fit: false,
            overlays: ui::Overlays::default(),
            panels: ui::PanelState::default(),
            themed: false,
            mesh_config: String::new(),
            part_ids: Vec::new(),
            view_rect: [0.0; 4],
            frame: 0,
            scroll: None,
            recorded: VecDeque::new(),
            timeline: ui::Timeline::default(),
            was_paused: false,
            trail_points: Vec::new(),
            has_mode: false,
            mode_sent: false,
            compare_effect: String::new(),
            ghost: None,
            ghost_parts: Vec::new(),
            recording: None,
            encodings: Vec::new(),
            notes: Default::default(),
        })
        .add_plugins(DefaultPlugins.set(WindowPlugin { primary_window: Some(window), ..default() }))
        .add_plugins(EguiPlugin::default())
        .add_systems(Startup, setup)
        .add_systems(
            Update,
            (
                sync_scene,
                fill_obstacles,
                sync_ghost,
                paint_cad_parts,
                draw_paths,
                orbit_camera,
                follow_tool,
                developer_hooks,
                record,
            )
                .chain(),
        )
        // After the window is closed and the exit asked for, in the same frame: the runner exits right after it.
        .add_systems(Last, finish_videos.after(bevy::window::ExitSystems))
        .add_systems(EguiPrimaryContextPass, panels)
        .run();
}

fn setup(
    mut commands: Commands,
    mut egui_settings: ResMut<EguiGlobalSettings>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut images: ResMut<Assets<Image>>,
    mut egui_textures: ResMut<EguiUserTextures>,
) {
    // The panels get their own camera on top, so the 3D camera can be confined to the free area.
    egui_settings.auto_create_primary_context = false;
    commands.insert_resource(Shapes {
        cube: meshes.add(Cuboid::new(1.0, 1.0, 1.0)),
        cylinder: meshes.add(Cylinder::new(0.5, 1.0)),
    });
    // A studio look: a dark room with softboxes for the metal to reflect (a plain sky gives metal nothing to
    // mirror, so it reads as grey plastic), contact shadows (SSAO, which needs MSAA off, so SMAA smooths the edges
    // instead), film-like tone mapping and a faint glow on highlights. The room is filtered on the graphics card,
    // since it is made here and not shipped prefiltered.
    let sky = GeneratedEnvironmentMapLight { environment_map: images.add(studio()), intensity: 1500.0, ..default() };
    // The tool view renders into a picture the panels show, not into the window: a camera with a viewport there
    // would clear or draw through the main view's picture under it. Only what the colours need is shared with the
    // main camera (HDR, the tone mapping, the sky light, the glow): without them the same parts look different.
    let size = (TOOL_VIEW_SIZE * 2.0).round();
    let picture = images.add(Image::new_target_texture(
        size.x as u32,
        size.y as u32,
        TextureFormat::Rgba8Unorm,
        Some(TextureFormat::Rgba8UnormSrgb),
    ));
    commands.insert_resource(ToolView(egui_textures.add_image(EguiTextureHandle::Strong(picture.clone()))));
    commands.spawn((
        ToolCamera,
        Camera3d::default(),
        Camera { order: -1, is_active: false, ..default() },
        RenderTarget::Image(picture.into()),
        Transform::default(),
        Hdr,
        Tonemapping::TonyMcMapface,
        Bloom::NATURAL,
        sky.clone(),
    ));
    commands.spawn((
        WorldCamera,
        Camera3d::default(),
        Transform::default(),
        Hdr,
        Msaa::Off,
        Smaa::default(),
        ScreenSpaceAmbientOcclusion::default(),
        Tonemapping::TonyMcMapface,
        Bloom::NATURAL,
        sky,
    ));
    commands.spawn((
        PrimaryEguiContext,
        Camera2d,
        RenderLayers::none(),
        Camera {
            order: 1,
            output_mode: CameraOutputMode::Write {
                blend_state: Some(BlendState::ALPHA_BLENDING),
                clear_color: ClearColorConfig::None,
            },
            clear_color: ClearColorConfig::Custom(Color::NONE),
            ..default()
        },
    ));
    commands.spawn((
        DirectionalLight { illuminance: 9000.0, shadow_maps_enabled: true, ..default() },
        Transform::from_xyz(2.0, 4.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
        // The machine is a few metres across: spend the shadow map's resolution there, for crisp shadows.
        CascadeShadowConfigBuilder { first_cascade_far_bound: 3.0, maximum_distance: 15.0, ..default() }.build(),
    ));
    // The sky light fills the shade now, so the back light only rims the edges.
    commands.spawn((
        DirectionalLight { illuminance: 1500.0, ..default() },
        Transform::from_xyz(-3.0, 2.0, -4.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}

/// The studio the parts reflect, as a cubemap: a dark grey room, a floor that bounces a little light back, a large
/// softbox overhead, a wide one low in front (faces turned to
/// the viewer mirror what is behind the viewer, a little below the horizon) and a narrow strip behind the machine
/// that the side faces mirror. Soft edges, so that the
/// reflections on the boxes' flat faces do not cut off hard.
fn studio() -> Image {
    const SIZE: u32 = 128;
    let mut image = Image::new_fill(
        Extent3d { width: SIZE, height: SIZE, depth_or_array_layers: 6 },
        TextureDimension::D2,
        &[0; 8],
        TextureFormat::Rgba16Float,
        RenderAssetUsages::default(),
    );
    image.texture_view_descriptor =
        Some(TextureViewDescriptor { dimension: Some(TextureViewDimension::Cube), ..default() });
    // (where it faces, its width and height as tangents of the half angle, how bright)
    let boxes = [
        (Vec3::new(0.1, 1.0, 0.2), Vec2::new(0.7, 0.4), 6.0),
        (Vec3::new(-0.3, -0.05, 1.0), Vec2::new(0.6, 0.3), 5.0),
        (Vec3::new(0.35, 0.0, -1.0), Vec2::new(0.15, 0.6), 4.0),
    ];
    for face in 0..6 {
        for y in 0..SIZE {
            for x in 0..SIZE {
                let (u, v) = ((x as f32 + 0.5) / SIZE as f32 * 2.0 - 1.0, (y as f32 + 0.5) / SIZE as f32 * 2.0 - 1.0);
                // The cube's faces in their GPU order: +x, -x, +y, -y, +z, -z.
                let d = [
                    Vec3::new(1.0, -v, -u),
                    Vec3::new(-1.0, -v, u),
                    Vec3::new(u, 1.0, v),
                    Vec3::new(u, -1.0, -v),
                    Vec3::new(u, -v, 1.0),
                    Vec3::new(-u, -v, -1.0),
                ][face]
                    .normalize();
                let mut light = 0.05 + 0.1 * (-d.y).max(0.0).sqrt();
                for (towards, half, bright) in boxes {
                    let normal = towards.normalize();
                    let across = normal.any_orthonormal_vector();
                    let up = normal.cross(across);
                    let facing = d.dot(normal);
                    if facing > 0.0 {
                        let p = d / facing;
                        let edge = |at: f32, half: f32| 1.0 - ((at.abs() - half) / (0.15 * half)).clamp(0.0, 1.0);
                        light += bright * edge(p.dot(across), half.x) * edge(p.dot(up), half.y);
                    }
                }
                image.set_color_at_3d(x, y, face as u32, Color::linear_rgb(light, light, light * 1.02)).unwrap();
            }
        }
    }
    image
}

/// Scene coordinates (x right, y into the depth, z up) to Bevy's (x right, y up, z towards the viewer).
fn to_bevy(p: [f64; 3]) -> Vec3 {
    Vec3::new(p[0] as f32, p[2] as f32, -p[1] as f32)
}

fn part_transform(part: &Part) -> Transform {
    let at = to_bevy(part.pos);
    match &part.shape {
        // The part's angle turns from +x towards up, which is a turn about Bevy's z axis.
        Shape::Box { size } => Transform::from_translation(at)
            .with_rotation(Quat::from_rotation_z(part.angle as f32))
            .with_scale(Vec3::new(size[0] as f32, size[2] as f32, size[1] as f32)),
        Shape::Cylinder { radius, length } => Transform::from_translation(at)
            .with_rotation(Quat::from_rotation_x(std::f32::consts::FRAC_PI_2))
            .with_scale(Vec3::new(2.0 * *radius as f32, *length as f32, 2.0 * *radius as f32)),
        Shape::Segment { to, width } => {
            let along = to_bevy(*to) - at;
            let length = along.length().max(1e-6);
            Transform::from_translation(at + along / 2.0)
                .with_rotation(Quat::from_rotation_arc(Vec3::X, along / length))
                .with_scale(Vec3::new(length, 0.004, *width as f32))
        }
    }
}

/// A part drawn from a vendor CAD mesh instead of a box: the index of its part in the scene.
#[derive(Component)]
struct CadPart(usize);

/// The CAD mesh for a part, if there is one: `assets/meshes/<config file name>/<part id>.glb`, made by
/// `catalog/tools/step_to_glb.py`. Without a config file, or without that file, the part stays a box.
/// shortcut: the mesh has the size of the config as it was converted (stroke, Z length); edit those in the
/// app and the mesh no longer matches. Upgrade to checking the stroke when someone edits them often.
fn cad_mesh(config_file: &str, part_id: &str) -> Option<String> {
    let name = std::path::Path::new(config_file).file_stem()?.to_str()?;
    let file = ["meshes", name, &format!("{part_id}.glb")].join("/");
    FileAssetReader::get_base_path().join("assets").join(&file).is_file().then_some(file)
}

/// The scene the 3D view draws: the newest, or while the timeline is away from it the one recorded last at or
/// before the timeline's time.
fn shown<'a>(model: &'a Model, recorded: &'a VecDeque<(f64, Scene)>, timeline: ui::Timeline) -> &'a Scene {
    let Some(time) = timeline.time else { return &model.scene };
    let after = recorded.partition_point(|(t, _)| *t <= time);
    recorded.get(after.saturating_sub(1)).map_or(&model.scene, |(_, scene)| scene)
}

/// The colour of the `index`-th part of `scene` in the load view (`ui::load_color`), where that is switched on.
fn load_color(state: &State, scene: &Scene, index: usize) -> Option<Color> {
    let id = scene.parts[index].id;
    let nth = scene.parts[..index].iter().filter(|p| p.id == id).count();
    state.overlays.load.then(|| ui::load_color(&state.model, id, nth)).flatten().map(palette)
}

/// A CAD mesh's own material, kept while the part is painted in its scene colour.
#[derive(Component)]
struct VendorMaterial(Handle<StandardMaterial>);

/// Keeps the vendor's colours on a CAD part, except where it has none (a STEP file through FreeCAD without
/// its window: the glTF's default material), while the heat, belt or load view colours it, or in x-ray: then the
/// part's own material paints it (see-through in x-ray, `sync_scene`). Runs every frame, since a glTF's meshes arrive some frames after the part is spawned.
fn paint_cad_parts(
    state: Res<State>,
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    cad_parts: Query<(Entity, &CadPart)>,
    children: Query<&Children>,
    mut meshes: Query<(&mut MeshMaterial3d<StandardMaterial>, Option<&VendorMaterial>)>,
) {
    let colourless = GltfAssetLabel::DefaultMaterial.to_string();
    let scene = shown(&state.model, &state.recorded, state.timeline);
    for (root, CadPart(index)) in &cad_parts {
        let (Some(part), Some((_, ours))) = (scene.parts.get(*index), state.parts.get(*index)) else { continue };
        let coloured = part.heat.is_some()
            || part.highlight != 0.0
            || state.overlays.xray
            || load_color(&state, scene, *index).is_some();
        for entity in children.iter_descendants(root) {
            let Ok((mut material, vendor)) = meshes.get_mut(entity) else { continue };
            let own = vendor.map_or(&material.0, |v| &v.0);
            let plain = asset_server.get_path(own).is_some_and(|p| p.label() == Some(colourless.as_str()));
            match (coloured || plain, vendor) {
                (true, None) => {
                    commands.entity(entity).insert(VendorMaterial(std::mem::replace(&mut material.0, ours.clone())));
                }
                (true, Some(_)) if material.0 != *ours => material.0 = ours.clone(),
                (false, Some(VendorMaterial(own))) => {
                    material.0 = own.clone();
                    commands.entity(entity).remove::<VendorMaterial>();
                }
                _ => {}
            }
        }
    }
}

/// Belt stretch and warmth made visible (`model::part_color`).
fn part_color(part: &Part) -> Color {
    let [r, g, b] = sim_app::model::part_color(part.color, part.highlight, part.heat);
    Color::srgb(r, g, b)
}

/// The painted finish: metallic, perceptual roughness and reflectance.
const PAINTED: [f32; 3] = [0.3, 0.45, 0.5];

/// A part's finish in its own colour (as `PAINTED`). Turned steel for round parts, matt rubber for the belt; by
/// colour and size for the rest, so that no machine needs naming: long grey parts are aluminium profiles, small dark
/// ones anodised, the others painted. Aluminium is not brushed (anisotropic): tried on 2026-11-13 with Bevy 0.20's
/// pbr_anisotropy_texture feature and tangents on the box mesh, any strength from 0.25 up drew the long, thin boxes
/// (a beam stretched 1.4 m by 0.09 m) burnt-out white with black faces, while 0 drew them as here. Brushing would
/// need a mesh per part at its true size, or a fixed Bevy; a fairly smooth, mostly metallic finish stands in.
fn finish(part: &Part) -> [f32; 3] {
    let [r, g, b] = part.color;
    let grey = r.max(g).max(b) - r.min(g).min(b) < 0.1;
    let long = part_transform(part).scale.max_element() >= 0.15;
    match part.shape {
        Shape::Cylinder { .. } => [0.9, 0.3, 0.5],
        _ if part.id.starts_with("belt") || part.id == "conveyor-belt" => [0.0, 0.9, 0.3],
        _ if grey && long => [0.8, 0.35, 0.5],
        _ if grey && r.max(g).max(b) < 0.35 => [0.4, 0.45, 0.5],
        _ => PAINTED,
    }
}

/// A colour of the panels' palette in the 3D view, so that amber, cyan and red mean the same in both.
fn palette(color: egui::Color32) -> Color {
    Color::srgb_u8(color.r(), color.g(), color.b())
}

/// How long a force's arrow is drawn: growing with the force's order of magnitude, so that 10 N and 1000 N both
/// show (0.03 m and 0.2 m).
fn arrow_length(force: f32) -> f32 {
    0.1 * (1.0 + force / 10.0).log10()
}

/// The planned path as a line in the 3D view, and over it the path the tool has really travelled since; a grid on
/// the floor; where they are switched on, the force arrows and where the plan has the tool; a box around the part
/// picked by a click; and the obstacles of the config with the clearance to the nearest.
fn draw_paths(state: Res<State>, mut gizmos: Gizmos) {
    if let Some((_, middle, width)) = state.floor {
        let flat = Isometry3d::new(middle, Quat::from_rotation_x(std::f32::consts::FRAC_PI_2));
        gizmos.grid(flat, UVec2::splat((width / 0.1) as u32), Vec2::splat(0.1), Color::srgba(1.0, 1.0, 1.0, 0.05));
    }
    let scene = shown(&state.model, &state.recorded, state.timeline);
    // The part's own box, a little larger so that it shows around it. A CAD part's box is the size the scene gives
    // it, which its mesh roughly fills.
    if let Some((id, nth)) = state.panels.part
        && let Some(part) = scene.parts.iter().filter(|p| p.id == id).nth(nth)
    {
        let mut around = part_transform(part);
        around.scale = around.scale.abs() * 1.06 + Vec3::splat(0.006);
        // Cyan, as the panels mark what is picked.
        gizmos.cube(around, palette(ui::ACTUAL));
    }
    if state.overlays.forces {
        for arrow in &scene.arrows {
            let force = to_bevy(arrow.force);
            if force.length() > 0.5 {
                let at = to_bevy(arrow.pos);
                gizmos.arrow(at, at + force.normalize() * arrow_length(force.length()), palette(ui::LIMIT));
            }
        }
    }
    if state.overlays.planned
        && let (Some(tool), Some(planned)) = (scene.tool, scene.planned)
    {
        // The way from the tool to the plan, as many times longer as the deflections are drawn.
        let (tool, planned) = (to_bevy(tool), to_bevy(planned));
        let ghost = tool + (planned - tool) * state.model.exaggeration as f32;
        gizmos.line(tool, ghost, GAP);
        gizmos.sphere(Isometry3d::from_translation(ghost), 0.012, GAP);
    }
    // The obstacles' edges, over their translucent faces (`fill_obstacles`), red while a part touches one, and the
    // clearance to the nearest as a line (its value is a label, `panels`).
    for obstacle in &scene.obstacles {
        let (centre, size) = (to_bevy(obstacle.aabb.centre), to_bevy(obstacle.aabb.size).abs());
        let colour = palette(if obstacle.contact { ui::LIMIT } else { ui::dim() });
        gizmos.cube(Transform::from_translation(centre).with_scale(size), colour.with_alpha(0.6));
    }
    if let Some((from, to, _)) = scene.clearance {
        gizmos.line(to_bevy(from), to_bevy(to), palette(ui::LIMIT));
    }
    if let Some(run) = &state.model.path_run {
        gizmos.linestrip(run.scene_line.iter().map(|p| to_bevy(*p)), palette(ui::PLAN));
    }
    // The travelled path up to the time the timeline shows: scrubbed back, the tool had not been further yet. The
    // points with their times are the window's (`trail_points`); the plain path has none, so scrubbed it is drawn
    // from those.
    let Some(trail) = &state.model.trail else { return };
    let (by, tube, all) = (state.overlays.trail, state.overlays.tube, &state.trail_points);
    let points = trail_until(all, state.timeline.time);
    // Scrubbed: the tool's error at that moment as an arrow from the path to the tool, as many times longer as the
    // deflections are drawn, so that its direction across or along the path shows.
    if let (Some(_), Some(p)) = (state.timeline.time, points.last()) {
        let (from, to) = (to_bevy(p.planned), to_bevy(p.at));
        if from.distance(to) > 0.0 {
            gizmos.arrow(from, from + (to - from) * state.model.exaggeration as f32, GAP);
        }
    }
    if all.is_empty() || (by == ui::TrailColor::Plain && !tube && state.timeline.time.is_none()) {
        gizmos.linestrip(trail.iter().map(|p| to_bevy(*p)), palette(ui::ACTUAL));
        return;
    }
    // In the tube each point stands off the plan as many times further as the deflections are drawn.
    let factor = if tube { state.model.exaggeration } else { 1.0 };
    let place = |p: &TrailPoint| to_bevy(std::array::from_fn(|i| p.planned[i] + (p.at[i] - p.planned[i]) * factor));
    // The scale is the whole path's (or the one fixed), so that it does not change while scrubbing.
    let range = trail_range(all, by).map(|range| ui::trail_scale(state.overlays, range));
    let tolerance = state.overlays.tolerance.filter(|_| by == ui::TrailColor::FollowingError);
    let color = |p: &TrailPoint| match range {
        _ if tolerance.is_some_and(|limit| trail_value(p, by) > limit) => palette(ui::LIMIT),
        Some((low, high)) if by != ui::TrailColor::Plain => {
            let [r, g, b] = ui::trail_color(((trail_value(p, by) - low) / (high - low).max(1e-12)) as f32);
            Color::srgb(r, g, b)
        }
        _ => palette(ui::ACTUAL),
    };
    gizmos.linestrip_gradient(points.iter().map(|p| (place(p), color(p))));
    if tube {
        // The gap itself every 2 cm of travel, from the plan to the point drawn.
        let mut since = f64::INFINITY;
        for pair in points.windows(2) {
            since += (0..3).map(|i| (pair[1].at[i] - pair[0].at[i]).powi(2)).sum::<f64>().sqrt();
            if since >= 0.02 {
                gizmos.line(to_bevy(pair[1].planned), place(&pair[1]), GAP);
                since = 0.0;
            }
        }
    }
}

/// The value a point of the travelled path is coloured by: the tool's speed in m/s, or its distance from the path the
/// plan drew in m (`Runner::trail_point`).
fn trail_value(p: &TrailPoint, by: ui::TrailColor) -> f64 {
    match by {
        ui::TrailColor::Speed => p.speed,
        _ => (0..3).map(|i| (p.at[i] - p.planned[i]).powi(2)).sum::<f64>().sqrt(),
    }
}

/// The lowest and highest value along the travelled path, for its colour scale.
fn trail_range(points: &[TrailPoint], by: ui::TrailColor) -> Option<(f64, f64)> {
    let values = points.iter().map(|p| trail_value(p, by));
    values.fold(None, |range, v| Some(range.map_or((v, v), |(low, high): (f64, f64)| (low.min(v), high.max(v)))))
}

/// The tool view's camera in front of the tool of the scene the view shows, looking at it; switched off with the
/// view, and where the scene has no tool.
fn follow_tool(state: Res<State>, camera: Single<(&mut Transform, &mut Camera), With<ToolCamera>>) {
    let (mut transform, mut camera) = camera.into_inner();
    let tool = shown(&state.model, &state.recorded, state.timeline).tool.map(to_bevy);
    camera.is_active = state.overlays.tool_view && tool.is_some();
    if let Some(tool) = tool {
        *transform = Transform::from_translation(tool + TOOL_VIEW_FROM).looking_at(tool, Vec3::Y);
    }
}

/// Starts the run compared beside the machine's: the config as it is with `effect` (a config path) switched the other
/// way. One that cannot start is said in the messages.
fn start_ghost(state: &mut State, effect: String) {
    state.compare_effect = effect.clone();
    let model = &mut state.model;
    let mut config = model.config.clone();
    let on = !config.bool(&effect).unwrap_or(false);
    match config.set(&effect, toml::Value::Boolean(on)).and_then(|()| Session::start(model.entry, &config)) {
        Ok(session) => {
            state.ghost = Some(Ghost {
                session,
                effect,
                on,
                entry: model.entry.id,
                base: model.config.clone(),
                scene: Scene::default(),
                sent: None,
                history: History::new(sim_app::model::HISTORY),
                recorded: VecDeque::new(),
            })
        }
        Err(e) => model.messages.push((e.key(), e.detail().into())),
    }
}

/// The ghost's scene the view draws: as `shown` picks the machine's, from the ghost's recorded scenes.
fn shown_ghost(ghost: &Ghost, timeline: ui::Timeline) -> &Scene {
    let Some(time) = timeline.time else { return &ghost.scene };
    let after = ghost.recorded.partition_point(|(t, _)| *t <= time);
    ghost.recorded.get(after.saturating_sub(1)).map_or(&ghost.scene, |(_, scene)| scene)
}

/// A command for the machine, and the same for the run compared beside it, so that both do the same.
fn command(state: &mut State, command: Command) {
    if let Some(ghost) = &state.ghost {
        ghost.session.send(Request::Command(command.clone()));
    }
    state.model.command(command);
}

/// The compared run: ended where the machine or its config changed, kept running, paused and drawn larger as the
/// machine's is, and drawn as see-through boxes of its scene's parts, on entities of its own. Its scenes are recorded
/// as the machine's are: scrubbed, the timeline shows both at the same time.
fn sync_ghost(
    mut state: ResMut<State>,
    mut commands: Commands,
    shapes: Res<Shapes>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut transforms: Query<&mut Transform, With<GhostPart>>,
    mut material: Local<Option<Handle<StandardMaterial>>>,
) {
    let state = &mut *state;
    if state.ghost.as_ref().is_some_and(|g| g.entry != state.model.entry.id || g.base != state.model.config) {
        state.ghost = None;
    }
    let Some(ghost) = &mut state.ghost else {
        state.ghost_parts.drain(..).for_each(|(entity, _)| commands.entity(entity).despawn());
        return;
    };
    // Waits while the machine waits: paused, scrubbed back, or showing its mode.
    let wanted = (state.model.paused || state.mode_sent, state.model.speed, state.model.exaggeration);
    if ghost.sent != Some(wanted) {
        [Request::Pause(wanted.0), Request::Speed(wanted.1), Request::Exaggeration(wanted.2)]
            .into_iter()
            .for_each(|r| ghost.session.send(r));
        ghost.sent = Some(wanted);
    }
    let (now, window) = (state.model.history.back().map(|row| row[0]), state.model.window_s);
    for event in ghost.session.drain() {
        match event {
            Event::Sample(sample) => ghost.history.push(&sample),
            Event::Scene(scene) => {
                // Stamped with the machine's time: the two runs started at different times on clocks of their own.
                if let Some(now) = now.filter(|_| !state.mode_sent) {
                    let recorded = &mut ghost.recorded;
                    if recorded.back().is_some_and(|(t, _)| *t > now) {
                        recorded.clear();
                    }
                    if recorded.back().is_none_or(|(t, _)| now - *t >= window / RECORDED as f64) {
                        recorded.push_back((now, scene.clone()));
                    }
                    while recorded.len() > RECORDED || recorded.front().is_some_and(|(t, _)| *t < now - window) {
                        recorded.pop_front();
                    }
                }
                ghost.scene = scene
            }
            // A command the compared run refuses leaves it standing: say so, or it looks like the effect did it.
            Event::Refused(key, detail) => {
                let why = format!("{} {}", state.model.tr.tr(key), detail);
                state.model.messages.push(("ui-compare-refused", why.trim().to_string()));
            }
            _ => {}
        }
    }
    let material = material
        .get_or_insert_with(|| {
            materials.add(StandardMaterial { base_color: GHOST, alpha_mode: AlphaMode::Blend, ..default() })
        })
        .clone();
    let parts = &shown_ghost(ghost, state.timeline).parts;
    for (entity, _) in state.ghost_parts.drain(parts.len().min(state.ghost_parts.len())..) {
        commands.entity(entity).despawn();
    }
    for (index, part) in parts.iter().enumerate() {
        let round = matches!(part.shape, Shape::Cylinder { .. });
        let mesh = if round { shapes.cylinder.clone() } else { shapes.cube.clone() };
        match state.ghost_parts.get_mut(index) {
            Some((entity, was_round)) => {
                if *was_round != round {
                    commands.entity(*entity).insert(Mesh3d(mesh));
                    *was_round = round;
                }
                if let Ok(mut transform) = transforms.get_mut(*entity) {
                    *transform = part_transform(part);
                }
            }
            None => {
                let entity = commands
                    .spawn((
                        GhostPart,
                        Mesh3d(mesh),
                        MeshMaterial3d(material.clone()),
                        part_transform(part),
                        NotShadowCaster,
                    ))
                    .id();
                state.ghost_parts.push((entity, round));
            }
        }
    }
}

/// Takes in new samples, records the scene, and moves the drawn parts to where the scene the view shows says they are.
fn sync_scene(
    mut state: ResMut<State>,
    mut commands: Commands,
    shapes: Res<Shapes>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    asset_server: Res<AssetServer>,
    mut transforms: Query<(&mut Transform, Has<CadPart>), Without<WorldCamera>>,
    clock: Res<Time>,
) {
    let state = &mut *state;
    state.model.poll();
    let news = state.model.session.view_news();
    state.has_mode = news.has_mode;
    if news.restarted {
        state.trail_points.clear();
        // The machine took a path (the model sends it by itself when its plan arrives): the compared run takes it too.
        if let (Some(ghost), Some(run), false) = (&state.ghost, &state.model.path_run, news.trail.is_empty()) {
            ghost.session.send(Request::Command(Command::Trajectory(run.points.clone())));
        }
    }
    state.trail_points.extend(news.trail);
    state.trail_points.drain(..state.trail_points.len().saturating_sub(sim_app::session::TRAIL));
    let mode = state.overlays.mode && state.has_mode;
    if mode != state.mode_sent {
        state.model.session.send(Request::Mode(mode));
        state.mode_sent = mode;
    }
    if let Some(now) = state.model.history.back().map(|row| row[0]) {
        let recorded = &mut state.recorded;
        // Time running again from the start is another run (another machine or config): its scenes do not follow.
        if recorded.back().is_some_and(|(t, _)| *t > now) {
            recorded.clear();
        }
        // Only a changed scene (paused, the same one arrives again and again), and not closer to the last than a
        // `RECORDED`th of the window, so that the recording reaches back over all of it.
        let step = state.model.window_s / RECORDED as f64;
        if !state.mode_sent && recorded.back().is_none_or(|(t, scene)| now - *t >= step && *scene != state.model.scene)
        {
            recorded.push_back((now, state.model.scene.clone()));
        }
        while recorded.len() > RECORDED || recorded.front().is_some_and(|(t, _)| *t < now - state.model.window_s) {
            recorded.pop_front();
        }
    }
    let timeline = &mut state.timeline;
    if timeline.playing
        && let (Some(time), Some((end, _))) = (timeline.time, state.recorded.back())
    {
        // The replay stops at the newest scene, and stays there rather than going live, to be looked at again.
        let next = time + clock.delta_secs_f64() * timeline.speed;
        (timeline.time, timeline.playing) = (Some(next.min(*end)), next < *end);
    }
    let scene = shown(&state.model, &state.recorded, state.timeline);
    let ids: Vec<&'static str> = scene.parts.iter().map(|p| p.id).collect();
    // Nothing is drawn before the panels have been laid out once: the camera is fitted to the view they leave.
    if state.view_rect[2] <= state.view_rect[0] {
        return;
    }
    if ids != state.part_ids || state.model.inputs.config_file != state.mesh_config {
        // The parts changed order, or one came or went (the payload), or it is another machine or config file.
        // The drawn parts stay, matched again by id and by how many of that id came before (the second
        // "post"): the belt reorders its parts nearly every frame, and a CAD mesh spawned anew takes frames to
        // load. Another config file may bring other CAD meshes, so then everything is drawn anew.
        // Drawn parts that were forgotten (`part_ids` cleared for a new machine or config) are drawn anew too.
        let same_config =
            state.model.inputs.config_file == state.mesh_config && state.part_ids.len() == state.parts.len();
        let mut kept = std::collections::HashMap::new();
        let mut seen = std::collections::HashMap::new();
        for (old_index, (entity, material)) in state.parts.drain(..).enumerate() {
            if same_config {
                kept.insert(occurrence(&mut seen, state.part_ids[old_index]), (entity, material));
            } else {
                commands.entity(entity).despawn();
            }
        }
        state.mesh_config = state.model.inputs.config_file.clone();
        seen.clear();
        for (index, part) in scene.parts.iter().enumerate() {
            if let Some((entity, material)) = kept.remove(&occurrence(&mut seen, part.id)) {
                if transforms.get(entity).is_ok_and(|(_, cad)| cad) {
                    commands.entity(entity).insert(CadPart(index));
                }
                state.parts.push((entity, material));
                continue;
            }
            // The colour and finish are set by the move below, as the part is drawn.
            let material = materials.add(StandardMaterial::default());
            if let Some(file) = cad_mesh(&state.mesh_config, part.id) {
                let at = part_transform(part).with_scale(Vec3::ONE);
                let entity = commands
                    .spawn((
                        WorldAssetRoot(asset_server.load(GltfAssetLabel::Scene(0).from_asset(file))),
                        at,
                        CadPart(index),
                    ))
                    .id();
                state.parts.push((entity, material));
                continue;
            }
            let mesh = if matches!(part.shape, Shape::Cylinder { .. }) {
                shapes.cylinder.clone()
            } else {
                shapes.cube.clone()
            };
            let entity = commands.spawn((Mesh3d(mesh), MeshMaterial3d(material.clone()), part_transform(part))).id();
            state.parts.push((entity, material));
        }
        for (entity, _) in kept.into_values() {
            commands.entity(entity).despawn();
        }
        if state.part_ids.is_empty() && !scene.parts.is_empty() {
            // First scene of a machine: the box its parts fill, for the camera to frame, and a floor under it.
            let corners = scene.parts.iter().flat_map(|p| {
                let t = part_transform(p);
                // Half the box along each axis: what its turned edges reach together.
                let (turned, half) = (Mat3::from_quat(t.rotation), t.scale.abs() / 2.0);
                let reach = turned.x_axis.abs() * half.x + turned.y_axis.abs() * half.y + turned.z_axis.abs() * half.z;
                [t.translation - reach, t.translation + reach]
            });
            let (low, high) =
                corners.fold((Vec3::splat(f32::MAX), Vec3::splat(f32::MIN)), |(l, h), c| (l.min(c), h.max(c)));
            state.bounds = Some((low, high));
            state.orbit.target = (low + high) / 2.0;
            state.fit = true;
            if let Some((old, ..)) = state.floor.take() {
                commands.entity(old).despawn();
            }
            // A floor some way around the machine, not to the horizon: under the dark sky it reads as a stage.
            let width = (high - low).x.max((high - low).z) * 2.2 + 0.6;
            let middle = Vec3::new(state.orbit.target.x, low.y - 0.02, state.orbit.target.z);
            let ground = materials.add(StandardMaterial {
                base_color: Color::srgb(0.17, 0.18, 0.2),
                perceptual_roughness: 0.95,
                ..default()
            });
            let place = Transform::from_translation(middle).with_scale(Vec3::new(width, 0.02, width));
            let floor = commands.spawn((Mesh3d(shapes.cube.clone()), MeshMaterial3d(ground), place)).id();
            state.floor = Some((floor, middle + Vec3::Y * 0.0105, width));
        }
        state.part_ids = ids;
        // On to the move below: the parts kept were drawn where the last scene had them.
    }
    for (index, (part, (entity, material))) in scene.parts.iter().zip(&state.parts).enumerate() {
        let mut cad = false;
        if let Ok((mut transform, is_cad)) = transforms.get_mut(*entity) {
            // A CAD mesh is drawn at its true size; only the boxes are stretched to the part.
            *transform = if is_cad { part_transform(part).with_scale(Vec3::ONE) } else { part_transform(part) };
            cad = is_cad;
        }
        // The load view's colour in place of the part's own; x-ray: CAD parts see-through, to show the belt and pulleys they hide.
        let load = load_color(state, scene, index);
        let own = load.unwrap_or_else(|| part_color(part));
        let (color, mode) = if cad && state.overlays.xray {
            (own.with_alpha(0.18), AlphaMode::Blend)
        } else {
            (own, AlphaMode::Opaque)
        };
        // A meaning colour (heat, load, belt tension) is drawn painted on every part: a metal finish darkens it, so
        // the same colour would read differently on a profile than on a motor.
        let meaning = load.is_some() || part.heat.is_some() || part.highlight != 0.0;
        let [metallic, roughness, reflectance] = if meaning { PAINTED } else { finish(part) };
        // Only where something changed, so that the other materials are not sent to the graphics card again.
        if materials.get(material).is_some_and(|m| {
            m.base_color != color
                || m.alpha_mode != mode
                || [m.metallic, m.perceptual_roughness, m.reflectance] != [metallic, roughness, reflectance]
        }) && let Some(mut material) = materials.get_mut(material)
        {
            (material.base_color, material.alpha_mode) = (color, mode);
            (material.metallic, material.perceptual_roughness, material.reflectance) =
                (metallic, roughness, reflectance);
        }
    }
}

/// The faces of an obstacle of the config, as `fill_obstacles` draws them.
#[derive(Component)]
struct ObstacleFace;

/// The obstacles of the config as translucent boxes, the colour of their edges (`draw_paths`): grey, red while a part
/// touches one. Solid enough to see where they stand, thin enough to see the machine behind them. One box per obstacle
/// of the scene shown, made anew when their number changes, moved and coloured every frame.
fn fill_obstacles(
    state: Res<State>,
    mut commands: Commands,
    shapes: Res<Shapes>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut faces: Local<Vec<(Entity, Handle<StandardMaterial>)>>,
    mut transforms: Query<&mut Transform, With<ObstacleFace>>,
) {
    let scene = shown(&state.model, &state.recorded, state.timeline);
    if faces.len() != scene.obstacles.len() {
        for (entity, _) in faces.drain(..) {
            commands.entity(entity).despawn();
        }
        for obstacle in &scene.obstacles {
            // Rough and unlit by reflections, so that the faces read as a tint and not as glass.
            let material = materials.add(StandardMaterial {
                alpha_mode: AlphaMode::Blend,
                perceptual_roughness: 1.0,
                reflectance: 0.0,
                ..default()
            });
            let place = Transform::from_translation(to_bevy(obstacle.aabb.centre))
                .with_scale(to_bevy(obstacle.aabb.size).abs());
            let entity = commands
                .spawn((
                    Mesh3d(shapes.cube.clone()),
                    MeshMaterial3d(material.clone()),
                    place,
                    ObstacleFace,
                    NotShadowCaster,
                ))
                .id();
            faces.push((entity, material));
        }
    }
    for (obstacle, (entity, material)) in scene.obstacles.iter().zip(faces.iter()) {
        if let Ok(mut transform) = transforms.get_mut(*entity) {
            *transform = Transform::from_translation(to_bevy(obstacle.aabb.centre))
                .with_scale(to_bevy(obstacle.aabb.size).abs());
        }
        let color = palette(if obstacle.contact { ui::LIMIT } else { ui::dim() }).with_alpha(0.22);
        if materials.get(material).is_some_and(|m| m.base_color != color)
            && let Some(mut m) = materials.get_mut(material)
        {
            m.base_color = color;
        }
    }
}

/// How far back the camera must stand, looking along `rotation` at `target`, for the box from `low` to `high`
/// to fill the view with a little room: each corner within the vertical field of view of 45 degrees and the
/// horizontal one the view's `aspect` gives.
fn fit_distance(low: Vec3, high: Vec3, target: Vec3, rotation: Quat, aspect: f32) -> f32 {
    let tan_v = std::f32::consts::FRAC_PI_8.tan();
    let tan_h = tan_v * aspect;
    let into_camera = rotation.inverse();
    let corners =
        (0..8).map(|i| Vec3::new([low.x, high.x][i & 1], [low.y, high.y][(i >> 1) & 1], [low.z, high.z][i >> 2]));
    // Seen from the camera, a corner at (x, y) and depth z towards it needs the camera at least that far back.
    let needed = corners.map(|c| into_camera * (c - target)).map(|p| p.z + (p.x.abs() / tan_h).max(p.y.abs() / tan_v));
    1.08 * needed.fold(0.1, f32::max)
}

/// A part's key across scenes: its id and how many parts of that id came before it.
fn occurrence(seen: &mut std::collections::HashMap<&'static str, usize>, id: &'static str) -> (&'static str, usize) {
    let count = seen.entry(id).or_default();
    *count += 1;
    (id, *count - 1)
}

/// Orbit with the left mouse button, zoom with the wheel, while the pointer is over the 3D view. A click there
/// without dragging picks the part under the pointer, or on empty space lets go of the one picked.
#[allow(clippy::too_many_arguments)]
fn orbit_camera(
    mut state: ResMut<State>,
    camera: Single<(&mut Transform, &Camera, &GlobalTransform), With<WorldCamera>>,
    window: Single<&Window, With<PrimaryWindow>>,
    buttons: Res<ButtonInput<MouseButton>>,
    motion: Res<AccumulatedMouseMotion>,
    scroll: Res<AccumulatedMouseScroll>,
    mut contexts: EguiContexts,
    mut ray_cast: MeshRayCast,
    parents: Query<&ChildOf>,
    cad: Query<(), With<CadPart>>,
    ghosts: Query<(), Or<(With<GhostPart>, With<ObstacleFace>)>>,
    mut pressed_at: Local<Option<Vec2>>,
) {
    let (mut camera, lens, seen_from) = camera.into_inner();
    let [left, top, right, bottom] = state.view_rect;
    // Over the view, but not over the controls that float on it, nor while a panel's control is being dragged.
    // The panels are drawn on egui's background layer; anything above it over the view (its controls, a menu, a
    // popup, a tooltip) takes the pointer, as does a control being dragged.
    let over_egui = contexts.ctx_mut().is_ok_and(|ctx| {
        let above = ctx
            .input(|i| i.pointer.hover_pos())
            .and_then(|p| ctx.layer_id_at(p))
            .is_some_and(|layer| layer.order != egui::Order::Background);
        above || ctx.egui_is_using_pointer()
    });
    let over_view =
        !over_egui && window.cursor_position().is_some_and(|p| p.x > left && p.x < right && p.y > top && p.y < bottom);
    if buttons.just_pressed(MouseButton::Left) {
        *pressed_at = window.cursor_position().filter(|_| over_view);
    }
    // A click, not the end of a turn of the camera: let go within a few pixels of where it was pressed.
    if buttons.just_released(MouseButton::Left)
        && let (Some(from), Some(at)) = (pressed_at.take(), window.cursor_position())
        && from.distance(at) < 4.0
    {
        // In x-ray the CAD parts are see-through: a click goes through them to the belt and pulleys they hide.
        let xray = state.overlays.xray;
        // The compared run's ghost is never picked, nor an obstacle's faces: they are not the machine's parts.
        let through = |e: Entity| {
            !ghosts.contains(e)
                && !(xray && std::iter::once(e).chain(parents.iter_ancestors(e)).any(|a| cad.contains(a)))
        };
        let settings = MeshRayCastSettings::default().with_filter(&through);
        let hit = lens
            .viewport_to_world(seen_from, at)
            .ok()
            .and_then(|ray| ray_cast.cast_ray(ray, &settings).first().map(|(entity, _)| *entity));
        // A CAD mesh's pieces are children of its part's entity, some levels down.
        let index = hit.and_then(|hit| {
            std::iter::once(hit)
                .chain(parents.iter_ancestors(hit))
                .find_map(|e| state.parts.iter().position(|(part, _)| *part == e))
        });
        let ids = &state.part_ids;
        let picked = index.map(|i| (ids[i], ids[..i].iter().filter(|id| **id == ids[i]).count()));
        state.panels.part = picked;
    }
    let state = &mut *state;
    let orbit = &mut state.orbit;
    if over_view {
        if buttons.pressed(MouseButton::Left) {
            orbit.yaw -= motion.delta.x * 0.005;
            orbit.pitch = (orbit.pitch + motion.delta.y * 0.005).clamp(-1.55, 1.55);
        }
        orbit.distance = (orbit.distance * (1.0 - scroll.delta.y * 0.1)).clamp(0.2, 20.0);
    }
    let rotation = Quat::from_euler(EulerRot::YXZ, orbit.yaw, -orbit.pitch, 0.0);
    if state.fit
        && let Some((low, high)) = state.bounds
        && right > left
        && bottom > top
    {
        orbit.distance = fit_distance(low, high, orbit.target, rotation, (right - left) / (bottom - top));
        state.fit = false;
    }
    camera.translation = orbit.target + rotation * Vec3::new(0.0, 0.0, orbit.distance);
    camera.rotation = rotation;
}

fn panels(
    mut contexts: EguiContexts,
    mut state: ResMut<State>,
    camera: Single<(&mut Camera, &GlobalTransform), With<WorldCamera>>,
    mut window: Single<&mut Window, With<PrimaryWindow>>,
    tool_view: Res<ToolView>,
    mut commands: Commands,
) -> Result {
    let (mut camera, camera_transform) = camera.into_inner();
    let ctx = contexts.ctx_mut()?;
    let state = &mut *state;
    let mut actions = Vec::new();
    if !state.themed {
        ui::apply_theme(ctx, state.panels.light);
        state.themed = true;
    }
    let mut root = egui::Ui::new(
        ctx.clone(),
        "root".into(),
        egui::UiBuilder::new().layer_id(egui::LayerId::background()).max_rect(ctx.viewport_rect()),
    );
    // The bar on top and the scopes below span the window; setup and tools flank the view between them.
    let edge = egui::Frame::new().fill(ui::graphite()).inner_margin(egui::Margin::symmetric(10, 6));
    let top = egui::Panel::top("bar")
        .exact_size(52.0)
        .frame(edge)
        .show(&mut root, |ui| ui::top_bar(ui, &state.model, state.panels, state.recording.is_some(), &mut actions))
        .response
        .rect;
    let bottom = egui::Panel::bottom("scopes")
        .resizable(true)
        .default_size(250.0)
        .frame(edge)
        .show(&mut root, |ui| ui::scopes(ui, &state.model, state.timeline, state.panels, &mut actions))
        .response
        .rect;
    let side = egui::Frame::new().fill(ui::graphite()).inner_margin(egui::Margin::same(10));
    let left = egui::Panel::left("setup")
        .resizable(true)
        .default_size(270.0)
        .frame(side)
        .show(&mut root, |ui| ui::setup(ui, &state.model, state.panels, &mut actions))
        .response
        .rect;
    let scene = shown(&state.model, &state.recorded, state.timeline);
    let right = egui::Panel::right("tools")
        .resizable(true)
        .default_size(430.0)
        .frame(side)
        .show(&mut root, |ui| {
            let ghost = state.ghost.is_some();
            ui::tools(
                ui,
                &state.model,
                scene,
                state.panels,
                &mut actions,
                state.scroll,
                ghost,
                state.overlays.tolerance,
            )
        })
        .response
        .rect;

    // The 3D camera draws into what the panels leave free.
    let scale = window.scale_factor();
    let (view_top, view_bottom) = (top.bottom(), bottom.top());
    let position = UVec2::new((left.right() * scale) as u32, (view_top * scale) as u32);
    let size = UVec2::new(
        ((right.left() - left.right()).max(0.0) * scale) as u32,
        ((view_bottom - view_top).max(0.0) * scale) as u32,
    );
    if size.x > 0 && size.y > 0 {
        camera.viewport = Some(Viewport { physical_position: position, physical_size: size, ..default() });
    }
    state.view_rect = [left.right(), view_top, right.left(), view_bottom];
    // The view's own controls float over its top left corner. While recording, they, the timeline and the tool
    // view are hidden, so that the frames show the view alone; Stop is then in the top bar.
    let recording = state.recording.is_some();
    let controls = (!recording).then(|| {
        egui::Area::new("view-controls".into())
            .fixed_pos(egui::pos2(left.right() + 10.0, view_top + 10.0))
            .show(ctx, |ui| {
                if state.compare_effect.is_empty() {
                    state.compare_effect = format!("effects.{}", state.model.inputs.compare_effect);
                }
                let view = ui::ViewInfo {
                    has_mode: state.has_mode,
                    range: trail_range(&state.trail_points, state.overlays.trail),
                    compare: (state.ghost.is_some(), &state.compare_effect),
                    recording: state.recording.is_some(),
                };
                ui::view_controls(ui, &state.model, state.overlays, view, &mut actions)
            })
            .response
            .rect
    });
    // The timeline floats over the view's bottom edge, once there is a scene to go back to.
    let span = state.recorded.front().zip(state.recorded.back()).map(|((first, _), (last, _))| (*first, *last));
    let timeline = span.filter(|_| !recording).map(|span| {
        let width = (right.left() - left.right() - 36.0).max(100.0);
        let area = egui::Area::new("timeline".into())
            .pivot(egui::Align2::LEFT_BOTTOM)
            .fixed_pos(egui::pos2(left.right() + 10.0, view_bottom - 10.0));
        area.show(ctx, |ui| ui::timeline(ui, &state.model, state.timeline, span, width, &mut actions)).response.rect
    });
    // The tool view in the view's bottom right corner, above the timeline.
    let tool_picture = (state.overlays.tool_view && !recording).then(|| {
        let above = timeline.map_or(view_bottom, |t| t.top()) - 8.0;
        let area = egui::Area::new("tool-view".into())
            .pivot(egui::Align2::RIGHT_BOTTOM)
            .fixed_pos(egui::pos2(right.left() - 10.0, above));
        let frame = egui::Frame::new().stroke(egui::Stroke::new(1.0, ui::machined())).corner_radius(3);
        area.show(ctx, |ui| frame.show(ui, |ui| ui.image((tool_view.0, TOOL_VIEW_SIZE)))).response.rect
    });
    // Space replays, the arrow keys step, while the pointer is over the view (or the timeline on it) and no field
    // takes the keys.
    let [view_left, view_top, view_right, view_bottom] = state.view_rect;
    let over_view = ctx
        .input(|i| i.pointer.hover_pos())
        .is_some_and(|p| p.x > view_left && p.x < view_right && p.y > view_top && p.y < view_bottom);
    if let Some(span) = span
        && over_view
        && ctx.memory(|m| m.focused().is_none())
    {
        let (space, back, on) = ctx.input(|i| {
            (i.key_pressed(egui::Key::Space), i.key_pressed(egui::Key::ArrowLeft), i.key_pressed(egui::Key::ArrowRight))
        });
        if space {
            actions.push(Action::Timeline(state.timeline.toggled(span)));
        }
        if back || on {
            actions.push(Action::Step(on));
        }
    }

    // Live values next to the parts they belong to, e.g. each motor's speed.
    let painter = ctx.layer_painter(egui::LayerId::background());
    let values = scene.labels.iter().map(|label| (to_bevy(label.pos), ui::label_text(&state.model, label)));
    // What the compared run has switched, under its tool.
    let ghost = state.ghost.as_ref().and_then(|g| {
        let effect = state
            .model
            .tr
            .tr(&format!("effect-{}", g.effect.strip_prefix("effects.").unwrap_or(&g.effect)))
            .to_string();
        let text = state.model.tr.tr_arg(if g.on { "ui-compare-ghost-on" } else { "ui-compare-ghost-off" }, &effect);
        Some((to_bevy(g.scene.tool?) - Vec3::Y * 0.12, text))
    });
    // Each force's value just past the tip of its arrow.
    let forces = scene.arrows.iter().filter(|_| state.overlays.forces).filter_map(|arrow| {
        let force = to_bevy(arrow.force);
        let size = force.length();
        let text = format!("{}  {:.0} {}", state.model.tr.tr(arrow.owner_key), size, "N");
        (size > 0.5).then(|| (to_bevy(arrow.pos) + force / size * (arrow_length(size) + 0.03), text))
    });
    // The clearance to the nearest obstacle in mm, at the middle of its line (`draw_paths`).
    let clearance = scene.clearance.map(|(from, to, gap)| {
        let middle = (to_bevy(from) + to_bevy(to)) / 2.0;
        (middle, state.model.tr.tr_arg("ui-clearance", &format!("{:.1}", gap * 1e3)))
    });
    // Each label where its point is seen, if that is in the view.
    let view = egui::Rect::from_min_max(egui::pos2(view_left, view_top), egui::pos2(view_right, view_bottom));
    let mut labels: Vec<_> = values
        .chain(forces)
        .chain(ghost)
        .chain(clearance)
        .filter_map(|(pos, text)| {
            let at = camera.world_to_viewport(camera_transform, pos).ok()?;
            let at = egui::pos2(at.x, at.y);
            view.contains(at)
                .then(|| (at, painter.layout_no_wrap(text, egui::FontId::proportional(14.0), egui::Color32::WHITE)))
        })
        .collect();
    // Top to bottom, each label where it is free of those placed before, of the view's controls and of the timeline.
    labels.sort_by(|a, b| a.0.y.total_cmp(&b.0.y));
    let mut taken: Vec<egui::Rect> =
        [controls, timeline, tool_picture].into_iter().flatten().map(|r| r.expand(4.0)).collect();
    for (at, galley) in labels {
        let size = galley.size() + egui::vec2(10.0, 6.0);
        let room = view.shrink(4.0);
        // Within the view across; then down or up by a label's height at a time, nearest first.
        let x = at.x.clamp(room.left() + size.x / 2.0, (room.right() - size.x / 2.0).max(room.left() + size.x / 2.0));
        let free = (0..24)
            .map(|i| (if i % 2 == 0 { 1.0 } else { -1.0 }) * ((i + 1) / 2) as f32 * (size.y + 2.0))
            .map(|dy| egui::Rect::from_center_size(egui::pos2(x, at.y + dy), size))
            .find(|frame| room.contains_rect(*frame) && !taken.iter().any(|t| t.intersects(*frame)));
        let Some(frame) = free else { continue };
        taken.push(frame);
        if !frame.contains(at) {
            // A label moved off its point is tied to it by a thin line.
            painter.line_segment([at, frame.clamp(at)], egui::Stroke::new(1.0, ui::dim()));
            painter.circle_filled(at, 2.0, ui::dim());
        }
        painter.rect_filled(frame, 3.0, egui::Color32::from_rgba_unmultiplied(36, 40, 45, 200));
        painter.galley(frame.min + egui::vec2(5.0, 3.0), galley, egui::Color32::WHITE);
    }

    for action in actions {
        let model = &mut state.model;
        match action {
            Action::Machine(entry) => {
                model.set_machine(entry);
                // Forget the drawn parts, so the camera frames the new machine; its scenes start anew, live.
                state.part_ids.clear();
                forget_run(state);
                state.panels.part = None;
                go_live(state);
                // Paused, no sample arrives to show that time started again: drop the old run's scenes here.
                state.recorded.clear();
            }
            Action::Language(lang) => {
                model.set_language(lang);
                window.title = model.tr.tr("ui-window-title").into();
            }
            Action::Pause(paused) => {
                // While scrubbing the simulation is held for the timeline: Pause or Resume means the live run.
                if state.timeline.time.is_some() {
                    go_live(state);
                }
                state.model.set_paused(paused);
            }
            Action::Speed(speed) => model.set_speed(speed),
            Action::View(yaw, pitch) => {
                (state.orbit.yaw, state.orbit.pitch) = (yaw.to_radians(), pitch.to_radians());
                state.fit = true;
            }
            Action::Overlays(overlays) => state.overlays = overlays,
            Action::Record(on) => set_recording(state, on),
            Action::Timeline(timeline) => set_timeline(state, timeline),
            Action::Step(on) => {
                let Some((last, _)) = state.recorded.back() else { continue };
                // From live, a step back is to the scene before the newest.
                let now = state.timeline.time.unwrap_or(*last);
                let mut times = state.recorded.iter().map(|(t, _)| *t);
                let to = if on { times.find(|t| *t > now) } else { times.rfind(|t| *t < now) };
                if let Some(to) = to {
                    let timeline = ui::Timeline { time: Some(to), playing: false, ..state.timeline };
                    set_timeline(state, timeline);
                }
            }
            Action::Panels(panels) => {
                // Another theme is put in place next frame.
                state.themed &= panels.light == state.panels.light;
                state.panels = panels;
            }
            Action::Exaggeration(factor) => model.set_exaggeration(factor),
            Action::Compare(on, effect) => {
                state.ghost = None;
                state.compare_effect = effect.clone();
                if on {
                    start_ghost(state, effect);
                }
            }
            Action::Target(axis, value) => model.target[axis] = value,
            Action::Command(c) => command(state, c),
            Action::Param(path, value) => model.set_param(&path, value),
            Action::Scope(scope, visible) => model.scopes[scope].visible = visible,
            Action::Window(seconds) => model.window_s = seconds,
            Action::Freeze(frozen) => model.set_frozen(frozen),
            Action::Export => {
                // Named after the date and time, so that successive exports do not overwrite each other.
                if let Some(path) = save_path("CSV", "csv", &format!("traces-{}.csv", video::now_stamp())) {
                    model.export_traces(&path);
                }
            }
            Action::ExportOverlay => {
                if let Some(path) = save_path("CSV", "csv", &format!("replay-overlay-{}.csv", video::now_stamp())) {
                    model.export_overlay(&path);
                }
            }
            Action::SavePlot(png, plot, rect) => {
                let (kind, extension) = if png { ("PNG", "png") } else { ("SVG", "svg") };
                let Some(path) = save_path(kind, extension, &format!("plot-{}.{extension}", video::now_stamp())) else {
                    continue;
                };
                if png {
                    let scale = window.scale_factor();
                    let area = [rect.left(), rect.top(), rect.width(), rect.height()]
                        .map(|v| (v * scale).round().max(0.0) as u32);
                    let saver = video::picture_saver(area, path, state.notes.clone());
                    commands.spawn(Screenshot::primary_window()).observe(saver);
                } else {
                    let message = match std::fs::write(&path, plot.svg()) {
                        Ok(()) => ("ui-plot-saved", path.display().to_string()),
                        Err(e) => ("ui-export-failed", format!("{}: {e}", path.display())),
                    };
                    model.messages.push(message);
                }
            }
            Action::Units(units) => model.units = units,
            Action::KeepLive => model.keep_live(),
            Action::KeepGhost => keep_ghost(state),
            Action::RecordRun => {
                let (set, file) = (model.inputs.run_set.clone(), model.inputs.run_file.clone());
                model.record_run(&set, &file);
            }
            Action::RunShown(i, shown) => model.runs.set_shown(i, shown),
            Action::RunReference(i) => model.runs.set_reference(i),
            Action::RemoveRun(i) => model.runs.remove(i),
            Action::ExportReport => {
                // The dialog holds up the window until it is closed, as the config's does.
                let picked = rfd::FileDialog::new()
                    .add_filter("PDF", &["pdf"])
                    .add_filter("HTML", &["html"])
                    .set_file_name("sizing-report.pdf")
                    .save_file();
                if let Some(path) = picked {
                    model.export_report(&path.display().to_string());
                }
            }
            Action::TuningAxis(axis) => model.tuning_axis = axis,
            Action::RunTuning => model.run_tuning(model.tuning_axis),
            Action::CloseTuning => model.tuning = None,
            Action::Inputs(inputs) => model.inputs = inputs,
            Action::RunAnalysis(kind) => model.run_analysis(kind),
            Action::ApplyOptimised => model.apply_optimised(),
            Action::UseCandidate(row) => model.use_candidate(row),
            Action::RunView(kind) => model.views.start(kind, model.entry, &model.config, &model.inputs),
            Action::OpenConfig(dialog) => {
                if let Some(path) = config_path(model, dialog, false) {
                    model.open_config(&path);
                    // Forget the drawn parts, so the camera frames the machine as the file sets it up (another machine,
                    // where the file is of one).
                    state.part_ids.clear();
                    state.panels.part = None;
                    forget_run(state);
                    go_live(state);
                    state.recorded.clear();
                }
            }
            Action::SaveConfig(dialog) => {
                if let Some(path) = config_path(model, dialog, true) {
                    model.save_config(&path);
                }
            }
            Action::ApplyParts => model.apply_parts(),
            Action::SetValue => {
                let assignment = model.inputs.assignment.clone();
                model.set_value(&assignment);
            }
            Action::CloseAnalysis => model.clear_analysis(),
            Action::Clear(kind) => model.clear(kind),
            Action::ClearMessages => model.messages.clear(),
        }
    }
    Ok(())
}

/// Moves the timeline, within what was recorded. Leaving the newest scene pauses the simulation, so that the
/// scopes' history stays where it was scrubbed; going live again pauses or runs it as it was before.
fn set_timeline(state: &mut State, mut timeline: ui::Timeline) {
    timeline.time = match (state.recorded.front(), state.recorded.back()) {
        (Some((first, _)), Some((last, _))) => timeline.time.map(|t| t.clamp(*first, *last)),
        _ => None,
    };
    match (state.timeline.time, timeline.time) {
        (None, Some(_)) => {
            state.was_paused = state.model.paused;
            state.model.set_paused(true);
        }
        (Some(_), None) => state.model.set_paused(state.was_paused),
        _ => {}
    }
    state.timeline = timeline;
}

/// The machine was started anew: its session knows nothing of the mode or of a path, and a run compared with the
/// one before says nothing about it.
fn forget_run(state: &mut State) {
    (state.mode_sent, state.ghost) = (false, None);
    state.trail_points.clear();
}

/// Starts recording the view, or stops and has the video made.
fn set_recording(state: &mut State, on: bool) {
    match (on, state.recording.take()) {
        (true, None) => match video::Recording::start() {
            Ok(recording) => state.recording = Some(recording),
            Err(message) => state.model.messages.push(message),
        },
        (false, Some(recording)) => state.encodings.push(recording.stop()),
        (_, kept) => state.recording = kept,
    }
}

/// While recording, a screenshot of the window for each frame due, cut to the 3D view; and the messages of videos
/// made.
fn record(
    mut state: ResMut<State>,
    mut commands: Commands,
    window: Single<&Window, With<PrimaryWindow>>,
    clear: Res<ClearColor>,
) {
    let state = &mut *state;
    if let Some(recording) = &mut state.recording {
        let due = recording.due();
        let scale = window.scale_factor();
        let [left, top, right, bottom] = state.view_rect.map(|v| (v * scale).round() as u32);
        let view = [left, top, right.saturating_sub(left), bottom.saturating_sub(top)];
        // Not before the panels have left the view room: the first frame fixes the video's size.
        if !due.is_empty() && view[2] >= 2 && view[3] >= 2 {
            commands.spawn(Screenshot::primary_window()).observe(recording.saver(
                view,
                clear.0.to_srgba().to_u8_array(),
                due,
            ));
        }
    }
    let mut done = Vec::new();
    if let Ok(mut notes) = state.notes.lock() {
        done.append(&mut notes);
    }
    state.encodings.retain_mut(|encoding| encoding.poll().map(|messages| done.extend(messages)).is_none());
    state.model.messages.extend(done);
}

/// At the app's exit, however it comes (the window closed, the screenshot hook): a recording is stopped, its frames
/// still queued are written and the videos still being made are waited for a few seconds (`Encoding::finish`).
/// Their messages go to the terminal, since the window is gone.
fn finish_videos(mut exits: MessageReader<AppExit>, mut state: ResMut<State>) {
    if exits.read().next().is_none() {
        return;
    }
    set_recording(&mut state, false);
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
    for encoding in std::mem::take(&mut state.encodings) {
        for (key, arg) in encoding.finish(deadline) {
            eprintln!("{}", state.model.tr.tr_arg(key, &arg));
        }
    }
}

/// Back to the newest scene, as the "Live" button does.
fn go_live(state: &mut State) {
    let live = ui::Timeline { time: None, playing: false, ..state.timeline };
    set_timeline(state, live);
}

/// Keeps the ghost run's window (as long as the scopes') as a run.
fn keep_ghost(state: &mut State) {
    let (Some(ghost), model) = (&state.ghost, &mut state.model) else { return };
    let rows = &ghost.history;
    let from = rows.first_at(rows.back().map_or(0.0, |last| last[0] - model.window_s));
    let effect = ghost.effect.strip_prefix("effects.").unwrap_or(&ghost.effect);
    let effect = model.tr.tr(&format!("effect-{effect}")).to_string();
    let switched = model.tr.tr_arg(if ghost.on { "ui-compare-ghost-on" } else { "ui-compare-ghost-off" }, &effect);
    let label = format!("{} · {switched}", model.tr.tr(RunKind::Ghost.key()));
    let (signals, motors) = (ghost.session.signals.clone(), &ghost.session.motors);
    let run = Run::from_rows(label, RunKind::Ghost, model.entry.id, signals, motors, rows.iter().skip(from));
    let run = model.rated(run);
    model.runs.add(run);
}

/// A file to save to, from the system's dialog, starting with `name`; `None` where it is cancelled. The dialog holds
/// up the window until it is closed, as the config's does.
fn save_path(kind: &str, extension: &str, name: &str) -> Option<std::path::PathBuf> {
    rfd::FileDialog::new().add_filter(kind, &[extension]).set_file_name(name).save_file()
}

/// The file the config file field names, or with `dialog` the one the system's file dialog is given (starting
/// where the field points); `None` where it is cancelled. The dialog holds up the window until it is closed.
fn config_path(model: &Model, dialog: bool, save: bool) -> Option<String> {
    let typed = std::path::Path::new(&model.inputs.config_file);
    if !dialog {
        return Some(model.inputs.config_file.clone());
    }
    let mut files = rfd::FileDialog::new().add_filter("toml", &["toml"]);
    // Machine files may be JSON (docs/machine-files.md); a config is saved as TOML.
    if !save {
        files = files.add_filter("json", &["json"]);
    }
    if let Some(folder) = typed.parent().filter(|folder| folder.is_dir()) {
        files = files.set_directory(folder);
    }
    if let Some(name) = typed.file_name() {
        files = files.set_file_name(name.to_string_lossy());
    }
    let picked = if save { files.save_file() } else { files.pick_file() };
    picked.map(|path| path.display().to_string())
}

/// For checking the window without a person in front of it: with SIM_APP_SCREENSHOT=path set, the app
/// starts the cycle, measures the tuning of the first axis, runs the three analyses (the cycle with its
/// error bands), saves one screenshot a second after they are done (after nine seconds at the earliest) and
/// exits a second later. To get another part of the panel into the picture, one of SIM_APP_SEARCH (how the
/// article numbers start), SIM_APP_PATH and SIM_APP_TRACE (a file each) runs that part in their place, and
/// SIM_APP_SWEEP (a measured frequency response, or nothing) runs the frequency response from 50 to 130 Hz
/// with 1.5 N m and, with a file, its fit. SIM_APP_RATIO (anything) runs the search tab's ratio sweep,
/// SIM_APP_KINEMATICS (a task file) its kinematics comparison, and SIM_APP_MAP (anything) the path tab's move maps. SIM_APP_POINTS runs the cycle alone, for its operating points.
/// SIM_APP_EFFECTS starts the cycle and analyses it, for the effect views; SIM_APP_COMPARE (an effect's name in
/// `[effects]`) runs the effect comparison with it as well. SIM_APP_CONFIG (a file) opens that config
/// file first (a config of another machine switches to it); SIM_APP_MACHINE (a machine's id) switches to that
/// machine before. SIM_APP_SCROLL (pixels) scrolls the right panel down. SIM_APP_OVERLAYS ("xray,forces,planned", any
/// of them, and "load"; "mode" for the mode, "toolview" for the tool view, "compare-<effect>" (its name in
/// `[effects]`) for the run with that effect switched, "trail-speed" or "trail-error" for the travelled path's colour
/// and "tube" for the deviation tube) switches on what the 3D view draws besides the parts, SIM_APP_EXAGGERATION (a factor) draws
/// the deflections larger, and SIM_APP_VIEW ("yaw,pitch" in degrees) turns the camera and frames the machine.
/// SIM_APP_CHECK (a check's name, e.g. "motor-peak") opens that check in the tools panel; SIM_APP_TAB (cycle, effects,
/// search, path, trace, sweep, tuning, runs (with the runs in the bottom panel) or views) opens that tab of it,
/// SIM_APP_THEME=light shows the light theme, and SIM_APP_WORKSPACE_Z (in the third axis's unit) sets
/// the height of the workspace map in the path tab. SIM_APP_PART (a part's id,
/// e.g. "motor", or "motor,1" for the second of them) picks that part as a click would. SIM_APP_SCRUB (seconds)
/// scrubs the timeline that far back from the newest scene, just before the screenshot. SIM_APP_RECORD (seconds)
/// records the view for that long from the start, as the Record button does, and the app exits only once its video
/// is made (or its frames are left).
fn developer_hooks(
    mut state: ResMut<State>,
    mut commands: Commands,
    mut exit: MessageWriter<AppExit>,
    mut record_for: Local<Option<f64>>,
) {
    let Ok(path) = std::env::var("SIM_APP_SCREENSHOT") else { return };
    state.frame += 1;
    if let (Some(limit), Some(recording)) = (*record_for, &state.recording)
        && recording.seconds() >= limit
    {
        set_recording(&mut state, false);
    }
    match state.frame {
        30 => {
            let asked = |name: &str| std::env::var(name).ok();
            *record_for = asked("SIM_APP_RECORD").and_then(|s| s.parse().ok());
            if record_for.is_some() {
                set_recording(&mut state, true);
            }
            if let Some(entry) = asked("SIM_APP_MACHINE").and_then(|id| MACHINES.iter().copied().find(|m| m.id == id)) {
                state.model.set_machine(entry);
                state.part_ids.clear();
            }
            if let Some(file) = asked("SIM_APP_CONFIG") {
                state.model.open_config(&file);
                state.part_ids.clear();
            }
            state.scroll = asked("SIM_APP_SCROLL").and_then(|pixels| pixels.parse().ok());
            // The 3D view's switches, as words ("xray,forces,planned"), a deflection factor and a view preset.
            let overlays = asked("SIM_APP_OVERLAYS").unwrap_or_default();
            let words: Vec<&str> = overlays.split(',').collect();
            let has = |word: &str| words.contains(&word);
            let trail = if has("trail-speed") {
                ui::TrailColor::Speed
            } else if has("trail-error") {
                ui::TrailColor::FollowingError
            } else {
                ui::TrailColor::Plain
            };
            state.overlays = ui::Overlays {
                xray: has("xray"),
                forces: has("forces"),
                planned: has("planned"),
                load: has("load"),
                mode: has("mode"),
                trail,
                tube: has("tube"),
                tool_view: has("toolview"),
                ..Default::default()
            };
            if let Some(effect) = words.iter().find_map(|w| w.strip_prefix("compare-")) {
                start_ghost(&mut state, format!("effects.{effect}"));
            }
            if let Some(factor) = asked("SIM_APP_EXAGGERATION").and_then(|f| f.parse().ok()) {
                state.model.set_exaggeration(factor);
            }
            // A check to open in the tools panel, as the top bar's gauge would (its name, e.g. "motor-peak").
            if let Some(check) = asked("SIM_APP_CHECK") {
                state.panels = ui::PanelState { tools: ui::ToolTab::Cycle, check: Some(check.leak()), ..state.panels };
            }
            let tabs = [
                ("cycle", ui::ToolTab::Cycle),
                ("effects", ui::ToolTab::EffectViews),
                ("search", ui::ToolTab::Search),
                ("path", ui::ToolTab::Path),
                ("trace", ui::ToolTab::Trace),
                ("sweep", ui::ToolTab::Sweep),
                ("tuning", ui::ToolTab::Tuning),
                ("runs", ui::ToolTab::Runs),
                ("views", ui::ToolTab::Views),
            ];
            if let Some((_, tab)) =
                asked("SIM_APP_TAB").and_then(|name| tabs.into_iter().find(|(word, _)| *word == name))
            {
                // The runs tab with the runs in the bottom panel.
                (state.panels.tools, state.panels.runs_view) = (tab, tab == ui::ToolTab::Runs);
            }
            state.panels.light = asked("SIM_APP_THEME").is_some_and(|theme| theme == "light");
            state.themed = false;
            if let Some(height) = asked("SIM_APP_WORKSPACE_Z").and_then(|z| z.parse().ok()) {
                state.model.inputs.workspace_z = Some(height);
            }
            if let Some(part) = asked("SIM_APP_PART") {
                let (id, nth) =
                    part.split_once(',').map_or((part.as_str(), 0), |(id, nth)| (id, nth.parse().unwrap_or(0)));
                state.panels.part = Some((id.to_string().leak(), nth));
            }
            if let Some([yaw, pitch]) = asked("SIM_APP_VIEW").and_then(|v| {
                v.split_once(',').and_then(|(a, b)| Some([a.parse::<f32>().ok()?, b.parse::<f32>().ok()?]))
            }) {
                (state.orbit.yaw, state.orbit.pitch) = (yaw.to_radians(), pitch.to_radians());
                state.fit = true;
            }
            // The parameter panel with "path = value" typed in its field and applied.
            if let Some(assignment) = asked("SIM_APP_SET") {
                state.panels.setup = ui::SetupTab::Parameters;
                state.model.set_value(&assignment);
                state.model.inputs.assignment = assignment;
            }
            let model = &mut state.model;
            let inputs = &mut model.inputs;
            let parts = [
                (asked("SIM_APP_SEARCH"), &mut inputs.series, Analysis::Search),
                (asked("SIM_APP_PATH"), &mut inputs.path_file, Analysis::Path),
                (asked("SIM_APP_TRACE"), &mut inputs.trace_file, Analysis::Trace),
            ];
            let mut kinds = Vec::new();
            for (value, field, kind) in parts {
                if let Some(value) = value {
                    *field = value;
                    kinds.push(kind);
                }
            }
            if let Some(file) = asked("SIM_APP_SWEEP") {
                (inputs.sweep_from, inputs.sweep_to, inputs.sweep_count, inputs.sweep_amplitude) =
                    (50.0, 130.0, 30, 1.5);
                kinds.push(Analysis::Sweep);
                if !file.is_empty() {
                    inputs.sweep_file = file;
                    kinds.push(Analysis::SweepFit);
                }
            }
            if asked("SIM_APP_POINTS").is_some() {
                kinds.push(Analysis::Cycle);
            }
            // The search tab's ratio sweep and kinematics comparison (of a task file), the path tab's move maps.
            use sim_app::search_views::ViewJob;
            let mut views = Vec::new();
            if asked("SIM_APP_RATIO").is_some() {
                views.push(ViewJob::Sweep);
            }
            if let Some(file) = asked("SIM_APP_KINEMATICS") {
                model.inputs.search.task_file = file;
                views.push(ViewJob::Kinematics);
            }
            if asked("SIM_APP_MAP").is_some() {
                views.push(ViewJob::Map);
            }
            for kind in views.iter().copied() {
                model.views.start(kind, model.entry, &model.config, &model.inputs);
            }
            // The effect views: their live parts need the machine moving.
            let mut cycle = false;
            if asked("SIM_APP_EFFECTS").is_some() {
                model.command(sim_components::Command::StartCycle);
                cycle = true;
                kinds.push(Analysis::Cycle);
            }
            if let Some(effect) = asked("SIM_APP_COMPARE") {
                model.inputs.compare_effect = effect;
                kinds.push(Analysis::Compare);
            }
            if kinds.is_empty() && views.is_empty() {
                model.command(sim_components::Command::StartCycle);
                cycle = true;
                model.run_tuning(0);
                model.inputs.bands = true;
                kinds = vec![Analysis::Cycle, Analysis::Sensitivity, Analysis::Optimise];
            }
            kinds.into_iter().for_each(|kind| model.run_analysis(kind));
            // The compared run starts the cycle with the machine.
            if let (Some(ghost), true) = (&state.ghost, cycle) {
                ghost.session.send(Request::Command(Command::StartCycle));
            }
        }
        // Wait here until the background jobs have answered. The second after that is for the tables' new
        // letters: they reach the picture a frame after they are first drawn.
        480 if state.model.tuning_running() || state.model.analysis_running() => state.frame -= 1,
        500 => {
            if let Some(back) = std::env::var("SIM_APP_SCRUB").ok().and_then(|s| s.parse::<f64>().ok())
                && let Some((last, _)) = state.recorded.back()
            {
                let timeline = ui::Timeline { time: Some(last - back), playing: false, ..state.timeline };
                set_timeline(&mut state, timeline);
            }
        }
        540 => {
            commands.spawn(Screenshot::primary_window()).observe(save_to_disk(path));
        }
        599 if state.recording.is_some() || !state.encodings.is_empty() => state.frame -= 1,
        600 => {
            exit.write(AppExit::Success);
        }
        _ => {}
    }
}
