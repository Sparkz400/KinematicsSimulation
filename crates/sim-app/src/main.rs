//! The app window: a 3D view built from the machine's scene description, next to the panels.
//! It knows no machine; it draws whatever parts the selected machine describes.

mod ui;

use bevy::anti_alias::smaa::Smaa;
use bevy::asset::io::file::FileAssetReader;
use bevy::gltf::GltfAssetLabel;
use bevy::world_serialization::WorldAssetRoot;
use bevy::camera::visibility::RenderLayers;
use bevy::camera::{CameraOutputMode, Hdr, Viewport};
use bevy::light::CascadeShadowConfigBuilder;
use bevy::pbr::ScreenSpaceAmbientOcclusion;
use bevy::post_process::bloom::Bloom;
use bevy::render::view::Tonemapping;
use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll};
use bevy::prelude::*;
use bevy::render::render_resource::BlendState;
use bevy::render::view::screenshot::{Screenshot, save_to_disk};
use bevy::window::PrimaryWindow;
use bevy_egui::{EguiContext, EguiContexts, EguiGlobalSettings, EguiPlugin, EguiPrimaryContextPass, PrimaryEguiContext, egui};
use sim_app::model::{Analysis, Model};
use sim_core::{Label, Part, Shape};
use sim_machines::{LANGUAGES, MACHINES};
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
    /// The floor under the machine, placed when the machine's first scene arrives.
    floor: Option<Entity>,
    /// The config file the drawn parts were made for, which decides which CAD meshes they use (`cad_mesh`).
    mesh_config: String,
    /// The 3D view's rectangle in logical pixels: (left, top, right, bottom).
    view_rect: [f32; 4],
    frame: u32,
    /// Where the screenshot hook scrolls the right panel to, in pixels from its top.
    scroll: Option<f32>,
}

#[derive(Resource)]
struct Shapes {
    cube: Handle<Mesh>,
    cylinder: Handle<Mesh>,
}

#[derive(Component)]
struct WorldCamera;

fn main() {
    let lang = std::env::var("SIM_APP_LANG").ok().and_then(|l| LANGUAGES.iter().copied().find(|k| *k == l)).unwrap_or(LANGUAGES[0]);
    let model = Model::new(MACHINES[0], lang);
    let window = Window { title: model.tr.tr("ui-window-title").into(), resolution: (1600u32, 900u32).into(), ..default() };
    App::new()
        .insert_resource(ClearColor(Color::srgb(0.13, 0.14, 0.16)))
        .insert_resource(State {
            model,
            orbit: Orbit { yaw: 0.35, pitch: 0.2, distance: 2.5, target: Vec3::ZERO },
            parts: Vec::new(),
            floor: None,
            mesh_config: String::new(),
            part_ids: Vec::new(),
            view_rect: [0.0; 4],
            frame: 0,
            scroll: None,
        })
        .add_plugins(DefaultPlugins.set(WindowPlugin { primary_window: Some(window), ..default() }))
        .add_plugins(EguiPlugin::default())
        .add_systems(Startup, setup)
        .add_systems(Update, (sync_scene, paint_cad_parts, draw_paths, orbit_camera, developer_hooks).chain())
        .add_systems(EguiPrimaryContextPass, panels)
        .run();
}

fn setup(mut commands: Commands, mut egui_settings: ResMut<EguiGlobalSettings>, mut meshes: ResMut<Assets<Mesh>>, mut images: ResMut<Assets<Image>>) {
    // The panels get their own camera on top, so the 3D camera can be confined to the free area.
    egui_settings.auto_create_primary_context = false;
    commands.insert_resource(Shapes {
        cube: meshes.add(Cuboid::new(1.0, 1.0, 1.0)),
        cylinder: meshes.add(Cylinder::new(0.5, 1.0)),
    });
    // A workshop look: soft sky light from all sides for the metal to reflect, contact shadows (SSAO, which
    // needs MSAA off, so SMAA smooths the edges instead), film-like tone mapping and a faint glow on highlights.
    let sky = EnvironmentMapLight::hemispherical_gradient(&mut images, Color::srgb(0.75, 0.8, 0.9), Color::srgb(0.45, 0.45, 0.47), Color::srgb(0.12, 0.12, 0.13));
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
        sky.with_intensity(1500.0),
    ));
    commands.spawn((
        PrimaryEguiContext,
        Camera2d,
        RenderLayers::none(),
        Camera {
            order: 1,
            output_mode: CameraOutputMode::Write { blend_state: Some(BlendState::ALPHA_BLENDING), clear_color: ClearColorConfig::None },
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
    commands.spawn((DirectionalLight { illuminance: 1500.0, ..default() }, Transform::from_xyz(-3.0, 2.0, -4.0).looking_at(Vec3::ZERO, Vec3::Y)));
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

/// A CAD mesh's own material, kept while the part is painted in its scene colour.
#[derive(Component)]
struct VendorMaterial(Handle<StandardMaterial>);

/// Keeps the vendor's colours on a CAD part, except where it has none (a STEP file through FreeCAD without
/// its window: the glTF's default material) or while the heat or belt view colours it: then the part's own
/// material paints it. Runs every frame, since a glTF's meshes arrive some frames after the part is spawned.
fn paint_cad_parts(
    state: Res<State>,
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    cad_parts: Query<(Entity, &CadPart)>,
    children: Query<&Children>,
    mut meshes: Query<(&mut MeshMaterial3d<StandardMaterial>, Option<&VendorMaterial>)>,
) {
    let colourless = GltfAssetLabel::DefaultMaterial.to_string();
    for (root, CadPart(index)) in &cad_parts {
        let (Some(part), Some((_, ours))) = (state.model.scene.parts.get(*index), state.parts.get(*index)) else { continue };
        let coloured = part.heat.is_some() || part.highlight != 0.0;
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

/// The planned path as a line in the 3D view, and over it the path the tool has really travelled since.
fn draw_paths(state: Res<State>, mut gizmos: Gizmos) {
    if let Some(run) = &state.model.path_run {
        gizmos.linestrip(run.scene_line.iter().map(|p| to_bevy(*p)), Color::srgb(1.0, 0.8, 0.1));
    }
    if let Some(trail) = &state.model.trail {
        gizmos.linestrip(trail.iter().map(|p| to_bevy(*p)), Color::srgb(0.2, 0.9, 1.0));
    }
}

/// Takes in new samples and moves the drawn parts to where the machine says they are.
fn sync_scene(
    mut state: ResMut<State>,
    mut commands: Commands,
    shapes: Res<Shapes>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    asset_server: Res<AssetServer>,
    mut transforms: Query<(&mut Transform, Has<CadPart>), Without<WorldCamera>>,
) {
    let state = &mut *state;
    state.model.poll();
    let scene = &state.model.scene;
    let ids: Vec<&'static str> = scene.parts.iter().map(|p| p.id).collect();
    // Nothing is drawn before the panels have been laid out once: the camera is fitted to the view they leave.
    if state.view_rect[2] <= state.view_rect[0] {
        return;
    }
    if ids != state.part_ids || state.model.inputs.config_file != state.mesh_config {
        // Another machine or config file, or a part came or went (the payload): rebuild the drawn parts.
        // The CAD parts stay, matched again by id: the belt reorders its parts nearly every frame, and a CAD
        // mesh spawned anew takes some frames to load.
        let same_config = state.model.inputs.config_file == state.mesh_config;
        let mut cad_kept = Vec::new();
        for (old_index, (entity, material)) in state.parts.drain(..).enumerate() {
            if same_config && transforms.get(entity).is_ok_and(|(_, cad)| cad) {
                cad_kept.push((state.part_ids[old_index], entity, material));
            } else {
                commands.entity(entity).despawn();
            }
        }
        state.mesh_config = state.model.inputs.config_file.clone();
        for (index, part) in scene.parts.iter().enumerate() {
            if let Some(kept) = cad_kept.iter().position(|(id, ..)| *id == part.id) {
                let (_, entity, material) = cad_kept.remove(kept);
                commands.entity(entity).insert(CadPart(index));
                state.parts.push((entity, material));
                continue;
            }
            // Turned steel for round parts, matt rubber for the belt, satin anodised or painted for the rest.
            let (metallic, perceptual_roughness) = match part.shape {
                Shape::Cylinder { .. } => (0.9, 0.3),
                _ if part.id.starts_with("belt") => (0.0, 0.85),
                _ => (0.3, 0.45),
            };
            let material = materials.add(StandardMaterial { base_color: part_color(part), perceptual_roughness, metallic, ..default() });
            if let Some(file) = cad_mesh(&state.mesh_config, part.id) {
                let at = part_transform(part).with_scale(Vec3::ONE);
                let entity = commands.spawn((WorldAssetRoot(asset_server.load(GltfAssetLabel::Scene(0).from_asset(file))), at, CadPart(index))).id();
                state.parts.push((entity, material));
                continue;
            }
            let mesh = if matches!(part.shape, Shape::Cylinder { .. }) { shapes.cylinder.clone() } else { shapes.cube.clone() };
            let entity = commands.spawn((Mesh3d(mesh), MeshMaterial3d(material.clone()), part_transform(part))).id();
            state.parts.push((entity, material));
        }
        for (_, entity, _) in cad_kept {
            commands.entity(entity).despawn();
        }
        if state.part_ids.is_empty() && !scene.parts.is_empty() {
            // First scene of a machine: aim the camera at its middle and fit it in.
            let ends = scene.parts.iter().filter_map(|p| if let Shape::Segment { to, .. } = &p.shape { Some(to_bevy(*to)) } else { None });
            let points: Vec<Vec3> = scene.parts.iter().map(|p| to_bevy(p.pos)).chain(ends).collect();
            let (low, high) = points.iter().fold((points[0], points[0]), |(l, h), p| (l.min(*p), h.max(*p)));
            state.orbit.target = (low + high) / 2.0;
            // Far enough back for the machine's height and, in a narrow view, its width
            // (the camera's vertical field of view is 45 degrees).
            let [view_left, view_top, view_right, view_bottom] = state.view_rect;
            let aspect = if view_bottom > view_top && view_right > view_left { (view_right - view_left) / (view_bottom - view_top) } else { 1.0 };
            let size = high - low;
            let half_view = (size.y / 2.0).max(size.x / 2.0 / aspect) * 1.45 + 0.1;
            state.orbit.distance = half_view / (std::f32::consts::FRAC_PI_8).tan() + size.z / 2.0;            // A floor under the lowest part, for the shadows to fall on.
            if let Some(old) = state.floor.take() {
                commands.entity(old).despawn();
            }
            let lowest = scene.parts.iter().map(|p| part_transform(p)).map(|t| t.translation.y - t.scale.y.abs() / 2.0).fold(low.y, f32::min);
            let ground = materials.add(StandardMaterial { base_color: Color::srgb(0.22, 0.23, 0.25), perceptual_roughness: 0.9, ..default() });
            let place = Transform::from_xyz(state.orbit.target.x, lowest - 0.01, state.orbit.target.z).with_scale(Vec3::new(8.0, 0.02, 8.0));
            state.floor = Some(commands.spawn((Mesh3d(shapes.cube.clone()), MeshMaterial3d(ground), place)).id());
        }
        state.part_ids = ids;
        return;
    }
    for (part, (entity, material)) in scene.parts.iter().zip(&state.parts) {
        if let Ok((mut transform, cad)) = transforms.get_mut(*entity) {
            // A CAD mesh is drawn at its true size; only the boxes are stretched to the part.
            *transform = if cad { part_transform(part).with_scale(Vec3::ONE) } else { part_transform(part) };
        }
        // Only where the colour changed, so that the other materials are not sent to the graphics card again.
        let color = part_color(part);
        if materials.get(material).is_some_and(|m| m.base_color != color)
            && let Some(mut material) = materials.get_mut(material)
        {
            material.base_color = color;
        }
    }
}

/// Orbit with the left mouse button, zoom with the wheel, while the pointer is over the 3D view.
fn orbit_camera(
    mut state: ResMut<State>,
    mut camera: Single<&mut Transform, With<WorldCamera>>,
    window: Single<&Window, With<PrimaryWindow>>,
    buttons: Res<ButtonInput<MouseButton>>,
    motion: Res<AccumulatedMouseMotion>,
    scroll: Res<AccumulatedMouseScroll>,
) {
    let [left, top, right, bottom] = state.view_rect;
    let over_view = window.cursor_position().is_some_and(|p| p.x > left && p.x < right && p.y > top && p.y < bottom);
    let orbit = &mut state.orbit;
    if over_view {
        if buttons.pressed(MouseButton::Left) {
            orbit.yaw -= motion.delta.x * 0.005;
            orbit.pitch = (orbit.pitch + motion.delta.y * 0.005).clamp(-1.55, 1.55);
        }
        orbit.distance = (orbit.distance * (1.0 - scroll.delta.y * 0.1)).clamp(0.2, 20.0);
    }
    let rotation = Quat::from_euler(EulerRot::YXZ, orbit.yaw, -orbit.pitch, 0.0);
    camera.translation = orbit.target + rotation * Vec3::new(0.0, 0.0, orbit.distance);
    camera.rotation = rotation;
}

fn panels(
    mut contexts: EguiContexts,
    mut state: ResMut<State>,
    camera: Single<(&mut Camera, &GlobalTransform), Without<EguiContext>>,
    mut window: Single<&mut Window, With<PrimaryWindow>>,
) -> Result {
    let (mut camera, camera_transform) = camera.into_inner();
    let ctx = contexts.ctx_mut()?;
    let state = &mut *state;
    let mut actions = Vec::new();
    let mut root = egui::Ui::new(ctx.clone(), "root".into(), egui::UiBuilder::new().layer_id(egui::LayerId::background()).max_rect(ctx.viewport_rect()));
    let left = egui::Panel::left("controls").resizable(true).default_size(320.0).show(&mut root, |ui| ui::controls(ui, &state.model, &mut actions)).response.rect;
    let right = egui::Panel::right("scopes").resizable(true).default_size(720.0).show(&mut root, |ui| ui::scopes(ui, &state.model, &mut actions, state.scroll)).response.rect;

    // The 3D camera draws into what the panels leave free.
    let scale = window.scale_factor();
    let position = UVec2::new((left.width() * scale) as u32, 0);
    let free_width = window.physical_width().saturating_sub(((left.width() + right.width()) * scale) as u32);
    if free_width > 0 && window.physical_height() > 0 {
        camera.viewport = Some(Viewport { physical_position: position, physical_size: UVec2::new(free_width, window.physical_height()), ..default() });
    }
    state.view_rect = [left.width(), 0.0, window.width() - right.width(), window.height()];

    // Live values next to the parts they belong to, e.g. each motor's speed.
    let painter = ctx.layer_painter(egui::LayerId::background());
    for label in &state.model.scene.labels {
        let Ok(at) = camera.world_to_viewport(camera_transform, to_bevy(label.pos)) else { continue };
        if at.x < state.view_rect[0] + 40.0 || at.x > state.view_rect[2] - 40.0 {
            continue;
        }
        let text = label_text(&state.model, label);
        let (pos, font) = (egui::pos2(at.x, at.y), egui::FontId::proportional(14.0));
        let galley = painter.layout_no_wrap(text, font, egui::Color32::WHITE);
        let frame = egui::Rect::from_center_size(pos, galley.size() + egui::vec2(10.0, 6.0));
        painter.rect_filled(frame, 4.0, egui::Color32::from_black_alpha(170));
        painter.galley(frame.min + egui::vec2(5.0, 3.0), galley, egui::Color32::WHITE);
    }

    for action in actions {
        let model = &mut state.model;
        match action {
            Action::Machine(entry) => {
                model.set_machine(entry);
                // Forget the drawn parts, so the camera frames the new machine.
                state.part_ids.clear();
            }
            Action::Language(lang) => {
                model.set_language(lang);
                window.title = model.tr.tr("ui-window-title").into();
            }
            Action::Pause(paused) => model.set_paused(paused),
            Action::Speed(speed) => model.set_speed(speed),
            Action::View(yaw, pitch) => (state.orbit.yaw, state.orbit.pitch) = (yaw.to_radians(), pitch.to_radians()),
            Action::Target(axis, value) => model.target[axis] = value,
            Action::Command(command) => model.command(command),
            Action::Param(path, value) => model.set_param(&path, value),
            Action::Scope(scope, visible) => model.scopes[scope].visible = visible,
            Action::Window(seconds) => model.window_s = seconds,
            Action::Freeze(frozen) => model.set_frozen(frozen),
            Action::Export => {
                // Named after the simulated time, so that successive exports do not overwrite each other.
                let time = model.visible_rows().last().map_or(0.0, |row| row[0]);
                model.export_traces(std::path::Path::new(&format!("traces-{time:.3}.csv")));
            }
            Action::ExportOverlay => model.export_overlay(std::path::Path::new("replay-overlay.csv")),
            Action::TuningAxis(axis) => model.tuning_axis = axis,
            Action::RunTuning => model.run_tuning(model.tuning_axis),
            Action::CloseTuning => model.tuning = None,
            Action::Inputs(inputs) => model.inputs = inputs,
            Action::RunAnalysis(kind) => model.run_analysis(kind),
            Action::ApplyOptimised => model.apply_optimised(),
            Action::UseCandidate(row) => model.use_candidate(row),
            Action::OpenConfig(dialog) => {
                if let Some(path) = config_path(model, dialog, false) {
                    model.open_config(&path);
                    // Forget the drawn parts, so the camera frames the machine as the file sets it up.
                    state.part_ids.clear();
                }
            }
            Action::SaveConfig(dialog) => {
                if let Some(path) = config_path(model, dialog, true) {
                    model.save_config(&path);
                }
            }
            Action::ApplyParts => model.apply_parts(),
            Action::CloseAnalysis => model.clear_analysis(),
            Action::Clear(kind) => model.clear(kind),
            Action::ClearMessages => model.messages.clear(),
        }
    }
    Ok(())
}

/// The file the config file field names, or with `dialog` the one the system's file dialog is given (starting
/// where the field points); `None` where it is cancelled. The dialog holds up the window until it is closed.
fn config_path(model: &Model, dialog: bool, save: bool) -> Option<String> {
    let typed = std::path::Path::new(&model.inputs.config_file);
    if !dialog {
        return Some(model.inputs.config_file.clone());
    }
    let mut files = rfd::FileDialog::new().add_filter("toml", &["toml"]);
    if let Some(folder) = typed.parent().filter(|folder| folder.is_dir()) {
        files = files.set_directory(folder);
    }
    if let Some(name) = typed.file_name() {
        files = files.set_file_name(name.to_string_lossy());
    }
    let picked = if save { files.save_file() } else { files.pick_file() };
    picked.map(|path| path.display().to_string())
}

/// "Motor A  1250 1/min"
fn label_text(model: &Model, label: &Label) -> String {
    format!("{}  {:.*} {}", model.tr.tr(label.owner_key), label.decimals, label.value, model.tr.tr(label.unit_key))
}

/// For checking the window without a person in front of it: with SIM_APP_SCREENSHOT=path set, the app
/// starts the cycle, measures the tuning of the first axis, runs the three analyses (the cycle with its
/// error bands), saves one screenshot a second after they are done (after nine seconds at the earliest) and
/// exits a second later. To get another part of the panel into the picture, one of SIM_APP_SEARCH (how the
/// article numbers start), SIM_APP_PATH and SIM_APP_TRACE (a file each) runs that part in their place, and
/// SIM_APP_SWEEP (a measured frequency response, or nothing) runs the frequency response from 50 to 130 Hz
/// with 1.5 N m and, with a file, its fit. SIM_APP_POINTS runs the cycle alone, for its operating points.
/// SIM_APP_EFFECTS starts the cycle and analyses it, for the effect views; SIM_APP_COMPARE (an effect's name in
/// `[effects]`) runs the effect comparison with it as well. SIM_APP_CONFIG (a file) opens that config
/// file first. SIM_APP_SCROLL (pixels) scrolls the right panel down.
fn developer_hooks(mut state: ResMut<State>, mut commands: Commands, mut exit: MessageWriter<AppExit>) {
    let Ok(path) = std::env::var("SIM_APP_SCREENSHOT") else { return };
    state.frame += 1;
    match state.frame {
        30 => {
            let asked = |name: &str| std::env::var(name).ok();
            if let Some(file) = asked("SIM_APP_CONFIG") {
                state.model.open_config(&file);
                state.part_ids.clear();
            }
            state.scroll = asked("SIM_APP_SCROLL").and_then(|pixels| pixels.parse().ok());
            let model = &mut state.model;
            let inputs = &mut model.inputs;
            let parts = [(asked("SIM_APP_SEARCH"), &mut inputs.series, Analysis::Search), (asked("SIM_APP_PATH"), &mut inputs.path_file, Analysis::Path), (asked("SIM_APP_TRACE"), &mut inputs.trace_file, Analysis::Trace)];
            let mut kinds = Vec::new();
            for (value, field, kind) in parts {
                if let Some(value) = value {
                    *field = value;
                    kinds.push(kind);
                }
            }
            if let Some(file) = asked("SIM_APP_SWEEP") {
                (inputs.sweep_from, inputs.sweep_to, inputs.sweep_count, inputs.sweep_amplitude) = (50.0, 130.0, 30, 1.5);
                kinds.push(Analysis::Sweep);
                if !file.is_empty() {
                    inputs.sweep_file = file;
                    kinds.push(Analysis::SweepFit);
                }
            }
            if asked("SIM_APP_POINTS").is_some() {
                kinds.push(Analysis::Cycle);
            }
            // The effect views: their live parts need the machine moving.
            if asked("SIM_APP_EFFECTS").is_some() {
                model.command(sim_components::Command::StartCycle);
                kinds.push(Analysis::Cycle);
            }
            if let Some(effect) = asked("SIM_APP_COMPARE") {
                model.inputs.compare_effect = effect;
                kinds.push(Analysis::Compare);
            }
            if kinds.is_empty() {
                model.command(sim_components::Command::StartCycle);
                model.run_tuning(0);
                model.inputs.bands = true;
                kinds = vec![Analysis::Cycle, Analysis::Sensitivity, Analysis::Optimise];
            }
            kinds.into_iter().for_each(|kind| model.run_analysis(kind));
        }
        // Wait here until the background jobs have answered. The second after that is for the tables' new
        // letters: they reach the picture a frame after they are first drawn.
        480 if state.model.tuning_running() || state.model.analysis_running() => state.frame -= 1,
        540 => {
            commands.spawn(Screenshot::primary_window()).observe(save_to_disk(path));
        }
        600 => {
            exit.write(AppExit::Success);
        }
        _ => {}
    }
}
