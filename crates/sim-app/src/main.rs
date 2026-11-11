//! The app window: a 3D view built from the machine's scene description, next to the panels.
//! It knows no machine; it draws whatever parts the selected machine describes.

mod ui;

use bevy::camera::visibility::RenderLayers;
use bevy::camera::{CameraOutputMode, Viewport};
use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll};
use bevy::prelude::*;
use bevy::render::render_resource::BlendState;
use bevy::render::view::screenshot::{Screenshot, save_to_disk};
use bevy::window::PrimaryWindow;
use bevy_egui::{EguiContext, EguiContexts, EguiGlobalSettings, EguiPlugin, EguiPrimaryContextPass, PrimaryEguiContext, egui};
use sim_app::model::Model;
use sim_core::{Part, Shape};
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
    /// The 3D view's rectangle in logical pixels: (left, top, right, bottom).
    view_rect: [f32; 4],
    frame: u32,
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
    let window = Window { title: model.tr.tr("ui-window-title").into(), ..default() };
    App::new()
        .insert_resource(ClearColor(Color::srgb(0.13, 0.14, 0.16)))
        .insert_resource(State {
            model,
            orbit: Orbit { yaw: 0.5, pitch: 0.35, distance: 2.5, target: Vec3::ZERO },
            parts: Vec::new(),
            part_ids: Vec::new(),
            view_rect: [0.0; 4],
            frame: 0,
        })
        .add_plugins(DefaultPlugins.set(WindowPlugin { primary_window: Some(window), ..default() }))
        .add_plugins(EguiPlugin::default())
        .add_systems(Startup, setup)
        .add_systems(Update, (sync_scene, orbit_camera, developer_hooks).chain())
        .add_systems(EguiPrimaryContextPass, panels)
        .run();
}

fn setup(mut commands: Commands, mut egui_settings: ResMut<EguiGlobalSettings>, mut meshes: ResMut<Assets<Mesh>>) {
    // The panels get their own camera on top, so the 3D camera can be confined to the free area.
    egui_settings.auto_create_primary_context = false;
    commands.insert_resource(Shapes {
        cube: meshes.add(Cuboid::new(1.0, 1.0, 1.0)),
        cylinder: meshes.add(Cylinder::new(0.5, 1.0)),
    });
    commands.spawn((WorldCamera, Camera3d::default(), Transform::default()));
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
    commands.spawn((DirectionalLight { illuminance: 8000.0, ..default() }, Transform::from_xyz(2.0, 4.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y)));
    commands.spawn((DirectionalLight { illuminance: 3000.0, ..default() }, Transform::from_xyz(-3.0, 2.0, -4.0).looking_at(Vec3::ZERO, Vec3::Y)));
}

/// Scene coordinates (x right, y into the depth, z up) to Bevy's (x right, y up, z towards the viewer).
fn to_bevy(p: [f64; 3]) -> Vec3 {
    Vec3::new(p[0] as f32, p[2] as f32, -p[1] as f32)
}

fn part_transform(part: &Part) -> Transform {
    let at = to_bevy(part.pos);
    match &part.shape {
        Shape::Box { size } => Transform::from_translation(at).with_scale(Vec3::new(size[0] as f32, size[2] as f32, size[1] as f32)),
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

/// Belt stretch made visible: tension above pretension shifts the colour to red, below to blue.
fn part_color(part: &Part) -> Color {
    let [r, g, b] = part.color;
    let h = part.highlight.clamp(-1.0, 1.0);
    let (toward, amount) = if h >= 0.0 { ([1.0, 0.1, 0.1], h) } else { ([0.1, 0.3, 1.0], -h) };
    let mix = |from: f32, to: f32| from + (to - from) * amount;
    Color::srgb(mix(r, toward[0]), mix(g, toward[1]), mix(b, toward[2]))
}

/// Takes in new samples and moves the drawn parts to where the machine says they are.
fn sync_scene(
    mut state: ResMut<State>,
    mut commands: Commands,
    shapes: Res<Shapes>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut transforms: Query<&mut Transform, Without<WorldCamera>>,
) {
    let state = &mut *state;
    state.model.poll();
    let scene = &state.model.scene;
    let ids: Vec<&'static str> = scene.parts.iter().map(|p| p.id).collect();
    if ids != state.part_ids {
        // Another machine, or a part came or went (the payload): rebuild the drawn parts.
        for (entity, _) in state.parts.drain(..) {
            commands.entity(entity).despawn();
        }
        for part in &scene.parts {
            let material = materials.add(StandardMaterial { base_color: part_color(part), perceptual_roughness: 0.6, ..default() });
            let mesh = if matches!(part.shape, Shape::Cylinder { .. }) { shapes.cylinder.clone() } else { shapes.cube.clone() };
            let entity = commands.spawn((Mesh3d(mesh), MeshMaterial3d(material.clone()), part_transform(part))).id();
            state.parts.push((entity, material));
        }
        if state.part_ids.is_empty() && !scene.parts.is_empty() {
            // First scene of a machine: aim the camera at its middle and fit it in.
            let ends = scene.parts.iter().filter_map(|p| if let Shape::Segment { to, .. } = &p.shape { Some(to_bevy(*to)) } else { None });
            let points: Vec<Vec3> = scene.parts.iter().map(|p| to_bevy(p.pos)).chain(ends).collect();
            let (low, high) = points.iter().fold((points[0], points[0]), |(l, h), p| (l.min(*p), h.max(*p)));
            state.orbit.target = (low + high) / 2.0;
            state.orbit.distance = (high - low).length().max(0.5) * 2.0;
        }
        state.part_ids = ids;
        return;
    }
    for (part, (entity, material)) in scene.parts.iter().zip(&state.parts) {
        if let Ok(mut transform) = transforms.get_mut(*entity) {
            *transform = part_transform(part);
        }
        if part.highlight != 0.0
            && let Some(mut material) = materials.get_mut(material)
        {
            material.base_color = part_color(part);
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
    mut camera: Single<&mut Camera, Without<EguiContext>>,
    mut window: Single<&mut Window, With<PrimaryWindow>>,
) -> Result {
    let ctx = contexts.ctx_mut()?;
    let state = &mut *state;
    let mut actions = Vec::new();
    let mut root = egui::Ui::new(ctx.clone(), "root".into(), egui::UiBuilder::new().layer_id(egui::LayerId::background()).max_rect(ctx.viewport_rect()));
    let left = egui::Panel::left("controls").resizable(true).default_size(320.0).show(&mut root, |ui| ui::controls(ui, &state.model, &mut actions)).response.rect;
    let right = egui::Panel::right("scopes").resizable(true).default_size(460.0).show(&mut root, |ui| ui::scopes(ui, &state.model, &mut actions)).response.rect;

    // The 3D camera draws into what the panels leave free.
    let scale = window.scale_factor();
    let position = UVec2::new((left.width() * scale) as u32, 0);
    let free_width = window.physical_width().saturating_sub(((left.width() + right.width()) * scale) as u32);
    if free_width > 0 && window.physical_height() > 0 {
        camera.viewport = Some(Viewport { physical_position: position, physical_size: UVec2::new(free_width, window.physical_height()), ..default() });
    }
    state.view_rect = [left.width(), 0.0, window.width() - right.width(), window.height()];

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
            Action::ClearMessages => model.messages.clear(),
        }
    }
    Ok(())
}

/// For checking the window without a person in front of it: with SIM_APP_SCREENSHOT=path set, the app
/// starts the cycle, saves one screenshot after a few seconds and exits.
fn developer_hooks(mut state: ResMut<State>, mut commands: Commands, mut exit: MessageWriter<AppExit>) {
    let Ok(path) = std::env::var("SIM_APP_SCREENSHOT") else { return };
    state.frame += 1;
    match state.frame {
        30 => state.model.command(sim_components::Command::StartCycle),
        240 => {
            commands.spawn(Screenshot::primary_window()).observe(save_to_disk(path));
        }
        300 => {
            exit.write(AppExit::Success);
        }
        _ => {}
    }
}
