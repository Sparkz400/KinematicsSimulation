//! The window driven by synthetic input: the app as `main` builds it, without a screen or graphics card (Bevy without
//! winit and without a renderer, so these run in CI as anywhere), fed the input messages winit would send, its panels'
//! widgets found by their labels in egui's accessibility tree.

use super::*;
use bevy::input::ButtonState;
use bevy::input::keyboard::{Key, KeyboardInput};
use bevy::input::mouse::{MouseButtonInput, MouseMotion, MouseScrollUnit, MouseWheel};
use bevy::render::RenderPlugin;
use bevy::render::settings::WgpuSettings;
use bevy::window::{CursorMoved, WindowEvent, WindowResized, WindowResolution};
use bevy::winit::WinitPlugin;
use bevy_egui::{EguiContext, EguiOutput};
use egui::accesskit::{Node, Role};

struct Driver {
    app: App,
    window: Entity,
    cursor: Vec2,
}

/// A widget as the accessibility tree has it: role, label, value, rectangle in logical pixels, ticked, and the nodes
/// it is inside, outermost first (which panel or area it belongs to).
#[derive(Debug, Clone)]
struct Widget {
    role: Role,
    label: String,
    value: String,
    rect: egui::Rect,
    toggled: Option<bool>,
    within: Vec<egui::accesskit::NodeId>,
}

impl Driver {
    fn new() -> Self {
        Self::of(MACHINES[0], "en")
    }

    fn of(entry: &'static sim_core::MachineEntry, lang: &'static str) -> Self {
        let model = Model::new(entry, lang);
        let window = Window { resolution: (1600u32, 900u32).into(), ..default() };
        let mut app = App::new();
        app.add_plugins(
            DefaultPlugins
                .build()
                .disable::<WinitPlugin>()
                // bevy_egui warns every frame that the window has no winit window behind it.
                .disable::<bevy::log::LogPlugin>()
                .set(RenderPlugin { render_creation: WgpuSettings { backends: None, ..default() }.into(), ..default() })
                // As in `main`: the close request is `on_close_request`'s to answer.
                .set(WindowPlugin { primary_window: Some(window), close_when_requested: false, ..default() }),
        );
        add_app(&mut app, model);
        // A frame is 16 ms of the clock egui goes by, however long it took: a click, a drag, a tooltip's delay and an
        // animation take the same frames on a busy machine as on an idle one. (By the wall clock a press and its
        // release a frame apart could be over 0.8 s apart there, and egui then saw no click.)
        app.insert_resource(bevy::time::TimeUpdateStrategy::ManualDuration(std::time::Duration::from_millis(16)));
        app.finish();
        app.cleanup();
        let window = app.world_mut().query_filtered::<Entity, With<PrimaryWindow>>().single(app.world()).unwrap();
        let mut driver = Self { app, window, cursor: Vec2::new(-1.0, -1.0) };
        driver.frames(2);
        for mut context in driver.app.world_mut().query::<&mut EguiContext>().iter_mut(driver.app.world_mut()) {
            context.get_mut().enable_accesskit();
        }
        driver.frames(2);
        driver.wait("the machine's first scene, framed", |d| d.state().bounds.is_some() && !d.state().fit);
        driver.settle();
        driver
    }

    fn frames(&mut self, n: usize) {
        for _ in 0..n {
            self.app.update();
        }
    }

    /// The one way to wait for the app: frames until `look` finds what it looks for, for up to `PATIENCE` of wall-clock
    /// time; failing with `what` and what it last saw. The session, the jobs and a tab's first layout run at their own
    /// pace, which on a loaded machine is many frames.
    fn eventually<T>(&mut self, what: &str, look: impl FnMut(&mut Self) -> Result<T, String>) -> T {
        self.eventually_within(PATIENCE, what, look)
    }

    /// `eventually` with another bound: an analysis's result, computed while others run, takes minutes on a loaded
    /// machine.
    fn eventually_within<T>(
        &mut self,
        patience: std::time::Duration,
        what: &str,
        mut look: impl FnMut(&mut Self) -> Result<T, String>,
    ) -> T {
        let start = std::time::Instant::now();
        loop {
            match look(self) {
                Ok(found) => return found,
                Err(seen) if start.elapsed() > patience => panic!("{what}: not within {patience:?}; last seen: {seen}"),
                Err(_) => {
                    self.app.update();
                    std::thread::sleep(std::time::Duration::from_millis(2));
                }
            }
        }
    }

    /// `eventually` for a condition on the app.
    fn wait(&mut self, what: &str, done: impl Fn(&Self) -> bool) {
        self.eventually(what, |d| if done(d) { Ok(()) } else { Err(String::new()) });
    }

    /// `wait` for an analysis's result, up to `RESULT_PATIENCE`.
    fn wait_result(&mut self, what: &str, done: impl Fn(&Self) -> bool) {
        let analyses = |d: &Self| format!("analyses running: {}", d.state().model.analysis_running());
        self.eventually_within(RESULT_PATIENCE, what, |d| if done(d) { Ok(()) } else { Err(analyses(d)) });
    }

    /// Until the panels' controls stand still: the same controls at the same places two frames running (a tab's rows
    /// that wrap are laid out from the frame before, a group opens with an animation).
    fn settle(&mut self) {
        let controls = |d: &mut Self| -> Vec<(Role, String, [i32; 4])> {
            let interactive = |r: Role| {
                matches!(
                    r,
                    Role::Button
                        | Role::CheckBox
                        | Role::RadioButton
                        | Role::ComboBox
                        | Role::SpinButton
                        | Role::TextInput
                        | Role::Slider
                )
            };
            let mut found: Vec<_> = d
                .widgets()
                .into_iter()
                .filter(|w| interactive(w.role))
                .map(|w| (w.role, w.label, [w.rect.min.x, w.rect.min.y, w.rect.max.x, w.rect.max.y].map(|v| v as i32)))
                .collect();
            found.sort_by(|a, b| format!("{a:?}").cmp(&format!("{b:?}")));
            found
        };
        let mut last = controls(self);
        self.eventually("the panels standing still", |d| {
            let now = controls(d);
            let still = now == last && !now.is_empty();
            last = now;
            if still { Ok(()) } else { Err(format!("{} controls moving", last.len())) }
        });
    }

    /// Until the 3D camera stands still, framed.
    fn camera_still(&mut self) {
        let mut last = camera(self);
        self.eventually("the camera standing still", |d| {
            d.frames(1);
            let now = camera(d);
            let still = now == last && !d.state().fit;
            last = now;
            if still { Ok(()) } else { Err(format!("{now:?}")) }
        });
    }

    fn state(&self) -> &State {
        self.app.world().resource::<State>()
    }

    fn state_mut(&mut self) -> Mut<'_, State> {
        self.app.world_mut().resource_mut::<State>()
    }

    fn send<M: bevy::ecs::message::Message>(&mut self, message: M) {
        self.app.world_mut().write_message(message);
    }

    /// The pointer to `to` (logical pixels), as winit reports a move: the window's cursor, the move for egui and
    /// the motion for the camera.
    fn move_to(&mut self, to: Vec2) {
        let delta = to - self.cursor;
        self.cursor = to;
        let window = self.window;
        self.app.world_mut().get_mut::<Window>(window).unwrap().set_cursor_position(Some(to));
        let moved = CursorMoved { window, position: to, delta: Some(delta) };
        self.send(WindowEvent::CursorMoved(moved.clone()));
        self.send(moved);
        self.send(MouseMotion { delta });
        self.frames(1);
    }

    fn button(&mut self, button: MouseButton, state: ButtonState) {
        let input = MouseButtonInput { button, state, window: self.window };
        self.send(WindowEvent::MouseButtonInput(input));
        self.send(input);
        self.frames(1);
    }

    fn click_at(&mut self, at: Vec2) {
        self.move_to(at);
        // Settled under the pointer, as a hand would be: a tooltip of the widget left goes.
        self.frames(2);
        self.button(MouseButton::Left, ButtonState::Pressed);
        self.button(MouseButton::Left, ButtonState::Released);
        self.frames(1);
    }

    /// A drag with `button` from `from` by `by`, in `steps` moves.
    fn drag(&mut self, button: MouseButton, from: Vec2, by: Vec2, steps: usize) {
        self.move_to(from);
        self.button(button, ButtonState::Pressed);
        for i in 1..=steps {
            self.move_to(from + by * i as f32 / steps as f32);
        }
        self.button(button, ButtonState::Released);
    }

    fn wheel(&mut self, y: f32, unit: MouseScrollUnit) {
        let wheel = MouseWheel { unit, x: 0.0, y, window: self.window, phase: bevy::input::touch::TouchPhase::Moved };
        self.send(WindowEvent::MouseWheel(wheel));
        self.send(wheel);
        self.frames(1);
    }

    fn key(&mut self, code: KeyCode, logical: Key) {
        for state in [ButtonState::Pressed, ButtonState::Released] {
            let text = match (&logical, state) {
                (Key::Character(c), ButtonState::Pressed) => Some(c.clone()),
                _ => None,
            };
            let input = KeyboardInput {
                key_code: code,
                logical_key: logical.clone(),
                state,
                text,
                repeat: false,
                window: self.window,
            };
            self.send(WindowEvent::KeyboardInput(input.clone()));
            self.send(input);
            self.frames(1);
        }
    }

    /// Text typed key by key, as a keyboard sends it.
    fn type_text(&mut self, text: &str) {
        for c in text.chars() {
            let code = match c {
                ' ' => KeyCode::Space,
                c if c.is_ascii_digit() => KeyCode::Digit0,
                _ => KeyCode::KeyA,
            };
            let logical = if c == ' ' { Key::Space } else { Key::Character(c.to_string().into()) };
            self.key(code, logical);
        }
    }

    fn resize(&mut self, width: f32, height: f32) {
        let window = self.window;
        self.app.world_mut().get_mut::<Window>(window).unwrap().resolution =
            WindowResolution::new(width as u32, height as u32);
        // What winit reports after resizing, which the cameras and egui go by.
        let resized = WindowResized { window, width, height };
        self.send(WindowEvent::WindowResized(resized.clone()));
        self.send(resized);
        self.frames(1);
    }

    /// The widgets egui drew last frame.
    fn widgets(&mut self) -> Vec<Widget> {
        // A frame first: what a click did is applied after the panels are drawn, so the frame it was in shows the
        // panels from before it.
        self.frames(1);
        let mut outputs = self.app.world_mut().query::<&EguiOutput>();
        let output = outputs.iter(self.app.world()).next().expect("an egui context");
        let Some(update) = &output.platform_output.accesskit_update else { return Vec::new() };
        let parents: std::collections::HashMap<_, _> =
            update.nodes.iter().flat_map(|(id, node)| node.children().iter().map(move |child| (*child, *id))).collect();
        let within = |mut id| {
            let mut chain = Vec::new();
            while let Some(parent) = parents.get(&id) {
                chain.push(*parent);
                id = *parent;
            }
            chain.reverse();
            chain
        };
        update.nodes.iter().map(|(id, node)| widget(node, within(*id))).collect()
    }

    /// The widget of role `role` labelled `label`, once the panels show it; fails naming what there was.
    fn find(&mut self, role: Role, label: &str) -> Widget {
        self.find_where(&format!("{role:?} \"{label}\""), |w| w.role == role && w.label == label)
    }

    /// The first widget for which `pick` holds, once the panels show one.
    fn find_where(&mut self, what: &str, pick: impl Fn(&Widget) -> bool) -> Widget {
        self.eventually(what, |d| {
            let widgets = d.widgets();
            widgets.iter().find(|w| pick(w)).cloned().ok_or_else(|| {
                let named: Vec<_> =
                    widgets.iter().filter(|w| !w.label.is_empty()).map(|w| (w.role, &w.label)).collect();
                format!("{named:?}")
            })
        })
    }

    /// The first widget of `role` in `region`, top to bottom, left to right, once there is one.
    fn first_in(&mut self, region: usize, role: Role) -> Widget {
        self.eventually(&format!("a {role:?} in region {region}"), |d| {
            d.in_region(region, role).into_iter().next().ok_or_else(|| "none".to_string())
        })
    }

    fn has(&mut self, role: Role, label: &str) -> bool {
        self.widgets().iter().any(|w| w.role == role && w.label == label)
    }

    /// Clicks the widget found by `find`.
    fn click(&mut self, role: Role, label: &str) {
        let at = self.find(role, label).rect.center();
        self.click_at(Vec2::new(at.x, at.y));
    }

    fn tr(&self, key: &str) -> String {
        self.state().model.tr.tr(key).to_string()
    }

    fn button_of(&mut self, key: &str) {
        let label = self.tr(key);
        self.click(Role::Button, &label);
    }

    /// Somewhere in the 3D view, clear of the controls floating on it: right of its middle, a little down.
    fn in_view(&self) -> Vec2 {
        let [left, top, right, bottom] = self.state().view_rect;
        Vec2::new(left + (right - left) * 0.7, top + (bottom - top) * 0.55)
    }

    /// A key held (`true`) or let go, as for a modifier.
    fn hold(&mut self, code: KeyCode, logical: Key, down: bool) {
        let state = if down { ButtonState::Pressed } else { ButtonState::Released };
        let input = KeyboardInput {
            key_code: code,
            logical_key: logical,
            state,
            text: None,
            repeat: false,
            window: self.window,
        };
        self.send(WindowEvent::KeyboardInput(input.clone()));
        self.send(input);
        self.frames(1);
    }

    /// A click into the field at `rect`, all of its text selected (Ctrl+A) and `text` typed over it.
    fn type_into(&mut self, rect: egui::Rect, text: &str) {
        self.click_at(Vec2::new(rect.center().x, rect.center().y));
        self.hold(KeyCode::ControlLeft, Key::Control, true);
        self.key(KeyCode::KeyA, Key::Character("a".into()));
        self.hold(KeyCode::ControlLeft, Key::Control, false);
        self.type_text(text);
    }

    fn enter(&mut self) {
        self.key(KeyCode::Enter, Key::Enter);
    }

    /// The panels as `view_rect` leaves them: top bar, setup, tools, scopes, and the view between them.
    fn regions(&self) -> [egui::Rect; 5] {
        let [left, top, right, bottom] = self.state().view_rect;
        let window = self.app.world().get::<Window>(self.window).unwrap();
        let (width, height) = (window.width(), window.height());
        let r = |x0: f32, y0: f32, x1: f32, y1: f32| egui::Rect::from_min_max(egui::pos2(x0, y0), egui::pos2(x1, y1));
        [
            r(0.0, 0.0, width, top),
            r(0.0, top, left, bottom),
            r(right, top, width, bottom),
            r(0.0, bottom, width, height),
            r(left, top, right, bottom),
        ]
    }

    /// The widgets of `role` wholly inside `region` (`regions`), top to bottom, left to right.
    fn in_region(&mut self, region: usize, role: Role) -> Vec<Widget> {
        let area = self.regions()[region];
        let mut found: Vec<_> =
            self.widgets().into_iter().filter(|w| w.role == role && area.contains_rect(w.rect)).collect();
        found.sort_by(|a, b| (a.rect.top(), a.rect.left()).partial_cmp(&(b.rect.top(), b.rect.left())).unwrap());
        found
    }

    /// Opens a tab of the setup or tools panel by its label's key: until it shows as open, and its controls stand
    /// still.
    fn tab(&mut self, key: &str) {
        let label = self.tr(key);
        let is_tab = |w: &Widget| w.role == Role::Button && w.label == label && w.toggled.is_some();
        let tab = self.find_where(&format!("the tab {label}"), is_tab);
        if tab.toggled != Some(true) {
            let at = tab.rect.center();
            self.click_at(Vec2::new(at.x, at.y));
            self.find_where(&format!("the tab {label} open"), |w| is_tab(w) && w.toggled == Some(true));
        }
        self.settle();
    }

    fn messages(&self) -> Vec<&'static str> {
        self.state().model.messages.iter().map(|(key, _)| *key).collect()
    }
}

/// How long a wait for the app may take: generous, for a loaded CI runner.
const PATIENCE: std::time::Duration = std::time::Duration::from_secs(60);
/// How long an analysis's result may take: the cycle's, while the sensitivity, the optimiser, the sweep, the tuning
/// and the search run beside it, took over 60 s with the machine loaded.
const RESULT_PATIENCE: std::time::Duration = std::time::Duration::from_secs(600);

const SETUP: usize = 1;
const TOOLS: usize = 2;
const TOP: usize = 0;

fn widget(node: &Node, within: Vec<egui::accesskit::NodeId>) -> Widget {
    let rect = node.bounds().map_or(egui::Rect::NOTHING, |b| {
        egui::Rect::from_min_max(egui::pos2(b.x0 as f32, b.y0 as f32), egui::pos2(b.x1 as f32, b.y1 as f32))
    });
    Widget {
        role: node.role(),
        label: node.label().unwrap_or_default().to_string(),
        value: node.value().unwrap_or_default().to_string(),
        rect,
        toggled: node.toggled().map(|t| t == egui::accesskit::Toggled::True),
        within,
    }
}

/// The 3D camera's place and turn.
fn camera(d: &mut Driver) -> Transform {
    *d.app.world_mut().query_filtered::<&Transform, With<WorldCamera>>().single(d.app.world()).unwrap()
}

/// The corners of the machine's box that the camera does not see: outside Bevy's default field of view (45 degrees
/// up and down, the view's width across) or behind it. Worked out here from the camera's transform alone.
fn corners_out_of_sight(d: &mut Driver) -> Vec<Vec3> {
    corners_beyond(d, 0.0)
}

/// `corners_out_of_sight` with `room` logical pixels at the top and the bottom of the view counted as out of sight.
fn corners_beyond(d: &mut Driver, room: f32) -> Vec<Vec3> {
    let (low, high) = d.state().bounds.expect("the machine's box");
    let [left, top, right, bottom] = d.state().view_rect;
    let aspect = (right - left) / (bottom - top);
    let free = (bottom - top - 2.0 * room) / (bottom - top);
    let camera = camera(d);
    let tan = (std::f32::consts::FRAC_PI_4 / 2.0).tan();
    (0..8)
        .map(|i| Vec3::new([low.x, high.x][i & 1], [low.y, high.y][(i >> 1) & 1], [low.z, high.z][i >> 2]))
        .filter(|corner| {
            let p = camera.rotation.inverse() * (*corner - camera.translation);
            let depth = -p.z;
            !(depth > 0.0 && (p.x / depth).abs() <= tan * aspect + 1e-4 && (p.y / depth).abs() <= tan * free + 1e-4)
        })
        .collect()
}

#[test]
fn a_drag_in_the_view_turns_the_camera_about_its_target() {
    let mut d = Driver::new();
    let (yaw, pitch, distance, target) = {
        let o = &d.state().orbit;
        (o.yaw, o.pitch, o.distance, o.target)
    };
    // 0.005 rad per pixel: 200 pixels left is 1 rad of yaw, 60 pixels down 0.3 rad of pitch. The input is taken in the
    // frame it is sent in, so the drag has turned the camera when it returns.
    let from = d.in_view();
    d.drag(MouseButton::Left, from, Vec2::new(-200.0, 60.0), 10);
    let o = &d.state().orbit;
    assert!((o.yaw - (yaw + 1.0)).abs() < 1e-4, "yaw {} from {yaw}", o.yaw);
    assert!((o.pitch - (pitch + 0.3)).abs() < 1e-4, "pitch {} from {pitch}", o.pitch);
    assert_eq!((o.distance, o.target), (distance, target));
    // The camera stands `distance` from the target, looking at it.
    let (yaw, pitch) = (o.yaw, o.pitch);
    let camera = camera(&mut d);
    assert!((camera.translation.distance(target) - distance).abs() < 1e-4);
    assert!(camera.forward().dot((target - camera.translation).normalize()) > 0.9999);
    let expected = Quat::from_euler(EulerRot::YXZ, yaw, -pitch, 0.0);
    assert!(camera.rotation.angle_between(expected) < 1e-4);
    // The pitch stops short of straight up or down.
    d.drag(MouseButton::Left, from, Vec2::new(0.0, 500.0), 10);
    assert!((d.state().orbit.pitch - 1.55).abs() < 1e-6, "{}", d.state().orbit.pitch);
}

/// Where egui saw the pointer last, and whether a button was down: the sign that a move or a release was taken.
fn pointer(d: &mut Driver) -> (Option<egui::Pos2>, bool) {
    let mut contexts = d.app.world_mut().query::<&mut EguiContext>();
    let ctx = contexts.iter_mut(d.app.world_mut()).next().unwrap().get_mut().clone();
    ctx.input(|i| (i.pointer.latest_pos(), i.pointer.any_down()))
}

#[test]
fn a_drag_that_starts_on_a_panel_does_not_turn_the_camera() {
    let mut d = Driver::new();
    let (yaw, pitch) = (d.state().orbit.yaw, d.state().orbit.pitch);
    // From the empty lower part of the setup panel into the view, and from the top bar's empty right end. Each drag is
    // taken when the pointer stands where it ended, released: only then does "the camera did not turn" mean anything.
    let [left, top, _, bottom] = d.state().view_rect;
    for (from, by) in [
        (Vec2::new(left - 120.0, bottom - 40.0), Vec2::new(400.0, -100.0)),
        (Vec2::new(1590.0, top - 5.0), Vec2::new(-600.0, 200.0)),
    ] {
        d.drag(MouseButton::Left, from, by, 10);
        let end = from + by;
        d.wait("the drag taken", |d| d.app.world().resource::<ButtonInput<MouseButton>>().get_pressed().len() == 0);
        assert_eq!(pointer(&mut d), (Some(egui::pos2(end.x, end.y)), false));
        assert_eq!((d.state().orbit.yaw, d.state().orbit.pitch), (yaw, pitch));
    }
    // Nor does one on the view's own controls (the deflections' slider sits there): the slider moves instead, the
    // sign that the drag was taken.
    let view = d.regions()[4];
    let slider =
        d.find_where("the deflections' slider over the view", |w| w.role == Role::Slider && view.contains_rect(w.rect));
    let exaggerate = slider.rect.center();
    d.drag(MouseButton::Left, Vec2::new(exaggerate.x, exaggerate.y), Vec2::new(300.0, 100.0), 10);
    d.wait("the slider moved", |d| d.state().model.exaggeration > 1.0);
    assert_eq!((d.state().orbit.yaw, d.state().orbit.pitch), (yaw, pitch));
}

#[test]
fn the_wheel_zooms_within_its_limits_and_only_over_the_view() {
    let mut d = Driver::new();
    let at = d.in_view();
    d.move_to(at);
    let distance = d.state().orbit.distance;
    // A notch is a tenth nearer; one back out again is a ninth further. The wheel is taken in the frame it is sent in.
    d.wheel(1.0, MouseScrollUnit::Line);
    assert!((d.state().orbit.distance - distance * 0.9).abs() < 1e-5);
    d.wheel(-1.0, MouseScrollUnit::Line);
    assert!((d.state().orbit.distance - distance).abs() < 1e-5);
    // A touchpad's pixels: 50 of them as a notch, not a jump to the nearest.
    d.wheel(50.0, MouseScrollUnit::Pixel);
    assert!((d.state().orbit.distance - distance * 0.9).abs() < 1e-4, "{}", d.state().orbit.distance);
    for _ in 0..40 {
        d.wheel(3.0, MouseScrollUnit::Line);
    }
    assert_eq!(d.state().orbit.distance, 0.2);
    for _ in 0..60 {
        d.wheel(-3.0, MouseScrollUnit::Line);
    }
    assert_eq!(d.state().orbit.distance, 20.0);
    // Over a panel the wheel scrolls the panel: the pointer is seen there first, then the wheel leaves the camera.
    let [left, _, _, bottom] = d.state().view_rect;
    let panel = Vec2::new(left - 100.0, bottom - 100.0);
    d.move_to(panel);
    assert_eq!(pointer(&mut d).0, Some(egui::pos2(panel.x, panel.y)));
    d.wheel(5.0, MouseScrollUnit::Line);
    assert_eq!(d.state().orbit.distance, 20.0);
    // The same wheel back over the view zooms: the wheel was not lost.
    d.move_to(at);
    d.wheel(5.0, MouseScrollUnit::Line);
    assert!(d.state().orbit.distance < 20.0);
}

#[test]
fn a_drag_with_the_right_or_middle_button_pans_the_target() {
    let mut d = Driver::new();
    let (target, distance, yaw) = (d.state().orbit.target, d.state().orbit.distance, d.state().orbit.yaw);
    let [_, top, _, bottom] = d.state().view_rect;
    // The point under the pointer stays under it: a pixel is 2 d tan(22.5 deg) / (view height) at the target.
    let per_pixel = 2.0 * distance * (std::f32::consts::FRAC_PI_8).tan() / (bottom - top);
    let from = d.in_view();
    d.drag(MouseButton::Right, from, Vec2::new(100.0, 0.0), 5);
    let right = camera(&mut d).right();
    let moved = d.state().orbit.target - target;
    assert!((moved - -right * 100.0 * per_pixel).length() < 1e-4, "{moved} against {}", -right * 100.0 * per_pixel);
    let target = d.state().orbit.target;
    d.drag(MouseButton::Middle, from, Vec2::new(0.0, 50.0), 5);
    let up = camera(&mut d).up();
    let moved = d.state().orbit.target - target;
    assert!((moved - up * 50.0 * per_pixel).length() < 1e-4, "{moved}");
    // Panning neither turns nor zooms, and a right click is no pick.
    assert_eq!((d.state().orbit.yaw, d.state().orbit.distance), (yaw, distance));
    assert_eq!(d.state().panels.part, None);
}

#[test]
fn the_view_presets_put_the_camera_where_they_say_with_the_machine_in_view() {
    let mut d = Driver::new();
    // (button, where the camera looks from, seen from the target)
    for (key, from, yaw, pitch) in [
        ("ui-view-front", Vec3::Z, 0f32, 0f32),
        ("ui-view-side", Vec3::X, 90.0, 0.0),
        ("ui-view-top", Vec3::Y, 0.0, 89.0),
        ("ui-view-iso", Vec3::new(30f32.to_radians().sin(), 0.0, 30f32.to_radians().cos()), 30.0, 20.0),
    ] {
        // Turned away first, so that each preset has to move the camera.
        let at = d.in_view();
        d.drag(MouseButton::Left, at, Vec2::new(77.0, -40.0), 4);
        d.button_of(key);
        let expected = match key {
            "ui-view-top" => Vec3::new(0.0, 89f32.to_radians().sin(), 89f32.to_radians().cos()),
            "ui-view-iso" => {
                let pitch = 20f32.to_radians();
                Vec3::new(from.x * pitch.cos(), pitch.sin(), from.z * pitch.cos())
            }
            _ => from,
        };
        // Until the preset is taken and the camera stands still, framed.
        let angles = (yaw.to_radians(), pitch.to_radians());
        d.wait(&format!("{key} taken"), |d| (d.state().orbit.yaw, d.state().orbit.pitch) == angles);
        d.camera_still();
        let camera = camera(&mut d);
        let seen_from = (camera.translation - d.state().orbit.target).normalize();
        assert!(seen_from.distance(expected) < 1e-4, "{key}: from {seen_from}, expected {expected}");
        // Clear of the folded controls on top and the timeline below (`OVERLAY_ROOM`).
        assert_eq!(corners_beyond(&mut d, OVERLAY_ROOM), Vec::<Vec3>::new(), "{key}");
        // Framed: zoomed in a little, a corner goes under them.
        d.state_mut().orbit.distance *= 0.8;
        d.camera_still();
        assert!(!corners_beyond(&mut d, OVERLAY_ROOM).is_empty(), "{key}: not framed but far off");
    }
}

#[test]
fn a_window_resize_keeps_the_machine_in_view() {
    let mut d = Driver::new();
    assert!(corners_out_of_sight(&mut d).is_empty());
    for (width, height) in [(1280.0, 720.0), (1000.0, 800.0), (1920.0, 1080.0), (1280.0, 720.0)] {
        d.resize(width, height);
        // The panels keep their widths and the scopes their height: the view takes what the window gained or lost.
        let size = |d: &Driver| {
            let [left, top, right, bottom] = d.state().view_rect;
            (right - left, bottom - top)
        };
        d.wait(&format!("the view at {width} x {height}"), |d| size(d) == (width - 700.0, height - 302.0));
        d.camera_still();
        assert_eq!(corners_out_of_sight(&mut d), Vec::<Vec3>::new(), "{width} x {height}");
    }
}

#[test]
fn pause_speed_units_and_theme_from_the_top_bar() {
    let mut d = Driver::new();
    d.button_of("ui-pause");
    d.wait("paused", |d| d.state().model.paused);
    d.button_of("ui-resume");
    d.wait("running", |d| !d.state().model.paused);
    // The speed slider is logarithmic from 0.01 to 10 and shows two decimals. A click at a fifth of the widget gave 0.03
    // (measured; 0.01 * 1000^0.2 = 0.040 for a fifth of the range, less where egui keeps the handle clear of the ends):
    // low in the range, above its start.
    let slider = d.first_in(TOP, Role::Slider).rect;
    d.click_at(Vec2::new(slider.left() + slider.width() * 0.2, slider.center().y));
    d.wait("the speed set", |d| d.state().model.speed != 1.0);
    let speed = d.state().model.speed;
    assert!((0.015..=0.045).contains(&speed), "{speed}");
    // The range reaches to 10 (it ended at 2): a click near the right end is above 2 (8 measured).
    d.click_at(Vec2::new(slider.right() - slider.height() / 2.5, slider.center().y));
    d.wait("the speed at the end", |d| d.state().model.speed != speed);
    assert!(d.state().model.speed > 2.0, "{}", d.state().model.speed);
    for key in ["ui-units-rpm", "ui-units-mm", "ui-theme-light"] {
        let label = d.tr(key);
        d.click(Role::CheckBox, &label);
        d.find_where(key, |w| w.role == Role::CheckBox && w.label == label && w.toggled == Some(true));
    }
    let (units, panels) = (d.state().model.units, d.state().panels);
    assert!(units.rpm && units.mm && panels.light);
}

#[test]
fn the_motion_tab_drives_the_machine() {
    let mut d = Driver::new();
    let tool = |d: &Driver| d.state().model.scene.tool.expect("a tool");
    let still = tool(&d);
    d.button_of("ui-start-cycle");
    d.wait("the cycle moves the tool", |d| {
        let now = tool(d);
        (0..3).map(|i| (now[i] - still[i]).abs()).sum::<f64>() > 1e-3
    });
    d.button_of("ui-stop");
    d.button_of("ui-disable");
    d.button_of("ui-enable");
    // The first axis's target typed into its field: a click opens it for typing, Enter takes it.
    let field = d.first_in(SETUP, Role::SpinButton).rect;
    d.type_into(field, "0.05");
    d.enter();
    d.wait("the target taken", |d| d.state().model.target[0] == 0.05);
    d.button_of("ui-move");
    let before = d.state().model.scene.clone();
    d.wait("the move moves the parts", |d| d.state().model.scene.parts != before.parts);
}

/// A number of the config's parameter panel, and its path.
fn some_parameter(d: &Driver) -> (String, f64) {
    let groups = d.state().model.param_groups();
    groups
        .iter()
        .flat_map(|g| &g.params)
        .find_map(|(path, _, value)| value.as_float().filter(|v| *v > 0.0).map(|v| (path.clone(), v)))
        .expect("a number in the config")
}

#[test]
fn a_value_typed_in_the_parameter_panel_reaches_the_simulation_and_a_bad_one_is_refused() {
    let mut d = Driver::new();
    d.tab("ui-parameters");
    let (path, old) = some_parameter(&d);
    // Found again before each value: the messages strip on top of the panel moves it down with the first message.
    let field = |d: &mut Driver| d.first_in(SETUP, Role::TextInput).rect;
    let new = old * 1.1;
    let at = field(&mut d);
    d.type_into(at, &format!("{path} = {new}"));
    d.enter();
    d.wait("the value set", |d| d.state().model.config.f64(&path).ok() == Some(new));
    // A word for a number, by the button: refused with its message, shown under the field, and the value kept.
    let at = field(&mut d);
    d.type_into(at, &format!("{path} = \"fast\""));
    d.button_of("ui-set-value-apply");
    d.wait("the word refused", |d| d.messages().len() == 1);
    assert_eq!(d.state().model.config.f64(&path).unwrap(), new);
    let (key, detail) = d.state().model.messages.last().cloned().expect("a message");
    let shown = d.state().model.tr.tr_arg(key, &detail);
    d.find_where(&format!("{shown} shown"), |w| w.role == Role::Label && w.value == shown);
    // A key the config does not have.
    let at = field(&mut d);
    d.type_into(at, "no.such.key = 1");
    d.enter();
    d.wait("the key refused", |d| d.messages().len() == 2);
    // A value outside what the simulation takes comes back from it, refused, and the value before stays. Its answer is
    // also the sign that the simulation has answered the value before it, which it took: that one brought no message.
    let at = field(&mut d);
    d.type_into(at, &format!("{path} = -1.0"));
    d.enter();
    d.wait("the simulation's refusal", |d| d.messages().len() == 3);
    d.wait("the value before back", |d| d.state().model.config.f64(&path).ok() == Some(new));
    assert_eq!(d.messages().len(), 3, "{:?}", d.messages());
}

#[test]
fn a_drag_on_a_parameter_changes_it() {
    let mut d = Driver::new();
    d.tab("ui-parameters");
    // The first group opened (it opens with an animation: until its controls stand still), its first number dragged.
    let groups = d.state().model.param_groups();
    let group = d.tr(&groups[0].key);
    d.click(Role::Button, &group);
    let (path, old) = groups[0]
        .params
        .iter()
        .find_map(|(path, _, value)| value.as_float().map(|v| (path.clone(), v)))
        .expect("a number in the first group");
    let is_field = |w: &Widget| w.role == Role::SpinButton && w.value.parse::<f64>().ok() == Some(old);
    d.find_where("the group's first number", is_field);
    d.settle();
    let at = d.find_where("the group's first number", is_field).rect.center();
    // One percent a pixel; egui counts a drag from a few pixels on: 40 pixels make some 30 percent.
    d.drag(MouseButton::Left, Vec2::new(at.x, at.y), Vec2::new(40.0, 0.0), 10);
    d.wait("the value dragged", |d| d.state().model.config.f64(&path).ok() != Some(old));
    let now = d.state().model.config.f64(&path).unwrap();
    assert!(now / old > 1.2 && now / old < 1.45, "{path}: {old} to {now}");
}

/// A message that arrives during a drag (the strip on top of the panel appears) does not end the drag: the field keeps
/// following the pointer. Half the drag, a message, the other half: the second half changes the value as much again.
#[test]
fn a_message_during_a_drag_does_not_end_it() {
    let mut d = Driver::new();
    d.tab("ui-parameters");
    let groups = d.state().model.param_groups();
    let group = d.tr(&groups[0].key);
    d.click(Role::Button, &group);
    let (path, old) = groups[0]
        .params
        .iter()
        .find_map(|(path, _, value)| value.as_float().map(|v| (path.clone(), v)))
        .expect("a number in the first group");
    let is_field = |w: &Widget| w.role == Role::SpinButton && w.value.parse::<f64>().ok() == Some(old);
    d.find_where("the group's first number", is_field);
    d.settle();
    let at = d.find_where("the group's first number", is_field).rect.center();
    let from = Vec2::new(at.x, at.y);
    d.move_to(from);
    d.button(MouseButton::Left, ButtonState::Pressed);
    for i in 1..=10 {
        d.move_to(from + Vec2::new(4.0 * i as f32, 0.0));
    }
    d.wait("the first half taken", |d| d.state().model.config.f64(&path).ok() != Some(old));
    let half = d.state().model.config.f64(&path).unwrap();
    assert!(d.messages().is_empty(), "{:?}", d.messages());
    d.state_mut().model.messages.push(("ui-job-cancelled", String::new()));
    d.frames(2);
    for i in 11..=20 {
        d.move_to(from + Vec2::new(4.0 * i as f32, 0.0));
    }
    d.button(MouseButton::Left, ButtonState::Released);
    d.frames(2);
    let end = d.state().model.config.f64(&path).unwrap();
    // 40 more pixels at one percent of the value each: some 40 % on top of the first half's value.
    assert!(end / half > 1.25, "{path}: {old}, {half} after the first half, {end} at the end");
}

#[test]
fn an_effect_switch_reaches_the_config_and_stays() {
    let mut d = Driver::new();
    d.tab("ui-effects");
    let (path, _, on) = d.state().model.effects().remove(0);
    let switch = d.first_in(SETUP, Role::CheckBox);
    assert_eq!(switch.toggled, Some(on));
    let at = switch.rect.center();
    d.click_at(Vec2::new(at.x, at.y));
    // Until the session has taken the switch (on a busy machine 60 frames went by before it had), and the box shows it.
    d.wait("the switch in the config", |d| d.state().model.config.bool(&path).ok() == Some(!on));
    d.eventually("the switch shown", |d| {
        let shown = d.first_in(SETUP, Role::CheckBox).toggled;
        if shown == Some(!on) { Ok(()) } else { Err(format!("{shown:?}")) }
    });
    // And stays: the simulation's answer to the edit (were it a refusal, the old value would be back) is seen by
    // a later edit's answer. The refusal of a negative number is that later answer.
    let (number, value) = some_parameter(&d);
    d.state_mut().model.set_value(&format!("{number} = -1.0"));
    d.wait("the simulation's answer", |d| !d.messages().is_empty());
    assert_eq!(d.state().model.config.f64(&number).unwrap(), value);
    assert_eq!(d.state().model.config.bool(&path).unwrap(), !on, "{:?}", d.messages());
}

#[test]
fn a_config_file_that_does_not_exist_is_said_and_changes_nothing() {
    let mut d = Driver::new();
    d.tab("ui-config");
    let config = d.state().model.config.clone();
    let field = d.first_in(SETUP, Role::TextInput).rect;
    d.type_into(field, "no-such-file.toml");
    d.wait("the file typed", |d| d.state().model.inputs.config_file == "no-such-file.toml");
    d.button_of("ui-config-open");
    d.wait("the file refused", |d| !d.messages().is_empty());
    assert_eq!(d.state().model.config, config);
    assert_eq!(d.messages().len(), 1, "{:?}", d.messages());
    // Tab moves on to the next field: the part names, typed in turn.
    d.tab("ui-config");
    let fields = d.eventually("the part fields", |d| {
        let fields = d.in_region(SETUP, Role::TextInput);
        if fields.len() > 2 { Ok(fields) } else { Err(format!("{} fields", fields.len())) }
    });
    let supply = d.state().model.inputs.parts[1].clone();
    d.type_into(fields[1].rect, "M1");
    // The motor's catalogue list is next, then the supply's field.
    d.key(KeyCode::Tab, Key::Tab);
    d.key(KeyCode::Tab, Key::Tab);
    d.type_text("x");
    let expected = ("M1".to_string(), format!("{supply}x"));
    let typed = |d: &Driver| (d.state().model.inputs.parts[0].clone(), d.state().model.inputs.parts[1].clone());
    d.wait("both typed", |d| typed(d) == expected);
}

/// Picks `entry` in the list whose shown value is `shown`.
fn pick(d: &mut Driver, shown: &str, entry: &str) {
    let combo = d.find_where(&format!("the list showing {shown}"), |w| w.role == Role::ComboBox && w.value == shown);
    d.click_at(Vec2::new(combo.rect.center().x, combo.rect.center().y));
    d.click(Role::Button, entry);
}

#[test]
fn the_machine_and_the_language_are_picked_from_their_lists() {
    let mut d = Driver::new();
    let (bounds, title) = (d.state().bounds, d.app.world().get::<Window>(d.window).unwrap().title.clone());
    let (machine, other) = (d.tr(MACHINES[0].key), d.tr(MACHINES[1].key));
    pick(&mut d, &machine, &other);
    d.wait("the other machine", |d| d.state().model.entry.id == MACHINES[1].id);
    d.wait("the new machine is framed", |d| d.state().bounds != bounds && !d.state().fit);
    d.camera_still();
    assert!(corners_out_of_sight(&mut d).is_empty());
    let (english, german) = (d.tr("lang-en"), d.tr("lang-de"));
    pick(&mut d, &english, &german);
    d.wait("German", |d| d.state().model.lang == "de");
    assert_ne!(d.app.world().get::<Window>(d.window).unwrap().title, title);
}

/// The labels of every widget the window shows, with the tabs of both side panels opened in turn.
fn every_label(d: &mut Driver) -> std::collections::BTreeSet<String> {
    let mut labels = std::collections::BTreeSet::new();
    let setup = ["ui-motion", "ui-effects", "ui-parameters", "ui-config"];
    let tools = [
        "ui-analysis",
        "ui-effect-views",
        "ui-search",
        "ui-path",
        "ui-trace",
        "ui-sweep-tab",
        "ui-tuning",
        "ui-loop-tab",
    ];
    for key in setup.iter().chain(&tools) {
        d.tab(key);
        for w in d.widgets() {
            let text = if w.label.is_empty() { w.value } else { w.label };
            if text.chars().any(char::is_alphabetic) {
                labels.insert(text);
            }
        }
    }
    labels
}

#[test]
fn every_label_changes_with_the_language() {
    let mut english = Driver::new();
    let mut german = Driver::of(MACHINES[0], "de");
    let (en, de) = (every_label(&mut english), every_label(&mut german));
    // The same in both languages: unit symbols, what the fields hold (a supply, a ranking's keyword), and the words
    // that are German too. Anything else here was left untranslated. (Numbers with a unit, as the timeline's time, are
    // the same in both; which plots have drawn yet depends on the clock.)
    let same: Vec<&str> = en
        .intersection(&de)
        .map(String::as_str)
        .filter(|l| !l.starts_with(|c: char| c == '+' || c == '-' || c.is_ascii_digit()) || *l == "400v")
        .collect();
    let alike =
        ["400v", "cost", "N m", "m", "mm", "mrad/s", "µm", "µrad", "Position [m]", "Simulation", "Live", "Motor"];
    let alike =
        [&alike[..], &["Motor A", "Motor A [N m]", "Motor B", "Motor B [N m]", "Pause", "Plan", "Filter"]].concat();
    let untranslated: Vec<_> = same.iter().filter(|l| !alike.contains(l)).collect();
    assert!(untranslated.is_empty(), "{untranslated:?}");
    assert!(same.len() > 10 && de.len() > 50, "{} labels in German", de.len());
}

/// Widgets of the window that overlap each other, or that stick out sideways from the panel they are in. Which panel
/// (or area over the view) a widget is in is told by the accessibility tree: the one of the panel's anchor (its first
/// tab, the machine's list, the view's first preset, ...) with which it shares the most enclosing nodes. A widget
/// above or below its panel's edge is scrolled out of sight and not looked at.
fn layout_problems(d: &mut Driver) -> Vec<String> {
    let regions = d.regions();
    let anchors: Vec<String> =
        ["ui-motion", "ui-analysis", "ui-scopes-live", "ui-view-front", "ui-timeline-live"].map(|k| d.tr(k)).into();
    let machine = d.tr(d.state().model.entry.key);
    let widgets: Vec<Widget> = d
        .widgets()
        .into_iter()
        // Scroll bars float over a panel's edge by design.
        .filter(|w| {
            w.rect.is_positive()
                && !matches!(w.role, Role::TextRun | Role::GenericContainer | Role::Unknown | Role::ScrollBar)
        })
        .collect();
    let anchors: Vec<(&Widget, usize)> = widgets
        .iter()
        .filter(|w| anchors.contains(&w.label) || (w.role == Role::ComboBox && w.value == machine))
        .filter_map(|w| Some((w, regions.iter().position(|r| r.contains_rect(w.rect))?)))
        .collect();
    let shared = |a: &Widget, b: &Widget| a.within.iter().zip(&b.within).take_while(|(x, y)| x == y).count();
    let mut problems = Vec::new();
    let mut shown = Vec::new();
    for w in &widgets {
        let Some(&(_, region)) = anchors.iter().max_by_key(|(anchor, _)| shared(anchor, w)) else { continue };
        let area = regions[region];
        if w.rect.top() < area.top() - 0.5 || w.rect.bottom() > area.bottom() + 0.5 {
            continue;
        }
        if w.rect.left() < area.left() - 0.5 || w.rect.right() > area.right() + 0.5 {
            problems.push(format!("cut off: {:?} {:?} {:?} at {:?} in {area:?}", w.role, w.label, w.value, w.rect));
        } else {
            shown.push(w);
        }
    }
    for (i, a) in shown.iter().enumerate() {
        for b in &shown[i + 1..] {
            let overlap = a.rect.intersect(b.rect);
            // A widget's own text inside it is no overlap; nor is a label wrapped over rows, whose box spans from the
            // middle of its first row (where the widgets before it stand) to the left of its last.
            let wrapped = |w: &Widget| w.role == Role::Label && w.rect.height() > 30.0;
            if overlap.width() > 1.0
                && overlap.height() > 1.0
                && !a.rect.contains_rect(b.rect)
                && !b.rect.contains_rect(a.rect)
                && !wrapped(a)
                && !wrapped(b)
            {
                problems.push(format!(
                    "overlap: {:?} {:?}{:?} {:?} and {:?} {:?}{:?} {:?}",
                    a.role, a.label, a.value, a.rect, b.role, b.label, b.value, b.rect
                ));
            }
        }
    }
    problems
}
#[test]
fn nothing_is_cut_off_or_overlaps_in_either_language() {
    for (lang, size) in
        [("en", (1600.0, 900.0)), ("de", (1600.0, 900.0)), ("en", (1280.0, 720.0)), ("de", (1280.0, 720.0))]
    {
        let mut d = Driver::of(MACHINES[0], lang);
        d.resize(size.0, size.1);
        let lang = format!("{lang} {size:?}");
        let mut problems = Vec::new();
        let setup = ["ui-motion", "ui-effects", "ui-parameters", "ui-config"];
        let tools = [
            "ui-analysis",
            "ui-effect-views",
            "ui-search",
            "ui-path",
            "ui-trace",
            "ui-sweep-tab",
            "ui-tuning",
            "ui-loop-tab",
        ];
        for (s, t) in setup.iter().cycle().zip(&tools) {
            d.tab(s);
            d.tab(t);
            problems.extend(layout_problems(&mut d).into_iter().map(|p| format!("{lang} {s} {t}: {p}")));
        }
        problems.dedup();
        assert!(problems.is_empty(), "{}", problems.join("\n"));
        // At the default size the timeline is one row: its first button and its last on the same line.
        if size.0 == 1600.0 {
            let view = d.regions()[4];
            let row: Vec<f32> = d
                .widgets()
                .into_iter()
                .filter(|w| view.contains_rect(w.rect) && (w.label == d.tr("ui-timeline-live") || w.label == "1×"))
                .map(|w| w.rect.center().y)
                .collect();
            assert!(row.len() == 2 && (row[0] - row[1]).abs() < 1.0, "{lang}: the timeline's rows at {row:?}");
        }
    }
}

#[test]
fn space_and_the_arrows_over_the_view_replay_and_step_but_not_while_typing() {
    let mut d = Driver::new();
    d.button_of("ui-start-cycle");
    d.wait("scenes recorded", |d| d.state().recorded.len() > 20);
    let view = d.in_view();
    d.move_to(view);
    // Keys are taken in the frame they are sent in.
    d.key(KeyCode::Space, Key::Space);
    assert!(d.state().timeline.playing && d.state().timeline.time.is_some(), "{:?}", d.state().timeline);
    d.key(KeyCode::Space, Key::Space);
    assert!(!d.state().timeline.playing);
    let at = d.state().timeline.time.unwrap();
    d.key(KeyCode::ArrowLeft, Key::ArrowLeft);
    let back = d.state().timeline.time.unwrap();
    assert!(back < at, "{back} after {at}");
    // On to the scene recorded next after it.
    let next = d.state().recorded.iter().map(|(t, _)| *t).find(|t| *t > back).unwrap();
    assert!(next >= at);
    d.key(KeyCode::ArrowRight, Key::ArrowRight);
    assert_eq!(d.state().timeline.time.unwrap(), next);
    // Typing into a field: a space is a space there, and the timeline stays, even with the pointer over the view. The
    // text arriving in the field is the sign that the keys were taken.
    d.tab("ui-parameters");
    let field = d.first_in(SETUP, Role::TextInput).rect;
    d.type_into(field, "a");
    d.move_to(view);
    let timeline = d.state().timeline;
    d.type_text(" b");
    d.key(KeyCode::ArrowLeft, Key::ArrowLeft);
    d.wait("the text typed", |d| d.state().model.inputs.assignment == "a b");
    assert_eq!(d.state().timeline, timeline);
    // Away from the view the keys do nothing either; back over it, the same key replays, the sign that it is taken.
    d.key(KeyCode::Escape, Key::Escape);
    d.move_to(Vec2::new(5.0, 400.0));
    d.key(KeyCode::Space, Key::Space);
    assert_eq!(d.state().timeline, timeline);
    d.move_to(view);
    d.key(KeyCode::Space, Key::Space);
    d.wait("the replay over the view", |d| d.state().timeline.playing);
}

#[test]
fn every_tool_tab_starts_its_computation() {
    let mut d = Driver::new();
    // Analysis: the cycle with its error bands, the sensitivity, the optimiser.
    let bands = d.tr("ui-analysis-bands");
    d.click(Role::CheckBox, &bands);
    d.wait("bands on", |d| d.state().model.inputs.bands);
    for (key, kind) in [
        ("ui-analysis-run-cycle", Analysis::Cycle),
        ("ui-analysis-run-sensitivity", Analysis::Sensitivity),
        ("ui-analysis-run-optimise", Analysis::Optimise),
    ] {
        let label = d.tr(key);
        let tools = d.regions()[TOOLS];
        let button = d.find_where(key, |w| w.role == Role::Button && w.label == label && tools.contains_rect(w.rect));
        d.click_at(Vec2::new(button.rect.center().x, button.rect.center().y));
        d.wait(key, |d| d.state().model.running(kind));
    }
    // The frequency response and the tuning measure; the motor search searches.
    d.tab("ui-sweep-tab");
    d.button_of("ui-sweep-run");
    d.wait("the sweep", |d| d.state().model.running(Analysis::Sweep));
    d.tab("ui-tuning");
    d.button_of("ui-tuning-run");
    d.wait("the tuning", |d| d.state().model.tuning_running());
    d.tab("ui-search");
    d.button_of("ui-search-run");
    d.wait("the search", |d| d.state().model.running(Analysis::Search) || d.state().model.has_result(Analysis::Search));
    // A trace from a file that does not exist: said, nothing replayed.
    d.tab("ui-trace");
    let field = d.first_in(TOOLS, Role::TextInput).rect;
    d.type_into(field, "no-such-trace.csv");
    d.wait("the file typed", |d| d.state().model.inputs.trace_file == "no-such-trace.csv");
    d.button_of("ui-trace-run");
    d.wait("the trace's answer", |d| d.messages().contains(&"ui-file-unreadable"));
    assert!(!d.state().model.running(Analysis::Trace));
    // The cycle's result arrives in its tab.
    d.tab("ui-analysis");
    d.wait_result("the cycle's result", |d| d.state().model.cycle_report.is_some());
}

#[test]
fn a_path_from_a_file_is_planned_run_and_drawn_as_two_lines() {
    let mut d = Driver::new();
    d.tab("ui-path");
    let field = d.first_in(TOOLS, Role::TextInput).rect;
    let file = concat!(env!("CARGO_MANIFEST_DIR"), "/../../examples/corner.path");
    d.type_into(field, file);
    d.wait("the file typed", |d| d.state().model.inputs.path_file == file);
    d.button_of("ui-path-run");
    d.wait_result("the path's analysis", |d| d.state().model.path_run.is_some());
    // The planned line, and the travelled one as the machine runs it.
    assert!(d.state().model.path_run.as_ref().unwrap().scene_line.len() > 10);
    d.wait("the travelled path", |d| d.state().model.trail.as_ref().is_some_and(|t| t.len() > 10));
    // The plan is drawn wider and a little behind, so that the travelled line over it shows: drawn alike, at the same
    // depth, the second line lost the depth test to the first and only the plan was seen.
    // (Checked by eye in the pictures; here that the plan's line is configured so.)
    let store = d.app.world().resource::<GizmoConfigStore>();
    let (planned, travelled) =
        (store.config::<PlanLine>().0, store.config::<bevy::gizmos::config::DefaultGizmoConfigGroup>().0);
    assert!(planned.depth_bias > travelled.depth_bias && planned.line.width > travelled.line.width);
}

#[test]
fn repeated_clicks_on_an_analysis_start_it_once() {
    let mut d = Driver::new();
    let label = d.tr("ui-analysis-run-cycle");
    let tools = d.regions()[TOOLS];
    let button = d.find_where("the cycle's button", |w| {
        w.role == Role::Button && w.label == label && tools.contains_rect(w.rect)
    });
    let at = Vec2::new(button.rect.center().x, button.rect.center().y);
    // Five clicks, each taken (egui saw it pressed and let go there) before the next.
    for _ in 0..5 {
        d.move_to(at);
        d.button(MouseButton::Left, ButtonState::Pressed);
        assert_eq!(pointer(&mut d), (Some(egui::pos2(at.x, at.y)), true));
        d.button(MouseButton::Left, ButtonState::Released);
        assert_eq!(pointer(&mut d), (Some(egui::pos2(at.x, at.y)), false));
    }
    d.wait("the analysis", |d| d.state().model.running(Analysis::Cycle));
    // From the first click on, a spinner stands where the button was, and in the top bar none: the later clicks
    // found no button to start another.
    assert!(!d.has(Role::Button, &label));
}

/// While an analysis runs the tools panel has a Cancel button where the spinner is (and a bar of its runs once it has
/// counted some); a click on it ends the job with the message that it was cancelled, and the button to start is back.
#[test]
fn a_running_analysis_has_a_cancel_button_that_cancels_it() {
    let mut d = Driver::new();
    // A thousand sampled bands: far longer than the clicks take.
    (d.state_mut().model.inputs.bands, d.state_mut().model.inputs.samples) = (true, 1000);
    let (run, cancel) = (d.tr("ui-analysis-run-cycle"), d.tr("ui-job-cancel"));
    d.click(Role::Button, &run);
    d.wait("the analysis", |d| d.state().model.running(Analysis::Cycle));
    let tools = d.regions()[TOOLS];
    let button = d.find_where("the cancel button", |w| {
        w.role == Role::Button && w.label == cancel && tools.contains_rect(w.rect)
    });
    d.click_at(Vec2::new(button.rect.center().x, button.rect.center().y));
    d.wait("the analysis ended", |d| !d.state().model.running(Analysis::Cycle));
    assert!(d.messages().contains(&"ui-job-cancelled"), "{:?}", d.messages());
    assert!(d.state().model.cycle_report.is_none());
    let tools = d.regions()[TOOLS];
    d.find_where("the cycle's button again", |w| {
        w.role == Role::Button && w.label == run && tools.contains_rect(w.rect)
    });
}

#[test]
fn switching_the_machine_during_an_analysis_leaves_no_stale_result() {
    let mut d = Driver::new();
    let label = d.tr("ui-analysis-run-cycle");
    d.click(Role::Button, &label);
    d.wait("the analysis", |d| d.state().model.running(Analysis::Cycle));
    let (machine, other) = (d.tr(MACHINES[0].key), d.tr(MACHINES[1].key));
    pick(&mut d, &machine, &other);
    d.wait("the other machine", |d| d.state().model.entry.id == MACHINES[1].id);
    assert!(!d.state().model.running(Analysis::Cycle));
    // As long as the first machine's analysis takes, and then some: nothing of it arrives.
    let mut first = Driver::new();
    let start = std::time::Instant::now();
    first.click(Role::Button, &label);
    first.wait_result("the first machine's analysis", |d| d.state().model.cycle_report.is_some());
    drop(first);
    let deadline = start.elapsed() + std::time::Duration::from_secs(2);
    let begun = std::time::Instant::now();
    while begun.elapsed() < deadline {
        d.frames(1);
        assert!(d.state().model.cycle_report.is_none());
    }
}

#[test]
fn closing_the_window_during_an_analysis_does_not_hang() {
    let mut d = Driver::new();
    d.button_of("ui-analysis-run-optimise");
    d.wait("the optimiser", |d| d.state().model.running(Analysis::Optimise));
    let window = d.window;
    d.send(bevy::window::WindowCloseRequested { window });
    let start = std::time::Instant::now();
    d.wait("the app exits", |d| d.app.should_exit().is_some());
    drop(d);
    assert!(start.elapsed().as_secs_f64() < 10.0, "{:?}", start.elapsed());
}

#[test]
fn a_value_at_rest_beside_a_part_reads_without_a_minus() {
    let d = Driver::new();
    let model = &d.state().model;
    let label =
        |value: f64, decimals: usize| sim_core::Label { pos: [0.0; 3], owner_key: "", value, unit_key: "", decimals };
    let text = |value, decimals| ui::label_text(model, &label(value, decimals)).trim().to_string();
    assert_eq!(text(-0.0, 0), "0");
    assert_eq!(text(-0.4, 0), "0");
    assert_eq!(text(-0.0004, 3), "0.000");
    assert_eq!(text(-0.6, 0), "-1");
    assert_eq!(text(1234.5678, 1), "1234.6");
}

#[test]
fn the_bottom_panel_keeps_its_height_with_the_kept_runs() {
    let mut d = Driver::new();
    // 250 pixels high by default, and so it stays.
    let bottom = d.state().view_rect[3];
    assert_eq!(bottom, 900.0 - 250.0);
    // It grew by a gap a frame when it was made to keep its height wrongly: a count of frames tells, whatever the load.
    d.frames(30);
    assert_eq!(d.state().view_rect[3], bottom, "live");
    let runs = d.tr("ui-scopes-runs");
    let scopes = d.regions()[3];
    let toggle =
        d.find_where("the runs toggle", |w| w.role == Role::Button && w.label == runs && scopes.contains_rect(w.rect));
    d.click_at(Vec2::new(toggle.rect.center().x, toggle.rect.center().y));
    d.wait("the runs view", |d| d.state().panels.runs_view);
    d.settle();
    assert_eq!(d.state().view_rect[3], bottom, "with no run kept");
    d.tab("ui-runs");
    d.button_of("ui-runs-keep-live");
    d.wait("a run kept", |d| d.state().model.runs.iter().count() == 1);
    // Until its plots are drawn (the panel's controls standing still), then the panel is as high as before.
    d.settle();
    assert_eq!(d.state().view_rect[3], bottom, "with a run kept");
}

#[test]
fn a_plot_across_labels_a_tick_every_60_pixels_or_so() {
    // The planned path's plot: 0.7 m across 340 pixels; egui_plot's base step is what 8 pixels span.
    let input = |range: f64, pixels: f64| egui_plot::GridInput {
        bounds: (-0.02, range - 0.02),
        base_step_size: range / pixels * 8.0,
    };
    let values =
        |marks: Vec<egui_plot::GridMark>| marks.iter().map(|m| (m.value * 1e6).round() / 1e6).collect::<Vec<_>>();
    assert_eq!(values(ui::x_ticks(input(0.7, 340.0))), [0.0, 0.2, 0.4, 0.6]);
    // A torque-speed chart from -600 to 600 rad/s over 380 pixels: 200 apart, 63 pixels.
    let marks = ui::x_ticks(egui_plot::GridInput { bounds: (-600.0, 600.0), base_step_size: 1200.0 / 380.0 * 8.0 });
    assert_eq!(values(marks), [-600.0, -400.0, -200.0, 0.0, 200.0, 400.0, 600.0]);
}

/// The loop-response tab on the first machines with one, two and three motors: its button measures every motor's
/// velocity loop in the background, the result arrives with the margins `sim-cli margins` prints (all stable), the
/// plots' labels show, and a click on another motor shows that one.
#[test]
fn the_loop_tab_measures_every_motor_of_three_machines() {
    let motors = |e: &sim_core::MachineEntry| (e.build)(&e.config()).unwrap().motors().len();
    for n in 1..=3 {
        let entry = MACHINES.iter().copied().find(|e| motors(e) == n).unwrap();
        let id = entry.id;
        let mut d = Driver::of(entry, "en");
        d.tab("ui-loop-tab");
        d.button_of("ui-loop-run");
        d.wait(id, |d| d.state().model.loop_response.running());
        d.wait_result(id, |d| d.state().model.loop_response.result.is_some());
        let motors = d.state().model.session.motors.len();
        let response = d.state().model.loop_response.result.as_ref().unwrap().1.clone();
        assert_eq!(response.curves.len(), motors, "{id}");
        assert!(response.margins.iter().all(|m| m.stable()), "{id}: {:?}", response.margins);
        assert!(response.curves.iter().all(|c| c.len() >= 60), "{id}");
        for key in ["ui-loop-phase-margin", "ui-loop-gain-margin", "ui-loop-open", "ui-loop-critical"] {
            let label = d.tr(key);
            d.find_where(key, |w| w.label.contains(&label) || w.value.contains(&label));
        }
        if motors > 1 {
            let other = d.tr(d.state().model.session.motors[1].key);
            let tools = d.regions()[TOOLS];
            let tab = d.find_where(&other, |w| w.label == other && w.toggled.is_some() && tools.contains_rect(w.rect));
            d.click_at(Vec2::new(tab.rect.center().x, tab.rect.center().y));
            d.wait("the second motor", |d| d.state().model.loop_response.motor == 1);
        }
    }
}

/// While the loop is measured a progress bar and a cancel button stand where the run button was; cancelled, the run
/// button is back and nothing of that measurement arrives.
#[test]
fn the_loop_measurement_shows_progress_and_is_cancelled() {
    let mut d = Driver::new();
    d.tab("ui-loop-tab");
    d.button_of("ui-loop-run");
    d.wait("some points measured", |d| d.state().model.loop_response.progress().is_some_and(|(done, _)| done > 0));
    let run = d.tr("ui-loop-run");
    assert!(!d.has(Role::Button, &run));
    let said = d.messages().len();
    d.button_of("ui-loop-cancel");
    d.wait("cancelled", |d| !d.state().model.loop_response.running());
    d.wait("the run button back", |d| d.state().model.loop_response.progress().is_none());
    assert!(d.has(Role::Button, &run));
    d.frames(60);
    assert!(d.state().model.loop_response.result.is_none() && d.messages().len() == said, "{:?}", d.messages());
}

/// How much of the 3D view the things floating on it (its controls, the timeline, the tool view) leave free, as a share
/// of its area: what they cover, each counted alone, which counts any overlap twice and so errs on the covered side.
fn view_free(d: &Driver) -> f32 {
    let view = d.regions()[4];
    let covered: f32 =
        d.state().covered.iter().map(|r| r.intersect(view)).filter(|r| r.is_positive()).map(|r| r.area()).sum();
    1.0 - covered / view.area()
}

/// At 1280 x 720, in either language, with no result and with the cycle's: every tab of either panel leaves the panels
/// at their widths (setup 270, tools 430 pixels; a tools tab widened its panel by up to 160 pixels), and the view's
/// controls start folded to one row, so that what floats on the view (controls and timeline) leaves at least 80 % of
/// it free. Opened, they take more; folded again, as little as before.
#[test]
fn at_1280_by_720_the_panels_keep_their_widths_and_the_view_stays_free() {
    for lang in ["en", "de"] {
        let mut d = Driver::of(MACHINES[0], lang);
        d.resize(1280.0, 720.0);
        d.button_of("ui-start-cycle");
        d.wait("the timeline on the view", |d| d.state().recorded.len() > 2 && d.state().covered.len() == 2);
        let setup = ["ui-motion", "ui-effects", "ui-parameters", "ui-config"];
        let tools = [
            "ui-analysis",
            "ui-effect-views",
            "ui-search",
            "ui-path",
            "ui-trace",
            "ui-sweep-tab",
            "ui-tuning",
            "ui-runs",
            "ui-views",
        ];
        let mut problems = Vec::new();
        for result in [false, true] {
            if result {
                d.tab("ui-analysis");
                let label = d.tr("ui-analysis-run-cycle");
                let tools = d.regions()[TOOLS];
                let run = d.find_where("run the cycle", |w| {
                    w.role == Role::Button && w.label == label && tools.contains_rect(w.rect)
                });
                d.click_at(Vec2::new(run.rect.center().x, run.rect.center().y));
                d.wait_result("the cycle's result", |d| d.state().model.cycle_report.is_some());
            }
            for (s, t) in setup.iter().cycle().zip(&tools) {
                d.tab(s);
                d.tab(t);
                let [_, left, right, _, _] = d.regions();
                let free = view_free(&d);
                if (left.width() - 270.0).abs() > 0.5 || (right.width() - 430.0).abs() > 0.5 || free < 0.8 {
                    problems.push(format!(
                        "{lang} {s} {t} result {result}: setup {} tools {} free {free:.2}",
                        left.width(),
                        right.width()
                    ));
                }
            }
        }
        assert!(problems.is_empty(), "{}", problems.join("\n"));
        let folded = view_free(&d);
        d.button_of("ui-view-more");
        d.find(Role::CheckBox, &d.tr("ui-view-xray"));
        d.settle();
        assert!(view_free(&d) < folded - 0.05, "{lang}: opened, {} free; folded, {folded}", view_free(&d));
        d.button_of("ui-view-less");
        d.settle();
        assert!((view_free(&d) - folded).abs() < 1e-3, "{lang}: folded again, {} free", view_free(&d));
    }
}

/// A wheel drawn upright has the cylinder mesh's axis (Bevy's y) along the scene's depth (Bevy's z); laid flat
/// (`Part::lay_flat`), along the scene's z, Bevy's y; a turned spoke laid flat turns about that axis, from +x towards
/// the scene's +y (Bevy's -z): a quarter turn takes its long side to Bevy's -z.
#[test]
fn a_wheel_laid_flat_faces_up() {
    let wheel = Part::new("idler", Shape::Cylinder { radius: 0.1, length: 0.03 }, [0.0; 3], [0.5; 3]);
    let axis = |part: &Part| part_transform(part).rotation * Vec3::Y;
    assert!(axis(&wheel).abs().abs_diff_eq(Vec3::Z, 1e-6), "{}", axis(&wheel));
    let mut flat = wheel.clone();
    flat.lay_flat();
    assert!(axis(&flat).abs().abs_diff_eq(Vec3::Y, 1e-6), "{}", axis(&flat));
    let mut spoke = Part::new("idler-bar", Shape::Box { size: [0.2, 0.004, 0.02] }, [0.0; 3], [0.9; 3])
        .turned(std::f64::consts::FRAC_PI_2);
    spoke.lay_flat();
    let long = part_transform(&spoke).rotation * Vec3::X;
    assert!(long.abs_diff_eq(Vec3::NEG_Z, 1e-6), "{long}");
}

/// A message raised by anything (a file that could not be opened, a failed job) is on screen whichever setup tab or
/// tool tab is open, once and not once per tab; Clear takes it away. It was drawn in the Motion tab only.
#[test]
fn a_message_is_on_screen_on_every_tab() {
    let mut d = Driver::new();
    d.state_mut().model.messages.push(("ui-config-saved", "somewhere.toml".into()));
    let shown = d.state().model.tr.tr_arg("ui-config-saved", "somewhere.toml").to_string();
    for tab in ["ui-motion", "ui-effects", "ui-parameters", "ui-config", "ui-path"] {
        d.tab(tab);
        d.find_where(&format!("the message on {tab}"), |w| w.role == Role::Label && w.value == shown);
        let count = d.widgets().iter().filter(|w| w.role == Role::Label && w.value == shown).count();
        assert_eq!(count, 1, "{tab}");
    }
    d.button_of("ui-clear");
    d.wait("the messages cleared", |d| d.state().model.messages.is_empty());
}

/// A change to a value of the config is some `set_param` away, as a drag makes it.
fn edit_a_parameter(d: &mut Driver) -> (String, f64, f64) {
    let (path, old) = some_parameter(d);
    let new = old * 1.1;
    d.state_mut().model.set_param(&path, toml::Value::Float(new));
    d.wait("the edit marked", |d| d.state().model.dirty());
    (path, old, new)
}

/// The undo and redo buttons and Ctrl+Z, Ctrl+Y and Ctrl+Shift+Z take an edit back and do it again, and the mark
/// "modified" shows while the config differs from what it came from.
#[test]
fn undo_and_redo_by_the_buttons_and_the_keys() {
    let mut d = Driver::new();
    let modified = d.tr("ui-modified");
    assert!(!d.state().model.dirty() && !d.widgets().iter().any(|w| w.value == modified));
    let (path, old, new) = edit_a_parameter(&mut d);
    d.find_where("the mark of edits", |w| w.role == Role::Label && w.value == modified);
    let at = |d: &Driver| d.state().model.config.f64(&path).ok();
    d.button_of("ui-undo");
    d.wait("undone by the button", |d| at(d) == Some(old));
    assert!(!d.state().model.dirty(), "back at what it came from");
    d.eventually("the mark gone", |d| {
        if d.widgets().iter().all(|w| w.value != modified) { Ok(()) } else { Err("still there".into()) }
    });
    d.button_of("ui-redo");
    d.wait("redone by the button", |d| at(d) == Some(new));
    let control = |d: &mut Driver, down| d.hold(KeyCode::ControlLeft, Key::Control, down);
    control(&mut d, true);
    d.key(KeyCode::KeyZ, Key::Character("z".into()));
    control(&mut d, false);
    d.wait("undone by Ctrl+Z", |d| at(d) == Some(old));
    control(&mut d, true);
    d.key(KeyCode::KeyY, Key::Character("y".into()));
    control(&mut d, false);
    d.wait("redone by Ctrl+Y", |d| at(d) == Some(new));
    control(&mut d, true);
    d.key(KeyCode::KeyZ, Key::Character("z".into()));
    d.wait("undone again", |d| at(d) == Some(old));
    d.hold(KeyCode::ShiftLeft, Key::Shift, true);
    d.key(KeyCode::KeyZ, Key::Character("Z".into()));
    d.hold(KeyCode::ShiftLeft, Key::Shift, false);
    control(&mut d, false);
    d.wait("redone by Ctrl+Shift+Z", |d| at(d) == Some(new));
}

/// Switching the machine over edits asks first: Cancel keeps the machine and the edits, Discard switches, Save writes
/// the config to the file the field names and then switches.
#[test]
fn switching_the_machine_over_edits_asks_first() {
    let mut d = Driver::new();
    let (path, _, new) = edit_a_parameter(&mut d);
    let (machine, other) = (d.tr(MACHINES[0].key), d.tr(MACHINES[1].key));
    pick(&mut d, &machine, &other);
    d.wait("the question", |d| d.state().ask.is_some());
    let save = d.tr("ui-unsaved-save");
    d.find(Role::Button, &save);
    d.frames(5);
    assert_eq!(d.state().model.entry.id, MACHINES[0].id, "not switched before the answer");
    d.button_of("ui-unsaved-cancel");
    d.wait("the question gone", |d| d.state().ask.is_none());
    d.frames(5);
    assert_eq!((d.state().model.entry.id, d.state().model.config.f64(&path).ok()), (MACHINES[0].id, Some(new)));
    // Save: to a file of the field's.
    let file = std::env::temp_dir().join(format!("window-unsaved-{}.toml", std::process::id()));
    d.state_mut().model.inputs.config_file = file.display().to_string();
    pick(&mut d, &machine, &other);
    d.button_of("ui-unsaved-save");
    d.wait("the other machine", |d| d.state().model.entry.id == MACHINES[1].id);
    let saved = sim_core::Config::parse(&std::fs::read_to_string(&file).unwrap()).unwrap();
    assert_eq!(saved.f64(&path).unwrap(), new);
    // Discard: back on the first machine with edits, then away from them.
    pick(&mut d, &other, &machine);
    d.wait("the first machine", |d| d.state().model.entry.id == MACHINES[0].id);
    edit_a_parameter(&mut d);
    pick(&mut d, &machine, &other);
    d.button_of("ui-unsaved-discard");
    d.wait("the other machine again", |d| d.state().model.entry.id == MACHINES[1].id);
    assert!(!d.state().model.dirty());
}

/// Closing the window over edits asks first (and does not close it); Cancel goes on, Discard ends the app. Without edits
/// it ends at once, as `closing_the_window_during_an_analysis_does_not_hang` has it.
#[test]
fn closing_the_window_over_edits_asks_first() {
    let mut d = Driver::new();
    edit_a_parameter(&mut d);
    let window = d.window;
    d.send(bevy::window::WindowCloseRequested { window });
    d.wait("the question", |d| d.state().ask.is_some());
    d.frames(5);
    assert!(d.app.should_exit().is_none());
    d.button_of("ui-unsaved-cancel");
    d.wait("the question gone", |d| d.state().ask.is_none());
    d.frames(5);
    assert!(d.app.should_exit().is_none());
    d.send(bevy::window::WindowCloseRequested { window });
    d.button_of("ui-unsaved-discard");
    d.wait("the app exits", |d| d.app.should_exit().is_some());
}

/// Opening a config over edits asks first, with the file typed in the field: Cancel keeps the edits and the old file;
/// Save writes the file the config was last opened from (not the one typed, which is then opened as it was); Discard
/// opens at once. (The button with the file dialog takes the same road: it only names the file another way.)
#[test]
fn opening_a_file_over_edits_asks_first() {
    let mut d = Driver::new();
    d.tab("ui-config");
    let dir = std::env::temp_dir();
    let [old, next] = ["old", "next"].map(|n| dir.join(format!("window-open-{n}-{}.toml", std::process::id())));
    let text = d.state().model.entry.default_config;
    std::fs::write(&old, text).unwrap();
    std::fs::write(&next, text).unwrap();
    let (old, next) = (old.display().to_string(), next.display().to_string());
    let (path, _, new) = edit_a_parameter(&mut d);
    // The config stands for the old file; another path is typed in the field.
    d.state_mut().model.file = old.clone();
    d.state_mut().model.inputs.config_file = next.clone();
    d.button_of("ui-config-open");
    d.wait("the question", |d| d.state().ask.is_some());
    d.frames(5);
    assert_eq!(d.state().model.file, old, "not opened before the answer");
    d.button_of("ui-unsaved-cancel");
    d.wait("the question gone", |d| d.state().ask.is_none());
    d.frames(5);
    let model = &d.state().model;
    assert_eq!((model.file.as_str(), model.config.f64(&path).ok()), (old.as_str(), Some(new)));
    assert!(model.dirty());
    // Save: the old file takes the edits, the one typed stays as it is, and is opened.
    d.button_of("ui-config-open");
    d.wait("the question again", |d| d.state().ask.is_some());
    // The Config tab has a Save button too: the question's is the one outside the setup panel.
    let label = d.tr("ui-unsaved-save");
    let panel: Vec<_> = d.in_region(SETUP, Role::Button).into_iter().map(|w| w.rect).collect();
    let save =
        d.widgets().into_iter().find(|w| w.label == label && !panel.contains(&w.rect)).expect("the question's Save");
    d.click_at(Vec2::new(save.rect.center().x, save.rect.center().y));
    d.wait("the file typed opened", |d| d.state().model.file == next);
    let saved = sim_core::Config::parse(&std::fs::read_to_string(&old).unwrap()).unwrap();
    assert_eq!(saved.f64(&path).unwrap(), new, "the old file has the edits");
    assert_eq!(std::fs::read_to_string(&next).unwrap(), text, "the file to open was written over");
    assert!(!d.state().model.dirty() && d.state().model.config.f64(&path).unwrap() != new);
    assert_eq!(d.state().model.inputs.config_file, next);
    // Discard: opens the file typed, the old one stays as saved.
    edit_a_parameter(&mut d);
    d.state_mut().model.inputs.config_file = old.clone();
    d.button_of("ui-config-open");
    d.wait("the question a third time", |d| d.state().ask.is_some());
    d.button_of("ui-unsaved-discard");
    d.wait("the old file opened", |d| d.state().model.file == old && !d.state().model.dirty());
    assert_eq!(std::fs::read_to_string(&next).unwrap(), text, "the discarded edits were saved");
    assert_eq!(d.state().model.config.f64(&path).unwrap(), new, "the saved file's value");
}

/// A JSON machine file is remembered for the next start: the settings hold its path, `restore` opens it again, and
/// Save (also the one the question over edits asks for) does not take it as the place to write TOML.
#[test]
fn a_json_machine_file_is_remembered_between_starts() {
    fn json(value: &toml::Value) -> String {
        match value {
            toml::Value::Table(t) => {
                let items: Vec<String> =
                    t.iter().map(|(k, v)| format!("{}:{}", json(&k.as_str().into()), json(v))).collect();
                format!("{{{}}}", items.join(","))
            }
            toml::Value::Array(a) => format!("[{}]", a.iter().map(json).collect::<Vec<_>>().join(",")),
            toml::Value::String(s) => format!("{s:?}"),
            other => other.to_string(),
        }
    }
    let shipped = concat!(env!("CARGO_MANIFEST_DIR"), "/../../machines/h-bot.toml");
    let table: toml::Table = std::fs::read_to_string(shipped).unwrap().parse().unwrap();
    let file = std::env::temp_dir().join(format!("window-machine-{}.json", std::process::id()));
    std::fs::write(&file, json(&toml::Value::Table(table))).unwrap();
    let file = file.display().to_string();
    let mut d = Driver::new();
    d.state_mut().model.open_config(&file);
    let model = &d.state().model;
    assert_eq!((model.entry.id, model.file.as_str()), ("h-bot", file.as_str()), "{:?}", model.messages);
    assert_eq!(model.inputs.config_file, "", "Save would write TOML over the JSON file");
    assert_eq!(model.save_target(), None);
    // The dialog Save then asks with starts beside it, as TOML; with no file, nowhere (not at the field's file).
    assert_eq!(model.save_suggestion(), file.replace(".json", ".toml"));
    assert_eq!(Model::new(MACHINES[0], "en").save_suggestion(), "");
    let stored = settings_of(d.state());
    assert_eq!(stored.config_file.as_deref(), Some(file.as_str()));
    // The next start: another machine shown, the file opened over it.
    let mut again = Model::new(MACHINES[0], "en");
    restore(&mut again, &stored);
    assert_eq!((again.entry.id, again.file.as_str()), ("h-bot", file.as_str()), "{:?}", again.messages);
    assert!(!again.dirty());
}

/// The filter narrows the parameters to those whose label or dotted path hold it: with a path typed, its group is
/// there and another is not.
#[test]
fn the_parameter_filter_narrows_the_list() {
    let mut d = Driver::new();
    d.tab("ui-parameters");
    let (path, _) = some_parameter(&d);
    let groups = d.state().model.param_groups();
    let own = groups.iter().find(|g| g.params.iter().any(|(p, ..)| *p == path)).unwrap();
    let other = groups.iter().find(|g| g.key != own.key).expect("a second group");
    let (own, other) = (d.tr(&own.key), d.tr(&other.key));
    d.find(Role::Button, &other);
    let fields = d.in_region(SETUP, Role::TextInput);
    d.type_into(fields[1].rect, &path.to_uppercase());
    d.wait("the filter typed", |d| d.state().model.inputs.param_filter == path.to_uppercase());
    d.eventually("only the group with the match", |d| {
        let (has_own, has_other) = (d.has(Role::Button, &own), d.has(Role::Button, &other));
        if has_own && !has_other { Ok(()) } else { Err(format!("own {has_own}, other {has_other}")) }
    });
}

/// The catalogue list of a part names the articles; picking one puts it in the field, which stays typeable.
#[test]
fn a_motor_is_picked_from_the_catalogue_list() {
    let mut d = Driver::new();
    d.tab("ui-config");
    let supply = d.state().model.inputs.parts[1].clone();
    let first = sim_components::catalog::motor_articles_for(&supply)[0];
    let list = d.in_region(SETUP, Role::ComboBox).into_iter().next().expect("the motor's list");
    d.click_at(Vec2::new(list.rect.center().x, list.rect.center().y));
    d.click(Role::Button, first);
    d.wait("the article in the field", |d| d.state().model.inputs.parts[0] == first);
}

/// The belt has a catalogue list too (it had none): the fifth of the parts' lists, from the top, names the belts by
/// profile and width; a name picked lands in the field.
#[test]
fn a_belt_is_picked_from_the_catalogue_list() {
    let mut d = Driver::new();
    d.tab("ui-config");
    // The first: the list scrolls after a dozen, and a row out of sight cannot be clicked.
    let belt = sim_components::catalog::belt_names()[0].clone();
    let lists = d.in_region(SETUP, Role::ComboBox);
    let list = lists.get(4).expect("the belt's list");
    d.click_at(Vec2::new(list.rect.center().x, list.rect.center().y));
    d.click(Role::Button, &belt);
    d.wait("the belt in the field", |d| d.state().model.inputs.parts[4] == belt);
}
