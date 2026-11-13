//! Recording the 3D view: numbered PNG frames in `recordings/<date-time>/` under the working directory, made into
//! `recordings/<date-time>.mp4` by Blender where it is installed. Blender is a big, optional program, so it is
//! looked for, not depended on; without it the frames stay for any other encoder.

use crate::model::Message;
use bevy::asset::RenderAssetUsages;
use bevy::ecs::observer::On;
use bevy::image::Image;
use bevy::render::render_resource::{Extent3d, TextureDimension, TextureFormat};
use bevy::render::view::screenshot::ScreenshotCaptured;
use std::ffi::OsString;
use std::ops::Range;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, mpsc};
use std::thread::JoinHandle;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

/// Frames per second of real time, in the frames and in the video.
pub const FPS: f64 = 30.0;

/// The Blender script that encodes the frames, carried in the program so that it works away from the sources too.
const SCRIPT: &str = include_str!("../tools/frames_to_mp4.py");

/// A cropped frame, which saves itself into the folder given under each of its numbers.
type Frame = Box<dyn FnOnce(&Path) -> Result<(), String> + Send>;

/// Frames waiting for the writer at most. Where the disk cannot keep up, more would only grow the memory (a frame
/// is some megabytes) or, waited for, hold up the window; past this many a frame is dropped and counted instead.
const QUEUE: usize = 8;

/// A recording in progress. Frames are written on a thread of their own, since encoding a PNG takes about as long
/// as drawing a frame; that thread then runs Blender, so the window never waits for either.
pub struct Recording {
    pub dir: PathBuf,
    started: Instant,
    next: u64,
    /// The size every frame is cut to, fixed at the first: Blender makes the video at the first frame's size.
    size: Option<[u32; 2]>,
    frames: mpsc::SyncSender<Frame>,
    /// Frame numbers dropped since the queue was full.
    dropped: Arc<AtomicU64>,
    /// The app is closing: the writer stops at an empty queue rather than waiting for screenshots never taken.
    closing: Arc<AtomicBool>,
    writer: JoinHandle<Vec<Message>>,
}

/// The video being made from a stopped recording; its messages come once Blender is done.
pub struct Encoding {
    video: PathBuf,
    closing: Arc<AtomicBool>,
    writer: Option<JoinHandle<Vec<Message>>>,
}

impl Encoding {
    /// The messages, once, when the video is made or given up on.
    pub fn poll(&mut self) -> Option<Vec<Message>> {
        let done = self.writer.take_if(|writer| writer.is_finished())?;
        Some(done.join().unwrap_or_else(|_| vec![("ui-video-refused", String::new())]))
    }

    /// At the app's exit: the frames still queued are written, and the video waited for until `deadline`. Blender
    /// still working then is left to finish on its own, which it does without the app (it reads only the frames,
    /// all written by then); killing it would lose a video that is nearly made, and waiting longer would hold the
    /// closing window for minutes. The message says so, and where the video will be.
    pub fn finish(mut self, deadline: Instant) -> Vec<Message> {
        self.closing.store(true, Ordering::Relaxed);
        loop {
            if let Some(messages) = self.poll() {
                return messages;
            }
            if Instant::now() >= deadline {
                return vec![("ui-video-detached", self.video.display().to_string())];
            }
            std::thread::sleep(Duration::from_millis(20));
        }
    }
}

impl Recording {
    /// Starts a recording into a new folder named after the time now (UTC: the standard library knows no time zone).
    pub fn start() -> Result<Self, Message> {
        // Absolute: Blender reads relative paths from elsewhere, and the messages say where the files are.
        let refused = |e: std::io::Error| ("ui-video-refused", e.to_string());
        let dir = new_folder(&std::path::absolute("recordings").map_err(refused)?, SystemTime::now()).map_err(refused)?;
        let (frames, received) = mpsc::sync_channel::<Frame>(QUEUE);
        let (dropped, closing) = (Arc::new(AtomicU64::new(0)), Arc::new(AtomicBool::new(false)));
        let (folder, lost, close) = (dir.clone(), dropped.clone(), closing.clone());
        let writer = std::thread::spawn(move || {
            let mut failed = None;
            // Ends when every sender is gone: the recording's, and those of screenshots still on their way. At the
            // app's exit, once the queue is empty: the screenshots not yet taken then never come.
            loop {
                let frame = match received.recv_timeout(Duration::from_millis(50)) {
                    Ok(frame) => frame,
                    Err(mpsc::RecvTimeoutError::Timeout) if !close.load(Ordering::Relaxed) => continue,
                    Err(_) => break,
                };
                if let Err(why) = frame(&folder) {
                    failed.get_or_insert(why);
                }
            }
            let mut messages = vec![match failed {
                Some(why) => ("ui-video-refused", why),
                None => encode(&folder),
            }];
            match lost.load(Ordering::Relaxed) {
                0 => {}
                n => messages.push(("ui-video-dropped", n.to_string())),
            }
            messages
        });
        Ok(Self { dir, started: Instant::now(), next: 0, size: None, frames, dropped, closing, writer })
    }

    /// Seconds since the recording started.
    pub fn seconds(&self) -> f64 {
        self.started.elapsed().as_secs_f64()
    }

    /// The frame numbers due by now, at `FPS` of real time. Empty where the window draws faster than that: the
    /// frame is skipped. More than one where it draws slower: the one picture is repeated under each, so the video
    /// keeps real time.
    pub fn due(&mut self) -> Range<u64> {
        let until = (self.seconds() * FPS) as u64 + 1;
        let due = self.next..until.max(self.next);
        self.next = due.end;
        due
    }

    /// What saves a screenshot of the window as the frames `numbers`, cut to the view at `view` (left, top, width,
    /// height in physical pixels), for `Screenshot`'s `observe`. Every frame has the first one's size, made even
    /// (H.264 stores colour at half resolution), however the window or the panels change meanwhile (`cut`); `fill`
    /// is the window's clear colour, for where the view got smaller.
    pub fn saver(&mut self, view: [u32; 4], fill: [u8; 4], numbers: Range<u64>) -> impl FnMut(On<ScreenshotCaptured>) + use<> {
        let size = *self.size.get_or_insert([view[2] & !1, view[3] & !1]);
        let (frames, dropped) = (self.frames.clone(), self.dropped.clone());
        move |captured| {
            let Ok(window) = captured.image.clone().try_into_dynamic() else { return };
            let window = window.to_rgba8();
            let pixels = cut(&window, [window.width(), window.height()], view, size, fill);
            let extent = Extent3d { width: size[0], height: size[1], depth_or_array_layers: 1 };
            let picture = Image::new(extent, TextureDimension::D2, pixels, TextureFormat::Rgba8UnormSrgb, RenderAssetUsages::default());
            let Ok(picture) = picture.try_into_dynamic() else { return };
            let picture = picture.to_rgb8();
            let numbers = numbers.clone();
            let count = numbers.end - numbers.start;
            let save = move |folder: &Path| save_numbered(folder, numbers, |file| picture.save(file).map_err(|e| e.to_string()));
            // Never waits: a full queue would hold up the window until the disk caught up.
            if frames.try_send(Box::new(save)).is_err() {
                dropped.fetch_add(count, Ordering::Relaxed);
            }
        }
    }

    /// Stops recording; the video is made once the last frames are written.
    pub fn stop(self) -> Encoding {
        Encoding { video: self.dir.with_extension("mp4"), closing: self.closing, writer: Some(self.writer) }
    }
}

/// A new folder in `parent`, named after `now`; a recording started in the same second gets "_2", "_3" and so on,
/// since its frames and video would otherwise go over the first one's.
fn new_folder(parent: &Path, now: SystemTime) -> std::io::Result<PathBuf> {
    std::fs::create_dir_all(parent)?;
    let name = timestamp(now);
    let mut n = 1;
    loop {
        let dir = parent.join(if n == 1 { name.clone() } else { format!("{name}_{n}") });
        // `create_dir`, not `create_dir_all`: it fails on a folder already there, which is how a taken name shows.
        match std::fs::create_dir(&dir) {
            Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists => n += 1,
            made => return made.map(|()| dir),
        }
    }
}

/// The view at `view` (left, top, width, height) in a window's RGBA picture `[width, height]` across, cut to `size`:
/// from the view's top left, as much of the view as fits in `size` and in the picture, the rest `fill`.
fn cut(window: &[u8], [width, height]: [u32; 2], [left, top, view_width, view_height]: [u32; 4], size: [u32; 2], fill: [u8; 4]) -> Vec<u8> {
    let (left, top) = (left.min(width) as usize, top.min(height) as usize);
    let (width, size_x) = (width as usize, size[0] as usize);
    let across = (view_width as usize).min(size_x).min(width - left);
    let down = (view_height.min(size[1]) as usize).min(height as usize - top);
    let mut out = fill.repeat(size_x * size[1] as usize);
    for y in 0..down {
        let from = ((top + y) * width + left) * 4;
        out[y * size_x * 4..][..across * 4].copy_from_slice(&window[from..][..across * 4]);
    }
    out
}

/// Saves one picture as each of the frames `numbers` in `folder`: encoded once by `write`, then copied, since a
/// repeated frame is the same file and a copy is far quicker than encoding it again.
fn save_numbered(folder: &Path, numbers: Range<u64>, write: impl FnOnce(&Path) -> Result<(), String>) -> Result<(), String> {
    let file = |n: u64| folder.join(format!("{n:06}.png"));
    let first = file(numbers.start);
    write(&first)?;
    for n in numbers.skip(1) {
        std::fs::copy(&first, file(n)).map_err(|e| e.to_string())?;
    }
    Ok(())
}

/// Runs Blender on the frames in `folder`, into `folder` with ".mp4"; says where the video is, or where the frames
/// are where there is no Blender or it failed.
fn encode(folder: &Path) -> Message {
    let frames = ("ui-video-frames", folder.display().to_string());
    let Some(blender) = blender() else { return frames };
    let script = folder.with_file_name("frames_to_mp4.py");
    let video = folder.with_extension("mp4");
    if std::fs::write(&script, SCRIPT).is_err() {
        return frames;
    }
    let mut command = Command::new(blender);
    command.args(["--background", "--factory-startup", "--python"]).arg(&script).arg("--").arg(folder).arg(&video);
    command.stdin(Stdio::null()).stdout(Stdio::null()).stderr(Stdio::null());
    #[cfg(windows)]
    {
        // No console window flashing up over the app.
        use std::os::windows::process::CommandExt;
        command.creation_flags(0x0800_0000);
    }
    match command.status() {
        Ok(status) if status.success() && video.is_file() => ("ui-video-saved", video.display().to_string()),
        _ => ("ui-video-failed", folder.display().to_string()),
    }
}

/// Blender: the environment variable BLENDER, else the newest of the standard installs on Windows.
fn blender() -> Option<PathBuf> {
    find_blender(std::env::var_os("BLENDER"), Path::new(r"C:\Program Files\Blender Foundation"))
}

/// `named` where that is a file, else the newest Blender in `installs`. A BLENDER naming no file (an install moved
/// or removed) is passed over, so that the message says no Blender was found rather than that Blender failed.
fn find_blender(named: Option<OsString>, installs: &Path) -> Option<PathBuf> {
    if let Some(path) = named.map(PathBuf::from).filter(|path| path.is_file()) {
        return Some(path);
    }
    let installs = std::fs::read_dir(installs).ok()?;
    let found = installs.flatten().filter_map(|entry| {
        let name = entry.file_name().into_string().ok()?;
        // "Blender 5.2" → [5, 2], so that 5.10 counts as newer than 5.9.
        let version: Vec<u32> = name.strip_prefix("Blender ")?.split('.').map(|n| n.parse().ok()).collect::<Option<_>>()?;
        let exe = entry.path().join("blender.exe");
        exe.is_file().then_some((version, exe))
    });
    found.max_by(|a, b| a.0.cmp(&b.0)).map(|(_, exe)| exe)
}

/// "2026-11-13_08-15-30" for a time, in UTC.
fn timestamp(time: SystemTime) -> String {
    let seconds = time.duration_since(UNIX_EPOCH).map_or(0, |d| d.as_secs());
    let (days, rest) = (seconds / 86_400, seconds % 86_400);
    // Days since 1970 to the civil date (Howard Hinnant's algorithm).
    let z = days as i64 + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let day = doy - (153 * mp + 2) / 5 + 1;
    let month = if mp < 10 { mp + 3 } else { mp - 9 };
    let year = yoe + era * 400 + i64::from(month <= 2);
    format!("{year:04}-{month:02}-{day:02}_{:02}-{:02}-{:02}", rest / 3600, rest % 3600 / 60, rest % 60)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    #[test]
    fn names_the_folder_after_the_date_and_time() {
        assert_eq!(timestamp(UNIX_EPOCH), "1970-01-01_00-00-00");
        // 2024-02-29 23:59:58, a leap day.
        assert_eq!(timestamp(UNIX_EPOCH + Duration::from_secs(1_709_251_198)), "2024-02-29_23-59-58");
    }

    /// A folder of its own for the test, empty.
    fn scratch(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("sim-app-video-{name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    /// Two recordings started in the same second get folders of their own (and so videos of their own); they used
    /// to share one, the second writing over the first one's frames.
    #[test]
    fn recordings_started_in_one_second_get_folders_of_their_own() {
        let parent = scratch("folders");
        let now = UNIX_EPOCH + Duration::from_millis(1_709_251_198_250);
        let first = new_folder(&parent, now).unwrap();
        let second = new_folder(&parent, now + Duration::from_millis(500)).unwrap();
        let third = new_folder(&parent, now).unwrap();
        assert_eq!(first, parent.join("2024-02-29_23-59-58"));
        assert_eq!(second, parent.join("2024-02-29_23-59-58_2"));
        assert_eq!(third, parent.join("2024-02-29_23-59-58_3"));
        assert_ne!(first.with_extension("mp4"), second.with_extension("mp4"));
        std::fs::remove_dir_all(&parent).unwrap();
    }

    /// Every frame is cut to the size the first one fixed, even, whether the view later grew, shrank or moved, or
    /// the window got narrower than the cut: the view from its top left, the rest the fill.
    #[test]
    fn frames_keep_the_first_ones_even_size() {
        // A window whose every pixel tells where it is.
        let window = |w: u32, h: u32| (0..h).flat_map(|y| (0..w).flat_map(move |x| [x as u8, y as u8, 0, 255])).collect::<Vec<u8>>();
        let pixel = |out: &[u8], x: usize, y: usize| <[u8; 4]>::try_from(&out[(y * 4 + x) * 4..][..4]).unwrap();
        let fill = [9, 9, 9, 255];
        // The size as `saver` fixes it: the first view's, made even.
        let mut fixed = None;
        let mut size = |view: [u32; 4]| *fixed.get_or_insert([view[2] & !1, view[3] & !1]);
        // The first view, 5 by 3 at (1, 2) in a window 8 by 6: the cut is 4 by 2.
        let first = [1, 2, 5, 3];
        assert_eq!(size(first), [4, 2]);
        let out = cut(&window(8, 6), [8, 6], first, [4, 2], fill);
        assert_eq!((out.len(), pixel(&out, 0, 0), pixel(&out, 3, 1)), (4 * 2 * 4, [1, 2, 0, 255], [4, 3, 0, 255]));
        // Grown and moved: the same size, from the new top left.
        let grown = [0, 1, 7, 5];
        assert_eq!(size(grown), [4, 2]);
        let out = cut(&window(8, 6), [8, 6], grown, [4, 2], fill);
        assert_eq!((out.len(), pixel(&out, 0, 0)), (32, [0, 1, 0, 255]));
        // Shrunk to 3 by 1: padded right and below.
        let out = cut(&window(8, 6), [8, 6], [2, 2, 3, 1], [4, 2], fill);
        assert_eq!((out.len(), pixel(&out, 2, 0), pixel(&out, 3, 0), pixel(&out, 0, 1)), (32, [4, 2, 0, 255], fill, fill));
        // The window itself narrower than the cut, and a view reaching past it: nothing read outside the picture.
        let out = cut(&window(3, 1), [3, 1], [1, 0, 9, 9], [4, 2], fill);
        assert_eq!((out.len(), pixel(&out, 1, 0), pixel(&out, 2, 0)), (32, [2, 0, 0, 255], fill));
    }

    /// A BLENDER naming no file falls back to the search; it used to be run and to fail, saying that Blender could
    /// not make the video rather than that none was found.
    #[test]
    fn a_missing_blender_variable_falls_back_to_the_search() {
        let installs = scratch("blender");
        for version in ["Blender 5.9", "Blender 5.10", "Blender 4.2", "Other"] {
            std::fs::create_dir_all(installs.join(version)).unwrap();
            std::fs::write(installs.join(version).join("blender.exe"), b"").unwrap();
        }
        let newest = installs.join("Blender 5.10").join("blender.exe");
        assert_eq!(find_blender(Some(installs.join("gone.exe").into()), &installs), Some(newest.clone()));
        assert_eq!(find_blender(None, &installs), Some(newest));
        let named = installs.join("Blender 4.2").join("blender.exe");
        assert_eq!(find_blender(Some(named.clone().into()), &installs), Some(named));
        assert_eq!(find_blender(Some(installs.join("gone.exe").into()), &installs.join("none")), None);
        std::fs::remove_dir_all(&installs).unwrap();
    }

    /// A frame repeated (the window drew slower than the video's frame rate) is encoded once and copied.
    #[test]
    fn a_repeated_frame_is_a_copy() {
        let folder = scratch("repeat");
        let mut writes = 0;
        save_numbered(&folder, 7..10, |file| {
            writes += 1;
            std::fs::write(file, b"picture").map_err(|e| e.to_string())
        })
        .unwrap();
        assert_eq!(writes, 1);
        for n in 7..10 {
            assert_eq!(std::fs::read(folder.join(format!("{n:06}.png"))).unwrap(), b"picture");
        }
        assert!(!folder.join("000010.png").exists());
        std::fs::remove_dir_all(&folder).unwrap();
    }
}
