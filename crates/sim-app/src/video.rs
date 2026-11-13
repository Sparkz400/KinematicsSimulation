//! Recording the 3D view: numbered PNG frames in `recordings/<date-time>/` under the working directory, made into
//! `recordings/<date-time>.mp4` by Blender where it is installed. Blender is a big, optional program, so it is
//! looked for, not depended on; without it the frames stay for any other encoder.

use crate::model::Message;
use bevy::ecs::observer::On;
use bevy::render::view::screenshot::ScreenshotCaptured;
use std::ops::Range;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::sync::mpsc;
use std::thread::JoinHandle;
use std::time::{Instant, SystemTime, UNIX_EPOCH};

/// Frames per second of real time, in the frames and in the video.
pub const FPS: f64 = 30.0;

/// The Blender script that encodes the frames, carried in the program so that it works away from the sources too.
const SCRIPT: &str = include_str!("../tools/frames_to_mp4.py");

/// A cropped frame, which saves itself into the folder given under each of its numbers.
type Frame = Box<dyn FnOnce(&Path) -> Result<(), String> + Send>;

/// A recording in progress. Frames are written on a thread of their own, since encoding a PNG takes about as long
/// as drawing a frame; that thread then runs Blender, so the window never waits for either.
pub struct Recording {
    pub dir: PathBuf,
    started: Instant,
    next: u64,
    frames: mpsc::Sender<Frame>,
    writer: JoinHandle<Message>,
}

/// The video being made from a stopped recording; its message comes once Blender is done.
pub struct Encoding(Option<JoinHandle<Message>>);

impl Encoding {
    /// The message, once, when the video is made or given up on.
    pub fn poll(&mut self) -> Option<Message> {
        let done = self.0.take_if(|writer| writer.is_finished())?;
        Some(done.join().unwrap_or_else(|_| ("ui-video-refused", String::new())))
    }
}

impl Recording {
    /// Starts a recording into a new folder named after the time now (UTC: the standard library knows no time zone).
    pub fn start() -> Result<Self, Message> {
        // Absolute: Blender reads relative paths from elsewhere, and the messages say where the files are.
        let refused = |e: std::io::Error| ("ui-video-refused", e.to_string());
        let dir = std::path::absolute(Path::new("recordings").join(timestamp(SystemTime::now()))).map_err(refused)?;
        std::fs::create_dir_all(&dir).map_err(refused)?;
        let (frames, received) = mpsc::channel::<Frame>();
        let folder = dir.clone();
        let writer = std::thread::spawn(move || {
            let mut failed = None;
            // Ends when every sender is gone: the recording's, and those of screenshots still on their way.
            for frame in received {
                if let Err(why) = frame(&folder) {
                    failed.get_or_insert(why);
                }
            }
            match failed {
                Some(why) => ("ui-video-refused", why),
                None => encode(&folder),
            }
        });
        Ok(Self { dir, started: Instant::now(), next: 0, frames, writer })
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

    /// What saves a screenshot of the window as the frames `numbers`, cut to `crop` (left, top, width, height in
    /// physical pixels), for `Screenshot`'s `observe`.
    pub fn saver(&self, crop: [u32; 4], numbers: Range<u64>) -> impl FnMut(On<ScreenshotCaptured>) + use<> {
        let frames = self.frames.clone();
        move |captured| {
            let Ok(image) = captured.image.clone().try_into_dynamic() else { return };
            let [left, top, width, height] = crop;
            // H.264 stores colour at half resolution, so it needs an even width and height.
            let picture = image.crop_imm(left, top, width & !1, height & !1).to_rgb8();
            let numbers = numbers.clone();
            let save = move |folder: &Path| numbers.map(|n| picture.save(folder.join(format!("{n:06}.png"))).map_err(|e| e.to_string())).collect();
            let _ = frames.send(Box::new(save));
        }
    }

    /// Stops recording; the video is made once the last frames are written.
    pub fn stop(self) -> Encoding {
        Encoding(Some(self.writer))
    }
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
    if let Some(path) = std::env::var_os("BLENDER") {
        return Some(path.into());
    }
    let installs = std::fs::read_dir(r"C:\Program Files\Blender Foundation").ok()?;
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
}
