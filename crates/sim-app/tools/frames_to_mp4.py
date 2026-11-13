"""Encodes the 3D view's recorded frames into an H.264 video at 30 frames per second, through Blender's video
sequencer and its FFmpeg output. The app runs it on stopping a recording:

    blender --background --factory-startup --python frames_to_mp4.py -- <folder of numbered PNGs> <video.mp4>

Written for Blender 5.2: the sequencer's strips are `sequence_editor.strips` (`sequences` before 4.4) and video
output needs `image_settings.media_type = "VIDEO"` (5.0 on). Exits with 1 on any failure, so the app can say
where the frames are instead.
"""
import os
import sys

import bpy


def main():
    # Absolute: Blender does not read relative paths from the working folder.
    frames_dir, out_path = (os.path.abspath(p) for p in sys.argv[sys.argv.index("--") + 1 :][:2])
    frames = sorted(f for f in os.listdir(frames_dir) if f.lower().endswith(".png"))
    if not frames:
        raise SystemExit("no frames in " + frames_dir)
    first = bpy.data.images.load(os.path.join(frames_dir, frames[0]))
    width, height = first.size
    scene = bpy.context.scene
    render = scene.render
    # H.264 in 4:2:0 needs even sizes. The app cuts every frame to the first one's even size; this guards frames
    # made otherwise.
    render.resolution_x, render.resolution_y = width - width % 2, height - height % 2
    render.resolution_percentage = 100
    render.fps, render.fps_base = 30, 1.0
    scene.frame_start, scene.frame_end = 1, len(frames)
    # The frames are already tone mapped by the app: pass their colours through unchanged.
    scene.view_settings.view_transform = "Standard"
    scene.view_settings.look = "None"
    strips = scene.sequence_editor_create().strips
    strip = strips.new_image(name="frames", filepath=os.path.join(frames_dir, frames[0]), channel=1, frame_start=1)
    for name in frames[1:]:
        strip.elements.append(name)
    render.use_sequencer = True
    settings = render.image_settings
    settings.media_type = "VIDEO"
    settings.file_format = "FFMPEG"
    render.ffmpeg.format = "MPEG4"
    render.ffmpeg.codec = "H264"
    render.ffmpeg.constant_rate_factor = "HIGH"
    render.ffmpeg.audio_codec = "NONE"
    # Without a '#' in the name Blender appends the frame range; render to a stem and rename after.
    stem = os.path.splitext(out_path)[0]
    render.filepath = stem + "_####"
    render.use_file_extension = True
    bpy.ops.render.render(animation=True)
    folder = os.path.dirname(stem)
    made = [f for f in os.listdir(folder) if f.startswith(os.path.basename(stem) + "_") and f.endswith(".mp4")]
    if not made:
        raise SystemExit("Blender wrote no video")
    os.replace(os.path.join(folder, made[0]), out_path)
    print("VIDEO", out_path)


try:
    main()
except BaseException as e:  # noqa: BLE001 - any failure is reported through the exit code
    print("FAILED", e, file=sys.stderr)
    sys.stdout.flush()
    os._exit(1)
