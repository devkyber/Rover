"""
The driving camera: H.264 straight from the camera, one compressed frame
at a time.

    cam = camera.Camera(on_frame=link.publish_video, on_event=link.event)
    cam.start()
    cam.fps, cam.mbps          # measured over the last second

WHY NOTHING IS ENCODED HERE
---------------------------
The Arducam B0201 compresses H.264 itself and offers it on its own video
node (config.CAMERA_DEVICE). The Orin Nano has no hardware encoder, so
compressing on the Jetson would cost most of a CPU core; forwarding the
camera's own stream costs about 5% of one. Measured 2026-10-04 at
1920x1080: 30 frames/s, 10 Mbit/s, the same bitrate as at 640x360 -- the
camera's bitrate is fixed. docs/11 section 4.2.

GStreamer does the capture. Its `appsink` hands over exactly one whole
frame per pull, which is what the station's decoder wants: a frame that is
cut in the wrong place cannot be decoded.

If the camera is unplugged the reader keeps retrying, so plugging it back
in brings the picture back without restarting the rover.
"""
import subprocess
import threading
import time

import config

try:
    import gi
    gi.require_version("Gst", "1.0")
    from gi.repository import Gst
except (ImportError, ValueError):       # Windows bench, or GStreamer bindings missing
    Gst = None

PIPELINE = (
    "v4l2src device={device} ! "
    "video/x-h264,width={width},height={height},framerate={fps}/1 ! "
    # config-interval=-1 repeats the stream's settings (SPS/PPS) in front of
    # every keyframe, so a station that connects mid-stream can start there.
    "h264parse config-interval=-1 ! "
    "video/x-h264,stream-format=byte-stream,alignment=au ! "
    # drop=true: if we ever fall behind, lose old frames, not freshness.
    "appsink name=sink sync=false max-buffers=4 drop=true"
)

# A keyframe (IDR picture) is NAL unit type 5. Its header byte is 0x65, 0x45
# or 0x25 depending on a priority field. The 00 00 01 start code can never
# occur inside a frame's own data -- H.264 escapes it -- so finding this
# pattern anywhere means the frame really contains a keyframe.
_KEYFRAME_MARKS = (b"\x00\x00\x01\x65", b"\x00\x00\x01\x45", b"\x00\x00\x01\x25")

FRAME_TIMEOUT_S = 3      # no frame for this long = camera gone, rebuild the pipeline
RETRY_S = 2

# Values of the camera's `power_line_frequency` control, by mains frequency.
_FLICKER_FILTER = {50: 1, 60: 2}


def available():
    return Gst is not None


def set_flicker_filter():
    """
    Tell the camera which mains frequency the lights run on. True if it took.

    Lights on 60 Hz mains flicker 120 times a second. The sensor exposes its
    rows one after another, so each row catches a different part of that
    flicker unless the exposure time is a whole number of flicker periods.
    With this control set, the camera's auto-exposure only picks such times.
    Set wrong, the video shows light and dark bands that drift slowly up or
    down. config.CAMERA_MAINS_HZ has the measurement.

    Done with v4l2-ctl (from v4l-utils) because the control belongs to the
    camera, not to the video stream: it can be set whether or not the
    stream is running.
    """
    value = _FLICKER_FILTER[config.CAMERA_MAINS_HZ]
    try:
        subprocess.run(["v4l2-ctl", "-d", config.CAMERA_DEVICE,
                        "-c", f"power_line_frequency={value}"],
                       check=True, capture_output=True, timeout=5)
        return True
    except (OSError, subprocess.SubprocessError):
        return False     # camera unplugged, or v4l2-ctl not installed


class Camera:
    def __init__(self, on_frame, on_event=print):
        """on_frame(data: bytes, is_key: bool) is called from the camera thread."""
        if Gst is None:
            raise RuntimeError("GStreamer Python bindings (python3-gi) not found")
        Gst.init(None)
        self.on_frame = on_frame
        self.on_event = on_event
        self.fps = 0.0
        self.mbps = 0.0
        self._running = False
        self._streaming = False
        self._flicker_filter_set = False

    def start(self):
        self._running = True
        threading.Thread(target=self._run, daemon=True).start()

    def stop(self):
        self._running = False

    def _run(self):
        launch = PIPELINE.format(device=config.CAMERA_DEVICE,
                                 width=config.CAMERA_WIDTH,
                                 height=config.CAMERA_HEIGHT,
                                 fps=config.CAMERA_FPS)
        while self._running:
            # Every time the stream is (re)opened, not once at start: the
            # camera forgets the setting when it is unplugged.
            self._flicker_filter_set = set_flicker_filter()
            pipeline = Gst.parse_launch(launch)
            pipeline.set_state(Gst.State.PLAYING)
            try:
                self._pump(pipeline.get_by_name("sink"))
            finally:
                pipeline.set_state(Gst.State.NULL)
            if self._streaming:
                self._streaming = False
                self.fps = self.mbps = 0.0
                self.on_event("camera: stream lost, retrying")
            if self._running:
                time.sleep(RETRY_S)

    def _pump(self, sink):
        """Hand frames over until the camera stops delivering them."""
        frames = size = 0
        window_start = time.monotonic()
        while self._running:
            sample = sink.emit("try-pull-sample", FRAME_TIMEOUT_S * Gst.SECOND)
            if sample is None:
                return
            buffer = sample.get_buffer()
            data = buffer.extract_dup(0, buffer.get_size())
            self.on_frame(data, any(mark in data for mark in _KEYFRAME_MARKS))

            if not self._streaming:
                self._streaming = True
                self.on_event(f"camera: streaming {config.CAMERA_WIDTH}x"
                              f"{config.CAMERA_HEIGHT} H.264"
                              + ("" if self._flicker_filter_set else
                                 f" -- could NOT set the {config.CAMERA_MAINS_HZ} Hz "
                                 "flicker filter (v4l2-ctl); expect moving bands indoors"))
            frames += 1
            size += len(data)
            elapsed = time.monotonic() - window_start
            if elapsed >= 1.0:
                self.fps = frames / elapsed
                self.mbps = size * 8 / elapsed / 1e6
                frames = size = 0
                window_start = time.monotonic()
