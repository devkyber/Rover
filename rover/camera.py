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
from collections import deque
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
    # Acquisition serves recording. Preview has its own bounded handoff below.
    "appsink name=sink sync=false max-buffers={capture_buffers} drop=false"
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
    def __init__(self, on_frame, on_event=print, on_record_frame=None):
        """on_frame(data: bytes, is_key: bool) is called from the camera thread."""
        if Gst is None:
            raise RuntimeError("GStreamer Python bindings (python3-gi) not found")
        Gst.init(None)
        self.on_frame = on_frame
        self.on_event = on_event
        self.on_record_frame = on_record_frame
        self._generation = 0
        self.fps = 0.0
        self.mbps = 0.0
        self._running = False
        self._streaming = False
        self._flicker_filter_set = False
        self._delivery = threading.Condition()
        self._preview = deque()
        self._preview_need_key = True
        self._pending_event = None
        self.preview_dropped = self.delivery_errors = self.record_errors = 0
        self.error = None

    def start(self):
        if self._running:
            return True
        self._running = True
        try:
            threading.Thread(target=self._deliver, name='camera-preview', daemon=True).start()
            threading.Thread(target=self._run, daemon=True).start()
        except Exception as error:
            self.error = f'Camera worker start failed: {type(error).__name__}: {error}'
            self.stop()
            return False
        return True

    def stop(self):
        self._running = False
        with self._delivery:
            self._delivery.notify()

    def _event(self, text):
        # Console/event callbacks may block; acquisition must keep draining.
        with self._delivery:
            self._pending_event = text
            self._delivery.notify()

    def _queue_preview(self, data, key):
        with self._delivery:
            if len(self._preview) >= config.CAMERA_NETWORK_QUEUE_FRAMES:
                self.preview_dropped += len(self._preview)
                self._preview.clear()
                self._preview_need_key = True
            if self._preview_need_key and not key:
                self.preview_dropped += 1
                return
            self._preview_need_key = False
            self._preview.append((data, key))
            self._delivery.notify()

    def _deliver(self):
        while True:
            with self._delivery:
                self._delivery.wait_for(lambda: not self._running or self._pending_event is not None or self._preview)
                if not self._running:
                    return
                if self._pending_event is not None:
                    event, self._pending_event = self._pending_event, None
                    frame = None
                else:
                    event, frame = None, self._preview.popleft()
            try:
                if event is not None:
                    self.on_event(event)
                else:
                    self.on_frame(*frame)
            except Exception:
                self.delivery_errors += 1
                # A failed preview callback cannot terminate camera recording.
                with self._delivery:
                    self.preview_dropped += len(self._preview)
                    self._preview.clear()
                    self._preview_need_key = True

    def _run(self):
        launch = PIPELINE.format(device=config.CAMERA_DEVICE,
                                 width=config.CAMERA_WIDTH,
                                 height=config.CAMERA_HEIGHT,
                                 fps=config.CAMERA_FPS,
                                 capture_buffers=config.CAMERA_CAPTURE_QUEUE_FRAMES)
        while self._running:
            # Every time the stream is (re)opened, not once at start: the
            # camera forgets the setting when it is unplugged.
            self._flicker_filter_set = set_flicker_filter()
            pipeline = None
            try:
                pipeline = Gst.parse_launch(launch)
                if pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
                    raise RuntimeError('Camera pipeline failed to start')
                with self._delivery:
                    self._preview.clear()
                    self._preview_need_key = True
                self._pump(pipeline.get_by_name("sink"))
            except Exception as error:
                self.error = f'{type(error).__name__}: {error}'
                self._event(f'camera: capture failed, retrying: {type(error).__name__}: {error}')
            finally:
                if pipeline is not None:
                    try:
                        pipeline.set_state(Gst.State.NULL)
                    except Exception as error:
                        self.error = f'Camera cleanup failed: {type(error).__name__}: {error}'
                        self._event(self.error)
            if self._streaming:
                self._streaming = False
                self.fps = self.mbps = 0.0
                self._event("camera: stream lost, retrying")
            if self._running:
                time.sleep(RETRY_S)

    def _pump(self, sink):
        """Hand frames over until the camera stops delivering them."""
        self._generation += 1
        frames = size = 0
        window_start = time.monotonic()
        while self._running:
            sample = sink.emit("try-pull-sample", FRAME_TIMEOUT_S * Gst.SECOND)
            if sample is None:
                return
            if not self._running:
                return
            buffer = sample.get_buffer()
            data = buffer.extract_dup(0, buffer.get_size())
            key = any(mark in data for mark in _KEYFRAME_MARKS)
            if self.on_record_frame:
                try:
                    self.on_record_frame(data, key, self._generation,
                                     None if buffer.pts == Gst.CLOCK_TIME_NONE else buffer.pts,
                                     None if buffer.dts == Gst.CLOCK_TIME_NONE else buffer.dts,
                                     buffer.has_flags(Gst.BufferFlags.DISCONT))
                except Exception as error:
                    self.record_errors += 1
                    self._event(f'camera: recording callback failed: {type(error).__name__}: {error}')
            self._queue_preview(data, key)

            if not self._streaming:
                self.error = None
                self._streaming = True
                self._event(f"camera: streaming {config.CAMERA_WIDTH}x"
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
