"""Show new Clegg acceleration candidates in a native Matplotlib window.

Read-only: this process watches the station REST receiver, never the Nano or
Rover control sockets. No window opens until a new completed waveform arrives.
Closing the native window stops this watcher. Use --file to view saved evidence.
"""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import queue
import sys
import textwrap
import threading
import time
import urllib.parse
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'rover'))
import config


def _finite(value):
    return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)


def _reject_constant(value):
    raise ValueError('Nonfinite JSON number: '+value)


def validate_impact(value):
    """Reject corrupt timestamps/arrays without altering the original samples."""
    if not isinstance(value, dict) or value.get('kind') != 'impact_candidate' or value.get('state') != 'ready':
        raise ValueError('A completed impact_candidate is required')
    if value.get('calibrated') is not False or not isinstance(value.get('id'), str) or not value['id']:
        raise ValueError('Expected identified, uncalibrated nominal acceleration')
    times, xyz, magnitude = (value.get(key) for key in ('time_s', 'xyz_g', 'magnitude_g'))
    if not isinstance(times, list) or not 1 <= len(times) <= config.CLEGG_IMPACT_MAX_POINTS:
        raise ValueError('Missing or oversized waveform')
    if not isinstance(xyz, list) or not isinstance(magnitude, list) or len(xyz) != len(times) or len(magnitude) != len(times):
        raise ValueError('Waveform array lengths differ')
    previous = -math.inf
    for stamp, axes, mag in zip(times, xyz, magnitude):
        if not _finite(stamp) or stamp <= previous:
            raise ValueError('Waveform timestamps must be finite and strictly increasing')
        if not isinstance(axes, list) or len(axes) != 3 or not all(_finite(axis) for axis in axes):
            raise ValueError('XYZ acceleration must contain three finite values per sample')
        if not _finite(mag) or mag < 0 or not math.isclose(mag, math.hypot(*axes), rel_tol=1e-6, abs_tol=1e-8):
            raise ValueError('Acceleration magnitude does not match XYZ')
        previous = stamp
    if not times[0] <= 0 <= times[-1] or not any(abs(stamp) <= 1e-10 for stamp in times):
        raise ValueError('Relative waveform must include its trigger at zero seconds')
    if value.get('trigger') not in ('manual', 'threshold_crossing'):
        raise ValueError('Unknown waveform trigger')
    if not isinstance(value.get('flags'), list) or not all(isinstance(flag, str) for flag in value['flags']):
        raise ValueError('Waveform flags must be a list of strings')
    if type(value.get('truncated')) is not bool:
        raise ValueError('Waveform truncation status is required')
    return value


class NewReadySelector:
    """Skip the first already-ready capture; retry fetch failures, avoid repeats."""
    def __init__(self, show_latest=False):
        self.initialized = False
        self.seen = deque(maxlen=config.CLEGG_RELEASE_HISTORY)
        self.show_latest = show_latest

    def candidate(self, status):
        if not isinstance(status, dict) or type(status.get('available')) is not bool:
            raise ValueError('Invalid station receiver status')
        latest = status.get('latest_impact')
        if latest is not None and (not isinstance(latest, dict) or not isinstance(latest.get('id'), str)):
            raise ValueError('Invalid latest waveform identity')
        ready = latest is not None and latest.get('state') == 'ready'
        if not self.initialized:
            self.initialized = True
            if ready and not self.show_latest:
                self.accept(latest['id'])
                return None
        if ready and latest['id'] not in self.seen:
            return latest['id']
        return None

    def accept(self, impact_id):
        if impact_id not in self.seen:
            self.seen.append(impact_id)


class StationClient:
    def __init__(self, station):
        parsed = urllib.parse.urlsplit(station)
        if (parsed.scheme not in ('http', 'https') or not parsed.hostname or parsed.username or parsed.password
                or parsed.path not in ('', '/') or parsed.query or parsed.fragment):
            raise ValueError('--station must be an HTTP(S) origin, without credentials, query or path')
        self.station = station.rstrip('/')
        self.opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def get(self, path):
        # Every request is GET. A byte bound and deadline also cover malformed
        # local responses; no network operation runs on the GUI thread.
        maximum = config.CLEGG_IMPACT_MAX_POINTS*1024
        deadline = time.monotonic()+config.CLEGG_CONNECT_TIMEOUT_S
        with self.opener.open(self.station+path, timeout=config.CLEGG_CONNECT_TIMEOUT_S) as response:
            data = bytearray()
            while True:
                if time.monotonic() > deadline:
                    raise TimeoutError('Station response deadline exceeded')
                block = response.read1(min(65536, maximum+1-len(data)))
                if not block:
                    break
                data.extend(block)
                if len(data) > maximum:
                    raise ValueError('Oversized station response')
        return json.loads(data, parse_constant=_reject_constant)


class ImpactWatcher:
    def __init__(self, client, report=print, show_latest=False):
        self.client, self.report = client, report
        self.selector = NewReadySelector(show_latest=show_latest)
        self.ready = queue.Queue(maxsize=1)
        self.stop_event = threading.Event()
        self.worker = None
        self.last_error = None

    def poll_once(self):
        impact_id = self.selector.candidate(self.client.get('/clegg/status'))
        if impact_id is None:
            return None
        impact = validate_impact(self.client.get('/clegg/impact?'+urllib.parse.urlencode({'id': impact_id})))
        if impact['id'] != impact_id:
            raise ValueError('Waveform identity changed during retrieval')
        self.selector.accept(impact_id)
        return impact

    def _run(self):
        while not self.stop_event.is_set():
            try:
                impact = self.poll_once()
                self.last_error = None
                if impact is not None:
                    try:
                        self.ready.get_nowait()
                    except queue.Empty:
                        pass
                    self.ready.put_nowait(impact)
            except (OSError, ValueError, TypeError) as exc:
                message = str(exc)
                if message != self.last_error:
                    self.report('Clegg popup waiting: '+message)
                    self.last_error = message
            self.stop_event.wait(config.CLEGG_SYNC_INTERVAL_S)

    def start(self):
        self.worker = threading.Thread(target=self._run, name='clegg-plot-rest', daemon=True)
        self.worker.start()
        return self

    def stop(self):
        self.stop_event.set()
        if self.worker:
            self.worker.join(timeout=config.CLEGG_CONNECT_TIMEOUT_S+1.)


def metadata_text(impact):
    flags = ', '.join(impact['flags']) or 'none'
    source = impact.get('source_ref') or 'unavailable'
    if not isinstance(source, str):
        source = json.dumps(source, ensure_ascii=False)
    position = impact.get('position') if isinstance(impact.get('position'), dict) else {}
    sample = position.get('sample') if isinstance(position.get('sample'), dict) else {}
    pos = sample.get('pos')
    if position.get('available') is True and isinstance(pos, list) and len(pos) == 3 and all(_finite(v) for v in pos):
        pose = ('Rover estimate near trigger '+', '.join(f'{v:.3f}' for v in pos)+' m'
                +f"; run {sample.get('run_id')}; frame {sample.get('pose_frame_id')}; IMU generation {sample.get('imu_generation')}; quality {sample.get('pose_quality') or 'unavailable'}"
                +f"; pose source {sample.get('source_log') or 'unavailable'} / seq {sample.get('source_seq')}")
    else:
        pose = 'Rover position unavailable: '+str(position.get('reason') or 'not associated')
    release = impact.get('release_request')
    request = (f"Release request {release.get('shot_id')}; intent only, actual drop/contact unvalidated"
               if isinstance(release, dict) else 'Release request: not associated')
    timing = impact.get('timing') if isinstance(impact.get('timing'), dict) else {}
    epoch = timing.get('sensor_boot_id')
    number = lambda key: f'{timing[key]:.6f} s' if _finite(timing.get(key)) else 'unavailable'
    times = ('Estimated trigger times: sensor '+number('sensor_estimated_s')
             +'; host monotonic '+number('host_time_estimate_s')+'; clock bound '+number('clock_error_s'))
    return '\n'.join((f"ID {impact['id']} · {len(impact['time_s'])} original samples · truncated={impact['truncated']} · flags: {flags}",
                      'Source: '+source+(f' · sensor boot {epoch}' if epoch else ''), times, pose, request,
                      'Nominal datasheet g; estimated sensor time. Contact, CIV and position accuracy are not validated; no confirmed map point.'))


def draw_impact(impact, figure=None, *, pyplot=None):
    validate_impact(impact)
    if pyplot is None:
        import matplotlib.pyplot as pyplot
    if figure is None:
        figure = pyplot.figure(figsize=(12, 7.5))
    figure.clear()
    axes = figure.add_axes((.085, .35, .89, .53))
    milliseconds = [stamp*1000 for stamp in impact['time_s']]
    for index, (name, color) in enumerate((('X', '#2563eb'), ('Y', '#16a34a'), ('Z', '#ea580c'))):
        line, = axes.plot(milliseconds, [values[index] for values in impact['xyz_g']],
                          label=name, color=color, linewidth=1.0)
        line.get_path().should_simplify = False
    line, = axes.plot(milliseconds, impact['magnitude_g'], label='Magnitude', color='#7c3aed', linewidth=1.6)
    line.get_path().should_simplify = False
    peak_index = max(range(len(milliseconds)), key=impact['magnitude_g'].__getitem__)
    axes.scatter([milliseconds[peak_index]], [impact['magnitude_g'][peak_index]], s=22, color='#7c3aed', zorder=4)
    manual = impact['trigger'] == 'manual'
    axes.axvline(0, color='#64748b', linestyle='--', linewidth=.8,
                 label='Manual marker' if manual else 'Acceleration trigger')
    axes.set_xlabel('Time from manual marker (ms)' if manual else 'Time from acceleration trigger (ms)')
    axes.set_ylabel('Acceleration (nominal g)')
    axes.grid(True, alpha=.25)
    axes.legend(loc='lower right', bbox_to_anchor=(1.,1.01), ncol=5, fontsize=9, borderaxespad=0)
    if len(milliseconds) == 1:
        axes.set_xlim(milliseconds[0]-1, milliseconds[0]+1)
    peak = impact['magnitude_g'][peak_index]
    kind = 'Manual sample window' if manual else 'Acceleration candidate'
    title = f'Clegg · {kind} · peak {peak:.2f} nominal g'
    if impact.get('synthetic') is True:
        title = 'SYNTHETIC CHECK · '+title
    figure.suptitle(title, fontsize=14)
    figure.text(.085, .26, '\n'.join(textwrap.fill(line, width=150, break_long_words=True)
                                   for line in metadata_text(impact).splitlines()),
                fontsize=8, va='top', family='sans-serif')
    manager = figure.canvas.manager
    if manager and hasattr(manager, 'set_window_title'):
        manager.set_window_title('Clegg acceleration — close window to stop watcher')
    figure.canvas.draw_idle()
    return figure


def _pyplot(native):
    import matplotlib
    matplotlib.use('TkAgg' if native else 'Agg', force=True)
    matplotlib.rcParams['path.simplify'] = False
    import matplotlib.pyplot as pyplot
    return pyplot


def _show_native(figure, pyplot):
    # Windows launchers may hide the console. Deliberately reveal only the
    # completed-data Tk window, including when pythonw inherited hidden state.
    manager = figure.canvas.manager
    manager.show()
    window = getattr(manager, 'window', None)
    if window is not None:
        window.deiconify()
        window.lift()
    pyplot.show()


def show_file(path, render=None):
    if path.stat().st_size > config.CLEGG_IMPACT_MAX_POINTS*1024:
        raise ValueError('Oversized saved impact JSON')
    impact = validate_impact(json.loads(path.read_text(encoding='utf-8'), parse_constant=_reject_constant))
    pyplot = _pyplot(native=render is None)
    figure = draw_impact(impact, pyplot=pyplot)
    try:
        if render is not None:
            if render.suffix.lower() != '.png':
                raise ValueError('--render requires a new .png path')
            render.parent.mkdir(parents=True, exist_ok=True)
            with render.open('xb') as output:
                figure.savefig(output, format='png', dpi=150)
            print('Rendered saved evidence: '+str(render.resolve()))
        else:
            _show_native(figure, pyplot)
    finally:
        pyplot.close(figure)


def watch(station, show_latest=False):
    watcher = ImpactWatcher(StationClient(station), show_latest=show_latest).start()
    print('Watching new Clegg candidates at '+station+'; no window until a new completed capture. Ctrl+C stops.', flush=True)
    try:
        # Creating a Tk figure is deliberately deferred until validated data.
        while True:
            try:
                impact = watcher.ready.get(timeout=.5)
                break
            except queue.Empty:
                continue
        pyplot = _pyplot(native=True)
        figure = draw_impact(impact, pyplot=pyplot)
        closed = threading.Event()
        figure.canvas.mpl_connect('close_event', lambda event: closed.set())

        def update():
            if closed.is_set():
                return False
            try:
                latest = watcher.ready.get_nowait()
            except queue.Empty:
                return True
            draw_impact(latest, figure, pyplot=pyplot)
            return True

        timer = figure.canvas.new_timer(interval=100)
        timer.add_callback(update)
        timer.start()
        _show_native(figure, pyplot)
        timer.stop()
        # A deliberate close exits; later network events cannot reopen a window.
    finally:
        watcher.stop()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--station', default='http://127.0.0.1:8766', help='Read-only receiver HTTP origin')
    parser.add_argument('--file', type=Path, help='Open a preserved impact JSON instead of watching')
    parser.add_argument('--show-latest', action='store_true', help='Explicitly show the first already-ready waveform, then watch new ones')
    parser.add_argument('--render', type=Path, help='With --file only: render a new PNG using Agg for verification')
    args = parser.parse_args(argv)
    if args.render and not args.file:
        parser.error('--render requires --file')
    try:
        if args.file:
            show_file(args.file, args.render)
        else:
            watch(args.station, show_latest=args.show_latest)
    except KeyboardInterrupt:
        return 0
    except (OSError, ValueError, TypeError, ImportError) as exc:
        parser.exit(1, 'Clegg plot: '+str(exc)+'\n')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
