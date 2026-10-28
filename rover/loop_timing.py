"""Bounded main-loop stage timings; call end() before the scheduled sleep.

Only completed work cycles enter a published window. Snapshot values remain
unchanged until the next window is published; callers receive independent small
dicts, so telemetry consumers cannot mutate the cached measurement.
"""
import math
import time

import config


STAGES = (
    'requests', 'command', 'wheel_read', 'imu_poll', 'imu_state',
    'imu_integrate', 'wheel_write', 'hammer', 'lift', 'pico',
    'scout_policy', 'sensor_record', 'telemetry', 'publish',
)


class LoopTiming:
    STAGES = STAGES

    def __init__(self, window_s=config.LOOP_TIMING_WINDOW_S, *, clock=time.perf_counter):
        if isinstance(window_s, bool) or not isinstance(window_s, (int, float)) or not math.isfinite(window_s) or window_s <= 0:
            raise ValueError('Loop timing window must be finite and positive')
        self.window_s = float(window_s)
        self._clock = clock
        self._window_start = None
        self._cycle_start = self._last_mark = None
        self._cycles = 0
        self._work_sum = self._work_max = 0.0
        self._stages = {name: [0.0, 0.0, 0] for name in STAGES}
        self._cached = dict(completed_cycles=0, elapsed_window_s=0.0,
                            work_ms_mean=0.0, work_ms_max=0.0, stages={})

    def start(self):
        """Begin one work cycle; scheduled sleep must be outside start/end."""
        if self._cycle_start is not None:
            raise RuntimeError('Loop timing cycle already started')
        now = self._clock()
        if self._window_start is None:
            self._window_start = now
        self._cycle_start = self._last_mark = now

    def mark(self, stage):
        """Attribute elapsed work since the prior mark/start to a fixed stage."""
        if stage not in self._stages:
            raise ValueError('Unknown loop timing stage: ' + str(stage))
        if self._cycle_start is None:
            raise RuntimeError('Loop timing cycle not started')
        now = self._clock()
        elapsed = now - self._last_mark
        value = self._stages[stage]
        value[0] += elapsed
        value[1] = max(value[1], elapsed)
        value[2] += 1
        self._last_mark = now

    def end(self):
        """Complete work and publish a window when due; return whether published.

        Unmarked work after the last mark contributes to total work time only.
        completed_cycles counts cycles in the last published window. The elapsed
        window includes scheduled gaps; work_ms_* excludes those gaps.
        """
        if self._cycle_start is None:
            raise RuntimeError('Loop timing cycle not started')
        now = self._clock()
        work = now - self._cycle_start
        self._cycle_start = self._last_mark = None
        self._cycles += 1
        self._work_sum += work
        self._work_max = max(self._work_max, work)
        elapsed = now - self._window_start
        if elapsed < self.window_s:
            return False
        stages = {name: dict(mean_ms=1000.0 * value[0] / value[2],
                             max_ms=1000.0 * value[1], count=value[2])
                  for name, value in self._stages.items() if value[2]}
        self._cached = dict(completed_cycles=self._cycles, elapsed_window_s=elapsed,
                            work_ms_mean=1000.0 * self._work_sum / self._cycles,
                            work_ms_max=1000.0 * self._work_max, stages=stages)
        self._window_start = now
        self._cycles = 0
        self._work_sum = self._work_max = 0.0
        for value in self._stages.values():
            value[:] = (0.0, 0.0, 0)
        return True

    def snapshot(self):
        """Copy the latest published numeric summary, never a timing history."""
        cached = self._cached
        return dict(cached, stages={name: dict(value) for name, value in cached['stages'].items()})
