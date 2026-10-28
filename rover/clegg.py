"""Clegg wire decoding and explicit clock/pose association, without hardware I/O."""
import json
import math
import struct
import zlib
import config

HEADER = struct.Struct('<4sBBHI')
SAMPLE = struct.Struct('<QQQhhhHH')
MAX_PAYLOAD = 2048  # Version-1 protocol bound, including metadata.
MAX_PACKET_SAMPLES = (MAX_PAYLOAD - 10) // SAMPLE.size


def encode_frame(kind, payload):
    prefix = struct.pack('<BBH', 1, kind, len(payload))
    return b'CG01' + prefix + struct.pack('<I', zlib.crc32(prefix + payload)) + payload


class Decoder:
    def __init__(self):
        self.buffer = bytearray()
        self.crc_errors = self.discarded_bytes = 0

    def feed(self, raw):
        self.buffer.extend(raw)
        result = []
        while len(self.buffer) >= HEADER.size:
            at = self.buffer.find(b'CG01')
            if at < 0:
                self.discarded_bytes += len(self.buffer) - 3
                del self.buffer[:-3]
                break
            if at:
                self.discarded_bytes += at
                del self.buffer[:at]
                if len(self.buffer) < HEADER.size:
                    break
            _, version, kind, size, checksum = HEADER.unpack_from(self.buffer)
            if version != 1 or kind not in (1, 2, 3, 4) or size > MAX_PAYLOAD:
                self.discarded_bytes += 1
                del self.buffer[0]
                continue
            end = HEADER.size + size
            if len(self.buffer) < end:
                break
            payload = bytes(self.buffer[HEADER.size:end])
            if zlib.crc32(bytes(self.buffer[4:8]) + payload) != checksum:
                self.crc_errors += 1
                self.discarded_bytes += 1
                del self.buffer[0]
                continue
            del self.buffer[:end]
            if kind == 2:
                if len(payload) < 10:
                    raise ValueError('Truncated sample packet')
                boot, count = struct.unpack_from('<QH', payload)
                if not 1 <= count <= MAX_PACKET_SAMPLES or len(payload) != 10 + count * SAMPLE.size:
                    raise ValueError('Sample packet layout mismatch')
                value = dict(boot_id=f'{boot:016x}', samples=[
                    dict(zip(('seq', 'estimated_us', 'fifo_poll_us', 'raw_x', 'raw_y',
                              'raw_z', 'flags', 'fifo_depth'), SAMPLE.unpack_from(payload, 10 + i * SAMPLE.size)))
                    for i in range(count)])
            else:
                value = json.loads(payload)
                if not isinstance(value, dict):
                    raise ValueError('Metadata must be an object')
            result.append((kind, value))
        return result


class ClockMap:
    """Causal round-trip interval; drift envelope is assumed, not calibrated."""
    def __init__(self, max_age_s=config.CLEGG_SYNC_MAX_AGE_S, drift_ppm=config.CLEGG_ASSUMED_DRIFT_PPM):
        self.max_age_s, self.drift_ppm = max_age_s, drift_ppm
        self.boot_id = self.offset_s = self.error_s = self.updated_s = None

    def update(self, boot_id, t0, d1, d2, t3):
        if not all(math.isfinite(x) for x in (t0, d1, d2, t3)) or t3 < t0 or d2 < d1:
            raise ValueError('Invalid synchronization timestamps')
        lower, upper = t0 - d1, t3 - d2
        if lower > upper:
            raise ValueError('Impossible causal clock interval')
        self.boot_id = boot_id
        self.offset_s, self.error_s = (lower + upper) / 2, (upper - lower) / 2
        self.updated_s = t3

    def convert(self, boot_id, device_s, now_s):
        if boot_id != self.boot_id or self.updated_s is None or not math.isfinite(device_s):
            return None
        age = now_s - self.updated_s
        if not 0 <= age <= self.max_age_s:
            return None
        uncertainty = self.error_s + abs(device_s + self.offset_s - self.updated_s) * self.drift_ppm * 1e-6
        return device_s + self.offset_s, uncertainty


def associate_pose(history, target_s, time_error_s, max_difference_s):
    """No interpolation across an invalid sample, run, or coordinate frame."""
    if target_s is None or time_error_s is None or not history:
        return dict(available=False, reason='clock_or_pose_history_unavailable')
    if not all(math.isfinite(x) for x in (target_s, time_error_s)) or time_error_s < 0:
        return dict(available=False, reason='invalid_time_bound')
    ordered = sorted(history, key=lambda p: p['captured_monotonic_s'])
    before = [p for p in ordered if p['captured_monotonic_s'] <= target_s-time_error_s]
    after = [p for p in ordered if p['captured_monotonic_s'] >= target_s+time_error_s]
    if not before or not after:
        return dict(available=False, reason='pose_time_not_bracketed')
    left, right = before[-1], after[0]
    identity = lambda p: (p.get('run_id'), p.get('pose_frame_id'), p.get('imu_generation'))
    if (identity(left) != identity(right) or any(v is None for v in identity(left))
            or not left.get('run_id') or not left.get('pose_frame_id')):
        return dict(available=False, reason='pose_frame_or_generation_changed')
    window = [p for p in ordered if left['captured_monotonic_s'] <= p['captured_monotonic_s'] <= right['captured_monotonic_s']]
    if any(p.get('pose_valid') is not True for p in window):
        return dict(available=False, reason='invalid_rover_pose')
    if any(identity(p) != identity(left) for p in window):
        return dict(available=False, reason='pose_frame_or_generation_changed')
    nearest = min(window, key=lambda p: abs(p['captured_monotonic_s'] - target_s))
    difference = abs(nearest['captured_monotonic_s'] - target_s)
    if max(target_s-left['captured_monotonic_s'], right['captured_monotonic_s']-target_s) + time_error_s > max_difference_s:
        return dict(available=False, reason='pose_timing_too_far')
    pos = nearest.get('pos')
    if not isinstance(pos, list) or len(pos) != 3 or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in pos):
        return dict(available=False, reason='invalid_position')
    return dict(available=True, reason=None, association='nearest_bracketed_sample',
                sample=nearest, difference_s=difference, clock_error_s=time_error_s,
                position_accuracy_validated=False, hammer_contact_position=None)
