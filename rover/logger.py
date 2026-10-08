"""
CSV logger for sensor data, and a replayer for offline processing.

Log format: one row per timestep, all sensor data in SI units.
Replay reads the CSV back as dicts -- feed into ESKF for offline tuning.

Log files are named automatically (date_time_tag) so that a run is never
overwritten and the file itself says when it happened. See new_log_path.
"""
import csv
import datetime
import json
import os

import config

# Raw logs go here. They are INPUTS: the only way to get one back is to
# run the experiment again. Derived things (plots) go in ../data/.
LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       os.pardir, "logs")


def new_log_path(tag="run", directory=None):
    """
    A fresh, never-colliding log path: logs/2026-09-06_1432_drive.csv

    Timestamped on purpose. A fixed name like run1.csv means the second
    run silently destroys the first, and you find out days later when the
    log you wanted to re-tune against is gone.

    The minute resolution is enough to sort a session; if two runs start
    inside the same minute a -2, -3 suffix keeps them apart.
    """
    directory = os.path.abspath(directory or LOG_DIR)
    os.makedirs(directory, exist_ok=True)

    stamp = datetime.datetime.now().strftime("%Y-%m-%d_%H%M")
    tag = "".join(c if c.isalnum() or c in "-_" else "-" for c in tag)

    path = os.path.join(directory, f"{stamp}_{tag}.csv")
    n = 2
    while os.path.exists(path):
        path = os.path.join(directory, f"{stamp}_{tag}-{n}.csv")
        n += 1
    return path

# Column names for the CSV. Built from config.MOTOR_IDS so that going
# from the 2-motor bench setup to the 4-wheel rover needs no edit here.
N_WHEELS = len(config.MOTOR_IDS)

# 't'     = host loop time (seconds since the run started)
# 'imu_t' = the IMU's own nominal clock, with arbitrary offset/rate. Its
# difference from host time is NOT directly latency. BMI088 uses the separate
# host FIFO interval imu_dt for integration, preserving nominal accel time.
COLUMNS = (['t', 'imu_t', 'imu_frame']
           + [f'w{i+1}_{f}' for i in range(N_WHEELS)
              for f in ('pos', 'vel', 'cur')]
           + ['ax', 'ay', 'az', 'gx', 'gy', 'gz']
           + ['imu_source', 'imu_dt', 'gyro_samples', 'imu_host_t',
              'accel_host_t', 'imu_temp_c', 'imu_tick', 'accel_raw', 'gyro_raw'])


class Logger:
    """Write sensor data to CSV, one row per timestep."""

    # Flush Python's user-space buffer every 100 rows (about 1 s at 100 Hz).
    # This reduces data lost on a process crash and makes new rows visible
    # to readers. It does not fsync the OS cache or guarantee durability
    # after power loss; fewer samples per second also means longer gaps.
    FLUSH_EVERY = 100          # rows == 1 s at 100 Hz

    def __init__(self, filename, metadata=None):
        self.filename = filename
        self.file = open(filename, 'x', newline='')
        if metadata is not None:
            try:
                with open(str(filename) + '.meta.json', 'x', encoding='utf-8') as f:
                    json.dump(metadata, f, indent=2, ensure_ascii=False)
                    f.write('\n')
            except BaseException:
                self.file.close()
                raise
        self.writer = csv.writer(self.file)
        self.writer.writerow(COLUMNS)
        self.file.flush()      # header on disk before the first row
        self.count = 0

    def log(self, timestamp, wheel_data, imu_data):
        """
        wheel_data: list of 4 dicts with 'position_rad', 'velocity_rads', 'current_A'
        imu_data:   dict with 'accel' (3,) and 'gyro' (3,) arrays
        """
        row = [f"{timestamp:.6f}",
               f"{imu_data.get('device_t', 0.0):.6f}",
               imu_data.get('frame', 'body')]

        for w in wheel_data:
            row.append(f"{w['position_rad']:.6f}")
            row.append(f"{w['velocity_rads']:.6f}")
            row.append(f"{w['current_A']:.6f}")

        a = imu_data['accel']
        g = imu_data['gyro']
        row.extend([f"{a[0]:.6f}", f"{a[1]:.6f}", f"{a[2]:.6f}"])
        row.extend([f"{g[0]:.6f}", f"{g[1]:.6f}", f"{g[2]:.6f}"])
        row.extend([imu_data.get('source', ''), imu_data.get('dt', ''),
                    json.dumps(imu_data['gyro_samples'], separators=(',', ':')) if 'gyro_samples' in imu_data else '',
                    imu_data.get('timestamp', ''), imu_data.get('accel_timestamp', ''),
                    imu_data.get('temperature_c', ''), imu_data.get('sensor_tick', ''),
                    json.dumps(imu_data['accel_raw']) if 'accel_raw' in imu_data else '',
                    json.dumps(imu_data['gyro_raw']) if 'gyro_raw' in imu_data else ''])

        self.writer.writerow(row)
        self.count += 1
        if self.count % self.FLUSH_EVERY == 0:
            self.file.flush()

    def close(self):
        self.file.close()
        print(f"Logged {self.count} rows -> {self.filename}")


def replay(filename):
    """
    Generator: yields one dict per row from a log CSV.

    Each dict has:
        't':      float   (seconds)
        'wheels': list of 4 dicts {position_rad, velocity_rads, current_A}
        'accel':  [ax, ay, az]
        'gyro':   [gx, gy, gz]
    """
    with open(filename, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            wheels = []
            for i in range(1, N_WHEELS + 1):
                wheels.append({
                    'position_rad':  float(row[f'w{i}_pos']),
                    'velocity_rads': float(row[f'w{i}_vel']),
                    'current_A':     float(row[f'w{i}_cur']),
                })

            accel = [float(row['ax']), float(row['ay']), float(row['az'])]
            gyro = [float(row['gx']), float(row['gy']), float(row['gz'])]

            # Logs made before 2026-09-11 have no imu_frame column. The
            # Arduino driver then logged board axes directly even though the
            # filter expected rover body axes. Preserve the raw CSV, but fix
            # that known legacy format at the replay boundary.
            source_frame = row.get('imu_frame') or 'arduino_sensor_legacy'
            imu_frame = source_frame
            if imu_frame == 'arduino_sensor_legacy':
                from imu_reader import arduino_sensor_to_body
                accel = arduino_sensor_to_body(accel)
                gyro = arduino_sensor_to_body(gyro)
                imu_frame = 'body'
            elif imu_frame != 'body':
                raise ValueError(f"Unsupported IMU frame in log: {imu_frame}")

            extra = {}
            if row.get('imu_source') == 'bmi088':
                # No fallback to loop time or one selected gyro frame.
                extra = {'imu_dt': float(row['imu_dt']),
                         'gyro_samples': json.loads(row['gyro_samples'])}
            yield {
                't':      float(row['t']),
                'imu_t':  float(row.get('imu_t', 0.0)),
                'wheels': wheels,
                'accel':  accel,
                'gyro':   gyro,
                'frame':  imu_frame,
                'source_frame': source_frame,
                **extra,
            }


# ── Quick test ─────────────────────────────────────────────────────

if __name__ == "__main__":
    import numpy as np

    test_file = "test_log.csv"

    # Write some fake data
    log = Logger(test_file)
    for i in range(5):
        wheels = [{'position_rad': i * 0.1, 'velocity_rads': 1.0,
                   'current_A': 0.5}] * N_WHEELS
        imu = {'accel': np.array([0, 0, 9.81]), 'gyro': np.array([0, 0, 0]),
               'device_t': i * 0.0096, 'frame': 'body'}
        log.log(i * 0.01, wheels, imu)
    log.close()

    # Read it back
    print("\nReplay:")
    for row in replay(test_file):
        print(f"  t={row['t']:.3f}  w1_pos={row['wheels'][0]['position_rad']:.3f}  "
              f"accel_z={row['accel'][2]:.2f}")

    os.remove(test_file)
    print("\nTest passed -- CSV write and read match.")
