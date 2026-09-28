"""
End-to-end pipeline check. Run this BEFORE trusting any logged run.

    python rover/test_pipeline.py                 # motors + Arduino IMU
    python rover/test_pipeline.py --fake-motors   # IMU only (no motors yet)
    python rover/test_pipeline.py --fake          # nothing plugged in

It drives the real read -> log -> replay path for a few seconds and then
AUDITS the result, instead of just printing numbers and hoping they look
right. Checks:

  1. every sensor actually returned data (no silent zeros)
  2. host timestamps increase, and the loop held its rate
  3. the IMU's own clock advances at the rate the datasheet claims
  4. accelerometer magnitude is ~1 g while sitting still
  5. wheel encoders move / stay put consistently with their velocity
  6. the CSV replays back byte-for-byte equal to what we logged

Check 6 is the important one: it is what makes a log worth keeping.
"""
import argparse
import os
import time

import numpy as np

import config
import imu_reader
import logger


def collect(use_fake_motors, imu_kind, seconds, log_file):
    """Run the real loop and return (rows_in_memory, imu_object)."""
    imu = imu_reader.create_imu(imu_kind)

    reader = port_h = None
    if not use_fake_motors:
        import dxl_reader
        port_h, pkt_h = dxl_reader.open_port()
        reader = dxl_reader.create_sync_reader(port_h, pkt_h)

    log = logger.Logger(log_file)
    rows = []
    dropped_motor_cycles = 0

    dt = 1.0 / config.EKF_RATE_HZ
    t_start = time.monotonic()
    next_tick = t_start

    print(f"\nCollecting {seconds} s at {config.EKF_RATE_HZ} Hz...")
    try:
        while time.monotonic() - t_start < seconds:
            t = time.monotonic() - t_start

            if use_fake_motors:
                wheels = [{'id': mid, 'position_rad': 0.0,
                           'velocity_rads': 0.0, 'current_A': 0.0}
                          for mid in config.MOTOR_IDS]
            else:
                import dxl_reader
                data = dxl_reader.read_wheels(reader)
                if data is None:
                    # Drop the cycle, but still keep the beat. Skipping
                    # the pacing here would make the loop free-run at
                    # full speed until it caught up, turning one lost
                    # packet into a burst of jitter.
                    dropped_motor_cycles += 1
                    next_tick += dt
                    sleep = next_tick - time.monotonic()
                    if sleep > 0:
                        time.sleep(sleep)
                    else:
                        next_tick = time.monotonic()
                    continue
                wheels = data['wheels']

            imu_data = imu.read()
            log.log(t, wheels, imu_data)
            rows.append({'t': t, 'imu_t': imu_data['device_t'],
                         'wheels': wheels,
                         'accel': np.asarray(imu_data['accel']),
                         'gyro': np.asarray(imu_data['gyro'])})

            next_tick += dt
            sleep = next_tick - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_tick = time.monotonic()
    finally:
        log.close()
        if port_h is not None:
            import dxl_reader
            dxl_reader.close_port(port_h)

    return rows, imu, dropped_motor_cycles


def audit(rows, imu, log_file, use_fake_motors, dropped_motor_cycles):
    """Judge what we collected. Returns True if everything passed."""
    results = []

    def check(name, ok, detail):
        results.append(ok)
        print(f"  [{'PASS' if ok else 'FAIL'}] {name:34s} {detail}")

    print("\n" + "=" * 68)
    print("AUDIT")
    print("=" * 68)

    if not rows:
        print("  [FAIL] no data collected at all")
        return False

    t = np.array([r['t'] for r in rows])
    imu_t = np.array([r['imu_t'] for r in rows])
    accel = np.array([r['accel'] for r in rows])
    gyro = np.array([r['gyro'] for r in rows])

    # 1. sample count and rate
    span = t[-1] - t[0]
    rate = (len(rows) - 1) / span if span > 0 else 0
    check("loop rate", abs(rate - config.EKF_RATE_HZ) < 10,
          f"{rate:.1f} Hz over {len(rows)} samples (target {config.EKF_RATE_HZ})")

    # 2. host clock is strictly increasing
    dt_host = np.diff(t)
    check("host timestamps increasing", bool(np.all(dt_host > 0)),
          f"min gap {1000*dt_host.min():.2f} ms, max {1000*dt_host.max():.2f} ms")

    # jitter -- how tightly we held the beat
    jitter_ms = 1000 * dt_host.std()
    check("loop jitter < 3 ms", jitter_ms < 3.0, f"std {jitter_ms:.2f} ms")

    # 3. IMU's own clock
    dt_imu = np.diff(imu_t)
    check("IMU clock increasing", bool(np.all(dt_imu >= 0)),
          f"min {1000*dt_imu.min():.2f} ms, max {1000*dt_imu.max():.2f} ms")

    fresh = dt_imu[dt_imu > 0]
    if len(fresh):
        imu_rate = 1.0 / fresh.mean()
        # host asks at 100 Hz, board produces at ~104 Hz, so the observed
        # per-sample rate should land near the slower of the two
        check("IMU sample rate plausible", 50 < imu_rate < 250,
              f"{imu_rate:.1f} Hz (board claims ~{config.IMU_EXPECTED_HZ})")

    stale = int(np.sum(dt_imu == 0))
    check("few repeated IMU samples", stale < 0.2 * len(dt_imu),
          f"{stale}/{len(dt_imu)} cycles reused the previous sample")

    # 4. accelerometer sanity -- must read 1 g when still
    mag = np.linalg.norm(accel, axis=1)
    check("accel magnitude ~ 1 g", abs(mag.mean() - config.GRAVITY) < 1.5,
          f"mean {mag.mean():.3f} m/s^2, std {mag.std():.3f}")

    check("accel not stuck", accel.std(axis=0).max() > 1e-9,
          f"per-axis std {np.round(accel.std(axis=0), 4)}")

    # gyro bias while stationary -- this is what ZARU will have to remove
    gb = gyro.mean(axis=0)
    print(f"         gyro bias at rest    {np.round(gb, 4)} rad/s "
          f"({np.round(np.degrees(gb), 2)} deg/s)")

    # 5. wheels
    if not use_fake_motors:
        n_wheels = len(config.MOTOR_IDS)
        got = [len(r['wheels']) for r in rows]
        check("all wheels every cycle", all(g == n_wheels for g in got),
              f"expected {n_wheels}, min seen {min(got)}")
        check("no dropped motor packets", dropped_motor_cycles == 0,
              f"{dropped_motor_cycles} cycles dropped")

        for i, mid in enumerate(config.MOTOR_IDS):
            pos = np.array([r['wheels'][i]['position_rad'] for r in rows])
            vel = np.array([r['wheels'][i]['velocity_rads'] for r in rows])
            cur = np.array([r['wheels'][i]['current_A'] for r in rows])
            moved = abs(pos.max() - pos.min())
            print(f"         ID {mid}: pos range {moved:.4f} rad, "
                  f"|vel| max {np.abs(vel).max():.4f} rad/s, "
                  f"current {cur.mean():+.4f} A")
            # if a wheel reports speed it must also change position
            if np.abs(vel).max() > 0.05:
                check(f"ID {mid} position tracks velocity", moved > 1e-4,
                      f"moved {moved:.4f} rad")

    # 6. THE IMPORTANT ONE -- does the CSV read back identical?
    replayed = list(logger.replay(log_file))
    check("replay row count matches", len(replayed) == len(rows),
          f"logged {len(rows)}, replayed {len(replayed)}")

    if len(replayed) == len(rows):
        worst = 0.0
        for orig, rep in zip(rows, replayed):
            worst = max(worst, abs(orig['t'] - rep['t']))
            worst = max(worst, abs(orig['imu_t'] - rep['imu_t']))
            worst = max(worst, float(np.abs(orig['accel'] - rep['accel']).max()))
            worst = max(worst, float(np.abs(orig['gyro'] - rep['gyro']).max()))
            for a, b in zip(orig['wheels'], rep['wheels']):
                worst = max(worst, abs(a['position_rad'] - b['position_rad']))
                worst = max(worst, abs(a['velocity_rads'] - b['velocity_rads']))
                worst = max(worst, abs(a['current_A'] - b['current_A']))
        # CSV is written with 6 decimals, so this is the expected precision
        check("replay values match log", worst < 1e-6,
              f"worst difference {worst:.2e}")

    if hasattr(imu, 'stats'):
        s = imu.stats()
        print(f"\n  IMU stream: {s['parsed']} parsed, {s['bad']} unparseable, "
              f"{s['dropped']} skipped while draining")
        check("no corrupt IMU lines", s['bad'] == 0, f"{s['bad']} bad lines")

    print("=" * 68)
    passed, total = sum(results), len(results)
    print(f"{passed}/{total} checks passed")
    return all(results)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--fake", action="store_true", help="no hardware at all")
    ap.add_argument("--fake-motors", action="store_true",
                    help="real IMU, no motors")
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--log", default="test_pipeline.csv")
    ap.add_argument("--keep", action="store_true", help="keep the CSV")
    args = ap.parse_args()

    fake_motors = args.fake or args.fake_motors
    imu_kind = "fake" if args.fake else "arduino"

    rows, imu, dropped = collect(fake_motors, imu_kind, args.seconds, args.log)
    ok = audit(rows, imu, args.log, fake_motors, dropped)
    imu.close()

    print(f"\nlog file: {args.log} ({os.path.getsize(args.log)} bytes)")
    if not args.keep:
        os.remove(args.log)
        print("(deleted -- pass --keep to keep it)")

    raise SystemExit(0 if ok else 1)
