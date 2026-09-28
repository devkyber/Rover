"""
Headless recording loop. Reads sensors at a fixed rate, runs the filter,
writes a log. No window, no web server, no motor writing.

    python record.py --seconds 20            # -> logs/2026-09-06_1432_still.csv
    python record.py --seconds 60 --tag drive
    python record.py --fake-motors           # real IMU, no motors
    python record.py --fake                  # nothing plugged in

The log name is built from the clock, so no run can overwrite another.

Use this to record the stationary log that everything else is tuned
against. To drive, use teleop.py -- it runs the same cycle.
"""
import argparse
import time

import numpy as np

import config
import logger
import odometry


def main(args):
    import imu_reader
    imu = imu_reader.create_imu("fake" if args.fake else args.imu)

    fake_motors = args.fake or args.fake_motors
    port_h = reader = None
    if not fake_motors:
        import dxl_reader
        port_h, pkt_h = dxl_reader.open_port()
        reader = dxl_reader.create_sync_reader(port_h, pkt_h)

    log_path = None if args.no_log else (args.log or logger.new_log_path(args.tag))
    log = logger.Logger(log_path) if log_path else None
    odo = odometry.Odometry()

    dt = 1.0 / config.EKF_RATE_HZ
    t_start = next_tick = time.monotonic()
    t_prev = 0.0
    dropped = 0

    print(f"Recording {args.seconds:.0f} s at {config.EKF_RATE_HZ} Hz. "
          f"Ctrl+C stops early and still closes the log.")
    try:
        while time.monotonic() - t_start < args.seconds:
            t = time.monotonic() - t_start

            if fake_motors:
                wheels = [{'id': m, 'position_rad': 0.0, 'velocity_rads': 0.0,
                           'current_A': 0.0} for m in config.MOTOR_IDS]
            else:
                import dxl_reader
                data = dxl_reader.read_wheels(reader)
                if data is None:
                    # Drop the cycle but keep the beat -- skipping the
                    # pacing here makes the loop free-run to catch up and
                    # turns one lost packet into a burst of jitter.
                    dropped += 1
                    next_tick += dt
                    time.sleep(max(0.0, next_tick - time.monotonic()))
                    continue
                wheels = data['wheels']

            imu_data = imu.read()
            step = t - t_prev if odo.cycles else dt
            t_prev = t
            odo.step(wheels, gyro=imu_data['gyro'],
                     accel=imu_data['accel'],
                     dt=step if 0 < step < 0.5 else dt)

            if log:
                log.log(t, wheels, imu_data)

            if odo.cycles % config.EKF_RATE_HZ == 0:      # once a second
                p, rpy = odo.position, odo.rpy_deg
                mode = (f"ALIGNING {100*odo.alignment_progress:.0f}%"
                        if not odo.ready else
                        ('STOPPED' if odo.zupt_active else 'moving'))
                print(f"  t={t:5.1f}  pos {p[0]:+.3f} {p[1]:+.3f}  "
                      f"yaw {rpy[2]:+7.2f} deg  {mode}")

            next_tick += dt
            sleep = next_tick - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_tick = time.monotonic()
    except KeyboardInterrupt:
        print("\nStopped early.")
    finally:
        if log:
            log.close()
        if port_h is not None:
            import dxl_reader
            dxl_reader.close_port(port_h)
        imu.close()

    elapsed = time.monotonic() - t_start
    print(f"\n{odo.cycles} cycles in {elapsed:.1f} s "
          f"({odo.cycles/max(elapsed,1e-6):.1f} Hz), {dropped} dropped")
    print(f"final pos  {np.round(odo.position, 4)} m")
    print(f"final rpy  {np.round(odo.rpy_deg, 2)} deg")
    print(f"alignment  {'complete' if odo.ready else 'INCOMPLETE'}")
    print(f"gyro bias  {np.round(odo.gyro_bias, 5)} rad/s "
          f"({np.round(np.degrees(odo.gyro_bias), 2)} deg/s)")
    print(f"ZUPT       {odo.zupt_cycles}/{odo.cycles} cycles "
          f"({100*odo.zupt_cycles/max(odo.cycles,1):.1f}%)")
    if args.log:
        print(f"\nlog: {args.log}\n  python replay.py {args.log}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--fake", action="store_true", help="no hardware at all")
    ap.add_argument("--fake-motors", action="store_true", help="real IMU only")
    ap.add_argument("--imu", default="arduino",
                    choices=["fake", "arduino", "bmi088"])
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--tag", default="still",
                    help="name fragment for the auto-named log")
    ap.add_argument("--log", default=None,
                    help="explicit CSV path, overriding the auto name")
    ap.add_argument("--no-log", action="store_true", help="do not write a log")
    main(ap.parse_args())
