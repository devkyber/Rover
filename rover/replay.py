"""
Run a recorded log back through the filter and judge the result.

    python replay.py ../logs/2026-09-06_4wheel_still.csv
    python replay.py ../logs/run.csv --plot ../data/run.png
    python replay.py ../logs/run.csv --no-zupt     # what does ZUPT buy?

Uses odometry.Odometry -- the same cycle the live loop runs, evaluated
with the current code and configuration. This can differ from recording day.

The sensor CSV does not contain independent trajectory ground truth. NIS asks:
"when the filter predicted a measurement and got it wrong, was it wrong
by about as much as it claimed it might be?" That needs only the
filter's own numbers. See docs/03_튜닝과_NIS.md.
"""
import argparse
import sys

import numpy as np

import config
import logger
import odometry
from config import NIS_DOF


def inspect(rows):
    """Check the recording itself before trusting anything in it."""
    t = np.array([r['t'] for r in rows])
    imu_t = np.array([r['imu_t'] for r in rows])
    accel = np.array([r['accel'] for r in rows])
    gyro = np.array([r['gyro'] for r in rows])
    dt = np.diff(t)

    # Drop the first interval. t[0] is not a loop period -- it is the
    # time from the run starting to the first row landing, which
    # includes opening the port and the IMU's first sample. Counting it
    # reported "1 gap, the loop stalled" on EVERY log ever recorded, and
    # a warning that is always on is a warning nobody reads.
    dt = dt[1:] if len(dt) > 1 else dt

    print("=" * 70)
    print(f"LOG  {len(rows)} samples, {t[-1]-t[0]:.1f} s, {1/dt.mean():.1f} Hz "
          f"(gap {1000*dt.min():.1f}-{1000*dt.max():.1f} ms)")
    print("=" * 70)

    stalls = int(np.sum(dt > 3 * dt.mean()))
    if stalls:
        print(f"  ! {stalls} gaps over 3x average -- the loop stalled there")
    stale = int(np.sum(np.diff(imu_t) == 0))
    if stale:
        print(f"  ! {stale} cycles reused the previous IMU sample "
              f"({100*stale/len(dt):.1f}%)")

    mag = np.linalg.norm(accel, axis=1)
    print(f"  |accel|  mean {mag.mean():.3f}  std {mag.std():.3f} m/s^2 "
          f"(still = {config.GRAVITY:.2f})")
    print(f"  gyro     max |rate| {np.round(np.abs(gyro).max(axis=0), 3)} rad/s")

    moving_any = False
    for i, mid in enumerate(config.MOTOR_IDS):
        vel = np.array([r['wheels'][i]['velocity_rads'] for r in rows])
        frac = np.mean(np.abs(vel) > config.ZUPT_WHEEL_SPEED_MAX)
        moving_any |= frac > 0.01
        print(f"  wheel {mid}  max {np.abs(vel).max():5.2f} rad/s, "
              f"moving {100*frac:4.1f}% of the log")
    print()
    return moving_any


def run(rows, use_zupt=True):
    odo = odometry.Odometry(use_zupt=use_zupt)
    history = []
    for k, r in enumerate(rows):
        # Real logs are not evenly spaced. Use the dt that actually
        # elapsed or Q is wrong on every stalled cycle.
        dt = (r['t'] - rows[k-1]['t']) if k else 1.0 / config.EKF_RATE_HZ
        if not (0 < dt < 0.5):
            dt = 1.0 / config.EKF_RATE_HZ
        odo.step(r['wheels'], gyro=r['gyro'], accel=r['accel'], dt=dt)
        snap = odo.snapshot()
        snap['t'] = r['t']
        history.append(snap)
    return odo, history


def report(odo, history, moving):
    end = history[-1]
    print("=" * 70)
    print("FILTER")
    print("=" * 70)
    print(f"  odom pose  {np.round(end['pos'], 3)} m  "
          "(wheel speed + ESKF attitude)")
    print(f"  odom vel   {np.round(end['vel'], 3)} m/s")
    print(f"  ESKF p     {np.round(end['filter_pos'], 3)} m  (diagnostic)")
    print(f"  ESKF v     {np.round(end['filter_vel'], 3)} m/s  (diagnostic)")
    print(f"  rpy        {np.round(end['rpy'], 2)} deg")
    print(f"  gyro bias  {np.round(odo.gyro_bias, 5)} rad/s "
          f"({np.round(np.degrees(odo.gyro_bias), 2)} deg/s)")
    print(f"  accel bias {np.round(odo.accel_bias, 4)} m/s^2")
    path = np.sum(np.linalg.norm(
        np.diff([h['pos'][:2] for h in history], axis=0), axis=1))
    print(f"  path       {path:.3f} m")
    print(f"  ZUPT       {odo.zupt_cycles}/{odo.cycles} cycles "
          f"({100*odo.zupt_cycles/odo.cycles:.1f}%)")
    print(f"  alignment  {'complete' if odo.ready else 'INCOMPLETE'}")

    # A stationary log carries the one piece of free ground truth there
    # is: we should have ended where we started.
    if odo.zupt_cycles > 0.9 * odo.cycles:
        drift = float(np.linalg.norm(end['filter_pos']))
        print(f"\n  Stationary throughout; internal ESKF p drift is:")
        print(f"  {drift:.4f} m  [{'GOOD' if drift < 0.05 else 'HIGH'}]")

    print()
    print("=" * 70)
    print("CONSISTENCY (NIS) -- no ground truth needed")
    print("=" * 70)
    print(f"  {'observation':12s} {'n':>6s} {'mean':>8s} {'target':>7s}   verdict")
    for name, samples in sorted(odo.nis.items()):
        dof = NIS_DOF.get(name, 1)
        mean = float(np.mean(samples))
        ratio = mean / dof
        if ratio > 3.0:
            verdict = "HIGH -- check model, timing, Q and R"
        elif ratio < 0.33:
            verdict = "LOW -- check noise assumptions and sample independence"
        else:
            verdict = "within heuristic band (not an accuracy test)"
        print(f"  {name:12s} {len(samples):6d} {mean:8.3f} {dof:7d}   {verdict}")

    if not moving:
        print("\n  NOTE: the wheels never turned in this log, so the 'wheel'")
        print("  and 'nhc' rows have almost no independent samples. Ignore")
        print("  their verdicts -- see docs 03 (tuning and NIS), section 6.1.")
    print()


def plot(history, filename):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = np.array([h['t'] for h in history])
    pos = np.array([h['pos'] for h in history])
    vel = np.array([h['vel'] for h in history])
    rpy = np.array([h['rpy'] for h in history])
    bg = np.degrees(np.array([h['bg'] for h in history]))
    stopped = np.array([h['stopped'] for h in history])

    fig, ax = plt.subplots(2, 2, figsize=(13, 9))

    ax[0, 0].plot(pos[:, 0], pos[:, 1], 'b-', lw=1.5)
    ax[0, 0].plot(*pos[0, :2], 'go', ms=9, label='start')
    ax[0, 0].plot(*pos[-1, :2], 'rs', ms=9, label='end')
    ax[0, 0].set_aspect('equal', 'datalim')
    ax[0, 0].set(xlabel='x (m)', ylabel='y (m)', title='path (top view)')

    for i, lbl in enumerate('xyz'):
        ax[0, 1].plot(t, vel[:, i], lw=1, label=f'v{lbl}')
    ax[0, 1].fill_between(t, -1, 1, where=stopped, alpha=0.15, color='green',
                          label='detected still')
    ax[0, 1].set(xlabel='t (s)', ylabel='m/s', title='velocity')

    for i, lbl in enumerate(['roll', 'pitch', 'yaw']):
        ax[1, 0].plot(t, rpy[:, i], lw=1, label=lbl)
    ax[1, 0].set(xlabel='t (s)', ylabel='deg', title='attitude')

    for i, lbl in enumerate('xyz'):
        ax[1, 1].plot(t, bg[:, i], lw=1, label=f'bg{lbl}')
    ax[1, 1].set(xlabel='t (s)', ylabel='deg/s',
                 title='gyro bias (should settle, then stay flat)')

    for a in ax.flat:
        a.legend(fontsize=8)
        a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(filename, dpi=110)
    print(f"plot saved: {filename}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("logfile")
    ap.add_argument("--plot", default=None, help="write a PNG here (../data/)")
    ap.add_argument("--no-zupt", action="store_true",
                    help="replay without ZUPT/ZARU, to see what they buy")
    args = ap.parse_args()

    rows = list(logger.replay(args.logfile))
    if not rows:
        sys.exit(f"{args.logfile} has no rows")
    if len(rows[0]['wheels']) != len(config.MOTOR_IDS):
        sys.exit(f"log has {len(rows[0]['wheels'])} wheels but config.MOTOR_IDS "
                 f"has {len(config.MOTOR_IDS)}. Set MOTOR_IDS to match the log.")

    moving = inspect(rows)
    odo, history = run(rows, use_zupt=not args.no_zupt)
    report(odo, history, moving)
    if args.plot:
        plot(history, args.plot)
