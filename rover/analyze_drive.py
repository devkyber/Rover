"""
Measure a drive log against a known ground truth.

    python analyze_drive.py ../logs/2026-09-06_2334_drive.csv --reach 1.20 --closes

Why this exists and replay.py does not answer it: replay's `path` is the
sum of |dp| over every cycle, so estimator jitter while the rover is
PARKED still adds to it. Over 20000 cycles that is metres of fake
distance. The honest numbers for an out-and-back run are:

    reach        how far from the start it actually got   <- scale error
    return       where it thinks it ended                 <- total drift
    lateral      sideways excursion on a straight line    <- heading error

Those three are all measurable with a tape measure, which is the point.
"""
import argparse
import sys

import numpy as np

import config
import logger
import odometry


def run(path):
    rows = list(logger.replay(path))
    odo = odometry.Odometry()
    t_prev, dt0 = 0.0, 1.0 / config.EKF_RATE_HZ
    hist = []
    for r in rows:
        step = r['t'] - t_prev if odo.cycles else dt0
        t_prev = r['t']
        odo.step(r['wheels'], gyro=r['gyro'], accel=r['accel'],
                 dt=step if 0 < step < 0.5 else dt0)
        hist.append((r['t'], odo.position.copy(), odo.stopped,
                     float(np.mean([abs(w['velocity_rads'])
                                    for w in r['wheels']]))))
    return odo, hist


def segments(hist, min_still=1.0):
    """Split the run into moving legs separated by >=min_still s of stillness."""
    legs, start, moving = [], None, False
    still_since = None
    still_position = None
    for t, p, stopped, wspeed in hist:
        rolling = wspeed > 0.05
        if rolling:
            if not moving:
                moving, start = True, (t, p)
            # A pause shorter than min_still is part of the same leg.
            still_since = None
            still_position = None
        elif moving:
            if still_since is None:
                still_since = t
                still_position = p.copy()
            elif t - still_since >= min_still:
                # Time and position must come from the same first-still sample.
                legs.append((start, (still_since, still_position)))
                moving = False
    if moving and start is not None:
        legs.append((start, (hist[-1][0], hist[-1][1])))
    return legs


def main(a):
    odo, hist = run(a.log)
    P = np.array([h[1] for h in hist])
    T = np.array([h[0] for h in hist])

    dist = np.linalg.norm(P[:, :2] - P[0, :2], axis=1)
    i_far = int(np.argmax(dist))

    print("=" * 66)
    print(f"  {a.log}")
    print(f"  {len(hist)} samples, {T[-1]:.1f} s")
    print("=" * 66)
    print(f"  reach        {dist[i_far]:.3f} m  at t={T[i_far]:.1f} s")
    print(f"  return       {np.linalg.norm(P[-1, :2]):.3f} m from start")
    print(f"  lateral max  {np.max(np.abs(P[:, 1])):.3f} m")
    print(f"  yaw          {odo.rpy_deg[2]:+.2f} deg")
    print(f"  ZUPT         {100*odo.zupt_cycles/max(odo.cycles,1):.1f}%")

    if a.reach:
        err = dist[i_far] - a.reach
        print(f"\n  vs tape {a.reach:.2f} m:  {err:+.3f} m "
              f"({100*err/a.reach:+.1f}%)   {verdict(abs(err), a.tol)}")
    if a.closes:
        e = np.linalg.norm(P[-1, :2])
        print(f"  returns to start:    {e:.3f} m           {verdict(e, a.tol)}")

    legs = segments(hist)
    if legs:
        print(f"\n  {len(legs)} legs (moving stretches separated by a stop):")
        print("   #   t_start  t_end   leg length   cumulative from start")
        for k, ((t0, p0), (t1, p1)) in enumerate(legs, 1):
            print(f"  {k:2d}   {t0:6.1f}  {t1:6.1f}   "
                  f"{np.linalg.norm(p1[:2]-p0[:2]):8.3f} m   "
                  f"{np.linalg.norm(p1[:2]-P[0,:2]):8.3f} m")

    if a.plot:
        plot(P, T, hist, a.plot, a.log)
        print(f"\n  plot -> {a.plot}")


def verdict(err, tol):
    return "[OK]" if err <= tol else f"[OUT by {err-tol:+.3f} m]"


def plot(P, T, hist, out, title):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(1, 3, figsize=(16, 4.6))
    ax[0].plot(P[:, 0], P[:, 1], lw=1.2)
    ax[0].plot(P[0, 0], P[0, 1], 'go', label='start')
    ax[0].plot(P[-1, 0], P[-1, 1], 'rx', ms=10, label='end')
    ax[0].set_aspect('equal'); ax[0].grid(alpha=.3); ax[0].legend()
    ax[0].set_title('path (top view)'); ax[0].set_xlabel('x [m]')
    ax[0].set_ylabel('y [m]')

    ax[1].plot(T, np.linalg.norm(P[:, :2] - P[0, :2], axis=1))
    ax[1].grid(alpha=.3); ax[1].set_title('distance from start [m]')
    ax[1].set_xlabel('t [s]')

    stopped = np.array([h[2] for h in hist], dtype=float)
    ax[2].plot(T, P[:, 0], label='x')
    ax[2].plot(T, P[:, 1], label='y')
    ax[2].fill_between(T, -9, 9, where=stopped > 0, alpha=.12,
                       color='g', label='still')
    ax[2].set_ylim(min(P[:, :2].min() - .1, -.2), max(P[:, :2].max() + .1, .2))
    ax[2].grid(alpha=.3); ax[2].legend(); ax[2].set_title('x, y vs time')
    ax[2].set_xlabel('t [s]')

    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(out, dpi=110)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--reach", type=float, default=None,
                    help="tape-measured furthest distance, m")
    ap.add_argument("--closes", action="store_true",
                    help="the run ended where it started")
    ap.add_argument("--tol", type=float, default=0.05)
    ap.add_argument("--plot", default=None)
    sys.exit(main(ap.parse_args()))
