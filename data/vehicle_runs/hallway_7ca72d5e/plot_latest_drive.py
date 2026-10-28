"""Plot immutable, captured runtime scalar outputs; no estimator replay or hardware."""
from pathlib import Path
from datetime import datetime, timedelta, timezone
import csv
import hashlib
import json
import math
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
import argparse
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('output', type=Path, help='New output directory; must not exist')
args = parser.parse_args()
OUTPUT = args.output.resolve()
OUTPUT.mkdir(parents=True, exist_ok=False)
CAPTURE = ROOT / "logs/drives/hallway_7ca72d5e"
RUN_ID = "7ca72d5ef4044b1cadaad6afc84820c3"
CSV_ROOT = CAPTURE / "session/rover"
STEM = "hallway_7ca72d5e"
KST = timezone(timedelta(hours=9))


def provenance(path):
    return {"path": str(path.resolve()) if not path.resolve().is_relative_to(ROOT) else path.relative_to(ROOT).as_posix(), "bytes": path.stat().st_size,
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def number(row, key):
    try:
        value = float(row.get(key, ""))
        return value if math.isfinite(value) else math.nan
    except (TypeError, ValueError):
        return math.nan


def main():
    csv_paths = sorted(CSV_ROOT.glob("system_*.csv"))
    if not csv_paths:
        raise ValueError("No captured scalar CSV files")
    rows, excluded = [], []
    headers = None
    for path in csv_paths:
        with path.open(newline="", encoding="utf-8") as stream:
            reader = csv.DictReader(stream)
            if headers is None:
                headers = reader.fieldnames
            if headers != reader.fieldnames:
                raise ValueError(f"CSV header mismatch: {path.name}")
            for line, row in enumerate(reader, start=2):
                if None in row or any(value is None for value in row.values()):
                    excluded.append({"path": str(path.resolve()) if not path.resolve().is_relative_to(ROOT) else path.relative_to(ROOT).as_posix(), "line": line,
                                     "reason": "Incomplete CSV row"})
                    continue
                if row.get("run_id") != RUN_ID:
                    raise ValueError(f"Unexpected run identity: {row.get('run_id')}")
                if not math.isfinite(number(row, "captured_monotonic_s")):
                    raise ValueError(f"Missing captured monotonic clock: {path.name}:{line}")
                rows.append(row)
    rows.sort(key=lambda row: number(row, "captured_monotonic_s"))
    if not rows:
        raise ValueError("No usable captured rows")
    clock = np.array([number(row, "captured_monotonic_s") for row in rows])
    if np.any(np.diff(clock) <= 0):
        raise ValueError("Duplicate or non-increasing captured monotonic timestamps")
    elapsed = clock - clock[0]
    minutes = elapsed / 60
    median_step = float(np.median(np.diff(clock)))
    gap_limit = max(0.25, 5 * median_step)
    get = lambda key: np.array([number(row, key) for row in rows])
    missing = []

    plt.rcParams.update({"font.size": 10, "axes.spines.top": False,
                         "axes.spines.right": False, "figure.facecolor": "#f7f8fa",
                         "axes.facecolor": "white"})
    fig, axes = plt.subplots(2, 2, figsize=(14, 9))
    fig.subplots_adjust(left=.075, right=.92, top=.88, bottom=.14,
                        hspace=.43, wspace=.34)

    wheel_ax = axes[0, 0]
    for name, color in zip(("fl", "fr", "rl", "rr"),
                           ("#2563eb", "#d97706", "#047857", "#9333ea")):
        key = f"wheel_{name}_speed_mps"
        if key in headers:
            wheel_ax.plot(minutes, get(key), lw=.7, alpha=.8, label=name.upper(), color=color)
        else:
            missing.append(key)
    wheel_ax.set(title="Recorded wheel velocity", xlabel="Elapsed from first captured row (min)",
                 ylabel="Encoder speed (m/s)")
    wheel_ax.legend(ncol=4, fontsize=8)

    battery_ax = axes[0, 1]
    current_ax = battery_ax.twinx()
    current_ax.spines["right"].set_visible(True)
    if "battery_v" in headers:
        battery_ax.plot(minutes, get("battery_v"), color="#2563eb", lw=.8)
    else:
        missing.append("battery_v")
    if "battery_a" in headers:
        current_ax.plot(minutes, get("battery_a"), color="#b45309", lw=.6, alpha=.55)
    else:
        missing.append("battery_a")
    battery_ax.set(title="Recorded pack voltage and current",
                   xlabel="Elapsed from first captured row (min)", ylabel="Voltage (V)")
    current_ax.set_ylabel("Current (A)", color="#b45309")
    battery_ax.tick_params(axis="y", labelcolor="#2563eb")
    current_ax.tick_params(axis="y", labelcolor="#b45309")

    path_ax = axes[1, 0]
    required_pose = {"estimate_pos_x_m", "estimate_pos_y_m", "pose_estimate_available",
                     "pose_frame_id", "pose_loss_count"}
    missing_pose = sorted(required_pose - set(headers))
    path_breaks = 0
    pose_points = 0
    frames = set()
    if missing_pose:
        missing.extend(missing_pose)
        path_ax.text(.5, .5, "Path unavailable: missing recorded pose fields",
                     transform=path_ax.transAxes, ha="center", va="center")
    else:
        xx, yy = [], []
        previous = None
        for index, row in enumerate(rows):
            x, y = number(row, "estimate_pos_x_m"), number(row, "estimate_pos_y_m")
            available = (row["pose_estimate_available"] == "True"
                         and math.isfinite(x) and math.isfinite(y)
                         and bool(row["pose_frame_id"]))
            if not available:
                xx.append(math.nan)
                yy.append(math.nan)
                previous = None
                continue
            frame = row["pose_frame_id"]
            frames.add(frame)
            loss = number(row, "pose_loss_count")
            if previous is not None:
                prior_i, prior_frame, prior_loss = previous
                if (frame != prior_frame or loss != prior_loss
                        or clock[index] - clock[prior_i] > gap_limit):
                    xx.append(math.nan)
                    yy.append(math.nan)
                    path_breaks += 1
            xx.append(x)
            yy.append(y)
            pose_points += 1
            previous = index, frame, loss
        if len(frames) > 1:
            path_ax.text(.5, .5, "Multiple local pose frames: no common path shown",
                         transform=path_ax.transAxes, ha="center", va="center")
        elif pose_points:
            path_ax.plot(xx, yy, lw=.75, color="#d97706", label="Original display estimate")
            finite = np.flatnonzero(np.isfinite(xx) & np.isfinite(yy))
            first, last = finite[0], finite[-1]
            path_ax.scatter([xx[first]], [yy[first]], s=35, c="#111827", zorder=4, label="First available")
            path_ax.scatter([xx[last]], [yy[last]], s=35, c="#dc2626", zorder=4, label="Last available")
            path_ax.legend(fontsize=8, loc="best")
        else:
            path_ax.text(.5, .5, "No available recorded display position",
                         transform=path_ax.transAxes, ha="center", va="center")
    path_ax.set(title="Original local position estimate | accuracy unverified",
                xlabel="Local X estimate (m)", ylabel="Local Y estimate (m)")
    path_ax.set_aspect("equal", adjustable="datalim")

    radio_ax = axes[1, 1]
    if "wifi_dbm" in headers:
        radio_ax.plot(minutes, get("wifi_dbm"), lw=.8, color="#0369a1")
    else:
        missing.append("wifi_dbm")
    radio_ax.set(title="Recorded Rover-side Wi-Fi signal",
                 xlabel="Elapsed from first captured row (min)", ylabel="RSSI (dBm)")
    for axis in axes.flat:
        axis.grid(alpha=.2)

    first_utc = datetime.fromisoformat(rows[0]["captured_host_utc"])
    last_utc = datetime.fromisoformat(rows[-1]["captured_host_utc"])
    first_kst, last_kst = first_utc.astimezone(KST), last_utc.astimezone(KST)
    heading = "Retained hallway driving capture"
    clock_label = f"{first_kst:%d Oct %Y, %H:%M:%S} to {last_kst:%H:%M:%S} KST"
    fig.suptitle(f"{heading} | run {RUN_ID[:8]}\n{clock_label} | "
                 f"{elapsed[-1]/60:.1f} min captured", fontsize=15, weight="bold")
    route_note = "Hallway route ~15 m is the operator's estimate, not a measured ground distance. "
    footer = ("Original runtime outputs only; no replay. Partial retained session; no closing summary.\n"
              + route_note + "Local estimated position is not a surveyed route.\n"
              "Path breaks at unavailable/missing pose, local-frame or loss-count changes, "
              f"and captured time gaps > {gap_limit:.3f} s. Camera delivery is not measured here.")
    fig.text(.075, .045, footer, fontsize=8, color="#4b5563", linespacing=1.6)
    fig.savefig(OUTPUT / (STEM + ".png"), dpi=160)
    plt.close(fig)

    inputs = csv_paths + [CAPTURE / "session/metadata.json", CAPTURE / "operator_labels.json"]
    summary = {
        "schema": "rover_latest_drive_delivery_v1", "run_id": RUN_ID,
        "method": "Original captured scalar outputs only; no estimator replay or hardware access.",
        "sources": [provenance(path) for path in inputs],
        "first_recorded_host_utc": first_utc.isoformat(), "last_recorded_host_utc": last_utc.isoformat(),
        "host_calendar_valid": True,
        "first_captured_kst": first_kst.isoformat(),
        "last_captured_kst": last_kst.isoformat(),
        "first_monotonic_s": float(clock[0]), "last_monotonic_s": float(clock[-1]),
        "first_runtime_t_s": number(rows[0], "t_s"), "last_runtime_t_s": number(rows[-1], "t_s"),
        "rows": len(rows), "duration_s": float(elapsed[-1]), "excluded_rows": excluded,
        "missing_plot_columns": sorted(set(missing)),
        "pose_path": {"available_points": pose_points, "local_frames": sorted(frames),
                      "explicit_breaks": path_breaks, "capture_gap_threshold_s": gap_limit,
                      "rule": "Only pose_estimate_available=True and finite XY with nonempty frame. "
                              "Break on unavailable pose, frame/loss-count change, or long captured gap. "
                              "Suppress combined path if multiple frames are present."},
        "limitations": ["Partial captured session; no closing session summary was captured.",
                        "Original display position accuracy is unverified; it is not a measured route.",
                        "Approximately 15 m is an operator estimate, not an independent distance measurement.",
                        "Host UTC/KST values are recorded clock observations; sensor synchronization is not established.",
                        "Wheel encoder speed is not independent ground-motion truth; RSSI is not packet loss.",
                        "Latest is limited to the available local retained driving evidence."],
        "outputs": [provenance(OUTPUT / (STEM + ".png")), provenance(Path(__file__).resolve())],
    }
    (OUTPUT / (STEM + "_analysis.json")).write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"run_id": RUN_ID, "rows": len(rows), "output": str(OUTPUT),
                      "pose_points": pose_points, "path_breaks": path_breaks,
                      "local_frame_count": len(frames), "missing_plot_columns": sorted(set(missing))}))


if __name__ == "__main__":
    main()
