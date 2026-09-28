"""
Interactive, read-only replay of a recorded rover log.

Run from rover/:

    python replay_gui.py
    python replay_gui.py ../logs/2026-09-06_2347_drive.csv

The filter still runs through every original sample.  Only the browser payload
is reduced to 20 Hz, because drawing 100 nearly identical frames per second
adds load without adding visible information.

This program never opens a serial port and never writes to a motor.
"""
from collections import deque
import argparse
import json
import os
import socket
import threading
import webbrowser

from flask import Flask, jsonify, render_template, request
import numpy as np

import config
import logger
import odometry
from analyze_drive import segments as drive_segments
from config import NIS_DOF


DISPLAY_HZ = 20.0
DEFAULT_PORT = 5001
STAGE_NAMES = ('predict', 'wheel', 'nhc', 'zupt', 'zaru', 'gravity')

METADATA_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             os.pardir, "logs", "experiments.json")


def ensure_port_free(port):
    """Fail clearly instead of silently sharing a stale replay server."""
    probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    probe.settimeout(0.3)
    busy = probe.connect_ex(("127.0.0.1", port)) == 0
    probe.close()
    if busy:
        raise SystemExit(
            f"Port {port} already has a replay server. That process may "
            "still contain old filter code. Stop it, or open its existing "
            f"page at http://127.0.0.1:{port}."
        )


def _log_directory():
    return os.path.abspath(logger.LOG_DIR)


def experiment_metadata():
    """Manual measurements belong to the experiment, not to filter code."""
    try:
        with open(METADATA_FILE, "r", encoding="utf-8") as file:
            document = json.load(file)
    except FileNotFoundError:
        return {}
    if document.get("schema_version") != 1:
        raise ValueError("Unsupported logs/experiments.json schema version.")
    return document.get("runs", {})


def available_logs():
    """Return raw logs newest first. Derived files never appear here."""
    root = _log_directory()
    files = []
    metadata = experiment_metadata()
    for name in os.listdir(root):
        if not name.lower().endswith(".csv"):
            continue
        path = os.path.join(root, name)
        files.append({
            "name": name,
            "bytes": os.path.getsize(path),
            "modified": os.path.getmtime(path),
            "note": metadata.get(name, {}).get("description", ""),
        })
    return sorted(files, key=lambda item: item["modified"], reverse=True)


def resolve_log(name):
    """Accept a file name only; prevent the browser from reading other files."""
    if not name or os.path.basename(name) != name:
        raise ValueError("Choose a file from the logs directory.")
    path = os.path.abspath(os.path.join(_log_directory(), name))
    if os.path.commonpath([path, _log_directory()]) != _log_directory():
        raise ValueError("Invalid log path.")
    if not os.path.isfile(path):
        raise ValueError(f"Log not found: {name}")
    return path


def _number(value, digits=5):
    return round(float(value), digits)


def build_ground_truth(run_metadata, analysis_history, frames, first_t):
    """Pair tape-measured checkpoints with substantial detected drive legs."""
    profile = run_metadata.get("ground_truth")
    if profile is None:
        return None
    points = profile.get("checkpoints_m", [])
    if len(points) < 2 or any(len(point) != 2 for point in points):
        raise ValueError("ground_truth.checkpoints_m needs at least two [x, y] points.")

    # Ignore tiny joystick nudges. They are visible in the replay but were not
    # part of the tape-measured sequence (13 mm at 23:34, 9 mm at 23:47).
    legs = []
    for start, end in drive_segments(analysis_history):
        distance = float(np.linalg.norm(end[1][:2] - start[1][:2]))
        if distance >= 0.05:
            legs.append((start, end))

    expected_legs = len(points) - 1
    status = "matched"
    if len(legs) != expected_legs:
        status = f"detected {len(legs)} substantial legs; expected {expected_legs}"

    events = [{
        "number": 0,
        "t": 0.0,
        "frame": 0,
        "truth": [_number(x, 3) for x in points[0]],
        "estimate": frames[0]["pos"][:2],
        "x_error": _number(frames[0]["pos"][0] - points[0][0], 4),
        "y_error": _number(frames[0]["pos"][1] - points[0][1], 4),
        "error": _number(np.linalg.norm(np.asarray(frames[0]["pos"][:2])
                                         - np.asarray(points[0])), 4),
    }]
    frame_times = np.asarray([frame["t"] for frame in frames])
    for number, (truth_point, leg) in enumerate(
            zip(points[1:], legs), start=1):
        end_t, end_position = leg[1]
        relative_t = float(end_t - first_t)
        frame_index = int(np.argmin(np.abs(frame_times - relative_t)))
        truth_xy = np.asarray(truth_point, dtype=float)
        estimate_xy = np.asarray(end_position[:2])
        events.append({
            "number": number,
            "t": _number(relative_t, 2),
            "frame": frame_index,
            "truth": [_number(x, 3) for x in truth_xy],
            "estimate": [_number(x, 4) for x in estimate_xy],
            "x_error": _number(estimate_xy[0] - truth_xy[0], 4),
            "y_error": _number(estimate_xy[1] - truth_xy[1], 4),
            "error": _number(np.linalg.norm(estimate_xy - truth_xy), 4),
        })

    errors = [event["error"] for event in events[1:]]
    return {
        "label": profile.get("label", "measured checkpoints"),
        "tolerance": float(profile.get("tolerance_m", 0.05)),
        "status": status,
        "events": events,
        "max_error": _number(max(errors), 4) if errors else None,
        "matched": len(events) - 1,
        "expected": expected_legs,
    }


def build_replay(path):
    """
    Run the production odometry cycle at the original log rate, then prepare a
    compact set of display frames. No filter math is duplicated here.
    """
    rows = list(logger.replay(path))
    run_metadata = experiment_metadata().get(os.path.basename(path), {})
    if not rows:
        raise ValueError("The log has no data rows.")
    if len(rows[0]["wheels"]) != len(config.MOTOR_IDS):
        raise ValueError(
            f"Log has {len(rows[0]['wheels'])} wheels, config expects "
            f"{len(config.MOTOR_IDS)}."
        )

    # Every raw cycle records lightweight yaw attribution. Display cycles also
    # record the full calculation, retained server-side and fetched on demand.
    odo = odometry.Odometry(trace='yaw')
    frames = []
    traces = []
    full_path = []
    full_filter_path = []
    analysis_history = []
    recent_nis = {name: deque(maxlen=300) for name in NIS_DOF}
    nis_lengths = {name: 0 for name in NIS_DOF}
    next_frame_t = rows[0]["t"]
    path_length = 0.0
    previous_xy = None
    yaw_attribution = {name: 0.0 for name in STAGE_NAMES}

    for index, row in enumerate(rows):
        dt = ((row["t"] - rows[index - 1]["t"])
              if index else 1.0 / config.EKF_RATE_HZ)
        if not 0.0 < dt < 0.5:
            dt = 1.0 / config.EKF_RATE_HZ

        is_last = index == len(rows) - 1
        capture_detail = row["t"] + 1e-9 >= next_frame_t or is_last
        odo.trace_enabled = True if capture_detail else 'yaw'
        odo.step(row["wheels"], gyro=row["gyro"], accel=row["accel"], dt=dt)

        trace = odo.last_trace
        for stage in trace["stages"]:
            if stage.get("applied") and stage["name"] in yaw_attribution:
                yaw_attribution[stage["name"]] += stage["state_delta"]["rpy_deg"][2]

        xy = odo.position[:2].copy()
        mean_wheel_rate = float(np.mean([
            abs(wheel["velocity_rads"]) for wheel in row["wheels"]
        ]))
        if previous_xy is not None and mean_wheel_rate > 0.05:
            path_length += float(np.linalg.norm(xy - previous_xy))
        previous_xy = xy
        analysis_history.append((row["t"], odo.position.copy(),
                                 odo.stopped, mean_wheel_rate))

        for name in NIS_DOF:
            samples = odo.nis.get(name, [])
            old_length = nis_lengths[name]
            if len(samples) > old_length:
                recent_nis[name].extend(samples[old_length:])
                nis_lengths[name] = len(samples)

        if row["t"] + 1e-9 < next_frame_t and not is_last:
            continue
        next_frame_t = row["t"] + 1.0 / DISPLAY_HZ

        wheel_mps = odometry.wheel_speeds_mps(row["wheels"])
        frame_nis = {
            name: _number(np.mean(values), 3)
            for name, values in recent_nis.items() if values
        }
        frame = {
            "t": _number(row["t"] - rows[0]["t"], 3),
            "pos": [_number(x, 4) for x in odo.position],
            "vel": [_number(x, 4) for x in odo.velocity],
            "filter_pos": [_number(x, 4) for x in odo.inertial_position],
            "filter_vel": [_number(x, 4) for x in odo.inertial_velocity],
            "rpy": [_number(x, 3) for x in odo.rpy_deg],
            "bg": [_number(np.degrees(x), 4) for x in odo.gyro_bias],
            "ba": [_number(x, 4) for x in odo.accel_bias],
            "wheel": [_number(x, 4) for x in wheel_mps],
            "current": [_number(w["current_A"], 4) for w in row["wheels"]],
            "accel": [_number(x, 4) for x in row["accel"]],
            "gyro": [_number(np.degrees(x), 4) for x in row["gyro"]],
            "stopped": bool(odo.zupt_active),
            "ready": bool(odo.ready),
            "alignment_progress": _number(odo.alignment_progress, 3),
            "nis": frame_nis,
            "yaw_sources": {
                name: _number(value, 5)
                for name, value in yaw_attribution.items()
            },
        }
        frames.append(frame)
        traces.append(trace)
        full_path.append(frame["pos"][:2])
        full_filter_path.append(frame["filter_pos"][:2])

    t = np.asarray([row["t"] for row in rows], dtype=float)
    dt = np.diff(t)
    timing_dt = dt[1:] if len(dt) > 1 else dt
    duration = float(t[-1] - t[0])
    rate = float(1.0 / np.mean(timing_dt)) if len(timing_dt) else 0.0
    max_gap_ms = float(1000.0 * np.max(timing_dt)) if len(timing_dt) else 0.0
    p99_gap_ms = (float(1000.0 * np.percentile(timing_dt, 99))
                  if len(timing_dt) else 0.0)
    stale_imu = int(np.sum(np.diff([row["imu_t"] for row in rows]) == 0.0))
    moving = any(
        abs(w["velocity_rads"]) > config.ZUPT_WHEEL_SPEED_MAX
        for row in rows for w in row["wheels"]
    )
    aggregate_nis = {
        name: {
            "mean": _number(np.mean(samples), 3),
            "target": dof,
            "count": len(samples),
        }
        for name, dof in NIS_DOF.items()
        if (samples := odo.nis.get(name, []))
    }

    end = frames[-1]
    truth = build_ground_truth(run_metadata, analysis_history,
                               frames, rows[0]["t"])
    summary = {
        "samples": len(rows),
        "display_frames": len(frames),
        "duration": _number(duration, 2),
        "rate": _number(rate, 2),
        "p99_gap_ms": _number(p99_gap_ms, 2),
        "max_gap_ms": _number(max_gap_ms, 2),
        "stale_imu": stale_imu,
        "path_length": _number(path_length, 3),
        "final_position": end["pos"],
        "final_filter_position": end["filter_pos"],
        "final_rpy": end["rpy"],
        "return_error": _number(np.linalg.norm(end["pos"][:2]), 3),
        "zupt_percent": _number(100.0 * odo.zupt_cycles / odo.cycles, 1),
        "alignment_complete": bool(odo.ready),
        "moving": moving,
        "nis": aggregate_nis,
        "yaw_attribution_deg": {
            name: _number(value, 4)
            for name, value in yaw_attribution.items()
        },
        "legacy_imu_frame_corrected": any(
            row.get("source_frame") == "arduino_sensor_legacy" for row in rows
        ),
    }
    return {
        "file": os.path.basename(path),
        "note": run_metadata.get("description", ""),
        "ids": config.MOTOR_IDS,
        "summary": summary,
        "frames": frames,
        "path": full_path,
        "filter_path": full_filter_path,
        "truth": truth,
        "_traces": traces,
    }


def create_app(preferred=""):
    app = Flask(__name__)
    cache = {}

    @app.get("/")
    def index():
        return render_template("replay.html", preferred=preferred,
                               nis_dof=NIS_DOF)

    @app.get("/api/logs")
    def logs_api():
        return jsonify({"logs": available_logs()})

    @app.get("/api/replay")
    def replay_api():
        try:
            path = resolve_log(request.args.get("file", ""))
            modified = os.path.getmtime(path)
            metadata_modified = (os.path.getmtime(METADATA_FILE)
                                 if os.path.exists(METADATA_FILE) else 0.0)
            key = (path, modified, metadata_modified)
            if key not in cache:
                cache.clear()  # one loaded run is enough for a local tool
                cache[key] = build_replay(path)
            payload = {name: value for name, value in cache[key].items()
                       if name != "_traces"}
            return jsonify(payload)
        except (OSError, ValueError, KeyError) as error:
            return jsonify({"error": str(error)}), 400

    @app.get("/api/trace")
    def trace_api():
        try:
            path = resolve_log(request.args.get("file", ""))
            modified = os.path.getmtime(path)
            metadata_modified = (os.path.getmtime(METADATA_FILE)
                                 if os.path.exists(METADATA_FILE) else 0.0)
            key = (path, modified, metadata_modified)
            if key not in cache:
                cache.clear()
                cache[key] = build_replay(path)
            index = int(request.args.get("index", "-1"))
            traces = cache[key]["_traces"]
            if not 0 <= index < len(traces):
                raise ValueError("Trace frame index is out of range.")
            return jsonify(traces[index])
        except (OSError, ValueError, KeyError) as error:
            return jsonify({"error": str(error)}), 400

    return app


def main():
    parser = argparse.ArgumentParser(description="Browser replay for rover CSV logs")
    parser.add_argument("logfile", nargs="?", help="optional log in ../logs/")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args()

    preferred = ""
    if args.logfile:
        path = os.path.abspath(args.logfile)
        if os.path.dirname(path) != _log_directory():
            raise SystemExit("The replay GUI only opens raw files from logs/.")
        preferred = os.path.basename(path)
        resolve_log(preferred)

    url = f"http://127.0.0.1:{args.port}"
    ensure_port_free(args.port)
    print(f"Rover replay GUI: {url}")
    print("Read-only: no COM ports or motors are opened.")
    if not args.no_browser:
        threading.Timer(0.8, lambda: webbrowser.open(url)).start()
    create_app(preferred).run(host="127.0.0.1", port=args.port, debug=False)


if __name__ == "__main__":
    main()
