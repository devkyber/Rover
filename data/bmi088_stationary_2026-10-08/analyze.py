"""Reproduce the stationary BMI088 bench analysis; no runtime calibration writes."""
import hashlib
import json
import platform
import struct
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
RAW = ROOT / "logs/2026-10-08_011054_bmi088_stationary.json"


def main():
    raw_bytes = RAW.read_bytes()
    r = json.loads(raw_bytes)
    assert hashlib.sha256(r["executed_source"].encode()).hexdigest() == r["source_sha256"]
    assert r["completed"] and len(r["samples"]) == 6001
    assert r["initial_ids"] == r["post_reset_ids"] == r["final_ids"] == [30, 15]
    assert all((v["actual"] & v["mask"]) == v["expected"]
               for group in (r["readback"], r["final_readback"]) for v in group)
    assert all(v["ids"] == [30, 15] and v["accel_error"] == 0
               and v["accel_power_enable"] == 4 for v in r["monitor"])
    samples = r["samples"]
    for s in samples:
        assert list(struct.unpack("<hhh", bytes(s["accel_rx"][:6]))) == s["accel_raw"]
        assert list(struct.unpack("<hhh", bytes(s["gyro_rx"]))) == s["gyro_raw"]
    gyro = np.asarray([s["gyro_raw"] for s in samples], dtype=float) * 500 / 32768
    accel = np.asarray([s["accel_raw"] for s in samples], dtype=float) * 6 * 9.80665 / 32768
    t = np.asarray([s["host_gyro_mid_ns"] - samples[0]["host_gyro_mid_ns"]
                    for s in samples], dtype=float) / 1e9
    dt = np.diff(t)
    tick_steps = np.diff([s["accel_sensor_ticks"] for s in samples]) % (1 << 24)
    assert np.all(dt > 0) and np.all((tick_steps > 0) & (tick_steps < 1024))
    accel_host_t = np.asarray([s["host_accel_mid_ns"] - samples[0]["host_accel_mid_ns"]
                              for s in samples], dtype=float) / 1e9
    sensor_t = np.r_[0, np.cumsum(tick_steps)] * r["config"]["accel_sensor_time_tick_s"]
    clock_slope, clock_intercept = np.polyfit(accel_host_t, sensor_t, 1)
    clock_residual = sensor_t - (clock_slope * accel_host_t + clock_intercept)
    # Fixed split: first 3,000 samples estimate bias; remaining 3,001 are withheld.
    split = 3000
    train_bias = gyro[:split].mean(axis=0)
    up = accel[:split].mean(axis=0)
    up /= np.linalg.norm(up)
    residual = gyro[split:] - train_bias
    increments = (residual[1:] + residual[:-1]) * 0.5 * np.diff(t[split:])[:, None]
    integrated = np.vstack((np.zeros(3), np.cumsum(increments, axis=0)))
    vertical_angle = integrated @ up
    norms = np.linalg.norm(accel, axis=1)
    mean = gyro.mean(axis=0)
    # The capture recorded the last monitor twice; retain the last reading per index.
    monitor = {v["sample_index"]: v for v in r["monitor"]}
    mt = np.asarray([t[i] for i in sorted(monitor)])
    temp = np.asarray([monitor[i]["temperature"]["celsius"] for i in sorted(monitor)])
    blocks = []
    for i in range(6):
        block = gyro[i * 1000:(i + 1) * 1000]
        blocks.append({"start_s_nominal": i * 10, "count": len(block),
                       "mean_dps": block.mean(axis=0).tolist(),
                       "std_dps": block.std(axis=0).tolist()})
    out = {
        "input": str(RAW.relative_to(ROOT)).replace("\\", "/"),
        "input_sha256": hashlib.sha256(raw_bytes).hexdigest(),
        "capture_source_sha256": r["source_sha256"],
        "analysis_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "analysis_versions": {"python": platform.python_version(), "numpy": np.__version__,
                              "matplotlib": matplotlib.__version__},
        "samples": len(samples), "duration_s": float(t[-1]),
        "config": r["config"], "frame": "sensor", "runtime_calibration_applied": False,
        "full_record_mean_gyro_dps": mean.tolist(),
        "full_record_mean_gyro_rads": np.deg2rad(mean).tolist(),
        "gyro_std_dps": gyro.std(axis=0).tolist(), "ten_second_blocks": blocks,
        "temperature_minmax_c": [float(temp.min()), float(temp.max())],
        "temperature_start_end_c": [r["temperature_before"]["celsius"], r["temperature_after"]["celsius"]],
        "mean_accel_mss": accel.mean(axis=0).tolist(),
        "accel_norm_mean_mss": float(norms.mean()), "accel_norm_std_mss": float(norms.std()),
        "accel_norm_difference_from_standard_g_percent": float((norms.mean() / 9.80665 - 1) * 100),
        "gyro_poll_interval_ms_min_p50_p95_p99_max": (np.percentile(dt, [0, 50, 95, 99, 100]) * 1000).tolist(),
        "gyro_poll_intervals_over_15ms": int(np.sum(dt > 0.015)),
        "sensor_tick_step_minmax": [int(tick_steps.min()), int(tick_steps.max())],
        "sensor_time_span_s": float(tick_steps.sum() * r["config"]["accel_sensor_time_tick_s"]),
        "accel_clock_fit": {"nominal_sensor_seconds_per_host_second": float(clock_slope),
                            "relative_rate_difference_percent": float((clock_slope - 1) * 100),
                            "intercept_s": float(clock_intercept),
                            "residual_std_us": float(clock_residual.std() * 1e6),
                            "scope": "Relative clock estimate for this capture, not a permanent correction"},
        "withheld_second_half": {
            "training_samples": split, "test_samples": len(samples) - split,
            "training_bias_dps": train_bias.tolist(), "duration_s": float(t[-1] - t[split]),
            "residual_integral_sensor_deg": integrated[-1].tolist(),
            "up_direction_estimated_from_training_accel": up.tolist(),
            "vertical_projection_final_deg": float(vertical_angle[-1]),
            "vertical_projection_max_abs_deg": float(np.max(np.abs(vertical_angle))),
            "scope": "Same stationary capture, later time segment; not independent turn/yaw validation",
        },
        "limitations": ["One stationary orientation and narrow temperature range",
                        "Sample standard deviations are not noise densities or Allan-deviation fits",
                        "Sensor output rates are 400 Hz; host polls at 100 Hz without hardware sync",
                        "No sensor-to-body mounting transform or gyro scale calibration established",
                        "No accelerometer offset/scale fitted from a single gravity orientation"],
    }
    (HERE / "summary.json").write_text(json.dumps(out, indent=2) + "\n", encoding="utf-8")

    plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False})
    fig, axes = plt.subplots(5, 1, figsize=(11, 12), constrained_layout=True)
    colors = ["#007c91", "#b86000", "#7040a0"]
    block_means = np.asarray([b["mean_dps"] for b in blocks])
    for k, label in enumerate("XYZ"):
        axes[0].plot(t[::5], gyro[::5, k], color=colors[k], lw=0.6, alpha=0.55, label=label)
        axes[0].plot(np.arange(6) * 10 + 5, block_means[:, k], color=colors[k], lw=2)
    axes[0].set(title="BMI088 stationary capture | sensor axes | 6,001 samples",
                ylabel="Gyro (deg/s)", xlim=(0, 60))
    axes[0].legend(ncol=3, loc="upper right")
    axes[0].text(0.01, 0.03, "Thin: raw samples (display decimated). Thick: 10-second means.",
                 transform=axes[0].transAxes, fontsize=9)
    axes[1].plot(t, norms, color="#007c91", lw=0.6)
    axes[1].axhline(9.80665, color="#b86000", ls="--", label="Standard gravity")
    axes[1].set(ylabel="|accel| (m/s^2)", xlim=(0, 60))
    axes[1].legend(loc="upper right")
    axes[2].step(mt, temp, where="post", color="#b86000")
    axes[2].set(ylabel="Temperature (deg C)", xlim=(0, 60))
    axes[3].plot(t[split:] - t[split], vertical_angle, color="#7040a0")
    axes[3].axhline(0, color="#888888", lw=0.6)
    axes[3].set(xlabel="Seconds in withheld second half", ylabel="Residual angle (deg)",
                title="First-half bias removed; fixed vertical projection (stationary diagnostic)")
    axes[4].plot(accel_host_t, sensor_t - accel_host_t, color="#007c91")
    axes[4].set(xlabel="Host elapsed seconds", ylabel="Clock difference (s)",
                title="Nominal accel sensor time minus host elapsed time", xlim=(0, 60))
    for ax in axes:
        ax.grid(True, alpha=0.2)
    fig.savefig(HERE / "stationary.png", dpi=160)
    plt.close(fig)
    print(json.dumps(out, indent=2))


if __name__ == "__main__":
    main()
