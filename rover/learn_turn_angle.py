"""Offline lesson: follow a known turn through the production gyro predictor.

Run from the project root: python -B rover/learn_turn_angle.py
Read docs/09_turn_angle_lesson.md for a line-by-line explanation.
This generates ideal samples in memory; it does not access hardware or logs.
"""
import math

import numpy as np

import config
import eskf


def main():
    # Change one of these values at a time, then predict the result on paper.
    true_rate_deg_s = 30.0
    sensor_bias_deg_s = 0.5
    estimated_bias_deg_s = 0.5
    dt_s = 0.01
    steps = 300

    state = eskf.create_state()
    state['bg'][2] = math.radians(estimated_bias_deg_s)
    gyro_raw = np.array([0.0, 0.0, math.radians(true_rate_deg_s + sensor_bias_deg_s)])
    accel_raw = np.array([0.0, 0.0, config.GRAVITY])
    raw_angle_rad = 0.0
    corrected_angle_rad = 0.0

    for _ in range(steps):
        raw_angle_rad += gyro_raw[2] * dt_s
        corrected_angle_rad += (gyro_raw[2] - state['bg'][2]) * dt_s
        eskf.predict(state, gyro_raw, accel_raw, dt_s)

    elapsed_s = steps * dt_s
    true_angle_deg = true_rate_deg_s * elapsed_s
    corrected_angle_deg = math.degrees(corrected_angle_rad)
    filter_yaw_deg = eskf.get_euler_deg(state)[2]
    expected_deg = (true_rate_deg_s + sensor_bias_deg_s - estimated_bias_deg_s) * elapsed_s
    wrapped_difference_deg = (filter_yaw_deg - corrected_angle_deg + 180.0) % 360.0 - 180.0

    assert math.isclose(corrected_angle_deg, expected_deg, abs_tol=1e-8)
    assert abs(wrapped_difference_deg) < 1e-8
    print(f"Elapsed time:                  {elapsed_s:.2f} s")
    print(f"Known physical turn:           {true_angle_deg:.3f} deg")
    print(f"Raw gyro integral:             {math.degrees(raw_angle_rad):.3f} deg")
    print(f"Bias-corrected integral:        {corrected_angle_deg:.3f} deg")
    print(f"Production quaternion yaw:     {filter_yaw_deg:.3f} deg (wrapped)")
    print("PASS: scalar integration matches the analytic answer and quaternion heading.")


if __name__ == '__main__':
    main()
