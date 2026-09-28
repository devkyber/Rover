# Code review and cleanup — 2026-09-17

## Scope and intent

Reviewed the application-owned rover Python, browser code, Arduino sketch,
Jetson Python/shell/PowerShell tools, existing documentation, and SDK patch
record. The [source map](08_learning_guide.md#8-complete-source-map) lists these
files. This was a maintainability and correctness review to support learning;
it was not a formal verification or a line-by-line audit of the upstream SDK.

No hardware, motor commands, firmware flashing, Jetson deployment, or camera
recording was performed during this cleanup. Existing experimental logs and
their metadata were preserved. The workspace is not currently a Git repository.
Before editing, copies of the existing source/document files were saved outside
the project in this temporary backup directory:
`C:\Users\tae06\AppData\Local\Temp\rover-cleanup-before-h_vzortd`.
Temporary storage is a recovery aid for this edit, not durable version control.

## Changes made

- Centralized NIS observation dimensions in `config.NIS_DOF`; CLI replay, live
  telemetry, replay backend and rendered replay JavaScript use that definition.
- Replaced the duplicate nominal 100 Hz fallback in `analyze_drive.py` with
  `config.EKF_RATE_HZ` and corrected its example command.
- Removed the unused `math` import from `teleop.py`.
- Corrected misleading comments: 104 Hz is above 100 Hz; motor position decoding
  is signed; NHC is an assumption; accelerometer norm deviation is only a
  disturbance heuristic; sequential Arduino reads are not exact synchronization.
- Clarified quaternion conventions, the axis-angle helper, log-flush durability,
  and the body-z approximation used by turn gates, without changing
  estimator equations, calibration constants, gates, or command behavior.
- Corrected documentation about alignment/update order, covariance initialization,
  accumulated versus wrapped yaw, and the claim that error can never decrease.
- Made NIS advice less prescriptive: a residual mismatch should trigger inspection
  of the model, timing and noise assumptions, not an automatic `R` adjustment.
- Added an explicit desktop dependency list and Python-cache ignore rules.
- Added the English learning guide and an offline turn-angle lesson using the
  actual production predictor, with an explanation of every lesson statement.

This is intentionally an incremental cleanup. Replacing dictionaries with new
state classes, splitting all embedded web pages, changing the filter's math, or
rewriting the hardware loop would make review and learning harder in one step.

## Behavior issues still open

These require focused changes and appropriate tests. They were not silently
changed as part of the readability work.

### A. Address before relying on unattended motor control

| Finding | Evidence in source / reproduction | Required follow-up |
|---|---|---|
| Motor transmission failure is discarded by `drive()`. | `write_velocities()` returns a success Boolean, but `drive()` ignores it and returns the requested raw values. A fake writer returning a communication failure reproduces this without hardware. | Propagate transport failure, distinguish requested from successfully transmitted commands, define fault behavior, and test failed writes. A successful broadcast transmission still is not a per-motor acknowledgement. |
| Some startup failures occur outside the cleanup block. | `teleop.main()` enables torque before logger/Pygame setup and before entering its `try/finally`. `setup_wheels()` can also partially complete. | Put resource acquisition under cleanup coverage from the first acquisition; test failures after each setup stage. |
| Cleanup can stop at the first motor torque-write exception. | `set_torque()` iterates writes that can raise, including when invoked by `stop()`. | Attempt cleanup for every motor, collect/report failures, and validate the motor-side watchdog strategy. |
| Sensor reads can delay the software deadman. | Control commands are prepared before sensor reads, but transmitted afterward; Arduino serial reads have a timeout. | Bound acquisition latency, track actual elapsed command age, and test a stalled stream and disconnected bus. |
| Joystick input has no ownership/authentication or finite-value validation. | Network listener accepts joystick events and converts values without validating all numeric cases. | Define the intended network boundary, validate finite/ranged values, and decide which client owns control. |

### B. Estimation and experiment correctness

| Finding | Consequence | Required follow-up |
|---|---|---|
| `Odometry.zupt_active` omits `self.use_zupt`. | With ZUPT disabled, the filter skips ZUPT/ZARU but the public output can still report active and force small wheel speeds to zero after the stillness hold. This contaminates an enabled/disabled comparison. | Align the property with the actual update condition; test both filter updates and public pose with a small nonzero wheel rate. |
| Arduino fallback can return the previous sample with a new host timestamp. | Callers can integrate an old sample as if it were fresh; finite-value checks and acquisition timing need explicit treatment. | Make freshness, sensor/host timestamps and validity part of the sample contract; test gaps, partial lines, nonfinite values and timestamp wrap/reset. |
| No measured turn-angle validation is established by the reviewed logs. | A straight-run result or synthetic turn does not establish real 90/180/360-degree accuracy. | Perform the independent turn experiment described in [09](09_turn_angle_lesson.md#7-what-a-real-turn-experiment-must-establish-next). |
| Public position directly integrates wheel speed and ESKF attitude. | Increasing wheel measurement covariance does not directly reduce slip distance error in that public output. Internal `P` is not automatically the covariance of this separately integrated pose. | Define the output model and uncertainty before adding slip handling. |
| IMU noise parameters are still partly placeholders/from another sensor. | Good-looking NIS or a passing simulator cannot establish calibration of the installed LSM6DS3. | Measure noise/bias behavior and validate on separate held-out motion logs. |
| Process discretization is first order and injection has no explicit error-coordinate covariance-reset Jacobian. | These are modeling/approximation limits to assess before treating covariance as quantitatively reliable, particularly with larger corrections. | Derive the chosen convention, evaluate correction sizes, and test covariance consistency; avoid a casual formula replacement. |
| Some replay paths handle empty/short logs poorly. | `analyze_drive` assumes a nonempty history; `replay.inspect` timing statistics assume enough intervals. | Add clear user-facing input validation with empty and single-row examples. |

The public-pose architecture and IMU limitations are also discussed in the
[existing roadmap](05_한계와_로드맵.md). This review is the implementation-specific
companion to that system roadmap.

### C. Camera tools and reproducibility

- `cam_stream.py` uses a single-threaded `HTTPServer` with a long-running stream
  handler; a streaming client can occupy the server. A failed capture can leave
  a previous frame displayed without indicating its age.
- `cam_burst.py` keeps frames in memory; longer captures require a bounded-memory
  design. The short diagnostic scripts should be distinguished from production services.
- `exp_probe.py` and `fps_probe.py` access the camera on import. Add entry-point
  guards when consolidating these experiments into reusable tools.
- `camtest.ps1` disables SSH host verification, stops a remote stream, and deletes
  named remote scratch paths. It should not be a beginner's general run command.
- Jetson scripts contain deployment-path and platform assumptions. Windows
  syntax checks do not establish Jetson driver, camera or GStreamer compatibility.
- There is no Git history or environment lock in this workspace. The added
  requirements list makes dependencies visible, but does not make an old run
  reproducible. Establish version control, record configuration/version metadata
  per experiment, and choose a supported Jetson runtime before deployment.

## Verification

| Check | Result |
|---|---|
| Parse all application Python source | 27/27 files parsed. |
| Existing filter math and structural checks | 11/11 passed, including Jacobians, yaw preservation, alignment and public pose. |
| Existing filter scenario checks | 10/10 passed; the trajectory cases use ten random seeds each. |
| Fake pipeline, 5 s, run after the filter suite | 10/10 passed; 500 samples at 100.0 Hz, timing standard deviation 0.38 ms, CSV round-trip maximum difference `5.00e-7`. |
| Teaching example | Default plus five documented variations passed analytic and quaternion-heading checks, including a full turn. |
| Replay server checks | Flask test client rendered the page, listed logs, rejected an invalid path, replayed a 600-sample synthetic log through alignment and updates, and returned a trace. |
| Shared diagnostic definition | Python consumers and rendered browser NIS dimensions matched `config.NIS_DOF`. |
| Browser script syntax | Node syntax checks passed for rendered replay and live-panel scripts. |
| Change containment | Before/after executable AST comparison confirmed unchanged estimator, odometry, sensor/motor driver and logger logic after stripping documentation. |
| Learning-document coverage | Every one of the 33 application source files has an entry; edited-document local file links and code fences checked. |
| Open-issue reproductions | A fake failed motor writer and a disabled-ZUPT odometry run reproduced the two noted issues without hardware. These are confirmed findings, not fixed tests. |

The large-bias scenario tests deliberately disable startup alignment to isolate
bias recovery. Their permitted large errors are not accuracy claims for normal
aligned startup. Likewise, fake pipeline timing is desktop software evidence,
not a measurement of real serial latency. Browser rendering was checked through
the server and script parser; a full interactive browser session was not exercised.
Arduino/Jetson execution and physical turn accuracy remain unverified here.

Desktop environment used:

| Component | Version |
|---|---|
| Python | 3.13.14, Windows |
| NumPy | 2.4.3 |
| pyserial | 3.5 |
| Matplotlib | 3.10.8 |
| Flask | 3.1.3 |
| Flask-SocketIO | 5.6.1 |
| pygame | 2.6.1 |

These are the installed versions used for this check, not a compatibility claim
for every platform or a newly generated dependency lock.
