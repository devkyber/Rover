"""
Hardware settings and shared odometry tuning parameters.
Change ports, geometry, noise settings, and motion thresholds here.
"""
import math
import os

# Windows laptop (bench) vs Jetson (rover). On Linux the names come from
# jetson/99-rover-serial.rules, which pins each adapter to a fixed
# /dev/<name> by VID:PID -- /dev/ttyUSB0 vs ttyUSB1 depends on plug order.
ON_WINDOWS = os.name == "nt"

# Small ESKF matrices must not compete with acquisition through BLAS workers.
NUMERIC_THREADS = 1
# Keep the normal Python timeslice; SPI acquisition uses its own process.
PYTHON_SWITCH_INTERVAL_S = 0.005
# CSV disk/metadata work belongs to one bounded writer, never the control loop.
CSV_LOG_QUEUE_MAX_RECORDS = 2048
CSV_LOG_CLOSE_TIMEOUT_S = 3.0
LOOP_TIMING_WINDOW_S = 1.0
# Optional estimation has its own process and bounded IPC, outside motor control.
ESTIMATION_INPUT_QUEUE_MAX = 64
# Display transport tolerates scheduler/IPC delay; scientific context has its
# own age limits below. None of these limits rejects a received IMU batch.
ESTIMATION_RESULT_MAX_AGE_S = 1.0
ESTIMATION_WHEEL_MAX_AGE_S = 0.5  # Historical wheel pairing for display estimates.
ESTIMATION_PRECISION_WHEEL_MAX_AGE_S = 0.05
ESTIMATION_CLOSE_TIMEOUT_S = 3.0
ESTIMATION_WHEEL_HISTORY = 256
ESTIMATION_POLL_HZ = 100
ESTIMATION_OUTPUT_HZ = 50

# ── DYNAMIXEL motor setup ──────────────────────────────────────────
# XH430-V350-R, Protocol 2.0, connected via U2D2

PORT       = "COM22" if ON_WINDOWS else "/dev/u2d2"  # U2D2 (FTDI FT232H, VID:PID 0403:6014)
BAUDRATE   = 4_500_000            # Baud Rate(8) value 7 = 4.5 Mbps
PROTOCOL   = 2.0

# Return Delay Time(9) is expected to be 0 on each motor (250 = 500 us).
# Recorded here because it is stored in the motor's EEPROM, not in this
# file -- if a motor is ever factory-reset it goes back to 250 and the
# loop silently gets slower.
RETURN_DELAY_TIME_EXPECTED = 0

# ── Control table addresses (XH430-V350-R) ─────────────────────────
# We read a contiguous block: addr 126 to 135 (10 bytes per motor)
#   126-127 : Present Current  (2 bytes, signed)
#   128-131 : Present Velocity (4 bytes, signed)
#   132-135 : Present Position (4 bytes, interpreted as signed int32)

ADDR_PRESENT_CURRENT  = 126
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_POSITION = 132

# Start of the contiguous read block, and its total length
SYNC_READ_START = 126             # first address to read
SYNC_READ_LEN   = 10             # 2 + 4 + 4 = 10 bytes

# Individual field sizes (for getData calls)
LEN_CURRENT  = 2
LEN_VELOCITY = 4
LEN_POSITION = 4

# ── Unit conversions ───────────────────────────────────────────────
# Raw → SI units

POSITION_TO_RAD  = 2 * math.pi / 4096      # 4096 pulses per revolution
VELOCITY_TO_RADS = 0.229 * 2 * math.pi / 60  # 0.229 rpm per unit → rad/s
CURRENT_TO_AMP   = 1.34e-3                  # 1.34 mA per unit → A

# -- Physical dimensions -------------------------------------------
# MEASURED on the rover, not guessed. Everything scales with these:
# a 10% error in the radius makes every distance 10% wrong, and any R
# tuned on top of that is wrong with it.
#
# ROBOTIS specifies 31 rpm at 24 V with NO LOAD. That is not a measured
# loaded-rover maximum. The command limit below only prevents simulation
# from requesting more than the default Velocity Limit(44)=135 allows.

# Current chassis, used for the 2026-10-02 recordings onward. The chassis
# of the 2026-09-06 recordings was 0.0625 m / 0.485 m. A log does not store
# these, so put the old values back to replay a September log.
WHEEL_RADIUS_M  = 0.055      # wheel radius, m
WHEELBASE_M     = 0.1585     # left-right track width, m
# Estimation calibration is separate from physical geometry and motor commands.
# A profile must match this chassis geometry; old September coefficients do not.
WHEEL_DISTANCE_SCALE = 1.0
WHEEL_CALIBRATION_PROFILE = None  # optional JSON, or --odometry-profile PATH
WHEEL_CALIBRATION_PROFILE_MAX_BYTES = 128 * 1024
MOTOR_SPEC_NO_LOAD_RPM = 31.0       # official spec at 24 V, no load
GOAL_VELOCITY_LIMIT_RAW = 135       # XH430 default Velocity Limit(44)
COMMAND_WHEEL_SPEED_LIMIT_MPS = (
    GOAL_VELOCITY_LIMIT_RAW * VELOCITY_TO_RADS * WHEEL_RADIUS_M
)

# -- IMU noise (Q) -------------------------------------------------
# BMI088 datasheet noise with placeholder bias random walks. These are
# not measured noise parameters for the installed sensor. Older tuning
# below used LSM6DS3 logs; changing the connected IMU does not validate it.
# Replace with installed-sensor measurements and independent validation.

GYRO_NOISE_DENSITY  = 0.014 * math.pi / 180  # 0.014 °/s/√Hz → rad/s/√Hz
GYRO_BIAS_RANDOM_WALK = 1e-5                  # placeholder, get from Allan variance
ACCEL_NOISE_DENSITY = 175e-6 * 9.81           # 175 µg/√Hz → m/s²/√Hz
ACCEL_BIAS_RANDOM_WALK = 1e-4                 # placeholder, get from Allan variance

GRAVITY = 9.81  # m/s²

# Maximum file accepted by the dashboard's manual Scout Plot viewer.
SCOUT_REPORT_MAX_IMPORT_BYTES = 1024 * 1024
SCOUT_POSTSHOT_CAPTURE_S = 15.0
SCOUT_POSTSHOT_MAX_TELEMETRY_ROWS = 1000

# Dedicated read-only raw relay; independent of ESP/control/video sockets.
SCOUT_PLOT_FEED_BIND = '0.0.0.0'
SCOUT_PLOT_FEED_PORT = 8012
SCOUT_PLOT_FEED_CHUNK_BYTES = 256 * 1024
SCOUT_PLOT_FEED_REQUEST_BYTES = 4096
SCOUT_PLOT_FEED_RESPONSE_BYTES = 512 * 1024
SCOUT_PLOT_FEED_MAX_CLIENTS = 4
SCOUT_PLOT_FEED_SOCKET_TIMEOUT_S = 1.0
SCOUT_PLOT_FEED_CLOSE_TIMEOUT_S = 1.0
SCOUT_PLOT_FEED_POLL_S = 0.1
# ── Filter tuning ──────────────────────────────────────────────────

# TUNED 2026-09-07 from the four floor-driving logs of 2026-09-06,
# against tape-measured leg lengths. Was 0.05 -- an untuned guess, and
# comparable to the rover's own top speed of 0.2 m/s, so the filter
# barely believed its own encoders and threw away 22% of every distance.
#
# 0.012 is NOT sensor noise. Measured from the logs, the wheels disagree
# with each other by 0.0007 m/s and the mean speed scatters by 0.0014
# m/s; encoder quantisation is 0.0015 m/s per wheel. R has to cover the
# MODEL error instead -- slip, rolling-radius error, scrub -- which the
# tape puts at 2.5-4% of speed. 0.012 sits above both. docs/03.
#
# Wheel velocity observation noise (std dev in m/s)
R_WHEEL_VELOCITY = 0.012

# ... and how much to distrust it per rad/s of turning. A skid-steer
# rover cannot turn without scrubbing its wheels sideways, so the
# "average wheel speed = forward speed" model is a straight-line model.
R_WHEEL_TURN_GAIN = 0.1

# NHC observation noise (std dev in m/s)
# v_y = 0 base noise (increases during turns)
R_NHC_VY_BASE = 0.01
R_NHC_VY_TURN_GAIN = 2.0  # κ: multiplied by |yaw_rate|
# v_z = 0 noise
R_NHC_VZ = 0.01

# The three constants below were TUNED FROM REAL DATA
# (../logs/2026-09-06_4wheel_still.csv, 20 s, 4 wheels) using NIS.
# The whole method, and the measurements behind these numbers, are in
# docs/03_튜닝과_NIS.md.
#
# Tuned by coordinate descent -- one R at a time, each driven to make its
# own mean NIS hit its degrees of freedom (3 for all three of these).
# It converged in one pass and then stopped moving:
#
#     mean NIS   zupt 2.93   zaru 3.33   gravity 3.27     (target 3)
#     20 s stationary drift 5.3 mm, ZUPT active 95.8%
#
# They were re-tuned after the gravity-Jacobian fix. Startup alignment and
# yaw-preserving updates were added later; current replay gives stationary
# NIS 1.50/1.68/0.45 for ZUPT/ZARU/gravity. Keep these values to isolate
# the structural change, but do not call them final until new holdout runs.
#
# NOTE the tuning is NOT monotonic: shrinking these together by 2.5x
# does not make the filter sharper, it makes it diverge (zupt NIS 25,
# gravity 16). Change one at a time and re-measure.

# ZUPT noise (std dev in m/s)
R_ZUPT = 0.00014812

# ZARU noise (std dev in rad/s)
# Started at 0.0001 (a guess). The first real stationary log gave mean NIS
# 1747 against a target of 3 -- the LSM6DS3 gyro is far noisier than that
# guess claimed -- and NIS fell to target when R rose about 70x.
R_ZARU = 0.00694874

# Gravity update noise (std dev in m/s^2)
# Started at 0.1 (a guess), which was far too timid.
R_GRAVITY = 0.02988870

# Gravity noise grows with | |accel| - g |, a heuristic for disturbance.
# This does not measure all non-gravitational acceleration: a sideways
# acceleration can change direction substantially with little norm change.
# The separate turn-rate gate also matters. Set from the driving logs of
# 2026-09-06 -- see docs/03.
R_GRAVITY_ACCEL_GAIN = 1.0

# Number of scalar residuals in each observation (NIS degrees of freedom).
# Under a consistent model, the expected NIS is this dimension. A matching
# mean is a diagnostic, not proof of accurate position or correct tuning.
NIS_DOF = {'wheel': 1, 'nhc': 2, 'zupt': 3, 'zaru': 3, 'gravity': 3}

# ── Timing ─────────────────────────────────────────────────────────

EKF_RATE_HZ = 100    # target filter rate
IMU_RATE_HZ = 400    # BMI088 output rate
# Public wheel path: integrate the gyro trajectory inside each delivered batch.
# 'endpoint' remains available for historical replay; method identity is logged.
ODOMETRY_POSITION_INTEGRATION = 'gyro_midpoint'

# BMI088 SPI on Jetson physical header pins 24 (accel) / 26 (gyro).
# Requested SPI speed; the Tegra clock divider may produce a different rate.
BMI088_SPI_BUS = 0
BMI088_ACCEL_CS = 0
BMI088_GYRO_CS = 1
BMI088_SPI_HZ = 1_000_000
BMI088_SPI_MODE = 3
BMI088_ACCEL_RANGE_G = 6
BMI088_GYRO_RANGE_DPS = 500
# 400 Hz accel, OSR4 / 40 Hz bandwidth; gyro 400 Hz / 47 Hz bandwidth.
# All gyro FIFO frames are retained; accel is the latest filtered reading.
BMI088_ACCEL_CONF = 0x8A
BMI088_GYRO_BANDWIDTH = 0x03
BMI088_READ_TIMEOUT_S = 0.100
# FIFO count snapshots have host-call brackets, not exact sensor timestamps.
BMI088_FIFO_PHASE_TOLERANCE_FRAMES = 2.0
# Clock integrity follows the measured consecutive accel read intervals,
# rather than declaring any interval longer than 75 ms a sensor reset.
BMI088_SENSOR_CLOCK_TOLERANCE_S = 0.010
# Manual-driving tolerance for additional error from omitted IMU intervals.
# These are provisional motion-envelope assumptions, not measured accuracy:
# missing motion is limited by adjacent measured rates/speeds + these margins.
IMU_GAP_MAX_TOLERATED_S = 0.050
IMU_GAP_HEADING_BUDGET_DEG = 2.0
IMU_GAP_POSITION_BUDGET_M = 0.05
IMU_GAP_RATE_MARGIN_DPS = 30.0
IMU_GAP_SPEED_MARGIN_MPS = 0.25
# IMU worker recovery is independent of manual driving, camera and Pico.
IMU_RETRY_DELAY_S = 3.0
# Arrival age never discards an otherwise valid, ordered acquisition batch.
# These are display-waiting and precision-context thresholds, respectively.
IMU_SAMPLE_MAX_AGE_S = 0.500
IMU_PRECISION_MAX_AGE_S = 0.050
IMU_QUEUE_MAX_SAMPLES = 256
# Separate SPI acquisition from the main process's Python scheduling.
IMU_PROCESS_START_TIMEOUT_S = 15.0
IMU_PROCESS_READ_TIMEOUT_S = 1.000
IMU_PROCESS_CLOSE_TIMEOUT_S = 0.200
IMU_PROCESS_MAX_REPLY_BYTES = 2_000_000
# Owner mounting description 2026-10-08: sensor +Y forward, +X right.
# body = matrix @ sensor; therefore +Z sensor must point up. Confirm gravity
# and positive CCW yaw on the installed rover before driving. The earlier
# loose-board stationary recording had -Z up and is not mounting evidence.
# None blocks body-frame operation; --sensor-frame is a bench diagnostic.
BMI088_IMU_TO_BODY = (
    (0.0, 1.0, 0.0),
    (-1.0, 0.0, 0.0),
    (0.0, 0.0, 1.0),
)

# The rover must remain stationary for this long when live odometry starts.
# We average the gyro to initialize its bias and use mean acceleration to
# initialize roll/pitch. The direction the rover faces then defines yaw=0.
STARTUP_ALIGNMENT_SECONDS = 5.0

# ── Wheel layout: THE one place that describes the rover ───────────
#
# Add or remove a line here and everything follows: motor IDs, sign
# flips, left/right grouping, CSV columns, the simulator. Nothing else
# needs editing.
#
#                       front
#                  FL ---------- FR
#                   |            |
#                   |     ^ +x   |      +x = forward
#                   |     |      |      +y = LEFT
#                   |            |      +z = up
#                  RL ---------- RR
#                       rear
#
# side: "L" or "R". The right-hand motors are mounted mirrored, so a
# positive shaft rotation drives that side BACKWARD -- "R" means the
# sign gets flipped in software. Do NOT also set the motor's own
# Drive Mode(10) Reverse bit; the two cancel out. Software wins because
# then a swapped motor is one edit here, not an EEPROM write.
#
# HOW TO CHECK: run rover_main.py, drive forward from the station. Every wheel
# must turn the way the rover would drive forward. If one is backwards,
# flip that row's side letter.

WHEELS = [
    # (motor id, name,          side)
    (1,          "front-left",  "L"),   # left  top
    (2,          "front-right", "R"),   # right top
    (3,          "rear-left",   "L"),   # left  bottom
    (4,          "rear-right",  "R"),   # right bottom
]

# BENCH TESTING ONLY. ROVER_MOTOR_IDS=5,6,7,8 swaps the motor IDs above, in
# order, for four other motors on the same bus -- names and sides stay, so the
# whole program runs unchanged against spare motors. Unset = the real rover.
# Set per command, never exported, or the real wheels stop answering.
_bench_ids = os.environ.get("ROVER_MOTOR_IDS")
if _bench_ids:
    _bench_ids = [int(x) for x in _bench_ids.split(",")]
    assert len(_bench_ids) == len(WHEELS), "ROVER_MOTOR_IDS needs one ID per wheel"
    WHEELS = [(i, w[1], w[2]) for i, w in zip(_bench_ids, WHEELS)]

# ── Everything below is derived. Do not edit. ──────────────────────
MOTOR_IDS   = [w[0] for w in WHEELS]
WHEEL_NAMES = [w[1] for w in WHEELS]
WHEEL_SIGN  = [+1 if w[2] == "L" else -1 for w in WHEELS]
WHEEL_LEFT  = [i for i, w in enumerate(WHEELS) if w[2] == "L"]
WHEEL_RIGHT = [i for i, w in enumerate(WHEELS) if w[2] == "R"]

assert WHEEL_LEFT and WHEEL_RIGHT, "need at least one wheel on each side"
assert len(set(MOTOR_IDS)) == len(MOTOR_IDS), "duplicate motor ID in WHEELS"

# ── ZUPT / ZARU stillness detection ────────────────────────────────
# See odometry.StillnessDetector for why the STD tests exist and the
# plain magnitude test was not enough.

ZUPT_WINDOW = 20              # samples in the rolling window (0.2 s @100 Hz)

ZUPT_WHEEL_SPEED_MAX = 0.02   # rad/s, max |wheel velocity| to call it stopped

# The load-bearing tests: a stationary sensor reads a STEADY wrong
# number, so its standard deviation is tiny no matter how big the bias.
#
# Set from a REAL 20 s stationary log (../logs/2026-09-06_2wheel_still.csv), not from a
# datasheet. Worst rolling-window std actually observed on that log:
#     gyro  0.0236 rad/s      accel  0.1475 m/s^2
# Thresholds sit just above those so genuine stillness is not rejected.
# Re-measure these if the IMU changes (BMI088 will be much quieter).
ZUPT_GYRO_STD_MAX  = 0.030    # rad/s
ZUPT_GYRO_MEAN_DELTA_MAX = ZUPT_GYRO_STD_MAX  # rad/s from stationary alignment reference; not field calibrated
ZUPT_ACCEL_STD_MAX = 0.250    # m/s^2

ZUPT_ACCEL_DEV_MAX = 0.40     # m/s^2, max | |accel| - g |

ZUPT_HOLD_SAMPLES  = 10       # consecutive quiet samples before trusting it

# ── Gravity update gating ──────────────────────────────────────────
# Only trust the accelerometer for roll/pitch when it is close to 1 g,
# i.e. we are not accelerating hard.
GRAVITY_GATE_MAX_DEV = 0.5    # m/s^2

# Do not trust the accelerometer for tilt while turning: centripetal
# acceleration is sideways specific force and looks exactly like roll.
GRAVITY_GATE_MAX_YAW_RATE = 0.1   # rad/s

# ── IMU over USB (Arduino Nano 33 IoT, LSM6DS3) ────────────────────
# Alternate reader and legacy recordings.
# Flash firmware/imu_stream/imu_stream.ino onto the board first.

IMU_PORT = "COM25" if ON_WINDOWS else "/dev/rover_imu"  # Arduino Nano 33 IoT (VID:PID 2341:8057)
IMU_BAUD = 500_000        # must match SERIAL_BAUD in the sketch

# Arduino board axes -> rover body axes.
#
# Established from all three 2026-09-06 driving logs, not guessed:
# wheel-derived forward acceleration correlates -0.76 to -0.82 with the
# sensor Y axis (slope -1.02 to -1.11), and only 0.01 to 0.06 with sensor X.
# The board is therefore mounted with:
#
#   rover forward (+X_body) = -Y_sensor
#   rover left    (+Y_body) = +X_sensor
#   rover up      (+Z_body) = +Z_sensor
#
# This is a proper 90-degree rotation (orthogonal, determinant +1), and the
# same matrix must be used for accelerometer and gyroscope vectors.
ARDUINO_IMU_TO_BODY = (
    (0.0, -1.0, 0.0),
    (1.0,  0.0, 0.0),
    (0.0,  0.0, 1.0),
)

# The sketch sends RAW sensor units; we convert here so the numbers are
# checkable in one place instead of baked into firmware.
ACCEL_G_TO_MS2   = GRAVITY          # LSM6DS3 accel is in g
GYRO_DPS_TO_RADS = math.pi / 180.0  # LSM6DS3 gyro is in deg/s

# LSM6DS3 on this board runs at ~104 Hz for both accel and gyro.
# That is ABOVE our 100 Hz target with little margin -- fine for a
# pipeline test, but the real BMI088 (400 Hz) is what P0 assumes.
IMU_EXPECTED_HZ = 104

# ── Station link (rover_main.py <-> station/ page on the laptop) ───
# One TCP port carries both WebSocket channels, /control and /video.
STATION_PORT = 8000

# Installation serializes these nonsecret policies for the standalone Jetson
# helper and generates the browser module; this is their only defaults source.
WIFI_RECOVERY = {
    'grace_s': 20,
    'cooldown_s': 60,
    'failed_checks': 3,
    'ping_count': 2,
    'ping_timeout_s': 1,
    'tcp_timeout_s': 1.0,
    'router_tcp_port': 80,
    'activation_timeout_s': 25,
    'deactivation_timeout_s': 8,
    'diagnostic_timeout_s': 2.0,
    'timer_boot_s': 45,
    'timer_interval_s': 10,
    'service_timeout_s': 90,
    'record_relative_dir': 'logs/wifi_recovery',
}
STATION_LINK_WATCHDOG = {
    'ping_interval_ms': 1000,
    'pong_timeout_ms': 5000,
    'connect_timeout_ms': 8000,
    'retry_delay_ms': 1000,
    'max_pending_pings': 8,
}

# Deadman. No drive message from the station for this long -> wheels are
# commanded to zero. The station sends 20 a second, so this is ten missed
# in a row. At the 0.2 m/s command limit the rover covers 10 cm in that
# time. (The proposal says 100 ms; tighten only after measuring how long
# the field Wi-Fi really stalls, or every hiccup becomes a stop.)
COMMAND_TIMEOUT_S = 0.5

TELEMETRY_HZ = 20         # telemetry messages to the station per second
# Latest-only browser publication drains only after actuators have stopped.
TELEMETRY_PUBLISH_CLOSE_TIMEOUT_S = 1.0

# Full run journals: disk I/O stays on a worker, including flush/fsync/rotation.
TELEMETRY_LOG_SUBDIR = 'sessions'
TELEMETRY_LOG_QUEUE_MAX = 512
TELEMETRY_LOG_FLUSH_S = 0.5
TELEMETRY_LOG_FSYNC_S = 2.0
TELEMETRY_LOG_ROTATE_BYTES = 32 * 1024 * 1024
TELEMETRY_LOG_CLOSE_TIMEOUT_S = 3.0

# ── Pico 2 I/O board: servos + battery monitor (firmware/pico/main.py) ──────
# On Linux the name comes from jetson/99-rover-serial.rules. None = find
# the board by its USB ID, whatever COM number Windows gave it.
PICO_PORT = None if ON_WINDOWS else "/dev/pico"
PICO_USB_ID = (0x2E8A, 0x0005)      # MicroPython on a Raspberry Pi board

# Servo commands to the Pico per second. Its failsafe stops the servos
# after 300 ms without one, so five in a row can go missing.
PICO_COMMAND_HZ = 20

# Four continuous-rotation PWM channels. The original PCB labels LIFT A/B
# now carry WIND / RELEASE; camera PAN stays on channel 2, AUX is spare.
# Measure neutral and direction with the mechanism unloaded. Wind and release
# stay silent until trim, offset and timing are configured. Pan retains its
# pre-existing nominal trim, still requiring bench validation.
SERVOS = [
    # (role, stop us, maximum speed offset us, direction). Zero = disabled.
    ("wind",     0,              0,                         +1),
    ("release",  0,              0,                         +1),
    ("pan",      1500,           50,                         -1),
    ("aux",      0,              0,                         +1),
]
# Right-button command magnitude, before the configured direction sign.
# PAN direction follows the operator's camera-view correction (2026-10-23).
# User-requested 2026-10-24: right 1400 us; left stays 50 us offset. Not RPM feedback.
PAN_RIGHT_OFFSET_US = 100
# Suppress the uncalibrated neutral pulse at idle, including the Pico's failsafe
# and watchdog boot. Regenerate/redeploy Pico settings when this changes.
# PWM-off removes the signal; physical stopping/holding still needs verification.
PAN_IDLE_PWM_OFF = True

# Current native PCB interface, reconciled 2026-10-15. CAD/BOM/pinout owner:
# ../PCB/projects/rover-pdb (source hashes: data/pdb_io_2026-10-15).
# Generated firmware constants below come from tools/build_pico_config.py.
PICO_FIRMWARE = {
    "VERSION": 2,
    "SERVO_PINS": (0, 1, 2, 3),  # J2, J3, J4, J8, respectively
    "I2C_SDA_PIN": 16, "I2C_SCL_PIN": 17, "I2C_HZ": 100_000,
    "I2C_TIMEOUT_US": 5000,
    "INA226_ADDRESS": 0x40, "INA226_CONFIG": 0x4527,
    "INA_RETRY_MS": 1000, "INA_CHECK_REPORTS": 20, "INA_STALE_MS": 200,
    "PWM_HZ": 50, "PULSE_MIN_US": 500, "PULSE_MAX_US": 2500,
    "FAILSAFE_MS": 300, "REPORT_MS": 50, "WATCHDOG_MS": 2000,
    "LINE_MAX": 64,
}
PICO_SETPOINT_MAX_AGE_S = 0.2
PICO_REPORT_TIMEOUT_S = 0.5
PICO_RETRY_S = 2

# Direct cylinder/string controls: bounded jog/output and input timeouts.
HAMMER_LIFT_ID = 5
HAMMER_LIFT_READ_INTERVAL_S = 0.2
HAMMER_LIFT_WATCHDOG_TICKS = 15
HAMMER_COMMAND_TIMEOUT_S = 0.3
DEFAULT_IMU_SOURCE = 'bmi088'
HAMMER_TEST_DEFAULT_SECONDS = 0.5
HAMMER_TEST_MAX_SECONDS = 2.0
HAMMER_TEST_MIN_SECONDS = 0.1
HAMMER_TEST_DEFAULT_SPEED = 0.5
# RELEASE / RE-ARM use the FS90MR's documented 1400-1600 us span around 1500 us.
# This is a command default, not a measured deadband or loaded-response result.
HAMMER_TEST_LATCH_DEFAULT_SPEED = 1.0
HAMMER_TEST_WIND_DEFAULT_SECONDS = 2.0
HAMMER_TEST_WIND_DEFAULT_SPEED = 1.0
HAMMER_TEST_WIND_DEFAULT_REVERSE = True
CYLINDER_TEST_MAX_RAW = 16
CYLINDER_TEST_UP_SIGN = -1
CYLINDER_TEST_POSITION_MODE = 4
CYLINDER_TEST_POLL_S = 0.02
CYLINDER_TEST_PROFILE_ACCEL = 5
CYLINDER_JOG_LEAD_COUNTS = 160
CYLINDER_EXTENDED_MIN_RAW = -1048575
CYLINDER_EXTENDED_MAX_RAW = 1048575
CYLINDER_ENCODER_COUNTS_PER_REV = 4096
CYLINDER_BASELINE_RAW = 2094
CYLINDER_BASELINE_SOURCE = 'logs/20261022T203936Z_cylinder_baseline/baseline.json'
CYLINDER_UP_POSITION_RAW = 1049
CYLINDER_UP_POSITION_SOURCE = 'logs/20261022T214001Z_cylinder_up_reference/up_reference.json'
CYLINDER_DOWN_POSITION_RAW = 2092
CYLINDER_DOWN_POSITION_SOURCE = 'logs/20261022T220601Z_cylinder_down_reference/down_reference.json'
HAMMER_TEST_SERVO_CENTER_US = 1500
HAMMER_TEST_SERVO_CENTER_MIN_US = 1400
HAMMER_TEST_SERVO_CENTER_MAX_US = 1600
HAMMER_TEST_SERVO_OFFSET_US = 100
HAMMER_TEST_RETRY_S = 2.0
HAMMER_TEST_SETPOINT_AGE_S = 0.2

# Jumper ESP: repeat --jumper for the two independently controlled Scouts.
JUMPER_SCOUT_IDS = ('scout_1', 'scout_2')
# The firmware protocol owns wire limits; these are Rover's control policy.
JUMPER_CONTROL_PORT = 9000
JUMPER_DEFAULT_HOSTS = ('192.168.0.102', '192.168.0.101')  # Scout 1 …ed:9c / Scout 2 …ed:ac; USB+ARP verified.
# Frozen v12.2 source defaults, not measured calibration. J5 positions are fixed
# inside the supplied firmware; the host never sends pulse/count/time targets.
JUMPER_OPERATOR_PROFILES = {
    JUMPER_DEFAULT_HOSTS[1]: {
        'scout_id': 'scout_2',
        'provenance': 'Supplied v12.2 final guide and unchanged control/settings source; not measured calibration',
        'settings': {
            'servo_model': 'J5 position servo (v12.2 source defaults)',
            'servo_mode': 'position', 'servo_neutral_us': 0,
            'prepare_ms': 0, 'latch_ms': 0, 'release_ms': 0,
            'motor_direction': 1, 'encoder_sign': 1, 'motor_duty_pct': 30,
            'load_timeout_ms': 0, 'stall_ms': 0, 'hold_tolerance_counts': 0,
        },
    },
}
# Both slots use the same frozen defaults; IP/MAC identity remains independent.
JUMPER_OPERATOR_PROFILES[JUMPER_DEFAULT_HOSTS[0]] = {
    **JUMPER_OPERATOR_PROFILES[JUMPER_DEFAULT_HOSTS[1]],
    'scout_id': 'scout_1',
    'settings': dict(JUMPER_OPERATOR_PROFILES[JUMPER_DEFAULT_HOSTS[1]]['settings']),
}
JUMPER_DATA_PORT = 9001
JUMPER_LEASE_TIMEOUT_S = 0.3
JUMPER_AUTOMATIC_CONTROL = True  # Backend maintains transport; no browser CLAIM/focus requirement.
JUMPER_MAINLOOP_TIMEOUT_S = 0.2
JUMPER_POLL_S = 0.02
JUMPER_CONTEXT_LOG_INTERVAL_S = 0.1
JUMPER_STREAM_SILENCE_TIMEOUT_S = 2.0
JUMPER_CONTROL_SILENCE_TIMEOUT_S = 1.5
JUMPER_COMMAND_ACK_TIMEOUT_S = 1.5
JUMPER_RECONNECT_DELAY_S = 3.0
JUMPER_COMMAND_QUEUE_MAX = 1
JUMPER_EVENT_QUEUE_MAX = 1024
JUMPER_REQUEST_HISTORY_MAX = 256
JUMPER_CLOSE_TIMEOUT_S = 2.0
JUMPER_CLOSE_COORDINATOR_ALLOWANCE_S = 1.0
JUMPER_LOG_SUBDIR = 'jumper'
# ── Battery monitor (INA226, read by the Pico) ─────────────────────
# The Pico sends the INA226's raw shunt voltage; current = voltage / this.
# 0.002 is RS1 on the PDB. The loose bench modules carry 0.010 (marked
# R010): change it here when testing with one of those, or every current
# reads five times too high.
BATTERY_SHUNT_OHM = 0.002
BATTERY_CELLS = 6                   # pack voltage / 6 is an average, not a cell measurement
# Battery warning only; no voltage-based motion lock.
BATTERY_LOW_V = 21.0

# ── Driving camera (Arducam B0201, USB) ────────────────────────────
# The camera's own H.264 node. The by-id name follows the camera;
# /dev/video2 would change if USB devices come up in another order.
# Full resolution costs nothing extra: the camera's bitrate is fixed
# near 10 Mbit/s at every resolution. docs/11 section 4.2.
CAMERA_DEVICE = ("/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd."
                 "_USB_2.0_Camera_SN0001-video-index2")
CAMERA_WIDTH  = 1920
CAMERA_HEIGHT = 1080
CAMERA_FPS    = 30
CAMERA_CAPTURE_QUEUE_FRAMES = 120  # bounded, non-leaky H.264 acquisition queue
CAMERA_NETWORK_QUEUE_FRAMES = 15   # preview may resync without dropping recordings

# Independent, operator-started video files. Never delete old recordings.
VIDEO_LOG_SUBDIR = 'video'
VIDEO_AUTO_START = True      # record on runtime start, independent of any station
VIDEO_SEGMENT_S = 60.0       # rotate on the next IDR; every file can start alone
VIDEO_SYNC_S = 1.0          # sync cadence, not a hard power-loss guarantee
VIDEO_GAP_S = 3.0
VIDEO_FRAME_GAP_FACTOR = 1.8  # timing diagnostic/retiming; alone not proof of lost frames
VIDEO_QUEUE_FRAMES = 120
VIDEO_QUEUE_BYTES = 16 * 1024 * 1024
VIDEO_MAX_FRAME_BYTES = 4 * 1024 * 1024
VIDEO_MIN_FREE_BYTES = 256 * 1024 * 1024
VIDEO_MUX_QUEUE_BYTES = 2 * 1024 * 1024
VIDEO_MUX_OUTPUT_BUFFERS = 256  # enough TS output capacity for 1080p/10 Mb/s
VIDEO_MUX_TIMEOUT_S = 3.0
VIDEO_WORKER_POLL_S = 0.02
VIDEO_CLOSE_TIMEOUT_S = 10.0

# Mains frequency of the lights where the rover drives: 50 or 60. Korea is 60.
# Indoor lights flicker at twice the mains frequency, and the camera reads its
# picture out row by row, so with the wrong value light and dark bands drift
# across the video. The camera's own default is 50 and it forgets the setting
# when it loses power, so camera.py sets it at every start.
# Measured 2026-10-05 under room lights: at 50, four bands per frame at 2.8% of
# brightness, moving 22 rows per frame; at 60, 0.02% (gone). Sunlight does not
# flicker, so outdoors the value does not matter.
CAMERA_MAINS_HZ = 60

# Jetson CPU temperature, in thousandths of a degree C. Absent on Windows.
CPU_TEMP_FILE = "/sys/devices/virtual/thermal/thermal_zone0/temp"

# Wi-Fi signal strength as Linux reports it. Absent on Windows.
WIFI_STATUS_FILE = "/proc/net/wireless"

# Operator-entered metric mission maps; durable journal I/O stays off drive loop.
MAPPING_LOG_SUBDIR = 'mission_maps'
MAPPING_QUEUE_MAX = 16
MAPPING_REPLY_QUEUE_MAX = 4
MAPPING_CLOSE_TIMEOUT_S = 2.0
MAPPING_MAX_MAPS = 64
MAPPING_MAX_RECORDS = 5000
MAPPING_MAX_JOURNAL_BYTES = 16 * 1024 * 1024
MAPPING_MAX_REQUEST_BYTES = 32 * 1024
MAPPING_MAX_RECORD_BYTES = 64 * 1024
MAPPING_MAX_CONTROL_POINTS = 32
MAPPING_MAX_TEXT = 2048
MAPPING_MAX_COORDINATE_M = 10000000.0
MAPPING_MIN_BASELINE_M = 0.1

# Clegg ADXL375 on the identified Nano RP2040 Connect. Arduino D10/11/12/13
# map to RP2040 GP5/7/4/6; this is independent of legacy IMU_PORT and Scout.
CLEGG_SPI_CS_GPIO = 5
CLEGG_SPI_MOSI_GPIO = 7
CLEGG_SPI_MISO_GPIO = 4
CLEGG_SPI_SCK_GPIO = 6
CLEGG_SPI_HZ = 2_000_000
CLEGG_RATE_REGISTER = 0x0F
CLEGG_SAMPLE_HZ = 3200
CLEGG_NOMINAL_G_PER_LSB = 0.049  # Datasheet typical, not an installed calibration.
CLEGG_RAW_LIMIT_COUNTS = 4000
CLEGG_RING_SAMPLES = 4096
CLEGG_PACKET_SAMPLES = 32
CLEGG_CONTEXT_PORT = 8011
CLEGG_CONTEXT_MAX_AGE_S = 0.15
CLEGG_CONTEXT_HISTORY = 512
CLEGG_CONTEXT_RESPONSE_MAX_BYTES = 262144
CLEGG_CONTEXT_CONNECT_TIMEOUT_S = 2.0
CLEGG_CONTEXT_ASSOCIATION_WAIT_S = 2.0
CLEGG_CONTEXT_POLL_S = 0.05
CLEGG_CONTEXT_LOG_MAX_AGE_S = 5.0
CLEGG_RELEASE_CONTEXT_TTL_S = 5.0
CLEGG_RELEASE_HISTORY = 64
CLEGG_SYNC_INTERVAL_S = 1.0
CLEGG_SYNC_MAX_AGE_S = 3.0
CLEGG_ASSUMED_DRIFT_PPM = 1000.0  # Unvalidated envelope, retained in metadata.
CLEGG_IMPACT_THRESHOLD_G = 10.0  # Candidate trigger only; not ground-contact proof.
CLEGG_IMPACT_REFRACTORY_S = 0.5
CLEGG_LOG_SUBDIR = 'clegg'
CLEGG_TCP_PORT = 9060
CLEGG_DISCOVERY_PORT = 9061
CLEGG_CONNECT_TIMEOUT_S = 2.0
CLEGG_RECEIVE_TIMEOUT_S = 0.5
CLEGG_RECONNECT_S = 1.0
CLEGG_STREAM_STALE_S = 3.0
CLEGG_IMPACT_PRE_S = 0.1
CLEGG_IMPACT_POST_S = 0.4
CLEGG_IMPACT_AUTO_REARM = True
CLEGG_IMPACT_MAX_POINTS = 8192
CLEGG_IMPACT_HISTORY = 12
CLEGG_WIFI_RETRY_MS = 10_000
CLEGG_WIFI_JOIN_TIMEOUT_MS = 15_000
# Dedicated laptop hotspot. Set static IP to '' when using DHCP on another LAN.
CLEGG_WIFI_STATIC_IP = '192.168.137.2'
CLEGG_WIFI_GATEWAY = '192.168.137.1'
CLEGG_WIFI_NETMASK = '255.255.255.0'
CLEGG_NINA_SPI_RX_GPIO = 8
CLEGG_NINA_SPI_TX_GPIO = 11
CLEGG_NINA_SPI_SCK_GPIO = 14
CLEGG_NINA_SPI_CS_GPIO = 9
CLEGG_NINA_ACK_GPIO = 10
CLEGG_NINA_RESET_GPIO = 3
CLEGG_NINA_GPIO0 = 2
CLEGG_NINA_SPI_HZ = 8_000_000
CLEGG_NINA_POWER_MODE = 0  # NINA noLowPowerMode: request WIFI_PS_NONE.
CLEGG_NINA_IO_TIMEOUT_MS = 2000
CLEGG_NINA_MAX_BLOCK = 1536
CLEGG_WIFI_SCAN_TIMEOUT_MS = 15_000
CLEGG_WIFI_ANNOUNCE_MS = 2000
CLEGG_FIFO_POLL_MS = 1
CLEGG_FIFO_READ_GAP_US = 6
# 43 records produce a 1484-byte frame, above NINA's configured TCP MSS.
# This is a packetization candidate; sustained wireless reliability needs measurement.
CLEGG_NETWORK_PACKET_SAMPLES = 43
CLEGG_PARTIAL_PACKET_MS = 20
