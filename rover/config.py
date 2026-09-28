"""
Hardware settings and shared odometry tuning parameters.
Change ports, geometry, noise settings, and motion thresholds here.
"""
import math

# ── DYNAMIXEL motor setup ──────────────────────────────────────────
# XH430-V350-R, Protocol 2.0, connected via U2D2

PORT       = "COM22"              # U2D2 (FTDI FT232H, VID:PID 0403:6014)
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

WHEEL_RADIUS_M  = 0.0625     # wheel radius, m
WHEELBASE_M     = 0.485      # left-right track width, m
MOTOR_SPEC_NO_LOAD_RPM = 31.0       # official spec at 24 V, no load
GOAL_VELOCITY_LIMIT_RAW = 135       # XH430 default Velocity Limit(44)
COMMAND_WHEEL_SPEED_LIMIT_MPS = (
    GOAL_VELOCITY_LIMIT_RAW * VELOCITY_TO_RADS * WHEEL_RADIUS_M
)

# -- IMU noise (Q) -------------------------------------------------
# WARNING: these are BMI088 DATASHEET values, but the IMU actually
# connected is an LSM6DS3, which is much noisier. They have not been
# measured. Allan variance on a multi-hour stationary log replaces all
# four -- docs/05 section 3-4.

GYRO_NOISE_DENSITY  = 0.014 * math.pi / 180  # 0.014 °/s/√Hz → rad/s/√Hz
GYRO_BIAS_RANDOM_WALK = 1e-5                  # placeholder, get from Allan variance
ACCEL_NOISE_DENSITY = 175e-6 * 9.81           # 175 µg/√Hz → m/s²/√Hz
ACCEL_BIAS_RANDOM_WALK = 1e-4                 # placeholder, get from Allan variance

GRAVITY = 9.81  # m/s²

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
# HOW TO CHECK: run teleop.py, push the left stick forward. Every wheel
# must turn the way the rover would drive forward. If one is backwards,
# flip that row's side letter.

WHEELS = [
    # (motor id, name,          side)
    (1,          "front-left",  "L"),   # left  top
    (2,          "front-right", "R"),   # right top
    (3,          "rear-left",   "L"),   # left  bottom
    (4,          "rear-right",  "R"),   # right bottom
]

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
# Stand-in for the BMI088 until it arrives.
# Flash arduino/imu_stream/imu_stream.ino onto the board first.

IMU_PORT = "COM25"        # Arduino Nano 33 IoT (VID:PID 2341:8057)
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
