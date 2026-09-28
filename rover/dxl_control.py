"""
Motor WRITING: torque, operating mode, goal velocity.

Kept separate from dxl_reader.py on purpose -- reading is safe, writing
moves the rover. Nothing here runs unless you call it.

Wheels use Velocity Control Mode (Operating Mode = 1). Goal Velocity(104)
is a signed int32 in units of 0.229 rev/min, limited by Velocity Limit(44)
which is 135 by default (= 30.9 rev/min, the motor's no-load speed).
"""
import sdk_path  # noqa: F401
from dynamixel_sdk import (
    GroupSyncWrite, COMM_SUCCESS,
    DXL_LOBYTE, DXL_HIBYTE, DXL_LOWORD, DXL_HIWORD,
)
import config

ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_VELOCITY = 104
LEN_GOAL_VELOCITY = 4

MODE_VELOCITY = 1

# Goal Velocity(104) is limited by Velocity Limit(44), default 135.
MAX_GOAL_VELOCITY = config.GOAL_VELOCITY_LIMIT_RAW


def _write1(pkt_h, port_h, dxl_id, address, value, what):
    """Write one byte and actually check the result."""
    comm, err = pkt_h.write1ByteTxRx(port_h, dxl_id, address, value)
    if comm != COMM_SUCCESS:
        raise RuntimeError(f"ID {dxl_id}: {what} failed: "
                           f"{pkt_h.getTxRxResult(comm)}")
    if err != 0:
        raise RuntimeError(f"ID {dxl_id}: {what} rejected: "
                           f"{pkt_h.getRxPacketError(err)}")


def set_torque(pkt_h, port_h, on, motor_ids=None):
    """Enable or disable torque on every motor."""
    for dxl_id in (motor_ids or config.MOTOR_IDS):
        _write1(pkt_h, port_h, dxl_id, ADDR_TORQUE_ENABLE,
                1 if on else 0, f"torque {'on' if on else 'off'}")


def setup_wheels(pkt_h, port_h, motor_ids=None):
    """
    Put every wheel into Velocity Control Mode and enable torque.

    Operating Mode lives in EEPROM, and EEPROM is READ-ONLY while torque
    is enabled. So torque must go off first -- otherwise the mode write
    is silently rejected and the wheel stays in Position mode, where it
    will refuse to spin past 360 degrees. That is the single most common
    way this goes wrong after a crashed run leaves torque on.
    """
    ids = motor_ids or config.MOTOR_IDS

    set_torque(pkt_h, port_h, False, ids)          # unlock EEPROM
    for dxl_id in ids:
        _write1(pkt_h, port_h, dxl_id, ADDR_OPERATING_MODE,
                MODE_VELOCITY, "set velocity mode")

    # Verify it took, rather than assuming.
    for dxl_id in ids:
        mode, comm, err = pkt_h.read1ByteTxRx(port_h, dxl_id,
                                              ADDR_OPERATING_MODE)
        if comm != COMM_SUCCESS or err != 0 or mode != MODE_VELOCITY:
            raise RuntimeError(
                f"ID {dxl_id}: Operating Mode is {mode}, expected "
                f"{MODE_VELOCITY} (Velocity). The write did not stick.")

    set_torque(pkt_h, port_h, True, ids)
    print(f"Wheels {ids}: velocity mode, torque ON")


def create_sync_writer(port_h, pkt_h):
    """One GroupSyncWrite for Goal Velocity, reused every cycle."""
    return GroupSyncWrite(port_h, pkt_h, ADDR_GOAL_VELOCITY,
                          LEN_GOAL_VELOCITY)


def _to_bytes(value):
    """Signed int32 -> the 4 little-endian bytes SyncWrite wants."""
    v = int(value) & 0xFFFFFFFF          # two's complement for negatives
    return [DXL_LOBYTE(DXL_LOWORD(v)), DXL_HIBYTE(DXL_LOWORD(v)),
            DXL_LOBYTE(DXL_HIWORD(v)), DXL_HIBYTE(DXL_HIWORD(v))]


def write_velocities(writer, raw_velocities, motor_ids=None):
    """
    Send one Goal Velocity per motor, in MOTOR_IDS order.

    raw_velocities: list of ints in DYNAMIXEL units (0.229 rev/min),
                    already sign-corrected and clamped.
    Returns True if the packet went out.
    """
    ids = motor_ids or config.MOTOR_IDS
    writer.clearParam()
    for dxl_id, v in zip(ids, raw_velocities):
        if not writer.addParam(dxl_id, _to_bytes(v)):
            raise RuntimeError(f"ID {dxl_id}: addParam failed")
    return writer.txPacket() == COMM_SUCCESS


def drive(writer, forward, turn, motor_ids=None):
    """
    Differential drive from two joystick axes, both in -1..+1.

    Applies config.WHEEL_SIGN, so the mirrored right-hand motors are
    handled in exactly one place -- the same place the odometry reads
    them back. Returns the raw values actually sent.
    """
    ids = motor_ids or config.MOTOR_IDS

    v_left = forward + turn
    v_right = forward - turn

    # Scale so a full-deflection stick maps to full speed without
    # clipping the turn away.
    biggest = max(1.0, abs(v_left), abs(v_right))
    v_left /= biggest
    v_right /= biggest

    raw = []
    for i in range(len(ids)):
        side = v_left if i in config.WHEEL_LEFT else v_right
        raw.append(int(round(side * MAX_GOAL_VELOCITY * config.WHEEL_SIGN[i])))

    write_velocities(writer, raw, ids)
    return raw


def stop(writer, pkt_h, port_h, motor_ids=None):
    """Zero every wheel, then release torque. Always call this on exit."""
    ids = motor_ids or config.MOTOR_IDS
    try:
        write_velocities(writer, [0] * len(ids), ids)
    finally:
        set_torque(pkt_h, port_h, False, ids)
        print("Wheels stopped, torque OFF")
