"""
Read position, velocity, and current from all 4 wheel motors
using a single Sync Read packet.

Usage:
    python rover/dxl_reader.py          # quick test: reads and prints 10 times
"""
import time
import sdk_path  # noqa: F401  -> puts rover/vendor/ on sys.path
from dynamixel_sdk import (
    PortHandler, PacketHandler, GroupSyncRead, COMM_SUCCESS,
)
import config


def open_port(port=config.PORT, baudrate=config.BAUDRATE):
    """Open the U2D2 serial port. Returns (port_handler, packet_handler)."""
    port_h = PortHandler(port)
    pkt_h  = PacketHandler(config.PROTOCOL) 

    # openPort() does NOT just return False on failure -- pyserial raises
    # SerialException from inside it, which otherwise escapes as a wall of
    # traceback. Two failures dominate and they need OPPOSITE fixes, so
    # tell them apart instead of guessing:
    #   port exists but is busy -> something else is holding it
    #   port does not exist     -> the adapter is unplugged
    try:
        opened = port_h.openPort()
    except Exception as e:
        text = str(e).lower()
        if "access is denied" in text or "permission" in text:
            hint = ("Something else is holding the port."
                    "\n  - Close DYNAMIXEL Wizard 2.0"
                    "\n  - Stop any teleop.py / record.py still running")
        elif "cannot find" in text or "no such" in text:
            hint = (f"{port} does not exist right now."
                    "\n  - Is the U2D2 plugged in?"
                    "\n  - Find the real port: python dxl_tool.py scan")
        else:
            hint = "Find the real port: python dxl_tool.py scan"
        raise RuntimeError(f"Cannot open {port}: {e}\n  {hint}") from e

    if not opened:
        raise RuntimeError(f"Cannot open port {port}")
    if not port_h.setBaudRate(baudrate):
        raise RuntimeError(f"Cannot set baudrate {baudrate}")

    print(f"Port {port} opened at {baudrate} bps")
    return port_h, pkt_h


def create_sync_reader(port_h, pkt_h, motor_ids=config.MOTOR_IDS):
    """
    Set up a GroupSyncRead that reads 10 contiguous bytes per motor:
      addr 126-127  Present Current   (2 bytes, signed int16)
      addr 128-131  Present Velocity  (4 bytes, signed int32)
      addr 132-135  Present Position  (4 bytes, interpreted as signed int32)
    """
    reader = GroupSyncRead(port_h, pkt_h,
                           config.SYNC_READ_START,
                           config.SYNC_READ_LEN)

    for mid in motor_ids:
        if not reader.addParam(mid):
            raise RuntimeError(f"Cannot add motor ID {mid} to SyncRead")

    return reader


def read_wheels(reader, motor_ids=config.MOTOR_IDS):
    """
    Do one Sync Read and return a dict:
    {
        'timestamp': float  (seconds, time.monotonic),
        'wheels': [
            {'id': 1, 'position_rad': ..., 'velocity_rads': ..., 'current_A': ...},
            ...
        ],
        'roundtrip_ms': float
    }
    """
    t0 = time.monotonic()
    result = reader.txRxPacket()
    t1 = time.monotonic()

    if result != COMM_SUCCESS:
        return None  # communication failed, skip this cycle

    wheels = []
    for mid in motor_ids:
        # isAvailable() must be checked before every getData(), because
        # getData() returns 0 when it fails -- and 0 is a perfectly
        # legal reading. Without this check a dead motor looks like a
        # wheel that is stopped at position 0.
        #
        # We drop the WHOLE cycle if any motor is missing. The SDK's
        # rxPacket() bails out on the first motor that does not answer
        # and leaves last_result False, which makes isAvailable() return
        # False for every ID -- so a partial read is not a thing that
        # can happen, and returning 3 wheels would silently skew the
        # odometry average.
        if (not reader.isAvailable(mid, config.ADDR_PRESENT_CURRENT, config.LEN_CURRENT)
                or not reader.isAvailable(mid, config.ADDR_PRESENT_VELOCITY, config.LEN_VELOCITY)
                or not reader.isAvailable(mid, config.ADDR_PRESENT_POSITION, config.LEN_POSITION)):
            return None

        # Read raw values
        raw_cur = reader.getData(mid, config.ADDR_PRESENT_CURRENT, config.LEN_CURRENT)
        raw_vel = reader.getData(mid, config.ADDR_PRESENT_VELOCITY, config.LEN_VELOCITY)
        raw_pos = reader.getData(mid, config.ADDR_PRESENT_POSITION, config.LEN_POSITION)

        # Convert to signed. getData() always hands back unsigned, so
        # every one of these needs doing by hand.
        #
        # Present Position is SIGNED too. In Velocity Control Mode it
        # accumulates multi-turn and its documented range is
        # -1,048,575 ~ 1,048,575 -- so a wheel that has been rolled
        # backwards past its zero reads as a value just under 2^32.
        # Missing this made a wheel sitting at -1.237 rad report
        # +6,588,396 rad. It stayed hidden while every wheel happened
        # to be positive; the 4-motor bring-up exposed it.
        if raw_cur > 0x7FFF:
            raw_cur -= 0x10000
        if raw_vel > 0x7FFFFFFF:
            raw_vel -= 0x100000000
        if raw_pos > 0x7FFFFFFF:
            raw_pos -= 0x100000000

        # Convert to SI units
        wheels.append({
            'id':            mid,
            'position_rad':  raw_pos * config.POSITION_TO_RAD,
            'velocity_rads': raw_vel * config.VELOCITY_TO_RADS,
            'current_A':     raw_cur * config.CURRENT_TO_AMP,
        })

    return {
        'timestamp':    t0,
        'wheels':       wheels,
        'roundtrip_ms': (t1 - t0) * 1000,
    }


def close_port(port_h):
    """Close the serial port."""
    port_h.closePort()
    print("Port closed")


# ── Quick test ─────────────────────────────────────────────────────

if __name__ == "__main__":
    port_h, pkt_h = open_port()
    reader = create_sync_reader(port_h, pkt_h)

    print(f"\nReading {len(config.MOTOR_IDS)} motors, 10 samples...\n")

    roundtrips = []
    for i in range(10):
        data = read_wheels(reader)
        if data is None:
            print(f"  [{i}] communication error")
            continue

        roundtrips.append(data['roundtrip_ms'])
        for w in data['wheels']:
            print(f"  [{i}] ID:{w['id']}  "
                  f"pos:{w['position_rad']:+7.3f} rad  "
                  f"vel:{w['velocity_rads']:+7.3f} rad/s  "
                  f"cur:{w['current_A']:+6.3f} A")
        print(f"       roundtrip: {data['roundtrip_ms']:.2f} ms")
        time.sleep(0.05)

    if roundtrips:
        avg = sum(roundtrips) / len(roundtrips)
        print(f"\nAvg roundtrip: {avg:.2f} ms  ->  max rate: {1000/avg:.0f} Hz")
        if avg < 10:
            print("100 Hz is feasible!")
        else:
            print(f"WARNING: 100 Hz needs < 10 ms, you got {avg:.1f} ms")

    close_port(port_h)
