"""
Read-only DYNAMIXEL diagnostics. Nothing here moves a motor.

    python dxl_tool.py scan     # which port, which baud, which IDs?
    python dxl_tool.py check    # are the motors configured correctly?

Run `scan` when nothing responds -- it sweeps ports, baud rates and IDs
and tells you what is actually out there. Run `check` before a drive --
it reads every register that can silently ruin a run (operating mode,
torque, limits, hardware error, voltage, temperature) and says what is
wrong in words.

Writing lives in dxl_control.py, deliberately in a separate file: reading
is safe, writing makes the rover move.
"""

import sdk_path  # noqa: F401
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS
import config

MODEL_XH430_V350 = 1040

# Baud Rate(8): register value -> actual bps
BAUD_TABLE = {0: 9600, 1: 57600, 2: 115200, 3: 1000000,
              4: 2000000, 5: 3000000, 6: 4000000, 7: 4500000}

OPERATING_MODES = {
    0: "Current",
    1: "Velocity",              # what wheels need
    3: "Position (default)",
    4: "Extended Position",
    5: "Current-based Position",
    16: "PWM",
}

# Hardware Error Status(70) bits
HW_ERROR_BITS = {
    0x01: "Input Voltage Error",
    0x04: "Overheating Error",
    0x08: "Motor Encoder Error",
    0x10: "Electrical Shock Error",
    0x20: "Overload Error",
}

# (address, size, label) -- read one at a time; this runs once, not in a loop
REGISTERS = [
    (0,   2, "Model Number"),
    (6,   1, "Firmware Version"),
    (7,   1, "ID"),
    (8,   1, "Baud Rate"),
    (9,   1, "Return Delay Time"),
    (10,  1, "Drive Mode"),
    (11,  1, "Operating Mode"),
    (31,  1, "Temperature Limit"),
    (38,  2, "Current Limit"),
    (44,  4, "Velocity Limit"),
    (64,  1, "Torque Enable"),
    (70,  1, "Hardware Error Status"),
    (98,  1, "Bus Watchdog"),
    (144, 2, "Present Input Voltage"),
    (146, 1, "Present Temperature"),
]


# Every rate Baud Rate(8) can hold, derived from BAUD_TABLE so the scan
# can never fall behind the register. The old hand-written list stopped
# at 4 Mbps and therefore could not find our own motors, which run at
# 4.5 Mbps -- a scan that cannot see the working configuration is worse
# than no scan. Slowest first: a wrong guess at a low rate fails fast.
BAUD_CANDIDATES = sorted(BAUD_TABLE.values())

KNOWN_MODELS = {
    # VERIFIED against the official e-Manual control table (Model Number(0)
    # initial value). Only add a number here after you have checked it --
    # a wrong entry here is worse than no entry, because it silently
    # mislabels the motor.
    1040: "XH430-V350  <- our wheel motors",
}


def list_serial_ports():
    """List every serial port Windows/Linux can see."""
    from serial.tools import list_ports
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found. Is the U2D2 plugged in?")
    for p in ports:
        print(f"  {p.device:10s}  {p.description}")
    return [p.device for p in ports]


def scan_port(port_name, id_range=range(0, 11), stop_early=True):
    """
    Try every baud rate on this port; on each, ping every ID in id_range.
    Returns a list of (baudrate, id, model_number) for everything found.

    Every ping to an absent ID costs a full serial timeout, so an
    exhaustive sweep is minutes, not seconds. Two things keep it short:
    the configured baud rate is tried FIRST (it is nearly always the
    answer), and by default the scan stops at the first rate that
    answers. Pass --all when you genuinely do not know what is out there.
    """
    found = []
    port_h = PortHandler(port_name)
    pkt_h = PacketHandler(config.PROTOCOL)

    try:
        opened = port_h.openPort()
    except Exception as e:
        # pyserial raises rather than returning False. Usually DYNAMIXEL
        # Wizard is still holding the port.
        print(f"  {port_name}: cannot open ({e})")
        return found
    if not opened:
        print(f"  {port_name}: cannot open (in use by another program?)")
        return found

    # Configured rate first -- it is the answer far more often than not.
    bauds = [config.BAUDRATE] + [b for b in BAUD_CANDIDATES
                                 if b != config.BAUDRATE]
    for baud in bauds:
        if not port_h.setBaudRate(baud):
            continue
        print(f"  {port_name} @ {baud} bps ...", end="", flush=True)

        hits = []
        for dxl_id in id_range:
            model, comm, err = pkt_h.ping(port_h, dxl_id)
            if comm == COMM_SUCCESS and err == 0:
                hits.append((dxl_id, model))

        if not hits:
            print(" nothing")
            continue

        print()
        for dxl_id, model in hits:
            name = KNOWN_MODELS.get(model, "not in our table")
            print(f"      ID {dxl_id:3d}  ->  model {model}  ({name})")
            found.append((baud, dxl_id, model))
        if stop_early:
            break

    port_h.closePort()
    return found


def read_register(pkt_h, port_h, dxl_id, address, size):
    """Read one register. Returns (value, ok)."""
    if size == 1:
        val, comm, err = pkt_h.read1ByteTxRx(port_h, dxl_id, address)
    elif size == 2:
        val, comm, err = pkt_h.read2ByteTxRx(port_h, dxl_id, address)
    else:
        val, comm, err = pkt_h.read4ByteTxRx(port_h, dxl_id, address)

    if comm != COMM_SUCCESS:
        print(f"      read {address} failed: {pkt_h.getTxRxResult(comm)}")
        return None, False
    if err != 0:
        print(f"      read {address} error: {pkt_h.getRxPacketError(err)}")
        return None, False
    return val, True


def check_motor(pkt_h, port_h, dxl_id):
    """Read every interesting register on one motor and judge it."""
    print(f"\n--- ID {dxl_id} ---")
    values = {}
    for address, size, label in REGISTERS:
        val, ok = read_register(pkt_h, port_h, dxl_id, address, size)
        if not ok:
            return False
        values[address] = val
        print(f"  {label:24s} ({address:3d}) = {val}")

    problems = []

    if values[0] != MODEL_XH430_V350:
        problems.append(
            f"Model Number is {values[0]}, expected {MODEL_XH430_V350} "
            "(XH430-V350). Is this one of the XC430 arm motors?")

    actual_baud = BAUD_TABLE.get(values[8])
    if actual_baud != config.BAUDRATE:
        problems.append(
            f"Baud Rate register says {actual_baud} bps, but config.BAUDRATE "
            f"is {config.BAUDRATE}.")

    if values[9] != config.RETURN_DELAY_TIME_EXPECTED:
        problems.append(
            f"Return Delay Time is {values[9]} (= {values[9]*2} us delay "
            f"per packet). Expected {config.RETURN_DELAY_TIME_EXPECTED}. "
            "Factory default is 250; set it to 0 in Wizard.")

    mode = values[11]
    if mode != 1:
        problems.append(
            f"Operating Mode is {mode} ({OPERATING_MODES.get(mode, '?')}). "
            "Wheels need mode 1 (Velocity Control). Mode 3 is the factory "
            "default and will NOT drive a wheel continuously.")

    if values[10] & 0x01:
        problems.append(
            "Drive Mode bit 0 (Reverse) is set on this motor. config.py "
            "also flips signs via WHEEL_SIGN -- doing both cancels out. "
            "Pick one.")

    hw_err = values[70]
    if hw_err:
        names = [n for bit, n in HW_ERROR_BITS.items() if hw_err & bit]
        problems.append(f"Hardware Error Status = {hw_err}: {', '.join(names)}. "
                        "The motor needs a REBOOT after you fix the cause.")

    volts = values[144] * 0.1
    if not (22.0 <= volts <= 26.0):
        problems.append(
            f"Input voltage is {volts:.1f} V. XH430-V350 wants 24 V; "
            "it will not reach rated torque at 12 V.")

    temp = values[146]
    if temp > values[31] - 15:
        problems.append(
            f"Temperature {temp} C is close to the limit ({values[31]} C).")

    if values[98] == 0:
        print("  note: Bus Watchdog is off. Fine on the bench; consider "
              "setting it (units of 20 ms) once the rover drives on its own, "
              "so a dead comms link stops the wheels.")

    print()
    if problems:
        for p in problems:
            print(f"  [PROBLEM] {p}")
    else:
        print("  OK -- nothing to fix on this motor.")
    return not problems



def cmd_scan(only_port=None, exhaustive=False):
    """Sweep ports, baud rates and IDs. Use when nothing responds."""
    if only_port:
        ports = [only_port]
    else:
        print("Serial ports on this machine:")
        ports = list_serial_ports()
        print()

    if not ports:
        raise SystemExit("No serial ports at all -- nothing is plugged in.")

    all_found = []
    print("Scanning (this takes ~10 s per port)...")
    for port_name in ports:
        all_found += [(port_name,) + f for f in scan_port(
            port_name,
            id_range=range(0, 21) if exhaustive else range(0, 11),
            stop_early=not exhaustive)]

    print()
    if not all_found:
        print("Nothing found. Checklist:")
        print("  - U2D2 plugged in, motors powered (24 V for XH430-V350)")
        print("  - RS-485 wiring: the U2D2 has separate TTL and RS-485 ports")
        print("  - Motors daisy-chained, last one may need a terminator")
        return

    bauds = sorted({f[1] for f in all_found})
    ids = sorted({f[2] for f in all_found})
    print("Put this in config.py:")
    print(f'    PORT       = "{all_found[0][0]}"')
    print(f"    BAUDRATE   = {bauds[0]}")
    print(f"    MOTOR_IDS  = {ids}   # <- keep only the 4 WHEEL motors")
    if len(bauds) > 1:
        print(f"  NOTE: motors answered at several baud rates {bauds} -- "
              "set them all to one rate in DYNAMIXEL Wizard.")


def cmd_check():
    """Read every register that can silently ruin a run."""
    import dxl_reader          # one place knows how to open the port

    try:
        port_h, pkt_h = dxl_reader.open_port()
    except RuntimeError as e:
        raise SystemExit(str(e))

    print(f"Checking motors {config.MOTOR_IDS}")
    all_ok = True
    try:
        for motor_id in config.MOTOR_IDS:
            all_ok &= check_motor(pkt_h, port_h, motor_id)
    finally:
        dxl_reader.close_port(port_h)

    print("\n" + "=" * 55)
    print("All motors ready." if all_ok
          else "Fix the problems above in DYNAMIXEL Wizard 2.0, then re-run.")


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser(description=__doc__.strip().split("\n")[0])
    ap.add_argument("command", nargs="?", default="check",
                    choices=["scan", "check"])
    ap.add_argument("--port", default=None,
                    help="scan only this port instead of every port")
    ap.add_argument("--all", action="store_true",
                    help="scan every baud rate and IDs 0-20 (slow)")
    args = ap.parse_args()

    if args.command == "scan":
        cmd_scan(args.port, args.all)
    else:
        cmd_check()
