"""
Make the vendored DYNAMIXEL SDK importable.

We keep our own copy of the SDK in rover/vendor/dynamixel_sdk/ so that
`import dynamixel_sdk` works even if nothing is pip-installed.

Any file that needs the SDK just does:

    import sdk_path          # noqa: F401  (side effect: sets sys.path)
    from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncRead

Note: the SDK still needs `pyserial` (`pip install pyserial`).
That is the ONLY external dependency for talking to the motors.
"""
import os
import sys

VENDOR_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "vendor")

if VENDOR_DIR not in sys.path:
    sys.path.insert(0, VENDOR_DIR)


if __name__ == "__main__":
    print(f"vendor dir: {VENDOR_DIR}")
    try:
        import dynamixel_sdk
        print(f"dynamixel_sdk imported from: {dynamixel_sdk.__file__}")
    except ImportError as e:
        print(f"FAILED: {e}")
        print("If it says 'No module named serial' -> pip install pyserial")
