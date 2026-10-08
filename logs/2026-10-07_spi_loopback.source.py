#!/usr/bin/env python3
"""Check SPI data loopback; disconnect peripherals and bridge pins 19 and 21.

This sends arbitrary byte patterns. Use only after physically disconnecting
the BMI088 and other SPI devices. A pass does not verify chip-select timing or
on-wire clock frequency. See docs/11_jetson_bringup.md for the procedure.
"""
import argparse
import datetime
import hashlib
import importlib.metadata
import json
import platform
import time
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--loopback-wired', action='store_true', required=True,
                        help='confirm peripherals disconnected and 19-21 bridged')
    parser.add_argument('--output', type=Path, required=True,
                        help='new JSON file; existing evidence is never overwritten')
    args = parser.parse_args()
    import spidev

    # Open exclusively before touching SPI, so an invalid/existing output path
    # cannot silently discard a completed hardware experiment.
    with args.output.open('x', encoding='utf-8') as output:
        report = {
            'started_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
            'hostname': platform.node(),
            'kernel': platform.release(),
            'spidev_version': importlib.metadata.version('spidev'),
            'probe_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            'wiring': 'User confirmed: BMI088 disconnected; physical 19 to 21',
            'bits_per_word': 8,
            'bit_order': 'MSB first',
            'cs_active_high': False,
            'hardware_loopback_flag': False,
            'clock_note': 'Requested SPI speeds; actual clock not measured',
            'scope': 'Data path only; no CS waveform or IMU validation',
            'cases': [],
            'completed': False,
        }
        patterns = [
            [0x00], [0xff], [0x55], [0xaa],
            [0x80, 0x00], [0x80, 0x00, 0x00],
            [0x12, 0x34, 0x56, 0x78], list(range(1, 17)),
            [(i * 73 + 19) & 0xff for i in range(64)],
            [(i * 73 + 19) & 0xff for i in range(257)],
        ]
        passed = total = 0
        started = time.monotonic()
        try:
            for cs in (0, 1):
                spi = spidev.SpiDev()
                try:
                    spi.open(0, cs)
                    spi.bits_per_word = 8
                    spi.lsbfirst = False
                    spi.cshigh = False
                    for mode in (0, 3):
                        spi.mode = mode
                        for speed in (100_000, 1_000_000, 5_000_000):
                            spi.max_speed_hz = speed
                            for tx in patterns:
                                case = {
                                    'device': f'/dev/spidev0.{cs}',
                                    'mode': mode, 'requested_hz': speed,
                                    'tx_hex': bytes(tx).hex(), 'trials': [],
                                }
                                report['cases'].append(case)
                                for _ in range(10):
                                    rx = spi.xfer2(tx)
                                    ok = rx == tx
                                    case['trials'].append({
                                        'rx_hex': bytes(rx).hex(), 'pass': ok,
                                    })
                                    total += 1
                                    passed += int(ok)
                                    time.sleep(0.001)
                finally:
                    spi.close()
            report['completed'] = True
        except BaseException as error:
            report['error'] = f'{type(error).__name__}: {error}'
            raise
        finally:
            report.update(passed=passed, total=total,
                          elapsed_s=time.monotonic() - started)
            json.dump(report, output, indent=2)
            output.write('\n')
        print(json.dumps({
            'passed': passed, 'total': total, 'output': str(args.output),
            'elapsed_s': report['elapsed_s'],
        }))
    return 0 if passed == total else 1


if __name__ == '__main__':
    raise SystemExit(main())
