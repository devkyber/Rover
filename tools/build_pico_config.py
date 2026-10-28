"""Generate the Pico's standalone settings block from rover/config.py."""
import argparse
import hashlib
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'rover'))
import config
from pico_link import stop_pulses

START = '# BEGIN GENERATED SETTINGS\n'
END = '# END GENERATED SETTINGS\n'


def render():
    if len(config.SERVOS) != len(config.PICO_FIRMWARE['SERVO_PINS']):
        raise ValueError('Each physical PWM output requires one servo configuration')
    for _, stop, offset, direction in config.SERVOS:
        if (type(stop) is not int or type(offset) is not int or offset < 0 or direction not in (-1, 1)
                or (stop == 0 and offset != 0) or (stop != 0 and not
                config.PICO_FIRMWARE['PULSE_MIN_US'] <= stop-offset <= stop+offset <= config.PICO_FIRMWARE['PULSE_MAX_US'])):
            raise ValueError('Invalid servo trim, speed offset or direction')
    values = dict(config.PICO_FIRMWARE, BOOT_STOP_US=tuple(stop_pulses()))
    body = ''.join(f'{key} = {value!r}\n' for key, value in values.items())
    digest = hashlib.sha256(body.encode()).hexdigest()[:16]
    return START + '# Source: rover/config.py; do not edit this block.\n' + body + f'CONFIG_ID = {digest!r}\n' + END


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    path = ROOT / 'firmware/pico/main.py'
    source = path.read_text(encoding='utf-8')
    start = source.index(START)
    end = source.index(END, start) + len(END)
    updated = source[:start] + render() + source[end:]
    if args.check:
        if source != updated:
            raise SystemExit('Pico settings stale: run python tools/build_pico_config.py')
        print('Pico settings match config.py')
    else:
        path.write_text(updated, encoding='utf-8')
        print('Generated firmware/pico/main.py settings')


if __name__ == '__main__':
    main()
