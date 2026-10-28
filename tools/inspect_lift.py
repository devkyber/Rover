"""Read ID 5 identity and mode; never changes torque, EEPROM or goals."""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'rover'))
import config
import dxl_reader


def main():
    port, packet = dxl_reader.open_port()
    try:
        result = {'id': config.HAMMER_LIFT_ID, 'port': config.PORT, 'baud': config.BAUDRATE}
        for name, address, size in [('model',0,2), ('firmware',6,1), ('mode',11,1),
                                    ('torque',64,1), ('hardware_error',70,1), ('watchdog',98,1),
                                    ('velocity_limit',44,4), ('present_position_raw',132,4)]:
            value, comm, error = getattr(packet, f'read{size}ByteTxRx')(port, config.HAMMER_LIFT_ID, address)
            if comm or error:
                raise RuntimeError(f'{name}: comm={comm}, error={error}')
            result[name] = value
        print(json.dumps(result, indent=2))
    finally:
        dxl_reader.close_port(port)


if __name__ == '__main__':
    main()
