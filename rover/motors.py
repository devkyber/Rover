"""
The wheel motors as one object -- real, or absent for bench work.

    wheels = motors.open_motors(use_motors=True)
    data = wheels.read()             # per-wheel dicts, or None if the bus was silent
    raw = wheels.drive(fwd, turn)    # both -1..+1; returns the raw goals sent
    wheels.close()                   # zero the wheels, torque off, release the port

This only bundles the four handles dxl_reader / dxl_control need, so the
control loop reads the same with or without motors. The DYNAMIXEL logic
itself stays in those two files.
"""
import config


class Motors:
    connected = True

    def __init__(self):
        import dxl_control
        import dxl_reader
        self._reader_mod, self._control_mod = dxl_reader, dxl_control
        self.port_h, self.pkt_h = dxl_reader.open_port()
        self.reader = dxl_reader.create_sync_reader(self.port_h, self.pkt_h)
        dxl_control.setup_wheels(self.pkt_h, self.port_h)
        self.writer = dxl_control.create_sync_writer(self.port_h, self.pkt_h)

    def read(self):
        data = self._reader_mod.read_wheels(self.reader)
        return None if data is None else data['wheels']

    def drive(self, forward, turn):
        return self._control_mod.drive(self.writer, forward, turn)

    def close(self):
        try:
            self._control_mod.stop(self.writer, self.pkt_h, self.port_h)
        finally:
            self._reader_mod.close_port(self.port_h)


class NoMotors:
    """Stands in when no motors are attached: wheels read zero, nothing is sent."""
    connected = False

    def read(self):
        return [{'id': m, 'position_rad': 0.0, 'velocity_rads': 0.0,
                 'current_A': 0.0} for m in config.MOTOR_IDS]

    def drive(self, forward, turn):
        return [0] * len(config.MOTOR_IDS)

    def close(self):
        pass


def open_motors(use_motors):
    return Motors() if use_motors else NoMotors()
