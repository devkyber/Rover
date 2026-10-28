"""Battery display only. Motion stops belong to E-STOP and link timeouts."""
import math
import config


class BatteryProtection:
    def __init__(self, source='off'):
        self.source = source
        self.enabled = source in ('usb', 'fake')
        self.volts = None
        self.updated_at = -math.inf

    def update(self, board, now):
        volts = (board.get('battery') or {}).get('v')
        age = board.get('ina_age_ms')
        valid = (board.get('status') == 'ok'
                 and type(volts) in (int, float) and math.isfinite(volts) and volts >= 0
                 and type(age) in (int, float) and math.isfinite(age)
                 and 0 <= age <= config.PICO_FIRMWARE['INA_STALE_MS'])
        self.volts = volts if valid else None
        self.updated_at = now

    def state(self, now):
        volts = self.volts if 0 <= now - self.updated_at < config.PICO_REPORT_TIMEOUT_S else None
        level = ('off' if not self.enabled else 'unavailable' if volts is None
                 else 'low' if volts <= config.BATTERY_LOW_V else 'ok')
        # Stable keys also let an already-open older dashboard work.
        return dict(enabled=self.enabled, simulated=self.source == 'fake',
                    level=level, voltage_v=volts, low_v=config.BATTERY_LOW_V,
                    blocked=False, latched=False, reason=None, release_allowed=True)
