"""Two independent Scout links behind Rover's existing Jumper interface.

Slot IDs follow argument order, never discovery order. Commands must name a slot
when both are configured; connection IDs and leases remain device-specific.
"""
from ipaddress import IPv4Address
from pathlib import Path

import config
from jumper_service import JumperService


def scout_hosts(value):
    """Normalize old single-host callers and repeatable CLI arguments early."""
    hosts = [] if value is None else [value] if isinstance(value, str) else list(value)
    if len(hosts) > len(config.JUMPER_SCOUT_IDS):
        raise ValueError('At most two --jumper addresses are supported')
    try:
        hosts = [str(IPv4Address(host)) for host in hosts]
    except (ValueError, TypeError) as exc:
        raise ValueError('--jumper requires a LAN IPv4 address for each Scout') from exc
    if len(set(hosts)) != len(hosts):
        raise ValueError('Each Scout requires a different --jumper IPv4 address')
    return hosts


def description(value):
    hosts = scout_hosts(value)
    return {scout_id: {'scout_id': scout_id, 'host': hosts[index] if index < len(hosts) else None}
            for index, scout_id in enumerate(config.JUMPER_SCOUT_IDS)}


class Scouts:
    def __init__(self, hosts, *, log_dir, event=lambda text, **fields: None,
                 service_factory=JumperService):
        slots = description(hosts)  # Validate before creating files or workers.
        self.services = {}
        self._failures = {}
        self._configured = [key for key, slot in slots.items() if slot['host']]
        for scout_id, slot in slots.items():
            def report(text, scout_id=scout_id):
                event(f'JUMPER {scout_id}: {text.removeprefix("JUMPER ")}', scout_id=scout_id)
            self.services[scout_id] = service_factory(
                slot['host'], scout_id=scout_id, log_dir=Path(log_dir) / scout_id, event=report)

    def _broadcast(self, method, *args):
        # One failed link must never skip stopping/updating the other link or
        # interrupt Rover's actuator cleanup. Failures remain visible per slot.
        for scout_id, service in self.services.items():
            try:
                getattr(service, method)(*args)
                self._failures.pop(scout_id, None)
            except Exception as exc:
                self._failures[scout_id] = f'{method}: {exc}'
                try:
                    service.update_policy(False, {})
                except Exception:
                    pass  # Keep the original error; never skip the peer's stop.

    def start(self):
        self._broadcast('start')

    def update_policy(self, allowed, context):
        self._broadcast('update_policy', allowed, context)

    def release(self, client):
        self._broadcast('release', client)

    def emergency_stop(self):
        self._broadcast('emergency_stop')

    def submit(self, client, message):
        scout_id = message.get('scout_id')
        if scout_id is None and len(self._configured) == 1:
            scout_id = self._configured[0]  # Existing one-Scout clients still work.
        if not isinstance(scout_id, str) or scout_id not in self._configured:
            return dict(ok=False, error='Select a configured scout_id; no command was sent',
                        request_id=message.get('request_id'))
        try:
            result = self.services[scout_id].submit(client, dict(message, scout_id=scout_id))
            self._failures.pop(scout_id, None)
            return dict(result, scout_id=scout_id)
        except Exception as exc:
            self._failures[scout_id] = str(exc)
            return dict(ok=False, scout_id=scout_id, error=str(exc), request_id=message.get('request_id'))

    def snapshot(self):
        slots = {}
        for scout_id, service in self.services.items():
            state = service.snapshot()
            if scout_id in self._failures:
                state = dict(state, error=self._failures[scout_id])
            slots[scout_id] = state
        # Retain Scout 1's legacy fields for historical readers and one-Scout UI.
        return dict(slots[config.JUMPER_SCOUT_IDS[0]], scouts=slots,
                    configured_scouts=list(self._configured))

    def close(self):
        self.update_policy(False, {})
        errors = []
        for scout_id, service in self.services.items():
            try:
                service.close()
            except Exception as exc:
                self._failures[scout_id] = f'close: {exc}'
                errors.append(f'{scout_id}: {exc}')
        if errors:
            raise RuntimeError('Scout finalization incomplete: ' + '; '.join(errors))
