#!/usr/bin/env python3
"""Recover the field profile after association or sustained local packet loss.

Policy is serialized from rover/config.py by install_wifi_recovery.py.
Only the field interface and its local gateway are used. No Internet is needed.
"""
from __future__ import annotations

import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import errno
import ipaddress
import json
import math
import os
from pathlib import Path
import socket
import subprocess
import time


def nmcli(*args, timeout=12):
    result = subprocess.run(['nmcli', *args], capture_output=True, text=True,
                            timeout=timeout, env={**os.environ, 'LC_ALL': 'C'})
    if result.returncode:
        raise RuntimeError(f'nmcli {args[0]} failed ({result.returncode})')
    return result.stdout.strip()


def decision(*, state, active_uuid, target_uuid, radio, autoconnect, profile_autoconnect):
    if radio != 'enabled' or autoconnect != 'yes' or profile_autoconnect != 'yes':
        return 'disabled'
    if active_uuid not in ('', '--', target_uuid):
        return 'other-profile'
    if state == 100 and active_uuid == target_uuid:
        return 'connected'
    if state in (30, 60, 120):
        return 'recoverable'
    return 'networkmanager-busy-or-unavailable'


def observe(args, target):
    values = nmcli('-g', 'GENERAL.STATE,GENERAL.CON-UUID,GENERAL.AUTOCONNECT',
                   'device', 'show', args.interface).splitlines()
    if len(values) != 3:
        raise RuntimeError('Unexpected NetworkManager device response')
    state = int(values[0].split()[0])
    action = decision(state=state, active_uuid=values[1], target_uuid=target,
                      radio=nmcli('radio', 'wifi'), autoconnect=values[2],
                      profile_autoconnect=nmcli('-g', 'connection.autoconnect',
                                               'connection', 'show', 'uuid', target))
    return {'device_state': state, 'active_uuid': values[1], 'action': action}


def icmp_probe(args, gateway):
    try:
        result = subprocess.run(
            ['ping', '-n', '-I', args.interface, '-c', str(args.ping_count),
             '-W', str(args.ping_timeout_s), gateway],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            timeout=args.ping_count * args.ping_timeout_s + 1)
        return 'reachable' if result.returncode == 0 else 'failed' if result.returncode == 1 else 'unknown'
    except (OSError, subprocess.TimeoutExpired):
        return 'unknown'


def tcp_probe(args, gateway, source):
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client:
            client.settimeout(args.tcp_timeout_s)
            client.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE,
                              args.interface.encode() + b'\0')
            client.bind((source, 0))
            client.connect((gateway, args.router_tcp_port))
        return 'reachable'
    except ConnectionRefusedError:
        return 'reachable'  # A received TCP RST proves IPv4 round-trip delivery.
    except (TimeoutError, socket.timeout):
        return 'failed'
    except OSError as exc:
        return 'failed' if exc.errno in (errno.EHOSTUNREACH, errno.ENETUNREACH, errno.ETIMEDOUT) else 'unknown'
    except AttributeError:
        return 'unknown'


def probe_gateway(args):
    try:
        gateway = ipaddress.IPv4Address(nmcli('-g', 'IP4.GATEWAY', 'device', 'show', args.interface))
        addresses = [ipaddress.IPv4Interface(value) for value in
                     nmcli('-g', 'IP4.ADDRESS', 'device', 'show', args.interface).splitlines()]
        address = next(value for value in addresses if gateway in value.network and gateway != value.ip)
        if gateway in (address.network.network_address, address.network.broadcast_address):
            raise ValueError('Gateway is a subnet boundary')
        source = address.ip
        if gateway.is_unspecified or gateway.is_multicast or gateway.is_loopback:
            raise ValueError('Invalid local gateway')
    except (OSError, ValueError, StopIteration, RuntimeError, subprocess.TimeoutExpired):
        return {'status': 'unknown', 'reason': 'gateway-or-address-unavailable'}
    ping = icmp_probe(args, str(gateway))
    tcp = 'not-needed' if ping == 'reachable' else tcp_probe(args, str(gateway), str(source))
    status = ('reachable' if 'reachable' in (ping, tcp) else
              'dead' if ping == tcp == 'failed' else 'unknown')
    return {'status': status, 'gateway': str(gateway), 'source': str(source), 'icmp': ping, 'tcp': tcp}


def read_state(path):
    try:
        value = json.loads(path.read_text())
        return value if isinstance(value, dict) else {}
    except (OSError, ValueError):
        return {}


def save_state(path, state):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix('.tmp')
    with temporary.open('w') as stream:
        json.dump(state, stream, allow_nan=False)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)


def boot_id():
    return Path('/proc/sys/kernel/random/boot_id').read_text().strip()


def valid_time(value, now):
    return type(value) in (int, float) and math.isfinite(value) and 0 <= value <= now


def audit(args, value):
    record = {**value, 'observed_utc': datetime.now(timezone.utc).isoformat(),
              'host_monotonic_s': time.monotonic()}
    if args.record_dir:
        try:
            path = Path(args.record_dir) / 'decisions.jsonl'
            descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_APPEND |
                                 getattr(os, 'O_NOFOLLOW', 0), 0o640)
            with os.fdopen(descriptor, 'a') as stream:
                os.fchmod(stream.fileno(), 0o640)
                stream.write(json.dumps(record, allow_nan=False) + '\n')
                stream.flush()
                if value.get('action') in ('reset-field-profile', 'reconnect-failed'):
                    os.fsync(stream.fileno())
        except OSError as exc:
            record['audit_error_errno'] = exc.errno
    print(json.dumps(record, allow_nan=False), flush=True)


def snapshot(args):
    commands = [
        ['iw', 'dev', args.interface, 'link'],
        ['iw', 'dev', args.interface, 'station', 'dump'],
        ['ip', '-j', '-4', 'address', 'show', 'dev', args.interface],
        ['ip', '-j', 'route', 'show', 'dev', args.interface],
        ['ip', '-j', 'neigh', 'show', 'dev', args.interface],
    ]
    result = []
    for command in commands:
        try:
            value = subprocess.run(command, capture_output=True, text=True,
                                   timeout=args.diagnostic_timeout_s)
            result.append({'argv': command, 'exit_code': value.returncode,
                           'stdout': value.stdout[-4096:], 'stderr': value.stderr[-256:]})
        except (OSError, subprocess.TimeoutExpired) as exc:
            result.append({'argv': command, 'error': type(exc).__name__})
    return result


def run(args, *, now=None):
    supplied_clock = now is not None
    now = time.monotonic() if now is None else now
    target = nmcli('-g', 'connection.uuid', 'connection', 'show', 'id', args.profile)
    report = {**observe(args, target), 'profile': args.profile, 'interface': args.interface,
              'profile_uuid': target, 'boot_id': boot_id()}
    if report['action'] == 'connected':
        report['probe'] = probe_gateway(args)
    if args.check:
        return report
    path = Path(args.state_file)
    previous = read_state(path)
    identity = {key: report[key] for key in ('profile_uuid', 'interface', 'boot_id')}
    if any(previous.get(key) != value for key, value in identity.items()):
        previous = {}
    last_attempt = previous.get('last_attempt')
    retained = {**identity, **({'last_attempt': last_attempt} if valid_time(last_attempt, now) else {})}

    def finish(action, state=None):
        report['action'] = action
        save_state(path, retained if state is None else state)
        audit(args, report)
        return report

    action = report['action']
    if action == 'networkmanager-busy-or-unavailable':
        # A busy/unknown sample breaks the consecutive-failure streak, while
        # the last recovery attempt still rate-limits the next failure.
        return finish(action)
    if action not in ('recoverable', 'connected'):
        return finish(action)
    if action == 'connected' and report['probe']['status'] != 'dead':
        return finish('connected' if report['probe']['status'] == 'reachable' else 'probe-unavailable')
    kind = 'packet-loss' if action == 'connected' else 'association'
    first_bad, count = previous.get('first_bad'), previous.get('failure_count')
    if (previous.get('failure_kind') != kind or not valid_time(first_bad, now) or
            type(count) is not int or count < 1):
        first_bad, count = now, 0
    current = {**retained, 'first_bad': first_bad, 'failure_kind': kind, 'failure_count': count + 1}
    report.update(failure_kind=kind, failure_count=count + 1, failure_age_s=now - first_bad)
    if now - first_bad < args.grace or count + 1 < args.failed_checks:
        return finish('grace', current)
    if valid_time(last_attempt, now) and now - last_attempt < args.cooldown:
        return finish('cooldown', current)
    if observe(args, target)['action'] != action:
        return finish('state-changed')
    if kind == 'packet-loss':
        report['confirmation_probe'] = probe_gateway(args)
        if report['confirmation_probe']['status'] != 'dead':
            return finish('probe-recovered' if report['confirmation_probe']['status'] == 'reachable' else 'probe-unavailable')
    report['before_recovery'] = snapshot(args)
    if observe(args, target)['action'] != action:
        return finish('state-changed')
    # Keep cooldown even when NM activation succeeds or a single probe recovers.
    attempt_at = now if supplied_clock else time.monotonic()
    retained['last_attempt'] = attempt_at
    current['last_attempt'] = attempt_at
    report['attempt_monotonic_s'] = attempt_at
    save_state(path, current)
    report['action'] = 'reset-field-profile' if kind == 'packet-loss' else 'retry-saved-profile'
    audit(args, report)
    try:
        if kind == 'packet-loss':
            nmcli('--wait', str(args.deactivation_timeout_s), 'connection', 'down',
                  'uuid', target, timeout=args.deactivation_timeout_s + 2)
            after_down = observe(args, target)
            if after_down['action'] in ('disabled', 'other-profile'):
                return finish('recovery-cancelled')
            if after_down['action'] == 'networkmanager-busy-or-unavailable':
                return finish('activation-in-progress', current)
        nmcli('--wait', str(args.activation_timeout_s), 'connection', 'up', 'uuid', target,
              'ifname', args.interface, timeout=args.activation_timeout_s + 2)
    except (OSError, RuntimeError, subprocess.TimeoutExpired) as exc:
        report['recovery_error'] = type(exc).__name__
        return finish('reconnect-failed', current)
    after = observe(args, target)
    if after['action'] == 'connected':
        report['post_probe'] = probe_gateway(args)
        if report['post_probe']['status'] == 'reachable':
            return finish('reconnected')
    return finish('reconnect-requested', current)


def validate_settings(settings):
    if not isinstance(settings, dict) or type(settings.get('schema_version')) is not int or settings['schema_version'] != 1:
        raise ValueError('Unsupported Wi-Fi recovery settings schema')
    positive = ('grace_s', 'cooldown_s', 'failed_checks', 'ping_count', 'ping_timeout_s',
                'tcp_timeout_s', 'router_tcp_port', 'activation_timeout_s',
                'deactivation_timeout_s', 'diagnostic_timeout_s')
    for key in positive:
        value = settings.get(key)
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError('Invalid Wi-Fi recovery setting: ' + key)
    for key in ('failed_checks', 'ping_count', 'ping_timeout_s', 'router_tcp_port',
                'activation_timeout_s', 'deactivation_timeout_s'):
        if type(settings[key]) is not int:
            raise ValueError('Expected integer Wi-Fi recovery setting: ' + key)
    if settings['failed_checks'] < 2 or not 1 <= settings['router_tcp_port'] <= 65535:
        raise ValueError('Invalid failure threshold or gateway TCP port')
    if not isinstance(settings.get('record_dir'), str) or not Path(settings['record_dir']).is_absolute():
        raise ValueError('Recovery record directory must be absolute')
    return settings


@contextmanager
def recovery_lock(path):
    import fcntl
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.with_suffix('.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        yield


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', default='rover-field')
    parser.add_argument('--interface', required=True)
    parser.add_argument('--settings', default='/etc/rover/wifi-recovery.json')
    parser.add_argument('--state-file', default='/run/rover-wifi-recover/state.json')
    parser.add_argument('--grace', type=float)
    parser.add_argument('--cooldown', type=float)
    parser.add_argument('--check', action='store_true', help='probe only; no files or connections changed')
    args = parser.parse_args()
    try:
        settings = validate_settings(json.loads(Path(args.settings).read_text()))
        for key in ('failed_checks', 'ping_count', 'ping_timeout_s', 'tcp_timeout_s',
                    'router_tcp_port', 'activation_timeout_s', 'deactivation_timeout_s',
                    'diagnostic_timeout_s', 'record_dir'):
            setattr(args, key, settings[key])
        args.grace = settings['grace_s'] if args.grace is None else args.grace
        args.cooldown = settings['cooldown_s'] if args.cooldown is None else args.cooldown
        if not all(math.isfinite(value) and value > 0 for value in (args.grace, args.cooldown)):
            raise ValueError('Grace/cooldown must be finite and positive')
        if args.check:
            print(json.dumps(run(args)), flush=True)
            return 0
        with recovery_lock(Path(args.state_file)):
            report = run(args)
        return int(report['action'] == 'reconnect-failed')
    except BlockingIOError:
        print(json.dumps({'action': 'another-recovery-running'}), flush=True)
        return 0
    except (OSError, RuntimeError, ValueError, subprocess.TimeoutExpired) as exc:
        if hasattr(args, 'record_dir'):
            audit(args, {'action': 'helper-error', 'error': type(exc).__name__})
        else:
            print(json.dumps({'action': 'helper-error', 'error': type(exc).__name__}), flush=True)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
