"""Durable operator evidence in a shared, explicitly defined metric XY frame.

This is registration and observation bookkeeping, not SLAM, automatic Scout
localization or a calibrated ground-property estimator. No sensor/hardware I/O.
The journal is the authority; metadata is only an atomic active-map pointer.
Corrupt/torn journals remain untouched and expose their valid prefix read-only.
"""
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import re
import threading

import config


SCHEMA = 'rover-mission-map-v1'
AXES = 'Operator-defined right-handed planar XY; metres; no geographic CRS implied.'
UNCERTAINTY = ('User-supplied bounds: local + translation at control centroid + '
               '2*distance-to-centroid*sin(yaw-bound/2). Missing bounds remain unknown. '
               'Fit residual is not verified position accuracy or statistical confidence.')
_MAP_ID = re.compile(r'm_[0-9a-f]{32}\Z')
_KINDS = {'scout_first_contact', 'scout_rest', 'clegg_impact', 'landmark'}


class MapError(ValueError):
    """An invalid/conflicting request or a store that cannot safely be written."""


def _json(value):
    return json.dumps(value, ensure_ascii=False, allow_nan=False,
                      sort_keys=True, separators=(',', ':')).encode('utf-8')


def _sha(value):
    return hashlib.sha256(_json(value)).hexdigest()


def _text(value, field, *, optional=False, limit=None):
    if optional and value is None:
        return None
    if (not isinstance(value, str) or not value.strip()
            or len(value) > (limit or config.MAPPING_MAX_TEXT)
            or any(ord(char) < 32 and char not in '\n\t' for char in value)):
        raise MapError(f'{field} must be nonempty text within its length limit')
    return value


def _number(value, field, *, positive=False, nonnegative=False, bound=None):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value)):
        raise MapError(f'{field} must be a finite number (not a string or boolean)')
    if positive and value <= 0 or nonnegative and value < 0:
        raise MapError(f'{field} must be {"positive" if positive else "nonnegative"}')
    if bound is not None and abs(value) > bound:
        raise MapError(f'{field} exceeds the supported range')
    return float(value)


def _xy(value, field):
    if not isinstance(value, list) or len(value) != 2:
        raise MapError(f'{field} must be [x, y] in metres')
    return [_number(v, field, bound=config.MAPPING_MAX_COORDINATE_M) for v in value]


def _bound(value, field):
    return (None if value is None else
            _number(value, field, positive=True, bound=config.MAPPING_MAX_COORDINATE_M))


def _fields(value, required, optional=()):
    if not isinstance(value, dict):
        raise MapError('Expected an object')
    missing = set(required) - value.keys()
    unknown = value.keys() - set(required) - set(optional)
    if missing or unknown:
        raise MapError(f'Invalid fields: missing={sorted(missing)}, unknown={sorted(unknown)}')


def fit_registration(points, max_residual_m):
    """Least-squares SE(2) fit: proper rotation + translation, never scale/shear."""
    if (not isinstance(points, list)
            or not 2 <= len(points) <= config.MAPPING_MAX_CONTROL_POINTS):
        raise MapError('Provide 2 to MAPPING_MAX_CONTROL_POINTS paired control points')
    clean = []
    for point in points:
        _fields(point, ('local_xy', 'map_xy'))
        clean.append(dict(local_xy=_xy(point['local_xy'], 'local_xy'),
                          map_xy=_xy(point['map_xy'], 'map_xy')))
    threshold = _number(max_residual_m, 'max_residual_m', positive=True,
                        bound=config.MAPPING_MAX_COORDINATE_M)
    local = [point['local_xy'] for point in clean]
    mapped = [point['map_xy'] for point in clean]
    def baseline(values):
        return max(math.hypot(a[0] - b[0], a[1] - b[1])
                   for a in values for b in values)
    if min(baseline(local), baseline(mapped)) < config.MAPPING_MIN_BASELINE_M:
        raise MapError('Control-point baseline is too small for reliable registration')
    n = len(clean)
    lc = [math.fsum(p[i] for p in local) / n for i in (0, 1)]
    mc = [math.fsum(p[i] for p in mapped) / n for i in (0, 1)]
    a = [[p[i] - lc[i] for i in (0, 1)] for p in local]
    b = [[p[i] - mc[i] for i in (0, 1)] for p in mapped]
    dot = math.fsum(p[0] * q[0] + p[1] * q[1] for p, q in zip(a, b))
    cross = math.fsum(p[0] * q[1] - p[1] * q[0] for p, q in zip(a, b))
    norm = math.sqrt(math.fsum(x*x+y*y for x, y in a)
                     * math.fsum(x*x+y*y for x, y in b))
    if norm == 0 or math.hypot(dot, cross) <= norm * 1e-12:
        raise MapError('Control-point correspondence does not determine a rotation')
    yaw = math.atan2(cross, dot)
    c, s = math.cos(yaw), math.sin(yaw)
    translation = [mc[0] - c*lc[0] + s*lc[1], mc[1] - s*lc[0] - c*lc[1]]
    residuals = [math.hypot(c*p[0]-s*p[1]+translation[0]-q[0],
                            s*p[0]+c*p[1]+translation[1]-q[1])
                 for p, q in zip(local, mapped)]
    if max(residuals) > threshold:
        raise MapError('Rigid registration residual exceeds max_residual_m; check scale/axes/evidence')
    return dict(control_points=clean, translation_m=translation, yaw_deg=math.degrees(yaw),
                reference_local_xy_m=lc,
                fit_rmse_m=math.sqrt(math.fsum(r*r for r in residuals)/n),
                fit_max_residual_m=max(residuals), max_residual_m=threshold)


def _project(state):
    registrations = {r['frame_id']: r for r in state['registrations']}
    superseded = {o['supersedes'] for o in state['observations'] if o.get('supersedes')}
    for observation in state['observations']:
        location = observation['location']
        observation.update(map_xy_m=None, uncertainty_m=None, registration_id=None,
                           superseded=observation['event_id'] in superseded)
        if location['kind'] == 'surveyed':
            observation.update(map_xy_m=list(location['xy_m']),
                               uncertainty_m=location['uncertainty_m'], resolution='surveyed')
        elif location['kind'] == 'local' and location['frame_id'] in registrations:
            reg = registrations[location['frame_id']]
            yaw = math.radians(reg['yaw_deg'])
            c, s = math.cos(yaw), math.sin(yaw)
            x, y = location['xy_m']
            tx, ty = reg['translation_m']
            bounds = [location['uncertainty_m'], reg['uncertainty_m'], reg.get('yaw_uncertainty_deg')]
            total_bound = None
            if all(value is not None for value in bounds):
                cx, cy = reg['reference_local_xy_m']
                radius = math.hypot(x-cx, y-cy)
                total_bound = bounds[0] + bounds[1] + 2*radius*math.sin(math.radians(bounds[2])/2)
            observation.update(map_xy_m=[c*x-s*y+tx, s*x+c*y+ty],
                               uncertainty_m=total_bound,
                               registration_id=reg['registration_id'], resolution='registered')
        else:
            observation['resolution'] = 'unknown' if location['kind'] == 'unknown' else 'unregistered_frame'
    return state


class MissionMapStore:
    """Synchronous single-owner library; call from MissionMapService's worker.

    Request IDs are durable within a map, including a deterministic create ID.
    Retry the exact original request after a lost response. A changed revision
    or payload under that ID is a collision, even if its contents look equivalent.
    """
    def __init__(self, root_dir):
        self.root = Path(root_dir).resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        self._lock_file = None
        self._active = None
        self._maps = {}
        self._error = None
        self._closed = False
        self._acquire_lock()
        try:
            self._source = self._source_identity()
            paths = sorted(self.root.glob('m_*'))
            if len(paths) > config.MAPPING_MAX_MAPS:
                raise MapError('Store exceeds MAPPING_MAX_MAPS; preserve evidence and use another store')
            for path in paths:
                if _MAP_ID.fullmatch(path.name):
                    self._maps[path.name] = self._load(path.name)
            pointer = self.root / 'active.json'
            if pointer.exists():
                if pointer.stat().st_size > config.MAPPING_MAX_REQUEST_BYTES or pointer.is_symlink():
                    raise MapError('Invalid active-map metadata')
                try:
                    selected = json.loads(pointer.read_text(encoding='utf-8'))['map_id']
                    if selected not in self._maps:
                        raise ValueError('Missing selected map')
                    self._active = selected
                except (ValueError, KeyError, TypeError) as exc:
                    self._error = f'Active-map metadata damaged; choose Open explicitly: {exc}'
        except Exception:
            self.close()
            raise

    def _acquire_lock(self):
        path = self.root / '.writer.lock'
        if path.is_symlink():
            raise MapError('Store lock must not be a symlink')
        stream = open(path, 'a+b')
        try:
            if os.name == 'nt':
                import msvcrt
                if stream.seek(0, os.SEEK_END) == 0:
                    stream.write(b'0')
                    stream.flush()
                stream.seek(0)
                msvcrt.locking(stream.fileno(), msvcrt.LK_NBLCK, 1)
            else:
                import fcntl
                fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            stream.close()
            raise MapError('Mission-map store already has a writer') from exc
        self._lock_file = stream

    @staticmethod
    def _source_identity():
        directory = Path(__file__).resolve().parent
        files = {}
        for name in ('mission_map.py', 'config.py', 'rover_main.py', 'odometry.py', 'eskf.py'):
            path = directory / name
            files['rover/' + name] = hashlib.sha256(path.read_bytes()).hexdigest()
        limits = {key: getattr(config, key) for key in dir(config) if key.startswith('MAPPING_')}
        return dict(source_sha256=files, mapping_config=limits, mapping_config_sha256=_sha(limits))

    def _path(self, map_id):
        if not isinstance(map_id, str) or not _MAP_ID.fullmatch(map_id):
            raise MapError('Invalid map_id')
        path = self.root / map_id
        if path.is_symlink() or path.resolve().parent != self.root:
            raise MapError('Map directory must stay inside its store')
        return path

    @staticmethod
    def _directory_sync(path):
        # POSIX needs directory fsync for durable names. Windows os.open cannot
        # open directories; file fsync + atomic os.replace is the supported path.
        if os.name != 'nt':
            descriptor = os.open(str(path), os.O_RDONLY)
            try:
                os.fsync(descriptor)
            finally:
                os.close(descriptor)

    def _select(self, map_id):
        temporary = self.root / 'active.json.tmp'
        if temporary.is_symlink() or (self.root / 'active.json').is_symlink():
            raise MapError('Metadata must not be a symlink')
        with open(temporary, 'wb') as stream:
            stream.write(_json(dict(map_id=map_id)))
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, self.root / 'active.json')
        self._directory_sync(self.root)
        self._active = map_id
        self._error = None

    def _load(self, map_id):
        path = self._path(map_id) / 'journal.jsonl'
        entry = dict(records=[], requests={}, error=None, bytes=0)
        try:
            if path.is_symlink():
                raise MapError('Journal must not be a symlink')
            size = path.stat().st_size
            if size > config.MAPPING_MAX_JOURNAL_BYTES:
                raise MapError('Journal exceeds maximum size')
            previous = None
            with open(path, 'rb') as stream:
                while True:
                    raw = stream.readline(config.MAPPING_MAX_RECORD_BYTES + 1)
                    if not raw:
                        break
                    if len(raw) > config.MAPPING_MAX_RECORD_BYTES or not raw.endswith(b'\n'):
                        raise MapError('Torn or oversized journal record')
                    if len(entry['records']) >= config.MAPPING_MAX_RECORDS:
                        raise MapError('Journal exceeds maximum record count')
                    record = json.loads(raw)
                    if not isinstance(record, dict):
                        raise MapError('Journal record must be an object')
                    digest = record.pop('sha256')
                    if (digest != _sha(record) or record['previous_sha256'] != previous
                            or record['revision'] != len(entry['records']) + 1
                            or record['map_id'] != map_id or record['schema'] != SCHEMA
                            or record['request_sha256'] != _sha(record['request'])
                            or record['request_id'] != record['request']['request_id']
                            or record['request_id'] in entry['requests']):
                        raise MapError('Journal hash/identity/order mismatch')
                    record['sha256'] = digest
                    # Validate stored semantics, without refitting controls or
                    # applying today's fit tolerance to accepted past evidence.
                    self._validate_record(record, entry['records'])
                    entry['records'].append(record)
                    entry['requests'][record['request_id']] = record
                    entry['bytes'] += len(raw)
                    previous = digest
            if not entry['records']:
                raise MapError('Journal is empty')
        except (OSError, ValueError, KeyError, TypeError, OverflowError, RecursionError) as exc:
            entry['error'] = f'Read-only recovery: {exc}'
        return entry

    @staticmethod
    def _validate_record(record, previous):
        action, data = record['action'], record['data']
        if not isinstance(data, dict) or not isinstance(record['request'], dict):
            raise MapError('Journal data/request must be objects')
        if record['request'].get('action') != action or record['request'].get('type') != 'mapping':
            raise MapError('Journal action/request mismatch')
        _text(record['request_id'], 'request_id', limit=128)
        if type(record['revision']) is not int:
            raise MapError('Journal revision must be an integer')
        def vector(value):
            if not isinstance(value, list) or len(value) != 2:
                raise MapError('Journal XY must have two numeric coordinates')
            for number in value:
                _number(number, 'journal coordinate')
        def bound(value):
            if value is not None:
                _number(value, 'journal uncertainty', positive=True)
        if not previous:
            if action != 'create':
                raise MapError('Invalid creation record')
            _fields(data, ('name', 'reference'))
            _text(data['name'], 'name', limit=120)
            _text(data['reference'], 'reference')
        elif action == 'register':
            if any(r['action'] == 'register' and r['data']['frame_id'] == data['frame_id'] for r in previous):
                raise MapError('Duplicate registration frame')
            _fields(data, ('translation_m', 'yaw_deg', 'control_points', 'uncertainty_m', 'evidence',
                           'registration_id', 'frame_id', 'reference_local_xy_m', 'yaw_uncertainty_deg',
                           'fit_rmse_m', 'fit_max_residual_m', 'max_residual_m'))
            vector(data['translation_m'])
            vector(data['reference_local_xy_m'])
            _number(data['yaw_deg'], 'journal yaw', bound=180)
            if data['yaw_uncertainty_deg'] is not None:
                _number(data['yaw_uncertainty_deg'], 'journal yaw bound', nonnegative=True, bound=180)
            bound(data['uncertainty_m'])
            for key in ('evidence', 'registration_id', 'frame_id'):
                _text(data[key], key)
            for key in ('fit_rmse_m', 'fit_max_residual_m', 'max_residual_m'):
                _number(data[key], key, nonnegative=True)
            if not isinstance(data['control_points'], list) or len(data['control_points']) < 2:
                raise MapError('Invalid journal control points')
            for point in data['control_points']:
                _fields(point, ('local_xy', 'map_xy'))
                vector(point['local_xy'])
                vector(point['map_xy'])
        elif action == 'observe':
            _fields(data, ('event_id', 'kind', 'shot_id', 'label', 'event_time', 'evidence', 'location'),
                    ('measurement', 'source_ref', 'supersedes'))
            if data['kind'] not in _KINDS:
                raise MapError('Invalid observation record')
            _text(data['event_id'], 'event_id')
            _text(data['label'], 'label')
            _text(data['evidence'], 'evidence')
            _text(data['shot_id'], 'shot_id', optional=(data['kind'] == 'landmark'))
            event_time = data['event_time']
            _fields(event_time, ('clock', 'value', 'uncertainty_s'))
            _text(event_time['clock'], 'clock')
            if event_time['value'] is not None:
                _number(event_time['value'], 'timestamp')
            elif event_time['uncertainty_s'] is not None:
                raise MapError('Unknown event time cannot have a time bound')
            if event_time['uncertainty_s'] is not None:
                _number(event_time['uncertainty_s'], 'timestamp bound', nonnegative=True)
            location = data['location']
            if not isinstance(location, dict):
                raise MapError('Journal location must be an object')
            if location.get('kind') == 'unknown':
                _fields(location, ('kind', 'reason'))
                _text(location['reason'], 'reason')
            elif location.get('kind') in ('local', 'surveyed'):
                _fields(location, ('kind', 'xy_m', 'uncertainty_m', 'evidence')
                        + (('frame_id',) if location['kind'] == 'local' else ()))
                vector(location['xy_m'])
                bound(location['uncertainty_m'])
                _text(location['evidence'], 'location evidence')
                if location['kind'] == 'local':
                    _text(location['frame_id'], 'frame_id')
            else:
                raise MapError('Invalid journal location kind')
            if 'measurement' in data:
                _fields(data['measurement'], ('quantity', 'value', 'unit', 'calibration_id', 'evidence'))
                for key, value in data['measurement'].items():
                    _number(value, key) if key == 'value' else _text(value, key)
            if 'source_ref' in data:
                _text(data['source_ref'], 'source_ref')
            if 'supersedes' in data:
                prior = next((r['data'] for r in previous if r['action'] == 'observe'
                              and r['data']['event_id'] == data['supersedes']), None)
                if (prior is None or prior['kind'] != data['kind'] or prior['shot_id'] != data['shot_id']
                        or any(r['action'] == 'observe' and r['data'].get('supersedes') == data['supersedes'] for r in previous)):
                    raise MapError('Invalid journal correction target')
        else:
            raise MapError('Invalid journal action')

    def _state(self, entry, revision=None):
        records = entry['records'] if revision is None else entry['records'][:revision]
        initial = records[0]['data'] if records else {}
        state = dict(schema=SCHEMA, map_id=(records[0]['map_id'] if records else self._active),
                     name=initial.get('name', ''), reference=initial.get('reference', ''),
                     units='m', axes=AXES, uncertainty_note=UNCERTAINTY,
                     revision=len(records), registrations=[], observations=[],
                     read_only=bool(entry['error']), error=entry['error'])
        for record in records[1:]:
            field = 'registrations' if record['action'] == 'register' else 'observations'
            state[field].append(deepcopy(record['data']))
        return _project(state)

    def status(self):
        entry = self._maps.get(self._active)
        return dict(map_id=self._active, revision=len(entry['records']) if entry else 0,
                    error=(entry['error'] if entry else None) or self._error,
                    read_only=bool((entry and entry['error']) or self._error),
                    available=not self._closed)

    def _available(self):
        return [dict(map_id=map_id, name=(entry['records'][0]['data']['name'] if entry['records'] else 'Damaged map'),
                     revision=len(entry['records']), read_only=bool(entry['error']), error=entry['error'])
                for map_id, entry in sorted(self._maps.items())]

    def _commit(self, map_id, request, action, data):
        entry = self._maps[map_id]
        if entry['error']:
            raise MapError(entry['error'])
        if len(entry['records']) >= config.MAPPING_MAX_RECORDS:
            raise MapError('Map record limit reached; export and create a new map')
        record = dict(schema=SCHEMA, map_id=map_id, revision=len(entry['records']) + 1,
                      request_id=request['request_id'], request_sha256=_sha(request), request=request,
                      action=action, data=data, recorded_at_utc=datetime.now(timezone.utc).isoformat(),
                      provenance=self._source,
                      previous_sha256=(entry['records'][-1]['sha256'] if entry['records'] else None))
        record['sha256'] = _sha(record)
        raw = _json(record) + b'\n'
        if len(raw) > config.MAPPING_MAX_RECORD_BYTES or entry['bytes'] + len(raw) > config.MAPPING_MAX_JOURNAL_BYTES:
            raise MapError('Map journal byte limit reached; export and create a new map')
        path = self._path(map_id) / 'journal.jsonl'
        try:
            if path.is_symlink():
                raise MapError('Journal must not be a symlink')
            with open(path, 'ab') as stream:
                if os.fstat(stream.fileno()).st_size != entry['bytes']:
                    raise MapError('Journal changed outside this writer')
                stream.write(raw)
                stream.flush()
                os.fsync(stream.fileno())
            self._directory_sync(path.parent)
        except (OSError, MapError) as exc:
            entry['error'] = f'Write outcome uncertain; reopen to recover before retrying: {exc}'
            raise MapError(entry['error']) from exc
        entry['records'].append(record)
        entry['requests'][request['request_id']] = record
        entry['bytes'] += len(raw)

    def handle(self, request):
        """Return JSON-safe final result; this method must have one owner thread."""
        result = dict(type='mapping_result', request_id=(request.get('request_id') if isinstance(request, dict) else None),
                      action=(request.get('action') if isinstance(request, dict) else None), ok=False)
        try:
            if self._closed:
                raise MapError('Mission-map store is closed')
            if not isinstance(request, dict):
                raise MapError('Mapping request must be an object')
            raw = _json(request)
            if len(raw) > config.MAPPING_MAX_REQUEST_BYTES:
                raise MapError('Mapping request exceeds size limit')
            request = json.loads(raw)
            if request.get('type') != 'mapping':
                raise MapError('type must be mapping')
            request_id = _text(request.get('request_id'), 'request_id', limit=128)
            action = request.get('action')
            base = ('type', 'request_id', 'action')
            if action == 'create':
                _fields(request, base + ('name', 'reference'))
                name = _text(request['name'], 'name', limit=120)
                reference = _text(request['reference'], 'reference')
                map_id = 'm_' + hashlib.sha256(request_id.encode('utf-8')).hexdigest()[:32]
                entry = self._maps.get(map_id)
                if entry:
                    prior = entry['requests'].get(request_id)
                    if prior is None or prior['request_sha256'] != _sha(request):
                        raise MapError('request_id collision')
                    if entry['error']:
                        raise MapError(entry['error'])
                    self._select(map_id)
                    result['state'] = self._state(entry, prior['revision'])
                else:
                    if len(self._maps) >= config.MAPPING_MAX_MAPS:
                        raise MapError('Map count limit reached')
                    path = self._path(map_id)
                    path.mkdir(exist_ok=False)
                    self._directory_sync(self.root)
                    self._maps[map_id] = dict(records=[], requests={}, error=None, bytes=0)
                    self._commit(map_id, request, action, dict(name=name, reference=reference))
                    self._select(map_id)
                    result['state'] = self._state(self._maps[map_id])
            elif action == 'open':
                _fields(request, base + ('map_id',))
                map_id = request['map_id']
                self._path(map_id)
                if map_id not in self._maps:
                    raise MapError('Unknown map_id')
                self._select(map_id)
                result['state'] = self._state(self._maps[map_id])
            elif action in ('snapshot', 'export'):
                _fields(request, base, ('map_id', 'expected_revision'))
                if 'map_id' in request and request['map_id'] != self._active:
                    raise MapError('Map selection changed; request a fresh snapshot')
                entry = self._maps.get(self._active)
                if action == 'snapshot':
                    state = (self._state(entry) if entry else
                             dict(schema=SCHEMA, map_id=None, revision=0, name='', reference='', units='m',
                                  axes=AXES, registrations=[], observations=[], read_only=bool(self._error),
                                  error=self._error, uncertainty_note=UNCERTAINTY))
                    state['available_maps'] = self._available()
                    result['state'] = state
                else:
                    if not entry:
                        raise MapError('Create or open a map first')
                    journal_path = self._path(self._active) / 'journal.jsonl'
                    if journal_path.is_symlink() or journal_path.stat().st_size > config.MAPPING_MAX_JOURNAL_BYTES:
                        raise MapError('Export journal is unsafe or exceeds the configured size limit')
                    # Hash the actual file, including any damaged tail, without
                    # exporting arbitrary filesystem paths or altering evidence.
                    digest = hashlib.sha256()
                    with open(journal_path, 'rb') as stream:
                        for block in iter(lambda: stream.read(65536), b''):
                            digest.update(block)
                    result['document'] = dict(schema=SCHEMA, state=self._state(entry),
                                              journal=deepcopy(entry['records']),
                                              journal_file_sha256=digest.hexdigest(),
                                              valid_prefix_only=bool(entry['error']),
                                              export_provenance=self._source)
            elif action in ('register', 'observe'):
                entry = self._maps.get(self._active)
                if not entry or request.get('map_id') != self._active:
                    raise MapError('Map selection changed; request a fresh snapshot')
                prior = entry['requests'].get(request_id)
                if prior:
                    if prior['request_sha256'] != _sha(request):
                        raise MapError('request_id collision')
                    result['state'] = self._state(entry, prior['revision'])
                else:
                    revision = request.get('expected_revision')
                    if type(revision) is not int or revision != len(entry['records']):
                        raise MapError('Revision conflict; refresh before recording')
                    if entry['error']:
                        raise MapError(entry['error'])
                    state = self._state(entry)
                    shared = base + ('map_id', 'expected_revision')
                    data = (self._registration(request, shared, state) if action == 'register'
                            else self._observation(request, shared, state))
                    self._commit(self._active, request, action, data)
                    result['state'] = self._state(entry)
            else:
                raise MapError('Unknown mapping action')
            result['ok'] = True
        except (OSError, ValueError, TypeError, KeyError, OverflowError, RecursionError) as exc:
            result['error'] = str(exc) or type(exc).__name__
        return result

    @staticmethod
    def _registration(request, shared, state):
        _fields(request, shared + ('frame_id', 'control_points', 'evidence', 'uncertainty_m', 'max_residual_m'),
                ('yaw_uncertainty_deg',))
        frame = _text(request['frame_id'], 'frame_id', limit=256)
        if any(reg['frame_id'] == frame for reg in state['registrations']):
            raise MapError('Frame registration is immutable; use a new map or estimator frame')
        fit = fit_registration(request['control_points'], request['max_residual_m'])
        fit.update(frame_id=frame, evidence=_text(request['evidence'], 'evidence'),
                   uncertainty_m=_bound(request['uncertainty_m'], 'uncertainty_m'),
                   yaw_uncertainty_deg=(None if request.get('yaw_uncertainty_deg') is None else
                                        _number(request['yaw_uncertainty_deg'], 'yaw_uncertainty_deg',
                                                nonnegative=True, bound=180)),
                   registration_id='r_' + hashlib.sha256(request['request_id'].encode()).hexdigest()[:32])
        return fit

    @staticmethod
    def _observation(request, shared, state):
        _fields(request, shared + ('kind', 'label', 'event_time', 'evidence', 'location'),
                ('shot_id', 'measurement', 'source_ref', 'supersedes'))
        kind = request['kind']
        if kind not in _KINDS:
            raise MapError('Unknown observation kind')
        shot = _text(request.get('shot_id'), 'shot_id', optional=(kind == 'landmark'), limit=128)
        event_time = request['event_time']
        _fields(event_time, ('clock', 'value', 'uncertainty_s'))
        if event_time['value'] is None and event_time['uncertainty_s'] is not None:
            raise MapError('Unknown event time must have uncertainty_s=null')
        event_time = dict(clock=_text(event_time['clock'], 'event_time.clock', limit=128),
                          value=(None if event_time['value'] is None else
                                 _number(event_time['value'], 'event_time.value', bound=1e15)),
                          uncertainty_s=(None if event_time['uncertainty_s'] is None else
                                         _number(event_time['uncertainty_s'], 'event_time.uncertainty_s', nonnegative=True)))
        location = request['location']
        if not isinstance(location, dict):
            raise MapError('location must be an object')
        loc_kind = location.get('kind')
        if loc_kind == 'unknown':
            _fields(location, ('kind', 'reason'))
            location = dict(kind=loc_kind, reason=_text(location['reason'], 'location.reason'))
        elif loc_kind in ('surveyed', 'local'):
            _fields(location, ('kind', 'xy_m', 'uncertainty_m', 'evidence')
                    + (('frame_id',) if loc_kind == 'local' else ()))
            location = dict(kind=loc_kind, xy_m=_xy(location['xy_m'], 'location.xy_m'),
                            uncertainty_m=_bound(location['uncertainty_m'], 'location.uncertainty_m'),
                            evidence=_text(location['evidence'], 'location.evidence'),
                            **(dict(frame_id=_text(location['frame_id'], 'location.frame_id', limit=256))
                               if loc_kind == 'local' else {}))
        else:
            raise MapError('location.kind must be surveyed, local or unknown')
        observation = dict(event_id='e_' + hashlib.sha256(request['request_id'].encode()).hexdigest()[:32],
                           kind=kind, shot_id=shot, label=_text(request['label'], 'label', limit=256),
                           event_time=event_time, evidence=_text(request['evidence'], 'evidence'), location=location)
        if 'source_ref' in request:
            observation['source_ref'] = _text(request['source_ref'], 'source_ref')
        if 'measurement' in request:
            measurement = request['measurement']
            _fields(measurement, ('quantity', 'value', 'unit', 'calibration_id', 'evidence'))
            observation['measurement'] = {key: (_number(value, 'measurement.value') if key == 'value'
                                               else _text(value, 'measurement.' + key))
                                          for key, value in measurement.items()}
        if 'supersedes' in request:
            target = _text(request['supersedes'], 'supersedes', limit=128)
            original = next((o for o in state['observations'] if o['event_id'] == target), None)
            if (not original or original['kind'] != kind or original['shot_id'] != shot
                    or original['superseded']):
                raise MapError('Correction must supersede a current event with the same kind and shot_id')
            observation['supersedes'] = target
        return observation

    def close(self):
        self._closed = True
        stream, self._lock_file = self._lock_file, None
        if stream is not None:
            if os.name == 'nt':
                import msvcrt
                stream.seek(0)
                msvcrt.locking(stream.fileno(), msvcrt.LK_UNLCK, 1)
            stream.close()


class MissionMapService:
    """Bounded asynchronous adapter. No disk I/O in submit() or status()."""
    def __init__(self, root_dir):
        self._root_dir = root_dir
        self._queue = queue.Queue(maxsize=config.MAPPING_QUEUE_MAX)
        self._lock = threading.Lock()
        self._closing = threading.Event()
        self._thread = None
        self._status = dict(map_id=None, revision=0, error=None, read_only=False, available=False)

    def start(self):
        with self._lock:
            if self._thread is not None or self._closing.is_set():
                return
            self._thread = threading.Thread(target=self._run, name='mission-map', daemon=True)
            self._thread.start()

    def status(self):
        with self._lock:
            return dict(self._status)

    @staticmethod
    def _reply(callback, result):
        try:
            callback(result)
        except Exception:
            # A disconnected client must not kill the persistence worker.
            pass

    def submit(self, request, callback):
        error = None
        try:
            if not isinstance(request, dict):
                raise MapError('Mapping request must be an object')
            raw = _json(request)
            if len(raw) > config.MAPPING_MAX_REQUEST_BYTES:
                raise MapError('Mapping request exceeds size limit')
            copied = json.loads(raw)
            with self._lock:
                if self._closing.is_set() or self._thread is None:
                    error = 'Mapping service is stopped'
                else:
                    try:
                        self._queue.put_nowait((copied, callback))
                    except queue.Full:
                        error = 'Mapping queue is full; refresh before retrying'
        except (ValueError, TypeError, OverflowError, RecursionError) as exc:
            error = str(exc)
        if error:
            self._reply(callback, dict(type='mapping_result',
                                       request_id=request.get('request_id') if isinstance(request, dict) else None,
                                       action=request.get('action') if isinstance(request, dict) else None,
                                       ok=False, error=error))
            return False
        return True

    def _run(self):
        store = None
        failure = None
        try:
            try:
                store = MissionMapStore(self._root_dir)
            except Exception as exc:
                failure = f'Mapping store unavailable: {exc}'
                with self._lock:
                    self._status.update(error=failure, read_only=True, available=False)
            while not self._closing.is_set() or not self._queue.empty():
                if store is not None:
                    with self._lock:
                        self._status = store.status()
                try:
                    request, callback = self._queue.get(timeout=0.1)
                except queue.Empty:
                    continue
                try:
                    if store is None:
                        result = dict(type='mapping_result', request_id=request.get('request_id'),
                                      action=request.get('action'), ok=False, error=failure)
                    else:
                        result = store.handle(request)
                        with self._lock:
                            self._status = store.status()
                    self._reply(callback, result)
                finally:
                    self._queue.task_done()
        finally:
            if store is not None:
                store.close()
            with self._lock:
                self._status['available'] = False

    def close(self):
        with self._lock:
            self._closing.set()
            thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(config.MAPPING_CLOSE_TIMEOUT_S)
        # A slow durable write may finish after this bounded wait. Keep its
        # writer lock held; never report completion or allow a second writer.
