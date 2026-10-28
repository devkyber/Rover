"""Validate portable wheel-distance calibration evidence without hardware imports.

A profile supplies one positive, dimensionless multiplier relative to explicitly
recorded wheel geometry. Loading it does not change any setting. A profile from a
different chassis is rejected even if its fitted error happens to be small.
"""
import datetime
import hashlib
import json
import math
from pathlib import Path
import re

import config


SCHEMA = 'rover-wheel-distance-calibration-v1'
SOURCE_ROLES = frozenset(('checkpoint_csv', 'raw_recording', 'replay_results', 'estimator_source'))
DATA_ROLES = frozenset(('calibration_training', 'previously_tuned_historical'))


class FrozenDict(dict):
    """Recursively frozen profile objects that remain JSON serializable."""
    def _immutable(self, *args, **kwargs):
        raise TypeError('Calibration provenance is immutable')

    __setitem__ = __delitem__ = clear = pop = popitem = setdefault = update = __ior__ = _immutable

    def __copy__(self):
        return self

    def __deepcopy__(self, memo):
        return self


def _freeze(value):
    if isinstance(value, dict):
        return FrozenDict((key, _freeze(item)) for key, item in value.items())
    if isinstance(value, list):
        return tuple(_freeze(item) for item in value)
    return value


def _fields(value, expected, label):
    if not isinstance(value, dict) or set(value) != set(expected):
        raise ValueError(f'{label} must contain exactly: {", ".join(sorted(expected))}')


def _number(value, label, *, positive=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{label} must be a finite number')
    try:
        number = float(value)
    except (ValueError, OverflowError):
        raise ValueError(f'{label} must be a finite number') from None
    if not math.isfinite(number) or (positive and number <= 0):
        raise ValueError(f'{label} must be finite' + (' and positive' if positive else ''))
    return number


def _text(value, label, limit=4096):
    if not isinstance(value, str) or not value.strip() or len(value) > limit or '\x00' in value:
        raise ValueError(f'{label} must be nonempty text of at most {limit} characters')
    return value


def validate_geometry(geometry):
    """Return normalized declared geometry; no inference from source names."""
    _fields(geometry, ('wheel_radius_m', 'wheelbase_m', 'wheel_sign'), 'geometry')
    signs = geometry['wheel_sign']
    if not isinstance(signs, (list, tuple)) or len(signs) != 4 or any(
            isinstance(value, bool) or not isinstance(value, int) or value not in (-1, 1)
            for value in signs):
        raise ValueError('geometry.wheel_sign must contain four integer signs, each -1 or +1')
    return {
        'wheel_radius_m': _number(geometry['wheel_radius_m'], 'geometry.wheel_radius_m', positive=True),
        'wheelbase_m': _number(geometry['wheelbase_m'], 'geometry.wheelbase_m', positive=True),
        'wheel_sign': list(signs),
    }


def fit_distance_scale(estimated_x, measured_x):
    """Fit one positive scale through zero; no offset, windows, or per-stop fit."""
    estimates, measurements = list(estimated_x), list(measured_x)
    if len(estimates) != len(measurements) or len(estimates) < 2:
        raise ValueError('At least two matching fixed checkpoints are required')
    x = [_number(value, 'estimated_x') for value in estimates]
    y = [_number(value, 'measured_x') for value in measurements]
    try:
        denominator = math.fsum(value * value for value in x)
        numerator = math.fsum(a * b for a, b in zip(x, y))
        scale = numerator / denominator if denominator > 0 else float('nan')
    except (OverflowError, ValueError):
        scale = float('nan')
    return _number(scale, 'fitted wheel_distance_scale', positive=True)


def validate_profile(profile, *, geometry):
    """Validate the complete v1 schema and exact declared chassis compatibility."""
    _fields(profile, ('schema', 'quantity', 'units', 'wheel_distance_scale', 'geometry',
                      'provenance', 'fit'), 'profile')
    if profile['schema'] != SCHEMA or profile['quantity'] != 'wheel_distance_scale':
        raise ValueError('Unsupported wheel calibration schema or quantity')
    if profile['units'] != 'dimensionless':
        raise ValueError('Wheel distance scale units must be dimensionless; geometry uses meters')
    scale = _number(profile['wheel_distance_scale'], 'wheel_distance_scale', positive=True)
    declared, requested = validate_geometry(profile['geometry']), validate_geometry(geometry)
    if declared != requested:
        raise ValueError(f'Wheel calibration geometry mismatch: profile={declared}, runtime={requested}')
    provenance = profile['provenance']
    _fields(provenance, ('created_utc', 'geometry_identity', 'data_role', 'training_runs',
                         'sources', 'limitations', 'unseen_validation', 'absolute_accuracy_validated'), 'provenance')
    timestamp = _text(provenance['created_utc'], 'provenance.created_utc', 80)
    try:
        created = datetime.datetime.fromisoformat(timestamp.replace('Z', '+00:00'))
    except ValueError:
        raise ValueError('provenance.created_utc must be an ISO8601 UTC timestamp') from None
    if created.tzinfo is None or created.utcoffset() != datetime.timedelta(0):
        raise ValueError('provenance.created_utc must identify UTC explicitly')
    _text(provenance['geometry_identity'], 'provenance.geometry_identity', 256)
    if not isinstance(provenance['data_role'], str) or provenance['data_role'] not in DATA_ROLES:
        raise ValueError('provenance.data_role must describe calibration training or historical tuning')
    runs = provenance['training_runs']
    if not isinstance(runs, list) or not 1 <= len(runs) <= 32:
        raise ValueError('provenance.training_runs must identify 1 to 32 runs')
    for run in runs:
        _text(run, 'training run', 256)
    if len(set(runs)) != len(runs):
        raise ValueError('Training run identifiers must be unique')
    sources = provenance['sources']
    if not isinstance(sources, list) or not 3 <= len(sources) <= 64:
        raise ValueError('provenance.sources must retain checkpoint, raw, and replay-result identities')
    roles, paths = set(), set()
    for source in sources:
        _fields(source, ('path', 'sha256', 'role'), 'provenance source')
        path = _text(source['path'], 'source.path')
        if path in paths:
            raise ValueError('Each provenance source path must appear once')
        paths.add(path)
        if not isinstance(source['sha256'], str) or re.fullmatch(r'[0-9a-f]{64}', source['sha256']) is None:
            raise ValueError('Each provenance source needs a lowercase SHA256')
        if not isinstance(source['role'], str) or source['role'] not in SOURCE_ROLES:
            raise ValueError('Unsupported provenance source role')
        roles.add(source['role'])
    if not {'checkpoint_csv', 'raw_recording', 'replay_results'} <= roles:
        raise ValueError('Checkpoint CSV, original recording, and replay-result sources are required')
    limitations = provenance['limitations']
    if not isinstance(limitations, list) or not 1 <= len(limitations) <= 32:
        raise ValueError('Profile must retain its calibration limitations')
    for item in limitations:
        _text(item, 'provenance limitation')
    if provenance['unseen_validation'] is not False or provenance['absolute_accuracy_validated'] is not False:
        raise ValueError('Calibration profile cannot claim unseen validation or validated absolute accuracy')
    fit = profile['fit']
    _fields(fit, ('method', 'measured_checkpoint_count', 'initial_reference_count', 'fixed_timestamps',
                  'checkpoint_columns', 'training_rmse_x_m', 'training_max_abs_x_error_m'), 'fit')
    if fit['method'] != 'zero_intercept_least_squares' or fit['fixed_timestamps'] is not True:
        raise ValueError('Profile requires fixed checkpoints and one zero-intercept least-squares scale')
    count = fit['measured_checkpoint_count']
    if isinstance(count, bool) or not isinstance(count, int) or count < 2:
        raise ValueError('At least two measured checkpoints are required')
    if type(fit['initial_reference_count']) is not int or fit['initial_reference_count'] not in (0, 1):
        raise ValueError('initial_reference_count must be 0 or 1; origin is not independent truth')
    columns = fit['checkpoint_columns']
    _fields(columns, ('estimated_x', 'measured_x', 'run', 'time', 'sample_time'), 'checkpoint columns')
    for value in columns.values():
        _text(value, 'checkpoint column', 256)
    for key in ('training_rmse_x_m', 'training_max_abs_x_error_m'):
        if _number(fit[key], key) < 0:
            raise ValueError(f'{key} must be nonnegative')
    if fit['training_max_abs_x_error_m'] < fit['training_rmse_x_m']:
        raise ValueError('Maximum training error cannot be smaller than RMS error')
    return scale


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f'Duplicate JSON key: {key}')
        result[key] = value
    return result


def load_profile(path, *, geometry):
    """Load an explicit compatible profile; return scale and immutable evidence.

    Source SHA256/path fields preserve the recorded calibration identity. Source
    recordings need not be installed on the Rover: this loader verifies schema
    and chassis compatibility, while the fitter verifies bytes when creating the
    profile. It also hashes the exact profile JSON bytes used for this run.
    """
    source = Path(path).expanduser().resolve()
    maximum_bytes = config.WHEEL_CALIBRATION_PROFILE_MAX_BYTES
    if type(maximum_bytes) is not int or maximum_bytes <= 0:
        raise ValueError('WHEEL_CALIBRATION_PROFILE_MAX_BYTES must be a positive integer')
    try:
        with source.open('rb') as stream:
            raw = stream.read(maximum_bytes + 1)
    except OSError as error:
        raise ValueError(f'Cannot read wheel calibration profile: {source}: {error}') from error
    if len(raw) > maximum_bytes:
        raise ValueError(f'Wheel calibration profile exceeds {maximum_bytes} bytes')
    try:
        profile = json.loads(raw.decode('utf-8'), object_pairs_hook=_unique_object,
                             parse_constant=lambda value: (_ for _ in ()).throw(ValueError(f'Nonfinite JSON constant: {value}')))
    except (UnicodeError, json.JSONDecodeError, RecursionError) as error:
        raise ValueError(f'Invalid wheel calibration JSON: {error}') from error
    scale = validate_profile(profile, geometry=geometry)
    return {'wheel_distance_scale': scale, 'provenance': _freeze({
        'profile_path': str(source), 'profile_sha256': hashlib.sha256(raw).hexdigest(),
        'profile_schema': SCHEMA, 'profile': profile,
    })}
