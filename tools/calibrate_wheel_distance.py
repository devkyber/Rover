"""Fit an explicit wheel-distance profile from fixed measured checkpoints.

This offline tool never imports drivers, chooses stop
times, edits old evidence, or activates a profile. Geometry, units, training run,
original recording and replay result must be declared explicitly. Output must
be a new file. Training error is not unseen validation or absolute accuracy.
"""
import argparse
import csv
import datetime
import hashlib
import io
import json
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'rover'))
from odometry_calibration import (DATA_ROLES, SCHEMA, fit_distance_scale,
                                  validate_geometry, validate_profile)


def file_identity(path, role):
    resolved = Path(path).resolve()
    digest = hashlib.sha256()
    with resolved.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return {'path': str(resolved), 'sha256': digest.hexdigest(), 'role': role}


def finite_csv_number(value, label):
    try:
        number = float(value)
    except (ValueError, TypeError, OverflowError):
        raise ValueError(f'{label} must be finite numeric CSV data') from None
    if not math.isfinite(number):
        raise ValueError(f'{label} must be finite numeric CSV data')
    return number


def build_profile(args):
    if args.input_wheel_distance_scale != 1.0:
        raise ValueError('Input replay must use wheel_distance_scale=1.0; already-calibrated output cannot be fitted again')
    geometry = validate_geometry({'wheel_radius_m': args.wheel_radius_m,
                                  'wheelbase_m': args.wheelbase_m,
                                  'wheel_sign': args.wheel_sign})
    maximum_csv_bytes = 16 * 1024 * 1024
    with args.checkpoints.open('rb') as stream:
        raw = stream.read(maximum_csv_bytes + 1)
    if len(raw) > maximum_csv_bytes:
        raise ValueError('Checkpoint CSV exceeds 16 MiB limit')
    reader = csv.DictReader(io.StringIO(raw.decode('utf-8-sig')))
    columns = {'estimated_x': args.estimated_column, 'measured_x': args.measured_column,
               'run': args.run_column, 'time': args.time_column, 'sample_time': args.sample_time_column}
    if reader.fieldnames is None or len(set(reader.fieldnames)) != len(reader.fieldnames) or not set(columns.values()) <= set(reader.fieldnames):
        raise ValueError('CSV needs unique headers and all declared checkpoint columns')
    selected = [row for row in reader if row[args.run_column] == args.training_run]
    if len(selected) < 2:
        raise ValueError('Training run needs at least two fixed measured checkpoints')
    x, y, times = [], [], []
    for row in selected:
        x.append(finite_csv_number(row[args.estimated_column], 'estimated distance'))
        y.append(finite_csv_number(row[args.measured_column], 'measured distance'))
        timestamp = finite_csv_number(row[args.time_column], 'archived checkpoint time')
        sample_time = finite_csv_number(row[args.sample_time_column], 'replay sample time')
        if timestamp != sample_time:
            raise ValueError('Checkpoint times must match original replay samples exactly; no time-window fitting')
        times.append(timestamp)
    if any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError('Selected fixed checkpoints must have unique increasing timestamps')
    fit_x, fit_y = list(x), list(y)
    if args.include_local_origin_anchor:
        fit_x.insert(0, 0.0)
        fit_y.insert(0, 0.0)
    scale = fit_distance_scale(fit_x, fit_y)
    errors = [scale * estimate - truth for estimate, truth in zip(x, y)]
    sources = [{'path': str(args.checkpoints.resolve()), 'sha256': hashlib.sha256(raw).hexdigest(),
                'role': 'checkpoint_csv'}]
    sources += [file_identity(path, 'raw_recording') for path in args.raw_recording]
    sources.append(file_identity(args.replay_results, 'replay_results'))
    sources += [file_identity(path, 'estimator_source') for path in args.estimator_source]
    # Bind checkpoint CSV rows and original recording hashes to the identified
    # replay-result metadata. This avoids creating a profile with unrelated files.
    replay = json.loads(args.replay_results.read_text(encoding='utf-8'))
    candidates = [result for result in replay.get('results', [])
                  if Path(result.get('input', {}).get('path', '')).name == args.training_run]
    if len(candidates) != 1:
        raise ValueError('Replay results must identify the exact training recording once')
    result = candidates[0]
    for options in (result.get('source', {}), result.get('estimator_options', {})):
        for key in ('wheel_distance_scale', 'training_fold_output_distance_scale'):
            if key in options and options[key] != 1.0:
                raise ValueError('Replay metadata identifies already-calibrated output; use an unscaled replay')
    if result['input'].get('sha256') not in {source['sha256'] for source in sources if source['role'] == 'raw_recording'}:
        raise ValueError('Original recording hash does not match training-run replay result')
    if replay.get('checkpoint_csv', {}).get('sha256') != sources[0]['sha256']:
        raise ValueError('Checkpoint CSV hash does not match identified replay results')
    archived = result.get('events', [])
    if len(archived) != len(selected) or any(
            finite_csv_number(event.get('archived_host_t_s'), 'result checkpoint time') != timestamp
            or finite_csv_number(event.get('candidate_x_m'), 'result estimate') != estimate
            or finite_csv_number(event.get('measured_tape_x_m'), 'result measured distance') != truth
            for event, timestamp, estimate, truth in zip(archived, times, x, y)):
        raise ValueError('Selected fixed checkpoints do not match replay-result values and timestamps')
    overrides = result.get('source', {}).get('overrides', {})
    for key, field in [('WHEEL_RADIUS_M', 'wheel_radius_m'), ('WHEELBASE_M', 'wheelbase_m')]:
        recorded = overrides.get(key, {}).get('replay_value')
        if recorded != geometry[field]:
            raise ValueError(f'Declared {field} does not match identified replay configuration')
    profile = {
        'schema': SCHEMA, 'quantity': 'wheel_distance_scale', 'units': 'dimensionless',
        'wheel_distance_scale': scale, 'geometry': geometry,
        'provenance': {
            'created_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
            'geometry_identity': args.geometry_id, 'data_role': args.data_role,
            'training_runs': [args.training_run], 'sources': sources,
            'limitations': [
                'One zero-intercept distance scale fitted to declared longitudinal tape checkpoints; no fitted offsets or selected time windows.',
                'Input replay explicitly declared wheel_distance_scale=1.0; do not fit this profile again to already-calibrated output.',
                'Training error is not unseen validation; total position and heading accuracy remain unvalidated.',
                'Only matching declared wheel radius, wheelbase, and wheel signs may load this profile. Recalibrate after geometry, load, or surface changes.',
                'Tape uncertainty and lateral/heading ground truth were not supplied; this profile cannot establish absolute accuracy.',
            ] + (['These historical recordings were already used in estimator tuning; this is a historical calibration profile, not present BMI088 chassis calibration.']
                 if args.data_role == 'previously_tuned_historical' else []),
            'unseen_validation': False, 'absolute_accuracy_validated': False,
        },
        'fit': {
            'method': 'zero_intercept_least_squares', 'measured_checkpoint_count': len(x),
            'initial_reference_count': int(args.include_local_origin_anchor), 'fixed_timestamps': True,
            'checkpoint_columns': columns,
            'training_rmse_x_m': math.sqrt(math.fsum(error * error for error in errors) / len(errors)),
            'training_max_abs_x_error_m': max(abs(error) for error in errors),
        },
    }
    validate_profile(profile, geometry=geometry)
    return profile


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('checkpoints', type=Path)
    parser.add_argument('--training-run', required=True)
    parser.add_argument('--geometry-id', required=True)
    parser.add_argument('--wheel-radius-m', required=True, type=float)
    parser.add_argument('--wheelbase-m', required=True, type=float)
    parser.add_argument('--wheel-sign', required=True, nargs=4, type=int)
    parser.add_argument('--distance-unit', required=True, choices=['m'])
    parser.add_argument('--data-role', required=True, choices=sorted(DATA_ROLES))
    parser.add_argument('--input-wheel-distance-scale', required=True, type=float,
                        help='Must explicitly declare 1.0; already-calibrated replay output is rejected')
    parser.add_argument('--raw-recording', required=True, action='append', type=Path)
    parser.add_argument('--replay-results', required=True, type=Path)
    parser.add_argument('--estimator-source', action='append', type=Path, default=[])
    parser.add_argument('--estimated-column', default='candidate_x_m')
    parser.add_argument('--measured-column', default='measured_tape_x_m')
    parser.add_argument('--run-column', default='run')
    parser.add_argument('--time-column', default='archived_host_t_s')
    parser.add_argument('--sample-time-column', default='sample_host_t_s')
    parser.add_argument('--include-local-origin-anchor', action='store_true',
                        help='Include (0,0) local reference; it is not another independent tape measurement')
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args(argv)
    if args.output.exists():
        parser.error('Output must be a new file; refusing to overwrite existing evidence')
    try:
        profile = build_profile(args)
        serialized = json.dumps(profile, indent=2, ensure_ascii=False, allow_nan=False) + '\n'
        with args.output.open('x', encoding='utf-8') as stream:
            stream.write(serialized)
    except (OSError, ValueError, KeyError, TypeError, UnicodeError) as error:
        parser.error(str(error))
    print(f'Saved {args.output.resolve()} (wheel_distance_scale={profile["wheel_distance_scale"]:.12g})')


if __name__ == '__main__':
    main()
