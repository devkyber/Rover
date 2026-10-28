"""Reproduce full-rate plots of the latest two observed Scout 2 jump bursts."""
from pathlib import Path
import csv
import hashlib
import json
import sys
import zipfile
import argparse

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT / 'rover'))
from vendor.jumper.scout_client import decode_record

DTYPE = np.dtype([('t', '<u8'), ('seq', '<u4'), ('sensor', 'u1'),
                  ('flags', 'u1'), ('size', '<u2'), ('payload', 'V20'), ('crc', '<u4')])
COLORS = ['#2672bd', '#d58028', '#269477']

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def sensor_arrays(records, sensor):
    selected = records[records['sensor'] == sensor].copy()
    xyz = np.ndarray((len(selected), 3), dtype='<i2', buffer=selected,
                     offset=16, strides=(40, 2)).copy()
    scale = 1 / 2048 if sensor == 1 else .049
    result = dict(t=selected['t'].astype(float) / 1e6, xyz=xyz * scale,
                  flags=selected['flags'].copy(), sequence=selected['seq'].copy())
    result['magnitude'] = np.linalg.norm(result['xyz'], axis=1)
    if sensor == 1:
        result['gyro'] = np.ndarray((len(selected), 3), dtype='<i2', buffer=selected,
                                    offset=22, strides=(40, 2)).copy() / 16.4
    return result

def break_line(t, y, origin, nominal_dt):
    # Never reorder timestamp reversals or bridge conspicuous sampling gaps.
    gap = np.flatnonzero((np.diff(t) <= 0) | (np.diff(t) > nominal_dt * 3)) + 1
    return np.insert(t - origin, gap, np.nan), np.insert(y, gap, np.nan)

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path, help='New output directory; must not exist')
    args=parser.parse_args()
    output=args.output.resolve(); output.mkdir(parents=True, exist_ok=False)
    archive=ROOT / 'logs/scout_jumps/2026-10-24_two_jumps.zip'
    with zipfile.ZipFile(archive) as z:
        member=next(n for n in z.namelist() if n.endswith('scout_0006_cb00b4ba.records.bin'))
        raw_bytes=z.read(member)
    raw_name=archive.relative_to(ROOT).as_posix()+'!'+member
    raw_sha=hashlib.sha256(raw_bytes).hexdigest()
    all_records=np.frombuffer(raw_bytes, dtype=DTYPE)
    windows = [(396.7, 399.5), (516.2, 519.0)]
    pulls = {}
    request_ids = ['748d5320-80db-4e42-9311-d0945d1d7dfc', 'e604a10a-1fcc-45e3-b6ff-ecc6f976fa49']
    for line in (ROOT / 'logs/scout_jumps/selected_pull_events.jsonl').open(encoding='utf-8'):
        if any(request in line for request in request_ids):
            event = json.loads(line)
            if event['kind'] == 'result':
                pulls[event['result']['request_id']] = event['result']
    shots, results = [], []
    for number, (lo, hi) in enumerate(windows, 1):
        slots = np.flatnonzero((all_records['t'] >= lo * 1e6) & (all_records['t'] <= hi * 1e6))
        start, end = int(slots.min()), int(slots.max()) + 1
        records = all_records[start:end]
        rows = []
        for slot, record in enumerate(records, start):
            row = decode_record(record.tobytes(), units_verified=True)
            row['record_slot'] = slot
            row['scale_basis'] = 'nominal firmware scaling; not measured calibration'
            rows.append(row)
        keys = list(dict.fromkeys(key for row in rows for key in row))
        with (output / f'jump_{number}_records.csv').open('w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, keys)
            writer.writeheader()
            writer.writerows(rows)
        icm, adxl = sensor_arrays(records, 1), sensor_arrays(records, 2)
        launch = float(icm['t'][np.flatnonzero(icm['magnitude'] > 3)[0]])
        low = icm['magnitude'] < .2
        bounds = np.flatnonzero(np.diff(np.r_[False, low, False])).reshape(-1, 2)
        first, last = max(bounds, key=lambda pair: pair[1] - pair[0])
        low_start, low_end = float(icm['t'][first]), float(icm['t'][last - 1])
        contact = float(icm['t'][np.flatnonzero((icm['t'] > low_end) & (icm['magnitude'] > 3))[0]])
        stat = dict(jump=number, source_raw=raw_name,
                    source_sha256=raw_sha, source_record_slot_start=start,
                    source_record_slot_end_exclusive=end, source_byte_start=start * 40,
                    source_byte_end_exclusive=end * 40, crc_checked_records=len(rows),
                    device_window_s=[lo, hi], launch_burst_onset_device_s=launch,
                    low_g_interval_device_s=[low_start, low_end],
                    low_g_duration_s=low_end - low_start,
                    contact_burst_onset_device_s=contact,
                    burst_onset_interval_s=contact - launch,
                    icm_peak_nominal_g=float(icm['magnitude'].max()),
                    adxl_peak_nominal_g=float(adxl['magnitude'].max()),
                    icm_saturation_flag_samples=int(np.count_nonzero(icm['flags'] & 4)),
                    global_sequence_gaps=int(np.maximum(np.diff(records['seq'].astype('i8')) - 1, 0).sum()),
                    global_sequence_regressions=int(np.count_nonzero(np.diff(records['seq'].astype('i8')) <= 0)))
        for name, sensor in [('icm', icm), ('adxl', adxl)]:
            before = sensor['t'] < low_start
            after = sensor['t'] > low_end
            for kind, mask in [('launch', before), ('contact', after)]:
                ix = np.flatnonzero(mask)[np.argmax(sensor['magnitude'][mask])]
                stat[f'{name}_{kind}_peak_nominal_g'] = float(sensor['magnitude'][ix])
                stat[f'{name}_{kind}_peak_device_s'] = float(sensor['t'][ix])
            stat[name + '_timestamp_nonincreasing_samples'] = int(np.count_nonzero(np.diff(sensor['t']) <= 0))
            stat[name + '_largest_positive_timestamp_step_ms'] = float(np.diff(sensor['t']).max() * 1000)
        stat['pull_result'] = pulls[request_ids[number - 1]]
        stat['pull_ack_device_s'] = stat['pull_result']['ack']['time_us'] / 1e6
        shots.append(dict(icm=icm, adxl=adxl, stat=stat))
        results.append(stat)

    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10,
                         'axes.spines.top': False, 'axes.spines.right': False,
                         'axes.titleweight': 'bold', 'axes.labelcolor': '#344454',
                         'text.color': '#233441', 'axes.edgecolor': '#b9c3cd',
                         'savefig.facecolor': 'white'})
    fig, axes = plt.subplots(3, 2, figsize=(14, 9.2), sharex='col', sharey='row')
    fig.subplots_adjust(top=.80, bottom=.14, left=.075, right=.98, hspace=.21, wspace=.12)
    fig.text(.075, .963, 'SCOUT 2  /  TWO CONSECUTIVE JUMPS', fontsize=21, weight='bold')
    interval = results[1]['launch_burst_onset_device_s'] - results[0]['launch_burst_onset_device_s']
    fig.text(.075, .928, f'Received sensor records  |  Launch bursts {interval:.3f} s apart  |  Full-rate waveforms', fontsize=11, color='#607180')
    for col, shot in enumerate(shots):
        stat, icm, adxl = shot['stat'], shot['icm'], shot['adxl']
        origin = stat['launch_burst_onset_device_s']
        contact = stat['contact_burst_onset_device_s'] - origin
        axes[0, col].set_title(f'Jump {col + 1}  |  ESP {origin:.3f} s', loc='left', pad=28, fontsize=13)
        axes[0, col].text(0, 1.035,
            f'ADXL peak {stat["adxl_peak_nominal_g"]:.1f} g  |  Contact peak {stat["adxl_contact_peak_nominal_g"]:.1f} g',
            transform=axes[0, col].transAxes, fontsize=10, color='#607180')
        for row, (sensor, key, dt) in enumerate([(icm, 'xyz', .0025), (adxl, 'xyz', .00125), (icm, 'gyro', .0025)]):
            ax = axes[row, col]
            for index, color in enumerate(COLORS):
                x, y = break_line(sensor['t'], sensor[key][:, index], origin, dt)
                ax.plot(x, y, lw=.7, alpha=.8, color=color, label='XYZ'[index])
            if row < 2:
                x, y = break_line(sensor['t'], sensor['magnitude'], origin, dt)
                ax.plot(x, y, color='#182c3d', lw=1.1, label='Magnitude')
            ax.axvspan(stat['low_g_interval_device_s'][0] - origin,
                       stat['low_g_interval_device_s'][1] - origin, color='#45a9c4', alpha=.12, zorder=0)
            ax.axvline(0, c='#bb7040', ls='--', lw=.8)
            ax.axvline(contact, c='#7b597e', ls='--', lw=.8)
            ax.axhline(0, color='#bdc8d2', lw=.6, zorder=0)
            ax.grid(alpha=.18)
            ax.set_xlim(-.35, 2.2)
        sat = (icm['flags'] & 4) != 0
        axes[0, col].scatter(icm['t'][sat] - origin, icm['magnitude'][sat],
                              s=13, color='#c54550', marker='x', zorder=5, label='Range flag')
        axes[0, col].text(.02, .96, 'Launch', transform=axes[0, col].transAxes, color='#a45f34', va='top')
        axes[0, col].annotate('Contact candidate', xy=(contact, 29), xytext=(contact + .13, 34),
                             fontsize=9, color='#775376', arrowprops=dict(arrowstyle='-', color='#775376'))
        axes[2, col].set_xlabel('Time from launch-burst onset (s)')
    axes[0, 0].set_ylabel('ICM-45605 acceleration (g)')
    axes[1, 0].set_ylabel('ADXL375 acceleration (g)')
    axes[2, 0].set_ylabel('ICM angular rate (deg/s)')
    axes[0, 0].set_ylim(-19, 39)
    axes[1, 0].set_ylim(-46, 64)
    axes[2, 0].set_ylim(-1650, 1650)
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper right', bbox_to_anchor=(.98, .916), frameon=False, ncol=5, fontsize=9)
    fig.text(.075, .055, 'Nominal scales; uncalibrated. Blue shading = ICM |a| < 0.2 g interval, not validated flight duration.', fontsize=9, color='#607180')
    fig.text(.075, .032, 'FIFO timestamps are estimated; reversals/gaps are not bridged. ICM range flags shown in red. Jetson wall clock invalid.', fontsize=9, color='#607180')
    fig.savefig(output / 'two_jumps.png', dpi=180)
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(11.5, 7), sharex=True, layout='constrained')
    fig.suptitle('SCOUT 2  /  JUMP COMPARISON\nNominal acceleration magnitudes, aligned at launch-burst onset', fontsize=16, weight='bold')
    for shot, color in zip(shots, ['#2672bd', '#d58028']):
        stat = shot['stat']
        for ax, name, dt in zip(axes, ['icm', 'adxl'], [.0025, .00125]):
            sensor = shot[name]
            x, y = break_line(sensor['t'], sensor['magnitude'], stat['launch_burst_onset_device_s'], dt)
            ax.plot(x, y, lw=1, color=color, label=f'Jump {stat["jump"]}')
            ax.grid(alpha=.2)
            ax.legend(frameon=False)
            ax.set_xlim(-.15, 1.6)
    axes[0].set_ylabel('ICM magnitude (g)')
    axes[1].set_ylabel('ADXL magnitude (g)')
    axes[1].set_xlabel('Time from launch-burst onset (s)')
    fig.savefig(output / 'comparison.png', dpi=180)
    plt.close(fig)
    summary = dict(scout_id='scout_2', field_calibrated=False, flight_duration_validated=False,
                    launch_burst_separation_s=interval, shots=results,
                    method=dict(scales='ICM raw/2048 g; ADXL raw*0.049 g; gyro raw/16.4 deg/s',
                        selection='Latest two pronounced launch/freefall/contact bursts in raw 0006; consistent with latest two PULL requests. Raw 0007 was also checked and has no later comparable burst.',
                        event_candidates='Launch: first ICM magnitude >3g in each selected window. Low-g: longest contiguous received ICM run below0.2g. Contact: first ICM >3g after that run. Heuristic event candidates, not independent physical labels.',
                        plot='All selected received samples, original order. Insert line breaks on nonincreasing timestamps or intervals above3 nominal periods. No smoothing, sorting, interpolation or resampling.',
                        limits='Nominal scales and FIFO estimated timestamps. ICM axis clipping, missing records and timestamp overlap retained. No height, range, trajectory or calibrated peak claim.'))
    (output / 'summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
    provenance = dict(capture_manifest='logs/scout_jumps/provenance/capture_manifest.json',
                      original_raw_sha256=raw_sha,
                      local_decoder_sha256=sha(ROOT / 'rover/vendor/jumper/scout_client.py'),
                      analysis_script_sha256=sha(Path(__file__)),
                      outputs={p.name: sha(p) for p in output.iterdir() if p.is_file() and p.name not in ('MANIFEST.json', 'preview.png')})
    (output / 'MANIFEST.json').write_text(json.dumps(provenance, indent=2), encoding='utf-8')
    print(json.dumps(summary, indent=2))

if __name__ == '__main__':
    main()
