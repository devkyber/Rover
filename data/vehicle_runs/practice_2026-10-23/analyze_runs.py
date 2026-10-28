"""Analyze captured scalar evidence only; never replay today's data with new code.

No hardware access. Means/availability use left-sample time weights, excluding
CSV intervals >0.5s. Encoder travel uses trapezoids of mean signed wheel speed;
it is nominal rolling travel, not ground truth. Run frames are never stitched.
"""
from pathlib import Path
import csv
import hashlib
import json
from collections import Counter
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
CAPTURE = ROOT / 'logs/drives'


def num(rows, key):
    def cv(value):
        try:
            return float(value)
        except (TypeError, ValueError):
            return np.nan
    return np.array([cv(row.get(key)) for row in rows])


def stats(x):
    a = np.asarray(x)
    a = a[np.isfinite(a)]
    if not len(a):
        return None
    return dict(n=len(a), min=float(a.min()), median=float(np.median(a)), p95=float(np.percentile(a,95)), max=float(a.max()))


def hashes(paths):
    return [dict(path=str(p.relative_to(ROOT)).replace('\\','/'), bytes=p.stat().st_size, sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in paths]


def analyze(session):
    m = json.loads((session/'metadata.json').read_text(encoding='utf-8'))
    files = sorted((session/'rover').glob('system_*.csv'))
    rows = []
    for p in files:
        with p.open(newline='',encoding='utf-8') as stream:
            rows.extend(csv.DictReader(stream))
    invalid_rows=[r for r in rows if not r.get('captured_monotonic_s') or None in r or any(v is None for v in r.values())]
    rows=[r for r in rows if r not in invalid_rows]
    rows.sort(key=lambda r: float(r['captured_monotonic_s']))
    if not rows:
        return dict(session=session.name, run_id=m['run_id'],rows=0,arguments=m['metadata'].get('arguments'),incomplete_csv_rows=len(invalid_rows),metadata=m)
    t = num(rows,'captured_monotonic_s')
    dt = np.diff(t)
    usable = (dt>0) & (dt<=0.5)
    weights = np.r_[np.where(usable,dt,0),0]
    duration = float(t[-1]-t[0])
    covered = float(weights.sum())
    wheels = np.stack([num(rows,'wheel_'+k+'_speed_mps') for k in ('fl','fr','rl','rr')])
    complete = np.isfinite(wheels).all(axis=0)
    v = wheels.mean(axis=0)
    maxwheel = np.max(np.abs(wheels),axis=0)
    moving = complete & (maxwheel>0.01)
    commanded = (np.abs(num(rows,'cmd_forward'))>0.01)|(np.abs(num(rows,'cmd_turn'))>0.01)
    def seconds(mask): return float(weights[mask].sum())
    def fraction(key, value='True'):
        present=np.array([r.get(key,'')!='' for r in rows])
        selected=np.array([r.get(key)==value for r in rows])
        return dict(seconds=seconds(selected), covered_fraction=seconds(selected)/seconds(present) if seconds(present) else None, rows=int(selected.sum()), present_rows=int(present.sum()))
    def counts(key): return dict(Counter(r.get(key,'') for r in rows))
    def travel(motion_only=False):
        ok=usable & complete[:-1] & complete[1:]
        if motion_only: ok &= moving[:-1] | moving[1:]
        return float(np.sum(((np.abs(v[:-1])+np.abs(v[1:]))/2*dt)[ok]))
    estvalid = np.array([r.get('pose_estimate_available')=='True' for r in rows])
    scientific = np.array([r.get('pose_valid')=='True' for r in rows])
    xy = np.stack([num(rows,'estimate_pos_'+k+'_m') for k in ('x','y')],axis=1)
    pos = xy[estvalid & np.isfinite(xy).all(axis=1)]
    if not len(pos):
        xy=np.stack([num(rows,'pos_'+k+'_m') for k in ('x','y')],axis=1)
        pos=xy[scientific & np.isfinite(xy).all(axis=1)]
    loss = num(rows,'pose_loss_count')
    lossfinite = loss[np.isfinite(loss)]
    starts = np.flatnonzero(moving & ~np.r_[False,moving[:-1]])
    ends = np.flatnonzero(moving & ~np.r_[moving[1:],False])
    bouts=[]
    for a,b in zip(starts,ends):
        if t[b]-t[a]>=0.3:
            bouts.append(dict(start_s=float(t[a]-t[0]), end_s=float(t[b]-t[0]), duration_s=float(t[b]-t[a]), peak_wheel_mps=float(np.nanmax(maxwheel[a:b+1])), command_seconds=seconds((np.arange(len(t))>=a)&(np.arange(len(t))<=b)&commanded)))
    events=[]; commands=[]
    for folder, out in [('rover/events.jsonl',events),('rover/commands.jsonl',commands)]:
        p=session/folder
        if p.exists():
            for line in p.read_text(encoding='utf-8').splitlines():
                try: out.append(json.loads(line))
                except json.JSONDecodeError: pass
    summ=session/'summary.json'
    result=dict(session=session.name,run_id=m['run_id'],recorded_created_host_utc=m.get('created_host_utc'),created_monotonic_s=m.get('created_monotonic_s'),
        arguments=m['metadata'].get('arguments'),description=m['metadata'].get('rover_description'),rows=len(rows),incomplete_csv_rows=len(invalid_rows),duration_s=duration,
        covered_s=covered,csv_interval_s=stats(dt),csv_gaps_over_0_5s=int((dt>0.5).sum()),csv_uncovered_gap_s=float(dt[dt>0.5].sum()),
        moving_s=seconds(moving),commanded_s=seconds(commanded),moving_without_command_s=seconds(moving & ~commanded),
        encoder_mean_body_travel_m=travel(),encoder_motion_body_travel_m=travel(True),wheel_speed_stats_mps=stats(maxwheel[complete]),body_speed_stats_mps=stats(v),
        pose_valid=fraction('pose_valid'),pose_estimate_available=fraction('pose_estimate_available'),pose_quality_counts=counts('pose_quality'),
        motion_pose_valid_s=seconds(moving & scientific),motion_estimate_available_s=seconds(moving & estvalid),
        pose_loss_first=float(lossfinite[0]) if len(lossfinite) else None,pose_loss_last=float(lossfinite[-1]) if len(lossfinite) else None,
        pose_loss_increments=int(np.maximum(np.diff(lossfinite),0).sum()) if len(lossfinite) else None,
        imu_status_counts=counts('imu_status'),imu_errors=counts('imu_error'),motors_status_counts=counts('motors_status'),
        battery_v=stats(num(rows,'battery_v')), battery_a=stats(num(rows,'battery_a')),battery_peak_w=stats(num(rows,'battery_w')),
        battery_below_21v_s=seconds(num(rows,'battery_v')<=21),battery_below_19_8v_s=seconds(num(rows,'battery_v')<=19.8),
        loop_hz=stats(num(rows,'loop_hz')),imu_age_s=stats(num(rows,'imu_age_s')),cpu_temp_c=stats(num(rows,'cpu_temp_c')),wifi_dbm=stats(num(rows,'wifi_dbm')),
        lift_position_raw=stats(num(rows,'lift_position_raw')),lift_error_counts=counts('lift_error'),
        cmd_estop=fraction('cmd_estop'),cmd_stale=fraction('cmd_stale'),
        final_available_xy_m=pos[-1].tolist() if len(pos) else None,available_xy_range_m=[pos.min(axis=0).tolist(),pos.max(axis=0).tolist()] if len(pos) else None,
        motion_bouts=bouts,event_rows=len(events),command_rows=len(commands),
        event_types=dict(Counter(str(x.get('data',{}).get('event',x.get('data',{}).get('type',x.get('event',x.get('kind','unknown'))))) for x in events)),
        summary=json.loads(summ.read_text(encoding='utf-8')) if summ.exists() else None,
        evidence=hashes([session/'metadata.json',*files, *[p for p in [session/'rover/events.jsonl',session/'rover/commands.jsonl',summ] if p.exists()]]))
    return result


if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path, help='New output directory; must not exist')
    args=parser.parse_args()
    OUTPUT=args.output.resolve(); OUTPUT.mkdir(parents=True, exist_ok=False)
    runs=[analyze(s) for s in sorted(CAPTURE.iterdir()) if (s/'metadata.json').exists()]
    (OUTPUT/'run_metrics.json').write_text(json.dumps(dict(method=__doc__,analyzer_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),capture=str(CAPTURE.relative_to(ROOT)),runs=runs),indent=2,allow_nan=False),encoding='utf-8')
    fields=['session','run_id','recorded_created_host_utc','rows','duration_s','moving_s','commanded_s','encoder_mean_body_travel_m','pose_loss_increments']
    with (OUTPUT/'run_summary.csv').open('w',newline='',encoding='utf-8') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader()
        writer.writerows({k:r.get(k) for k in fields} for r in runs)
    for r in sorted(runs,key=lambda x:x.get('encoder_mean_body_travel_m',0),reverse=True):
        if not r.get('rows'): continue
        print(r['session'],r['arguments'], 'duration',round(r['duration_s'],1),'moving',round(r['moving_s'],1),'travel',round(r['encoder_mean_body_travel_m'],2),'valid',r['pose_valid']['covered_fraction'],'display',r['pose_estimate_available']['covered_fraction'],'loss',r['pose_loss_increments'])
