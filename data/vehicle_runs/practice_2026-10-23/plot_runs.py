"""Static measurement plots using the immutable captured CSV, no estimator replay."""
from pathlib import Path
import csv
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from analyze_runs import CAPTURE, HERE, num
import argparse
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('output', type=Path, help='New output directory; must not exist')
args=parser.parse_args()
OUTPUT=args.output.resolve(); OUTPUT.mkdir(parents=True, exist_ok=False)

metrics=json.loads((HERE/'run_metrics.json').read_text())['runs']
moving=sorted([r for r in metrics if r.get('moving_s',0)>1],key=lambda r:r['encoder_mean_body_travel_m'],reverse=True)
plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False,'figure.facecolor':'#f8fafc','axes.facecolor':'#ffffff'})
fig,axes=plt.subplots(1,3,figsize=(15,5),layout='constrained')
labels=[r['run_id'][:8] for r in moving]
y=np.arange(len(moving))
axes[0].barh(y,[r['encoder_motion_body_travel_m'] for r in moving],color='#0d9488')
axes[0].set(yticks=y,yticklabels=labels,xlabel='Nominal mean-wheel travel (m)',title='Recorded wheel motion')
axes[1].barh(y,[100*r['pose_valid']['covered_fraction'] for r in moving],color='#15803d',label='Scientific-valid pose')
axes[1].scatter([100*r['pose_estimate_available']['covered_fraction'] for r in moving],y,color='#f97316',label='Display estimate',zorder=3)
axes[1].set(yticks=y,yticklabels=labels,xlabel='Time coverage (%)',xlim=(0,105),title='Availability is separate from accuracy')
axes[1].legend(loc='best',fontsize=8)
axes[2].barh(y,[r['pose_loss_increments'] for r in moving],color='#dc2626')
axes[2].set(yticks=y,yticklabels=labels,xlabel='Recorded continuity-loss increments',title='Estimator interruptions')
for ax in axes: ax.invert_yaxis();ax.grid(axis='x',alpha=.18)
fig.suptitle('October 23 Rover recordings | separate local run frames; no external position truth',fontsize=14)
fig.savefig(OUTPUT/'run_overview.png',dpi=160)
plt.close(fig)

selected=moving[:2]+[r for r in metrics if r.get('run_id','').startswith('37b43655')]
for r in selected:
    rows=[]
    for p in sorted((CAPTURE/r['session']/'rover').glob('system_*.csv')):
        with p.open(newline='',encoding='utf-8') as stream:rows.extend(csv.DictReader(stream))
    rows=[r for r in rows if r.get('captured_monotonic_s') and None not in r and all(v is not None for v in r.values())]
    rows.sort(key=lambda x:float(x['captured_monotonic_s']))
    t=num(rows,'captured_monotonic_s');t-=t[0]
    fig,axes=plt.subplots(2,2,figsize=(13,8),layout='constrained')
    for k in ('fl','fr','rl','rr'): axes[0,0].plot(t/60,num(rows,'wheel_'+k+'_speed_mps'),lw=.65,label=k.upper(),alpha=.8)
    axes[0,0].set(xlabel='Run elapsed (min)',ylabel='Encoder speed (m/s)',title='Wheel velocity feedback')
    axes[0,0].legend(ncol=4,fontsize=8);axes[0,0].grid(alpha=.2)
    ax=axes[0,1];ax.plot(t/60,num(rows,'battery_v'),color='#2563eb',lw=.9)
    ax.set(xlabel='Run elapsed (min)',ylabel='Pack voltage (V)',title='Measured pack voltage and current')
    ax2=ax.twinx();ax2.plot(t/60,num(rows,'battery_a'),color='#94a3b8',alpha=.5,lw=.65);ax2.set_ylabel('Current (A)')
    ax.grid(alpha=.2)
    ax=axes[1,0]
    for key,color,label in [('pose_estimate_available','#f97316','Display estimate'),('pose_valid','#15803d','Scientific-valid pose')]:
        flag=np.array([x.get(key)=='True' for x in rows],dtype=float)
        ax.plot(t/60,flag,color=color,lw=.8,label=label)
    ax.set(xlabel='Run elapsed (min)',ylabel='Available (1 / 0)',ylim=(-.05,1.05),title='Pose availability in the original runtime')
    ax.legend(fontsize=8,loc='lower right');ax.grid(alpha=.2)
    ax=axes[1,1]
    x=num(rows,'estimate_pos_x_m');yy=num(rows,'estimate_pos_y_m')
    if not np.isfinite(x).any():x=num(rows,'pos_x_m');yy=num(rows,'pos_y_m')
    # NaN gaps intentionally break polylines; never connect different run frames.
    ax.plot(x,yy,color='#f97316',lw=.7,label='Original display estimate')
    good=np.array([x.get('pose_valid')=='True' for x in rows])
    ax.plot(np.where(good,x,np.nan),np.where(good,yy,np.nan),color='#15803d',lw=1,label='Scientific-valid intervals')
    finite=np.isfinite(x)&np.isfinite(yy)
    if finite.any():
        ids=np.flatnonzero(finite);ax.scatter(x[ids[[0,-1]]],yy[ids[[0,-1]]],c=['#111827','#dc2626'],s=35,zorder=5)
    ax.set(xlabel='Local X estimate (m)',ylabel='Local Y estimate (m)',title='Reported path | accuracy unverified')
    ax.set_aspect('equal',adjustable='datalim');ax.legend(fontsize=8);ax.grid(alpha=.2)
    fig.suptitle(f"Run {r['run_id'][:8]} | {r['duration_s']/60:.1f} min | nominal encoder travel {r['encoder_motion_body_travel_m']:.2f} m",fontsize=14)
    fig.savefig(OUTPUT/('run_'+r['run_id'][:8]+'.png'),dpi=160)
    plt.close(fig)
print('Saved overview and',len(selected),'run plots')
