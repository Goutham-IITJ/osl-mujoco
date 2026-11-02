"""Offline replay of saved telemetry through both plants, with unchanged knee C settings."""
import argparse
import csv
import hashlib
import json
import math
from datetime import datetime
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from oslbench.sim.live_knee import LiveKnee


def read_samples(path):
    rows = {}
    for line in path.read_text().splitlines():
        event = json.loads(line)
        if event['kind'] != 'batch':
            continue
        for row in event['payload']['samples']:
            sequence = row['sequence']
            if sequence in rows and rows[sequence] != row:
                raise ValueError('Conflicting duplicate sample')
            rows[sequence] = row
    samples = [rows[k] for k in sorted(rows)]
    if not samples or any(b['sequence'] != a['sequence']+1 for a,b in zip(samples,samples[1:])):
        raise ValueError('Empty recording or missing sample sequence')
    if not all(r['encoder_valid'] for r in samples):
        raise ValueError('This comparison requires all encoder samples valid')
    return samples


def save_csv(path, rows):
    with path.open('w', newline='') as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def metrics(rows, steps):
    rms = lambda values: math.sqrt(sum(x*x for x in values)/len(values))
    tail = [r for r in rows if r['source_time_s'] >= .5]
    out = dict(pairs=len(rows), rms_difference_deg=rms([r['difference_deg'] for r in rows]),
        peak_difference_deg=max(abs(r['difference_deg']) for r in rows),
        rms_target_error_deg=rms([r['sim_tracking_error_deg'] for r in rows]),
        peak_angle_deg=max(r['sim_angle_deg'] for r in rows),
        tail_mean_angle_deg=sum(r['sim_angle_deg'] for r in tail)/len(tail),
        tail_rms_difference_deg=rms([r['difference_deg'] for r in tail]),
        clipped_steps=sum(r['current_clipped'] for r in steps),
        belt_fit_exceeded_steps=sum(r['belt_fit_exceeded'] for r in steps))
    for key in ('current_requested_A','current_applied_A','knee_drive_torque_Nm',
                'belt_actuator_torque_Nm','belt_deflection_rad'):
        out['peak_abs_'+key] = max(abs(r[key]) for r in steps)
    def crossing(threshold):
        for a,b in zip(rows, rows[1:]):
            if a['sim_angle_deg'] < threshold <= b['sim_angle_deg']:
                f=(threshold-a['sim_angle_deg'])/(b['sim_angle_deg']-a['sim_angle_deg'])
                return a['source_time_s']+f*(b['source_time_s']-a['source_time_s'])
    lo,hi=crossing(1.5),crossing(13.5)
    out['rise_10_90_ms']=(hi-lo)*1000 if lo is not None and hi is not None else None
    out['phase_lag']='Not reported: a single step, not a periodic gait cycle'
    return out


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--recording', type=Path, default=ROOT/'build/hardware_step_20261030_134549_560')
    p.add_argument('--outdir', type=Path)
    p.add_argument('--feedback', choices=('actuator-difference','legacy'), default='actuator-difference',
                   help='legacy = knee-error derivative (hardware signal choice); fixed 200 Hz simulation timing remains')
    args=p.parse_args()
    out=args.outdir or ROOT/'build'/('knee_fixture_replay_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    out.mkdir(parents=True, exist_ok=False)
    metadata=json.loads((args.recording/'metadata.json').read_text())
    samples=read_samples(args.recording/'hardware_batches.jsonl')
    results, curves={},{}
    for plant in ('full-leg','knee-fixture'):
        steps=[]
        prepared=LiveKnee.prepare(plant=plant, feedback=args.feedback)
        runtime=LiveKnee(metadata['hardware'],samples[0],steps.append,prepared=prepared)
        pairs=[runtime.accept(row) for row in samples]
        folder=out/plant
        folder.mkdir()
        save_csv(folder/'comparison.csv',pairs)
        save_csv(folder/'simulation.csv',steps)
        results[plant]=metrics(pairs,steps)
        (folder/'metadata.json').write_text(json.dumps(dict(
            model=runtime.ctx['ident'], settings=vars(runtime.args),
            source_metadata=metadata, source_sha256=hashlib.sha256((args.recording/'hardware_batches.jsonl').read_bytes()).hexdigest(),
            replay='Original source timestamps; same LiveKnee stepping/initialization; no wall-clock pacing',
            limitations='Fixture mass/friction and torque constant provisional; fixed 200 Hz differs from recorded variable hardware dt; actuator-difference uses C, legacy uses knee-error D'
        ),indent=2,default=str)+'\n')
        curves[plant]=pairs
    with (args.recording/'comparison.csv').open() as file:
        old=list(csv.DictReader(file))
    new=curves['full-leg']
    if len(old)!=len(new):
        raise ValueError('Replay count differs from original recording')
    results['historical_full_leg_max_angle_difference_deg']=max(abs(float(a['sim_angle_deg'])-b['sim_angle_deg']) for a,b in zip(old,new))
    results['feedback']=args.feedback
    results['historical_reproduction_expected']=args.feedback=='actuator-difference'
    (out/'summary.json').write_text(json.dumps(results,indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,1,figsize=(10,7),sharex=True)
    time=[r['source_time_s'] for r in new]
    axes[0].plot(time,[r['hardware_angle_deg'] for r in new],label='Recorded hardware',color='tab:orange')
    axes[0].plot(time,[r['target_deg'] for r in new],'--',label='Recorded target',color='gray')
    for name,rows in curves.items():
        color='tab:blue' if name=='full-leg' else 'tab:green'
        axes[0].plot(time,[r['sim_angle_deg'] for r in rows],label=name+' simulation',color=color)
        axes[1].plot(time,[r['difference_deg'] for r in rows],label=name,color=color)
    axes[0].set_ylabel('Knee angle (degrees)')
    axes[1].set_ylabel('Simulation - hardware (degrees)')
    axes[1].set_xlabel('Original source time (s)')
    for ax in axes:
        ax.grid(alpha=.3)
        ax.legend()
    fig.suptitle('Recorded 15-degree step: knee gains, '+args.feedback+' feedback')
    fig.tight_layout()
    fig.savefig(out/'comparison.png',dpi=160)
    plt.close(fig)
    print(json.dumps(results,indent=2))
    print('Saved:',out)


if __name__=='__main__':
    main()
