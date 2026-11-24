"""Validate knee 5/10/15 degree recordings with fixed fixture and confirmed gains."""
import argparse, csv, hashlib, io, json, math, sys, zipfile
from pathlib import Path
from datetime import datetime
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from replay_knee_fixture import metrics,save_csv
from oslbench.sim.live_knee import LiveKnee
from oslbench.control.current import CurrentPID,PIDGains,CurrentLimits


def crossing(rows,key,value):
    for a,b in zip(rows,rows[1:]):
        if a[key]<value<=b[key]:
            return a['source_time_s']+(value-a[key])/(b[key]-a[key])*(b['source_time_s']-a['source_time_s'])


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--archive',type=Path,default=Path.home()/'Downloads/All_Step_Results.zip')
    p.add_argument('--outdir',type=Path)
    p.add_argument('--joint',choices=('knee','ankle'),default='knee')
    a=p.parse_args()
    out=a.outdir or ROOT/'build'/(a.joint+'_multistep_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    out.mkdir(parents=True,exist_ok=False)
    summary=[];curves={}
    with zipfile.ZipFile(a.archive) as z:
        for angle in (5,10,15):
            member=(f'All_Step_Results/step_test_knee_{angle}_single_run/step_response_data.csv' if a.joint=='knee'
                    else f'All_Step_Results/step_ankle_{angle}_single_run/step_response_data.csv')
            raw=z.read(member)
            hardware=list(csv.DictReader(io.StringIO(raw.decode('utf-8-sig'))))
            folder=out/f'{a.joint}_{angle}deg';folder.mkdir()
            (folder/'original_hardware.csv').write_bytes(raw)
            samples=[];elapsed=0.;errors=[]
            gains=(1.5,.15,.06) if a.joint=='knee' else (2.,0.,.075)
            current_sign=1 if a.joint=='knee' else -1
            pid=CurrentPID(PIDGains(*gains),CurrentLimits(sim_clip_A=60),integral_limit_deg_s=50)
            for k,r in enumerate(hardware):
                dt=float(r['dt_s']);target=float(r['reference_deg']);actual=float(r['actual_deg'])
                if not dt>0 or target!=angle or not math.isfinite(actual):
                    raise ValueError('Invalid interval, target or angle')
                elapsed+=dt
                requested=pid.update(target,actual,dt)
                errors.append(max(abs(pid.last[x]-current_sign*float(r[y])) for x,y in [('p_A','p_term_A'),('i_A','i_term_A'),('d_A','d_term_A')]))
                samples.append(dict(sequence=k,time_s=elapsed,dt_s=dt,reference_deg=target,actual_deg=actual,
                    encoder_valid=True,requested_current_A=current_sign*sum(float(r[t]) for t in ('p_term_A','i_term_A','d_term_A')),
                    commanded_current_A=current_sign*float(r['current_cmd_A'])))
            for feedback in ('legacy','actuator-difference'):
                if max(errors)>1e-8:
                    raise ValueError('Recorded PID terms do not match the selected gains/sign convention')
                steps=[]
                if a.joint=='knee':
                    prepared=LiveKnee.prepare(plant='knee-fixture',feedback=feedback)
                    runtime=LiveKnee
                else:
                    from oslbench.sim.ankle_fixture import AnkleFixtureReplay
                    prepared=AnkleFixtureReplay.prepare(feedback=feedback)
                    runtime=AnkleFixtureReplay
                sim=runtime({'flexion_positive_multiplier':1},samples[0],steps.append,prepared=prepared)
                pairs=[sim.accept(s) for s in samples]
                dest=folder/feedback;dest.mkdir()
                save_csv(dest/'comparison.csv',pairs);save_csv(dest/'simulation.csv',steps)
                result=dict(joint=a.joint,angle_deg=angle,feedback=feedback,**metrics(pairs,steps,angle,a.joint))
                lo,hi=(crossing(pairs,'hardware_angle_deg',v*angle) for v in (.1,.9))
                result.update(hardware_rise_10_90_ms=(hi-lo)*1000 if hi is not None and lo is not None else None,
                    hardware_peak_angle_deg=max(r['hardware_angle_deg'] for r in pairs),
                    pid_term_max_difference_A=max(errors),
                    hardware_clipped_rows=sum(r['current_clamped']=='True' for r in hardware))
                summary.append(result);curves[angle,feedback]=pairs
                (dest/'metadata.json').write_text(json.dumps(dict(source_member=member,source_csv_sha256=hashlib.sha256(raw).hexdigest(),
                    model=sim.ctx.get('ident',{'path':sim.bench.path,'sha256':hashlib.sha256(Path(sim.bench.path).read_bytes()).hexdigest()}),settings=vars(prepared[0]),
                    current_normalization=f'Original current command multiplied by {current_sign} for positive model-coordinate current; original CSV preserved',
                    time='Reconstructed measurement timeline from cumulative actual dt; CSV time_s is loop-start clock',
                    validity='CSV has no encoder-validity flags; finite values assumed usable; row order provides synthetic sequence',
                    initialization='First measured angle only, zero velocity/belt deflection; no pre-step baseline recorded',
                    interpretation='Fixed model, no fitting; fixed 200 Hz simulation differs from hardware variable intervals'),indent=2,default=str)+'\n')
    (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');save_csv(out/'summary.csv',summary)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,3,figsize=(14,7),sharex=True)
    for col,angle in enumerate((5,10,15)):
        rows=curves[angle,'legacy'];t=[r['source_time_s'] for r in rows]
        axes[0,col].plot(t,[r['hardware_angle_deg'] for r in rows],color='tab:orange',label='Hardware')
        axes[0,col].axhline(angle,color='gray',ls='--',label='Target')
        for fb,label,color in [('legacy',a.joint.capitalize()+'-error D','tab:blue'),('actuator-difference','Configuration C','tab:green')]:
            r=curves[angle,fb]
            axes[0,col].plot(t,[v['sim_angle_deg'] for v in r],label=label,color=color)
            axes[1,col].plot(t,[v['difference_deg'] for v in r],label=label,color=color)
        axes[0,col].set_title(f'{angle}° step');axes[1,col].set_xlabel('Reconstructed source time (s)')
        for ax in axes[:,col]:ax.grid(alpha=.25)
    axes[0,0].set_ylabel(a.joint.capitalize()+' angle (degrees)');axes[1,0].set_ylabel('Simulation − hardware (degrees)')
    axes[0,0].legend(fontsize=8);fig.tight_layout();fig.savefig(out/'comparison.png',dpi=170);plt.close(fig)
    print(json.dumps(summary,indent=2));print('Saved:',out)


if __name__=='__main__':main()
