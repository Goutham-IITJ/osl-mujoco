"""Replay saved tracking CSVs or compare both live joints with an assembled leg."""
import argparse,csv,io,json,math,os,time,zipfile
from pathlib import Path
from datetime import datetime
from urllib.request import Request,build_opener,ProxyHandler
import numpy as np
from oslbench.sim.assembled_tracking import AssembledTracking,JOINTS
from oslbench.presentation.assembled_tracking import TrackingPresentation,dashboard

ROOT=Path(__file__).resolve().parents[2]

def archive_ticks(archive,run):
    with zipfile.ZipFile(archive) as z:
        prefix='Tracking Results & Codes & Dataset/Results/'+run+'/run_1/'
        rows={j:list(csv.DictReader(io.StringIO(z.read(prefix+j+'_trajectory_data.csv').decode('utf-8-sig')))) for j in JOINTS}
    if len(rows['knee'])!=len(rows['ankle']):raise ValueError('Joint row counts differ')
    elapsed={j:0. for j in JOINTS}
    for k in range(len(rows['knee'])):
        t=float(rows['knee'][k]['time_s']);tick=dict(control_tick=k,reference_time_s=t,joints={})
        if abs(t-float(rows['ankle'][k]['time_s']))>1e-8:raise ValueError('Reference clocks differ')
        for j in JOINTS:
            r=rows[j][k];dt=float(r['dt_s']);elapsed[j]+=dt
            tick['joints'][j]=dict(time_s=elapsed[j],reference_deg=float(r['reference_deg']),actual_deg=float(r['actual_deg']),encoder_valid=True)
        yield tick

def live_ticks(args,raw):
    from hardware.gait_tracking.telemetry import SCHEMA,validate_sample
    opener=build_opener(ProxyHandler({}));token=os.environ.get('OSL_TELEMETRY_TOKEN')
    headers={'Authorization':'Bearer '+token} if token else {}
    def get(path):
        with opener.open(Request(f'http://{args.host}:{args.port}'+path,headers=headers),timeout=1) as r:return json.load(r)
    meta=None;deadline=time.monotonic()+60
    while time.monotonic()<deadline:
        try:meta=get('/v1/metadata');break
        except Exception:time.sleep(.2)
    if meta is None or meta.get('schema')!=SCHEMA:raise ValueError('No compatible two-joint server')
    raw.write(json.dumps(dict(kind='metadata',payload=meta))+'\n');raw.flush()
    seq=-1;local={j:-1 for j in JOINTS};pending={};start=time.monotonic()
    while time.monotonic()-start<args.seconds:
        b=get(f'/v1/samples?after={seq}&limit=1000')
        if b.get('session_id')!=meta['session_id']:raise ValueError('Hardware session changed')
        raw.write(json.dumps(dict(kind='batch',receipt_elapsed_s=time.monotonic()-start,payload=b))+'\n');raw.flush()
        for r in b['samples']:
            validate_sample(r);j=r['joint']
            if r['sequence']!=seq+1 or r['joint_sequence']!=local[j]+1:raise ValueError('Sample gap or duplicate; stop comparison')
            seq=r['sequence'];local[j]=r['joint_sequence'];k=r['control_tick']
            signs=meta['joints'][j];tick=pending.setdefault(k,dict(control_tick=k,reference_time_s=r['reference_time_s'],joints={}))
            if abs(tick['reference_time_s']-r['reference_time_s'])>1e-8:raise ValueError('Tick reference clocks differ')
            tick['joints'][j]=dict(time_s=r['time_s'],reference_deg=signs['angle_multiplier']*r['reference_deg'],
                actual_deg=signs['angle_multiplier']*r['actual_deg'],encoder_valid=r['encoder_valid'])
            if len(tick['joints'])==2:
                yield pending.pop(k)
            if len(pending)>4000:raise ValueError('Unpaired joint backlog exceeded')
        current=get('/v1/metadata')
        if current.get('producer_complete') and seq==current.get('final_sequence'):
            if pending:raise ValueError('Unpaired final joint tick')
            return
        time.sleep(.05)
    raise ValueError('Duration expired before producer completion')

def summarize(pairs,steps):
    result={}
    for j in JOINTS:
        p=[r for r in pairs if r['joint']==j and r['difference_deg'] is not None]
        s=[r for r in steps if r['joint']==j]
        if not p:result[j]={'pairs':0};continue
        rms=lambda x:math.sqrt(sum(v*v for v in x)/len(x))
        result[j]=dict(pairs=len(p),rms_difference_deg=rms([r['difference_deg'] for r in p]),
            peak_difference_deg=max(abs(r['difference_deg']) for r in p),
            hardware_target_rms_deg=rms([r['hardware_tracking_error_deg'] for r in p]),
            simulation_target_rms_deg=rms([r['sim_tracking_error_deg'] for r in p]),
            belt_fit_exceeded_steps=sum(r['belt_fit_exceeded'] for r in s),clipped_steps=sum(r['current_clipped'] for r in s))
        for key in ('joint_drive_torque_Nm','belt_actuator_torque_Nm','current_requested_A','current_applied_A','belt_deflection_rad'):
            result[j]['peak_abs_'+key]=max((abs(r[key]) for r in s),default=0.)
        t=np.array([r['source_time_s'] for r in p]);hw=np.array([r['hardware_angle_deg'] for r in p]);sim=np.array([r['sim_angle_deg'] for r in p])
        use=(t>=t[0]+.2)&(t<=t[-1]-.2)
        if sum(use)>20:
            lags=np.arange(-.2,.2001,.005);errors=[np.mean((np.interp(t[use]+lag,t,sim)-hw[use])**2) for lag in lags]
            idx=int(np.argmin(errors));result[j].update(best_fit_lag_ms=float(lags[idx]*1000),lag_fit_rms_deg=float(np.sqrt(errors[idx])),
                lag_note='Positive = simulation later than hardware; ±200 ms search, fixed 200 ms trimmed window; exploratory for nonstationary data')
    return result

def save_csv(path,rows):
    if not rows:return
    with path.open('x',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)

def plot(out,pairs):
    if not pairs:return
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,2,figsize=(13,7))
    for col,j in enumerate(JOINTS):
        rows=[r for r in pairs if r['joint']==j];t=[r['source_time_s'] for r in rows]
        for key,label,color in [('hardware_angle_deg','Hardware','#e58b22'),('sim_angle_deg','Simulation','#168b70'),('target_deg','Target','#777777')]:
            axes[0,col].plot(t,[r[key] for r in rows],label=label,color=color,lw=1.2)
        axes[1,col].plot(t,[r['difference_deg'] for r in rows]);axes[0,col].set_title(j.capitalize());axes[0,col].legend()
        axes[0,col].set_ylabel('Angle (degrees)');axes[1,col].set_ylabel('Simulation − hardware (degrees)');axes[1,col].set_xlabel('Source time (s)')
        for ax in axes[:,col]:ax.grid(alpha=.25)
    fig.tight_layout();fig.savefig(out/'comparison.png',dpi=170);plt.close(fig)

def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--live',action='store_true');p.add_argument('--host',default='127.0.0.1');p.add_argument('--port',type=int,default=8794)
    p.add_argument('--archive',type=Path,default=Path.home()/'Downloads/Tracking Results & Codes & Dataset.zip')
    p.add_argument('--run',default='knee_ankle_normal_walk_0.6ms');p.add_argument('--all',action='store_true')
    p.add_argument('--feedback',choices=('legacy','actuator-difference'),default='legacy')
    p.add_argument('--ankle-range',choices=('cad','hardware'),default='cad',help='hardware explicitly expands runtime ankle range to ±31°; unverified coordinate/travel assumption')
    p.add_argument('--outdir',type=Path);p.add_argument('--seconds',type=float,default=120)
    p.add_argument('--no-dashboard',action='store_true');p.add_argument('--no-browser',action='store_true');p.add_argument('--dashboard-port',type=int,default=8798)
    p.add_argument('--view',action='store_true');p.add_argument('--speed',type=float,default=1.)
    p.add_argument('--physics-dt',type=float,default=.0005,help='Explicit numerical diagnostic; authored baseline is 0.0005 s')
    args=p.parse_args(argv)
    if args.speed<=0:p.error('speed must be positive')
    if args.all:
        if args.live:p.error('--all is for offline replay')
        root=args.outdir or ROOT/'build'/('assembled_matrix_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'));root.mkdir(parents=True,exist_ok=False)
        with zipfile.ZipFile(args.archive) as z:runs=sorted({n.split('/Results/')[1].split('/')[0] for n in z.namelist() if '/Results/' in n and n.endswith('knee_trajectory_data.csv')})
        for run in runs:
            main(['--archive',str(args.archive),'--run',run,'--feedback',args.feedback,'--ankle-range',args.ankle_range,'--physics-dt',str(args.physics_dt),'--no-dashboard','--outdir',str(root/run)])
        print('Matrix saved:',root);return 0
    out=args.outdir or ROOT/'build'/('assembled_tracking_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'));out.mkdir(parents=True,exist_ok=False)
    presentation=TrackingPresentation('physical live' if args.live else 'recorded physical: '+args.run,out)
    pairs=[];steps=[];sim=None;viewer=None;dash=None;status='COMPLETE';started=time.perf_counter();last_draw=started
    def step(row):steps.append(row);presentation.step(row)
    if not args.no_dashboard:
        dash=dashboard(presentation,args.dashboard_port,not args.no_browser);print('Dashboard:',dash.url,flush=True)
    raw=(out/'hardware_batches.jsonl').open('x')
    try:
        stream=live_ticks(args,raw) if args.live else archive_ticks(args.archive,args.run)
        for tick in stream:
            if sim is None:
                sim=AssembledTracking(tick,args.feedback,args.ankle_range,step,args.physics_dt)
                presentation.assumptions=json.dumps(sim.metadata)
                if args.view:
                    import mujoco.viewer
                    viewer=mujoco.viewer.launch_passive(sim.model,sim.data)
                started=time.perf_counter();origin=tick['reference_time_s']
            for row in sim.accept(tick):pairs.append(row);presentation.add(row)
            if dash or viewer:
                if not args.live:
                    wait=(tick['reference_time_s']-origin)/args.speed-(time.perf_counter()-started)
                    if wait>0:time.sleep(min(wait,.05))
                now=time.perf_counter()
                if now-last_draw>=.1:
                    presentation.status='RUNNING'+(' | OUTSIDE BELT FIT' if sim.fit_exceeded else '')
                    if dash:dash.update(presentation.snapshot());dash.pump()
                    if viewer:viewer.sync()
                    last_draw=now
                if presentation.stop_requested:status='STOPPED_BY_DASHBOARD';break
    except KeyboardInterrupt:status='STOPPED_BY_USER'
    except Exception as exc:status='STOPPED: '+str(exc)
    finally:
        raw.close()
        if viewer:viewer.close()
        save_csv(out/'comparison.csv',pairs);save_csv(out/'simulation.csv',steps)
        summary=dict(status=status,feedback=args.feedback,ankle_range=args.ankle_range,joints=summarize(pairs,steps))
        if sim:
            summary['within_belt_fit']=sim.fit_exceeded==0
            (out/'metadata.json').write_text(json.dumps(sim.metadata,indent=2)+'\n')
        (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');plot(out,pairs)
        presentation.status=status
        if dash:
            dash.update(presentation.snapshot());print('Saved. Dashboard retained; Ctrl+C closes it.',flush=True)
            try:
                while not presentation.stop_requested:dash.pump();time.sleep(.1)
            except KeyboardInterrupt:pass
            dash.close()
        print(json.dumps(summary,indent=2));print('Saved:',out)
    return 0 if status=='COMPLETE' else 1

if __name__=='__main__':raise SystemExit(main())
