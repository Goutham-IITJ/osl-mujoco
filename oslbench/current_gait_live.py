"""Presentation and lifecycle for the production current-gait iterator.

No control or integration equations. The viewer, dashboard and saved rows consume
the same iterator; no preflight simulation or replay is performed.
"""
from pathlib import Path
import json
import time
import hashlib
from datetime import datetime
import numpy as np
from . import viewer as vw
from . import drivetrain_dashboard as dd


class LiveState:
    def __init__(self, paused=False):
        self.status='paused' if paused else 'active'
        self.restart=False

    def key(self, key):
        key=str(key).lower()
        if key=='space' and self.status in ('paused','active'):
            self.status='active' if self.status=='paused' else 'paused'
        elif key=='r' and self.status!='stopped':
            self.restart=True
        elif key=='escape':
            self.status='stopped'


def snapshot(rows, ctx, status, args):
    from experiments import run_current_controlled_gait as gait
    if not rows:
        q=float(ctx['bench'].data.qpos[ctx['bench'].knee_qpos])
        initial={k:0. for k in dd.SCALARS}
        initial.update(ref=float(np.degrees(ctx['res']['ref_rad'][0])),act=float(np.degrees(q)),
            th_a=ctx['sim'].layer.theta_a,err=float(np.degrees(ctx['res']['ref_rad'][0]-q)),
            k_s=ctx['sim'].layer.belt(q).K_s,
            status=f'{status.upper()} | t=0 s | {args.feedback_mode} | Kp={args.kp:g}, '
                   f'Kd={args.kd:g}, Ki={args.ki:g} | dt={ctx["dt"]*1000:g} ms | control=1000 Hz',
            foot='Initial state; no physics steps yet. Space start/pause | R restart | Esc save and exit')
        return initial
    a=np.asarray(rows);c={n:i for i,n in enumerate(gait.CSV_HEADER)}
    f=lambda n:a[:,c[n]]
    last=lambda n:float(f(n)[-1])
    # Decimate once on a shared index set, always including the final point.
    ix=np.unique(np.r_[np.arange(0,len(rows),max(1,len(rows)//250)),len(rows)-1])
    series=lambda n:f(n)[ix]
    return dict(phase=last('gait_phase_pct'),ref=last('reference_deg'),act=last('knee_angle_deg'),
        err=last('tracking_error_deg'),tau_j=last('belt_torque_joint_side_Nm'),
        i_q=last('current_actual_modelled_A'),th_a=last('theta_a_rad'),th_ad=last('theta_a_dot_rad_s'),
        th_s=last('theta_s_rad'),tau_a=last('belt_torque_actuator_side_Nm'),k_s=last('K_s_Nm_per_rad'),
        x=series('gait_phase_pct'),y_act=series('knee_angle_deg'),y_thj=series('knee_angle_deg'),
        y_thar=np.degrees(series('theta_a_rad')/ctx['audit'].belt_ratio),
        y_ths=series('theta_s_rad'),y_tau=series('belt_torque_joint_side_Nm'),
        y_iq=series('current_actual_modelled_A'),y_err=series('tracking_error_deg'),
        y_request=series('current_request_A'),y_applied=series('current_applied_A'),
        y_actual=series('current_actual_modelled_A'),y_em=series('estimated_actuator_output_torque_Nm'),
        y_belt_actuator=series('belt_torque_actuator_side_Nm'),
        status=f'{status.upper()} | t={len(rows)*ctx["dt"]:.4f} s | {args.feedback_mode} | '
               f'Kp={args.kp:g}, Kd={args.kd:g}, Ki={args.ki:g} | dt={ctx["dt"]*1000:g} ms | control=1000 Hz',
        foot='Space pause/resume | R archive this attempt and restart | Esc save and exit\n'
             'SIMULATION ONLY. Modelled current, not measured. Knee servo disconnected. '
             'Electromagnetic output torque differs from compliant belt-transmitted torque.\n'
             'Deflection +/-0.055 rad is an approximate fit range, not a hard limit. '
             'Angle is post-step; reference/belt signals are pre-step, as in saved legacy CSVs. '
             'Displayed knee/shaft gap uses post-step positions; belt deflection uses pre-step positions.')


def save_attempt(root, attempt, rows, ctx, args, status, dash):
    from experiments import run_current_controlled_gait as gait
    path=Path(root)/f'attempt_{attempt:03d}'
    path.mkdir()  # Refuse any overwrite, including interrupted attempts.
    md=gait.metadata(ctx,args,'ideal' if ctx['cfg'].current_loop_tau_s==0 else 'first_order')
    md.update(live=True,run_status=status,recorded_physics_steps=len(rows),
        actual_control_updates=ctx['src'].updates,recorded_horizon_s=len(rows)*ctx['dt'],
        renderer='mujoco.viewer.launch_passive, same model/data as iter_run',
        derivative_feedback=('actuator output angle backward difference, referred through external ratio once'
          if args.feedback_mode=='actuator-difference' else args.derivative_on),
        source_sha256={str(p.relative_to(Path(gait.ROOT))):hashlib.sha256(p.read_bytes()).hexdigest()
          for folder in ('oslbench','experiments') for p in (Path(gait.ROOT)/folder).glob('*.py')})
    met=gait.metrics(rows,ctx,args) if rows else {'recorded_physics_steps':0}
    gait.write_outputs(str(path),rows,met,md,False)
    if rows:
        # Full precision companion for parity; conventional CSV remains unchanged.
        import csv
        with (path/'gait_trace_full_precision.csv').open('x',newline='') as f:
            w=csv.writer(f);w.writerow(gait.CSV_HEADER);w.writerows(rows)
        plots=gait.plots(str(path),rows,met,md,ctx,False)
        if len(plots)!=5:
            raise RuntimeError('Saved data but required plots could not be generated: '+str(path))
    if hasattr(dash,'_body'):
        (path/'dashboard_snapshot.json').write_bytes(dash._body)
        # Offline final dashboard: replace polling with the very same saved snapshot.
        html=dash.html.decode('utf-8').replace('await (await fetch("/data", {cache:"no-store"})).json()',
            json.dumps(json.loads(dash._body)))
        (path/'dashboard.html').write_text(html,encoding='utf-8')
    print('Saved',status,path,flush=True)
    return path


def run_live(ctx,args):
    import mujoco
    import mujoco.viewer
    from experiments import run_current_controlled_gait as gait
    if ctx['sim'].servo_connected:
        raise RuntimeError('Current gait requires disconnected knee servo')
    root=Path(args.outdir) if args.outdir else Path(gait.OUTROOT)/('live_'+args.feedback_mode+'_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    root.mkdir(parents=True,exist_ok=False)
    phase=ctx['res']['gait_phase_percent'];ref=np.degrees(ctx['res']['ref_rad'])
    limits=dict(ang=(-10,125),ths=(-.07,.07),gap=(-5,5),tau=(-160,160),iq=(-60,60),fit_edge=.055,stance_end=60)
    dash=dd.make_drivetrain_dashboard(dict(subject='AB19',trial='one right knee gait cycle',current_controlled=True),
        (phase[::8],ref[::8]),limits,force=args.dashboard,port=args.port,open_browser=not args.no_browser)
    state=LiveState(args.start_paused);dash.bind_keys(state.key)
    def key(code):
        state.key({32:'space',82:'r',114:'r',256:'escape'}.get(code,''))
    cam=vw.derive_camera(ctx['bench'],ctx['res'],azimuth=args.azimuth,elevation=args.elevation,distance=args.distance)
    rows=[];attempt=1;saved=False;stream=gait.iter_run(ctx,args)
    # Reset visibly before the generator's first step; no integration occurs here.
    ctx['src'].reset();ctx['sim'].reset(float(ctx['res']['ref_rad'][0]) if args.init=='aligned' else 0.)
    spf=max(1,int(round(args.speed/vw.FRAME_HZ/ctx['dt'])))
    print('Live output:',root,'Dashboard:',getattr(dash,'url','none'),flush=True)
    try:
        with mujoco.viewer.launch_passive(ctx['bench'].model,ctx['bench'].data,key_callback=key,
                                         show_left_ui=False,show_right_ui=False) as viewer:
            with viewer.lock():vw.apply_camera(viewer.cam,cam,args.zoom,args.elevation)
            while viewer.is_running() and state.status!='stopped':
                started=time.perf_counter();dash.pump()
                if state.restart:
                    if not saved:
                        dash.update(snapshot(rows,ctx,'restarted',args))
                        saved=True
                        save_attempt(root,attempt,rows,ctx,args,'restarted',dash)
                    attempt+=1;rows=[];saved=False;stream=gait.iter_run(ctx,args)
                    ctx['src'].reset();ctx['sim'].reset(float(ctx['res']['ref_rad'][0]) if args.init=='aligned' else 0.)
                    dash.clear_traces();state.restart=False;state.status='active'
                if state.status=='active':
                    for _ in range(spf):
                        rows.append(next(stream))
                        if len(rows)==ctx['n']:
                            state.status='complete';break
                viewer.sync()
                dash.update(snapshot(rows,ctx,state.status,args))
                if state.status=='complete' and not saved:
                    saved=True
                    save_attempt(root,attempt,rows,ctx,args,'complete',dash)
                    if args.exit_on_complete:break
                budget=spf*ctx['dt']/args.speed if state.status=='active' else 1/vw.FRAME_HZ
                time.sleep(max(0.,budget-(time.perf_counter()-started)))
    except KeyboardInterrupt:
        state.status='stopped'
    finally:
        try:
            if not saved:
                state.status='stopped';dash.update(snapshot(rows,ctx,state.status,args))
                save_attempt(root,attempt,rows,ctx,args,'stopped',dash)
        finally:
            dash.close()
