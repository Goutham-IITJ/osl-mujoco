"""One hardware stream, independent MuJoCo, comparison and simulation dashboards."""
import argparse
from collections import deque
import csv
import json
import math
import os
from pathlib import Path
import queue
import threading
import time

from oslbench.apps.knee_telemetry import connect_receiver
from oslbench.sim.live_knee import LiveKnee
from oslbench.presentation.live_knee import Presentation, dashboard


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host',default='127.0.0.1');parser.add_argument('--port',type=int,default=8794)
    parser.add_argument('--outdir',type=Path,required=True);parser.add_argument('--seconds',type=float,default=60)
    parser.add_argument('--comparison-port',type=int,default=8796);parser.add_argument('--telemetry-port',type=int,default=8797)
    parser.add_argument('--buffer-ms',type=float,default=100);parser.add_argument('--no-browser',action='store_true')
    parser.add_argument('--view',action='store_true',help='Also show the independently simulated MuJoCo leg')
    args=parser.parse_args(argv)
    if not math.isfinite(args.seconds) or args.seconds<=0 or not math.isfinite(args.buffer_ms) or not 0<=args.buffer_ms<=2000:
        parser.error('seconds must be positive; buffer-ms must be between 0 and 2000')
    if args.outdir.exists(): raise FileExistsError(args.outdir)
    print('Waiting for hardware server (30 seconds)...',flush=True)
    receiver=connect_receiver(args.host,args.port,os.environ.get('OSL_TELEMETRY_TOKEN'))
    args.outdir.mkdir(parents=True)
    presentation=Presentation(receiver.metadata)
    presentation.output_dir=str(args.outdir.resolve())
    metadata=dict(hardware=receiver.metadata,physics_dt_s=.0005,controller='knee',feedback='C',control_hz=200,
        torque_constant_Nm_A=.5701,torque_constant_verified=False,buffer_ms=args.buffer_ms,
        reference='live target only; zero-order hold, no AB19 replay',
        initialization='first valid encoder angle only; zero velocity and zero belt deflection',
        alignment='fixed grid from first source time; linear angle sampling within bracketing states; target applied at next grid boundary (<=0.5 ms)',
        gaps='stop on missing sequence after first accepted sample or source interval >100 ms',
        interpretation='combined controller and plant discrepancy; C differs from supplied hardware knee derivative')
    (args.outdir/'metadata.json').write_text(json.dumps(metadata,indent=2),encoding='utf-8')
    inbox=queue.Queue(maxsize=64);stop=threading.Event();overflow=threading.Event()
    def worker():
        while not stop.is_set():
            try: item=('batch',receiver.poll(),time.monotonic())
            except ValueError as exc: item=('fatal',str(exc),time.monotonic())
            except Exception as exc: item=('disconnect',type(exc).__name__,time.monotonic())
            try: inbox.put_nowait(item)
            except queue.Full: overflow.set();return
            if item[0]=='fatal': return
            stop.wait(.05)
    thread=threading.Thread(target=worker,daemon=True);thread.start()
    dashboards=[];viewer=None;runtime=None;pending=deque();status='WAITING';failed=False
    start=time.monotonic();last_publish=0.;last_view=0.;last_sample_receipt=None;latest_source=None
    step_writer=None;pair_writer=None
    with (args.outdir/'hardware_batches.jsonl').open('x',encoding='utf-8') as raw, \
         (args.outdir/'simulation.csv').open('x',newline='') as steps, \
         (args.outdir/'comparison.csv').open('x',newline='') as pairs:
        def write_step(row):
            nonlocal step_writer
            if step_writer is None:
                step_writer=csv.DictWriter(steps,fieldnames=list(row));step_writer.writeheader()
            step_writer.writerow(row)
            if row['physics_step']%20==0: presentation.telemetry.append(dict(row))
        try:
            dashboards.append(dashboard(presentation,args.comparison_port,'comparison',args.telemetry_port,not args.no_browser))
            dashboards.append(dashboard(presentation,args.telemetry_port,'telemetry',args.comparison_port,not args.no_browser))
            print('Comparison:',dashboards[0].url,'Simulation telemetry:',dashboards[1].url,flush=True)
            while time.monotonic()-start<args.seconds:
                for dash in dashboards: dash.pump()
                if presentation.stop_requested:
                    status='STOPPED by dashboard; files saved'
                    break
                now=time.monotonic()
                if overflow.is_set(): raise ValueError('Receiver queue overflow; recording incomplete')
                # Preserve full received batches before integrating any samples.
                for _ in range(64):
                    try: kind,payload,receipt=inbox.get_nowait()
                    except queue.Empty: break
                    raw.write(json.dumps(dict(kind=kind,payload=payload,receipt_elapsed_s=receipt-start))+'\n')
                    if kind=='fatal': raise ValueError(payload)
                    if kind=='disconnect': status='DISCONNECTED — simulation held';continue
                    if payload['samples']:
                        last_sample_receipt=receipt
                        latest_source=payload['samples'][-1]['time_s']
                        for row in payload['samples']: pending.append((receipt+args.buffer_ms/1000,row))
                        if len(pending)>4000:
                            raise ValueError('Simulation backlog exceeds 4000 samples; stop rather than grow without bound')
                    age=payload['producer_age_s']
                    status='LIVE' if age is not None and age<.5 else 'STALE — simulation held'
                for _ in range(20):
                    if not pending or pending[0][0]>time.monotonic(): break
                    _,row=pending.popleft()
                    if runtime is None:
                        runtime=LiveKnee(receiver.metadata,row,write_step)
                        metadata.update(model=runtime.ctx['ident'],initial_sequence=row['sequence'],
                                        omitted_initial_samples=row['sequence'])
                        (args.outdir/'metadata.json').write_text(json.dumps(metadata,indent=2),encoding='utf-8')
                        if args.view:
                            import mujoco.viewer
                            viewer=mujoco.viewer.launch_passive(runtime.bench.model,runtime.bench.data)
                    result=runtime.accept(row)
                    if pair_writer is None:
                        pair_writer=csv.DictWriter(pairs,fieldnames=list(result));pair_writer.writeheader()
                    pair_writer.writerow(result);presentation.add(result)
                    if row['status']=='abort_pending': status='HARDWARE ABORT REPORTED'
                if last_sample_receipt is not None and now-last_sample_receipt>.5 and not pending:
                    status='STALE / DISCONNECTED — simulation held'
                lag=max(0.,latest_source-runtime.previous['time_s']) if runtime and runtime.previous and latest_source else 0.
                display_status='CATCHING UP' if lag>max(.5,args.buffer_ms/1000+.2) else status
                presentation.status=display_status+f' | buffered samples: {len(pending)} | source lag: {lag:.3f} s'
                if now-last_publish>=.2:
                    snap=presentation.snapshot()
                    for dash in dashboards: dash.update(snap)
                    raw.flush();steps.flush();pairs.flush();last_publish=now
                if viewer and now-last_view>=1/30:
                    if not viewer.is_running(): break
                    viewer.sync();last_view=now
                time.sleep(.001)
        except KeyboardInterrupt: status='STOPPED by user'
        except Exception as exc:
            status='STOPPED: '+str(exc);failed=True;print(status,flush=True)
        finally:
            stop.set();thread.join(timeout=2)
            presentation.status=status
            summary=presentation.snapshot();summary['unprocessed_samples']=len(pending)
            summary['physics_steps']=runtime.k if runtime else 0
            summary['unprocessed_batches']=inbox.qsize()
            summary['run_ended']=True
            summary['source_lag_s']=max(0.,latest_source-runtime.previous['time_s']) if runtime and runtime.previous and latest_source else None
            (args.outdir/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
            for dash in dashboards: dash.close()
            if viewer: viewer.close()
    print('Saved:',args.outdir,'Status:',status,flush=True)
    return int(failed)


if __name__=='__main__': raise SystemExit(main())
