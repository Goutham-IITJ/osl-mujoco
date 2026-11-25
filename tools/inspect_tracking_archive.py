"""Static source and CSV audit. Never imports or executes supplied hardware code."""
import argparse,ast,csv,hashlib,io,json,math,statistics,zipfile
from pathlib import Path,PurePosixPath
from datetime import datetime

ROOT=Path(__file__).resolve().parents[1]

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--archive',type=Path,default=Path.home()/'Downloads/Tracking Results & Codes & Dataset.zip')
    p.add_argument('--outdir',type=Path)
    a=p.parse_args();out=a.outdir or ROOT/'build'/('tracking_inspection_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    out.mkdir(parents=True,exist_ok=False)
    report=dict(archive_sha256=hashlib.sha256(a.archive.read_bytes()).hexdigest(),sources=[],datasets=[],results=[],members=[])
    with zipfile.ZipFile(a.archive) as z:
        for info in z.infolist():
            path=PurePosixPath(info.filename)
            if path.is_absolute() or '..' in path.parts or '\\' in info.filename:
                raise ValueError('Unsafe archive member')
            report['members'].append(dict(path=info.filename,size=info.file_size))
            if not info.filename.endswith(('.py','.csv')):continue
            raw=z.read(info);s=raw.decode('utf-8-sig')
            dest=out/'originals'/Path(*path.parts[1:]);dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes(raw)
            if info.filename.endswith('.py'):
                tree=ast.parse(s);settings={}
                for n in tree.body:
                    if isinstance(n,ast.Assign) and isinstance(n.targets[0],ast.Name):
                        try:value=ast.literal_eval(n.value)
                        except (ValueError,TypeError):continue
                        if isinstance(value,(int,float,bool,str,type(None))):settings[n.targets[0].id]=value
                report['sources'].append(dict(path=info.filename,sha256=hashlib.sha256(raw).hexdigest(),settings=settings))
                continue
            rows=list(csv.DictReader(io.StringIO(s)))
            if '/Dataset/' in info.filename:
                report['datasets'].append(dict(path=info.filename,rows=len(rows),columns=list(rows[0]),
                    time_range=[float(rows[0]['time']),float(rows[-1]['time'])],
                    nonfinite_by_column={k:sum(not math.isfinite(float(r[k])) for r in rows) for k in rows[0]}))
                continue
            joint='knee' if path.name.startswith('knee') else 'ankle';sign=1 if joint=='knee' else -1
            kp,ki,kd=(1.5,.15,.06) if joint=='knee' else (2.,0.,.075)
            errors=[];pid_errors=[];integral=0.;previous=None;times=[];dts=[];currents=[];ref=[];actual=[]
            for r in rows:
                t=float(r['time_s']);dt=float(r['dt_s']);target=float(r['reference_deg']);measured=float(r['actual_deg'])
                if not all(math.isfinite(float(r[k])) for k in r if k!='current_clamped') or dt<=0:
                    raise ValueError('Nonfinite result or nonpositive interval')
                err=target-measured;integral=max(-50,min(50,integral+err*dt))
                terms=[kp*err,ki*integral,0. if previous is None else kd*(err-previous)/dt]
                previous=err
                pid_errors.append(max(abs(v-sign*float(r[k])) for v,k in zip(terms,['p_term_A','i_term_A','d_term_A'])))
                times.append(t);dts.append(dt);errors.append(err);currents.append(float(r['current_cmd_A']));ref.append(target);actual.append(measured)
            intervals=[b-a for a,b in zip(times,times[1:])]
            report['results'].append(dict(path=info.filename,run=path.parts[-3],joint=joint,rows=len(rows),
                source_sha256=hashlib.sha256(raw).hexdigest(),time_range_s=[times[0],times[-1]],
                nominal_rate_from_median_Hz=1/statistics.median(intervals),
                time_monotonic=all(d>0 for d in intervals),max_time_gap_s=max(intervals),
                dt_range_s=[min(dts),max(dts)],reference_range_deg=[min(ref),max(ref)],actual_range_deg=[min(actual),max(actual)],
                hardware_target_rms_deg=math.sqrt(sum(e*e for e in errors)/len(errors)),
                hardware_target_peak_deg=max(map(abs,errors)),peak_commanded_current_A=max(map(abs,currents)),
                clipped_rows=sum(r['current_clamped']=='True' for r in rows),
                pid_term_max_error_A=max(pid_errors),
                torque_scale_max_error_Nm=max(abs(float(r['torque_Nm'])-.5701*float(r['current_cmd_A'])) for r in rows),
                timing_note='Shared loop-start time_s; per-joint dt reconstructs later sensor-read instants',
                angle_note='Knee reference/actual exported sign-flipped; error and reference velocity are not flipped' if joint=='knee' else 'Ankle angle exported directly; motor/PID current sign reversed'))
    (out/'inventory.json').write_text(json.dumps(report,indent=2)+'\n')
    with (out/'recording_summary.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(report['results'][0]));w.writeheader();w.writerows(report['results'])
    print('Saved:',out)
    for r in report['results']:
        print(r['run'],r['joint'],r['rows'],round(r['time_range_s'][1],3),'RMS',round(r['hardware_target_rms_deg'],3),
              'clipped',r['clipped_rows'],'PID audit',r['pid_term_max_error_A'],'range',r['reference_range_deg'])

if __name__=='__main__':main()
