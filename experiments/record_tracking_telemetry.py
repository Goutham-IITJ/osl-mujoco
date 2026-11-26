"""Record two-joint hardware telemetry; no simulation or motor-command endpoint."""
import argparse,csv,json,os,sys,time
from pathlib import Path
from datetime import datetime
from urllib.request import Request,build_opener,ProxyHandler
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from hardware.gait_tracking.telemetry import SCHEMA,validate_sample

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--host',default='127.0.0.1');p.add_argument('--port',type=int,default=8794)
    p.add_argument('--seconds',type=float,default=120.)
    p.add_argument('--outdir',type=Path)
    args=p.parse_args()
    out=args.outdir or ROOT/'build'/('tracking_received_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    out.mkdir(parents=True,exist_ok=False)
    headers={'Authorization':'Bearer '+os.environ['OSL_TELEMETRY_TOKEN']} if os.environ.get('OSL_TELEMETRY_TOKEN') else {}
    opener=build_opener(ProxyHandler({}))
    def get(path):
        with opener.open(Request(f'http://{args.host}:{args.port}'+path,headers=headers),timeout=1) as response:
            data=response.read(8_000_001)
        if len(data)>8_000_000:raise ValueError('Oversized response')
        return json.loads(data)
    meta=None;deadline=time.monotonic()+60
    print('Waiting up to 60 seconds for hardware/synthetic server. This recorder sends no motor commands.')
    while time.monotonic()<deadline:
        try:meta=get('/v1/metadata');break
        except Exception:time.sleep(.2)
    if meta is None:raise RuntimeError('No server connection; check host/token/port')
    if meta.get('schema')!=SCHEMA:raise ValueError('Requires two-joint telemetry schema; old knee receiver is separate')
    (out/'metadata.json').write_text(json.dumps(meta,indent=2)+'\n')
    sequence=-1;counts={'knee':0,'ankle':0};jointseq={'knee':-1,'ankle':-1};last_time=-1.
    streams={};writers={};status='DURATION_REACHED';started=time.monotonic();last_print=started
    try:
        with (out/'hardware_batches.jsonl').open('x') as raw:
            while time.monotonic()-started<args.seconds:
                batch=get(f'/v1/samples?after={sequence}&limit=1000')
                if batch.get('schema')!=SCHEMA or batch.get('session_id')!=meta['session_id']:
                    raise ValueError('Session/schema changed')
                raw.write(json.dumps(dict(kind='batch',receipt_elapsed_s=time.monotonic()-started,payload=batch))+'\n');raw.flush()
                for row in batch['samples']:
                    validate_sample(row);joint=row['joint']
                    if row['sequence']!=sequence+1 or row['joint_sequence']!=jointseq[joint]+1 or row['time_s']<=last_time:
                        raise ValueError('Missing/duplicate/out-of-order samples; recording incomplete')
                    if joint not in streams:
                        streams[joint]=(out/(joint+'_telemetry.csv')).open('x',newline='')
                        writers[joint]=csv.DictWriter(streams[joint],fieldnames=list(row));writers[joint].writeheader()
                    writers[joint].writerow(row);streams[joint].flush()
                    sequence=row['sequence'];jointseq[joint]=row['joint_sequence'];last_time=row['time_s'];counts[joint]+=1
                current=get('/v1/metadata')
                if current.get('producer_complete') and sequence==current.get('final_sequence'):
                    status='COMPLETE';break
                if time.monotonic()-last_print>2:
                    print('Received:',counts,'last sequence:',sequence);last_print=time.monotonic()
                time.sleep(.05)
    except KeyboardInterrupt:status='STOPPED_BY_USER'
    except Exception as exc:status='INCOMPLETE: '+str(exc)
    finally:
        for stream in streams.values():stream.close()
        (out/'summary.json').write_text(json.dumps(dict(status=status,counts=counts,last_sequence=sequence,
            note='Raw controller angles/current signs; commanded current is not measured current; laptop stop does not stop hardware'),indent=2)+'\n')
        print(status,'Saved:',out)
    return 0 if status=='COMPLETE' else 1

if __name__=='__main__':raise SystemExit(main())
