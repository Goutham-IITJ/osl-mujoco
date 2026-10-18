"""Display recorded telemetry in the existing browser dashboard surface; no physics."""
from pathlib import Path
import argparse
import json
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0,str(ROOT))
from oslbench.telemetry.replay import ReplaySource
from oslbench.presentation.replay import ReplayPresentation, ReplayWebDashboard, SPEEDS


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',required=True)
    parser.add_argument('--speed',type=float,choices=SPEEDS,default=1.)
    parser.add_argument('--autoplay',action='store_true',help='Default is paused')
    parser.add_argument('--port',type=int,default=8791)
    parser.add_argument('--no-browser',action='store_true')
    parser.add_argument('--exit-on-end',action='store_true')
    parser.add_argument('--outdir',help='Optional new evidence directory under build/phase6e_replay')
    args=parser.parse_args()
    source=ReplaySource(args.manifest)
    presentation=ReplayPresentation(source,args.speed)
    out=None
    if args.outdir:
        out=Path(args.outdir).resolve()
        base=(ROOT/'build/phase6e_replay').resolve()
        if not out.is_relative_to(base) or out==base:
            parser.error('--outdir must be a new subdirectory under build/phase6e_replay')
        out.mkdir(parents=True,exist_ok=False)
    dashboard=ReplayWebDashboard(presentation,args.port,not args.no_browser)
    if args.autoplay:presentation.key('space')
    print('RECORDED TELEMETRY REPLAY: '+source.dataset_id,flush=True)
    print(dashboard.url,flush=True)
    print('Browser controls: play/pause, reset, next timestamp, speed, stop. Ctrl+C stops.',flush=True)
    if out:
        (out/'launch.json').write_text(json.dumps(dict(argv=sys.argv,url=dashboard.url,manifest=str(Path(args.manifest).resolve())),indent=2),encoding='utf-8')
        (out/'dashboard.html').write_bytes(dashboard.html)
    previous=time.monotonic()
    try:
        while presentation.status!='STOPPED':
            now=time.monotonic()
            presentation.advance(now-previous)
            previous=now
            # Account for elapsed time with the old speed/status before applying UI events.
            dashboard.pump()
            dashboard.update(presentation.snapshot())
            if args.exit_on_end and presentation.status=='EOF':break
            time.sleep(.02)
    except KeyboardInterrupt:
        presentation.key('escape')
    finally:
        if out:
            (out/'final_snapshot.json').write_text(json.dumps(presentation.snapshot(),indent=2),encoding='utf-8')
        dashboard.close()
    return 0


if __name__=='__main__':
    raise SystemExit(main())
