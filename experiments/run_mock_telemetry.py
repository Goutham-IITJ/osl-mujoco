"""Hardware-independent synthetic telemetry demo; existing local browser UI only."""
from pathlib import Path
import argparse
import json
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
from oslbench.telemetry_source import MockTelemetrySource
from oslbench.telemetry_presentation import TelemetryPresentation
from oslbench.replay_presentation import ReplayWebDashboard


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',type=int,default=8792)
    parser.add_argument('--no-browser',action='store_true')
    parser.add_argument('--stale-after',type=float,default=.5,help='Demo receipt-age threshold in seconds, not a hardware limit')
    parser.add_argument('--exit-on-disconnect',action='store_true')
    parser.add_argument('--outdir',help='New evidence directory below build/phase6f_foundation')
    args=parser.parse_args()
    source=MockTelemetrySource()
    presentation=TelemetryPresentation(source,args.stale_after)
    out=None
    if args.outdir:
        out=Path(args.outdir).resolve();base=(ROOT/'build/phase6f_foundation').resolve()
        if not out.is_relative_to(base) or out==base:parser.error('--outdir must be a new subdirectory under build/phase6f_foundation')
        out.mkdir(parents=True,exist_ok=False)
    dashboard=ReplayWebDashboard(presentation,args.port,not args.no_browser)
    print('SYNTHETIC MOCK TELEMETRY - NO HARDWARE',flush=True)
    print(dashboard.url,flush=True)
    if out:(out/'launch.json').write_text(json.dumps(dict(argv=sys.argv,url=dashboard.url,metadata=source.metadata),indent=2),encoding='utf-8')
    start=time.monotonic()
    try:
        while source.status!='CLOSED':
            presentation.update(time.monotonic()-start)
            dashboard.pump();dashboard.update(presentation.snapshot())
            if args.exit_on_disconnect and source.status=='DISCONNECTED':break
            time.sleep(.02)
    except KeyboardInterrupt:pass
    finally:
        if out:(out/'final_snapshot.json').write_text(json.dumps(presentation.snapshot(),indent=2),encoding='utf-8')
        source.close();dashboard.close()
    return 0


if __name__=='__main__':raise SystemExit(main())
