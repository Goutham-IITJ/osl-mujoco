"""Receive, record and display physical knee observations; never commands a motor."""
import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import time
from urllib.error import HTTPError, URLError

from oslbench.telemetry.knee import KneeReceiver
from oslbench.presentation.replay import ReplayWebDashboard


PAGE = '''<!doctype html><meta charset="utf-8"><title>OSL knee telemetry</title>
<style>body{font:16px system-ui;margin:30px;background:#101c29;color:#eee}canvas{width:100%;height:360px}pre{white-space:pre-wrap}</style>
<h1>OSL knee telemetry</h1><p>Encoder motion and controller target. No simulated trace or motor commands.</p>
<p id="state">Connecting…</p><p>Blue: target · Orange: encoder · flexion-positive degrees; source seconds</p>
<canvas id="plot"></canvas><pre id="stats"></pre>
<script>let busy=false;async function poll(){if(busy)return;busy=true;try{
const r=await fetch('/data');if(!r.ok)throw Error();const s=await r.json();
document.getElementById('state').textContent=s.status+' | '+s.source+' | '+s.session_id;
document.getElementById('stats').textContent=JSON.stringify(s.metrics,null,2);
const c=document.getElementById('plot'),x=c.getContext('2d');c.width=c.clientWidth*devicePixelRatio;c.height=360*devicePixelRatio;
const p=s.points;if(p.length){const a=p[0][0],b=p[p.length-1][0],v=p.flatMap(r=>r.slice(1).filter(y=>y!==null));
const lo=Math.min(...v)-2,hi=Math.max(...v)+2,w=c.width,h=c.height;
x.fillStyle='#eee';x.font='16px system-ui';x.fillText(`${lo.toFixed(1)} to ${hi.toFixed(1)} deg; ${a.toFixed(2)} to ${b.toFixed(2)} s`,20,20);
for(let k=1;k<=2;k++){x.strokeStyle=k===1?'#6db8ff':'#ffaf62';x.lineWidth=2;x.beginPath();let pen=false;for(const r of p){if(r[k]===null){pen=false;continue;}const px=20+(r[0]-a)/(b-a||1)*(w-40),py=h-20-(r[k]-lo)/(hi-lo)*(h-50);if(pen)x.lineTo(px,py);else x.moveTo(px,py);pen=true;}x.stroke();}}}
catch(e){document.getElementById('state').textContent='DISCONNECTED — retained data, not live';}finally{busy=false;}}
setInterval(poll,100);poll();</script>'''


class KneePresentation:
    def __init__(self, metadata):
        self.metadata = metadata
        self.points = deque(maxlen=2000)
        self.status = "WAITING"
        self.count = self.valid = self.clipped = 0
        self.sum_sq = self.peak = self.peak_requested = self.peak_commanded = 0.0
        self.missing = 0

    def key(self, key):
        pass

    def add(self, batch):
        sign = self.metadata["flexion_positive_multiplier"]
        for row in batch["samples"]:
            if self.points and row["sequence"] != self.last_sequence+1:
                self.points.append([row["time_s"], None, None])
            self.last_sequence = row["sequence"]
            self.points.append([row["time_s"], sign*row["reference_deg"],
                                sign*row["actual_deg"] if row["encoder_valid"] else None])
            self.count += 1
            if row["encoder_valid"]:
                error = row["reference_deg"]-row["actual_deg"]
                self.sum_sq += error*error
                self.peak = max(self.peak, abs(error))
                self.valid += 1
            req, cmd = row["requested_current_A"], row["commanded_current_A"]
            self.clipped += abs(req-cmd) > 1e-9
            self.peak_requested = max(self.peak_requested, abs(req))
            self.peak_commanded = max(self.peak_commanded, abs(cmd))
        self.missing += batch["missing_this_batch"]
        age = batch["producer_age_s"]
        self.status = "STALE" if age is None or age > .5 else "LIVE"
        if self.missing:
            self.status += " — SAMPLE GAPS"

    def snapshot(self):
        return dict(status=self.status, source=self.metadata["source"],
            session_id=self.metadata["session_id"], points=list(self.points),
            metrics=dict(samples=self.count, valid_encoder_samples=self.valid,
                missing_samples=self.missing,
                hardware_target_RMS_deg=math.sqrt(self.sum_sq/self.valid) if self.valid else None,
                hardware_target_peak_deg=self.peak if self.valid else None,
                peak_requested_current_A=self.peak_requested,
                peak_commanded_current_A=self.peak_commanded, clipped_samples=self.clipped,
                note="Hardware target error, NOT sim-to-real error. Commanded current is not measured current."))


def connect_receiver(host, port, token, wait_s=30):
    """Allow recording to be started before a short hardware step test."""
    deadline = time.monotonic() + wait_s
    while True:
        try:
            return KneeReceiver(host, port, token)
        except HTTPError:
            raise  # Authentication/protocol errors need correction, not retries.
        except (URLError, TimeoutError, OSError):
            if time.monotonic() >= deadline:
                raise
            time.sleep(.1)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8794)
    parser.add_argument("--outdir", type=Path, required=True)
    parser.add_argument("--seconds", type=float, default=60)
    parser.add_argument("--dashboard-port", type=int, default=8795)
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args(argv)
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error("--seconds must be finite and positive")
    if args.outdir.exists():
        raise FileExistsError(args.outdir)
    print("Waiting up to 30 seconds for the hardware telemetry server...", flush=True)
    receiver = connect_receiver(args.host, args.port, os.environ.get("OSL_TELEMETRY_TOKEN"))
    args.outdir.mkdir(parents=True, exist_ok=False)
    (args.outdir/"metadata.json").write_text(json.dumps(receiver.metadata, indent=2), encoding="utf-8")
    presentation = KneePresentation(receiver.metadata)
    dashboard = ReplayWebDashboard(presentation, args.dashboard_port, open_browser=False)
    dashboard.html = PAGE.encode("utf-8")
    if not args.no_browser:
        dashboard._open()
    print(f"Dashboard: {dashboard.url}\nRecording: {args.outdir}", flush=True)
    start = time.monotonic()
    failed = False
    try:
        with (args.outdir/"batches.jsonl").open("x", encoding="utf-8") as log:
            while time.monotonic()-start < args.seconds:
                try:
                    batch = receiver.poll()
                except (URLError, TimeoutError, OSError) as exc:
                    presentation.status = "DISCONNECTED: " + type(exc).__name__
                    log.write(json.dumps(dict(event="disconnect", receipt_elapsed_s=time.monotonic()-start))+'\n')
                except ValueError as exc:
                    presentation.status = "STOPPED: " + str(exc)
                    failed = True
                    break
                else:
                    batch["receipt_elapsed_s"] = time.monotonic()-start
                    log.write(json.dumps(batch, allow_nan=False)+'\n')
                    presentation.add(batch)
                log.flush()
                dashboard.update(presentation.snapshot())
                time.sleep(.05)
    except KeyboardInterrupt:
        presentation.status = "STOPPED by user"
    finally:
        snapshot = presentation.snapshot()
        (args.outdir/"summary.json").write_text(json.dumps(snapshot, indent=2), encoding="utf-8")
        dashboard.close()
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
