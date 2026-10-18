"""Observation-only presentation over ReplaySource's one virtual clock."""
import json
import math
import threading

from oslbench.telemetry.replay import ReplaySource
from oslbench.presentation.drivetrain_dashboard import DrivetrainWebDashboard
from oslbench.presentation.observations import observation_channels, MAX_PLOT_PTS  # compatibility re-exports

SPEEDS = (.25, .5, 1., 2.)


class ReplayPresentation:
    def __init__(self, source, speed=1.):
        if not isinstance(source, ReplaySource):
            raise TypeError("Expected ReplaySource")
        self.source = source
        self.set_speed(speed)
        self.reset()

    def reset(self):
        self.source.reset()
        self.latest = {}
        self.history = {name: [] for name in self.source.manifest['channels']}
        self.delivered = 0
        self.status = 'EOF' if self.source.ended else 'PAUSED'

    def set_speed(self, speed):
        if isinstance(speed, bool) or speed not in SPEEDS:
            raise ValueError("Speed must be .25, .5, 1 or 2")
        self.speed = float(speed)

    def key(self, key):
        if key == 'r':
            self.reset()
        elif key == 'escape':
            self.status = 'STOPPED'
        elif key == 'space' and self.status in ('PAUSED', 'PLAYING'):
            self.status = 'PLAYING' if self.status == 'PAUSED' else 'PAUSED'
        elif key == 'step' and self.status == 'PAUSED':
            self.step()
        elif key in ('speed0.25', 'speed0.5', 'speed1', 'speed2'):
            self.set_speed(float(key[5:]))

    def _deliver(self, records):
        for record in records:
            self.latest[record.channel] = record
            self.history[record.channel].append(record)
            self.delivered += 1
        if self.source.ended:
            self.status = 'EOF'
        return records

    def advance(self, elapsed_s):
        if isinstance(elapsed_s, bool) or not isinstance(elapsed_s, (int, float)) or not math.isfinite(elapsed_s) or elapsed_s < 0:
            raise ValueError("Elapsed wall time must be finite and nonnegative")
        if self.status != 'PLAYING':
            return ()
        end = self.source.records[-1].time_s
        target = min(end, self.source.time_s + elapsed_s * self.speed)
        return self._deliver(self.source.advance_to(target))

    def step(self):
        """Deliver all observations at the next timestamp, remaining paused."""
        if self.status != 'PAUSED' or self.source.ended:
            return ()
        target = self.source.records[self.delivered].time_s
        return self._deliver(self.source.advance_to(target))

    def snapshot(self):
        manifest = self.source.manifest
        channels = observation_channels(manifest, self.latest, self.history, self.source.time_s)
        simulation = manifest['source']=='simulation' and all(m['source']=='simulation' for m in manifest['channels'].values())
        return dict(dataset_id=self.source.dataset_id, clock_id=self.source.clock_id,
            identity=manifest.get('identity', {}), provenance=manifest['source'],
            label='RECORDED SIMULATION TELEMETRY - NOT PHYSICAL HARDWARE DATA' if simulation else 'RECORDED OBSERVATIONS - PROVENANCE AS DECLARED; NOT VERIFIED HARDWARE DATA',
            status=self.status, time_s=self.source.time_s, speed=self.speed,
            delivered=self.delivered, total=len(self.source.records),
            last_sequence=self.delivered-1 if self.delivered else None,
            channels=channels)


# Replay-specific content reuses the existing server, /data, /key, lifecycle and
# browser opening implementation. The live dashboard's page/update remain untouched.
_PAGE = r"""<!doctype html><html lang="en"><meta charset="utf-8">
<title>OSL recorded telemetry replay</title>
<style>
body{background:#101b2a;color:#e5edf5;font:15px system-ui;margin:24px}h1{font-size:24px;margin-bottom:8px}
button{background:#263e58;color:white;border:1px solid #58728e;padding:9px 13px;margin:4px;cursor:pointer}
.banner{color:#ffc86a}#status{font:17px monospace;padding:14px 0}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));gap:14px}
.card{background:#192a3d;padding:16px;border-radius:8px}.card h3{font-size:15px;margin:0 0 8px}canvas{width:100%;height:150px}small{color:#bccddd}table{width:100%;border-collapse:collapse;font-size:12px}td,th{padding:8px;text-align:left;border-bottom:1px solid #405066}.scroll{overflow:auto}.bad{color:#ffbd9e}#error{color:#ffbd9e}details{margin:14px 0}pre{white-space:pre-wrap;word-break:break-all}
</style><h1 id="heading">OSL telemetry replay</h1><div id="label" class="banner"></div><div id="identity"></div>
<p id="mode_note">Recorded observations only. No physics, controller or actuator commands. No complete robot pose is reconstructed.</p>
<div><button data-key="space">Play / pause (Space)</button><button data-key="r">Reset (R)</button><button data-key="step">Next timestamp (N)</button>
<button data-key="speed0.25">0.25x</button><button data-key="speed0.5">0.5x</button><button data-key="speed1">1x</button><button data-key="speed2">2x</button><button data-key="escape">Stop server</button></div>
<div id="status"></div><div id="error"></div>
<div class="scroll"><table><thead><tr><th>Channel</th><th>Value</th><th>Unit / frame</th><th>Quality / retained</th><th>Source time (s)</th><th id="age_label">Age (s)</th><th>Local receipt (s)</th><th>Sequence / snapshot</th><th>Stage / provenance</th></tr></thead><tbody id="rows"></tbody></table></div>
<p><small>Each channel keeps its own timestamp. HELD means retained for context, not a new observation. Replay assumes no freshness threshold; mock STALE uses its explicitly configured receipt-age threshold, not a hardware limit. Dots are recorded valid samples; missing/invalid samples create no plotted value. Axes: source seconds and the channel's native unit.</small></p>
<div class="grid" id="plots"></div><details><summary>Source identity / provenance metadata</summary><pre id="meta"></pre></details>
<script>
const $=id=>document.getElementById(id);let busy=false;
function send(k){return fetch('/key?k='+encodeURIComponent(k));}
document.querySelectorAll('button').forEach(b=>b.onclick=()=>send(b.dataset.key));
document.addEventListener('keydown',e=>{let k=({' ':'space','r':'r','n':'step'})[e.key.toLowerCase()];if(k){e.preventDefault();send(k);}});
const fmt=x=>x===null?'--':Number(x).toPrecision(8);
function draw(c,points,unit){const ctx=c.getContext('2d');c.width=Math.max(320,c.clientWidth*devicePixelRatio);c.height=150*devicePixelRatio;const w=c.width,h=c.height;ctx.clearRect(0,0,w,h);const p=points.filter(a=>a[1]!==null);if(!p.length)return;let xs=p.map(a=>a[0]),ys=p.map(a=>a[1]);let lo=Math.min(...ys),hi=Math.max(...ys),a=Math.min(...xs),b=Math.max(...xs);ctx.fillStyle='#9eb8d0';ctx.font=`${11*devicePixelRatio}px system-ui`;ctx.fillText(`${lo.toPrecision(4)} .. ${hi.toPrecision(4)} ${unit}`,6,14*devicePixelRatio);ctx.fillText(`${a.toPrecision(4)} .. ${b.toPrecision(4)} s`,6,h-3);ctx.fillStyle='#6cdbcf';for(const [x,y] of p){let px=8+(x-a)/(b-a||1)*(w-16),py=h-24*devicePixelRatio-(y-lo)/(hi-lo||1)*(h-45*devicePixelRatio);ctx.beginPath();ctx.arc(px,py,2*devicePixelRatio,0,7);ctx.fill();}}
async function poll(){if(busy)return;busy=true;try{const response=await fetch('/data');if(!response.ok)throw Error(response.status);const s=await response.json();$('label').textContent=s.label;if(s.mode==='mock'||s.mode==='telemetry'){document.title=s.mode==='mock'?'OSL synthetic mock telemetry':'OSL observation telemetry';$('heading').textContent=document.title;$('mode_note').textContent=s.mode==='mock'?'Synthetic observations only. No Raspberry Pi, physical sensors, physics or actuator commands.':'External observations as declared. No physics or actuator commands.';$('age_label').textContent='Receipt age (s)';document.querySelectorAll('button').forEach(b=>b.hidden=b.dataset.key!=='escape');}$('identity').textContent=`Dataset: ${s.dataset_id} | Clock: ${s.clock_id}`;$('status').textContent=`${s.status} | t=${fmt(s.time_s)} s | ${(s.mode==='mock'||s.mode==='telemetry')?'local receipt clock':s.speed+'x'} | ${s.delivered}/${s.total??'--'} observations | sequence ${s.last_sequence??'--'}`;$('meta').textContent=JSON.stringify(s.identity,null,2);$('rows').replaceChildren();$('plots').replaceChildren();for(const [name,v] of Object.entries(s.channels)){const tr=document.createElement('tr');for(const val of [name,fmt(v.value),`${v.unit} / ${v.frame}`,`${v.quality}${v.held?' / HELD':''}${v.freshness?' / '+v.freshness:''}`,fmt(v.time_s),fmt(v.age_s),fmt(v.received_time_s??null),`${v.sequence??'--'} / ${v.snapshot_id??'--'}`,`${v.stage} / ${v.source}`]){const td=document.createElement('td');td.textContent=val;tr.append(td);}if(v.quality!=='VALID')tr.className='bad';tr.title=v.sign;$('rows').append(tr);const card=document.createElement('div');card.className='card';const title=document.createElement('h3');title.textContent=name;const canvas=document.createElement('canvas');card.append(title,canvas);$('plots').append(card);draw(canvas,v.points,v.unit);}$('error').textContent='';}catch(e){$('error').textContent='Connection unavailable / server stopped. Displayed data is retained.';}finally{busy=false;}}
setInterval(poll,80);poll();
</script></html>"""


class ReplayWebDashboard(DrivetrainWebDashboard):
    def __init__(self, presentation, port=8791, open_browser=True):
        self.alive = True
        self.key_cb = presentation.key
        self._keys = []
        self.hits = 0
        self.html = _PAGE.encode('utf-8')
        self.update(presentation.snapshot())
        self.srv = self._serve(port)
        self.port = self.srv.server_address[1]
        self.url = f'http://127.0.0.1:{self.port}/'
        threading.Thread(target=self.srv.serve_forever, daemon=True,
                         name='osl-replay-dashboard').start()
        self.opened = bool(open_browser) and self._open()

    def update(self, snapshot):
        self._body = json.dumps(snapshot, allow_nan=False).encode('utf-8')
