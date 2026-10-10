#!/usr/bin/env python3
"""
drivetrain_dashboard.py -- the Option-C demo's second window.  PRESENTATION ONLY.

WHAT THIS FILE IS
    A CONSUMER of simulation state.  It holds no model, takes no timestep, integrates
    nothing and computes no torque or current.  `update()` is handed the numbers the
    drivetrain layer and MuJoCo just produced, copies them into a dict, and an HTTP
    thread serialises that dict when the browser asks for it.  Drawing happens in the
    browser's process.

    Two consequences worth stating out loud for a presentation:
      * the page CANNOT show anything the simulation did not produce -- there is no
        second integrator, no interpolation and no smoothing anywhere in this file;
      * it cannot write a command.  Delete this file and the physics is identical.

    `tests/test_drivetrain_sim.py` asserts the first half of that structurally: this
    module imports neither `mujoco` nor `oslbench.simulation`, so it *cannot* step a
    plant even by accident.

WHY IT IS NOT oslbench/dashboard.py
    That one exists, is tested, and drives the SERVO demo (`experiments/run_live_demo.py`)
    with three fixed plots sized by an `ang`/`err`/`tau` limits dict.  Option C has
    eleven signals and four different plots, two of which (theta_j against theta_a/n_t,
    and the deflection band) have no counterpart there.  Widening the existing class
    would have put the working servo demo at risk for a presentation feature, so this is
    a separate file and `oslbench/dashboard.py` is untouched.

THE ONE JUDGEMENT CALL IN HERE
    Plot 2 draws theta_j and theta_a/n_t on the SAME axis.  Everywhere else in this
    repository, putting an actuator-side and a joint-side quantity on one axis is the
    mistake being guarded against -- they differ by n_t = 4.61.  It is correct here and
    only here, because theta_a/n_t has already been referred THROUGH the transmission, so
    the two curves are the same physical quantity measured at the two ends of the belt.
    The GAP between them IS the belt deflection, which is the whole point of the panel:
    a rigid transmission would draw one line.  The axis label says so.

    Torque is handled the other way: `tau_j` (joint side) and `tau_a` (actuator side)
    are both displayed as NUMBERS but are never plotted on one axis, because those two
    really do differ by n_t and comparing them by eye would be wrong.
"""

from __future__ import annotations

import json
import math
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

__all__ = ["BACKENDS", "make_drivetrain_dashboard", "DrivetrainWebDashboard",
           "DrivetrainTerminalDashboard", "NoDashboard", "MAX_PLOT_PTS"]

BACKENDS = ("auto", "web", "terminal", "none")
MAX_PLOT_PTS = 320          # DISPLAY decimation only -- every point drawn is a real step

BANNER = "OPTION C — PAPER-DERIVED ACTUATOR + COMPLIANT BELT"
DISCLAIMER = "SIMULATION ONLY — NOT HARDWARE VALIDATION"


def _sf(v):
    """Scalar for JSON: finite -> rounded float, non-finite -> None plus a bad flag.

    A NaN would serialise as the literal `NaN`, which is not valid JSON and which
    `JSON.parse` rejects -- the page would silently stop updating at the exact moment
    something went wrong, which is the worst possible failure mode for a diagnostic.
    """
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None, True
    if not math.isfinite(f):
        return None, True
    return round(f, 6), False


def _thin(a):
    """A trace for JSON, decimated to MAX_PLOT_PTS and stripped of non-finite values."""
    vals = [float(x) for x in a]
    bad = any(not math.isfinite(x) for x in vals)
    if len(vals) > MAX_PLOT_PTS:
        step = len(vals) // MAX_PLOT_PTS + 1
        vals = vals[::step]
    return [None if not math.isfinite(x) else round(x, 6) for x in vals], bad


# ===================================================================== the page
# Self-contained: no CDN, no internet, no fonts fetched.  Canvas 2D only.
_PAGE = r"""<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8">
<title>__BANNER__</title>
<style>
  :root { color-scheme: light }
  * { box-sizing: border-box }
  body { margin:0; background:#f4f4f2; color:#17171a;
         font:13px/1.45 "Segoe UI",system-ui,-apple-system,sans-serif }
  header { padding:10px 16px 8px; border-bottom:2px solid #17171a; background:#fff }
  h1 { margin:0; font-size:15px; letter-spacing:.055em; font-weight:650 }
  .warn { margin-top:3px; font-size:11.5px; font-weight:650; letter-spacing:.05em;
          color:#9a2b11 }
  .meta { margin-top:4px; font-size:11px; color:#5b5b63 }
  main { padding:12px 16px 20px; max-width:1500px }
  .nums { display:grid; gap:7px; grid-template-columns:repeat(auto-fit,minmax(132px,1fr));
          margin-bottom:13px }
  .n { background:#fff; border:1px solid #d8d8d4; border-left:3px solid #8a8a94;
       border-radius:3px; padding:6px 8px }
  .n.j { border-left-color:#1f4e8c }      /* joint side  */
  .n.a { border-left-color:#8c4a1f }      /* actuator side */
  .n.b { border-left-color:#2c6e49 }      /* belt        */
  .n .k { font-size:9.5px; letter-spacing:.07em; color:#5b5b63; text-transform:uppercase;
          white-space:nowrap; overflow:hidden; text-overflow:ellipsis }
  .n .v { font-size:17px; font-variant-numeric:tabular-nums; font-weight:600;
          margin-top:1px }
  .n .u { font-size:9.5px; color:#75757e; margin-left:3px; font-weight:400 }
  .side { float:right; font-size:8.5px; letter-spacing:.06em; color:#8a8a94;
          margin-top:2px }
  .grid { display:grid; gap:12px; grid-template-columns:repeat(auto-fit,minmax(430px,1fr)) }
  .card { background:#fff; border:1px solid #d8d8d4; border-radius:3px; padding:9px 11px 6px }
  .card h2 { margin:0 0 1px; font-size:11.5px; letter-spacing:.04em; font-weight:650 }
  .card .sub { font-size:10px; color:#75757e; margin-bottom:5px; min-height:13px }
  canvas { width:100%; height:188px; display:block }
  .lg { font-size:10px; color:#4b4b53; margin-top:3px }
  .sw { display:inline-block; width:9px; height:9px; border-radius:2px; margin:0 3px 0 9px;
        vertical-align:-1px }
  .lg span:first-child .sw { margin-left:0 }
  footer { margin-top:14px; padding-top:9px; border-top:1px solid #d8d8d4;
           font-size:10.5px; color:#5b5b63; white-space:pre-wrap }
  .status { margin-top:7px; font-size:10.5px; color:#17171a; font-variant-numeric:tabular-nums }
  .stale { color:#9a2b11; font-weight:650 }
</style></head><body>
<header>
  <h1>__BANNER__</h1>
  <div class="warn">__DISCLAIMER__</div>
  <div class="meta" id="meta"></div>
</header>
<main>
  <div class="nums" id="nums"></div>
  <div class="grid">
    <div class="card">
      <h2>1 &nbsp; Reference vs actual knee angle</h2>
      <div class="sub">joint side, both &mdash; the tracking task</div>
      <canvas id="c1"></canvas>
      <div class="lg"><span><i class="sw" style="background:#9a9aa4"></i>reference</span>
        <span><i class="sw" style="background:#1f4e8c"></i>actual &theta;<sub>j</sub></span></div>
    </div>
    <div class="card">
      <h2>2 &nbsp; &theta;<sub>j</sub> vs &theta;<sub>a</sub>/n<sub>t</sub> &mdash; the belt deflection, visually</h2>
      <div class="sub">both referred to the JOINT side. The two nearly coincide &mdash; the
        belt is stiff &mdash; so their difference is drawn again on the right axis at its
        own scale. A rigid transmission would draw one line and a flat zero.</div>
      <canvas id="c2"></canvas>
      <div class="lg"><span><i class="sw" style="background:#1f4e8c"></i>&theta;<sub>j</sub> (knee, left)</span>
        <span><i class="sw" style="background:#8c4a1f"></i>&theta;<sub>a</sub>/n<sub>t</sub> (actuator, left)</span>
        <span><i class="sw" style="background:#2c6e49"></i>gap = &minus;&theta;<sub>s</sub> (right)</span></div>
    </div>
    <div class="card">
      <h2>3 &nbsp; Belt deflection &theta;<sub>s</sub> vs gait phase</h2>
      <div class="sub" id="sub3"></div>
      <canvas id="c3"></canvas>
      <div class="lg"><span><i class="sw" style="background:#2c6e49"></i>&theta;<sub>s</sub></span>
        <span><i class="sw" style="background:#efe3c8"></i>outside Best et al. fitted range</span></div>
    </div>
    <div class="card">
      <h2>4 &nbsp; Joint torque and commanded current</h2>
      <div class="sub">separate scales &mdash; left N&middot;m (joint side), right A (drive).
        Never one axis: these are different quantities.</div>
      <canvas id="c4"></canvas>
      <div class="lg"><span><i class="sw" style="background:#1f4e8c"></i>&tau;<sub>j</sub> (left)</span>
        <span><i class="sw" style="background:#9a2b11"></i>I<sub>q</sub> (right)</span></div>
    </div>
  </div>
  <div class="status" id="status"></div>
  <footer id="foot"></footer>
</main>
<script>
const CFG = __CONFIG__, POLL = __POLL__;
const DPR = Math.max(1, Math.min(3, window.devicePixelRatio || 1));
document.getElementById("meta").textContent =
  "subject " + CFG.subject + "  ·  " + CFG.trial;
document.getElementById("sub3").textContent =
  "shaded band = |θs| > " + CFG.fit_edge.toFixed(3) +
  " rad, where the belt law is extrapolation of the paper's regression, not their data";

// Eleven readouts.  `s` tags the side a number belongs to, because joint-side and
// actuator-side quantities must never be mistaken for one another.
const NUMS = [
  {k:"gait phase",            f:"phase",  u:"%",       d:1, s:""},
  {k:"reference angle",       f:"ref",    u:"deg",     d:2, s:"joint"},
  {k:"actual knee angle θj",f:"act",    u:"deg",     d:2, s:"joint"},
  {k:"tracking error",        f:"err",    u:"deg",     d:3, s:"joint"},
  {k:"joint torque τj",  f:"tau_j",  u:"N·m",d:3, s:"joint"},
  {k:"motor current Iq",      f:"i_q",    u:"A",       d:3, s:"drive"},
  {k:"actuator angle θa",f:"th_a",   u:"rad",     d:4, s:"actuator"},
  {k:"actuator vel θȧ", f:"th_ad", u:"rad/s", d:3, s:"actuator"},
  {k:"belt deflection θs",f:"th_s",  u:"rad",     d:5, s:"belt"},
  {k:"belt torque τa",   f:"tau_a",  u:"N·m",d:3, s:"actuator"},
  {k:"belt stiffness Ks",     f:"k_s",    u:"N·m/rad", d:1, s:"belt"},
];
const CLS = {joint:"j", actuator:"a", belt:"b", drive:"a", "":""};
const box = document.getElementById("nums");
NUMS.forEach(d => {
  const el = document.createElement("div");
  el.className = "n " + (CLS[d.s] || "");
  el.innerHTML = '<div class="k">' + d.k + (d.s ? '<span class="side">' + d.s +
    '</span>' : '') + '</div><div class="v" id="v_' + d.f + '">&mdash;</div>';
  box.appendChild(el);
});

function fit(c){
  const r = c.getBoundingClientRect();
  c.width = Math.max(1, Math.round(r.width*DPR));
  c.height = Math.max(1, Math.round(r.height*DPR));
  const x = c.getContext("2d"); x.setTransform(DPR,0,0,DPR,0,0); return x;
}
const PAD = {l:46, r:50, t:8, b:20};
function frame(x, w, h, ylo, yhi, ylab, rlo, rhi, rlab, rcol){
  rcol = rcol || "#9a2b11";
  x.clearRect(0,0,w,h);
  const X = p => PAD.l + (w-PAD.l-PAD.r)*(p/100);
  const Y = v => h-PAD.b - (h-PAD.t-PAD.b)*((v-ylo)/((yhi-ylo)||1));
  // stance shading: human stance phase, drawn for orientation only
  x.fillStyle = "#ececeb"; x.fillRect(X(0), PAD.t, X(CFG.stance_end)-X(0), h-PAD.t-PAD.b);
  x.strokeStyle = "#dcdcd8"; x.lineWidth = 1; x.font = "10px system-ui";
  x.fillStyle = "#75757e";
  for (let i=0;i<=4;i++){
    const v = ylo + (yhi-ylo)*i/4, y = Math.round(Y(v))+0.5;
    x.beginPath(); x.moveTo(PAD.l,y); x.lineTo(w-PAD.r,y); x.stroke();
    x.textAlign="right"; x.fillText(v.toFixed(Math.abs(yhi-ylo)<0.2?3:1), PAD.l-4, y+3);
    if (rlab){
      const rv = rlo + (rhi-rlo)*i/4;
      x.textAlign="left"; x.fillStyle=rcol;
      x.fillText(rv.toFixed(Math.abs(rhi-rlo)<4?2:1), w-PAD.r+4, y+3);
      x.fillStyle="#75757e";
    }
  }
  x.textAlign="center";
  for (let p=0;p<=100;p+=25){
    const px = Math.round(X(p))+0.5;
    x.beginPath(); x.moveTo(px,PAD.t); x.lineTo(px,h-PAD.b); x.stroke();
    x.fillText(p+"%", px, h-PAD.b+13);
  }
  if (ylo < 0 && yhi > 0){
    const y0 = Math.round(Y(0))+0.5;
    x.strokeStyle="#b4b4ac"; x.beginPath();
    x.moveTo(PAD.l,y0); x.lineTo(w-PAD.r,y0); x.stroke();
  }
  x.save(); x.translate(11, h/2); x.rotate(-Math.PI/2); x.textAlign="center";
  x.fillText(ylab, 0, 0); x.restore();
  if (rlab){ x.save(); x.translate(w-9, h/2); x.rotate(Math.PI/2); x.textAlign="center";
             x.fillStyle=rcol; x.fillText(rlab, 0, 0); x.restore(); }
  return {X, Y, Yr: v => h-PAD.b - (h-PAD.t-PAD.b)*((v-rlo)/((rhi-rlo)||1))};
}
function line(x, A, xs, ys, col, map){
  if (!xs || xs.length < 2) return;
  x.strokeStyle = col; x.lineWidth = 1.6; x.lineJoin="round"; x.beginPath();
  let on = false;
  for (let i=0;i<xs.length;i++){
    const v = ys[i];
    if (v === null || v === undefined){ on = false; continue; }
    const px = A.X(xs[i]), py = (map||A.Y)(v);
    if (on) x.lineTo(px,py); else { x.moveTo(px,py); on = true; }
  }
  x.stroke();
}

let lastSeq = -1, lastAt = Date.now();
async function tick(){
  let d;
  try { d = await (await fetch("/data", {cache:"no-store"})).json(); }
  catch(e){ setTimeout(tick, 400); return; }
  if (d.seq !== lastSeq){ lastSeq = d.seq; lastAt = Date.now(); }
  NUMS.forEach(n => {
    const v = d[n.f], el = document.getElementById("v_"+n.f);
    el.innerHTML = (v === null || v === undefined) ? "&mdash;"
      : v.toFixed(n.d) + '<span class="u">' + n.u + "</span>";
  });
  const x1=fit(document.getElementById("c1")), c1=document.getElementById("c1");
  let A = frame(x1, c1.clientWidth, c1.clientHeight, CFG.ang[0], CFG.ang[1], "deg");
  line(x1, A, CFG.ref.map(p=>p[0]), CFG.ref.map(p=>p[1]), "#9a9aa4");
  line(x1, A, d.x, d.y_act, "#1f4e8c");

  const c2=document.getElementById("c2"), x2=fit(c2);
  // The gap is ~0.9 deg on an ~85 deg axis -- real, and invisible at the shared scale.
  // So it gets its own right-hand axis, sized from the MEASURED gap (not from panel 3's
  // fit-edge-padded axis, which would leave it using a fifth of the height). The two
  // absolute curves stay honest on the left axis: the fact that they nearly overlap IS
  // the result -- this belt is stiff at this load.
  A = frame(x2, c2.clientWidth, c2.clientHeight, CFG.ang[0], CFG.ang[1],
            "deg (both JOINT-referred)", CFG.gap[0], CFG.gap[1], "gap, deg", "#2c6e49");
  line(x2, A, d.x, d.y_thj, "#1f4e8c");
  line(x2, A, d.x, d.y_thar, "#8c4a1f");
  if (d.y_thar && d.y_thj){
    const gap = d.y_thar.map((v,i) =>
      (v === null || d.y_thj[i] === null) ? null : v - d.y_thj[i]);
    line(x2, A, d.x, gap, "#2c6e49", A.Yr);
  }

  const c3=document.getElementById("c3"), x3=fit(c3);
  A = frame(x3, c3.clientWidth, c3.clientHeight, CFG.ths[0], CFG.ths[1], "rad");
  // the extrapolation band, drawn before the trace so the trace stays readable
  const e = CFG.fit_edge, w3 = c3.clientWidth, h3 = c3.clientHeight;
  x3.fillStyle = "#efe3c8";
  if (CFG.ths[1] > e)  x3.fillRect(PAD.l, A.Y(CFG.ths[1]), w3-PAD.l-PAD.r, A.Y(e)-A.Y(CFG.ths[1]));
  if (CFG.ths[0] < -e) x3.fillRect(PAD.l, A.Y(-e), w3-PAD.l-PAD.r, A.Y(CFG.ths[0])-A.Y(-e));
  line(x3, A, d.x, d.y_ths, "#2c6e49");

  const c4=document.getElementById("c4"), x4=fit(c4);
  A = frame(x4, c4.clientWidth, c4.clientHeight, CFG.tau[0], CFG.tau[1], "N·m",
            CFG.iq[0], CFG.iq[1], "A");
  line(x4, A, d.x, d.y_tau, "#1f4e8c");
  line(x4, A, d.x, d.y_iq, "#9a2b11", A.Yr);

  const stale = Date.now() - lastAt > 2500;
  const st = document.getElementById("status");
  st.textContent = (d.status || "") + (d.nonfinite ? "   [NON-FINITE VALUE SEEN]" : "")
    + (stale ? "   [simulation not advancing — paused, finished, or closed]" : "");
  st.className = "status" + (stale || d.nonfinite ? " stale" : "");
  document.getElementById("foot").textContent = d.foot || "";
  setTimeout(tick, POLL);
}
window.addEventListener("keydown", ev => {
  const k = ev.key === " " ? "space" : ev.key.toLowerCase();
  if (["space","r","escape"].includes(k)){
    ev.preventDefault();
    fetch("/key?k="+encodeURIComponent(k), {cache:"no-store"}).catch(()=>{});
  }
});
window.addEventListener("resize", () => {});
tick();
</script></body></html>
"""


# ===================================================================== web backend
class DrivetrainWebDashboard:
    """Browser tab fed by a pre-encoded JSON snapshot.  Standard library only."""

    kind = "web"

    def __init__(self, meta, static, limits, port=8788, open_browser=True, poll_ms=60):
        self.alive = True
        self.key_cb = None
        self._keys = []
        self.hits = 0

        xs, ys = static
        cfg = dict(
            subject=str(meta["subject"]), trial=str(meta["trial"]),
            ref=[[round(float(a), 3), round(float(b), 3)] for a, b in zip(xs, ys)],
            ang=[float(limits["ang"][0]), float(limits["ang"][1])],
            ths=[float(limits["ths"][0]), float(limits["ths"][1])],
            gap=[float(limits["gap"][0]), float(limits["gap"][1])],
            tau=[float(limits["tau"][0]), float(limits["tau"][1])],
            iq=[float(limits["iq"][0]), float(limits["iq"][1])],
            fit_edge=float(limits["fit_edge"]),
            stance_end=float(limits["stance_end"]))
        self.html = (_PAGE.replace("__CONFIG__", json.dumps(cfg))
                          .replace("__POLL__", str(int(poll_ms)))
                          .replace("__BANNER__", BANNER)
                          .replace("__DISCLAIMER__", DISCLAIMER)).encode("utf-8")
        self._snap = _blank_snapshot()
        self._body = json.dumps(self._snap).encode("utf-8")

        self.srv = self._serve(port)
        self.port = self.srv.server_address[1]
        self.url = f"http://127.0.0.1:{self.port}/"
        threading.Thread(target=self.srv.serve_forever, daemon=True,
                         name="osl-drivetrain-dashboard").start()
        self.opened = bool(open_browser) and self._open()

    # ------------------------------------------------------------------- plumbing
    def _serve(self, port):
        outer = self

        class Handler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"
            server_version = "osl-drivetrain-dashboard"

            def log_message(self, *a):               # keep the demo terminal clean
                pass

            def _send(self, body, ctype, code=200):
                self.send_response(code)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                try:
                    self.wfile.write(body)
                except (BrokenPipeError, ConnectionResetError):
                    pass

            def do_GET(self):
                path = self.path.split("?", 1)[0]
                if path in ("/", "/index.html"):
                    outer.hits += 1
                    self._send(outer.html, "text/html; charset=utf-8")
                elif path == "/data":
                    self._send(outer._body, "application/json")
                elif path == "/key":
                    q = self.path.split("?", 1)[1] if "?" in self.path else ""
                    key = ""
                    for part in q.split("&"):
                        if part.startswith("k="):
                            key = part[2:]
                    outer._keys.append(key)
                    self._send(b"{}", "application/json")
                else:
                    self._send(b"not found", "text/plain", 404)

        return ThreadingHTTPServer(("127.0.0.1", int(port)), Handler)

    def _open(self):
        try:
            return bool(webbrowser.open(self.url))
        except Exception:                                           # noqa: BLE001
            return False

    def wait_ready(self, timeout=6.0):
        """Block until the browser has actually fetched the page, or give up.

        Worth doing: it means the demo starts on a populated page instead of a blank
        one, which is the difference between a presentation that looks ready and one
        that looks broken for the first two seconds.
        """
        end = time.time() + float(timeout)
        while time.time() < end:
            if self.hits:
                return True
            time.sleep(0.05)
        return False

    def bind_keys(self, cb):
        self.key_cb = cb

    def pump(self):
        """Deliver any queued key presses.  Returns False to ask the loop to stop."""
        while self._keys:
            k = self._keys.pop(0)
            if self.key_cb:
                self.key_cb(k)
        return self.alive

    def close(self):
        self.alive = False
        try:
            self.srv.shutdown()
            self.srv.server_close()
        except Exception:                                           # noqa: BLE001
            pass

    # ------------------------------------------------------------------- the data
    def update(self, s):
        """Copy one step's state into the published snapshot.  No arithmetic on physics.

        The only computation here is rounding and decimation for display.  Publishing
        REPLACES the dict rather than mutating it, so the HTTP thread can never read a
        half-written frame; the body is pre-encoded so the browser's poll costs the
        simulation loop nothing beyond this one `json.dumps`.
        """
        if not self.alive:
            return
        bad = False
        snap = dict(seq=self._snap["seq"] + 1,
                    foot=str(s.get("foot", "")), status=str(s.get("status", "")))
        for f in SCALARS:
            snap[f], b = _sf(s.get(f))
            bad = bad or b
        for f in TRACES:
            snap[f], b = _thin(s.get(f, ()))
            bad = bad or b
        snap["nonfinite"] = bad
        self._snap = snap
        self._body = json.dumps(snap).encode("utf-8")

    def clear_traces(self):
        snap = dict(self._snap)
        for f in TRACES:
            snap[f] = []
        self._snap = snap
        self._body = json.dumps(snap).encode("utf-8")


SCALARS = ("phase", "ref", "act", "err", "tau_j", "i_q", "th_a", "th_ad", "th_s",
           "tau_a", "k_s")
TRACES = ("x", "y_act", "y_thj", "y_thar", "y_ths", "y_tau", "y_iq")


def _blank_snapshot():
    snap = dict(seq=0, foot="waiting for the first simulation step...", status="",
                nonfinite=False)
    snap.update({f: None for f in SCALARS})
    snap.update({f: [] for f in TRACES})
    return snap


# ================================================================ terminal backend
class DrivetrainTerminalDashboard:
    """In-terminal readout, for when there is no browser.  Same eleven signals."""

    kind = "terminal"

    HDR = (f"{'phase%':>7} {'ref':>8} {'act':>8} {'err':>8} {'tau_j':>9} "
           f"{'I_q':>8} {'th_a':>9} {'th_a_dot':>9} {'th_s':>10} {'tau_a':>8} "
           f"{'K_s':>8}")

    def __init__(self, meta=None, why="", every=0.10, **_):
        self.alive = True
        self.key_cb = None
        self._n = 0
        self._every = float(every)
        self._last = -1e9
        if why:
            print(f"  [dashboard] {why}")
        print(f"  [dashboard] terminal readout; {BANNER}; {DISCLAIMER}")

    def bind_keys(self, cb):
        self.key_cb = cb

    def wait_ready(self, timeout=0.0):
        return True

    def pump(self):
        return self.alive

    def close(self):
        self.alive = False

    def clear_traces(self):
        self._last = -1e9

    def update(self, s):
        t = time.perf_counter()
        if t - self._last < self._every:
            return
        self._last = t
        if self._n % 20 == 0:
            print("    " + self.HDR)
        self._n += 1

        def g(f, w, d):
            v = s.get(f)
            try:
                v = float(v)
            except (TypeError, ValueError):
                return f"{'--':>{w}}"
            return f"{v:>{w}.{d}f}" if math.isfinite(v) else f"{'nan':>{w}}"

        print("    " + " ".join((
            g("phase", 7, 2), g("ref", 8, 3), g("act", 8, 3), g("err", 8, 3),
            g("tau_j", 9, 3), g("i_q", 8, 3), g("th_a", 9, 4), g("th_ad", 9, 3),
            g("th_s", 10, 6), g("tau_a", 8, 3), g("k_s", 8, 1))))


class NoDashboard:
    """No second window at all.  Every method is a no-op; the physics is unchanged."""

    kind = "none"
    alive = True

    def bind_keys(self, cb):
        pass

    def wait_ready(self, timeout=0.0):
        return True

    def pump(self):
        return True

    def close(self):
        pass

    def update(self, s):
        pass

    def clear_traces(self):
        pass


def make_drivetrain_dashboard(meta, static, limits, force=None, port=8788,
                              open_browser=True, every=0.10):
    """Build the second window: web -> terminal.

    `every` throttles the TERMINAL backend only (seconds between printed rows); pass 0.0
    to print every update, which is what the headless path wants since it drives updates
    on a phase cadence rather than in real time.

    Every backend that is skipped or fails prints WHY, on its own line.  That is
    deliberate and was learned the hard way on the servo demo: a soft one-line demotion
    is exactly how a demo ends up with no second window five minutes before a lab
    meeting.
    """
    force = (force or "auto").lower()
    if force not in BACKENDS:
        raise ValueError(f"dashboard backend must be one of {BACKENDS}, got {force!r}")
    if force == "none":
        return NoDashboard()

    notes = []
    if force in ("auto", "web"):
        try:
            d = DrivetrainWebDashboard(meta, static, limits, port=port,
                                       open_browser=open_browser)
            print(f"  [dashboard] browser window at {d.url}"
                  + ("" if d.opened else "   <- open this URL manually"))
            return d
        except Exception as exc:                                    # noqa: BLE001
            notes.append(f"web: {type(exc).__name__}: {exc}")
            if force == "web":
                raise

    for nt in notes:
        print(f"  [dashboard] unavailable, {nt}")
    return DrivetrainTerminalDashboard(meta, why="falling back to the terminal readout",
                                       every=every)
