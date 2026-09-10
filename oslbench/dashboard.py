#!/usr/bin/env python3
"""
oslbench/dashboard.py -- PRESENTATION ONLY.  The live demo's second window.

This file contains NO physics, NO controller and NO simulation.  It draws numbers and
curves that oslbench/viewer.py hands it after each MuJoCo step.  Nothing here can change
what the simulation does; delete this file and the experiment is unaffected.

WHY A BROWSER WINDOW IS THE DEFAULT
    First attempt used tkinter.  On this machine `import tkinter` (or Tk() creation)
    raises inside .venv, so make_dashboard() silently fell back to the terminal readout
    and no second window ever appeared.  That is now fixed two ways: the default backend
    no longer depends on a GUI toolkit at all, and every backend that fails says so on
    its own line instead of degrading quietly.

    The default WebDashboard uses only http.server, json, threading and webbrowser --
    all Python standard library, all present in any interpreter that can run mujoco.  It
    serves ONE self-contained page on 127.0.0.1 with no CDN and no internet access
    required, and the browser polls a JSON snapshot published by the simulation loop.
    matplotlib is deliberately not used: it is absent from .venv (it lives only in
    .venv-analysis, which has no mujoco), and .venv must not be modified.  A browser
    also draws in its OWN process, so dashboard redraw cannot slow the physics.

    Backends are tried in order (web -> tk -> terminal) and the choice is reported.

DELIBERATELY NOT SHOWN
    human_knee_moment and human_knee_power from the AB19 CSV.  Those two columns are
    known-defective (see oslbench.reference.audit_reference §R4) and must not appear in a
    presentation.  This module is never given them.
"""

from __future__ import annotations

import json
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

# ------------------------------------------------------------------ palette (light)
BG = "#f4f5f7"
CARD = "#ffffff"
INK = "#16181d"
MUTED = "#6a707c"
LINE = "#d8dbe0"
SHADE = "#eceff4"
C_REF = "#98a2b3"      # human reference knee angle
C_ACT = "#1266d4"      # simulated OSL knee angle
C_ERR = "#c9364b"      # tracking error
C_TAU = "#6b46d6"      # knee torque
C_MARK = "#101319"     # current-position marker

MONO = ("Consolas", 11)
MONO_B = ("Consolas", 13, "bold")
SANS = ("Segoe UI", 10)
SANS_T = ("Segoe UI", 15, "bold")
SANS_S = ("Segoe UI", 10)

# the eight fields required on the live display, in order
FIELDS = ("Subject", "Trial", "Gait phase", "Reference knee angle", "Actual knee angle",
          "Tracking error", "Knee torque", "Torque authority")


# ===================================================================== one plot panel
class _Plot:
    """A single strip chart on its own Canvas.  x is ALWAYS % gait cycle, 0..100.

    Draws once at construction: frame, gridlines, axis labels, the optional stance
    shading and the optional static reference curve.  Per frame it only moves three
    existing canvas items (live polyline, phase marker, current-point dot) with
    canvas.coords(), which is far cheaper and flicker-free compared with redrawing.
    """

    def __init__(self, parent, tk, title, colour, ylo, yhi, width, height,
                 static=None, static_colour=C_REF, zero_line=False, stance_end=None,
                 xlabel=False):
        self.tk = tk
        self.ylo, self.yhi = float(ylo), float(yhi)
        self.w, self.h = width, height
        self.pl, self.pr, self.pt, self.pb = 54, 12, 20, (26 if xlabel else 12)
        self.canvas = tk.Canvas(parent, width=width, height=height, bg=CARD,
                                highlightthickness=1, highlightbackground=LINE)
        c = self.canvas

        x0, x1 = self.pl, self.w - self.pr
        y0, y1 = self.pt, self.h - self.pb

        # stance shading -- a property of the HUMAN gait cycle, labelled as such
        if stance_end is not None:
            c.create_rectangle(x0, y0, self._px(stance_end), y1,
                               fill=SHADE, outline="")

        for xp in (0, 25, 50, 75, 100):                       # x grid + ticks
            xx = self._px(xp)
            c.create_line(xx, y0, xx, y1, fill=LINE)
            if xlabel:
                c.create_text(xx, y1 + 12, text=str(xp), fill=MUTED, font=SANS_S)
        for yv in (self.ylo, 0.5 * (self.ylo + self.yhi), self.yhi):
            yy = self._py(yv)
            c.create_line(x0, yy, x1, yy, fill=LINE)
            c.create_text(x0 - 6, yy, text=f"{yv:g}", anchor="e", fill=MUTED, font=SANS_S)
        if zero_line and self.ylo < 0.0 < self.yhi:
            c.create_line(x0, self._py(0.0), x1, self._py(0.0), fill="#aab0ba")

        c.create_text(x0, 10, text=title, anchor="w", fill=INK, font=SANS)
        if xlabel:
            c.create_text(x1, self.h - 6, text="% gait cycle", anchor="e",
                          fill=MUTED, font=SANS_S)

        # static reference curve (drawn once -- it never changes)
        if static is not None:
            xs, ys = static
            pts = []
            for a, b in zip(xs, ys):
                pts += [self._px(a), self._py(b)]
            if len(pts) >= 4:
                c.create_line(*pts, fill=static_colour, width=3, smooth=True)

        # The three live items, seeded off-canvas so coords() always has something.
        # smooth=False on purpose: the live trace is redrawn ~60x/s and the plotted
        # samples are already dense, so bezier smoothing would only cost frame time.
        self.trace = c.create_line(-9, -9, -8, -8, fill=colour, width=2)
        self.mark = c.create_line(-9, y0, -9, y1, fill=C_MARK, width=1)
        self.dot = c.create_oval(-9, -9, -5, -5, fill=colour, outline=CARD, width=2)

    def _px(self, phase):
        return self.pl + (self.w - self.pl - self.pr) * max(0.0, min(100.0, phase)) / 100.0

    def _py(self, v):
        f = (float(v) - self.ylo) / (self.yhi - self.ylo) if self.yhi > self.ylo else 0.5
        f = max(0.0, min(1.0, f))
        return (self.h - self.pb) - f * (self.h - self.pt - self.pb)

    def set_trace(self, xs, ys):
        n = len(xs)
        if n == 0:
            self.canvas.coords(self.trace, -9, -9, -8, -8)
            return
        pts = []
        for i in range(n):
            pts += [self._px(xs[i]), self._py(ys[i])]
        if n == 1:                       # tkinter needs >= 2 points for a line
            pts += pts
        self.canvas.coords(self.trace, *pts)

    def set_marker(self, phase, value):
        x = self._px(phase)
        self.canvas.coords(self.mark, x, self.pt, x, self.h - self.pb)
        y = self._py(value)
        self.canvas.coords(self.dot, x - 4, y - 4, x + 4, y + 4)


# ======================================================================= tk dashboard
class Dashboard:
    """Second lightweight window: eight readouts on the left, three strip charts on the
    right.  Every value comes from the same MuJoCo step as the frame you are watching --
    oslbench/viewer.py passes in exactly what it read out of data.sensordata."""

    kind = "tk"

    def __init__(self, tk, title, subtitle, meta, static, limits, geometry=None):
        self.tk = tk
        self.key_cb = None
        self.alive = True
        self.root = tk.Tk()
        self.root.title("OSL V2 bench -- live tracking dashboard")
        self.root.configure(bg=BG)
        self.root.protocol("WM_DELETE_WINDOW", self._closed)
        self.root.bind("<KeyPress>", self._on_key)
        if geometry:
            self.root.geometry(geometry)

        head = tk.Frame(self.root, bg=BG)
        head.pack(fill="x", padx=12, pady=(10, 4))
        tk.Label(head, text=title, font=SANS_T, fg=INK, bg=BG, anchor="w").pack(fill="x")
        tk.Label(head, text=subtitle, font=SANS, fg=C_ERR, bg=BG,
                 anchor="w").pack(fill="x")

        body = tk.Frame(self.root, bg=BG)
        body.pack(fill="both", expand=True, padx=12, pady=4)

        # ---------------------------------------------------------- left: the readouts
        left = tk.Frame(body, bg=CARD, highlightthickness=1, highlightbackground=LINE)
        left.pack(side="left", fill="y", padx=(0, 10))
        self.val = {}
        for r, name in enumerate(FIELDS):
            tk.Label(left, text=name, font=SANS, fg=MUTED, bg=CARD, anchor="w"
                     ).grid(row=r, column=0, sticky="w", padx=(12, 8), pady=3)
            lab = tk.Label(left, text="--", font=(MONO_B if r >= 2 else MONO),
                           fg=INK, bg=CARD, anchor="e", width=18)
            lab.grid(row=r, column=1, sticky="e", padx=(0, 12), pady=3)
            self.val[name] = lab
        self.val["Subject"].config(text=meta["subject"])
        self.val["Trial"].config(text=meta["trial"], font=MONO)

        tk.Frame(left, bg=LINE, height=1).grid(row=len(FIELDS), column=0, columnspan=2,
                                               sticky="ew", pady=(8, 6))
        self.foot = tk.Label(left, text="", font=SANS_S, fg=MUTED, bg=CARD,
                             justify="left", anchor="w")
        self.foot.grid(row=len(FIELDS) + 1, column=0, columnspan=2, sticky="w",
                       padx=12, pady=(0, 10))

        # --------------------------------------------------------- right: the 3 plots
        right = tk.Frame(body, bg=BG)
        right.pack(side="left", fill="both", expand=True)
        w, h = 620, 132
        self.p_ang = _Plot(right, tk, "knee angle  [deg]   grey = AB19 human reference,"
                                      "  blue = simulated OSL", C_ACT,
                           limits["ang"][0], limits["ang"][1], w, h,
                           static=static, stance_end=limits["stance_end"])
        self.p_ang.canvas.pack(pady=(0, 6))
        self.p_err = _Plot(right, tk, "tracking error  [deg]   simulated - reference",
                           C_ERR, limits["err"][0], limits["err"][1], w, h,
                           zero_line=True, stance_end=limits["stance_end"])
        self.p_err.canvas.pack(pady=(0, 6))
        self.p_tau = _Plot(right, tk, "knee actuator torque  [N.m]   bench torque, NOT a "
                                      "human knee moment", C_TAU,
                           limits["tau"][0], limits["tau"][1], w, h,
                           zero_line=True, stance_end=limits["stance_end"], xlabel=True)
        self.p_tau.canvas.pack()

        self.status = tk.Label(self.root, text="", font=SANS_S, fg=MUTED, bg=BG,
                               anchor="w")
        self.status.pack(fill="x", padx=12, pady=(2, 8))
        self.root.update()

    # ------------------------------------------------------------------------ plumbing
    def _closed(self):
        self.alive = False

    def _on_key(self, ev):
        if self.key_cb:
            self.key_cb(getattr(ev, "keysym", ""))

    def bind_keys(self, cb):
        self.key_cb = cb

    def wait_ready(self, timeout=0.0):
        return True

    def pump(self):
        """Service the Tk event loop from OUR loop -- we never call mainloop(), so the
        simulation keeps ownership of timing.  Returns False once the window is gone."""
        if not self.alive:
            return False
        try:
            self.root.update()
        except Exception:
            self.alive = False
        return self.alive

    def close(self):
        if self.alive:
            try:
                self.root.destroy()
            except Exception:
                pass
        self.alive = False

    # -------------------------------------------------------------------- per-frame
    def update(self, s):
        if not self.alive:
            return
        v = self.val
        v["Gait phase"].config(text=f"{s['phase']:7.1f} %")
        v["Reference knee angle"].config(text=f"{s['ref_deg']:7.1f} deg")
        v["Actual knee angle"].config(text=f"{s['act_deg']:7.1f} deg")
        v["Tracking error"].config(text=f"{s['err_deg']:+7.1f} deg",
                                   fg=(C_ERR if abs(s["err_deg"]) > 5.0 else INK))
        v["Knee torque"].config(text=f"{s['tau']:+7.1f} N.m")
        v["Torque authority"].config(text=f"{s['auth']:7.1f} %",
                                     fg=(C_ERR if s["auth"] > 90.0 else INK))
        self.foot.config(text=s["foot"])
        self.status.config(text=s["status"])
        self.p_ang.set_trace(s["x"], s["y_ang"])
        self.p_err.set_trace(s["x"], s["y_err"])
        self.p_tau.set_trace(s["x"], s["y_tau"])
        self.p_ang.set_marker(s["phase"], s["act_deg"])
        self.p_err.set_marker(s["phase"], s["err_deg"])
        self.p_tau.set_marker(s["phase"], s["tau"])

    def clear_traces(self):
        for p in (self.p_ang, self.p_err, self.p_tau):
            p.set_trace([], [])


# ======================================================================= web dashboard
_PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<title>OSL V2 bench - live tracking dashboard</title>
<style>
 :root { color-scheme: light }
 html,body { margin:0; background:#f4f5f7; color:#16181d;
             font:13px/1.35 "Segoe UI",system-ui,sans-serif; }
 #wrap { padding:14px 16px 18px; }
 h1 { font-size:19px; margin:0 0 2px; letter-spacing:.2px; }
 .sub { color:#c9364b; font-size:12.5px; margin:0 0 12px; }
 .cols { display:flex; gap:14px; align-items:flex-start; flex-wrap:wrap; }
 .card { background:#fff; border:1px solid #d8dbe0; border-radius:6px; }
 #read { padding:12px 14px; min-width:310px; flex:0 0 auto; }
 #read table { border-collapse:collapse; width:100%; }
 #read td { padding:4px 0; white-space:nowrap; }
 #read td.k { color:#6a707c; padding-right:16px; }
 #read td.v { text-align:right; font:13px/1.2 Consolas,ui-monospace,monospace; }
 #read tr.big td.v { font-size:16px; font-weight:700; }
 #read td.v.warn { color:#c9364b; }
 .rule { border-top:1px solid #d8dbe0; margin:9px 0 8px; }
 #foot { color:#6a707c; font-size:11.5px; white-space:pre-line; }
 #plots { flex:1 1 640px; min-width:520px; display:flex; flex-direction:column; gap:8px; }
 .pw { padding:0; overflow:hidden; }
 canvas { display:block; width:100%; height:150px; }
 #bar { margin-top:10px; color:#6a707c; font-size:11.5px; }
 #dead { display:none; margin-top:10px; padding:7px 10px; border-radius:5px;
         background:#fdecef; border:1px solid #f3c3cb; color:#c9364b; font-weight:600; }
</style></head><body><div id="wrap">
<h1 id="t"></h1><p class="sub" id="s"></p>
<div class="cols">
  <div class="card" id="read">
    <table><tbody id="tb"></tbody></table>
    <div class="rule"></div><div id="foot"></div>
  </div>
  <div id="plots">
    <div class="card pw"><canvas id="c0"></canvas></div>
    <div class="card pw"><canvas id="c1"></canvas></div>
    <div class="card pw"><canvas id="c2"></canvas></div>
  </div>
</div>
<div id="bar"></div><div id="dead"></div>
</div><script>
const CFG = __CONFIG__;
const C = {ref:"#98a2b3", act:"#1266d4", err:"#c9364b", tau:"#6b46d6",
           mark:"#101319", line:"#d8dbe0", shade:"#eceff4", muted:"#6a707c",
           ink:"#16181d", card:"#ffffff"};
document.getElementById("t").textContent = CFG.title;
document.getElementById("s").textContent = CFG.subtitle;

/* ---- the eight required readouts, in order ---- */
const ROWS = [["Subject","subject"],["Trial","trial"],["Gait phase","phase"],
              ["Reference knee angle","ref"],["Actual knee angle","act"],
              ["Tracking error","err"],["Knee torque","tau"],
              ["Torque authority","auth"]];
const cell = {};
const tb = document.getElementById("tb");
for (const [label,key] of ROWS) {
  const tr = document.createElement("tr");
  if (key!=="subject" && key!=="trial") tr.className = "big";
  const a = document.createElement("td"); a.className="k"; a.textContent=label;
  const b = document.createElement("td"); b.className="v"; b.textContent="--";
  tr.appendChild(a); tr.appendChild(b); tb.appendChild(tr); cell[key]=b;
}
cell.subject.textContent = CFG.subject;
cell.trial.textContent = CFG.trial;
cell.trial.style.fontSize = "12px";

const PANELS = [
  {cv:"c0", lo:CFG.ang[0], hi:CFG.ang[1], col:C.act, zero:false, xlab:false,
   ref:CFG.ref, tk:"y_ang", mk:"act",
   title:"KNEE ANGLE  [deg]      grey = AB19 human reference      blue = simulated OSL V2"},
  {cv:"c1", lo:CFG.err[0], hi:CFG.err[1], col:C.err, zero:true, xlab:false,
   ref:null, tk:"y_err", mk:"err",
   title:"TRACKING ERROR  [deg]      simulated - reference"},
  {cv:"c2", lo:CFG.tau[0], hi:CFG.tau[1], col:C.tau, zero:true, xlab:true,
   ref:null, tk:"y_tau", mk:"tau",
   title:"BENCH ACTUATOR TORQUE  [N.m]      measured on the simulated actuator; "
         +"NOT a human knee moment"}];

let S = null;
function fit(p){
  const cv=document.getElementById(p.cv), r=cv.getBoundingClientRect();
  const dpr=window.devicePixelRatio||1;
  cv.width=Math.max(320,Math.round(r.width*dpr));
  cv.height=Math.round(150*dpr);
  p.g=cv.getContext("2d"); p.g.setTransform(dpr,0,0,dpr,0,0);
  p.w=cv.width/dpr; p.h=cv.height/dpr;
  p.L=58; p.R=14; p.T=22; p.B=p.xlab?26:12;
}
const px=(p,x)=>p.L+(p.w-p.L-p.R)*Math.min(100,Math.max(0,x))/100;
const py=(p,v)=>{const f=Math.min(1,Math.max(0,(v-p.lo)/(p.hi-p.lo)));
                 return (p.h-p.B)-f*(p.h-p.T-p.B);};
function poly(g,p,xs,ys,col,w){
  g.strokeStyle=col; g.lineWidth=w; g.lineJoin="round"; g.beginPath();
  let up=false;
  for(let i=0;i<xs.length;i++){
    const x=xs[i],y=ys[i];
    if(x==null||y==null||!isFinite(x)||!isFinite(y)){up=false;continue;}
    const X=px(p,x),Y=py(p,y);
    if(up) g.lineTo(X,Y); else {g.moveTo(X,Y); up=true;}
  }
  g.stroke();
}
function draw(p){
  const g=p.g, x0=p.L, x1=p.w-p.R, y0=p.T, y1=p.h-p.B;
  g.clearRect(0,0,p.w,p.h); g.fillStyle=C.card; g.fillRect(0,0,p.w,p.h);
  g.fillStyle=C.shade; g.fillRect(x0,y0,px(p,CFG.stance_end)-x0,y1-y0);
  g.strokeStyle=C.line; g.lineWidth=1;
  g.font="10.5px 'Segoe UI',sans-serif"; g.textBaseline="middle";
  for(const t of [0,25,50,75,100]){const X=Math.round(px(p,t))+.5;
    g.beginPath();g.moveTo(X,y0);g.lineTo(X,y1);g.stroke();
    if(p.xlab){g.fillStyle=C.muted;g.textAlign="center";g.fillText(String(t),X,y1+13);} }
  for(const v of [p.lo,(p.lo+p.hi)/2,p.hi]){const Y=Math.round(py(p,v))+.5;
    g.strokeStyle=C.line;g.beginPath();g.moveTo(x0,Y);g.lineTo(x1,Y);g.stroke();
    g.fillStyle=C.muted;g.textAlign="right";g.fillText(String(v),x0-7,Y);}
  if(p.zero&&p.lo<0&&p.hi>0){g.strokeStyle="#aab0ba";const Y=Math.round(py(p,0))+.5;
    g.beginPath();g.moveTo(x0,Y);g.lineTo(x1,Y);g.stroke();}
  g.fillStyle=C.ink;g.textAlign="left";g.font="11px 'Segoe UI',sans-serif";
  g.fillText(p.title,x0,11);
  if(p.xlab){g.fillStyle=C.muted;g.textAlign="right";
             g.fillText("% gait cycle",x1,p.h-7);}
  if(p.ref) poly(g,p,p.ref.map(a=>a[0]),p.ref.map(a=>a[1]),C.ref,3);
  if(!S) return;
  poly(g,p,S.x,S[p.tk],p.col,2);
  const ph=S.phase, mv=S[p.mk];
  if(ph!=null){const X=Math.round(px(p,ph))+.5;
    g.strokeStyle=C.mark;g.lineWidth=1;g.beginPath();
    g.moveTo(X,y0);g.lineTo(X,y1);g.stroke();
    if(mv!=null&&isFinite(mv)){g.fillStyle=p.col;g.beginPath();
      g.arc(px(p,ph),py(p,mv),4.5,0,6.2832);g.fill();
      g.strokeStyle=C.card;g.lineWidth=2;g.stroke();}}
}
function redraw(){for(const p of PANELS) draw(p);}
function sizeAll(){for(const p of PANELS) fit(p); redraw();}
window.addEventListener("resize",sizeAll);
const f1=(v,u)=>v==null?"--":v.toFixed(1)+u;
function paint(){
  cell.phase.textContent = f1(S.phase," %");
  cell.ref.textContent   = f1(S.ref," deg");
  cell.act.textContent   = f1(S.act," deg");
  cell.err.textContent   = S.err==null?"--":(S.err>=0?"+":"")+S.err.toFixed(1)+" deg";
  cell.tau.textContent   = S.tau==null?"--":(S.tau>=0?"+":"")+S.tau.toFixed(1)+" N.m";
  cell.auth.textContent  = f1(S.auth," %");
  cell.err.className  = "v"+(S.err!=null&&Math.abs(S.err)>5?" warn":"");
  cell.auth.className = "v"+(S.auth!=null&&S.auth>90?" warn":"");
  document.getElementById("foot").textContent = S.foot||"";
  document.getElementById("bar").textContent  = S.status||"";
  const dead=document.getElementById("dead");
  if(S.ended){dead.style.display="block";
    dead.textContent="SIMULATION ENDED - this page is no longer live.";}
  if(S.nonfinite){dead.style.display="block";
    dead.textContent="NON-FINITE VALUES REPORTED BY THE SIMULATION.";}
  redraw();
}
async function tick(){
  try{
    const r=await fetch("state",{cache:"no-store"});
    if(r.ok){S=await r.json(); paint();}
  }catch(e){}
  setTimeout(tick,__POLL__);
}
document.addEventListener("keydown",ev=>{
  const k={" ":"space","Spacebar":"space","r":"r","R":"r","Escape":"escape"}[ev.key];
  if(!k) return;
  ev.preventDefault();
  fetch("key?k="+k,{cache:"no-store"}).catch(()=>{});
});
sizeAll(); tick();
</script></body></html>
"""


def _thin(a):
    """Sanitised, rounded plain-Python list for the JSON payload.  Non-finite values are
    replaced and flagged rather than emitted, because NaN is not legal JSON and would
    silently freeze the page instead of showing that the simulation went bad."""
    v = np.asarray(a, dtype=float)
    if v.size == 0:
        return [], False
    good = np.isfinite(v)
    if not good.all():
        v = np.where(good, v, 0.0)
        return np.round(v, 3).tolist(), True
    return np.round(v, 3).tolist(), False


def _sf(v):
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None, True
    return (f, False) if f == f and abs(f) != float("inf") else (None, True)


class WebDashboard:
    """Second window as a browser tab.  Standard library only: http.server serves ONE
    self-contained page on 127.0.0.1 (no CDN, no internet), and the page polls a JSON
    snapshot that the simulation loop publishes.

    It runs no physics and holds no model.  update() is handed exactly what
    oslbench/viewer.py just read out of data.sensordata on the current step; the numbers
    are copied into a dict, and the HTTP thread serialises that dict when the browser
    asks.  Two consequences worth stating for the presentation:
      * the page CANNOT show anything the simulation did not produce -- there is no second
        integrator, no interpolation and no smoothing anywhere in this file;
      * drawing happens in the browser's process, so dashboard redraw cannot slow the
        physics loop, and the window is opened once and never raised again, so it cannot
        keep stealing focus.
    """

    kind = "web"

    def __init__(self, title, subtitle, meta, static, limits, port=8787,
                 open_browser=True, poll_ms=60):
        self.alive = True
        self.key_cb = None
        self._keys = []
        self.hits = 0
        self._t0 = time.time()

        xs, ys = static
        cfg = dict(
            title=title, subtitle=subtitle,
            subject=str(meta["subject"]), trial=str(meta["trial"]),
            ref=[[round(float(a), 3), round(float(b), 3)] for a, b in zip(xs, ys)],
            ang=[float(limits["ang"][0]), float(limits["ang"][1])],
            err=[float(limits["err"][0]), float(limits["err"][1])],
            tau=[float(limits["tau"][0]), float(limits["tau"][1])],
            stance_end=float(limits["stance_end"]))
        self.html = (_PAGE.replace("__CONFIG__", json.dumps(cfg))
                          .replace("__POLL__", str(int(poll_ms)))).encode("utf-8")
        self._snap = dict(seq=0, phase=None, ref=None, act=None, err=None, tau=None,
                          auth=None, x=[], y_ang=[], y_err=[], y_tau=[],
                          foot="waiting for the first simulation step...",
                          status="", ended=False, nonfinite=False)
        self._body = json.dumps(self._snap).encode("utf-8")

        self.srv = self._serve(port)
        self.port = self.srv.server_address[1]
        self.url = f"http://127.0.0.1:{self.port}/"
        threading.Thread(target=self.srv.serve_forever, daemon=True,
                         name="dashboard-http").start()
        self.opened = bool(open_browser) and self._open()

    # ---------------------------------------------------------------------- plumbing
    def _serve(self, port):
        outer = self

        class Handler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"
            server_version = "osl-bench-dashboard"

            def log_message(self, *a):           # keep the demo terminal clean
                pass

            def _send(self, body, ctype, code=200):
                self.send_response(code)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                path = self.path.split("?", 1)[0].rstrip("/") or "/"
                if path in ("/", "/index.html"):
                    outer.hits += 1
                    self._send(outer.html, "text/html; charset=utf-8")
                elif path == "/state":
                    outer.hits += 1
                    self._send(outer._body, "application/json")
                elif path == "/key":
                    q = self.path.split("?", 1)[1] if "?" in self.path else ""
                    k = ""
                    for part in q.split("&"):
                        if part.startswith("k="):
                            k = part[2:]
                    if k in ("space", "r", "escape"):
                        outer._keys.append(k)
                        self._send(b"ok", "text/plain")
                    else:
                        # 200 for an unrecognised key would hide a typo in the page's
                        # keymap; a 204 would too (an empty body still reads as success).
                        self._send(b"unknown key", "text/plain", 400)
                else:
                    self._send(b"not found", "text/plain", 404)

        last = None
        for p in ([int(port)] if port else []) + [0]:
            try:
                return ThreadingHTTPServer(("127.0.0.1", p), Handler)
            except OSError as exc:                                 # port already in use
                last = exc
        raise last

    def _open(self):
        try:
            return bool(webbrowser.open(self.url))
        except Exception:                                          # pragma: no cover
            return False

    def wait_ready(self, timeout=6.0):
        """Give the browser a moment to make its first request, so the presentation
        starts with a populated page instead of an empty one.  Never blocks the physics
        for longer than `timeout`, and the demo runs fine if it returns False."""
        end = time.time() + max(0.0, timeout)
        while time.time() < end:
            if self.hits:
                return True
            time.sleep(0.05)
        return bool(self.hits)

    def bind_keys(self, cb):
        self.key_cb = cb

    def pump(self):
        """Deliver keys pressed in the browser into the SAME callback the MuJoCo window
        uses, from the simulation thread, between steps."""
        while self._keys:
            k = self._keys.pop(0)
            if self.key_cb:
                self.key_cb(k)
        return self.alive

    def close(self):
        if not self.alive:
            return
        self.alive = False
        snap = dict(self._snap)
        snap["ended"] = True
        snap["status"] = "simulation ended"
        self._snap = snap
        self._body = json.dumps(snap).encode("utf-8")
        time.sleep(0.9 if self.hits else 0.0)     # let the page show ENDED, then stop
        try:
            self.srv.shutdown()
            self.srv.server_close()
        except Exception:                                          # pragma: no cover
            pass

    # ------------------------------------------------------------------- per-frame
    def update(self, s):
        if not self.alive:
            return
        bad = False
        vals = {}
        for dst, src in (("phase", "phase"), ("ref", "ref_deg"), ("act", "act_deg"),
                         ("err", "err_deg"), ("tau", "tau"), ("auth", "auth")):
            vals[dst], b = _sf(s[src])
            bad = bad or b
        snap = dict(seq=self._snap["seq"] + 1, ended=False,
                    foot=str(s["foot"]), status=str(s["status"]), **vals)
        for dst, src in (("x", "x"), ("y_ang", "y_ang"), ("y_err", "y_err"),
                         ("y_tau", "y_tau")):
            snap[dst], b = _thin(s[src])
            bad = bad or b
        snap["nonfinite"] = bad
        # Publish by REPLACING the dict, so an HTTP thread can never read a half-written
        # frame; the pre-encoded body means the browser's poll costs the sim nothing.
        self._snap = snap
        self._body = json.dumps(snap).encode("utf-8")

    def clear_traces(self):
        snap = dict(self._snap)
        snap["x"] = snap["y_ang"] = snap["y_err"] = snap["y_tau"] = []
        self._snap = snap
        self._body = json.dumps(snap).encode("utf-8")


# ================================================================== terminal fallback
class TerminalDashboard:
    """Same interface, no window: rewrites one status line in place.  Last resort, used
    only when neither a browser nor tkinter could be started, so a missing GUI can never
    stop the demo."""

    kind = "terminal"

    def __init__(self, meta, why="", **_):
        self.alive = True
        self.meta = meta
        print(f"  [dashboard] NO SECOND WINDOW -- falling back to the terminal readout."
              + (f"  {why}" if why else ""))
        print(f"  Subject: {meta['subject']}    Trial: {meta['trial']}")

    def bind_keys(self, cb):
        pass

    def wait_ready(self, timeout=0.0):
        return True

    def pump(self):
        return True

    def close(self):
        print()

    def clear_traces(self):
        pass

    def update(self, s):
        print(f"\r  phase {s['phase']:6.1f} % | ref {s['ref_deg']:6.1f} | "
              f"act {s['act_deg']:6.1f} | err {s['err_deg']:+6.1f} deg | "
              f"tau {s['tau']:+7.1f} N.m | authority {s['auth']:5.1f} %   ",
              end="", flush=True)


# ============================================================================ factory
BACKENDS = ("auto", "web", "tk", "terminal", "none")


class NoDashboard:
    """--dashboard none.  Explicitly asked for, so it says nothing."""

    kind = "none"
    alive = True
    url = None

    def __init__(self, *a, **kw):
        pass

    def bind_keys(self, cb):
        pass

    def wait_ready(self, timeout=0.0):
        return True

    def pump(self):
        return True

    def close(self):
        pass

    def clear_traces(self):
        pass

    def update(self, s):
        pass


def make_dashboard(title, subtitle, meta, static, limits, geometry=None, force=None,
                   port=8787, open_browser=True):
    """Build the second window.  Order is web -> tk -> terminal.

    `force` is one of BACKENDS; 'auto' / None means try them in order.  Every backend
    that is skipped or that fails prints WHY, on its own line.  That is deliberate: the
    first version of this file caught the tkinter failure, printed one soft note and
    demoted to the terminal readout, which is exactly how a demo ends up with no second
    window five minutes before a presentation.
    """
    force = (force or "auto").lower()
    if force not in BACKENDS:
        raise ValueError(f"dashboard backend must be one of {BACKENDS}, got {force!r}")
    if force == "none":
        return NoDashboard()

    notes = []
    if force in ("auto", "web"):
        try:
            d = WebDashboard(title, subtitle, meta, static, limits, port=port,
                             open_browser=open_browser)
            print(f"  [dashboard] browser window at {d.url}"
                  + ("" if d.opened else "  <- open this URL manually"))
            return d
        except Exception as exc:
            notes.append(f"web: {type(exc).__name__}: {exc}")
            if force == "web":
                raise

    if force in ("auto", "tk"):
        try:
            import tkinter as tk
            d = Dashboard(tk, title, subtitle, meta, static, limits, geometry)
            print("  [dashboard] tkinter window opened")
            return d
        except Exception as exc:
            notes.append(f"tk: {type(exc).__name__}: {exc}")
            if force == "tk":
                raise

    for nt in notes:
        print(f"  [dashboard] backend unavailable -- {nt}")
    return TerminalDashboard(
        meta, why=("Asked for with --dashboard terminal." if force == "terminal"
                   else "Run --dashboard web to see the real error, or "
                        "`python -m oslbench.dashboard` to test the window alone."))


# ===================================================================== manual self-test
def _selftest(port=8787, seconds=90.0, open_browser=True, force="auto"):
    """`python -m oslbench.dashboard` -- opens the dashboard on a synthetic
    trajectory, with NO mujoco and NO model, purely to prove the second window works on
    this machine before the presentation.  Prints every key the page sends back."""
    ph = np.arange(0.0, 100.0, 0.5)
    ref = 20.0 + 24.0 * np.sin(np.radians(3.6 * ph)) + 24.0 * np.sin(np.radians(1.8 * ph))
    limits = dict(ang=(-10.0, 80.0), err=(-15.0, 15.0), tau=(-25.0, 25.0),
                  stance_end=60.0)
    d = make_dashboard("SELF-TEST -- synthetic data, NOT the OSL bench experiment",
                       "If you can read this in a browser, the demo dashboard will work.",
                       dict(subject="--", trial="self-test, no simulation"),
                       (ph, ref), limits, force=force, port=port,
                       open_browser=open_browser)
    d.bind_keys(lambda k: print(f"  key from the page: {k}"))
    print(f"  backend {d.kind}; self-test running for {seconds:g} s, Ctrl-C to stop.")
    if hasattr(d, "wait_ready"):
        print(f"  page fetched: {d.wait_ready(10.0)}")
    grad = 10.0 * np.gradient(ref)
    act = ref - 2.0 * np.gradient(ref)
    t0 = time.time()
    try:
        while time.time() - t0 < seconds:
            i = int((time.time() - t0) % 1.2 / 1.2 * len(ph))
            d.update(dict(phase=float(ph[i]), ref_deg=float(ref[i]),
                          act_deg=float(act[i]), err_deg=float(act[i] - ref[i]),
                          tau=float(grad[i]),
                          auth=abs(float(grad[i])) / 142.2 * 100.0,
                          x=ph[:i], y_ang=act[:i], y_err=(act - ref)[:i], y_tau=grad[:i],
                          foot="SELF-TEST\nno model, no controller, no MuJoCo",
                          status=f"self-test  t {time.time() - t0:5.1f} s"))
            d.pump()
            time.sleep(1.0 / 60.0)
    except KeyboardInterrupt:
        print("\n  stopped.")
    d.close()
    return 0


if __name__ == "__main__":
    import argparse
    import sys

    p = argparse.ArgumentParser(description="Open the live demo's dashboard on synthetic data "
                                            "to check it works on this machine.")
    p.add_argument("--dashboard", choices=BACKENDS, default="auto")
    p.add_argument("--port", type=int, default=8787)
    p.add_argument("--seconds", type=float, default=90.0)
    p.add_argument("--no-browser", action="store_true")
    a = p.parse_args()
    sys.exit(_selftest(port=a.port, seconds=a.seconds, open_browser=not a.no_browser,
                       force=a.dashboard))
