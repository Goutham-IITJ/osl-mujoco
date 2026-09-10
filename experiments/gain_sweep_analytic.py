#!/usr/bin/env python3
"""
gain_sweep_analytic.py -- closed-form / numerical PREDICTION of the bench gain sweep.

This is a companion to run_gain_sweep.py, not a replacement.  It reproduces the
exact 1-DOF dynamics MuJoCo integrates for models/osl_v2_bench.xml with the ankle
held fixed, so its numbers are a falsifiable prediction: run run_gain_sweep.py on
Windows and the two should agree to within the integrator difference.  It exists
because it needs only numpy/matplotlib, so the tuning argument can be made and
plotted without MuJoCo.

On a fixed base with the ankle locked the knee is a single-DOF pendulum:
    I_eff*qdd = tau_act + tau_gravity(q) - b*qd - frictionloss*sign(qd)
    tau_act   = clip(kp*(ref - q) - kv*qd,  -142.2, +142.2)
so the ONLY plant parameters are the ones authored in the MJCF.  Nothing is fitted.

CONTROLLER TUNING ONLY -- kp/kv are properties of the controller, not the leg.

USAGE (needs matplotlib; use the analysis venv, not osl-mujoco's .venv)
    .venv-analysis\\Scripts\\python.exe experiments\\gain_sweep_analytic.py
    .venv-analysis\\Scripts\\python.exe experiments\\gain_sweep_analytic.py --no-plot
"""

from __future__ import annotations

import argparse
import math
import os

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# ------------------------------------------------ plant, all from osl_v2_bench.xml
I_BODY = 0.251998          # kg.m^2 knee-distal inertia about the knee axis (COMPUTED)
ARMATURE = 0.01
I_EFF = I_BODY + ARMATURE  # 0.261998
B_JOINT = 0.3              # N.m.s/rad
FRIC = 0.4                 # N.m
MASS = 4.505917            # kg knee-distal
G = 9.81
RX0, RZ0 = 0.00179, -0.20027
MGD = MASS * G * math.hypot(RX0, RZ0)      # 8.8529 N.m
DELTA = math.atan2(RX0, -RZ0)              # 0.00894 rad
FORCE_LIM = 142.2          # authored forcerange, UNCHANGED
Q_LO, Q_HI = -0.0872664625997, 2.09439510239
DT = 0.0005

ZETA = 0.7
KPS = [60.0, 200.0, 600.0, 2000.0]


def kv_for(kp, zeta=ZETA):
    return 2.0 * zeta * math.sqrt(kp * I_EFF) - B_JOINT


def tau_gravity(q):
    return -MGD * math.sin(q + DELTA)


def simulate(kp, kv, ref):
    n = len(ref)
    q = np.zeros(n); qd = np.zeros(n); tau = np.zeros(n); sat = np.zeros(n, bool)
    q[0] = ref[0]
    for k in range(n - 1):
        cmd = min(max(ref[k], Q_LO), Q_HI)
        ta = kp * (cmd - q[k]) - kv * qd[k]
        ta_c = min(max(ta, -FORCE_LIM), FORCE_LIM)
        sat[k] = abs(ta) > FORCE_LIM + 1e-9
        tau[k] = ta_c
        net = ta_c + tau_gravity(q[k]) - B_JOINT * qd[k]
        if abs(qd[k]) < 1e-6 and abs(net) <= FRIC:
            qd_new = 0.0
        else:
            f = -FRIC * math.copysign(1.0, qd[k] if abs(qd[k]) > 1e-9 else net)
            qd_new = qd[k] + (net + f) / I_EFF * DT
            if qd[k] != 0.0 and qd_new * qd[k] < 0 and abs(net) <= FRIC:
                qd_new = 0.0
        qd[k + 1] = qd_new
        q[k + 1] = min(max(q[k] + qd_new * DT, Q_LO), Q_HI)
    tau[-1] = tau[-2]; sat[-1] = sat[-2]
    return q, qd, tau, sat


# ------------------------------------------------------------- reference battery
def refs():
    R = {}
    for amp in (10.0, 45.0):
        t = np.arange(0.0, 1.5, DT)
        R[f"step_{int(amp)}deg"] = (t, np.where(t < 0.2, 0.0, math.radians(amp)), 0.4)
    f = 0.9
    t = np.arange(0.0, 4.0 / f, DT)
    R["sine_gait"] = (t, math.radians(30) - math.radians(30) * np.cos(2 * math.pi * f * t),
                      2.0 / f)
    T, pre, post = 0.3, 0.1, 0.6
    t = np.arange(0.0, pre + T + post, DT)
    s = np.clip((t - pre) / T, 0.0, 1.0)
    R["minjerk_60deg"] = (t, math.radians(60) * (10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5), 0.05)
    f0, f1, dur = 0.2, 8.0, 10.0
    t = np.arange(0.0, dur, DT)
    ph = 2 * math.pi * (f0 * t + 0.5 * (f1 - f0) / dur * t ** 2)
    R["chirp"] = (t, math.radians(30) + math.radians(10) * np.sin(ph), 0.2)
    return R


def lin(kp, kv, f_hz):
    w = 2 * math.pi * f_hz
    H = kp / (kp - I_EFF * w ** 2 + 1j * (kv + B_JOINT) * w)
    return abs(H), math.degrees(np.angle(H)), abs(1 - H)


def bandwidth(kp, kv):
    f = np.linspace(0.1, 60.0, 60000)
    mag = np.array([lin(kp, kv, x)[0] for x in f])
    i = np.argmax(mag < 10 ** (-3 / 20))
    return float(f[i]) if i else float("nan")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-plot", action="store_true")
    ap.add_argument("--out", default=os.path.join(ROOT, "experiments",
                                                  "gain_sweep_prediction.png"))
    args = ap.parse_args()

    print("PLANT (osl_v2_bench.xml, ankle locked, fixed base)")
    print(f"  I_eff {I_EFF:.6f} kg.m2   b {B_JOINT}   fric {FRIC} N.m   m*g*d {MGD:.4f} N.m")
    pend = math.sqrt(MGD / I_EFF)
    print(f"  passive pendulum: {pend:.3f} rad/s = {pend / 2 / math.pi:.3f} Hz "
          f"(near gait frequency -- gravity alone adds {MGD:.2f} N.m/rad of stiffness)")
    print(f"  gravity is {100 * MGD / 60:.0f}% of kp at kp=60 but only "
          f"{100 * MGD / 2000:.1f}% at kp=2000\n")

    G_ = [(kp, kv_for(kp)) for kp in KPS]
    print(f"{'kp':>6} {'kv':>8} {'wn Hz':>7} {'f-3dB Hz':>9} "
          f"{'lag@0.9Hz ms':>13} {'lag %GC':>8} {'fric band deg':>14}")
    for kp, kv in G_:
        _, ph, _ = lin(kp, kv, 0.9)
        lag = -ph / 360.0 / 0.9 * 1000
        print(f"{kp:6.0f} {kv:8.3f} {math.sqrt(kp / I_EFF) / 2 / math.pi:7.3f} "
              f"{bandwidth(kp, kv):9.2f} {lag:13.1f} {lag * 0.9 / 10:8.2f} "
              f"{math.degrees(FRIC / kp):14.3f}")

    R = refs()
    print(f"\n{'reference':>15} {'kp':>6} {'RMSe':>8} {'peak e':>8} {'ss e':>7} "
          f"{'peak tau':>9} {'%auth':>7} {'%sat':>6}")
    print("-" * 76)
    table = {}
    for name, (t, r, a0) in R.items():
        for kp, kv in G_:
            q, qd, tau, sat = simulate(kp, kv, r)
            i0 = int(a0 / DT)
            e = np.degrees(q - r)
            m = dict(rms=float(np.sqrt(np.mean(e[i0:] ** 2))),
                     peak=float(np.max(np.abs(e[i0:]))),
                     ss=float(np.mean(np.abs(e[-200:]))),
                     tau=float(np.max(np.abs(tau))),
                     auth=100 * float(np.max(np.abs(tau))) / FORCE_LIM,
                     sat=100 * float(sat.mean()))
            table[(name, kp)] = (m, t, r, q, tau)
            print(f"{name:>15} {kp:6.0f} {m['rms']:8.3f} {m['peak']:8.3f} "
                  f"{m['ss']:7.3f} {m['tau']:9.2f} {m['auth']:7.1f} {m['sat']:6.1f}")
        print()

    if args.no_plot:
        return
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed -- skipping the figure (--no-plot to silence)")
        return

    fig, ax = plt.subplots(2, 2, figsize=(13, 8.5))
    colors = {60: "#c0392b", 200: "#e67e22", 600: "#27ae60", 2000: "#2c3e50"}

    a = ax[0, 0]
    _, t, r, _, _ = table[("sine_gait", 60.0)]
    a.plot(t, np.degrees(r), "k--", lw=2.2, label="reference", zorder=5)
    for kp, _ in G_:
        _, t, r, q, _ = table[("sine_gait", kp)]
        a.plot(t, np.degrees(q), color=colors[kp], lw=1.4, label=f"kp={kp:.0f}")
    a.set_title("0.9 Hz gait-rate sinusoid, 0-60 deg (synthetic, NOT human data)")
    a.set_xlabel("time (s)"); a.set_ylabel("knee flexion (deg)")
    a.legend(fontsize=8, ncol=2); a.grid(alpha=0.3)

    a = ax[0, 1]
    for kp, _ in G_:
        _, t, r, q, _ = table[("sine_gait", kp)]
        a.plot(t, np.degrees(q - r), color=colors[kp], lw=1.4, label=f"kp={kp:.0f}")
    a.axhline(0, color="k", lw=0.8)
    a.set_title("tracking error -- dominated by phase lag, not amplitude loss")
    a.set_xlabel("time (s)"); a.set_ylabel("error (deg)")
    a.legend(fontsize=8, ncol=2); a.grid(alpha=0.3)

    # NB: autoscale, do NOT draw the +/-142.2 lines here -- they are ~7x the peak
    # demand and would crush the traces flat.  The margin is stated as text instead.
    a = ax[1, 0]
    pk = 0.0
    for kp, _ in G_:
        m, t, r, q, tau = table[("minjerk_60deg", kp)]
        a.plot(t, tau, color=colors[kp], lw=1.4, label=f"kp={kp:.0f}")
        pk = max(pk, m["tau"])
    a.set_title("knee torque, min-jerk 0-60 deg in 0.3 s (fastest walking excursion)")
    a.set_xlabel("time (s)"); a.set_ylabel("torque (N.m)")
    a.legend(fontsize=8, loc="upper right"); a.grid(alpha=0.3)
    a.text(0.30, 0.60, f"worst peak {pk:.1f} N.m = {100 * pk / FORCE_LIM:.1f}% of the\n"
                       f"authored +/-{FORCE_LIM:.1f} N.m authority (off-scale above)",
           transform=a.transAxes, fontsize=8.5, va="center",
           bbox=dict(boxstyle="round,pad=0.35", fc="#fdf3e7", ec="#e67e22", lw=0.8))

    a = ax[1, 1]
    kps = [kp for kp, _ in G_]
    rms = [table[("sine_gait", kp)][0]["rms"] for kp in kps]
    auth = [table[("chirp", kp)][0]["auth"] for kp in kps]
    a.semilogx(kps, rms, "o-", color="#27ae60", lw=2, label="RMS error, 0.9 Hz sine (deg)")
    a.set_xlabel("kp (N.m/rad)"); a.set_ylabel("RMS error (deg)", color="#27ae60")
    a.grid(alpha=0.3, which="both")
    a2 = a.twinx()
    a2.semilogx(kps, auth, "s--", color="#c0392b", lw=2,
                label="peak torque on 0.2-8 Hz chirp (% authority)")
    a2.set_ylabel("% of torque authority", color="#c0392b")
    a.set_title("the trade: error falls, torque demand rises")
    for x, y in zip(kps, rms):
        a.annotate(f"{y:.1f} deg", (x, y), textcoords="offset points", xytext=(5, 7),
                   fontsize=8, color="#27ae60")
    a.axvline(600, color="#27ae60", ls=":", alpha=0.7, lw=1.5)
    a.text(0.52, 0.06, "recommended kp=600", transform=a.transAxes, fontsize=8.5,
           color="#1e8449", ha="left",
           bbox=dict(boxstyle="round,pad=0.3", fc="#eafaf1", ec="#27ae60", lw=0.8))
    h1, l1 = a.get_legend_handles_labels(); h2, l2 = a2.get_legend_handles_labels()
    a.legend(h1 + h2, l1 + l2, fontsize=8, loc="upper center")

    fig.suptitle("OSL V2 bench knee -- controller gain sweep PREDICTION "
                 "(zeta=0.7; controller tuning only, the model is unchanged)",
                 fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.965))
    fig.savefig(args.out, dpi=140)
    print(f"figure -> {args.out}")


if __name__ == "__main__":
    main()
