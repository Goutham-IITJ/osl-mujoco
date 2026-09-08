#!/usr/bin/env python3
"""
bench_track_ab19.py -- FIRST REAL BENCH EXPERIMENT: make the CAD-derived OSL V2 knee
track subject AB19's measured knee-angle trajectory for one complete gait cycle.

MODEL      models/osl_v2_bench.xml only.  KA_L1 is NOT used.
CONTROLLER kp = 600 N.m/rad, kv = 17.253 N.m.s/rad (zeta = 0.7), set at RUN TIME.
REFERENCE  build/AB19_knee_gait_reference.csv -- AB19 / levelground / ccw / normal /
           trial 01_01, one complete right gait cycle, 0-100 %, knee angle already
           converted to the OSL flexion-positive convention.

WHAT IS AND IS NOT A MODEL CHANGE
    models/osl_v2_bench.xml is never opened for writing.  Gains are written only into
    the compiled mjModel.  forcerange (+/-142.2 N.m knee) and ctrlrange are untouched
    and are re-asserted after the run.  Mass, inertia, geometry, joint limits, damping,
    frictionloss and armature are untouched.  This is controller tuning, not a change
    to the physical OSL model.

TWO DIFFERENT PHYSICAL QUANTITIES -- DO NOT CONFLATE THEM
    `human_knee_moment` from the Camargo dataset is a BIOMECHANICAL HUMAN JOINT MOMENT.
    `tau_sensor_Nm` is the OSL BENCH ACTUATOR TORQUE.  They are not the same quantity
    and are never combined, differenced or plotted on a shared axis here.  The bench is
    fixed to the world: `knee_prox` has no freejoint, so the actuator fights only
    I*alpha + m*g*d*sin(theta) + b*omega + frictionloss and carries NO body weight.
    Bench torque is therefore not comparable to a human knee moment in stance, where
    the human moment is dominated by body weight the bench never bears.  Stance
    (0-60 %) is shaded on every torque plot for that reason.

ACTUATOR CONVENTIONS ARE IMPORTED, NOT REIMPLEMENTED
    load_and_verify(), set_gains() and EXPECT come from gain_sweep_bench.py so there is
    exactly one torque implementation in this repository.  Importing it also re-runs its
    nine preflight checks, so this experiment refuses to start on an altered model.

USAGE
    Simulate + metrics + CSV (osl-mujoco's own venv: mujoco + numpy is all it needs):
        .venv\\Scripts\\python.exe experiments\\bench_track_ab19.py
    Plots only, from an existing CSV (analysis venv; needs matplotlib, not mujoco):
        .venv-analysis\\Scripts\\python.exe experiments\\bench_track_ab19.py --plot-from ^
            build\\bench_track_ab19\\bench_track_ab19.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import textwrap

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)                      # so gain_sweep_bench is importable
sys.path.insert(0, os.path.join(ROOT, "scripts"))

# mujoco and matplotlib are BOTH optional at import time, on purpose: osl-mujoco's .venv
# has mujoco and no matplotlib, the analysis venv has matplotlib and no mujoco.  Each
# half of this script must run in the venv that can support it.
try:
    import mujoco
except ImportError:                                          # pragma: no cover
    mujoco = None

KP = 600.0                  # N.m/rad   validated by the gain sweep
KV = 17.253                 # N.m.s/rad zeta = 0.7 crediting the authored joint damping
DT_EXPECT = 0.0005          # s, the authored MJCF timestep
DEFAULT_CSV = os.path.join(ROOT, "build", "AB19_knee_gait_reference.csv")
OUTDIR = os.path.join(ROOT, "build", "bench_track_ab19")
STANCE_END = 60.0           # % gait cycle, nominal toe-off for labelling only

SUBJECT = "AB19"
TRIAL = "levelground / ccw / normal / trial 01_01, one complete right gait cycle"
MASS_KG = 68.0              # AB19, paper Table 1 -- used ONLY for the audit below

NOTES: list[str] = []
FLAGS: list[str] = []


def note(msg):
    NOTES.append(msg)
    print(f"    {msg}")


def flag(msg):
    FLAGS.append(msg)
    print(f"    [FLAG] {msg}")


# ===================================================== 1. reference: load and audit
def load_reference(path):
    """Read the AB19 CSV verbatim.  Nothing is rescaled, resigned or repaired here."""
    with open(path, newline="") as fh:
        rows = list(csv.DictReader(fh))
    need = ("gait_phase_percent", "time_s", "human_knee_angle_deg",
            "human_knee_moment", "human_knee_power")
    missing = [c for c in need if c not in (rows[0] if rows else {})]
    if missing:
        sys.exit(f"FATAL: {path} is missing columns {missing}")
    col = {c: np.array([float(r[c]) for r in rows]) for c in need}
    col["n"] = len(rows)
    return col


def audit_reference(ref, knee_lo_deg, knee_hi_deg):
    """Check the reference against the bench ROM and against biomechanical plausibility.

    This is a REPORT, not a repair: no column is modified.  Two of these checks exist
    because a reference that is silently wrong produces a tracking result that looks
    perfect and means nothing.
    """
    ph, t = ref["gait_phase_percent"], ref["time_s"]
    a, m, p = (ref["human_knee_angle_deg"], ref["human_knee_moment"],
               ref["human_knee_power"])
    n = ref["n"]
    T = float(t[-1] - t[0])

    print("\n[R1] REFERENCE GRID")
    note(f"{n} samples, gait phase {ph[0]:g} -> {ph[-1]:g} %, "
         f"uniform 1 % grid = {np.allclose(np.diff(ph), 1.0)}")
    note(f"cycle duration {T:.4f} s -> stride frequency {1.0 / T:.3f} Hz "
         f"(sampled at {1.0 / float(np.mean(np.diff(t))):.1f} Hz)")
    if not np.all(np.diff(t) > 0):
        sys.exit("FATAL: time_s is not strictly increasing")
    if not (abs(ph[0]) < 1e-9 and abs(ph[-1] - 100.0) < 1e-9):
        flag(f"gait phase does not span exactly 0-100 % (got {ph[0]:g}-{ph[-1]:g})")

    print("\n[R2] BENCH ROM CONTAINMENT  (instruction 13)")
    note(f"reference knee angle range [{a.min():.3f}, {a.max():.3f}] deg")
    note(f"bench authored knee ROM    [{knee_lo_deg:.3f}, {knee_hi_deg:.3f}] deg")
    below = int(np.sum(a < knee_lo_deg))
    above = int(np.sum(a > knee_hi_deg))
    exceeded = bool(below or above)
    if exceeded:
        flag(f"reference EXCEEDS the bench ROM: {below} samples below "
             f"{knee_lo_deg:.2f} deg, {above} above {knee_hi_deg:.2f} deg "
             f"-> the commanded reference WILL be clamped by ctrlrange")
    else:
        note("reference lies entirely INSIDE the bench ROM -- no clamping required, "
             "margin " f"{a.min() - knee_lo_deg:.3f} deg below / "
             f"{knee_hi_deg - a.max():.3f} deg above")

    print("\n[R3] FLEXION-SIGN LANDMARK CHECK  (the conversion is verified, not assumed)")
    st = ph <= 40.0
    i_st = int(np.argmax(a[st]))
    i_sw = int(np.argmax(a))
    note(f"stance flexion peak {a[st][i_st]:.2f} deg at {ph[st][i_st]:g} % "
         f"(expect ~15-20 deg near 15 %)")
    note(f"swing flexion peak  {a[i_sw]:.2f} deg at {ph[i_sw]:g} % "
         f"(expect ~60-65 deg near 70-73 %)")
    ok_sign = (10.0 <= a[st][i_st] <= 28.0 and 5.0 <= ph[st][i_st] <= 25.0
               and 50.0 <= a[i_sw] <= 80.0 and 60.0 <= ph[i_sw] <= 80.0)
    if ok_sign:
        note("landmarks are consistent with FLEXION-POSITIVE -- conversion confirmed")
    else:
        flag("landmarks do NOT match the expected flexion-positive pattern; check the "
             "sign conversion before trusting anything downstream")

    print("\n[R4] HUMAN MOMENT / POWER PLAUSIBILITY  (carried through, never used as "
          "bench torque)")
    pk_m, pk_p = float(np.max(np.abs(m))), float(np.max(np.abs(p)))
    note(f"peak |human_knee_moment| = {pk_m:.3f}  (column unit not declared in the CSV)")
    note(f"peak |human_knee_power|  = {pk_p:.3f}")
    note(f"if N.m       -> {pk_m / MASS_KG:.3f} N.m/kg for {MASS_KG:g} kg")
    note(f"if mN.m/kg   -> {pk_m / 1000.0:.3f} N.m/kg = {pk_m / 1000.0 * MASS_KG:.2f} N.m")
    # The Camargo dataset documents moment in N.m/kg and power in W/kg, so the expected
    # peak here is ~0.4-0.6 N.m/kg.  Anything else is an extraction scale error.
    if not 0.15 <= pk_m / MASS_KG <= 1.5:
        flag(f"peak moment is {pk_m / MASS_KG:.2f} N.m/kg if the column is N.m -- a "
             f"human walking knee peaks near 0.4-0.6 N.m/kg, so the moment column is "
             f"off by roughly {pk_m / MASS_KG / 0.5:.0f}x or is in another unit "
             f"({pk_m / 1000.0:.3f} N.m/kg would be plausible)")

    # DECISIVE TEST: two independent biomechanical signals cannot be ~perfectly
    # correlated in magnitude.  If they are, the CSV carries one signal twice.
    rho = float(np.corrcoef(np.abs(m), np.abs(p))[0, 1])
    note(f"corr(|moment|, |power|) = {rho:.6f}")
    if abs(rho) > 0.99:
        flag(f"|human_knee_moment| and |human_knee_power| are correlated at "
             f"{rho:.5f} -- they are ONE signal scaled, not two independent "
             f"measurements. Moment and power have different zero crossings in real "
             f"gait, so this cannot be biomechanics.")

    # power should equal moment * omega; if it does not, the two columns are not a
    # matched pair and neither can be interpreted as biomechanics.
    om = np.gradient(np.deg2rad(a), t)
    big = np.abs(m) > 0.05 * pk_m
    ratio = p[big] / m[big]
    r_sd = float(np.std(ratio))
    corr = float(np.corrcoef(ratio, om[big])[0, 1]) if np.std(ratio) > 1e-12 else 0.0
    note(f"power/moment ratio over the cycle: mean {np.mean(ratio):+.3f}, "
         f"sd {r_sd:.3f}; d(angle)/dt spans {om.min():+.2f} .. {om.max():+.2f} rad/s")
    if r_sd < 0.25 * float(np.std(om[big])) or abs(corr) < 0.7:
        flag("power/moment is nearly CONSTANT and uncorrelated with the reference "
             f"angular velocity (corr {corr:+.2f}) -- so human_knee_power is not "
             "moment x omega for this angle column.  The two kinetic columns are not a "
             "matched pair; treat both as UNVERIFIED and re-check the extraction.")
    if int(np.argmax(np.abs(m))) == 0:
        flag("peak |human_knee_moment| falls on the very FIRST sample (0 % gait "
             "phase); the early-stance moment peak should sit near 15-25 %, which "
             "suggests a boundary/alignment problem in the kinetic columns")
    # shape test: a human knee moment is biphasic. A monotone decay from a maximum at
    # 0 % to ~0 is not a knee-moment waveform at all, whatever the units are.
    st40 = ph <= 40.0
    dm = np.diff(np.abs(m[st40]))
    if float(np.mean(dm <= 0)) > 0.95 and abs(m[st40][-1]) < 0.15 * pk_m:
        flag(f"human_knee_moment decays almost MONOTONICALLY from its maximum at 0 % to "
             f"{100 * abs(m[st40][-1]) / pk_m:.0f} % of peak by 40 % gait phase "
             f"({float(np.mean(dm <= 0)) * 100:.0f} % of samples decreasing). A human "
             f"knee moment is biphasic with an early-stance peak near 15-25 %, so this "
             f"is not a knee-moment waveform -- most likely the wrong column, a unit/"
             f"scale error, or a filter/boundary artefact in the extraction.")

    return dict(T=T, exceeded_rom=exceeded, n_below=below, n_above=above,
                sign_ok=ok_sign, peak_moment=pk_m, peak_power=pk_p)


# ==================================================== 2. resample onto the bench dt
class NaturalCubic:
    """Natural cubic spline, pure numpy (scipy is deliberately not a dependency of
    osl-mujoco's .venv).  Used instead of linear interpolation because the reference
    arrives on a 1 % grid (~83 Hz here): linear interpolation would give a piecewise
    constant reference VELOCITY, i.e. an 83 Hz staircase in a signal we plot and
    differentiate.  The spline passes exactly through every original sample, so no
    measured value is altered."""

    def __init__(self, x, y):
        x = np.asarray(x, float)
        y = np.asarray(y, float)
        n = len(x)
        h = np.diff(x)
        A = np.zeros((n, n))
        rhs = np.zeros(n)
        A[0, 0] = A[-1, -1] = 1.0                    # natural: y'' = 0 at both ends
        for i in range(1, n - 1):
            A[i, i - 1] = h[i - 1]
            A[i, i] = 2.0 * (h[i - 1] + h[i])
            A[i, i + 1] = h[i]
            rhs[i] = 6.0 * ((y[i + 1] - y[i]) / h[i] - (y[i] - y[i - 1]) / h[i - 1])
        self.x, self.y, self.h = x, y, h
        self.m = np.linalg.solve(A, rhs)             # second derivatives at the knots

    def _seg(self, xq):
        return np.clip(np.searchsorted(self.x, xq, side="right") - 1, 0, len(self.h) - 1)

    def __call__(self, xq):
        i = self._seg(xq)
        h, a, b = self.h[i], self.x[i + 1] - xq, xq - self.x[i]
        return (self.m[i] * a ** 3 + self.m[i + 1] * b ** 3) / (6 * h) \
            + (self.y[i] / h - self.m[i] * h / 6) * a \
            + (self.y[i + 1] / h - self.m[i + 1] * h / 6) * b

    def deriv(self, xq):
        i = self._seg(xq)
        h, a, b = self.h[i], self.x[i + 1] - xq, xq - self.x[i]
        return (-self.m[i] * a ** 2 + self.m[i + 1] * b ** 2) / (2 * h) \
            - (self.y[i] / h - self.m[i] * h / 6) \
            + (self.y[i + 1] / h - self.m[i + 1] * h / 6)


def resample(ref, dt, cycles, kind="cubic"):
    """Put the reference on the bench timestep, preserving the 0-100 % phase mapping
    exactly: phase is an affine function of time within a cycle, so it is recomputed
    analytically rather than interpolated."""
    t0 = float(ref["time_s"][0])
    T = float(ref["time_s"][-1] - t0)
    tr = ref["time_s"] - t0
    n_cyc = int(round(T / dt))
    n = n_cyc * cycles
    tq_cyc = np.arange(n_cyc) * dt                       # [0, T) -- no duplicate knot
    tq = np.concatenate([tq_cyc + c * T for c in range(cycles)])
    phase = 100.0 * (tq % T) / T
    src = tq % T

    out = {"time_s": tq, "gait_phase_percent": phase}
    for c in ("human_knee_angle_deg", "human_knee_moment", "human_knee_power"):
        if kind == "cubic":
            sp = NaturalCubic(tr, ref[c])
            out[c] = sp(src)
            if c == "human_knee_angle_deg":
                out["ref_vel_rad_s"] = np.deg2rad(sp.deriv(src))
        else:
            out[c] = np.interp(src, tr, ref[c])
            if c == "human_knee_angle_deg":
                out["ref_vel_rad_s"] = np.gradient(np.deg2rad(out[c]), dt)
    out["ref_rad"] = np.deg2rad(out["human_knee_angle_deg"])

    print("\n[R5] RESAMPLE")
    note(f"{kind} interpolation, {ref['n']} samples -> {n_cyc} steps/cycle "
         f"x {cycles} cycle(s) = {n} steps at dt = {dt * 1e3:.3f} ms")
    # The interpolant must pass through every measured sample: evaluate it AT the
    # original knots, not by re-interpolating the dense grid (that would measure the
    # check's own error, not the interpolant's).
    if kind == "cubic":
        err = float(np.max(np.abs(NaturalCubic(tr, ref["human_knee_angle_deg"])(tr)
                                  - ref["human_knee_angle_deg"])))
    else:
        err = float(np.max(np.abs(np.interp(tr, tr, ref["human_knee_angle_deg"])
                                  - ref["human_knee_angle_deg"])))
    note(f"interpolant error at the original knots: {err:.2e} deg "
         f"(0 => every measured sample is reproduced exactly, none altered)")
    note(f"phase mapping preserved analytically: phase = 100*(t mod {T:.4f})/{T:.4f}; "
         f"grid is [0, T) so the commanded phase runs 0 -> {phase.max():.4f} %")
    note(f"reference angular velocity from the spline derivative, peak "
         f"{np.max(np.abs(out['ref_vel_rad_s'])):.3f} rad/s")
    gap = float(ref["human_knee_angle_deg"][-1] - ref["human_knee_angle_deg"][0])
    note(f"cycle closure: angle at 100 % minus angle at 0 % = {gap:+.3f} deg")
    if abs(gap) > 0.5 and cycles > 1:
        flag(f"the reference cycle does not close ({gap:+.3f} deg): running "
             f"{cycles} cycles puts a step discontinuity at every wrap. Use "
             f"--cycles 1, or close the cycle in the extraction first.")
    return out, n


# =========================================================== 3. simulate + log
def simulate(info, res, n, kp, kv):
    """One (or more) gait cycles.  Actuator handling is copied from
    gain_sweep_bench.run() so there is one torque convention, not two."""
    import gain_sweep_bench as gsb

    model, data = info["model"], info["data"]
    ka, aa = info["ka"], info["aa"]
    kq, aq = info["knee_qpos"], info["ankle_qpos"]
    S = info["sens"]
    lim = info["forcerange0"][0][1]

    gsb.set_gains(model, ka, kp, kv)
    # Ankle: left on its OWN authored servo (kp=60, kv=0) and commanded to hold its
    # keyframe value -- a fixed boundary condition, not a second variable (instruction 5).
    gsb.set_gains(model, aa, gsb.EXPECT["ankle_kp_authored"],
                  gsb.EXPECT["ankle_kv_authored"])

    # load_and_verify() stores mj_name2id's raw result, which is -1 if the "flat"
    # keyframe is ever renamed; mj_resetDataKeyframe(-1) would raise. Mirror the guard
    # gain_sweep_bench uses internally.
    kid = info["keyframe"]
    mujoco.mj_resetDataKeyframe(model, data, kid if kid >= 0 else 0)
    ankle_hold = float(data.qpos[aq])
    data.qpos[kq] = float(res["ref_rad"][0])       # start ON the reference, as gsb does
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)
    print("\n[S1] INITIAL CONDITION")
    note(f"knee seeded at the reference value {math.degrees(res['ref_rad'][0]):.3f} deg, "
         f"qvel = 0 (the reference itself starts at "
         f"{res['ref_vel_rad_s'][0]:+.3f} rad/s, so expect a brief start transient)")
    note(f"ankle held at its keyframe value {math.degrees(ankle_hold):.3f} deg by its "
         f"own authored kp=60/kv=0 servo, untouched")

    ref = res["ref_rad"]
    cr = info["ctrlrange"]
    rows = []
    q = np.empty(n); qd = np.empty(n); tau = np.empty(n)
    ctrl = np.empty(n); satf = np.zeros(n, bool); clamped = 0
    ankle_dev = 0.0

    for k in range(n):
        cmd = float(np.clip(ref[k], cr[0], cr[1]))
        clamped += int(abs(cmd - ref[k]) > 1e-12)
        data.ctrl[ka] = cmd
        data.ctrl[aa] = ankle_hold
        mujoco.mj_step(model, data)

        q[k] = float(data.sensordata[S["knee_q"]])
        qd[k] = float(data.sensordata[S["knee_qd"]])
        tau[k] = float(data.sensordata[S["knee_tau"]])
        ctrl[k] = cmd
        tau_direct = float(data.actuator_force[ka])
        tau_ideal = kp * (cmd - q[k]) - kv * qd[k]      # unclamped, the servo's request
        satf[k] = abs(tau_ideal) > lim + 1e-6
        a_q = float(data.sensordata[S["ankle_q"]])
        ankle_dev = max(ankle_dev, abs(a_q - ankle_hold))

        rows.append((
            f"{data.time:.6f}", f"{res['gait_phase_percent'][k]:.6f}",
            f"{ref[k]:.8f}", f"{math.degrees(ref[k]):.6f}",
            f"{q[k]:.8f}", f"{math.degrees(q[k]):.6f}",
            f"{math.degrees(q[k] - ref[k]):.6f}",
            f"{res['ref_vel_rad_s'][k]:.6f}", f"{qd[k]:.6f}",
            f"{cmd:.8f}",
            f"{tau[k]:.6f}", f"{tau_direct:.6f}", f"{tau_ideal:.6f}",
            f"{100.0 * abs(tau[k]) / lim:.4f}", int(satf[k]),
            f"{tau[k] * qd[k]:.6f}",
            f"{a_q:.8f}", f"{float(data.sensordata[S['ankle_tau']]):.6f}",
            f"{res['human_knee_angle_deg'][k]:.6f}",
            f"{res['human_knee_moment'][k]:.6f}",
            f"{res['human_knee_power'][k]:.6f}",
        ))

    err = np.degrees(q - ref)
    pw = tau * qd
    M = dict(
        rms_err_deg=float(np.sqrt(np.mean(err ** 2))),
        peak_err_deg=float(np.max(np.abs(err))),
        mae_err_deg=float(np.mean(np.abs(err))),
        peak_tau_Nm=float(np.max(np.abs(tau))),
        pct_authority=100.0 * float(np.max(np.abs(tau))) / lim,
        sat_pct=100.0 * float(satf.mean()),
        peak_vel_rad_s=float(np.max(np.abs(qd))),
        peak_ref_vel_rad_s=float(np.max(np.abs(res["ref_vel_rad_s"]))),
        peak_power_W=float(np.max(np.abs(pw))),
        mean_abs_power_W=float(np.mean(np.abs(pw))),
        ref_clamped_steps=clamped,
        ankle_dev_deg=math.degrees(ankle_dev),
        force_limit_Nm=lim,
        final_time_s=float(data.time),
        n_steps=n,
    )
    # error after the start transient, so one seeded initial condition cannot flatter
    # or spoil the headline number
    i1 = min(int(0.05 / model.opt.timestep), n - 2)
    M["rms_err_deg_after_50ms"] = float(np.sqrt(np.mean(err[i1:] ** 2)))
    M["peak_err_deg_after_50ms"] = float(np.max(np.abs(err[i1:])))

    # ---- diagnostic: is the residual error PHASE LAG or genuine tracking failure?
    # A position servo lags by roughly kv-limited first-order time; if the whole error
    # collapses when the reference is shifted in time, the error is a time shift and is
    # removable for free (advance the reference / feedforward), NOT by raising kp.
    # This only measures the data already logged; it changes nothing in the experiment.
    best_s, best_rms = 0, M["rms_err_deg"]
    for s in range(1, min(int(0.15 / model.opt.timestep), n // 4)):
        e = np.degrees(q[s:] - ref[:-s])          # compare sim(t) with ref(t - s*dt)
        rr = float(np.sqrt(np.mean(e ** 2)))
        if rr < best_rms:
            best_s, best_rms = s, rr
    M["lag_ms"] = best_s * model.opt.timestep * 1e3
    M["lag_pct_gc"] = 100.0 * best_s * model.opt.timestep / (n * model.opt.timestep)
    M["rms_err_deg_lag_removed"] = best_rms
    M["pct_err_from_lag"] = 100.0 * (1.0 - best_rms / M["rms_err_deg"]) \
        if M["rms_err_deg"] > 0 else 0.0
    return rows, M, dict(q=q, qd=qd, tau=tau, ctrl=ctrl, err=err, power=pw)


CSV_HEADER = ("time_s", "gait_phase_percent", "ref_rad", "ref_deg", "sim_rad",
              "sim_deg", "err_deg", "ref_vel_rad_s", "sim_vel_rad_s", "ctrl_rad",
              "tau_sensor_Nm", "tau_actforce_Nm", "tau_unclamped_Nm", "pct_authority",
              "saturated", "knee_power_W", "ankle_q_rad", "ankle_tau_Nm",
              "human_knee_angle_deg", "human_knee_moment", "human_knee_power")


# ================================================================== 4. plots A-F
def make_plots(path_csv, outdir, kp, kv, flags):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("\n[P] matplotlib not available in this venv -- CSV written, plots skipped.")
        print("    Regenerate them from the CSV in the analysis venv:")
        print(f"      .venv-analysis\\Scripts\\python.exe experiments\\"
              f"bench_track_ab19.py --plot-from {os.path.relpath(path_csv, ROOT)}")
        return None, None

    with open(path_csv, newline="") as fh:
        lines = fh.read().splitlines()
    # Provenance comment lines sit ABOVE the header, so the header row must be FOUND,
    # not assumed to be line 1.  Locate it by its first field: a comment written through
    # csv.writer would be quoted and would not start with '#', so don't test for that.
    hdr = next((i for i, ln in enumerate(lines)
                if ln.split(",")[0].strip().strip('"') == CSV_HEADER[0]), None)
    if hdr is None:
        sys.exit(f"FATAL: no '{CSV_HEADER[0]}' header row in {path_csv} -- is it a "
                 f"bench_track_ab19 output CSV?")
    rows = list(csv.DictReader(lines[hdr:]))
    missing = [c for c in CSV_HEADER if c not in (rows[0] if rows else {})]
    if missing:
        sys.exit(f"FATAL: {path_csv} is missing columns {missing}")
    C = {k: np.array([float(r[k]) for r in rows]) for k in CSV_HEADER}
    ph = C["gait_phase_percent"]
    o = np.argsort(ph, kind="stable")            # in case >1 cycle was logged

    def shade(ax):
        ax.axvspan(0, STANCE_END, color="0.55", alpha=0.13, lw=0, zorder=0)
        ax.axvspan(STANCE_END, 100, color="#2980b9", alpha=0.07, lw=0, zorder=0)
        ax.set_xlim(0, 100)
        ax.grid(alpha=0.3)
        ax.set_xlabel("gait phase (%)")

    fig, ax = plt.subplots(2, 3, figsize=(16.5, 9))

    a = ax[0, 0]; shade(a)
    a.plot(ph[o], C["human_knee_angle_deg"][o], "k--", lw=2.4, label="AB19 human reference")
    a.plot(ph[o], C["sim_deg"][o], color="#27ae60", lw=1.6, label="OSL bench simulated")
    a.set_ylabel("knee flexion (deg)")
    a.set_title("A. human reference vs simulated knee angle")
    a.legend(fontsize=8, loc="center left")

    a = ax[0, 1]; shade(a)
    a.plot(ph[o], C["err_deg"][o], color="#c0392b", lw=1.4)
    a.axhline(0, color="k", lw=0.8)
    a.set_ylabel("simulated - reference (deg)")
    a.set_title("B. tracking error vs gait phase")

    a = ax[0, 2]; shade(a)
    a.plot(ph[o], C["tau_sensor_Nm"][o], color="#8e44ad", lw=1.4)
    a.axhline(0, color="k", lw=0.8)
    a.set_ylabel("bench actuator torque (N.m)")
    a.set_title("C. simulated knee torque vs gait phase")
    lim = 142.2
    pk_t = float(np.max(np.abs(C["tau_sensor_Nm"])))
    a.text(0.02, 0.02, f"peak {pk_t:.2f} N.m = {100 * pk_t / lim:.1f} % of "
                       f"+/-{lim:.1f} N.m authority",
           transform=a.transAxes, fontsize=8, va="bottom",
           bbox=dict(boxstyle="round,pad=0.3", fc="#f9f0fb", ec="#8e44ad", lw=0.8))

    a = ax[1, 0]; shade(a)
    a.plot(ph[o], C["ref_vel_rad_s"][o], "k--", lw=1.6, label="reference")
    a.plot(ph[o], C["sim_vel_rad_s"][o], color="#e67e22", lw=1.4, label="simulated")
    a.axhline(0, color="k", lw=0.8)
    a.set_ylabel("knee angular velocity (rad/s)")
    a.set_title("D. knee angular velocity vs gait phase")
    a.legend(fontsize=8, loc="lower left")

    a = ax[1, 1]; shade(a)
    a.plot(ph[o], C["knee_power_W"][o], color="#16a085", lw=1.4)
    a.axhline(0, color="k", lw=0.8)
    a.set_ylabel("actuator mechanical power (W)")
    a.set_title("E. simulated knee power (torque x omega) vs gait phase")

    a = ax[1, 2]; shade(a)
    # reference drawn thick UNDERNEATH so that "the command equals the reference" is
    # visible as coincidence rather than as one line hiding the other
    a.plot(ph[o], C["ref_deg"][o], color="#e74c3c", lw=3.2, alpha=0.55,
           label="reference (pre-clamp)")
    a.plot(ph[o], np.degrees(C["ctrl_rad"][o]), color="#2c3e50", lw=1.3,
           label="commanded ctrl (post-clamp)")
    a.set_ylabel("actuator command (deg)")
    a.set_title("F. actuator control vs gait phase")
    a.legend(fontsize=8, loc="center left")
    # 1e-4 deg, not 1e-9: ref_deg and ctrl_rad are written to finite precision, so an
    # exact comparison after a CSV round-trip would report false clamping everywhere.
    # The authoritative count is ref_clamped_steps in the metrics CSV, computed in-run.
    nclamp = int(np.sum(np.abs(np.degrees(C["ctrl_rad"]) - C["ref_deg"]) > 1e-4))
    a.text(0.02, 0.02, "command == reference at every step: NO clamping"
           if nclamp == 0 else f"reference CLAMPED at {nclamp} of {len(ph)} steps",
           transform=a.transAxes, fontsize=8, va="bottom",
           bbox=dict(boxstyle="round,pad=0.3",
                     fc="#eafaf1" if nclamp == 0 else "#fdecea",
                     ec="#27ae60" if nclamp == 0 else "#c0392b", lw=0.8))

    # stance/swing captions last, in axes coordinates with headroom made for them, so
    # they cannot be clipped by the spine or hidden behind a legend
    for a in ax.ravel():
        lo, hi = a.get_ylim()
        a.set_ylim(lo, hi + 0.13 * (hi - lo))
        a.text(0.30, 0.985, "stance 0-60 %", transform=a.transAxes, ha="center",
               va="top", fontsize=8.5, color="0.30")
        a.text(0.80, 0.985, "swing 60-100 %", transform=a.transAxes, ha="center",
               va="top", fontsize=8.5, color="#1f618d")

    fig.suptitle(f"OSL V2 CAD bench knee tracking {SUBJECT} -- {TRIAL}\n"
                 f"kp={kp:.0f} N.m/rad, kv={kv:.3f} N.m.s/rad (runtime only; "
                 f"models/osl_v2_bench.xml unchanged)", fontsize=11)
    fig.text(0.5, 0.012,
             "Panel C is BENCH ACTUATOR TORQUE, not a human knee moment: the bench is "
             "fixed to the world and carries no body weight, so the actuator opposes "
             "only inertia, gravity on the knee-distal segment,\njoint damping and "
             "friction. The shaded stance region is therefore NOT comparable to human "
             "stance-phase knee kinetics. Only the knee ANGLE is taken from AB19.",
             ha="center", va="bottom", fontsize=8.5,
             bbox=dict(boxstyle="round,pad=0.4", fc="#fdf6e3", ec="#b58900", lw=0.9))
    fig.tight_layout(rect=(0, 0.075, 1, 0.94))
    p1 = os.path.join(outdir, "bench_track_ab19.png")
    fig.savefig(p1, dpi=140)

    # The human kinetic columns get their OWN figure, deliberately not sharing an axis
    # with any bench signal, and carrying the audit warning.
    fig2, ax2 = plt.subplots(1, 2, figsize=(13.4, 6.4))
    for a, key, lab in ((ax2[0], "human_knee_moment", "human knee moment"),
                        (ax2[1], "human_knee_power", "human knee power")):
        shade(a)
        a.plot(ph[o], C[key][o], color="#7f8c8d", lw=1.5)
        a.axhline(0, color="k", lw=0.8)
        a.set_ylabel(f"{lab} (unit UNVERIFIED)")
        a.set_title(f"{lab} -- AB19 reference, retained not used", fontsize=10.5)
    msg = ("Retained per instruction 8 and NOT used to drive or judge the bench. These "
           "are biomechanical human joint\nquantities, a different physical quantity "
           "from the bench actuator torque in panel C of the main figure.")
    if flags:
        msg += "\n\nAUDIT FLAGS -- do not interpret these two curves until resolved:\n"
        msg += "\n".join(textwrap.fill(f, 112, initial_indent="- ",
                                       subsequent_indent="  ") for f in flags[:4])
    fig2.text(0.5, 0.015, msg, ha="center", va="bottom", fontsize=8.2,
              linespacing=1.35,
              bbox=dict(boxstyle="round,pad=0.4", fc="#fdf6e3", ec="#b58900", lw=0.9))
    fig2.suptitle(f"{SUBJECT} human reference kinetics -- separate signals, "
                  f"NOT bench actuator torque", fontsize=11)
    fig2.tight_layout(rect=(0, 0.36, 1, 0.94))
    p2 = os.path.join(outdir, "bench_track_ab19_human_reference.png")
    fig2.savefig(p2, dpi=140)
    print(f"\n[P] figures -> {p1}\n              {p2}")
    return p1, p2


# ====================================================================== 5. main
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--csv", default=DEFAULT_CSV, help="AB19 reference CSV")
    ap.add_argument("--outdir", default=OUTDIR)
    ap.add_argument("--kp", type=float, default=KP)
    ap.add_argument("--kv", type=float, default=KV)
    ap.add_argument("--cycles", type=int, default=1,
                    help="gait cycles to run (instruction 6: exactly one, the default)")
    ap.add_argument("--interp", choices=("cubic", "linear"), default="cubic")
    ap.add_argument("--no-plot", action="store_true")
    ap.add_argument("--plot-from", default=None,
                    help="skip the simulation and rebuild the plots from an existing CSV")
    args = ap.parse_args()
    os.makedirs(args.outdir, exist_ok=True)

    if args.plot_from:
        make_plots(args.plot_from, args.outdir, args.kp, args.kv, [])
        return 0

    if mujoco is None:
        sys.exit("FATAL: mujoco is not importable in this interpreter. Run the "
                 "simulation with osl-mujoco's own venv:\n"
                 "  .venv\\Scripts\\python.exe experiments\\bench_track_ab19.py")
    import gain_sweep_bench as gsb

    print("=" * 78)
    print(f"FIRST BENCH TRACKING EXPERIMENT -- {SUBJECT}")
    print(f"  reference : {args.csv}")
    print(f"  trial     : {TRIAL}")
    print(f"  controller: kp={args.kp:.1f} N.m/rad  kv={args.kv:.3f} N.m.s/rad "
          f"(runtime only)")
    print("=" * 78)

    # preflight from gain_sweep_bench: nine checks on the compiled model.  If the model
    # has drifted from what the gain sweep validated, stop here.
    info = gsb.load_and_verify()
    if gsb.FAILS:
        print(f"\nREFUSING TO RUN: {len(gsb.FAILS)} preflight failure(s)")
        for f in gsb.FAILS:
            print(f"  - {f}")
        return len(gsb.FAILS)
    dt = float(info["model"].opt.timestep)
    if abs(dt - DT_EXPECT) > 1e-12:
        sys.exit(f"FATAL: bench timestep is {dt} s, expected {DT_EXPECT} s")

    ref = load_reference(args.csv)
    lo_deg = math.degrees(info["ctrlrange"][0])
    hi_deg = math.degrees(info["ctrlrange"][1])
    A = audit_reference(ref, lo_deg, hi_deg)
    res, n = resample(ref, dt, args.cycles, args.interp)

    rows, M, sig = simulate(info, res, n, args.kp, args.kv)

    # integrity: the authored limits must be exactly as compiled
    model = info["model"]
    fr_k = tuple(float(x) for x in model.actuator_forcerange[info["ka"]])
    fr_a = tuple(float(x) for x in model.actuator_forcerange[info["aa"]])
    cr = tuple(float(x) for x in model.actuator_ctrlrange[info["ka"]])
    print("\n[S2] INTEGRITY  (the model must be exactly as authored)")
    ok = (fr_k == info["forcerange0"][0] and fr_a == info["forcerange0"][1]
          and cr == tuple(info["ctrlrange"]))
    note(f"knee forcerange {fr_k}  ankle {fr_a}  knee ctrlrange {cr}")
    note("unchanged from the authored values" if ok else "CHANGED -- INVALID RUN")
    if not ok:
        return 1

    p_csv = os.path.join(args.outdir, "bench_track_ab19.csv")
    with open(p_csv, "w", newline="") as fh:
        # written directly, NOT through csv.writer: these contain commas and would come
        # back quoted, which breaks a '#'-prefix test on re-read
        fh.write(f"# subject {SUBJECT}; {TRIAL}\n")
        fh.write(f"# model {os.path.relpath(info['path'], ROOT)} (UNMODIFIED); "
                 f"kp={args.kp} kv={args.kv} set at runtime; dt={dt}\n")
        fh.write("# tau_sensor_Nm is BENCH ACTUATOR torque; human_knee_moment is a "
                 "BIOMECHANICAL HUMAN moment. Different quantities, never combined.\n")
        w = csv.writer(fh)
        w.writerow(list(CSV_HEADER))
        w.writerows(rows)

    p_met = os.path.join(args.outdir, "bench_track_ab19_metrics.csv")
    with open(p_met, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["metric", "value"])
        for k, v in M.items():
            w.writerow([k, v])
        for k, v in A.items():
            w.writerow([f"ref_{k}", v])
        for i, f in enumerate(FLAGS):
            w.writerow([f"audit_flag_{i + 1}", f])

    print("\n" + "=" * 78)
    print("RESULTS  (instruction 12)")
    print("=" * 78)
    print(f"  RMS angle error            {M['rms_err_deg']:9.4f} deg")
    print(f"  peak angle error           {M['peak_err_deg']:9.4f} deg")
    print(f"  mean absolute error        {M['mae_err_deg']:9.4f} deg")
    print(f"  RMS  err after 50 ms       {M['rms_err_deg_after_50ms']:9.4f} deg  "
          f"(start transient excluded)")
    print(f"  peak err after 50 ms       {M['peak_err_deg_after_50ms']:9.4f} deg")
    print(f"  peak |actuator torque|     {M['peak_tau_Nm']:9.4f} N.m")
    print(f"  % of OSL torque authority  {M['pct_authority']:9.4f} %  "
          f"(limit +/-{M['force_limit_Nm']:.1f} N.m, UNCHANGED)")
    print(f"  torque saturation          {M['sat_pct']:9.4f} %  of steps")
    print(f"  peak |angular velocity|    {M['peak_vel_rad_s']:9.4f} rad/s  "
          f"(reference {M['peak_ref_vel_rad_s']:.4f})")
    print(f"  peak |actuator power|      {M['peak_power_W']:9.4f} W")
    print(f"  ankle drift from keyframe  {M['ankle_dev_deg']:9.4f} deg")
    print(f"  steps simulated            {M['n_steps']:9d}  "
          f"({M['final_time_s']:.4f} s = {args.cycles} gait cycle(s))")
    print("\nIS THE RESIDUAL ERROR LAG OR FAILURE?  (diagnostic, changes nothing)")
    print(f"  best-fit servo lag         {M['lag_ms']:9.4f} ms "
          f"({M['lag_pct_gc']:.2f} % of the gait cycle)")
    print(f"  RMS error with lag removed {M['rms_err_deg_lag_removed']:9.4f} deg  "
          f"<- vs {M['rms_err_deg']:.4f} deg raw")
    print(f"  share of RMS error that is pure time shift  "
          f"{M['pct_err_from_lag']:.1f} %")
    if M["pct_err_from_lag"] > 60.0:
        print("  => the residual is mostly a TIME SHIFT, removable by advancing the "
              "reference\n     or adding feedforward. Do NOT raise kp to fix it.")
    print("\nROM / CLAMPING  (instruction 13)")
    print(f"  human reference exceeded the bench ROM "
          f"[{lo_deg:.2f}, {hi_deg:.2f}] deg : "
          f"{'YES' if A['exceeded_rom'] else 'NO'}"
          + (f"  ({A['n_below']} below, {A['n_above']} above)"
             if A["exceeded_rom"] else ""))
    print(f"  reference clamping actually applied to ctrl        : "
          f"{'YES' if M['ref_clamped_steps'] else 'NO'}"
          f"  ({M['ref_clamped_steps']} of {n} steps)")
    print("\nQUANTITY SEPARATION")
    print("  tau_sensor_Nm      = OSL bench ACTUATOR torque (fixed base, no body weight)")
    print("  human_knee_moment  = BIOMECHANICAL human joint moment, retained only")
    print("  These are different physical quantities and are never combined here.")
    if FLAGS:
        print(f"\nAUDIT FLAGS ({len(FLAGS)}) -- reference quality, not simulation failures")
        for f in FLAGS:
            print(f"  ! {f}")
    print(f"\noutputs\n  {p_csv}\n  {p_met}")

    if not args.no_plot:
        make_plots(p_csv, args.outdir, args.kp, args.kv, FLAGS)
    return 0


if __name__ == "__main__":
    sys.exit(main())
