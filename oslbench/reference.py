"""
oslbench.reference -- the human gait reference: load it, validate it, resample it onto
the bench timestep.

WHAT THE REFERENCE IS
    build/AB19_knee_gait_reference.csv -- subject AB19 of the Camargo et al. open
    dataset, levelground / ccw / normal walking, trial 01_01, ONE complete right gait
    cycle, extracted on a uniform 1 % gait-phase grid (101 rows, 0 -> 100 %).
    time_s 30.5700 -> 31.7750, so the stride period is T = 1.2050 s.
    The knee angle is already in the OSL flexion-positive convention; [R3] verifies
    that by landmark rather than assuming it.

WHAT THIS FILE DOES NOT DO
    No MuJoCo.  No control law.  No plotting.  It also never REPAIRS the data: the
    audit is a report.  `human_knee_moment` and `human_knee_power` are carried through
    verbatim, defects and all, and are never used as a bench torque.

WHY A SPLINE AND NOT np.interp
    101 samples over 1.2050 s is ~83 Hz.  Linear interpolation onto dt = 0.5 ms gives
    a piecewise-CONSTANT reference velocity, i.e. an 83 Hz staircase in a signal that
    is plotted and differentiated.  The natural cubic spline passes exactly through
    every measured sample (verified at the knots each run: ~1.4e-14 deg), so no
    measured value is altered.
"""

from __future__ import annotations

import csv
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_REFERENCE_CSV = os.path.join(ROOT, "build", "AB19_knee_gait_reference.csv")

SUBJECT = "AB19"
TRIAL = "levelground / ccw / normal / trial 01_01, one complete right gait cycle"
SUBJECT_MASS_KG = 68.0       # AB19, dataset Table 1 -- used ONLY by the [R4] audit
STANCE_END = 60.0            # % gait cycle, nominal toe-off, for shading/labels only

RAW_COLUMNS = ("gait_phase_percent", "time_s", "human_knee_angle_deg",
               "human_knee_moment", "human_knee_power")
RESAMPLED_COLUMNS = ("human_knee_angle_deg", "human_knee_moment", "human_knee_power")


class _Report:
    """Collects the audit transcript on the object instead of in module globals.

    The original scripts appended to module-level NOTES/FLAGS lists, which meant the
    demo imported the experiment *for its side effects*.  Same text, same order, now
    owned by the returned object.
    """

    def __init__(self, verbose: bool = True):
        self.notes: list[str] = []
        self.flags: list[str] = []
        self.verbose = bool(verbose)

    def section(self, title: str) -> None:
        if self.verbose:
            print(f"\n{title}")

    def note(self, msg: str) -> None:
        self.notes.append(msg)
        if self.verbose:
            print(f"    {msg}")

    def flag(self, msg: str) -> None:
        self.flags.append(msg)
        if self.verbose:
            print(f"    [FLAG] {msg}")


class GaitReference:
    """The AB19 CSV, verbatim.  Nothing is rescaled, resigned or repaired.

    raw          {column name: np.ndarray} for all five CSV columns
    n            number of samples (101)
    phase        gait_phase_percent, %
    time_s       absolute trial time, s
    angle_deg    human_knee_angle_deg, flexion-positive
    moment       human_knee_moment  -- UNIT UNDECLARED, see the [R4] audit flags
    power        human_knee_power   -- UNIT UNDECLARED, see the [R4] audit flags
    """

    def __init__(self, raw: dict, n: int, path: str):
        self.raw, self.n, self.path = raw, int(n), path

    def __getitem__(self, key):        # dict-style access, kept for the audit code
        return self.raw[key] if key != "n" else self.n

    @property
    def phase(self):
        return self.raw["gait_phase_percent"]

    @property
    def time_s(self):
        return self.raw["time_s"]

    @property
    def angle_deg(self):
        return self.raw["human_knee_angle_deg"]

    @property
    def moment(self):
        return self.raw["human_knee_moment"]

    @property
    def power(self):
        return self.raw["human_knee_power"]

    @property
    def period_s(self) -> float:
        """Stride period T, s.  1.2050 s for this trial."""
        return float(self.time_s[-1] - self.time_s[0])


class ReferenceAudit:
    """The result of `audit_reference` (a REPORT -- no column was modified).

    T             stride period, s
    exceeded_rom  True if any sample lies outside the bench knee ROM
    n_below       samples below the ROM lower bound
    n_above       samples above the ROM upper bound
    sign_ok       True if the flexion-positive landmarks check out
    peak_moment   peak |human_knee_moment| in the column's own (undeclared) unit
    peak_power    peak |human_knee_power|
    notes         the transcript, in order
    flags         the problems found, in order (5 fire on the shipped CSV)
    """

    def __init__(self, report: _Report, **kw):
        self._report = report
        self.__dict__.update(kw)

    @property
    def notes(self):
        return self._report.notes

    @property
    def flags(self):
        return self._report.flags

    @property
    def report(self):
        """The running transcript.  Handed to `BenchSimulation.reset()` so the [S1]
        initial-condition notes join the same ordered list as the reference notes."""
        return self._report

    def fields(self) -> dict:
        """The audit's numeric findings, in the order the metrics CSV stores them
        (ref_T, ref_exceeded_rom, ref_n_below, ref_n_above, ref_sign_ok,
        ref_peak_moment, ref_peak_power)."""
        return {k: v for k, v in self.__dict__.items() if not k.startswith("_")}


def load_reference(path: str | None = None) -> GaitReference:
    """Read the AB19 CSV verbatim.  Nothing is rescaled, resigned or repaired here."""
    path = os.path.abspath(path or DEFAULT_REFERENCE_CSV)
    with open(path, newline="") as fh:
        rows = list(csv.DictReader(fh))
    missing = [c for c in RAW_COLUMNS if c not in (rows[0] if rows else {})]
    if missing:
        sys.exit(f"FATAL: {path} is missing columns {missing}")
    raw = {c: np.array([float(r[c]) for r in rows]) for c in RAW_COLUMNS}
    return GaitReference(raw, len(rows), path)


def audit_reference(ref: GaitReference, knee_lo_deg: float, knee_hi_deg: float,
                    verbose: bool = True) -> ReferenceAudit:
    """Check the reference against the bench ROM and against biomechanical plausibility.

    This is a REPORT, not a repair: no column is modified.  Two of these checks exist
    because a reference that is silently wrong produces a tracking result that looks
    perfect and means nothing.
    """
    rep = _Report(verbose)
    note, flag = rep.note, rep.flag

    ph, t = ref["gait_phase_percent"], ref["time_s"]
    a, m, p = (ref["human_knee_angle_deg"], ref["human_knee_moment"],
               ref["human_knee_power"])
    n = ref.n
    T = float(t[-1] - t[0])

    rep.section("[R1] REFERENCE GRID")
    note(f"{n} samples, gait phase {ph[0]:g} -> {ph[-1]:g} %, "
         f"uniform 1 % grid = {np.allclose(np.diff(ph), 1.0)}")
    note(f"cycle duration {T:.4f} s -> stride frequency {1.0 / T:.3f} Hz "
         f"(sampled at {1.0 / float(np.mean(np.diff(t))):.1f} Hz)")
    if not np.all(np.diff(t) > 0):
        sys.exit("FATAL: time_s is not strictly increasing")
    if not (abs(ph[0]) < 1e-9 and abs(ph[-1] - 100.0) < 1e-9):
        flag(f"gait phase does not span exactly 0-100 % (got {ph[0]:g}-{ph[-1]:g})")

    rep.section("[R2] BENCH ROM CONTAINMENT")
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

    rep.section("[R3] FLEXION-SIGN LANDMARK CHECK  (the conversion is verified, not assumed)")
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

    rep.section("[R4] HUMAN MOMENT / POWER PLAUSIBILITY  (carried through, never used as "
                "bench torque)")
    pk_m, pk_p = float(np.max(np.abs(m))), float(np.max(np.abs(p)))
    note(f"peak |human_knee_moment| = {pk_m:.3f}  (column unit not declared in the CSV)")
    note(f"peak |human_knee_power|  = {pk_p:.3f}")
    note(f"if N.m       -> {pk_m / SUBJECT_MASS_KG:.3f} N.m/kg for {SUBJECT_MASS_KG:g} kg")
    note(f"if mN.m/kg   -> {pk_m / 1000.0:.3f} N.m/kg = "
         f"{pk_m / 1000.0 * SUBJECT_MASS_KG:.2f} N.m")
    # The Camargo dataset documents moment in N.m/kg and power in W/kg, so the expected
    # peak here is ~0.4-0.6 N.m/kg.  Anything else is an extraction scale error.
    if not 0.15 <= pk_m / SUBJECT_MASS_KG <= 1.5:
        flag(f"peak moment is {pk_m / SUBJECT_MASS_KG:.2f} N.m/kg if the column is N.m -- a "
             f"human walking knee peaks near 0.4-0.6 N.m/kg, so the moment column is "
             f"off by roughly {pk_m / SUBJECT_MASS_KG / 0.5:.0f}x or is in another unit "
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

    return ReferenceAudit(rep, T=T, exceeded_rom=exceeded, n_below=below, n_above=above,
                          sign_ok=ok_sign, peak_moment=pk_m, peak_power=pk_p)


# ==================================================== interpolation and resampling
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


class ResampledReference:
    """The reference on the bench timestep -- what the controller is actually given.

    time_s          s, starts at 0 (the trial's absolute t0 is subtracted)
    gait_phase_percent   %, computed ANALYTICALLY as 100*(t mod T)/T
    ref_rad         the commanded knee angle, rad  <-- this is q_ref
    ref_vel_rad_s   d(ref)/dt from the spline derivative, rad/s (reporting only)
    human_knee_angle_deg / human_knee_moment / human_knee_power   resampled, verbatim
    n / n_per_cycle / cycles / T / dt / interp
    """

    def __init__(self, cols: dict, n: int, n_per_cycle: int, cycles: int,
                 T: float, dt: float, interp: str):
        self._cols = cols
        self.n, self.n_per_cycle, self.cycles = int(n), int(n_per_cycle), int(cycles)
        self.T, self.dt, self.interp = float(T), float(dt), str(interp)

    def __getitem__(self, key):
        return self._cols[key]

    @property
    def ref_rad(self):
        return self._cols["ref_rad"]

    @property
    def ref_vel_rad_s(self):
        return self._cols["ref_vel_rad_s"]

    @property
    def time_s(self):
        return self._cols["time_s"]

    @property
    def phase(self):
        return self._cols["gait_phase_percent"]

    @property
    def angle_deg(self):
        return self._cols["human_knee_angle_deg"]

    @property
    def moment(self):
        return self._cols["human_knee_moment"]

    @property
    def power(self):
        return self._cols["human_knee_power"]


def resample_reference(ref: GaitReference, dt: float, cycles: int = 1,
                       kind: str = "cubic", audit: ReferenceAudit | None = None,
                       verbose: bool = True) -> ResampledReference:
    """Put the reference on the bench timestep, preserving the 0-100 % phase mapping
    exactly: phase is an affine function of time within a cycle, so it is recomputed
    analytically rather than interpolated.

    Notes and flags raised here are appended to `audit` when one is supplied, so the
    experiment's flag list stays a single ordered sequence.
    """
    rep = audit._report if audit is not None else _Report(verbose)
    note, flag = rep.note, rep.flag

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
    for c in RESAMPLED_COLUMNS:
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

    rep.section("[R5] RESAMPLE")
    note(f"{kind} interpolation, {ref.n} samples -> {n_cyc} steps/cycle "
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
    return ResampledReference(out, n, n_cyc, cycles, T, dt, kind)


def cycle_closure_at_100pct(res: ResampledReference) -> float:
    """Cycle-closure gap at the true 100 % knot, in degrees, as the audit reports it.
    The resampled grid stops at T-dt, so recover the 100 % value by extending one step."""
    dphi = float(res["gait_phase_percent"][-1] - res["gait_phase_percent"][-2])
    # linear extension of the last interval to phase 100 %; the reference is smooth there
    slope = float(res["ref_rad"][-1] - res["ref_rad"][-2])
    at100 = float(res["ref_rad"][-1]) + slope * (100.0 - res["gait_phase_percent"][-1]) / dphi
    return math.degrees(at100 - float(res["ref_rad"][0]))


# ============================================================ synthetic references
def _t(dur, dt):
    return np.arange(0.0, dur, dt)


def synthetic_references(dt: float, knee_lo: float, knee_hi: float) -> dict:
    """Synthetic, safe, entirely non-human reference motions (radians) for the gain sweep.

    Each entry is (t, ref, description, analysis_start_s).  analysis_start_s
    excludes the part of the trace where the error is dominated by an artefact of
    the reference rather than by the controller -- the instant of a step edge, or
    the first settling cycle of a sinusoid.  Reporting an RMS error that includes
    a step discontinuity would just re-report the step size.
    """
    refs = {}

    for amp in (10.0, 45.0):
        t = _t(1.5, dt)
        refs[f"step_{int(amp)}deg"] = (
            t, np.where(t < 0.2, 0.0, math.radians(amp)),
            f"hard {amp:.0f} deg step at t=0.2 s -- diagnostic only; a step is "
            f"infinitely fast so it WILL saturate at high kp",
            0.4)

    f = 0.9
    t = _t(4.0 / f, dt)
    refs["sine_gait"] = (
        t, math.radians(30.0) - math.radians(30.0) * np.cos(2 * math.pi * f * t),
        "0.9 Hz sinusoid, 0->60 deg: the amplitude and rate of a walking knee, "
        "but synthetic -- NOT the Camargo reference",
        2.0 / f)

    T, pre, post = 0.3, 0.1, 0.6
    t = _t(pre + T + post, dt)
    s = np.clip((t - pre) / T, 0.0, 1.0)
    refs["minjerk_60deg"] = (
        t, math.radians(60.0) * (10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5),
        "minimum-jerk 0->60 deg in 0.3 s: the fastest excursion a walking knee "
        "makes, smooth so it is achievable",
        0.05)

    f0, f1, dur = 0.2, 8.0, 10.0
    t = _t(dur, dt)
    ph = 2 * math.pi * (f0 * t + 0.5 * (f1 - f0) / dur * t ** 2)
    refs["chirp"] = (
        t, math.radians(30.0) + math.radians(10.0) * np.sin(ph),
        f"linear chirp {f0}->{f1} Hz at 10 deg amplitude about 30 deg: measures "
        f"closed-loop bandwidth and phase lag directly",
        0.2)

    # every sample must be inside ctrlrange, with margin
    for name, (t, r, _, _) in refs.items():
        lo, hi = float(np.min(r)), float(np.max(r))
        if lo < knee_lo + 1e-6 or hi > knee_hi - 1e-6:
            sys.exit(f"reference {name!r} spans [{math.degrees(lo):.2f}, "
                     f"{math.degrees(hi):.2f}] deg, outside the knee ROM "
                     f"[{math.degrees(knee_lo):.2f}, {math.degrees(knee_hi):.2f}]"
                     f" -- refusing to run")
    return refs
