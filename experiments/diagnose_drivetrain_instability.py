#!/usr/bin/env python3
"""
diagnose_drivetrain_instability.py -- DIAGNOSIS ONLY.  Nothing here is a fix.

WHY THIS SCRIPT EXISTS
    `experiments/run_bench_ab19_drivetrain.py` ran on real MuJoCo and Case B (drivetrain
    ON) diverged: MuJoCo reported NaN / Inf / huge QACC at DOF 0 at t = 0.6560 s, after
    which the belt-energy expression overflowed.  Case A (drivetrain OFF) was unaffected
    -- RMS 4.3616 deg, peak |tau| 16.0041 N.m, i.e. the frozen benchmark.

    A stub-plant run plus a hand-written 2-mass eigenvalue model produced a HYPOTHESIS:
    `PDCurrentSource` feeds the JOINT velocity into a current that acts on the ACTUATOR
    shaft, with the compliant belt in between, and that non-collocated derivative term
    pumps the belt mode instead of damping it.  A hypothesis from a stub is not a
    measurement.  This script measures it on the real engine.

WHAT IT IS ALLOWED TO TOUCH -- NOTHING
    No production file is modified.  `oslbench/controller.py`, `oslbench/drivetrain.py`,
    `oslbench/drivetrain_sim.py`, `models/osl_v2_bench.xml`, `tests/oracle/*` and the
    AB19 reference CSV are all read-only here.  Kp stays 600 and Kd stays 17.253 in
    production; the values swept below live only in local `PDController` objects created
    inside this file.  The two feedback variants in TEST C are DIAGNOSTIC-ONLY and are
    implemented in this file, never in `PDCurrentSource`.  The timestep in TEST D is
    changed on the in-memory `mjModel` for the duration of one run and the model is
    reloaded for every run, so no XML is edited and no run can inherit another's state.

THE SIX TESTS
    A  Kd sweep at Kp = 600, Kd in {0, 0.25, 0.5, 1, 1.25, 1.5, 2, 5, 10, 17.253}
       -> where is the empirical stability boundary in Kd?
    B  (Kp=600, Kd=0) vs (Kp=0, Kd=17.253) vs (Kp=600, Kd=17.253)
       -> is it really the derivative term, and is Kp innocent?
    C  Variant J (derivative on JOINT velocity -- the law that diverged, and what
       production shipped when this diagnosis was written) vs Variant A (derivative
       on joint-referred ACTUATOR velocity), both Kp=600, Kd=17.253
       -> is this a non-collocated feedback problem?
    D  h = 0.5 ms vs h = 0.25 ms on the most informative case
       -> is it an integration instability instead?
    E  per-step power and energy on the diverging case, including the work the
       derivative term does ON THE ACTUATOR -> is energy being injected, and by what?
    F  the dominant frequency of theta_s(t) before the runaway, measured, against the
       2-mass belt-mode prediction -> is the growing mode actually the belt mode?

HOW A RUN IS STOPPED BEFORE IT OVERFLOWS
    `BenchSimulation.run` has no abort, so this script drives `sim.reset`/`sim.step`
    itself -- still read-only use of the public API -- and checks four guards after
    every step.  Each guard is a stated physical absurdity, not a tuning knob:

        |tau_j|   > 10x the authored knee authority (1422 N.m)
        |theta_s| > the deflection that 1422 N.m implies (via paper eq. 12)
        |qdot|    > 50 rad/s, ~10x the AB19 reference peak of 5.05 rad/s
        |I_q|     > 400 A, ~13x the 30.93 A that the authored 142.2 N.m re-expresses to
        any recorded channel non-finite

    Crossing one of these ends the run at that step.  Values are therefore recorded
    while still finite and modest -- the diagnosis is read from the approach to
    divergence, which is where the mechanism is visible, not from the overflow.

    Separately, |theta_s| > 0.055 rad is FLAGGED but not fatal: that is the edge of the
    range Best et al. actually fitted the belt over (docs/DRIVETRAIN_PARAMETERS.md, and
    oslbench/drivetrain.py:585-586).  Past it the belt law is extrapolation of their
    regression, so the first crossing time is recorded for every run.

NOTHING HERE IS A HARDWARE RESULT
    The actuator parameters are PAPER-DERIVED from Best et al. 2025 (IEEE/ASME T-MECH
    30(6):4732-4743) and are not measurements of this hardware.  An instability found
    here is a property of this simulation -- a model written one way and closed one way
    -- and is NOT a claim that the OSL V2 is unstable.  NOT VALIDATED ON HARDWARE.

USAGE (Windows PowerShell)
    .venv\\Scripts\\python.exe experiments\\diagnose_drivetrain_instability.py
    .venv\\Scripts\\python.exe experiments\\diagnose_drivetrain_instability.py --steps 1600
"""

from __future__ import annotations

import argparse
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)
TESTS = os.path.join(ROOT, "tests")
if TESTS not in sys.path:
    sys.path.insert(0, TESTS)

import stub_mujoco                                                       # noqa: E402

MJ = stub_mujoco.install()          # real MuJoCo WINS if importable; stub is a fallback
USING_STUB = stub_mujoco.is_stub(MJ)

import numpy as np                                                       # noqa: E402

from oslbench import drivetrain as D                                     # noqa: E402
from oslbench.controller import KD, KP, PDController                      # noqa: E402
from oslbench.drivetrain import PAPER                                     # noqa: E402
from oslbench.drivetrain_sim import (DrivetrainBenchSimulation,           # noqa: E402
                                     DrivetrainLayer, PDCurrentSource,
                                     belt_potential_energy)
from oslbench.model import load_bench_model                               # noqa: E402
from oslbench.reference import (SUBJECT, TRIAL, audit_reference,          # noqa: E402
                                load_reference, resample_reference)

P = PAPER
OUTDIR = os.path.join(ROOT, "build", "drivetrain_instability")

# ------------------------------------------------------------------ the abort guards
KNEE_AUTHORITY_NM = 142.2            # the authored knee forcerange, models/osl_v2_bench.xml
GUARD_TAU_J_NM = 10.0 * KNEE_AUTHORITY_NM                       # 1422 N.m
GUARD_THETA_S_RAD = abs(D.deflection_for_joint_torque(GUARD_TAU_J_NM, P))
GUARD_QDOT_RAD_S = 50.0              # ~10x the AB19 reference peak joint speed (5.05)
GUARD_IQ_A = 400.0                   # ~13x 142.2/k_t_joint = 30.93 A
FIT_EDGE_RAD = 0.055                 # edge of the paper's fitted belt range -- FLAG only

# Test F only: how much of theta_s's RMS must sit in the detrended (oscillatory)
# part before a "frequency" is worth quoting.  A tracking-only run still has a
# nonzero detrended residual, and a frequency read off that residual is noise,
# not a mode.  This threshold is a JUDGEMENT CALL, not a derived quantity -- the
# raw fraction is always printed next to it so a reader can overrule it.
OSC_GATE = 0.35

# ------------------------------------------------------------- joint-referred 2-mass
J_AR = P.n_t ** 2 * P.J_a            # 0.208908143  kg.m^2, actuator inertia at the joint
B_AR = P.n_t ** 2 * P.B_a            # 1.287877260  N.m.s/rad
K_T_JOINT = P.n_t * P.k_t * P.n_a    # 4.597092     N.m/A  (paper-derived, not measured)
I_J = 0.261998                       # kg.m^2, knee-referred inertia INCL. armature 0.01
B_J = 0.3                            # N.m.s/rad, the knee dof's XML damping

KD_SWEEP = (0.0, 0.25, 0.5, 1.0, 1.25, 1.5, 2.0, 5.0, 10.0, KD)

# MuJoCo's OWN complaint counters.  The failure that started this was reported as
# "NaN / Inf / huge QACC at DOF 0", which is mjWARN_BADQACC.  Watching the counter
# directly is what makes "the instability was already visible before MuJoCo noticed"
# a measurement rather than an assertion.  Resolved by NAME so no index is hard-coded,
# and absent on the stub, where it degrades to 0.
_WARN = getattr(MJ, "mjtWarning", None)
BAD_WARNINGS = tuple(
    int(getattr(_WARN, nm)) for nm in ("mjWARN_BADQACC", "mjWARN_BADQPOS",
                                       "mjWARN_BADQVEL", "mjWARN_BADCTRL")
    if _WARN is not None and hasattr(_WARN, nm)
)


def warning_count(bench) -> int:
    """Total BADQACC/BADQPOS/BADQVEL/BADCTRL warnings MuJoCo has raised so far."""
    w = getattr(bench.data, "warning", None)
    if w is None or not BAD_WARNINGS:
        return 0
    try:
        return sum(int(w[i].number) for i in BAD_WARNINGS)
    except (TypeError, IndexError, AttributeError):
        return 0


# ==================================================================================
# THE DIAGNOSTIC FEEDBACK LAW  -- DIAGNOSTIC-ONLY, lives here and nowhere else
# ==================================================================================
class DiagCurrentSource:
    """`PDCurrentSource` with ONE argument swapped, and nothing else changed.

    Variant "J"  -- derivative feedback on the JOINT velocity theta_j_dot.
                    This is what `oslbench.drivetrain_sim.PDCurrentSource` does today.
                    Section 0 below asserts the two agree to the last bit, so TEST C is
                    comparing production behaviour against an alternative and not
                    against a reimplementation of itself.

    Variant "A"  -- derivative feedback on the JOINT-REFERRED ACTUATOR velocity,
                    theta_a_dot / n_t.  This is the collocated choice: the velocity is
                    measured on the same side of the belt as the torque is applied.

    WHY theta_a_dot / n_t AND NOT theta_a_dot
        Kd = 17.253 N.m.s/rad is a JOINT-side gain -- it was solved from a joint-side
        damping ratio (`controller.kd_for_damping_ratio(600, 0.261998, 0.3, 0.7)`).
        Feeding raw theta_a_dot would multiply the effective gain by n_t = 4.61 and so
        would change TWO things at once, which would make the comparison worthless.
        Dividing by n_t keeps the gain, the units and the nominal damping identical and
        changes only WHICH SIDE OF THE BELT the velocity is measured on.  That is the
        single variable TEST C is for.

    Everything else is byte-for-byte the production path: the same `PDController`, the
    same `torque_unclamped` call, the same clip into ctrlrange inside `command()`, the
    same division by the same paper-derived k_t_joint, and no current clamp (the paper
    documents none, so none is invented).
    """

    def __init__(self, pd: PDController, ref_rad, layer: DrivetrainLayer,
                 p=PAPER, variant: str = "J"):
        if variant not in ("J", "A"):
            raise ValueError("variant must be 'J' (joint velocity) or 'A' (actuator)")
        self.pd = pd
        self.ref = ref_rad
        self.layer = layer
        self.p = p
        self.variant = variant
        self.k_t_joint = p.n_t * p.k_t * p.n_a
        self.last_request_Nm = 0.0
        self.last_qdot_used = 0.0

    def __call__(self, k: int, theta_j: float, theta_j_dot: float) -> float:
        q_ref = float(self.ref[k if k >= 0 else 0])
        if self.variant == "J":
            qdot = theta_j_dot                               # production, verbatim
        else:
            qdot = self.layer.theta_a_dot / self.p.n_t       # collocated, DIAGNOSTIC
        self.last_qdot_used = qdot
        tau_req = self.pd.torque_unclamped(q_ref, theta_j, qdot)
        self.last_request_Nm = tau_req
        return tau_req / self.k_t_joint


# ==================================================================================
# ONE RUN
# ==================================================================================
class Trace:
    """Everything one run recorded, plus its verdict.  Channels are numpy at the end.

    Every layer-side channel is paired with the PRE-step joint state, because that is
    the configuration the belt and the controller were both evaluated at inside
    `DrivetrainBenchSimulation.step`.  Mixing pre- and post-step samples would make the
    power balance in TEST E look wrong when only the bookkeeping was.
    """

    CHANNELS = ("t", "ref_rad", "theta_j", "theta_j_dot", "theta_a", "theta_a_dot",
                "theta_s", "tau_j", "tau_a", "tau_m", "tau_f", "i_q", "K_s", "U",
                "tau_request", "qdot_used", "servo_force",
                "p_motor", "p_fric", "p_Ba", "p_belt_joint", "p_act_belt",
                "p_kd_actual", "p_kd_colloc")

    def __init__(self, label: str, kp: float, kd: float, variant: str, dt: float,
                 n_planned: int):
        self.label, self.kp, self.kd = label, float(kp), float(kd)
        self.variant, self.dt, self.n_planned = variant, float(dt), int(n_planned)
        self.d = {c: [] for c in self.CHANNELS}
        self.verdict = "STABLE"
        self.t_unstable = None
        self.reason = ""
        self.t_fit_exit = None          # first time |theta_s| left the fitted range
        self.t_mj_warning = None        # first time MuJoCo itself complained
        self.servo_violations = 0
        self.i_j = float("nan")         # joint-side inertia estimate used for predictions

    # ------------------------------------------------------------------- recording
    def push(self, **kw) -> None:
        for key, val in kw.items():
            self.d[key].append(float(val))

    def finish(self) -> "Trace":
        self.a = {k: np.asarray(v, float) for k, v in self.d.items()}
        self.n = len(self.a["t"])
        return self

    # --------------------------------------------------------------------- summary
    def peak(self, ch: str) -> float:
        v = self.a[ch]
        if v.size == 0:
            return float("nan")
        finite = v[np.isfinite(v)]
        return float(np.max(np.abs(finite))) if finite.size else float("inf")

    @property
    def stable(self) -> bool:
        return self.verdict == "STABLE"

    def classify(self) -> "Trace":
        """Three verdicts, not two.  A run that merely FINISHED is not thereby stable.

            UNSTABLE   a stated absurdity was reached and the run was cut short
            MARGINAL   it finished, but it asked the belt for more than the authored
                       knee authority (142.2 N.m) or drove the deflection outside the
                       range Best et al. actually fitted -- so it finished only because
                       the gait cycle ended first, and every torque it printed past
                       that point is extrapolation of the paper's regression
            STABLE     it finished inside both

        The MARGINAL band exists because a binary verdict reads a 1221 N.m run as a
        success, which would put the Kd boundary in the wrong place.
        """
        if self.verdict == "UNSTABLE":
            return self
        over_tau = self.peak("tau_j") > KNEE_AUTHORITY_NM
        if over_tau or self.t_fit_exit is not None:
            self.verdict = "MARGINAL"
            bits = []
            if over_tau:
                bits.append(f"|tau_j| peaked at {self.peak('tau_j'):.1f} N.m > "
                            f"{KNEE_AUTHORITY_NM:g} N.m authority")
            if self.t_fit_exit is not None:
                bits.append(f"theta_s left the fitted range at t={self.t_fit_exit:.4f} s")
            self.reason = "; ".join(bits)
        return self

    def row(self) -> dict:
        """The quantities the diagnosis table needs, in the order they were asked for."""
        return {
            "label": self.label,
            "kp": self.kp,
            "kd": self.kd,
            "variant": self.variant,
            "dt_s": self.dt,
            "verdict": self.verdict,
            "t_unstable_s": ("--" if self.t_unstable is None
                             else f"{self.t_unstable:.4f}"),
            "reason": self.reason or "--",
            "steps_run": self.n,
            "steps_planned": self.n_planned,
            "max_theta_j_deg": math.degrees(self.peak("theta_j")),
            "max_theta_a_rad": self.peak("theta_a"),
            "max_theta_s_rad": self.peak("theta_s"),
            "max_theta_j_dot_rad_s": self.peak("theta_j_dot"),
            "max_theta_a_dot_rad_s": self.peak("theta_a_dot"),
            "max_tau_j_Nm": self.peak("tau_j"),
            "max_i_q_A": self.peak("i_q"),
            "max_ctrl_out_Nm": self.peak("tau_request"),
            "max_belt_energy_J": self.peak("U"),
            "theta_s_fit_exit_s": ("never" if self.t_fit_exit is None
                                   else f"{self.t_fit_exit:.4f}"),
            "mujoco_warning_s": ("never" if self.t_mj_warning is None
                                 else f"{self.t_mj_warning:.4f}"),
            "rms_err_deg": self.rms_err_deg(),
            "servo_violations": self.servo_violations,
        }

    def rms_err_deg(self) -> float:
        """RMS tracking error over the steps that actually ran.  For an aborted run this
        is NOT comparable with Case A's 4.3616 deg -- it covers a shorter window and a
        diverging one.  Reported so the two are never silently compared."""
        if self.n == 0:
            return float("nan")
        e = np.degrees(self.a["ref_rad"] - self.a["theta_j"])
        e = e[np.isfinite(e)]
        return float(np.sqrt(np.mean(e ** 2))) if e.size else float("nan")


def guard_trip(rec: dict) -> str:
    """The stated physical absurdities.  Returns a reason string, or "" to continue."""
    for key, val in rec.items():
        if not math.isfinite(val):
            return f"non-finite {key}"
    if abs(rec["tau_j"]) > GUARD_TAU_J_NM:
        return f"|tau_j| {abs(rec['tau_j']):.1f} > {GUARD_TAU_J_NM:.1f} N.m (10x authority)"
    if abs(rec["theta_s"]) > GUARD_THETA_S_RAD:
        return (f"|theta_s| {abs(rec['theta_s']):.4f} > {GUARD_THETA_S_RAD:.4f} rad "
                f"({GUARD_THETA_S_RAD / FIT_EDGE_RAD:.1f}x the fitted range)")
    if abs(rec["theta_j_dot"]) > GUARD_QDOT_RAD_S:
        return f"|theta_j_dot| {abs(rec['theta_j_dot']):.1f} > {GUARD_QDOT_RAD_S:.0f} rad/s"
    if abs(rec["i_q"]) > GUARD_IQ_A:
        return f"|I_q| {abs(rec['i_q']):.1f} > {GUARD_IQ_A:.0f} A"
    return ""


def run_one(res, n: int, kp: float, kd: float, variant: str = "J",
            dt_override: float | None = None, label: str = "",
            use_production_source: bool = False) -> Trace:
    """One drivetrain-ON run, stopped at the first stated absurdity.

    A FRESH model is loaded every time.  Case B disconnects the knee actuator by writing
    zeros into the compiled mjModel, so sharing one model between runs would make the
    answer depend on the order the runs happened in -- the exact hidden coupling that
    produces a confident wrong number.  Two loads cost a second and remove the question.
    """
    bench = load_bench_model(verbose=False)
    if bench.failures:
        raise RuntimeError(f"preflight failed: {bench.failures}")

    if dt_override is not None:
        bench.model.opt.timestep = float(dt_override)      # in-memory only; XML untouched
    dt = float(bench.model.opt.timestep)

    layer = DrivetrainLayer(P, enabled=True, substeps=1)
    pd = PDController(kp, kd, bench.knee_ctrlrange, bench.knee_forcerange[1], "knee_diag")
    if use_production_source:
        src = PDCurrentSource(pd, res["ref_rad"], layer, P)  # the shipped class, verbatim
    else:
        src = DiagCurrentSource(pd, res["ref_rad"], layer, P, variant)
    sim = DrivetrainBenchSimulation(bench, pd, layer=layer, current_source=src,
                                    disconnect_servo=True)
    if sim.servo_connected:
        raise RuntimeError("the knee servo did not disconnect -- would be a double count")

    ref = np.asarray(res["ref_rad"], float)
    n = int(min(n, len(ref)))
    tr = Trace(label or f"kp{kp:g}_kd{kd:g}_{variant}", kp, kd, variant, dt, n)
    tr.i_j = float(bench.i_eff)

    sim.reset(float(ref[0]), ref_vel0=float(res["ref_vel_rad_s"][0]))
    sim.ankle_dev = 0.0

    data = bench.data
    warn0 = warning_count(bench)
    for k in range(n):
        # The PRE-step joint state: exactly the two numbers sim.step is about to read
        # and hand to the current source.  Read, never written.
        th_j = float(data.qpos[bench.knee_qpos])
        th_jd = float(data.qvel[bench.knee_dof])

        # The one failure the abort guards CANNOT catch, because it happens mid-step and
        # not between steps: `belt_potential_energy` cubes |theta_s|, so a runaway can
        # raise OverflowError inside drivetrain_sim before step() ever returns a number
        # for us to inspect.  That is the exact failure reported on the real engine
        # ("then the belt energy calculation overflows"), so it must be a recorded
        # outcome, not a traceback that kills the whole diagnostic run.  DIAGNOSTIC-ONLY
        # containment: it changes nothing about the physics, only about who reports it.
        try:
            sim.step(float(ref[k]), k)
        except (OverflowError, FloatingPointError, ValueError) as exc:
            tr.verdict = "UNSTABLE"
            tr.t_unstable = float(data.time)
            tr.reason = f"{type(exc).__name__} inside sim.step: {exc}"
            break
        L = sim.last_layer

        servo = sim.knee_servo_force()
        if servo != 0.0:
            tr.servo_violations += 1

        # ---- powers, all at the pre-step configuration the belt was evaluated at ----
        th_ad = L.theta_a_dot_pre
        p_motor = L.tau_m * th_ad                   # shaft mechanical input
        p_fric = L.tau_f * th_ad                    # paper eq (2) losses
        p_Ba = P.B_a * th_ad * th_ad                # viscous losses, >= 0 by construction
        p_belt_joint = L.tau_j * th_jd              # belt -> joint
        p_act_belt = L.tau_a * th_ad                # actuator -> belt
        # THE CENTREPIECE.  The derivative term contributes -Kd*qdot_used to the joint-
        # referred request, i.e. -Kd*qdot_used/n_t on the actuator shaft.  Its power on
        # the shaft is that torque times theta_a_dot.  Written joint-referred.
        # Both DiagCurrentSource and the shipped PDCurrentSource now expose
        # `last_qdot_used`, so this reads the velocity that was ACTUALLY fed back
        # instead of inferring it from which class was constructed -- one less place
        # for the two to drift apart.
        p_kd_actual = -(kd * src.last_qdot_used) * (th_ad / P.n_t)
        # and the collocated counterfactual, which is <= 0 for ALL signals by algebra:
        p_kd_colloc = -(kd * th_jd) * th_jd

        rec = dict(t=k * dt, ref_rad=float(ref[k]), theta_j=th_j, theta_j_dot=th_jd,
                   theta_a=L.theta_a_pre, theta_a_dot=th_ad, theta_s=L.theta_s,
                   tau_j=L.tau_j, tau_a=L.tau_a, tau_m=L.tau_m, tau_f=L.tau_f,
                   i_q=L.i_q, K_s=L.K_s, U=L.U,
                   tau_request=getattr(src, "last_request_Nm", 0.0),
                   qdot_used=getattr(src, "last_qdot_used", th_jd),
                   servo_force=servo,
                   p_motor=p_motor, p_fric=p_fric, p_Ba=p_Ba,
                   p_belt_joint=p_belt_joint, p_act_belt=p_act_belt,
                   p_kd_actual=p_kd_actual, p_kd_colloc=p_kd_colloc)

        if tr.t_fit_exit is None and abs(L.theta_s) > FIT_EDGE_RAD:
            tr.t_fit_exit = k * dt
        if tr.t_mj_warning is None and warning_count(bench) > warn0:
            tr.t_mj_warning = k * dt

        reason = guard_trip(rec)
        if not reason and tr.t_mj_warning is not None:
            reason = "MuJoCo raised BADQACC/BADQPOS/BADQVEL"
        tr.push(**rec)
        if reason:
            tr.verdict = "UNSTABLE"
            tr.t_unstable = k * dt
            tr.reason = reason
            break

    if not bench.limits_unchanged():
        raise RuntimeError("authored limits changed during the run")
    return tr.finish().classify()


# ==================================================================================
# ANALYSIS HELPERS  (self-contained: importing layer_dynamic_test would install the
# stub at import time, which must never happen on a run that is meant to be real)
# ==================================================================================
def detrend(x: np.ndarray, w: int) -> np.ndarray:
    """Subtract a centred moving average of width `w` samples.

    The AB19 gait fundamental is 0.83 Hz and the belt mode is 14-17 Hz, a 17-20x
    separation, so a moving average one belt period wide removes the slow tracking
    component and leaves the mode.  For an exponentially growing mode a little of the
    growth survives (of order sigma/omega ~ 0.1), which biases nothing that follows:
    the frequency comes from zero crossings and the growth rate from peak RATIOS.
    """
    w = max(3, int(w) | 1)
    if x.size < w + 2:
        return x - x.mean() if x.size else x
    pad = w // 2
    xp = np.concatenate([np.full(pad, x[0]), x, np.full(pad, x[-1])])
    ker = np.ones(w) / w
    return x - np.convolve(xp, ker, mode="valid")[:x.size]


def zero_cross_freq(xh: np.ndarray, dt: float) -> float:
    """Dominant frequency from sign changes: f = (crossings/2) / span.  Hysteresis at
    10 % of the RMS suppresses the chatter crossings that a stick-slip friction law
    produces, which would otherwise inflate f by a large factor."""
    if xh.size < 8:
        return float("nan")
    hyst = 0.1 * float(np.sqrt(np.mean(xh ** 2)))
    if hyst <= 0:
        return float("nan")
    state, idx = 0, []
    for i, v in enumerate(xh):
        if state >= 0 and v < -hyst:
            state = -1
            idx.append(i)
        elif state <= 0 and v > hyst:
            state = 1
            idx.append(i)
    if len(idx) < 3:
        return float("nan")
    span = (idx[-1] - idx[0]) * dt
    return (len(idx) - 1) / (2.0 * span) if span > 0 else float("nan")


def half_cycle_peaks(xh: np.ndarray, dt: float):
    """(times, |peak|) one per half cycle, using the same hysteresis crossings."""
    if xh.size < 8:
        return np.array([]), np.array([])
    hyst = 0.1 * float(np.sqrt(np.mean(xh ** 2)))
    if hyst <= 0:
        return np.array([]), np.array([])
    state, bounds = 0, [0]
    for i, v in enumerate(xh):
        if state >= 0 and v < -hyst:
            state, _ = -1, bounds.append(i)
        elif state <= 0 and v > hyst:
            state, _ = 1, bounds.append(i)
    bounds.append(xh.size)
    ts, pk = [], []
    for lo, hi in zip(bounds[:-1], bounds[1:]):
        if hi - lo < 3:
            continue
        seg = np.abs(xh[lo:hi])
        j = int(np.argmax(seg))
        ts.append((lo + j) * dt)
        pk.append(float(seg[j]))
    return np.asarray(ts), np.asarray(pk)


def growth_rate(ts: np.ndarray, pk: np.ndarray) -> float:
    """sigma [1/s] from a least-squares fit of log(peak) against time.  Positive means
    the oscillation is growing.  Returns nan if there is nothing to fit."""
    m = (pk > 0) & np.isfinite(pk)
    if m.sum() < 3:
        return float("nan")
    return float(np.polyfit(ts[m], np.log(pk[m]), 1)[0])


def belt_mode_hz(k_s: float, i_j: float) -> float:
    """Undamped 2-mass belt mode, joint-referred:  f = sqrt(K_s*(1/J_ar + 1/I_j))/2pi.

    DERIVED, not measured.  The two inertias are the actuator reflected through n_t^2
    and the joint-side effective inertia; K_s is the belt's LOCAL stiffness, which rises
    with |theta_s|, so this frequency is amplitude dependent by construction."""
    if k_s <= 0 or i_j <= 0:
        return float("nan")
    return math.sqrt(k_s * (1.0 / J_AR + 1.0 / i_j)) / (2.0 * math.pi)


def closed_loop_A(kp: float, kd: float, k_s: float, variant: str) -> np.ndarray:
    """Linear closed-loop state matrix of the Option-C architecture, joint-referred.

    State x = [theta_j, theta_ar, theta_j_dot, theta_ar_dot], where theta_ar =
    theta_a/n_t is the actuator angle referred to the joint.  Linearised about
    theta_s = 0, so K_s is a CONSTANT here -- the real belt hardens.

    The two equations are exactly what the code does, written down:

      joint      I_j  * theta_j_ddot  = K_s*(theta_ar - theta_j) - b_j*theta_j_dot
      actuator   J_ar * theta_ar_ddot = tau_req - B_ar*theta_ar_dot
                                        - K_s*(theta_ar - theta_j)

    `tau_j = -sgn(theta_s)*rho(|theta_s|)` (drivetrain.py `joint_torque_from_deflection`)
    linearises to `-K_s*theta_s = K_s*(theta_ar - theta_j)` on the joint, and the belt is
    massless so the reaction referred to the joint is equal and opposite.  The motor
    torque reaches ONLY the actuator row: that is the whole point of Option C, and the
    reason the feedback can be non-collocated at all.

      variant "J":  tau_req = kp*(ref - theta_j) - kd*theta_j_dot    <-- DIVERGED
      variant "A":  tau_req = kp*(ref - theta_j) - kd*theta_ar_dot   <-- collocated

    Dropped by linearising: Coulomb friction (f_c, f_g, joint frictionloss 0.4),
    gravity, the knee/ankle inertial coupling, the forcerange clamp, and MuJoCo's
    implicitfast velocity term.  A boundary from this is INDICATIVE, not exact."""
    kdj = kd if variant == "J" else 0.0
    kda = kd if variant == "A" else 0.0
    return np.array([
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
        [-k_s / I_J, k_s / I_J, -B_J / I_J, 0.0],
        [(k_s - kp) / J_AR, -k_s / J_AR, -kdj / J_AR, -(B_AR + kda) / J_AR],
    ])


def max_real_eig(kp: float, kd: float, k_s: float, variant: str) -> float:
    """Largest real part of the closed-loop eigenvalues.  > 0 means exponential growth.
    DERIVED from the linear model above, not measured."""
    return float(np.max(np.real(np.linalg.eigvals(closed_loop_A(kp, kd, k_s, variant)))))


def kd_boundary(kp: float, k_s: float, variant: str,
                hi: float = 200.0, tol: float = 1e-4, eps: float = 1e-9):
    """Smallest Kd > 0 at which max Re(lambda) crosses zero, by bisection.

    Returns None when no crossing exists in [0, hi] -- which is itself the answer for a
    feedback path that does not destabilise.  Also returns None if the system is ALREADY
    unstable at Kd = 0, since then Kd is not the mechanism.

    `eps` matters: with kp = 0 there is no position feedback, so the closed loop has a
    genuine ZERO eigenvalue (a free rigid-body mode) and `eigvals` returns it as +-1e-16
    dust.  Testing against 0.0 exactly would read that dust as "already unstable" and
    hide the answer.  A marginal case bisects down to ~0, which is the correct and more
    informative statement: ANY derivative feedback at all destabilises it."""
    if max_real_eig(kp, 0.0, k_s, variant) > eps:
        return None
    if max_real_eig(kp, hi, k_s, variant) <= 0.0:
        return None
    lo = 0.0
    while hi - lo > tol:
        mid = 0.5 * (lo + hi)
        if max_real_eig(kp, mid, k_s, variant) > 0.0:
            hi = mid
        else:
            lo = mid
    return 0.5 * (lo + hi)


def analysis_window(tr: Trace, frac: float = 0.35):
    """The last `frac` of the steps that ran -- the approach to divergence, which is
    where the growing mode dominates the slow tracking signal."""
    if tr.n < 40:
        return slice(0, tr.n)
    return slice(int(tr.n * (1.0 - frac)), tr.n)


# ==================================================================================
# OUTPUT
# ==================================================================================
def write_table(path: str, rows: list, note: str) -> str:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    if not rows:
        return path
    keys = list(rows[0].keys())
    with open(path, "w", newline="") as fh:
        fh.write(f"# {note}\n")
        fh.write(f"# engine={'STUB' if USING_STUB else 'MUJOCO'}  "
                 f"DIAGNOSTIC ONLY -- no production file was modified\n")
        fh.write("# BENCH ACTUATOR / BELT quantities. No human knee moment here. "
                 "NOT VALIDATED ON HARDWARE.\n")
        fh.write(",".join(keys) + "\n")
        for r in rows:
            fh.write(",".join(f"{r[k]:.10g}" if isinstance(r[k], float) else str(r[k])
                              for k in keys) + "\n")
    return path


def write_trace(path: str, tr: Trace, note: str) -> str:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    keys = list(Trace.CHANNELS)
    with open(path, "w", newline="") as fh:
        fh.write(f"# {note}\n")
        fh.write(f"# engine={'STUB' if USING_STUB else 'MUJOCO'}  kp={tr.kp:g} "
                 f"kd={tr.kd:g} variant={tr.variant} dt={tr.dt:g} "
                 f"verdict={tr.verdict} t_unstable={tr.t_unstable}\n")
        fh.write("# BENCH ACTUATOR / BELT quantities. No human knee moment here.\n")
        fh.write(",".join(keys) + "\n")
        for i in range(tr.n):
            fh.write(",".join(f"{tr.a[k][i]:.12g}" for k in keys) + "\n")
    return path


def head(num, title: str) -> None:
    print()
    print("=" * 78)
    print(f"[{num}] {title}")
    print("=" * 78)


def md_table(rows: list, cols: list, hdr: list, fmt: dict) -> list:
    out = ["| " + " | ".join(hdr) + " |",
           "|" + "|".join("---" for _ in hdr) + "|"]
    for r in rows:
        cells = []
        for c in cols:
            v = r[c]
            cells.append(fmt.get(c, "{}").format(v) if isinstance(v, float) else str(v))
        out.append("| " + " | ".join(cells) + " |")
    return out


# ==================================================================================
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--csv", default=None, help="AB19 reference CSV")
    ap.add_argument("--outdir", default=OUTDIR)
    ap.add_argument("--steps", type=int, default=0,
                    help="cap the steps per run (0 = the whole gait cycle)")
    ap.add_argument("--cycles", type=int, default=1)
    ap.add_argument("--interp", choices=("cubic", "linear"), default="cubic")
    args = ap.parse_args()
    os.makedirs(args.outdir, exist_ok=True)

    print("=" * 78)
    print("DRIVETRAIN INSTABILITY DIAGNOSIS -- DIAGNOSIS ONLY, NOTHING IS FIXED HERE")
    print("=" * 78)
    print(f"  engine        : {'STUB (NOT A RESULT)' if USING_STUB else 'REAL MuJoCo'}")
    print(f"  subject/trial : {SUBJECT} / {TRIAL}")
    print(f"  production    : Kp={KP:g} N.m/rad  Kd={KD:g} N.m.s/rad  -- UNCHANGED")
    print(f"  k_t_joint     : {K_T_JOINT:.6f} N.m/A   (paper-derived, not measured)")
    print(f"  J_ar, B_ar    : {J_AR:.9f} kg.m^2, {B_AR:.9f} N.m.s/rad (joint-referred)")
    print(f"  abort guards  : |tau_j|>{GUARD_TAU_J_NM:.1f} N.m  "
          f"|theta_s|>{GUARD_THETA_S_RAD:.4f} rad  "
          f"|qdot|>{GUARD_QDOT_RAD_S:.0f} rad/s  |I_q|>{GUARD_IQ_A:.0f} A")
    print(f"  belt fit edge : |theta_s| = {FIT_EDGE_RAD} rad  (flag, not fatal)")
    if USING_STUB:
        print()
        print("  *** real MuJoCo is NOT importable here.  Every number below is from")
        print("  *** tests/stub_mujoco.py and is a WIRING CHECK, NOT A MEASUREMENT.")

    bench0 = load_bench_model()
    if bench0.failures:
        print(f"\nREFUSING TO RUN: {len(bench0.failures)} preflight failure(s)")
        for f in bench0.failures:
            print(f"  - {f}")
        return len(bench0.failures)
    dt0 = bench0.timestep
    lo_deg, hi_deg = (math.degrees(x) for x in bench0.knee_ctrlrange)

    ref = load_reference(args.csv)
    audit = audit_reference(ref, lo_deg, hi_deg, verbose=False)
    res = resample_reference(ref, dt0, args.cycles, args.interp, audit=audit,
                             verbose=False)
    n_full = res.n if args.steps <= 0 else min(args.steps, res.n)
    print(f"\n  reference     : {res.n} samples at dt={dt0} s; running {n_full} per case")

    md = []     # the machine-generated report body
    md.append("# Drivetrain instability -- measured tables")
    md.append("")
    md.append(f"Engine: **{'STUB (NOT A RESULT)' if USING_STUB else 'REAL MuJoCo'}**. "
              f"Generated by `experiments/diagnose_drivetrain_instability.py`. "
              f"DIAGNOSIS ONLY -- no production file was modified. "
              f"Bench actuator/belt quantities; NOT VALIDATED ON HARDWARE.")
    md.append("")

    ROW = ["kd", "verdict", "t_unstable_s", "max_theta_j_deg", "max_theta_a_rad",
           "max_theta_s_rad", "max_theta_j_dot_rad_s", "max_theta_a_dot_rad_s",
           "max_tau_j_Nm", "max_i_q_A", "max_ctrl_out_Nm", "max_belt_energy_J",
           "theta_s_fit_exit_s", "mujoco_warning_s"]
    HDR = ["Kd", "verdict", "t_unstable s", "max th_j deg", "max th_a rad",
           "max th_s rad", "max th_j_dot", "max th_a_dot", "max tau_j Nm",
           "max I_q A", "max ctrl Nm", "max U J", "th_s left fit s",
           "MuJoCo warned s"]
    FMT = {"kd": "{:.3f}", "max_theta_j_deg": "{:.2f}", "max_theta_a_rad": "{:.4f}",
           "max_theta_s_rad": "{:.5f}", "max_theta_j_dot_rad_s": "{:.3f}",
           "max_theta_a_dot_rad_s": "{:.3f}", "max_tau_j_Nm": "{:.2f}",
           "max_i_q_A": "{:.2f}", "max_ctrl_out_Nm": "{:.2f}",
           "max_belt_energy_J": "{:.4g}"}

    # ============================================================== [0] EQUIVALENCE
    head(0, "EQUIVALENCE: which variant IS the shipped PDCurrentSource?")
    print("  TEST C only means something if one of its variants is production arithmetic")
    print("  bit for bit.  Both variants and the shipped class are fed the same")
    print("  (k, theta_j, theta_j_dot, theta_a_dot) states and must agree to the bit.")
    print("  theta_a_dot is varied too, which is what makes J and A distinguishable: a")
    print("  sweep at theta_a_dot = 0 would let a collocated law masquerade as J.")
    pd_chk = PDController(KP, KD, bench0.knee_ctrlrange, bench0.knee_forcerange[1])
    layer_chk = DrivetrainLayer(P, enabled=True, substeps=1)
    prod = PDCurrentSource(pd_chk, res["ref_rad"], layer_chk, P)
    diag = {v: DiagCurrentSource(pd_chk, res["ref_rad"], layer_chk, P, v)
            for v in ("J", "A")}
    worst = {"J": 0.0, "A": 0.0}
    nchk = 0
    for k in range(0, res.n, 7):
        for q in (-0.1, 0.0, 0.3, 1.2, 2.5):
            for qd in (-7.0, -0.4, 0.0, 0.4, 7.0):
                for th_ad in (-30.0, -2.0, 0.0, 2.0, 30.0):
                    layer_chk.theta_a_dot = th_ad
                    a = prod(k, q, qd)
                    for v in ("J", "A"):
                        worst[v] = max(worst[v], abs(a - diag[v](k, q, qd)))
                    nchk += 1
    print(f"  {nchk} states compared")
    for v in ("J", "A"):
        tag = "   <-- EXACT MATCH" if worst[v] == 0.0 else ""
        print(f"    vs variant {v}: worst |difference| = {worst[v]:.3e} A{tag}")
    matched = [v for v in ("J", "A") if worst[v] == 0.0]
    # The production class DECLARES which side it reads.  Cross-check the declaration
    # against the measured arithmetic: a stale constant would be worse than none.
    side = getattr(PDCurrentSource, "FEEDBACK_SIDE", "?")
    declared = {"joint": "J", "actuator": "A"}.get(side, "?")
    print(f"  PDCurrentSource.FEEDBACK_SIDE declares '{side}' -> variant {declared}")
    if len(matched) != 1 or matched[0] != declared:
        print(f"  *** PDCurrentSource matches {matched or 'NEITHER variant'} but "
              f"declares {declared}.  TEST C IS INVALID.  Stopping. ***")
        return 1
    prod_variant = matched[0]
    meaning = ("JOINT velocity -- the non-collocated law that diverged"
               if prod_variant == "J" else
               "ACTUATOR velocity, collocated -- the law the diagnosis recommended")
    print(f"  production == variant {prod_variant}  ({meaning})")
    md.append("## 0. Which variant is production arithmetic")
    md.append("")
    md.append(f"The shipped `PDCurrentSource` and both `DiagCurrentSource` variants were "
              f"fed {nchk} identical `(k, theta_j, theta_j_dot, theta_a_dot)` states, "
              f"with `theta_a_dot` varied so the two laws are distinguishable. Worst "
              f"difference vs variant J: **{worst['J']:.3e} A**; vs variant A: "
              f"**{worst['A']:.3e} A**. Production is therefore **variant "
              f"{prod_variant}**, matching its own `FEEDBACK_SIDE = '{side}'` "
              f"declaration. So TEST C compares production behaviour against an "
              f"alternative, not against a reimplementation.")
    md.append("")

    # ======================================================== [P] ANALYTIC PREDICTION
    head("P", "ANALYTIC PREDICTION -- engine-independent, pre-registers A/B/C")
    print("  Linear 2-mass model of the SAME architecture, about theta_s = 0.  This uses")
    print("  no engine at all, so it is valid evidence regardless of which one ran, and")
    print("  it predicts Tests A, B and C BEFORE they are read.  Linearised: Coulomb")
    print("  friction (f_c, f_g, joint frictionloss 0.4), gravity and the belt hardening")
    print("  are all dropped, so this LOCATES a boundary, it does not replace the runs.")
    print()
    print(f"  I_j   = {I_J:.6f} kg.m^2 (incl. armature)   b_j  = {B_J:.3f} N.m.s/rad")
    print(f"  J_ar  = {J_AR:.9f} kg.m^2              B_ar = {B_AR:.9f} N.m.s/rad")
    print()
    rows_p = []
    for ks_label, ks in (("K_s(0) = p1", P.p1),
                         ("K_s at the 16.0 N.m bench load",
                          D.belt_stiffness_at_torque(16.004120587370423, P)),
                         ("K_s at the 0.055 rad fit edge",
                          D.belt_stiffness(FIT_EDGE_RAD, P))):
        f_hz = belt_mode_hz(ks, I_J)
        kd_b_j = kd_boundary(KP, ks, "J")
        kd_b_a = kd_boundary(KP, ks, "A")
        print(f"  {ks_label:34s}  K_s={ks:9.3f}  open-loop belt mode {f_hz:6.2f} Hz")
        for kd in (0.0, 1.0, 5.0, KD):
            re_j = max_real_eig(KP, kd, ks, "J")
            re_a = max_real_eig(KP, kd, ks, "A")
            print(f"      Kd={kd:8.3f}   max Re(lambda): joint-vel {re_j:+9.4f}   "
                  f"actuator-vel {re_a:+9.4f}")
        print(f"      Kd boundary: joint-velocity  Kd < "
              f"{('none found' if kd_b_j is None else f'{kd_b_j:.4f}')}"
              f"   |   actuator-velocity  Kd < "
              f"{('no limit in [0, 200]' if kd_b_a is None else f'{kd_b_a:.4f}')}")
        rows_p.append(dict(K_s_case=ks_label, K_s_Nm_rad=ks,
                           belt_mode_Hz=f_hz,
                           kd_boundary_jointvel=(float("nan") if kd_b_j is None
                                                 else kd_b_j),
                           kd_boundary_actvel=(float("nan") if kd_b_a is None
                                               else kd_b_a),
                           maxRe_kd0_jointvel=max_real_eig(KP, 0.0, ks, "J"),
                           maxRe_kdprod_jointvel=max_real_eig(KP, KD, ks, "J"),
                           maxRe_kdprod_actvel=max_real_eig(KP, KD, ks, "A"),
                           kd_prod_over_boundary=(float("nan") if not kd_b_j
                                                  else KD / kd_b_j)))
        print()
    kd_b0 = kd_boundary(KP, P.p1, "J")
    if kd_b0:
        print(f"  PRE-REGISTERED, before any run below is read: at Kp={KP:g} with")
        print(f"  K_s(0)={P.p1:g}, joint-velocity feedback is predicted unstable for")
        print(f"  Kd > {kd_b0:.4f}.  Production Kd = {KD:.3f} is {KD / kd_b0:.1f}x that.")
        print(f"  So Test A's boundary should land near Kd ~ {kd_b0:.2f}, and Test C's")
        print(f"  actuator-velocity variant should survive the SAME Kd that kills the")
        print(f"  joint-velocity one.  If it does not, this explanation is WRONG and")
        print(f"  the doc must say so.")
    write_table(os.path.join(args.outdir, "diag_P_analytic.csv"), rows_p,
                "ANALYTIC PREDICTION -- linear 2-mass eigenvalues, no engine involved")
    md.append("## P. Analytic prediction (engine-independent)")
    md.append("")
    md += md_table(rows_p, ["K_s_case", "K_s_Nm_rad", "belt_mode_Hz",
                            "kd_boundary_jointvel", "kd_boundary_actvel",
                            "maxRe_kd0_jointvel", "maxRe_kdprod_jointvel",
                            "maxRe_kdprod_actvel", "kd_prod_over_boundary"],
                  ["K_s case", "K_s N.m/rad", "belt mode Hz",
                   "Kd bound (joint vel)", "Kd bound (act vel)",
                   "max Re, Kd=0", f"max Re, Kd={KD:.3f}",
                   f"max Re, Kd={KD:.3f} act-vel", "Kd_prod / bound"],
                  {"K_s_Nm_rad": "{:.3f}", "belt_mode_Hz": "{:.3f}",
                   "kd_boundary_jointvel": "{:.4f}", "kd_boundary_actvel": "{:.4f}",
                   "maxRe_kd0_jointvel": "{:+.4f}", "maxRe_kdprod_jointvel": "{:+.4f}",
                   "maxRe_kdprod_actvel": "{:+.4f}", "kd_prod_over_boundary": "{:.1f}"})
    md.append("")
    md.append("Linearised about `theta_s = 0`; Coulomb friction, gravity and belt "
              "hardening dropped. `nan` in a boundary column means no sign change was "
              "found in `Kd` over `[0, 200]` -- that feedback path did not go unstable "
              "anywhere in the searched range.")
    md.append("")

    # ================================================================== [A] Kd SWEEP
    head("A", "Kd SWEEP at Kp = 600 -- where is the boundary?")
    rows_a, traces_a = [], {}
    for kd in KD_SWEEP:
        tr = run_one(res, n_full, KP, kd, "J", label=f"A_kd{kd:g}")
        traces_a[kd] = tr
        rows_a.append(tr.row())
        print(f"  Kd={kd:8.3f}  {tr.verdict:8s}  t_unstable="
              f"{('--' if tr.t_unstable is None else f'{tr.t_unstable:.4f} s'):>10s}  "
              f"max|th_s|={tr.peak('theta_s'):9.5f} rad  "
              f"max|tau_j|={tr.peak('tau_j'):10.2f} N.m  "
              f"max|I_q|={tr.peak('i_q'):8.2f} A")
        if tr.reason:
            print(f"                 stopped by: {tr.reason}")
    write_table(os.path.join(args.outdir, "diag_A_kd_sweep.csv"), rows_a,
                "TEST A -- Kd sweep at Kp=600, drivetrain ON, variant J "
                "(joint velocity -- the law that diverged)")

    stable_kd = [kd for kd in KD_SWEEP if traces_a[kd].stable]
    unstable_kd = [kd for kd in KD_SWEEP if not traces_a[kd].stable]
    if stable_kd and unstable_kd:
        lo, hi = max(stable_kd), min(unstable_kd)
        print(f"\n  empirical boundary: fully STABLE up to Kd = {lo:g}; first Kd that "
              f"is no longer STABLE = {hi:g}  ->  boundary in ({lo:g}, {hi:g})")
        print(f"  production Kd = {KD:g} is {KD / hi:.1f}x the smallest non-stable "
              f"value tested")
        bnd = f"({lo:g}, {hi:g})"
    else:
        bnd = "not bracketed by the swept values"
        print(f"\n  boundary {bnd}")

    # The run-based bracket is an UPPER bound on the true boundary: a Kd whose growth
    # rate is slow simply cannot reveal itself inside one 1.205 s gait cycle, so it is
    # scored STABLE for want of time rather than for want of a growing mode.  Pricing
    # that directly keeps the run bracket and the analytic boundary from looking like a
    # contradiction when they are not.
    t_win = n_full * dt0
    ks_ref = D.belt_stiffness_at_torque(16.004120587370423, P)
    print(f"\n  how much growth could the {t_win:.3f} s window even show?  (linear "
          f"model, K_s={ks_ref:.1f})")
    rows_a2 = []
    for kd in KD_SWEEP:
        sig = max_real_eig(KP, kd, ks_ref, "J")
        g = math.exp(sig * t_win) if sig * t_win < 700 else float("inf")
        rows_a2.append(dict(kd=kd, verdict=traces_a[kd].verdict,
                            sigma_pred_per_s=sig,
                            e_fold_s=(1.0 / sig if sig > 0 else float("inf")),
                            growth_over_window=g))
        print(f"      Kd={kd:8.3f}  {traces_a[kd].verdict:8s}  predicted sigma="
              f"{sig:+8.3f} /s  e-folding="
              f"{('n/a (decaying)' if sig <= 0 else f'{1.0 / sig:.4f} s'):>16s}  "
              f"growth over window = {g:.3e}")
    print("      A predicted sigma > 0 scored STABLE means the window was too short,")
    print("      NOT that the mode is absent.  Read the run bracket as an upper bound.")
    write_table(os.path.join(args.outdir, "diag_A_growth_budget.csv"), rows_a2,
                "TEST A -- predicted growth rate vs what a one-gait-cycle window can show")

    md.append("## A. Kd sweep at Kp = 600 (variant J = joint-velocity feedback)")
    md.append("")
    md += md_table(rows_a, ROW, HDR, FMT)
    md.append("")
    md.append(f"Empirical stability boundary in Kd: **{bnd}**. Production "
              f"`Kd = {KD:g}`.")
    md.append("")
    md.append(f"### A2. Why that bracket is an upper bound")
    md.append("")
    md += md_table(rows_a2, ["kd", "verdict", "sigma_pred_per_s", "e_fold_s",
                             "growth_over_window"],
                   ["Kd", "run verdict", "predicted sigma /s", "e-folding s",
                    f"growth over {t_win:.3f} s"],
                   {"kd": "{:.3f}", "sigma_pred_per_s": "{:+.3f}",
                    "e_fold_s": "{:.4f}", "growth_over_window": "{:.3e}"})
    md.append("")
    md.append(f"The test window is one gait cycle, {t_win:.3f} s. A `Kd` with a small "
              f"positive growth rate is scored STABLE because the window ends first, "
              f"not because the mode is absent -- so the bracket above is an **upper "
              f"bound** on the true boundary, and the analytic boundary in section P "
              f"is the sharper number.")
    md.append("")

    # =============================================================== [B] IS IT Kd?
    head("B", "IS IT REALLY THE DERIVATIVE TERM?")
    cases_b = (("Kp=600, Kd=0", KP, 0.0), ("Kp=0, Kd=17.253", 0.0, KD),
               ("Kp=600, Kd=17.253", KP, KD))
    rows_b, traces_b = [], {}
    for name, kp, kd in cases_b:
        tr = run_one(res, n_full, kp, kd, "J", label=f"B_{kp:g}_{kd:g}")
        traces_b[name] = tr
        r = tr.row()
        r["case"] = name
        rows_b.append(r)
        print(f"  {name:22s}  {tr.verdict:8s}  t_unstable="
              f"{('--' if tr.t_unstable is None else f'{tr.t_unstable:.4f} s'):>10s}  "
              f"max|th_s|={tr.peak('theta_s'):9.5f} rad  "
              f"RMS(window)={tr.rms_err_deg():8.3f} deg")
    write_table(os.path.join(args.outdir, "diag_B_kp_kd.csv"), rows_b,
                "TEST B -- Kp/Kd isolation, drivetrain ON, variant J")
    md.append("## B. Is it really the derivative term?")
    md.append("")
    md += md_table(rows_b, ["case"] + ROW[1:], ["case"] + HDR[1:], FMT)
    md.append("")
    md.append("Read this as an isolation test only. A stable `Kd = 0` does NOT mean the "
              "shipped controller should use `Kd = 0`: with no derivative term the loop "
              "has only the MJCF joint damping (0.3 N.m.s/rad) left, which is not a "
              "controller design.")
    md.append("")

    # =================================================== [C] JOINT vs ACTUATOR VEL
    head("C", "JOINT vs ACTUATOR VELOCITY -- the collocation question")
    print("  Both at Kp=600, Kd=17.253.  The ONLY difference is which velocity the")
    print("  derivative term reads: theta_j_dot (joint) or theta_a_dot/n_t (actuator).")
    print(f"  Section 0 resolved production to variant {prod_variant}; the labels below")
    print("  name the SIGNAL, not which one ships, so this stays true after a fix.")
    rows_c, traces_c = [], {}
    for variant, name in (("J", "Variant J -- joint velocity (non-collocated)"),
                          ("A", "Variant A -- actuator velocity / n_t (collocated)")):
        tr = run_one(res, n_full, KP, KD, variant, label=f"C_{variant}")
        traces_c[variant] = tr
        r = tr.row()
        r["case"] = name
        rows_c.append(r)
        print(f"  {name:48s}")
        print(f"      {tr.verdict:8s}  t_unstable="
              f"{('--' if tr.t_unstable is None else f'{tr.t_unstable:.4f} s'):>10s}  "
              f"max|th_s|={tr.peak('theta_s'):9.5f} rad  "
              f"max|tau_j|={tr.peak('tau_j'):10.2f} N.m  "
              f"RMS(window)={tr.rms_err_deg():7.3f} deg")
    write_table(os.path.join(args.outdir, "diag_C_variants.csv"), rows_c,
                "TEST C -- joint vs actuator velocity in the derivative term "
                "(DIAGNOSTIC-ONLY variants)")
    write_trace(os.path.join(args.outdir, "diag_trace_variantJ.csv"), traces_c["J"],
                "TEST C variant J -- per-step trace (joint-velocity feedback law)")
    write_trace(os.path.join(args.outdir, "diag_trace_variantA.csv"), traces_c["A"],
                "TEST C variant A -- per-step trace (DIAGNOSTIC-ONLY collocated law)")
    md.append("## C. Joint velocity vs actuator velocity (diagnostic-only variants)")
    md.append("")
    md += md_table(rows_c, ["case"] + ROW[1:], ["case"] + HDR[1:], FMT)
    md.append("")

    # ====================================================== [D] TIMESTEP: NUMERICS?
    head("D", "TIMESTEP -- integration instability, or not?")
    print("  The same case at h = 0.5 ms and h = 0.25 ms.  The timestep is set on the")
    print("  in-memory mjModel for one run; models/osl_v2_bench.xml is never written.")
    rows_d = []
    traces_d = []       # kept ONLY so section H can audit these runs too; see note there
    for h in (dt0, dt0 / 2.0):
        res_h = resample_reference(ref, h, args.cycles, args.interp, verbose=False)
        n_h = res_h.n if args.steps <= 0 else min(int(args.steps * (dt0 / h)), res_h.n)
        tr = run_one(res_h, n_h, KP, KD, "J", dt_override=h, label=f"D_h{h:g}")
        traces_d.append(tr)
        r = tr.row()
        r["case"] = f"h = {h * 1e3:.3f} ms"
        rows_d.append(r)
        print(f"  h={h * 1e3:6.3f} ms  {tr.verdict:8s}  t_unstable="
              f"{('--' if tr.t_unstable is None else f'{tr.t_unstable:.4f} s'):>10s}  "
              f"steps={tr.n:6d}/{tr.n_planned}  max|th_s|={tr.peak('theta_s'):9.5f} rad")
    # and the boundary at the halved step, to see whether the THRESHOLD moved.  The
    # SAME Kd list is used at both step sizes -- a boundary measured on two different
    # grids would not be comparable, which is the only thing this test is for.
    print("\n  the SAME Kd sweep at h = 0.25 ms (the question is whether the boundary "
          "MOVES):")
    res_h2 = resample_reference(ref, dt0 / 2.0, args.cycles, args.interp, verbose=False)
    n_h2 = res_h2.n if args.steps <= 0 else min(args.steps * 2, res_h2.n)
    bnd_half = []
    for kd in KD_SWEEP:
        tr = run_one(res_h2, n_h2, KP, kd, "J", dt_override=dt0 / 2.0,
                     label=f"D_bnd_kd{kd:g}")
        traces_d.append(tr)
        bnd_half.append((kd, tr.verdict))
        r = tr.row()
        r["case"] = f"h = 0.250 ms, Kd = {kd:g}"
        rows_d.append(r)
        print(f"      Kd={kd:8.3f}  {tr.verdict:8s}  t_unstable="
              f"{('--' if tr.t_unstable is None else f'{tr.t_unstable:.4f} s'):>10s}  "
              f"max|th_s|={tr.peak('theta_s'):9.5f} rad")
    write_table(os.path.join(args.outdir, "diag_D_timestep.csv"), rows_d,
                "TEST D -- timestep comparison (runtime opt.timestep only, XML untouched)")
    s_half = [kd for kd, v in bnd_half if v == "STABLE"]
    u_half = [kd for kd, v in bnd_half if v != "STABLE"]
    bnd_h = (f"({max(s_half):g}, {min(u_half):g})" if s_half and u_half
             else "not bracketed")
    print(f"\n  boundary at h=0.5 ms : {bnd}")
    print(f"  boundary at h=0.25 ms: {bnd_h}")
    md.append("## D. Timestep")
    md.append("")
    md += md_table(rows_d, ["case"] + ROW[1:], ["case"] + HDR[1:], FMT)
    md.append("")
    md.append(f"Kd boundary at h = 0.5 ms: **{bnd}**. At h = 0.25 ms: **{bnd_h}**. "
              f"A boundary that does not move with h is evidence against a simple "
              f"integration instability; one that vanishes at h/2 would be evidence for "
              f"numerical coupling and would have to be chased before any conclusion "
              f"about controller architecture.")
    md.append("")

    # ========================================================= [E] ENERGY AND POWER
    head("E", "ENERGY / POWER -- is energy being injected, and by what?")
    md.append("## E. Energy and power on the diverging case")
    md.append("")
    rows_e = []

    def energy_row(tr: Trace, w: slice, name: str) -> dict:
        """Work integrals and MEAN POWER over one window.

        Mean power is reported next to total work because the windows are not the same
        length -- an unstable run is cut short -- and comparing two totals over two
        different durations would be a unit error dressed up as a result.
        """
        a, h = tr.a, tr.dt
        span = max(1, len(a["t"][w])) * h
        W_kd = float(np.sum(a["p_kd_actual"][w]) * h)
        W_col = float(np.sum(a["p_kd_colloc"][w]) * h)
        jd = detrend(a["theta_j_dot"][w], max(3, int(round(1.0 / (15.0 * h)))))
        ad = detrend(a["theta_a_dot"][w] / P.n_t, max(3, int(round(1.0 / (15.0 * h)))))
        anti = float(np.mean((jd * ad) < 0.0)) if jd.size else float("nan")
        uu = a["U"][w]
        return dict(case=name, t0_s=float(a["t"][w][0]), t1_s=float(a["t"][w][-1]),
                    span_s=span,
                    W_kd_on_actuator_J=W_kd, W_kd_collocated_J=W_col,
                    P_kd_on_actuator_W=W_kd / span, P_kd_collocated_W=W_col / span,
                    W_motor_J=float(np.sum(a["p_motor"][w]) * h),
                    W_friction_J=float(np.sum(a["p_fric"][w]) * h),
                    W_Ba_J=float(np.sum(a["p_Ba"][w]) * h),
                    W_belt_to_joint_J=float(np.sum(a["p_belt_joint"][w]) * h),
                    dU_belt_J=float(uu[-1] - uu[0]) if uu.size > 1 else float("nan"),
                    antiphase_fraction=anti, U_peak_J=float(np.max(uu)) if uu.size else
                    float("nan"))

    def show_energy(r: dict) -> None:
        print(f"  {r['case']}   window [{r['t0_s']:.4f}, {r['t1_s']:.4f}] s "
              f"({r['span_s']:.4f} s)")
        print(f"      W by the Kd term ON THE ACTUATOR (actual, non-collocated) = "
              f"{r['W_kd_on_actuator_J']:+.6g} J  ({r['P_kd_on_actuator_W']:+.4g} W mean)")
        print(f"      W by the Kd term IF collocated (-Kd*qdot^2, <=0 by algebra)  = "
              f"{r['W_kd_collocated_J']:+.6g} J  ({r['P_kd_collocated_W']:+.4g} W mean)")
        print(f"      W motor {r['W_motor_J']:+.6g} J | W friction "
              f"{r['W_friction_J']:+.6g} J | W B_a {r['W_Ba_J']:+.6g} J | "
              f"W belt->joint {r['W_belt_to_joint_J']:+.6g} J")
        print(f"      change in stored belt energy over the window = "
              f"{r['dU_belt_J']:+.6g} J  (peak U {r['U_peak_J']:.6g} J)")
        print(f"      fraction of the window with theta_j_dot and theta_a_dot/n_t in "
              f"ANTIPHASE = {r['antiphase_fraction']:.3f}")

    wJ = analysis_window(traces_c["J"])
    rows_e.append(energy_row(traces_c["J"], wJ, "variant J (own window)"))
    rows_e.append(energy_row(traces_c["A"], analysis_window(traces_c["A"]),
                             "variant A (own window)"))
    # variant A restricted to EXACTLY variant J's time window, so the two can be
    # compared without the window length doing any of the work
    tJ = traces_c["J"].a["t"]
    tA = traces_c["A"].a["t"]
    i0 = int(np.searchsorted(tA, tJ[wJ][0]))
    i1 = int(min(len(tA), np.searchsorted(tA, tJ[wJ][-1]) + 1))
    if i1 - i0 > 4:
        rows_e.append(energy_row(traces_c["A"], slice(i0, i1),
                                 "variant A (matched to J's window)"))
    for r in rows_e:
        show_energy(r)
    write_table(os.path.join(args.outdir, "diag_E_energy.csv"), rows_e,
                "TEST E -- work integrals over the analysis window")
    md += md_table(rows_e, ["case", "t0_s", "t1_s", "span_s", "W_kd_on_actuator_J",
                            "W_kd_collocated_J", "P_kd_on_actuator_W",
                            "P_kd_collocated_W", "W_motor_J", "W_friction_J",
                            "W_Ba_J", "W_belt_to_joint_J", "dU_belt_J",
                            "antiphase_fraction", "U_peak_J"],
                  ["case", "t0 s", "t1 s", "span s", "W_Kd on actuator J",
                   "W_Kd if collocated J", "P_Kd on actuator W",
                   "P_Kd if collocated W", "W motor J", "W friction J", "W B_a J",
                   "W belt->joint J", "dU belt J", "antiphase frac", "U peak J"],
                  {"t0_s": "{:.4f}", "t1_s": "{:.4f}", "span_s": "{:.4f}",
                   "W_kd_on_actuator_J": "{:+.6g}",
                   "W_kd_collocated_J": "{:+.6g}", "P_kd_on_actuator_W": "{:+.4g}",
                   "P_kd_collocated_W": "{:+.4g}", "W_motor_J": "{:+.6g}",
                   "W_friction_J": "{:+.6g}", "W_Ba_J": "{:+.6g}",
                   "W_belt_to_joint_J": "{:+.6g}", "dU_belt_J": "{:+.6g}",
                   "antiphase_fraction": "{:.3f}", "U_peak_J": "{:.6g}"})
    md.append("")
    md.append("`W_Kd on actuator` is the work the derivative term does on the actuator "
              "shaft, `integral of (-Kd*qdot_used/n_t)*theta_a_dot dt`, written "
              "joint-referred. `W_Kd if collocated` is the same integral with the "
              "actuator velocity replaced by the joint velocity, i.e. "
              "`integral of -Kd*theta_j_dot^2 dt`, which is non-positive for every "
              "possible signal. A positive actual value with a negative collocated "
              "counterfactual is energy injection caused by the collocation, not by the "
              "gain.")
    md.append("")

    # ======================================================== [F] BELT-MODE FREQUENCY
    head("F", "BELT-MODE FREQUENCY -- measured against the 2-mass prediction")
    rows_f = []
    for variant in ("J", "A"):
        tr = traces_c[variant]
        w = analysis_window(tr)
        a, h = tr.a, tr.dt
        th_s = a["theta_s"][w]
        if th_s.size < 16:
            print(f"  variant {variant}: only {th_s.size} samples in the window -- "
                  f"no frequency reported")
            continue
        # one pass to get a width, then re-detrend with one belt period
        xh0 = detrend(th_s, max(3, int(round(1.0 / (15.0 * h)))))
        f0 = zero_cross_freq(xh0, h)
        wid = max(3, int(round(1.0 / (f0 * h)))) if np.isfinite(f0) and f0 > 0 else 130
        xh = detrend(th_s, wid)
        f_meas = zero_cross_freq(xh, h)
        ts, pk = half_cycle_peaks(xh, h)
        sigma = growth_rate(ts, pk)
        ratio = float(pk[-1] / pk[-2]) if pk.size >= 2 and pk[-2] > 0 else float("nan")
        # Is there a mode to measure at all?  On a run that is not ringing, the
        # detrended signal is tracking residual and quantisation, and a "frequency"
        # read off it means nothing.  The gate is stated rather than left implicit:
        # OSC_GATE is a judgement call, so the raw fraction and the run's own
        # verdict are both reported, letting a reader overrule the gate.
        rms_raw = float(np.sqrt(np.mean((th_s - th_s.mean()) ** 2)))
        rms_osc = float(np.sqrt(np.mean(xh ** 2)))
        osc_frac = rms_osc / rms_raw if rms_raw > 0 else float("nan")
        mode = "yes" if (np.isfinite(osc_frac) and osc_frac > OSC_GATE) else "NO MODE"
        ks_mean = float(np.mean(P.p1 + 2.0 * P.p2 * np.abs(th_s)))
        ks_peak = float(P.p1 + 2.0 * P.p2 * np.max(np.abs(th_s)))
        f_mean = belt_mode_hz(ks_mean, tr.i_j)
        f_peak = belt_mode_hz(ks_peak, tr.i_j)
        pct = (100.0 * (f_meas - f_mean) / f_mean
               if np.isfinite(f_meas) and np.isfinite(f_mean) and f_mean else float("nan"))
        r = dict(case=f"variant {variant}", run_verdict=tr.verdict,
                 mode_present=mode, osc_fraction=osc_frac,
                 f_measured_Hz=f_meas,
                 f_pred_at_mean_Ks_Hz=f_mean, f_pred_at_peak_Ks_Hz=f_peak,
                 pct_diff_vs_mean=pct, K_s_mean_Nm_rad=ks_mean,
                 K_s_peak_Nm_rad=ks_peak, sigma_per_s=sigma,
                 last_peak_ratio=ratio, detrend_width_samples=float(wid),
                 n_half_cycles=float(pk.size))
        rows_f.append(r)
        print(f"  variant {variant} [{tr.verdict}]:  coherent oscillation in theta_s: "
              f"{mode} (oscillatory RMS is {osc_frac:.2f} of total, gate {OSC_GATE})")
        print(f"              measured {f_meas:8.3f} Hz   predicted "
              f"{f_mean:8.3f} Hz at mean K_s={ks_mean:9.2f}  ({pct:+.2f} %)")
        print(f"              predicted {f_peak:8.3f} Hz at peak K_s={ks_peak:9.2f} "
              f"(the mode HARDENS with amplitude, so a band is the honest prediction)")
        print(f"              growth sigma = {sigma:+.3f} /s over {int(pk.size)} half "
              f"cycles; last peak ratio {ratio:.4f}")
        if mode != "yes":
            print(f"              -> this run is not ringing.  The frequency above is "
                  f"read off a tracking residual and is NOT a mode measurement.")
    write_table(os.path.join(args.outdir, "diag_F_belt_mode.csv"), rows_f,
                "TEST F -- measured theta_s frequency vs the 2-mass belt-mode prediction")
    md.append("## F. Belt-mode frequency")
    md.append("")
    md += md_table(rows_f, ["case", "run_verdict", "mode_present", "osc_fraction",
                            "f_measured_Hz",
                            "f_pred_at_mean_Ks_Hz",
                            "f_pred_at_peak_Ks_Hz", "pct_diff_vs_mean",
                            "K_s_mean_Nm_rad", "K_s_peak_Nm_rad", "sigma_per_s",
                            "last_peak_ratio", "n_half_cycles"],
                  ["case", "verdict", "mode present", "osc frac", "f measured Hz",
                   "f pred @mean K_s", "f pred @peak K_s",
                   "% diff vs mean", "K_s mean", "K_s peak", "sigma /s",
                   "last peak ratio", "half cycles"],
                  {"osc_fraction": "{:.2f}", "f_measured_Hz": "{:.3f}",
                   "f_pred_at_mean_Ks_Hz": "{:.3f}",
                   "f_pred_at_peak_Ks_Hz": "{:.3f}", "pct_diff_vs_mean": "{:+.2f}",
                   "K_s_mean_Nm_rad": "{:.2f}", "K_s_peak_Nm_rad": "{:.2f}",
                   "sigma_per_s": "{:+.3f}", "last_peak_ratio": "{:.4f}",
                   "n_half_cycles": "{:.0f}"})
    md.append("")
    md.append(f"Prediction is `f = sqrt(K_s*(1/J_ar + 1/I_j))/2pi` with "
              f"`J_ar = n_t^2*J_a = {J_AR:.9f}` and `I_j = {traces_c['J'].i_j:.6f}` "
              f"kg.m^2 -- DERIVED from the paper's parameters and the bench's effective "
              f"inertia, not fitted to the measurement. `K_s = p1 + 2*p2*|theta_s|` is "
              f"amplitude dependent, so the honest prediction is the band between the "
              f"mean and peak columns.")
    md.append("")

    # ================================================================ [G] theta_s RANGE
    head("G", "DID theta_s STAY INSIDE THE PAPER'S FITTED RANGE?")
    print(f"  Best et al. fitted the belt law out to roughly |theta_s| = "
          f"{FIT_EDGE_RAD} rad")
    print(f"  (~93 N.m).  Past that the law is EXTRAPOLATION of their regression.")
    rows_g = []
    for kd in KD_SWEEP:
        tr = traces_a[kd]
        pk = tr.peak("theta_s")
        r = dict(kd=kd, verdict=tr.verdict, max_theta_s_rad=pk,
                 x_fit_edge=pk / FIT_EDGE_RAD,
                 left_fit_at_s=(-1.0 if tr.t_fit_exit is None else tr.t_fit_exit),
                 inside_fit=("yes" if tr.t_fit_exit is None else "NO"))
        rows_g.append(r)
        print(f"  Kd={kd:8.3f}  max|th_s|={pk:9.5f} rad = {pk / FIT_EDGE_RAD:7.2f}x the "
              f"fit edge   inside fitted range: "
              f"{'yes' if tr.t_fit_exit is None else f'NO, left at t={tr.t_fit_exit:.4f} s'}")
    write_table(os.path.join(args.outdir, "diag_G_theta_s_range.csv"), rows_g,
                "TEST G -- belt deflection against the paper's fitted range")
    md.append("## G. Belt deflection against the paper's fitted range")
    md.append("")
    md += md_table(rows_g, ["kd", "verdict", "max_theta_s_rad", "x_fit_edge",
                            "left_fit_at_s", "inside_fit"],
                  ["Kd", "verdict", "max |th_s| rad", "x fit edge", "left fit at s",
                   "inside fit"],
                  {"kd": "{:.3f}", "max_theta_s_rad": "{:.5f}", "x_fit_edge": "{:.2f}",
                   "left_fit_at_s": "{:.4f}"})
    md.append("")
    md.append(f"`-1` in the *left fit at* column means it never left. The fit edge is "
              f"|theta_s| = {FIT_EDGE_RAD} rad (`oslbench/drivetrain.py` lines 585-586). "
              f"Any torque printed beyond it is extrapolation of the paper's regression, "
              f"so the LATE part of a diverging run carries no quantitative weight -- "
              f"only the approach to divergence does.")
    md.append("")

    # ===================================================================== [H] SERVO
    head("H", "THE SERVO WAS OFF IN EVERY RUN")
    # Every run, and that word has to be literal.  An earlier version summed only tests
    # A/B/C and still claimed "all runs", which quietly left TEST D's twelve halved-step
    # runs unaudited -- and D is the test whose whole job is to rule out a NUMERICAL
    # cause, so it is the last place to accept a weaker guarantee.
    audited = ([traces_a[kd] for kd in KD_SWEEP] + list(traces_b.values())
               + list(traces_c.values()) + traces_d)
    tot = sum(t.servo_violations for t in audited)
    print(f"  runs audited: {len(audited)}  (A {len(KD_SWEEP)} + B {len(traces_b)} + "
          f"C {len(traces_c)} + D {len(traces_d)})")
    print(f"  steps with nonzero knee position-actuator force, all runs: {tot}")
    print("  A nonzero value would mean the servo and the belt both acted in the same")
    print("  mj_step, making every number above a double count rather than a measurement.")
    md.append("## H. Control")
    md.append("")
    md.append(f"Steps with a nonzero knee position-actuator force across all "
              f"**{len(audited)}** runs above: **{tot}**. A nonzero count would make all "
              f"of the above a double count rather than a measurement.")
    md.append("")

    # ===================================================================== RESULTS.md
    path_md = os.path.join(args.outdir, "RESULTS.md")
    with open(path_md, "w") as fh:
        fh.write("\n".join(md) + "\n")

    print()
    print("=" * 78)
    print("WRITTEN")
    print("=" * 78)
    for f in sorted(os.listdir(args.outdir)):
        print(f"  {os.path.join('build', 'drivetrain_instability', f)}")
    print()
    print(f"  Paste build/drivetrain_instability/RESULTS.md into the diagnosis doc.")
    if USING_STUB:
        print()
        print("  REMINDER: engine=STUB.  These are wiring checks, NOT measurements.")
        return 0
    print()
    print("  DIAGNOSIS ONLY.  No production file was modified and nothing was fixed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
