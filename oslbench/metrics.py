"""
oslbench.metrics -- the numbers.  No plotting, no simulation, no file I/O.

Every metric here is a function of arrays that were already recorded.  Nothing in this
file can change the experiment, which is the point: the headline result and the
diagnostics are computed from the same logged signals the CSV contains.

DEFINITIONS (all angle metrics are in DEGREES, against the UNCLAMPED reference)
    rms_err_deg        sqrt(mean((q - q_ref)^2))
    peak_err_deg       max |q - q_ref|
    mae_err_deg        mean |q - q_ref|
    peak_tau_Nm        max |bench actuator torque|  -- NOT a human knee moment
    pct_authority      100 * peak_tau / 142.2, the fraction of the knee's authority used
    sat_pct            % of steps where the requested torque exceeded forcerange
    peak_vel_rad_s     max |measured knee velocity|
    peak_power_W       max |tau * qdot|
    ankle_dev_deg      worst drift of the ankle boundary condition from its keyframe
    *_after_50ms       the same error metrics with the seeded start transient excluded
    lag_*              the phase-lag diagnostic, below

THE LAG DIAGNOSTIC, AND WHY IT MATTERS
    A PD position servo lags a moving reference.  Shifting the reference back in time by
    s steps and re-measuring the RMS error says how much of the residual is a pure time
    shift.  On the validated run 90.1 % of the RMS error disappears at a 29.5 ms shift,
    which means the error is LAG, not stiffness -- so the fix is to advance the
    reference or add feedforward, NOT to raise Kp.  This function only reads what was
    already logged; it changes nothing.
"""

from __future__ import annotations

import math

import numpy as np


# -------------------------------------------------------------- primitive metrics
def rms(x) -> float:
    return float(np.sqrt(np.mean(np.asarray(x, float) ** 2)))


def peak_abs(x) -> float:
    return float(np.max(np.abs(np.asarray(x, float))))


def mean_abs(x) -> float:
    return float(np.mean(np.abs(np.asarray(x, float))))


def percent_authority(peak_tau: float, force_limit: float) -> float:
    """How much of the knee's +/-142.2 N.m authority the run actually used, in %."""
    return 100.0 * float(peak_tau) / float(force_limit)


def saturation_percent(saturated) -> float:
    """% of steps where the control law asked for more torque than forcerange allows."""
    return 100.0 * float(np.asarray(saturated, bool).mean())


def tracking_metrics(err_deg, tau, qdot, force_limit: float, saturated) -> dict:
    """The seven headline numbers, from logged signals alone."""
    return dict(
        rms_err_deg=rms(err_deg),
        peak_err_deg=peak_abs(err_deg),
        mae_err_deg=mean_abs(err_deg),
        peak_tau_Nm=peak_abs(tau),
        pct_authority=percent_authority(peak_abs(tau), force_limit),
        sat_pct=saturation_percent(saturated),
        peak_vel_rad_s=peak_abs(qdot),
    )


# ------------------------------------------------------------- the lag diagnostic
def lag_diagnostic(q, ref_rad, dt: float, rms_err_deg: float,
                   max_lag_s: float = 0.15) -> dict:
    """Is the residual error PHASE LAG or genuine tracking failure?

    Shift the reference back by s steps, recompute the RMS error, keep the best shift.
    If the whole error collapses, the error is a time shift and is removable for free
    (advance the reference / feedforward), NOT by raising Kp.
    """
    q = np.asarray(q, float)
    ref = np.asarray(ref_rad, float)
    n = len(q)
    best_s, best_rms = 0, float(rms_err_deg)
    for s in range(1, min(int(max_lag_s / dt), n // 4)):
        e = np.degrees(q[s:] - ref[:-s])          # compare sim(t) with ref(t - s*dt)
        rr = float(np.sqrt(np.mean(e ** 2)))
        if rr < best_rms:
            best_s, best_rms = s, rr
    return dict(
        lag_ms=best_s * dt * 1e3,
        lag_pct_gc=100.0 * best_s * dt / (n * dt),
        rms_err_deg_lag_removed=best_rms,
        pct_err_from_lag=(100.0 * (1.0 - best_rms / rms_err_deg)
                          if rms_err_deg > 0 else 0.0),
    )


# --------------------------------------------------------- the benchmark's metrics
def bench_metrics(result, res, dt: float) -> dict:
    """Every metric written to bench_track_ab19_metrics.csv, in file order.

    `result` is a simulation.RunResult, `res` the resampled reference (only its
    reference-velocity column is used, for reporting).
    """
    err = result.error_deg
    pw = result.power_W
    tau = result.tau
    lim = result.force_limit
    n = result.n

    M = dict(
        rms_err_deg=rms(err),
        peak_err_deg=peak_abs(err),
        mae_err_deg=mean_abs(err),
        peak_tau_Nm=peak_abs(tau),
        pct_authority=percent_authority(peak_abs(tau), lim),
        sat_pct=saturation_percent(result.saturated),
        peak_vel_rad_s=peak_abs(result.qdot),
        peak_ref_vel_rad_s=peak_abs(res["ref_vel_rad_s"]),
        peak_power_W=peak_abs(pw),
        mean_abs_power_W=mean_abs(pw),
        ref_clamped_steps=result.clamped_steps,
        ankle_dev_deg=math.degrees(result.ankle_dev_rad),
        force_limit_Nm=lim,
        final_time_s=result.final_time_s,
        n_steps=n,
    )
    # error after the start transient, so one seeded initial condition cannot flatter
    # or spoil the headline number
    i1 = min(int(0.05 / dt), n - 2)
    M["rms_err_deg_after_50ms"] = rms(err[i1:])
    M["peak_err_deg_after_50ms"] = peak_abs(err[i1:])
    M.update(lag_diagnostic(result.q, result.ref_rad, dt, M["rms_err_deg"]))
    return M


# --------------------------------------------------------- the gain sweep's metrics
SWEEP_METRIC_KEYS = ("rms_err", "rms_err_all", "peak_err", "ss_err", "peak_tau",
                     "pct_auth", "sat_pct", "overshoot_pct", "settle_s", "chatter",
                     "ankle_dev_deg", "analysis_start_s")


def sweep_metrics(result, dt: float, analysis_start: float = 0.0) -> dict:
    """The gain sweep's per-case metrics.

    `analysis_start` excludes the part of the trace where the error is dominated by an
    artefact of the reference (a step edge, a settling cycle) rather than by the
    controller.  `chatter` is the RMS torque slew rate: a high-gain servo that is
    fighting the integrator shows up here before it shows up in the error.
    """
    err = result.error_deg
    n = result.n
    i0 = min(int(analysis_start / dt), n - 2)
    peak_tau = peak_abs(result.tau)
    dtau = np.diff(result.tau)
    return dict(
        rms_err=rms(err[i0:]),
        rms_err_all=rms(err),
        peak_err=peak_abs(err[i0:]),
        ss_err=mean_abs(err[-int(0.1 / dt):]),
        peak_tau=peak_tau,
        pct_auth=percent_authority(peak_tau, result.force_limit),
        sat_pct=100.0 * int(np.sum(result.saturated)) / n,
        chatter=float(np.sqrt(np.mean(np.square(dtau))) / dt) if len(dtau) else 0.0,
        ankle_dev_deg=math.degrees(result.ankle_dev_rad),
        analysis_start_s=analysis_start,
        final_time=result.final_time_s,
    )


def step_shape(t, ref, q, dt: float):
    """Overshoot (% of the commanded step) and 2 % settling time (s) for a step input.

    Returns (nan, nan) for a step to zero, where "% overshoot" is undefined.
    """
    i0 = int(0.2 / dt)
    tgt = float(ref[-1])
    if abs(tgt) < 1e-9:
        return float("nan"), float("nan")
    seg, ts = q[i0:], t[i0:] - t[i0]
    os_ = 100.0 * (float(np.max(seg)) - tgt) / tgt
    out = np.where(np.abs(seg - tgt) > 0.02 * abs(tgt))[0]
    st = float(ts[out[-1]]) if len(out) and out[-1] < len(seg) - 1 else float("nan")
    return os_, st


# ------------------------------------------------------------- the live demo's metrics
def cycle_metrics(act_deg, tau, qdot, ref_deg, force_limit: float, k: int) -> dict:
    """The same headline numbers as the quantitative experiment, measured from the frames
    actually shown in the live demo -- so the demo cannot quietly disagree with the CSV.

    Note `sat` here counts steps within 0.1 % of the limit in the DELIVERED torque,
    which is what a viewer can observe; the benchmark's `sat_pct` counts steps where the
    REQUEST exceeded the limit.  Both are 0 on the validated run.
    """
    e = act_deg[:k] - ref_deg[:k]
    return dict(rms=rms(e), peak=peak_abs(e),
                tau=peak_abs(tau[:k]),
                sat=100.0 * float(np.mean(np.abs(tau[:k]) >= 0.999 * force_limit)),
                vel=peak_abs(qdot[:k]),
                auth=percent_authority(peak_abs(tau[:k]), force_limit))
