#!/usr/bin/env python3
"""Reproducible time-domain step response for the original MuJoCo position servo."""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
from dataclasses import dataclass

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from oslbench.controller import PDController  # noqa: E402
from oslbench.model import load_bench_model  # noqa: E402
from oslbench.reference import step_reference  # noqa: E402
from oslbench.simulation import BenchSimulation  # noqa: E402

DT = 0.0005
STEP_TIME_S = 0.2
DURATION_S = 5.0
TARGET_DEG = 20.0
SETTLING_FRACTION = 0.02


@dataclass(frozen=True)
class StepCase:
    name: str
    kp: float
    kd: float


CASES = (
    StepCase("kp600_kd17_253", 600.0, 17.253),
    StepCase("kp5_kd3_253", 5.0, 3.253),
)


def make_reference(dt: float = DT, step_time_s: float = STEP_TIME_S,
                   duration_s: float = DURATION_S,
                   target_deg: float = TARGET_DEG) -> tuple[np.ndarray, np.ndarray]:
    """Return every physics-time sample and the matching step reference in radians."""
    ref = step_reference(target_deg, step_time_s, duration_s,
                         samples=max(2, int(round(duration_s / dt)) + 1))
    time_s = np.arange(0.0, duration_s, dt)
    ref_rad = np.where(time_s < step_time_s, 0.0, math.radians(target_deg))
    if len(time_s) != int(round(duration_s / dt)):
        raise RuntimeError("step reference did not produce an exact physics-time grid")
    if not np.isfinite(ref["time_s"]).all() or not np.isfinite(ref["human_knee_angle_deg"]).all():
        raise RuntimeError("generated step reference contains non-finite values")
    return time_s, ref_rad


def _first_crossing(time_s: np.ndarray, angle_deg: np.ndarray,
                    level_deg: float, start: int) -> float | None:
    hits = np.flatnonzero(angle_deg[start:] >= level_deg)
    return None if len(hits) == 0 else float(time_s[start + int(hits[0])])


def calculate_metrics(time_s: np.ndarray, ref_deg: np.ndarray, angle_deg: np.ndarray,
                      torque_request: np.ndarray, torque_delivered: np.ndarray,
                      saturated: np.ndarray, step_time_s: float, target_deg: float,
                      torque_limit: float, dt: float) -> dict[str, float | str]:
    """Calculate step metrics with explicit, conservative availability rules.

    Rise time is the first 10%-to-90% crossing after the command step. Overshoot and
    peak time use the maximum actual angle after the step. Settling is the first sample
    after the last excursion outside target +/-2% of the step amplitude; it is
    unavailable if the trace ends outside that band.
    """
    arrays = (time_s, ref_deg, angle_deg, torque_request, torque_delivered, saturated)
    if any(len(a) != len(time_s) for a in arrays):
        raise ValueError("all metric arrays must have the same length")
    if not all(np.isfinite(np.asarray(a, dtype=float)).all() for a in arrays):
        raise ValueError("metric arrays must be finite")
    start = int(np.searchsorted(time_s, step_time_s, side="left"))
    if start >= len(time_s):
        raise ValueError("step time is outside the trace")
    post_t = time_s[start:]
    post_q = angle_deg[start:]
    peak_i = int(np.argmax(post_q))
    peak_angle = float(post_q[peak_i])
    peak_time = float(post_t[peak_i])
    amplitude = abs(target_deg - float(ref_deg[0]))
    band = SETTLING_FRACTION * amplitude
    outside = np.flatnonzero(np.abs(post_q - target_deg) > band)
    if len(outside) == 0:
        settling_time = float(step_time_s)
    elif int(outside[-1]) + 1 >= len(post_t):
        settling_time = float("nan")
    else:
        settling_time = float(post_t[int(outside[-1]) + 1])
    t10 = _first_crossing(time_s, angle_deg, float(ref_deg[0]) + 0.1 * amplitude, start)
    t90 = _first_crossing(time_s, angle_deg, float(ref_deg[0]) + 0.9 * amplitude, start)
    rise_time = float(t90 - t10) if t10 is not None and t90 is not None else float("nan")
    return {
        "kp_Nm_per_rad": float("nan"),  # filled by the caller
        "kd_Nms_per_rad": float("nan"),  # filled by the caller
        "step_time_s": float(step_time_s),
        "target_deg": float(target_deg),
        "rise_time_10_90_s": rise_time,
        "overshoot_pct": max(0.0, 100.0 * (peak_angle - target_deg) / amplitude),
        "peak_time_s": peak_time,
        "settling_time_s": settling_time,
        "final_angle_deg": float(angle_deg[-1]),
        "steady_state_error_deg": float(target_deg - angle_deg[-1]),
        "peak_torque_request_Nm": float(np.max(np.abs(torque_request))),
        "peak_torque_delivered_Nm": float(np.max(np.abs(torque_delivered))),
        "saturation_duration_s": float(np.count_nonzero(saturated) * dt),
        "saturation_pct": float(100.0 * np.mean(saturated)),
        "torque_limit_Nm": float(torque_limit),
        "n_steps": float(len(time_s)),
        "duration_s": float(time_s[-1]),
    }


def _write_trace(path: str, ref_rad, result, case: StepCase) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(("time_s", "reference_deg", "knee_angle_deg", "knee_velocity_rad_s",
                         "torque_request_Nm", "torque_delivered_Nm", "saturated"))
        for i in range(len(result.time_s)):
            writer.writerow((f"{result.time_s[i]:.6f}", f"{math.degrees(ref_rad[i]):.8f}",
                             f"{math.degrees(result.q[i]):.8f}", f"{result.qdot[i]:.8f}",
                             f"{result.tau_unclamped[i]:.8f}", f"{result.tau[i]:.8f}",
                             int(result.saturated[i])))


def _write_metrics(path: str, rows: list[dict[str, float | str]]) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    keys = list(rows[0])
    with open(path, "w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=keys)
        writer.writeheader()
        for row in rows:
            writer.writerow({
                key: "unavailable" if isinstance(value, float) and math.isnan(value) else value
                for key, value in row.items()
            })


def plot_results(trace_paths: dict[str, str], metrics: dict[str, dict], output_path: str,
                 step_time_s: float, target_deg: float, torque_limit: float) -> None:
    """Write a time-axis PNG; this intentionally has no gait-phase conventions."""
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(3, 1, figsize=(11, 10), sharex=True)
    colours = ("tab:blue", "tab:orange")
    for colour, (name, path) in zip(colours, trace_paths.items()):
        data = np.genfromtxt(path, delimiter=",", names=True)
        t = data["time_s"]
        ref = data["reference_deg"]
        q = data["knee_angle_deg"]
        axes[0].plot(t, ref, "k--", linewidth=1.3, label="reference" if name == CASES[0].name else None)
        axes[0].plot(t, q, color=colour, label=name)
        axes[1].plot(t, q - ref, color=colour, label=name)
        axes[2].plot(t, data["torque_request_Nm"], color=colour, linestyle="--",
                     label=f"{name} request")
        axes[2].plot(t, data["torque_delivered_Nm"], color=colour, label=f"{name} delivered")
    axes[0].axvline(step_time_s, color="0.25", linewidth=1)
    axes[0].axhline(target_deg, color="0.4", linestyle=":", linewidth=1)
    band = SETTLING_FRACTION * abs(target_deg)
    axes[0].axhspan(target_deg - band, target_deg + band, color="0.5", alpha=0.12,
                    label="±2% settling band")
    axes[2].axhline(torque_limit, color="k", linestyle=":", label="forcerange")
    axes[2].axhline(-torque_limit, color="k", linestyle=":")
    axes[0].set_ylabel("angle [deg]")
    axes[1].set_ylabel("error [deg]\nactual - reference")
    axes[2].set_ylabel("torque [N.m]")
    axes[2].set_xlabel("time [s]")
    for axis in axes:
        axis.grid(True, alpha=0.3)
        axis.legend(loc="best")
    fig.suptitle("Original MuJoCo position-servo step response")
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def run_case(bench, time_s: np.ndarray, ref_rad: np.ndarray, case: StepCase):
    sim = BenchSimulation(bench, PDController.knee(bench, case.kp, case.kd))
    nonfinite = None

    def check(state):
        nonlocal nonfinite
        values = (state.time_s, state.q_ref, state.q, state.qdot,
                  state.tau_unclamped, state.tau)
        if not all(math.isfinite(float(value)) for value in values):
            nonfinite = state.k

    result = sim.run(ref_rad, len(ref_rad), on_step=check)
    if nonfinite is not None:
        raise RuntimeError(f"{case.name} produced a non-finite state at step {nonfinite}")
    if not all(np.isfinite(a).all() for a in
               (result.time_s, result.q, result.qdot, result.tau_unclamped, result.tau)):
        raise RuntimeError(f"{case.name} produced non-finite logged arrays")
    ref_deg = np.degrees(ref_rad)
    angle_deg = np.degrees(result.q)
    metrics = calculate_metrics(
        result.time_s, ref_deg, angle_deg, result.tau_unclamped, result.tau,
        result.saturated, STEP_TIME_S, TARGET_DEG, case.kp and bench.force_limit, DT)
    metrics["kp_Nm_per_rad"] = case.kp
    metrics["kd_Nms_per_rad"] = case.kd
    metrics["initial_torque_request_Nm"] = case.kp * math.radians(TARGET_DEG)
    return result, metrics


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--outdir", default=os.path.join(ROOT, "build", "step_response"))
    parser.add_argument("--no-plot", action="store_true")
    args = parser.parse_args(argv)
    os.makedirs(args.outdir, exist_ok=True)
    time_s, ref_rad = make_reference()
    bench = load_bench_model()
    if bench.failures:
        print("REFUSING TO RUN: " + "; ".join(bench.failures), file=sys.stderr)
        return 1
    trace_paths, metrics_rows, metrics_by_case = {}, [], {}
    for case in CASES:
        result, metrics = run_case(bench, time_s, ref_rad, case)
        trace_path = os.path.join(args.outdir, f"{case.name}.csv")
        _write_trace(trace_path, ref_rad, result, case)
        trace_paths[case.name] = trace_path
        metrics_rows.append({"case": case.name, **metrics})
        metrics_by_case[case.name] = metrics
        print(f"{case.name}: final={metrics['final_angle_deg']:.6f} deg, "
              f"error={metrics['steady_state_error_deg']:.6f} deg, "
              f"peak delivered torque={metrics['peak_torque_delivered_Nm']:.6f} N.m, "
              f"saturation={metrics['saturation_duration_s']:.6f} s")
    _write_metrics(os.path.join(args.outdir, "metrics.csv"), metrics_rows)
    if not args.no_plot:
        plot_results(trace_paths, metrics_by_case,
                     os.path.join(args.outdir, "step_response.png"),
                     STEP_TIME_S, TARGET_DEG, bench.force_limit)
    print(f"outputs: {args.outdir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
