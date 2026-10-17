#!/usr/bin/env python3
"""
run_ankle_current_loop_sensitivity.py -- how much does the ASSUMED current-loop lag matter?

    SIMULATION ONLY.  THE NONZERO TIME CONSTANTS ARE ILLUSTRATIVE MODEL ASSUMPTIONS.
    The real driver's current-loop bandwidth has never been measured in this project,
    and nothing in this sweep estimates it.

WHAT THIS ANSWERS, AND WHAT IT CANNOT
    The ankle experiment has always assumed the driver's inner current loop is ideal --
    applied current equals the clipped command within one control interval.  That is an
    assumption, not a measurement.  This sweep makes it adjustable and asks:

        IF the loop had a first-order lag of tau_i, how different would the 5 deg step
        response be?

    That is a SENSITIVITY question about our own model.  It does NOT identify the real
    tau_i.  A run at 10 ms matching hardware better than one at 1 ms would be weak
    evidence at best, because every other unresolved parameter (the torque constant is
    still open to 1.75x) could absorb the same discrepancy.  Treat the output as "how
    much does this assumption matter", not "what is the driver's bandwidth".

WHAT IS HELD FIXED
    Everything except tau_i.  Same MJCF and model identity, same 0 -> 5 deg step and
    initial condition, same Kp/Ki/Kd, same encoder and current-sign mappings, same
    torque constant and belt ratio, same 1 kHz controller against 0.5 ms physics, same
    current clip, same duration.  No gain is touched to make trajectories line up.

THE CONFIGURATION, AND WHY
    `--kt manual_rotor --belt cad`: the 0.855 N.m/A actuator-output equivalent under the
    provisional reading that the reported ~0.095 N.m/A figure is a ROTOR constant, with
    the CAD's 50/11 belt ratio.  This is the WORKING HYPOTHESIS of
    docs/PHYSICAL_ANKLE_CONTROL_ALIGNMENT.md section 5, not established hardware truth.
    It is held fixed here so that tau_i is the only variable; the six-configuration
    sweep in `run_ankle_parameter_sensitivity.py` is where that choice is varied.

HOW IT REUSES THE EXISTING WORK
    It shells out to `experiments/run_ankle_current_step.py` once per time constant, the
    same pattern `run_ankle_parameter_sensitivity.py` uses.  No controller, drivetrain
    or metric definition is duplicated here -- a second copy of the control law is
    exactly the thing that makes two experiments stop being comparable.

USAGE (Windows PowerShell)
    .venv\\Scripts\\python.exe experiments\\run_ankle_current_loop_sensitivity.py

OUTPUT
    build/ankle_current_loop_sensitivity/ -- one subdirectory per time constant, plus a
    master comparison CSV, four plots and a report.  Existing ankle-step and
    six-configuration outputs are never touched.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

RUNNER = ROOT / "experiments" / "run_ankle_current_step.py"
DEFAULT_ROOT = ROOT / "build" / "ankle_current_loop_sensitivity"

# The sweep.  Diagnostic values spanning fast-to-slow; NOT claims about the hardware.
TAU_MS = (0.0, 1.0, 2.0, 5.0, 10.0, 20.0)

# Held fixed -- the working hypothesis, labelled as one.
KT_CHOICE = "manual_rotor"
BELT_CHOICE = "cad"
KT_VALUE = 0.855
BELT_VALUE = 50.0 / 11.0
HYPOTHESIS = ("WORKING HYPOTHESIS, not validated hardware truth: k_t_output = 0.855 "
              "N.m/A, i.e. the reported ~0.095 N.m/A read as a ROTOR constant x the 9:1 "
              "planetary, with the CAD's 50/11 belt ratio.")

STEP_DEG = 5.0
SETTLING_BAND_DEG = 0.02 * STEP_DEG          # +/-2 % of the step, same as the knee test
BANNER = "ANKLE CURRENT-LOOP SENSITIVITY -- how much does the ASSUMED lag matter?"
DISCLAIMER = ("SIMULATION ONLY. NONZERO TIME CONSTANTS ARE ILLUSTRATIVE MODEL "
              "ASSUMPTIONS, NOT MEASURED HARDWARE VALUES.")

# Metrics copied straight through from each run's own metrics file, so this driver
# never re-derives a number the experiment already defined.
PASS_THROUGH = (
    "final_angle_deg", "steady_state_error_deg", "rise_time_10_90_s",
    "overshoot_pct", "peak_time_s", "settling_time_s",
    "peak_current_request_A", "peak_current_applied_A",
    "peak_current_actual_modelled_A", "saturation_duration_s", "saturation_pct",
    "peak_estimated_actuator_output_torque_Nm", "peak_belt_torque_ACTUATOR_side_Nm",
    "peak_belt_torque_JOINT_side_Nm", "max_abs_theta_s_rad",
    "theta_s_as_multiple_of_paper_fit_edge", "max_abs_theta_a_dot_rad_s",
    "peak_applied_current_envelope", "ankle_servo_nonzero_steps",
    "n_physics_steps", "n_control_updates",
    "current_loop_model", "model_backend", "model_ngeom", "model_xml_sha256",
    "model_ankle_properties_match_mjcf", "physics_dt_s", "control_hz",
)


def rel(path: Path) -> str:
    """Repo-relative when possible, absolute otherwise.

    `Path.relative_to` RAISES for a path outside ROOT, which made `--outdir /tmp/...`
    crash the sweep rather than run it. A display helper must never be able to fail the
    experiment it is only labelling.
    """
    p = Path(path)
    try:
        return str(p.resolve().relative_to(ROOT)).replace("\\", "/")
    except ValueError:
        return str(p)


def read_metrics(path: Path) -> dict[str, str]:
    out: dict[str, str] = {}
    if not path.exists():
        return out
    with open(path, newline="", encoding="utf-8") as fh:
        for row in csv.reader(fh):
            if not row or row[0].startswith("#") or row[0] == "metric":
                continue
            if len(row) >= 2:
                out[row[0]] = row[1]
    return out


def read_trace(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    if not path.exists():
        return [], []
    rows = [r for r in csv.reader(open(path, newline="", encoding="utf-8"))
            if r and not r[0].startswith("#")]
    if not rows:
        return [], []
    head = rows[0]
    return head, [dict(zip(head, r)) for r in rows[1:]]


def col(data: list[dict[str, str]], name: str) -> list[float]:
    return [float(r[name]) for r in data if name in r and r[name] != ""]


def run_one(tau_ms: float, out_root: Path) -> dict[str, object]:
    """One real-MuJoCo run at one time constant.  Never overwrites an existing dir."""
    stem = f"tau_{tau_ms:g}ms".replace(".", "p")
    outdir = out_root / stem
    n = 2
    while outdir.exists():
        outdir = out_root / f"{stem}_{n}"
        n += 1
    outdir.mkdir(parents=True)

    cmd = [sys.executable, str(RUNNER),
           "--backend", "mujoco",              # real engine REQUIRED; no stub fallback
           "--kt", KT_CHOICE, "--belt", BELT_CHOICE,
           "--current-loop-tau-ms", f"{tau_ms:g}",
           "--outdir", str(outdir)]
    done = subprocess.run(cmd, cwd=str(ROOT), text=True, capture_output=True, check=False)
    (outdir / "run.log").write_text(done.stdout + "\n--- stderr ---\n" + done.stderr,
                                    encoding="utf-8")

    res: dict[str, object] = {
        "current_loop_tau_ms": tau_ms,
        "is_baseline": tau_ms == 0.0,
        "kt_choice": KT_CHOICE, "kt_value_Nm_per_A": KT_VALUE,
        "belt_choice": BELT_CHOICE, "belt_ratio": BELT_VALUE,
        "output_dir": rel(outdir),
        "return_code": done.returncode,
        "failure": "" if done.returncode == 0 else
                   f"exited {done.returncode}; see {outdir.name}/run.log",
    }
    m = read_metrics(outdir / "ankle_current_step_metrics.csv")
    for k in PASS_THROUGH:
        res[k] = m.get(k, "")
    # The run REPORTS the tau it used; the driver KNOWS the tau it asked for. Keep both
    # and cross-check, rather than letting the pass-through overwrite the commanded
    # value with a string (which it did, and which crashed the report writer).
    res["reported_tau_ms"] = m.get("current_loop_tau_ms", "")
    try:
        res["tau_commanded_equals_reported"] = (
            abs(float(res["reported_tau_ms"]) - tau_ms) < 1e-12)
    except (TypeError, ValueError):
        res["tau_commanded_equals_reported"] = "" if done.returncode else False

    head, data = read_trace(outdir / "ankle_current_step.csv")
    res["n_rows"] = len(data)
    if data:
        # Two things the per-run metrics do not already answer, both about the LOOP.
        try:
            tgt = col(data, "current_applied_A")
            act = col(data, "current_actual_modelled_A")
            res["max_abs_command_minus_actual_A"] = max(
                abs(a - b) for a, b in zip(tgt, act)) if tgt and act else ""
            res["rows_where_actual_differs_from_command"] = sum(
                1 for a, b in zip(tgt, act) if a != b)
            ang = col(data, "encoder_meas_deg")
            res["stayed_finite"] = all(math.isfinite(v) for v in ang + act)
        except (KeyError, ValueError) as exc:                   # noqa: BLE001
            res["trace_parse_error"] = f"{type(exc).__name__}: {exc}"
    return res


def write_master(path: Path, results: list[dict[str, object]]) -> None:
    keys: list[str] = []
    for r in results:
        for k in r:
            if k not in keys:
                keys.append(k)
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(("# " + BANNER,))
        w.writerow(("# " + DISCLAIMER,))
        w.writerow(("# " + HYPOTHESIS,))
        w.writerow(("# Only current_loop_tau_ms varies. Model, step, gains, encoder and "
                    "current-sign mappings, torque constant, belt ratio, loop rates, "
                    "clip and duration are identical across rows.",))
        w.writerow(keys)
        for r in results:
            w.writerow([r.get(k, "") for k in keys])


def make_plots(out_root: Path, results: list[dict[str, object]]) -> list[Path]:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:                                    # noqa: BLE001
        print(f"  [plot] skipped: matplotlib unavailable ({type(exc).__name__}: {exc})")
        return []

    traces = []
    for r in results:
        if r.get("return_code") != 0:
            continue
        od = Path(str(r["output_dir"]))
        if not od.is_absolute():
            od = ROOT / od
        _, d = read_trace(od / "ankle_current_step.csv")
        if d:
            traces.append((float(r["current_loop_tau_ms"]), d))
    if not traces:
        print("  [plot] skipped: no successful runs to plot")
        return []

    cmap = plt.get_cmap("viridis")
    colours = {t: cmap(i / max(1, len(traces) - 1)) for i, (t, _) in enumerate(traces)}
    note = ("Nonzero time constants are ILLUSTRATIVE MODEL ASSUMPTIONS, not measured "
            "hardware values.\n" + HYPOTHESIS)
    made = []

    panels = (
        ("encoder_angle", "encoder_meas_deg", "ankle angle, encoder frame [deg]",
         "1  Encoder-frame ankle angle vs time", True),
        ("tracking_error", "error_deg", "tracking error [deg]",
         "2  Tracking error vs time", False),
        ("joint_torque", "belt_torque_joint_side_Nm", "belt torque, JOINT side [N.m]",
         "4  Joint-side belt torque vs time", False),
    )
    for stem, column, ylab, title, with_ref in panels:
        fig, ax = plt.subplots(figsize=(11, 5.0))
        if with_ref:
            ax.axhline(STEP_DEG, color="#9a9aa4", ls="--", lw=1.2,
                       label=f"reference {STEP_DEG:g} deg")
        for tau, d in traces:
            ax.plot(col(d, "time_s"), col(d, column), lw=1.3, color=colours[tau],
                    label=(f"tau_i = {tau:g} ms"
                           + ("  (ideal baseline)" if tau == 0 else "")))
        ax.set_xlabel("time since PID engagement [s]")
        ax.set_ylabel(ylab)
        ax.set_title(f"{title}\n{DISCLAIMER}", fontsize=10, fontweight="bold")
        ax.grid(alpha=0.25, lw=0.6)
        ax.legend(fontsize=8, ncol=2)
        fig.text(0.01, 0.01, note, fontsize=7, color="#5b5b63")
        fig.tight_layout(rect=(0, 0.06, 1, 1))
        p = out_root / f"{stem}.png"
        fig.savefig(p, dpi=130)
        plt.close(fig)
        made.append(p)

    # 3: commanded vs modelled actual current -- the one plot that shows the lag itself
    fig, ax = plt.subplots(figsize=(11, 5.0))
    for tau, d in traces:
        ax.plot(col(d, "time_s"), col(d, "current_actual_modelled_A"), lw=1.3,
                color=colours[tau], label=f"actual (modelled), tau_i = {tau:g} ms")
    t0, d0 = traces[0]
    ax.plot(col(d0, "time_s"), col(d0, "current_applied_A"), lw=1.0, ls=":",
            color="#9a2b11", label="commanded (clipped) -- identical in every run")
    ax.set_xlabel("time since PID engagement [s]")
    ax.set_ylabel("current [A]   COMMANDED and MODELLED, neither is measured")
    ax.set_title("3  Commanded current vs modelled actual current\n" + DISCLAIMER,
                 fontsize=10, fontweight="bold")
    ax.grid(alpha=0.25, lw=0.6)
    ax.legend(fontsize=8, ncol=2)
    ax.set_xlim(-0.01, 0.35)                 # the lag lives in the transient
    fig.text(0.01, 0.01, note + "\nx-axis limited to the transient, where the lag acts.",
             fontsize=7, color="#5b5b63")
    fig.tight_layout(rect=(0, 0.07, 1, 1))
    p = out_root / "current_command_vs_actual.png"
    fig.savefig(p, dpi=130)
    plt.close(fig)
    made.append(p)
    return made


def fmt(v, nd=4):
    try:
        f = float(v)
    except (TypeError, ValueError):
        return str(v) if v not in ("", None) else "--"
    return "not reached" if not math.isfinite(f) else f"{f:.{nd}f}"


def write_report(path: Path, results: list[dict[str, object]],
                 taus_used=TAU_MS) -> None:
    base = next((r for r in results if r["is_baseline"]), None)
    lines = [
        f"# {BANNER}",
        "",
        f"**{DISCLAIMER}**",
        "",
        HYPOTHESIS,
        "",
        "Only the current-loop time constant varies. The model, step, initial "
        "condition, gains, encoder and current-sign mappings, torque constant, belt "
        "ratio, controller/physics rates, current clip and duration are identical "
        "across every row.",
        "",
        "## What this sweep can and cannot tell you",
        "",
        "It measures how sensitive **our model's** 5 deg step response is to an "
        "assumption we have never tested. It does **not** estimate the real driver's "
        "bandwidth. A particular tau_i matching hardware better than another would be "
        "weak evidence at best: the torque constant alone is still open to 1.75x, and "
        "could absorb the same discrepancy.",
        "",
        "## Comparison",
        "",
        "| tau_i [ms] | final [deg] | err [deg] | rise 10-90 [s] | overshoot [%] | "
        "settling [s] | peak cmd I [A] | peak actual I [A] | sat [%] | "
        "peak tau_j [N.m] | max th_s [rad] |",
        "|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    for r in results:
        if r.get("return_code") != 0:
            lines.append(f"| {float(r['current_loop_tau_ms']):g} | **FAILED**: "
                         f"{r['failure']} |" + " |" * 9)
            continue
        lines.append(
            f"| {float(r['current_loop_tau_ms']):g}"
            + ("  (baseline)" if r["is_baseline"] else "")
            + f" | {fmt(r.get('final_angle_deg'), 5)}"
            f" | {fmt(r.get('steady_state_error_deg'), 5)}"
            f" | {fmt(r.get('rise_time_10_90_s'), 4)}"
            f" | {fmt(r.get('overshoot_pct'), 3)}"
            f" | {fmt(r.get('settling_time_s'), 4)}"
            f" | {fmt(r.get('peak_current_applied_A'), 4)}"
            f" | {fmt(r.get('peak_current_actual_modelled_A'), 4)}"
            f" | {fmt(r.get('saturation_pct'), 3)}"
            f" | {fmt(r.get('peak_belt_torque_JOINT_side_Nm'), 4)}"
            f" | {fmt(r.get('max_abs_theta_s_rad'), 6)} |")

    lines += ["", "## Sensitivity, relative to the tau_i = 0 baseline", ""]
    if base is not None and base.get("return_code") == 0:
        lines += ["| metric | 0 ms | 20 ms | change |", "|---|---|---|---|"]
        worst = next((r for r in results
                      if float(r["current_loop_tau_ms"]) == max(taus_used)), None)
        if worst is not None and worst.get("return_code") == 0:
            for key, label, nd in (
                    ("final_angle_deg", "final angle [deg]", 5),
                    ("steady_state_error_deg", "steady-state error [deg]", 5),
                    ("rise_time_10_90_s", "rise time 10-90 [s]", 4),
                    ("overshoot_pct", "overshoot [%]", 3),
                    ("settling_time_s", "settling [s]", 4),
                    ("peak_current_actual_modelled_A", "peak actual current [A]", 4),
                    ("peak_belt_torque_JOINT_side_Nm", "peak joint torque [N.m]", 4),
                    ("max_abs_theta_s_rad", "max belt deflection [rad]", 6)):
                a, b = base.get(key, ""), worst.get(key, "")
                try:
                    fa, fb = float(a), float(b)
                    ch = (f"{100 * (fb - fa) / abs(fa):+.2f} %" if fa not in (0.0,)
                          and math.isfinite(fa) and math.isfinite(fb) else
                          f"{fb - fa:+.6g} absolute")
                except (TypeError, ValueError):
                    ch = "--"
                lines.append(f"| {label} | {fmt(a, nd)} | {fmt(b, nd)} | {ch} |")
    lines += [
        "",
        "## Integrity",
        "",
        "| check | why it matters |",
        "|---|---|",
        "| `model_backend` must read `real MuJoCo` on every row | a stub run would be a "
        "different plant, not a rougher one |",
        "| `model_ankle_properties_match_mjcf` must be `True` | proves the compiled "
        "ankle is the authored one |",
        "| `ankle_servo_nonzero_steps` must be 0 | otherwise the servo and the belt "
        "both drive the joint and every number is a double count |",
        "| at tau_i = 0, `rows_where_actual_differs_from_command` must be 0 | the "
        "baseline must be an exact passthrough, not an approximation of one |",
        "",
        "## Remaining physical uncertainty",
        "",
        "- The driver's real current-loop time constant and bandwidth: **unknown**. "
        "Needs measured current telemetry or an authoritative driver specification.",
        "- The torque constant's shaft reference: still open to 1.75x.",
        "- Transmission efficiency, encoder latency and filtering: not modelled.",
        "",
        "Hardware validation still requires actual current and angle telemetry from the "
        "rig, or a manufacturer specification of the driver's response. Nothing here "
        "substitutes for either.",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    ap = argparse.ArgumentParser(description=f"{BANNER}. {DISCLAIMER}")
    ap.add_argument("--outdir", default=str(DEFAULT_ROOT))
    ap.add_argument("--tau-ms", type=float, nargs="*", default=None,
                    help="override the swept time constants in ms (default "
                         f"{' '.join(f'{t:g}' for t in TAU_MS)})")
    ap.add_argument("--no-plot", action="store_true")
    args = ap.parse_args()

    taus = TAU_MS if args.tau_ms is None else tuple(args.tau_ms)
    for t in taus:
        if not math.isfinite(t) or t < 0.0:
            print(f"REFUSING TO RUN: time constant must be finite and >= 0, got {t!r}")
            return 2
    if not any(t == 0.0 for t in taus):
        print("NOTE: the sweep does not include tau_i = 0, so there is no ideal "
              "baseline to compare against.")

    out_root = Path(args.outdir)
    out_root.mkdir(parents=True, exist_ok=True)

    print("=" * 78)
    print(f"  {BANNER}")
    print(f"  {DISCLAIMER}")
    print("=" * 78)
    print(f"  held fixed : --kt {KT_CHOICE} ({KT_VALUE} N.m/A) "
          f"--belt {BELT_CHOICE} ({BELT_VALUE:.6f})")
    print(f"               {HYPOTHESIS}")
    print(f"  varying    : current-loop tau_i = "
          f"{', '.join(f'{t:g}' for t in taus)} ms")
    print(f"  backend    : real MuJoCo REQUIRED for every run; the runner refuses to "
          f"fall back to the stub")
    print(f"  outputs    : {rel(out_root)}")
    print("-" * 78)

    results = []
    for t in taus:
        print(f"  tau_i = {t:g} ms ...", end=" ", flush=True)
        r = run_one(t, out_root)
        results.append(r)
        if r["return_code"] != 0:
            print(f"FAILED ({r['failure']})")
        else:
            print(f"ok   final {fmt(r.get('final_angle_deg'), 4)} deg   "
                  f"peak |I| {fmt(r.get('peak_current_actual_modelled_A'), 3)} A   "
                  f"peak tau_j {fmt(r.get('peak_belt_torque_JOINT_side_Nm'), 3)} N.m")

    master = out_root / "current_loop_sensitivity_summary.csv"
    write_master(master, results)
    report = out_root / "CURRENT_LOOP_SENSITIVITY.md"
    write_report(report, results, taus)
    plots = [] if args.no_plot else make_plots(out_root, results)

    print("-" * 78)
    print("  outputs")
    for p in [master, report] + plots:
        print(f"    {rel(p)}")

    ok = sum(1 for r in results if r["return_code"] == 0)
    print(f"\n  {ok} of {len(results)} runs completed")
    base = next((r for r in results if r["is_baseline"]), None)
    if base is not None and base.get("return_code") == 0:
        n_diff = base.get("rows_where_actual_differs_from_command", "?")
        print(f"  baseline passthrough check: {n_diff} row(s) where the modelled actual "
              f"current differs from the command (must be 0)")
    print(f"\n  {DISCLAIMER}")
    return 0 if ok == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
