#!/usr/bin/env python3
"""Run and compare the six existing ankle parameterisations.

This is an orchestration/reporting layer around
``run_ankle_current_step.py``.  It does not duplicate the controller or drivetrain
model and never writes to the ordinary ``build/ankle_current_step`` outputs.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RUNNER = ROOT / "experiments" / "run_ankle_current_step.py"
DEFAULT_ROOT = ROOT / "build" / "ankle_sensitivity"
CONFIGS = (
    ("script", "cad"),
    ("script", "paper"),
    ("paper", "cad"),
    ("paper", "paper"),
    ("manual_rotor", "cad"),
    ("manual_rotor", "paper"),
)
KT_VALUES = {"script": 0.5701, "paper": 0.9972, "manual_rotor": 0.855}
KT_PROVENANCE = {
    "script": "reported hardware-script output-shaft value; provenance unresolved",
    "paper": "derived from paper rotor k_t 0.1108 N.m/A x planetary ratio 9",
    "manual_rotor": "derived from reported ~0.095 N.m/A value read as rotor k_t x 9",
}
BELT_VALUES = {"cad": 50.0 / 11.0, "paper": 4.61}
BELT_PROVENANCE = {
    "cad": "CAD/MJCF pulley tooth counts 50/11",
    "paper": "Best et al. 2025 paper value",
}
P1 = 876.0
P2 = 14913.0
SETTLING_BAND_DEG = 0.1
EXPECTED_STEPS = 8000
EXPECTED_UPDATES = 4000


def read_metrics(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    with path.open(newline="") as handle:
        for row in csv.reader(handle):
            if len(row) >= 2 and row[0] and not row[0].startswith("#"):
                values[row[0]] = row[1]
    return values


def read_telemetry(path: Path) -> tuple[list[str], list[dict[str, float]]]:
    with path.open(newline="") as handle:
        rows = list(csv.reader(handle))
    header = next(row for row in rows if row and row[0] == "time_s")
    data = []
    for row in rows:
        if not row or row[0] == "time_s" or row[0].startswith("#"):
            continue
        if len(row) != len(header):
            continue
        data.append({name: float(value) for name, value in zip(header, row)})
    return header, data


def peak(data: list[dict[str, float]], name: str) -> tuple[float, float, float]:
    item = max(data, key=lambda row: abs(row[name]))
    return abs(item[name]), item["time_s"], item[name]


def residual(
    data: list[dict[str, float]], fn
) -> tuple[float, float]:
    values = [(abs(fn(row)), row["time_s"]) for row in data]
    return max(values)


def run_one(kt: str, belt: str, out_root: Path) -> dict[str, object]:
    stem = f"kt_{kt}__belt_{belt}"
    outdir = out_root / stem
    suffix = 2
    while outdir.exists():
        outdir = out_root / f"{stem}_{suffix}"
        suffix += 1
    outdir.mkdir(parents=True)
    command = [
        sys.executable,
        str(RUNNER),
        "--backend", "mujoco",
        "--kt", kt,
        "--belt", belt,
        "--outdir", str(outdir),
    ]
    completed = subprocess.run(
        command, cwd=ROOT, text=True, capture_output=True, check=False
    )
    (outdir / "run.log").write_text(
        completed.stdout + "\n--- stderr ---\n" + completed.stderr,
        encoding="utf-8",
    )
    result: dict[str, object] = {
        "kt_choice": kt,
        "belt_choice": belt,
        "kt_value_Nm_per_A": KT_VALUES[kt],
        "kt_provenance": KT_PROVENANCE[kt],
        "belt_ratio": BELT_VALUES[belt],
        "belt_provenance": BELT_PROVENANCE[belt],
        "output_dir": str(outdir.relative_to(ROOT)),
        "return_code": completed.returncode,
        "failure": "" if completed.returncode == 0 else (
            f"experiment exited with code {completed.returncode}; see run.log"
        ),
    }
    csv_path = outdir / "ankle_current_step.csv"
    metrics_path = outdir / "ankle_current_step_metrics.csv"
    if completed.returncode != 0 or not csv_path.exists() or not metrics_path.exists():
        result["failure"] = result["failure"] or "missing generated CSV or metrics"
        return result

    metrics = read_metrics(metrics_path)
    _, data = read_telemetry(csv_path)
    result.update({
        "backend": metrics.get("model_backend", ""),
        "model_sha256": metrics.get("model_model_xml_sha256", ""),
        "ngeom": metrics.get("model_ngeom", ""),
        "nmesh": metrics.get("model_nmesh", ""),
        "ankle_armature": metrics.get("model_ankle_armature", ""),
        "ankle_damping": metrics.get("model_ankle_damping", ""),
        "ankle_frictionloss": metrics.get("model_ankle_frictionloss", ""),
        "final_angle_deg": metrics.get("final_angle_deg", ""),
        "steady_state_error_deg": metrics.get("steady_state_error_deg", ""),
        "rise_time_10_90_s": metrics.get("rise_time_10_90_s", ""),
        "overshoot_pct": metrics.get("overshoot_pct", ""),
        "peak_time_s": metrics.get("peak_time_s", ""),
        "settling_time_s": metrics.get("settling_time_s", ""),
        "peak_current_request_A": metrics.get("peak_current_request_A", ""),
        "peak_current_applied_A": metrics.get("peak_current_applied_A", ""),
        "saturation_duration_s": metrics.get("saturation_duration_s", ""),
        "saturation_pct": metrics.get("saturation_pct", ""),
        "peak_actuator_output_torque_Nm": metrics.get(
            "peak_estimated_actuator_output_torque_Nm", ""
        ),
        "peak_belt_actuator_torque_Nm": metrics.get(
            "peak_belt_torque_ACTUATOR_side_Nm", ""
        ),
        "peak_belt_joint_torque_Nm": metrics.get(
            "peak_belt_torque_JOINT_side_Nm", ""
        ),
        "max_abs_belt_deflection_rad": metrics.get("max_abs_theta_s_rad", ""),
        "geometry_sign": metrics.get("mjcf_q_to_encoder_sign", ""),
        "geometry_sign_provenance": metrics.get(
            "mjcf_q_to_encoder_sign_provenance", ""
        ),
        "n_physics_steps": metrics.get("n_physics_steps", ""),
        "n_control_updates": metrics.get("n_control_updates", ""),
        "ankle_servo_nonzero_steps": metrics.get("ankle_servo_nonzero_steps", ""),
        "ankle_servo_peak_abs_force_Nm": metrics.get(
            "ankle_servo_peak_abs_force_Nm", ""
        ),
        "failure": metrics.get("DIVERGED_at_s", ""),
    })
    if not data:
        result["failure"] = "telemetry CSV contains no rows"
        return result

    nt = BELT_VALUES[belt]
    belt_residual, belt_time = residual(
        data,
        lambda row: row["belt_torque_joint_side_Nm"]
        - nt * row["belt_torque_actuator_side_Nm"],
    )
    law_residual, law_time = residual(
        data,
        lambda row: row["belt_torque_joint_side_Nm"]
        + (0.0 if row["theta_s_rad"] == 0.0 else math.copysign(
            P1 * abs(row["theta_s_rad"]) + P2 * abs(row["theta_s_rad"]) ** 2,
            row["theta_s_rad"],
        )),
    )
    result.update({
        "max_belt_ratio_residual_Nm": belt_residual,
        "belt_ratio_residual_time_s": belt_time,
        "max_belt_law_residual_Nm": law_residual,
        "belt_law_residual_time_s": law_time,
        "peak_current_request_time_s": peak(data, "current_request_A")[1],
        "peak_current_applied_time_s": peak(data, "current_applied_A")[1],
        "peak_actuator_output_time_s": peak(
            data, "estimated_actuator_output_torque_Nm"
        )[1],
        "peak_belt_actuator_time_s": peak(
            data, "belt_torque_actuator_side_Nm"
        )[1],
        "peak_belt_joint_time_s": peak(data, "belt_torque_joint_side_Nm")[1],
    })
    warnings = []
    if metrics.get("model_backend") != "real MuJoCo":
        warnings.append("backend is not real MuJoCo")
    if metrics.get("n_physics_steps") != str(EXPECTED_STEPS):
        warnings.append("unexpected physics-step count")
    if metrics.get("n_control_updates") != str(EXPECTED_UPDATES):
        warnings.append("unexpected control-update count")
    if metrics.get("ankle_servo_nonzero_steps") != "0":
        warnings.append("ankle servo applied nonzero force")
    if float(metrics.get("peak_current_applied_A", "inf")) >= 60.0:
        warnings.append("current clipping reached")
    result["warnings"] = "; ".join(warnings)
    return result


def write_summary(path: Path, results: list[dict[str, object]]) -> None:
    fields = [
        "kt_choice", "kt_value_Nm_per_A", "kt_provenance", "belt_choice",
        "belt_ratio", "belt_provenance", "backend", "model_sha256", "ngeom",
        "nmesh", "ankle_armature", "ankle_damping", "ankle_frictionloss",
        "final_angle_deg", "steady_state_error_deg", "rise_time_10_90_s",
        "overshoot_pct", "peak_time_s", "settling_time_s",
        "settling_tolerance_deg", "peak_current_request_A",
        "peak_current_applied_A", "saturation_duration_s", "saturation_pct",
        "peak_actuator_output_torque_Nm", "peak_belt_actuator_torque_Nm",
        "peak_belt_joint_torque_Nm", "max_abs_belt_deflection_rad",
        "max_belt_ratio_residual_Nm", "belt_ratio_residual_time_s",
        "max_belt_law_residual_Nm", "belt_law_residual_time_s",
        "peak_current_request_time_s", "peak_current_applied_time_s",
        "peak_actuator_output_time_s", "peak_belt_actuator_time_s",
        "peak_belt_joint_time_s", "n_physics_steps", "n_control_updates",
        "ankle_servo_nonzero_steps", "ankle_servo_peak_abs_force_Nm",
        "geometry_sign", "geometry_sign_provenance", "output_dir", "failure",
        "warnings",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for result in results:
            row = {field: result.get(field, "") for field in fields}
            row["settling_tolerance_deg"] = SETTLING_BAND_DEG
            writer.writerow(row)


def make_plots(out_root: Path, results: list[dict[str, object]]) -> None:
    import matplotlib.pyplot as plt

    angle_fig, angle_ax = plt.subplots(figsize=(9, 5))
    torque_fig, (current_ax, torque_ax) = plt.subplots(2, 1, sharex=True,
                                                        figsize=(9, 7))
    for result in results:
        if result.get("failure"):
            continue
        data = read_telemetry(
            ROOT / str(result["output_dir"]) / "ankle_current_step.csv"
        )[1]
        label = (f"{result['kt_choice']}/{result['belt_choice']} "
                 f"(k={float(result['kt_value_Nm_per_A']):.4f}, "
                 f"n={float(result['belt_ratio']):.4f})")
        t = [row["time_s"] for row in data]
        angle_ax.plot(t, [row["encoder_meas_deg"] for row in data], label=label)
        current_ax.plot(t, [abs(row["current_applied_A"]) for row in data],
                        label=label)
        torque_ax.plot(t, [abs(row["belt_torque_joint_side_Nm"]) for row in data],
                       label=label)
    angle_ax.axhline(5.0, color="black", ls="--", lw=0.8, label="5 deg reference")
    angle_ax.set(title="OSL V2 ankle sensitivity: encoder response (simulation)",
                 ylabel="encoder angle [deg]", xlabel="time [s]")
    current_ax.set_ylabel("|applied current| [A]")
    torque_ax.set_ylabel("|joint-side belt torque| [N.m]")
    torque_ax.set_xlabel("time [s]")
    for ax in (angle_ax, current_ax, torque_ax):
        ax.grid(alpha=0.25)
        ax.legend(fontsize=7, loc="best")
    angle_fig.tight_layout()
    torque_fig.tight_layout()
    angle_fig.savefig(out_root / "angle_comparison.png", dpi=140)
    torque_fig.savefig(out_root / "current_joint_torque_comparison.png", dpi=140)
    plt.close(angle_fig)
    plt.close(torque_fig)


def write_report(path: Path, results: list[dict[str, object]]) -> None:
    successful = [r for r in results if not r.get("failure")]
    def floats(name: str) -> list[float]:
        return [float(r[name]) for r in successful if r.get(name, "") != ""]
    lines = [
        "# OSL V2 ankle parameter sensitivity",
        "",
        "This report contains six **real-MuJoCo simulation predictions**, not hardware "
        "measurements. Controller gains, clipping, timing, initialisation, compiled "
        "MJCF and encoder sign were held constant.",
        "",
        "## Parameter meanings",
        "",
        "- `script`: 0.5701 N.m/A, reported as an actuator-output constant from the "
        "hardware script; its shaft provenance is unresolved.",
        "- `paper`: 0.9972 N.m/A, derived from the paper rotor constant 0.1108 N.m/A "
        "times the 9:1 planetary ratio.",
        "- `manual_rotor`: 0.8550 N.m/A, derived by reading the reported 0.095 N.m/A "
        "figure as a rotor constant and multiplying by 9; that source shaft is "
        "unconfirmed.",
        "- `cad`: belt ratio 50/11 = 4.5454545 from the MJCF pulley tooth counts.",
        "- `paper`: belt ratio 4.61 from Best et al. 2025.",
        "",
        "The selected output-side torque constant is converted to the rotor-current "
        "input expected by `DrivetrainLayer`; the planetary ratio is then applied "
        "once inside that layer. The belt ratio enters once through "
        "`theta_s = theta_j - theta_a/n_t`. No rigid joint-torque scaling replaces "
        "the compliant belt.",
        "",
        "## Results",
        "",
        "The complete machine-readable comparison is in "
        "`build/ankle_sensitivity/sensitivity_summary.csv`. Settling uses the same "
        "definition as the ankle experiment: the first sample after the last excursion "
        "outside a ±0.1° band around the 5° target.",
        "",
        f"- Successful runs: {len(successful)}/{len(results)}.",
    ]
    if successful:
        lines.extend([
            f"- Final-angle range: {min(floats('final_angle_deg')):.6f}° to "
            f"{max(floats('final_angle_deg')):.6f}°.",
            f"- Rise-time range: {min(floats('rise_time_10_90_s')):.6f} s to "
            f"{max(floats('rise_time_10_90_s')):.6f} s.",
            f"- Peak applied-current range: {min(floats('peak_current_applied_A')):.6f} A "
            f"to {max(floats('peak_current_applied_A')):.6f} A.",
            f"- Peak joint-side belt-torque range: "
            f"{min(floats('peak_belt_joint_torque_Nm')):.6f} to "
            f"{max(floats('peak_belt_joint_torque_Nm')):.6f} N.m.",
            f"- Maximum belt-deflection range: "
            f"{min(floats('max_abs_belt_deflection_rad')):.9f} to "
            f"{max(floats('max_abs_belt_deflection_rad')):.9f} rad.",
        ])
    lines.extend([
        "",
        "## Interpretation",
        "",
        "The torque-constant choice changes the current-to-output-torque conversion "
        "and therefore changes the simulated plant response, transmitted torque and "
        "belt deflection. It does not change the configured current clip or the "
        "controller gains. The belt-ratio choice changes the actuator/joint kinematic "
        "relationship and the compliant belt loading, so it can change both dynamics "
        "and the joint-side torque even when the output torque constant is fixed.",
        "",
        "The six traces should be read as a range of plausible predictions, not as a "
        "ranking or a fit to the informal lab estimate. Tracking, settling and "
        "saturation are robust only to the extent shown by the six rows; no candidate "
        "is preferred without primary hardware evidence. Differences combine torque "
        "scaling with belt-stiffness interpretation through the changed deflection "
        "kinematics.",
        "",
        "The largest ambiguity would be removed by the manufacturer's specification "
        "for the exact actuator variant, explicitly stating rotor torque constant, "
        "planetary ratio and shaft reference. Direct measured current and a named "
        "torque shaft would further test the ideal-current-loop assumption.",
        "",
        "Equation residuals, backend/model identity, update counts, servo checks and "
        "per-run warnings are recorded in `sensitivity_summary.csv`; generated plots "
        "are `angle_comparison.png` and `current_joint_torque_comparison.png`.",
    ])
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-root", type=Path, default=DEFAULT_ROOT)
    args = parser.parse_args()
    out_root = args.out_root if args.out_root.is_absolute() else ROOT / args.out_root
    out_root.mkdir(parents=True, exist_ok=True)
    results = [run_one(kt, belt, out_root) for kt, belt in CONFIGS]
    summary = out_root / "sensitivity_summary.csv"
    write_summary(summary, results)
    make_plots(out_root, results)
    write_report(ROOT / "docs" / "ANKLE_PARAMETER_SENSITIVITY.md", results)
    for result in results:
        status = "OK" if not result.get("failure") else f"FAILED: {result['failure']}"
        print(f"{result['kt_choice']:12s} {result['belt_choice']:5s} {status}")
    print(f"summary: {summary}")
    return 0 if all(not result.get("failure") for result in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
