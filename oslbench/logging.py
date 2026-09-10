"""
oslbench.logging -- the recorded signals.

One row per simulation step, written to CSV.  This module formats and writes; it does
not know how the physics works, and it never computes a physical quantity that is not
already present in the state handed to it.

NAME COLLISION, DELIBERATELY ACCEPTED
    Python 3 uses absolute imports, so `import logging` anywhere in the repo still gets
    the standard library.  Only `from . import logging` inside this package reaches this
    file.  The name is kept because "logging.py -> recorded signals" is what the code
    map says.

THE 21 BENCHMARK COLUMNS
    time_s               s, MuJoCo's clock after the step
    gait_phase_percent   %, 0-100, analytic
    ref_rad / ref_deg    the reference angle, UNCLAMPED
    sim_rad / sim_deg    the measured knee angle
    err_deg              sim - ref, deg
    ref_vel_rad_s        d(ref)/dt from the spline
    sim_vel_rad_s        measured knee velocity
    ctrl_rad             what was actually commanded (reference after the ROM clip)
    tau_sensor_Nm        BENCH ACTUATOR torque, N.m, after the forcerange clamp
    tau_actforce_Nm      MuJoCo's actuator_force (pre-step state -- see simulation.py)
    tau_unclamped_Nm     Kp*e - Kd*qdot as requested, before the clamp
    pct_authority        100*|tau|/142.2
    saturated            1 if the request exceeded forcerange
    knee_power_W         tau * qdot
    ankle_q_rad          the ankle boundary condition, for drift monitoring
    ankle_tau_Nm         what the ankle servo spends holding it
    human_knee_angle_deg the resampled reference, in the source's own units
    human_knee_moment    carried through VERBATIM; see the reference audit flags
    human_knee_power     carried through VERBATIM; see the reference audit flags
"""

from __future__ import annotations

import csv
import math
import os

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# WHERE OUTPUTS GO.  Everything this module writes lands under build/, which is a
# generated directory: nothing in models/ or build/AB19_knee_gait_reference.csv is ever
# written by the experiment.
DEFAULT_OUTDIR = os.path.join(ROOT, "build", "bench_track_ab19")
SWEEP_OUTDIR = os.path.join(ROOT, "build", "gain_sweep")
BENCH_CSV_NAME = "bench_track_ab19.csv"
METRICS_CSV_NAME = "bench_track_ab19_metrics.csv"


def bench_csv_path(outdir: str | None = None) -> str:
    return os.path.join(outdir or DEFAULT_OUTDIR, BENCH_CSV_NAME)


def metrics_csv_path(outdir: str | None = None) -> str:
    return os.path.join(outdir or DEFAULT_OUTDIR, METRICS_CSV_NAME)


BENCH_CSV_HEADER = ("time_s", "gait_phase_percent", "ref_rad", "ref_deg", "sim_rad",
                    "sim_deg", "err_deg", "ref_vel_rad_s", "sim_vel_rad_s", "ctrl_rad",
                    "tau_sensor_Nm", "tau_actforce_Nm", "tau_unclamped_Nm",
                    "pct_authority", "saturated", "knee_power_W", "ankle_q_rad",
                    "ankle_tau_Nm", "human_knee_angle_deg", "human_knee_moment",
                    "human_knee_power")

SWEEP_CSV_HEADER = ("time_s", "kp", "kv", "ref_rad", "ref_deg", "sim_rad", "sim_deg",
                    "err_deg", "ref_vel_rad_s", "sim_vel_rad_s", "ctrl_rad",
                    "tau_sensor_Nm", "tau_actforce_Nm", "tau_unclamped_Nm",
                    "pct_authority", "saturated", "ankle_q_rad", "ankle_tau_Nm")


class BenchLog:
    """The per-step table for the AB19 benchmark: 21 columns, pre-formatted as text.

    Formatting happens here, at append time, so the CSV on disk and any check made
    against it see exactly the same digits.
    """

    header = BENCH_CSV_HEADER

    def __init__(self, force_limit: float):
        self.force_limit = float(force_limit)
        self._rows: list[tuple] = []

    def append(self, state, res, k: int) -> None:
        """Record one step.  `state` is a simulation.StepState, `res` the resampled
        reference (for phase, reference velocity and the carried-through human columns)."""
        lim = self.force_limit
        self._rows.append((
            f"{state.time_s:.6f}", f"{res['gait_phase_percent'][k]:.6f}",
            f"{state.q_ref:.8f}", f"{math.degrees(state.q_ref):.6f}",
            f"{state.q:.8f}", f"{math.degrees(state.q):.6f}",
            f"{math.degrees(state.q - state.q_ref):.6f}",
            f"{res['ref_vel_rad_s'][k]:.6f}", f"{state.qdot:.6f}",
            f"{state.command:.8f}",
            f"{state.tau:.6f}", f"{state.tau_actuator:.6f}",
            f"{state.tau_unclamped:.6f}",
            f"{100.0 * abs(state.tau) / lim:.4f}", int(state.saturated),
            f"{state.tau * state.qdot:.6f}",
            f"{state.ankle_q:.8f}", f"{state.ankle_tau:.6f}",
            f"{res['human_knee_angle_deg'][k]:.6f}",
            f"{res['human_knee_moment'][k]:.6f}",
            f"{res['human_knee_power'][k]:.6f}",
        ))

    def rows(self) -> list[tuple]:
        return self._rows

    def __len__(self) -> int:
        return len(self._rows)


class SweepLog:
    """The gain sweep's per-step table: 18 columns, decimated (default every 2nd step
    -> 1 kHz), with the gains carried on every row so the trace is self-describing."""

    header = SWEEP_CSV_HEADER

    def __init__(self, force_limit: float, kp: float, kv: float, decim: int = 2):
        self.force_limit, self.kp, self.kv = float(force_limit), float(kp), float(kv)
        self.decim = int(decim)
        self._rows: list[tuple] = []

    def append(self, state, ref_vel, k: int) -> None:
        if k % self.decim:
            return
        lim = self.force_limit
        self._rows.append((
            f"{state.time_s:.6f}", f"{self.kp:.1f}", f"{self.kv:.4f}",
            f"{state.q_ref:.8f}", f"{math.degrees(state.q_ref):.6f}",
            f"{state.q:.8f}", f"{math.degrees(state.q):.6f}",
            f"{math.degrees(state.q - state.q_ref):.6f}",
            f"{ref_vel[k]:.6f}", f"{state.qdot:.6f}",
            f"{state.command:.8f}", f"{state.tau:.6f}",
            f"{state.tau_actuator:.6f}", f"{state.tau_unclamped:.6f}",
            f"{100.0 * abs(state.tau) / lim:.4f}", int(state.saturated),
            f"{state.ankle_q:.8f}", f"{state.ankle_tau:.6f}",
        ))

    def rows(self) -> list[tuple]:
        return self._rows

    def __len__(self) -> int:
        return len(self._rows)


# ------------------------------------------------------------------------- writers
def write_bench_csv(path: str, log: BenchLog, provenance: list[str]) -> str:
    """Write the 21-column benchmark trace with its provenance comments.

    The `#` lines are written directly rather than through csv.writer: they contain
    commas, csv.writer would quote them, and the quote would break the `#`-prefix test
    used when the file is read back for plotting.
    """
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", newline="") as fh:
        for line in provenance:
            fh.write(f"# {line}\n")
        w = csv.writer(fh)
        w.writerow(list(BENCH_CSV_HEADER))
        w.writerows(log.rows())
    return path


def write_metrics_csv(path: str, metrics: dict, audit_fields: dict | None = None,
                      flags: list[str] | None = None,
                      audit_prefix: str = "ref_") -> str:
    """Write the metric/value table, then the audit fields, then the audit flags.

    The flags are stored as `audit_flag_1..N` in the order they fired, so the reference
    problems travel with the result instead of scrolling off a console.
    """
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["metric", "value"])
        for k, v in metrics.items():
            w.writerow([k, v])
        for k, v in (audit_fields or {}).items():
            w.writerow([f"{audit_prefix}{k}", v])
        for i, f in enumerate(flags or []):
            w.writerow([f"audit_flag_{i + 1}", f])
    return path


def write_sweep_csv(path: str, named_rows, provenance: list[str]) -> str:
    """Write the sweep trace: comment rows, a `reference`-prefixed header, then every
    (reference name, row) pair in run order."""
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        for line in provenance:
            w.writerow([f"# {line}"])
        w.writerow(["reference"] + list(SWEEP_CSV_HEADER))
        for name, rows in named_rows:
            for row in rows:
                w.writerow([name] + list(row))
    return path


def write_sweep_metrics_csv(path: str, results: dict, keys, zeta: float,
                            kv_of) -> str:
    """Write one row per (reference, kp) pair.  `kv_of(kp)` supplies the matching kv so
    the file records the gain PAIR, not just the swept kp."""
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["reference", "kp", "kv", "zeta"] + list(keys))
        for (name, kp), m in results.items():
            w.writerow([name, kp, f"{kv_of(kp):.4f}", zeta] +
                       [f"{m[k]:.6f}" for k in keys])
    return path


# -------------------------------------------------------------------------- reader
def read_bench_csv(path: str) -> dict:
    """Read a benchmark trace back into arrays, skipping the provenance comments.

    The header row is FOUND by matching its first field against BENCH_CSV_HEADER[0]
    rather than assumed to be line 1, because the number of provenance lines is not
    part of the format.  Used by plotting.py and by the oracle comparison.
    """
    with open(path, newline="") as fh:
        rows = list(csv.reader(fh))
    hdr_i = next((i for i, r in enumerate(rows)
                  if r and r[0].strip() == BENCH_CSV_HEADER[0]), None)
    if hdr_i is None:
        raise ValueError(f"{path}: no header row starting with "
                         f"{BENCH_CSV_HEADER[0]!r} -- is this a bench trace?")
    header = [c.strip() for c in rows[hdr_i]]
    body = [r for r in rows[hdr_i + 1:] if r and not r[0].startswith("#")]
    cols = {}
    for j, name in enumerate(header):
        vals = [r[j] for r in body]
        cols[name] = (np.array([int(v) for v in vals]) if name == "saturated"
                      else np.array([float(v) for v in vals]))
    cols["_header"] = header
    cols["_comments"] = [",".join(r) for r in rows[:hdr_i]]
    return cols


def read_metrics_csv(path: str) -> dict:
    """Read a metrics CSV back as {name: str}.  Values stay STRINGS because the file also
    holds the audit flags, which are sentences; the caller floats what it needs."""
    with open(path, newline="") as fh:
        rows = [r for r in csv.reader(fh)
                if r and not r[0].lstrip().startswith("#")]
    return {r[0].strip(): r[1].strip() for r in rows if len(r) >= 2}
