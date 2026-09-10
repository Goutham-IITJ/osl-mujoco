"""
oslbench -- the OSL V2 bench experiment, as reusable modules.

THE CODE MAP (one line each; docs/CODE_MAP.md is the same map, for the meeting)

    controller.py   THE PD CONTROL LAW.  tau = Kp*(q_ref - q) - Kd*qdot, Kp = 600,
                    Kd = 17.253, plus the ROM clip and the forcerange clamp.  Start here.
    reference.py    the human gait reference: load the AB19 CSV, audit it, resample it
                    onto the 0.5 ms simulation grid.  No MuJoCo.
    model.py        load models/osl_v2_bench.xml, resolve ids, verify the compiled model
                    is the validated bench, measure the plant inertia.  No control law.
    simulation.py   the physics loop: reference -> controller -> ctrl -> mj_step ->
                    measurements.  ONE implementation, shared by every entry point.
    logging.py      the recorded signals: one row per step, then the CSV writers.
    metrics.py      RMS/peak/MAE error, peak torque, % authority, saturation, the lag
                    diagnostic.  Numbers only.
    plotting.py     the offline six-panel figure, drawn FROM the logged CSV.
    viewer.py       the live MuJoCo window and the video recorder.  Visualisation only.
    dashboard.py    the browser dashboard.  No physics at all -- it is handed state.

ENTRY POINTS (thin scripts; each one is a short script you can read in a minute)

    experiments/run_bench_ab19.py     the quantitative benchmark -> CSV + metrics + plots
    experiments/run_gain_sweep.py     the Kp/Kd tuning sweep on synthetic references
    experiments/run_live_demo.py      the live MuJoCo window (+ browser dashboard)
    experiments/plot_bench_results.py offline figures from an existing CSV
    experiments/verify_against_oracle.py  re-run and compare against the frozen result

WHAT IS BEING SIMULATED, IN ONE SENTENCE
    The CAD-derived Open-Source Leg V2 knee, on a FIXED-BASE bench, tracking the
    knee-flexion angle measured from human subject AB19 during level-ground walking.
    It is joint-level trajectory tracking.  It is NOT whole-body walking, and the
    actuator torque reported is BENCH ACTUATOR TORQUE, not a human knee moment.

WHICH PYTHON
    osl-mujoco\\.venv          mujoco + numpy: runs the simulation.  Do not modify it.
    osl-mujoco\\.venv-analysis matplotlib + numpy: draws the offline figures.
    Nothing here needs MyoAssist or its environments.
"""

from __future__ import annotations

__version__ = "1.0.0"

from .controller import KD, KP, PDController, kd_for_damping_ratio
from .metrics import bench_metrics, sweep_metrics, tracking_metrics
from .model import BenchModel, load_bench_model
from .reference import (audit_reference, load_reference, resample_reference,
                        synthetic_references)
from .simulation import BenchSimulation, RunResult, StepState

__all__ = [
    "KP", "KD", "PDController", "kd_for_damping_ratio",
    "BenchModel", "load_bench_model",
    "load_reference", "audit_reference", "resample_reference", "synthetic_references",
    "BenchSimulation", "RunResult", "StepState",
    "bench_metrics", "sweep_metrics", "tracking_metrics",
]
