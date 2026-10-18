import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from experiments.run_step_response import calculate_metrics, make_reference


def test_step_reference_has_exact_physics_grid():
    t, ref = make_reference()
    assert len(t) == 10000
    assert np.allclose(np.diff(t), 0.0005)
    assert ref[0] == 0.0
    assert ref[int(0.2 / 0.0005) - 1] == 0.0
    assert ref[int(0.2 / 0.0005)] == math.radians(20.0)
    assert ref[-1] == math.radians(20.0)


def test_step_metrics_use_explicit_conventions():
    t = np.arange(0.0, 1.0, 0.1)
    ref = np.where(t < 0.2, 0.0, 10.0)
    q = np.array([0.0, 0.0, 1.0, 5.0, 9.0, 10.5, 10.0, 10.0, 10.0, 10.0])
    request = np.array([0.0, 0.0, 50.0, 40.0, 20.0, 5.0, 0.0, 0.0, 0.0, 0.0])
    delivered = np.clip(request, -30.0, 30.0)
    saturated = request > 30.0
    metrics = calculate_metrics(t, ref, q, request, delivered, saturated,
                                0.2, 10.0, 30.0, 0.1)
    assert metrics["rise_time_10_90_s"] == 0.2
    assert metrics["peak_time_s"] == 0.5
    assert metrics["overshoot_pct"] == 5.0
    assert np.isclose(metrics["saturation_duration_s"], 0.2)
    assert metrics["final_angle_deg"] == 10.0
