"""Executable module-boundary and explicit hardware-mode contracts."""
import importlib
import subprocess
import sys
from types import SimpleNamespace
from oslbench.contracts import Sample, PositionCommand


def test_architecture_compatibility_identity():
    for old, new in (("controller", "control.servo"), ("drivetrain", "control.drivetrain"),
                     ("simulation", "sim.servo"), ("telemetry_source", "telemetry.source"),
                     ("telemetry_replay", "telemetry.replay"),
                     ("replay_presentation", "presentation.replay")):
        assert importlib.import_module("oslbench." + old) is importlib.import_module("oslbench." + new)


def test_architecture_data_and_control_import_isolation():
    code = """
import sys
import oslbench.control.current, oslbench.control.gait, oslbench.control.torque
import oslbench.telemetry.http, oslbench.telemetry.replay, oslbench.telemetry.microstrain
assert not any(n == 'mujoco' or n.startswith(('mujoco.', 'oslbench.sim.', 'oslbench.presentation.')) for n in sys.modules)
"""
    subprocess.run([sys.executable, "-c", code], check=True)


def test_architecture_provenance_includes_nested_sources():
    from oslbench.paths import source_files, ROOT
    from pathlib import Path
    names = {p.relative_to(ROOT).as_posix() for p in source_files()}
    assert {"oslbench/control/current.py", "oslbench/sim/current_gait.py",
            "oslbench/presentation/current_gait.py"} <= names


def test_architecture_d1_measured_only_kinematics():
    from oslbench.presentation.mirroring import mirror_measured_knee
    data = SimpleNamespace(qpos=[0.0], time=3, ctrl=[7])
    calls = []
    def apply(sample):
        mirror_measured_knee(sample, None, data, 0, kinematics=lambda m,d: calls.append(d.qpos[0]))
    apply(Sample(.25, "rad", "knee_joint", 2, "acquisition", "measured"))
    assert calls == [.25] and data.time == 3 and data.ctrl == [7]
    for sample in (Sample(.2,"rad","knee_joint",2,"held_command","measured"),
                   Sample(.2,"rad","imu_filter_ned",2,"acquisition","measured"),
                   Sample(.2,"rad","knee_joint",2,"post_step"),
                   Sample(.2,"rad","knee_joint",2,"acquisition","measured",False)):
        try: apply(sample)
        except ValueError: pass
        else: assert False, "Non-measured/invalid position accepted"
    assert calls == [.25]


def test_architecture_d2_requires_explicit_estimator():
    from oslbench.apps.imu_reference import prepare_imu_reference
    from oslbench.telemetry_schema import ReplayRecord
    sample = Sample(.2,"rad","imu_filter_ned",2,"acquisition","measured")
    record = ReplayRecord(0,0,"imu_roll",2,"device",sample,"measured","rad","imu_filter_ned","acquisition")
    expected = PositionCommand("knee", .1, "knee_joint")
    calls = []
    def estimator(records, tick):
        calls.append((records,tick))
        return expected
    assert prepare_imu_reference([record],3,estimate_reference=estimator) is expected
    assert calls == [((record,),3)]
    for estimator in (None, lambda records,tick: sample, lambda records,tick: PositionCommand("knee",.2,"rotor")):
        try: prepare_imu_reference([record],3,estimate_reference=estimator)
        except ValueError: pass
        else: assert False, "Implicit/incorrect reference accepted"
