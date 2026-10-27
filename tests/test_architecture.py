"""Executable module-boundary and explicit hardware-mode contracts."""
import importlib
import subprocess
import sys
from types import SimpleNamespace
from oslbench.contracts import Sample, PositionCommand


def test_architecture_canonical_modules_and_retired_paths():
    pairs = (
        ("ankle_current_control", "sim.ankle"),
        ("controller", "control.servo"),
        ("current_gait_live", "presentation.current_gait"),
        ("dashboard", "presentation.servo_dashboard"),
        ("drivetrain", "control.drivetrain"),
        ("drivetrain_dashboard", "presentation.drivetrain_dashboard"),
        ("drivetrain_sim", "sim.compliant"),
        ("feedback_location_diagnostic", "control.feedback"),
        ("http_telemetry_source", "telemetry.http"),
        ("imu_service", "telemetry.imu_service"),
        ("imu_telemetry", "telemetry.imu_schema"),
        ("joint_current_control", "control.gait"),
        ("microstrain_reader", "telemetry.microstrain"),
        ("model", "sim.model"),
        ("observation_presentation", "presentation.observations"),
        ("replay_presentation", "presentation.replay"),
        ("simulation", "sim.servo"),
        ("telemetry_presentation", "presentation.telemetry"),
        ("telemetry_replay", "telemetry.replay"),
        ("telemetry_source", "telemetry.source"),
        ("viewer", "presentation.viewer"),
    )
    from pathlib import Path
    package = Path(__file__).resolve().parents[1] / "oslbench"
    for old, new in pairs:
        canonical = importlib.import_module("oslbench." + new)
        assert canonical.__name__ == "oslbench." + new
        assert not (package / (old + ".py")).exists()
        assert importlib.util.find_spec("oslbench." + old) is None
    assert not (package / "_compat.py").exists()
    from typing import get_type_hints
    from oslbench.control.torque import PDCurrentSource
    from oslbench.control.compliance import DrivetrainLayer
    assert get_type_hints(PDCurrentSource.__init__)["layer"] is DrivetrainLayer


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
