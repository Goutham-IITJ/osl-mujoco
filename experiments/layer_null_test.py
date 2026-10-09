#!/usr/bin/env python3
"""
layer_null_test.py -- NULL TEST for the Option-C drivetrain layer.

    "Instantiate the drivetrain layer but disabled, run the existing AB19 benchmark,
     require exact reproduction of the frozen oracle metrics."        -- the directive

    "The new actuator layer must be switchable OFF so that:
         layer OFF -> exact oracle reproduction"                      -- the directive

WHAT A NULL TEST IS FOR
    Adding an actuator layer to a validated benchmark creates exactly one way to
    destroy the benchmark silently: the new code path perturbs the old one even when
    it is supposed to be inert.  A stale `qfrc_applied`, an extra `mj_forward`, a
    re-seeded keyframe, a gain written twice -- any of these would shift the result by
    a little, and "a little" is indistinguishable from physics unless something checks.
    This file checks.

THE TEST IS IN TWO PARTS, AND THE FIRST IS THE STRONGER ONE
    [1] ADDITIVITY, in-process.  Run the SAME resampled reference through the plain
        `BenchSimulation` and through `DrivetrainBenchSimulation` with the layer
        disabled, in one interpreter, and require the two runs to be BIT-IDENTICAL:
        every element of every per-step array, every scalar, every metric, and the two
        written CSVs byte-for-byte.  This is the decisive test, because it compares the
        new path against the old path AS THEY ARE TODAY.  It cannot be fooled by an
        out-of-date oracle, and it does not care whether the oracle was regenerated.

        Section 1 opens with a DETERMINISM CONTROL: plain-versus-plain, two runs of the
        identical code.  If that control ever failed, the engine would be
        non-reproducible across resets and the additivity comparison below it would be
        meaningless.  The control is here so a PASS in section 1 means something.

    [3]/[4] ORACLE, on disk.  Then compare the disabled-layer run against the FROZEN
        result in tests/oracle/ -- the metrics to 1e-9 relative (and bit-exactly on the
        five headline numbers), and all 21 trace columns to the resolution the CSV is
        written at.  This is the weaker test logically -- it is implied by [1] plus the
        already-passing experiments/verify_against_oracle.py -- but it is the one the
        directive asked for by name, so it is run directly rather than argued for.

NOT A SECOND MONOLITHIC SCRIPT
    `experiments/verify_against_oracle.py` already owns the oracle comparison logic:
    the per-column tolerance table, the trace differ and the metric differ.  None of it
    is copied here.  This file IMPORTS `compare_trace`, `compare_metrics`, `FMT_TOL`,
    `METRIC_RTOL` and `HEADLINE` from that script and re-uses them unchanged, so the
    two files cannot drift apart into two different definitions of "matches".

WHAT RUNS WITHOUT MuJoCo
    Section 1 runs on `tests/stub_mujoco.py` when MuJoCo is unavailable, and that is
    not a consolation prize -- `StubData` has NO `qfrc_applied` attribute at all, so a
    disabled layer that touched it would raise AttributeError and the run would crash
    instead of quietly passing.  On the stub, section 1 completing IS the proof that
    the disabled path never writes.  Sections 2-4 need the real engine and are skipped,
    with a printed record of what is still owed.

Run:  python experiments/layer_null_test.py                    (sections 1, 2-stub)
      .venv\\Scripts\\python.exe experiments\\layer_null_test.py   (all sections)
"""

from __future__ import annotations

import hashlib
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (ROOT, os.path.join(ROOT, "tests"), os.path.join(ROOT, "experiments")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

# The stub must be installed BEFORE oslbench is imported, because oslbench.simulation
# binds `mujoco` at module import time.  install() returns the REAL module when it is
# importable, so inside .venv this line is a no-op and the real engine is used.
import stub_mujoco                                                      # noqa: E402

MJ = stub_mujoco.install()
USING_STUB = stub_mujoco.is_stub(MJ)

from oslbench import logging as blog                                    # noqa: E402
from oslbench.controller import KD, KP, PDController                    # noqa: E402
from oslbench.drivetrain import PAPER                                   # noqa: E402
from oslbench.drivetrain_sim import (DrivetrainBenchSimulation,         # noqa: E402
                                     DrivetrainLayer)
from oslbench.metrics import bench_metrics                              # noqa: E402
from oslbench.model import load_bench_model                             # noqa: E402
from oslbench.reference import (SUBJECT, audit_reference,               # noqa: E402
                                load_reference, resample_reference)
from oslbench.simulation import BenchSimulation                         # noqa: E402

# Re-used, never re-defined.  See "NOT A SECOND MONOLITHIC SCRIPT" above.
import verify_against_oracle as V                                       # noqa: E402

WORK_DIR = os.path.join(ROOT, "build", "layer_null_test")

# Every per-step array and every scalar a RunResult carries.  Listed explicitly rather
# than discovered with vars(), so that a field ADDED to RunResult later shows up as a
# test failure here instead of silently escaping comparison.
RESULT_ARRAYS = ("ref_rad", "time_s", "ctrl", "q", "qdot", "tau", "tau_actuator",
                 "tau_unclamped", "saturated", "ankle_q", "ankle_tau")
RESULT_SCALARS = ("n", "force_limit", "clamped_steps", "ankle_dev_rad", "final_time_s")

FAILURES: list[str] = []


def check(ok: bool, label: str, detail: str = "") -> bool:
    mark = "PASS" if ok else "FAIL"
    print(f"    [{mark}] {label}" + (f"   {detail}" if detail else ""))
    if not ok:
        FAILURES.append(label)
    return bool(ok)


def md5(path: str, body_only: bool = False) -> str:
    """md5 of the file, or of its DATA rows only, with line endings normalised.

    `body_only` drops the leading `# ...` provenance comments.  Those lines legitimately
    differ between two runs -- they record which run wrote the file, and the oracle's
    record a different script entirely -- so including them would make a byte comparison
    fail for a reason that has nothing to do with the physics.  Everything from the
    header row down is compared verbatim, character for character.

    CR/LF is stripped from the end of every line before hashing.  `tests/oracle/*.csv`
    is known to carry CRLF/LF churn in the working tree (a git checkout artefact --
    the two files are content-identical to HEAD after `tr -d '\\r'`), and a digit
    comparison that failed on a line terminator would be reporting the wrong thing.
    """
    h = hashlib.md5()
    with open(path, "rb") as fh:
        for line in fh:
            if body_only and line.startswith(b"#"):
                continue
            h.update(line.rstrip(b"\r\n") + b"\n")
    return h.hexdigest()


# --------------------------------------------------------------------------- setup
def prepare():
    """Compile the bench and resample AB19 ONCE, so every run below sees one input.

    Sharing the reference is deliberate: if each run resampled its own copy, a
    difference in the resampler would show up as a difference in the simulation and be
    misattributed to the layer.
    """
    bench = load_bench_model(verbose=False)
    if bench.failures:
        for f in bench.failures:
            print(f"  model precondition FAILED: {f}")
        FAILURES.append("bench model preconditions")
    dt = bench.timestep
    ref = load_reference(None)
    audit = audit_reference(ref, math.degrees(bench.knee_ctrlrange[0]),
                            math.degrees(bench.knee_ctrlrange[1]), verbose=False)
    res = resample_reference(ref, dt, 1, "cubic", audit=audit, verbose=False)
    return bench, res, audit, dt


def run_once(bench, res, factory, tag: str, watch_applied: bool = False):
    """Mirror verify_against_oracle.rerun()'s run, with the simulation class swapped.

    `factory(bench, controller)` builds the simulation, so the ONLY difference between
    the runs compared below is the class being constructed.  Gains come from
    oslbench.controller and are not touched: KP = %s, KD = %s.
    """
    log = blog.BenchLog(bench.force_limit)
    sim = factory(bench, PDController.knee(bench, KP, KD))
    seen = {"max_abs": 0.0, "checked": 0, "steps": 0, "absent": False}

    def on_step(st):
        log.append(st, res, st.k)
        seen["steps"] += 1
        if watch_applied:
            arr = getattr(bench.data, "qfrc_applied", None)
            if arr is None:
                seen["absent"] = True
            else:
                seen["max_abs"] = max(seen["max_abs"], float(np.abs(arr).max()))
                seen["checked"] += 1

    result = sim.run(res["ref_rad"], res.n,
                     ref_vel0=float(res["ref_vel_rad_s"][0]), on_step=on_step)
    metrics = bench_metrics(result, res, float(bench.timestep))
    os.makedirs(WORK_DIR, exist_ok=True)
    csv = blog.write_bench_csv(
        os.path.join(WORK_DIR, f"{tag}.csv"), log,
        [f"subject {SUBJECT}; NULL TEST run '{tag}' by experiments/layer_null_test.py",
         f"model {bench.relpath()} (UNMODIFIED); kp={KP} kv={KD}; "
         f"dt={bench.timestep}",
         "this file is a TEST artefact, not a reported result"])
    return sim, result, metrics, csv, seen


run_once.__doc__ = run_once.__doc__ % (KP, KD)


# ------------------------------------------------------------------------------- 1
def compare_runs(a, b, m_a, m_b, csv_a, csv_b, label_a: str, label_b: str) -> int:
    """Require two runs to be bit-identical.  Returns the number of failures added."""
    before = len(FAILURES)
    worst_name, worst = "-", 0.0
    bad_arrays = []
    for name in RESULT_ARRAYS:
        x, y = getattr(a, name), getattr(b, name)
        if x.shape != y.shape:
            bad_arrays.append(f"{name} (shape {x.shape} vs {y.shape})")
            continue
        if not np.array_equal(x, y):
            d = float(np.abs(x.astype(float) - y.astype(float)).max())
            bad_arrays.append(f"{name} (max |diff| = {d:.3e})")
            if d > worst:
                worst_name, worst = name, d
    check(not bad_arrays,
          f"all {len(RESULT_ARRAYS)} per-step arrays bit-identical "
          f"({label_a} vs {label_b})",
          "" if not bad_arrays else "; ".join(bad_arrays))
    if bad_arrays:
        print(f"           worst column: {worst_name}, max |diff| = {worst:.6e}")

    bad_scalars = [f"{k}: {getattr(a, k)!r} vs {getattr(b, k)!r}"
                   for k in RESULT_SCALARS
                   if getattr(a, k) != getattr(b, k)]
    check(not bad_scalars, f"all {len(RESULT_SCALARS)} run scalars identical",
          "" if not bad_scalars else "; ".join(bad_scalars))

    shared = set(m_a) & set(m_b)
    check(set(m_a) == set(m_b), "the two runs report the same metric keys",
          f"{len(shared)} shared, {len(set(m_a) ^ set(m_b))} unique")
    bad_metrics = []
    for k in sorted(shared):
        va, vb = m_a[k], m_b[k]
        same = (va == vb) if not isinstance(va, float) else (
            va == vb or (math.isnan(va) and math.isnan(vb)))
        if not same:
            bad_metrics.append(f"{k}: {va!r} vs {vb!r}")
    check(not bad_metrics, f"all {len(shared)} metrics bit-identical",
          "" if not bad_metrics else "; ".join(bad_metrics))

    h_a, h_b = md5(csv_a, body_only=True), md5(csv_b, body_only=True)
    check(h_a == h_b,
          "the two written CSVs are byte-for-byte identical from the header row down",
          f"md5 {h_a[:16]}... / {h_b[:16]}...")
    return len(FAILURES) - before


def section_1_additivity(bench, res):
    print("\n" + "-" * 78)
    print("[1] ADDITIVITY IN-PROCESS -- the decisive test")
    print("-" * 78)
    print(f"    engine: {'STUB (tests/stub_mujoco.py)' if USING_STUB else 'REAL MuJoCo'}"
          f"   steps: {res.n}   dt: {bench.timestep} s")

    print("\n  [1a] DETERMINISM CONTROL: plain BenchSimulation, run TWICE.")
    print("       Nothing is being tested about the layer here.  This establishes that")
    print("       two runs of IDENTICAL code give IDENTICAL output, without which the")
    print("       comparison in [1b] would prove nothing.")
    plain = lambda b, c: BenchSimulation(b, c)                          # noqa: E731
    _, r1, m1, c1, _ = run_once(bench, res, plain, "plain_1")
    _, r2, m2, c2, _ = run_once(bench, res, plain, "plain_2")
    n_bad = compare_runs(r1, r2, m1, m2, c1, c2, "plain #1", "plain #2")
    if n_bad:
        print("\n       THE CONTROL FAILED.  Stop: the engine is not reproducible")
        print("       across resets in this configuration, so [1b] cannot be")
        print("       interpreted either way.  Fix this before reading anything below.")
        return None, None

    print("\n  [1b] THE NULL TEST: plain BenchSimulation vs DrivetrainBenchSimulation")
    print("       with DrivetrainLayer(PAPER, enabled=False), same reference, same")
    print("       gains, same process.  The layer object is CONSTRUCTED and SEEDED --")
    print("       it is present and inert, not absent.")
    layer = DrivetrainLayer(PAPER, enabled=False)
    off = lambda b, c: DrivetrainBenchSimulation(b, c, layer=layer)     # noqa: E731
    sim_off, r3, m3, c3, seen = run_once(bench, res, off, "layer_off",
                                         watch_applied=True)
    compare_runs(r1, r3, m1, m3, c1, c3, "plain", "layer OFF")

    check(layer.steps == 0,
          "the disabled layer advanced its own state ZERO times over the whole run",
          f"layer.steps = {layer.steps}, theta_a = {layer.theta_a!r}")
    check(sim_off.last_layer is None,
          "the disabled layer recorded no LayerStep", f"{sim_off.last_layer!r}")

    print("\n  [1c] THE HEADLINE NUMBERS FROM THE LAYER-OFF RUN")
    if USING_STUB:
        print("       WARNING -- THESE ARE STUB NUMBERS.  tests/stub_mujoco.py is a")
        print("       hand-written scalar plant, not MuJoCo, so these values are NOT")
        print("       the validated bench result and must never be quoted as it.  The")
        print("       oracle numbers are rms 4.3616 deg / peak tau 16.0041 N.m; the")
        print("       difference below is the stub, not the layer.  What section 1")
        print("       proves is that the two code paths AGREE, whatever the plant is.")
    for k in V.HEADLINE:
        if k in m3:
            print(f"       {k:<18} {m3[k]!r}")
    return r3, (m3, c3, seen)


# ------------------------------------------------------------------------------- 2
def section_2_qfrc_applied(bench, seen):
    print("\n" + "-" * 78)
    print("[2] THE DISABLED PATH WRITES NOTHING TO qfrc_applied")
    print("-" * 78)
    if seen["absent"] or not hasattr(bench.data, "qfrc_applied"):
        print("    `StubData` has NO qfrc_applied attribute.  That makes this the")
        print("    strongest form of the test available anywhere: had the disabled path")
        print("    written to it, section 1 would have raised AttributeError on step 0")
        print("    rather than passing.  Section 1 ran to completion over")
        print(f"    {seen['steps']} steps x 3 runs, so the disabled path provably")
        print("    never touched it.")
        check(True, "no write to qfrc_applied (proved by absence of the attribute)")
        return

    check(seen["max_abs"] == 0.0,
          f"qfrc_applied stayed exactly zero on ALL {bench.model.nv} dofs for every "
          f"one of {seen['checked']} steps",
          f"max |qfrc_applied| over the run = {seen['max_abs']!r}")

    print("\n    POSITIVE CONTROL -- so [2] cannot pass for a trivial reason such as")
    print("    the write going somewhere this test never looks.  ONE step, at a")
    print("    constant reference, with the layer enabled and the belt pre-wound.")
    print("    This is a wiring check, not a gait experiment: no reference trajectory")
    print("    is involved and nothing is logged or reported from it.")
    sim = DrivetrainBenchSimulation(bench, PDController.knee(bench, KP, KD),
                                    layer=DrivetrainLayer(PAPER, enabled=True))
    sim.reset(0.0)
    sim.layer.seed_from_joint_torque(0.0, 16.004120587370423)
    sim.step(0.0, 0)
    wrote = float(bench.data.qfrc_applied[bench.knee_dof])
    check(wrote != 0.0,
          "with the layer ENABLED, the same step writes a NONZERO knee torque",
          f"qfrc_applied[knee_dof] = {wrote!r} N.m")
    check(wrote == sim.last_layer.tau_j_mean,
          "and that value is bit-identical to layer.tau_j_mean")
    other = np.delete(np.abs(np.asarray(bench.data.qfrc_applied, float)),
                      bench.knee_dof)
    other_max = float(other.max()) if other.size else 0.0
    check(other_max == 0.0,
          "the enabled layer wrote to the knee dof ONLY, no other dof",
          f"max |qfrc_applied| elsewhere = {other_max!r} over {other.size} other dof(s)")


# ----------------------------------------------------------------------------- 3/4
def section_3_oracle(metrics, audit, csv_off):
    print("\n" + "-" * 78)
    print("[3] THE FROZEN ORACLE -- metrics")
    print("-" * 78)
    o_met = os.path.join(V.ORACLE_DIR, blog.METRICS_CSV_NAME)
    o_csv = os.path.join(V.ORACLE_DIR, blog.BENCH_CSV_NAME)
    for p in (o_met, o_csv):
        if not os.path.isfile(p):
            check(False, f"oracle file present: {os.path.relpath(p, ROOT)}")
            return
    print(f"    frozen : {os.path.relpath(o_met, ROOT)}")
    print(f"    tolerance: bit-exact preferred, {V.METRIC_RTOL:g} relative accepted "
          f"(verify_against_oracle.py's own rule, imported not copied)")
    old = blog.read_metrics_csv(o_met)
    bad, lines = V.compare_metrics(metrics, audit, old)
    print("\n".join(lines))
    check(bad == 0, "every metric and every audit field matches the frozen oracle",
          f"{bad} disagreement(s)")

    exact = [k for k in V.HEADLINE
             if k in metrics and k in old and float(metrics[k]) == float(old[k])]
    check(len(exact) == len(V.HEADLINE),
          f"all {len(V.HEADLINE)} HEADLINE metrics are BIT-EXACT, not merely within "
          f"tolerance", f"{len(exact)}/{len(V.HEADLINE)}")

    print("\n" + "-" * 78)
    print("[4] THE FROZEN ORACLE -- all 21 trace columns")
    print("-" * 78)
    bad_t, lines = V.compare_trace(csv_off, o_csv)
    print("\n".join(lines))
    check(bad_t == 0, "every logged column matches the frozen oracle trace",
          f"{bad_t} column(s) disagree")

    same_bytes = md5(csv_off, body_only=True) == md5(o_csv, body_only=True)
    check(same_bytes,
          "the layer-off trace and the frozen oracle trace are BYTE-IDENTICAL from the "
          "header row down",
          "(the `#` provenance lines differ by design and are excluded)")


# ---------------------------------------------------------------------------- main
def main() -> int:
    print(__doc__.split("Run:")[0].rstrip())
    print("=" * 78)
    if USING_STUB:
        print("MuJoCo IS NOT AVAILABLE IN THIS INTERPRETER.")
        print("  Section 1 runs against tests/stub_mujoco.py -- see the docstring for")
        print("  why that is a real result and not a placeholder.")
        print("  Sections 2 (real qfrc_applied), 3 (oracle metrics) and 4 (oracle")
        print("  trace) require the compiled model and are SKIPPED.  They are owed to")
        print("  exactly one command:")
        print("      .venv\\Scripts\\python.exe experiments\\layer_null_test.py")
    else:
        print(f"Engine: REAL MuJoCo.  All sections will run.")
    print("=" * 78)

    bench, res, audit, dt = prepare()
    r_off, extra = section_1_additivity(bench, res)
    if r_off is None:
        print("\nABORTED after the determinism control failed.")
        return 2
    metrics, csv_off, seen = extra
    section_2_qfrc_applied(bench, seen)
    if USING_STUB:
        print("\n" + "-" * 78)
        print("[3]/[4] THE FROZEN ORACLE -- SKIPPED, no MuJoCo here")
        print("-" * 78)
        print("    The oracle files record the output of the REAL engine.  Comparing a")
        print("    stub run against them would fail for the right reason and teach")
        print("    nothing, so it is not attempted.  What section 1 established still")
        print("    stands on its own: the disabled layer changes nothing.")
    else:
        section_3_oracle(metrics, audit, csv_off)
        print(f"\n    model limits unchanged after all runs: "
              f"{bench.limits_unchanged()}")
        check(bench.limits_unchanged(),
              "forcerange and ctrlrange are untouched after every run")

    print("\n" + "=" * 78)
    if FAILURES:
        print(f"NULL TEST: {len(FAILURES)} FAILURE(S)")
        for f in FAILURES:
            print(f"    - {f}")
        print("  Do NOT report any enabled-layer number until this passes.")
        return 1
    scope = ("sections 1-2 on the stub; 3-4 still owed to the real engine"
             if USING_STUB else "all sections, real MuJoCo")
    print(f"NULL TEST: all checks passed  ({scope})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
