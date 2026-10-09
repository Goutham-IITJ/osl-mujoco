"""Armature pilot -- does the actuator idealisation change the DESIGN, or just the number?

Part 4b argued that the research direction only has a pulse if an actuator idealisation
moves the *selected gains*, not merely the predicted error.  The `armature` parameter is
the sharpest test available without hardware: it is a pure PLACEHOLDER (0.01 kg.m^2) that
Part 1 showed is plausibly 7-49x too small, because MuJoCo's armature is the reflected
rotor inertia N^2 * J_rotor and the OSL V2 knee's reduction is N ~ 49.4 (N^2 = 2440).

Three questions, in order of what they settle:

  A  FIXED GAINS, varying plant.  Is the AB19 tracking metric even sensitive to inertia?
  B  RE-TUNED GAINS.  Where does the repo's own selection rule land on each plant, and
     what does a controller tuned on the placeholder plant cost when deployed elsewhere?
  C  STEP RESPONSE.  Same fixed gains -- does the transient behaviour move?

Nothing is modified on disk except this experiment's own output directory.  The armature
is written into the COMPILED model in memory, exactly as PDController.write_to_model
writes the gains; models/osl_v2_bench.xml is never opened for writing.

Run:  .venv\\Scripts\\python.exe experiments\\run_armature_pilot.py
"""
from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from oslbench.controller import PDController, kd_for_damping_ratio
from oslbench.model import load_bench_model
from oslbench.reference import load_reference, resample_reference
from oslbench.simulation import BenchSimulation

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_OUTDIR = os.path.join(ROOT, "build", "armature_pilot")

# The frozen validated benchmark result.  Used as the tuning SPEC in experiment B:
# "the smallest Kp that still meets the accuracy we already achieved" is a specification,
# not an arbitrary cost weight.
SPEC_RMS_DEG = 4.3616

# armature ladder.  0.010 is the authored placeholder; the rest are N^2 * J_rotor for
# J_rotor = 3e-5 .. 2e-4 kg.m^2, the normal range for a BLDC rotor of this actuator class.
LADDER = (
    (0.0100, "PLACEHOLDER (authored)"),
    (0.0732, "J_rotor = 3e-5"),
    (0.1220, "J_rotor = 5e-5"),
    (0.2440, "J_rotor = 1e-4"),
    (0.4881, "J_rotor = 2e-4"),
)


def set_armature(bench, value: float) -> float:
    """Write the knee's reflected inertia into the compiled model.  Returns the new I_eff.

    This is an in-memory write to mjModel.dof_armature, the same class of operation as
    PDController.write_to_model.  The XML is untouched.
    """
    bench.model.dof_armature[bench.knee_dof] = float(value)
    return float(bench.i_body) + float(value)


def rms(x) -> float:
    return float(np.sqrt(np.mean(np.asarray(x, float) ** 2)))


def track(bench, ref_rad, kp: float, kd: float):
    """One AB19 cycle at the given gains on the current plant.  Returns (rms_deg, peak_deg,
    peak_tau)."""
    sim = BenchSimulation(bench, knee=PDController.knee(bench, kp=kp, kd=kd))
    out = sim.run(ref_rad)
    err = np.asarray(out.error_deg, float)
    return rms(err), float(np.max(np.abs(err))), float(np.max(np.abs(out.tau)))


def step_response(bench, kp: float, kd: float,
                  start_deg: float = 30.0, end_deg: float = 40.0, seconds: float = 1.5):
    """A step at fixed gains.  Returns (overshoot_percent, peak_time_s).

    Reported alongside the tracking metric because the two are sensitive to different
    things: see the note in the printed summary.
    """
    n = int(round(seconds / bench.timestep))
    target = math.radians(end_deg)
    sim = BenchSimulation(bench, knee=PDController.knee(bench, kp=kp, kd=kd))
    sim.reset(math.radians(start_deg))
    q = np.empty(n)
    for k in range(n):
        q[k] = sim.step(target, k).q
    final = float(q[-1])
    travel = final - math.radians(start_deg)
    if abs(travel) < 1e-9:
        return float("nan"), float("nan")
    overshoot = (float(np.max(q)) - final) / abs(travel) * 100.0
    return overshoot, float(np.argmax(q)) * bench.timestep


def tune(bench, ref_rad, i_eff: float, kp_grid, zeta: float, spec: float):
    """The repo's own selection rule, applied to whatever plant is currently loaded.

    Smallest Kp on the grid meeting the RMS spec, with Kd from the zeta rule evaluated on
    THIS plant's I_eff -- i.e. the gains an engineer would pick if this model were the
    truth.
    """
    for kp in kp_grid:
        kd = kd_for_damping_ratio(float(kp), i_eff, bench.b_joint, zeta)
        r, _, _ = track(bench, ref_rad, float(kp), kd)
        if r <= spec:
            return float(kp), float(kd), r
    return float("nan"), float("nan"), float("nan")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--outdir", default=DEFAULT_OUTDIR)
    ap.add_argument("--zeta", type=float, default=0.7)
    ap.add_argument("--spec", type=float, default=SPEC_RMS_DEG,
                    help="RMS tracking spec in degrees for the re-tuning experiment")
    ap.add_argument("--kp-min", type=float, default=100.0)
    ap.add_argument("--kp-max", type=float, default=8000.0)
    ap.add_argument("--kp-points", type=int, default=110)
    ap.add_argument("--skip-retune", action="store_true",
                    help="run A and C only (much faster)")
    args = ap.parse_args()
    os.makedirs(args.outdir, exist_ok=True)

    bench = load_bench_model(verbose=False)
    if bench.failures:
        print("model failed its structural checks:")
        for f in bench.failures:
            print("   ", f)
        return 1

    ref = load_reference()
    res = resample_reference(ref, bench.timestep, cycles=1)
    ref_rad = np.asarray(res.ref_rad, float)
    qd_ref = np.asarray(res.ref_vel_rad_s, float)

    armature0 = float(bench.armature)
    kp0 = 600.0
    kd0 = kd_for_damping_ratio(kp0, bench.i_body + armature0, bench.b_joint, args.zeta)

    print(f"model      {bench.relpath()}")
    print(f"I_body     {bench.i_body:.6f} kg.m^2   (CAD-derived)")
    print(f"armature   {armature0:.6f} kg.m^2   (PLACEHOLDER as authored)")
    print(f"b_joint    {bench.b_joint:g} N.m.s/rad,  frictionloss {bench.frictionloss:g} N.m")
    print(f"reference  {len(ref_rad)} steps, rms(qdot_ref) = {rms(qd_ref):.6f} rad/s")
    print(f"gains held for A and C:  Kp = {kp0:g}, Kd = {kd0:.3f}\n")

    kp_grid = np.unique(np.round(np.geomspace(args.kp_min, args.kp_max, args.kp_points)))
    rows = []

    # ------------------------------------------------------------------ A and C
    print("=" * 78)
    print("A/C  FIXED GAINS (Kp = %g, Kd = %.3f), varying the reflected inertia" % (kp0, kd0))
    print("=" * 78)
    print("%9s %8s %7s %8s %9s %9s %9s %10s"
          % ("armature", "I_eff", "zeta", "omega_n", "RMS deg", "peak deg",
             "peak N.m", "overshoot"))
    for a, label in LADDER:
        i_eff = set_armature(bench, a)
        zeta = (kd0 + bench.b_joint) / (2.0 * math.sqrt(kp0 * i_eff))
        wn = math.sqrt(kp0 / i_eff)
        r, pk, tau = track(bench, ref_rad, kp0, kd0)
        over, tpk = step_response(bench, kp0, kd0)
        rows.append(dict(armature=a, label=label, i_eff=i_eff, zeta=zeta, omega_n=wn,
                         rms_fixed=r, peak_fixed=pk, tau_fixed=tau,
                         overshoot=over, peak_time=tpk))
        print("%9.4f %8.4f %7.3f %8.2f %9.3f %9.3f %9.2f %9.1f%%"
              % (a, i_eff, zeta, wn, r, pk, tau, over))

    # ------------------------------------------------------------------------ B
    if not args.skip_retune:
        print()
        print("=" * 78)
        print("B    RE-TUNED per plant by the repo's own rule "
              "(smallest Kp with RMS <= %.4f deg, Kd from zeta = %g)" % (args.spec, args.zeta))
        print("=" * 78)
        for row in rows:
            set_armature(bench, row["armature"])
            kp_s, kd_s, r_s = tune(bench, ref_rad, row["i_eff"], kp_grid, args.zeta, args.spec)
            row.update(kp_star=kp_s, kd_star=kd_s, rms_retuned=r_s)
            print("armature %7.4f  ->  Kp* = %7.0f   Kd* = %6.2f   (RMS %.3f)   %s"
                  % (row["armature"], kp_s, kd_s, r_s, row["label"]))

        base = rows[0]
        print("\n     TRANSFER -- gains tuned on the PLACEHOLDER plant "
              "(Kp = %.0f, Kd = %.2f) deployed elsewhere:" % (base["kp_star"], base["kd_star"]))
        print("     %9s %11s %11s %16s %15s"
              % ("armature", "Kp*(true)", "Kp* ratio", "RMS ideal-tuned", "RMS true-tuned"))
        for row in rows:
            set_armature(bench, row["armature"])
            r_x, _, _ = track(bench, ref_rad, base["kp_star"], base["kd_star"])
            row["rms_transfer"] = r_x
            print("     %9.4f %11.0f %10.2fx %16.3f %15.3f"
                  % (row["armature"], row["kp_star"], row["kp_star"] / base["kp_star"],
                     r_x, row["rms_retuned"]))

    # ------------------------------------------------------------------- restore
    set_armature(bench, armature0)

    # ----------------------------------------------------------------------- CSV
    keys = sorted({k for r in rows for k in r})
    keys = ["armature", "label"] + [k for k in keys if k not in ("armature", "label")]
    path = os.path.join(args.outdir, "armature_pilot.csv")
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(",".join(keys) + "\n")
        for r in rows:
            fh.write(",".join(str(r.get(k, "")) for k in keys) + "\n")

    # ------------------------------------------------------------------- verdict
    span_rms = rows[-1]["rms_fixed"] / rows[0]["rms_fixed"]
    span_over = rows[-1]["overshoot"] / rows[0]["overshoot"]
    print("\n" + "=" * 78)
    print("VERDICT")
    print("=" * 78)
    print("  a 48x change in reflected inertia moves")
    print("      AB19 tracking RMS  by %5.2fx   <- the metric the bench currently reports"
          % span_rms)
    print("      step overshoot     by %5.2fx   <- the metric it does not" % span_over)
    if not args.skip_retune:
        print("      selected Kp*       by %5.2fx"
              % (rows[-1]["kp_star"] / rows[0]["kp_star"]))
    print("\n  If tracking RMS barely moves while overshoot and the selected gain move a")
    print("  lot, then steady tracking error on a smooth periodic reference is the WRONG")
    print("  observable for an actuator-fidelity study, and the transient/stability")
    print("  metrics are the right ones.  See docs/ARMATURE_PILOT.md.")
    print(f"\nwrote {os.path.relpath(path, ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
