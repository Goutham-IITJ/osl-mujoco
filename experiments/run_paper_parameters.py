r"""Stage 1 of docs/ACTUATOR_DYNAMICS_ANALYSIS.md -- the paper-derived RIGID-EQUIVALENT plant.

WHAT THIS IS
    Best et al. (2025), IEEE/ASME T-MECH 30(6):4732, measured the OSL V2 actuator at the
    ACTUATOR OUTPUT shaft (post-9:1-planetary):

        J_a = 9.83e-3 kg.m^2     B_a = 6.06e-2 N.m.s/rad     f_c = 17.1e-2 N.m

    In the RIGID-BELT LIMIT ONLY (theta_s == 0, so theta_a = n_t * theta_j) those refer to
    the joint as n_t^2*J_a, n_t^2*B_a and n_t*f_c.  With the paper's n_t = 4.61:

        armature      0.2089  kg.m^2       vs the authored placeholder 0.0100   (21x)
        damping       1.2879  N.m.s/rad    vs the authored placeholder 0.3000   (4.3x)
        frictionloss  0.7883  N.m          vs the authored placeholder 0.4000   (2.0x)

    This script writes those three numbers into the COMPILED model in memory, re-runs the
    two experiments the armature pilot used, restores the model, and reports the deltas.

WHAT THIS IS NOT
    NOT the actuator model.  Section 8.2 of the analysis shows the rigid-belt reduction is
    NOT valid at our bandwidth: the belt mode sits at 10-17 Hz against a closed loop of
    5.7-7.6 Hz, only ~1.8-2.4x separation.  The missing belt compliance is a LARGER gap
    than the inertia (Section 8.3: a 876 N.m/rad series spring against Kp = 600 would
    render 356 N.m/rad, a 41 % stiffness shortfall).  This run measures how much of the
    bench's behaviour is attributable to the inertia/damping/friction corrections ALONE,
    before any compliance is introduced.  Stage 4 is where compliance arrives.

    NOTHING here has been validated against hardware.  The paper characterised the ANKLE
    and skipped the knee as "redundant" given identical construction -- so every parameter
    used here is transferred across joints on the paper's authority.

THIS IS A TEST, NOT A FIT
    The five predicted values below are LITERALS copied out of
    docs/ACTUATOR_DYNAMICS_ANALYSIS.md Section 10, written before any of this was run.
    They are printed before the first simulation step.  If the measurements land near them
    the model is behaving as understood; if they do not, something in the chain is wrong
    and that is worth more than the experiment itself.

READ-ONLY.  models/osl_v2_bench.xml, tools/build_mjcf.py, oslbench/*, tests/oracle/* and
the AB19 reference data are never opened for writing.  The three dof properties are
written into the compiled mjModel exactly as PDController.write_to_model writes the gains,
and restored before exit.  A SHA-256 of every protected file is taken at entry and
re-checked at exit.

Run:  .venv\Scripts\python.exe experiments\run_paper_parameters.py
"""
from __future__ import annotations

import argparse
import hashlib
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from oslbench.controller import KD, KP
from oslbench.controller import PDController
from oslbench.model import BENCH_XML, dof_inertia, load_bench_model
from oslbench.reference import load_reference, resample_reference
from oslbench.simulation import BenchSimulation

try:
    import mujoco
except ImportError:                                              # pragma: no cover
    mujoco = None

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_OUTDIR = os.path.join(ROOT, "build", "paper_parameters")

# ---------------------------------------------------------------- PAPER-DERIVED
# Best et al. 2025, Section III-A1, Table of identified parameters (p. 4735).
# Verified against a rasterised image of that page, not only the text layer.
J_A = 9.83e-3          # kg.m^2       at the ACTUATOR OUTPUT -- already contains n_a^2
B_A = 6.06e-2          # N.m.s/rad    at the actuator output
F_C = 17.1e-2          # N.m          at the actuator output
F_G = 82.1e-3          # N.m/A        actuator-output torque per amp of rotor current
K_T = 110.8e-3         # N.m/A        ROTOR torque constant
N_A = 9.0              # planetary reduction inside the Dephy ActPack 4.1 / AK80-9
N_T_PAPER = 4.61       # belt reduction quoted by the paper
P1 = 876.0             # N.m/rad      belt stiffness at zero deflection
P2 = 14913.0           # N.m/rad^2    belt stiffening coefficient

# ------------------------------------------------------------------------- CAD
# Our Onshape export: p_b0001_InputPulley_5mm_11teeth + p_b0012_OutputPulley_5mm_50teeth
# + GT3_5mm_325mm_belt, each appearing TWICE in the URDF (once per joint).
N_T_CAD = 50.0 / 11.0  # = 4.545454...  COMPUTED, NOT USED IN THE MAIN RUN

# The three values actually written, rounded exactly as the analysis document states them
# so that the document and this script cannot drift apart.
PAPER_ARMATURE = 0.2089        # = round(N_T_PAPER**2 * J_A, 4)
PAPER_DAMPING = 1.2879         # = round(N_T_PAPER**2 * B_A, 4)
PAPER_FRICTIONLOSS = 0.7883    # = round(N_T_PAPER * F_C, 4)

# -------------------------------------------------------- FROZEN ORACLE VALUES
# tests/oracle/bench_track_ab19_metrics.csv.  Hard-coded here only as a fallback; the
# file itself is read at run time and is the authority.
ORACLE = dict(rms_err_deg=4.361613372292228, peak_err_deg=9.090662089393883,
              peak_tau_Nm=16.004120587370423, sat_pct=0.0)
ORACLE_METRICS = os.path.join(ROOT, "tests", "oracle", "bench_track_ab19_metrics.csv")

# ------------------------------------------------------- PREDICTIONS (LITERALS)
# docs/ACTUATOR_DYNAMICS_ANALYSIS.md Section 10, written before this script existed.
PREDICTED = dict(i_eff=0.4609, omega_n=36.08, zeta=0.5575,
                 overshoot_pct=12.1, peak_tau_Nm=28.2, rms_deg=4.58)

# Files this script must not touch.  Verified by hash at entry and exit.
PROTECTED = (
    os.path.join("models", "osl_v2_bench.xml"),
    os.path.join("tools", "build_mjcf.py"),
    os.path.join("oslbench", "model.py"),
    os.path.join("oslbench", "controller.py"),
    os.path.join("oslbench", "simulation.py"),
    os.path.join("tests", "oracle", "bench_track_ab19.csv"),
    os.path.join("tests", "oracle", "bench_track_ab19_metrics.csv"),
)


# --------------------------------------------------------------------- helpers
def sha256(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def fingerprint() -> dict:
    """SHA-256 of every protected file that exists.  Compared at exit."""
    out = {}
    for rel in PROTECTED:
        p = os.path.join(ROOT, rel)
        if os.path.isfile(p):
            out[rel] = sha256(p)
    return out


def rms(x) -> float:
    return float(np.sqrt(np.mean(np.asarray(x, float) ** 2)))


def read_oracle() -> dict:
    """Read the frozen metrics CSV.  Read-only; falls back to the literals above."""
    if not os.path.isfile(ORACLE_METRICS):
        return dict(ORACLE, source="hard-coded literal (metrics CSV not found)")
    vals = {}
    with open(ORACLE_METRICS, "r", encoding="utf-8") as fh:
        for line in fh:
            if "," not in line:
                continue
            key, _, val = line.partition(",")
            try:
                vals[key.strip()] = float(val.strip().strip('"'))
            except ValueError:
                pass
    return dict(rms_err_deg=vals.get("rms_err_deg", ORACLE["rms_err_deg"]),
                peak_err_deg=vals.get("peak_err_deg", ORACLE["peak_err_deg"]),
                peak_tau_Nm=vals.get("peak_tau_Nm", ORACLE["peak_tau_Nm"]),
                sat_pct=vals.get("sat_pct", ORACLE["sat_pct"]),
                source=os.path.relpath(ORACLE_METRICS, ROOT))


def set_knee_dyn(bench, armature: float, damping: float, frictionloss: float) -> None:
    """Write the three knee dof properties into the COMPILED model.

    Same class of in-memory write as PDController.write_to_model.  The XML is untouched.
    MuJoCo reads dof_armature / dof_damping / dof_frictionloss every step, so these take
    effect immediately with no recompile.
    """
    d = bench.knee_dof
    bench.model.dof_armature[d] = float(armature)
    bench.model.dof_damping[d] = float(damping)
    bench.model.dof_frictionloss[d] = float(frictionloss)


def measure_i_eff(bench) -> float:
    """Re-measure M[knee,knee] out of the compiled model -- do not assume the write took.

    Deliberately measured rather than computed as i_body + armature, so that a silent
    failure to apply the write shows up as a number rather than as a clean-looking run.
    """
    model, data = bench.model, bench.data
    mujoco.mj_resetDataKeyframe(model, data, bench.keyframe_id())
    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)
    i_mul, _ = dof_inertia(model, data, bench.knee_dof)
    return float(i_mul)


def read_knee_dyn(bench) -> tuple:
    d = bench.knee_dof
    return (float(bench.model.dof_armature[d]),
            float(bench.model.dof_damping[d]),
            float(bench.model.dof_frictionloss[d]))


def track(bench, ref_rad, kp: float, kd: float) -> dict:
    """One AB19 cycle on whatever plant is currently loaded."""
    sim = BenchSimulation(bench, knee=PDController.knee(bench, kp=kp, kd=kd))
    out = sim.run(ref_rad)
    err = np.asarray(out.error_deg, float)
    return dict(rms_deg=rms(err),
                peak_err_deg=float(np.max(np.abs(err))),
                peak_tau_Nm=float(np.max(np.abs(out.tau))),
                sat_pct=100.0 * float(np.mean(out.saturated)),
                pct_authority=100.0 * float(np.max(np.abs(out.tau))) / bench.force_limit,
                clamped_steps=int(out.clamped_steps),
                ankle_dev_deg=math.degrees(float(out.ankle_dev_rad)))


def step_response(bench, kp: float, kd: float, start_deg: float = 30.0,
                  end_deg: float = 40.0, seconds: float = 1.5) -> dict:
    """The armature pilot's step task, unchanged: 10 deg step at 30 deg flexion, 1.5 s.

    Friction and damping stay ACTIVE -- this is not a frictionless test.
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
        return dict(overshoot_pct=float("nan"), peak_time_s=float("nan"),
                    final_deg=math.degrees(final))
    return dict(overshoot_pct=(float(np.max(q)) - final) / abs(travel) * 100.0,
                peak_time_s=float(np.argmax(q)) * bench.timestep,
                final_deg=math.degrees(final))


def second_order(kp: float, kd: float, i_eff: float, b: float) -> dict:
    """DERIVED, not measured: the textbook second-order quantities."""
    wn = math.sqrt(kp / i_eff)
    zeta = (kd + b) / (2.0 * math.sqrt(kp * i_eff))
    over = (math.exp(-math.pi * zeta / math.sqrt(1.0 - zeta * zeta)) * 100.0
            if zeta < 1.0 else 0.0)
    return dict(omega_n=wn, f_n_hz=wn / (2.0 * math.pi), zeta=zeta,
                overshoot_theory_pct=over)


def delta(new: float, old: float) -> str:
    """'x2.90' style ratio, or an absolute delta when the baseline is zero."""
    if old == 0.0:
        return f"{new:+.4f} abs"
    return f"x{new / old:.3f}"


# ------------------------------------------------------------------------ main
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--outdir", default=DEFAULT_OUTDIR)
    ap.add_argument("--kp", type=float, default=KP, help="held fixed; default 600")
    ap.add_argument("--kd", type=float, default=KD, help="held fixed; default 17.253")
    args = ap.parse_args()
    os.makedirs(args.outdir, exist_ok=True)

    if mujoco is None:
        sys.exit("FATAL: mujoco is not importable in this interpreter.\n"
                 "  Use osl-mujoco's own venv:  .venv\\Scripts\\python.exe ...")

    before = fingerprint()
    xml_sha_before = before.get(os.path.join("models", "osl_v2_bench.xml"), "")

    # ------------------------------------------------------- load and precondition
    bench = load_bench_model(verbose=False)
    if bench.failures:
        print("model failed its structural checks -- refusing to run:")
        for f in bench.failures:
            print("   ", f)
        return 1

    arm0, dmp0, frc0 = read_knee_dyn(bench)
    i_eff0 = measure_i_eff(bench)
    i_body = i_eff0 - arm0
    kp, kd = float(args.kp), float(args.kd)

    ref = load_reference()
    res = resample_reference(ref, bench.timestep, cycles=1)
    ref_rad = np.asarray(res.ref_rad, float)
    qd_ref = np.asarray(res.ref_vel_rad_s, float)
    rms_qd = rms(qd_ref)
    oracle = read_oracle()

    bar = "=" * 79
    print(bar)
    print("STAGE 1  --  paper-derived RIGID-EQUIVALENT actuator parameters")
    print("            Best et al. 2025, T-MECH 30(6):4732, DOI 10.1109/TMECH.2024.3508469")
    print(bar)
    print(f"model            {bench.relpath()}")
    print(f"                 sha256 {xml_sha_before[:16]}...  (re-checked at exit)")
    print(f"reference        {len(ref_rad)} steps, rms(qdot_ref) = {rms_qd:.6f} rad/s")
    print(f"gains HELD FIXED Kp = {kp:g} N.m/rad, Kd = {kd:g} N.m.s/rad   "
          f"(the validated pair, not re-tuned)")
    print(f"oracle source    {oracle['source']}")

    # ------------------------------------------------- PAPER-DERIVED arithmetic
    print("\n" + bar)
    print("PAPER-DERIVED  --  parameters read out of Best et al. Section III-A1")
    print(bar)
    print(f"  k_t = {K_T:.4g} N.m/A   (ROTOR torque constant)")
    print(f"  n_a = {N_A:g}             (ActPack 4.1 planetary, INSIDE the actuator)")
    print(f"  J_a = {J_A:.4g} kg.m^2    at the ACTUATOR OUTPUT -- already contains n_a^2")
    print(f"        implied rotor inertia J_a/n_a^2 = {J_A / N_A**2:.4g} kg.m^2")
    print(f"  B_a = {B_A:.4g} N.m.s/rad at the actuator output")
    print(f"  f_c = {F_C:.4g} N.m       at the actuator output")
    print(f"  f_g = {F_G:.4g} N.m/A     CURRENT-DEPENDENT -- cannot be expressed by "
          f"MuJoCo frictionloss")
    print(f"  n_t = {N_T_PAPER:g}          belt reduction quoted by the paper")
    print(f"  k_t*n_a         = {K_T * N_A:.4f} N.m/A   torque constant at the actuator output")
    print(f"  k_t*n_a*n_t     = {K_T * N_A * N_T_PAPER:.4f} N.m/A   at the joint")

    print("\n  RIGID-BELT REDUCTION (theta_s == 0, so theta_a = n_t*theta_j).  DERIVED:")
    print("      %-18s %12s %12s %12s" % ("joint-side", "n_t = 4.61", "authored", "ratio"))
    for name, expr, val, old in (
            ("n_t^2 * J_a", "armature", N_T_PAPER**2 * J_A, arm0),
            ("n_t^2 * B_a", "damping", N_T_PAPER**2 * B_A, dmp0),
            ("n_t   * f_c", "frictionloss", N_T_PAPER * F_C, frc0)):
        print("      %-18s %12.6f %12.4f %11.2fx" % (f"{name} -> {expr}", val, old, val / old))

    # the current-dependent friction, which is the part that does not fit in the model
    tau_a_at_16 = oracle["peak_tau_Nm"] / N_T_PAPER
    i_q = (tau_a_at_16 + F_C) / (K_T * N_A - F_G)
    print(f"\n  CURRENT-DEPENDENT FRICTION, at the oracle's {oracle['peak_tau_Nm']:.2f} N.m peak:")
    print(f"      tau_a  = tau_j/n_t                        = {tau_a_at_16:.4f} N.m")
    print(f"      I_q    = (tau_a + f_c)/(k_t*n_a - f_g)    = {i_q:.3f} A")
    print(f"      joint friction n_t*(f_c + f_g*|I_q|)      = "
          f"{N_T_PAPER * (F_C + F_G * i_q):.4f} N.m   "
          f"({N_T_PAPER * (F_C + F_G * i_q) / frc0:.2f}x the authored {frc0:g})")
    print(f"      of which only {N_T_PAPER * F_C:.4f} N.m is constant -- the VARIABLE part "
          f"({N_T_PAPER * F_G * i_q:.4f} N.m) is larger,")
    print("      and MuJoCo frictionloss cannot represent it.  NOT modelled in this run.")

    # ------------------------------------------- item 9: CAD ratio, NOT substituted
    print("\n" + bar)
    print("CAD-RATIO COMPARISON  --  COMPUTED ONLY, *NOT* SUBSTITUTED INTO THE MAIN RUN")
    print(bar)
    print(f"  Our Onshape export fixes the belt at 50/11 = {N_T_CAD:.6f} on BOTH joints")
    print("    p_b0001_InputPulley_5mm_11teeth.stl  +  p_b0012_OutputPulley_5mm_50teeth.stl")
    print("    +  GT3_5mm_325mm_belt.stl            each appearing twice in the URDF")
    print(f"  Paper n_t = {N_T_PAPER:g} (3 mm-pitch build); ours is the 5 mm-pitch variant, "
          f"which the")
    print("  paper says is STIFFER -- so p1 = 876 N.m/rad is likely a LOWER BOUND for ours.")
    print("      %-16s %14s %14s %10s" % ("quantity", "n_t = 4.61", "n_t = 50/11", "diff"))
    for name, val_p, val_c in (
            ("n_t^2 * J_a", N_T_PAPER**2 * J_A, N_T_CAD**2 * J_A),
            ("n_t^2 * B_a", N_T_PAPER**2 * B_A, N_T_CAD**2 * B_A),
            ("n_t   * f_c", N_T_PAPER * F_C, N_T_CAD * F_C)):
        print("      %-16s %14.6f %14.6f %9.2f%%"
              % (name, val_p, val_c, 100.0 * (val_c - val_p) / val_p))
    print(f"      {'total ratio n_a*n_t':16s} {N_A * N_T_PAPER:14.4f} "
          f"{N_A * N_T_CAD:14.4f} {100.0 * (N_A * N_T_CAD - N_A * N_T_PAPER) / (N_A * N_T_PAPER):8.2f}%")
    print("  The two ratios differ by under 3 %, so this run's conclusions do not hinge on")
    print("  which is used.  Recorded so the choice is visible rather than implicit.")

    # ----------------------------------------------------- PREDICTIONS, UP FRONT
    pred_derived = second_order(kp, kd, i_body + PAPER_ARMATURE, PAPER_DAMPING)
    pred_rms = math.degrees((kd + PAPER_DAMPING) / kp * rms_qd)
    print("\n" + bar)
    print("PREDICTED  --  literals from docs/ACTUATOR_DYNAMICS_ANALYSIS.md Section 10,")
    print("               printed BEFORE the first simulation step.  This is a test, not a fit.")
    print(bar)
    print("      %-22s %14s %16s" % ("quantity", "doc literal", "re-derived here"))
    print("      %-22s %14.4f %16.4f" % ("I_eff (kg.m^2)", PREDICTED["i_eff"],
                                         i_body + PAPER_ARMATURE))
    print("      %-22s %14.2f %16.2f" % ("omega_n (rad/s)", PREDICTED["omega_n"],
                                         pred_derived["omega_n"]))
    print("      %-22s %14.4f %16.4f" % ("zeta", PREDICTED["zeta"], pred_derived["zeta"]))
    print("      %-22s %14.1f %16.1f" % ("step overshoot (%)", PREDICTED["overshoot_pct"],
                                         pred_derived["overshoot_theory_pct"]))
    print("      %-22s %14.1f %16.1f" % ("peak AB19 torque (N.m)", PREDICTED["peak_tau_Nm"],
                                         1.06 * (i_body + PAPER_ARMATURE) * 57.72))
    print("      %-22s %14.2f %16.2f" % ("RMS error (deg)", PREDICTED["rms_deg"], pred_rms))
    print("  peak torque uses the pilot's MEASURED 1.06 * I_eff * qddot_peak relation;")
    print("  RMS uses the closed-form lag law e = (Kd + b)*rms(qdot_ref)/Kp.")

    # -------------------------------------------------- validity limit, up front
    ks0 = P1
    ks16 = math.sqrt(P1**2 + 4.0 * P2 * oracle["peak_tau_Nm"])
    w_locked = math.sqrt(ks0 / PAPER_ARMATURE)
    w_two = math.sqrt(ks0 * (1.0 / PAPER_ARMATURE + 1.0 / i_body))
    rendered = kp * ks0 / (kp + ks0)
    print("\n" + bar)
    print("VALIDITY LIMIT  --  why this is an APPROXIMATION, not the actuator model")
    print(bar)
    print(f"  belt stiffness  K_s(0) = p1              = {ks0:.0f} N.m/rad")
    print(f"                  K_s at {oracle['peak_tau_Nm']:.1f} N.m               = {ks16:.0f} N.m/rad"
          f"   (= sqrt(p1^2 + 4*p2*|tau|))")
    print(f"  belt mode, actuator vs held joint        = {w_locked:.1f} rad/s = "
          f"{w_locked / (2 * math.pi):.2f} Hz")
    print(f"  belt mode, free two-mass                 = {w_two:.1f} rad/s = "
          f"{w_two / (2 * math.pi):.2f} Hz")
    print(f"  our closed loop with this I_eff          = {pred_derived['omega_n']:.1f} rad/s = "
          f"{pred_derived['f_n_hz']:.2f} Hz")
    print(f"  SEPARATION                               = "
          f"{w_locked / pred_derived['omega_n']:.2f}x to {w_two / pred_derived['omega_n']:.2f}x"
          f"   -- too small for lumping to be exact")
    print(f"  series-stiffness loss NOT modelled here: Kp*K_s/(Kp+K_s) = {rendered:.1f} "
          f"N.m/rad,")
    print(f"      a {100.0 * (1.0 - rendered / kp):.1f} % shortfall against the commanded "
          f"Kp = {kp:g}.  This is the paper's")
    print("      measured C0 failure mode and it is invisible in every number below.")
    print(f"  Coulomb deadband f_c/Kp: {math.degrees(frc0 / kp):.4f} deg authored -> "
          f"{math.degrees(PAPER_FRICTIONLOSS / kp):.4f} deg here (vs a 10 deg step)")

    rows = []

    def run_plant(tag: str, source: str, armature: float, damping: float,
                  frictionloss: float) -> dict:
        set_knee_dyn(bench, armature, damping, frictionloss)
        a, d, f = read_knee_dyn(bench)
        i_eff = measure_i_eff(bench)              # MEASURED out of the compiled model
        der = second_order(kp, kd, i_eff, d)
        gait = track(bench, ref_rad, kp, kd)      # MEASURED
        stp = step_response(bench, kp, kd)        # MEASURED
        row = dict(tag=tag, source=source, armature=a, damping=d, frictionloss=f,
                   i_eff_measured=i_eff, **der, **gait, **stp)
        rows.append(row)
        return row

    # -------------------------------------------------------------------- run it
    print("\n" + bar)
    print("MEASURED  --  MuJoCo output.  Gains fixed at Kp = %g, Kd = %g throughout."
          % (kp, kd))
    print(bar)
    print("%-26s %9s %9s %8s %8s %9s %9s %8s %9s"
          % ("plant", "armature", "I_eff", "zeta", "omega_n", "RMS deg", "peak deg",
             "peak Nm", "overshoot"))

    def show(row: dict) -> None:
        print("%-26s %9.4f %9.6f %8.4f %8.2f %9.4f %9.4f %8.3f %8.2f%%"
              % (row["tag"], row["armature"], row["i_eff_measured"], row["zeta"],
                 row["omega_n"], row["rms_deg"], row["peak_err_deg"],
                 row["peak_tau_Nm"], row["overshoot_pct"]))

    base = run_plant("as-authored (control)", "PLACEHOLDER", arm0, dmp0, frc0)
    show(base)
    paper = run_plant("paper rigid-equivalent", "PAPER-DERIVED n_t=4.61",
                      PAPER_ARMATURE, PAPER_DAMPING, PAPER_FRICTIONLOSS)
    show(paper)

    # ------------------------------------------------------------------- restore
    set_knee_dyn(bench, arm0, dmp0, frc0)
    restored = run_plant("as-authored (restored)", "PLACEHOLDER", arm0, dmp0, frc0)
    show(restored)

    # ---------------------------------------------- did the control reproduce it?
    print("\n" + bar)
    print("CONTROL CHECK  --  does the as-authored plant still reproduce the frozen oracle?")
    print(bar)
    print("      %-24s %14s %14s %12s" % ("metric", "oracle", "measured", "delta"))
    ctrl_ok = True
    for key, name, tol in (("rms_deg", "RMS error (deg)", 1e-3),
                           ("peak_err_deg", "peak error (deg)", 1e-3),
                           ("peak_tau_Nm", "peak torque (N.m)", 1e-2),
                           ("sat_pct", "saturation (%)", 1e-9)):
        okey = {"rms_deg": "rms_err_deg", "peak_err_deg": "peak_err_deg",
                "peak_tau_Nm": "peak_tau_Nm", "sat_pct": "sat_pct"}[key]
        got, want = base[key], oracle[okey]
        ok = abs(got - want) <= tol
        ctrl_ok &= ok
        print("      %-24s %14.6f %14.6f %12.2e  %s"
              % (name, want, got, got - want, "ok" if ok else "MISMATCH"))
    print("      %-24s %14s %14s"
          % ("restore bit-identical?", "", "YES" if restored["rms_deg"] == base["rms_deg"]
             else "NO"))

    # ----------------------------------------------------- predicted vs measured
    print("\n" + bar)
    print("PREDICTED vs MEASURED  --  the actual test")
    print(bar)
    print("      %-24s %12s %12s %12s" % ("quantity", "predicted", "MEASURED", "error"))
    for name, pkey, mkey, fmt in (
            ("I_eff (kg.m^2)", "i_eff", "i_eff_measured", "%12.4f"),
            ("omega_n (rad/s)", "omega_n", "omega_n", "%12.2f"),
            ("zeta", "zeta", "zeta", "%12.4f"),
            ("step overshoot (%)", "overshoot_pct", "overshoot_pct", "%12.2f"),
            ("peak AB19 torque (N.m)", "peak_tau_Nm", "peak_tau_Nm", "%12.3f"),
            ("RMS error (deg)", "rms_deg", "rms_deg", "%12.4f")):
        p, m = PREDICTED[pkey], paper[mkey]
        print(("      %-24s " + fmt + " " + fmt + " %11.1f%%")
              % (name, p, m, 100.0 * (m - p) / p))
    print("  omega_n and zeta are DERIVED on both sides, so their agreement only confirms")
    print("  the write took effect.  Overshoot, torque and RMS are the real predictions.")
    print("  Second-order overshoot theory on the MEASURED zeta: %.2f %% vs measured %.2f %%"
          % (paper["overshoot_theory_pct"], paper["overshoot_pct"]))

    # -------------------------------------------------------- the seven deltas
    print("\n" + bar)
    print("BASELINE vs PAPER-DERIVED  --  the seven comparisons requested")
    print(bar)
    print("      %-24s %14s %14s %12s" % ("quantity", "as-authored", "paper-derived", "change"))
    for name, key in (("RMS error (deg)", "rms_deg"),
                      ("peak error (deg)", "peak_err_deg"),
                      ("peak torque (N.m)", "peak_tau_Nm"),
                      ("saturation (%)", "sat_pct"),
                      ("step overshoot (%)", "overshoot_pct"),
                      ("omega_n (rad/s) DERIVED", "omega_n"),
                      ("zeta DERIVED", "zeta")):
        print("      %-24s %14.4f %14.4f %12s"
              % (name, base[key], paper[key], delta(paper[key], base[key])))
    print("      %-24s %14.2f %14.2f %12s"
          % ("torque authority used %", base["pct_authority"], paper["pct_authority"],
             delta(paper["pct_authority"], base["pct_authority"])))

    # ------------------------------------------------------------------- the CSV
    keys = ["tag", "source", "armature", "damping", "frictionloss", "i_eff_measured",
            "omega_n", "f_n_hz", "zeta", "overshoot_theory_pct",
            "rms_deg", "peak_err_deg", "peak_tau_Nm", "sat_pct", "pct_authority",
            "overshoot_pct", "peak_time_s", "final_deg", "clamped_steps", "ankle_dev_deg"]
    path = os.path.join(args.outdir, "paper_parameters.csv")
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("# Stage 1, docs/ACTUATOR_DYNAMICS_ANALYSIS.md.  Gains fixed "
                 f"Kp={kp:g} Kd={kd:g}.\n")
        fh.write("# armature/damping/frictionloss written into the COMPILED model only; "
                 "XML untouched.\n")
        fh.write(f"# PAPER-DERIVED n_t={N_T_PAPER:g}: J_a={J_A:g} B_a={B_A:g} f_c={F_C:g}; "
                 f"CAD n_t={N_T_CAD:.6f} computed but NOT substituted.\n")
        fh.write("# RIGID-BELT APPROXIMATION -- belt compliance (p1=876) NOT modelled.\n")
        fh.write(",".join(keys) + "\n")
        for r in rows:
            fh.write(",".join(str(r.get(k, "")) for k in keys) + "\n")

    # ------------------------------------------------------------ integrity gate
    after = fingerprint()
    arm1, dmp1, frc1 = read_knee_dyn(bench)
    dyn_restored = (arm1 == arm0 and dmp1 == dmp0 and frc1 == frc0)
    hashes_ok = (before == after)
    limits_ok = bench.limits_unchanged()

    print("\n" + bar)
    print("INTEGRITY")
    print(bar)
    for rel in PROTECTED:
        if rel in before:
            same = before[rel] == after.get(rel)
            print(f"      {'unchanged' if same else 'CHANGED  '}  {rel}")
    print(f"      compiled dof restored to ({arm0:g}, {dmp0:g}, {frc0:g}): "
          f"{'YES' if dyn_restored else 'NO'}")
    print(f"      forcerange / ctrlrange unchanged: {'YES' if limits_ok else 'NO'}")
    print()
    print(f"XML MODIFIED? {'NO' if hashes_ok else 'YES -- INVESTIGATE'}")
    print(f"ORACLE BASELINE PRESERVED? "
          f"{'YES' if (hashes_ok and dyn_restored and limits_ok and ctrl_ok) else 'NO'}")
    print()
    print("REMINDER: this is the RIGID-BELT approximation only.  Belt compliance, "
          "current-dependent")
    print("friction, backlash and the actuator-side state are all still MISSING.  "
          "Nothing here has")
    print("been validated against hardware.  Stage 2 is NOT started.")
    print(f"\nwrote {os.path.relpath(path, ROOT)}")
    return 0 if (hashes_ok and dyn_restored and limits_ok) else 1


if __name__ == "__main__":
    raise SystemExit(main())
