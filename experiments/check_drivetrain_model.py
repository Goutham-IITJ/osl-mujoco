#!/usr/bin/env python3
"""
check_drivetrain_model.py -- numerical sanity checks for oslbench/drivetrain.py.

WHAT THIS SCRIPT IS
    A printed walk through the paper's drivetrain equations at labelled operating
    points, plus every check the paper makes verifiable, plus the three-way comparison
    between what our MuJoCo bench has TODAY, the rigid equivalent of the paper's
    actuator, and the paper's actual drivetrain.

WHAT THIS SCRIPT IS NOT
    Not a measurement.  We have never put an OSL V2 actuator on a dynamometer.  Every
    parameter is Best et al.'s, used on their authority; see docs/DRIVETRAIN_PARAMETERS.md.
    Not a simulation either -- it evaluates algebra, it does not integrate anything.
    It does not import MuJoCo and it does not touch models/osl_v2_bench.xml except to
    READ the three placeholder attributes for the comparison table.

    Source:  T. K. Best et al., IEEE/ASME T-MECH 30(6):4732-4743, Dec 2025,
             DOI 10.1109/TMECH.2024.3508469, Sec. II-III, eqs (1)-(12), Fig. 3.

RUN IT
    python experiments\\check_drivetrain_model.py        (stdlib only -- no MuJoCo needed)
"""

from __future__ import annotations

import importlib.util
import math
import os
import sys
import xml.etree.ElementTree as ET

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BENCH_XML = os.path.join(ROOT, "models", "osl_v2_bench.xml")


def _load_drivetrain():
    """Load the module without executing oslbench/__init__.py (which imports numpy).
    sys.modules must be populated BEFORE exec_module -- @dataclass looks itself up there."""
    path = os.path.join(ROOT, "oslbench", "drivetrain.py")
    spec = importlib.util.spec_from_file_location("oslbench_drivetrain_standalone", path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


D = _load_drivetrain()
P = D.PAPER

# --------------------------------------------------------------------------------
# OPERATING POINTS.  Every one is either a paper value or a clearly-labelled derived
# test value.  NONE of them is a measurement of our hardware.
# --------------------------------------------------------------------------------
BENCH_PEAK_TAU = 16.004120587370423   # OUR MuJoCo bench's peak knee torque, from
#                                       tests/oracle/bench_track_ab19_metrics.csv.
#                                       A simulation output, used here only as a
#                                       realistic place to evaluate the paper's model.
FIG3_MAX_DEFL = 0.055                 # PAPER: right edge of Fig. 3's abscissa
PAPER_PEAK_TAU = 160.0                # PAPER, Sec. II: "160 Nm for 10 seconds"
TEST_JOINT_VEL = 2.0                  # ASSUMED test value [rad/s] -- a plausible knee
#                                       rate in swing; nothing depends on the exact number
I_BODY = 0.251998                     # CAD-DERIVED, from oslbench/model.py:I_BODY_EXPECT
#                                       (knee-distal inertia about the knee axis)

PASS: list[str] = []
FAIL: list[str] = []


def check(name: str, got: float, expect: float, tol: float, units: str = "") -> None:
    ok = abs(got - expect) <= tol * max(1.0, abs(expect))
    (PASS if ok else FAIL).append(name)
    print(f"  [{'PASS' if ok else 'FAIL'}]  {name:<52s} {got:16.9f} vs {expect:14.9f} "
          f"{units}")


def rule(title: str) -> None:
    print()
    print("=" * 86)
    print(title)
    print("=" * 86)


# ==================================================================================
def section_parameters() -> None:
    """Print the paper's parameters with their frames.

    WHY THE TOTAL RATIO IS PRINTED WITH A WARNING ATTACHED
        n_a*n_t is the mechanical ratio of the device Best et al. describe.  It is
        NOT 49.4 and NOT 58.4.  Those two numbers are the gear ratios hard-coded in
        the MyoAssist KA_L1 model in the other repository, they were never measured
        on any hardware of ours, and the physical OSL V2 total ratio must not be
        labelled 49.4.  The printed line below carries the two numbers so the
        distinction is visible at the console; the attribution lives here because
        tests/test_oslbench.py forbids naming out-of-scope projects outside a
        docstring, and routing around that guard would defeat its purpose.
    """
    rule("1.  PARAMETERS AND THEIR FRAMES  (paper Sec. II and III-A1)")
    print("  Every value below is MEASURED-BY-THE-PAPER or PAPER-DERIVED.  None is ours.\n")
    rows = (
        ("n_a", P.n_a, "-", "rotor -> actuator output", "planetary, 9:1 catalogue"),
        ("n_t", P.n_t, "-", "actuator output -> joint", "belt, 4.61:1"),
        ("k_t", P.k_t, "N.m/A", "ROTOR", "torque constant"),
        ("J_a", P.J_a, "kg.m^2", "ACTUATOR OUTPUT", "rotor+gearbox, lumped"),
        ("B_a", P.B_a, "N.m.s/rad", "ACTUATOR OUTPUT", "viscous"),
        ("f_c", P.f_c, "N.m", "ACTUATOR OUTPUT", "Coulomb"),
        ("f_g", P.f_g, "N.m/A", "amps -> actuator N.m", "current-dependent friction"),
        ("p1", P.p1, "N.m/rad", "JOINT", "belt, linear term"),
        ("p2", P.p2, "N.m/rad^2", "JOINT", "belt, quadratic term"),
    )
    print(f"  {'sym':<5s} {'value':>14s}  {'units':<11s} {'frame / side':<26s} note")
    for sym, val, units, frame, note in rows:
        print(f"  {sym:<5s} {val:14.6g}  {units:<11s} {frame:<26s} {note}")

    print()
    complaints = P.validate()
    print(f"  validate()                     {complaints if complaints else 'clean'}")
    print(f"  total mechanical ratio n_a*n_t {P.total_ratio:.4f}      "
          f"<- NOT 49.4 and NOT 58.4 (see this function's docstring)")
    print(f"  k_t*n_a      (actuator output) {P.k_t_actuator:.6f} N.m/A")
    print(f"  k_t*n_a*n_t  (joint, RIGID)    {P.k_t_joint:.6f} N.m/A")
    print(f"  J_a/n_a^2    (implied rotor)   {P.rotor_inertia_implied:.6e} kg.m^2   "
          f"<- DERIVED; the paper reports only the lumped J_a")
    print(f"  CAD belt ratio 50/11           {D.CAD_BELT_RATIO:.6f}      "
          f"({100*(D.CAD_BELT_RATIO/P.n_t - 1):+.2f}% vs the paper's 4.61; NOT substituted)")


# ==================================================================================
def section_paper_checks() -> None:
    rule("2.  CHECKS AGAINST VALUES THE PAPER STATES EXPLICITLY")
    print("  Only quantities the paper pins down are checked here.  Identities that must")
    print("  hold for the equations as printed are checked too.\n")

    check("K_s(0) = p1 = 876 N.m/rad  (eq 5 at zero)",
          D.belt_stiffness(0.0, P), 876.0, 0.0, "N.m/rad")
    check("K_s at zero load via sqrt(p1^2+4p2|tau|)",
          D.belt_stiffness_at_torque(0.0, P), 876.0, 1e-12, "N.m/rad")
    check("rho(0.055) vs Fig. 3's right edge (~93 N.m)",
          D.rho(FIG3_MAX_DEFL, P), 93.291825, 1e-9, "N.m")
    check("eq (6) -> (1)(2) round trip, tau_a = +5, w = +3",
          D.motor_torque(D.current_for_actuator_torque(5.0, 3.0, P), P)
          - D.friction_torque(3.0, D.current_for_actuator_torque(5.0, 3.0, P), P),
          5.0, 1e-12, "N.m")
    check("eq (6) -> (1)(2) round trip, tau_a = -5, w = +3",
          D.motor_torque(D.current_for_actuator_torque(-5.0, 3.0, P), P)
          - D.friction_torque(3.0, D.current_for_actuator_torque(-5.0, 3.0, P), P),
          -5.0, 1e-12, "N.m")
    check("eq (12) -> (4) round trip at the bench peak",
          D.joint_torque_from_deflection(
              D.deflection_for_joint_torque(BENCH_PEAK_TAU, P), P),
          BENCH_PEAK_TAU, 1e-12, "N.m")
    check("K_s: (5)o(12) vs the closed form, at the bench peak",
          D.belt_stiffness(D.deflection_for_joint_torque(BENCH_PEAK_TAU, P), P),
          D.belt_stiffness_at_torque(BENCH_PEAK_TAU, P), 1e-12, "N.m/rad")
    check("eq (7) tau_j = n_t*tau_a, quasi-static",
          D.joint_torque_linearised(3.471609672, 0.0, 0.0, P),
          P.n_t * 3.471609672, 1e-12, "N.m")
    check("Appendix A1 positivity: k_t*n_a - f_g > 0",
          P.k_t_actuator - P.f_g, 0.9151, 1e-4, "N.m/A")
    check("paper's 160 N.m peak needs a sane current",
          PAPER_PEAK_TAU / P.k_t_joint, 34.804611, 1e-6, "A (rigid limit)")

    print()
    print("  NOT reproduced, deliberately:")
    print("    * the paper's \"up to 20%\" steady-state torque/impedance error from")
    print("      ignoring belt stiffening.  Reproducing it needs their desired stiffness")
    print("      K_d, which the paper does not state for that claim.  Recorded as their")
    print("      claim, not verified here.")
    print("    * VAF 99.7%, RMS residual < 0.65 N.m, belt fit R^2 = 0.997.  These are fit")
    print("      qualities of THEIR regression against THEIR data; we have no data.")


# ==================================================================================
def section_operating_points() -> None:
    rule("3.  THE MODEL EVALUATED AT LABELLED OPERATING POINTS")
    print(f"  Joint velocity is held at an ASSUMED {TEST_JOINT_VEL:g} rad/s throughout, so")
    print(f"  theta_a_dot = n_t*theta_j_dot = {P.n_t*TEST_JOINT_VEL:.3f} rad/s (rigid")
    print("  kinematics used only to pick a velocity; the torques below do not assume it).")
    print("  Accelerations are zero: these are CONSTANT-VELOCITY (steady-state) balances,")
    print("  so I_q has to pay for damping and friction even at zero joint torque.\n")

    points = (
        (0.0, "no load"),
        (5.0, "light load          (derived test value)"),
        (BENCH_PEAK_TAU, "OUR bench's peak    (simulation output, not hardware)"),
        (50.0, "mid load            (derived test value)"),
        (D.rho(FIG3_MAX_DEFL, P), "Fig. 3 right edge   (edge of the fitted range)"),
        (PAPER_PEAK_TAU, "paper's 10 s peak   (EXTRAPOLATES the belt fit)"),
    )
    th_a_dot = P.n_t * TEST_JOINT_VEL

    print(f"  {'tau_j':>10s} {'tau_a':>9s} {'I_q':>8s} {'tau_f':>8s} {'theta_s':>10s} "
          f"{'theta_s':>8s} {'K_s':>9s} {'tau_belt':>9s}  label")
    print(f"  {'[N.m]':>10s} {'[N.m]':>9s} {'[A]':>8s} {'[N.m]':>8s} {'[rad]':>10s} "
          f"{'[deg]':>8s} {'[N.m/rad]':>9s} {'[N.m]':>9s}")
    print("  " + "-" * 84)
    for tau_j, label in points:
        tau_a = D.actuator_load_torque_from_joint(tau_j, P)
        i_q = D.current_for_actuator_torque(tau_a + P.B_a * th_a_dot, th_a_dot, P)
        tau_f = D.friction_torque(th_a_dot, i_q, P)
        th_s = D.deflection_for_joint_torque(tau_j, P)
        k_s = D.belt_stiffness(th_s, P)
        tau_s = D.belt_torque(th_s, P)
        flag = "  <-- beyond Fig. 3" if abs(th_s) > FIG3_MAX_DEFL else ""
        print(f"  {tau_j:10.4f} {tau_a:9.4f} {i_q:8.3f} {tau_f:8.4f} {th_s:10.6f} "
              f"{math.degrees(th_s):8.4f} {k_s:9.2f} {tau_s:9.4f}  {label}{flag}")

    print()
    print("  Reading the table:")
    print("    tau_a    = tau_j/n_t          the actuator output torque the joint demands")
    print("    I_q      from eq (6), including B_a*theta_a_dot so the balance is honest")
    print("    tau_f    eq (2); it GROWS with I_q, which is the whole point of f_g")
    print("    theta_s  eq (12); NEGATIVE for positive tau_j, by the paper's eq (4)")
    print("    tau_belt = -tau_j; the belt's own torque, sign-opposite by construction")

    print()
    print("  Friction split at our bench's peak torque, referred to the JOINT:")
    tau_a = D.actuator_load_torque_from_joint(BENCH_PEAK_TAU, P)
    i_q = D.current_for_actuator_torque(tau_a, 1.0, P)
    const = D.reflect_coulomb_friction_to_joint(P)
    full = D.joint_friction_full(1.0, i_q, P)
    print(f"    I_q = {i_q:.4f} A  (from tau_a = {tau_a:.4f} N.m, quasi-static)")
    print(f"    constant       n_t*f_c        = {const:7.4f} N.m   "
          f"<- MuJoCo frictionloss CAN represent this")
    print(f"    load-dependent n_t*f_g*|I_q|  = {full-const:7.4f} N.m   "
          f"<- it CANNOT represent this at all")
    print(f"    total                         = {full:7.4f} N.m   "
          f"({(full-const)/const:.2f}x more unrepresentable than representable)")

    print()
    print("  Belt stiffening over the load range:")
    for tau_j in (0.0, 5.0, BENCH_PEAK_TAU, 50.0, PAPER_PEAK_TAU):
        k_s = D.belt_stiffness_at_torque(tau_j, P)
        print(f"    tau_j = {tau_j:7.3f} N.m  ->  K_s = {k_s:8.2f} N.m/rad  "
              f"({k_s/P.p1:.3f}x the zero-load value)")
    print("    A linear spring would hold K_s constant.  It does not: that nonlinearity")
    print("    is the reason the paper needed (5) at all.")


# ==================================================================================
def read_xml_placeholders() -> dict:
    """READ-ONLY.  Pull the knee joint's three authored attributes out of the bench XML."""
    try:
        root = ET.parse(BENCH_XML).getroot()
    except Exception as exc:                                      # noqa: BLE001
        print(f"  (could not read {os.path.relpath(BENCH_XML, ROOT)}: {exc})")
        return {}
    for joint in root.iter("joint"):
        if joint.get("name") == "knee":
            return {k: float(joint.get(k, "nan"))
                    for k in ("armature", "damping", "frictionloss")}
    return {}


def section_three_way() -> None:
    rule("4.  THREE-WAY COMPARISON  --  what Stage 1's single number actually was")
    xml = read_xml_placeholders()
    arm = xml.get("armature", 0.01)
    dmp = xml.get("damping", 0.3)
    frc = xml.get("frictionloss", 0.4)
    j_rigid = D.reflect_actuator_inertia_to_joint(P)
    b_rigid = D.reflect_actuator_damping_to_joint(P)
    f_rigid = D.reflect_coulomb_friction_to_joint(P)
    tau_a = D.actuator_load_torque_from_joint(BENCH_PEAK_TAU, P)
    i_q = D.current_for_actuator_torque(tau_a, 1.0, P)

    print(f"  Column A reads models/osl_v2_bench.xml (knee joint) live.  Column B is the")
    print("  paper's actuator collapsed into ONE rigid joint-side approximation.  Column C")
    print("  is the paper's model, which does NOT collapse into one number -- that refusal")
    print("  is the finding.\n")

    print(f"  {'effect':<30s} {'A: CAD PLACEHOLDER':>22s} {'B: PAPER RIGID EQ':>21s}  C: PAPER DRIVETRAIN")
    print("  " + "-" * 84)
    print(f"  {'added joint inertia':<30s} {arm:16.6f} kg.m2 {j_rigid:15.6f} kg.m2  "
          f"J_a = {P.J_a:.5f} kg.m2 at the ACTUATOR OUTPUT")
    print(f"  {'viscous damping':<30s} {dmp:16.6f} N.m.s {b_rigid:15.6f} N.m.s  "
          f"B_a = {P.B_a:.5f} N.m.s at the ACTUATOR OUTPUT")
    print(f"  {'Coulomb friction':<30s} {frc:16.6f} N.m   {f_rigid:15.6f} N.m    "
          f"f_c = {P.f_c:.5f} N.m at the ACTUATOR OUTPUT")
    print(f"  {'current-dep. friction':<30s} {'none':>22s} {'none':>21s}  "
          f"f_g*|I_q| = {P.f_g*abs(i_q):.5f} N.m at {i_q:.2f} A")
    print(f"  {'transmission':<30s} {'rigid (implicit)':>22s} {'rigid':>21s}  "
          f"rho = p2*th^2 + p1*th, K_s {P.p1:.0f} -> "
          f"{D.belt_stiffness_at_torque(BENCH_PEAK_TAU, P):.0f} N.m/rad")
    print(f"  {'mechanical DOF at the knee':<30s} {1:>22d} {1:>21d}  2 (theta_a AND theta_j)")
    print(f"  {'torque input':<30s} {'position servo':>22s} {'position servo':>21s}  "
          f"q-axis current I_q")

    print()
    print("  Ratios, B relative to A (the Stage 1 'armature' story, itemised):")
    print(f"    inertia   {j_rigid/arm:6.2f}x     ({arm:.4f} -> {j_rigid:.6f} kg.m^2)")
    print(f"    damping   {b_rigid/dmp:6.2f}x     ({dmp:.4f} -> {b_rigid:.6f} N.m.s/rad)")
    print(f"    friction  {f_rigid/frc:6.2f}x     ({frc:.4f} -> {f_rigid:.6f} N.m)")
    i_now, i_paper = I_BODY + arm, I_BODY + j_rigid
    print(f"    TOTAL joint inertia including the CAD bodies "
          f"(I_body = {I_BODY:.6f} kg.m^2, CAD-DERIVED):")
    print(f"        now {i_now:.6f}  ->  paper-rigid {i_paper:.6f} kg.m^2   "
          f"({i_paper/i_now:.3f}x)")

    print()
    print("  WHY COLUMN C CANNOT BE WRITTEN AS ONE ARMATURE NUMBER.")
    print("    1. J_a and B_a sit on the FAR SIDE of a spring.  Reflecting them through")
    print("       n_t^2 presumes the belt is rigid, which is exactly what the paper")
    print(f"       measured it not to be (K_s runs {P.p1:.0f} -> "
          f"{D.belt_stiffness_at_torque(PAPER_PEAK_TAU, P):.0f} N.m/rad over the load range).")
    print("    2. The friction has a term proportional to |I_q|.  A constant frictionloss")
    print(f"       cannot express it, and at our bench's peak it is the LARGER part")
    print(f"       ({P.n_t*P.f_g*abs(i_q):.3f} vs {f_rigid:.3f} N.m at the joint).")
    print("    3. The belt adds a genuine second coordinate.  One DOF cannot carry the")
    print("       energy stored in theta_s; no choice of armature makes it appear.")
    print("    4. Column B is therefore a COMPARISON, not a target.  Matching it would")
    print("       make our bench a better rigid model, not a model of the paper's device.")


# ==================================================================================
def main() -> int:
    print(__doc__.strip().splitlines()[0])
    print(f"module: oslbench/drivetrain.py    python {sys.version.split()[0]}    "
          f"numpy loaded: {'numpy' in sys.modules}    mujoco loaded: "
          f"{'mujoco' in sys.modules}")

    section_parameters()
    section_paper_checks()
    section_operating_points()
    section_three_way()

    rule("SUMMARY")
    print(f"  explicit-value checks: {len(PASS)} passed, {len(FAIL)} failed")
    if FAIL:
        for name in FAIL:
            print(f"    FAILED: {name}")
    print()
    print("  What this run establishes:  the equations in oslbench/drivetrain.py reproduce")
    print("  Best et al.'s stated values and are internally consistent in sign, frame and")
    print("  algebra.  What it does NOT establish:  that the OSL V2 in our lab behaves")
    print("  this way.  That would need hardware we have not instrumented.")
    print()
    print("  Nothing was written.  models/osl_v2_bench.xml was opened read-only.")
    return 1 if FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
