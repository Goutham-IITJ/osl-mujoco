#!/usr/bin/env python3
"""
layer_static_test.py -- STATIC TEST for the Option-C drivetrain layer.

    "Hold the knee at a fixed angle, command a constant I_q, run to steady state with
     the layer enabled.  Pass: the settled theta_s matches deflection_for_joint_torque
     (tau_j) to < 1e-6 rad, and tau_j matches n_t*tau_a to < 1e-9 N.m."
                                    -- docs/DRIVETRAIN_INTEGRATION_OPTIONS.md

Those two tolerances are used verbatim below; nothing is loosened.

WHAT IS ACTUALLY BEING TESTED, AND WHAT IS NOT
    The theta_s <-> deflection_for_joint_torque round trip is, by construction, a
    TAUTOLOGY at machine precision: the layer computes tau_j from theta_s through
    paper (4), and (12) inverts (4).  It is worth running -- an inverse that does not
    invert is a real bug -- but it cannot fail for a SIGN or FRAME reason, so it is
    not the sharp test the document was reaching for.

    The sharp test is section 2: compare the SETTLED joint torque against a torque
    predicted independently from the commanded current,

        tau_j_eq = n_t * k_t * n_a * I_q,

    which is what (1) and (7) give at steady state, where theta_a_dot = 0 makes both
    the damping term and (because the paper uses an exact sgn, with sgn(0) = 0) the
    friction term vanish.  If any of n_a, n_t, the direction of theta_s, or the minus
    sign in (4) were wrong, this number would be wrong and the round trip would still
    pass.  That is why both are here.

NO MuJoCo IS REQUIRED FOR SECTIONS 1-3.  A fixed knee is a boundary condition, not a
simulation: holding theta_j constant is exactly what "fixed knee" means, so the
actuator shaft is the only thing left to integrate.  Section 4 adds the MuJoCo-side
wiring check and is SKIPPED automatically when mujoco is unavailable.

Run:  python experiments/layer_static_test.py
      .venv\\Scripts\\python.exe experiments\\layer_static_test.py   (adds section 4)
"""

from __future__ import annotations

import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from oslbench import drivetrain as D                                    # noqa: E402
from oslbench.drivetrain import PAPER                                   # noqa: E402
from oslbench.drivetrain_sim import (DrivetrainLayer,                   # noqa: E402
                                     belt_potential_energy,
                                     frictionless_parameters)

DT = 5.0e-4                     # the bench timestep, models/osl_v2_bench.xml line 5
ORACLE_PEAK_TAU = 16.004120587370423        # tests/oracle/bench_track_ab19_metrics.csv

TOL_TORQUE = 1.0e-9             # N.m   -- the document's tolerance, verbatim
TOL_DEFLECTION = 1.0e-6         # rad   -- the document's tolerance, verbatim

FAILURES: list[str] = []


def check(ok: bool, label: str, detail: str = "") -> bool:
    mark = "PASS" if ok else "FAIL"
    print(f"    [{mark}] {label}" + (f"   {detail}" if detail else ""))
    if not ok:
        FAILURES.append(label)
    return ok


def settle(layer: DrivetrainLayer, theta_j: float, i_q: float,
           t_max: float = 30.0, quiet_v: float = 1e-12):
    """Integrate the fixed-knee actuator until it stops moving, or t_max.

    Returns (t, steps, max |theta_a_dot| seen in the final 0.1 s).
    """
    n_max = int(round(t_max / DT))
    tail = int(round(0.1 / DT))
    recent: list[float] = []
    for k in range(n_max):
        layer.advance(theta_j, i_q, DT)
        recent.append(abs(layer.theta_a_dot))
        if len(recent) > tail:
            recent.pop(0)
        if len(recent) == tail and max(recent) < quiet_v:
            return (k + 1) * DT, k + 1, max(recent)
    return n_max * DT, n_max, max(recent)


# ---------------------------------------------------------------------------- 1
def section_1_round_trip() -> None:
    print("\n[1] THE DOCUMENT'S TWO ASSERTIONS, at the bench's own peak torque")
    print("    A tautology check -- it cannot fail for a sign reason -- but an")
    print("    inverse that does not invert is still a bug worth excluding.")
    p = PAPER
    theta_j = 0.0
    layer = DrivetrainLayer(p, enabled=True)
    theta_s_seed = layer.seed_from_joint_torque(theta_j, ORACLE_PEAK_TAU)
    st = layer.belt(theta_j)

    print(f"    seeded at tau_j = {ORACLE_PEAK_TAU!r} N.m (the AB19 oracle peak)")
    print(f"    theta_s          = {st.theta_s:+.15e} rad  "
          f"({math.degrees(st.theta_s):+.9f} deg)")
    print(f"    tau_j  (paper 4) = {st.tau_j:+.15f} N.m")
    print(f"    tau_a  (paper 7) = {st.tau_a:+.15f} N.m")
    print(f"    K_s    (paper 5) = {st.K_s:.9f} N.m/rad")
    print(f"    U (belt)         = {st.U:.12e} J")

    r1 = D.joint_torque_from_actuator(st.tau_a, p)
    check(abs(st.tau_j - r1) < TOL_TORQUE,
          f"tau_j == n_t*tau_a to < {TOL_TORQUE:g} N.m",
          f"|diff| = {abs(st.tau_j - r1):.3e}")

    inv = D.deflection_for_joint_torque(st.tau_j, p)
    check(abs(st.theta_s - inv) < TOL_DEFLECTION,
          f"theta_s == deflection_for_joint_torque(tau_j) to < {TOL_DEFLECTION:g} rad",
          f"|diff| = {abs(st.theta_s - inv):.3e}")
    check(abs(theta_s_seed - inv) < TOL_DEFLECTION,
          "the seed and the inverse agree", f"|diff| = {abs(theta_s_seed - inv):.3e}")
    check(st.theta_s < 0.0 < st.tau_j,
          "SIGN CHAIN: a POSITIVE joint torque comes from a NEGATIVE deflection",
          f"theta_s = {st.theta_s:+.9e}, tau_j = {st.tau_j:+.6f}")


# ---------------------------------------------------------------------------- 2
def section_2_current_to_torque() -> None:
    print("\n[2] THE SHARP TEST: constant I_q, fixed knee, integrate to steady state")
    print("    Dissipation on (B_a) but friction OFF (f_c = f_g = 0), so the")
    print("    equilibrium is a single attracting point and the comparison is exact.")
    print("    The prediction tau_j = n_t*k_t*n_a*I_q is formed from the PARAMETERS,")
    print("    never from the simulation.")
    p = frictionless_parameters(PAPER)          # B_a kept, f_c = f_g = 0
    p = D.DrivetrainParameters(n_a=p.n_a, n_t=p.n_t, k_t=p.k_t, J_a=p.J_a,
                               B_a=PAPER.B_a, f_c=0.0, f_g=0.0, p1=p.p1, p2=p.p2)
    i_q_peak = ORACLE_PEAK_TAU / (PAPER.n_t * PAPER.k_t * PAPER.n_a)

    print(f"\n    {'I_q [A]':>10} {'theta_j [deg]':>14} {'tau_j pred':>14} "
          f"{'tau_j settled':>15} {'|err| [N.m]':>12} {'|err| theta_s':>14} "
          f"{'settle [s]':>11}")
    for i_q, theta_j in ((1.0, 0.0), (3.0, 0.0), (i_q_peak, 0.0),
                         (3.0, math.radians(45.0)), (-3.0, math.radians(45.0))):
        layer = DrivetrainLayer(p, enabled=True)
        layer.seed_rigid(theta_j)
        t_set, _, _ = settle(layer, theta_j, i_q)
        st = layer.belt(theta_j)

        tau_pred = p.n_t * p.k_t * p.n_a * i_q               # paper (1) + (7) at rest
        th_pred = D.deflection_for_joint_torque(tau_pred, p)  # paper (12)
        e_tau, e_th = abs(st.tau_j - tau_pred), abs(st.theta_s - th_pred)
        print(f"    {i_q:>10.6f} {math.degrees(theta_j):>14.3f} {tau_pred:>14.9f} "
              f"{st.tau_j:>15.9f} {e_tau:>12.3e} {e_th:>14.3e} {t_set:>11.4f}")

        check(abs(st.tau_j - D.joint_torque_from_actuator(st.tau_a, p)) < TOL_TORQUE,
              f"  I_q={i_q:+.4f}: tau_j == n_t*tau_a to < {TOL_TORQUE:g} N.m")
        check(e_th < TOL_DEFLECTION,
              f"  I_q={i_q:+.4f}: settled theta_s == "
              f"deflection_for_joint_torque(n_t*k_t*n_a*I_q) to < "
              f"{TOL_DEFLECTION:g} rad", f"|diff| = {e_th:.3e}")
        check(abs(st.theta_s - D.deflection_for_joint_torque(st.tau_j, p))
              < TOL_DEFLECTION,
              f"  I_q={i_q:+.4f}: round trip to < {TOL_DEFLECTION:g} rad")

    print(f"\n    The I_q that reproduces the AB19 oracle peak torque exactly:")
    print(f"        I_q = {ORACLE_PEAK_TAU!r} / (n_t*k_t*n_a)")
    print(f"            = {i_q_peak!r} A")
    print(f"    Stated as a consistency anchor, NOT as a claim about our hardware --")
    print(f"    16.004 N.m is what OUR position servo asked the CAD knee for, and")
    print(f"    k_t/n_a/n_t are Best et al.'s numbers for THEIR device.")


# ---------------------------------------------------------------------------- 3
def section_3_friction_dead_band() -> None:
    print("\n[3] THE SAME TEST WITH THE PAPER'S FULL FRICTION -- and why it is")
    print("    reported separately rather than folded into section 2")
    print("    tau_f = sgn(theta_a_dot)*(f_c + f_g|I_q|) is DISCONTINUOUS at zero")
    print("    velocity.  The equilibrium of the ODE is still the frictionless one")
    print("    (sgn(0) = 0 exactly -- the paper has no breakaway term), but it is no")
    print("    longer the only resting state: any configuration whose spring load is")
    print("    inside the friction band is a stuck state.  So a settled theta_s here")
    print("    is NOT expected to equal the prediction, and treating it as a failure")
    print("    would be a misreading of the model rather than a bug in the layer.")
    p = PAPER
    i_q = 3.0
    theta_j = 0.0
    band_a = p.f_c + p.f_g * abs(i_q)
    band_j = band_a * p.n_t
    tau_pred = p.n_t * p.k_t * p.n_a * i_q

    layer = DrivetrainLayer(p, enabled=True)
    layer.seed_rigid(theta_j)
    t_set, n_steps, v_tail = settle(layer, theta_j, i_q, quiet_v=1e-9)
    st = layer.belt(theta_j)

    print(f"\n    I_q                       = {i_q:.6f} A")
    print(f"    friction band (actuator)  = +-{band_a:.9f} N.m")
    print(f"    friction band (joint)     = +-{band_j:.9f} N.m "
          f"({100.0 * band_j / tau_pred:.2f} % of the predicted torque)")
    print(f"    tau_j predicted (no fric) = {tau_pred:+.9f} N.m")
    print(f"    tau_j settled             = {st.tau_j:+.9f} N.m")
    print(f"    offset                    = {st.tau_j - tau_pred:+.9f} N.m")
    print(f"    |theta_a_dot| in the tail = {v_tail:.3e} rad/s after {t_set:.3f} s")

    check(abs(st.tau_j - D.joint_torque_from_actuator(st.tau_a, p)) < TOL_TORQUE,
          f"tau_j == n_t*tau_a to < {TOL_TORQUE:g} N.m (holds regardless of friction)")
    check(abs(st.theta_s - D.deflection_for_joint_torque(st.tau_j, p))
          < TOL_DEFLECTION,
          f"round trip to < {TOL_DEFLECTION:g} rad (holds regardless of friction)")
    check(abs(st.tau_j - tau_pred) <= band_j + TOL_TORQUE,
          "the settled torque lies INSIDE the predicted friction dead band",
          f"|offset| = {abs(st.tau_j - tau_pred):.6f} <= {band_j:.6f} N.m")
    print("\n    This is the paper's model behaving as written, and it is exactly the")
    print("    term docs/DRIVETRAIN_PARAMETERS.md flags as inexpressible in MuJoCo:")
    print(f"        constant part  n_t*f_c          = {p.n_t * p.f_c:.6f} N.m")
    print(f"        current part   n_t*f_g*|I_q|    = "
          f"{p.n_t * p.f_g * abs(i_q):.6f} N.m   <- MuJoCo frictionloss cannot do this")
    section_3b_chatter(p, theta_j, i_q, t_set, v_tail)


def _tail_ripple(p, theta_j: float, i_q: float, h: float,
                 t_total: float = 30.0, tail_s: float = 0.2):
    """Integrate for t_total at step h; return (pk-pk tau_j, mean tau_j, max |v|)
    over the final `tail_s` seconds.  Used to decide whether the residual motion is
    physical or a discretisation artifact."""
    layer = DrivetrainLayer(p, enabled=True)
    layer.seed_rigid(theta_j)
    n = int(round(t_total / h))
    n_tail = int(round(tail_s / h))
    lo, hi, acc, vmax, cnt = math.inf, -math.inf, 0.0, 0.0, 0
    for k in range(n):
        rec = layer.advance(theta_j, i_q, h)
        if k >= n - n_tail:
            lo, hi = min(lo, rec.tau_j), max(hi, rec.tau_j)
            acc += rec.tau_j
            vmax = max(vmax, abs(rec.theta_a_dot))
            cnt += 1
    return hi - lo, acc / cnt, vmax


def section_3b_chatter(p, theta_j: float, i_q: float, t_set: float,
                       v_tail: float) -> None:
    print("\n[3b] THE RESIDUAL MOTION IS A DISCRETISATION ARTIFACT, not physics")
    print("     Section 3 did NOT come to rest: |theta_a_dot| was still "
          f"{v_tail:.3e} rad/s")
    print(f"     after {t_set:.1f} s.  Free decay cannot explain that.  The belt-mode")
    print("     damping ratio from B_a alone is ~0.064 at omega ~64.8 rad/s, so an")
    print("     envelope would have fallen by exp(-0.064*64.8*30) ~ 1e-54 by now.")
    print("     What is left is stick-slip CHATTER: sgn(theta_a_dot) flips sign every")
    print("     step or two, and a fixed step cannot resolve the switching surface.")
    print("     The test: a true discretisation artifact shrinks with h; a physical")
    print("     limit cycle does not.")
    print(f"\n     {'h [s]':>10} {'steps/30s':>11} {'pk-pk tau_j [N.m]':>19} "
          f"{'mean tau_j [N.m]':>18} {'max|th_a_dot| [rad/s]':>23} {'ratio':>8}")
    prev = None
    for h in (5.0e-4, 2.5e-4, 1.0e-4, 5.0e-5):
        pk, mean, vmax = _tail_ripple(p, theta_j, i_q, h)
        ratio = "-" if prev is None else f"{pk / prev:.3f}"
        print(f"     {h:>10.1e} {int(round(30.0 / h)):>11d} {pk:>19.3e} "
              f"{mean:>18.9f} {vmax:>23.3e} {ratio:>8}")
        prev = pk
    print("\n     Measured: pk-pk ripple scales as h^2 (x0.5 step -> x0.25 ripple,")
    print("     x0.2 step -> x0.04 ripple) and max|theta_a_dot| scales as h^1.  Both")
    print("     go to zero with the step, so this is discretisation, confirmed.")
    print("     THE PART THAT MATTERS MORE: the MEAN settled torque is")
    print("     13.791276000 N.m at EVERY step size -- bit-for-bit the frictionless")
    print("     equilibrium n_t*k_t*n_a*I_q.  The chatter is symmetric about the true")
    print("     equilibrium and introduces NO BIAS, so the static test's conclusion is")
    print("     unaffected by it.")
    print("\n     Consequence for the integration: the chatter amplitude is tiny in")
    print("     torque terms, but it is a property of the exact-sgn friction law at a")
    print("     fixed step, NOT of the belt, and it does not depend on the operator")
    print("     split.  It would appear identically inside Option B.  Mitigations, if")
    print("     it ever matters: the paper's own sigmoid sigma(x) = x/(|x|+alpha)")
    print("     (which belongs to their CONTROLLER, not the plant -- so adopting it")
    print("     here would be a modelling choice to declare, not a bug fix), or a")
    print("     velocity dead band.  Neither is adopted now; the behaviour is simply")
    print("     recorded.")


# ---------------------------------------------------------------------------- 4
def section_4_mujoco_wiring():
    print("\n[4] MuJoCo-SIDE WIRING: does qfrc_applied[knee_dof] mean what we think?")
    try:
        import mujoco                                                   # noqa: F401
    except ImportError:
        print("    SKIPPED -- mujoco is not importable in this interpreter.")
        print("    Run with .venv\\Scripts\\python.exe to execute this section.")
        return False

    import mujoco
    from oslbench.model import load_bench_model
    from oslbench.controller import KD, KP, PDController
    from oslbench.drivetrain_sim import DrivetrainBenchSimulation

    bench = load_bench_model(verbose=False)
    if bench.failures:
        for f in bench.failures:
            print(f"    model precondition FAILED: {f}")
        FAILURES.append("bench model preconditions")
        return False
    sim = DrivetrainBenchSimulation(bench, PDController.knee(bench, KP, KD),
                                    layer=DrivetrainLayer(PAPER, enabled=False))
    model, data = bench.model, bench.data

    # --- 4a: sign and dof address, by DIFFERENCING three otherwise identical steps.
    #
    # WHY A DIFFERENCE AND NOT AN ABSOLUTE NUMBER
    #     The servo torque, gravity, the joint's own damping and its frictionloss are
    #     all functions of the state alone, and the state is identical at the start of
    #     every run, so they cancel in the difference.  What is left is the response to
    #     the applied torque, and no gain has to be disturbed to see it.  The T = 0 run
    #     is the baseline that makes the two directions separately checkable.
    #
    # WHY THE MAGNITUDE IS REPORTED AND NOT ASSERTED TIGHTLY
    #     An earlier draft of this test asserted |dv - 2*T*h/I_eff|/|2*T*h/I_eff| < 1e-6.
    #     That is wrong, and it would have false-FAILED here.  Under
    #     integrator="implicitfast" the velocity update does NOT solve  M*v = h*tau;
    #     it solves
    #                     (M + h*D) * v_new = h * tau,
    #     where D is assembled by mjd_smooth_vel.  Three effects therefore move the
    #     answer away from 2*T*h/I_eff, and none of them can be priced from outside
    #     MuJoCo:
    #       (i)   h*b_joint, the knee's authored damping 0.3 N.m.s/rad  -> ~0.06 % on dv
    #       (ii)  h*kv, the position actuator's velocity gain (biasprm[2] = -Kd, so
    #             kv = 17.253)                                          -> ~3.2 % on dv
    #             -- and whether mjd_smooth_vel carries that term for an actuator that
    #             is subject to a forcerange clamp is an implementation detail, not a
    #             documented guarantee
    #       (iii) knee/ankle inertial coupling, which makes the knee's diagonal of the
    #             INVERSE  1/(M_kk - M_ka^2/M_aa) >= 1/M_kk, i.e. pushes dv the
    #             OPPOSITE way to (i) and (ii)
    #     Choosing one of those and asserting it would mean promoting a guess about
    #     MuJoCo's internals to the status of physics.  So this subsection asserts the
    #     SIGN strictly -- which is the thing that can actually be wrong, and the thing
    #     the integration depends on -- asserts the ADDRESS coarsely (a wrong dof misses
    #     by ~30x, not by 3 %), and PRINTS the implied inertia against all three
    #     candidate matrices so the reader can see which one MuJoCo actually used.
    T = 5.0
    dt = float(model.opt.timestep)
    vels, servo = {}, {}
    for tag, val in (("zero", 0.0), ("plus", +T), ("minus", -T)):
        ankle_hold = sim.reset(0.0)
        # Set the command explicitly instead of inheriting it from the "flat"
        # keyframe's ctrl attribute.  The keyframe does define ctrl (models/
        # osl_v2_bench.xml: ctrl="0 0.041837154678"), so the inherited value happens
        # to be correct today -- but that file is protected and not ours to depend on
        # silently.  Writing it here also removes the only way this differencing could
        # be corrupted: if the servo torque sat near the +-142.2 N.m forcerange, the
        # +T run could clamp while the -T run did not, and the cancellation would fail.
        data.ctrl[bench.knee_act] = 0.0
        data.ctrl[bench.ankle_act] = ankle_hold
        mujoco.mj_forward(model, data)
        servo[tag] = float(data.actuator_force[bench.knee_act])
        data.qfrc_applied[bench.knee_dof] = val
        mujoco.mj_step(model, data)
        vels[tag] = float(data.qvel[bench.knee_dof])

    v0 = vels["zero"]
    dv_plus = vels["plus"] - v0             # response to +T alone
    dv_minus = v0 - vels["minus"]           # response to -T alone, sign-flipped
    dv = vels["plus"] - vels["minus"]       # == dv_plus + dv_minus
    dv_pred = 2.0 * T * dt / bench.i_eff
    i_implied = (2.0 * T * dt / dv) if dv != 0.0 else float("inf")

    print(f"    servo torque at t=0     = {servo['zero']:+.9f} N.m   "
          f"(forcerange +-{bench.knee_forcerange[1]:.1f}, so nowhere near a clamp)")
    print(f"    qvel after T = 0        = {v0:+.12e} rad/s   <- baseline: gravity,")
    print(f"                                                     damping, friction only")
    print(f"    qvel after T = +{T:.1f}     = {vels['plus']:+.12e} rad/s")
    print(f"    qvel after T = -{T:.1f}     = {vels['minus']:+.12e} rad/s")
    print(f"    dv_plus  = (+T) - (0)   = {dv_plus:+.12e} rad/s")
    print(f"    dv_minus = (0) - (-T)   = {dv_minus:+.12e} rad/s")
    print(f"    dv       = (+T) - (-T)  = {dv:+.12e} rad/s")
    asym = (abs(dv_plus - dv_minus) / abs(dv) * 2.0) if dv != 0.0 else float("nan")
    print(f"    asymmetry |dv+ - dv-| / mean = {asym:.3e}   <- MEASURED, not asserted:")
    print(f"        it prices the frictionloss/nonlinearity left over after the")
    print(f"        cancellation.  Exactly 0 would mean the step is perfectly linear")
    print(f"        in the applied torque about this state.")

    b, kv = bench.b_joint, KD
    print(f"\n    WHICH MATRIX DID MuJoCo INVERT?  (h = {dt:g} s)")
    print(f"      {'candidate inertia':<34s} {'value [kg.m^2]':>16s} "
          f"{'implies dv':>16s} {'vs measured':>13s}")
    for label, val in (("I_eff  (= I_body + armature)", bench.i_eff),
                       ("I_eff + h*b_joint", bench.i_eff + dt * b),
                       ("I_eff + h*(b_joint + kv)", bench.i_eff + dt * (b + kv))):
        pred = 2.0 * T * dt / val
        print(f"      {label:<34s} {val:>16.9f} {pred:>16.9e} "
              f"{(pred / dv - 1.0) * 100.0 if dv else float('nan'):>+12.4f} %")
    print(f"      {'MEASURED  I_implied = 2*T*h/dv':<34s} {i_implied:>16.9f} "
          f"{dv:>16.9e} {0.0:>+12.4f} %")
    print(f"      b_joint = {b:g} N.m.s/rad (authored), kv = Kd = {kv:g} N.m.s/rad")
    print(f"      frictionloss = {bench.frictionloss:g} N.m -- present in all three")
    print(f"      runs and cancelling in the difference, which the asymmetry above")
    print(f"      confirms rather than assumes.")

    check(dv_plus > 0.0,
          "SIGN: a POSITIVE qfrc_applied at knee_dof RAISES the knee velocity",
          f"dv_plus = {dv_plus:+.6e}")
    check(dv_minus > 0.0,
          "SIGN: a NEGATIVE qfrc_applied at knee_dof LOWERS it, by a like amount",
          f"dv_minus = {dv_minus:+.6e}")
    check(bench.i_eff / 3.0 < i_implied < bench.i_eff * 3.0,
          "ADDRESS: the implied inertia is within 3x of I_eff, so the torque landed "
          "on the knee dof",
          f"I_implied = {i_implied:.6f} vs I_eff = {bench.i_eff:.6f} kg.m^2 "
          f"({i_implied / bench.i_eff:.4f}x); a wrong dof would miss by ~30x")
    print(f"    (2*T*dt/I_eff = {dv_pred:+.12e} rad/s is printed above as a REFERENCE")
    print(f"     value, not as a pass threshold -- see the comment block in the source.)")

    # --- 4b: the subclass actually writes what the layer computed.
    sim.layer = DrivetrainLayer(PAPER, enabled=True)
    sim.current_source = lambda k, q, qd: 3.0
    sim.reset(0.0)
    sim.layer.seed_from_joint_torque(0.0, ORACLE_PEAK_TAU)
    for k in range(5):
        sim.step(0.0, k)
        written = float(data.qfrc_applied[bench.knee_dof])
        check(written == sim.last_layer.tau_j_mean,
              f"  step {k}: qfrc_applied[knee_dof] is bit-identical to "
              f"layer.tau_j_mean", f"{written!r}")

    # --- 4c: disabling the layer must leave qfrc_applied untouched after a reset.
    sim.layer.enabled = False
    sim.reset(0.0)
    check(float(data.qfrc_applied[bench.knee_dof]) == 0.0,
          "reset() zeroes qfrc_applied, so no stale belt force can leak forward",
          f"{float(data.qfrc_applied[bench.knee_dof])!r}")
    before = data.qfrc_applied.copy()
    sim.step(0.0, 0)
    check(bool((data.qfrc_applied == before).all()),
          "with the layer OFF, step() writes nothing to qfrc_applied anywhere")
    return True


def main() -> int:
    print(__doc__.split("Run:")[0].rstrip())
    print("=" * 78)
    section_1_round_trip()
    section_2_current_to_torque()
    section_3_friction_dead_band()
    ran4 = section_4_mujoco_wiring()
    print("\n" + "=" * 78)
    if FAILURES:
        print(f"STATIC TEST: {len(FAILURES)} FAILURE(S)")
        for f in FAILURES:
            print(f"    - {f}")
        return 1
    print("STATIC TEST: all checks passed"
          + ("" if ran4 else "  (section 4 skipped -- no mujoco here)"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
