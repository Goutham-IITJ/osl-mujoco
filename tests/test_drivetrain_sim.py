#!/usr/bin/env python3
"""
test_drivetrain_sim.py -- tests for oslbench/drivetrain_sim.py, the Option-C layer.

WHY THIS FILE EXISTS
    `tests/test_drivetrain.py` covers `oslbench/drivetrain.py` -- the paper's equations,
    deliberately free of numpy and MuJoCo.  Nothing covered `drivetrain_sim.py`, the
    module that wires those equations to MuJoCo.  That gap had a cost: the derivative
    term of the knee controller was fed JOINT velocity while its current acts on the
    ACTUATOR shaft, with the compliant belt in between, and nothing failed until a real
    MuJoCo run diverged at t = 0.3485 s.

    So the centre of gravity here is one question: WHICH SIDE OF THE BELT does the
    derivative feedback read?  The control law is

        tau_req = Kp*(theta_ref - theta_j) - Kd*(theta_a_dot / n_t)
                       ^ JOINT position          ^ ACTUATOR velocity

    and that asymmetry is load-bearing, not an oversight:

      - the POSITION term is joint-side because tracking the joint is the task;
      - the VELOCITY term must be actuator-side because the current it commands acts on
        the actuator.  Damping one inertia in proportion to another inertia's velocity,
        across a spring, is NON-COLLOCATED feedback.  In the belt mode the two inertias
        swing in antiphase, so the "damping" torque points along the actuator's motion
        and pumps the mode.

    Every test below is a guard against that reverting quietly.  `test_..._not_joint_side`
    and `test_joint_velocity_has_zero_influence_...` are the two that would fail first.

    MEASURED, for the record (real MuJoCo, AB19, Kp = 600, Kd = 17.253, h = 0.5 ms):
    joint-velocity feedback diverged at t = 0.3485 s with |tau_j| through 1463 N.m;
    actuator-velocity feedback completed the 1.205 s cycle with peak |tau_j| = 16.70 N.m.
    Evidence: docs/DRIVETRAIN_INSTABILITY_DIAGNOSIS.md.

WHAT THESE TESTS ARE NOT FOR
    They do not validate anything against hardware -- no OSL V2 actuator has ever been
    measured here.  They also do not re-test the paper's equations; that is
    test_drivetrain.py's job.  Stability itself is not asserted here either: a unit test
    is the wrong instrument for it.  What is asserted is the WIRING that the measured
    stability result depends on.

RUN IT
    python tests\\test_drivetrain_sim.py
    python tests\\test_drivetrain_sim.py -v
    python tests\\test_drivetrain_sim.py -k feedback

    Also collected by tests/run_tests.py, so the one command Goutham already runs covers
    these.  Needs numpy; falls back to tests/stub_mujoco.py when MuJoCo is absent, and
    none of these tests step an engine anyway.
"""

from __future__ import annotations

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for _p in (ROOT, HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)


class Skip(Exception):
    """Raised to skip a test that cannot run in this environment."""


try:
    import stub_mujoco
    stub_mujoco.install()           # returns REAL mujoco when importable
except Exception:                                               # noqa: BLE001
    pass

from oslbench.controller import PDController, kd_for_damping_ratio   # noqa: E402
from oslbench.drivetrain import PAPER                                # noqa: E402
from oslbench.drivetrain_sim import (DrivetrainLayer,                # noqa: E402
                                     PDCurrentSource)

# The authored knee ctrlrange, so `command()` never clips in these tests and the
# arithmetic assertions compare against the law as written.
CTRL = (math.radians(-5.0), math.radians(120.0))
FORCE_LIM = 142.2
KP = 600.0
KD = 17.253
N_T = PAPER.n_t
K_T_JOINT = PAPER.n_t * PAPER.k_t * PAPER.n_a

# A reference array long enough for any k used below; a constant so that changing k
# never changes the answer and the tests stay about the feedback, not the trajectory.
REF = [0.5] * 64


def _source(kp: float = KP, kd: float = KD):
    """A PDCurrentSource wired to its own fresh layer, as production wires it."""
    layer = DrivetrainLayer(PAPER, enabled=True, substeps=1)
    pd = PDController(kp, kd, CTRL, FORCE_LIM, "knee_test")
    return PDCurrentSource(pd, REF, layer, PAPER), layer


# =====================================================================================
# THE FEEDBACK SOURCE -- the tests this file exists for
# =====================================================================================
def test_derivative_feedback_is_actuator_side_not_joint_side():
    """THE regression guard.  Vary each velocity alone and see which one moves I_q.

    If someone swaps `self.layer.theta_a_dot / self.p.n_t` back to `theta_j_dot`, this
    is the first thing that fails, and it fails on both halves at once: the joint
    velocity starts mattering and the actuator velocity stops.
    """
    src, layer = _source()
    theta_j = 0.3

    # (a) joint velocity moves, actuator velocity pinned -> the current must NOT move.
    layer.theta_a_dot = 0.0
    base = src(0, theta_j, 0.0)
    for th_jd in (-9.0, -1.0, 0.25, 4.0, 11.0):
        got = src(0, theta_j, th_jd)
        assert got == base, (
            f"I_q changed when only theta_j_dot changed ({th_jd:+g} rad/s): "
            f"{got:.12g} != {base:.12g} A.  The derivative term is reading JOINT "
            f"velocity.  That is the non-collocated law that diverged at t = 0.3485 s "
            f"on real MuJoCo -- see docs/DRIVETRAIN_INSTABILITY_DIAGNOSIS.md.")

    # (b) actuator velocity moves, joint velocity pinned -> the current MUST move.
    seen = set()
    for th_ad in (-30.0, -4.0, 0.0, 4.0, 30.0):
        layer.theta_a_dot = th_ad
        seen.add(round(src(0, theta_j, 0.0), 12))
    assert len(seen) == 5, (
        "I_q did not respond to theta_a_dot, so the derivative term is not reading the "
        "actuator at all -- it is either dead or reading the joint.")


def test_joint_velocity_has_zero_influence_on_the_current():
    """A wider, blunter version of (a): no theta_j_dot anywhere may change the output.

    Kept separate because it sweeps the whole plausible range INCLUDING the magnitudes
    seen in a divergence (50 rad/s was the diagnostic's abort guard), and because a
    partial revert -- say a blend of the two velocities -- would slip past a two-point
    check but not past this one.
    """
    src, layer = _source()
    for theta_j in (-0.08, 0.0, 0.4, 1.3, 2.0):
        for th_ad in (-50.0, 0.0, 50.0):
            layer.theta_a_dot = th_ad
            ref = src(0, theta_j, 0.0)
            for th_jd in (-50.0, -12.5, -0.001, 0.001, 12.5, 50.0):
                got = src(0, theta_j, th_jd)
                assert got == ref, (
                    f"theta_j_dot={th_jd:+g} changed I_q by {got - ref:+.3e} A at "
                    f"theta_j={theta_j:+g}, theta_a_dot={th_ad:+g}.  The joint velocity "
                    f"must have NO influence on the commanded current.")


def test_derivative_term_is_exactly_minus_kd_theta_a_dot_over_n_t():
    """Strip the proportional term off and check the remainder against the formula.

    This pins the DIVISION BY n_t as well as the signal.  Feeding raw theta_a_dot would
    pass the two tests above while silently multiplying the effective gain by the
    transmission ratio n_t = 4.61, which would change the gain at the same time as the
    feedback point.

    n_t = 4.61 is a dimensionless RATIO and is NOT k_t_joint = 4.597092 N.m/A, the
    torque constant.  The two are nearly equal by coincidence and belong to different
    frames; dividing by the wrong one would be a 0.3 % error that no stability test
    would ever notice, which is why this asserts the formula and not just the sign.
    """
    assert abs(N_T - 4.61) < 1e-12, f"n_t is {N_T}, expected the ratio 4.61"
    assert abs(K_T_JOINT - 4.597092) < 1e-6, "k_t_joint is the N.m/A constant, not n_t"
    assert N_T != K_T_JOINT, "n_t and k_t_joint must not be the same object or value"
    src, layer = _source()
    theta_j, q_ref = 0.3, REF[0]
    for th_ad in (-30.0, -7.5, 0.0, 7.5, 30.0):
        layer.theta_a_dot = th_ad
        tau_req = src(0, theta_j, 123.456) * K_T_JOINT      # theta_j_dot is a decoy
        derivative = tau_req - KP * (q_ref - theta_j)
        want = -KD * th_ad / N_T
        assert abs(derivative - want) < 1e-9, (
            f"derivative term is {derivative:+.9f} N.m, expected {want:+.9f} N.m "
            f"(= -Kd*theta_a_dot/n_t at theta_a_dot={th_ad:+g}).  Difference "
            f"{derivative - want:+.3e}.  Check whether the division by n_t = {N_T:g} "
            f"is still there.")


def test_proportional_term_is_still_joint_side():
    """The fix must move the VELOCITY term only.  The position term stays on the joint.

    Moving the proportional term to theta_a/n_t as well would be a different controller:
    it would regulate the actuator's position and let the belt deflection sit as a
    permanent tracking offset.  Tracking the joint is the task, so this must not drift.
    """
    src, layer = _source()
    layer.theta_a_dot = 0.0
    q_ref = REF[0]
    for theta_j in (-0.05, 0.0, 0.2, 0.5, 1.1):
        tau_req = src(0, theta_j, 0.0) * K_T_JOINT
        want = KP * (q_ref - theta_j)
        assert abs(tau_req - want) < 1e-9, (
            f"at theta_a_dot = 0 the request should be pure Kp*(ref - theta_j) = "
            f"{want:+.9f} N.m, got {tau_req:+.9f} N.m")


def test_feedback_side_constant_agrees_with_the_arithmetic():
    """`FEEDBACK_SIDE` is what other code and the docs read.  A stale label is a trap.

    A comment can rot silently; this makes the label falsifiable.
    """
    assert PDCurrentSource.FEEDBACK_SIDE == "actuator", (
        f"FEEDBACK_SIDE is {PDCurrentSource.FEEDBACK_SIDE!r}; the collocated law this "
        f"module implements is 'actuator'")
    src, layer = _source()
    layer.theta_a_dot = 10.0
    moved_with_actuator = src(0, 0.3, 0.0) != src(0, 0.3, 0.0) * 0  # non-zero response
    layer.theta_a_dot = 0.0
    at_zero = src(0, 0.3, 0.0)
    layer.theta_a_dot = 10.0
    at_ten = src(0, 0.3, 0.0)
    assert moved_with_actuator and at_ten != at_zero, (
        "FEEDBACK_SIDE says 'actuator' but the arithmetic does not respond to "
        "theta_a_dot -- the declaration and the code disagree")


def test_last_qdot_used_reports_the_velocity_actually_fed_back():
    """Logs, the A/B experiment and the diagnostic all read `last_qdot_used`.

    If it reported the wrong signal, every energy and power number computed from it
    would be wrong in a way that looked self-consistent.
    """
    src, layer = _source()
    for th_ad in (-18.0, 0.0, 6.0):
        layer.theta_a_dot = th_ad
        src(0, 0.3, 99.0)
        assert abs(src.last_qdot_used - th_ad / N_T) < 1e-15, (
            f"last_qdot_used = {src.last_qdot_used:+.9g}, expected "
            f"{th_ad / N_T:+.9g} (theta_a_dot/n_t)")
        assert src.last_theta_j_dot == 99.0, (
            "last_theta_j_dot should still record the joint velocity that was offered "
            "and not used, so logs can show both sides of the belt")


def test_the_two_laws_really_do_differ_on_a_realistic_antiphase_state():
    """Guard against a test suite that passes because both laws agree everywhere.

    In the belt mode the inertias move in ANTIPHASE: theta_j_dot and theta_a_dot/n_t
    have opposite signs.  That is the state the instability lives in, so the two laws
    must be measurably different there -- otherwise the tests above prove nothing about
    the case that matters.
    """
    src, layer = _source()
    th_jd, th_ad = +2.0, -2.0 * N_T          # antiphase, equal joint-referred magnitude
    layer.theta_a_dot = th_ad
    collocated = src(0, 0.3, th_jd) * K_T_JOINT
    pd = PDController(KP, KD, CTRL, FORCE_LIM, "knee_ref")
    joint_side = pd.torque_unclamped(REF[0], 0.3, th_jd)     # the OLD law, for contrast
    assert abs(collocated - joint_side) > 1.0, (
        f"the collocated and joint-velocity laws differ by only "
        f"{abs(collocated - joint_side):.3e} N.m on an antiphase state; the regression "
        f"tests above would not detect a revert")
    # And the sign of the difference is the mechanism: on this state the old law's
    # derivative term pushes one way and the new one pushes the other.
    assert (collocated - KP * (REF[0] - 0.3)) > 0.0 > (joint_side - KP * (REF[0] - 0.3)), (
        "on an antiphase state the two derivative terms should have OPPOSITE signs -- "
        "that opposition is the whole reason one law pumps the belt mode and the other "
        "damps it")


def test_kd_term_does_non_positive_work_on_the_actuator_for_every_signal():
    """The algebraic property that makes the collocated law safe, checked numerically.

    The derivative term contributes -Kd*qdot_fb to the joint-referred request, i.e.
    -Kd*qdot_fb/n_t on the shaft.  With qdot_fb = theta_a_dot/n_t its power on the shaft
    is -Kd*theta_a_dot^2/n_t^2, which is <= 0 for EVERY theta_a_dot -- it can only
    remove energy.  With qdot_fb = theta_j_dot the power is -Kd*theta_j_dot*theta_a_dot/
    n_t, whose sign depends on the phase between two different bodies, and is POSITIVE
    whenever they are in antiphase.  That is the energy source, in one line.

    The derivative torque is recovered from the IMPLEMENTATION's own output (total
    request minus the proportional term) rather than recomputed as -Kd*qdot.  An earlier
    version of this test wrote the minus sign itself, which made it `-Kd*v^2 <= 0` --
    true by construction and blind to a sign flip in production.  Reading the torque
    back out is what makes this a test of the code instead of a test of the test.
    """
    src, layer = _source()
    q_ref, theta_j = REF[0], 0.3
    p_prop = KP * (q_ref - theta_j)         # the proportional term, to be subtracted off
    worst = -math.inf
    for th_ad in (-40.0, -9.0, -0.5, 0.0, 0.5, 9.0, 40.0):
        layer.theta_a_dot = th_ad
        tau_total = src(0, theta_j, -th_ad) * K_T_JOINT   # antiphase joint vel as decoy
        tau_derivative = tau_total - p_prop               # FROM the implementation
        p_shaft = tau_derivative * (th_ad / N_T)          # joint-referred power
        worst = max(worst, p_shaft)
        assert p_shaft <= 0.0, (
            f"the Kd term delivered {p_shaft:+.6g} W to the actuator at "
            f"theta_a_dot = {th_ad:+g} rad/s (derivative torque "
            f"{tau_derivative:+.4f} N.m).  A collocated damping term can never do "
            f"positive work; this one did, so it is either not collocated or its sign "
            f"is inverted.")
    assert worst == 0.0, (
        f"the least-dissipative case should be exactly 0 W (at theta_a_dot = 0), "
        f"got {worst:+.6g} W")


def test_feedback_is_linear_in_actuator_velocity_with_no_hidden_saturation():
    """No clamp, deadband or blend may sit on the fed-back velocity.

    A "protective" saturation on qdot_fb would leave every other test in this file
    passing -- none of them probes far enough out -- while quietly changing the physics
    at exactly the velocities a divergence reaches.  The diagnostic's own abort guard
    sits at 50 rad/s JOINT-side, i.e. 230 rad/s on the shaft, so that is the range the
    feedback has to stay linear over.

    Linearity is checked as a constant finite difference: with the reference and
    theta_j fixed, d(tau_req)/d(theta_a_dot) must be exactly -Kd/n_t everywhere.
    """
    src, layer = _source()
    theta_j = 0.3
    want_slope = -KD / N_T
    prev = None
    for th_ad in (-300.0, -230.0, -120.0, -50.0, -7.0, -1.0, 0.0,
                  1.0, 7.0, 50.0, 120.0, 230.0, 300.0):
        layer.theta_a_dot = th_ad
        tau = src(0, theta_j, 0.0) * K_T_JOINT
        if prev is not None:
            slope = (tau - prev[1]) / (th_ad - prev[0])
            assert abs(slope - want_slope) < 1e-9, (
                f"d(tau)/d(theta_a_dot) = {slope:+.9f} between theta_a_dot = "
                f"{prev[0]:+g} and {th_ad:+g}, expected {want_slope:+.9f}. The feedback "
                f"is not linear there -- look for a clamp, a deadband, or a blend with "
                f"theta_j_dot. Any of those changes the physics without changing a gain.")
        prev = (th_ad, tau)


# =====================================================================================
# WIRING -- the mistakes that would make the above pass while the run misbehaves
# =====================================================================================
def test_layer_is_required_and_the_old_three_argument_call_is_rejected():
    """The pre-fix call was `PDCurrentSource(pd, ref, PAPER)`.  It must not silently work.

    Rejecting it is the difference between a loud TypeError at construction and a run
    that quietly feeds back the wrong velocity for 2410 steps.
    """
    pd = PDController(KP, KD, CTRL, FORCE_LIM, "knee_test")
    try:
        PDCurrentSource(pd, REF, PAPER)                  # the OLD signature
    except TypeError as exc:
        assert "theta_a_dot" in str(exc), (
            f"the error should explain WHY a layer is needed; got: {exc}")
    else:
        raise AssertionError(
            "PDCurrentSource(pd, ref, PAPER) was accepted.  The old three-argument call "
            "must fail loudly, or a stale call site reverts the fix in silence.")
    try:
        PDCurrentSource(pd, REF)                         # no layer at all
    except TypeError:
        pass
    else:
        raise AssertionError("the layer argument must be required, not defaulted")


def test_current_conversion_is_the_paper_derived_constant():
    """I_q = tau_req / k_t_joint, and k_t_joint is n_t*k_t*n_a -- not a fitted number."""
    src, _ = _source()
    assert abs(src.k_t_joint - K_T_JOINT) < 1e-12
    assert abs(src.k_t_joint - 4.597092) < 1e-6, (
        f"k_t_joint = {src.k_t_joint:.6f}, expected 4.597092 N.m/A")


def test_gains_are_the_validated_ones_and_were_not_retuned():
    """The fix was supposed to change the feedback POINT, not the gains."""
    from oslbench import controller as C
    assert C.KP == 600.0, f"KP is {C.KP}, expected 600 -- the fix must not retune gains"
    assert C.KD == 17.253, f"KD is {C.KD}, expected 17.253"
    # And that KD is still the single-mass figure it was derived as.  This is a
    # PROVENANCE check, not an endorsement: the derivation has no belt in it, which is
    # exactly why Kd is the next thing to revisit.
    want = kd_for_damping_ratio(600.0, 0.261998, 0.3, 0.7)
    assert abs(C.KD - want) < 5e-4, (
        f"KD = {C.KD} no longer matches kd_for_damping_ratio(600, 0.261998, 0.3, 0.7) "
        f"= {want:.6f}; if Kd was retuned, say so in the docs")


def test_layer_velocity_is_pre_step_so_both_velocities_are_consistent_in_time():
    """After a rigid seed at rest both velocities are 0, so step 0 is unambiguous.

    `DrivetrainBenchSimulation.step` calls the current source BEFORE `layer.advance`, so
    `layer.theta_a_dot` is the end-of-previous-step value -- the same instant MuJoCo's
    `qvel` is read at.  If a future edit moved the current-source call after `advance`,
    the feedback would use a half-step-newer velocity than the position term.  This test
    pins the resting case; `test_step_calls_the_current_source_before_advancing_the_shaft`
    below pins the ORDERING itself, which is the part that can actually be edited wrong.
    """
    src, layer = _source()
    layer.seed_rigid(0.5, 0.0)
    assert layer.theta_a_dot == 0.0
    i_q = src(0, 0.5, 0.0)
    assert abs(i_q) < 1e-12, (
        f"at the seeded reference, at rest, the request should be 0; got {i_q:+.3e} A")
    assert src.last_qdot_used == 0.0


def test_step_calls_the_current_source_before_advancing_the_shaft():
    """THE ORDERING, asserted against the real `step()` rather than against a docstring.

    Every other test in this file exercises `PDCurrentSource` in isolation, so all of
    them pass regardless of WHEN the simulation calls it.  That is a real gap: moving the
    call to after `layer.advance` would feed back a velocity half a step newer than the
    position term -- a different controller, with no gain changed and no test failing.

    This drives one real `step()` with a recording stand-in for the layer and checks the
    interleaving directly: the source must be asked for a current while `theta_a_dot`
    still holds the PREVIOUS step's value, and `advance` must be called after that.

    Needs a compiled model, so it SKIPS when neither MuJoCo nor the stub can load one.
    """
    try:
        from oslbench.model import load_bench_model
        from oslbench.drivetrain_sim import DrivetrainBenchSimulation
        bench = load_bench_model(verbose=False)
        if bench.failures:
            raise Skip(f"model preflight failed: {bench.failures}")
    except Skip:
        raise
    except Exception as exc:                                    # noqa: BLE001
        raise Skip(f"no loadable bench model here: {type(exc).__name__}: {exc}")

    events = []
    layer = DrivetrainLayer(PAPER, enabled=True, substeps=1)
    real_advance = layer.advance
    sentinel = 12.345                      # a value no integrator would produce by itself

    def recording_advance(theta_j, i_q, dt):
        events.append(("advance", layer.theta_a_dot))
        return real_advance(theta_j, i_q, dt)

    layer.advance = recording_advance
    pd = PDController(KP, KD, CTRL, FORCE_LIM, "knee_order")
    src = PDCurrentSource(pd, REF, layer, PAPER)
    real_call = src.__call__

    def recording_call(k, theta_j, theta_j_dot):
        events.append(("source", layer.theta_a_dot))
        return real_call(k, theta_j, theta_j_dot)

    sim = DrivetrainBenchSimulation(bench, pd, layer=layer, current_source=recording_call,
                                    disconnect_servo=True)
    assert sim.layer is layer, "the simulation must hold the very layer we instrumented"
    sim.reset(REF[0], ref_vel0=0.0)
    layer.theta_a_dot = sentinel           # a value only the PRE-step read can see
    sim.step(REF[0], 0)

    kinds = [e[0] for e in events]
    assert kinds[:2] == ["source", "advance"], (
        f"step() called {kinds} -- the current source must be asked for a current "
        f"BEFORE layer.advance moves the shaft. Calling it afterwards feeds the "
        f"derivative term a velocity half a step newer than the position term.")
    assert events[0][1] == sentinel, (
        f"the source saw theta_a_dot = {events[0][1]}, not the pre-step {sentinel}. "
        f"Something advanced the shaft before the feedback was computed.")
    assert events[1][1] == sentinel, (
        "advance() should still see the pre-step velocity as its own starting point")


def test_the_current_source_and_the_simulation_share_one_layer_instance():
    """Two different DrivetrainLayer objects would damp a shaft nothing is driving.

    That failure mode is the loudest warning in `PDCurrentSource`'s docstring and it
    would look STABLE -- the feedback would read a shaft sitting at rest, so the
    derivative term would contribute nothing and the run would simply behave like
    Kd = 0. Stable, and meaningless. Worth an assertion rather than a comment.
    """
    try:
        from oslbench.model import load_bench_model
        from oslbench.drivetrain_sim import DrivetrainBenchSimulation
        bench = load_bench_model(verbose=False)
        if bench.failures:
            raise Skip(f"model preflight failed: {bench.failures}")
    except Skip:
        raise
    except Exception as exc:                                    # noqa: BLE001
        raise Skip(f"no loadable bench model here: {type(exc).__name__}: {exc}")

    layer = DrivetrainLayer(PAPER, enabled=True, substeps=1)
    pd = PDController(KP, KD, CTRL, FORCE_LIM, "knee_share")
    src = PDCurrentSource(pd, REF, layer, PAPER)
    sim = DrivetrainBenchSimulation(bench, pd, layer=layer, current_source=src,
                                    disconnect_servo=True)
    assert src.layer is sim.layer, (
        "the current source and the simulation hold DIFFERENT layer instances; the "
        "feedback would read a shaft that nothing is driving")
    # reset must not replace it either -- it may only re-seed it in place
    sim.reset(REF[0], ref_vel0=0.0)
    assert src.layer is sim.layer, "reset() replaced the layer instead of re-seeding it"


# =====================================================================================
# THE LIVE DEMO IS PRESENTATION ONLY -- it must never become a second implementation
# =====================================================================================
DEMO = os.path.join(ROOT, "experiments", "run_live_drivetrain_demo.py")
DEMO_DASH = os.path.join(ROOT, "oslbench", "drivetrain_dashboard.py")


def _src(path):
    if not os.path.exists(path):
        raise Skip(f"{os.path.relpath(path, ROOT)} not present")
    with open(path, "r", encoding="utf-8") as fh:
        return fh.read()


def test_demo_dashboard_cannot_run_physics():
    """The second window must be unable to step a plant, structurally and not by habit.

    `oslbench/dashboard.py` holds this invariant for the servo demo and it is worth
    holding here too: if the dashboard cannot import an engine or a simulation, then no
    amount of future editing can quietly turn the presentation layer into a second
    integrator whose numbers disagree with the real one.

    Checked by parsing the AST, not by searching the text. The first version of this
    test grepped for substrings and failed on the module's own DOCSTRING, which
    explains what it does not import -- a test that cannot tell prose from code is
    worse than no test, because the fix is to delete the explanation.
    """
    import ast
    tree = ast.parse(_src(DEMO_DASH))
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported.update(a.name.split(".")[0] for a in node.names)
        elif isinstance(node, ast.ImportFrom):
            if node.level:                      # a relative import inside oslbench/
                imported.add("." + (node.module or ""))
            elif node.module:
                imported.add(node.module.split(".")[0])
                imported.add(node.module)
    banned = {"mujoco", "numpy", "oslbench.simulation", "oslbench.drivetrain_sim",
              "oslbench.drivetrain", "oslbench.model", ".simulation",
              ".drivetrain_sim", ".drivetrain", ".model"}
    hit = imported & banned
    assert not hit, (
        f"the drivetrain dashboard must stay a pure consumer of state, but it imports "
        f"{sorted(hit)}. It holds no model, takes no timestep and integrates nothing; "
        f"keeping the engine un-importable is what makes that structural.")
    # and it really is only the standard library
    assert imported <= {"json", "math", "threading", "time", "webbrowser",
                        "http", "http.server", "__future__"}, (
        f"unexpected dependency in the dashboard: "
        f"{sorted(imported - {'json', 'math', 'threading', 'time', 'webbrowser', 'http', 'http.server', '__future__'})}")


def test_demo_does_not_restate_the_control_law():
    """The demo may CALL PDCurrentSource; it may not re-derive the PD torque or current.

    This is the thing most likely to go wrong in a presentation script: someone needs a
    number on screen, re-computes it from the gains "just for the display", and now two
    expressions have to stay in sync. The demo reads every value from the LayerStep the
    physics returned instead.
    """
    text = _src(DEMO)
    body = "\n".join(ln for ln in text.splitlines()
                     if not ln.lstrip().startswith("#"))
    for banned in ("KP *", "KD *", "KP*", "KD*", "* KP", "* KD",
                   "kp *", "kd *", "/ k_t_joint", "/K_T_JOINT", "/ K_T_JOINT",
                   "torque_unclamped"):
        assert banned not in body, (
            f"the demo appears to re-implement the control law or the current "
            f"conversion: found {banned!r}. It must read these from PDCurrentSource and "
            f"the LayerStep, not recompute them.")
    assert "PDCurrentSource(" in text, "the demo must use the shipped current source"
    assert "disconnect_servo=True" in text, (
        "the demo must use the existing servo-disconnect mechanism")
    assert "sim.step(" in text, "the demo must drive the shared step"


def test_demo_does_not_offer_to_change_the_physics():
    """No --kp/--kv/--dt/--timestep flag, and no write to the model.

    A demo with a gain flag is a demo that can be shown with gains nobody validated.
    The gains are imported from oslbench.controller and are not settable here.
    """
    text = _src(DEMO)
    for banned in ('"--kp"', '"--kv"', '"--kd"', '"--dt"', '"--timestep"',
                   '"--p1"', '"--n-t"', '"--substeps"'):
        assert banned not in text, (
            f"the demo must not expose {banned} -- it would let the presentation show "
            f"parameters the quantitative run never validated")
    assert "opt.timestep" not in text, "the demo must not touch the timestep"
    for banned in ("osl_v2_bench.xml\", \"w", "osl_v2_bench.xml', 'w",
                   "write_to_model", "mj_saveModel", "mj_saveLastXML"):
        assert banned not in text, f"the demo must not write the model: found {banned!r}"


def test_demo_declares_the_option_c_label_and_the_disclaimer():
    """The two strings the brief requires, and they must reach the screen.

    Asserted on the dashboard module (where they are defined and baked into the page)
    and on the demo (which prints them to the terminal), so neither can drift.
    """
    import oslbench.drivetrain_dashboard as DD
    assert DD.BANNER == "OPTION C — PAPER-DERIVED ACTUATOR + COMPLIANT BELT", DD.BANNER
    assert DD.DISCLAIMER == "SIMULATION ONLY — NOT HARDWARE VALIDATION", DD.DISCLAIMER
    page = _src(DEMO_DASH)
    assert "__BANNER__" in page and "__DISCLAIMER__" in page, (
        "the page template must carry both placeholders so they are rendered in the "
        "browser window, not only printed in the terminal")
    text = _src(DEMO)
    assert "BANNER" in text and "DISCLAIMER" in text


def test_demo_dashboard_publishes_all_eleven_requested_signals():
    """The eleven live readouts, by name, so one cannot be dropped silently."""
    import oslbench.drivetrain_dashboard as DD
    want = {"phase", "ref", "act", "err", "tau_j", "i_q", "th_a", "th_ad", "th_s",
            "tau_a", "k_s"}
    assert set(DD.SCALARS) == want, (
        f"missing {want - set(DD.SCALARS)}, unexpected {set(DD.SCALARS) - want}")
    assert len(DD.SCALARS) == 11
    # and the four plots' traces
    want_tr = {"x", "y_act", "y_thj", "y_thar", "y_ths", "y_tau", "y_iq"}
    assert set(DD.TRACES) == want_tr, (
        f"missing {want_tr - set(DD.TRACES)}, unexpected {set(DD.TRACES) - want_tr}")


def test_demo_dashboard_survives_a_non_finite_value():
    """A NaN must become JSON `null` plus a flag -- never the literal NaN.

    `JSON.parse` rejects `NaN`, so an unguarded one would make the page stop updating at
    the exact moment something went wrong. That is the worst failure mode a diagnostic
    can have, so it gets a test rather than a comment.
    """
    import json
    import oslbench.drivetrain_dashboard as DD
    d = DD.DrivetrainWebDashboard(
        dict(subject="T", trial="t"), ([0.0, 50.0], [0.0, 1.0]),
        dict(ang=(0, 1), ths=(-1, 1), gap=(-1, 1), tau=(-1, 1), iq=(-1, 1),
             fit_edge=0.055, stance_end=60.0),
        port=0, open_browser=False)
    try:
        s = {f: float("nan") for f in DD.SCALARS}
        s.update({f: [0.0, float("inf"), 1.0] for f in DD.TRACES})
        s["foot"] = s["status"] = ""
        d.update(s)
        body = d._body.decode()
        assert "NaN" not in body and "Infinity" not in body, (
            f"non-finite values leaked into the JSON body: {body[:200]}")
        snap = json.loads(body)          # must parse, which is the whole point
        assert snap["nonfinite"] is True, "the non-finite flag was not raised"
        assert snap["tau_j"] is None
    finally:
        d.close()


def test_demo_dashboard_update_does_no_physics_arithmetic():
    """update() may round and decimate. It may not scale, integrate or convert units.

    If the page could rescale a signal, the numbers on screen would stop being the
    numbers the simulation produced -- and nobody would be able to tell from the screen.
    """
    import oslbench.drivetrain_dashboard as DD
    d = DD.DrivetrainWebDashboard(
        dict(subject="T", trial="t"), ([0.0, 50.0], [0.0, 1.0]),
        dict(ang=(0, 1), ths=(-1, 1), gap=(-1, 1), tau=(-1, 1), iq=(-1, 1),
             fit_edge=0.055, stance_end=60.0),
        port=0, open_browser=False)
    try:
        vals = dict(phase=12.5, ref=1.5, act=2.5, err=1.0, tau_j=-3.25, i_q=0.75,
                    th_a=1.125, th_ad=-2.5, th_s=-0.004, tau_a=-0.705, k_s=876.0)
        s = dict(vals)
        s.update({f: [] for f in DD.TRACES})
        s["foot"] = s["status"] = ""
        d.update(s)
        import json
        snap = json.loads(d._body.decode())
        for k, v in vals.items():
            assert snap[k] == v, (
                f"{k} came back as {snap[k]} instead of {v} -- the dashboard altered a "
                f"value it was only supposed to display")
    finally:
        d.close()


def test_seed_rigid_puts_the_actuator_at_n_t_times_the_joint():
    """A sanity check on the state the feedback reads: theta_a_dot = n_t*theta_j_dot."""
    layer = DrivetrainLayer(PAPER, enabled=True, substeps=1)
    layer.seed_rigid(0.25, 3.0)
    assert abs(layer.theta_a_dot - N_T * 3.0) < 1e-12, (
        f"theta_a_dot = {layer.theta_a_dot}, expected n_t*3 = {N_T * 3.0}")
    # ...so a rigid transmission makes the two laws IDENTICAL, which is why the old law
    # was invisible until the belt arrived.
    src = PDCurrentSource(PDController(KP, KD, CTRL, FORCE_LIM), REF, layer, PAPER)
    src(0, 0.25, 3.0)
    assert abs(src.last_qdot_used - 3.0) < 1e-12, (
        "under a rigid seed theta_a_dot/n_t must equal theta_j_dot exactly; if it does "
        "not, the n_t in the feedback path does not match the n_t in the kinematics")


def test_module_still_guards_its_mujoco_import():
    """drivetrain_sim must stay importable without MuJoCo, as the sandbox relies on."""
    import oslbench.drivetrain_sim as DS
    assert DS is not None
    import oslbench.simulation as S
    src_path = S.__file__
    with open(src_path, "r", encoding="utf-8") as fh:
        text = fh.read()
    assert "except ImportError" in text, (
        "oslbench/simulation.py must keep guarding `import mujoco`, or every "
        "sandbox-side check in this repo stops running")


# =====================================================================================
def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    verbose = "-v" in argv or "--verbose" in argv
    pattern = None
    if "-k" in argv:
        pattern = argv[argv.index("-k") + 1]

    g = globals()
    names = [n for n in g if n.startswith("test_")]
    names.sort(key=lambda n: g[n].__code__.co_firstlineno)
    if pattern:
        names = [n for n in names if pattern in n]

    print("drivetrain_sim tests -- oslbench/drivetrain_sim.py (the Option-C layer)")
    print(f"python {sys.version.split()[0]}   "
          f"derivative feedback: {PDCurrentSource.FEEDBACK_SIDE} side")
    print("-" * 78)

    failed, skipped = [], []
    for name in names:
        try:
            g[name]()
        except Skip as exc:
            skipped.append(name)
            print(f"SKIP  {name}: {exc}")
        except AssertionError as exc:
            failed.append(name)
            print(f"FAIL  {name}: {exc}")
        except Exception as exc:                                  # noqa: BLE001
            failed.append(name)
            print(f"ERROR {name}: {type(exc).__name__}: {exc}")
        else:
            if verbose:
                print(f"ok    {name}")

    print("-" * 78)
    print(f"{len(names) - len(failed) - len(skipped)} passed, {len(failed)} failed, "
          f"{len(skipped)} skipped, {len(names)} total")
    if failed:
        print("failed: " + ", ".join(failed))
    return len(failed)


if __name__ == "__main__":
    raise SystemExit(main())
