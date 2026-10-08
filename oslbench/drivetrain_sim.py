"""
oslbench.drivetrain_sim -- Option C: the EXTERNAL actuator-dynamics layer.

WHAT THIS IS
    `oslbench/drivetrain.py` holds Best et al.'s drivetrain EQUATIONS and no state.
    This file adds the one thing those equations need in order to run inside a
    simulation and nothing else: ownership of the actuator shaft coordinate
    (theta_a, theta_a_dot), a time integrator for it, and a way to hand the belt's
    joint-side torque to MuJoCo.

    It is Option C of `docs/DRIVETRAIN_INTEGRATION_OPTIONS.md`, implemented at the
    smallest size that lets the three acceptance tests run:

        experiments/layer_null_test.py      layer OFF  -> oracle reproduced exactly
        experiments/layer_static_test.py    layer ON   -> signs and frames are right
        experiments/layer_energy_drift.py   layer ON   -> splitting error MEASURED

WHAT IT DELIBERATELY DOES NOT DO
    No MJCF is touched.  No controller is touched.  No gain is changed.  nq stays 2.
    There is no gait logic, no reference loading, no plotting and no CSV here.
    `DrivetrainLayer` does not import MuJoCo at all; only `DrivetrainBenchSimulation`
    does, and that import is inherited rather than repeated.

THE COORDINATE THAT MATTERS MOST
    theta_a is the ACTUATOR OUTPUT shaft -- the belt's INPUT pulley, downstream of the
    9:1 planetary.  J_a, B_a, f_c and f_g all live on that shaft (the n_a^2 rotor
    reflection is already inside J_a).  Reflection from theta_a to the joint therefore
    uses the BELT ratio n_t alone, never n_a*n_t.  See docs/DRIVETRAIN_PARAMETERS.md.

        theta_s = theta_j - theta_a/n_t          belt deflection, JOINT frame, paper (3)
        tau_j   = -sgn(theta_s)*rho(|theta_s|)   what the belt gives the JOINT, paper (4)
        tau_a   = tau_j/n_t                      what the belt takes from the ACTUATOR, (7)

THE EQUATIONS OF MOTION THIS LAYER INTEGRATES
    Only the actuator shaft.  MuJoCo already owns the joint side.

        J_a*theta_a_ddot = tau_m - tau_f - B_a*theta_a_dot - tau_j/n_t

    with tau_m = I_q*k_t*n_a (paper 1) and tau_f = sgn(theta_a_dot)(f_c + f_g|I_q|)
    (paper 2).  Note what this makes tau_j = n_t*tau_a: an INSTANTANEOUS IDENTITY that
    holds because the belt is modelled as MASSLESS, not because the drivetrain is
    assumed rigid.  The belt is not rigid here -- it is the entire point.

WHY THE OPERATOR SPLIT IS NOT AS BAD AS STAGE 2 FEARED
    Stage 2 estimated an artificial damping of order omega*h/4 ~ 0.02 from a one-step
    coupling lag.  That estimate was pessimistic, and the reason is worth stating,
    because it is checkable rather than hopeful:

      * The belt torque depends on POSITIONS ONLY (theta_j, theta_a).  It has no
        velocity term at all -- the paper reports no belt damping.
      * MuJoCo's `implicitfast` updates velocity first using forces evaluated at the
        OLD configuration, then position with the NEW velocity.  `qfrc_applied` is
        read once per mj_step, at that same old configuration.
      * This layer does exactly the same: it evaluates tau_j at the old (theta_j,
        theta_a), uses it for its own velocity update, and hands the SAME number to
        MuJoCo.

    So both sides see the identical spring force at the identical configuration, and
    the split scheme is algebraically the monolithic symplectic Euler scheme for the
    coupled two-degree-of-freedom system.  There is no one-step lag to damp anything.
    `experiments/layer_energy_drift.py` exists to test that claim instead of trusting
    it.  CAVEAT: the argument holds for `Euler` and `implicitfast`.  It would NOT hold
    for `RK4`, which evaluates forces at intermediate configurations.

SUB-STEPPING
    `substeps > 1` advances the actuator n times per MuJoCo step with the joint angle
    held frozen, and hands MuJoCo the TIME-AVERAGE of tau_j so the impulse over the
    step is preserved.  This is available so the question "is sub-stepping required?"
    can be answered by measurement.  Note that averaging deliberately breaks the exact
    scheme symmetry described above, so substeps=1 is expected to conserve energy
    BETTER, not worse -- which is itself a result worth reporting.

SIGN CHAIN, WRITTEN OUT ONCE
    I_q > 0  ->  tau_m > 0  ->  theta_a accelerates positive  ->  theta_a/n_t grows
    ->  theta_s = theta_j - theta_a/n_t goes NEGATIVE  ->  paper (4) returns tau_j > 0.
    A positive current produces a positive joint torque through a NEGATIVE deflection.
    If that feels backwards, it is the same sign convention as paper Fig. 3.
"""

from __future__ import annotations

import math

from . import drivetrain as D
from .drivetrain import PAPER, DrivetrainParameters
from .controller import PDController
from .simulation import BenchSimulation

__all__ = [
    "BeltState", "LayerStep", "DrivetrainLayer", "DrivetrainBenchSimulation",
    "belt_potential_energy", "frictionless_parameters",
    "PDCurrentSource", "ConstantCurrent", "StepCurrent",
]


# --------------------------------------------------------------------------- energy
def belt_potential_energy(theta_s: float, p: DrivetrainParameters = PAPER) -> float:
    """U(theta_s) = p2*|theta_s|^3/3 + p1*theta_s^2/2   [J], the belt's stored energy.

    This is the potential whose gradient is the paper's own belt law, and it is the
    only quantity in the model that makes an energy-conservation test meaningful.
    The derivation is one line and worth keeping here because a sign error in it would
    silently invalidate the drift test:

        dU/dtheta_s = p2*theta_s*|theta_s| + p1*theta_s
                    = sgn(theta_s) * (p2*theta_s^2 + p1*|theta_s|)
                    = sgn(theta_s) * rho(|theta_s|)
                    = -tau_j            by paper (4).

    So tau_j = -dU/dtheta_s, and because theta_s = theta_j - theta_a/n_t,

        Q_j = -dU/dtheta_j = tau_j          (the joint feels +tau_j)
        Q_a = -dU/dtheta_a = -tau_j/n_t     (the actuator feels -tau_a)

    which is exactly the pair of couplings implemented below.  U >= 0 always, U(0) = 0,
    and U is even in theta_s.
    """
    a = abs(theta_s)
    return p.p2 * a ** 3 / 3.0 + p.p1 * theta_s ** 2 / 2.0


def frictionless_parameters(p: DrivetrainParameters = PAPER) -> DrivetrainParameters:
    """A copy of `p` with B_a = f_c = f_g = 0 -- the energy-drift test's parameter set.

    Every dissipative term in the paper's actuator model is removed and NOTHING else
    changes, so the remaining system is conservative by construction and any measured
    energy change is numerical rather than physical.  This is a TEST FIXTURE.  It is
    not a claim that the OSL V2 drivetrain is frictionless.
    """
    return DrivetrainParameters(
        n_a=p.n_a, n_t=p.n_t, k_t=p.k_t, J_a=p.J_a,
        B_a=0.0, f_c=0.0, f_g=0.0,
        p1=p.p1, p2=p.p2,
    )


# ----------------------------------------------------------------- small records
class BeltState:
    """The belt evaluated at one configuration.  Pure function of (theta_j, theta_a)."""

    __slots__ = ("theta_s", "tau_j", "tau_a", "K_s", "U")

    def __init__(self, theta_s: float, tau_j: float, tau_a: float,
                 K_s: float, U: float):
        self.theta_s, self.tau_j, self.tau_a = theta_s, tau_j, tau_a
        self.K_s, self.U = K_s, U

    def __repr__(self) -> str:                                  # pragma: no cover
        return (f"BeltState(theta_s={self.theta_s:+.9e} rad, tau_j={self.tau_j:+.9f} "
                f"N.m, tau_a={self.tau_a:+.9f} N.m, K_s={self.K_s:.6f} N.m/rad, "
                f"U={self.U:.9e} J)")


class LayerStep:
    """What one `DrivetrainLayer.advance` call did.  Diagnostics, not physics.

    TIME STAMPS MATTER HERE, so they are recorded explicitly rather than implied.
    One semi-implicit sub-step reads some quantities BEFORE the update and some AFTER,
    and mixing them silently would make the consistency residuals in
    `experiments/layer_dynamic_test.py` look like physics errors when they are only
    bookkeeping errors:

        theta_j         the joint angle the belt was evaluated at (pre-mj_step)
        theta_a_pre     actuator angle BEFORE the sub-step  -> pairs with theta_s
        theta_a         actuator angle AFTER  the sub-step
        theta_a_dot_pre velocity BEFORE  -> the argument tau_f was evaluated at
        theta_a_dot     velocity AFTER   -> the velocity the implicit B_a term used
        theta_a_ddot    (theta_a_dot - theta_a_dot_pre)/dt, the realised acceleration
        theta_s, tau_j, tau_a, K_s, U      all at (theta_j, theta_a_pre)
        tau_m           at i_q;  tau_f at (theta_a_dot_pre, i_q)

    With substeps > 1 the `*_pre` fields belong to the LAST sub-step, which is the only
    one the returned belt state describes either.  substeps == 1 is the verified
    configuration (see docs/DRIVETRAIN_INTEGRATION_OPTIONS.md), where it is exact.
    """

    __slots__ = ("tau_j", "tau_j_mean", "tau_a", "theta_s", "theta_a", "theta_a_dot",
                 "tau_m", "tau_f", "i_q", "K_s", "U",
                 "theta_j", "theta_a_pre", "theta_a_dot_pre", "theta_a_ddot")

    def __init__(self, **kw):
        for key, val in kw.items():
            setattr(self, key, val)


# --------------------------------------------------------------------- THE LAYER
class DrivetrainLayer:
    """The actuator shaft, its integrator, and the belt coupling.  NO MuJoCo.

        p           the frozen paper parameters (or a variant, e.g. frictionless)
        enabled     False -> the layer is inert; nothing may be written anywhere
        substeps    actuator sub-steps per outer step (see the module docstring)

    State
        theta_a      rad     actuator OUTPUT shaft angle (post-planetary, pre-belt)
        theta_a_dot  rad/s   its velocity

    The class is deliberately dumb about time: `advance` is told dt rather than owning
    a clock, so the same object can be driven by MuJoCo, by a bare loop, or by a test.
    """

    def __init__(self, p: DrivetrainParameters = PAPER, enabled: bool = False,
                 substeps: int = 1):
        if substeps < 1:
            raise ValueError("substeps must be >= 1")
        self.p = p
        self.enabled = bool(enabled)
        self.substeps = int(substeps)
        self.theta_a = 0.0
        self.theta_a_dot = 0.0
        self.steps = 0

    # ------------------------------------------------------------------ seeding
    def seed_rigid(self, theta_j: float, theta_j_dot: float = 0.0) -> None:
        """Place the actuator where a RIGID belt would put it: theta_s = 0 exactly.

        theta_a = n_t*theta_j is the zero-deflection, zero-stored-energy state.  It is
        the right initial condition for any run that should not begin with the belt
        already wound up.
        """
        self.theta_a = self.p.n_t * float(theta_j)
        self.theta_a_dot = self.p.n_t * float(theta_j_dot)

    def seed_from_joint_torque(self, theta_j: float, tau_j: float,
                               theta_j_dot: float = 0.0) -> float:
        """Wind the belt up to a chosen joint torque and return the deflection used.

        Inverts paper (12) to get theta_s, then (3) to get the theta_a that produces
        it.  This is how the drift test "initialises a belt deflection" in units that
        mean something physically (a torque) rather than an arbitrary angle.
        """
        theta_s = D.deflection_for_joint_torque(float(tau_j), self.p)
        self.theta_a = self.p.n_t * (float(theta_j) - theta_s)
        self.theta_a_dot = self.p.n_t * float(theta_j_dot)
        return theta_s

    def seed_deflection(self, theta_j: float, theta_s: float,
                        theta_j_dot: float = 0.0) -> None:
        """Set an explicit belt deflection theta_s at joint angle theta_j."""
        self.theta_a = self.p.n_t * (float(theta_j) - float(theta_s))
        self.theta_a_dot = self.p.n_t * float(theta_j_dot)

    # ------------------------------------------------------------------- reading
    def belt(self, theta_j: float) -> BeltState:
        """Evaluate the belt at the CURRENT (theta_j, theta_a).  No state changes.

        Every number here comes from `oslbench/drivetrain.py`; nothing is re-derived.
        """
        p = self.p
        theta_s = D.belt_deflection(float(theta_j), self.theta_a, p)   # paper (3)
        tau_j = D.joint_torque_from_deflection(theta_s, p)             # paper (4)
        tau_a = D.actuator_load_torque_from_joint(tau_j, p)            # paper (7)
        return BeltState(theta_s, tau_j, tau_a,
                         D.belt_stiffness(theta_s, p),                 # paper (5)
                         belt_potential_energy(theta_s, p))

    def kinetic_energy(self) -> float:
        """0.5*J_a*theta_a_dot^2 [J] -- the ACTUATOR's kinetic energy only.

        The joint side's kinetic energy belongs to whoever owns the joint (MuJoCo, or
        the caller in a MuJoCo-free test), so it is deliberately not included here.
        """
        return 0.5 * self.p.J_a * self.theta_a_dot ** 2

    # -------------------------------------------------------------- THE INTEGRATOR
    def _advance_once(self, theta_j: float, i_q: float, dt: float):
        """One semi-implicit (symplectic) Euler sub-step of the ACTUATOR shaft.

            J_a*theta_a_ddot = tau_m - tau_f - B_a*theta_a_dot - tau_j/n_t

        Damping is taken IMPLICITLY because B_a/J_a = 6.166 1/s makes the explicit
        damping step dt*B_a/J_a = 3.08e-3 at h = 0.5 ms -- small, but treating it
        implicitly is unconditionally stable and costs one division:

            v_new = (v + dt*(tau_m - tau_f - tau_a)/J_a) / (1 + dt*B_a/J_a)
            x_new = x + dt*v_new

        The spring term tau_a is EXPLICIT, at the old configuration, on purpose: that
        is what makes this scheme match what MuJoCo does with `qfrc_applied`, and it is
        what makes the pair of them symplectic together.  See the module docstring.
        """
        p = self.p
        st = self.belt(theta_j)
        tau_m = D.motor_torque(i_q, p)                                 # paper (1)
        tau_f = D.friction_torque(self.theta_a_dot, i_q, p)            # paper (2)
        x_pre, v_pre = self.theta_a, self.theta_a_dot
        acc_num = tau_m - tau_f - st.tau_a
        self.theta_a_dot = ((self.theta_a_dot + dt * acc_num / p.J_a)
                            / (1.0 + dt * p.B_a / p.J_a))
        self.theta_a += dt * self.theta_a_dot
        return st, tau_m, tau_f, x_pre, v_pre

    def advance(self, theta_j: float, i_q: float, dt: float) -> LayerStep:
        """Advance the actuator by dt and return the joint torque MuJoCo should apply.

        The returned `tau_j_mean` is the number to write into `qfrc_applied`: with
        substeps == 1 it is exactly the tau_j used internally, and with substeps > 1 it
        is the time-average over the sub-steps so that the IMPULSE delivered to the
        joint over the outer step is right.

        theta_j is held FIXED across sub-steps because MuJoCo has not moved the joint
        yet -- it moves when mj_step runs, after this returns.
        """
        n = self.substeps
        h = dt / n
        acc = 0.0
        st = tau_m = tau_f = None
        x_pre = v_pre = 0.0
        for _ in range(n):
            st, tau_m, tau_f, x_pre, v_pre = self._advance_once(theta_j, i_q, h)
            acc += st.tau_j
        self.steps += 1
        return LayerStep(tau_j=st.tau_j, tau_j_mean=acc / n, tau_a=st.tau_a,
                         theta_s=st.theta_s, theta_a=self.theta_a,
                         theta_a_dot=self.theta_a_dot, tau_m=tau_m, tau_f=tau_f,
                         i_q=float(i_q), K_s=st.K_s, U=st.U,
                         theta_j=float(theta_j), theta_a_pre=x_pre,
                         theta_a_dot_pre=v_pre,
                         theta_a_ddot=(self.theta_a_dot - v_pre) / h)


# ------------------------------------------------------------------ CURRENT SOURCES
# A current source is any callable (k, theta_j, theta_j_dot) -> I_q amps.  These three
# cover every input this phase needs.  None of them is a tuned controller; the first two
# are open-loop waveforms for the plant test, and the third is the EXISTING PD law
# re-pointed at the current input instead of at the joint.

class ConstantCurrent:
    """I_q = const, for every step.  The simplest possible plant excitation."""

    def __init__(self, amps: float):
        self.amps = float(amps)

    def __call__(self, k: int, theta_j: float, theta_j_dot: float) -> float:
        return self.amps

    def __repr__(self) -> str:                                  # pragma: no cover
        return f"ConstantCurrent({self.amps:+.4f} A)"


class StepCurrent:
    """I_q = `first` until t >= `t_switch`, then `second`.  Switching is on STEP INDEX,

    computed as round(t_switch/dt), so the transition lands on an exact step boundary
    and the trace is reproducible independently of floating-point time accumulation.
    """

    def __init__(self, first: float, second: float, t_switch: float, dt: float):
        self.first, self.second = float(first), float(second)
        self.t_switch, self.dt = float(t_switch), float(dt)
        self.k_switch = int(round(float(t_switch) / float(dt)))

    def __call__(self, k: int, theta_j: float, theta_j_dot: float) -> float:
        return self.first if k < self.k_switch else self.second

    def __repr__(self) -> str:                                  # pragma: no cover
        return (f"StepCurrent({self.first:+.4f} -> {self.second:+.4f} A at "
                f"t = {self.t_switch:.4f} s = step {self.k_switch})")


class PDCurrentSource:
    """Turn the EXISTING joint-space PD law into a CURRENT command.  No new control law.

    The law is `oslbench.controller.PDController.torque_unclamped`, used verbatim and
    with its gains untouched.  The only new line of arithmetic is the division that
    converts the requested JOINT torque into the motor current that would produce it
    through the transmission:

            I_q = tau_requested / k_t_joint ,     k_t_joint = n_t * k_t * n_a

    THE DERIVATIVE TERM IS COLLOCATED WITH THE ACTUATOR.  READ THIS BEFORE EDITING.
        The velocity fed to the derivative term is the actuator shaft's own velocity,
        referred to the joint:

            tau_req = Kp*(theta_ref - theta_j) - Kd*(theta_a_dot / n_t)
                           ^ joint-side error        ^ ACTUATOR-side velocity

        It is NOT `theta_j_dot`, and the `theta_j_dot` argument this object is handed is
        deliberately ignored for feedback (it is still recorded, as `last_theta_j_dot`,
        so logs and tests can see both).  That asymmetry is the point:

          - the POSITION term stays joint-side because tracking the joint is the task;
          - the VELOCITY term must be actuator-side because the current it commands acts
            on the actuator, and the belt sits between the two.

        Feeding `theta_j_dot` here is a NON-COLLOCATED feedback loop: a damping torque
        applied to one inertia in proportion to the velocity of a different inertia,
        with a spring between them.  In the belt mode the two swing in ANTIPHASE, so
        `-Kd*theta_j_dot` applied on the actuator points along the actuator's own motion
        and PUMPS the mode instead of damping it.  This was measured on real MuJoCo, not
        argued: at Kp = 600, Kd = 17.253 the joint-velocity law diverges at t = 0.3485 s
        with |tau_j| through 1463 N.m, while this collocated law completes the same AB19
        cycle with peak |tau_j| = 16.70 N.m.  The linear 2-mass model puts the
        joint-velocity stability boundary at Kd < 1.41 -- 12x below the gain in use --
        and finds NO upper boundary at all for the actuator-velocity law in [0, 200].
        Full evidence: `docs/DRIVETRAIN_INSTABILITY_DIAGNOSIS.md`.

        `tests/test_drivetrain_sim.py` pins this down so it cannot revert quietly; see
        `test_derivative_feedback_is_actuator_side_not_joint_side` and its neighbours.

    WHY theta_a_dot / n_t AND NOT theta_a_dot
        Kd = 17.253 N.m.s/rad is a JOINT-side gain: it was solved from a joint-side
        damping ratio, `controller.kd_for_damping_ratio(600, 0.261998, 0.3, 0.7)`.
        Feeding raw `theta_a_dot` would multiply the effective gain by the TRANSMISSION
        RATIO n_t = 4.61 and so change the gain at the same time as the feedback point.
        Dividing by n_t keeps the gain, its units and the nominal damping identical, and
        changes only WHICH SIDE OF THE BELT the velocity is read from.

        Do not confuse n_t = 4.61 (dimensionless ratio) with k_t_joint = 4.597092 N.m/A
        (the torque constant below).  The two numbers are nearly equal by coincidence and
        belong to different frames; this file's history has enough ratio-vs-constant
        confusion in it already.  A cross-check that holds exactly: the shaft-referred
        damping this term implies, times n_t^2, is Kd.

    TIME ALIGNMENT
        `DrivetrainBenchSimulation.step` calls this BEFORE `layer.advance`, so
        `layer.theta_a_dot` still holds the end-of-previous-step value -- the same
        instant MuJoCo's `qvel` is read at.  Both velocities are therefore pre-step and
        mutually consistent; no half-step offset is introduced by this change.  After
        `reset`, `seed_rigid` sets `theta_a_dot = n_t * 0 = 0` and `qvel` is 0, so the
        first step is consistent too.

    WHAT THIS CHANGES, AND WHAT IT DOES NOT
        It does NOT change the gains, the reference, or the position error signal.  What
        changes is the PATH from the law's output to the joint.  Before Option C: the
        request became joint torque instantly, inside MuJoCo's position actuator.  Now:
        the request becomes a current, the current drives the actuator shaft through
        J_a / B_a / friction, and the joint feels only what the belt transmits.  That is
        the actuator-fidelity comparison this phase exists to make.

    WHY THE REQUEST IS NOT CLAMPED HERE
        `torque_unclamped` is used, not `torque`, because the +-142.2 N.m forcerange is
        a property of the MuJoCo position actuator that has just been disconnected, and
        `oslbench/drivetrain.py` documents NO drive current limit from the paper.
        Inventing one would be inventing physics.  Peak |I_q| is logged instead, so the
        reader can see what the drive was asked for.  ASSUMED: no current limit.

    SIGN
        The chain is I_q > 0 -> tau_m > 0 -> theta_a rises -> theta_s = theta_j -
        theta_a/n_t goes NEGATIVE -> paper (4) returns tau_j > 0.  So a positive torque
        request yields a positive joint torque, the same sign the servo had.
    """

    # Which side of the belt the derivative term reads.  Declared as data so that a
    # test, a log header or the diagnostic script can ASSERT the wiring instead of
    # trusting a comment.  "actuator" is collocated; "joint" is the law that diverged.
    FEEDBACK_SIDE = "actuator"

    def __init__(self, pd: PDController, ref_rad, layer: "DrivetrainLayer",
                 p: DrivetrainParameters = PAPER):
        # `layer` is REQUIRED and deliberately positional-before-`p`.  The derivative
        # term cannot be evaluated without the actuator state, so there is no sensible
        # default -- and a default would be exactly the silent revert to joint-velocity
        # feedback that this class now exists to prevent.  A caller that passes the old
        # three-argument form `(pd, ref, PAPER)` lands `PAPER` here and is rejected
        # immediately, with a message naming the fix, rather than failing mid-run.
        if not hasattr(layer, "theta_a_dot"):
            raise TypeError(
                "PDCurrentSource now requires the DrivetrainLayer as its third "
                "argument, because the derivative term reads the ACTUATOR velocity "
                f"theta_a_dot/n_t, not theta_j_dot.  Got {type(layer).__name__}.  "
                "Call PDCurrentSource(pd, ref_rad, layer, p) -- and pass the SAME "
                "layer instance that DrivetrainBenchSimulation was given, or the "
                "feedback will read a shaft that is not the one being driven.")
        self.pd = pd
        self.ref = ref_rad
        self.layer = layer
        self.p = p
        self.k_t_joint = p.n_t * p.k_t * p.n_a          # N.m/A, PAPER-DERIVED
        self.last_request_Nm = 0.0
        self.last_qdot_used = 0.0       # the velocity the derivative term actually used
        self.last_theta_j_dot = 0.0     # the joint velocity it did NOT use, for logs

    def __call__(self, k: int, theta_j: float, theta_j_dot: float) -> float:
        q_ref = float(self.ref[k if k >= 0 else 0])
        # THE ONE LINE.  theta_j_dot is recorded and then not used for feedback.
        self.last_theta_j_dot = float(theta_j_dot)
        qdot_fb = self.layer.theta_a_dot / self.p.n_t
        self.last_qdot_used = qdot_fb
        tau_req = self.pd.torque_unclamped(q_ref, theta_j, qdot_fb)
        self.last_request_Nm = tau_req
        return tau_req / self.k_t_joint

    def __repr__(self) -> str:                                  # pragma: no cover
        return (f"PDCurrentSource(kp={self.pd.kp:g}, kd={self.pd.kd:g}, "
                f"k_t_joint={self.k_t_joint:.6f} N.m/A, "
                f"derivative on {self.FEEDBACK_SIDE} velocity)")


# ------------------------------------------------------- THE MuJoCo-SIDE SUBCLASS
class DrivetrainBenchSimulation(BenchSimulation):
    """`BenchSimulation` plus an optional drivetrain layer on the KNEE.

        sim = DrivetrainBenchSimulation(bench, PDController.knee(bench, kp, kv),
                                        layer=DrivetrainLayer(enabled=False))

    WHY A SUBCLASS AND NOT AN EDIT
        `BenchSimulation.step` has no pre-step hook, and its `on_step` callback both
        fires AFTER mj_step and is documented read-only.  The belt torque must be
        written BEFORE mj_step, from the pre-step configuration.  Overriding `step`
        is therefore the only route that leaves `oslbench/simulation.py` untouched.

    THE OFF GUARANTEE
        With `layer is None` or `layer.enabled is False`, this class writes NOTHING
        anywhere and calls `super().step` with the identical arguments, so the run is
        bit-identical to the plain `BenchSimulation`.  That is not an aspiration --
        `experiments/layer_null_test.py` checks it against the frozen oracle.
        `reset()` calls `mj_resetDataKeyframe`, which zeroes `qfrc_applied`, so a
        previously-enabled run cannot leak a stale force into a later one.

    THE CURRENT INPUT
        `current_source` is a callable (k, theta_j, theta_j_dot) -> I_q amps, so the
        layer's input is a CURRENT, as the paper's plant demands.  The default is zero
        current, which is the honest default: this stage implements the PLANT, not a
        controller for it.  See `ConstantCurrent`, `StepCurrent`, `PDCurrentSource`.

    THE SERVO DISCONNECT  (`disconnect_servo`)
        The MJCF knee `position` actuator and the belt torque are two independent
        actuators on one dof.  Running both is DOUBLE-COUNTED ACTUATION: the joint gets
        Kp*(ctrl-q) - Kd*qdot AND tau_j in the same mj_step, so any result would
        describe a bench that does not exist.  With `disconnect_servo=True` the knee
        position actuator is zeroed IN THE COMPILED MODEL, at runtime, by writing a
        Kp=0/Kd=0 controller through `PDController.write_to_model` -- the same public
        call the normal gains already go through.  `models/osl_v2_bench.xml` is never
        opened for writing, and `forcerange`/`ctrlrange` are not touched, so
        `bench.limits_unchanged()` still passes.

        After the disconnect the knee's ONLY torque source is `qfrc_applied`, i.e. the
        belt.  Three things follow and all three are intended:
          - the `knee_tau` actuatorfrc sensor reads 0.  That is the PROOF the servo is
            off, not a measurement failure.  Do not paper over it.
          - the `implicitfast` velocity solve loses the actuator's kv contribution to
            (M + h*D), because biasprm[2] was its only source.  No servo means no servo
            damping; only the joint's own damping = 0.3 remains.
          - `forcerange` no longer bounds anything, because `qfrc_applied` is a raw
            generalized force.  Torque authority must be judged on the current side,
            and `oslbench/drivetrain.py` documents no drive current limit.

        `self.knee` keeps its real gains either way, so `StepState.tau_unclamped` still
        reports the CONTROLLER REQUEST.  Request / position-actuator output / belt
        torque stay three separately observable quantities.

    THE OFF GUARANTEE IS UNAFFECTED
        `disconnect_servo` is honoured only when a layer is present AND enabled.  With
        the layer off this class still writes nothing anywhere, which is what
        `experiments/layer_null_test.py` checks against the frozen oracle.
    """

    def __init__(self, bench, knee=None, ankle=None,
                 layer: DrivetrainLayer | None = None, current_source=None,
                 disconnect_servo: bool = False):
        super().__init__(bench, knee, ankle)
        self.layer = layer
        self.current_source = current_source or (lambda k, q, qd: 0.0)
        self.last_layer: LayerStep | None = None
        self.servo_connected = True
        if disconnect_servo and layer is not None and layer.enabled:
            self.disconnect_knee_servo()

    # --------------------------------------------------------- the servo disconnect
    def disconnect_knee_servo(self) -> None:
        """Zero the knee position actuator's gain and bias in the COMPILED mjModel.

        Reuses `PDController.write_to_model` rather than poking gainprm/biasprm here,
        so there is exactly one place in the project that knows MuJoCo's position
        actuator algebra.  Kp = Kd = 0 makes that algebra return 0 for any `ctrl`.
        """
        bench = self.bench
        PDController(0.0, 0.0, bench.knee_ctrlrange, bench.knee_forcerange[1],
                     "knee_disconnected").write_to_model(bench.model, bench.knee_act)
        self.servo_connected = False

    def knee_servo_force(self) -> float:
        """The knee position actuator's force at the CURRENT state, N.m.

        Call after `mj_forward`/`mj_step`.  Must be exactly 0.0 once disconnected --
        that is the assertion the dynamic test makes instead of trusting the write.
        """
        return float(self.bench.data.actuator_force[self.bench.knee_act])

    # ------------------------------------------------------------------- reset
    def reset(self, q0_rad: float, ref_vel0: float | None = None, report=None):
        """Plain reset, then seed the actuator to zero belt deflection at q0.

        Seeding touches only this layer's own two floats; when the layer is disabled
        nothing observable changes, and `qfrc_applied` has just been zeroed by
        `mj_resetDataKeyframe` inside the parent.
        """
        hold = super().reset(q0_rad, ref_vel0, report)
        if self.layer is not None:
            self.layer.seed_rigid(float(q0_rad), 0.0)
            self.layer.steps = 0
        self.last_layer = None
        return hold

    # -------------------------------------------------------------------- step
    def step(self, q_ref: float, k: int = -1):
        """Write the belt torque (if enabled), then run the parent step verbatim."""
        layer = self.layer
        if layer is not None and layer.enabled:
            bench = self.bench
            data = bench.data
            theta_j = float(data.qpos[bench.knee_qpos])
            theta_j_dot = float(data.qvel[bench.knee_dof])
            i_q = float(self.current_source(k, theta_j, theta_j_dot))
            rec = layer.advance(theta_j, i_q, float(bench.model.opt.timestep))
            data.qfrc_applied[bench.knee_dof] = rec.tau_j_mean
            self.last_layer = rec
        return super().step(q_ref, k)


# ------------------------------------------------------------------ self-check
if __name__ == "__main__":                                        # pragma: no cover
    p = PAPER
    lay = DrivetrainLayer(enabled=True)
    lay.seed_rigid(0.0)
    st = lay.belt(0.0)
    assert st.theta_s == 0.0 and st.tau_j == 0.0 and st.U == 0.0
    th_s = lay.seed_from_joint_torque(0.0, 16.004120587370423)
    print(f"seed to tau_j=16.004120587370423 -> theta_s = {th_s:.12f} rad")
    print(f"round trip tau_j = "
          f"{D.joint_torque_from_deflection(th_s, p):.12f} N.m")
    print(f"U at that deflection = {belt_potential_energy(th_s, p):.12e} J")
    print(f"n_t*tau_a == tau_j : "
          f"{D.joint_torque_from_actuator(lay.belt(0.0).tau_a, p):.12f}")
    fp = frictionless_parameters()
    print(f"frictionless: B_a={fp.B_a} f_c={fp.f_c} f_g={fp.f_g} "
          f"J_a={fp.J_a} p1={fp.p1} p2={fp.p2} n_t={fp.n_t}")
    print(f"dU/dtheta_s numeric vs -tau_j at theta_s=-0.01: "
          f"{(belt_potential_energy(-0.01 + 1e-7) - belt_potential_energy(-0.01 - 1e-7)) / 2e-7:+.6f}"
          f" vs {-D.joint_torque_from_deflection(-0.01, p):+.6f}")
    print("drivetrain_sim self-check OK")
