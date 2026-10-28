"""Path B: torque-domain PD and its own rotor-current conversion."""
from __future__ import annotations
from .drivetrain import PAPER, DrivetrainParameters
from .servo import PDController
from .compliance import DrivetrainLayer


class PDCurrentSource:
    """Turn the EXISTING joint-space PD law into a CURRENT command.  No new control law.

    The law is `oslbench.control.servo.PDController.torque_unclamped`, used verbatim and
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
        `oslbench/control/drivetrain.py` documents NO drive current limit from the paper.
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
