"""Compliant drivetrain actuation wiring for MuJoCo, paths B and C."""
from __future__ import annotations
import math
from oslbench.control import drivetrain as D
from oslbench.control.drivetrain import PAPER, DrivetrainParameters
from oslbench.control.servo import PDController
from oslbench.control.compliance import *
from oslbench.control.torque import PDCurrentSource
from .servo import BenchSimulation


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
