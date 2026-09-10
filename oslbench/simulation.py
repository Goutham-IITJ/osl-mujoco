"""
oslbench.simulation -- the physics loop.  ONE implementation, used by the quantitative
benchmark, the gain sweep, the live viewer and the recorder.

READ `step()` AND YOU HAVE READ THE EXPERIMENT:

    q_ref  ->  controller.command()   (clip into the joint ROM)
           ->  data.ctrl[knee]        (a MuJoCo position servo: ctrl IS the angle, rad)
           ->  mujoco.mj_step()       (MuJoCo evaluates Kp*(ctrl-q) - Kd*qdot,
                                       clamps it to forcerange, integrates 0.5 ms)
           ->  data.sensordata[...]   (measure q, qdot, tau back out)

WHY ONE IMPLEMENTATION MATTERS
    The live demo used to have its own copy of that loop.  Two copies can drift, and
    then "the demo shows the same experiment" is a promise rather than a fact.  Now the
    viewer calls this same `step()`, so it CANNOT be running a different controller.
    `oslbench.viewer.consistency_check` still re-proves it numerically for the meeting.

WHAT THIS FILE DOES NOT DO
    No plotting, no metrics, no CSV.  It returns state; other modules interpret it.

ONE SUBTLETY WORTH KNOWING (it is easy to state this wrong)
    `tau_actuator` (MuJoCo's own actuator_force) is computed from the state BEFORE the
    step, while `tau_unclamped` is the Python law evaluated on the POST-step q and
    qdot.  They are therefore NOT equal, and neither is a bug.  Both are logged, and
    the identity test in tests/ compares the law against MuJoCo using the pre-step
    state, which is the only comparison that is meant to match.
"""

from __future__ import annotations

import math

import numpy as np

try:
    import mujoco
except ImportError:                                          # pragma: no cover
    mujoco = None

from .controller import PDController


class StepState:
    """Everything measured at one 0.5 ms step.  What the logger and the viewer consume."""

    __slots__ = ("k", "time_s", "q_ref", "command", "command_clamped",
                 "q", "qdot", "tau", "tau_actuator", "tau_unclamped",
                 "saturated", "ankle_q", "ankle_tau")

    def __init__(self, **kw):
        for key, val in kw.items():
            setattr(self, key, val)

    @property
    def error_rad(self) -> float:
        """q - q_ref, rad.  Against the UNCLAMPED reference, as the CSV reports it."""
        return self.q - self.q_ref

    @property
    def error_deg(self) -> float:
        return math.degrees(self.q - self.q_ref)

    @property
    def power_W(self) -> float:
        """Mechanical power at the knee joint, tau * qdot (W)."""
        return self.tau * self.qdot


class RunResult:
    """Per-step arrays for a whole run.  The input to metrics.py and logging.py.

    ref_rad        the reference actually commanded, unclamped, rad
    time_s         data.time after each step, s
    ctrl           the command after the ROM clip, rad
    q / qdot       measured knee angle (rad) and velocity (rad/s)
    tau            knee torque from the actuatorfrc sensor, N.m (post-clamp)
    tau_actuator   data.actuator_force, N.m (pre-step state -- see the module docstring)
    tau_unclamped  the law's request before the forcerange clamp, N.m
    saturated      bool array, True where the request exceeded forcerange
    ankle_q        rad; ankle_tau  N.m
    clamped_steps  how many commands the ROM clip actually moved (0 for AB19)
    ankle_dev_rad  worst |ankle angle - ankle keyframe| over the run
    final_time_s   data.time at the end
    """

    def __init__(self, n: int, ref_rad, force_limit: float):
        self.n = int(n)
        self.ref_rad = np.asarray(ref_rad, float)
        self.force_limit = float(force_limit)
        z = lambda: np.empty(n)                      # noqa: E731
        self.time_s, self.ctrl = z(), z()
        self.q, self.qdot = z(), z()
        self.tau, self.tau_actuator, self.tau_unclamped = z(), z(), z()
        self.ankle_q, self.ankle_tau = z(), z()
        self.saturated = np.zeros(n, bool)
        self.clamped_steps = 0
        self.ankle_dev_rad = 0.0
        self.final_time_s = 0.0

    def store(self, k: int, st: StepState) -> None:
        self.time_s[k] = st.time_s
        self.ctrl[k] = st.command
        self.q[k] = st.q
        self.qdot[k] = st.qdot
        self.tau[k] = st.tau
        self.tau_actuator[k] = st.tau_actuator
        self.tau_unclamped[k] = st.tau_unclamped
        self.saturated[k] = st.saturated
        self.ankle_q[k] = st.ankle_q
        self.ankle_tau[k] = st.ankle_tau
        self.clamped_steps += int(st.command_clamped)

    @property
    def error_deg(self):
        """q - q_ref in degrees, elementwise."""
        return np.degrees(self.q - self.ref_rad)

    @property
    def power_W(self):
        return self.tau * self.qdot


class BenchSimulation:
    """The bench, its controller, and the loop that couples them.

        bench    an oslbench.model.BenchModel
        knee     the PDController under test (Kp = 600, Kd = 17.253 by default)
        ankle    the ankle's own authored servo, installed unchanged

    Constructing this object WRITES the gains into the compiled mjModel.  It does not
    touch models/osl_v2_bench.xml, forcerange or ctrlrange.
    """

    def __init__(self, bench, knee: PDController | None = None,
                 ankle: PDController | None = None):
        if mujoco is None:                                    # pragma: no cover
            raise RuntimeError("mujoco is required to run the simulation")
        self.bench = bench
        self.knee = knee if knee is not None else PDController.knee(bench)
        self.ankle = ankle if ankle is not None else PDController.ankle_hold(bench)
        # install both controllers into the compiled model, once
        self.knee.write_to_model(bench.model, bench.knee_act)
        self.ankle.write_to_model(bench.model, bench.ankle_act)
        self.ankle_hold = 0.0

    # ------------------------------------------------------------------------ reset
    def reset(self, q0_rad: float, ref_vel0: float | None = None,
              report=None) -> float:
        """Put the bench in the validated initial condition and return the ankle hold angle.

        Reset to the "flat" keyframe, read the ankle's keyframe angle (that becomes the
        fixed boundary condition), seed the knee ON the reference so the run does not
        start with an artificial position error, and zero all velocities.
        """
        bench = self.bench
        model, data = bench.model, bench.data
        # BenchModel keeps mj_name2id's raw result, which is -1 if the "flat" keyframe is
        # ever renamed; mj_resetDataKeyframe(-1) would raise.  keyframe_id() guards it.
        mujoco.mj_resetDataKeyframe(model, data, bench.keyframe_id())
        self.ankle_hold = float(data.qpos[bench.ankle_qpos])
        data.qpos[bench.knee_qpos] = float(q0_rad)
        data.qvel[:] = 0.0
        mujoco.mj_forward(model, data)

        if report is not None:
            report.section("[S1] INITIAL CONDITION")
            vel_txt = "" if ref_vel0 is None else (
                f" (the reference itself starts at {ref_vel0:+.3f} rad/s, so expect a "
                f"brief start transient)")
            report.note(f"knee seeded at the reference value "
                        f"{math.degrees(q0_rad):.3f} deg, qvel = 0{vel_txt}")
            report.note(f"ankle held at its keyframe value "
                        f"{math.degrees(self.ankle_hold):.3f} deg by its own authored "
                        f"kp={self.ankle.kp:g}/kv={self.ankle.kd:g} servo, untouched")
        return self.ankle_hold

    # ------------------------------------------------------------------- ONE STEP
    def step(self, q_ref: float, k: int = -1) -> StepState:
        """Advance the bench by one timestep (0.5 ms) toward the reference angle q_ref.

        This is the whole experiment.  The knee command is the reference clipped into
        the joint ROM; MuJoCo's position actuator turns it into
        Kp*(ctrl - q) - Kd*qdot, clamps that to forcerange, and integrates.
        """
        bench, knee = self.bench, self.knee
        model, data = bench.model, bench.data
        S = bench.sensors

        cmd = knee.command(q_ref)                              # rad, clipped to the ROM
        data.ctrl[bench.knee_act] = cmd
        data.ctrl[bench.ankle_act] = self.ankle.command(self.ankle_hold)
        mujoco.mj_step(model, data)                            # <-- the physics

        q = float(data.sensordata[S["knee_q"]])                # rad
        qd = float(data.sensordata[S["knee_qd"]])              # rad/s
        tau = float(data.sensordata[S["knee_tau"]])            # N.m, after the clamp
        tau_req = knee.torque_unclamped(q_ref, q, qd)          # N.m, before the clamp
        a_q = float(data.sensordata[S["ankle_q"]])
        self.ankle_dev = max(getattr(self, "ankle_dev", 0.0), abs(a_q - self.ankle_hold))

        return StepState(
            k=k, time_s=float(data.time),
            q_ref=float(q_ref), command=cmd,
            command_clamped=knee.command_clamped(q_ref),
            q=q, qdot=qd, tau=tau,
            tau_actuator=float(data.actuator_force[bench.knee_act]),
            tau_unclamped=tau_req,
            saturated=abs(tau_req) > knee.torque_limit + 1e-6,
            ankle_q=a_q, ankle_tau=float(data.sensordata[S["ankle_tau"]]),
        )

    # ------------------------------------------------------------------ whole runs
    def run(self, ref_rad, n: int | None = None, ref_vel0: float | None = None,
            report=None, on_step=None) -> RunResult:
        """Seed, then step through every sample of `ref_rad`.

        on_step(state) is called after each step for live consumers (viewer, dashboard);
        it may only READ the state -- it cannot change the command.
        """
        ref = np.asarray(ref_rad, float)
        n = int(n if n is not None else len(ref))
        self.reset(float(ref[0]), ref_vel0, report)
        self.ankle_dev = 0.0

        out = RunResult(n, ref[:n], self.knee.torque_limit)
        for k in range(n):
            st = self.step(float(ref[k]), k)
            out.store(k, st)
            if on_step is not None:
                on_step(st)
        out.ankle_dev_rad = self.ankle_dev
        out.final_time_s = float(self.bench.data.time)
        return out

    def run_sweep(self, ref_rad, dt: float | None = None) -> RunResult:
        """The gain sweep's run: identical stepping, over a synthetic reference array.

        Kept as a separate name only for readability at the call site -- it is `run`
        with no reporting, so the sweep and the benchmark cannot diverge.
        """
        return self.run(ref_rad)
