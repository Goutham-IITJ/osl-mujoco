"""Typed adapter for path C; no changes to scheduling or conversions."""
from oslbench.contracts import finite, Sample, RobotState, CurrentState, CurrentCommand, layer_current


class GaitContractAdapter:
    """Used by the production runner; original source owns all scheduling/math.

    Angles use the existing positive MuJoCo knee/shaft conventions. Times are
    relative simulation-clock seconds; no wall time or measured current is implied.
    """
    def __init__(self, source, layer):
        finite(source.dt)
        if source.dt <= 0 or layer.substeps != 1:
            raise ValueError("Adapter requires positive dt and one layer substep")
        self.source, self.layer = source, layer
        if hasattr(source, "layer") and source.layer is not layer:
            raise ValueError("Feedback and plant must share the same layer")
        self.state = None
        self.currents = None

    def __call__(self, k, q, qd):
        t = max(k, 0) * self.source.dt
        def s(v, unit, frame):
            return Sample(float(v), unit, frame, t, "pre_step")
        state = RobotState(s(q, "rad", "knee_joint"), s(qd, "rad/s", "knee_joint"),
                           s(self.layer.theta_a, "rad", "actuator_output"),
                           s(self.layer.theta_a_dot, "rad/s", "actuator_output"))
        return layer_current(self.update(k, state))

    def update(self, k, state):
        if not isinstance(state, RobotState):
            raise TypeError("Expected RobotState")
        t = max(k, 0) * self.source.dt
        signals = (state.knee_angle, state.knee_velocity,
                   state.actuator_output_angle, state.actuator_output_velocity)
        if any(not s.valid or s.source != "simulation" or s.stage != "pre_step" or s.time_s != t for s in signals):
            raise ValueError("Controller needs valid synchronous simulation pre-step state")
        if (state.actuator_output_angle.value != self.layer.theta_a or
                state.actuator_output_velocity.value != self.layer.theta_a_dot):
            raise ValueError("State must describe the feedback layer being driven")
        self.state = state
        i_layer = self.source(k, state.knee_angle.value, state.knee_velocity.value)
        last = self.source.last
        held_t = (max(k, 0) // self.source.steps_per_update) * self.source.steps_per_update * self.source.dt
        self.currents = CurrentState(
            CurrentCommand("knee", last["current_request_A"], "requested", held_t),
            CurrentCommand("knee", last["current_applied_A"], "applied", held_t),
            CurrentCommand("knee", last["current_actual_modelled_A"], "modelled", t),
            CurrentCommand("knee", i_layer, "layer_rotor_equivalent", t))
        return self.currents.layer_rotor_equivalent

    def post_step(self, k, q, qd, record):
        """Capture actual runtime samples without recomputing forces or deflection."""
        t = max(k, 0) * self.source.dt
        def s(v, unit, frame, post=False):
            return Sample(float(v), unit, frame, t + self.source.dt if post else t,
                          "post_step" if post else "step_input")
        self.state = RobotState(s(q, "rad", "knee_joint", True),
            s(qd, "rad/s", "knee_joint", True),
            s(record.theta_a, "rad", "actuator_output", True),
            s(record.theta_a_dot, "rad/s", "actuator_output", True),
            s(record.tau_m, "Nm", "actuator_output"),
            s(record.tau_j, "Nm", "knee_joint"),
            s(record.theta_s, "rad", "knee_joint"))
        return self.state
