"""Small simulation contracts; no hardware semantics or new control equations."""
from dataclasses import dataclass
import math


def finite(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError("Expected a finite numeric value")


FRAMES = ("knee_joint", "actuator_output", "rotor")
CURRENTS = ("requested", "applied", "modelled", "layer_rotor_equivalent")


@dataclass(frozen=True)
class Sample:
    value: float
    unit: str
    frame: str
    time_s: float
    stage: str
    source: str = "simulation"
    valid: bool = True

    def __post_init__(self):
        finite(self.value)
        finite(self.time_s)
        # Additive read-only orientation observations; existing robot/command
        # contracts below still require their original units and frames.
        if self.frame == "imu_filter_ned" and self.stage == "acquisition":
            if self.time_s < 0 or self.unit not in ("rad", "1"):
                raise ValueError("Invalid orientation time or unit")
            if self.source not in ("measured", "synthetic_mock") or type(self.valid) is not bool:
                raise ValueError("Explicit orientation provenance and validity required")
            return
        if self.time_s < 0 or self.unit not in ("rad", "rad/s", "Nm", "A"):
            raise ValueError("Invalid time or unit")
        if self.frame not in FRAMES or self.stage not in ("pre_step", "post_step", "held_command", "step_input"):
            raise ValueError("Unknown frame or acquisition stage")
        if not isinstance(self.source, str) or not self.source.strip() or type(self.valid) is not bool:
            raise ValueError("Explicit source and boolean validity required")


@dataclass(frozen=True)
class RobotState:
    knee_angle: Sample
    knee_velocity: Sample
    actuator_output_angle: Sample
    actuator_output_velocity: Sample
    electromagnetic_output_torque: Sample | None = None
    belt_joint_torque: Sample | None = None
    belt_deflection: Sample | None = None

    def __post_init__(self):
        schema = (("knee_angle", "rad", "knee_joint"),
                  ("knee_velocity", "rad/s", "knee_joint"),
                  ("actuator_output_angle", "rad", "actuator_output"),
                  ("actuator_output_velocity", "rad/s", "actuator_output"),
                  ("electromagnetic_output_torque", "Nm", "actuator_output"),
                  ("belt_joint_torque", "Nm", "knee_joint"),
                  ("belt_deflection", "rad", "knee_joint"))
        for name, unit, frame in schema:
            sample = getattr(self, name)
            if sample is None and name in ("electromagnetic_output_torque", "belt_joint_torque", "belt_deflection"):
                continue
            if not isinstance(sample, Sample) or (sample.unit, sample.frame) != (unit, frame):
                raise ValueError("Incompatible state field: " + name)


@dataclass(frozen=True)
class PositionCommand:
    target: str
    position_rad: float
    coordinate: str

    def __post_init__(self):
        finite(self.position_rad)
        if self.target != "knee" or self.coordinate not in FRAMES:
            raise ValueError("Unsupported target/coordinate")


@dataclass(frozen=True)
class TorqueCommand:
    target: str
    torque_Nm: float
    application_frame: str

    def __post_init__(self):
        finite(self.torque_Nm)
        if self.target != "knee" or self.application_frame not in FRAMES:
            raise ValueError("Unsupported target/application frame")


@dataclass(frozen=True)
class CurrentCommand:
    target: str
    current_A: float
    convention: str
    time_s: float

    def __post_init__(self):
        finite(self.current_A)
        finite(self.time_s)
        if self.target != "knee" or self.convention not in CURRENTS or self.time_s < 0:
            raise ValueError("Unsupported current target/convention/time")


@dataclass(frozen=True)
class CurrentState:
    requested: CurrentCommand
    applied: CurrentCommand
    modelled: CurrentCommand
    layer_rotor_equivalent: CurrentCommand
    source: str = "simulation"

    def __post_init__(self):
        for name in CURRENTS:
            c = getattr(self, name)
            if not isinstance(c, CurrentCommand) or c.convention != name:
                raise ValueError("Current stages cannot be interchanged")
        if self.source != "simulation":
            raise ValueError("This adapter only models simulated current")


def layer_current(command):
    """Compatibility boundary: consume only already-converted rotor current.

    Deliberately performs NO ratio or torque-constant conversion.
    """
    if not isinstance(command, CurrentCommand):
        raise TypeError("Drivetrain boundary requires CurrentCommand")
    if command.target != "knee" or command.convention != "layer_rotor_equivalent":
        raise ValueError("Expected layer-equivalent knee rotor current")
    return command.current_A


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
