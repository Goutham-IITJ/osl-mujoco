"""Compatibility import; implementation: oslbench.telemetry.imu_schema."""
from oslbench._compat import alias as _alias
_impl = _alias(__name__, "oslbench.telemetry.imu_schema")
globals().update({k: v for k, v in vars(_impl).items() if not k.startswith("__")})
