"""Compatibility import; implementation: oslbench.presentation.telemetry."""
from oslbench._compat import alias as _alias
_impl = _alias(__name__, "oslbench.presentation.telemetry")
globals().update({k: v for k, v in vars(_impl).items() if not k.startswith("__")})
