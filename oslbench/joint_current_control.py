"""Compatibility import; implementation: oslbench.control.gait."""
from oslbench._compat import alias as _alias
_impl = _alias(__name__, "oslbench.control.gait")
globals().update({k: v for k, v in vars(_impl).items() if not k.startswith("__")})
