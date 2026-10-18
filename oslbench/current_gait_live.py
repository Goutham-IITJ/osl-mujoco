"""Compatibility import; implementation: oslbench.presentation.current_gait."""
from oslbench._compat import alias as _alias
_impl = _alias(__name__, "oslbench.presentation.current_gait")
globals().update({k: v for k, v in vars(_impl).items() if not k.startswith("__")})
