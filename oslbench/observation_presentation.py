"""Compatibility import; implementation: oslbench.presentation.observations."""
from oslbench._compat import alias as _alias
_impl = _alias(__name__, "oslbench.presentation.observations")
globals().update({k: v for k, v in vars(_impl).items() if not k.startswith("__")})
