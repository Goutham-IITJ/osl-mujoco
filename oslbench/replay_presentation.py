"""Compatibility import; implementation: oslbench.presentation.replay."""
from oslbench._compat import alias as _alias
_impl = _alias(__name__, "oslbench.presentation.replay")
globals().update({k: v for k, v in vars(_impl).items() if not k.startswith("__")})
