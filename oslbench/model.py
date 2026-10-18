"""Compatibility import; implementation: oslbench.sim.model."""
from oslbench._compat import alias as _alias
_impl = _alias(__name__, "oslbench.sim.model")
globals().update({k: v for k, v in vars(_impl).items() if not k.startswith("__")})
