"""Compatibility import; implementation: oslbench.sim.ankle."""
from oslbench._compat import alias as _alias
_impl = _alias(__name__, "oslbench.sim.ankle")
globals().update({k: v for k, v in vars(_impl).items() if not k.startswith("__")})
