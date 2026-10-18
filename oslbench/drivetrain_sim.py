"""Compatibility import; implementation: oslbench.sim.compliant."""
from oslbench._compat import alias as _alias
_impl = _alias(__name__, "oslbench.sim.compliant")
globals().update({k: v for k, v in vars(_impl).items() if not k.startswith("__")})
