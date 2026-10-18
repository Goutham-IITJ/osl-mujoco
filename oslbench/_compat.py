"""Import aliases for historical module paths; no duplicated implementation."""
from importlib import import_module
import sys


def alias(name, target):
    module = import_module(target)
    sys.modules[name] = module
    return module
