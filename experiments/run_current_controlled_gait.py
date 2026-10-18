"""Compatibility entry point for the current-gait application."""
from pathlib import Path
import sys
ROOT = str(Path(__file__).resolve().parents[1])
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)
from oslbench.apps.current_gait import *

if __name__ == "__main__":
    raise SystemExit(main())
