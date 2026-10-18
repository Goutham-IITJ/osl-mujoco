"""Stable repository-relative assets and recursive Python provenance inventory."""
from pathlib import Path

ROOT = str(Path(__file__).resolve().parents[1])


def source_files(root=ROOT):
    """Include canonical nested implementations and compatibility entry points."""
    root = Path(root)
    return tuple(sorted(p for folder in ("oslbench", "experiments")
                        for p in (root / folder).rglob("*.py")
                        if "__pycache__" not in p.parts))
