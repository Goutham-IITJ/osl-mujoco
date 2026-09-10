#!/usr/bin/env python3
"""
run_tests.py -- run the bench experiment's tests.  No pytest, no dependencies but numpy.

WHY THERE IS NO TEST FRAMEWORK HERE
    The suite is one file of plain `assert` statements.  A runner that finds functions
    named test_*, calls them, and prints one line each is about forty lines of code, and
    it means a reviewer can run the tests in whatever Python they already have -- there
    is nothing to install and nothing to configure.

USAGE (Windows PowerShell)
    python tests\\run_tests.py                        numpy only; uses tests/stub_mujoco.py
    .venv\\Scripts\\python.exe tests\\run_tests.py    real MuJoCo (what you should trust)
    .venv\\Scripts\\python.exe tests\\run_tests.py -v full traceback for each failure
    .venv\\Scripts\\python.exe tests\\run_tests.py -k controller   run a subset

WHAT A GREEN RUN DOES AND DOES NOT PROVE
    It proves the wiring and the algebra: the control law, the two saturations, the
    reference contract, the metric definitions, and that the model XML and the AB19 CSV
    are byte-for-byte unchanged.

    It does NOT prove the validated benchmark numbers.  That is
    experiments/verify_against_oracle.py, which needs real MuJoCo and compares a fresh
    run against the frozen tests/oracle/ result.

EXIT CODE
    0 = every test passed.  Otherwise the number of failures.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for p in (ROOT, HERE):
    if p not in sys.path:
        sys.path.insert(0, p)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-v", "--verbose", action="store_true",
                    help="print the full traceback for each failure")
    ap.add_argument("-k", default=None, metavar="SUBSTRING",
                    help="only run tests whose name contains this")
    args = ap.parse_args()

    import test_oslbench as T

    names = [n for n in dir(T) if n.startswith("test_")]
    names.sort(key=lambda n: getattr(T, n).__code__.co_firstlineno)
    if args.k:
        names = [n for n in names if args.k in n]

    print("=" * 78)
    print("OSL V2 BENCH -- TESTS")
    print(f"  engine   : {'REAL MuJoCo' if not T.USING_STUB else 'tests/stub_mujoco.py'}")
    print(f"  selected : {len(names)} test(s)" + (f"  (-k {args.k})" if args.k else ""))
    print("=" * 78)

    passed, failed, skipped, t0 = 0, [], [], time.time()
    for n in names:
        fn = getattr(T, n)
        label = n[5:].replace("_", " ")
        try:
            fn()
        except T.Skip as e:
            skipped.append((n, str(e)))
            print(f"  SKIP  {label}\n          {e}")
        except AssertionError as e:
            failed.append((n, e, traceback.format_exc()))
            print(f"  FAIL  {label}")
            for line in str(e).splitlines():
                print(f"          {line}")
        except Exception as e:                                  # noqa: BLE001
            failed.append((n, e, traceback.format_exc()))
            print(f"  ERROR {label}\n          {type(e).__name__}: {e}")
        else:
            passed += 1
            print(f"  ok    {label}")

    dt = time.time() - t0
    print("=" * 78)
    print(f"{passed} passed, {len(failed)} failed, {len(skipped)} skipped "
          f"in {dt:.2f} s")
    if args.verbose and failed:
        for n, _e, tb in failed:
            print("-" * 78 + f"\n{n}\n" + tb, end="")
    if failed:
        print("\nthe wiring is broken -- fix this before running the experiment")
    elif T.USING_STUB:
        print("\nNOTE: this ran on the stub plant, so the numbers here are not the\n"
              "      validated result. For that, in .venv:\n"
              "      .venv\\Scripts\\python.exe experiments\\verify_against_oracle.py")
    print("=" * 78)
    return len(failed)


if __name__ == "__main__":
    sys.exit(main())
