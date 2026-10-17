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

    # Extra suites, collected into the SAME run so that one command covers the repo.
    # `test_drivetrain` covers oslbench/drivetrain.py (the paper's equations, signs and
    # frames); `test_drivetrain_sim` guards the Option-C feedback point, where the
    # derivative term must read actuator-side velocity rather than joint velocity --
    # reading joint velocity is the non-collocated law that diverged on real MuJoCo at
    # t = 0.3485 s.  Both belong in the command that actually gets run, not in files
    # somebody has to remember.  Import failures are reported as a failed pseudo-test
    # rather than swallowed: a suite that silently stops being collected is exactly the
    # thing this is meant to prevent.
    extra, extra_errors = [], []
    for mod_name in ("test_drivetrain", "test_drivetrain_sim", "test_step_response",
                     "test_ankle_current_step", "test_joint_current_control",
                     "test_gait_feedback_location_ab", "test_gait_feedback_location_estimator",
                     "test_current_gait_live"):
        try:
            extra.append((mod_name, __import__(mod_name)))
        except Exception as exc:                                # noqa: BLE001
            extra_errors.append((mod_name, exc, traceback.format_exc()))

    names = [n for n in dir(T) if n.startswith("test_")]
    names.sort(key=lambda n: getattr(T, n).__code__.co_firstlineno)
    if args.k:
        names = [n for n in names if args.k in n]

    # (module, test name) pairs, so a name collision between suites cannot hide a test.
    selected = [(T, n) for n in names]
    for mod_name, mod in extra:
        enames = [n for n in dir(mod) if n.startswith("test_")]
        enames.sort(key=lambda n: getattr(mod, n).__code__.co_firstlineno)
        if args.k:
            enames = [n for n in enames if args.k in n]
        selected += [(mod, n) for n in enames]

    print("=" * 78)
    print("OSL V2 BENCH -- TESTS")
    print(f"  engine   : {'REAL MuJoCo' if not T.USING_STUB else 'tests/stub_mujoco.py'}")
    print(f"  suites   : test_oslbench"
          + "".join(f" + {m}" for m, _ in extra))
    print(f"  selected : {len(selected)} test(s)" + (f"  (-k {args.k})" if args.k else ""))
    print("=" * 78)

    passed, failed, skipped, t0 = 0, [], [], time.time()
    for mod_name, exc, tb in extra_errors:
        failed.append((f"{mod_name} (import)", exc, tb))
        print(f"  ERROR {mod_name} could not be imported\n"
              f"          {type(exc).__name__}: {exc}")
    for mod, n in selected:
        fn = getattr(mod, n)
        label = n[5:].replace("_", " ")
        # Qualify the name for the summary and the -v dump.  Execution was already
        # collision-proof (selected holds (module, name) pairs), but `failed` used to
        # store the bare name, so two suites sharing a test name would have produced an
        # ambiguous failure line -- which is the moment you least want ambiguity.
        qual = f"{mod.__name__}.{n}"
        # Each suite defines its own Skip; test_oslbench's is not test_drivetrain's, so
        # a module's own class must be used or its skips would be reported as ERRORs.
        skip_exc = getattr(mod, "Skip", T.Skip)
        try:
            fn()
        except skip_exc as e:
            skipped.append((qual, str(e)))
            print(f"  SKIP  {label}\n          {e}")
        except AssertionError as e:
            failed.append((qual, e, traceback.format_exc()))
            print(f"  FAIL  {label}")
            for line in str(e).splitlines():
                print(f"          {line}")
        except Exception as e:                                  # noqa: BLE001
            failed.append((qual, e, traceback.format_exc()))
            print(f"  ERROR {label}\n          {type(e).__name__}: {e}")
        else:
            passed += 1
            print(f"  ok    {label}")

    dt = time.time() - t0
    print("=" * 78)
    print(f"{passed} passed, {len(failed)} failed, {len(skipped)} skipped "
          f"in {dt:.2f} s")
    # The counts must account for every selected test plus any import error, or a
    # silently dropped suite would look like a clean run.
    expected = len(selected) + len(extra_errors)
    if passed + len(failed) + len(skipped) != expected:
        print(f"*** ACCOUNTING ERROR: {passed}+{len(failed)}+{len(skipped)} != "
              f"{expected} selected. Some test was neither run nor reported. ***")
        return 1
    if failed:
        print("failed: " + ", ".join(n for n, _e, _tb in failed))
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
