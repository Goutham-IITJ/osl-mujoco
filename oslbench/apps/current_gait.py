"""Current-gait command-line application; simulation runtime is library-owned."""
from oslbench.paths import ROOT
import os
import sys
if "--backend" in sys.argv[1:] and "stub" in sys.argv[1:]:
    sys.path.insert(0, os.path.join(ROOT, "tests"))
    import stub_mujoco
    stub_mujoco.install()
from oslbench.sim.current_gait import *
from oslbench.presentation.current_gait_plots import plots


def view(ctx, args):
    """Compatibility entry point for the unified current-controlled viewer."""
    from oslbench.presentation.current_gait import run_live
    return run_live(ctx, args)



def main() -> int:
    ap = argparse.ArgumentParser(description=f"{BANNER}. {DISCLAIMER}")
    ap.add_argument("--csv", default=DEFAULT_REFERENCE_CSV)
    ap.add_argument("--kp", type=float, default=0.75, help="A/deg (ankle-script value)")
    ap.add_argument("--ki", type=float, default=0.0, help="A/(deg.s)")
    ap.add_argument("--kd", type=float, default=0.03, help="A/(deg/s)")
    ap.add_argument("--current-loop", choices=("ideal", "first_order"), default="ideal",
                    help="'ideal' (default) isolates the outer loop and drivetrain")
    ap.add_argument("--current-loop-tau-ms", type=float, default=0.0)
    ap.add_argument("--current-clip", type=float, default=60.0,
                    help="SIMULATION-ONLY current clip in A. Not a safety limit.")
    ap.add_argument("--kt", choices=sorted(TORQUE_CONSTANT_CHOICES),
                    default="manual_rotor")
    ap.add_argument("--belt", choices=("cad", "paper"), default="cad")
    ap.add_argument("--derivative-on", choices=("error", "measurement"), default="error")
    ap.add_argument("--feedback-mode", choices=("legacy", "actuator-difference"), default="legacy",
                    help="actuator-difference selects verified Variant C (requires Ki=0 and derivative-on=error)")
    ap.add_argument("--dashboard", choices=("web", "none"), default="web")
    ap.add_argument("--port", type=int, default=8789)
    ap.add_argument("--no-browser", action="store_true")
    ap.add_argument("--start-paused", action="store_true")
    ap.add_argument("--exit-on-complete", action="store_true")
    ap.add_argument("--init", choices=("aligned", "zero"), default="aligned")
    ap.add_argument("--cycles", type=int, default=1)
    ap.add_argument("--backend", choices=("mujoco", "stub"), default="mujoco")
    ap.add_argument("--outdir", default=None)
    ap.add_argument("--tag", default=None, help="override the auto output tag")
    ap.add_argument("--view", action="store_true", help="live MuJoCo viewer (no files)")
    ap.add_argument("--speed", type=float, default=0.5, choices=(0.25, 0.5, 1.0))
    ap.add_argument("--max-cycles", type=int, default=0)
    ap.add_argument("--zoom", type=float, default=0.45)
    ap.add_argument("--elevation", type=float, default=-6.0)
    ap.add_argument("--azimuth", type=float, default=None)
    ap.add_argument("--distance", type=float, default=None)
    ap.add_argument("--no-plot", action="store_true")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    tau = args.current_loop_tau_ms
    if not math.isfinite(tau) or tau < 0.0:
        print(f"REFUSING TO RUN: --current-loop-tau-ms must be finite and >= 0, got {tau!r}")
        return 2
    if args.current_loop == "first_order" and tau == 0.0:
        print("NOTE: --current-loop first_order with tau = 0 ms is identical to 'ideal'.")

    mj, backend, is_stub = require_backend(args.backend)
    print("=" * 78)
    print(f"  {BANNER}")
    print(f"  {DISCLAIMER}")
    if is_stub:
        print("  *** STUB -- NOT MUJOCO. Wiring check only, not a simulation result. ***")
    print("=" * 78)
    print(f"  backend   {backend}")

    ctx = build(args, mj, is_stub)
    if ctx is None:
        return 1
    ident = ctx["ident"]
    print(f"  loaded    {ident['model_path']}  sha256[:16] {ident['model_xml_sha256']}")
    print(f"            nq={ident['nq']} nv={ident['nv']} nu={ident['nu']} "
          f"ngeom={ident['ngeom']} nmesh={ident['nmesh']}  "
          f"dt={ident['timestep_s'] * 1e3:.3f} ms")
    print(f"            knee armature={ident['knee_armature']:g} "
          f"damping={ident['knee_damping']:g} "
          f"frictionloss={ident['knee_frictionloss']:g}  matches the MJCF: "
          f"{ident['knee_properties_match_mjcf']}")
    print(f"  joint     KNEE (qpos {ctx['bench'].knee_qpos}, dof {ctx['bench'].knee_dof}), "
          f"authority +/-{ctx['bench'].force_limit:.1f} N.m")
    print(f"  reference {SUBJECT} knee angle, {ctx['ref'].n} rows, cycle "
          f"{ctx['ref'].period_s:.4f} s, first sample "
          f"{ctx['ref'].angle_deg[0]:.4f} deg -> {ctx['n']} steps")
    for line in ctx["cfg"].describe():
        print(f"  cfg       {line}")
    print(f"  schedule  {CONTROL_HZ:g} Hz controller / {1 / ctx['dt']:g} Hz physics "
          f"-> {ctx['steps_per_update']} step(s) per update, command held")
    print(f"  init      {args.init}")
    print(f"  k_t       {ctx['audit'].k_t_output_Nm_per_A:.6f} N.m/A at the ACTUATOR "
          f"OUTPUT ({args.kt})  -- WORKING HYPOTHESIS")
    print(f"  belt      n_t {ctx['audit'].belt_ratio:.6f} ({args.belt})")

    if args.view:
        if is_stub:
            print("\nREFUSING TO VIEW: the stub has no renderer. Use real MuJoCo.")
            return 1
        view(ctx, args)
        return 0

    rows = run(ctx, args)
    m = metrics(rows, ctx, args)
    loop_mode = "ideal" if ctx["cfg"].current_loop_tau_s == 0.0 else "first_order"
    md = metadata(ctx, args, loop_mode)
    m.update({f"meta_{k}": v for k, v in md.items()})

    tag = args.tag or (
        f"kp{args.kp:g}_kd{args.kd:g}_{loop_mode}"
        + (f"_tau{tau:g}ms" if loop_mode != "ideal" else "")
        + f"_{args.kt}_{args.belt}_{args.init}"
        + ("_C" if args.feedback_mode == "actuator-difference" else "") + ("_STUB" if is_stub else ""))
    outdir = args.outdir or os.path.join(OUTROOT, tag)
    if os.path.exists(outdir) and not args.outdir:
        i = 2
        while os.path.exists(f"{outdir}_{i}"):
            i += 1
        outdir = f"{outdir}_{i}"
        print(f"  note      output directory existed; writing to {os.path.basename(outdir)} "
              f"instead of overwriting")

    print("-" * 78)
    print(f"  servo     knee position actuator contributed "
          f"{m['knee_servo_nonzero_steps']} nonzero force(s) over {len(rows)} steps, "
          f"peak |force| {m['knee_servo_peak_abs_force_Nm']:.3e} N.m")
    if m["knee_servo_nonzero_steps"]:
        print("\nREFUSING TO REPORT METRICS: the knee servo acted, so the servo and the "
              "belt both drove the joint and every number would be a double count.")
        return 1
    print(f"            that zero is the proof the belt is the only actuation")
    print(f"  stable    numerically stable: {m['numerically_stable']}   "
          f"inside authored ROM: {m['stayed_inside_authored_rom']} "
          f"({m['knee_min_deg']:.2f}..{m['knee_max_deg']:.2f} deg)")
    print("\n  TRACKING")
    for k in ("rms_error_deg", "rms_error_deg_after_50ms", "mae_error_deg",
              "peak_abs_error_deg", "final_error_deg", "best_fit_lag_ms",
              "rms_error_deg_lag_removed", "pct_error_from_lag"):
        print(f"    {k:38s} {m[k]:.6g}")
    print("  CURRENT   (commanded / modelled; never measured)")
    for k in ("peak_current_request_A", "peak_current_applied_A",
              "peak_current_actual_modelled_A", "saturation_duration_s",
              "saturation_pct", "peak_applied_current_envelope"):
        v = m[k]
        print(f"    {k:38s} {v:.6g}" if isinstance(v, float) else f"    {k:38s} {v}")
    print("  TORQUE    (frame in the name; never compare across frames)")
    for k in ("peak_implied_rotor_torque_Nm",
              "peak_estimated_actuator_output_torque_Nm",
              "peak_belt_torque_ACTUATOR_side_Nm", "peak_belt_torque_JOINT_side_Nm",
              "pct_of_knee_authority", "max_abs_theta_s_rad",
              "theta_s_as_multiple_of_paper_fit_edge"):
        print(f"    {k:38s} {m[k]:.6g}")

    p_csv, p_met, p_md = write_outputs(outdir, rows, m, md, is_stub)
    pl = [] if args.no_plot else plots(outdir, rows, m, md, ctx, is_stub)
    print("\n  outputs")
    for p in [p_csv, p_met, p_md] + pl:
        print(f"    {os.path.relpath(p, ROOT)}")

    ok = ctx["bench"].limits_unchanged()
    print("-" * 78)
    print(f"  [F] authored knee forcerange/ctrlrange unchanged: {ok}; the MJCF was "
          f"never opened for writing")
    print("\n  HOW TO READ THIS")
    print("  This is a CURRENT-domain controller. run_bench_ab19.py is a TORQUE-domain")
    print("  position servo at 600 N.m/rad. Comparing their RMS errors compares two")
    print("  control PATHWAYS, not two tunings. The gains here came from the ANKLE")
    print("  step-test script and are NOT hardware-confirmed for the knee.")
    print(f"  {DISCLAIMER}")
    if is_stub:
        print("  *** STUB -- NOT MUJOCO. Re-run in .venv for a simulation result. ***")
    return 0 if ok else 1



if __name__ == "__main__":
    raise SystemExit(main())
