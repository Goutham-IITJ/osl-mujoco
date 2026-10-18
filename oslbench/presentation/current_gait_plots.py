"""Plots saved current-gait output; never advances a simulation."""
import os
import numpy as np
from oslbench.sim.current_gait import BANNER, CSV_HEADER, DISCLAIMER, FIT_EDGE_RAD
from oslbench.reference import STANCE_END


def plots(outdir, rows, m, md, ctx, is_stub):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:                                    # noqa: BLE001
        print(f"  [plot] skipped: matplotlib unavailable ({type(exc).__name__}: {exc})")
        return []
    col = {c: i for i, c in enumerate(CSV_HEADER)}

    def f(n):
        return np.asarray([float(r[col[n]]) for r in rows], float)

    ph, t = f("gait_phase_pct"), f("time_s")
    sub = (f"Kp {md['kp_A_per_deg']:g} A/deg, Kd {md['kd_A_per_deg_per_s']:g} "
           f"A/(deg/s), current loop {md['current_loop_mode']}"
           + (f" tau_i {md['current_loop_tau_ms']:g} ms"
              if md["current_loop_mode"] != "ideal" else "")
           + f", k_t {md['k_t_output_choice']}, belt {md['belt_ratio_n_t']:.4f}")
    head = ("*** STUB -- NOT MUJOCO *** " if is_stub else "") + BANNER
    made = []

    def finish(fig, ax, name, ylab, title):
        import textwrap
        for a in (ax if isinstance(ax, (list, np.ndarray)) else [ax]):
            a.grid(alpha=0.25, lw=0.6)
            a.axvspan(0, STANCE_END, color="#ececeb", zorder=0)
        subtitle = '\n'.join(textwrap.wrap(sub, width=110))
        fig.suptitle(f"{head}\n{title}\n{subtitle}\n{DISCLAIMER}", fontsize=9,
                     fontweight="bold", color="#9a2b11" if is_stub else "black")
        fig.tight_layout(rect=(0, 0, 1, 0.84))
        p = os.path.join(outdir, name)
        fig.savefig(p, dpi=130)
        plt.close(fig)
        made.append(p)

    fig, ax = plt.subplots(figsize=(11, 4.6))
    ax.plot(ph, f("reference_deg"), color="#9a9aa4", lw=1.6, label="AB19 knee reference")
    ax.plot(ph, f("knee_angle_deg"), color="#1f4e8c", lw=1.4, label="simulated knee")
    ax.set_xlabel("gait phase [%]  (shaded 0-60 % = human stance, orientation only)")
    ax.set_ylabel("knee flexion [deg]")
    ax.legend(fontsize=8)
    finish(fig, ax, "gait_reference_vs_actual.png", "", "1  Reference vs actual knee angle")

    fig, ax = plt.subplots(figsize=(11, 4.0))
    ax.axhline(0, color="#b4b4ac", lw=0.8)
    ax.plot(ph, f("tracking_error_deg"), color="#9a2b11", lw=1.3)
    ax.set_xlabel("gait phase [%]")
    ax.set_ylabel("Tracking error [deg]\nreference - measured")
    ax.set_title(f"RMS {m['rms_error_deg']:.4f} deg | peak {m['peak_abs_error_deg']:.4f} "
                 f"deg | lag {m['best_fit_lag_ms']:.1f} ms | lag-removed RMS "
                 f"{m['rms_error_deg_lag_removed']:.4f} deg", fontsize=9)
    finish(fig, ax, "gait_tracking_error.png", "", "2  Tracking error (reference - measured)")

    fig, ax = plt.subplots(figsize=(11, 4.4))
    ax.plot(ph, f("current_request_A"), color="#8c4a1f", lw=0.9,
            label="PID request, pre-clip [A]")
    ax.plot(ph, f("current_applied_A"), color="#1f4e8c", lw=1.3,
            label="applied (clipped) command [A]")
    ax.plot(ph, f("current_actual_modelled_A"), color="#2c6e49", lw=1.1, ls="--",
            label="modelled actual, after the current loop [A]")
    for v, lab, c in ((md["current_clip_A"], "sim clip", "#9a2b11"),
                      (12.0, "reported rated", "#2c6e49"),
                      (28.0, "reported peak", "#c08a2e")):
        for s in (+1, -1):
            ax.axhline(s * v, color=c, lw=0.8, ls=":", label=lab if s > 0 else None)
    ax.set_xlabel("gait phase [%]")
    ax.set_ylabel("Current [A]\ncommanded / modelled, not measured")
    ax.legend(fontsize=7.5, ncol=3)
    finish(fig, ax, "gait_current.png", "", "3  Current: request, applied, modelled actual")

    fig, ax = plt.subplots(figsize=(11, 4.4))
    ax.plot(ph, f("belt_torque_actuator_side_Nm"), color="#8c4a1f", lw=1.1,
            label="belt torque, ACTUATOR side [N.m]")
    ax.plot(ph, f("estimated_actuator_output_torque_Nm"), color="#c08a2e", lw=0.9, ls=":",
            label="I_q x k_t_output = ESTIMATED actuator-output torque [N.m]")
    ax.set_xlabel("gait phase [%]")
    ax.set_ylabel("actuator-side torque [N.m]")
    ax.legend(fontsize=7.5, loc="upper left")
    axb = ax.twinx()
    axb.plot(ph, f("belt_torque_joint_side_Nm"), color="#1f4e8c", lw=1.4,
             label="belt torque, JOINT side [N.m]")
    axb.set_ylabel("joint-side torque [N.m]", color="#1f4e8c")
    axb.legend(fontsize=7.5, loc="lower right")
    finish(fig, ax, "gait_torque.png", "",
           "4  Torque, actuator side and joint side (separate scales, they differ by n_t)")

    ths = f("theta_s_rad")
    pk = max(float(np.max(np.abs(ths))), 1e-9)
    fig, ax = plt.subplots(figsize=(11, 4.0))
    near = pk > 0.2 * FIT_EDGE_RAD
    lim = max(FIT_EDGE_RAD * 1.4, pk * 1.2) if near else pk * 1.3
    if near:
        ax.axhspan(FIT_EDGE_RAD, lim, color="#efe3c8",
                   label=f"|theta_s| > {FIT_EDGE_RAD:g} rad = outside the paper's fit")
        ax.axhspan(-lim, -FIT_EDGE_RAD, color="#efe3c8")
    ax.plot(ph, ths, color="#2c6e49", lw=1.3,
            label=f"belt deflection theta_s [rad], peak {pk:.3e}"
                  + ("" if near else f"  ({pk / FIT_EDGE_RAD:.3f}x the fit edge, "
                                     f"off-scale here)"))
    ax.set_ylim(-lim, lim)
    ax.set_xlabel("gait phase [%]")
    ax.set_ylabel("belt deflection [rad]")
    ax.legend(fontsize=7.5)
    finish(fig, ax, "gait_belt_deflection.png", "", "5  Belt deflection")
    return made
