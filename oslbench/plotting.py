"""
oslbench.plotting -- OFFLINE figures, built from a logged CSV.

This module never runs MuJoCo and never computes physics.  It opens a CSV that a
completed run wrote, and draws it.  That separation is not cosmetic: it is why the
figures can be produced in a different Python environment from the simulation.

WHY TWO ENVIRONMENTS
    osl-mujoco's `.venv` has mujoco + numpy and NO matplotlib (and must not be
    modified).  `.venv-analysis` has matplotlib and no mujoco.  So the run writes a CSV
    from `.venv`, and `experiments/plot_bench_results.py` draws it from `.venv-analysis`.
    Nothing is recomputed in between -- the figures are a rendering of the recorded
    numbers, not a second simulation.

THE SIX PANELS (all against % gait cycle, stance 0-60 % shaded grey, swing blue)
    A  human reference vs simulated knee angle
    B  tracking error
    C  BENCH ACTUATOR torque, with the % of authority used
    D  reference vs simulated angular velocity
    E  actuator mechanical power, tau * omega
    F  reference (pre-clamp) vs commanded ctrl (post-clamp)

The human moment/power columns get a SEPARATE figure, deliberately not sharing an axis
with any bench signal, carrying the reference audit flags.
"""

from __future__ import annotations

import os
import sys
import textwrap

import numpy as np

from .logging import (BENCH_CSV_HEADER, DRIVETRAIN_EXTRA_HEADER,
                      read_bench_csv)
from .reference import STANCE_END, SUBJECT, TRIAL

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
KNEE_FORCE_LIMIT_NM = 142.2       # authored in models/osl_v2_bench.xml, for the % box


def _shade(ax):
    """Stance/swing shading, grid and x-axis, identical on every panel in this module.

    Extracted from `plot_bench_results`, where it was a closure, so that a second
    figure cannot drift to a different stance boundary or a different x-limit than the
    first.  The gait-phase convention is one fact and belongs in one place.
    """
    ax.axvspan(0, STANCE_END, color="0.55", alpha=0.13, lw=0, zorder=0)
    ax.axvspan(STANCE_END, 100, color="#2980b9", alpha=0.07, lw=0, zorder=0)
    ax.set_xlim(0, 100)
    ax.grid(alpha=0.3)
    ax.set_xlabel("gait phase (%)")


def _captions(axes):
    """Add headroom, then the stance/swing captions, on a flat iterable of axes."""
    for a in axes:
        lo, hi = a.get_ylim()
        a.set_ylim(lo, hi + 0.13 * (hi - lo))
        a.text(0.30, 0.985, "stance 0-60 %", transform=a.transAxes, ha="center",
               va="top", fontsize=8.5, color="0.30")
        a.text(0.80, 0.985, "swing 60-100 %", transform=a.transAxes, ha="center",
               va="top", fontsize=8.5, color="#1f618d")


def plot_bench_results(path_csv: str, outdir: str, kp: float, kv: float,
                       flags: list[str] | None = None,
                       force_limit: float = KNEE_FORCE_LIMIT_NM):
    """Draw both figures from a bench_track_ab19 CSV.  Returns (main_png, human_png)."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("\n[P] matplotlib not available in this venv -- CSV written, plots skipped.")
        print("    Regenerate them from the CSV in the analysis venv:")
        print(f"      .venv-analysis\\Scripts\\python.exe experiments\\"
              f"plot_bench_results.py {os.path.relpath(path_csv, ROOT)}")
        return None, None

    flags = flags or []
    C = read_bench_csv(path_csv)
    missing = [c for c in BENCH_CSV_HEADER if c not in C]
    if missing:
        sys.exit(f"FATAL: {path_csv} is missing columns {missing}")
    ph = C["gait_phase_percent"]
    o = np.argsort(ph, kind="stable")            # in case >1 cycle was logged
    shade = _shade

    fig, ax = plt.subplots(2, 3, figsize=(16.5, 9))

    a = ax[0, 0]; shade(a)
    a.plot(ph[o], C["human_knee_angle_deg"][o], "k--", lw=2.4,
           label="AB19 human reference")
    a.plot(ph[o], C["sim_deg"][o], color="#27ae60", lw=1.6, label="OSL bench simulated")
    a.set_ylabel("knee flexion (deg)")
    a.set_title("A. human reference vs simulated knee angle")
    a.legend(fontsize=8, loc="center left")

    a = ax[0, 1]; shade(a)
    a.plot(ph[o], C["err_deg"][o], color="#c0392b", lw=1.4)
    a.axhline(0, color="k", lw=0.8)
    a.set_ylabel("simulated - reference (deg)")
    a.set_title("B. tracking error vs gait phase")

    a = ax[0, 2]; shade(a)
    a.plot(ph[o], C["tau_sensor_Nm"][o], color="#8e44ad", lw=1.4)
    a.axhline(0, color="k", lw=0.8)
    a.set_ylabel("bench actuator torque (N.m)")
    a.set_title("C. simulated knee torque vs gait phase")
    lim = float(force_limit)
    pk_t = float(np.max(np.abs(C["tau_sensor_Nm"])))
    a.text(0.02, 0.02, f"peak {pk_t:.2f} N.m = {100 * pk_t / lim:.1f} % of "
                       f"+/-{lim:.1f} N.m authority",
           transform=a.transAxes, fontsize=8, va="bottom",
           bbox=dict(boxstyle="round,pad=0.3", fc="#f9f0fb", ec="#8e44ad", lw=0.8))

    a = ax[1, 0]; shade(a)
    a.plot(ph[o], C["ref_vel_rad_s"][o], "k--", lw=1.6, label="reference")
    a.plot(ph[o], C["sim_vel_rad_s"][o], color="#e67e22", lw=1.4, label="simulated")
    a.axhline(0, color="k", lw=0.8)
    a.set_ylabel("knee angular velocity (rad/s)")
    a.set_title("D. knee angular velocity vs gait phase")
    a.legend(fontsize=8, loc="lower left")

    a = ax[1, 1]; shade(a)
    a.plot(ph[o], C["knee_power_W"][o], color="#16a085", lw=1.4)
    a.axhline(0, color="k", lw=0.8)
    a.set_ylabel("actuator mechanical power (W)")
    a.set_title("E. simulated knee power (torque x omega) vs gait phase")

    a = ax[1, 2]; shade(a)
    # reference drawn thick UNDERNEATH so that "the command equals the reference" is
    # visible as coincidence rather than as one line hiding the other
    a.plot(ph[o], C["ref_deg"][o], color="#e74c3c", lw=3.2, alpha=0.55,
           label="reference (pre-clamp)")
    a.plot(ph[o], np.degrees(C["ctrl_rad"][o]), color="#2c3e50", lw=1.3,
           label="commanded ctrl (post-clamp)")
    a.set_ylabel("actuator command (deg)")
    a.set_title("F. actuator control vs gait phase")
    a.legend(fontsize=8, loc="center left")
    # 1e-4 deg, not 1e-9: ref_deg and ctrl_rad are written to finite precision, so an
    # exact comparison after a CSV round-trip would report false clamping everywhere.
    # The authoritative count is ref_clamped_steps in the metrics CSV, computed in-run.
    nclamp = int(np.sum(np.abs(np.degrees(C["ctrl_rad"]) - C["ref_deg"]) > 1e-4))
    a.text(0.02, 0.02, "command == reference at every step: NO clamping"
           if nclamp == 0 else f"reference CLAMPED at {nclamp} of {len(ph)} steps",
           transform=a.transAxes, fontsize=8, va="bottom",
           bbox=dict(boxstyle="round,pad=0.3",
                     fc="#eafaf1" if nclamp == 0 else "#fdecea",
                     ec="#27ae60" if nclamp == 0 else "#c0392b", lw=0.8))

    # stance/swing captions last, in axes coordinates with headroom made for them, so
    # they cannot be clipped by the spine or hidden behind a legend
    _captions(ax.ravel())

    fig.suptitle(f"OSL V2 CAD bench knee tracking {SUBJECT} -- {TRIAL}\n"
                 f"kp={kp:.0f} N.m/rad, kv={kv:.3f} N.m.s/rad (runtime only; "
                 f"models/osl_v2_bench.xml unchanged)", fontsize=11)
    fig.text(0.5, 0.012,
             "Panel C is BENCH ACTUATOR TORQUE, not a human knee moment: the bench is "
             "fixed to the world and carries no body weight, so the actuator opposes "
             "only inertia, gravity on the knee-distal segment,\njoint damping and "
             "friction. The shaded stance region is therefore NOT comparable to human "
             "stance-phase knee kinetics. Only the knee ANGLE is taken from AB19.",
             ha="center", va="bottom", fontsize=8.5,
             bbox=dict(boxstyle="round,pad=0.4", fc="#fdf6e3", ec="#b58900", lw=0.9))
    fig.tight_layout(rect=(0, 0.075, 1, 0.94))
    os.makedirs(outdir, exist_ok=True)
    p1 = os.path.join(outdir, "bench_track_ab19.png")
    fig.savefig(p1, dpi=140)

    # The human kinetic columns get their OWN figure, deliberately not sharing an axis
    # with any bench signal, and carrying the audit warning.
    fig2, ax2 = plt.subplots(1, 2, figsize=(13.4, 6.4))
    for a, key, lab in ((ax2[0], "human_knee_moment", "human knee moment"),
                        (ax2[1], "human_knee_power", "human knee power")):
        shade(a)
        a.plot(ph[o], C[key][o], color="#7f8c8d", lw=1.5)
        a.axhline(0, color="k", lw=0.8)
        a.set_ylabel(f"{lab} (unit UNVERIFIED)")
        a.set_title(f"{lab} -- AB19 reference, retained not used", fontsize=10.5)
    msg = ("Retained for provenance and NOT used to drive or judge the bench. These "
           "are biomechanical human joint\nquantities, a different physical quantity "
           "from the bench actuator torque in panel C of the main figure.")
    if flags:
        msg += "\n\nAUDIT FLAGS -- do not interpret these two curves until resolved:\n"
        msg += "\n".join(textwrap.fill(f, 112, initial_indent="- ",
                                      subsequent_indent="  ") for f in flags[:4])
    fig2.text(0.5, 0.015, msg, ha="center", va="bottom", fontsize=8.2,
              linespacing=1.35,
              bbox=dict(boxstyle="round,pad=0.4", fc="#fdf6e3", ec="#b58900", lw=0.9))
    fig2.suptitle(f"{SUBJECT} human reference kinetics -- separate signals, "
                  f"NOT bench actuator torque", fontsize=11)
    fig2.tight_layout(rect=(0, 0.36, 1, 0.94))
    p2 = os.path.join(outdir, "bench_track_ab19_human_reference.png")
    fig2.savefig(p2, dpi=140)
    print(f"\n[P] figures -> {p1}\n              {p2}")
    return p1, p2


# ------------------------------------------------- the actuator-layer figure (Case B)
def plot_drivetrain_results(path_csv: str, outdir: str, kp: float, kv: float,
                            path_csv_servo: str | None = None,
                            force_limit: float = KNEE_FORCE_LIMIT_NM):
    """Draw the actuator-layer figure from a drivetrain trace.  Returns the PNG path.

    The panels are the signals that DO NOT EXIST in the servo benchmark, which is the
    whole point of the comparison: the current the drive was asked for, the torques on
    the actuator-output shaft, the belt deflection that is the only path from the drive
    to the joint, and the two velocities whose ratio the belt sets.

    THE THREE TORQUES ARE NEVER DRAWN ON ONE AXIS BY ACCIDENT.  Panel C carries the
    controller REQUEST and the TRANSMITTED joint torque together, because the gap
    between them is the result; panel D carries the ACTUATOR-SIDE torques, which live
    on the other side of n_t and are a different quantity with a different magnitude.
    Mixing those two groups on one axis is the mistake this layout exists to prevent.

    If `path_csv_servo` is given, the servo run's knee angle and error are overlaid as
    the Case A reference, so A-vs-B is visible in the figure and not only in a table.
    """
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("\n[P] matplotlib not available in this venv -- CSV written, plots "
              "skipped.")
        return None

    C = read_bench_csv(path_csv)
    missing = [c for c in DRIVETRAIN_EXTRA_HEADER if c not in C]
    if missing:
        sys.exit(f"FATAL: {path_csv} is missing drivetrain columns {missing}")
    A = None
    if path_csv_servo and os.path.exists(path_csv_servo):
        A = read_bench_csv(path_csv_servo)

    ph = C["gait_phase_percent"]
    o = np.argsort(ph, kind="stable")
    fig, ax = plt.subplots(3, 3, figsize=(16.5, 13.0))

    a = ax[0, 0]; _shade(a)
    a.plot(ph[o], C["human_knee_angle_deg"][o], "k--", lw=2.4,
           label="AB19 human reference")
    if A is not None:
        a.plot(ph[o], A["sim_deg"][np.argsort(A["gait_phase_percent"], kind="stable")],
               color="#95a5a6", lw=2.6, alpha=0.9, label="Case A: servo (drivetrain OFF)")
    a.plot(ph[o], C["sim_deg"][o], color="#27ae60", lw=1.6,
           label="Case B: drivetrain ON")
    a.set_ylabel("knee flexion (deg)")
    a.set_title("A. reference vs simulated knee angle, both cases")
    a.legend(fontsize=8, loc="center left")

    a = ax[0, 1]; _shade(a)
    if A is not None:
        a.plot(ph[o], A["err_deg"][np.argsort(A["gait_phase_percent"], kind="stable")],
               color="#95a5a6", lw=2.4, alpha=0.9, label="Case A (OFF)")
    a.plot(ph[o], C["err_deg"][o], color="#c0392b", lw=1.4, label="Case B (ON)")
    a.axhline(0, color="k", lw=0.8)
    a.set_ylabel("simulated - reference (deg)")
    a.set_title("B. tracking error, both cases")
    a.legend(fontsize=8, loc="lower left")

    a = ax[0, 2]; _shade(a)
    a.plot(ph[o], C["i_q_A"][o], color="#d35400", lw=1.4)
    a.axhline(0, color="k", lw=0.8)
    a.set_ylabel("commanded motor current I_q (A)")
    a.set_title("C. drive current (ASSUMED: no current limit)")
    pk_i = float(np.max(np.abs(C["i_q_A"])))
    a.text(0.02, 0.02, f"peak |I_q| = {pk_i:.3f} A\n"
                       f"{force_limit:.1f} N.m would be {force_limit / 4.597092:.2f} A "
                       f"(reference line only,\nNOT a hardware current limit)",
           transform=a.transAxes, fontsize=7.6, va="bottom",
           bbox=dict(boxstyle="round,pad=0.3", fc="#fdf2e9", ec="#d35400", lw=0.8))

    a = ax[1, 0]; _shade(a)
    a.plot(ph[o], C["tau_request_Nm"][o], color="#2c3e50", lw=2.4, alpha=0.6,
           label="controller REQUEST (PD, unclamped)")
    a.plot(ph[o], C["tau_j_Nm"][o], color="#8e44ad", lw=1.4,
           label="TRANSMITTED by belt to knee")
    a.axhline(0, color="k", lw=0.8)
    a.set_ylabel("joint-side torque (N.m)")
    a.set_title("D. what was asked for vs what the belt delivered")
    a.legend(fontsize=8, loc="lower left")

    a = ax[1, 1]; _shade(a)
    a.plot(ph[o], C["tau_m_Nm"][o], color="#16a085", lw=1.3, label="tau_m  motor")
    a.plot(ph[o], C["tau_f_Nm"][o], color="#c0392b", lw=1.1, label="tau_f  friction")
    a.plot(ph[o], C["tau_a_Nm"][o], color="#2980b9", lw=1.3, label="tau_a  net actuator")
    a.axhline(0, color="k", lw=0.8)
    a.set_ylabel("ACTUATOR-SIDE torque (N.m)")
    a.set_title("E. actuator-output shaft torques (divide joint side by n_t)")
    a.legend(fontsize=8, loc="lower left")

    a = ax[1, 2]; _shade(a)
    a.plot(ph[o], 1e3 * C["theta_s_rad"][o], color="#884ea0", lw=1.4)
    a.axhline(0, color="k", lw=0.8)
    a.set_ylabel("belt deflection theta_s (mrad)")
    a.set_title("F. belt deflection -- the only path from drive to joint")
    pk_s = float(np.max(np.abs(C["theta_s_rad"])))
    a.text(0.02, 0.02, f"peak |theta_s| = {1e3 * pk_s:.3f} mrad\n"
                       f"K_s range {float(np.min(C['K_s_Nm_rad'])):.0f}.."
                       f"{float(np.max(C['K_s_Nm_rad'])):.0f} N.m/rad",
           transform=a.transAxes, fontsize=7.6, va="bottom",
           bbox=dict(boxstyle="round,pad=0.3", fc="#f5eef8", ec="#884ea0", lw=0.8))

    a = ax[2, 0]; _shade(a)
    a.plot(ph[o], C["sim_vel_rad_s"][o], color="#e67e22", lw=1.5,
           label="joint side  theta_j_dot")
    a.plot(ph[o], C["theta_a_dot_rad_s"][o], color="#34495e", lw=1.0, alpha=0.85,
           label="actuator side  theta_a_dot")
    a.axhline(0, color="k", lw=0.8)
    a.set_ylabel("angular velocity (rad/s)")
    a.set_title("G. actuator-side vs joint-side velocity")
    a.legend(fontsize=8, loc="lower left")

    a = ax[2, 1]; _shade(a)
    a.plot(ph[o], C["theta_a_rad"][o], color="#34495e", lw=1.4,
           label="theta_a (actuator)")
    a.plot(ph[o], C["sim_rad"][o], color="#e67e22", lw=1.4, label="theta_j (knee)")
    a.axhline(0, color="k", lw=0.8)
    a.set_ylabel("angle (rad)")
    a.set_title("H. actuator vs joint angle (theta_a spans n_t x more)")
    a.legend(fontsize=8, loc="lower left")

    a = ax[2, 2]; _shade(a)
    a.plot(ph[o], C["servo_force_Nm"][o], color="#c0392b", lw=1.6)
    a.plot(ph[o], C["tau_sensor_Nm"][o], color="#7f8c8d", lw=1.0, ls="--",
           label="actuatorfrc sensor")
    a.axhline(0, color="k", lw=0.8)
    a.set_ylabel("MuJoCo position-actuator force (N.m)")
    a.set_title("I. PROOF THE SERVO IS OFF: it must be 0 everywhere")
    pk_sv = float(np.max(np.abs(C["servo_force_Nm"])))
    a.text(0.02, 0.5, f"peak |servo force| = {pk_sv:.3e} N.m\n"
           + ("no double-count: the belt is the only actuation"
              if pk_sv == 0.0 else "NONZERO -- the servo is acting in parallel!"),
           transform=a.transAxes, fontsize=8, va="center",
           bbox=dict(boxstyle="round,pad=0.3",
                     fc="#eafaf1" if pk_sv == 0.0 else "#fdecea",
                     ec="#27ae60" if pk_sv == 0.0 else "#c0392b", lw=0.9))
    a.legend(fontsize=8, loc="lower left")

    _captions(ax.ravel())
    fig.suptitle(
        f"OSL V2 CAD bench -- AB19 tracking WITH the Best et al. actuator layer "
        f"(Option C)\nkp={kp:.0f} N.m/rad, kv={kv:.3f} N.m.s/rad UNCHANGED from the "
        f"servo benchmark; this is an actuator-model comparison, not controller tuning",
        fontsize=11)
    fig.text(0.5, 0.008,
             "NONE of these torques is a human knee moment. Panels D-E are bench "
             "actuator and belt torques on a fixed-base bench with no body weight. "
             "tau_a is ACTUATOR-SIDE and tau_j is JOINT-SIDE:\nthey differ by the belt "
             "ratio n_t and must not be compared directly. The actuator parameters are "
             "PAPER-DERIVED from Best et al. 2025 and are NOT measurements of this "
             "hardware; nothing here is validated against hardware.",
             ha="center", va="bottom", fontsize=8.5,
             bbox=dict(boxstyle="round,pad=0.4", fc="#fdf6e3", ec="#b58900", lw=0.9))
    fig.tight_layout(rect=(0, 0.055, 1, 0.945))
    os.makedirs(outdir, exist_ok=True)
    p = os.path.join(outdir, "bench_track_ab19_drivetrain.png")
    fig.savefig(p, dpi=140)
    print(f"\n[P] drivetrain figure -> {p}")
    return p
