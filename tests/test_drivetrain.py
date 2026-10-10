#!/usr/bin/env python3
"""
test_drivetrain.py -- unit tests for oslbench/drivetrain.py (Best et al. 2025 eqs 1-12).

WHAT THESE TESTS ARE FOR
    They check that the PAPER'S EQUATIONS are encoded correctly and that the SIGNS and
    FRAMES are right.  Every assertion is either
      (a) an exact paper value the paper itself states  (K_s(0) = 876 N.m/rad),
      (b) an algebraic identity that must hold for the equations as printed
          (odd symmetry of tau_j, the (12)->(4) round trip, power balance across the
          belt, K_s = drho/dtheta_s by finite difference), or
      (c) a frame guard -- a test whose only job is to fail if somebody moves a
          quantity to the wrong shaft (J_a is NOT rotor inertia; the total ratio is
          41.49, NOT 49.4).

WHAT THESE TESTS ARE NOT FOR
    They do not validate the model against hardware.  We have never measured an OSL V2
    actuator.  A passing run means "we transcribed Best et al. correctly", not "the OSL
    V2 behaves like this".

    They also do not touch MuJoCo.  drivetrain.py is deliberately free of numpy and
    mujoco, and test_module_has_no_numpy_or_mujoco_import proves it stays that way.

RUN IT
    python tests\\test_drivetrain.py          (any interpreter -- stdlib only)
    python tests\\test_drivetrain.py -v
    python tests\\test_drivetrain.py -k belt

    It has its own runner on purpose: tests/run_tests.py hard-codes
    `import test_oslbench as T`, and this stage is not allowed to modify existing files.
"""

from __future__ import annotations

import importlib.util
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
MODULE_PATH = os.path.join(ROOT, "oslbench", "drivetrain.py")


def _load_drivetrain():
    """Load oslbench/drivetrain.py WITHOUT executing oslbench/__init__.py.

    The package __init__ eagerly imports the numpy-dependent modules, so a plain
    `import oslbench.drivetrain` would drag numpy in and defeat the point of the test
    below.  Loading the file directly proves the module really is standalone.

    THE ONE TRAP:  sys.modules[name] must be set BEFORE exec_module.  @dataclass looks
    its own module up by name in sys.modules while processing the class body, and an
    unregistered module gives `AttributeError: 'NoneType' object has no attribute
    '__dict__'` from dataclasses.py.  That is not a bug in drivetrain.py.
    """
    spec = importlib.util.spec_from_file_location("oslbench_drivetrain_standalone",
                                                  MODULE_PATH)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


D = _load_drivetrain()
P = D.PAPER

# Tolerances.  TIGHT is for algebra that should close to machine precision; FD is for
# finite-difference checks, where the step size sets the floor.
TIGHT = 1e-12
FD = 1e-5


class Skip(Exception):
    """Raised by a test that cannot run here.  Mirrors tests/test_oslbench.py."""


def close(a: float, b: float, tol: float = TIGHT) -> bool:
    """Relative-or-absolute closeness, so it works at 876 and at 1e-6 alike."""
    return abs(a - b) <= tol * max(1.0, abs(a), abs(b))


def approx(a: float, b: float, tol: float = TIGHT, what: str = "") -> None:
    assert close(a, b, tol), f"{what or 'value'}: {a!r} != {b!r} (tol {tol})"


# A deflection ladder used by several tests.  0.055 rad is where Fig. 3's abscissa ends,
# so it is the largest deflection the paper's own fit was exercised over.
DEFL = (1e-9, 1e-6, 1e-4, 1e-3, 5e-3, 0.01, 0.0146, 0.02, 0.0417, 0.055)
VELS = (-12.0, -1.0, -1e-6, 0.0, 1e-6, 1.0, 12.0)
CURRENTS = (-30.0, -4.0, -0.5, 0.0, 0.5, 4.0, 30.0)


# ==================================================================================
# 1.  MOTOR TORQUE -- paper (1), tau_m = I_q * k_t * n_a
# ==================================================================================

def test_motor_torque_equals_iq_kt_na():
    for i_q in CURRENTS:
        approx(D.motor_torque(i_q, P), i_q * P.k_t * P.n_a, TIGHT, f"tau_m({i_q})")


def test_motor_torque_one_amp_is_the_actuator_side_constant():
    # 1 A buys k_t*n_a = 0.9972 N.m at the ACTUATOR OUTPUT (belt input), not at the joint.
    approx(D.motor_torque(1.0, P), 0.9972, TIGHT, "tau_m(1 A)")
    approx(D.motor_torque(1.0, P), P.k_t_actuator, TIGHT, "k_t_actuator")


def test_motor_torque_is_linear_and_odd():
    approx(D.motor_torque(0.0, P), 0.0, TIGHT, "tau_m(0)")
    for i_q in CURRENTS:
        approx(D.motor_torque(2.0 * i_q, P), 2.0 * D.motor_torque(i_q, P), TIGHT, "linear")
        approx(D.motor_torque(-i_q, P), -D.motor_torque(i_q, P), TIGHT, "odd")


def test_motor_torque_is_not_on_the_rotor_or_joint_frame():
    """FRAME GUARD.  tau_m carries n_a exactly once: not zero times, not n_a*n_t times."""
    t = D.motor_torque(10.0, P)
    assert not close(t, 10.0 * P.k_t, 1e-6), "tau_m must include n_a (rotor frame leaked)"
    assert not close(t, 10.0 * P.k_t_joint, 1e-6), "tau_m must NOT include n_t"
    approx(t / (10.0 * P.k_t), P.n_a, TIGHT, "n_a appears exactly once")


def test_joint_torque_constant_is_consistent_with_the_papers_quoted_peak():
    """Sec. II: the OSL V2 is "capable of producing 160 Nm for 10 seconds".

    k_t*n_a*n_t = 4.597 N.m/A, so 160 N.m needs ~34.8 A -- a plausible peak for an
    ActPack-class drive.  This is a consistency check on the paper's own numbers, NOT a
    measurement of our hardware and NOT a torque limit for our model.
    """
    i_for_160 = 160.0 / P.k_t_joint
    assert 20.0 < i_for_160 < 60.0, f"implausible peak current {i_for_160:.1f} A"
    approx(P.k_t_joint * i_for_160, 160.0, 1e-9, "round trip")


# ==================================================================================
# 2.  FRICTION -- paper (2), tau_f = sgn(theta_a_dot) * (f_c + f_g*|I_q|)
# ==================================================================================

def test_friction_sign_follows_velocity_not_current():
    for i_q in CURRENTS:
        assert D.friction_torque(+1.0, i_q, P) > 0.0
        assert D.friction_torque(-1.0, i_q, P) < 0.0
        approx(D.friction_torque(+1.0, i_q, P), -D.friction_torque(-1.0, i_q, P),
               TIGHT, "odd in velocity")


def test_friction_at_zero_velocity_is_zero_no_stiction():
    """sgn(0) = 0, so the model has NO breakaway/stiction term.  The paper has none and
    none was invented; this test pins that down so it cannot be added by accident."""
    for i_q in CURRENTS:
        approx(D.friction_torque(0.0, i_q, P), 0.0, TIGHT, "tau_f at rest")


def test_friction_constant_component_is_f_c():
    approx(D.friction_torque(1.0, 0.0, P), P.f_c, TIGHT, "f_c")
    approx(D.friction_torque(1.0, 0.0, P), 0.171, TIGHT, "f_c value")


def test_friction_current_component_is_f_g_times_abs_iq():
    for i_q in CURRENTS:
        extra = D.friction_torque(1.0, i_q, P) - D.friction_torque(1.0, 0.0, P)
        approx(extra, P.f_g * abs(i_q), TIGHT, f"f_g*|I_q| at {i_q} A")


def test_friction_depends_on_current_MAGNITUDE_only():
    for i_q in (0.5, 4.0, 30.0):
        approx(D.friction_torque(1.0, +i_q, P), D.friction_torque(1.0, -i_q, P),
               TIGHT, "|I_q|")


def test_friction_is_coulomb_not_viscous():
    """Independent of |velocity|.  The viscous part is B_a, a separate term in (1)."""
    a = D.friction_torque(1e-6, 4.0, P)
    b = D.friction_torque(100.0, 4.0, P)
    approx(a, b, TIGHT, "speed-independent")


def test_friction_opposes_the_motor_in_equation_1():
    tau_a = D.actuator_output_torque(4.0, 1.0, 0.0, P)
    assert tau_a < D.motor_torque(4.0, P), "friction must SUBTRACT in (1)"
    approx(tau_a, D.motor_torque(4.0, P) - D.friction_torque(1.0, 4.0, P) - P.B_a * 1.0,
           TIGHT, "eq (1) assembled")


def test_default_sign_is_exact_not_smoothed():
    """The paper's sigma(x) = x/(|x|+alpha) is introduced in the CONTROLLER (Sec. III-B2),
    not in the plant model (1)-(2).  So it must be opt-in, and off by default."""
    exact = D.friction_torque(1e-9, 0.0, P)
    approx(exact, P.f_c, TIGHT, "exact sgn at tiny velocity")
    smoothed = D.friction_torque(1e-9, 0.0, P, smooth_alpha=D.SMOOTH_ALPHA_PAPER)
    assert abs(smoothed) < 1e-6, "sigma must smooth the zero crossing"
    approx(D.smooth_sign(1e-9), 1e-9 / (1e-9 + 0.05), TIGHT, "sigma formula")
    approx(D.smooth_sign(0.0), 0.0, TIGHT, "sigma(0)")
    approx(D.friction_torque(100.0, 0.0, P, smooth_alpha=D.SMOOTH_ALPHA_PAPER), P.f_c,
           1e-3, "sigma -> 1 far from zero")


def test_equation_6_inverts_equations_1_and_2_exactly():
    """(6) solves (24) for I_q.  Feeding its output back through (1)-(2) must return the
    requested actuator torque for EVERY sign combination -- the sharpest available test
    of the friction algebra (the sgn(.) inside the denominator is easy to get wrong)."""
    for tau_des in (-8.0, -1.0, -0.05, 0.05, 1.0, 8.0):
        for v in (-5.0, -0.1, 0.1, 5.0):
            i_q = D.current_for_actuator_torque(tau_des, v, P)
            got = D.motor_torque(i_q, P) - D.friction_torque(v, i_q, P)
            approx(got, tau_des, 1e-10, f"(6) round trip at tau={tau_des}, v={v}")


def test_equation_6_denominator_stays_positive():
    """Appendix A1's positivity condition k_t*n_a - f_g > 0 keeps (6) well posed."""
    assert P.k_t_actuator > P.f_g
    assert P.validate() == []
    bad = D.DrivetrainParameters(f_g=2.0)          # f_g > k_t*n_a = 0.9972
    assert any("positivity" in m for m in bad.validate()), bad.validate()


# ==================================================================================
# 3.  BELT DEFLECTION -- paper (3), theta_j = theta_a/n_t + theta_s
# ==================================================================================

def test_belt_deflection_is_theta_j_minus_theta_a_over_n_t():
    for th_j in (-0.5, 0.0, 0.3, 2.0):
        for th_a in (-5.0, 0.0, 1.383, 9.22):
            approx(D.belt_deflection(th_j, th_a, P), th_j - th_a / P.n_t, TIGHT, "(3)")


def test_kinematics_round_trip_all_three_ways():
    for th_a in (-9.22, 0.0, 1.0, 4.61, 12.3):
        for th_s in (-0.02, 0.0, 0.0146, 0.055):
            th_j = D.joint_angle(th_a, th_s, P)
            approx(D.belt_deflection(th_j, th_a, P), th_s, 1e-14, "theta_s recovered")
            approx(D.actuator_angle(th_j, th_s, P), th_a, 1e-12, "(11): theta_a recovered")


def test_rigid_limit_zero_deflection_gives_theta_a_equals_n_t_theta_j():
    for th_j in (-1.0, 0.0, 0.25, 2.094):
        approx(D.actuator_angle(th_j, 0.0, P), P.n_t * th_j, TIGHT, "rigid limit")
        approx(D.joint_angle(P.n_t * th_j, 0.0, P), th_j, 1e-14, "rigid limit inverse")


def test_state_object_derives_theta_s_consistently():
    st = D.DrivetrainState(actuator=D.ActuatorState(4.61, 9.22),
                           joint=D.JointState(1.05, 2.1))
    approx(st.theta_s(P), 1.05 - 4.61 / P.n_t, TIGHT, "state theta_s")
    approx(st.theta_s(P), D.belt_deflection(1.05, 4.61, P), TIGHT, "matches free fn")
    approx(st.theta_s_dot(P), 2.1 - 9.22 / P.n_t, TIGHT, "state theta_s_dot")
    # theta_s is a METHOD, not a stored field -- it cannot drift out of step with (3).
    assert "theta_s" not in getattr(st, "__dataclass_fields__", {})


def test_two_degrees_of_freedom_are_really_independent():
    """The belt state is genuine extra state: theta_j alone does not determine theta_s."""
    a = D.DrivetrainState(D.ActuatorState(4.61, 0.0), D.JointState(1.0, 0.0))
    b = D.DrivetrainState(D.ActuatorState(4.70, 0.0), D.JointState(1.0, 0.0))
    assert not close(a.theta_s(P), b.theta_s(P), 1e-6), \
        "same theta_j, different theta_a -> different theta_s"


# ==================================================================================
# 4.  BELT TORQUE -- paper (4), tau_j = -tau_s = -sgn(theta_s)*rho(|theta_s|)
# ==================================================================================

def test_rho_is_the_paper_quadratic_on_the_positive_domain():
    for x in DEFL:
        approx(D.rho(x, P), P.p2 * x * x + P.p1 * x, TIGHT, f"rho({x})")
    approx(D.rho(0.0, P), 0.0, TIGHT, "rho(0)")


def test_rho_rejects_negative_input():
    """The paper defines rho on theta_s in R+ only; the sign lives in (4), not in rho."""
    for x in (-1e-9, -0.01, -1.0):
        try:
            D.rho(x, P)
        except ValueError:
            continue
        raise AssertionError(f"rho({x}) should raise ValueError, not silently abs()")


def test_rho_matches_figure_3_right_edge():
    """Fig. 3's abscissa ends at about 0.055 rad, where the fitted curve reads ~93 N.m."""
    approx(D.rho(0.055, P), 93.291825, 1e-9, "rho(0.055)")
    assert 90.0 < D.rho(0.055, P) < 96.0


def test_rho_is_strictly_increasing_and_stiffening():
    prev = -1.0
    for x in DEFL:
        val = D.rho(x, P)
        assert val > prev, f"rho not increasing at {x}"
        prev = val
    # stiffening: the secant slope grows with deflection
    s_small = D.rho(0.005, P) / 0.005
    s_large = D.rho(0.05, P) / 0.05
    assert s_large > s_small, "belt must stiffen, not soften"


def test_joint_torque_is_negative_for_positive_deflection():
    """THE sign convention of (4), and the one most likely to be flipped by accident.

    Positive actuator torque drives theta_a up, which makes theta_s = theta_j - theta_a/n_t
    NEGATIVE, and (4) then returns a POSITIVE joint torque.  See the module docstring.
    """
    for x in DEFL:
        assert D.joint_torque_from_deflection(+x, P) < 0.0, f"tau_j(+{x}) must be < 0"
        assert D.joint_torque_from_deflection(-x, P) > 0.0, f"tau_j(-{x}) must be > 0"


def test_joint_torque_is_odd_and_zero_at_zero():
    approx(D.joint_torque_from_deflection(0.0, P), 0.0, TIGHT, "tau_j(0)")
    for x in DEFL:
        approx(D.joint_torque_from_deflection(-x, P),
               -D.joint_torque_from_deflection(+x, P), TIGHT, f"odd at {x}")


def test_joint_torque_table_positive_and_negative():
    """Explicit values from the closed form, both signs, so a regression is visible."""
    table = {
        0.001: -(P.p2 * 1e-6 + P.p1 * 1e-3),
        0.010: -(P.p2 * 1e-4 + P.p1 * 1e-2),
        0.020: -(P.p2 * 4e-4 + P.p1 * 2e-2),
        0.055: -93.291825,
    }
    for x, expect in table.items():
        approx(D.joint_torque_from_deflection(x, P), expect, 1e-9, f"tau_j({x})")
        approx(D.joint_torque_from_deflection(-x, P), -expect, 1e-9, f"tau_j(-{x})")
    approx(D.joint_torque_from_deflection(0.01, P), -10.2513, 1e-9, "tau_j(0.01)")


def test_belt_torque_is_the_negative_of_joint_torque():
    """(4) reads tau_j = -tau_s.  tau_s is the belt's own torque, tau_j what it does to
    the joint; keeping both makes the minus sign auditable instead of implicit."""
    for x in (-0.055, -0.01, 0.0, 0.01, 0.055):
        approx(D.belt_torque(x, P), -D.joint_torque_from_deflection(x, P), TIGHT, "tau_s")


def test_equation_12_inverts_equation_4():
    for tau_j in (-160.0, -93.0, -16.004120587370423, -1.0, 0.0, 1.0,
                  16.004120587370423, 93.0, 160.0):
        th_s = D.deflection_for_joint_torque(tau_j, P)
        approx(D.joint_torque_from_deflection(th_s, P), tau_j, 1e-9,
               f"(12)->(4) round trip at {tau_j}")


def test_equation_12_sign_and_magnitude_at_the_bench_peak():
    """Our bench's measured peak knee torque is 16.004120587370423 N.m (oracle CSV).
    Feeding it through (12) gives the belt deflection it would imply -- NOT a measurement
    of belt deflection, just the paper's model evaluated at our operating point."""
    th_s = D.deflection_for_joint_torque(16.004120587370423, P)
    assert th_s < 0.0, "positive joint torque implies NEGATIVE deflection by (4)"
    approx(th_s, -0.014627186872385, 1e-9, "theta_s at the bench peak")
    approx(math.degrees(th_s), -0.838076, 1e-5, "theta_s in degrees")
    assert abs(math.degrees(th_s)) < 1.0, "deflection should be well under a degree"


def test_equation_12_stays_inside_the_fitted_range_at_bench_loads():
    th_s = abs(D.deflection_for_joint_torque(16.004120587370423, P))
    assert th_s < 0.055, "bench peak is inside Fig. 3's fitted range -- no extrapolation"


# ==================================================================================
# 5.  BELT STIFFNESS -- paper (5), K_s = drho/dtheta_s = 2*p2*|theta_s| + p1
# ==================================================================================

def test_belt_stiffness_formula():
    for x in DEFL:
        expect = 2.0 * P.p2 * x + P.p1
        approx(D.belt_stiffness(+x, P), expect, TIGHT, f"K_s(+{x})")
        approx(D.belt_stiffness(-x, P), expect, TIGHT, f"K_s(-{x})")


def test_belt_stiffness_is_even_and_monotone_in_magnitude():
    prev = -1.0
    for x in DEFL:
        approx(D.belt_stiffness(x, P), D.belt_stiffness(-x, P), TIGHT, "even")
        val = D.belt_stiffness(x, P)
        assert val > prev, f"K_s not increasing at {x}"
        prev = val


def test_belt_stiffness_is_the_derivative_of_rho():
    """(5) says K_s = drho/dtheta_s.  Check it by central difference on rho itself."""
    h = 1e-7
    for x in (1e-3, 5e-3, 0.01, 0.02, 0.055):
        fd = (D.rho(x + h, P) - D.rho(x - h, P)) / (2.0 * h)
        approx(fd, D.belt_stiffness(x, P), FD, f"drho/dtheta_s at {x}")


def test_belt_stiffness_is_minus_the_slope_of_joint_torque():
    """Consequence of (4)+(5): dtau_j/dtheta_s = -K_s on BOTH sides of zero."""
    h = 1e-7
    for x in (-0.02, -1e-3, 1e-3, 0.02):
        fd = (D.joint_torque_from_deflection(x + h, P)
              - D.joint_torque_from_deflection(x - h, P)) / (2.0 * h)
        approx(fd, -D.belt_stiffness(x, P), FD, f"dtau_j/dtheta_s at {x}")


def test_belt_stiffness_at_torque_closed_form():
    """(5) composed with (12) collapses: K_s = sqrt(p1^2 + 4*p2*|tau_j|).  The -p1 inside
    (12) and the +p1 in (5) cancel exactly.  Both routes must agree."""
    for tau_j in (0.0, 1.0, 16.004120587370423, 50.0, 93.0, 160.0):
        via_defl = D.belt_stiffness(D.deflection_for_joint_torque(tau_j, P), P)
        closed = D.belt_stiffness_at_torque(tau_j, P)
        approx(closed, math.sqrt(P.p1 ** 2 + 4.0 * P.p2 * abs(tau_j)), TIGHT, "closed form")
        approx(via_defl, closed, 1e-9, f"two routes at {tau_j} N.m")
        approx(D.belt_stiffness_at_torque(-tau_j, P), closed, TIGHT, "even in tau_j")


def test_belt_stiffness_at_the_bench_peak():
    approx(D.belt_stiffness_at_torque(16.004120587370423, P), 1312.270475656, 1e-9,
           "K_s at the bench peak")
    # 1.50x the zero-load value: the stiffening is not a second-order detail.
    ratio = D.belt_stiffness_at_torque(16.004120587370423, P) / P.p1
    assert 1.4 < ratio < 1.6, ratio


# ==================================================================================
# 6.  ZERO-DEFLECTION LIMIT -- the paper's explicit K_s(0) = p1 = 876 N.m/rad
# ==================================================================================

def test_zero_deflection_stiffness_is_exactly_876():
    # Exact equality is legitimate: 2*p2*0 + p1 is p1 with no rounding.
    assert D.belt_stiffness(0.0, P) == 876.0, D.belt_stiffness(0.0, P)
    assert D.belt_stiffness(-0.0, P) == 876.0
    approx(D.belt_stiffness(0.0, P), P.p1, TIGHT, "K_s(0) = p1")


def test_zero_load_stiffness_via_the_closed_form_is_also_876():
    # sqrt(876^2) is exact in IEEE-754 (876^2 = 767376 is exactly representable and
    # sqrt is correctly rounded), so this is exact too.
    approx(D.belt_stiffness_at_torque(0.0, P), 876.0, 1e-12, "K_s at zero load")


def test_stiffness_approaches_876_from_both_sides():
    """The departure from 876 is exactly 2*p2*|theta_s|, so assert that rather than an
    arbitrary tolerance -- it pins the limit AND the rate of approach."""
    for eps in (1e-12, 1e-9, 1e-7, 1e-5):
        approx(D.belt_stiffness(+eps, P) - 876.0, 2.0 * P.p2 * eps, 1e-9, f"K_s(+{eps})")
        approx(D.belt_stiffness(-eps, P) - 876.0, 2.0 * P.p2 * eps, 1e-9, f"K_s(-{eps})")
        assert abs(D.belt_stiffness(eps, P) - 876.0) < 1.0, "still ~876 near zero"


# ==================================================================================
# 7.  PARAMETER SANITY -- values, units, magnitudes, immutability
# ==================================================================================

def test_paper_parameter_values_verbatim():
    approx(P.n_a, 9.0, TIGHT, "n_a")
    approx(P.n_t, 4.61, TIGHT, "n_t")
    approx(P.k_t, 110.8e-3, TIGHT, "k_t")
    approx(P.J_a, 9.83e-3, TIGHT, "J_a")
    approx(P.B_a, 6.06e-2, TIGHT, "B_a")
    approx(P.f_c, 17.1e-2, TIGHT, "f_c")
    approx(P.f_g, 82.1e-3, TIGHT, "f_g")
    approx(P.p1, 876.0, TIGHT, "p1")
    approx(P.p2, 14913.0, TIGHT, "p2")


def test_parameters_validate_clean():
    assert P.validate() == [], P.validate()


def test_parameter_magnitudes_are_physically_sane():
    assert 1.0 < P.n_a < 100.0                     # a gearbox, not a direct drive
    assert 1.0 < P.n_t < 10.0                      # single-stage belt
    assert 1e-3 < P.k_t < 1.0                      # N.m/A for a 100 mm-class BLDC
    assert 1e-4 < P.J_a < 1e-1                     # kg.m^2 at the actuator output
    assert 1e-3 < P.B_a < 1.0                      # N.m.s/rad
    assert 1e-3 < P.f_c < 10.0                     # N.m
    assert 1e-3 < P.f_g < P.k_t_actuator           # N.m/A, and (6) needs f_g < k_t*n_a
    assert 1e2 < P.p1 < 1e4                        # N.m/rad
    assert 1e3 < P.p2 < 1e6                        # N.m/rad^2


def test_total_ratio_is_41_point_49_and_not_49_point_4():
    """FRAME GUARD.  49.4 and 58.4 are MyoAssist KA_L1 gain constants, not OSL V2
    mechanical reductions.  The physical total ratio is n_a*n_t = 41.49."""
    approx(P.total_ratio, 41.49, TIGHT, "n_a*n_t")
    approx(P.total_ratio, P.n_a * P.n_t, TIGHT, "definition")
    assert not close(P.total_ratio, 49.4, 1e-3), "total ratio must not be 49.4"
    assert not close(P.total_ratio, 58.4, 1e-3), "total ratio must not be 58.4"


def test_derived_torque_constants():
    approx(P.k_t_actuator, 0.9972, TIGHT, "k_t*n_a")
    approx(P.k_t_joint, 0.9972 * 4.61, 1e-12, "k_t*n_a*n_t")
    approx(P.k_t_joint, 4.5970920, 1e-7, "k_t_joint value")


def test_J_a_is_not_rotor_inertia_and_must_not_be_reflected_through_n_a():
    """FRAME GUARD.  The paper's J_a is measured AT THE ACTUATOR OUTPUT and already
    contains the planetary reflection ("the combined effects of rotor and gearbox
    inertial torques").  Multiplying it by n_a^2 double-counts the 9:1 by a factor 81."""
    approx(P.rotor_inertia_implied, P.J_a / 81.0, TIGHT, "J_a/n_a^2")
    approx(P.rotor_inertia_implied, 1.21358e-4, 1e-5, "implied rotor inertia")
    assert P.rotor_inertia_implied < P.J_a / 50.0, "rotor inertia must be far smaller"
    # the double-counted number, for the record: it is off by 81x and must never appear
    assert not close(D.reflect_actuator_inertia_to_joint(P), (P.total_ratio ** 2) * P.J_a,
                     1e-6), "reflection must use n_t^2, not (n_a*n_t)^2"


def test_parameters_are_frozen():
    try:
        P.k_t = 0.2                                        # type: ignore[misc]
    except Exception as exc:                               # FrozenInstanceError
        assert "frozen" in type(exc).__name__.lower() or "cannot assign" in str(exc), exc
        return
    raise AssertionError("DrivetrainParameters must be immutable")


def test_belt_ratio_swap_is_explicit_and_changes_nothing_else():
    cad = P.with_belt_ratio(D.CAD_BELT_RATIO)
    approx(cad.n_t, 50.0 / 11.0, TIGHT, "CAD belt ratio")
    approx(D.CAD_BELT_RATIO, 4.5454545454545455, 1e-12, "50/11")
    for name in ("n_a", "k_t", "J_a", "B_a", "f_c", "f_g", "p1", "p2"):
        approx(getattr(cad, name), getattr(P, name), TIGHT, f"{name} unchanged")
    assert cad.validate() == []
    # the two candidate ratios differ by ~1.4%, and squared by ~2.9%
    assert 0.005 < abs(D.PAPER_BELT_RATIO - D.CAD_BELT_RATIO) / D.PAPER_BELT_RATIO < 0.02
    assert D.PAPER_BELT_RATIO == 4.61, "the default must remain the paper's value"
    assert P.n_t == D.PAPER_BELT_RATIO, "PAPER must not silently use the CAD ratio"


def test_validate_catches_nonsense():
    assert any("p2" in m for m in D.DrivetrainParameters(p2=-1.0).validate())
    assert any("n_t" in m for m in D.DrivetrainParameters(n_t=0.0).validate())
    assert any("J_a" in m for m in D.DrivetrainParameters(J_a=float("nan")).validate())


# ==================================================================================
# 8.  COORDINATE / TORQUE TRANSFORMATION -- paper (7), and the two sides of the spring
# ==================================================================================

def test_joint_torque_from_actuator_is_n_t_times_actuator_torque():
    for tau_a in (-35.0, -3.472, 0.0, 3.472, 35.0):
        approx(D.joint_torque_from_actuator(tau_a, P), P.n_t * tau_a, TIGHT, "(7)")
        approx(D.actuator_load_torque_from_joint(D.joint_torque_from_actuator(tau_a, P), P),
               tau_a, 1e-12, "round trip")


def test_the_two_sides_of_the_belt_agree():
    """The belt gives tau_j from theta_s via (4); the shaft gives tau_j from tau_a via (7).
    Both must describe the same torque -- that consistency is what closes the system."""
    for th_s in (-0.055, -0.0146, 1e-6, 0.01, 0.055):
        tau_j = D.joint_torque_from_deflection(th_s, P)
        tau_a = D.actuator_load_torque_from_joint(tau_j, P)
        approx(D.joint_torque_from_actuator(tau_a, P), tau_j, 1e-12, "two sides agree")


def test_power_balance_across_the_massless_belt():
    """THE sign test.  For a massless elastic belt storing U(theta_s),

        dU/dt = tau_a*theta_a_dot - tau_j*theta_j_dot

    (power in at the actuator end, minus power delivered to the joint).  U is the
    integral of rho, so this ties (3), (4) and (7) together and fails if ANY of the three
    signs is wrong.  Checked by finite difference on U along a straight-line motion.
    """
    def U(th_s: float) -> float:
        a = abs(th_s)
        return P.p2 * a ** 3 / 3.0 + P.p1 * a * a / 2.0

    h = 1e-8
    cases = ((4.61, 1.0, 9.22, 2.5), (4.61, 1.0, -9.22, 2.5),
             (4.80, 1.0, 3.0, -1.0), (4.00, 1.0, 0.0, 4.0), (4.61, 0.98, -3.0, -2.0))
    for th_a, th_j, dth_a, dth_j in cases:
        th_s = D.belt_deflection(th_j, th_a, P)
        if abs(th_s) < 1e-6:                        # skip the kink; tested separately
            continue
        tau_j = D.joint_torque_from_deflection(th_s, P)
        tau_a = D.actuator_load_torque_from_joint(tau_j, P)
        s_plus = D.belt_deflection(th_j + h * dth_j, th_a + h * dth_a, P)
        s_minus = D.belt_deflection(th_j - h * dth_j, th_a - h * dth_a, P)
        dU_dt = (U(s_plus) - U(s_minus)) / (2.0 * h)
        expect = tau_a * dth_a - tau_j * dth_j
        approx(dU_dt, expect, 1e-4, f"power balance at theta_s={th_s:.4f}")


def test_stored_energy_is_nonnegative_and_its_gradient_is_the_restoring_torque():
    h = 1e-7

    def U(th_s: float) -> float:
        a = abs(th_s)
        return P.p2 * a ** 3 / 3.0 + P.p1 * a * a / 2.0

    approx(U(0.0), 0.0, TIGHT, "U(0)")
    for th_s in (-0.055, -0.01, 0.01, 0.055):
        assert U(th_s) > 0.0, "a spring cannot store negative energy"
        approx(U(th_s), U(-th_s), TIGHT, "U even")
        fd = -(U(th_s + h) - U(th_s - h)) / (2.0 * h)
        approx(fd, D.joint_torque_from_deflection(th_s, P), 1e-4,
               f"tau_j = -dU/dtheta_s at {th_s}")


def test_impedance_reflections_pick_up_n_t_twice_and_torques_once():
    """DIMENSIONAL CONSISTENCY.  In the RIGID limit theta_a_ddot = n_t*theta_j_ddot, so
    an actuator-side inertia appears at the joint as n_t^2*J_a; a friction torque, which
    does not scale with velocity, appears as n_t*f_c only."""
    th_j_ddot, th_j_dot = 3.0, 2.0
    tau_j_inertial = P.n_t * (P.J_a * (P.n_t * th_j_ddot))
    approx(tau_j_inertial, D.reflect_actuator_inertia_to_joint(P) * th_j_ddot, 1e-12,
           "n_t^2 * J_a")
    tau_j_viscous = P.n_t * (P.B_a * (P.n_t * th_j_dot))
    approx(tau_j_viscous, D.reflect_actuator_damping_to_joint(P) * th_j_dot, 1e-12,
           "n_t^2 * B_a")
    approx(D.reflect_coulomb_friction_to_joint(P), P.n_t * P.f_c, TIGHT, "n_t * f_c")


def test_rigid_limit_reflection_values():
    approx(D.reflect_actuator_inertia_to_joint(P), 0.208908143, 1e-9, "n_t^2*J_a")
    approx(D.reflect_actuator_damping_to_joint(P), 1.287877260, 1e-9, "n_t^2*B_a")
    approx(D.reflect_coulomb_friction_to_joint(P), 0.788310000, 1e-9, "n_t*f_c")
    cad = P.with_belt_ratio(D.CAD_BELT_RATIO)
    approx(D.reflect_actuator_inertia_to_joint(cad), 0.203099174, 1e-9, "CAD n_t^2*J_a")
    approx(D.reflect_actuator_damping_to_joint(cad), 1.252066116, 1e-9, "CAD n_t^2*B_a")
    approx(D.reflect_coulomb_friction_to_joint(cad), 0.777272727, 1e-9, "CAD n_t*f_c")
    assert D.reflect_actuator_inertia_to_joint(cad) < D.reflect_actuator_inertia_to_joint(P)


def test_equation_7_is_the_reflections_assembled():
    """(7) tau_j = n_t*(tau_a_des - B_a*theta_a_dot - J_a*theta_a_ddot).  Substituting the
    rigid kinematics turns it into exactly the reflected joint-side impedances."""
    th_j_dot, th_j_ddot = 2.0, 3.0
    tau_des = 3.472
    got = D.joint_torque_linearised(tau_des, P.n_t * th_j_dot, P.n_t * th_j_ddot, P)
    expect = (P.n_t * tau_des
              - D.reflect_actuator_damping_to_joint(P) * th_j_dot
              - D.reflect_actuator_inertia_to_joint(P) * th_j_ddot)
    approx(got, expect, 1e-12, "(7) == reflections")


def test_full_joint_friction_exceeds_the_constant_part_under_load():
    """The part MuJoCo's frictionloss CAN represent is the smaller part.  At our bench's
    peak torque the current-dependent term is about twice the constant one."""
    tau_j = 16.004120587370423
    tau_a = D.actuator_load_torque_from_joint(tau_j, P)
    i_q = D.current_for_actuator_torque(tau_a, 1.0, P)
    approx(tau_a, 3.471609672, 1e-9, "tau_a at the bench peak")
    approx(i_q, 3.980559143, 1e-9, "I_q at the bench peak")
    const = D.reflect_coulomb_friction_to_joint(P)
    full = D.joint_friction_full(1.0, i_q, P)
    approx(full, P.n_t * (P.f_c + P.f_g * abs(i_q)), TIGHT, "n_t*(f_c + f_g|I_q|)")
    approx(const, 0.788310, 1e-6, "the part frictionloss CAN represent")
    approx(full - const, 1.506566, 1e-6, "the part it CANNOT")
    assert full - const > 1.9 * const, "the unrepresentable part is the larger one"


def test_actuator_acceleration_closes_equation_1():
    for i_q in (-10.0, 0.0, 4.0):
        for v in (-3.0, 0.0, 3.0):
            for tau_a in (-5.0, 0.0, 3.472):
                a = D.actuator_acceleration(i_q, v, tau_a, P)
                approx(D.actuator_output_torque(i_q, v, a, P), tau_a, 1e-9,
                       "acceleration closes (1)")


def test_actuator_output_torque_signs_of_each_term():
    base = D.actuator_output_torque(4.0, 0.0, 0.0, P)
    approx(base, D.motor_torque(4.0, P), TIGHT, "no velocity, no acceleration")
    assert D.actuator_output_torque(4.0, 1.0, 0.0, P) < base, "damping+friction subtract"
    assert D.actuator_output_torque(4.0, 0.0, 1.0, P) < base, "inertia subtracts"
    assert D.actuator_output_torque(4.0, -1.0, 0.0, P) > base, "both flip with velocity"


# ==================================================================================
# 9.  NUMERICAL CONTINUITY AROUND theta_s = 0
#     The paper OBSERVES backlash and explicitly NEGLECTS it, so the correct behaviour
#     here is a smooth pass through zero with NO deadband.
# ==================================================================================

def test_joint_torque_is_continuous_through_zero():
    for eps in (1e-12, 1e-9, 1e-6, 1e-4):
        lo = D.joint_torque_from_deflection(-eps, P)
        hi = D.joint_torque_from_deflection(+eps, P)
        assert abs(hi) <= P.p1 * eps * 1.01, (eps, hi)
        approx(lo, -hi, TIGHT, "odd across zero")
        approx(0.5 * (lo + hi), 0.0, 1e-15, "limits agree with tau_j(0) = 0")


def test_joint_torque_is_C1_through_zero_no_backlash_deadband():
    """One-sided slopes must BOTH tend to -p1.  A backlash model would instead give a
    dead zone (zero slope near zero); the paper says it neglects backlash, so there is
    none, and this test would catch one being added silently."""
    for eps in (1e-8, 1e-7, 1e-6):
        right = (D.joint_torque_from_deflection(eps, P)
                 - D.joint_torque_from_deflection(0.0, P)) / eps
        left = (D.joint_torque_from_deflection(0.0, P)
                - D.joint_torque_from_deflection(-eps, P)) / eps
        # the exact one-sided secant slope of -(p2*x^2 + p1*x) over [0, eps]
        approx(right, -(P.p1 + P.p2 * eps), 1e-7, f"right slope at {eps}")
        approx(left, -(P.p1 + P.p2 * eps), 1e-7, f"left slope at {eps}")
        assert abs(right - left) < 1e-9, "no kink in tau_j at zero"
        # the departure from the limit -p1 shrinks linearly with eps: it IS p2*eps
        assert abs(right + P.p1) <= P.p2 * eps * (1.0 + 1e-9), (eps, right)
    # and explicitly: an arbitrarily small deflection produces a nonzero torque
    assert D.joint_torque_from_deflection(1e-9, P) != 0.0, "no deadband allowed"
    approx(D.joint_torque_from_deflection(1e-9, P), -P.p1 * 1e-9, 1e-6, "linear at 0+")


def test_backlash_is_documented_as_neglected():
    note = D.BACKLASH_NOTE.lower()
    assert "backlash" in note and "neglect" in note, D.BACKLASH_NOTE


def test_stiffness_is_continuous_at_zero_but_has_a_kink():
    """K_s itself is continuous; its DERIVATIVE jumps from -2*p2 to +2*p2 at zero, which
    is inherent to the |theta_s| in (5).  Recorded as a property of the paper's model,
    not treated as a defect."""
    h = 1e-9
    approx(D.belt_stiffness(+h, P), D.belt_stiffness(-h, P), 1e-12, "K_s continuous")
    right = (D.belt_stiffness(2 * h, P) - D.belt_stiffness(h, P)) / h
    left = (D.belt_stiffness(-h, P) - D.belt_stiffness(-2 * h, P)) / h
    approx(right, +2.0 * P.p2, 1e-3, "dK_s/dtheta_s on the right")
    approx(left, -2.0 * P.p2, 1e-3, "dK_s/dtheta_s on the left")


def test_joint_torque_is_strictly_monotone_across_zero():
    """A spring must resist in one direction only.  Any sign error inside (4) or rho
    would show up as a non-monotone sweep."""
    n = 401
    prev = float("inf")
    for k in range(n):
        th_s = -0.06 + 0.12 * k / (n - 1)
        val = D.joint_torque_from_deflection(th_s, P)
        assert val < prev, f"tau_j not strictly decreasing at theta_s = {th_s}"
        prev = val


def test_sign_helper_is_zero_at_zero():
    approx(D.sign(0.0), 0.0, TIGHT, "sgn(0)")
    approx(D.sign(-0.0), 0.0, TIGHT, "sgn(-0.0)")
    approx(D.sign(1e-300), 1.0, TIGHT, "sgn(tiny +)")
    approx(D.sign(-1e-300), -1.0, TIGHT, "sgn(tiny -)")


def test_no_nans_or_infinities_anywhere_on_a_sweep():
    n = 201
    for k in range(n):
        th_s = -0.06 + 0.12 * k / (n - 1)
        v = -12.0 + 24.0 * k / (n - 1)
        i_q = -30.0 + 60.0 * k / (n - 1)
        vals = (D.joint_torque_from_deflection(th_s, P),
                D.belt_stiffness(th_s, P),
                D.belt_torque(th_s, P),
                D.friction_torque(v, i_q, P),
                D.motor_torque(i_q, P),
                D.actuator_output_torque(i_q, v, 0.0, P),
                D.belt_stiffness_at_torque(th_s * 1000.0, P),
                D.deflection_for_joint_torque(th_s * 1000.0, P))
        for val in vals:
            assert math.isfinite(val), (k, val)


# ==================================================================================
# 10.  MODULE HYGIENE -- the standalone claim, and the __all__ surface
# ==================================================================================

def test_module_has_no_numpy_or_mujoco_import():
    with open(MODULE_PATH, "r", encoding="utf-8") as fh:
        src = fh.read()
    for banned in ("import numpy", "import mujoco", "from numpy", "from mujoco"):
        assert banned not in src, f"drivetrain.py must stay standalone: found {banned!r}"
    assert "import math" in src and "dataclasses" in src


def test_module_loads_with_neither_numpy_nor_mujoco_present():
    """Re-load it in a namespace where those modules are absent from sys.modules."""
    saved = {k: sys.modules.pop(k) for k in list(sys.modules)
             if k == "numpy" or k.startswith("numpy.") or k == "mujoco"
             or k.startswith("mujoco.")}
    try:
        mod = _load_drivetrain()
        assert mod.PAPER.validate() == []
        assert "numpy" not in sys.modules and "mujoco" not in sys.modules
    finally:
        sys.modules.update(saved)


def test_all_public_names_resolve_and_nothing_public_is_missing():
    missing = [n for n in D.__all__ if not hasattr(D, n)]
    assert not missing, f"__all__ lists names the module does not define: {missing}"
    public = {n for n in dir(D)
              if not n.startswith("_") and n not in ("annotations", "dataclass",
                                                     "field", "replace", "math")}
    absent = sorted(public - set(D.__all__))
    assert not absent, f"public names missing from __all__: {absent}"


# ==================================================================================
# RUNNER  (same conventions as tests/run_tests.py: definition order, -v, -k)
# ==================================================================================

def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    verbose = "-v" in argv or "--verbose" in argv
    pattern = None
    if "-k" in argv:
        pattern = argv[argv.index("-k") + 1]

    g = globals()
    names = [n for n in g if n.startswith("test_")]
    names.sort(key=lambda n: g[n].__code__.co_firstlineno)
    if pattern:
        names = [n for n in names if pattern in n]

    print(f"drivetrain unit tests -- {os.path.relpath(MODULE_PATH, ROOT)}")
    print(f"python {sys.version.split()[0]}, stdlib only "
          f"(numpy loaded: {'numpy' in sys.modules})")
    print("-" * 78)

    failed, skipped = [], []
    for name in names:
        try:
            g[name]()
        except Skip as exc:
            skipped.append(name)
            print(f"SKIP  {name}: {exc}")
        except AssertionError as exc:
            failed.append(name)
            print(f"FAIL  {name}: {exc}")
        except Exception as exc:                                  # noqa: BLE001
            failed.append(name)
            print(f"ERROR {name}: {type(exc).__name__}: {exc}")
        else:
            if verbose:
                print(f"ok    {name}")

    print("-" * 78)
    print(f"{len(names) - len(failed) - len(skipped)} passed, "
          f"{len(failed)} failed, {len(skipped)} skipped, {len(names)} total")
    if failed:
        print("failed: " + ", ".join(failed))
    return len(failed)


if __name__ == "__main__":
    raise SystemExit(main())
