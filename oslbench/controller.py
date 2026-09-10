"""
oslbench.controller -- THE CONTROLLER.  This is the whole control law, in one file.

    error = q_ref - q
    tau   = Kp * error - Kd * qdot          clamped to +/- 142.2 N.m at the knee

Kp = 600 N.m/rad and Kd = 17.253 N.m.s/rad are the gains validated by the sweep in
`experiments/run_gain_sweep.py`.  Kd is not guessed: it is solved from a target
damping ratio, see `kd_for_damping_ratio`.

WHERE THE TORQUE IS ACTUALLY PRODUCED
    MuJoCo's `position` actuator evaluates exactly the same algebra internally:
        tau = gainprm[0]*ctrl + biasprm[0] + biasprm[1]*q + biasprm[2]*qdot
    so writing gainprm = [Kp], biasprm = [0, -Kp, -Kd] makes MuJoCo compute
    Kp*(ctrl - q) - Kd*qdot and then clamp it to the actuator's `forcerange`.
    `write_to_model` performs those four writes and nothing else.  `torque()` below
    is the same expression in Python: it is what the logger records as the servo's
    request, and what the tests compare against MuJoCo's own actuator force.

TWO SEPARATE SATURATIONS, DO NOT CONFUSE THEM
    1. COMMAND saturation.  `ctrl` for a MuJoCo position servo *is* the reference
       angle in radians, and its `ctrlrange` equals the joint's range of motion,
       [-5, 120] deg at the knee.  A reference outside the ROM is clipped before it
       is ever commanded -- see `command()`.  For the AB19 reference this never
       happens (measured: 0 of 2410 steps).
    2. TORQUE saturation.  The torque the law asks for is clamped to `forcerange`,
       +/-142.2 N.m at the knee, +/-168.2 N.m at the ankle -- see `torque()`.  For
       the AB19 reference this never happens either (measured: 0.0 % of steps, peak
       16.00 N.m = 11.25 % of authority).

UNITS
    angles rad, angular velocity rad/s, torque N.m, Kp N.m/rad, Kd N.m.s/rad.

WHAT THIS FILE MUST NEVER DO
    step MuJoCo, read files, or print.  It is pure control algebra plus the four
    array writes that install it.  The model XML is never opened for writing: gains
    live only in the compiled mjModel in memory.
"""

from __future__ import annotations

import math

# --------------------------------------------------------------------- the gains
KP = 600.0        # N.m/rad     validated by experiments/run_gain_sweep.py
KD = 17.253       # N.m.s/rad   = kd_for_damping_ratio(600, 0.261998, 0.3, zeta=0.7)
KV = KD           # alias: MuJoCo and the original scripts call this gain "kv"

ZETA_DEFAULT = 0.7        # target closed-loop damping ratio
AUTHORED_ANKLE_KP = 60.0  # the ankle keeps the servo authored in the MJCF ...
AUTHORED_ANKLE_KD = 0.0   # ... so it is a fixed boundary condition, not a variable

SAT_TOL_NM = 1e-6         # a request within 1 uN.m of the limit is not "saturated"


def kd_for_damping_ratio(kp: float, i_eff: float, b_joint: float,
                         zeta: float = ZETA_DEFAULT) -> float:
    """Solve  zeta = (Kd + b_joint) / (2*sqrt(Kp*I_eff))  for Kd.

    b_joint is the joint damping already present in the MJCF (0.3 N.m.s/rad at the
    knee).  Crediting it matters at low Kp, where 0.3 is a real fraction of the
    damping the loop needs.  With Kp = 600, I_eff = 0.261998 kg.m^2, b = 0.3 and
    zeta = 0.7 this returns 17.253 N.m.s/rad -- the validated Kd.
    """
    return 2.0 * zeta * math.sqrt(kp * i_eff) - b_joint


kv_for = kd_for_damping_ratio          # name used by the original gain-sweep script


class PDController:
    """A joint-space PD position servo with an explicit command clamp and torque clamp.

    kp             N.m/rad
    kd             N.m.s/rad
    command_limits (lo, hi) rad -- the actuator's ctrlrange, i.e. the joint ROM
    torque_limit   N.m, symmetric -- the actuator's forcerange upper bound
    """

    def __init__(self, kp: float, kd: float,
                 command_limits: tuple[float, float],
                 torque_limit: float, name: str = "knee"):
        self.kp = float(kp)
        self.kd = float(kd)
        self.command_limits = (float(command_limits[0]), float(command_limits[1]))
        self.torque_limit = float(torque_limit)
        self.name = str(name)

    # ------------------------------------------------------------ the control law
    def command(self, q_ref: float) -> float:
        """The reference angle as it will be commanded: clipped into ctrlrange (rad)."""
        lo, hi = self.command_limits
        return lo if q_ref < lo else (hi if q_ref > hi else float(q_ref))

    def torque_unclamped(self, q_ref: float, q: float, qdot: float) -> float:
        """The torque the law asks for, before the actuator limit (N.m)."""
        error = self.command(q_ref) - q                 # rad
        return self.kp * error - self.kd * qdot          # N.m

    def torque(self, q_ref: float, q: float, qdot: float) -> float:
        """The torque the actuator can actually deliver: clamped to forcerange (N.m)."""
        tau = self.torque_unclamped(q_ref, q, qdot)
        lim = self.torque_limit
        return -lim if tau < -lim else (lim if tau > lim else tau)

    # ------------------------------------------------------------- saturation flags
    def command_clamped(self, q_ref: float) -> bool:
        """True if the reference had to be clipped into the joint's range of motion."""
        return abs(self.command(q_ref) - float(q_ref)) > 1e-12

    def torque_saturated(self, q_ref: float, q: float, qdot: float) -> bool:
        """True if the law asked for more torque than the actuator is allowed to give."""
        return abs(self.torque_unclamped(q_ref, q, qdot)) > self.torque_limit + SAT_TOL_NM

    # --------------------------------------------- install the gains into MuJoCo
    def write_to_model(self, model, actuator_id: int) -> None:
        """Put Kp and Kd into the COMPILED mjModel, so MuJoCo's position actuator
        computes  tau = Kp*(ctrl - q) - Kd*qdot  and clamps it to forcerange.

        models/osl_v2_bench.xml is never opened for writing.  forcerange and
        ctrlrange are deliberately not touched, so the authored torque authority and
        ROM still bound everything the controller can do.
        """
        model.actuator_gainprm[actuator_id, 0] = self.kp     # tau += kp*ctrl
        model.actuator_biasprm[actuator_id, 0] = 0.0         # no constant bias
        model.actuator_biasprm[actuator_id, 1] = -self.kp    # tau += -kp*q
        model.actuator_biasprm[actuator_id, 2] = -self.kd    # tau += -kd*qdot

    # ------------------------------------------------------------------ factories
    @classmethod
    def knee(cls, bench, kp: float = KP, kd: float = KD) -> "PDController":
        """The tracking controller: the validated pair, on the knee actuator."""
        return cls(kp, kd, bench.knee_ctrlrange, bench.knee_forcerange[1], "knee")

    @classmethod
    def ankle_hold(cls, bench) -> "PDController":
        """The ankle's own MJCF-authored servo (kp=60, kv=0), left exactly as authored.
        It holds the ankle at its keyframe angle: a fixed boundary condition."""
        return cls(AUTHORED_ANKLE_KP, AUTHORED_ANKLE_KD,
                   bench.ankle_ctrlrange, bench.ankle_forcerange[1], "ankle")

    def natural_frequency(self, i_eff: float) -> float:
        """Undamped natural frequency sqrt(Kp/I_eff) in rad/s -- reporting only."""
        return math.sqrt(self.kp / i_eff)

    def damping_ratio(self, i_eff: float, b_joint: float) -> float:
        """(Kd + b_joint) / (2*sqrt(Kp*I_eff)) -- reporting only."""
        return (self.kd + b_joint) / (2.0 * math.sqrt(self.kp * i_eff))

    def __repr__(self) -> str:
        return (f"PDController({self.name}, Kp={self.kp:g} N.m/rad, "
                f"Kd={self.kd:g} N.m.s/rad, torque_limit=+/-{self.torque_limit:g} N.m, "
                f"command_limits=({math.degrees(self.command_limits[0]):.2f}, "
                f"{math.degrees(self.command_limits[1]):.2f}) deg)")
