# The bench experiment, question by question

This is the document to read before judging the result. It answers, in order: what is
being simulated, what the human data is, what the controller is, what happens on each
0.5 ms step, what is measured, and — the part that matters most — what this experiment
does **not** establish.

If you only want to find a file, read `docs/CODE_MAP.md` instead. If you want to run it,
the Quick Start is in `README.md`.

## What model is being simulated?

`models/osl_v2_bench.xml`: the Open-Source Leg V2, generated from the official Onshape
CAD export by `tools/build_mjcf.py`, welded to the world at the knee-proximal segment.
Three bodies (`knee_prox`, `shank`, `foot`), two hinges (knee, ankle), two position
actuators, and the sensor set the real leg reports — joint encoders, actuator torque, two
IMUs and a six-axis load cell.

Mass, centre of mass and inertia of each segment come from the exact mesh volume
(divergence theorem over each closed triangle mesh) times a density assigned per part
family. Both hinge anchors come from the CAD, not from a choice: knee at
`(0.00065, 0, 0.19311)`, ankle at `(0, 0, −0.13873)`, 331.84 mm apart. Range of motion
comes from this export's own hard stops: knee −5° to +120°, ankle −30° to +20°.
`scripts/check_model.py` holds MuJoCo to those numbers on every run rather than merely
asking whether the file compiled.

Because the base is welded, the model has exactly two degrees of freedom (`nq = nv = 2`),
and the knee's is the only one that moves during the experiment.

## Why this model, and not something simpler or something bigger?

Simpler would be a rod and a point mass. That was rejected because the whole reason the
CAD model exists is to get the segment inertia right, and a generic rod would put the
answer back to being a guess.

Bigger would be a whole-body walking model. That was also rejected, but only *for this
experiment*, and for one reason: on a fixed base, exactly one mass property of the leg
can affect the answer, and that makes the result interpretable. With the proximal segment
welded, the knee sees an effective inertia of

```
I_eff = I_body + armature = 0.251998 + 0.010 = 0.261998 kg·m²
```

plus joint damping 0.3 N·m·s/rad, joint friction 0.4 N·m and a gravity moment of
8.8529 N·m at full extension. Nothing else in the model can change the trajectory. So
when the tracking error comes out at 4.36° we can say precisely what produced it — and,
as it turns out, show that 90 % of it is servo phase lag rather than a modelling defect.
On a walking model the same number would be entangled with ground contact, body weight
and balance, and there would be no way to attribute it.

The bench is therefore the *first* experiment, not the intended final one.

## What human data is being tracked?

`build/AB19_knee_gait_reference.csv`: one complete right-leg gait cycle from subject AB19
of the Camargo et al. open lower-limb dataset (Georgia Tech), trial
`levelground / ccw / normal / 01_01`. 101 samples spanning 0 to 100 % of the gait cycle,
which at this subject's cadence is a period of 1.2050 s.

`oslbench/reference.py` loads five columns verbatim, audits them, and resamples the knee
**angle** column onto the model's 0.5 ms grid with a natural cubic spline — 101 samples
become 2410 steps. The spline reproduces every original measured sample to about
1e-14 degrees, so resampling adds no error at the data points; only between them is it an
interpolation, which is unavoidable for any 100 Hz-equivalent recording driven at 2 kHz.

Gait phase is not interpolated. It is computed analytically as
`100 × (t mod T) / T`, so it stays in `[0, 100)` by construction and can never drift.

## Why AB19?

Because the dataset publishes it in a form that needs no assumptions: a single named
subject, a single named trial, level ground at normal speed, one clean right-leg cycle.
An averaged or normalised "typical" gait curve would have been smoother and less honest —
it would have hidden which subject and which trial produced it, and there would be nothing
to go back to if a column turned out to be wrong.

Which matters, because a column *did* turn out to be wrong. See "What is wrong with the
data" below.

## What is the controller?

A joint-space PD position controller. In full, from `oslbench/controller.py`:

```python
error = q_ref - q                       # rad      reference minus measured angle
tau   = Kp * error - Kd * qdot          # N.m      the requested torque
```

with `Kp = 600.0` N·m/rad and `Kd = 17.253` N·m·s/rad, and two saturations applied around
it:

1. **The command is clipped into the joint's range of motion** before the error is formed,
   so the controller can never chase an angle the hardware cannot reach. Measured on the
   AB19 reference: **0 of 2410 steps** were clipped — the human trajectory lies entirely
   inside the mechanism's ROM.
2. **The requested torque is clamped to the actuator's `forcerange`**, ±142.2 N·m at the
   knee. Measured: **0.00 % of steps** saturated, with a peak request of 16.00 N·m, which
   is 11.25 % of the available authority.

There is no integral term, no feedforward, no gain scheduling and no impedance law. That
is deliberate: this is a baseline whose failure mode should be diagnosable, not a
candidate final controller.

## What are Kp and Kd, physically?

`Kp` is a rotational stiffness in N·m per radian of tracking error — how hard the actuator
pulls toward the commanded angle. `Kd` is a rotational damping in N·m per rad/s of joint
velocity — it opposes motion, which is what stops the stiffness from causing overshoot and
ringing.

Together with the plant they set the closed-loop behaviour of a second-order system:

```
natural frequency   ω_n = sqrt(Kp / I_eff)            = sqrt(600 / 0.261998) = 47.86 rad/s ≈ 7.6 Hz
damping ratio       ζ   = (Kd + b_joint) / (2 sqrt(Kp · I_eff))
```

Note that the joint's own damping `b_joint = 0.3` counts toward ζ. Ignoring it is a
common way to end up slightly under-damped.

## Why *these* values of Kp and Kd?

`Kd` is not tuned at all — it is derived. Given a target damping ratio, rearranging the
expression above gives

```python
# oslbench/controller.py:56
Kd = 2 * zeta * sqrt(Kp * I_eff) - b_joint
```

At ζ = 0.7 — the standard choice for a servo that should settle fast without overshooting
— and Kp = 600, this gives Kd = 17.253. So there is exactly one free parameter in this
controller, and `Kd` follows from it.

`Kp = 600` came from a sweep, not a preference. `experiments/run_gain_sweep.py` runs the
same plant and the same control law across a range of Kp with Kd re-derived at each point,
and `BENCH_TUNING_AND_DATASET.md` records the reasoning. The short version: below about
Kp = 300 the loop is too slow to follow the swing-phase velocity peak; above about
Kp = 600 the tracking error stops improving, because what remains is not stiffness-limited.

That last claim is measured rather than asserted. `oslbench/metrics.py:lag_diagnostic`
searches for the time shift that best aligns the simulated trajectory with the reference,
and finds **29.5 ms** — about 2.4 % of the gait cycle. Removing that shift drops the RMS
error from 4.3616° to 0.4333°, i.e. **90.1 % of the tracking error is pure phase lag.**
Raising Kp to chase a lag is the wrong fix; advancing the reference in time, or adding a
feedforward term, is the right one. Neither has been done, because this run is the
baseline they would have to be measured against.

## What exactly happens on each 0.5 ms step?

`oslbench/simulation.py:179`, in order:

1. Look up the reference angle for this step, `q_ref[k]`, from the resampled array.
2. Read the current measured state from the sensors: `q` (knee angle, rad) and `qdot`
   (knee velocity, rad/s).
3. Clip `q_ref` into `ctrlrange` and write it to `data.ctrl[knee]`. Note that for a MuJoCo
   position actuator `ctrl` **is an angle in radians**, not a torque.
4. Call `mujoco.mj_step`. Inside that call MuJoCo evaluates the actuator, which is
   configured so that its output is exactly `Kp*(ctrl − q) − Kd*qdot`, clamps the result to
   `forcerange`, applies it to the knee hinge, and integrates the two-DOF plant forward by
   0.5 ms against gravity, joint damping and joint friction.
5. Read back what happened: the new `q`, `qdot`, and the actuator torque that was applied.
6. Hand all of it to the logger as one row.

Steps 1–3 and 6 are Python. Step 4 is entirely MuJoCo. The controller runs *inside* the
simulation loop, once per step, at 2 kHz — not once per frame and not offline.

One detail worth stating because it looks like a bug and is not: after `mj_step`, MuJoCo's
`actuator_force` reports the torque that was used to *take* the step, i.e. evaluated on the
pre-step state. The logged `tau_unclamped` column is evaluated on the post-step state.
They are close but not equal, and the identity test in `tests/test_oslbench.py` compares
the actuator's torque against the control law evaluated on the pre-step state, which is
the honest comparison.

## How does the controller's torque reach the physics?

Through the compiled model, at runtime. A MuJoCo `position` actuator computes

```
tau = gainprm[0]*ctrl + biasprm[0] + biasprm[1]*q + biasprm[2]*qdot
```

so writing `gainprm[0] = Kp`, `biasprm = [0, −Kp, −Kd]` makes MuJoCo itself evaluate
`Kp*(ctrl − q) − Kd*qdot`. That is four array writes, in
`oslbench/controller.py:116 write_to_model`.

Two consequences worth noting. `models/osl_v2_bench.xml` is never edited — the XML's
authored `kp = 60` placeholder servo is overwritten in memory, and the file on disk is
byte-for-byte unchanged (the tests assert its sha256). And `forcerange` and `ctrlrange`
are deliberately *not* written, so the authored torque authority and ROM keep bounding
everything the controller can do.

## What is measured?

Per step, 21 columns into `build/bench_track_ab19/bench_track_ab19.csv`: time, gait phase,
reference angle and velocity, commanded angle, measured angle and velocity, tracking
error, actuator torque, unclamped requested torque, torque as a percentage of authority, a
saturation flag, mechanical power, the ankle's held angle, and the three human columns
carried through verbatim.

From that, `oslbench/metrics.py` computes the summary. The validated result:

| Metric | Value |
| --- | --- |
| RMS tracking error | 4.3616° |
| peak tracking error | 9.0907° |
| mean absolute error | 3.2799° |
| peak actuator torque | 16.0041 N·m |
| torque authority used | 11.25 % of ±142.2 N·m |
| torque saturation | 0.00 % of steps |
| reference clipped into ROM | 0 of 2410 steps |
| peak knee velocity | 5.0559 rad/s (reference peak 5.1077) |
| servo lag | 29.5 ms = 2.45 % of the cycle |
| RMS error with the lag removed | 0.4333° (so 90.1 % of the error is lag) |

`experiments/verify_against_oracle.py` re-runs the experiment and compares all 21 columns
and all metrics against a frozen copy in `tests/oracle/`, so these numbers are defended by
a check rather than by a note in a file.

## Is that torque a human knee moment?

**No.** This is the single most important sentence in this document.

The reported torque is the *bench actuator torque* of a fixed-base device. The proximal
segment is welded to the world: there is no pelvis, no hip, no ground contact, no heel
strike and no body weight. The actuator is fighting shank-and-foot inertia, gravity on
that segment, joint damping and joint friction, and nothing else — about 8.9 N·m of
gravity torque at full extension.

A real human knee during stance carries body weight through a closed kinetic chain, which
is why a human knee moment peaks at a value the bench never sees. The two quantities are
not comparable during stance at all; only swing (roughly 60–100 % of the cycle) is even
loosely comparable. The stance shading on the plots is there to orient the reader in the
gait cycle and does **not** imply the bench is in stance.

The code enforces the separation: `tau_sensor_Nm` (bench actuator torque) and
`human_knee_moment` (the dataset's biomechanical column) are logged side by side and are
never added, subtracted, ratioed or plotted on the same axis anywhere in this repository.

## Is this whole-body walking?

**No.** Nothing walks. One knee joint of a bench-mounted device follows an angle
trajectory that was recorded from a walking human. There is no locomotion, no balance, no
contact, no energetics and no gait controller. The gait cycle appears only as the
*shape of a reference signal*.

## What has been validated?

Four things, each by something that runs.

The **CAD-to-MuJoCo chain**: `scripts/check_model.py` compiles the scene and compares
masses, world centres of mass, hinge anchors and the mesh bounding box against a sidecar
prediction written by the build, and exits non-zero on disagreement.

The **control law and the plumbing**: `tests/run_tests.py` — 44 tests — checks the PD
equation against arithmetic done independently in the test, that Kd really is the ζ = 0.7
value, that both saturations fire in both directions, that the gains reach the compiled
model and that `forcerange` and `ctrlrange` survive, that the reference contract holds
(101 rows, phase in `[0, 100)`, period 1.2050 s, 2410 uniform resamples), and that the
model XML and the AB19 CSV are byte-for-byte unchanged.

The **numerical result**: `experiments/verify_against_oracle.py` re-runs the experiment
and requires all 21 logged columns and every metric to match the frozen validated copy.

That **the live demo and the quantitative experiment are the same code**: before the
viewer opens, `oslbench/viewer.py:consistency_check` replays a span of the benchmark
through the live path and refuses to start unless angle, velocity, torque and command
agree to 1e-12. The dashboard has no plant and no controller in it — it draws the last
published snapshot of the one shared state.

## What remains unvalidated?

**The transmission parameters are placeholders.** Armature 0.010 kg·m², damping
0.30 N·m·s/rad and friction 0.40 N·m at the knee are stand-ins for the belt, pulley,
gearbox and bearings, which the model collapses into a single hinge. They are round
numbers to be identified from bench data, and they are not innocuous: they set how much of
the tracking error is the plant rather than the servo.

**Torque production is idealised.** The `forcerange` of ±142.2 N·m (knee) and ±168.2 N·m
(ankle) is derived from the motor/gear relationship published for this hardware — the
model represents the available joint torque authority derived from the motor/gear
relationship, but idealizes the torque production. It does not model the motor as an
electrical system: no current, no voltage, no torque–speed curve, no efficiency, no
thermal state, no backlash, no transmission compliance and no actuator bandwidth. It
should not be described as a full motor/gearbox simulation, and these are not
experimentally verified hardware values.

**The mass is 0.42 kg light, and the reason is known but not measured.** The CAD export
contains no motors — all 260 distinct meshes were searched and the only motor-related part
is a coupling — so the 4.9558 kg total sits below the 5.377 kg published for a real build.
That shortfall is 0.211 kg per motor, the right order for a brushless motor of this class,
but it is a hypothesis until a motor goes on a scale. `MOTOR_MASS` in the build script
defaults to zero on purpose: inferring the motor mass from (published − ours) and then
citing the agreement as validation would be circular.

For reference, the official MyoAssist `OpenSourceLeg_KA_L1` model comes out at
approximately 4.5275 kg excluding the socket, against our CAD-derived 4.9558 kg. The foot
is the unresolved part: ours is 0.9152 kg against their 0.2910 kg, and that difference has
not been explained. Do **not** say one is simply "better" — they are different objects
derived in different ways, and the discrepancy is an open item.

**Two columns of the human data are defective.** The audit prints five flags, and they
concern `human_knee_moment` and `human_knee_power` only. In magnitude the two are
correlated at 0.99962, which means they are one signal scaled rather than two independent
measurements; their ratio is nearly constant and is uncorrelated with the reference angular
velocity, so power is not moment × ω for this angle column; the peak moment is about 7.6
N·m/kg where a human walking knee peaks near 0.4–0.6, consistent with a unit error; the
peak falls on the very first sample instead of in early stance; and the signal decays
almost monotonically like a filter start-up transient. Until that extraction is redone
both columns are **unverified**. They are carried through the CSV verbatim so the defect
stays visible, and nothing in the experiment depends on them: it is driven by the angle
column alone, and the angle column is sound.

**The hardware has not been validated.** Nothing in this repository has been compared
against the physical Open-Source Leg. There is no bench data, no encoder log and no torque
measurement from the real device. Every number here is a simulation result.

## What is the sim-to-real limitation?

Stated as plainly as possible: a position servo tracking a recorded angle on an idealised
actuator with placeholder transmission parameters, on a fixed base, is the *easiest*
version of this problem. The real leg will be harder in at least five ways this experiment
cannot see — the motor has finite bandwidth and a torque–speed limit that a saturation
clamp does not capture; the transmission has backlash and compliance that a single hinge
does not have; the socket–limb interface is compliant and this bench has no socket at all;
under body weight the knee sees loads two orders of magnitude above the 16 N·m measured
here; and a real controller has to survive sensor noise, quantisation and a control loop
that is not perfectly synchronous with the plant.

So the correct reading of the 4.36° RMS result is narrow and it is worth being precise
about: *this device model, on this bench, with this controller and these gains, can follow
a human knee-angle trajectory using 11 % of its rated torque, and 90 % of its residual
error is a 29.5 ms phase lag that is removable in software.* It is a working baseline and
a diagnosable one. It is not a claim about the hardware.
