# Bench controller tuning + Camargo dataset acquisition

**Scope of this report.** Answers A–F for the dataset-acquisition and controller-tuning
request. **No model was modified.** `models/osl_v2_bench.xml`, the KA_L1 fragment, all
MyoAssist code and the controller integration are byte-for-byte untouched; the only new
files are the three scripts and one figure listed in §Files. The human dataset has **not**
been used as a tracking input — instruction 11 is respected: every reference motion in the
gain sweep is synthetic.

**Two-tier evidence.** Every number below is tagged. `[MEASURED]` was read off a file,
`[COMPUTED]` was derived from measured values, `[PREDICTED]` came from the numpy
reproduction of the bench dynamics and is falsifiable by one Windows run, and
`[UNCONFIRMED]` still needs a machine I do not have. The sandbox I work in has **no
network access** (`data.mendeley.com` is not on the egress allowlist) and **no MuJoCo**, so
A, B and the real C must come from you running two commands. The scripts that produce them
are written, compiled and logic-tested. See §Commands.

---

## A. Confirmed dataset availability

**Status: three named, resolvable deposits identified; download not yet executed
`[UNCONFIRMED]`.** The Camargo et al. (2021) dataset is not embedded in the paper PDF
(`pdfdetach -list` reports zero attachments) `[MEASURED]`. It is distributed as three
Mendeley Data parts, cited in the paper's data-availability statement:

| Part | DOI | Note |
|---|---|---|
| 1 of 3 | `10.17632/fcgm3chfff.1` | |
| 2 of 3 | `10.17632/k9kvm5tn3f.1` | suffix printed across a line break in the PDF as `k9kvm5t-` / `n3f.1`; **verify by resolving before trusting** |
| 3 of 3 | `10.17632/jj3r5f9pnf.1` | |

Two secondary routes exist and should be treated as cross-checks, not the primary source:
the lab's own page at `epic.gatech.edu/opensource-biomechanics-camargo-et-al`, and the
Elsevier supplementary material attached to the article record.

`experiments/camargo_probe.py --check` resolves each DOI, prints the HTTP status and the
redirect target, and then attempts a file listing. Note honestly that the listing endpoint
it uses (`data.mendeley.com/public-api/datasets/{id}/files`) is marked UNVERIFIED in the
script and a failure there is non-fatal — DOI resolution is the load-bearing check, the
listing is a convenience. If the listing fails you will still see whether the deposits
exist and where they redirect, and you download through the browser.

One structural risk is worth flagging before you download 3 parts: the tables are stored as
MATLAB **`table` objects**. `scipy.io.loadmat` generally *cannot* decode these — they arrive
as opaque MCOS containers. The probe detects this case explicitly and prints the remediation
(a one-time CSV export from MATLAB or Octave) rather than failing with a confusing traceback.
If the files turn out to be v7.3 the probe reads them through `h5py` instead. This is the
single most likely thing to cost you an hour, so it is checked first.

## B. Selected subject, trial, files, columns, units

**Subject `AB19` `[MEASURED from paper Table 1]`** — male, 1.70 m, 68.0 kg. Chosen because
he sits essentially on the cohort mean (1.70 ± 0.07 m, 68.3 ± 10.83 kg), so nothing in the
first experiment depends on an atypical body. Backups, in order, are `AB27` (near-identical
anthropometry, independent subject) and `AB20`. Subject IDs skip AB22, AB26 and AB29, so do
not read a gap as a missing download. The fallback rule from instruction 2 is implemented in
the probe, not left to judgement: if AB19 has no `levelground` trial carrying a finite knee
moment, the probe says so and names the next `PREFERRED` subject that does.

**Trial condition:** `levelground`, self-selected **normal** speed only (cohort 1.17 ± 0.21
m/s; the slow/fast conditions are 0.88 and 1.45 m/s). One subject, one condition, per
instructions 3 and 4.

**Layout `[MEASURED from the paper's description]`:** `subject / date / mode / sensor`, e.g.
`AB19/<date>/levelground/ik/*.mat`. The `sensor` level is where the variables live:
`ik` (inverse kinematics → joint angles), `id` (inverse dynamics → moments and powers),
`gcRight` (gait cycle / heel-strike percentage), `conditions` (speed and condition labels),
`gon` (goniometer, 1000 Hz, useful as an independent sign check).

**Expected file and column mapping.** These are the names to *look for* — they are the
OpenSim gait2392 conventions the pipeline implies, and they are **`[UNCONFIRMED]` until the
probe prints the real headers.** Nothing downstream hard-codes them:

| Quantity | Expected file | Expected column | Expected unit |
|---|---|---|---|
| right knee angle | `levelground/ik/*.mat` | `knee_angle_r` | deg |
| right knee moment | `levelground/id/*.mat` | `knee_angle_r_moment` | N·m/kg (mass-normalised) |
| right knee power | `levelground/id/*.mat` | `knee_angle_r_power` | W/kg (mass-normalised) |
| time | every table | `Header` | s (200 Hz for `ik`/`id`) |
| gait phase | `levelground/gcRight/*.mat` | `HeelStrike` | % 0–100 |
| walking speed | `levelground/conditions/*.mat` | `Speed` / label column | m/s |
| subject mass | paper Table 1 (`SubjectInfo` if present) | — | kg (68.0 for AB19) |

Two things about this table matter more than the names. First, **units are inferred from the
data, not from the paper** (instruction 6): `infer_unit()` distinguishes degrees from radians
by numeric range, mass-normalised from absolute moment by magnitude, percent from fraction,
and — importantly — flexion-positive from flexion-negative. That last one is a real trap:
gait2392's `knee_angle_r` is flexion-*negative* while the paper's figures are
flexion-*positive*, so the stored sign must be checked against landmarks (stance flexion
peak ~15–20° near 15 % GC, swing peak ~60–65° near 70–73 %, toe-off ~60–62 %) and against
the 1000 Hz goniometer. Getting this wrong would silently invert the whole experiment.

Second, **gait phase is pre-computed per sample** `[MEASURED from the paper]` — the authors
interpolate linearly between right heel strikes, where heel strike is detected as zero
linear velocity of the heel marker. So instruction 7 resolves to "use it directly"; the
heel-strike-to-heel-strike reimplementation is coded as a fallback only and should not be
needed. The probe reports which branch applies.

One coverage constraint will bite during stride selection: ground reaction force was recorded
only "during certain representative steps," and "gait cycles without ground reaction force
data were excluded from moment and power analyses." Kinematics are therefore available on
far more strides than kinetics, and strides must be filtered to those with a non-null knee
moment. The probe counts these per trial. No figure is digitized, and specifically not
Figure 6, which is a 22-subject average (instruction 8).

## C. Gain sweep results

**Status: `[PREDICTED]`, not yet run in MuJoCo.** The sandbox has no MuJoCo, so rather than
leave C empty I reproduced in numpy the *exact* dynamics MuJoCo integrates for this model.
That reproduction is legitimate here for a specific structural reason: `knee_prox` carries no
freejoint, so the bench is welded to the world, and with the ankle held at its keyframe the
knee is a genuine single-DOF pendulum. The equation of motion is then fully determined by
values authored in the MJCF, with nothing fitted:

```
I_eff * qdd = tau_act + tau_gravity(q) - b*qd - frictionloss*sign(qd)
tau_act     = clip(kp*(ref - q) - kv*qd, -142.2, +142.2)
```

with `[COMPUTED from the MJCF]` `I_eff = 0.251998 + 0.01 armature = 0.261998 kg·m²`,
`b = 0.3 N·m·s/rad`, `frictionloss = 0.4 N·m`, `m·g·d = 8.8529 N·m`. Running
`experiments/run_gain_sweep.py` on Windows should agree with the tables below to within the
integrator difference (semi-implicit Euler here vs MuJoCo's), and any disagreement larger
than a few percent is a finding in itself, not a nuisance.

**Gains, from ζ = 0.7 crediting the damping the model already has.** Solving
`ζ = (kv + b_joint) / (2√(kp·I_eff))` for `kv` rather than ignoring the authored
0.3 N·m·s/rad — which matters at low kp, where 0.3 is 6 % of the total:

| kp (N·m/rad) | kv (N·m·s/rad) | ω_n (Hz) | f_−3dB (Hz) | lag @0.9 Hz | lag (%GC) | friction deadband |
|---|---|---|---|---|---|---|
| 60 | 5.251 | 2.409 | 2.43 | 96.6 ms | 8.69 | 0.382° |
| 200 | 9.834 | 4.397 | 4.44 | 51.4 ms | 4.63 | 0.115° |
| **600** | **17.253** | **7.616** | **7.68** | **29.4 ms** | **2.65** | **0.038°** |
| 2000 | 31.747 | 13.905 | 14.03 | 16.0 ms | 1.44 | 0.011° |

**Tracking, five synthetic references, all four gains `[PREDICTED]`.** Errors in degrees,
torque in N·m, `%auth` against the authored ±142.2 N·m:

| reference | kp | RMS err | peak err | ss err | peak τ | %auth | %sat |
|---|---|---|---|---|---|---|---|
| step 10° | 60 | 1.141 | 1.457 | 1.136 | 10.47 | 7.4 | 0.0 |
| step 10° | 200 | 0.343 | 0.347 | 0.347 | 34.91 | 24.5 | 0.0 |
| step 10° | 600 | 0.131 | 0.131 | 0.131 | 104.72 | 73.6 | 0.0 |
| step 10° | 2000 | 0.051 | 0.051 | 0.051 | 142.20 | 100.0 | 0.6 |
| step 45° | 60 | 4.991 | 5.206 | 5.206 | 47.12 | 33.1 | 0.0 |
| step 45° | 200 | 1.710 | 1.737 | 1.737 | 142.20 | 100.0 | 0.2 |
| step 45° | 600 | 0.635 | 0.638 | 0.635 | 142.20 | 100.0 | 1.7 |
| step 45° | 2000 | 0.184 | 0.184 | 0.184 | 142.20 | 100.0 | 4.9 |
| sine 0.9 Hz 0–60° | 60 | 10.815 | 18.083 | 6.027 | 5.30 | 3.7 | 0.0 |
| sine 0.9 Hz 0–60° | 200 | 6.074 | 9.624 | 3.220 | 5.61 | 3.9 | 0.0 |
| **sine 0.9 Hz 0–60°** | **600** | **3.523** | **5.361** | **1.710** | **5.75** | **4.0** | **0.0** |
| sine 0.9 Hz 0–60° | 2000 | 1.929 | 2.847 | 0.863 | 5.80 | 4.1 | 0.0 |
| min-jerk 60°/0.3 s | 60 | 13.389 | 32.132 | 6.543 | 12.50 | 8.8 | 0.0 |
| min-jerk 60°/0.3 s | 200 | 7.348 | 19.638 | 2.082 | 17.07 | 12.0 | 0.0 |
| **min-jerk 60°/0.3 s** | **600** | **4.145** | **11.394** | **0.702** | **19.28** | **13.6** | **0.0** |
| min-jerk 60°/0.3 s | 2000 | 2.218 | 6.144 | 0.210 | 19.72 | 13.9 | 0.0 |
| chirp 0.2–8 Hz | 60 | 8.494 | 16.302 | 6.625 | 14.23 | 10.0 | 0.0 |
| chirp 0.2–8 Hz | 200 | 7.389 | 13.905 | 7.336 | 37.53 | 26.4 | 0.0 |
| chirp 0.2–8 Hz | 600 | 5.805 | 12.733 | 6.994 | 80.82 | 56.8 | 0.0 |
| chirp 0.2–8 Hz | 2000 | 3.419 | 8.230 | 4.554 | 111.55 | 78.4 | 0.0 |

Figure: `experiments/gain_sweep_prediction.png`.

Three results in that table are worth more than the rest.

**The steps are the wrong discriminator, and the table shows why.** A step is infinitely
fast, so it asks for infinite torque; the 45° step saturates the authored limit at *every*
gain from 200 up, and the 10° step reaches 100 % authority at kp=2000. Saturation percentages
in the step rows are artifacts of the input, not statements about the leg. Keep the steps as
diagnostics — they are how you see gravity droop and overshoot cleanly — but do not select a
gain on them.

**On smooth references, the residual error is almost pure phase lag.** The linearised
closed-loop magnitude |H| at 0.9 Hz is ≈ 1.000 at all four gains, yet the RMS error still
falls from 10.8° to 1.9°. That error is therefore timing, not amplitude attenuation — visible
in the top-right panel of the figure as an error trace in quadrature with the reference
rather than in phase with it. This has a practical consequence for later: lag can be removed
essentially for free by advancing the reference in time or adding a feedforward term, whereas
buying the same reduction by raising kp costs torque and physical plausibility. Do not raise
kp to fix a problem that is really a time shift.

**kp=60 — the authored value — is not merely imprecise, it is qualitatively wrong here.** Its
2.41 Hz closed-loop bandwidth sits *below* the frequency content of the reference it must
track, and gravity contributes 8.85 N·m/rad, which is 15 % of kp at kp=60 but only 0.4 % at
kp=2000. So at kp=60 gravity is a first-order part of the loop, not a disturbance: it drags
steady state 5.2° off target at 45° flexion and shows up as *negative* overshoot on the step.
The passive pendulum frequency about the knee is 0.925 Hz — which is to say, the leg's own
unforced dynamics sit right on top of the gait fundamental. A servo that soft does not
command the trajectory so much as negotiate with it.

## D. Recommended gain pair

> **kp = 600 N·m/rad, kv ≈ 17.25 N·m·s/rad** (ζ = 0.7), set at runtime on the knee actuator.

Five reasons, in the order they matter.

Bandwidth first: 7.68 Hz is roughly 8× the 0.9 Hz gait fundamental and sits above the ~5–6 Hz
content of a real knee trajectory, so the loop is not the bottleneck on the reference. kp=200
at 4.44 Hz is marginal against that content, and kp=60 at 2.43 Hz is simply inside it.

Accuracy is sufficient and honest: 3.52° RMS with 5.36° peak on the gait-rate sinusoid, and
0.70° steady-state on the fast min-jerk excursion. Against a 60° swing excursion that is
~6 % RMS, and most of it is the removable phase lag discussed above.

Torque demand stays in a defensible band: 13.6 % of authority worst case on a smooth
reference, with **zero** saturation on any smooth reference. Nothing about the result depends
on hitting a limit.

The nonlinearities stop mattering: gravity droop falls from 5.2° at kp=60 to 0.64°, and the
Coulomb friction deadband from 0.382° to 0.038° — an order of magnitude below anything the
experiment will resolve.

And the honest reason to stop at 600 rather than take 2000: kp=2000 buys about 1.6° more RMS
accuracy, and charges 78.4 % of torque authority on the 8 Hz chirp, 10.5 % overshoot once
saturated, and a joint stiffness 6–20× any physically plausible OSL impedance. That last cost
is the real one. A gain that only works because the simulator has no bandwidth limit,
backlash or transmission compliance produces a result that will not transfer to the hardware,
and transfer is the point of the twin. kp=600 is the largest gain that is still arguable as a
physically implementable impedance.

Numerical stability is *not* what limits this: `ω_n·dt = 0.044` even at kp=2000, comfortably
stable. The binding constraints are torque saturation and physical plausibility, which is a
better position to be in — it means the choice is a modelling argument, not a solver artifact.

**This is controller tuning, not a change to the physical OSL model.** kp and kv are
properties of the tracking controller. They are written only into the compiled `mjModel` at
runtime; `models/osl_v2_bench.xml` is never opened for writing. That claim is auditable
rather than asserted: section [8] of `experiments/run_gain_sweep.py` re-reads `forcerange` and
`ctrlrange` after the entire sweep and fails the run if either differs from the authored
values, and it re-runs one configuration to confirm bit-identical output. Mass, inertia,
geometry, joint limits, damping, frictionloss and armature are untouched throughout.

## E. Expected torque demand vs. OSL torque authority

`[PREDICTED]`, knee, against the authored ±142.2 N·m:

| Demand | Peak τ | % authority |
|---|---|---|
| 0.9 Hz gait-rate sinusoid, 0–60° | 5.30–5.80 N·m | 3.7–4.1 % |
| min-jerk 60° in 0.3 s (fastest realistic excursion) | 12.50–19.72 N·m | 8.8–13.9 % |
| loose non-simultaneous upper bound (I·α + b·ω + gravity + friction) | 27.63 N·m | 19.4 % |
| earlier peak-swing estimate | 33.5 N·m | 23.5 % |

The conclusion to report is that **torque authority was never the limiting factor on any
smooth reference**; saturation appears only on artificial step inputs. Because demand sits
below a quarter of authority, the idealizations in the torque model (no torque–speed curve,
no thermal state, no backlash, no transmission compliance) are second-order for *this*
experiment. That is a statement about this bench task, not a general endorsement.

Two caveats must travel with any use of these numbers.

**The fixed base is the central scientific caveat.** `knee_prox` is welded to the world, so
the actuator fights only `I·α + m·g·d·sin θ + b·ω + frictionloss` — it does **not** carry body
weight. Simulated bench torque is therefore *not* comparable to a human knee moment in
stance, where the moment is dominated by the body weight the bench never bears. Kinematic
comparison is valid across 0–100 % of the cycle; torque comparison is loosely valid only in
swing (~60–100 %). Shade and label 0–60 % on every torque plot, and never extrapolate a bench
torque agreement into a claim about stance.

**Required wording whenever the torque authority is described.** `OpenSourceLeg_KA_L1` models
gearbox-related torque authority via knee gain 49.4, ankle 58.4, and ctrlrange ±2.88
motor-shaft torque, giving knee ≈ ±142.27 N·m and ankle ≈ ±168.19 N·m. It does **not** model
the motor as an electrical system, nor current, voltage, torque–speed curve, efficiency,
thermal state, backlash, transmission compliance, device-authored rotor inertia, or actuator
bandwidth/latency. Therefore do **not** describe this as a full motor/gearbox simulation. The
correct wording is: *"the model represents the available joint torque authority derived from
the motor/gear relationship, but idealizes the torque production."* Also do not call the
49.4 / 58.4 / 30 A / 0.096 N·m/A values experimentally verified hardware values — they are
recoverable model derivation values from repository history unless a hardware datasheet
confirms them.

## F. Exact next step for the human gait-tracking experiment

Not started, by instruction. The next step is a single gate, then one script.

**The gate:** run the two commands in §Commands. Item A becomes confirmed or it does not;
item B's expected column names are replaced with real ones; item C's predictions are replaced
with MuJoCo output. If the MuJoCo sweep disagrees with the prediction by more than a few
percent, that disagreement is the next thing to investigate and the gait experiment waits.

**Then, and only then, the tracking experiment**, whose shape is now fully constrained by the
above: take AB19's `levelground` normal-speed strides, keep only those with a finite knee
moment, resample the pre-computed 0–100 % gait phase onto the 0.5 ms bench timestep, verify
the flexion sign against the landmark angles before anything else, clamp the reference into
the model's authored `[−5°, +120°]` ROM and *report* any clamping rather than silently
absorbing it, drive `ctrl` with that reference at kp=600 / kv=17.25, and compare reference
angle, simulated angle, tracking error, torque and angular velocity — with the 0–60 % stance
region shaded on torque plots and excluded from any torque claim.

Two decisions are deferred to that point and should be made explicitly rather than by
default, since both are physics assumptions: whether to advance the reference in time (or add
feedforward) to remove the ~29 ms phase lag, and whether to cross-check on `KA_L1`, whose
`[0°, +120°]` hard stop would clip the terminal-stance hyperextension that our `[−5°, +120°]`
range admits. That clipping difference is exactly the sort of thing that looks like a
controller result and is not one.

---

## Commands

Both are read-only with respect to models. Run from the `osl-mujoco` root.

**1. The gain sweep (uses `osl-mujoco`'s existing venv — mujoco 3.12 + numpy is all it
needs; that venv is not modified):**

```
.venv\Scripts\python.exe experiments\run_gain_sweep.py
```

Writes `build/gain_sweep/gain_sweep_trace.csv` and `gain_sweep_metrics.csv`. It runs nine
preflight checks and **refuses to sweep** if the model does not match expectations, so a
non-zero exit code means a real discrepancy, not a crash. Exit code equals the failure count,
matching `tools/validate_mjcf.py`.

**2. The dataset probe (needs a separate analysis venv — `osl-mujoco/.venv` deliberately has
no scipy/requests/h5py and must not be modified):**

```
py -3 -m venv .venv-analysis
.venv-analysis\Scripts\python.exe -m pip install numpy scipy h5py requests
.venv-analysis\Scripts\python.exe experiments\camargo_probe.py --check
.venv-analysis\Scripts\python.exe experiments\camargo_probe.py --probe <unzip-root> --subject AB19
```

`--check` tests availability (item A). `--probe` prints the seven sections that answer item B
off the real files. Optional: `experiments\gain_sweep_analytic.py --no-plot` in the analysis
venv reproduces the §C prediction tables for side-by-side comparison.

## Files

Created, all new:

- `experiments/run_gain_sweep.py` — the runtime-only MuJoCo sweep (originally written as one
  file, `experiments/gain_sweep_bench.py`; the reusable half now lives in `oslbench/`, see
  `docs/REFACTOR_MAP.md`). numpy + mujoco + stdlib
  `csv` only, so it runs in the existing venv. Logs 18 columns per step including torque
  recorded three independent ways (the `knee_tau` actuatorfrc sensor, `data.actuator_force`,
  and the unclamped analytic `kp*(cmd−q) − kv*qd`) so that the gain injection itself is
  cross-checked rather than trusted.
- `experiments/camargo_probe.py` — availability check + structure/column/unit discovery.
  Discovers rather than assumes; every guess is printed as a guess; averages nothing.
- `experiments/gain_sweep_analytic.py` — the numpy prediction and the figure.
- `experiments/gain_sweep_prediction.png` — the four-panel figure.

Modified: none. Not touched: `models/osl_v2_bench.xml`, the Onshape URDF export, the KA_L1
fragment, all MyoAssist code, the controller integration, `osl-mujoco/.venv`.

Tests run: `py_compile` on all three scripts (clean); an AST arity sweep (clean); the probe's
discovery path exercised end-to-end against a synthetic dataset tree in the correct
`subject/date/mode/sensor` layout, where it correctly located AB19, listed the sensor folders,
read every table, and inferred `knee_angle_r` → "DEGREES, flexion-POSITIVE",
`knee_angle_r_moment` → "N·m/kg (mass-normalised)", `knee_angle_r_power` → "W/kg",
`HeelStrike` → "PERCENT 0-100", `Header` → "SECONDS, dt = 5.000 ms → 200.0 Hz, monotonic",
and detected gait phase at `gcRight/HeelStrike`. The MuJoCo sweep has **not** been executed —
no MuJoCo in my sandbox.

One methodological note on that last test: passing against a synthetic tree proves the
discovery logic is correct, not that the real dataset matches my expectations. Those are
different claims, and only your run collapses the second one.
