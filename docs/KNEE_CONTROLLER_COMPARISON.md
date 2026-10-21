# Supplied knee controller versus current gait controller

The supplied knee gains improve gait tracking with actuator-side derivative feedback, but do not resolve the simulated oscillation with knee-side derivative feedback. Do not restore the original knee-side derivative or replace production settings based on this study alone. No production files or historical results were changed; no commit or push was made.

## What the supplied file establishes

The source is Preetam's supplied `knee_step_test_PID_control_me.py`, preserved verbatim in [the experiment output](../build/knee_hardware_controller_comparison/supplied_knee_controller.txt). Its executable settings are evidence of the supplied configuration, not independent evidence of hardware calibration or successful full-gait operation. Comments call the gains untuned defaults, describe a nominal 1 kHz rate despite `LOOP_HZ=200`, and describe a ratio placeholder of 1.0 despite `GEAR_RATIO_FACTOR=4.61`. This investigation uses executable values.

| Setting | Existing knee gait configuration | Supplied knee script |
|---|---|---|
| Kp, A/degree | 0.75, explicitly ankle-derived | 1.5 |
| Ki, A/(degree second) | 0 | 0.15 |
| Kd, A/(degree/second) | 0.03 | 0.06 |
| Control rate | Fixed 1,000 Hz | Configured 200 Hz, measured elapsed dt |
| Integral clamp | Inactive at Ki=0; generic implementation uses current limit / abs(Ki), or 400 degree-seconds at Ki=0.15 | 50 degree-seconds, giving at most 7.5 A integral contribution |
| Current clamp | +/-60 A simulation command | +/-60 A command; this is not evidence of a safe hardware operating current |
| First derivative | Suppressed | Suppressed by actual code, despite comments anticipating a step kick |
| Feedback | Joint angle for P/I; A/B/C differ in D | Raw external knee AS5048B angle for all terms, with glitch rejection |
| Coordinates | Flexion-positive model reference/measurement; signs +1/+1 | Flexion-negative encoder, motor-command sign -1; display sign -1 is cosmetic |
| Torque constant | Baseline 0.855 Nm/A at actuator output: provisional 0.095 x 9; existing `script` alternative 0.5701 | KT=0.5701 Nm/A, labelled output-shaft; used only in logging |
| External belt ratio | CAD 50/11 = 4.54545, used by compliant plant | 4.61, used only in torque-equivalent logging |

The script's law is `I = Kp*e + Ki*clamp(sum(e*dt), +/-50) + Kd*(e-e_previous)/dt`, in degrees and amperes, with the first D term zero and signed output clipped to +/-60 A. The old A law has the same structure with Ki=0. The knee script has doubled P and D and introduced I; it is not a fundamentally different controller architecture.

The repository explicitly records the gait gains as transferred from the ankle. The original ankle hardware file is not available here for a direct source-to-source comparison. Knee CAN ID 100, encoder address 0x40, offset 120 degrees, -110 to +3 degree limits and negative-flexion convention are supplied knee-specific settings. The PID arithmetic is shared in concept; the torque constant is an actuator property, not inherently a knee or ankle property.

Hardware homing, encoder wrapping, 15-degree glitch rejection, I2C failure handling, hard limits, 20-degree error abort, timing jitter and motor-driver dynamics are not reproduced by this deterministic gait experiment. Initial conditions remain the established aligned gait initialization, not the script's zero-to-minus-15-degree step.

## Conversion and feedback interpretation

Preetam's recalled 0.5702 is within 0.018% of the file's 0.5701. This agreement warrants investigating the existing `script` hypothesis, but is not an independent measurement: both may have the same source. The study uses the exact supplied 0.5701. The shaft reference, motor variant, current convention, efficiency and physical transmission remain unverified.

The PID emits current directly. KT does not alter its hardware equation. In simulation it alters the plant's torque per commanded amp. `ConversionAudit` rescales the command to the existing layer's internal current convention so `actuator torque = chosen output KT * modelled current`; the planetary ratio is not multiplied a second time. This rescaling also affects the layer's current-dependent friction. Changing KT is therefore a conditional plant/conversion hypothesis, not merely relabelling logged torque. The 9:1 internal reduction and CAD belt ratio remain fixed throughout.

- **A:** D uses backward differences of knee tracking error, matching the supplied signal choice.
- **B:** D uses reference backward difference minus instantaneous actuator-output velocity divided by belt ratio.
- **C:** D uses reference backward difference minus actuator-angle backward difference, divided by belt ratio, at the same controller interval.

**All three retain knee-angle feedback for P and I.** B/C never replaced the entire knee feedback loop. B is an ideal velocity diagnostic; C uses sampled actuator angles, but that additional signal and its calibration would still have to be available on hardware.

## Experiment and results

24 real MuJoCo 3.12.0 runs: 2 gain sets x 3 feedback choices x 2 output KT choices x 2 control rates. Each runs one 1.205-second AB19 cycle, 2,410 physics steps at 0.5 ms, with the same model, reference, aligned initialization, drivetrain parameters, CAD belt ratio, ideal current loop and 60 A clamp. Controller updates are 1 ms or 5 ms, with commands held between updates. No parameter tuning was performed.

The isolated adapter extracts only the supplied arithmetic `PID` class using AST; it never imports or executes the hardware script's device initialization. For new-gain B/C it preserves the supplied knee-side P/I, including the 50 degree-second clamp, and changes only D. Production B/C reject nonzero Ki, so the experiment extends them locally rather than modifying production.

Selected results (absolute peaks):

| Gains | KT output | Rate Hz | Feedback | RMS deg | Peak error deg | Lag ms | Belt actuator-side Nm | Knee drive / joint-side belt Nm | Current A |
|---|---:|---:|---|---:|---:|---:|---:|---:|---:|
| Old | 0.855 | 1000 | A | 4.469 | 9.479 | 22.5 | 30.86 | 140.26 | 12.49 |
| Old | 0.855 | 1000 | C | 4.524 | 9.234 | 22.0 | 4.07 | 18.51 | 7.74 |
| Knee | 0.855 | 1000 | A | 16.603 | 49.875 | 94.5 | 2490.62* | 11321.01* | 60.00 |
| Knee | 0.855 | 1000 | B | 2.201 | 4.703 | 10.0 | 4.10 | 18.65 | 8.55 |
| Knee | 0.855 | 1000 | C | 2.186 | 4.685 | 10.0 | 4.10 | 18.65 | 8.56 |
| Old | 0.5701 | 200 | A | 6.711 | 13.371 | 35.5 | 8.28 | 37.66 | 11.95 |
| Old | 0.5701 | 200 | C | 6.846 | 13.465 | 35.5 | 4.28 | 19.47 | 11.16 |
| Knee | 0.5701 | 200 | A | 4.688 | 19.661 | 16.0 | 375.34* | 1706.09* | 60.00 |
| Knee | 0.5701 | 200 | B | 3.529 | 7.201 | 16.0 | 4.18 | 19.00 | 12.33 |
| Knee | 0.5701 | 200 | C | 3.394 | 7.055 | 15.5 | 4.19 | 19.04 | 12.37 |

Full results: [comparison.csv](../build/knee_hardware_controller_comparison/comparison.csv), with configuration, trace and metrics per case in the same output directory. Plots: [0.855](../build/knee_hardware_controller_comparison/comparison_manual_rotor.png) and [0.5701](../build/knee_hardware_controller_comparison/comparison_script.png).

`Lag` is the existing best-fit nonnegative time shift minimizing overlap RMS over 0–299.5 ms, at 0.5 ms resolution. It is a waveform-delay diagnostic, not a frequency-response phase angle; it is especially unreliable as a physical interpretation for oscillatory cases. RMS/peak/lag retain the historical trace convention pairing pre-step reference with post-step angle. Independently computed physical-time RMS is saved separately in `independent_analysis_checks.json`; for example the supplied-rate/KT knee C result remains approximately 3.45 degrees under that alignment. No lag is removed from the reported RMS.

The joint-side belt torque equals `qfrc_applied[knee]` in every logged row (one drivetrain substep), and knee servo force is zero. These are the same applied drive torque, not two independent measurements. Passive, constraint and net inverse-dynamics joint torques are not reported as drive torque. `KT * I * ratio` is only a rigid-equivalent estimate, not the dynamic belt torque or a sensor measurement.

## Interpretation and recommendation

At fixed baseline KT/rate, knee C reduces RMS by about 52% and peak error by about 49%, while joint-side belt torque changes from 18.51 to 18.65 Nm. At fixed 0.5701/200 Hz, changing old C to knee C reduces RMS from 6.846 to 3.394 degrees and lag from 35.5 to 15.5 ms, with torque 19.47 to 19.04 Nm and current 11.16 to 12.37 A. B/C remain below the approximate 0.055-radian belt-fit boundary, with no clipping or authored-ROM violations in all tested gain/KT/rate combinations.

**Every new-gain A case oscillates, clips current and exceeds the belt-fit boundary.** With 0.5701/200 Hz, deflection reaches 0.310 rad, 791 steps (0.3955 seconds) are outside the fit, current clips for 220 steps (0.110 seconds), and the authored model range is exceeded on 28 steps. With 0.855/1000 Hz, maximum deflection is 0.842 rad. Starred torque peaks are polynomial extrapolations, not credible physical torque predictions or evidence that the hardware produces those torques. Do not regard finite simulation numbers as physical stability.

Full-cycle failures intentionally continue for diagnosis without the hardware script's aborts. Applying its nominal position/error checks to the ideal sampled state would first flag the 0.5701/200 Hz knee A case at about 1.165 seconds; the belt fit is already exceeded at 0.6405 seconds. The 0.855/1000 Hz knee A case leaves the fit at 0.2785 seconds and reaches a nominal guard at 0.607 seconds. These are offline flags under the assumed coordinate mapping, not hardware emulation.

The data do not support the hypothesis that borrowing smaller ankle gains alone caused the old torque problem: the supplied larger knee gains make knee-side differentiation worse in this plant. They also do not prove physical knee-side feedback is wrong. The compliant plant, numerical integration, sensor bandwidth/noise, timing and driver dynamics are still approximate; hardware knee-side control may behave differently. The failing extrapolated runs have not been timestep-convergence validated, and their exact peaks should not be used to infer hardware loads.

**Next candidate:** investigate Kp=1.5, Ki=0.15, Kd=0.06 with C, an explicit 50 degree-second integral limit, and the supplied 200 Hz / 0.5701 output-KT hypothesis. Keep 0.855 and knee-side A as documented comparison cases until the torque/current convention is confirmed. Do not change production or restore A yet. Before selecting a hardware-equivalent model, confirm that these are the gains actually used for full gait (not just step tuning), the actual encoder/control timing, output-KT definition and transmission, and the available actuator feedback. Follow with multicycle and timestep/current-loop checks of the candidate; this study establishes only one-cycle simulation behavior. It does not isolate the benefit of Ki from the simultaneous P/D increases.

## Validation and preservation

- Six focused checks passed: supplied PID/sign/limit/reset equivalence; exact A/B/C arithmetic parity at old gains; preserved knee-side P/I for nonzero-Ki B/C; real-MuJoCo 200/1000 Hz scheduling and torque conversion.
- All three old 0.855/1000 Hz A/B/C traces reproduce historical results exactly across all 29 channels and 2,410 rows.
- All 24 traces are finite and independently checked for errors and applied torque; command holds and current-to-torque identity are verified in each run.
- Full suite: **264 passed, zero failed/skipped** with localhost access. The initial sandboxed run had 256 passes and eight socket/worker failures; both logs are retained.
- Preservation inventory: 621 existing source/test/documentation/model/result files unchanged after the runs. Existing uncommitted work is preserved. The two new experiment files and this report are the only intended additions outside the isolated output directory.

Run the focused checks with `.venv/Scripts/python.exe -B experiments/test_knee_hardware_controller_comparison.py <supplied-script-path>`. Reproduce the study using `.venv/Scripts/python.exe -B experiments/run_knee_hardware_controller_comparison.py --hardware-script <supplied-script-path> --outdir build/<new-directory>`. Existing output directories are refused. Logs, supplied-source hash, manifests, independent checks and full-precision traces remain under `build/knee_hardware_controller_comparison/`.
