# Meeting readiness — 26 October 2026

## What can be reported

- Ankle-derived tuning is preserved as a named configuration; a separate knee
  configuration uses Kp=1.5, Ki=0.15, Kd=0.06, integral clamp 50 degree-seconds,
  200 Hz, C feedback and provisional actuator-output constant 0.5701 Nm/A.
- In the controlled C comparison at the same 200 Hz and 0.5701 assumption, RMS
  error changes from 6.846 to 3.394 degrees; peak error 13.465 to 7.055 degrees;
  fitted lag 35.5 to 15.5 ms. Knee-profile peak joint-side belt/drive torque is
  19.042 Nm and peak commanded current 12.372 A, with no current clipping.
  These are prior one-cycle simulation results, not new physical measurements.
- The numerical best tested C case was 2.186 degrees RMS at 1 kHz and 0.855 Nm/A.
  Do not call this the verified hardware configuration. Do not attribute all
  improvement to Ki: P and D also changed. See LIVE_DEMO.md for the controlled table.
- Buffered encoder telemetry, laptop recording, a live plot and the complete
  instrumented step controller are built. A prior 50-second synthetic local run
  recorded 10,073 samples without gaps/duplicates. Physical/Wi-Fi validation is pending.

## Scope distinctions

CSV replay plays recorded observations. Mock telemetry generates synthetic
observations. The new hardware receiver fetches buffered live JSON samples from
the controller's encoder-read/control loop; it does not read the hardware CSV.
It writes its own JSONL recording for evidence.

The knee telemetry command opens a browser plot, not MuJoCo. A measured-angle
kinematic display helper exists, but is not connected to this receiver. Such a
display would mirror measurements, not predict motion. Independent simulation
needs the shared target as input, its own feedback/state, time alignment and a
separate measured-versus-simulated comparison.

Same gains alone do not isolate the plant gap. Hardware uses knee error
differences for D; simulation C uses actuator-angle differences for D. Controller
law, rate, units, signs, limits, initialization and physical load must be matched
or explicitly treated as separate contributions to the observed difference.

## Checklist and priority

| Work | Status | Next action |
|---|---|---|
| Shared PID and separate ankle/knee profiles | Complete locally | Preserve both configurations |
| Controlled knee gait comparison | Complete for one cycle | Present matched-setting table |
| Step controller + separate telemetry module | Built and packaged | Hardware operator runs supplied package |
| Encoder origin, held readings and signs | Source inspected | Confirm calibration on physical leg |
| Buffered transport/auth/session/gap tests | Passed locally | Synthetic sender across actual Wi-Fi first |
| Physical step telemetry | Pending hardware access | Compare laptop samples with hardware CSV |
| Multi-graph redraw/scroll issue | Repeated DOM replacement removed | Manual scroll check on demo browser |
| Independent MuJoCo + live-reference comparison | Pending | Implement source-time buffer and simulation adapter |
| Optional measured-angle MuJoCo display | Helper only | Wire separately and label as mirroring |
| Hardware law versus C feedback mismatch | Unresolved | Investigate matched knee-side law/model; retain C diagnostic |
| 0.5701 torque/current interpretation | Provisional | Confirm shaft, current definition, ratio and sign |
| Actual gait controller and gains | Awaiting supplied gait source | Instrument after step pipeline validation |
| Final-sample delivery at shutdown | Known limitation | Add bounded post-motor-stop drain/ack strategy |
| Recording/reconnection completeness | Local cases tested | Test Wi-Fi outage and restart with sample counts |
| Multi-cycle/timestep/current-loop sensitivity | Further work | Run controlled tests before declaring robustness |
| MuJoCo viewer smoothness | Not benchmarked for reported lag | Measure frame times and simulation/wall-clock ratio |
| IMU-to-knee reconstruction | Deferred by project decision | Not needed for encoder route |

## Before the meeting

1. Demonstrate the selected knee simulation and its saved metrics.
2. Demonstrate synthetic telemetry, explicitly labelled synthetic; verify scroll
   behaviour in the older multi-graph dashboard. Do not claim physical connection.
3. Present the pipeline diagram and this pending checklist. Agree the next
   milestone is physical step validation followed by independent parallel simulation.
4. Ask for hardware confirmation of gains/law, encoder calibration, control rate,
   current/shaft convention and bench load. Schedule physical work when access returns.

Avoid another gain sweep solely to improve tomorrow's displayed error. First
establish comparability. Viewer lag and tracking phase lag are different: measure
render/snapshot/physics costs separately, then optimize the bottleneck. Keep the
physics timestep fixed; decimate display updates rather than discarding recorded
samples or changing controller timing. The current viewer already batches physics
steps per frame; per-tick rendering is not established as its bottleneck.

## Dashboard correction in this update

The replay/mock page previously removed all table rows and graph cards every
80 ms. It now retains nodes per channel, updates cells/canvases in place, removes
only absent channels, uses a fixed table layout and avoids resetting canvas size
unless dimensions change. Display polling is 100 ms; acquisition and recordings
are unchanged. An executable JavaScript regression checks 500 updates of eight
graphs, value/quality changes and channel removal. All eight replay UI tests pass.
Browser-level scroll behaviour and viewer frame timing are not measured by this test.

No physical motor execution, controller tuning, commit or push was performed.
