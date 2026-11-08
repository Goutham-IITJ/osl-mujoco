# Two-joint assembled-leg comparison

This is the new live/offline comparison application. It receives BOTH joints
from `hardware/gait_tracking/telemetry.py`, runs two independent current PIDs and
two compliant drivetrain states against ONE mechanically coupled full-leg model,
and displays both the angle comparisons and simulated variables. Neither the
old knee-only live command nor the hardware-only recorder provides this workflow.

## Run a saved hardware recording with graphs and MuJoCo

From the repository root in PowerShell:

```powershell
.venv\Scripts\python.exe -B -m oslbench.apps.assembled_tracking --run knee_ankle_normal_walk_0.6ms --feedback actuator-difference --ankle-range hardware --view
```

The browser opens at `http://127.0.0.1:8798/`. `--view` also opens the native
MuJoCo model. Replay is paced near source time; `--speed 2` requests twice speed.
Simulation computes the full physics trajectory regardless of display rate.
The dashboard stays available after replay finishes; Ctrl+C closes the server.
Each run writes a new `build/assembled_tracking_*` folder automatically.

Use these `--run` values with the same command:

| Reference condition | Run value |
|---|---|
| Slower normal walking | `knee_ankle_normal_walk_0.6ms` |
| Faster normal walking | `knee_ankle_normal_walk_1.2ms` |
| Ramp ascent | `knee_ankle_ramp_ascent_1ms` |
| Ramp descent | `knee_ankle_ramp_descent_1ms` |
| Stairs down | `knee_ankle_stairs_down_0.35ms` |
| Stairs up | `knee_ankle_stairs_up_0.31ms` |

For all six without interactive windows:

```powershell
.venv\Scripts\python.exe -B -m oslbench.apps.assembled_tracking --all --no-dashboard --feedback actuator-difference --ankle-range hardware
```

`--archive` overrides the default Downloads tracking ZIP. This workflow uses the
recorded targets, not a rebuilt dataset reference. Terrain names identify the
reference waveform: the simulation has no terrain/ground contact or imposed GRF.

## Live lab workflow

1. Use the Pi code package described in `hardware/gait_tracking/README.md`.
2. Set the SAME token in the Pi shell and laptop shell.
3. Start the laptop comparison, then start ONE Pi trajectory controller within
   its 60-second connection wait. The controller's homing/settling precedes samples.

```powershell
$env:OSL_TELEMETRY_TOKEN = 'your-shared-token'
.venv\Scripts\python.exe -B -m oslbench.apps.assembled_tracking --live --host PI_IP_ADDRESS --seconds 120 --feedback actuator-difference --ankle-range hardware --view
```

Use the Pi's actual Wi-Fi address. Run this comparison OR the hardware-only
recorder as appropriate; the comparison already saves the raw stream. A second
recorder is unnecessary. The Pi controller remains independent of the laptop.
Laptop Stop saves and stops the comparison; it does not stop the physical leg.
The duration is a maximum collection period after connection, including wait for
tracking samples. Producer completion triggers final save. Errors/disconnections
save partial data with STOPPED status. Sequence gaps, session changes and unpaired
joint ticks are rejected, rather than filled with fabricated observations.

## Dashboard controls

- Joint: both, knee, ankle.
- Graphs: Comparison / Torques / Current / Belt / PID / Motion.
- Each joint's comparison shows target, hardware and simulation together; other
  panels separate sim/hardware difference and the two target-tracking errors.
- Simulation panels show separate drive/belt/electromagnetic/friction torque,
  requested/applied current, belt deformation/stiffness/energy, PID terms and
  velocities. These values are not physical torque/current measurements.
- Pause freezes display only; logging and physics continue. Stop saves the run.
- Window selects 5 s, 10 s or retained history. DOM cards are created once;
  graph updates do not replace the page or reset scroll. Two tabs can independently
  select comparison versus simulation panels from the same run.

Files: paired `comparison.csv`, full-rate per-joint `simulation.csv`, `summary.json`,
`metadata.json`, `comparison.png`. Live runs additionally save source metadata
and raw batches in `hardware_batches.jsonl`. Offline plots/errors retain source
alignment and do not remove phase lag. Best-fit lag is an exploratory ±200 ms
search with fixed 200 ms edge trimming and 5 ms resolution, separately reported
from raw errors; it is not a network-latency estimate.

## Explicit experimental assumptions

The nominal root is `knee_prox` fixed to world; both ankle and knee position
servos are disconnected and only belt torques are applied. Physical mount and
encoder zero/direction should still be checked. Initial measured angles are
assigned at first reference time, with zero velocities and belt deflections.
Live reference time and per-joint sensor-read time are retained; offline sensor
times are reconstructed from cumulative per-joint dt. CSVs lack validity flags;
finite stored observations are assumed usable. Simulation feedback is always its
own state after initialization. Invalid live encoder readings exclude comparisons.

Confirmed gains: knee 1.5/.15/.06; ankle 2/0/.075. Controller update period 5 ms;
physics timestep 0.5 ms. Both joints share the paper-derived drivetrain parameter
family with CAD belt ratio 50/11, ideal current loop and provisional .5701 Nm/A
output-shaft interpretation. No physical parameters were fitted in these runs.

IMPORTANT: `--ankle-range hardware` explicitly widens the IN-MEMORY joint range
to ±31 degrees to accommodate the supplied reference/safety-limit convention.
The authored CAD range is -30 to +20; it remains unchanged on disk. This is a
diagnostic envelope, not proof of physical travel or a corrected anatomical
zero. Without that flag, the strict CAD range rejects out-of-range targets.
Do not present results as fully validated until the coordinate/range is resolved.

Configuration C (`actuator-difference`) is the new application's explicit default
and was used for the completed matrix. `--feedback legacy` selects the hardware-
style joint-error D; slow normal walking diverged around 0.58 s on the coupled
model, exceeded belt fit and was stopped at .15 rad deflection. Those incomplete
errors/torques must not be compared as full-run predictions. C and hardware
therefore have matching gains but different derivative feedback signals.

## Evidence and validation, 4 November 2026

Completed six C replays: `build/assembled_matrix_20261104_C/`. No clipping or
belt fit exceedance. Four runtime tests check separate sources, negative feedback,
servo disconnection, independent feedback, range rejection and CSV timing.
Executable Node dashboard checks cover selectors, stable cards, pause, labels
and Stop request. Local HTTP/dashboard endpoint and synthetic source-to-live-
simulation/native-viewer integration passed: 60 pairs per joint, COMPLETE,
`build/assembled_validation_20261104/live_result/`. This is local synthetic
validation, not a new physical lab trial. Full browser layout was not inspected
with a browser screenshot; static comparison plots were inspected visually.

Initial development runs used an incorrect inherited motor sign; those folders
(`assembled_first_*` and `assembled_fine_*`) are invalid implementation checks,
not evidence about hardware/model behavior. The adapter now explicitly enforces
negative feedback. The corrected joint-error D run `assembled_normal_20261104_A`
still diverges; the corrected C run and matrix complete. The fine-timestep
development check is not a convergence claim and did not change the default.
