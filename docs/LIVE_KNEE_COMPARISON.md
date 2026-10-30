# Live hardware and independent simulation comparison

Use this integrated application instead of running the two old mock tools.
One process consumes the hardware target stream, runs the shared knee PID and
MuJoCo drivetrain, records results, and publishes two browser views:

- Port **8796**: target, measured hardware knee and independent simulated knee
  on one graph; simulation-minus-hardware error, both target errors, RMS and peak
  difference on the same source-time axis.
- Port **8797**: simulation PID terms, currents, torques, knee/actuator velocities,
  belt deflection, stiffness, energy and step/clipping/model-fit diagnostics.
- `--view` additionally opens the same simulation model in the native MuJoCo
  viewer. This is not a third simulation and does not mirror the encoder.

The dashboards link to each other and use the same session and integrated state.
All torque/belt/current-loop quantities on the simulation page are model outputs.
Hardware commanded current is logged separately and is not measured current.

## Display controls and plot layout

Each chart has source time in seconds on X, named quantities in its title and
units on Y, with numbered ticks and a grid. The motion comparison overlays
hardware and simulation; its target curve is optional via a checkbox. Errors
are separate charts. Simulation telemetry uses one quantity per plot, including
four separate torque charts. Select Torques, Belt, Current, PID, Motion or All.

**Pause display** freezes only that browser view. Acquisition, independent
simulation and CSV recording continue. **Resume live** jumps to the newest
available view. The time-window selector shows 2, 5, 10 or 20 seconds, limited
by retained history. It also works on the frozen snapshot. The two pages can
be paused independently.

**Stop laptop run and save** stops this laptop application and closes both local
dashboards and the viewer after saving. It asks for confirmation and explicitly
does not stop the physical hardware controller. Ctrl+C in the laptop terminal
also saves and stops. Pausing or selecting a category never deletes samples.

The recording folder is displayed on both pages. A timestamp folder is created
by the example command's `$runStamp` and `--outdir`, not automatically by the
application. An explicitly supplied folder must be new; existing folders are
refused. Use millisecond timestamps if launching multiple runs in one second.

## Demonstrate without hardware

Startup update (30 October): run the laptop command in terminal B first. It prepares
the model and native viewer before receiving data. Once it prints `READY`, start
the synthetic publisher in terminal A within 30 seconds. Previously viewer startup
blocked processing after reception began and could exhaust the 4,000-sample queue.
The bounded queue, source sample checks and physics timestep remain unchanged.
The native viewer now refreshes at most 10 Hz and yields its refresh while source
lag exceeds the catch-up threshold. This skips display refreshes only, not source
samples, controller updates, CSV rows or physics steps. Per-stage wall times are
recorded in `summary.json` under `processing_wall_seconds`.

Local validation on 30 October with the real viewer and both dashboard endpoints
polled for a 60-second run processed 120,450 physics steps and 12,046 comparison
samples. Final source lag was 85 ms, with 17 buffered samples remaining at the
duration cutoff; the bounded queue did not overflow. This tests synthetic local
operation, not the physical hardware or browser rendering performance.

In terminal A, from the repository root:

```powershell
Remove-Item Env:OSL_TELEMETRY_TOKEN -ErrorAction SilentlyContinue
.venv\Scripts\python.exe -B hardware\knee_telemetry.py --seconds 180
```

In terminal B (start first, then wait for READY):

```powershell
Remove-Item Env:OSL_TELEMETRY_TOKEN -ErrorAction SilentlyContinue
$runStamp = Get-Date -Format 'yyyyMMdd_HHmmss_fff'
.venv\Scripts\python.exe -B -m oslbench.apps.live_knee_comparison --seconds 120 --view --outdir "build\live_comparison_$runStamp"
```

Both browser pages open automatically. Synthetic source labels remain visible.
The synthetic encoder is an artificial signal: its error against MuJoCo tests
the comparison machinery, not physical accuracy. Ctrl+C stops and saves.

## Connect the physical step controller

Set the identical `OSL_TELEMETRY_TOKEN` in both terminals. Start the laptop
command below first; start the already supplied hardware step controller within
30 seconds after `READY` appears (model/viewer preparation comes first).
Replace the example IP with the hardware computer's Wi-Fi IP.

```powershell
$env:OSL_TELEMETRY_TOKEN = 'your-shared-token'
$runStamp = Get-Date -Format 'yyyyMMdd_HHmmss_fff'
.venv\Scripts\python.exe -B -m oslbench.apps.live_knee_comparison --host 192.168.1.50 --seconds 60 --view --outdir "build\live_comparison_$runStamp"
```

No new motor-side code is required for this integration. Hardware retains its
controller and shutdown responsibility. The laptop sends no motor commands.

## Timing and interpretation

- The live factory does not load AB19. Only streamed target angles drive it.
- The first valid measured angle initializes the knee once. Velocity and belt
  deflection start at zero; these are assumptions, not measured internal states.
- The existing knee profile is reused: 1.5/0.15/0.06 gains, integral limit 50,
  C derivative feedback, 200 Hz, provisional 0.5701 Nm/A and fixed 0.5 ms physics.
  The hardware script's knee-error derivative still differs from C. The error
  combines controller and plant differences; it is not isolated plant validation.
- Targets are held between received source samples. A default 100 ms delivery
  buffer smooths batches; simulation time advances from source timestamps,
  never packet arrival intervals. Incoming targets take effect at the next physics
  grid boundary (within 0.5 ms). Simulated angle is interpolated between bracketing
  states only for comparison at the exact hardware timestamp. No phase shift is
  fitted away, and no cyclic gait percentage is invented for a step test.
- No data means no further physics advance. Gaps after initialization, source
  intervals exceeding 100 ms, session changes, nonfinite simulation states and
  receiver-queue overflow stop the experiment explicitly. An initially truncated
  stream is recorded in metadata; use a complete new session for validation.
- An invalid encoder reading is omitted from paired error metrics. Simulated
  feedback remains independent. A belt deflection outside +/-0.055 rad is flagged
  because it exceeds the fitted belt-law range, not a known physical safety limit.
- Display publication is 5 Hz and viewer synchronization is capped at 30 Hz;
  every physics step is still recorded. The status shows pending samples and
  source lag, with CATCHING UP when the simulation falls behind. Ending a run
  does not pretend queued samples were simulated; the summary reports them.

## Saved evidence

Each new output folder contains `metadata.json`, raw `hardware_batches.jsonl`,
full-rate `simulation.csv`, paired `comparison.csv`, and `summary.json`.
Angles are sampled at source times; torque/belt quantities describe their logged
step-input times and positions describe post-step times. The full recording is
not reduced to the plotted recent history. Existing output folders are refused.

Tests:

```powershell
.venv\Scripts\python.exe -B -m unittest discover -s tests -p test_live_knee_comparison.py -v
.venv\Scripts\python.exe -B tests/run_tests.py
```

Local tests establish the software path only. Physical calibration, load matching,
current/shaft conventions and the actual Wi-Fi experiment remain unverified.
