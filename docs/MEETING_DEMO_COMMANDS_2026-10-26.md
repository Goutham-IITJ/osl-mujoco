# Meeting demo commands

Run these in PowerShell from the repository root. Stop each demo with Ctrl+C or
close its viewer before moving on. Use new output folders to preserve old results.

```powershell
Set-Location 'C:\Users\Goutham A.S\Documents\GitHub\osl-mujoco'
```

## 1. Show the selected knee simulation

```powershell
.venv\Scripts\python.exe -B experiments\run_current_controlled_gait.py --controller knee --view --start-paused --speed 0.25 --zoom 1.2 --dashboard web
```

Press Space to run, R to restart, and Esc to exit. This opens MuJoCo and the gait
dashboard. It uses knee gains, C feedback, 200 Hz and provisional 0.5701 Nm/A.
The slow-motion setting changes playback pacing, not the physics timestep.

## 2. Reproduce the fair gain comparison

Both runs use C, 200 Hz and 0.5701 Nm/A. The timestamp creates fresh output folders.

```powershell
$runStamp = Get-Date -Format 'yyyyMMdd_HHmmss'
.venv\Scripts\python.exe -B experiments\run_current_controlled_gait.py --controller ankle --feedback-mode actuator-difference --control-hz 200 --kt script --dashboard none --outdir "build\meeting_$runStamp\ankle_matched"
.venv\Scripts\python.exe -B experiments\run_current_controlled_gait.py --controller knee --dashboard none --outdir "build\meeting_$runStamp\knee"
```

Expected RMS errors are approximately 6.846 and 3.394 degrees. The original ankle
profile without overrides is a different baseline, not this matched comparison.

## 3. Test the multi-graph dashboard and scrolling

```powershell
.venv\Scripts\python.exe -B experiments\run_mock_telemetry.py
```

This is synthetic observation telemetry, not CSV replay or MuJoCo. Scroll down
while values/receipt ages update. The mock deliberately includes missing/stale
data and disconnection; these are test conditions. Close with Ctrl+C.

## 4. Demonstrate the new network telemetry pipeline without hardware

Open two PowerShell terminals in the repository root. Clear the token in both
for this loopback-only demo. Start terminal A first, then B promptly.

Terminal A — synthetic sender:

```powershell
Remove-Item Env:OSL_TELEMETRY_TOKEN -ErrorAction SilentlyContinue
.venv\Scripts\python.exe -B hardware\knee_telemetry.py --seconds 180
```

Terminal B — receiver and live graph:

```powershell
Remove-Item Env:OSL_TELEMETRY_TOKEN -ErrorAction SilentlyContinue
$runStamp = Get-Date -Format 'yyyyMMdd_HHmmss'
.venv\Scripts\python.exe -B -m oslbench.apps.knee_telemetry --seconds 120 --outdir "build\meeting_$runStamp\synthetic_telemetry"
```

The browser uses port 8795. It should show LIVE and synthetic, target and encoder
curves, increasing sample counts and zero missing samples in a healthy run.
It saves metadata.json, batches.jsonl and summary.json. No MuJoCo window or motor
operation is involved. This is the telemetry demo to show while hardware is unavailable.

## 5. Optional checks

```powershell
.venv\Scripts\python.exe -B tests/run_tests.py -k replay_ui
.venv\Scripts\python.exe -B -m unittest discover -s tests -p test_knee_live_connection.py -v
```

Expected: 8 replay/UI tests and 12 connection tests pass. The executable graph
regression requires Node.js. Local socket access is needed for the network tests.

Physical step execution is a separate procedure in [hardware instructions](../hardware/README.md).
Do not present synthetic curves as physical measurements.
