# Meeting demo and first hardware connection

Run laptop commands from `C:\Users\Goutham A.S\Documents\GitHub\osl-mujoco` in PowerShell. This guide uses the existing code; it does not change gains or hardware behavior.

## 1. Explain the model with the offline knee demo

```powershell
.venv\Scripts\python.exe -B experiments\run_current_controlled_gait.py --controller knee --view --start-paused --speed 0.25 --zoom 1.2 --dashboard web
```

This uses AB19 knee-angle reference, knee gains 1.5/0.15/0.06, 200 Hz, feedback C and provisional 0.5701 Nm/A. Explain that slow viewing changes presentation pacing, not the physics timestep. Use the dashboard's playback controls. The native model and the plots share the simulation runtime.

To show the preserved ankle-derived profile on the same knee bench, close the preceding viewer/server first:

```powershell
.venv\Scripts\python.exe -B experiments\run_current_controlled_gait.py --controller ankle --view --start-paused --speed 0.25 --dashboard web
```

This changes the profile, not the model joint. The default ankle profile changes rate, constant and feedback as well as gains, so use the controlled comparison table in the briefing when attributing improvements.

Optional lowest-RMS historical C configuration (a simulation candidate, not hardware-matched):

```powershell
.venv\Scripts\python.exe -B experiments\run_current_controlled_gait.py --controller knee --control-hz 1000 --kt manual_rotor --feedback-mode actuator-difference --view --start-paused --dashboard web
```

## 2. Prove the integrated pipeline locally first

Start the **laptop receiver first** using Terminal B below. Model/viewer preparation happens before connection. Wait for `READY` in that terminal, then run this publisher in Terminal A within 30 seconds. The publisher does not access physical devices.

```powershell
Remove-Item Env:OSL_TELEMETRY_TOKEN -ErrorAction SilentlyContinue
.venv\Scripts\python.exe -B hardware\knee_telemetry.py --seconds 180
```

Terminal B (run this first):

```powershell
Remove-Item Env:OSL_TELEMETRY_TOKEN -ErrorAction SilentlyContinue
$runStamp = Get-Date -Format 'yyyyMMdd_HHmmss_fff'
.venv\Scripts\python.exe -B -m oslbench.apps.live_knee_comparison --seconds 120 --view --outdir "build\live_comparison_$runStamp"
```

Show the native model, [comparison page](http://127.0.0.1:8796) and [simulation telemetry](http://127.0.0.1:8797). They open automatically. The source must say synthetic. Show separate torque/belt/current/PID categories, pause one display, resume, then stop the laptop run and inspect its saved CSVs. This is a software demonstration, not an accuracy result.

Regenerate `$runStamp` for every run. A `FileExistsError` means the output directory already exists; preserve it and generate a fresh timestamp. Close old runs to free dashboard ports.

## 3. Test Wi-Fi without motors

On the hardware host, keep `knee_telemetry.py` alongside the supplied edited controller. In its existing Python environment:

```sh
export OSL_TELEMETRY_TOKEN='replace-with-the-shared-random-token'
hostname -I
python3 knee_telemetry.py --host 0.0.0.0 --seconds 180
```

Use the Wi-Fi IPv4 address from that host. Both machines need a network that allows device-to-device connections and inbound TCP 8794 on the hardware host. A token is a shared nonempty ASCII password, not an issued key. Avoid quotes/spaces in the actual token; generate one if needed with `python3 -c "import secrets; print(secrets.token_hex(24))"`. Keep it out of screenshots and paper artifacts. HTTP here is intended for the trusted lab network.

Laptop terminal, replace the example IP and token:

```powershell
$env:OSL_TELEMETRY_TOKEN = 'replace-with-the-shared-random-token'
Test-NetConnection -ComputerName 192.168.1.50 -Port 8794
$runStamp = Get-Date -Format 'yyyyMMdd_HHmmss_fff'
.venv\Scripts\python.exe -B -m oslbench.apps.live_knee_comparison --host 192.168.1.50 --seconds 120 --view --outdir "build\wifi_synthetic_$runStamp"
```

Pass criteria: correct source identity, increasing timestamps/sequences, no missing samples, both pages receive updates, independent model moves, and files save. A working TCP test alone does not test token authentication or sample validity. Inspect backlog/unprocessed counts in the summary. If sequence gaps occur, investigate connectivity, publishing and receiver speed before treating the record as complete.

Stop the synthetic sender and laptop run before using the physical controller on the same port.

## 4. First physical step test

Have the hardware operator use the established bench setup, limits and stop procedure. Confirm the supplied edited script matches the controller they normally use. Preserve the original file and hardware CSV. Do not run the hardware script on the laptop.

Start the laptop first (same shared token still set), and wait for `READY` after model/viewer preparation:

```powershell
$runStamp = Get-Date -Format 'yyyyMMdd_HHmmss_fff'
.venv\Scripts\python.exe -B -m oslbench.apps.live_knee_comparison --host 192.168.1.50 --seconds 60 --view --outdir "build\hardware_step_$runStamp"
```

Within the 30-second connection window **after READY**, the hardware operator runs:

```sh
export OSL_TELEMETRY_TOKEN='replace-with-the-shared-random-token'
python3 knee_step_test_PID_control_me.py
```

The edited script starts its own telemetry server; do not run another `knee_telemetry.py` server simultaneously. It publishes target and the existing encoder measurement from the PID loop. No additional encoder process is needed. Confirm the source is **physical**, encoder validity is sensible, and the sign agrees with actual knee flexion.

The laptop may remain open after the short hardware trial while showing disconnection/stale status. Stop and save the laptop run. Its stop button does not stop the motor-side program. Check the final hardware CSV against received sequence/time coverage: server shutdown can leave final samples unfetched. Treat the captured overlap as partial if appropriate; do not imply all commanded motion was captured.

## 5. Inspect the evidence before tuning anything

```powershell
Get-ChildItem "build\hardware_step_$runStamp"
Get-Content "build\hardware_step_$runStamp\summary.json"
Get-Content "build\hardware_step_$runStamp\metadata.json"
Get-Content "build\hardware_step_$runStamp\comparison.csv" -TotalCount 4
Get-Content "build\hardware_step_$runStamp\simulation.csv" -TotalCount 4
```

Check encoder validity, sequence gaps, time coverage, source lag/backlog, clipping, finite values and belt-fit flags before discussing RMS. Retain raw error and timing; do not shift curves to make them look better. Record bench fixture, initial knee/ankle position, load and controller version alongside the trial. Current comparison includes different derivative feedback in hardware and simulation, so label it a combined controller/plant discrepancy.

Move to the main gait controller only after this step pipeline works. That requires adding the same publish hook to the actual gait script after reviewing its target, encoder, units and scheduling; replacing the step target with a gait assumption in the laptop is not sufficient.

## 6. Repeatable local checks

```powershell
.venv\Scripts\python.exe -B -m unittest tests.test_live_knee_comparison tests.test_knee_live_connection
.venv\Scripts\python.exe -B tests\run_tests.py
```

The first command passed 19 tests in this review. Network tests require localhost socket access. These tests do not prove the physical controller or Wi-Fi connection is working.
