# Knee step-test telemetry package

Copy both `knee_step_test_PID_control_me.py` and `knee_telemetry.py` into the
hardware controller's existing working folder, preserving a backup of the
original controller first. Keep its existing hardware dependencies/environment.
`knee_telemetry.py` uses Python's standard
library, does not open CAN/I2C and has no actuator-command endpoint. The controller
remains responsible for hardware control and shutdown. This server runs **inside
that controller process**, with networking on background threads. No separate Pi
web-server installation is required. The laptop is an HTTP client; its browser
dashboard is a separate loopback-only server.

## Complete edited step controller included

The included `knee_step_test_PID_control_me.py` is the complete supplied step
controller with four marked telemetry additions already inserted. **Do not
insert the blocks below again.** They explain the edits for review. Removing
the four marked blocks exactly restores the original supplied source. Gains,
motor commands, guards, timing settings and CSV/plotting code are preserved.
The controller still requires its original CAN/I2C/driver dependencies. It is
intended for the hardware host, not for execution on the simulation laptop.

Before opening CAN/I2C (server startup can fail, e.g. if its port is occupied):

```python
import os
from knee_telemetry import KneeTelemetryServer

telemetry = KneeTelemetryServer(
    host="0.0.0.0", port=8794,
    token=os.environ["OSL_TELEMETRY_TOKEN"],
    source="physical", angle_sign=-1,
)
```

Inside the PID loop, after `actuators.set_torque(...)` and the soft-error guard,
immediately before `STATE.log["t"].append(t_wall)`:

```python
telemetry.publish(
    time_s=now_t - t0_pid,
    dt_s=dt,
    reference_deg=reference_deg,
    actual_deg=actual_deg,
    requested_current_A=current_cmd,
    commanded_current_A=current_clamped,
    encoder_valid=(not jump_bad and STATE.i2c_fail_streak == 0),
    status="abort_pending" if abort_reason is not None else "running",
)
```

After the existing motor-disable/CAN/I2C cleanup in `finally`, the edited file
calls `telemetry.close()`. Server shutdown follows motor shutdown. Do not
reset `t0_pid` within a telemetry session; create a new server/session for a new
time origin. Call `publish` from one controller thread only.

`actual_deg` comes from the existing calibrated AS5048B magnetic knee encoder
read. The supplied controller may reuse a previous reading after an I2C failure
or rejected jump; `encoder_valid=False` marks those held values. No second
encoder reader should be created. `reference_deg` comes from the controller's
target generator (a step here, a gait trajectory in the final gait script).
Current variables are signed amperes passed to the driver despite the API name
`set_torque`. A successful call is not a measurement of actual motor current.
The CSV's `current_cmd` column contains the clipped value and its
`current_clamped` column is a boolean; the hook deliberately uses local variables.

Raw angles are transmitted unchanged. The display multiplies both target and
encoder by -1 for flexion-positive plots, consistent with this supplied script.
Confirm this calibration/sign for the final gait controller. No torque is
inferred: the 0.5701 Nm/A shaft/current interpretation remains unverified.

## Connect over the same Wi-Fi

The token is a shared password, not an issued API key. Use the same nonempty
ASCII string on both machines, preferably 32 or more random characters. Generate
one with `python3 -c "import secrets; print(secrets.token_hex(24))"` and copy that
value to both terminals. Avoid spaces, quotes and non-ASCII characters.

First, test transport without accessing motors. On the Pi/Linux hardware host:

```sh
export OSL_TELEMETRY_TOKEN='replace-with-a-shared-random-token'
python3 knee_telemetry.py --host 0.0.0.0 --seconds 120
```

This command is a **synthetic connection test**, not the hardware controller.
After testing, stop it before starting the instrumented hardware script on the
same port. Find the hardware host's Wi-Fi IPv4 address (`hostname -I` on Linux,
`ipconfig` on Windows). Permit inbound TCP 8794 only on the trusted private
network. The network must allow device-to-device traffic; guest Wi-Fi can block
it. No router port-forwarding is needed. HTTP and its token are unencrypted;
use this on a trusted LAN or through an SSH tunnel, not an internet-facing port.

On the Windows laptop, from the repository root:

```powershell
$env:OSL_TELEMETRY_TOKEN = 'replace-with-a-shared-random-token'
.venv\Scripts\python.exe -B -m oslbench.apps.knee_telemetry --host 192.168.1.50 --seconds 120 --outdir build\knee_live\session_001
```

Replace the IP with the hardware host address. Dashboard: localhost port 8795.
Use a new output directory each run; existing directories are refused. Output:
`metadata.json`, timestamped raw `batches.jsonl`, and `summary.json` with hardware
target error and current statistics. Token is not saved in these files.

## Run the actual step test

1. Stop the synthetic sender so that port 8794 is free.
2. On the laptop, set the same token and run the receiver command above with
   a new output folder (for example `build\knee_live\step_001`). It waits up to
   30 seconds for the hardware server. The recording duration starts after the
   connection succeeds.
3. Within those 30 seconds, on the hardware host, from the folder containing
   both Python files, run:

   ```sh
   export OSL_TELEMETRY_TOKEN='replace-with-the-same-shared-token'
   python3 knee_step_test_PID_control_me.py
   ```

   Use the existing hardware Python environment. This command executes the
   supplied physical homing and step test; it is not the synthetic demo.
4. The dashboard initially waits for PID samples, plots the step and encoder
   response, then reports disconnection when the controller exits. The laptop
   retains its recording. This is expected for a finite step test. Stop the
   receiver with Ctrl+C or let its requested duration finish.

The original settings use `PRE_STEP_S = 0.0` and `POST_STEP_S = 1.0`. Only the
PID phase is streamed, matching the original CSV logging; homing/hold are not
streamed. Start the receiver first to capture this short test. The last batch
can be missed if the server exits before it is fetched; compare saved sequence
counts with the hardware CSV when assessing completeness. The first connection
and hardware timing still need validation on the actual machines.

For a two-terminal laptop-only test, omit the token and use:

```powershell
.venv\Scripts\python.exe -B hardware\knee_telemetry.py --seconds 120
.venv\Scripts\python.exe -B -m oslbench.apps.knee_telemetry --seconds 30 --outdir build\knee_live\synthetic_001
```

## Timing, data loss and next simulation connection

The default ring retains 4000 ticks (20 seconds at 200 Hz). The laptop polls
batches every 50 ms; it does not poll only the latest value. Gaps, invalid encoder
samples, disconnections and session changes are distinguished. Short outages
can catch up within the ring capacity; older overwritten ticks become explicit
sequence gaps. A session change stops recording rather than joining clocks.
Stale means producer age over 0.5 s (display threshold, not a hardware limit).
Source times drive plots; receipt times measure delivery, not motion. One-way
Wi-Fi latency cannot be measured exactly without clock synchronization.

Publishing does no networking/disk I/O and never waits for its buffer lock. A
busy buffer drops that tick and increments a counter. Python threads still share
the GIL; this is not a hard-real-time guarantee. Compare actual hardware loop-dt
statistics with and without telemetry before accepting it for experiments.

The live plot currently shows **hardware target and encoder**, not a simulated
curve. The next stage supplies the streamed target to the existing knee
simulation, which must use its own simulated feedback and integrate on source
time with a jitter buffer. Encoder motion is the comparison signal, not a
continual overwrite of simulated joint position. That integration needs the
actual gait controller, verified reference/encoder signs, initial conditions
and physical load/setup. Current replay is a separate possible experiment;
do not convert driver amperes to simulated torque before confirming shaft,
ratio and sign conventions. No IMU work is needed for this connection.

## Local verification

```powershell
.venv\Scripts\python.exe -B -m unittest discover -s tests -p test_knee_live_connection.py -v
.venv\Scripts\python.exe -B tests/run_tests.py
```

The connection tests are a separate standard-library unittest suite. They cover
authentication, read-only endpoints, buffer overflow and contention, invalid
samples, session changes, signs, held encoder readings, staleness, catch-up,
recording and refusal to overwrite an existing output directory.
