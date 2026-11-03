# Assembled-leg gait telemetry handoff

One `telemetry.py` is shared by all five controller scripts. Copy this complete
folder to the Pi, for example `/home/legpi/test_codes/gait_tracking`. Keep the
existing working hardware environment and dependencies. Run ONE controller at
a time. The scripts contain real homing and motor commands for BOTH joints;
`telemetry.py` itself has no motor-command endpoint and reads no CAN/I²C devices.

The new module supports joint-labelled streams in one process and on one port.
It does not require a separate server command on the Pi: the controller starts
it automatically. It uses the Python standard library only.

## What changed

Each supplied controller has three marked `TRACKING TELEMETRY` additions:

1. Start the HTTP server before opening CAN/I²C. A missing token or occupied
   port fails before the controller opens the hardware.
2. Publish each joint's already-read encoder, target, currents, PID terms,
   validity and timestamps after its motor command/guard, before existing logging.
3. After existing motor-disable/CAN/I²C cleanup, mark producer completion, allow
   two seconds for final collection, then close the server.

Removing these blocks reproduces the supplied source exactly, except the two
normal-walk dataset paths changed from absolute Pi paths to paths relative to
this folder. Other four scripts already use relative dataset paths. Gains,
homing, signs, guards, current limits, spline/stride selection, TIME_SCALE,
CSV/plot generation and motor commands remain as supplied. `manifest.json`
records source and modified controller hashes.

All five original code snapshots keep their own settings. In particular normal
walk and stairs-up currently have TIME_SCALE=2. They are not automatically
configured to reproduce every historical result-folder label. The operator
must review the selected script's existing USER SETTINGS for the intended run.

## On the Pi

Activate the SAME environment used for the working hardware scripts, then:

```bash
cd /home/legpi/test_codes/gait_tracking
export OSL_TELEMETRY_TOKEN='replace-with-your-shared-token'
python knee_ankle_trajectory_PID_control_me_normal_walk.py
```

Choose one of the other included script filenames for ramp ascent/descent or
stairs up/down. Use the hardware operator's normal control/stop procedure.
The existing controller still performs homing and settling before publishing
tracking samples. Its existing CSV/plots are saved according to its own run-folder
logic; the laptop recording is additional. Do not launch `telemetry.py` separately
on the Pi when a real controller is running.

Both systems need the same nonempty token. The LAN server binds `0.0.0.0:8794`.
The token is a shared arbitrary secret; it is not a motor/device identifier.
Authentication is bearer-token HTTP, without encryption; use the intended lab LAN.

## On the laptop (repository root, PowerShell)

Start this recorder before the operator starts the Pi controller; it waits up
to 60 seconds for the server:

```powershell
$env:OSL_TELEMETRY_TOKEN = 'replace-with-your-shared-token'
.venv\Scripts\python.exe -B experiments\record_tracking_telemetry.py --host PI_IP_ADDRESS --seconds 120
```

Use the Pi's Wi-Fi IP address, not 127.0.0.1, for a remote hardware test. The
duration must cover homing/hold plus tracking; increase it for longer trials.
The recorder saves automatically into a new `build/tracking_received_*` folder:

- `metadata.json`: session, signs, schema and source information.
- `hardware_batches.jsonl`: raw HTTP batches with laptop receipt elapsed time.
- `knee_telemetry.csv` and `ankle_telemetry.csv`: raw signed, timestamped samples.
- `summary.json`: counts, final sequence and COMPLETE/INCOMPLETE status.

COMPLETE means the final published sequence was received; it is not proof of
successful physical tracking or a successful simulated comparison. Dropped or
out-of-order samples produce an incomplete recording. Ctrl+C on the laptop
saves what it has; IT DOES NOT STOP THE PHYSICAL LEG. Disconnection likewise
does not stop the hardware controller.

## What is sent

One HTTP service, two joint-labelled streams, schema `osl-joint-telemetry-v1`.
The laptop polls `/v1/metadata` and `/v1/samples?after=N&limit=1000` on port8794.
Each sample has a global sequence, joint sequence and shared control-tick index.
There are approximately 200 samples/s per enabled joint (400 entries/s total).
The bounded buffer holds 8000 entries. Publish does not wait for a network
request or buffer lock; dropped ticks are visible as sequence gaps. Python is
soft real-time and the added work can affect loop timing; recorded dt must be
checked on the Pi.

`reference_time_s` is shared loop-start time when the reference spline was
evaluated; `time_s` is the later encoder-read time; `dt_s` is that joint's actual
controller interval. Both clocks share the controller's PID time origin.
Angles/velocity/PID/current signs are the controller's ORIGINAL signs, not the
cosmetic signs in its old CSV output. Metadata supplies display angle/current
multipliers (knee angle -1/current +1; ankle angle +1/current -1). Current is
requested/commanded current, not measured motor current. Validity flags mark
held or glitch-rejected encoder samples.

## Compatibility and local check

This is an extension for both joints. It is not a drop-in stream for the old
knee-only laptop receiver (`osl-knee-telemetry-v1`). Keep the old step package
unchanged. Use `record_tracking_telemetry.py` for this package. That recorder is
hardware-data acquisition only: no graphs, MuJoCo, or simulated trajectory yet.
The assembled-leg simulation/dashboard connection is the next bounded task.

For a hardware-free two-terminal test on the laptop, use a loopback synthetic
publisher instead of any controller script:

```powershell
$env:OSL_TELEMETRY_TOKEN = 'replace-with-your-shared-token'
.venv\Scripts\python.exe -B hardware\gait_tracking\telemetry.py --seconds 15
```

In the second terminal set the same token and run:

```powershell
$env:OSL_TELEMETRY_TOKEN = 'replace-with-your-shared-token'
.venv\Scripts\python.exe -B experiments\record_tracking_telemetry.py --seconds 30
```

Local validation: controller-source preservation and authenticated HTTP/channel/
drop tests passed; a three-second synthetic two-joint run delivered 599 samples
per joint with final COMPLETE status. No hardware controller was executed.

The older simulation's ankle coordinate/range mismatch and exact assembled
mount orientation remain to be resolved before a trustworthy sim-to-real gait
comparison. These do not prevent collecting the controller's raw observations.
