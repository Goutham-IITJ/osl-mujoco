# Phase 6F-B: passive IMU telemetry over HTTP

Implemented and tested with injected readers and loopback HTTP. **No physical Pi,
USB IMU, installed MSCL runtime or hardware mounting was available for validation.**
No motor interface, ROS, MuJoCo state mirroring, gait reference generation or
predictive comparison is implemented. Orientation is not a knee encoder reading.

## Library decision and verified sources

Research checked 2026-10-19:

- [GV7 software guide](https://s3.amazonaws.com/files.microstrain.com/GV7_User_Manual/user_manual_content/software/Software.htm)
  describes MIP SDK as C/C++ and recommends it for production. It also documents
  MSCL Python support and warns of possible future inertial-product EOL.
- [Current HBK MIP SDK](https://github.com/HBK-MicroStrain/mip_sdk) supplies C/C++
  serial support on Linux. It is the longer-term production option, but requires
  a native reader/bridge for this Python application. An arbitrary PyPI `mip`
  package is not that SDK.
- [Current HBK MSCL integration guide](https://github.com/HBK-MicroStrain/MSCL/blob/main/guides/Integration.md)
  specifies `pymscl` and `import mscl`.
  [Published pymscl 68.6.0](https://pypi.org/project/pymscl/68.6.0/) requires Python
  >=3.11; the Linux ARM64 wheel is tagged `manylinux_2_39_aarch64`. This is **not**
  proof it will install on the team's Pi OS. Check Python, architecture and glibc.
  No Pi software version is currently known. An older glibc or 32-bit OS needs a
  compatible official build or a vendor-supported source build; do not force an
  incompatible wheel. No packages were installed on Windows for this work.
- The reader uses official MSCL APIs inspected at source revision
  `11a12423b5cf22b87c5bc883ef5dd32d43ffaf71`:
  [inertial example](https://github.com/HBK-MicroStrain/MSCL/blob/11a12423b5cf22b87c5bc883ef5dd32d43ffaf71/examples/python/inertial/main.py),
  [packet API](https://github.com/HBK-MicroStrain/MSCL/blob/11a12423b5cf22b87c5bc883ef5dd32d43ffaf71/src/mscl/MicroStrain/MIP/Packets/MipDataPacket.h),
  [shared timestamps](https://github.com/HBK-MicroStrain/MSCL/blob/11a12423b5cf22b87c5bc883ef5dd32d43ffaf71/src/mscl/MicroStrain/MIP/Packets/MipSharedDataFields.h),
  [official field-decoder tests](https://github.com/HBK-MicroStrain/MSCL/blob/11a12423b5cf22b87c5bc883ef5dd32d43ffaf71/tests/src/Test_InertialFieldParser_NAV.cpp).
  The older inertial example still uses uppercase `MSCL`; the current installation
  guide uses lowercase `mscl`. This implementation targets the current package,
  with a clear failure if its native extension/API is unavailable.

**Decision:** use MSCL for this small Python acquisition phase, isolated behind
`MsclReader.read(timeout_ms)` / `close()`. Replacing this reader with a supported
MIP SDK bridge later does not require changing the HTTP client or presentation.
No custom binary parser is implemented. Official examples' configuration and
start-sampling commands are deliberately not copied.

## Components and scope

| Component | Responsibility |
|---|---|
| `oslbench/telemetry/microstrain.py` | Linux-only MSCL passive decoded-field reader; advisory device lock |
| `oslbench/telemetry/imu_service.py` | One acquisition thread, locked latest-channel cache, GET-only endpoints |
| `oslbench/telemetry/imu_schema.py` | Strict versioned JSON validation; separate from replay-v1 |
| `oslbench/telemetry/http.py` | Background HTTP worker implementing existing TelemetrySource; thin presentation subclass |
| `experiments/run_imu_service.py` | Pi process lifecycle |
| `experiments/run_imu_telemetry.py` | Laptop client plus existing ReplayWebDashboard |

The service uses Python's standard-library HTTP server rather than introducing
Flask. Praveen's unavailable plotting application is not assumed to provide an API.
The server is for this trusted-network read-only workflow, not public hosting.
It does not use a Flask reloader or multi-process workers. The per-device `flock`
prevents another copy of this service reading the same resolved USB path; foreign
applications may not honor the advisory lock. Stop other IMU readers first.
No motor port, smbus, actuator interface or motor-process scheduling is touched.
Actual Pi CPU/USB contention still needs checking alongside the motor workload.

## Pi launch

From this repository on the Pi, inspect the actual environment first:

```bash
python3 --version
uname -m
ldd --version
ls -l /dev/serial/by-id/
```

Only on a compatible Pi environment, create/use a separate acquisition environment:

```bash
python3 -m venv .venv-imu
.venv-imu/bin/python -m pip install -r requirements-imu-pi.txt
.venv-imu/bin/python -c "import mscl; print(mscl.__file__)"
.venv-imu/bin/python -B experiments/run_imu_service.py \
  --device /dev/serial/by-id/ACTUAL_DEVICE_ID \
  --mounting "Unknown; not calibrated to prosthesis" \
  --bind 127.0.0.1 --port 8793
```

Replace the USB path with the actual device. Verify serial permissions with the Pi
administrator. The reader calls `Connection.Serial`, `InertialNode` and bounded
`getDataPackets(100)`; it does not idle, reset, enable streams, change rates, change
frames or save settings. The existing device stream must contain filter Euler
and/or quaternion fields. A silent/wrong stream remains WAITING; it is not replaced
with synthetic data. Configure the actual sensor separately with the team only if
needed, then restart this read-only service.

Recommended access: keep the service loopback-only and establish a laptop SSH tunnel:

```powershell
ssh -N -L 8793:127.0.0.1:8793 PI_USER@PI_ADDRESS
```

Alternatively bind to the Pi's **specific RFC1918 IPv4 address**, set
`OSL_IMU_TOKEN` to a random secret of at least 24 characters in both processes,
and restrict the Pi firewall to the laptop. Wildcard/public binds are refused.
The token is read from the environment, not a CLI argument or URL. HTTP bearer
tokens are not encryption: use the SSH tunnel on an untrusted network. Neither
endpoint supplies CORS or write operations. Do not expose the port to the internet.

Ctrl+C/SIGTERM stops the server and joins the acquisition worker. Exceptions retain
last observations with DISCONNECTED and an error. A read error or reference-clock
reset ends the reader session; fix the cause and restart the service. It does not
silently reconnect USB and merge epochs. The genuine Linux lock/native-library/
device lifecycle still requires validation on the Pi.

## Laptop launch

With the tunnel active (or substitute the verified Pi LAN IPv4 address):

```powershell
.\.venv\Scripts\python.exe -B experiments\run_imu_telemetry.py --host 127.0.0.1 --pi-port 8793 --interval 0.05 --timeout 1 --port 8794
```

No new laptop dependencies. `--no-browser` prints the dashboard URL instead of
opening it. `--connect-wait 10` bounds initial metadata discovery. The HTTP worker
does all network I/O; `poll(local_time_s)` only drains a bounded latest-channel
handoff. The existing dashboard displays tables/plots; its metadata panel includes
device timestamps, per-channel Pi receipt ages, packet sequences and transport
errors. Stop server or Ctrl+C closes the client and browser server.

HTTP polling defaults to 20 Hz, **not** a confirmed sensor output rate. Service
acquisition is independent of HTTP requests. HTTP retries recover automatically
within the same Pi session. A restarted Pi service produces SESSION_CHANGED and
requires restarting the laptop client. Unsupported metadata, clock reversal,
mutated duplicate observations and malformed payloads are rejected without
repairing data or refreshing observations. Connection errors do not erase history.

## Payload and timing

`GET /v1/latest` returns `osl-imu-http/1`. `GET /health` returns schema, session,
status, error, server time and stale threshold. HTTP 200 indicates endpoint
availability; inspect state and observation age before claiming data availability.
Both endpoints require the token when configured. Responses use `Cache-Control:
no-store`. Writes are unsupported.

The [complete synthetic JSON example](../build/phase6fb_telemetry/synthetic_example.json)
is generated from an injected reader, with `source: synthetic_mock`. This excerpt
illustrates one channel; it is not a complete wire payload:

```json
{
  "imu_roll": {
    "sequence": 0,
    "time_s": 0.25,
    "value": 0.2,
    "valid": true,
    "device_timestamps": {
      "reference_ns": {"value": 1000000000, "valid": true}
    },
    "reason": ""
  }
}
```

| Field | Meaning |
|---|---|
| `session_id` | New random identity on each service start |
| `metadata` | Exact declared units, frames, source, clock, ordering and mounting note |
| `server_time_s` | Pi monotonic seconds since service construction |
| `samples.<channel>.time_s` | Pi application receipt/dequeue time; **not** device measurement time |
| `sequence` | Service orientation-packet counter, not an asserted hardware counter |
| `value`, `valid`, `reason` | Finite valid/invalid value, or explicit null with invalidity/reason |
| `reference_ns` | Genuine shared reference timestamp: device nanoseconds since startup, if present |
| `gps_unix_ns` | Genuine shared GPS timestamp converted by MSCL to Unix/UTC nanoseconds, if present |
| `external_ns` | SDK external timestamp nanoseconds; external epoch is unverified, never synchronized implicitly |
| `legacy_unix_ns` | Optional legacy packet timestamp as interpreted by MSCL; Gen7 prefers shared fields |
| `stale_after_s` | Configurable Pi host-receipt silence threshold; default 0.5 s, not a safety limit |

Invalid SDK timestamps have `valid: false, value: null`; absent timestamps are
omitted. Nonfinite sensor values become explicit null/invalid with reason
`nonfinite_device_value`, never zero. Finite device-invalid readings retain their
values. Fields not in an incoming packet are not invented: never-seen channels
are UNAVAILABLE; previously seen channels retain their original times and age.
Only timestamps whose APIs were verified above are decoded; other shared timing
fields are outside this phase.

The existing `ReplayRecord.time_s` uses the explicitly named Pi receipt clock to
avoid switching clock domains when a sensor timestamp is absent. All genuine
decoded device timestamps remain in the payload and transport diagnostics. Browser
local receipt time is presentation-consumption time, as in Phase 6F-A; HTTP-worker
receipt elapsed time is separately included in diagnostics. No cross-machine
timestamp subtraction or clock synchronization is claimed.

The laptop estimates Pi receipt age as server age + HTTP round trip + elapsed local
time since response. This conservative age can mark a channel STALE even immediately
after receipt. Repeated HTTP responses do not emit duplicate observations or reset
their local receipt ages. This still cannot establish sensor acquisition freshness
when the SDK/device buffered data before the Pi application received it.

Latest-per-channel caching is deliberately lossy. It is not a recorder and does not
promise every 20 Hz sample. Stable observation sequence is `packet_sequence * 7 +
channel_index`, in the registry's roll/pitch/yaw/q0/q1/q2/q3 order; gaps remain visible.
Clock reset, sensor reconnect and Pi restart require a new session rather than
silently renumbering received history.

## Orientation semantics

Only verified **filter** fields are mapped:

- [Euler 0x82/0x05](https://s3.amazonaws.com/files.microstrain.com/GV7_User_Manual/external_content/dcp/Data/0x82/data/0x05.htm):
  `imu_roll`, `imu_pitch`, `imu_yaw` in radians; 3-2-1 yaw-pitch-roll convention.
- [Quaternion 0x82/0x03](https://s3.amazonaws.com/files.microstrain.com/GV7_User_Manual/external_content/dcp/Data/0x82/data/0x03.htm):
  `imu_q0` through `imu_q3`, dimensionless, preserving SDK vector indices. No
  reordering to another library's convention, normalization or sign flipping.

Both use `imu_filter_ned`, meaning device filter attitude relative to NED.
The configured device-to-vehicle rotation and physical mounting remain unverified.
The application does not transform into a prosthesis frame. SDK validity flags
are retained; they do not establish calibration, filter convergence or heading
accuracy. Complementary-filter fields in descriptor set 0x80 are deliberately not
mixed with filter data. If those are the only existing outputs, verify them before
adding a separately declared mapping.

The live-only `IMU_CHANNELS` registry and acquisition Sample case are additive.
The replay-v1 `CHANNELS`, its CSV/JSON schema and original robot/command contracts
remain unchanged. No orientation observations enter a controller.

## Hardware checks still required

1. Record Pi OS, architecture, glibc, Python, installed pymscl version and import path;
   verify native bindings expose the documented API.
2. Verify USB identity, model/firmware, permissions, exclusive ownership and currently
   enabled descriptors/rates. Approximately 20 Hz is a report, not a measurement.
3. Compare real Euler/quaternion values with the device's trusted output; verify
   quaternion ordering, frame direction, mounting transform and heading observability.
4. Verify validity flags and available source timestamps; unplug/replug, reboot,
   clock-reset and stopped-stream behavior. Measure buffering and latency.
5. Verify Pi locking, signal shutdown, token/firewall/tunnel behavior, and resource
   contention with other workloads. Keep motor acquisition/control separate.
6. Only then characterize calibration and synchronization before IMU-driven
   reference generation, pose reconstruction or measured-versus-predicted analysis.

## Verification

```powershell
.\.venv\Scripts\python.exe -B tests\run_tests.py -k test_imu_
.\.venv\Scripts\python.exe -B tests\run_tests.py
git diff --check
```

See the [Phase 6F-B report](../build/phase6fb_telemetry/REPORT.md) for actual results
and preservation checks. No physical hardware validation is claimed.
