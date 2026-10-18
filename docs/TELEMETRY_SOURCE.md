# Read-only telemetry-source foundation (Phase 6F-A)

This document describes the Phase 6F-A groundwork and its unchanged mock demo.
Phase 6F-B adds a separate [passive IMU reader and HTTP client](IMU_TELEMETRY.md),
tested without physical hardware. Calibration, physical mirroring and predictive
comparison remain pending. The mock still produces hand-authored synthetic
observations without running physics.

## Source boundary

[TelemetrySource](../oslbench/telemetry/source.py) is a small Python Protocol:

```python
@property
def metadata(self) -> dict: ...
@property
def status(self) -> str: ...
def poll(self, local_time_s: float) -> tuple[ReplayRecord, ...]: ...
def close(self) -> None: ...
```

`poll` is nonblocking and returns only newly available observations. Empty output means
no new data; status distinguishes OPEN, EXHAUSTED (finite input drained), DISCONNECTED
and CLOSED. Close is idempotent and terminal. Subsequent polls return empty tuples,
subject to the same finite/nonnegative/nondecreasing local-time requirement. There is
no command, seek, reconnect or reset operation in the source protocol. A new connection
or source-clock epoch should be represented by a new session.

The existing `ReplayRecord` envelope is reused despite its historical name. It retains
sequence, snapshot/group, channel, observation timestamp, clock ID, Sample or None,
source provenance, unit, frame and stage. No duplicate sample/state or timestamp type
was introduced. RobotState remains available when a complete valid group exists, but
this consumer does not fabricate groups or complete poses from partial arrivals.

Metadata uses the existing channel-declaration vocabulary: dataset/session ID, source,
clock `{id, unit: "s"}`, optional identity and channels containing quantity/unit/frame,
stage/source/timestamp/sign. This is source metadata, not a new version of the replay
file schema. The mock uses `synthetic_mock`, which the existing Sample source string
supports; it does not extend the Phase 6D CSV schema's provenance enumeration.

A future adapter must translate its documented measurements into supported quantities
before crossing this boundary. Envelope/sample fields must agree with the channel and
clock declarations. Arrival sequences increase (gaps are preserved, not repaired).
Different channels may have different and globally out-of-order source timestamps;
per-channel time may not reverse within a session. The presentation validates a whole
batch before changing displayed observations. Invalid adapter output raises an error;
it is not repaired or silently dropped.

Existing commands remain distinct contracts, and no command type is constructed here.
Hardware acquisition must remain separate from controller/simulation execution.

## Timing, missing data and lifecycle

`local_time_s` belongs to the caller's monotonic elapsed clock. It controls when the mock
makes records available and when the presentation polls a future source. It is **not**
a synchronization estimate or a conversion of the producer's clock.

`TelemetryPresentation` records local receipt time when each batch is consumed. It
preserves the original source timestamp without subtraction from the local clock.
The UI displays source time, local receipt time and receipt age separately. STALE is
an explicit receipt-age policy (`age > stale_after_s`); it does not establish physical
measurement freshness. An old buffered observation can be recently received. Actual
source age requires clock and buffering evidence from the hardware implementation.

An empty poll never creates observations. A declared channel with no arrival remains
UNAVAILABLE. A record with Sample=None is MISSING. A finite Sample with valid=False
remains INVALID; its value is labelled invalid and is not plotted as a valid point.
NaN/infinity are rejected by the existing Sample contract. Retained values keep their
source timestamps, validity and age through gaps and disconnection. Staleness is an
additional presentation flag, never a mutation of Sample.valid. History is bounded to
the most recent 320 received records per channel; this UI is not a telemetry recorder.

## Deterministic synthetic source and demo

`MockTelemetrySource` declares knee angle/velocity, actuator-output angle and joint belt
torque using established model-coordinate units and signs. It emits six fixed records
between local times 0 and 1.5 seconds, then becomes EXHAUSTED and deliberately
DISCONNECTED at local time 3 seconds. The actuator-output channel never arrives, one
knee observation is explicitly missing, and the torque sample is invalid. Source times
near 100 seconds deliberately differ from receipt times near zero. Its `post_step`
stage labels are synthetic fixture examples of the existing contract, not evidence
that a physics or hardware step occurred. No IMU model or physical channel set is implied.

```powershell
.\.venv\Scripts\python.exe -B experiments\run_mock_telemetry.py
```

The browser opens on loopback port 8792 and starts receiving synthetic data immediately.
This is the existing local dashboard server, not a new hardware network protocol or
background service. Use `--port 0` for a free port, `--no-browser` to open the printed URL
manually, and `--stale-after 0.5` to set the demo's receipt-age threshold. That threshold
is not a hardware timeout or safety limit.

The same browser table and plots used for replay show a prominent SYNTHETIC MOCK label,
source status and quality/age per channel. Replay play/reset/speed buttons are hidden:
live acquisition is not given a fake replay lifecycle. Stop server or Ctrl+C closes the
source and server; closing only the browser tab does not stop the process. By default
DISCONNECTED remains displayed and ages continue increasing. `--exit-on-disconnect`
exits after the scripted disconnection. A new run creates a fresh mock source.

Optional `--outdir build/phase6f_foundation/<new-name>` saves launch identity and a final
presentation snapshot; existing directories are refused. It does not save hardware data.
The existing replay launcher and canonical live-gait launcher remain independent.

## Reuse and validation

Only channel-to-UI formatting was extracted from ReplayPresentation into the shared
`observation_channels` function. Replay continues to use its original virtual clock,
reset, timing and lifecycle. The receipt-aware consumer uses that formatter with local
receipt ages. ReplayWebDashboard's existing server and plotting/table page are reused;
there is no second plotting implementation or MuJoCo view.

```powershell
.\.venv\Scripts\python.exe -B tests\run_tests.py -k telemetry_source
.\.venv\Scripts\python.exe -B tests\run_tests.py
```

[Verification report](../build/phase6f_foundation/REPORT.md): six focused tests and 244
full-suite tests passed. Tests cover deterministic batches, asynchronous timestamps,
missing/invalid/unavailable channels, receipt-age staleness, disconnection/closure,
malformed envelopes, shared HTTP presentation and replay regression. A fresh process
blocks MuJoCo/NumPy/controller imports and all command constructors while mock and
replay presentation operate successfully.

## Pending hardware decisions

Obtain actual available measurements, sensor/encoder/IMU identities, units, frames,
polarity, calibration, clock/reset semantics, sampling/buffering behavior, fault flags
and disconnect/reconnect behavior before writing an adapter. Existing simulation-style
stage names may require a small additive contract extension for real acquisition; do
not mislabel hardware records to fit them. Select transport only from the confirmed
Raspberry Pi implementation. No transport, driver or physical command capability is
chosen in this phase. See the [authoritative roadmap](NEXT_PHASE_PROPOSAL.md).
