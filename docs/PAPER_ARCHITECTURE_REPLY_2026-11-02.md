# Draft reply and figure notes

Dear Sir,

Yes, we can use the recorded knee and ankle trajectories for further comparison.
The current comparison pipeline has been tested with the isolated knee step
response; the other trajectories will provide additional validation.

I checked the diagram against the implementation and prepared a corrected
layered figure. The main clarifications are:

1. The Pi controller generates the target and controls the physical knee through
   the CAN motor driver at a nominal 200 Hz. The external knee encoder is read
   over I²C. The 200 Hz figure is the control-loop rate, not the CAN bus bitrate.

2. The data backend is a small HTTP server on the hardware host, with a bounded
   in-memory sample buffer. The laptop polls it for JSON batches over TCP/IP,
   normally through Wi-Fi on port 8794. It is not a database. Received samples
   and simulation outputs are saved locally as JSON/CSV records.

3. The simulation receives the same target angle and calculates its own current
   command using its own simulated feedback. The hardware encoder angle is used
   once to initialize the simulated angle and then to compare the two responses.
   It is not continuously imposed on the MuJoCo model.

4. The core signals required for this comparison are the timestamped target
   angle, measured encoder angle and its validity, sample sequence, and angle
   units/sign/zero convention. Our implemented packet also includes the actual
   controller dt, requested and clamped current commands, and controller status.
   Velocity is not required for the current path; commanded current is not a
   measured-current signal.

5. The simulation is aligned to source timestamps rather than one physics step
   per received sample. It uses a 0.5 ms physics timestep and a 200 Hz controller,
   giving approximately ten physics steps between hardware samples. Live input
   is buffered by 100 ms by default to absorb arrival jitter. There is no
   handshake that makes the Pi wait for the simulation, and wall-clock execution
   can lag. Comparison angles are interpolated to the original sample times.

The gains can match, but feedback configuration C uses actuator-side sampled
angle differences for D, whereas the supplied physical controller uses knee-error
differences. We also tested the latter in simulation. This distinction is marked
in the figure rather than claiming identical controller implementations.

The actuator and compliant belt model converts simulated current into knee
torque, which drives the MuJoCo rigid-body model. Model calibration is a separate
offline process; the plotted errors do not automatically update its parameters.
Saved hardware samples can replace the network input for repeatable replay.

Regards,
Goutham

## Figure and scope

- Figure: `docs/assets/research_briefing/04_paper_hardware_simulation` in PNG,
  editable SVG and vector PDF formats.
- Generator: `make_paper_architecture.py` in the same directory.
- The live application currently selects the full-leg plant. The separate
  knee-only adapter has been exercised through offline replay; the figure shows
  those available plant paths without claiming a knee-only live hardware run.
- The tested controller stream and receiver are knee-specific. Recorded ankle
  trajectories need joint/channel, controller, and physical setup verification
  before reusing the comparison path; a dual-joint live stream has not been
  demonstrated.
- Losses, sample validity, sequence gaps and bounded backlog are monitored;
  source-time alignment is not a hard real-time or clock-synchronization claim.
- Evidence: `hardware/knee_telemetry.py`, `hardware/knee_step_test_PID_control_me.py`,
  `oslbench/telemetry/knee.py`, `oslbench/apps/live_knee_comparison.py`,
  `oslbench/sim/live_knee.py`, and the isolated replay experiment scripts.
