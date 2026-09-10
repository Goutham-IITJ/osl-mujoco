# Live demo: OSL V2 CAD bench — AB19 human knee trajectory tracking

Run: `.venv\Scripts\python.exe experiments\run_live_demo.py`

Two windows open by themselves: the MuJoCo viewer with the CAD-derived bench, and a live
dashboard. The demo runs at half speed by default (`--speed 1.0` for real time).

## What the demo demonstrates

The Open-Source Leg V2 knee, as built from our own Onshape CAD export, following the
knee-angle trajectory of a real human subject through one complete gait cycle, looping
continuously. The reference is subject AB19 from the Camargo et al. lower-limb dataset,
trial levelground / ccw / normal / 01_01, one right-leg cycle of 1.2050 s, resampled from
the dataset's 101 samples onto the model's 0.5 ms step grid by a cubic spline that
reproduces every measured sample to 1.4e-14 degrees. The knee is driven by a position
servo at kp = 600 N·m/rad and kv = 17.253 N·m·s/rad, both written into the compiled model
at runtime, and the ankle is held at its keyframe angle as a fixed boundary condition.

The live dashboard is not a re-simulation or an approximation. Every number and every
plotted point comes out of `data.sensordata` on the same MuJoCo step that produced the
frame on screen. The dashboard is a single self-contained HTML page that the demo itself
serves on `127.0.0.1` and opens in the default browser; the page polls the simulation loop
about sixteen times a second for a snapshot of that one shared state and draws it on a
canvas. It loads nothing from the internet, so it works offline, and because it redraws in
the browser's process it cannot slow the physics. There is no second simulation anywhere:
the page has no plant model and no controller in it, only the last published snapshot. What
it shows is the subject and trial, the gait phase, the reference and actual knee angle, the
tracking error, the knee torque and the fraction of torque authority in use, above three
plots against percent of gait cycle: the full reference knee angle with the simulated
trajectory accumulating over it, the tracking error against its zero line, and the bench
actuator torque, each with a marker at the current phase.

Before the window opens, the demo runs the quantitative experiment in-process — the same
`oslbench.simulation.BenchSimulation` that `experiments/run_bench_ab19.py` uses — and
replays the identical span through the one shared `step()` function, and it refuses to
start unless the two agree to within 1e-12 on angle, velocity, torque and command. It also
verifies that the loaded model really is `models/osl_v2_bench.xml` with the authored 0.5 ms
timestep, that the CSV really is the 1.2050 s AB19 cycle, and that the gains are the
validated pair. So the claim "the picture and the numbers come from the same code" is a
check the program performs, not an assertion in a slide.

There is only one implementation of the control law and one implementation of the stepping
loop in this repository, so "the same code" is structural rather than enforced: the live
demo and the benchmark both call `oslbench.controller.PDController` and
`oslbench.simulation.BenchSimulation.step`, and there is nowhere else for either of them
to get a different answer. `docs/CODE_MAP.md` points at both files.

The measured result, which the demo prints again at the end of every cycle, is an RMS
tracking error of 4.36°, a peak error of 9.09°, a peak actuator torque of 16.00 N·m,
0.00 % torque saturation, and a peak knee velocity of 5.05 rad/s. The reference lies
entirely inside the joint's range of motion, so it is never clamped. The peak error is
servo phase lag, not stiffness: at kp = 600 the loop lags about 29 ms, and 29 ms times the
reference's peak velocity of 5.1 rad/s is about 8.6°, which is why the error is largest at
toe-off where the knee moves fastest. Raising kp is the wrong fix; advancing the reference
in time or adding feedforward is the right one.

## What the demo does NOT demonstrate

It is not whole-body walking, and it is not a human gait simulation. `osl_v2_bench.xml` is
a fixed-base bench: the body above the knee is welded to the world. There is no pelvis, no
hip, no ground contact, no foot strike and no body weight. The actuator is therefore
fighting only the shank-and-foot inertia, gravity on that segment, joint damping and
joint friction — roughly 8.9 N·m of gravity torque at full extension, against a 142.2 N·m
authority.

The consequence is that the torque trace is **bench actuator torque and is not a human
knee moment**. During stance a real knee carries body weight through a closed kinetic
chain, so the two quantities are not comparable there; only swing, roughly 60–100 % of the
cycle, is loosely comparable. The 0–60 % shading on the plots marks human stance for
orientation only and does not imply that the bench is in stance.

The demo also does not show the `human_knee_moment` and `human_knee_power` columns of the
AB19 CSV, and this is deliberate. Those two columns are defective: they are correlated at
0.9996 in magnitude, which means they are one signal scaled rather than two independent
measurements; the peak magnitude is about fifteen times a plausible walking value,
consistent with a factor-of-1000 unit error; the peak falls on the very first sample
instead of in early stance; and both decay monotonically like a filter start-up transient.
Until that extraction is redone they are unverified, so they stay out of the presentation.
Nothing in the experiment depends on them — it is driven by the angle column alone, and
the angle column is sound.

Finally, the model represents the available joint torque authority derived from the
motor/gear relationship, but idealizes torque production. It does not model the motor as
an electrical system, nor current, voltage, torque-speed behaviour, efficiency, thermal
state, backlash, transmission compliance or actuator bandwidth. It should not be described
as a full motor/gearbox simulation, and a position servo is a simulation baseline, not the
final hardware controller.

## Controls and options

Space pauses and resumes, R restarts the cycle at 0 %, Esc exits; either window may have
focus, and the demo ends when the MuJoCo window is closed. `--speed` scales how many steps
are taken per rendered frame and never changes the 0.5 ms physics timestep, so slow motion
shows the same trajectory, not different physics; the default is 0.5. `--zoom` (default
0.45) sets how tightly the camera frames the leg, and `--elevation` / `--azimuth` set the
viewing angle. `--record build\demo.mp4` renders the scene offscreen without opening the
viewer, one frame at a time, and streams it straight to ffmpeg — no video is ever buffered
in memory. `--verbose` prints the full verification transcript instead of the six-line
summary.

For the dashboard, `--dashboard` selects the backend: `auto` (the default) tries the
browser page, then a tkinter window, then an in-terminal readout, and prints a line for
every backend it skips and why; `web`, `tk` and `terminal` pin one of them; `none` turns
the second window off. `--port` moves the page off 8787 (0 picks any free port, and an
already-occupied port falls back to a free one automatically), and `--no-browser` starts
the server but does not open a browser, printing the URL instead. `--dash-geometry` only
affects the tkinter backend. Running `.venv\Scripts\python.exe -m oslbench.dashboard`
opens the dashboard alone on synthetic data, with no MuJoCo and no model, which is the
quickest way to confirm the second window works on a given machine.

## Files

`experiments/run_live_demo.py` is the entry point — argument parsing and four calls, no
physics of its own. `oslbench/viewer.py` holds the camera derivation, the viewer loop, the
consistency proof and the `--record` path; `oslbench/dashboard.py` draws the second window
and contains no physics, no plant and no controller. The controller and the stepping loop
are `oslbench/controller.py` and `oslbench/simulation.py`, shared with the quantitative
benchmark.

None of these modifies the model, the controller, the reference CSV, the quantitative
experiment, or any official MyoAssist or KA_L1 source. The demo writes nothing to disk
unless `--record` is given, and it re-asserts the knee's force and control ranges after
every run to prove the model was untouched.
