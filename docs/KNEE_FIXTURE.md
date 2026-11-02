# Knee-only bench fixture

User-confirmed mounting, 31 October 2026: the knee is upright. The lower
plate, where the remaining leg would attach, is fixed to the desk. The upper
thigh attachment moves. No ankle, foot, or thigh is attached in this variant.

`models/osl_v2_knee_fixture.xml` is a separate one-joint MuJoCo plant. The
full-leg model and historical results remain unchanged. Its fixed housing
visuals reuse the CAD generator's knee-module grouping; clamps, cables and
the desk are omitted. These fixed visuals do not add moving inertia.

The moving CAD `knee_prox` assembly retains its reconstructed 0.449846 kg
mass, centre of mass and inertia, including the output-side parts assigned
to this cluster by the original CAD generator. This is not a measured mass
of the lab assembly. The frame is moved to the knee hinge without moving
any geometry at zero angle. The hinge axis changes from -Y to +Y because
parent and child are reversed; the relative knee-angle convention is retained.
Physical encoder polarity still needs confirmation against observed motion.

Joint armature 0.01 kg m², damping 0.3 N m s/rad and friction loss 0.4 N m
are inherited assumptions, not measurements of this fixture. The actuator
accepts joint torque in N m; it does not accept current directly. Do not
interpret a pose preview as a controller response or a calibrated prediction.

## Reproduce and inspect

From the repository root in PowerShell:

```powershell
.venv\Scripts\python.exe -B tools\build_knee_fixture.py
.venv\Scripts\python.exe -B experiments\preview_knee_fixture.py --view --angle 15
```

The preview holds a prescribed 15-degree pose and allows camera inspection.
Use `--angle 0` for the neutral pose. It sends no hardware commands.

```powershell
.venv\Scripts\python.exe -B experiments\preview_knee_fixture.py --png build\knee_fixture_preview\pose_15deg.png
.venv\Scripts\python.exe -B -m unittest discover -s tests -p test_knee_fixture.py -v
```

The builder reuses `tools/build_mjcf.py` CAD segmentation and writes only the
new fixture XML and its JSON manifest. Tests check the single-joint structure,
neutral geometry and inertia preservation, relative rotation after re-rooting,
and finite state during one second of physics stepping.

## Integration boundary

The live comparison and gait commands still default to the original full-leg
plant. Offline replay now selects the fixture explicitly through a one-joint
adapter in `oslbench/sim/knee_fixture.py`. Both plants use the same controller
configuration, current conversion, `DrivetrainLayer`, and `LiveKnee` source-time
stepping. The adapter applies the belt's mean joint torque to MuJoCo; it adds
no ankle and no second torque source. Full-leg precondition checks remain intact.

```powershell
.venv\Scripts\python.exe -B experiments\replay_knee_fixture.py
```

This defaults to the saved physical recording in
`build/hardware_step_20261030_134549_560`, requires no hardware or network, and
creates a new timestamped `build/knee_fixture_replay_*` folder. `--recording`
selects a recording directory; `--outdir` selects a new output directory (an
existing directory is refused). It writes paired comparison CSVs, full-rate
simulation CSVs, metadata, summary JSON and a comparison PNG for both plants.
Only the first encoder angle initializes the simulation; subsequent measured
angles are used exclusively for comparison. Target commands retain their
recorded source times and the original zero-order hold behavior.

First replay: all 199 pairs retained; full-leg simulated angles exactly reproduce
the historical recording. Full-leg vs fixture RMS simulation/hardware difference
is 1.1953 vs 0.5117 degrees; peak difference 5.2120 vs 2.3291 degrees. The fixture
removes the overshoot but does not improve every metric: 10–90% rise time is
72.70 ms (full-leg 68.16 ms; hardware 50.97 ms), and tail RMS difference after
0.5 s is 0.2520 degrees (full-leg 0.1710). These are one-step results, not gait
validation. No gain, update-rate or torque-constant change was made.

Before numerical validation, verify the moving CAD components against the lab
assembly, and compare the hardware's knee-error derivative with simulation
configuration C in separate controlled runs. Matching mounting alone does not
make these controllers identical. Keep the 0.5701 N m/A interpretation provisional.
