# Ankle-only hanging fixture

On 2 November 2026 the user confirmed that the ankle housing is clamped above
the output, with the prosthetic foot still attached below, like a normal leg.
`models/osl_v2_ankle_fixture_with_foot.xml` represents that arrangement using
the original CAD orientation (+Z proximal, -Z distal), with the ankle hinge
translated to the world origin. No knee, upper shank or ground contact is present.
The fixture is anchored to world; desk/clamp geometry is not represented.

The fixed ankle housing visuals reuse the existing CAD ankle-module grouping.
The moving output uses the complete original foot segment, including its
CAD-assigned rotating output, adapter and blade parts. Its reconstructed moving
mass is 0.915159 kg. Inertia and relative geometry are preserved from the source
model, rather than guessed from the photo. Exact lab-to-CAD component matching
and measured moving mass remain unverified.

The single ankle hinge retains the CAD +Y axis and original range. Joint
armature 0.008 kg m², damping 0.25 Nm s/rad and frictionloss 0.3 Nm remain
inherited assumptions. A motor actuator accepts joint torque in Nm; it is not
a current-input controller. Encoder polarity/zero and load details should be
checked against the hardware recordings before a quantitative comparison.

## Build and preview

```powershell
.venv\Scripts\python.exe -B tools\build_ankle_fixture.py --load with-foot
.venv\Scripts\python.exe -B experiments\preview_knee_fixture.py --model ankle-with-foot --angle 0 --view
.venv\Scripts\python.exe -B -m unittest discover -s tests -p test_ankle_fixture.py -v
```

The shared preview holds a prescribed pose and does not run a PID or hardware
connection. Image: `build/ankle_fixture_preview/neutral.png`. Two tests passed:
structure/mass/inertia/relative rotation and one second of finite physics.

The builder also supports an explicitly separate `--load output-only` diagnostic
variant. That contains only ankle-concentric rotating clusters and excludes the
distal adapter/blade clusters; it is not the user-confirmed physical setup and
is not used for the ankle comparison. Both variants carry provisional manifests.
The source full-leg and knee-only models are unchanged.

The offline ankle replay is now connected to shared PID/current/drivetrain code
with confirmed gains 2.0/0.0/0.075. See `ANKLE_MULTISTEP_2026-11-02.md` for the
5/10/15-degree comparisons, assumptions and commands. The historical ankle-derived
controller profile remains distinct. This does not select the ankle model for
the knee live hardware application.
