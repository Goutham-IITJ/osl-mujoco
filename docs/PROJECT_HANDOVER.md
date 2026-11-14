# Project handover — OSL V2 MuJoCo

## Current-loop dynamics gait sensitivity, 14 November 2026

Reused the existing exact first-order current loop in closed-loop offline
replays of all six gait trials, at 0, 0.25, 0.725, 1.5, 3, 5 and 8 ms.
Held the 2810 kg/m³ model, ratio 4.61, output gain 0.7044, controller gains
and actuator-side derivative fixed. The zero-lag baseline reproduced the
saved angle RMS to 1e-10 degrees. Verified the current update on each
physics sample via generated torque divided by the output coefficient.
All 42 runs completed without clipping or belt-fit exceedance. CubeMars
line-to-line L/R gives 0.725 ms as an electrical reference case, not a
measured driver bandwidth. Fast-walk knee command jump RMS fell from
12.63 to 12.45 A at 0.725 ms. At 8 ms the command still jumped by 11.90 A,
while modeled current jumps fell to 4.18 A. Hardware command reference
is 1.69 A on the same grid. No measured current data. Scripts, metrics,
traces and visually checked PNGs: `build/ak80_current_dynamics_20261114/`.
Next: characterize the physical inner loop before adopting a lag or test
command-side derivative filtering separately if smoother commands are required.

## Aluminium density gait replay, 14 November 2026

Built a separate 5.068973 kg bench MJCF from the recalculated segment mass,
COM and inertia for 2810 kg/m³. Replayed all six archived gait trials with
the 2700 and 2810 kg/m³ models at belt ratio 4.61 and output coefficient
0.7044 Nm/A, using identical gains and Configuration C. The 2700 model
reproduced prior RMSE within 1e-10 degrees. Knee hardware RMSE decreased
by 0.0023–0.0211 degrees across trials, and ankle changes were under
0.0003 degrees. No simulated clipping or belt-fit exceedance. Model,
paired angles, RMSE and reproducible script: `build/aluminium_density_gait_20261114/`.
Next: confirm alloy and physical component inventory before adopting the density.

## Aluminium density sensitivity, 14 November 2026

Recomputed segment mass, centre of mass and inertia using the CAD builder
with all 245 aluminium-assigned instances at 2810 instead of 2700 kg/m³.
The model total rose from 4.955763 to 5.068973 kg (+0.113210 kg, +2.28%).
Inventory volume sums independently reproduced each mass increment. Evidence
is in `build/aluminium_density_2810_20261114/`. The density change was local
to the calculation. No dynamic replay or physical weighing was performed.
Compare against the measured mass for the same component inventory before
adopting a density change as a hardware correction.

## Belt ratio gait sensitivity, 14 November 2026

Replayed all six recorded gait conditions at 50/11 and the user-confirmed
4.61 hardware belt ratio, holding the effective torque coefficient fixed
within each comparison (0.5701 and 0.7044 Nm/A). Applied each ratio to
both the drivetrain and actuator-side derivative, with zero initial belt
deflection. Both 50/11 baselines reproduced saved angle RMS within 1e-10
degrees. Maximum ratio-only changes were 0.012 degrees in hardware RMS,
0.044 A in current RMS and 0.014 A in RMS current jumps. No clipping or
belt-fit exceedances. Evidence: `build/belt_ratio_gait_20261114/`.
Use 4.61 for the hardware-matched configuration. The archived results
document 50/11, and the production defaults have not been edited.

## Current command comparison figures, 14 November 2026

Created step and gait current overlays in `build/ak80_current_comparison_20261114/`
from archived hardware commands and reproduced 0.5701/0.7044 simulations.
Step plots use joint-error feedback and show the first 250 ms. Gait plots
use the six assembled Configuration C recordings. Checked archived angle
metrics during replay, verified gait current sign against recorded P terms,
and visually reviewed both figures. Current traces are commands, and the
physical motor current remains unmeasured in these recordings.

## AK80-9 gearbox coefficient step replay, 14 November 2026

Replayed recorded 5/10/15-degree knee and ankle steps with 0.5701 versus
0.7044 Nm/A in the existing isolated fixtures. Retained gains, timing,
belt law and effective friction. Verified archived input/model hashes and
baseline metrics for both derivative signals. With the joint-error signal
used in the paper, RMS improved for all three knee steps and worsened for
all three ankle steps. The 0.7044 simulation still rose 20–29 ms slower
than hardware. Configuration C gave small ankle RMS improvements instead.
All completed without simulation clipping or belt-fit exceedance.
Evidence: `build/ak80_step_07044_20261114/` includes both feedback modes,
paired traces, summary, protocol, replay script and a reviewed plot.
Next: confirm physical output coefficient before updating reported results.

## AK80-9 manual revision and 0.7044 gait sensitivity, 14 November 2026

See `docs/AK80_9_MANUAL_REVISION_2026-11-14.md`. The supplied CubeMars V3
Drive Software Manual p. 31 lists 0.7044 Nm/A at the output and p. 23 links
it to servo current mode. An older manufacturer manual copy lists 0.5701.
Replayed all six archived gait conditions at 0.7044 with effective friction
and other model settings fixed. Baseline reproduced within 1e-10 degrees.
Knee RMS fell on all six trials. Ankle RMS fell on two and rose on four.
All completed with zero clipping and belt-fit exceedance. Evidence in
`build/ak80_gait_07044_20261114/`. Next: confirm as-tested firmware and
measure torque versus actual Iq before using either figure as calibration.

## AK80-9 replay comparison figures, 13 November 2026

Shareable overlay, numerical difference plot and RMS table are in
`build/ak80_gait_sensitivity_share_20261113/`. Recomputed RMS from 12,416
timestamp-matched angles across all six trials. Maximum difference between
the simulated angles was 5.68e-14 degrees with the output coefficient and
effective friction fixed. Visually checked both PNGs. Confirmation of the
physical 0.5701 Nm/A output coefficient remains open.

## AK80-9 gait sensitivity, 13 November 2026

Replayed all six recorded gait CSVs offline in `build/ak80_gait_sensitivity_20261113_160126/`.
The baseline reproduced archived RMS to 1e-10 degrees. Setting rotor k_t to
0.095 while retaining effective output k_t of 0.5701 and the same friction
per commanded amp reproduced all angles to 6e-14 degrees. Using nominal
0.855 Nm/A output with 0.095 rotor and the same friction per commanded amp
reduced knee hardware RMS in five of six runs and increased ankle RMS in all
six. Largest changes were knee ramp descent −0.453 degrees and ankle ramp
ascent +0.386 degrees. No clipping or belt fit exceedance. Preserved gains,
mass, inertia, timing and the archived Configuration C. See `summary.csv`,
`protocol.json`, `replay.py` and paired traces. Next: confirm whether 0.5701
was measured and how current is defined before treating either value as a
hardware calibration. Step tests and inertial tuning remain separate.

## Gait recording assets, 13 November 2026

Ran the current-controlled AB19 knee gait and the assembled slow-walk replay
in MuJoCo 3.12.0 with Configuration C, 200 Hz control and 0.5 ms physics.
Evidence: `build/gait_recordings_20261113_144526/`, including
`Gait_raw_results.zip` (13 CSV, PNG and metadata entries; CRC verified).
AB19 target RMS was 3.394 degrees; assembled simulation versus recorded
hardware RMS was 0.762 degrees knee and 0.237 degrees ankle (2,173 pairs
each). Assembled ankle range used the existing experimental ±31-degree
override. No new physical trial or controller/model changes.

The side-by-side clips synchronize the repository dashboard plots and
offscreen MuJoCo rendering against the new result files. Playback is slowed
for viewing; use the raw CSV timestamps for analysis. The AB19 view depicts
the ankle at its servo hold angle. Both 1280×720 MP4s loaded and seeked in
Chrome; reviewed the paired screenshots. The full-view MP4s and PNGs were
copied to the user's Downloads/Recordings folder. Next: share the full-view
clips and raw results with the professor.

## AK80-9 motor parameter audit, 13 November 2026

See `docs/AK80_9_PARAMETER_AUDIT_2026-11-13.md`. Found the local CubeMars
V3.0.1 manual and checked the AK80-9 V3 KV100 catalogue specifications against
the hardware script, simulation code and archived runs. Verified the slow-walk
archive's effective generated output torque coefficient of 0.5701 Nm/A over
43,446 nonzero-current rows. Nominal manufacturer candidate is 0.095 Nm/A rotor
and 0.855 Nm/A ideal gearbox output, subject to current convention. The existing
adapter's current scaling also changes gear friction. Next: isolate the torque
coefficient in offline replay, then compare the current-rescaling variant.
The audit used archived evidence. Dynamic sensitivity is pending.

## Robotic-device-only MyoAssist 1.0 comparison, 13 November 2026

Created `docs/OSL_V2_DEVICE_COMPARISON_2026-11-13.md` after the user narrowed
the scope to the prosthesis and supplied Robbins et al.'s MyoAssist 1.0 preprint.
Section III-B confirms the simplified direct-torque device representation; the
local KA_L1 MJCF and YAML establish exact device values. Verified axis spacing
and compared rigid-body knee and ankle inertias at zero joint angles using
compiled models and body COM Jacobians. No dynamic test or model change. Use
this device-only report for the professor; matched physical component inventory
and head-to-head hardware response remain open.

## CAD versus MyoAssist comparison, 13 November 2026

Created `docs/MYOASSIST_COMPARISON_2026-11-13.md` from the user-supplied
MyoAssist 0.1 ICORR 2025 PDF, local Myo repositories, current runtime and archives.
Distinguishes the paper/legacy 54-muscle model with four socket coordinates from
modern rigid-socket KA_L1 composed with 65 or 15 remaining muscles. Clarifies that
our active plant is a device-only CAD model with explicit compliant drivetrain,
not a biological musculoskeletal model or the old servo-only experiment.
Compiled bench, walk scaffold, saved modern scene and legacy scene using existing
MuJoCo 3.12; fresh myolegs22 composition used existing MyoAssist MuJoCo 3.11.
No new rollout, parameter change or hardware experiment. Evidence and reproducible
inspection script: `build/myoassist_comparison_20261113/`. PDF text extracted and
relevant page rendered/visually checked. Historical reports preserved.

Key qualifications: 260 source CAD meshes are not 260 moving bodies (compiled
bench: 80 mesh assets, 217 geoms, two joints); socket-normalized device mass gap is
+0.428280 kg for our model, but component inventories are not fully matched.
49.4/58.4 are Myo torque gains, not verified V2 mechanical reductions. Existing
Myo traces demonstrate exploration, not stable prosthetic walking. The explicit
drivetrain adds shaft states outside MuJoCo nq; authored joint placeholders remain.
Next: professor review; resolve physical foot/battery inventory, inertias, torque
constant and feedback-law differences before any matched human/device experiment.
No commits, pushes or changes to either model source.


## Notebook torque-PD drivetrain timestep slide, 12 November 2026

Created `build/final_evaluation_20261112_torque_pd_step/output/OSL_V2_Torque_PD_Drivetrain_Timestep.pptx`.
The standalone slide documents the earlier torque-PD path and paper belt ratio
4.61 using the notebook notation. The belt torque is evaluated from the pre-step
state and the damping term uses the new shaft speed, per the implementation.
The current-PID replay path and its CAD belt ratio remain separately documented.
Rendered and reviewed the slide. Package and first-party import checks passed.

## Standalone gait-cycle RMS comparison, 12 November 2026

Created `build/final_evaluation_20261112_gait_rms/output/OSL_V2_Knee_Gait_Cycle_RMS_Comparison.pptx`.
Computed all three RMS differences from the same 435 paired knee samples
in the selected slow-walk cycle. Hardware versus target is 1.273299 degrees,
MuJoCo versus target is 1.636524 degrees, and hardware versus MuJoCo is
0.685574 degrees. The target is the logged AB01-derived command. Rendered
and reviewed the slide. Package and first-party import passed.

## Standalone knee gait cycle with human target, 12 November 2026

Created `build/final_evaluation_20261112_gait_three_traces/output/OSL_V2_Human_Hardware_and_MuJoCo_Gait_Cycle.pptx`.
Added the logged AB01-derived target_deg trace to the same selected slow-walk
cycle. The 0.69 degree RMS still compares the recorded hardware and MuJoCo
angles. Rendered and reviewed the slide. Package and first-party import passed.
Canva import remains untested.

## Standalone hardware and simulated knee cycle, 12 November 2026

Created `build/final_evaluation_20261112_hardware_gait_cycle/output/OSL_V2_Hardware_and_Simulated_Knee_Cycle.pptx`.
It shows the first complete interior slow-walk knee cycle from
`build/assembled_normal_20261104_C/comparison.csv`, bounded by successive
reference extension minima at source times 2.107644 s and 4.275673 s.
The 435 paired angles give 0.685574 degrees RMS simulation-to-hardware
difference without relative time shifting. Rendered and reviewed the slide.
Package and first-party import passed. Canva import remains untested.

## Monochrome controller development slide, 12 November 2026

Created `build/final_evaluation_20261112_controller_mono/output/OSL_V2_Controller_Development_Monochrome.pptx`
as a standalone replacement for the controller journey page. The revised layout
uses editable equation text and matches the PID diagram's E[k] notation. Code
inspection confirmed the bounded integral is active in the knee profile at
Ki = 0.15 and inactive at Ki = 0. Rendered and reviewed the slide. Package and
first-party import checks passed. Canva import remains untested.

## Standalone AB19 gait tracking result, 12 November 2026

Created `build/final_evaluation_20261112_ab19/output/OSL_V2_AB19_Gait_Tracking.pptx`
to introduce the human reference and the early single-knee simulation result
on one slide. A static line plot uses all 2,410 rows per archived case from
`build/knee_controller_selection/{old_matched,knee}/gait_trace.csv`. The two
cases use the same reference, fixed-base model and 200 Hz Configuration C.
Recomputed simulation-versus-target RMS as 6.846268 and 3.394414 degrees.
The plot labels the cases initial and revised PID settings. The stance/swing
descriptions are general context. No toe-off marker is inferred. The single
slide was rendered and checked, and package and first-party import passed.
Canva import remains untested.

## Standalone PID signal-label revision, 12 November 2026

Created `build/final_evaluation_20261112_pid_labels/output/OSL_V2_Knee_Angle_PID.pptx`
to replace the first current-pipeline slide. It names the angle and current
signals with units and defines E as the bounded accumulated angle error.
Rendered and reviewed the slide. Package, layout and first-party import
checks passed. Canva import remains untested.

## Standalone drivetrain timestep revision, 12 November 2026

Created `build/final_evaluation_20261112_drivetrain_step/output/OSL_V2_Drivetrain_Timestep.pptx`
as a replacement for the second current-pipeline slide. It uses physics indices
n and n + 1 and adds the continuous shaft torque balance, reflected belt load
and symbol definitions. Speaker notes show the implicit damping discretization.
Rendered and reviewed the one slide. Package, layout and first-party import
checks passed. Canva import remains untested.

## Standalone current-control and drivetrain slides, 12 November 2026

Created `build/final_evaluation_20261112_current_pipeline/output/OSL_V2_Current_Control_and_Drivetrain_Slides.pptx`
for insertion after the controller development slide and before the AB19 results.
The two editable monochrome slides show the sampled position PID with separate
Configuration C derivative feedback, and one compliant drivetrain timestep.
Equations were checked against the current path, including old-state belt
evaluation, implicit viscous damping and the actuator-output current conversion.
The notebook's earlier 4.61 belt ratio is discussed in notes. The current
configuration uses 50/11. Both slides were rendered and reviewed, and package
integrity and import passed. Canva import remains untested.

## Standalone controller development slide, 12 November 2026

Created `build/final_evaluation_20261112_controller/output/OSL_V2_Controller_Development.pptx`
for insertion after CAD reconstruction and before the current-controller details
and AB19 tracking results. The single slide includes the original servo gains,
the torque-PD request and rotor-current conversion, the feedback-location
instability diagnosis and correction, and the current PID transition. Speaker
notes contain the gain derivation and source paths. Rendered and reviewed the
slide, and verified one-slide PPTX package integrity and import. Canva import
remains untested.

## Final evaluation CAD slide update, 12 November 2026

Created a copy of Visual Revision 3 with slide 4 rebuilt around a same-view
visual-origin diagnostic and a distinct URDF inertial audit. The four stages
show mesh placement, rigid body recovery, mass and inertia reconstruction, and
explicit MJCF emission. Appended slide 30 with the part inertia equations, the
parallel-axis aggregation and the nominal density assignments by part family.
The density assignments remain model assumptions.

Deck: `build/final_evaluation_20261112_cad/output/OSL_V2_MuJoCo_Sim_to_Real_Visual_Revision_3_CAD_Update.pptx`.
Previews: `rendered/slide-04.png` and `rendered/slide-30.png` in that build directory.
Reviewed both rendered slides. The finalizer passed package, layout, chart and
import checks. Verified 30 slides, 32 embedded chart workbooks, matching text on
the other existing slides, and identical embedded MP4 bytes on slide 13.
PowerPoint slideshow playback was not tested. SHA-256:
`63bd34f110c4e9129508b20f0f6177df81800fe0aac7ae79369e917d4a5f5b22`.

## Final evaluation presentation visual revision 3, 10 November 2026

Completed the requested media/design pass in a new directory, preserving v1/v2:
`build/final_evaluation_20261110_v3/output/OSL_V2_MuJoCo_Sim_to_Real_Visual_Revision_3.pptx`.
There are 17 main slides (720 seconds planned speech) and 12 backup slides, with
29 notes and 32 editable native charts / embedded XLSX files. The deck adds an
embedded H.264 MP4 on slide 13 and an interface-diagnostics backup on slide 28.
The cover now has a dark navy background, and editable process diagrams use
purple/control, blue/actuator, orange/belt and teal/plant stages. Slide 4 was
redesigned around the failed export, reconstruction stages and corrected MuJoCo
model. Slide 7 groups each equation inside its process stage, connects the held
current to the physics loop and distinguishes the 5 ms and 0.5 ms updates.

No saved video clips were found in build. The new clip visualises the first six
seconds of the archived original-gain slow-walk simulation in
`build/assembled_normal_20261104_C/simulation.csv`. Interpolated saved simulated
joint angles were assigned to the unchanged MJCF and rendered with `mj_forward`
only; no new physics stepping or hardware action. There are 72 frames at 12 fps;
the browser-encoded H.264 MP4 lasts approximately 6.7 seconds, so it must not be
used for response-timing measurements. Source/model hashes and camera/method
are recorded in `media/provenance.json`; the model matches the archived metadata
hash. A 30 fps render was stopped in this new revision directory to reduce media
processing time; its partial frames are private authoring intermediates.

Captured the actual `oslbench/presentation/assembled_tracking.html` layout and
plot code displaying archived comparison/simulation CSV values, with no network
or hardware connection. Screens show angles, residuals, current and belt states.
The preserved historical `build/dashboard_video_review/frame_01.png` supplements
the new captures. These are interface evidence, not new physical tests. An offline
copy of the real assembled interface has working view selectors, an explicit
archive status and a disabled Stop control. UI/data hashes are in
`media/source_hashes.json`. Original source files remain unchanged.

User media package:
`build/final_evaluation_20261110_v3/output/OSL_V2_Presentation_Media.zip`.
It includes the standalone MP4, six MuJoCo stills, model/fixture/Myo views, interface
captures, provenance, a README and `Open_Archived_Dashboard.html`. For Canva,
upload the MP4 manually if PPTX import does not preserve embedded media.
Canva import and native PowerPoint slideshow playback have not been tested.
Browser MP4 decode confirmed a 408 x 380 H.264 clip, and embedded video bytes
were checked identical to the standalone MP4. Media relationships and on-click
timing are present in the PPTX; the poster is the image fallback.

Reviewed all 29 slide PNGs and refreshed/rechecked affected curves and diagrams.
The packaged finalizer passed integrity, geometry/font, native charts/workbooks
and exact-file first-party import. ZIP CRC and inventory checks passed. Images
were resized only in the new authoring copies; archived originals are intact.
Chart curves now use at most 160 selected original points per series to keep the
expanded media deck within the importer's construction budget. All full-record
reported metrics and comparison definitions remain unchanged. Validation records
are `validation.json` and `chart_snapshot_reviewed_receipt.json`.
PPTX SHA-256:
`b012b667fef698d7d482b7bd682f43bc5ecdeb01d505464963fa5ffe89673d1f`.
Next: user review of slides 4, 7, 13 and 28, Canva/media playback check, and a
12-minute rehearsal. Existing paper/model uncertainties remain as recorded below.
No source controller/model edits, commits, pushes or hardware commands.

## Final evaluation presentation revision 2, 10 November 2026

Completed the user's technical revision in a new directory, preserving version 1:
`build/final_evaluation_20261110_v2/output/OSL_V2_MuJoCo_Sim_to_Real_Revision_2.pptx`.
The title is now CAD-Derived OSL V2 Simulation in MuJoCo / Sim-to-Real Analysis.
There are 16 main slides (690 seconds planned speech) and 11 backup slides.
All 27 slides have speaker notes and evidence paths; 32 native charts have
embedded editable XLSX data. Canva import fidelity remains untested.

Replaced the main position-servo benchmark with the archived current-controlled
AB19 comparison (matched C/200 Hz/0.5701 Nm/A cases, target RMS 6.846 to 3.394 deg).
The earlier servo and torque formulations are development history. Expanded CAD
reconstruction with original visual-origin failure/correction images, the custom
pipeline, and inertia equations. Added a dedicated current/planetary/belt/physics
update diagram and a parameter table. Expanded MyoAssist using the saved
80-muscle lineage composition (65 retained muscles and two OSL motors) and local
reflex-control documentation; no stable prosthetic walking result is claimed.
Used the original `paper/Software_architecture.pdf` figure. Added six-time-constant
current-loop response curves and actual original/tuned ankle curves plus gain
ranges. Original hardware gains remain the comparison baseline. Target RMS and
simulation-to-hardware RMS are visibly distinguished; C and hardware joint-error
D are not described as identical controllers. Historical 1 kHz/0.855 Nm/A studies
are labelled separately from the later hardware-comparison settings.

Evidence includes 18 additional archived CSV sources, with hashes in
`additional_source_hashes.json`, plus the earlier evidence and source images.
No new dynamic experiment or hardware action was performed and no gains or
mechanical defaults changed. Full-record metrics were retained. Plot curves use
up to 200 selected original points per series (largest-triangle selection), rounded
to ten significant digits for portability; this does not recalculate the reported
metrics. The first-party importer initially hit its XML construction limit on the
larger chart package; point selection resolved this without rasterizing charts.

All slide layouts were reviewed as PNGs, with affected curves rechecked after
selection. The packaged finalizer passed package integrity, layout/font, native
chart title/data/workbook checks and exact-file first-party import. ZIP CRC and
inventory checks confirmed 27 slides, 27 notes, 32 charts and 32 embedded XLSX
files. Receipts: `validation.json` and `chart_snapshot_final_receipt.json` in the
new directory. Final SHA-256:
`434a00ba8b3ab408112ac6b01fcd264b4a7ce014bcb3a83c884829b4afb8145e`.
Local exclusive-copy publication retains the no-overwrite and hash checks because
the Windows sandbox rejects hard links; managed runtime files remain untouched.

Remaining: user content review, Canva import check and timed rehearsal. Paper
component correspondence (SACH/Variflex, batteries), belt-ratio discrepancy and
unverified physical parameters remain unresolved, as described in the existing
paper notes. No manuscript edits, commits or pushes were performed.

## Final evaluation presentation, 10 November 2026

Created the editable PPTX for the 12 November evaluation in
`build/final_evaluation_20261110_v1/output/OSL_V2_MuJoCo_Final_Evaluation.pptx`.
It has 13 main slides with an 11-minute speaking budget and 11 supplementary
slides, leaving time for questions in the 15-minute slot. All 24 slides contain
speaker notes and evidence paths. Thirty native charts have embedded XLSX data;
text, diagrams, tables and those charts remain editable in the PPTX. Original
gain-grid and clipping figures remain images. Canva import fidelity is untested.

Used the user-supplied `Downloads/ICORR_2027 (4).pdf` for manuscript context and
the two physical-device photographs. The main presentation concerns Goutham's
simulation work; hardware construction is context. MyoAssist occupies one brief
exploration slide, without claiming stable prosthetic walking. Rendered existing
CAD and saved MyoAssist MJCFs at prescribed poses using `mj_forward` only; no
new dynamic experiment, hardware command, gain change or physical calibration.

Authoring source, selected chart data, source hashes, slide manifest and PNG
previews are in the new build directory. Full-record step RMS values were checked
against archived paired CSVs. Chart samples are selected and rounded to ten
significant digits for Office portability; displayed metrics retain the original
full-record basis. The packaged validators passed PPTX integrity, layout/font,
native chart title/data/workbook checks, and first-party import. All slide layouts
were reviewed as rendered PNGs. The final file contains 24 notes and 30 embedded
workbooks. Validation records are `validation.json` and
`chart_snapshot_receipt.json`; SHA-256 is
`fd2bab2320977a047be698c536e645240ef5f2bbdec2fdedb005d1b801448ecb`.
Windows sandbox hard-link publication was replaced locally with exclusive-copy
publication while retaining the packaged validation and final hash checks.
Managed dependency files were not modified.

Keep hardware-difference RMS separate from target-tracking RMS. Isolated primary
steps use joint-error D; assembled results use C, unlike hardware's derivative.
The new paper describes a SACH foot/off-board batteries and a 4.61 belt ratio;
saved replays use the CAD-derived model/component accounting and 50/11 ratio.
These correspondences remain unresolved and are qualified in the notes/backup
slides. The paper's remaining template text and broad fidelity conclusions were
not adopted as verified claims. Paper and historical outputs were preserved.
Next: review the deck with the user, check any Canva import, rehearse to 11 minutes,
and continue manuscript revisions separately before the 17 November submission.
No commit or push performed.

## Paper gait and step replay cross-check, 7 November 2026

Replayed all six assembled trajectories at 200 Hz control and 0.5 ms physics
with the original hardware gains and the top-ranked simulation gains. All 24
joint/configuration/trajectory metric rows reproduced the saved ranking's
RMS errors and sample counts within 1e-10, with zero current clipping or belt-fit
exceedance. New evidence in `build/paper_gait_replay_20261107/` includes paired
CSV traces, an RMS summary, and two visually checked 12-panel PNG/SVG figures.
Reproduce with `experiments/replay_ranked_assembled_gains.py`.

Reran the three steps per isolated fixture using the existing multi-step
experiment. The paper's joint-error D rise times were reproduced within 0.1 ms
of the displayed values. Simulation's 10–90% rise was 19.74–28.22 ms longer
than hardware. This transient discrepancy coexists with full-record step RMS
differences of 0.144–0.475 degrees. Step plots and metrics are in
`build/paper_replay_20261107_knee_steps/` and
`build/paper_replay_20261107_ankle_steps/`. Report step rise separately from
continuous gait RMS, which uses configuration C.

## Paper mass and gain study, 7 November 2026

Mass arithmetic cross-check passed on a translated cuboid and all 260 CAD
meshes. At the existing assigned densities and segmentation, the independent
surface-volume sum matched the 4.955763 kg compiled model within 1.61e-12 kg
per segment. Physical inventory and weighing are still needed. Evidence in
`build/assembled_mass_check_20261107/`.

An experiment-only gain override in the assembled runtime enabled a 54-candidate
configuration C sweep. Two development trajectories selected knee 2.25/.30/.09
and ankle 3/0/.1125, scoring .6637 relative to the hardware gains. On four
evaluation trajectories the candidate tracked the target more closely but
exceeded the 60 A simulated knee clip on stairs ascent, at both .5 and .25 ms
physics steps. The hardware gains passed all four at both steps. Baseline
hardware-difference metrics matched the saved assembled results. Four runtime
tests passed. The sweep, figures and raw evidence are in
`build/assembled_gain_sweep_20261107/`.

The complete six-panel tuning score and peak requested current grids are in
`build/assembled_gain_sweep_figures_20261107/`. The separate stairs ascent
replay matched the saved request peaks and clipping counts. Its knee trace
shows 70.225 A requested and 60.000 A applied for one 5 ms interval with
the lowest-error tuning gains. The PNG and SVG figures were generated from
the saved sweep and replay; the PNGs were visually checked. Only the baseline
and lowest-error tuning candidate were run on the four evaluation trajectories.
That original split study was followed by a separate retrospective ranking of
all 54 sets across all six recordings. At 0.5 ms, 44 passed the no-clipping and
belt-fit checks. Lowest normalized target RMSE (0.835701) used the baseline
knee 1.50/0.15/0.060 and ankle 3.00/0/0.1125. It passed all six again at
0.25 ms physics, with a maximum request of 51.52 A. Ankle agreement with
the recorded hardware worsened on five of six trajectories, so keep the
hardware gains for sim-to-real comparisons. See
`build/assembled_gain_ranking_20261107/Ranking.json`.

Read `PAPER_MASS_AND_GAIN_STUDY_2026-11-07.md` for methods, limitations and text
for Sections III-B, III-E and IV. The local Section III Word draft was read for
structure. The paper draft and hardware controller gains are preserved.

## Paper drafting milestone, 7 November 2026

The Overleaf upload copy is `paper/Software_architecture.pdf`. It matches the
hardware figure's visual style with white boxes, gray outlines and a blue
telemetry accent. The `Pi telemetry server` block reads `HTTP · JSON batches`,
with endpoint, port and authentication details in the manuscript prose. The
CSV branch is labeled `Per-joint records`. Both sources lead to the `Laptop
input adapter`, which sends the target to the simulated controller and the
measured angle and sample time to the comparison. Checked these paths against
the live receiver, CSV reader and assembled runtime. The vector figure was
visually reviewed after rendering.

The user selected a three-column block layout with short signal and protocol
labels. Use `paper/Fig_Software_Architecture_Three_Columns.pdf` for III-A.
It gives the physical bench, data interface and MuJoCo their own columns.
The vector PDF was rendered and visually checked. The earlier compact
two-column version is `paper/Fig_Software_Architecture_Compact.pdf`.

Revised standalone figure `paper/Fig_Software_Architecture_Interfaces.pdf` adds
I²C/CAN payloads, CSV field and clock definitions, the assembled live HTTP setup
and JSON polling, and separate target, initialization and measurement paths.
Checked labels against the hardware scripts and replay adapters, then rendered
and visually inspected the vector PDF. Use the archived CSV branch to explain
the reported results. The Word draft still contains the earlier figure.

Drafted Section III-A in the root `paper/` folder with one Word page and a
standalone vector diagram. Files are `Section_III_Software_and_Sim_to_Real_Framework.docx`
and `Fig_Software_Architecture.pdf`. Used the 7 November attached draft
`Downloads/ICORR_2027 (1).pdf` for section placement and cross-references.
Checked the archived step and assembled workflows, including 5 ms simulated
control updates, 0.5 ms physics steps, initialization from encoder angles,
timestamp-aligned comparisons, and the different derivative feedback used in
the two result sets. Verified the final one-page layout via Microsoft Word PDF
export and visual inspection. Next paper page is MuJoCo model construction.
Hardware gain reuse and physical component mass checks remain separate topics.

## Paper handoff, 5 November 2026

Prepared `build/OSL_MuJoCo_Paper_Handoff_2026-11-05.zip` and its matching folder
for Preetam. Start at `00_START_HERE.md`. The archive contains the six assembled
tracking comparisons, both isolated step series, the original physical knee
capture and supporting diagnostics, methods and vector architecture figure, and
a runnable replay snapshot with the original input archives. The incomplete
joint-error D run is isolated from the completed C results.

Verified 310 source copies by SHA-256 and the 315-file ZIP by CRC. From the
bundled snapshot, reproduced slow walking, both isolated step series, and the
full-leg versus knee-fixture replay. Their 16 reported RMS values matched the
archived values to 1e-10 degrees. Evidence is in
`build/paper_handoff_check_20261105/` and the package's `05_Reproduction/VERIFICATION.md`.
The bundle contains the meshes needed by its three MJCF files. Full CAD
regeneration requires the complete original export. Original runs and the
paper's existing model assumptions are preserved.

## Latest integration: 4 November 2026 — assembled replay + live dashboard

New `oslbench.apps.assembled_tracking` connects the dual-joint hardware schema to
one coupled full-leg MuJoCo model with two independent shared PID/drivetrain states.
New joint/category-selectable dashboard on 8798 includes both comparisons and
simulation torques/current/belt/PID/motion, pause, stable DOM and Stop/save.
Guide `ASSEMBLED_TRACKING.md`; results `ASSEMBLED_TRACKING_RESULTS_2026-11-04.md`.
All six archived conditions completed with C and explicit runtime `--ankle-range
hardware` (±31 deg; CAD default not edited, zero/travel unverified). Results in
`build/assembled_matrix_20261104_C/`, including aggregate CSV/manifest. RMS knee/
ankle: slow walk .762/.237, fast walk 2.272/.662, ramp up 1.351/.505, ramp down
2.381/.691, stairs down 1.277/.675, stairs up 1.090/.330 deg. No clipping/fit
exceedance in C matrix. Joint-error D slow walk diverges by ~.58 s and is incomplete.
Hardware and C derivative signals differ, so matching gains do not mean same law.
New adapter initially inherited wrong motor sign; invalid `assembled_first_*` and
`assembled_fine_*` development artifacts excluded, explicit +1 and negative-
feedback assertions/tests fixed before completed C matrix. No timestep change
in reported matrix. No source models, mechanical defaults or hardware gains tuned.
Four runtime tests and executable Node controls test passed. HTTP page/data/Stop
and synthetic live/source/MuJoCo-viewer integration passed with 60 pairs per joint
(latest `build/assembled_validation_20261104/live_result_1793734555609325200/`).
Browser screenshot layout not checked; comparison plot visually inspected.
Next physical lab check uses NEW assembled app, NOT old knee live app/recorder.
Exact commands in guide; old knee applications preserved. Remaining: physical
mount/zero/travel, D instability, uncertain KT/drivetrain/preload, real streaming
timing/performance. No physical hardware run or commit/push performed.

## Latest handoff milestone: 4 November 2026 — dual-joint telemetry package

User confirmed the continuous tracking recordings used the FULLY ASSEMBLED leg.
Do not use independent fixed knee/ankle fixtures as the matching gait plant.
User requested a bounded hardware-code handoff first, mindful of usage budget.
Deliverable ZIP: `build/osl_gait_tracking_telemetry_20261104.zip` (18 files,
CRC checked); final synthetic sample count was 599 per joint.
New `hardware/gait_tracking/`: all five supplied trajectory scripts with marked
telemetry startup/publish/shutdown additions, one shared standalone `telemetry.py`,
all reference CSV dependencies, README and source-hash manifest. Portable normal-
walk dataset paths are the only non-telemetry source changes; gains, guards,
timing settings, motion and logging algorithms preserved. Old knee module untouched.
Builder: `tools/build_tracking_telemetry_package.py` (does not overwrite README).
Protocol is `osl-joint-telemetry-v1` with both joint tags, global/per-joint sequences,
control tick, raw controller signs and separate reference/sensor-read times.
Use new `experiments/record_tracking_telemetry.py`; old knee receiver/simulation
cannot consume this stream. Recorder has no dashboards or simulation yet.
Local auth/channel/drop tests and all-five-controller source-preservation test
passed; synthetic end-to-end COMPLETE with 599 samples/joint in
`build/tracking_received_20261104_001414_067189`. First demo attempt failed due to
Windows coarse monotonic timestamps; synthetic generator corrected to perf_counter
and retested. Physical scripts already use perf_counter and were not affected.
No hardware script executed. Tests require loopback network outside sandbox.
Next: operator lab telemetry check, then assembled-leg replay/controller adapter
and dashboards. Ankle coordinate/range and actual assembled mount orientation
still need resolution before quantitative comparison. No model tuning/production
controller change/commit/push performed.

## Latest inspection: 3 November 2026 — continuous tracking archive

Inspected `Downloads/Tracking Results & Codes & Dataset.zip` statically; no supplied
hardware script imported/executed, no simulation/tuning. Report:
`TRACKING_ARCHIVE_INSPECTION_2026-11-03.md`. Evidence:
`build/tracking_inspection_20261103/` (original code/CSV copies, inventory/hashes,
recording summary). Reproducible tool `tools/inspect_tracking_archive.py`.
Five code snapshots, five AB01 angle/GRF pairs, six paired knee/ankle result runs:
normal walk at two speeds, ramps up/down, stairs up/down. All logs nominal 200 Hz;
folder suffixes `0.6ms` etc are not control dt. Fixed confirmed gains and logged
PID terms match within 3.56e-15 A; no recorded clipping flags. Distinguish hardware
target errors from sim-to-real errors; no tracking simulation results yet.
Blocking interpretation questions: separate fixtures versus assembled mechanical
leg (asked user), ankle zero/direction/range (several targets outside CAD +20 deg),
source snapshot/run time-scale correspondence. Preserve loop-start vs per-joint
measurement clocks, mixed exported column signs, nonzero homed initial pose.
Next: resolve setup/ranges and run one controlled normal-walking replay before
expanding to all conditions. No model defaults or hardware code changed.

## Paper preparation milestone: 2 November 2026 — Section 3 scope

Construction audit completed as a separate working document:
`PAPER_CAD_TO_MUJOCO_AUDIT.md`; numerical evidence and source hashes in
`baselines/paper_model_construction_20261102.json`. Recomputed segment aggregation
with a centroid-centred summation and compiled the current bench/knee/ankle XMLs
using MuJoCo 3.12.0 with `mj_forward` only. Tensors agree to numerical precision;
none of the three segments needed the builder's repair fallback. Rechecked local
MyoAssist XML mass totals with/without socket. Source/model hashes unchanged.
No full tests, physics stepping, model regeneration, tuning or hardware action.
Remaining: recover CAD version, verify actual lab component correspondence and
primary-paper comparisons; then prepare subsection 3.2 prose and audit drivetrain
parameter sources. Earlier statements below describing this audit as next work
are superseded by this entry.

Follow-up: user accepted the five-subsection outline in the working notes.
Reviewed the historical prior-art/research-direction notes and all three pages
of `Downloads/ICORR_2027.pdf`, including rendered pages. The PDF is an early
T-Motor-actuated hardware draft with template content; software Section 3 has
not yet been inserted. Prior-art map explicitly records a failed search and
cannot substantiate novelty. Added a proposed outcome-independent gain-evaluation
protocol: distinguish tuning from plant identification; fixed-gain recordings
cannot establish hardware-optimal gains. No sweep, simulation, primary-literature
search or hardware operation performed. Existing PDF/Overleaf content untouched.

User owns Section 3, Software and Sim-to-Real Framework, and may revise its
subheadings. Paper preparation is a separate chat from ongoing testing/analysis;
proceed gradually with implementation/source verification before manuscript prose.
See `PAPER_SECTION3_WORKING_NOTES.md` for the tentative structure, initial evidence,
open provenance questions and next bounded task. Read-only XML checks confirmed
raw URDF mass/inertia problems, current bench/fixture masses and both fixture
source hashes matching the current bench. Code inspection found the custom CAD
lineage in current step-replay paths and an implemented optional first-order
current-loop approximation; hardware identification remains unestablished.
No simulation, tests, model regeneration, tuning or primary-PDF verification was
performed in this paper-preparation pass. Next: detailed CAD pipeline audit and
equivalent-component MyoAssist cross-check before drafting subsection 3.2.

## Latest replay milestone: 2 November 2026 — ankle steps

Completed six ankle replays (5/10/15 degrees, joint-error D and C), confirmed
gains 2/0/.075. Report `ANKLE_MULTISTEP_2026-11-02.md`; outputs
`build/ankle_multistep_20261102_results/`. Primary RMS discrepancies:
0.143611/0.237994/0.379364 degrees; simulated rise 78.07/72.67/70.33 ms vs
hardware 52.82/49.59/48.63 ms. No clipping or belt-fit exceedance.
Ankle CSV current sign is -1 relative to displayed angle PID domain; normalized
term arithmetic matches within 1.78e-15 A. Original CSVs retained intact.
Shared stepping and fixture adapter now support joint selection without renamed
knee channels in ankle outputs. Confirmed gains are explicit in ankle context;
historical ankle profile remains unchanged. 17 tests passed. Knee regression
reproduced all 12 prior comparison/simulation CSV files byte for byte.
Next: physical ankle encoder zero/direction, measured load/current conversion,
and continuous trajectory validation. Transferred drivetrain and joint losses
remain provisional; positive CSV angle mapped to positive CAD coordinate is an
explicit assumption. Earlier pending-ankle-replay statements below are superseded.

## Latest model milestone: 2 November 2026 — ankle fixture

User confirmed ankle housing clamped above, prosthetic foot attached below,
normal leg orientation. Built `models/osl_v2_ankle_fixture_with_foot.xml` with
one ankle joint, no knee, fixed housing and full CAD foot/output segment.
Moving CAD mass 0.915159 kg; inherited joint losses remain provisional.
Guide `ANKLE_FIXTURE.md`; builder `tools/build_ankle_fixture.py --load with-foot`.
Shared preview supports `--model ankle-with-foot`. Render inspected at
`build/ankle_fixture_preview/neutral.png`; two ankle tests passed, including
inertia/rotation preservation and one-second finite physics stepping.
An unused output-only diagnostic variant was also built; it is not the lab setup.
Earlier requests for mounting information are superseded by this clarification;
physical encoder zero/sign and exact moving mass still require verification.
Next: ankle controller/drivetrain adapter and 5/10/15-degree replay with confirmed
2.0/0/.075 gains. No ankle controller replay or live validation completed yet.

## Latest milestone: 2 November 2026 — knee multi-amplitude replay

Completed fixed-model knee 5/10/15-degree comparison from `Downloads/All_Step_Results.zip`.
See `KNEE_MULTISTEP_2026-11-02.md`; reproducible command is
`experiments/compare_knee_step_archive.py`. Evidence: `build/knee_multistep_20261102/`.
All 199 rows per file retained; both C and knee-error D replayed, no parameter fitting.
Knee-error D RMS differences: 0.194868, 0.326615, 0.475019 degrees. Simulated rises
82.17/74.78/72.82 ms versus hardware 53.95/55.04/52.96 ms. New 15-degree trial
differs from the original physical recording; do not attribute different RMS to
a model improvement. Five fixture tests passed; plot inspected. CSVs lack original
telemetry validity/session flags; timing reconstructed from cumulative dt.
Confirmed gains: knee 1.5/.15/.06, ankle 2.0/0/.075. Ankle mounting photo pending.
Earlier table's statement that multi-amplitude comparison is pending is superseded
by this milestone. Next: verify physical parameters, continuous-trajectory validation;
no new parameter set adopted, defaults and historical recordings preserved.

Last reviewed: **2 November 2026 (Asia/Calcutta)**. This is the current continuity
entry point for a new agent/account. It supersedes older documents' project-status
and next-step claims, not their historical results. Recheck the working tree and
newer user instructions before acting. This is a handover, not a chat export.

## Purpose and present stage

We built a CAD-derived OSL V2 simulation, current-domain controller and compliant
actuator/belt model, then a read-only hardware telemetry pipeline to compare a
physical knee response against an independent MuJoCo response. The immediate work
is **offline comparison of recorded steps, investigating remaining discrepancies,
and preparing evidence for a research paper**. Whole-body walking and IMU-derived
gait control are not the current completed result.

The user is a **co-author**. On 2 November they clarified that they have the paper
structure but **have not started their own section**. No manuscript was identified
in this audit. Obtain the structure, assigned section and latest author guidance
before drafting. `PAPER_ARCHITECTURE_REPLY_2026-11-02.md` is a draft explanatory
reply and figure notes, not the manuscript. Sending any reply is a separate action.

Working repository: `C:/Users/Goutham A.S/Documents/GitHub/osl-mujoco`.
Related folders: sibling `AB19` (human reference data), `OSL Papers` (literature
PDFs), and `myo_folder` (parked human/MyoAssist work). The lab collaborator named
in recent discussion is Preetam; correspondence was being prepared for Saurav sir.

## Read in this order

1. This file: present state and next work.
2. [Hardware integration](../hardware/README.md) and [live comparison](LIVE_KNEE_COMPARISON.md).
3. [Knee fixture](KNEE_FIXTURE.md), [derivative audit](KNEE_DERIVATIVE_AUDIT_2026-10-31.md),
   and [parameter sensitivity](KNEE_FIXTURE_SENSITIVITY_2026-10-31.md).
4. [Paper architecture notes](PAPER_ARCHITECTURE_REPLY_2026-11-02.md) and
   [research briefing](RESEARCH_BRIEFING_2026-10-30.md) for explanation/history.
5. [Control paths](CONTROL_PATHS.md), [architecture](ARCHITECTURE.md), and
   [documentation audit](DOCUMENTATION_AUDIT_2026-11-02.md) as needed.

## What is established, and what remains open

| Work | Status and evidence |
|---|---|
| Original position-servo and AB19 benchmark | Preserved regression baseline; `tests/oracle/`. Human gait reference, not robot measurements. |
| Current-domain controller/drivetrain and dashboards | Implemented; older simulation reports indexed in `RESEARCH_INDEX.md`. |
| Physical knee telemetry | A 15-degree isolated-knee step was received, compared live and saved on 30 October. `build/hardware_step_20261030_134549_560/metadata.json` identifies source `physical`; 199 paired samples are in the saved comparison. This supersedes older blanket statements that no physical test exists. |
| Mounting correction | User confirmed on 31 October: lower knee plate fixed to desk, upper thigh attachment moving; no attached thigh, ankle or foot. Separate knee-only fixture created. Its CAD mass/inertia and inherited losses remain provisional. |
| Fixture comparison | Offline replay of the same 199 samples: RMS difference 1.195285 degrees (full leg) versus 0.511719 degrees (fixture); peak 5.211969 versus 2.329077 degrees. Saved in `build/knee_fixture_replay_20261031_results/`. Historical full-leg simulated angles reproduced exactly. |
| Remaining discrepancy | Fixture rise time 72.70 ms versus recorded hardware 50.97 ms. Fixture tail RMS 0.2520 degrees versus full-leg 0.1710: not every metric improved. Single step evidence, not gait validation. |
| Derivative audit | Fixture C versus knee-error D gave RMS 0.511719 versus 0.511868 degrees. Recorded P/I/D arithmetic matched to 3.55e-15 A in the audit. Does not establish identical independent dynamic responses. `build/knee_derivative_audit_20261031/`. |
| Parameter sensitivity | Ten baseline/one-at-a-time cases completed; no new parameter set adopted. `build/knee_fixture_sensitivity_20261031/`. |
| New step archive | Received 2 November: knee and ankle 5, 10, 15 degrees; 199 rows in each of six CSVs. No 20-degree CSV. Contents inspected; multi-amplitude simulation comparison not yet completed in the reviewed evidence. |
| IMU path | Separate passive acquisition implementation and hardware-free tests. No physical deployment validation established by this audit; do not confuse it with the successful knee encoder stream. |
| Paper | Structure exists with user; assigned section and structure location still need to be supplied. Architecture PNG/SVG/PDF are under `docs/assets/research_briefing/`. |

Numbers above were checked against saved reports/results, not recomputed in this
documentation audit. A 57% reduction in one RMS metric is not general model validation.

## Hardware telemetry: what actually runs

```text
Hardware host (Pi in the reported test)
  target -> physical PID -> CAN current command -> motor/knee
                 ^                                  |
                 +---------- I2C encoder ------------+
  existing loop values -> knee_telemetry.py bounded buffer -> HTTP JSON batches
                                                            |
Laptop: receiver -> raw recording ---------------------------+
        target -> independent PID -> actuator/belt -> MuJoCo
        encoder -> one-time angle initialization + comparison only
        paired source-time errors -> dashboards and CSV recordings
```

- `hardware/knee_step_test_PID_control_me.py`: complete supplied physical step
  controller with marked telemetry additions. Startup, per-tick publish and
  shutdown hooks; do not insert them a second time. It executes physical homing
  and motor control, so it is not a laptop test script.
- `hardware/knee_telemetry.py`: standard-library HTTP server inside that process,
  port **8794**, bounded **4000-tick** ring, source/session/sequence metadata.
  Publishes existing readings; no second encoder reader or motor-command endpoint.
- `oslbench/telemetry/knee.py`: laptop batch receiver. Polling approximately every
  **50 ms** retrieves multiple samples from a nominal **200 Hz** controller.
- `oslbench/apps/knee_telemetry.py`: hardware-only receiver/recorder, dashboard 8795.
- `oslbench/apps/live_knee_comparison.py`: integrated comparison; dashboards
  **8796** (motion/error) and **8797** (simulation diagnostics), optional native viewer.
- `oslbench/sim/live_knee.py`: source-time stepping using shared simulation/control.
  `oslbench/sim/knee_fixture.py`: separate one-joint plant adapter.

The physical loop provides source timestamp, actual dt, target, measured angle,
encoder validity, requested/clamped current and status. The initial telemetry
metadata uses a -1 multiplier to present flexion-positive angles; do not blindly
apply this again to exported CSVs that already use display signs. Commanded current
is not measured motor current, and reported torque columns are not torque sensors.

Live simulation uses a **0.5 ms physics step**, **200 Hz knee controller**, default
**100 ms delivery buffer**, and compares angles at original source timestamps.
Targets are held between source samples; angles are interpolated for comparison.
The hardware never waits for the laptop. No cross-machine clock synchronization or
hard real-time lockstep is claimed. Initial velocity/belt deflection are assumed zero.
Gaps after initialization, invalid sequence/session behavior, excessive sample
intervals or backlog cause explicit failure; missing target history is not invented.
Initial truncation and pending samples at exit must be checked in run metadata.

Pause display affects a browser view only. Stop laptop run saves/stops the laptop;
it **does not stop the physical controller**. Motor shutdown belongs to the hardware
operator/controller. A single successful test does not establish timing neutrality
of the telemetry thread, completeness of every recording, or calibrated plant fidelity.

Live comparison still selects the **full-leg plant**. The **knee-only fixture has
been compared offline**; there is no fixture-selection CLI flag in the current live
application. Do not describe the offline fixture result as a live fixture experiment.

The knee profile is Kp=1.5, Ki=0.15, Kd=0.06 (current/degree units), integral limit
50 degree-seconds, C feedback, 200 Hz, provisional 0.5701 Nm/A. Hardware uses
knee-error derivative; C uses actuator-side finite differences. The preserved ankle
profile on the knee bench is different (0.75/0/0.03, 1 kHz, 0.855 Nm/A). Selecting
`--controller ankle` does not create an ankle plant. Simulation clips are not physical
safety limits. Torque/current shaft meaning, moving inertia, friction and actual
variable hardware dt remain sources of uncertainty.

## Next work: new multi-amplitude data

Original archive: `C:/Users/Goutham A.S/Downloads/All_Step_Results.zip`.
A byte-identical continuity copy is under `data/hardware/2026-11-02/` with
`manifest.json` (archive and CSV hashes, row counts, provenance). Preserve originals.
This archive's 15-degree recording is not automatically the same trial as the
30 October live recording; compare hashes/timestamps and provenance before claiming so.

1. Validate all six files: timestamps/dt, signs/zeros, clipping, controller terms,
   test identity, sample coverage and physical setup. Ankle P/error magnitudes
   indicate Kp=2 A/degree, different from the preserved ankle-derived profile;
   verify the full controller rather than guessing Ki/Kd or current polarity.
2. Implement a documented CSV-to-replay input adapter for the three knee files.
   Existing `experiments/replay_knee_fixture.py` expects `metadata.json`,
   `hardware_batches.jsonl` and an original `comparison.csv`; it cannot consume
   this ZIP or a plain hardware CSV directly. CSVs lack telemetry session/sequence
   and validity fields: any row indices or validity assumptions must be labelled
   as import metadata, not fabricated hardware observations.
3. Generalize analysis before using it on 5/10-degree steps: the existing replay
   metrics hardcode **1.5 and 13.5 degrees** for rise time and a **0.5 s** tail.
   Derive thresholds from each step's initial/final target, identify step onset,
   and explicitly handle missing crossings. Keep the original 15-degree result.
4. First compare all knee recordings with one unchanged fixture/controller baseline.
   Report RMS/peak difference, rise time, overshoot, final/tail error, current and
   timing. Label C vs hardware-style derivative separately. Preserve raw timing;
   do not shift curves to improve apparent agreement.
5. Confirm ankle fixture, moving load, coordinates and controller before ankle
   simulation. The implemented live packet/receiver is knee-specific; dual-joint
   live comparison has not been demonstrated.
6. If calibration is justified, decide calibration/validation allocation before
   tuning. The historical 15-degree test has already informed model choices, so
   it is not untouched validation data. Reserve a genuinely independent trial
   or obtain another one for a strong validation claim.

User preference expressed in the recent chat: use recorded data for fast, repeatable
model iterations; use live hardware for new trajectories/setup changes and final
validation. Do not start hardware merely to recreate an existing recording.

## Resume commands and checks

Use existing `.venv` from repository root in PowerShell. Do not upgrade it while
reproducing evidence. The README contains environment versions and setup limitations.

```powershell
git status --short
.venv\Scripts\python.exe -B experiments\replay_knee_fixture.py --help
.venv\Scripts\python.exe -B experiments\replay_knee_fixture.py
# Optional controlled derivative comparison, in a separate new output directory:
.venv\Scripts\python.exe -B experiments\replay_knee_fixture.py --feedback legacy
```

Replay defaults to the preserved 30 October recording and creates a new timestamped
folder. It does not process the new ZIP. Hardware/synthetic launch procedures are
in `LIVE_KNEE_COMPARISON.md`; start the laptop first and wait for READY. Use fresh
output folders. Never use a real token in checked-in instructions.

For implementation changes, relevant checks include:

```powershell
.venv\Scripts\python.exe -B -m unittest discover -s tests -p test_knee_fixture.py -v
.venv\Scripts\python.exe -B -m unittest discover -s tests -p test_knee_live_connection.py -v
.venv\Scripts\python.exe -B -m unittest discover -s tests -p test_live_knee_comparison.py -v
.venv\Scripts\python.exe -B tests\run_tests.py
```

Inspect the engine banner: a stub pass is not MuJoCo physics verification.
These are resume instructions, not a claim that the suites ran during this audit.

## Preservation before changing accounts

At audit start HEAD was `4f9eb68` on the local working repository, with numerous
existing modified/deleted/untracked files. Important fixture, live comparison,
telemetry and test files were untracked. **HEAD alone does not represent this work.**
Retired root redirectors were already deleted; preserve that in-progress state.
See `docs/baselines/continuity_20261102.json` for the captured status and file hashes.

- Retain the complete working folder, including untracked code/models and Git
  history. Review/commit the intended working changes after coordinating with
  active development chats; this audit does not stage, commit or push them.
- Make a separate backup of `build/`, especially the physical recording, fixture
  replay, derivative/sensitivity evidence and `AB19_knee_gait_reference.csv`.
  They are generally Git-ignored. **A manifest is not a backup.**
- Retain CAD/mesh assets (`osl_v2_0_assembly/`, `models/`), oracle, external data,
  original hardware CSVs/controller version, and paper structure/draft when supplied.
- The archive copy is inside the working folder, not a second-device backup.
  Verify external backup contents and hashes before relying on it. No account
  migration or external backup was performed by this documentation task.

Suggested first prompt on the new account:

> Read AGENTS.md and docs/PROJECT_HANDOVER.md, check the current Git status and
> available evidence, and summarize the current stage and unresolved assumptions.
> We are continuing recorded knee-step comparison and paper preparation. Preserve
> existing work and historical outputs. Then follow my next instruction.

Keep this file current as work proceeds. Documentation preserves technical context;
it does not guarantee that original app chats will be visible after account switching.
