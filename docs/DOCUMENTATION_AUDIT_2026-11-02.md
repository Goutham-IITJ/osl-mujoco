# Documentation continuity audit — 2 November 2026

## Conclusion

Substantial technical documentation exists, including knee telemetry. It was not
a reliable stand-alone onboarding path: central status pages stopped at October 18,
later fixture results were scattered, and the newest dataset/next steps lived in
chat and Downloads. `PROJECT_HANDOVER.md` now supplies the current entry point.

## Scope and method

Inventoried every Markdown document at repository root, docs/, and hardware/;
reviewed their headings/status and evidence navigation. Read the current telemetry,
fixture, derivative, sensitivity, control, architecture and entry-point documents
in detail and checked key claims against implementation and saved JSON/CSV. Older
long technical analyses were mapped by scope rather than re-derived line by line.
This was not a complete literature/PDF review or a new scientific validation.
The single repository PDF is the architecture figure, with editable SVG/source.
Sibling OSL Papers contains literature PDFs; no manuscript was identified.

Reviewed the ten most recent turns of the project chat “Check context from other
chats” (local ID `01a15a92-7279-78e0-89d1-704f763be3e9`) for recent decisions,
then verified the incoming archive and saved evidence locally. Older chat history
was not exhaustively exported. No messages were sent to collaborators.

The user clarified in this audit that they are a co-author, possess the paper
structure, and have not started their section. Structure/assignment are still
needed; the architecture reply must not be mistaken for a manuscript.

## Gaps addressed

- Added AGENTS.md and PROJECT_HANDOVER.md with current state and a restart prompt.
- Qualified obsolete status in README, roadmap, research index, baseline, architecture,
  control-path and telemetry guides while retaining historical evidence.
- Corrected live-viewer refresh documentation to 10 Hz, matching code.
- Recorded physical-vs-synthetic, full-leg-live-vs-fixture-offline and IMU distinctions.
- Recorded CSV ingestion and 15-degree metric assumptions blocking direct reuse
  of the replay script with the new multi-amplitude dataset.
- Copied the original step ZIP into data/hardware/2026-11-02/ and hashed its CSVs.
- Corrected the misleading .gitignore comment that all build outputs are reproducible.
- Added a continuity file inventory; it is not a backup or a clean release.

## Remaining preparation

1. Provide the paper structure, assigned section, intended claims and author feedback.
2. Review and commit intended untracked/modified source with the ongoing development
   work; this audit did not stage or commit it.
3. Back up ignored physical recordings/reference inputs and working files separately.
   No off-machine backup was made.
4. Implement/verify the new CSV input path and amplitude-aware metrics before
   multi-step comparisons. Confirm ankle fixture/controller and dataset provenance.
5. Keep handover and evidence inventory current after subsequent development.

## Document map

Current entry means useful for starting now; historical does not mean scientifically
invalid. Date/scope govern interpretation. See the handover for current priorities.

| Document | How to use it |
|---|---|
| [AGENTS.md](../AGENTS.md) | Agent entry instructions |
| [BENCH_GAIT_TRACKING_DESIGN.md](../BENCH_GAIT_TRACKING_DESIGN.md) | Historical research/design/meeting evidence; read in its original scope |
| [BENCH_TUNING_AND_DATASET.md](../BENCH_TUNING_AND_DATASET.md) | Historical research/design/meeting evidence; read in its original scope |
| [docs/ACTUATOR_DYNAMICS_ANALYSIS.md](ACTUATOR_DYNAMICS_ANALYSIS.md) | Historical research/design/meeting evidence; read in its original scope |
| [docs/ANKLE_PARAMETER_SENSITIVITY.md](ANKLE_PARAMETER_SENSITIVITY.md) | Historical research/design/meeting evidence; read in its original scope |
| [docs/ARCHITECTURE.md](ARCHITECTURE.md) | Technical or workflow reference; current handover qualifies project-wide status |
| [docs/ARMATURE_PILOT.md](ARMATURE_PILOT.md) | Historical research/design/meeting evidence; read in its original scope |
| [docs/BENCH_EXPERIMENT.md](BENCH_EXPERIMENT.md) | Technical or workflow reference; current handover qualifies project-wide status |
| [docs/CAD_FINDINGS.md](CAD_FINDINGS.md) | Technical or workflow reference; current handover qualifies project-wide status |
| [docs/CODE_MAP.md](CODE_MAP.md) | Historical research/design/meeting evidence; read in its original scope |
| [docs/CONTROL_PATHS.md](CONTROL_PATHS.md) | Technical or workflow reference; current handover qualifies project-wide status |
| [docs/DEMO_AND_HARDWARE_2026-10-30.md](DEMO_AND_HARDWARE_2026-10-30.md) | Historical research/design/meeting evidence; read in its original scope |
| [docs/DRIVETRAIN_INSTABILITY_DIAGNOSIS.md](DRIVETRAIN_INSTABILITY_DIAGNOSIS.md) | Historical research/design/meeting evidence; read in its original scope |
| [docs/DRIVETRAIN_INTEGRATION_OPTIONS.md](DRIVETRAIN_INTEGRATION_OPTIONS.md) | Historical research/design/meeting evidence; read in its original scope |
| [docs/DRIVETRAIN_PARAMETERS.md](DRIVETRAIN_PARAMETERS.md) | Technical or workflow reference; current handover qualifies project-wide status |
| [docs/IMU_TELEMETRY.md](IMU_TELEMETRY.md) | Technical or workflow reference; current handover qualifies project-wide status |
| [docs/KNEE_CONTROLLER_COMPARISON.md](KNEE_CONTROLLER_COMPARISON.md) | Technical or workflow reference; current handover qualifies project-wide status |
| [docs/KNEE_DERIVATIVE_AUDIT_2026-10-31.md](KNEE_DERIVATIVE_AUDIT_2026-10-31.md) | Current continuation / recent result (retain stated limitations) |
| [docs/KNEE_FIXTURE.md](KNEE_FIXTURE.md) | Current continuation / recent result (retain stated limitations) |
| [docs/KNEE_FIXTURE_SENSITIVITY_2026-10-31.md](KNEE_FIXTURE_SENSITIVITY_2026-10-31.md) | Current continuation / recent result (retain stated limitations) |
| [docs/LIVE_DEMO.md](LIVE_DEMO.md) | Technical or workflow reference; current handover qualifies project-wide status |
| [docs/LIVE_KNEE_COMPARISON.md](LIVE_KNEE_COMPARISON.md) | Current continuation / recent result (retain stated limitations) |
| [docs/MEETING_DEMO_COMMANDS_2026-10-26.md](MEETING_DEMO_COMMANDS_2026-10-26.md) | Historical research/design/meeting evidence; read in its original scope |
| [docs/MEETING_READINESS_2026-10-26.md](MEETING_READINESS_2026-10-26.md) | Historical research/design/meeting evidence; read in its original scope |
| [docs/MYOASSIST_FINDINGS.md](MYOASSIST_FINDINGS.md) | Historical research/design/meeting evidence; read in its original scope |
| [docs/MYOASSIST_PIVOT.md](MYOASSIST_PIVOT.md) | Historical research/design/meeting evidence; read in its original scope |
| [docs/NEXT_PHASE_PROPOSAL.md](NEXT_PHASE_PROPOSAL.md) | Dated roadmap/index/snapshot; current handover takes precedence for status |
| [docs/PAPER_ARCHITECTURE_REPLY_2026-11-02.md](PAPER_ARCHITECTURE_REPLY_2026-11-02.md) | Current continuation / recent result (retain stated limitations) |
| [docs/PHYSICAL_ANKLE_CONTROL_ALIGNMENT.md](PHYSICAL_ANKLE_CONTROL_ALIGNMENT.md) | Historical research/design/meeting evidence; read in its original scope |
| [docs/PRIOR_ART_AND_DIRECTION.md](PRIOR_ART_AND_DIRECTION.md) | Historical research/design/meeting evidence; read in its original scope |
| [docs/PROJECT_HANDOVER.md](PROJECT_HANDOVER.md) | Current continuation / recent result (retain stated limitations) |
| [docs/README_PRE_PHASE6B.md](README_PRE_PHASE6B.md) | Historical research/design/meeting evidence; read in its original scope |
| [docs/REFACTOR_MAP.md](REFACTOR_MAP.md) | Historical research/design/meeting evidence; read in its original scope |
| [docs/RESEARCH_BRIEFING_2026-10-30.md](RESEARCH_BRIEFING_2026-10-30.md) | Historical research/design/meeting evidence; read in its original scope |
| [docs/RESEARCH_INDEX.md](RESEARCH_INDEX.md) | Dated roadmap/index/snapshot; current handover takes precedence for status |
| [docs/TELEMETRY_REPLAY.md](TELEMETRY_REPLAY.md) | Technical or workflow reference; current handover qualifies project-wide status |
| [docs/TELEMETRY_SOURCE.md](TELEMETRY_SOURCE.md) | Technical or workflow reference; current handover qualifies project-wide status |
| [docs/VALIDATION.md](VALIDATION.md) | Historical research/design/meeting evidence; read in its original scope |
| [docs/WEEKLY_GAIT_PROGRESS_REPORT.md](WEEKLY_GAIT_PROGRESS_REPORT.md) | Historical research/design/meeting evidence; read in its original scope |
| [docs/WEEKLY_KNEE_TELEMETRY_REPORT_2026-10-26.md](WEEKLY_KNEE_TELEMETRY_REPORT_2026-10-26.md) | Historical research/design/meeting evidence; read in its original scope |
| [docs/WORKING_BASELINE.md](WORKING_BASELINE.md) | Dated roadmap/index/snapshot; current handover takes precedence for status |
| [hardware/README.md](../hardware/README.md) | Current continuation / recent result (retain stated limitations) |
| [README.md](../README.md) | Current continuation / recent result (retain stated limitations) |

## Validation of this documentation change

Checked local Markdown targets in the changed entry points/handover, archive-copy
identity, CSV row counts and hash inventory. Checked the documented CLI flags
against help output and implementation. No simulation, physical test, controller
change, parameter tuning, manuscript editing or full regression run was performed.
Historical numerical results are attributed to the saved reports.
