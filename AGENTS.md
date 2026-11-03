# Project continuity

Before continuing development, read `docs/PROJECT_HANDOVER.md`. It records the
current work, evidence, unresolved questions, and next steps without relying on
access to previous chats. Use `docs/DOCUMENTATION_AUDIT_2026-11-02.md` to navigate
the older documentation. Recheck Git status because other chats may be editing.

Preserve existing user work, untracked files, recordings, and historical results.
In particular, `build/` contains irreplaceable physical recordings and reference
inputs despite being ignored by Git. Do not clean it or overwrite old runs.
Use new output directories. Do not update the frozen oracle to hide regressions.

The active work is recorded knee-step comparison and model investigation.
Live hardware comparison, offline fixture replay, AB19 simulation, passive IMU
telemetry, and pose mirroring are distinct workflows. Read their specific guides
before using them. Current commands and ownership packages supersede historical
root-module imports; do not restore removed compatibility modules by accident.

Separate measured, simulated, derived, and assumed quantities. Commanded current
is not measured current. A successful physical telemetry test is not validation
of the complete plant or all trajectories. Preserve gains and mechanical defaults
unless the requested experiment explicitly varies them and records provenance.

At a meaningful milestone, update `docs/PROJECT_HANDOVER.md` with the date,
completed work, evidence paths, validation actually performed, remaining issues,
and next step. Keep historical reports intact. Never put passwords or tokens in
documentation. Work on hardware commands only within the user's explicit scope.
