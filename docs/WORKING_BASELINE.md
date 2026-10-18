# Working baseline and provenance

The [Phase 6B inventory](baselines/phase6b_20261018.json) captures the workspace on
2026-10-18. It is not a committed revision, dependency lockfile or new simulation.
The branch was `main`, HEAD `f3b1dfeba7fbc89680cc4ec4031ed213c34252b8`.
Before editing, only NEXT_PHASE_PROPOSAL.md was modified and there were no untracked
files. The earlier f57e0cf audit is no longer the current HEAD.

The baseline is a **dirty working tree**: checking out HEAD alone does not reproduce
these documents. Preserve existing work; do not reset, clean or regenerate evidence.

## Reading and checking the manifest

- `before` records exact Git status, tracked diff/stat, untracked names and hashes
  before Phase 6B edits. The roadmap's pre-existing modification is included.
- `after` records final status, modified/untracked names and final hashes. The manifest
  excludes its own hash to avoid recursion; its path is listed explicitly.
- Source hashing covers project Python code, tests, tracked documentation/models,
  the protected AB19 reference and oracle. Environment contents, caches, unrelated
  datasets and secrets are not copied or hashed. Environment versions come from
  Python `--version`, pyvenv configuration and selected installed package metadata.
- `phase5_manifest` identifies the existing final source manifest by path/hash and
  records the pre-edit comparison: all eight files matched. The roadmap changes in
  Phase 6B intentionally make its current hash differ; the earlier manifest is untouched.
- `phase5_live_sha256` identifies every file in the completed live attempt; canonical
  evidence paths link the report and estimator/convergence history. Those runs retain
  their own source/configuration identity, not this inventory's HEAD.

Resolve manifest paths relative to the repository root. Use PowerShell
`Get-FileHash -Algorithm SHA256 -LiteralPath <path>` and compare against the relevant
snapshot. A mismatch means investigate the change; it is not permission to overwrite
anything. Create a new uniquely named inventory for subsequent work.

Python and package versions are observed locally, not proof of a reproducible clean
install. No full dependency lock exists. Hardware fidelity, device interfaces and
physical safety limits remain unverified. Tests, simulations and oracle execution
were deliberately not run for Phase 6B; historical successful executions remain linked
through the [research index](RESEARCH_INDEX.md).
