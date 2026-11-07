# CAD-to-MuJoCo construction: evidence audit for Section 3.2

Date: 2 November 2026 (Asia/Calcutta).

This separate working document establishes the construction process and its
limits before manuscript drafting. It concerns the current local files, not an
exhaustive reconstruction of every earlier development attempt. The user recalls
obtaining the OSL V2 Onshape assembly as a URDF through the OSL website/GitHub;
the exact upstream Onshape document/version, export date and export settings
remain to be recovered. Do not replace that missing provenance with a guessed URL.

The present simulation retains the custom CAD-derived model lineage. The
inspected recorded-step paths select fixture derivatives of that model, rather
than substituting the MyoAssist model. Model-construction correctness and physical
predictive accuracy are separate questions.

## 1. Evidence and verification performed

Primary local implementation:

- [CAD parser and mesh integration](../tools/oslcad.py).
- [Native MJCF builder](../tools/build_mjcf.py).
- [Knee fixture builder](../tools/build_knee_fixture.py) and
  [ankle fixture builder](../tools/build_ankle_fixture.py).
- [Bench loader](../oslbench/sim/model.py),
  [recorded-step comparison](../experiments/compare_knee_step_archive.py), and
  [ankle replay adapter](../oslbench/sim/ankle_fixture.py).

Supporting historical records:
[CAD findings](CAD_FINDINGS.md),
[MyoAssist findings](MYOASSIST_FINDINGS.md), and
[initial Section 3 working notes](PAPER_SECTION3_WORKING_NOTES.md).
Historical comments are checked against current code rather than assumed current.

This audit inspected the implementation, recomputed segment properties in memory,
compiled three current models with real MuJoCo 3.12.0, and called `mj_forward`.
It did not regenerate model files, step physics, run a gain sweep, operate hardware,
or rerun the full test suite. Source/model SHA-256 hashes were unchanged across
the reconstruction check. Detailed numerical results and hashes are retained in
[the audit evidence JSON](baselines/paper_model_construction_20261102.json).

## 2. Why direct use of the export was insufficient

The [retained URDF](../osl_v2_0_assembly/urdf/osl_v2_0_assembly.urdf) contains 682
links and 681 joints. A read-only check found 657 inertial blocks whose declared
masses sum to 0.014753439 kg. Fifty-two inertia tensors have at least one
non-positive eigenvalue. These properties are not a plausible dynamic model of
the complete device.

The current builder attributes the mass problem to absent material assignments
in the export. The raw file establishes the numerical problem; it does not by
itself establish what happened in the upstream Onshape material settings.

The builder also records earlier incorrect assemblies caused by ignoring visual
origins. This is a different issue from invalid inertial values. The paper should
explain both rather than saying vaguely that all frames were incorrect.

The builder comments describe why a compiler option did not solve the original
URDF-import failure. This audit did not reproduce that failure on historical
MuJoCo versions; avoid a universal claim about all importers or versions.

## 3. Current construction process, in order

### 3.1 Resolve the assembly and mesh transforms

`OslCad` reads joint parent/child relationships and joint-origin translations and
rotations. It computes link poses at zero joint displacement by traversing the
assembly tree. Roll-pitch-yaw is interpreted as `Rz(yaw) Ry(pitch) Rx(roll)`.

`visual_origins()` separately reads each visual mesh origin. `geom_pose()`
composes the link pose and visual origin:

\[
{}^W T_{G_i} = {}^W T_{L_i}\,{}^{L_i}T_{G_i}.
\]

Here W is the CAD world frame, L_i the link frame and G_i the mesh frame. The
same composed pose is used for geometry placement, mesh centroids, inertia
rotation and collision construction. This avoids placing meshes using the
link frame alone. Mesh filenames are resolved with a case-insensitive lookup.

The current export has no non-unit mesh scales and no links with multiple visual
elements. The parser reads one visual mesh per link; do not describe it as a
general converter supporting arbitrary URDF mesh scaling or multiple visuals.

### 3.2 Recover the intended rigid segments

The parser contracts fixed joints into rigid clusters using union-find. Clusters
without meshes do not contribute geometry to the emitted device. The many
non-fixed CAD mate joints are not all retained as simulation degrees of freedom.

The builder assigns geometry-bearing clusters to three segments: `knee_prox`,
`shank`, and `foot`. It uses volume-weighted cluster centroids, the authored knee
and ankle axis locations, and a 20 mm sagittal-distance rule for axis-concentric
clusters. Knee-concentric output clusters are assigned to `knee_prox`;
ankle-concentric output clusters to `foot`. Other clusters are assigned by their
height relative to the axes.

This encodes the intended belt-driven topology, where output pulleys move
relative to their housings. It is a model-specific reconstruction rule that
requires mechanical review, not a generic consequence of URDF syntax. The
20 mm threshold is an algorithm choice, not an identified physical parameter.

The full-leg bench emits two hinges. Axis locations are declared in the builder
and their CAD derivation is documented in `CAD_FINDINGS.md`; the current audit
does not independently re-identify those axes from hardware.

### 3.3 Compute mesh mass properties and assign densities

`mesh_mass_props()` integrates signed tetrahedra formed by each mesh triangle
and the origin to obtain volume, centroid and a centroidal inertia tensor at
unit density. It also checks whether every undirected edge occurs twice after
coordinate rounding. All 260 cached distinct meshes passed that edge-count check
in this audit. That check alone does not prove consistent orientation, absence
of self-intersections, or physical correctness of internal geometry.

An ordered set of part-name rules assigns densities, including 2700 kg/m^3 for
default aluminium, 7850 for steel families, 1250 for covers/belts, and 1550 for
the carbon-fibre foot. Electronics and battery assemblies use effective assigned
densities. These are modeling assumptions, not measured material properties or
measured component masses. A plausible density for a material does not establish
the effective density of a complex assembly.

For part i with volume V_i and assigned density rho_i:

\[
m_i=\rho_i V_i,\qquad
c_i^W=p_{G_i}^W+R_{G_i}^W c_i^{G_i},\qquad
I_i^W=\rho_i R_{G_i}^W I_{i,\rho=1}^{G_i}(R_{G_i}^W)^T.
\]

The inertia tensors in this expression are about each part's own centroid.

### 3.4 Aggregate mass, centre of mass and inertia

For a segment S:

\[
M_S=\sum_{i\in S}m_i,\qquad c_S={\sum_{i\in S}m_i c_i\over M_S},
\]

\[
I_S=\sum_{i\in S}\left[I_i+m_i\left(
\lVert c_i-c_S\rVert^2\mathbf{1}-(c_i-c_S)(c_i-c_S)^T\right)\right].
\]

Here bold 1 denotes the 3-by-3 identity matrix. The implementation first sums
about the world origin and then shifts to the combined centre of mass. The audit
used the equivalent expression above, directly about the combined centre.

This is part-wise inertial reconstruction followed by segment aggregation.
There are three rigid segments in the bench, not hundreds of independently
moving bodies. Any paper wording such as "per-link inertia model" must preserve
that distinction.

### 3.5 Simplify displayed geometry while retaining mass

Meshes with volume below 0.10 cm^3 are omitted from the emitted visual geometry.
Both retained and omitted parts remain in the segment inertial calculation.

| Segment | Parts included in inertials | Retained visual parts | Omitted visual parts | Reconstructed mass (kg) |
|---|---:|---:|---:|---:|
| knee_prox | 88 | 28 | 60 | 0.449846436 |
| shank | 489 | 142 | 347 | 3.590757963 |
| foot | 73 | 47 | 26 | 0.915158936 |
| Total | 650 | 217 | 433 | 4.955763334 |

Counts refer to part instances; 260 is the distinct-mesh count. Tiny-part
omission reduces visual complexity without discarding their computed mass or
inertia. It does not establish measured mass fidelity.

### 3.6 Emit MJCF inertials and separate collision geometry

`principal()` diagonalizes each symmetric segment inertia tensor and emits a
principal-axis quaternion and diagonal moments. It includes a fallback to floor
small eigenvalues and enforce the inertia triangle inequality. None of the
three reconstructed segment tensors required that fallback in this audit.

The current bench explicitly sets `inertiafromgeom="false"` and
`balanceinertia="false"`. Its inertials are supplied by the builder. Visual
meshes have disabled contact; separate protective module boxes and a foot mesh
provide collision geometry. This does not model deformable foot mechanics, and
collision simplification is not an exact geometric representation of every part.

Ground-scene contact and flat-foot pose settings are additional authored model
choices. The current isolated-fixture results do not validate walking contact.

## 4. Connection to the models used now

| Workflow | Model artifact | Relationship to the reconstruction |
|---|---|---|
| Full-leg bench | `models/osl_v2_bench.xml` | Three CAD-derived rigid segments; proximal segment fixed; two hinges |
| Recorded isolated-knee comparison | `models/osl_v2_knee_fixture.xml` | Knee housing fixed; proximal output/attachment segment moves; same reconstructed proximal inertia |
| Recorded ankle-with-foot comparison | `models/osl_v2_ankle_fixture_with_foot.xml` | Ankle housing fixed; complete reconstructed foot/output segment moves |

The knee builder reverses the parent/child arrangement and the axis sign relative
to the full-leg chain to preserve the intended relative angle. The ankle builder
translates the ankle axis to the fixture origin while preserving CAD orientation.
Both fixture manifests identify the current bench XML hash as their source.

All three XMLs compiled successfully with a 0.5 ms timestep. Their compiled
centroidal inertia tensors matched the reconstructed tensors within
2.3e-14 kg m^2 in the largest matrix-element difference. The alternative segment
summation matched the builder within 4.9e-17 kg m^2.

This cross-check reuses the existing mesh integrator, density assignments,
transforms and segmentation. It checks aggregation and emitted/compiled tensor
consistency; it is not an independent verification of all geometry calculations.
The full six-dimensional body frame and physical encoder mapping are not
validated by comparing tensors alone.

Fixture omission of fixed housing mass is appropriate for a rigidly anchored
plant's moving dynamics. It must not be interpreted as the physical device's
total mass. Mount flexibility and actual moving-component correspondence remain
experimental questions.

## 5. MyoAssist mass cross-check and its limits

The local `assist_sim/models/OpenSourceLeg/KA_L1model.xml` was read directly,
without loading or changing the MyoAssist environment. Its path and hash are in
the evidence JSON. Its explicit body masses sum to 5.377474 kg, including a
0.849991 kg socket. Excluding that socket gives 4.527483 kg.

| Local model accounting | Mass (kg) |
|---|---:|
| CAD bench, no socket | 4.955763334 |
| MyoAssist XML, socket included | 5.377474 |
| MyoAssist XML, socket excluded | 4.527483 |
| CAD bench minus MyoAssist excluding socket | +0.428280334 |

Removing the socket improves comparability but does not establish identical
component sets or segment boundaries. The local MyoAssist foot body is 0.291 kg;
our foot/output segment is 0.915159 kg and includes multiple output components.
Those names do not establish that the same objects were weighed or modeled.

The builder's optional motor-mass addition is zero. Its comments hypothesize
omitted motors as an explanation for a total-mass gap. That explanation is not
established by this audit, and comparing against a socket-inclusive total can
reverse the apparent direction of the discrepancy. Do not infer motor mass from
the difference and then cite the resulting total agreement as validation.

This is a local XML comparison, not verification of a published paper's mass
measurement, and not evidence that either model is physically more accurate.

## 6. Claim-to-evidence decisions for the paper

| Candidate statement | Decision and evidence |
|---|---|
| A custom pipeline generated native MJCF from the retained OSL CAD export. | Supported by builder, parser and model lineage; recover upstream revision for the citation. |
| Direct use of exported inertials was unsuitable. | Supported by raw mass total and non-positive tensors; avoid unsupported claims about every importer/version. |
| Mesh poses include link and visual-origin transforms. | Supported by `geom_pose()` and its consumers. |
| Individual part inertials are combined using rigid-body transformations and the parallel-axis theorem. | Supported by implementation and the alternative summation check. |
| Small visual parts are omitted without omitting their inertial contribution. | Supported by separate culling and aggregation paths. |
| Current fixture replays derive from this CAD model. | Supported by selected execution paths, fixture builders, source hashes and inertia agreement. |
| All physical masses/inertias were measured or experimentally identified. | Unsupported; densities and component correspondence remain provisional. |
| Every emitted tensor required repair. | False for this audit: none of the three segment tensors needed repair. |
| This is the first OSL MuJoCo model or is more accurate than MyoAssist. | Not established; do not use. |

## 7. Items to resolve before final manuscript prose

1. Recover the original CAD link, revision, export settings and any lab geometry
   modifications. Preserve exact input hashes and version the conversion scripts.
2. Match the lab's T-Motor build, foot, attachment hardware and moving components
   to the modeled component list. Weigh the relevant assemblies where available.
3. Check density assignments and omitted motor/electronics representations;
   quantify their effect instead of inferring missing components from a total.
4. Mechanically review the hinge axes, cluster assignment and fixture boundary
   conditions; confirm physical encoder zero and direction separately.
5. Distinguish reconstruction of rigid-body inertia from explicit actuator
   inertia, armature and friction. Resolve possible overlap in Section 3.3/3.5.
6. Check primary CAD/model sources and MyoAssist papers before adding literature
   comparisons or novelty language. No new online literature search was part of
   this local construction audit.

Subsection 3.2 can follow the process in Section 3 of this audit, with a compact
pipeline figure and a clear density/geometry limitation statement. The detailed
part rules and audit artifacts can be referenced as supplementary material.
Final prose should describe the reproducible method rather than recounting
assistant interactions or every unsuccessful development attempt.
