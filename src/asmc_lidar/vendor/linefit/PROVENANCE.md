# linefit reference core

Source: https://github.com/lorenwel/linefit_ground_segmentation
Revision: e6b9418cc5756e30e4ea87b11d8792f936a2002a
License: BSD-3-Clause (LICENSE).

Paper: Himmelsbach et al., Fast Segmentation of 3D Point Clouds for Ground
Vehicles, IEEE IV 2010.

Only the two per-scan cout messages in ground_segmentation.cc are disabled.
Algorithm and thread partitioning are unchanged, including upstream's
integer-division tail behavior when counts are not divisible by n_threads.
That limitation must be evaluated separately from frozen 90-sector/4-thread
parity. The PCL viewer is linked but visualize=false always.

The runtime build applies ground_live_review/generate_linefit.py to a build
copy: insertionThread runs once in original point order. The upstream bin's
separate atomic z/range stores are not an atomic minimum pair; identical raw
inputs produced different labels in repeated runs. Ordered insertion repairs
that reproducibility defect. Four-thread fit/assignment and their tail behavior
remain unchanged. Report exact old comparisons and these differences separately.

V2 review adds an explicit `linefit_cover_all` option to a generated header and
source, not this vendored checkout. It partitions fitting/assignment as
`count*i/threads` to `count*(i+1)/threads`, covering the tail exactly once.
The default remains false for frozen V1 reproduction. Fixed configurations
and 1/2/4-thread coverage tests are separate from parameter tuning. Generated
point reason flags and per-sector fit-rejection counters observe existing
branches without changing them. The one-point vehicle regression identified
in sync_001231 is reported, not hidden behind a parity tolerance.
The complete raw sequence also loses 10 retained vehicle points and 41 obstacle
points with coverage-only, without an additional complete object loss. This
implementation correction is therefore not presented as an object-preservation
improvement and does not silently replace the V1 default.
