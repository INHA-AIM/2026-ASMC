# V4 frozen candidate experiment
Two opt-in policies, no parameter search after GT scoring.
- Jaeho: keep the same three RANSAC attempts and 0.30 m distance. Accept normal-road
  planes only when origin height is -2.25..-1.35 m. A rejected horizontal plane is
  excluded from plane search but its points are kept as nonground, not silently
  removed as wall. No temporal plane reuse without motion compensation.
- Patchwork++: keep fixed1.80, RNRoff, segmentation/adaptation unchanged. Replace
  the flat25cm guard ONLY where a near-field road plane is supported. Fit within
  radius5..15m/z[-2.9,-.7], 60 RANSAC iterations/0.12m, >=30 inliers/35%,
  tilt<=15deg, height[-2.1,-1.5], median residual<=.08m, span>=4x2m.
  Support domain: inlier XY bounds+2m and radius<=17m. Else keep old guard.
  This is an environment adapter improvement, not new upstream Patchwork++.
Inputs: old sync1439/raw4048; new general361/tunnel2360/banked754, with same
roll/pitch, ROI/voxel/calibration, exact GT and original point lineage.
Frozen gates:
1. Against EACH method's existing V3, no extra complete actor loss on raw/sync;
   class total and observed-lower retention do not decrease; uncertain not increased.
2. Jaeho: positive-height selections zero, height jumps reduced >=75% on tunnel;
   missing plane <=5% of tunnel sequence.
3. Patchwork++: banked asphalt residual relative reduction >=25%, in addition to1.
   Background topology sampled every20frames identically; actor topology all frames.
If both pass, prefer stronger worst-class object/lower retention, then fragment
diagnostics and cost. If neither passes, keep operational baseline and explicitly
report the failed gates. No tuning to force passing, no planner/output replacement.
New drives have NO actor GT; they cannot pass gate1 by themselves.

## Pre-scoring addendum: isolated second Jaeho policy

The first height gate produced31 unselected tunnel frames despite eliminating
positive planes. Keep those results intact. `jaeho_support` adds a same-scan
near-field RoadAnchor only when those three RANSAC attempts fail. No changed
thresholds: use the above frozen supported domain and the original0.30m ground
distance; keep points outside support. No past-plane reuse. Record source/domain
and unselected frames separately. Re-run the same actor gates, with a further
target of zero unselected tunnel scans; report ground-mask residual bursts as well
as plane jumps so abstention cannot masquerade as stability. This is a separate
implementation hypothesis, not parameter search over the failed GT objects.
