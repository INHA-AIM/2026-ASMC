# PR141 team integration checkpoint

Baseline: #141 `9f711282`, main `af911727` (already an ancestor). Sources: #142 `e38b328d`, #143 `a7831212`, #144 `1b7780c1`. Each source is preserved by a merge commit. Original PRs remain open; no main merge.

## Runtime protection

The running `asmc-jamming-compare` remains on the previous checkpoint, master 11373, dashboard 8089 and LiDAR 8097. Integration uses `2026-ASMC-jamming-pr141-sync-20261030`, container `asmc-jamming-pr141-sync-build`, independent `/build` and `/devel`. Do not change the active checkout or overlay during the user's drive. This is a staged integration, not deployment to the active container.

## Decisions

- camera is PR142's BBOX/LiDAR and person/dog state machine, including 5s minimum observation stop, latest-location reselection and 4s re-stop suppression. Removed PR137 guards are not restored. Emergency/input/speed checks remain those of PR142. Added only PR141 prepared-handoff epoch/candidate and applied-steering interfaces.
- Default camera speed changes from 10 to 20km/h; avoidance10, creep3. Camera clearance/lateral margin .05/.05m, overrun1.5m, shoulder2m. These values do not change LOCAL/localization profiles. A448 approach25 and common selector/watchdog remain.
- PR143 READY notice is per behavior command, consumed once, expired/cancelled/terminal notices discarded; actual departure is rechecked. Gaps front .6s/rear1s. This transaction does not reuse jamming epochs. Recording script requires explicit container/master/registered run and a writable D mount.
- Generic `lidar.launch` now selects `lidar_wall_vehicle.yaml`, enabling the full PR144 window and three-view measured-history settings. Explicit `pipeline_config` can still select the generic YAML. sensor Z -1.55m is not a ground-height threshold. No steel-model experiment imported.
- Both LiDAR YAML vehicle rear extents change .700 to the planner's canonical .790m. Width1.892/length4.635/wheelbase3/front-overhang.845 and sensor mounts stay fixed. ROI fixture distinguishes the extra 9cm rear self-mask from front/lateral boundaries.
- NORMAL retains the PR141 geometry and general control; its roundabout transaction/gaps and its shared LiDAR input intentionally receive the new changes.

## Evidence and acceptance

Run: `jamming-pr141-sync-20261030-r01`. Build/test logs and final identity belong to its D output. Original PR tests and previous PR141 drives are provenance, not acceptance of this integration. Full-course, camera20km/h and repeated cross-strategy comparisons remain pending until actually tested. Final local/remote results are recorded at freeze.

## Rollback and activation

Rollback source is `9f711282` and `freeze/jamming-20261030-0130-r2`; preserve the old container profile/overlay. Do not reset a dirty checkout. After the user stops the old autonomy/perception, activate the new checkout/overlay as one identity and restart camera, lidar, views and exactly one autonomy. Keep master11373 and current console ports for this local deployment; repository operational defaults remain11311/8088/8091. Port occupancy is never resolved by killing a foreign owner.

## 06:20 integration plan update — camera point extraction scope

Remote integration HEAD was rechecked at 06:05 KST on 2026-10-30:
`cf2168465bb33099e5a75da6d1d1605d336e48b2` includes Jaeho's
`fix(lidar): restrict camera point extraction to tunnel approach window`.
This is an incoming integration requirement, not a declaration of deployment:
the active compare checkout was still `47d9d2d2` when inspected. The current
execution location is `2026-ASMC-jamming-compare` / `asmc-jamming-compare`; the
earlier staged-build paragraph above describes the previous integration stage.

The next integration uses the newest remote integration tip (including this
commit and Yuntae's roundabout settings), followed by Seojun's
`fix/merge-response-consistency-20261029` tip, currently `25f26f3b`. Preserve
local localization/LOCAL recovery commits as well. Recheck tips immediately
before merging; do not replace the remote branch with the older local history.

Jaeho's ownership boundary must remain explicit:
- Camera-owned point extraction, residual splitting, steel fragment extraction,
  vehicle measurement history and related wall cleanup are restricted by the
  existing verified tunnel-route window when `tunnel_only_updates: true`.
- The window retains its existing 30m lead, 10m exit buffer and exit hysteresis.
  GPS loss carries an already verified window; GPS loss elsewhere never arms it.
- Outside the window both `refinement.enabled` and `steel_fragment_merge` are
  forced OFF in the frame-local configuration. Refinement reports disabled and
  projects zero points. Do not merely disable history while extraction remains ON.
- Ordinary three-view camera class matching and HD-map delivery protection stay
  global. A missing YOLO box does not partition an ordinary-road cloud through
  the tunnel-specific extraction path. General camera classification is not
  being removed from the roundabout.
- Preserve the versioned runtime/config identity
  `tunnel-only-updates-v2-camera-geometry-scope`; an old running LiDAR binary is
  not accepted as evidence for the new scoped behavior.

Integration order: snapshot current failure and source identities; merge the
latest team integration and Seojun changes; run build/contract checks; deploy the
new LiDAR/planner identity to the compare stack; then test early planner
preparation and test2. Planner preparation begins at tollgate A451, while the
LiDAR extraction window remains its reviewed 30m lead. Do not widen LiDAR
geometry processing to the tollgate just because planner preparation starts there.

Required checks: camera-present ordinary-road/roundabout scans have disabled
refinement and zero projected points; exact 30m approach enables extraction;
GPS loss inside retains it; exit clears it; startup/GPS loss outside cannot
rearm it. Reuse Jaeho's early-object replay to check that one object does not
split into two tracks. His reported 144 related tests and 782-scan replay
(100 early-target scans, previously 9 split scans) are contributor evidence;
report our post-integration checks separately. Preserve Yuntae's roundabout
front gap and Seojun's ordinary/localization avoidance separation.

This plan update does not restart a live node, change LiDAR thresholds, or claim
full-course acceptance. The 06:20 closeout must name the integrated source SHA,
actual running configuration identity, tests completed and any remaining gap.
