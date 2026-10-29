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
