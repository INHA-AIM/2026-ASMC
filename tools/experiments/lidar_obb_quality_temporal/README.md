# LiDAR OBB observation-quality temporal stabilisation experiment (experiment 02) tools

Isolated experiment tooling on branch `exp/lidar-obb-quality-temporal-*`. Nothing here is used by production
launch files. The contract and evaluator were committed before the baseline run and are SHA256-locked in
`results/BASELINE_LOCK.json` of the run root.

| file | role |
|---|---|
| `EVAL_CONTRACT.json` | frozen evaluation contract: inputs, canonical long/short/axis-yaw metrics, observability buckets, split/over-merge and ROI-boundary definitions, C/D justification rule, tuning/holdout protocol, gates and verdict rules |
| `prepare_inputs.py` | freezes per-scene inputs/GT from `/home/autonav/Lidar_Data` (read-only; bytecode writing disabled inside the script) |
| `extract_bag_gt.py`, `bag_to_scene.py` | static GT bag -> scene-form input (read-only on the bag) |
| `eval_lib.py` | per-scene matching + metrics, point-dump features, split/over-merge attribution, clustering counterfactual |
| `diagnostics.py` | baseline observability / split-merge / ROI-boundary analyses, C/D justification, event list + review sample |
| `render_events.py` | renders the stratified event review sample to PNG |
| `compare.py`, `tune_eval.py` | baseline-vs-candidate verdict (literal contract); development-only configuration selection |
| `identity_check.py`, `lock_baseline.py`, `collect_tests.py` | default-false identity (jsonl + points), baseline freeze, single-source test counting from JUnit XML |
| `run_arm.sh` | one arm over the frozen scenes in one isolated namespace (uses the run root's `tools/ns_batch.sh`) |
| `infra/` | copies of the run-root infrastructure scripts (snapshots, isolation, builds, tests, sanitizer, perf, B0 truth table) |

Harness: `src/lidar+sensorfusion/src/Main/test/lidar_dataset_replay.cpp` (test-only target, adds `prof` stage counts
and the optional `--points-out` cluster point dump to the experiment-01 harness).
All Python must run as `PYTHONDONTWRITEBYTECODE=1 python3 -B ...`.
