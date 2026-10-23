# Reproduce the frozen LiDAR baseline
Clone this branch with submodules. Read PROJECT_STATUS.md first.

## Environment
Ubuntu20.04, ROS Noetic, C++17, Release -O2. Exact locally observed versions are handoff/lidar/environment.json.
System prerequisites: ROS Noetic/catkin, morai_msgs pinned beta_drive submodule, Eigen3, PCL, OpenCV, yaml-cpp, OpenSSL, GeographicLib, Python3 numpy/pandas/scipy/PyYAML/rosbag. Package manifests and CMakeLists are authoritative.
Build outer message/dependency workspace using scripts/build_ws.sh (review its environment first); then:
    source /opt/ros/noetic/setup.bash
    source devel/setup.bash
    catkin_make -C src/lidar+sensorfusion --build build/handoff -DCATKIN_DEVEL_PREFIX="$PWD/src/lidar+sensorfusion/build/handoff/devel" -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS_RELEASE=-O2 -DCMAKE_C_FLAGS_RELEASE=-O2 -DCATKIN_ENABLE_TESTING=ON -j2
    make -C src/lidar+sensorfusion/build/handoff lidar_dataset_replay
The original machine reused an existing outer dependency workspace and compiled identical frozen algorithm source; a brand-new dependency installation/container build is NOT yet validated.

## Representative data and one-command replay/evaluation
    python3 handoff/lidar/fetch_data.py --out "$HOME/asmc-repro"
    export LIDAR_SETUP="$PWD/src/lidar+sensorfusion/build/handoff/devel/setup.bash"
    python3 handoff/lidar/run.py --data "$HOME/asmc-repro/actual_prefix_0_1155_v1" --out "$HOME/asmc-repro/run-$(date +%s)"
The fetcher uses your existing SSH GitHub authentication. Do not paste tokens into chat. DATA_REF.json pins the separate data branch commit; manifests verify every file.
run.py verifies input hashes, creates an isolated network/PID namespace and own roscore, replays baseline, then evaluates GT in a separate phase. UDP is OFF. It refuses to overwrite output directories.
Linux unprivileged user namespaces, iproute2 and ROS are required. Where namespaces are disabled, use a separately isolated approved container; do not point this at a live vehicle ROS master.
Paths are configurable via --data/--out/--setup or LIDAR_SETUP. params.yaml uses @REPO@ markers resolved to the current clone; no original user paths are required.
Data prefix includes scans0..1155, full warmup0..1085; inspect1086..1155 including subsequent ID transfer. Do not cold-start at1086 and expect historical IDs.

## Tests and evidence
ROS-free Python evaluation requires numpy/pandas/scipy: existing tools/experiments/lidar_obb_quality_temporal/eval_lib.py is reused. Portable data integrity/download checks need Python3+git. Tracking unit tests need compiled ROS dependencies but no live ROS master. The separate test logs explicitly include rejected-candidate failures.
Local verification manifests and numerical/discrete comparison are handoff/lidar/evidence. Timing is excluded; exact output tolerance0 is required for the preserved prefix. Full-prefix and event-window metrics are distinct from full-lap claims.
Actual user PC only: live packets, MORAI sensor mount/GT reference, manual forward/reverse/turn, fresh valid static bag and camera/planner full-load15Hz. GitHub access does not execute the user's ROS/MORAI environment.

## Collaboration loop
External: inspect code/data -> separate branch -> available tests -> record changes/evidence.
User PC: separate checkout/worktree at the exact commit -> build/bag replay/MORAI checks -> commit curated result/manifest/logs back to a results branch (no secrets/host paths). Do not force-switch existing work.
Results must name real executing commit/config; no adoption before results. Keep data branch outside operational/main merges. Main review is an explicit human step.

Portable tests actually run: python3 -B handoff/lidar/evaluation_tests/test_response_eval.py (11 pass); python3 -B handoff/lidar/evaluation_tests/test_alignment.py (12 pass). Prefix replay at af346e9ea7c4520378f9424e1be4294e69efd680:1156/1156 identical numeric/discrete output excluding timing, tolerance0. Evaluation performed separately with fixed aligned GT. The final handoff only adds documentation/evidence around the unchanged algorithm.

End-to-end download confirmation: pinned data commit446cf839f66e45641274fd4e5669b865108e429d downloaded from GitHub, all file hashes verified. Replayed using handoff entrypoint commit5e84ac5152fe8eaeb11aaf230a1f98c9bd6e49c1:1156/1156 exact excluding timing, tolerance0. DOWNLOADED_PREFIX_* records this execution; EVENT_WINDOW.csv isolates1086..1155 after full warmup.
