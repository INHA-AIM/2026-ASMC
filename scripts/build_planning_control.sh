#!/usr/bin/env bash
# ROS/catkin과 분리된 UDP-only Planning/Control 빌드.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_JOBS="${BUILD_JOBS:-$(nproc)}"

for package_name in frenet_planner mpc_controller; do
  package_dir="$REPO_ROOT/src/$package_name"
  build_dir="$package_dir/build"
  cmake -S "$package_dir" -B "$build_dir" -DCMAKE_BUILD_TYPE=Release
  cmake --build "$build_dir" --parallel "$BUILD_JOBS"
done

echo "[build_planning_control] done"
