#!/usr/bin/env bash
# ROS/catkin과 분리된 UDP-only Planning/Control 빌드.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_JOBS="${BUILD_JOBS:-$(nproc)}"

for package_name in frenet_planner mpc_controller; do
  package_dir="$REPO_ROOT/src/$package_name"
  build_dir="$package_dir/build"

  # CMake caches absolute source paths. If the repository was moved or copied,
  # discard the generated build tree before configuring it at the new path.
  cache_file="$build_dir/CMakeCache.txt"
  if [[ -f "$cache_file" ]]; then
    cached_source="$(sed -n 's/^CMAKE_HOME_DIRECTORY:INTERNAL=//p' "$cache_file")"
    if [[ -n "$cached_source" && "$cached_source" != "$package_dir" ]]; then
      echo "[build_planning_control] removing stale $package_name build cache"
      cmake -E remove_directory "$build_dir"
    fi
  fi

  cmake -S "$package_dir" -B "$build_dir" -DCMAKE_BUILD_TYPE=Release
  cmake --build "$build_dir" --parallel "$BUILD_JOBS"
done

echo "[build_planning_control] done"
