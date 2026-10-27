#!/usr/bin/env python3
"""Compare the selected node with package C++ sources without starting ROS.

This checks asmc_lidar sources only. Configuration IDs, dependency builds,
already-running processes and sensor quality require separate checks.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys


def source_sha256(package):
    package = Path(package)
    paths = [package / 'CMakeLists.txt', package / 'package.xml',
             package / 'cmake/build_info.hpp.in']
    for folder in ('include', 'src'):
        paths.extend(p for p in (package / folder).rglob('*')
                     if p.is_file() and p.suffix in ('.cpp', '.hpp', '.h'))
    digest = hashlib.sha256()
    for path in sorted(paths, key=lambda p: p.relative_to(package).as_posix()):
        digest.update((path.relative_to(package).as_posix() + '\n' +
                       hashlib.sha256(path.read_bytes()).hexdigest() + '\n').encode())
    return digest.hexdigest()


def inspect_build(executable, package):
    # Old nodes may ignore unknown flags and start ROS. With no live master and
    # a bounded subprocess, even those binaries cannot advertise/control a car.
    import os
    env = dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:1')
    result = subprocess.run([str(executable), '--build-info'], env=env,
                            capture_output=True, text=True, timeout=5, check=True)
    info = json.loads(result.stdout)
    if info.get('schema') != 'asmc.lidar.build.v1':
        raise ValueError('node does not provide the supported build identity; rebuild')
    info['selected_executable'] = str(Path(executable).resolve())
    info['selected_package'] = str(Path(package).resolve())
    info['expected_source_sha256'] = source_sha256(package)
    info['source_matches'] = info['package_source_sha256'] == info['expected_source_sha256']
    return info


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--executable', type=Path)
    parser.add_argument('--package-dir', type=Path)
    args = parser.parse_args()
    try:
        if args.package_dir is None:
            import rospkg
            args.package_dir = Path(rospkg.RosPack().get_path('asmc_lidar'))
        if args.executable is None:
            import roslib.packages
            nodes = roslib.packages.find_node('asmc_lidar', 'lidar_pipeline_node') or []
            if len(nodes) != 1:
                raise ValueError('expected one selected executable, found: ' + repr(nodes))
            args.executable = Path(nodes[0])
        info = inspect_build(args.executable, args.package_dir)
        print(json.dumps(info, indent=2, sort_keys=True))
        return 0 if info['source_matches'] else 1
    except (OSError, ValueError, KeyError, ImportError, subprocess.SubprocessError) as error:
        print('LiDAR build verification failed: ' + str(error), file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
