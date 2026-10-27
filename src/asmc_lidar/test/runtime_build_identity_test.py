#!/usr/bin/env python3
"""Detect stale/mixed source and binaries; configuration edits are independent."""
import importlib.util
from pathlib import Path
import shutil
import sys
import tempfile

executable, package = map(Path, sys.argv[1:])
spec = importlib.util.spec_from_file_location('build_check', package/'tools/check_runtime_build.py')
check = importlib.util.module_from_spec(spec)
spec.loader.exec_module(check)
assert check.inspect_build(executable, package)['source_matches']
with tempfile.TemporaryDirectory(prefix='lidar-build-identity-') as tmp:
    copy = Path(tmp)
    for name in ('src', 'include', 'cmake'):
        shutil.copytree(package/name, copy/name)
    for name in ('CMakeLists.txt', 'package.xml'):
        shutil.copy2(package/name, copy/name)
    assert check.inspect_build(executable, copy)['source_matches']
    (copy/'config').mkdir()
    (copy/'config/lidar_pipeline.yaml').write_text('fusion: {views: front}\n')
    assert check.inspect_build(executable, copy)['source_matches']
    source = copy/'src/fusion/multi_camera_fusion.cpp'
    source.write_text(source.read_text() + '\n// stale binary fixture\n')
    assert not check.inspect_build(executable, copy)['source_matches']
print('PASS: matching, relocated, configuration-only and stale source builds')
