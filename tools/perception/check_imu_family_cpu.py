#!/usr/bin/env python3
"""Noetic CPU-only IMU/family/serialization regressions; no optimizer or GPU."""
import argparse
import json
from pathlib import Path
import sys
import unittest


def main(args):
    root = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(root / 'tools/perception'))
    patterns = [
        ('src/asmc_camera/test/bev', 'test_shared_lane_family.py'),
        ('src/asmc_camera/test/bev', 'test_joint_family_fit.py'),
        ('src/asmc_camera/test/bev', 'test_family_fit_policy.py'),
        ('src/asmc_camera/test/runtime', 'test_imu_lane_adapter.py'),
        ('src/asmc_camera/test/runtime', 'test_lane_latest_work.py'),
        ('src/asmc_camera/test/runtime', 'test_sensor_timing.py'),
        ('tools/perception', 'test_imu_runtime_metrics.py'),
    ]
    suite = unittest.TestSuite()
    selected = {}
    for folder, pattern in patterns:
        path = root / folder
        part = unittest.TestLoader().discover(str(path), pattern=pattern, top_level_dir=str(path))
        selected[pattern] = part.countTestCases()
        if not selected[pattern]: raise RuntimeError('required regression pattern matched no tests: ' + pattern)
        suite.addTests(part)
    suite.addTests(unittest.TestLoader().loadTestsFromName(
        'test_camera_runtime_measurements.RuntimeMeasurementsTest.test_lane_goal_uses_configured15_instead_of_legacy10'))
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    args.output.write_text(json.dumps(dict(tests=result.testsRun, failures=len(result.failures),
        errors=len(result.errors), skipped=len(result.skipped), passed=result.wasSuccessful(),
        required_patterns=selected, gpu_used=False, live_master_used=False), indent=2) + '\n')
    return not result.wasSuccessful()


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    raise SystemExit(main(p.parse_args()))
