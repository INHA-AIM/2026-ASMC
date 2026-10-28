#!/usr/bin/env python3
"""Check real lane-node geometry hashes on the CI group's isolated ROS master."""
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import rospy
import yaml

from asmc_camera.nodes.lane_vectorizer_node import LaneVectorizerNode


def main():
    deadline = time.monotonic() + 15
    while True:
        try:
            rospy.get_master().getPid()
            break
        except Exception:
            if time.monotonic() >= deadline:
                raise
            time.sleep(.1)
    rospy.init_node('lane_geometry_contract_fixture', disable_signals=True)
    root = Path(__file__).resolve().parents[2]
    config = root / 'src/asmc_camera/config'
    for name in ('cameras_v3.yaml', 'bev/lane_fitting.yaml', 'camera/lane_input.yaml'):
        for key, value in yaml.safe_load((config / name).read_text()).items():
            rospy.set_param('~' + key, value)
    rospy.set_param('~managed', False)
    rospy.set_param('~enable_boundary_family_candidate', True)
    rospy.set_param('~enable_independent_boundary_pipeline', False)
    rospy.set_param('~imu_motion', yaml.safe_load((config / 'bev/imu_motion.yaml').read_text()))
    launch = ET.parse(root / 'src/asmc_planner/launch/autonomy_no_bridges_v2.launch').getroot()
    expected = next(arg.attrib['default'] for arg in launch.findall('arg')
                    if arg.attrib['name'] == 'expected_config_sha256')
    results = []
    try:
        for backend in ('python', 'native', 'auto'):
            rospy.set_param('~association_backend', backend)
            node = LaneVectorizerNode()
            try:
                assert node.boundary_family_config_sha256 == expected, (backend, node.boundary_family_config_sha256, expected)
                results.append(dict(backend=node.association_backend,
                                    family_hash=node.boundary_family_config_sha256,
                                    boundary_hash=node.boundary_config_sha256))
            finally:
                node._shutdown()
        assert len({result['boundary_hash'] for result in results}) == 1
        # Negative control: a real geometry gate must still change both hashes.
        rospy.set_param('~max_lateral_residual_m', rospy.get_param('~max_lateral_residual_m') + .01)
        node = LaneVectorizerNode()
        try:
            assert node.boundary_family_config_sha256 != expected
            assert node.boundary_config_sha256 != results[0]['boundary_hash']
        finally:
            node._shutdown()
    finally:
        rospy.signal_shutdown('lane geometry contract fixture complete')
    print(json.dumps(dict(passed=True, backends=results, geometry_negative_control=True)))


if __name__ == '__main__':
    main()
