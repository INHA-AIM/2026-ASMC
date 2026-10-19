"""Geometry selection can run without a model, family solver or ROS node."""
import hashlib
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

import rospy
import yaml
from std_msgs.msg import Header
from asmc_msgs.msg import SensorFrameTiming
from asmc_camera.bev.calibration import load_calibration_snapshot
from asmc_camera.bev.calibration import calibration_id
from asmc_camera.runtime.ground_projection import select_ground_projection


class GroundProjectionTest(unittest.TestCase):
    def setUp(self):
        self.path = Path(__file__).resolve().parents[2] / 'config/cameras_v3.yaml'
        _, self.cameras = load_calibration_snapshot(self.path)
        self.headers, self.raw = {}, {}
        for view, camera in self.cameras.items():
            h = Header(seq=45, stamp=rospy.Time(10), frame_id='camera_'+view)
            self.headers[view] = h
            self.raw[view] = SensorFrameTiming(sensor_topic=camera.image_topic,
                sensor_frame_id=h.frame_id, published_stamp=h.stamp,
                sensor_wire_sequence=h.seq, sensor_wire_sequence_valid=True)
        self.calls = []
        def wait(raw, frame, timeout):
            self.calls.append((frame, timeout))
            return NS(quaternion=(0., 0., 0., 1.), source_ns=10**10,
                      bracket_ms=20., bridge_session_id='bridge'), 'ok'
        self.timeline = NS(wait_attitude=wait)

    def select(self, cameras=None, config=None):
        return select_ground_projection(cameras or self.cameras, self.headers, self.raw,
                                        self.timeline, config or {'max_wait_ms': 10.})

    def test_identity_and_single_camera_use_do_not_require_family(self):
        for cameras in (self.cameras, {'front': self.cameras['front']}):
            selected = self.select(cameras)
            self.assertEqual(selected.reason, 'corrected')
            self.assertEqual(set(selected.cameras), set(cameras))
            for view, camera in cameras.items():
                self.assertEqual(selected.cameras[view].ground_plane.coefficients, camera.ground_plane.coefficients)
            with self.assertRaises(TypeError):
                selected.cameras['other'] = self.cameras['front']
        self.assertTrue(all(0 <= timeout <= 10 for _, timeout in self.calls))

    def test_missing_wrong_identity_and_timeline_failure_are_all_or_nominal(self):
        self.raw['right'].sensor_wire_sequence += 1
        self.assertEqual(self.select().reason, 'source_identity_mismatch')
        self.assertFalse(self.select().cameras)
        del self.raw['right']
        self.assertEqual(self.select().reason, 'missing_source_timing')
        self.timeline.wait_attitude = lambda *args: (None, 'imu_bracket_missing')
        self.assertEqual(self.select().reason, 'imu_bracket_missing')

    def test_excessive_source_skew_or_tilt_falls_back_without_mutating_calibration(self):
        nominal = self.cameras['front'].ground_plane
        def wait(raw, *args):
            ns = 10**10 + (36000000 if raw.sensor_frame_id == 'camera_right' else 0)
            return NS(quaternion=(0., 0., 0., 1.), source_ns=ns,
                      bracket_ms=20., bridge_session_id='bridge'), 'ok'
        self.timeline.wait_attitude = wait
        self.assertEqual(self.select().reason, 'camera_source_skew')
        self.assertFalse(self.select().cameras)
        self.timeline.wait_attitude = lambda *args: (NS(quaternion=(.5, 0., 0., .8660254)), 'ok')
        self.assertEqual(self.select().reason, 'attitude_out_of_range')
        self.assertEqual(self.cameras['front'].ground_plane, nominal)

    def test_cpp_consumer_manifest_matches_canonical_python_calibration_ids(self):
        manifest = yaml.safe_load((self.path.parent/'bev/projection_context_v1.yaml').read_text())
        self.assertEqual(manifest['calibration_sha256'], hashlib.sha256(self.path.read_bytes()).hexdigest())
        self.assertEqual(manifest['calibration_ids'],
                         {v: calibration_id(c) for v, c in self.cameras.items()})


if __name__ == '__main__':
    unittest.main()
